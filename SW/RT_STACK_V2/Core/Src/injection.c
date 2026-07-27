/** @file injection.c  Injector command validation, scheduling and pair guard. */
#include "injection.h"
#include "injection_internal.h"

#include "board_output_config.h"
#include "engine_config.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

#define DEBUG_OUTPUT_OWNER  0xFEU
#define NO_CYLINDER_OWNER   0xFFU

typedef enum
{
  INJECTOR_PAIR_OFF = 0,
  INJECTOR_PAIR_BOOST,
  INJECTOR_PAIR_HOLD
} injector_pair_phase_t;

typedef struct
{
  injection_command_t command;
  bool command_changed;
} injection_cylinder_state_t;

typedef struct
{
  volatile injector_pair_phase_t phase;
  volatile uint8_t owner_cylinder_index;

  /* Immutable from start until INJECTOR_PAIR_OFF. */
  uint8_t active_output_index;
  uint16_t active_hold_dac_code;
  volatile uint32_t boost_deadline_tick;
  volatile uint32_t off_deadline_tick;
  uint32_t active_sync_epoch;
} injector_pair_state_t;

typedef enum
{
  INJECTION_BEGIN_OK = 0,
  INJECTION_BEGIN_INVALID,
  INJECTION_BEGIN_EMERGENCY,
  INJECTION_BEGIN_BUSY,
  INJECTION_BEGIN_HARDWARE
} injection_begin_result_t;

typedef struct
{
  volatile bool active;
  uint8_t cylinder_index;
  uint8_t output_index;
  uint8_t pair_index;
  uint32_t period_us;
  uint32_t period_ticks;
  uint32_t pulse_width_us;
  uint32_t next_pulse_tick;
  uint32_t tick_hz;
  uint32_t pulse_count;
  uint32_t skipped_period_count;
} injection_debug_runtime_t;

static injection_cylinder_state_t cylinder_state[ENGINE_CYLINDER_COUNT];
static injector_pair_state_t pair_state[BOARD_INJECTOR_PAIR_COUNT];
static volatile uint32_t fault_flags;
static volatile uint32_t emergency_state;
static bool configuration_valid;
static bool runtime_latched;
static float previous_angle_deg;
static float previous_cycle_deg;
static uint32_t previous_sync_epoch;
static injection_debug_runtime_t debug_runtime;

#define EMERGENCY_LATCH_BIT  1UL

static void fault_set(uint32_t flags)
{
  (void)__atomic_fetch_or(&fault_flags, flags, __ATOMIC_RELAXED);
}

static bool emergency_is_latched(void)
{
  return (__atomic_load_n(&emergency_state, __ATOMIC_ACQUIRE) &
          EMERGENCY_LATCH_BIT) != 0U;
}

static void emergency_latch(void)
{
  uint32_t expected = __atomic_load_n(&emergency_state, __ATOMIC_RELAXED);
  uint32_t desired;

  do
  {
    desired = ((expected + 2U) & ~EMERGENCY_LATCH_BIT) |
              EMERGENCY_LATCH_BIT;
  }
  while (!__atomic_compare_exchange_n(&emergency_state, &expected, desired,
                                       true, __ATOMIC_RELEASE,
                                       __ATOMIC_RELAXED));
}

static uint32_t critical_enter(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return primask;
}

static void critical_exit(uint32_t primask)
{
  __DMB();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static bool injection_configuration_is_valid(void)
{
  uint32_t used_outputs = 0U;
  uint32_t used_ids = 0U;
  uint8_t i;

  if ((ENGINE_CYLINDER_COUNT == 0U) ||
      (ENGINE_CYLINDER_COUNT > BOARD_INJECTOR_OUTPUT_COUNT))
  {
    return false;
  }

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    const engine_cylinder_config_t *c = &engine_cylinders[i];
    const injection_calibration_t *k = &c->injection;
    uint32_t id_bit;
    uint32_t output_bit;

    if ((c->cylinder_id == 0U) || (c->cylinder_id > 32U) ||
        (c->injector_output == 0U) ||
        (c->injector_output > BOARD_INJECTOR_OUTPUT_COUNT) ||
        !isfinite(c->firing_tdc_deg) || (c->firing_tdc_deg < 0.0f) ||
        (c->firing_tdc_deg >= ENGINE_CYCLE_DEG) ||
        !isfinite(k->default_start_btdc_deg) ||
        !isfinite(k->minimum_start_btdc_deg) ||
        !isfinite(k->maximum_start_btdc_deg) ||
        (k->minimum_start_btdc_deg > k->default_start_btdc_deg) ||
        (k->default_start_btdc_deg > k->maximum_start_btdc_deg) ||
        (k->minimum_start_btdc_deg < 0.0f) ||
        (k->maximum_start_btdc_deg > ENGINE_CYCLE_DEG) ||
        (k->minimum_duration_us == 0U) ||
        (k->minimum_duration_us > k->default_duration_us) ||
        (k->default_duration_us > k->maximum_duration_us) ||
        (k->boost_dac_code > 4095U) || (k->hold_dac_code > 4095U))
    {
      return false;
    }

    id_bit = 1UL << (c->cylinder_id - 1U);
    output_bit = 1UL << (c->injector_output - 1U);
    if (((used_ids & id_bit) != 0U) || ((used_outputs & output_bit) != 0U))
    {
      return false;
    }
    used_ids |= id_bit;
    used_outputs |= output_bit;

    if (board_injector_outputs[c->injector_output - 1U].pair_index >=
        BOARD_INJECTOR_PAIR_COUNT)
    {
      return false;
    }
  }
  return (INJECTOR_IDLE_DAC_CODE <= 4095U);
}

#if ((ENGINE_INJECTOR_TEST_API_ENABLED != 0U) && \
     (INJECTION_CURRENT_CALIBRATION_VALID != 0U))
static int8_t cylinder_index_for_injector_output(uint8_t injector_output)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    if (engine_cylinders[i].injector_output == injector_output)
    {
      return (int8_t)i;
    }
  }
  return -1;
}
#endif

static void pair_gpio_off(uint8_t pair_index)
{
  const board_injector_pair_t *hw = &board_injector_pairs[pair_index];
  board_gpio_write(hw->boost_port, hw->boost_pin, false);
  board_gpio_write(hw->select_port, hw->select_pin, false);
}

/* Normal/IRQ shutdown also restores HAL's comparator state.  The emergency
 * variant below deliberately touches only peripheral registers and GPIO. */
static void pair_off_normal(uint8_t pair_index)
{
  const board_injector_pair_t *hw = &board_injector_pairs[pair_index];

  /* The register clear is the safety action.  HAL_COMP_Stop is then used to
   * restore the handle state, but a RESET/locked HAL handle cannot prevent the
   * physical comparator from being disabled. */
  if ((hw->comparator != NULL) && (hw->comparator->Instance != NULL))
  {
    __HAL_COMP_DISABLE(hw->comparator);
    if (HAL_COMP_Stop(hw->comparator) != HAL_OK)
    {
      fault_set(INJECTION_FAULT_HARDWARE);
    }
  }
  else
  {
    fault_set(INJECTION_FAULT_HARDWARE);
  }
  pair_gpio_off(pair_index);
  if ((hw->threshold_dac != NULL) && (hw->threshold_dac->Instance != NULL))
  {
    if (HAL_DAC_SetValue(hw->threshold_dac, hw->dac_channel,
                        DAC_ALIGN_12B_R, INJECTOR_IDLE_DAC_CODE) != HAL_OK)
    {
      fault_set(INJECTION_FAULT_HARDWARE);
    }
  }
  else
  {
    fault_set(INJECTION_FAULT_HARDWARE);
  }
  __DMB();
  pair_state[pair_index].owner_cylinder_index = NO_CYLINDER_OWNER;
  pair_state[pair_index].phase = INJECTOR_PAIR_OFF;
}

static void pair_off_emergency(uint8_t pair_index)
{
  const board_injector_pair_t *hw = &board_injector_pairs[pair_index];

  /* Error_Handler/NMI may run before MX_COMPx_Init assigned Instance. */
  if ((hw->comparator != NULL) && (hw->comparator->Instance != NULL))
  {
    __HAL_COMP_DISABLE(hw->comparator);
  }
  pair_gpio_off(pair_index);
  pair_state[pair_index].owner_cylinder_index = NO_CYLINDER_OWNER;
  pair_state[pair_index].phase = INJECTOR_PAIR_OFF;
}

static void force_all_pairs_off(bool invalidate_runtime, bool emergency_path)
{
  uint8_t i;

  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    if (emergency_path)
    {
      pair_off_emergency(i);
    }
    else
    {
      pair_off_normal(i);
    }
  }
  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    cylinder_state[i].command_changed = true;
  }
  if (invalidate_runtime)
  {
    runtime_latched = false;
  }
}

static bool any_pair_active(void)
{
  uint8_t i;

  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    if (pair_state[i].phase != INJECTOR_PAIR_OFF)
    {
      return true;
    }
  }
  return false;
}

void injection_init(void)
{
  uint8_t i;

  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  debug_runtime.cylinder_index = 0U;
  debug_runtime.output_index = 0U;
  debug_runtime.pair_index = 0U;
  debug_runtime.period_us = 0U;
  debug_runtime.period_ticks = 0U;
  debug_runtime.pulse_width_us = 0U;
  debug_runtime.next_pulse_tick = 0U;
  debug_runtime.tick_hz = 0U;
  debug_runtime.pulse_count = 0U;
  debug_runtime.skipped_period_count = 0U;
  __atomic_store_n(&emergency_state, 0U, __ATOMIC_RELEASE);
  __atomic_store_n(&fault_flags, INJECTION_FAULT_NONE, __ATOMIC_RELAXED);
  runtime_latched = false;
  previous_angle_deg = 0.0f;
  previous_cycle_deg = ENGINE_CYCLE_DEG;
  previous_sync_epoch = 0U;
  configuration_valid = injection_configuration_is_valid();

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    cylinder_state[i].command.enabled = false;
    cylinder_state[i].command.start_btdc_deg =
        engine_cylinders[i].injection.default_start_btdc_deg;
    cylinder_state[i].command.duration_us =
        engine_cylinders[i].injection.default_duration_us;
    cylinder_state[i].command_changed = true;
  }
  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    const board_injector_pair_t *hw = &board_injector_pairs[i];

    pair_state[i].phase = INJECTOR_PAIR_OFF;
    pair_state[i].owner_cylinder_index = NO_CYLINDER_OWNER;
    pair_state[i].active_output_index = 0U;
    pair_state[i].active_hold_dac_code = INJECTOR_IDLE_DAC_CODE;
    pair_state[i].boost_deadline_tick = 0U;
    pair_state[i].off_deadline_tick = 0U;
    pair_state[i].active_sync_epoch = 0U;
    pair_off_emergency(i);

    if ((hw->comparator == NULL) || (hw->comparator->Instance == NULL) ||
        (hw->threshold_dac == NULL) || (hw->threshold_dac->Instance == NULL) ||
        (HAL_DAC_SetValue(hw->threshold_dac,
                          hw->dac_channel,
                          DAC_ALIGN_12B_R, INJECTOR_IDLE_DAC_CODE) != HAL_OK) ||
        (HAL_DAC_Start(hw->threshold_dac, hw->dac_channel) != HAL_OK))
    {
      fault_set(INJECTION_FAULT_HARDWARE);
      configuration_valid = false;
    }
  }

  if (!configuration_valid)
  {
    fault_set(INJECTION_FAULT_CONFIGURATION);
    emergency_latch();
    force_all_pairs_off(true, true);
  }
}

injection_result_t injection_set_command(uint8_t cylinder_id,
                                         const injection_command_t *command)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  const injection_calibration_t *limits;
  uint32_t primask;

  if (!configuration_valid)
  {
    return INJECTION_RESULT_CONFIGURATION_ERROR;
  }
  if (emergency_is_latched())
  {
    return INJECTION_RESULT_EMERGENCY_LATCHED;
  }
  if (index < 0)
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_RESULT_UNKNOWN_CYLINDER;
  }
  if (command == NULL)
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_RESULT_COMMAND_OUT_OF_RANGE;
  }
#if (INJECTION_CURRENT_CALIBRATION_VALID == 0U)
  if (command->enabled)
  {
    /* Never interpret placeholder DAC counts as verified current limits. */
    fault_set(INJECTION_FAULT_CONFIGURATION);
    return INJECTION_RESULT_CONFIGURATION_ERROR;
  }
#endif
  limits = &engine_cylinders[(uint8_t)index].injection;
  if (!isfinite(command->start_btdc_deg))
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_RESULT_NONFINITE_COMMAND;
  }
  if (command->enabled &&
      ((command->start_btdc_deg < limits->minimum_start_btdc_deg) ||
       (command->start_btdc_deg > limits->maximum_start_btdc_deg) ||
       (command->duration_us < limits->minimum_duration_us) ||
       (command->duration_us > limits->maximum_duration_us)))
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_RESULT_COMMAND_OUT_OF_RANGE;
  }

  primask = critical_enter();
  if (!command->enabled)
  {
    uint8_t output_index = (uint8_t)
        (engine_cylinders[(uint8_t)index].injector_output - 1U);
    uint8_t pair_index = board_injector_outputs[output_index].pair_index;
    if (pair_state[pair_index].owner_cylinder_index == (uint8_t)index)
    {
      pair_off_normal(pair_index);
    }
  }
  if ((cylinder_state[(uint8_t)index].command.enabled != command->enabled) ||
      (cylinder_state[(uint8_t)index].command.start_btdc_deg !=
       command->start_btdc_deg) ||
      (cylinder_state[(uint8_t)index].command.duration_us != command->duration_us))
  {
    cylinder_state[(uint8_t)index].command = *command;
    cylinder_state[(uint8_t)index].command_changed = true;
  }
  critical_exit(primask);
  return INJECTION_RESULT_OK;
}

injection_result_t injection_load_defaults(uint8_t cylinder_id, bool enable)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  injection_command_t command;

  if (index < 0)
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_RESULT_UNKNOWN_CYLINDER;
  }
  command.enabled = enable;
  command.start_btdc_deg =
      engine_cylinders[(uint8_t)index].injection.default_start_btdc_deg;
  command.duration_us =
      engine_cylinders[(uint8_t)index].injection.default_duration_us;
  return injection_set_command(cylinder_id, &command);
}

bool injection_phase_sync_required(void)
{
#if (INJECTION_REQUIRES_PHASE_SYNC != 0U)
  bool required = false;
  uint32_t primask = critical_enter();

  for (uint8_t i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    if (cylinder_state[i].command.enabled)
    {
      required = true;
      break;
    }
  }
  critical_exit(primask);
  return required;
#else
  return false;
#endif
}

void injection_service_deadlines(uint32_t now_tick)
{
  uint8_t i;

  if (!any_pair_active())
  {
    return;
  }

  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    uint32_t primask = critical_enter();
    injector_pair_phase_t phase = pair_state[i].phase;

    if (phase == INJECTOR_PAIR_OFF)
    {
      critical_exit(primask);
      continue;
    }
    if (actuator_deadline_reached(now_tick, pair_state[i].off_deadline_tick))
    {
      pair_off_normal(i);
      critical_exit(primask);
      continue;
    }
    if ((phase == INJECTOR_PAIR_BOOST) &&
        actuator_deadline_reached(now_tick,
                                  pair_state[i].boost_deadline_tick))
    {
      const board_injector_pair_t *hw = &board_injector_pairs[i];
      board_gpio_write(hw->boost_port, hw->boost_pin, false);
      if (HAL_DAC_SetValue(hw->threshold_dac, hw->dac_channel,
                           DAC_ALIGN_12B_R,
                           pair_state[i].active_hold_dac_code) != HAL_OK)
      {
        fault_set(INJECTION_FAULT_HARDWARE);
        pair_off_normal(i);
      }
      else
      {
        pair_state[i].phase = INJECTOR_PAIR_HOLD;
        __DMB();
        if (emergency_is_latched())
        {
          /* NMI may have forced OFF between the pre-transition read and the
           * HOLD publication.  Never let this stale foreground/IRQ work
           * resurrect the pair state. */
          pair_off_emergency(i);
        }
      }
    }
    critical_exit(primask);
  }
}

static injection_begin_result_t begin_injection_output(
    uint8_t owner,
    uint8_t output_index,
    const injection_calibration_t *drive,
    uint32_t duration_us,
    uint32_t now_tick,
    uint32_t tick_hz,
    uint32_t sync_epoch,
    bool require_enabled_command)
{
  const board_injector_output_t *output =
      &board_injector_outputs[output_index];
  uint8_t pair_index = output->pair_index;
  const board_injector_pair_t *hw = &board_injector_pairs[pair_index];
  uint32_t boost_duration_us = drive->boost_duration_us;
  uint32_t boost_deadline = now_tick;
  uint32_t off_deadline;
  uint32_t primask;
  HAL_StatusTypeDef status;

  if (boost_duration_us > duration_us)
  {
    boost_duration_us = duration_us;
  }
  if (!actuator_deadline_from_us(now_tick, duration_us, tick_hz,
                                 &off_deadline) ||
      ((boost_duration_us != 0U) &&
       !actuator_deadline_from_us(now_tick, boost_duration_us, tick_hz,
                                  &boost_deadline)))
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return INJECTION_BEGIN_INVALID;
  }

  primask = critical_enter();
  if (emergency_is_latched())
  {
    critical_exit(primask);
    return INJECTION_BEGIN_EMERGENCY;
  }
  if (pair_state[pair_index].phase != INJECTOR_PAIR_OFF)
  {
    critical_exit(primask);
    return INJECTION_BEGIN_BUSY;
  }
  if (require_enabled_command &&
      ((owner >= ENGINE_CYLINDER_COUNT) ||
       !cylinder_state[owner].command.enabled))
  {
    critical_exit(primask);
    return INJECTION_BEGIN_INVALID;
  }

  pair_state[pair_index].owner_cylinder_index = owner;
  pair_state[pair_index].active_output_index = output_index;
  pair_state[pair_index].active_hold_dac_code = drive->hold_dac_code;
  pair_state[pair_index].boost_deadline_tick = boost_deadline;
  pair_state[pair_index].off_deadline_tick = off_deadline;
  pair_state[pair_index].active_sync_epoch = sync_epoch;

  if ((hw->comparator == NULL) || (hw->comparator->Instance == NULL) ||
      (hw->threshold_dac == NULL) || (hw->threshold_dac->Instance == NULL))
  {
    fault_set(INJECTION_FAULT_HARDWARE);
    pair_off_emergency(pair_index);
    critical_exit(primask);
    return INJECTION_BEGIN_HARDWARE;
  }

  status = HAL_DAC_SetValue(hw->threshold_dac, hw->dac_channel,
                            DAC_ALIGN_12B_R,
                            (boost_duration_us != 0U) ? drive->boost_dac_code
                                                      : drive->hold_dac_code);
  board_gpio_write(hw->select_port, hw->select_pin,
                   output->select_state == GPIO_PIN_SET);
  board_gpio_write(hw->boost_port, hw->boost_pin, boost_duration_us != 0U);
  if (status == HAL_OK)
  {
    status = HAL_COMP_Start(hw->comparator);
  }

  /* An NMI may have asserted the emergency latch while HAL was starting the
   * comparator. Recheck before and after publishing the active pair. */
  if (status != HAL_OK)
  {
    fault_set(INJECTION_FAULT_HARDWARE);
    pair_off_normal(pair_index);
    critical_exit(primask);
    return INJECTION_BEGIN_HARDWARE;
  }
  if (emergency_is_latched())
  {
    pair_off_emergency(pair_index);
    critical_exit(primask);
    return INJECTION_BEGIN_EMERGENCY;
  }

  __DMB();
  pair_state[pair_index].phase = (boost_duration_us != 0U)
                               ? INJECTOR_PAIR_BOOST
                               : INJECTOR_PAIR_HOLD;
  __DMB();
  if (emergency_is_latched())
  {
    pair_off_emergency(pair_index);
    critical_exit(primask);
    return INJECTION_BEGIN_EMERGENCY;
  }
  critical_exit(primask);
  return INJECTION_BEGIN_OK;
}

static bool begin_injection(uint8_t cylinder_index,
                            const actuator_runtime_snapshot_t *runtime)
{
  const engine_cylinder_config_t *config = &engine_cylinders[cylinder_index];
  injection_begin_result_t result = begin_injection_output(
      cylinder_index,
      (uint8_t)(config->injector_output - 1U),
      &config->injection,
      cylinder_state[cylinder_index].command.duration_us,
      runtime->now_tick,
      runtime->tick_hz,
      runtime->sync_epoch,
      true);

  if (result == INJECTION_BEGIN_BUSY)
  {
    fault_set(INJECTION_FAULT_PAIR_BUSY);
  }
  return result == INJECTION_BEGIN_OK;
}

void injection_service(const actuator_runtime_snapshot_t *runtime)
{
  bool require_phase = injection_phase_sync_required();
  uint8_t candidate[BOARD_INJECTOR_PAIR_COUNT];
  bool collision[BOARD_INJECTOR_PAIR_COUNT];
  float angle_step;
  uint8_t i;

  if (runtime != NULL)
  {
    injection_service_deadlines(runtime->now_tick);
  }

  if (!configuration_valid)
  {
    force_all_pairs_off(true, true);
    return;
  }
  if (emergency_is_latched())
  {
    force_all_pairs_off(true, true);
    return;
  }
  if (injection_debug_active())
  {
    /* The controller owns stationary injector tests.  Never let the normal
     * angle scheduler compete for a shared driver while one is armed. */
    runtime_latched = false;
    return;
  }
  if (!actuator_snapshot_valid(runtime, require_phase))
  {
    if ((runtime != NULL) && runtime->outputs_enabled)
    {
      fault_set(INJECTION_FAULT_BAD_RUNTIME);
    }
    /* injection_init() already established OFF and idle DAC levels.  Perform
     * the expensive HAL shutdown only on the transition out of a valid
     * runtime (or if an active pair is observed), not on every disabled loop. */
    if (runtime_latched || any_pair_active())
    {
      force_all_pairs_off(true, false);
    }
    return;
  }

  if (!runtime_latched || (runtime->sync_epoch != previous_sync_epoch) ||
      (runtime->cycle_deg != previous_cycle_deg))
  {
    force_all_pairs_off(false, false);
    previous_angle_deg = runtime->angle_deg;
    previous_cycle_deg = runtime->cycle_deg;
    previous_sync_epoch = runtime->sync_epoch;
    runtime_latched = true;
    return;
  }

  angle_step = actuator_forward_angle(previous_angle_deg, runtime->angle_deg,
                                      runtime->cycle_deg);
  if (angle_step > ACTUATOR_MAX_ANGLE_STEP_DEG)
  {
    fault_set(INJECTION_FAULT_ANGLE_JUMP);
    force_all_pairs_off(false, false);
    previous_angle_deg = runtime->angle_deg;
    return;
  }

  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    candidate[i] = NO_CYLINDER_OWNER;
    collision[i] = false;
  }

  /* First collect all crossings, without touching hardware.  If two cylinder
   * events mapped to one shared driver occur in this sample, reject both.
   * This is deterministic and independent of firing-table iteration order. */
  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    injection_cylinder_state_t *state = &cylinder_state[i];
    const engine_cylinder_config_t *config = &engine_cylinders[i];
    bool skip_new_event = state->command_changed;
    float start_angle_deg;
    uint8_t output_index;
    uint8_t pair_index;

    state->command_changed = false;
    if (!state->command.enabled || skip_new_event)
    {
      continue;
    }

    output_index = (uint8_t)(config->injector_output - 1U);
    pair_index = board_injector_outputs[output_index].pair_index;
    start_angle_deg = actuator_wrap_angle(config->firing_tdc_deg -
                                           state->command.start_btdc_deg,
                                           runtime->cycle_deg);
    if (actuator_angle_crossed(start_angle_deg, previous_angle_deg,
                               runtime->angle_deg))
    {
      if (candidate[pair_index] == NO_CYLINDER_OWNER)
      {
        candidate[pair_index] = i;
      }
      else
      {
        candidate[pair_index] = NO_CYLINDER_OWNER;
        collision[pair_index] = true;
      }
    }
  }

  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    uint8_t selected = candidate[i];

    if (collision[i])
    {
      fault_set(INJECTION_FAULT_PAIR_COLLISION);
      continue;
    }
    if (selected == NO_CYLINDER_OWNER)
    {
      continue;
    }
    if (pair_state[i].phase != INJECTOR_PAIR_OFF)
    {
      fault_set(INJECTION_FAULT_PAIR_BUSY);
    }
    else if (!begin_injection(selected, runtime) &&
             !emergency_is_latched())
    {
      fault_set(INJECTION_FAULT_MISSED_EVENT);
    }
  }

  previous_angle_deg = runtime->angle_deg;
}

bool injection_debug_active(void)
{
  return __atomic_load_n(&debug_runtime.active, __ATOMIC_ACQUIRE);
}

void injection_debug_get_state(injection_debug_state_t *state)
{
  uint32_t primask;

  if (state == NULL)
  {
    return;
  }

  primask = critical_enter();
  state->active = injection_debug_active();
  state->injector_output = (debug_runtime.period_us != 0U) ?
                           (uint8_t)(debug_runtime.output_index + 1U) : 0U;
  state->period_us = debug_runtime.period_us;
  state->pulse_width_us = debug_runtime.pulse_width_us;
  state->pulse_count = debug_runtime.pulse_count;
  state->skipped_period_count = debug_runtime.skipped_period_count;
  critical_exit(primask);
}

injection_debug_result_t injection_debug_start(uint8_t injector_output,
                                               uint32_t period_us,
                                               uint32_t pulse_width_us,
                                               uint32_t now_tick,
                                               uint32_t tick_hz)
{
#if (ENGINE_INJECTOR_TEST_API_ENABLED == 0U)
  (void)injector_output;
  (void)period_us;
  (void)pulse_width_us;
  (void)now_tick;
  (void)tick_hz;
  return INJECTION_DEBUG_NOT_CONFIGURED;
#else
  if (!configuration_valid)
  {
    return INJECTION_DEBUG_NOT_CONFIGURED;
  }
#if (INJECTION_CURRENT_CALIBRATION_VALID == 0U)
  (void)injector_output;
  (void)period_us;
  (void)pulse_width_us;
  (void)now_tick;
  (void)tick_hz;
  return INJECTION_DEBUG_CALIBRATION_REQUIRED;
#else
  int8_t cylinder_index;
  const injection_calibration_t *drive;
  uint32_t period_deadline;
  uint32_t pulse_deadline;
  uint32_t period_ticks;
  uint8_t output_index;
  uint8_t pair_index;
  uint32_t emergency_snapshot;
  uint32_t primask;

  if ((injector_output == 0U) ||
      (injector_output > BOARD_INJECTOR_OUTPUT_COUNT))
  {
    return INJECTION_DEBUG_INVALID_OUTPUT;
  }

  cylinder_index = cylinder_index_for_injector_output(injector_output);
  if (cylinder_index < 0)
  {
    return INJECTION_DEBUG_UNMAPPED_OUTPUT;
  }
  drive = &engine_cylinders[(uint8_t)cylinder_index].injection;

  if ((period_us == 0U) ||
      ((uint64_t)period_us >
       ((uint64_t)ENGINE_INJECTOR_TEST_MAX_PERIOD_MS * 1000ULL)) ||
      (pulse_width_us < drive->minimum_duration_us) ||
      (pulse_width_us > drive->maximum_duration_us) ||
      (((uint64_t)pulse_width_us +
        (uint64_t)ENGINE_INJECTOR_TEST_MIN_OFF_US) > (uint64_t)period_us) ||
      (((uint64_t)pulse_width_us * 1000ULL) >
       ((uint64_t)period_us *
        (uint64_t)ENGINE_INJECTOR_TEST_MAX_DUTY_PERMILLE)) ||
      !actuator_deadline_from_us(now_tick, period_us, tick_hz,
                                 &period_deadline) ||
      !actuator_deadline_from_us(now_tick, pulse_width_us, tick_hz,
                                 &pulse_deadline))
  {
    return INJECTION_DEBUG_INVALID_TIMING;
  }
  (void)pulse_deadline;

  period_ticks = period_deadline - now_tick;
  output_index = (uint8_t)(injector_output - 1U);
  pair_index = board_injector_outputs[output_index].pair_index;

  primask = critical_enter();
  emergency_snapshot = __atomic_load_n(&emergency_state, __ATOMIC_ACQUIRE);
  if ((emergency_snapshot & EMERGENCY_LATCH_BIT) != 0U)
  {
    critical_exit(primask);
    return INJECTION_DEBUG_EMERGENCY_LATCHED;
  }
  if (injection_debug_active() || any_pair_active())
  {
    critical_exit(primask);
    return INJECTION_DEBUG_PAIR_BUSY;
  }

  debug_runtime.cylinder_index = (uint8_t)cylinder_index;
  debug_runtime.output_index = output_index;
  debug_runtime.pair_index = pair_index;
  debug_runtime.period_us = period_us;
  debug_runtime.period_ticks = period_ticks;
  debug_runtime.pulse_width_us = pulse_width_us;
  debug_runtime.next_pulse_tick = period_deadline;
  debug_runtime.tick_hz = tick_hz;
  debug_runtime.pulse_count = 0U;
  debug_runtime.skipped_period_count = 0U;
  runtime_latched = false;
  __atomic_store_n(&debug_runtime.active, true, __ATOMIC_RELEASE);
  __DMB();
  /* PRIMASK does not mask NMI. If an NMI emergency ran while the state was
   * being filled, its generation change wins over this arming attempt. */
  if ((__atomic_load_n(&emergency_state, __ATOMIC_ACQUIRE) !=
       emergency_snapshot) || emergency_is_latched())
  {
    __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
    critical_exit(primask);
    return INJECTION_DEBUG_EMERGENCY_LATCHED;
  }
  critical_exit(primask);
  return INJECTION_DEBUG_OK;
#endif /* INJECTION_CURRENT_CALIBRATION_VALID */
#endif /* ENGINE_INJECTOR_TEST_API_ENABLED */
}

injection_debug_result_t injection_debug_service(uint32_t now_tick)
{
  injection_begin_result_t begin_result;
  const injection_calibration_t *drive;
  uint32_t lateness;
  uint32_t skipped;
  uint32_t primask;

  if (!injection_debug_active())
  {
    return INJECTION_DEBUG_OK;
  }

  injection_service_deadlines(now_tick);
  if (emergency_is_latched())
  {
    __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
    return INJECTION_DEBUG_EMERGENCY_LATCHED;
  }
  if (!injection_debug_active())
  {
    return INJECTION_DEBUG_OK;
  }
  if (!actuator_deadline_reached(now_tick, debug_runtime.next_pulse_tick))
  {
    return INJECTION_DEBUG_OK;
  }

  lateness = now_tick - debug_runtime.next_pulse_tick;
  skipped = lateness / debug_runtime.period_ticks;
  drive = &engine_cylinders[debug_runtime.cylinder_index].injection;
  begin_result = begin_injection_output(
      DEBUG_OUTPUT_OWNER,
      debug_runtime.output_index,
      drive,
      debug_runtime.pulse_width_us,
      now_tick,
      debug_runtime.tick_hz,
      0U,
      false);

  if (begin_result == INJECTION_BEGIN_OK)
  {
    primask = critical_enter();
    if (emergency_is_latched() || !injection_debug_active())
    {
      critical_exit(primask);
      return INJECTION_DEBUG_EMERGENCY_LATCHED;
    }
    debug_runtime.pulse_count =
        (debug_runtime.pulse_count == UINT32_MAX) ? UINT32_MAX :
        debug_runtime.pulse_count + 1U;
    debug_runtime.skipped_period_count =
        (skipped > (UINT32_MAX - debug_runtime.skipped_period_count)) ?
        UINT32_MAX : debug_runtime.skipped_period_count + skipped;
    /* Deliberately schedule from now.  A late foreground pass skips missed
     * periods and can never emit a catch-up burst. */
    debug_runtime.next_pulse_tick = now_tick + debug_runtime.period_ticks;
    critical_exit(primask);
    return INJECTION_DEBUG_OK;
  }

  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  if (begin_result == INJECTION_BEGIN_BUSY)
  {
    fault_set(INJECTION_FAULT_PAIR_BUSY);
    return INJECTION_DEBUG_PAIR_BUSY;
  }
  if (begin_result == INJECTION_BEGIN_HARDWARE)
  {
    return INJECTION_DEBUG_HARDWARE_ERROR;
  }
  if (begin_result == INJECTION_BEGIN_EMERGENCY)
  {
    return INJECTION_DEBUG_EMERGENCY_LATCHED;
  }
  return INJECTION_DEBUG_INVALID_TIMING;
}

void injection_debug_stop(void)
{
  uint32_t primask = critical_enter();

  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  if ((debug_runtime.period_us != 0U) &&
      (debug_runtime.pair_index < BOARD_INJECTOR_PAIR_COUNT) &&
      (pair_state[debug_runtime.pair_index].owner_cylinder_index ==
       DEBUG_OUTPUT_OWNER))
  {
    pair_off_normal(debug_runtime.pair_index);
  }
  runtime_latched = false;
  critical_exit(primask);
}

void injection_debug_abort_isr(void)
{
  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  fault_set(INJECTION_FAULT_DEBUG_INTERLOCK | INJECTION_FAULT_EMERGENCY);
  emergency_latch();
  force_all_pairs_off(true, true);
}

void injection_disable_cylinder(uint8_t cylinder_id)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  uint32_t primask;
  uint8_t output_index;
  uint8_t pair_index;

  if (index < 0)
  {
    return;
  }
  output_index = (uint8_t)
      (engine_cylinders[(uint8_t)index].injector_output - 1U);
  pair_index = board_injector_outputs[output_index].pair_index;

  primask = critical_enter();
  cylinder_state[(uint8_t)index].command.enabled = false;
  cylinder_state[(uint8_t)index].command_changed = true;
  if (pair_state[pair_index].owner_cylinder_index == (uint8_t)index)
  {
    pair_off_normal(pair_index);
  }
  critical_exit(primask);
}

void injection_disable_mask(uint32_t cylinder_id_mask)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    uint8_t id = engine_cylinders[i].cylinder_id;
    if ((cylinder_id_mask & (1UL << (id - 1U))) != 0U)
    {
      injection_disable_cylinder(id);
    }
  }
}

void injection_emergency_off(void)
{
  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  emergency_latch();
  fault_set(INJECTION_FAULT_EMERGENCY);
  force_all_pairs_off(true, true);
}

bool injection_rearm_after_emergency(void)
{
  uint8_t i;
  uint32_t observed;
  uint32_t desired;

  if (!configuration_valid)
  {
    return false;
  }
  __atomic_store_n(&debug_runtime.active, false, __ATOMIC_RELEASE);
  observed = __atomic_load_n(&emergency_state, __ATOMIC_ACQUIRE);
  /* Restore HAL handle state after the direct-register NMI path. */
  for (i = 0U; i < BOARD_INJECTOR_PAIR_COUNT; ++i)
  {
    COMP_HandleTypeDef *comparator = board_injector_pairs[i].comparator;
    if ((comparator == NULL) || (comparator->Instance == NULL))
    {
      fault_set(INJECTION_FAULT_HARDWARE);
      return false;
    }
    __HAL_COMP_DISABLE(comparator);
    if (HAL_COMP_Stop(comparator) != HAL_OK)
    {
      fault_set(INJECTION_FAULT_HARDWARE);
      return false;
    }
  }
  force_all_pairs_off(true, false);
  desired = observed & ~EMERGENCY_LATCH_BIT;
  return __atomic_compare_exchange_n(&emergency_state, &observed, desired,
                                      false, __ATOMIC_ACQ_REL,
                                      __ATOMIC_ACQUIRE);
}

uint32_t injection_faults(void)
{
  return __atomic_load_n(&fault_flags, __ATOMIC_RELAXED);
}

void injection_clear_faults(uint32_t fault_mask)
{
  (void)__atomic_fetch_and(&fault_flags, ~fault_mask, __ATOMIC_RELAXED);
}

void injection_update(uint8_t cylinder_id, float start_btdc_deg,
                      float duration_ms)
{
  injection_command_t command;
  float duration_us;

  if (cylinder_id == 0U)
  {
    return;
  }
  duration_us = duration_ms * 1000.0f;
  if (!isfinite(duration_ms) || (duration_ms < 0.0f) ||
      (duration_us > (float)UINT32_MAX))
  {
    fault_set(INJECTION_FAULT_BAD_COMMAND);
    return;
  }
  command.enabled = true;
  command.start_btdc_deg = start_btdc_deg;
  command.duration_us = (uint32_t)(duration_us + 0.5f);
  (void)injection_set_command(cylinder_id, &command);
}

void injection_off(int16_t cylinder_id_mask)
{
  injection_disable_mask((uint32_t)(uint16_t)cylinder_id_mask);
}
