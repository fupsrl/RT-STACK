/** @file spark.c  Ignition command validation and event scheduling. */
#include "spark.h"

#include "board_output_config.h"
#include "engine_config.h"
#include <math.h>
#include <stddef.h>

typedef enum
{
  COIL_IDLE = 0,
  COIL_CHARGING
} coil_phase_t;

typedef struct
{
  spark_command_t command;
  bool command_changed;
  volatile coil_phase_t phase;

  /* Immutable from COIL_CHARGING until the coil is released. */
  float active_spark_angle_deg;
  volatile uint32_t hard_off_tick;
  uint32_t active_sync_epoch;
} spark_cylinder_state_t;

static spark_cylinder_state_t cylinder_state[ENGINE_CYLINDER_COUNT];
static volatile uint32_t fault_flags;
/* Bit 0 is the latch; upper bits are a generation tag.  A generation-tagged
 * compare/exchange prevents rearm from clearing an emergency which arrived
 * during the rearm sequence (NMI cannot be masked by critical_enter()). */
static volatile uint32_t emergency_state;
static bool runtime_latched;
static float previous_angle_deg;
static float previous_cycle_deg;
static uint32_t previous_sync_epoch;
static bool configuration_valid;

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

static bool spark_configuration_is_valid(void)
{
  uint32_t used_outputs = 0U;
  uint32_t used_ids = 0U;
  uint8_t i;

  if ((ENGINE_CYLINDER_COUNT == 0U) ||
      (ENGINE_CYLINDER_COUNT > BOARD_IGNITION_OUTPUT_COUNT))
  {
    return false;
  }

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    const engine_cylinder_config_t *c = &engine_cylinders[i];
    const ignition_calibration_t *k = &c->ignition;
    uint32_t id_bit;
    uint32_t output_bit;

    if ((c->cylinder_id == 0U) || (c->cylinder_id > 32U) ||
        (c->ignition_output == 0U) ||
        (c->ignition_output > BOARD_IGNITION_OUTPUT_COUNT) ||
        !isfinite(c->firing_tdc_deg) || (c->firing_tdc_deg < 0.0f) ||
        (c->firing_tdc_deg >= ENGINE_CYCLE_DEG) ||
        !isfinite(k->default_advance_deg) ||
        !isfinite(k->minimum_advance_deg) ||
        !isfinite(k->maximum_advance_deg) ||
        (k->minimum_advance_deg > k->default_advance_deg) ||
        (k->default_advance_deg > k->maximum_advance_deg) ||
        (k->minimum_advance_deg < -ENGINE_CYCLE_DEG) ||
        (k->maximum_advance_deg > ENGINE_CYCLE_DEG) ||
        (k->minimum_dwell_us == 0U) ||
        (k->minimum_dwell_us > k->default_dwell_us) ||
        (k->default_dwell_us > k->maximum_dwell_us) ||
        (k->maximum_dwell_us > k->hard_dwell_limit_us))
    {
      return false;
    }

    id_bit = 1UL << (c->cylinder_id - 1U);
    output_bit = 1UL << (c->ignition_output - 1U);
    if (((used_ids & id_bit) != 0U) || ((used_outputs & output_bit) != 0U))
    {
      /* One scheduler state may not own the same coil as another state. */
      return false;
    }
    used_ids |= id_bit;
    used_outputs |= output_bit;
  }
  return true;
}

static void coil_write_by_cylinder(uint8_t cylinder_index, bool charge)
{
  uint8_t output_index =
      (uint8_t)(engine_cylinders[cylinder_index].ignition_output - 1U);
  board_ignition_write(output_index, charge);
}

static void release_coil(uint8_t cylinder_index, bool deadline_fault)
{
  coil_write_by_cylinder(cylinder_index, false);
  cylinder_state[cylinder_index].phase = COIL_IDLE;
  if (deadline_fault)
  {
    cylinder_state[cylinder_index].command_changed = true;
    fault_set(SPARK_FAULT_HARD_DEADLINE);
  }
}

static void force_all_coils_off(bool invalidate_runtime)
{
  uint8_t i;

  for (i = 0U; i < BOARD_IGNITION_OUTPUT_COUNT; ++i)
  {
    board_ignition_write(i, false);
  }
  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    cylinder_state[i].phase = COIL_IDLE;
    cylinder_state[i].command_changed = true;
  }
  if (invalidate_runtime)
  {
    runtime_latched = false;
  }
}

static bool any_coil_charging(void)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    if (cylinder_state[i].phase == COIL_CHARGING)
    {
      return true;
    }
  }
  return false;
}

void spark_init(void)
{
  uint8_t i;

  __atomic_store_n(&emergency_state, 0U, __ATOMIC_RELEASE);
  __atomic_store_n(&fault_flags, SPARK_FAULT_NONE, __ATOMIC_RELAXED);
  runtime_latched = false;
  previous_angle_deg = 0.0f;
  previous_cycle_deg = ENGINE_CYCLE_DEG;
  previous_sync_epoch = 0U;
  configuration_valid = spark_configuration_is_valid();

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    cylinder_state[i].command.enabled = false;
    cylinder_state[i].command.advance_deg =
        engine_cylinders[i].ignition.default_advance_deg;
    cylinder_state[i].command.dwell_us =
        engine_cylinders[i].ignition.default_dwell_us;
    cylinder_state[i].command_changed = true;
    cylinder_state[i].phase = COIL_IDLE;
    cylinder_state[i].active_spark_angle_deg = 0.0f;
    cylinder_state[i].hard_off_tick = 0U;
    cylinder_state[i].active_sync_epoch = 0U;
  }
  force_all_coils_off(true);

  if (!configuration_valid)
  {
    fault_set(SPARK_FAULT_CONFIGURATION);
    emergency_latch();
  }
}

spark_result_t spark_set_command(uint8_t cylinder_id,
                                 const spark_command_t *command)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  const ignition_calibration_t *limits;
  uint32_t primask;

  if (!configuration_valid)
  {
    return SPARK_RESULT_CONFIGURATION_ERROR;
  }
  if (emergency_is_latched())
  {
    return SPARK_RESULT_EMERGENCY_LATCHED;
  }
  if (index < 0)
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return SPARK_RESULT_UNKNOWN_CYLINDER;
  }
  if (command == NULL)
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return SPARK_RESULT_COMMAND_OUT_OF_RANGE;
  }

  limits = &engine_cylinders[(uint8_t)index].ignition;
  if (!isfinite(command->advance_deg))
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return SPARK_RESULT_NONFINITE_COMMAND;
  }
  if (command->enabled &&
      ((command->advance_deg < limits->minimum_advance_deg) ||
       (command->advance_deg > limits->maximum_advance_deg) ||
       (command->dwell_us < limits->minimum_dwell_us) ||
       (command->dwell_us > limits->maximum_dwell_us)))
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return SPARK_RESULT_COMMAND_OUT_OF_RANGE;
  }

  primask = critical_enter();
  if (!command->enabled &&
      (cylinder_state[(uint8_t)index].phase == COIL_CHARGING))
  {
    release_coil((uint8_t)index, false);
  }
  if ((cylinder_state[(uint8_t)index].command.enabled != command->enabled) ||
      (cylinder_state[(uint8_t)index].command.advance_deg != command->advance_deg) ||
      (cylinder_state[(uint8_t)index].command.dwell_us != command->dwell_us))
  {
    cylinder_state[(uint8_t)index].command = *command;
    cylinder_state[(uint8_t)index].command_changed = true;
  }
  critical_exit(primask);
  return SPARK_RESULT_OK;
}

spark_result_t spark_load_defaults(uint8_t cylinder_id, bool enable)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  spark_command_t command;

  if (index < 0)
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return SPARK_RESULT_UNKNOWN_CYLINDER;
  }
  command.enabled = enable;
  command.advance_deg =
      engine_cylinders[(uint8_t)index].ignition.default_advance_deg;
  command.dwell_us = engine_cylinders[(uint8_t)index].ignition.default_dwell_us;
  return spark_set_command(cylinder_id, &command);
}

bool spark_phase_sync_required(void)
{
#if (IGNITION_REQUIRES_PHASE_SYNC != 0U)
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

void spark_service_deadlines(uint32_t now_tick)
{
  uint8_t i;

  if (!any_coil_charging())
  {
    return;
  }

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    uint32_t primask = critical_enter();
    if ((cylinder_state[i].phase == COIL_CHARGING) &&
        actuator_deadline_reached(now_tick, cylinder_state[i].hard_off_tick))
    {
      /* ISR only moves an active state toward OFF. */
      release_coil(i, true);
    }
    critical_exit(primask);
  }
}

static bool begin_dwell(uint8_t index,
                        const actuator_runtime_snapshot_t *runtime,
                        float spark_angle_deg)
{
  const ignition_calibration_t *limits = &engine_cylinders[index].ignition;
  uint32_t requested_limit_us;
  uint32_t deadline;
  uint32_t primask;

  requested_limit_us = cylinder_state[index].command.dwell_us;
  if (requested_limit_us <= (UINT32_MAX - IGNITION_DEADLINE_GUARD_US))
  {
    requested_limit_us += IGNITION_DEADLINE_GUARD_US;
  }
  if (requested_limit_us > limits->hard_dwell_limit_us)
  {
    requested_limit_us = limits->hard_dwell_limit_us;
  }
  if (!actuator_deadline_from_us(runtime->now_tick, requested_limit_us,
                                 runtime->tick_hz, &deadline))
  {
    fault_set(SPARK_FAULT_TIMING_UNREACHABLE);
    return false;
  }

  primask = critical_enter();
  if (emergency_is_latched() ||
      (cylinder_state[index].phase != COIL_IDLE) ||
      !cylinder_state[index].command.enabled)
  {
    critical_exit(primask);
    return false;
  }

  /* Publish all immutable event data before energizing the output.  The phase
   * is published last, so the deadline IRQ cannot release a half-built event. */
  cylinder_state[index].active_spark_angle_deg = spark_angle_deg;
  cylinder_state[index].hard_off_tick = deadline;
  cylinder_state[index].active_sync_epoch = runtime->sync_epoch;
  coil_write_by_cylinder(index, true);
  __DMB();
  cylinder_state[index].phase = COIL_CHARGING;

  /* NMI can preempt even this IRQ-masked section.  If it latched emergency
   * after the first check, undo both the output and the published state before
   * returning to foreground.  If NMI arrives after this check, NMI itself
   * performs the shutdown. */
  if (emergency_is_latched())
  {
    coil_write_by_cylinder(index, false);
    cylinder_state[index].phase = COIL_IDLE;
    cylinder_state[index].command_changed = true;
    critical_exit(primask);
    return false;
  }
  critical_exit(primask);
  return true;
}

void spark_service(const actuator_runtime_snapshot_t *runtime)
{
  bool require_phase = spark_phase_sync_required();
  float angle_step;
  uint8_t i;

  if (runtime != NULL)
  {
    spark_service_deadlines(runtime->now_tick);
  }

  if (!configuration_valid || emergency_is_latched() ||
      !actuator_snapshot_valid(runtime, require_phase))
  {
    if ((runtime != NULL) && runtime->outputs_enabled)
    {
      fault_set(SPARK_FAULT_BAD_RUNTIME);
    }
    /* Initialization already establishes the physical OFF state.  Once an
     * invalid/disabled runtime has been handled, subsequent superloop passes
     * stay cheap so trigger decoding retains maximum foreground bandwidth. */
    if (runtime_latched || any_coil_charging())
    {
      force_all_coils_off(true);
    }
    return;
  }

  if (!runtime_latched || (runtime->sync_epoch != previous_sync_epoch) ||
      (runtime->cycle_deg != previous_cycle_deg))
  {
    force_all_coils_off(false);
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
    fault_set(SPARK_FAULT_ANGLE_JUMP);
    force_all_coils_off(false);
    previous_angle_deg = runtime->angle_deg;
    return;
  }

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    spark_cylinder_state_t *state = &cylinder_state[i];
    const engine_cylinder_config_t *config = &engine_cylinders[i];
    bool skip_new_event;
    coil_phase_t phase;
    float spark_angle_deg;
    uint32_t primask;

    /* Take command_changed and phase as one foreground snapshot.  Otherwise
     * the deadline IRQ could release a coil between these reads and allow the
     * same event to start again. */
    primask = critical_enter();
    skip_new_event = state->command_changed;
    state->command_changed = false;
    phase = state->phase;
    critical_exit(primask);
    if (!state->command.enabled)
    {
      continue;
    }

    if (phase == COIL_CHARGING)
    {
      if ((state->active_sync_epoch != runtime->sync_epoch) ||
          actuator_angle_crossed(state->active_spark_angle_deg,
                                   previous_angle_deg, runtime->angle_deg))
      {
        primask = critical_enter();
        if (state->phase == COIL_CHARGING)
        {
          release_coil(i, false);
        }
        critical_exit(primask);
      }
      continue;
    }
    if (skip_new_event)
    {
      continue;
    }

    spark_angle_deg = actuator_wrap_angle(config->firing_tdc_deg -
                                           state->command.advance_deg,
                                           runtime->cycle_deg);
    {
      float dwell_angle_deg = runtime->rpm * 0.000006f *
                              (float)state->command.dwell_us;
      float dwell_start_deg;
      float remaining_deg;

      if (!isfinite(dwell_angle_deg) ||
          (dwell_angle_deg >= runtime->cycle_deg))
      {
        fault_set(SPARK_FAULT_TIMING_UNREACHABLE);
        continue;
      }
      dwell_start_deg = actuator_wrap_angle(spark_angle_deg - dwell_angle_deg,
                                             runtime->cycle_deg);
      remaining_deg = actuator_forward_angle(runtime->angle_deg, spark_angle_deg,
                                               runtime->cycle_deg);

      /* Never begin charging if this foreground interval already crossed the
       * intended release point.  Waiting one cycle is safer than a late spark. */
      if (actuator_angle_crossed(spark_angle_deg, previous_angle_deg,
                                 runtime->angle_deg))
      {
        fault_set(SPARK_FAULT_MISSED_EVENT);
      }
      else if (actuator_angle_crossed(dwell_start_deg, previous_angle_deg,
                                      runtime->angle_deg) ||
               ((remaining_deg > 0.0f) &&
                (remaining_deg <= dwell_angle_deg)))
      {
        (void)begin_dwell(i, runtime, spark_angle_deg);
      }
    }
  }

  previous_angle_deg = runtime->angle_deg;
}

void spark_disable_cylinder(uint8_t cylinder_id)
{
  int8_t index = engine_cylinder_index(cylinder_id);
  uint32_t primask;

  if (index < 0)
  {
    return;
  }
  primask = critical_enter();
  cylinder_state[(uint8_t)index].command.enabled = false;
  cylinder_state[(uint8_t)index].command_changed = true;
  if (cylinder_state[(uint8_t)index].phase == COIL_CHARGING)
  {
    release_coil((uint8_t)index, false);
  }
  critical_exit(primask);
}

void spark_disable_mask(uint32_t cylinder_id_mask)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    uint8_t id = engine_cylinders[i].cylinder_id;
    if ((cylinder_id_mask & (1UL << (id - 1U))) != 0U)
    {
      spark_disable_cylinder(id);
    }
  }
}

void spark_emergency_off(void)
{
  emergency_latch();
  fault_set(SPARK_FAULT_EMERGENCY);
  force_all_coils_off(true);
}

bool spark_rearm_after_emergency(void)
{
  uint32_t observed;
  uint32_t desired;

  if (!configuration_valid)
  {
    return false;
  }
  observed = __atomic_load_n(&emergency_state, __ATOMIC_ACQUIRE);
  force_all_coils_off(true);
  desired = observed & ~EMERGENCY_LATCH_BIT;

  /* Failure means a newer emergency generation arrived while outputs were
   * being checked/off.  Leave that newer latch intact. */
  return __atomic_compare_exchange_n(&emergency_state, &observed, desired,
                                      false, __ATOMIC_ACQ_REL,
                                      __ATOMIC_ACQUIRE);
}

uint32_t spark_faults(void)
{
  return __atomic_load_n(&fault_flags, __ATOMIC_RELAXED);
}

void spark_clear_faults(uint32_t fault_mask)
{
  (void)__atomic_fetch_and(&fault_flags, ~fault_mask, __ATOMIC_RELAXED);
}

void spark_update(uint8_t cylinder_id, float advance_deg, float dwell_ms)
{
  spark_command_t command;
  float dwell_us;

  if (cylinder_id == 0U)
  {
    return;
  }
  dwell_us = dwell_ms * 1000.0f;
  if (!isfinite(dwell_ms) || (dwell_ms < 0.0f) ||
      (dwell_us > (float)UINT32_MAX))
  {
    fault_set(SPARK_FAULT_BAD_COMMAND);
    return;
  }
  command.enabled = true;
  command.advance_deg = advance_deg;
  command.dwell_us = (uint32_t)(dwell_us + 0.5f);
  (void)spark_set_command(cylinder_id, &command);
}

void spark_off(int16_t cylinder_id_mask)
{
  spark_disable_mask((uint32_t)(uint16_t)cylinder_id_mask);
}
