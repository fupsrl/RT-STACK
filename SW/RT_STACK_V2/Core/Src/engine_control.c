/** @file engine_control.c  Trigger-to-actuator state machine. */
#include "engine_control.h"

#include "actuator_runtime.h"
#include "actuator_safety.h"
#include "engine_config.h"
#include "injection.h"
#include "injection_internal.h"
#include "main.h"
#include "spark.h"
#include "trigger_capture_stm32.h"
#include "trigger_recorder.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define SPARK_FATAL_FAULTS                                                \
  (SPARK_FAULT_CONFIGURATION | SPARK_FAULT_BAD_RUNTIME |                 \
   SPARK_FAULT_ANGLE_JUMP | SPARK_FAULT_HARD_DEADLINE |                  \
   SPARK_FAULT_MISSED_EVENT | SPARK_FAULT_TIMING_UNREACHABLE |           \
   SPARK_FAULT_EMERGENCY)

#define INJECTION_FATAL_FAULTS                                            \
  (INJECTION_FAULT_CONFIGURATION | INJECTION_FAULT_BAD_RUNTIME |         \
   INJECTION_FAULT_ANGLE_JUMP | INJECTION_FAULT_PAIR_BUSY |              \
   INJECTION_FAULT_PAIR_COLLISION |                                      \
   INJECTION_FAULT_HARDWARE | INJECTION_FAULT_MISSED_EVENT |             \
   INJECTION_FAULT_EMERGENCY | INJECTION_FAULT_DEBUG_INTERLOCK)

static trigger_recorder_t recorder;
static trigger_decoder_t decoder;
static trigger_output_t decoder_output;
static engine_control_state_t control_state;
static const trigger_decoder_config_t *active_trigger_config;

/* Only capture/fault interrupt contexts write these values.  All relevant
 * IRQs use the same preemption priority; the foreground only reads them. */
static volatile uint32_t isr_fault_bits;
static volatile uint32_t capture_overrun_count[TRIGGER_CHANNEL_COUNT];

static uint32_t handled_dropped_count[TRIGGER_CHANNEL_COUNT];
static uint32_t handled_overrun_count[TRIGGER_CHANNEL_COUNT];
static uint32_t serviced_primary_wheel_epoch;
static uint32_t serviced_phase_wheel_epoch;
static bool serviced_phase_required;
static volatile bool initialized;
static volatile bool trigger_quiet_satisfied;
static volatile uint32_t last_trigger_activity_timestamp;
static volatile engine_injector_test_result_t injector_test_last_result;

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

static uint32_t saturating_add(uint32_t value, uint32_t increment)
{
  return (increment > (UINT32_MAX - value)) ? UINT32_MAX : value + increment;
}

static bool phase_is_required(void)
{
  return spark_phase_sync_required() || injection_phase_sync_required();
}

static void injector_test_set_last_result(
    engine_injector_test_result_t result)
{
  __atomic_store_n(&injector_test_last_result, result, __ATOMIC_RELEASE);
}

static engine_injector_test_result_t injector_test_map_result(
    injection_debug_result_t result)
{
  switch (result)
  {
    case INJECTION_DEBUG_OK:
      return ENGINE_INJECTOR_TEST_OK;
    case INJECTION_DEBUG_NOT_CONFIGURED:
      return ENGINE_INJECTOR_TEST_DISABLED;
    case INJECTION_DEBUG_CALIBRATION_REQUIRED:
      return ENGINE_INJECTOR_TEST_CALIBRATION_LOCKED;
    case INJECTION_DEBUG_INVALID_OUTPUT:
    case INJECTION_DEBUG_INVALID_TIMING:
      return ENGINE_INJECTOR_TEST_INVALID_ARGUMENT;
    case INJECTION_DEBUG_UNMAPPED_OUTPUT:
      return ENGINE_INJECTOR_TEST_UNMAPPED_OUTPUT;
    case INJECTION_DEBUG_EMERGENCY_LATCHED:
      return ENGINE_INJECTOR_TEST_FAULT_LATCHED;
    case INJECTION_DEBUG_PAIR_BUSY:
      return ENGINE_INJECTOR_TEST_PAIR_BUSY;
    case INJECTION_DEBUG_HARDWARE_ERROR:
    default:
      return ENGINE_INJECTOR_TEST_HARDWARE_ERROR;
  }
}

#if (ENGINE_INJECTOR_TEST_API_ENABLED != 0U)
static bool injector_test_quiet_time_elapsed(uint32_t now)
{
  uint64_t quiet_ticks;
  uint32_t last_activity;
  uint32_t primask;
  bool elapsed;

  primask = critical_enter();
  if (__atomic_load_n(&trigger_quiet_satisfied, __ATOMIC_ACQUIRE))
  {
    critical_exit(primask);
    return true;
  }
  quiet_ticks = (((uint64_t)control_state.timestamp_hz *
                  (uint64_t)ENGINE_INJECTOR_TEST_QUIET_TIME_MS) + 999ULL) /
                1000ULL;
  if ((quiet_ticks == 0ULL) || (quiet_ticks > (uint64_t)INT32_MAX))
  {
    critical_exit(primask);
    return false;
  }
  last_activity = __atomic_load_n(&last_trigger_activity_timestamp,
                                  __ATOMIC_ACQUIRE);
  elapsed = actuator_deadline_reached(
      now, last_activity + (uint32_t)quiet_ticks);
  if (elapsed)
  {
    /* Latch until the next edge. The critical section prevents a new edge's
     * false store from being overwritten by this older observation. */
    __atomic_store_n(&trigger_quiet_satisfied, true, __ATOMIC_RELEASE);
  }
  critical_exit(primask);
  return elapsed;
}
#endif

static bool trigger_config_is_actuator_compatible(
    const trigger_decoder_config_t *config,
    trigger_config_error_t config_error)
{
  if ((config_error != TRIGGER_CONFIG_OK) || (config == NULL))
  {
    return false;
  }

  const trigger_wheel_config_t *primary =
      &config->wheels[config->primary_wheel];
  return isfinite(config->engine_cycle_degrees) &&
         isfinite(primary->cycle_degrees) &&
         (fabsf(config->engine_cycle_degrees - ENGINE_CYCLE_DEG) <= 0.001f) &&
         (fabsf(primary->cycle_degrees - ENGINE_CRANK_CYCLE_DEG) <= 0.001f) &&
         (primary->angle_direction == 1);
}

static void latch_fault(uint32_t fault)
{
  control_state.latched_faults |= fault;
}

static void update_actuator_diagnostics(void)
{
  control_state.ignition_faults = spark_faults();
  control_state.injection_faults = injection_faults();

  if ((control_state.ignition_faults & SPARK_FATAL_FAULTS) != 0U)
  {
    latch_fault(ENGINE_FAULT_IGNITION);
  }
  if ((control_state.injection_faults & INJECTION_FATAL_FAULTS) != 0U)
  {
    latch_fault(ENGINE_FAULT_INJECTION);
  }
}

/** Service the mutually exclusive stationary injector-test mode.
 * Returns true when normal angle-based actuator scheduling must be skipped. */
static bool service_injector_test_mode(uint32_t now,
                                       engine_mode_t previous_mode)
{
  injection_debug_result_t debug_result = INJECTION_DEBUG_OK;
  bool test_active;

  if ((previous_mode != ENGINE_MODE_INJECTOR_TEST) &&
      !injection_debug_active())
  {
    return false;
  }

  test_active = injection_debug_active();
  /* Spark and the angle-driven injector scheduler stay explicitly outside
   * this mode. The injector deadline IRQ remains live and may only advance an
   * already-started test pulse toward OFF. */
  spark_service(NULL);

  if (test_active &&
      ((control_state.latched_faults != ENGINE_FAULT_NONE) ||
       control_state.outputs_requested))
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
    actuators_emergency_off();
  }
  else if (test_active &&
           (decoder_output.synced || !isfinite(decoder_output.rpm) ||
            (fabsf(decoder_output.rpm) > 0.001f) ||
            (trigger_recorder_pending(&recorder) != 0U)))
  {
    /* Normally the capture ISR has already performed this faster interlock. */
    injection_debug_abort_isr();
    spark_emergency_off();
    injector_test_set_last_result(
        ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY);
    latch_fault(ENGINE_FAULT_INJECTION);
  }
  else if (test_active)
  {
    debug_result = injection_debug_service(now);
    if (debug_result != INJECTION_DEBUG_OK)
    {
      injector_test_set_last_result(injector_test_map_result(debug_result));
      latch_fault(ENGINE_FAULT_INJECTION);
      actuators_emergency_off();
    }
  }

  update_actuator_diagnostics();
  test_active = injection_debug_active();
  if (control_state.latched_faults != ENGINE_FAULT_NONE)
  {
    if (test_active)
    {
      actuators_emergency_off();
      update_actuator_diagnostics();
    }
    control_state.mode = ENGINE_MODE_FAULT;
    control_state.outputs_enabled = false;
  }
  else if (test_active)
  {
    control_state.mode = ENGINE_MODE_INJECTOR_TEST;
    control_state.outputs_enabled = true;
  }
  else
  {
    control_state.mode = ENGINE_MODE_DISABLED;
    control_state.outputs_enabled = false;
  }
  return true;
}

static void load_default_commands(void)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    uint8_t cylinder_id = engine_cylinders[i].cylinder_id;

    if (spark_load_defaults(cylinder_id,
                            ENGINE_DEFAULT_IGNITION_ENABLED != 0U) !=
        SPARK_RESULT_OK)
    {
      latch_fault(ENGINE_FAULT_CONFIGURATION);
    }
    if (injection_load_defaults(cylinder_id,
                                ENGINE_DEFAULT_INJECTION_ENABLED != 0U) !=
        INJECTION_RESULT_OK)
    {
      latch_fault(ENGINE_FAULT_CONFIGURATION);
    }
  }
}

static void account_capture_losses(void)
{
  uint8_t channel;
  uint32_t dropped_total = 0U;
  uint32_t overrun_total = 0U;

  for (channel = 1U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    uint32_t dropped = trigger_recorder_dropped(&recorder, channel);
    uint32_t overrun = capture_overrun_count[channel];
    uint32_t dropped_delta = dropped - handled_dropped_count[channel];
    uint32_t overrun_delta = overrun - handled_overrun_count[channel];

    if (dropped_delta != 0U)
    {
      /* The next queued event's per-channel sequence reports this exact gap to
       * the decoder.  Do not count it twice here.  The engine fault is latched
       * immediately even if no later edge arrives. */
      handled_dropped_count[channel] = dropped;
      latch_fault(ENGINE_FAULT_CAPTURE_QUEUE);
    }
    if (overrun_delta != 0U)
    {
      trigger_decoder_note_event_loss(&decoder, channel, overrun_delta);
      handled_overrun_count[channel] = overrun;
      latch_fault(ENGINE_FAULT_CAPTURE_OVERRUN);
    }
    dropped_total = saturating_add(dropped_total, dropped);
    overrun_total = saturating_add(overrun_total, overrun);
  }

  control_state.capture_dropped_events = dropped_total;
  control_state.capture_overruns = overrun_total;
  latch_fault(__atomic_load_n(&isr_fault_bits, __ATOMIC_ACQUIRE));
}

bool engine_control_init(const trigger_decoder_config_t *trigger_config,
                         uint32_t timestamp_hz)
{
  trigger_config_error_t config_error;

  __atomic_store_n(&initialized, false, __ATOMIC_RELEASE);
  memset(&control_state, 0, sizeof(control_state));
  memset(&decoder_output, 0, sizeof(decoder_output));
  memset(handled_dropped_count, 0, sizeof(handled_dropped_count));
  memset(handled_overrun_count, 0, sizeof(handled_overrun_count));
  memset((void *)capture_overrun_count, 0, sizeof(capture_overrun_count));
  __atomic_store_n(&isr_fault_bits, 0U, __ATOMIC_RELEASE);
  /* Require one complete quiet interval after initialization too. This keeps
   * an already-rotating but not-yet-observed wheel from being treated as a
   * stationary engine during the capture-startup window. */
  __atomic_store_n(&trigger_quiet_satisfied, false, __ATOMIC_RELEASE);
  __atomic_store_n(&last_trigger_activity_timestamp,
                   trigger_capture_now(), __ATOMIC_RELEASE);
  injector_test_set_last_result(ENGINE_INJECTOR_TEST_OK);

  trigger_recorder_init(&recorder);
  active_trigger_config = trigger_config;
  config_error = trigger_decoder_context_init(
      &decoder, trigger_config, timestamp_hz);

  control_state.mode = ENGINE_MODE_DISABLED;
  control_state.timestamp_hz = timestamp_hz;
  control_state.trigger_config_error = config_error;
  control_state.outputs_requested = (ENGINE_OUTPUTS_ARM_AT_BOOT != 0U);

  spark_init();
  injection_init();
  load_default_commands();
  serviced_primary_wheel_epoch = 0U;
  serviced_phase_wheel_epoch = 0U;
  serviced_phase_required = phase_is_required();

  if (!trigger_config_is_actuator_compatible(trigger_config, config_error))
  {
    latch_fault(ENGINE_FAULT_CONFIGURATION);
  }
  update_actuator_diagnostics();

  if (control_state.latched_faults != ENGINE_FAULT_NONE)
  {
    control_state.mode = ENGINE_MODE_FAULT;
    actuators_emergency_off();
  }

  __atomic_store_n(&initialized, true, __ATOMIC_RELEASE);
  return (control_state.latched_faults == ENGINE_FAULT_NONE);
}

bool engine_control_record_trigger_isr(uint8_t channel,
                                       trigger_edge_t edge,
                                       uint32_t timestamp)
{
  bool recorded;

  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    return false;
  }

  __atomic_store_n(&last_trigger_activity_timestamp, timestamp,
                   __ATOMIC_RELEASE);
  __atomic_store_n(&trigger_quiet_satisfied, false, __ATOMIC_RELEASE);
  if (injection_debug_active())
  {
    /* A stationary test and a trigger edge are mutually exclusive.  This
     * path is ISR-safe and cuts power before the edge can reach the queue. */
    injection_debug_abort_isr();
    spark_emergency_off();
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
    (void)__atomic_fetch_or(&isr_fault_bits, ENGINE_FAULT_INJECTION,
                            __ATOMIC_RELEASE);
    return false;
  }

  recorded = trigger_recorder_record_isr(&recorder, channel, edge, timestamp);
  if (!recorded)
  {
    (void)__atomic_fetch_or(&isr_fault_bits, ENGINE_FAULT_CAPTURE_QUEUE,
                            __ATOMIC_RELEASE);
    actuators_emergency_off();
  }
  return recorded;
}

void engine_control_capture_overrun_isr(uint8_t channel)
{
  __atomic_store_n(&last_trigger_activity_timestamp, trigger_capture_now(),
                   __ATOMIC_RELEASE);
  __atomic_store_n(&trigger_quiet_satisfied, false, __ATOMIC_RELEASE);
  if (injection_debug_active())
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
  }
  if ((channel > 0U) && (channel < TRIGGER_CHANNEL_COUNT))
  {
    uint32_t count = capture_overrun_count[channel];
    if (count != UINT32_MAX)
    {
      capture_overrun_count[channel] = count + 1U;
    }
  }
  (void)__atomic_fetch_or(&isr_fault_bits, ENGINE_FAULT_CAPTURE_OVERRUN,
                          __ATOMIC_RELEASE);
  actuators_emergency_off();
}

void engine_control_deadline_isr(uint32_t now_timestamp)
{
  if (__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    spark_service_deadlines(now_timestamp);
    injection_service_deadlines(now_timestamp);
  }
}

void engine_control_emergency_fault_isr(uint32_t fault_bit)
{
  if (injection_debug_active())
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
  }
  (void)__atomic_fetch_or(&isr_fault_bits, fault_bit, __ATOMIC_RELEASE);
  actuators_emergency_off();
}

void engine_control_service(void)
{
  trigger_event_t event;
  actuator_runtime_snapshot_t runtime;
  engine_mode_t previous_mode;
  uint32_t previous_primary_wheel_epoch;
  uint32_t previous_phase_wheel_epoch;
  uint32_t processed_events = 0U;
  uint32_t now;
  uint32_t primary_wheel_epoch = 0U;
  uint32_t phase_wheel_epoch = 0U;
  bool epoch_snapshot_valid = false;
  bool require_phase;
  bool sync_ready;

  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    return;
  }

  previous_mode = control_state.mode;
  previous_primary_wheel_epoch = serviced_primary_wheel_epoch;
  previous_phase_wheel_epoch = serviced_phase_wheel_epoch;
  account_capture_losses();
  while ((processed_events < ENGINE_MAX_TRIGGER_EVENTS_PER_SERVICE) &&
         trigger_recorder_pop_oldest(&recorder, &event))
  {
    trigger_decoder_process_event(&decoder, &event);
    ++processed_events;
  }
  if (trigger_recorder_pending(&recorder) != 0U)
  {
    latch_fault(ENGINE_FAULT_CAPTURE_QUEUE);
    trigger_decoder_force_sync_loss(&decoder, TRIGGER_LOSS_FORCED);
    actuators_emergency_off();
  }

  /* Read now after the queue drain.  An edge arriving after the drain remains
   * queued for the next pass and is therefore never processed against an
   * older foreground timestamp. */
  now = trigger_capture_now();
#if (ENGINE_INJECTOR_TEST_API_ENABLED != 0U)
  (void)injector_test_quiet_time_elapsed(now);
#endif
  trigger_decoder_poll(&decoder, now, &decoder_output);

  control_state.timestamp = now;
  control_state.angle_deg = decoder_output.crank_angle_deg;
  control_state.rpm = decoder_output.rpm;
  control_state.crank_synced = decoder_output.synced;
  control_state.phase_synced = decoder_output.phase_known;
  control_state.sync_epoch = decoder_output.sync_epoch;
  control_state.trigger_latched_faults = decoder_output.latched_faults;
  if ((decoder_output.wheel_count > 0U) &&
      (decoder.config != NULL) &&
      (decoder.config->primary_wheel < decoder_output.wheel_count))
  {
    uint8_t primary = decoder.config->primary_wheel;
    control_state.last_trigger_loss =
        decoder_output.wheel[primary].last_loss_reason;
    primary_wheel_epoch = decoder_output.wheel[primary].sync_epoch;
    if ((decoder.config->phase_wheel != TRIGGER_NO_WHEEL) &&
        (decoder.config->phase_wheel < decoder_output.wheel_count))
    {
      phase_wheel_epoch =
          decoder_output.wheel[decoder.config->phase_wheel].sync_epoch;
    }
    epoch_snapshot_valid = true;
  }

  require_phase = phase_is_required();
  sync_ready = decoder_output.synced &&
               (!require_phase || decoder_output.phase_known);

  if ((previous_mode == ENGINE_MODE_RUNNING) &&
      ((!decoder_output.synced) ||
       (serviced_phase_required && !decoder_output.phase_known) ||
       (epoch_snapshot_valid &&
        ((primary_wheel_epoch != previous_primary_wheel_epoch) ||
         (serviced_phase_required &&
          (phase_wheel_epoch != previous_phase_wheel_epoch))))) &&
      control_state.outputs_requested &&
      (ENGINE_LATCH_FAULT_ON_SYNC_LOSS != 0U))
  {
    latch_fault(ENGINE_FAULT_TRIGGER_SYNC_LOST);
    actuators_emergency_off();
  }

  update_actuator_diagnostics();
  if (service_injector_test_mode(now, previous_mode))
  {
    if (epoch_snapshot_valid)
    {
      serviced_primary_wheel_epoch = primary_wheel_epoch;
      serviced_phase_wheel_epoch = phase_wheel_epoch;
    }
    serviced_phase_required = require_phase;
    return;
  }

  if (control_state.latched_faults != ENGINE_FAULT_NONE)
  {
    control_state.mode = ENGINE_MODE_FAULT;
  }
  else if (!control_state.outputs_requested)
  {
    control_state.mode = ENGINE_MODE_DISABLED;
  }
  else if (sync_ready)
  {
    control_state.mode = ENGINE_MODE_RUNNING;
  }
  else
  {
    control_state.mode = ENGINE_MODE_SYNCING;
  }

  control_state.outputs_enabled = (control_state.mode == ENGINE_MODE_RUNNING);
  runtime = (actuator_runtime_snapshot_t) {
    .angle_deg = control_state.angle_deg,
    .rpm = control_state.rpm,
    .cycle_deg = control_state.phase_synced ?
                 ENGINE_CYCLE_DEG : ENGINE_CRANK_CYCLE_DEG,
    .now_tick = now,
    .tick_hz = control_state.timestamp_hz,
    .sync_epoch = control_state.sync_epoch,
    .crank_synced = control_state.crank_synced,
    .phase_synced = control_state.phase_synced,
    .outputs_enabled = control_state.outputs_enabled,
  };

  spark_service(&runtime);
  injection_service(&runtime);
  update_actuator_diagnostics();

  if (epoch_snapshot_valid)
  {
    serviced_primary_wheel_epoch = primary_wheel_epoch;
    serviced_phase_wheel_epoch = phase_wheel_epoch;
  }
  serviced_phase_required = require_phase;

  /* A scheduler may discover an angle jump, missed event, pair collision, or
   * hard deadline while servicing this snapshot.  Shut every output down in
   * the same foreground pass. */
  if ((control_state.latched_faults != ENGINE_FAULT_NONE) &&
      (control_state.mode != ENGINE_MODE_FAULT))
  {
    control_state.mode = ENGINE_MODE_FAULT;
    control_state.outputs_enabled = false;
    actuators_emergency_off();
    update_actuator_diagnostics();
  }
}

bool engine_control_request_outputs(bool enable)
{
  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    return false;
  }

  if (enable)
  {
    if (injection_debug_active() ||
        (control_state.mode == ENGINE_MODE_INJECTOR_TEST))
    {
      injector_test_set_last_result(ENGINE_INJECTOR_TEST_ENGINE_BUSY);
      return false;
    }
    if (control_state.latched_faults != ENGINE_FAULT_NONE)
    {
      return false;
    }
    control_state.outputs_requested = true;
    control_state.mode = ENGINE_MODE_SYNCING;
    return true;
  }

  if (injection_debug_active() ||
      (control_state.mode == ENGINE_MODE_INJECTOR_TEST))
  {
    engine_control_stop_injector_test();
    return true;
  }
  control_state.outputs_requested = false;
  control_state.outputs_enabled = false;
  control_state.mode = (control_state.latched_faults == ENGINE_FAULT_NONE) ?
                       ENGINE_MODE_DISABLED : ENGINE_MODE_FAULT;
  spark_service(NULL);
  injection_service(NULL);
  return true;
}

engine_injector_test_result_t engine_control_start_injector_test(
    const engine_injector_test_config_t *config)
{
#if (ENGINE_INJECTOR_TEST_API_ENABLED == 0U)
  (void)config;
  injector_test_set_last_result(ENGINE_INJECTOR_TEST_DISABLED);
  return ENGINE_INJECTOR_TEST_DISABLED;
#else
  injection_debug_result_t debug_result;
  engine_injector_test_result_t result;
  uint32_t now;
  uint32_t pending_faults;
  uint32_t injection_fault_bits;
  uint32_t primask;

  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_NOT_INITIALIZED);
    return ENGINE_INJECTOR_TEST_NOT_INITIALIZED;
  }
  if ((config == NULL) || (config->period_ms == 0U) ||
      (config->period_ms > ENGINE_INJECTOR_TEST_MAX_PERIOD_MS) ||
      (config->period_ms > (UINT32_MAX / 1000U)) ||
      (config->pulse_width_us == 0U))
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_INVALID_ARGUMENT);
    return ENGINE_INJECTOR_TEST_INVALID_ARGUMENT;
  }
#if (INJECTION_CURRENT_CALIBRATION_VALID == 0U)
  injector_test_set_last_result(ENGINE_INJECTOR_TEST_CALIBRATION_LOCKED);
  return ENGINE_INJECTOR_TEST_CALIBRATION_LOCKED;
#endif

  update_actuator_diagnostics();
  if (control_state.latched_faults != ENGINE_FAULT_NONE)
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
    return ENGINE_INJECTOR_TEST_FAULT_LATCHED;
  }
  if (control_state.outputs_requested || control_state.outputs_enabled ||
      (control_state.mode != ENGINE_MODE_DISABLED) ||
      injection_debug_active())
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_ENGINE_BUSY);
    return ENGINE_INJECTOR_TEST_ENGINE_BUSY;
  }

  /* Complete any normal OFF transition before entering the short final
   * critical section that closes the trigger-edge arming race. */
  spark_service(NULL);
  injection_service(NULL);
  update_actuator_diagnostics();
  if (control_state.latched_faults != ENGINE_FAULT_NONE)
  {
    injector_test_set_last_result(ENGINE_INJECTOR_TEST_FAULT_LATCHED);
    return ENGINE_INJECTOR_TEST_FAULT_LATCHED;
  }

  now = trigger_capture_now();
  primask = critical_enter();
  if (control_state.outputs_requested || control_state.outputs_enabled ||
      (control_state.mode != ENGINE_MODE_DISABLED) ||
      injection_debug_active())
  {
    result = ENGINE_INJECTOR_TEST_ENGINE_BUSY;
  }
  else if (control_state.crank_synced ||
           !isfinite(control_state.rpm) ||
           (fabsf(control_state.rpm) > 0.001f) ||
           (trigger_recorder_pending(&recorder) != 0U) ||
           !injector_test_quiet_time_elapsed(now))
  {
    result = ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY;
  }
  else
  {
    debug_result = injection_debug_start(
        config->injector_output,
        config->period_ms * 1000U,
        config->pulse_width_us,
        now,
        control_state.timestamp_hz);
    result = injector_test_map_result(debug_result);
    pending_faults = __atomic_load_n(&isr_fault_bits, __ATOMIC_ACQUIRE);
    injection_fault_bits = injection_faults();
    latch_fault(pending_faults);
    control_state.injection_faults = injection_fault_bits;
    if ((injection_fault_bits & INJECTION_FATAL_FAULTS) != 0U)
    {
      latch_fault(ENGINE_FAULT_INJECTION);
    }
    if ((result == ENGINE_INJECTOR_TEST_OK) &&
        (!injection_debug_active() ||
         (control_state.latched_faults != ENGINE_FAULT_NONE)))
    {
      injection_debug_stop();
      latch_fault(ENGINE_FAULT_INJECTION);
      result = ENGINE_INJECTOR_TEST_FAULT_LATCHED;
    }
    if (result == ENGINE_INJECTOR_TEST_OK)
    {
      control_state.mode = ENGINE_MODE_INJECTOR_TEST;
      control_state.outputs_requested = false;
      control_state.outputs_enabled = true;
      control_state.timestamp = now;
      injector_test_set_last_result(ENGINE_INJECTOR_TEST_OK);
      __DMB();

      /* Publish success before the final NMI-visible check. If an emergency
       * occurs before this sample it is corrected below; if it occurs after
       * the sample, its FAULT_LATCHED store cannot be overwritten here. */
      pending_faults = __atomic_load_n(&isr_fault_bits, __ATOMIC_ACQUIRE);
      injection_fault_bits = injection_faults();
      if (!injection_debug_active() || (pending_faults != ENGINE_FAULT_NONE) ||
          ((injection_fault_bits & INJECTION_FATAL_FAULTS) != 0U))
      {
        if (injection_debug_active())
        {
          injection_debug_stop();
        }
        latch_fault(pending_faults | ENGINE_FAULT_INJECTION);
        control_state.injection_faults = injection_fault_bits;
        control_state.mode = ENGINE_MODE_FAULT;
        control_state.outputs_enabled = false;
        result = ENGINE_INJECTOR_TEST_FAULT_LATCHED;
        injector_test_set_last_result(result);
      }
    }
    else
    {
      if (control_state.latched_faults != ENGINE_FAULT_NONE)
      {
        control_state.mode = ENGINE_MODE_FAULT;
        control_state.outputs_enabled = false;
      }
    }
  }
  if ((control_state.mode != ENGINE_MODE_INJECTOR_TEST) &&
      (result != ENGINE_INJECTOR_TEST_OK))
  {
    /* Busy/stationary checks do not enter the low-level branch above. */
    injector_test_set_last_result(result);
  }
  critical_exit(primask);
  return result;
#endif /* ENGINE_INJECTOR_TEST_API_ENABLED */
}

void engine_control_stop_injector_test(void)
{
#if (ENGINE_INJECTOR_TEST_API_ENABLED == 0U)
  /* Keep an accidentally linked production call from affecting normal
   * ignition/injection state. */
  return;
#else
  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
  {
    return;
  }
  if (!injection_debug_active() &&
      (control_state.mode != ENGINE_MODE_INJECTOR_TEST))
  {
    return;
  }

  latch_fault(__atomic_load_n(&isr_fault_bits, __ATOMIC_ACQUIRE));
  update_actuator_diagnostics();
  injection_debug_stop();
  spark_service(NULL);
  injection_service(NULL);
  latch_fault(__atomic_load_n(&isr_fault_bits, __ATOMIC_ACQUIRE));
  update_actuator_diagnostics();
  control_state.outputs_requested = false;
  control_state.outputs_enabled = false;
  control_state.mode = (control_state.latched_faults == ENGINE_FAULT_NONE) ?
                       ENGINE_MODE_DISABLED : ENGINE_MODE_FAULT;
  injector_test_set_last_result(
      (control_state.latched_faults == ENGINE_FAULT_NONE) ?
      ENGINE_INJECTOR_TEST_OK : ENGINE_INJECTOR_TEST_FAULT_LATCHED);
#endif
}

void engine_control_get_injector_test_state(
    engine_injector_test_state_t *state)
{
  injection_debug_state_t injection_state;
  uint32_t primask;

  if (state == NULL)
  {
    return;
  }
  memset(&injection_state, 0, sizeof(injection_state));
  primask = critical_enter();
  injection_debug_get_state(&injection_state);
  state->active = injection_state.active;
  state->injector_output = injection_state.injector_output;
  state->period_ms = injection_state.period_us / 1000U;
  state->pulse_width_us = injection_state.pulse_width_us;
  state->pulse_count = injection_state.pulse_count;
  state->skipped_period_count = injection_state.skipped_period_count;
  state->last_result = __atomic_load_n(&injector_test_last_result,
                                       __ATOMIC_ACQUIRE);
  critical_exit(primask);
}

bool engine_control_clear_faults(void)
{
  uint32_t primask;
  trigger_config_error_t config_error;
  uint8_t channel;

  if (!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE) ||
      control_state.outputs_requested || injection_debug_active() ||
      (control_state.mode == ENGINE_MODE_INJECTOR_TEST))
  {
    return false;
  }

  primask = critical_enter();
  trigger_recorder_init(&recorder);
  config_error = trigger_decoder_context_init(
      &decoder, active_trigger_config, control_state.timestamp_hz);
  memset(&decoder_output, 0, sizeof(decoder_output));
  memset(handled_dropped_count, 0, sizeof(handled_dropped_count));
  memset(handled_overrun_count, 0, sizeof(handled_overrun_count));
  for (channel = 0U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    capture_overrun_count[channel] = 0U;
  }
  __atomic_store_n(&isr_fault_bits, 0U, __ATOMIC_RELEASE);
  critical_exit(primask);

  if (!trigger_config_is_actuator_compatible(active_trigger_config,
                                             config_error) ||
      !spark_rearm_after_emergency() ||
      !injection_rearm_after_emergency())
  {
    latch_fault(ENGINE_FAULT_CONFIGURATION);
    actuators_emergency_off();
    control_state.mode = ENGINE_MODE_FAULT;
    return false;
  }

  spark_clear_faults(UINT32_MAX);
  injection_clear_faults(UINT32_MAX);
  memset(&control_state, 0, sizeof(control_state));
  control_state.mode = ENGINE_MODE_DISABLED;
  control_state.timestamp_hz = decoder.timestamp_frequency_hz;
  control_state.trigger_config_error = TRIGGER_CONFIG_OK;
  serviced_primary_wheel_epoch = 0U;
  serviced_phase_wheel_epoch = 0U;
  serviced_phase_required = phase_is_required();
  injector_test_set_last_result(ENGINE_INJECTOR_TEST_OK);
  return true;
}

void engine_control_get_state(engine_control_state_t *state)
{
  uint32_t primask;

  if (state == NULL)
  {
    return;
  }
  primask = critical_enter();
  *state = control_state;
  critical_exit(primask);
}
