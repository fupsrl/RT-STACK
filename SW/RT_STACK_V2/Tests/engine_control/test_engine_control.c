/* Host integration tests for the recorder -> decoder -> engine-control seam.
 * The production recorder, decoder, and state machine are linked unchanged;
 * only the STM32 clock and actuator endpoints below are replaced by fakes. */
#include "engine_control.h"
#include "injection.h"
#include "injection_internal.h"
#include "spark.h"
#include "trigger_capture_stm32.h"
#include "trigger_recorder.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_TICK_HZ       1000000U
#define SHORT_PERIOD_TICKS   10000U

static unsigned checks_run;

static void fail(const char *expression, const char *file, int line)
{
  (void)fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expression);
  exit(EXIT_FAILURE);
}

#define CHECK(expression)                                                     \
  do                                                                          \
  {                                                                           \
    ++checks_run;                                                             \
    if (!(expression))                                                       \
    {                                                                         \
      fail(#expression, __FILE__, __LINE__);                                  \
    }                                                                         \
  } while (0)

typedef struct
{
  uint32_t now;
  uint32_t spark_fault_bits;
  uint32_t injection_fault_bits;
  bool spark_requires_phase;
  bool injection_requires_phase;
  unsigned spark_emergency_calls;
  unsigned injection_emergency_calls;
  unsigned spark_rearm_calls;
  unsigned injection_rearm_calls;
  unsigned spark_deadline_calls;
  unsigned injection_deadline_calls;
  unsigned spark_service_calls;
  unsigned injection_service_calls;
  unsigned injection_debug_start_calls;
  unsigned injection_debug_service_calls;
  unsigned injection_debug_stop_calls;
  unsigned injection_debug_abort_calls;
  bool spark_runtime_present;
  bool injection_runtime_present;
  actuator_runtime_snapshot_t spark_runtime;
  actuator_runtime_snapshot_t injection_runtime;
  injection_debug_result_t injection_debug_start_result;
  injection_debug_result_t injection_debug_service_result;
  injection_debug_state_t injection_debug_state;
  uint8_t injection_debug_start_output;
  uint32_t injection_debug_start_period_us;
  uint32_t injection_debug_start_pulse_width_us;
  uint32_t injection_debug_start_now;
  uint32_t injection_debug_start_tick_hz;
  uint32_t injection_debug_service_now;
  uint32_t injection_debug_next_pulse_tick;
  uint32_t injection_debug_period_ticks;
} fake_platform_t;

static fake_platform_t fake;

static void fake_reset(void)
{
  memset(&fake, 0, sizeof(fake));
}

/* -------------------------- Capture clock fake -------------------------- */

uint32_t trigger_capture_timestamp_hz(void)
{
  return TEST_TICK_HZ;
}

bool trigger_capture_stm32_init(const trigger_decoder_config_t *config)
{
  return config != NULL;
}

uint32_t trigger_capture_now(void)
{
  return fake.now;
}

void trigger_capture_stm32_timer_irq(void)
{
}

void trigger_capture_stm32_stop(void)
{
}

/* ----------------------------- Spark fake ------------------------------- */

void spark_init(void)
{
  fake.spark_fault_bits = SPARK_FAULT_NONE;
}

spark_result_t spark_load_defaults(uint8_t cylinder_id, bool enable)
{
  (void)cylinder_id;
  (void)enable;
  return SPARK_RESULT_OK;
}

bool spark_phase_sync_required(void)
{
  return fake.spark_requires_phase;
}

void spark_service(const actuator_runtime_snapshot_t *runtime)
{
  ++fake.spark_service_calls;
  fake.spark_runtime_present = runtime != NULL;
  if (runtime != NULL)
  {
    fake.spark_runtime = *runtime;
  }
}

void spark_service_deadlines(uint32_t now_tick)
{
  (void)now_tick;
  ++fake.spark_deadline_calls;
}

void spark_emergency_off(void)
{
  ++fake.spark_emergency_calls;
}

/* The converter behavior is covered by Tests/dcdc. This fake satisfies the
 * shared engine emergency-shutdown seam used by the integration test. */
void dcdc_control_emergency_off(void)
{
}

bool spark_rearm_after_emergency(void)
{
  ++fake.spark_rearm_calls;
  return true;
}

uint32_t spark_faults(void)
{
  return fake.spark_fault_bits;
}

void spark_clear_faults(uint32_t fault_mask)
{
  fake.spark_fault_bits &= ~fault_mask;
}

/* --------------------------- Injection fake ----------------------------- */

void injection_init(void)
{
  fake.injection_fault_bits = INJECTION_FAULT_NONE;
  memset(&fake.injection_debug_state, 0,
         sizeof(fake.injection_debug_state));
  fake.injection_debug_next_pulse_tick = 0U;
  fake.injection_debug_period_ticks = 0U;
}

injection_result_t injection_load_defaults(uint8_t cylinder_id, bool enable)
{
  (void)cylinder_id;
  (void)enable;
  return INJECTION_RESULT_OK;
}

bool injection_phase_sync_required(void)
{
  return fake.injection_requires_phase;
}

void injection_service(const actuator_runtime_snapshot_t *runtime)
{
  ++fake.injection_service_calls;
  fake.injection_runtime_present = runtime != NULL;
  if (runtime != NULL)
  {
    fake.injection_runtime = *runtime;
  }
}

void injection_service_deadlines(uint32_t now_tick)
{
  (void)now_tick;
  ++fake.injection_deadline_calls;
}

injection_debug_result_t injection_debug_start(uint8_t injector_output,
                                               uint32_t period_us,
                                               uint32_t pulse_width_us,
                                               uint32_t now_tick,
                                               uint32_t tick_hz)
{
  uint64_t period_ticks;

  ++fake.injection_debug_start_calls;
  fake.injection_debug_start_output = injector_output;
  fake.injection_debug_start_period_us = period_us;
  fake.injection_debug_start_pulse_width_us = pulse_width_us;
  fake.injection_debug_start_now = now_tick;
  fake.injection_debug_start_tick_hz = tick_hz;

  if (fake.injection_debug_start_result != INJECTION_DEBUG_OK)
  {
    return fake.injection_debug_start_result;
  }

  period_ticks = (((uint64_t)period_us * (uint64_t)tick_hz) + 999999ULL) /
                 1000000ULL;
  if ((period_ticks == 0ULL) || (period_ticks > (uint64_t)INT32_MAX))
  {
    return INJECTION_DEBUG_INVALID_TIMING;
  }

  fake.injection_debug_state.active = true;
  fake.injection_debug_state.injector_output = injector_output;
  fake.injection_debug_state.period_us = period_us;
  fake.injection_debug_state.pulse_width_us = pulse_width_us;
  fake.injection_debug_state.pulse_count = 0U;
  fake.injection_debug_state.skipped_period_count = 0U;
  fake.injection_debug_period_ticks = (uint32_t)period_ticks;
  fake.injection_debug_next_pulse_tick = now_tick + (uint32_t)period_ticks;
  return INJECTION_DEBUG_OK;
}

injection_debug_result_t injection_debug_service(uint32_t now_tick)
{
  ++fake.injection_debug_service_calls;
  fake.injection_debug_service_now = now_tick;

  if (fake.injection_debug_service_result != INJECTION_DEBUG_OK)
  {
    fake.injection_debug_state.active = false;
    return fake.injection_debug_service_result;
  }
  if (fake.injection_debug_state.active &&
      ((int32_t)(now_tick - fake.injection_debug_next_pulse_tick) >= 0))
  {
    uint32_t lateness = now_tick - fake.injection_debug_next_pulse_tick;
    uint32_t skipped = lateness / fake.injection_debug_period_ticks;

    ++fake.injection_debug_state.pulse_count;
    fake.injection_debug_state.skipped_period_count += skipped;
    fake.injection_debug_next_pulse_tick =
        now_tick + fake.injection_debug_period_ticks;
  }
  return INJECTION_DEBUG_OK;
}

void injection_debug_stop(void)
{
  ++fake.injection_debug_stop_calls;
  fake.injection_debug_state.active = false;
}

void injection_debug_abort_isr(void)
{
  ++fake.injection_debug_abort_calls;
  fake.injection_debug_state.active = false;
  fake.injection_fault_bits |= INJECTION_FAULT_DEBUG_INTERLOCK |
                               INJECTION_FAULT_EMERGENCY;
}

bool injection_debug_active(void)
{
  return fake.injection_debug_state.active;
}

void injection_debug_get_state(injection_debug_state_t *state)
{
  if (state != NULL)
  {
    *state = fake.injection_debug_state;
  }
}

void injection_emergency_off(void)
{
  ++fake.injection_emergency_calls;
  fake.injection_debug_state.active = false;
}

bool injection_rearm_after_emergency(void)
{
  ++fake.injection_rearm_calls;
  return true;
}

uint32_t injection_faults(void)
{
  return fake.injection_fault_bits;
}

void injection_clear_faults(uint32_t fault_mask)
{
  fake.injection_fault_bits &= ~fault_mask;
}

/* --------------------------- Test geometry ------------------------------ */

static const trigger_wheel_config_t crank_wheel =
{
  .name = "host crank 4-1",
  .input_channel = 1U,
  .active_edge = TRIGGER_EDGE_RISING,
  .type = TRIGGER_WHEEL_MISSING_TOOTH,
  .cycle_degrees = 360.0f,
  .angle_at_index0_deg = 0.0f,
  .angle_direction = +1,
  .interval_tolerance_permille = 200U,
  .speed_filter_permille = 1000U,
  .sync_confirmations = 1U,
  .minimum_interval_us = 100U,
  .minimum_timeout_us = 100000U,
  .timeout_interval_multiplier = 3.0f,
  .geometry.missing_tooth = {
    .tooth_positions = 4U,
    .missing_teeth = 1U,
  },
};

static const trigger_decoder_config_t crank_config =
{
  .wheels = &crank_wheel,
  .wheel_count = 1U,
  .primary_wheel = 0U,
  .phase_wheel = TRIGGER_NO_WHEEL,
  .engine_cycle_degrees = 720.0f,
  .phase_alignment_tolerance_deg = 30.0f,
};

static const trigger_wheel_config_t phased_wheels[] =
{
  {
    .name = "host crank 4-1",
    .input_channel = 1U,
    .active_edge = TRIGGER_EDGE_RISING,
    .type = TRIGGER_WHEEL_MISSING_TOOTH,
    .cycle_degrees = 360.0f,
    .angle_at_index0_deg = 0.0f,
    .angle_direction = +1,
    .interval_tolerance_permille = 200U,
    .speed_filter_permille = 1000U,
    .sync_confirmations = 1U,
    .minimum_interval_us = 100U,
    .minimum_timeout_us = 100000U,
    .timeout_interval_multiplier = 3.0f,
    .geometry.missing_tooth = {
      .tooth_positions = 4U,
      .missing_teeth = 1U,
    },
  },
  {
    .name = "host phase 4-1",
    .input_channel = 2U,
    .active_edge = TRIGGER_EDGE_RISING,
    .type = TRIGGER_WHEEL_MISSING_TOOTH,
    .cycle_degrees = 720.0f,
    .angle_at_index0_deg = 0.0f,
    .angle_direction = +1,
    .interval_tolerance_permille = 200U,
    .speed_filter_permille = 1000U,
    .sync_confirmations = 1U,
    .minimum_interval_us = 100U,
    .minimum_timeout_us = 200000U,
    .timeout_interval_multiplier = 3.0f,
    .geometry.missing_tooth = {
      .tooth_positions = 4U,
      .missing_teeth = 1U,
    },
  },
};

static const trigger_decoder_config_t phased_config =
{
  .wheels = phased_wheels,
  .wheel_count = 2U,
  .primary_wheel = 0U,
  .phase_wheel = 1U,
  .engine_cycle_degrees = 720.0f,
  .phase_alignment_tolerance_deg = 30.0f,
};

static engine_control_state_t read_engine_state(void)
{
  engine_control_state_t state;
  memset(&state, 0, sizeof(state));
  engine_control_get_state(&state);
  return state;
}

static engine_injector_test_state_t read_injector_test_state(void)
{
  engine_injector_test_state_t state;
  memset(&state, 0, sizeof(state));
  engine_control_get_injector_test_state(&state);
  return state;
}

static void record_edge(uint8_t channel, uint32_t timestamp)
{
  CHECK(engine_control_record_trigger_isr(channel, TRIGGER_EDGE_RISING,
                                           timestamp));
}

/* Two observed 4-1 gaps are needed because the first establishes the
 * reference candidate and the second confirms its spacing. */
static void queue_initial_crank_sync(void)
{
  record_edge(1U, 10000U);
  record_edge(1U, 20000U);
  record_edge(1U, 30000U);
  record_edge(1U, 50000U);
  record_edge(1U, 60000U);
  record_edge(1U, 70000U);
  record_edge(1U, 90000U);
}

static void enter_running_crank_only(const trigger_decoder_config_t *config)
{
  engine_control_state_t state;

  fake_reset();
  CHECK(engine_control_init(config, TEST_TICK_HZ));
  CHECK(engine_control_request_outputs(true));
  queue_initial_crank_sync();
  fake.now = 90000U;
  engine_control_service();

  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_RUNNING);
  CHECK(state.crank_synced);
  CHECK(!state.phase_synced);
  CHECK(state.outputs_enabled);
  CHECK(state.latched_faults == ENGINE_FAULT_NONE);
}

static void expect_incompatible_configuration(float engine_cycle,
                                              float primary_cycle,
                                              int8_t direction)
{
  trigger_wheel_config_t wheel = crank_wheel;
  trigger_decoder_config_t config = crank_config;
  engine_control_state_t state;

  wheel.cycle_degrees = primary_cycle;
  wheel.angle_direction = direction;
  config.wheels = &wheel;
  config.engine_cycle_degrees = engine_cycle;

  fake_reset();
  CHECK(!engine_control_init(&config, TEST_TICK_HZ));
  state = read_engine_state();
  CHECK(state.trigger_config_error == TRIGGER_CONFIG_OK);
  CHECK(state.mode == ENGINE_MODE_FAULT);
  CHECK((state.latched_faults & ENGINE_FAULT_CONFIGURATION) != 0U);
  CHECK(!state.outputs_requested);

  /* Reinitializing the decoder during clear must not bypass the additional
   * actuator-domain compatibility check. */
  CHECK(!engine_control_clear_faults());
  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_FAULT);
  CHECK((state.latched_faults & ENGINE_FAULT_CONFIGURATION) != 0U);
  CHECK(fake.spark_rearm_calls == 0U);
  CHECK(fake.injection_rearm_calls == 0U);
}

static void test_incompatible_configurations_and_clear(void)
{
  expect_incompatible_configuration(360.0f, 360.0f, +1);
  expect_incompatible_configuration(720.0f, 720.0f, +1);
  expect_incompatible_configuration(720.0f, 360.0f, -1);
}

static void queue_phase_acquisition_after_90000(void)
{
  /* Keep the primary wheel current while the half-speed phase wheel observes
   * two complete 4-1 reference gaps. Equal timestamps intentionally exercise
   * the recorder's cross-channel chronological merge. */
  record_edge(1U, 100000U);
  record_edge(1U, 110000U);
  record_edge(1U, 130000U);
  record_edge(1U, 140000U);
  record_edge(1U, 150000U);
  record_edge(1U, 170000U);
  record_edge(1U, 180000U);
  record_edge(1U, 190000U);
  record_edge(1U, 210000U);
  record_edge(1U, 220000U);
  record_edge(1U, 230000U);
  record_edge(1U, 250000U);

  record_edge(2U, 90000U);
  record_edge(2U, 110000U);
  record_edge(2U, 130000U);
  record_edge(2U, 170000U);
  record_edge(2U, 190000U);
  record_edge(2U, 210000U);
  record_edge(2U, 250000U);
}

static void test_runtime_cycle_selection(void)
{
  engine_control_state_t state;

  enter_running_crank_only(&phased_config);
  CHECK(fake.spark_runtime_present);
  CHECK(fake.injection_runtime_present);
  CHECK(fake.spark_runtime.cycle_deg == 360.0f);
  CHECK(fake.injection_runtime.cycle_deg == 360.0f);

  queue_phase_acquisition_after_90000();
  fake.now = 250000U;
  engine_control_service();
  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_RUNNING);
  CHECK(state.crank_synced);
  CHECK(state.phase_synced);
  CHECK(state.outputs_enabled);
  CHECK(state.latched_faults == ENGINE_FAULT_NONE);
  CHECK(fake.spark_runtime.cycle_deg == 720.0f);
  CHECK(fake.injection_runtime.cycle_deg == 720.0f);
}

static void test_live_phase_requirement_gates_running(void)
{
  engine_control_state_t state;

  fake_reset();
  CHECK(engine_control_init(&phased_config, TEST_TICK_HZ));
  fake.spark_requires_phase = true;
  CHECK(engine_control_request_outputs(true));
  queue_initial_crank_sync();
  fake.now = 90000U;
  engine_control_service();

  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_SYNCING);
  CHECK(state.crank_synced);
  CHECK(!state.phase_synced);
  CHECK(!state.outputs_enabled);
  CHECK(fake.spark_runtime_present);
  CHECK(fake.spark_runtime.cycle_deg == 360.0f);

  queue_phase_acquisition_after_90000();
  fake.now = 250000U;
  engine_control_service();
  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_RUNNING);
  CHECK(state.phase_synced);
  CHECK(state.outputs_enabled);
  CHECK(fake.spark_runtime.cycle_deg == 720.0f);
}

static void test_capture_queue_overflow_latches_fault(void)
{
  engine_control_state_t state;
  bool recorded = true;
  uint32_t timestamp = 10000U;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));

  /* A ring of length N intentionally holds only N-1 events. */
  for (unsigned event = 0U;
       event < (unsigned)TRIGGER_RECORDER_QUEUE_LENGTH;
       ++event)
  {
    recorded = engine_control_record_trigger_isr(
        1U, TRIGGER_EDGE_RISING, timestamp);
    timestamp += SHORT_PERIOD_TICKS;
    if (!recorded)
    {
      break;
    }
  }
  CHECK(!recorded);
  CHECK(fake.spark_emergency_calls > 0U);
  CHECK(fake.injection_emergency_calls > 0U);

  fake.now = timestamp;
  engine_control_service();
  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_FAULT);
  CHECK((state.latched_faults & ENGINE_FAULT_CAPTURE_QUEUE) != 0U);
  CHECK(state.capture_dropped_events == 1U);
  CHECK(!state.outputs_enabled);
}

static void test_capture_overrun_latches_fault_and_decoder_loss(void)
{
  engine_control_state_t state;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  engine_control_capture_overrun_isr(1U);
  CHECK(fake.spark_emergency_calls > 0U);
  CHECK(fake.injection_emergency_calls > 0U);

  fake.now = 1000U;
  engine_control_service();
  state = read_engine_state();
  CHECK(state.mode == ENGINE_MODE_FAULT);
  CHECK((state.latched_faults & ENGINE_FAULT_CAPTURE_OVERRUN) != 0U);
  CHECK(state.capture_overruns == 1U);
  CHECK(state.last_trigger_loss == TRIGGER_LOSS_EVENT_DROPPED);
  CHECK((state.trigger_latched_faults &
         (1UL << TRIGGER_LOSS_EVENT_DROPPED)) != 0U);
  CHECK(!state.outputs_enabled);
}

static void test_transient_loss_and_reacquisition_still_latches(void)
{
  engine_control_state_t state;
  uint32_t previous_epoch;

  enter_running_crank_only(&crank_config);
  state = read_engine_state();
  previous_epoch = state.sync_epoch;

  /* The first 14 ms interval is neither a valid tooth nor a valid gap. The
   * remaining records contain two correct references and reacquire sync before
   * the same foreground drain ends. The final synced output must not hide the
   * intervening epoch change from engine_control. */
  record_edge(1U, 104000U);
  record_edge(1U, 114000U);
  record_edge(1U, 124000U);
  record_edge(1U, 144000U);
  record_edge(1U, 154000U);
  record_edge(1U, 164000U);
  record_edge(1U, 184000U);
  fake.now = 184000U;
  engine_control_service();

  state = read_engine_state();
  CHECK(state.crank_synced);
  CHECK(state.sync_epoch != previous_epoch);
  CHECK((state.trigger_latched_faults &
         (1UL << TRIGGER_LOSS_UNEXPECTED_INTERVAL)) != 0U);
  CHECK((state.latched_faults & ENGINE_FAULT_TRIGGER_SYNC_LOST) != 0U);
  CHECK(state.mode == ENGINE_MODE_FAULT);
  CHECK(!state.outputs_enabled);
  CHECK(fake.spark_emergency_calls > 0U);
  CHECK(fake.injection_emergency_calls > 0U);
}

static void test_deadline_dispatch(void)
{
  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  engine_control_deadline_isr(12345U);
  CHECK(fake.spark_deadline_calls == 1U);
  CHECK(fake.injection_deadline_calls == 1U);
}

static void test_injector_test_start_and_first_period(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 2U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };
  engine_control_state_t engine;
  engine_injector_test_state_t test;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  fake.now = 999999U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY);
  CHECK(fake.injection_debug_start_calls == 0U);
  fake.now = 1000500U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  CHECK(fake.injection_debug_start_calls == 1U);
  CHECK(fake.injection_debug_start_output == 2U);
  CHECK(fake.injection_debug_start_period_us == 10000U);
  CHECK(fake.injection_debug_start_pulse_width_us == 500U);
  CHECK(fake.injection_debug_start_now == 1000500U);
  CHECK(fake.injection_debug_start_tick_hz == TEST_TICK_HZ);

  engine = read_engine_state();
  CHECK(engine.mode == ENGINE_MODE_INJECTOR_TEST);
  CHECK(!engine.outputs_requested);
  CHECK(engine.outputs_enabled);
  CHECK(engine.timestamp == 1000500U);
  test = read_injector_test_state();
  CHECK(test.active);
  CHECK(test.injector_output == 2U);
  CHECK(test.period_ms == 10U);
  CHECK(test.pulse_width_us == 500U);
  CHECK(test.pulse_count == 0U);
  CHECK(test.skipped_period_count == 0U);
  CHECK(test.last_result == ENGINE_INJECTOR_TEST_OK);

  /* Arming never emits an immediate pulse. The controller delegates the
   * exact free-running timestamp to the debug scheduler on each service. */
  fake.now = 1010499U;
  engine_control_service();
  CHECK(fake.injection_debug_service_calls == 1U);
  CHECK(fake.injection_debug_service_now == 1010499U);
  CHECK(read_injector_test_state().pulse_count == 0U);
  CHECK(fake.spark_runtime_present == false);

  fake.now = 1010500U;
  engine_control_service();
  CHECK(fake.injection_debug_service_calls == 2U);
  CHECK(fake.injection_debug_service_now == 1010500U);
  test = read_injector_test_state();
  CHECK(test.active);
  CHECK(test.pulse_count == 1U);
  CHECK(read_engine_state().mode == ENGINE_MODE_INJECTOR_TEST);
}

static void test_injector_test_output_arm_interlock(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 1U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };
  engine_injector_test_state_t test;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  CHECK(engine_control_request_outputs(true));
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_ENGINE_BUSY);
  CHECK(fake.injection_debug_start_calls == 0U);

  CHECK(engine_control_request_outputs(false));
  fake.now = 1000100U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  CHECK(!engine_control_request_outputs(true));
  test = read_injector_test_state();
  CHECK(test.active);
  CHECK(test.last_result == ENGINE_INJECTOR_TEST_ENGINE_BUSY);
  CHECK(read_engine_state().mode == ENGINE_MODE_INJECTOR_TEST);
  engine_control_stop_injector_test();
}

static void test_injector_test_explicit_stop(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 3U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };
  engine_control_state_t engine;
  engine_injector_test_state_t test;
  unsigned debug_service_calls;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  fake.now = 1001000U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  engine_control_stop_injector_test();
  CHECK(fake.injection_debug_stop_calls == 1U);
  test = read_injector_test_state();
  CHECK(!test.active);
  CHECK(test.last_result == ENGINE_INJECTOR_TEST_OK);
  engine = read_engine_state();
  CHECK(engine.mode == ENGINE_MODE_DISABLED);
  CHECK(!engine.outputs_requested);
  CHECK(!engine.outputs_enabled);

  /* Stop is idempotent and a later foreground pass cannot resurrect the
   * retained debug configuration. */
  engine_control_stop_injector_test();
  CHECK(fake.injection_debug_stop_calls == 1U);
  debug_service_calls = fake.injection_debug_service_calls;
  fake.now = 1011000U;
  engine_control_service();
  CHECK(fake.injection_debug_service_calls == debug_service_calls);
  CHECK(!read_injector_test_state().active);
}

static void test_idle_injector_test_stop_does_not_stop_engine(void)
{
  engine_control_state_t before;
  engine_control_state_t after;
  unsigned debug_stop_calls;

  enter_running_crank_only(&crank_config);
  before = read_engine_state();
  CHECK(before.mode == ENGINE_MODE_RUNNING);
  CHECK(before.outputs_requested);
  CHECK(before.outputs_enabled);
  debug_stop_calls = fake.injection_debug_stop_calls;

  engine_control_stop_injector_test();
  after = read_engine_state();
  CHECK(after.mode == ENGINE_MODE_RUNNING);
  CHECK(after.outputs_requested);
  CHECK(after.outputs_enabled);
  CHECK(fake.injection_debug_stop_calls == debug_stop_calls);
}

static void test_injector_test_argument_and_mapping_results(void)
{
  engine_injector_test_config_t config = {
    .injector_output = 1U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };
  engine_injector_test_state_t test;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  CHECK(engine_control_start_injector_test(NULL) ==
        ENGINE_INJECTOR_TEST_INVALID_ARGUMENT);
  config.period_ms = 0U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_INVALID_ARGUMENT);
  CHECK(fake.injection_debug_start_calls == 0U);

  config.period_ms = 10U;
  fake.now = 1000000U;
  config.injector_output = 13U;
  fake.injection_debug_start_result = INJECTION_DEBUG_INVALID_OUTPUT;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_INVALID_ARGUMENT);
  CHECK(fake.injection_debug_start_calls == 1U);
  CHECK(!read_injector_test_state().active);

  config.injector_output = 5U;
  fake.injection_debug_start_result = INJECTION_DEBUG_UNMAPPED_OUTPUT;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_UNMAPPED_OUTPUT);
  CHECK(fake.injection_debug_start_calls == 2U);
  test = read_injector_test_state();
  CHECK(!test.active);
  CHECK(test.last_result == ENGINE_INJECTOR_TEST_UNMAPPED_OUTPUT);
  CHECK(read_engine_state().mode == ENGINE_MODE_DISABLED);
}

static void test_injector_test_requires_trigger_quiet_time(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 1U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  record_edge(1U, 1000U);
  fake.now = 1000U;
  engine_control_service();       /* drain the edge: queue is now empty */

  fake.now = 1000999U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY);
  CHECK(fake.injection_debug_start_calls == 0U);

  fake.now = 1001000U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  CHECK(fake.injection_debug_start_calls == 1U);
  engine_control_stop_injector_test();

  /* Once observed, quiet remains true until another edge; an ancient
   * absolute deadline must not flip after half of the 32-bit timer range. */
  fake.now = 0x90001000U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  engine_control_stop_injector_test();
}

static void test_capture_overrun_restarts_injector_quiet_time(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 1U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  fake.now = 1234U;
  engine_control_capture_overrun_isr(1U);
  engine_control_service();
  CHECK((read_engine_state().latched_faults &
         ENGINE_FAULT_CAPTURE_OVERRUN) != 0U);
  CHECK(engine_control_clear_faults());

  fake.now = 1001233U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY);
  fake.now = 1001234U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);
  engine_control_stop_injector_test();
}

static void test_trigger_edge_aborts_injector_test_in_isr(void)
{
  const engine_injector_test_config_t config = {
    .injector_output = 1U,
    .period_ms = 10U,
    .pulse_width_us = 500U,
  };
  engine_control_state_t engine;
  engine_injector_test_state_t test;

  fake_reset();
  CHECK(engine_control_init(&crank_config, TEST_TICK_HZ));
  fake.now = 1000100U;
  CHECK(engine_control_start_injector_test(&config) ==
        ENGINE_INJECTOR_TEST_OK);

  CHECK(!engine_control_record_trigger_isr(1U, TRIGGER_EDGE_RISING,
                                            1000200U));
  CHECK(fake.injection_debug_abort_calls == 1U);
  CHECK(fake.spark_emergency_calls == 1U);
  CHECK(!fake.injection_debug_state.active);
  CHECK((fake.injection_fault_bits & INJECTION_FAULT_DEBUG_INTERLOCK) != 0U);
  test = read_injector_test_state();
  CHECK(!test.active);
  CHECK(test.last_result == ENGINE_INJECTOR_TEST_FAULT_LATCHED);

  /* A stop/revoke call before the regular service pass must account pending
   * ISR faults and must not replace FAULT_LATCHED with OK. */
  engine_control_stop_injector_test();
  CHECK(read_injector_test_state().last_result ==
        ENGINE_INJECTOR_TEST_FAULT_LATCHED);
  engine = read_engine_state();
  CHECK(engine.mode == ENGINE_MODE_FAULT);
  CHECK(!engine.outputs_enabled);

  /* The rejected edge never enters the recorder. The regular foreground pass
   * keeps the already-published fault and every output disabled. */
  fake.now = 200U;
  engine_control_service();
  engine = read_engine_state();
  CHECK(engine.mode == ENGINE_MODE_FAULT);
  CHECK((engine.latched_faults & ENGINE_FAULT_INJECTION) != 0U);
  CHECK((engine.injection_faults & INJECTION_FAULT_DEBUG_INTERLOCK) != 0U);
  CHECK(!engine.crank_synced);
  CHECK(!engine.outputs_enabled);
  CHECK(fake.injection_debug_service_calls == 0U);
}

int main(void)
{
  test_incompatible_configurations_and_clear();
  test_runtime_cycle_selection();
  test_live_phase_requirement_gates_running();
  test_capture_queue_overflow_latches_fault();
  test_capture_overrun_latches_fault_and_decoder_loss();
  test_transient_loss_and_reacquisition_still_latches();
  test_deadline_dispatch();
  test_injector_test_start_and_first_period();
  test_injector_test_output_arm_interlock();
  test_injector_test_explicit_stop();
  test_idle_injector_test_stop_does_not_stop_engine();
  test_injector_test_argument_and_mapping_results();
  test_injector_test_requires_trigger_quiet_time();
  test_capture_overrun_restarts_injector_quiet_time();
  test_trigger_edge_aborts_injector_test_in_isr();

  (void)printf("PASS: %u engine-control integration checks\n", checks_run);
  return EXIT_SUCCESS;
}
