#include "dcdc_control.h"
#include "dcdc_pwm.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOCK_HISTORY_CAPACITY 128U

static unsigned int check_count;

#define CHECK(expression)                                                     \
  do                                                                          \
  {                                                                           \
    ++check_count;                                                            \
    if (!(expression))                                                       \
    {                                                                         \
      fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__,     \
              #expression);                                                   \
      exit(EXIT_FAILURE);                                                     \
    }                                                                         \
  } while (0)

typedef struct
{
  bool running;
  bool fail_start;
  bool fail_set;
  bool emergency_during_start;
  bool emergency_during_set;
  bool emergency_on_critical_exit;
  uint16_t duty_permille;
  uint16_t history[MOCK_HISTORY_CAPACITY];
  size_t history_count;
  unsigned int start_count;
  unsigned int set_count;
  unsigned int stop_count;
} mock_pwm_t;

static bool mock_start(void *context)
{
  mock_pwm_t *mock = context;

  ++mock->start_count;
  if (mock->fail_start)
  {
    return false;
  }
  if (mock->emergency_during_start)
  {
    /* Model the worst ordering: shutdown runs, then a faulty backend reports
     * success and re-arms itself before returning to the controller. */
    dcdc_control_overvoltage_isr();
  }
  mock->running = true;
  mock->duty_permille = 0U;
  return true;
}

static bool mock_set_duty(void *context, uint16_t duty_permille)
{
  mock_pwm_t *mock = context;

  ++mock->set_count;
  if (mock->fail_set || !mock->running)
  {
    return false;
  }
  if (mock->emergency_during_set)
  {
    dcdc_control_overvoltage_isr();
    mock->running = true;
  }
  mock->duty_permille = duty_permille;
  if (mock->history_count < MOCK_HISTORY_CAPACITY)
  {
    mock->history[mock->history_count] = duty_permille;
    ++mock->history_count;
  }
  return true;
}

static void mock_stop(void *context)
{
  mock_pwm_t *mock = context;

  ++mock->stop_count;
  mock->running = false;
  mock->duty_permille = 0U;
}

static bool mock_is_running(const void *context)
{
  const mock_pwm_t *mock = context;
  return mock->running;
}

static uint32_t mock_critical_enter(void *context)
{
  (void)context;
  return 0U;
}

static void mock_critical_exit(void *context, uint32_t token)
{
  mock_pwm_t *mock = context;

  (void)token;
  if (mock->emergency_on_critical_exit)
  {
    mock->emergency_on_critical_exit = false;
    dcdc_control_overvoltage_isr();
  }
}

static dcdc_pwm_driver_t mock_driver(mock_pwm_t *mock)
{
  dcdc_pwm_driver_t driver = {
    .context = mock,
    .start = mock_start,
    .set_duty_permille = mock_set_duty,
    .stop = mock_stop,
    .is_running = mock_is_running,
    .critical_enter = mock_critical_enter,
    .critical_exit = mock_critical_exit,
  };
  return driver;
}

static dcdc_control_config_t control_config(void)
{
  dcdc_control_config_t config = {
    .output_calibration_valid = true,
    .vbus_plausible_min_mv = 1000U,
    .vbus_plausible_max_mv = 70000U,
    .target_min_mv = 20000U,
    .target_default_mv = 40000U,
    .target_max_mv = 50000U,
    .overvoltage_reset_mv = 55000U,
    .overvoltage_trip_mv = 60000U,
    .control_period_ms = 10U,
    .control_deadline_ms = 40U,
    .sample_timeout_ms = 30U,
    .startup_valid_sample_count = 2U,
    .soft_start_mv_per_second = 2000000U,
    .minimum_active_duty_permille = 50U,
    .maximum_duty_permille = 450U,
    .duty_rise_per_control_permille = 50U,
    .duty_fall_per_control_permille = 100U,
    .kp_duty_per_volt = 0.1f,
    .ki_duty_per_volt_second = 0.0f,
    .kd_duty_second_per_volt = 0.0f,
    .anti_windup_per_second = 10.0f,
    .measurement_filter_time_constant_ms = 0U,
    .derivative_filter_time_constant_ms = 0U,
  };
  return config;
}

static dcdc_vbus_sample_t sample(uint32_t vbus_mv,
                                 uint32_t captured_at_ms)
{
  dcdc_vbus_sample_t value = {
    .vbus_mv = vbus_mv,
    .captured_at_ms = captured_at_ms,
    .valid = true,
  };
  return value;
}

static void initialize_and_enable(mock_pwm_t *mock,
                                  dcdc_control_config_t *config)
{
  dcdc_pwm_driver_t driver;

  memset(mock, 0, sizeof(*mock));
  driver = mock_driver(mock);
  CHECK(dcdc_control_init(config, &driver, 0U));
  CHECK(mock->stop_count == 1U);
  CHECK(dcdc_control_request_enable(true));
}

static void provide_startup_samples(uint32_t vbus_mv)
{
  dcdc_vbus_sample_t value;

  value = sample(vbus_mv, 0U);
  dcdc_control_service(&value, 0U);
  value = sample(vbus_mv, 1U);
  dcdc_control_service(&value, 1U);
}

static bool phase_a_level(const dcdc_pwm_plan_t *plan, uint32_t tick)
{
  uint32_t counter;

  if (tick < plan->phase_shift_timer_ticks)
  {
    counter = tick;
    return counter < plan->phase_a_compare;
  }
  counter = plan->full_period_timer_ticks - tick;
  return counter <= plan->phase_a_compare;
}

static bool phase_b_level(const dcdc_pwm_plan_t *plan, uint32_t tick)
{
  uint32_t counter;

  if (tick < plan->phase_shift_timer_ticks)
  {
    counter = tick;
    return counter >= plan->phase_b_compare;
  }
  counter = plan->full_period_timer_ticks - tick;
  return counter > plan->phase_b_compare;
}

static void check_waveform(const dcdc_pwm_plan_t *plan)
{
  uint32_t a_high_count = 0U;
  uint32_t b_high_count = 0U;
  uint32_t a_rise = UINT32_MAX;
  uint32_t b_rise = UINT32_MAX;
  uint32_t a_fall = UINT32_MAX;
  uint32_t b_fall = UINT32_MAX;
  uint32_t a_rise_count = 0U;
  uint32_t b_rise_count = 0U;
  uint32_t a_fall_count = 0U;
  uint32_t b_fall_count = 0U;
  bool previous_a;
  bool previous_b;
  uint32_t tick;

  CHECK(plan->phase_a_compare != 0U);
  previous_a = phase_a_level(plan, plan->full_period_timer_ticks - 1U);
  previous_b = phase_b_level(plan, plan->full_period_timer_ticks - 1U);
  for (tick = 0U; tick < plan->full_period_timer_ticks; ++tick)
  {
    bool current_a = phase_a_level(plan, tick);
    bool current_b = phase_b_level(plan, tick);

    a_high_count += current_a ? 1U : 0U;
    b_high_count += current_b ? 1U : 0U;
    if (current_a && !previous_a)
    {
      a_rise = tick;
      ++a_rise_count;
    }
    if (!current_a && previous_a)
    {
      a_fall = tick;
      ++a_fall_count;
    }
    if (current_b && !previous_b)
    {
      b_rise = tick;
      ++b_rise_count;
    }
    if (!current_b && previous_b)
    {
      b_fall = tick;
      ++b_fall_count;
    }
    previous_a = current_a;
    previous_b = current_b;
  }

  CHECK(a_high_count == b_high_count);
  CHECK(a_high_count == (2U * plan->phase_a_compare));
  CHECK(a_rise_count == 1U);
  CHECK(b_rise_count == 1U);
  CHECK(a_fall_count == 1U);
  CHECK(b_fall_count == 1U);
  CHECK(((a_rise + plan->phase_shift_timer_ticks) %
         plan->full_period_timer_ticks) == b_rise);
  CHECK(((a_fall + plan->phase_shift_timer_ticks) %
         plan->full_period_timer_ticks) == b_fall);
}

static void test_pwm_plan_and_waveform(void)
{
  const uint16_t duties[] = { 50U, 250U, 450U };
  const uint16_t overlap_duties[] = { 550U, 900U };
  dcdc_pwm_config_t config = {
    .timer_clock_hz = 170000000U,
    .switching_frequency_hz = 100000U,
    .minimum_on_time_ns = 100U,
    .minimum_off_time_ns = 100U,
    .minimum_active_duty_permille = 50U,
    .maximum_duty_permille = 450U,
  };
  dcdc_pwm_plan_t plan;
  size_t i;

  for (i = 0U; i < (sizeof(duties) / sizeof(duties[0])); ++i)
  {
    CHECK(dcdc_pwm_plan_compute(&config, duties[i], &plan));
    CHECK(plan.actual_frequency_hz == 100000U);
    CHECK(plan.full_period_timer_ticks ==
          (2U * plan.phase_shift_timer_ticks));
    CHECK(plan.phase_shift_timer_ticks == plan.auto_reload);
    CHECK((uint32_t)plan.phase_a_compare + plan.phase_b_compare ==
          plan.phase_shift_timer_ticks);
    CHECK(plan.actual_duty_permille <= config.maximum_duty_permille);
    check_waveform(&plan);
  }

  CHECK(dcdc_pwm_plan_compute(&config, 0U, &plan));
  CHECK(plan.phase_a_compare == 0U);
  CHECK(plan.phase_b_compare == plan.auto_reload);
  CHECK(plan.actual_duty_permille == 0U);
  CHECK(!dcdc_pwm_plan_compute(&config, 49U, &plan));
  CHECK(!dcdc_pwm_plan_compute(&config, 451U, &plan));

  /* Independent boost phases intentionally overlap above 50%; they must keep
   * equal widths and the same exact half-period cyclic edge displacement. */
  config.maximum_duty_permille = 900U;
  for (i = 0U;
       i < (sizeof(overlap_duties) / sizeof(overlap_duties[0])); ++i)
  {
    CHECK(dcdc_pwm_plan_compute(&config, overlap_duties[i], &plan));
    CHECK(plan.actual_duty_permille <= config.maximum_duty_permille);
    check_waveform(&plan);
  }

  /* Minimum pulse conversion must use the exact prescaler ratio, not a
   * truncated intermediate timer frequency. */
  config.switching_frequency_hz = 100U;
  config.minimum_on_time_ns = 123456U;
  config.minimum_off_time_ns = 123456U;
  CHECK(dcdc_pwm_plan_compute(&config, 0U, &plan));
  CHECK((2ULL * plan.minimum_on_ticks *
         ((uint64_t)plan.prescaler + 1ULL) * 1000000000ULL) >=
        ((uint64_t)config.minimum_on_time_ns * config.timer_clock_hz));
  CHECK((2ULL * (plan.minimum_on_ticks - 1ULL) *
         ((uint64_t)plan.prescaler + 1ULL) * 1000000000ULL) <
        ((uint64_t)config.minimum_on_time_ns * config.timer_clock_hz));

  config.switching_frequency_hz = 100000000U;
  config.minimum_on_time_ns = 100U;
  config.minimum_off_time_ns = 100U;
  CHECK(!dcdc_pwm_plan_compute(&config, 0U, &plan));
  config.switching_frequency_hz = 100000U;
  config.minimum_on_time_ns = 1000U;
  CHECK(!dcdc_pwm_plan_compute(&config, 0U, &plan));
  CHECK(!dcdc_pwm_plan_compute(NULL, 0U, &plan));
  CHECK(!dcdc_pwm_plan_compute(&config, 0U, NULL));
}

static void test_adc_conversion_validation(void)
{
  uint32_t vbus_mv = 0U;

  CHECK(dcdc_vbus_adc_to_mv(4095U, 3300U, &vbus_mv));
  CHECK(vbus_mv == 85800U);
  CHECK(dcdc_vbus_adc_to_mv(0U, 3300U, &vbus_mv));
  CHECK(vbus_mv == 0U);
  CHECK(!dcdc_vbus_adc_to_mv(4096U, 3300U, &vbus_mv));
  CHECK(!dcdc_vbus_adc_to_mv(100U, 2799U, &vbus_mv));
  CHECK(!dcdc_vbus_adc_to_mv(100U, 3601U, &vbus_mv));
  CHECK(!dcdc_vbus_adc_to_mv(100U, 3300U, NULL));
}

static void test_soft_start_and_slew(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;

  config.soft_start_mv_per_second = 100000U;
  initialize_and_enable(&mock, &config);
  provide_startup_samples(20000U);
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_SOFT_START);
  CHECK(state.pwm_running);
  CHECK(state.duty_permille == 0U);
  CHECK(mock.start_count == 1U);

  value = sample(20000U, 6U);
  dcdc_control_service(&value, 6U);
  dcdc_control_get_state(&state);
  CHECK(state.ramped_target_mv == 20000U);
  CHECK(state.duty_permille == 0U);
  CHECK(mock.set_count == 0U);

  value = sample(20000U, 11U);
  dcdc_control_service(&value, 11U);
  dcdc_control_get_state(&state);
  CHECK(state.ramped_target_mv == 21000U);
  CHECK(state.duty_permille == 50U);
  CHECK(mock.duty_permille == 50U);

  value = sample(20000U, 21U);
  dcdc_control_service(&value, 21U);
  dcdc_control_get_state(&state);
  CHECK(state.ramped_target_mv == 22000U);
  CHECK(state.duty_permille == 100U);
  CHECK(mock.history_count == 2U);
  CHECK(mock.history[1] - mock.history[0] <=
        config.duty_rise_per_control_permille);
}

static void test_elapsed_time_and_repeated_enable(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t before;
  dcdc_control_state_t after;
  dcdc_vbus_sample_t value;
  unsigned int start_count;

  config.soft_start_mv_per_second = 100000U;
  initialize_and_enable(&mock, &config);
  provide_startup_samples(20000U);

  /* A legal 20 ms interval advances a 100 V/s ramp by 2 V, not by the
   * nominal 10 ms scheduler period. It still emits only one duty update. */
  value = sample(20000U, 21U);
  dcdc_control_service(&value, 21U);
  dcdc_control_get_state(&before);
  CHECK(before.ramped_target_mv == 22000U);
  CHECK(before.duty_permille == 50U);
  CHECK(mock.set_count == 1U);

  start_count = mock.start_count;
  CHECK(dcdc_control_request_enable(true));
  dcdc_control_get_state(&after);
  CHECK(after.mode == before.mode);
  CHECK(after.duty_permille == before.duty_permille);
  CHECK(after.confirmed_sample_count == before.confirmed_sample_count);
  CHECK(mock.start_count == start_count);
  CHECK(mock.running);
  CHECK(mock.duty_permille == before.duty_permille);
}

static void test_pid_direction_and_anti_windup(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;
  uint32_t now_ms;

  config.duty_rise_per_control_permille = 450U;
  config.duty_fall_per_control_permille = 450U;
  initialize_and_enable(&mock, &config);
  provide_startup_samples(39000U);
  value = sample(39000U, 11U);
  dcdc_control_service(&value, 11U);
  dcdc_control_get_state(&state);
  CHECK(state.duty_permille == 100U);
  value = sample(41000U, 21U);
  dcdc_control_service(&value, 21U);
  dcdc_control_get_state(&state);
  CHECK(state.duty_permille == 0U);

  config.kp_duty_per_volt = 0.05f;
  config.anti_windup_per_second = 20.0f;
  initialize_and_enable(&mock, &config);
  provide_startup_samples(10000U);
  for (now_ms = 11U; now_ms <= 101U; now_ms += 10U)
  {
    value = sample(10000U, now_ms);
    dcdc_control_service(&value, now_ms);
  }
  dcdc_control_get_state(&state);
  CHECK(state.duty_permille == config.maximum_duty_permille);
  CHECK(state.integral_duty <= 0.0f);
  CHECK(state.integral_duty >= -0.451f);

  value = sample(50000U, 111U);
  dcdc_control_service(&value, 111U);
  dcdc_control_get_state(&state);
  CHECK(state.duty_permille == 0U);
  CHECK(mock.duty_permille == 0U);
}

static void test_sensor_stale_overvoltage_and_backend_failures(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  CHECK(mock.running);
  dcdc_control_service(NULL, 32U);
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_VBUS_STALE) != 0U);
  CHECK(!state.pwm_running);
  CHECK(!mock.running);

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  value = sample(30000U, 2U);
  value.valid = false;
  dcdc_control_service(&value, 2U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_VBUS_SENSOR) != 0U);
  CHECK(!mock.running);

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  value = sample(config.overvoltage_trip_mv, 2U);
  dcdc_control_service(&value, 2U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!mock.running);

  initialize_and_enable(&mock, &config);
  mock.fail_start = true;
  provide_startup_samples(30000U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_PWM_BACKEND) != 0U);
  CHECK(!state.pwm_running);

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  mock.fail_set = true;
  value = sample(30000U, 11U);
  dcdc_control_service(&value, 11U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_PWM_BACKEND) != 0U);
  CHECK(!mock.running);
}

static void test_deadline_isr_and_timestamp_wrap(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_pwm_driver_t driver;
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  dcdc_control_deadline_tick_isr(42U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_CONTROL_DEADLINE) != 0U);
  CHECK(!state.pwm_running);
  CHECK(!mock.running);

  memset(&mock, 0, sizeof(mock));
  config.startup_valid_sample_count = 1U;
  driver = mock_driver(&mock);
  CHECK(dcdc_control_init(&config, &driver, UINT32_MAX - 10U));
  CHECK(dcdc_control_request_enable(true));
  value = sample(30000U, UINT32_MAX - 5U);
  dcdc_control_service(&value, UINT32_MAX - 5U);
  dcdc_control_get_state(&state);
  CHECK(state.pwm_running);

  /* Unsigned elapsed-time arithmetic keeps the pre-wrap sample fresh for
   * 26 ms, then expires it at 31 ms. */
  dcdc_control_service(NULL, 20U);
  dcdc_control_get_state(&state);
  CHECK(state.faults == DCDC_FAULT_NONE);
  CHECK(state.pwm_running);
  dcdc_control_service(NULL, 25U);
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_VBUS_STALE) != 0U);
  CHECK(!state.pwm_running);
}

static void test_emergency_latch_and_clear(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;
  unsigned int set_count;

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  value = sample(30000U, 11U);
  dcdc_control_service(&value, 11U);
  set_count = mock.set_count;
  dcdc_control_emergency_off();
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_EMERGENCY_STOP) != 0U);
  CHECK(!state.enable_requested);
  CHECK(!state.pwm_running);
  CHECK(!mock.running);
  CHECK(!dcdc_control_request_enable(true));

  value = sample(30000U, 12U);
  dcdc_control_service(&value, 12U);
  CHECK(mock.set_count == set_count);
  CHECK(dcdc_control_clear_faults());
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_DISABLED);
  CHECK(state.faults == DCDC_FAULT_NONE);
  CHECK(dcdc_control_request_enable(true));

  dcdc_control_overvoltage_isr();
  dcdc_control_get_state(&state);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!state.enable_requested);
}

static void test_emergency_during_backend_call_stays_off(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;

  initialize_and_enable(&mock, &config);
  mock.emergency_during_start = true;
  provide_startup_samples(30000U);
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!state.enable_requested);
  CHECK(!state.pwm_running);
  CHECK(!mock.running);
  CHECK(mock.duty_permille == 0U);

  initialize_and_enable(&mock, &config);
  provide_startup_samples(30000U);
  mock.emergency_during_set = true;
  value = sample(30000U, 11U);
  dcdc_control_service(&value, 11U);
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!state.enable_requested);
  CHECK(!state.pwm_running);
  CHECK(!mock.running);
  CHECK(mock.duty_permille == 0U);
}

static void test_pending_emergency_after_state_transition_is_preserved(void)
{
  mock_pwm_t mock;
  dcdc_control_config_t config = control_config();
  dcdc_control_state_t state;
  dcdc_vbus_sample_t value;

  memset(&mock, 0, sizeof(mock));
  dcdc_pwm_driver_t driver = mock_driver(&mock);
  CHECK(dcdc_control_init(&config, &driver, 0U));
  mock.emergency_on_critical_exit = true;
  CHECK(dcdc_control_request_enable(true));
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!state.enable_requested);
  CHECK(!state.pwm_running);

  value = sample(30000U, 1U);
  dcdc_control_service(&value, 1U);
  CHECK(dcdc_control_clear_faults());
  mock.emergency_on_critical_exit = true;
  CHECK(dcdc_control_clear_faults());
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_FAULT);
  CHECK((state.faults & DCDC_FAULT_OVERVOLTAGE) != 0U);
  CHECK(!state.enable_requested);
  CHECK(!state.pwm_running);
}

static void test_default_configuration_is_hardware_locked(void)
{
  mock_pwm_t mock = {0};
  dcdc_pwm_driver_t driver = mock_driver(&mock);
  dcdc_control_state_t state;
  /* The production-locked calibration deliberately has zero-valued voltage
   * limits. A nonzero reading is therefore invalid, but it must not unlock or
   * start the converter. */
  dcdc_vbus_sample_t value = sample(1U, 0U);

  CHECK(dcdc_control_init(dcdc_control_default_config(), &driver, 0U));
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_LOCKED);
  CHECK(!state.output_calibration_valid);
  CHECK(!dcdc_control_request_enable(true));
  CHECK(!dcdc_control_set_target_mv(0U));
  dcdc_control_service(&value, 0U);
  dcdc_control_get_state(&state);
  CHECK(state.mode == DCDC_MODE_LOCKED);
  CHECK(!state.pwm_running);
  CHECK(mock.start_count == 0U);
  CHECK(mock.set_count == 0U);
}

int main(void)
{
  test_adc_conversion_validation();
  test_soft_start_and_slew();
  test_elapsed_time_and_repeated_enable();
  test_pid_direction_and_anti_windup();
  test_sensor_stale_overvoltage_and_backend_failures();
  test_deadline_isr_and_timestamp_wrap();
  test_emergency_latch_and_clear();
  test_emergency_during_backend_call_stays_off();
  test_pending_emergency_after_state_transition_is_preserved();
  test_default_configuration_is_hardware_locked();
  test_pwm_plan_and_waveform();

  printf("DCDC host tests passed (%u checks).\n", check_count);
  return EXIT_SUCCESS;
}
