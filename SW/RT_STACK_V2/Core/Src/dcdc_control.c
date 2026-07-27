/**
 ******************************************************************************
 * @file    dcdc_control.c
 * @brief   Fixed-rate VBUS controller with soft start and latched shutdowns.
 ******************************************************************************
 */
#include "dcdc_control.h"

#include <math.h>
#include <stddef.h>

#include "dcdc_config.h"

#define PERMILLE_TO_DUTY 0.001f
#define MV_TO_VOLTS       0.001f

typedef struct
{
  dcdc_control_config_t config;
  dcdc_pwm_driver_t driver;
  volatile dcdc_mode_t mode;
  volatile uint32_t faults;
  volatile bool initialized;
  volatile bool enable_requested;
  volatile bool pwm_running;
  volatile uint32_t foreground_heartbeat_ms;
  volatile uint32_t latest_sample_ms;
  volatile bool sample_seen;
  volatile bool latest_sample_valid;

  uint32_t latest_vbus_mv;
  uint32_t requested_target_mv;
  float ramped_target_mv;
  float filtered_vbus_mv;
  float previous_filtered_vbus_mv;
  float filtered_derivative_volts_per_second;
  float integral_duty;
  uint16_t confirmed_sample_count;
  uint16_t applied_duty_permille;
  uint32_t enable_requested_at_ms;
  uint32_t last_control_ms;
} dcdc_controller_t;

static dcdc_controller_t controller;

static const dcdc_control_config_t default_config = {
  .output_calibration_valid = (DCDC_OUTPUT_PERMISSION != 0U),
  .vbus_plausible_min_mv = DCDC_VBUS_PLAUSIBLE_MIN_MV,
  .vbus_plausible_max_mv = DCDC_VBUS_PLAUSIBLE_MAX_MV,
  .target_min_mv = DCDC_VBUS_TARGET_MIN_MV,
  .target_default_mv = DCDC_VBUS_TARGET_DEFAULT_MV,
  .target_max_mv = DCDC_VBUS_TARGET_MAX_MV,
  .overvoltage_reset_mv = DCDC_VBUS_OVERVOLTAGE_RESET_MV,
  .overvoltage_trip_mv = DCDC_VBUS_OVERVOLTAGE_TRIP_MV,
  .control_period_ms = DCDC_CONTROL_PERIOD_MS,
  .control_deadline_ms = DCDC_CONTROL_DEADLINE_MS,
  .sample_timeout_ms = DCDC_VBUS_SAMPLE_TIMEOUT_MS,
  .startup_valid_sample_count = DCDC_STARTUP_VALID_SAMPLE_COUNT,
  .soft_start_mv_per_second = DCDC_SOFT_START_MV_PER_SECOND,
  .minimum_active_duty_permille = DCDC_MINIMUM_ACTIVE_DUTY_PERMILLE,
  .maximum_duty_permille = DCDC_MAXIMUM_DUTY_PERMILLE,
  .duty_rise_per_control_permille =
      DCDC_DUTY_RISE_PER_CONTROL_PERMILLE,
  .duty_fall_per_control_permille =
      DCDC_DUTY_FALL_PER_CONTROL_PERMILLE,
  .kp_duty_per_volt = DCDC_PID_KP_DUTY_PER_VOLT,
  .ki_duty_per_volt_second = DCDC_PID_KI_DUTY_PER_VOLT_SECOND,
  .kd_duty_second_per_volt = DCDC_PID_KD_DUTY_SECOND_PER_VOLT,
  .anti_windup_per_second = DCDC_PID_ANTI_WINDUP_PER_SECOND,
  .measurement_filter_time_constant_ms =
      DCDC_VBUS_FILTER_TIME_CONSTANT_MS,
  .derivative_filter_time_constant_ms =
      DCDC_DERIVATIVE_FILTER_TIME_CONSTANT_MS,
};

static float clamp_float(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

static uint16_t move_duty_toward(uint16_t current, uint16_t requested)
{
  if (requested > current)
  {
    uint32_t next = (uint32_t)current +
                    controller.config.duty_rise_per_control_permille;
    return (uint16_t)((next < requested) ? next : requested);
  }
  if (requested < current)
  {
    uint16_t step = controller.config.duty_fall_per_control_permille;
    if ((uint32_t)requested + step < current)
    {
      return (uint16_t)(current - step);
    }
    return requested;
  }
  return current;
}

static bool driver_is_complete(const dcdc_pwm_driver_t *driver)
{
  return (driver != NULL) && (driver->start != NULL) &&
         (driver->set_duty_permille != NULL) && (driver->stop != NULL) &&
         (driver->is_running != NULL) &&
         (driver->critical_enter != NULL) &&
         (driver->critical_exit != NULL);
}

static bool full_config_is_valid(const dcdc_control_config_t *config)
{
  if ((config == NULL) || (config->control_period_ms == 0U) ||
      (config->control_deadline_ms < (2U * config->control_period_ms)) ||
      (config->sample_timeout_ms < config->control_period_ms) ||
      (config->startup_valid_sample_count == 0U))
  {
    return false;
  }

  if (!config->output_calibration_valid)
  {
    return true;
  }

  if ((config->vbus_plausible_min_mv >= config->target_min_mv) ||
      (config->target_min_mv > config->target_default_mv) ||
      (config->target_default_mv > config->target_max_mv) ||
      (config->target_max_mv > config->overvoltage_reset_mv) ||
      (config->overvoltage_reset_mv >= config->overvoltage_trip_mv) ||
      (config->overvoltage_trip_mv > config->vbus_plausible_max_mv) ||
      (config->soft_start_mv_per_second == 0U) ||
      (config->minimum_active_duty_permille == 0U) ||
      (config->minimum_active_duty_permille >
       config->maximum_duty_permille) ||
      (config->maximum_duty_permille >= 1000U) ||
      (config->duty_rise_per_control_permille == 0U) ||
      (config->duty_fall_per_control_permille == 0U) ||
      !isfinite(config->kp_duty_per_volt) ||
      !isfinite(config->ki_duty_per_volt_second) ||
      !isfinite(config->kd_duty_second_per_volt) ||
      !isfinite(config->anti_windup_per_second) ||
      (config->kp_duty_per_volt < 0.0f) ||
      (config->ki_duty_per_volt_second < 0.0f) ||
      (config->kd_duty_second_per_volt < 0.0f) ||
      (config->anti_windup_per_second <= 0.0f) ||
      ((config->kp_duty_per_volt == 0.0f) &&
       (config->ki_duty_per_volt_second == 0.0f)))
  {
    return false;
  }
  return true;
}

static void reset_pid(void)
{
  controller.ramped_target_mv = 0.0f;
  controller.filtered_vbus_mv = 0.0f;
  controller.previous_filtered_vbus_mv = 0.0f;
  controller.filtered_derivative_volts_per_second = 0.0f;
  controller.integral_duty = 0.0f;
  controller.confirmed_sample_count = 0U;
  controller.applied_duty_permille = 0U;
}

static void stop_pwm(void)
{
  if (controller.initialized)
  {
    controller.driver.stop(controller.driver.context);
  }
  controller.pwm_running = false;
  controller.applied_duty_permille = 0U;
}

static void latch_fault(uint32_t fault)
{
  uint32_t critical_token =
      controller.driver.critical_enter(controller.driver.context);

  stop_pwm();
  controller.faults |= fault;
  controller.enable_requested = false;
  controller.mode = DCDC_MODE_FAULT;
  controller.driver.critical_exit(controller.driver.context, critical_token);
}

static bool sample_is_plausible(const dcdc_vbus_sample_t *sample)
{
  return sample->valid &&
         (sample->vbus_mv >= controller.config.vbus_plausible_min_mv) &&
         (sample->vbus_mv <= controller.config.vbus_plausible_max_mv);
}

const dcdc_control_config_t *dcdc_control_default_config(void)
{
  return &default_config;
}

bool dcdc_vbus_adc_to_mv(uint16_t raw,
                         uint16_t vdda_mv,
                         uint32_t *vbus_mv)
{
  uint64_t numerator;
  uint64_t denominator;

  if ((vbus_mv == NULL) || (raw > DCDC_ADC_FULL_SCALE_COUNTS) ||
      (vdda_mv < DCDC_ADC_VALID_VDDA_MIN_MV) ||
      (vdda_mv > DCDC_ADC_VALID_VDDA_MAX_MV))
  {
    return false;
  }

  numerator = (uint64_t)raw * vdda_mv *
      ((uint64_t)DCDC_VBUS_DIVIDER_HIGH_OHMS +
       DCDC_VBUS_DIVIDER_LOW_OHMS);
  denominator = (uint64_t)DCDC_ADC_FULL_SCALE_COUNTS *
                DCDC_VBUS_DIVIDER_LOW_OHMS;
  *vbus_mv = (uint32_t)((numerator + (denominator / 2ULL)) / denominator);
  return true;
}

bool dcdc_control_init(const dcdc_control_config_t *config,
                       const dcdc_pwm_driver_t *driver,
                       uint32_t now_ms)
{
  controller = (dcdc_controller_t){0};
  if (!full_config_is_valid(config) || !driver_is_complete(driver))
  {
    controller.mode = DCDC_MODE_FAULT;
    controller.faults = DCDC_FAULT_CONFIGURATION;
    return false;
  }

  controller.config = *config;
  controller.driver = *driver;
  controller.initialized = true;
  controller.foreground_heartbeat_ms = now_ms;
  controller.last_control_ms = now_ms;
  controller.requested_target_mv = config->target_default_mv;
  controller.driver.stop(controller.driver.context);
  controller.mode = config->output_calibration_valid ?
      DCDC_MODE_DISABLED : DCDC_MODE_LOCKED;
  return true;
}

bool dcdc_control_request_enable(bool enable)
{
  uint32_t critical_token;

  if (!controller.initialized)
  {
    return false;
  }

  critical_token =
      controller.driver.critical_enter(controller.driver.context);

  if (!enable)
  {
    controller.enable_requested = false;
    stop_pwm();
    reset_pid();
    controller.mode = (controller.faults != DCDC_FAULT_NONE) ?
        DCDC_MODE_FAULT :
        (controller.config.output_calibration_valid ?
         DCDC_MODE_DISABLED : DCDC_MODE_LOCKED);
    controller.driver.critical_exit(controller.driver.context,
                                    critical_token);
    return true;
  }

  if (!controller.config.output_calibration_valid ||
      (controller.faults != DCDC_FAULT_NONE))
  {
    controller.driver.critical_exit(controller.driver.context,
                                    critical_token);
    return false;
  }

  /* An enable request is level-triggered, not a restart command. Repeating it
   * must not reset the PID or bypass the fresh-sample startup gate while an
   * old nonzero PWM command is still active. */
  if (controller.enable_requested)
  {
    controller.driver.critical_exit(controller.driver.context,
                                    critical_token);
    return true;
  }

  reset_pid();
  controller.enable_requested = true;
  controller.enable_requested_at_ms = controller.foreground_heartbeat_ms;
  controller.last_control_ms = controller.foreground_heartbeat_ms;
  controller.mode = DCDC_MODE_WAITING_FOR_VBUS;
  controller.driver.critical_exit(controller.driver.context, critical_token);
  return true;
}

bool dcdc_control_set_target_mv(uint32_t target_mv)
{
  if (!controller.initialized ||
      !controller.config.output_calibration_valid ||
      (target_mv < controller.config.target_min_mv) ||
      (target_mv > controller.config.target_max_mv))
  {
    return false;
  }

  controller.requested_target_mv = target_mv;
  if (controller.ramped_target_mv > (float)target_mv)
  {
    /* A lower request takes effect immediately; only upward changes ramp. */
    controller.ramped_target_mv = (float)target_mv;
  }
  return true;
}

bool dcdc_control_clear_faults(void)
{
  uint32_t sample_age;
  uint32_t critical_token;

  if (!controller.initialized)
  {
    return false;
  }

  critical_token =
      controller.driver.critical_enter(controller.driver.context);

  if (controller.enable_requested || controller.pwm_running ||
      !controller.latest_sample_valid ||
      !controller.sample_seen)
  {
    controller.driver.critical_exit(controller.driver.context,
                                    critical_token);
    return false;
  }

  sample_age = controller.foreground_heartbeat_ms -
               controller.latest_sample_ms;
  if ((sample_age > controller.config.sample_timeout_ms) ||
      (controller.latest_vbus_mv > controller.config.overvoltage_reset_mv))
  {
    controller.driver.critical_exit(controller.driver.context,
                                    critical_token);
    return false;
  }

  controller.faults = DCDC_FAULT_NONE;
  reset_pid();
  controller.mode = controller.config.output_calibration_valid ?
      DCDC_MODE_DISABLED : DCDC_MODE_LOCKED;
  controller.driver.critical_exit(controller.driver.context, critical_token);
  return true;
}

static bool begin_switching(uint32_t now_ms)
{
  controller.filtered_vbus_mv = (float)controller.latest_vbus_mv;
  controller.previous_filtered_vbus_mv = controller.filtered_vbus_mv;
  controller.ramped_target_mv = controller.filtered_vbus_mv;
  if (controller.ramped_target_mv > (float)controller.requested_target_mv)
  {
    controller.ramped_target_mv = (float)controller.requested_target_mv;
  }
  controller.integral_duty = 0.0f;
  controller.applied_duty_permille = 0U;
  controller.pwm_running = true;
  controller.last_control_ms = now_ms;
  controller.mode = DCDC_MODE_SOFT_START;

  if (!controller.driver.start(controller.driver.context) ||
      (controller.faults != DCDC_FAULT_NONE) ||
      !controller.enable_requested ||
      !controller.driver.is_running(controller.driver.context))
  {
    latch_fault((controller.faults != DCDC_FAULT_NONE) ?
                controller.faults : DCDC_FAULT_PWM_BACKEND);
    return false;
  }

  return true;
}

static void run_pid_step(uint32_t now_ms, uint32_t elapsed_ms)
{
  const float period_seconds =
      (float)elapsed_ms * 0.001f;
  const float maximum_duty =
      (float)controller.config.maximum_duty_permille * PERMILLE_TO_DUTY;
  float measurement_alpha;
  float derivative_alpha;
  float ramp_step_mv;
  float error_volts;
  float raw_derivative;
  float unconstrained_duty;
  float constrained_duty;
  float applied_duty;
  float integral_delta;
  uint16_t requested_duty_permille;
  uint16_t next_duty_permille;

  measurement_alpha = (controller.config.measurement_filter_time_constant_ms ==
                       0U) ? 1.0f :
      (float)elapsed_ms /
      (float)(controller.config.measurement_filter_time_constant_ms +
              elapsed_ms);
  controller.filtered_vbus_mv += measurement_alpha *
      ((float)controller.latest_vbus_mv - controller.filtered_vbus_mv);

  ramp_step_mv = (float)controller.config.soft_start_mv_per_second *
                 period_seconds;
  if (controller.ramped_target_mv < (float)controller.requested_target_mv)
  {
    controller.ramped_target_mv += ramp_step_mv;
    if (controller.ramped_target_mv > (float)controller.requested_target_mv)
    {
      controller.ramped_target_mv = (float)controller.requested_target_mv;
    }
  }

  raw_derivative = ((controller.filtered_vbus_mv -
                     controller.previous_filtered_vbus_mv) * MV_TO_VOLTS) /
                   period_seconds;
  derivative_alpha =
      (controller.config.derivative_filter_time_constant_ms == 0U) ? 1.0f :
      (float)elapsed_ms /
      (float)(controller.config.derivative_filter_time_constant_ms +
              elapsed_ms);
  controller.filtered_derivative_volts_per_second += derivative_alpha *
      (raw_derivative - controller.filtered_derivative_volts_per_second);
  controller.previous_filtered_vbus_mv = controller.filtered_vbus_mv;

  error_volts = (controller.ramped_target_mv -
                  controller.filtered_vbus_mv) * MV_TO_VOLTS;
  unconstrained_duty =
      (controller.config.kp_duty_per_volt * error_volts) +
      controller.integral_duty -
      (controller.config.kd_duty_second_per_volt *
       controller.filtered_derivative_volts_per_second);
  if (!isfinite(unconstrained_duty))
  {
    latch_fault(DCDC_FAULT_NUMERIC);
    return;
  }

  constrained_duty = clamp_float(unconstrained_duty, 0.0f, maximum_duty);
  requested_duty_permille = (uint16_t)(constrained_duty * 1000.0f + 0.5f);
  if ((requested_duty_permille != 0U) &&
      (requested_duty_permille <
       controller.config.minimum_active_duty_permille))
  {
    /* Do not generate sub-minimum pulses. Below half the minimum command the
     * safer quantization is OFF; above it use the validated minimum pulse. */
    requested_duty_permille =
        (requested_duty_permille <
         (controller.config.minimum_active_duty_permille / 2U)) ? 0U :
        controller.config.minimum_active_duty_permille;
  }

  next_duty_permille = move_duty_toward(
      controller.applied_duty_permille, requested_duty_permille);
  if ((next_duty_permille != 0U) &&
      (next_duty_permille <
       controller.config.minimum_active_duty_permille))
  {
    next_duty_permille =
        (requested_duty_permille == 0U) ? 0U :
        controller.config.minimum_active_duty_permille;
  }

  applied_duty = (float)next_duty_permille * PERMILLE_TO_DUTY;
  integral_delta = period_seconds *
      ((controller.config.ki_duty_per_volt_second * error_volts) +
       (controller.config.anti_windup_per_second *
        (applied_duty - unconstrained_duty)));
  controller.integral_duty = clamp_float(
      controller.integral_duty + integral_delta,
      -maximum_duty, maximum_duty);

  if (!isfinite(controller.integral_duty))
  {
    latch_fault(DCDC_FAULT_NUMERIC);
    return;
  }

  /* Commit foreground state before calling the backend. If an off-only ISR
   * preempts anywhere below, its FAULT/OFF state is the last writer. If it
   * preempts earlier, the post-call checks detect the latched fault and stop
   * a backend that incorrectly reported success. */
  controller.applied_duty_permille = next_duty_permille;
  controller.last_control_ms = now_ms;
  controller.mode = (controller.ramped_target_mv <
                     (float)controller.requested_target_mv) ?
      DCDC_MODE_SOFT_START : DCDC_MODE_REGULATING;

  if ((controller.faults != DCDC_FAULT_NONE) ||
      !controller.enable_requested ||
      !controller.driver.set_duty_permille(controller.driver.context,
                                            next_duty_permille) ||
      (controller.faults != DCDC_FAULT_NONE) ||
      !controller.enable_requested ||
      !controller.driver.is_running(controller.driver.context))
  {
    latch_fault((controller.faults != DCDC_FAULT_NONE) ?
                controller.faults : DCDC_FAULT_PWM_BACKEND);
  }
}

void dcdc_control_service(const dcdc_vbus_sample_t *sample, uint32_t now_ms)
{
  uint32_t elapsed_control;
  uint32_t sample_age;
  bool new_sample = false;

  if (!controller.initialized)
  {
    return;
  }
  controller.foreground_heartbeat_ms = now_ms;

  if (sample != NULL)
  {
    new_sample = !controller.sample_seen ||
                 (sample->captured_at_ms != controller.latest_sample_ms);
    if (new_sample)
    {
      controller.sample_seen = true;
      controller.latest_sample_ms = sample->captured_at_ms;
      controller.latest_vbus_mv = sample->vbus_mv;

      /* Raw overvoltage wins over the broader plausibility classification:
       * an out-of-range high sample must never be reported merely as a sensor
       * issue when it already crosses the hard shutdown threshold. */
      if (sample->valid && controller.config.output_calibration_valid &&
          (sample->vbus_mv >= controller.config.overvoltage_trip_mv))
      {
        controller.latest_sample_valid = true;
        latch_fault(DCDC_FAULT_OVERVOLTAGE);
        return;
      }

      controller.latest_sample_valid = sample_is_plausible(sample);

      if (!controller.latest_sample_valid)
      {
        controller.confirmed_sample_count = 0U;
        if (controller.enable_requested || controller.pwm_running)
        {
          latch_fault(DCDC_FAULT_VBUS_SENSOR);
          return;
        }
      }
      else
      {
        if (controller.confirmed_sample_count < UINT16_MAX)
        {
          ++controller.confirmed_sample_count;
        }
      }
    }
    else if (!sample->valid &&
             (controller.enable_requested || controller.pwm_running))
    {
      latch_fault(DCDC_FAULT_VBUS_SENSOR);
      return;
    }
  }

  if (!controller.enable_requested)
  {
    if (controller.pwm_running)
    {
      stop_pwm();
    }
    return;
  }

  if (controller.faults != DCDC_FAULT_NONE)
  {
    stop_pwm();
    return;
  }

  if (!controller.sample_seen)
  {
    if ((now_ms - controller.enable_requested_at_ms) >
        controller.config.sample_timeout_ms)
    {
      latch_fault(DCDC_FAULT_VBUS_STALE);
    }
    return;
  }

  sample_age = now_ms - controller.latest_sample_ms;
  if (sample_age > controller.config.sample_timeout_ms)
  {
    latch_fault(DCDC_FAULT_VBUS_STALE);
    return;
  }

  if (!controller.latest_sample_valid)
  {
    latch_fault(DCDC_FAULT_VBUS_SENSOR);
    return;
  }

  if (!controller.pwm_running)
  {
    if (controller.confirmed_sample_count >=
        controller.config.startup_valid_sample_count)
    {
      (void)begin_switching(now_ms);
    }
    return;
  }

  elapsed_control = now_ms - controller.last_control_ms;
  if (elapsed_control > controller.config.control_deadline_ms)
  {
    latch_fault(DCDC_FAULT_CONTROL_DEADLINE);
    return;
  }
  if (elapsed_control < controller.config.control_period_ms)
  {
    return;
  }

  /* Use the real, bounded elapsed interval. There is no catch-up burst: one
   * foreground pass produces at most one duty update, while PID integration,
   * derivative and soft-start remain dimensionally correct if a legal pass is
   * later than the nominal period. */
  run_pid_step(now_ms, elapsed_control);
}

void dcdc_control_deadline_tick_isr(uint32_t now_ms)
{
  uint32_t heartbeat_age;
  uint32_t sample_age;

  if (!controller.initialized || !controller.pwm_running)
  {
    return;
  }

  heartbeat_age = now_ms - controller.foreground_heartbeat_ms;
  sample_age = now_ms - controller.latest_sample_ms;
  if (heartbeat_age > controller.config.control_deadline_ms)
  {
    latch_fault(DCDC_FAULT_CONTROL_DEADLINE);
  }
  else if (!controller.sample_seen ||
           (sample_age > controller.config.sample_timeout_ms))
  {
    latch_fault(DCDC_FAULT_VBUS_STALE);
  }
}

void dcdc_control_overvoltage_isr(void)
{
  if (controller.initialized)
  {
    latch_fault(DCDC_FAULT_OVERVOLTAGE);
  }
}

void dcdc_control_emergency_off(void)
{
  if (controller.initialized)
  {
    latch_fault(DCDC_FAULT_EMERGENCY_STOP);
  }
}

void dcdc_control_get_state(dcdc_control_state_t *state)
{
  if (state == NULL)
  {
    return;
  }

  *state = (dcdc_control_state_t){
    .mode = controller.mode,
    .faults = controller.faults,
    .enable_requested = controller.enable_requested,
    .pwm_running = controller.pwm_running,
    .output_calibration_valid = controller.config.output_calibration_valid,
    .latest_sample_valid = controller.latest_sample_valid,
    .confirmed_sample_count = controller.confirmed_sample_count,
    .duty_permille = controller.applied_duty_permille,
    .latest_vbus_mv = controller.latest_vbus_mv,
    .latest_sample_ms = controller.latest_sample_ms,
    .target_mv = controller.requested_target_mv,
    .ramped_target_mv = (uint32_t)controller.ramped_target_mv,
    .filtered_vbus_mv = controller.filtered_vbus_mv,
    .integral_duty = controller.integral_duty,
  };
}
