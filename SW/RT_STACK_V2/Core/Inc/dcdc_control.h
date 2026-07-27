/** @file dcdc_control.h  Fail-safe VBUS PID and converter state machine. */
#ifndef DCDC_CONTROL_H
#define DCDC_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "dcdc_pwm.h"

typedef enum
{
  DCDC_MODE_UNINITIALIZED = 0,
  DCDC_MODE_LOCKED,
  DCDC_MODE_DISABLED,
  DCDC_MODE_WAITING_FOR_VBUS,
  DCDC_MODE_SOFT_START,
  DCDC_MODE_REGULATING,
  DCDC_MODE_FAULT,
} dcdc_mode_t;

typedef enum
{
  DCDC_FAULT_NONE             = 0U,
  DCDC_FAULT_CONFIGURATION    = (1UL << 0),
  DCDC_FAULT_VBUS_SENSOR      = (1UL << 1),
  DCDC_FAULT_VBUS_STALE       = (1UL << 2),
  DCDC_FAULT_OVERVOLTAGE      = (1UL << 3),
  DCDC_FAULT_CONTROL_DEADLINE = (1UL << 4),
  DCDC_FAULT_PWM_BACKEND      = (1UL << 5),
  DCDC_FAULT_EMERGENCY_STOP   = (1UL << 6),
  DCDC_FAULT_NUMERIC          = (1UL << 7),
} dcdc_fault_t;

typedef struct
{
  uint32_t vbus_mv;
  uint32_t captured_at_ms;
  bool valid;
} dcdc_vbus_sample_t;

typedef struct
{
  bool output_calibration_valid;

  uint32_t vbus_plausible_min_mv;
  uint32_t vbus_plausible_max_mv;
  uint32_t target_min_mv;
  uint32_t target_default_mv;
  uint32_t target_max_mv;
  uint32_t overvoltage_reset_mv;
  uint32_t overvoltage_trip_mv;

  uint32_t control_period_ms;
  uint32_t control_deadline_ms;
  uint32_t sample_timeout_ms;
  uint16_t startup_valid_sample_count;
  uint32_t soft_start_mv_per_second;

  uint16_t minimum_active_duty_permille;
  uint16_t maximum_duty_permille;
  uint16_t duty_rise_per_control_permille;
  uint16_t duty_fall_per_control_permille;

  float kp_duty_per_volt;
  float ki_duty_per_volt_second;
  float kd_duty_second_per_volt;
  float anti_windup_per_second;
  uint32_t measurement_filter_time_constant_ms;
  uint32_t derivative_filter_time_constant_ms;
} dcdc_control_config_t;

typedef struct
{
  dcdc_mode_t mode;
  uint32_t faults;
  bool enable_requested;
  bool pwm_running;
  bool output_calibration_valid;
  bool latest_sample_valid;
  uint16_t confirmed_sample_count;
  uint16_t duty_permille;
  uint32_t latest_vbus_mv;
  uint32_t latest_sample_ms;
  uint32_t target_mv;
  uint32_t ramped_target_mv;
  float filtered_vbus_mv;
  float integral_duty;
} dcdc_control_state_t;

const dcdc_control_config_t *dcdc_control_default_config(void);

/* Convert the nominal divider reading. This routine validates ADC range and
 * VDDA, but cannot detect D53 conduction or divider calibration errors. */
bool dcdc_vbus_adc_to_mv(uint16_t raw,
                         uint16_t vdda_mv,
                         uint32_t *vbus_mv);

bool dcdc_control_init(const dcdc_control_config_t *config,
                       const dcdc_pwm_driver_t *driver,
                       uint32_t now_ms);
/* A true return accepts the synchronous request. An asynchronous watchdog or
 * emergency IRQ may still latch FAULT immediately afterward; use get_state()
 * for authoritative operational state and never infer that PWM is energized
 * from a request/clear return value alone. */
bool dcdc_control_request_enable(bool enable);
bool dcdc_control_set_target_mv(uint32_t target_mv);
bool dcdc_control_clear_faults(void);
void dcdc_control_service(const dcdc_vbus_sample_t *sample, uint32_t now_ms);
void dcdc_control_get_state(dcdc_control_state_t *state);

/* Off-only paths. They are safe to call from interrupt/fault context and can
 * never start or increase PWM. The 1 ms deadline hook belongs in SysTick. */
void dcdc_control_deadline_tick_isr(uint32_t now_ms);
void dcdc_control_overvoltage_isr(void);
void dcdc_control_emergency_off(void);

#endif /* DCDC_CONTROL_H */
