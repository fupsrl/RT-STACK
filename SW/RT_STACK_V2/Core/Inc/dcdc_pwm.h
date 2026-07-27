/** @file dcdc_pwm.h  Platform-independent two-phase PWM timing model. */
#ifndef DCDC_PWM_H
#define DCDC_PWM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t timer_clock_hz;
  uint32_t switching_frequency_hz;
  uint32_t minimum_on_time_ns;
  uint32_t minimum_off_time_ns;
  uint16_t minimum_active_duty_permille;
  uint16_t maximum_duty_permille;
} dcdc_pwm_config_t;

typedef struct
{
  uint16_t prescaler;
  uint16_t auto_reload;
  uint16_t phase_a_compare;
  uint16_t phase_b_compare;
  uint16_t requested_duty_permille;
  uint16_t actual_duty_permille;
  uint16_t minimum_on_ticks;
  uint16_t minimum_off_ticks;
  uint32_t actual_frequency_hz;
  uint32_t full_period_timer_ticks;
  uint32_t phase_shift_timer_ticks;
} dcdc_pwm_plan_t;

/* CH1 uses PWM1 and CH3 uses PWM2 on one center-aligned counter. For on_ticks
 * n, CCR1=n and CCR3=ARR-n. Their equal pulses are centered at the counter
 * valley and peak, so phase B is exactly half a full PWM period after phase A. */
bool dcdc_pwm_plan_compute(const dcdc_pwm_config_t *config,
                           uint16_t duty_permille,
                           dcdc_pwm_plan_t *plan);

typedef struct
{
  void *context;
  bool (*start)(void *context);
  bool (*set_duty_permille)(void *context, uint16_t duty_permille);
  void (*stop)(void *context);
  bool (*is_running)(const void *context);
  uint32_t (*critical_enter)(void *context);
  void (*critical_exit)(void *context, uint32_t token);
} dcdc_pwm_driver_t;

#endif /* DCDC_PWM_H */
