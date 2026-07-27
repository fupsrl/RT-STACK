/** @file dcdc_pwm.c  Pure TIM1 center-aligned PWM calculations. */
#include "dcdc_pwm.h"

#include <stddef.h>

#define NANOSECONDS_PER_SECOND 1000000000ULL
#define PERMILLE_SCALE               1000ULL
#define TIMER_16_BIT_MAX             65535ULL

static uint64_t divide_round_nearest(uint64_t numerator, uint64_t denominator)
{
  return (numerator + (denominator / 2ULL)) / denominator;
}

static uint64_t divide_round_up(uint64_t numerator, uint64_t denominator)
{
  return (numerator + denominator - 1ULL) / denominator;
}

bool dcdc_pwm_plan_compute(const dcdc_pwm_config_t *config,
                           uint16_t duty_permille,
                           dcdc_pwm_plan_t *plan)
{
  uint64_t ticks_per_half_cycle;
  uint64_t prescaler_plus_one;
  uint64_t auto_reload;
  uint64_t minimum_on_ticks;
  uint64_t minimum_off_ticks;
  uint64_t configured_minimum_ticks;
  uint64_t configured_maximum_ticks;
  uint64_t on_ticks;

  if ((config == NULL) || (plan == NULL) ||
      (config->timer_clock_hz == 0U) ||
      (config->switching_frequency_hz == 0U) ||
      (config->minimum_active_duty_permille >
       config->maximum_duty_permille) ||
      (config->maximum_duty_permille >= 1000U) ||
      (duty_permille > config->maximum_duty_permille))
  {
    return false;
  }

  ticks_per_half_cycle = divide_round_nearest(
      config->timer_clock_hz,
      2ULL * (uint64_t)config->switching_frequency_hz);
  if (ticks_per_half_cycle < 2ULL)
  {
    return false;
  }

  prescaler_plus_one = divide_round_up(ticks_per_half_cycle,
                                       TIMER_16_BIT_MAX);
  if ((prescaler_plus_one == 0ULL) ||
      (prescaler_plus_one > (TIMER_16_BIT_MAX + 1ULL)))
  {
    return false;
  }

  auto_reload = divide_round_nearest(
      config->timer_clock_hz,
      2ULL * (uint64_t)config->switching_frequency_hz *
          prescaler_plus_one);
  if ((auto_reload < 2ULL) || (auto_reload > TIMER_16_BIT_MAX))
  {
    return false;
  }

  minimum_on_ticks = divide_round_up(
      (uint64_t)config->minimum_on_time_ns * config->timer_clock_hz,
      2ULL * NANOSECONDS_PER_SECOND * prescaler_plus_one);
  minimum_off_ticks = divide_round_up(
      (uint64_t)config->minimum_off_time_ns * config->timer_clock_hz,
      2ULL * NANOSECONDS_PER_SECOND * prescaler_plus_one);

  if ((minimum_on_ticks > auto_reload) ||
      (minimum_off_ticks > auto_reload) ||
      (minimum_on_ticks > TIMER_16_BIT_MAX) ||
      (minimum_off_ticks > TIMER_16_BIT_MAX))
  {
    return false;
  }

  configured_minimum_ticks = divide_round_up(
      auto_reload * config->minimum_active_duty_permille,
      PERMILLE_SCALE);
  configured_maximum_ticks =
      (auto_reload * config->maximum_duty_permille) / PERMILLE_SCALE;

  /* maximum_duty=0 is the intentional hardware-locked configuration. */
  if (config->maximum_duty_permille != 0U)
  {
    if ((configured_minimum_ticks < minimum_on_ticks) ||
        (configured_maximum_ticks > (auto_reload - minimum_off_ticks)) ||
        (configured_minimum_ticks > configured_maximum_ticks))
    {
      return false;
    }
  }

  if ((duty_permille != 0U) &&
      (duty_permille < config->minimum_active_duty_permille))
  {
    return false;
  }

  /* Floor normal commands so timer quantization can never exceed the user's
   * maximum-duty ceiling. The single exception is the validated minimum
   * active pulse, which is rounded upward to satisfy minimum on-time. */
  on_ticks = (auto_reload * duty_permille) / PERMILLE_SCALE;
  if ((duty_permille != 0U) &&
      (on_ticks < configured_minimum_ticks))
  {
    on_ticks = configured_minimum_ticks;
  }
  if (on_ticks > configured_maximum_ticks)
  {
    on_ticks = configured_maximum_ticks;
  }
  if ((on_ticks != 0ULL) &&
      ((on_ticks < minimum_on_ticks) ||
       ((auto_reload - on_ticks) < minimum_off_ticks)))
  {
    return false;
  }

  *plan = (dcdc_pwm_plan_t){
    .prescaler = (uint16_t)(prescaler_plus_one - 1ULL),
    .auto_reload = (uint16_t)auto_reload,
    .phase_a_compare = (uint16_t)on_ticks,
    .phase_b_compare = (uint16_t)(auto_reload - on_ticks),
    .requested_duty_permille = duty_permille,
    .actual_duty_permille = (uint16_t)divide_round_nearest(
        on_ticks * PERMILLE_SCALE, auto_reload),
    .minimum_on_ticks = (uint16_t)minimum_on_ticks,
    .minimum_off_ticks = (uint16_t)minimum_off_ticks,
    .actual_frequency_hz = (uint32_t)divide_round_nearest(
        config->timer_clock_hz,
        2ULL * prescaler_plus_one * auto_reload),
    .full_period_timer_ticks = (uint32_t)(2ULL * auto_reload),
    .phase_shift_timer_ticks = (uint32_t)auto_reload,
  };
  return true;
}
