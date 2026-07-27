/** @file dcdc_pwm_stm32.h  STM32G4 TIM1 interleaved PWM backend. */
#ifndef DCDC_PWM_STM32_H
#define DCDC_PWM_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "dcdc_pwm.h"

bool dcdc_pwm_stm32_init(TIM_HandleTypeDef *timer,
                          const dcdc_pwm_config_t *config,
                          bool outputs_permitted);
const dcdc_pwm_driver_t *dcdc_pwm_stm32_driver(void);
void dcdc_pwm_stm32_get_plan(dcdc_pwm_plan_t *plan);

/* Direct, off-only hardware action; safe even before controller init. */
void dcdc_pwm_stm32_emergency_off(void);

#endif /* DCDC_PWM_STM32_H */
