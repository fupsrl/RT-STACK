/**
 ******************************************************************************
 * @file    dcdc_platform_stm32.h
 * @brief   User-owned STM32 integration for DCDC PWM, ADC and safety IRQs.
 ******************************************************************************
 */
#ifndef DCDC_PLATFORM_STM32_H
#define DCDC_PLATFORM_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef struct
{
  TIM_HandleTypeDef *pwm_timer;
  ADC_HandleTypeDef *vbus_adc;
  volatile uint16_t *adc_dma_values;
  uint8_t adc_dma_value_count;
  uint8_t vbus_dma_index;
  uint8_t vref_dma_index;
  uint16_t fallback_vdda_mv;
} dcdc_platform_stm32_config_t;

/* Call after Cube has initialized TIM1 and ADC1, but before ADC calibration. */
bool dcdc_platform_stm32_init(
    const dcdc_platform_stm32_config_t *config,
    uint32_t now_ms);

/* Call after ADC calibration and before starting ADC1 DMA. */
bool dcdc_platform_stm32_configure_analog_watchdog(void);

/* Call after ADC1 DMA starts. Enables the safety IRQ and optional boot arm. */
bool dcdc_platform_stm32_start(void);

/* Foreground and off-only 1 ms ISR hooks. */
void dcdc_platform_stm32_service(uint32_t now_ms);
void dcdc_platform_stm32_systick_isr(uint32_t now_ms);

/* User-owned vector implementation; startup_stm32g4xx.s supplies the weak
 * alias and no Cube-generated declaration is required. */
void ADC1_2_IRQHandler(void);

#endif /* DCDC_PLATFORM_STM32_H */
