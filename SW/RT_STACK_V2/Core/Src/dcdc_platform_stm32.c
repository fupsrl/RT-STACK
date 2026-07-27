/**
 ******************************************************************************
 * @file    dcdc_platform_stm32.c
 * @brief   Regeneration-safe STM32 DCDC integration and ADC safety vector.
 ******************************************************************************
 */
#include "dcdc_platform_stm32.h"

#include <stddef.h>

#include "dcdc_config.h"
#include "dcdc_control.h"
#include "dcdc_pwm_stm32.h"

typedef struct
{
  dcdc_platform_stm32_config_t config;
  dcdc_vbus_sample_t sample;
  bool initialized;
  bool sample_available;
} dcdc_platform_context_t;

static dcdc_platform_context_t platform;

static uint32_t tim1_input_clock_hz(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();

  /* STM32 timers receive twice PCLK when their APB prescaler is not DIV1. */
  if ((RCC->CFGR & RCC_CFGR_PPRE2_Msk) != RCC_CFGR_PPRE2_DIV1)
  {
    timer_clock_hz *= 2U;
  }
  return timer_clock_hz;
}

static bool platform_config_is_valid(
    const dcdc_platform_stm32_config_t *config)
{
  return (config != NULL) && (config->pwm_timer != NULL) &&
         (config->vbus_adc != NULL) &&
         (config->vbus_adc->Instance == ADC1) &&
         (config->adc_dma_values != NULL) &&
         (config->adc_dma_value_count != 0U) &&
         (config->vbus_dma_index < config->adc_dma_value_count) &&
         (config->vref_dma_index < config->adc_dma_value_count) &&
         (config->fallback_vdda_mv != 0U);
}

bool dcdc_platform_stm32_init(
    const dcdc_platform_stm32_config_t *config,
    uint32_t now_ms)
{
  dcdc_pwm_config_t pwm;

  platform = (dcdc_platform_context_t){0};
  if (!platform_config_is_valid(config))
  {
    return false;
  }

  pwm = (dcdc_pwm_config_t){
    .timer_clock_hz = tim1_input_clock_hz(),
    .switching_frequency_hz = DCDC_PWM_FREQUENCY_HZ,
    .minimum_on_time_ns = DCDC_PWM_MINIMUM_ON_TIME_NS,
    .minimum_off_time_ns = DCDC_PWM_MINIMUM_OFF_TIME_NS,
    .minimum_active_duty_permille =
        DCDC_MINIMUM_ACTIVE_DUTY_PERMILLE,
    .maximum_duty_permille = DCDC_MAXIMUM_DUTY_PERMILLE,
  };

  if (!dcdc_pwm_stm32_init(config->pwm_timer, &pwm,
                           DCDC_OUTPUT_PERMISSION != 0U) ||
      !dcdc_control_init(dcdc_control_default_config(),
                         dcdc_pwm_stm32_driver(), now_ms))
  {
    dcdc_pwm_stm32_emergency_off();
    return false;
  }

  platform.config = *config;
  platform.initialized = true;
  return true;
}

bool dcdc_platform_stm32_configure_analog_watchdog(void)
{
  if (!platform.initialized)
  {
    return false;
  }

#if (DCDC_OUTPUT_PERMISSION != 0U)
  ADC_AnalogWDGConfTypeDef watchdog = {0};

  watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  watchdog.Channel = ADC_CHANNEL_9;
  watchdog.ITMode = ENABLE;
  watchdog.HighThreshold = DCDC_VBUS_ANALOG_WATCHDOG_HIGH_RAW;
  watchdog.LowThreshold = 0U;
  watchdog.FilteringConfig = ADC_AWD_FILTERING_NONE;
  return HAL_ADC_AnalogWDGConfig(platform.config.vbus_adc, &watchdog) ==
         HAL_OK;
#else
  return true;
#endif
}

bool dcdc_platform_stm32_start(void)
{
  if (!platform.initialized)
  {
    return false;
  }

#if (DCDC_OUTPUT_PERMISSION != 0U)
  __HAL_ADC_CLEAR_FLAG(platform.config.vbus_adc, ADC_FLAG_AWD1);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
#endif

#if (DCDC_OUTPUTS_ARM_AT_BOOT != 0U)
  return dcdc_control_request_enable(true);
#else
  return true;
#endif
}

void dcdc_platform_stm32_service(uint32_t now_ms)
{
  uint16_t vbus_raw;
  uint16_t vref_raw;
  uint32_t vdda_mv;

  if (!platform.initialized)
  {
    return;
  }

  if (__HAL_ADC_GET_FLAG(platform.config.vbus_adc, ADC_FLAG_EOS) != 0U)
  {
    uint32_t vbus_mv = 0U;
    bool sample_valid;

    __HAL_ADC_CLEAR_FLAG(platform.config.vbus_adc, ADC_FLAG_EOS);
    vbus_raw = platform.config.adc_dma_values[
        platform.config.vbus_dma_index];
    vref_raw = platform.config.adc_dma_values[
        platform.config.vref_dma_index];
    vdda_mv = (vref_raw != 0U) ?
        __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_12B) :
        platform.config.fallback_vdda_mv;
    sample_valid = (vdda_mv <= UINT16_MAX) &&
        dcdc_vbus_adc_to_mv(vbus_raw, (uint16_t)vdda_mv, &vbus_mv);
    platform.sample = (dcdc_vbus_sample_t){
      .vbus_mv = vbus_mv,
      .captured_at_ms = now_ms,
      .valid = sample_valid,
    };
    platform.sample_available = true;
  }

  dcdc_control_service(platform.sample_available ? &platform.sample : NULL,
                       now_ms);
}

void dcdc_platform_stm32_systick_isr(uint32_t now_ms)
{
  dcdc_control_deadline_tick_isr(now_ms);
}

void ADC1_2_IRQHandler(void)
{
  ADC_TypeDef *adc = platform.initialized ?
      platform.config.vbus_adc->Instance : ADC1;

  if ((adc->ISR & ADC_ISR_AWD1) != 0U)
  {
    /* Clear TIM1 MOE before acknowledging the ADC event. */
    dcdc_control_overvoltage_isr();
    adc->ISR = ADC_ISR_AWD1;
  }
}
