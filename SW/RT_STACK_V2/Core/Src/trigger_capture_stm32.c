/** @file trigger_capture_stm32.c  STM32G474 trigger timestamp backend. */
#include "trigger_capture_stm32.h"

#include "engine_control.h"
#include "main.h"

#include <limits.h>
#include <string.h>

#define ACTUATOR_DEADLINE_PERIOD_US  100U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} trigger_pin_t;

/* Indexed by the electrical TMG channel number. */
static const trigger_pin_t trigger_pins[TRIGGER_CHANNEL_COUNT] =
{
  { NULL, 0U },
  { TMG_OUT1_GPIO_Port, TMG_OUT1_Pin },
  { TMG_OUT2_GPIO_Port, TMG_OUT2_Pin },
  { TMG_OUT3_GPIO_Port, TMG_OUT3_Pin },
  { TMG_OUT4_GPIO_Port, TMG_OUT4_Pin },
  { TMG_OUT5_GPIO_Port, TMG_OUT5_Pin },
  { TMG_OUT6_GPIO_Port, TMG_OUT6_Pin },
  { TMG_OUT7_GPIO_Port, TMG_OUT7_Pin },
  { TMG_OUT8_GPIO_Port, TMG_OUT8_Pin },
  { TMG_OUT9_GPIO_Port, TMG_OUT9_Pin },
};

static trigger_edge_t channel_edge[TRIGGER_CHANNEL_COUNT];
static bool channel_enabled[TRIGGER_CHANNEL_COUNT];
static uint32_t exti_mask;
static uint32_t deadline_period_ticks;
static bool capture_started;

static bool capture_is_started(void)
{
  return __atomic_load_n(&capture_started, __ATOMIC_ACQUIRE);
}

static void capture_publish_started(bool started)
{
  __atomic_store_n(&capture_started, started, __ATOMIC_RELEASE);
}

uint32_t trigger_capture_timestamp_hz(void)
{
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();

  /* STM32 timers receive twice PCLK when the APB prescaler is not DIV1. */
  if ((RCC->CFGR & RCC_CFGR_PPRE1_Msk) != RCC_CFGR_PPRE1_DIV1)
  {
    timer_hz *= 2U;
  }
  return timer_hz;
}

static uint32_t gpio_mode_for_edge(trigger_edge_t edge)
{
  if (edge == TRIGGER_EDGE_FALLING)
  {
    return GPIO_MODE_IT_FALLING;
  }
  if (edge == TRIGGER_EDGE_BOTH)
  {
    return GPIO_MODE_IT_RISING_FALLING;
  }
  return GPIO_MODE_IT_RISING;
}

static trigger_edge_t sampled_edge(uint8_t channel)
{
  trigger_edge_t configured = channel_edge[channel];

  if (configured != TRIGGER_EDGE_BOTH)
  {
    return configured;
  }
  return (HAL_GPIO_ReadPin(trigger_pins[channel].port,
                           trigger_pins[channel].pin) == GPIO_PIN_SET) ?
         TRIGGER_EDGE_RISING : TRIGGER_EDGE_FALLING;
}

static bool build_channel_map(const trigger_decoder_config_t *config)
{
  uint8_t wheel;

  if ((config == NULL) || (config->wheels == NULL) ||
      (config->wheel_count == 0U) ||
      (config->wheel_count > TRIGGER_MAX_WHEELS))
  {
    return false;
  }

  memset(channel_enabled, 0, sizeof(channel_enabled));
  for (wheel = 0U; wheel < config->wheel_count; ++wheel)
  {
    uint8_t channel = config->wheels[wheel].input_channel;
    trigger_edge_t edge = config->wheels[wheel].active_edge;

    if ((channel == 0U) || (channel >= TRIGGER_CHANNEL_COUNT) ||
        channel_enabled[channel] ||
        ((edge != TRIGGER_EDGE_RISING) &&
         (edge != TRIGGER_EDGE_FALLING) &&
         (edge != TRIGGER_EDGE_BOTH)) ||
        /* An EXTI pending bit cannot preserve two opposite edges or their
         * order. BOTH is therefore accepted only by PA10/TIM2_CH4 capture. */
        ((edge == TRIGGER_EDGE_BOTH) && (channel != 5U)))
    {
      return false;
    }
    channel_enabled[channel] = true;
    channel_edge[channel] = edge;
  }
  return true;
}

static void configure_trigger_gpio(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t channel;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Remove CubeMX's fixed-edge EXTI/output setup first.  The user wheel list
   * below is the single source of truth for enabled channels and polarity. */
  for (channel = 1U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    HAL_GPIO_DeInit(trigger_pins[channel].port, trigger_pins[channel].pin);
  }

  exti_mask = 0U;
  for (channel = 1U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    gpio.Pin = trigger_pins[channel].pin;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0U;

    if (!channel_enabled[channel])
    {
      gpio.Mode = GPIO_MODE_INPUT;
    }
    else if (channel == 5U)
    {
      /* PA10 is the only TMG input routed to a free 32-bit capture channel. */
      gpio.Mode = GPIO_MODE_AF_PP;
      gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      gpio.Alternate = GPIO_AF10_TIM2;
    }
    else
    {
      gpio.Mode = gpio_mode_for_edge(channel_edge[channel]);
      exti_mask |= gpio.Pin;
    }
    HAL_GPIO_Init(trigger_pins[channel].port, &gpio);
  }
}

static void configure_tim2(void)
{
  uint32_t timer_hz = trigger_capture_timestamp_hz();

  __HAL_RCC_TIM2_CLK_ENABLE();
  __DSB();

  TIM2->CR1 = 0U;
  TIM2->CR2 = 0U;
  TIM2->SMCR = 0U;
  TIM2->DIER = 0U;
  TIM2->CCER = 0U;
  TIM2->CCMR1 = 0U;
  TIM2->CCMR2 = 0U;
  TIM2->PSC = 0U;
  TIM2->ARR = UINT32_MAX;
  TIM2->CNT = 0U;

  deadline_period_ticks =
      (timer_hz + (1000000U / ACTUATOR_DEADLINE_PERIOD_US) - 1U) /
      (1000000U / ACTUATOR_DEADLINE_PERIOD_US);
  if (deadline_period_ticks == 0U)
  {
    deadline_period_ticks = 1U;
  }
  TIM2->CCR1 = deadline_period_ticks;

  if (channel_enabled[5U])
  {
    /* CC4 maps directly to TI4.  Polarity follows the user wheel config. */
    TIM2->CCMR2 = TIM_CCMR2_CC4S_0;
    if (channel_edge[5U] == TRIGGER_EDGE_FALLING)
    {
      TIM2->CCER = TIM_CCER_CC4P;
    }
    else if (channel_edge[5U] == TRIGGER_EDGE_BOTH)
    {
      TIM2->CCER = TIM_CCER_CC4P | TIM_CCER_CC4NP;
    }
    TIM2->CCER |= TIM_CCER_CC4E;
  }

  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->DIER = TIM_DIER_CC1IE |
               (channel_enabled[5U] ? TIM_DIER_CC4IE : 0U);
}

bool trigger_capture_stm32_init(const trigger_decoder_config_t *config)
{
  /* This is a startup/reconfiguration API and requires outputs disabled.
   * Publish the closed gate and mask every producer before changing the shared
   * channel map. CC1 is also stopped briefly because TIM2 is reinitialized. */
  capture_publish_started(false);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  if ((RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) != 0U)
  {
    TIM2->DIER = 0U;
    TIM2->CCER = 0U;
    TIM2->CR1 = 0U;
  }

  if (!build_channel_map(config) || (trigger_capture_timestamp_hz() == 0U))
  {
    return false;
  }

  configure_trigger_gpio();
  configure_tim2();

  if (exti_mask != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(exti_mask);
  }
  NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  NVIC_ClearPendingIRQ(TIM2_IRQn);

  /* Equal preemption priority means the per-channel ISR producers cannot
   * preempt one another.  SysTick/ADC/CAN remain lower priority. */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1U, 0U);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 0U);
  /* The common timestamp clock must exist before the capture gate is visible
   * or any producer IRQ can run. An edge arriving after CEN is retained as a
   * peripheral pending flag until its IRQ is enabled below. */
  TIM2->CR1 = TIM_CR1_CEN;
  capture_publish_started(true);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  if ((exti_mask & (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                    GPIO_PIN_8 | GPIO_PIN_9)) != 0U)
  {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  if ((exti_mask & (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                    GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) != 0U)
  {
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
  return true;
}

uint32_t trigger_capture_now(void)
{
  return TIM2->CNT;
}

void trigger_capture_stm32_timer_irq(void)
{
  uint32_t status = TIM2->SR;

  if ((status & TIM_SR_CC1IF) != 0U)
  {
    uint32_t now = TIM2->CNT;
    uint32_t next = TIM2->CCR1 + deadline_period_ticks;

    TIM2->SR = ~TIM_SR_CC1IF;
    if ((int32_t)(now - next) >= 0)
    {
      next = now + deadline_period_ticks;
    }
    TIM2->CCR1 = next;
    engine_control_deadline_isr(now);
  }

  if ((status & TIM_SR_CC4IF) != 0U)
  {
    /* SR was read above; reading CCR4 now acknowledges CC4IF. Do not write the
     * flag afterward: that could erase a new edge arriving after this read. */
    uint32_t captured_timestamp = TIM2->CCR4;
    if (capture_is_started() && channel_enabled[5U])
    {
      (void)engine_control_record_trigger_isr(
          5U, sampled_edge(5U), captured_timestamp);
    }
  }

  /* Read the live flag, not only the entry snapshot: a second edge may have
   * overcaptured CCR4 while the first capture was being consumed above. */
  if ((TIM2->SR & TIM_SR_CC4OF) != 0U)
  {
    TIM2->SR = ~TIM_SR_CC4OF;
    engine_control_capture_overrun_isr(5U);
  }
}

void trigger_capture_stm32_stop(void)
{
  capture_publish_started(false);
  /* Keep CC1 and TIM2_IRQn alive: actuator hard-off deadlines must remain
   * enforceable even after trigger acquisition is stopped. */
  TIM2->DIER &= ~TIM_DIER_CC4IE;
  TIM2->CCER &= ~TIM_CCER_CC4E;
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/** HAL EXTI callback for all non-PA10 trigger channels. */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  uint8_t channel;

  if (!capture_is_started())
  {
    return;
  }
  for (channel = 1U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    if ((channel != 5U) && channel_enabled[channel] &&
        (trigger_pins[channel].pin == gpio_pin))
    {
      (void)engine_control_record_trigger_isr(
          channel, sampled_edge(channel), TIM2->CNT);
      return;
    }
  }
}
