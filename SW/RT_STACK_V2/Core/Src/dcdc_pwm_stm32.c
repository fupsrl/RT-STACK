/**
 ******************************************************************************
 * @file    dcdc_pwm_stm32.c
 * @brief   Hardware-synchronous 180-degree TIM1 PWM on PC0 and PE13.
 ******************************************************************************
 */
#include "dcdc_pwm_stm32.h"

#include <stddef.h>

typedef struct
{
  TIM_HandleTypeDef *timer;
  dcdc_pwm_config_t config;
  dcdc_pwm_plan_t plan;
  bool initialized;
  bool outputs_permitted;
  bool running;
  bool hardware_enabled;
} dcdc_pwm_stm32_context_t;

static dcdc_pwm_stm32_context_t pwm_context;

static uint32_t interrupt_lock(void)
{
  uint32_t previous_primask = __get_PRIMASK();

  __disable_irq();
  __DMB();
  return previous_primask;
}

static void interrupt_unlock(uint32_t previous_primask)
{
  __DMB();
  if (previous_primask == 0U)
  {
    __enable_irq();
  }
}

static void pins_force_low(void)
{
  const uint32_t pc0_shift = 0U;
  const uint32_t pe13_shift = 26U;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOEEN;
  __DSB();

  GPIOC->BRR = GPIO_PIN_0;
  GPIOE->BRR = GPIO_PIN_13;
  GPIOC->OTYPER &= ~GPIO_PIN_0;
  GPIOE->OTYPER &= ~GPIO_PIN_13;
  GPIOC->PUPDR &= ~(3UL << pc0_shift);
  GPIOE->PUPDR &= ~(3UL << pe13_shift);
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << pc0_shift)) |
                 (1UL << pc0_shift);
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << pe13_shift)) |
                 (1UL << pe13_shift);
  __DSB();
}

static void pins_select_tim1(void)
{
  const uint32_t pc0_shift = 0U;
  const uint32_t pe13_shift = 26U;
  const uint32_t pe13_af_shift = 20U;

  /* OFF is preloaded before changing either pin back to alternate function. */
  GPIOC->BRR = GPIO_PIN_0;
  GPIOE->BRR = GPIO_PIN_13;
  GPIOC->OTYPER &= ~GPIO_PIN_0;
  GPIOE->OTYPER &= ~GPIO_PIN_13;
  GPIOC->PUPDR &= ~(3UL << pc0_shift);
  GPIOE->PUPDR &= ~(3UL << pe13_shift);
  GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~(3UL << pc0_shift)) |
                   (2UL << pc0_shift);
  GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~(3UL << pe13_shift)) |
                   (2UL << pe13_shift);
  GPIOC->AFR[0] = (GPIOC->AFR[0] & ~0xFUL) | 2UL;
  GPIOE->AFR[1] = (GPIOE->AFR[1] & ~(0xFUL << pe13_af_shift)) |
                  (2UL << pe13_af_shift);
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << pc0_shift)) |
                 (2UL << pc0_shift);
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << pe13_shift)) |
                 (2UL << pe13_shift);
  __DSB();
}

void dcdc_pwm_stm32_emergency_off(void)
{
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
  {
    /* MOE is the first write: both timer outputs become inactive together. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC3E);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCR1 = 0U;
    TIM1->CCR3 = TIM1->ARR;
  }
  pins_force_low();
  pwm_context.running = false;
  pwm_context.hardware_enabled = false;
}

static void hardware_outputs_off(dcdc_pwm_stm32_context_t *pwm)
{
  TIM_TypeDef *tim = pwm->timer->Instance;

  tim->BDTR &= ~TIM_BDTR_MOE;
  tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC3E);
  tim->CR1 &= ~TIM_CR1_CEN;
  pins_force_low();
  pwm->hardware_enabled = false;
}

static bool pwm_set_duty(void *context, uint16_t duty_permille)
{
  dcdc_pwm_stm32_context_t *pwm = context;
  dcdc_pwm_plan_t next;
  uint32_t previous_cr1;
  uint32_t previous_primask;
  bool success = true;

  if ((pwm == NULL) || !pwm->initialized ||
      !dcdc_pwm_plan_compute(&pwm->config, duty_permille, &next))
  {
    return false;
  }

  /* The critical section prevents an off-only ISR from clearing CEN/MOE and
   * then having the foreground restore an old CR1 snapshot or restart TIM1.
   * A pending shutdown runs immediately when PRIMASK is restored. */
  previous_primask = interrupt_lock();
  if ((duty_permille != 0U) &&
      (!pwm->running || !pwm->outputs_permitted))
  {
    interrupt_unlock(previous_primask);
    return false;
  }

  /* Both CCR registers are preloaded. UDIS prevents a hardware update event
   * from transferring one new compare with one old compare between writes. */
  previous_cr1 = pwm->timer->Instance->CR1;
  pwm->timer->Instance->CR1 = previous_cr1 | TIM_CR1_UDIS;
  pwm->timer->Instance->CCR1 = next.phase_a_compare;
  pwm->timer->Instance->CCR3 = next.phase_b_compare;
  __DMB();
  pwm->timer->Instance->CR1 = previous_cr1;
  pwm->plan = next;

  /* PWM1/CCR=0 and PWM2/CCR=ARR can expose a one-timer-tick endpoint
   * compare. At a requested zero duty, disable the output stage completely
   * instead of relying on that boundary behavior. The logical controller
   * remains armed and may restart from a later nonzero command. */
  if (duty_permille == 0U)
  {
    hardware_outputs_off(pwm);
    interrupt_unlock(previous_primask);
    return true;
  }

  if (!pwm->hardware_enabled)
  {
    TIM_TypeDef *tim = pwm->timer->Instance;
    const uint32_t center_aligned_cr1 =
        tim->CR1 & ~(TIM_CR1_CEN | TIM_CR1_DIR);

    /* A previous zero-duty stop can occur on the down-counting half-cycle.
     * Briefly select edge-aligned/up while stopped, seed CNT=0 and issue UG,
     * then restore center-aligned mode so every restart has deterministic
     * phase order (PC0 first, PE13 one half-cycle later). */
    tim->CR1 = center_aligned_cr1 & ~TIM_CR1_CMS_Msk;
    tim->CNT = 0U;
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0U;
    tim->CR1 = center_aligned_cr1;
    pins_select_tim1();
    tim->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;
    tim->CR1 |= TIM_CR1_CEN;
    tim->BDTR |= TIM_BDTR_MOE;
    __DSB();
    pwm->hardware_enabled = true;
  }

  success = pwm->running && pwm->outputs_permitted &&
      pwm->hardware_enabled;
  interrupt_unlock(previous_primask);
  return success;
}

static bool pwm_start(void *context)
{
  dcdc_pwm_stm32_context_t *pwm = context;
  uint32_t previous_primask;

  if ((pwm == NULL) || !pwm->initialized || !pwm->outputs_permitted)
  {
    return false;
  }

  previous_primask = interrupt_lock();
  dcdc_pwm_stm32_emergency_off();
  /* emergency_off deliberately clears the global context state. Restore the
   * logical arm only after all hardware outputs have reached GPIO-low. */
  pwm->running = true;
  if (!pwm_set_duty(pwm, 0U))
  {
    pwm->running = false;
    interrupt_unlock(previous_primask);
    return false;
  }
  interrupt_unlock(previous_primask);
  return pwm->running;
}

static void pwm_stop(void *context)
{
  (void)context;
  dcdc_pwm_stm32_emergency_off();
}

static bool pwm_is_running(const void *context)
{
  const dcdc_pwm_stm32_context_t *pwm = context;
  if ((pwm == NULL) || !pwm->initialized || !pwm->running)
  {
    return false;
  }
  if (!pwm->hardware_enabled)
  {
    return true;
  }
  return ((pwm->timer->Instance->CR1 & TIM_CR1_CEN) != 0U) &&
         ((pwm->timer->Instance->BDTR & TIM_BDTR_MOE) != 0U) &&
         ((pwm->timer->Instance->CCER &
           (TIM_CCER_CC1E | TIM_CCER_CC3E)) ==
          (TIM_CCER_CC1E | TIM_CCER_CC3E));
}

static uint32_t pwm_critical_enter(void *context)
{
  (void)context;
  return interrupt_lock();
}

static void pwm_critical_exit(void *context, uint32_t token)
{
  (void)context;
  interrupt_unlock(token);
}

static const dcdc_pwm_driver_t pwm_driver = {
  .context = &pwm_context,
  .start = pwm_start,
  .set_duty_permille = pwm_set_duty,
  .stop = pwm_stop,
  .is_running = pwm_is_running,
  .critical_enter = pwm_critical_enter,
  .critical_exit = pwm_critical_exit,
};

bool dcdc_pwm_stm32_init(TIM_HandleTypeDef *timer,
                          const dcdc_pwm_config_t *config,
                          bool outputs_permitted)
{
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef channel = {0};
  TIM_BreakDeadTimeConfigTypeDef break_config = {0};
  dcdc_pwm_plan_t initial_plan;

  dcdc_pwm_stm32_emergency_off();
  pwm_context = (dcdc_pwm_stm32_context_t){0};
  if ((timer == NULL) || (config == NULL) ||
      !dcdc_pwm_plan_compute(config, 0U, &initial_plan))
  {
    return false;
  }

  if (outputs_permitted &&
      ((config->minimum_active_duty_permille == 0U) ||
       (config->maximum_duty_permille == 0U) ||
       (config->minimum_on_time_ns == 0U) ||
       (config->minimum_off_time_ns == 0U)))
  {
    return false;
  }

  timer->Instance = TIM1;
  timer->Init.Prescaler = initial_plan.prescaler;
  timer->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  timer->Init.Period = initial_plan.auto_reload;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  /* One UEV per complete up/down PWM cycle instead of at both extrema. */
  timer->Init.RepetitionCounter = 1U;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(timer) != HAL_OK)
  {
    return false;
  }

  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if ((HAL_TIM_ConfigClockSource(timer, &clock) != HAL_OK) ||
      (HAL_TIM_PWM_Init(timer) != HAL_OK))
  {
    return false;
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(timer, &master) != HAL_OK)
  {
    return false;
  }

  channel.Pulse = 0U;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;
  channel.OCIdleState = TIM_OCIDLESTATE_RESET;
  channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  channel.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) != HAL_OK)
  {
    return false;
  }
  channel.OCMode = TIM_OCMODE_PWM2;
  channel.Pulse = initial_plan.auto_reload;
  if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_3) != HAL_OK)
  {
    return false;
  }

  break_config.OffStateRunMode = TIM_OSSR_ENABLE;
  break_config.OffStateIDLEMode = TIM_OSSI_ENABLE;
  break_config.LockLevel = TIM_LOCKLEVEL_OFF;
  break_config.DeadTime = 0U;
  break_config.BreakState = TIM_BREAK_DISABLE;
  break_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  break_config.BreakFilter = 0U;
  break_config.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  break_config.Break2State = TIM_BREAK2_DISABLE;
  break_config.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  break_config.Break2Filter = 0U;
  break_config.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  break_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(timer, &break_config) != HAL_OK)
  {
    return false;
  }

  /* Cube's post-init selects AF2. Return to explicit GPIO-low state until the
   * controller has confirmed fresh VBUS samples and requests startup. */
  HAL_TIM_MspPostInit(timer);
  dcdc_pwm_stm32_emergency_off();

  pwm_context.timer = timer;
  pwm_context.config = *config;
  pwm_context.plan = initial_plan;
  pwm_context.outputs_permitted = outputs_permitted;
  pwm_context.initialized = true;
  return true;
}

const dcdc_pwm_driver_t *dcdc_pwm_stm32_driver(void)
{
  return &pwm_driver;
}

void dcdc_pwm_stm32_get_plan(dcdc_pwm_plan_t *plan)
{
  if (plan != NULL)
  {
    *plan = pwm_context.plan;
  }
}
