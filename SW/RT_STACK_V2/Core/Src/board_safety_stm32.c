/** @file board_safety_stm32.c  Early reset/boot output mitigation. */
#include "board_safety_stm32.h"

#include "main.h"

static void preload_low_output(GPIO_TypeDef *port, uint32_t pins)
{
  uint32_t mode_mask = 0U;
  uint32_t output_mode = 0U;
  uint32_t pin;

  /* Preload the OFF level before changing MODER, so the pin cannot pulse high
   * during the input/analog-to-output transition. */
  port->BRR = pins;
  for (pin = 0U; pin < 16U; ++pin)
  {
    uint32_t pin_bit = 1UL << pin;
    if ((pins & pin_bit) != 0U)
    {
      uint32_t shift = 2U * pin;
      mode_mask |= 3UL << shift;
      output_mode |= 1UL << shift;
    }
  }
  port->OTYPER &= ~pins;
  port->OSPEEDR &= ~mode_mask;
  port->PUPDR &= ~mode_mask;
  port->MODER = (port->MODER & ~mode_mask) | output_mode;
}

void board_force_actuator_pins_low_early(void)
{
  const uint32_t port_a = GPIO_PIN_0 | GPIO_PIN_2 |
                          BOOST910_Pin | BOOST1112_Pin |
                          BOOST78_Pin | BOOST56_Pin;
  const uint32_t port_b = GPIO_PIN_1 | IGN7_Pin | IGN10_Pin | IGN9_Pin;
  const uint32_t port_c = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7 |
                          GPIO_PIN_8 |
                          BOOST34_Pin | BOOST12_Pin | SEL910_Pin;
  const uint32_t port_d = IGN12_Pin | IGN11_Pin;
  const uint32_t port_e = SEL12_Pin | SEL34_Pin | SEL56_Pin | SEL78_Pin |
                          SEL1112_Pin | GPIO_PIN_13 | IGN2_Pin | IGN1_Pin | IGN4_Pin |
                          IGN3_Pin | IGN6_Pin | IGN5_Pin | IGN8_Pin;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
                  RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN |
                  RCC_AHB2ENR_GPIOEEN;
  __DSB();

  preload_low_output(GPIOA, port_a);
  preload_low_output(GPIOB, port_b);
  preload_low_output(GPIOC, port_c);
  preload_low_output(GPIOD, port_d);
  preload_low_output(GPIOE, port_e);
  __DSB();
}
