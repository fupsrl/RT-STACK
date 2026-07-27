/**
 ******************************************************************************
 * @file    trigger_capture_stm32.h
 * @brief   STM32G474 hybrid trigger capture backend.
 *
 * TIM2 is one 32-bit clock shared by every trigger input and the actuator
 * safety compare.  PA10/TMG5 uses TIM2_CH4 input capture; the other board
 * inputs use EXTI and read TIM2->CNT in their ISR.
 ******************************************************************************
 */
#ifndef TRIGGER_CAPTURE_STM32_H
#define TRIGGER_CAPTURE_STM32_H

#include "trigger_decoder.h"

#include <stdbool.h>
#include <stdint.h>

/* Return the TIM2 input clock after APB timer-clock multiplication. */
uint32_t trigger_capture_timestamp_hz(void);

/* Configure pins from the wheel list, start TIM2, and enable capture IRQs.
 * Call only during startup or reconfiguration with all actuators disabled.
 * BOTH-edge acquisition is supported only on PA10/TMG5 hardware capture;
 * EXTI-backed channels must select one polarity. */
bool trigger_capture_stm32_init(const trigger_decoder_config_t *config);

uint32_t trigger_capture_now(void);

/* Called by TIM2_IRQHandler in stm32g4xx_it.c. */
void trigger_capture_stm32_timer_irq(void);

/* Stop new trigger IRQs without resetting the timestamp counter. */
void trigger_capture_stm32_stop(void);

#endif /* TRIGGER_CAPTURE_STM32_H */
