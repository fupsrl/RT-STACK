/**
 ******************************************************************************
 * @file    engine_watchdog_stm32.h
 * @brief   Runtime independent-watchdog supervision for the engine loop.
 ******************************************************************************
 */
#ifndef ENGINE_WATCHDOG_STM32_H
#define ENGINE_WATCHDOG_STM32_H

#include <stdbool.h>

/* Start the hardware IWDG. Once started it cannot be stopped until reset.
 * Returns false only if the clock-domain configuration did not complete. */
bool engine_watchdog_stm32_start(void);

/* Call only after one complete, healthy foreground control-loop pass. */
void engine_watchdog_stm32_refresh(void);

#endif /* ENGINE_WATCHDOG_STM32_H */
