/** @file board_safety_stm32.h  Earliest-possible board output mitigation. */
#ifndef BOARD_SAFETY_STM32_H
#define BOARD_SAFETY_STM32_H

/* Preload and configure every coil/injector control net low.  This function
 * uses registers only and is safe to call before HAL_Init(). */
void board_force_actuator_pins_low_early(void);

#endif /* BOARD_SAFETY_STM32_H */
