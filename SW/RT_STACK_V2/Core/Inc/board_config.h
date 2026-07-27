/**
 ******************************************************************************
 * @file    board_config.h
 * @brief   Board-revision feature gates and known routing constraints.
 ******************************************************************************
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* The current PCB connects PA11 to the TJA1051 TXD input and PA12 to its RXD
 * output.  STM32G474 FDCAN1 has the opposite fixed roles: PA11=RX, PA12=TX.
 * Enabling FDCAN on an unmodified board would therefore make PA12 contend with
 * the transceiver output.  Keep this 0 until the two logic-side nets have been
 * physically swapped (or a corrected PCB revision is used), then set it to 1.
 */
#define BOARD_FDCAN1_TRANSCEIVER_WIRING_FIXED  0U

/* Smallest available hysteresis is a conservative starting point for the six
 * injector current comparators.  Confirm it against measured sensor noise and
 * the required peak/hold ripple on the real driver before loaded operation. */
#define BOARD_INJECTOR_COMPARATOR_HYSTERESIS   COMP_HYSTERESIS_10MV

/* Runtime independent watchdog: /64 with reload 499 is nominally 1.0 s at
 * the STM32G4's typical 32 kHz LSI frequency. LSI tolerance makes this an
 * approximate safety timeout, not a precision interval. The IWDG is not frozen
 * when a debugger halts the core, so a debug halt will deliberately reset it. */
#define BOARD_IWDG_PRESCALER_BITS               IWDG_PR_PR_2
#define BOARD_IWDG_RELOAD_VALUE                 499U

#endif /* BOARD_CONFIG_H */
