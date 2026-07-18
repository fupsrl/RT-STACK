/**
 ******************************************************************************
 * @file    injection_internal.h
 * @brief   Engine-controller-only stationary injector test interface.
 *
 * This header intentionally lives beside the implementation, outside the
 * public Core/Inc API. Application code must use the interlocked
 * engine_control_* injector-test API instead of these actuator primitives.
 ******************************************************************************
 */
#ifndef INJECTION_INTERNAL_H
#define INJECTION_INTERNAL_H

#include "injection.h"

typedef enum
{
  INJECTION_DEBUG_OK = 0,
  INJECTION_DEBUG_NOT_CONFIGURED,
  INJECTION_DEBUG_CALIBRATION_REQUIRED,
  INJECTION_DEBUG_INVALID_OUTPUT,
  INJECTION_DEBUG_UNMAPPED_OUTPUT,
  INJECTION_DEBUG_INVALID_TIMING,
  INJECTION_DEBUG_EMERGENCY_LATCHED,
  INJECTION_DEBUG_PAIR_BUSY,
  INJECTION_DEBUG_HARDWARE_ERROR
} injection_debug_result_t;

typedef struct
{
  bool active;
  uint8_t injector_output;
  uint32_t period_us;
  uint32_t pulse_width_us;
  uint32_t pulse_count;
  uint32_t skipped_period_count;
} injection_debug_state_t;

/* These primitives contain no engine-stationary or spark interlock. Only
 * engine_control.c is authorized to use them in firmware. */
injection_debug_result_t injection_debug_start(uint8_t injector_output,
                                               uint32_t period_us,
                                               uint32_t pulse_width_us,
                                               uint32_t now_tick,
                                               uint32_t tick_hz);
injection_debug_result_t injection_debug_service(uint32_t now_tick);
void injection_debug_stop(void);
void injection_debug_abort_isr(void);
bool injection_debug_active(void);
void injection_debug_get_state(injection_debug_state_t *state);

#endif /* INJECTION_INTERNAL_H */
