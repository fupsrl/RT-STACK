/**
 ******************************************************************************
 * @file    engine_runtime.h
 * @brief   Live engine state and shared time/angle helpers.
 *
 * The trigger decoder (in main.c) publishes the current position/speed here;
 * the spark and injection modules consume them. Also provides the DWT cycle
 * time base and small angle helpers shared by both modules.
 ******************************************************************************
 */
#ifndef ENGINE_RUNTIME_H
#define ENGINE_RUNTIME_H

#include "main.h"        /* DWT, SystemCoreClock */
#include <stdint.h>
#include <stdbool.h>

/* Live engine state, updated every main loop from the trigger decoder.
 * engine_angle: 0..720 once the phase is known, otherwise 0..360. */
extern volatile float engine_angle;
extern volatile float engine_rpm;

/* Free-running cycle counter (DWT): 32-bit @ SystemCoreClock (170 MHz). */
static inline uint32_t cycle_now(void)
{
  return DWT->CYCCNT;
}

/* Convert an interval in cycles to microseconds. */
static inline float cycles_to_us(uint32_t cycles)
{
  return (float)cycles / ((float)SystemCoreClock / 1000000.0f);
}

/* Wrap an angle into [0, 720). */
static inline float wrap720(float a)
{
  while (a >= 720.0f) a -= 720.0f;
  while (a <  0.0f)   a += 720.0f;
  return a;
}

/* true if 'target' is crossed going forward from 'prev' (exclusive) to 'curr'
 * (inclusive), with 720 -> 0 wrap. */
static inline bool angle_crossed(float target, float prev, float curr)
{
  if (prev == curr) return false;
  if (prev <  curr) return (target > prev) && (target <= curr);
  return (target > prev) || (target <= curr);
}

#endif /* ENGINE_RUNTIME_H */
