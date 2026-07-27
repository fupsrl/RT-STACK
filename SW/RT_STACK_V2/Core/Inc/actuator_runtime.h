/**
 ******************************************************************************
 * @file    actuator_runtime.h
 * @brief   Coherent input snapshot shared by the actuator schedulers.
 ******************************************************************************
 */
#ifndef ACTUATOR_RUNTIME_H
#define ACTUATOR_RUNTIME_H

#include "engine_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float angle_deg;          /* normalized to [0, cycle_deg) */
  float rpm;
  /* Angle domain used by this snapshot: 360 for crank-only scheduling or
   * 720 for phased sequential scheduling.  A domain change is a scheduler
   * discontinuity and causes active outputs to be released/re-latched. */
  float cycle_deg;
  uint32_t now_tick;        /* free-running unsigned timebase */
  uint32_t tick_hz;
  uint32_t sync_epoch;      /* increment on every loss/reacquisition of sync */
  bool crank_synced;
  bool phase_synced;
  bool outputs_enabled;     /* final permission from the engine state machine */
} actuator_runtime_snapshot_t;

static inline bool actuator_snapshot_valid(const actuator_runtime_snapshot_t *s,
                                           bool require_phase)
{
  bool cycle_supported;

  if (s == NULL)
  {
    return false;
  }
  cycle_supported = isfinite(s->cycle_deg) &&
                    ((s->cycle_deg == ENGINE_CRANK_CYCLE_DEG) ||
                     (s->cycle_deg == ENGINE_CYCLE_DEG));

  return s->outputs_enabled && s->crank_synced && cycle_supported &&
         (!require_phase ||
          (s->phase_synced && (s->cycle_deg == ENGINE_CYCLE_DEG))) &&
         isfinite(s->angle_deg) && isfinite(s->rpm) &&
         (s->angle_deg >= 0.0f) && (s->angle_deg < s->cycle_deg) &&
         (s->rpm >= ENGINE_MIN_SCHEDULING_RPM) &&
         (s->rpm <= ENGINE_MAX_SCHEDULING_RPM) && (s->tick_hz != 0U);
}

static inline float actuator_wrap_angle(float angle_deg, float cycle_deg)
{
  /* All callers first range-check calibration and commands, so at most a few
   * iterations are possible and no libm remainder edge cases are introduced. */
  while (angle_deg >= cycle_deg)
  {
    angle_deg -= cycle_deg;
  }
  while (angle_deg < 0.0f)
  {
    angle_deg += cycle_deg;
  }
  return angle_deg;
}

static inline float actuator_forward_angle(float previous_deg, float current_deg,
                                           float cycle_deg)
{
  float delta = current_deg - previous_deg;
  return (delta >= 0.0f) ? delta : (delta + cycle_deg);
}

/* Target is excluded at previous_deg and included at current_deg. */
static inline bool actuator_angle_crossed(float target_deg,
                                          float previous_deg,
                                          float current_deg)
{
  if (previous_deg == current_deg)
  {
    return false;
  }
  if (previous_deg < current_deg)
  {
    return (target_deg > previous_deg) && (target_deg <= current_deg);
  }
  return (target_deg > previous_deg) || (target_deg <= current_deg);
}

/* Valid for intervals shorter than half the 32-bit timebase period. */
static inline bool actuator_deadline_reached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0);
}

static inline bool actuator_deadline_from_us(uint32_t now,
                                             uint32_t duration_us,
                                             uint32_t tick_hz,
                                             uint32_t *deadline)
{
  uint64_t ticks;

  if ((deadline == NULL) || (tick_hz == 0U))
  {
    return false;
  }
  ticks = (((uint64_t)duration_us * (uint64_t)tick_hz) + 999999ULL) /
          1000000ULL;
  if ((ticks == 0ULL) || (ticks > 0x7FFFFFFFULL))
  {
    return false;
  }
  *deadline = now + (uint32_t)ticks;
  return true;
}

#endif /* ACTUATOR_RUNTIME_H */
