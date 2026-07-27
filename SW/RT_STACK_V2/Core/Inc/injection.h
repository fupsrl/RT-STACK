/** @file injection.h  Safe, angle-scheduled peak-and-hold injection control. */
#ifndef INJECTION_H
#define INJECTION_H

#include "actuator_runtime.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool enabled;
  float start_btdc_deg;
  uint32_t duration_us;
} injection_command_t;

typedef enum
{
  INJECTION_RESULT_OK = 0,
  INJECTION_RESULT_UNKNOWN_CYLINDER,
  INJECTION_RESULT_NONFINITE_COMMAND,
  INJECTION_RESULT_COMMAND_OUT_OF_RANGE,
  INJECTION_RESULT_CONFIGURATION_ERROR,
  INJECTION_RESULT_EMERGENCY_LATCHED
} injection_result_t;

enum
{
  INJECTION_FAULT_NONE          = 0U,
  INJECTION_FAULT_CONFIGURATION = (1UL << 0),
  INJECTION_FAULT_BAD_COMMAND   = (1UL << 1),
  INJECTION_FAULT_BAD_RUNTIME   = (1UL << 2),
  INJECTION_FAULT_ANGLE_JUMP    = (1UL << 3),
  INJECTION_FAULT_PAIR_BUSY     = (1UL << 4),
  INJECTION_FAULT_HARDWARE      = (1UL << 5),
  INJECTION_FAULT_MISSED_EVENT  = (1UL << 6),
  INJECTION_FAULT_EMERGENCY     = (1UL << 7),
  /* Two events for the same shared driver crossed in one scheduler sample.
   * Neither is started; this is independent of firing-table order. */
  INJECTION_FAULT_PAIR_COLLISION = (1UL << 8),
  /* A trigger edge arrived while the stationary-engine injector test was
   * armed. The test is immediately cancelled and emergency-latched OFF. */
  INJECTION_FAULT_DEBUG_INTERLOCK = (1UL << 9)
};

/* Starts the six threshold DAC channels and leaves every injector off. */
void injection_init(void);

/* Invalid commands are rejected atomically.  Timing changes affect the next
 * pulse only; an active pulse retains its latched duration and drive profile. */
injection_result_t injection_set_command(uint8_t cylinder_id,
                                         const injection_command_t *command);
injection_result_t injection_load_defaults(uint8_t cylinder_id, bool enable);

/* True only when an enabled injection command requires cam phase. */
bool injection_phase_sync_required(void);

void injection_service(const actuator_runtime_snapshot_t *runtime);

/* 100 us timer-IRQ service: performs BOOST->HOLD and the absolute pulse-off
 * deadline.  It never starts a new injector event. */
void injection_service_deadlines(uint32_t now_tick);

void injection_disable_cylinder(uint8_t cylinder_id);
void injection_disable_mask(uint32_t cylinder_id_mask);

/* Direct-register emergency path; latches all pairs off until rearmed. */
void injection_emergency_off(void);
bool injection_rearm_after_emergency(void);

uint32_t injection_faults(void);
void injection_clear_faults(uint32_t fault_mask);

/* Compatibility command helpers.  injection_service remains mandatory. */
void injection_update(uint8_t cylinder_id, float start_btdc_deg,
                      float duration_ms);
void injection_off(int16_t cylinder_id_mask);

#endif /* INJECTION_H */
