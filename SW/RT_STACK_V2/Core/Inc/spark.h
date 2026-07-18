/** @file spark.h  Safe, angle-scheduled ignition control. */
#ifndef SPARK_H
#define SPARK_H

#include "actuator_runtime.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool enabled;
  float advance_deg;
  uint32_t dwell_us;
} spark_command_t;

typedef enum
{
  SPARK_RESULT_OK = 0,
  SPARK_RESULT_UNKNOWN_CYLINDER,
  SPARK_RESULT_NONFINITE_COMMAND,
  SPARK_RESULT_COMMAND_OUT_OF_RANGE,
  SPARK_RESULT_CONFIGURATION_ERROR,
  SPARK_RESULT_EMERGENCY_LATCHED
} spark_result_t;

enum
{
  SPARK_FAULT_NONE               = 0U,
  SPARK_FAULT_CONFIGURATION      = (1UL << 0),
  SPARK_FAULT_BAD_COMMAND        = (1UL << 1),
  SPARK_FAULT_BAD_RUNTIME        = (1UL << 2),
  SPARK_FAULT_ANGLE_JUMP         = (1UL << 3),
  SPARK_FAULT_HARD_DEADLINE      = (1UL << 4),
  SPARK_FAULT_MISSED_EVENT       = (1UL << 5),
  SPARK_FAULT_TIMING_UNREACHABLE = (1UL << 6),
  SPARK_FAULT_EMERGENCY          = (1UL << 7)
};

/* Initializes calibration state and forces every physical IGN output low. */
void spark_init(void);

/* Explicit command API.  Invalid commands are rejected without changing the
 * previous command.  A changed command never modifies an active dwell event. */
spark_result_t spark_set_command(uint8_t cylinder_id,
                                 const spark_command_t *command);
spark_result_t spark_load_defaults(uint8_t cylinder_id, bool enable);

/* True only when at least one enabled ignition command needs 720-degree
 * phase.  Used by the engine gate before it marks outputs RUNNING. */
bool spark_phase_sync_required(void);

/* Main-context angle scheduler.  Pass the same coherent snapshot to spark and
 * injection once per foreground iteration. */
void spark_service(const actuator_runtime_snapshot_t *runtime);

/* Absolute-time safety service.  Call from the 100 us timer IRQ as well as
 * from foreground if convenient.  It only releases coils; it never starts one. */
void spark_service_deadlines(uint32_t now_tick);

void spark_disable_cylinder(uint8_t cylinder_id);
void spark_disable_mask(uint32_t cylinder_id_mask);

/* Fault/NMI-safe GPIO shutdown.  This latches the module off until init or an
 * explicit rearm, and does not depend on HAL state. */
void spark_emergency_off(void);
bool spark_rearm_after_emergency(void);

uint32_t spark_faults(void);
void spark_clear_faults(uint32_t fault_mask);

/* Compatibility helpers for existing application code.  spark_update only
 * changes a command; spark_service is still required to schedule outputs. */
void spark_update(uint8_t cylinder_id, float advance_deg, float dwell_ms);
void spark_off(int16_t cylinder_id_mask);

#endif /* SPARK_H */
