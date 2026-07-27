/**
 ******************************************************************************
 * @file    engine_control.h
 * @brief   Engine state machine joining trigger decoding and actuators.
 *
 * Capture ISRs only enqueue timestamped edges.  The foreground service drains
 * every queued event, polls the decoder, then presents one coherent snapshot
 * to ignition and injection.  Safety deadline interrupts may only move an
 * already active output toward OFF.
 ******************************************************************************
 */
#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include "trigger_decoder.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ENGINE_MODE_DISABLED = 0,
  ENGINE_MODE_SYNCING,
  ENGINE_MODE_RUNNING,
  ENGINE_MODE_FAULT,
  ENGINE_MODE_INJECTOR_TEST
} engine_mode_t;

/** User request for one stationary, periodically actuated physical injector. */
typedef struct
{
  uint8_t injector_output;  /* Physical INJ1..INJ12 output number. */
  uint32_t period_ms;       /* Time between pulse starts. */
  uint32_t pulse_width_us;  /* Calibrated peak/hold on-time. */
} engine_injector_test_config_t;

typedef enum
{
  ENGINE_INJECTOR_TEST_OK = 0,
  ENGINE_INJECTOR_TEST_DISABLED,
  ENGINE_INJECTOR_TEST_NOT_INITIALIZED,
  ENGINE_INJECTOR_TEST_INVALID_ARGUMENT,
  ENGINE_INJECTOR_TEST_UNMAPPED_OUTPUT,
  ENGINE_INJECTOR_TEST_CALIBRATION_LOCKED,
  ENGINE_INJECTOR_TEST_ENGINE_BUSY,
  ENGINE_INJECTOR_TEST_ENGINE_NOT_STATIONARY,
  ENGINE_INJECTOR_TEST_FAULT_LATCHED,
  ENGINE_INJECTOR_TEST_PAIR_BUSY,
  ENGINE_INJECTOR_TEST_HARDWARE_ERROR
} engine_injector_test_result_t;

typedef struct
{
  bool active;
  uint8_t injector_output;
  uint32_t period_ms;
  uint32_t pulse_width_us;
  uint32_t pulse_count;
  uint32_t skipped_period_count;
  engine_injector_test_result_t last_result;
} engine_injector_test_state_t;

enum
{
  ENGINE_FAULT_NONE                 = 0U,
  ENGINE_FAULT_CONFIGURATION        = (1UL << 0),
  ENGINE_FAULT_CAPTURE_QUEUE        = (1UL << 1),
  ENGINE_FAULT_CAPTURE_OVERRUN      = (1UL << 2),
  ENGINE_FAULT_TRIGGER_SYNC_LOST    = (1UL << 3),
  ENGINE_FAULT_IGNITION             = (1UL << 4),
  ENGINE_FAULT_INJECTION            = (1UL << 5),
  ENGINE_FAULT_CLOCK                = (1UL << 6),
  ENGINE_FAULT_PLATFORM             = (1UL << 7)
};

typedef struct
{
  engine_mode_t mode;
  float angle_deg;
  float rpm;
  uint32_t timestamp;
  uint32_t timestamp_hz;
  uint32_t sync_epoch;
  bool crank_synced;
  bool phase_synced;
  bool outputs_requested;
  bool outputs_enabled;

  trigger_config_error_t trigger_config_error;
  trigger_loss_reason_t last_trigger_loss;
  uint32_t trigger_latched_faults;
  uint32_t ignition_faults;
  uint32_t injection_faults;
  uint32_t latched_faults;
  uint32_t capture_dropped_events;
  uint32_t capture_overruns;
} engine_control_state_t;

/** Initialize the queue, decoder, output drivers, and default commands.
 * trigger_config and everything it references must remain immutable and valid
 * for the lifetime of the controller. */
bool engine_control_init(const trigger_decoder_config_t *trigger_config,
                         uint32_t timestamp_hz);

/** Drain all trigger records and service the state machine and actuators. */
void engine_control_service(void);

/** ISR entry for EXTI or timer-captured edges. */
bool engine_control_record_trigger_isr(uint8_t channel,
                                       trigger_edge_t edge,
                                       uint32_t timestamp);

/** Report a hardware timer overcapture or equivalent backend data loss. */
void engine_control_capture_overrun_isr(uint8_t channel);

/** 100 us absolute-time safety service.  It never starts an output. */
void engine_control_deadline_isr(uint32_t now_timestamp);

/** Fault-handler entry.  fault_bit must be one of ENGINE_FAULT_* above. */
void engine_control_emergency_fault_isr(uint32_t fault_bit);

/** Request or revoke actuator permission.  Enabling never bypasses sync. */
bool engine_control_request_outputs(bool enable);

/**
 * Arm a stationary-engine injector bench test.
 *
 * This foreground API is available only when explicitly enabled in
 * engine_config.h and current calibration is acknowledged. Normal outputs
 * must be disarmed, the trigger inputs must be quiet, and no fault may be
 * latched. The first pulse occurs after one complete period.
 */
engine_injector_test_result_t engine_control_start_injector_test(
    const engine_injector_test_config_t *config);

/** Stop a bench test immediately; safe and idempotent between pulses. */
void engine_control_stop_injector_test(void);

/** Copy the current bench-test counters and most recent API result. */
void engine_control_get_injector_test_state(
    engine_injector_test_state_t *state);

/**
 * Clear recoverable faults only while output permission is revoked.
 * Synchronization and queued events are deliberately discarded.
 */
bool engine_control_clear_faults(void);

/** Copy the current diagnostic state coherently. */
void engine_control_get_state(engine_control_state_t *state);

#endif /* ENGINE_CONTROL_H */
