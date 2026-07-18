/**
 ******************************************************************************
 * @file    engine_config.h
 * @brief   User-editable engine and actuator calibration.
 *
 * This is the only file that normally needs to change when the engine changes.
 * Physical STM32 pins and peripherals live in board_output_config.h.
 *
 * Angles use the 720 degree four-stroke cycle.  Each cylinder has an explicit
 * firing TDC, which makes even-fire and odd-fire engines equally readable.
 ******************************************************************************
 */
#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

#include <stdint.h>

/* ============================= Engine layout ============================= */

#define ENGINE_CYCLE_DEG                    720.0f
#define ENGINE_CRANK_CYCLE_DEG              360.0f
#define ENGINE_CYLINDER_COUNT               4U

/* ========================== Runtime permissions ========================== */

/* Keep this at 0 while developing or testing with unprotected loads.  Setting
 * it to 1 only requests output operation: valid trigger configuration, the
 * required crank/cam synchronization, and a fault-free runtime are still
 * mandatory before any coil or injector can be energized. */
#define ENGINE_OUTPUTS_ARM_AT_BOOT             0U

/* Load each cylinder's default command at startup.  These switches make it
 * easy to build ignition-only or injection-only applications without editing
 * the scheduler code. */
#define ENGINE_DEFAULT_IGNITION_ENABLED        1U
#define ENGINE_DEFAULT_INJECTION_ENABLED       0U

/* The fitted ACS722 current sensors are bidirectional and their zero point,
 * polarity, sensitivity, supply, and per-channel offset must be measured on
 * the assembled board.  Raw DAC codes below are examples, not load-safe
 * current limits.  Injection commands that enable an output are rejected
 * until the measured codes have been entered and this acknowledgement is 1. */
#ifndef INJECTION_CURRENT_CALIBRATION_VALID
#define INJECTION_CURRENT_CALIBRATION_VALID     0U
#endif

/* Periodic injector actuation is a stationary-engine bench feature, never a
 * normal runtime mode.  Keep it compiled but locked at 0 in production.  It
 * also remains unavailable until INJECTION_CURRENT_CALIBRATION_VALID is 1. */
#ifndef ENGINE_INJECTOR_TEST_API_ENABLED
#define ENGINE_INJECTOR_TEST_API_ENABLED         0U
#endif

/* Additional bench-test limits. The requested pulse must also satisfy the
 * mapped cylinder's normal minimum/maximum injection duration. */
#define ENGINE_INJECTOR_TEST_MIN_OFF_US        2000U
#define ENGINE_INJECTOR_TEST_MAX_PERIOD_MS    10000U
#define ENGINE_INJECTOR_TEST_MAX_DUTY_PERMILLE  250U
/* After any observed trigger edge, require this much quiet time before a
 * stationary test can be armed. This closes the gap where an unsynchronized
 * but still-rotating wheel might otherwise report zero RPM. */
#define ENGINE_INJECTOR_TEST_QUIET_TIME_MS      1000U

#if (ENGINE_INJECTOR_TEST_API_ENABLED > 1U)
#error "ENGINE_INJECTOR_TEST_API_ENABLED must be 0 or 1"
#endif
#if ((ENGINE_INJECTOR_TEST_MIN_OFF_US == 0U) || \
     (ENGINE_INJECTOR_TEST_MAX_PERIOD_MS == 0U) || \
     (ENGINE_INJECTOR_TEST_QUIET_TIME_MS == 0U) || \
     (ENGINE_INJECTOR_TEST_MAX_DUTY_PERMILLE == 0U) || \
     (ENGINE_INJECTOR_TEST_MAX_DUTY_PERMILLE > 1000U))
#error "Invalid stationary injector test safety limits"
#endif

/* A loss of position while outputs are running requires an explicit fault
 * clear and re-arm.  This is intentionally conservative for a controller that
 * can drive real coils and injectors. */
#define ENGINE_LATCH_FAULT_ON_SYNC_LOSS        1U

/* Bound foreground work under electrical noise or an unexpectedly dense wheel
 * set. Exceeding this budget is fail-safe: outputs are shut down and the
 * remaining records are drained over subsequent passes. */
#define ENGINE_MAX_TRIGGER_EVENTS_PER_SERVICE 256U

/* Sequential operation is the safe default.  Set either value to 0 only when
 * the corresponding calibration intentionally implements wasted-spark or
 * batch injection without a 720 degree phase reference. */
#ifndef IGNITION_REQUIRES_PHASE_SYNC
#define IGNITION_REQUIRES_PHASE_SYNC        1U
#endif
#ifndef INJECTION_REQUIRES_PHASE_SYNC
#define INJECTION_REQUIRES_PHASE_SYNC       1U
#endif

/* Runtime snapshots outside this speed range are rejected. */
#define ENGINE_MIN_SCHEDULING_RPM          20.0f
#define ENGINE_MAX_SCHEDULING_RPM       20000.0f

/* A larger forward jump means the foreground scheduler was not serviced
 * often enough to place an event safely.  Outputs are switched off and the
 * current angle is used as a fresh starting point. */
#define ACTUATOR_MAX_ANGLE_STEP_DEG         45.0f

/* Extra dwell allowed after the requested dwell time if the crank slows just
 * before the spark angle.  The per-cylinder hard limit still wins. */
#define IGNITION_DEADLINE_GUARD_US         250U

/* DAC value used while no injector is active. */
#define INJECTOR_IDLE_DAC_CODE            1024U

typedef struct
{
  float    default_advance_deg;
  float    minimum_advance_deg;
  float    maximum_advance_deg;
  uint32_t default_dwell_us;
  uint32_t minimum_dwell_us;
  uint32_t maximum_dwell_us;
  uint32_t hard_dwell_limit_us;
} ignition_calibration_t;

typedef struct
{
  float    default_start_btdc_deg;
  float    minimum_start_btdc_deg;
  float    maximum_start_btdc_deg;
  uint32_t default_duration_us;
  uint32_t minimum_duration_us;
  uint32_t maximum_duration_us;

  /* Peak-and-hold electrical calibration.  boost_duration_us is clipped to
   * the commanded injection duration for very short pulses.  The DAC codes
   * must be derived from measured zero-current and codes/ampere for this PCB. */
  uint32_t boost_duration_us;
  uint16_t boost_dac_code;
  uint16_t hold_dac_code;
} injection_calibration_t;

typedef struct
{
  uint8_t cylinder_id;       /* API-visible cylinder number, 1..32 */
  float   firing_tdc_deg;    /* compression TDC in [0, 720) */
  uint8_t ignition_output;   /* board IGN channel, 1 based */
  uint8_t injector_output;   /* board injector channel, 1 based */
  ignition_calibration_t ignition;
  injection_calibration_t injection;
} engine_cylinder_config_t;

/* Compact initializers keep the firing table easy to scan.  They may be
 * replaced by literal per-cylinder values wherever individual trim is needed.
 */
#define IGNITION_STANDARD_CALIBRATION                                      \
  { 15.0f, -10.0f, 60.0f, 3000U, 500U, 4500U, 5000U }

/* Placeholder raw thresholds: injection remains locked out while
 * INJECTION_CURRENT_CALIBRATION_VALID is zero. */
#define INJECTION_STANDARD_CALIBRATION                                     \
  { 360.0f, 0.0f, 720.0f, 8000U, 100U, 30000U, 1000U, 2048U, 1024U }

/* Inline-four example, firing order 1-3-4-2.  TDC is explicit rather than
 * accumulated, so a long/short or odd-fire pattern is entered directly.
 *
 * cylinder   TDC      IGN output   injector output
 */
static const engine_cylinder_config_t engine_cylinders[ENGINE_CYLINDER_COUNT] =
{
  { 1U,   0.0f, 1U, 1U, IGNITION_STANDARD_CALIBRATION, INJECTION_STANDARD_CALIBRATION },
  { 3U, 180.0f, 3U, 3U, IGNITION_STANDARD_CALIBRATION, INJECTION_STANDARD_CALIBRATION },
  { 4U, 360.0f, 4U, 4U, IGNITION_STANDARD_CALIBRATION, INJECTION_STANDARD_CALIBRATION },
  { 2U, 540.0f, 2U, 2U, IGNITION_STANDARD_CALIBRATION, INJECTION_STANDARD_CALIBRATION },
};

static inline int8_t engine_cylinder_index(uint8_t cylinder_id)
{
  uint8_t i;

  for (i = 0U; i < ENGINE_CYLINDER_COUNT; ++i)
  {
    if (engine_cylinders[i].cylinder_id == cylinder_id)
    {
      return (int8_t)i;
    }
  }
  return -1;
}

#endif /* ENGINE_CONFIG_H */
