/**
 ******************************************************************************
 * @file    dcdc_config.h
 * @brief   User-editable two-phase boost-converter calibration.
 *
 * TIM1 produces two equal-duty outputs separated by half a switching period:
 * PC0/TIM1_CH1 drives phase 1 and PE13/TIM1_CH3 drives phase 2.
 *
 * IMPORTANT HARDWARE BLOCKERS ON THE CURRENT PCB
 * ------------------------------------------------
 * D53 is fitted from VBUS_ADC to ground, in parallel with the 1 kOhm lower
 * divider resistor (R146).  Its forward conduction makes the nominal 25:1 /
 * 1:1 divider nonlinear and eventually clamps the feedback.  The converter
 * also has no phase-current measurement or cycle-by-cycle hardware limit.
 * Closed-loop switching therefore remains locked until those issues and the
 * power-stage ratings have been addressed on real hardware.
 ******************************************************************************
 */
#ifndef DCDC_CONFIG_H
#define DCDC_CONFIG_H

/* ========================== Hardware acknowledgements =================== */

/* Set each acknowledgement to 1 only after the stated work is complete. */
#define DCDC_VBUS_FEEDBACK_LINEARITY_CONFIRMED   0U
#define DCDC_CURRENT_LIMIT_PROTECTION_CONFIRMED  0U
#define DCDC_POWER_STAGE_CALIBRATION_VALID       0U

/* Boot remains passive by default. Runtime code may instead call
 * dcdc_control_request_enable(true) after all external interlocks are safe. */
#define DCDC_OUTPUTS_ARM_AT_BOOT                  0U

#define DCDC_OUTPUT_PERMISSION \
  ((DCDC_VBUS_FEEDBACK_LINEARITY_CONFIRMED != 0U) && \
   (DCDC_CURRENT_LIMIT_PROTECTION_CONFIRMED != 0U) && \
   (DCDC_POWER_STAGE_CALIBRATION_VALID != 0U))

/* ============================== PWM timing ============================== */

/* This frequency is only a timer-configuration placeholder while output is
 * locked. Select it from measured inductor ripple, saturation current,
 * switching loss, diode recovery, thermal limits, and control bandwidth. */
#define DCDC_PWM_FREQUENCY_HZ                100000U

/* Required minimum total high and low times for either phase. Enter measured
 * safe values before acknowledging the power-stage calibration. */
#define DCDC_PWM_MINIMUM_ON_TIME_NS               0U
#define DCDC_PWM_MINIMUM_OFF_TIME_NS              0U

/* A nonzero command below MIN_ACTIVE is promoted to MIN_ACTIVE; a command
 * below the controller's turn-off threshold becomes zero instead. */
#define DCDC_MINIMUM_ACTIVE_DUTY_PERMILLE          0U
#define DCDC_MAXIMUM_DUTY_PERMILLE                 0U
#define DCDC_DUTY_RISE_PER_CONTROL_PERMILLE        0U
#define DCDC_DUTY_FALL_PER_CONTROL_PERMILLE        0U

/* ============================ VBUS acquisition ========================== */

/* Nominal schematic values. They are not a valid calibration while D53 is
 * fitted. The conversion uses measured VDDA and 64-bit integer arithmetic. */
#define DCDC_VBUS_DIVIDER_HIGH_OHMS            25000U
#define DCDC_VBUS_DIVIDER_LOW_OHMS              1000U
#define DCDC_ADC_FULL_SCALE_COUNTS              4095U
#define DCDC_ADC_VALID_VDDA_MIN_MV              2800U
#define DCDC_ADC_VALID_VDDA_MAX_MV              3600U

/* Enter a measured raw ADC threshold for the ADC1 analog watchdog. This is a
 * second, off-only overvoltage path; it does not make the present D53-clamped
 * feedback safe. */
#define DCDC_VBUS_ANALOG_WATCHDOG_HIGH_RAW          0U

/* All voltage values below are deliberately zero rather than pretending that
 * a safe target can be derived from unspecified power-component ratings. */
#define DCDC_VBUS_PLAUSIBLE_MIN_MV                   0U
#define DCDC_VBUS_PLAUSIBLE_MAX_MV                   0U
#define DCDC_VBUS_TARGET_MIN_MV                      0U
#define DCDC_VBUS_TARGET_DEFAULT_MV                  0U
#define DCDC_VBUS_TARGET_MAX_MV                      0U
#define DCDC_VBUS_OVERVOLTAGE_RESET_MV               0U
#define DCDC_VBUS_OVERVOLTAGE_TRIP_MV                0U

/* ============================= Control loop ============================= */

#define DCDC_CONTROL_PERIOD_MS                       1U
#define DCDC_CONTROL_DEADLINE_MS                     4U
#define DCDC_VBUS_SAMPLE_TIMEOUT_MS                 10U
#define DCDC_STARTUP_VALID_SAMPLE_COUNT              4U

/* Reference ramp during startup. Enter a validated rate before enabling. */
#define DCDC_SOFT_START_MV_PER_SECOND                 0U

/* PID output is duty fraction (0.0 to 1.0); error is volts. Kd defaults to
 * zero in most boost-voltage bring-up work because raw ADC differentiation is
 * noisy. Tune the loop from measured plant/load data, never by guesswork. */
#define DCDC_PID_KP_DUTY_PER_VOLT                   0.0f
#define DCDC_PID_KI_DUTY_PER_VOLT_SECOND            0.0f
#define DCDC_PID_KD_DUTY_SECOND_PER_VOLT            0.0f
#define DCDC_PID_ANTI_WINDUP_PER_SECOND             0.0f
#define DCDC_VBUS_FILTER_TIME_CONSTANT_MS             5U
#define DCDC_DERIVATIVE_FILTER_TIME_CONSTANT_MS       2U

/* ========================= Compile-time guardrails ====================== */

#if ((DCDC_VBUS_FEEDBACK_LINEARITY_CONFIRMED > 1U) || \
     (DCDC_CURRENT_LIMIT_PROTECTION_CONFIRMED > 1U) || \
     (DCDC_POWER_STAGE_CALIBRATION_VALID > 1U) || \
     (DCDC_OUTPUTS_ARM_AT_BOOT > 1U))
#error "DCDC acknowledgement and arm switches must be 0 or 1"
#endif

#if ((DCDC_PWM_FREQUENCY_HZ == 0U) || (DCDC_CONTROL_PERIOD_MS == 0U) || \
     (DCDC_CONTROL_DEADLINE_MS < (2U * DCDC_CONTROL_PERIOD_MS)) || \
     (DCDC_VBUS_SAMPLE_TIMEOUT_MS < DCDC_CONTROL_PERIOD_MS) || \
     (DCDC_STARTUP_VALID_SAMPLE_COUNT == 0U) || \
     (DCDC_VBUS_DIVIDER_LOW_OHMS == 0U) || \
     (DCDC_ADC_FULL_SCALE_COUNTS == 0U) || \
     (DCDC_ADC_VALID_VDDA_MIN_MV >= DCDC_ADC_VALID_VDDA_MAX_MV))
#error "Invalid basic DCDC timing or ADC configuration"
#endif

#if (DCDC_OUTPUT_PERMISSION != 0U)
#if ((DCDC_PWM_MINIMUM_ON_TIME_NS == 0U) || \
     (DCDC_PWM_MINIMUM_OFF_TIME_NS == 0U) || \
     (DCDC_MINIMUM_ACTIVE_DUTY_PERMILLE == 0U) || \
     (DCDC_MAXIMUM_DUTY_PERMILLE >= 1000U) || \
     (DCDC_MINIMUM_ACTIVE_DUTY_PERMILLE > DCDC_MAXIMUM_DUTY_PERMILLE) || \
     (DCDC_DUTY_RISE_PER_CONTROL_PERMILLE == 0U) || \
     (DCDC_DUTY_FALL_PER_CONTROL_PERMILLE == 0U) || \
     (DCDC_VBUS_ANALOG_WATCHDOG_HIGH_RAW == 0U) || \
     (DCDC_VBUS_ANALOG_WATCHDOG_HIGH_RAW > DCDC_ADC_FULL_SCALE_COUNTS) || \
     (DCDC_VBUS_PLAUSIBLE_MIN_MV >= DCDC_VBUS_TARGET_MIN_MV) || \
     (DCDC_VBUS_TARGET_MIN_MV > DCDC_VBUS_TARGET_DEFAULT_MV) || \
     (DCDC_VBUS_TARGET_DEFAULT_MV > DCDC_VBUS_TARGET_MAX_MV) || \
     (DCDC_VBUS_TARGET_MAX_MV > DCDC_VBUS_OVERVOLTAGE_RESET_MV) || \
     (DCDC_VBUS_OVERVOLTAGE_RESET_MV >= DCDC_VBUS_OVERVOLTAGE_TRIP_MV) || \
     (DCDC_VBUS_OVERVOLTAGE_TRIP_MV > DCDC_VBUS_PLAUSIBLE_MAX_MV) || \
     (DCDC_SOFT_START_MV_PER_SECOND == 0U))
#error "Complete and validate every DCDC power-stage limit before enabling"
#endif
#endif

/* The C preprocessor accepts only integer constant expressions in #if. The
 * floating-point PID gains, their finiteness, and the requirement for P or I
 * action are therefore checked by dcdc_control_init() before it can arm PWM. */

#if ((DCDC_OUTPUTS_ARM_AT_BOOT != 0U) && (DCDC_OUTPUT_PERMISSION == 0U))
#error "DCDC output cannot arm until every hardware/calibration gate is valid"
#endif

#endif /* DCDC_CONFIG_H */
