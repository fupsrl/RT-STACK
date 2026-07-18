/**
 ******************************************************************************
 * @file    trigger_decoder.h
 * @brief   Hardware-independent, configurable engine trigger decoder.
 *
 * The decoder deliberately knows nothing about EXTI, DWT, or STM32 timers.
 * A capture backend supplies timestamped edges.  This makes the same wheel
 * configuration usable with an EXTI timestamp, timer input capture, DMA, or
 * the host tests.
 *
 * Normal integration:
 *
 *   trigger_decoder_context_init(&decoder,
 *                                trigger_decoder_default_config(),
 *                                timestamp_frequency_hz);
 *
 *   // For every captured edge, in timestamp order:
 *   trigger_decoder_process_event(&decoder, &event);
 *
 *   // Once per foreground pass, after reading the current timestamp:
 *   trigger_decoder_poll(&decoder, now, &output);
 *
 * The optional trigger_recorder module provides bounded per-channel queues
 * and merges events from several interrupt sources into timestamp order.
 ******************************************************************************
 */
#ifndef TRIGGER_DECODER_H
#define TRIGGER_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TMG signals are numbered 1..9.  Slot zero is intentionally unused so an
 * electrical channel number can be used directly as an array index. */
#define TRIGGER_CHANNEL_COUNT              10U
#define TMG_CH_COUNT                       TRIGGER_CHANNEL_COUNT

/* Static bounds: raise these if a configuration needs more wheels or a
 * longer cyclic pattern.  No heap allocation is used on the ECU. */
#ifndef TRIGGER_MAX_WHEELS
#define TRIGGER_MAX_WHEELS                  9U
#endif

#ifndef TRIGGER_MAX_PATTERN_INTERVALS
#define TRIGGER_MAX_PATTERN_INTERVALS      32U
#endif

#define TRIGGER_NO_WHEEL                  UINT8_MAX

typedef enum
{
  TRIGGER_EDGE_RISING  = 1U,
  TRIGGER_EDGE_FALLING = 2U,
  TRIGGER_EDGE_BOTH    = 3U
} trigger_edge_t;

typedef enum
{
  TRIGGER_WHEEL_MISSING_TOOTH = 0,
  TRIGGER_WHEEL_INTERVAL_PATTERN
} trigger_wheel_type_t;

typedef enum
{
  TRIGGER_INTERVAL_SHORT = 1U,
  TRIGGER_INTERVAL_LONG  = 2U
} trigger_interval_class_t;

/** Geometry for a conventional N-M wheel, for example 60-2 or 36-1. */
typedef struct
{
  uint16_t tooth_positions; /* N: equally spaced positions in one cycle */
  uint16_t missing_teeth;   /* M: consecutive positions without teeth   */
} trigger_missing_tooth_config_t;

/**
 * Geometry for a cyclic long/short interval pattern.
 *
 * intervals[i] describes the distance from edge i to edge (i+1), wrapping at
 * interval_count.  A LONG interval is long_interval_units times a SHORT one.
 * Example {S,S,L,L} has four uniquely identifiable edges and six total units.
 */
typedef struct
{
  const trigger_interval_class_t *intervals;
  uint16_t interval_count;
  float long_interval_units; /* may be non-integer, for example 1.5 */
} trigger_interval_pattern_config_t;

/** One independently decoded physical wheel. */
typedef struct
{
  const char *name;                 /* diagnostic name; may be NULL       */
  uint8_t input_channel;            /* TMG channel, 1..9                  */
  trigger_edge_t active_edge;       /* edge(s) represented by the pattern */
  trigger_wheel_type_t type;

  float cycle_degrees;              /* 360 crank, normally 720 cam        */
  float angle_at_index0_deg;        /* angle exactly at pattern edge zero */
  int8_t angle_direction;           /* +1 increasing, -1 decreasing       */

  /* Timing acceptance.  For example 350 means +/-35% around the expected
   * interval.  The normalized interval estimate follows speed using the IIR
   * coefficient below (1000 = use the newest interval immediately). */
  uint16_t interval_tolerance_permille;
  uint16_t speed_filter_permille;

  /* Missing-tooth: number of reference gaps required (2 recommended).
   * Pattern: number of complete matching pattern rotations required. */
  uint8_t sync_confirmations;

  /* Reject electrical glitches shorter than minimum_interval_us.  Declare a
   * stopped/missing wheel when the edge age exceeds the greater of
   * minimum_timeout_us and timeout_interval_multiplier expected intervals. */
  uint32_t minimum_interval_us;
  uint32_t minimum_timeout_us;
  float timeout_interval_multiplier;

  union
  {
    trigger_missing_tooth_config_t missing_tooth;
    trigger_interval_pattern_config_t pattern;
  } geometry;
} trigger_wheel_config_t;

/** Complete engine trigger configuration. */
typedef struct
{
  const trigger_wheel_config_t *wheels;
  uint8_t wheel_count;

  /* The primary wheel supplies engine speed and the continuously interpolated
   * crank angle.  Set phase_wheel to TRIGGER_NO_WHEEL for crank-only use. */
  uint8_t primary_wheel;
  uint8_t phase_wheel;
  float engine_cycle_degrees;       /* normally 720 for a four-stroke      */

  /* Maximum circular error allowed between the decoded phase-wheel angle and
   * the primary-wheel revolution candidate. Used only when phase_wheel is set
   * and must be strictly less than half the primary wheel cycle. */
  float phase_alignment_tolerance_deg;
} trigger_decoder_config_t;

/** A coherent edge record produced by any capture backend. */
typedef struct
{
  uint32_t timestamp;               /* free-running common timebase        */
  uint16_t sequence;                /* per-channel, increments every edge  */
  uint8_t channel;                  /* physical TMG number, 1..9           */
  trigger_edge_t edge;
} trigger_event_t;

typedef enum
{
  TRIGGER_LOSS_NONE = 0,
  TRIGGER_LOSS_TIMEOUT,
  TRIGGER_LOSS_EVENT_DROPPED,
  TRIGGER_LOSS_TIMESTAMP_ORDER,
  TRIGGER_LOSS_INTERVAL_TOO_SHORT,
  TRIGGER_LOSS_UNEXPECTED_INTERVAL,
  TRIGGER_LOSS_REFERENCE_COUNT,
  TRIGGER_LOSS_PHASE_ALIGNMENT,
  TRIGGER_LOSS_FORCED
} trigger_loss_reason_t;

typedef enum
{
  TRIGGER_CONFIG_OK = 0,
  TRIGGER_CONFIG_NULL,
  TRIGGER_CONFIG_TOO_MANY_WHEELS,
  TRIGGER_CONFIG_BAD_PRIMARY,
  TRIGGER_CONFIG_BAD_PHASE,
  TRIGGER_CONFIG_BAD_CHANNEL,
  TRIGGER_CONFIG_DUPLICATE_CHANNEL,
  TRIGGER_CONFIG_BAD_ANGLE,
  TRIGGER_CONFIG_BAD_TIMING_LIMIT,
  TRIGGER_CONFIG_BAD_MISSING_TOOTH,
  TRIGGER_CONFIG_BAD_PATTERN,
  TRIGGER_CONFIG_AMBIGUOUS_PATTERN,
  TRIGGER_CONFIG_BAD_CYCLE_RELATION,
  TRIGGER_CONFIG_BAD_TIMESTAMP_FREQUENCY
} trigger_config_error_t;

typedef struct
{
  float angle_deg;
  float rpm;
  bool synced;
  uint16_t edge_index;
  uint32_t last_edge_timestamp;
  uint32_t sync_epoch;
  trigger_loss_reason_t last_loss_reason;
  uint32_t accepted_event_count;
  uint32_t rejected_event_count;
  uint32_t dropped_event_count;
  uint32_t sync_loss_count;
} trigger_wheel_output_t;

/** Decoder result.  The first fields retain the original application's names
 * so telemetry/application integration remains straightforward. */
typedef struct
{
  float angle[TRIGGER_CHANNEL_COUNT];
  bool ch_synced[TRIGGER_CHANNEL_COUNT];
  float crank_angle_deg;             /* 0..engine cycle if phase is known */
  float rpm;
  bool synced;                       /* primary wheel is synchronized      */
  bool phase_known;

  uint8_t wheel_count;
  uint32_t sync_epoch;               /* changes whenever valid position is lost */
  uint32_t latched_faults;            /* bit n corresponds to loss reason n      */
  trigger_wheel_output_t wheel[TRIGGER_MAX_WHEELS];
} trigger_output_t;

/* Decoder state is public only so it can be statically allocated.  Application
 * code should treat every member below as private and use the API functions. */
typedef struct
{
  bool configured;
  bool synced;
  bool have_timestamp;
  bool have_sequence;
  bool reference_candidate;
  bool pattern_candidate;
  uint8_t reference_confirmations;
  uint16_t candidate_intervals_remaining;
  uint16_t edge_index;
  uint16_t intervals_since_reference;
  uint16_t last_sequence;
  uint32_t last_edge_timestamp;
  uint32_t last_period_ticks;
  float unit_period_ticks;
  uint32_t minimum_interval_ticks;
  uint32_t minimum_timeout_ticks;
  uint32_t pattern_period_history[TRIGGER_MAX_PATTERN_INTERVALS];
  uint16_t pattern_history_fill;
  uint16_t pattern_candidate_edge;
  uint32_t sync_epoch;
  trigger_loss_reason_t last_loss_reason;
  uint32_t accepted_event_count;
  uint32_t rejected_event_count;
  uint32_t dropped_event_count;
  uint32_t sync_loss_count;
} trigger_wheel_state_t;

typedef struct
{
  const trigger_decoder_config_t *config;
  uint32_t timestamp_frequency_hz;
  trigger_config_error_t config_error;
  int8_t channel_to_wheel[TRIGGER_CHANNEL_COUNT];
  trigger_wheel_state_t wheel[TRIGGER_MAX_WHEELS];
  uint8_t phase_turn;
  bool phase_known;
  uint32_t sync_epoch;
  uint32_t latched_faults;
} trigger_decoder_t;

/** Return the user-editable configuration from trigger_decoder_config.h. */
const trigger_decoder_config_t *trigger_decoder_default_config(void);

/** Validate and initialize one decoder instance. */
trigger_config_error_t trigger_decoder_context_init(
    trigger_decoder_t *decoder,
    const trigger_decoder_config_t *config,
    uint32_t timestamp_frequency_hz);

/** Human-readable configuration error for startup diagnostics. */
const char *trigger_decoder_config_error_string(trigger_config_error_t error);

/** Consume one event. Events for each channel must be supplied in order. */
void trigger_decoder_process_event(trigger_decoder_t *decoder,
                                   const trigger_event_t *event);

/** Explicitly report timer overcapture, DMA overwrite, or backend queue loss. */
void trigger_decoder_note_event_loss(trigger_decoder_t *decoder,
                                     uint8_t channel,
                                     uint32_t lost_event_count);

/** Apply timeouts, interpolate angles to now, and produce a coherent result. */
void trigger_decoder_poll(trigger_decoder_t *decoder,
                          uint32_t now,
                          trigger_output_t *output);

/** Force all wheels out of sync, for example when engine control is disabled. */
void trigger_decoder_force_sync_loss(trigger_decoder_t *decoder,
                                     trigger_loss_reason_t reason);

/* -------------------------------------------------------------------------
 * Legacy latest-edge adapter.
 *
 * This keeps an existing main.c buildable while it is migrated to the queued
 * API.  It deliberately declares synchronization lost if more than one edge
 * arrived between foreground calls; it can never reconstruct missing events.
 * New code should not use this adapter.
 * ---------------------------------------------------------------------- */
typedef struct
{
  const volatile uint32_t *last_edge_cyc;
  const volatile uint32_t *period_cyc;
  const volatile uint32_t *edge_count;
  uint32_t now_cyc;
  uint32_t cpu_hz;
} trigger_input_t;

void trigger_decoder_init(void);
void trigger_decoder_update(const trigger_input_t *input,
                            trigger_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* TRIGGER_DECODER_H */
