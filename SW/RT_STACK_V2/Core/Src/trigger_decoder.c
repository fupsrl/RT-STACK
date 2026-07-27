/**
 ******************************************************************************
 * @file    trigger_decoder.c
 * @brief   Generic missing-tooth and cyclic interval-pattern decoder.
 ******************************************************************************
 */
#include "trigger_decoder.h"
#include "trigger_decoder_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PERMILLE_SCALE       1000.0f
#define TIMESTAMP_HALF_RANGE 0x80000000UL

static trigger_decoder_t legacy_decoder;
static uint32_t legacy_seen_count[TRIGGER_CHANNEL_COUNT];

static float wrap_angle(float angle, float cycle)
{
  if ((!isfinite(angle)) || (!isfinite(cycle)) || (cycle <= 0.0f))
  {
    return 0.0f;
  }

  angle = fmodf(angle, cycle);
  if (angle < 0.0f)
  {
    angle += cycle;
  }
  return angle;
}

static float absolute_value(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float circular_distance(float a, float b, float cycle)
{
  float difference = absolute_value(wrap_angle(a - b, cycle));
  return (difference > (0.5f * cycle)) ? (cycle - difference) : difference;
}

static uint32_t microseconds_to_ticks(uint32_t microseconds, uint32_t tick_hz)
{
  uint64_t ticks = ((uint64_t)microseconds * (uint64_t)tick_hz + 999999ULL) /
                   1000000ULL;
  if (ticks >= TIMESTAMP_HALF_RANGE)
  {
    ticks = TIMESTAMP_HALF_RANGE - 1ULL;
  }
  return (uint32_t)ticks;
}

static float pattern_total_units(const trigger_wheel_config_t *config)
{
  float units = 0.0f;
  const trigger_interval_pattern_config_t *pattern = &config->geometry.pattern;
  for (uint16_t index = 0U; index < pattern->interval_count; index++)
  {
    units += (pattern->intervals[index] == TRIGGER_INTERVAL_LONG) ?
             pattern->long_interval_units : 1.0f;
  }
  return units;
}

static float pattern_units_at(const trigger_wheel_config_t *config,
                              uint16_t index)
{
  const trigger_interval_pattern_config_t *pattern = &config->geometry.pattern;
  return (pattern->intervals[index % pattern->interval_count] ==
          TRIGGER_INTERVAL_LONG) ? pattern->long_interval_units : 1.0f;
}

static uint16_t wheel_edge_count(const trigger_wheel_config_t *config)
{
  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    return (uint16_t)(config->geometry.missing_tooth.tooth_positions -
                      config->geometry.missing_tooth.missing_teeth);
  }
  return config->geometry.pattern.interval_count;
}

static float wheel_max_interval_units(const trigger_wheel_config_t *config)
{
  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    return (float)config->geometry.missing_tooth.missing_teeth + 1.0f;
  }
  return config->geometry.pattern.long_interval_units;
}

static float wheel_total_interval_units(const trigger_wheel_config_t *config)
{
  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    return (float)config->geometry.missing_tooth.tooth_positions;
  }
  return pattern_total_units(config);
}

static float wheel_edge_angle(const trigger_wheel_config_t *config,
                              uint16_t edge_index)
{
  float unit_angle;
  float preceding_units = 0.0f;

  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    unit_angle = config->cycle_degrees /
                 (float)config->geometry.missing_tooth.tooth_positions;
    preceding_units = (float)edge_index;
  }
  else
  {
    unit_angle = config->cycle_degrees / pattern_total_units(config);
    for (uint16_t index = 0U; index < edge_index; index++)
    {
      preceding_units += (float)pattern_units_at(config, index);
    }
  }

  return wrap_angle(config->angle_at_index0_deg +
                    ((float)config->angle_direction * preceding_units * unit_angle),
                    config->cycle_degrees);
}

static float wheel_next_interval_units(const trigger_wheel_config_t *config,
                                       uint16_t edge_index)
{
  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    uint16_t physical_teeth = wheel_edge_count(config);
    return (edge_index == (uint16_t)(physical_teeth - 1U)) ?
           wheel_max_interval_units(config) : 1.0f;
  }
  return pattern_units_at(config, edge_index);
}

static float wheel_next_interval_degrees(const trigger_wheel_config_t *config,
                                         uint16_t edge_index)
{
  return config->cycle_degrees *
         (float)wheel_next_interval_units(config, edge_index) /
         wheel_total_interval_units(config);
}

static bool pattern_has_unique_rotation(
    const trigger_interval_pattern_config_t *pattern)
{
  for (uint16_t shift = 1U; shift < pattern->interval_count; shift++)
  {
    bool identical = true;
    for (uint16_t index = 0U; index < pattern->interval_count; index++)
    {
      if (pattern->intervals[index] !=
          pattern->intervals[(index + shift) % pattern->interval_count])
      {
        identical = false;
        break;
      }
    }
    if (identical)
    {
      return false;
    }
  }
  return true;
}

static bool interval_classes_are_separable(float long_units,
                                           uint16_t tolerance_permille)
{
  float tolerance = (float)tolerance_permille / PERMILLE_SCALE;
  float short_high = 1.0f + tolerance;
  float long_low = long_units * (1.0f - tolerance);
  return short_high < long_low;
}

static trigger_config_error_t validate_config(
    const trigger_decoder_config_t *config,
    uint32_t timestamp_frequency_hz)
{
  if ((config == NULL) || (config->wheels == NULL))
  {
    return TRIGGER_CONFIG_NULL;
  }
  if (timestamp_frequency_hz == 0U)
  {
    return TRIGGER_CONFIG_BAD_TIMESTAMP_FREQUENCY;
  }
  if ((config->wheel_count == 0U) ||
      (config->wheel_count > TRIGGER_MAX_WHEELS))
  {
    return TRIGGER_CONFIG_TOO_MANY_WHEELS;
  }
  if (config->primary_wheel >= config->wheel_count)
  {
    return TRIGGER_CONFIG_BAD_PRIMARY;
  }
  if ((config->phase_wheel != TRIGGER_NO_WHEEL) &&
      ((config->phase_wheel >= config->wheel_count) ||
       (config->phase_wheel == config->primary_wheel)))
  {
    return TRIGGER_CONFIG_BAD_PHASE;
  }
  if ((!isfinite(config->engine_cycle_degrees)) ||
      (config->engine_cycle_degrees <= 0.0f))
  {
    return TRIGGER_CONFIG_BAD_ANGLE;
  }

  bool channel_used[TRIGGER_CHANNEL_COUNT] = {false};
  for (uint8_t wheel = 0U; wheel < config->wheel_count; wheel++)
  {
    const trigger_wheel_config_t *wheel_config = &config->wheels[wheel];
    if ((wheel_config->input_channel == 0U) ||
        (wheel_config->input_channel >= TRIGGER_CHANNEL_COUNT))
    {
      return TRIGGER_CONFIG_BAD_CHANNEL;
    }
    if (channel_used[wheel_config->input_channel])
    {
      return TRIGGER_CONFIG_DUPLICATE_CHANNEL;
    }
    channel_used[wheel_config->input_channel] = true;

    if ((wheel_config->active_edge != TRIGGER_EDGE_RISING) &&
        (wheel_config->active_edge != TRIGGER_EDGE_FALLING) &&
        (wheel_config->active_edge != TRIGGER_EDGE_BOTH))
    {
      return TRIGGER_CONFIG_BAD_CHANNEL;
    }
    if ((!isfinite(wheel_config->cycle_degrees)) ||
        (!isfinite(wheel_config->angle_at_index0_deg)) ||
        (wheel_config->cycle_degrees <= 0.0f) ||
        ((wheel_config->angle_direction != 1) &&
         (wheel_config->angle_direction != -1)))
    {
      return TRIGGER_CONFIG_BAD_ANGLE;
    }
    if ((wheel_config->interval_tolerance_permille == 0U) ||
        (wheel_config->interval_tolerance_permille >= 1000U) ||
        (wheel_config->speed_filter_permille == 0U) ||
        (wheel_config->speed_filter_permille > 1000U) ||
        (wheel_config->sync_confirmations == 0U) ||
        (!isfinite(wheel_config->timeout_interval_multiplier)) ||
        (wheel_config->timeout_interval_multiplier < 1.0f))
    {
      return TRIGGER_CONFIG_BAD_TIMING_LIMIT;
    }

    if (wheel_config->type == TRIGGER_WHEEL_MISSING_TOOTH)
    {
      uint16_t positions = wheel_config->geometry.missing_tooth.tooth_positions;
      uint16_t missing = wheel_config->geometry.missing_tooth.missing_teeth;
      if ((positions < 3U) || (missing == 0U) ||
          (missing >= (uint16_t)(positions - 1U)) ||
          ((uint32_t)missing + 1U > UINT8_MAX) ||
          !interval_classes_are_separable(
              (float)((uint32_t)missing + 1U),
              wheel_config->interval_tolerance_permille))
      {
        return TRIGGER_CONFIG_BAD_MISSING_TOOTH;
      }
    }
    else if (wheel_config->type == TRIGGER_WHEEL_INTERVAL_PATTERN)
    {
      const trigger_interval_pattern_config_t *pattern =
          &wheel_config->geometry.pattern;
      if ((pattern->intervals == NULL) || (pattern->interval_count < 2U) ||
          (pattern->interval_count > TRIGGER_MAX_PATTERN_INTERVALS) ||
          (!isfinite(pattern->long_interval_units)) ||
          (pattern->long_interval_units <= 1.0f))
      {
        return TRIGGER_CONFIG_BAD_PATTERN;
      }
      bool have_short = false;
      bool have_long = false;
      for (uint16_t index = 0U; index < pattern->interval_count; index++)
      {
        if (pattern->intervals[index] == TRIGGER_INTERVAL_SHORT)
        {
          have_short = true;
        }
        else if (pattern->intervals[index] == TRIGGER_INTERVAL_LONG)
        {
          have_long = true;
        }
        else
        {
          return TRIGGER_CONFIG_BAD_PATTERN;
        }
      }
      if ((!have_short) || (!have_long))
      {
        return TRIGGER_CONFIG_BAD_PATTERN;
      }
      if (!pattern_has_unique_rotation(pattern))
      {
        return TRIGGER_CONFIG_AMBIGUOUS_PATTERN;
      }
      if (!interval_classes_are_separable(
              pattern->long_interval_units,
              wheel_config->interval_tolerance_permille))
      {
        return TRIGGER_CONFIG_BAD_TIMING_LIMIT;
      }
    }
    else
    {
      return TRIGGER_CONFIG_BAD_PATTERN;
    }
  }

  const trigger_wheel_config_t *primary =
      &config->wheels[config->primary_wheel];
  float turns = config->engine_cycle_degrees / primary->cycle_degrees;
  float rounded_turns = floorf(turns + 0.5f);
  if ((rounded_turns < 1.0f) || (rounded_turns > 255.0f) ||
      (absolute_value(turns - rounded_turns) > 0.001f))
  {
    return TRIGGER_CONFIG_BAD_CYCLE_RELATION;
  }

  if (config->phase_wheel != TRIGGER_NO_WHEEL)
  {
    const trigger_wheel_config_t *phase =
        &config->wheels[config->phase_wheel];
    if (absolute_value(phase->cycle_degrees -
                       config->engine_cycle_degrees) > 0.001f)
    {
      return TRIGGER_CONFIG_BAD_CYCLE_RELATION;
    }
    /* Adjacent phase candidates are one primary-wheel revolution apart.  A
     * tolerance at or beyond their midpoint cannot identify a unique turn. */
    if ((!isfinite(config->phase_alignment_tolerance_deg)) ||
        (config->phase_alignment_tolerance_deg <= 0.0f) ||
        (config->phase_alignment_tolerance_deg >=
         (0.5f * primary->cycle_degrees)))
    {
      return TRIGGER_CONFIG_BAD_TIMING_LIMIT;
    }
  }

  return TRIGGER_CONFIG_OK;
}

const trigger_decoder_config_t *trigger_decoder_default_config(void)
{
  return &trigger_default_config;
}

trigger_config_error_t trigger_decoder_context_init(
    trigger_decoder_t *decoder,
    const trigger_decoder_config_t *config,
    uint32_t timestamp_frequency_hz)
{
  if (decoder == NULL)
  {
    return TRIGGER_CONFIG_NULL;
  }

  memset(decoder, 0, sizeof(*decoder));
  decoder->config = config;
  decoder->timestamp_frequency_hz = timestamp_frequency_hz;
  decoder->config_error = validate_config(config, timestamp_frequency_hz);

  for (uint8_t channel = 0U; channel < TRIGGER_CHANNEL_COUNT; channel++)
  {
    decoder->channel_to_wheel[channel] = -1;
  }

  if (decoder->config_error != TRIGGER_CONFIG_OK)
  {
    return decoder->config_error;
  }

  for (uint8_t wheel = 0U; wheel < config->wheel_count; wheel++)
  {
    const trigger_wheel_config_t *wheel_config = &config->wheels[wheel];
    trigger_wheel_state_t *state = &decoder->wheel[wheel];
    decoder->channel_to_wheel[wheel_config->input_channel] = (int8_t)wheel;
    state->configured = true;
    state->minimum_interval_ticks = microseconds_to_ticks(
        wheel_config->minimum_interval_us, timestamp_frequency_hz);
    state->minimum_timeout_ticks = microseconds_to_ticks(
        wheel_config->minimum_timeout_us, timestamp_frequency_hz);
  }

  return TRIGGER_CONFIG_OK;
}

const char *trigger_decoder_config_error_string(trigger_config_error_t error)
{
  switch (error)
  {
    case TRIGGER_CONFIG_OK:                      return "configuration valid";
    case TRIGGER_CONFIG_NULL:                    return "null decoder configuration";
    case TRIGGER_CONFIG_TOO_MANY_WHEELS:         return "invalid wheel count";
    case TRIGGER_CONFIG_BAD_PRIMARY:             return "invalid primary wheel index";
    case TRIGGER_CONFIG_BAD_PHASE:               return "invalid phase wheel index";
    case TRIGGER_CONFIG_BAD_CHANNEL:             return "invalid trigger input channel or edge";
    case TRIGGER_CONFIG_DUPLICATE_CHANNEL:       return "two wheels use the same input channel";
    case TRIGGER_CONFIG_BAD_ANGLE:               return "invalid cycle, angle, or direction";
    case TRIGGER_CONFIG_BAD_TIMING_LIMIT:        return "invalid tolerance, filter, or timeout";
    case TRIGGER_CONFIG_BAD_MISSING_TOOTH:       return "invalid missing-tooth geometry";
    case TRIGGER_CONFIG_BAD_PATTERN:             return "invalid LONG/SHORT pattern";
    case TRIGGER_CONFIG_AMBIGUOUS_PATTERN:       return "pattern has indistinguishable cyclic rotations";
    case TRIGGER_CONFIG_BAD_CYCLE_RELATION:      return "wheel cycles do not fit the engine cycle";
    case TRIGGER_CONFIG_BAD_TIMESTAMP_FREQUENCY: return "timestamp frequency is zero";
    default:                                     return "unknown trigger configuration error";
  }
}

static void set_fault(trigger_decoder_t *decoder,
                      trigger_loss_reason_t reason)
{
  if ((uint32_t)reason < 32U)
  {
    decoder->latched_faults |= (1UL << (uint32_t)reason);
  }
}

static void lose_wheel_sync(trigger_decoder_t *decoder,
                            uint8_t wheel,
                            trigger_loss_reason_t reason)
{
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  bool was_synced = state->synced;

  state->synced = false;
  state->reference_candidate = false;
  state->pattern_candidate = false;
  state->reference_confirmations = 0U;
  state->candidate_intervals_remaining = 0U;
  state->intervals_since_reference = 0U;
  state->last_loss_reason = reason;
  set_fault(decoder, reason);

  if (was_synced)
  {
    state->sync_loss_count++;
    state->sync_epoch++;
    /* Only wheels that define actuator position may invalidate an actuator
     * epoch. Auxiliary diagnostic wheels retain their own per-wheel epoch. */
    if ((wheel == decoder->config->primary_wheel) ||
        (wheel == decoder->config->phase_wheel))
    {
      decoder->sync_epoch++;
    }
  }

  if ((wheel == decoder->config->primary_wheel) ||
      (wheel == decoder->config->phase_wheel))
  {
    decoder->phase_known = false;
  }
}

static void establish_sync(trigger_decoder_t *decoder,
                           uint8_t wheel,
                           uint16_t edge_index)
{
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  state->synced = true;
  state->edge_index = edge_index;
  state->last_loss_reason = TRIGGER_LOSS_NONE;
  state->reference_candidate = false;
  state->pattern_candidate = false;
  state->candidate_intervals_remaining = 0U;
  state->sync_epoch++;
}

static bool period_matches(float period,
                           float unit_period,
                           float expected_units,
                           uint16_t tolerance_permille)
{
  float expected = unit_period * expected_units;
  float tolerance = expected * (float)tolerance_permille / PERMILLE_SCALE;
  return absolute_value(period - expected) <= tolerance;
}

static void filter_unit_period(trigger_wheel_state_t *state,
                               const trigger_wheel_config_t *config,
                               float measured_unit_period)
{
  if (state->unit_period_ticks <= 0.0f)
  {
    state->unit_period_ticks = measured_unit_period;
    return;
  }

  float alpha = (float)config->speed_filter_permille / PERMILLE_SCALE;
  state->unit_period_ticks +=
      alpha * (measured_unit_period - state->unit_period_ticks);
}

static void start_missing_reference_candidate(trigger_wheel_state_t *state)
{
  state->reference_candidate = true;
  state->reference_confirmations = 1U;
  state->intervals_since_reference = 0U;
}

static bool process_missing_tooth(trigger_decoder_t *decoder,
                                  uint8_t wheel,
                                  uint32_t period_ticks)
{
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  uint16_t physical_teeth = wheel_edge_count(config);
  float gap_units = wheel_max_interval_units(config);
  float period = (float)period_ticks;

  if (state->unit_period_ticks <= 0.0f)
  {
    state->unit_period_ticks = period;
    return false;
  }

  /* If startup began on the gap, the first normal interval must immediately
   * pull the provisional reference down instead of waiting for the IIR. */
  if (period < state->unit_period_ticks * 0.75f)
  {
    state->unit_period_ticks = period;
  }

  bool normal = period_matches(period, state->unit_period_ticks, 1U,
                               config->interval_tolerance_permille);
  bool gap = period_matches(period, state->unit_period_ticks, gap_units,
                            config->interval_tolerance_permille);

  if (normal && gap)
  {
    float normal_error = absolute_value(period / state->unit_period_ticks - 1.0f);
    float gap_error = absolute_value(period / state->unit_period_ticks -
                                     gap_units) / gap_units;
    gap = gap_error < normal_error;
    normal = !gap;
  }

  if ((!normal) && (!gap))
  {
    state->rejected_event_count++;
    lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_UNEXPECTED_INTERVAL);
    return false;
  }

  if (gap)
  {
    filter_unit_period(state, config, period / gap_units);

    if (state->synced)
    {
      state->intervals_since_reference++;
      if (state->intervals_since_reference != physical_teeth)
      {
        state->rejected_event_count++;
        lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_REFERENCE_COUNT);
        start_missing_reference_candidate(state);
        return false;
      }

      state->edge_index = 0U;
      state->intervals_since_reference = 0U;
      return true;
    }

    if (!state->reference_candidate)
    {
      start_missing_reference_candidate(state);
      return false;
    }

    state->intervals_since_reference++;
    if (state->intervals_since_reference == physical_teeth)
    {
      state->reference_confirmations++;
      state->intervals_since_reference = 0U;
      if (state->reference_confirmations >= config->sync_confirmations)
      {
        establish_sync(decoder, wheel, 0U);
      }
    }
    else
    {
      start_missing_reference_candidate(state);
    }
    return false;
  }

  filter_unit_period(state, config, period);
  if (state->synced)
  {
    state->intervals_since_reference++;
    if (state->intervals_since_reference >= physical_teeth)
    {
      state->rejected_event_count++;
      lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_REFERENCE_COUNT);
      return false;
    }
    state->edge_index++;
  }
  else if (state->reference_candidate)
  {
    state->intervals_since_reference++;
    if (state->intervals_since_reference >= physical_teeth)
    {
      state->reference_candidate = false;
      state->reference_confirmations = 0U;
      state->intervals_since_reference = 0U;
    }
  }
  return false;
}

static void pattern_history_push(trigger_wheel_state_t *state,
                                 uint16_t pattern_length,
                                 uint32_t period_ticks)
{
  if (state->pattern_history_fill < pattern_length)
  {
    state->pattern_period_history[state->pattern_history_fill++] = period_ticks;
    return;
  }

  for (uint16_t index = 1U; index < pattern_length; index++)
  {
    state->pattern_period_history[index - 1U] =
        state->pattern_period_history[index];
  }
  state->pattern_period_history[pattern_length - 1U] = period_ticks;
}

static bool find_pattern_rotation(const trigger_wheel_config_t *config,
                                  const trigger_wheel_state_t *state,
                                  uint16_t *edge_index,
                                  float *unit_period)
{
  uint16_t length = config->geometry.pattern.interval_count;
  uint16_t match_count = 0U;
  uint16_t matched_edge = 0U;
  float matched_period = 0.0f;

  if (state->pattern_history_fill < length)
  {
    return false;
  }

  for (uint16_t candidate = 0U; candidate < length; candidate++)
  {
    float numerator = 0.0f;
    float denominator = 0.0f;
    for (uint16_t sample = 0U; sample < length; sample++)
    {
      float units = (float)pattern_units_at(config,
          (uint16_t)((candidate + sample) % length));
      numerator += (float)state->pattern_period_history[sample] * units;
      denominator += units * units;
    }
    float fitted_period = numerator / denominator;

    bool matches = true;
    for (uint16_t sample = 0U; sample < length; sample++)
    {
      float units = pattern_units_at(config,
          (uint16_t)((candidate + sample) % length));
      if (!period_matches((float)state->pattern_period_history[sample],
                          fitted_period, units,
                          config->interval_tolerance_permille))
      {
        matches = false;
        break;
      }
    }

    if (matches)
    {
      match_count++;
      matched_edge = candidate;
      matched_period = fitted_period;
    }
  }

  if (match_count != 1U)
  {
    return false;
  }

  /* A full history ending at the current event begins with the interval from
   * this same edge index in the previous cycle. */
  *edge_index = matched_edge;
  *unit_period = matched_period;
  return true;
}

static void try_start_pattern_candidate(trigger_decoder_t *decoder,
                                        uint8_t wheel)
{
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  uint16_t edge_index;
  float unit_period;

  if (!find_pattern_rotation(config, state, &edge_index, &unit_period))
  {
    return;
  }

  state->unit_period_ticks = unit_period;
  if (config->sync_confirmations == 1U)
  {
    establish_sync(decoder, wheel, edge_index);
    return;
  }

  state->pattern_candidate = true;
  state->pattern_candidate_edge = edge_index;
  state->candidate_intervals_remaining = (uint16_t)(
      (uint16_t)(config->sync_confirmations - 1U) *
      config->geometry.pattern.interval_count);
}

static bool process_interval_pattern(trigger_decoder_t *decoder,
                                     uint8_t wheel,
                                     uint32_t period_ticks)
{
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  uint16_t length = config->geometry.pattern.interval_count;
  pattern_history_push(state, length, period_ticks);

  if (state->synced)
  {
    float expected_units = pattern_units_at(config, state->edge_index);
    if (!period_matches((float)period_ticks, state->unit_period_ticks,
                        expected_units,
                        config->interval_tolerance_permille))
    {
      state->rejected_event_count++;
      lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_UNEXPECTED_INTERVAL);
      try_start_pattern_candidate(decoder, wheel);
      return false;
    }

    filter_unit_period(state, config,
                       (float)period_ticks / expected_units);
    uint16_t old_index = state->edge_index;
    state->edge_index = (uint16_t)((old_index + 1U) % length);
    return (old_index == (uint16_t)(length - 1U));
  }

  if (state->pattern_candidate)
  {
    float expected_units = pattern_units_at(config,
                                             state->pattern_candidate_edge);
    if (period_matches((float)period_ticks, state->unit_period_ticks,
                       expected_units,
                       config->interval_tolerance_permille))
    {
      filter_unit_period(state, config,
                         (float)period_ticks / expected_units);
      state->pattern_candidate_edge = (uint16_t)(
          (state->pattern_candidate_edge + 1U) % length);
      if (state->candidate_intervals_remaining > 0U)
      {
        state->candidate_intervals_remaining--;
      }
      if (state->candidate_intervals_remaining == 0U)
      {
        establish_sync(decoder, wheel, state->pattern_candidate_edge);
      }
      return false;
    }

    state->pattern_candidate = false;
    state->candidate_intervals_remaining = 0U;
  }

  try_start_pattern_candidate(decoder, wheel);
  return false;
}

static float wheel_angle_at(const trigger_decoder_t *decoder,
                            uint8_t wheel,
                            uint32_t timestamp)
{
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  const trigger_wheel_state_t *state = &decoder->wheel[wheel];
  if ((!state->synced) || (state->unit_period_ticks <= 0.0f))
  {
    return 0.0f;
  }

  uint32_t elapsed_ticks = timestamp - state->last_edge_timestamp;
  if (elapsed_ticks >= TIMESTAMP_HALF_RANGE)
  {
    elapsed_ticks = 0U; /* timestamp was sampled before this edge */
  }

  float units = wheel_next_interval_units(config, state->edge_index);
  float expected_ticks = state->unit_period_ticks * units;
  float fraction = (expected_ticks > 0.0f) ?
                   (float)elapsed_ticks / expected_ticks : 0.0f;
  if (fraction > 1.0f)
  {
    fraction = 1.0f;
  }

  float edge_angle = wheel_edge_angle(config, state->edge_index);
  float advance = wheel_next_interval_degrees(config, state->edge_index) *
                  fraction * (float)config->angle_direction;
  return wrap_angle(edge_angle + advance, config->cycle_degrees);
}

static void update_phase_from_event(trigger_decoder_t *decoder,
                                    uint8_t wheel,
                                    uint32_t timestamp)
{
  const trigger_decoder_config_t *config = decoder->config;
  if ((config->phase_wheel == TRIGGER_NO_WHEEL) ||
      (wheel != config->phase_wheel) ||
      (!decoder->wheel[wheel].synced) ||
      (!decoder->wheel[config->primary_wheel].synced))
  {
    return;
  }

  const trigger_wheel_config_t *primary_config =
      &config->wheels[config->primary_wheel];
  uint8_t turns = (uint8_t)lroundf(config->engine_cycle_degrees /
                                   primary_config->cycle_degrees);
  float primary_angle = wheel_angle_at(decoder, config->primary_wheel,
                                       timestamp);
  float phase_angle = wheel_edge_angle(&config->wheels[wheel],
                                       decoder->wheel[wheel].edge_index);

  uint8_t best_turn = 0U;
  float best_error = config->engine_cycle_degrees;
  for (uint8_t turn = 0U; turn < turns; turn++)
  {
    float candidate = primary_angle +
                      (float)turn * primary_config->cycle_degrees;
    float error = circular_distance(candidate, phase_angle,
                                    config->engine_cycle_degrees);
    if (error < best_error)
    {
      best_error = error;
      best_turn = turn;
    }
  }

  if (best_error > config->phase_alignment_tolerance_deg)
  {
    lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_PHASE_ALIGNMENT);
    return;
  }

  /* phase_turn describes the revolution containing the last *physical*
   * primary edge.  wheel_angle_at() can already be beyond the coordinate
   * wrap while waiting for that next edge; poll() applies this virtual wrap,
   * and primary_coordinate_wrap_update() commits it when the edge arrives. */
  float primary_edge_angle = wheel_edge_angle(
      primary_config, decoder->wheel[config->primary_wheel].edge_index);
  bool interpolated_wrap = (primary_config->angle_direction > 0) ?
      (primary_angle < primary_edge_angle) :
      (primary_angle > primary_edge_angle);
  if (interpolated_wrap)
  {
    best_turn = (primary_config->angle_direction > 0) ?
        (uint8_t)((best_turn + turns - 1U) % turns) :
        (uint8_t)((best_turn + 1U) % turns);
  }

  decoder->phase_turn = best_turn;
  decoder->phase_known = true;
}

static void primary_coordinate_wrap_update(trigger_decoder_t *decoder,
                                           uint16_t previous_edge_index,
                                           uint16_t current_edge_index)
{
  if (!decoder->phase_known)
  {
    return;
  }

  const trigger_decoder_config_t *config = decoder->config;
  const trigger_wheel_config_t *primary =
      &config->wheels[config->primary_wheel];
  float previous_angle = wheel_edge_angle(primary, previous_edge_index);
  float current_angle = wheel_edge_angle(primary, current_edge_index);
  bool coordinate_wrapped = (primary->angle_direction > 0) ?
      (current_angle < previous_angle) : (current_angle > previous_angle);

  if (!coordinate_wrapped)
  {
    return;
  }

  uint8_t turns = (uint8_t)lroundf(config->engine_cycle_degrees /
                                   primary->cycle_degrees);
  if (primary->angle_direction > 0)
  {
    decoder->phase_turn = (uint8_t)((decoder->phase_turn + 1U) % turns);
  }
  else
  {
    decoder->phase_turn = (uint8_t)((decoder->phase_turn + turns - 1U) % turns);
  }
}

void trigger_decoder_process_event(trigger_decoder_t *decoder,
                                   const trigger_event_t *event)
{
  if ((decoder == NULL) || (event == NULL) ||
      (decoder->config_error != TRIGGER_CONFIG_OK) ||
      (event->channel == 0U) || (event->channel >= TRIGGER_CHANNEL_COUNT))
  {
    return;
  }

  int8_t mapped = decoder->channel_to_wheel[event->channel];
  if (mapped < 0)
  {
    return;
  }

  uint8_t wheel = (uint8_t)mapped;
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  trigger_wheel_state_t *state = &decoder->wheel[wheel];
  bool was_synced = state->synced;
  uint16_t previous_edge_index = state->edge_index;
  uint32_t previous_sync_epoch = state->sync_epoch;

  if (state->have_sequence)
  {
    uint16_t expected = (uint16_t)(state->last_sequence + 1U);
    if (event->sequence != expected)
    {
      uint16_t difference = (uint16_t)(event->sequence - expected);
      state->dropped_event_count += (difference == 0U) ? 1U : difference;
      lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_EVENT_DROPPED);
      state->have_timestamp = false; /* interval across the gap is unusable */
    }
  }
  state->last_sequence = event->sequence;
  state->have_sequence = true;

  if ((((uint8_t)config->active_edge & (uint8_t)event->edge) == 0U) ||
      ((event->edge != TRIGGER_EDGE_RISING) &&
       (event->edge != TRIGGER_EDGE_FALLING)))
  {
    return;
  }

  if (!state->have_timestamp)
  {
    state->last_edge_timestamp = event->timestamp;
    state->have_timestamp = true;
    return;
  }

  uint32_t period_ticks = event->timestamp - state->last_edge_timestamp;
  if ((period_ticks == 0U) || (period_ticks >= TIMESTAMP_HALF_RANGE))
  {
    state->rejected_event_count++;
    lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_TIMESTAMP_ORDER);
    state->last_edge_timestamp = event->timestamp;
    return;
  }

  if (period_ticks < state->minimum_interval_ticks)
  {
    /* Treat a too-close edge as electrical noise.  Keep the previous valid
     * timestamp, otherwise the glitch would also corrupt the next interval. */
    state->rejected_event_count++;
    lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_INTERVAL_TOO_SHORT);
    return;
  }

  state->last_period_ticks = period_ticks;
  state->last_edge_timestamp = event->timestamp;
  state->accepted_event_count++;

  if (config->type == TRIGGER_WHEEL_MISSING_TOOTH)
  {
    (void)process_missing_tooth(decoder, wheel, period_ticks);
  }
  else
  {
    (void)process_interval_pattern(decoder, wheel, period_ticks);
  }

  if ((wheel == decoder->config->primary_wheel) && was_synced &&
      state->synced && (state->sync_epoch == previous_sync_epoch))
  {
    primary_coordinate_wrap_update(decoder, previous_edge_index,
                                   state->edge_index);
  }
  update_phase_from_event(decoder, wheel, event->timestamp);
}

void trigger_decoder_note_event_loss(trigger_decoder_t *decoder,
                                     uint8_t channel,
                                     uint32_t lost_event_count)
{
  if ((decoder == NULL) || (decoder->config_error != TRIGGER_CONFIG_OK) ||
      (channel == 0U) || (channel >= TRIGGER_CHANNEL_COUNT) ||
      (lost_event_count == 0U))
  {
    return;
  }

  int8_t mapped = decoder->channel_to_wheel[channel];
  if (mapped >= 0)
  {
    trigger_wheel_state_t *state = &decoder->wheel[(uint8_t)mapped];
    state->dropped_event_count += lost_event_count;
    state->have_timestamp = false;
    lose_wheel_sync(decoder, (uint8_t)mapped, TRIGGER_LOSS_EVENT_DROPPED);
  }
}

static uint32_t wheel_timeout_ticks(const trigger_wheel_config_t *config,
                                    const trigger_wheel_state_t *state)
{
  float units = state->synced ?
      wheel_next_interval_units(config, state->edge_index) :
      wheel_max_interval_units(config);
  float adaptive = state->unit_period_ticks * units *
                   config->timeout_interval_multiplier;
  uint32_t adaptive_ticks = (adaptive >= (float)(TIMESTAMP_HALF_RANGE - 1UL)) ?
                            (TIMESTAMP_HALF_RANGE - 1UL) : (uint32_t)adaptive;
  return (adaptive_ticks > state->minimum_timeout_ticks) ?
         adaptive_ticks : state->minimum_timeout_ticks;
}

static float wheel_rpm(const trigger_decoder_t *decoder, uint8_t wheel)
{
  const trigger_wheel_config_t *config = &decoder->config->wheels[wheel];
  const trigger_wheel_state_t *state = &decoder->wheel[wheel];
  if ((!state->synced) || (state->unit_period_ticks <= 0.0f))
  {
    return 0.0f;
  }

  float cycle_ticks = state->unit_period_ticks *
                      wheel_total_interval_units(config);
  return 60.0f * (float)decoder->timestamp_frequency_hz *
         (config->cycle_degrees / 360.0f) / cycle_ticks;
}

void trigger_decoder_poll(trigger_decoder_t *decoder,
                          uint32_t now,
                          trigger_output_t *output)
{
  if (output == NULL)
  {
    return;
  }
  memset(output, 0, sizeof(*output));

  if ((decoder == NULL) || (decoder->config_error != TRIGGER_CONFIG_OK))
  {
    return;
  }

  const trigger_decoder_config_t *config = decoder->config;
  for (uint8_t wheel = 0U; wheel < config->wheel_count; wheel++)
  {
    const trigger_wheel_config_t *wheel_config = &config->wheels[wheel];
    trigger_wheel_state_t *state = &decoder->wheel[wheel];
    if (state->have_timestamp && (state->unit_period_ticks > 0.0f))
    {
      uint32_t age = now - state->last_edge_timestamp;
      if ((age >= TIMESTAMP_HALF_RANGE) ||
          (age > wheel_timeout_ticks(wheel_config, state)))
      {
        lose_wheel_sync(decoder, wheel, TRIGGER_LOSS_TIMEOUT);
        /* The modular age is ambiguous after half a timer range.  The next
         * physical edge must establish a fresh timestamp baseline. */
        state->have_timestamp = false;
        state->unit_period_ticks = 0.0f;
        state->pattern_history_fill = 0U;
      }
    }

    trigger_wheel_output_t *wheel_output = &output->wheel[wheel];
    wheel_output->synced = state->synced;
    wheel_output->edge_index = state->edge_index;
    wheel_output->last_edge_timestamp = state->last_edge_timestamp;
    wheel_output->sync_epoch = state->sync_epoch;
    wheel_output->last_loss_reason = state->last_loss_reason;
    wheel_output->accepted_event_count = state->accepted_event_count;
    wheel_output->rejected_event_count = state->rejected_event_count;
    wheel_output->dropped_event_count = state->dropped_event_count;
    wheel_output->sync_loss_count = state->sync_loss_count;
    wheel_output->angle_deg = wheel_angle_at(decoder, wheel, now);
    wheel_output->rpm = wheel_rpm(decoder, wheel);

    uint8_t channel = wheel_config->input_channel;
    output->angle[channel] = wheel_output->angle_deg;
    output->ch_synced[channel] = wheel_output->synced;
  }

  uint8_t primary = config->primary_wheel;
  output->wheel_count = config->wheel_count;
  output->synced = output->wheel[primary].synced;
  output->rpm = output->wheel[primary].rpm;
  output->phase_known = decoder->phase_known && output->synced &&
      ((config->phase_wheel == TRIGGER_NO_WHEEL) ||
       output->wheel[config->phase_wheel].synced);

  float primary_angle = output->wheel[primary].angle_deg;
  if (output->phase_known)
  {
    const trigger_wheel_config_t *primary_config = &config->wheels[primary];
    uint8_t output_turn = decoder->phase_turn;
    float edge_angle = wheel_edge_angle(primary_config,
                                        decoder->wheel[primary].edge_index);
    bool interpolated_wrap = (primary_config->angle_direction > 0) ?
        (primary_angle < edge_angle) : (primary_angle > edge_angle);

    if (interpolated_wrap)
    {
      uint8_t turns = (uint8_t)lroundf(config->engine_cycle_degrees /
                                       primary_config->cycle_degrees);
      output_turn = (primary_config->angle_direction > 0) ?
          (uint8_t)((output_turn + 1U) % turns) :
          (uint8_t)((output_turn + turns - 1U) % turns);
    }
    primary_angle += (float)output_turn * primary_config->cycle_degrees;
  }
  output->crank_angle_deg = wrap_angle(primary_angle,
                                       output->phase_known ?
                                       config->engine_cycle_degrees :
                                       config->wheels[primary].cycle_degrees);
  output->sync_epoch = decoder->sync_epoch;
  output->latched_faults = decoder->latched_faults;
}

void trigger_decoder_force_sync_loss(trigger_decoder_t *decoder,
                                     trigger_loss_reason_t reason)
{
  if ((decoder == NULL) || (decoder->config_error != TRIGGER_CONFIG_OK))
  {
    return;
  }

  if (reason == TRIGGER_LOSS_NONE)
  {
    reason = TRIGGER_LOSS_FORCED;
  }
  for (uint8_t wheel = 0U; wheel < decoder->config->wheel_count; wheel++)
  {
    lose_wheel_sync(decoder, wheel, reason);
    decoder->wheel[wheel].have_timestamp = false;
    decoder->wheel[wheel].unit_period_ticks = 0.0f;
    decoder->wheel[wheel].pattern_history_fill = 0U;
  }
  decoder->phase_known = false;
}

/* --------------------------- Legacy adapter ----------------------------- */

static void legacy_channel_snapshot(const trigger_input_t *input,
                                    uint8_t channel,
                                    uint32_t *count,
                                    uint32_t *timestamp)
{
  uint32_t before;
  uint32_t after;
  do
  {
    before = input->edge_count[channel];
    *timestamp = input->last_edge_cyc[channel];
    after = input->edge_count[channel];
  } while (before != after);
  *count = after;
}

void trigger_decoder_init(void)
{
  memset(legacy_seen_count, 0, sizeof(legacy_seen_count));
  (void)trigger_decoder_context_init(&legacy_decoder,
                                     trigger_decoder_default_config(),
                                     1U);
}

void trigger_decoder_update(const trigger_input_t *input,
                            trigger_output_t *output)
{
  if ((input == NULL) || (output == NULL))
  {
    return;
  }

  if ((legacy_decoder.timestamp_frequency_hz != input->cpu_hz) &&
      (input->cpu_hz != 0U))
  {
    (void)trigger_decoder_context_init(&legacy_decoder,
                                       trigger_decoder_default_config(),
                                       input->cpu_hz);
    memset(legacy_seen_count, 0, sizeof(legacy_seen_count));
  }

  for (uint8_t channel = 1U; channel < TRIGGER_CHANNEL_COUNT; channel++)
  {
    uint32_t count;
    uint32_t timestamp;
    legacy_channel_snapshot(input, channel, &count, &timestamp);
    uint32_t difference = count - legacy_seen_count[channel];
    if (difference == 0U)
    {
      continue;
    }

    if (difference > 1U)
    {
      trigger_decoder_note_event_loss(&legacy_decoder, channel,
                                      difference - 1U);
    }

    trigger_event_t event = {
      .timestamp = timestamp,
      .sequence = (uint16_t)count,
      .channel = channel,
      .edge = TRIGGER_EDGE_RISING,
    };
    trigger_decoder_process_event(&legacy_decoder, &event);
    legacy_seen_count[channel] = count;
  }

  trigger_decoder_poll(&legacy_decoder, input->now_cyc, output);
}
