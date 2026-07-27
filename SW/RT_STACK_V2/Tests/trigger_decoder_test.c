/* Host tests for the hardware-independent trigger decoder and recorder.
 * Build example (from RT_STACK_V2):
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc \
 *       Tests/trigger_decoder_test.c Core/Src/trigger_decoder.c \
 *       Core/Src/trigger_recorder.c -lm -o trigger_decoder_test
 */
#include "trigger_decoder.h"
#include "trigger_recorder.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

static unsigned tests_run;

static void fail(const char *expression, const char *file, int line)
{
  (void)fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expression);
  exit(EXIT_FAILURE);
}

#define CHECK(expression) \
  do { tests_run++; if (!(expression)) fail(#expression, __FILE__, __LINE__); } while (0)

static void check_near(float actual, float expected, float tolerance,
                       const char *file, int line)
{
  tests_run++;
  if ((!isfinite(actual)) || (fabsf(actual - expected) > tolerance))
  {
    (void)fprintf(stderr,
                  "FAIL %s:%d: actual %.6f, expected %.6f +/- %.6f\n",
                  file, line, (double)actual, (double)expected,
                  (double)tolerance);
    exit(EXIT_FAILURE);
  }
}

#define CHECK_NEAR(actual, expected, tolerance) \
  check_near((actual), (expected), (tolerance), __FILE__, __LINE__)

static const trigger_wheel_config_t missing_wheel =
{
  .name = "test 12-2",
  .input_channel = 1U,
  .active_edge = TRIGGER_EDGE_RISING,
  .type = TRIGGER_WHEEL_MISSING_TOOTH,
  .cycle_degrees = 360.0f,
  .angle_at_index0_deg = 0.0f,
  .angle_direction = +1,
  .interval_tolerance_permille = 250U,
  .speed_filter_permille = 250U,
  .sync_confirmations = 2U,
  .minimum_interval_us = 100U,
  .minimum_timeout_us = 10000U,
  .timeout_interval_multiplier = 2.0f,
  .geometry.missing_tooth = {
    .tooth_positions = 12U,
    .missing_teeth = 2U,
  },
};

static const trigger_decoder_config_t missing_config =
{
  .wheels = &missing_wheel,
  .wheel_count = 1U,
  .primary_wheel = 0U,
  .phase_wheel = TRIGGER_NO_WHEEL,
  .engine_cycle_degrees = 720.0f,
};

typedef struct
{
  trigger_decoder_t decoder;
  uint32_t timestamp;
  uint16_t sequence;
} fixture_t;

static void fixture_init(fixture_t *fixture,
                         const trigger_decoder_config_t *config,
                         uint32_t initial_timestamp)
{
  fixture->timestamp = initial_timestamp;
  fixture->sequence = 0U;
  CHECK(trigger_decoder_context_init(&fixture->decoder, config, 1000000U) ==
        TRIGGER_CONFIG_OK);
}

static void emit_period(fixture_t *fixture, uint8_t channel, uint32_t period)
{
  fixture->timestamp += period;
  trigger_event_t event = {
    .timestamp = fixture->timestamp,
    .sequence = fixture->sequence++,
    .channel = channel,
    .edge = TRIGGER_EDGE_RISING,
  };
  trigger_decoder_process_event(&fixture->decoder, &event);
}

static void emit_missing_revolution(fixture_t *fixture)
{
  /* Current event is edge zero.  Nine short intervals reach edge nine; the
   * tenth physical interval is the 3-unit gap back to edge zero. */
  for (unsigned edge = 1U; edge < 10U; edge++)
  {
    emit_period(fixture, 1U, 1000U);
  }
  emit_period(fixture, 1U, 3000U);
}

static void acquire_missing_sync(fixture_t *fixture, uint32_t initial_timestamp)
{
  fixture_init(fixture, &missing_config, initial_timestamp);
  emit_period(fixture, 1U, 0U); /* first event establishes timestamp only */
  emit_missing_revolution(fixture);
  emit_missing_revolution(fixture);
}

static void test_configuration_validation(void)
{
  trigger_decoder_t decoder;
  CHECK(trigger_decoder_context_init(&decoder, NULL, 1000000U) ==
        TRIGGER_CONFIG_NULL);
  CHECK(trigger_decoder_context_init(&decoder, &missing_config, 0U) ==
        TRIGGER_CONFIG_BAD_TIMESTAMP_FREQUENCY);

  static const trigger_interval_class_t ambiguous[] = {
    TRIGGER_INTERVAL_SHORT, TRIGGER_INTERVAL_LONG,
    TRIGGER_INTERVAL_SHORT, TRIGGER_INTERVAL_LONG,
  };
  trigger_wheel_config_t wheel = missing_wheel;
  wheel.type = TRIGGER_WHEEL_INTERVAL_PATTERN;
  wheel.geometry.pattern.intervals = ambiguous;
  wheel.geometry.pattern.interval_count = (uint16_t)ARRAY_COUNT(ambiguous);
  wheel.geometry.pattern.long_interval_units = 2.0f;
  trigger_decoder_config_t config = missing_config;
  config.wheels = &wheel;
  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_AMBIGUOUS_PATTERN);

  static const trigger_interval_class_t unique_but_overlapping[] = {
    TRIGGER_INTERVAL_SHORT, TRIGGER_INTERVAL_SHORT,
    TRIGGER_INTERVAL_LONG, TRIGGER_INTERVAL_LONG,
  };
  wheel.geometry.pattern.intervals = unique_but_overlapping;
  wheel.geometry.pattern.interval_count =
      (uint16_t)ARRAY_COUNT(unique_but_overlapping);
  wheel.geometry.pattern.long_interval_units = 1.2f;
  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_BAD_TIMING_LIMIT);
}

static void test_missing_tooth_sync_speed_and_interpolation(void)
{
  fixture_t fixture;
  acquire_missing_sync(&fixture, 100U);

  trigger_output_t output;
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp, &output);
  CHECK(output.synced);
  CHECK(!output.phase_known);
  CHECK(output.wheel[0].edge_index == 0U);
  CHECK_NEAR(output.crank_angle_deg, 0.0f, 0.01f);
  CHECK_NEAR(output.rpm, 5000.0f, 0.5f);

  trigger_decoder_poll(&fixture.decoder, fixture.timestamp + 500U, &output);
  CHECK_NEAR(output.crank_angle_deg, 15.0f, 0.1f);

  /* Walk to the final physical tooth; interpolation across the missing gap
   * spans 90 degrees, not one ordinary 30-degree tooth. */
  for (unsigned edge = 1U; edge < 10U; edge++)
  {
    emit_period(&fixture, 1U, 1000U);
  }
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp + 1500U, &output);
  CHECK(output.wheel[0].edge_index == 9U);
  CHECK_NEAR(output.crank_angle_deg, 315.0f, 0.2f);
}

static void test_missing_tooth_wrong_count_loses_sync(void)
{
  fixture_t fixture;
  acquire_missing_sync(&fixture, 100U);

  /* A false gap after only three normal intervals must not be accepted as the
   * reference, even though its ratio is exactly the configured gap ratio. */
  emit_period(&fixture, 1U, 1000U);
  emit_period(&fixture, 1U, 1000U);
  emit_period(&fixture, 1U, 1000U);
  emit_period(&fixture, 1U, 3000U);

  trigger_output_t output;
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp, &output);
  CHECK(!output.synced);
  CHECK(output.wheel[0].last_loss_reason == TRIGGER_LOSS_REFERENCE_COUNT);
  CHECK(output.wheel[0].sync_loss_count == 1U);
}

static void test_event_loss_and_timeout(void)
{
  fixture_t fixture;
  acquire_missing_sync(&fixture, 100U);

  /* Skip one producer sequence number. */
  fixture.sequence++;
  emit_period(&fixture, 1U, 2000U);

  trigger_output_t output;
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp, &output);
  CHECK(!output.synced);
  CHECK(output.wheel[0].last_loss_reason == TRIGGER_LOSS_EVENT_DROPPED);
  CHECK(output.wheel[0].dropped_event_count == 1U);

  acquire_missing_sync(&fixture, 100U);
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp + 10001U, &output);
  CHECK(!output.synced);
  CHECK(output.wheel[0].last_loss_reason == TRIGGER_LOSS_TIMEOUT);

  acquire_missing_sync(&fixture, 100U);
  trigger_decoder_poll(&fixture.decoder,
                       fixture.timestamp + 0x80000000UL, &output);
  CHECK(!output.synced);
  CHECK(output.wheel[0].last_loss_reason == TRIGGER_LOSS_TIMEOUT);
}

static const trigger_interval_class_t ssll_pattern[] =
{
  TRIGGER_INTERVAL_SHORT,
  TRIGGER_INTERVAL_SHORT,
  TRIGGER_INTERVAL_LONG,
  TRIGGER_INTERVAL_LONG,
};

static const trigger_wheel_config_t pattern_wheel =
{
  .name = "test S-S-L-L",
  .input_channel = 2U,
  .active_edge = TRIGGER_EDGE_RISING,
  .type = TRIGGER_WHEEL_INTERVAL_PATTERN,
  .cycle_degrees = 720.0f,
  .angle_at_index0_deg = 0.0f,
  .angle_direction = +1,
  .interval_tolerance_permille = 200U,
  .speed_filter_permille = 250U,
  .sync_confirmations = 1U,
  .minimum_interval_us = 100U,
  .minimum_timeout_us = 10000U,
  .timeout_interval_multiplier = 2.0f,
  .geometry.pattern = {
    .intervals = ssll_pattern,
    .interval_count = (uint16_t)ARRAY_COUNT(ssll_pattern),
    .long_interval_units = 2.0f,
  },
};

static const trigger_decoder_config_t pattern_config =
{
  .wheels = &pattern_wheel,
  .wheel_count = 1U,
  .primary_wheel = 0U,
  .phase_wheel = TRIGGER_NO_WHEEL,
  .engine_cycle_degrees = 720.0f,
};

static void test_interval_pattern_sync(void)
{
  fixture_t fixture;
  fixture_init(&fixture, &pattern_config, 0U);
  emit_period(&fixture, 2U, 0U);     /* edge zero */
  emit_period(&fixture, 2U, 1000U);  /* edge one  */
  emit_period(&fixture, 2U, 1000U);  /* edge two  */
  emit_period(&fixture, 2U, 2000U);  /* edge three */
  emit_period(&fixture, 2U, 2000U);  /* edge zero */

  trigger_output_t output;
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp, &output);
  CHECK(output.synced);
  CHECK(output.wheel[0].edge_index == 0U);
  CHECK_NEAR(output.crank_angle_deg, 0.0f, 0.01f);
  CHECK_NEAR(output.rpm, 20000.0f, 1.0f);

  trigger_decoder_poll(&fixture.decoder, fixture.timestamp + 500U, &output);
  CHECK_NEAR(output.crank_angle_deg, 60.0f, 0.2f);

  emit_period(&fixture, 2U, 2000U); /* expected SHORT: deliberately wrong */
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp, &output);
  CHECK(!output.synced);
  CHECK(output.wheel[0].last_loss_reason ==
        TRIGGER_LOSS_UNEXPECTED_INTERVAL);
}

static void test_timestamp_wrap(void)
{
  fixture_t fixture;
  acquire_missing_sync(&fixture, UINT32_MAX - 8000U);

  trigger_output_t output;
  trigger_decoder_poll(&fixture.decoder, fixture.timestamp + 500U, &output);
  CHECK(output.synced);
  CHECK_NEAR(output.rpm, 5000.0f, 0.5f);
  CHECK_NEAR(output.crank_angle_deg, 15.0f, 0.1f);
}

static void test_crank_cam_phase_fusion(void)
{
  trigger_wheel_config_t wheels[2] = {missing_wheel, pattern_wheel};
  wheels[1].minimum_timeout_us = 20000U;
  trigger_decoder_config_t config = {
    .wheels = wheels,
    .wheel_count = 2U,
    .primary_wheel = 0U,
    .phase_wheel = 1U,
    .engine_cycle_degrees = 720.0f,
    .phase_alignment_tolerance_deg = 30.0f,
  };
  trigger_decoder_t decoder;
  config.phase_alignment_tolerance_deg = 180.0f;
  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_BAD_TIMING_LIMIT);
  config.phase_alignment_tolerance_deg = 30.0f;
  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_OK);

  uint16_t crank_sequence = 0U;
  uint16_t cam_sequence = 0U;
  for (uint32_t time_ms = 0U; time_ms <= 36U; time_ms++)
  {
    uint32_t revolution_time = time_ms % 12U;
    bool crank_edge = (revolution_time <= 9U);
    if (crank_edge)
    {
      trigger_event_t event = {
        .timestamp = time_ms * 1000U,
        .sequence = crank_sequence++,
        .channel = 1U,
        .edge = TRIGGER_EDGE_RISING,
      };
      trigger_decoder_process_event(&decoder, &event);
    }

    bool cam_edge = (time_ms == 0U) || (time_ms == 4U) ||
                    (time_ms == 8U) || (time_ms == 16U) ||
                    (time_ms == 24U) || (time_ms == 28U) ||
                    (time_ms == 32U);
    if (cam_edge)
    {
      trigger_event_t event = {
        .timestamp = time_ms * 1000U,
        .sequence = cam_sequence++,
        .channel = 2U,
        .edge = TRIGGER_EDGE_RISING,
      };
      trigger_decoder_process_event(&decoder, &event);
    }

    if (time_ms == 24U)
    {
      trigger_output_t output;
      trigger_decoder_poll(&decoder, time_ms * 1000U, &output);
      CHECK(output.synced);
      CHECK(output.phase_known);
      CHECK_NEAR(output.crank_angle_deg, 0.0f, 0.1f);
    }
  }

  trigger_output_t output;
  trigger_decoder_poll(&decoder, 36000U, &output);
  CHECK(output.phase_known);
  CHECK_NEAR(output.crank_angle_deg, 360.0f, 0.1f);
}

static void test_nonzero_index_angle_and_late_gap_are_continuous(void)
{
  trigger_wheel_config_t wheels[2] = {missing_wheel, pattern_wheel};
  wheels[0].angle_at_index0_deg = 30.0f;
  wheels[1].minimum_timeout_us = 20000U;
  trigger_decoder_config_t config = {
    .wheels = wheels,
    .wheel_count = 2U,
    .primary_wheel = 0U,
    .phase_wheel = 1U,
    .engine_cycle_degrees = 720.0f,
    .phase_alignment_tolerance_deg = 30.0f,
  };
  trigger_decoder_t decoder;
  uint16_t crank_sequence = 0U;
  uint16_t cam_sequence = 0U;

  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_OK);
  for (uint32_t time_ms = 0U; time_ms <= 33U; ++time_ms)
  {
    uint32_t revolution_time = time_ms % 12U;
    if (revolution_time <= 9U)
    {
      trigger_event_t crank = {
        .timestamp = time_ms * 1000U,
        .sequence = crank_sequence++,
        .channel = 1U,
        .edge = TRIGGER_EDGE_RISING,
      };
      trigger_decoder_process_event(&decoder, &crank);
    }

    if ((time_ms == 0U) || (time_ms == 4U) || (time_ms == 8U) ||
        (time_ms == 16U) || (time_ms == 24U) || (time_ms == 28U) ||
        (time_ms == 32U))
    {
      trigger_event_t cam = {
        .timestamp = time_ms * 1000U,
        .sequence = cam_sequence++,
        .channel = 2U,
        .edge = TRIGGER_EDGE_RISING,
      };
      trigger_decoder_process_event(&decoder, &cam);
    }
  }

  trigger_output_t output;
  trigger_decoder_poll(&decoder, 34000U, &output);
  CHECK(output.phase_known);
  CHECK_NEAR(output.crank_angle_deg, 330.0f, 0.2f);
  trigger_decoder_poll(&decoder, 35000U, &output);
  CHECK_NEAR(output.crank_angle_deg, 360.0f, 0.2f);
  trigger_decoder_poll(&decoder, 36000U, &output);
  CHECK_NEAR(output.crank_angle_deg, 390.0f, 0.2f);

  /* Deliver the reference edge 10% late.  The predicted value at the clamped
   * interval end and the post-edge value must be identical, not 30 vs 390. */
  trigger_event_t late_gap = {
    .timestamp = 36300U,
    .sequence = crank_sequence++,
    .channel = 1U,
    .edge = TRIGGER_EDGE_RISING,
  };
  trigger_decoder_process_event(&decoder, &late_gap);
  trigger_decoder_poll(&decoder, late_gap.timestamp, &output);
  CHECK(output.phase_known);
  CHECK_NEAR(output.crank_angle_deg, 390.0f, 0.2f);
}

static void test_phase_acquisition_during_interpolated_wrap(void)
{
  trigger_wheel_config_t wheels[2] = {missing_wheel, pattern_wheel};
  wheels[0].angle_at_index0_deg = 80.0f;
  wheels[1].angle_at_index0_deg = 260.0f;
  wheels[1].minimum_timeout_us = 20000U;
  trigger_decoder_config_t config = {
    .wheels = wheels,
    .wheel_count = 2U,
    .primary_wheel = 0U,
    .phase_wheel = 1U,
    .engine_cycle_degrees = 720.0f,
    .phase_alignment_tolerance_deg = 30.0f,
  };
  trigger_decoder_t decoder;
  CHECK(trigger_decoder_context_init(&decoder, &config, 1000000U) ==
        TRIGGER_CONFIG_OK);

  /* Establish a precise internal position: primary edge 9 is 350 degrees,
   * and its three-unit gap reaches edge zero at physical 440 degrees.  The
   * phase edge is observed one unit into that gap at physical 380 degrees. */
  decoder.wheel[0].synced = true;
  decoder.wheel[0].have_timestamp = true;
  decoder.wheel[0].have_sequence = true;
  decoder.wheel[0].edge_index = 9U;
  decoder.wheel[0].intervals_since_reference = 9U;
  decoder.wheel[0].last_sequence = 0U;
  decoder.wheel[0].last_edge_timestamp = 10000U;
  decoder.wheel[0].unit_period_ticks = 1000.0f;

  decoder.wheel[1].synced = true;
  decoder.wheel[1].have_timestamp = true;
  decoder.wheel[1].have_sequence = true;
  decoder.wheel[1].edge_index = 0U;
  decoder.wheel[1].last_sequence = 0U;
  decoder.wheel[1].last_edge_timestamp = 10000U;
  decoder.wheel[1].unit_period_ticks = 1000.0f;

  trigger_event_t phase = {
    .timestamp = 11000U,
    .sequence = 1U,
    .channel = 2U,
    .edge = TRIGGER_EDGE_RISING,
  };
  trigger_decoder_process_event(&decoder, &phase);

  trigger_output_t output;
  trigger_decoder_poll(&decoder, phase.timestamp, &output);
  CHECK(output.phase_known);
  CHECK_NEAR(output.crank_angle_deg, 380.0f, 0.2f);

  trigger_event_t next_primary = {
    .timestamp = 13000U,
    .sequence = 1U,
    .channel = 1U,
    .edge = TRIGGER_EDGE_RISING,
  };
  trigger_decoder_process_event(&decoder, &next_primary);
  trigger_decoder_poll(&decoder, next_primary.timestamp, &output);
  CHECK(output.phase_known);
  CHECK_NEAR(output.crank_angle_deg, 440.0f, 0.2f);
}

static void test_recorder_order_and_overflow(void)
{
  trigger_recorder_t recorder;
  trigger_recorder_init(&recorder);

  CHECK(trigger_recorder_record_isr(&recorder, 1U, TRIGGER_EDGE_RISING,
                                    0x00000010UL));
  CHECK(trigger_recorder_record_isr(&recorder, 2U, TRIGGER_EDGE_RISING,
                                    0xfffffff0UL));

  trigger_event_t event;
  CHECK(trigger_recorder_pop_oldest(&recorder, &event));
  CHECK(event.channel == 2U);
  CHECK(trigger_recorder_pop_oldest(&recorder, &event));
  CHECK(event.channel == 1U);
  CHECK(!trigger_recorder_pop_oldest(&recorder, &event));

  trigger_recorder_init(&recorder);
  for (uint32_t index = 0U; index < TRIGGER_RECORDER_QUEUE_LENGTH - 1U; index++)
  {
    CHECK(trigger_recorder_record_isr(&recorder, 1U, TRIGGER_EDGE_RISING,
                                      index));
  }
  CHECK(!trigger_recorder_record_isr(&recorder, 1U, TRIGGER_EDGE_RISING,
                                     100U));
  CHECK(trigger_recorder_dropped(&recorder, 1U) == 1U);
  CHECK(trigger_recorder_pending(&recorder) ==
        (TRIGGER_RECORDER_QUEUE_LENGTH - 1U));

  CHECK(trigger_recorder_pop_oldest(&recorder, &event));
  CHECK(trigger_recorder_record_isr(&recorder, 1U, TRIGGER_EDGE_RISING,
                                    101U));
  while (trigger_recorder_pop_oldest(&recorder, &event))
  {
    /* Drain. */
  }
  CHECK(trigger_recorder_pending(&recorder) == 0U);
  CHECK(event.sequence == TRIGGER_RECORDER_QUEUE_LENGTH);
}

int main(void)
{
  test_configuration_validation();
  test_missing_tooth_sync_speed_and_interpolation();
  test_missing_tooth_wrong_count_loses_sync();
  test_event_loss_and_timeout();
  test_interval_pattern_sync();
  test_timestamp_wrap();
  test_crank_cam_phase_fusion();
  test_nonzero_index_angle_and_late_gap_are_continuous();
  test_phase_acquisition_during_interpolated_wrap();
  test_recorder_order_and_overflow();

  (void)printf("PASS: %u trigger checks\n", tests_run);
  return EXIT_SUCCESS;
}
