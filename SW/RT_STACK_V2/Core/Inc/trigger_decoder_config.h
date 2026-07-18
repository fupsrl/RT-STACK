/**
 ******************************************************************************
 * @file    trigger_decoder_config.h
 * @brief   User-editable trigger wheel configuration.
 *
 * This is the only file a normal user must edit to select trigger wheels.
 * The decoder supports any number of wheels up to TRIGGER_MAX_WHEELS.  Each
 * wheel can be a conventional missing-tooth wheel or an arbitrary cyclic
 * LONG/SHORT interval pattern.
 *
 * Pattern convention:
 *   intervals[i] is the interval FROM edge i TO edge i+1.
 *   angle_at_index0_deg is the angle exactly at edge zero.
 *
 * The example cam below is intentionally disabled in trigger_wheels[].  Its
 * mechanical angles are illustrative; enable it only after measuring the
 * installed engine.
 ******************************************************************************
 */
#ifndef TRIGGER_DECODER_CONFIG_H
#define TRIGGER_DECODER_CONFIG_H

#include "trigger_decoder.h"

/* ----------------------- Reusable interval patterns ---------------------- */

/* A cyclic S,S,L,L sequence has four unique rotations.  With a LONG interval
 * equal to two SHORT units, its edge positions are 0, 120, 240, and 480 engine
 * degrees over a 720-degree cycle. */
static const trigger_interval_class_t trigger_example_cam_pattern[] =
{
  TRIGGER_INTERVAL_SHORT,
  TRIGGER_INTERVAL_SHORT,
  TRIGGER_INTERVAL_LONG,
  TRIGGER_INTERVAL_LONG,
};

/* ------------------------------ Wheel list ------------------------------- */

static const trigger_wheel_config_t trigger_wheels[] =
{
  {
    .name = "crank 60-2",
    .input_channel = 1U,                  /* TMG_OUT1 / PD10 */
    .active_edge = TRIGGER_EDGE_RISING,
    .type = TRIGGER_WHEEL_MISSING_TOOTH,
    .cycle_degrees = 360.0f,
    .angle_at_index0_deg = 0.0f,          /* first tooth after gap */
    .angle_direction = +1,
    .interval_tolerance_permille = 350U,  /* +/-35% */
    .speed_filter_permille = 250U,
    .sync_confirmations = 2U,             /* two correctly spaced gaps */
    .minimum_interval_us = 20U,
    .minimum_timeout_us = 100000U,        /* 100 ms minimum stop timeout */
    .timeout_interval_multiplier = 2.5f,
    .geometry.missing_tooth = {
      .tooth_positions = 60U,
      .missing_teeth = 2U,
    },
  },

  /* To enable a phase/cam wheel, add another initializer here and increase
   * trigger_default_config.wheel_count.  Example:
   *
   * {
   *   .name = "cam S-S-L-L",
   *   .input_channel = 2U,                // TMG_OUT2 / PD12
   *   .active_edge = TRIGGER_EDGE_RISING,
   *   .type = TRIGGER_WHEEL_INTERVAL_PATTERN,
   *   .cycle_degrees = 720.0f,
   *   .angle_at_index0_deg = 0.0f,        // MEASURE THIS ON THE ENGINE
   *   .angle_direction = +1,
   *   .interval_tolerance_permille = 250U,
   *   .speed_filter_permille = 250U,
   *   .sync_confirmations = 1U,
   *   .minimum_interval_us = 50U,
   *   .minimum_timeout_us = 300000U,
   *   .timeout_interval_multiplier = 2.5f,
   *   .geometry.pattern = {
   *     .intervals = trigger_example_cam_pattern,
   *     .interval_count = 4U,
   *     .long_interval_units = 2.0f,
   *   },
   * },
   */
};

static const trigger_decoder_config_t trigger_default_config =
{
  .wheels = trigger_wheels,
  .wheel_count = 1U,
  .primary_wheel = 0U,
  .phase_wheel = TRIGGER_NO_WHEEL,
  .engine_cycle_degrees = 720.0f,
  .phase_alignment_tolerance_deg = 30.0f,
};

#endif /* TRIGGER_DECODER_CONFIG_H */
