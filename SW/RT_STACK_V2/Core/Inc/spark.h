/**
 ******************************************************************************
 * @file    spark.h
 * @brief   Ignition (spark/dwell) control module.
 *
 * Angle-based spark, time-based dwell: the coil charges for 'dwell' ms ending
 * at the cylinder's spark angle (TDC - advance). The channel and TDC are taken
 * from the firing order in engine_config.h.
 ******************************************************************************
 */
#ifndef SPARK_H
#define SPARK_H

#include <stdint.h>

/** Initialize the module: disarm all cylinders, force every coil off.
 *  Call once before the main loop. */
void spark_init(void);

/**
 * @brief Update one cylinder's spark inputs and run the dispatch.
 *
 * Call once per main-loop iteration. cyl = physical cylinder id updates (and
 * arms) that cylinder's advance/dwell; cyl = 0 only re-runs the dispatch with
 * the stored inputs.
 *
 * @param cyl      physical cylinder id; 0 = none (just re-run)
 * @param advance  ignition advance before TDC, in degrees
 * @param dwell    coil charge time, in milliseconds (clamped to DWELL_MAX_MS)
 */
void spark_update(uint8_t cyl, float advance, float dwell);

/**
 * @brief Shut off cylinders by bitmask (disarm + force coil off).
 *        Bit (cyl-1) set -> cylinder 'cyl' off, e.g. 0b1011 -> cyl 1,2,4.
 * @param mask cylinders to shut off; -1 shuts off all.
 */
void spark_off(int16_t mask);

#endif /* SPARK_H */
