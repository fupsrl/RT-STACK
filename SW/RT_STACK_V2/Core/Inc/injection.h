/**
 ******************************************************************************
 * @file    injection.h
 * @brief   Fuel injection (current-controlled) module.
 *
 * Angle-based start, time-based durations: when the angle crosses the start
 * angle (cylinder TDC - startBtdc) the injector charges (boost) then holds,
 * with the comparator threshold switching high->low at boost end. The channel
 * and TDC are taken from the firing order in engine_config.h.
 ******************************************************************************
 */
#ifndef INJECTION_H
#define INJECTION_H

#include <stdint.h>

/** Initialize the module: start the threshold DACs, disarm all cylinders,
 *  force every injector off. Call once before the main loop. */
void injection_init(void);

/**
 * @brief Update one cylinder's injection inputs and run the dispatch.
 *
 * Call once per main-loop iteration. cyl = physical cylinder id updates (and
 * arms) that cylinder; cyl = 0 only re-runs the dispatch with the stored inputs.
 *
 * @param cyl       physical cylinder id; 0 = none (just re-run)
 * @param startBtdc injection start, in degrees before TDC
 * @param duration  injection duration, in milliseconds
 */
void injection_update(uint8_t cyl, float startBtdc, float duration);

/**
 * @brief Shut off cylinders by bitmask (disarm + force injector off).
 *        Bit (cyl-1) set -> cylinder 'cyl' off, e.g. 0b1011 -> cyl 1,2,4.
 * @param mask cylinders to shut off; -1 shuts off all.
 */
void injection_off(int16_t mask);

#endif /* INJECTION_H */
