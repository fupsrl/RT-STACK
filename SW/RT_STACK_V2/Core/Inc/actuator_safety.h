/** @file actuator_safety.h  Common emergency shutdown convenience API. */
#ifndef ACTUATOR_SAFETY_H
#define ACTUATOR_SAFETY_H

#include "spark.h"
#include "injection.h"
#include "dcdc_control.h"

static inline void actuators_emergency_off(void)
{
  spark_emergency_off();
  injection_emergency_off();
  dcdc_control_emergency_off();
}

#endif /* ACTUATOR_SAFETY_H */
