#include "actuator_test_hal.h"
#include "injection.h"
#include "injection_internal.h"
#include "spark.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t actuator_test_primask;
GPIO_TypeDef actuator_test_gpio[30];
static COMP_TypeDef comparator_instance[6];
static DAC_TypeDef dac_instance[3];
COMP_HandleTypeDef hcomp1 = { &comparator_instance[0], HAL_COMP_STATE_READY };
COMP_HandleTypeDef hcomp2 = { &comparator_instance[1], HAL_COMP_STATE_READY };
COMP_HandleTypeDef hcomp3 = { &comparator_instance[2], HAL_COMP_STATE_READY };
COMP_HandleTypeDef hcomp4 = { &comparator_instance[3], HAL_COMP_STATE_READY };
COMP_HandleTypeDef hcomp5 = { &comparator_instance[4], HAL_COMP_STATE_READY };
COMP_HandleTypeDef hcomp6 = { &comparator_instance[5], HAL_COMP_STATE_READY };
DAC_HandleTypeDef hdac1 = { .Instance = &dac_instance[0] };
DAC_HandleTypeDef hdac3 = { .Instance = &dac_instance[1] };
DAC_HandleTypeDef hdac4 = { .Instance = &dac_instance[2] };

static bool emergency_on_ignition_high;
static bool emergency_on_ignition_low;
static bool emergency_during_comp_start;
static bool emergency_during_comp_stop;
static bool emergency_during_hold_dac;
static bool hold_dac_was_irq_serialized;
static uint32_t comparator_start_count[6];

static uint8_t comparator_index(const COMP_HandleTypeDef *handle)
{
  uint8_t i;

  for (i = 0U; i < 6U; ++i)
  {
    if (handle->Instance == &comparator_instance[i])
    {
      return i;
    }
  }
  return 6U;
}

static void reset_actuator_test_hal(void)
{
  COMP_HandleTypeDef *comparators[6] = {
    &hcomp1, &hcomp2, &hcomp3, &hcomp4, &hcomp5, &hcomp6
  };
  uint8_t i;

  memset(actuator_test_gpio, 0, sizeof(actuator_test_gpio));
  memset(comparator_start_count, 0, sizeof(comparator_start_count));
  memset(dac_instance, 0, sizeof(dac_instance));
  memset(hdac1.value, 0, sizeof(hdac1.value));
  memset(hdac1.started, 0, sizeof(hdac1.started));
  memset(hdac3.value, 0, sizeof(hdac3.value));
  memset(hdac3.started, 0, sizeof(hdac3.started));
  memset(hdac4.value, 0, sizeof(hdac4.value));
  memset(hdac4.started, 0, sizeof(hdac4.started));
  actuator_test_primask = 0U;
  emergency_during_comp_start = false;
  emergency_during_comp_stop = false;
  emergency_during_hold_dac = false;

  for (i = 0U; i < 6U; ++i)
  {
    comparators[i]->Instance->CSR = 0U;
    comparators[i]->State = HAL_COMP_STATE_READY;
  }
}

void actuator_test_gpio_write_hook(GPIO_TypeDef *port, uint16_t pin, bool high)
{
  if ((port == IGN1_GPIO_Port) && (pin == IGN1_Pin) && high &&
      emergency_on_ignition_high)
  {
    emergency_on_ignition_high = false;
    spark_emergency_off();
  }
  if ((port == IGN1_GPIO_Port) && (pin == IGN1_Pin) && !high &&
      emergency_on_ignition_low)
  {
    emergency_on_ignition_low = false;
    spark_emergency_off();
  }
}

HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *handle)
{
  uint8_t index;

  if ((handle == NULL) || (handle->State != HAL_COMP_STATE_READY))
    return HAL_ERROR;
  index = comparator_index(handle);
  if (index < 6U)
  {
    ++comparator_start_count[index];
  }
  handle->Instance->CSR |= COMP_CSR_EN;
  handle->State = HAL_COMP_STATE_BUSY;
  if (emergency_during_comp_start)
  {
    emergency_during_comp_start = false;
    injection_emergency_off();
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_COMP_Stop(COMP_HandleTypeDef *handle)
{
  if ((handle == NULL) || (handle->State == HAL_COMP_STATE_RESET))
    return HAL_ERROR;
  handle->Instance->CSR &= ~COMP_CSR_EN;
  handle->State = HAL_COMP_STATE_READY;
  if (emergency_during_comp_stop)
  {
    emergency_during_comp_stop = false;
    injection_emergency_off();
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *handle, uint32_t channel,
                                   uint32_t alignment, uint32_t value)
{
  (void)alignment;
  if ((handle == NULL) || (channel > 1U) || (value > 4095U))
    return HAL_ERROR;
  handle->value[channel] = value;
  if ((value == 1024U) && emergency_during_hold_dac)
  {
    emergency_during_hold_dac = false;
    hold_dac_was_irq_serialized = (actuator_test_primask != 0U);
    injection_emergency_off();
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *handle, uint32_t channel)
{
  if ((handle == NULL) || (channel > 1U))
    return HAL_ERROR;
  handle->started[channel] = 1U;
  return HAL_OK;
}

static actuator_runtime_snapshot_t snapshot(float angle, uint32_t now)
{
  actuator_runtime_snapshot_t s = {
    .angle_deg = angle, .rpm = 1000.0f, .now_tick = now,
    .cycle_deg = ENGINE_CYCLE_DEG,
    .tick_hz = 1000000U, .sync_epoch = 1U,
    .crank_synced = true, .phase_synced = true, .outputs_enabled = true
  };
  return s;
}

static void test_runtime_helpers(void)
{
  uint32_t deadline;
  assert(actuator_deadline_from_us(0xFFFFFF00U, 1000U, 1000000U, &deadline));
  assert(deadline == 0x000002E8U);
  assert(!actuator_deadline_reached(0x000002E7U, deadline));
  assert(actuator_deadline_reached(0x000002E8U, deadline));
  assert(actuator_angle_crossed(2.0f, 719.0f, 3.0f));
  assert(!actuator_angle_crossed(719.0f, 719.0f, 3.0f));
  assert(actuator_forward_angle(359.0f, 1.0f,
                                ENGINE_CRANK_CYCLE_DEG) == 2.0f);
  assert(actuator_wrap_angle(540.0f, ENGINE_CRANK_CYCLE_DEG) == 180.0f);
}

static void test_live_phase_requirement(void)
{
  spark_init();
  injection_init();
  assert(!spark_phase_sync_required());
  assert(!injection_phase_sync_required());

  assert(spark_load_defaults(1U, true) == SPARK_RESULT_OK);
#if (IGNITION_REQUIRES_PHASE_SYNC != 0U)
  assert(spark_phase_sync_required());
#else
  assert(!spark_phase_sync_required());
#endif
  spark_disable_cylinder(1U);
  assert(!spark_phase_sync_required());

  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
#if (INJECTION_REQUIRES_PHASE_SYNC != 0U)
  assert(injection_phase_sync_required());
#else
  assert(!injection_phase_sync_required());
#endif
  injection_disable_cylinder(1U);
  assert(!injection_phase_sync_required());
}

#if (IGNITION_REQUIRES_PHASE_SYNC == 0U) || \
    (INJECTION_REQUIRES_PHASE_SYNC == 0U)
static actuator_runtime_snapshot_t crank_snapshot(float angle, uint32_t now)
{
  actuator_runtime_snapshot_t s = snapshot(angle, now);
  s.cycle_deg = ENGINE_CRANK_CYCLE_DEG;
  s.phase_synced = false;
  return s;
}
#endif

static void test_spark_latched_event_and_deadline(void)
{
  actuator_runtime_snapshot_t s;
  spark_command_t changed = { true, 30.0f, 4000U };

  spark_init();
  assert(spark_load_defaults(1U, true) == SPARK_RESULT_OK);
  s = snapshot(680.0f, 1000U); spark_service(&s); /* runtime latch */
  s = snapshot(681.0f, 1100U); spark_service(&s); /* command latch */
  actuator_test_gpio[0].BSRR = actuator_test_gpio[0].BRR = 0U;
  s = snapshot(688.0f, 1200U); spark_service(&s); /* dwell begins */
  assert(actuator_test_gpio[0].BSRR == IGN1_Pin);

  /* Updating advance while charging must not move the active 705 deg spark. */
  assert(spark_set_command(1U, &changed) == SPARK_RESULT_OK);
  s = snapshot(706.0f, 1400U); spark_service(&s);
  assert(actuator_test_gpio[0].BRR == IGN1_Pin);

  /* Start another default event and prove the absolute tick cutoff. */
  spark_init();
  assert(spark_load_defaults(1U, true) == SPARK_RESULT_OK);
  s = snapshot(680.0f, 1000U); spark_service(&s);
  s = snapshot(681.0f, 1100U); spark_service(&s);
  s = snapshot(688.0f, 1200U); spark_service(&s);
  actuator_test_gpio[0].BRR = 0U;
  spark_service_deadlines(4449U);
  assert(actuator_test_gpio[0].BRR == 0U);
  spark_service_deadlines(4450U); /* 3000 us + 250 us guard */
  assert(actuator_test_gpio[0].BRR == IGN1_Pin);
  assert((spark_faults() & SPARK_FAULT_HARD_DEADLINE) != 0U);
}

static void test_injection_immutable_duration_and_gating(void)
{
  actuator_runtime_snapshot_t s;
  injection_command_t changed = { true, 360.0f, 12000U };

  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  assert(actuator_test_gpio[18].BSRR == BOOST12_Pin);

  assert(injection_set_command(1U, &changed) == INJECTION_RESULT_OK);
  injection_service_deadlines(2199U);
  assert(actuator_test_gpio[18].BRR == BOOST12_Pin); /* init wrote low */
  actuator_test_gpio[18].BRR = 0U;
  injection_service_deadlines(2200U);
  assert(actuator_test_gpio[18].BRR == BOOST12_Pin);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);

  injection_service_deadlines(9199U);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  injection_service_deadlines(9200U); /* original 8000 us, not new 12000 us */
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);

  /* Loss of phase immediately removes output permission. */
  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
#if (INJECTION_REQUIRES_PHASE_SYNC != 0U)
  s.phase_synced = false;
#else
  s.outputs_enabled = false;
#endif
  injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
}

static void test_validation_and_emergency_latch(void)
{
  spark_command_t bad_spark = { true, NAN, 3000U };
  injection_command_t bad_injection = { true, 360.0f, 50000U };

  spark_init();
  injection_init();
  assert(spark_set_command(1U, &bad_spark) == SPARK_RESULT_NONFINITE_COMMAND);
  assert(injection_set_command(1U, &bad_injection) ==
         INJECTION_RESULT_COMMAND_OUT_OF_RANGE);
  spark_emergency_off();
  injection_emergency_off();
  assert(spark_load_defaults(1U, true) == SPARK_RESULT_EMERGENCY_LATCHED);
  assert(injection_load_defaults(1U, true) ==
         INJECTION_RESULT_EMERGENCY_LATCHED);
  assert(spark_rearm_after_emergency());
  assert(injection_rearm_after_emergency());
}

static void test_nmi_race_guards(void)
{
  actuator_runtime_snapshot_t s;
  spark_command_t spark_command = { true, 15.0f, 3000U };
  injection_command_t injection_command = { true, 360.0f, 8000U };

  /* NMI immediately after the coil-on write must be observed by begin_dwell. */
  spark_init();
  assert(spark_load_defaults(1U, true) == SPARK_RESULT_OK);
  s = snapshot(680.0f, 1000U); spark_service(&s);
  s = snapshot(681.0f, 1100U); spark_service(&s);
  emergency_on_ignition_high = true;
  actuator_test_gpio[0].BRR = 0U;
  s = snapshot(688.0f, 1200U); spark_service(&s);
  assert(actuator_test_gpio[0].BRR == IGN1_Pin);
  assert(spark_set_command(1U, &spark_command) ==
         SPARK_RESULT_EMERGENCY_LATCHED);

  /* A newer NMI generation during rearm must make rearm fail. */
  emergency_on_ignition_low = true;
  assert(!spark_rearm_after_emergency());
  assert(spark_set_command(1U, &spark_command) ==
         SPARK_RESULT_EMERGENCY_LATCHED);
  assert(spark_rearm_after_emergency());

  /* NMI from inside HAL_COMP_Start cannot leave the comparator re-enabled. */
  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  emergency_during_comp_start = true;
  s = snapshot(361.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert(injection_set_command(1U, &injection_command) ==
         INJECTION_RESULT_EMERGENCY_LATCHED);

  emergency_during_comp_stop = true;
  assert(!injection_rearm_after_emergency());
  assert(injection_set_command(1U, &injection_command) ==
         INJECTION_RESULT_EMERGENCY_LATCHED);
  assert(injection_rearm_after_emergency());
}

static void test_pair_collision_is_fail_closed(void)
{
  actuator_runtime_snapshot_t s;
  injection_command_t cylinder_2 = { true, 180.0f, 8000U };

  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  assert(injection_set_command(2U, &cylinder_2) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);

  /* Cylinders 1 and 2 both requested pair 1 at 360 degrees. */
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert((injection_faults() & INJECTION_FAULT_PAIR_COLLISION) != 0U);
}

static void test_comparator_shutdown_paths(void)
{
  actuator_runtime_snapshot_t s;
  COMP_TypeDef *saved_instance;

  /* Emergency may be reached before CubeMX assigned every Instance. */
  saved_instance = hcomp1.Instance;
  hcomp1.Instance = NULL;
  injection_emergency_off();
  hcomp1.Instance = saved_instance;

  /* Even a broken HAL state cannot defeat the direct normal-path disable. */
  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  hcomp1.State = HAL_COMP_STATE_RESET;
  s.outputs_enabled = false;
  injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  hcomp1.State = HAL_COMP_STATE_READY;
}

static void test_deadline_transition_cannot_resurrect_pair(void)
{
  actuator_runtime_snapshot_t s;
  injection_command_t command = { true, 360.0f, 8000U };

  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);
  emergency_during_hold_dac = true;
  hold_dac_was_irq_serialized = false;
  injection_service_deadlines(2200U);
  assert(hold_dac_was_irq_serialized);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert(injection_set_command(1U, &command) ==
         INJECTION_RESULT_EMERGENCY_LATCHED);
  assert(injection_rearm_after_emergency());
}

static void test_injector_debug_validation(void)
{
  injection_debug_state_t state;

  reset_actuator_test_hal();
  injection_init();

  assert(injection_debug_start(0U, 10000U, 1000U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_OUTPUT);
  assert(injection_debug_start(13U, 10000U, 1000U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_OUTPUT);
  assert(injection_debug_start(5U, 10000U, 1000U, 0U, 1000000U) ==
         INJECTION_DEBUG_UNMAPPED_OUTPUT);
  assert(injection_debug_start(1U, 10000U, 99U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_TIMING);
  assert(injection_debug_start(1U, 40000U, 30001U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_TIMING);
  assert(injection_debug_start(1U, 2500U, 1000U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_TIMING);
  assert(injection_debug_start(1U, 4000U, 1001U, 0U, 1000000U) ==
         INJECTION_DEBUG_INVALID_TIMING);
  assert(injection_debug_start(1U, 10000U, 1000U, 0U, 0U) ==
         INJECTION_DEBUG_INVALID_TIMING);

  injection_debug_get_state(&state);
  assert(!state.active);
  assert(state.pulse_count == 0U);
  assert(comparator_start_count[0] == 0U);
}

static void test_injector_debug_periodic_timing_and_stop(void)
{
  injection_debug_state_t state;

  reset_actuator_test_hal();
  injection_init();
  assert(injection_debug_start(2U, 10000U, 1000U, 1000U, 1000000U) ==
         INJECTION_DEBUG_OK);
  assert(injection_debug_active());
  injection_debug_get_state(&state);
  assert(state.active);
  assert(state.injector_output == 2U);
  assert(state.period_us == 10000U);
  assert(state.pulse_width_us == 1000U);
  assert(state.pulse_count == 0U);

  /* Arming never actuates immediately: the first pulse is one full T later. */
  assert(injection_debug_service(10999U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 0U);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  actuator_test_gpio[12].BSRR = actuator_test_gpio[12].BRR = 0U;
  assert(injection_debug_service(11000U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 1U);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  assert(actuator_test_gpio[12].BRR == SEL12_Pin); /* Output 2 selected. */
  assert(actuator_test_gpio[18].BSRR == BOOST12_Pin);

  injection_debug_get_state(&state);
  assert(state.pulse_count == 1U);
  assert(state.skipped_period_count == 0U);
  assert(injection_debug_service(11999U) == INJECTION_DEBUG_OK);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  assert(injection_debug_service(12000U) == INJECTION_DEBUG_OK);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);

  assert(injection_debug_service(20999U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 1U);
  assert(injection_debug_service(21000U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 2U);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);

  /* Stop is synchronous and idempotent, including during an active pulse. */
  injection_debug_stop();
  injection_debug_stop();
  assert(!injection_debug_active());
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert(injection_debug_service(31000U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 2U);

  /* Stopping between pulses must also cancel the pending first pulse. */
  assert(injection_debug_start(1U, 10000U, 1000U, 50000U, 1000000U) ==
         INJECTION_DEBUG_OK);
  injection_debug_stop();
  assert(injection_debug_service(60000U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 2U);
}

static void test_injector_debug_late_service_skips_without_burst(void)
{
  injection_debug_state_t state;

  reset_actuator_test_hal();
  injection_init();
  assert(injection_debug_start(1U, 10000U, 1000U, 500U, 1000000U) ==
         INJECTION_DEBUG_OK);

  /* Service is two complete periods late. Exactly one pulse may start. */
  assert(injection_debug_service(30500U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 1U);
  injection_debug_get_state(&state);
  assert(state.pulse_count == 1U);
  assert(state.skipped_period_count == 2U);

  assert(injection_debug_service(31500U) == INJECTION_DEBUG_OK);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert(injection_debug_service(40499U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 1U);
  assert(injection_debug_service(40500U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 2U);
  injection_debug_stop();
}

static void test_injector_debug_tick_wrap(void)
{
  reset_actuator_test_hal();
  injection_init();
  assert(injection_debug_start(1U, 5000U, 1000U, 0xFFFFFF00U,
                               1000000U) == INJECTION_DEBUG_OK);
  assert(injection_debug_service(0x00001287U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 0U);
  assert(injection_debug_service(0x00001288U) == INJECTION_DEBUG_OK);
  assert(comparator_start_count[0] == 1U);
  injection_debug_stop();
}

static void test_injector_debug_pair_busy_and_emergency_cancel(void)
{
  actuator_runtime_snapshot_t s;

  /* The debug path cannot take a shared driver away from a normal event. */
  reset_actuator_test_hal();
  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = snapshot(350.0f, 1000U); injection_service(&s);
  s = snapshot(351.0f, 1100U); injection_service(&s);
  s = snapshot(361.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  assert(injection_debug_start(2U, 10000U, 1000U, 1300U, 1000000U) ==
         INJECTION_DEBUG_PAIR_BUSY);
  assert(!injection_debug_active());
  injection_emergency_off();
  assert(injection_rearm_after_emergency());

  /* Both generic emergency-off and the trigger-ISR interlock cancel tests. */
  assert(injection_debug_start(1U, 10000U, 1000U, 20000U, 1000000U) ==
         INJECTION_DEBUG_OK);
  assert(injection_debug_service(30000U) == INJECTION_DEBUG_OK);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  injection_emergency_off();
  assert(!injection_debug_active());
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert(injection_debug_start(1U, 10000U, 1000U, 40000U, 1000000U) ==
         INJECTION_DEBUG_EMERGENCY_LATCHED);
  assert(injection_rearm_after_emergency());

  assert(injection_debug_start(1U, 10000U, 1000U, 50000U, 1000000U) ==
         INJECTION_DEBUG_OK);
  injection_debug_abort_isr();
  assert(!injection_debug_active());
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) == 0U);
  assert((injection_faults() & INJECTION_FAULT_DEBUG_INTERLOCK) != 0U);
  assert(injection_debug_start(1U, 10000U, 1000U, 60000U, 1000000U) ==
         INJECTION_DEBUG_EMERGENCY_LATCHED);
  assert(injection_rearm_after_emergency());
}

#if (IGNITION_REQUIRES_PHASE_SYNC == 0U)
static void test_crank_only_wasted_spark(void)
{
  actuator_runtime_snapshot_t s;

  /* Cylinder 2 has a 540 degree calibrated TDC.  In the crank domain it must
   * schedule at 180 degrees, so its default 15 degree spark is at 165. */
  spark_init();
  assert(spark_load_defaults(2U, true) == SPARK_RESULT_OK);
  s = crank_snapshot(140.0f, 1000U); spark_service(&s);
  s = crank_snapshot(141.0f, 1100U); spark_service(&s);
  actuator_test_gpio[1].BSRR = actuator_test_gpio[1].BRR = 0U;
  s = crank_snapshot(148.0f, 1200U); spark_service(&s);
  assert(actuator_test_gpio[1].BSRR == IGN2_Pin);
  s = crank_snapshot(166.0f, 1300U); spark_service(&s);
  assert(actuator_test_gpio[1].BRR == IGN2_Pin);

  /* 359 -> 0 is a two-degree forward move, not a 362/722 degree jump. */
  spark_init();
  assert(spark_load_defaults(1U, true) == SPARK_RESULT_OK);
  s = crank_snapshot(350.0f, 2000U); spark_service(&s);
  s = crank_snapshot(351.0f, 2100U); spark_service(&s);
  s = crank_snapshot(359.0f, 2200U); spark_service(&s);
  s = crank_snapshot(1.0f, 2300U); spark_service(&s);
  assert((spark_faults() & SPARK_FAULT_ANGLE_JUMP) == 0U);
}
#endif

#if (INJECTION_REQUIRES_PHASE_SYNC == 0U)
static void test_crank_only_batch_injection(void)
{
  actuator_runtime_snapshot_t s;

  /* Cylinder 2: 540 TDC - 360 BTDC = 180 in the crank domain. */
  injection_init();
  assert(injection_load_defaults(2U, true) == INJECTION_RESULT_OK);
  s = crank_snapshot(170.0f, 1000U); injection_service(&s);
  s = crank_snapshot(171.0f, 1100U); injection_service(&s);
  s = crank_snapshot(181.0f, 1200U); injection_service(&s);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  injection_emergency_off();

  /* Cylinder 1's batch target is exactly zero; prove crossing at crank wrap. */
  injection_init();
  assert(injection_load_defaults(1U, true) == INJECTION_RESULT_OK);
  s = crank_snapshot(350.0f, 2000U); injection_service(&s);
  s = crank_snapshot(351.0f, 2100U); injection_service(&s);
  s = crank_snapshot(359.0f, 2200U); injection_service(&s);
  s = crank_snapshot(1.0f, 2300U); injection_service(&s);
  assert((injection_faults() & INJECTION_FAULT_ANGLE_JUMP) == 0U);
  assert((hcomp1.Instance->CSR & COMP_CSR_EN) != 0U);
  injection_emergency_off();
}
#endif

int main(void)
{
  memset(actuator_test_gpio, 0, sizeof(actuator_test_gpio));
  test_runtime_helpers();
  test_live_phase_requirement();
  test_spark_latched_event_and_deadline();
  test_injection_immutable_duration_and_gating();
  test_validation_and_emergency_latch();
  test_nmi_race_guards();
  test_pair_collision_is_fail_closed();
  test_comparator_shutdown_paths();
  test_deadline_transition_cannot_resurrect_pair();
  test_injector_debug_validation();
  test_injector_debug_periodic_timing_and_stop();
  test_injector_debug_late_service_skips_without_burst();
  test_injector_debug_tick_wrap();
  test_injector_debug_pair_busy_and_emergency_cancel();
#if (IGNITION_REQUIRES_PHASE_SYNC == 0U)
  test_crank_only_wasted_spark();
#endif
#if (INJECTION_REQUIRES_PHASE_SYNC == 0U)
  test_crank_only_batch_injection();
#endif
  puts("actuator tests: PASS");
  return 0;
}
