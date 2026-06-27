/**
 ******************************************************************************
 * @file    injection.c
 * @brief   Fuel injection (current-controlled) module. See injection.h.
 ******************************************************************************
 */
#include "injection.h"
#include "engine_config.h"
#include "engine_runtime.h"

typedef enum { INJ_OFF = 0, INJ_BOOST, INJ_HOLD } inj_phase_t;

/* Per-cylinder inputs, indexed by firing-order position. */
static struct
{
  float start_btdc;   /* injection start before TDC (deg) */
  float duration;     /* injection duration (ms) */
  bool  armed;
} icyl[CYLINDER_COUNT];

/* Per-injector-channel state (1..12). State is per channel because the two
 * injectors of a pair share a comparator and SEL line (twin guard). */
static struct
{
  inj_phase_t phase;
  uint32_t    start_cyc;   /* injection start timestamp (cycles) */
  float       prev_angle;  /* engine angle on the previous dispatch */
} ich[INJ_CHANNELS];

/* Force an injector channel (idx 0..11) to its off state. */
static void inj_off_channel(uint8_t idx)
{
  const inj_hw_t *hw = &inj_hw[idx];
  HAL_COMP_Stop(hw->comp);
  HAL_GPIO_WritePin(hw->boost_port, hw->boost_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(hw->sel_port, hw->sel_pin, GPIO_PIN_RESET);
  HAL_DAC_SetValue(hw->dac, hw->dac_channel, DAC_ALIGN_12B_R, COMP_DAC_LOW);
  ich[idx].phase = INJ_OFF;
}

/* Per-channel injection state machine (ch = 1..12). */
static void inject_channel(uint8_t ch, float fromAngle, float duration, float ang)
{
  uint8_t idx = ch - 1U;
  const inj_hw_t *hw = &inj_hw[idx];

  float prev = ich[idx].prev_angle;
  ich[idx].prev_angle = ang;
  if (prev < 0.0f)                              /* first dispatch: only latch */
    return;

  if (ich[idx].phase == INJ_OFF)
  {
    if (!angle_crossed(fromAngle, prev, ang))
      return;
    if (ich[idx ^ 1U].phase != INJ_OFF)         /* twin shares COMP/SEL */
      return;
    HAL_DAC_SetValue(hw->dac, hw->dac_channel, DAC_ALIGN_12B_R, COMP_DAC_HIGH);
    HAL_GPIO_WritePin(hw->sel_port, hw->sel_pin, hw->sel_state);
    HAL_GPIO_WritePin(hw->boost_port, hw->boost_pin, GPIO_PIN_SET);
    HAL_COMP_Start(hw->comp);
    ich[idx].start_cyc = cycle_now();
    ich[idx].phase = INJ_BOOST;
    return;
  }

  float elapsed_ms = (float)(cycle_now() - ich[idx].start_cyc)
                     / ((float)SystemCoreClock / 1000.0f);

  if (elapsed_ms >= duration)
  {
    inj_off_channel(idx);
  }
  else if ((ich[idx].phase == INJ_BOOST) && (elapsed_ms >= BOOST_TIME_MS))
  {
    HAL_GPIO_WritePin(hw->boost_port, hw->boost_pin, GPIO_PIN_RESET);
    HAL_DAC_SetValue(hw->dac, hw->dac_channel, DAC_ALIGN_12B_R, COMP_DAC_LOW);
    ich[idx].phase = INJ_HOLD;
  }
}

/* Run the injection state machine for every armed cylinder. */
static void injection_run(void)
{
  float ang = engine_angle;
  float tdc = 0.0f;
  for (uint8_t i = 0U; i < CYLINDER_COUNT; i++)
  {
    tdc += firing_order[i].spacing;            /* cumulative TDC in 0..720 */
    if (icyl[i].armed)
      inject_channel(firing_order[i].inj_ch, wrap720(tdc - icyl[i].start_btdc),
                     icyl[i].duration, ang);
  }
}

void injection_update(uint8_t cyl, float startBtdc, float duration)
{
  int8_t i = cyl_index(cyl);
  if (i >= 0)
  {
    icyl[i].start_btdc = startBtdc;
    icyl[i].duration   = duration;
    icyl[i].armed      = true;
  }
  injection_run();
}

void injection_off(int16_t mask)
{
  for (uint8_t cyl = 1U; cyl <= CYLINDER_COUNT; cyl++)
  {
    if ((((uint16_t)mask >> (cyl - 1U)) & 1U) == 0U)
      continue;

    int8_t i = cyl_index(cyl);
    if (i < 0)
      continue;
    icyl[i].armed = false;
    inj_off_channel((uint8_t)(firing_order[i].inj_ch - 1U));
  }
}

void injection_init(void)
{
  for (uint8_t i = 0U; i < CYLINDER_COUNT; i++)
  {
    icyl[i].start_btdc = 0.0f;
    icyl[i].duration   = 0.0f;
    icyl[i].armed      = false;
  }
  for (uint8_t idx = 0U; idx < INJ_CHANNELS; idx++)
  {
    ich[idx].phase      = INJ_OFF;
    ich[idx].start_cyc  = 0U;
    ich[idx].prev_angle = -1.0f;
  }

  /* Start the threshold DACs at the low value (a pair shares a DAC channel,
   * so one start per pair suffices), then force every injector off. */
  for (uint8_t idx = 0U; idx < INJ_CHANNELS; idx += 2U)
  {
    HAL_DAC_SetValue(inj_hw[idx].dac, inj_hw[idx].dac_channel, DAC_ALIGN_12B_R, COMP_DAC_LOW);
    HAL_DAC_Start(inj_hw[idx].dac, inj_hw[idx].dac_channel);
  }
  for (uint8_t idx = 0U; idx < INJ_CHANNELS; idx++)
    inj_off_channel(idx);
}
