/**
 ******************************************************************************
 * @file    spark.c
 * @brief   Ignition (spark/dwell) control module. See spark.h.
 ******************************************************************************
 */
#include "spark.h"
#include "engine_config.h"
#include "engine_runtime.h"

typedef enum { SPARK_IDLE = 0, SPARK_CHARGING } spark_phase_t;

/* Per-cylinder state, indexed by firing-order position (0..CYLINDER_COUNT-1). */
static struct
{
  float         advance;     /* spark advance before TDC (deg) */
  float         dwell;       /* coil dwell (ms) */
  bool          armed;       /* cylinder participates in the dispatch */
  spark_phase_t phase;       /* coil charging state */
  float         prev_angle;  /* engine angle on the previous dispatch */
} scyl[CYLINDER_COUNT];

/* Drive the coil output of cylinder i (by firing position). */
static void coil_write(uint8_t i, GPIO_PinState state)
{
  uint8_t ch = firing_order[i].ign_ch;
  HAL_GPIO_WritePin(ign_hw[ch - 1U].port, ign_hw[ch - 1U].pin, state);
}

/* Run the spark state machine for every armed cylinder. */
static void spark_run(void)
{
  float ang = engine_angle;
  float omega = engine_rpm * 0.006f;          /* engine-deg per ms */
  float tdc = 0.0f;

  for (uint8_t i = 0U; i < CYLINDER_COUNT; i++)
  {
    tdc += firing_order[i].spacing;           /* cumulative TDC in 0..720 */
    if (!scyl[i].armed)
      continue;

    float spark_angle = wrap720(tdc - scyl[i].advance);
    float prev = scyl[i].prev_angle;
    scyl[i].prev_angle = ang;
    if (prev < 0.0f)                          /* first dispatch: only latch */
      continue;

    if (scyl[i].phase == SPARK_IDLE)
    {
      /* Start charging when the predicted time to the spark reaches the dwell. */
      if (omega <= 0.0f)
        continue;
      float remaining = wrap720(spark_angle - ang);
      if ((remaining / omega) <= scyl[i].dwell)
      {
        coil_write(i, GPIO_PIN_SET);          /* coil on (dwell starts) */
        scyl[i].phase = SPARK_CHARGING;
      }
    }
    else /* SPARK_CHARGING */
    {
      if (angle_crossed(spark_angle, prev, ang))
      {
        coil_write(i, GPIO_PIN_RESET);        /* spark! (falling edge) */
        scyl[i].phase = SPARK_IDLE;
      }
    }
  }
}

void spark_update(uint8_t cyl, float advance, float dwell)
{
  if (dwell > DWELL_MAX_MS)                    /* clamp dwell to protect the coil */
    dwell = DWELL_MAX_MS;

  int8_t i = cyl_index(cyl);
  if (i >= 0)
  {
    scyl[i].advance = advance;
    scyl[i].dwell   = dwell;
    scyl[i].armed   = true;
  }
  spark_run();
}

void spark_off(int16_t mask)
{
  for (uint8_t cyl = 1U; cyl <= CYLINDER_COUNT; cyl++)
  {
    if ((((uint16_t)mask >> (cyl - 1U)) & 1U) == 0U)
      continue;

    int8_t i = cyl_index(cyl);
    if (i < 0)
      continue;
    scyl[i].armed = false;
    scyl[i].phase = SPARK_IDLE;
    coil_write(i, GPIO_PIN_RESET);
  }
}

void spark_init(void)
{
  for (uint8_t i = 0U; i < CYLINDER_COUNT; i++)
  {
    scyl[i].advance    = 0.0f;
    scyl[i].dwell      = 0.0f;
    scyl[i].armed      = false;
    scyl[i].phase      = SPARK_IDLE;
    scyl[i].prev_angle = -1.0f;
  }
  /* Force every coil off (covers channels not in the firing order too). */
  for (uint8_t ch = 0U; ch < IGN_CHANNELS; ch++)
    HAL_GPIO_WritePin(ign_hw[ch].port, ign_hw[ch].pin, GPIO_PIN_RESET);
}
