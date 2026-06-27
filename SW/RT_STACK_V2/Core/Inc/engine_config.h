/**
 ******************************************************************************
 * @file    engine_config.h
 * @brief   All engine calibration and hardware mapping in one place:
 *          cylinder count, firing order + TDC spacing, channel assignments,
 *          ignition/injection timing limits, thresholds, and the physical
 *          pin/peripheral maps for the IGN and injector channels.
 *
 * Edit this file to adapt the firmware to a given engine and board. The spark
 * and injection modules (spark.*, injection.*) read everything from here.
 ******************************************************************************
 */
#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

#include "main.h"   /* HAL types, GPIO/COMP/DAC handles' types, pin defines */

/* Peripheral handles owned by main.c (CubeMX), referenced by the maps below. */
extern COMP_HandleTypeDef hcomp1, hcomp2, hcomp3, hcomp4, hcomp5, hcomp6;
extern DAC_HandleTypeDef  hdac1, hdac3, hdac4;

/* ============================== Calibration ============================== */
#define CYLINDER_COUNT    4U       /* number of cylinders */
#define IGN_CHANNELS     12U       /* IGN output channels on the board */
#define INJ_CHANNELS     12U       /* injector channels on the board */

/* Ignition */
#define DWELL_MAX_MS      4.0f     /* coil dwell clamp (ms), protects the coil */

/* Injection */
#define BOOST_TIME_MS     1.0f     /* boost-phase duration (ms): BOOSTx high then low */
#define COMP_DAC_HIGH     2048U    /* current-comparator threshold during boost (raw 12-bit) */
#define COMP_DAC_LOW      1024U    /* current-comparator threshold during hold/rest */

/* Firing sequence. One entry per cylinder, in FIRING ORDER.
 *  - cyl     : physical cylinder id (used to address the cylinder in the API).
 *  - spacing : crank-cycle angle (deg) from the previous firing TDC to this
 *              cylinder's TDC; first entry is the offset from cycle 0. The
 *              cumulative sum gives each cylinder's TDC in the 0..720 cycle
 *              (use 720/CYLINDER_COUNT for even firing, per-entry for odd-fire).
 *  - ign_ch  : ignition channel 1..12 (index into ign_hw).
 *  - inj_ch  : injector channel 1..12 (index into inj_hw). */
typedef struct
{
  uint8_t cyl;
  float   spacing;
  uint8_t ign_ch;
  uint8_t inj_ch;
} firing_step_t;

/* Example: inline-4, firing order 1-3-4-2, even 180 deg spacing. */
static const firing_step_t firing_order[CYLINDER_COUNT] =
{
  { 1,   0.0f, 1, 1 },   /* cyl 1 -> TDC 0   */
  { 3, 180.0f, 3, 3 },   /* cyl 3 -> TDC 180 */
  { 4, 180.0f, 4, 4 },   /* cyl 4 -> TDC 360 */
  { 2, 180.0f, 2, 2 },   /* cyl 2 -> TDC 540 */
};

/* ========================== Hardware channel maps ======================= */
/* Ignition channel 1..12 -> output pin. Indexed as ign_hw[ign_ch - 1]. */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} ign_hw_t;

static const ign_hw_t ign_hw[IGN_CHANNELS] =
{
  { IGN1_GPIO_Port,  IGN1_Pin  }, { IGN2_GPIO_Port,  IGN2_Pin  },
  { IGN3_GPIO_Port,  IGN3_Pin  }, { IGN4_GPIO_Port,  IGN4_Pin  },
  { IGN5_GPIO_Port,  IGN5_Pin  }, { IGN6_GPIO_Port,  IGN6_Pin  },
  { IGN7_GPIO_Port,  IGN7_Pin  }, { IGN8_GPIO_Port,  IGN8_Pin  },
  { IGN9_GPIO_Port,  IGN9_Pin  }, { IGN10_GPIO_Port, IGN10_Pin },
  { IGN11_GPIO_Port, IGN11_Pin }, { IGN12_GPIO_Port, IGN12_Pin },
};

/* Injector channel 1..12 -> comparator / threshold DAC / SEL / BOOST.
 * Indexed as inj_hw[inj_ch - 1]. The two injectors of a pair (1&2, 3&4, ...)
 * share a comparator and SEL line; sel_state picks which one. */
typedef struct
{
  COMP_HandleTypeDef *comp;
  DAC_HandleTypeDef  *dac;
  uint32_t            dac_channel;
  GPIO_TypeDef       *sel_port;
  uint16_t            sel_pin;
  GPIO_PinState       sel_state;
  GPIO_TypeDef       *boost_port;
  uint16_t            boost_pin;
} inj_hw_t;

static const inj_hw_t inj_hw[INJ_CHANNELS] =
{
  { &hcomp1, &hdac1, DAC_CHANNEL_1, SEL12_GPIO_Port,   SEL12_Pin,   GPIO_PIN_SET,   BOOST12_GPIO_Port,   BOOST12_Pin   }, /* inj 1  */
  { &hcomp1, &hdac1, DAC_CHANNEL_1, SEL12_GPIO_Port,   SEL12_Pin,   GPIO_PIN_RESET, BOOST12_GPIO_Port,   BOOST12_Pin   }, /* inj 2  */
  { &hcomp2, &hdac1, DAC_CHANNEL_2, SEL34_GPIO_Port,   SEL34_Pin,   GPIO_PIN_SET,   BOOST34_GPIO_Port,   BOOST34_Pin   }, /* inj 3  */
  { &hcomp2, &hdac1, DAC_CHANNEL_2, SEL34_GPIO_Port,   SEL34_Pin,   GPIO_PIN_RESET, BOOST34_GPIO_Port,   BOOST34_Pin   }, /* inj 4  */
  { &hcomp3, &hdac3, DAC_CHANNEL_1, SEL56_GPIO_Port,   SEL56_Pin,   GPIO_PIN_SET,   BOOST56_GPIO_Port,   BOOST56_Pin   }, /* inj 5  */
  { &hcomp3, &hdac3, DAC_CHANNEL_1, SEL56_GPIO_Port,   SEL56_Pin,   GPIO_PIN_RESET, BOOST56_GPIO_Port,   BOOST56_Pin   }, /* inj 6  */
  { &hcomp4, &hdac3, DAC_CHANNEL_2, SEL78_GPIO_Port,   SEL78_Pin,   GPIO_PIN_SET,   BOOST78_GPIO_Port,   BOOST78_Pin   }, /* inj 7  */
  { &hcomp4, &hdac3, DAC_CHANNEL_2, SEL78_GPIO_Port,   SEL78_Pin,   GPIO_PIN_RESET, BOOST78_GPIO_Port,   BOOST78_Pin   }, /* inj 8  */
  { &hcomp5, &hdac4, DAC_CHANNEL_1, SEL910_GPIO_Port,  SEL910_Pin,  GPIO_PIN_SET,   BOOST910_GPIO_Port,  BOOST910_Pin  }, /* inj 9  */
  { &hcomp5, &hdac4, DAC_CHANNEL_1, SEL910_GPIO_Port,  SEL910_Pin,  GPIO_PIN_RESET, BOOST910_GPIO_Port,  BOOST910_Pin  }, /* inj 10 */
  { &hcomp6, &hdac4, DAC_CHANNEL_2, SEL1112_GPIO_Port, SEL1112_Pin, GPIO_PIN_SET,   BOOST1112_GPIO_Port, BOOST1112_Pin }, /* inj 11 */
  { &hcomp6, &hdac4, DAC_CHANNEL_2, SEL1112_GPIO_Port, SEL1112_Pin, GPIO_PIN_RESET, BOOST1112_GPIO_Port, BOOST1112_Pin }, /* inj 12 */
};

/* Firing-order index of a physical cylinder id (firing_order[].cyl); -1 if not
 * found, so cyl = 0 (or any unknown id) means "no cylinder". */
static inline int8_t cyl_index(uint8_t cyl)
{
  for (uint8_t i = 0U; i < CYLINDER_COUNT; i++)
    if (firing_order[i].cyl == cyl)
      return (int8_t)i;
  return -1;
}

#endif /* ENGINE_CONFIG_H */
