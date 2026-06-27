/**
 ******************************************************************************
 * @file    trigger_decoder_config.h
 * @brief   Trigger pattern configuration (crank + up to 4 cams).
 *
 * Edit ONLY this file to adapt the decoder to your trigger wheel and phase
 * sensors. The algorithm lives in trigger_decoder.c; this header is included
 * by that file alone.
 ******************************************************************************
 */
#ifndef TRIGGER_DECODER_CONFIG_H
#define TRIGGER_DECODER_CONFIG_H

#include <stdint.h>

/* ============================ CRANK (mandatory) ========================== */
#define CRANK_CH              1U      /* TMG channel of the crank sensor */
#define CRANK_WHEEL_TEETH    60U      /* tooth positions per rev (teeth + missing) */
#define CRANK_WHEEL_MISSING   2U      /* missing teeth in the gap (60-2 -> 2) */
#define CRANK_GAP_RATIO       1.6f    /* period/prev_period above which it is the gap */
#define CRANK_TOOTH0_OFFSET   0.0f    /* crank angle at the 1st tooth after the gap (TDC) */
#define CRANK_INTERP_ORDER    1U      /* interpolation: 0 = constant velocity, 1 = with acceleration */

/* ============================ CAM (1..4, optional) ====================== */
/* Each cam reads an independent absolute angle (works with VVT, where the cam
 * moves within a window). The sequence of the last 'len' intervals, each
 * classified LONG/SHORT, identifies the position: with len=2 there are 4
 * combinations (LL, LS, SL, SS) -> 4 angles. Disable a cam with tmg_ch = 0.
 *
 * LONG/SHORT classification: an interval is LONG if
 *     period >= adaptive_short_reference * long_ratio
 * The reference auto-scales with engine speed, so long_ratio is simply
 * "how many times farther apart a long tooth is than a short one". */

#define CAM_COUNT            4U       /* number of cam slots handled */
#define CAM_PATTERN_MAXLEN   4U       /* max symbols in the comparison window */
#define CAM_PATTERN_MAXROWS  8U       /* max rows in a cam's table */

/* Cam-angle interpolation between teeth (for fine VVT control): the position
 * advances at the current engine speed and re-anchors to the recognized value
 * at each cam tooth. Requires cam tables expressed in engine degrees (0..720). */
#define CAM_INTERP_ENABLE    1U       /* 1 = interpolated cam angle; 0 = sample-and-hold */
#define CAM_CYCLE_DEG        720.0f   /* angular period of the cam in the frame used (wrap) */

typedef enum { CAM_S = 0, CAM_L = 1 } cam_sym_t;  /* S = short interval, L = long */

typedef struct
{
  cam_sym_t seq[CAM_PATTERN_MAXLEN]; /* expected sequence: [0]=oldest .. [len-1]=newest */
  float     angle_deg;               /* cam angle when the sequence matches */
} cam_row_t;

typedef struct
{
  uint8_t   tmg_ch;      /* TMG channel of the cam sensor; 0 = cam unused */
  float     long_ratio;  /* LONG if period >= short_reference * long_ratio */
  uint8_t   len;         /* symbols used in the window (1..CAM_PATTERN_MAXLEN) */
  uint8_t   rows;        /* valid rows in row[] */
  cam_row_t row[CAM_PATTERN_MAXROWS];
} cam_cfg_t;

/* Pattern tables. Example: CAM 1 with 4 teeth, 2-symbol window.
 * The angles (0/90/180/270) are placeholders: set them to match your cam. */
static const cam_cfg_t cam_cfg[CAM_COUNT] =
{
  /* -------------------------------- CAM 1 -------------------------------- */
  {
    .tmg_ch = 2U, .long_ratio = 1.5f, .len = 2U, .rows = 4U,
    .row = {
      { { CAM_L, CAM_L },   0.0f },   /* long  long  -> position 1 */
      { { CAM_L, CAM_S },  90.0f },   /* long  short -> position 2 */
      { { CAM_S, CAM_L }, 180.0f },   /* short long  -> position 3 */
      { { CAM_S, CAM_S }, 270.0f },   /* short short -> position 4 */
    }
  },
  /* -------------------------------- CAM 2 -------------------------------- */
  { .tmg_ch = 0U },   /* disabled */
  /* -------------------------------- CAM 3 -------------------------------- */
  { .tmg_ch = 0U },   /* disabled */
  /* -------------------------------- CAM 4 -------------------------------- */
  { .tmg_ch = 0U },   /* disabled */
};

/* ===================== FUSION crank + cam -> 0..720 ===================== */
/* Combines the crank (0..360) with one cam to produce the engine angle 0..720.
 * The chosen cam must be one configured above and have its table expressed in
 * ENGINE DEGREES 0..720, sharing the crank's origin (CRANK_TOOTH0_OFFSET). Each
 * recognition fixes the revolution; between recognitions the revolution is kept
 * by toggling it on each crank wrap. */
#define FUSION_ENABLE   1U     /* 1 = engine_angle 0..720; 0 = stays 0..360 (crank only) */
#define FUSION_CAM_CH   2U     /* TMG channel of the phase cam */

#endif /* TRIGGER_DECODER_CONFIG_H */
