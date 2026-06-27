/**
 ******************************************************************************
 * @file    trigger_decoder.c
 * @brief   60-2 crankshaft decoder + up to 4 independent cam decoders + fusion.
 *
 * Crank : N-M wheel (default 60-2), output 0..360 deg with sub-tooth
 *         interpolation (constant velocity or acceleration-compensated).
 * Cam   : each cam reads an independent absolute angle; the sequence of the
 *         last 'len' LONG/SHORT intervals identifies the position (works with
 *         VVT, where the cam shifts within an angular window).
 * Fusion: combines crank (0..360) with one cam to give the engine angle 0..720.
 *
 * All pattern configuration is in trigger_decoder_config.h. Replace this file
 * to use a different decode algorithm, keeping the trigger_decoder.h interface.
 ******************************************************************************
 */
#include "trigger_decoder.h"
#include "trigger_decoder_config.h"

#define CRANK_DEG_PER_TOOTH  (360.0f / (float)CRANK_WHEEL_TEETH)

/* Wrap x into [0, period). */
static float wrapf(float x, float period)
{
  while (x >= period) x -= period;
  while (x < 0.0f)    x += period;
  return x;
}

/* Consistent snapshot of one channel. Seqlock: the ISR increments edge_count
 * last, so if it is unchanged across the two reads the data is not torn.
 * The ISRs are never blocked. */
static void channel_snapshot(const trigger_input_t *in, uint32_t ch,
                             uint32_t *count, uint32_t *period, uint32_t *edge)
{
  uint32_t c1, c2;
  do
  {
    c1      = in->edge_count[ch];
    *period = in->period_cyc[ch];
    *edge   = in->last_edge_cyc[ch];
    c2      = in->edge_count[ch];
  } while (c1 != c2);
  *count = c2;
}

/* ============================== Crankshaft =============================== */
static struct {
  uint32_t seen_count;   /* edge_count already processed */
  uint32_t period;       /* last tooth period (cycles) */
  uint32_t period_prev;  /* previous tooth period; 0 = invalid (no accel) */
  uint32_t edge_cyc;     /* timestamp of the last tooth */
  int32_t  tooth;        /* tooth index; -1 = not synced */
  bool     synced;
} crank;

/* Degrees advanced from the last crank edge, 'elapsed' cycles ago.
 * Order 0: constant velocity (last tooth).
 * Order 1: constant acceleration from the last two tooth periods,
 *          theta(t) = w0*t + 0.5*a*t^2. */
static float crank_advance(uint32_t elapsed)
{
  float dtheta = CRANK_DEG_PER_TOOTH;
  float t = (float)elapsed;

#if (CRANK_INTERP_ORDER >= 1)
  if (crank.period_prev != 0U)
  {
    /* Average tooth speeds hold at each tooth's time-midpoint. */
    float w_last = dtheta / (float)crank.period;
    float w_prev = dtheta / (float)crank.period_prev;
    float a  = (w_last - w_prev) / (0.5f * ((float)crank.period + (float)crank.period_prev));
    float w0 = w_last + a * (0.5f * (float)crank.period);   /* speed at the edge */
    return (w0 * t) + (0.5f * a * t * t);
  }
#endif
  return (t / (float)crank.period) * dtheta;
}

static void crank_update(const trigger_input_t *in, trigger_output_t *out)
{
  uint32_t count, period, edge;
  channel_snapshot(in, CRANK_CH, &count, &period, &edge);

  /* Process a new, valid edge (the first edge has no period yet). */
  if ((count != crank.seen_count) && (count >= 2U))
  {
    bool is_gap = (crank.period != 0U) &&
                  ((float)period >= CRANK_GAP_RATIO * (float)crank.period);

    if (is_gap)
    {
      /* This edge is the first tooth after the gap: re-align the wheel. The
       * gap spans (missing + 1) intervals -> estimate the normal period. */
      crank.tooth       = 0;
      crank.synced      = true;
      crank.period_prev = 0U;                  /* accel invalid across the gap */
      crank.period      = period / (CRANK_WHEEL_MISSING + 1U);
    }
    else
    {
      if (crank.synced)
        crank.tooth = (crank.tooth + 1) % (int32_t)CRANK_WHEEL_TEETH;
      crank.period_prev = crank.period;
      crank.period      = period;
    }

    crank.edge_cyc   = edge;
    crank.seen_count = count;
  }

  float angle = 0.0f;
  float rpm   = 0.0f;

  if (crank.synced && (crank.tooth >= 0) && (crank.period != 0U))
  {
    float max_adv = (float)(CRANK_WHEEL_MISSING + 1U) * CRANK_DEG_PER_TOOTH;
    float adv = crank_advance(in->now_cyc - crank.edge_cyc);  /* unsigned sub: wrap-safe */
    if (adv > max_adv) adv = max_adv;          /* clamp inside the gap */
    if (adv < 0.0f)    adv = 0.0f;

    angle = wrapf(CRANK_TOOTH0_OFFSET + ((float)crank.tooth * CRANK_DEG_PER_TOOTH) + adv, 360.0f);
    rpm   = 60.0f * (float)in->cpu_hz / ((float)CRANK_WHEEL_TEETH * (float)crank.period);
  }

  out->angle[CRANK_CH]     = angle;
  out->ch_synced[CRANK_CH] = crank.synced;
  out->crank_angle_deg     = angle;
  out->rpm                 = rpm;
  out->synced              = crank.synced;
}

/* ================================== Cams ================================= */
static struct {
  uint32_t  seen_count;                /* edge_count already processed */
  float     ref;                       /* adaptive short-interval reference */
  cam_sym_t hist[CAM_PATTERN_MAXLEN];  /* symbol window, newest at the tail */
  uint8_t   fill;                      /* symbols accumulated */
  float     angle;                     /* last recognized angle */
  uint32_t  edge_cyc;                  /* timestamp anchoring 'angle' */
  bool      synced;
} cams[CAM_COUNT];

static void cam_update(const trigger_input_t *in, uint32_t c)
{
  const cam_cfg_t *cfg = &cam_cfg[c];
  uint8_t len = cfg->len;
  if (len == 0U) return;
  if (len > CAM_PATTERN_MAXLEN) len = CAM_PATTERN_MAXLEN;

  uint32_t count, period, edge;
  channel_snapshot(in, cfg->tmg_ch, &count, &period, &edge);
  if ((count == cams[c].seen_count) || (count < 2U))
    return;                                    /* no new valid edge */
  cams[c].seen_count = count;

  /* Classify the interval LONG/SHORT against the adaptive short reference. */
  if (cams[c].fill == 0U)
    cams[c].ref = (float)period;               /* bootstrap on the first interval */
  cam_sym_t sym = ((float)period >= cams[c].ref * cfg->long_ratio) ? CAM_L : CAM_S;

  /* Track the reference: fast on SHORT (anchor to the short tooth), slow on
   * LONG (recovery, so a bad estimate cannot stay stuck). */
  cams[c].ref += ((float)period - cams[c].ref) * ((sym == CAM_S) ? 0.25f : 0.02f);

  /* Push the symbol into the window (newest at the tail). */
  for (uint8_t i = 1U; i < len; i++)
    cams[c].hist[i - 1U] = cams[c].hist[i];
  cams[c].hist[len - 1U] = sym;
  if (cams[c].fill < len) cams[c].fill++;

  /* Match the window against the pattern table. */
  if (cams[c].fill < len) return;
  for (uint8_t r = 0U; r < cfg->rows; r++)
  {
    bool match = true;
    for (uint8_t i = 0U; i < len; i++)
      if (cams[c].hist[i] != cfg->row[r].seq[i]) { match = false; break; }
    if (match)
    {
      cams[c].angle    = cfg->row[r].angle_deg;
      cams[c].edge_cyc = edge;                 /* anchor for interpolation */
      cams[c].synced   = true;
      return;
    }
  }
}

/* Cam output angle: interpolated to the current engine speed (for fine VVT),
 * re-anchored to the recognized value at each cam tooth. */
static float cam_output(const trigger_input_t *in, uint32_t c)
{
#if CAM_INTERP_ENABLE
  if (crank.synced && (crank.period != 0U) && cams[c].synced)
  {
    float rate = CRANK_DEG_PER_TOOTH / (float)crank.period;   /* deg per cycle */
    return wrapf(cams[c].angle + (float)(in->now_cyc - cams[c].edge_cyc) * rate, CAM_CYCLE_DEG);
  }
#else
  (void)in;
#endif
  return cams[c].angle;
}

/* ================================= Fusion =============================== */
#if FUSION_ENABLE
static struct {
  int      rev;          /* 0/1: which crank revolution within the 720 cycle */
  bool     phase_known;
  float    last_crank;   /* crank angle on the previous pass (wrap detection) */
  uint32_t cam_seen;     /* edge_count of the phase cam */
} fus;

/* Combine crank (0..360) + phase cam -> engine angle 0..720. */
static void fusion_update(const trigger_input_t *in, trigger_output_t *out)
{
  float crank_ang = out->angle[CRANK_CH];

  /* A crank wrap (360 -> 0) toggles the revolution within the cycle. */
  if (out->synced)
  {
    if (fus.phase_known && (fus.last_crank > 270.0f) && (crank_ang < 90.0f))
      fus.rev ^= 1;
    fus.last_crank = crank_ang;
  }

  /* A new phase-cam recognition fixes the revolution, by comparing the cam's
   * engine-frame angle with the current crank angle (diff ~0 -> rev 0,
   * diff ~360 -> rev 1). */
  uint32_t cc = in->edge_count[FUSION_CAM_CH];
  if ((cc != fus.cam_seen) && out->ch_synced[FUSION_CAM_CH])
  {
    float diff = wrapf(out->angle[FUSION_CAM_CH] - crank_ang, 720.0f);
    fus.rev = ((diff >= 180.0f) && (diff < 540.0f)) ? 1 : 0;
    fus.phase_known = true;
  }
  fus.cam_seen = cc;

  /* Engine angle: 0..720 once the phase is known, otherwise stay at 0..360. */
  out->phase_known     = fus.phase_known && out->synced;
  out->crank_angle_deg = out->phase_known ? (crank_ang + (float)fus.rev * 360.0f) : crank_ang;
}
#endif /* FUSION_ENABLE */

/* =============================== Public API ============================= */
void trigger_decoder_init(void)
{
  crank.seen_count  = 0U;
  crank.period      = 0U;
  crank.period_prev = 0U;
  crank.edge_cyc    = 0U;
  crank.tooth       = -1;
  crank.synced      = false;

  for (uint32_t c = 0U; c < CAM_COUNT; c++)
  {
    cams[c].seen_count = 0U;
    cams[c].ref        = 0.0f;
    cams[c].fill       = 0U;
    cams[c].angle      = 0.0f;
    cams[c].edge_cyc   = 0U;
    cams[c].synced     = false;
    for (uint32_t i = 0U; i < CAM_PATTERN_MAXLEN; i++)
      cams[c].hist[i] = CAM_S;
  }

#if FUSION_ENABLE
  fus.rev         = 0;
  fus.phase_known = false;
  fus.last_crank  = 0.0f;
  fus.cam_seen    = 0U;
#endif
}

void trigger_decoder_update(const trigger_input_t *in, trigger_output_t *out)
{
  for (uint32_t i = 0U; i < TMG_CH_COUNT; i++)
  {
    out->angle[i]     = 0.0f;
    out->ch_synced[i] = false;
  }
  out->phase_known = false;

  crank_update(in, out);

  for (uint32_t c = 0U; c < CAM_COUNT; c++)
  {
    uint8_t ch = cam_cfg[c].tmg_ch;
    if (ch == 0U) continue;                    /* slot disabled */
    cam_update(in, c);
    out->angle[ch]     = cam_output(in, c);
    out->ch_synced[ch] = cams[c].synced;
  }

#if FUSION_ENABLE
  fusion_update(in, out);                      /* crank 0..360 + cam -> 0..720 */
#endif
}
