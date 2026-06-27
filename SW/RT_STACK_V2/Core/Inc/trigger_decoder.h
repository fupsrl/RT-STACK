/**
 ******************************************************************************
 * @file    trigger_decoder.h
 * @brief   Trigger-wheel decoder interface.
 *
 * The interface is stable: to change trigger pattern you only replace
 * trigger_decoder.c (e.g. 60-2, 36-1, 24+1, custom cam teeth...), leaving
 * main.c and this header untouched.
 *
 * Data flow:
 *   EXTI ISR (main.c)  ->  tmg_* arrays (raw timestamps)  ->  this decoder
 *   ->  crankshaft angle  ->  engine_angle (main.c)
 ******************************************************************************
 */
#ifndef TRIGGER_DECODER_H
#define TRIGGER_DECODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of TMG channels. Indexed by TMG number (1..9); slots 0 and 5 are
 * unused. Must match the size of the tmg_* arrays in main.c. */
#define TMG_CH_COUNT  10U

/**
 * @brief Decoder inputs: timing data produced by the ISRs + current instant.
 *
 * The pointers reference volatile memory updated in interrupt context; the
 * decoder takes a consistent snapshot internally, so the caller passes them
 * as-is. Indexed by channel (see tmg_* in main.c).
 */
typedef struct
{
  const volatile uint32_t *last_edge_cyc;  /* tmg_last_edge_cyc[] : timestamp of last edge */
  const volatile uint32_t *period_cyc;     /* tmg_period_cyc[]    : cycles between the last two edges */
  const volatile uint32_t *edge_count;     /* tmg_edge_count[]    : number of edges seen */
  uint32_t now_cyc;                        /* cycle_now()         : current DWT counter */
  uint32_t cpu_hz;                         /* SystemCoreClock     : for time/rpm conversions */
} trigger_input_t;

/**
 * @brief Decoder outputs.
 */
typedef struct
{
  float angle[TMG_CH_COUNT];     /* absolute angle (deg) per channel: crank + each cam */
  bool  ch_synced[TMG_CH_COUNT]; /* per channel: true when its angle is valid */
  float crank_angle_deg;         /* crankshaft position: 0..720 if phase known, else 0..360 */
  float rpm;                     /* instantaneous engine speed (rpm) from the crank */
  bool  synced;                  /* true when the crank is locked */
  bool  phase_known;             /* true when crank_angle_deg spans the full 0..720 cycle */
} trigger_output_t;

/** Reset the decoder's internal state. Call once before the main loop. */
void trigger_decoder_init(void);

/**
 * @brief Process the timing data and update the outputs.
 *
 * Call once per main-loop iteration. Non-blocking and independent of any main
 * global: everything comes in via @p in and out via @p out.
 */
void trigger_decoder_update(const trigger_input_t *in, trigger_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TRIGGER_DECODER_H */
