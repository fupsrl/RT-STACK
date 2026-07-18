/**
 ******************************************************************************
 * @file    trigger_recorder.h
 * @brief   Bounded ISR-to-foreground queues for timestamped trigger edges.
 *
 * There is one single-producer/single-consumer queue per TMG channel.  This
 * avoids races between unrelated capture interrupts.  The foreground pop
 * function merges the channel queues by timestamp, so crank/cam events reach
 * the decoder in chronological order.
 ******************************************************************************
 */
#ifndef TRIGGER_RECORDER_H
#define TRIGGER_RECORDER_H

#include "trigger_decoder.h"

#ifndef TRIGGER_RECORDER_QUEUE_LENGTH
#define TRIGGER_RECORDER_QUEUE_LENGTH  64U
#endif

#if ((TRIGGER_RECORDER_QUEUE_LENGTH < 2U) || \
     ((TRIGGER_RECORDER_QUEUE_LENGTH & (TRIGGER_RECORDER_QUEUE_LENGTH - 1U)) != 0U))
#error "TRIGGER_RECORDER_QUEUE_LENGTH must be a power of two and at least 2"
#endif

typedef struct
{
  trigger_event_t event[TRIGGER_RECORDER_QUEUE_LENGTH];
  /* These objects intentionally are not merely volatile.  trigger_recorder.c
   * accesses shared indices and dropped_events only through GCC __atomic
   * acquire/release operations, which provide both the required compiler
   * barrier and the Cortex-M memory ordering.  next_sequence has one ISR
   * writer and is never read by the foreground. */
  uint16_t write_index;
  uint16_t read_index;
  uint16_t next_sequence;
  uint32_t dropped_events;
} trigger_channel_queue_t;

typedef struct
{
  trigger_channel_queue_t channel[TRIGGER_CHANNEL_COUNT];
} trigger_recorder_t;

/** Initialize before capture interrupts are enabled. */
void trigger_recorder_init(trigger_recorder_t *recorder);

/**
 * Record an edge from its channel's ISR/capture callback.
 *
 * The timestamp must already have been latched/read by the backend.  Sequence
 * is assigned even when the queue is full, allowing the decoder to detect the
 * gap at the next successfully queued event.
 */
bool trigger_recorder_record_isr(trigger_recorder_t *recorder,
                                 uint8_t channel,
                                 trigger_edge_t edge,
                                 uint32_t timestamp);

/** Pop the oldest timestamp across all channel queues. */
bool trigger_recorder_pop_oldest(trigger_recorder_t *recorder,
                                 trigger_event_t *event);

/** Monotonic per-channel overflow diagnostic. */
uint32_t trigger_recorder_dropped(const trigger_recorder_t *recorder,
                                  uint8_t channel);

/** Number of queued records across every channel at this instant. */
uint32_t trigger_recorder_pending(const trigger_recorder_t *recorder);

#endif /* TRIGGER_RECORDER_H */
