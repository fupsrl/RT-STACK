/**
 ******************************************************************************
 * @file    trigger_recorder.c
 * @brief   Lock-free per-channel trigger event queues.
 ******************************************************************************
 */
#include "trigger_recorder.h"

#include <string.h>

#define QUEUE_MASK  (TRIGGER_RECORDER_QUEUE_LENGTH - 1U)

void trigger_recorder_init(trigger_recorder_t *recorder)
{
  if (recorder != NULL)
  {
    memset(recorder, 0, sizeof(*recorder));
  }
}

bool trigger_recorder_record_isr(trigger_recorder_t *recorder,
                                 uint8_t channel,
                                 trigger_edge_t edge,
                                 uint32_t timestamp)
{
  if ((recorder == NULL) || (channel == 0U) ||
      (channel >= TRIGGER_CHANNEL_COUNT))
  {
    return false;
  }

  trigger_channel_queue_t *queue = &recorder->channel[channel];
  uint16_t sequence = queue->next_sequence++;
  uint16_t write_index = __atomic_load_n(&queue->write_index, __ATOMIC_RELAXED);
  uint16_t next_index = (uint16_t)((write_index + 1U) & QUEUE_MASK);
  uint16_t read_index = __atomic_load_n(&queue->read_index, __ATOMIC_ACQUIRE);

  if (next_index == read_index)
  {
    (void)__atomic_fetch_add(&queue->dropped_events, 1U, __ATOMIC_RELAXED);
    return false;
  }

  queue->event[write_index] = (trigger_event_t) {
    .timestamp = timestamp,
    .sequence = sequence,
    .channel = channel,
    .edge = edge,
  };

  /* Publish only after the complete event has been written. */
  __atomic_store_n(&queue->write_index, next_index, __ATOMIC_RELEASE);
  return true;
}

bool trigger_recorder_pop_oldest(trigger_recorder_t *recorder,
                                 trigger_event_t *event)
{
  if ((recorder == NULL) || (event == NULL))
  {
    return false;
  }

  bool found = false;
  uint8_t selected_channel = 0U;
  uint16_t selected_read_index = 0U;
  trigger_event_t selected_event = {0};

  for (uint8_t channel = 1U; channel < TRIGGER_CHANNEL_COUNT; channel++)
  {
    trigger_channel_queue_t *queue = &recorder->channel[channel];
    uint16_t read_index = __atomic_load_n(&queue->read_index, __ATOMIC_RELAXED);
    uint16_t write_index = __atomic_load_n(&queue->write_index, __ATOMIC_ACQUIRE);

    if (read_index == write_index)
    {
      continue;
    }

    trigger_event_t candidate = queue->event[read_index];
    if ((!found) ||
        ((int32_t)(candidate.timestamp - selected_event.timestamp) < 0))
    {
      found = true;
      selected_channel = channel;
      selected_read_index = read_index;
      selected_event = candidate;
    }
  }

  if (!found)
  {
    return false;
  }

  *event = selected_event;
  trigger_channel_queue_t *selected = &recorder->channel[selected_channel];
  uint16_t next_index = (uint16_t)((selected_read_index + 1U) & QUEUE_MASK);
  __atomic_store_n(&selected->read_index, next_index, __ATOMIC_RELEASE);
  return true;
}

uint32_t trigger_recorder_dropped(const trigger_recorder_t *recorder,
                                  uint8_t channel)
{
  if ((recorder == NULL) || (channel == 0U) ||
      (channel >= TRIGGER_CHANNEL_COUNT))
  {
    return 0U;
  }

  return __atomic_load_n(&recorder->channel[channel].dropped_events,
                         __ATOMIC_RELAXED);
}

uint32_t trigger_recorder_pending(const trigger_recorder_t *recorder)
{
  uint32_t total = 0U;
  uint8_t channel;

  if (recorder == NULL)
  {
    return 0U;
  }

  for (channel = 1U; channel < TRIGGER_CHANNEL_COUNT; ++channel)
  {
    const trigger_channel_queue_t *queue = &recorder->channel[channel];
    uint16_t read_index = __atomic_load_n(&queue->read_index, __ATOMIC_RELAXED);
    uint16_t write_index = __atomic_load_n(&queue->write_index, __ATOMIC_ACQUIRE);
    total += (uint32_t)((write_index - read_index) & QUEUE_MASK);
  }
  return total;
}
