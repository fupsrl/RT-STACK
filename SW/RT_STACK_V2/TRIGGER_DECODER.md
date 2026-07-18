# Trigger decoder and capture backend

The trigger subsystem is split in two deliberately independent parts:

1. A **capture backend** records a physical channel, edge polarity, per-channel
   sequence number, and timestamp.
2. `trigger_decoder.c` interprets those events using the wheel table in
   `Core/Inc/trigger_decoder_config.h`.

Consequently wheel configuration does not depend on whether a timestamp came
from EXTI, timer input capture, DMA, or a host test.

## Input capture decision for this board

The STM32G474VE alternate-function table gives the following options:

| TMG net | MCU pin | Direct timer input-capture option |
|---|---|---|
| TMG_OUT1 | PD10 | none |
| TMG_OUT2 | PD12 | TIM4_CH1 (AF2) |
| TMG_OUT3 | PC9 | TIM3_CH4 (AF2), TIM8_CH4 (AF4) |
| TMG_OUT4 | PD14 | TIM4_CH3 (AF2) |
| TMG_OUT5 | PA10 | TIM1_CH3 (AF6), TIM2_CH4 (AF10) |
| TMG_OUT6 | PD11 | none (TIM5_ETR is available, but is not a CCR capture channel) |
| TMG_OUT7 | PD13 | TIM4_CH2 (AF2) |
| TMG_OUT8 | PD15 | TIM4_CH4 (AF2) |
| TMG_OUT9 | PA8 | TIM1_CH1 (AF6) |

Source: STM32G474xB/xC/xE datasheet, Table 13, alternate functions:
<https://www.st.com/resource/en/datasheet/stm32g474ve.pdf>.

Input capture is better **where a suitable channel exists**: the edge latches
the counter in hardware, so ISR latency does not become timestamp jitter, and
DMA/overcapture detection can be used. It is not a good whole-board replacement
here because:

- the default crank is TMG_OUT1/PD10, which has no capture channel;
- TMG_OUT6/PD11 also has no direct capture channel;
- TIM4 would be shared by four inputs and is only 16-bit;
- PA8/TMG_OUT9 and PA10/TMG_OUT5 overlap TIM1 channels already configured on
  PC0 and PE13 in this project;
- PD10/TMG_OUT1 and PA10/TMG_OUT5 both use EXTI line 10, so they cannot both be
  routed through EXTI at the same time.

The practical backend for this PCB is therefore hybrid:

- run free-running 32-bit TIM2 as the common timestamp clock;
- capture PA10/TMG_OUT5 in TIM2_CH4 (this also resolves the EXTI10 collision);
- for the remaining channels, read `TIM2->CNT` in the EXTI handler;
- put both kinds of timestamp into the same `trigger_recorder_t` queues.

TIM2 is owned exclusively by this backend and the actuator deadline service;
do not assign it to another CubeMX peripheral or application feature.

This gives every event the same clock domain. A future PCB should route the
primary crank input to a 32-bit timer channel if hardware capture/DMA is wanted
for the signal where it matters most.

EXTI is adequate at ordinary engine edge rates when its interrupt remains
short and highest priority. The recorder prevents foreground polling from
losing already-serviced edges. It cannot recover two physical edges that
coalesce into one EXTI pending bit while interrupts are blocked; that remains
an acquisition-backend limitation. For this reason the STM32 backend rejects
`TRIGGER_EDGE_BOTH` on EXTI-backed channels; BOTH is supported only on
PA10/TMG_OUT5 hardware capture. Use a single edge on all other TMG inputs.

## Integration API

Allocate one recorder, decoder, and output:

```c
static trigger_recorder_t trigger_recorder;
static trigger_decoder_t trigger_decoder;
static trigger_output_t trigger_output;
```

Initialize them after the common timestamp timer has been configured, but
before capture interrupts are enabled:

```c
trigger_recorder_init(&trigger_recorder);

trigger_config_error_t error = trigger_decoder_context_init(
    &trigger_decoder,
    trigger_decoder_default_config(),
    trigger_timestamp_hz);

if (error != TRIGGER_CONFIG_OK) {
    /* Keep actuators disabled and report
       trigger_decoder_config_error_string(error). */
}
```

The acquisition handler supplies the timestamp; the recorder never reads a
timer itself:

```c
(void)trigger_recorder_record_isr(&trigger_recorder,
                                  tmg_channel,
                                  TRIGGER_EDGE_RISING,
                                  captured_tim2_timestamp);
```

Drain all queued events before asking for the current state. Read `now` only
after draining, which guarantees it cannot predate the latest processed edge:

```c
trigger_event_t event;
while (trigger_recorder_pop_oldest(&trigger_recorder, &event)) {
    trigger_decoder_process_event(&trigger_decoder, &event);
}

uint32_t now = TIM2->CNT;
trigger_decoder_poll(&trigger_decoder, now, &trigger_output);
```

If timer `CCxOF`, a DMA overwrite, or another backend loss is detected, report
it immediately:

```c
trigger_decoder_note_event_loss(&trigger_decoder, tmg_channel, lost_count);
```

The per-channel sequence field independently catches recorder queue gaps at the
next event. Do not silently continue after an overcapture: position is no
longer knowable.

## User wheel configuration

Edit only `Core/Inc/trigger_decoder_config.h` for normal engine adaptation.

- Add/remove entries in `trigger_wheels[]` and set `wheel_count`.
- Select any physical TMG channel and rising/falling polarity per wheel. BOTH
  edges are available only on PA10/TMG_OUT5 with this board backend.
- Use `TRIGGER_WHEEL_MISSING_TOOTH` for N-M geometry.
- Use `TRIGGER_WHEEL_INTERVAL_PATTERN` for a cyclic array such as
  `{SHORT, SHORT, LONG, LONG}`.
- Set the exact angle at pattern index zero, direction, interval tolerance,
  filtering, confirmation count, glitch rejection, and stop timeout per wheel.
- Select the primary speed wheel and optional 720-degree phase wheel in
  `trigger_default_config`.
- Set `phase_alignment_tolerance_deg` to the maximum measured crank/cam angle
  error. It must be below half a primary revolution so the engine turn remains
  uniquely identifiable; use a much tighter measured value in practice.

Pattern rotations must be unique. For example `{S,L,S,L}` is rejected because
rotating it by two entries gives the same sequence; no decoder could know which
of those two positions was observed.

The default enables only the existing 60-2 crank definition. The example cam
is disabled because its real mechanical angle must be measured on the engine.

## Host verification

From `RT_STACK_V2`:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc \
    Tests/trigger_decoder_test.c Core/Src/trigger_decoder.c \
    Core/Src/trigger_recorder.c -lm -o trigger_decoder_test
./trigger_decoder_test
```

The tests cover configuration rejection, 12-2 acquisition, exact reference
tooth counting, speed/angle interpolation, missing events, timeouts, arbitrary
LONG/SHORT acquisition, crank/cam 720-degree fusion, 32-bit timestamp wrap,
nonzero index/wrap continuity, phase acquisition during an interpolated wrap,
queue ordering, and queue overflow.
