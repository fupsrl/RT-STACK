# RT_STACK_V2 firmware

This firmware is organized around four intentionally separate concerns:

1. `trigger_capture_stm32.c` timestamps electrical edges.
2. `trigger_decoder.c` interprets user-defined wheel geometry.
3. `engine_control.c` gates the ignition/injection schedulers through a
   fail-safe engine state machine.
4. `dcdc_control.c` regulates VBUS, `dcdc_pwm_stm32.c` owns the
   hardware-synchronous two-phase TIM1 waveform, and
   `dcdc_platform_stm32.c` contains the board/ADC/interrupt integration.

`main.c` is now platform startup and sensor glue. Engine behavior is configured
in tables rather than embedded in the superloop.

## Start here

### 1. Configure the trigger wheels

Edit `Core/Inc/trigger_decoder_config.h`.

- Add one entry to `trigger_wheels[]` for every physical wheel.
- Set its TMG input channel, active edge, mechanical cycle, measured index-zero
  angle, direction, tolerances, filtering, confirmation count, and timeout.
- Choose either a conventional missing-tooth `N-M` wheel or an arbitrary cyclic
  `SHORT`/`LONG` interval pattern.
- Set `wheel_count`, `primary_wheel`, and optional `phase_wheel` in
  `trigger_default_config`.
- Set a measured crank/cam phase-alignment tolerance when a phase wheel is
  enabled.

The default is a crank-only 60-2 wheel on TMG1/PD10. The documented cam example
is deliberately disabled because its actual installed angle must be measured.
Sequential ignition and injection will not run without a configured phase
wheel.

The decoder itself accepts increasing or decreasing angle conventions and
general related wheel cycles. The present ignition/injection scheduler is
intentionally narrower: it enables actuators only for a +1 primary direction,
a 360-degree primary cycle, and a 720-degree engine cycle. Other combinations
remain usable for decoder development but fail actuator compatibility checks.

See [TRIGGER_DECODER.md](TRIGGER_DECODER.md) for pattern conventions, capture
pin capabilities, and integration details.

### 2. Configure the engine and actuators

Edit `Core/Inc/engine_config.h`.

- Set `ENGINE_CYLINDER_COUNT`.
- Give every cylinder an explicit compression TDC, ignition output, and injector
  output. Explicit TDCs support even-fire and odd-fire engines equally.
- Set per-cylinder ignition advance/dwell defaults and accepted limits.
- Set per-cylinder injection start, duration, boost time, and boost/hold DAC
  values.
- Select sequential or intentionally crank-only operation with
  `IGNITION_REQUIRES_PHASE_SYNC` and `INJECTION_REQUIRES_PHASE_SYNC`.

The engine gate derives its phase requirement from the currently enabled
commands. Enabling a sequential command without cam phase moves the controller
to `SYNCING`; disabling all sequential commands permits an intentionally
configured crank-only scheduler without requiring a reboot.

Physical PCB routing is separate in `Core/Inc/board_output_config.h`; it should
change only for a board revision.

Injection is disabled and calibration-locked by default. The fitted ACS722
sensor is bidirectional, so its nominal midscale is not a safe boost limit and
the example raw DAC codes must not be used with a load. Measure zero-current
code, polarity, codes/ampere, and the required boost/hold currents on every
pair; enter the resulting codes, then set
`INJECTION_CURRENT_CALIBRATION_VALID=1` and deliberately enable injection.
The six current comparators default to 10 mV hysteresis through
`BOARD_INJECTOR_COMPARATOR_HYSTERESIS`; validate that value against measured
noise and current ripple as part of the same calibration.

### 3. Deliberately arm outputs

`ENGINE_OUTPUTS_ARM_AT_BOOT` defaults to `0`. This keeps the firmware safe for
decoder and communications development. Setting it to `1` requests output
operation, but does not bypass configuration validation, synchronization, RPM
limits, event deadlines, or the fault latch.

The application can instead call:

```c
engine_control_request_outputs(true);
```

Only a fault-free `ENGINE_MODE_RUNNING` state can energize an output. Clear a
recoverable fault only after revoking output permission:

```c
engine_control_request_outputs(false);
engine_control_clear_faults();
```

### Stationary injector bench API

The periodic injector test is deliberately unavailable in production builds.
It requires both `INJECTION_CURRENT_CALIBRATION_VALID=1` and
`ENGINE_INJECTOR_TEST_API_ENABLED=1` in `Core/Inc/engine_config.h`. Enable it
only after the peak-and-hold current calibration has been measured on the
assembled board.

The API addresses the physical injector output number (`1..12`), not a
cylinder number. The selected output must appear exactly once in
`engine_cylinders[]`, because that cylinder entry supplies its calibrated
boost/hold profile and accepted pulse-width limits.

```c
engine_injector_test_config_t test = {
  .injector_output = 2U, /* physical INJ2 output */
  .period_ms = 100U,
  .pulse_width_us = 5000U
};

engine_injector_test_result_t result =
    engine_control_start_injector_test(&test);

/* Call engine_control_service() normally while the test is armed. */

engine_control_stop_injector_test();
```

`period_ms` is the start-to-start period and `pulse_width_us` is the injector
on-time. The first pulse occurs only after one complete period; late foreground
service skips missed periods and never emits a catch-up burst. Configured
minimum off-time, maximum duty cycle, mapped-cylinder duration limits, and the
normal 100 us hard-off deadline still apply.

The controller accepts the test only while the engine is stationary, normal
outputs are disarmed, no trigger records are pending, the trigger inputs have
been quiet for `ENGINE_INJECTOR_TEST_QUIET_TIME_MS` (also after startup or an
overcapture), and no fault is latched.
Only one injector may be tested at a time. While armed, the controller reports
`ENGINE_MODE_INJECTOR_TEST` and rejects normal output arming. Any trigger edge
immediately switches every injector off, cancels the test, and latches
`INJECTION_FAULT_DEBUG_INTERLOCK`; revoke output permission and clear faults
before attempting another test.

## Two-phase DCDC control

The boost converter uses two independent, active-high low-side phases:

- PC0/TIM1_CH1 drives Q31/L1 through the non-inverting TC4427;
- PE13/TIM1_CH3 drives Q32/L2 through the other TC4427 channel.

TIM1 counts up and down. CH1 runs PWM1 with `CCR1 = on_ticks`; CH3 runs PWM2
with `CCR3 = ARR - on_ticks`. The two pulses have equal duty and their centers
and corresponding steady-state edges are exactly one half-period apart. Phase
overlap above 50% is intentional for an interleaved boost converter; these are
not complementary half-bridge switches and timer deadtime is not applied.

All user calibration is in `Core/Inc/dcdc_config.h`. Before any output can be
enabled, that file requires explicit confirmation of feedback linearity,
hardware current-limit protection, and measured power-stage calibration. Then
enter the measured switching frequency, minimum on/off times, duty envelope,
VBUS target/trip/recovery limits, raw ADC watchdog threshold, soft-start rate,
and PID gains. The target and safety values are deliberately zero in the
repository because safe values cannot be inferred from unspecified inductor,
capacitor, thermal, and load ratings.

Runtime control is explicit:

```c
/* PWM stays off until several fresh, plausible VBUS samples are confirmed. */
dcdc_control_request_enable(true);

/* A higher target follows the configured soft-start ramp. */
dcdc_control_set_target_mv(measured_safe_target_mv);

/* Stop switching. This does not discharge the output capacitors. */
dcdc_control_request_enable(false);

/* After a fault: keep disabled, provide a fresh safe VBUS sample, then clear. */
dcdc_control_clear_faults();
```

The boolean APIs report synchronous request acceptance, not a guarantee that
the converter remains enabled. A watchdog or emergency interrupt can latch a
fault immediately afterward; read `dcdc_control_get_state()` for authoritative
state and fault bits.

The controller accepts at most one update per configured control period and
uses the real, deadline-bounded elapsed time for its soft-start, filtered
derivative-on-measurement, and back-calculation anti-windup calculations. It
never emits catch-up duty bursts. Compare values are
preloaded and written as a pair while update events are suppressed, so a timer
boundary cannot mix one old phase with one new phase. A zero-duty command
clears MOE/CEN/channel enables and returns both pins to GPIO-low; a later
nonzero command starts both timer channels as one hardware operation.

VBUS uses PC3/ADC1_IN9. The feedback network is physically:

```text
VBUS -- R145 25 kOhm --+-- VBUS_ADC / PC3
                       +-- R146 1 kOhm -- GND
                       +-- D53 LED ------ GND
                           anode   cathode
```

Without D53, `VADC = VBUS / 26`. D53 is not in series as a harmless indicator;
it is parallel to R146. Once `VADC` approaches the LED forward voltage, LED
current shunts the lower resistor and the transfer becomes nonlinear:

```text
(VBUS - VADC) / 25k = VADC / 1k + I_LED(VADC)
```

The approximate onset is `VBUS = 26 * Vf`: about 47 V for a 1.8 V red LED,
52 V for 2.0 V, 57 V for 2.2 V, or 78 V for a 3.0 V blue/white LED. D53 has no
specified part number, color, minimum-hot `Vf`, or datasheet, so no onset can be
relied on. As a simple 2.0 V clamp illustration, a real 60/80/100 V bus can be
reported near 52 V while LED current rises to roughly 0.32/1.12/1.92 mA. A real
LED has a sloped I-V curve rather than a hard clamp, but it remains strongly
temperature- and part-dependent. The PID can therefore integrate toward maximum
duty while real VBUS keeps rising, and the ADC analog watchdog is blinded if its
threshold is above the same apparent plateau.

D53 must be removed/DNP and the divider characterized before closed-loop use.
On the next PCB, put the indicator on its own rated resistor chain or buffered
driver. R145 is also a single unspecified 0402: with an approximately 2 V ADC
node it dissipates about 85 mW at 48 V, 135 mW at 60 V, and 243 mW at 80 V,
while sustaining roughly 46/58/78 V. Replace it with a voltage- and
power-derated series resistor chain, and add an independent precision
overvoltage comparator to TIM1 break or the power-stage enable.

There is also no phase-current sensing, cycle-by-cycle current limit, external
DCDC enable, or TIM1 break input on the current board. Firmware adds a raw ADC
analog-watchdog shutdown and a 1 ms off-only foreground deadline check, but
neither can protect against inductor saturation, a short, a failed MCU, or a
blocked interrupt. Hardware current limiting, a break/enable path, input
pull-downs, battery transient protection, and validated component ratings are
required before loaded operation. Stopping PWM leaves roughly millifarads of
VBUS capacitance charged; treat OFF as "not switching," not "safe voltage."

## Trigger capture: input capture versus EXTI

Input capture improves timestamp precision because the timer latches the edge
before interrupt latency. It does not remove the ISR or increase throughput by
itself; DMA would be required for that.

A full input-capture conversion is not possible on this PCB: the default crank
PD10 and PD11 have no direct capture channel, several alternatives consume a
shared 16-bit timer, and TIM1 channels conflict with existing outputs. The
implemented hybrid uses:

- free-running 32-bit TIM2 at the APB1 timer clock as the common timebase;
- TIM2_CH4 hardware capture for PA10/TMG5;
- short EXTI handlers that read `TIM2->CNT` for the remaining TMG pins;
- lock-free per-channel queues merged chronologically in the foreground.

Rising+falling (`TRIGGER_EDGE_BOTH`) is accepted only on PA10/TMG5. An EXTI
pending bit cannot retain two opposite edges or their order, so the platform
rejects BOTH-edge configurations on every EXTI-backed TMG input.

This also resolves the PA10/PD10 EXTI-line-10 collision. Queue overflow or TIM2
overcapture is never hidden: synchronization is invalidated and all actuators
are shut down.

## Ignition and injection behavior

Commands are explicit and validated:

```c
spark_command_t spark = { true, 20.0f, 3000U };
injection_command_t fuel = { true, 360.0f, 8000U };

spark_set_command(1U, &spark);
injection_set_command(1U, &fuel);
```

An active event keeps an immutable copy of its timing. Updating a command only
affects the next engine cycle. Shared peak-and-hold injector pairs have explicit
ownership; ambiguous simultaneous events fail closed instead of depending on
table order.

TIM2 compare channel 1 runs a 100 us safety service. It may end coil dwell,
BOOST, or injection, but it can never start an output. Main-loop stalls,
position loss, invalid numeric commands, excessive angle steps, output-pair
collisions, CPU faults, queue overflow, and capture overrun all lead toward OFF.

The independent hardware watchdog is also enabled after platform initialization.
It is refreshed only after a complete foreground control pass and resets the MCU
after roughly one second if that loop locks up. The timeout uses the imprecise
LSI clock and is deliberately not frozen by a debugger; it complements the
100 us actuator deadline service rather than replacing it.

Important hardware limitation: the current PCB has no independent global
driver-enable/break line. Timer IRQ deadlines protect against foreground stalls,
but no software can force GPIO low while the CPU is halted or physically
failed. The actuator/logic inputs also have no external fail-safe pull-downs,
so they are indeterminate during reset before firmware can configure the GPIOs.
Firmware now drives every IGN/BOOST/SEL/comparator-output and DCDC driver net
low directly from `Reset_Handler`, before clock/RAM/C-runtime initialization,
which shortens that window but cannot eliminate the reset interval itself.
Validate with protected dummy loads before connecting coils, injectors, or the
power stage; a future board must add pull-downs and a hardware
watchdog-controlled global driver enable.

PB8/BOOT0 also has no external pull-down. Firmware programs the option bytes to
ignore BOOT0 and boot from flash, but that code can run only after the application
has already booted once. Production boards must program those option bytes during
manufacturing or add a physical pull-down; firmware alone cannot guarantee the
first cold boot.

## Sensors and CAN

- PB2/ADC2_IN12 now supplies the external VBATT reading and uses a long sampling
  time suitable for a divider.
- ADC1 VREFINT is used to calculate VDDA for MCU temperature compensation.
- CAN frames are nonblocking and defined in `RT_STACK_V2.dbc`:
  - `0x100` backward-compatible angle/temperature/raw-voltage status;
  - `0x101` RPM, engine mode, sync/output flags, epoch, and VDDA;
  - `0x102` engine/trigger/actuator faults and loss counters.

### CAN hardware blocker

The present PCB connects MCU PA11 to TJA1051 `TXD` and PA12 to TJA1051 `RXD`,
but STM32G474 FDCAN1 requires PA11=`RX` and PA12=`TX`. Besides preventing CAN
operation, enabling the peripheral would make the PA12 output contend with the
transceiver `RXD` output. Firmware therefore leaves both pins passive and CAN
disabled by default. Swap the two logic-side nets (or use a corrected board),
then set `BOARD_FDCAN1_TRANSCEIVER_WIRING_FIXED` to `1` in
`Core/Inc/board_config.h`.

## CubeMX ownership and regeneration

`Core` is STM32CubeMX's application-code directory; it is not the CMSIS CPU
core. The ownership boundary inside it is important:

- `main.c`, `stm32g4xx_it.c`, `stm32g4xx_hal_msp.c`, and their matching headers
  are Cube-generated scaffolding. Custom code in them is limited to marked
  `USER CODE` blocks.
- Timer modes, pins, DMA and the HSE/PLL configuration are owned by
  `RT_STACK_V2.ioc`. The generated `SystemClock_Config()` and `MX_TIM1_Init()`
  bodies must not be hand-maintained.
- Files such as `engine_control.c`, `trigger_decoder.c`, `dcdc_control.c`, and
  `dcdc_platform_stm32.c` are user-owned modules and Cube leaves them intact.
- `main.c` contains only short initialization/service calls for those modules.
  The generated SysTick handler contains one off-only DCDC deadline hook. The
  fast ADC1 watchdog vector is implemented entirely in
  `dcdc_platform_stm32.c`, so it cannot be erased by regeneration.

ADC1_2 NVIC enable is deliberately owned by the DCDC platform module rather
than the `.ioc`: it is enabled only when all DCDC hardware/calibration gates
permit output, and its first action is the direct MOE shutdown. Do not also
enable the same vector in CubeMX without first changing the platform to use the
Cube-generated handler/HAL callback, or two vector definitions would conflict.

The checked configuration has been regenerated in an isolated directory with
STM32CubeMX and clean-built with STM32CubeIDE. Custom modules and protected hooks
survive; generated clock and TIM1 code are reproduced from the `.ioc`.

## Verification

Trigger host tests:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc \
  Tests/trigger_decoder_test.c Core/Src/trigger_decoder.c \
  Core/Src/trigger_recorder.c -lm -o trigger_decoder_test
./trigger_decoder_test
```

Actuator host tests:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic -DACTUATOR_HOST_TEST \
  -DINJECTION_CURRENT_CALIBRATION_VALID=1 \
  -DENGINE_INJECTOR_TEST_API_ENABLED=1 \
  -ICore/Inc -ICore/Src -ITests/actuators Tests/actuators/test_actuators.c \
  Core/Src/spark.c Core/Src/injection.c -lm -o test_actuators
./test_actuators
```

The CMake test project runs the trigger decoder, sequential actuators,
crank-only actuators, engine-control integration, and pure DCDC PWM/PID suites
with strict warnings. All suites are hardware-independent. The checked CubeIDE
Debug build also links all STM32 sources using the repository linker script.

Before loaded-engine use, perform hardware-in-the-loop sweeps for every wheel
pattern and RPM range, sensor disconnects, noisy/extra/missing edges, stopped
engine at every active-output phase, queue/overcapture injection, and measured
maximum coil/BOOST/injector pulse widths.

## Clock source

The confirmed 20 MHz HSE crystal is the PLL source: `20 MHz / 2 * 34 / 2 =
170 MHz`. Voltage scaling remains range-1 boost, flash latency remains four wait
states, and AHB/APB1/APB2 remain undivided, so TIM1 still receives 170 MHz.
Clock security is enabled after `SystemClock_Config()`. Loss of HSE enters NMI,
forces ignition, injection and DCDC outputs off, then resets the MCU. If USB or
another exact-48-MHz peripheral is added later, use HSI48/CRS rather than trying
to derive 48 MHz from the 340 MHz PLL VCO.
