# RT_STACK_V2 firmware

This firmware is organized around three intentionally separate concerns:

1. `trigger_capture_stm32.c` timestamps electrical edges.
2. `trigger_decoder.c` interprets user-defined wheel geometry.
3. `engine_control.c` gates the ignition/injection schedulers through a
   fail-safe engine state machine.

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
crank-only actuators, and the engine-control integration seam with strict
warnings. All suites are hardware-independent. The checked CubeIDE Debug build
also links all STM32 sources using the repository linker script.

Before loaded-engine use, perform hardware-in-the-loop sweeps for every wheel
pattern and RPM range, sensor disconnects, noisy/extra/missing edges, stopped
engine at every active-output phase, queue/overcapture injection, and measured
maximum coil/BOOST/injector pulse widths.

## Clock note

The firmware intentionally remains on HSI16/PLL at 170 MHz. The schematic does
not establish the fitted crystal frequency reliably enough to change the clock
source. HSE/CSS must be enabled only after the assembled hardware frequency is
confirmed and the PLL settings are updated together.
