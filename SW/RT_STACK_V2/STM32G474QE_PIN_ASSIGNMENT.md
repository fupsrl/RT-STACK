# STM32G474QE pin assignment for the RT-STACK redesign

Status: proposed LQFP128 board contract, checked against the STM32CubeMX 6.17 device database for `STM32G474Q(B-C-E)Tx`. It intentionally does not change the current STM32G474VET6 `.ioc`; the present firmware still targets the old LQFP100 board.

The machine-readable, package-pin-by-package-pin assignment is in [STM32G474QE_PIN_ASSIGNMENT.csv](STM32G474QE_PIN_ASSIGNMENT.csv). The 24 serialized outputs have their own contract in [STM32G474QE_SERIAL_OUTPUT_ASSIGNMENT.csv](STM32G474QE_SERIAL_OUTPUT_ASSIGNMENT.csv).

## Design assumptions

This assignment keeps all functions found in the current firmware and the draft `mcu.kicad_sch`:

- nine crank/cam/spare timing inputs;
- 12 ignition commands;
- six injector-pair select, BOOST, current-sense and comparator paths;
- 12 conditioned 12 V inputs and 12 protected 12 V outputs;
- two interleaved DCDC PWM outputs, VBUS feedback and a hardware-limit input;
- two CJ125 wideband-lambda interfaces with independent ADC and heater-control paths;
- three FDCAN buses, four serial links and two dedicated SPI buses;
- a 24-bit serialized low-speed control bank, including nine programmable trigger pull-ups;
- two status LEDs, HSE, SWD, reset and boot control.

It also preserves the existing external injector current-limit gates, so `COMP1_OUT` through `COMP6_OUT` still consume physical pins. Removing those six pads is possible only if the external gating circuit is redesigned around internal timer/HRTIM break paths.

## Serialized low-speed control and programmable pull-ups

The timing sheet already contains U62, an 8-bit `SN74LV8T595`-style part, although its outputs are currently unconnected. Nine trigger pull-ups alone exceed that capacity. Reuse U62 as the first stage and make the new chain **24 bits: U62 plus two additional 8-bit stages**. A single additional stage would provide only 16 bits and is too tight once safe output-enable, direct CJ125 chip selects, CJ125 reset and preservation of PA0/USB/BOOT0 are included.

SPI1 is dedicated to this output chain:

| Signal | MCU pin | Function |
|---|---:|---|
| `SERIAL_CTRL_SCK` | PG2 / pin 84 | SPI1_SCK, AF5 |
| `SERIAL_CTRL_MOSI` | PG4 / pin 86 | SPI1_MOSI, AF5 |
| `SERIAL_CTRL_LATCH` | PG5 / pin 103 | Storage-register clock |
| `SERIAL_CTRL_OE_N` | PF2 / pin 26 | Asynchronous bank disable, external pull-up |

Cascade each stage's serial output into the next stage and latch all 24 bits atomically:

| Bit | Logical output | Reset-safe value |
|---:|---|---:|
| 0 | `CRANK_PULLUP_EN` | 0 |
| 1 | `CAM1_PULLUP_EN` | 0 |
| 2 | `CAM2_PULLUP_EN` | 0 |
| 3 | `CAM3_PULLUP_EN` | 0 |
| 4 | `CAM4_PULLUP_EN` | 0 |
| 5 | `SPARE1_PULLUP_EN` | 0 |
| 6 | `SPARE2_PULLUP_EN` | 0 |
| 7 | `SPARE3_PULLUP_EN` | 0 |
| 8 | `SPARE4_PULLUP_EN` | 0 |
| 9 | `GPIO12V_OUT1` | 0 |
| 10 | `GPIO12V_OUT2` | 0 |
| 11 | `GPIO12V_OUT3` | 0 |
| 12 | `GPIO12V_OUT4` | 0 |
| 13 | `GPIO12V_OUT8` | 0 |
| 14 | `GPIO12V_OUT9` | 0 |
| 15 | `GPIO12V_OUT11` | 0 |
| 16 | `GPIO12V_OUT12` | 0 |
| 17 | `LED2` | 0 |
| 18 | `CJ125_RESET_RELEASE` | 0 |
| 19–23 | Reserved | 0 |

Only slow, non-timing protected outputs move to this bank. `IGNx`, injector select, BOOST, DCDC and both lambda-heater PWM signals remain direct MCU outputs. The serialized `GPIO12V_OUT` channels are not suitable for edge-scheduled injection/ignition or independent high-rate PWM.

The pull-up bits must control a high-side switch or analog switch that inserts each raw input's +5 V pull-up resistor. Do not connect a shift-register output directly to a trigger signal: driving a logic zero would clamp the sensor input rather than disconnect its pull-up. Keep the comparator-output-side +3.3 V pull-ups fixed so every MCU capture input always has a defined logic level.

`/OE` must not remain tied low as it is in the current draft. Give PF2 an external pull-up and arrange `/SRCLR` and the external load controls so reset means disabled. Firmware must hold PF2 high, shift the complete all-zero safe image, pulse `SERIAL_CTRL_LATCH`, and only then drive PF2 low. External pull-downs must keep every load off while the register outputs are high impedance; the level-shifted CJ125 reset must remain asserted. A watchdog or MCU reset must immediately return `/OE` high.

Maintain one 24-bit shadow word and update it under one owner. Every transaction transmits all 24 bits before one latch pulse, so unrelated outputs cannot glitch. Treat trigger electrical-mode changes as stopped-engine configuration: disable capture processing, update and latch the pull-up mask, clear pending/overcapture state, then reacquire synchronization.

Expose this as a named per-channel setting such as `trigger[i].pullup = DISABLED | ENABLED`; never expose serialized bit numbers to the user. Validate the requested electrical mode alongside polarity, capture edge and digital-filter settings before accepting a wheel configuration.

## The nine trigger inputs

The correct peripheral mode is **timer input capture**, not input compare. The G474 has only two 32-bit general-purpose timers, TIM2 and TIM5, with four capture channels each. Therefore eight 32-bit captures is the physical maximum; a ninth independent 32-bit capture does not exist on this MCU.

| Logical input | Package pin | Port | Capture channel | AF | Counter |
|---|---:|---|---|---:|---:|
| CRANK_TRG | 113 | PD3 | TIM2_CH1 | 2 | 32 bit |
| CAM1_TRG | 114 | PD4 | TIM2_CH2 | 2 | 32 bit |
| CAM2_TRG | 117 | PD7 | TIM2_CH3 | 2 | 32 bit |
| CAM3_TRG | 116 | PD6 | TIM2_CH4 | 2 | 32 bit |
| CAM4_TRG | 97 | PF6 | TIM5_CH1 | 6 | 32 bit |
| SPARE1_TRG | 15 | PF7 | TIM5_CH2 | 6 | 32 bit |
| SPARE2_TRG | 16 | PF8 | TIM5_CH3 | 6 | 32 bit |
| SPARE3_TRG | 17 | PF9 | TIM5_CH4 | 6 | 32 bit |
| SPARE4_TRG | 1 | PE2 | TIM3_CH1 | 2 | 16 bit + software epoch |

This mapping deliberately spends the two QUADSPI pin banks and some FMC alternatives. It avoids PA0–PA3, the fast ADC5 pins and almost all HRTIM pins. That is the right trade unless external parallel/QSPI memory is a firm requirement.

At a suggested 10 MHz common timer tick, TIM2/TIM5 wrap after about 429.5 s and TIM3 wraps every 6.5536 ms. Maintain a TIM3 overflow epoch and form a wider timestamp before passing an event to the decoder. The clock rate, filter and polarity belong in per-channel configuration; do not bake them into the ISR.

TIM2, TIM5 and TIM3 must share a timestamp domain. Configure the same timer-kernel clock and prescaler, synchronize their initial reset/start through the timer master/slave ITR network where the final CubeMX configuration permits it, and verify the measured counter offsets at startup. Do not compare raw CCR values from independently started counters.

### Is capture actually better than EXTI plus an ISR?

Yes. The current EXTI path timestamps an edge only after interrupt entry by reading `TIM2->CNT`, so interrupt latency and pre-emption become timestamp error. Input capture latches the counter in `CCR1..CCR4` at the hardware edge. ISR latency then affects only how quickly the sample is drained, not its time.

It also gives:

- per-channel edge polarity and digital filtering;
- an overcapture flag when software fails to drain a previous edge;
- DMA as an option for the highest-rate channels;
- a clean ISR that copies `{CCR, edge, status}` into a queue while decoding remains outside interrupt context.

The limitation is that each channel still has one CCR latch. A second edge before the first is consumed causes overcapture, so use DMA or a sufficiently short interrupt path for the crank input.

## Timed outputs

The remaining timer-rich pins are assigned to functions that benefit from deterministic edges:

| Function group | Pins/peripheral | Result |
|---|---|---|
| IGN1–4 | PF12–PF15 / TIM20_CH1–4 | Four hardware output-compare channels |
| IGN5–8 | PE9, PE11, PE13, PE14 / TIM1_CH1–4 | Four hardware output-compare channels |
| IGN9–10 | PE3, PE4 / TIM3_CH2–3 | Shares the 10 MHz time base with SPARE4 capture |
| IGN11 | PB15 / HRTIM1_CHD2 | HRTIM set/reset output |
| IGN12 | PA6 / TIM16_CH1 | Hardware output-compare channel |
| BOOST12–BOOST1112 | PA8, PA9, PA10, PB12, PB13, PB14 / HRTIM A1, A2, B1, C1, C2, D1 | Six independently controlled high-resolution outputs |
| DCDC phases | PC8, PC9 / HRTIM E1, E2 | One HRTIM timing unit, two interleaved outputs |
| Lambda heater 1 | PF10 / TIM15_CH2 | Independent hardware PWM, AF3 |
| Lambda heater 2 | PB7 / TIM4_CH2 | Independent hardware PWM, AF2 |

Every actuator command is active high. Add a physical pull-down at each gate/driver input and initialize timer output state and GPIO output-data registers before switching a pad to output or alternate-function mode.

The mixed timer instances should be hidden behind a table-driven channel descriptor. User-facing configuration should identify `IGN1`, `BOOST34`, and so on; it should never require the user to know that one channel is on TIM20 and another is on HRTIM D.

## DCDC implementation

Use PC8/HRTIM1_CHE1 and PC9/HRTIM1_CHE2. For period `T` and duty `D`:

- phase A set event: `0`; reset event: `D*T`;
- phase B set event: `T/2`; reset event: `(T/2 + D*T) mod T`.

Write all compare values through HRTIM preload registers and transfer them on the same period event. Trigger the VBUS ADC conversion from HRTIM at a repeatable quiet point in the switching cycle, feed that sample to the PID, clamp the requested duty and rate-limit it before the next preload transfer.

PD14/COMP7_INP is reserved for an independent DCDC overcurrent/overvoltage signal. Route COMP7 internally to HRTIM external event 5 or 10 for a cycle-by-cycle output reset. Use PB11/HRTIM1_FLT4 for the separately latched gate-driver or analog fault that requires an explicit clear. PB10 is a resistor-selectable second external fault option on HRTIM1_FLT3.

The DCDC must start disabled and stay disabled until clocks, ADC calibration, VBUS plausibility and fault inputs are valid. PID software is not a substitute for cycle-by-cycle hardware shutdown.

## Dual CJ125 lambda interfaces

Both CJ125s share SPI4 but retain direct, separately pulled-up chip selects:

| CJ125 signal | Channel 1 | Channel 2 | Notes |
|---|---|---|---|
| SPI clock | PE12 / SPI4_SCK | Shared | AF5 |
| SPI data to CJ125 | PE6 / SPI4_MOSI | Shared | AF5 |
| SPI data from CJ125 | PE5 / SPI4_MISO | Shared | AF5; only the selected device may drive it |
| `/SS` | PG0 | PG1 | Direct GPIO; high/inactive through reset |
| `UA` | PC0 / ADC1_IN6 | PA4 / ADC2_IN17 | Lambda/pump-current signal |
| `UR` | PE8 / ADC3_IN6 | PE10 / ADC4_IN14 | Ri/temperature signal |
| Heater PWM | PF10 / TIM15_CH2 | PB7 / TIM4_CH2 | Direct hardware PWM; never serialized |
| `/RST` | `SERIAL_Q18` | Shared | Active low through a proper 5 V-domain reset/level-shift stage |

ADC1/ADC2 can sample the two `UA` channels simultaneously, while ADC3/ADC4 can do the same for both `UR` channels. This keeps PA0 available for future fast analog work. Bosch specifies 16-bit SPI frames at up to 2 Mbaud. Keep the CJ125 bus on SPI4 and the output registers on SPI1; this avoids changing SPI mode or disturbing the serialized control image during lambda traffic.

CJ125 is a 5 V device. Use explicit unidirectional 3.3 V-to-5 V translation for SCK, MOSI, both `/SS` signals and reset, and a 5 V-to-3.3 V path for MISO. Pull both `/SS` inputs high on the CJ125 side. Do not assume that a nominally 5 V-tolerant STM32 input solves the output-high threshold in the opposite direction.

The existing `uego.kicad_sch` divides each `UA` and `UR` output with 600 ohm over 1 kohm. Its 0.625 ratio is appropriate for mapping approximately 5 V into the 3.3 V ADC range, but its roughly 1.6 kohm total resistance loads the CJ125 output by milliamps. Bosch specifies the `UA` output swing under a load below 10 microamps. Replace that network with a high-impedance divider followed by an ADC buffer, or buffer first and then attenuate; retain anti-alias filtering and ADC clamp protection. Route the scaled signals listed above, never raw 5 V-domain `UA`/`UR`.

Each heater needs a protected low-side power stage, hard gate-off bias and independent duty control. Connect CJ125 `DIAHG` to its heater gate and `DIAHD` to its drain through the Bosch application resistor so SPI diagnostics can detect open load and shorts. The LSU 4.9 specifies heater PWM at 100 Hz or above and about 7.5 W steady-state power. Heater control must include battery-voltage feed-forward, a condensation/dew-point delay, a controlled warm-up ramp, and closed-loop correction from `UR`; maximum duty must never be applied immediately to a cold sensor.

The existing CJ125 sheet is a useful starting point, not layout-ready. Its SCK and `/SS` symbol directions are wrong for ERC, the heater-driver connectivity needs review, and the library footprint is `SO24W` while Bosch's current public product page lists LQFP32. Freeze the exact CJ125 ordering code and validate its package/pinout before routing.

## Analog and injector current paths

| Pair | Comparator/ADC input | Physical comparator output | Internal threshold |
|---|---|---|---|
| INJ1/2 | PA1 / COMP1_INP / ADC1_IN2 | PF4 / COMP1_OUT | DAC1_CH1 |
| INJ3/4 | PA7 / COMP2_INP / ADC2_IN4 | PB9 / COMP2_OUT | DAC1_CH2 |
| INJ5/6 | PC1 / COMP3_INP / ADC1_IN7 | PC2 / COMP3_OUT | DAC3_CH1 |
| INJ7/8 | PE7 / COMP4_INP / ADC3_IN4 | PB1 / COMP4_OUT | DAC3_CH2 |
| INJ9/10 | PD12 / COMP5_INP / ADC3_IN9 | PC7 / COMP5_OUT | DAC4_CH1 |
| INJ11/12 | PD11 / COMP6_INP / ADC3_IN8 | PC6 / COMP6_OUT | DAC4_CH2 |

The DAC thresholds are internal signals; do not spend DAC output pads on them. Moving COMP4–6 inputs away from PB0/PB11/PB13 preserves more useful HRTIM and analog combinations.

VBUS remains on PC3/ADC1_IN9, VBATT on PB2/ADC2_IN12 and injector temperature on PC4/ADC2_IN5. The new board must remove the LEDs currently connected across the VBUS and VBATT divider lower resistors. Those LEDs clamp and linearize neither measurement; they make both ADC readings voltage- and temperature-dependent. `LED1` remains a direct output on PD5; `LED2` moves to serialized output bit 17. Both require their own series resistors.

## Communications and debug

- FDCAN1: PD0 RX, PD1 TX.
- FDCAN2: PB5 RX, PB6 TX.
- FDCAN3: PB3 RX, PB4 TX.
- UART4: PC10 TX, PC11 RX.
- UART5: PC12 TX, PD2 RX.
- USART1: PE0 TX, PE1 RX.
- USART2: PA2 TX, PA3 RX.
- Dedicated output-control SPI1: PG2 SCK and PG4 MOSI, with PG5 as the 24-bit latch clock and PF2 as active-low output enable. PG3 remains assigned to `GPIO12VIN_7`; this write-only chain needs no MISO.
- Dedicated CJ125 SPI4: PE12 SCK, PE5 MISO and PE6 MOSI, with direct chip selects on PG0 and PG1. Name these nets `CJ125_SPI_*`; do not retain the misleading old `SPI3_*` names.

On every CAN transceiver, MCU TX goes to transceiver TXD pin 1, while transceiver RXD pin 4 goes to MCU RX. This corrects the direction error in the current CAN wiring.

Using PB3/PB4 for the third CAN bus means SWO and full JTAG are unavailable; SWD remains on PA13/PA14. If only two CAN buses are required, leave PB3/PB4 free and restore SWO.

The 20 MHz HSE crystal belongs on PF0/OSC_IN and PF1/OSC_OUT. PE0/PE1 are ordinary GPIO/USART pins and must not be connected to that crystal. Route PG10/NRST, PA13/SWDIO, PA14/SWCLK, 3.3 V sense and ground to the debug connector.

## Pins deliberately preserved

| Pin | Reason |
|---|---|
| PA0 | Fast ADC1/2, comparator and OPAMP-capable analog expansion |
| PA11/PA12 | The unique USB DM/DP pair |
| PB8/BOOT0 | Controlled boot strap plus HRTIM external-event capability |
| PA13/PA14 | SWD |
| PF0/PF1 | Confirmed HSE |
| PG10 | Reset |

PC14/PC15 are used as slow general inputs, so this assignment does not retain an LSE crystal. If RTC/LSE is required, move at least two slow `GPIO12VIN` signals to an external input register or expander.

## Board-level constraints before layout

1. Connect every VDD/VSS pair. Pins 43 and 44 are both VREF+ and both must be connected and decoupled; pin 45 is VDDA and pin 42 is VSSA.
2. Keep the 20 MHz crystal and its load network at PF0/PF1, with short symmetric traces and no connector stubs.
3. Put defined external pull-downs on IGN, BOOST, injector-select, DCDC and protected-output control inputs.
4. Route CRANK first, then CAM inputs, away from HRTIM/DCDC switching nodes. The MCU-side traces begin after the existing comparator and 3.3 V pull-up.
5. Add a real hardware DCDC fault path. Keep it independent of the PID and normal interrupt system.
6. Do not assume an analog-enabled pin is 5 V tolerant. Scale and clamp every external analog signal to the VDDA/VREF range.
7. Use an output-register family whose input thresholds are guaranteed at a 3.3 V MCU drive level and whose temperature/qualification match the product. Do not substitute a 5 V `74HC595` whose VIH is not guaranteed by a 3.3 V output.
8. Keep the eight serialized protected outputs explicitly classified as slow/non-safety-critical. If they need PWM, immediate fault reaction or independent availability, replace this chain with a qualified smart high-side/low-side SPI driver with diagnostics.
9. Validate both CJ125 5 V logic translation and the high-impedance `UA`/`UR` analog front ends before layout. Keep these traces away from DCDC and ignition edges.
10. The currently listed `STM32G474QET6` order code is industrial grade, not an automotive-qualified MCU. Confirm the required ambient/junction range and qualification before freezing an engine-bay production BOM.

## CubeMX and regeneration

The current `RT_STACK_V2.ioc` still describes the STM32G474VET6/LQFP100 production prototype. Re-targeting that file in place would make the existing board unbuildable and would mix two hardware variants.

Create a separate `RT_STACK_G474QE.ioc` only after this pin contract is accepted. Generated startup, linker and HAL initialization files should then be produced from that QE `.ioc`; application modules and the board/channel descriptor remain outside generated code. No hand edits should be required in generated IRQ, MSP or `main.c` sections except inside CubeMX `USER CODE` blocks.

## Primary references

- [STM32G474QE product page](https://www.st.com/en/microcontrollers-microprocessors/stm32g474qe.html)
- [STM32G474xB/C/E datasheet, DS12288](https://www.st.com/resource/en/datasheet/stm32g474qe.pdf)
- [STM32G4 reference manual, RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [Bosch CJ125 product page](https://www.bosch-semiconductors.com/products/automotive-ics/ice-powertrain/cj125/)
- [Bosch CJ125 product information](https://www.bosch-semiconductors.com/media/automotive_systems_ics/pdf_1/ic_engine_management/cj125_product_info_2019-02.pdf)
- [Bosch LSU 4.9 datasheet](https://www.bosch-motorsport.com/content/downloads/Raceparts/Resources/pdf/Data%20Sheet_69034379_Lambda_Sensor_LSU_4.9.pdf)
