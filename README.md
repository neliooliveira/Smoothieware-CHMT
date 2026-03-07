# Smoothieware-CHMT

Smoothieware firmware for the Charmhigh CHM-T36VA, CHM-T48VB, and equivalent pick-and-place machines. Runs on the stock STM32F407 control board.

Originally ported to STM32 by [mattthebaker](https://github.com/mattthebaker/Smoothieware-CHMT), then extended for OpenPnP by [c-riegel](https://github.com/c-riegel/Smoothieware-CHMT) (advanced motion, feed rate tuning, 7-axis peeler support), and further developed by [janm012012](https://github.com/janm012012/Smoothieware-CHMT) (DMA serial, drag pin PWM, smart sensing, camera lighting). Encoder feedback is being added by c-riegel.

Precompiled firmware is available in the `STM32F407xG` folder. To flash the mainboard, a full chip erase is required. While flashing, the vacuum pump and blower may run at full power due to the MCU outputs being in an undefined state. No official schematics are available for these machines, and this firmware is provided as-is with no warranty. **Flash at your own risk. Always back up your existing firmware before flashing.**

### Attention

This firmware has not yet been tested on the CHM-T48VB (RS422 serial). If you have test results, please report them. The last known good commit is [f306fb](https://github.com/janm012012/Smoothieware-CHMT/tree/f306fb6256647447e799c124dffefa7ebee5d7d8).

## Building

Follow the standard Smoothieware build process. From the project root:

```bash
make clean
make all
```

The build targets STM32F407xG. Config is compiled into the binary via `src/config.default`.

To flash via SWD:

```bash
st-flash write STM32F407xG/main.bin 0x08000000
```

To back up existing firmware before flashing:

```bash
st-flash read firmware-backup-$(date +%Y%m%d).bin 0x08000000 0x80000
```

Note: MRI (gdb over serial) is not supported on STM32. Use SWD/JTAG for debugging.

## Serial Configuration

Both the CHM-T36VA and CHM-T48VB share the same control board with minor differences:

| Machine | Interface | Max Baud (stock) | Transceiver |
|---------|-----------|------------------|-------------|
| CHM-T36VA | RS232 | 115200 | U32 (RS232 level shifter) |
| CHM-T48VB | RS422 | 460800 | RS422 driver |

### `rts_cts_handshake` setting

| Value | Description |
|-------|-------------|
| `0` | Default: no hardware flow control. Both USART1 (RS232) and USART2 (RS422) active. Use this for stock boards. |
| `1` | Vespamans modified 48VB: USART1 with RTS/CTS on PA_12/PA_11. |
| `2` | janm's 36VA: USART1 with RTS via PD_5 (TX on second RS232 connector). |

For stock boards, set `rts_cts_handshake` to `0` and `uart0.baud_rate` to `115200`. The baud rate applies to both serial ports.

Serial uses DMA for both TX and RX, with a 512-byte circular RX buffer. The RS422 transceiver is enabled at startup via PC_1.

### Hardware Flow Control Modifications

To achieve higher throughput (up to 4Mbit) with hardware RTS/CTS flow control, board modifications are required. Both machine variants share the same general approach:

#### CHM-T48VB

* Add a 1+1 channel isolator chip to unpopulated position U28 (e.g., ADUM121N0BRZ-RL7). Place kapton tape on pads 2 & 3 — these connect to RTS/CTS wires, not board pads.
* Pull two wires from unpopulated U18 pin 4 & 1 (SO8, RS485 transceiver) to pin 2 & 3 of the new isolator chip.
* Bridge the RX/TX/RTS/CTS path at U32 (RS232 level shifter position) with wires for direct logic-level signaling.
* Add a 4-pole through-hole JST connector (or Yeonho SMW250/SMH250).
* Move the 0R resistor from R132 to R131 to route RX from the RS232 input.
* Optionally remove ESD protection components near connectors (required for multi-Mbit speeds).

#### CHM-T36VA

* Disconnect pin 2 & 3 on U28 from board pads.
* Pull two wires from unpopulated U18 pin 4 & 1 to pin 2 & 3 of U28.
* Remove U32 and connect 4 wires in its place.
* Optionally remove ESD protection components.

#### USB-Serial Adapter

Keep the adapter as close to the control board as possible. Adapters based on the XR21B1420 chipset have the lowest latency (tested on Linux). The adapter must supply the isolator voltage (e.g., 3.3V).

With RTS/CTS enabled, set OpenPnP to RTS/CTS flow control and disable "Confirmation Flow Control."

## Machine Commands

### Standard G-codes

| Code | Description |
|------|-------------|
| `G0` / `G1` | Linear move with feed rate (`F`) parameter |
| `G28` | Home all axes |
| `G92` | Set position |
| `M114` | Report current position |
| `M115` | Firmware identification (used by OpenPnP auto-detection) |
| `M204 Snnn` | Set acceleration (mm/s^2) |
| `M205 Xnnn` | Set junction deviation |
| `M400` | Wait for moves to complete |
| `M999` | Reset from halt state |

### Vacuum and Pneumatics

| Code | Description |
|------|-------------|
| `M808 Snnn` | Vacuum pump PWM (16kHz, S = percent) |
| `M810 Snnn` | Blower PWM (16kHz, S = percent) |

### Drag Pin

| Code | Description |
|------|-------------|
| `M816` | Drag pin activate. Without `S` argument: firmware manages PWM automatically to prevent coil overheating. With `S nnn`: manual PWM override (you must manage current yourself). |
| `M817` | Drag pin release. Returns error and enters HALT if pin does not retract after timeout. Requires `COMMAND_ERROR_REGEX` set to `^.*(error\|!!).*` in OpenPnP. |
| `M119.1` | Read drag pin status. Returns `ok0` or `ok1`. Requires `ACTUATOR_READ_REGEX` of `^ok(?<Value>\d)`. |

Anti-Stiction Wiggle (ASW): if the drag pin gets stuck, the firmware automatically wiggles X/Y to free it. Enable by adding `switch.dragpin.dragpin true` to `config.default`. ASW status is reported in the `ok` response when engaged.

### Camera and Lighting

| Code | Description |
|------|-------------|
| `M820` | Buzzer on |
| `M821` | Buzzer off |
| `M822 Snnn` | Down-looking camera lighting PWM (16kHz, S = percent, via OT2) |

### Encoder (X/Y Axis Feedback)

The X and Y axes have quadrature encoders connected to STM32 hardware encoder timers. When enabled, the encoder module provides position feedback independent of stepper step counting.

| Pin | Function | Timer |
|-----|----------|-------|
| PA_15 | X encoder channel A | TIM2 CH1 |
| PB_3 | X encoder channel B | TIM2 CH2 |
| PA_0 | Y encoder channel A | TIM5 CH1 |
| PA_1 | Y encoder channel B | TIM5 CH2 |

TIM2 and TIM5 run in hardware encoder mode with 4x resolution (counting on both edges of both channels) and maximum input filtering for noise rejection.

#### Encoder Configuration

Add to `config.default`:

```
encoder_enable                true
encoder_x_counts_per_mm       0        # set via M923 or M924 after calibration
encoder_y_counts_per_mm       0        # set via M923 or M924 after calibration
```

When both `encoder_x_counts_per_mm` and `encoder_y_counts_per_mm` are non-zero, encoder-driven position control is active.

#### Encoder M-codes

| Code | Parameters | Description |
|------|------------|-------------|
| `M918` | | Report encoder positions. Response: `ok EX:<count> EY:<count>` |
| `M919` | `Xnnn Ynnn` | Set encoder counters (e.g., `M919 X0 Y0` to zero after homing) |
| `M921` | | Report stepper step counts. Response: `ok SX:<count> SY:<count>` |
| `M922` | `Xnnn Ynnn` | Set stepper step counts. Returns error if machine is moving. |
| `M923` | `Xnnn Ynnn` | Set encoder counts per mm. Reports current values if called without parameters. |
| `M924` | | Auto-calibrate encoder counts per mm. Moves axes slowly over a known distance, measures encoder response, calculates and sets counts per mm. Must be homed first. |

#### Encoder Calibration

After homing, run `M924` to auto-calibrate. The machine moves slowly toward the origin (front-left), measures encoder counts over the travel distance, and calculates counts per mm for both axes. Alternatively, calibrate manually:

1. Home the machine (`G28`)
2. Zero encoders: `M919 X0 Y0`
3. Move a known distance slowly: `G0 X-200 Y-200 F500`
4. Read encoder counts: `M918`
5. Calculate: counts_per_mm = |encoder_count| / 200
6. Set: `M923 X<value> Y<value>`

If the encoder counts go the wrong direction (negative when they should be positive), the encoder polarity needs to be inverted in the firmware.

### OpenPnP Integration

This firmware supports M115 for automatic detection by OpenPnP Issues & Solutions.

Recommended OpenPnP serial settings for stock boards:

| Setting | Value |
|---------|-------|
| Baud rate | 115200 |
| Flow control | XON/XOFF |
| Line ending | LF |
| `POSITION_REPORT_REGEX` | `^okC:X:(?<X>-?\d+\.\d+)Y:(?<Y>-?\d+\.\d+)Z:(?<Z>-?\d+\.\d+)A:(?<A>-?\d+\.\d+)B:(?<B>-?\d+\.\d+)C:(?<C>-?\d+\.\d)D:(?<D>-?\d+\.\d)` |
| `COMMAND_ERROR_REGEX` | `^.*(error\|!!).*` |

## Timer Allocation

| Timer | Function |
|-------|----------|
| TIM2 | X axis encoder (hardware encoder mode) |
| TIM3 | us_ticker (system microsecond timer, extended from 16-bit to 32-bit via overflow ISR) |
| TIM5 | Y axis encoder (hardware encoder mode) |
| TIM7 | Step ticker (100kHz step pulse generation) |
| TIM14 | Unstep timer (one-shot, minimum step pulse width) |

Note: `us_ticker` was moved from TIM2 to TIM3 to free TIM2 for the X axis encoder. TIM3 is 16-bit; an overflow ISR extends the counter to 32-bit with a race-guarded read.

## Port Status

All peripherals ported and tested on STM32F407:

* mbed hooks, stm32f4xx libs, timers, watchdog, GPIO, ADC, PWM, build scripts

Machine status:

* Config file: 48VB complete
* Pin map: 48VB/36VA complete
* Operation: all system functions operational
* 7-axis support (PAXIS=7), planner queue size 128

## License

Smoothieware is released under the [GNU GPL v3](http://www.gnu.org/licenses/gpl-3.0.en.html).
