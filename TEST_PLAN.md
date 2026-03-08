# Firmware Test Plan — Next Machine Visit

This plan covers flashing the updated firmware (janm012012 base + encoder
module) onto the CHM-T48VB for the first time.

## Before Flashing

- [ ] Back up the current working firmware binary from the MCU
  ```bash
  st-flash read firmware-backup-$(date +%Y%m%d).bin 0x08000000 0x80000
  ```
- [ ] Build the new firmware from `c-riegel/Smoothieware-CHMT` `chmt` branch
- [ ] Apply post-build binary fixup (the claude instance on the Ubuntu machine
      knows this procedure — have it document the steps to README.md)
- [ ] Keep the backup binary and the StarTech ICUSB422IS adapter ready for
      rollback

## Phase 1: Basic Communication (janm's serial changes on 48VB)

This is the highest-risk step. janm's firmware has DMA serial changes that have
NOT been tested on the CHM-T48VB with RS422. If serial doesn't work, nothing
else matters.

- [ ] Flash the new firmware
- [ ] Connect via USB serial (StarTech ICUSB422IS) at 115200 baud
- [ ] Send `M115` — should return firmware identification string
  - If no response: try different baud rates (janm may default differently)
  - If still no response: **rollback to backup firmware**
- [ ] Send `G28` — verify homing works
- [ ] Send `G0 X50 Y50 F1000` — verify basic movement
- [ ] Send `M114` — verify position report

**STOP HERE if any of the above fail.** Rollback and investigate before
continuing.

## Phase 1.5: Baud Rate Increase

After basic serial is confirmed working at 115200, test higher baud rates.
janm's README states the CHM-T48VB RS422 transceiver supports up to 460800 baud.

- [ ] Change `uart0.baud_rate` in `config.default` to `460800`
- [ ] Rebuild and flash firmware
- [ ] Change OpenPnP `machine.xml` serial baud to `460800`
- [ ] Send `M115` — verify response
- [ ] Run `G28` — verify homing works
- [ ] Send several rapid G0 moves — verify no communication errors or garbled responses
- [ ] Run a short placement job — verify reliability under sustained communication

**If 460800 fails:** fall back to 115200 and continue. Higher baud is a throughput
optimization, not a requirement. It becomes more important when streaming S-curve
segments (Phase 3).

## Phase 2: Encoder Module Verification

- [ ] Send `M918` — should return `ok EX:0 EY:0` (or whatever count)
- [ ] Manually jog X axis: `G0 X10 F500`, then `M918`
  - Verify EX count changed
  - Note the count for 10mm of travel (this gives us counts per mm)
- [ ] Jog X back: `G0 X0 F500`, then `M918`
  - Verify EX count returned to approximately 0
  - If count went the wrong direction (larger instead of back toward 0),
    note this — we need to swap IC polarity
- [ ] Repeat for Y axis: `G0 Y10 F500`, `M918`, `G0 Y0 F500`, `M918`
- [ ] Record encoder resolution:
  - X: ______ counts per 10mm → ______ counts/mm
  - Y: ______ counts per 10mm → ______ counts/mm
- [ ] Auto-calibrate: `M924` (homes, moves toward origin, computes signed
      counts/mm for both axes automatically)
  - Verify reported values match manual measurement
  - Check with `M923` (no args) to confirm values were stored
- [ ] If manual calibration preferred: `M923 X___ Y___` with measured values

## Phase 3: Encoder vs Stepper Comparison

- [ ] Home the machine: `G28`, `G92 X0 Y0`, `M919 X0 Y0`
- [ ] Send `M918` and `M921` — both should show zero/near-zero
- [ ] Move at slow speed: `G0 X100 Y100 F1000`
- [ ] Compare `M918` vs `M921` — at slow speed, should be very close
- [ ] Move at medium speed: `G0 X0 Y0 F5000`
- [ ] Compare `M918` vs `M921` — may start to see small differences
- [ ] Move at high speed: `G0 X100 Y100 F20000`
- [ ] Compare `M918` vs `M921` — differences here = lost steps
- [ ] Repeat high speed moves several times, recording the divergence
  - This tells us how many steps we lose at speed and validates the
    entire reason for encoder feedback

## Phase 4: Encoder-Driven Position Control

After calibration (M924 or M923), encoder-driven control is active for G0/G1.

- [ ] Home: `G28`, then `M919 X0 Y0`
- [ ] Send `G0 X50 F1000` — motor should stop when encoder reaches target, not
      when stepper step count is exhausted
- [ ] Check `M918` — encoder count should match `50 * counts_per_mm`
- [ ] Repeat for Y: `G0 Y50 F1000`, check `M918`
- [ ] Run several moves at increasing speeds, verify encoder always lands on
      target even if stepper loses steps

## Phase 5: Move Timeout Safety

- [ ] With encoder-driven control active, physically block the X axis (hold the
      gantry) during a move
- [ ] The firmware should halt within a few seconds and report
      `error: encoder X move timeout`
- [ ] Recover with `M999`, verify machine responds normally
- [ ] Repeat for Y axis

## Phase 6: Segment Buffering (M920)

Requires encoder-driven control to be working (Phases 3-4 passed).

- [ ] Home, calibrate: `G28`, `M919 X0 Y0`, `M924`
- [ ] Send a simple 3-segment buffered move:
  ```
  M920 S3
  G0 X10 Y10 F500
  G0 X30 Y30 F2000
  G0 X50 Y50 F500
  ```
- [ ] Verify machine moves smoothly (accel → cruise → decel)
- [ ] Check final position: `M918` should match X50 Y50 encoder targets
- [ ] Send `M920 S2` with only 1 G0 — verify firmware waits for the second
- [ ] Send the second G0 — verify execution starts
- [ ] Test error cases:
  - `M920 S0` — should return error
  - `M920 S5` while another M920 is active — should return error
  - `M920 S129` — should return error (max 128)

## Phase 7: M-Code Verification

- [ ] `M919 X0 Y0` — set encoder counts, verify with `M918`
- [ ] `M922 X0 Y0` — set stepper counts (machine must be idle), verify with
      `M921`
- [ ] `M922 X0 Y0` while moving — should return error
- [ ] `M923 X100 Y100` — set counts per mm, verify it echoes back correctly
- [ ] `M924` — auto-calibrate, verify output and stored values

## Rollback Procedure

If anything goes wrong:
```bash
st-flash write firmware-backup-YYYYMMDD.bin 0x08000000
```
The machine should return to its previous working state.

## Results

Fill in after testing:

| Measurement | X Axis | Y Axis |
|-------------|--------|--------|
| Encoder counts per 10mm | | |
| Encoder counts per mm | | |
| Encoder direction correct? | | |
| Steps lost at 1000 mm/min | | |
| Steps lost at 5000 mm/min | | |
| Steps lost at 20000 mm/min | | |
