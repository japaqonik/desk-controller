# Desk Controller

Firmware for a motorized standing desk, built around an **Arduino Nano (ATmega328P)**.
Manual up/down with soft start/stop, overcurrent protection, and (planned) a
distance-sensor-based "go to height" with EEPROM presets.

> Status: **Step 1 complete** — manual UP/DOWN with ramping and current logging.
> Closed-loop positioning and presets are planned (see roadmap below).

## Hardware

| Part | Role |
|---|---|
| Arduino Nano (ATmega328P) | controller |
| IBT-2 (BTS7960) | H-bridge motor driver |
| 12 V / 8.33 A PSU | motor supply |
| VL53L0X (I2C) | height sensor (distance to floor) |
| 4× momentary switch | UP / DOWN / P1 / FN |

Pin map and wiring details live in [`src/config.h`](src/config.h) and the
[implementation plan](desk-controller-plan.md).

Key design choices:
- **20 kHz PWM** on Timer1 (inaudible, within the BTS7960 spec), so `millis()` is untouched.
- No `delay()`, no `String`, no heap — fixed **5 ms control tick**, watchdog enabled.
- Direction, speed limits, ramp rates and current thresholds are all `constexpr` in `config.h`.

## Build & flash (PlatformIO)

The project ships several environments so each bring-up step can be flashed
independently. Pick one in the VSCode PlatformIO bar, or from the CLI:

```bash
pio run -e motor_test -t upload     # Step 1: manual motor control
pio device monitor -e motor_test    # 115200 baud
```

| Environment | Purpose |
|---|---|
| `buttons_test` | Step 0: button diagnostics (Serial only, no motor) |
| `i2c_scan` | I2C scanner + VL53L0X model-ID check |
| `motor_test` | Step 1: manual UP/DOWN with ramp + current logging |

> Nano clones with the CH340 chip often use the **old bootloader (57600 baud)** —
> hence `board = nanoatmega328`. Switch to `nanoatmega328new` if your board uses
> the new bootloader.

## Roadmap

- [x] Step 0 — button identification and debounce
- [x] Step 1 — manual drive: soft start/stop, state machine, current logging
- [ ] Current-sense calibration (RC filter on IS lines + ammeter)
- [ ] Phase 2 — height sensor: reading, filtering, plausibility, soft limits
- [ ] Phase 3 — closed-loop "go to height"
- [ ] Phase 4 — EEPROM presets (#1–#4) with an FN key
- [ ] Phase 5 — calibration combo, polish

See [`desk-controller-plan.md`](desk-controller-plan.md) for the full design.

## Safety

This drives a motor from a 12 V / 8+ A supply. Common ground between the Nano,
IBT-2 and PSU is mandatory. During bring-up, keep the motor free-running, keep a
hand near the power switch, and only raise speed/current limits after verifying
behavior. The current limit also serves as backup end-stop detection.
