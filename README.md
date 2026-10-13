# Desk Controller

Firmware for a motorized standing desk, built around an **Arduino Nano (ATmega328P)**.
Current build is a **minimum viable version**: manual up/down with soft
start/stop and INA219-based overcurrent protection. Position sensing and presets
are deferred (see the plan).

> Status: **manual mode complete** — hold-to-move up/down, soft ramp, overcurrent
> protection via INA219. No position sensing / presets yet (deferred; a distance
> sensor proved too short-range on the real floor — see the plan appendix for
> alternatives).

## Hardware

| Part | Role |
|---|---|
| Arduino Nano (ATmega328P) | controller |
| IBT-2 (BTS7960) | H-bridge motor driver |
| 12 V / 8.33 A PSU | motor supply |
| INA219 (I2C) | current sensing (overcurrent protection) |
| 4× momentary switch | UP / DOWN / P1 / FN (P1/FN unused for now) |

Pin map and wiring details live in [`src/config.h`](src/config.h) and the
[implementation plan](desk-controller-plan.md).

Key design choices:
- **20 kHz PWM** on Timer1 (inaudible, within the BTS7960 spec), so `millis()` is untouched.
- No `delay()`, no `String`, no heap — fixed **5 ms control tick**, watchdog enabled.
- Current measured digitally over I2C (INA219) — clean even at low current, no PWM
  noise on the ADC (the IBT-2 IS line was too noisy).
- Direction, speed, ramp rates and current thresholds are all `constexpr` in `config.h`.

## Build & flash (PlatformIO)

The final firmware is the `main` environment (also the default). The others are
bring-up / diagnostic tools. Pick one in the VSCode PlatformIO bar, or from the CLI:

```bash
pio run -t upload                 # builds the default env (main)
pio device monitor                # 115200 baud
pio run -e motor_test -t upload   # the slower test tool with verbose logging
```

| Environment | Purpose |
|---|---|
| `main` | **Final firmware** — manual mode, 100% speed, no timeout, Serial errors only |
| `motor_test` | Test tool — 60% speed, run-time timeout, verbose per-tick Serial log |
| `buttons_test` | Button diagnostics (Serial only, no motor) |
| `i2c_scan` | I2C scanner + device ID check (INA219 / sensors) |
| `ina_test` | INA219 verification and current logging |

> Nano clones with the CH340 chip often use the **old bootloader (57600 baud)** —
> hence `board = nanoatmega328`. Switch to `nanoatmega328new` if your board uses
> the new bootloader. The `upload_port` is pinned to `/dev/ttyUSB0`; edit if yours differs.

## Controls

- **UP (D2)** / **DOWN (D3)** held → move; both together → stop.
- Soft start/stop ramp on every move.
- Overcurrent (sustained or instantaneous) → FAULT: motor cut, LED blinks,
  clears after all buttons are released.

## Roadmap

- [x] Button identification and debounce
- [x] Manual drive: soft start/stop, state machine
- [x] Current sensing via INA219 + overcurrent protection
- [ ] Position sensing (encoder / distance / etc. — see plan appendix, section 12)
- [ ] Closed-loop "go to position"
- [ ] EEPROM presets (#1–#4) with an FN key

See [`desk-controller-plan.md`](desk-controller-plan.md) for the full design and
the discussion of position-sensing options.

## Safety

This drives a motor from a 12 V / 8+ A supply. Common ground between the Nano,
IBT-2 and PSU is mandatory; an inline fuse on +12 V is recommended. Keep a hand
near the power switch during testing.

Note: with the high-ratio gearbox the current barely rises under load, so the
overcurrent limit reliably catches a hard overload but may **not** catch a gentle
drive into an end stop. Until end-stop handling is based on position/limit
switches, do not rely on the current limit alone to stop at the extremes.
