# Desk Controller

Firmware for a motorized standing desk, built around an **Arduino Nano (ATmega328P)**:
manual up/down with soft start/stop, overcurrent and undervoltage protection.

> Status: **manual mode complete and in use.** Closed-loop go-to-position and
> EEPROM presets were built and tested with an HC-SR04 sonar, but the sonar's wide
> cone is fooled by objects under the desk, so that work is archived in `attic/`
> until a better position source (narrow-beam LiDAR or encoder) is chosen.

## Hardware

| Part | Role |
|---|---|
| Arduino Nano (ATmega328P) | controller |
| IBT-2 (BTS7960) | H-bridge motor driver |
| 12 V PSU (>= ~8 A) | motor supply |
| INA219 (I2C, R020 shunt) | current + bus voltage sensing (protection) |
| 4× momentary switch | UP / DOWN / P1 / FN (P1/FN unused for now) |

Pin map and wiring details live in [`src/config.h`](src/config.h) and the
[implementation plan](desk-controller-plan.md).

Key design choices:
- **20 kHz PWM** on Timer1 (inaudible, within the BTS7960 spec), so `millis()` is untouched.
- No `delay()`, no `String`, no heap — fixed **5 ms control tick**, watchdog enabled.
- Current measured digitally over I2C (INA219 with a 0.02 Ω shunt) — clean even at
  low current (the IBT-2 IS line was too noisy).
- Direction (`kUpSign`), speed, ramp rates and protection thresholds are all
  `constexpr` in `config.h`.

## Build & flash (PlatformIO)

The final firmware is the `main` environment (also the default):

```bash
pio run -t upload                 # builds and flashes main
pio device monitor                # 115200 baud
pio run -e motor_test -t upload   # verbose test tool
```

| Environment | Purpose |
|---|---|
| `main` | **Final firmware** — manual mode, 100% speed, Serial errors only |
| `motor_test` | Motor test tool — verbose per-tick log, run-time timeout |
| `ina_test` | INA219 verification and current logging |
| `buttons_test` | Button diagnostics (Serial only, no motor) |
| `i2c_scan` | I2C scanner + device ID check |

> Nano clones with the CH340 chip often use the **old bootloader (57600 baud)** —
> hence `board = nanoatmega328`. The port is pinned by a stable `by-id` symlink in
> `platformio.ini`; update it if you use a different adapter.

## Controls

- **UP (D2)** / **DOWN (D3)** held → move; both together → stop.
- Soft start/stop ramp on every move.
- Overcurrent (sustained or instantaneous, separate up/down limits) or supply
  undervoltage → FAULT: motor cut, clears after all buttons are released.

## Roadmap

- [x] Button identification and debounce (+ click / long-press events)
- [x] Manual drive: soft start/stop, state machine
- [x] Current sensing via INA219 (R020 shunt) + overcurrent + undervoltage
- [x] Closed-loop go-to + EEPROM presets — built and tested with sonar, **archived**
- [ ] Reliable position source (narrow-beam LiDAR or shaft encoder)
- [ ] Re-enable go-to + presets in `main`

Archived work: [`attic/sonar_goto/`](attic/sonar_goto/README.md) (sonar, go-to test,
presets) and [`attic/height_sensor/`](attic/height_sensor/README.md) (VL53L0X).

## Safety

This drives a motor from a 12 V / several-amp supply. Common ground between the
Nano, IBT-2 and PSU is mandatory; an inline fuse on +12 V is recommended. Keep a
hand near the power switch during testing.

With the high-ratio gearbox the desk moves slowly and current barely rises under
normal load, so the overcurrent limit reliably catches a hard stop but may **not**
catch a gentle drive into an end stop. There are no soft limits yet; the current
limit is the backup end-stop.
