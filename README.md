# Desk Controller

Firmware for a motorized standing desk, built around an **Arduino Nano (ATmega328P)**.
Manual up/down with soft start/stop and INA219-based overcurrent protection is
done; a closed-loop "go to position" using an HC-SR04 ultrasonic sensor is
working in a test harness, with EEPROM presets next.

> Status:
> - **Manual mode: complete** — hold-to-move up/down, soft ramp, INA219
>   overcurrent protection (separate up/down limits) and undervoltage fault.
> - **Closed-loop go-to-position: working in `closed_loop_test`** — drives to a
>   target distance with slow-down near the target and no-progress / stale-sensor
>   safety. Not yet in the final firmware.
> - **Presets: next** (store/recall heights in EEPROM).
>
> An earlier VL53L0X ToF sensor was too short-range on the real floor and is
> archived in `attic/`; the HC-SR04 ultrasonic sensor replaced it.

## Hardware

| Part | Role |
|---|---|
| Arduino Nano (ATmega328P) | controller |
| IBT-2 (BTS7960) | H-bridge motor driver |
| 12 V PSU (>= ~8 A) | motor supply |
| INA219 (I2C, R020 shunt) | current sensing (protection) |
| HC-SR04 (D12 trig / D11 echo) | ultrasonic height sensor (position) |
| 4× momentary switch | UP / DOWN / P1 / FN |

Pin map and wiring details live in [`src/config.h`](src/config.h) and the
[implementation plan](desk-controller-plan.md).

Key design choices:
- **20 kHz PWM** on Timer1 (inaudible, within the BTS7960 spec), so `millis()` is untouched.
- No `delay()`, no `String`, no heap — fixed **5 ms control tick**, watchdog enabled.
- Current measured digitally over I2C (INA219 with a 0.02 Ω shunt) — clean even at
  low current, no PWM noise on the ADC (the IBT-2 IS line was too noisy).
- HC-SR04 echo timed by a **pin-change interrupt** (D11), so the control loop never
  blocks.
- Direction (`kUpSign`), speeds, ramp rates, current and go-to thresholds are all
  `constexpr` in `config.h`.

## Build & flash (PlatformIO)

The final firmware is the `main` environment (also the default). The others are
bring-up / diagnostic tools. In the VSCode PlatformIO bar, or from the CLI:

```bash
pio run -t upload                      # builds the default env (main)
pio device monitor                     # 115200 baud
pio run -e closed_loop_test -t upload  # sonar go-to test harness
```

| Environment | Purpose |
|---|---|
| `main` | **Final firmware** — manual mode, 100% speed, Serial errors only |
| `motor_test` | Motor test tool — verbose per-tick Serial log, run-time timeout |
| `closed_loop_test` | Sonar go-to-position test (manual + P1 drives to a target) |
| `sonar_test` | HC-SR04 distance logging |
| `ina_test` | INA219 verification and current logging |
| `buttons_test` | Button diagnostics (Serial only, no motor) |
| `i2c_scan` | I2C scanner + device ID check |

> Nano clones with the CH340 chip often use the **old bootloader (57600 baud)** —
> hence `board = nanoatmega328`. The port is pinned by a stable `by-id` symlink in
> `platformio.ini`; update it if you use a different adapter.

## Controls (final firmware)

- **UP (D2)** / **DOWN (D3)** held → move; both together → stop.
- Soft start/stop ramp on every move.
- Overcurrent (sustained or instantaneous, separate up/down) or supply
  undervoltage → FAULT: motor cut, clears after all buttons are released.

(Preset buttons P1/FN and closed-loop go-to are being integrated; today they live
in `closed_loop_test`.)

## Roadmap

- [x] Button identification and debounce
- [x] Manual drive: soft start/stop, state machine
- [x] Current sensing via INA219 (R020 shunt) + overcurrent + undervoltage
- [x] HC-SR04 ultrasonic position sensing (non-blocking, filtered)
- [x] Closed-loop "go to position" (in `closed_loop_test`)
- [ ] EEPROM presets (store/recall), FN key for presets #2–#4
- [ ] Integrate go-to + presets into the final `main` firmware

See [`desk-controller-plan.md`](desk-controller-plan.md) for the full design and
the discussion of position-sensing options.

## Safety

This drives a motor from a 12 V / several-amp supply. Common ground between the
Nano, IBT-2 and PSU is mandatory; an inline fuse on +12 V is recommended. Keep a
hand near the power switch during testing.

Notes:
- With the high-ratio gearbox the desk moves slowly and current barely rises under
  load, so the overcurrent limit reliably catches a hard overload but may **not**
  catch a gentle drive into an end stop. Soft limits are not yet implemented;
  the current limit is the backup end-stop.
- Closed-loop moves abort on no-progress (wrong direction / stall), stale sensor
  and a move timeout; any button press cancels an automatic move.
