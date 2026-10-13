# Motorized Standing Desk Controller: Implementation Plan (v3)

Arduino Nano (ATmega328P) + IBT-2 (BTS7960) + 12 V / 8 A PSU + VL53L0X-class distance sensor + 4 buttons.
Position comes from the distance sensor only (no dead-reckoning). Current sensing is used for protection.

---

## 1. Goals and phases

| Phase | Scope |
|---|---|
| 1 | Manual UP/DOWN with soft start/stop and overcurrent protection (no sensor needed) |
| 2 | Height sensor: reading, filtering, plausibility checks, soft limits |
| 3 | Closed-loop "go to height" |
| 4 | Presets (#1-#4) stored in EEPROM, button logic with FN key |
| 5 | Calibration combo (soft limits), polish |

---

## 2. Hardware

### 2.1 Final pin map

| Function | Pin | Notes |
|---|---|---|
| Button UP | D2 | `INPUT_PULLUP`, other side to GND |
| Button DOWN | D3 | `INPUT_PULLUP` |
| Button P1 | D4 | `INPUT_PULLUP` |
| Button FN | D5 | `INPUT_PULLUP` |
| IBT-2 RPWM | D9 | Timer1 channel A (OCR1A) |
| IBT-2 LPWM | D10 | Timer1 channel B (OCR1B) |
| IBT-2 R_EN | D7 | digital output |
| IBT-2 L_EN | D8 | digital output |
| IBT-2 R_IS | A2 | current sense, right half-bridge |
| IBT-2 L_IS | A1 | current sense, left half-bridge |
| VL53L0X SDA | A4 | I2C |
| VL53L0X SCL | A5 | I2C |
| Feedback LED | D13 | on-board LED |

### 2.2 Power and wiring checklist
- Inline fuse on +12 V sized to the PSU/wiring (about 7.5-10 A).
- Common GND for PSU, IBT-2 and Nano.
- Bulk capacitor (about 1000 µF / 25 V) close to the IBT-2 supply terminals.
- Thick, short motor and supply wires; keep sensor I2C wires away from motor wires.
- Power the VL53L0X breakout from the Nano 5 V / VIN as the module expects. If it is a generic GY-530 board, do not add extra 5 V pull-ups.
- Optional RC filter on each IS line (e.g. 4.7 kΩ in series + 100 nF to GND, near the Nano) to smooth the PWM-chopped signal.

### 2.3 Sensor verification (do this first)
1. Run an I2C scanner: the sensor should answer at `0x29`.
2. Read register `0xC0`: a VL53L0X returns `0xEE`.
3. Log readings at the lowest and the highest desk position over the real floor. The VL53L0X is realistic up to about 1.2 m on a bright surface and much less on dark floors or carpet. If readings at the top position are unstable:
   - put a light-coloured target patch on the floor in the beam,
   - or swap to a VL53L1X later (only the `HeightSensor` class changes).

---

## 3. Software setup

PlatformIO project:

```ini
[env:nanoatmega328new]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 115200
build_unflags = -std=gnu++11
build_flags = -std=gnu++17 -fno-exceptions
src_build_flags = -Wall -Wextra -Werror
lib_deps = pololu/VL53L0X
```

Rules: no `delay()`, no `String`, no heap allocation, no `new`/`malloc`, constants via `constexpr`, `enum class`, watchdog enabled, fixed 5 ms control tick based on `millis()`.

Suggested layout:

```
src/
  main.cpp            // tick scheduler, wiring of modules
  Config.h            // pins, thresholds, timings (constexpr)
  Buttons.h/.cpp      // debounce, click / long press events
  MotorDriver.h/.cpp  // Timer1 PWM, direction, dead time, enable pins
  CurrentSensor.h/.cpp
  HeightSensor.h/.cpp // VL53L0X wrapper, filtering, validity
  Ramp.h              // slew-rate limiter
  MotionController.h/.cpp // state machine: manual + go-to-height
  Settings.h/.cpp     // EEPROM: limits, presets, CRC
tools/
  i2c_scan/           // separate sketch: scanner + model ID check
```

---

## 4. Timer1 PWM (20 kHz on D9 and D10)

BTS7960 is rated for PWM up to 25 kHz, so use about 20 kHz (inaudible, within spec). Timer1 is 16-bit, so TOP can be set freely through `ICR1`.

```cpp
void pwmInit() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  // Mode 14: fast PWM, TOP = ICR1; non-inverting on OC1A (D9) and OC1B (D10)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);  // prescaler 1
  ICR1  = 799;   // 16 MHz / 800 = 20 kHz
  OCR1A = 0;     // RPWM duty 0..799
  OCR1B = 0;     // LPWM duty 0..799
}
```

Notes:
- Timer0 is untouched, so `millis()` keeps working. The `Servo` library (Timer1) cannot be used in this project.
- Do not call `analogWrite()` on D9/D10.
- At duty 0 fast PWM still emits a very narrow pulse. For a true zero, disconnect the pin from the timer (clear `COM1A1`/`COM1B1`) and drive it LOW. `MotorDriver` handles this in one place.

---

## 5. Phase 1: manual drive

### 5.1 MotorDriver
- `setOutput(int16_t)` in the range -799..799; sign = direction. Which sign is "up" is a single constant in `Config.h`, adjusted on the bench.
- On direction change: output 0, wait a few ms (dead time), then switch the other channel. The opposite channel is always forced to 0 first.
- R_EN / L_EN HIGH only while driving; both LOW on fault or idle (hard cut of the power stage).

### 5.2 Ramp (soft start / soft stop)
- Slew-rate limiter: about 0.7 s from 0 to full speed, shorter ramp down on release.
- Emergency stop (fault) bypasses the ramp.

### 5.3 CurrentSensor
- Read only the IS pin of the active direction (A2 for R, A1 for L).
- Oversample (e.g. 16 samples) and average, then an EMA filter. Convert to amps using a calibration factor (the typical IBT-2 gives roughly 0.1 V per amp, but verify with a multimeter/ammeter).
- IS is chopped by the PWM, so readings during ramp (partial duty) are lower than the real current. Apply the strict thresholds at cruise (full duty) and the blanking window during ramp-up.
- Treat an abnormally high IS reading as a driver fault (overtemperature / overcurrent protection of the BTS7960).
- Fallback if the IS signal turns out unusable: shunt resistor or INA219 on I2C (different address than the VL53L0X).

### 5.4 Protection logic
Initial placeholders (must stay below the 8 A PSU and the motor stall current; tune on the bench):

| Check | Initial value |
|---|---|
| Blanking after start | 150-300 ms |
| Sustained limit | about 5-6 A for > 200 ms |
| Instantaneous limit | about 7 A |
| Max continuous run time | e.g. 30 s |

- Exceeding a limit: stop immediately, enter `FAULT`, error pattern on D13. `FAULT` clears only after all buttons are released.
- Optional soft limiting: reduce PWM when current approaches the sustained limit.
- The current limit also acts as backup end-stop detection.

### 5.5 State machine
`IDLE → RAMP_UP → RUN → RAMP_DOWN → IDLE`, plus `FAULT`. Direction reversal only through `IDLE`.

### 5.6 Watchdog
`wdt_enable(WDTO_250MS)`, reset in the main tick. On any reset the motor outputs start disabled.

---

## 6. Phase 2: height sensor

### 6.1 HeightSensor
- Pololu `VL53L0X` library, continuous mode, long-range profile, about 33 ms timing budget (about 30 Hz).
- Enable the AVR Wire timeout (`Wire.setWireTimeout`) so an I2C glitch cannot hang the loop.
- Filtering: median of 5 samples, then a light EMA.
- API: `heightMm()`, `isValid()`, `ageMs()` (here "height" is the raw distance to the floor in mm).

### 6.2 What is stored
Presets and limits are stored as **raw sensor distance (mm)**. Absolute accuracy of the sensor does not matter, only repeatability (roughly ±3-5 mm at about 1 m, better when averaged at rest).

### 6.3 Plausibility checks
The sensor is in the motor housing near the side edge, so something can still enter the beam.
- Reject readings that change faster than the maximum desk speed (plus margin).
- While the motor is stopped, a sudden jump is an obstruction: ignore it and keep the last good value.
- Before any automatic move, compare the current reading with the last resting value; if they differ by more than a tolerance, refuse the move and signal an error.
- Check the sensor's range status flags; discard invalid or out-of-range readings.

---

## 7. Phase 3: closed-loop go-to-height

1. Speed profile: ramp up, cruise, proportional slow-down near the target, deadband about ±2 mm.
2. Measure the coast distance at cruise speed and start braking early (tunable constant). Always approach the target from the same direction at low speed for repeatability.
3. Soft limits (min/max sensor distance) just inside the mechanical stops; the current limit remains as backup.
4. Safety while moving:
   - sensor invalid or stale: stop,
   - PWM high for about 500 ms with less than 0.5 mm height change: stall fault,
   - height changing in the wrong direction: fault,
   - move timeout.
5. Any button press aborts an automatic move (the press is consumed until release).
6. If the sensor is down, manual jog still works (protected by the current limit only).

---

## 8. Phase 4: buttons and presets

### 8.1 Roles
- **B1 (D2) = UP**, **B2 (D3) = DOWN**: hold to move. Both together = stop.
- **B3 (D4)**: click = go to preset #1, hold (≥ 1.5 s) = save preset #1.
- **B4 (D5) = FN**: alone does nothing. While held, B1 / B2 / B3 act as presets **#2 / #3 / #4** (click = go, hold = save).

### 8.2 Rules
- A button's mode (normal or FN) is decided when it is pressed.
- Pressing FN during a manual move stops that move.
- Releasing FN while a preset button is still held cancels the action.
- Going to an unset preset does nothing and flashes an error on D13.
- Successful save: distinct LED pattern. Fault: another pattern.
- Debounce 20-30 ms; click vs. long press decided on release (or at the long-press threshold).

### 8.3 EEPROM (`Settings`)
```cpp
struct Settings {
  uint16_t magic;
  uint8_t  version;
  uint16_t minMm;        // soft limit (lowest desk position)
  uint16_t maxMm;        // soft limit (highest desk position)
  uint16_t preset[4];    // 0 = unset
  uint16_t crc;
};
```
Write with `EEPROM.update` only on an explicit save. On load: verify magic and CRC, otherwise use defaults; clamp presets to the calibrated range.

---

## 9. Phase 5: calibration

The desk is not moved by hand and the sensor is absolute, so no re-homing is needed. Only the soft limits must be known.

1. First: read distances over Serial and set min/max constants (or write them to EEPROM through a Serial command).
2. Later: calibration combo **FN + UP + DOWN held for 3 s** enters calibration mode (LED indicates it):
   - drive to the bottom, hold B3 to store min,
   - drive to the top, hold B3 to store max,
   - FN exits.

---

## 10. Test order

1. I2C scan and sensor ID check; log readings at the lowest and highest position.
2. Motor bench test without load: directions, ramp, PWM at 20 kHz (check with a scope or a multimeter's frequency mode).
3. Calibrate the IS to amps conversion with an ammeter; log current over UART (Serial Plotter).
4. Tune current limits under load (including a deliberate stall test with a short, safe duration).
5. Sensor stability over the real floor; soft limits.
6. Closed loop at low speed, then full speed; measure coast distance.
7. Fault injection: cover the sensor, block the desk, unplug I2C, hold several buttons, power cycle during a move.
8. Presets: save, power cycle, return to presets repeatedly; check repeatability.

---

## 11. Open items

- Confirm the sensor model via the register `0xC0` check; confirm usable range at the highest desk position.
- Motor rated and stall current (to set better initial thresholds).
- Which motor direction corresponds to "up" (one constant in `Config.h`).
- Whether the IS signal is clean enough for protection (otherwise shunt / INA219).
- Confirm how the mechanism behaves at its end stops (hard stop vs. clutch) before any stall testing.
