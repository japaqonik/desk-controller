# Motorized Standing Desk Controller: Implementation Plan (v3)

Arduino Nano (ATmega328P) + IBT-2 (BTS7960) + 12 V / 8 A PSU + VL53L0X-class distance sensor + 4 buttons.
Position comes from the distance sensor only (no dead-reckoning). Current sensing is used for protection.

> **Current scope (see section 13):** the implemented target is a *minimum build*
> — manual up/down with overcurrent protection via an **INA219** (integrated and
> working), and **no presets / no position sensing**. The distance-sensor work is
> archived (range too short on the real floor). The original sections below (6–9)
> and the appendix (section 12) describe deferred/future functionality.

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

---

## 12. Appendix: alternative position-sensing methods

The main plan derives position from a distance sensor only. In practice the
VL53L0X ranged only ~30–70 cm on the real floor (limited by surface
reflectivity, not ambient light), which is short of the desk's travel. This
appendix records alternatives so the position source can be swapped without
touching the motor/button/protection layers. The idea is a `PositionSensor`
module with a stable API (`position()`, `isValid()`, `ageMs()`), consumed by
`MotionController`; only this module changes between methods.

### 12.1 Options overview

| Method | Type | Needs homing? | Light/floor sensitive? | Notes |
|---|---|---|---|---|
| VL53L0X distance (current) | absolute | no | yes (reflectivity) | range too short on this floor |
| VL53L1X distance | absolute | no | yes (less) | ~4 m, drop-in: only `HeightSensor` changes |
| Ultrasonic (MaxBotix) | absolute | no | no (acoustic) | immune to color; weak on soft/angled surfaces |
| Rotary encoder (quadrature) | relative | yes | no | needs interrupt pins (conflicts with D2/D3 buttons) |
| Magnetic encoder AS5600 (I2C) | abs. per turn | yes (multi-turn) | no | counts turns over full travel; no extra IRQ pins |
| Linear/rotary potentiometer | absolute | no | no | mechanical coupling; wiper wear; limited range |
| Run-time dead-reckoning | relative | yes, often | no | drifts; MVP only |

### 12.2 Recommended alternative: encoder + current-based homing

For this hardware (DC gearmotor, IBT-2, existing current sensing) the strongest
alternative is a **shaft encoder for position** plus **homing against an end
stop detected by current**:

- **AS5600** (I2C) is preferred over a quadrature encoder because the Nano's
  interrupt pins D2/D3 are already used by the UP/DOWN buttons. AS5600 is
  absolute within one turn; over the full travel the firmware counts turns.
- The end stop is found by driving into it and detecting the current rise
  (reuses the protection sensor — ideally upgraded to INA219, see below). That
  point is the zero reference; the encoder counts position from there.
- Presets become stored encoder positions in non-volatile memory (same scheme
  as section 8.3, with the field meaning "encoder position" instead of "mm").

### 12.3 Persisting position without constant homing

Assumption: the desk only moves via this firmware. Then position can be restored
from storage at boot instead of homing every time — but the hard case is a
**power loss during a move**.

- **Store on stop only + "in-motion" flag:** write position when a move ends;
  set an `in_motion` flag at move start, clear it at move end. On boot, if the
  flag is still set, the last power-off was "dirty" (interrupted mid-move) →
  position is uncertain → do a homing pass. Otherwise trust the stored value.
  Minimal wear, homing only in the rare dirty case.
- **Continuous storage (needs the right memory):** writing position every few
  tens of ms would destroy the internal EEPROM (~100k cycles). Use **FRAM**
  (I2C, e.g. MB85RC-series): effectively unlimited write cycles, instant writes.
  Then a power loss costs only the last ~10–50 ms of movement (sub-millimeter),
  with no wear. External EEPROM with wear-leveling is a cheaper, weaker option.
- **Periodic / cyclic re-homing:** relative encoders drift (missed counts,
  slip). Keep a **boot counter** in non-volatile memory and force a homing pass
  every N starts (or every N move cycles, or once a day). This caps accumulated
  error without homing on every boot. Combined with the dirty-flag check above,
  the user rarely sees a homing move.
- **Optional brown-out save:** detect the supply dropping (ADC divider or the
  AVR brown-out detector) and write position in the last few ms before the bulk
  capacitor drains (one EEPROM write ≈ 3.3 ms; FRAM is faster). Eliminates even
  the dirty-case homing, but requires verifying the energy reserve after power
  loss.

### 12.4 Current sensing upgrade (enables protection and homing)

The IBT-2 IS line proved too noisy at low current (free-running ~0.2 A sits near
the ADC noise floor, even with an RC filter). An **INA219/INA226** (I2C
shunt-based current/voltage monitor, address different from other I2C devices)
would give a clean, calibrated current reading usable even at low current. This
improves overcurrent protection and provides reliable end-stop detection for the
homing scheme above. This matches the fallback already noted in section 5.3.

### 12.5 Pin / bus implications

- Quadrature encoders want D2/D3 (external interrupts), which are taken by
  UP/DOWN. Prefer an **I2C** position source (AS5600) or remap the buttons.
- AS5600, INA219 and a future VL53L1X all share the I2C bus (A4/A5) at distinct
  addresses — budget for that if combining them.
- A potentiometer needs one spare ADC pin (A0 and A3 are currently free).

### 12.6 Distance-sensor technologies compared

If position is derived from distance to the floor, these are the realistic
sensor families. All "distance to floor" methods have a weak spot; none is
perfect in a normal room.

| Technology | Example | Immune to floor color / light? | Range | Integration | Notes |
|---|---|---|---|---|---|
| Optical ToF | VL53L0X / VL53L1X | no (reflectivity; sun adds IR) | 1.2 m / ~4 m | easy (I2C) | VL53L1X is a drop-in upgrade |
| Optical triangulation | Sharp GP2Y0A | no | 0.1–1.5 m | easy (analog) | nonlinear, noisy; inferior to ToF |
| 1D LiDAR | TF-Luna, TFmini | partly | several m | easy (UART/I2C) | robust, slight overkill |
| Ultrasonic (basic) | HC-SR04 | yes for color, poor otherwise | ~4 m | easy | wide cone, soft surfaces absorb echo |
| Ultrasonic (quality) | MaxBotix HRLV | yes for color | several m | medium | good on hard floor; carpet still weak |
| mmWave radar | 24/60 GHz modules | **yes** | varies | **hard** | mostly presence sensors; ranging units complex |

Key point: optical methods depend on **surface reflectivity** (the observed
limitation here), ultrasonic struggles with soft/angled surfaces (carpet) and a
wide beam, radar is immune but complex. This is why commercial desks usually
measure the **mechanism** (encoder) rather than distance to the floor.

### 12.7 Other methods (not distance / encoder / pot)

- **Current-ripple counting:** a brushed DC motor's current has periodic ripple
  (one bump per commutator segment pass). Counting ripples counts rotation using
  **only the current sensor** (INA219) — no extra hardware; it reuses the
  protection signal. Used commercially (car windows, actuators). Downsides: needs
  a clean, fast-sampled current signal and careful filtering; noisy at start/stop
  and can miss counts. Elegant as a complement, less reliable as the sole source.
- **Back-EMF integration:** motor voltage ∝ speed; integrate to get position.
  Dead-reckoning (drifts), fails near zero speed. Rarely sufficient alone.
- **Limit switches / Hall + magnets:** discrete reference points, not continuous
  position. Cheap and reliable for homing when combined with a counter.
- **Reflective optical strip / slotted tape + sensor:** a hand-made linear
  encoder along the column.
- **Linear magnetic encoder (tape + head, e.g. AS5311):** proper absolute/
  incremental linear measurement; immune to light/dirt; more mounting work.
- **Magnetostrictive (Temposonics), LVDT:** industrial-grade, accurate, absolute
  — but large and expensive; overkill here.
- **Optical-mouse sensor / camera:** exotic and unreliable on a smooth column /
  too heavy for the Nano.

### 12.8 Decision guide

- **Least effort, works in a normal room:** VL53L1X (drop-in distance sensor).
- **Most reliable, commercial-standard:** AS5600 encoder on the shaft + homing by
  current-detected end stop; presets stored as encoder positions.
- **Least extra hardware:** current-ripple counting from the INA219 (hardest to
  make reliable).
- **No homing ever required:** absolute sensor (distance sensor, or linear
  magnetic encoder); relative encoders need homing or persisted position
  (see 12.3).

### 12.9 Available parts / current status

- An **INA219** (I2C shunt current/voltage monitor) is on hand. It is the
  preferred upgrade over the IBT-2 IS line for current sensing (clean, calibrated,
  usable even at low current) and it enables both solid overcurrent protection and
  current-based end-stop homing. Wire it in series on the +12 V line (Vin+/Vin−),
  logic from 5 V, SDA/SCL on A4/A5 (address 0x40, distinct from other I2C parts).

---

## 13. Current scope: minimum viable build (implemented)

Position sensing and presets are **deferred**. The height-sensor code is archived
in `attic/height_sensor/` (VL53L0X range was insufficient on the real floor). The
implemented build is a **manual-mode minimum**:

- **Manual up/down** (hold to move, both = stop), with soft start/stop ramp.
- **Overcurrent protection** via the INA219 (replacing the noisy IBT-2 IS line).
  Measured free-running current: cruise ~0.6–0.76 A at 100% speed. Thresholds:
  sustained 1.2 A / instantaneous 1.5 A, 400 ms inrush blanking, 3-tick decision
  debounce, plus a hard fault on INA219 shunt saturation.
- **No presets, no closed-loop, no distance sensor** — height is set manually.

### 13.1 Firmware layout

Shared modules live in `src/` (`config.h`, `buttons`, `motor_driver`,
`current_sensor` (INA219), `ramp`, `motion_controller`). PlatformIO environments:

| Env | Role | Speed | Timeout | Serial |
|---|---|---|---|---|
| `main` | final firmware (default) | 100% | none | errors only |
| `motor_test` | bring-up / tuning tool | 60% | 5 s | verbose per-tick log |
| `buttons_test` | button diagnostics | — | — | events |
| `i2c_scan` | I2C scan + device ID | — | — | scan |
| `ina_test` | INA219 verification / logging | — | — | current log |

`config.h` holds the final defaults; `motor_test` overrides speed and timeout at
runtime via `MotionController::setSpeedLimit()` / `setMaxRunMs()`.

### 13.2 Known limitation

With the high-ratio gearbox the motor current barely rises under load (hard
finger braking reached only ~0.8 A). The overcurrent limit therefore catches a
real overload but may **not** catch a gentle drive into an end stop. Until
end-stop handling is based on position or limit switches (section 12), the
current limit should not be relied on as the sole end-stop protection.

Position sensing (encoder / distance / etc.), persisted position, homing and
presets remain documented above as future options to revisit once the minimum
build is solid.
