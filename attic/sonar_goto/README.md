# attic/sonar_goto — archived HC-SR04 sonar, go-to test and presets

Working, previously tested code. **Not compiled** by any active environment.

## Why it's here

The HC-SR04 worked (non-blocking echo timing via pin-change interrupt, stable
readings at rest), and closed-loop go-to-position reached its target reliably.
But the sonar's wide cone (~30 cm footprint on the floor) picks up anything under
the desk: a backpack hung near the sensor turned 588 mm into 420 mm. That is not
a safe basis for automatic moves to stored presets.

Candidates for a replacement position source: a narrow-beam 1D LiDAR (TF-Luna /
TFmini-S, ~2° beam) or a shaft encoder (AS5600) with current-based homing. See
plan section 12.

## Files

- `distance_sensor.{h,cpp}` — HC-SR04 driver: PCINT on D11 for echo timing,
  median + EMA, `distanceMm()` / `isValid()` / `ageMs()`.
- `settings.{h,cpp}` — EEPROM presets (4 slots, magic + version + CRC-16).
- `sonar_test/` — distance logging.
- `closed_loop_test/` — manual + P1 go-to a hardcoded target (verified working).
- `preset_test/` — P1 click = go to preset, P1 hold = save current height.

Still active in `src/` (sensor-independent, reused by any future sensor):
- `MotionController` go-to mode (`updateDistance()`, `requestGoTo()`, no-progress /
  stale / timeout safety) and the `kGoto*` constants in `config.h`.
- `Buttons` click / long-press events.

## How to restore (with a new sensor)

1. Write a driver with the same API as `DistanceSensor` (or restore this one).
2. Move `settings.*` and the needed test folder back into `src/`.
3. Re-add the env to `platformio.ini`, e.g.:
   `build_src_filter = -<*> +<preset_test/> +<buttons.cpp> +<motor_driver.cpp> +<current_sensor.cpp> +<motion_controller.cpp> +<distance_sensor.cpp> +<settings.cpp>`
4. For HC-SR04 specifically, re-add to `cfg` in `config.h`:
   `kPinSonarTrig = 12`, `kPinSonarEcho = 11`, `kSonarTimeoutUs = 25000`,
   `kSonarMedianN = 5`, `kSonarEmaAlpha = 0.40f`, `kSonarMinMm = 20`,
   `kSonarMaxMm = 4000`.
5. Before enabling automatic moves, add the plan 6.3 check: refuse a move if the
   reading at rest differs too much from the last known resting position.
