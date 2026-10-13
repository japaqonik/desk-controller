# attic/height_sensor — archived VL53L0X height sensor (Phase 2)

This is working, previously-built code for the VL53L0X distance sensor. It is
**not compiled** by any active PlatformIO environment.

## Why it's here

On the real floor the VL53L0X ranged only ~30–70 cm (even with a bright target
patch), which is less than the desk's travel. It was archived rather than
deleted. Revisit with a **VL53L1X** (longer range on the same surface) — per the
plan, only the `HeightSensor` class needs to change.

Verified during bring-up: sensor responds at I2C `0x29`, Model ID `0xEE`,
continuous ranging and the median+EMA+plausibility pipeline work. The limitation
is purely optical range, not the code.

## Files

- `height_sensor.h/.cpp` — VL53L0X wrapper (non-blocking read, median+EMA,
  validity/age API).
- `height_test/main.cpp` — Phase 2 bring-up tool: logs `dist/valid/age`.

## How to restore

1. Move `height_sensor.{h,cpp}` back into `src/`.
2. Move `height_test/` back into `src/`.
3. In `platformio.ini`, under `[env]` re-add:
   `lib_deps = pololu/VL53L0X@^1.3.1`
4. Re-add the `[env:height_test]` environment:
   `build_src_filter = -<*> +<height_test/> +<height_sensor.cpp>`
5. Re-add the height-sensor constants to `cfg` in `src/config.h` (the module
   references `cfg::kVlTimingBudgetUs`, `kVlTimeoutMs`, `kHeightMedianN`,
   `kHeightEmaAlpha`, `kHeightMaxStepMm`, `kHeightValidMinMm`,
   `kHeightValidMaxMm`).

For a VL53L1X, swap the library and the init/profile calls in
`height_sensor.cpp`; the filtering and public API can stay the same.
