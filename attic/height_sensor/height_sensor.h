// =============================================================================
// height_sensor.h - VL53L0X wrapper: filtering + validity (plan 6)
// =============================================================================
// Continuous ranging. Pipeline: raw read -> range-status/plausibility check ->
// median-of-N -> light EMA. "Height" is the raw distance to the floor in mm.
//
// API (plan 6.1):
//   heightMm() - last good filtered distance in mm
//   isValid()  - whether the last reading passed all checks
//   ageMs()    - time since the last GOOD reading (staleness)
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class HeightSensor {
public:
  // Returns false if the sensor did not initialise (not present / wrong wiring).
  bool begin();

  // Call periodically (e.g. every control tick). Non-blocking: only consumes a
  // sample when the sensor has one ready. Updates filtered value / validity.
  void update();

  uint16_t heightMm() const { return filteredMm_; }
  bool     isValid()  const { return valid_; }
  uint32_t ageMs()    const { return millis() - lastGoodMs_; }
  bool     present()  const { return present_; }

private:
  uint16_t medianMm(uint16_t sample);

  bool     present_    = false;
  bool     valid_      = false;
  uint16_t filteredMm_ = 0;
  uint16_t lastGoodMm_ = 0;
  uint32_t lastGoodMs_ = 0;

  // Median window.
  uint16_t window_[cfg::kHeightMedianN] = {0};
  uint8_t  windowFill_ = 0;
  uint8_t  windowPos_  = 0;
  bool     emaInit_    = false;
  float    ema_        = 0.0f;
};
