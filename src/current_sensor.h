// =============================================================================
// current_sensor.h - reads IS line of the active half-bridge (plan 5.3)
// =============================================================================
// Pipeline: oversample (average) -> median-of-N -> subtract zero offset ->
// convert to amps -> EMA. The median rejects PWM-related spikes; the offset
// removes the ~35-count zero reading of the filtered IS line.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class CurrentSensor {
public:
  void begin();

  // Reads the IS line matching the direction: dir>0 -> R_IS (A2), dir<0 -> L_IS (A1).
  // dir==0 -> no read, current decays to 0 via EMA.
  // Returns the filtered current in amps.
  float update(int8_t dir);

  float amps() const { return emaA_; }
  uint16_t lastCounts() const { return lastCounts_; }  // median counts after offset (for calibration)
  void  reset();

private:
  uint16_t readAveraged(uint8_t pin) const;   // oversample + average -> raw counts
  uint16_t medianCounts(uint16_t sample);     // push into window, return median

  float    emaA_ = 0.0f;
  uint16_t lastCounts_ = 0;                    // median counts after offset subtraction

  // Median window (circular buffer of averaged raw counts).
  uint16_t window_[cfg::kCurrentMedianN] = {0};
  uint8_t  windowFill_ = 0;                     // how many valid samples (0..N)
  uint8_t  windowPos_  = 0;                     // next write position
};
