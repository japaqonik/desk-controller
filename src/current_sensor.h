// =============================================================================
// current_sensor.h - reads IS line of the active half-bridge (plan 5.3)
// =============================================================================
#pragma once
#include <Arduino.h>

class CurrentSensor {
public:
  void begin();

  // Reads the IS line matching the direction: dir>0 -> R_IS (A2), dir<0 -> L_IS (A1).
  // dir==0 -> no read, current decays to 0 via EMA.
  // Returns the filtered current in amps.
  float update(int8_t dir);

  float amps() const { return emaA_; }
  uint16_t lastCounts() const { return lastCounts_; }  // raw averaged ADC counts (for calibration)
  void  reset() { emaA_ = 0.0f; lastCounts_ = 0; }

private:
  float readAmps(uint8_t pin);
  float emaA_ = 0.0f;
  uint16_t lastCounts_ = 0;
};
