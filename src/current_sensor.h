// =============================================================================
// current_sensor.h - motor current via INA219 (I2C)
// =============================================================================
// Replaces the noisy IBT-2 IS line. The INA219 measures current across its
// shunt and reports it digitally over I2C - clean even at low current, no PWM
// noise on the ADC. Light EMA smooths the reading.
//
// The direction argument is kept for API compatibility with the caller; the
// INA219 is in the common +12V line and measures total magnitude regardless of
// H-bridge direction, so we report the absolute current.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class CurrentSensor {
public:
  // Returns false if the INA219 did not respond (not present / wrong address).
  bool begin();

  // Reads the INA219 and updates the filtered current. dir is unused for the
  // measurement (INA219 is in the supply line) but kept for API symmetry.
  float update(int8_t dir);

  float    amps()      const { return emaA_; }
  float    busVolts()  const { return busV_; }
  bool     present()   const { return present_; }
  bool     saturated() const { return saturated_; }
  void     reset()           { emaA_ = 0.0f; }

private:
  bool  present_   = false;
  bool  saturated_ = false;
  float emaA_      = 0.0f;
  float busV_      = 0.0f;
};
