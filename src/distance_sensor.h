// =============================================================================
// distance_sensor.h - HC-SR04 ultrasonic distance (non-blocking, PCINT)
// =============================================================================
// Echo timing is captured by a pin-change interrupt on ECHO (D11 = PCINT3),
// so the main loop never blocks: trigger() emits a 10 us ping, the ISR records
// the rising/falling edge timestamps via micros(), and update() consumes the
// finished measurement, filters it (median + EMA) and validates it.
//
// This is suitable for the 5 ms control loop (no pulseIn blocking). Note the
// ISR covers all of port B (D8-D13); the pin-change mask is set for D11 only,
// so PWM on D9/D10 does not trigger it.
//
// distanceMm()/isValid()/ageMs() mirror the archived height-sensor API.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class DistanceSensor {
public:
  void begin();

  // Call frequently. Triggers a new ping when the previous one finished and the
  // inter-ping gap elapsed; consumes a completed echo (set by the ISR) and
  // updates the filtered distance. Never blocks.
  void update();

  uint16_t distanceMm() const { return filteredMm_; }
  bool     isValid()    const { return valid_; }
  uint32_t ageMs()      const { return millis() - lastGoodMs_; }
  uint32_t lastEchoUs() const;  // raw echo width (diagnostics)

private:
  uint16_t medianMm(uint16_t sample);
  void     acceptReading(uint16_t mm);
  void     trigger();

  uint32_t lastTrigMs_  = 0;   // millis() of last ping (rate limit)
  bool     pingPending_ = false;

  bool     valid_       = false;
  uint16_t filteredMm_  = 0;
  uint32_t lastGoodMs_  = 0;

  uint16_t window_[cfg::kSonarMedianN] = {0};
  uint8_t  windowFill_  = 0;
  uint8_t  windowPos_   = 0;
  bool     emaInit_     = false;
  float    ema_         = 0.0f;
};
