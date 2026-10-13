// =============================================================================
// current_sensor.cpp (plan 5.3)
// =============================================================================
#include "current_sensor.h"
#include "config.h"

using namespace cfg;

void CurrentSensor::begin() {
  pinMode(kPinRis, INPUT);
  pinMode(kPinLis, INPUT);
  emaA_ = 0.0f;
}

float CurrentSensor::readAmps(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < kCurrentOversample; ++i) {
    acc += analogRead(pin);
  }
  const float avgCounts = static_cast<float>(acc) / kCurrentOversample;
  lastCounts_ = static_cast<uint16_t>(avgCounts + 0.5f);
  const float volts = avgCounts * kAdcVoltsPerStep;
  return volts / kIsVoltsPerAmp;
}

float CurrentSensor::update(int8_t dir) {
  float raw = 0.0f;
  if (dir > 0) {
    raw = readAmps(kPinRis);
  } else if (dir < 0) {
    raw = readAmps(kPinLis);
  }
  emaA_ += kCurrentEmaAlpha * (raw - emaA_);
  return emaA_;
}
