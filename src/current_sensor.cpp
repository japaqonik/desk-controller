// =============================================================================
// current_sensor.cpp (plan 5.3)
// =============================================================================
#include "current_sensor.h"

using namespace cfg;

void CurrentSensor::begin() {
  pinMode(kPinRis, INPUT);
  pinMode(kPinLis, INPUT);
  reset();
}

void CurrentSensor::reset() {
  emaA_       = 0.0f;
  lastCounts_ = 0;
  windowFill_ = 0;
  windowPos_  = 0;
  for (uint8_t i = 0; i < kCurrentMedianN; ++i) window_[i] = 0;
}

// Oversample one pin and return the averaged raw ADC counts.
uint16_t CurrentSensor::readAveraged(uint8_t pin) const {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < kCurrentOversample; ++i) {
    acc += analogRead(pin);
  }
  return static_cast<uint16_t>((acc + kCurrentOversample / 2) / kCurrentOversample);
}

// Insert a sample into the circular window and return the current median.
uint16_t CurrentSensor::medianCounts(uint16_t sample) {
  window_[windowPos_] = sample;
  windowPos_ = static_cast<uint8_t>((windowPos_ + 1) % kCurrentMedianN);
  if (windowFill_ < kCurrentMedianN) ++windowFill_;

  // Copy valid part and insertion-sort (N is small, e.g. 5).
  uint16_t tmp[kCurrentMedianN];
  for (uint8_t i = 0; i < windowFill_; ++i) tmp[i] = window_[i];
  for (uint8_t i = 1; i < windowFill_; ++i) {
    const uint16_t key = tmp[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = key;
  }
  return tmp[windowFill_ / 2];
}

float CurrentSensor::update(int8_t dir) {
  if (dir == 0) {
    // Not driving: let the reading decay and clear the median window so a new
    // move starts clean.
    reset();
    return emaA_;
  }

  const uint8_t pin = (dir > 0) ? kPinRis : kPinLis;
  const uint16_t avg = readAveraged(pin);
  const uint16_t med = medianCounts(avg);

  // Subtract zero offset (clamp at 0) -> counts proportional to current.
  const uint16_t offsetCounts =
      (med > kIsZeroCounts) ? static_cast<uint16_t>(med - kIsZeroCounts) : 0;
  lastCounts_ = offsetCounts;

  const float volts = static_cast<float>(offsetCounts) * kAdcVoltsPerStep;
  const float rawA  = volts / kIsVoltsPerAmp;

  emaA_ += kCurrentEmaAlpha * (rawA - emaA_);
  return emaA_;
}
