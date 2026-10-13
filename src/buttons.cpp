// =============================================================================
// buttons.cpp
// =============================================================================
#include "buttons.h"
#include "config.h"

using namespace cfg;

namespace {
constexpr uint16_t kDebounceMs = 25;
}

void Buttons::begin() {
  pins_[static_cast<uint8_t>(Btn::Up)]   = kPinBtnUp;
  pins_[static_cast<uint8_t>(Btn::Down)] = kPinBtnDown;
  pins_[static_cast<uint8_t>(Btn::P1)]   = kPinBtnP1;
  pins_[static_cast<uint8_t>(Btn::Fn)]   = kPinBtnFn;
  for (uint8_t i = 0; i < kN; ++i) {
    pinMode(pins_[i], INPUT_PULLUP);
    stable_[i]   = false;
    lastRaw_[i]  = false;
    debAccum_[i] = 0;
  }
}

void Buttons::update(uint16_t dtMs) {
  for (uint8_t i = 0; i < kN; ++i) {
    const bool raw = (digitalRead(pins_[i]) == LOW);  // pressed = LOW
    if (raw != lastRaw_[i]) {
      lastRaw_[i]  = raw;
      debAccum_[i] = 0;   // reset window
    } else if (raw != stable_[i]) {
      debAccum_[i] += dtMs;
      if (debAccum_[i] >= kDebounceMs) {
        stable_[i] = raw;
      }
    }
  }
}

bool Buttons::anyDown() const {
  for (uint8_t i = 0; i < kN; ++i) {
    if (stable_[i]) return true;
  }
  return false;
}
