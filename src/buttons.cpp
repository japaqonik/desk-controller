// =============================================================================
// buttons.cpp
// =============================================================================
#include "buttons.h"
#include "config.h"

using namespace cfg;

namespace {
constexpr uint16_t kDebounceMs  = 25;
constexpr uint16_t kLongPressMs = 1500;  // >= this held -> long press (plan 8.1)
}

void Buttons::begin() {
  pins_[static_cast<uint8_t>(Btn::Up)]   = kPinBtnUp;
  pins_[static_cast<uint8_t>(Btn::Down)] = kPinBtnDown;
  pins_[static_cast<uint8_t>(Btn::P1)]   = kPinBtnP1;
  pins_[static_cast<uint8_t>(Btn::Fn)]   = kPinBtnFn;
  for (uint8_t i = 0; i < kN; ++i) {
    pinMode(pins_[i], INPUT_PULLUP);
    stable_[i]    = false;
    lastRaw_[i]   = false;
    debAccum_[i]  = 0;
    heldMs_[i]    = 0;
    longFired_[i] = false;
    clickEvt_[i]  = false;
    longEvt_[i]   = false;
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
        // State transition (debounced).
        stable_[i] = raw;
        if (stable_[i]) {
          // Pressed.
          heldMs_[i]    = 0;
          longFired_[i] = false;
        } else {
          // Released: if it was not a long press, it's a click.
          if (!longFired_[i]) clickEvt_[i] = true;
        }
      }
    }

    // While stably held, accumulate hold time and fire long-press once.
    if (stable_[i]) {
      if (heldMs_[i] < 0xFFFF) heldMs_[i] += dtMs;
      if (!longFired_[i] && heldMs_[i] >= kLongPressMs) {
        longFired_[i] = true;
        longEvt_[i]   = true;
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

bool Buttons::takeClick(Btn b) {
  const uint8_t i = static_cast<uint8_t>(b);
  if (i >= kN || !clickEvt_[i]) return false;
  clickEvt_[i] = false;
  return true;
}

bool Buttons::takeLongPress(Btn b) {
  const uint8_t i = static_cast<uint8_t>(b);
  if (i >= kN || !longEvt_[i]) return false;
  longEvt_[i] = false;
  return true;
}
