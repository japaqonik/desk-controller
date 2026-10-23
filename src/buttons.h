// =============================================================================
// buttons.h - debounce + click / long-press for 4 buttons (plan 8.2)
// =============================================================================
// isDown()     - debounced held state.
// Events (consume-once; cleared when read):
//   takeClick(b)     - short press: fired on RELEASE if it was not a long press.
//   takeLongPress(b) - fired ONCE when held past the long-press threshold.
// Click vs long-press is decided at the threshold (long press fires while held;
// the following release does not also produce a click).
// =============================================================================
#pragma once
#include <Arduino.h>

enum class Btn : uint8_t { Up = 0, Down = 1, P1 = 2, Fn = 3, Count = 4 };

class Buttons {
public:
  void begin();
  void update(uint16_t dtMs);

  bool isDown(Btn b) const { return stable_[static_cast<uint8_t>(b)]; }
  bool anyDown() const;

  // Consume-once event getters (return true once per event, then reset).
  bool takeClick(Btn b);
  bool takeLongPress(Btn b);

private:
  static constexpr uint8_t kN = static_cast<uint8_t>(Btn::Count);
  uint8_t  pins_[kN]      = {0};
  bool     stable_[kN]    = {false};
  bool     lastRaw_[kN]   = {false};
  uint16_t debAccum_[kN]  = {0};   // ms since raw == candidate
  uint16_t heldMs_[kN]    = {0};   // ms held in stable-down state
  bool     longFired_[kN] = {false};
  bool     clickEvt_[kN]  = {false};
  bool     longEvt_[kN]   = {false};
};
