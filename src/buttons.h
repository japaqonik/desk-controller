// =============================================================================
// buttons.h - debounce for 4 buttons (plan 8.2). Step 1 only uses the pressed
// state; click / long-press will be added in Phase 4.
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

private:
  static constexpr uint8_t kN = static_cast<uint8_t>(Btn::Count);
  uint8_t  pins_[kN] = {0};
  bool     stable_[kN] = {false};
  bool     lastRaw_[kN] = {false};
  uint16_t debAccum_[kN] = {0};   // ms since raw == candidate
};
