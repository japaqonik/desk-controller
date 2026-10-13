// =============================================================================
// ramp.h - slew-rate limiter (plan 5.2)
// =============================================================================
// Drives the current value toward the target at a limited rate.
// Separate up and down rates. Emergency stop (hardStop) bypasses the ramp.
// Works in float for smoothness; the result is cast to int16 in MotorDriver.
// =============================================================================
#pragma once
#include <Arduino.h>

class Ramp {
public:
  Ramp(float upPerMs, float downPerMs)
      : upPerMs_(upPerMs), downPerMs_(downPerMs), current_(0.0f), target_(0.0f) {}

  void setTarget(float target) { target_ = target; }

  // Immediate stop (fault / direction change).
  void hardStop() {
    current_ = 0.0f;
    target_  = 0.0f;
  }

  // Called every tick. dtMs = time since last step in ms.
  // Returns the updated current value.
  float update(uint16_t dtMs) {
    const float dt = static_cast<float>(dtMs);
    if (current_ < target_) {
      current_ += upPerMs_ * dt;
      if (current_ > target_) current_ = target_;
    } else if (current_ > target_) {
      // Reducing magnitude = braking (faster rate), but only when moving
      // toward zero. Toward zero use downPerMs, away from zero use upPerMs.
      const float rate = (fabsf(target_) < fabsf(current_)) ? downPerMs_ : upPerMs_;
      current_ -= rate * dt;
      if (current_ < target_) current_ = target_;
    }
    return current_;
  }

  float current() const { return current_; }

private:
  const float upPerMs_;
  const float downPerMs_;
  float current_;
  float target_;
};
