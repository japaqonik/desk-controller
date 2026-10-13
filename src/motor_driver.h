// =============================================================================
// motor_driver.h - Timer1 PWM 20 kHz, direction, dead-time, enable (plan 4, 5.1)
// =============================================================================
#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  void begin();

  // setOutput(duty) in range -kPwmMax..+kPwmMax.
  // Sign = direction (positive -> RPWM, negative -> LPWM).
  // A direction change forces zero + dead-time (handled internally).
  void setOutput(int16_t duty);

  // Hard stop: both sides 0, enable LOW (cut the power stage).
  void disable();

  // Enable R_EN/L_EN. Called when a move starts.
  void enable();

  bool isEnabled() const { return enabled_; }

private:
  void applyDuty(int16_t duty);          // sets OCR, handles hard zero
  void setChannel(bool right, uint16_t value);

  int8_t  lastDir_   = 0;      // -1, 0, +1
  bool    enabled_   = false;
  uint32_t deadUntil_ = 0;     // millis() until which dead-time lasts
};
