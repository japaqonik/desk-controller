// =============================================================================
// motor_driver.cpp (plan 4, 5.1)
// =============================================================================
#include "motor_driver.h"
#include "config.h"

using namespace cfg;

void MotorDriver::begin() {
  pinMode(kPinRen, OUTPUT);
  pinMode(kPinLen, OUTPUT);
  digitalWrite(kPinRen, LOW);   // power stage cut at startup
  digitalWrite(kPinLen, LOW);

  pinMode(kPinRpwm, OUTPUT);
  pinMode(kPinLpwm, OUTPUT);
  digitalWrite(kPinRpwm, LOW);
  digitalWrite(kPinLpwm, LOW);

  // Timer1: Mode 14 (fast PWM, TOP = ICR1), non-inverting on OC1A/OC1B.
  // COM bits are set only when duty > 0 (hard zero by detaching the pin).
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);  // prescaler 1
  ICR1  = kPwmTop;   // 20 kHz
  OCR1A = 0;
  OCR1B = 0;

  lastDir_   = 0;
  enabled_   = false;
  deadUntil_ = 0;
}

void MotorDriver::enable() {
  digitalWrite(kPinRen, HIGH);
  digitalWrite(kPinLen, HIGH);
  enabled_ = true;
}

void MotorDriver::disable() {
  // Hard stop: detach both pins from the timer, force LOW, enable LOW.
  TCCR1A = _BV(WGM11);           // clear COM1A1/COM1B1 -> pins under port control
  digitalWrite(kPinRpwm, LOW);
  digitalWrite(kPinLpwm, LOW);
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(kPinRen, LOW);
  digitalWrite(kPinLen, LOW);
  enabled_   = false;
  lastDir_   = 0;
}

// Attach/detach a channel from the timer and set its OCR value.
// right=true -> OC1A (D9, RPWM); right=false -> OC1B (D10, LPWM).
void MotorDriver::setChannel(bool right, uint16_t value) {
  if (right) {
    if (value == 0) {
      TCCR1A &= ~_BV(COM1A1);      // detach OC1A from the timer
      digitalWrite(kPinRpwm, LOW); // hard zero
      OCR1A = 0;
    } else {
      OCR1A = value;
      TCCR1A |= _BV(COM1A1);       // attach OC1A (non-inverting)
    }
  } else {
    if (value == 0) {
      TCCR1A &= ~_BV(COM1B1);
      digitalWrite(kPinLpwm, LOW);
      OCR1B = 0;
    } else {
      OCR1B = value;
      TCCR1A |= _BV(COM1B1);
    }
  }
}

void MotorDriver::applyDuty(int16_t duty) {
  if (duty > 0) {
    setChannel(false, 0);          // opposite channel always forced to 0 first
    setChannel(true, static_cast<uint16_t>(duty));
  } else if (duty < 0) {
    setChannel(true, 0);
    setChannel(false, static_cast<uint16_t>(-duty));
  } else {
    setChannel(true, 0);
    setChannel(false, 0);
  }
}

void MotorDriver::setOutput(int16_t duty) {
  // Clamp.
  if (duty > kPwmMax)  duty = kPwmMax;
  if (duty < -kPwmMax) duty = -kPwmMax;

  const int8_t dir = (duty > 0) ? 1 : (duty < 0 ? -1 : 0);
  const uint32_t now = millis();

  // Direction change: force zero and start dead-time.
  if (dir != 0 && lastDir_ != 0 && dir != lastDir_) {
    applyDuty(0);
    deadUntil_ = now + kDeadTimeMs;
    lastDir_   = 0;
    return;   // this tick only zeroes; motion resumes after dead-time
  }

  // Dead-time in progress: hold zero.
  if (now < deadUntil_) {
    applyDuty(0);
    return;
  }

  applyDuty(duty);
  if (dir != 0) lastDir_ = dir;
}
