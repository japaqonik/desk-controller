// =============================================================================
// motion_controller.h - Step 1 state machine (plan 5.5)
// =============================================================================
// IDLE -> RAMP_UP -> RUN -> RAMP_DOWN -> IDLE, plus FAULT.
// Direction reversal only through IDLE. FAULT clears after all buttons released.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"
#include "current_sensor.h"
#include "ramp.h"

enum class MotionState : uint8_t { Idle, RampUp, Run, RampDown, Fault };

enum class FaultReason : uint8_t { None, OverCurrentInstant, OverCurrentSustained, RunTimeout };

class MotionController {
public:
  MotionController();
  void begin();

  // Optional per-firmware run-time safety limit (0 = disabled). Defaults to
  // cfg::kMaxRunMs. The final firmware disables it; the test tool sets its own.
  void setMaxRunMs(uint32_t ms) { maxRunMs_ = ms; }

  // Optional per-firmware speed limit (PWM duty, 0..cfg::kPwmMax). Defaults to
  // cfg::kTestSpeedLimit. The test tool may lower it.
  void setSpeedLimit(int16_t duty) { speedLimit_ = duty; }

  // Input from button logic: requested direction -1 (down), 0 (stop), +1 (up).
  // "up"/"down" are mapped to the PWM sign via cfg::kUpSign in tick().
  void requestDirection(int8_t dir);

  // Called every fixed tick (dtMs). anyButtonHeld allows clearing FAULT.
  void tick(uint16_t dtMs, bool anyButtonHeld);

  MotionState state() const { return state_; }
  FaultReason fault() const { return fault_; }
  float       amps()  const { return current_.amps(); }
  float       busVolts() const { return current_.busVolts(); }
  bool        currentPresent() const { return current_.present(); }
  int16_t     output() const { return lastOutput_; }

private:
  void enterFault(FaultReason r);

  MotorDriver   motor_;
  CurrentSensor current_;
  Ramp          ramp_;

  MotionState state_      = MotionState::Idle;
  FaultReason fault_      = FaultReason::None;
  int8_t      reqDir_     = 0;    // requested logical direction (-1/0/+1)
  int8_t      activeDir_  = 0;    // current physical motion direction
  int16_t     lastOutput_ = 0;

  uint32_t runStartMs_      = 0;  // when the current move started (blanking + timeout)
  uint32_t sustainedStart_  = 0;  // since when current > sustained (0 = not exceeded)
  uint8_t  ocCount_         = 0;  // consecutive instant/saturation over-threshold ticks
  uint32_t maxRunMs_        = cfg::kMaxRunMs;  // run-time limit (0 = disabled)
  int16_t  speedLimit_      = cfg::kTestSpeedLimit;  // max PWM duty
};
