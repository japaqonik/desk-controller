// =============================================================================
// motion_controller.cpp (plan 5.4, 5.5)
// =============================================================================
#include "motion_controller.h"
#include "config.h"

using namespace cfg;

MotionController::MotionController()
    : ramp_(kRampUpPerMs, kRampDownPerMs) {}

void MotionController::begin() {
  motor_.begin();
  current_.begin();
  ramp_.hardStop();
  state_     = MotionState::Idle;
  fault_     = FaultReason::None;
  reqDir_    = 0;
  activeDir_ = 0;
  lastOutput_ = 0;
}

void MotionController::requestDirection(int8_t dir) {
  reqDir_ = dir;
}

void MotionController::enterFault(FaultReason r) {
  motor_.disable();
  ramp_.hardStop();
  state_      = MotionState::Fault;
  fault_      = r;
  activeDir_  = 0;
  lastOutput_ = 0;
}

void MotionController::tick(uint16_t dtMs, bool anyButtonHeld) {
  const uint32_t now = millis();

  // --- FAULT: exit only after all buttons are released ---
  if (state_ == MotionState::Fault) {
    motor_.disable();
    if (!anyButtonHeld) {
      state_ = MotionState::Idle;
      fault_ = FaultReason::None;
      current_.reset();
    }
    lastOutput_ = 0;
    return;
  }

  // Logical direction -> physical PWM sign.
  const int8_t physReq = static_cast<int8_t>(reqDir_ * kUpSign);

  // --- State machine ---
  switch (state_) {
    case MotionState::Idle:
      motor_.disable();
      current_.reset();
      ramp_.hardStop();
      if (reqDir_ != 0) {
        activeDir_      = physReq;
        runStartMs_     = now;
        sustainedStart_ = 0;
        motor_.enable();
        ramp_.setTarget(static_cast<float>(activeDir_) * kTestSpeedLimit);
        state_ = MotionState::RampUp;
      }
      break;

    case MotionState::RampUp:
    case MotionState::Run:
      // Stop or direction change requested -> ramp down.
      if (reqDir_ == 0 || physReq != activeDir_) {
        ramp_.setTarget(0.0f);
        state_ = MotionState::RampDown;
      } else {
        ramp_.setTarget(static_cast<float>(activeDir_) * kTestSpeedLimit);
        if (fabsf(ramp_.current()) >= static_cast<float>(abs(kTestSpeedLimit)) - 0.5f) {
          state_ = MotionState::Run;
        }
      }
      break;

    case MotionState::RampDown:
      ramp_.setTarget(0.0f);
      if (fabsf(ramp_.current()) < 0.5f) {
        state_ = MotionState::Idle;
      }
      break;

    case MotionState::Fault:
      break;  // handled above
  }

  // --- Update ramp and motor output ---
  const float out = ramp_.update(dtMs);
  lastOutput_ = static_cast<int16_t>(out);
  if (state_ != MotionState::Idle) {
    motor_.setOutput(lastOutput_);
  }

  // Current physical motion direction (by output sign).
  const int8_t movingDir = (lastOutput_ > 0) ? 1 : (lastOutput_ < 0 ? -1 : 0);

  // --- Current measurement ---
  const float amps = current_.update(movingDir);

  // --- Protection (plan 5.3/5.4) ---
  // Current thresholds apply ONLY at full duty (Run state). During RAMP_UP the
  // IS line is PWM-chopped at partial duty and the reading is unreliable
  // (plan 5.3), so we do NOT trip OC there - only the timeout guards against
  // a jam.
  if (state_ == MotionState::RampUp || state_ == MotionState::Run) {
    const uint32_t runFor = now - runStartMs_;

    // Current thresholds only at cruise + after the blanking window.
    if (state_ == MotionState::Run && runFor >= kBlankingMs) {
      // Instantaneous limit.
      if (amps >= kCurrentInstantA) {
        enterFault(FaultReason::OverCurrentInstant);
        return;
      }
      // Sustained limit.
      if (amps >= kCurrentSustainedA) {
        if (sustainedStart_ == 0) sustainedStart_ = now;
        else if ((now - sustainedStart_) >= kSustainedMs) {
          enterFault(FaultReason::OverCurrentSustained);
          return;
        }
      } else {
        sustainedStart_ = 0;
      }
    } else {
      sustainedStart_ = 0;  // reset the counter until we are at cruise
    }

    // Run-time limit (always active while moving).
    if (runFor >= kMaxRunMs) {
      enterFault(FaultReason::RunTimeout);
      return;
    }
  }
}
