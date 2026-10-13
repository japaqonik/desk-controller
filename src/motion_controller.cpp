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
        ocCount_        = 0;
        motor_.enable();
        ramp_.setTarget(static_cast<float>(activeDir_) * speedLimit_);
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
        ramp_.setTarget(static_cast<float>(activeDir_) * speedLimit_);
        if (fabsf(ramp_.current()) >= static_cast<float>(abs(speedLimit_)) - 0.5f) {
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
  // With the INA219 the current reading is clean throughout (it measures the DC
  // supply line, not a PWM-chopped IS pin), so it is valid during RAMP_UP too.
  // We still apply the tuned thresholds at cruise to avoid tripping on inrush,
  // but an INA219 saturation (current beyond the shunt's full scale) is always
  // treated as an immediate overcurrent fault while moving.
  if (state_ == MotionState::RampUp || state_ == MotionState::Run) {
    const uint32_t runFor = now - runStartMs_;

    // Instant/saturation condition: current beyond the instantaneous limit or
    // the shunt full scale. Debounced over kOcDebounceTicks consecutive ticks so
    // a single I2C glitch cannot trip a fault, while a real jam (which persists)
    // still trips within a few ms. Active after the blanking window.
    const bool overInstant =
        kCurrentProtectionEnabled && runFor >= kBlankingMs &&
        (current_.saturated() || amps >= kCurrentInstantA);
    if (overInstant) {
      if (++ocCount_ >= kOcDebounceTicks) {
        enterFault(FaultReason::OverCurrentInstant);
        return;
      }
    } else {
      ocCount_ = 0;
    }

    // Sustained limit: time-based (already debounced by kSustainedMs), only at
    // cruise after the blanking window.
    if (kCurrentProtectionEnabled && state_ == MotionState::Run && runFor >= kBlankingMs) {
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

    // Run-time limit (0 = disabled). Always active while moving when set.
    if (maxRunMs_ != 0 && runFor >= maxRunMs_) {
      enterFault(FaultReason::RunTimeout);
      return;
    }
  }
}
