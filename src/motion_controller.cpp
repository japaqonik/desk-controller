// =============================================================================
// motion_controller.cpp (plan 5.4, 5.5, 7)
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
  goToReq_   = false;
}

void MotionController::requestDirection(int8_t dir) {
  reqDir_ = dir;
}

void MotionController::updateDistance(uint16_t mm, bool valid, uint32_t ageMs) {
  distMm_    = mm;
  distValid_ = valid;
  distAgeMs_ = ageMs;
}

void MotionController::requestGoTo(uint16_t targetMm) {
  // Only start from Idle and only if we have a fresh, valid reading.
  if (state_ == MotionState::Idle && distValid_ && distAgeMs_ <= kGotoSensorMaxAgeMs) {
    targetMm_ = targetMm;
    goToReq_  = true;
  }
}

void MotionController::enterFault(FaultReason r) {
  motor_.disable();
  ramp_.hardStop();
  state_      = MotionState::Fault;
  fault_      = r;
  lastFault_  = r;   // sticky for diagnostics (survives auto-clear)
  activeDir_  = 0;
  lastOutput_ = 0;
  goToReq_    = false;
}

// Shared current / undervoltage / timeout protection. Runs whenever the motor is
// driving (manual ramp states or a positioned move). Returns true if a fault was
// entered (caller should stop processing this tick).
void MotionController::runProtection(uint16_t /*dtMs*/) {
  const uint32_t now = millis();
  const uint32_t runFor = now - runStartMs_;
  const float amps = current_.amps();

  const bool goingUp = (static_cast<int8_t>(activeDir_ * kUpSign) > 0);
  const float instantLimit   = goingUp ? kCurrentInstantUpA   : kCurrentInstantDownA;
  const float sustainedLimit = goingUp ? kCurrentSustainedUpA : kCurrentSustainedDownA;

  const bool afterBlanking = runFor >= kBlankingMs;
  const bool atCruise      = (state_ == MotionState::Run || state_ == MotionState::GoTo);

  // Instant / saturation (debounced).
  const bool overInstant =
      kCurrentProtectionEnabled && afterBlanking &&
      (current_.saturated() || amps >= instantLimit);
  if (overInstant) {
    if (++ocCount_ >= kOcDebounceTicks) { enterFault(FaultReason::OverCurrentInstant); return; }
  } else {
    ocCount_ = 0;
  }

  // Sustained (time-based) at cruise.
  if (kCurrentProtectionEnabled && atCruise && afterBlanking) {
    if (amps >= sustainedLimit) {
      if (sustainedStart_ == 0) sustainedStart_ = now;
      else if ((now - sustainedStart_) >= kSustainedMs) {
        enterFault(FaultReason::OverCurrentSustained); return;
      }
    } else {
      sustainedStart_ = 0;
    }
  } else {
    sustainedStart_ = 0;
  }

  // Undervoltage (debounced).
  if (kCurrentProtectionEnabled && kUnderVoltageV > 0.0f && afterBlanking &&
      current_.present() && current_.busVolts() < kUnderVoltageV) {
    if (++uvCount_ >= kOcDebounceTicks) { enterFault(FaultReason::UnderVoltage); return; }
  } else {
    uvCount_ = 0;
  }

  // Manual run-time limit (0 = disabled). Positioned moves use their own timeout.
  if (maxRunMs_ != 0 && state_ != MotionState::GoTo && runFor >= maxRunMs_) {
    enterFault(FaultReason::RunTimeout); return;
  }
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

  // A manual button press always cancels / overrides a positioned move.
  if (state_ == MotionState::GoTo && reqDir_ != 0) {
    ramp_.setTarget(0.0f);
    gotoExit_ = 2;  // cancelled by button
    state_ = MotionState::RampDown;
  }

  const int8_t physReq = static_cast<int8_t>(reqDir_ * kUpSign);

  // --- State machine ---
  switch (state_) {
    case MotionState::Idle:
      motor_.disable();
      current_.reset();
      ramp_.hardStop();
      if (reqDir_ != 0) {
        // Manual move.
        activeDir_      = physReq;
        runStartMs_     = now;
        sustainedStart_ = 0; ocCount_ = 0; uvCount_ = 0;
        motor_.enable();
        ramp_.setTarget(static_cast<float>(activeDir_) * speedLimit_);
        state_ = MotionState::RampUp;
      } else if (goToReq_) {
        // Positioned move. Decide direction from target vs current distance.
        goToReq_ = false;
        const int32_t err = static_cast<int32_t>(targetMm_) - static_cast<int32_t>(distMm_);
        if (static_cast<uint16_t>(err < 0 ? -err : err) <= kGotoDeadbandMm) {
          break;  // already at target
        }
        // Higher distance = up. err>0 => need to go up (logical +1).
        const int8_t logicalDir = (err > 0) ? +1 : -1;
        activeDir_      = static_cast<int8_t>(logicalDir * kUpSign);
        runStartMs_     = now;
        gotoStartMs_    = now;
        gotoProgressMs_ = now;
        gotoBestErr_    = static_cast<uint16_t>(err < 0 ? -err : err);
        gotoExit_       = 0;
        sustainedStart_ = 0; ocCount_ = 0; uvCount_ = 0;
        motor_.enable();
        state_ = MotionState::GoTo;
      }
      break;

    case MotionState::RampUp:
    case MotionState::Run:
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

    case MotionState::GoTo: {
      // Sensor must stay valid and fresh.
      if (!distValid_ || distAgeMs_ > kGotoSensorMaxAgeMs) {
        enterFault(FaultReason::GoToStale);
        return;
      }
      const int32_t err = static_cast<int32_t>(targetMm_) - static_cast<int32_t>(distMm_);
      const uint16_t absErr = static_cast<uint16_t>(err < 0 ? -err : err);

      // Reached target.
      if (absErr <= kGotoDeadbandMm) {
        ramp_.setTarget(0.0f);
        gotoExit_ = 1;  // reached
        state_ = MotionState::RampDown;
        break;
      }

      // Speed profile: slow down within the slow zone, clamp to [min,max].
      int16_t duty = kGotoMaxDuty;
      if (absErr < kGotoSlowZoneMm) {
        duty = static_cast<int16_t>(
            static_cast<int32_t>(kGotoMaxDuty) * absErr / kGotoSlowZoneMm);
        if (duty < kGotoMinDuty) duty = kGotoMinDuty;
      }
      const int8_t logicalDir = (err > 0) ? +1 : -1;
      activeDir_ = static_cast<int8_t>(logicalDir * kUpSign);
      ramp_.setTarget(static_cast<float>(activeDir_) * duty);

      // --- Go-to safety: progress-based (robust to sensor noise) ---
      // Instead of reacting to single-sample movement (which sensor noise of a
      // few mm defeats), we track the best |err| reached and require it to keep
      // improving by at least kGotoProgressMm within kGotoProgressWindowMs.
      // This covers both "wrong direction" and "stall": if the desk is not
      // getting meaningfully closer to the target, the move is aborted.
      const bool movePhase = (now - gotoStartMs_) >= kGotoGraceMs;
      if (!movePhase) {
        gotoBestErr_      = absErr;     // reset baseline during ramp
        gotoProgressMs_   = now;
      } else {
        if (absErr + kGotoProgressMm <= gotoBestErr_) {
          gotoBestErr_    = absErr;     // meaningful progress toward target
          gotoProgressMs_ = now;
        } else if ((now - gotoProgressMs_) >= kGotoProgressWindowMs) {
          // No meaningful progress for the whole window -> abort. Distinguish
          // "moving away" (wrong dir) from "not moving" (stall) for the log.
          if (absErr > gotoBestErr_ + kGotoProgressMm) {
            enterFault(FaultReason::GoToWrongDir);
          } else {
            enterFault(FaultReason::GoToStall);
          }
          return;
        }
      }

      // Absolute timeout.
      if ((now - gotoStartMs_) >= kGotoTimeoutMs) {
        enterFault(FaultReason::GoToTimeout); return;
      }
      break;
    }

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

  // Current measurement (direction by output sign).
  const int8_t movingDir = (lastOutput_ > 0) ? 1 : (lastOutput_ < 0 ? -1 : 0);
  current_.update(movingDir);

  // Protection while driving.
  if (state_ == MotionState::RampUp || state_ == MotionState::Run ||
      state_ == MotionState::GoTo) {
    runProtection(dtMs);
  }
}
