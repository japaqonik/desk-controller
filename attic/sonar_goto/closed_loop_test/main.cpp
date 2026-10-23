// =============================================================================
// Closed-loop test: go-to-position with the HC-SR04 sonar
// =============================================================================
// Manual UP/DOWN as usual, plus P1 (D4) triggers a positioned move to a
// HARDCODED target distance (see kTestTargetMm). Use this to verify that the
// closed loop drives in the correct direction and stops near the target BEFORE
// adding presets/EEPROM.
//
// SAFETY: speed is capped (cfg::kGotoMaxDuty) and the move aborts on
// wrong-direction / stall / stale-sensor / timeout. Any button press cancels a
// positioned move. Keep the test target CLOSE to the current position (short
// cables!) and a hand on the power switch.
//
// Verbose Serial log every 100 ms: state, out, dist, target, current, bus.
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "buttons.h"
#include "distance_sensor.h"
#include "motion_controller.h"

namespace {
Buttons          g_buttons;
DistanceSensor   g_sonar;
MotionController  g_motion;

unsigned long g_lastTick = 0;
unsigned long g_lastLog  = 0;
bool          g_p1Prev   = false;

// HARDCODED test target. Set this close to the current resting distance so the
// first automated move is short. (Desk rests ~610 mm in the test rig.)
constexpr uint16_t kTestTargetMm = 650;

const char* stateName(MotionState s) {
  switch (s) {
    case MotionState::Idle:     return "IDLE";
    case MotionState::RampUp:   return "RAMP_UP";
    case MotionState::Run:      return "RUN";
    case MotionState::RampDown: return "RAMP_DOWN";
    case MotionState::Fault:    return "FAULT";
    case MotionState::GoTo:     return "GOTO";
  }
  return "?";
}

const char* faultName(FaultReason f) {
  switch (f) {
    case FaultReason::None:                 return "-";
    case FaultReason::OverCurrentInstant:   return "OC_INSTANT";
    case FaultReason::OverCurrentSustained: return "OC_SUSTAINED";
    case FaultReason::RunTimeout:           return "RUN_TIMEOUT";
    case FaultReason::UnderVoltage:         return "UNDERVOLTAGE";
    case FaultReason::GoToWrongDir:         return "GOTO_WRONGDIR";
    case FaultReason::GoToStall:            return "GOTO_STALL";
    case FaultReason::GoToStale:            return "GOTO_STALE";
    case FaultReason::GoToTimeout:          return "GOTO_TIMEOUT";
  }
  return "?";
}
}  // namespace

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);

  g_buttons.begin();
  g_sonar.begin();
  g_motion.begin();
  g_motion.setSpeedLimit(static_cast<int16_t>(cfg::kPwmMax * 0.60f));
  g_motion.setMaxRunMs(8000);
  g_lastTick = millis();

  Serial.println();
  Serial.println(F("=== closed_loop_test (manual + P1 go-to) ==="));
  Serial.print(F("INA219: "));
  Serial.println(g_motion.currentPresent() ? F("OK") : F("NOT FOUND"));
  Serial.print(F("Hardcoded go-to target: "));
  Serial.print(kTestTargetMm);
  Serial.println(F(" mm (P1=D4 to trigger)"));
  Serial.println(F("UP=D2 DOWN=D3 (both=stop). Any button cancels a go-to."));

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastTick) < cfg::kTickMs) return;
  const uint16_t dt = static_cast<uint16_t>(now - g_lastTick);
  g_lastTick = now;

  g_buttons.update(dt);
  g_sonar.update();

  // Feed sonar to the controller.
  g_motion.updateDistance(g_sonar.distanceMm(), g_sonar.isValid(), g_sonar.ageMs());

  // Manual direction.
  const bool up   = g_buttons.isDown(Btn::Up);
  const bool down = g_buttons.isDown(Btn::Down);
  int8_t reqDir = 0;
  if (up && !down)      reqDir = +1;
  else if (down && !up) reqDir = -1;
  g_motion.requestDirection(reqDir);

  // P1 rising edge triggers the hardcoded go-to (only when idle & no manual input).
  const bool p1 = g_buttons.isDown(Btn::P1);
  if (p1 && !g_p1Prev && reqDir == 0) {
    g_motion.requestGoTo(kTestTargetMm);
  }
  g_p1Prev = p1;

  g_motion.tick(dt, g_buttons.anyDown());

  if ((now - g_lastLog) >= 100) {
    g_lastLog = now;
    Serial.print(F("state="));
    Serial.print(stateName(g_motion.state()));
    Serial.print(F(" out="));
    Serial.print(g_motion.output());
    Serial.print(F(" dist="));
    Serial.print(g_sonar.distanceMm());
    Serial.print(g_sonar.isValid() ? F("(ok)") : F("(--)"));
    Serial.print(F(" tgt="));
    Serial.print(g_motion.targetMm());
    Serial.print(F(" I="));
    Serial.print(g_motion.amps(), 2);
    Serial.print(F("A bus="));
    Serial.print(g_motion.busVolts(), 1);
    Serial.print(F("V"));
    if (g_motion.state() == MotionState::Fault) {
      Serial.print(F(" fault="));
      Serial.print(faultName(g_motion.fault()));
    }
    Serial.print(F(" last="));
    Serial.print(faultName(g_motion.lastFault()));
    Serial.print(F(" exit="));
    Serial.print(g_motion.gotoExitReason());
    Serial.println();
  }
}
