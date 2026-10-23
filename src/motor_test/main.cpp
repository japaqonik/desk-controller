// =============================================================================
// Step 1: motor test - manual UP/DOWN with ramp and overcurrent protection
// =============================================================================
// Controls:
//   UP (D2)   held -> drive "up"   (sign per cfg::kUpSign)
//   DOWN (D3) held -> drive "down"
//   both             -> stop
//   P1 / FN          -> ignored in this step
//
// SAFETY (bring-up):
//   - speed limited to cfg::kTestSpeedLimit,
//   - max single-move time cfg::kMaxRunMs,
//   - 250 ms watchdog,
//   - current thresholds below the PSU capability.
// Motor FREE-RUNNING at first. Determine the "up" direction by observing
// rotation and, if needed, flip cfg::kUpSign in Config.h.
//
// Serial log every ~100 ms: state, PWM output, current [A], raw ADC counts.
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "buttons.h"
#include "motion_controller.h"

namespace {
Buttons          g_buttons;
MotionController  g_motion;

unsigned long g_lastTick = 0;
unsigned long g_lastLog  = 0;

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
  // Clear any pending watchdog reset and disable the WDT early. On the old
  // bootloader a still-running WDT can trap the board in a reset loop on the
  // next upload (so you must power-cycle). Doing this first prevents that.
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);
  digitalWrite(cfg::kPinLed, LOW);

  g_buttons.begin();
  g_motion.begin();
  // Test/observability tool: full speed with verbose logging, plus a safety
  // timeout (the final fw runs at full speed too but logs errors only and has
  // no timeout).
  g_motion.setSpeedLimit(static_cast<int16_t>(cfg::kPwmMax * 1.00f));
  g_motion.setMaxRunMs(5000);

  g_lastTick = millis();

  Serial.println();
  Serial.println(F("=== motor_test tool (UP/DOWN, ramp, protection) ==="));
  Serial.println(F("BUILD: v14 blank 1s + undervolt"));  // firmware version tag
  Serial.print(F("INA219: "));
  Serial.println(g_motion.currentPresent() ? F("OK") : F("NOT FOUND (current=0)"));
  Serial.print(F("Current protection: "));
  Serial.println(cfg::kCurrentProtectionEnabled ? F("ENABLED") : F("disabled"));
  Serial.print(F("Test speed limit: "));
  Serial.print(static_cast<int16_t>(cfg::kPwmMax * 1.00f));
  Serial.print(F("/"));
  Serial.println(cfg::kPwmMax);
  Serial.println(F("UP=D2 up, DOWN=D3 down, both=stop. Motor should be FREE-RUNNING."));
  Serial.println(F("------------------------------------------------------"));

  // Watchdog at the end of setup, after hardware init.
  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastTick) < cfg::kTickMs) {
    return;  // wait for the next tick (no delay)
  }
  const uint16_t dt = static_cast<uint16_t>(now - g_lastTick);
  g_lastTick = now;

  // 1. Buttons.
  g_buttons.update(dt);

  // 2. Map buttons -> requested (logical) direction.
  const bool up   = g_buttons.isDown(Btn::Up);
  const bool down = g_buttons.isDown(Btn::Down);
  int8_t reqDir = 0;
  if (up && !down)       reqDir = +1;   // up (logical)
  else if (down && !up)  reqDir = -1;   // down
  else                   reqDir = 0;    // both or none -> stop
  g_motion.requestDirection(reqDir);

  // 3. State machine + protection.
  g_motion.tick(dt, g_buttons.anyDown());

  // 4. LED: on while moving, fast blink in FAULT.
  if (g_motion.state() == MotionState::Fault) {
    digitalWrite(cfg::kPinLed, ((now / 100) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(cfg::kPinLed, g_motion.state() == MotionState::Idle ? LOW : HIGH);
  }

  // 5. Log every ~100 ms.
  if ((now - g_lastLog) >= 100) {
    g_lastLog = now;
    Serial.print(F("state="));
    Serial.print(stateName(g_motion.state()));
    Serial.print(F(" out="));
    Serial.print(g_motion.output());
    Serial.print(F(" I="));
    Serial.print(g_motion.amps(), 3);
    Serial.print(F("A bus="));
    Serial.print(g_motion.busVolts(), 2);
    Serial.print(F("V"));
    if (g_motion.state() == MotionState::Fault) {
      Serial.print(F(" fault="));
      Serial.print(faultName(g_motion.fault()));
    }
    Serial.println();
  }
}
