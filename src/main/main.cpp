// =============================================================================
// Desk Controller - main firmware (manual mode)
// =============================================================================
// Minimum viable build: manual up/down with soft start/stop and INA219
// overcurrent protection. No position sensing, no presets (see plan section 13).
//
// Controls:
//   UP (D2)   held -> drive "up"   (sign per cfg::kUpSign)
//   DOWN (D3) held -> drive "down"
//   both             -> stop
//   P1 / FN          -> unused in this build
//
// Serial is used only for a startup banner and fault reporting (no periodic log).
// Run-time timeout is disabled here (cfg::kMaxRunMs = 0); the user has a physical
// power switch. Overcurrent protection is active.
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
FaultReason   g_lastFault = FaultReason::None;

const char* faultName(FaultReason f) {
  switch (f) {
    case FaultReason::None:                 return "-";
    case FaultReason::OverCurrentInstant:   return "OC_INSTANT";
    case FaultReason::OverCurrentSustained: return "OC_SUSTAINED";
    case FaultReason::RunTimeout:           return "RUN_TIMEOUT";
    case FaultReason::UnderVoltage:         return "UNDERVOLTAGE";
  }
  return "?";
}
}  // namespace

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);
  digitalWrite(cfg::kPinLed, LOW);

  g_buttons.begin();
  g_motion.begin();          // uses cfg::kMaxRunMs (0 = no timeout) by default
  g_lastTick = millis();

  Serial.println();
  Serial.println(F("Desk Controller - manual mode"));
  if (!g_motion.currentPresent()) {
    Serial.println(F("WARN: INA219 not found - current protection inactive"));
  }

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastTick) < cfg::kTickMs) {
    return;
  }
  const uint16_t dt = static_cast<uint16_t>(now - g_lastTick);
  g_lastTick = now;

  g_buttons.update(dt);

  const bool up   = g_buttons.isDown(Btn::Up);
  const bool down = g_buttons.isDown(Btn::Down);
  int8_t reqDir = 0;
  if (up && !down)       reqDir = +1;
  else if (down && !up)  reqDir = -1;
  g_motion.requestDirection(reqDir);

  g_motion.tick(dt, g_buttons.anyDown());

  // LED: on while moving, fast blink in FAULT, off when idle. Disabled via
  // cfg::kLedEnabled (e.g. closed case).
  if (cfg::kLedEnabled) {
    if (g_motion.state() == MotionState::Fault) {
      digitalWrite(cfg::kPinLed, ((now / 100) % 2) ? HIGH : LOW);
    } else {
      digitalWrite(cfg::kPinLed, g_motion.state() == MotionState::Idle ? LOW : HIGH);
    }
  }

  // Report a fault once, when it occurs (error-only Serial output).
  const FaultReason f = g_motion.fault();
  if (f != g_lastFault) {
    if (f != FaultReason::None) {
      Serial.print(F("FAULT: "));
      Serial.print(faultName(f));
      Serial.print(F("  I="));
      Serial.print(g_motion.amps(), 3);
      Serial.println(F("A"));
    }
    g_lastFault = f;
  }
}
