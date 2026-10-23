// =============================================================================
// Preset test: one EEPROM preset on P1 (store / recall)
// =============================================================================
// Manual UP/DOWN as usual. P1 (D4):
//   - click (short press)      -> go to the stored preset height
//   - long press (>= 1.5 s)    -> save the current height as the preset
// The preset persists in EEPROM across power cycles. If the preset is unset, a
// click does nothing (logged). Any button cancels an automatic move.
//
// This verifies the preset store/recall + closed-loop path before expanding to
// 4 presets with FN and integrating into the final firmware.
//
// Verbose Serial log; watch "PRESET saved/go/unset" lines.
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "buttons.h"
#include "distance_sensor.h"
#include "motion_controller.h"
#include "settings.h"

namespace {
Buttons          g_buttons;
DistanceSensor   g_sonar;
MotionController  g_motion;
Settings          g_settings;

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
}  // namespace

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);

  g_buttons.begin();
  g_sonar.begin();
  g_motion.begin();
  g_settings.begin();
  g_lastTick = millis();

  Serial.println();
  Serial.println(F("=== preset_test (P1: click=go, hold=save) ==="));
  Serial.print(F("Preset #1: "));
  if (g_settings.hasPreset(0)) { Serial.print(g_settings.preset(0)); Serial.println(F(" mm")); }
  else                          { Serial.println(F("unset")); }

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
  g_motion.updateDistance(g_sonar.distanceMm(), g_sonar.isValid(), g_sonar.ageMs());

  // Manual direction.
  const bool up   = g_buttons.isDown(Btn::Up);
  const bool down = g_buttons.isDown(Btn::Down);
  int8_t reqDir = 0;
  if (up && !down)      reqDir = +1;
  else if (down && !up) reqDir = -1;
  g_motion.requestDirection(reqDir);

  // P1 long press -> save current height as preset #1.
  if (g_buttons.takeLongPress(Btn::P1)) {
    if (g_sonar.isValid()) {
      const uint16_t h = g_sonar.distanceMm();
      g_settings.savePreset(0, h);
      Serial.print(F("PRESET #1 saved: "));
      Serial.print(h);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("PRESET save REFUSED: sensor invalid"));
    }
  }

  // P1 click -> go to preset #1.
  if (g_buttons.takeClick(Btn::P1)) {
    if (g_settings.hasPreset(0)) {
      Serial.print(F("PRESET #1 go -> "));
      Serial.print(g_settings.preset(0));
      Serial.println(F(" mm"));
      g_motion.requestGoTo(g_settings.preset(0));
    } else {
      Serial.println(F("PRESET #1 unset"));
    }
  }

  g_motion.tick(dt, g_buttons.anyDown());

  if ((now - g_lastLog) >= 200) {
    g_lastLog = now;
    Serial.print(F("state="));
    Serial.print(stateName(g_motion.state()));
    Serial.print(F(" dist="));
    Serial.print(g_sonar.distanceMm());
    Serial.print(g_sonar.isValid() ? F("(ok)") : F("(--)"));
    Serial.print(F(" tgt="));
    Serial.print(g_motion.targetMm());
    Serial.print(F(" out="));
    Serial.print(g_motion.output());
    Serial.println();
  }
}
