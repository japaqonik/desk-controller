// =============================================================================
// HC-SR04 distance sensor test
// =============================================================================
// Logs the filtered distance, validity and reading age over Serial. No motor
// control - safe. Use it to check range and repeatability at the desk's lowest
// and highest positions before building presets on top of it.
//
// Wiring: TRIG=D12, ECHO=D11, VCC=5V, GND common.
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "distance_sensor.h"

namespace {
DistanceSensor g_sonar;
unsigned long g_lastTick = 0;
unsigned long g_lastLog  = 0;
}  // namespace

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);

  g_sonar.begin();
  g_lastTick = millis();

  Serial.println();
  Serial.println(F("=== HC-SR04 distance sensor test ==="));
  Serial.println(F("TRIG=D12 ECHO=D11. Logging distance [mm]."));
  Serial.println(F("Check reading at lowest and highest desk position."));

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastTick) < cfg::kTickMs) return;
  g_lastTick = now;

  g_sonar.update();

  digitalWrite(cfg::kPinLed, g_sonar.isValid() ? HIGH : LOW);

  if ((now - g_lastLog) >= 100) {
    g_lastLog = now;
    Serial.print(F("dist="));
    Serial.print(g_sonar.distanceMm());
    Serial.print(F("mm valid="));
    Serial.print(g_sonar.isValid() ? F("yes") : F("no "));
    Serial.print(F(" age="));
    Serial.print(g_sonar.ageMs());
    Serial.print(F("ms echo="));
    Serial.print(g_sonar.lastEchoUs());
    Serial.print(F("us echoPin="));
    Serial.print(digitalRead(cfg::kPinSonarEcho));
    Serial.println();
  }
}
