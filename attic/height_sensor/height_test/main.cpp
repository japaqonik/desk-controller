// =============================================================================
// Phase 2: height sensor test
// =============================================================================
// Reads the VL53L0X continuously and logs the filtered distance, validity and
// reading age over Serial. No motor control - safe to run with the sensor
// pointed at the floor.
//
// Use this to:
//   - confirm the sensor initialises,
//   - log distances at the lowest and highest desk position (plan test 1, 5),
//   - check stability over the real floor before setting soft limits.
//
// Wiring: SDA=A4, SCL=A5, power from 5V/VIN, common GND.
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "height_sensor.h"

namespace {
HeightSensor g_height;
unsigned long g_lastTick = 0;
unsigned long g_lastLog  = 0;
bool g_ok = false;
}  // namespace

void setup() {
  // Disable the watchdog early (see motor_test for the bootloader-loop rationale).
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  pinMode(cfg::kPinLed, OUTPUT);

  Serial.println();
  Serial.println(F("=== Phase 2: height sensor test ==="));

  g_ok = g_height.begin();
  if (!g_ok) {
    Serial.println(F("VL53L0X init FAILED - check wiring (SDA=A4, SCL=A5, 5V, GND)."));
  } else {
    Serial.println(F("VL53L0X OK, continuous ranging. Logging distance [mm]."));
  }

  g_lastTick = millis();
  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastTick) < cfg::kTickMs) {
    return;
  }
  g_lastTick = now;

  g_height.update();

  // LED: on when the current reading is valid.
  digitalWrite(cfg::kPinLed, g_height.isValid() ? HIGH : LOW);

  if ((now - g_lastLog) >= 100) {
    g_lastLog = now;
    if (!g_ok) {
      Serial.println(F("no sensor"));
      return;
    }
    Serial.print(F("dist="));
    Serial.print(g_height.heightMm());
    Serial.print(F("mm valid="));
    Serial.print(g_height.isValid() ? F("yes") : F("no "));
    Serial.print(F(" age="));
    Serial.print(g_height.ageMs());
    Serial.println(F("ms"));
  }
}
