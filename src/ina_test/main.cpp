// =============================================================================
// INA219 current sensor test
// =============================================================================
// Verifies the INA219 and logs bus voltage, shunt voltage and current over
// Serial. No motor control here - safe. Use it to:
//   - confirm the INA219 answers on I2C,
//   - watch the zero-current reading (offset),
//   - calibrate against an ammeter under load.
//
// Wiring:
//   Vin+  <- +12V from PSU         Vin-  -> +12V to IBT-2   (series in +12V)
//   Vcc   <- Nano 5V               GND   -> common GND
//   SDA   -> A4                    SCL   -> A5
//
// A reading at/near the +-3.2 A full scale means the current exceeds what this
// shunt can measure (treated as a fault in the real firmware).
// =============================================================================
#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <INA219.h>

#include "config.h"

namespace {
INA219 g_ina(cfg::kInaAddress);
bool   g_ok = false;
unsigned long g_lastLog = 0;
}  // namespace

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  Serial.println();
  Serial.println(F("=== INA219 current sensor test ==="));

  if (!g_ina.begin() || !g_ina.isConnected()) {
    Serial.print(F("INA219 NOT found at 0x"));
    Serial.println(cfg::kInaAddress, HEX);
    Serial.println(F("Check wiring / address (run i2c_scan)."));
  } else {
    // Calibrate for the module shunt and expected max current.
    g_ina.setMaxCurrentShunt(cfg::kInaMaxCurrentA, cfg::kInaShuntOhms);
    g_ok = true;
    Serial.print(F("INA219 OK @0x"));
    Serial.print(cfg::kInaAddress, HEX);
    Serial.print(F("  shunt="));
    Serial.print(cfg::kInaShuntOhms, 3);
    Serial.print(F("ohm maxI="));
    Serial.print(cfg::kInaMaxCurrentA, 1);
    Serial.println(F("A"));
    Serial.println(F("Logging: bus[V] shunt[mV] current[A]"));
  }

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  const unsigned long now = millis();
  if ((now - g_lastLog) < 200) return;
  g_lastLog = now;

  if (!g_ok) {
    Serial.println(F("no INA219"));
    return;
  }

  const float busV   = g_ina.getBusVoltage();
  const float shuntmV = g_ina.getShuntVoltage_mV();
  const float curA   = g_ina.getCurrent();

  Serial.print(F("bus="));
  Serial.print(busV, 2);
  Serial.print(F("V shunt="));
  Serial.print(shuntmV, 2);
  Serial.print(F("mV I="));
  Serial.print(curA, 3);
  Serial.print(F("A"));
  if (fabsf(curA) >= cfg::kInaMaxCurrentA - 0.1f) {
    Serial.print(F("  <-- NEAR/OVER FULL SCALE"));
  }
  Serial.println();
}
