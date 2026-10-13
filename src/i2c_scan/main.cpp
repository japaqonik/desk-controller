// =============================================================================
// Tool: I2C scanner + sensor model check (plan section 2.3)
// =============================================================================
// What it does:
//   1. Scans the I2C bus and prints addresses that respond.
//      The VL53L0X should respond at 0x29.
//   2. If a device exists at 0x29, reads register 0xC0 (Model ID).
//      The VL53L0X returns 0xEE.
//
// Wiring: SDA=A4, SCL=A5 (Nano hardware I2C). Power the module from 5V/VIN.
// This tool does NOT drive the motor - safe.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kVl53Addr    = 0x29;
constexpr uint8_t kModelIdReg  = 0xC0;
constexpr uint8_t kVl53ModelId = 0xEE;  // VL53L0X
}  // namespace

static void scanBus() {
  Serial.println(F("--- I2C scan ---"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("  device @ 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == kVl53Addr) Serial.print(F("  <- expected VL53L0X"));
      Serial.println();
      ++found;
    }
  }
  if (found == 0) {
    Serial.println(F("  NO devices! Check SDA/SCL/power/GND."));
  }
}

static void checkModelId() {
  Wire.beginTransmission(kVl53Addr);
  Wire.write(kModelIdReg);
  if (Wire.endTransmission(false) != 0) {
    Serial.println(F("  no response at @0x29 - skipping Model ID read"));
    return;
  }
  if (Wire.requestFrom(kVl53Addr, static_cast<uint8_t>(1)) != 1) {
    Serial.println(F("  failed to read Model ID"));
    return;
  }
  const uint8_t id = static_cast<uint8_t>(Wire.read());
  Serial.print(F("  Model ID (0xC0) = 0x"));
  Serial.print(id, HEX);
  if (id == kVl53ModelId) {
    Serial.println(F("  -> VL53L0X OK"));
  } else {
    Serial.println(F("  -> WARNING: not 0xEE (different model or read error)"));
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000 /* us */, true /* reset_on_timeout */);
#endif

  Serial.println();
  Serial.println(F("=== I2C scanner + sensor ID ==="));
}

void loop() {
  scanBus();
  checkModelId();
  Serial.println(F("(rescan in 3 s)"));
  Serial.println();

  // In a diagnostic tool a delay is acceptable.
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {}
}
