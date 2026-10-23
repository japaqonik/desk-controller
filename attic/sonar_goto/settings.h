// =============================================================================
// settings.h - persistent presets in EEPROM (plan 8.3)
// =============================================================================
// Stores up to 4 preset heights (raw sonar distance in mm; 0 = unset), guarded
// by a magic number and CRC. On load: if magic/CRC mismatch, defaults are used
// (all presets unset). Writes happen only on an explicit save (EEPROM.update, so
// unchanged bytes are not rewritten).
// =============================================================================
#pragma once
#include <Arduino.h>

class Settings {
public:
  static constexpr uint8_t kPresetCount = 4;

  // Load from EEPROM; falls back to defaults (all unset) if invalid.
  void begin();

  // 0 = unset. Index 0..3.
  uint16_t preset(uint8_t i) const {
    return (i < kPresetCount) ? data_.preset[i] : 0;
  }
  bool hasPreset(uint8_t i) const { return preset(i) != 0; }

  // Set a preset in RAM and persist the whole block to EEPROM.
  void savePreset(uint8_t i, uint16_t mm);

private:
  struct Data {
    uint16_t magic;
    uint8_t  version;
    uint16_t preset[kPresetCount];
    uint16_t crc;
  };

  static uint16_t crc16(const uint8_t* p, uint8_t len);
  void writeToEeprom();

  Data data_{};
};
