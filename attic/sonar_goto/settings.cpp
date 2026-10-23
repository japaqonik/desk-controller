// =============================================================================
// settings.cpp (plan 8.3)
// =============================================================================
#include "settings.h"
#include <EEPROM.h>

namespace {
constexpr uint16_t kMagic   = 0xDE5C;  // "DESC"
constexpr uint8_t  kVersion = 1;
constexpr int      kEepromAddr = 0;    // base address in EEPROM
}  // namespace

uint16_t Settings::crc16(const uint8_t* p, uint8_t len) {
  // CRC-16/CCITT (0x1021), seed 0xFFFF. Small and sufficient for integrity.
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(p[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void Settings::begin() {
  Data d{};
  EEPROM.get(kEepromAddr, d);

  const uint16_t crc = crc16(reinterpret_cast<const uint8_t*>(&d),
                             sizeof(Data) - sizeof(d.crc));
  if (d.magic == kMagic && d.version == kVersion && d.crc == crc) {
    data_ = d;  // valid
  } else {
    // Defaults: everything unset.
    data_.magic   = kMagic;
    data_.version = kVersion;
    for (uint8_t i = 0; i < kPresetCount; ++i) data_.preset[i] = 0;
    data_.crc = crc16(reinterpret_cast<const uint8_t*>(&data_),
                      sizeof(Data) - sizeof(data_.crc));
    // Do not write defaults to EEPROM here; only explicit saves persist.
  }
}

void Settings::writeToEeprom() {
  data_.magic   = kMagic;
  data_.version = kVersion;
  data_.crc = crc16(reinterpret_cast<const uint8_t*>(&data_),
                    sizeof(Data) - sizeof(data_.crc));
  EEPROM.put(kEepromAddr, data_);  // EEPROM.put uses update() internally
}

void Settings::savePreset(uint8_t i, uint16_t mm) {
  if (i >= kPresetCount) return;
  data_.preset[i] = mm;
  writeToEeprom();
}
