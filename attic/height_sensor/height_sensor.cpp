// =============================================================================
// height_sensor.cpp (plan 6)
// =============================================================================
#include "height_sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

using namespace cfg;

namespace {
VL53L0X g_vl;
}  // namespace

bool HeightSensor::begin() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  // Prevent an I2C glitch from hanging the control loop (plan 6.1).
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

  g_vl.setTimeout(kVlTimeoutMs);
  if (!g_vl.init()) {
    present_ = false;
    valid_   = false;
    return false;
  }

  // Long-range profile: lower the return signal rate limit and widen pulse /
  // final ranging VCSEL periods (per Pololu long-range example).
  g_vl.setSignalRateLimit(0.1f);
  g_vl.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  g_vl.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  g_vl.setMeasurementTimingBudget(kVlTimingBudgetUs);
  g_vl.startContinuous();  // free-running continuous mode

  present_ = true;
  valid_   = false;
  return true;
}

uint16_t HeightSensor::medianMm(uint16_t sample) {
  window_[windowPos_] = sample;
  windowPos_ = static_cast<uint8_t>((windowPos_ + 1) % kHeightMedianN);
  if (windowFill_ < kHeightMedianN) ++windowFill_;

  uint16_t tmp[kHeightMedianN];
  for (uint8_t i = 0; i < windowFill_; ++i) tmp[i] = window_[i];
  for (uint8_t i = 1; i < windowFill_; ++i) {
    const uint16_t key = tmp[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = key;
  }
  return tmp[windowFill_ / 2];
}

void HeightSensor::update() {
  if (!present_) return;

  // Non-blocking data-ready check: poll RESULT_INTERRUPT_STATUS instead of the
  // library's blocking readRangeContinuousMillimeters() (which would stall the
  // control loop waiting for a sample). Bits [2:0] != 0 means a sample is ready.
  const uint8_t status = g_vl.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (g_vl.timeoutOccurred() || (status & 0x07) == 0) {
    return;  // no sample yet (or I2C hiccup) - try again next tick
  }

  // Read the ranging result (16-bit) and clear the interrupt.
  const uint16_t raw = g_vl.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  g_vl.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  if (g_vl.timeoutOccurred()) {
    valid_ = false;
    return;
  }

  // Range plausibility (plan 6.3): discard out-of-range readings. The VL53L0X
  // reports ~8190/8191 for out-of-range; our window bounds also catch those.
  if (raw < kHeightValidMinMm || raw > kHeightValidMaxMm) {
    valid_ = false;
    return;
  }

  // Reject readings that jump faster than physically possible between samples
  // (plan 6.3). Only applies once we have a reference good value.
  if (lastGoodMs_ != 0) {
    const int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(lastGoodMm_);
    const uint16_t absDelta = static_cast<uint16_t>(delta < 0 ? -delta : delta);
    if (absDelta > kHeightMaxStepMm) {
      // Treat as a glitch/obstruction: ignore, keep last good value.
      valid_ = false;
      return;
    }
  }

  // Passed checks: feed the filters.
  const uint16_t med = medianMm(raw);
  if (!emaInit_) {
    ema_     = static_cast<float>(med);
    emaInit_ = true;
  } else {
    ema_ += kHeightEmaAlpha * (static_cast<float>(med) - ema_);
  }

  filteredMm_ = static_cast<uint16_t>(ema_ + 0.5f);
  lastGoodMm_ = raw;
  lastGoodMs_ = millis();
  valid_      = true;
}
