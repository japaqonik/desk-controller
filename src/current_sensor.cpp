// =============================================================================
// current_sensor.cpp - INA219 implementation
// =============================================================================
#include "current_sensor.h"
#include <Wire.h>
#include <INA219.h>

using namespace cfg;

namespace {
INA219 g_ina(kInaAddress);
}  // namespace

bool CurrentSensor::begin() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  // Prevent an I2C glitch from hanging the control loop.
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

  if (!g_ina.begin() || !g_ina.isConnected()) {
    present_ = false;
    return false;
  }
  g_ina.setMaxCurrentShunt(kInaMaxCurrentA, kInaShuntOhms);
  present_ = true;
  emaA_    = 0.0f;
  return true;
}

float CurrentSensor::update(int8_t /*dir*/) {
  if (!present_) {
    emaA_ = 0.0f;
    return emaA_;
  }

  // Magnitude of current (sign depends on H-bridge direction / wiring).
  float a = g_ina.getCurrent();
  if (a < 0.0f) a = -a;

  // Saturation: at/near full scale the real current exceeds what this shunt can
  // measure. Flag it so the controller can treat it as an overcurrent fault.
  saturated_ = (a >= (kInaMaxCurrentA - 0.1f));

  emaA_ += kCurrentEmaAlpha * (a - emaA_);
  busV_  = g_ina.getBusVoltage();
  return emaA_;
}
