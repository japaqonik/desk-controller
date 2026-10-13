// =============================================================================
// config.h - pins, timing constants and thresholds (plan sections 2.1, 4, 5)
// =============================================================================
// Everything as constexpr. This is the single place for manual tuning.
// =============================================================================
#pragma once
#include <Arduino.h>

namespace cfg {

// --- Button pins (confirmed on hardware in Step 0) ---
constexpr uint8_t kPinBtnUp   = 2;   // D2
constexpr uint8_t kPinBtnDown = 3;   // D3
constexpr uint8_t kPinBtnP1   = 4;   // D4
constexpr uint8_t kPinBtnFn   = 5;   // D5

// --- IBT-2 (BTS7960) pins ---
constexpr uint8_t kPinRpwm = 9;    // Timer1 OC1A
constexpr uint8_t kPinLpwm = 10;   // Timer1 OC1B
constexpr uint8_t kPinRen  = 7;    // R_EN
constexpr uint8_t kPinLen  = 8;    // L_EN
constexpr uint8_t kPinRis  = A2;   // current sense, right half-bridge
constexpr uint8_t kPinLis  = A1;   // current sense, left half-bridge

constexpr uint8_t kPinLed = 13;    // on-board LED

// --- PWM (Timer1, fast PWM, TOP = ICR1) ---
// 16 MHz / (ICR1+1) = frequency. 16e6/800 = 20 kHz.
constexpr uint16_t kPwmTop = 799;        // duty 0..799
constexpr int16_t  kPwmMax = kPwmTop;    // maximum duty (full speed)

// --- "Up" direction ---
// Positive setOutput() drives the RPWM channel. Whether that is physically
// "up" is decided on the bench. Flipping the sign reverses direction without
// rewiring.
constexpr int8_t kUpSign = +1;   // +1 or -1

// --- Dead-time on direction change (plan 5.1) ---
constexpr uint16_t kDeadTimeMs = 5;

// --- Ramp / slew-rate (plan 5.2) ---
// Duty units per millisecond. 799 / 700ms ~= 1.14.
// Ramp up ~0.7 s, faster ramp down.
constexpr float kRampUpPerMs   = 1.15f;   // 0..max in ~0.7 s
constexpr float kRampDownPerMs = 2.60f;   // down to 0 in ~0.3 s

// --- Test speed limit ---
// --- Default speed limit (PWM duty) ---
// Max speed used by the final firmware (main). The motor_test tool lowers it at
// runtime via setSpeedLimit(). Current sensing is clean (INA219), so 100% is OK.
constexpr int16_t kTestSpeedLimit = static_cast<int16_t>(kPwmMax * 1.00f);

// --- Control loop ---
constexpr uint16_t kTickMs = 5;   // fixed 5 ms tick (software)

// --- CurrentSensor (INA219) ---
// Current is read digitally from the INA219 (see INA219 section below). Only a
// light EMA is applied in software; the INA219 reading is already clean.
constexpr float    kCurrentEmaAlpha   = 0.20f;  // EMA filter (0..1), lower = smoother

// Current protection master switch. Keep FALSE while free-running / uncalibrated
// so an unexpected reading cannot trigger a false FAULT. Set TRUE once the
// thresholds are verified under load (before desk mounting).
constexpr bool     kCurrentProtectionEnabled = true;

// --- Protection thresholds (plan 5.4) ---
// Based on measured INA219 currents (free-running, 100% speed):
//   cruise ~0.6-0.76 A; hard finger braking barely reached ~0.8 A (the gearbox
//   masks load). Thresholds are set low so a real jam is still detected.
constexpr uint16_t kBlankingMs        = 400;  // ignore current for first 400 ms (covers inrush)
constexpr float    kCurrentSustainedA = 1.2f; // sustained limit
constexpr uint16_t kSustainedMs       = 200;  // ... held for > 200 ms
constexpr float    kCurrentInstantA   = 1.5f; // instantaneous limit
constexpr uint8_t  kOcDebounceTicks   = 3;    // consecutive over-threshold ticks before fault
constexpr uint32_t kMaxRunMs          = 0;    // run-time limit, 0 = disabled (final fw). Tools set their own.

// --- Watchdog ---
// WDTO_250MS enabled in code; reset every tick.

// NOTE: height-sensor (VL53L0X) constants live with the archived module in
// attic/height_sensor/. Removed from active config since no build uses them.

// --- Current sensor: INA219 (I2C) ---
// Replaces the noisy IBT-2 IS line. Standard module has a 0.1 ohm shunt ->
// +-3.2 A range at the default PGA gain. That is enough for protection if the
// normal running current stays below it; a reading at/above full scale is treated
// as "too much current -> fault". Calibrate with an ammeter if needed.
constexpr uint8_t kInaAddress     = 0x40;  // default; check with i2c_scan
constexpr float   kInaShuntOhms   = 0.100f; // module shunt resistor
constexpr float   kInaMaxCurrentA = 3.2f;   // full-scale with 0.1 ohm @ gain 8

}  // namespace cfg
