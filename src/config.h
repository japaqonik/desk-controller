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
// TEMPORARILY lowered while diagnosing the IS current reading. Will return to
// 100% once the IS->amps scale is calibrated (needs an RC filter on IS lines).
constexpr int16_t kTestSpeedLimit = static_cast<int16_t>(kPwmMax * 0.40f);

// --- Control loop ---
constexpr uint16_t kTickMs = 5;   // fixed 5 ms tick (software)

// --- CurrentSensor (plan 5.3) ---
// IBT-2 gives ~0.1 V/A on the IS line. ADC: 5V / 1024 = 4.887 mV/step.
// This is a STARTING value - to be calibrated with an ammeter (plan test 3).
// NOTE: with the RC filter (4.7k + 100nF) the zero reading is a stable offset
// (~35 counts observed) and must be SUBTRACTED. The real signal from a small
// free-running current (~0.2 A) is near the noise floor, so IS is only reliable
// at larger currents (jam / end-stop). Calibrate kIsVoltsPerAmp under load.
constexpr uint8_t  kCurrentOversample = 16;     // samples per averaged read
constexpr uint8_t  kCurrentMedianN    = 5;      // median window over averaged reads
constexpr uint16_t kIsZeroCounts      = 35;     // ADC counts at ~zero current (offset)
constexpr float    kAdcVoltsPerStep   = 5.0f / 1024.0f;
constexpr float    kIsVoltsPerAmp     = 0.100f; // TO BE CALIBRATED under load
constexpr float    kCurrentEmaAlpha   = 0.20f;  // EMA filter (0..1), lower = smoother

// Current protection master switch. Keep FALSE while free-running / uncalibrated
// so the near-noise IS reading cannot trigger a false FAULT. Set TRUE once the
// IS->amps scale and thresholds are calibrated under load (before desk mounting).
constexpr bool     kCurrentProtectionEnabled = false;

// --- Protection thresholds (plan 5.4) ---
// PSU max 8.33 A -> keep thresholds BELOW it so we trip before the PSU folds.
// Motor stall ~19 A, but the PSU cannot deliver that. Free-running draw is low.
constexpr uint16_t kBlankingMs        = 300;  // ignore current for first 300 ms
constexpr float    kCurrentSustainedA = 5.0f; // sustained limit
constexpr uint16_t kSustainedMs       = 200;  // ... held for > 200 ms
constexpr float    kCurrentInstantA   = 6.5f; // instantaneous limit
constexpr uint32_t kMaxRunMs          = 5000; // max single-move time (test: 5 s)

// --- Watchdog ---
// WDTO_250MS enabled in code; reset every tick.

// NOTE: height-sensor (VL53L0X) constants live with the archived module in
// attic/height_sensor/. Removed from active config since no build uses them.

}  // namespace cfg
