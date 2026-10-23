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
// Set false to keep the on-board LED (D13) dark (e.g. inside a closed case).
// The PWR and RX/TX LEDs are hardwired and cannot be disabled in firmware.
constexpr bool    kLedEnabled = false;

// --- Distance sensor: HC-SR04 ultrasonic ---
// TRIG = D12 (output pulse), ECHO = D11 (input, pulse width ~ distance).
// Distance[mm] = echo_us * 0.343 / 2  (speed of sound ~343 m/s).
// Ultrasonic is immune to floor color (unlike ToF) but can be noisy / lose echo
// on soft or angled surfaces - filtering and validity checks matter.
constexpr uint8_t  kPinSonarTrig   = 12;
constexpr uint8_t  kPinSonarEcho   = 11;
constexpr uint16_t kSonarTimeoutUs = 25000;  // ~4.3 m max wait; no-echo => invalid
constexpr uint8_t  kSonarMedianN   = 5;      // median window
constexpr float    kSonarEmaAlpha  = 0.40f;  // light EMA after median
constexpr uint16_t kSonarMinMm     = 20;     // discard readings below this
constexpr uint16_t kSonarMaxMm     = 4000;   // discard readings above this

// --- PWM (Timer1, fast PWM, TOP = ICR1) ---
// 16 MHz / (ICR1+1) = frequency. 16e6/800 = 20 kHz.
constexpr uint16_t kPwmTop = 799;        // duty 0..799
constexpr int16_t  kPwmMax = kPwmTop;    // maximum duty (full speed)

// --- "Up" direction ---
// Positive setOutput() drives the RPWM channel. Whether that is physically
// "up" is decided on the bench. Flipping the sign reverses direction without
// rewiring.
constexpr int8_t kUpSign = -1;   // +1 or -1 (bench: desk goes UP at negative out)

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
// Separate limits for up vs down: lifting fights gravity (high current),
// lowering is gravity-assisted (lower current). Blanking covers the ~0.7 s ramp
// so start-up inrush does not trip the limit.
constexpr uint16_t kBlankingMs        = 1000; // ignore current for first 1 s (covers ramp + inrush)
constexpr uint16_t kSustainedMs       = 300;  // sustained limit must hold > this
constexpr uint8_t  kOcDebounceTicks   = 3;    // consecutive over-threshold ticks before fault

// Up (lifting) - higher (lifting fights gravity).
constexpr float    kCurrentSustainedUpA = 4.5f;
constexpr float    kCurrentInstantUpA   = 5.0f;
// Down (lowering) - tighter (gravity-assisted, lower normal current).
constexpr float    kCurrentSustainedDownA = 3.5f;
constexpr float    kCurrentInstantDownA   = 4.0f;

// Undervoltage: if the INA219 bus voltage sags below this while moving (after
// the blanking window), the supply cannot deliver the demanded current -> fault.
// Stops the motor from stalling against a drooping rail (heating, no motion).
// 0 = disabled. Set below the nominal 12 V but above a normal loaded dip.
constexpr float    kUnderVoltageV     = 9.0f;

constexpr uint32_t kMaxRunMs          = 0;    // run-time limit, 0 = disabled (final fw). Tools set their own.

// --- Watchdog ---
// WDTO_250MS enabled in code; reset every tick.

// NOTE: height-sensor (VL53L0X) constants live with the archived module in
// attic/height_sensor/. Removed from active config since no build uses them.

// --- Current sensor: INA219 (I2C) ---
// Replaces the noisy IBT-2 IS line. The module's shunt was swapped to a 0.02 ohm
// (Viking LRP, R020, 1%, 50ppm) current-sense resistor to widen the range:
//   max shunt voltage 320 mV (gain 8) / 0.02 ohm = +-16 A hardware range.
// kInaMaxCurrentA sets the library LSB (resolution); keep it above the PSU
// capability (8.33 A) with margin. kInaSaturationA is the "beyond measurable /
// too much current" safety threshold used by the controller.
constexpr uint8_t kInaAddress     = 0x40;   // default; check with i2c_scan
constexpr float   kInaShuntOhms   = 0.020f; // R020 current-sense resistor
constexpr float   kInaMaxCurrentA = 10.0f;  // measurement range for LSB (> PSU 8.33 A)
constexpr float   kInaSaturationA = 9.0f;   // treat >= this as overcurrent (below PSU max)

}  // namespace cfg
