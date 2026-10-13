// =============================================================================
// Step 0: button diagnostics
// =============================================================================
// Goal: confirm the pin map and polarity BEFORE running the motor.
//
// What it does:
//   - configures D2..D5 as INPUT_PULLUP (button shorts the pin to GND),
//   - simple debounce,
//   - prints PRESS / RELEASE events with hold time over Serial,
//   - lights the on-board LED (D13) while any button is pressed.
//
// This sketch does NOT drive the motor and is completely safe.
//
// How to read the output:
//   Press the physical UP button -> you should see "D2 (UP)   PRESS".
//   If the label is wrong (e.g. UP shows as D3) -> fix the pin map in Config.h.
//   If a button reports PRESS without being touched -> pullup / wiring issue.
// =============================================================================

#include <Arduino.h>

namespace {

// --- Pin map per plan (section 2.1) ---
struct ButtonDef {
  uint8_t     pin;
  const char* name;
};

constexpr ButtonDef kButtons[] = {
    {2, "D2 (UP)  "},
    {3, "D3 (DOWN)"},
    {4, "D4 (P1)  "},
    {5, "D5 (FN)  "},
};
constexpr uint8_t kButtonCount = sizeof(kButtons) / sizeof(kButtons[0]);

constexpr uint8_t  kLedPin        = 13;
constexpr uint16_t kDebounceMs    = 25;   // 20-30 ms per plan
constexpr uint16_t kReportEveryMs = 2000; // periodic heartbeat + state report

// Debounce state for each button.
struct ButtonState {
  bool          stable;        // true = pressed (after debounce)
  bool          lastRaw;       // last raw read (true = pressed)
  unsigned long lastChangeMs;  // when the raw read last changed
  unsigned long pressStartMs;  // when it became stably pressed
};

ButtonState g_state[kButtonCount];
unsigned long g_lastHeartbeat = 0;
unsigned long g_loops = 0;  // loop iteration counter (proof the current build runs)

// Pressed => pin LOW (INPUT_PULLUP + short to GND).
inline bool readPressed(uint8_t pin) { return digitalRead(pin) == LOW; }

}  // namespace

void setup() {
  Serial.begin(115200);
  // Short window to open the monitor (no delay-loop in production logic,
  // but fine here in diagnostics).
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {
    // wait for USB CDC if applicable (Nano usually works immediately)
  }

  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);

  for (uint8_t i = 0; i < kButtonCount; ++i) {
    pinMode(kButtons[i].pin, INPUT_PULLUP);
    g_state[i].stable       = false;
    g_state[i].lastRaw      = false;
    g_state[i].lastChangeMs = 0;
    g_state[i].pressStartMs = 0;
  }

  Serial.println();
  Serial.println(F("=== Step 0: button diagnostics ==="));
  Serial.println(F("Buttons: INPUT_PULLUP, other leg to GND."));
  Serial.println(F("Press each button in turn and check the label."));
  Serial.println(F("Expected: UP=D2, DOWN=D3, P1=D4, FN=D5."));
  Serial.println(F("-------------------------------------------"));
}

void loop() {
  const unsigned long now = millis();

  for (uint8_t i = 0; i < kButtonCount; ++i) {
    ButtonState& s      = g_state[i];
    const bool   raw    = readPressed(kButtons[i].pin);

    if (raw != s.lastRaw) {
      s.lastRaw      = raw;
      s.lastChangeMs = now;  // reset debounce window
    }

    // Treat as stable when the raw read held for > kDebounceMs.
    if ((now - s.lastChangeMs) >= kDebounceMs && raw != s.stable) {
      s.stable = raw;
      if (s.stable) {
        s.pressStartMs = now;
        Serial.print(F("  "));
        Serial.print(kButtons[i].name);
        Serial.println(F("  PRESS"));
      } else {
        const unsigned long held = now - s.pressStartMs;
        Serial.print(F("  "));
        Serial.print(kButtons[i].name);
        Serial.print(F("  RELEASE  (held "));
        Serial.print(held);
        Serial.println(F(" ms)"));
      }
    }
  }

  // LED: on while anything is pressed.
  bool anyPressed = false;
  for (uint8_t i = 0; i < kButtonCount; ++i) {
    if (g_state[i].stable) { anyPressed = true; break; }
  }
  digitalWrite(kLedPin, anyPressed ? HIGH : LOW);

  // Heartbeat: every 2 s print RAW pin reads + debounced state + loop counter.
  // RAW = direct digitalRead (0=LOW/shorted to GND, 1=HIGH/pullup).
  // With a released button RAW must be 1. If RAW=0 without pressing -> hardware.
  if ((now - g_lastHeartbeat) >= kReportEveryMs) {
    g_lastHeartbeat = now;
    Serial.print(F("[state] loops="));
    Serial.print(g_loops);
    Serial.print(F("  "));
    for (uint8_t i = 0; i < kButtonCount; ++i) {
      const int rawLevel = digitalRead(kButtons[i].pin);  // 0 or 1
      Serial.print(kButtons[i].name);
      Serial.print(F(" raw="));
      Serial.print(rawLevel);
      Serial.print(g_state[i].stable ? F("/DOWN  ") : F("/up    "));
    }
    Serial.println();
  }
  ++g_loops;
}
