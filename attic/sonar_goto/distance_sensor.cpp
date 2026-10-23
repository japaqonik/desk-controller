// =============================================================================
// distance_sensor.cpp - HC-SR04 (non-blocking, pin-change interrupt on D11)
// =============================================================================
#include "distance_sensor.h"

using namespace cfg;

namespace {
constexpr uint16_t kMeasureGapMs = 60;     // >= gap between pings (avoid echoes)
constexpr float    kUsToMm       = 0.1715f; // echo us -> mm (343 m/s, round trip)

// ISR-shared state. D11 = PCINT3 on port B (PCINT0_vect).
volatile uint32_t g_echoRiseUs   = 0;
volatile uint32_t g_echoWidthUs  = 0;   // last completed echo width (us)
volatile bool     g_echoReady    = false;
volatile bool     g_echoHigh     = false;
volatile uint8_t  g_lastPortB    = 0;   // previous port B snapshot

// Bit mask of the ECHO pin within port B. D8..D13 = PB0..PB5, so D11 = PB3.
constexpr uint8_t kEchoBit = _BV(PB3);   // D11
}  // namespace

// Pin-change interrupt for port B (PCINT0..7). Masked to ECHO (D11) only, but
// the ISR still fires for the whole port, so we detect the ECHO edge ourselves.
ISR(PCINT0_vect) {
  const uint8_t portb = PINB;
  const uint8_t changed = portb ^ g_lastPortB;
  g_lastPortB = portb;
  if (changed & kEchoBit) {
    const uint32_t now = micros();
    if (portb & kEchoBit) {        // rising edge: echo started
      g_echoRiseUs = now;
      g_echoHigh   = true;
    } else if (g_echoHigh) {       // falling edge: echo finished
      g_echoWidthUs = now - g_echoRiseUs;
      g_echoHigh    = false;
      g_echoReady   = true;
    }
  }
}

void DistanceSensor::begin() {
  pinMode(kPinSonarTrig, OUTPUT);
  digitalWrite(kPinSonarTrig, LOW);
  pinMode(kPinSonarEcho, INPUT);

  // Enable pin-change interrupt on D11 (PCINT3) only.
  noInterrupts();
  g_lastPortB = PINB;
  PCMSK0 = kEchoBit;      // only PB3 (D11) triggers
  PCIFR |= _BV(PCIF0);    // clear any pending flag
  PCICR |= _BV(PCIE0);    // enable port B pin-change interrupts
  interrupts();

  valid_      = false;
  filteredMm_ = 0;
  windowFill_ = 0;
  windowPos_  = 0;
  emaInit_    = false;
  pingPending_ = false;
}

uint32_t DistanceSensor::lastEchoUs() const {
  noInterrupts();
  const uint32_t w = g_echoWidthUs;
  interrupts();
  return w;
}

uint16_t DistanceSensor::medianMm(uint16_t sample) {
  window_[windowPos_] = sample;
  windowPos_ = static_cast<uint8_t>((windowPos_ + 1) % kSonarMedianN);
  if (windowFill_ < kSonarMedianN) ++windowFill_;

  uint16_t tmp[kSonarMedianN];
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

void DistanceSensor::acceptReading(uint16_t mm) {
  if (mm < kSonarMinMm || mm > kSonarMaxMm) {
    valid_ = false;
    return;
  }
  const uint16_t med = medianMm(mm);
  if (!emaInit_) { ema_ = static_cast<float>(med); emaInit_ = true; }
  else           { ema_ += kSonarEmaAlpha * (static_cast<float>(med) - ema_); }
  filteredMm_ = static_cast<uint16_t>(ema_ + 0.5f);
  lastGoodMs_ = millis();
  valid_      = true;
}

void DistanceSensor::trigger() {
  // 10 us trigger pulse (negligible blocking).
  digitalWrite(kPinSonarTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kPinSonarTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kPinSonarTrig, LOW);
  lastTrigMs_  = millis();
  pingPending_ = true;
}

void DistanceSensor::update() {
  // Consume a completed echo captured by the ISR.
  if (g_echoReady) {
    noInterrupts();
    const uint32_t widthUs = g_echoWidthUs;
    g_echoReady = false;
    interrupts();
    pingPending_ = false;
    acceptReading(static_cast<uint16_t>(static_cast<float>(widthUs) * kUsToMm));
  }

  const uint32_t now = millis();

  // If a ping was sent but no echo returned within a sensible time, mark stale
  // and allow a new ping (out of range / lost echo).
  if (pingPending_ && (now - lastTrigMs_) > (kSonarTimeoutUs / 1000 + 5)) {
    pingPending_ = false;
    valid_ = false;
  }

  // Trigger a new ping after the inter-ping gap, when none is in flight.
  if (!pingPending_ && (now - lastTrigMs_) >= kMeasureGapMs) {
    trigger();
  }
}
