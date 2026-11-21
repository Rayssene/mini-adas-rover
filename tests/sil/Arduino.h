// Fake Arduino for the software-in-the-loop (SIL) test.
// It simulates time, the HC-SR04 sensor and records what the sketch writes
// to the pins, so the REAL sketch (rover.ino) can run on a PC.
#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H
#include <stdint.h>
#include <stdio.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define A0 14
#define A5 19

namespace sim {
static unsigned long long timeUs = 0;    // simulated time
static float obstacleCm = 150.0f;        // distance in front of the sensor
static bool echoConnected = true;        // false = ECHO wire unplugged
static int pinLevel[20] = {0};           // last digitalWrite per pin
static int pinPwm[20] = {0};             // last analogWrite per pin
static int trigLevel = 0;
static unsigned long long trigFallUs = 0;
static bool measuring = false;
static const int TRIG = 4, ECHO = 3;

inline void advance(unsigned long us) { timeUs += us; }
}  // namespace sim

inline void pinMode(uint8_t, uint8_t) {}
inline void delayMicroseconds(unsigned int us) { sim::advance(us); }
inline unsigned long micros() { sim::advance(1); return (unsigned long)sim::timeUs; }
inline unsigned long millis() { return (unsigned long)(sim::timeUs / 1000); }

inline void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin == sim::TRIG) {
    if (sim::trigLevel == HIGH && value == LOW) {  // falling edge starts a measurement
      sim::trigFallUs = sim::timeUs;
      sim::measuring = true;
    }
    sim::trigLevel = value;
  }
  if (pin < 20) sim::pinLevel[pin] = value;
}

inline void analogWrite(uint8_t pin, int value) {
  if (pin < 20) sim::pinPwm[pin] = value;
}

// HC-SR04 model: ECHO goes HIGH ~400 us after the trigger and stays HIGH for
// the round-trip time. Beyond ~400 cm the pulse is ~38 ms long (no echo).
inline int digitalRead(uint8_t pin) {
  sim::advance(5);
  if (pin != sim::ECHO || !sim::echoConnected || !sim::measuring) return LOW;
  unsigned long long start = sim::trigFallUs + 400;
  unsigned long long length = (sim::obstacleCm > 400.0f)
      ? 38000ULL
      : (unsigned long long)(sim::obstacleCm * 2.0f / 0.0343f);
  return (sim::timeUs >= start && sim::timeUs < start + length) ? HIGH : LOW;
}

struct FakeSerial {
  void begin(long) {}
  void print(unsigned long) {}
  void print(int) {}
  void print(char) {}
  void print(const char*) {}
  void print(float, int) {}
  void println(const char*) {}
  void println(int) {}
};
static FakeSerial Serial;
#endif
