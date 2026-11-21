// rover_logic.h
// Hardware-independent logic of the mini ADAS rover.
//
// Plain C++ without any Arduino function: it takes numbers in (time, distance)
// and gives numbers out (wheel speeds, head angle, LED, buzzer). The same file
// runs on the Arduino, in the Wokwi simulator and on a PC, where
// tests/test_logic.cpp checks it automatically.

#ifndef ROVER_LOGIC_H
#define ROVER_LOGIC_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Tuning parameters (requirement IDs in docs/requirements.md)
// ---------------------------------------------------------------------------

// Wheel speeds as PWM duty cycle, 0..255
const int16_t CRUISE_SPEED = 200;   // free road (REQ-03)
const int16_t CRAWL_SPEED = 110;    // slowest speed that still moves a TT motor reliably
const int16_t TURN_SPEED = 170;     // turning on the spot (REQ-04)
const int16_t REVERSE_SPEED = 140;  // backing out of a dead end (REQ-04)

// Distances in cm (REQ-03, REQ-04)
const float SLOW_DOWN_CM = 80.0f;   // below this the rover starts slowing down
const float STOP_CM = 25.0f;        // below this: emergency stop (AEB)
const float CLEAR_CM = 40.0f;       // a side counts as free above this distance

// Head (servo) angles in degrees. 90 = straight ahead. If your servo is
// mounted the other way round, swap HEAD_LEFT and HEAD_RIGHT.
const uint8_t HEAD_CENTER = 90;
const uint8_t HEAD_LEFT = 160;
const uint8_t HEAD_RIGHT = 20;

// Durations in ms
const unsigned long STARTUP_MS = 2000;     // stand still after power-on (REQ-07)
const unsigned long SELF_TEST_MS = 400;    // LED + buzzer test at power-on (REQ-07)
const unsigned long BRAKE_HOLD_MS = 300;   // stand still after an emergency stop
const unsigned long BRAKE_BEEP_MS = 150;   // warning beep at an emergency stop (REQ-06)
const unsigned long SETTLE_MS = 400;       // let the servo turn before trusting a reading
const unsigned long TURN_MS = 400;         // time-based turn (no wheel sensors yet)
const unsigned long REVERSE_MS = 500;      // time-based reverse

// Valid range of the HC-SR04 (REQ-05)
const float MIN_VALID_CM = 2.0f;
const float MAX_VALID_CM = 400.0f;

// ---------------------------------------------------------------------------
// Sensor helpers
// ---------------------------------------------------------------------------

// Echo time in microseconds -> distance in cm.
// Sound: ~0.0343 cm/us at 20 degC; the echo covers the way there and back.
inline float echoToCm(unsigned long echoUs) {
  return echoUs * 0.0343f / 2.0f;
}

inline bool isValidDistance(float cm) {
  return cm >= MIN_VALID_CM && cm <= MAX_VALID_CM;
}

// Result of one measurement (REQ-05). "No echo" and "no answer" are different:
//   ECHO_OUT_OF_RANGE: the sensor answered, but nothing reflected within range
//                      -> the road is free, not a fault
//   ECHO_NO_RESPONSE:  the sensor never answered -> possible fault
enum EchoResult : uint8_t { ECHO_OK, ECHO_OUT_OF_RANGE, ECHO_NO_RESPONSE };

// ---------------------------------------------------------------------------
// Median of the last 3 readings (REQ-02)
//
// The parking assistant used 5 readings. A moving vehicle needs a faster
// reaction: with 3 readings a real obstacle shows up after 2 measurements
// (~120 ms) instead of 3 (~180 ms), and one wrong reading is still ignored.
// ---------------------------------------------------------------------------
const uint8_t FILTER_SIZE = 3;

class MedianFilter {
 public:
  MedianFilter() : count_(0), next_(0) {}

  void reset() {
    count_ = 0;
    next_ = 0;
  }

  bool empty() const { return count_ == 0; }
  uint8_t count() const { return count_; }

  void add(float value) {
    values_[next_] = value;
    next_ = (next_ + 1) % FILTER_SIZE;
    if (count_ < FILTER_SIZE) count_++;
  }

  float median() const {
    if (count_ == 0) return 0.0f;
    float s[FILTER_SIZE];
    for (uint8_t i = 0; i < count_; i++) s[i] = values_[i];
    for (uint8_t i = 1; i < count_; i++) {  // insertion sort
      float key = s[i];
      int8_t j = i - 1;
      while (j >= 0 && s[j] > key) {
        s[j + 1] = s[j];
        j--;
      }
      s[j + 1] = key;
    }
    if (count_ % 2 == 1) return s[count_ / 2];
    return (s[count_ / 2 - 1] + s[count_ / 2]) / 2.0f;
  }

 private:
  float values_[FILTER_SIZE];
  uint8_t count_;
  uint8_t next_;
};

// ---------------------------------------------------------------------------
// Speed limiting by distance (REQ-03)
//
//   distance >= 80 cm      -> cruise speed
//   25 cm .. 80 cm         -> linear ramp from crawl to cruise speed
//   distance < 25 cm       -> 0 (emergency stop)
// ---------------------------------------------------------------------------
inline int16_t speedForDistance(float cm) {
  if (cm < STOP_CM) return 0;
  if (cm >= SLOW_DOWN_CM) return CRUISE_SPEED;
  float f = (cm - STOP_CM) / (SLOW_DOWN_CM - STOP_CM);  // 0..1
  return CRAWL_SPEED + (int16_t)(f * (CRUISE_SPEED - CRAWL_SPEED) + 0.5f);
}

// On for the first onMs of every periodMs (blinking without delay()).
inline bool blinkPhase(unsigned long nowMs, unsigned long onMs, unsigned long periodMs) {
  return (nowMs % periodMs) < onMs;
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum Mode : uint8_t {
  MODE_STARTUP,
  MODE_DRIVE,
  MODE_BRAKE,
  MODE_SCAN_LEFT,
  MODE_SCAN_RIGHT,
  MODE_REVERSE,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT,
  MODE_FAULT
};

inline const char* modeName(Mode m) {
  switch (m) {
    case MODE_STARTUP: return "STARTUP";
    case MODE_DRIVE: return "DRIVE";
    case MODE_BRAKE: return "BRAKE";
    case MODE_SCAN_LEFT: return "SCAN_L";
    case MODE_SCAN_RIGHT: return "SCAN_R";
    case MODE_REVERSE: return "REVERSE";
    case MODE_TURN_LEFT: return "TURN_L";
    case MODE_TURN_RIGHT: return "TURN_R";
    case MODE_FAULT: return "FAULT";
  }
  return "?";
}

// What the controller knows each cycle.
struct RoverInputs {
  unsigned long nowMs;
  bool hasReading;     // is there a filtered distance for the CURRENT head angle?
  float distanceCm;    // filtered distance (only meaningful if hasReading)
  bool sensorFault;    // no valid reading for too long (REQ-05)
};

// What the controller wants the hardware to do.
struct RoverCommand {
  int16_t left;        // -255..255, negative = backwards
  int16_t right;
  uint8_t headAngle;
  bool led;
  bool buzzer;
};

class RoverController {
 public:
  RoverController()
      : mode_(MODE_STARTUP), modeStartMs_(0), leftCm_(0), rightCm_(0) {}

  void begin(unsigned long nowMs) { enter(MODE_STARTUP, nowMs); }

  Mode mode() const { return mode_; }
  float leftCm() const { return leftCm_; }
  float rightCm() const { return rightCm_; }

  RoverCommand update(const RoverInputs& in) {
    // REQ-05: a sensor fault overrides everything. A vehicle that cannot
    // see must stop.
    if (in.sensorFault && mode_ != MODE_FAULT) {
      enter(MODE_FAULT, in.nowMs);
    }

    RoverCommand cmd = {0, 0, HEAD_CENTER, false, false};  // default: stand still
    unsigned long t = in.nowMs - modeStartMs_;              // time in this mode

    switch (mode_) {
      case MODE_STARTUP:  // REQ-07
        cmd.led = t < SELF_TEST_MS;
        cmd.buzzer = t < SELF_TEST_MS;
        if (t >= STARTUP_MS) enter(MODE_DRIVE, in.nowMs);
        break;

      case MODE_DRIVE:  // REQ-01, REQ-03
        if (!in.hasReading) {
          break;  // no data, no motion
        }
        {
          int16_t speed = speedForDistance(in.distanceCm);
          if (speed == 0) {
            enter(MODE_BRAKE, in.nowMs);  // AEB: stop in this same cycle
            cmd.led = true;
            cmd.buzzer = true;
          } else {
            cmd.left = speed;
            cmd.right = speed;
          }
        }
        break;

      case MODE_BRAKE:  // REQ-03, REQ-06
        cmd.led = true;
        cmd.buzzer = t < BRAKE_BEEP_MS;
        if (t >= BRAKE_HOLD_MS) enter(MODE_SCAN_LEFT, in.nowMs);
        break;

      case MODE_SCAN_LEFT:  // REQ-04
        cmd.headAngle = HEAD_LEFT;
        cmd.led = true;
        if (t >= SETTLE_MS && in.hasReading) {
          leftCm_ = in.distanceCm;
          enter(MODE_SCAN_RIGHT, in.nowMs);
          cmd.headAngle = HEAD_RIGHT;
        }
        break;

      case MODE_SCAN_RIGHT:  // REQ-04
        cmd.headAngle = HEAD_RIGHT;
        cmd.led = true;
        if (t >= SETTLE_MS && in.hasReading) {
          rightCm_ = in.distanceCm;
          decide(in.nowMs);
          cmd.headAngle = HEAD_CENTER;
        }
        break;

      case MODE_REVERSE:  // REQ-04, REQ-06
        cmd.left = -REVERSE_SPEED;
        cmd.right = -REVERSE_SPEED;
        cmd.led = true;
        cmd.buzzer = blinkPhase(t, 200, 400);  // reversing beep
        if (t >= REVERSE_MS) {
          enter(MODE_SCAN_LEFT, in.nowMs);
          cmd = RoverCommand{0, 0, HEAD_LEFT, true, false};
        }
        break;

      case MODE_TURN_LEFT:  // REQ-04
        cmd.left = -TURN_SPEED;
        cmd.right = TURN_SPEED;
        if (t >= TURN_MS) {
          enter(MODE_DRIVE, in.nowMs);
          cmd.left = 0;
          cmd.right = 0;
        }
        break;

      case MODE_TURN_RIGHT:  // REQ-04
        cmd.left = TURN_SPEED;
        cmd.right = -TURN_SPEED;
        if (t >= TURN_MS) {
          enter(MODE_DRIVE, in.nowMs);
          cmd.left = 0;
          cmd.right = 0;
        }
        break;

      case MODE_FAULT:  // REQ-05
        cmd.led = blinkPhase(in.nowMs, 250, 500);
        if (!in.sensorFault && in.hasReading) {
          enter(MODE_DRIVE, in.nowMs);  // DRIVE still checks the distance first
        }
        break;
    }
    return cmd;
  }

 private:
  void enter(Mode m, unsigned long nowMs) {
    mode_ = m;
    modeStartMs_ = nowMs;
  }

  // After scanning: turn towards the side with more space, or reverse
  // if neither side is free (REQ-04).
  void decide(unsigned long nowMs) {
    float best = (leftCm_ > rightCm_) ? leftCm_ : rightCm_;
    if (best < CLEAR_CM) {
      enter(MODE_REVERSE, nowMs);
    } else if (leftCm_ >= rightCm_) {
      enter(MODE_TURN_LEFT, nowMs);
    } else {
      enter(MODE_TURN_RIGHT, nowMs);
    }
  }

  Mode mode_;
  unsigned long modeStartMs_;
  float leftCm_;
  float rightCm_;
};

#endif  // ROVER_LOGIC_H
