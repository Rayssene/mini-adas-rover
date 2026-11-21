// Software-in-the-loop (SIL) test.
//
// Runs the REAL sketch rover/rover.ino on a PC against a simulated HC-SR04
// (tests/sil/Arduino.h) and checks what arrives at the motor driver pins.
// Unlike the unit tests, this also covers the hardware layer: echo timing,
// fault detection, filter resets and the L9110S signal inversion.
//
// Build and run (from the repository root):
//   g++ -std=gnu++11 -I tests/sil -I rover tests/sil/sil_test.cpp -o sil_test
//   ./sil_test

#include "Arduino.h"
#include "../../rover/rover.ino"

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; \
  printf("  FAILED line %d at t=%lu ms: %s\n", __LINE__, millis(), #c); } } while (0)

static void runFor(unsigned long ms) {
  unsigned long long end = sim::timeUs + ms * 1000ULL;
  while (sim::timeUs < end) {
    loop();
    sim::advance(200);  // time spent in the rest of the loop
  }
}

// Runs until the controller reaches `mode`. Returns false after timeoutMs.
static bool runUntil(Mode mode, unsigned long timeoutMs) {
  unsigned long long end = sim::timeUs + timeoutMs * 1000ULL;
  while (sim::timeUs < end) {
    if (controller.mode() == mode) return true;
    runFor(5);
  }
  return controller.mode() == mode;
}

// Wheel speed as the motor actually sees it, from the L9110S pin levels.
static int wheel(int pinSpeed, int pinDir) {
  return sim::pinLevel[pinDir] == LOW ? sim::pinPwm[pinSpeed]
                                      : -(255 - sim::pinPwm[pinSpeed]);
}
static int leftWheel() { return wheel(PIN_LEFT_SPEED, PIN_LEFT_DIR); }
static int rightWheel() { return wheel(PIN_RIGHT_SPEED, PIN_RIGHT_DIR); }

int main() {
  setup();

  printf("SIL-01: startup, then cruise on a free road (REQ-07, REQ-03)\n");
  sim::obstacleCm = 150;
  runFor(1500);
  CHECK(controller.mode() == MODE_STARTUP);
  CHECK(leftWheel() == 0 && rightWheel() == 0);
  runFor(1000);
  CHECK(controller.mode() == MODE_DRIVE);
  CHECK(leftWheel() == CRUISE_SPEED && rightWheel() == CRUISE_SPEED);
  CHECK(head.angle == HEAD_CENTER);

  printf("SIL-02: slow down as the obstacle gets closer (REQ-03)\n");
  sim::obstacleCm = 60;
  runFor(300);
  CHECK(leftWheel() == speedForDistance(60));
  CHECK(leftWheel() < CRUISE_SPEED);

  printf("SIL-03: emergency stop within 200 ms (REQ-02, REQ-03)\n");
  sim::obstacleCm = 20;
  runFor(200);
  CHECK(leftWheel() == 0 && rightWheel() == 0);
  CHECK(controller.mode() == MODE_BRAKE || controller.mode() == MODE_SCAN_LEFT);

  printf("SIL-04: scan, turn towards the free side, drive on (REQ-04)\n");
  CHECK(runUntil(MODE_SCAN_LEFT, BRAKE_HOLD_MS + 100));
  CHECK(head.angle == HEAD_LEFT);
  CHECK(leftWheel() == 0 && rightWheel() == 0);
  sim::obstacleCm = 150;             // free on the left
  CHECK(runUntil(MODE_SCAN_RIGHT, SETTLE_MS + 300));
  CHECK(head.angle == HEAD_RIGHT);
  sim::obstacleCm = 50;              // less space on the right
  CHECK(runUntil(MODE_TURN_LEFT, SETTLE_MS + 300));
  runFor(50);
  CHECK(leftWheel() < 0 && rightWheel() > 0);  // inverted PWM on the L9110S
  CHECK(head.angle == HEAD_CENTER);
  sim::obstacleCm = 150;
  CHECK(runUntil(MODE_DRIVE, TURN_MS + 100));
  runFor(300);
  CHECK(leftWheel() == CRUISE_SPEED);

  printf("SIL-05: dead end -> reverse with the buzzer, then scan again (REQ-04, REQ-06)\n");
  sim::obstacleCm = 20;
  CHECK(runUntil(MODE_BRAKE, 300));
  CHECK(runUntil(MODE_REVERSE, BRAKE_HOLD_MS + 2 * SETTLE_MS + 600));
  runFor(50);
  CHECK(leftWheel() == -REVERSE_SPEED && rightWheel() == -REVERSE_SPEED);
  CHECK(sim::pinLevel[PIN_BUZZER] == HIGH);
  CHECK(runUntil(MODE_SCAN_LEFT, REVERSE_MS + 100));

  printf("SIL-06: no echo within range is free road, not a fault (REQ-05)\n");
  sim::obstacleCm = 500;
  runFor(3000);
  CHECK(controller.mode() != MODE_FAULT);
  CHECK(!sensorFault);
  CHECK(controller.mode() == MODE_DRIVE);
  CHECK(leftWheel() == CRUISE_SPEED);

  printf("SIL-07: unplugged sensor -> stop within 0.7 s, LED blinks (REQ-05)\n");
  sim::echoConnected = false;
  runFor(700);
  CHECK(controller.mode() == MODE_FAULT);
  CHECK(leftWheel() == 0 && rightWheel() == 0);
  bool ledOn = false, ledOff = false;
  for (int i = 0; i < 20; i++) {
    runFor(50);
    if (sim::pinLevel[PIN_LED]) ledOn = true; else ledOff = true;
  }
  CHECK(ledOn && ledOff);

  printf("SIL-08: sensor works again -> drive on (REQ-05)\n");
  sim::echoConnected = true;
  sim::obstacleCm = 150;
  runFor(300);
  CHECK(controller.mode() == MODE_DRIVE);
  CHECK(leftWheel() == CRUISE_SPEED);

  printf("\n%d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
