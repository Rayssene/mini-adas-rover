// Unit tests for rover_logic.h. They run on a PC, no Arduino needed.
//
// Build and run (from the repository root):
//   g++ -std=c++11 -Wall -Wextra -I rover tests/test_logic.cpp -o test_logic
//   ./test_logic
//
// Each test names the requirement it verifies (traceability).

#include <cmath>
#include <cstdio>

#include "rover_logic.h"

static int failures = 0;
static int checks = 0;

#define CHECK(condition)                                           \
  do {                                                             \
    checks++;                                                      \
    if (!(condition)) {                                            \
      failures++;                                                  \
      std::printf("  FAILED line %d: %s\n", __LINE__, #condition); \
    }                                                              \
  } while (0)

static bool near(float a, float b) { return std::fabs(a - b) < 0.01f; }

static RoverInputs reading(unsigned long now, float cm) {
  RoverInputs in = {now, true, cm, false};
  return in;
}
static RoverInputs noReading(unsigned long now) {
  RoverInputs in = {now, false, 0.0f, false};
  return in;
}
static RoverInputs fault(unsigned long now) {
  RoverInputs in = {now, false, 0.0f, true};
  return in;
}

// Brings a fresh controller to MODE_DRIVE at time STARTUP_MS.
static RoverController driving() {
  RoverController c;
  c.begin(0);
  c.update(noReading(STARTUP_MS));
  return c;
}

// ---------------------------------------------------------------------------

static void test_sensor_helpers() {
  std::printf("REQ-01/REQ-05: echo conversion and valid range\n");
  CHECK(near(echoToCm(1000), 17.15f));
  CHECK(near(echoToCm(5831), 100.0f));
  CHECK(!isValidDistance(1.9f));
  CHECK(isValidDistance(2.0f));
  CHECK(isValidDistance(400.0f));
  CHECK(!isValidDistance(400.1f));
}

static void test_median_filter() {
  std::printf("REQ-02: median of 3 ignores one wrong reading\n");
  MedianFilter f;
  CHECK(f.empty());
  f.add(60.0f);
  CHECK(near(f.median(), 60.0f));
  f.add(400.0f);  // outlier
  f.add(58.0f);
  CHECK(near(f.median(), 60.0f));
  f.add(5.0f);    // single wrong "very close" reading
  CHECK(near(f.median(), 58.0f));
  f.add(20.0f);   // obstacle really appears: 2 of 3 readings agree
  CHECK(f.median() < 25.0f);
  f.reset();
  CHECK(f.empty());
}

static void test_speed_limiting() {
  std::printf("REQ-03: speed limited by distance\n");
  CHECK(speedForDistance(300.0f) == CRUISE_SPEED);
  CHECK(speedForDistance(80.0f) == CRUISE_SPEED);
  CHECK(speedForDistance(79.0f) < CRUISE_SPEED);
  CHECK(speedForDistance(52.5f) == (CRUISE_SPEED + CRAWL_SPEED) / 2);
  CHECK(speedForDistance(25.0f) == CRAWL_SPEED);
  CHECK(speedForDistance(24.9f) == 0);
  // The speed never increases when the obstacle gets closer
  int16_t previous = speedForDistance(200.0f);
  bool monotonic = true;
  for (float cm = 200.0f; cm >= 0.0f; cm -= 0.5f) {
    int16_t s = speedForDistance(cm);
    if (s > previous) monotonic = false;
    previous = s;
  }
  CHECK(monotonic);
}

static void test_startup() {
  std::printf("REQ-07: self-test and stand still after power-on\n");
  RoverController c;
  c.begin(0);
  RoverCommand cmd = c.update(reading(100, 300.0f));
  CHECK(c.mode() == MODE_STARTUP);
  CHECK(cmd.led && cmd.buzzer);           // self-test
  CHECK(cmd.left == 0 && cmd.right == 0); // no motion even with a free road
  cmd = c.update(reading(1000, 300.0f));
  CHECK(!cmd.led && !cmd.buzzer);
  CHECK(cmd.left == 0 && cmd.right == 0);
  CHECK(c.mode() == MODE_STARTUP);
  c.update(reading(STARTUP_MS, 300.0f));
  CHECK(c.mode() == MODE_DRIVE);
}

static void test_drive() {
  std::printf("REQ-01/REQ-03: driving needs data and follows the speed limit\n");
  RoverController c = driving();
  RoverCommand cmd = c.update(noReading(2100));
  CHECK(cmd.left == 0 && cmd.right == 0);  // no data, no motion
  cmd = c.update(reading(2200, 150.0f));
  CHECK(cmd.left == CRUISE_SPEED && cmd.right == CRUISE_SPEED);
  CHECK(cmd.headAngle == HEAD_CENTER);
  cmd = c.update(reading(2300, 50.0f));
  CHECK(cmd.left > CRAWL_SPEED && cmd.left < CRUISE_SPEED);
  CHECK(c.mode() == MODE_DRIVE);
}

static void test_emergency_brake() {
  std::printf("REQ-03/REQ-06: emergency stop in the same cycle, with a beep\n");
  RoverController c = driving();
  c.update(reading(2100, 150.0f));
  RoverCommand cmd = c.update(reading(2160, 20.0f));
  CHECK(c.mode() == MODE_BRAKE);
  CHECK(cmd.left == 0 && cmd.right == 0);
  CHECK(cmd.buzzer && cmd.led);
  cmd = c.update(reading(2160 + BRAKE_BEEP_MS, 20.0f));
  CHECK(!cmd.buzzer);                      // beep is short
  CHECK(cmd.left == 0 && cmd.right == 0);  // still standing
}

// Drives the controller through brake -> scan left -> scan right.
static RoverController scanWith(float leftCm, float rightCm, unsigned long& now) {
  RoverController c = driving();
  now = 2100;
  c.update(reading(now, 10.0f));                     // brake
  now += BRAKE_HOLD_MS;
  RoverCommand cmd = c.update(noReading(now));       // -> scan left
  CHECK(c.mode() == MODE_SCAN_LEFT);
  cmd = c.update(reading(now + 100, leftCm));        // too early: servo still moving
  CHECK(c.mode() == MODE_SCAN_LEFT);
  CHECK(cmd.headAngle == HEAD_LEFT);
  CHECK(cmd.left == 0 && cmd.right == 0);
  now += SETTLE_MS;
  cmd = c.update(reading(now, leftCm));              // -> scan right
  CHECK(c.mode() == MODE_SCAN_RIGHT);
  CHECK(cmd.headAngle == HEAD_RIGHT);
  now += SETTLE_MS;
  c.update(reading(now, rightCm));                   // -> decide
  return c;
}

static void test_scan_and_turn() {
  std::printf("REQ-04: look both ways and turn towards the free side\n");
  unsigned long now = 0;
  RoverController c = scanWith(150.0f, 60.0f, now);
  CHECK(near(c.leftCm(), 150.0f) && near(c.rightCm(), 60.0f));
  CHECK(c.mode() == MODE_TURN_LEFT);
  RoverCommand cmd = c.update(noReading(now + 100));
  CHECK(cmd.left < 0 && cmd.right > 0);   // spin left on the spot
  CHECK(cmd.headAngle == HEAD_CENTER);
  c.update(noReading(now + TURN_MS));
  CHECK(c.mode() == MODE_DRIVE);

  c = scanWith(45.0f, 200.0f, now);
  CHECK(c.mode() == MODE_TURN_RIGHT);
  cmd = c.update(noReading(now + 100));
  CHECK(cmd.left > 0 && cmd.right < 0);
}

static void test_dead_end() {
  std::printf("REQ-04/REQ-06: dead end -> reverse with beeps, then scan again\n");
  unsigned long now = 0;
  RoverController c = scanWith(30.0f, 35.0f, now);
  CHECK(c.mode() == MODE_REVERSE);
  RoverCommand cmd = c.update(noReading(now + 50));
  CHECK(cmd.left < 0 && cmd.right < 0);
  CHECK(cmd.buzzer);                      // reversing beep (on phase)
  cmd = c.update(noReading(now + 250));
  CHECK(!cmd.buzzer);                     // off phase
  cmd = c.update(noReading(now + REVERSE_MS));
  CHECK(c.mode() == MODE_SCAN_LEFT);
  CHECK(cmd.left == 0 && cmd.right == 0);
}

static void test_sensor_fault() {
  std::printf("REQ-05: sensor fault stops the rover in every mode\n");
  RoverController c = driving();
  c.update(reading(2100, 200.0f));
  RoverCommand cmd = c.update(fault(2200));
  CHECK(c.mode() == MODE_FAULT);
  CHECK(cmd.left == 0 && cmd.right == 0);  // full speed -> 0 in one cycle
  CHECK(cmd.headAngle == HEAD_CENTER);

  bool sawOn = false, sawOff = false;
  for (unsigned long t = 2200; t < 3200; t += 50) {
    cmd = c.update(fault(t));
    if (cmd.led) sawOn = true; else sawOff = true;
    CHECK(cmd.left == 0 && cmd.right == 0);
  }
  CHECK(sawOn && sawOff);  // LED blinks

  c.update(reading(3300, 150.0f));          // sensor works again
  CHECK(c.mode() == MODE_DRIVE);

  unsigned long now = 0;
  c = scanWith(150.0f, 60.0f, now);         // fault while turning
  cmd = c.update(fault(now + 100));
  CHECK(c.mode() == MODE_FAULT);
  CHECK(cmd.left == 0 && cmd.right == 0);

  RoverController s;                        // fault during startup
  s.begin(0);
  s.update(fault(600));
  CHECK(s.mode() == MODE_FAULT);
}

static void test_mode_names() {
  std::printf("REQ-08: mode names for the telemetry\n");
  CHECK(modeName(MODE_DRIVE)[0] == 'D');
  CHECK(modeName(MODE_FAULT)[0] == 'F');
  CHECK(modeName(MODE_SCAN_LEFT)[5] == 'L');
}

int main() {
  test_sensor_helpers();
  test_median_filter();
  test_speed_limiting();
  test_startup();
  test_drive();
  test_emergency_brake();
  test_scan_and_turn();
  test_dead_end();
  test_sensor_fault();
  test_mode_names();

  std::printf("\n%d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
