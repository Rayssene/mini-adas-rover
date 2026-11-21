/*
  Mini ADAS Rover (Arduino Uno)

  A four-wheel rover with an ultrasonic sensor on a servo "head". It drives,
  slows down when an obstacle gets closer, brakes automatically before it
  hits it (AEB), looks left and right, and turns towards the free side.

  Requirements REQ-01 ... REQ-08: docs/requirements.md
  Hardware build and pin map: based on the RoverKit guide (docs/wiring.md)

  Structure:
    rover_logic.h  -> pure logic: filter, speed limiting, state machine
    rover.ino      -> this file: sensor, motors, servo, LED, buzzer, telemetry
*/

#include <Servo.h>
#include "rover_logic.h"

// ===========================================================================
// Configuration
// ===========================================================================
// If a side drives backwards when it should drive forwards, set its flag to 1
// (or swap the two motor wires on the driver).
#define INVERT_LEFT 0
#define INVERT_RIGHT 0

// ===========================================================================
// Pins (RoverKit pin map)
// ===========================================================================
const uint8_t PIN_LEFT_SPEED = 11;   // L9110S A-IA (PWM)
const uint8_t PIN_LEFT_DIR = 12;     // L9110S A-IB
const uint8_t PIN_RIGHT_SPEED = 5;   // L9110S B-IA (PWM)
const uint8_t PIN_RIGHT_DIR = 6;     // L9110S B-IB
const uint8_t PIN_SERVO = A5;
const uint8_t PIN_BUZZER = A0;       // active buzzer: tone() is NOT used,
                                     // it would take Timer2 and stop the PWM on D11
const uint8_t PIN_LED = 13;
const uint8_t PIN_TRIG = 4;
const uint8_t PIN_ECHO = 3;

// ===========================================================================
// Timing
// ===========================================================================
const unsigned long MEASURE_PERIOD_MS = 60;    // ~16 measurements/s (REQ-01)
const unsigned long FAULT_TIMEOUT_MS = 500;    // REQ-05
const unsigned long TELEMETRY_PERIOD_MS = 100; // REQ-08
const unsigned long ECHO_START_TIMEOUT_US = 3000;  // a working sensor answers within ~0.5 ms
const unsigned long ECHO_MAX_US = 25000;           // ~430 cm, beyond the sensor's range

// ===========================================================================
// State
// ===========================================================================
Servo head;
MedianFilter filter;
RoverController controller;

uint8_t currentHeadAngle = HEAD_CENTER;
bool sensorFault = false;
float lastRawCm = 0;
RoverCommand lastCmd = {0, 0, HEAD_CENTER, false, false};

unsigned long lastMeasureMs = 0;
unsigned long lastResponseMs = 0;
unsigned long lastTelemetryMs = 0;

// ===========================================================================
// Motors: L9110S
// Each side has two inputs. IA gets the PWM speed, IB the direction.
//   forwards:  IB = LOW,  IA = duty
//   backwards: IB = HIGH, IA = 255 - duty  (the PWM is inverted)
// ===========================================================================
void setMotor(uint8_t pinSpeed, uint8_t pinDir, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) {
    digitalWrite(pinDir, LOW);
    analogWrite(pinSpeed, speed);
  } else {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinSpeed, 255 + speed);
  }
}

void setWheels(int16_t left, int16_t right) {
  setMotor(PIN_LEFT_SPEED, PIN_LEFT_DIR, left, INVERT_LEFT);
  setMotor(PIN_RIGHT_SPEED, PIN_RIGHT_DIR, right, INVERT_RIGHT);
}

// ===========================================================================
// Sensor
// ===========================================================================
// One HC-SR04 measurement.
// Important difference: "no echo came back" usually means nothing is in
// range (free road), while "the sensor never answered" means a fault
// (unplugged, broken). Both make pulseIn() return 0, so the echo is timed
// by hand to tell them apart.
EchoResult measure(float& cm) {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long t0 = micros();
  while (digitalRead(PIN_ECHO) == LOW) {
    if (micros() - t0 > ECHO_START_TIMEOUT_US) return ECHO_NO_RESPONSE;
  }
  unsigned long start = micros();
  while (digitalRead(PIN_ECHO) == HIGH) {
    if (micros() - start > ECHO_MAX_US) return ECHO_OUT_OF_RANGE;
  }
  cm = echoToCm(micros() - start);
  return ECHO_OK;
}

void updateSensor(unsigned long now) {
  if (now - lastMeasureMs < MEASURE_PERIOD_MS) return;
  lastMeasureMs = now;

  float cm = 0;
  EchoResult r = measure(cm);
  if (r == ECHO_OUT_OF_RANGE) {
    cm = MAX_VALID_CM;  // nothing within range: treat as far away
  }

  if (r != ECHO_NO_RESPONSE && isValidDistance(cm)) {
    filter.add(cm);   // REQ-02
    lastResponseMs = now;
    sensorFault = false;
  } else if (!sensorFault && now - lastResponseMs > FAULT_TIMEOUT_MS) {
    sensorFault = true;  // REQ-05
    filter.reset();
  }
  lastRawCm = (r == ECHO_NO_RESPONSE) ? -1.0f : cm;
}

// ===========================================================================
// Outputs
// ===========================================================================
void applyCommand(const RoverCommand& cmd) {
  // A reading belongs to the direction the head pointed at. When the head
  // turns, old readings are thrown away (REQ-04).
  if (cmd.headAngle != currentHeadAngle) {
    head.write(cmd.headAngle);
    currentHeadAngle = cmd.headAngle;
    filter.reset();
  }
  setWheels(cmd.left, cmd.right);
  digitalWrite(PIN_LED, cmd.led ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, cmd.buzzer ? HIGH : LOW);
}

// REQ-08: one CSV line every 100 ms for the Serial Monitor / Serial Plotter.
void sendTelemetry(unsigned long now) {
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  Serial.print(now);
  Serial.print(',');
  Serial.print(modeName(controller.mode()));
  Serial.print(',');
  Serial.print(lastRawCm, 1);
  Serial.print(',');
  Serial.print(filter.empty() ? -1.0f : filter.median(), 1);
  Serial.print(',');
  Serial.print(lastCmd.left);
  Serial.print(',');
  Serial.print(lastCmd.right);
  Serial.print(',');
  Serial.println(currentHeadAngle);
}

// ===========================================================================
// Arduino entry points
// ===========================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LEFT_SPEED, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_SPEED, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  setWheels(0, 0);  // motors off before anything else
  head.attach(PIN_SERVO);
  head.write(HEAD_CENTER);

  unsigned long now = millis();
  lastResponseMs = now;
  controller.begin(now);
  Serial.println("time_ms,mode,raw_cm,filtered_cm,left,right,head_deg");
}

void loop() {
  unsigned long now = millis();

  updateSensor(now);

  RoverInputs in;
  in.nowMs = now;
  in.hasReading = !filter.empty();
  in.distanceCm = filter.median();
  in.sensorFault = sensorFault;

  Mode before = controller.mode();
  lastCmd = controller.update(in);

  // Readings taken while the rover was turning point in other directions.
  // When it starts driving again, it waits for a fresh reading.
  if (controller.mode() == MODE_DRIVE && before != MODE_DRIVE) {
    filter.reset();
  }
  applyCommand(lastCmd);
  sendTelemetry(now);
}
