// Fake Servo for the SIL test: remembers the last angle.
#ifndef FAKE_SERVO_H
#define FAKE_SERVO_H
struct Servo {
  int angle = -1;
  void attach(int) {}
  void write(int a) { angle = a; }
};
#endif
