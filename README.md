# Mini ADAS Rover

![CI](https://github.com/Rayssene/mini-adas-rover/actions/workflows/ci.yml/badge.svg)

A four-wheel Arduino rover with small versions of driver-assistance functions
found in cars: **distance-based speed limiting**, **automatic emergency braking
(AEB)**, and **obstacle avoidance** with an ultrasonic sensor on a servo "head".

The project is developed the automotive way: **requirements → design →
implementation → verification**, with unit tests, a software-in-the-loop test
and a simulation before the hardware is built.

> **Status:** v1 software complete and tested on the PC and in the Wokwi
> simulator. Hardware build in progress.

## What it does

```
DRIVE ──(< 25 cm)──> BRAKE ──> SCAN LEFT ──> SCAN RIGHT ──┬──> TURN LEFT / RIGHT ──> DRIVE
  ▲                                              ▲        │
  │                                              └────────┴──< REVERSE (dead end)
  └── any mode ──(sensor fault)──> FAULT: stop, LED blinks ──(sensor OK)──> DRIVE
```

| Distance ahead | Behaviour |
|----------------|-----------|
| ≥ 80 cm | cruise speed |
| 25 – 80 cm | speed falls linearly to crawl speed |
| < 25 cm | emergency stop, beep, look left and right, turn towards the free side |
| both sides blocked | reverse slowly with a reversing beep, then look again |
| sensor not answering > 0.5 s | stop immediately, LED blinks |

Requirements REQ-01 … REQ-08 and the design decisions behind them:
[docs/requirements.md](docs/requirements.md).

## How it is verified

| Level | What | Where |
|-------|------|-------|
| Unit tests | State machine, filter and speed limiting, on a PC (104 checks) | `tests/test_logic.cpp` |
| Software-in-the-loop | The **real sketch** running on a PC against a simulated ultrasonic sensor, checking the signals at the motor driver pins | `tests/sil/` |
| Simulation | Interactive test in the Wokwi Arduino simulator | [docs/simulation.md](docs/simulation.md) |
| CI | All of the above on every push, plus a compile for the Arduino Uno | `.github/workflows/ci.yml` |
| Hardware | Test on the real rover | *coming with the build* |

## Repository structure

```
rover/
  rover.ino          hardware layer: sensor timing, L9110S motors, servo, LED, buzzer, telemetry
  rover_logic.h      pure logic: median filter, speed limiting, state machine (no Arduino code)
tests/
  test_logic.cpp     unit tests for rover_logic.h
  sil/               software-in-the-loop test with a fake Arduino and HC-SR04
wokwi/diagram.json   circuit for the Wokwi simulator
docs/                requirements, wiring, simulation guide
```

## Run the tests

```
g++ -std=c++11 -I rover tests/test_logic.cpp -o test_logic && ./test_logic
g++ -std=gnu++11 -I tests/sil -I rover tests/sil/sil_test.cpp -o sil_test && ./sil_test
```

## Hardware

Four TT gear motors on an acrylic chassis, L9110S motor driver, Arduino Uno with
Sensor Shield, HC-SR04 on an SG90 servo, two 18650 cells. The mechanical build
and pin map follow the [RoverKit](https://roverkit.vercel.app) guide; the
control software in this repository is my own.
Parts and pin map: [docs/wiring.md](docs/wiring.md).

## Roadmap

- [x] **v1** Speed limiting, AEB, obstacle avoidance, fault handling (simulation)
- [ ] **v1.1** Hardware build and hardware test report
- [ ] **v2** Wheel speed sensors and PID speed control: drives straight, turns by angle instead of time
- [ ] **v3** Adaptive cruise control: follow an object at a set distance
- [ ] **v4** Remote control and live telemetry over WiFi (ESP32)
- [ ] **v5** Split into two control units connected by CAN bus

## License

MIT
