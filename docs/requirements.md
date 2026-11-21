# Requirements: Mini ADAS Rover v1

Each requirement has an ID that is used in the code comments and in the tests.
Automated tests: `tests/test_logic.cpp`. Simulation tests: `docs/simulation.md`.

| ID | Requirement | Verified by |
|----|-------------|-------------|
| REQ-01 | While driving, the rover measures the distance ahead at least 10 times per second. | Code review (60 ms period), SIM-02 |
| REQ-02 | Distance readings are filtered (median of the last 3), so one wrong reading does not change the rover's behaviour. | Unit test |
| REQ-03 | Speed is limited by the distance ahead: **80 cm or more** → cruise speed; **25 to under 80 cm** → speed falls linearly to crawl speed; **under 25 cm** → emergency stop (AEB) in the same control cycle. Without a valid reading the rover does not move. | Unit test, SIM-02, SIM-03 |
| REQ-04 | After an emergency stop the rover stands still for 0.3 s, looks left and right (waiting 0.4 s at each side for the servo to settle), and turns towards the side with more space if that side has at least 40 cm. If neither side is free, it reverses slowly for 0.5 s and looks again. A reading is only used for the direction the head pointed at when it was taken. | Unit test, SIM-04, SIM-05 |
| REQ-05 | If the sensor does not answer for more than 0.5 s, the rover stops immediately in every mode and the status LED blinks. It drives again only after a valid reading. An echo that does not come back within range counts as "free road", not as a fault. | Unit test, SIM-06 |
| REQ-06 | Warnings: a short beep at an emergency stop, an intermittent beep while reversing, and the status LED on while braking, scanning and reversing. | Unit test, SIM-03, SIM-05 |
| REQ-07 | At power-on the motors are switched off first. LED and buzzer run a 0.4 s self-test, and the rover does not move for 2 s. | Unit test, SIM-01 |
| REQ-08 | The rover sends telemetry over USB every 100 ms as CSV: `time_ms,mode,raw_cm,filtered_cm,left,right,head_deg`. | SIM-01 |

## Design decisions

- **Median of 3, not 5 (REQ-02).** The parking assistant used 5 readings. A moving vehicle must react faster: with 3, a real obstacle is seen after 2 measurements (~120 ms) instead of 3 (~180 ms). At about 0.5 m/s that is 3 cm less travel before braking, and one wrong reading is still ignored.
- **Speed ramp instead of on/off (REQ-03).** Slowing down before the stop distance means the rover is slow when it has to brake, so it stops in a shorter distance. Real AEB and ACC systems also work with distance-dependent speed or deceleration.
- **"No data, no motion" (REQ-03, REQ-05).** Every time the head turns or the rover starts driving again, old readings are thrown away, and the rover waits for a fresh one before moving.
- **No echo ≠ sensor fault (REQ-05).** In a large room the echo may not come back at all. That means "nothing in range", so it must not stop the rover. Only a sensor that never answers is a fault. The code times the echo pin by hand to tell the two cases apart.
- **Fail-safe = stop (REQ-05).** For a moving vehicle the safe state is standing still. The parking assistant went to its most critical warning; the rover stops its motors.

## Known limitations of v1

- **Turns and reversing are time-based.** How far the rover turns in 0.4 s depends on the battery voltage and the floor. Fix in v2: wheel speed sensors.
- **No rear sensor.** Reversing is short and slow for that reason.
- **One forward sensor.** The HC-SR04's beam is about 15° wide, so narrow, low or side obstacles can be missed.
