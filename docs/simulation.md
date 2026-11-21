# Running the rover in Wokwi

Wokwi (https://wokwi.com) simulates the Arduino with your real code, so the
logic can be tested before the parts arrive. Wokwi has no DC motors, so four
LEDs stand in for the motor driver signals.

## Set up the project (5 minutes)

1. Go to **wokwi.com**, sign in (free), and start a new **Arduino Uno** project.
2. **sketch.ino**: delete its content and paste the content of `rover/rover.ino`.
3. **diagram.json**: open the `diagram.json` tab, delete its content and paste `wokwi/diagram.json`.
4. Add the header: click the small arrow next to the file tabs → **New file…** → name it `rover_logic.h` → paste the content of `rover/rover_logic.h`.
5. Click **Save**, then the green **Start** button.

The Servo library is built into Wokwi, so nothing else is needed.

## Reading the simulation

| Part in Wokwi | Stands for |
|---------------|------------|
| Green LED "L speed" / "R speed" | PWM signal for the left/right motors. Brighter = faster. |
| Orange LED "L reverse" / "R reverse" | That side drives backwards. (Then the speed LED is inverted: dimmer = faster.) |
| Servo | The rover's head |
| HC-SR04 | Click it during the simulation to get a **distance slider**. You play "the world". |
| Red LED | Status LED |

The **Serial Monitor** at the bottom shows the telemetry every 100 ms, e.g.
`4200,DRIVE,150.1,150.1,200,200,90` (time, mode, raw cm, filtered cm, left, right, head angle).

Wokwi's buzzer is a passive buzzer, so it only clicks when the code switches it
on. On the real rover the active buzzer beeps.

## Simulation tests

The sensor in Wokwi always reports the slider value, whichever way the head
points. To test the scan, move the slider while the head is turned.

| ID | Requirement | What to do | Expected |
|----|-------------|-----------|----------|
| SIM-01 | REQ-07, REQ-08 | Start the simulation with the slider at 150 cm. | LED and buzzer for 0.4 s; mode STARTUP for 2 s with left/right = 0; then DRIVE. A CSV line every ~100 ms. |
| SIM-02 | REQ-01, REQ-03 | Slider at 150, then 60, then 30 cm. | left/right = 200, then 167, then 118. Green LEDs get dimmer. |
| SIM-03 | REQ-03, REQ-06 | Slider at 20 cm. | Mode BRAKE, left/right = 0 in the next telemetry line, buzzer click, status LED on. |
| SIM-04 | REQ-04 | After SIM-03: when the head turns left (head_deg 160), set the slider to 150. When it turns right (20), set it to 50. | TURN_L for 0.4 s (left negative, right positive), then DRIVE. |
| SIM-05 | REQ-04, REQ-06 | Slider at 20 cm and leave it there. | BRAKE → SCAN_L → SCAN_R → REVERSE (both negative, orange LEDs on) → SCAN_L … |
| SIM-06 | REQ-05 | While in DRIVE, delete the wire from ECHO to pin 3 (click the wire, press Delete). | After ~0.5 s: FAULT, left/right = 0, status LED blinks. Undo (Ctrl+Z) the deletion: back to DRIVE. |
| SIM-07 | REQ-05 | Slider at 400 cm (maximum). | Mode stays DRIVE at cruise speed: far away is not a fault. |

Note the results (pass / fail and what you saw) in this file, and save the
Wokwi project link in the README so others can try it.
