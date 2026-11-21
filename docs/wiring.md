# Hardware and wiring

The mechanical build and the pin map follow the **RoverKit** guide
(https://roverkit.vercel.app), so the code runs on a rover built with it.

## Parts (v1)

| Part | Qty | Already owned? |
|------|-----|----------------|
| Arduino Uno R3 + USB cable | 1 | yes |
| HC-SR04 ultrasonic sensor | 1 | yes |
| SG90 micro servo | 1 | yes |
| Red LED + 220 Ω resistor | 1 | yes |
| 4WD acrylic chassis with 4 TT gear motors (3–6 V, 1:48) and 65 mm wheels | 1 | buy |
| L9110S dual motor driver | 1 | buy |
| Sensor Shield V5 for the Uno | 1 | buy |
| Servo bracket for the HC-SR04 | 1 | buy (often in the chassis kit) |
| 2× protected 18650 Li-ion cells + 2-slot series holder | 1 | buy |
| Li-ion charger for 18650 cells | 1 | buy |
| Slide switch, 5 V active buzzer, DC barrel plug 5.5 × 2.1 mm | 1 each | buy |
| Jumper wires, M3 standoffs and screws, heat-shrink tubing | | buy |

**For v2 (wheel speed control):** 2× optical speed sensors (LM393 type) with encoder discs for TT motors.

**Tools:** soldering iron, small screwdrivers, wire stripper, multimeter.

> Use only **protected** 18650 cells and charge them in a proper Li-ion charger.
> An unprotected cell can deliver tens of amps into a short circuit.

## Pin map

| Function | Arduino pin | Note |
|----------|-------------|------|
| Left motors speed (L9110S A-IA) | D11 | PWM, Timer2 |
| Left motors direction (L9110S A-IB) | D12 | |
| Right motors speed (L9110S B-IA) | D5 | PWM, Timer0 |
| Right motors direction (L9110S B-IB) | D6 | |
| Servo (head) | A5 | |
| Active buzzer | A0 | |
| Status LED (via 220 Ω) | D13 | |
| HC-SR04 TRIG | D4 | |
| HC-SR04 ECHO | D3 | |

## Why these pins

- The **Servo library uses Timer1**, which switches off PWM on D9 and D10. The speed signals are therefore on D5 (Timer0) and D11 (Timer2).
- **`tone()` uses Timer2**, which would stop the PWM on D11. That is why the buzzer is an *active* buzzer switched with `digitalWrite()`.
- The two front and the two rear motors on each side are wired in parallel to one channel of the L9110S, so each side is driven like one wheel (skid steering).
