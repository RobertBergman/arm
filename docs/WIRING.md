# Wiring

```
                         5 V / 10 A supply
                          +            -
                          |            |
                 +--------+------------+---------+
                 |   V+ screw terminal  GND      |
                 |                               |
  ESP32 DevKit   |        PCA9685 board          |
  ------------   |                               |
   3V3  ---------+ VCC                           |
   GND  ---------+ GND                           |
   GPIO21 (SDA) -+ SDA          [C2: 1000 uF]    |
   GPIO22 (SCL) -+ SCL                           |
                 |  ch0  ch1  ch2  ch3  ch4  ch5 |
                 +---+----+----+----+----+----+--+
                     |    |    |    |    |    |
                 turret   |  elbow  |  wrist  gripper
                      shoulder   wrist   roll
                                 pitch
```

| PCA9685 channel | Joint | Servo | Extension lead |
| --- | --- | --- | --- |
| 0 | Turret | DS3218MG | none |
| 1 | Shoulder | DS3218MG | 30 cm |
| 2 | Elbow | DS3218MG | 30 cm |
| 3 | Wrist pitch | MG90S | 50 cm |
| 4 | Wrist roll | MG90S | 50 cm |
| 5 | Gripper | MG90S | 50 cm |

## Rules that prevent damage

1. **Servo power comes only from the 5 V supply through the PCA9685 `V+` screw
   terminal.** Never power the servos from the ESP32 or from USB: three standard
   servos can draw more than 5 A together.
2. `VCC` on the PCA9685 is the logic supply. Connect it to **3V3** on an ESP32
   (to 5V on an Arduino Uno/Nano) — not to `V+`.
3. Servo plugs go on the 3-pin headers with the **brown/black wire (ground) on
   the outer edge of the board**, red in the middle, orange/yellow (signal) inside.
4. Solder the 1000 µF capacitor into the `C2` pads, observing polarity (stripe = minus).
5. The supply ground, PCA9685 ground and ESP32 ground must all be connected
   (they are, through the board, if wired as above).
6. Check the polarity at the screw terminal before plugging in the servos.

On an Arduino Uno/Nano use `A4` (SDA) and `A5` (SCL) instead of GPIO21/22.

## Mounting

The printed `electronics_tray` screws to the rear-left grid tile (4 × M3 × 8).
The PCA9685 sits on the four standoffs (M2.5 × 6); the ESP32 and the barrel-jack
adapter are strapped down with cable ties through the slots.

Cable routing: tie the wrist and gripper leads to the slots on the left
forearm rail and on the back of the upper arm. Leave a loop of about 6 cm at the
elbow and shoulder, and a loop of about 12 cm between the turret and the tray so
the turret can swing ±135° freely.
