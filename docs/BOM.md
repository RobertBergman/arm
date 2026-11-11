# Bill of materials

Everything the printed parts are designed around. The CAD pockets, hole patterns
and joint spacings are generated from the dimensions in
[`cad/armcad/specs.py`](../cad/armcad/specs.py), so **buy these exact part types**
(or edit that file to match what you bought and re-run the build).

Prices are rough guides only and vary a lot by seller.

## Electronics

| Qty | Part | What to look for | Used for | Approx. price |
| --- | --- | --- | --- | --- |
| 3 | **DS3218MG digital servo, 20 kg·cm, 270° version** | Standard size 40 × 20 × 40.5 mm, 25T metal spline, 4.8–6.8 V. Listings say "DS3218MG 270 degree" | Turret, shoulder, elbow | $12–18 each |
| 3 | **MG90S micro servo, 180°** | Metal gear, 22.8 × 12.2 × 28.5 mm, ships with a bag of plastic horns and screws | Wrist pitch, wrist roll, gripper | $3–5 each |
| 3 | **25T aluminium servo horn, round disc** | Ø20 mm disc, 4 × M3 threaded holes on a 14 mm circle, with centre screw. Often sold in packs of 4–10 | One per DS3218MG | $1–2 each |
| 1 | **PCA9685 16-channel 12-bit PWM servo driver** | The common 62.5 × 25.4 mm board (Adafruit #815 or clone) with the blue screw terminal | Generates all six servo signals | $4–15 |
| 1 | **ESP32 DevKit board** (30- or 38-pin) | Any ESP32-WROOM dev board with USB. An Arduino Uno/Nano also works with the same firmware | Controller | $6–10 |
| 1 | **5 V DC power supply, 10 A (50 W)** | Regulated, with a 5.5 × 2.1 mm barrel plug. 5 A is the bare minimum | Servo power | $12–20 |
| 1 | **DC barrel jack to screw-terminal adapter**, female 5.5 × 2.1 mm | | Connects the supply to the PCA9685 | $1–2 |
| 1 | **Electrolytic capacitor 1000 µF, 10 V or higher** | Solders into the `C2` pads of the PCA9685 board | Absorbs servo current spikes | $1 |
| 5 | **Servo extension leads** (JR/Futaba 3-pin, male–female) | 2 × 30 cm and 3 × 50 cm | Shoulder and elbow (30 cm); wrist pitch, wrist roll, gripper (50 cm) | $5 per pack |
| 4 | **Female–female jumper wires** (Dupont) | 10–20 cm | ESP32 ↔ PCA9685 (3.3 V, GND, SDA, SCL) | $2 |
| 1 | USB cable for the ESP32 | Data-capable | Programming and serial control | – |

Why these servos: with the arm stretched out horizontally and a 60 g payload the
shoulder needs 5.5 kg·cm, the elbow 2.6 kg·cm and the wrist 0.9 kg·cm
(see [VERIFICATION.md](VERIFICATION.md)). That is 3.3×, 6.9× and 2.0× below the
stall torque of the servos above, even with every printed part counted as solid plastic.

## Fasteners

An M3 socket-head assortment (6–20 mm) plus the small bag below covers everything.

| Qty | Fastener | Where |
| --- | --- | --- |
| 17 | M3 × 8 socket-head screw | 12 horn bolts (4 per disc horn), 1 wrist pivot, 4 electronics tray |
| 2 | M3 × 10 socket-head screw | Shoulder and elbow pivots |
| 18 | M3 × 12 socket-head screw | 12 servo tabs (4 per DS3218MG), 6 base-to-grid |
| 1 | M3 × 16 socket-head screw | Idler finger pivot |
| 24 | M3 flat washer (0.5 mm thick) | Under servo-tab screws, pivot screws, and as joint shims |
| 4 | M2 × 8 self-tapping screw | Wrist horn (2), drive finger horn (2) |
| 2 | M2 × 6 self-tapping screw | Gripper body to roll horn |
| 4 | M2.5 × 6 screw | PCA9685 to the tray standoffs |
| 6 | M2 × 8 self-tapping screw | MG90S mounting tabs — normally included with the servos |

No nuts or threaded inserts are needed: M3 screws thread directly into 2.8 mm
printed holes.

## Consumables

| Qty | Item | Notes |
| --- | --- | --- |
| ~0.6 kg | PLA or PETG filament | The arm alone is about 0.2 kg; the six grid tiles are the rest. One 1 kg spool is enough |
| 2 | Self-adhesive rubber or neoprene pads, 2 mm thick | Cut to about 16 × 25 mm, one per finger. The jaw geometry assumes 2 mm pads |
| ~12 | Small cable ties (2.5 mm) | Cable routing and strapping the ESP32 to the tray |
| – | Light grease (silicone or PTFE) | Gripper gear teeth, turret shim, pivot screws |
| – | Medium thread-locker (optional) | Pivot screws |

## Payload

The gripper opens from 26 to 58 mm and is tuned for the 84 × 38 × 35 mm truck
of the original model. Real Matchbox vehicles are usually 28–32 mm wide: set
`holdWidth` in `firmware/arm_controller/ArmConfig.h` to the vehicle width minus 1.5 mm.
