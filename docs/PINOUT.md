# Pinout & wiring guide

Everything here matches `firmware/arduino_motor_controller/hardware_config.h`.
If you change the wiring, change that file too — they must agree.

---

## 1. The big picture

```
   ┌────────────┐   USB (data)   ┌─────────────┐   6 control wires   ┌──────────────┐
   │  Pi 5      │◄──────────────►│ Arduino Uno │────────────────────►│ L298N driver │
   │            │                │             │                     │              │
   │  camera ───┤                │  pin 2 ◄─── E-stop switch          │  OUT1..OUT4 ─┼──► motors
   │  (CSI)     │                └─────────────┘                     └──────┬───────┘
   │  GPIO ─────┼──► HC-SR04 (via divider)                                  │
   └────────────┘                                                    separate battery
                                                                      (motor power only)
```

**The Pi never touches the motor driver.** Its only link to the motor side is
the USB cable to the Arduino. (The HC-SR04 sensors are the one exception — they
do go to Pi GPIO.)

---

## 2. Motor driver → Arduino (the 6 control wires)

This is the set you asked about. All six go to the **Arduino**, never the Pi.

| L298N pin | Arduino pin | Function |
|---|---|---|
| **ENA** | **5** | Left motor speed (PWM) |
| **IN1** | **7** | Left direction A |
| **IN2** | **8** | Left direction B |
| **IN3** | **9** | Right direction A |
| **IN4** | **10** | Right direction B |
| **ENB** | **6** | Right motor speed (PWM) |

⚠ **Remove the ENA and ENB jumpers** on the L298N board. Those jumpers tie the
enable pins to +5 V (full speed always). With them on, PWM does nothing and the
motors only run flat out or not at all.

Motor outputs:

| L298N | Connects to |
|---|---|
| OUT1 / OUT2 | Left motor |
| OUT3 / OUT4 | Right motor |

If a motor spins backwards, **do not rewire** — flip `LEFT_MOTOR_INVERTED` or
`RIGHT_MOTOR_INVERTED` in `hardware_config.h` and re-flash. (It currently has
`RIGHT_MOTOR_INVERTED = true`.)

---

## 3. Power — read before connecting anything

| Connection | Goes to |
|---|---|
| L298N **+12 V** | Battery **+** (7–12 V, sized for your motors) |
| L298N **GND** | Battery **−** **and** Arduino **GND** |
| Arduino power | The **Pi's USB cable** (already powering it) |
| L298N **+5 V** out | **Leave disconnected** |

Three rules:

1. **Motors get their own supply.** Never run motors from the Pi's or Arduino's
   5 V rail — the current spikes will brown out the Pi and can corrupt the SD card.
2. **Common ground is mandatory.** The L298N ground and the Arduino ground must
   be joined, or the control signals have no reference and the driver behaves
   erratically. This is the single most commonly missed wire.
3. **Don't back-feed the Arduino.** Since the Arduino is USB-powered from the
   Pi, leave the L298N's 5 V output unconnected to avoid two supplies fighting.

---

## 4. E-stop (Arduino pin 2)

The firmware has `ESTOP_SENSE_PIN = 2`, `ESTOP_ACTIVE_LOW = true`, and the pin
is configured `INPUT_PULLUP`. So: **pin 2 pulled LOW = emergency stop active.**

```
   Arduino pin 2 ──────┬────── switch ────── Arduino GND
                       │
              (internal pull-up holds it HIGH when the switch is open)
```

Closing the switch pulls pin 2 to GND → the firmware latches `FAULT_ESTOP`,
zeroes the motors, and refuses to drive until you send `CLEAR`.

⚠ **The switch on pin 2 is the *second* layer, not the primary one.** The
physical E-stop must **also** interrupt motor power — put it in the battery
positive line feeding the L298N's +12 V. Reason: with a pull-up, a broken
signal wire reads as "not pressed", so the software path alone is not
fail-safe. Cutting power is.

**Right now you have no E-stop at all.** Telemetry reads `estop=0` only because
nothing is connected and the pull-up holds pin 2 high. Wire one before the
wheels ever touch the ground.

---

## 5. HC-SR04 ultrasonic → **Arduino** (current default)

| HC-SR04 | Arduino pin |
|---|---|
| VCC | 5 V |
| GND | GND |
| **TRIG** | **11** |
| **ECHO** | **12** |

**No voltage divider needed.** The Arduino is 5 V logic, so ECHO connects
directly. This is why the sensor lives here rather than on the Pi.

Two more reasons this is the better home for it:

- The obstacle stop is a **reflex on the Arduino** — it holds the motors at zero
  without waiting on the USB link or the Pi's control loop.
- It keeps the Pi's 3.3 V GPIO out of contact with a 5 V signal entirely.

⚠ **ECHO must never go on pin 2.** Pin 2 is the E-stop sense input, and it is
active-LOW. The HC-SR04's ECHO idles LOW, so wiring it there holds the E-stop
permanently engaged and the car can never arm. (Verified: it reported `estop=1`
on 120/120 samples and refused every ARM.)

The reflex is non-latching: the car stops while something is within
`ULTRASONIC_STOP_CM` (30 cm) and resumes by itself past `ULTRASONIC_CLEAR_CM`
(40 cm). Reversing away from an obstacle is still allowed. Tune these in
`hardware_config.h`, and set `USE_ULTRASONIC = false` to compile it out.

Distance appears in telemetry as `dist_cm`, with `obstacle=1` while held.

### Alternative: on the Pi instead

Only if you would rather not modify the firmware. Then it is TRIG → BCM 23
(pin 16), ECHO → BCM 24 (pin 18) **through a 1 kΩ / 2 kΩ divider**, driven by
`sensors/ultrasonic.py`. The divider is mandatory there — Pi GPIO is 3.3 V only.

---

## 6. Camera

The Camera Module 3 (IMX708) plugs into the Pi's **CSI** ribbon connector —
contacts facing the correct way, latch pushed down firmly. No GPIO wiring.

Verified working on this Pi.

---

## 7. Connection order (do it in this sequence)

1. Arduino ↔ Pi by USB. Confirm `/dev/ttyACM0` appears.
2. The 6 control wires, Arduino → L298N. **Jumpers removed.**
3. Common ground: L298N GND ↔ Arduino GND.
4. E-stop: switch to pin 2 + GND, **and** in the battery positive line.
5. Motors to OUT1–OUT4.
6. **Battery last**, and only with the wheels lifted off the ground.

---

## 8. Verify before applying motor power

```bash
cd ~/autocar
python3 tools/serial_check.py     # want: state=DISARMED, bad_frames=0
python3 tools/camera_check.py     # want: a captured frame, camera FREE
python3 -m pi.teleop --term       # want: ARMED, L/R targets ramp with WASD
```

All three pass with no motor power connected. Only then connect the battery,
**wheels still lifted**, and repeat the bench tests in `HARDWARE_SETUP.md`:
direction, E-stop cuts power, watchdog stops the motors, stall current.
