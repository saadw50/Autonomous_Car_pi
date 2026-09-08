# Hardware setup & wiring

## Bill of materials (typical)

- Raspberry Pi 5 (8 GB) — perception + planning
- Arduino Uno — motor control + safety (runs the firmware)
- Motor driver: L298N or TB6612 (differential drive, 2 brushed DC motors)
- Camera Module 3 (IMX708) on the Pi's CSI port
- A ranging sensor for Phase 4: HC-SR04 (cheap) or a VL53L0X ToF (better)
- Physical E-stop switch in the motor-power path
- Separate battery for motors (do NOT power motors from the Pi/Arduino 5 V)
- USB A–B cable, Pi ↔ Arduino (data-capable, not charge-only)

## Connections

```
Pi  ── USB ──► Arduino            (serial, 115200; also powers the Uno logic)
Arduino ─ pins ─► motor driver ─► motors
Arduino ◄─ E-stop sense (pin 2)
Physical E-stop ─► cuts driver ENABLE / motor power directly (hardware layer)
Pi CSI ─► Camera Module 3
Arduino ─► ranging sensor (Phase 4)   # keep the reflex on the Arduino, not the Pi
```

## Firmware pin map (from hardware_config.h)

| Function | Pin |
|---|---|
| Left PWM | 5 |
| Left IN1 / IN2 | 7 / 8 |
| Right PWM | 6 |
| Right IN1 / IN2 | 9 / 10 |
| E-stop sense | 2 (INT0, active-low, INPUT_PULLUP) |
| Status LED | LED_BUILTIN |

## Two firmware tweaks recommended before real driving

The firmware is already strong (CRC, watchdog, fault latching, ramp limiting).
Two changes make it safer — do them wheels-up and re-run the bench tests:

1. **Move PWM off Timer0 pins.** Pins 5 and 6 share Timer0 with `millis()`, and
   `analogWrite(pin, 0)` there can emit a narrow pulse instead of a clean 0% —
   a motor that twitches when commanded dead. Move PWM to pins **9/10** (Timer1)
   and the direction pins to 5/6. Timer1 also lets you raise the PWM frequency
   out of the audible band later. (Edit the pin constants in `hardware_config.h`.)

2. **Make the E-stop interrupt-driven.** `ESTOP_SENSE_PIN = 2` is INT0. Attach a
   `CHANGE` interrupt that calls `forceMotorPinsSafe()` directly, so E-stop
   latency doesn't depend on the main loop (or a blocking `Serial.print`). Add a
   short debounce (require ~3 consecutive active reads) so a bouncing switch
   doesn't latch `FAULT_ESTOP` spuriously.

Neither is required to start Phase 1 with the wheels up, but both should land
before the car drives on the floor.

## HC-SR04 ultrasonic sensors (independent obstacle reflex)

These connect to the **Pi GPIO** (not the Arduino), so the obstacle-stop works
with no Arduino and no vision. Driver: `sensors/ultrasonic.py`; pins in
`config.ULTRASONIC_SENSORS`. You can wire several (front/left/right).

**⚠ Voltage warning:** the HC-SR04 `ECHO` pin outputs **5 V**, but Pi GPIO is
**3.3 V only**. You MUST drop ECHO with a voltage divider or you can damage the
Pi. TRIG can be driven directly at 3.3 V.

Per sensor:
```
HC-SR04 VCC  -> Pi 5V (pin 2 or 4)
HC-SR04 GND  -> Pi GND
HC-SR04 TRIG -> Pi GPIO (BCM 23 for 'front')      # direct, 3.3V is fine
HC-SR04 ECHO -> [ R1=1k ] -> Pi GPIO (BCM 24)      # divider node -> Pi
                               |
                            [ R2=2k ] -> GND        # 5V*2k/(1k+2k) = 3.3V
```

Default pins (BCM): front TRIG=23 ECHO=24. Uncomment the left/right entries in
`config.ULTRASONIC_SENSORS` and pick free GPIOs for more sensors.

Test once wired (on the Pi):
```bash
python3 sensors/ultrasonic.py          # prints live distances; wave your hand
```
Tune `ULTRASONIC_STOP_M` / `ULTRASONIC_CLEAR_M` in `config.py` for the stop and
resume distances (hysteresis prevents chattering at the threshold).

## Bench acceptance tests (Phase 0 — all must pass, wheels lifted)

| Test | Pass condition |
|---|---|
| Direction | positive command ⇒ both wheels forward (flip `*_MOTOR_INVERTED` only) |
| Hardware E-stop | E-stop cuts motor power even while commanding full throttle |
| Software E-stop | pin 2 asserted ⇒ state ESTOP, PWM 0 within ~20 ms on a scope |
| Watchdog | unplug USB mid-drive ⇒ motors stop within `ttl_ms` (≤250 ms) |
| Stall current | measured per channel; driver + supply have headroom |
| CRC rejection | corrupt frame ⇒ NACK BAD_CRC, `bad_frames` increments, armed car stops |
| Ramp | 0→max takes ~225 ms; every stop path bypasses the ramp |

Capture a `TEL` log of each test (`tools/recorder.py` or `serial_check.py`).
