# Autonomous Car

A Raspberry Pi 5 + Arduino prototype that follows a lane, detects objects, stops
for stop signs, and performs a U-turn. The Pi does perception and planning; the
Arduino runs safety-critical motor control over a CRC-framed serial protocol.

```
 Camera (IMX708) ─► lane.py ─┐
                             ├─► state_machine.py ─► controller.py ─► motor_link.py ─► Arduino ─► motors
 Camera ─► detect.py (YOLO) ─┘            (the brain)      (PD)          (serial)      (firmware)
                                                                            ▲
                                                                     recorder.py logs every run
```

## Layout

| Path | What it is | Runs on |
|---|---|---|
| `config.py` | Every tunable value, in one place | both |
| `pi/motor_link.py` | Serial driver: CRC-16 framing, 20 Hz heartbeat, `set(throttle, steer)` | Pi |
| `pi/teleop.py` | Keyboard drive (GUI or SSH terminal) | Pi |
| `perception/camera.py` | One camera API (Picamera2 / webcam / video file) | both |
| `perception/lane.py` | Lighting-adaptive lane detector (left/right slope-split fit) | both |
| `perception/detect.py` | Async YOLO — objects + stop signs, never blocks control | Pi |
| `perception/score_detector.py` | Offline lane-detector scoring | both |
| `control/controller.py` | PD steering + throttle shaping | both |
| `control/state_machine.py` | Driving brain: lane-follow / stop-sign / obstacle / U-turn | both |
| `control/autodrive.py` | Top-level loop that wires it all together | both |
| `tools/serial_check.py` | Prove the Pi ↔ Arduino link | Pi |
| `tools/test_crc.py` | Prove the CRC matches the firmware (no deps) | both |
| `tools/capture_frames.py` | Build the lane test set | both |
| `tools/recorder.py` | Record video + telemetry per run | both |
| `docs/` | Setup, protocol, hardware, architecture, and the 12-week roadmap PDF | — |

The Arduino firmware itself lives in your existing
`firmware/arduino_motor_controller/`. This project talks to it, unchanged (see
`docs/HARDWARE_SETUP.md` for the two recommended firmware tweaks).

## Simulator — develop with no hardware at all

`sim/` closes the loop with zero hardware: it renders a synthetic track from the
car's pose, runs the **real** perception / state machine / PD controller / serial
command path, and drives a vehicle model with the resulting motor commands.

```bash
python -m sim.run_sim                    # headless, prints a summary
python -m sim.run_sim --show             # watch it drive
python -m sim.run_sim --stop-sign        # test the stop-sign behaviour
python -m sim.run_sim --obstacle         # test the obstacle stop
python -m sim.run_sim --uturn-at 4       # test the U-turn
python -m sim.run_sim --brightness 1.6 --glare 0.5 --noise 0.2   # stress the lighting
python -m sim.run_sim --seconds 40 --save run.avi                # record a demo
```

Only the camera and the motors are synthetic — everything else is the code that
will run on the car. Use `--brightness/--glare/--noise` to test lighting
robustness; this is how the exposure-invariance bug in the on-track test was
found.

## Try it now, on the laptop (no hardware)

```bash
# 1. Prove the serial CRC matches the firmware — pure Python, no installs:
python tools/test_crc.py

# 2. Exercise the whole control stack against a simulated Arduino:
python -m pi.motor_link --sim
```

With `numpy`+`opencv` installed you can also run lane detection on your existing
images and replay clips:

```bash
python -m perception.lane "New/Negative100.jpg"
python -m control.autodrive --sim --source some_clip.mp4 --show --no-detect
```

## Bring-up order (see docs/SETUP.md for detail)

1. **Free the camera & deploy** — `tools/deploy.sh` copies this to the Pi and
   disables the old `cctv.service` that was hogging the camera.
2. **Prove the link** — plug the Arduino into the Pi's USB, then
   `python tools/serial_check.py`.
3. **First drive** — `python -m pi.teleop --term` (wheels lifted!).
4. **Perception** — `python tools/capture_frames.py`, then
   `python perception/score_detector.py --label` and `... score`.
5. **Autonomous** — `python -m control.autodrive --show`.

## Safety

- Every entry point DISARMs the motors on exit, Ctrl-C, and any exception.
- The Arduino watchdog stops the motors if the 20 Hz heartbeat lapses.
- Always do first drives with the **wheels off the ground**.
- Keep the physical E-stop in the motor-power path — software is the second layer.

See `docs/` for the full protocol, hardware notes, architecture, and roadmap.
