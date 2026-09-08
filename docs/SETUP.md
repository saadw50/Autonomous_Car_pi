# Setup & bring-up guide

The Pi at the time of the last audit: Raspberry Pi 5, 8 GB, Raspberry Pi OS
(Debian 12 bookworm, 64-bit), Camera Module 3 (IMX708). Software already present:
OpenCV 4.12, NumPy 2.4, Ultralytics 8.3.223, PyTorch 2.9 (CPU), Picamera2,
pyserial 3.5. So most of the stack is installed — the steps below are mostly
config, not installs.

---

## 0. One-time Pi housekeeping (do these yourself over SSH)

These need `sudo`, so run them on the Pi. They free the camera and tidy up.

```bash
# The old CCTV recorder holds the camera and uploads to Google Drive.
# Stop it and stop it launching at boot (fully reversible):
sudo systemctl stop cctv.service
sudo systemctl disable cctv.service
pkill -f cctv.py; pkill -f 'rclone move'
find ~ -maxdepth 1 -name 'CHUNK_*.avi' -size 0 -delete   # remove empty recordings

# (Re-enable later if you ever want the CCTV back:)
#   sudo systemctl enable --now cctv.service

# Confirm the camera is now free:
rpicam-hello --list-cameras          # should list the imx708
fuser /dev/video0 || echo "camera free"
```

Optional but recommended before you start Phase 1:

```bash
# Bigger swap (guards against OOM during YOLO export). Reversible.
sudo dphys-swapfile swapoff
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup && sudo dphys-swapfile swapon
```

**Do NOT run a full `apt dist-upgrade` mid-project.** 300+ packages are
upgradable; a big upgrade can shift the OpenCV/NumPy/Torch versions the whole
project depends on and needs a reboot. Do it deliberately at a checkpoint.

---

## 1. Deploy this project to the Pi

From the laptop, in `D:\Autonomus Car\autocar`:

```bash
# Windows PowerShell / Git Bash. Replace the IP if it changed.
scp -r . pi@192.168.0.202:~/autocar
```

Or use `tools/deploy.sh` (from Git Bash) which also runs the housekeeping above.

Passwordless SSH (so you stop typing the password) — generate a key on the
laptop and install it:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""      # if you don't have one
ssh-copy-id pi@192.168.0.202                            # or paste the .pub into ~/.ssh/authorized_keys
```

Then change the default password on the Pi: `passwd`.

---

## 2. Install the remaining Python deps on the Pi

```bash
# System libs (from APT, not pip) if any are missing:
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-serial

# NCNN for the fast YOLO path (Phase 4):
pip install --break-system-packages ncnn
```

---

## 3. Prove each layer, in order

```bash
cd ~/autocar

# a) CRC agrees with the firmware (no hardware):
python tools/test_crc.py

# b) Control stack against a fake Arduino (no hardware):
python -m pi.motor_link --sim

# c) Camera works and is free:
python perception/camera.py picamera2

# d) PLUG THE ARDUINO INTO THE PI'S USB, then prove the real link:
python tools/serial_check.py           # expect state=DISARMED, no bad_frames

# e) First real drive — WHEELS OFF THE GROUND:
python -m pi.teleop --term             # WASD; space=stop; x=disarm; q=quit
```

If `serial_check.py` finds no port, the Arduino isn't enumerating — check the
USB cable (must be data, not charge-only) and `ls /dev/ttyACM* /dev/ttyUSB*`.

---

## 4. Perception & scoring (Phase 2)

```bash
python tools/capture_frames.py                 # SPACE to grab frames across lighting
python perception/score_detector.py --label    # click the true lane centre in each
python perception/score_detector.py            # MAE + failure rate; target <10% fail
```

Iterate on `config.py` (ROI, thresholds, Hough) and re-score until it's good.

---

## 5. Autonomous driving (Phases 3–4)

```bash
# Perception-only dry run (motors simulated) with the live overlay:
python -m control.autodrive --sim --show

# Full autonomy, recording the run, wheels ON the ground in a safe space:
python -m control.autodrive --show --record
# press 'u' in the window to command a U-turn; 'q' to stop.
```

Tune the driving in `config.py`: `KP`/`KD`, `BASE_THROTTLE`, the stop-sign and
obstacle area thresholds, and the U-turn timings (`UTURN_*` — measure these on
your actual car with the wheels down).

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `serial_check` finds no port | Arduino not plugged in / charge-only cable | data USB cable; check `dmesg` |
| Camera "device busy" | `cctv.service` still running | step 0 |
| Motors never move in teleop | firmware in FAULT/ESTOP | press `r` (CLEAR+ARM); check E-stop pin |
| Car stutters / stops randomly | heartbeat starved | don't block the loop; keep `DRIVE_TTL_MS` > loop period |
| YOLO ~1–2 FPS | PyTorch on CPU | export to NCNN, `YOLO_IMGSZ=320` |
| Lane jumps around | ROI/threshold off | re-score with `score_detector.py`, tune `config.py` |
