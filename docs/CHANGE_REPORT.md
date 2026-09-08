# Change Report

**Scope:** every change made by Claude relative to the inherited project — the
work of ~8 people over roughly a year. Standing policy: *nothing inherited is
edited or deleted without saying so here.*

Date of this report: **2026-09-06**

---

## 1. Bottom line

**No inherited file has been modified or deleted.** Everything added lives in
one new folder, `autocar/`, sitting beside the original tree. The original code
runs exactly as it did before.

Two changes were made outside that folder, both to *system state* on the Pi (not
to code), both reversible, both listed in §4.

---

## 2. Inherited work — verified intact

| Path | What it is | Status |
|---|---|---|
| `firmware/arduino_motor_controller/*.ino`, `hardware_config.h` | Arduino motor firmware (CRC framing, watchdog, fault latching, ramp limiting) | **untouched** |
| `src/motor_protocol.py` (143 lines) | Frame encode/decode, CRC-16/CCITT-FALSE, `ProtocolError` | **untouched** |
| `src/motor_link.py` (313 lines) | Pi-side serial client: two-tier timeouts, request/ACK transaction locking, injectable serial factory | **untouched** |
| `tests/test_motor_protocol.py`, `tests/test_motor_link.py` | Unit tests for the above | **untouched — run and passing** |
| `tools/manual_motor_test.py` | Manual motor bring-up tool | **untouched** |
| `lane.py` (root) | Lane detection with slope filtering + left/right split | **untouched** |
| `New/lane_tester.py`, `New/move new.py` | Earlier lane experiments | **untouched** |
| `yolo_test.py`, `gpu_test.py`, `check_color_windows.py` | Test scripts | **untouched** |
| `FIRMWARE_SETUP.md` | Hardware/firmware setup guide | **untouched** |
| `Negative/` (300 images), `New/*.jpg` | Track image dataset | **untouched** (read only) |
| `Cource/` (78 lecture videos) | Reference course the project follows | **untouched** |
| `output/`, `tmp/`, `Autonomous_Car_6_Month_Roadmap.pdf` | Prior roadmap + build artefacts | **untouched** |

**Verification performed:** `python -m pytest tests/ -q` →
**8 passed, 3 subtests passed.** The inherited motor layer is healthy.

---

## 3. What was ADDED (all new files, all inside `autocar/`)

Nothing here replaces anything; it is a parallel stack.

| File | Purpose |
|---|---|
| `config.py` | All tunable constants in one place |
| `pi/motor_link.py` | Serial driver + `SimSerial` fake Arduino ⚠ *see §5 — duplicates `src/`* |
| `pi/teleop.py` | Keyboard driving (GUI + SSH terminal) |
| `perception/camera.py` | Picamera2 / webcam / video-file camera abstraction |
| `perception/lane.py` | Lighting-robust lane detector |
| `perception/detect.py` | Async YOLO object + stop-sign detection |
| `perception/score_detector.py` | Offline detector scoring + labelling tool |
| `control/controller.py` | PD steering + throttle shaping |
| `control/state_machine.py` | Lane-follow / stop-sign / obstacle / U-turn brain |
| `control/autodrive.py` | Top-level autonomous loop |
| `sensors/ultrasonic.py` | HC-SR04 driver (Pi GPIO), independent obstacle reflex |
| `sim/track_sim.py` | **Simulator**: synthetic track, camera render, vehicle model |
| `sim/run_sim.py` | **Closed-loop simulation runner** (no hardware needed) |
| `sim/replay_sim.py` | **Simulation on the REAL camera footage** (`Negative1..209`). `--mode replay` plays the recorded run; `--mode reactive` closes the loop by warping each real frame to the car's simulated lateral offset and heading |
| `tools/test_crc.py` | Proves CRC matches the firmware |
| `tools/serial_check.py` | Pi↔Arduino link bring-up check |
| `tools/capture_frames.py` | Build a lane test set |
| `tools/recorder.py` | Record video + telemetry per run |
| `tools/deploy.sh` | Deploy to the Pi |
| `docs/*` | SETUP, PROTOCOL, ARCHITECTURE, HARDWARE_SETUP, 3-month roadmap, this report |

---

## 4. Changes made OUTSIDE `autocar/` (system state on the Pi)

Both were run by the user on the Pi, and both are reversible.

| Change | Why | How to undo |
|---|---|---|
| `cctv.service` stopped and disabled | It held the camera open (V4L2) and uploaded chunks to Google Drive via rclone, so no other program could use the camera | `sudo systemctl enable --now cctv.service` |
| Deleted **0-byte** `CHUNK_*.avi` files in `~` | Failed/empty recordings, no data in them | nothing to restore — they were empty |

Also created on the Pi: `~/autocar/` (new, holds the uploaded copy of this
project) and `~/autocar/data/sample/` (copies of 10 dataset images).
`~/cctv.py` and `~/calibrate_color.py` were **not modified**.

> The Pi is currently **offline/unreachable**; all further work is local.

---

## 4b. Decisions taken by the user (2026-09-06)

| Question | Decision | Consequence |
|---|---|---|
| Consolidate the motor layer onto the team's `src/`? | **Keep both for now** | `src/motor_link.py` and `src/motor_protocol.py` stay **untouched**. `autocar/` continues to use `autocar/pi/motor_link.py`. The duplication in §5 is accepted, not resolved — revisit before hardware bring-up. |
| `LANE_LOST` recovery behaviour? | **Leave as-is (stop and wait)** | The car halts when it loses the lane and waits for the dead-end U-turn trigger. Safest option; accepts multi-second freezes. §8 stays open by choice, not by oversight. |
| Reconcile the two roadmaps? | **Use the 3-month roadmap** | The 3-month plan is the working plan; the team's 6-month PDF is kept as history. The 3-month PDF has been regenerated to reflect current status and an offline-first sequence. |

## 5. ⚠ Duplication I introduced — accepted for now (see §4b)

I built `autocar/` before fully surveying `src/`. As a result some new code
**overlaps existing work**. This is the main thing to resolve.

| New file | Overlaps | Assessment |
|---|---|---|
| `autocar/pi/motor_link.py` | `src/motor_link.py` + `src/motor_protocol.py` | **The inherited version is better.** It has two-tier timeouts (fall to zero, then DISARM), request/ACK transaction locking that guarantees sequence ordering across threads, a `ProtocolError` type, an injectable `serial_factory`, and **unit tests**. Mine adds only a `SimSerial` fake and a simpler API. |
| `autocar/perception/lane.py` | root `lane.py` | Root `lane.py` already did slope filtering and left/right splitting. Mine adds the lighting robustness (CLAHE, top-hat, brightness gate, on-track ratio test, line-support guard, confidence + smoothing). |
| `autocar/docs/HARDWARE_SETUP.md` | `FIRMWARE_SETUP.md` | Overlapping hardware documentation. |
| `autocar/docs/Autonomous_Car_3_Month_Roadmap.pdf` | `Autonomous_Car_6_Month_Roadmap.pdf` | Two roadmaps now exist with different horizons. |

**Recommendation:** keep the team's `src/motor_link.py` + `src/motor_protocol.py`
as the motor layer (it is the stronger implementation and it is tested), and
port only the `SimSerial` fake onto it so the simulator still runs. Retire
`autocar/pi/motor_link.py`. I have **not** done this yet — it changes which code
is authoritative, so it is your call.

---

## 6. Fixes found and applied to the new code

| Issue | Where | Fix |
|---|---|---|
| `HoughLinesP` returns `(N,1,4)` on OpenCV 4 (Pi) but `(N,4)` on OpenCV 5 (laptop) | `perception/lane.py` | Normalize with `reshape(-1, 4)` so one codebase runs on both |
| Absolute brightness threshold for "on track" broke when the scene brightened (auto-exposure would trigger this on the real car) | `perception/lane.py`, `config.py` | Replaced with an exposure-invariant **ratio** (ROI median ÷ p95). Caught by the simulator, not by the static images. |
| Lane detector failed under changing light (original used a fixed threshold of 210) | `perception/lane.py` | CLAHE + white top-hat + brightness gate + line-support guard |
| State machine measured time with wall-clock `time.time()`, so timed manoeuvres (stop-sign hold, U-turn duration, obstacle clear) ran for the wrong number of steps in simulation and could not be tuned off-hardware | `control/state_machine.py`, `control/controller.py`, `sim/run_sim.py` | Injectable `clock`. Real car passes `time.time()`; the simulator passes its virtual clock. Also caught by the simulator. |
| U-turns chained: finishing a U-turn left the car briefly without a lane, which immediately re-fired the dead-end auto U-turn, so the car turned repeatedly | `control/state_machine.py`, `config.py` | Added `UTURN_COOLDOWN_S` (6 s) suppressing only the *automatic* dead-end trigger after a U-turn, and cleared the lane-lost timer on exit. An explicit U-turn request still always works. |

---

## 7. Measured results (all reproducible offline)

- Inherited unit tests: **8 passed, 3 subtests passed**
- **Lane detector over the FULL 300-frame real dataset** (labels supplied by the user: 1–209 are lane frames, 210+ are not):
  - lane frames detected: **206/209 = 98.6 %**
  - non-lane frames rejected: **90/90 = 100 %** (zero false positives)
  - both lane edges locked (confidence 1.0): 72 = 34.4 % → the multi-lane selection gap
  - the only 3 misses (104, 105, 106) are *boundary* frames where a large bright wood table fills much of the view; the on-track guard correctly judges the car to be leaving the mat
- Lane detector on the earlier 10-frame lighting spread: **10/10**
- Lane detector across simulated lighting (dim 0.55× → bright 1.9×, heavy glare): **5/6**; the sixth (dim + glare + heavy noise) rejects, which is a *safe stop*, not a false lane
- On-track ratio separation: real lanes 0.295–0.427 vs real wood 0.714–0.902
- YOLO on the Pi 5: ~10 FPS @ imgsz=320 (measured while the Pi was online); stop sign = COCO class 11
- Closed-loop simulation: 25 s run, 6.7 m travelled, **100 % of steps on track**, pure `LANE_FOLLOW`
- Simulated behaviours: stop-sign hold measured at **exactly 3.0 s** (= `STOP_SIGN_HOLD_S`); obstacle stop holds while blocked; U-turn measured at **exactly 2.6 s** (= `UTURN_DURATION_S`) with no chained second turn

---

## 8. Known open issue — `LANE_LOST` recovery is weak

Found by the closed-loop simulator, **not yet fixed** (needs a design decision).

`LANE_LOST` commands throttle 0. Once the car has stopped and no lane is in
view, nothing moves it, so it cannot re-acquire the lane — the state is
effectively absorbing. Observed after a U-turn: the car sat stopped for 5.2 s
until the U-turn cooldown expired and the dead-end trigger could fire again.

Options:
1. **Creep and search** — in `LANE_LOST`, drive slowly forward (or rotate
   slowly) instead of stopping, so the camera sweeps for the lane. Best
   behaviour, slightly less safe.
2. **Stop, then search** — hold still briefly, then begin a slow search rotate
   if still lost. Safer, a compromise.
3. **Leave as-is** — stop and wait for the dead-end U-turn to recover. Safest,
   but the car freezes for seconds at a time.

Recommendation: option 2. **Decision taken: leave as-is** (see §4b).

**New evidence (real-footage replay).** Running the recorded run through the
full stack showed the concrete cost of this choice: at the three boundary
frames (104–106) the car stops, and because a stopped car cannot advance, the
lane-lost timer runs out and the dead-end trigger fires — **two full U-turns
(52 steps)** before it creeps past. On the real car this would look like the
vehicle freezing and spinning at the track edge.

This does not change the decision, but it should be stated as a known
behaviour in the report. Revisit if the demo shows it.

## 8b. Hardware link PROVEN (2026-09-08)

A replacement **Arduino Uno R3** was connected by USB (`2341:0043` →
`/dev/ttyACM0`); the Pi moved to **192.168.0.203**. The board arrived **blank**
(silent at 9600/57600/115200/250000) and was flashed with
`firmware/arduino_motor_controller` from the Arduino IDE. Verified afterwards on
real hardware, with **no motor power connected** (the safest bench state):

| Test | Result |
|---|---|
| `tools/serial_check.py` | state=DISARMED, estop=0, **bad_frames=0**, continuous telemetry |
| CLEAR / ARM | DISARMED → ARMED |
| Differential mix `(0.4, 0.0)` | tgt `(72, 72)` — exact |
| Differential mix `(0.4, 0.5)` | tgt `(162, -18)` — exact |
| Differential mix `(0.0, -0.5)` | tgt `(-90, 90)` — exact |
| **Watchdog** (heartbeat stopped) | **FAULT/WATCHDOG in 0.40 s** (250 ms TTL + 200 ms telemetry period) |
| CLEAR recovery | back to DISARMED |

This closes the last unverified item in §9 for the serial link. The firmware was
flashed **unmodified** — the Timer1/E-stop tweaks in `docs/HARDWARE_SETUP.md`
were deliberately not applied, so the baseline is proven before anything changes.

Note: `~/autocar/firmware/` on the Pi was empty until now — the firmware lives
outside `autocar/`, so earlier uploads never included it. Now synced.

## 8c. FIRMWARE MODIFIED — ultrasonic reflex (2026-09-08)

**This is the first change to inherited firmware.** It was made as a **copy**:

- `firmware/arduino_motor_controller/` (original, team's) — **untouched**, still dated Aug 27
- `autocar/firmware/arduino_motor_controller/` (modified copy) — the version now flashed

Requested by the user, who wired an HC-SR04 to **Arduino pins 11 (TRIG) and 12
(ECHO)**. The Arduino is 5 V logic, so ECHO needs no divider — which also solved
the "no resistors available" problem.

### Why the sensor moved to the Arduino
An earlier attempt put ECHO on **pin 2**, the E-stop sense input. Pin 2 is
active-LOW and the HC-SR04's ECHO idles LOW, so the E-stop was permanently
engaged: measured `estop=1` on **120/120** samples, `state=ESTOP`, and every ARM
refused. Fixed by moving to pins 11/12.

### What changed
| File | Change |
|---|---|
| `hardware_config.h` | Added `USE_ULTRASONIC`, pins 11/12, ping period, timeout, `STOP_CM`/`CLEAR_CM` hysteresis, confirm count, and three `static_assert`s (ECHO ≠ E-stop pin, clear > stop, ECHO on PORTB) |
| `arduino_motor_controller.ino` | Pin-change ISR on PCINT0 to time ECHO; non-blocking ping state machine; obstacle hold in `updateMotors()`; two new telemetry fields; wired into `setup()`/`loop()` |
| `pi/motor_link.py` | `Telemetry` gains `dist_cm` and `obstacle`, parsed only when present |
| `docs/PROTOCOL.md`, `docs/PINOUT.md` | Documented |

### Design decisions
- **Interrupt-timed, not `pulseIn()`.** `pulseIn` blocks up to ~25 ms, which would
  stall the serial and safety work in `loop()`; a polled read loses accuracy
  whenever Serial is transmitting telemetry (worst case ~1.5 ms ≈ 25 cm error).
- **Non-latching hold, not a fault.** An obstacle is a condition, not a failure:
  the car resumes by itself once clear, with no operator `CLEAR` needed.
  Reversing away from an obstacle is still permitted.
- **Timeout means "clear", not "blocked".** A missing or dead sensor must never
  wedge the car in a permanent stop.
- **Telemetry fields appended, not inserted** — old parsers keep working.

### Verification — **flashed and confirmed on real hardware (2026-09-08)**
- Host syntax check on the Pi: `g++ -fsyntax-only -std=c++17 -Wall` → **exit 0, no warnings**
- `tools/test_crc.py` → ALL PASS; `python -m pi.motor_link --sim` → fields default to 0 on old firmware

On the board, with **no motor power** connected:

| Test | Result |
|---|---|
| TEL field count | **14** (was 12) — new firmware confirmed |
| Distance readings | stable 25–28 cm, **43/43 samples valid**, no dropouts |
| `obstacle` flag | 1 while inside the 30 cm threshold |
| **Forward while blocked** | target `90` → output `0` — **reflex holds** |
| **Reverse while blocked** | target `-90` → output `-90` — **permitted**, so the car can back away |
| Watchdog (alongside the new ISR) | **FAULT in 0.30 s** — unaffected |
| CLEAR recovery | returns to DISARMED |

Note: a first flash attempt loaded the *original* firmware by mistake (identical
filename, different folder). Detected instantly from the 12-field telemetry.

Still unconfirmed: the **hysteresis release** (obstacle clearing above 40 cm) —
needs the obstacle physically moved away.

## 9. Still unverified

- **Real Pi↔Arduino serial handshake.** The Arduino is physically missing. `tools/serial_check.py` will prove it when one is available.
- Real-world driving of any kind (no motors have moved).
- The U-turn timing constants (`UTURN_*`) are guesses; they must be measured on the real car.
