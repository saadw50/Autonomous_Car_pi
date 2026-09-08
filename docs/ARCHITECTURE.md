# Architecture

## Processes / threads

```
main loop (control/autodrive.py) ~20 Hz
├─ camera.read()                 blocking, fast
├─ detector.submit(frame)        hands newest frame to the detector thread
├─ detector.result               instant read of the last YOLO result
├─ lane.process(frame)           lane offset + confidence
├─ brain.update(lane, det)       state machine -> (throttle, steer)
└─ link.set(throttle, steer)     updates the heartbeat target

Detector thread (perception/detect.py) ~6 Hz
└─ runs YOLO on the latest frame, writes DetectResult; never blocks the loop

MotorLink threads (pi/motor_link.py)
├─ heartbeat thread  20 Hz: DRIVE,seq,left,right,ttl
└─ reader thread     parses ACK/NACK/TEL, keeps latest Telemetry

Arduino (firmware)
└─ its own loop: safety monitor, serial, motor update, telemetry, LED
```

The control loop never waits on YOLO, and the firmware watchdog is fed by the
heartbeat independent of frame timing. Those two decisions are what keep the car
smooth and safe.

## The driving state machine

```
                 obstacle close (centre)
      ┌──────────────────────────────────────► OBSTACLE_STOP ──┐
      │                                          ▲   │ clear ≥ 0.5s
      │                                          │   ▼
   LANE_FOLLOW ── stop sign close ─► STOP_SIGN ──┴──►(resume, cooldown 8s)
      │  ▲            (hold 3s)
      │  │
      │  └───────────────── U-turn done ◄── UTURN
      │                                       ▲
      ├── 'u' key / API request ──────────────┤
      │                                       │
      └── lane lost ─► LANE_LOST ── lost >2s ──┘  (dead-end auto U-turn)
                          │ coast to stop
                          └► lane found ─► LANE_FOLLOW
```

Priority each tick: **U-turn in progress** > **obstacle** > **stop sign** >
**lane lost** > **lane follow**. All thresholds and timings live in `config.py`.

## Why these choices

- **Lane centre = midpoint of two fitted lines**, not the mean of all Hough
  points. The old code averaged every segment's `x1`, mixing both lanes and
  noise into one jittery number.
- **LAB-L + Otsu**, not a fixed threshold of 210. Adapts to lighting instead of
  failing the moment the light changes.
- **Async detection** so a 150 ms inference doesn't become a 150 ms hole in
  steering.
- **Heartbeat driver** so perception hiccups never starve the firmware watchdog.
- **Everything DISARMs on exit** — the safe state is the default, not an
  afterthought.

## What's deliberately not here yet

Encoders / odometry, IMU fusion, SLAM, and mapping. The U-turn is open-loop and
timed because there are no encoders. Encoders are the correct next project: they
unlock closed-loop speed and odometry, and everything mapping-related depends on
them. See the 12-week roadmap PDF in this folder.
