"""
autodrive.py - the top-level autonomous driving loop.

Wires everything together:
    camera -> lane detector + async YOLO -> driving brain -> motor link
plus a run recorder for later debugging.

Runs at ~HEARTBEAT_HZ. YOLO runs in its own thread and never stalls this loop.

Usage
-----
    # On the Pi, real hardware:
    python -m control.autodrive

    # On a laptop, no hardware, replay a video, watch the overlay:
    python -m control.autodrive --sim --source myclip.mp4 --show

    # On the Pi but motors disconnected (perception only, safe):
    python -m control.autodrive --sim --show

Flags
-----
    --sim         use the simulated motor link (no Arduino needed)
    --source X    camera source: int webcam index, video file, or 'picamera2'
    --show        display the annotated feed (needs a desktop / X)
    --no-detect   skip YOLO (lane-only, faster)
    --uturn-key   press 'u' in the --show window to request a U-turn

Safety: the loop always DISARMs the motors on exit, Ctrl-C, or any exception.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import cv2

try:
    from .. import config
except (ImportError, ValueError):
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config

from perception.camera import Camera
from perception.lane import LaneDetector
from perception.detect import Detector, DetectResult, draw_boxes
from control.state_machine import DrivingBrain, State
from pi.motor_link import MotorLink, decode_faults
from sensors.ultrasonic import Ultrasonic

try:
    from tools.recorder import RunRecorder
except Exception:
    RunRecorder = None


def parse_args():
    p = argparse.ArgumentParser(description="Autonomous driving loop")
    p.add_argument("--sim", action="store_true", help="simulated motor link")
    p.add_argument("--source", default=None, help="camera index / video / picamera2")
    p.add_argument("--show", action="store_true", help="show annotated feed")
    p.add_argument("--no-detect", action="store_true", help="disable YOLO")
    p.add_argument("--port", default=None, help="serial port override")
    p.add_argument("--record", action="store_true", help="record the run")
    return p.parse_args()


def main():
    args = parse_args()
    src = args.source
    if src is not None and src.isdigit():
        src = int(src)

    cam = Camera(source=src)
    lane_det = LaneDetector()
    detector = None if args.no_detect else Detector()
    ultra = Ultrasonic()
    brain = DrivingBrain()
    link = MotorLink(port=args.port, sim=args.sim, verbose=False)
    recorder = RunRecorder() if (args.record and RunRecorder) else None

    print(f"[autodrive] camera={cam.backend} sim={args.sim} "
          f"detect={'off' if args.no_detect else detector.enabled}")

    # graceful shutdown
    running = {"go": True}

    def _stop(*_):
        running["go"] = False
    signal.signal(signal.SIGINT, _stop)

    # arm
    link.clear(); time.sleep(0.2)
    link.arm(); time.sleep(0.3)

    period = 1.0 / config.HEARTBEAT_HZ
    empty_det = DetectResult()
    frames = 0
    t_start = time.time()

    try:
        while running["go"]:
            t0 = time.time()
            frame = cam.read()
            if frame is None:
                print("[autodrive] camera stream ended")
                break

            if detector is not None:
                detector.submit(frame)
                det = detector.result
            else:
                det = empty_det

            lane = lane_det.process(frame)
            throttle, steer, info = brain.update(lane, det)

            # Independent ultrasonic reflex: overrides everything and stops the
            # car if the front sensor sees something close. Fast, and needs
            # neither vision nor the Arduino.
            us_blocked = ultra.enabled and ultra.front_blocked()
            if us_blocked:
                throttle, steer = 0.0, 0.0
                info = "ULTRASONIC STOP"

            # hard safety: if firmware reports a fault, don't fight it
            if link.tel.faulted and not args.sim:
                throttle, steer = 0.0, 0.0

            link.set(throttle, steer)

            frames += 1
            if args.show or recorder:
                overlay = lane.overlay if lane.overlay is not None else frame
                if detector is not None:
                    draw_boxes(overlay, det)
                dtxt = ""
                if ultra.enabled:
                    dfront = ultra.distance("front")
                    dtxt = f" us={dfront*100:.0f}cm" if dfront is not None else " us=--"
                banner = (f"{brain.state.value} | {info} | thr={throttle:.2f} "
                          f"steer={steer:+.2f} off={lane.offset:+.2f} "
                          f"conf={lane.confidence:.1f}{dtxt}")
                cv2.putText(overlay, banner, (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                if link.tel.faulted:
                    cv2.putText(overlay, "FAULT:" + ",".join(
                        decode_faults(link.tel.fault_mask)), (8, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                if recorder:
                    recorder.write(overlay, brain.state.value, throttle, steer,
                                   lane, link.tel)
                if args.show:
                    cv2.imshow("autodrive", overlay)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        break
                    if k == ord("u"):
                        brain.request_uturn()
                        print("[autodrive] U-turn requested")

            sleep = period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        link.set(0.0, 0.0)
        link.close()
        if detector is not None:
            detector.close()
        if ultra.enabled:
            ultra.close()
        cam.release()
        if recorder:
            recorder.close()
        if args.show:
            cv2.destroyAllWindows()
        dur = time.time() - t_start
        print(f"[autodrive] stopped. {frames} frames in {dur:.1f}s "
              f"({frames / dur:.1f} FPS). Motors disarmed.")


if __name__ == "__main__":
    main()
