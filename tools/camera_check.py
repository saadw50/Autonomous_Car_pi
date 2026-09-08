"""
camera_check.py - verify the camera and help you aim it at the track.

    python3 tools/camera_check.py             # one capture + lane result
    python3 tools/camera_check.py --loop      # live confidence readout (SSH-friendly)
    python3 tools/camera_check.py --fps       # measure capture frame rate

The --loop mode prints a one-line live readout, so you can point the camera at
the track over SSH with no screen and watch the lane confidence rise. Aim for
conf 1.0 (both tape edges) with the lane centred.

Saves data/live.jpg and data/live_overlay.jpg so you can copy them off and look.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from perception.camera import Camera
from perception.lane import LaneDetector


def bar(value, width=20):
    n = int(max(0.0, min(1.0, value)) * width)
    return "#" * n + "-" * (width - n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="picamera2")
    ap.add_argument("--loop", action="store_true", help="continuous readout")
    ap.add_argument("--fps", action="store_true", help="measure frame rate")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    src = args.source
    if src.isdigit():
        src = int(src)

    try:
        cam = Camera(source=src)
    except Exception as e:
        print("FAILED to open the camera:", e)
        print("  * is another process holding it?   fuser /dev/video0")
        print("  * is it detected?                  rpicam-hello --list-cameras")
        return 1
    print("camera backend :", cam.backend)

    frame = cam.read()
    if frame is None:
        print("FAILED: camera opened but returned no frame")
        cam.release()
        return 1
    print("frame          :", frame.shape, frame.dtype)

    os.makedirs("data", exist_ok=True)
    cv2.imwrite("data/live.jpg", frame)

    det = LaneDetector()
    r = det.process(frame)
    print("lane           : offset=%+.3f conf=%.2f L=%s R=%s"
          % (r.offset, r.confidence, r.left_found, r.right_found))
    cv2.imwrite("data/live_overlay.jpg", r.overlay)
    print("saved          : data/live.jpg, data/live_overlay.jpg")

    if args.fps:
        n = 30
        t0 = time.time()
        for _ in range(n):
            cam.read()
        dt = time.time() - t0
        print("capture rate   : %.1f FPS (%.0f ms/frame)" % (n / dt, 1000 * dt / n))

    if args.loop:
        print("\nLive readout - point the camera at the track. Ctrl-C to stop.")
        print("Aim for conf 1.00 (both edges) with offset near 0.\n")
        t_end = time.time() + args.seconds
        try:
            while time.time() < t_end:
                f = cam.read()
                if f is None:
                    break
                res = det.process(f)
                mark = "BOTH" if res.confidence >= 1.0 else (
                    "one " if res.confidence > 0 else "NONE")
                sys.stdout.write(
                    "\r  conf %.2f [%s] %s  offset %+.2f  L=%-5s R=%-5s   "
                    % (res.confidence, bar(res.confidence), mark, res.offset,
                       res.left_found, res.right_found))
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        print()
        cv2.imwrite("data/live.jpg", cam.read() if cam.read() is not None else frame)

    cam.release()
    print("\nOK - camera works.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
