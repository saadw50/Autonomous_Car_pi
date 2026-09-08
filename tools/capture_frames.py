"""
capture_frames.py - build the lane-detection test set (Phase 2, week 5).

Saves still frames from the camera to data/test_frames/ so you can score the
lane detector offline and repeatably. Capture across lighting: bright, dim,
glare, shadow, partial occlusion.

    python tools/capture_frames.py                 # picamera2/webcam, press SPACE to grab
    python tools/capture_frames.py --source clip.mp4 --every 15   # grab every 15th frame
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

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "data", "test_frames")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=None)
    ap.add_argument("--every", type=int, default=0,
                    help="auto-save every Nth frame (0 = manual, SPACE to grab)")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    src = args.source
    if src and src.isdigit():
        src = int(src)

    cam = Camera(source=src)
    n = existing = len([f for f in os.listdir(OUT) if f.endswith(".jpg")])
    i = 0
    print(f"Saving to {OUT} (already {existing} frames). "
          f"{'auto every %d' % args.every if args.every else 'SPACE=save'}  q=quit")
    while True:
        frame = cam.read()
        if frame is None:
            break
        i += 1
        save = args.every and (i % args.every == 0)
        cv2.imshow("capture (SPACE=save, q=quit)", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord(" ") or save:
            path = os.path.join(OUT, f"frame_{n:04d}.jpg")
            cv2.imwrite(path, frame)
            n += 1
            print("saved", path)
    cam.release()
    cv2.destroyAllWindows()
    print(f"Total frames: {n}")


if __name__ == "__main__":
    main()
