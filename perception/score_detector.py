"""
score_detector.py - offline lane-detector scoring (Phase 2).

You label the true lane-centre x for each test frame once; this then reports
mean absolute error and failure rate for the current detector. EVERY change to
lane.py is judged by this number instead of by eye.

Workflow
--------
1. Capture frames:      python tools/capture_frames.py
2. Label them:          python perception/score_detector.py --label
   (click the true lane centre at the bottom of each frame; s=skip, q=quit)
   -> writes data/labels.csv
3. Score the detector:  python perception/score_detector.py
   -> prints MAE (pixels & normalized) and failure rate

A "failure" is a frame where the detector reports no confident lane, or is off
by more than FAIL_PX pixels.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from perception.lane import LaneDetector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAMES = os.path.join(ROOT, "data", "test_frames")
LABELS = os.path.join(ROOT, "data", "labels.csv")
FAIL_PX = 60


def label():
    files = sorted(glob.glob(os.path.join(FRAMES, "*.jpg")))
    if not files:
        print("No frames in", FRAMES, "- run tools/capture_frames.py first.")
        return
    rows = {}
    if os.path.exists(LABELS):
        with open(LABELS) as f:
            for r in csv.reader(f):
                if r:
                    rows[r[0]] = r[1]
    state = {"x": None}

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["x"] = x

    cv2.namedWindow("label")
    cv2.setMouseCallback("label", on_click)
    for path in files:
        name = os.path.basename(path)
        img = cv2.resize(cv2.imread(path), (config.CAM_WIDTH, config.CAM_HEIGHT))
        state["x"] = None
        while True:
            disp = img.copy()
            cv2.putText(disp, f"{name}: click TRUE lane centre. n=next s=skip q=quit",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            if state["x"] is not None:
                cv2.line(disp, (state["x"], 0), (state["x"], disp.shape[0]),
                         (0, 255, 0), 1)
            cv2.imshow("label", disp)
            k = cv2.waitKey(20) & 0xFF
            if k == ord("q"):
                cv2.destroyAllWindows()
                _save(rows)
                return
            if k == ord("s"):
                break
            if k == ord("n") and state["x"] is not None:
                rows[name] = str(state["x"])
                break
    cv2.destroyAllWindows()
    _save(rows)


def _save(rows):
    with open(LABELS, "w", newline="") as f:
        w = csv.writer(f)
        for k, v in sorted(rows.items()):
            w.writerow([k, v])
    print(f"Saved {len(rows)} labels to {LABELS}")


def score():
    if not os.path.exists(LABELS):
        print("No labels. Run with --label first.")
        return
    det = LaneDetector()
    errs, fails, total = [], 0, 0
    with open(LABELS) as f:
        for name, truth in csv.reader(f):
            path = os.path.join(FRAMES, name)
            if not os.path.exists(path):
                continue
            total += 1
            img = cv2.resize(cv2.imread(path),
                             (config.CAM_WIDTH, config.CAM_HEIGHT))
            det._last_center = None  # score each frame independently
            r = det.process(img)
            if r.confidence <= 0:
                fails += 1
                continue
            e = abs(r.lane_center_px - float(truth))
            errs.append(e)
            if e > FAIL_PX:
                fails += 1
    if total == 0:
        print("No matching frames.")
        return
    mae = np.mean(errs) if errs else float("nan")
    print(f"Frames scored : {total}")
    print(f"MAE           : {mae:.1f} px  ({mae / (config.CAM_WIDTH/2):.3f} normalized)")
    print(f"Failure rate  : {fails}/{total} = {100*fails/total:.1f}%  "
          f"(fail = no lane or >|{FAIL_PX}px|)")
    print("Target        : MAE < ~30px, failure rate < 10%")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", action="store_true")
    args = ap.parse_args()
    label() if args.label else score()
