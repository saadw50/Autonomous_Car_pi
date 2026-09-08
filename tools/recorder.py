"""
recorder.py - record a run for later debugging.

Writes, per run, into logs/run_YYYYmmdd_HHMMSS/ :
    video.avi        the annotated feed
    telemetry.csv    one row per frame: time, state, throttle, steer, lane
                     offset/confidence, and the firmware telemetry

"Without recorded runs you cannot debug a failure you saw once." This is that.
"""

from __future__ import annotations

import csv
import os
import time

import cv2

try:
    from .. import config
except (ImportError, ValueError):
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


class RunRecorder:
    def __init__(self, base=None, fps=None):
        base = base or config.LOG_DIR
        stamp = time.strftime("run_%Y%m%d_%H%M%S")
        self.dir = os.path.join(base, stamp)
        os.makedirs(self.dir, exist_ok=True)
        self.fps = fps or config.HEARTBEAT_HZ
        self._vw = None
        self._csv = open(os.path.join(self.dir, "telemetry.csv"), "w", newline="")
        self._w = csv.writer(self._csv)
        self._w.writerow(["t", "state", "throttle", "steer", "offset",
                          "confidence", "fw_state", "fault_mask",
                          "cur_l", "cur_r", "estop", "drive_age_ms"])
        self._t0 = time.time()
        print(f"[recorder] logging to {self.dir}")

    def write(self, frame, state, throttle, steer, lane, tel):
        if config.LOG_VIDEO:
            if self._vw is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self._vw = cv2.VideoWriter(
                    os.path.join(self.dir, "video.avi"), fourcc, self.fps, (w, h))
            self._vw.write(frame)
        if config.LOG_TELEMETRY:
            self._w.writerow([
                f"{time.time() - self._t0:.3f}", state, f"{throttle:.3f}",
                f"{steer:.3f}", f"{lane.offset:.3f}", f"{lane.confidence:.2f}",
                tel.state, tel.fault_mask, tel.cur_l, tel.cur_r, tel.estop,
                tel.drive_age_ms])

    def close(self):
        if self._vw is not None:
            self._vw.release()
        self._csv.close()
