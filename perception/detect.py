"""
detect.py - object + stop-sign detection that never blocks the control loop.

YOLO inference on a Pi is slow (a few FPS). The control loop must run at 20 Hz
regardless, so detection runs in a BACKGROUND THREAD: it grabs whatever the
latest frame is, runs the model, and writes the result into a shared slot the
control loop reads instantly. The loop never waits on inference.

(Torch releases the GIL during inference, so a thread gives real parallelism
here and avoids the cost of shipping frames between processes. If you later move
to a separate process, the public API below stays the same.)

Stop signs are COCO class 11 - already in yolov8n, no training needed.

Degrades gracefully: if ultralytics/torch is missing, Detector runs in a no-op
mode so the rest of the stack still starts (you just get no detections).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


@dataclass
class Box:
    cls: int
    conf: float
    xyxy: tuple            # (x1, y1, x2, y2) in pixels
    area_frac: float       # fraction of frame area
    cx_frac: float         # centre x as fraction of width (0..1)


@dataclass
class DetectResult:
    t_wall: float = 0.0
    boxes: list = field(default_factory=list)
    fps: float = 0.0

    def stop_signs(self):
        return [b for b in self.boxes if b.cls == config.COCO_STOP_SIGN]

    def obstacles(self):
        return [b for b in self.boxes if b.cls in config.COCO_OBSTACLES]


class Detector:
    """Async YOLO detector. Feed frames with submit(); read latest with result."""

    def __init__(self, model_path=None, imgsz=None, conf=None, rate_hz=None):
        self.model_path = model_path or config.YOLO_MODEL
        self.imgsz = imgsz or config.YOLO_IMGSZ
        self.conf = conf or config.YOLO_CONF
        self.period = 1.0 / (rate_hz or config.DETECT_HZ)

        self._model = None
        self._enabled = self._load()
        self._frame = None
        self._frame_lock = threading.Lock()
        self.result = DetectResult(t_wall=time.time())
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _load(self) -> bool:
        try:
            from ultralytics import YOLO
        except Exception as e:
            print("[detect] ultralytics not available -> detection disabled:", e)
            return False
        try:
            self._model = YOLO(self.model_path)
            return True
        except Exception as e:
            print("[detect] could not load model -> detection disabled:", e)
            return False

    def submit(self, frame):
        """Hand the detector the newest frame (cheap; just stores a reference)."""
        with self._frame_lock:
            self._frame = frame

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.time()
            if not self._enabled:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                frame = None if self._frame is None else self._frame.copy()
            if frame is None:
                time.sleep(0.02)
                continue
            try:
                boxes = self._infer(frame)
                dt = time.time() - t0
                self.result = DetectResult(t_wall=time.time(), boxes=boxes,
                                           fps=(1.0 / dt if dt > 0 else 0.0))
            except Exception as e:
                print("[detect] inference error:", e)
                time.sleep(0.1)
            # pace to the target rate
            sleep = self.period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _infer(self, frame) -> list:
        h, w = frame.shape[:2]
        area = float(h * w)
        res = self._model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                  verbose=False)
        out = []
        if not res:
            return out
        r = res[0]
        if r.boxes is None:
            return out
        for b in r.boxes:
            cls = int(b.cls[0])
            cf = float(b.conf[0])
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            a = ((x2 - x1) * (y2 - y1)) / area
            cx = ((x1 + x2) / 2) / w
            out.append(Box(cls, cf, (x1, y1, x2, y2), a, cx))
        return out

    def close(self):
        self._stop.set()

    @property
    def enabled(self):
        return self._enabled


def draw_boxes(frame, result: DetectResult, names=None):
    import cv2
    img = frame
    for b in result.boxes:
        x1, y1, x2, y2 = [int(v) for v in b.xyxy]
        col = (0, 0, 255) if b.cls == config.COCO_STOP_SIGN else (0, 200, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        label = f"{b.cls}:{b.conf:.2f}"
        cv2.putText(img, label, (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    return img


if __name__ == "__main__":
    import sys, cv2
    from camera import Camera  # when run as a script from perception/
    src = sys.argv[1] if len(sys.argv) > 1 else None
    if src and src.isdigit():
        src = int(src)
    cam = Camera(source=src)
    det = Detector()
    print("Detector enabled:", det.enabled, "| camera:", cam.backend)
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            det.submit(frame)
            r = det.result
            draw_boxes(frame, r)
            cv2.putText(frame, f"det fps: {r.fps:.1f}  signs:{len(r.stop_signs())}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("detect", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        det.close(); cam.release(); cv2.destroyAllWindows()
