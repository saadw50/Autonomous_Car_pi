"""
camera.py - one camera interface, three backends.

The rest of the code just calls Camera().read() and gets a BGR frame (OpenCV
order). Under the hood this uses, in order of preference:

  1. Picamera2  - the correct backend on the Pi for the Camera Module 3 (IMX708).
                  We configure it for "RGB888", which Picamera2 delivers in BGR
                  byte order - exactly what OpenCV wants. (The old yolotest.py
                  bug was feeding the raw XRGB array straight to YOLO/imshow.)
  2. cv2.VideoCapture(index)  - a USB webcam / laptop cam for development.
  3. A video file  - replay recorded footage on a laptop with no camera at all.

Pick a backend explicitly with source=, or leave it None to auto-detect.
"""

from __future__ import annotations

import time

import cv2

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


class Camera:
    def __init__(self, source=None, width=None, height=None, fps=None):
        self.w = width or config.CAM_WIDTH
        self.h = height or config.CAM_HEIGHT
        self.fps = fps or config.CAM_FPS
        self.backend = None
        self._picam = None
        self._cap = None
        self._is_file = False

        if isinstance(source, str) and source not in ("picamera2", "auto"):
            # treat as a video file / device path
            self._open_file_or_device(source)
        elif source == "picamera2" or source is None:
            if not self._open_picamera2() and not self._open_webcam(0):
                raise RuntimeError("No camera backend available. Pass a video "
                                   "file path as source= to run on a laptop.")
        elif isinstance(source, int):
            if not self._open_webcam(source):
                raise RuntimeError(f"Could not open camera index {source}")

    # -- backends -----------------------------------------------------------
    def _open_picamera2(self) -> bool:
        try:
            from picamera2 import Picamera2
        except Exception:
            return False
        try:
            self._picam = Picamera2()
            cfg = self._picam.create_preview_configuration(
                main={"format": "RGB888", "size": (self.w, self.h)})
            self._picam.configure(cfg)
            self._picam.start()
            time.sleep(0.5)
            self.backend = "picamera2"
            return True
        except Exception as e:
            print("[camera] picamera2 unavailable:", e)
            self._picam = None
            return False

    def _open_webcam(self, index: int) -> bool:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self._cap = cap
        self.backend = f"webcam:{index}"
        return True

    def _open_file_or_device(self, path: str):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open source '{path}'")
        self._cap = cap
        self._is_file = True
        self.backend = f"file:{path}"

    # -- read ---------------------------------------------------------------
    def read(self):
        """Return a BGR frame (HxWx3 uint8), or None if the stream ended."""
        if self._picam is not None:
            frame = self._picam.capture_array()
            # RGB888 from picamera2 is already BGR-ordered for OpenCV; if a
            # 4-channel XRGB slips through, drop the alpha.
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]
            if frame.shape[1] != self.w or frame.shape[0] != self.h:
                frame = cv2.resize(frame, (self.w, self.h))
            return frame
        if self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                if self._is_file:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop the file
                    ok, frame = self._cap.read()
                if not ok:
                    return None
            if frame.shape[1] != self.w or frame.shape[0] != self.h:
                frame = cv2.resize(frame, (self.w, self.h))
            return frame
        return None

    def release(self):
        if self._picam is not None:
            try:
                self._picam.stop(); self._picam.close()
            except Exception:
                pass
        if self._cap is not None:
            self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()


if __name__ == "__main__":
    import sys
    src = sys.argv[1] if len(sys.argv) > 1 else None
    if src and src.isdigit():
        src = int(src)
    cam = Camera(source=src)
    print("Backend:", cam.backend)
    frame = cam.read()
    print("Frame:", None if frame is None else frame.shape)
    cam.release()
