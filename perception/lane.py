"""
lane.py - lane detection that survives changing light.

Fixes every problem in the original lane_tester.py / "move new.py":

  * Adaptive tape extraction (LAB L-channel + Otsu) instead of a hardcoded
    threshold of 210 that dies the moment the lighting changes.
  * Trapezoidal ROI, not a rectangle.
  * Segments split into LEFT and RIGHT lanes by slope sign, each fitted with a
    line (np.polyfit) and extrapolated to the bottom row. The lane centre is the
    midpoint of the two - NOT the mean of every Hough x1, which mixed both lanes
    and all the noise into one jittery number.
  * Holds the last good centre when one side vanishes, and reports a confidence.
  * Optional bird's-eye warp so offset can be reported as a physical-ish value.

The main entry point is LaneDetector.process(frame) -> LaneResult.
Everything is pure OpenCV/NumPy, so this module runs and can be scored on a
laptop against saved images (see tools/score_detector.py).
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


@dataclass
class LaneResult:
    offset: float          # normalized cross-track error in [-1, 1]; + = car left of centre
    confidence: float      # 0..1
    lane_center_px: float  # pixel x of the lane centre at the bottom row
    left_found: bool
    right_found: bool
    overlay: np.ndarray | None = None  # annotated frame for debugging/logging


class LaneDetector:
    def __init__(self, cfg=config):
        self.cfg = cfg
        self._last_center = None
        self._warp_M = None
        self._warp_size = None

    # -- preprocessing ------------------------------------------------------
    def _mask_tape(self, frame):
        """Return a binary image of the bright lane tape, robust to uneven
        light, glare, and wood-grain texture.

        White top-hat keeps thin bright structures (tape) and removes large
        bright regions (wood floor, glare, windows) and slow lighting
        gradients. CLAHE first so it behaves the same in dim and bright frames.
        Set config.USE_TOPHAT = False to fall back to the old global-Otsu path.
        """
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l = lab[:, :, 0]

        if not self.cfg.USE_TOPHAT:
            l = cv2.GaussianBlur(l, (5, 5), 0)
            thr, binary = cv2.threshold(l, 0, 255,
                                        cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            if self.cfg.LANE_L_BIAS:
                _, binary = cv2.threshold(l, min(255, thr + self.cfg.LANE_L_BIAS),
                                          255, cv2.THRESH_BINARY)
            return binary

        clahe = cv2.createCLAHE(clipLimit=self.cfg.CLAHE_CLIP,
                                tileGridSize=(8, 8))
        l = clahe.apply(l)
        k = self.cfg.TOPHAT_KERNEL
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        tophat = cv2.morphologyEx(l, cv2.MORPH_TOPHAT, kernel)
        # Otsu on the top-hat, but floor it at TOPHAT_MIN so flat/low-contrast
        # wood produces (almost) no mask instead of noise.
        thr, _ = cv2.threshold(tophat, 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        thr = max(thr, self.cfg.TOPHAT_MIN)
        _, binary = cv2.threshold(tophat, thr, 255, cv2.THRESH_BINARY)

        # Brightness gate: real tape is genuinely white. AND-ing with an
        # absolute brightness mask on the original L kills mid-gray wood grain,
        # whose streaks are thin (survive top-hat) but not white.
        bright = cv2.threshold(lab[:, :, 0], self.cfg.BRIGHT_MIN, 255,
                               cv2.THRESH_BINARY)[1]
        binary = cv2.bitwise_and(binary, bright)

        # clean speckle
        binary = cv2.morphologyEx(
            binary, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        return binary

    def _roi_poly(self, w, h):
        c = self.cfg
        return np.array([[
            (int(c.ROI_BOTTOM_LEFT * w), h),
            (int(c.ROI_BOTTOM_RIGHT * w), h),
            (int(c.ROI_TOP_RIGHT * w), int(c.ROI_TOP_Y * h)),
            (int(c.ROI_TOP_LEFT * w), int(c.ROI_TOP_Y * h)),
        ]], np.int32)

    def _roi(self, img):
        h, w = img.shape[:2]
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, self._roi_poly(w, h), 255)
        return cv2.bitwise_and(img, mask)

    # -- geometry -----------------------------------------------------------
    @staticmethod
    def _fit_x_at(segments, y_eval):
        """Fit x = m*y + b to a set of segments, return x at y_eval or None."""
        xs, ys = [], []
        for x1, y1, x2, y2 in segments:
            xs += [x1, x2]
            ys += [y1, y2]
        if len(xs) < 2:
            return None
        # fit x as a function of y (lanes are near-vertical)
        m, b = np.polyfit(ys, xs, 1)
        return m * y_eval + b

    def process(self, frame) -> LaneResult:
        c = self.cfg
        h, w = frame.shape[:2]
        binary = self._mask_tape(frame)

        # Coverage guard: how much of the ROI did the tape mask fill? A sane
        # lane is a few thin lines (small coverage). A flood means glare or bare
        # wood - we still try to fit, but cap the confidence so the car doesn't
        # trust a hallucinated lane.
        roi_mask = np.zeros(binary.shape, np.uint8)
        cv2.fillPoly(roi_mask, self._roi_poly(w, h), 255)
        roi_area = int(np.count_nonzero(roi_mask)) or 1
        coverage = np.count_nonzero(cv2.bitwise_and(binary, roi_mask)) / roi_area
        glare = coverage > c.MAX_TAPE_COVERAGE

        # On-track check: the mat is dark and carries bright tape, so the ROI's
        # median sits far below its bright peak. Bare wood/floor is uniformly
        # mid-toned, so median and peak are close. Using the RATIO (not an
        # absolute brightness) keeps this correct under auto-exposure and in
        # brighter or dimmer rooms.
        l_full = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[:, :, 0]
        roi_vals = l_full[roi_mask > 0]
        roi_median = float(np.median(roi_vals))
        roi_p95 = float(np.percentile(roi_vals, 95))
        dark_ratio = roi_median / max(1.0, roi_p95)
        off_track = dark_ratio > c.MAT_DARK_RATIO_MAX

        edges = cv2.Canny(binary, 50, 150)
        cropped = cv2.bitwise_and(edges, roi_mask)
        lines = cv2.HoughLinesP(cropped, 1, np.pi / 180, c.HOUGH_THRESHOLD,
                                minLineLength=c.HOUGH_MIN_LINE,
                                maxLineGap=c.HOUGH_MAX_GAP)

        left, right = [], []
        if lines is not None:
            # OpenCV 4 returns (N, 1, 4); OpenCV 5 returns (N, 4). Normalize so
            # the same code runs on the Pi (cv2 4.x) and a laptop (cv2 5.x).
            for ln in np.asarray(lines).reshape(-1, 4):
                x1, y1, x2, y2 = ln
                if x2 == x1:
                    slope = 999.0
                else:
                    slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < c.MIN_ABS_SLOPE:
                    continue  # near-horizontal -> not a lane edge
                # left-lane edges rise to the right (negative slope in image
                # coords); right-lane edges fall. Also split by x position.
                mid_x = (x1 + x2) / 2
                if slope < 0 and mid_x < w * 0.6:
                    left.append((x1, y1, x2, y2))
                elif slope > 0 and mid_x > w * 0.4:
                    right.append((x1, y1, x2, y2))

        # Line-support guard: a real lane edge is backed by a lot of line
        # length. Scattered short wood-grain segments fall below the floor and
        # are discarded, so texture can't masquerade as a lane.
        def support(segs):
            return sum(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
                       for x1, y1, x2, y2 in segs)
        if support(left) < c.MIN_SEG_SUPPORT:
            left = []
        if support(right) < c.MIN_SEG_SUPPORT:
            right = []

        y_eval = h - 1
        lx = self._fit_x_at(left, y_eval) if left else None
        rx = self._fit_x_at(right, y_eval) if right else None

        left_found = lx is not None
        right_found = rx is not None

        if left_found and right_found:
            center = (lx + rx) / 2.0
            conf = 1.0
        elif left_found:
            center = lx + w * 0.25          # assume lane ~half-width to the right
            conf = 0.5
        elif right_found:
            center = rx - w * 0.25
            conf = 0.5
        else:
            center = self._last_center if self._last_center is not None else w / 2
            conf = 0.0

        # Off the mat (bright wood/floor) -> reject outright, no matter what
        # lines were fitted. A flooded mask (glare) on the mat -> distrust.
        if off_track:
            conf = 0.0
            left_found = right_found = False
        elif glare:
            conf = 0.0 if (not left_found and not right_found) else conf * 0.4

        # smooth
        if self._last_center is not None and conf > 0:
            a = c.LANE_SMOOTHING
            center = a * self._last_center + (1 - a) * center
        if conf > 0:
            self._last_center = center

        offset = (center - w / 2) / (w / 2)   # [-1, 1], + means lane centre is right
        # cross-track error the controller uses: car should steer toward centre.
        # We define offset as (lane_center - image_center); + -> steer right.

        overlay = self._draw(frame, left, right, center)
        return LaneResult(offset=float(np.clip(offset, -1, 1)),
                          confidence=conf, lane_center_px=float(center),
                          left_found=left_found, right_found=right_found,
                          overlay=overlay)

    # -- debug drawing ------------------------------------------------------
    def _draw(self, frame, left, right, center):
        img = frame.copy()
        h, w = img.shape[:2]
        for grp, col in ((left, (0, 0, 255)), (right, (255, 0, 0))):
            for x1, y1, x2, y2 in grp:
                cv2.line(img, (x1, y1), (x2, y2), col, 3)
        cv2.line(img, (w // 2, h), (w // 2, h - 40), (200, 200, 200), 1)
        cv2.circle(img, (int(center), h - 8), 8, (0, 255, 0), -1)
        return img

    # -- optional bird's-eye warp (calibrate the src points on your car) ----
    def make_birdseye(self, w, h):
        """Perspective transform mapping the trapezoidal road to a rectangle.
        Calibrate src against a printed grid on the floor for real cm output."""
        c = self.cfg
        src = np.float32([
            [c.ROI_TOP_LEFT * w, c.ROI_TOP_Y * h],
            [c.ROI_TOP_RIGHT * w, c.ROI_TOP_Y * h],
            [c.ROI_BOTTOM_RIGHT * w, h],
            [c.ROI_BOTTOM_LEFT * w, h]])
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, 0 + h]])
        self._warp_M = cv2.getPerspectiveTransform(src, dst)
        self._warp_size = (w, h)
        return self._warp_M

    def warp(self, img):
        if self._warp_M is None:
            self.make_birdseye(img.shape[1], img.shape[0])
        return cv2.warpPerspective(img, self._warp_M, self._warp_size)


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if not path:
        print("usage: python -m perception.lane <image_or_video>")
        sys.exit(1)
    det = LaneDetector()
    img = cv2.imread(path)
    if img is not None:
        img = cv2.resize(img, (config.CAM_WIDTH, config.CAM_HEIGHT))
        r = det.process(img)
        print(f"offset={r.offset:+.3f} conf={r.confidence:.2f} "
              f"L={r.left_found} R={r.right_found}")
        cv2.imshow("lane", r.overlay); cv2.waitKey(0); cv2.destroyAllWindows()
    else:
        cap = cv2.VideoCapture(path)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.resize(frame, (config.CAM_WIDTH, config.CAM_HEIGHT))
            r = det.process(frame)
            cv2.putText(r.overlay, f"off={r.offset:+.2f} conf={r.confidence:.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("lane", r.overlay)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release(); cv2.destroyAllWindows()
