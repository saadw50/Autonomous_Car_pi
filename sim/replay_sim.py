"""
replay_sim.py - simulate driving using YOUR REAL camera footage.

The synthetic simulator (track_sim.py) is useful but it is not your track.
This one drives on the actual frames captured from the car's camera
(Negative1..Negative209), so perception, the state machine and the controller
are all exercised against real glare, real tape and real lighting.

Two modes
---------
replay    Play the recorded frames in order and run the full stack on them.
          Honest but OPEN loop: steering cannot change what the camera sees,
          because the frames were recorded on a fixed run. Good for validating
          perception and decisions on real data.

reactive  CLOSED loop on real imagery (default). The car's simulated lateral
          offset and heading error are applied to each real frame with a
          ground-plane warp (camera view -> bird's eye -> shift/rotate ->
          back), so if the controller steers badly the view really does drift
          off the lane and the error grows. This is what lets you tune the PD
          controller on real footage with no hardware.

          The warp is geometrically plausible, not calibrated: PX_PER_CM is an
          estimate. Treat the numbers as relative, not absolute.

Usage
-----
    python -m sim.replay_sim                        # reactive, frames 1-209
    python -m sim.replay_sim --mode replay          # pure playback
    python -m sim.replay_sim --show                 # watch it
    python -m sim.replay_sim --save replay.avi      # record it
    python -m sim.replay_sim --start 1 --end 209
    python -m sim.replay_sim --brightness 1.4       # stress the lighting
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from perception.lane import LaneDetector
from perception.detect import DetectResult
from control.state_machine import DrivingBrain
from pi.motor_link import MotorLink

DEFAULT_DIR = r"D:\Autonomus Car\Negative"

# Rough ground-plane scale for the bird's-eye view. Not calibrated - it sets
# how strongly a lateral error translates into image movement.
PX_PER_CM = 2.2
MAX_SPEED_CMS = 60.0     # cm/s at full motor command
TRACK_GAUGE = 18.0       # cm between wheels
LANE_HALF_CM = 45.0      # how far off-centre counts as "out of lane"


def load_frames(folder, start, end):
    frames = []
    for i in range(start, end + 1):
        p = os.path.join(folder, f"Negative{i}.jpg")
        if os.path.exists(p):
            img = cv2.imread(p)
            if img is not None:
                frames.append((i, cv2.resize(img, (config.CAM_WIDTH,
                                                   config.CAM_HEIGHT))))
    return frames


class GroundWarp:
    """Camera view <-> bird's-eye, so we can move the virtual camera on the
    ground plane and re-render what it would see."""

    def __init__(self, w, h):
        c = config
        self.w, self.h = w, h
        src = np.float32([
            [c.ROI_TOP_LEFT * w, c.ROI_TOP_Y * h],
            [c.ROI_TOP_RIGHT * w, c.ROI_TOP_Y * h],
            [c.ROI_BOTTOM_RIGHT * w, h],
            [c.ROI_BOTTOM_LEFT * w, h]])
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = np.linalg.inv(self.M)

    def apply(self, frame, lat_cm: float, psi_rad: float):
        """Return the frame as it would look with the camera shifted lat_cm to
        the right and rotated by psi_rad."""
        if abs(lat_cm) < 1e-3 and abs(psi_rad) < 1e-4:
            return frame
        bev = cv2.warpPerspective(frame, self.M, (self.w, self.h),
                                  borderMode=cv2.BORDER_REPLICATE)
        # rotate about the car (bottom centre of the bird's-eye view)
        cx, cy = self.w / 2.0, float(self.h)
        A = cv2.getRotationMatrix2D((cx, cy), math.degrees(psi_rad), 1.0)
        # shifting the car right moves the world left in the image
        A[0, 2] -= lat_cm * PX_PER_CM
        bev = cv2.warpAffine(bev, A, (self.w, self.h),
                             borderMode=cv2.BORDER_REPLICATE)
        return cv2.warpPerspective(bev, self.Minv, (self.w, self.h),
                                   borderMode=cv2.BORDER_REPLICATE)


def parse_args():
    p = argparse.ArgumentParser(description="Drive on real recorded frames")
    p.add_argument("--dir", default=DEFAULT_DIR)
    p.add_argument("--start", type=int, default=1)
    p.add_argument("--end", type=int, default=209)
    p.add_argument("--mode", choices=["reactive", "replay"], default="reactive")
    p.add_argument("--fps", type=float, default=config.HEARTBEAT_HZ)
    p.add_argument("--show", action="store_true")
    p.add_argument("--save", default=None)
    p.add_argument("--brightness", type=float, default=1.0)
    p.add_argument("--loops", type=int, default=1, help="repeat the sequence")
    return p.parse_args()


def main():
    args = parse_args()
    frames = load_frames(args.dir, args.start, args.end)
    if not frames:
        print(f"No frames found in {args.dir} for {args.start}..{args.end}")
        return 1
    print(f"[replay_sim] {len(frames)} real frames, mode={args.mode}")

    det = LaneDetector()
    sim_clock = {"t": 0.0}
    brain = DrivingBrain(clock=lambda: sim_clock["t"])
    link = MotorLink(sim=True)
    warp = GroundWarp(config.CAM_WIDTH, config.CAM_HEIGHT)
    empty = DetectResult()

    link.clear(); link.arm()

    dt = 1.0 / args.fps
    lat = 0.0          # lateral offset from the recorded path, cm (+ = right)
    psi = 0.0          # heading error, radians
    pos = 0.0          # distance along the recorded path, in frames
    t = 0.0
    out_of_lane = 0
    steps = 0
    states = {}
    lat_hist = []
    writer = None

    total = len(frames) * args.loops
    while pos < total - 1:
        sim_clock["t"] = t
        idx, base = frames[int(pos) % len(frames)]
        if args.brightness != 1.0:
            base = np.clip(base.astype(np.float32) * args.brightness,
                           0, 255).astype(np.uint8)

        view = warp.apply(base, lat, psi) if args.mode == "reactive" else base

        lane = det.process(view)
        throttle, steer, info = brain.update(lane, empty)
        link.set(throttle, steer)
        left, right = link.target

        # vehicle model
        vl = (left / config.MAX_CMD) * MAX_SPEED_CMS
        vr = (right / config.MAX_CMD) * MAX_SPEED_CMS
        v = 0.5 * (vl + vr)
        omega = (vl - vr) / TRACK_GAUGE
        if args.mode == "reactive":
            psi += omega * dt
            psi = max(-0.6, min(0.6, psi))
            lat += v * math.sin(psi) * dt
        # advance along the recorded path with forward speed
        pos += max(0.0, v * math.cos(psi)) * dt / 6.0   # ~6 cm between frames

        steps += 1
        t += dt
        states[brain.state.value] = states.get(brain.state.value, 0) + 1
        lat_hist.append(lat)
        if abs(lat) > LANE_HALF_CM:
            out_of_lane += 1

        if args.show or args.save:
            disp = (lane.overlay if lane.overlay is not None else view).copy()
            cv2.putText(disp, f"frame {idx}  {brain.state.value}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(disp, f"thr={throttle:.2f} steer={steer:+.2f} "
                              f"conf={lane.confidence:.1f}", (8, 46),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            col = (0, 0, 255) if abs(lat) > LANE_HALF_CM else (255, 255, 0)
            cv2.putText(disp, f"lat={lat:+.0f}cm psi={math.degrees(psi):+.0f}deg",
                        (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            if args.save:
                if writer is None:
                    writer = cv2.VideoWriter(
                        args.save, cv2.VideoWriter_fourcc(*"MJPG"),
                        args.fps, (disp.shape[1], disp.shape[0]))
                writer.write(disp)
            if args.show:
                cv2.imshow("replay_sim", disp)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    link.close()
    if writer:
        writer.release(); print("saved", args.save)
    if args.show:
        cv2.destroyAllWindows()

    lat_arr = np.array(lat_hist) if lat_hist else np.zeros(1)
    print("\n=== replay simulation summary ===")
    print(f"  mode          : {args.mode}")
    print(f"  steps         : {steps}   sim time {t:.1f}s")
    print(f"  frames used   : {int(pos)} of {total}")
    if args.mode == "reactive":
        print(f"  |lat| mean    : {np.abs(lat_arr).mean():.1f} cm")
        print(f"  |lat| max     : {np.abs(lat_arr).max():.1f} cm")
        print(f"  in lane       : {100*(steps-out_of_lane)/max(1,steps):.1f} % "
              f"of steps (|lat| <= {LANE_HALF_CM:.0f} cm)")
    print(f"  states        : {states}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
