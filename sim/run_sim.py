"""
run_sim.py - drive the car in simulation, closed loop, with zero hardware.

The loop is the REAL stack end to end:

    render camera view  ->  perception/lane.py  ->  control/state_machine.py
         ^                                                    |
         |                                                    v
    sim vehicle model   <-  motor commands  <-  pi/motor_link.py (SimSerial)

so the same perception, the same PD controller, the same state machine and the
same serial command path that will run on the car are exercised here. Only the
camera and the motors are synthetic.

Usage
-----
    python -m sim.run_sim                      # headless, prints a summary
    python -m sim.run_sim --show               # watch it drive (needs a GUI)
    python -m sim.run_sim --stop-sign          # put a stop sign on the track
    python -m sim.run_sim --obstacle           # put an obstacle on the track
    python -m sim.run_sim --uturn-at 6         # request a U-turn at t=6s
    python -m sim.run_sim --brightness 1.6 --glare 0.5 --noise 0.2
    python -m sim.run_sim --seconds 40 --save run.avi
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
from sim.track_sim import (CarSim, TrackWorld, MAT_GRAY, FLOOR_GRAY,
                           render_camera, render_map, synth_detections)


def on_mat(world: TrackWorld, pose) -> bool:
    """True if the car's centre is still on the dark track mat."""
    x, y = int(pose.x), int(pose.y)
    if not (0 <= x < world.map.shape[1] and 0 <= y < world.map.shape[0]):
        return False
    # mat and tape are both "on track"; the bright floor is not
    return int(world.map[y, x]) < (MAT_GRAY + FLOOR_GRAY) // 2


def parse_args():
    p = argparse.ArgumentParser(description="Closed-loop driving simulator")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--fps", type=float, default=config.HEARTBEAT_HZ)
    p.add_argument("--show", action="store_true")
    p.add_argument("--save", default=None, help="write an .avi of the run")
    p.add_argument("--stop-sign", action="store_true")
    p.add_argument("--obstacle", action="store_true")
    p.add_argument("--uturn-at", type=float, default=None,
                   help="request a U-turn at this time (seconds)")
    p.add_argument("--brightness", type=float, default=1.0)
    p.add_argument("--glare", type=float, default=0.0)
    p.add_argument("--noise", type=float, default=0.0)
    p.add_argument("--yolo", action="store_true",
                   help="use the real YOLO detector instead of ground truth")
    return p.parse_args()


def main():
    args = parse_args()
    world = TrackWorld()
    pose = world.start_pose()
    if args.stop_sign:
        world.add_stop_sign(ahead_cm=260, pose=pose)
    if args.obstacle:
        world.add_obstacle(ahead_cm=300, pose=pose)

    car = CarSim(pose)
    lane_det = LaneDetector()
    # Virtual clock: the brain and controller must advance with SIMULATED time,
    # not wall clock, or timed manoeuvres (stop-sign hold, U-turn duration)
    # take the wrong number of steps and can't be tuned off-hardware.
    sim_clock = {"t": 0.0}
    brain = DrivingBrain(clock=lambda: sim_clock["t"])
    link = MotorLink(sim=True)
    detector = None
    if args.yolo:
        from perception.detect import Detector
        detector = Detector()

    link.clear()
    link.arm()

    dt = 1.0 / args.fps
    steps = int(args.seconds * args.fps)
    writer = None
    off_track = 0
    states = {}
    t = 0.0

    for i in range(steps):
        sim_clock["t"] = t
        frame = render_camera(world, car.pose,
                              brightness=args.brightness, glare=args.glare,
                              noise=args.noise, seed=i if args.noise else None)
        lane = lane_det.process(frame)

        if detector is not None:
            detector.submit(frame)
            det = detector.result
        else:
            det = synth_detections(world, car.pose)

        if args.uturn_at is not None and t >= args.uturn_at:
            brain.request_uturn()
            args.uturn_at = None

        throttle, steer, info = brain.update(lane, det)
        link.set(throttle, steer)

        # drive the vehicle from the actual command on the wire
        left, right = link.target
        car.step(left, right, dt)

        states[brain.state.value] = states.get(brain.state.value, 0) + 1
        if not on_mat(world, car.pose):
            off_track += 1
        t += dt

        if args.show or args.save:
            view = lane.overlay if lane.overlay is not None else frame
            view = view.copy()
            cv2.putText(view, f"{brain.state.value} | {info}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(view, f"thr={throttle:+.2f} steer={steer:+.2f} "
                              f"L={left} R={right} conf={lane.confidence:.1f}",
                        (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(view, f"t={t:5.1f}s  {'ON' if on_mat(world, car.pose) else 'OFF'} track",
                        (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            mp = render_map(world, car.pose, car, size=view.shape[0])
            combo = np.hstack([view, mp])
            if args.save:
                if writer is None:
                    writer = cv2.VideoWriter(
                        args.save, cv2.VideoWriter_fourcc(*"MJPG"),
                        args.fps, (combo.shape[1], combo.shape[0]))
                writer.write(combo)
            if args.show:
                cv2.imshow("sim", combo)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("u"):
                    brain.request_uturn()

    link.close()
    if writer is not None:
        writer.release()
        print(f"saved {args.save}")
    if args.show:
        cv2.destroyAllWindows()

    dist = 0.0
    for a, b in zip(car.trail, car.trail[1:]):
        dist += math.dist(a, b)
    print("\n=== simulation summary ===")
    print(f"  duration      : {t:.1f} s ({steps} steps @ {args.fps:g} Hz)")
    print(f"  distance      : {dist/100:.1f} m")
    print(f"  on track      : {100*(steps-off_track)/max(1,steps):.1f} % of steps")
    print(f"  states        : {states}")
    return 0 if off_track < steps * 0.2 else 1


if __name__ == "__main__":
    sys.exit(main())
