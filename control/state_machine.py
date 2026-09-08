"""
state_machine.py - the driving brain.

Turns perception (lane offset + detections) into a driving state and the
throttle/steer command for that state. Keeping this separate from the I/O
(camera, motors) makes it testable and easy to reason about.

States
------
  LANE_FOLLOW    normal driving: PD steer on the lane, cruise throttle
  STOP_SIGN      a stop sign is close: hold still for STOP_SIGN_HOLD_S, then
                 resume and ignore that sign for STOP_SIGN_COOLDOWN_S
  OBSTACLE_STOP  something is close ahead: hold until it clears OBSTACLE_CLEAR_S
  UTURN          open-loop timed 180-deg manoeuvre, then back to LANE_FOLLOW
  LANE_LOST      no confident lane: coast to a stop; if it stays lost long
                 enough, optionally trigger a U-turn (dead end)

Transitions are priority-ordered every tick: E-stop-like conditions (obstacle)
beat stop signs beat normal lane following.
"""

from __future__ import annotations

import time
from enum import Enum

try:
    from .. import config
    from .controller import SteeringController
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config
    from control.controller import SteeringController


class State(Enum):
    LANE_FOLLOW = "LANE_FOLLOW"
    STOP_SIGN = "STOP_SIGN"
    OBSTACLE_STOP = "OBSTACLE_STOP"
    UTURN = "UTURN"
    LANE_LOST = "LANE_LOST"


class DrivingBrain:
    def __init__(self, cfg=config, clock=time.time):
        self.cfg = cfg
        # Injectable clock: the real car passes time.time(); the simulator
        # passes its virtual clock so timed manoeuvres (stop-sign hold, U-turn
        # duration, obstacle clear) advance with SIMULATED time. Without this
        # the sim finishes timed states in the wrong number of steps and the
        # timings cannot be tuned off-hardware.
        self._clock = clock
        self.state = State.LANE_FOLLOW
        self.controller = SteeringController(cfg, clock=clock)
        self._state_since = clock()
        self._sign_cooldown_until = 0.0
        self._obstacle_gone_since = None
        self._lane_lost_since = None
        self._uturn_requested = False
        self._uturn_cooldown_until = 0.0

    # -- external triggers --------------------------------------------------
    def request_uturn(self):
        self._uturn_requested = True

    def _enter(self, state: State):
        if state != self.state:
            self.state = state
            self._state_since = self._clock()
            if state == State.LANE_FOLLOW:
                self.controller.reset()

    def _in_state_for(self):
        return self._clock() - self._state_since

    # -- main tick ----------------------------------------------------------
    def update(self, lane, detections):
        """lane: LaneResult, detections: DetectResult.
        Returns (throttle, steer, info_string)."""
        c = self.cfg
        now = self._clock()

        # --- gather perception facts ---
        obstacle_close = any(
            b.area_frac >= c.OBSTACLE_MIN_AREA
            and abs(b.cx_frac - 0.5) < c.OBSTACLE_CENTER_BAND / 2
            for b in detections.obstacles()
        )
        sign_close = any(b.area_frac >= c.STOP_SIGN_MIN_AREA
                         for b in detections.stop_signs())
        lane_ok = lane.confidence > 0.0

        # track lane-lost timer
        if lane_ok:
            self._lane_lost_since = None
        elif self._lane_lost_since is None:
            self._lane_lost_since = now

        # ================= priority-ordered transitions =================
        # 1) explicit U-turn request or dead-end auto U-turn
        if self.state != State.UTURN:
            # The dead-end trigger is suppressed during the cooldown that
            # follows a U-turn, so one turn can't chain into another while the
            # car is still re-acquiring the lane. An explicit request always
            # wins, cooldown or not.
            lost_long = (self._lane_lost_since is not None and
                         now - self._lane_lost_since > c.UTURN_TRIGGER_LANE_LOST_S
                         and now >= self._uturn_cooldown_until)
            if self._uturn_requested or lost_long:
                self._uturn_requested = False
                self._enter(State.UTURN)

        # 2) obstacle beats everything except an in-progress U-turn
        if self.state != State.UTURN and obstacle_close:
            self._obstacle_gone_since = None
            self._enter(State.OBSTACLE_STOP)

        # --- handle each state ---
        if self.state == State.UTURN:
            return self._do_uturn()

        if self.state == State.OBSTACLE_STOP:
            if not obstacle_close:
                if self._obstacle_gone_since is None:
                    self._obstacle_gone_since = now
                if now - self._obstacle_gone_since >= c.OBSTACLE_CLEAR_S:
                    self._enter(State.LANE_FOLLOW)
                    return self._follow(lane, "obstacle cleared")
            else:
                self._obstacle_gone_since = None
            return 0.0, 0.0, "OBSTACLE_STOP"

        if self.state == State.STOP_SIGN:
            if self._in_state_for() >= c.STOP_SIGN_HOLD_S:
                self._sign_cooldown_until = now + c.STOP_SIGN_COOLDOWN_S
                self._enter(State.LANE_FOLLOW)
                return self._follow(lane, "resumed after stop")
            return 0.0, 0.0, f"STOP_SIGN {self._in_state_for():.1f}s"

        # normal-ish states: check stop sign
        if sign_close and now >= self._sign_cooldown_until:
            self._enter(State.STOP_SIGN)
            return 0.0, 0.0, "STOP_SIGN enter"

        if not lane_ok:
            self._enter(State.LANE_LOST)
            # coast to a stop
            return 0.0, 0.0, "LANE_LOST"

        # default: follow the lane
        self._enter(State.LANE_FOLLOW)
        return self._follow(lane, "lane follow")

    # -- state bodies -------------------------------------------------------
    def _follow(self, lane, info):
        thr, steer = self.controller.update(lane.offset, lane.confidence)
        return thr, steer, info

    def _do_uturn(self):
        c = self.cfg
        t = self._in_state_for()
        if t >= c.UTURN_DURATION_S:
            # Start the cooldown and clear the lane-lost timer so the car gets
            # a fair chance to re-acquire the lane before anything re-triggers.
            self._uturn_cooldown_until = self._clock() + c.UTURN_COOLDOWN_S
            self._lane_lost_since = None
            self._enter(State.LANE_FOLLOW)
            return 0.0, 0.0, "UTURN done"
        return c.UTURN_THROTTLE, c.UTURN_STEER, f"UTURN {t:.1f}/{c.UTURN_DURATION_S:.1f}s"
