"""
controller.py - PD steering controller with output shaping.

Takes the normalized lane offset ([-1, 1]) and produces a steer command
([-1, 1]) plus a throttle that eases off in hard turns. Includes:
  * PD control (P + D on the error) instead of the original unit-less P-only.
  * Slew limiting on both steer and throttle so commands can't jump.
  * A derivative computed on wall-clock dt, so it behaves the same at any rate.
"""

from __future__ import annotations

import time

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


class SteeringController:
    def __init__(self, cfg=config, clock=time.time):
        self.cfg = cfg
        # Injectable clock so the simulator can advance virtual time. On the
        # real car this is time.time(); in the sim it is the simulated clock,
        # which keeps derivative and timing behaviour identical in both.
        self._clock = clock
        self._prev_err = 0.0
        self._prev_t = None
        self._steer = 0.0
        self._throttle = 0.0

    def reset(self):
        self._prev_err = 0.0
        self._prev_t = None
        self._steer = 0.0
        self._throttle = 0.0

    @staticmethod
    def _slew(current, target, max_step):
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def update(self, offset: float, confidence: float = 1.0):
        """offset: lane_center - image_center, normalized. + means centre is to
        the right, so the car should steer right (+steer)."""
        c = self.cfg
        now = self._clock()
        dt = 0.05 if self._prev_t is None else max(1e-3, now - self._prev_t)
        self._prev_t = now

        error = offset  # want lane centre at image centre -> drive error to 0
        deriv = (error - self._prev_err) / dt
        self._prev_err = error

        raw_steer = c.KP * error + c.KD * deriv
        raw_steer = max(-1.0, min(1.0, raw_steer))
        # low confidence -> damp the command toward zero (don't chase noise)
        raw_steer *= max(0.2, confidence)

        self._steer = self._slew(self._steer, raw_steer, c.STEER_SLEW)

        # throttle eases down as |steer| rises
        target_thr = c.BASE_THROTTLE * (1.0 - 0.6 * abs(self._steer))
        target_thr = max(c.MIN_THROTTLE, target_thr)
        self._throttle = self._slew(self._throttle, target_thr, c.THROTTLE_SLEW)

        return self._throttle, self._steer
