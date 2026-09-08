"""
ultrasonic.py - HC-SR04 distance sensors on the Pi GPIO.

An independent obstacle reflex that needs neither vision nor the Arduino: a
background thread pings every configured sensor and keeps the latest distances,
so the control loop can read them instantly and stop if something is too close.

Supports several sensors at once (you have many) - front / left / right etc.

Backend: gpiozero's DistanceSensor (works on the Pi 5 via the lgpio pin
factory). If gpiozero/GPIO isn't available (e.g. on a laptop), it runs in a
disabled no-op mode so the rest of the stack still imports and runs.

WIRING WARNING: HC-SR04 ECHO is 5 V; the Pi GPIO is 3.3 V. Use a voltage
divider on every ECHO line or you can damage the Pi. See docs/HARDWARE_SETUP.md.
"""

from __future__ import annotations

import threading
import time

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


class Ultrasonic:
    def __init__(self, cfg=config):
        self.cfg = cfg
        self._sensors = {}          # name -> DistanceSensor
        self._dist = {}             # name -> metres (or None)
        self._lock = threading.Lock()
        self.enabled = False
        self._stop = threading.Event()

        if not cfg.ULTRASONIC_ENABLED:
            print("[ultrasonic] disabled in config")
            return
        if not self._init_hw():
            return
        self.enabled = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _init_hw(self) -> bool:
        try:
            from gpiozero import DistanceSensor
        except Exception as e:
            print("[ultrasonic] gpiozero not available -> disabled:", e)
            return False
        try:
            for s in self.cfg.ULTRASONIC_SENSORS:
                self._sensors[s["name"]] = DistanceSensor(
                    echo=s["echo"], trigger=s["trig"],
                    max_distance=self.cfg.ULTRASONIC_MAX_M)
                self._dist[s["name"]] = None
            print(f"[ultrasonic] {len(self._sensors)} sensor(s): "
                  f"{list(self._sensors)}")
            return True
        except Exception as e:
            print("[ultrasonic] could not init sensors -> disabled:", e)
            return False

    def _loop(self):
        period = 1.0 / self.cfg.ULTRASONIC_POLL_HZ
        while not self._stop.is_set():
            for name, sensor in self._sensors.items():
                try:
                    d = sensor.distance  # metres, 0..max
                except Exception:
                    d = None
                with self._lock:
                    self._dist[name] = d
            time.sleep(period)

    # -- public API ---------------------------------------------------------
    def distance(self, name="front"):
        """Latest distance in metres for one sensor, or None."""
        with self._lock:
            return self._dist.get(name)

    def all(self):
        with self._lock:
            return dict(self._dist)

    def min_distance(self):
        """Closest reading across all sensors, or None if nothing valid."""
        with self._lock:
            vals = [d for d in self._dist.values() if d is not None]
        return min(vals) if vals else None

    def front_blocked(self):
        """True if the front sensor sees an obstacle within the stop distance."""
        d = self.distance("front")
        return d is not None and d <= self.cfg.ULTRASONIC_STOP_M

    def front_clear(self):
        """True if the front is beyond the clear distance (hysteresis)."""
        d = self.distance("front")
        return d is None or d >= self.cfg.ULTRASONIC_CLEAR_M

    def close(self):
        self._stop.set()
        for s in self._sensors.values():
            try:
                s.close()
            except Exception:
                pass


if __name__ == "__main__":
    us = Ultrasonic()
    if not us.enabled:
        print("No ultrasonic hardware. On the Pi, wire an HC-SR04 and set the "
              "pins in config.ULTRASONIC_SENSORS.")
    else:
        print("Reading sensors. Ctrl-C to stop.")
        try:
            while True:
                readings = {k: (f"{v*100:.0f}cm" if v is not None else "--")
                            for k, v in us.all().items()}
                blocked = "BLOCKED" if us.front_blocked() else "clear"
                print(f"\r{readings}  front:{blocked}   ", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            us.close()
            print()
