"""
teleop.py - keyboard drive on top of motor_link.

Your first real drive and a permanent test tool. Uses the MotorLink heartbeat,
so the firmware watchdog stays fed while you hold a key.

Controls (hold keys):
    W / S : forward / reverse throttle
    A / D : steer left / right
    SPACE : immediate stop (throttle & steer 0)
    X     : DISARM (emergency)
    R     : re-ARM after a fault (sends CLEAR then ARM)
    Q / Esc : quit (auto-disarms)

Two input backends:
  * a small OpenCV window (works over VNC / on the Pi desktop) - default
  * raw terminal keys with --term (no GUI, SSH-friendly, Linux only)
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from .. import config
except (ImportError, ValueError):
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config

from pi.motor_link import MotorLink, decode_faults

STEP_THR = 0.08
STEP_STEER = 0.12
DECAY = 0.85  # release keys -> commands ease back to 0


def run_gui(link):
    import cv2
    import numpy as np
    thr = steer = 0.0
    panel = np.zeros((200, 480, 3), np.uint8)
    cv2.namedWindow("teleop")
    print("GUI teleop. Focus the window. WASD to drive, SPACE stop, Q quit.")
    while True:
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("w"):
            thr = min(1.0, thr + STEP_THR)
        elif k == ord("s"):
            thr = max(-1.0, thr - STEP_THR)
        elif k == ord("a"):
            steer = max(-1.0, steer - STEP_STEER)
        elif k == ord("d"):
            steer = min(1.0, steer + STEP_STEER)
        elif k == ord(" "):
            thr = steer = 0.0
        elif k == ord("x"):
            link.disarm()
        elif k == ord("r"):
            link.clear(); time.sleep(0.1); link.arm()
        else:
            thr *= DECAY
            steer *= DECAY
            if abs(thr) < 0.02:
                thr = 0.0
            if abs(steer) < 0.02:
                steer = 0.0
        link.set(thr, steer)

        panel[:] = 0
        t = link.tel
        cv2.putText(panel, f"state:{t.state} thr:{thr:+.2f} steer:{steer:+.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(panel, f"L:{t.cur_l} R:{t.cur_r} estop:{t.estop}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        faults = ",".join(decode_faults(t.fault_mask)) or "none"
        cv2.putText(panel, f"faults:{faults}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("teleop", panel)
    cv2.destroyAllWindows()


def run_term(link):
    """Raw-terminal teleop for SSH (Linux only). One keypress per command."""
    import termios, tty, select
    thr = steer = 0.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("Terminal teleop. WASD drive, space stop, x disarm, r rearm, q quit.")
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                ch = sys.stdin.read(1).lower()
                if ch in ("q", "\x1b"):
                    break
                elif ch == "w":
                    thr = min(1.0, thr + STEP_THR)
                elif ch == "s":
                    thr = max(-1.0, thr - STEP_THR)
                elif ch == "a":
                    steer = max(-1.0, steer - STEP_STEER)
                elif ch == "d":
                    steer = min(1.0, steer + STEP_STEER)
                elif ch == " ":
                    thr = steer = 0.0
                elif ch == "x":
                    link.disarm()
                elif ch == "r":
                    link.clear(); time.sleep(0.1); link.arm()
            link.set(thr, steer)
            t = link.tel
            sys.stdout.write(
                f"\rstate:{t.state:9s} thr:{thr:+.2f} steer:{steer:+.2f} "
                f"L:{t.cur_l:4d} R:{t.cur_r:4d} faults:"
                f"{','.join(decode_faults(t.fault_mask)) or 'none':12s}   ")
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--term", action="store_true", help="terminal (SSH) mode")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    link = MotorLink(port=args.port, sim=args.sim, verbose=False)
    link.clear(); time.sleep(0.2)
    link.arm(); time.sleep(0.3)
    try:
        if args.term:
            run_term(link)
        else:
            run_gui(link)
    finally:
        link.close()
        print("Disarmed. Bye.")


if __name__ == "__main__":
    main()
