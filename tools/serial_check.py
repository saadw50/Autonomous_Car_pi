"""
serial_check.py - confirm the Pi can see and talk to the Arduino.

Run this FIRST when you plug the Arduino in (Phase 1, week 3):
    python tools/serial_check.py            # auto-detect port
    python tools/serial_check.py /dev/ttyACM0

It lists serial ports, opens the link, sends PING and STATUS, and prints the
first telemetry frames. If you see ACK/TEL with matching CRC, the whole link
is proven end-to-end.
"""

from __future__ import annotations

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pi.motor_link import MotorLink, decode_faults


def list_ports():
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found. Is the Arduino plugged into the Pi's USB?")
        for p in ports:
            print(f"  {p.device:15s} {p.description}")
        return [p.device for p in ports]
    except Exception as e:
        print("Could not enumerate ports:", e)
        return []


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    print("Serial ports:")
    ports = list_ports()
    if port is None:
        for p in ports:
            if "ACM" in p or "USB" in p:
                port = p
                break
    if port is None:
        print("\nNo Arduino-like port. Plug it in and retry (or pass a port).")
        return
    print(f"\nOpening {port} ...")
    link = MotorLink(port=port, verbose=True)
    try:
        link.ping()
        time.sleep(0.3)
        link.clear()
        time.sleep(0.3)
        print("\nWaiting for telemetry (3s)...")
        for _ in range(15):
            t = link.tel
            print(f"  state={t.state:9s} estop={t.estop} "
                  f"faults={decode_faults(t.fault_mask) or 'none'} "
                  f"bad_frames={t.bad_frames}")
            time.sleep(0.2)
    finally:
        link.close()
    print("\nIf you saw state=DISARMED and no bad_frames, the link is good.")


if __name__ == "__main__":
    main()
