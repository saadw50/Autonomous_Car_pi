"""
motor_link.py - the Pi-side driver for the Arduino motor controller.

This is the Phase-1 "missing piece": it speaks the exact serial protocol the
firmware defines (arduino_motor_controller.ino), so the perception/control code
can steer the car in clean normalized units and never worry about framing.

Protocol (from the firmware header comment):
    Frame:  PAYLOAD*CCCC\\n
    CCCC :  uppercase CRC-16/CCITT-FALSE over the ASCII PAYLOAD (4 hex chars)
    Commands:  CLEAR,seq | ARM,seq | DRIVE,seq,left,right,ttl_ms | DISARM,seq |
               PING,seq | STATUS,seq
    Replies :  ACK,seq,state,fault_mask | NACK,seq,reason |
               TEL,millis,state,fault_mask,cur_l,cur_r,tgt_l,tgt_r,estop,
                   drive_age_ms,last_seq,bad_frames | HELLO,1,state

Key design points:
  * A background heartbeat thread sends DRIVE at HEARTBEAT_HZ whether or not new
    steering has arrived. That constant feed is what keeps the firmware watchdog
    happy; without it the car stutters to a halt on every slow frame.
  * A reader thread parses every reply and keeps the latest telemetry.
  * set(throttle, steer) takes normalized [-1, 1] values; the differential mix
    to left/right happens here.
  * The link auto-DISARMs on close(), on exception, and on SIGINT, so a crash
    stops the car instead of leaving it running.

Runs without pyserial or a real Arduino in --sim mode (SimSerial), so the whole
stack can be developed and tested on a laptop.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field

try:
    import serial  # pyserial
except ImportError:  # allows import on machines without pyserial
    serial = None

# Import config whether run as a module or a script.
try:
    from .. import config
except (ImportError, ValueError):
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE  (init 0xFFFF, poly 0x1021, no reflection, no final xor).
# This MUST byte-for-byte match crc16CcittFalse() in the firmware.
# Check value for the ASCII string "123456789" is 0x29B1.
# ---------------------------------------------------------------------------
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def frame(payload: str) -> bytes:
    """Wrap a payload string into a full protocol frame with CRC and newline."""
    crc = crc16_ccitt_false(payload.encode("ascii"))
    return f"{payload}*{crc:04X}\n".encode("ascii")


# ---------------------------------------------------------------------------
# Telemetry snapshot
# ---------------------------------------------------------------------------
@dataclass
class Telemetry:
    t_wall: float = 0.0            # local time we received it
    millis: int = 0
    state: str = "UNKNOWN"
    fault_mask: int = 0
    cur_l: int = 0
    cur_r: int = 0
    tgt_l: int = 0
    tgt_r: int = 0
    estop: int = 0
    drive_age_ms: int = 0
    last_seq: int = 0
    bad_frames: int = 0
    # Appended by the ultrasonic-capable firmware; stay at defaults on older
    # firmware that sends only the first eleven fields.
    dist_cm: int = 0          # HC-SR04 distance, 0 = no reading / out of range
    obstacle: int = 0         # 1 while the Arduino reflex is holding the motors

    @property
    def armed(self) -> bool:
        return self.state == "ARMED"

    @property
    def faulted(self) -> bool:
        return self.state in ("FAULT", "ESTOP") or self.fault_mask != 0


FAULT_NAMES = {
    1 << 0: "ESTOP", 1 << 1: "WATCHDOG", 1 << 2: "RX_OVERFLOW",
    1 << 3: "BAD_CRC", 1 << 4: "BAD_COMMAND", 1 << 5: "BAD_SEQUENCE",
}


def decode_faults(mask: int) -> list[str]:
    return [name for bit, name in FAULT_NAMES.items() if mask & bit]


# ---------------------------------------------------------------------------
# Simulated serial port for hardware-free development
# ---------------------------------------------------------------------------
class SimSerial:
    """Pretends to be an armed Arduino. Echoes plausible ACK/TEL so the whole
    control stack can run on a laptop with no hardware attached."""

    def __init__(self):
        self._rx = bytearray()
        self._state = "DISARMED"
        self._last_seq = 0
        self._tgt = (0, 0)
        self._t0 = time.time()
        self.is_open = True

    # write side: parse commands and queue replies
    def write(self, data: bytes):
        text = data.decode("ascii", "replace").strip()
        payload = text.split("*")[0]
        parts = payload.split(",")
        cmd = parts[0]
        seq = parts[1] if len(parts) > 1 else "0"
        if cmd == "ARM":
            self._state = "ARMED"
        elif cmd == "DISARM":
            self._state = "DISARMED"
        elif cmd == "CLEAR":
            self._state = "DISARMED"
        elif cmd == "DRIVE" and len(parts) >= 4:
            try:
                self._tgt = (int(parts[2]), int(parts[3]))
            except ValueError:
                pass
        self._queue(f"ACK,{seq},{self._state},0")

    def _queue(self, payload: str):
        self._rx += frame(payload)

    def _tel(self):
        ms = int((time.time() - self._t0) * 1000)
        self._queue(
            f"TEL,{ms},{self._state},0,{self._tgt[0]},{self._tgt[1]},"
            f"{self._tgt[0]},{self._tgt[1]},0,0,{self._last_seq},0"
        )

    @property
    def in_waiting(self):
        if len(self._rx) == 0:
            self._tel()
        return len(self._rx)

    def readline(self):
        idx = self._rx.find(b"\n")
        if idx < 0:
            self._tel()
            idx = self._rx.find(b"\n")
        line = bytes(self._rx[:idx + 1])
        del self._rx[:idx + 1]
        return line

    def close(self):
        self.is_open = False


# ---------------------------------------------------------------------------
# The link
# ---------------------------------------------------------------------------
class MotorLink:
    def __init__(self, port: str | None = None, baud: int | None = None,
                 sim: bool = False, max_cmd: int | None = None,
                 ttl_ms: int | None = None, heartbeat_hz: float | None = None,
                 verbose: bool = False):
        self.max_cmd = max_cmd or config.MAX_CMD
        self.ttl_ms = ttl_ms or config.DRIVE_TTL_MS
        self.hb_period = 1.0 / (heartbeat_hz or config.HEARTBEAT_HZ)
        self.verbose = verbose

        if sim or serial is None:
            if serial is None and not sim:
                print("[motor_link] pyserial not installed -> SIM mode")
            self.ser = SimSerial()
            self.sim = True
        else:
            self.ser = serial.Serial(port or config.SERIAL_PORT,
                                     baud or config.SERIAL_BAUD, timeout=0.1)
            self.sim = False
            time.sleep(2.0)  # allow the Uno to reset after the port opens

        self._seq = 1
        self._seq_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self.tel = Telemetry()
        self._target = (0, 0)          # latest (left, right) to heartbeat
        self._armed = False
        self._stop = threading.Event()

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._hb = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb.start()

    # -- framing / tx -------------------------------------------------------
    def _next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            if self._seq == 0:
                self._seq = 1
            return s

    def _send(self, payload: str):
        data = frame(payload)
        with self._tx_lock:
            self.ser.write(data)
        if self.verbose:
            print("  ->", payload)

    # -- reader -------------------------------------------------------------
    def _read_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                if getattr(self.ser, "in_waiting", 0):
                    line = self.ser.readline()
                    if line:
                        self._handle(line)
                else:
                    time.sleep(0.002)
            except Exception:
                time.sleep(0.05)

    def _handle(self, raw: bytes):
        try:
            text = raw.decode("ascii", "replace").strip()
        except Exception:
            return
        if "*" not in text:
            return
        payload, _, crc_hex = text.rpartition("*")
        if len(crc_hex) != 4:
            return
        try:
            if int(crc_hex, 16) != crc16_ccitt_false(payload.encode("ascii")):
                return  # corrupt reply, ignore
        except ValueError:
            return
        p = payload.split(",")
        if not p:
            return
        if p[0] == "TEL" and len(p) >= 12:
            self.tel = Telemetry(
                t_wall=time.time(), millis=int(p[1]), state=p[2],
                fault_mask=int(p[3]), cur_l=int(p[4]), cur_r=int(p[5]),
                tgt_l=int(p[6]), tgt_r=int(p[7]), estop=int(p[8]),
                drive_age_ms=int(p[9]), last_seq=int(p[10]), bad_frames=int(p[11]),
                # Ultrasonic fields only exist on the newer firmware.
                dist_cm=int(p[12]) if len(p) > 12 else 0,
                obstacle=int(p[13]) if len(p) > 13 else 0)
        elif p[0] in ("ACK", "HELLO") and len(p) >= 3:
            self.tel.state = p[2] if p[0] == "ACK" else p[2]
        elif p[0] == "NACK" and self.verbose:
            print("  <- NACK", ",".join(p[1:]))

    # -- heartbeat ----------------------------------------------------------
    def _heartbeat_loop(self):
        while not self._stop.is_set():
            if self._armed:
                left, right = self._target
                self._send(f"DRIVE,{self._next_seq()},{left},{right},{self.ttl_ms}")
            time.sleep(self.hb_period)

    # -- public API ---------------------------------------------------------
    def clear(self):
        self._send(f"CLEAR,{self._next_seq()}")

    def arm(self):
        self._send(f"ARM,{self._next_seq()}")
        self._armed = True

    def disarm(self):
        self._armed = False
        self._target = (0, 0)
        self._send(f"DISARM,{self._next_seq()}")

    def ping(self):
        self._send(f"PING,{self._next_seq()}")

    def _clamp(self, v: float) -> int:
        return int(max(-self.max_cmd, min(self.max_cmd, round(v))))

    def set(self, throttle: float, steer: float):
        """throttle and steer in [-1, 1]. Differential mix -> left/right.
        +steer turns right (left wheel drives faster)."""
        throttle = max(-1.0, min(1.0, throttle))
        steer = max(-1.0, min(1.0, steer))
        left = self._clamp((throttle + steer) * self.max_cmd)
        right = self._clamp((throttle - steer) * self.max_cmd)
        self._target = (left, right)

    @property
    def target(self):
        """The (left, right) motor commands currently being heartbeated.
        The simulator reads this back so it drives on the real command path."""
        return self._target

    def stop(self):
        self._target = (0, 0)

    def close(self):
        try:
            self.disarm()
            time.sleep(0.05)
        finally:
            self._stop.set()
            try:
                self.ser.close()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ---------------------------------------------------------------------------
# Quick manual test:  python -m pi.motor_link --sim
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    sim = "--sim" in sys.argv
    print("Opening link (sim=%s)..." % sim)
    link = MotorLink(sim=sim, verbose=True)
    try:
        link.clear(); time.sleep(0.2)
        link.arm();   time.sleep(0.3)
        print("Driving forward gently for 1s...")
        link.set(0.4, 0.0); time.sleep(1.0)
        print("Curving right for 1s...")
        link.set(0.4, 0.5); time.sleep(1.0)
        link.stop(); time.sleep(0.3)
        print("Telemetry:", link.tel)
    finally:
        link.close()
        print("Closed. Faults:", decode_faults(link.tel.fault_mask))
