"""
test_crc.py - proves the Pi-side CRC matches the firmware.

Pure Python, no dependencies. Run it anywhere:
    python tools/test_crc.py

The check value 0x29B1 for "123456789" is the standard CRC-16/CCITT-FALSE
vector; if this passes, our framing agrees with crc16CcittFalse() in the .ino.
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pi.motor_link import crc16_ccitt_false, frame


def check(name, got, want):
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {got}, want {want}")
    return ok


def main():
    ok = True
    # 1) canonical vector
    ok &= check("CRC of '123456789'",
                f"0x{crc16_ccitt_false(b'123456789'):04X}", "0x29B1")

    # 2) a real command frame round-trips (payload*CRC recomputes clean)
    payload = "DRIVE,42,120,-120,250"
    f = frame(payload).decode().strip()
    body, _, crc_hex = f.rpartition("*")
    ok &= check("frame body preserved", body, payload)
    recomputed = f"{crc16_ccitt_false(body.encode()):04X}"
    ok &= check("frame CRC self-consistent", crc_hex, recomputed)

    # 3) known ARM frame example
    p = "ARM,1"
    print(f"  info: ARM,1 -> {frame(p).decode().strip()}")

    # 4) empty payload edge case
    check("CRC of '' is 0xFFFF", f"0x{crc16_ccitt_false(b''):04X}", "0xFFFF")

    print("\nRESULT:", "ALL PASS" if ok else "FAILURES ABOVE")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
