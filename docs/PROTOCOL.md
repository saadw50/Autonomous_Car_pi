# Serial protocol (Pi ↔ Arduino)

This is the contract between `pi/motor_link.py` and
`firmware/arduino_motor_controller/arduino_motor_controller.ino`. The firmware
is the source of truth; `motor_link.py` matches it exactly.

## Framing

```
PAYLOAD*CCCC\n
```

- `PAYLOAD` — ASCII, comma-separated fields.
- `*` — separator.
- `CCCC` — 4 uppercase hex digits, **CRC-16/CCITT-FALSE** over the ASCII
  `PAYLOAD` (init `0xFFFF`, poly `0x1021`, no reflection, no final XOR).
  Check value for `"123456789"` is `0x29B1`. Verified by `tools/test_crc.py`.
- `\n` — line terminator (`\r` is ignored by the firmware).

## Commands (Pi → Arduino)

| Command | Meaning |
|---|---|
| `CLEAR,seq` | Clear latched faults, reset to DISARMED. Bypasses the sequence check. |
| `ARM,seq` | Enter ARMED (refused if E-stop active or faults latched). |
| `DRIVE,seq,left,right,ttl_ms` | Set motor targets. `left`,`right` ∈ [-255,255]; `ttl_ms` ∈ [100,1000]. |
| `DISARM,seq` | Stop and enter DISARMED. |
| `PING,seq` / `STATUS,seq` | Ask for an ACK. |

`seq` is a 16-bit counter. The firmware accepts a frame only if the sequence is
"newer" (delta non-zero and < 0x8000 mod 65536). `motor_link.py` increments
`seq` on every command. `CLEAR` resets the firmware's expectation.

## Replies (Arduino → Pi)

| Reply | Fields |
|---|---|
| `ACK,seq,state,fault_mask` | Command accepted. |
| `NACK,seq,reason` | Rejected (e.g. `BAD_CRC`, `NOT_ARMED`, `ESTOP_ACTIVE`). |
| `TEL,millis,state,fault_mask,cur_l,cur_r,tgt_l,tgt_r,estop,drive_age_ms,last_seq,bad_frames,dist_cm,obstacle` | Telemetry, ~5 Hz. |
| `HELLO,1,state` | Sent once at boot. |

`state` ∈ {`DISARMED`, `ARMED`, `ESTOP`, `FAULT`}.

`dist_cm` and `obstacle` were **appended** by the ultrasonic-capable firmware
(HC-SR04 on Arduino pins 11/12). `dist_cm` is the measured distance in
centimetres, `0` meaning no reading or out of range; `obstacle` is `1` while the
Arduino reflex is holding the motors at zero. Appending rather than inserting
keeps the change backward compatible: a parser that reads only the first eleven
fields still works, and `motor_link.py` defaults both to 0 on older firmware.

`fault_mask` bits: `ESTOP=1`, `WATCHDOG=2`, `RX_OVERFLOW=4`, `BAD_CRC=8`,
`BAD_COMMAND=16`, `BAD_SEQUENCE=32`. `motor_link.decode_faults()` turns the mask
into names.

## The heartbeat

The firmware stops the motors if it does not receive a fresh valid `DRIVE`
within `ttl_ms`. So `motor_link` runs a background thread that sends `DRIVE` at
`HEARTBEAT_HZ` (20 Hz, period 50 ms) using the latest `(left, right)` target,
with `ttl_ms = DRIVE_TTL_MS` (250 ms). The control loop just calls
`link.set(throttle, steer)`; the heartbeat carries it to the firmware. This is
why a slow perception frame doesn't make the car stutter.

## Differential mix

`set(throttle, steer)` with both in [-1, 1]:

```
left  = clamp((throttle + steer) * MAX_CMD, -MAX_CMD, +MAX_CMD)
right = clamp((throttle - steer) * MAX_CMD, -MAX_CMD, +MAX_CMD)
```

`+steer` ⇒ left wheel faster ⇒ turns right. `MAX_CMD` (180) stays ≤ the
firmware's own `MAX_COMMAND`, which clamps again as a backstop.
