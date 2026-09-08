"""
Central configuration for the autonomous car.

Everything tunable lives here so the rest of the code stays clean. Values are
grouped by subsystem. Units are noted on every line that has one.

Nothing in this file talks to hardware; importing it is always safe.
"""

# ----------------------------------------------------------------------------
# Serial link to the Arduino motor controller
# ----------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyACM0"   # Arduino Uno usually enumerates here on the Pi.
SERIAL_BAUD = 115200           # MUST match hardware_config.h SERIAL_BAUD.

# The firmware clamps DRIVE magnitude to its own MAX_COMMAND (180 by default).
# We scale our normalized commands to this before sending; keep <= firmware max.
MAX_CMD = 180                  # 0..255. Output ceiling for each motor channel.

# Watchdog TTL sent with every DRIVE frame. The firmware stops the motors if it
# does not get a fresh DRIVE within this window. MUST be > heartbeat period and
# within the firmware's [MIN_WATCHDOG_MS, MAX_WATCHDOG_MS] = [100, 1000].
DRIVE_TTL_MS = 250
HEARTBEAT_HZ = 20              # DRIVE frames per second (period = 50 ms).

# ----------------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------------
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30

# ----------------------------------------------------------------------------
# Lane detection
# ----------------------------------------------------------------------------
# ROI is a trapezoid expressed as fractions of the frame (0..1).
# (top_y is where the road starts; narrower at top like a real road ahead.)
ROI_TOP_Y = 0.55               # ignore everything above this fraction of height
ROI_TOP_LEFT = 0.30
ROI_TOP_RIGHT = 0.70
ROI_BOTTOM_LEFT = 0.00
ROI_BOTTOM_RIGHT = 1.00

# White-tape extraction.
# The track is a dark glossy mat with thin white tape, on a wood floor, under
# uneven light with glare. The robust trick is WHITE TOP-HAT morphology: it
# keeps bright structures THINNER than the kernel (the tape) and removes large
# bright areas (wood floor, big glare, windows) and gradual lighting. CLAHE
# normalizes contrast first so it works in both dim and bright frames.
USE_TOPHAT = True
CLAHE_CLIP = 2.5               # contrast-limited equalization strength
TOPHAT_KERNEL = 31             # px; must be LARGER than the tape width so tape
                               #   survives the top-hat. Tape ~15-25px at 640w.
TOPHAT_MIN = 28                # min top-hat intensity to count as tape; guards
                               #   against low-contrast wood grain (raise to
                               #   reject more, lower to catch dimmer tape).
BRIGHT_MIN = 135               # tape is genuinely bright white; a pixel must be
                               #   at least this bright (0-255 L) AND pass the
                               #   top-hat. This is what rejects mid-gray wood
                               #   grain, whose streaks are thin but not white.
MAX_TAPE_COVERAGE = 0.26       # if the tape mask fills more than this fraction
                               #   of the ROI, it's glare/wood, not tape -> the
                               #   detector drops confidence instead of lying.
MIN_SEG_SUPPORT = 70           # a lane side needs at least this many pixels of
                               #   total line length; scattered short wood-grain
                               #   segments can't reach it, so they're rejected.
# On-track test. The track is a DARK mat carrying BRIGHT tape, so the ROI's
# median sits far below its bright peak. Bare wood/floor is uniformly mid-toned,
# so its median sits close to its peak.
#
# We compare median/p95 as a RATIO rather than an absolute brightness, because a
# ratio is invariant to overall exposure - a camera with auto-exposure, or a
# brighter room, scales both numbers together. (An absolute threshold silently
# breaks the moment the scene brightens; the simulator caught exactly that.)
#
# Measured:  real lanes 0.295-0.427 | real wood 0.714-0.902
#            simulated lanes (dim/normal/bright/glare) 0.283-0.443
MAT_DARK_RATIO_MAX = 0.60      # above this => not on the mat => reject

# Legacy fixed-Otsu path (used only if USE_TOPHAT = False).
LANE_L_BIAS = 0                # added to the Otsu threshold; +brighter-only

# Hough parameters.
HOUGH_THRESHOLD = 42
HOUGH_MIN_LINE = 35
HOUGH_MAX_GAP = 20
MIN_ABS_SLOPE = 0.40           # drop near-horizontal segments (noise/cross lines)

# Exponential smoothing on the measured lane centre (0..1, higher = smoother).
LANE_SMOOTHING = 0.6

# How many consecutive frames without a confident lane before we call it lost.
LANE_LOST_FRAMES = 10

# ----------------------------------------------------------------------------
# Object / sign detection (YOLO)
# ----------------------------------------------------------------------------
YOLO_MODEL = "yolov8n.pt"      # swapped for an NCNN export on the Pi for speed.
YOLO_IMGSZ = 320               # smaller = faster on the Pi.
YOLO_CONF = 0.45
DETECT_HZ = 6                  # inference rate; control loop never waits on it.

# COCO class ids we care about.
COCO_STOP_SIGN = 11
COCO_OBSTACLES = (0, 1, 2, 3, 5, 7, 15, 16)  # person, bike, car, moto, bus, truck, cat, dog

# A detection only matters when it is big enough (close enough). Fractions of
# frame area.
STOP_SIGN_MIN_AREA = 0.010     # ~10% of frame width square
OBSTACLE_MIN_AREA = 0.045
# Obstacles only stop us if they sit roughly ahead, not off to the far edge.
OBSTACLE_CENTER_BAND = 0.60    # central fraction of frame width to watch

# ----------------------------------------------------------------------------
# Control (steering + speed)
# ----------------------------------------------------------------------------
BASE_THROTTLE = 0.45           # cruise throttle, normalized 0..1
MIN_THROTTLE = 0.25            # floor when steering hard
KP = 0.9                       # proportional gain on normalized cross-track error
KD = 0.35                      # derivative gain
STEER_SLEW = 0.15              # max change in steer command per control tick
THROTTLE_SLEW = 0.08

# ----------------------------------------------------------------------------
# Behaviours / manoeuvres
# ----------------------------------------------------------------------------
STOP_SIGN_HOLD_S = 3.0         # seconds to sit still at a stop sign
STOP_SIGN_COOLDOWN_S = 8.0     # ignore the same sign for this long after resuming
OBSTACLE_CLEAR_S = 0.5         # obstacle must be gone this long before resuming

# U-turn is an open-loop timed manoeuvre (no encoders yet). Tune on the bench.
UTURN_THROTTLE = 0.40
UTURN_STEER = 1.0              # full lock
UTURN_DURATION_S = 2.6         # time to sweep ~180 deg; MEASURE THIS on your car
UTURN_TRIGGER_LANE_LOST_S = 2.0  # auto U-turn after lane lost this long (dead end)
UTURN_COOLDOWN_S = 6.0         # after finishing a U-turn, don't let the
                               #   dead-end trigger fire another one until the
                               #   car has had time to re-acquire the lane.
                               #   Without this the car U-turns repeatedly:
                               #   turn -> lane briefly lost -> turn again.

# ----------------------------------------------------------------------------
# Ultrasonic (HC-SR04) - independent obstacle reflex on the Pi GPIO
# ----------------------------------------------------------------------------
# This is a fast, cheap safety layer that does NOT depend on vision or the
# Arduino: if anything is closer than the stop distance, the car halts.
ULTRASONIC_ENABLED = True
# BCM pin numbers (not physical pin numbers). You have several sensors - add
# more entries for side sensors. WIRING WARNING: the HC-SR04 ECHO pin outputs
# 5 V; the Pi GPIO is 3.3 V only. Put a voltage divider on every ECHO line
# (e.g. 1 kOhm from ECHO to the Pi pin, 2 kOhm from that pin to GND) or you can
# damage the Pi. TRIG can be driven directly at 3.3 V.
ULTRASONIC_SENSORS = [
    {"name": "front", "trig": 23, "echo": 24},
    # {"name": "left",  "trig": 17, "echo": 27},
    # {"name": "right", "trig": 5,  "echo": 6},
]
ULTRASONIC_MAX_M = 2.0         # sensor range ceiling
ULTRASONIC_STOP_M = 0.30       # stop if the front sensor sees anything closer
ULTRASONIC_CLEAR_M = 0.40      # must be at least this far to resume (hysteresis)
ULTRASONIC_POLL_HZ = 15        # how often the background thread pings

# ----------------------------------------------------------------------------
# Logging
# ----------------------------------------------------------------------------
LOG_DIR = "logs"
LOG_VIDEO = True               # record the annotated feed of each run
LOG_TELEMETRY = True
