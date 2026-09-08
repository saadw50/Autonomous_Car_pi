"""
track_sim.py - a closed-loop driving simulator. No hardware, no network.

Builds a synthetic world (dark track mat + white tape lanes on a bright
"wood" floor), models the car as a differential drive, and renders what the
forward camera would see from the car's pose. Feed the rendered frame to the
real perception stack, feed the resulting motor commands back in, and the car
drives itself in simulation.

Deliberately mimics the real track's key property: the mat is DARK and the
surrounding floor is BRIGHT, so the detector's dark-mat guard behaves the same
way it does on the real images.

Units: world is in centimetres, 1 world pixel = 1 cm.

Run the closed loop with:  python -m sim.run_sim
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

try:
    from .. import config
except (ImportError, ValueError):
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import config

# --- world appearance (tuned to resemble the real track) --------------------
FLOOR_GRAY = 170      # bright surround, like the wood floor
MAT_GRAY = 70         # dark track mat
TAPE_GRAY = 235       # white tape

WORLD_W = WORLD_H = 1600      # cm  (16 m x 16 m)
TRACK_WIDTH = 120             # cm, mat width
LANE_HALF = 45                # cm, tape offset from centreline
TAPE_PX = 5                   # cm, tape thickness

# --- camera geometry (what patch of ground the camera sees) -----------------
CAM_NEAR = 35        # cm ahead of the car at the bottom of the image
CAM_FAR = 260        # cm ahead at the top of the image
CAM_NEAR_HALF = 70   # cm half-width of the visible ground at CAM_NEAR
CAM_FAR_HALF = 230   # cm half-width at CAM_FAR

# --- vehicle ---------------------------------------------------------------
MAX_SPEED_CMS = 60.0   # cm/s at full motor command
TRACK_GAUGE = 18.0     # cm between the two wheels


@dataclass
class Pose:
    x: float
    y: float
    theta: float       # radians; 0 = +x. forward=(cos,sin), right=(-sin,cos)


@dataclass
class WorldObject:
    kind: str          # "stop_sign" | "obstacle"
    x: float
    y: float
    radius: float = 20.0


class TrackWorld:
    """An oval track drawn once into a big top-down image."""

    def __init__(self, axes=(600, 450), center=(WORLD_W // 2, WORLD_H // 2)):
        self.center = center
        self.axes = axes
        self.objects: list[WorldObject] = []
        self.map = self._build()

    def _build(self):
        img = np.full((WORLD_H, WORLD_W), FLOOR_GRAY, np.uint8)
        cx, cy = self.center
        ax, ay = self.axes
        # dark mat band along the centreline
        cv2.ellipse(img, (cx, cy), (ax, ay), 0, 0, 360, MAT_GRAY,
                    thickness=TRACK_WIDTH, lineType=cv2.LINE_AA)
        # white tape on both edges of the lane
        cv2.ellipse(img, (cx, cy), (ax - LANE_HALF, ay - LANE_HALF), 0, 0, 360,
                    TAPE_GRAY, thickness=TAPE_PX, lineType=cv2.LINE_AA)
        cv2.ellipse(img, (cx, cy), (ax + LANE_HALF, ay + LANE_HALF), 0, 0, 360,
                    TAPE_GRAY, thickness=TAPE_PX, lineType=cv2.LINE_AA)
        return img

    def start_pose(self) -> Pose:
        """On the centreline at the right-hand side of the oval, heading down."""
        cx, cy = self.center
        return Pose(x=cx + self.axes[0], y=cy, theta=math.pi / 2)

    def add_stop_sign(self, ahead_cm=250.0, pose: Pose | None = None):
        p = pose or self.start_pose()
        self.objects.append(WorldObject(
            "stop_sign", p.x + math.cos(p.theta) * ahead_cm,
            p.y + math.sin(p.theta) * ahead_cm))

    def add_obstacle(self, ahead_cm=250.0, pose: Pose | None = None):
        p = pose or self.start_pose()
        self.objects.append(WorldObject(
            "obstacle", p.x + math.cos(p.theta) * ahead_cm,
            p.y + math.sin(p.theta) * ahead_cm, radius=25.0))


class CarSim:
    """Differential-drive kinematics driven by the same motor commands the
    firmware would receive (-MAX_CMD .. +MAX_CMD per side)."""

    def __init__(self, pose: Pose, max_cmd: int | None = None):
        self.pose = pose
        self.max_cmd = max_cmd or config.MAX_CMD
        self.trail: list[tuple[float, float]] = []

    def step(self, left_cmd: float, right_cmd: float, dt: float):
        vl = (left_cmd / self.max_cmd) * MAX_SPEED_CMS
        vr = (right_cmd / self.max_cmd) * MAX_SPEED_CMS
        v = 0.5 * (vl + vr)
        # +steer means left wheel faster, which turns the car right (theta up)
        omega = (vl - vr) / TRACK_GAUGE
        p = self.pose
        p.theta += omega * dt
        p.x += v * math.cos(p.theta) * dt
        p.y += v * math.sin(p.theta) * dt
        self.trail.append((p.x, p.y))
        if len(self.trail) > 4000:
            self.trail.pop(0)
        return v, omega


# ---------------------------------------------------------------------------
# Rendering the camera view
# ---------------------------------------------------------------------------
def _ground_quad(pose: Pose):
    """The four world points of the trapezoid of ground the camera sees."""
    ct, st = math.cos(pose.theta), math.sin(pose.theta)
    fwd = np.array([ct, st])
    right = np.array([-st, ct])
    origin = np.array([pose.x, pose.y])
    near = origin + fwd * CAM_NEAR
    far = origin + fwd * CAM_FAR
    return np.float32([
        near - right * CAM_NEAR_HALF,   # near-left  -> image bottom-left
        near + right * CAM_NEAR_HALF,   # near-right -> image bottom-right
        far + right * CAM_FAR_HALF,     # far-right  -> image top-right
        far - right * CAM_FAR_HALF,     # far-left   -> image top-left
    ])


def render_camera(world: TrackWorld, pose: Pose, w=None, h=None,
                  brightness=1.0, glare=0.0, noise=0.0, seed=None):
    """Render the forward camera view as a BGR frame.

    brightness : multiplier (0.5 = dim scene, 1.5 = bright scene)
    glare      : 0..1, adds a bright blob like a reflection/window
    noise      : 0..1, gaussian sensor noise
    """
    w = w or config.CAM_WIDTH
    h = h or config.CAM_HEIGHT
    src = _ground_quad(pose)
    dst = np.float32([[0, h - 1], [w - 1, h - 1], [w - 1, 0], [0, 0]])
    M = cv2.getPerspectiveTransform(src, dst)
    view = cv2.warpPerspective(world.map, M, (w, h),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=FLOOR_GRAY)

    view = view.astype(np.float32)
    if brightness != 1.0:
        view *= brightness
    if glare > 0:
        rng = np.random.default_rng(seed)
        gx = int(rng.uniform(0.1, 0.9) * w)
        gy = int(rng.uniform(0.0, 0.5) * h)
        blob = np.zeros((h, w), np.float32)
        cv2.circle(blob, (gx, gy), int(0.35 * w), 1.0, -1)
        blob = cv2.GaussianBlur(blob, (0, 0), 0.12 * w)
        view += blob * (140.0 * glare)
    if noise > 0:
        rng = np.random.default_rng(seed)
        view += rng.normal(0, 18.0 * noise, view.shape).astype(np.float32)
    view = np.clip(view, 0, 255).astype(np.uint8)
    return cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)


# ---------------------------------------------------------------------------
# Ground-truth "detections" (so the state machine can be tested without YOLO)
# ---------------------------------------------------------------------------
def synth_detections(world: TrackWorld, pose: Pose, w=None, h=None):
    """Return a DetectResult-shaped object from the world's objects, using true
    geometry. Lets stop-sign / obstacle behaviour be tested with no model."""
    from perception.detect import Box, DetectResult
    w = w or config.CAM_WIDTH
    h = h or config.CAM_HEIGHT
    ct, st = math.cos(pose.theta), math.sin(pose.theta)
    boxes = []
    for obj in world.objects:
        dx, dy = obj.x - pose.x, obj.y - pose.y
        fwd = dx * ct + dy * st                 # distance ahead
        lat = -dx * st + dy * ct                # + is to the right
        if fwd < 20 or fwd > CAM_FAR:
            continue
        # apparent size grows as it gets closer
        half_w_cm = CAM_NEAR_HALF + (CAM_FAR_HALF - CAM_NEAR_HALF) * \
            ((fwd - CAM_NEAR) / max(1.0, CAM_FAR - CAM_NEAR))
        if abs(lat) > half_w_cm:
            continue
        cx_frac = 0.5 + 0.5 * (lat / half_w_cm)
        scale = obj.radius / max(1.0, fwd)      # angular size
        area_frac = min(0.6, scale * scale * 6.0)
        side = math.sqrt(area_frac) * w
        x1 = cx_frac * w - side / 2
        y1 = h * 0.45 - side / 2
        cls = config.COCO_STOP_SIGN if obj.kind == "stop_sign" else 2
        boxes.append(Box(cls, 0.9, (x1, y1, x1 + side, y1 + side),
                         area_frac, cx_frac))
    return DetectResult(t_wall=0.0, boxes=boxes, fps=0.0)


# ---------------------------------------------------------------------------
# Top-down debug view
# ---------------------------------------------------------------------------
def render_map(world: TrackWorld, pose: Pose, car: CarSim | None = None,
               size=420):
    img = cv2.cvtColor(world.map, cv2.COLOR_GRAY2BGR)
    if car and len(car.trail) > 1:
        pts = np.int32(car.trail).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, (0, 200, 255), 3)
    quad = _ground_quad(pose).astype(np.int32)
    cv2.polylines(img, [quad], True, (0, 255, 0), 3)
    cv2.circle(img, (int(pose.x), int(pose.y)), 12, (0, 0, 255), -1)
    for o in world.objects:
        col = (0, 0, 255) if o.kind == "stop_sign" else (255, 0, 255)
        cv2.circle(img, (int(o.x), int(o.y)), int(o.radius), col, -1)
    return cv2.resize(img, (size, size))


if __name__ == "__main__":
    world = TrackWorld()
    pose = world.start_pose()
    frame = render_camera(world, pose)
    print("camera frame:", frame.shape)
    cv2.imwrite("sim_camera_view.jpg", frame)
    cv2.imwrite("sim_map.jpg", render_map(world, pose))
    print("wrote sim_camera_view.jpg and sim_map.jpg")
