"""Simulated camera + detector for testing the YOLO pipeline without the robot.

The simulated robot carries a camera with the same lens/mounting model as the real one.
``SimCamera`` renders simple frames (blocks as coloured boxes) so frames travel through the
real bridge; ``SimDetector`` returns exact YOLO-style boxes for the virtual blocks that are
in view. This exercises projection, tracking, A* updates and the GUI end to end. It does NOT
test the neural network itself (that is tested on the teammate's example photos).

The walls of the simulated arena hide what is behind them (``WallView``): a block behind a wall is
neither drawn nor detected, and the walls themselves are drawn in the picture. The simulator's walls are
taken to be ``WALL_HEIGHT_M`` high, which is higher than the lens, so nothing is seen over them.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from .localization import compose, inverse
from .vision import CameraModel

WALL_HEIGHT_M = 0.20        # the simulator's arena walls for the camera (the lens is 0.145 m above the floor)
RAY_STEP_RAD = math.radians(0.5)


class SimBlocks:
    """Virtual WRO blocks in the simulated world (map metres)."""

    def __init__(self, size=0.05, height=0.10):
        self.size, self.height = size, height
        self._lock = threading.Lock()
        self.blocks = {}       # id -> (x, y, cls)
        self._next = 1

    def add(self, x, y, cls='red_obstacle'):
        with self._lock:
            i = self._next; self._next += 1
            self.blocks[i] = (x, y, cls)
            return i

    def remove(self, block_id):
        with self._lock:
            self.blocks.pop(block_id, None)

    def items(self):
        with self._lock:
            return list(self.blocks.items())


def _box_for_block(cam, robot_pose, bx, by, size, height):
    """Image box of a block for a robot at robot_pose (map frame), or None if not fully usable."""
    inv = inverse(robot_pose)
    # A real camera only sees what is in front of it and inside its field of view. Without this test a
    # block far off to the side projects to a huge box (its corners are almost in the image plane).
    centre_x, centre_y, _ = compose(inv, (bx, by, 0.0))
    dx, dy = centre_x - cam.x_offset_m, centre_y - cam.y_offset_m
    c, s = math.cos(cam.yaw_offset_rad), math.sin(cam.yaw_offset_rad)
    forward, left = c * dx + s * dy, -s * dx + c * dy
    if forward <= 0.05 or abs(math.atan2(left, forward)) > math.atan(cam.width / 2.0 / cam.fx) + math.radians(8.0):
        return None
    pts = []
    for dx in (-size / 2, size / 2):
        for dy in (-size / 2, size / 2):
            rx, ry, _ = compose(inv, (bx + dx, by + dy, 0.0))
            for z in (0.0, height):
                p = cam.ground_to_pixel(rx, ry, z)
                if p is None:
                    return None
                pts.append(p)
    a = np.array(pts)
    x1, y1 = a.min(axis=0); x2, y2 = a.max(axis=0)
    if x2 < 0 or y2 < 0 or x1 >= cam.width or y1 >= cam.height:
        return None
    return [max(0.0, x1), max(0.0, y1), min(cam.width - 1.0, x2), min(cam.height - 1.0, y2)]


class WallView:
    """What the walls of the simulated arena hide from the camera of a robot at ``robot_pose``.

    One ray every half degree across the field of view, along the floor from the lens to the first wall cell
    of ``world`` (an occupancy grid; None = no walls, everything is visible, as before walls were looked at).
    """

    def __init__(self, cam, robot_pose, world, wall_height=WALL_HEIGHT_M, max_range=4.0):
        self.cam, self.pose, self.wall_height = cam, robot_pose, wall_height
        self.lens = compose(robot_pose, (cam.x_offset_m, cam.y_offset_m, cam.yaw_offset_rad))
        self.bearings = self.dist = None
        cells = getattr(world, 'cells', None)
        if cells is None or wall_height <= 0:
            return
        half = math.atan(cam.width / 2.0 / cam.fx) + math.radians(8.0)
        count = int(2 * half / RAY_STEP_RAD) + 1
        self.bearings = np.linspace(-half, half, count)          # from the lens axis, positive = to the left
        step = world.resolution / 2.0
        reach = np.arange(step, max_range, step)
        angle = self.lens[2] + self.bearings
        xs = self.lens[0] + np.outer(np.cos(angle), reach)
        ys = self.lens[1] + np.outer(np.sin(angle), reach)
        cols = np.floor((xs - world.origin[0]) / world.resolution).astype(int)
        rows = np.floor((ys - world.origin[1]) / world.resolution).astype(int)
        height, width = cells.shape
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        wall = np.zeros(xs.shape, dtype=bool)
        wall[inside] = np.asarray(cells)[rows[inside], cols[inside]] != 0
        hit = wall.any(axis=1)
        self.dist = np.where(hit, reach[wall.argmax(axis=1)], np.inf)

    def _rays_at(self, x, y, size):
        """(indices of the rays that point at a thing of this width, its distance from the lens)"""
        dx, dy = x - self.lens[0], y - self.lens[1]
        distance = math.hypot(dx, dy)
        bearing = math.atan2(math.sin(math.atan2(dy, dx) - self.lens[2]), math.cos(math.atan2(dy, dx) - self.lens[2]))
        half = math.atan2(size / 2.0, max(distance, 1e-6))
        index = np.nonzero(np.abs(self.bearings - bearing) <= half)[0]
        if len(index) == 0:                                  # narrower than the gap between two rays
            index = np.array([int(np.argmin(np.abs(self.bearings - bearing)))])
        return index, distance

    def visible(self, x, y, size, height, box, min_fraction=0.5):
        """The part of a thing's image box that no wall hides: [x1, y1, x2, y2], or None when less than
        ``min_fraction`` of its width can be seen (a detector needs to see most of a block to know it)."""
        if self.dist is None or box is None:
            return box
        index, distance = self._rays_at(x, y, size)
        wall = self.dist[index]
        # the line of sight to the middle of the thing, where it passes the wall: below the top of the wall?
        sight = self.cam.height_m + (height / 2.0 - self.cam.height_m) * np.minimum(wall, distance) / max(distance, 1e-6)
        hidden = (wall < distance - size / 2.0) & (sight < self.wall_height)
        seen = np.nonzero(~hidden)[0]
        if len(seen) == 0 or len(seen) < min_fraction * len(index):
            return None
        if len(seen) == len(index):
            return box
        # rays run from right (first) to left (last) in the picture: cut the box to the part that is seen
        x1, y1, x2, y2 = box
        width = x2 - x1
        right, left = seen[0] / len(index), (seen[-1] + 1) / len(index)
        return [x2 - left * width, y1, x2 - right * width, y2]

    def draw(self, image):
        """Paint the walls into the picture (far parts darker), as flat faces between neighbouring rays."""
        if self.dist is None:
            return
        import cv2
        inv, cam = inverse(self.pose), self.cam
        angle = self.lens[2] + self.bearings
        corners = []
        for k, distance in enumerate(self.dist):
            if not np.isfinite(distance):
                corners.append(None)
                continue
            rx, ry, _ = compose(inv, (self.lens[0] + distance * math.cos(angle[k]),
                                      self.lens[1] + distance * math.sin(angle[k]), 0.0))
            foot, top = cam.ground_to_pixel(rx, ry, 0.0), cam.ground_to_pixel(rx, ry, self.wall_height)
            corners.append(None if foot is None or top is None else (foot, top, distance))
        limit = 4 * max(cam.width, cam.height)
        for a, b in zip(corners[:-1], corners[1:]):
            if a is None or b is None:
                continue
            shade = int(np.clip(118 - 22 * (a[2] + b[2]) / 2.0, 62, 118))
            face = np.clip(np.array([a[0], b[0], b[1], a[1]], dtype=float), -limit, limit).astype(np.int32)
            cv2.fillConvexPoly(image, face, (shade + 14, shade + 4, shade - 6))
            cv2.line(image, tuple(int(v) for v in face[3]), tuple(int(v) for v in face[2]), (58, 52, 48), 1)


def _world_of(backend):
    """The occupancy grid the simulated robot drives in (None for a backend without one)."""
    return getattr(backend, 'world', None)


class SimDetector:
    """Drop-in for WRODetector.predict(): exact boxes for virtual blocks in view."""

    def __init__(self, backend, blocks: SimBlocks, camera: CameraModel, max_range=2.0, noise_px=0.0, seed=1,
                 wall_height=WALL_HEIGHT_M):
        self.backend, self.blocks, self.camera, self.max_range = backend, blocks, camera, max_range
        self.noise_px = noise_px            # box-edge jitter like a real detector (0 = exact boxes)
        self.wall_height = wall_height      # walls of the simulated arena hide what is behind them (0 = they do not)
        self._rng = np.random.default_rng(seed)

    def predict(self, frame_bgr):
        pose = self.backend.truth_pose()
        walls = None
        out = []
        for _, (bx, by, cls) in self.blocks.items():
            if math.hypot(bx - pose[0], by - pose[1]) > self.max_range:
                continue
            box = _box_for_block(self.camera, pose, bx, by, self.blocks.size, self.blocks.height)
            if box is None:
                continue          # (boxes cut off at the image edge are reported, like real YOLO)
            if walls is None:
                walls = WallView(self.camera, pose, _world_of(self.backend), self.wall_height)
            box = walls.visible(bx, by, self.blocks.size, self.blocks.height, box)
            if box is None:
                continue          # behind a wall
            if self.noise_px:
                box = [v + float(self._rng.uniform(-self.noise_px, self.noise_px)) for v in box]
                box = [min(max(box[0], 0.0), self.camera.width - 1.0), min(max(box[1], 0.0), self.camera.height - 1.0),
                       min(max(box[2], 0.0), self.camera.width - 1.0), min(max(box[3], 0.0), self.camera.height - 1.0)]
            x1, y1, x2, y2 = [int(round(v)) for v in box]
            out.append(dict(class_id=0 if cls.startswith('green') else 1, class_name=cls, confidence=0.95,
                            bbox_xyxy=[x1, y1, x2, y2], bbox_xywh=[x1, y1, x2 - x1, y2 - y1],
                            center_xy=[(x1 + x2) / 2, (y1 + y2) / 2]))
        return out


class SimCamera:
    """Renders JPEG frames for the SimBackend (grey floor, the arena's walls, coloured block boxes)."""

    def __init__(self, backend, blocks: SimBlocks, camera: CameraModel, rate_hz=10.0, wall_height=WALL_HEIGHT_M):
        self.backend, self.blocks, self.camera = backend, blocks, camera
        self.period = 1.0 / rate_hz
        self.wall_height = wall_height
        self._cache = None

    def things(self, pose):
        """What stands on the floor: [(x, y, size, height, colour (B, G, R), label or None)]."""
        return [(bx, by, self.blocks.size, self.blocks.height,
                 (60, 160, 60) if cls.startswith('green') else (50, 50, 210), None)
                for _, (bx, by, cls) in self.blocks.items()]

    def latest_image(self):
        now = time.monotonic()
        if self._cache is not None and now - self._cache[2] < self.period:
            return self._cache
        import cv2
        cam = self.camera
        img = np.full((cam.height, cam.width, 3), 200, np.uint8)
        horizon = cam.ground_to_pixel(50.0, 0.0)
        if horizon is not None:
            img[:max(0, int(horizon[1])), :] = (150, 150, 150)
        pose = self.backend.truth_pose()
        walls = WallView(cam, pose, _world_of(self.backend), self.wall_height)
        walls.draw(img)
        things = sorted(self.things(pose), key=lambda t: -math.hypot(t[0] - pose[0], t[1] - pose[1]))   # far ones first
        for x, y, size, height, color, name in things:
            box = walls.visible(x, y, size, height, _box_for_block(cam, pose, x, y, size, height), min_fraction=0.0)
            if box is None:
                continue
            p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
            cv2.rectangle(img, p1, p2, color, -1)
            if name:                                        # something with a name: dark outline, the name above
                cv2.rectangle(img, p1, p2, (40, 40, 40), 2)
                cv2.putText(img, name, (p1[0], max(14, p1[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 2,
                            cv2.LINE_AA)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self._cache = (buf.tobytes(), 'jpeg', now) if ok else None
        return self._cache

    def camera_info(self):
        c = self.camera
        return dict(width=c.width, height=c.height, k=[c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0],
                    d=list(c.dist), frame_id='camera')
