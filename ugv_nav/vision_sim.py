"""Simulated camera + detector for testing the YOLO pipeline without the robot.

The simulated robot carries a camera with the same lens/mounting model as the real one.
``SimCamera`` renders simple frames (blocks as coloured boxes) so frames travel through the
real bridge; ``SimDetector`` returns exact YOLO-style boxes for the virtual blocks that are
in view. This exercises projection, tracking, A* updates and the GUI end to end. It does NOT
test the neural network itself (that is tested on the teammate's example photos).
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from .localization import compose, inverse
from .vision import CameraModel


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


class SimDetector:
    """Drop-in for WRODetector.predict(): exact boxes for virtual blocks in view."""

    def __init__(self, backend, blocks: SimBlocks, camera: CameraModel, max_range=2.0, noise_px=0.0, seed=1):
        self.backend, self.blocks, self.camera, self.max_range = backend, blocks, camera, max_range
        self.noise_px = noise_px            # box-edge jitter like a real detector (0 = exact boxes)
        self._rng = np.random.default_rng(seed)

    def predict(self, frame_bgr):
        pose = self.backend.truth_pose()
        out = []
        for _, (bx, by, cls) in self.blocks.items():
            if math.hypot(bx - pose[0], by - pose[1]) > self.max_range:
                continue
            box = _box_for_block(self.camera, pose, bx, by, self.blocks.size, self.blocks.height)
            if box is None:
                continue          # (boxes cut off at the image edge are reported, like real YOLO)
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
    """Renders JPEG frames for the SimBackend (grey floor + coloured block boxes)."""

    def __init__(self, backend, blocks: SimBlocks, camera: CameraModel, rate_hz=10.0):
        self.backend, self.blocks, self.camera = backend, blocks, camera
        self.period = 1.0 / rate_hz
        self._cache = None

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
        for _, (bx, by, cls) in self.blocks.items():
            box = _box_for_block(cam, pose, bx, by, self.blocks.size, self.blocks.height)
            if box is not None:
                color = (60, 160, 60) if cls.startswith('green') else (50, 50, 210)
                cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, -1)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self._cache = (buf.tobytes(), 'jpeg', now) if ok else None
        return self._cache

    def camera_info(self):
        c = self.camera
        return dict(width=c.width, height=c.height, k=[c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0],
                    d=list(c.dist), frame_id='camera')
