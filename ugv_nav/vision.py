"""Camera + YOLO obstacle pipeline (runs on the edge laptop).

    UGV camera frame --(bridge)--> laptop --> WRO YOLOv5n detector (third_party/wro_yolo, unchanged)
        --> floor projection (camera lens calibration + mounting) --> robot frame --> map frame
        --> ObstacleTracker (confirm over several frames; remove only when the spot is in view and empty)
        --> Navigator.set_vision_obstacles() --> A* replans

Frames
------
Robot frame: x forward, y left, z up, origin = robot centre on the floor (same as the bridge).
Camera frame (OpenCV): x right, y down, z forward (optical axis).

The camera is mounted ``height_m`` above the floor, ``x_offset_m`` ahead of the robot centre,
``y_offset_m`` to the left, tilted down by ``pitch_rad`` and turned by ``yaw_offset_rad``.
Height is measured (14.5 cm); pitch / x offset / yaw are fitted by ``calibrate()`` from
2-3 blocks placed at measured positions.

Distance comes from where the block touches the floor (bottom edge of the YOLO box): the
WRO blocks (10 cm tall) are usually below the LiDAR plane, so the camera is the only sensor
that sees them.
"""
from __future__ import annotations

import importlib.util
import math
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from .localization import transform_point

ROOT = Path(__file__).resolve().parents[1]
WRO_DIR = ROOT / 'third_party' / 'wro_yolo'

# Lens calibration published by the user's MentorPi (/ascamera/camera_publisher/rgb0/camera_info).
MENTORPI_K = (473.4506985179141, 474.2169451085363, 323.5512181265506, 238.6016133237558)
MENTORPI_D = (-0.02749631166333748, -0.000535160640560652, -0.005899822232353089,
              0.003423120278803321, -0.03559287109655516)


# --------------------------------------------------------------------------- detector
def load_detector(model_path=None, conf_threshold=0.5):
    """Load the teammate's WRO detector without modifying it. Raises ImportError with a hint."""
    try:
        spec = importlib.util.spec_from_file_location('wro_detector', WRO_DIR / 'wro_detector.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as exc:
        raise ImportError('YOLO needs extra packages on the laptop: '
                          '.\\.venv\\Scripts\\python.exe -m pip install -r requirements-yolo.txt  (%s)' % exc)
    return module.WRODetector(model_path or WRO_DIR / 'models' / 'best.onnx', conf_threshold=conf_threshold)


# --------------------------------------------------------------------------- camera geometry
@dataclass
class CameraModel:
    fx: float = MENTORPI_K[0]
    fy: float = MENTORPI_K[1]
    cx: float = MENTORPI_K[2]
    cy: float = MENTORPI_K[3]
    dist: tuple = MENTORPI_D                 # plumb_bob k1, k2, p1, p2, k3
    width: int = 640
    height: int = 480
    height_m: float = 0.145                  # lens above the floor (measured)
    pitch_rad: float = math.radians(10.0)    # tilt down (fitted by calibration)
    x_offset_m: float = 0.08                 # lens ahead of robot centre (fitted)
    y_offset_m: float = 0.0                  # lens left of robot centre
    yaw_offset_rad: float = 0.0              # lens turned left (fitted)

    @classmethod
    def from_camera_info(cls, info, base=None):
        base = base or cls()
        k = info['k']
        d = tuple(info.get('d') or ()) or base.dist
        return replace(base, fx=k[0], fy=k[4], cx=k[2], cy=k[5], dist=tuple(d[:5]) + (0.0,) * (5 - len(d[:5])),
                       width=int(info.get('width') or base.width), height=int(info.get('height') or base.height))

    # -- lens distortion (OpenCV plumb_bob) -------------------------------------------------
    def _distort(self, x, y):
        k1, k2, p1, p2, k3 = (tuple(self.dist) + (0.0,) * 5)[:5]
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        return (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)

    def _undistort(self, xd, yd):
        x, y = xd, yd
        for _ in range(20):
            dx, dy = self._distort(x, y)
            x, y = x - (dx - xd), y - (dy - yd)
        return x, y

    # -- axes of the camera in the robot frame ---------------------------------------------
    def _axes(self):
        p, w = self.pitch_rad, self.yaw_offset_rad
        cp, sp, cw, sw = math.cos(p), math.sin(p), math.cos(w), math.sin(w)
        forward = np.array([cp * cw, cp * sw, -sp])      # optical axis (z_c)
        right = np.array([sw, -cw, 0.0])                 # x_c
        down = np.cross(forward, right)                  # y_c = z_c x x_c
        return right, down, forward

    def normalized(self, u, v):
        """Undistorted normalized image coordinates of a pixel (independent of the mounting)."""
        return self._undistort((u - self.cx) / self.fx, (v - self.cy) / self.fy)

    def pixel_to_ground(self, u, v):
        """Floor point (x, y) in the robot frame seen at pixel (u, v), or None above the horizon."""
        return self.normalized_to_ground(*self.normalized(u, v))

    def normalized_to_ground(self, xn, yn, z=0.0):
        """Point at height z (default: the floor) seen along the ray of normalized coords (xn, yn)."""
        right, down, forward = self._axes()
        ray = xn * right + yn * down + forward
        if ray[2] > -1e-6 or self.height_m <= z:
            return None
        t = (self.height_m - z) / -ray[2]
        return (self.x_offset_m + t * ray[0], self.y_offset_m + t * ray[1])

    def ground_to_pixel(self, x, y, z=0.0):
        """Pixel of a robot-frame point, or None if behind the camera."""
        right, down, forward = self._axes()
        rel = np.array([x - self.x_offset_m, y - self.y_offset_m, z - self.height_m])
        zc = rel @ forward
        if zc <= 1e-6:
            return None
        xd, yd = self._distort((rel @ right) / zc, (rel @ down) / zc)
        return (self.fx * xd + self.cx, self.fy * yd + self.cy)

    def sees(self, x, y, margin_px=20, max_range=1.5, min_range=0.12):
        """True if the floor point (robot frame) is inside the image and in reliable range."""
        d = math.hypot(x - self.x_offset_m, y - self.y_offset_m)
        if not min_range <= d <= max_range:
            return False
        px = self.ground_to_pixel(x, y)
        return (px is not None and margin_px <= px[0] <= self.width - margin_px
                and margin_px <= px[1] <= self.height - margin_px)


def block_centre_from_box(box_xyxy, cam: CameraModel, block_depth=0.05, scale=(1.0, 1.0), block_height=0.10):
    """Robot-frame centre of a block from its YOLO box.

    Normally the bottom edge (where the block touches the floor) is projected onto the floor.
    If the block is so close that its bottom is cut off by the image, the TOP edge is projected onto
    the plane of the block top (the WRO blocks are 10 cm tall, the camera is higher), which is the
    block's far top edge. Returns None if neither edge is usable or the box is cut off at a side.
    ``scale`` converts frame pixels to calibration pixels if the frame size differs.
    """
    x1, y1, x2, y2 = box_xyxy
    u = (x1 + x2) / 2.0 * scale[0]
    if x1 * scale[0] <= 2 or x2 * scale[0] >= cam.width - 3:
        return None          # cut off at the image side: box centre is not the block centre
    if y2 * scale[1] < cam.height - 3:
        return _centre_from_normalized(cam.normalized(u, y2 * scale[1]), cam, block_depth)
    if y1 * scale[1] > 3 and cam.height_m > block_height + 0.01:
        top = cam.normalized_to_ground(*cam.normalized(u, y1 * scale[1]), z=block_height)
        if top is None:
            return None
        dx, dy = top[0] - cam.x_offset_m, top[1] - cam.y_offset_m
        n = math.hypot(dx, dy) or 1.0
        return (top[0] - dx / n * block_depth / 2, top[1] - dy / n * block_depth / 2)   # far edge -> centre
    return None


def _centre_from_normalized(norm, cam, block_depth):
    ground = cam.normalized_to_ground(*norm)
    if ground is None:
        return None
    gx, gy = ground
    dx, dy = gx - cam.x_offset_m, gy - cam.y_offset_m
    n = math.hypot(dx, dy) or 1.0
    return (gx + dx / n * block_depth / 2, gy + dy / n * block_depth / 2)   # near face -> centre


def calibrate(samples, cam: CameraModel, block_depth=0.05):
    """Fit pitch, x offset and yaw offset from samples [(box_xyxy, (true_x, true_y) robot frame)].

    Returns (fitted CameraModel, rms error in metres). Needs >= 2 samples at different distances.
    """
    if len(samples) < 2:
        raise ValueError('Add at least 2 calibration samples (blocks at different distances).')

    prepared = []
    for box, target in samples:
        x1, y1, x2, y2 = box
        if y2 >= cam.height - 3:
            raise ValueError('A calibration block touches the bottom of the image; move it further away.')
        prepared.append((cam.normalized((x1 + x2) / 2.0, y2), target))   # lens part computed once

    def cost(c):
        total = 0.0
        for norm, (tx, ty) in prepared:
            p = _centre_from_normalized(norm, c, block_depth)
            if p is None:
                return float('inf')
            total += (p[0] - tx) ** 2 + (p[1] - ty) ** 2
        return total

    def make(v):
        return replace(cam, pitch_rad=float(v[0]), x_offset_m=float(v[1]), yaw_offset_rad=float(v[2]))

    # 1) coarse grid over plausible mountings, 2) Nelder-Mead from the best few starts.
    coarse = []
    for p in np.radians(np.arange(-10, 50.1, 2.0)):
        for x in np.arange(-0.05, 0.25, 0.02):
            for w in np.radians(np.arange(-15, 15.1, 2.0)):
                v = np.array([p, x, w])
                coarse.append((cost(make(v)), v))
    coarse.sort(key=lambda item: item[0])
    best = (float('inf'), None)
    for _, start in coarse[:5]:
        v, e = _nelder_mead(lambda v: cost(make(v)), start, np.array([math.radians(2.0), 0.02, math.radians(2.0)]))
        if e < best[0]:
            best = (e, v)
    best = (best[0], make(best[1]))
    return best[1], math.sqrt(best[0] / len(samples))


def _nelder_mead(f, x0, step, iters=400, tol=1e-12):
    """Small Nelder-Mead minimiser (no SciPy dependency)."""
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = np.argsort(vals); pts = [pts[i] for i in order]; vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        centroid = np.mean(pts[:-1], axis=0)
        xr = centroid + (centroid - pts[-1]); fr = f(xr)
        if fr < vals[0]:
            xe = centroid + 2 * (centroid - pts[-1]); fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = centroid + 0.5 * (pts[-1] - centroid); fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


# --------------------------------------------------------------------------- camera tilt from block size
def box_fully_visible(box_xyxy, cam: CameraModel, scale=(1.0, 1.0)):
    x1, y1, x2, y2 = box_xyxy
    return (x1 * scale[0] > 2 and x2 * scale[0] < cam.width - 3
            and y1 * scale[1] > 3 and y2 * scale[1] < cam.height - 3)


def pitch_from_box(box_xyxy, cam: CameraModel, block_height=0.10, block_depth=0.06, scale=(1.0, 1.0)):
    """Camera tilt implied by ONE fully visible block of known height -- independent of the configured tilt.

    The bottom edge of the box is the block's near floor edge, the top edge its far top edge (the
    camera looks down onto the block top). With the image angles a_b, a_t of those two edges below
    the optical axis and tilt p, the forward distances are

        d_near = h_cam / tan(p + a_b)          d_far = (h_cam - H) / tan(p + a_t) = d_near + depth

    One equation, one unknown: p. Returns (pitch_rad, forward_distance_m) or None if the box is
    cut off, the camera is not above the block top, or no plausible tilt (-5..50 deg) fits.
    """
    if cam.height_m <= block_height + 0.01 or not box_fully_visible(box_xyxy, cam, scale):
        return None
    x1, y1, x2, y2 = box_xyxy
    u = (x1 + x2) / 2.0 * scale[0]
    a_b = math.atan(cam.normalized(u, y2 * scale[1])[1])
    a_t = math.atan(cam.normalized(u, y1 * scale[1])[1])
    if a_b <= a_t:
        return None
    hc, top = cam.height_m, cam.height_m - block_height

    def mismatch(p):
        return top / math.tan(p + a_t) - hc / math.tan(p + a_b) - block_depth

    lo = max(math.radians(-5.0), -a_t + math.radians(0.3))     # keep both rays below the horizon
    hi = math.radians(50.0)
    if lo >= hi:
        return None
    f_lo = mismatch(lo)
    step = math.radians(0.5)
    p = lo + step
    while p <= hi:                                              # first sign change, then bisection
        f = mismatch(p)
        if (f_lo > 0) != (f > 0):
            a, b = p - step, p
            for _ in range(40):
                m = (a + b) / 2
                if (mismatch(m) > 0) == (f_lo > 0):
                    a = m
                else:
                    b = m
            pitch = (a + b) / 2
            distance = hc / math.tan(pitch + a_b)
            return (pitch, distance) if 0.12 <= distance <= 3.0 else None
        p += step
    return None


class TiltEstimator:
    """Robust running estimate of the camera tilt from many block sightings (median + spread)."""

    def __init__(self, window=80, min_samples=12, max_spread_deg=1.2, min_range=0.25, max_range=1.6):
        self.window, self.min_samples, self.max_spread_deg = window, min_samples, max_spread_deg
        self.min_range, self.max_range = min_range, max_range
        self.samples = []

    def reset(self):
        self.samples = []

    def add(self, pitch_rad, distance_m):
        if self.min_range <= distance_m <= self.max_range:
            self.samples.append(math.degrees(pitch_rad))
            del self.samples[:-self.window]

    def estimate(self):
        """dict(deg, spread_deg, n, confident) or None before the first sample."""
        if not self.samples:
            return None
        values = np.array(self.samples)
        median = float(np.median(values))
        spread = float(1.4826 * np.median(np.abs(values - median)))
        return dict(deg=round(median, 2), spread_deg=round(spread, 2), n=len(values),
                    confident=len(values) >= self.min_samples and spread <= self.max_spread_deg)


def block_size_fault(box_xyxy, cam: CameraModel, block_height=0.10, block_depth=0.05, scale=(1.0, 1.0)):
    """'' if the box can be a 10 cm block standing on the floor, otherwise the reason it cannot.

    The bottom edge of the box gives the floor distance; a block at that distance has a known height
    in the picture. Something much taller (a door, a wall poster, a person) or much smaller (a far
    away coloured thing) is not a block. A box that runs out of the top of the picture counts as "at
    least this tall". Boxes whose bottom is cut off cannot be judged here.
    """
    x1, y1, x2, y2 = box_xyxy
    if cam.height_m <= block_height + 0.01 or y2 * scale[1] >= cam.height - 3:
        return ''
    u = (x1 + x2) / 2.0 * scale[0]
    near = cam.normalized_to_ground(*cam.normalized(u, y2 * scale[1]))
    if near is None:
        return 'above the floor line'               # bottom edge at/above the horizon: not standing on the floor
    dx, dy = near[0] - cam.x_offset_m, near[1] - cam.y_offset_m
    n = math.hypot(dx, dy) or 1.0
    top = cam.ground_to_pixel(near[0] + dx / n * block_depth, near[1] + dy / n * block_depth, block_height)
    if top is None:
        return ''
    expected = y2 * scale[1] - top[1]
    actual = (y2 - y1) * scale[1]
    if expected <= 1:
        return ''
    if actual > 1.7 * expected + 6:
        return 'too tall for a block'
    top_cut = y1 * scale[1] <= 3
    wide = (x2 - x1) * scale[0] > 1.4 * actual      # a block lying on its side is half as tall
    if not top_cut and actual < (0.3 if wide else 0.5) * expected - 4:
        return 'too small for a block'
    return ''


def colour_fraction(frame_bgr, box_xyxy, cls):
    """Share of the box centre that really has the class colour (green / red). None if unknown class."""
    import cv2
    name = str(cls).lower()
    if not (name.startswith('green') or name.startswith('red')):
        return None
    h, w = frame_bgr.shape[:2]
    x1, y1, x2, y2 = box_xyxy
    mx, my = 0.2 * (x2 - x1), 0.2 * (y2 - y1)       # centre 60 %: the box edge is background
    a, b = max(0, int(x1 + mx)), max(0, int(y1 + my))
    c, d = min(w, int(math.ceil(x2 - mx))), min(h, int(math.ceil(y2 - my)))
    if c - a < 2 or d - b < 2:
        return None
    hsv = cv2.cvtColor(frame_bgr[b:d, a:c], cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    if name.startswith('green'):
        mask = (hue >= 35) & (hue <= 95) & (sat >= 60) & (val >= 40)
    else:
        mask = ((hue <= 8) | (hue >= 172)) & (sat >= 90) & (val >= 50)
    return float(mask.mean())


# --------------------------------------------------------------------------- tracking
@dataclass
class TrackerConfig:
    min_confidence: float = 0.6
    assoc_dist_m: float = 0.12        # same block if within this distance ...
    assoc_per_range: float = 0.15     # ... or within this share of its distance (far sightings scatter more)
    merge_dist_m: float = 0.18        # two same-colour pins this close are ONE block seen twice
    confirm_hits: int = 4             # seen this many times ...
    confirm_window_s: float = 2.0     # ... within this time before it becomes an obstacle
    remove_misses: int = 8            # spot in view but nothing seen this many frames -> removed
    max_range_m: float = 1.5          # detections further away are ignored (projection too uncertain)
    block_size_m: float = 0.05
    block_height_m: float = 0.10
    weight_cap: float = 25.0          # limits how "stiff" a position average can become
    report_shift_m: float = 0.03      # tell the navigator when an estimate moved this far


@dataclass
class Track:
    id: int
    cls: str
    x: float
    y: float
    conf: float
    hits: int = 1
    first: float = 0.0
    last_seen: float = 0.0
    misses: int = 0
    confirmed: bool = False
    weight: float = 1.0               # sum of observation weights (close, confident sightings count more)
    votes: dict = field(default_factory=dict)      # class -> summed weight (stops red/green flicker)
    reported: tuple = None            # position last given to the navigator


class ObstacleTracker:
    def __init__(self, config: TrackerConfig = TrackerConfig()):
        self.cfg = config
        self.tracks = {}
        self._next = 1
        self.events = []      # (time, text)

    def obstacles(self):
        return [t for t in self.tracks.values() if t.confirmed]

    def clear(self):
        changed = any(t.confirmed for t in self.tracks.values())
        self.tracks.clear()
        return changed

    def soften(self):
        """Let new sightings dominate quickly (after the camera calibration changed)."""
        for t in self.tracks.values():
            t.weight = min(t.weight, 1.0)

    def update(self, detections, visible, now, covered=None):
        """detections: [(map_x, map_y, class_name, confidence[, range_m])]; visible(map_x, map_y) -> bool;
        covered(map_x, map_y, cls) -> bool: a detection box in this frame covers that spot (e.g. a block so
        close that its position cannot be computed). A covered obstacle counts as seen, never as missing.

        Position = weighted average: weight = confidence / range^2, so close sightings (accurate) dominate
        far ones (a small tilt error is a large distance error at range). The class is a weighted vote,
        so one object seen sometimes as red and sometimes as green stays ONE obstacle.

        Returns True if the set of confirmed obstacles (or their reported positions/classes) changed.
        """
        c = self.cfg
        changed = False
        matched = set()
        for det in detections:
            x, y, cls, conf = det[:4]
            rng = det[4] if len(det) > 4 else None
            if conf < c.min_confidence:
                continue
            w = 1.0 if rng is None else conf / max(float(rng), 0.25) ** 2
            best, best_d = None, max(c.assoc_dist_m, c.assoc_per_range * float(rng or 0.0))
            for t in self.tracks.values():
                d = math.hypot(t.x - x, t.y - y)
                if d <= best_d and t.id not in matched:
                    best, best_d = t, d
            if best is None:
                t = Track(self._next, cls, x, y, conf, first=now, last_seen=now, weight=w, votes={cls: w})
                self.tracks[t.id] = t
                self._next += 1
                matched.add(t.id)
                continue
            alpha = w / (best.weight + w)
            best.x += alpha * (x - best.x); best.y += alpha * (y - best.y)
            best.weight = min(best.weight + w, c.weight_cap)
            best.votes[cls] = best.votes.get(cls, 0.0) + w
            best.hits += 1; best.conf = max(best.conf * 0.9, conf); best.last_seen = now; best.misses = 0
            matched.add(best.id)
            top = max(best.votes, key=best.votes.get)
            if top != best.cls:
                if best.confirmed:
                    changed = True
                    self.events.append((now, 'reclassified %s -> %s at (%.2f, %.2f)' % (best.cls, top, best.x, best.y)))
                best.cls = top
            if not best.confirmed and best.hits >= c.confirm_hits and now - best.first <= c.confirm_window_s + 1e-9:
                best.confirmed = True
                best.reported = (best.x, best.y)
                changed = True
                self.events.append((now, 'added %s at (%.2f, %.2f)' % (best.cls, best.x, best.y)))
            elif best.confirmed and math.dist(best.reported, (best.x, best.y)) > c.report_shift_m:
                best.reported = (best.x, best.y)
                changed = True
        for t in list(self.tracks.values()):
            if t.id in matched:
                continue
            if not t.confirmed:
                if now - t.first > c.confirm_window_s:
                    del self.tracks[t.id]          # never confirmed: noise
                continue
            if covered is not None and covered(t.x, t.y, t.cls):
                t.misses = 0
                t.last_seen = now
                continue
            if visible(t.x, t.y):
                t.misses += 1
                if t.misses >= c.remove_misses:
                    del self.tracks[t.id]
                    changed = True
                    self.events.append((now, 'removed %s at (%.2f, %.2f): spot in view, nothing seen' % (t.cls, t.x, t.y)))
        changed = self._merge_duplicates(now) or changed
        del self.events[:-30]
        return changed

    def _merge_duplicates(self, now):
        """One block must be one pin: same-colour tracks closer than ``merge_dist_m`` are joined."""
        c = self.cfg
        changed = False
        tracks = sorted(self.tracks.values(), key=lambda t: (not t.confirmed, -t.weight, t.id))
        gone = set()
        for i, keep in enumerate(tracks):
            if keep.id in gone:
                continue
            for other in tracks[i + 1:]:
                if other.id in gone or other.cls != keep.cls or \
                        math.hypot(keep.x - other.x, keep.y - other.y) > c.merge_dist_m:
                    continue
                total = keep.weight + other.weight
                keep.x += (other.x - keep.x) * other.weight / total
                keep.y += (other.y - keep.y) * other.weight / total
                keep.weight = min(total, c.weight_cap)
                keep.hits += other.hits
                keep.last_seen = max(keep.last_seen, other.last_seen)
                for name, w in other.votes.items():
                    keep.votes[name] = keep.votes.get(name, 0.0) + w
                gone.add(other.id)
                if other.confirmed:
                    changed = True
                    self.events.append((now, 'two %s pins at (%.2f, %.2f) were one block: joined'
                                        % (keep.cls, keep.x, keep.y)))
        for tid in gone:
            del self.tracks[tid]
        return changed


# --------------------------------------------------------------------------- worker
@dataclass
class VisionStats:
    fps: float = 0.0
    frames: int = 0
    status: str = 'off'
    ignored_shape: int = 0            # detections dropped because the box cannot be a block


class VisionWorker:
    """Background thread: newest camera frame -> YOLO -> map positions -> tracker -> navigator.

    ``auto_tilt``: when enough fully visible blocks have been seen, the camera tilt implied by their
    known height replaces the configured tilt (``on_camera_change(camera, estimate)`` tells the console).
    ``shape_filter``: boxes that cannot be a block (far too wide/flat, or a size that contradicts the
    floor distance once the tilt is known) are shown but not put on the map.
    """

    def __init__(self, client, navigator_getter, detector, camera: CameraModel, tracker=None,
                 max_turn_rate=0.35, clock=time.monotonic, auto_tilt=False, shape_filter=True,
                 on_camera_change=None, colour_check=False, min_colour=0.25):
        self.client = client
        self.get_nav = navigator_getter
        self.detector = detector
        self.camera = camera
        self.tracker = tracker or ObstacleTracker()
        self.max_turn_rate = max_turn_rate
        self.clock = clock
        self.auto_tilt = auto_tilt
        self.shape_filter = shape_filter
        self.colour_check = colour_check      # the box must really contain its class colour (green / red)
        self.min_colour = min_colour
        self.on_camera_change = on_camera_change
        self.tilt = TiltEstimator()
        self.stats = VisionStats()
        self.frame = None                     # (frame id, JPEG bytes from the UGV, width, height)
        self.last_boxes = []                  # dicts for the display, see process()
        self.frame_size = (camera.width, camera.height)
        self._lock = threading.Lock()
        self._frame_ready = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._thread = None
        self._last_image_time = None

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name='vision', daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        with self._lock:
            self._frame_ready.notify_all()
        if self._thread:
            self._thread.join(timeout=2)

    def set_camera(self, camera, keep_tilt_samples=False):
        """New mounting/calibration from the operator. Positions computed with the old one are dropped
        (blocks still in view are found again within a few frames)."""
        with self._lock:
            old = self.camera
            self.camera = camera
            if not keep_tilt_samples:
                self.tilt.reset()
            big = (abs(old.pitch_rad - camera.pitch_rad) > math.radians(1.5)
                   or abs(old.yaw_offset_rad - camera.yaw_offset_rad) > math.radians(1.5)
                   or abs(old.x_offset_m - camera.x_offset_m) > 0.02 or abs(old.height_m - camera.height_m) > 0.01)
            changed = self.tracker.clear() if big else False
            self.tracker.soften()
        nav = self.get_nav()
        if changed and nav is not None:
            nav.set_vision_obstacles([], reason='camera calibration changed')

    def snapshot(self):
        with self._lock:
            return dict(boxes=list(self.last_boxes), fps=self.stats.fps, frames=self.stats.frames,
                        status=self.stats.status, frame_id=None if self.frame is None else self.frame[0],
                        obstacles=[(t.id, t.cls, t.x, t.y, t.conf) for t in self.tracker.obstacles()],
                        events=list(self.tracker.events[-6:]), camera=self.camera, frame_size=self.frame_size,
                        tilt=self.tilt.estimate(), ignored_shape=self.stats.ignored_shape)

    def wait_frame(self, after_id=None, timeout=2.0):
        """Newest frame as (id, jpeg bytes, width, height, boxes), waiting up to ``timeout`` for one newer
        than ``after_id``. None if nothing new arrived."""
        with self._lock:
            if self.frame is None or self.frame[0] == after_id:
                self._frame_ready.wait(timeout)
            if self.frame is None or self.frame[0] == after_id:
                return None
            return self.frame + (list(self.last_boxes),)

    def clear_obstacles(self):
        with self._lock:
            changed = self.tracker.clear()
        nav = self.get_nav()
        if nav is not None and changed:
            nav.set_vision_obstacles([], reason='vision obstacles cleared by operator')

    def _loop(self):
        times = []
        while not self._stop.is_set():
            image = self.client.latest_image()
            if image is None or image[2] == self._last_image_time:
                self._stop.wait(0.02)
                continue
            self._last_image_time = image[2]
            try:
                self.process(image[0])
            except Exception as exc:   # vision problems must never stop navigation
                with self._lock:
                    self.stats.status = 'error: %s' % exc
            now = self.clock()
            times = [t for t in times if now - t < 2.0] + [now]
            with self._lock:
                self.stats.fps = (len(times) - 1) / max(now - times[0], 1e-6) if len(times) > 1 else 0.0

    def _block_like(self, box, cam, scale):
        """False for boxes that cannot be an upright or fallen 5 x 5 x 10 cm block (e.g. a long barrier)."""
        if not box_fully_visible(box, cam, scale):
            return True                        # cut-off boxes are judged by position only
        w = (box[2] - box[0]) * scale[0]
        h = (box[3] - box[1]) * scale[1]
        # upright block: taller than wide. A square / round thing (w ~ h) is not a block; a block lying
        # on its side is clearly wider than tall.
        return h > 2 and (0.25 <= w / h <= 0.95 or 1.4 <= w / h <= 2.4)

    def process(self, encoded):
        """Run one frame (also used directly by tests). Returns the display boxes."""
        import cv2
        frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('could not decode camera frame')
        detections = self.detector.predict(frame)
        now = self.clock()
        nav = self.get_nav()
        snap = nav.snapshot(blocking=False) if nav is not None else None
        pose = snap['pose'] if snap and snap['localization'][0] == 'OK' else None
        tel = self.client.snapshot()['telemetry'] if hasattr(self.client, 'snapshot') else None
        turning = abs(((tel or {}).get('odom') or {}).get('wz', 0.0)) > self.max_turn_rate
        cfg = self.tracker.cfg
        with self._lock:
            cam = self.camera
        h, w = frame.shape[:2]
        scale = (cam.width / float(w), cam.height / float(h))

        # 1) camera tilt from the known block height (works without a map pose, never while turning fast)
        tilt_samples = []
        for det in detections:
            if det['confidence'] >= cfg.min_confidence and self._block_like(det['bbox_xyxy'], cam, scale):
                sample = pitch_from_box(det['bbox_xyxy'], cam, cfg.block_height_m, scale=scale)
                tilt_samples.append(sample)
                if sample is not None and not turning:
                    self.tilt.add(*sample)
            else:
                tilt_samples.append(None)
        estimate = self.tilt.estimate()
        off_by = abs(estimate['deg'] - math.degrees(cam.pitch_rad)) if estimate else 0.0
        # Sightings disagree with the configured tilt but there are not enough yet: wait with the map
        # (a few frames) rather than pin blocks at positions that are about to be corrected.
        learning = bool(self.auto_tilt and estimate and not estimate['confident'] and off_by > 1.0
                        and any(sample is not None for sample in tilt_samples))
        if self.auto_tilt and estimate and estimate['confident'] and off_by > 0.4:
            cam = replace(cam, pitch_rad=math.radians(estimate['deg']))
            with self._lock:
                self.camera = cam
                dropped = self.tracker.clear() if off_by > 1.5 else False    # old positions used the wrong tilt
                self.tracker.soften()
                self.tracker.events.append((now, 'camera tilt set to %.1f deg from %d block sightings'
                                            % (estimate['deg'], estimate['n'])))
            if dropped and nav is not None:
                nav.set_vision_obstacles([], reason='camera tilt corrected')
            if self.on_camera_change is not None:
                try:
                    self.on_camera_change(cam, estimate)
                except Exception:
                    pass

        # 2) map positions
        base = getattr(nav, 'base_grid', None) if nav is not None else None
        boxes, mapped = [], []
        rejected = shape = 0
        for det, sample in zip(detections, tilt_samples):
            box = [float(v) for v in det['bbox_xyxy']]
            entry = dict(box=box, cls=det['class_name'], conf=float(det['confidence']), range=None, map=None,
                         used=False, note='')
            boxes.append(entry)
            if self.shape_filter and not self._block_like(box, cam, scale):
                entry['note'] = 'not block-shaped'
                shape += 1
                continue
            fault = block_size_fault(box, cam, cfg.block_height_m, cfg.block_size_m, scale) if self.shape_filter else ''
            if fault:
                entry['note'] = fault                 # e.g. a door detected as a red block
                shape += 1
                continue
            if self.colour_check:
                share = colour_fraction(frame, box, det['class_name'])
                if share is not None and share < self.min_colour:
                    entry['note'] = 'colour does not match'
                    shape += 1
                    continue
            robot_xy = block_centre_from_box(box, cam, cfg.block_size_m, scale, cfg.block_height_m)
            if robot_xy is None:
                entry['note'] = 'cut off'
                continue
            rng = math.hypot(robot_xy[0] - cam.x_offset_m, robot_xy[1] - cam.y_offset_m)
            entry['range'] = rng
            entry['robot'] = (float(robot_xy[0]), float(robot_xy[1]))
            if (self.shape_filter and sample is not None and estimate and estimate['confident']
                    and abs(math.degrees(sample[0]) - estimate['deg']) > 4.0):
                entry['note'] = 'size does not fit a block'     # e.g. a big green object far away
                shape += 1
                continue
            if math.hypot(*robot_xy) > cfg.max_range_m:
                entry['note'] = 'too far'
                continue
            if pose is None:
                entry['note'] = 'no map pose'
                continue
            map_xy = transform_point(pose, robot_xy)
            if base is not None and _impossible_position(base, map_xy):
                entry['note'] = 'inside a wall / outside the map'
                rejected += 1          # projection error: calibrate the camera
                continue
            entry['map'] = (float(map_xy[0]), float(map_xy[1]))
            entry['used'] = not turning
            mapped.append((map_xy[0], map_xy[1], det['class_name'], det['confidence'], rng))
        changed = False
        if learning:
            for entry in boxes:
                entry['used'] = False
                entry['note'] = entry['note'] or 'learning the camera tilt'
        elif pose is not None and not turning:   # skip map updates while turning fast (motion blur, pose lag)
            c, s = math.cos(pose[2]), math.sin(pose[2])

            def to_robot(mx, my):
                dx, dy = mx - pose[0], my - pose[1]
                return c * dx + s * dy, -s * dx + c * dy

            def visible(mx, my):
                return cam.sees(*to_robot(mx, my), max_range=cfg.max_range_m)

            def covered(mx, my, cls):
                rx, ry = to_robot(mx, my)
                for z in (0.0, cfg.block_height_m / 2, cfg.block_height_m):   # any part of the block inside a box
                    px = cam.ground_to_pixel(rx, ry, z)
                    if px is None:
                        continue
                    u, v = px[0] / scale[0], px[1] / scale[1]
                    for b in boxes:
                        x1, y1, x2, y2 = b['box']
                        if x1 - 10 <= u <= x2 + 10 and y1 - 10 <= v <= y2 + 10:
                            return True
                return False
            with self._lock:
                changed = self.tracker.update(mapped, visible, now, covered)
        with self._lock:
            self.last_boxes = boxes
            self.frame_size = (w, h)
            self.stats.frames += 1
            self.stats.ignored_shape += shape
            self.frame = (self.stats.frames, bytes(encoded), w, h)
            self.stats.status = ('learning the camera tilt from block size (%d sightings)' % estimate['n'] if learning
                                 else 'turning: map updates paused' if turning else
                                 'no map pose (confirm the robot pose)' if pose is None else
                                 '%d detection(s) on a wall/outside the map ignored: check the camera tilt' % rejected
                                 if rejected else 'ok')
            obstacles = [dict(id=t.id, cls=t.cls, x=t.x, y=t.y, size=cfg.block_size_m, conf=round(t.conf, 2))
                         for t in self.tracker.obstacles()]
            self._frame_ready.notify_all()
        if changed and nav is not None:
            nav.set_vision_obstacles(obstacles, reason='camera')
        return boxes


def _impossible_position(grid, map_xy):
    """A block cannot be inside a mapped wall or outside the arena (allows 1 cell of slack)."""
    r, c = grid.world_to_cell(map_xy)
    h, w = grid.shape
    if not (0 <= r < h and 0 <= c < w):
        return True
    window = grid.cells[max(0, r - 1):r + 2, max(0, c - 1):c + 2]
    return bool(window.all())       # every surrounding cell is wall: certainly not a free-standing block
