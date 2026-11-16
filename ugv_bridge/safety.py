"""Pure UGV-side safety checks (no I/O), shared with the edge for consistency."""
from __future__ import annotations

import math


def scan_points(scan, mount, max_range=None, min_valid=0.0):
    """Convert a LaserScan dict to (x, y) points in the robot base frame.

    mount = (x, y, yaw) of the LiDAR in the base frame. Invalid, NaN/inf,
    below max(range_min, min_valid) or above range_max readings are ignored.
    """
    if not scan:
        return []
    mx, my, myaw = mount
    c, s = math.cos(myaw), math.sin(myaw)
    lo = max(float(scan.get('range_min', 0.0)), min_valid)
    hi = float(scan.get('range_max', float('inf')))
    if max_range is not None:
        hi = min(hi, max_range)
    angle = float(scan['angle_min']); inc = float(scan['angle_increment'])
    points = []
    for i, r in enumerate(scan['ranges']):
        if r is None or not math.isfinite(r) or r < lo or r > hi:
            continue
        a = angle + i * inc
        lx, ly = r * math.cos(a), r * math.sin(a)
        points.append((mx + c * lx - s * ly, my + s * lx + c * ly))
    return points


def obstacle_blocking(points, vx, vy, wz, half_length, half_width, stop_distance, rotate_clearance):
    """Return a reason string if the commanded motion is blocked, else None.

    Translation: an obstacle inside the rectangle swept by the robot body over
    ``stop_distance`` in the direction of travel blocks motion. Rotation: any
    point inside the circle that encloses the body plus ``rotate_clearance``
    blocks turning in place. Points inside the body rectangle itself are
    treated as self-returns only if they are within the body (e.g. chassis);
    they never block (configure the mount/min range to exclude them).
    """
    speed = math.hypot(vx, vy)
    if speed > 1e-6:
        ux, uy = vx / speed, vy / speed
        for px, py in points:
            if abs(px) <= half_length and abs(py) <= half_width:
                continue  # inside body outline: self-return
            along = px * ux + py * uy
            across = -px * uy + py * ux
            # Body extent along/across the travel direction (conservative for diagonal travel).
            extent_along = abs(half_length * ux) + abs(half_width * uy)
            extent_across = abs(half_length * uy) + abs(half_width * ux)
            if 0.0 < along <= extent_along + stop_distance and abs(across) <= extent_across:
                return 'obstacle %.2f m ahead' % (along - extent_along)
    if abs(wz) > 1e-6:
        radius = math.hypot(half_length, half_width) + rotate_clearance
        for px, py in points:
            if abs(px) <= half_length and abs(py) <= half_width:
                continue
            if math.hypot(px, py) <= radius:
                return 'obstacle within turning radius'
    return None


def stopping_distance(speed, margin, reaction_time, decel):
    """Distance needed to stop from ``speed``: margin + reaction travel + braking travel."""
    speed = abs(speed)
    return margin + speed * reaction_time + (speed * speed / (2.0 * decel) if decel > 0 else 0.0)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def slew(current, target, max_delta):
    """Acceleration-limit increases in magnitude; decreases (braking) are immediate."""
    if abs(target) <= abs(current) and (target == 0 or (target > 0) == (current > 0)):
        return target
    if (target > 0) != (current > 0) and current != 0:
        return 0.0  # pass through zero before reversing
    step = max(-max_delta, min(max_delta, target - current))
    return current + step
