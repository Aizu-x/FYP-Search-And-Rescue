"""Replaceable ideal holonomic constant-speed polyline follower (meters/seconds)."""
import math


class SimulatedUGV:
    def __init__(self):
        self.route = []; self.position = None; self.index = 0
        self.state = 'idle'; self.speed = 0.25; self.history = []

    def set_route(self, route):
        self.route = [tuple(p) for p in route]; self.index = 0
        self.position = self.route[0] if self.route else None
        self.history = [self.position] if self.position else []
        self.state = 'ready' if self.route else 'idle'

    def start(self):
        if self.route and self.state in ('ready', 'paused'):
            self.state = 'running'

    def pause(self):
        if self.state == 'running':
            self.state = 'paused'

    def reset(self):
        self.set_route(self.route)

    def step(self, dt, guard=None):
        """Guard checks every traversed subsegment before motion, including turns."""
        if not math.isfinite(dt) or dt < 0 or not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('Time must be nonnegative; speed must be positive and finite.')
        if self.state != 'running':
            return
        budget = dt*self.speed
        while self.index < len(self.route)-1 and budget > 0:
            target = self.route[self.index+1]; distance = math.dist(self.position, target)
            travel = min(budget, distance)
            ratio = travel/distance if distance else 1.0
            nxt = tuple(a+(b-a)*ratio for a, b in zip(self.position, target))
            if guard is not None and not guard(self.position, nxt):
                self.state = 'collision'; return
            self.position = nxt; self.history.append(nxt); budget -= travel
            if distance <= travel+1e-12:
                self.index += 1
        if self.index >= len(self.route)-1:
            self.state = 'arrived'
