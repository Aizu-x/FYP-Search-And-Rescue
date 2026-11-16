"""Run the real bridge server + simulated robot on localhost (closed-loop simulation mode)."""
from __future__ import annotations

from ugv_bridge.protocol import generate_key_hex, parse_key
from ugv_bridge.server import BridgeConfig, BridgeServer

from .edge_client import EdgeClient
from .sim_backend import SimBackend


class SimulatedUGVSession:
    """Owns a SimBackend, a BridgeServer bound to 127.0.0.1:<random> and an EdgeClient."""

    def __init__(self, world_grid, start_pose, half_length=0.10, half_width=0.09, bridge_overrides=None,
                 sim_kwargs=None, link_timeout=0.6):
        self.key = parse_key(generate_key_hex())
        values = dict(bind_host='127.0.0.1', port=0, half_length_m=half_length, half_width_m=half_width,
                      max_vx=0.25, max_wz=1.0)
        values.update(bridge_overrides or {})
        self.backend = SimBackend(world_grid, start_pose, body_radius=max(half_length, half_width),
                                  **(sim_kwargs or {}))
        self.server = BridgeServer(BridgeConfig(values), self.backend, self.key).start()
        self.client = EdgeClient('127.0.0.1', self.server.port, self.key, 'sim-edge', link_timeout=link_timeout).start()

    def close(self):
        self.client.close()
        self.server.shutdown()
