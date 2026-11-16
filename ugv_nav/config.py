"""Load config/navigation.toml into typed objects."""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields
from pathlib import Path

from ugv_planner.config import Footprint, PlannerConfig

from .controller import ControllerConfig
from .localization import LocalizationConfig
from .navigator import NavigatorConfig
from .safety_monitor import SafetyConfig

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'config' / 'navigation.toml'


@dataclass
class NetworkConfig:
    ugv_host: str = ''
    ugv_port: int = 8765
    key_file: str = 'mentorpi.key'
    client_id: str = 'edge-laptop'
    link_timeout_s: float = 0.6


@dataclass
class EdgeConfig:
    network: NetworkConfig
    environment: str
    planner: PlannerConfig
    navigator: NavigatorConfig
    key_path: Path


def _pick(cls, section):
    names = {f.name for f in fields(cls)}
    unknown = set(section) - names
    if unknown:
        raise ValueError('Unknown keys for %s: %s' % (cls.__name__, sorted(unknown)))
    return cls(**section)


def local_config_path(path=None):
    """config/navigation.local.toml next to config/navigation.toml."""
    path = Path(path or DEFAULT_CONFIG)
    return path.with_name(path.stem + '.local' + path.suffix)


def load_config(path=DEFAULT_CONFIG) -> EdgeConfig:
    path = Path(path)
    data = tomllib.loads(path.read_text(encoding='utf-8'))
    local = local_config_path(path)
    if local.is_file():                # YOUR values (robot size, IP ...): kept out of git, win over the shipped file
        for section, values in tomllib.loads(local.read_text(encoding='utf-8')).items():
            if isinstance(values, dict):
                data.setdefault(section, {}).update(values)
            else:
                data[section] = values
    robot = data.get('robot', {})
    radius = float(robot.get('radius_m', 0.0)) or None
    footprint = Footprint(radius, float(robot.get('margin_m', 0.0)), verified=radius is not None)
    p = data.get('planner', {})
    planner = PlannerConfig(int(p.get('connectivity', 8)), footprint, bool(p.get('inflation', True)),
                            clearance_weight=float(p.get('clearance_weight', 4.0)),
                            preferred_clearance_m=float(p.get('preferred_clearance_m', 0.20)))
    c = dict(data.get('controller', {}))
    rate = float(c.pop('control_rate_hz', 10.0)); ttl = float(c.pop('command_ttl_s', 0.3))
    s = dict(data.get('safety', {}))
    replan = bool(s.pop('replan_on_obstacle', True)); map_range = float(s.pop('obstacle_map_range_m', 1.0))
    auto = bool(s.pop('auto_continue_after_replan', False)); max_auto = int(s.pop('max_auto_replans', 5))
    auto_delay = float(s.pop('auto_continue_delay_s', 1.0))
    safety = _pick(SafetyConfig, dict(s, half_length_m=float(robot.get('half_length_m', 0.0)) or None,
                                      half_width_m=float(robot.get('half_width_m', 0.0)) or None))
    nav = NavigatorConfig(control_rate_hz=rate, command_ttl_s=ttl, use_shortcut=bool(p.get('use_shortcut', False)),
                          smooth_path=bool(p.get('smooth_path', True)),
                          lidar_correction=bool(data.get('lidar_localization', {}).get('continuous_correction', False)),
                          lidar_correction_period_s=float(data.get('lidar_localization', {}).get('period_s', 1.0)),
                          planner_engine=str(p.get('engine', 'fast')),
                          replan_on_obstacle=replan, obstacle_map_range_m=map_range,
                          auto_continue_after_replan=auto, max_auto_replans=max_auto, auto_continue_delay_s=auto_delay,
                          controller=_pick(ControllerConfig, c),
                          localization=_pick(LocalizationConfig, data.get('localization', {})), safety=safety)
    network = _pick(NetworkConfig, data.get('network', {}))
    key_path = Path(network.key_file)
    if not key_path.is_absolute():
        key_path = path.parent / key_path
    resolution = float(data.get('map', {}).get('resolution_m', 0.03))
    if abs(resolution - 0.03) > 1e-12:
        raise ValueError('ugv_planner maps are 0.03 m/cell; config resolution_m must match.')
    return EdgeConfig(network, data.get('map', {}).get('environment', 'ENV1'), planner, nav, key_path)
