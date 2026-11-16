from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Footprint:
    """Circular bounding footprint. None means dimensions have not been supplied."""
    radius_m: float | None = None
    margin_m: float = 0.0
    verified: bool = False

    def __post_init__(self):
        for value in (self.radius_m, self.margin_m):
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError('Radius and margin must be finite and nonnegative.')
        if self.verified and self.radius_m is None:
            raise ValueError('Supply a radius before marking the footprint verified.')

    @property
    def effective_radius(self):
        return (self.radius_m or 0.0) + self.margin_m


@dataclass(frozen=True)
class PlannerConfig:
    connectivity: int = 8
    footprint: Footprint = Footprint()
    inflation: bool = True
    record_search: bool = False
    clearance_weight: float = 0.0
    preferred_clearance_m: float = 0.20

    def __post_init__(self):
        if self.connectivity not in (4, 8):
            raise ValueError('Connectivity must be 4 or 8.')
        if not math.isfinite(self.clearance_weight) or self.clearance_weight < 0:
            raise ValueError('Clearance weight must be finite and nonnegative.')
        if not math.isfinite(self.preferred_clearance_m) or self.preferred_clearance_m <= 0:
            raise ValueError('Preferred extra clearance must be finite and positive.')

    @property
    def planning_radius(self):
        return self.footprint.effective_radius if self.inflation else 0.0


# Approximate requested world locations, quantized explicitly to cell centers.
# Chosen before planning, using occupancy only; GT never selects endpoints.
EXAMPLES = {
    'ENV1': ((0.345, 0.315), (2.655, 2.595)),
    'ENV2': ((0.345, 0.315), (2.505, 2.505)),
    'ENV3': ((0.345, 0.315), (1.965, 1.965)),
}
