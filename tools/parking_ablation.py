"""Frozen one-factor policy definitions for paired parking experiments."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    from dataclasses import asdict, dataclass, replace

    import numpy as np

    @dataclass(frozen=True)
    class ParkingPolicy:
        roof_only: bool = False
        minimum_support: int = 1
        hazard_support_radius: float = 2.0
        adaptive_speed: bool = True
        allow_prefix: bool = True
        allow_observation: bool = True

        def configure(self, planner):
            planner.arbiter.adaptive_speed = self.adaptive_speed
            planner.arbiter.allow_prefix = self.allow_prefix
            planner.allow_observation = self.allow_observation

        def metadata(self):
            return asdict(self)

    FULL = ParkingPolicy()
    POLICIES = {
        "full": FULL,
        "roof_only": replace(FULL, roof_only=True),
        "uniform_one": replace(FULL, hazard_support_radius=0.0),
        "uniform_two": replace(FULL, minimum_support=2, hazard_support_radius=0.0),
        "no_speed_search": replace(FULL, adaptive_speed=False),
        "no_observation": replace(FULL, allow_observation=False),
        "full_segment": replace(FULL, allow_prefix=False),
    }

    def fixed_crossing(scenario):
        """World-coordinate encounter inputs, independent of the tested policy."""
        origin = np.asarray(scenario["pedestrian_origin"], dtype=float)
        direction = np.asarray(scenario["pedestrian_direction"], dtype=float)
        if (
            origin.shape != (2,)
            or direction.shape != (2,)
            or not np.isfinite([*origin, *direction]).all()
            or not np.isclose(np.linalg.norm(direction), 1.0, atol=1e-6)
        ):
            raise ValueError(
                "Fixed pedestrian origin/direction must be finite 2D and unit direction"
            )
        return origin, direction
