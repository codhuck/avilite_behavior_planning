"""Directional parking command, with the matching-core type when available."""

import math
from dataclasses import dataclass

from avlite.c30_control import c31_control_model as control

if hasattr(control, "DirectionalAckermannCommand"):
    DirectionalAckermannCommand = control.DirectionalAckermannCommand
else:

    @dataclass
    class DirectionalAckermannCommand(control.AckermannControlCommand):
        """Requires a bridge explicitly supporting this command (PlanningBasicSim)."""

        direction: int = 1
        hold_brake: bool = False

        def __post_init__(self):
            if self.direction not in (-1, 1):
                raise ValueError("Direction must be -1 or 1")
            if not all(math.isfinite(v) for v in (self.steer, self.acceleration)):
                raise ValueError("Control values must be finite")
