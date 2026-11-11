"""Settings for the HD-map behavior planner plugin."""

from avlite.c60_apps.c64_settings_schema import SettingsSchema
from pydantic import Field


class PluginSettingsSchema(SettingsSchema):
    sensor_signal_model_path: str = Field(
        default="",
        description="Custom weights matching the selected classifier architecture; empty uses its default artifact.",
    )
    sensor_signal_provider: str = Field(
        default="",
        description="Explicit registered classifier provider; empty uses the selected backend.",
    )
    sensor_signal_backend: str = Field(
        default="carla_aperture",
        pattern="^(carla_aperture|autoware|trained_rgb)$",
        description="Signal backend: legacy CARLA aperture, Autoware MobileNet, or locally trained RGB model.",
    )
    sensor_signal_geometry: str = Field(
        default="", description="Static signal-head survey JSON; no dynamic states."
    )
    stop_lookahead: float = Field(
        default=50.0,
        gt=0.0,
        description="Distance ahead along the global route at which a stop sign becomes active (m).",
    )
    stop_buffer: float = Field(
        default=2.0,
        ge=0.0,
        description="Front-bumper clearance before the mapped stop line or control position (m).",
    )
    stop_hold_time: float = Field(
        default=1.0,
        ge=0.0,
        description="Continuous stopped time before release, measured in simulation seconds.",
    )
    stopped_speed: float = Field(
        default=0.1,
        ge=0.0,
        description="Ego speed at or below which the vehicle counts as stopped (m/s).",
    )
    arrival_tolerance: float = Field(
        default=1.0,
        ge=0.0,
        description="Allowed longitudinal distance from the target stop point (m).",
    )
    passed_tolerance: float = Field(
        default=3.0,
        ge=0.0,
        description="Legacy compatibility setting; overshot stop signs are never silently ignored.",
    )
    max_deceleration: float = Field(
        default=3.0,
        gt=0.0,
        description="Comfortable deceleration used for the stop velocity envelope (m/s^2).",
    )
    crosswalk_speed: float = Field(
        default=2.0, gt=0, description="Approach speed at mapped crossings (m/s)."
    )
    max_lateral_acceleration: float = Field(
        default=0.5, gt=0, description="Curvature-based lateral acceleration budget (m/s^2)."
    )
    pedestrian_margin: float = Field(
        default=1.5, ge=0, description="Crosswalk occupancy margin (m)."
    )
    observation_max_age: float = Field(
        default=0.3, gt=0, description="Maximum dynamic-observation age in simulation seconds."
    )
    telemetry_directory: str = Field(
        default="logs/hdmap", description="Per-run behavior telemetry and events directory."
    )


# The AVLite plugin loader assigns the per-plugin settings filepath.
PluginSettings = PluginSettingsSchema()
