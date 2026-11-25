"""Settings for the HD-map behavior planner plugin."""

from typing import Literal

from avlite.c60_apps.c64_settings_schema import SettingsSchema
from pydantic import Field


class PluginSettingsSchema(SettingsSchema):
    mission_file: str = Field(
        default="", description="Drive-to-parking mission JSON; empty disables the task."
    )
    decision_status_file: str = Field(
        default="", description="Optional JSON status file for the read-only mission monitor."
    )
    cruise_speed_limit: float | None = Field(
        default=None,
        gt=0,
        description="Optional reference-speed ceiling in m/s; the community demo uses 3.0.",
    )
    coordinator_initial_strategy: str = Field(
        default="auto",
        description="PlanningCoordinatorTask startup selection: auto, road-rules, road-tree or race. Auto selects by map type.",
    )
    parking_hazard_support_radius: float = Field(
        default=2.0,
        ge=0.0,
        description="Require two independent ground observations within this distance of a LiDAR hazard (multi-LiDAR only).",
    )
    parking_minimum_sensor_support: int = Field(
        default=1,
        ge=1,
        le=8,
        description="Independent synchronized LiDARs required to confirm each ground cell; higher values can require stopping.",
    )
    parking_multi_lidar: bool = Field(
        default=False,
        description="Fuse simultaneous calibrated LiDARs; explicit ego-footprint exception requires a collision-free initial pose.",
    )
    parking_active_observation: bool = Field(
        default=False,
        description="Enable bounded visibility-aware observation manoeuvres for sensor parking.",
    )
    parking_scene_file: str = Field(
        default="", description="Surveyed parking spaces JSON in localization coordinates."
    )
    parking_lidar_occupancy: bool = Field(
        default=False,
        description="Use current LiDAR ground cells for parking; unseen cells block motion.",
    )
    parking_ground_z: float | None = Field(
        default=None,
        description="Surveyed flat parking ground elevation; required for LiDAR occupancy.",
    )
    parking_simulator_obstacles: bool = Field(
        default=False,
        description="BasicSim demonstration only: use simulator occupancy instead of a ParkingObservation producer.",
    )

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
    sensor_clock_offset: float | None = Field(
        default=None,
        allow_inf_nan=False,
        description="Runtime time minus sensor time (s). None anchors the first frame assembly stamp to runtime; hardware should supply a calibrated offset.",
    )
    required_observation_origin: Literal["any", "sensor", "simulator_truth", "scenario"] = Field(
        default="any",
        description="Accepted regulatory evidence origin. Sensor profiles require sensor; any preserves legacy scenarios.",
    )
    observation_max_age: float = Field(
        default=0.3, gt=0, description="Maximum dynamic-observation age in executor-clock seconds."
    )
    telemetry_directory: str = Field(
        default="logs/hdmap", description="Per-run behavior telemetry and events directory."
    )


# The AVLite plugin loader assigns the per-plugin settings filepath.
PluginSettings = PluginSettingsSchema()
