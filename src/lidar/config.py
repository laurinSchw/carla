"""Validated LiDAR sensor configuration, resource limits and the attribute schema (handoff §5.5)."""

from __future__ import annotations

from typing import Dict, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

SENSOR_SEMANTIC = "ray_cast_semantic"  # sensor.lidar.ray_cast_semantic: tag + hit actor id, no intensity/noise
SENSOR_RAYCAST = "ray_cast"            # sensor.lidar.ray_cast: intensity, noise/dropoff, no tags
SensorType = Literal["ray_cast_semantic", "ray_cast"]

BLUEPRINT_IDS = {t: f"sensor.lidar.{t}" for t in (SENSOR_SEMANTIC, SENSOR_RAYCAST)}

#: attributes that exist only on sensor.lidar.ray_cast (not on the semantic sensor); names as in the CARLA 0.9.x
#: blueprint library — to be verified against 0.9.16 on a live server
RAYCAST_ONLY = ("noise_stddev", "dropoff_general_rate", "dropoff_intensity_limit",
                "dropoff_zero_intensity", "atmosphere_attenuation_rate")


class Mount(BaseModel):
    """Sensor pose relative to the vehicle (CARLA frame: x forward, y right, z up; degrees)."""
    model_config = ConfigDict(extra="forbid", frozen=True)
    x_m: float = Field(default=0.0, ge=-10.0, le=10.0)
    y_m: float = Field(default=0.0, ge=-10.0, le=10.0)
    z_m: float = Field(default=2.4, ge=-1.0, le=10.0)      # roof
    pitch_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    yaw_deg: float = Field(default=0.0, ge=-180.0, le=180.0)
    roll_deg: float = Field(default=0.0, ge=-90.0, le=90.0)


class LidarConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    sensor_type: SensorType = SENSOR_SEMANTIC
    channels: int = Field(default=32, ge=1, le=256)
    range_m: float = Field(default=100.0, gt=0.0, le=300.0)
    points_per_second: int = Field(default=500_000, ge=1_000, le=10_000_000)
    #: None = one full sweep per simulation step, i.e. 1 / fixed_delta_seconds (resolved by ``resolve``)
    rotation_frequency_hz: Optional[float] = Field(default=None, gt=0.0, le=100.0)
    upper_fov_deg: float = Field(default=15.0, ge=-90.0, le=90.0)
    lower_fov_deg: float = Field(default=-30.0, ge=-90.0, le=90.0)
    horizontal_fov_deg: float = Field(default=360.0, gt=0.0, le=360.0)
    mount: Mount = Field(default_factory=Mount)

    # sensor.lidar.ray_cast only
    noise_stddev: Optional[float] = Field(default=None, ge=0.0, le=10.0)
    dropoff_general_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_intensity_limit: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_zero_intensity: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    atmosphere_attenuation_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def _rules(self) -> "LidarConfig":
        if self.lower_fov_deg >= self.upper_fov_deg:
            raise ValueError("lower_fov_deg must be below upper_fov_deg")
        if self.sensor_type == SENSOR_SEMANTIC:
            used = [n for n in RAYCAST_ONLY if getattr(self, n) is not None]
            if used:
                raise ValueError(f"{used} exist only on sensor type 'ray_cast', not on 'ray_cast_semantic'")
        return self

    # -- derived -------------------------------------------------------------------------------
    def resolve(self, fixed_delta_seconds: float) -> "LidarConfig":
        """Fill / validate ``rotation_frequency_hz`` so that one measurement is one full sweep (defect L7)."""
        want = 1.0 / fixed_delta_seconds
        if self.rotation_frequency_hz is None:
            return self.model_copy(update={"rotation_frequency_hz": want})
        if abs(self.rotation_frequency_hz - want) > 1e-6 * want:
            raise ValueError(
                f"rotation_frequency_hz {self.rotation_frequency_hz} must equal 1/fixed_delta_seconds = {want:g} "
                f"so that each simulation step delivers one full sweep"
            )
        return self

    def points_per_frame(self, fixed_delta_seconds: float) -> int:
        return int(round(self.points_per_second * fixed_delta_seconds))

    @property
    def blueprint_id(self) -> str:
        return BLUEPRINT_IDS[self.sensor_type]

    def blueprint_attributes(self) -> Dict[str, str]:
        """String attributes for ``blueprint.set_attribute`` (None-valued optionals are left at CARLA's default)."""
        if self.rotation_frequency_hz is None:
            raise ValueError("call resolve() first: rotation_frequency_hz is not set")
        attrs = {
            "channels": self.channels,
            "range": self.range_m,
            "points_per_second": self.points_per_second,
            "rotation_frequency": self.rotation_frequency_hz,
            "upper_fov": self.upper_fov_deg,
            "lower_fov": self.lower_fov_deg,
            "horizontal_fov": self.horizontal_fov_deg,
        }
        if self.sensor_type == SENSOR_RAYCAST:
            for n in RAYCAST_ONLY:
                if getattr(self, n) is not None:
                    attrs[n] = getattr(self, n)
        return {k: str(v) for k, v in attrs.items()}


class LidarLimits(BaseModel):
    """Server-side resource policy (handoff §5.5 budget and auto-release)."""
    model_config = ConfigDict(extra="forbid", frozen=True)
    max_sensors: int = Field(default=4, ge=0)
    max_total_points_per_second: int = Field(default=2_000_000, ge=0)
    keep_alive_s: float = Field(default=10.0, ge=0.0)   # wall time without viewers before auto-release
    frame_timeout_s: float = Field(default=0.25, gt=0.0)  # how long a tick waits for a sensor's measurement
    max_consecutive_misses: int = Field(default=40, ge=1)  # then the sensor is declared dead (ERROR) and freed; 40 = 2 s at 20 FPS


#: stream profile quantisation (display only; recorded data is never subsampled)
POINT_BUCKETS = (10_000, 25_000, 50_000)


#: CARLA semantic tag ids with names and colours as in CARLA's documented table (CityScapes palette). Ids 24+ are
#: the extended 0.9.12+ set. Unverified against the 0.9.16 documentation: treat names/colours as display hints.
SEMANTIC_TAGS = (
    (0, "Unlabeled", (0, 0, 0)), (1, "Roads", (128, 64, 128)), (2, "SideWalks", (244, 35, 232)),
    (3, "Building", (70, 70, 70)), (4, "Wall", (102, 102, 156)), (5, "Fence", (190, 153, 153)),
    (6, "Pole", (153, 153, 153)), (7, "TrafficLight", (250, 170, 30)), (8, "TrafficSign", (220, 220, 0)),
    (9, "Vegetation", (107, 142, 35)), (10, "Terrain", (152, 251, 152)), (11, "Sky", (70, 130, 180)),
    (12, "Pedestrian", (220, 20, 60)), (13, "Rider", (255, 0, 0)), (14, "Car", (0, 0, 142)),
    (15, "Truck", (0, 0, 70)), (16, "Bus", (0, 60, 100)), (17, "Train", (0, 80, 100)),
    (18, "Motorcycle", (0, 0, 230)), (19, "Bicycle", (119, 11, 32)), (20, "Static", (110, 190, 160)),
    (21, "Dynamic", (170, 120, 50)), (22, "Other", (55, 90, 80)), (23, "Water", (45, 60, 150)),
    (24, "RoadLine", (157, 234, 50)), (25, "Ground", (81, 0, 81)), (26, "Bridge", (150, 100, 100)),
    (27, "RailTrack", (230, 150, 140)), (28, "GuardRail", (180, 165, 180)),
)


def _attr(name, label, kind, unit, lo, hi, default, types):
    return {"name": name, "label": label, "type": kind, "unit": unit, "min": lo, "max": hi,
            "default": default, "sensor_types": list(types)}


def lidar_schema(limits: Optional[Dict[str, Any]] = None) -> dict:
    """JSON description of what can be configured, per sensor type, plus the semantic tag table; the web UI builds its
    LiDAR forms from it. Only attributes valid for a sensor type are listed under it. Names and ranges follow CARLA 0.9.x and
    are unverified against 0.9.16."""
    both = (SENSOR_SEMANTIC, SENSOR_RAYCAST)
    d, m = LidarConfig(), Mount()
    common = [
        _attr("channels", "Channels", "int", "", 1, 256, d.channels, both),
        _attr("range_m", "Range", "float", "m", 0.0, 300.0, d.range_m, both),
        _attr("points_per_second", "Points per second", "int", "pt/s", 1_000, 10_000_000, d.points_per_second, both),
        _attr("rotation_frequency_hz", "Rotation frequency", "float", "Hz", 0.0, 100.0, 20.0, both),
        _attr("upper_fov_deg", "Upper FOV", "float", "deg", -90, 90, d.upper_fov_deg, both),
        _attr("lower_fov_deg", "Lower FOV", "float", "deg", -90, 90, d.lower_fov_deg, both),
        _attr("horizontal_fov_deg", "Horizontal FOV", "float", "deg", 0.0, 360.0, d.horizontal_fov_deg, both),
    ]
    mount = [
        _attr("mount.x_m", "Mount x", "float", "m", -10, 10, m.x_m, both),
        _attr("mount.y_m", "Mount y", "float", "m", -10, 10, m.y_m, both),
        _attr("mount.z_m", "Mount z", "float", "m", -1, 10, m.z_m, both),
        _attr("mount.pitch_deg", "Mount pitch", "float", "deg", -90, 90, m.pitch_deg, both),
        _attr("mount.yaw_deg", "Mount yaw", "float", "deg", -180, 180, m.yaw_deg, both),
        _attr("mount.roll_deg", "Mount roll", "float", "deg", -90, 90, m.roll_deg, both),
    ]
    ray = (SENSOR_RAYCAST,)
    raycast_only = [
        _attr("noise_stddev", "Noise std", "float", "m", 0.0, 10.0, None, ray),
        _attr("dropoff_general_rate", "General dropoff rate", "float", "", 0.0, 1.0, None, ray),
        _attr("dropoff_intensity_limit", "Dropoff intensity limit", "float", "", 0.0, 1.0, None, ray),
        _attr("dropoff_zero_intensity", "Dropoff at zero intensity", "float", "", 0.0, 1.0, None, ray),
        _attr("atmosphere_attenuation_rate", "Atmosphere attenuation", "float", "1/m", 0.0, 1.0, None, ray),
    ]

    def for_type(t):
        return [a for a in common + mount + raycast_only if t in a["sensor_types"]]

    return {
        "protocol_version": 3,
        "sensor_types": {
            SENSOR_SEMANTIC: {"label": "Semantic LiDAR", "blueprint": BLUEPRINT_IDS[SENSOR_SEMANTIC],
                              "attributes": for_type(SENSOR_SEMANTIC), "point_fields": ["tag", "object_idx", "cos_inc"]},
            SENSOR_RAYCAST: {"label": "LiDAR (intensity)", "blueprint": BLUEPRINT_IDS[SENSOR_RAYCAST],
                             "attributes": for_type(SENSOR_RAYCAST), "point_fields": ["intensity"]},
        },
        "semantic_tags": [{"id": i, "name": n, "color": list(c)} for i, n, c in SEMANTIC_TAGS],
        "stream_point_buckets": list(POINT_BUCKETS) + ["all"],
        "limits": limits or {},
    }
