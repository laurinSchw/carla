"""Validated LiDAR sensor configuration, resource limits and the attribute schema (handoff §5.5)."""

from __future__ import annotations

from typing import Dict, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

SemanticType = "semantic"  # sensor.lidar.ray_cast_semantic
RaycastType = "raycast"    # sensor.lidar.ray_cast

BLUEPRINT_IDS = {
    "semantic": "sensor.lidar.ray_cast_semantic",
    "raycast": "sensor.lidar.ray_cast",
}

#: attributes that exist only on sensor.lidar.ray_cast (not on the semantic sensor); names as in the CARLA 0.9.x
#: blueprint library — to be verified against 0.9.16 on a live server
RAYCAST_ONLY = ("noise_stddev", "dropoff_general_rate", "dropoff_intensity_limit",
                "dropoff_zero_intensity", "atmosphere_attenuation_rate")


class Mount(BaseModel):
    """Sensor pose relative to the vehicle (CARLA frame: x forward, y right, z up; degrees)."""
    model_config = ConfigDict(extra="forbid", frozen=True)
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = Field(default=2.4, ge=0.0, le=5.0)
    pitch_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    yaw_deg: float = Field(default=0.0, ge=-180.0, le=180.0)
    roll_deg: float = Field(default=0.0, ge=-180.0, le=180.0)


class LidarConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    sensor_type: Literal["semantic", "raycast"] = "semantic"
    channels: int = Field(default=32, ge=1, le=128)
    range_m: float = Field(default=100.0, gt=0.0, le=200.0)
    points_per_second: int = Field(default=500_000, ge=1_000, le=5_000_000)
    #: None = one full sweep per simulation step, i.e. 1 / fixed_delta_seconds (resolved by ``resolve``)
    rotation_frequency_hz: Optional[float] = Field(default=None, gt=0.0, le=100.0)
    upper_fov_deg: float = Field(default=15.0, ge=-90.0, le=90.0)
    lower_fov_deg: float = Field(default=-30.0, ge=-90.0, le=90.0)
    horizontal_fov_deg: float = Field(default=360.0, gt=0.0, le=360.0)
    mount: Mount = Field(default_factory=Mount)

    # sensor.lidar.ray_cast only
    noise_stddev: Optional[float] = Field(default=None, ge=0.0)
    dropoff_general_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_intensity_limit: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_zero_intensity: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    atmosphere_attenuation_rate: Optional[float] = Field(default=None, ge=0.0)

    @model_validator(mode="after")
    def _rules(self) -> "LidarConfig":
        if self.lower_fov_deg >= self.upper_fov_deg:
            raise ValueError("lower_fov_deg must be below upper_fov_deg")
        if self.sensor_type == "semantic":
            used = [n for n in RAYCAST_ONLY if getattr(self, n) is not None]
            if used:
                raise ValueError(f"{used} exist only on sensor.lidar.ray_cast, not on the semantic sensor")
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
        if self.sensor_type == "raycast":
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


#: stream profile quantisation (display only; recorded data is never subsampled)
POINT_BUCKETS = (10_000, 25_000, 50_000)


def lidar_schema() -> dict:
    """JSON-friendly description of the valid settings per sensor type, for UIs (``GET /api/lidar/schema``)."""
    common = LidarConfig.model_json_schema()["properties"]
    return {
        "sensor_types": {
            "semantic": {
                "blueprint": BLUEPRINT_IDS["semantic"],
                "output": ["x", "y", "z", "cos_inc", "object_idx", "tag"],
                "attributes": [k for k in common if k not in RAYCAST_ONLY and k != "sensor_type"],
            },
            "raycast": {
                "blueprint": BLUEPRINT_IDS["raycast"],
                "output": ["x", "y", "z", "intensity"],
                "attributes": [k for k in common if k != "sensor_type"],
            },
        },
        "properties": common,
        "stream_point_buckets": list(POINT_BUCKETS) + ["all"],
        "note": "Attribute names and valid ranges follow CARLA 0.9.x and are unverified against 0.9.16.",
    }
