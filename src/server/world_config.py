"""Validated configuration for the persistent CARLA world (handoff section 5.2).

One object carries every simulation setting; cross-field rules fail fast at construction instead of surfacing as odd
CARLA behaviour later. The model is FLAT on purpose: the web UI builds its configuration forms from this schema
(``GET /api/config/schema``) and only understands scalar fields. Grouped views (``lidar``, ``driver_mix``,
``advisory``) are properties, and the old object-style keyword arguments are still accepted at construction.
"""

from typing import Any, Dict, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

from src.lidar.config import LidarLimits
from src.traffic.advisory import AdvisoryConfig
from src.traffic.drivers import DriverMix


class WorldConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    host: str
    port: int = Field(default=2000, ge=1, le=65535)
    timeout_s: float = Field(default=30.0, gt=0)

    # simulation clock
    fixed_delta_seconds: float = Field(default=0.05, gt=0, le=0.1)
    max_substep_delta_time: float = Field(default=0.01, gt=0)
    max_substeps: int = Field(default=10, ge=1, le=16)

    # Traffic Manager. NOTE: -20.0 means 20 % FASTER than the speed limit (CARLA semantics). It applies to vehicles
    # WITHOUT a driver profile (client vehicles, or all vehicles with drivers_enabled=False); background vehicles with a
    # profile use that profile's own speed offset (src/traffic/drivers.py).
    tm_port: int = Field(default=8001, ge=1024, le=65535)
    seed: int = 42
    global_speed_pct: float = -20.0
    safety_distance_m: float = Field(default=2.5, ge=0)

    # V2X
    v2x_enabled: bool = True
    # Application-layer awareness filter (metres) applied after the radio model; None = the radio alone decides (S13)
    awareness_range_m: Optional[float] = Field(default=75.0, ge=10.0, le=500.0)
    v2x_update_rate_hz: float = Field(default=2.0, gt=0)
    # Number of intersection RSUs to create (None = one per intersection, all of them)
    rsu_max_count: Optional[int] = Field(default=None, ge=0)
    # Share of BACKGROUND vehicles that are V2X stations (seeded draw at spawn). Client vehicles are always stations.
    v2x_penetration_rate: float = Field(default=1.0, ge=0.0, le=1.0)

    # Background-traffic drivers: seeded profiles on top of the Traffic Manager, and the optional V2X advisory
    drivers_enabled: bool = True
    driver_mix_cautious: float = Field(default=0.2, ge=0.0, allow_inf_nan=False)
    driver_mix_normal: float = Field(default=0.6, ge=0.0, allow_inf_nan=False)
    driver_mix_aggressive: float = Field(default=0.2, ge=0.0, allow_inf_nan=False)
    advisory_enabled: bool = False
    advisory_trigger_level: int = Field(default=3, ge=1, le=4)
    advisory_extra_slowdown_pct: float = Field(default=40.0, ge=0.0, le=100.0)
    advisory_hazard_slowdown_pct: float = Field(default=60.0, ge=0.0, le=100.0)
    advisory_hold_s: float = Field(default=3.0, ge=0.0)

    # LiDAR budget (REFACTOR_HANDOFF 5.5): a sensor is a costly resource that exists only on request
    lidar_max_sensors: int = Field(default=4, ge=0, le=64)
    lidar_max_points_per_second: int = Field(default=2_000_000, ge=0)
    # a sensor without viewers is destroyed after this many WALL seconds (a UI resource policy, not model data)
    lidar_keep_alive_s: float = Field(default=10.0, ge=0.0)
    lidar_frame_timeout_s: float = Field(default=0.25, gt=0)       # how long a tick waits for a sensor's measurement
    lidar_max_consecutive_misses: int = Field(default=40, ge=1)    # then the sensor is declared dead and freed

    # A session whose WebSocket closed is kept this many seconds before its vehicle is removed, so a flaky connection
    # does not destroy a run (decision D4). 0 = remove at once.
    session_disconnect_grace_s: float = Field(default=30.0, ge=0.0)
    # Observers (no token) may use the read-only pages and topics (decision D12)
    observer_public: bool = True
    # The old /client/{leave,vehicle,lidar}/{session_id} routes and /ws/client/{id} accept the session id alone (False) or also
    # need the session's X-Session-Token / the admin token (True). Off by default so existing scripts keep working.
    client_routes_require_token: bool = False
    # Rate at which ground-truth station states are published (whole number of frames, like the V2X rate)
    world_state_rate_hz: float = Field(default=10.0, gt=0)
    # Row cap of one v2x.links message; more rows are cut and the message says truncated
    v2x_links_max_rows: int = Field(default=2000, ge=1)
    event_log_size: int = Field(default=1000, ge=10)

    # Map to load on connect, e.g. "Town03" (any entry of the CARLA server's available maps; the name may be given without the
    # path). None = use whatever map the CARLA server has loaded. Changing it needs a new world: POST /admin/map.
    map_name: Optional[str] = None

    # Pace the simulation to wall-clock time (True) or tick as fast as possible (False: headless runs)
    realtime: bool = True
    # Where runs are recorded (one directory per run)
    runs_dir: str = "runs"

    # web server port, only used to reject a clashing tm_port
    web_port: int = 8000

    @model_validator(mode="before")
    @classmethod
    def _accept_grouped_arguments(cls, data: Any) -> Any:
        """Old call sites pass objects (``lidar=LidarLimits(..)``, ``advisory=AdvisoryConfig(..)``, ``driver_mix=DriverMix(..)``,
        ``v2x_range_m=..``); expand them into the flat fields."""
        if not isinstance(data, dict):
            return data
        data = dict(data)
        lidar = data.pop("lidar", None)
        if lidar is not None:
            for field, value in (("lidar_max_sensors", lidar.max_sensors),
                                 ("lidar_max_points_per_second", lidar.max_total_points_per_second),
                                 ("lidar_keep_alive_s", lidar.keep_alive_s), ("lidar_frame_timeout_s", lidar.frame_timeout_s),
                                 ("lidar_max_consecutive_misses", lidar.max_consecutive_misses)):
                data.setdefault(field, value)
        mix = data.pop("driver_mix", None)
        if mix is not None:
            data.setdefault("driver_mix_cautious", mix.cautious)
            data.setdefault("driver_mix_normal", mix.normal)
            data.setdefault("driver_mix_aggressive", mix.aggressive)
        adv = data.pop("advisory", None)
        if adv is not None:
            for field, value in (("advisory_enabled", adv.enabled), ("advisory_trigger_level", adv.trigger_level),
                                 ("advisory_extra_slowdown_pct", adv.extra_slowdown_pct),
                                 ("advisory_hazard_slowdown_pct", adv.hazard_slowdown_pct), ("advisory_hold_s", adv.hold_s)):
                data.setdefault(field, value)
        if "v2x_range_m" in data:
            data.setdefault("awareness_range_m", data.pop("v2x_range_m"))
        return data

    @model_validator(mode="after")
    def _cross_field_rules(self) -> "WorldConfig":
        # CARLA substepping rule: dt <= max_substep_delta_time * max_substeps
        if self.fixed_delta_seconds > self.max_substep_delta_time * self.max_substeps + 1e-12:
            raise ValueError(
                f"fixed_delta_seconds ({self.fixed_delta_seconds}) must be <= "
                f"max_substep_delta_time * max_substeps "
                f"({self.max_substep_delta_time * self.max_substeps})"
            )
        if self.tm_port in (self.port, self.port + 1, self.web_port):
            raise ValueError(
                f"tm_port {self.tm_port} clashes with the CARLA RPC/streaming ports "
                f"({self.port}/{self.port + 1}) or the web server port ({self.web_port})"
            )
        # one V2X update every N frames; N must be a whole number of frames
        n = 1.0 / (self.v2x_update_rate_hz * self.fixed_delta_seconds)
        if abs(n - round(n)) > 1e-6 or round(n) < 1:
            raise ValueError(
                f"1 / (v2x_update_rate_hz * fixed_delta_seconds) = {n:.4f} is not a whole "
                f"number of frames; choose a rate that divides the simulation rate"
            )
        m = 1.0 / (self.world_state_rate_hz * self.fixed_delta_seconds)
        if abs(m - round(m)) > 1e-6 or round(m) < 1:
            raise ValueError(
                f"1 / (world_state_rate_hz * fixed_delta_seconds) = {m:.4f} is not a whole number of frames")
        DriverMix(self.driver_mix_cautious, self.driver_mix_normal, self.driver_mix_aggressive)  # finite, >= 0, not all 0
        return self

    # ── grouped views for the code that uses them ──────────────────────────────────
    @property
    def driver_mix(self) -> DriverMix:
        return DriverMix(self.driver_mix_cautious, self.driver_mix_normal, self.driver_mix_aggressive)

    @property
    def advisory(self) -> AdvisoryConfig:
        return AdvisoryConfig(enabled=self.advisory_enabled, trigger_level=self.advisory_trigger_level,
                              extra_slowdown_pct=self.advisory_extra_slowdown_pct,
                              hazard_slowdown_pct=self.advisory_hazard_slowdown_pct, hold_s=self.advisory_hold_s)

    @property
    def lidar(self) -> LidarLimits:
        return LidarLimits(max_sensors=self.lidar_max_sensors, max_total_points_per_second=self.lidar_max_points_per_second,
                           keep_alive_s=self.lidar_keep_alive_s, frame_timeout_s=self.lidar_frame_timeout_s,
                           max_consecutive_misses=self.lidar_max_consecutive_misses)

    @property
    def v2x_range_m(self) -> Optional[float]:
        """Old name of ``awareness_range_m``."""
        return self.awareness_range_m

    @property
    def world_state_every_n_frames(self) -> int:
        return round(1.0 / (self.world_state_rate_hz * self.fixed_delta_seconds))

    @property
    def v2x_every_n_frames(self) -> int:
        return round(1.0 / (self.v2x_update_rate_hz * self.fixed_delta_seconds))


# Fields that can change while the world runs (PATCH /api/config); everything else needs a new world (restart).
# "next V2X update" fields take effect when the V2X network runs next, "immediately" fields from the next frame.
HOT_FIELDS: Dict[str, str] = {
    "awareness_range_m": "next V2X update",
    "v2x_penetration_rate": "for background vehicles spawned from now on",
    "advisory_enabled": "next V2X update",
    "advisory_trigger_level": "next V2X update",
    "advisory_extra_slowdown_pct": "next V2X update",
    "advisory_hazard_slowdown_pct": "next V2X update",
    "advisory_hold_s": "next V2X update",
    "session_disconnect_grace_s": "immediately",
    "observer_public": "immediately",
    "client_routes_require_token": "immediately",
    "world_state_rate_hz": "immediately",
    "v2x_links_max_rows": "immediately",
    "lidar_max_sensors": "for new requests (running sensors keep running)",
    "lidar_max_points_per_second": "for new requests (running sensors keep running)",
    "lidar_keep_alive_s": "immediately",
}


def config_schema() -> dict:
    """JSON schema of WorldConfig with ``x-apply`` on every property: "hot" or "restart", plus ``x-effect``."""
    schema = WorldConfig.model_json_schema()
    for name, prop in schema.get("properties", {}).items():
        prop["x-apply"] = "hot" if name in HOT_FIELDS else "restart"
        if name in HOT_FIELDS:
            prop["x-effect"] = HOT_FIELDS[name]
    return schema
