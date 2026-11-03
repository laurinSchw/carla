"""Validated configuration for the persistent CARLA world (handoff section 5.2).

One object carries every simulation setting; cross-field rules fail fast at construction
instead of surfacing as odd CARLA behaviour later. Defaults reproduce the behaviour before
this class existed (Phase 2 is a refactor, not a model change).
"""

from pydantic import BaseModel, ConfigDict, Field, model_validator


class WorldConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    host: str
    port: int = Field(default=2000, ge=1, le=65535)
    timeout_s: float = Field(default=30.0, gt=0)

    # simulation clock
    fixed_delta_seconds: float = Field(default=0.05, gt=0, le=0.1)
    max_substep_delta_time: float = Field(default=0.01, gt=0)
    max_substeps: int = Field(default=10, ge=1, le=16)

    # Traffic Manager. NOTE: -20.0 means 20 % FASTER than the speed limit (CARLA semantics),
    # while src/config.py says 30.0 "% slower". Current behaviour (-20.0) is kept; the
    # contradiction is recorded in docs/science/CHANGELOG.md.
    tm_port: int = Field(default=8001, ge=1024, le=65535)
    seed: int = 42
    global_speed_pct: float = -20.0
    safety_distance_m: float = Field(default=2.5, ge=0)

    # V2X
    v2x_enabled: bool = True
    v2x_range_m: float = Field(default=75.0, ge=10.0, le=500.0)
    v2x_update_rate_hz: float = Field(default=2.0, gt=0)

    # web server port, only used to reject a clashing tm_port
    web_port: int = 8000

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
        return self

    @property
    def v2x_every_n_frames(self) -> int:
        return round(1.0 / (self.v2x_update_rate_hz * self.fixed_delta_seconds))
