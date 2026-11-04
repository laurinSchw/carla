"""Headless, seeded runs: ``python -m src.cli run scenario.yaml [--fake]``.

A scenario fixes everything this server controls that can change a result: the seed, the configuration, the
population, the commands and the tick at which each command executes. The world is started paused; spawning and the
start of the recording happen on the simulation thread before the first tick; then the world is released and each
command runs just before the tick with its index (``PersistentCARLAServer.at_tick``), so it happens at the same frame
in every run. ``realtime`` is off: the simulation ticks as fast as it can.

What this does NOT fix: CARLA's own physics / Traffic Manager determinism (live, two same-seed runs were measured
within ~cm for most vehicles but one vehicle 3 m off: AGENTS.md section 6a). On the fake world (``fake: true``) the
whole chain is deterministic and the data files of two runs are compared byte for byte by tests/e2e/test_scenarios.py.

    name: crossing
    seed: 7
    fake: true                       # or --fake: the deterministic synthetic world instead of CARLA
    duration_frames: 400             # ticks after the release
    config: {awareness_range_m: 75}  # any WorldConfig field (realtime is forced off)
    spawn: {vehicles: 12, pedestrians: 4}
    record: [links, raw_lidar]       # links on/off and raw LiDAR; stations/messages/events are always written
    commands:
      - {frame: 100, action: inject_denm, x: 100, y: 100, cause_code: ACCIDENT, relevance_distance_m: 200, duration_s: 5}
      - {frame: 150, action: lidar_request, vehicle: first, record_raw: true}
      - {frame: 300, action: set_config, changes: {awareness_range_m: 50}}
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field, field_validator

logger = logging.getLogger(__name__)

ACTIONS = ("inject_denm", "cancel_denm", "lidar_request", "lidar_release", "set_config", "rsu", "spawn_traffic",
           "clear_traffic", "spawn_pedestrians", "clear_pedestrians")
#: what ``record`` may contain; stations, messages and events are always written by the recorder
RECORD_KINDS = ("links", "raw_lidar", "stations", "messages", "lidar_meta")


class Spawn(BaseModel):
    model_config = ConfigDict(extra="forbid")
    vehicles: int = Field(default=0, ge=0, le=300)
    pedestrians: int = Field(default=0, ge=0, le=200)


class Command(BaseModel):
    model_config = ConfigDict(extra="allow")
    frame: int = Field(ge=0, description="Executes just before tick number ``frame`` after the release (0 = before the first)")
    action: Literal[ACTIONS]  # type: ignore[valid-type]


class Scenario(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(min_length=1, max_length=64)
    seed: int = 42
    fake: bool = False
    host: str = "localhost"
    port: int = 2000
    duration_frames: int = Field(ge=1)
    config: Dict[str, Any] = Field(default_factory=dict)
    spawn: Spawn = Field(default_factory=Spawn)
    record: List[str] = Field(default_factory=lambda: ["links"])
    commands: List[Command] = Field(default_factory=list)

    @field_validator("record")
    @classmethod
    def _known_kinds(cls, v: List[str]) -> List[str]:
        bad = sorted(set(v) - set(RECORD_KINDS))
        if bad:
            raise ValueError(f"unknown record kind(s) {bad}; allowed: {list(RECORD_KINDS)}")
        return v

    @field_validator("config")
    @classmethod
    def _no_reserved(cls, v: Dict[str, Any]) -> Dict[str, Any]:
        reserved = sorted(set(v) & {"host", "port", "seed", "realtime", "runs_dir"})
        if reserved:
            raise ValueError(f"set {reserved} at the top level / on the command line, not in config")
        return v


def load_scenario(path: Union[str, Path]) -> Scenario:
    """YAML (``.yaml``/``.yml``, needs PyYAML, which uvicorn[standard] brings) or JSON."""
    p = Path(path)
    text = p.read_text()
    if p.suffix.lower() in (".yaml", ".yml"):
        import yaml
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: a scenario is a mapping")
    return Scenario.model_validate(data)


def _pick_vehicle(world: Any, which: Any) -> int:
    """``first`` / None, an index into the background vehicles sorted by actor id, or ``{"actor_id": N}``."""
    ids = sorted(a.id for a in world._background_vehicles)
    if isinstance(which, dict) and "actor_id" in which:
        return int(which["actor_id"])
    if which in (None, "first"):
        if not ids:
            raise LookupError("no background vehicle to attach to")
        return ids[0]
    if isinstance(which, int) and 0 <= which < len(ids):
        return ids[which]
    raise LookupError(f"cannot pick vehicle {which!r} (have {len(ids)} background vehicles)")


def _execute(world: Any, cmd: Command) -> Any:
    """One command, on the simulation thread."""
    from src.v2x.safety_messages import DENMCauseCode
    a: Dict[str, Any] = cmd.model_extra or {}
    act = cmd.action
    if act == "inject_denm":
        cause = a.get("cause_code", "ACCIDENT")
        cause = int(DENMCauseCode[cause.upper()] if isinstance(cause, str) else DENMCauseCode(int(cause)))
        return world.inject_denm(float(a["x"]), float(a["y"]), float(a.get("z", 0.0)), cause, int(a.get("subcause_code", 0)),
                                 float(a.get("relevance_distance_m", 150.0)), int(a.get("information_quality", 7)),
                                 a.get("station_id"), a.get("duration_s"))
    if act == "cancel_denm":
        return world.cancel_denm(int(a["station_id"]), int(a.get("action_id", 0)))
    if act == "lidar_request":
        from src.lidar.config import LidarConfig
        from src.server.roles import ADMIN
        cfg = LidarConfig.model_validate(a.get("config", {}))
        # pinned by default: an unwatched sensor would otherwise be released by the WALL-clock keep-alive, and the
        # run would depend on wall time
        return world.lidar_request(ADMIN, _pick_vehicle(world, a.get("vehicle")), cfg,
                                   record_raw=bool(a.get("record_raw", False)), pinned=bool(a.get("pinned", True)))
    if act == "lidar_release":
        from src.server.roles import ADMIN
        return world.lidar_release(ADMIN, int(a["sensor"]))
    if act == "set_config":
        return world.apply_config(dict(a["changes"]))
    if act == "rsu":
        return world.set_rsu(int(a["station_id"]), a.get("enabled"), a.get("tx_power_dbm"))
    if act == "spawn_traffic":
        return world.spawn_background_traffic(int(a["count"]))
    if act == "clear_traffic":
        return world.despawn_background_traffic()
    if act == "spawn_pedestrians":
        return world.spawn_pedestrians(int(a["count"]))
    if act == "clear_pedestrians":
        return world.despawn_pedestrians()
    raise ValueError(f"unknown action {act!r}")


def run_scenario(sc: Scenario, runs_dir: Union[str, Path] = "runs", fake: Optional[bool] = None,
                 timeout_s: float = 3600.0) -> Dict[str, Any]:
    """Run ``sc`` to completion and return its final manifest (plus ``_dir``). ``fake`` overrides the scenario's flag.

    A failing command does not abort the run: it is logged, recorded as a ``scheduled_command_failed`` event and
    listed under ``_failed_commands`` of the result (the CLI then exits non-zero).
    """
    use_fake = sc.fake if fake is None else fake
    if use_fake:
        from src.testing.fake_carla import install
        install(sc.seed)
    from src.server.persistent_world import PersistentCARLAServer
    from src.server.world_config import WorldConfig

    cfg = WorldConfig(host="fake" if use_fake else sc.host, port=sc.port, seed=sc.seed, realtime=False,
                      runs_dir=str(runs_dir), **sc.config)
    world = PersistentCARLAServer(cfg)
    world.start(paused=True)
    done = threading.Event()
    result: Dict[str, Any] = {}
    failed: List[Dict[str, Any]] = []
    try:
        def command(c: Command) -> None:
            try:
                _execute(world, c)
            except Exception as exc:
                failed.append({"frame": c.frame, "action": c.action, "error": repr(exc)})
                raise                                   # at_tick logs it and records the event

        def finish() -> None:
            result.update(world.stop_run() or {})
            done.set()

        def setup() -> Dict[str, Any]:
            if sc.spawn.vehicles:
                world.spawn_background_traffic(sc.spawn.vehicles)
            if sc.spawn.pedestrians:
                world.spawn_pedestrians(sc.spawn.pedestrians)
            info = world.start_run(sc.name, record_lidar="raw_lidar" in sc.record, record_links="links" in sc.record,
                                   extra_manifest={"scenario": sc.model_dump(mode="json")})
            base = world.tick_index                     # command frames count from the release
            for cmd in sc.commands:
                world.at_tick(base + cmd.frame, lambda c=cmd: command(c))
            world.at_tick(base + sc.duration_frames, finish)
            return info

        info = world.submit(setup).result(timeout=600)
        t0 = time.monotonic()
        world.resume()
        if not done.wait(timeout_s):
            raise TimeoutError(f"scenario did not finish within {timeout_s:.0f} s")
        logger.info("scenario %s finished in %.1f s", sc.name, time.monotonic() - t0)
        manifest = dict(result) if result else world.run_manifest(info["run_id"])
        manifest["_dir"] = str(Path(runs_dir) / info["run_id"])
        manifest["_failed_commands"] = failed
        manifest["_wall_s"] = round(time.monotonic() - t0, 3)
        return manifest
    finally:
        world.stop()
