"""Operations behind the ``/api/*`` routes (config patching, RSU control, map geometry, DENM wrappers, run helpers).

A mixin of ``PersistentCARLAServer``; methods named ``*_published``/``cached_*`` are safe from any thread, everything else
runs on the simulation thread (the routes use ``on_sim_thread``).
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

from src.recording.recorder import RunAlreadyRecording, list_runs
from src.server.map_geometry import build_map_geometry
from src.server.route_utils import ConflictError
from src.server.world_config import HOT_FIELDS, WorldConfig
from src.v2x.frame import HAZARD_STATION_BASE

_RUN_ID = re.compile(r"^[A-Za-z0-9_.-]{1,128}$")
_RECORD_KINDS = {"links", "stations", "messages", "lidar_meta", "raw_lidar"}
DENM_DEFAULT_DURATION_S = 86400.0   # "until cancelled" = one simulated day


class RestartRequiredError(PermissionError):
    """A configuration field that cannot change while the world runs. Its own class so that an OS-level
    ``PermissionError`` (e.g. a ``runs/`` directory owned by root) is not reported as "restart required"."""


class ApiOpsMixin:
    # -- configuration -----------------------------------------------------------------------------
    def apply_config(self, changes: Dict[str, Any]) -> dict:
        """Change hot fields of the running world, all or nothing. Raises ``RestartRequiredError`` for a field that needs a restart,
        ``ValueError`` for an unknown field or a value the configuration rules reject."""
        self._require_sim_thread()
        if not isinstance(changes, dict) or not changes:
            raise ValueError("body must be a non-empty object of field: value")
        unknown = [k for k in changes if k not in WorldConfig.model_fields]
        if unknown:
            raise ValueError(f"unknown configuration field(s): {', '.join(sorted(unknown))}")
        cold = sorted(k for k in changes if k not in HOT_FIELDS)
        if cold:
            raise RestartRequiredError(f"{', '.join(cold)} cannot change while the world runs (restart required)")
        old = self.config
        new = WorldConfig.model_validate({**old.model_dump(), **changes})   # runs every cross-field rule
        self.config = new
        if "awareness_range_m" in changes:
            self.v2x_range = new.awareness_range_m
            if self.v2x_network is not None:
                self.v2x_network.max_range = new.awareness_range_m
        if any(k.startswith("advisory_") for k in changes):
            self._advisory.config = new.advisory
        if self.lidar is not None:
            self.lidar.limits = new.lidar
        applied = {k: {"old": getattr(old, k), "new": getattr(new, k)} for k in changes}
        self._record_event("config_changed", changes=applied)
        self._event("config_changed", changes=applied)
        return {"applied": applied, "frame": self._frame,
                "effect": {k: HOT_FIELDS[k] for k in changes}}

    @property
    def observer_public(self) -> bool:
        return self.config.observer_public

    @property
    def client_routes_require_token(self) -> bool:
        return self.config.client_routes_require_token

    # -- RSUs --------------------------------------------------------------------------------------
    def set_rsu(self, station_id: int, enabled: Optional[bool] = None, tx_power_dbm: Optional[float] = None) -> dict:
        """``station_id`` is the negative id shown in the world state (``-intersection_id``)."""
        self._require_sim_thread()
        net = self.v2x_network
        if net is None:
            raise ConflictError("V2X is disabled")
        iid = -int(station_id)
        rsu = net.get_rsu(iid) or net.get_disabled_rsus().get(iid)
        if iid <= 0 or rsu is None:
            raise LookupError(f"no RSU with station_id {station_id}")
        if tx_power_dbm is not None:
            rsu.config.tx_power_dbm = float(tx_power_dbm)
        if enabled is True:
            net.enable_rsu(iid)
        elif enabled is False:
            net.disable_rsu(iid)
        info = {"station_id": -iid, "enabled": net.get_rsu(iid) is not None, "tx_power_dbm": rsu.config.tx_power_dbm}
        self._record_event("rsu_changed", **info)
        self._event("rsu_changed", **info)
        return info

    # -- map ---------------------------------------------------------------------------------------
    def cached_map_geometry(self) -> Optional[dict]:
        return getattr(self, "_map_geometry", None)

    def get_map_geometry(self) -> dict:
        self._require_sim_thread()
        if getattr(self, "_map_geometry", None) is None:
            self._map_geometry = build_map_geometry(self.world.get_map())
        return self._map_geometry

    # -- DENM (the admin acts as a fixed roadside transmitter; station ids as in the V2X frame) ------
    @staticmethod
    def _denm_row(info: dict) -> dict:
        return {**info, "station_id": -(info["station_id"] + 1), "action_id": 0}

    def inject_denm(self, x: float, y: float, z: float, cause_code: int, subcause_code: int, relevance_distance_m: float,
                    information_quality: int, station_id: Optional[int], duration_s: Optional[float]) -> dict:
        if station_id is not None:
            raise ValueError("only fixed roadside transmitters are supported: send station_id null "
                             "(DENMs originating from a vehicle or an RSU are not modelled)")
        info = self.publish_hazard(x, y, z, cause_code, DENM_DEFAULT_DURATION_S if duration_s is None else duration_s,
                                   relevance_distance_m)
        return {**self._denm_row(info), "subcause_code_ignored": subcause_code, "information_quality_ignored": information_quality}

    def list_denm(self) -> List[dict]:
        return [self._denm_row(h) for h in self.get_hazards()]

    def cancel_denm(self, station_id: int, action_id: int) -> bool:
        raw = -int(station_id) - 1
        if raw < HAZARD_STATION_BASE or action_id != 0:
            return False
        return self.clear_hazard(raw)

    # -- runs --------------------------------------------------------------------------------------
    def start_run_api(self, name: str, record: List[str], config: Optional[Dict[str, Any]]) -> dict:
        bad = sorted(set(record) - _RECORD_KINDS)
        if bad:
            raise ValueError(f"unknown record kind(s): {', '.join(bad)}; allowed: {', '.join(sorted(_RECORD_KINDS))}")
        if self.recorder is not None:       # refuse before the config is touched
            raise RunAlreadyRecording(f"a run is already being recorded: {self.recorder.run_id}")
        if config:
            self.apply_config(config)
        summary = self.start_run(name, record_lidar="raw_lidar" in record, record_links="links" in record)
        self._event("run_started", run_id=summary.get("run_id"), name=name)
        return summary

    def run_status(self) -> Optional[dict]:
        rec = self.recorder
        return rec.status() if rec is not None else None

    def list_runs(self) -> List[dict]:
        return list_runs(Path(self.config.runs_dir))

    def run_manifest(self, run_id: str) -> dict:
        if not _RUN_ID.match(run_id) or run_id in (".", ".."):
            raise LookupError(f"no such run: {run_id!r}")
        path = Path(self.config.runs_dir) / run_id / "manifest.json"
        try:
            return json.loads(path.read_text())
        except (OSError, ValueError):
            raise LookupError(f"no such run: {run_id!r}")
