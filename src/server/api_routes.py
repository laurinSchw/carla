# pyright: reportOptionalMemberAccess=false
"""Read-only status/config/map endpoints and the admin DENM injection (REFACTOR_HANDOFF 5.7.2, decision D11).

Public ones follow ``observer_public``: when it is false they need the admin token like everything else.
"""

import logging
import sys
from typing import Any, Dict, List, Optional, Union

from fastapi import APIRouter, Header, HTTPException, Request, Response
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, field_validator

from src.recording.recorder import DEFAULT_RECORD, RunAlreadyRecording
from src.server.admin_routes import check_admin_token
from src.server.api_ops import RestartRequiredError
from src.server.route_utils import on_sim_thread
from src.server.world_config import HOT_FIELDS, config_schema
from src.v2x.safety_messages import DENMCauseCode

logger = logging.getLogger(__name__)

router = APIRouter(tags=["api"])

_world = None


def set_persistent_world(world) -> None:
    global _world
    _world = world


def _read_gate(x_admin_token: Optional[str]) -> None:
    """Observer reads are allowed unless the server switched them off; the admin token always works."""
    if x_admin_token:
        check_admin_token(x_admin_token)
        return
    if _world is not None and not _world.observer_public:
        raise HTTPException(status_code=403, detail="observer access is disabled; send X-Admin-Token")


def _ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
    return _world


@router.get("/api/status", summary="Server status (public)")
async def get_status(x_admin_token: Optional[str] = Header(default=None)):
    """Is CARLA connected, which map, which frame; ``connected: false`` before /admin/connect."""
    if _world is None or not _world._running:
        return {"connected": False}
    _read_gate(x_admin_token)
    status = _world.get_status()
    run = _world.get_run_status_published()
    return {
        "connected": True, "map": status["map"], "frame": status["frame"],
        "sim_time": run.sim_time if run else None,
        "real_time_factor": run.data.get("real_time_factor") if run else None,
        "fixed_delta_seconds": _world.fixed_delta_seconds, "v2x_enabled": status["v2x_enabled"],
        "active_sessions": status["active_sessions"], "run_active": _world.run_status() is not None,
        "observer_public": _world.observer_public,
        "fake": bool(getattr(sys.modules.get("carla"), "IS_FAKE", False)),   # a synthetic world, not CARLA
    }


@router.get("/api/config", summary="The resolved configuration of the running world")
async def get_config(x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    return world.config.model_dump()


@router.get("/api/config/schema", summary="JSON schema of the configuration (for forms)")
async def get_config_schema(x_admin_token: Optional[str] = Header(default=None)):
    _read_gate(x_admin_token)
    return config_schema()


@router.patch("/api/config", summary="Change hot configuration fields of the running world (admin)")
async def patch_config(changes: Dict[str, Any], x_admin_token: str = Header(...)):
    """Body: ``{"field": value, ...}`` with fields marked ``x-apply: hot`` in /api/config/schema. All or nothing; every
    change becomes an event with the frame at which it was applied. Fields that need a restart answer 409."""
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return await on_sim_thread(world, lambda: world.apply_config(changes))
    except RestartRequiredError as exc:
        raise HTTPException(status_code=409, detail={"error": "restart_required", "message": str(exc),
                                                     "hot_fields": sorted(HOT_FIELDS)})
    except ValueError as exc:                                  # unknown field or a value the configuration rules reject
        errors = getattr(exc, "errors", None)                 # pydantic.ValidationError has .errors(); plain ValueError does not
        detail = errors(include_url=False, include_context=False) if callable(errors) else str(exc)
        raise HTTPException(status_code=422, detail=detail)


@router.get("/api/v2x/rsus", summary="RSUs: position, power, range, enabled")
async def list_rsus(x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    state = world.get_world_state()
    return {"rsus": state.data.get("rsus", []) if state else [], "frame": state.frame if state else None}


class RsuPatch(BaseModel):
    enabled: Optional[bool] = None
    tx_power_dbm: Optional[float] = Field(default=None, ge=0.0, le=40.0)


@router.patch("/api/v2x/rsus/{station_id}", summary="Enable/disable an RSU or change its transmit power (admin)")
async def patch_rsu(station_id: int, body: RsuPatch, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return await on_sim_thread(world, lambda: world.set_rsu(station_id, body.enabled, body.tx_power_dbm))
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))


@router.get("/api/map/geometry", summary="Road network: lane polylines and junction boxes")
async def get_map_geometry(request: Request, x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    geom = world.cached_map_geometry()
    if geom is None:
        geom = await on_sim_thread(world, world.get_map_geometry)
    etag = f'"{geom["version"]}"'
    if request.headers.get("if-none-match") == etag:
        return Response(status_code=304, headers={"ETag": etag})
    return JSONResponse(geom, headers={"ETag": etag, "Cache-Control": "no-cache"})


# ── DENM injection ────────────────────────────────────────────────────────────────────────────────

class DenmRequest(BaseModel):
    x: float = Field(description="CARLA world x of the event, metres")
    y: float = Field(description="CARLA world y of the event, metres")
    z: float = 0.0
    cause_code: Union[int, str] = Field(
        default="ACCIDENT", description="ETSI cause code number or name: " + ", ".join(c.name for c in DENMCauseCode))
    subcause_code: int = Field(default=0, ge=0, le=255)
    relevance_distance_m: float = Field(default=150.0, gt=0, le=2000)
    information_quality: int = Field(default=7, ge=0, le=7)
    station_id: Optional[int] = Field(default=None, description="Originating station (a V2X vehicle's actor id or "
                                      "-intersection_id of an RSU); null = a fixed roadside transmitter at the event")
    duration_s: Optional[float] = Field(default=None, gt=0, description="Simulation seconds until it is cancelled; "
                                        "null = until cancelled")

    @field_validator("cause_code")
    @classmethod
    def _cause(cls, v):
        try:
            return int(DENMCauseCode[v.upper()] if isinstance(v, str) else DENMCauseCode(v))
        except (KeyError, ValueError):
            raise ValueError(f"unknown DENM cause code {v!r}")


@router.post("/api/v2x/denm", status_code=201, summary="Inject a DENM at a map position (admin)")
async def inject_denm(req: DenmRequest, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return await on_sim_thread(world, lambda: world.inject_denm(
            req.x, req.y, req.z, req.cause_code, req.subcause_code, req.relevance_distance_m,
            req.information_quality, req.station_id, req.duration_s))
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))


@router.get("/api/v2x/denm", summary="DENMs in the last V2X update")
async def list_denm(x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    return {"denms": world.list_denm()}


@router.delete("/api/v2x/denm/{station_id}/{action_id}", summary="Cancel a DENM (admin)")
async def cancel_denm(station_id: int, action_id: int, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    if not await on_sim_thread(world, lambda: world.cancel_denm(station_id, action_id)):
        raise HTTPException(status_code=404, detail="no such DENM")
    return {"cancelled": True}


# ── recorded runs ─────────────────────────────────────────────────────────────────────────────────

class RunRequest(BaseModel):
    name: str = Field(min_length=1, max_length=64, description="Shown in the run id and the manifest")
    record: List[str] = Field(default_factory=lambda: list(DEFAULT_RECORD),
                              description="links, stations, messages, lidar_meta, raw_lidar")
    config: Optional[Dict[str, Any]] = Field(default=None, description="Hot configuration fields applied before the run starts")


@router.post("/api/runs", status_code=201, summary="Start recording a run (admin)")
async def start_run(req: RunRequest, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return await on_sim_thread(world, lambda: world.start_run_api(req.name, req.record, req.config))
    except RunAlreadyRecording as exc:
        raise HTTPException(status_code=409, detail={"error": "run_active", "message": str(exc)})
    except RestartRequiredError as exc:
        raise HTTPException(status_code=409, detail={"error": "restart_required", "message": str(exc)})
    except OSError as exc:                                     # e.g. runs/ not writable (owned by root after a container run)
        raise HTTPException(status_code=503, detail={"error": "storage", "message": f"cannot create the run directory: {exc}"})
    except ValueError as exc:
        errors = getattr(exc, "errors", None)
        raise HTTPException(status_code=422, detail=errors(include_url=False, include_context=False) if callable(errors) else str(exc))


@router.post("/api/runs/current/stop", summary="Stop the active run (admin)")
async def stop_run(x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        manifest = await on_sim_thread(world, world.stop_run)
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    if manifest is None:
        raise HTTPException(status_code=404, detail="no run is being recorded")
    return manifest


@router.get("/api/runs", summary="Recorded runs and the active one (admin)")
async def list_runs(x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    return {"active": world.run_status(), "runs": world.list_runs()}


@router.get("/api/runs/{run_id}/manifest", summary="The manifest of a run (admin)")
async def run_manifest(run_id: str, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return world.run_manifest(run_id)
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
