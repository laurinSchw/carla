# TODO: CARLA handles / the module-global world are Optional until a connected-world type exists (WorldController state machine)
# pyright: reportOptionalMemberAccess=false
"""
Admin-only FastAPI router.

All endpoints require the ``X-Admin-Token`` header to match the value of
the ``ADMIN_TOKEN`` environment variable set at server startup.

Mount on the main app with::

    from src.server.admin_routes import router as admin_router
    app.include_router(admin_router)
"""

import os
import logging
import threading
from dataclasses import asdict
from typing import Optional

from fastapi import APIRouter, HTTPException, Header, Depends, Query
from pydantic import BaseModel, ConfigDict, Field, ValidationError, create_model

from src.recording.recorder import RunAlreadyRecording
from src.server.route_utils import ConflictError, on_sim_thread
from src.server.world_config import WorldConfig

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/admin", tags=["admin"])

# Injected once at startup by start_server.py
_world = None
_connect_lock = threading.Lock()
_connect_state = "idle"  # idle | connecting | running | error
_connect_error: Optional[str] = None
_server_module = None  # reference back to server.app for set_persistent_world propagation


def set_persistent_world(world) -> None:
    global _world
    _world = world


def set_server_module(mod) -> None:
    global _server_module
    _server_module = mod


# ------------------------------------------------------------------ #
# Auth dependency                                                     #
# ------------------------------------------------------------------ #

def check_admin_token(token: str) -> None:
    """Raise 403 if ``token`` is not ADMIN_TOKEN (500 if the server has none). Shared by the hub and API routes."""
    expected = os.environ.get("ADMIN_TOKEN", "")
    if not expected:
        raise HTTPException(status_code=500, detail="ADMIN_TOKEN is not configured on the server")
    if token != expected:
        raise HTTPException(status_code=403, detail="Invalid admin token")


def _require_admin(x_admin_token: str = Header(..., alias="X-Admin-Token")):
    """Raise 403 if the token does not match ADMIN_TOKEN env var."""
    expected = os.environ.get("ADMIN_TOKEN", "")
    if not expected:
        raise HTTPException(status_code=500, detail="ADMIN_TOKEN is not configured on the server")
    if x_admin_token != expected:
        raise HTTPException(status_code=403, detail="Invalid admin token")


# ------------------------------------------------------------------ #
# Request models                                                      #
# ------------------------------------------------------------------ #

class TrafficRequest(BaseModel):
    count: int = Field(default=20, ge=1, le=200)

class PedestrianRequest(BaseModel):
    count: int = Field(default=15, ge=1, le=100)

# Fields of WorldConfig that are not part of a connect request (set by the server itself)
_NOT_REQUESTABLE = ("host", "port", "web_port", "runs_dir")


def _connect_request_model():
    """ConnectRequest = carla_host + carla_port + EVERY other WorldConfig field, optional. Generated from WorldConfig, so the
    request, the OpenAPI document and the configuration forms can never drift apart; the values are validated once, by WorldConfig."""
    fields = {
        name: (Optional[info.annotation], Field(default=None, description=info.description))
        for name, info in WorldConfig.model_fields.items() if name not in _NOT_REQUESTABLE
    }
    return create_model(
        "ConnectRequest", __config__=ConfigDict(extra="forbid"),
        carla_host=(str, Field(default="192.168.1.110")), carla_port=(int, Field(default=2000, ge=1, le=65535)), **fields)


ConnectRequest = _connect_request_model()


# ------------------------------------------------------------------ #
# Endpoints                                                           #
# ------------------------------------------------------------------ #

@router.get("/status", summary="World status", dependencies=[Depends(_require_admin)])
async def admin_status():
    """Return world-level status — works even before connect (world=None)."""
    if _world is None:
        return {
            "running": False,
            "connected": False,
            "frame": 0,
            "elapsed_seconds": 0,
            "background_vehicles": 0,
            "pedestrians": 0,
            "active_sessions": 0,
            "client_vehicles": 0,
            "v2x_enabled": False,
            "rsu_count": 0,
            "vru_count": 0,
            "map": None,
            "connect_state": _connect_state,
            "connect_error": _connect_error,
        }
    return {**_world.get_status(), "connected": True,
            "connect_state": _connect_state, "connect_error": _connect_error}


@router.post("/connect", summary="Connect to CARLA and start world", dependencies=[Depends(_require_admin)])
async def connect_world(req: ConnectRequest):
    """
    Start (or restart) the persistent CARLA world.

    Call this from the admin panel to connect to a CARLA server at any IP.
    Safe to call again if the world is already running — stops the old one first.
    A second request while a connect is in progress is rejected with 409.
    """
    values = {k: v for k, v in req.model_dump(exclude_unset=True).items() if k not in ("carla_host", "carla_port")}
    try:
        config = WorldConfig(host=req.carla_host, port=req.carla_port, **values)
    except (ValidationError, TypeError, ValueError) as exc:
        # without the offending input: a NaN/Infinity cannot be serialised back as JSON
        detail = ([{k: v for k, v in e.items() if k != "input"} for e in exc.errors(include_url=False, include_context=False)]
                  if isinstance(exc, ValidationError) else str(exc))
        raise HTTPException(status_code=422, detail=detail)

    _begin_connect(config)
    return {
        "ok": True,
        "message": f"Connecting to CARLA at {req.carla_host}:{req.carla_port} (async)…",
    }


def _begin_connect(config: WorldConfig) -> None:
    """Stop the running world (if any) and start a new one with ``config`` on a background thread; 409 while one is connecting."""
    global _connect_state, _connect_error
    with _connect_lock:
        if _connect_state == "connecting":
            raise HTTPException(status_code=409, detail="A connect is already in progress")
        _connect_state, _connect_error = "connecting", None

    def _check_map_exists():
        """A wrong map name must not cost the running world: ask the CARLA server before anything is stopped."""
        if not config.map_name:
            return
        import carla
        from src.server.persistent_world import PersistentCARLAServer
        client = carla.Client(config.host, config.port)
        client.set_timeout(config.timeout_s)
        available = list(client.get_available_maps())
        if PersistentCARLAServer.match_map(config.map_name, available) is None:
            names = ", ".join(sorted(PersistentCARLAServer.short_map_name(m) for m in available))
            raise ValueError(f"map {config.map_name!r} is not available on the CARLA server; available: {names}")

    def _start():
        global _world, _connect_state, _connect_error
        from src.server.persistent_world import PersistentCARLAServer
        try:
            _check_map_exists()
            # Stop previous world if any
            if _world is not None:
                try:
                    _world.stop()
                except Exception as e:
                    logger.warning("Error stopping previous world: %s", e)

            world = PersistentCARLAServer(config)
            try:
                world.start()
            except BaseException:
                world.stop()  # release whatever start() got as far as creating
                raise
            _world = world

            # Propagate to client routes
            if _server_module is not None:
                _server_module.set_persistent_world(world)

            _connect_state = "running"
            logger.info("Persistent world started via admin /connect (%s:%s)", config.host, config.port)
        except Exception as exc:
            logger.error("Connect to CARLA %s:%s failed: %s", config.host, config.port, exc)
            _connect_error = f"{type(exc).__name__}: {exc}"
            _connect_state = "error"

    threading.Thread(target=_start, daemon=True, name="admin-connect").start()


class MapRequest(BaseModel):
    map_name: str = Field(min_length=1, max_length=128, description="e.g. Town03; see GET /admin/maps")


@router.get("/maps", summary="Maps the CARLA server can load", dependencies=[Depends(_require_admin)])
async def list_maps():
    _assert_ready()
    _assert_running()
    maps = await on_sim_thread(_world, _world.list_maps)
    return {"current": _world.get_status().get("map"), "maps": maps}


@router.post("/map", summary="Switch the CARLA map (restarts the world)", dependencies=[Depends(_require_admin)])
async def change_map(req: MapRequest):
    """Load another map and restart the world with the same configuration. ``load_world`` replaces the world on the CARLA
    server: actors of other users on it are destroyed too (the lab world is shared). Sessions, LiDAR sensors, spawned traffic and an
    open run end, exactly as with ``/connect``. 422 for a map the CARLA server does not have, 409 while a connect runs."""
    if _connect_state == "connecting":                      # before _assert_running: a stopping world is not "running"
        raise HTTPException(status_code=409, detail="A connect is already in progress")
    _assert_ready()
    _assert_running()
    available = await on_sim_thread(_world, _world.list_maps)
    target = _world.match_map(req.map_name, available)
    if target is None:
        raise HTTPException(status_code=422, detail=f"map {req.map_name!r} is not available; available: {', '.join(available)}")
    try:
        new_config = WorldConfig.model_validate({**_world.config.model_dump(), "map_name": _world.short_map_name(target)})
    except ValidationError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    _begin_connect(new_config)
    return {"ok": True, "message": f"Loading {target} and restarting the world (async)…"}


@router.get("/sessions", summary="All client sessions", dependencies=[Depends(_require_admin)])
async def list_sessions():
    """Return all sessions (active and ended). Returns [] when world is not started."""
    if _world is None:
        return []
    return [s.to_dict() for s in _world.sessions.all_sessions()]


@router.delete("/sessions/{session_id}", summary="Kick a client", dependencies=[Depends(_require_admin)])
async def kick_session(session_id: str):
    """Force-end a session and despawn the client vehicle."""
    if _world is None:
        raise HTTPException(status_code=404, detail="Session not found — world not started")
    session = _world.sessions.get_session(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Session not found")
    await on_sim_thread(_world, _world.despawn_client_vehicle, session_id)
    _world.sessions.end_session(session_id)
    return {"ok": True, "kicked": session_id}


@router.post("/traffic", summary="Spawn background traffic", dependencies=[Depends(_require_admin)])
async def spawn_traffic(req: TrafficRequest):
    """Spawn autopilot background vehicles."""
    _assert_ready()
    _assert_running()
    spawned = await on_sim_thread(_world, _world.spawn_background_traffic, req.count)
    return {"requested": req.count, "spawned": spawned}


@router.delete("/traffic", summary="Clear background traffic", dependencies=[Depends(_require_admin)])
async def clear_traffic():
    """Destroy all admin-spawned background vehicles. Returns 0 when world not started."""
    if _world is None:
        return {"removed": 0}
    removed = await on_sim_thread(_world, _world.despawn_background_traffic)
    return {"removed": removed}


@router.post("/pedestrians", summary="Spawn pedestrians", dependencies=[Depends(_require_admin)])
async def spawn_pedestrians(req: PedestrianRequest):
    """Spawn AI-controlled pedestrians."""
    _assert_ready()
    _assert_running()
    spawned = await on_sim_thread(_world, _world.spawn_pedestrians, req.count)
    return {"requested": req.count, "spawned": spawned}


@router.delete("/pedestrians", summary="Clear pedestrians", dependencies=[Depends(_require_admin)])
async def clear_pedestrians():
    """Destroy all admin-spawned pedestrians. Returns 0 when world not started."""
    if _world is None:
        return {"removed": 0}
    removed = await on_sim_thread(_world, _world.despawn_pedestrians)
    return {"removed": removed}


@router.post("/stop", summary="Stop the CARLA server", dependencies=[Depends(_require_admin)])
async def stop_world():
    """Gracefully stop the persistent world (tick loop + actor cleanup)."""
    if _world is None:
        return {"ok": False, "message": "World not running"}
    threading.Thread(target=_world.stop, daemon=True, name="admin-stop").start()
    return {"ok": True, "message": "Shutdown initiated"}


# ------------------------------------------------------------------ #
# V2X read-only endpoints                                             #
# ------------------------------------------------------------------ #

@router.get("/v2x/rsus", summary="List registered RSUs", dependencies=[Depends(_require_admin)])
async def list_rsus():
    """Return all RSUs registered in the V2X network."""
    if _world is None or getattr(_world, "v2x_network", None) is None:
        return []
    rsus = _world.v2x_network.get_rsus()
    return [
        {
            "intersection_id": rsu.config.intersection_id,
            "position": {
                "x": rsu.config.position[0],
                "y": rsu.config.position[1],
                "z": rsu.config.position[2],
            },
            "broadcast_range_m": rsu.config.broadcast_range_m,
            "tx_power_dbm": rsu.config.tx_power_dbm,
        }
        for rsu in rsus.values()
    ]


@router.get("/v2x/vrus", summary="List active VRUs", dependencies=[Depends(_require_admin)])
async def list_vrus():
    """Return all active VRU VRUAM messages from the V2X network."""
    if _world is None or getattr(_world, "v2x_network", None) is None:
        return []
    vru_messages = _world.v2x_network.get_all_vruam()
    return [
        {
            "station_id": vruam.station_id,
            "profile": vruam.vru_profile.name,
            "position": {
                "x": round(vruam.position[0], 2),
                "y": round(vruam.position[1], 2),
                "z": round(vruam.position[2], 2),
            },
            "speed": round(vruam.speed, 2),
            "heading": round(vruam.heading, 1),
            "motion_state": vruam.motion_state,
            "timestamp": vruam.timestamp,
        }
        for vruam in vru_messages.values()
    ]


@router.get("/v2x/spat", summary="Current SPAT messages", dependencies=[Depends(_require_admin)])
async def list_spat():
    """Return current Signal Phase and Timing (SPAT) messages from all RSUs."""
    if _world is None or getattr(_world, "v2x_network", None) is None:
        return []
    spat_messages = _world.v2x_network.get_all_spat()
    return [
        {
            "intersection_id": spat.intersection_id,
            "intersection_name": spat.intersection_name,
            "status": spat.status,
            "timestamp": round(spat.timestamp, 2),
            "states": [
                {
                    "signal_group_id": s.signal_group_id,
                    "movement_event": s.movement_event,
                    "likely_time_s": round(s.likely_time_s, 1) if s.likely_time_s is not None else None,
                    "min_end_time_s": round(s.min_end_time_s, 1) if s.min_end_time_s is not None else None,
                    "max_end_time_s": round(s.max_end_time_s, 1) if s.max_end_time_s is not None else None,
                }
                for s in spat.states
            ],
        }
        for spat in spat_messages.values()
    ]


class RunRequest(BaseModel):
    name: str = Field(default="run", min_length=1, max_length=64)
    record_lidar: bool = Field(default=False, description="Also write every LiDAR frame at full resolution (large).")
    record_links: bool = Field(default=True, description="Write one record per V2X transmission attempt.")


@router.post("/runs", status_code=201, summary="Start recording a run", dependencies=[Depends(_require_admin)])
async def start_run(req: RunRequest):
    """Write frame-stamped, full-resolution data to ``runs/<UTC>_<sha>_<name>/`` until stopped.

    Files: ``manifest.json`` (settings, seeds, versions), ``events.jsonl``, ``stations.jsonl`` (ground truth every V2X update),
    ``v2x_links.jsonl`` (channel decision per attempt), optional raw LiDAR.
    """
    _assert_ready()
    _assert_running()
    try:
        return await on_sim_thread(_world, _world.start_run, req.name, req.record_lidar, req.record_links)
    except RunAlreadyRecording as exc:
        raise HTTPException(status_code=409, detail=str(exc))


@router.post("/runs/current/stop", summary="Stop the current recording", dependencies=[Depends(_require_admin)])
async def stop_run():
    _assert_ready()
    manifest = await on_sim_thread(_world, _world.stop_run)
    if manifest is None:
        raise HTTPException(status_code=404, detail="no run is being recorded")
    return manifest


@router.get("/runs", summary="Current recording and finished runs", dependencies=[Depends(_require_admin)])
async def list_runs_endpoint():
    if _world is None:
        return {"current": None, "runs": []}
    return _world.get_run_status()


class HazardRequest(BaseModel):
    x: float = Field(allow_inf_nan=False, description="CARLA world x of the hazard (metres)")
    y: float = Field(allow_inf_nan=False, description="CARLA world y of the hazard (metres)")
    z: float = Field(default=0.0, allow_inf_nan=False)
    cause_code: int = Field(default=10, ge=0, le=255, description="ETSI cause code; 10 = obstacle on road, 2 = accident, 3 = road works")
    duration_s: float = Field(default=30.0, gt=0, le=3600, description="Simulation seconds the warning stays valid")
    relevance_distance_m: float = Field(default=150.0, gt=0, le=2000)
    tx_power_dbm: Optional[float] = Field(default=None, ge=0, le=46, description="Default 33 dBm (roadside unit)")


@router.post("/v2x/hazards", status_code=201, summary="Place a hazard warning (DENM)", dependencies=[Depends(_require_admin)])
async def publish_hazard(req: HazardRequest):
    """The warning is sent through the DSRC channel every V2X update; a vehicle only reacts if it RECEIVED it
    (see ``GET /admin/v2x/links?msg_type=DENM``). Needs V2X enabled."""
    _assert_ready()
    _assert_running()
    try:
        return await on_sim_thread(_world, _world.publish_hazard, req.x, req.y, req.z, req.cause_code, req.duration_s,
                                   req.relevance_distance_m, req.tx_power_dbm)
    except ValueError as exc:  # unknown cause code
        raise HTTPException(status_code=422, detail=str(exc))
    except ConflictError as exc:
        raise HTTPException(status_code=409, detail=str(exc))


@router.get("/v2x/hazards", summary="Active hazard warnings", dependencies=[Depends(_require_admin)])
async def list_hazards():
    return _world.get_hazards() if _world is not None else []


@router.delete("/v2x/hazards/{station_id}", summary="Cancel a hazard warning", dependencies=[Depends(_require_admin)])
async def clear_hazard(station_id: int):
    _assert_ready()
    if not await on_sim_thread(_world, _world.clear_hazard, station_id):
        raise HTTPException(status_code=404, detail="no such hazard")
    return {"cleared": station_id}


@router.get("/v2x/links", summary="Per-link channel results of the last V2X update",
            dependencies=[Depends(_require_admin)])
async def list_links(msg_type: Optional[str] = None, rx_id: Optional[int] = None,
                     delivered: Optional[bool] = None, limit: int = Query(default=200, ge=1, le=5000)):
    """One record per (transmitter, receiver, message) attempt, with the full channel decision.

    Filter by ``msg_type`` (BSM, MAP, SPAT, VRUAM), ``rx_id`` and ``delivered``; ``summary`` counts
    all matching records, ``links`` is truncated to ``limit``.
    """
    net = getattr(_world, "v2x_network", None) if _world is not None else None
    if net is None:
        return {"summary": {"total": 0}, "links": []}
    links = [
        l for l in net.get_link_results()  # list is replaced per update: a safe snapshot
        if (msg_type is None or l.msg_type == msg_type)
        and (rx_id is None or l.rx_id == rx_id)
        and (delivered is None or l.delivered == delivered)
    ]
    by_cause: dict = {}
    for l in links:
        by_cause[l.drop_cause or "delivered"] = by_cause.get(l.drop_cause or "delivered", 0) + 1
    return {
        "summary": {"total": len(links), "by_outcome": by_cause},
        "links": [asdict(l) for l in links[:limit]],
    }


# ------------------------------------------------------------------ #
# Internal helpers                                                    #
# ------------------------------------------------------------------ #

def _assert_ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="Persistent world not initialised")


def _assert_running():
    if not _world._running:
        raise HTTPException(status_code=503, detail="CARLA simulation is not running")
