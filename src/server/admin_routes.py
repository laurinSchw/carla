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
from typing import Optional

from fastapi import APIRouter, HTTPException, Header, Depends
from pydantic import BaseModel, Field, ValidationError

from src.server.route_utils import on_sim_thread
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

class ConnectRequest(BaseModel):
    carla_host: str = Field(default="192.168.1.110")
    carla_port: int = Field(default=2000, ge=1, le=65535)
    v2x_enabled: bool = True
    v2x_range: float = Field(default=75.0, ge=10.0, le=500.0)
    tm_port: int = Field(default=8001, ge=1024, le=65535)


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
    try:
        config = WorldConfig(
            host=req.carla_host,
            port=req.carla_port,
            v2x_enabled=req.v2x_enabled,
            v2x_range_m=req.v2x_range,
            tm_port=req.tm_port,
        )
    except ValidationError as exc:
        raise HTTPException(status_code=422, detail=exc.errors(include_url=False, include_context=False))

    global _connect_state, _connect_error
    with _connect_lock:
        if _connect_state == "connecting":
            raise HTTPException(status_code=409, detail="A connect is already in progress")
        _connect_state, _connect_error = "connecting", None

    def _start():
        global _world, _connect_state, _connect_error
        from src.server.persistent_world import PersistentCARLAServer
        try:
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
            logger.info("Persistent world started via admin /connect (%s:%s)", req.carla_host, req.carla_port)
        except Exception as exc:
            logger.error("Connect to CARLA %s:%s failed: %s", req.carla_host, req.carla_port, exc)
            _connect_error = f"{type(exc).__name__}: {exc}"
            _connect_state = "error"

    threading.Thread(target=_start, daemon=True, name="admin-connect").start()
    return {
        "ok": True,
        "message": f"Connecting to CARLA at {req.carla_host}:{req.carla_port} (async)…",
    }


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
    rsus = getattr(_world.v2x_network, "_rsus", {})
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
    vru_messages = getattr(_world.v2x_network, "vru_messages", {})
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
    spat_messages = getattr(_world.v2x_network, "spat_messages", {})
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
                    "min_end_time_s": round(s.min_end_time_s, 1),
                    "max_end_time_s": round(s.max_end_time_s, 1),
                }
                for s in spat.states
            ],
        }
        for spat in spat_messages.values()
    ]


# ------------------------------------------------------------------ #
# Internal helpers                                                    #
# ------------------------------------------------------------------ #

def _assert_ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="Persistent world not initialised")


def _assert_running():
    if not _world._running:
        raise HTTPException(status_code=503, detail="CARLA simulation is not running")
