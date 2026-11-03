# TODO: CARLA handles / the module-global world are Optional until a connected-world type exists (WorldController state machine)
# pyright: reportOptionalMemberAccess=false
"""
Client-facing FastAPI router.

Any device on the network (laptop, raspi, workstation…) can:
  1. POST /client/join   → register + spawn their leading vehicle
  2. WS   /ws/client/{session_id}  → receive real-time telemetry (closing it keeps the vehicle for the
     disconnect grace period, decision D4; reconnecting in time resumes the session)
  3. DELETE /client/leave/{session_id}  → graceful exit + despawn

No authentication is required — any device on the LAN can connect.

Mount on the main app with::

    from src.server.client_routes import router as client_router
    app.include_router(client_router)
"""

import asyncio
import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from src.lidar.config import LidarConfig, lidar_schema
from src.lidar.manager import LidarError, StreamProfile
from src.server.route_utils import on_sim_thread

logger = logging.getLogger(__name__)

router = APIRouter(tags=["client"])

TELEMETRY_PERIOD_S = 0.2  # 5 Hz
LIDAR_PERIOD_S = 0.1  # 10 Hz

# Injected once at startup by start_server.py
_world = None

# Vehicle blueprints a client may request (validated to be always-available)
ALLOWED_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020",
    "vehicle.ford.mustang",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.lincoln.mkz_2020",
    "vehicle.mercedes.coupe",
    "vehicle.mini.cooperst",
    "vehicle.nissan.micra",
    "vehicle.seat.leon",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2_2021",
]
_DEFAULT_BLUEPRINT = "vehicle.tesla.model3"


def set_persistent_world(world) -> None:
    global _world
    _world = world


# ------------------------------------------------------------------ #
# Request / response models                                           #
# ------------------------------------------------------------------ #

class JoinRequest(BaseModel):
    client_name: str = Field(..., min_length=1, max_length=64,
                             description="Friendly name shown in the admin panel (e.g. 'raspi-1')")
    vehicle_blueprint: str = Field(default=_DEFAULT_BLUEPRINT,
                                   description="CARLA blueprint ID for the leading vehicle")
    duration_seconds: Optional[float] = Field(
        default=None, ge=10, le=86400,
        description="Session lifetime in seconds. Omit or set null to run until the client leaves "
                    "or has had no connection for longer than session_disconnect_grace_s.",
    )
    enable_lidar: bool = Field(
        default=False,
        description="Attach a LiDAR sensor and open /ws/lidar/{session_id} for binary streaming.",
    )

class JoinResponse(BaseModel):
    session_id: str
    session_token: str  # secret for the WebSocket hub and LiDAR requests: send it in the first hub message or the X-Session-Token header, never in a URL
    actor_id: int
    ws_url: str
    lidar_ws_url: Optional[str] = None
    lidar_error: Optional[dict] = None  # set when enable_lidar was asked for but refused (e.g. budget)
    message: str


# ------------------------------------------------------------------ #
# REST endpoints                                                      #
# ------------------------------------------------------------------ #

@router.get("/client/blueprints", summary="Available vehicle blueprints")
async def get_blueprints():
    return {"blueprints": ALLOWED_BLUEPRINTS, "default": _DEFAULT_BLUEPRINT}


@router.post("/client/join", response_model=JoinResponse, summary="Join the live server")
async def join_server(req: JoinRequest, request: Request):
    """
    Register a client device.  Spawns a leading vehicle immediately and
    returns the session_id and the WebSocket URL for telemetry streaming.
    """
    _assert_world_ready()

    # Sanitise blueprint choice
    blueprint = req.vehicle_blueprint if req.vehicle_blueprint in ALLOWED_BLUEPRINTS else _DEFAULT_BLUEPRINT

    client_ip = request.client.host if request.client else "unknown"

    session = _world.sessions.create_session(
        client_name=req.client_name,
        client_ip=client_ip,
        vehicle_blueprint=blueprint,
        duration_seconds=req.duration_seconds,
    )

    try:
        actor_id = await on_sim_thread(
            _world, _world.spawn_client_vehicle, session.session_id, blueprint
        )
    except HTTPException:
        _world.sessions.remove_session(session.session_id)
        raise
    except Exception as exc:
        _world.sessions.remove_session(session.session_id)
        logger.error("Vehicle spawn failed for session %s: %s", session.session_id, exc)
        raise HTTPException(status_code=500, detail=f"Vehicle spawn failed: {exc}")

    if actor_id is None:
        _world.sessions.remove_session(session.session_id)
        raise HTTPException(status_code=500, detail="Could not find a free spawn point")

    duration_msg = (
        f"{int(req.duration_seconds)}s" if req.duration_seconds else "until you leave"
    )

    lidar_ws_url: Optional[str] = None
    lidar_error: Optional[dict] = None
    if req.enable_lidar:
        try:
            await on_sim_thread(_world, _world.enable_client_lidar, session.session_id)
            session.lidar_enabled = True
            lidar_ws_url = f"/ws/lidar/{session.session_id}"
        except LidarError as exc:  # joining still works without LiDAR; tell the client why
            lidar_error = exc.detail
            logger.warning("LiDAR refused for session %s: %s", session.session_id, exc)

    return JoinResponse(
        session_id=session.session_id,
        session_token=session.session_token,
        actor_id=actor_id,
        ws_url=f"/ws/client/{session.session_id}",
        lidar_ws_url=lidar_ws_url,
        lidar_error=lidar_error,
        message=f"Welcome {req.client_name}! Vehicle actor={actor_id} spawned. Session runs {duration_msg}.",
    )


@router.delete("/client/leave/{session_id}", summary="Leave the server")
async def leave_server(session_id: str):
    """Gracefully end a session and despawn the leading vehicle."""
    _assert_world_ready()
    session = _world.sessions.get_session(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Session not found")

    await on_sim_thread(_world, _world.despawn_client_vehicle, session_id)
    _world.sessions.end_session(session_id)
    return {"ok": True, "message": f"Goodbye {session.client_name}!"}


@router.get("/client/vehicle/{session_id}", summary="Get vehicle telemetry (REST)")
async def get_vehicle_state(session_id: str):
    """One-shot telemetry poll (for clients that can't use WebSocket)."""
    _assert_world_ready()
    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        raise HTTPException(status_code=404, detail="Session not found or ended")

    _world.sessions.touch(session_id)  # a poll is a sign of life for the disconnect grace rule (D4)
    telemetry = _world.get_vehicle_telemetry(session_id)
    if telemetry is None:
        raise HTTPException(status_code=404, detail="Vehicle not found")

    return {**telemetry, "session": session.to_dict()}


# ------------------------------------------------------------------ #
# WebSocket — per-client telemetry stream                             #
# ------------------------------------------------------------------ #

@router.websocket("/ws/client/{session_id}")
async def client_websocket(websocket: WebSocket, session_id: str):
    """
    Stream real-time telemetry to a specific client at ~5 Hz.

    The server sends JSON frames::

        {
          "type": "telemetry",
          "data": { "actor_id": …, "x": …, "y": …, "z": …,
                    "yaw": …, "speed_kmh": …, "frame": … },
          "remaining_seconds": 120.5   # null if no duration limit
        }

    The client can send ``"ping"`` to keep the connection alive.
    When the session expires the server sends ``{"type": "session_expired"}``
    and closes the connection.

    Closing the socket does NOT end the session (decision D4): the vehicle is kept for
    ``session_disconnect_grace_s`` seconds and a reconnect within that time resumes the session.
    The simulation thread removes it when nobody came back in time. With a grace of 0 the
    session ends at once, unless another connection (e.g. the hub) still holds it.
    """
    await websocket.accept()

    if _world is None:
        await websocket.send_json({"type": "error", "message": "Server not initialised"})
        await websocket.close()
        return

    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        await websocket.send_json({"type": "error", "message": "Invalid or expired session"})
        await websocket.close()
        return

    _world.sessions.mark_connected(session_id)
    logger.info("Client WS connected: session=%s (%s)", session_id, session.client_name)

    async def _receive_loop():
        # Separate task: a client "ping" is answered at once and never delays telemetry.
        while True:
            msg = await websocket.receive_text()
            if msg == "ping":
                await websocket.send_json({"type": "pong"})

    receiver = asyncio.create_task(_receive_loop())
    try:
        while True:
            # Session expired?
            if not session.is_active or session.is_expired:
                await websocket.send_json({"type": "session_expired"})
                break
            if receiver.done():  # client closed or the socket failed
                receiver.result()
                break

            _world.sessions.touch(session_id)

            telemetry = _world.get_vehicle_telemetry(session_id)
            if telemetry:
                v2x_data = _world.get_vehicle_v2x_data(session_id)
                await websocket.send_json({
                    "type": "telemetry",
                    "data": {**telemetry, "v2x": v2x_data},
                    "remaining_seconds": session.remaining_seconds,
                })

            await asyncio.sleep(TELEMETRY_PERIOD_S)

    except WebSocketDisconnect:
        logger.info("Client WS disconnected: session=%s", session_id)
    except RuntimeError as exc:
        if "close message" in str(exc):  # peer closed while we were sending: a normal disconnect
            logger.info("Client WS closed by peer: session=%s", session_id)
        else:
            logger.error("Client WS error for session %s: %s", session_id, exc)
    except Exception as exc:
        logger.error("Client WS error for session %s: %s", session_id, exc)
    finally:
        receiver.cancel()
        await _client_ws_closed(session_id)


async def _client_ws_closed(session_id: str) -> None:
    """Release this socket's hold on the session; end it at once only when the grace period is 0 (decision D4)."""
    world = _world  # the module global may be replaced (stop / new connect) while we await
    if world is None:
        return
    world.sessions.mark_disconnected(session_id)
    session = world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        return
    # grace > 0: the simulation thread removes it if nobody reconnects in time; grace 0: ended here. The despawn is queued
    # before the first await and shielded: if this task is cancelled the vehicle of an ended session must still be removed.
    fut = world.end_session_if_unheld(session_id) if hasattr(world, "end_session_if_unheld") else None
    if fut is None:
        logger.info("Client WS closed: session=%s kept (grace %.0f s or held by another connection)", session_id,
                    float(getattr(world, "session_disconnect_grace_s", 0.0)))
        return
    try:
        await asyncio.shield(asyncio.wrap_future(fut))
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # world may have stopped meanwhile
        logger.warning("Cleanup of session %s skipped: %s", session_id, exc)


# ------------------------------------------------------------------ #
# LiDAR on request — REST                                             #
# ------------------------------------------------------------------ #

def _lidar_error_response(exc: LidarError) -> JSONResponse:
    return JSONResponse(status_code=exc.status, content=exc.detail)


def _active_session(session_id: str):
    _assert_world_ready()
    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        raise HTTPException(status_code=404, detail="Invalid or expired session")
    return session


@router.get("/client/lidar/schema", summary="Valid LiDAR settings per sensor type")
async def lidar_schema_endpoint():
    return lidar_schema()


@router.get("/client/lidar/{session_id}", summary="State of this session's LiDAR sensor")
async def get_lidar(session_id: str):
    _active_session(session_id)
    if _world.lidar is None:
        raise HTTPException(status_code=503, detail="LiDAR is not available")
    return {"sensor": _world.get_client_lidar_info(session_id), "budget": _world.lidar.budget()}


@router.put("/client/lidar/{session_id}", status_code=202,
            summary="Request or reconfigure this session's LiDAR (applied at the next tick boundary)")
async def put_lidar(session_id: str, config: LidarConfig = LidarConfig()):
    session = _active_session(session_id)
    try:
        info = await on_sim_thread(_world, _world.enable_client_lidar, session_id, config)
    except LidarError as exc:
        return _lidar_error_response(exc)
    session.lidar_enabled = True
    return {"sensor": info, "lidar_ws_url": f"/ws/lidar/{session_id}"}


@router.delete("/client/lidar/{session_id}", status_code=202, summary="Release this session's LiDAR")
async def delete_lidar(session_id: str):
    session = _active_session(session_id)
    try:
        removed = await on_sim_thread(_world, _world.disable_client_lidar, session_id)
    except LidarError as exc:
        return _lidar_error_response(exc)
    session.lidar_enabled = False
    return {"released": removed}


# ------------------------------------------------------------------ #
# WebSocket — per-client LiDAR binary stream                         #
# ------------------------------------------------------------------ #

# Mounted as WS /ws/lidar/{session_id} by lidar_routes.py, which shares the path with /ws/lidar/{sensor_id}
# (a session id is a UUID, a sensor id a number); one route for both keeps the match independent of router order.
async def session_lidar_websocket(websocket: WebSocket, session_id: str, max_points: Optional[int] = None,
                          max_hz: Optional[float] = None, compress: bool = False):
    """
    Stream LiDAR frames of this session's sensor as binary protocol v4 (``src/lidar/protocol.py``).

    Query: ``max_points`` (quantised to 10k/25k/50k/all), ``max_hz`` (a divisor of the simulation
    rate), ``compress`` (zlib). Frames are in the SENSOR frame with the sensor pose in the header.
    Text messages ``{"type": "state", "state": ..., "frame": ...}`` announce lifecycle changes.
    Accepts ``"ping"`` for keep-alive. Closing the socket does NOT release the sensor; it is released
    ``keep_alive_s`` after the last viewer left (or when the session ends).
    """
    await websocket.accept()

    if _world is None or _world.lidar is None:
        await websocket.send_json({"type": "error", "message": "Server not initialised"})
        await websocket.close()
        return

    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        await websocket.send_json({"type": "error", "message": "Invalid or expired session"})
        await websocket.close()
        return

    info = _world.get_client_lidar_info(session_id)
    if info is None:
        await websocket.send_json({"type": "error", "message": "LiDAR not enabled for this session"})
        await websocket.close()
        return

    profile = StreamProfile.quantise(max_points, max_hz, compress, 1.0 / _world.fixed_delta_seconds)
    sensor_id = info["sensor_id"]
    try:
        _world.lidar.viewer_attached(sensor_id)
    except LidarError:  # the sensor was released between the lookup and now
        await websocket.send_json({"type": "state", "state": "destroyed", "frame": None, "error": None})
        await websocket.close()
        return
    logger.info("LiDAR WS connected: session=%s (%s) sensor=%d profile=%s", session_id, session.client_name, sensor_id, profile)

    async def _receive_loop():
        while True:
            msg = await websocket.receive_text()
            if msg == "ping":
                await websocket.send_json({"type": "pong"})

    receiver = asyncio.create_task(_receive_loop())
    last_frame, last_state = -1, None
    try:
        while True:
            if not session.is_active or session.is_expired:
                await websocket.send_json({"type": "session_expired"})
                break
            if receiver.done():
                receiver.result()
                break

            current = _world.get_client_lidar_info(session_id)
            state = current["state"] if current else "destroyed"
            if state != last_state:
                await websocket.send_json({"type": "state", "state": state,
                                           "frame": current["last_frame"] if current else last_frame,
                                           "error": current["error"] if current else None})
                last_state = state
            if current is None or current["sensor_id"] != sensor_id:
                break  # released (or replaced): the viewer reconnects if it wants a new sensor

            # encoding a 25k-250k point frame takes 1-45 ms: keep it off the event loop that serves every other client
            got = await asyncio.to_thread(_world.get_client_lidar_frame, session_id, profile, last_frame)
            if got is not None:
                last_frame, payload = got
                await websocket.send_bytes(payload)

            await asyncio.sleep(LIDAR_PERIOD_S)

    except WebSocketDisconnect:
        logger.info("LiDAR WS disconnected: session=%s", session_id)
    except RuntimeError as exc:
        if "close message" in str(exc):  # peer closed while we were sending: a normal disconnect
            logger.info("LiDAR WS closed by peer: session=%s", session_id)
        else:
            logger.error("LiDAR WS error for session %s: %s", session_id, exc)
    except Exception as exc:
        logger.error("LiDAR WS error for session %s: %s", session_id, exc)
    finally:
        receiver.cancel()
        _world.lidar.viewer_detached(sensor_id)


# ------------------------------------------------------------------ #
# Internal helpers                                                    #
# ------------------------------------------------------------------ #

def _assert_world_ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
