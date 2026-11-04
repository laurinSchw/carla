# pyright: reportOptionalMemberAccess=false
"""LiDAR API (REFACTOR_HANDOFF 5.5): sensors exist only on request, viewers are separate from sensors.

Credentials follow ``roles.py``: ``X-Admin-Token`` (admin) or ``X-Session-Token`` (the ``session_token`` from
``POST /client/join``) as request headers; on ``WS /ws/lidar/{sensor_id}`` also as the first message
``{"op": "auth", ...}``. Never in a URL. Without a credential the caller is an observer, who may list and view sensors
while ``observer_public`` is on. The permission rules live in ``LidarOpsMixin`` / ``LidarManager``.

The older per-session routes (``/client/lidar/{session_id}``, ``WS /ws/lidar/{session_id}``) stay in
``client_routes.py``; both kinds of LiDAR socket share the path ``/ws/lidar/{id}`` and are told apart here: a sensor
id is a number, a session id a UUID.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from typing import Any, Dict, Optional

from fastapi import APIRouter, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

import src.server.client_routes as _client_mod
from src.lidar.config import LidarConfig, lidar_schema
from src.lidar.manager import LidarError, StreamProfile
from src.server.roles import AuthError, Principal, authenticate
from src.server.route_utils import on_sim_thread

logger = logging.getLogger(__name__)

router = APIRouter(tags=["lidar"])

_world = None

_END_STATES = ("destroyed", "error")
POLL_S = 0.02                 # how soon the socket looks for a new frame (the simulation runs at up to 20 FPS)
AUTH_TIMEOUT_S = 5.0          # with observer_public off, a viewer must authenticate within this time
PROFILE_REPORT_S = 1.0        # stream_profile is repeated at most this often while only the point counts change
MAX_MESSAGE_BYTES = 16 * 1024

# WebSocket close codes (the 4xxx range is free for applications)
WS_AUTH_FAILED = 4401
WS_FORBIDDEN = 4403
WS_NOT_FOUND = 4404


def set_persistent_world(world) -> None:
    global _world
    _world = world


# ── helpers ───────────────────────────────────────────────────────────────────────────────────────

def _ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running or _world.lidar is None:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
    return _world


def _principal(x_admin_token: Optional[str], x_session_token: Optional[str]) -> Principal:
    """Headers -> Principal. No credential: observer. A wrong one is an error (403 admin, 401 session), never a downgrade."""
    try:
        if _world is None and x_session_token and not x_admin_token:
            raise AuthError("auth_failed", "unknown or ended session")    # no world, no sessions
        return authenticate(_world.sessions if _world is not None else None,
                            x_admin_token or None, x_session_token or None)
    except AuthError as exc:
        if exc.code == "admin_not_configured":
            raise HTTPException(status_code=500, detail=exc.message)
        raise HTTPException(status_code=403 if x_admin_token else 401, detail=exc.message)


def _read_gate(principal: Principal) -> None:
    if principal.kind == "observer" and _world is not None and not _world.observer_public:
        raise HTTPException(status_code=403, detail="observer access is disabled; send X-Admin-Token or X-Session-Token")


def _lidar_error(exc: LidarError) -> JSONResponse:
    return JSONResponse(status_code=exc.status, content=exc.detail)


def _accepted(info: dict) -> JSONResponse:
    return JSONResponse(status_code=202, content={
        "sensor_id": info["sensor_id"], "state": info["state"], "ws_url": f"/ws/lidar/{info['sensor_id']}", "sensor": info})


def _session_principal(session_id: str, x_admin_token: Optional[str], x_session_token: Optional[str]) -> Principal:
    """The admin, or the vehicle of exactly this session."""
    principal = _principal(x_admin_token, x_session_token)
    if principal.kind == "admin":
        session = _world.sessions.get_session(session_id)
        if session is None or not session.is_active:
            raise HTTPException(status_code=404, detail="Invalid or expired session")
        return principal
    if principal.kind != "vehicle" or principal.session_id != session_id:
        raise HTTPException(status_code=403, detail="X-Session-Token must belong to this session (or send X-Admin-Token)")
    return principal


# ── REST ──────────────────────────────────────────────────────────────────────────────────────────

@router.get("/api/lidar/schema", summary="What can be configured per sensor type, the semantic tag table and the limits")
async def get_schema(x_admin_token: Optional[str] = Header(default=None),
                     x_session_token: Optional[str] = Header(default=None)):
    _read_gate(_principal(x_admin_token, x_session_token))
    limits: Dict[str, Any] = {}
    if _world is not None and _world.lidar is not None:
        lim = _world.lidar.limits
        limits = {"max_sensors": lim.max_sensors, "max_total_points_per_second": lim.max_total_points_per_second,
                  "keep_alive_s": lim.keep_alive_s, "fixed_delta_seconds": _world.fixed_delta_seconds}
    return lidar_schema(limits)


@router.get("/api/lidar/sensors", summary="LiDAR sensors and the budget")
async def list_sensors(x_admin_token: Optional[str] = Header(default=None),
                       x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(_principal(x_admin_token, x_session_token))
    return {"sensors": world.lidar.list_sensors(), "budget": world.lidar.budget()}


class SensorRequest(BaseModel):
    actor_id: Optional[int] = Field(default=None, description="Vehicle to attach to; a vehicle session may omit it (own vehicle)")
    config: Optional[LidarConfig] = None
    record_raw: bool = Field(default=False, description="Keep the sensor alive and record its raw frames in a run (admin)")
    pinned: bool = Field(default=False, description="Never auto-release without viewers (admin)")


@router.post("/api/lidar/sensors", status_code=202, summary="Request a sensor (admin: any vehicle; vehicle: its own)")
async def create_sensor(req: SensorRequest, x_admin_token: Optional[str] = Header(default=None),
                        x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    principal = _principal(x_admin_token, x_session_token)
    if principal.kind == "observer":
        raise HTTPException(status_code=403, detail="observers cannot create LiDAR sensors")
    try:
        info = await on_sim_thread(world, lambda: world.lidar_request(
            principal, req.actor_id, req.config, record_raw=req.record_raw, pinned=req.pinned))
    except LidarError as exc:
        return _lidar_error(exc)
    return _accepted(info)


@router.get("/api/lidar/sensors/{sensor_id}", summary="One sensor")
async def get_sensor(sensor_id: int, x_admin_token: Optional[str] = Header(default=None),
                     x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(_principal(x_admin_token, x_session_token))
    info = world.lidar.get_info(sensor_id)
    if info is None:
        raise HTTPException(status_code=404, detail=f"no LiDAR sensor {sensor_id}")
    return info


@router.put("/api/lidar/sensors/{sensor_id}", status_code=202,
            summary="Reconfigure a sensor (destroyed and respawned at a tick boundary)")
async def reconfigure_sensor(sensor_id: int, config: LidarConfig, x_admin_token: Optional[str] = Header(default=None),
                             x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    principal = _principal(x_admin_token, x_session_token)
    try:
        info = await on_sim_thread(world, lambda: world.lidar_reconfigure(principal, sensor_id, config))
    except LidarError as exc:
        return _lidar_error(exc)
    return _accepted(info)


class SensorFlags(BaseModel):
    pinned: Optional[bool] = None
    record_raw: Optional[bool] = None


@router.patch("/api/lidar/sensors/{sensor_id}", summary="Pin a sensor / mark it for raw recording (admin)")
async def patch_sensor(sensor_id: int, body: SensorFlags, x_admin_token: Optional[str] = Header(default=None),
                       x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    principal = _principal(x_admin_token, x_session_token)
    try:
        return await on_sim_thread(world, lambda: world.lidar_set_flags(principal, sensor_id, body.pinned, body.record_raw))
    except LidarError as exc:
        return _lidar_error(exc)


@router.delete("/api/lidar/sensors/{sensor_id}", status_code=202, summary="Release a sensor")
async def release_sensor(sensor_id: int, x_admin_token: Optional[str] = Header(default=None),
                         x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    principal = _principal(x_admin_token, x_session_token)
    try:
        await on_sim_thread(world, lambda: world.lidar_release(principal, sensor_id))
    except LidarError as exc:
        return _lidar_error(exc)
    return {"sensor_id": sensor_id, "state": "stopping"}


@router.get("/api/sessions/{session_id}/lidar", summary="The LiDAR sensor of a session (its vehicle or the admin)")
async def get_session_sensor(session_id: str, x_admin_token: Optional[str] = Header(default=None),
                             x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    _session_principal(session_id, x_admin_token, x_session_token)
    return {"sensor": world.get_client_lidar_info(session_id), "budget": world.lidar.budget()}


async def _enable_session_sensor(session_id: str, config: Optional[LidarConfig], x_admin_token: Optional[str],
                                 x_session_token: Optional[str]):
    world = _ready()
    _session_principal(session_id, x_admin_token, x_session_token)
    try:
        info = await on_sim_thread(world, world.enable_client_lidar, session_id, config)
    except LidarError as exc:
        return _lidar_error(exc)
    session = world.sessions.get_session(session_id)
    if session is not None:
        session.lidar_enabled = True
    return _accepted(info)


@router.post("/api/sessions/{session_id}/lidar", status_code=202,
             summary="Request a sensor on the session's own vehicle (an existing one is reconfigured)")
async def create_session_sensor(session_id: str, config: Optional[LidarConfig] = None,
                                x_admin_token: Optional[str] = Header(default=None),
                                x_session_token: Optional[str] = Header(default=None)):
    return await _enable_session_sensor(session_id, config, x_admin_token, x_session_token)


@router.put("/api/sessions/{session_id}/lidar", status_code=202,
            summary="Reconfigure the session's sensor (requested if it has none)")
async def put_session_sensor(session_id: str, config: LidarConfig, x_admin_token: Optional[str] = Header(default=None),
                             x_session_token: Optional[str] = Header(default=None)):
    return await _enable_session_sensor(session_id, config, x_admin_token, x_session_token)


@router.delete("/api/sessions/{session_id}/lidar", status_code=202, summary="Release the session's sensor")
async def delete_session_sensor(session_id: str, x_admin_token: Optional[str] = Header(default=None),
                                x_session_token: Optional[str] = Header(default=None)):
    world = _ready()
    _session_principal(session_id, x_admin_token, x_session_token)
    try:
        removed = await on_sim_thread(world, world.disable_client_lidar, session_id)
    except LidarError as exc:
        return _lidar_error(exc)
    session = world.sessions.get_session(session_id)
    if session is not None:
        session.lidar_enabled = False
    return {"released": removed}


# ── WebSocket: binary protocol v4 frames + JSON control messages ─────────────────────────────────

def parse_profile(msg: Dict[str, Any], sim_hz: float) -> StreamProfile:
    """``{"max_points": int|null, "max_hz": number|null, "compress": bool}`` -> quantised StreamProfile (ValueError if bad)."""
    max_points = msg.get("max_points")
    max_hz = msg.get("max_hz")
    compress = msg.get("compress", False)
    if max_points is not None and (isinstance(max_points, bool) or not isinstance(max_points, int) or max_points <= 0):
        raise ValueError("max_points must be a positive integer or null")
    if max_hz is not None and (isinstance(max_hz, bool) or not isinstance(max_hz, (int, float)) or not max_hz > 0):
        raise ValueError("max_hz must be a positive number or null")
    if not isinstance(compress, bool):
        raise ValueError("compress must be true or false")
    return StreamProfile.quantise(max_points, float(max_hz) if max_hz is not None else None, compress, sim_hz)


@router.websocket("/ws/lidar/{sensor_ref}")
async def lidar_websocket(websocket: WebSocket, sensor_ref: str, max_points: Optional[int] = None,
                          max_hz: Optional[float] = None, compress: bool = False):
    """``/ws/lidar/{sensor_id}`` views one sensor; ``/ws/lidar/{session_id}`` is the older per-session stream."""
    if not sensor_ref.isdigit():
        await _client_mod.session_lidar_websocket(websocket, sensor_ref, max_points, max_hz, compress)
        return
    await sensor_websocket(websocket, int(sensor_ref), max_points, max_hz, compress)


async def sensor_websocket(websocket: WebSocket, sensor_id: int, max_points: Optional[int] = None,
                           max_hz: Optional[float] = None, compress: bool = False) -> None:
    """
    View one sensor. Binary messages are protocol v4 frames (``src/lidar/protocol.py``); text messages are JSON::

        {"type": "state", "state": "active", "frame": 1234, "error": null}
        {"type": "stream_profile", "max_points": 25000, "max_hz": 10.0, "every_n_frames": 2, "compress": false,
         "points_total": 41000, "points_sent": 25000, "subsampled": true}
        {"type": "auth", "role": "admin"}
        {"type": "pong"}
        {"type": "error", "code": "...", "message": "..."}

    The client may send ``{"op": "auth", "admin_token": ...}`` / ``{"op": "auth", "session_token": ...}`` (needed when
    ``observer_public`` is off, within ``AUTH_TIMEOUT_S``), ``{"op": "profile", "max_points": 25000, "max_hz": 10,
    "compress": true}`` (quantised server side so viewers share encoded frames) and ``"ping"`` / ``{"op": "ping"}``.
    Closing the socket does not release the sensor (it is released ``keep_alive_s`` after its last viewer left).
    """
    await websocket.accept()
    world = _world
    if world is None or world.lidar is None:
        await websocket.send_json({"type": "error", "code": "not_ready", "message": "Server not initialised"})
        await websocket.close()
        return
    mgr = world.lidar

    async def _refuse(code: str, message: str, close_code: int) -> None:
        await websocket.send_json({"type": "error", "code": code, "message": message})
        await websocket.close(code=close_code)

    try:  # credentials on the handshake (non-browser clients); browsers use the auth message
        principal = authenticate(world.sessions, websocket.headers.get("x-admin-token") or None,
                                 websocket.headers.get("x-session-token") or None)
    except AuthError as exc:
        await _refuse(exc.code, exc.message, WS_AUTH_FAILED)
        return
    if mgr.get_info(sensor_id) is None:
        await _refuse("not_found", f"no LiDAR sensor {sensor_id}", WS_NOT_FOUND)
        return

    sim_hz = 1.0 / world.fixed_delta_seconds
    box: Dict[str, Any] = {"principal": principal, "changed": True, "auth_failed": None,
                           "profile": StreamProfile.quantise(max_points, max_hz, compress, sim_hz)}

    async def _receive_loop() -> None:
        while True:
            raw = await websocket.receive_text()
            if raw == "ping":
                await websocket.send_json({"type": "pong"})
                continue
            try:
                if len(raw) > MAX_MESSAGE_BYTES:
                    raise ValueError(f"message larger than {MAX_MESSAGE_BYTES} bytes")
                msg = json.loads(raw)
                if not isinstance(msg, dict) or not isinstance(msg.get("op"), str):
                    raise ValueError('expected {"op": ...}')
                op = msg["op"]
                if op == "ping":
                    await websocket.send_json({"type": "pong"})
                elif op == "profile":
                    box["profile"], box["changed"] = parse_profile(msg, sim_hz), True
                elif op == "auth":
                    try:
                        p = authenticate(world.sessions, msg.get("admin_token"), msg.get("session_token"))
                    except AuthError as exc:
                        box["auth_failed"] = exc
                        return
                    if p.kind == "observer":
                        raise ValueError("auth needs admin_token or session_token")
                    box["principal"] = p
                    await websocket.send_json({"type": "auth", "role": p.kind})
                else:
                    raise ValueError(f"unknown op {op!r}")
            except ValueError as exc:
                await websocket.send_json({"type": "error", "code": "bad_message", "message": str(exc)})

    receiver = asyncio.create_task(_receive_loop())
    viewer_id = uuid.uuid4().hex[:8]
    attached = False
    deadline = time.monotonic() + AUTH_TIMEOUT_S
    last_state: Optional[str] = None
    last_frame = -1
    last_report: Optional[tuple] = None
    last_report_t = 0.0
    try:
        while True:
            if receiver.done():
                if box["auth_failed"] is not None:
                    exc = box["auth_failed"]
                    await _refuse(exc.code, exc.message, WS_AUTH_FAILED)
                    return
                receiver.result()          # re-raises a disconnect
                break
            info = mgr.get_info(sensor_id)
            if not attached:
                if info is not None and not world.lidar_may_view(box["principal"], info):
                    if box["principal"].kind != "observer" or time.monotonic() >= deadline:
                        await _refuse("forbidden", "you may not view this sensor (observer access is disabled or it is "
                                      "not on your vehicle); authenticate with {\"op\": \"auth\", ...}", WS_FORBIDDEN)
                        return
                    await asyncio.sleep(POLL_S)
                    continue
                if info is not None:
                    try:
                        mgr.viewer_attached(sensor_id)
                        attached = True
                        logger.info("LiDAR WS %s viewing sensor=%d as %s", viewer_id, sensor_id, box["principal"].kind)
                    except LidarError:
                        info = None    # released between the lookup and now
            state = info["state"] if info else "destroyed"
            if state != last_state:
                last_state = state
                await websocket.send_json({"type": "state", "state": state,
                                           "frame": info["last_frame"] if info else (last_frame if last_frame >= 0 else None),
                                           "error": info["error"] if info else None})
                if state in _END_STATES:
                    break
            prof: StreamProfile = box["profile"]
            # encoding a 25k-250k point frame takes 1-45 ms: keep it off the event loop that serves every other client
            got = await asyncio.to_thread(mgr.get_stream_frame_counted, sensor_id, prof, last_frame)
            if got is not None:
                last_frame, payload, total, sent = got
                report = (prof, total > sent)
                now = time.monotonic()
                if box["changed"] or report != last_report or now - last_report_t >= PROFILE_REPORT_S:
                    box["changed"], last_report, last_report_t = False, report, now
                    await websocket.send_json({
                        "type": "stream_profile", "max_points": prof.max_points, "max_hz": sim_hz / prof.every_n_frames,
                        "every_n_frames": prof.every_n_frames, "compress": prof.compress,
                        "points_total": total, "points_sent": sent, "subsampled": total > sent})
                await websocket.send_bytes(payload)
            await asyncio.sleep(POLL_S)
        await websocket.close(code=1000)
    except WebSocketDisconnect:
        logger.info("LiDAR WS %s disconnected: sensor=%d", viewer_id, sensor_id)
    except RuntimeError as exc:
        if "close message" in str(exc) or "disconnect" in str(exc).lower():
            logger.info("LiDAR WS %s closed by peer: sensor=%d", viewer_id, sensor_id)
        else:
            logger.error("LiDAR WS error for sensor %d: %s", sensor_id, exc)
    except Exception as exc:
        logger.error("LiDAR WS error for sensor %d: %s", sensor_id, exc)
    finally:
        receiver.cancel()
        if attached:
            mgr.viewer_detached(sensor_id)
