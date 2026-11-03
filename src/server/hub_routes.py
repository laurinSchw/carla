"""``/ws``: the multiplexed JSON WebSocket (see ``ws_hub.py``) plus the topic catalogue."""

from typing import Optional

from fastapi import APIRouter, Header, HTTPException, WebSocket

from src.server.admin_routes import check_admin_token

from src.server.topics import TOPICS
from src.server.ws_hub import Hub

router = APIRouter(tags=["hub"])

_world = None
_hub = None


def set_persistent_world(world) -> None:
    global _world, _hub
    _world = world
    _hub = Hub(world) if world is not None else None


@router.get("/api/topics", summary="WebSocket topics, who may subscribe and which filter they need")
async def list_topics():
    return {"endpoint": "/ws", "topics": [
        {"name": t.name, "roles": sorted(t.roles), "max_hz": t.max_hz, "default_hz": t.default_hz,
         "filter_required": t.filter_required, "ground_truth": t.ground_truth, "description": t.description}
        for t in TOPICS.values()]}


@router.get("/api/hub/connections", summary="WebSocket clients and their subscriptions (admin)")
async def hub_connections(x_admin_token: Optional[str] = Header(default=None)):
    check_admin_token(x_admin_token or "")
    if _hub is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    return _hub.describe()


@router.websocket("/ws")
async def hub_websocket(websocket: WebSocket):
    if _hub is None:
        await websocket.accept()
        await websocket.send_json({"topic": "error", "code": "not_ready", "message": "Server not initialised"})
        await websocket.close()
        return
    await _hub.serve(websocket)
