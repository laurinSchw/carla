"""/ws/client/{id} disconnect semantics (decision D4) against the real tick loop and the recording CARLA stub.

Closing the telemetry socket keeps the session for ``session_disconnect_grace_s``; a reconnect within that time resumes
it; the simulation thread removes it after the grace period; a grace of 0 removes it at once.
"""

import importlib
import sys
import time

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")
from fastapi.testclient import TestClient  # noqa: E402

from src.server.world_config import WorldConfig  # noqa: E402
from tests.unit.carla_stub import Recorder, make_carla_module  # noqa: E402


@pytest.fixture
def env(monkeypatch, tmp_path):
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    import src.server.app as app_mod
    servers = []

    def make(**cfg):
        srv = pw.PersistentCARLAServer(WorldConfig(host="stub", runs_dir=str(tmp_path / "runs"), **cfg))
        srv.start()
        servers.append(srv)
        app_mod.set_persistent_world(srv)
        return srv, TestClient(app_mod.app)

    yield make
    app_mod.set_persistent_world(None)
    for s in servers:
        s.stop()
    sys.modules.pop("src.server.persistent_world", None)


def _wait(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        v = cond()
        if v:
            return v
        time.sleep(0.02)
    return None


def _join(client) -> dict:
    r = client.post("/client/join", json={"client_name": "grace-car"})
    assert r.status_code == 200, r.text
    return r.json()


def _first_telemetry(ws) -> dict:
    for _ in range(20):
        msg = ws.receive_json()
        if msg["type"] == "telemetry":
            return msg
    raise AssertionError("no telemetry frame")


def test_disconnect_keeps_the_vehicle_during_the_grace_period(env):
    srv, client = env(session_disconnect_grace_s=30.0)
    j = _join(client)
    sid = j["session_id"]
    with client.websocket_connect(j["ws_url"]) as ws:
        _first_telemetry(ws)
        assert srv.sessions.get_session(sid).ws_connections == 1
    time.sleep(1.5)  # more than one session check (every 20 frames)
    session = srv.sessions.get_session(sid)
    assert session.is_active and session.ws_connections == 0
    assert sid in srv._client_vehicles


def test_reconnect_within_the_grace_period_resumes_the_session(env):
    srv, client = env(session_disconnect_grace_s=30.0)
    j = _join(client)
    sid = j["session_id"]
    with client.websocket_connect(j["ws_url"]) as ws:
        first = _first_telemetry(ws)
    with client.websocket_connect(j["ws_url"]) as ws:
        again = _first_telemetry(ws)
        assert srv.sessions.get_session(sid).ws_connections == 1
    assert again["data"]["actor_id"] == first["data"]["actor_id"] == j["actor_id"]
    assert sid in srv._client_vehicles and srv.sessions.get_session(sid).is_active


def test_grace_zero_removes_the_session_at_once(env):
    srv, client = env(session_disconnect_grace_s=0.0)
    j = _join(client)
    sid = j["session_id"]
    with client.websocket_connect(j["ws_url"]) as ws:
        _first_telemetry(ws)
    assert srv.sessions.get_session(sid).is_active is False  # ended before the despawn is handed over
    assert _wait(lambda: sid not in srv._client_vehicles, timeout=2.0), "vehicle not despawned"  # well before any session check
    with client.websocket_connect(j["ws_url"]) as ws:
        assert ws.receive_json()["type"] == "error"  # the session is gone, no resume


def test_the_session_is_removed_after_the_grace_period(env):
    srv, client = env(session_disconnect_grace_s=0.4)
    j = _join(client)
    sid = j["session_id"]
    with client.websocket_connect(j["ws_url"]) as ws:
        _first_telemetry(ws)
    assert sid in srv._client_vehicles  # not at once
    assert _wait(lambda: sid not in srv._client_vehicles, timeout=8.0), "grace never expired"
    assert srv.sessions.get_session(sid).is_active is False
    assert any(e["kind"] == "session_disconnected_removed" for e in srv.events.tail(50))
    with client.websocket_connect(j["ws_url"]) as ws:
        assert ws.receive_json()["type"] == "error"


def test_an_open_telemetry_socket_is_never_removed_by_the_grace_rule(env):
    srv, client = env(session_disconnect_grace_s=0.3)
    j = _join(client)
    sid = j["session_id"]
    with client.websocket_connect(j["ws_url"]) as ws:
        _first_telemetry(ws)
        time.sleep(1.5)
        assert _first_telemetry(ws)["data"]["actor_id"] == j["actor_id"]
        assert sid in srv._client_vehicles and srv.sessions.get_session(sid).is_active
