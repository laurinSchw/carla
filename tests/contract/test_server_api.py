"""HTTP/WebSocket contract tests for the server (no CARLA needed).

Drives the real FastAPI app (src.server.app) with a FakeWorld. They cover what a deployed
server must do: pages, admin auth, the join -> telemetry -> leave life cycle, kick, admin
population endpoints and the behaviour when the world is missing or stopped.
"""

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")

from fastapi.testclient import TestClient  # noqa: E402

import src.server.app as app_mod
from tests.contract.fake_world import FakeWorld

TOKEN = "test-token"
AUTH = {"X-Admin-Token": TOKEN}


@pytest.fixture(autouse=True)
def _env(monkeypatch):
    monkeypatch.setenv("ADMIN_TOKEN", TOKEN)
    yield
    app_mod.set_persistent_world(None)


@pytest.fixture
def client():
    return TestClient(app_mod.app)


@pytest.fixture
def world():
    w = FakeWorld()
    app_mod.set_persistent_world(w)
    return w


def _join(client, name="car-1", **extra):
    return client.post("/client/join", json={"client_name": name, **extra})


# --- pages --------------------------------------------------------------

@pytest.mark.parametrize("path", ["/", "/admin", "/client"])
def test_pages_return_200_html(client, path):
    r = client.get(path)
    assert r.status_code == 200
    assert "text/html" in r.headers["content-type"]


def test_unknown_page_is_404(client):
    assert client.get("/v2v").status_code == 404
    assert client.get("/api/simulation/status").status_code == 404


def test_openapi_schema_is_served(client):
    assert client.get("/openapi.json").status_code == 200


# --- admin auth ---------------------------------------------------------

ADMIN_GETS = ["/admin/status", "/admin/sessions", "/admin/v2x/rsus", "/admin/v2x/vrus", "/admin/v2x/spat"]


@pytest.mark.parametrize("path", ADMIN_GETS)
def test_admin_requires_token_header(client, path):
    assert client.get(path).status_code == 422  # header missing


@pytest.mark.parametrize("path", ADMIN_GETS)
def test_admin_rejects_wrong_token(client, path):
    assert client.get(path, headers={"X-Admin-Token": "nope"}).status_code == 403


def test_admin_500_when_server_has_no_token(client, monkeypatch):
    monkeypatch.delenv("ADMIN_TOKEN", raising=False)
    assert client.get("/admin/status", headers=AUTH).status_code == 500


@pytest.mark.parametrize("path", ADMIN_GETS)
def test_admin_accepts_correct_token_without_world(client, path):
    assert client.get(path, headers=AUTH).status_code == 200


def test_status_before_connect(client):
    body = client.get("/admin/status", headers=AUTH).json()
    assert body["connected"] is False and body["running"] is False
    assert client.get("/admin/sessions", headers=AUTH).json() == []


def test_status_with_world(client, world):
    body = client.get("/admin/status", headers=AUTH).json()
    assert body["connected"] is True and body["running"] is True and body["map"] == "Town01"


# --- client life cycle --------------------------------------------------

def test_blueprints_listed(client):
    body = client.get("/client/blueprints").json()
    assert body["default"] in body["blueprints"]


def test_join_before_connect_is_503(client):
    assert _join(client).status_code == 503


def test_join_when_simulation_stopped_is_503(client):
    app_mod.set_persistent_world(FakeWorld(running=False))
    assert _join(client).status_code == 503


@pytest.mark.parametrize("payload", [
    {}, {"client_name": ""}, {"client_name": "x" * 65},
    {"client_name": "ok", "duration_seconds": 5},
    {"client_name": "ok", "duration_seconds": 10**7},
])
def test_join_validation(client, world, payload):
    assert client.post("/client/join", json=payload).status_code == 422


def test_join_leave_lifecycle(client, world):
    r = _join(client, duration_seconds=60)
    assert r.status_code == 200
    j = r.json()
    assert j["ws_url"] == f"/ws/client/{j['session_id']}" and j["actor_id"] in world.spawned.values()

    sessions = client.get("/admin/sessions", headers=AUTH).json()
    assert [s["session_id"] for s in sessions] == [j["session_id"]]

    tele = client.get(f"/client/vehicle/{j['session_id']}")
    assert tele.status_code == 200 and tele.json()["speed_kmh"] == 12.5

    assert client.delete(f"/client/leave/{j['session_id']}").status_code == 200
    assert j["session_id"] in world.despawned
    assert client.get(f"/client/vehicle/{j['session_id']}").status_code == 404
    ended = client.get("/admin/sessions", headers=AUTH).json()
    assert ended[0]["session_id"] == j["session_id"]


def test_join_unknown_blueprint_falls_back_to_default(client, world):
    r = _join(client, vehicle_blueprint="vehicle.evil.tank")
    assert r.status_code == 200
    sid = r.json()["session_id"]
    assert world.sessions.get_session(sid).vehicle_blueprint == client.get("/client/blueprints").json()["default"]


def test_join_spawn_failure_leaves_no_session(client, world):
    world.spawn_ok = False
    assert _join(client).status_code == 500
    assert world.sessions.all_sessions() == []


def test_leave_unknown_session_is_404(client, world):
    assert client.delete("/client/leave/does-not-exist").status_code == 404


def test_multiple_clients_get_distinct_sessions_and_actors(client, world):
    a, b = _join(client, "a").json(), _join(client, "b").json()
    assert a["session_id"] != b["session_id"] and a["actor_id"] != b["actor_id"]
    assert len(client.get("/admin/sessions", headers=AUTH).json()) == 2


def test_kick_session(client, world):
    sid = _join(client).json()["session_id"]
    assert client.delete(f"/admin/sessions/{sid}", headers=AUTH).json() == {"ok": True, "kicked": sid}
    assert sid in world.despawned
    assert client.delete(f"/admin/sessions/{sid}x", headers=AUTH).status_code == 404


def test_kick_requires_auth(client, world):
    sid = _join(client).json()["session_id"]
    assert client.delete(f"/admin/sessions/{sid}").status_code == 422
    assert client.delete(f"/admin/sessions/{sid}", headers={"X-Admin-Token": "bad"}).status_code == 403


# --- websocket telemetry ------------------------------------------------

def test_ws_streams_telemetry_and_answers_ping(client, world):
    sid = _join(client).json()["session_id"]
    with client.websocket_connect(f"/ws/client/{sid}") as ws:
        first = ws.receive_json()
        assert first["type"] == "telemetry"
        assert {"actor_id", "x", "y", "z", "yaw", "speed_kmh", "frame", "v2x"} <= set(first["data"])
        ws.send_text("ping")
        types = {ws.receive_json()["type"] for _ in range(3)}
        assert "pong" in types


def test_ws_invalid_session_gets_error_and_close(client, world):
    with client.websocket_connect("/ws/client/not-a-session") as ws:
        assert ws.receive_json()["type"] == "error"


def test_ws_without_world_gets_error(client):
    with client.websocket_connect("/ws/client/whatever") as ws:
        assert ws.receive_json()["type"] == "error"


def test_ws_disconnect_ends_session_and_despawns(client, world):
    """Current behaviour (handoff C11, decision D4): a WS close ends the session at once."""
    sid = _join(client).json()["session_id"]
    with client.websocket_connect(f"/ws/client/{sid}") as ws:
        ws.receive_json()
    assert sid in world.despawned
    assert world.sessions.get_session(sid).is_active is False


def test_ws_expired_session_is_reported(client, world):
    sid = _join(client, duration_seconds=60).json()["session_id"]
    from datetime import datetime, timedelta, timezone
    world.sessions.get_session(sid).expires_at = datetime.now(timezone.utc) - timedelta(seconds=1)
    with client.websocket_connect(f"/ws/client/{sid}") as ws:
        assert ws.receive_json()["type"] == "session_expired"


# --- admin population / control ----------------------------------------

def test_traffic_and_pedestrians_lifecycle(client, world):
    assert client.post("/admin/traffic", json={"count": 7}, headers=AUTH).json() == {"requested": 7, "spawned": 7}
    assert client.post("/admin/pedestrians", json={"count": 3}, headers=AUTH).json() == {"requested": 3, "spawned": 3}
    assert client.get("/admin/status", headers=AUTH).json()["background_vehicles"] == 7
    assert client.delete("/admin/traffic", headers=AUTH).json() == {"removed": 7}
    assert client.delete("/admin/pedestrians", headers=AUTH).json() == {"removed": 3}


@pytest.mark.parametrize("path,payload", [
    ("/admin/traffic", {"count": 0}), ("/admin/traffic", {"count": 201}),
    ("/admin/pedestrians", {"count": 0}), ("/admin/pedestrians", {"count": 101}),
])
def test_population_validation(client, world, path, payload):
    assert client.post(path, json=payload, headers=AUTH).status_code == 422


def test_population_requires_world(client):
    assert client.post("/admin/traffic", json={"count": 1}, headers=AUTH).status_code == 503
    assert client.delete("/admin/traffic", headers=AUTH).json() == {"removed": 0}


def test_connect_validates_request(client):
    bad = client.post("/admin/connect", json={"carla_port": 0}, headers=AUTH)
    assert bad.status_code == 422
    assert client.post("/admin/connect", json={"tm_port": 80}, headers=AUTH).status_code == 422


def test_connect_requires_auth(client):
    assert client.post("/admin/connect", json={}).status_code == 422
    assert client.post("/admin/connect", json={}, headers={"X-Admin-Token": "bad"}).status_code == 403


def test_stop_without_world(client):
    assert client.post("/admin/stop", headers=AUTH).json()["ok"] is False


def test_stop_with_world_requests_shutdown(client, world):
    assert client.post("/admin/stop", headers=AUTH).json()["ok"] is True
    for _ in range(50):
        if world.stopped:
            break
        import time; time.sleep(0.05)
    assert world.stopped
