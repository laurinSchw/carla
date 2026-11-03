"""/api/lidar/*, /api/sessions/{id}/lidar and WS /ws/lidar/{sensor_id} against the real tick loop and the recording CARLA stub.

Covers the role rules (admin: any vehicle of this world; vehicle: only its own; observer: read and view only, and only
while ``observer_public``), that credentials work as headers and as the first WebSocket message, that the published
owner is the client name (never the session id), and that the binary stream carries protocol-v3 frames of the sensor.
"""

import importlib
import json
import sys

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")
from fastapi.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

from src.lidar import protocol  # noqa: E402
from src.server.world_config import WorldConfig  # noqa: E402
from tests.unit.carla_stub import Recorder, make_carla_module  # noqa: E402
from tests.unit.test_hub_views import _wait  # noqa: E402

ADMIN = {"X-Admin-Token": "tok"}
SMALL = {"points_per_second": 100_000}


@pytest.fixture
def api(monkeypatch, tmp_path):
    monkeypatch.setenv("ADMIN_TOKEN", "tok")
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    import src.server.app as app_mod
    import src.server.lidar_routes as lidar_routes
    monkeypatch.setattr(lidar_routes, "AUTH_TIMEOUT_S", 0.3)
    srv = pw.PersistentCARLAServer(WorldConfig(host="stub", runs_dir=str(tmp_path / "runs"), lidar_max_sensors=3,
                                               lidar_max_points_per_second=1_000_000))
    srv.start()
    app_mod.set_persistent_world(srv)
    _wait(lambda: srv.get_world_state())
    yield TestClient(app_mod.app), srv, rec
    app_mod.set_persistent_world(None)
    srv.stop()
    sys.modules.pop("src.server.persistent_world", None)


def _join(c, name="car-1"):
    r = c.post("/client/join", json={"client_name": name})
    assert r.status_code == 200, r.text
    j = r.json()
    return j, {"X-Session-Token": j["session_token"]}


def _state(srv, sensor_id):
    info = srv.lidar.get_info(sensor_id)
    return info["state"] if info else None


def _live_sensor_actors(rec):
    return [a for a in rec.sensors if a.listening]


# ── REST ──────────────────────────────────────────────────────────────────────────────────────────

def test_schema_and_sensor_list_are_public_reads(api):
    c, srv, _ = api
    schema = c.get("/api/lidar/schema").json()
    assert schema["protocol_version"] == 3 and set(schema["sensor_types"]) == {"ray_cast_semantic", "ray_cast"}
    assert schema["limits"]["max_sensors"] == 3 and schema["limits"]["fixed_delta_seconds"] == 0.05
    body = c.get("/api/lidar/sensors").json()
    assert body["sensors"] == [] and body["budget"]["max_sensors"] == 3
    assert c.get("/api/lidar/sensors/99").status_code == 404
    assert c.get("/api/lidar/sensors", headers={"X-Admin-Token": "wrong"}).status_code == 403
    assert c.get("/api/lidar/sensors", headers={"X-Session-Token": "nope"}).status_code == 401


def test_vehicle_requests_reconfigures_and_releases_its_own_sensor(api):
    c, srv, rec = api
    j, me = _join(c, "car-1")
    sid = j["session_id"]
    _, other = _join(c, "car-2")
    assert c.post(f"/api/sessions/{sid}/lidar", json=SMALL).status_code == 403          # observer
    assert c.post(f"/api/sessions/{sid}/lidar", json=SMALL, headers=other).status_code == 403   # another session
    r = c.post(f"/api/sessions/{sid}/lidar", json=SMALL, headers=me)
    assert r.status_code == 202, r.text
    sensor_id = r.json()["sensor_id"]
    assert r.json()["ws_url"] == f"/ws/lidar/{sensor_id}" and r.json()["sensor"]["actor_id"] == j["actor_id"]
    listed = c.get("/api/lidar/sensors")
    assert listed.json()["sensors"][0]["owner"] == "car-1"
    assert sid not in listed.text and j["session_token"] not in listed.text              # credential-adjacent: never published
    assert c.get(f"/api/sessions/{sid}/lidar", headers=me).json()["sensor"]["sensor_id"] == sensor_id
    assert c.get(f"/client/lidar/{sid}").json()["sensor"]["sensor_id"] == sensor_id     # the old route sees the same sensor
    # one sensor per session through the generic route
    dup = c.post("/api/lidar/sensors", json={"config": SMALL}, headers=me)
    assert dup.status_code == 409 and dup.json()["error"] == "lidar_sensor_exists"
    # a vehicle may not use another actor, pin, or record raw
    assert c.post("/api/lidar/sensors", json={"actor_id": j["actor_id"] + 1000}, headers=other).status_code == 403
    assert c.post("/api/lidar/sensors", json={"pinned": True}, headers=other).status_code == 403
    assert c.post("/api/lidar/sensors", json={"record_raw": True}, headers=other).status_code == 403

    assert _wait(lambda: _state(srv, sensor_id) == "active")
    same = c.put(f"/api/lidar/sensors/{sensor_id}", json=SMALL, headers=me)
    assert same.status_code == 202 and same.json()["state"] == "active"                  # unchanged settings: no respawn
    assert c.put(f"/api/lidar/sensors/{sensor_id}", json={"channels": 64, **SMALL}, headers=other).status_code == 403
    r = c.put(f"/api/lidar/sensors/{sensor_id}", json={"channels": 64, **SMALL}, headers=me)
    assert r.status_code == 202 and r.json()["state"] == "reconfiguring"
    assert _wait(lambda: _state(srv, sensor_id) == "active")
    assert srv.lidar.get_info(sensor_id)["config"]["channels"] == 64
    assert c.put(f"/api/lidar/sensors/{sensor_id}", json={"channels": 0}, headers=me).status_code == 422

    assert c.delete(f"/api/lidar/sensors/{sensor_id}", headers=other).status_code == 403
    r = c.delete(f"/api/lidar/sensors/{sensor_id}", headers=me)
    assert r.status_code == 202 and r.json()["state"] == "stopping"
    assert _wait(lambda: srv.lidar.get_info(sensor_id) is None)
    assert _wait(lambda: not _live_sensor_actors(rec))
    assert c.delete(f"/api/lidar/sensors/{sensor_id}", headers=me).status_code == 404


def test_session_route_put_and_delete(api):
    c, srv, _ = api
    j, me = _join(c)
    sid = j["session_id"]
    r = c.put(f"/api/sessions/{sid}/lidar", json=SMALL, headers=me)       # PUT without a sensor requests one
    assert r.status_code == 202
    assert c.delete(f"/api/sessions/{sid}/lidar", headers=ADMIN).json() == {"released": True}
    assert c.get("/api/sessions/nope/lidar", headers=ADMIN).status_code == 404


def test_admin_attaches_pins_and_the_sensor_goes_with_its_vehicle(api):
    c, srv, rec = api
    j, me = _join(c)
    assert c.post("/api/lidar/sensors", json={"actor_id": j["actor_id"]}).status_code == 403      # observer
    assert c.post("/api/lidar/sensors", json={"actor_id": 424242}, headers=ADMIN).status_code == 404
    assert c.post("/api/lidar/sensors", json={}, headers=ADMIN).status_code == 404                # actor_id required
    r = c.post("/api/lidar/sensors", json={"actor_id": j["actor_id"], "config": SMALL, "pinned": True}, headers=ADMIN)
    assert r.status_code == 202, r.text
    sensor_id = r.json()["sensor_id"]
    info = srv.lidar.get_info(sensor_id)
    assert info["owner"] == "admin" and info["pinned"] is True
    # the vehicle sees the sensor on its actor but may not change or release it
    assert c.delete(f"/api/lidar/sensors/{sensor_id}", headers=me).status_code == 403
    assert c.patch(f"/api/lidar/sensors/{sensor_id}", json={"pinned": False}, headers=me).status_code == 403
    r = c.patch(f"/api/lidar/sensors/{sensor_id}", json={"pinned": False, "record_raw": True}, headers=ADMIN)
    assert r.status_code == 200 and r.json()["pinned"] is False and r.json()["record_raw"] is True
    # budget: 1,000,000 points/s in total
    over = c.post("/api/lidar/sensors", json={"actor_id": j["actor_id"], "config": {"points_per_second": 950_000}},
                  headers=ADMIN)
    assert over.status_code == 409 and over.json()["error"] == "lidar_budget_exceeded"
    assert over.json()["reason"] == "points_per_second" and over.json()["available_pps"] == 900_000
    assert _wait(lambda: _state(srv, sensor_id) == "active")
    # leaving destroys the admin's sensor on that vehicle before the vehicle
    assert c.delete(f"/client/leave/{j['session_id']}").status_code == 200
    assert srv.lidar.get_info(sensor_id) is None and not _live_sensor_actors(rec)


def test_observer_public_off_needs_a_credential(api):
    c, srv, _ = api
    j, me = _join(c)
    assert c.patch("/api/config", json={"observer_public": False}, headers=ADMIN).status_code == 200
    assert c.get("/api/lidar/sensors").status_code == 403
    assert c.get("/api/lidar/schema").status_code == 403
    assert c.get("/api/lidar/sensors", headers=me).status_code == 200
    assert c.get("/api/lidar/schema", headers=ADMIN).status_code == 200


# ── WebSocket ─────────────────────────────────────────────────────────────────────────────────────

def _sensor(c, srv, actor_id, **extra):
    r = c.post("/api/lidar/sensors", json={"actor_id": actor_id, "config": SMALL, **extra}, headers=ADMIN)
    assert r.status_code == 202, r.text
    sensor_id = r.json()["sensor_id"]
    assert _wait(lambda: _state(srv, sensor_id) == "active")
    return sensor_id


def _next_frame(ws, timeout_msgs=200):
    texts = []
    for _ in range(timeout_msgs):
        m = ws.receive()
        if m.get("bytes") is not None:
            return protocol.decode(m["bytes"]), texts
        texts.append(m["text"])
    raise AssertionError(f"no binary frame, got {texts[-5:]}")


def _text_where(ws, pred, max_msgs=300):
    for _ in range(max_msgs):
        m = ws.receive()
        if m.get("text") and pred(json.loads(m["text"])):
            return json.loads(m["text"])
    raise AssertionError("no matching text message")


def test_ws_streams_protocol_v3_frames_of_the_sensor(api):
    c, srv, _ = api
    j, _ = _join(c)
    sensor_id = _sensor(c, srv, j["actor_id"])
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:                        # observer, observer_public on
        (header, points), texts = _next_frame(ws)
        assert any(json.loads(t)["type"] == "state" for t in texts)
        assert header.sensor_id == sensor_id and header.ego_actor_id == j["actor_id"] and len(points) > 0
        assert _wait(lambda: srv.lidar.get_info(sensor_id)["viewers"] == 1)          # published at the next tick
        ws.send_json({"op": "profile", "max_points": 10_000, "max_hz": 5, "compress": True})
        ws.send_text("ping")
        seen = []
        for _ in range(300):
            m = ws.receive()
            if m.get("text"):
                seen.append(json.loads(m["text"]))
                if seen[-1]["type"] == "stream_profile" and seen[-1]["compress"] and any(x["type"] == "pong" for x in seen):
                    break
        prof = seen[-1]
        assert prof["type"] == "stream_profile" and prof["compress"] is True
        assert prof["every_n_frames"] == 4 and prof["max_hz"] == 5.0 and prof["max_points"] == 10_000   # 20 FPS / 5 Hz; bucket 10k
        assert prof["points_total"] == 200 and prof["points_sent"] == 200 and prof["subsampled"] is False  # the stub has 200 points
        ws.send_json({"op": "profile", "max_hz": -1})
        assert _text_where(ws, lambda x: x["type"] == "error")["code"] == "bad_message"
    assert _wait(lambda: srv.lidar.get_info(sensor_id)["viewers"] == 0)
    # the sensor ends: the viewer is told and the socket closes
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:
        _next_frame(ws)
        assert c.delete(f"/api/lidar/sensors/{sensor_id}", headers=ADMIN).status_code == 202
        states = []
        with pytest.raises(WebSocketDisconnect) as closed:
            while True:
                m = ws.receive()
                if m["type"] == "websocket.close":
                    raise WebSocketDisconnect(m.get("code", 1000))
                if m.get("text") and json.loads(m["text"])["type"] == "state":
                    states.append(json.loads(m["text"])["state"])
        assert closed.value.code == 1000 and states[-1] == "destroyed"


def test_ws_unknown_sensor_and_wrong_credentials(api):
    c, srv, _ = api
    j, _ = _join(c)
    sensor_id = _sensor(c, srv, j["actor_id"])
    with c.websocket_connect("/ws/lidar/999") as ws:
        assert ws.receive_json()["code"] == "not_found"
        with pytest.raises(WebSocketDisconnect) as e:
            ws.receive_text()
        assert e.value.code == 4404
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:
        ws.send_json({"op": "auth", "admin_token": "wrong"})
        for _ in range(300):
            m = ws.receive()
            if m["type"] == "websocket.close":
                break
        assert m["code"] == 4401
    with c.websocket_connect(f"/ws/lidar/{sensor_id}", headers={"X-Session-Token": "nope"}) as ws:
        assert ws.receive_json()["code"] == "auth_failed"


def test_ws_observer_public_off_needs_the_auth_message(api):
    c, srv, _ = api
    j, me = _join(c, "car-1")
    _, other = _join(c, "car-2")
    sensor_id = _sensor(c, srv, j["actor_id"])
    c.patch("/api/config", json={"observer_public": False}, headers=ADMIN)
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:                         # no auth within the timeout
        msg = ws.receive_json()
        assert msg["type"] == "error" and msg["code"] == "forbidden"
        with pytest.raises(WebSocketDisconnect) as e:
            ws.receive_text()
        assert e.value.code == 4403
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:                         # admin via the first message
        ws.send_json({"op": "auth", "admin_token": "tok"})
        (header, _), texts = _next_frame(ws)
        assert header.sensor_id == sensor_id and any(json.loads(t)["type"] == "auth" for t in texts)
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:                         # the vehicle the sensor is on
        ws.send_json({"op": "auth", "session_token": me["X-Session-Token"]})
        (header, _), _ = _next_frame(ws)
        assert header.ego_actor_id == j["actor_id"]
    with c.websocket_connect(f"/ws/lidar/{sensor_id}") as ws:                         # another vehicle: not its sensor
        ws.send_json({"op": "auth", "session_token": other["X-Session-Token"]})
        for _ in range(300):
            m = ws.receive()
            if m["type"] == "websocket.close":
                break
        assert m["code"] == 4403


def test_legacy_session_socket_shares_the_path(api):
    c, srv, _ = api
    r = c.post("/client/join", json={"client_name": "old", "enable_lidar": True})
    j = r.json()
    assert j["lidar_ws_url"] == f"/ws/lidar/{j['session_id']}"
    info = srv.lidar.sensor_of_owner(j["session_id"])
    assert info["owner"] == "old"
    assert _wait(lambda: _state(srv, info["sensor_id"]) == "active")
    with c.websocket_connect(j["lidar_ws_url"] + "?max_points=10000") as ws:
        (header, _), _ = _next_frame(ws)
        assert header.sensor_id == info["sensor_id"]


def test_admin_sensor_on_background_vehicle_goes_before_the_vehicle(api):
    c, srv, rec = api
    assert srv.submit(srv.spawn_background_traffic, 2).result(5) == 2
    actor_id = srv._background_vehicles[0].id
    sensor_id = _sensor(c, srv, actor_id)
    srv.submit(srv.despawn_background_traffic).result(5)
    assert srv.lidar.get_info(sensor_id) is None and not _live_sensor_actors(rec)
