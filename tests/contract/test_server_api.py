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

ADMIN_GETS = ["/admin/status", "/admin/sessions", "/admin/v2x/rsus", "/admin/v2x/vrus", "/admin/v2x/spat", "/admin/v2x/links", "/admin/runs"]


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


def test_v2x_links_endpoint_filters_and_summarises(client, world):
    from src.v2x.dsrc_channel import LinkResult

    def link(msg, delivered, cause=None, rx=1):
        return LinkResult(1, 0.5, 5, rx, msg, "I2V", 10.0, True, 33.0, 80.0, 0.0, -50.0, 35.0,
                          1.0, 1.0, 1.0, delivered, cause)

    world.v2x_network = type("Net", (), {"get_link_results": lambda self: [
        link("SPAT", True), link("MAP", False, "channel_or_mac"), link("SPAT", False, "out_of_awareness_range", rx=2),
    ]})()
    body = client.get("/admin/v2x/links", headers=AUTH).json()
    assert body["summary"] == {"total": 3, "by_outcome": {"delivered": 1, "channel_or_mac": 1, "out_of_awareness_range": 1}}
    spat = client.get("/admin/v2x/links?msg_type=SPAT&rx_id=2", headers=AUTH).json()
    assert [l["drop_cause"] for l in spat["links"]] == ["out_of_awareness_range"]
    assert len(client.get("/admin/v2x/links?limit=1", headers=AUTH).json()["links"]) == 1


# --- LiDAR on request ---------------------------------------------------

def _lidar_session(client, world, name="car-1", enable=True):
    j = _join(client, name, enable_lidar=enable).json()
    return j


def test_join_with_lidar_returns_a_ws_url_and_the_sensor_goes_active_after_ticks(client, world):
    j = _lidar_session(client, world)
    assert j["lidar_ws_url"] == f"/ws/lidar/{j['session_id']}" and j["lidar_error"] is None
    assert client.get(f"/client/lidar/{j['session_id']}").json()["sensor"]["state"] == "requested"
    world.tick_lidar(2)
    body = client.get(f"/client/lidar/{j['session_id']}").json()
    assert body["sensor"]["state"] == "active" and body["budget"]["sensors"] == 1


def test_join_still_works_when_the_lidar_budget_is_exhausted_and_says_why(client, world):
    _lidar_session(client, world, "a")
    _lidar_session(client, world, "b")
    j = _lidar_session(client, world, "c")
    assert j["session_id"] and j["lidar_ws_url"] is None
    assert j["lidar_error"]["error"] == "lidar_budget_exceeded" and j["lidar_error"]["max_sensors"] == 2


def test_put_lidar_reconfigures_and_is_rejected_over_budget_with_409(client, world):
    sid = _lidar_session(client, world)["session_id"]
    world.tick_lidar(2)
    r = client.put(f"/client/lidar/{sid}", json={"channels": 64, "points_per_second": 400000})
    assert r.status_code == 202 and r.json()["sensor"]["state"] == "reconfiguring"
    world.tick_lidar(2)
    info = client.get(f"/client/lidar/{sid}").json()["sensor"]
    assert info["state"] == "active" and info["config"]["channels"] == 64
    r = client.put(f"/client/lidar/{sid}", json={"points_per_second": 2000000})
    assert r.status_code == 409 and r.json()["error"] == "lidar_budget_exceeded"
    assert r.json()["requested_pps"] == 2000000 and r.json()["available_pps"] == 1000000


def test_put_lidar_with_an_invalid_config_is_422_and_wrong_rotation_is_rejected(client, world):
    sid = _lidar_session(client, world, enable=False)["session_id"]
    assert client.put(f"/client/lidar/{sid}", json={"channels": 0}).status_code == 422
    assert client.put(f"/client/lidar/{sid}", json={"noise_stddev": 0.1}).status_code == 422  # semantic sensor
    r = client.put(f"/client/lidar/{sid}", json={"rotation_frequency_hz": 10.0})
    assert r.status_code == 422 and r.json()["error"] == "lidar_invalid_config"


def test_put_lidar_can_create_a_sensor_after_joining_and_delete_releases_it(client, world):
    sid = _lidar_session(client, world, enable=False)["session_id"]
    assert client.get(f"/client/lidar/{sid}").json()["sensor"] is None
    assert client.put(f"/client/lidar/{sid}", json={}).status_code == 202
    world.tick_lidar(2)
    assert client.delete(f"/client/lidar/{sid}").json() == {"released": True}
    world.tick_lidar()
    assert client.get(f"/client/lidar/{sid}").json()["sensor"] is None
    assert client.delete(f"/client/lidar/{sid}").json() == {"released": False}


def test_leaving_the_session_destroys_its_sensor(client, world):
    sid = _lidar_session(client, world)["session_id"]
    world.tick_lidar(2)
    client.delete(f"/client/leave/{sid}")
    assert world.lidar.list_sensors() == []


def test_lidar_schema_lists_valid_attributes_per_sensor_type(client, world):
    schema = client.get("/client/lidar/schema").json()
    assert "noise_stddev" not in schema["sensor_types"]["semantic"]["attributes"]
    assert "noise_stddev" in schema["sensor_types"]["raycast"]["attributes"]
    assert schema["stream_point_buckets"] == [10000, 25000, 50000, "all"]
    tags = {t["id"]: t for t in schema["semantic_tags"]}
    assert tags[14]["name"] == "Car" and tags[12]["name"] == "Pedestrian" and len(tags[1]["color"]) == 3


def test_lidar_ws_streams_v3_frames_of_the_ticked_frame_and_counts_viewers(client, world):
    from src.lidar import protocol
    j = _lidar_session(client, world)
    world.tick_lidar(3)
    sensor_id = world.lidar.sensor_of_owner(j["session_id"])["sensor_id"]
    with client.websocket_connect(j["lidar_ws_url"] + "?max_points=10000") as ws:
        state = ws.receive_json()
        assert state["type"] == "state" and state["state"] == "active"
        header, points = protocol.decode(ws.receive_bytes())
        assert header.frame == world._frame and header.sensor_id == sensor_id and header.sensor_frame
        assert len(points) == 10000                                 # 25k per frame, stream profile caps it
        assert world.lidar._sensors[sensor_id].viewers == 1
    assert world.lidar._sensors[sensor_id].viewers == 0              # viewer left; sensor stays until keep-alive


def test_lidar_ws_refuses_sessions_without_a_sensor(client, world):
    sid = _lidar_session(client, world, enable=False)["session_id"]
    with client.websocket_connect(f"/ws/lidar/{sid}") as ws:
        assert ws.receive_json()["type"] == "error"


def test_lidar_endpoints_reject_unknown_sessions(client, world):
    assert client.get("/client/lidar/nope").status_code == 404
    assert client.put("/client/lidar/nope", json={}).status_code == 404


def test_lidar_viewer_scripts_are_served_as_javascript(client):
    for name in ("lidar-decoder.js", "lidar-view.js"):
        r = client.get(f"/static/js/{name}")
        assert r.status_code == 200 and "javascript" in r.headers["content-type"], name
    assert "LidarDecoder" in client.get("/static/js/lidar-decoder.js").text
    page = client.get("/client").text
    assert "/static/js/lidar-decoder.js" in page and 'id="lidarCanvas"' in page


# --- run recording --------------------------------------------------------

def test_run_recording_lifecycle_over_http(client, world):
    assert client.get("/admin/runs", headers=AUTH).json() == {"current": None, "runs": []}
    r = client.post("/admin/runs", json={"name": "science run", "record_links": True}, headers=AUTH)
    assert r.status_code == 201 and r.json()["run_id"].endswith("_science-run") and r.json()["closed"] is False
    assert client.post("/admin/runs", json={"name": "second"}, headers=AUTH).status_code == 409
    assert client.get("/admin/runs", headers=AUTH).json()["current"]["run_id"] == r.json()["run_id"]
    stopped = client.post("/admin/runs/current/stop", headers=AUTH)
    assert stopped.status_code == 200 and stopped.json()["complete"] is True
    assert client.post("/admin/runs/current/stop", headers=AUTH).status_code == 404
    listing = client.get("/admin/runs", headers=AUTH).json()
    assert listing["current"] is None and [x["name"] for x in listing["runs"]] == ["science run"]


def test_run_endpoints_require_the_admin_token_and_a_running_world(client, world):
    assert client.post("/admin/runs", json={"name": "x"}).status_code == 422
    assert client.post("/admin/runs", json={"name": "x"}, headers={"X-Admin-Token": "nope"}).status_code == 403
    assert client.post("/admin/runs", json={"name": ""}, headers=AUTH).status_code == 422       # name must not be empty
    world._running = False
    assert client.post("/admin/runs", json={"name": "x"}, headers=AUTH).status_code == 503


def test_connect_request_carries_seed_drivers_and_advisory_settings():
    from src.server.admin_routes import ConnectRequest
    from src.server.world_config import WorldConfig
    from src.traffic.advisory import AdvisoryConfig
    from src.traffic.drivers import DriverMix
    req = ConnectRequest(carla_host="h", seed=7, drivers_enabled=False, driver_mix={"cautious": 1, "normal": 0, "aggressive": 0},
                         advisory_enabled=True, advisory_trigger_level=2)
    cfg = WorldConfig(host="h", seed=req.seed, drivers_enabled=req.drivers_enabled, driver_mix=DriverMix(**req.driver_mix),
                      advisory=AdvisoryConfig(enabled=req.advisory_enabled, trigger_level=req.advisory_trigger_level))
    assert cfg.seed == 7 and cfg.drivers_enabled is False and cfg.advisory.enabled and cfg.advisory.trigger_level == 2
    assert cfg.driver_mix.weights() == [1.0, 0.0, 0.0]
