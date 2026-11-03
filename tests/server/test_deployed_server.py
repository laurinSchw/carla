"""Smoke and end-to-end tests against a running server. See conftest.py for configuration.

    SERVER_URL=http://HOST:8000 ADMIN_TOKEN=... pytest tests/server -v
    (+ CARLA_HOST=... CARLA_PORT=2001 to run the simulation tests)
"""

import json
import time

import pytest

# ---------------------------------------------------------------- reachable


def test_root_page_ok(server):
    status, ctype, body = server.request("GET", "/")
    assert status == 200 and "text/html" in ctype and b"CARLA" in body


@pytest.mark.parametrize("path", ["/admin", "/client"])
def test_pages_served(server, path):
    status, ctype, body = server.request("GET", path)
    assert status == 200 and "text/html" in ctype and len(body) > 500


def test_openapi_lists_expected_routes(server):
    status, spec = server.json("GET", "/openapi.json")
    assert status == 200
    paths = set(spec["paths"])
    assert {"/admin/status", "/admin/connect", "/client/join", "/client/blueprints"} <= paths


def test_removed_monolith_endpoints_are_gone(server):
    for path in ("/api/simulation/status", "/api/v2v/vehicles", "/v2v"):
        assert server.request("GET", path)[0] == 404


def test_blueprints_endpoint(server):
    status, body = server.json("GET", "/client/blueprints")
    assert status == 200 and body["default"] in body["blueprints"]


def test_server_survives_many_quick_requests(server):
    t0 = time.time()
    for _ in range(50):
        assert server.request("GET", "/client/blueprints")[0] == 200
    assert time.time() - t0 < 15


# ---------------------------------------------------------------- auth


def test_admin_without_token_rejected(server):
    assert server.request("GET", "/admin/status")[0] == 422


def test_admin_wrong_token_rejected(server):
    assert server.request("GET", "/admin/status", admin=True, token="definitely-wrong")[0] == 403
    assert server.request("DELETE", "/admin/traffic", admin=True, token="definitely-wrong")[0] == 403


def test_admin_status_shape(admin):
    status, body = admin.json("GET", "/admin/status", admin=True)
    assert status == 200
    assert {"running", "connected", "frame", "active_sessions"} <= set(body)


def test_join_validation_errors(server):
    assert server.json("POST", "/client/join", {})[0] == 422
    assert server.json("POST", "/client/join", {"client_name": ""})[0] == 422


# ---------------------------------------------------------------- simulation (needs CARLA)



@pytest.mark.carla
def test_world_is_running_and_ticking(live_world):
    _, a = live_world.json("GET", "/admin/status", admin=True)
    time.sleep(1.5)
    _, b = live_world.json("GET", "/admin/status", admin=True)
    assert a["running"] and b["running"]
    assert b["frame"] > a["frame"], "world frame counter is not advancing"
    assert b["map"]


@pytest.mark.carla
def test_client_join_telemetry_leave(live_world):
    status, j = live_world.json("POST", "/client/join", {"client_name": "pytest-e2e", "duration_seconds": 120})
    assert status == 200, j
    sid = j["session_id"]
    try:
        status, tele = live_world.json("GET", f"/client/vehicle/{sid}")
        assert status == 200 and tele["actor_id"] == j["actor_id"]
        assert {"x", "y", "z", "yaw", "speed_kmh", "frame"} <= set(tele)

        _, sessions = live_world.json("GET", "/admin/sessions", admin=True)
        assert sid in {s["session_id"] for s in sessions if s.get("is_active", True)}
    finally:
        live_world.json("DELETE", f"/client/leave/{sid}")
    assert live_world.json("GET", f"/client/vehicle/{sid}")[0] == 404


@pytest.mark.carla
def test_websocket_telemetry_stream(live_world, record_property):
    sync = pytest.importorskip("websockets.sync.client")
    status, j = live_world.json("POST", "/client/join", {"client_name": "pytest-ws"})
    assert status == 200, j
    sid = j["session_id"]
    frames = []
    try:
        with sync.connect(live_world.ws_base + j["ws_url"], open_timeout=10) as ws:
            t0 = time.time()
            while time.time() - t0 < 4.0:
                try:
                    msg = json.loads(ws.recv(timeout=2))
                except TimeoutError:
                    break
                if msg["type"] == "telemetry":
                    frames.append((time.time(), msg["data"]["frame"]))
            ws.send("ping")
    finally:
        live_world.json("DELETE", f"/client/leave/{sid}")
    assert len(frames) >= 4, f"only {len(frames)} telemetry frames in 4 s"
    rate = (len(frames) - 1) / (frames[-1][0] - frames[0][0])
    record_property("telemetry_hz", round(rate, 2))
    print(f"\nmeasured telemetry rate: {rate:.2f} Hz (documented 5 Hz)")
    assert rate >= 4.0, f"telemetry only {rate:.2f} Hz; documented 5 Hz (defect C6 was 2.5 Hz)"
    assert frames[-1][1] >= frames[0][1], "simulation frame went backwards"


@pytest.mark.carla
def test_websocket_close_cleans_up_session(live_world):
    sync = pytest.importorskip("websockets.sync.client")
    _, j = live_world.json("POST", "/client/join", {"client_name": "pytest-cleanup"})
    sid = j["session_id"]
    with sync.connect(live_world.ws_base + j["ws_url"], open_timeout=10) as ws:
        ws.recv(timeout=5)
    for _ in range(30):
        if live_world.json("GET", f"/client/vehicle/{sid}")[0] == 404:
            break
        time.sleep(0.2)
    else:
        live_world.json("DELETE", f"/client/leave/{sid}")
        pytest.fail("session still active 6 s after the websocket closed")


@pytest.mark.carla
def test_no_leaked_sessions_after_tests(live_world):
    _, sessions = live_world.json("GET", "/admin/sessions", admin=True)
    leaked = [s for s in sessions if s.get("client_name", "").startswith("pytest-") and s.get("is_active", True)]
    assert not leaked, f"test sessions left active: {leaked}"


@pytest.mark.carla
def test_admin_population_roundtrip(live_world):
    status, body = live_world.json("POST", "/admin/traffic", {"count": 3}, admin=True, timeout=60)
    try:
        assert status == 200 and body["spawned"] >= 1, body
    finally:
        # always clean up so a failing run does not leave vehicles in the simulation
        cstatus, cbody = live_world.json("DELETE", "/admin/traffic", admin=True, timeout=60)
    assert cstatus == 200 and cbody["removed"] >= 1, cbody


@pytest.mark.carla
def test_lidar_websocket_streams_decodable_frames(live_world, record_property):
    """Join with LiDAR, read /ws/lidar/{session}, decode frames with BinaryProtocol."""
    sync = pytest.importorskip("websockets.sync.client")
    from src.utils.binary_protocol import BinaryProtocol

    status, j = live_world.json("POST", "/client/join",
                                {"client_name": "pytest-lidar", "enable_lidar": True})
    assert status == 200, j
    sid = j["session_id"]
    if not j.get("lidar_ws_url"):
        live_world.json("DELETE", f"/client/leave/{sid}")
        pytest.fail("join returned no lidar_ws_url: the server could not enable LiDAR (check server logs)")
    frames, sizes, points = 0, [], []
    try:
        with sync.connect(live_world.ws_base + j["lidar_ws_url"], open_timeout=10, max_size=None) as ws:
            t0 = time.time()
            while time.time() - t0 < 6.0 and frames < 20:
                try:
                    msg = ws.recv(timeout=2)
                except TimeoutError:
                    break
                if isinstance(msg, str):
                    assert json.loads(msg)["type"] != "error", msg
                    continue
                decoded = BinaryProtocol.decode(msg)
                assert decoded is not None, "binary LiDAR frame could not be decoded"
                pts, meta = decoded
                frames += 1
                sizes.append(len(msg))
                points.append(meta["num_points"])
                assert meta["ego_actor_id"] == j["actor_id"]
            elapsed = time.time() - t0
    finally:
        live_world.json("DELETE", f"/client/leave/{sid}")
    assert frames >= 3, f"only {frames} LiDAR frames in 6 s"
    record_property("lidar_frames", frames)
    print(f"\nLiDAR: {frames} frames in {elapsed:.1f} s, "
          f"avg {sum(points)/len(points):.0f} points, avg {sum(sizes)/len(sizes)/1024:.0f} kB/frame")
    assert max(points) > 0, "LiDAR frames contained no points"
