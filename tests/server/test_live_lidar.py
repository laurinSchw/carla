"""Phase 4 acceptance against a live CARLA server (handoff §6, Phase 4). Marker: carla.

Needs the same environment as test_deployed_server.py (SERVER_URL, ADMIN_TOKEN) plus CARLA_HOST/CARLA_PORT so the
test can also count sensor actors on the CARLA side:

    SERVER_URL=http://127.0.0.1:8010 ADMIN_TOKEN=... CARLA_HOST=192.168.0.104 CARLA_PORT=2002 \
        pytest tests/server/test_live_lidar.py -m carla -s
"""

import json
import os
import time

import pytest

pytestmark = pytest.mark.carla


@pytest.fixture(scope="module")
def sensor_count():
    carla = pytest.importorskip("carla")
    host = os.environ.get("CARLA_HOST")
    if not host:
        pytest.skip("CARLA_HOST not set: cannot count sensor actors on the CARLA side")
    client = carla.Client(host, int(os.environ.get("CARLA_PORT", "2000")))
    client.set_timeout(10.0)
    return lambda: len(client.get_world().get_actors().filter("sensor.lidar.*"))


def _join(world, name, lidar=True):
    status, j = world.json("POST", "/client/join", {"client_name": name, "enable_lidar": lidar})
    assert status == 200, j
    return j


def _sensor(world, sid):
    _, body = world.json("GET", f"/client/lidar/{sid}")
    return body["sensor"]


def _wait(cond, timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        v = cond()
        if v:
            return v
        time.sleep(0.1)
    return None


def _frames(world, j, seconds, params=""):
    sync = pytest.importorskip("websockets.sync.client")
    from src.lidar import protocol
    out, t0 = [], time.time()
    with sync.connect(world.ws_base + j["lidar_ws_url"] + params, open_timeout=10, max_size=None) as ws:
        while time.time() - t0 < seconds:
            try:
                msg = ws.recv(timeout=2)
            except TimeoutError:
                break
            if isinstance(msg, bytes):
                out.append(protocol.decode(msg)[0])
    return out


def test_sensor_reaches_active_then_reconfigures_32_to_64_channels_without_leaking_actors(live_world, sensor_count):
    baseline = sensor_count()
    j = _join(live_world, "pytest-reconf")
    sid = j["session_id"]
    try:
        assert _wait(lambda: (_sensor(live_world, sid) or {}).get("state") == "active"), _sensor(live_world, sid)
        info = _sensor(live_world, sid)
        assert info["config"]["channels"] == 32 and sensor_count() == baseline + 1

        status, body = live_world.json("PUT", f"/client/lidar/{sid}", {"channels": 64})
        assert status == 202 and body["sensor"]["state"] == "reconfiguring", body
        assert _wait(lambda: (lambda s: s and s["state"] == "active" and s["config"]["channels"] == 64)(_sensor(live_world, sid)))
        assert sensor_count() == baseline + 1, "reconfigure must replace the sensor, not add one"

        frames = _frames(live_world, j, 3.0)
        assert len(frames) >= 5, f"only {len(frames)} frames after reconfigure"
        ids = [h.frame for h in frames]
        assert ids == sorted(set(ids)), "frames must be unique and ordered"
        assert all(h.ego_actor_id == j["actor_id"] and h.sensor_frame for h in frames)
    finally:
        live_world.json("DELETE", f"/client/leave/{sid}")
    assert _wait(lambda: sensor_count() == baseline), f"sensor actors leaked: {sensor_count()} vs baseline {baseline}"


def test_streamed_frames_are_frames_the_server_ticked(live_world):
    j = _join(live_world, "pytest-frames")
    try:
        assert _wait(lambda: (_sensor(live_world, j["session_id"]) or {}).get("state") == "active")
        _, before = live_world.json("GET", "/admin/status", admin=True)
        frames = _frames(live_world, j, 2.0)
        _, after = live_world.json("GET", "/admin/status", admin=True)
        assert frames
        assert all(before["frame"] <= h.frame <= after["frame"] + 5 for h in frames), (before["frame"], after["frame"], [h.frame for h in frames][:5])
    finally:
        live_world.json("DELETE", f"/client/leave/{j['session_id']}")


def test_viewer_leaving_releases_the_sensor_after_keep_alive_is_not_needed_for_leave(live_world, sensor_count):
    baseline = sensor_count()
    j = _join(live_world, "pytest-leave")
    assert _wait(lambda: sensor_count() == baseline + 1)
    live_world.json("DELETE", f"/client/leave/{j['session_id']}")
    assert _wait(lambda: sensor_count() == baseline), "leaving must destroy the session's sensor"


def test_three_lidar_sessions_with_traffic_keep_the_simulation_ticking(live_world, sensor_count, record_property):
    """Handoff Phase 4: 3 LiDAR sessions at the default config, 50 vehicles + 20 pedestrians.

    The server does not expose per-tick durations, so this measures the achieved simulation rate from the frame
    counter instead (target: 20 FPS). The tick p95 itself is not measured here.
    """
    baseline = sensor_count()
    sessions = []
    try:
        for n in range(3):
            sessions.append(_join(live_world, f"pytest-load-{n}"))
        assert _wait(lambda: all((_sensor(live_world, s["session_id"]) or {}).get("state") == "active" for s in sessions), 30)
        assert live_world.json("POST", "/admin/traffic", {"count": 50}, admin=True, timeout=120)[0] == 200
        assert live_world.json("POST", "/admin/pedestrians", {"count": 20}, admin=True, timeout=120)[0] == 200
        time.sleep(2.0)
        _, a = live_world.json("GET", "/admin/status", admin=True)
        t0 = time.time()
        time.sleep(10.0)
        _, b = live_world.json("GET", "/admin/status", admin=True)
        fps = (b["frame"] - a["frame"]) / (time.time() - t0)
        record_property("sim_fps_3_lidar_70_actors", round(fps, 2))
        print(f"\nsimulation rate with 3 LiDAR sessions + 50 vehicles + 20 pedestrians: {fps:.1f} FPS (target 20)")
        assert fps >= 15.0, f"simulation slowed to {fps:.1f} FPS"
        assert sensor_count() == baseline + 3
        _, links = live_world.json("GET", "/admin/v2x/links?msg_type=BSM&limit=1", admin=True)
        print("V2X BSM links in the last update:", links["summary"])
    finally:
        for s in sessions:
            live_world.json("DELETE", f"/client/leave/{s['session_id']}")
        live_world.json("DELETE", "/admin/traffic", admin=True, timeout=120)
        live_world.json("DELETE", "/admin/pedestrians", admin=True, timeout=120)
    assert _wait(lambda: sensor_count() == baseline), "sensor actors leaked after the load test"
