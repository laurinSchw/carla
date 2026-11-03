"""LiDAR through the real PersistentCARLAServer tick loop, against the recording CARLA stub (Phase 4 acceptance).

* only the simulation thread talks to CARLA when sensors are created, reconfigured and destroyed
* every streamed frame is a frame that was ticked
* no sensor actor is left behind, whichever way it ends
"""

import importlib
import sys
import time

import pytest

from src.lidar import protocol
from src.lidar.config import LidarConfig, LidarLimits
from src.lidar.manager import LidarBudgetExceeded
from src.server.world_config import WorldConfig
from tests.unit.carla_stub import Recorder, make_carla_module

SIM = "carla-tick"


@pytest.fixture
def env(monkeypatch):
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    servers = []

    def make(**cfg):
        srv = pw.PersistentCARLAServer(WorldConfig(host="stub", **cfg))
        srv.start()
        servers.append(srv)
        return srv

    yield rec, make
    for s in servers:
        s.stop()
    sys.modules.pop("src.server.persistent_world", None)


def _join(srv, name="c1"):
    session = srv.sessions.create_session(name, "127.0.0.1", "vehicle.test.car0", None)
    actor_id = srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    return session, actor_id


def _wait(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def _live_sensors(rec):
    return [a for a in rec.actors.values() if a.type_id.startswith("sensor.lidar")]


def test_sensor_goes_active_and_streams_frames_that_were_ticked(env):
    rec, make = env
    srv = make()
    session, actor_id = _join(srv)
    info = srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert info["state"] == "requested" and info["actor_id"] == actor_id
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")

    seen, last = [], -1
    for _ in range(40):
        got = srv.get_client_lidar_frame(session.session_id, after_frame=last)
        if got:
            last = got[0]
            header, points = protocol.decode(got[1])
            seen.append(header)
            assert header.ego_actor_id == actor_id and header.sensor_frame
            assert header.location[2] == pytest.approx(2.4) and header.rotation[1] == 90.0
            assert len(points) == 200 and set(points["object_idx"]) == {actor_id}
        time.sleep(0.05)
    assert len(seen) >= 5
    assert all(h.frame <= rec.frame for h in seen)                       # nothing from the future
    assert [h.frame for h in seen] == sorted({h.frame for h in seen})    # no duplicates, in order


def test_only_the_sim_thread_touches_carla_for_the_whole_sensor_lifecycle(env):
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    mark = len(rec.calls)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    srv.submit(srv.enable_client_lidar, session.session_id, LidarConfig(channels=64)).result(5)
    assert _wait(lambda: (lambda i: i["state"] == "active" and i["config"]["channels"] == 64)(
        srv.get_client_lidar_info(session.session_id)))
    srv.submit(srv.disable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id) is None)
    foreign = [c for c in rec.calls[mark:] if c[1] != SIM]
    assert foreign == [], f"CARLA called from non-simulation threads: {foreign[:5]}"
    kinds = [c[0] for c in rec.calls[mark:]]
    assert kinds.count("sensor.listen") == 2 and kinds.count("sensor.destroy") == 2   # spawn, respawn
    assert _live_sensors(rec) == []


def test_reconfigure_32_to_64_channels_respawns_one_sensor_with_the_new_attributes(env):
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    first = _live_sensors(rec)[0]
    assert first.bp.attrs["channels"] == "32" and first.bp.attrs["rotation_frequency"] == "20.0"
    srv.submit(srv.enable_client_lidar, session.session_id, LidarConfig(channels=64)).result(5)
    assert _wait(lambda: (lambda i: i["state"] == "active" and i["config"]["channels"] == 64)(
        srv.get_client_lidar_info(session.session_id)))
    live = _live_sensors(rec)
    assert len(live) == 1 and live[0] is not first and live[0].bp.attrs["channels"] == "64"


def test_leaving_destroys_the_sensor_before_its_vehicle(env):
    rec, make = env
    srv = make()
    session, actor_id = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    sensor_id = _live_sensors(rec)[0].id
    mark = len(rec.calls)
    srv.submit(srv.despawn_client_vehicle, session.session_id).result(5)
    order = [(c[0], c[2]) for c in rec.calls[mark:] if c[0] in ("sensor.destroy", "actor.destroy")]
    assert order and order[0] == ("sensor.destroy", sensor_id)
    assert _live_sensors(rec) == [] and srv.get_client_lidar_info(session.session_id) is None


def test_over_budget_request_is_refused_and_creates_no_actor(env):
    rec, make = env
    srv = make(lidar=LidarLimits(max_sensors=1))
    s1, _ = _join(srv, "a")
    s2, _ = _join(srv, "b")
    srv.submit(srv.enable_client_lidar, s1.session_id).result(5)
    with pytest.raises(LidarBudgetExceeded):
        srv.submit(srv.enable_client_lidar, s2.session_id).result(5)
    assert _wait(lambda: len(_live_sensors(rec)) == 1)


def test_viewerless_sensor_is_auto_released_and_leaks_no_actor(env):
    rec, make = env
    srv = make(lidar=LidarLimits(keep_alive_s=0.3))
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id) is not None)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id) is None, timeout=5.0)
    assert _wait(lambda: _live_sensors(rec) == [])            # the actor goes one tick after the sensor stops counting
    assert any(e["state"] == "stopping" and "no viewers" in e["reason"] for e in srv.lidar.events)


def test_a_viewer_keeps_the_sensor_alive_past_the_keep_alive_time(env):
    rec, make = env
    srv = make(lidar=LidarLimits(keep_alive_s=0.3))
    session, _ = _join(srv)
    info = srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    srv.lidar.viewer_attached(info["sensor_id"])
    time.sleep(1.0)
    assert srv.get_client_lidar_info(session.session_id)["state"] == "active"
    srv.lidar.viewer_detached(info["sensor_id"])
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id) is None, timeout=5.0)


def test_stopping_the_server_leaves_no_sensor_actor(env):
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    srv.stop()
    assert _live_sensors(rec) == []


def test_frames_follow_the_carla_frame_id_not_a_wall_clock_counter(env):
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    got = []
    assert _wait(lambda: got.append(srv.get_client_lidar_frame(session.session_id)) or got[-1] is not None)
    header, _ = protocol.decode(got[-1][1])
    assert 1000 < header.frame <= rec.frame
    assert header.sim_time == pytest.approx(header.frame * 0.05, abs=0.2)


def test_status_reports_tick_and_phase_timings(env):
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    assert _wait(lambda: srv.get_status()["timing_ms"]["tick"].get("n", 0) >= 10)
    t = srv.get_status()["timing_ms"]
    for phase in ("tick", "world_tick", "lidar"):
        assert t[phase]["n"] > 0 and 0 <= t[phase]["p50"] <= t[phase]["p95"] <= t[phase]["max"], (phase, t[phase])
    assert t["tick"]["p95"] >= t["world_tick"]["p50"]          # a tick contains the world tick


def test_reapplying_the_same_settings_does_not_respawn_the_sensor(env):
    """Review finding: every Apply (even a double click or a stream-profile change) destroyed and respawned the sensor."""
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id, LidarConfig(channels=32)).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    for _ in range(3):
        srv.submit(srv.enable_client_lidar, session.session_id, LidarConfig(channels=32)).result(5)
    time.sleep(0.3)
    kinds = [c[0] for c in rec.calls]
    assert kinds.count("sensor.listen") == 1 and kinds.count("sensor.destroy") == 0
    assert srv.get_client_lidar_info(session.session_id)["state"] == "active"


def test_a_new_request_right_after_a_release_creates_a_fresh_sensor(env):
    """Review finding: PUT right after DELETE failed with a 400 until the old sensor was gone."""
    rec, make = env
    srv = make()
    session, _ = _join(srv)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    first = srv.get_client_lidar_info(session.session_id)["sensor_id"]
    srv.submit(srv.disable_client_lidar, session.session_id).result(5)
    again = srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    assert again["sensor_id"] != first
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id)["state"] == "active")
    assert _wait(lambda: len(_live_sensors(rec)) == 1)


def test_a_recorded_lidar_run_keeps_sensors_alive_without_viewers(env, tmp_path):
    from src.lidar.config import LidarLimits
    rec, make = env
    srv = make(lidar=LidarLimits(keep_alive_s=0.2), runs_dir=str(tmp_path / "runs"))
    session, _ = _join(srv)
    srv.submit(srv.start_run, "lidar run", True).result(5)
    srv.submit(srv.enable_client_lidar, session.session_id).result(5)
    time.sleep(1.2)                                                   # 6x the keep-alive, no viewer
    assert srv.get_client_lidar_info(session.session_id)["state"] == "active"
    srv.submit(srv.stop_run).result(5)
    assert _wait(lambda: srv.get_client_lidar_info(session.session_id) is None, timeout=4.0)
