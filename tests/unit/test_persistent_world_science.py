"""Science-readiness through the real PersistentCARLAServer tick loop (CARLA stub): drivers, seeds, recording."""

import importlib
import json
import sys
import time
from pathlib import Path

import pytest

from src.server.world_config import WorldConfig
from src.traffic.advisory import AdvisoryConfig
from tests.unit.carla_stub import Recorder, make_carla_module

SIM = "carla-tick"


@pytest.fixture
def env(monkeypatch, tmp_path):
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    servers = []

    def make(**cfg):
        srv = pw.PersistentCARLAServer(WorldConfig(host="stub", runs_dir=str(tmp_path / "runs"), **cfg))
        srv.start()
        servers.append(srv)
        return srv

    yield rec, make, tmp_path
    for s in servers:
        s.stop()
    sys.modules.pop("src.server.persistent_world", None)


def _wait(cond, timeout=4.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def _tm_settings(rec, name):
    return [c[2] for c in rec.calls if c[0] == f"tm.{name}"]


def test_pedestrian_seed_is_set_at_start(env):
    rec, make, _ = env
    make(seed=123)
    seeds = [c for c in rec.calls if c[0] == "world.set_pedestrians_seed"]
    assert len(seeds) == 1 and seeds[0][2] == 123


def test_background_vehicles_get_a_seeded_profile_applied_through_the_traffic_manager(env):
    rec, make, _ = env
    srv = make()
    srv.submit(srv.spawn_background_traffic, 12).result(5)
    speeds = _tm_settings(rec, "vehicle_percentage_speed_difference")
    assert len(speeds) == 12 and len(set(s[1][0] for s in speeds)) > 3          # varied, not one global value
    assert len(srv._driver_of) == 12
    assert {c[1] for c in rec.calls if c[0].startswith("tm.vehicle_percentage")} == {SIM}   # only the sim thread
    assert len(_tm_settings(rec, "distance_to_leading_vehicle")) == 12


def test_same_seed_gives_the_same_driver_sequence_in_two_separate_servers(env):
    rec, make, _ = env
    a = make(seed=11)
    a.submit(a.spawn_background_traffic, 10).result(5)
    first = [p for p in a._driver_of.values()]
    a.submit(a.despawn_background_traffic).result(5)
    a.stop()
    sys.modules.pop("src.server.persistent_world", None)
    rec2 = Recorder()
    sys.modules["carla"] = make_carla_module(rec2)
    pw = importlib.import_module("src.server.persistent_world")
    b = pw.PersistentCARLAServer(WorldConfig(host="stub", seed=11))
    b.start()
    try:
        b.submit(b.spawn_background_traffic, 10).result(5)
        assert [p for p in b._driver_of.values()] == first
    finally:
        b.stop()


def test_drivers_can_be_switched_off(env):
    rec, make, _ = env
    srv = make(drivers_enabled=False)
    srv.submit(srv.spawn_background_traffic, 4).result(5)
    assert _tm_settings(rec, "vehicle_percentage_speed_difference") == [] and srv._driver_of == {}


def test_a_recorded_run_has_manifest_events_stations_and_stamped_rows(env):
    rec, make, tmp = env
    srv = make(seed=7)
    srv.submit(srv.spawn_background_traffic, 5).result(5)
    started = srv.submit(srv.start_run, "unit run").result(5)
    run_dir = Path(started["path"])
    assert run_dir.parent == tmp / "runs" and run_dir.name.endswith("_unit-run")
    session = srv.sessions.create_session("c", "127.0.0.1", "vehicle.test.car0", None)
    srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    assert _wait(lambda: srv.recorder.counts["stations.jsonl"] >= 3)
    manifest = srv.submit(srv.stop_run).result(5)

    assert manifest["complete"] is True and manifest["config"]["seed"] == 7
    assert manifest["seeds"]["drivers"] == 9 and manifest["fixed_delta_seconds"] == 0.05
    assert manifest["driver_profiles"]["aggressive"]["speed_pct"] == -25.0
    assert manifest["row_counts"]["stations.jsonl"] >= 3 and "realtime_pacing" in manifest["reproducibility"]
    on_disk = json.loads((run_dir / "manifest.json").read_text())
    assert on_disk["complete"] is True and on_disk["end_frame"] >= on_disk["start_frame"]

    events = [json.loads(l) for l in (run_dir / "events.jsonl").read_text().splitlines()]
    kinds = [e["kind"] for e in events]
    assert kinds[0] == "run_start" and kinds[-1] == "run_stop"
    assert kinds.count("existing_background_vehicle") == 5 and "client_join" in kinds
    assert all("frame" in e and "sim_time" in e for e in events)
    assert [e["frame"] for e in events] == sorted(e["frame"] for e in events)

    stations = [json.loads(l) for l in (run_dir / "stations.jsonl").read_text().splitlines()]
    frames = [s["frame"] for s in stations]
    assert frames == sorted(set(frames)) and all(b - a == 10 for a, b in zip(frames, frames[1:]))   # every 10th frame
    first = stations[-1]["stations"]
    bg = [s for s in first if s["kind"] == "background"]
    assert len(bg) == 5 and all({"x_m", "y_m", "z_m", "speed_mps", "driver", "v2x_equipped"} <= set(s) for s in bg)
    assert {s["driver"] for s in bg} <= {"cautious", "normal", "aggressive"}


def test_only_one_run_at_a_time_and_stop_without_a_run_is_harmless(env):
    rec, make, _ = env
    srv = make()
    assert srv.submit(srv.stop_run).result(5) is None
    srv.submit(srv.start_run, "a").result(5)
    from src.recording.recorder import RunAlreadyRecording
    with pytest.raises(RunAlreadyRecording, match="already"):
        srv.submit(srv.start_run, "b").result(5)
    status = srv.get_run_status()
    assert status["current"]["closed"] is False
    srv.submit(srv.stop_run).result(5)
    assert srv.get_run_status()["current"] is None and len(srv.get_run_status()["runs"]) == 1


def test_stopping_the_server_closes_an_open_run_with_a_complete_manifest(env):
    rec, make, _ = env
    srv = make()
    run = srv.submit(srv.start_run, "left open").result(5)
    time.sleep(0.4)
    srv.stop()
    m = json.loads((Path(run["path"]) / "manifest.json").read_text())
    assert m["complete"] is True and m["end_frame"] > m["start_frame"]


def test_links_are_recorded_only_when_asked_and_without_v2x_no_links_file_rows(env):
    rec, make, _ = env
    srv = make(v2x_enabled=False)
    run = srv.submit(srv.start_run, "no v2x").result(5)
    assert _wait(lambda: srv.recorder.counts["stations.jsonl"] >= 2)
    srv.submit(srv.stop_run).result(5)
    import gzip
    assert gzip.decompress((Path(run["path"]) / "v2x_links.jsonl.gz").read_bytes()) == b""


def test_advisory_slows_a_threatened_vehicle_then_restores_its_own_speed(env):
    rec, make, _ = env
    srv = make(advisory=AdvisoryConfig(enabled=True, trigger_level=3, extra_slowdown_pct=40.0, hold_s=1.0))
    srv.submit(srv.spawn_background_traffic, 2).result(5)
    victim = srv._background_vehicles[0]
    base = srv._driver_of[victim.id].speed_pct
    srv.submit(srv.start_run, "adv").result(5)

    class Net:
        vehicles = {victim.id: victim}
        level = 4

        def get_threats(self, vid):
            return [{"level": Net.level}]

    srv.v2x_network = Net()
    srv.submit(lambda: srv._advisory_step(10.0)).result(5)
    Net.level = 0
    srv.submit(lambda: srv._advisory_step(11.5)).result(5)
    applied = [c[2][1][0] for c in rec.calls if c[0] == "tm.vehicle_percentage_speed_difference" and c[2][0] == victim.id]
    assert applied[-2:] == [base + 40.0, base]
    events = [e for e in (json.loads(l) for l in (Path(srv.recorder.path) / "events.jsonl").read_text().splitlines()) if e["kind"] == "advisory"]
    assert [e["action"] for e in events] == ["slow", "restore"] and events[0]["threat_level"] == 4
