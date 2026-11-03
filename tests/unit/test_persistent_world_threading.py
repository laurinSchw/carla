"""Threading rules of PersistentCARLAServer, checked against a recording stub of ``carla``.

Phase 2 invariants (REFACTOR_HANDOFF section 4):
  * after start() only the simulation thread ("carla-tick") calls CARLA        (C1-C5)
  * nothing ticks the world behind the tick loop's back, _frame == CARLA frame (C1, C2)
  * published telemetry carries the frame it belongs to, readers make no RPC   (C10)
  * same seed -> same spawn choices                                            (C8)
"""

import importlib
import sys
import threading
import time

import pytest

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


def _calls_after(rec, mark):
    return rec.calls[mark:]


def _join(srv, name="c1"):
    session = srv.sessions.create_session(name, "127.0.0.1", "vehicle.test.car0", None)
    actor_id = srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    return session, actor_id


def test_only_the_sim_thread_calls_carla_after_start(env):
    rec, make = env
    srv = make()
    mark = len(rec.calls)

    srv.submit(srv.spawn_background_traffic, 5).result(5)
    srv.submit(srv.spawn_pedestrians, 3).result(5)
    session, actor_id = _join(srv)
    # readers on this (non-sim) thread
    srv.get_status(); srv.get_vehicle_telemetry(session.session_id); srv.get_vehicle_v2x_data(session.session_id)
    time.sleep(0.2)
    srv.submit(srv.despawn_client_vehicle, session.session_id).result(5)
    srv.submit(srv.despawn_pedestrians).result(5)
    srv.submit(srv.despawn_background_traffic).result(5)

    foreign = [c for c in _calls_after(rec, mark) if c[1] != SIM]
    assert foreign == [], f"CARLA called from non-simulation threads: {foreign[:5]}"
    assert any(c[0] == "world.tick" for c in _calls_after(rec, mark))


def test_mutating_methods_refuse_foreign_threads(env):
    rec, make = env
    srv = make()
    for fn, args in [(srv.spawn_background_traffic, (1,)), (srv.despawn_background_traffic, ()),
                     (srv.spawn_pedestrians, (1,)), (srv.despawn_pedestrians, ()),
                     (srv.despawn_client_vehicle, ("x",)), (srv.enable_client_lidar, ("x",))]:
        with pytest.raises(RuntimeError, match="simulation thread"):
            fn(*args)


def test_no_batch_ticks_the_world_and_frame_matches_carla(env):
    rec, make = env
    srv = make()
    srv.submit(srv.spawn_background_traffic, 4).result(5)
    srv.submit(srv.spawn_pedestrians, 2).result(5)   # needs one explicit tick for controllers
    _join(srv)
    srv.submit(srv.despawn_pedestrians).result(5)
    srv.submit(srv.despawn_background_traffic).result(5)
    batches = [c for c in rec.calls if c[0] == "client.apply_batch_sync"]
    assert batches and not any(c[2]["do_tick"] for c in batches), "a batch ticked the world (C1)"
    assert not any(c[0] == "batch.implicit_tick" for c in rec.calls)
    srv.submit(lambda: None).result(5)
    time.sleep(0.15)
    # _frame is CARLA's frame id (no drift from the extra pedestrian tick, C2); the stub's
    # frame may be one tick ahead if the loop ticked between our reads.
    assert 0 <= rec.frame - srv._frame <= 2


def test_telemetry_is_stamped_with_the_snapshot_frame_and_needs_no_rpc(env):
    rec, make = env
    srv = make()
    session, actor_id = _join(srv)
    first = srv.get_vehicle_telemetry(session.session_id)
    assert first and first["actor_id"] == actor_id  # available immediately after join
    time.sleep(0.3)
    mark = len(rec.calls)
    tele = srv.get_vehicle_telemetry(session.session_id)
    assert tele["frame"] > first["frame"], "telemetry is not refreshed every frame"
    assert tele["frame"] <= rec.frame and tele["speed_kmh"] == 18.0  # (3,4,0) m/s
    assert [c for c in rec.calls[mark:] if c[1] != SIM] == []


def test_submit_after_stop_fails_instead_of_touching_carla(env):
    rec, make = env
    srv = make()
    srv.stop()
    with pytest.raises(RuntimeError):
        srv.submit(srv.spawn_background_traffic, 1).result(5)


def test_stop_destroys_everything_on_the_sim_thread_and_restores_settings(env):
    rec, make = env
    srv = make()
    srv.submit(srv.spawn_background_traffic, 3).result(5)
    _join(srv)
    mark = len(rec.calls)
    srv.stop()
    after = rec.calls[mark:]
    assert any(c[0] == "client.apply_batch_sync" and c[1] == SIM for c in after), "no cleanup batch on sim thread"
    assert any(c[0] == "world.apply_settings" for c in after), "world settings not restored"
    assert rec.actors == {}, f"actors left behind: {list(rec.actors)}"
    assert not threading.enumerate() or SIM not in [t.name for t in threading.enumerate()]


def test_same_seed_gives_same_spawn_choices(env):
    rec, make = env

    def run(seed):
        before = set(rec.actors)
        srv = make(seed=seed)
        srv.submit(srv.spawn_background_traffic, 6).result(5)
        spawned = sorted(
            (a.transform.location.x, a.bp.id) for i, a in rec.actors.items() if i not in before
        )
        srv.stop()
        return spawned

    assert run(7) == run(7)
    assert run(7) != run(8)


def test_expired_session_is_cleaned_up_by_the_sim_thread(env):
    rec, make = env
    srv = make()
    session = srv.sessions.create_session("tmp", "127.0.0.1", "vehicle.test.car0", 10)
    srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    session.expires_at = session.spawn_time  # already expired
    mark = len(rec.calls)
    deadline = time.time() + 10
    while time.time() < deadline and session.is_active:
        time.sleep(0.05)
    assert not session.is_active, "expired session was never ended"
    assert session.session_id not in srv._client_vehicles
    assert [c for c in rec.calls[mark:] if c[1] != SIM] == []


def test_v2x_failure_does_not_stop_the_simulation(env):
    rec, make = env
    srv = make()

    def boom(snapshot, **kw):
        raise ValueError("v2x exploded")

    srv.v2x_network.update = boom
    f0 = srv._frame
    time.sleep(1.0)  # > 10 frames at 20 FPS: the V2X update runs, fails, loop carries on
    assert srv._tick_thread.is_alive() and srv._running
    assert srv._frame > f0 + 10
    assert srv.submit(lambda: 42).result(5) == 42
