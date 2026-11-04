"""The hub reads what the simulation thread publishes: world state, V2X frame, stats, events, run status, session grace.

Runs the real PersistentCARLAServer tick loop against the recording CARLA stub, and the real WebSocket hub through the app.
"""

import concurrent.futures
import contextlib
import importlib
import sys
import time

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")
from fastapi.testclient import TestClient  # noqa: E402

from src.server.world_config import WorldConfig  # noqa: E402
from tests.unit.carla_stub import Recorder, make_carla_module  # noqa: E402


@contextlib.contextmanager
def _client(app):
    """A TestClient without the app lifespan: the lifespan stops the world, which the ``env`` fixture does anyway; entering it made
    the exit race with the stop under load (CancelledError in TestClient.__exit__)."""
    yield TestClient(app)


@contextlib.contextmanager
def _ws(client, url="/ws"):
    """``client.websocket_connect`` that tolerates a known Starlette TestClient teardown race: under CPU load the portal future
    of ``cs.cancel`` can already be cancelled when the socket closes (CancelledError from ``__exit__``). The server side is
    unaffected, the assertions of the tests are made inside the block."""
    cm = client.websocket_connect(url)
    ws = cm.__enter__()
    try:
        yield ws
    finally:
        try:
            cm.__exit__(None, None, None)
        except concurrent.futures.CancelledError:
            pass


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

    yield make
    import src.server.app as app_mod
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


def _join(srv, name="c1"):
    s = srv.sessions.create_session(name, "127.0.0.1", "vehicle.test.car0", None)
    srv.submit(srv.spawn_client_vehicle, s.session_id, "vehicle.test.car0").result(5)
    return s


def test_world_state_is_published_with_every_station_and_the_map(env):
    srv = env()
    srv.submit(srv.spawn_background_traffic, 3).result(5)
    _join(srv)
    pub = _wait(lambda: (p := srv.get_world_state()) and len(p.data["stations"]) >= 4 and p)
    assert pub and pub.data["map"] and pub.frame > 0 and pub.version >= 1
    kinds = {(s["kind"], s["role"]) for s in pub.data["stations"]}
    assert ("vehicle", "client") in kinds and ("vehicle", "background") in kinds
    assert all({"id", "x", "y", "z", "yaw", "speed_mps", "v2x"} <= set(s) for s in pub.data["stations"])


def test_a_v2x_frame_summary_and_stats_follow_each_update(env):
    srv = env()
    srv.submit(srv.spawn_background_traffic, 3).result(5)
    assert _wait(lambda: srv.get_v2x_frame() is not None)
    fr, summary, stats = srv.get_v2x_frame(), srv.get_v2x_published(), srv.get_v2x_stats()
    assert fr.frame == summary.frame == stats.frame and summary.data["stations"] is not None
    assert srv.get_v2x_frame().frame >= fr.frame


def test_run_status_reports_the_real_time_factor_and_tick_timing(env):
    srv = env()
    pub = _wait(lambda: srv.get_run_status_published())
    assert pub and pub.data["running"] is True and pub.data["fixed_delta_seconds"] == 0.05
    assert "tick_duration_ms" in pub.data and pub.data["background_vehicles"] == 0


def test_admin_actions_and_session_changes_become_events(env):
    srv = env()
    srv.submit(srv.spawn_background_traffic, 2).result(5)
    _join(srv)
    kinds = [e["kind"] for e in srv.events.tail(50)]
    assert "traffic_spawned" in kinds and "session_joined" in kinds
    assert all(e["frame"] is not None for e in srv.events.tail(50) if e["kind"] in ("traffic_spawned", "session_joined"))


def test_a_session_without_a_connection_is_removed_after_the_grace_period(env):
    srv = env(session_disconnect_grace_s=0.4)
    s = _join(srv)
    assert s.session_id in srv._client_vehicles
    srv.sessions.mark_connected(s.session_id)
    srv.sessions.mark_disconnected(s.session_id)
    assert _wait(lambda: s.session_id not in srv._client_vehicles, timeout=8.0), "grace never expired"
    assert any(e["kind"] == "session_disconnected_removed" for e in srv.events.tail(50))


def test_a_connected_session_is_never_removed_by_the_grace_rule(env):
    srv = env(session_disconnect_grace_s=0.3)
    s = _join(srv)
    srv.sessions.mark_connected(s.session_id)
    time.sleep(1.5)
    assert s.session_id in srv._client_vehicles


def test_hub_streams_world_state_to_an_observer_over_the_websocket(env):
    import src.server.app as app_mod
    srv = env()
    srv.submit(srv.spawn_background_traffic, 2).result(5)
    # a state published before the spawn has no stations; wait for one that does
    assert _wait(lambda: (w := srv.get_world_state()) and len(w.data["stations"]) == 2)
    app_mod.set_persistent_world(srv)
    with _client(app_mod.app) as client, _ws(client) as ws:
        ws.send_json({"op": "subscribe", "topic": "world.state", "max_hz": 5})
        seen = []
        for _ in range(6):
            msg = ws.receive_json()
            seen.append(msg["topic"])
            if msg["topic"] == "world.state":
                assert len(msg["data"]["stations"]) == 2 and msg["frame"] > 0 and msg["seq"] >= 1
                break
        assert "world.state" in seen


def test_the_hub_refuses_ground_truth_filters_and_other_vehicles_inboxes_to_a_vehicle(env):
    import src.server.app as app_mod
    srv = env()
    s = _join(srv)
    _wait(lambda: srv.get_world_state())
    app_mod.set_persistent_world(srv)
    with _client(app_mod.app) as client, _ws(client) as ws:
        ws.send_json({"op": "auth", "session_token": s.session_token})
        assert ws.receive_json()["topic"] == "auth"
        ws.send_json({"op": "subscribe", "topic": "world.state"})
        err = ws.receive_json()
        assert err["topic"] == "error" and err["code"] == "forbidden"
        ws.send_json({"op": "subscribe", "topic": "v2x.inbox", "filter": {"station_id": s.vehicle_actor_id + 999}})
        err = ws.receive_json()
        assert err["topic"] == "error" and err["code"] == "forbidden"


def test_a_wrong_token_closes_the_socket_with_4401(env):
    import src.server.app as app_mod
    from starlette.websockets import WebSocketDisconnect
    srv = env()
    app_mod.set_persistent_world(srv)
    with _client(app_mod.app) as client, _ws(client) as ws:
        ws.send_json({"op": "auth", "session_token": "nope"})
        with pytest.raises(WebSocketDisconnect) as exc:
            for _ in range(3):
                ws.receive_json()
        assert exc.value.code == 4401


def test_hub_close_at_grace_zero_ends_the_session_unless_another_connection_holds_it(env):
    import src.server.app as app_mod
    srv = env(session_disconnect_grace_s=0.0)
    s = _join(srv)
    _wait(lambda: srv.get_world_state())
    app_mod.set_persistent_world(srv)
    with _client(app_mod.app) as client:
        with _ws(client) as ws:
            ws.send_json({"op": "auth", "session_token": s.session_token})
            assert ws.receive_json()["topic"] == "auth"
            srv.sessions.mark_connected(s.session_id)           # a second connection (e.g. /ws/client)
        assert _wait(lambda: srv.sessions.get_session(s.session_id).ws_connections == 1)
        assert srv.sessions.get_session(s.session_id).is_active   # still held
        with _ws(client) as ws:
            ws.send_json({"op": "auth", "session_token": s.session_token})
            assert ws.receive_json()["topic"] == "auth"
            srv.sessions.mark_disconnected(s.session_id)         # the other one closes first
        # the hub was the last connection: session ended, vehicle gone
        assert _wait(lambda: not srv.sessions.get_session(s.session_id).is_active)
        assert _wait(lambda: s.session_id not in srv._client_vehicles)


def test_hub_close_with_grace_keeps_the_session(env):
    import src.server.app as app_mod
    srv = env(session_disconnect_grace_s=30.0)
    s = _join(srv)
    app_mod.set_persistent_world(srv)
    with _client(app_mod.app) as client:
        with _ws(client) as ws:
            ws.send_json({"op": "auth", "session_token": s.session_token})
            assert ws.receive_json()["topic"] == "auth"
        assert _wait(lambda: srv.sessions.get_session(s.session_id).ws_connections == 0)
        assert srv.sessions.get_session(s.session_id).is_active and s.session_id in srv._client_vehicles
