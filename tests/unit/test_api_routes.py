"""/api/* routes against the real tick loop and the recording CARLA stub."""

import importlib
import sys

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")
from fastapi.testclient import TestClient  # noqa: E402

from src.server.world_config import WorldConfig  # noqa: E402
from tests.unit.carla_stub import Recorder, make_carla_module  # noqa: E402
from tests.unit.test_hub_views import _wait  # noqa: E402


@pytest.fixture
def admin_globals():
    """Route tests set admin_routes' module state directly: save it and restore it so no test leaks into the next."""
    import src.server.admin_routes as admin_mod
    saved = (admin_mod._world, admin_mod._connect_state, admin_mod._connect_error)
    yield admin_mod
    admin_mod._world, admin_mod._connect_state, admin_mod._connect_error = saved


@pytest.fixture
def api(monkeypatch, tmp_path):
    monkeypatch.setenv("ADMIN_TOKEN", "tok")
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    import src.server.app as app_mod
    srv = pw.PersistentCARLAServer(WorldConfig(host="stub", runs_dir=str(tmp_path / "runs")))
    srv.start()
    app_mod.set_persistent_world(srv)
    _wait(lambda: srv.get_world_state())
    yield TestClient(app_mod.app), {"X-Admin-Token": "tok"}, srv
    app_mod.set_persistent_world(None)
    srv.stop()
    sys.modules.pop("src.server.persistent_world", None)


def test_status_and_config(api):
    c, h, _ = api
    assert c.get("/api/status").json()["connected"] is True
    assert c.get("/api/config").json()["seed"] == 42
    assert c.get("/api/config/schema").json()["properties"]["awareness_range_m"]["x-apply"] == "hot"


def test_patch_config_hot_cold_and_invalid(api):
    c, h, srv = api
    r = c.patch("/api/config", json={"awareness_range_m": 120.0}, headers=h)
    assert r.status_code == 200 and srv.config.awareness_range_m == 120.0 and srv.v2x_network.max_range == 120.0
    assert c.patch("/api/config", json={"seed": 1}, headers=h).status_code == 409
    assert c.patch("/api/config", json={"nope": 1}, headers=h).status_code == 422
    assert c.patch("/api/config", json={"awareness_range_m": 1.0}, headers=h).status_code == 422
    assert c.patch("/api/config", json={"awareness_range_m": 90.0}).status_code == 422   # no token header


def test_observer_gate(api):
    c, h, srv = api
    c.patch("/api/config", json={"observer_public": False}, headers=h)
    assert c.get("/api/config").status_code == 403
    assert c.get("/api/config", headers=h).status_code == 200


def test_denm_inject_list_cancel(api):
    c, h, _ = api
    r = c.post("/api/v2x/denm", json={"x": 1, "y": 2, "cause_code": "ACCIDENT"}, headers=h)
    assert r.status_code == 201, r.text
    row = r.json()
    assert row["station_id"] < 0
    assert [d["station_id"] for d in c.get("/api/v2x/denm").json()["denms"]] == [row["station_id"]]
    assert c.post("/api/v2x/denm", json={"x": 1, "y": 2, "station_id": 5}, headers=h).status_code == 422
    assert c.delete(f"/api/v2x/denm/{row['station_id']}/0", headers=h).status_code == 200
    assert c.delete(f"/api/v2x/denm/{row['station_id']}/0", headers=h).status_code == 404


def test_run_lifecycle_and_manifest_path_safety(api):
    c, h, _ = api
    r = c.post("/api/runs", json={"name": "t1"}, headers=h)
    assert r.status_code == 201, r.text
    run_id = r.json()["run_id"]
    assert c.post("/api/runs", json={"name": "t2"}, headers=h).status_code == 409
    assert c.get("/api/runs", headers=h).json()["active"]["run_id"] == run_id
    assert c.post("/api/runs/current/stop", headers=h).status_code == 200
    assert c.post("/api/runs/current/stop", headers=h).status_code == 404
    runs = c.get("/api/runs", headers=h).json()["runs"]
    assert runs[0]["status"] == "finished"
    assert c.get(f"/api/runs/{run_id}/manifest", headers=h).status_code == 200
    assert c.get("/api/runs/..%2f..%2fetc/manifest", headers=h).status_code == 404
    assert c.post("/api/runs", json={"name": "t3", "record": ["bogus"]}, headers=h).status_code == 422


def test_rsu_patch(api):
    from src.v2x.rsu import RoadSideUnit, RSUConfig
    c, h, srv = api
    assert c.patch("/api/v2x/rsus/-999", json={"enabled": False}, headers=h).status_code == 404
    rsu = RoadSideUnit(RSUConfig(intersection_id=7, position=(1.0, 2.0, 0.0)), [])
    srv.v2x_network.register_rsu(rsu)
    r = c.patch("/api/v2x/rsus/-7", json={"enabled": False, "tx_power_dbm": 30}, headers=h)
    assert r.status_code == 200 and r.json() == {"station_id": -7, "enabled": False, "tx_power_dbm": 30.0}
    assert srv.v2x_network.get_rsu(7) is None
    assert c.patch("/api/v2x/rsus/-7", json={"tx_power_dbm": 99}, headers=h).status_code == 422
    assert c.patch("/api/v2x/rsus/-7", json={"enabled": True}, headers=h).json()["enabled"] is True


def test_map_geometry_builder():
    from types import SimpleNamespace as NS
    from src.server.map_geometry import build_map_geometry

    class Wp:
        def __init__(self, x, y):
            self.transform = NS(location=NS(x=x, y=y)); self.is_junction = False
        def next(self, d):
            return [Wp(self.transform.location.x + d, 0.0)]

    geom = build_map_geometry(NS(name="T", get_topology=lambda: [(Wp(0, 0), Wp(10, 0))]))
    assert geom["lanes"][0][0] == [0.0, 0.0] and geom["lanes"][0][-1] == [10.0, 0.0] and len(geom["version"]) == 16


def test_map_list_and_change_restarts_the_world_on_the_new_map(api, monkeypatch, admin_globals):
    import time
    import src.server.admin_routes as admin_mod
    c, h, srv = api
    admin_mod._world = srv
    r = c.get("/admin/maps", headers=h)
    assert r.status_code == 200 and "Town03" in r.json()["maps"] and r.json()["current"].endswith("TestTown")
    assert c.post("/admin/map", json={"map_name": "Nowhere"}, headers=h).status_code == 422
    assert c.post("/admin/map", json={"map_name": "town03"}, headers=h).status_code == 200      # name is case-insensitive
    for _ in range(100):
        if admin_mod._connect_state == "running" and admin_mod._world is not srv:
            break
        time.sleep(0.1)
    new = admin_mod._world
    try:
        assert new is not srv
        assert new.config.map_name == "Town03"
        assert new.get_status()["map"].endswith("Town03")
        assert new.config.seed == srv.config.seed and new.config.runs_dir == srv.config.runs_dir   # same configuration
    finally:
        new.stop()
        admin_mod._world = srv


def test_connect_with_a_map_name_loads_it_and_an_unknown_map_fails(api):
    import pytest as _pytest
    from src.server.persistent_world import PersistentCARLAServer
    c, h, srv = api
    assert PersistentCARLAServer.match_map("Town03", ["/Game/Carla/Maps/Town03", "/Game/Carla/Maps/Town10HD_Opt"]) == "/Game/Carla/Maps/Town03"
    assert PersistentCARLAServer.match_map("/Game/Carla/Maps/town10hd_opt/", ["/Game/Carla/Maps/Town10HD_Opt"]) == "/Game/Carla/Maps/Town10HD_Opt"
    assert PersistentCARLAServer.match_map("Town99", ["/Game/Carla/Maps/Town03"]) is None
    bad = PersistentCARLAServer(WorldConfig(host="stub", map_name="Town99", runs_dir=srv.config.runs_dir))
    with _pytest.raises(ValueError, match="not available"):
        bad.start()


def test_map_switch_conflicts_and_bad_names_do_not_cost_the_running_world(api, admin_globals):
    import src.server.admin_routes as admin_mod
    c, h, srv = api
    admin_mod._world = srv
    # a connect with an unknown map must fail before the running world is stopped
    admin_mod._connect_state = "idle"
    assert c.post("/admin/connect", json={"carla_host": "stub", "map_name": "Town99"}, headers=h).status_code == 200
    import time
    for _ in range(50):
        if admin_mod._connect_state != "connecting":
            break
        time.sleep(0.1)
    # nothing was stopped: the healthy world stays "running" (no red error on a running world), the reason is still reported
    assert admin_mod._connect_state == "running" and "not available" in admin_mod._connect_error
    assert admin_mod._world is srv and srv._running, "the working world was stopped by a bad map name"
    # while a connect runs, /admin/map answers 409 (not 503)
    admin_mod._connect_state = "connecting"
    try:
        assert c.post("/admin/map", json={"map_name": "Town03"}, headers=h).status_code == 409
    finally:
        admin_mod._connect_state = "idle"
    # the name is constrained
    assert c.post("/admin/connect", json={"carla_host": "stub", "map_name": "../x y"}, headers=h).status_code == 422


def test_map_routes_same_map_is_a_noop_and_a_stopped_world_is_409(api, admin_globals):
    admin_mod = admin_globals
    c, h, srv = api
    admin_mod._world = srv
    admin_mod._connect_state = "running"
    current = srv.get_status()["map"]
    r = c.post("/admin/map", json={"map_name": srv.short_map_name(current)}, headers=h)
    assert r.status_code == 200 and r.json()["changed"] is False
    assert admin_mod._world is srv and srv._running, "loading the current map restarted the world"
    srv._running = False
    try:
        assert c.get("/admin/maps", headers=h).status_code == 409
        assert c.post("/admin/map", json={"map_name": "Town03"}, headers=h).status_code == 409
    finally:
        srv._running = True
