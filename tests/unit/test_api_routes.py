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
