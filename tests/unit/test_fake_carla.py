"""The fake simulator: determinism, kinematics, spawn rules, traffic lights and LiDAR geometry."""

import math

import numpy as np
import pytest

from src.lidar.transforms import to_world
from src.testing.fake_carla import city, lidar
from src.testing.fake_carla.sim import Client, build_module


@pytest.fixture
def world():
    w = Client("fake", 2000, seed=3).get_world()
    s = w.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    w.apply_settings(s)
    return w


def spawn_cars(w, n, autopilot=True):
    bp = w.get_blueprint_library().find("vehicle.tesla.model3")
    cars = []
    for sp in w.get_map().get_spawn_points()[:n]:
        v = w.try_spawn_actor(bp, sp)
        assert v is not None
        v.set_autopilot(autopilot)
        cars.append(v)
    return cars


def test_module_is_marked_fake_and_has_the_api_the_server_uses():
    m = build_module()
    for name in ("Client", "Transform", "Location", "Rotation", "TrafficLightState", "command", "WorldSettings"):
        assert hasattr(m, name), name
    assert m.IS_FAKE is True and m.TrafficLightState.Green != m.TrafficLightState.Red


def test_install_refuses_to_replace_the_real_module(monkeypatch):
    import sys
    import types
    from src.testing.fake_carla import install
    monkeypatch.setitem(sys.modules, "carla", types.ModuleType("carla"))
    with pytest.raises(RuntimeError, match="real carla"):
        install()


def test_ticks_advance_frame_and_time_and_snapshot_matches():
    w = Client(seed=1).get_world()
    f0 = w.get_snapshot().frame
    s = w.get_settings()
    s.fixed_delta_seconds = 0.05
    w.apply_settings(s)
    assert w.tick() == f0 + 1 == w.get_snapshot().frame
    for _ in range(19):
        w.tick()
    assert w.get_snapshot().timestamp.elapsed_seconds == pytest.approx(1.0)


def test_vehicles_move_at_the_ring_speed_and_are_deterministic(world):
    cars = spawn_cars(world, 6)
    start = [(c.st.x, c.st.y) for c in cars]
    for _ in range(40):
        world.tick()
    snap = world.get_snapshot()
    for c, (x0, y0) in zip(cars, start):
        v = snap.find(c.id).get_velocity()
        assert math.hypot(v.x, v.y) == pytest.approx(c.ring.speed_mps)
        assert (c.st.x, c.st.y) != (x0, y0)
    # a second world with the same seed and calls reproduces every position
    w2 = Client("fake", 2000, seed=3).get_world()
    s = w2.get_settings()
    s.fixed_delta_seconds = 0.05
    w2.apply_settings(s)
    cars2 = spawn_cars(w2, 6)
    for _ in range(40):
        w2.tick()
    assert [(c.st.x, c.st.y, c.st.yaw) for c in cars] == [(c.st.x, c.st.y, c.st.yaw) for c in cars2]


def test_stationary_without_autopilot_and_braking_control(world):
    (car,) = spawn_cars(world, 1, autopilot=False)
    x, y = car.st.x, car.st.y
    for _ in range(10):
        world.tick()
    assert (car.st.x, car.st.y) == (x, y) and world.get_snapshot().find(car.id).get_velocity().length() == 0
    assert car.get_control().brake == 1.0
    car.set_autopilot(True)
    world.tick()
    assert car.get_control().throttle > 0


def test_speed_follows_the_traffic_manager_percentage(world):
    (car,) = spawn_cars(world, 1)
    tm = Client(world=world).get_trafficmanager(8001)
    tm.global_percentage_speed_difference(-20.0)            # CARLA: negative = faster than the limit
    world.tick()
    assert world.get_snapshot().find(car.id).get_velocity().length() == pytest.approx(car.ring.speed_mps * 1.2)


def test_yaw_follows_the_ring_and_yaw_rate_is_in_deg_per_s(world):
    (car,) = spawn_cars(world, 1)
    yaws, rates = [], []
    for _ in range(400):
        world.tick()
        yaws.append(car.st.yaw)
        rates.append(car.st.yaw_rate)
    assert max(abs(r) for r in rates) > 20                      # corners
    straight = [r for r in rates if abs(r) < 0.5]
    assert len(straight) > 100                                  # and long straights
    assert max(abs(r) for r in rates) < 120                     # no jumps: a car at ~8 m/s on a 10 m corner turns ~45 deg/s


def test_spawn_rules(world):
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    sp = world.get_map().get_spawn_points()[0]
    assert world.try_spawn_actor(bp, sp) is not None
    assert world.try_spawn_actor(bp, sp) is None                # occupied
    off_road = type(sp)(type(sp.location)(-500.0, 900.0, 0.0), sp.rotation)
    assert world.try_spawn_actor(bp, off_road) is None
    with pytest.raises(RuntimeError):
        world.spawn_actor(bp, sp)


def test_batch_commands_spawn_autopilot_and_destroy(world):
    import src.testing.fake_carla.sim as sim
    m = build_module()
    client = Client(world=world)
    bp = world.get_blueprint_library().find("vehicle.audi.a2")
    sps = world.get_map().get_spawn_points()
    batch = [m.command.SpawnActor(bp, sps[i]).then(m.command.SetAutopilot(m.command.FutureActor, True, 8001))
             for i in range(3)]
    res = client.apply_batch_sync(batch, False)
    assert all(r.error is None for r in res)
    cars = [world.get_actor(r.actor_id) for r in res]
    assert all(isinstance(c, sim.Vehicle) and c.autopilot for c in cars)
    res2 = client.apply_batch_sync([m.command.DestroyActor(c) for c in cars], False)
    assert all(r.error is None for r in res2) and all(world.get_actor(c.id) is None for c in cars)
    assert client.apply_batch_sync([m.command.DestroyActor(cars[0])], False)[0].error


def test_actor_list_filter_and_blueprints(world):
    lights = world.get_actors().filter("traffic.traffic_light")
    assert len(lights) == 36
    assert len(world.get_blueprint_library().filter("vehicle.*")) >= 14
    wheels = [int(b.get_attribute("number_of_wheels")) for b in world.get_blueprint_library().filter("vehicle.*")]
    assert set(wheels) == {2, 4}                                # two-wheelers exist, the server must skip them
    with pytest.raises(IndexError):
        world.get_blueprint_library().find("vehicle.nope")


def test_pedestrians_walk_on_sidewalks_only_with_a_started_controller(world):
    bps = world.get_blueprint_library()
    loc = world.get_random_location_from_navigation()
    ped = world.try_spawn_actor(bps.find("walker.pedestrian.0001"), type(world.get_map().get_spawn_points()[0])(loc))
    ctrl = world.try_spawn_actor(bps.find("controller.ai.walker"), type(world.get_map().get_spawn_points()[0])(), ped)
    x0 = (ped.st.x, ped.st.y)
    for _ in range(20):
        world.tick()
    assert (ped.st.x, ped.st.y) == x0                           # no controller started: standing
    ctrl.start()
    ctrl.go_to_location(world.get_random_location_from_navigation())
    ctrl.set_max_speed(1.4)
    for _ in range(100):
        world.tick()
    assert math.hypot(ped.st.x - x0[0], ped.st.y - x0[1]) > 3.0
    assert math.hypot(ped.st.vx, ped.st.vy) <= 1.4 + 1e-9


def test_traffic_light_cycle_is_consistent(world):
    for t in np.arange(0, 2 * city.CYCLE_S, 0.25):
        ns, ns_el = city.light_state(float(t), 0)
        ew, ew_el = city.light_state(float(t), 1)
        assert not (ns == "Green" and ew in ("Green", "Yellow")), t
        assert not (ew == "Green" and ns in ("Green", "Yellow")), t
        assert ns_el >= 0 and ew_el >= 0
    # the actors expose the same through the CARLA-like API; groups hold 4 lights of one intersection
    tl = world.get_actors().filter("traffic.traffic_light")[0]
    group = tl.get_group_traffic_lights()
    assert len(group) == 4 and tl.id in {g.id for g in group}
    assert {g.axis for g in group} == {0, 1}
    assert tl.get_red_time() == tl.get_green_time() + tl.get_yellow_time()


def test_map_topology_has_lanes_and_junctions(world):
    topo = world.get_map().get_topology()
    assert len(topo) == 84                                      # 48 lane segments + 4 connectors x 9 junctions
    juncs = {a.get_junction().id for a, _ in topo if a.is_junction}
    assert juncs == set(range(1, 10))
    a, b = topo[0]
    w = a.next(2.0)
    assert w and abs(w[0].transform.location.x - a.transform.location.x) + abs(w[0].transform.location.y - a.transform.location.y) == pytest.approx(2.0)
    assert a.next(10_000.0) == []


# ── LiDAR geometry ─────────────────────────────────────────────────────────────────────────────────

def cast(origin, rot, dirs, boxes, semantic=True, range_m=100.0):
    return lidar.cast(origin, rot, np.asarray(dirs, dtype=float), range_m, boxes, city.GRID, 7.0, 9.0, semantic)


def test_ray_hits_ground_at_the_expected_distance_and_tag():
    d = np.array([[math.cos(math.radians(-30)), 0.0, math.sin(math.radians(-30))]])
    pts = cast((0.0, 0.0, 2.4), (0, 0, 0), d, [])
    assert len(pts) == 1
    expected_x = 2.4 / math.tan(math.radians(30))
    assert pts["x"][0] == pytest.approx(expected_x, rel=1e-5) and pts["z"][0] == pytest.approx(-2.4, rel=1e-5)
    assert pts["tag"][0] == lidar.TAG_ROADS and pts["object_idx"][0] == 0         # on the x=0 / y=0 roads
    assert pts["cos_inc"][0] == pytest.approx(math.sin(math.radians(30)), rel=1e-5)


def test_ray_hits_a_box_with_its_actor_id_and_tag():
    box = lidar.Box(10.0, 0.0, 0.75, 2.3, 0.9, 0.75, 0.0, 77, lidar.TAG_CAR)
    pts = cast((0.0, 0.0, 0.75), (0, 0, 0), [[1.0, 0.0, 0.0]], [box])
    assert len(pts) == 1 and pts["object_idx"][0] == 77 and pts["tag"][0] == lidar.TAG_CAR
    assert pts["x"][0] == pytest.approx(10.0 - 2.3)                                # the front face
    assert pts["cos_inc"][0] == pytest.approx(1.0)


def test_box_is_blocked_by_a_nearer_box_and_range_limits_hits():
    near = lidar.Box(5.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1, lidar.TAG_CAR)
    far = lidar.Box(20.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 2, lidar.TAG_CAR)
    pts = cast((0.0, 0.0, 1.0), (0, 0, 0), [[1.0, 0.0, 0.0]], [far, near])
    assert pts["object_idx"].tolist() == [1]
    assert len(cast((0.0, 0.0, 1.0), (0, 0, 0), [[1.0, 0.0, 0.0]], [far], range_m=10.0)) == 0


def test_rotated_box_and_sensor_frame_roundtrip():
    # a box seen from a sensor with yaw 90: forward of the sensor is world +y
    box = lidar.Box(0.0, 10.0, 1.0, 1.0, 1.0, 1.0, 45.0, 5, lidar.TAG_CAR)
    pts = cast((0.0, 0.0, 1.0), (0, 90, 0), [[1.0, 0.0, 0.0]], [box])
    assert len(pts) == 1
    local = np.array([[pts["x"][0], pts["y"][0], pts["z"][0]]])
    world_pt = to_world(local, (0.0, 0.0, 1.0), (0, 90, 0))[0]
    assert world_pt[0] == pytest.approx(0.0, abs=1e-4) and 8.0 < world_pt[1] < 10.0 and world_pt[2] == pytest.approx(1.0)
    assert local[0, 0] == pytest.approx(world_pt[1])                              # sensor x = world y here


def test_raycast_type_has_intensity_instead_of_tags():
    pts = cast((0.0, 0.0, 2.0), (0, 0, 0), [[0.0, 0.0, -1.0]], [], semantic=False)
    assert pts.dtype.names == ("x", "y", "z", "intensity") and 0 <= pts["intensity"][0] <= 1


def test_sensor_delivers_one_measurement_per_tick_with_the_right_frame_and_a_self_hit(world):
    (car,) = spawn_cars(world, 1)
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    bp.set_attribute("points_per_second", "100000")
    got = []
    sensor = world.spawn_actor(bp, type(car.get_transform())(type(car.get_location())(0, 0, 2.4)), car)
    sensor.listen(got.append)
    for _ in range(3):
        f = world.tick()
        assert got[-1].frame == f
    assert len(got) == 3
    raw = np.frombuffer(got[-1].raw_data, dtype=lidar.SEMANTIC_DTYPE)
    assert len(raw) > 1000 and (raw["object_idx"] == car.id).any(), "CARLA's sensor sees its own vehicle"
    assert got[-1].transform.location.z == pytest.approx(2.4)       # the vehicle stands on the ground (z = 0)
    sensor.stop()
    world.tick()
    assert len(got) == 3
    car.destroy()
    assert world.get_actor(sensor.id) is None                                       # attached sensors die with the parent


def test_lidar_is_a_pure_function_of_the_world_state(world):
    cars = spawn_cars(world, 8)
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    bp.set_attribute("points_per_second", "60000")
    sensor = world.spawn_actor(bp, type(cars[0].get_transform())(type(cars[0].get_location())(0, 0, 2.4)), cars[0])
    out = []
    sensor.listen(out.append)
    world.tick()
    a = out[-1].raw_data
    # same state again (new world, same seed, same calls)
    w2 = Client("fake", 2000, seed=3).get_world()
    s = w2.get_settings()
    s.fixed_delta_seconds = 0.05
    w2.apply_settings(s)
    cars2 = spawn_cars(w2, 8)
    bp2 = w2.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    bp2.set_attribute("points_per_second", "60000")
    sensor2 = w2.spawn_actor(bp2, type(cars2[0].get_transform())(type(cars2[0].get_location())(0, 0, 2.4)), cars2[0])
    out2 = []
    sensor2.listen(out2.append)
    w2.tick()
    assert out2[-1].raw_data == a


# ── additions for this branch's server (per-vehicle TM settings, walker seed, install) ───────────────

def test_per_vehicle_tm_settings_are_accepted_and_the_speed_offset_moves_the_car(world):
    """src/traffic/drivers.py applies these names per vehicle; only the speed offset changes the fake's motion."""
    from src.testing.fake_carla.sim import TM_VEHICLE_SETTINGS
    from src.traffic.drivers import PROFILES
    (car,) = spawn_cars(world, 1)
    tm = Client(world=world).get_trafficmanager(8001)
    assert set(PROFILES["normal"].tm_settings()) <= set(TM_VEHICLE_SETTINGS)
    for name, value in PROFILES["cautious"].tm_settings().items():
        getattr(tm, name)(car, value)
    assert world.tm_settings[car.id]["keep_slow_lane_rule_percentage"] == 80.0
    world.tick()
    v = world.get_snapshot().find(car.id).get_velocity().length()
    assert v == pytest.approx(car.ring.speed_mps * (1 - 15.0 / 100))
    with pytest.raises(AttributeError):
        tm.keep_right_rule_percentage(car, 10)          # the wrong name (AGENTS.md section 5) fails like on the live TM
    car.destroy()
    assert car.id not in world.tm_settings


def test_walker_targets_follow_set_pedestrians_seed():
    a, b = Client(seed=1).get_world(), Client(seed=2).get_world()
    a.set_pedestrians_seed(42)
    b.set_pedestrians_seed(42)
    pa = [(p.x, p.y) for p in (a.get_random_location_from_navigation() for _ in range(5))]
    pb = [(p.x, p.y) for p in (b.get_random_location_from_navigation() for _ in range(5))]
    assert pa == pb


def test_install_is_idempotent_and_marks_the_module(monkeypatch):
    import sys
    from src.testing.fake_carla import install
    monkeypatch.setitem(sys.modules, "carla", None)     # restored (or removed) after the test
    sys.modules.pop("carla")
    first = install(1)
    assert sys.modules["carla"] is first and first.IS_FAKE is True
    assert install(2) is first


def test_production_modules_do_not_import_the_fake():
    """The fake is reached only through --fake / fake: true (start_server.py, src/scenario.py)."""
    import re
    from pathlib import Path
    root = Path(__file__).resolve().parents[2] / "src"
    imports = re.compile(r"^\s*(from|import)\s+src\.testing", re.M)
    offenders = [str(p.relative_to(root)) for p in root.rglob("*.py")
                 if "testing" not in p.parts and p.name != "scenario.py" and imports.search(p.read_text(errors="ignore"))]
    assert offenders == []
