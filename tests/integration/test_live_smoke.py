"""Read-mostly smoke tests against a live CARLA server. Run: pytest -m carla.

They do not change world settings (no sync mode, no ticking). The BSM test spawns one
vehicle and always destroys it.
"""

import pytest

pytestmark = pytest.mark.carla


def test_server_version_matches_client(carla_client):
    assert carla_client.get_server_version() == carla_client.get_client_version()
    assert carla_client.get_server_version().startswith("0.9.16")


def test_world_has_map_and_vehicle_blueprints(carla_client):
    world = carla_client.get_world()
    assert world.get_map().name
    assert len(world.get_blueprint_library().filter("vehicle.*")) > 0


def test_bsm_from_real_vehicle(carla_client):
    from src.v2x.safety_messages import create_bsm_from_carla

    world = carla_client.get_world()
    bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    assert spawn_points, "map has no spawn points"
    vehicle = None
    for sp in spawn_points:
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is not None:
            break
    assert vehicle is not None, "could not spawn a vehicle"
    try:
        world.wait_for_tick(seconds=10.0)
        bsm = create_bsm_from_carla(vehicle, vehicle.id, 0, snapshot=world.get_snapshot())
        assert bsm.vehicle_id == vehicle.id
        assert bsm.vehicle_length > 0 and bsm.vehicle_width > 0
        # sim-time stamp from the snapshot, not wall clock (handoff S10)
        assert bsm.timestamp == pytest.approx(world.get_snapshot().timestamp.elapsed_seconds, abs=5.0)
    finally:
        vehicle.destroy()
