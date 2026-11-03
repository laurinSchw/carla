"""Regression tests for model fixes from REFACTOR_HANDOFF §3 (each has a docs/science/CHANGELOG.md entry)."""

from types import SimpleNamespace as NS

from src.v2x.network import V2XNetwork
from src.v2x.rsu import RoadSideUnit, RSUConfig
from src.v2x.safety_messages import create_bsm_from_carla


def _vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def test_s2_yaw_rate_is_degrees_per_second_not_converted_again():
    """CARLA get_angular_velocity() is deg/s; 12.5 deg/s in must give 12.5 deg/s out (was x57.3)."""
    actor_snap = NS(
        get_transform=lambda: NS(location=_vec(), rotation=NS(yaw=0.0)),
        get_velocity=lambda: _vec(5.0),
        get_angular_velocity=lambda: _vec(0.0, 0.0, 12.5),
    )
    vehicle = NS(
        id=1, type_id="vehicle.tesla.model3", attributes={"number_of_wheels": "4"},
        bounding_box=NS(extent=_vec(2.3, 0.9, 0.75)),
        get_transform=lambda: NS(location=_vec(), rotation=NS(yaw=0.0)),
        get_velocity=lambda: _vec(5.0),
        get_angular_velocity=lambda: _vec(),
        get_control=lambda: NS(steer=0.0, brake=0.0, throttle=0.2, reverse=False),
        get_physics_control=lambda: NS(wheels=[NS(max_steer_angle=70.0)]),
    )
    snapshot = NS(timestamp=NS(elapsed_seconds=1.0), find=lambda i: actor_snap)
    bsm = create_bsm_from_carla(vehicle, 1, 0, snapshot=snapshot, prev_velocity=5.0, delta_time=0.5)
    assert bsm.yaw_rate == 12.5


def test_c7_two_hz_gate_does_not_skip_updates_from_float_accumulation():
    """Accumulating 0.05 s per frame and updating every 10th frame must give exactly 2 Hz.

    Before the fix the sim-time gate saw dt = 0.4999.. on ~23% of ticks and skipped them.
    """
    net = V2XNetwork(update_rate_hz=2.0)
    sim_time, performed, frames = 0.0, 0, 7200
    for frame in range(1, frames + 1):
        sim_time += 0.05
        if frame % 10 == 0:
            snapshot = NS(timestamp=NS(elapsed_seconds=sim_time))
            performed += net.update(snapshot)
    assert performed == frames // 10


def test_s17_spat_timing_is_none_when_carla_cannot_provide_it():
    """No fabricated 5 s fallback: unreadable phase timing is reported as unknown."""

    class _Light:
        def get_state(self):
            return "whatever"

        def get_elapsed_time(self):
            raise RuntimeError("RPC failed")

    rsu = RoadSideUnit(RSUConfig(intersection_id=3, position=(0.0, 0.0, 0.0)), traffic_lights=[_Light()])
    spat = rsu.get_spat(timestamp=10.0)
    assert len(spat.states) == 1
    state = spat.states[0]
    assert state.likely_time_s is None
    assert state.min_end_time_s is None
    assert state.max_end_time_s is None
