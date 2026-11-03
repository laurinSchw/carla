"""Golden test pinning every field of create_bsm_from_carla() for a fixed mock snapshot.

Pins current behaviour INCLUDING known defects (e.g. S8 x/y stored in x_m/y_m). Fixes change this file deliberately.
"""

import sys
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.safety_messages import create_bsm_from_carla
from tests.golden.helpers import assert_matches_golden


def _vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _vehicle(actor_id=7, type_id="vehicle.tesla.model3", steer=0.2, brake=0.0,
             throttle=0.6, reverse=False):
    return NS(
        id=actor_id,
        type_id=type_id,
        attributes={"number_of_wheels": "4"},
        bounding_box=NS(extent=_vec(2.3, 0.9, 0.75)),
        get_transform=lambda: NS(location=_vec(-1.0, -1.0, -1.0), rotation=NS(yaw=-1.0)),
        get_velocity=lambda: _vec(-99.0, 0.0, 0.0),
        get_angular_velocity=lambda: _vec(0.0, 0.0, -99.0),
        get_control=lambda: NS(steer=steer, brake=brake, throttle=throttle, reverse=reverse),
        get_physics_control=lambda: NS(wheels=[NS(max_steer_angle=70.0)]),
    )


def _snapshot(actor_id=7):
    actor_snap = NS(
        get_transform=lambda: NS(location=_vec(120.5, -33.25, 0.5), rotation=NS(yaw=-135.0)),
        get_velocity=lambda: _vec(6.0, 8.0, 0.0),
        get_angular_velocity=lambda: _vec(0.0, 0.0, 0.5),
    )
    return NS(
        timestamp=NS(elapsed_seconds=12.35),
        find=lambda i: actor_snap if i == actor_id else None,
    )


def test_bsm_from_snapshot_golden():
    bsm = create_bsm_from_carla(_vehicle(), 7, 130, snapshot=_snapshot(),
                                prev_velocity=9.0, delta_time=0.5)
    assert_matches_golden("bsm_from_snapshot", asdict(bsm))


def test_bsm_braking_reverse_golden():
    v = _vehicle(actor_id=8, type_id="vehicle.ford.ambulance", steer=-0.5, brake=0.7,
                 throttle=0.0, reverse=True)
    bsm = create_bsm_from_carla(v, 8, 5, snapshot=_snapshot(8),
                                prev_velocity=12.0, delta_time=0.5)
    assert_matches_golden("bsm_braking_reverse", asdict(bsm))
