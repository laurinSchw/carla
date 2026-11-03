"""Golden tests pinning DSRCChannel output (seed 42) at the pre-refactor commit.

Pure-refactor commits must keep these identical (REFACTOR_HANDOFF Phase 0).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig
from src.v2x.safety_messages import BSMCore, BrakingStatus, VehicleType
from tests.golden.helpers import assert_matches_golden

TICKS = 5  # consecutive broadcast_all() calls on one channel, so the RNG sequence is pinned


def _bsm(vid: int, x: float, y: float) -> BSMCore:
    return BSMCore(
        timestamp=1000.0, msg_count=0, vehicle_id=vid,
        vehicle_type=VehicleType.PASSENGER_CAR,
        latitude=x, longitude=y, elevation=0.0, position_accuracy=0.5,
        speed=10.0, heading=0.0, steering_angle=0.0, longitudinal_accel=0.0,
        lateral_accel=0.0, vertical_accel=0.0, yaw_rate=0.0,
        vehicle_length=4.5, vehicle_width=1.8, vehicle_height=1.5,
        brake_status=BrakingStatus.OFF, brake_pressure=0.0,
        transmission_state="forward", throttle_confidence=100.0,
        brake_confidence=100.0, steering_confidence=100.0,
    )


def _line(n, spacing):
    # collinear: intermediate vehicles block the TX->RX segment (NLOS geometry)
    return {i: (i * spacing, 0.0) for i in range(n)}


def _grid(rows, cols, spacing):
    return {r * cols + c: (c * spacing, r * spacing) for r in range(rows) for c in range(cols)}


SCENARIOS = {
    "pair_30m": (lambda: {0: (0.0, 0.0), 1: (30.0, 0.0)}, {}),
    "pair_250m": (lambda: {0: (0.0, 0.0), 1: (250.0, 0.0)}, {}),
    "pair_400m": (lambda: {0: (0.0, 0.0), 1: (400.0, 0.0)}, {}),
    "line10_nlos": (lambda: _line(10, 25.0), {}),
    "line10_no_nlos_model": (lambda: _line(10, 25.0), {"enable_nlos_model": False}),
    "grid50": (lambda: _grid(5, 10, 20.0), {}),
    "grid100": (lambda: _grid(10, 10, 30.0), {}),
}


def _run(positions, cfg_kwargs):
    ch = DSRCChannel(DSRCConfig(random_seed=42, **cfg_kwargs))
    msgs = {vid: _bsm(vid, *p) for vid, p in positions.items()}
    ticks = []
    for _ in range(TICKS):
        ch.broadcast_all(msgs, dict(positions))
        ticks.append({str(rx): sorted(ch.get_received(rx)) for rx in positions})
    return {"received_per_tick": ticks, "stats": ch.get_stats()}


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_dsrc_channel_golden(name):
    make_positions, cfg_kwargs = SCENARIOS[name]
    assert_matches_golden(f"dsrc_{name}", _run(make_positions(), cfg_kwargs))
