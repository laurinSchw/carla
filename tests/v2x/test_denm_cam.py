"""
Unit tests for DENM and CAM protocol support.

Covers:
  - DENM dataclass construction and field validation
  - V2XNetwork DENM store: publish / get / cancel
  - create_cam_from_bsm() field mapping
  - API endpoints: GET /cam, GET /cam/{id}, GET /denm, GET /denm/{id},
                   POST /denm, DELETE /denm/{station}/{action}

Level 1 (pure unit) — no CARLA runtime required.
"""

import pytest
import time

# Re-use mock infrastructure from the existing basic test suite
from tests.v2x.test_bsm_basic import (
    MockVehicle,
    MockWorld,
    MockWorldSnapshot,
    MockActorSnapshot,
    _transparent_dsrc,
)

from src.v2x.safety_messages import (
    BSMCore,
    DENM,
    DENMCauseCode,
    create_cam_from_bsm,
    BrakingStatus,
    VehicleType,
)
from src.v2x.network import V2XNetwork

# ── Helpers ──────────────────────────────────────────────────────────────────


def _make_bsm(vehicle_id: int = 1, x: float = 10.0, y: float = 20.0,
              speed: float = 5.0) -> BSMCore:
    """Construct a minimal BSMCore without a CARLA actor."""
    return BSMCore(
        timestamp=100.0,
        msg_count=1,
        vehicle_id=vehicle_id,
        latitude=x,
        longitude=y,
        speed=speed,
        heading=90.0,
    )


def _make_denm(station_id: int = 1, action_id: int = 42,
               cause: DENMCauseCode = DENMCauseCode.ACCIDENT) -> DENM:
    return DENM(
        station_id=station_id,
        action_id=action_id,
        detection_time=200.0,
        reference_time=200.0,
        event_position=(10.0, 20.0, 0.0),
        cause_code=cause,
        subcause_code=1,
        information_quality=5,
        relevance_distance=100.0,
    )


def _make_network() -> V2XNetwork:
    return V2XNetwork(
        max_range=200.0,
        update_rate_hz=2.0,
        dsrc_config=_transparent_dsrc(),
    )


# ── DENM dataclass tests ─────────────────────────────────────────────────────


class TestDENMDataclass:
    def test_fields_set_correctly(self):
        d = _make_denm()
        assert d.station_id == 1
        assert d.action_id == 42
        assert d.cause_code == DENMCauseCode.ACCIDENT
        assert d.event_position == (10.0, 20.0, 0.0)
        assert d.termination is None

    def test_cause_code_enum_values(self):
        assert int(DENMCauseCode.ACCIDENT) == 2
        assert int(DENMCauseCode.ROAD_WORKS) == 3
        assert int(DENMCauseCode.COLLISION_RISK) == 97
        assert int(DENMCauseCode.EMERGENCY_VEHICLE_APPROACHING) == 95

    def test_default_termination_is_none(self):
        d = _make_denm()
        assert d.termination is None

    def test_optional_fields_default_to_none(self):
        d = _make_denm()
        assert d.event_speed is None
        assert d.event_heading is None


# ── DENM network store tests ─────────────────────────────────────────────────


class TestDENMNetworkStore:
    def test_publish_adds_to_store(self):
        net = _make_network()
        d = _make_denm(station_id=1, action_id=10)
        net.publish_denm(d)
        assert len(net.get_denm()) == 1

    def test_get_denm_all(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=2, action_id=1))
        assert len(net.get_denm()) == 2

    def test_get_denm_by_station(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=1, action_id=2))
        net.publish_denm(_make_denm(station_id=2, action_id=1))
        result = net.get_denm(station_id=1)
        assert len(result) == 2
        assert all(d.station_id == 1 for d in result)

    def test_get_denm_unknown_station_returns_empty(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        assert net.get_denm(station_id=99) == []

    def test_publish_same_key_updates_in_place(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1,
                                    cause=DENMCauseCode.ACCIDENT))
        updated = DENM(
            station_id=1, action_id=1,
            detection_time=300.0, reference_time=300.0,
            event_position=(0.0, 0.0, 0.0),
            cause_code=DENMCauseCode.ROAD_WORKS,
        )
        net.publish_denm(updated)
        stored = net.get_denm()
        assert len(stored) == 1
        assert stored[0].cause_code == DENMCauseCode.ROAD_WORKS

    def test_cancel_removes_denm(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=5))
        removed = net.cancel_denm(station_id=1, action_id=5)
        assert removed is True
        assert len(net.get_denm()) == 0

    def test_cancel_nonexistent_returns_false(self):
        net = _make_network()
        assert net.cancel_denm(station_id=99, action_id=99) is False

    def test_cancel_only_removes_matching_entry(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=1, action_id=2))
        net.cancel_denm(station_id=1, action_id=1)
        remaining = net.get_denm()
        assert len(remaining) == 1
        assert remaining[0].action_id == 2


# ── create_cam_from_bsm tests ────────────────────────────────────────────────


class TestCreateCamFromBsm:
    def test_station_id_maps_to_vehicle_id(self):
        bsm = _make_bsm(vehicle_id=7)
        cam = create_cam_from_bsm(bsm)
        assert cam.station_id == 7

    def test_reference_position_maps_correctly(self):
        bsm = _make_bsm(x=100.0, y=200.0)
        cam = create_cam_from_bsm(bsm)
        assert cam.reference_position == (100.0, 200.0, 0.0)

    def test_speed_and_heading_copied(self):
        bsm = _make_bsm(speed=12.5)
        bsm.heading = 45.0
        cam = create_cam_from_bsm(bsm)
        assert cam.speed == 12.5
        assert cam.heading == 45.0

    def test_forward_direction(self):
        bsm = _make_bsm()
        bsm.transmission_state = "forward"
        cam = create_cam_from_bsm(bsm)
        assert cam.drive_direction == "forward"

    def test_reverse_direction(self):
        bsm = _make_bsm()
        bsm.transmission_state = "reverse"
        cam = create_cam_from_bsm(bsm)
        assert cam.drive_direction == "reverse"

    def test_generation_time_equals_bsm_timestamp(self):
        bsm = _make_bsm()
        cam = create_cam_from_bsm(bsm)
        assert cam.generation_time == bsm.timestamp
