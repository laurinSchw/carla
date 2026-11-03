#!/usr/bin/env python3
"""
V2X Protocol Message Tests

Level 1 — pure unit tests, no CARLA runtime required.

Coverage:
  - VRUProfile and ObjectClassification enum values
  - MAPMessage / SPATMessage field defaults and construction
  - SPATMovementState phase timing window invariants
  - CPMMessage / PerceivedObject relative-position fields
  - VRUAwarenessMessage motion-state logic
  - V2XNetwork RSU and VRU registries (mocked actors)
  - V2XNetwork.update() dispatches to RSU and VRU pipelines
  - get_v2x_summary() includes V2X-specific keys
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.messages import (
    VRUProfile, ObjectClassification,
    MAPMessage, MAPLane,
    SPATMessage, SPATMovementState,
    PerceivedObject, CPMMessage,
    VRUAwarenessMessage,
)
from src.v2x.rsu import RSUConfig, RoadSideUnit
from src.v2x.network import V2XNetwork
from src.v2x.dsrc_channel import DSRCConfig


# ── Helpers ────────────────────────────────────────────────────────────────────

def _transparent_dsrc() -> DSRCConfig:
    """Zero-loss channel — deterministic protocol tests."""
    return DSRCConfig(
        shadowing_std_los_db=0.0,
        shadowing_std_nlos_db=0.0,
        channel_busy_ratio=0.0,
        enable_nlos_model=False,
        random_seed=0,
    )


class _MockVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _MockRotation:
    def __init__(self, yaw=0.0):
        self.pitch, self.yaw, self.roll = 0.0, yaw, 0.0


class _MockTransform:
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.location = _MockVector(x, y, z)
        self.rotation = _MockRotation(yaw)


class _MockControl:
    throttle = 0.0
    brake    = 0.0
    steer    = 0.0
    reverse  = False


class _MockBBox:
    extent = _MockVector(2.25, 0.9, 0.75)


class _MockVehicle:
    """Minimal CARLA vehicle mock."""
    def __init__(self, vid=0, x=0.0, y=0.0):
        self.id         = vid
        self.type_id    = 'vehicle.tesla.model3'
        self.attributes = {'number_of_wheels': '4'}
        self.bounding_box = _MockBBox()
        self._x, self._y = x, y

    def get_transform(self):
        return _MockTransform(self._x, self._y)

    def get_velocity(self):
        return _MockVector(5.0, 0.0, 0.0)

    def get_angular_velocity(self):
        return _MockVector()

    def get_control(self):
        return _MockControl()

    def get_physics_control(self):
        wheel = MagicMock()
        wheel.max_steer_angle = 70.0
        ctrl = MagicMock()
        ctrl.wheels = [wheel] * 4
        return ctrl

    def get_world(self):
        return None


class _MockWalker:
    """Minimal CARLA walker mock (no vehicle control)."""
    def __init__(self, vid=1000, x=10.0, y=5.0):
        self.id = vid
        self._x, self._y = x, y

    def get_transform(self):
        return _MockTransform(self._x, self._y)

    def get_velocity(self):
        return _MockVector(1.2, 0.0, 0.0)  # ~walking speed


class _MockSnapshot:
    """Minimal snapshot. Without vehicles it finds no actor (BSMs are then skipped, S10);
    pass the vehicles to make them visible in this tick."""
    def __init__(self, *vehicles):
        self._vehicles = {v.id: v for v in vehicles}

    def find(self, actor_id):
        return self._vehicles.get(actor_id)  # a vehicle mock offers the same getters as an ActorSnapshot

    @property
    def timestamp(self):
        ts = MagicMock()
        ts.elapsed_seconds = 5.0
        return ts


# ══════════════════════════════════════════════════════════════════════════════
# Enum sanity
# ══════════════════════════════════════════════════════════════════════════════

class TestVRUProfileEnum(unittest.TestCase):

    def test_profile_values_match_etsi(self):
        """ETSI TS 103 300-3 §6.1.3 integer values."""
        self.assertEqual(VRUProfile.PEDESTRIAN,   0)
        self.assertEqual(VRUProfile.CYCLIST,      1)
        self.assertEqual(VRUProfile.MOTORCYCLIST, 2)
        self.assertEqual(VRUProfile.ANIMAL,       3)
        self.assertEqual(VRUProfile.OTHER,        255)

    def test_object_classification_values(self):
        self.assertEqual(ObjectClassification.UNKNOWN,    0)
        self.assertEqual(ObjectClassification.PEDESTRIAN, 1)
        self.assertEqual(ObjectClassification.CYCLIST,    2)
        self.assertEqual(ObjectClassification.VEHICLE,    3)


# ══════════════════════════════════════════════════════════════════════════════
# MAP message
# ══════════════════════════════════════════════════════════════════════════════

class TestMAPMessage(unittest.TestCase):

    def _make_map(self, iid=1, revision=1):
        return MAPMessage(
            intersection_id=iid,
            revision=revision,
            ref_point=(10.0, 20.0, 0.5),
        )

    def test_default_speed_limit(self):
        """Default speed limit should be 50 km/h in m/s."""
        m = self._make_map()
        self.assertAlmostEqual(m.speed_limit_ms, 13.89, places=1)

    def test_lanes_default_empty(self):
        m = self._make_map()
        self.assertEqual(m.lanes, [])

    def test_ref_point_preserved(self):
        m = self._make_map()
        self.assertEqual(m.ref_point, (10.0, 20.0, 0.5))

    def test_add_lane(self):
        m = self._make_map()
        m.lanes.append(MAPLane(lane_id=1, direction="ingress"))
        self.assertEqual(len(m.lanes), 1)
        self.assertEqual(m.lanes[0].direction, "ingress")


# ══════════════════════════════════════════════════════════════════════════════
# SPAT message
# ══════════════════════════════════════════════════════════════════════════════

class TestSPATMessage(unittest.TestCase):

    def _make_spat(self, ts=100.0):
        return SPATMessage(
            intersection_id=1,
            intersection_name="intersection_1",
            timestamp=ts,
            states=[
                SPATMovementState(
                    signal_group_id=0,
                    movement_event="protectedMovementAllowed",
                    min_end_time_s=ts + 2.0,
                    max_end_time_s=ts + 4.0,
                    likely_time_s=ts + 3.0,
                )
            ],
        )

    def test_status_default_running(self):
        s = self._make_spat()
        self.assertEqual(s.status, "running")

    def test_phase_window_ordering(self):
        """min_end must not exceed max_end."""
        s = self._make_spat(ts=0.0)
        state = s.states[0]
        self.assertLessEqual(state.min_end_time_s, state.max_end_time_s)

    def test_likely_time_within_window(self):
        s = self._make_spat(ts=0.0)
        state = s.states[0]
        self.assertGreaterEqual(state.likely_time_s, state.min_end_time_s)
        self.assertLessEqual(state.likely_time_s, state.max_end_time_s)

    def test_multiple_signal_groups(self):
        ts = 50.0
        spat = SPATMessage(
            intersection_id=2,
            intersection_name="x",
            timestamp=ts,
            states=[
                SPATMovementState(0, "stop-And-Remain", ts + 1, ts + 3),
                SPATMovementState(1, "protectedMovementAllowed", ts + 0, ts + 2),
            ],
        )
        self.assertEqual(len(spat.states), 2)
        self.assertEqual(spat.states[0].movement_event, "stop-And-Remain")


# ══════════════════════════════════════════════════════════════════════════════
# CPM message
# ══════════════════════════════════════════════════════════════════════════════

class TestCPMMessage(unittest.TestCase):

    def test_perceived_object_confidence_range(self):
        obj = PerceivedObject(
            object_id=1,
            object_type=ObjectClassification.PEDESTRIAN,
            dx=5.0, dy=3.0,
            confidence=0.9,
        )
        self.assertGreaterEqual(obj.confidence, 0.0)
        self.assertLessEqual(obj.confidence, 1.0)

    def test_cpm_defaults_empty_lists(self):
        cpm = CPMMessage(station_id=1, timestamp=10.0, sender_position=(0.0, 0.0, 0.0))
        self.assertEqual(cpm.perceived_objects, [])
        self.assertEqual(cpm.sensor_info_list, [])

    def test_relative_position_stored(self):
        obj = PerceivedObject(
            object_id=42,
            object_type=ObjectClassification.CYCLIST,
            dx=-3.5, dy=8.2, dz=0.1,
        )
        self.assertAlmostEqual(obj.dx, -3.5)
        self.assertAlmostEqual(obj.dy,  8.2)
        self.assertAlmostEqual(obj.dz,  0.1)


# ══════════════════════════════════════════════════════════════════════════════
# VRUAM message
# ══════════════════════════════════════════════════════════════════════════════

class TestVRUAwarenessMessage(unittest.TestCase):

    def _make_vruam(self, speed=1.4, profile=VRUProfile.PEDESTRIAN):
        return VRUAwarenessMessage(
            station_id=1000,
            timestamp=20.0,
            vru_profile=profile,
            position=(5.0, 10.0, 0.0),
            speed=speed,
            heading=90.0,
        )

    def test_default_motion_state(self):
        vruam = self._make_vruam()
        self.assertEqual(vruam.motion_state, "moving")

    def test_cluster_defaults(self):
        vruam = self._make_vruam()
        self.assertIsNone(vruam.cluster_id)
        self.assertFalse(vruam.is_cluster_leader)

    def test_profile_stored(self):
        vruam = self._make_vruam(profile=VRUProfile.CYCLIST)
        self.assertEqual(vruam.vru_profile, VRUProfile.CYCLIST)

    def test_position_tuple(self):
        vruam = self._make_vruam()
        self.assertEqual(len(vruam.position), 3)


# ══════════════════════════════════════════════════════════════════════════════
# RSU
# ══════════════════════════════════════════════════════════════════════════════

class TestRSUConfig(unittest.TestCase):

    def test_default_tx_power(self):
        """ETSI TR 101 607 RSU default: 33 dBm."""
        cfg = RSUConfig(intersection_id=1, position=(0.0, 0.0, 0.0))
        self.assertEqual(cfg.tx_power_dbm, 33.0)

    def test_default_range(self):
        cfg = RSUConfig(intersection_id=1, position=(0.0, 0.0, 0.0))
        self.assertEqual(cfg.broadcast_range_m, 300.0)


class TestRoadSideUnit(unittest.TestCase):

    def _make_rsu(self, iid=1, position=(0.0, 0.0, 0.0)):
        cfg = RSUConfig(intersection_id=iid, position=position)
        return RoadSideUnit(cfg)

    def test_get_map_returns_correct_intersection_id(self):
        rsu = self._make_rsu(iid=5)
        m = rsu.get_map(timestamp=10.0)
        self.assertEqual(m.intersection_id, 5)

    def test_get_map_updates_timestamp(self):
        rsu = self._make_rsu()
        rsu.get_map(timestamp=1.0)
        m2 = rsu.get_map(timestamp=99.0)
        self.assertAlmostEqual(m2.timestamp, 99.0)

    def test_get_map_cached_same_object(self):
        """MAP cache should return the same MAPMessage instance (only timestamp updated)."""
        rsu = self._make_rsu()
        m1 = rsu.get_map(1.0)
        m2 = rsu.get_map(2.0)
        self.assertIs(m1, m2)

    def test_get_spat_no_traffic_lights(self):
        """RSU with no traffic lights should return empty states list."""
        rsu = self._make_rsu()
        spat = rsu.get_spat(timestamp=5.0)
        self.assertIsInstance(spat, SPATMessage)
        self.assertEqual(spat.states, [])

    def test_get_spat_with_mock_traffic_light(self):
        """SPAT states populated when a mock traffic light is present."""
        tl = MagicMock()
        tl.get_state.return_value = object()  # Unknown state → "unavailable"
        tl.get_elapsed_time.return_value = 2.0
        tl.get_green_time.return_value  = 10.0
        tl.get_yellow_time.return_value = 3.0
        tl.get_red_time.return_value    = 15.0

        cfg = RSUConfig(intersection_id=1, position=(0.0, 0.0, 0.0))
        rsu = RoadSideUnit(cfg, traffic_lights=[tl])
        spat = rsu.get_spat(timestamp=0.0)
        self.assertEqual(len(spat.states), 1)
        self.assertIsInstance(spat.states[0], SPATMovementState)

    def test_from_traffic_light_factory(self):
        tl = MagicMock()
        tl.get_transform.return_value = _MockTransform(50.0, 80.0, 0.5)
        rsu = RoadSideUnit.from_traffic_light(tl, intersection_id=7)
        self.assertEqual(rsu.config.intersection_id, 7)
        self.assertAlmostEqual(rsu.config.position[0], 50.0)
        self.assertAlmostEqual(rsu.config.position[1], 80.0)


# ══════════════════════════════════════════════════════════════════════════════
# V2XNetwork — registry and update dispatch
# ══════════════════════════════════════════════════════════════════════════════

class TestV2XNetwork(unittest.TestCase):

    def _make_network(self):
        return V2XNetwork(
            max_range=200.0,
            update_rate_hz=2.0,
            dsrc_config=_transparent_dsrc(),
        )

    def _make_rsu(self, iid):
        cfg = RSUConfig(intersection_id=iid, position=(0.0, 0.0, 0.0))
        return RoadSideUnit(cfg)

    # ── RSU registry ───────────────────────────────────────────────────────

    def test_register_rsu_stores_rsu(self):
        net = self._make_network()
        rsu = self._make_rsu(iid=1)
        net.register_rsu(rsu)
        self.assertIn(1, net._rsus)

    def test_unregister_rsu_removes_rsu(self):
        net = self._make_network()
        net.register_rsu(self._make_rsu(iid=1))
        net.unregister_rsu(1)
        self.assertNotIn(1, net._rsus)
        self.assertNotIn(1, net.map_messages)
        self.assertNotIn(1, net.spat_messages)

    # ── VRU registry ───────────────────────────────────────────────────────

    def test_register_vru_stores_vru(self):
        net = self._make_network()
        walker = _MockWalker(vid=1000)
        net.register_vru(1000, walker, VRUProfile.PEDESTRIAN)
        self.assertIn(1000, net._vrus)
        self.assertEqual(net._vru_profiles[1000], VRUProfile.PEDESTRIAN)

    def test_unregister_vru_removes_vru(self):
        net = self._make_network()
        walker = _MockWalker(vid=1000)
        net.register_vru(1000, walker)
        net.unregister_vru(1000)
        self.assertNotIn(1000, net._vrus)
        self.assertNotIn(1000, net._vru_profiles)

    # ── update() dispatches RSU and VRU pipelines ──────────────────────────

    def test_update_populates_spat_and_map(self):
        net = self._make_network()
        # Register a vehicle so V2V update proceeds
        veh = _MockVehicle(vid=0, x=0.0, y=0.0)
        net.register(0, veh)
        rsu = self._make_rsu(iid=3)
        net.register_rsu(rsu)

        snapshot = _MockSnapshot(*net._vrus.values())
        updated = net.update(snapshot=snapshot, force=True)

        self.assertTrue(updated)
        self.assertIn(3, net.map_messages)
        self.assertIn(3, net.spat_messages)

    def test_update_populates_vruam(self):
        net = self._make_network()
        veh = _MockVehicle(vid=0, x=0.0, y=0.0)
        net.register(0, veh)
        walker = _MockWalker(vid=1000, x=5.0, y=5.0)
        net.register_vru(1000, walker, VRUProfile.PEDESTRIAN)

        snapshot = _MockSnapshot(*net._vrus.values())
        net.update(snapshot=snapshot, force=True)

        self.assertIn(1000, net.vru_messages)
        vruam = net.vru_messages[1000]
        self.assertEqual(vruam.vru_profile, VRUProfile.PEDESTRIAN)
        self.assertGreater(vruam.speed, 0.0)  # MockWalker has velocity (1.2, 0, 0)

    def test_vruam_motion_state_moving(self):
        net = self._make_network()
        veh = _MockVehicle(vid=0)
        net.register(0, veh)
        walker = _MockWalker(vid=1001)  # speed ~1.2 m/s > 0.3 threshold
        net.register_vru(1001, walker, VRUProfile.CYCLIST)

        net.update(snapshot=_MockSnapshot(*net._vrus.values()), force=True)

        vruam = net.get_vruam(1001)
        self.assertIsNotNone(vruam)
        self.assertEqual(vruam.motion_state, "moving")

    def test_vruam_stationary_when_speed_below_threshold(self):
        """Walkers with near-zero velocity should be marked stationary."""
        class _StillWalker(_MockWalker):
            def get_velocity(self):
                return _MockVector(0.0, 0.0, 0.0)

        net = self._make_network()
        veh = _MockVehicle(vid=0)
        net.register(0, veh)
        net.register_vru(2000, _StillWalker(vid=2000), VRUProfile.PEDESTRIAN)
        net.update(snapshot=_MockSnapshot(*net._vrus.values()), force=True)

        vruam = net.get_vruam(2000)
        self.assertEqual(vruam.motion_state, "stationary")

    # ── get_all_vruam ──────────────────────────────────────────────────────

    def test_get_all_vruam_returns_copy(self):
        net = self._make_network()
        veh = _MockVehicle(vid=0)
        net.register(0, veh)
        w1 = _MockWalker(vid=1000)
        w2 = _MockWalker(vid=1001)
        net.register_vru(1000, w1)
        net.register_vru(1001, w2)
        net.update(snapshot=_MockSnapshot(*net._vrus.values()), force=True)

        all_vruam = net.get_all_vruam()
        self.assertEqual(len(all_vruam), 2)
        # Modifying the returned dict does not affect internal state
        all_vruam.clear()
        self.assertEqual(len(net.vru_messages), 2)

    # ── get_v2x_summary ───────────────────────────────────────────────────

    def test_get_v2x_summary_has_v2x_keys(self):
        net = self._make_network()
        summary = net.get_v2x_summary()
        for key in ("rsu_count", "vru_count", "active_spat", "active_vruam"):
            self.assertIn(key, summary, f"Missing key: {key}")

    def test_get_v2x_summary_counts_match(self):
        net = self._make_network()
        veh = _MockVehicle(vid=0)
        net.register(0, veh)
        net.register_rsu(self._make_rsu(1))
        net.register_rsu(self._make_rsu(2))
        net.register_vru(1000, _MockWalker(vid=1000))

        net.update(snapshot=_MockSnapshot(*net._vrus.values()), force=True)

        summary = net.get_v2x_summary()
        self.assertEqual(summary["rsu_count"],   2)
        self.assertEqual(summary["vru_count"],   1)
        self.assertEqual(summary["active_spat"], 2)
        self.assertEqual(summary["active_vruam"], 1)

    # ── backward compat: V2V interface still works ─────────────────────────

    def test_v2v_register_and_get_neighbors_still_works(self):
        """V2XNetwork must not break existing V2V behavior."""
        net = self._make_network()
        veh0 = _MockVehicle(vid=0, x=0.0,  y=0.0)
        veh1 = _MockVehicle(vid=1, x=10.0, y=0.0)
        net.register(0, veh0)
        net.register(1, veh1)

        net.update(snapshot=_MockSnapshot(veh0, veh1), force=True)

        # Both vehicles are within 200 m range and transparent channel → neighbors
        nbrs = net.get_neighbors(0)
        self.assertTrue(any(n.vehicle_id == 1 for n in nbrs))


if __name__ == '__main__':
    unittest.main()
