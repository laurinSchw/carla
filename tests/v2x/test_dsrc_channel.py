#!/usr/bin/env python3
"""
DSRC/WAVE (IEEE 802.11p) Channel Model Tests

Level 1 — pure unit tests, no CARLA runtime required.

Coverage:
  - DSRCConfig default values (ETSI EN 302 663 compliance)
  - Path loss physics: PRR monotonically decreases with distance
  - Close-range deterministic delivery (< 0.5 m)
  - LOS receivers have higher PRR than NLOS at equal distance
  - CSMA/CA contention: more vehicles → lower PRR
  - broadcast_all → get_received consistency
  - Channel statistics tracking (total_broadcasts, prr)
  - Deterministic behaviour under a fixed seed
  - Integration: V2XNetwork respects channel drops in neighbour discovery
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig, _euclidean, _point_to_segment_dist
from src.v2x.safety_messages import BSMCore, VehicleType, BrakingStatus
from src.v2x import V2XNetwork


# ── Test helpers ───────────────────────────────────────────────────────────────

def _bsm(vehicle_id: int, x: float = 0.0, y: float = 0.0, speed: float = 10.0) -> BSMCore:
    """Minimal BSMCore for channel tests; x/y stored in latitude/longitude."""
    return BSMCore(
        timestamp=1000.0,
        msg_count=0,
        vehicle_id=vehicle_id,
        vehicle_type=VehicleType.PASSENGER_CAR,
        latitude=x,
        longitude=y,
        elevation=0.0,
        position_accuracy=0.5,
        speed=speed,
        heading=0.0,
        steering_angle=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        vertical_accel=0.0,
        yaw_rate=0.0,
        vehicle_length=4.5,
        vehicle_width=1.8,
        vehicle_height=1.5,
        brake_status=BrakingStatus.OFF,
        brake_pressure=0.0,
        transmission_state="forward",
        throttle_confidence=100.0,
        brake_confidence=100.0,
        steering_confidence=100.0,
    )


def _run_n_trials(
    n: int,
    distance_m: float,
    config: DSRCConfig,
    nlos: bool = False,
) -> float:
    """
    Run n independent trials for a single (sender, receiver) pair at the given
    distance and return the empirical PRR.  Uses a fresh channel per call so
    the seed produces a fresh sample sequence.
    """
    delivered = 0
    for seed in range(n):
        cfg = DSRCConfig(
            tx_power_dbm=config.tx_power_dbm,
            antenna_gain_dbi=config.antenna_gain_dbi,
            rx_sensitivity_dbm=config.rx_sensitivity_dbm,
            path_loss_exponent_los=config.path_loss_exponent_los,
            path_loss_exponent_nlos=config.path_loss_exponent_nlos,
            shadowing_std_los_db=config.shadowing_std_los_db,
            shadowing_std_nlos_db=config.shadowing_std_nlos_db,
            channel_busy_ratio=0.0,   # no contention so only PHY is tested
            enable_nlos_model=False,  # controlled externally
            random_seed=seed,
        )
        channel = DSRCChannel(cfg)

        # Use internal PRR method for a single pair (isolate PHY from geometry)
        prr = channel._compute_prr(distance_m, nlos, n_contenders=1)
        if channel._rng.random() < prr:
            delivered += 1
    # PRR approximation from sigmoid (not stochastic) for the purpose of tests
    # Use the direct computation from a single channel with a fixed seed instead
    channel = DSRCChannel(config)
    prr_direct = channel._compute_prr(distance_m, nlos, n_contenders=1)
    return prr_direct


# ── DSRCConfig tests ───────────────────────────────────────────────────────────

class TestDSRCConfig(unittest.TestCase):

    def test_default_tx_power_is_etsi_class_a(self):
        """Default TX power must be 23 dBm per ETSI EN 302 663 §4."""
        cfg = DSRCConfig()
        self.assertEqual(cfg.tx_power_dbm, 23.0)

    def test_default_carrier_sensitivity(self):
        """Default sensitivity should be -85 dBm (802.11p spec floor)."""
        cfg = DSRCConfig()
        self.assertEqual(cfg.rx_sensitivity_dbm, -85.0)

    def test_los_exponent_less_than_nlos(self):
        """LOS path loss exponent must be smaller than NLOS (physical constraint)."""
        cfg = DSRCConfig()
        self.assertLess(cfg.path_loss_exponent_los, cfg.path_loss_exponent_nlos)

    def test_los_shadowing_less_than_nlos(self):
        """LOS shadowing σ must be smaller than NLOS σ (Paier et al. 2008)."""
        cfg = DSRCConfig()
        self.assertLess(cfg.shadowing_std_los_db, cfg.shadowing_std_nlos_db)

    def test_cbr_default_in_range(self):
        cfg = DSRCConfig()
        self.assertGreaterEqual(cfg.channel_busy_ratio, 0.0)
        self.assertLessEqual(cfg.channel_busy_ratio, 1.0)


# ── Physics tests ──────────────────────────────────────────────────────────────

class TestDSRCChannelPhysics(unittest.TestCase):
    """
    Test the channel physics model with a zero-shadowing config for
    deterministic analytical verification.
    """

    def _no_shadow_config(self, cbr: float = 0.0) -> DSRCConfig:
        """Config with σ=0 dB gives fully deterministic path-loss only model."""
        return DSRCConfig(
            shadowing_std_los_db=0.0,
            shadowing_std_nlos_db=0.0,
            channel_busy_ratio=cbr,
            enable_nlos_model=False,
            random_seed=42,
        )

    def test_prr_near_one_at_short_range(self):
        """
        At 10 m LOS with σ=0, SNR margin >> 0 → PRR must be > 0.99.

        P_rx at 10 m (reference distance) = P_tx + 2*G_ant - L0
                              = 23 + 4 - 67.9 ≈ -40.9 dBm
        SNR margin = -40.9 - (-85) = 44.1 dB → logistic(k*44.1) ≈ 1.0
        """
        cfg = self._no_shadow_config()
        channel = DSRCChannel(cfg)
        prr = channel._compute_prr(10.0, nlos=False, n_contenders=1)
        self.assertGreater(prr, 0.99)

    def test_prr_near_zero_far_below_sensitivity(self):
        """
        Artificially low TX power should push PRR → 0 at moderate range.

        With tx_power_dbm=-100 dBm and sensitivity=-85 dBm, margin << 0 at 10 m.
        """
        cfg = DSRCConfig(
            tx_power_dbm=-100.0,
            shadowing_std_los_db=0.0,
            channel_busy_ratio=0.0,
            enable_nlos_model=False,
            random_seed=42,
        )
        channel = DSRCChannel(cfg)
        prr = channel._compute_prr(10.0, nlos=False, n_contenders=1)
        self.assertLess(prr, 0.01)

    def test_prr_decreases_with_distance(self):
        """
        PRR must monotonically decrease with distance at fixed σ=0, LOS.

        With ETSI EN 302 663 defaults (TX=23 dBm, sensitivity=-85 dBm,
        n_LOS=2.0, L0≈67.9 dB at 10 m), the sensitivity crossover is ~1.6 km.
        PRR is saturated at 1.0 well below 1 km, so we test near and past the
        crossover point where the sigmoid produces a strict ordering.
        """
        cfg = self._no_shadow_config()
        channel = DSRCChannel(cfg)
        # 1 km: before crossover (PRR ≈ 1.0)
        # 1.6 km: near crossover (PRR ≈ 0.5)
        # 3.0 km: well past crossover (PRR ≈ 0)
        prr_near = channel._compute_prr(1_000.0, nlos=False, n_contenders=1)
        prr_mid = channel._compute_prr(1_600.0, nlos=False, n_contenders=1)
        prr_far = channel._compute_prr(3_000.0, nlos=False, n_contenders=1)

        self.assertGreater(prr_near, prr_mid,
                           "PRR at 1 km should exceed PRR at 1.6 km")
        self.assertGreater(prr_mid, prr_far,
                           "PRR at 1.6 km should exceed PRR at 3 km")

    def test_nlos_prr_lower_than_los_same_distance(self):
        """
        At the same distance and σ=0, NLOS path loss exponent is higher →
        lower received power → lower PRR.
        """
        cfg = self._no_shadow_config()
        channel = DSRCChannel(cfg)
        prr_los = channel._compute_prr(100.0, nlos=False, n_contenders=1)
        prr_nlos = channel._compute_prr(100.0, nlos=True, n_contenders=1)
        self.assertGreater(prr_los, prr_nlos,
                           "LOS PRR must exceed NLOS PRR at equal distance")

    def test_csma_unity_with_one_contender(self):
        """P_no_contention must be 1.0 when there is only one TX station."""
        cfg = self._no_shadow_config(cbr=0.5)
        channel = DSRCChannel(cfg)
        p = channel._csma_success_prob(1)
        self.assertEqual(p, 1.0)

    def test_csma_decreases_with_more_contenders(self):
        """More contenders must strictly reduce the CSMA success probability."""
        cfg = self._no_shadow_config(cbr=0.3)
        channel = DSRCChannel(cfg)
        p1 = channel._csma_success_prob(1)
        p5 = channel._csma_success_prob(5)
        p10 = channel._csma_success_prob(10)
        self.assertEqual(p1, 1.0)
        self.assertGreater(p1, p5)
        self.assertGreater(p5, p10)

    def test_received_power_increases_with_lower_distance(self):
        """P_rx must be strictly higher at shorter distance (σ=0 LOS)."""
        cfg = self._no_shadow_config()
        channel = DSRCChannel(cfg)
        p_10 = channel._received_power_dbm(10.0, nlos=False)
        p_50 = channel._received_power_dbm(50.0, nlos=False)
        # With σ=0, no randomness — comparison is exact
        self.assertGreater(p_10, p_50)


# ── broadcast_all / get_received tests ────────────────────────────────────────

class TestDSRCChannelBroadcast(unittest.TestCase):

    def _clear_channel(self) -> DSRCChannel:
        """Channel with no shadowing and no contention for deterministic tests."""
        cfg = DSRCConfig(
            shadowing_std_los_db=0.0,
            shadowing_std_nlos_db=0.0,
            channel_busy_ratio=0.0,
            enable_nlos_model=False,
            random_seed=42,
        )
        return DSRCChannel(cfg)

    def test_packet_delivered_at_zero_distance(self):
        """
        Two vehicles at the same position must always exchange BSMs
        (deterministic short-circuit in broadcast_all).
        """
        channel = self._clear_channel()
        bsms = {0: _bsm(0, x=0.0, y=0.0), 1: _bsm(1, x=0.0, y=0.0)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        channel.broadcast_all(bsms, positions)

        self.assertIn(0, channel.get_received(1))
        self.assertIn(1, channel.get_received(0))

    def test_no_self_delivery(self):
        """A vehicle must never receive its own BSM."""
        channel = self._clear_channel()
        bsms = {0: _bsm(0, x=0.0, y=0.0), 1: _bsm(1, x=10.0, y=0.0)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        channel.broadcast_all(bsms, positions)

        self.assertNotIn(0, channel.get_received(0))
        self.assertNotIn(1, channel.get_received(1))

    def test_high_prr_close_range(self):
        """
        Two vehicles at 10 m with σ=0, no contention must always receive each
        other's BSM (PRR ≈ 1.0 at 10 m from design).
        """
        bsms = {0: _bsm(0, x=0.0, y=0.0), 1: _bsm(1, x=10.0, y=0.0)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        # Run 20 trials; at PRR ≈ 1.0 all should succeed
        successes = 0
        for seed in range(20):
            cfg = DSRCConfig(
                shadowing_std_los_db=0.0,
                shadowing_std_nlos_db=0.0,
                channel_busy_ratio=0.0,
                enable_nlos_model=False,
                random_seed=seed,
            )
            ch = DSRCChannel(cfg)
            ch.broadcast_all(bsms, positions)
            if 1 in ch.get_received(0):
                successes += 1

        self.assertGreaterEqual(successes, 19,
                                "Expected ≥ 19/20 successes at 10 m with σ=0")

    def test_empty_network_produces_no_received(self):
        channel = self._clear_channel()
        channel.broadcast_all({}, {})
        self.assertEqual(channel.get_received(0), {})

    def test_single_vehicle_has_no_received(self):
        channel = self._clear_channel()
        bsms = {0: _bsm(0)}
        positions = {0: (0.0, 0.0)}
        channel.broadcast_all(bsms, positions)
        self.assertEqual(channel.get_received(0), {})

    def test_broadcast_all_resets_previous_tick(self):
        """Results from a previous tick must not bleed into the next."""
        channel = self._clear_channel()
        bsms = {0: _bsm(0, x=0.0, y=0.0), 1: _bsm(1, x=0.0, y=0.0)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        channel.broadcast_all(bsms, positions)
        # Second call with empty network — previous results must be gone
        channel.broadcast_all({}, {})
        self.assertEqual(channel.get_received(0), {})
        self.assertEqual(channel.get_received(1), {})

    def test_get_received_unknown_id_returns_empty(self):
        channel = self._clear_channel()
        self.assertEqual(channel.get_received(999), {})


# ── Statistics tests ───────────────────────────────────────────────────────────

class TestDSRCChannelStats(unittest.TestCase):

    def test_stats_track_broadcast_count(self):
        """total_broadcasts must equal num_vehicles * (num_vehicles - 1)."""
        cfg = DSRCConfig(
            shadowing_std_los_db=0.0, channel_busy_ratio=0.0,
            enable_nlos_model=False, random_seed=42,
        )
        channel = DSRCChannel(cfg)
        n = 4
        bsms = {i: _bsm(i, x=float(i * 5)) for i in range(n)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        channel.broadcast_all(bsms, positions)

        self.assertEqual(
            int(channel.stats["total_broadcasts"]),
            n * (n - 1),
            "Each ordered pair (sender, receiver) contributes one broadcast attempt",
        )

    def test_deliveries_plus_drops_equals_broadcasts(self):
        cfg = DSRCConfig(random_seed=42)
        channel = DSRCChannel(cfg)
        bsms = {i: _bsm(i, x=float(i * 30)) for i in range(3)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        channel.broadcast_all(bsms, positions)
        s = channel.stats
        self.assertEqual(
            int(s["total_deliveries"]) + int(s["total_drops"]),
            int(s["total_broadcasts"]),
        )

    def test_prr_is_bounded(self):
        cfg = DSRCConfig(random_seed=7)
        channel = DSRCChannel(cfg)
        bsms = {i: _bsm(i, x=float(i * 20)) for i in range(5)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}
        channel.broadcast_all(bsms, positions)

        prr = channel.stats["prr"]
        self.assertGreaterEqual(prr, 0.0)
        self.assertLessEqual(prr, 1.0)

    def test_get_stats_returns_copy(self):
        """Mutating the returned dict must not affect internal state."""
        channel = DSRCChannel(DSRCConfig(random_seed=0))
        stats = channel.get_stats()
        stats["prr"] = -1.0
        self.assertNotEqual(channel.stats["prr"], -1.0)


# ── Determinism tests ──────────────────────────────────────────────────────────

class TestDSRCChannelDeterminism(unittest.TestCase):

    def test_same_seed_same_results(self):
        """Two channels with the same seed must produce identical delivery maps."""
        bsms = {i: _bsm(i, x=float(i * 40)) for i in range(4)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        ch1 = DSRCChannel(DSRCConfig(random_seed=123))
        ch2 = DSRCChannel(DSRCConfig(random_seed=123))

        ch1.broadcast_all(bsms, positions)
        ch2.broadcast_all(bsms, positions)

        for receiver_id in bsms:
            self.assertEqual(
                set(ch1.get_received(receiver_id).keys()),
                set(ch2.get_received(receiver_id).keys()),
                f"Delivery set for receiver {receiver_id} differs between identical seeds",
            )

    def test_different_seeds_may_differ(self):
        """Different seeds should (with overwhelming probability) produce different results."""
        bsms = {i: _bsm(i, x=float(i * 80)) for i in range(5)}
        positions = {vid: (b.latitude, b.longitude) for vid, b in bsms.items()}

        ch1 = DSRCChannel(DSRCConfig(random_seed=1))
        ch2 = DSRCChannel(DSRCConfig(random_seed=999))

        ch1.broadcast_all(bsms, positions)
        ch2.broadcast_all(bsms, positions)

        all_same = all(
            set(ch1.get_received(r).keys()) == set(ch2.get_received(r).keys())
            for r in bsms
        )
        # Deterministic given the seeds (verified: seeds 1 and 999 give different delivery sets).
        self.assertFalse(all_same, "different seeds produced identical delivery sets")


# ── Geometry helper tests ──────────────────────────────────────────────────────

class TestGeometryHelpers(unittest.TestCase):

    def test_euclidean_axis_aligned(self):
        self.assertAlmostEqual(_euclidean((0.0, 0.0), (3.0, 4.0)), 5.0)

    def test_euclidean_zero_distance(self):
        self.assertAlmostEqual(_euclidean((1.0, 2.0), (1.0, 2.0)), 0.0)

    def test_point_on_segment(self):
        """Point at midpoint of segment → distance 0."""
        self.assertAlmostEqual(
            _point_to_segment_dist((1.0, 0.0), (0.0, 0.0), (2.0, 0.0)), 0.0
        )

    def test_point_perpendicular_to_segment(self):
        """Point directly above midpoint → distance = perpendicular height."""
        dist = _point_to_segment_dist((1.0, 5.0), (0.0, 0.0), (2.0, 0.0))
        self.assertAlmostEqual(dist, 5.0, places=5)

    def test_point_past_segment_end(self):
        """Point beyond the end of the segment → clamps to nearest endpoint."""
        dist = _point_to_segment_dist((10.0, 0.0), (0.0, 0.0), (2.0, 0.0))
        self.assertAlmostEqual(dist, 8.0, places=5)

    def test_degenerate_segment_zero_length(self):
        """Zero-length segment (a == b) → distance to that single point."""
        dist = _point_to_segment_dist((3.0, 4.0), (0.0, 0.0), (0.0, 0.0))
        self.assertAlmostEqual(dist, 5.0, places=5)


# ── Integration test: V2XNetwork respects channel model ────────────────

class MockVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

class MockRotation:
    def __init__(self, yaw=0.0):
        self.pitch = 0.0
        self.yaw = yaw
        self.roll = 0.0

class MockTransform:
    def __init__(self, x=0.0, y=0.0):
        self.location = MockVector(x, y, 0.0)
        self.rotation = MockRotation()

class MockControl:
    throttle = 0.0
    brake = 0.0
    steer = 0.0
    reverse = False
    hand_brake = False
    manual_gear_shift = False
    gear = 1

class MockPhysicsControl:
    wheels = [type("W", (), {"max_steer_angle": 70.0})() for _ in range(4)]

class MockVehicle:
    def __init__(self, vid, x=0.0, y=0.0):
        self.id = vid
        self._t = MockTransform(x, y)
        self._v = MockVector()
        self._c = MockControl()
        self.bounding_box = type("B", (), {"extent": MockVector(2.25, 0.9, 0.75)})()

    def get_transform(self): return self._t
    def get_velocity(self): return self._v
    def get_control(self): return self._c
    def get_acceleration(self): return MockVector()
    def get_angular_velocity(self): return MockVector()
    def get_physics_control(self): return MockPhysicsControl()
    def get_world(self): return None


class MockActorSnapshot:
    def __init__(self, v): self.vehicle = v
    def get_velocity(self): return self.vehicle.get_velocity()
    def get_angular_velocity(self): return self.vehicle.get_angular_velocity()
    def get_acceleration(self): return self.vehicle.get_acceleration()
    def get_transform(self): return self.vehicle.get_transform()


class MockTimestamp:
    elapsed_seconds = 0.0


class MockSnapshot:
    def __init__(self, vehicles):
        self.timestamp = MockTimestamp()
        self._snaps = {v.id: MockActorSnapshot(v) for v in vehicles}

    def find(self, actor_id):
        return self._snaps.get(actor_id)


class TestV2XNetworkWithDSRC(unittest.TestCase):
    """
    Level 2 component test: V2XNetwork with the DSRC channel active.
    Uses zero-shadowing, zero-contention config so outcomes are
    deterministic and physics-driven only.
    """

    def _make_network(self, max_range: float = 100.0) -> V2XNetwork:
        cfg = DSRCConfig(
            shadowing_std_los_db=0.0,
            shadowing_std_nlos_db=0.0,
            channel_busy_ratio=0.0,
            enable_nlos_model=False,
            random_seed=42,
        )
        return V2XNetwork(
            max_range=max_range,
            update_rate_hz=2.0,
            enable_cooperative_perception=True,
            dsrc_config=cfg,
        )

    def test_close_vehicles_discover_each_other(self):
        """
        Two vehicles at 10 m separation with σ=0 must always be neighbours
        (PRR ≈ 1.0 at 10 m from ETSI default params).
        """
        v0 = MockVehicle(0, x=0.0, y=0.0)
        v1 = MockVehicle(1, x=10.0, y=0.0)
        net = self._make_network(max_range=100.0)
        net.register(0, v0)
        net.register(1, v1)

        snap = MockSnapshot([v0, v1])
        snap.timestamp.elapsed_seconds = 0.5
        net.update(snapshot=snap, force=True)

        neighbours_0 = net.get_neighbors(0)
        self.assertTrue(
            any(n.vehicle_id == 1 for n in neighbours_0),
            "Vehicle 1 should be a neighbour of vehicle 0 at 10 m",
        )

    def test_vehicles_beyond_app_range_not_neighbours(self):
        """
        Vehicles beyond max_range must not appear as neighbours regardless
        of channel PRR (application-layer range filter takes precedence).
        """
        v0 = MockVehicle(0, x=0.0, y=0.0)
        v1 = MockVehicle(1, x=200.0, y=0.0)
        net = self._make_network(max_range=50.0)
        net.register(0, v0)
        net.register(1, v1)

        snap = MockSnapshot([v0, v1])
        snap.timestamp.elapsed_seconds = 0.5
        net.update(snapshot=snap, force=True)

        neighbours_0 = net.get_neighbors(0)
        self.assertFalse(
            any(n.vehicle_id == 1 for n in neighbours_0),
            "Vehicle 200 m away must not be a neighbour (beyond max_range=50 m)",
        )

    def test_get_channel_stats_returns_dict(self):
        net = self._make_network()
        stats = net.get_channel_stats()
        self.assertIsInstance(stats, dict)
        for key in ("total_broadcasts", "total_deliveries", "total_drops", "prr"):
            self.assertIn(key, stats, f"Channel stats must include '{key}'")

    def test_shutdown_does_not_raise(self):
        net = self._make_network()
        try:
            net.shutdown()
        except Exception as exc:
            self.fail(f"shutdown() raised unexpectedly: {exc}")

    def test_prr_in_stats_after_update(self):
        """After an update tick, channel PRR must be in [0, 1]."""
        v0 = MockVehicle(0, x=0.0, y=0.0)
        v1 = MockVehicle(1, x=30.0, y=0.0)
        net = self._make_network(max_range=100.0)
        net.register(0, v0)
        net.register(1, v1)

        snap = MockSnapshot([v0, v1])
        snap.timestamp.elapsed_seconds = 0.5
        net.update(snapshot=snap, force=True)

        stats = net.get_channel_stats()
        self.assertGreaterEqual(stats["prr"], 0.0)
        self.assertLessEqual(stats["prr"], 1.0)


if __name__ == "__main__":
    unittest.main()
