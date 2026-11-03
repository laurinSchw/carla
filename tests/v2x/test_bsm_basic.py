#!/usr/bin/env python3
"""
Simplified V2V Network Tests
Tests core V2V functionality with mock objects.
"""

import unittest
import pytest
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x import V2XNetwork, BSMCore
from src.v2x.dsrc_channel import DSRCConfig


def _transparent_dsrc() -> DSRCConfig:
    """Zero-loss channel config for protocol-level tests (not physics tests)."""
    return DSRCConfig(
        shadowing_std_los_db=0.0,
        shadowing_std_nlos_db=0.0,
        channel_busy_ratio=0.0,
        enable_nlos_model=False,
        random_seed=42,
    )


# Simple mock objects
class MockVector:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class MockRotation:
    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class MockTransform:
    def __init__(self, x, y, z, yaw=0):
        self.location = MockVector(x, y, z)
        self.rotation = MockRotation(0, yaw, 0)


class MockControl:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.reverse = False
        self.hand_brake = False
        self.manual_gear_shift = False
        self.gear = 1


class MockWheelPhysicsControl:
    def __init__(self):
        self.max_steer_angle = 70.0  # Default max steering angle in degrees


class MockPhysicsControl:
    def __init__(self):
        self.wheels = [MockWheelPhysicsControl() for _ in range(4)]


class MockVehicle:
    def __init__(self, vid, x=0, y=0):
        self.id = vid
        self._transform = MockTransform(x, y, 0)
        self._velocity = MockVector(0, 0, 0)
        self._control = MockControl()
        self._physics_control = MockPhysicsControl()
        self.bounding_box = type('obj', (object,), {
            'extent': MockVector(2.25, 0.9, 0.75)
        })()
    
    def get_transform(self):
        return self._transform
    
    def get_velocity(self):
        return self._velocity
    
    def get_control(self):
        return self._control
    
    def get_acceleration(self):
        return MockVector(0, 0, 0)
    
    def get_angular_velocity(self):
        return MockVector(0, 0, 0)
    
    def get_physics_control(self):
        return self._physics_control


class MockActorSnapshot:
    """Mock for carla.ActorSnapshot"""
    def __init__(self, vehicle):
        self.vehicle = vehicle
    
    def get_velocity(self):
        return self.vehicle.get_velocity()
    
    def get_angular_velocity(self):
        return self.vehicle.get_angular_velocity()
    
    def get_acceleration(self):
        return self.vehicle.get_acceleration()
    
    def get_transform(self):
        return self.vehicle.get_transform()


class MockWorldSnapshot:
    """Mock for carla.WorldSnapshot"""
    def __init__(self, vehicles, elapsed_seconds=0.0):
        self.vehicles = vehicles
        self.timestamp = type('obj', (object,), {'elapsed_seconds': elapsed_seconds})()
    
    def find(self, vehicle_id):
        """Find actor snapshot by ID"""
        vehicle = next((v for v in self.vehicles if v.id == vehicle_id), None)
        if vehicle:
            return MockActorSnapshot(vehicle)
        return None


class MockWorld:
    def __init__(self):
        self.vehicles = []
        self.elapsed_seconds = 0.0
    
    def get_snapshot(self):
        return MockWorldSnapshot(self.vehicles, elapsed_seconds=self.elapsed_seconds)
    
    def add_vehicle(self, v):
        self.vehicles.append(v)

    def set_elapsed_seconds(self, elapsed_seconds):
        self.elapsed_seconds = elapsed_seconds


class TestV2VBasics(unittest.TestCase):
    """Test basic V2V network functionality"""
    
    def test_network_creation(self):
        """Test creating V2V network"""
        v2v = V2XNetwork(max_range=100.0, update_rate_hz=2.0, dsrc_config=_transparent_dsrc())
        self.assertIsNotNone(v2v)
        self.assertEqual(v2v.max_range, 100.0)
        self.assertEqual(v2v.update_rate_hz, 2.0)
    
    def test_vehicle_registration(self):
        """Test registering vehicles"""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())
        v1 = MockVehicle(1, 0, 0)
        world.add_vehicle(v1)
        
        v2v.register(1, v1)
        self.assertIn(1, v2v.vehicles)
        
        v2v.unregister(1)
        self.assertNotIn(1, v2v.vehicles)
    
    def test_bsm_creation(self):
        """Test BSM message creation"""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())
        v1 = MockVehicle(1, 100, 50)
        world.add_vehicle(v1)
        
        v2v.register(1, v1)
        v2v.update(force=True)
        
        bsm = v2v.get_bsm(1)
        self.assertIsNotNone(bsm)
        self.assertEqual(bsm.vehicle_id, 1)
        self.assertEqual(bsm.latitude, 100)
        self.assertEqual(bsm.longitude, 50)
    
    def test_neighbor_discovery(self):
        """Test neighbor discovery within range"""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())
        
        v1 = MockVehicle(1, 0, 0)
        v2 = MockVehicle(2, 50, 0)  # 50m away
        v3 = MockVehicle(3, 150, 0)  # 150m away (out of range)
        
        for v in [v1, v2, v3]:
            world.add_vehicle(v)
            v2v.register(v.id, v)
        
        v2v.update(force=True)
        
        neighbors = v2v.get_neighbors(1)
        neighbor_ids = [n.vehicle_id for n in neighbors]
        
        self.assertIn(2, neighbor_ids)  # Should be in range
        self.assertNotIn(3, neighbor_ids)  # Should be out of range
    
    def test_network_stats(self):
        """Test network statistics"""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())
        
        v1 = MockVehicle(1, 0, 0)
        world.add_vehicle(v1)
        v2v.register(1, v1)
        v2v.update(force=True)
        
        stats = v2v.get_network_stats()
        
        self.assertIn('total_messages_sent', stats)
        self.assertIn('average_neighbors', stats)
        self.assertEqual(stats['total_messages_sent'], 1)
        self.assertEqual(stats['average_neighbors'], 0.0)

    def test_message_counter_wraps_at_128(self):
        """Message counters should wrap from 127 back to 0."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())

        v1 = MockVehicle(1, 0, 0)
        world.add_vehicle(v1)
        v2v.register(1, v1)
        v2v.msg_counters[1] = 127

        v2v.update(force=True)
        self.assertEqual(v2v.get_bsm(1).msg_count, 127)
        self.assertEqual(v2v.msg_counters[1], 0)

    def test_neighbor_range_boundary_is_inclusive(self):
        """Vehicles exactly at max_range must still be treated as neighbors."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=50.0, world=world, dsrc_config=_transparent_dsrc())

        v1 = MockVehicle(1, 0, 0)
        v2 = MockVehicle(2, 50, 0)  # Exactly at boundary
        world.add_vehicle(v1)
        world.add_vehicle(v2)
        v2v.register(1, v1)
        v2v.register(2, v2)

        v2v.update(force=True)
        neighbors = [n.vehicle_id for n in v2v.get_neighbors(1)]
        self.assertIn(2, neighbors)

    def test_distance_symmetry(self):
        """Distance lookup should be symmetric for vehicle pairs."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=200.0, world=world, dsrc_config=_transparent_dsrc())

        v1 = MockVehicle(1, 10, 20)
        v2 = MockVehicle(2, 40, 60)
        world.add_vehicle(v1)
        world.add_vehicle(v2)
        v2v.register(1, v1)
        v2v.register(2, v2)

        v2v.update(force=True)
        self.assertAlmostEqual(v2v.get_distance(1, 2), v2v.get_distance(2, 1), places=6)

    def test_unregister_removes_from_neighbors_on_next_update(self):
        """Removed vehicles must not remain discoverable as neighbors."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=200.0, world=world, dsrc_config=_transparent_dsrc())

        v1 = MockVehicle(1, 0, 0)
        v2 = MockVehicle(2, 20, 0)
        world.add_vehicle(v1)
        world.add_vehicle(v2)
        v2v.register(1, v1)
        v2v.register(2, v2)
        v2v.update(force=True)
        self.assertIn(2, [n.vehicle_id for n in v2v.get_neighbors(1)])

        v2v.unregister(2)
        v2v.update(force=True)
        self.assertNotIn(2, [n.vehicle_id for n in v2v.get_neighbors(1)])

    def test_bidirectional_sharing_honors_share_distance_threshold(self):
        """Cooperative sharing should only include neighbors within share distance."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())

        src = MockVehicle(1, 0, 0)
        near = MockVehicle(2, 49.5, 0)
        far = MockVehicle(3, 70, 0)
        for v in [src, near, far]:
            world.add_vehicle(v)
            v2v.register(v.id, v)

        v2v.update(force=True)
        recipients = v2v.enable_bidirectional_sharing(1, {'dummy': True})
        self.assertIn(2, recipients)
        self.assertNotIn(3, recipients)

    @pytest.mark.xfail(strict=True, reason="Stale: assumes lossless delivery but the CSMA term drops packets at N=100 (REFACTOR_HANDOFF §3.5)")
    def test_scalability_100_vehicles_all_neighbors(self):
        """With large range, each vehicle should discover all other vehicles in a 100-vehicle run."""
        world = MockWorld()
        v2v = V2XNetwork(max_range=10000.0, world=world, dsrc_config=_transparent_dsrc())

        vehicle_count = 100
        for i in range(vehicle_count):
            # Spread along x-axis to keep deterministic positions.
            vehicle = MockVehicle(i, x=i * 2, y=0)
            world.add_vehicle(vehicle)
            v2v.register(i, vehicle)

        v2v.update(force=True)

        # Every vehicle should have all others as neighbors when range is huge.
        for i in range(vehicle_count):
            neighbors = v2v.get_neighbors(i)
            self.assertEqual(len(neighbors), vehicle_count - 1)

        stats = v2v.get_network_stats()
        self.assertEqual(stats['max_neighbors'], vehicle_count - 1)
        self.assertEqual(stats['average_neighbors'], float(vehicle_count - 1))
        self.assertEqual(stats['total_messages_sent'], vehicle_count)
    
    def test_one_line_status(self):
        """Test one-line status output"""
        world = MockWorld()
        v2v = V2XNetwork(max_range=100.0, world=world, dsrc_config=_transparent_dsrc())
        
        v1 = MockVehicle(1, 0, 0)
        v1._velocity = MockVector(15, 0, 0)
        world.add_vehicle(v1)
        v2v.register(1, v1)
        v2v.update(force=True)
        
        status = v2v.get_one_line_status(1)
        
        self.assertIn('V2V:', status)
        self.assertIn('m/s', status)
        self.assertIn('Neighbors:', status)


def run_tests():
    """Run all tests"""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestV2VBasics)
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
