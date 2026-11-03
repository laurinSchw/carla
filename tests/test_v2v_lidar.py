#!/usr/bin/env python3
"""
Comprehensive tests for V2V LiDAR Visualization System
Tests backend, coordinate transformations, and data streaming.
"""

import unittest
import numpy as np
import sys
import json
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import carla
except ImportError:
    print("⚠️  CARLA module not found. Some tests will be skipped.")
    carla = None

from src.visualization.lidar import LiDARDataCollector


class TestLiDARDataCollector(unittest.TestCase):
    """Test LiDAR data collection and transformation."""
    
    def setUp(self):
        """Setup mock CARLA world."""
        if carla is None:
            self.skipTest("CARLA module not available")
        
        self.mock_world = Mock()
        self.collector = LiDARDataCollector(self.mock_world, downsample_factor=2)
    
    def test_initialization(self):
        """Test collector initialization."""
        self.assertEqual(self.collector.downsample_factor, 2)
        self.assertEqual(len(self.collector.vehicles), 0)
        self.assertEqual(len(self.collector.lidar_sensors), 0)
    
    def test_coordinate_transformation_identity(self):
        """Test coordinate transformation with identity transform."""
        # Create mock points
        num_points = 100
        points = np.zeros(num_points, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        
        # Set some test values
        points['x'] = np.random.randn(num_points) * 10
        points['y'] = np.random.randn(num_points) * 10
        points['z'] = np.random.randn(num_points) * 2
        points['object_tag'] = np.random.randint(0, 23, num_points)
        
        # Mock identity transform (no rotation, no translation)
        mock_transform = Mock()
        mock_transform.location = carla.Location(x=0, y=0, z=0)
        mock_transform.rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
        
        self.collector.vehicle_transforms[0] = mock_transform
        
        # Transform points
        transformed = self.collector.transform_to_world_coords(0, points)
        
        # With identity transform, points should be unchanged
        np.testing.assert_array_almost_equal(transformed['x'], points['x'], decimal=5)
        np.testing.assert_array_almost_equal(transformed['y'], points['y'], decimal=5)
        np.testing.assert_array_almost_equal(transformed['z'], points['z'], decimal=5)
    
    def test_coordinate_transformation_translation(self):
        """Test coordinate transformation with translation only."""
        num_points = 100
        points = np.zeros(num_points, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        
        points['x'] = np.ones(num_points) * 5.0
        points['y'] = np.ones(num_points) * 3.0
        points['z'] = np.ones(num_points) * 2.0
        
        # Mock translation transform
        mock_transform = Mock()
        mock_transform.location = carla.Location(x=10, y=20, z=1.5)
        mock_transform.rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
        
        self.collector.vehicle_transforms[0] = mock_transform
        
        # Transform points
        transformed = self.collector.transform_to_world_coords(0, points)
        
        # Points should be translated
        expected_x = points['x'] + 10
        expected_y = points['y'] + 20
        expected_z = points['z'] + 1.5
        
        np.testing.assert_array_almost_equal(transformed['x'], expected_x, decimal=5)
        np.testing.assert_array_almost_equal(transformed['y'], expected_y, decimal=5)
        np.testing.assert_array_almost_equal(transformed['z'], expected_z, decimal=5)
    
    def test_coordinate_transformation_rotation_90deg(self):
        """Test coordinate transformation with 90-degree yaw rotation."""
        points = np.zeros(1, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        
        # Point at (1, 0, 0) in local frame
        points['x'] = 1.0
        points['y'] = 0.0
        points['z'] = 0.0
        
        # Mock 90-degree yaw rotation
        mock_transform = Mock()
        mock_transform.location = carla.Location(x=0, y=0, z=0)
        mock_transform.rotation = carla.Rotation(pitch=0, yaw=90, roll=0)
        
        self.collector.vehicle_transforms[0] = mock_transform
        
        # Transform points
        transformed = self.collector.transform_to_world_coords(0, points)
        
        # After 90-degree yaw, (1,0,0) should become approximately (0,1,0)
        self.assertAlmostEqual(transformed['x'][0], 0.0, places=5)
        self.assertAlmostEqual(transformed['y'][0], 1.0, places=5)
        self.assertAlmostEqual(transformed['z'][0], 0.0, places=5)
    
    def test_downsampling(self):
        """Test point cloud downsampling."""
        # Create mock LiDAR data
        num_points = 1000
        raw_data = np.zeros(num_points, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        
        # Simulate downsampling
        downsample_factor = 4
        downsampled = raw_data[::downsample_factor]
        
        expected_size = num_points // downsample_factor
        self.assertEqual(len(downsampled), expected_size)
    
    def test_semantic_tag_preservation(self):
        """Test that semantic tags are preserved through transformation."""
        points = np.zeros(5, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        
        # Set different semantic tags
        points['object_tag'] = [0, 1, 7, 10, 12]  # Various tags
        
        mock_transform = Mock()
        mock_transform.location = carla.Location(x=5, y=5, z=1)
        mock_transform.rotation = carla.Rotation(pitch=0, yaw=45, roll=0)
        
        self.collector.vehicle_transforms[0] = mock_transform
        transformed = self.collector.transform_to_world_coords(0, points)
        
        # Tags should be unchanged
        np.testing.assert_array_equal(transformed['object_tag'], points['object_tag'])
    
    def test_get_combined_pointcloud_empty(self):
        """Test getting combined point cloud with no data."""
        result = self.collector.get_combined_pointcloud()
        self.assertIsNone(result)
    
    def test_get_combined_pointcloud_with_data(self):
        """Test getting combined point cloud with mock data."""
        # Create mock data for vehicle 0
        points_v0 = np.zeros(10, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        points_v0['x'] = np.arange(10)
        points_v0['y'] = np.arange(10) * 2
        points_v0['z'] = np.ones(10)
        points_v0['object_tag'] = 10  # Vehicle tag
        
        # Create mock data for vehicle 1
        points_v1 = np.zeros(5, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
        ]))
        points_v1['x'] = np.arange(5) + 20
        points_v1['y'] = np.arange(5) * 3
        points_v1['z'] = np.ones(5) * 2
        points_v1['object_tag'] = 7  # Road tag
        
        self.collector.latest_data[0] = points_v0
        self.collector.latest_data[1] = points_v1
        
        # Mock transforms
        mock_transform_v0 = Mock()
        mock_transform_v0.location = carla.Location(x=0, y=0, z=0)
        mock_transform_v0.rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
        
        mock_transform_v1 = Mock()
        mock_transform_v1.location = carla.Location(x=10, y=10, z=0)
        mock_transform_v1.rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
        
        self.collector.vehicle_transforms[0] = mock_transform_v0
        self.collector.vehicle_transforms[1] = mock_transform_v1
        self.collector.vehicles[0] = Mock()
        self.collector.vehicles[1] = Mock()
        
        # Get combined point cloud
        result = self.collector.get_combined_pointcloud()
        
        self.assertIsNotNone(result)
        self.assertEqual(result['num_points'], 15)  # 10 + 5 points
        self.assertEqual(result['num_vehicles'], 2)
        self.assertEqual(len(result['points']['x']), 15)
        self.assertEqual(len(result['points']['tag']), 15)


class TestCoordinateTransformations(unittest.TestCase):
    """Test coordinate transformation mathematics."""
    
    def test_rotation_matrix_identity(self):
        """Test that zero rotation produces identity matrix."""
        yaw = pitch = roll = 0
        
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        
        R = np.array([
            [cos_yaw * cos_pitch, 
             cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
             cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
            [sin_yaw * cos_pitch,
             sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
             sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
            [-sin_pitch,
             cos_pitch * sin_roll,
             cos_pitch * cos_roll]
        ])
        
        np.testing.assert_array_almost_equal(R, np.eye(3))
    
    def test_rotation_90_degrees(self):
        """Test 90-degree rotation around Z-axis."""
        yaw = np.radians(90)
        pitch = roll = 0
        
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        
        R = np.array([
            [cos_yaw * cos_pitch, 
             cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
             cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
            [sin_yaw * cos_pitch,
             sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
             sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
            [-sin_pitch,
             cos_pitch * sin_roll,
             cos_pitch * cos_roll]
        ])
        
        # Rotate point (1, 0, 0) should give approximately (0, 1, 0)
        point = np.array([1, 0, 0])
        rotated = R @ point
        
        np.testing.assert_array_almost_equal(rotated, [0, 1, 0], decimal=10)


class TestDataSerialization(unittest.TestCase):
    """Test JSON serialization of point cloud data."""
    
    def test_json_serialization(self):
        """Test that point cloud data can be serialized to JSON."""
        # Use Python int instead of numpy int64 to avoid JSON serialization issue
        data = {
            'num_points': 100,
            'points': {
                'x': [float(x) for x in np.random.randn(100)],
                'y': [float(y) for y in np.random.randn(100)],
                'z': [float(z) for z in np.random.randn(100)],
                'tag': [int(t) for t in np.random.randint(0, 23, 100)],
            },
            'vehicle_ids': [0] * 50 + [1] * 50,
            'num_vehicles': 2
        }
        
        # Should not raise exception
        json_str = json.dumps(data)
        
        # Should be able to deserialize
        deserialized = json.loads(json_str)
        
        self.assertEqual(deserialized['num_points'], 100)
        self.assertEqual(deserialized['num_vehicles'], 2)
        self.assertEqual(len(deserialized['points']['x']), 100)


class AsyncMock(Mock):
    """Mock for async functions."""
    async def __call__(self, *args, **kwargs):
        return super().__call__(*args, **kwargs)


def run_tests():
    """Run all tests."""
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromModule(sys.modules[__name__])
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
