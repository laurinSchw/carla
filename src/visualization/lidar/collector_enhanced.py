#!/usr/bin/env python3
"""
Enhanced LiDAR Data Collector with Performance Optimizations

Supports multiple quality tiers and CARLA sensor optimizations:
- Dropoff parameters for performance tuning
- Multiple sensor types (semantic, ray_cast, radar)
- Configurable quality presets (low/medium/high)
- Adaptive streaming based on network conditions
"""

import numpy as np
import carla
import logging
import threading
import time as _time
from typing import Dict, Optional, Tuple, List
from enum import Enum

try:
    try:
        from src.utils.binary_protocol import BinaryProtocol
    except ImportError:
        from utils.binary_protocol import BinaryProtocol  # type: ignore
except ImportError:
    BinaryProtocol = None  # type: ignore

logger = logging.getLogger(__name__)


class LiDARQualityPreset(Enum):
    """Pre-configured LiDAR quality settings for different use cases."""
    
    LOW = "low"
    """16-channel, 500K pts/sec, aggressive dropoff (40%) - Best for server architecture"""
    
    MEDIUM = "medium"
    """32-channel, 1M pts/sec, moderate dropoff (20%) - Balanced performance"""
    
    HIGH = "high"
    """64-channel, 2M pts/sec, no dropoff - Demo/offline analysis only"""
    
    RADAR_ONLY = "radar"
    """Radar sensor only - Maximum performance, velocity data"""
    
    HYBRID = "hybrid"
    """Radar + Low-res LiDAR - Best of both worlds"""


# Quality preset configurations
QUALITY_PRESETS = {
    LiDARQualityPreset.LOW: {
        'sensor_type': 'sensor.lidar.ray_cast_semantic',
        'channels': '16',
        'points_per_second': '500000',
        'rotation_frequency': '10.0',
        'range': '100.0',
        'upper_fov': '15.0',
        'lower_fov': '-25.0',
        'horizontal_fov': '360.0',
        'dropoff_general_rate': '0.4',  # 40% points dropped for performance
        'dropoff_intensity_limit': '0.8',
        'dropoff_zero_intensity': '0.2',
    },
    LiDARQualityPreset.MEDIUM: {
        'sensor_type': 'sensor.lidar.ray_cast_semantic',
        'channels': '32',
        'points_per_second': '1000000',
        'rotation_frequency': '20.0',
        'range': '100.0',
        'upper_fov': '15.0',
        'lower_fov': '-30.0',
        'horizontal_fov': '360.0',
        'dropoff_general_rate': '0.2',  # 20% points dropped
        'dropoff_intensity_limit': '0.9',
        'dropoff_zero_intensity': '0.1',
    },
    LiDARQualityPreset.HIGH: {
        'sensor_type': 'sensor.lidar.ray_cast_semantic',
        'channels': '64',
        'points_per_second': '2000000',
        'rotation_frequency': '20.0',
        'range': '150.0',
        'upper_fov': '15.0',
        'lower_fov': '-30.0',
        'horizontal_fov': '360.0',
        'dropoff_general_rate': '0.0',  # No dropoff
        'dropoff_intensity_limit': '1.0',
        'dropoff_zero_intensity': '0.0',
    },
    LiDARQualityPreset.RADAR_ONLY: {
        'sensor_type': 'sensor.other.radar',
        'horizontal_fov': '35.0',
        'vertical_fov': '20.0',
        'points_per_second': '1500',
        'max_range': '200.0',
    },
    LiDARQualityPreset.HYBRID: {
        # Uses both radar and low-res LiDAR
        'lidar': {
            'sensor_type': 'sensor.lidar.ray_cast',  # Non-semantic for speed
            'channels': '16',
            'points_per_second': '500000',
            'rotation_frequency': '10.0',
            'range': '80.0',
            'dropoff_general_rate': '0.3',
        },
        'radar': {
            'sensor_type': 'sensor.other.radar',
            'horizontal_fov': '35.0',
            'vertical_fov': '20.0',
            'max_range': '200.0',
        }
    },
}


class LiDARDataCollector:
    """
    Enhanced LiDAR data collector with performance optimizations.
    
    Features:
    - Multiple quality presets for different performance needs
    - CARLA dropoff parameters for sensor optimization
    - Support for radar and hybrid sensor configurations
    - Thread-safe data collection
    - Adaptive streaming support
    """
    
    def __init__(
        self, 
        world: carla.World, 
        downsample_factor: int = 1,
        quality_preset: LiDARQualityPreset = LiDARQualityPreset.MEDIUM,
        custom_config: Optional[Dict] = None,
    ):
        """
        Initialize enhanced LiDAR data collector.
        
        Args:
            world: CARLA world instance
            downsample_factor: Keep every Nth point (1=no downsampling)
            quality_preset: Pre-configured quality setting
            custom_config: Optional custom configuration to override preset
        """
        self.world = world
        self.downsample_factor = downsample_factor
        self.quality_preset = quality_preset
        self.custom_config = custom_config or {}
        
        # Merge preset with custom config
        if quality_preset != LiDARQualityPreset.HYBRID:
            self.sensor_config = QUALITY_PRESETS[quality_preset].copy()
            self.sensor_config.update(custom_config)
        else:
            # Hybrid mode has nested config
            self.sensor_config = QUALITY_PRESETS[LiDARQualityPreset.HYBRID]['lidar'].copy()
            self.sensor_config.update(custom_config.get('lidar', {}))
            self.radar_config = QUALITY_PRESETS[LiDARQualityPreset.HYBRID]['radar'].copy()
            self.radar_config.update(custom_config.get('radar', {}))
        
        self.vehicles: Dict[int, carla.Actor] = {}
        self.lidar_sensors: Dict[int, carla.Sensor] = {}
        self.radar_sensors: Dict[int, carla.Sensor] = {}
        self.latest_data: Dict[int, Optional[np.ndarray]] = {}
        self.latest_radar_data: Dict[int, Optional[np.ndarray]] = {}
        self.vehicle_transforms: Dict[int, carla.Transform] = {}
        self.actor_ids: Dict[int, int] = {}
        
        # Performance metrics
        self._lock = threading.RLock()
        self._is_active = True
        self._data_received_count = 0
        self._last_frame_time: Dict[int, float] = {}
        self._avg_fps: Dict[int, float] = {}
        
        logger.info(f"LiDAR collector initialized with {quality_preset.value} quality preset")
        
    def register_vehicle(self, vehicle_id: int, vehicle: carla.Actor):
        """
        Register a vehicle and attach sensors based on quality preset.
        
        Args:
            vehicle_id: Unique vehicle identifier
            vehicle: CARLA vehicle actor
        """
        self.vehicles[vehicle_id] = vehicle
        self.actor_ids[vehicle_id] = vehicle.id
        self.latest_data[vehicle_id] = None
        self.latest_radar_data[vehicle_id] = None
        self._last_frame_time[vehicle_id] = 0.0
        self._avg_fps[vehicle_id] = 0.0
        
        # Attach sensors based on preset
        if self.quality_preset == LiDARQualityPreset.RADAR_ONLY:
            self._attach_radar_sensor(vehicle_id, vehicle)
        elif self.quality_preset == LiDARQualityPreset.HYBRID:
            self._attach_lidar_sensor(vehicle_id, vehicle, self.sensor_config)
            self._attach_radar_sensor(vehicle_id, vehicle, self.radar_config)
        else:
            self._attach_lidar_sensor(vehicle_id, vehicle, self.sensor_config)
        
        logger.info(f"Registered vehicle {vehicle_id} with {self.quality_preset.value} sensors")
    
    def _attach_lidar_sensor(self, vehicle_id: int, vehicle: carla.Actor, config: Dict):
        """Attach LiDAR sensor to vehicle with optimized configuration."""
        bp_lib = self.world.get_blueprint_library()
        sensor_type = config.get('sensor_type', 'sensor.lidar.ray_cast_semantic')
        
        lidar_bp = bp_lib.find(sensor_type)
        if lidar_bp is None:
            logger.error(f"Sensor type {sensor_type} not found")
            return
        
        # Apply all configuration parameters
        for key, value in config.items():
            if key != 'sensor_type':
                try:
                    lidar_bp.set_attribute(key, str(value))
                except Exception as e:
                    logger.warning(f"Failed to set {key}={value}: {e}")
        
        # Mount on vehicle roof
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.4))
        lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        
        # Setup callback
        lidar_sensor.listen(lambda data: self._on_lidar_data(vehicle_id, data))
        
        self.lidar_sensors[vehicle_id] = lidar_sensor
        logger.debug(f"LiDAR sensor attached to vehicle {vehicle_id}")
    
    def _attach_radar_sensor(self, vehicle_id: int, vehicle: carla.Actor, config: Optional[Dict] = None):
        """Attach radar sensor to vehicle."""
        config = config or self.sensor_config
        bp_lib = self.world.get_blueprint_library()
        
        radar_bp = bp_lib.find('sensor.other.radar')
        
        # Apply radar-specific config
        for key, value in config.items():
            if key != 'sensor_type':
                try:
                    radar_bp.set_attribute(key, str(value))
                except Exception as e:
                    logger.warning(f"Failed to set radar {key}={value}: {e}")
        
        # Mount on vehicle roof, slightly offset from LiDAR
        radar_transform = carla.Transform(carla.Location(x=0.5, z=2.0))
        radar_sensor = self.world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)
        
        # Setup callback
        radar_sensor.listen(lambda data: self._on_radar_data(vehicle_id, data))
        
        self.radar_sensors[vehicle_id] = radar_sensor
        logger.debug(f"Radar sensor attached to vehicle {vehicle_id}")
    
    def _on_lidar_data(self, vehicle_id: int, data):
        """Callback for LiDAR data reception with performance tracking."""
        if not self._is_active:
            return
            
        try:
            # Calculate FPS
            current_time = _time.time()
            last_time = self._last_frame_time.get(vehicle_id, current_time)
            fps = 1.0 / max(0.001, current_time - last_time)
            self._last_frame_time[vehicle_id] = current_time
            
            # Update running average FPS
            old_fps = self._avg_fps.get(vehicle_id, fps)
            self._avg_fps[vehicle_id] = 0.9 * old_fps + 0.1 * fps
            
            # Convert binary data to numpy array
            points = np.frombuffer(data.raw_data, dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('cos_inc_angle', np.float32), ('object_idx', np.uint32), 
                ('object_tag', np.uint32)
            ]))
            
            # Apply downsampling
            if self.downsample_factor > 1 and len(points) > 0:
                points = points[::self.downsample_factor]
            
            # Thread-safe update
            with self._lock:
                if not self._is_active:
                    return
                    
                self.vehicle_transforms[vehicle_id] = data.transform
                self.latest_data[vehicle_id] = points
                self._data_received_count += 1
                
                # Log performance metrics every 100 frames
                if self._data_received_count % 100 == 0:
                    logger.debug(
                        f"LiDAR vehicle {vehicle_id}: "
                        f"{len(points)} points, {self._avg_fps.get(vehicle_id, 0):.1f} FPS"
                    )
                    
        except Exception as e:
            logger.error(f"Error processing LiDAR data for vehicle {vehicle_id}: {e}")
    
    def _on_radar_data(self, vehicle_id: int, data):
        """Callback for radar data reception."""
        if not self._is_active:
            return
            
        try:
            # Radar data format: [velocity, altitude, azimuth, depth]
            points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
            
            with self._lock:
                self.latest_radar_data[vehicle_id] = points
                
        except Exception as e:
            logger.error(f"Error processing radar data for vehicle {vehicle_id}: {e}")
    
    def transform_to_world_coords(self, vehicle_id: int, local_points: np.ndarray) -> np.ndarray:
        """Transform local sensor coordinates to world coordinates."""
        if vehicle_id not in self.vehicle_transforms:
            return local_points
            
        transform = self.vehicle_transforms[vehicle_id]
        location = transform.location
        rotation = transform.rotation
        
        # Convert rotation to radians
        yaw = np.radians(rotation.yaw)
        pitch = np.radians(rotation.pitch)
        roll = np.radians(rotation.roll)
        
        # Rotation matrices
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        
        rotation_matrix = np.array([
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
        
        # Apply transformation
        local_xyz = np.column_stack([local_points['x'], local_points['y'], local_points['z']])
        world_xyz = (rotation_matrix @ local_xyz.T).T
        world_xyz[:, 0] += location.x
        world_xyz[:, 1] += location.y
        world_xyz[:, 2] += location.z
        
        # Create output with world coordinates
        world_points = local_points.copy()
        world_points['x'] = world_xyz[:, 0]
        world_points['y'] = world_xyz[:, 1]
        world_points['z'] = world_xyz[:, 2]
        
        return world_points
    
    def get_combined_pointcloud(self) -> Optional[Dict]:
        """Get combined point cloud from all vehicles in world coordinates."""
        raw = self._get_raw_combined()
        if raw is None:
            return None
        
        combined, ego_transform, ego_actor_id, num_vehicles = raw
        
        return {
            'num_points': int(len(combined)),
            'points': {
                'x': combined['x'].tolist(),
                'y': combined['y'].tolist(),
                'z': combined['z'].tolist(),
                'cos': combined['cos_inc_angle'].tolist(),
                'tag': combined['object_tag'].tolist(),
                'idx': combined['object_idx'].tolist(),
            },
            'ego_transform': ego_transform,
            'ego_actor_id': ego_actor_id,
            'num_vehicles': num_vehicles,
            'quality_preset': self.quality_preset.value,
            'fps': {str(k): round(v, 2) for k, v in self._avg_fps.items()},
        }
    
    def _get_raw_combined(self) -> Optional[Tuple]:
        """Thread-safe internal helper for combined point cloud data."""
        if not self._is_active:
            return None
        
        all_points = []
        ego_transform = None
        ego_actor_id = 0
        
        with self._lock:
            if not self._is_active:
                return None
            
            for vehicle_id, points in self.latest_data.items():
                if points is not None and len(points) > 0:
                    world_points = self.transform_to_world_coords(vehicle_id, points)
                    all_points.append(world_points)
                    
                    if vehicle_id == 0 and vehicle_id in self.vehicle_transforms:
                        transform = self.vehicle_transforms[vehicle_id]
                        ego_transform = {
                            'x': float(transform.location.x),
                            'y': float(transform.location.y),
                            'z': float(transform.location.z),
                            'yaw': float(transform.rotation.yaw),
                            'pitch': float(transform.rotation.pitch),
                            'roll': float(transform.rotation.roll),
                        }
                        ego_actor_id = int(self.actor_ids.get(0, 0))
            
            num_vehicles = len(self.vehicles)
        
        if not all_points:
            return None
        
        combined = np.concatenate(all_points)
        return combined, ego_transform, ego_actor_id, num_vehicles
    
    def get_combined_pointcloud_binary(self) -> Optional[bytes]:
        """Get binary encoded point cloud for WebSocket streaming."""
        if BinaryProtocol is None:
            logger.warning("BinaryProtocol not available")
            return None
        
        raw = self._get_raw_combined()
        if raw is None:
            return None
        
        combined, ego_transform, ego_actor_id, num_vehicles = raw
        
        return BinaryProtocol.encode_pointcloud(
            combined,
            ego_transform,
            ego_actor_id,
            num_vehicles
        )
    
    def is_active(self) -> bool:
        """Check if collector is active."""
        return self._is_active
    
    def get_performance_stats(self) -> Dict:
        """Get performance statistics for monitoring."""
        with self._lock:
            return {
                'quality_preset': self.quality_preset.value,
                'vehicles': len(self.vehicles),
                'total_frames': self._data_received_count,
                'fps': {str(k): round(v, 2) for k, v in self._avg_fps.items()},
                'downsample_factor': self.downsample_factor,
                'active': self._is_active,
            }
    
    def cleanup(self):
        """Clean up all sensors and resources."""
        self._is_active = False
        
        # Destroy all sensors
        sensors_to_destroy = (
            list(self.lidar_sensors.values()) + 
            list(self.radar_sensors.values())
        )
        
        for sensor in sensors_to_destroy:
            try:
                if sensor and sensor.is_alive:
                    sensor.destroy()
            except Exception as e:
                logger.warning(f"Error destroying sensor: {e}")
        
        self.lidar_sensors.clear()
        self.radar_sensors.clear()
        self.vehicles.clear()
        self.latest_data.clear()
        self.latest_radar_data.clear()
        
        logger.info("LiDAR collector cleanup complete")
