#!/usr/bin/env python3
"""
LiDAR Data Collector for V2X Communication
Collects and processes semantic LiDAR data from multiple vehicles.
"""

from __future__ import annotations

import numpy as np
import logging
import threading
import time as _time
from typing import TYPE_CHECKING, Dict, Optional, Tuple

if TYPE_CHECKING:
    import carla

try:
    try:
        from src.utils.binary_protocol import BinaryProtocol
    except ImportError:
        from utils.binary_protocol import BinaryProtocol  # type: ignore
except ImportError:
    BinaryProtocol = None  # type: ignore

logger = logging.getLogger(__name__)


class LiDARDataCollector:
    """Collects and processes semantic LiDAR data from multiple vehicles."""
    
    def __init__(self, world: carla.World, downsample_factor: int = 1, **lidar_config):
        """
        Initialize LiDAR data collector.
        
        Args:
            world: CARLA world instance
            downsample_factor: Keep every Nth point (1=no downsampling, higher=more downsampling)
            **lidar_config: LiDAR sensor configuration (channels, range, points_per_second, etc.)
        """
        self.world = world
        self.downsample_factor = downsample_factor
        self.lidar_config = lidar_config  # Store custom config
        self.vehicles: Dict[int, carla.Actor] = {}
        self.lidar_sensors: Dict[int, carla.Sensor] = {}
        self.latest_data: Dict[int, Optional[np.ndarray]] = {}
        self.vehicle_transforms: Dict[int, carla.Transform] = {}
        self.actor_ids: Dict[int, int] = {}  # Map vehicle_id -> actor_id
        
        # Thread safety
        self._lock = threading.RLock()
        self._is_active = True  # Flag to indicate if collector is active
        self._data_received_count = 0  # Track total data received
        
    def register_vehicle(self, vehicle_id: int, vehicle: carla.Actor):
        """Register a vehicle and attach semantic LiDAR sensor.
        
        Args:
            vehicle_id: Unique vehicle identifier
            vehicle: CARLA vehicle actor
        """
        self.vehicles[vehicle_id] = vehicle
        self.actor_ids[vehicle_id] = vehicle.id  # Store actor ID
        self.latest_data[vehicle_id] = None
        
        # Create semantic LiDAR blueprint
        bp_lib = self.world.get_blueprint_library()
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
        
        # Apply custom config or use defaults
        config = {
            'channels': '64',
            'range': '100.0',
            'points_per_second': '2000000',
            'rotation_frequency': '20.0',
            'upper_fov': '15.0',
            'lower_fov': '-30.0',
            'horizontal_fov': '360.0',
        }
        config.update({k: str(v) for k, v in self.lidar_config.items()})
        
        for key, value in config.items():
            lidar_bp.set_attribute(key, value)
        
        # Attach LiDAR to vehicle roof
        import carla  # lazy: keeps this module importable without CARLA
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.4))
        lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        
        # Setup callback
        lidar_sensor.listen(lambda data: self._on_lidar_data(vehicle_id, data))  # type: ignore
        
        self.lidar_sensors[vehicle_id] = lidar_sensor  # type: ignore
        logger.info(f"Registered vehicle {vehicle_id} with semantic LiDAR")
        
    def _on_lidar_data(self, vehicle_id: int, data):
        """Callback for LiDAR data reception.
        
        Args:
            vehicle_id: Vehicle that generated the data
            data: CARLA SemanticLidarMeasurement
        """
        # Ignore data if collector is being cleaned up
        if not self._is_active:
            return
            
        try:
            # Convert to numpy array: [x, y, z, cos_angle, object_idx, object_tag]
            # CARLA binary layout: x, y, z, cos_inc_angle, object_idx (actor ID), object_tag (semantic label)
            points = np.frombuffer(data.raw_data, dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('cos_inc_angle', np.float32), ('object_idx', np.uint32), ('object_tag', np.uint32)
            ]))
            
            # Downsample only if factor > 1
            if self.downsample_factor > 1:
                points = points[::self.downsample_factor]
            
            # Thread-safe update
            with self._lock:
                # Double-check active flag inside lock
                if not self._is_active:
                    return
                    
                # Get sensor transform directly from measurement (always available, even if vehicle destroyed)
                # Data contains sensor transform at measurement time - no need to query vehicle
                self.vehicle_transforms[vehicle_id] = data.transform
                self.latest_data[vehicle_id] = points
                self._data_received_count += 1
                
                # Debug logging every 100 frames
                if self._data_received_count % 100 == 0:
                    logger.debug(f"LiDAR data received: vehicle {vehicle_id}, total count: {self._data_received_count}, points: {len(points)}")
        except Exception as e:
            logger.error(f"Error processing LiDAR data for vehicle {vehicle_id}: {e}")
        
    def transform_to_world_coords(self, vehicle_id: int, local_points: np.ndarray) -> np.ndarray:
        """Transform local LiDAR coordinates to world coordinates.
        
        Args:
            vehicle_id: Vehicle ID
            local_points: Nx6 array of local points [x, y, z, cos, tag, idx]
            
        Returns:
            Nx6 array with world coordinates [x, y, z, cos, tag, idx]
        """
        if vehicle_id not in self.vehicle_transforms:
            return local_points
            
        transform = self.vehicle_transforms[vehicle_id]
        
        # Extract position and rotation
        location = transform.location
        rotation = transform.rotation
        
        # Convert rotation to radians
        yaw = np.radians(rotation.yaw)
        pitch = np.radians(rotation.pitch)
        roll = np.radians(rotation.roll)
        
        # Create rotation matrix (Unreal Engine coordinate system: X-forward, Y-right, Z-up)
        # Rotation order: Roll -> Pitch -> Yaw (ZYX Euler angles)
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        
        # Combined rotation matrix
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
        
        # Extract XYZ coordinates
        local_xyz = np.column_stack([local_points['x'], local_points['y'], local_points['z']])
        
        # Apply rotation and translation
        world_xyz = (rotation_matrix @ local_xyz.T).T
        world_xyz[:, 0] += location.x
        world_xyz[:, 1] += location.y
        world_xyz[:, 2] += location.z
        
        # Create output array with world coordinates
        world_points = local_points.copy()
        world_points['x'] = world_xyz[:, 0]
        world_points['y'] = world_xyz[:, 1]
        world_points['z'] = world_xyz[:, 2]
        
        return world_points
        
    def _get_raw_combined(self) -> Optional[Tuple]:
        """Thread-safe internal helper: return combined point cloud data.

        Returns:
            (combined structured array, ego_transform dict, ego_actor_id int,
             num_vehicles int) or None if no data is available.
        """
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

    def get_combined_pointcloud(self) -> Optional[Dict]:
        """Get combined point cloud from all vehicles in world coordinates.

        Returns:
            Dictionary with point cloud data or None if no data available.
            The ``points`` sub-dict now includes ``idx`` (object_idx) so the
            frontend can filter ego self-hits.
        """
        raw = self._get_raw_combined()
        if raw is None:
            return None

        combined, ego_transform, _, num_vehicles = raw

        return {
            'num_points': int(len(combined)),
            'points': {
                'x': combined['x'].astype(float).tolist(),
                'y': combined['y'].astype(float).tolist(),
                'z': combined['z'].astype(float).tolist(),
                'tag': combined['object_tag'].astype(int).tolist(),
                'idx': combined['object_idx'].astype(int).tolist(),
            },
            'num_vehicles': num_vehicles,
            'ego_transform': ego_transform,
        }

    def get_combined_pointcloud_binary(self) -> Optional[bytes]:
        """Get combined point cloud encoded as a binary payload.

        Uses :class:`BinaryProtocol` V2 format (52-byte header+transform,
        then num_points × 20 bytes of float32 point data).  Returns *None*
        when no data is available or when the BinaryProtocol import failed.
        """
        if BinaryProtocol is None:
            return None

        raw = self._get_raw_combined()
        if raw is None:
            return None

        combined, ego_transform, ego_actor_id, num_vehicles = raw

        if ego_transform is not None:
            pos = (ego_transform['x'], ego_transform['y'], ego_transform['z'])
            rot = (ego_transform['yaw'], ego_transform['pitch'], ego_transform['roll'])
        else:
            pos = (0.0, 0.0, 0.0)
            rot = (0.0, 0.0, 0.0)

        # Translate all points to ego-sensor-relative coordinates.
        # This makes distance = sqrt(px²+py²+pz²) exactly correct in the frontend,
        # regardless of where in the world the vehicle is — no floating-point
        # cancellation and no stale-header issues.
        combined_rel = combined.copy()
        combined_rel['x'] -= pos[0]
        combined_rel['y'] -= pos[1]
        combined_rel['z'] -= pos[2]

        return BinaryProtocol.encode(
            combined_rel,
            ego_transform=((0.0, 0.0, 0.0), rot),  # ego is at origin in relative frame
            timestamp=_time.time(),
            num_vehicles=num_vehicles,
            ego_actor_id=ego_actor_id,
        )
    
    def cleanup(self):
        """Cleanup sensors - MUST be called before destroying vehicles."""
        logger.info(f"🔧 Starting LiDAR cleanup: {len(self.lidar_sensors)} sensors, {len(self.vehicles)} vehicles, data_received={self._data_received_count}")
        
        # CRITICAL: Set inactive flag FIRST to stop callbacks from updating data
        with self._lock:
            self._is_active = False
            logger.info("✓ Collector marked as inactive - callbacks will be ignored")
        
        # Stop all sensors to prevent new callbacks
        for vehicle_id, sensor in list(self.lidar_sensors.items()):
            try:
                if sensor is not None:
                    sensor.stop()  # Stop listening before destroy
                    logger.debug(f"Stopped sensor for vehicle {vehicle_id}")
            except Exception as e:
                logger.warning(f"Error stopping sensor {vehicle_id}: {e}")
        
        # Small delay to ensure callbacks finish
        import time
        time.sleep(0.1)
        
        # Then destroy sensors
        for vehicle_id, sensor in list(self.lidar_sensors.items()):
            try:
                if sensor is not None:
                    sensor.destroy()
                    logger.debug(f"Destroyed sensor for vehicle {vehicle_id}")
            except Exception as e:
                logger.warning(f"Error destroying sensor {vehicle_id}: {e}")
        
        # Clear all data structures
        with self._lock:
            self.lidar_sensors.clear()
            self.latest_data.clear()
            self.vehicle_transforms.clear()
            self.vehicles.clear()
            self.actor_ids.clear()
        
        logger.info(f"✅ All LiDAR sensors cleaned up successfully (processed {self._data_received_count} frames)")
    
    def is_active(self) -> bool:
        """Check if collector is active and accepting data."""
        return self._is_active
