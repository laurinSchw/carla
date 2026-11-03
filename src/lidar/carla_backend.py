"""CARLA implementation of ``LidarBackend``. ``carla`` is imported lazily (unit tests run without it)."""

from __future__ import annotations

import logging
import queue
from typing import Optional

import numpy as np

from src.lidar import protocol
from src.lidar.config import LidarConfig
from src.lidar.manager import Measurement

logger = logging.getLogger(__name__)

# raw layout of sensor.lidar.ray_cast_semantic (CARLA 0.9.x) and sensor.lidar.ray_cast
_SEMANTIC_RAW = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos_inc", "<f4"),
                          ("object_idx", "<u4"), ("tag", "<u4")])
_RAYCAST_RAW = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])


class CarlaLidarHandle:
    """One live sensor. The CARLA callback only enqueues; the simulation thread dequeues the measurement of
    the frame it just ticked, so a frame never mixes with state from another frame (handoff L3)."""

    def __init__(self, sensor, config: LidarConfig) -> None:
        self._sensor = sensor
        self._semantic = config.sensor_type == "semantic"
        self._queue: "queue.Queue" = queue.Queue()
        sensor.listen(self._queue.put)

    def get_measurement(self, frame: int, timeout_s: float) -> Optional[Measurement]:
        import time
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                data = self._queue.get(timeout=max(deadline - time.monotonic(), 0.0))
            except queue.Empty:
                return None
            if data.frame < frame:
                continue  # stale (e.g. delivered late for an earlier tick)
            if data.frame > frame:
                return None  # we are behind CARLA; keep it out of this frame
            return self._convert(data)

    def _convert(self, data) -> Measurement:
        raw = np.frombuffer(data.raw_data, dtype=_SEMANTIC_RAW if self._semantic else _RAYCAST_RAW)
        t = data.transform
        return Measurement(
            frame=int(data.frame), points=raw,
            field_mask=protocol.SEMANTIC_FIELDS if self._semantic else protocol.RAYCAST_FIELDS,
            location=(t.location.x, t.location.y, t.location.z),
            rotation=(t.rotation.pitch, t.rotation.yaw, t.rotation.roll),
            sim_time=float(getattr(data, "timestamp", 0.0)),
        )

    def destroy(self) -> None:
        """stop() ends the callbacks; destroy() then needs no sleep (the old collector slept 0.1 s, defect L9)."""
        try:
            self._sensor.stop()
        finally:
            self._sensor.destroy()


class CarlaLidarBackend:
    def __init__(self, world) -> None:
        self._world = world

    def spawn(self, actor_id: int, config: LidarConfig) -> CarlaLidarHandle:
        import carla  # lazy
        actor = self._world.get_actor(actor_id)
        if actor is None:
            raise RuntimeError(f"actor {actor_id} does not exist")
        bp = self._world.get_blueprint_library().find(config.blueprint_id)
        for key, value in config.blueprint_attributes().items():
            bp.set_attribute(key, value)
        m = config.mount
        transform = carla.Transform(
            carla.Location(x=m.x_m, y=m.y_m, z=m.z_m),
            carla.Rotation(pitch=m.pitch_deg, yaw=m.yaw_deg, roll=m.roll_deg),
        )
        sensor = self._world.spawn_actor(bp, transform, attach_to=actor)
        return CarlaLidarHandle(sensor, config)
