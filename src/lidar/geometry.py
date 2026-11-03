"""Sensor → world transform. Uses the 4x4 matrix CARLA itself provides (``carla.Transform.get_matrix()``).

The previous hand-rolled rotation matrix had a wrong third row for non-zero pitch/roll (handoff L1): for a
vehicle pitched 10 deg and rolled 5 deg a point 6 m up was placed about 2.3 m away from where CARLA puts it.
"""

from __future__ import annotations

import numpy as np


def sensor_to_world(points: np.ndarray, matrix_4x4) -> np.ndarray:
    """Return a copy of ``points`` (structured, x/y/z fields) with x, y, z moved to the world frame.

    ``matrix_4x4`` is ``np.array(sensor_transform.get_matrix())`` of the sensor's WORLD transform.
    """
    m = np.asarray(matrix_4x4, dtype=np.float64)
    if m.shape != (4, 4):
        raise ValueError(f"expected a 4x4 matrix, got {m.shape}")
    xyz = np.column_stack([points["x"], points["y"], points["z"]]).astype(np.float64)
    world = xyz @ m[:3, :3].T + m[:3, 3]
    out = points.copy()
    out["x"], out["y"], out["z"] = world[:, 0], world[:, 1], world[:, 2]
    return out
