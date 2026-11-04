"""Synthetic LiDAR: rays against a ground plane and oriented boxes (vehicles, pedestrians, buildings).

Not a physics model: no noise, no intensity model, no drop-off. It only has to produce point clouds with the
right structure for the frontend and the server: sensor-frame coordinates, per-ray hit actor id in
``object_idx`` and semantic tags, ordered by channel like CARLA's output.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from src.lidar.transforms import transform_matrix

TAG_ROADS, TAG_SIDEWALK, TAG_BUILDING, TAG_TERRAIN = 1, 2, 3, 10
TAG_PEDESTRIAN, TAG_CAR = 12, 14
SEMANTIC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos_inc", "<f4"),
                           ("object_idx", "<u4"), ("tag", "<u4")])
RAYCAST_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])


@dataclass(frozen=True)
class Box:
    """Oriented box standing on its centre: yaw only (vehicles and walkers stay level)."""
    cx: float
    cy: float
    cz: float
    half_x: float
    half_y: float
    half_z: float
    yaw_deg: float
    actor_id: int
    tag: int


def ray_directions(channels: int, lower_deg: float, upper_deg: float, n_rays: int, horizontal_fov_deg: float,
                   frame: int) -> np.ndarray:
    """Local unit directions (x forward, y right, z up), channel by channel."""
    per_channel = max(n_rays // channels, 1)
    elev = np.radians(np.linspace(lower_deg, upper_deg, channels))
    fov = np.radians(horizontal_fov_deg)
    jitter = ((frame * 0.6180339887) % 1.0) * fov / per_channel          # a different phase every frame
    az = -fov / 2 + jitter + np.arange(per_channel) * fov / per_channel
    el, az = np.meshgrid(elev, az, indexing="ij")
    ce = np.cos(el)
    return np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)], axis=-1).reshape(-1, 3)


def _ground_tag(x: np.ndarray, y: np.ndarray, grid: Sequence[float], road_half: float, sidewalk: float) -> np.ndarray:
    dist = np.minimum(np.min(np.abs(x[:, None] - np.array(grid)[None, :]), axis=1),
                      np.min(np.abs(y[:, None] - np.array(grid)[None, :]), axis=1))
    tag = np.full(x.shape, TAG_TERRAIN, dtype=np.uint32)
    tag[dist < sidewalk] = TAG_SIDEWALK
    tag[dist < road_half] = TAG_ROADS
    return tag


def cast(origin: Tuple[float, float, float], rotation_pyr: Tuple[float, float, float], dirs_local: np.ndarray,
         range_m: float, boxes: Sequence[Box], grid: Sequence[float], road_half: float, sidewalk: float,
         semantic: bool) -> np.ndarray:
    """Structured array of hits in the sensor frame (rays that hit nothing within range are dropped)."""
    m = transform_matrix(origin, rotation_pyr)
    rot = m[:3, :3]
    o = np.array(origin, dtype=np.float64)
    d = dirs_local @ rot.T                                   # world directions
    n = len(d)
    t_best = np.full(n, np.inf)
    hit_id = np.zeros(n, dtype=np.uint32)
    hit_tag = np.zeros(n, dtype=np.uint32)
    normal = np.zeros((n, 3))

    down = d[:, 2] < -1e-9                                   # ground plane z = 0
    t_g = np.full(n, np.inf)
    t_g[down] = -o[2] / d[down, 2]
    ok = (t_g > 0) & (t_g <= range_m)
    t_best[ok] = t_g[ok]
    normal[ok] = (0, 0, 1)
    gx, gy = o[0] + t_g[ok] * d[ok, 0], o[1] + t_g[ok] * d[ok, 1]
    hit_tag[ok] = _ground_tag(gx, gy, grid, road_half, sidewalk)

    for b in boxes:
        radius = float(np.sqrt(b.half_x ** 2 + b.half_y ** 2 + b.half_z ** 2))
        c = np.array([b.cx, b.cy, b.cz]) - o
        c2 = float(c @ c)
        if c2 > (range_m + radius) ** 2:
            continue
        # cheap bounding-sphere test first: only the rays that pass near the box go through the slab test
        proj = d @ c
        cand = np.flatnonzero((proj > 0) & (c2 - proj * proj < radius * radius))
        if cand.size == 0:
            continue
        dc = d[cand]
        cy_, sy_ = np.cos(np.radians(b.yaw_deg)), np.sin(np.radians(b.yaw_deg))
        # box-local frame: rotate by -yaw about z
        rel = o - np.array([b.cx, b.cy, b.cz])
        ol = np.array([cy_ * rel[0] + sy_ * rel[1], -sy_ * rel[0] + cy_ * rel[1], rel[2]])
        dl = np.stack([cy_ * dc[:, 0] + sy_ * dc[:, 1], -sy_ * dc[:, 0] + cy_ * dc[:, 1], dc[:, 2]], axis=1)
        half = np.array([b.half_x, b.half_y, b.half_z])
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / dl
            t1 = (-half - ol) * inv
            t2 = (half - ol) * inv
        tmin = np.minimum(t1, t2)
        tmax = np.maximum(t1, t2)
        tn = np.max(tmin, axis=1)
        tf = np.min(tmax, axis=1)
        ok_box = (tn <= tf) & (tf > 0) & (tn > 0) & (tn <= range_m) & (tn < t_best[cand])
        if not ok_box.any():
            continue
        hit = cand[ok_box]
        t_best[hit] = tn[ok_box]
        hit_id[hit] = b.actor_id
        hit_tag[hit] = b.tag
        axis = np.argmax(tmin[ok_box], axis=1)                 # the face that was entered last
        rows = np.arange(len(axis))
        nl = np.zeros((len(axis), 3))
        nl[rows, axis] = -np.sign(dl[ok_box][rows, axis])
        normal[hit] = np.stack([cy_ * nl[:, 0] - sy_ * nl[:, 1], sy_ * nl[:, 0] + cy_ * nl[:, 1], nl[:, 2]], axis=1)

    valid = np.isfinite(t_best)
    pts_world = o + d[valid] * t_best[valid, None]
    local = (pts_world - o) @ rot                            # inverse rotation: rows are world vectors
    cos_inc = np.abs(np.sum(normal[valid] * d[valid], axis=1))
    if semantic:
        out = np.empty(int(valid.sum()), dtype=SEMANTIC_DTYPE)
        out["cos_inc"], out["object_idx"], out["tag"] = cos_inc, hit_id[valid], hit_tag[valid]
    else:
        out = np.empty(int(valid.sum()), dtype=RAYCAST_DTYPE)
        out["intensity"] = np.clip(cos_inc * np.exp(-0.002 * t_best[valid]), 0.0, 1.0)
    out["x"], out["y"], out["z"] = local[:, 0], local[:, 1], local[:, 2]
    return out
