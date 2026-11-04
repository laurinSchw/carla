"""Road network for the map view (``GET /api/map/geometry``), built from CARLA's map on the simulation thread.

Every topology edge (one lane between two junction-free waypoints) becomes a polyline with a point about every
``step_m`` metres. Junctions come with an axis-aligned box. The result is plain JSON-able data and is cached per
map name by the caller; ``version`` is a hash of the content so clients can cache with an ETag.

TODO(verify) on a real CARLA 0.9.16: ``Waypoint.next`` stepping and ``Junction.bounding_box`` (a junction box
is assumed to have no rotation; if it has, the box is only approximate).
"""

from __future__ import annotations

import hashlib
import json
from typing import Any, Dict, List, Tuple

MAX_POINTS_PER_LANE = 5000


def _xy(wp) -> Tuple[float, float]:
    loc = wp.transform.location
    return (round(float(loc.x), 2), round(float(loc.y), 2))


def lane_polyline(start, end, step_m: float) -> List[Tuple[float, float]]:
    """Walk from ``start`` towards ``end`` in ``step_m`` steps; stops when ``end`` is within one step or the lane
    ends/branches away. Always begins with start and finishes with end."""
    pts = [_xy(start)]
    cur = start
    target = _xy(end)
    for _ in range(MAX_POINTS_PER_LANE):
        nxt = cur.next(step_m)
        if not nxt:
            break
        cur = nxt[0]
        p = _xy(cur)
        if ((p[0] - target[0]) ** 2 + (p[1] - target[1]) ** 2) ** 0.5 < step_m:
            break
        pts.append(p)
    if pts[-1] != target:
        pts.append(target)
    return pts


def build_map_geometry(carla_map, step_m: float = 2.0) -> Dict[str, Any]:
    lanes: List[List[List[float]]] = []
    junctions: Dict[int, Dict[str, Any]] = {}
    seen = set()
    for a, b in carla_map.get_topology():
        key = (_xy(a), _xy(b))
        if key in seen:
            continue
        seen.add(key)
        lanes.append([list(p) for p in lane_polyline(a, b, step_m)])
        for wp in (a, b):
            if getattr(wp, "is_junction", False):
                j = wp.get_junction()
                if j is not None and j.id not in junctions:
                    bb = j.bounding_box
                    junctions[j.id] = {"id": int(j.id), "bbox": [
                        round(bb.location.x - bb.extent.x, 2), round(bb.location.y - bb.extent.y, 2),
                        round(bb.location.x + bb.extent.x, 2), round(bb.location.y + bb.extent.y, 2)]}
    body = {"map_name": carla_map.name, "step_m": step_m, "lanes": lanes,
            "junctions": [junctions[k] for k in sorted(junctions)]}
    digest = hashlib.sha1(json.dumps(body, separators=(",", ":")).encode()).hexdigest()[:16]
    return {**body, "version": digest}
