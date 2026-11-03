"""A small synthetic city: a 3x3 grid of intersections, closed driving rings, buildings and traffic lights.

Everything is deterministic data. Vehicles do not follow a lane graph: each one is assigned a closed *ring* (a
rounded rectangle around one or more blocks, driven in the lane on its right) and moves along it at the ring's
speed. Rings share road segments (opposite directions use different lanes) and cross at intersections, so there
are meetings, line-of-sight blockers and threats, but no collision avoidance and traffic lights are not obeyed.
That is enough to exercise every code path of the server and the frontend; it is not a traffic model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

GRID = (0.0, 100.0, 200.0)          # x and y of the road centre lines
LANE_OFFSET = 1.75                  # lane centre from the road centre line
ROAD_HALF_WIDTH = 7.0
SIDEWALK_OFFSET = 9.0
CORNER_RADIUS = 10.0
ROAD_EXTENT = (-40.0, 240.0)
JUNCTION_HALF = 9.0


@dataclass
class Ring:
    ring_id: int
    points: np.ndarray               # Nx2, closed polyline (last point connects to first)
    cum: np.ndarray                  # cumulative arc length at each point, cum[-1] = length
    speed_mps: float
    clockwise: bool
    yaw_rad: np.ndarray = field(default=None)  # type: ignore[assignment]   # unwrapped heading at each point

    @property
    def length(self) -> float:
        return float(self.cum[-1])

    def pose(self, s: float) -> Tuple[float, float, float]:
        """x, y, yaw_deg at arc length s (wraps)."""
        s = s % self.length
        x = float(np.interp(s, self.cum, self.points[:, 0]))
        y = float(np.interp(s, self.cum, self.points[:, 1]))
        yaw = float(np.interp(s, self.cum, self.yaw_rad))
        return x, y, (math.degrees(yaw) + 180.0) % 360.0 - 180.0


def _rounded_rect(x0: float, y0: float, x1: float, y1: float, clockwise: bool, r: float = CORNER_RADIUS,
                  arc_steps: int = 24) -> np.ndarray:
    """Closed polyline around a rectangle with rounded corners. Clockwise as seen on a screen where y points down
    (east along the top edge, south along the right edge)."""
    def arc(cx, cy, a0, a1):
        return [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))
                for a in np.linspace(a0, a1, arc_steps)]
    pts: List[Tuple[float, float]] = []
    # clockwise order: top-left corner -> top edge -> top-right corner -> right edge -> bottom-right -> bottom edge ...
    pts += arc(x0 + r, y0 + r, 180, 270)
    pts += arc(x1 - r, y0 + r, 270, 360)
    pts += arc(x1 - r, y1 - r, 0, 90)
    pts += arc(x0 + r, y1 - r, 90, 180)
    if not clockwise:
        pts = pts[::-1]
    arr = np.array(pts + [pts[0]])
    # drop consecutive duplicates
    keep = np.r_[True, np.any(np.abs(np.diff(arr, axis=0)) > 1e-9, axis=1)]
    return arr[keep]


def build_rings() -> List[Ring]:
    specs = []   # (x0, y0, x1, y1, clockwise)
    cells = [(GRID[i], GRID[j], GRID[i + 1], GRID[j + 1]) for i in range(2) for j in range(2)]
    for c in cells:
        specs.append((*c, True))
    for c in cells:
        specs.append((*c, False))
    specs.append((GRID[0], GRID[0], GRID[2], GRID[2], True))       # outer ring, both directions
    specs.append((GRID[0], GRID[0], GRID[2], GRID[2], False))
    specs.append((GRID[0], GRID[0], GRID[2], GRID[1], True))       # top row
    specs.append((GRID[0], GRID[1], GRID[2], GRID[2], False))      # bottom row
    rings = []
    for k, (x0, y0, x1, y1, cw) in enumerate(specs):
        # the lane is on the right of the driving direction: inside for clockwise, outside for counter-clockwise
        d = -LANE_OFFSET if cw else LANE_OFFSET
        pts = _rounded_rect(x0 - d, y0 - d, x1 + d, y1 + d, cw)
        seg = np.hypot(*np.diff(pts, axis=0).T)
        cum = np.r_[0.0, np.cumsum(seg)]
        # heading at a vertex = mean direction of its two segments; unwrapped so interpolation never jumps
        seg_dir = np.arctan2(*np.diff(pts, axis=0).T[::-1])
        prev = np.r_[seg_dir[-1], seg_dir]
        nxt = np.r_[seg_dir, seg_dir[0]]
        yaw = np.unwrap(prev + 0.5 * np.angle(np.exp(1j * (nxt - prev))))
        rings.append(Ring(k, pts, cum, speed_mps=7.0 + 0.7 * k, clockwise=cw, yaw_rad=yaw))
    return rings


def spawn_slots(rings: List[Ring], spacing_m: float = 40.0) -> List[Tuple[int, float]]:
    """(ring_id, arc length) of every spawn point, interleaved over rings so that the first ones are spread."""
    per_ring = [[(r.ring_id, k * spacing_m + 7.0 * r.ring_id) for k in range(int(r.length // spacing_m))] for r in rings]
    out: List[Tuple[int, float]] = []
    for k in range(max(len(p) for p in per_ring)):
        for p in per_ring:
            if k < len(p):
                out.append(p[k])
    return out


# ── buildings ──────────────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Building:
    cx: float
    cy: float
    half_x: float
    half_y: float
    height: float


def build_buildings() -> List[Building]:
    """One block per grid cell, set back from the roads, plus two at the edges."""
    b = []
    heights = [18.0, 26.0, 14.0, 22.0]
    k = 0
    for i in range(2):
        for j in range(2):
            cx, cy = (GRID[i] + GRID[i + 1]) / 2, (GRID[j] + GRID[j + 1]) / 2
            half = 100.0 / 2 - SIDEWALK_OFFSET - 6.0
            b.append(Building(cx, cy, half, half, heights[k % 4]))
            k += 1
    b.append(Building(-25.0, 100.0, 8.0, 40.0, 12.0))
    b.append(Building(225.0, 100.0, 8.0, 40.0, 16.0))
    return b


# ── map topology (what map_geometry.py reads) ──────────────────────────────────────────────────────

def lane_edges() -> List[Tuple[Tuple[float, float], Tuple[float, float], bool]]:
    """Lane segments between junction boundaries: (start, end, is_junction_connector). Both directions."""
    edges = []
    lo, hi = ROAD_EXTENT
    j = JUNCTION_HALF
    for g in GRID:
        # roads along y (x = g) and along x (y = g)
        stops = [lo] + [v for c in GRID for v in (c - j, c + j)] + [hi]
        spans = [(stops[k], stops[k + 1]) for k in range(0, len(stops), 2)]
        for a, b in spans:
            if b - a < 1.0:
                continue
            # direction +, lane offset +LANE_OFFSET on the right; direction -, the other side
            edges.append(((g - LANE_OFFSET, a), (g - LANE_OFFSET, b), False))     # south: right = west
            edges.append(((g + LANE_OFFSET, b), (g + LANE_OFFSET, a), False))     # north: right = east
            edges.append(((a, g + LANE_OFFSET), (b, g + LANE_OFFSET), False))     # east: right = south
            edges.append(((b, g - LANE_OFFSET), (a, g - LANE_OFFSET), False))     # west: right = north
    return edges


def junction_connectors() -> List[Tuple[Tuple[float, float], Tuple[float, float], int]]:
    """Straight lanes through each junction (4 per junction) with the junction id."""
    out = []
    jid = 0
    for gy in GRID:
        for gx in GRID:
            jid += 1
            j = JUNCTION_HALF
            out.append(((gx - LANE_OFFSET, gy - j), (gx - LANE_OFFSET, gy + j), jid))
            out.append(((gx + LANE_OFFSET, gy + j), (gx + LANE_OFFSET, gy - j), jid))
            out.append(((gx - j, gy + LANE_OFFSET), (gx + j, gy + LANE_OFFSET), jid))
            out.append(((gx + j, gy - LANE_OFFSET), (gx - j, gy - LANE_OFFSET), jid))
    return out


def intersections() -> List[Tuple[int, float, float]]:
    out, jid = [], 0
    for gy in GRID:
        for gx in GRID:
            jid += 1
            out.append((jid, gx, gy))
    return out


# ── traffic light cycle ────────────────────────────────────────────────────────────────────────────

GREEN_S, YELLOW_S, ALL_RED_S = 12.0, 2.0, 0.0
CYCLE_S = 2 * (GREEN_S + YELLOW_S)


def light_state(t: float, axis: int, offset_s: float = 0.0) -> Tuple[str, float]:
    """('Green'|'Yellow'|'Red', seconds since this state began) for the light of ``axis`` (0 = north-south,
    1 = east-west) at simulation time t. The axes alternate; ``offset_s`` shifts the intersection's cycle."""
    p = (t + offset_s + axis * (GREEN_S + YELLOW_S)) % CYCLE_S
    if p < GREEN_S:
        return "Green", p
    if p < GREEN_S + YELLOW_S:
        return "Yellow", p - GREEN_S
    return "Red", p - (GREEN_S + YELLOW_S)
