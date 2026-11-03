"""
V2X Protocol Messages — Infrastructure and VRU Communication

Extends the V2V BSM/CAM/DENM stack with:
  - SAE J2735 / ETSI TS 103 301  MAP + SPAT   (V2I — signal phase data)
  - ETSI TS 103 324               CPM         (Collective Perception)
  - ETSI TS 103 300-3             VRUAM       (Vulnerable Road User Awareness)

Standards:
  SAE J2735-202007 — DSRC Message Set Dictionary
  ETSI TS 103 301 v2.1.1 — ITS; Vehicular Communications; Basic Set of
      Applications; Facilities layer protocols and communication requirements
      for infrastructure services (MAP, SPAT, IVIM).
  ETSI TS 103 324 v2.1.1 — ITS; Vehicular Communications; Collective
      Perception Service (CPS) — object sharing from any ITS station.
  ETSI TS 103 300-3 v2.1.1 — ITS; Vehicular Communications; VRU Awareness
      Basic Service (VRUAM) — pedestrians, cyclists, motorcyclists.

Note: All position fields carry CARLA world (x, y, z) coordinates, not
geodetic degrees — consistent with the BSMCore x_m / y_m / z_m fields.

Vocabs : 
    - Intelligent Transport System Station (ITS-S) — any device capable of V2X communication, including vehicles, roadside units, and vulnerable road users (pedestrians, cyclists).
    - Road Side Unit (RSU) — a fixed ITS-S installed at an intersection or along the road, broadcasting MAP and SPAT messages.
    - Vulnerable Road User (VRU) — a pedestrian, cyclist, or motorcyclist equipped with a VRU Awareness Message (VRUAM) capable device.
    - MAP Message — a message describing the geometry of an intersection, including lanes and their attributes.
    - SPAT Message — a message conveying the current signal phase and timing at an intersection.
    - IVIM Message — Intersection Vehicle Information Message, providing additional information about the intersection.
    - CPM Message — Collective Perception Message, sharing perceived objects detected by sensors.
    - VRUAM Message — Vulnerable Road User Awareness Message, broadcasting the position and motion state
    - CPS — Collective Perception Service, enabling ITS stations to share sensor-detected objects.
    - SPAT — Signal Phase and Timing, providing information about traffic signal states.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Dict


# ── VRU Profile (ETSI TS 103 300-3 §6.1.3) ────────────────────────────────────

class VRUProfile(IntEnum):
    """
    VRU device / user classification.

    Maps to ETSI TS 103 300-3 VRU Profile values.
    """
    PEDESTRIAN   = 0   # ITS-S on foot
    CYCLIST      = 1   # Bicycle, e-bike, cargo-bike
    MOTORCYCLIST = 2   # Powered two-wheeled vehicle (≤ 400 cc)
    ANIMAL       = 3   # Any animal on carriageway
    OTHER        = 255


# ── CPM object classification (ETSI TS 103 324 §6.1.3) ───────────────────────

class ObjectClassification(IntEnum):
    """Perceived object type as reported in a CPM."""
    UNKNOWN    = 0
    PEDESTRIAN = 1
    CYCLIST    = 2
    VEHICLE    = 3
    ANIMAL     = 4
    OBSTACLE   = 5


# ══════════════════════════════════════════════════════════════════════════════
# MAP + SPAT  (SAE J2735 §7.4 / §7.18 — ETSI TS 103 301)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class MAPLane:
    """
    Single lane descriptor inside a MAP intersection.

    Corresponds to SAE J2735 GenericLane.  Minimal set of fields sufficient
    for lane-level routing and SPAT association.
    """
    lane_id: int
    direction: str                        # "ingress" | "egress"
    shared_with: List[int] = field(default_factory=list)  # Connected lane IDs
    speed_limit_ms: float = 13.89         # 50 km/h default


@dataclass
class MAPMessage:
    """
    MAP Data Message — SAE J2735 §7.4 / ETSI TS 103 301 v2.1.1.

    Describes intersection geometry so receiving vehicles can associate SPAT
    phase information with specific approach / departure lanes.  The message
    is transmitted by an RSU and cached by vehicles; it changes only when road
    geometry changes (revision counter bump).

    ref_point carries CARLA world (x, y, z) coordinates — not geodetic.
    """
    intersection_id: int
    revision: int                         # Increments on geometry change
    ref_point: tuple                      # (x, y, z) CARLA world stop-bar centre
    lane_width_m: float = 3.5             # Nominal lane width (m)
    speed_limit_ms: float = 13.89         # 50 km/h default
    lanes: List[MAPLane] = field(default_factory=list)
    timestamp: float = 0.0                # RSU generation time (simulation seconds)


@dataclass
class SPATMovementState:
    """
    Signal state for one signal group (approach direction).

    Corresponds to SAE J2735 MovementState / ETSI TS 103 301 MovementEvent.

    movement_event values follow SAE J2735 MovementPhaseState strings:
      "unavailable", "dark", "stop-And-Remain", "stop-Then-Proceed",
      "permissiveMovementAllowed", "protectedMovementAllowed",
      "permissiveClearance", "protectedClearance",
      "caution-Conflicting-Traffic"
    """
    signal_group_id: int
    movement_event: str                   # SAE J2735 MovementPhaseState string
    min_end_time_s: Optional[float]       # Earliest end of current phase (sim seconds); None = unknown
    max_end_time_s: Optional[float]       # Latest end of current phase (sim seconds); None = unknown
    likely_time_s: Optional[float] = None # Most-probable end time; None = unknown


@dataclass
class SPATMessage:
    """
    Signal Phase and Timing Message — SAE J2735 §7.18 / ETSI TS 103 301 v2.1.1.

    Broadcast by RSUs at 2 Hz so that approaching vehicles can plan their
    trajectory through an intersection (speed advisory, stop-line timing).
    """
    intersection_id: int
    intersection_name: str
    timestamp: float                      # Simulation time at generation (seconds)
    status: str = "running"               # "running" | "shutdown" | "standby"
    states: List[SPATMovementState] = field(default_factory=list)


# ══════════════════════════════════════════════════════════════════════════════
# CPM  (ETSI TS 103 324 v2.1.1)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class PerceivedObject:
    """
    Single object entry in a Collective Perception Message.

    Corresponds to ETSI TS 103 324 PerceivedObject.  Position is relative to
    the sender in CARLA world-coordinate deltas (metres): dx = East, dy = North.
    heading and speed are absolute (world frame).
    """
    object_id: int
    object_type: ObjectClassification
    # Relative position from sender (CARLA world delta, metres)
    dx: float
    dy: float
    dz: float = 0.0
    # Absolute kinematics
    speed: float = 0.0                    # m/s
    heading: float = 0.0                  # degrees (world frame)
    # Bounding box (half-extents)
    length_m: float = 0.5
    width_m: float = 0.5
    # Detection confidence [0.0–1.0]
    confidence: float = 1.0
    # Measurement age (seconds since last sensor update)
    measurement_age_s: float = 0.0


@dataclass
class SensorInfo:
    """
    Sensor metadata included in a CPM sender's perception block.

    Corresponds to ETSI TS 103 324 SensorInformation.
    """
    sensor_id: int
    sensor_type: str                      # "lidar" | "radar" | "camera" | "fusion"
    detection_range_m: float = 100.0
    horizontal_fov_deg: float = 360.0


@dataclass
class CPMMessage:
    """
    Collective Perception Message — ETSI TS 103 324 v2.1.1.

    Transmitted by any equipped ITS station (vehicle or RSU) to share
    sensor-detected objects with nearby stations.  Complements BSM/CAM
    by distributing an environment model beyond the sender's own state.

    sender_position carries CARLA world (x, y, z) coordinates.
    """
    station_id: int
    timestamp: float                      # Simulation time (seconds)
    sender_position: tuple                # (x, y, z) CARLA world coords
    management_container: Dict = field(default_factory=dict)
    perceived_objects: List[PerceivedObject] = field(default_factory=list)
    sensor_info_list: List[SensorInfo] = field(default_factory=list)


# ══════════════════════════════════════════════════════════════════════════════
# VRUAM  (ETSI TS 103 300-3 v2.1.1)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class VRUAwarenessMessage:
    """
    VRU Awareness Message — ETSI TS 103 300-3 v2.1.1.

    Broadcast by Vulnerable Road User ITS stations (pedestrians, cyclists,
    motorcyclists).  The VRU basic service runs at 2–25 Hz depending on motion
    state; this simulation uses 2 Hz to match the BSM/CAM beacon cadence.

    position carries CARLA world (x, y, z) coordinates.
    """
    station_id: int
    timestamp: float
    vru_profile: VRUProfile
    position: tuple                       # (x, y, z) CARLA world coordinates
    speed: float = 0.0                    # m/s
    heading: float = 0.0                  # degrees (world frame)
    # Cluster membership — ETSI TS 103 300-3 §7.5
    cluster_id: Optional[int] = None
    is_cluster_leader: bool = False
    # Carriageway context
    is_on_road: bool = False              # True when VRU crosses carriageway
    # Motion state
    motion_state: str = "moving"          # "stationary" | "moving" | "stopped"
