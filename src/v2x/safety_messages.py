"""
V2V Message Standards - BSM (Basic Safety Message) Implementation

This implementation is based on the SAE J2735 and ETSI ITS-G5 published standards
for Vehicle-to-Vehicle (V2V) communication.

Standards Reference:
- SAE J2735 - Dedicated Short Range Communications (DSRC) Message Set Dictionary
  https://www.sae.org/standards/content/j2735_202007/
- ETSI EN 302 637-2 - Intelligent Transport Systems (ITS); Vehicular Communications;
  Cooperative Awareness Messages (CAM)

Note: SAE J2735 and ETSI ITS-G5 are published industry standards. This is an
original implementation based on the public specification, not derived from
any copyrighted source code.

CARLA Integration:
- Vehicle data extraction uses CARLA Simulator Python API
  CARLA Simulator (MIT License) - https://carla.org/
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import IntEnum
import time
import math


class VehicleType(IntEnum):
    """Vehicle classification"""
    PASSENGER_CAR = 0
    BUS = 1
    TRUCK = 2
    MOTORCYCLE = 3
    EMERGENCY = 4
    UNKNOWN = 99


# ── Vehicle-type detection from CARLA blueprint ID ────────────────────────────
# Maps substrings of vehicle.type_id to VehicleType.
# Checked in order; first match wins.  "passenger_car" catch-all is not listed
# here — any 4-wheeled blueprint that misses all patterns falls back to
# PASSENGER_CAR in _vehicle_type_from_actor().
_BLUEPRINT_TYPE_MAP: list = [
    # Emergency
    ("ambulance",           VehicleType.EMERGENCY),
    ("firetruck",           VehicleType.EMERGENCY),
    ("police",              VehicleType.EMERGENCY),
    # Buses / large passenger vans
    ("fusorosa",            VehicleType.BUS),      # Mitsubishi Fuso Rosa (full bus)
    ("carlacola",           VehicleType.BUS),      # CarlaMotors generic bus
    (".t2",                 VehicleType.BUS),      # VW Transporter T2 / T2_2021 (minibus)
    # Trucks / utility
    ("cybertruck",          VehicleType.TRUCK),    # Tesla Cybertruck
    ("sprinter",            VehicleType.TRUCK),    # Mercedes Sprinter
    # Motorcycles / bicycles / micro-vehicles
    ("harley",              VehicleType.MOTORCYCLE),
    ("kawasaki",            VehicleType.MOTORCYCLE),
    ("vespa",               VehicleType.MOTORCYCLE),
    ("yamaha",              VehicleType.MOTORCYCLE),
    ("ninja",               VehicleType.MOTORCYCLE),
    ("crossbike",           VehicleType.MOTORCYCLE),
    ("century",             VehicleType.MOTORCYCLE),  # diamondback.century
    ("omafiets",            VehicleType.MOTORCYCLE),  # gazelle.omafiets bicycle
    ("low_rider",           VehicleType.MOTORCYCLE),
]


def _vehicle_type_from_actor(vehicle) -> 'VehicleType':
    """
    Derive VehicleType from the CARLA actor's blueprint ID.

    Uses vehicle.type_id (e.g. 'vehicle.ford.ambulance') to classify the
    vehicle.  Falls back to actor.attributes['number_of_wheels'] (a cached
    blueprint dict — no RPC call) so that any two-wheeled blueprint not yet
    in _BLUEPRINT_TYPE_MAP is still classified as MOTORCYCLE rather than
    PASSENGER_CAR.

    Args:
        vehicle: CARLA vehicle actor with .type_id attribute.

    Returns:
        VehicleType enum value.
    """
    type_id = getattr(vehicle, 'type_id', '').lower()

    for fragment, vtype in _BLUEPRINT_TYPE_MAP:
        if fragment in type_id:
            return vtype

    # Wheel-count fallback: two-wheeled vehicles not caught by name lookup.
    # Uses actor.attributes (a cached blueprint dict — no RPC call) rather than
    # get_physics_control() which is a blocking server round-trip.
    try:
        num_wheels = int(getattr(vehicle, 'attributes', {}).get('number_of_wheels', '4'))
        if num_wheels <= 2:
            return VehicleType.MOTORCYCLE
    except (ValueError, TypeError):
        pass

    return VehicleType.PASSENGER_CAR


class BrakingStatus(IntEnum):
    """Braking system status"""
    UNAVAILABLE = 0
    OFF = 1
    ON = 2
    ENGAGED = 3


@dataclass
class BSMCore:
    """
    Basic Safety Message (BSM) - Core Data
    Based on SAE J2735 standard for V2V communication.
    
    This is the essential data transmitted at high frequency (2-10 Hz)
    for cooperative awareness and collision avoidance.
    """
    # Temporal information
    timestamp: float  # Simulation time (seconds)
    msg_count: int  # Message counter (0-127, wraps around)
    
    # Vehicle identification
    vehicle_id: int  # Unique vehicle identifier
    vehicle_type: VehicleType = VehicleType.PASSENGER_CAR
    
    # Position (WGS84 or local coordinate system)
    latitude: float = 0.0  # or X in local coords
    longitude: float = 0.0  # or Y in local coords
    elevation: float = 0.0  # or Z in local coords
    
    # Position accuracy (meters)
    position_accuracy: float = 1.0
    
    # Motion state
    speed: float = 0.0  # m/s
    heading: float = 0.0  # degrees (0-359.9)
    steering_angle: float = 0.0  # degrees
    
    # Acceleration (m/s²)
    longitudinal_accel: float = 0.0
    lateral_accel: float = 0.0
    vertical_accel: float = 0.0
    yaw_rate: float = 0.0  # degrees/second
    
    # Vehicle size (meters)
    vehicle_length: float = 4.5
    vehicle_width: float = 1.8
    vehicle_height: float = 1.5
    
    # Brake status
    brake_status: BrakingStatus = BrakingStatus.UNAVAILABLE
    brake_pressure: float = 0.0  # 0-100%
    
    # Transmission state
    transmission_state: str = "neutral"  # park, reverse, neutral, forward
    
    # Control confidence
    throttle_confidence: float = 100.0  # 0-100%
    brake_confidence: float = 100.0  # 0-100%
    steering_confidence: float = 100.0  # 0-100%


@dataclass
class BSMPartII:
    """
    BSM Part II - Optional Extended Data
    Transmitted less frequently or on-demand for specific scenarios.
    """
    # Path history (breadcrumb trail)
    path_history: List[tuple] = field(default_factory=list)  # [(x, y, time), ...]
    path_prediction: List[tuple] = field(default_factory=list)  # Predicted path
    
    # Special vehicle status
    is_emergency_vehicle: bool = False
    siren_active: bool = False
    lights_active: bool = False
    
    # Weather/road info from sensors
    exterior_lights: str = "off"  # off, low_beam, high_beam, fog
    wiper_status: str = "off"  # off, intermittent, low, high
    
    # V2V specific
    cooperative_status: str = "available"  # available, busy, unavailable


@dataclass
class CooperativeAwarenessMessage:
    """
    CAM (Cooperative Awareness Message) - European standard
    Similar to BSM but with different structure (ETSI ITS-G5)
    """
    station_id: int  # Unique ITS station ID
    generation_time: float  # Timestamp
    
    # Reference position
    reference_position: tuple  # (lat, lon, alt) or (x, y, z)
    
    # High frequency container
    heading: float
    speed: float
    drive_direction: str  # forward, backward
    
    # Vehicle role
    vehicle_role: str  # default, public_transport, emergency, road_work
    
    # Low frequency container (optional, sent every 500ms)
    vehicle_length: float = 4.5
    vehicle_width: float = 1.8
    
    # Path history
    path_history: List[tuple] = field(default_factory=list)


@dataclass
class V2VEnhancedMessage:
    """
    Enhanced V2V message combining BSM with sensor data sharing
    For cooperative perception and sensor fusion scenarios.
    """
    # Core BSM data
    bsm: BSMCore
    
    # Part II (optional extended data)
    bsm_part2: Optional[BSMPartII] = None
    
    # Sensor data sharing (cooperative perception)
    detected_objects: List[Dict] = field(default_factory=list)  # Objects detected by sensors
    lidar_points_summary: Optional[Dict] = None  # Downsampled LiDAR data
    camera_detections: List[Dict] = field(default_factory=list)  # Vision-based detections
    
    # Communication metadata
    transmission_time: float = 0.0  # When message was sent
    reception_time: float = 0.0  # When message was received
    link_quality: float = 100.0  # Signal strength (0-100%)
    hop_count: int = 0  # For multi-hop scenarios
    
    # Message priority
    priority: int = 0  # 0=normal, 1=high, 2=emergency


def calculate_threat_level(ego_bsm: BSMCore, other_bsm: BSMCore) -> tuple:
    """
    Calculate collision threat level between two vehicles.
    
    Args:
        ego_bsm: Ego vehicle BSM
        other_bsm: Other vehicle BSM
    
    Returns:
        (threat_level, time_to_collision, distance)
        threat_level: 0=no threat, 1=low, 2=medium, 3=high, 4=imminent
    """
    # Calculate distance
    dx = other_bsm.latitude - ego_bsm.latitude
    dy = other_bsm.longitude - ego_bsm.longitude
    distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate relative velocity
    ego_vx = ego_bsm.speed * math.cos(math.radians(ego_bsm.heading))
    ego_vy = ego_bsm.speed * math.sin(math.radians(ego_bsm.heading))
    
    other_vx = other_bsm.speed * math.cos(math.radians(other_bsm.heading))
    other_vy = other_bsm.speed * math.sin(math.radians(other_bsm.heading))
    
    rel_vx = other_vx - ego_vx
    rel_vy = other_vy - ego_vy

    if distance > 1e-6:
        # Use radial closing speed along line of sight, not relative speed magnitude.
        ux = dx / distance
        uy = dy / distance
        closing_speed = -(rel_vx * ux + rel_vy * uy)
    else:
        closing_speed = float('inf')
    
    # Time to collision (TTC)
    if closing_speed > 0.1:  # Positive means vehicles are converging
        ttc = distance / closing_speed
    else:
        ttc = float('inf')
    
    # Determine threat level
    if distance > 100 or closing_speed <= 0.1:
        threat = 0  # No threat
    elif ttc > 10:
        threat = 1  # Low threat
    elif ttc > 5:
        threat = 2  # Medium threat
    elif ttc > 2:
        threat = 3  # High threat
    else:
        threat = 4  # Imminent collision
    
    return (threat, ttc, distance)


def create_bsm_from_carla(vehicle, vehicle_id: int, msg_count: int, 
                          snapshot=None, prev_velocity=None, delta_time=0.05) -> BSMCore:
    """
    Create BSM message from CARLA vehicle actor.
    
    CRITICAL: Uses snapshot data for scientifically accurate measurements.
    vehicle.get_velocity() returns CACHED data from previous tick, which is 
    incorrect for synchronous simulation. Always use snapshot.find().get_velocity().
    
    Args:
        vehicle: CARLA vehicle actor
        vehicle_id: Unique vehicle identifier
        msg_count: Message counter
        snapshot: WorldSnapshot for fresh data (REQUIRED for accuracy)
        prev_velocity: Previous velocity for accel calculation
        delta_time: Time since last update
    
    Returns:
        BSMCore instance
    """
    transform = vehicle.get_transform()
    sim_timestamp = time.time()
    
    # Get FRESH velocity from snapshot (not cached from previous tick)
    if snapshot:
        actor_snapshot = snapshot.find(vehicle.id)
        if actor_snapshot:
            transform = actor_snapshot.get_transform()
            velocity = actor_snapshot.get_velocity()
            angular_velocity = actor_snapshot.get_angular_velocity()
            if hasattr(snapshot, 'timestamp') and hasattr(snapshot.timestamp, 'elapsed_seconds'):
                sim_timestamp = float(snapshot.timestamp.elapsed_seconds)
        else:
            # Fallback if actor is missing in snapshot (e.g., destroyed this tick).
            velocity = vehicle.get_velocity()
            angular_velocity = vehicle.get_angular_velocity()
    else:
        # Fallback to cached data (SCIENTIFICALLY INACCURATE)
        velocity = vehicle.get_velocity()
        angular_velocity = vehicle.get_angular_velocity()
    
    # Calculate speed
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    
    # Calculate accelerations
    if prev_velocity is not None:
        long_accel = (speed_ms - prev_velocity) / delta_time if delta_time > 0 else 0.0
    else:
        long_accel = 0.0
    
    # Get vehicle control
    control = vehicle.get_control()
    
    # Calculate vehicle-specific steering angle
    try:
        physics_control = vehicle.get_physics_control()
        max_steer_angle = physics_control.wheels[0].max_steer_angle
        steering_angle = control.steer * max_steer_angle
    except Exception:
        # Fallback to generic approximation if physics control unavailable
        steering_angle = control.steer * 70.0
    
    # Calculate lateral acceleration (simplified centripetal acceleration)
    if abs(control.steer) > 0.01 and speed_ms > 0.1:
        # Use wheelbase from vehicle dimensions
        wheelbase = vehicle.bounding_box.extent.x * 2 * 0.6  # Approximate 60% of length
        try:
            turn_radius = wheelbase / abs(math.tan(math.radians(steering_angle)))
            lateral_accel = (speed_ms ** 2) / turn_radius * math.copysign(1, control.steer)
        except (ZeroDivisionError, ValueError):
            lateral_accel = 0.0
    else:
        lateral_accel = 0.0
    
    # Get yaw rate from angular velocity (convert rad/s to deg/s)
    yaw_rate = math.degrees(angular_velocity.z)
    
    # Determine brake status
    if control.brake > 0.5:
        brake_status = BrakingStatus.ENGAGED
    elif control.brake > 0.1:
        brake_status = BrakingStatus.ON
    else:
        brake_status = BrakingStatus.OFF
    
    # Determine transmission state
    if control.reverse:
        trans_state = "reverse"
    elif control.throttle > 0:
        trans_state = "forward"
    else:
        trans_state = "neutral"
    
    # Get bounding box for vehicle dimensions
    bbox = vehicle.bounding_box
    vehicle_length = bbox.extent.x * 2
    vehicle_width = bbox.extent.y * 2
    vehicle_height = bbox.extent.z * 2
    
    return BSMCore(
        timestamp=sim_timestamp,
        msg_count=msg_count % 128,
        vehicle_id=vehicle_id,
        vehicle_type=_vehicle_type_from_actor(vehicle),
        latitude=transform.location.x,
        longitude=transform.location.y,
        elevation=transform.location.z,
        position_accuracy=0.5,  # Assuming high accuracy in simulation
        speed=speed_ms,
        heading=transform.rotation.yaw % 360,
        steering_angle=steering_angle,
        longitudinal_accel=long_accel,
        lateral_accel=lateral_accel,
        vertical_accel=0.0,  # Road vehicles typically don't need vertical accel
        yaw_rate=yaw_rate,
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        vehicle_height=vehicle_height,
        brake_status=brake_status,
        brake_pressure=control.brake * 100,
        transmission_state=trans_state,
        throttle_confidence=100.0,
        brake_confidence=100.0,
        steering_confidence=100.0
    )


class DENMCauseCode(IntEnum):
    """
    DENM cause codes — ETSI EN 302 637-3 Table 2.
    Describes the type of environmental notification event.
    """
    RESERVED = 0
    TRAFFIC_CONDITION = 1
    ACCIDENT = 2
    ROAD_WORKS = 3
    ADVERSE_WEATHER_ADHESION = 6
    HAZARDOUS_SURFACE = 9
    OBSTACLE_ON_ROAD = 10
    ANIMAL_ON_ROAD = 11
    HUMAN_PRESENCE_ON_ROAD = 14
    WRONG_WAY_DRIVING = 17
    RESCUE_RECOVERY_IN_PROGRESS = 18
    SLOW_VEHICLE = 26
    VEHICLE_BREAKDOWN = 91
    POST_CRASH = 92
    HUMAN_PROBLEM = 93
    STATIONARY_VEHICLE = 94
    EMERGENCY_VEHICLE_APPROACHING = 95
    DANGEROUS_CURVE = 96
    COLLISION_RISK = 97
    SIGNAL_VIOLATION = 98
    DANGEROUS_END_OF_QUEUE = 99


@dataclass
class DENM:
    """
    Decentralized Environmental Notification Message (DENM)
    Based on ETSI EN 302 637-3 — ITS Vehicular Communications.

    Used to alert nearby vehicles of hazardous or relevant road events.
    """
    # Management container
    station_id: int          # Originating ITS station ID (maps to vehicle_id)
    action_id: int           # Sequence number at originating station
    detection_time: float    # Simulation time when event was detected (seconds)
    reference_time: float    # Time of last update (seconds)
    event_position: tuple    # (x, y, z) in local CARLA coordinates

    # Situation container
    cause_code: DENMCauseCode = DENMCauseCode.RESERVED
    subcause_code: int = 0   # Standard-defined per cause_code
    information_quality: int = 0  # 0 (unavailable) – 7 (fully safe/certain)

    # Location / relevance
    relevance_distance: float = 150.0    # metres; default = MEDIUM range
    relevance_traffic_direction: str = "allTrafficDirections"
    event_speed: Optional[float] = None  # m/s at event location
    event_heading: Optional[float] = None  # degrees at event location

    # Termination flag (None = active, "isCancellation" = revoked)
    termination: Optional[str] = None


def create_cam_from_bsm(bsm: 'BSMCore') -> 'CooperativeAwarenessMessage':
    """
    Derive a CAM from an existing BSMCore.

    CAM (ETSI EN 302 637-2) and BSM (SAE J2735) carry equivalent kinematic data.
    This maps the common fields so the API can expose both formats.
    """
    direction = "reverse" if bsm.transmission_state == "reverse" else "forward"
    return CooperativeAwarenessMessage(
        station_id=bsm.vehicle_id,
        generation_time=bsm.timestamp,
        reference_position=(bsm.latitude, bsm.longitude, bsm.elevation),
        heading=bsm.heading,
        speed=bsm.speed,
        drive_direction=direction,
        vehicle_role="default",
        vehicle_length=bsm.vehicle_length,
        vehicle_width=bsm.vehicle_width,
    )


# Message priority constants
PRIORITY_ROUTINE = 0
PRIORITY_HIGH = 1
PRIORITY_EMERGENCY = 2

# Transmission rates (Hz)
BSM_RATE_NORMAL = 2  # 2 Hz for normal driving
BSM_RATE_HIGH = 10  # 10 Hz for critical situations
CAM_RATE = 2  # 2 Hz for CAM messages

# Communication range (meters)
V2V_RANGE_SHORT = 50
V2V_RANGE_MEDIUM = 150
V2V_RANGE_LONG = 300

# Data sharing thresholds
SHARE_SENSOR_DATA_DISTANCE = 50  # Share sensor data within 50m
SHARE_LIDAR_DISTANCE = 30  # Share LiDAR data within 30m
EMERGENCY_BROADCAST_DISTANCE = 300  # Emergency messages go further
