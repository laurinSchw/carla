"""A deterministic stand-in for the CARLA simulator, behind the ``carla`` Python API shape this project uses.

``src.testing.fake_carla.install()`` registers it as the ``carla`` module, so the real server code
(``PersistentCARLAServer``, the V2X network, the LiDAR manager) runs on it unchanged. It is a *test double*:
vehicles move on rings (see ``city.py``), there is no physics, no collision, no traffic-light obedience.
Given the same seed and the same sequence of calls, every tick produces the same state.
"""

from __future__ import annotations

import enum
import fnmatch
import math
import random
import types
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from src.lidar.transforms import compose
from src.testing.fake_carla import city, lidar
from src.testing.fake_carla.geom import BoundingBox, Location, Rotation, Transform, Vector3D, wrap_deg

FIRST_FRAME = 1000
VEHICLE_BLUEPRINTS = [
    "vehicle.tesla.model3", "vehicle.audi.a2", "vehicle.bmw.grandtourer", "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020", "vehicle.ford.mustang", "vehicle.jeep.wrangler_rubicon",
    "vehicle.lincoln.mkz_2020", "vehicle.mercedes.coupe", "vehicle.mini.cooperst", "vehicle.nissan.micra",
    "vehicle.seat.leon", "vehicle.toyota.prius", "vehicle.volkswagen.t2_2021",
]
TWO_WHEELERS = ["vehicle.harley-davidson.low_rider", "vehicle.bh.crossbike"]
CAR_EXTENT = Vector3D(2.3, 0.9, 0.75)
WALKER_EXTENT = Vector3D(0.2, 0.2, 0.9)


class TrafficLightState(enum.Enum):
    Red = 0
    Yellow = 1
    Green = 2
    Off = 3
    Unknown = 4


# ── blueprints ─────────────────────────────────────────────────────────────────────────────────────

class _Attribute:
    def __init__(self, value: str, recommended: Optional[List[str]] = None):
        self.value = value
        self.recommended_values = recommended or [value]

    def __int__(self) -> int:
        return int(self.value)

    def as_int(self) -> int:
        return int(self.value)


class ActorBlueprint:
    def __init__(self, bid: str, attributes: Dict[str, _Attribute]):
        self.id = bid
        self._attrs = attributes

    def has_attribute(self, name: str) -> bool:
        return name in self._attrs

    def set_attribute(self, name: str, value: Any) -> None:
        if name not in self._attrs:
            raise IndexError(f"blueprint {self.id} has no attribute {name}")
        self._attrs[name] = _Attribute(str(value), self._attrs[name].recommended_values)

    def get_attribute(self, name: str) -> _Attribute:
        return self._attrs[name]


class BlueprintLibrary:
    def __init__(self):
        self._bps: List[ActorBlueprint] = []
        for bid in VEHICLE_BLUEPRINTS:
            self._bps.append(ActorBlueprint(bid, {
                "number_of_wheels": _Attribute("4"),
                "color": _Attribute("255,255,255", ["255,255,255", "30,30,30", "180,0,0", "0,60,160"]),
                "role_name": _Attribute("autopilot")}))
        for bid in TWO_WHEELERS:
            self._bps.append(ActorBlueprint(bid, {"number_of_wheels": _Attribute("2"), "role_name": _Attribute("autopilot")}))
        for i in range(1, 9):
            self._bps.append(ActorBlueprint(f"walker.pedestrian.{i:04d}", {"is_invincible": _Attribute("true"),
                                                                         "role_name": _Attribute("walker")}))
        self._bps.append(ActorBlueprint("controller.ai.walker", {}))
        for kind in ("ray_cast", "ray_cast_semantic"):
            attrs = {n: _Attribute(v) for n, v in [("channels", "32"), ("range", "100"), ("points_per_second", "56000"),
                                                    ("rotation_frequency", "10"), ("upper_fov", "10"),
                                                    ("lower_fov", "-30"), ("horizontal_fov", "360")]}
            if kind == "ray_cast":
                attrs.update({n: _Attribute("0.0") for n in ("noise_stddev", "dropoff_general_rate",
                                                               "dropoff_intensity_limit", "dropoff_zero_intensity",
                                                               "atmosphere_attenuation_rate")})
            self._bps.append(ActorBlueprint(f"sensor.lidar.{kind}", attrs))

    def find(self, bid: str) -> ActorBlueprint:
        for bp in self._bps:
            if bp.id == bid:
                return _fresh(bp)
        raise IndexError(f"blueprint {bid} not found")

    def filter(self, pattern: str) -> List[ActorBlueprint]:
        return [_fresh(bp) for bp in self._bps if fnmatch.fnmatch(bp.id, pattern)]

    def __iter__(self):
        return iter(self._bps)


def _fresh(bp: ActorBlueprint) -> ActorBlueprint:
    return ActorBlueprint(bp.id, dict(bp._attrs))


# ── actors ─────────────────────────────────────────────────────────────────────────────────────────

@dataclass
class _State:
    """Kinematic state of one actor at one tick (also what a snapshot stores)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0          # deg/s
    ax: float = 0.0
    ay: float = 0.0


class Actor:
    def __init__(self, world: "World", actor_id: int, type_id: str):
        self._world = world
        self.id = actor_id
        self.type_id = type_id
        self.is_alive = True
        self.attributes: Dict[str, str] = {}
        self.parent: Optional["Actor"] = None
        self.st = _State()
        self.bounding_box = BoundingBox(Location(0, 0, 0), Vector3D(0.1, 0.1, 0.1))

    def get_world(self) -> "World":
        return self._world

    def get_transform(self) -> Transform:
        return Transform(Location(self.st.x, self.st.y, self.st.z), Rotation(0.0, self.st.yaw, 0.0))

    def get_location(self) -> Location:
        return Location(self.st.x, self.st.y, self.st.z)

    def get_velocity(self) -> Vector3D:
        return Vector3D(self.st.vx, self.st.vy, 0.0)

    def get_angular_velocity(self) -> Vector3D:
        return Vector3D(0.0, 0.0, self.st.yaw_rate)

    def get_acceleration(self) -> Vector3D:
        return Vector3D(self.st.ax, self.st.ay, 0.0)

    def set_autopilot(self, enabled: bool = True, port: Optional[int] = None) -> None:
        pass

    def destroy(self) -> bool:
        return self._world._destroy(self)


class Vehicle(Actor):
    def __init__(self, world: "World", actor_id: int, bp: ActorBlueprint, ring: "city.Ring", s: float):
        super().__init__(world, actor_id, bp.id)
        self.two_wheels = bp.get_attribute("number_of_wheels").as_int() == 2
        self.attributes = {"role_name": "autopilot", "number_of_wheels": str(bp.get_attribute("number_of_wheels").value)}
        self.bounding_box = BoundingBox(Location(0, 0, CAR_EXTENT.z), Vector3D(1.1, 0.4, 0.9) if self.two_wheels else CAR_EXTENT)
        self.ring, self.s = ring, s
        self.autopilot = False
        self.steer = 0.0
        self._place()

    def _place(self) -> None:
        x, y, yaw = self.ring.pose(self.s)
        self.st.x, self.st.y, self.st.z, self.st.yaw = x, y, 0.0, yaw

    def set_autopilot(self, enabled: bool = True, port: Optional[int] = None) -> None:
        self.autopilot = bool(enabled)

    def get_control(self):
        moving = self.autopilot
        return types.SimpleNamespace(throttle=0.35 if moving else 0.0, steer=self.steer, brake=0.0 if moving else 1.0,
                                     reverse=False, hand_brake=False, manual_gear_shift=False, gear=1 if moving else 0)

    def get_physics_control(self):
        return types.SimpleNamespace(wheels=[types.SimpleNamespace(max_steer_angle=70.0)] * 4)

    def advance(self, dt: float, speed_scale: float) -> None:
        old_vx, old_vy, old_yaw = self.st.vx, self.st.vy, self.st.yaw
        if self.autopilot:
            pct = float(self._world.tm_settings.get(self.id, {}).get("vehicle_percentage_speed_difference", 0.0))
            speed = max(self.ring.speed_mps * speed_scale * (1.0 - pct / 100.0), 0.0)
            self.s = (self.s + speed * dt) % self.ring.length
            self._place()
            d = math.radians(self.st.yaw)
            self.st.vx, self.st.vy = speed * math.cos(d), speed * math.sin(d)
            self.st.yaw_rate = wrap_deg(self.st.yaw - old_yaw) / dt
            self.steer = max(-1.0, min(1.0, math.radians(self.st.yaw_rate) * 2.8 / max(speed, 1.0) / math.radians(70)))
        else:
            self.st.vx = self.st.vy = self.st.yaw_rate = 0.0
            self.steer = 0.0
        self.st.ax, self.st.ay = (self.st.vx - old_vx) / dt, (self.st.vy - old_vy) / dt


class Walker(Actor):
    def __init__(self, world: "World", actor_id: int, bp: ActorBlueprint, loc: Location):
        super().__init__(world, actor_id, bp.id)
        self.attributes = {"role_name": "walker"}
        self.bounding_box = BoundingBox(Location(0, 0, WALKER_EXTENT.z), WALKER_EXTENT)
        self.st.x, self.st.y, self.st.z = loc.x, loc.y, 0.9
        self.controller: Optional["WalkerController"] = None

    def advance(self, dt: float) -> None:
        c = self.controller
        if c is None or not c.started or c.target is None:
            self.st.vx = self.st.vy = 0.0
            return
        dx, dy = c.target.x - self.st.x, c.target.y - self.st.y
        dist = math.hypot(dx, dy)
        step = c.max_speed * dt
        if dist <= step:
            self.st.x, self.st.y = c.target.x, c.target.y
            c.target = self._world.random_navigation_location(self.st.x, self.st.y)
            self.st.vx = self.st.vy = 0.0
            return
        self.st.vx, self.st.vy = dx / dist * c.max_speed, dy / dist * c.max_speed
        self.st.x += self.st.vx * dt
        self.st.y += self.st.vy * dt
        self.st.yaw = math.degrees(math.atan2(dy, dx))


class WalkerController(Actor):
    def __init__(self, world: "World", actor_id: int, walker: Walker):
        super().__init__(world, actor_id, "controller.ai.walker")
        self.parent = walker
        self.started = False
        self.target: Optional[Location] = None
        self.max_speed = 1.4
        walker.controller = self

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def go_to_location(self, loc: Location) -> None:
        self.target = loc

    def set_max_speed(self, v: float) -> None:
        self.max_speed = float(v)

    def get_transform(self) -> Transform:
        return self.parent.get_transform() if self.parent else Transform()


class TrafficLight(Actor):
    def __init__(self, world: "World", actor_id: int, x: float, y: float, axis: int, group: List["TrafficLight"]):
        super().__init__(world, actor_id, "traffic.traffic_light")
        self.st.x, self.st.y, self.st.z = x, y, 3.5
        self.axis = axis
        self.group = group

    def _now(self) -> Tuple[str, float]:
        return city.light_state(self._world.sim_time, self.axis)

    def get_state(self) -> TrafficLightState:
        return TrafficLightState[self._now()[0]]

    def get_elapsed_time(self) -> float:
        return self._now()[1]

    def get_green_time(self) -> float:
        return city.GREEN_S

    def get_yellow_time(self) -> float:
        return city.YELLOW_S

    def get_red_time(self) -> float:
        return city.GREEN_S + city.YELLOW_S

    def get_group_traffic_lights(self) -> List["TrafficLight"]:
        return list(self.group)


class Sensor(Actor):
    """A LiDAR attached to ``parent``; delivers one measurement per tick to its listener."""
    def __init__(self, world: "World", actor_id: int, bp: ActorBlueprint, mount: Transform, parent: Actor):
        super().__init__(world, actor_id, bp.id)
        self.parent, self.mount = parent, mount
        self.attributes = {k: str(bp.get_attribute(k).value) for k in bp._attrs}
        self.callback: Optional[Callable[[Any], None]] = None

    def listen(self, callback: Callable[[Any], None]) -> None:
        self.callback = callback

    def stop(self) -> None:
        self.callback = None

    def is_listening(self) -> bool:
        return self.callback is not None

    def get_transform(self) -> Transform:
        assert self.parent is not None
        loc, _rot = compose((self.parent.st.x, self.parent.st.y, self.parent.st.z), (0.0, self.parent.st.yaw, 0.0),
                            (self.mount.location.x, self.mount.location.y, self.mount.location.z),
                            (self.mount.rotation.pitch, self.mount.rotation.yaw, self.mount.rotation.roll))
        r = self.mount.rotation
        return Transform(Location(*loc), Rotation(r.pitch, wrap_deg(self.parent.st.yaw + r.yaw), r.roll))

    def measure(self, frame: int, dt: float, boxes: List[lidar.Box]) -> Any:
        a = self.attributes
        t = self.get_transform()
        semantic = self.type_id.endswith("semantic")
        n_rays = max(int(float(a["points_per_second"]) * dt), 1)
        dirs = lidar.ray_directions(int(a["channels"]), float(a["lower_fov"]), float(a["upper_fov"]), n_rays,
                                    float(a["horizontal_fov"]), frame)
        pts = lidar.cast((t.location.x, t.location.y, t.location.z), (t.rotation.pitch, t.rotation.yaw, t.rotation.roll),
                         dirs, float(a["range"]), boxes, city.GRID, city.ROAD_HALF_WIDTH, city.SIDEWALK_OFFSET, semantic)
        return types.SimpleNamespace(frame=frame, timestamp=frame * dt, raw_data=pts.tobytes(), transform=t,
                                     horizontal_angle=0.0, channels=int(a["channels"]))


# ── world ──────────────────────────────────────────────────────────────────────────────────────────

class _ActorSnapshot:
    def __init__(self, st: _State):
        self._st = _State(**vars(st))

    def get_transform(self) -> Transform:
        return Transform(Location(self._st.x, self._st.y, self._st.z), Rotation(0.0, self._st.yaw, 0.0))

    def get_velocity(self) -> Vector3D:
        return Vector3D(self._st.vx, self._st.vy, 0.0)

    def get_angular_velocity(self) -> Vector3D:
        return Vector3D(0.0, 0.0, self._st.yaw_rate)

    def get_acceleration(self) -> Vector3D:
        return Vector3D(self._st.ax, self._st.ay, 0.0)


class WorldSnapshot:
    def __init__(self, frame: int, elapsed: float, delta: float, actors: Dict[int, _ActorSnapshot]):
        self.frame = frame
        self.timestamp = types.SimpleNamespace(frame=frame, elapsed_seconds=elapsed, delta_seconds=delta,
                                               platform_timestamp=elapsed)
        self._actors = actors

    def find(self, actor_id: int) -> Optional[_ActorSnapshot]:
        return self._actors.get(actor_id)

    def __contains__(self, actor_id: int) -> bool:
        return actor_id in self._actors


class ActorList(list):
    def filter(self, pattern: str) -> "ActorList":
        return ActorList(a for a in self if fnmatch.fnmatch(a.type_id, pattern))

    def find(self, actor_id: int):
        return next((a for a in self if a.id == actor_id), None)


class WorldSettings:
    def __init__(self):
        self.synchronous_mode = False
        self.fixed_delta_seconds: Optional[float] = None
        self.substepping = True
        self.max_substep_delta_time = 0.01
        self.max_substeps = 10
        self.no_rendering_mode = False


class Waypoint:
    def __init__(self, edge, i_xy, transform_yaw, is_junction, junction, along):
        self.edge, self.i_xy, self.is_junction = edge, i_xy, is_junction
        self._junction = junction
        self._yaw = transform_yaw
        self.transform = Transform(Location(i_xy[0], i_xy[1], 0.0), Rotation(0.0, transform_yaw, 0.0))
        self._along = along

    def get_junction(self):
        return self._junction

    def next(self, distance: float) -> List["Waypoint"]:
        (x0, y0), (x1, y1) = self.edge[0], self.edge[1]
        length = math.hypot(x1 - x0, y1 - y0)
        along = self._along + distance
        if along > length + 1e-6:
            return []
        f = along / length
        return [Waypoint(self.edge, (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f), self._yaw, self.is_junction,
                         self._junction, along)]


class _Junction:
    def __init__(self, jid: int, cx: float, cy: float):
        self.id = jid
        self.bounding_box = BoundingBox(Location(cx, cy, 0.0), Vector3D(city.JUNCTION_HALF, city.JUNCTION_HALF, 5.0))


class Map:
    name = "Carla/Maps/FakeTown"

    def __init__(self, spawn_points: List[Transform]):
        self._spawn_points = spawn_points
        self._junctions = {jid: _Junction(jid, gx, gy) for jid, gx, gy in city.intersections()}

    def get_spawn_points(self) -> List[Transform]:
        return list(self._spawn_points)

    def get_topology(self):
        out = []
        for (x0, y0), (x1, y1), _ in [(a, b, c) for a, b, c in city.lane_edges()]:
            yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
            edge = ((x0, y0), (x1, y1))
            out.append((Waypoint(edge, (x0, y0), yaw, False, None, 0.0),
                        Waypoint(edge, (x1, y1), yaw, False, None, math.hypot(x1 - x0, y1 - y0))))
        for (x0, y0), (x1, y1), jid in city.junction_connectors():
            yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
            edge = ((x0, y0), (x1, y1))
            j = self._junctions[jid]
            out.append((Waypoint(edge, (x0, y0), yaw, True, j, 0.0),
                        Waypoint(edge, (x1, y1), yaw, True, j, math.hypot(x1 - x0, y1 - y0))))
        return out


class World:
    def __init__(self, seed: int = 0):
        self.settings = WorldSettings()
        self.frame = FIRST_FRAME
        self.sim_time = 0.0
        self._rng = random.Random(seed)
        self.rings = city.build_rings()
        self.buildings = city.build_buildings()
        self._slots = city.spawn_slots(self.rings)
        self._spawn_points = []
        for ring_id, s in self._slots:
            x, y, yaw = self.rings[ring_id].pose(s)
            self._spawn_points.append(Transform(Location(x, y, 0.3), Rotation(0.0, yaw, 0.0)))
        self._map = Map(self._spawn_points)
        self._bp = BlueprintLibrary()
        self._actors: Dict[int, Actor] = {}
        self._next_id = 100
        self._speed_scale = 1.0
        self.tm_settings: Dict[int, Dict[str, Any]] = {}   # actor id -> TM per-vehicle settings
        self._snapshot = WorldSnapshot(self.frame, 0.0, 0.05, {})
        self._build_traffic_lights()
        self._snapshot = self._make_snapshot()

    # ── setup ────────────────────────────────────────────────────────────────────────────────────
    def _build_traffic_lights(self) -> None:
        for jid, gx, gy in city.intersections():
            group: List[TrafficLight] = []
            for k, (dx, dy, axis) in enumerate([(-12, -12, 0), (12, -12, 1), (12, 12, 0), (-12, 12, 1)]):
                self._next_id += 1
                tl = TrafficLight(self, self._next_id, gx + dx, gy + dy, axis, group)
                group.append(tl)
                self._actors[tl.id] = tl

    # ── API ──────────────────────────────────────────────────────────────────────────────────────
    def get_settings(self) -> WorldSettings:
        s = WorldSettings()
        s.__dict__.update(self.settings.__dict__)
        return s

    def apply_settings(self, s: WorldSettings) -> int:
        self.settings = s
        return self.frame

    @property
    def dt(self) -> float:
        return self.settings.fixed_delta_seconds or 0.05

    def get_map(self) -> Map:
        return self._map

    def get_blueprint_library(self) -> BlueprintLibrary:
        return self._bp

    def get_actors(self) -> ActorList:
        return ActorList(a for a in self._actors.values() if a.is_alive)

    def get_actor(self, actor_id: int) -> Optional[Actor]:
        a = self._actors.get(actor_id)
        return a if a is not None and a.is_alive else None

    def get_snapshot(self) -> WorldSnapshot:
        return self._snapshot

    def random_navigation_location(self, near_x: Optional[float] = None, near_y: Optional[float] = None) -> Location:
        """A point on a sidewalk; with ``near_*`` on the same sidewalk line (so walkers stay on their line)."""
        r = self._rng
        if near_x is not None and near_y is not None:
            # same line: which of the lines is nearest?
            best = None
            for g in city.GRID:
                for side in (-city.SIDEWALK_OFFSET, city.SIDEWALK_OFFSET):
                    for axis in (0, 1):
                        d = abs((near_y if axis == 0 else near_x) - (g + side))
                        if best is None or d < best[0]:
                            best = (d, g, side, axis)
            assert best is not None
            _, g, side, axis = best
        else:
            g, side, axis = r.choice(city.GRID), r.choice((-city.SIDEWALK_OFFSET, city.SIDEWALK_OFFSET)), r.choice((0, 1))
        along = r.uniform(-20.0, 220.0)
        return Location(along, g + side, 0.5) if axis == 0 else Location(g + side, along, 0.5)

    def get_random_location_from_navigation(self) -> Location:
        return self.random_navigation_location()

    def set_pedestrians_seed(self, seed: int) -> None:
        """The walkers' navigation targets are the only random stream of the fake world; the server seeds it with
        its master seed at start (as it seeds CARLA's walker navigation)."""
        self._rng = random.Random(seed)

    # ── spawning ─────────────────────────────────────────────────────────────────────────────────
    def _new_id(self) -> int:
        self._next_id += 1
        return self._next_id

    def _nearest_ring_point(self, x: float, y: float) -> Optional[Tuple["city.Ring", float, float]]:
        best = None
        for ring in self.rings:
            for k in range(0, int(ring.length), 2):
                px, py, _ = ring.pose(float(k))
                d = math.hypot(px - x, py - y)
                if best is None or d < best[0]:
                    best = (d, ring, float(k))
        if best is None or best[0] > 4.0:
            return None
        return best[1], best[2], best[0]

    def try_spawn_actor(self, bp: ActorBlueprint, transform: Transform, attach_to: Optional[Actor] = None) -> Optional[Actor]:
        t = transform
        if bp.id.startswith("vehicle."):
            hit = self._nearest_ring_point(t.location.x, t.location.y)
            if hit is None:
                return None
            ring, s, _ = hit
            for a in self._actors.values():
                if isinstance(a, Vehicle) and a.is_alive and math.hypot(a.st.x - t.location.x, a.st.y - t.location.y) < 5.0:
                    return None
            v = Vehicle(self, self._new_id(), bp, ring, s)
            self._actors[v.id] = v
            return v
        if bp.id.startswith("walker."):
            w = Walker(self, self._new_id(), bp, t.location)
            self._actors[w.id] = w
            return w
        if bp.id == "controller.ai.walker":
            if not isinstance(attach_to, Walker):
                return None
            c = WalkerController(self, self._new_id(), attach_to)
            self._actors[c.id] = c
            return c
        if bp.id.startswith("sensor.lidar."):
            if attach_to is None:
                return None
            s = Sensor(self, self._new_id(), bp, t, attach_to)
            self._actors[s.id] = s
            return s
        return None

    def spawn_actor(self, bp: ActorBlueprint, transform: Transform, attach_to: Optional[Actor] = None) -> Actor:
        a = self.try_spawn_actor(bp, transform, attach_to)
        if a is None:
            raise RuntimeError(f"Spawn failed: collision or invalid location for {bp.id}")
        return a

    def _destroy(self, actor: Actor) -> bool:
        if not actor.is_alive:
            return False
        actor.is_alive = False
        self._actors.pop(actor.id, None)
        self.tm_settings.pop(actor.id, None)
        for a in list(self._actors.values()):          # attached actors go with their parent, like in CARLA
            if a.parent is actor:
                a.is_alive = False
                self._actors.pop(a.id, None)
        return True

    # ── time ─────────────────────────────────────────────────────────────────────────────────────
    def _make_snapshot(self) -> WorldSnapshot:
        return WorldSnapshot(self.frame, self.sim_time, self.dt,
                             {a.id: _ActorSnapshot(a.st) for a in self._actors.values()
                              if a.is_alive and not isinstance(a, (Sensor, WalkerController))})

    def tick(self, timeout: float = 0.0) -> int:
        dt = self.dt
        self.frame += 1
        self.sim_time += dt
        for a in list(self._actors.values()):
            if isinstance(a, Vehicle):
                a.advance(dt, self._speed_scale)
            elif isinstance(a, Walker):
                a.advance(dt)
        self._snapshot = self._make_snapshot()
        sensors = [a for a in self._actors.values() if isinstance(a, Sensor) and a.is_listening()]
        if sensors:
            boxes = self._boxes()
            for s in sensors:
                cb = s.callback
                if cb is not None:
                    cb(s.measure(self.frame, dt, boxes))
        return self.frame

    def _boxes(self) -> List[lidar.Box]:
        out: List[lidar.Box] = []
        for a in self._actors.values():
            if isinstance(a, Vehicle):
                e = a.bounding_box.extent
                out.append(lidar.Box(a.st.x, a.st.y, a.st.z + e.z, e.x, e.y, e.z, a.st.yaw, a.id, lidar.TAG_CAR))
            elif isinstance(a, Walker):
                out.append(lidar.Box(a.st.x, a.st.y, WALKER_EXTENT.z, 0.25, 0.25, WALKER_EXTENT.z, a.st.yaw, a.id,
                                     lidar.TAG_PEDESTRIAN))
        for b in self.buildings:
            out.append(lidar.Box(b.cx, b.cy, b.height / 2, b.half_x, b.half_y, b.height / 2, 0.0, 0, lidar.TAG_BUILDING))
        return out

    def set_global_speed_pct(self, pct: float) -> None:
        self._speed_scale = 1.0 - pct / 100.0


# ── client, traffic manager, commands ──────────────────────────────────────────────────────────────

#: per-vehicle TM setters the project calls (``src/traffic/drivers.py``); the names are the ones of the live 0.9.16 TM
#: (AGENTS.md: ``keep_slow_lane_rule_percentage``). Only the speed offset changes the fake's motion; the rest is stored.
TM_VEHICLE_SETTINGS = (
    "vehicle_percentage_speed_difference", "distance_to_leading_vehicle", "random_left_lanechange_percentage",
    "random_right_lanechange_percentage", "keep_slow_lane_rule_percentage", "ignore_lights_percentage",
    "ignore_signs_percentage", "ignore_vehicles_percentage", "ignore_walkers_percentage", "auto_lane_change",
)


class TrafficManager:
    def __init__(self, world: World, port: int):
        self._world, self._port = world, port

    def get_port(self) -> int:
        return self._port

    def set_synchronous_mode(self, on: bool) -> None:
        pass

    def set_random_device_seed(self, seed: int) -> None:
        pass

    def set_global_distance_to_leading_vehicle(self, d: float) -> None:
        pass

    def global_percentage_speed_difference(self, pct: float) -> None:
        self._world.set_global_speed_pct(pct)

    def __getattr__(self, name: str) -> Callable[..., None]:
        if name not in TM_VEHICLE_SETTINGS:
            raise AttributeError(f"fake TrafficManager has no method {name!r}")

        def setter(actor: Actor, value: Any) -> None:
            self._world.tm_settings.setdefault(actor.id, {})[name] = value
        return setter


class _Command:
    def __init__(self, kind: str, *args: Any):
        self.kind, self.args = kind, args
        self.chained: List["_Command"] = []

    def then(self, other: "_Command") -> "_Command":
        self.chained.append(other)
        return self


class _FutureActor:
    pass


class Client:
    def __init__(self, host: str = "fake", port: int = 2000, seed: int = 0, world: Optional[World] = None):
        self.host, self.port = host, port
        self._world = world or World(seed)

    def set_timeout(self, t: float) -> None:
        pass

    def get_server_version(self) -> str:
        return "0.9.16-fake"

    def get_client_version(self) -> str:
        return "0.9.16-fake"

    def get_world(self) -> World:
        return self._world

    def get_available_maps(self) -> List[str]:
        return ["/Game/Carla/Maps/FakeTown"]

    def load_world(self, name: str) -> World:
        if name.rstrip("/").split("/")[-1] != "FakeTown":
            raise RuntimeError("the fake world has a single map (FakeTown)")
        return self._world

    def get_trafficmanager(self, port: int = 8000) -> TrafficManager:
        return TrafficManager(self._world, port)

    def apply_batch_sync(self, commands: List[_Command], do_tick: bool = False) -> List[Any]:
        results = []
        for cmd in commands:
            err, actor_id = None, 0
            if cmd.kind == "spawn":
                bp, transform, parent = cmd.args
                actor = self._world.try_spawn_actor(bp, transform, parent)
                if actor is None:
                    err = "spawn failed"
                else:
                    actor_id = actor.id
                    for ch in cmd.chained:
                        if ch.kind == "autopilot":
                            actor.set_autopilot(ch.args[1], ch.args[2] if len(ch.args) > 2 else None)
            elif cmd.kind == "destroy":
                actor = cmd.args[0]
                actor_id = actor.id
                if not self._world._destroy(actor):
                    err = "actor not found"
            elif cmd.kind == "autopilot":
                cmd.args[0].set_autopilot(cmd.args[1])
            results.append(types.SimpleNamespace(error=err, actor_id=actor_id))
        if do_tick:
            self._world.tick()
        return results


def build_module(seed: int = 0) -> types.ModuleType:
    mod = types.ModuleType("carla")
    mod.__version__ = "0.9.16-fake"  # type: ignore[attr-defined]
    mod.IS_FAKE = True  # type: ignore[attr-defined]
    mod.Client = lambda host="fake", port=2000: Client(host, port, seed)  # type: ignore[attr-defined]
    mod.Transform, mod.Location, mod.Rotation, mod.Vector3D, mod.BoundingBox = Transform, Location, Rotation, Vector3D, BoundingBox  # type: ignore[attr-defined]
    mod.TrafficLightState = TrafficLightState  # type: ignore[attr-defined]
    mod.World, mod.Actor, mod.Sensor, mod.TrafficManager = World, Actor, Sensor, TrafficManager  # type: ignore[attr-defined]
    mod.WorldSettings, mod.BlueprintLibrary, mod.ActorBlueprint = WorldSettings, BlueprintLibrary, ActorBlueprint  # type: ignore[attr-defined]
    mod.command = types.SimpleNamespace(  # type: ignore[attr-defined]
        SpawnActor=lambda bp, tr, parent=None: _Command("spawn", bp, tr, parent),
        DestroyActor=lambda a: _Command("destroy", a),
        SetAutopilot=lambda a, on=True, port=None: _Command("autopilot", a, on, port),
        FutureActor=_FutureActor(),
    )
    return mod
