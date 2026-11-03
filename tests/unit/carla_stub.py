"""A minimal in-memory stand-in for the ``carla`` module that records WHO calls it.

Every call that would be an RPC to the CARLA server is appended to ``Recorder.calls`` as
(name, thread_name, extra). Tests then assert that after start-up only the simulation
thread talks to CARLA, and that nothing ticks the world behind the tick loop's back.
"""

import threading
import types


class Recorder:
    def __init__(self):
        self.calls = []
        self.frame = 1000
        self.next_id = 500
        self.actors = {}
        self.lock = threading.Lock()
        self.foreign_ticks = 0

    def rec(self, name, extra=None):
        with self.lock:
            self.calls.append((name, threading.current_thread().name, extra))

    def tick(self):
        with self.lock:
            self.frame += 1
            return self.frame


class Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Location(Vec):
    pass


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()


class _Attr:
    def __init__(self, value):
        self.recommended_values = [value]
        self._v = value

    def __int__(self):
        return int(self._v)


class Blueprint:
    def __init__(self, bid, wheels="4"):
        self.id = bid
        self._wheels = wheels
        self.attrs = {}

    def has_attribute(self, name):
        return name in ("color", "is_invincible", "number_of_wheels")

    def set_attribute(self, name, value):
        self.attrs[name] = value

    def get_attribute(self, name):
        return _Attr(self._wheels if name == "number_of_wheels" else "255,0,0")


class BlueprintLibrary:
    def __init__(self, rec):
        self.rec = rec
        self.vehicles = [Blueprint(f"vehicle.test.car{i}") for i in range(4)]
        self.walkers = [Blueprint(f"walker.pedestrian.{i:04d}") for i in range(3)]

    def filter(self, pattern):
        self.rec.rec("bp_filter", pattern)
        if pattern.startswith("vehicle"):
            return list(self.vehicles)
        if pattern.startswith("walker"):
            return list(self.walkers)
        return []

    def find(self, bid):
        self.rec.rec("bp_find", bid)
        for bp in self.vehicles + self.walkers:
            if bp.id == bid:
                return bp
        if bid == "controller.ai.walker":
            return Blueprint(bid)
        return None


class Actor:
    def __init__(self, rec, actor_id, bp=None, transform=None):
        self.rec, self.id, self.bp = rec, actor_id, bp
        self.transform = transform or Transform()
        self.destroyed = False
        self.attributes = {}
        self.type_id = getattr(bp, "id", "test.actor")

    def get_transform(self):
        self.rec.rec("actor.get_transform", self.id)
        return self.transform

    def get_location(self):
        self.rec.rec("actor.get_location", self.id)
        return self.transform.location

    def get_velocity(self):
        self.rec.rec("actor.get_velocity", self.id)
        return Vec(3.0, 4.0, 0.0)

    bounding_box = types.SimpleNamespace(extent=types.SimpleNamespace(x=2.3, y=0.9, z=0.75))

    def get_control(self):
        return types.SimpleNamespace(steer=0.0, brake=0.0, throttle=0.0, reverse=False)

    def get_physics_control(self):
        return types.SimpleNamespace(wheels=[types.SimpleNamespace(max_steer_angle=70.0)])

    def get_angular_velocity(self):
        return Vec()

    def set_autopilot(self, on, port=None):
        self.rec.rec("actor.set_autopilot", self.id)

    def destroy(self):
        self.rec.rec("actor.destroy", self.id)
        self.destroyed = True
        self.rec.actors.pop(self.id, None)
        return True

    # walker controller API
    def start(self):
        self.rec.rec("controller.start", self.id)

    def stop(self):
        self.rec.rec("controller.stop", self.id)

    def go_to_location(self, loc):
        self.rec.rec("controller.go_to_location", self.id)

    def set_max_speed(self, v):
        self.rec.rec("controller.set_max_speed", self.id)


class _ActorSnap:
    def __init__(self, actor):
        self._a = actor

    def get_transform(self):
        return self._a.transform

    def get_velocity(self):
        return Vec(3.0, 4.0, 0.0)

    def get_angular_velocity(self):
        return Vec()

    def get_acceleration(self):
        return Vec()


class Snapshot:
    def __init__(self, rec):
        self.frame = rec.frame
        self.timestamp = types.SimpleNamespace(frame=rec.frame, elapsed_seconds=rec.frame * 0.05)
        self._actors = dict(rec.actors)

    def find(self, actor_id):
        a = self._actors.get(actor_id)
        return _ActorSnap(a) if a else None


class _Map:
    def __init__(self, rec):
        self.name = "Carla/Maps/TestTown"
        self.rec = rec

    def get_spawn_points(self):
        self.rec.rec("map.get_spawn_points")
        return [Transform(Location(float(i * 20), 0.0, 0.0)) for i in range(40)]


class _Actors(list):
    def filter(self, pattern):
        return []


class World:
    def __init__(self, rec):
        self.rec = rec
        self.settings = types.SimpleNamespace(
            synchronous_mode=False, fixed_delta_seconds=None, substepping=False,
            max_substep_delta_time=0.01, max_substeps=10)
        self.bp_lib = BlueprintLibrary(rec)

    def tick(self):
        self.rec.rec("world.tick")
        return self.rec.tick()

    def get_snapshot(self):
        self.rec.rec("world.get_snapshot")
        return Snapshot(self.rec)

    def get_settings(self):
        self.rec.rec("world.get_settings")
        return types.SimpleNamespace(**vars(self.settings))

    def apply_settings(self, s):
        self.rec.rec("world.apply_settings")
        self.settings = s

    def get_map(self):
        self.rec.rec("world.get_map")
        return _Map(self.rec)

    def get_blueprint_library(self):
        return self.bp_lib

    def get_actors(self):
        self.rec.rec("world.get_actors")
        return _Actors()

    def get_actor(self, actor_id):
        self.rec.rec("world.get_actor", actor_id)
        return self.rec.actors.get(actor_id)

    def try_spawn_actor(self, bp, transform, attach_to=None):
        self.rec.rec("world.try_spawn_actor")
        self.rec.next_id += 1
        a = Actor(self.rec, self.rec.next_id, bp, transform)
        self.rec.actors[a.id] = a
        return a

    def get_random_location_from_navigation(self):
        self.rec.rec("world.get_random_location")
        return Location(1.0, 2.0, 0.0)


class _TM:
    def __init__(self, rec):
        self.rec = rec

    def get_port(self):
        self.rec.rec("tm.get_port")
        return 8001

    def __getattr__(self, name):
        def call(*a, **k):
            self.rec.rec(f"tm.{name}")
        return call


class _Cmd:
    def __init__(self, kind, *args):
        self.kind, self.args, self.then_cmd = kind, args, None

    def then(self, other):
        self.then_cmd = other
        return self


class Client:
    def __init__(self, rec, host, port):
        self.rec, self.host, self.port = rec, host, port
        self.world = World(rec)

    def set_timeout(self, t):
        self.rec.rec("client.set_timeout")

    def get_world(self):
        self.rec.rec("client.get_world")
        return self.world

    def get_trafficmanager(self, port):
        self.rec.rec("client.get_trafficmanager")
        return _TM(self.rec)

    def apply_batch_sync(self, batch, do_tick=False):
        self.rec.rec("client.apply_batch_sync", {"n": len(batch), "do_tick": do_tick})
        results = []
        for cmd in batch:
            if cmd.kind == "spawn":
                self.rec.next_id += 1
                a = Actor(self.rec, self.rec.next_id, cmd.args[0], cmd.args[1])
                self.rec.actors[a.id] = a
                results.append(types.SimpleNamespace(error=None, actor_id=a.id))
            elif cmd.kind == "destroy":
                actor = cmd.args[0]
                self.rec.actors.pop(actor.id, None)
                results.append(types.SimpleNamespace(error=None, actor_id=actor.id))
            else:
                results.append(types.SimpleNamespace(error=None, actor_id=0))
        if do_tick:
            self.rec.tick()
            self.rec.rec("batch.implicit_tick")
        return results


def make_carla_module(rec: Recorder):
    mod = types.ModuleType("carla")
    mod.Client = lambda host, port: Client(rec, host, port)
    mod.Transform, mod.Location, mod.Rotation = Transform, Location, Rotation
    mod.command = types.SimpleNamespace(
        SpawnActor=lambda bp, tr, parent=None: _Cmd("spawn", bp, tr, parent),
        DestroyActor=lambda a: _Cmd("destroy", a),
        SetAutopilot=lambda a, on, port=None: _Cmd("autopilot", a),
        FutureActor=object(),
    )
    for name in ("World", "Actor", "Sensor", "TrafficManager", "WorldSettings",
                 "BlueprintLibrary", "ActorBlueprint"):
        setattr(mod, name, type(name, (), {}))
    return mod
