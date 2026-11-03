"""A CARLA-free stand-in for PersistentCARLAServer, exposing what the routes use.

It uses the real SessionManager, so session behaviour (create/end/expire) is the
production code; only spawning and telemetry are faked.
"""

from concurrent.futures import Future

from src.server.session_manager import SessionManager


class FakeWorld:
    def __init__(self, running: bool = True, spawn_ok: bool = True):
        self.sessions = SessionManager()
        self._running = running
        self.spawn_ok = spawn_ok
        self.spawned = {}  # session_id -> actor_id
        self.despawned = []
        self.stopped = False
        self._next_actor = 100
        self.background = 0
        self.pedestrians = 0
        self.v2x_network = None

    def submit(self, fn, *args, **kwargs):
        """Mirror PersistentCARLAServer.submit: run now, hand back a Future."""
        fut = Future()
        if not self._running:
            fut.set_exception(RuntimeError("simulation is not running"))
            return fut
        try:
            fut.set_result(fn(*args, **kwargs))
        except BaseException as exc:  # noqa: BLE001
            fut.set_exception(exc)
        return fut

    # --- lifecycle -------------------------------------------------------
    def stop(self):
        self.stopped = True
        self._running = False

    def get_status(self):
        return {
            "running": self._running, "frame": 42, "elapsed_seconds": 1.0,
            "map": "Town01", "background_vehicles": self.background,
            "pedestrians": self.pedestrians,
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self.spawned), "v2x_enabled": False,
            "v2x_range": 0, "rsu_count": 0, "vru_count": 0,
        }

    # --- client vehicles -------------------------------------------------
    def spawn_client_vehicle(self, session_id, blueprint_id):
        if not self.spawn_ok:
            return None
        self._next_actor += 1
        self.spawned[session_id] = self._next_actor
        return self._next_actor

    def despawn_client_vehicle(self, session_id):
        self.despawned.append(session_id)
        self.spawned.pop(session_id, None)

    def get_vehicle_telemetry(self, session_id):
        if session_id not in self.spawned:
            return None
        return {"actor_id": self.spawned[session_id], "x": 1.0, "y": 2.0, "z": 0.0,
                "yaw": 90.0, "speed_kmh": 12.5, "frame": 42}

    def get_vehicle_v2x_data(self, session_id):
        return {"neighbors": [], "threats": []}

    def enable_client_lidar(self, session_id):
        return False

    # --- admin population ------------------------------------------------
    def spawn_background_traffic(self, count):
        self.background += count
        return count

    def despawn_background_traffic(self):
        n, self.background = self.background, 0
        return n

    def spawn_pedestrians(self, count):
        self.pedestrians += count
        return count

    def despawn_pedestrians(self):
        n, self.pedestrians = self.pedestrians, 0
        return n
