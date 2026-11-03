"""A CARLA-free stand-in for PersistentCARLAServer, exposing what the routes use.

It uses the real SessionManager, so session behaviour (create/end/expire) is the
production code; only spawning and telemetry are faked.
"""

import tempfile
from concurrent.futures import Future
from pathlib import Path

import numpy as np

from src.lidar import protocol
from src.lidar.config import LidarConfig, LidarLimits
from src.lidar.manager import LidarError, LidarManager, Measurement, StreamProfile
from src.recording.recorder import RunAlreadyRecording, RunRecorder, list_runs
from src.server.session_manager import SessionManager

_PT = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"), ("object_idx", "u4"), ("tag", "u4")])


class FakeLidarHandle:
    def __init__(self, config):
        self.config = config

    def get_measurement(self, frame, timeout_s):
        n = self.config.points_per_frame(0.05)
        pts = np.zeros(n, dtype=_PT)
        pts["x"] = np.arange(n)
        return Measurement(frame, pts, protocol.SEMANTIC_FIELDS, (0.0, 0.0, 2.4), (0.0, 0.0, 0.0))

    def destroy(self):
        pass


class FakeLidarBackend:
    def spawn(self, actor_id, config):
        return FakeLidarHandle(config)


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
        self.fixed_delta_seconds = 0.05
        self.recorder = None
        self._runs_dir = Path(tempfile.mkdtemp(prefix="fake-runs-"))
        self.lidar = LidarManager(FakeLidarBackend(), LidarLimits(max_sensors=2, max_total_points_per_second=1_000_000),
                                  0.05, seed=1)
        self._frame = 1000

    def tick_lidar(self, n=1):
        for _ in range(n):
            self._frame += 1
            self.lidar.on_tick(self._frame, self._frame * 0.05)

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
        self.lidar.destroy_owner_now(session_id)
        self.despawned.append(session_id)
        self.spawned.pop(session_id, None)

    def get_vehicle_telemetry(self, session_id):
        if session_id not in self.spawned:
            return None
        return {"actor_id": self.spawned[session_id], "x": 1.0, "y": 2.0, "z": 0.0,
                "yaw": 90.0, "speed_kmh": 12.5, "frame": 42}

    def get_vehicle_v2x_data(self, session_id):
        return {"neighbors": [], "threats": []}

    def enable_client_lidar(self, session_id, config=None):
        actor_id = self.spawned.get(session_id)
        if actor_id is None:
            raise LidarError("session has no vehicle")
        existing = self.lidar.sensor_of_owner(session_id)
        if existing is not None:
            if config is not None:
                self.lidar.reconfigure(existing["sensor_id"], config, requester=session_id)
            return self.lidar.get_info(existing["sensor_id"])
        return self.lidar.get_info(self.lidar.request(session_id, actor_id, config, own_actor_id=actor_id).sensor_id)

    def disable_client_lidar(self, session_id):
        existing = self.lidar.sensor_of_owner(session_id)
        if existing is None:
            return False
        self.lidar.release(existing["sensor_id"], requester=session_id)
        return True

    def get_client_lidar_info(self, session_id):
        return self.lidar.sensor_of_owner(session_id)

    def get_client_lidar_frame(self, session_id, profile=StreamProfile(), after_frame=-1):
        info = self.get_client_lidar_info(session_id)
        return None if info is None else self.lidar.get_stream_frame(info["sensor_id"], profile, after_frame)

    # --- run recording ---------------------------------------------------
    def start_run(self, name, record_lidar=False, record_links=True):
        if self.recorder is not None:
            raise RunAlreadyRecording(f"a run is already being recorded: {self.recorder.run_id}")
        self.recorder = RunRecorder(self._runs_dir, name, {"config": {"seed": 42}}, start_frame=self._frame,
                                    start_sim_time=0.0, git_sha="fake", record_links=record_links, record_lidar=record_lidar)
        return self.recorder.summary()

    def stop_run(self):
        rec, self.recorder = self.recorder, None
        return None if rec is None else rec.stop(self._frame, 1.0)

    def get_run_status(self):
        return {"current": self.recorder.summary() if self.recorder else None, "runs": list_runs(self._runs_dir)}

    # --- hazards ----------------------------------------------------------
    def publish_hazard(self, x, y, z=0.0, cause_code=10, duration_s=30.0, relevance_distance_m=150.0, tx_power_dbm=None):
        from src.v2x.safety_messages import DENMCauseCode
        if not getattr(self, "v2x_on", True):
            from src.server.route_utils import ConflictError
            raise ConflictError("V2X is disabled")
        cause = DENMCauseCode(cause_code)               # ValueError for unknown codes, like the real server
        self._hazards = getattr(self, "_hazards", {})
        sid = 1_000_000 + len(self._hazards)
        self._hazards[sid] = {"station_id": sid, "x_m": x, "y_m": y, "z_m": z, "cause_code": int(cause), "cause": cause.name,
                              "relevance_distance_m": relevance_distance_m, "reference_time": 0.0, "expires_sim_time": duration_s}
        return self._hazards[sid]

    def clear_hazard(self, station_id):
        return getattr(self, "_hazards", {}).pop(station_id, None) is not None

    def get_hazards(self):
        return list(getattr(self, "_hazards", {}).values())

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
