# TODO: CARLA handles / the module-global world are Optional until a connected-world type exists (WorldController state machine)
# pyright: reportOptionalMemberAccess=false
"""
Persistent CARLA world — the heart of the live-server architecture.

The admin starts this once; it runs indefinitely in a background thread.
Clients (laptop, raspi, workstation…) join and leave dynamically — each
gets one "leading vehicle" tracked here.

Usage
-----
    world = PersistentCARLAServer(WorldConfig(host=..., port=...))
    world.start()          # connect to CARLA, start the simulation thread
    ...
    world.stop()           # graceful shutdown + actor cleanup

Threading model (REFACTOR_HANDOFF section 4.1, defects C1-C5, C10)
------------------------------------------------------------------
Exactly one thread, the simulation thread ("carla-tick"), talks to CARLA once ``start()``
has returned. Everything else (HTTP handlers, WebSocket tasks) calls ``submit(fn, ...)``
and waits on the returned Future; the simulation thread runs it between two ticks. No
caller ticks the world: ``apply_batch_sync`` is always called with ``do_tick=False`` and
the one explicit tick (pedestrian controllers) goes through ``_tick()``, which keeps
``_frame`` equal to CARLA's own frame id.

Readers never call CARLA. After every tick the simulation thread publishes telemetry (and,
after every V2X update, the per-client V2X payload) as plain dicts stamped with the frame
they belong to; getters just return the last published dict.
"""

from __future__ import annotations

import collections
import logging
import math
import random
import threading
import time
from concurrent.futures import Future
from typing import Any, Callable, Dict, List, Optional

import carla

from src.lidar.carla_backend import CarlaLidarBackend
from src.lidar.config import LidarConfig
from src.lidar.manager import LidarError, LidarManager, StreamProfile
from src.traffic.advisory import V2XAdvisory
from src.traffic.drivers import DriverAssigner, DriverProfile, apply_profile
from src.recording.recorder import RunAlreadyRecording, RunRecorder, list_runs
from src.server.session_manager import SessionManager
from src.server.sim_executor import SimThreadExecutor
from src.server.world_config import WorldConfig

logger = logging.getLogger(__name__)

# Blueprints that are always available in CARLA 0.9.x Town maps.
_FALLBACK_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.lincoln.mkz_2020",
]

_SESSION_CHECK_EVERY_N_FRAMES = 100  # ~5 s at 20 FPS


def _git_sha() -> str:
    """Short git sha for run manifests: env GIT_SHA (set in Docker builds) or ``git rev-parse``, else ``nogit``."""
    import os
    import subprocess
    if os.environ.get("GIT_SHA"):
        return os.environ["GIT_SHA"]
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=3)
        return out.stdout.strip() or "nogit"
    except Exception:
        return "nogit"


class PersistentCARLAServer:
    """
    Manages a persistent CARLA world that runs indefinitely.

    Background traffic and pedestrians are controlled by the admin via the
    REST API.  Each client session spawns exactly one "leading vehicle".
    """

    def __init__(self, config: WorldConfig):
        self.config = config
        self.host = config.host
        self.port = config.port
        self.fixed_delta_seconds = config.fixed_delta_seconds
        self.v2x_enabled = config.v2x_enabled
        self.v2x_range = config.v2x_range_m

        # Seeded RNG for spawn points / blueprints (defect C8)
        self._rng = random.Random(config.seed)
        # separate stream so the penetration draw never shifts spawn-point/blueprint choices
        self._v2x_rng = random.Random(config.seed + 1)

        # background-traffic drivers: seeded profile draw (+2), optional V2X advisory, per-vehicle bookkeeping
        self._drivers = DriverAssigner(config.driver_mix, config.seed + 2)
        self._driver_of: Dict[int, DriverProfile] = {}
        self._advisory = V2XAdvisory(config.advisory)
        self._advised: Dict[int, float] = {}  # actor id -> base speed offset while an advisory is active

        # run recording (one at a time); created by start_run()
        self.recorder: Optional[RunRecorder] = None
        self._sim_time = 0.0  # simulation time of the last tick (stamps recorded events)

        # CARLA handles
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.tm: Optional[carla.TrafficManager] = None
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.spawn_points: List[carla.Transform] = []
        self._original_settings: Optional[carla.WorldSettings] = None
        self._map_name: Optional[str] = None

        # Managed actors (touched only by the simulation thread; other threads may len() them)
        self._background_vehicles: List[carla.Actor] = []
        self._pedestrians: List[carla.Actor] = []
        self._pedestrian_controllers: List[carla.Actor] = []
        self._client_vehicles: Dict[str, carla.Actor] = {}  # session_id → actor

        # Session manager (shared with HTTP routes; has its own lock)
        self.sessions = SessionManager()

        # V2X network (created in start())
        self.v2x_network = None

        # Simulation thread
        self._exec = SimThreadExecutor()
        self._ready = threading.Event()
        self._running = False
        self._tick_thread: Optional[threading.Thread] = None
        self._frame = 0
        self._start_time: Optional[float] = None
        self._last_v2x_frame = 0
        self._last_session_check_frame = 0

        # Published, immutable-by-convention snapshots read by other threads.
        # A reader sees either the previous or the next dict, never a half-built one.
        self._telemetry: Dict[str, dict] = {}
        self._v2x_payloads: Dict[str, Optional[dict]] = {}

        # Callback fired when a session expires (so WS can notify the client)
        self.on_session_expired: Optional[Callable[[str], None]] = None

        # Per-client LiDAR collectors (session_id → LiDARDataCollector)
        # durations (ms) of recent ticks and their phases, for /admin/status (appended by the simulation thread only)
        self._timing = {k: collections.deque(maxlen=1000) for k in
                        ("tick", "world_tick", "lidar", "v2x_update", "v2x_publish")}
        self.lidar: Optional[LidarManager] = None  # created in start(), driven from the simulation thread

    # ------------------------------------------------------------------ #
    # Threading helpers                                                    #
    # ------------------------------------------------------------------ #

    def submit(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Future:
        """Run ``fn`` on the simulation thread between two ticks; returns a Future."""
        return self._exec.submit(fn, *args, **kwargs)

    def _require_sim_thread(self) -> None:
        if not self._exec.is_sim_thread():
            raise RuntimeError(
                "CARLA state may only be changed on the simulation thread; use submit()"
            )

    def _tick(self) -> None:
        """The only place that ticks the world; ``_frame`` follows CARLA's own frame id."""
        self._frame = self.world.tick()

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Connect to CARLA, apply settings, start the simulation thread."""
        cfg = self.config
        logger.info("Connecting to CARLA at %s:%s", self.host, self.port)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(cfg.timeout_s)
        self.world = self.client.get_world()
        self._map_name = self.world.get_map().name

        logger.info("Connected to map: %s", self._map_name)

        # Store original settings for restoration on stop
        self._original_settings = self.world.get_settings()

        # Synchronous mode + physics substepping on a fresh copy (WorldConfig has already
        # checked fixed_delta_seconds <= max_substep_delta_time * max_substeps).
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = cfg.fixed_delta_seconds
        settings.substepping = True
        settings.max_substep_delta_time = cfg.max_substep_delta_time
        settings.max_substeps = cfg.max_substeps
        self.world.apply_settings(settings)

        # Traffic Manager
        self.tm = self.client.get_trafficmanager(cfg.tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(cfg.seed)
        self.tm.set_global_distance_to_leading_vehicle(cfg.safety_distance_m)
        self.tm.global_percentage_speed_difference(cfg.global_speed_pct)
        try:  # walkers pick random navigation targets: seed that stream too (reproducibility)
            self.world.set_pedestrians_seed(cfg.seed)
        except Exception as exc:
            logger.warning("could not seed pedestrians: %s", exc)

        self.bp_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        logger.info("Map has %d spawn points", len(self.spawn_points))

        # LiDAR on request (sensors are created per session, never at start-up)
        self.lidar = LidarManager(
            CarlaLidarBackend(self.world), cfg.lidar, cfg.fixed_delta_seconds, seed=cfg.seed,
        )
        self.lidar.add_event_listener(lambda ev: self._record_event("lidar", **ev))
        self.lidar.add_raw_listener(lambda info, lf: self.recorder.lidar_frame(info, lf) if self.recorder else None)

        # V2X network (V2V + V2I RSU + V2P VRU)
        if self.v2x_enabled:
            self._init_v2x()

        self._running = True
        self._start_time = time.monotonic()
        self._ready.clear()
        self._tick_thread = threading.Thread(
            target=self._sim_main, daemon=True, name="carla-tick"
        )
        self._tick_thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError("simulation thread did not start")
        logger.info("Persistent CARLA server started (%.0f FPS target)", 1 / self.fixed_delta_seconds)

    def stop(self) -> None:
        """Stop the simulation thread; it destroys all managed actors on its way out."""
        logger.info("Stopping persistent CARLA server…")
        self._running = False
        thread = self._tick_thread
        if thread and thread.is_alive():
            thread.join(timeout=30.0)
        else:
            # Simulation thread never ran (start() failed part-way): clean up directly.
            self._cleanup_all()
            self._restore_settings()
        logger.info("Persistent CARLA server stopped")

    def _restore_settings(self) -> None:
        if self.world and self._original_settings:
            try:
                self.world.apply_settings(self._original_settings)
                logger.info("World settings restored")
            except Exception as e:
                logger.warning("Could not restore world settings: %s", e)

    def _init_v2x(self) -> None:
        try:
            from src.v2x.network import V2XNetwork
            from src.v2x.rsu import RoadSideUnit
            self.v2x_network = V2XNetwork(
                max_range=self.v2x_range,
                update_rate_hz=self.config.v2x_update_rate_hz,
                world=self.world,
            )
            # One RSU per CARLA traffic-light group (= one signalised intersection), ordered
            # by the smallest actor id in the group so ids are stable between runs (S7).
            all_tl = list(self.world.get_actors().filter("traffic.traffic_light"))
            groups: dict = {}
            for tl in sorted(all_tl, key=lambda t: t.id):
                try:
                    members = list(tl.get_group_traffic_lights())
                except Exception:
                    members = []
                members.append(tl)
                key = min(m.id for m in members)
                groups.setdefault(key, {}).update({m.id: m for m in members})
            for rsu_idx, key in enumerate(sorted(groups)):
                lights = [groups[key][i] for i in sorted(groups[key])]
                rsu = RoadSideUnit.from_traffic_lights(lights, intersection_id=rsu_idx + 1)
                self.v2x_network.register_rsu(rsu)
            logger.info(
                "V2X network initialised: range=%.0f m, %d RSUs",
                self.v2x_range, self.v2x_network.rsu_count,
            )
        except ImportError:
            logger.warning("V2X module not available — V2X disabled")
            self.v2x_network = None

    # ------------------------------------------------------------------ #
    # Simulation thread                                                    #
    # ------------------------------------------------------------------ #

    def _sim_main(self) -> None:
        """Body of the simulation thread: tick loop, then orderly shutdown."""
        self._exec.bind_current_thread()
        self._last_v2x_frame = self._last_session_check_frame = self._frame
        self._ready.set()
        try:
            self._tick_loop()
        finally:
            self._exec.close("simulation stopped")
            try:
                self._cleanup_all()
            finally:
                self._restore_settings()

    def _tick_loop(self) -> None:
        logger.info("Tick loop running…")
        every_n = self.config.v2x_every_n_frames
        while self._running:
            # Real-time pacing: record wall-clock time before tick so we can
            # sleep the remainder of fixed_delta_seconds afterwards.
            tick_start = time.monotonic()
            try:
                self._exec.drain()  # submitted work runs here, at a frame boundary
                t_a = time.monotonic()
                self._tick()
                snapshot = self.world.get_snapshot()
                t_b = time.monotonic()
                self._sim_time = float(snapshot.timestamp.elapsed_seconds)
                self._publish_telemetry(snapshot)
                self._lidar_tick(snapshot)
                t_c = time.monotonic()
                self._run_periodic_work(snapshot, every_n)
                self._timing["world_tick"].append((t_b - t_a) * 1000.0)
                self._timing["lidar"].append((t_c - t_b) * 1000.0)

            except RuntimeError as exc:
                logger.error("Tick error (frame %d): %s", self._frame, exc)
                time.sleep(0.1)  # brief back-off before retry
                continue

            elapsed = time.monotonic() - tick_start
            self._timing["tick"].append(elapsed * 1000.0)
            sleep_for = self.fixed_delta_seconds - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

        logger.info("Tick loop stopped at frame %d", self._frame)

    def _lidar_tick(self, snapshot) -> None:
        """Advance LiDAR sensors for the frame just ticked. A sensor problem must not stop the simulation."""
        if self.lidar is None:
            return
        try:
            self.lidar.on_tick(self._frame, float(snapshot.timestamp.elapsed_seconds))
        except Exception:
            logger.exception("LiDAR update failed at frame %d (simulation continues)", self._frame)

    def _run_periodic_work(self, snapshot, every_n: int) -> None:
        """V2X update and session expiry. An error here must not stop the simulation."""
        if self._frame - self._last_v2x_frame >= every_n:
            self._last_v2x_frame = self._frame
            if self.v2x_network is not None:
                try:
                    t0 = time.monotonic()
                    self.v2x_network.update(snapshot)
                    t1 = time.monotonic()
                    self._advisory_step(self._sim_time)
                    self._publish_v2x()
                    self._timing["v2x_update"].append((t1 - t0) * 1000.0)
                    self._timing["v2x_publish"].append((time.monotonic() - t1) * 1000.0)
                except Exception:
                    logger.exception("V2X update failed at frame %d (simulation continues)", self._frame)
            self._record_v2x(snapshot)  # after the update, so stations and links belong to the same frame

        if self._frame - self._last_session_check_frame >= _SESSION_CHECK_EVERY_N_FRAMES:
            self._last_session_check_frame = self._frame
            try:
                self._check_expired_sessions()
            except Exception:
                logger.exception("Session expiry check failed at frame %d (simulation continues)", self._frame)

    def _publish_telemetry(self, snapshot) -> None:
        """Telemetry of every client vehicle, from the snapshot of the frame just ticked."""
        telemetry: Dict[str, dict] = {}
        sim_time = float(snapshot.timestamp.elapsed_seconds)
        for sid, actor in self._client_vehicles.items():
            snap = snapshot.find(actor.id)
            if snap is None:
                continue
            telemetry[sid] = self._telemetry_entry(
                actor.id, snap.get_transform(), snap.get_velocity(), snapshot.frame, sim_time
            )
        self._telemetry = telemetry

    @staticmethod
    def _telemetry_entry(actor_id, transform, velocity, frame, sim_time) -> dict:
        speed_kmh = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5 * 3.6
        return {
            "actor_id": actor_id,
            "x": round(transform.location.x, 2),
            "y": round(transform.location.y, 2),
            "z": round(transform.location.z, 2),
            "yaw": round(transform.rotation.yaw, 1),
            "speed_kmh": round(speed_kmh, 1),
            "frame": frame,
            "sim_time": sim_time,
        }

    def _publish_v2x(self) -> None:
        payloads: Dict[str, Optional[dict]] = {}
        for sid in self._client_vehicles:
            try:
                payloads[sid] = self._build_v2x_data(sid)
            except Exception as exc:  # one bad client must not stop the others
                logger.warning("V2X payload for session %s failed: %s", sid, exc)
        self._v2x_payloads = payloads

    def _check_expired_sessions(self) -> None:
        for session in self.sessions.get_expired_sessions():
            logger.info("Session %s expired — removing vehicle", session.session_id)
            self._despawn_client_vehicle(session.session_id)
            self.sessions.end_session(session.session_id)
            if self.on_session_expired:
                self.on_session_expired(session.session_id)

    # ------------------------------------------------------------------ #
    # Admin — background traffic  (simulation thread only: use submit())  #
    # ------------------------------------------------------------------ #

    def spawn_background_traffic(self, count: int) -> int:
        """Spawn *count* autopilot vehicles. Returns number actually spawned."""
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        occupied = self._all_vehicle_locations()
        free_points = [
            sp for sp in self.spawn_points
            if not self._too_close(sp.location, occupied, min_dist=8.0)
        ]
        self._rng.shuffle(free_points)
        targets = free_points[:count]

        bps = [
            bp for bp in self.bp_lib.filter("vehicle.*")
            if int(bp.get_attribute("number_of_wheels")) >= 4
        ]
        if not bps:
            logger.error("No 4-wheel blueprints available")
            return 0

        batch = []
        for sp in targets:
            bp = self._rng.choice(bps)
            if bp.has_attribute("color"):
                color = self._rng.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)
            batch.append(
                carla.command.SpawnActor(bp, sp).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, self.tm.get_port()
                    )
                )
            )

        results = self.client.apply_batch_sync(batch, False)
        spawned = 0
        for r in results:
            if r.error:
                logger.debug("Spawn error: %s", r.error)
                continue
            actor = self.world.get_actor(r.actor_id)
            if actor:
                self._background_vehicles.append(actor)
                profile = self._assign_driver(actor)
                self._maybe_register_background_v2x(actor)
                self._record_event("spawn_background_vehicle", actor_id=actor.id, type_id=actor.type_id,
                                   driver=profile.name if profile else None,
                                   driver_settings=profile.tm_settings() if profile else None,
                                   v2x_equipped=self.v2x_network is not None and actor.id in self.v2x_network.vehicles)
                spawned += 1

        logger.info("Spawned %d / %d background vehicles", spawned, count)
        return spawned

    def _assign_driver(self, actor) -> Optional[DriverProfile]:
        """Draw a seeded driver profile for a background vehicle and apply it through the Traffic Manager."""
        if not self.config.drivers_enabled:
            return None
        profile = self._drivers.next_profile()
        self._driver_of[actor.id] = profile
        failed = apply_profile(self.tm, actor, profile)
        if failed:
            self._record_event("driver_setting_failed", actor_id=actor.id, settings=failed)
        return profile

    def _advisory_step(self, sim_time: float) -> None:
        """V2X-assisted driving (optional): a vehicle that RECEIVED threatening BSMs slows down for a while."""
        net = self.v2x_network
        if net is None or not self.config.advisory.enabled:
            return
        levels = {}
        for vid in self._driver_of:
            if vid in net.vehicles:
                threats = net.get_threats(vid)
                levels[vid] = max((t["level"] for t in threats), default=0)
        for vid, action in self._advisory.update(sim_time, levels):
            profile = self._driver_of.get(vid)
            actor = next((a for a in self._background_vehicles if a.id == vid), None)
            if profile is None or actor is None:
                continue
            slow = profile.speed_pct + self.config.advisory.extra_slowdown_pct if action == "slow" else profile.speed_pct
            try:
                self.tm.vehicle_percentage_speed_difference(actor, slow)
            except Exception as exc:
                logger.warning("advisory could not set speed for %s: %s", vid, exc)
            self._record_event("advisory", actor_id=vid, action=action, speed_pct=slow, threat_level=levels.get(vid))

    def _maybe_register_background_v2x(self, actor) -> None:
        """Make a background vehicle a V2X station with probability ``v2x_penetration_rate``."""
        if self.v2x_network is None:
            return
        rate = self.config.v2x_penetration_rate
        if rate < 1.0 and self._v2x_rng.random() >= rate:
            return
        self.v2x_network.register(actor.id, actor)

    def despawn_background_traffic(self) -> int:
        """Destroy all admin-managed background vehicles."""
        self._require_sim_thread()
        count = len(self._background_vehicles)
        if count == 0:
            return 0
        if self.v2x_network is not None:
            for v in self._background_vehicles:
                self.v2x_network.unregister(v.id)
        for v in self._background_vehicles:
            self._driver_of.pop(v.id, None)
            self._advisory.forget(v.id)
        self._record_event("despawn_background_traffic", count=count)
        batch = [carla.command.DestroyActor(v) for v in self._background_vehicles]
        self.client.apply_batch_sync(batch, False)
        self._background_vehicles.clear()
        logger.info("Despawned %d background vehicles", count)
        return count

    # ------------------------------------------------------------------ #
    # Admin — pedestrians                                                  #
    # ------------------------------------------------------------------ #

    def spawn_pedestrians(self, count: int) -> int:
        """Spawn *count* AI-controlled pedestrians. Returns number spawned."""
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        ped_bps = list(self.bp_lib.filter("walker.pedestrian.*"))
        ctrl_bp = self.bp_lib.find("controller.ai.walker")
        if not ped_bps or ctrl_bp is None:
            logger.error("Pedestrian blueprints not found")
            return 0

        # Collect random navigation locations first
        spawn_batch = []
        for _ in range(count):
            loc = self.world.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = self._rng.choice(ped_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            spawn_batch.append(carla.command.SpawnActor(bp, carla.Transform(loc)))

        results = self.client.apply_batch_sync(spawn_batch, False)
        ped_actors: List[carla.Actor] = []
        for r in results:
            if r.error:
                continue
            actor = self.world.get_actor(r.actor_id)
            if actor:
                self._pedestrians.append(actor)
                ped_actors.append(actor)

        # Spawn AI controllers attached to each pedestrian
        ctrl_batch = [
            carla.command.SpawnActor(ctrl_bp, carla.Transform(), p)
            for p in ped_actors
        ]
        self._tick()  # needed before attaching controllers (counted in _frame)
        ctrl_results = self.client.apply_batch_sync(ctrl_batch, False)

        for r in ctrl_results:
            if r.error:
                continue
            ctrl = self.world.get_actor(r.actor_id)
            if ctrl:
                self._pedestrian_controllers.append(ctrl)
                ctrl.start()
                dest = self.world.get_random_location_from_navigation()
                if dest:
                    ctrl.go_to_location(dest)
                ctrl.set_max_speed(1.4)

        spawned = len(ped_actors)
        # Register spawned pedestrians as V2P VRUs in the V2X network
        if self.v2x_network is not None:
            from src.v2x.messages import VRUProfile
            for ped in ped_actors:
                self.v2x_network.register_vru(ped.id, ped, VRUProfile.PEDESTRIAN)
        self._record_event("spawn_pedestrians", requested=count, spawned=spawned)
        logger.info("Spawned %d / %d pedestrians", spawned, count)
        return spawned

    def despawn_pedestrians(self) -> int:
        """Destroy all admin-managed pedestrians."""
        self._require_sim_thread()
        for ctrl in self._pedestrian_controllers:
            try:
                ctrl.stop()
            except Exception:
                pass
        # Unregister VRUs from V2X network before destroying actors
        if self.v2x_network is not None:
            for ped in self._pedestrians:
                try:
                    self.v2x_network.unregister_vru(ped.id)
                except Exception:
                    pass
        count = len(self._pedestrians)
        batch = (
            [carla.command.DestroyActor(c) for c in self._pedestrian_controllers]
            + [carla.command.DestroyActor(p) for p in self._pedestrians]
        )
        if batch:
            self.client.apply_batch_sync(batch, False)
        self._pedestrian_controllers.clear()
        self._pedestrians.clear()
        logger.info("Despawned %d pedestrians", count)
        return count

    # ------------------------------------------------------------------ #
    # Client — leading vehicle                                             #
    # ------------------------------------------------------------------ #

    def spawn_client_vehicle(
        self,
        session_id: str,
        blueprint_id: str,
        spawn_index: Optional[int] = None,
    ) -> Optional[int]:
        """
        Spawn a leading vehicle for a client session.

        Returns the CARLA actor ID, or None if spawn failed.
        """
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        session = self.sessions.get_session(session_id)
        if session is None:
            raise ValueError(f"Unknown session: {session_id}")

        # Already has a vehicle?
        if session_id in self._client_vehicles:
            return self._client_vehicles[session_id].id

        bp = self._resolve_blueprint(blueprint_id)
        if bp.has_attribute("color"):
            bp.set_attribute("color", "255,50,50")  # distinctive red

        sp = self._pick_spawn_point(spawn_index)
        actor = self.world.try_spawn_actor(bp, sp)
        if actor is None:
            logger.error("Failed to spawn vehicle for session %s", session_id)
            return None

        actor.set_autopilot(True, self.tm.get_port())
        self._client_vehicles[session_id] = actor
        session.vehicle_actor_id = actor.id
        self._record_event("client_join", session_id=session_id, actor_id=actor.id, blueprint=blueprint_id)

        # Publish a first telemetry sample now so the joining client can read it at once
        # (the next per-frame publish replaces it).
        self._telemetry = {
            **self._telemetry,
            session_id: self._telemetry_entry(
                actor.id, actor.get_transform(), actor.get_velocity(), self._frame,
                self._frame * self.fixed_delta_seconds,
            ),
        }

        if self.v2x_network is not None:
            self.v2x_network.register(actor.id, actor)

        logger.info(
            "Spawned client vehicle actor=%d for session %s (%s)",
            actor.id, session_id, session.client_name,
        )
        return actor.id

    def despawn_client_vehicle(self, session_id: str) -> None:
        """Despawn a client vehicle (called on leave, kick or expiry)."""
        self._require_sim_thread()
        self._despawn_client_vehicle(session_id)

    def _despawn_client_vehicle(self, session_id: str) -> None:
        self._record_event("client_leave", session_id=session_id)
        if self.lidar is not None:
            self.lidar.destroy_owner_now(session_id)  # stop the sensor before its parent vehicle goes
        actor = self._client_vehicles.pop(session_id, None)
        self._telemetry = {k: v for k, v in self._telemetry.items() if k != session_id}
        self._v2x_payloads = {k: v for k, v in self._v2x_payloads.items() if k != session_id}
        if actor is None:
            return
        try:
            if self.v2x_network is not None:
                self.v2x_network.unregister(actor.id)
        except Exception:
            pass
        try:
            actor.destroy()
        except Exception as exc:
            logger.warning("Error destroying client vehicle: %s", exc)
        logger.info("Despawned client vehicle for session %s", session_id)

    # ------------------------------------------------------------------ #
    # State queries (any thread; read published data, never call CARLA)    #
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        v2x = self.v2x_network
        return {
            "running": self._running,
            "frame": self._frame,
            "elapsed_seconds": round(elapsed, 1),
            "map": self._map_name,
            "background_vehicles": len(self._background_vehicles),
            "pedestrians": len(self._pedestrians),
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self._client_vehicles),
            "v2x_enabled": v2x is not None,
            "v2x_range": self.v2x_range if v2x else 0,
            "rsu_count": v2x.rsu_count if v2x else 0,
            "vru_count": v2x.vru_count if v2x else 0,
            "timing_ms": self.get_timing(),
            "v2x_phase_ms": {k: round(v, 2) for k, v in dict(v2x.phase_ms).items()} if v2x else {},
        }

    def get_timing(self) -> dict:
        """p50 / p95 / max (ms) over the last ~1000 samples of each phase. ``tick`` is the work per frame (excluding the
        pacing sleep): the simulation holds 20 FPS only while its p95 stays below ``fixed_delta_seconds`` * 1000."""
        out = {}
        for name, samples in self._timing.items():
            vals = sorted(samples.copy())
            if not vals:
                out[name] = {"n": 0}
                continue
            out[name] = {"n": len(vals), "p50": round(vals[len(vals) // 2], 2),
                         "p95": round(vals[min(len(vals) - 1, int(len(vals) * 0.95))], 2), "max": round(vals[-1], 2)}
        return out

    def get_vehicle_telemetry(self, session_id: str) -> Optional[dict]:
        """Last published telemetry for a client's vehicle (stamped with its frame)."""
        return self._telemetry.get(session_id)

    def get_vehicle_v2x_data(self, session_id: str) -> Optional[dict]:
        """Last published V2X payload for a client (updated at each V2X update)."""
        return self._v2x_payloads.get(session_id)

    def _build_v2x_data(self, session_id: str) -> Optional[dict]:
        """Build the V2X payload for one client (simulation thread only)."""
        if self.v2x_network is None:
            return None
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            return None
        vid = actor.id
        bsm = self.v2x_network.get_bsm(vid)
        if bsm is None:
            return None  # first few frames before first V2X update

        def _safe_ttc(ttc: float) -> Optional[float]:
            return None if not math.isfinite(ttc) else round(ttc, 2)

        vx, vy = bsm.x_m, bsm.y_m  # CARLA world x/y, metres
        neighbors = self.v2x_network.get_neighbors(vid)
        threats = self.v2x_network.get_threats(vid)

        net = self.v2x_network

        # SPAT / MAP this vehicle actually RECEIVED through the channel (not a radius test)
        spat_nearby = []
        for iid, spat in net.spat_inbox.get(vid, {}).items():
            rsu = net.get_rsu(iid)
            dist = (
                math.hypot(vx - rsu.config.position[0], vy - rsu.config.position[1])
                if rsu is not None else None
            )
            spat_nearby.append({
                "intersection_id": spat.intersection_id,
                "distance": round(dist, 1) if dist is not None else None,
                "states": [
                    {
                        "signal_group_id": s.signal_group_id,
                        "movement_event": s.movement_event,
                        "likely_time_s": round(s.likely_time_s, 1)
                        if s.likely_time_s is not None else None,
                    }
                    for s in spat.states
                ],
            })
        map_received = sorted(net.map_inbox.get(vid, {}))

        # VRUAMs this vehicle actually RECEIVED through the channel
        vru_nearby = []
        for vruam in net.vruam_inbox.get(vid, {}).values():
            px, py = vruam.position[0], vruam.position[1]
            vru_nearby.append({
                "station_id": vruam.station_id,
                "profile": vruam.vru_profile.name,
                "distance": round(math.hypot(vx - px, vy - py), 1),
                "speed": round(vruam.speed, 2),
                "motion_state": vruam.motion_state,
            })

        return {
            "bsm": {
                "vehicle_id": bsm.vehicle_id,
                "speed": round(bsm.speed, 2),
                "heading": round(bsm.heading, 1),
                "position": {
                    "x": round(bsm.x_m, 2),
                    "y": round(bsm.y_m, 2),
                    "z": round(bsm.z_m, 2),
                },
            },
            "neighbors": [
                {
                    "vehicle_id": n.vehicle_id,
                    "distance": round(
                        self.v2x_network.get_distance(vid, n.vehicle_id) or 0.0, 1
                    ),
                    "speed": round(n.speed, 2),
                    "heading": round(n.heading, 1),
                }
                for n in neighbors
            ],
            "threats": [
                {
                    "other_vehicle_id": t["other_vehicle_id"],
                    "threat_level": t["level"],
                    "time_to_collision": _safe_ttc(t["ttc"]),
                    "distance": round(t["distance"], 1),
                }
                for t in threats
            ],
            "spat_nearby": spat_nearby,
            "map_received": map_received,
            "vru_nearby": vru_nearby,
        }

    # ------------------------------------------------------------------ #
    # Run recording (handoff §5.6): simulation thread for start/stop       #
    # ------------------------------------------------------------------ #

    def start_run(self, name: str, record_lidar: bool = False, record_links: bool = True) -> dict:
        """Start writing a run directory (events, ground-truth stations, V2X link results, optional raw LiDAR)."""
        self._require_sim_thread()
        if self.recorder is not None:
            raise RunAlreadyRecording(f"a run is already being recorded: {self.recorder.run_id}")
        from pathlib import Path as _Path
        cfg = self.config
        recorder = RunRecorder(
            _Path(cfg.runs_dir), name, self._run_manifest(),
            start_frame=self._frame, start_sim_time=self._sim_time,
            git_sha=_git_sha(), record_links=record_links, record_lidar=record_lidar,
        )
        self.recorder = recorder
        self._record_event("run_start", name=name)
        for a in self._background_vehicles:  # actors that already exist when the run starts
            p = self._driver_of.get(a.id)
            self._record_event("existing_background_vehicle", actor_id=a.id, driver=p.name if p else None,
                               driver_settings=p.tm_settings() if p else None)
        for sid, a in self._client_vehicles.items():
            self._record_event("existing_client_vehicle", session_id=sid, actor_id=a.id)
        logger.info("Recording run %s", recorder.run_id)
        return recorder.summary()

    def stop_run(self) -> Optional[dict]:
        self._require_sim_thread()
        return self._finish_run()

    def _finish_run(self) -> Optional[dict]:
        recorder, self.recorder = self.recorder, None
        if recorder is None:
            return None
        self._record_event_to(recorder, "run_stop")
        manifest = recorder.stop(self._frame, self._sim_time)
        logger.info("Run %s stopped: %s", recorder.run_id, manifest["row_counts"])
        return manifest

    def get_run_status(self) -> dict:
        from pathlib import Path as _Path
        rec = self.recorder
        return {"current": rec.summary() if rec else None, "runs": list_runs(_Path(self.config.runs_dir))}

    def _run_manifest(self) -> dict:
        import platform
        cfg = self.config
        try:
            import numpy
            numpy_version = numpy.__version__
        except Exception:
            numpy_version = None
        try:
            server_version = self.client.get_server_version()
            client_version = self.client.get_client_version()
        except Exception:
            server_version = client_version = None
        from src.traffic.drivers import PROFILES
        from dataclasses import asdict as _asdict
        return {
            "config": cfg.model_dump(mode="json"),
            "map": self._map_name,
            "fixed_delta_seconds": cfg.fixed_delta_seconds,
            "seeds": {"master": cfg.seed, "v2x_penetration": cfg.seed + 1, "drivers": cfg.seed + 2,
                      "dsrc": self.v2x_network._dsrc_channel.config.random_seed if self.v2x_network else None,
                      "traffic_manager": cfg.seed, "pedestrians": cfg.seed},
            "driver_profiles": {k: _asdict(v) for k, v in PROFILES.items()},
            "dsrc_config": _asdict(self.v2x_network._dsrc_channel.config) if self.v2x_network else None,
            "awareness_range_m": self.v2x_range if self.v2x_network else None,
            "v2x_update_every_n_frames": cfg.v2x_every_n_frames,
            "versions": {"python": platform.python_version(), "numpy": numpy_version,
                         "carla_client": client_version, "carla_server": server_version, "platform": platform.platform()},
            "reproducibility": {
                "synchronous_mode": True, "realtime_pacing": True,
                "note": "the tick loop sleeps to real time, so the timing of admin/client commands relative to frames is not "
                        "reproducible; same seed + same command frames give the same V2X data only if CARLA and the "
                        "Traffic Manager are deterministic on this machine (not verified live).",
            },
        }

    def _record_event(self, kind: str, **data) -> None:
        rec = self.recorder
        if rec is not None:
            self._record_event_to(rec, kind, **data)

    def _record_event_to(self, rec: RunRecorder, kind: str, **data) -> None:
        try:
            rec.event(self._frame, self._sim_time, kind, **data)
        except Exception:
            logger.exception("recording event %s failed", kind)

    def _record_v2x(self, snapshot) -> None:
        """Ground truth of every station plus this update's link results (full precision, never decimated)."""
        rec = self.recorder
        if rec is None:
            return
        try:
            net = self.v2x_network
            equipped = set(net.vehicles) if net is not None else set()
            stations = []
            kinds = [(a, "client") for a in self._client_vehicles.values()] + [(a, "background") for a in self._background_vehicles]
            for actor, kind in kinds:
                snap = snapshot.find(actor.id)
                if snap is None:
                    continue
                t, v = snap.get_transform(), snap.get_velocity()
                p = self._driver_of.get(actor.id)
                stations.append({
                    "id": actor.id, "kind": kind, "v2x_equipped": actor.id in equipped, "driver": p.name if p else None,
                    "x_m": t.location.x, "y_m": t.location.y, "z_m": t.location.z, "yaw_deg": t.rotation.yaw,
                    "vx_mps": v.x, "vy_mps": v.y, "speed_mps": math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
                })
            if net is not None:
                for iid, rsu in net.get_rsus().items():
                    stations.append({"id": iid, "kind": "rsu", "x_m": rsu.config.position[0],
                                     "y_m": rsu.config.position[1], "z_m": rsu.config.position[2]})
                for vid, m in net.get_all_vruam().items():
                    stations.append({"id": vid, "kind": "vru", "x_m": m.position[0], "y_m": m.position[1],
                                     "z_m": m.position[2], "speed_mps": m.speed, "heading_deg": m.heading})
            rec.stations(self._frame, self._sim_time, stations)
            if net is not None:
                rec.links(self._frame, self._sim_time, net.get_link_results())
        except Exception:
            logger.exception("recording V2X frame %d failed (simulation continues)", self._frame)

    # ------------------------------------------------------------------ #
    # LiDAR (sim thread for changes; published snapshots for reads)         #
    # ------------------------------------------------------------------ #

    def enable_client_lidar(self, session_id: str, config: Optional[LidarConfig] = None) -> dict:
        """Request (or reconfigure) a LiDAR sensor on the session's own vehicle.

        Returns the sensor's info dict. Raises ``LidarError`` (budget 409, forbidden 403, ...).
        """
        self._require_sim_thread()
        if self.lidar is None:
            raise LidarError("LiDAR is not available")
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            raise LidarError("session has no vehicle", session_id=session_id)
        existing = self.lidar.sensor_of_owner(session_id)
        if existing is not None:
            if config is not None:
                self.lidar.reconfigure(existing["sensor_id"], config, requester=session_id)
            return self.lidar.get_info(existing["sensor_id"])
        sensor = self.lidar.request(session_id, actor.id, config, own_actor_id=actor.id)
        logger.info("LiDAR requested for session %s (actor=%d, sensor=%d)", session_id, actor.id, sensor.sensor_id)
        return self.lidar.get_info(sensor.sensor_id)

    def disable_client_lidar(self, session_id: str) -> bool:
        self._require_sim_thread()
        existing = self.lidar.sensor_of_owner(session_id) if self.lidar else None
        if existing is None:
            return False
        self.lidar.release(existing["sensor_id"], requester=session_id)
        return True

    def get_client_lidar_info(self, session_id: str) -> Optional[dict]:
        return self.lidar.sensor_of_owner(session_id) if self.lidar else None

    def get_client_lidar_frame(
        self, session_id: str, profile: StreamProfile = StreamProfile(), after_frame: int = -1
    ):
        """(frame, protocol-v3 bytes) of the newest frame after ``after_frame`` for this session's sensor, or None."""
        info = self.get_client_lidar_info(session_id)
        if info is None:
            return None
        return self.lidar.get_stream_frame(info["sensor_id"], profile, after_frame)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _resolve_blueprint(self, blueprint_id: str) -> carla.ActorBlueprint:
        bp = self.bp_lib.find(blueprint_id)
        if bp is not None:
            return bp
        for fallback_id in _FALLBACK_BLUEPRINTS:
            bp = self.bp_lib.find(fallback_id)
            if bp is not None:
                return bp
        return list(self.bp_lib.filter("vehicle.*"))[0]

    def _pick_spawn_point(self, spawn_index: Optional[int]) -> carla.Transform:
        if spawn_index is not None and 0 <= spawn_index < len(self.spawn_points):
            return self.spawn_points[spawn_index]
        occupied = self._all_vehicle_locations()
        free = [
            sp for sp in self.spawn_points
            if not self._too_close(sp.location, occupied, min_dist=10.0)
        ]
        if free:
            return self._rng.choice(free)
        return self._rng.choice(self.spawn_points)

    def _all_vehicle_locations(self) -> List[carla.Location]:
        locs = []
        for actor in list(self._background_vehicles) + list(self._client_vehicles.values()):
            try:
                locs.append(actor.get_location())
            except Exception:
                pass
        return locs

    @staticmethod
    def _too_close(loc: carla.Location, others: List[carla.Location], min_dist: float) -> bool:
        for other in others:
            if ((loc.x - other.x) ** 2 + (loc.y - other.y) ** 2) ** 0.5 < min_dist:
                return True
        return False

    def _cleanup_all(self) -> None:
        """Destroy everything this server spawned (simulation thread, or start() failure)."""
        if self.recorder is not None:  # finish the run file cleanly on every way out
            try:
                self._finish_run()
            except Exception as exc:
                logger.warning("closing the run recording: %s", exc)
        # Stop LiDAR sensor callbacks before batch-destroying vehicles
        if self.lidar is not None:
            try:
                self.lidar.shutdown()
            except Exception as exc:
                logger.warning("LiDAR shutdown: %s", exc)

        all_actors = (
            list(self._client_vehicles.values())
            + self._background_vehicles
            + self._pedestrian_controllers
            + self._pedestrians
        )
        if all_actors and self.client:
            batch = [carla.command.DestroyActor(a) for a in all_actors]
            try:
                self.client.apply_batch_sync(batch, False)
            except Exception as exc:
                logger.warning("Cleanup error: %s", exc)
        self._client_vehicles.clear()
        self._background_vehicles.clear()
        self._pedestrian_controllers.clear()
        self._pedestrians.clear()
        self._telemetry = {}
        self._v2x_payloads = {}
