"""
Persistent CARLA world — the heart of the live-server architecture.

The admin starts this once; it runs indefinitely in a background thread.
Clients (laptop, raspi, workstation…) join and leave dynamically — each
gets one "leading vehicle" tracked here.

Usage
-----
    world = PersistentCARLAServer(host, port, config)
    world.start()          # connect to CARLA, start tick loop
    ...
    world.stop()           # graceful shutdown + actor cleanup

Thread safety
-------------
All actor-list mutations go through ``_lock``.  The tick thread only reads
the vehicle dict (for V2X updates) and calls ``_check_expired_sessions``
which re-acquires the lock for any despawn operations.
"""

import carla
import threading
import logging
import random
import time
from typing import Any, Callable, Dict, List, Optional

from src.server.session_manager import SessionManager, ClientSession

logger = logging.getLogger(__name__)

# Blueprints that are always available in CARLA 0.9.x Town maps.
_FALLBACK_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.lincoln.mkz_2020",
]


def _cleanup_lidar_collector(collector, session_id: str) -> None:
    """Background cleanup target — runs in a daemon thread after session ends."""
    try:
        collector.cleanup()
    except Exception as exc:
        logger.warning("LiDAR cleanup for session %s: %s", session_id, exc)


class PersistentCARLAServer:
    """
    Manages a persistent CARLA world that runs indefinitely.

    Background traffic and pedestrians are controlled by the admin via the
    REST API.  Each client session spawns exactly one "leading vehicle".
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        fixed_delta_seconds: float = 0.05,
        timeout: float = 30.0,
        tm_port: int = 8001,
        tm_seed: int = 42,
        global_speed_pct: float = -20.0,
        safety_distance: float = 2.5,
        v2x_enabled: bool = True,
        v2x_range: float = 75.0,
    ):
        self.host = host
        self.port = port
        self.fixed_delta_seconds = fixed_delta_seconds
        self.timeout = timeout
        self.tm_port = tm_port
        self.tm_seed = tm_seed
        self.global_speed_pct = global_speed_pct
        self.safety_distance = safety_distance
        self.v2x_enabled = v2x_enabled
        self.v2x_range = v2x_range

        # CARLA handles
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.tm: Optional[carla.TrafficManager] = None
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.spawn_points: List[carla.Transform] = []
        self._original_settings: Optional[carla.WorldSettings] = None

        # Managed actors
        self._background_vehicles: List[carla.Actor] = []
        self._pedestrians: List[carla.Actor] = []
        self._pedestrian_controllers: List[carla.Actor] = []
        self._client_vehicles: Dict[str, carla.Actor] = {}  # session_id → actor

        # Session manager (shared with HTTP routes)
        self.sessions = SessionManager()

        # V2X / LiDAR references (set externally or created here)
        self.v2x_network = None

        # State
        self._running = False
        self._tick_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame = 0
        self._start_time: Optional[float] = None

        # Callback fired when a session expires (so WS can notify the client)
        self.on_session_expired: Optional[Callable[[str], None]] = None

        # Per-client LiDAR collectors (session_id → LiDARDataCollector)
        self._client_lidar_collectors: Dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Connect to CARLA and start the continuous tick loop."""
        logger.info("Connecting to CARLA at %s:%s", self.host, self.port)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()

        logger.info("Connected to map: %s", self.world.get_map().name)

        # Store original settings for restoration on stop
        self._original_settings = self.world.get_settings()

        # Apply synchronous mode + physics substepping on a fresh copy.
        # Per CARLA docs: substepping keeps physics dt <= 0.01 s for accurate
        # vehicle dynamics.  Constraint: fixed_delta_seconds <= max_substep_delta_time
        # * max_substeps  →  0.05 <= 0.01 * 10 = 0.10 ✓
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.substepping = True
        settings.max_substep_delta_time = 0.01   # optimal physics step (< 0.01666 s)
        settings.max_substeps = 10
        self.world.apply_settings(settings)

        # Traffic Manager
        self.tm = self.client.get_trafficmanager(self.tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(self.tm_seed)
        self.tm.set_global_distance_to_leading_vehicle(self.safety_distance)
        self.tm.global_percentage_speed_difference(self.global_speed_pct)

        self.bp_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        logger.info("Map has %d spawn points", len(self.spawn_points))

        # V2X network (V2V + V2I RSU + V2P VRU)
        if self.v2x_enabled:
            self._init_v2x()

        self._running = True
        self._start_time = time.monotonic()
        self._tick_thread = threading.Thread(
            target=self._tick_loop, daemon=True, name="carla-tick"
        )
        self._tick_thread.start()
        logger.info("Persistent CARLA server started (%.0f FPS target)", 1 / self.fixed_delta_seconds)

    def stop(self) -> None:
        """Stop the tick loop and destroy all managed actors."""
        logger.info("Stopping persistent CARLA server…")
        self._running = False

        if self._tick_thread and self._tick_thread.is_alive():
            self._tick_thread.join(timeout=5.0)

        self._cleanup_all()

        if self.world and self._original_settings:
            try:
                self.world.apply_settings(self._original_settings)
                logger.info("World settings restored")
            except Exception as e:
                logger.warning("Could not restore world settings: %s", e)

        logger.info("Persistent CARLA server stopped")

    def _init_v2x(self) -> None:
        try:
            from src.v2x.network import V2XNetwork
            from src.v2x.rsu import RoadSideUnit
            self.v2x_network = V2XNetwork(
                max_range=self.v2x_range,
                update_rate_hz=2.0,
                world=self.world,
            )
            # Auto-create RSUs from CARLA traffic lights (up to 10 intersections)
            all_tl = list(self.world.get_actors().filter("traffic.traffic_light"))
            junctions_seen: dict = {}
            for tl in all_tl:
                jid = id(tl) % 10000  # stable pseudo-ID per actor
                if jid not in junctions_seen:
                    junctions_seen[jid] = tl
            for rsu_idx, (_jid, tl) in enumerate(list(junctions_seen.items())[:10]):
                rsu = RoadSideUnit.from_traffic_light(tl, intersection_id=rsu_idx + 1)
                self.v2x_network.register_rsu(rsu)
            logger.info(
                "V2X network initialised: range=%.0f m, %d RSUs",
                self.v2x_range, len(self.v2x_network._rsus),
            )
        except ImportError:
            logger.warning("V2X module not available — V2X disabled")
            self.v2x_network = None

    # ------------------------------------------------------------------ #
    # Tick loop                                                            #
    # ------------------------------------------------------------------ #

    def _tick_loop(self) -> None:
        logger.info("Tick loop running…")
        while self._running:
            # Real-time pacing: record wall-clock time before tick so we can
            # sleep the remainder of fixed_delta_seconds afterwards.  This keeps
            # the simulation at ≈ 1× real-time speed for a realistic V2X feel.
            tick_start = time.monotonic()
            try:
                self.world.tick()
                self._frame += 1

                # V2X update at 2 Hz (every 10 frames @ 20 FPS)
                if self._frame % 10 == 0:
                    if self.v2x_network is not None:
                        snapshot = self.world.get_snapshot()
                        self.v2x_network.update(snapshot)

                # Session expiry check every ~5 s (100 frames @ 20 FPS)
                if self._frame % 100 == 0:
                    self._check_expired_sessions()

            except RuntimeError as exc:
                logger.error("Tick error (frame %d): %s", self._frame, exc)
                time.sleep(0.1)  # brief back-off before retry
                continue

            # Sleep the remainder of the target frame period so the simulation
            # advances at real-time speed (real-time factor ≈ 1.0).
            elapsed = time.monotonic() - tick_start
            sleep_for = self.fixed_delta_seconds - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

        logger.info("Tick loop stopped after %d frames", self._frame)

    def _check_expired_sessions(self) -> None:
        for session in self.sessions.get_expired_sessions():
            logger.info("Session %s expired — removing vehicle", session.session_id)
            collector = self._client_lidar_collectors.pop(session.session_id, None)
            self._despawn_client_vehicle_locked(session.session_id)
            self.sessions.end_session(session.session_id)
            if self.on_session_expired:
                self.on_session_expired(session.session_id)
            if collector is not None:
                threading.Thread(
                    target=_cleanup_lidar_collector,
                    args=(collector, session.session_id),
                    daemon=True,
                ).start()

    # ------------------------------------------------------------------ #
    # Admin — background traffic                                           #
    # ------------------------------------------------------------------ #

    def spawn_background_traffic(self, count: int) -> int:
        """Spawn *count* autopilot vehicles. Returns number actually spawned."""
        if not self.world:
            raise RuntimeError("Server not started")

        with self._lock:
            occupied = self._all_vehicle_locations()
            free_points = [
                sp for sp in self.spawn_points
                if not self._too_close(sp.location, occupied, min_dist=8.0)
            ]
            random.shuffle(free_points)
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
                bp = random.choice(bps)
                if bp.has_attribute("color"):
                    color = random.choice(bp.get_attribute("color").recommended_values)
                    bp.set_attribute("color", color)
                batch.append(
                    carla.command.SpawnActor(bp, sp).then(
                        carla.command.SetAutopilot(
                            carla.command.FutureActor, True, self.tm.get_port()
                        )
                    )
                )

            results = self.client.apply_batch_sync(batch, True)
            spawned = 0
            for r in results:
                if r.error:
                    logger.debug("Spawn error: %s", r.error)
                    continue
                actor = self.world.get_actor(r.actor_id)
                if actor:
                    self._background_vehicles.append(actor)
                    spawned += 1

            logger.info("Spawned %d / %d background vehicles", spawned, count)
            return spawned

    def despawn_background_traffic(self) -> int:
        """Destroy all admin-managed background vehicles."""
        with self._lock:
            count = len(self._background_vehicles)
            if count == 0:
                return 0
            batch = [carla.command.DestroyActor(v) for v in self._background_vehicles]
            self.client.apply_batch_sync(batch, True)
            self._background_vehicles.clear()
            logger.info("Despawned %d background vehicles", count)
            return count

    # ------------------------------------------------------------------ #
    # Admin — pedestrians                                                  #
    # ------------------------------------------------------------------ #

    def spawn_pedestrians(self, count: int) -> int:
        """Spawn *count* AI-controlled pedestrians. Returns number spawned."""
        if not self.world:
            raise RuntimeError("Server not started")

        with self._lock:
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
                bp = random.choice(ped_bps)
                if bp.has_attribute("is_invincible"):
                    bp.set_attribute("is_invincible", "false")
                spawn_batch.append(carla.command.SpawnActor(bp, carla.Transform(loc)))

            results = self.client.apply_batch_sync(spawn_batch, True)
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
            self.world.tick()  # needed before attaching controllers
            ctrl_results = self.client.apply_batch_sync(ctrl_batch, True)

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
            logger.info("Spawned %d / %d pedestrians", spawned, count)
            return spawned

    def despawn_pedestrians(self) -> int:
        """Destroy all admin-managed pedestrians."""
        with self._lock:
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
                self.client.apply_batch_sync(batch, True)
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
        if not self.world:
            raise RuntimeError("Server not started")

        session = self.sessions.get_session(session_id)
        if session is None:
            raise ValueError(f"Unknown session: {session_id}")

        with self._lock:
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

            if self.v2x_network is not None:
                self.v2x_network.register(actor.id, actor)

            logger.info(
                "Spawned client vehicle actor=%d for session %s (%s)",
                actor.id, session_id, session.client_name,
            )
            return actor.id

    def despawn_client_vehicle(self, session_id: str) -> None:
        """Public: despawn a client vehicle (called on leave or kick)."""
        with self._lock:
            collector = self._client_lidar_collectors.pop(session_id, None)
            self._despawn_client_vehicle_locked(session_id)
        if collector is not None:
            threading.Thread(
                target=_cleanup_lidar_collector,
                args=(collector, session_id),
                daemon=True,
            ).start()

    def _despawn_client_vehicle_locked(self, session_id: str) -> None:
        """Must be called with ``_lock`` held (or from tick thread)."""
        actor = self._client_vehicles.pop(session_id, None)
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
    # State queries (called from HTTP routes / WebSocket tasks)           #
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        v2x = self.v2x_network
        return {
            "running": self._running,
            "frame": self._frame,
            "elapsed_seconds": round(elapsed, 1),
            "map": self.world.get_map().name if self.world else None,
            "background_vehicles": len(self._background_vehicles),
            "pedestrians": len(self._pedestrians),
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self._client_vehicles),
            "v2x_enabled": v2x is not None,
            "v2x_range": self.v2x_range if v2x else 0,
            "rsu_count": len(getattr(v2x, "_rsus", {})),
            "vru_count": len(getattr(v2x, "_vrus", {})),
        }

    def get_vehicle_telemetry(self, session_id: str) -> Optional[dict]:
        """Return live telemetry for a client's vehicle."""
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            return None
        try:
            t = actor.get_transform()
            v = actor.get_velocity()
            speed_kmh = (v.x**2 + v.y**2 + v.z**2) ** 0.5 * 3.6
            return {
                "actor_id": actor.id,
                "x": round(t.location.x, 2),
                "y": round(t.location.y, 2),
                "z": round(t.location.z, 2),
                "yaw": round(t.rotation.yaw, 1),
                "speed_kmh": round(speed_kmh, 1),
                "frame": self._frame,
            }
        except Exception as exc:
            logger.debug("Telemetry read error for %s: %s", session_id, exc)
            return None

    def get_vehicle_v2x_data(self, session_id: str) -> Optional[dict]:
        """Return full V2X payload: BSM, neighbors, threats, nearby SPAT and VRUAMs."""
        if self.v2x_network is None:
            return None
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            return None
        vid = actor.id
        bsm = self.v2x_network.get_bsm(vid)
        if bsm is None:
            return None  # first few frames before first V2X update

        import math

        def _safe_ttc(ttc: float) -> Optional[float]:
            return None if not math.isfinite(ttc) else round(ttc, 2)

        vx, vy = bsm.latitude, bsm.longitude  # CARLA x/y stored in lat/lon fields
        neighbors = self.v2x_network.get_neighbors(vid)
        threats = self.v2x_network.get_threats(vid)

        # Nearby SPAT — RSUs whose broadcast range covers the vehicle
        spat_nearby = []
        for iid, rsu in getattr(self.v2x_network, "_rsus", {}).items():
            rx, ry = rsu.config.position[0], rsu.config.position[1]
            dist = math.sqrt((vx - rx) ** 2 + (vy - ry) ** 2)
            if dist <= rsu.config.broadcast_range_m:
                spat = self.v2x_network.spat_messages.get(iid)
                if spat is not None:
                    spat_nearby.append({
                        "intersection_id": spat.intersection_id,
                        "distance": round(dist, 1),
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

        # Nearby VRUAMs — pedestrians / cyclists within 100 m
        vru_nearby = []
        for vruam in getattr(self.v2x_network, "vru_messages", {}).values():
            px, py = vruam.position[0], vruam.position[1]
            dist = math.sqrt((vx - px) ** 2 + (vy - py) ** 2)
            if dist <= 100.0:
                vru_nearby.append({
                    "station_id": vruam.station_id,
                    "profile": vruam.vru_profile.name,
                    "distance": round(dist, 1),
                    "speed": round(vruam.speed, 2),
                    "motion_state": vruam.motion_state,
                })

        return {
            "bsm": {
                "vehicle_id": bsm.vehicle_id,
                "speed": round(bsm.speed, 2),
                "heading": round(bsm.heading, 1),
                "position": {
                    "x": round(bsm.latitude, 2),
                    "y": round(bsm.longitude, 2),
                    "z": round(bsm.elevation, 2),
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
            "vru_nearby": vru_nearby,
        }

    def enable_client_lidar(self, session_id: str) -> bool:
        """Attach a LiDAR sensor to the client's vehicle for per-session streaming."""
        try:
            from src.visualization.lidar.collector import LiDARDataCollector
        except ImportError:
            logger.warning("LiDAR not available (CARLA not installed)")
            return False

        actor = self._client_vehicles.get(session_id)
        if actor is None:
            logger.warning("enable_client_lidar: no vehicle for session %s", session_id)
            return False
        if session_id in self._client_lidar_collectors:
            return True

        collector = LiDARDataCollector(
            self.world,
            downsample_factor=2,
            channels="32",
            points_per_second="500000",
        )
        try:
            collector.register_vehicle(0, actor)
        except Exception as exc:
            logger.error(
                "LiDAR sensor attach failed for session %s: %s", session_id, exc
            )
            return False

        self._client_lidar_collectors[session_id] = collector
        logger.info("LiDAR enabled for session %s (actor=%d)", session_id, actor.id)
        return True

    def get_client_lidar_binary(self, session_id: str) -> Optional[bytes]:
        """Return the latest binary point-cloud frame for the session's LiDAR."""
        collector = self._client_lidar_collectors.get(session_id)
        if collector is None or not collector.is_active():
            return None
        return collector.get_combined_pointcloud_binary()

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
            return random.choice(free)
        return random.choice(self.spawn_points)

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
        with self._lock:
            # Stop LiDAR sensor callbacks before batch-destroying vehicles
            for sid, collector in list(self._client_lidar_collectors.items()):
                try:
                    collector.cleanup()
                except Exception as exc:
                    logger.warning("LiDAR cleanup for session %s: %s", sid, exc)
            self._client_lidar_collectors.clear()

            all_actors = (
                list(self._client_vehicles.values())
                + self._background_vehicles
                + self._pedestrian_controllers
                + self._pedestrians
            )
            if all_actors and self.client:
                batch = [carla.command.DestroyActor(a) for a in all_actors]
                try:
                    self.client.apply_batch_sync(batch, True)
                except Exception as exc:
                    logger.warning("Cleanup error: %s", exc)
            self._client_vehicles.clear()
            self._background_vehicles.clear()
            self._pedestrian_controllers.clear()
            self._pedestrians.clear()
