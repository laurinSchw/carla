"""LiDAR sensors on request: lifecycle state machine, budget, ownership, frame-exact collection.

A *sensor* is a CARLA actor attached to a vehicle (costly, explicitly requested); a *viewer* is a stream
consumer. Many viewers share one sensor. Everything that touches CARLA happens in ``on_tick`` (and the
``request``/``reconfigure``/``release`` calls), which the owner calls from the simulation thread at a tick
boundary. Readers on other threads use only the published snapshots (``list_sensors``, ``get_stream_frame``).

States (handoff §5.5)::

    requested -> spawning -> warming_up -> active
    active -> reconfiguring -> warming_up -> active
    active|warming_up -> stopping -> destroyed        (release, owner gone, auto-release)
    any -> error {reason}

Every transition is an event stamped with the frame it happened at (``events``).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Protocol, Tuple

import numpy as np

from src.lidar import protocol
from src.lidar.config import POINT_BUCKETS, LidarConfig, LidarLimits

logger = logging.getLogger(__name__)


class State(str, Enum):
    REQUESTED = "requested"
    SPAWNING = "spawning"
    WARMING_UP = "warming_up"
    ACTIVE = "active"
    RECONFIGURING = "reconfiguring"
    STOPPING = "stopping"
    DESTROYED = "destroyed"
    ERROR = "error"


_COUNTED = (State.REQUESTED, State.SPAWNING, State.WARMING_UP, State.ACTIVE, State.RECONFIGURING)


class LidarError(Exception):
    """Base class; ``detail`` is a JSON-friendly dict for API error bodies."""

    status = 400

    def __init__(self, message: str, **detail):
        super().__init__(message)
        self.detail = {"error": self.code, "message": message, **detail}

    code = "lidar_error"


class LidarBudgetExceeded(LidarError):
    status = 409
    code = "lidar_budget_exceeded"


class LidarPermissionError(LidarError):
    status = 403
    code = "lidar_forbidden"


class LidarBusy(LidarError):
    """The sensor is changing state (requested/spawning/reconfiguring/stopping): try again after the next tick."""
    status = 409
    code = "lidar_busy"


class LidarInvalidConfig(LidarError):
    status = 422
    code = "lidar_invalid_config"


class LidarNotFound(LidarError):
    status = 404
    code = "lidar_sensor_not_found"


# --- backend protocol -------------------------------------------------------------------------

@dataclass
class Measurement:
    """One LiDAR measurement as the backend delivers it: points in the SENSOR frame and the sensor's world pose."""
    frame: int
    points: np.ndarray                     # structured, fields per ``field_mask``
    field_mask: int
    location: Tuple[float, float, float]
    rotation: Tuple[float, float, float]   # pitch, yaw, roll (deg)
    sim_time: float = 0.0


class LidarHandle(Protocol):
    def get_measurement(self, frame: int, timeout_s: float) -> Optional[Measurement]:
        """Block up to ``timeout_s`` for the measurement of exactly ``frame``; older ones are discarded."""

    def destroy(self) -> None: ...


class LidarBackend(Protocol):
    def spawn(self, actor_id: int, config: LidarConfig) -> LidarHandle:
        """Create and start a sensor attached to ``actor_id``. Raises on failure."""


# --- stream profiles --------------------------------------------------------------------------

MAX_STREAM_DIVISOR = 20  # stream rates are divisors 1..20 of the simulation rate (20 FPS: down to 1 Hz)


def _index_scores(seed: int, sensor_id: int, start: int, stop: int) -> np.ndarray:
    """Deterministic uint64 score per point index in [start, stop) (splitmix64): independent of how many points a frame has."""
    mask = 0xFFFFFFFFFFFFFFFF
    salt = (seed * 0x9E3779B97F4A7C15 + sensor_id * 0xBF58476D1CE4E5B9 + 0x94D049BB133111EB) & mask
    with np.errstate(over="ignore"):
        x = np.arange(start, stop, dtype=np.uint64) * np.uint64(0x9E3779B97F4A7C15) + np.uint64(salt)
        x = (x ^ (x >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        x = (x ^ (x >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        x = x ^ (x >> np.uint64(31))
    return x


@dataclass(frozen=True)
class StreamProfile:
    """Display-only stream settings, quantised so each frame is encoded once per profile and shared."""
    max_points: Optional[int] = None  # None = all points
    every_n_frames: int = 1
    compress: bool = False

    @staticmethod
    def quantise(max_points: Optional[int], max_hz: Optional[float], compress: bool, sim_hz: float) -> "StreamProfile":
        if max_points is not None:
            bigger = [b for b in POINT_BUCKETS if b >= max_points]
            max_points = bigger[0] if bigger else None
        every = 1
        if max_hz is not None and max_hz > 0:
            every = max(1, int(np.ceil(sim_hz / max_hz - 1e-9)))  # largest rate that divides sim_hz and is <= max_hz
        return StreamProfile(max_points, every, compress)


# --- sensor record ----------------------------------------------------------------------------

@dataclass
class Sensor:
    sensor_id: int
    actor_id: int
    owner: str
    config: LidarConfig
    record_raw: bool = False
    pinned: bool = False
    state: State = State.REQUESTED
    error: Optional[str] = None
    created_frame: int = 0
    last_frame: Optional[int] = None
    missed_frames: int = 0
    consecutive_misses: int = 0
    viewers: int = 0
    pending_config: Optional[LidarConfig] = None
    handle: Optional[LidarHandle] = None
    no_viewers_since: Optional[float] = None
    #: shown as ``owner`` in the published info (e.g. the client name); ``owner`` itself is the ownership key (a session
    #: id) and is never published: it is what the unauthenticated /client routes act on
    label: Optional[str] = None

    def info(self, dt: float) -> dict:
        return {
            "sensor_id": self.sensor_id, "actor_id": self.actor_id, "owner": self.label or self.owner,
            "state": self.state.value, "error": self.error, "config": self.config.model_dump(),
            "viewers": self.viewers, "pinned": self.pinned, "record_raw": self.record_raw,
            "points_per_frame": self.config.points_per_frame(dt),
            "created_frame": self.created_frame, "last_frame": self.last_frame,
            "missed_frames": self.missed_frames,
        }


@dataclass(frozen=True)
class LidarFrame:
    frame: int
    sim_time: float
    wall_time: float
    sensor_id: int
    ego_actor_id: int
    measurement: Measurement


class LidarManager:
    def __init__(
        self,
        backend: LidarBackend,
        limits: LidarLimits,
        fixed_delta_seconds: float,
        *,
        seed: int = 42,
        wall_clock: Callable[[], float] = time.time,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._backend = backend
        self.limits = limits
        self.dt = fixed_delta_seconds
        self._seed = seed
        self._wall = wall_clock
        self._mono = monotonic

        self._sensors: Dict[int, Sensor] = {}
        self._next_id = 1
        self._frame = 0
        self.events: List[dict] = []
        self._raw_listeners: List[Callable[[dict, LidarFrame], None]] = []
        self._event_listeners: List[Callable[[dict], None]] = []

        # published for other threads: replaced, never mutated
        self._published: Tuple[dict, ...] = ()
        self._published_owners: Tuple[Tuple[str, dict], ...] = ()   # (ownership key, info): not for clients
        self._published_budget: dict = {}
        self.info_version = 0  # increases with every change of the published sensor list (the hub pushes on change)
        self._latest: Dict[int, LidarFrame] = {}
        # newest frame divisible by n, per sensor and n: a viewer asking for "every n-th frame" gets the newest due frame
        # however its polling phase drifts against the simulation ticks (a pure `frame % n` test on the latest frame
        # delivered nothing at all when polls kept landing after odd ticks)
        self._due: Dict[int, Dict[int, LidarFrame]] = {}
        self.hold_all = False  # True while a run records raw LiDAR: sensors must not be auto-released
        self._cache_lock = threading.Lock()
        self._encoded: Dict[Tuple[int, int, StreamProfile], bytes] = {}
        self._hash_cache: Dict[int, np.ndarray] = {}  # per sensor: seeded per-index scores for subsampling

    # -- requests (simulation thread) ------------------------------------------------------------
    def request(
        self,
        owner: str,
        actor_id: int,
        config: Optional[LidarConfig] = None,
        *,
        is_admin: bool = False,
        own_actor_id: Optional[int] = None,
        record_raw: bool = False,
        pinned: bool = False,
        label: Optional[str] = None,
    ) -> Sensor:
        """Ask for a sensor on ``actor_id``. Vehicle sessions may only use their own actor; ``pinned`` is admin-only
        (``record_raw`` is checked by the caller). ``label`` is what the published info shows as the owner (default: ``owner``)."""
        if not is_admin and actor_id != own_actor_id:
            raise LidarPermissionError("a vehicle may only attach a LiDAR to its own actor", actor_id=actor_id)
        if pinned and not is_admin:
            raise LidarPermissionError("only an admin may pin a sensor")
        cfg = self._resolve(config or LidarConfig())
        self._check_budget(cfg, replacing=None)
        sensor = Sensor(self._next_id, actor_id, owner, cfg, record_raw, pinned, created_frame=self._frame, label=label)
        self._next_id += 1
        self._sensors[sensor.sensor_id] = sensor
        self._event(sensor, "new sensor")
        self._publish()
        return sensor

    def reconfigure(self, sensor_id: int, config: LidarConfig, *, requester: str, is_admin: bool = False) -> Sensor:
        sensor = self._get(sensor_id)
        self._check_owner(sensor, requester, is_admin)
        if sensor.state not in (State.ACTIVE, State.WARMING_UP, State.ERROR):
            raise LidarBusy(f"sensor is {sensor.state.value}, try again after the next tick", state=sensor.state.value)
        cfg = self._resolve(config)
        self._check_budget(cfg, replacing=sensor)
        sensor.pending_config = cfg
        sensor.error = None
        self._set_state(sensor, State.RECONFIGURING, "config change")
        self._publish()
        return sensor

    def release(self, sensor_id: int, *, requester: str, is_admin: bool = False) -> None:
        sensor = self._get(sensor_id)
        self._check_owner(sensor, requester, is_admin)
        self._begin_stop(sensor, "released by request")

    def set_flags(self, sensor_id: int, *, pinned: Optional[bool] = None, record_raw: Optional[bool] = None) -> Sensor:
        """Pin a sensor (no auto-release) or mark it for raw recording. Admin-only: the caller checks the role."""
        sensor = self._get(sensor_id)
        if sensor.state in (State.STOPPING, State.DESTROYED):
            raise LidarBusy(f"sensor is {sensor.state.value}", state=sensor.state.value)
        changed = []
        if pinned is not None and pinned != sensor.pinned:
            sensor.pinned = pinned
            changed.append(f"pinned={pinned}")
        if record_raw is not None and record_raw != sensor.record_raw:
            sensor.record_raw = record_raw
            changed.append(f"record_raw={record_raw}")
        if changed:
            if not (sensor.pinned or sensor.record_raw) and sensor.viewers == 0:
                sensor.no_viewers_since = self._mono()   # the keep-alive starts now, not at the last viewer of long ago
            self._event(sensor, "flags " + ", ".join(changed))
            self._publish()
        return sensor

    def release_owner(self, owner: str, reason: str = "owner session ended") -> int:
        n = 0
        for sensor in list(self._sensors.values()):
            if sensor.owner == owner and sensor.state not in (State.STOPPING, State.DESTROYED):
                self._begin_stop(sensor, reason)
                n += 1
        return n

    def destroy_owner_now(self, owner: str, reason: str = "owner session ended") -> int:
        """Destroy the owner's sensors immediately instead of at the next tick boundary.

        Needed when the attached vehicle is about to be destroyed: stop the sensor first, then its parent.
        """
        return sum(self.destroy_sensor_now(s.sensor_id, reason) for s in list(self._sensors.values()) if s.owner == owner)

    def destroy_actor_now(self, actor_id: int, reason: str = "vehicle despawned") -> int:
        """Destroy every sensor attached to ``actor_id`` now (whoever owns it): call before destroying the actor."""
        return sum(self.destroy_sensor_now(s.sensor_id, reason) for s in list(self._sensors.values()) if s.actor_id == actor_id)

    def destroy_sensor_now(self, sensor_id: int, reason: str) -> int:
        sensor = self._sensors.get(sensor_id)
        if sensor is None or sensor.state is State.DESTROYED:
            return 0
        self._destroy_handle(sensor)
        self._set_state(sensor, State.DESTROYED, reason)
        del self._sensors[sensor_id]
        self._latest = {k: v for k, v in self._latest.items() if k != sensor_id}
        self._due = {k: v for k, v in self._due.items() if k != sensor_id}
        self._hash_cache.pop(sensor_id, None)
        self._publish()
        return 1

    def release_actor(self, actor_id: int, reason: str = "vehicle gone") -> int:
        n = 0
        for sensor in list(self._sensors.values()):
            if sensor.actor_id == actor_id and sensor.state not in (State.STOPPING, State.DESTROYED):
                self._begin_stop(sensor, reason)
                n += 1
        return n

    def shutdown(self) -> None:
        """Destroy every sensor now (server stop)."""
        for sensor in list(self._sensors.values()):
            self._destroy_handle(sensor)
            self._set_state(sensor, State.DESTROYED, "shutdown")
        self._sensors.clear()
        self._latest = {}
        self._due = {}
        self._hash_cache.clear()
        self._publish()

    # -- viewers (any thread, only touches counters) ---------------------------------------------
    def viewer_attached(self, sensor_id: int) -> None:
        s = self._get(sensor_id)
        s.viewers += 1
        s.no_viewers_since = None

    def viewer_detached(self, sensor_id: int) -> None:
        s = self._sensors.get(sensor_id)
        if s is not None and s.viewers > 0:
            s.viewers -= 1
            if s.viewers == 0:
                s.no_viewers_since = self._mono()

    # -- tick boundary (simulation thread) -------------------------------------------------------
    def on_tick(self, frame: int, sim_time: float) -> None:
        """Advance every sensor after ``world.tick()`` returned ``frame``; collect that frame's measurements."""
        self._frame = frame
        for sensor in list(self._sensors.values()):
            try:
                self._advance(sensor, frame, sim_time)
            except Exception as exc:  # one broken sensor must not stop the simulation
                logger.exception("LiDAR sensor %d failed", sensor.sensor_id)
                self._fail(sensor, f"{type(exc).__name__}: {exc}")
        for sid in [s.sensor_id for s in self._sensors.values() if s.state is State.DESTROYED]:
            del self._sensors[sid]
            self._latest = {k: v for k, v in self._latest.items() if k != sid}
            self._due = {k: v for k, v in self._due.items() if k != sid}
            self._hash_cache.pop(sid, None)
        self._publish()

    def _advance(self, s: Sensor, frame: int, sim_time: float) -> None:
        st = s.state
        if st is State.REQUESTED:
            self._set_state(s, State.SPAWNING, "tick boundary")
            self._spawn(s)
        elif st is State.RECONFIGURING:
            self._destroy_handle(s)
            s.config, s.pending_config = s.pending_config, None
            self._spawn(s)
        elif st is State.STOPPING:
            self._destroy_handle(s)
            self._set_state(s, State.DESTROYED, s.error or "stopped")
        elif st in (State.WARMING_UP, State.ACTIVE):
            if self._auto_release_due(s):
                self._begin_stop(s, f"no viewers for {self.limits.keep_alive_s:g} s")
                return
            self._collect(s, frame, sim_time)

    def _spawn(self, s: Sensor) -> None:
        try:
            s.handle = self._backend.spawn(s.actor_id, s.config)
        except Exception as exc:
            self._fail(s, f"spawn failed: {exc}")
            return
        s.no_viewers_since = self._mono() if s.viewers == 0 else None
        self._set_state(s, State.WARMING_UP, "spawned")

    def _collect(self, s: Sensor, frame: int, sim_time: float) -> None:
        m = s.handle.get_measurement(frame, self.limits.frame_timeout_s) if s.handle else None
        if m is None:
            s.missed_frames += 1
            s.consecutive_misses += 1
            if s.consecutive_misses >= self.limits.max_consecutive_misses:
                # a sensor that stopped delivering (parent destroyed, server hiccup) would otherwise block every tick for
                # frame_timeout_s forever; give it up, free its handle and its budget
                self._fail(s, f"no measurement for {s.consecutive_misses} consecutive frames")
            return
        s.consecutive_misses = 0
        if s.state is State.WARMING_UP:
            self._set_state(s, State.ACTIVE, f"first measurement (frame {m.frame})")
        s.last_frame = m.frame
        lf = LidarFrame(m.frame, sim_time, self._wall(), s.sensor_id, s.actor_id, m)
        self._latest = {**self._latest, s.sensor_id: lf}
        due = dict(self._due.get(s.sensor_id, {}))
        for n in range(1, MAX_STREAM_DIVISOR + 1):
            if lf.frame % n == 0:
                due[n] = lf
        self._due = {**self._due, s.sensor_id: due}
        if self._raw_listeners:
            info = s.info(self.dt)
            for cb in self._raw_listeners:
                try:
                    cb(info, lf)
                except Exception:
                    logger.exception("LiDAR raw listener failed")

    def _auto_release_due(self, s: Sensor) -> bool:
        if s.pinned or s.record_raw or self.hold_all or s.viewers > 0 or s.no_viewers_since is None:
            return False
        return (self._mono() - s.no_viewers_since) >= self.limits.keep_alive_s

    # -- reading (any thread) --------------------------------------------------------------------
    def list_sensors(self) -> List[dict]:
        return list(self._published)

    def get_info(self, sensor_id: int) -> Optional[dict]:
        return next((i for i in self._published if i["sensor_id"] == sensor_id), None)

    def sensor_of_owner(self, owner: str) -> Optional[dict]:
        """The owner's live sensor (one that is stopping does not count: a new request replaces it)."""
        return next((i for o, i in self._published_owners
                     if o == owner and i["state"] not in (State.STOPPING.value, State.DESTROYED.value)), None)

    def owner_of(self, sensor_id: int) -> Optional[str]:
        """Ownership key of a published sensor (a session id or "admin"); for permission checks, never for clients."""
        return next((o for o, i in self._published_owners if i["sensor_id"] == sensor_id), None)

    def budget(self) -> dict:
        """Budget as published after the last change (safe from any thread); the simulation thread uses ``_budget_now``."""
        return dict(self._published_budget) or self._budget_now()

    def _budget_now(self) -> dict:
        # a sensor being reconfigured is counted at its NEW size: it is about to hold that much
        used = sum((s.pending_config or s.config).points_per_second for s in list(self._sensors.values()) if s.state in _COUNTED)
        return {
            "used_pps": used, "max_pps": self.limits.max_total_points_per_second,
            "sensors": sum(1 for s in list(self._sensors.values()) if s.state in _COUNTED),
            "max_sensors": self.limits.max_sensors,
        }

    def add_event_listener(self, callback: Callable[[dict], None]) -> None:
        """Called on the simulation thread with every lifecycle event (``frame``, ``sensor_id``, ``state``, ``reason``)."""
        self._event_listeners.append(callback)

    def add_raw_listener(self, callback: Callable[[dict, LidarFrame], None]) -> None:
        """Called on the simulation thread with every full-resolution frame (recording hooks)."""
        self._raw_listeners.append(callback)

    def get_stream_frame(
        self, sensor_id: int, profile: StreamProfile = StreamProfile(), after_frame: int = -1
    ) -> Optional[Tuple[int, bytes]]:
        """Newest frame newer than ``after_frame`` that is due for ``profile``, as protocol-v3 bytes.

        "Due" means the newest frame whose number is a multiple of ``profile.every_n_frames``, so a viewer polling at an
        unlucky phase still gets every n-th frame. The bytes are produced once per (sensor, frame, profile) and shared by
        all viewers on that profile. May be called from any thread (encoding is serialised by a lock).
        """
        got = self.get_stream_frame_counted(sensor_id, profile, after_frame)
        return None if got is None else (got[0], got[1])

    def get_stream_frame_counted(
        self, sensor_id: int, profile: StreamProfile = StreamProfile(), after_frame: int = -1
    ) -> Optional[Tuple[int, bytes, int, int]]:
        """Like ``get_stream_frame`` plus ``(points_total, points_sent)``: how many points the measurement had and how many the
        encoded frame carries (fewer when the profile subsamples for display)."""
        n = max(1, profile.every_n_frames)
        lf = self._due.get(sensor_id, {}).get(n) if n <= MAX_STREAM_DIVISOR else self._latest.get(sensor_id)
        if lf is None or lf.frame <= after_frame or (n > MAX_STREAM_DIVISOR and lf.frame % n != 0):
            return None
        total = len(lf.measurement.points)
        sent = total if profile.max_points is None else min(total, profile.max_points)
        key = (sensor_id, lf.frame, profile)
        with self._cache_lock:
            data = self._encoded.get(key)
            if data is None:
                data = self._encode(lf, profile)
                # keep only this sensor's newest frames; other sensors' cached entries are not ours to evict
                self._encoded = {k: v for k, v in self._encoded.items() if k[0] != sensor_id or k[1] == lf.frame}
                self._encoded[key] = data
        return lf.frame, data, total, sent

    # -- internals -------------------------------------------------------------------------------
    def _encode(self, lf: LidarFrame, profile: StreamProfile) -> bytes:
        m = lf.measurement
        points = m.points
        if profile.max_points is not None and len(points) > profile.max_points:
            points = points[self._subsample_indices(lf.sensor_id, len(points), profile.max_points)]
        return protocol.encode(
            points, frame=lf.frame, sim_time=lf.sim_time, wall_time=lf.wall_time,
            ego_actor_id=lf.ego_actor_id, sensor_id=lf.sensor_id,
            location=m.location, rotation=m.rotation, field_mask=m.field_mask, compress=profile.compress,
        )

    def _subsample_indices(self, sensor_id: int, n: int, k: int) -> np.ndarray:
        """Seeded random selection of ``k`` of ``n`` points (a stride would show CARLA's per-channel ordering as rings).

        Every point index gets a fixed pseudo-random score that depends only on (seed, sensor, index); the ``k`` lowest
        scores among the first ``n`` are kept. Unlike drawing a permutation of size ``n``, the selection hardly changes when
        the point count varies from frame to frame (semantic LiDAR returns a different number of points each frame).
        """
        scores = self._hash_cache.get(sensor_id)
        if scores is None or len(scores) < n:
            old = 0 if scores is None else len(scores)
            new = _index_scores(self._seed, sensor_id, old, max(n, old * 2, 1024))
            scores = new if scores is None else np.concatenate([scores, new])
            self._hash_cache[sensor_id] = scores
        if k >= n:
            return np.arange(n)
        return np.sort(np.argpartition(scores[:n], k)[:k])

    def _resolve(self, config: LidarConfig) -> LidarConfig:
        try:
            return config.resolve(self.dt)
        except ValueError as exc:
            raise LidarInvalidConfig(str(exc)) from exc

    def _check_budget(self, cfg: LidarConfig, replacing: Optional[Sensor]) -> None:
        b = self._budget_now()
        used, count = b["used_pps"], b["sensors"]
        if replacing is not None and replacing.state in _COUNTED:
            used -= (replacing.pending_config or replacing.config).points_per_second
            count -= 1
        available = self.limits.max_total_points_per_second - used
        if count + 1 > self.limits.max_sensors or cfg.points_per_second > available:
            raise LidarBudgetExceeded(
                "LiDAR budget exceeded",
                reason="max_sensors" if count + 1 > self.limits.max_sensors else "points_per_second",
                requested_pps=cfg.points_per_second, available_pps=max(available, 0),
                sensors=count, max_sensors=self.limits.max_sensors,
            )

    def _check_owner(self, s: Sensor, requester: str, is_admin: bool) -> None:
        if not is_admin and s.owner != requester:
            raise LidarPermissionError("sensor belongs to another session", sensor_id=s.sensor_id)

    def _get(self, sensor_id: int) -> Sensor:
        s = self._sensors.get(sensor_id)
        if s is None:
            raise LidarNotFound(f"no LiDAR sensor {sensor_id}", sensor_id=sensor_id)
        return s

    def _begin_stop(self, s: Sensor, reason: str) -> None:
        if s.state in (State.STOPPING, State.DESTROYED):
            return
        s.error = None if s.state is not State.ERROR else s.error
        self._set_state(s, State.STOPPING, reason)
        self._publish()

    def _fail(self, s: Sensor, reason: str) -> None:
        self._destroy_handle(s)
        s.error = reason
        s.pending_config = None
        self._set_state(s, State.ERROR, reason)

    def _destroy_handle(self, s: Sensor) -> None:
        h, s.handle = s.handle, None
        if h is not None:
            try:
                h.destroy()
            except Exception as exc:
                logger.warning("destroying LiDAR sensor %d: %s", s.sensor_id, exc)

    def _set_state(self, s: Sensor, state: State, reason: str) -> None:
        s.state = state
        self._event(s, reason)

    def _event(self, s: Sensor, reason: str) -> None:
        ev = {"frame": self._frame, "sensor_id": s.sensor_id, "actor_id": s.actor_id,
              "state": s.state.value, "reason": reason}
        self.events.append(ev)
        for cb in self._event_listeners:
            try:
                cb(ev)
            except Exception:
                logger.exception("LiDAR event listener failed")
        if len(self.events) > 2000:
            del self.events[:1000]

    def _publish(self) -> None:
        owned = tuple((s.owner, s.info(self.dt)) for s in list(self._sensors.values()))
        self._published_owners = owned
        self._published = tuple(i for _, i in owned)
        self._published_budget = self._budget_now()
        self.info_version += 1
