"""What the WebSocket hub reads from the world (REFACTOR_HANDOFF 5.7.3), as a mixin of ``PersistentCARLAServer``.

Everything here is built on the SIMULATION thread and published by replacing one reference (``Published`` snapshots, the immutable
``V2XFrame``, the event log), so the hub reads it from the event loop without locks. Nothing here changes CARLA.
"""

from __future__ import annotations

import collections
import logging
import time
from typing import Any, Dict, Optional

from src.server.events import EventLog
from src.server.published import Published
from src.server.v2x_publisher import V2XPublisher
from src.v2x.frame import build_v2x_frame

logger = logging.getLogger(__name__)

_RUN_STATUS_EVERY_N_FRAMES = 20   # about 1 Hz at 20 FPS


class HubViewsMixin:
    def _init_hub_views(self, config) -> None:
        self.events = EventLog(config.event_log_size)
        self.v2x_links_max_rows = config.v2x_links_max_rows
        self.session_disconnect_grace_s = config.session_disconnect_grace_s
        self._v2x_pub = V2XPublisher()
        self._world_state: Optional[Published] = None
        self._world_state_version = 0
        self._run_status: Optional[Published] = None
        self._tick_wall: collections.deque = collections.deque(maxlen=200)

    # -- events ---------------------------------------------------------------------------------
    def _event(self, kind: str, **data: Any) -> None:
        self.events.add(kind, frame=self._frame, sim_time=self._sim_time, **data)

    # -- published after each V2X update ----------------------------------------------------------
    def _publish_v2x_frame(self) -> None:
        net = self.v2x_network
        if net is None:
            return
        try:
            self._v2x_pub.on_update(build_v2x_frame(net, self._frame, self._sim_time))
        except Exception:
            logger.exception("Building the V2X frame failed at frame %d (simulation continues)", self._frame)

    # -- published every few frames ----------------------------------------------------------------
    def _publish_hub_views(self, snapshot, tick_start: float) -> None:
        """Ground-truth world state (every ``world_state_every_n_frames``) and run status (about 1 Hz). Failures are logged and
        never stop the simulation."""
        self._tick_wall.append(tick_start)
        try:
            if self._frame % self.config.world_state_every_n_frames == 0:
                self._publish_world_state(snapshot)
            if self._frame % _RUN_STATUS_EVERY_N_FRAMES == 0:
                self._publish_run_status(snapshot)
        except Exception:
            logger.exception("Publishing hub views failed at frame %d (simulation continues)", self._frame)

    @staticmethod
    def _station_entry(actor, snap, kind: str, role: str, name: Optional[str], v2x: bool) -> Optional[dict]:
        if snap is None:
            return None
        t, v = snap.get_transform(), snap.get_velocity()
        return {"id": actor.id, "kind": kind, "role": role, "name": name, "v2x": v2x,
                "x": round(t.location.x, 2), "y": round(t.location.y, 2), "z": round(t.location.z, 2),
                "yaw": round(t.rotation.yaw, 1), "speed_mps": round((v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5, 2)}

    def _publish_world_state(self, snapshot) -> None:
        net = self.v2x_network
        equipped = set(net.vehicles) if net is not None else set()
        vrus = set(net._vrus) if net is not None else set()
        names = {s.session_id: s.client_name for s in self.sessions.get_active_sessions()}
        stations = []
        for sid, actor in self._client_vehicles.items():
            e = self._station_entry(actor, snapshot.find(actor.id), "vehicle", "client", names.get(sid), True)
            if e:
                stations.append(e)
        for actor in self._background_vehicles:
            e = self._station_entry(actor, snapshot.find(actor.id), "vehicle", "background", None, actor.id in equipped)
            if e:
                stations.append(e)
        for actor in self._pedestrians:
            e = self._station_entry(actor, snapshot.find(actor.id), "pedestrian", "background", None, actor.id in vrus)
            if e:
                stations.append(e)
        rsus = []
        if net is not None:
            for iid, rsu in net.get_rsus().items():
                x, y, z = rsu.config.position
                rsus.append({"station_id": -int(iid), "intersection_id": iid, "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
                             "enabled": True, "tx_power_dbm": rsu.config.tx_power_dbm,
                             "broadcast_range_m": rsu.config.broadcast_range_m})
            rsus.sort(key=lambda r: r["intersection_id"])
        self._world_state_version += 1
        self._world_state = Published(self._world_state_version, self._frame, self._sim_time,
                                      {"map": self._map_name, "stations": stations, "rsus": rsus})

    def _publish_run_status(self, snapshot) -> None:
        walls = list(self._tick_wall)
        rtf = None
        if len(walls) >= 2 and walls[-1] > walls[0]:
            rtf = round((len(walls) - 1) * self.fixed_delta_seconds / (walls[-1] - walls[0]), 3)
        timing = self.get_timing()
        rec = self.recorder
        self._run_status = Published(self._frame // _RUN_STATUS_EVERY_N_FRAMES, self._frame, self._sim_time, {
            "running": self._running, "frame": self._frame, "sim_time": self._sim_time, "map": self._map_name,
            "fixed_delta_seconds": self.fixed_delta_seconds, "real_time_factor": rtf,
            "tick_duration_ms": timing.get("tick"), "v2x_update_ms": timing.get("v2x_update"),
            "sessions": len(self.sessions.get_active_sessions()),
            "background_vehicles": len(self._background_vehicles), "pedestrians": len(self._pedestrians),
            "lidar_budget": self.lidar.budget() if self.lidar else None,
            "recording": rec.summary() if rec else None,
        })

    # -- getters the topics call (any thread) ---------------------------------------------------------
    def get_world_state(self) -> Optional[Published]:
        return self._world_state

    def get_v2x_published(self) -> Optional[Published]:
        return self._v2x_pub.summary

    def get_v2x_frame(self):
        return self._v2x_pub.frame

    def get_v2x_stats(self) -> Optional[Published]:
        return self._v2x_pub.stats

    def get_run_status_published(self) -> Optional[Published]:
        return self._run_status

    # -- sessions whose WebSocket closed (decision D4) --------------------------------------------------
    def _remove_disconnected_sessions(self) -> None:
        """Called with the periodic session check on the simulation thread."""
        self.sessions.prune_ended()
        grace = self.session_disconnect_grace_s
        if grace <= 0:
            return
        for session in self.sessions.get_grace_expired(grace):
            logger.info("Session %s: no connection for %.0f s, removing its vehicle", session.session_id, grace)
            self._despawn_client_vehicle(session.session_id)
            self.sessions.end_session(session.session_id)
            self._event("session_disconnected_removed", client=session.client_name, actor_id=session.vehicle_actor_id, grace_s=grace)
