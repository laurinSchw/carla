"""Operations behind ``/api/lidar/*`` and ``/api/sessions/{id}/lidar`` (REFACTOR_HANDOFF 5.5, roles of 5.7.1).

A mixin of ``PersistentCARLAServer``. Who may do what:

* *admin*: a sensor on any vehicle of this world, pin it, mark it for raw recording, reconfigure/release any sensor;
* *vehicle* (session token): only on its own vehicle, and only its own sensors;
* *observer*: list and view sensors (``observer_public``), never create or change one.

Methods named ``lidar_may_view`` read published data and are safe from any thread; everything else runs on the
simulation thread (the routes use ``on_sim_thread``). The ownership key of a vehicle's sensor is its session id (as for
the ``/client/lidar`` routes), the published ``owner`` is the client name.
"""

from __future__ import annotations

from typing import Optional

from src.lidar.config import LidarConfig
from src.lidar.manager import LidarError, LidarPermissionError, State
from src.server.roles import Principal

ADMIN_OWNER = "admin"


class LidarActorNotFound(LidarError):
    status = 404
    code = "lidar_actor_not_found"


class LidarSensorExists(LidarError):
    """A session has one sensor; change it with PUT instead of requesting a second one."""
    status = 409
    code = "lidar_sensor_exists"


class LidarUnavailable(LidarError):
    status = 409
    code = "lidar_unavailable"


def owner_key(principal: Principal) -> str:
    return ADMIN_OWNER if principal.kind == "admin" else str(principal.session_id)


class LidarOpsMixin:
    def _lidar_mgr(self):
        if self.lidar is None:
            raise LidarUnavailable("LiDAR is not available in this world")
        return self.lidar

    def _lidar_vehicle_known(self, actor_id: int) -> bool:
        """A client or background vehicle of this world (sensors on other people's actors are not ours to add)."""
        return (any(a.id == actor_id for a in self._client_vehicles.values())
                or any(a.id == actor_id for a in self._background_vehicles))

    def lidar_request(self, principal: Principal, actor_id: Optional[int], config: Optional[LidarConfig] = None,
                      record_raw: bool = False, pinned: bool = False) -> dict:
        """Create a sensor (state ``requested``; it spawns at the next tick boundary). Returns its info."""
        self._require_sim_thread()
        mgr = self._lidar_mgr()
        if principal.kind == "observer":
            raise LidarPermissionError("observers cannot create LiDAR sensors")
        if record_raw and principal.kind != "admin":
            raise LidarPermissionError("only an admin may mark a sensor for raw recording")
        own_actor: Optional[int] = None
        label: Optional[str] = None
        if principal.kind == "vehicle":
            actor = self._client_vehicles.get(str(principal.session_id))
            if actor is None:
                raise LidarActorNotFound("this session has no vehicle")
            own_actor = actor.id
            label = principal.label
            if actor_id is None:
                actor_id = own_actor
            if mgr.sensor_of_owner(owner_key(principal)) is not None:
                existing = mgr.sensor_of_owner(owner_key(principal))
                raise LidarSensorExists(f"this session already has sensor {existing['sensor_id']}; reconfigure it with PUT",
                                        sensor_id=existing["sensor_id"])
        if actor_id is None:
            raise LidarActorNotFound("actor_id is required")
        if principal.kind == "admin" and not self._lidar_vehicle_known(actor_id):
            raise LidarActorNotFound(f"no vehicle with actor id {actor_id} in this world", actor_id=actor_id)
        sensor = mgr.request(owner_key(principal), actor_id, config, is_admin=principal.kind == "admin",
                             own_actor_id=own_actor, record_raw=record_raw, pinned=pinned, label=label)
        return sensor.info(mgr.dt)

    def lidar_reconfigure(self, principal: Principal, sensor_id: int, config: LidarConfig) -> dict:
        """Destroy and respawn with ``config`` at the next tick boundary; the same settings again change nothing."""
        self._require_sim_thread()
        mgr = self._lidar_mgr()
        if principal.kind == "observer":
            raise LidarPermissionError("observers cannot change LiDAR sensors")
        info = mgr.get_info(sensor_id)
        if (info is not None and info["state"] in (State.ACTIVE.value, State.WARMING_UP.value)
                and config.resolve(mgr.dt).model_dump() == info["config"]
                and (principal.kind == "admin" or mgr.owner_of(sensor_id) == owner_key(principal))):
            return info          # a double click or an unchanged form must not cost a respawn
        sensor = mgr.reconfigure(sensor_id, config, requester=owner_key(principal), is_admin=principal.kind == "admin")
        return sensor.info(mgr.dt)

    def lidar_set_flags(self, principal: Principal, sensor_id: int, pinned: Optional[bool],
                        record_raw: Optional[bool]) -> dict:
        self._require_sim_thread()
        mgr = self._lidar_mgr()
        if principal.kind != "admin":
            raise LidarPermissionError("only an admin may pin a sensor or mark it for raw recording")
        return mgr.set_flags(sensor_id, pinned=pinned, record_raw=record_raw).info(mgr.dt)

    def lidar_release(self, principal: Principal, sensor_id: int) -> None:
        self._require_sim_thread()
        mgr = self._lidar_mgr()
        if principal.kind == "observer":
            raise LidarPermissionError("observers cannot release LiDAR sensors")
        mgr.release(sensor_id, requester=owner_key(principal), is_admin=principal.kind == "admin")

    def lidar_may_view(self, principal: Principal, info: dict) -> bool:
        """Viewing: the admin always, anyone while ``observer_public``, a vehicle otherwise only sensors on its own actor."""
        if principal.kind == "admin" or self.observer_public:
            return True
        return principal.kind == "vehicle" and principal.actor_id is not None and info.get("actor_id") == principal.actor_id
