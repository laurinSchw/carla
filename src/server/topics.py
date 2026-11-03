"""The WebSocket hub's topics: who may subscribe, which filter is required, and how each one is produced.

A provider only READS data the simulation thread published (``Published`` snapshots, the immutable
``V2XFrame``, the event log, the LiDAR manager's snapshots); it never calls CARLA and never blocks.

Ground truth (all stations, unfiltered links, run internals) is for observer and admin. A vehicle gets its own
telemetry, its own inbox (what it actually received) and the LiDAR sensor list (decisions D9, D10).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet, List, Optional

from src.server.roles import Principal
from src.server.published import Published, link_to_dict, links_to_columns, message_to_dict, to_jsonable

ADMIN_R, OBSERVER_R, VEHICLE_R = "admin", "observer", "vehicle"


class FilterError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code, self.message = code, message


@dataclass
class ProviderContext:
    source: Any                       # the world (or a fake): see HubSource below
    principal: Principal
    filter: Dict[str, Any]
    state: Dict[str, Any]             # per subscription, e.g. the cursor of the events topic
    cache: Dict[Any, Any]             # shared by all connections of one hub; cleared when the frame changes


@dataclass(frozen=True)
class TopicSpec:
    name: str
    roles: FrozenSet[str]
    max_hz: float                     # a client may ask for less, never for more
    default_hz: float
    provide: Callable[[ProviderContext], Optional[Published]]
    description: str
    filter_required: bool = False
    validate_filter: Optional[Callable[[Dict[str, Any], Principal], Dict[str, Any]]] = None
    ground_truth: bool = False


# ── filters ───────────────────────────────────────────────────────────────────────────────────────

LINK_TYPES = ("V2V", "V2I", "I2V", "V2P", "P2V")
MSG_TYPES = ("BSM", "SPAT", "MAP", "VRUAM", "DENM", "CPM")
OUTCOMES = ("delivered", "dropped", "all")
ROLES_IN_LINK = ("any", "tx", "rx")


def _int_list(v: Any, key: str, limit: int = 500) -> List[int]:
    if not isinstance(v, list) or not all(isinstance(i, int) and not isinstance(i, bool) for i in v):
        raise FilterError("bad_filter", f"{key} must be a list of integers")
    if len(v) > limit:
        raise FilterError("bad_filter", f"{key} may have at most {limit} entries")
    return v


def _choice_list(v: Any, key: str, allowed) -> List[str]:
    if not isinstance(v, list) or not all(isinstance(i, str) and i in allowed for i in v):
        raise FilterError("bad_filter", f"{key} must be a list of {', '.join(allowed)}")
    return v


def validate_links_filter(f: Dict[str, Any], principal: Principal) -> Dict[str, Any]:
    unknown = set(f) - {"station_ids", "role", "msg_types", "link_types", "outcome"}
    if unknown:
        raise FilterError("bad_filter", f"unknown filter keys: {', '.join(sorted(unknown))}")
    out: Dict[str, Any] = {}
    if "station_ids" in f:
        out["station_ids"] = _int_list(f["station_ids"], "station_ids")
    if "msg_types" in f:
        out["msg_types"] = _choice_list(f["msg_types"], "msg_types", MSG_TYPES)
    if "link_types" in f:
        out["link_types"] = _choice_list(f["link_types"], "link_types", LINK_TYPES)
    if "outcome" in f:
        if f["outcome"] not in OUTCOMES:
            raise FilterError("bad_filter", f"outcome must be one of {', '.join(OUTCOMES)}")
        out["outcome"] = f["outcome"]
    if "role" in f:
        if f["role"] not in ROLES_IN_LINK:
            raise FilterError("bad_filter", f"role must be one of {', '.join(ROLES_IN_LINK)}")
        out["role"] = f["role"]
    if not any(k in out for k in ("station_ids", "msg_types", "link_types", "outcome")):
        raise FilterError("filter_required", "v2x.links needs a filter with at least one of station_ids, "
                                             "msg_types, link_types, outcome")
    return out


def validate_inbox_filter(f: Dict[str, Any], principal: Principal) -> Dict[str, Any]:
    unknown = set(f) - {"station_id"}
    if unknown:
        raise FilterError("bad_filter", f"unknown filter keys: {', '.join(sorted(unknown))}")
    sid = f.get("station_id")
    if principal.kind == "vehicle":
        if sid is None:
            sid = principal.actor_id
        if sid != principal.actor_id:
            raise FilterError("forbidden", "a vehicle may only read its own inbox")
    if not isinstance(sid, int) or isinstance(sid, bool):
        raise FilterError("filter_required", "v2x.inbox needs {\"station_id\": <int>}")
    return {"station_id": sid}


def validate_world_state_filter(f: Dict[str, Any], principal: Principal) -> Dict[str, Any]:
    unknown = set(f) - {"kinds"}
    if unknown:
        raise FilterError("bad_filter", f"unknown filter keys: {', '.join(sorted(unknown))}")
    if "kinds" in f:
        return {"kinds": _choice_list(f["kinds"], "kinds", ("vehicle", "pedestrian"))}
    return {}


def validate_no_filter(f: Dict[str, Any], principal: Principal) -> Dict[str, Any]:
    if f:
        raise FilterError("bad_filter", "this topic takes no filter")
    return {}


# ── providers ─────────────────────────────────────────────────────────────────────────────────────

def _world_state(ctx: ProviderContext) -> Optional[Published]:
    pub = ctx.source.get_world_state()
    if pub is None:
        return None
    kinds = ctx.filter.get("kinds")
    if kinds is None:
        return pub
    key = ("world.state", pub.version, tuple(sorted(kinds)))
    hit = ctx.cache.get(key)
    if hit is None:
        data = dict(pub.data)
        data["stations"] = [s for s in pub.data["stations"] if s["kind"] in kinds]
        hit = ctx.cache[key] = Published(pub.version, pub.frame, pub.sim_time, data)
    return hit


def _session_self(ctx: ProviderContext) -> Optional[Published]:
    p = ctx.principal
    tel = ctx.source.get_vehicle_telemetry(p.session_id) if p.session_id else None
    if tel is None:
        return None
    session = ctx.source.sessions.get_session(p.session_id)
    ctx.source.sessions.touch(p.session_id)
    data = {"telemetry": tel, "remaining_seconds": session.remaining_seconds if session else None,
            "client_name": session.client_name if session else None, "actor_id": p.actor_id,
            "station_id": p.actor_id}
    return Published(int(tel["frame"]), int(tel["frame"]), tel.get("sim_time"), data)


def _v2x_frame(ctx: ProviderContext) -> Optional[Published]:
    return ctx.source.get_v2x_published()


def _links(ctx: ProviderContext) -> Optional[Published]:
    frame = ctx.source.get_v2x_frame()
    if frame is None:
        return None
    f = ctx.filter
    key = ("v2x.links", frame.frame, repr(sorted((k, tuple(v) if isinstance(v, list) else v) for k, v in f.items())))
    hit = ctx.cache.get(key)
    if hit is not None:
        return hit
    ids = set(f.get("station_ids", ()))
    role = f.get("role", "any")
    mt, lt, outcome = f.get("msg_types"), f.get("link_types"), f.get("outcome", "all")
    rows = []
    for l in frame.links:
        if ids:
            if role == "tx" and l.tx_id not in ids:
                continue
            if role == "rx" and l.rx_id not in ids:
                continue
            if role == "any" and l.tx_id not in ids and l.rx_id not in ids:
                continue
        if mt and l.msg_type not in mt:
            continue
        if lt and l.link_type not in lt:
            continue
        if outcome == "delivered" and not l.delivered:
            continue
        if outcome == "dropped" and l.delivered:
            continue
        rows.append(l)
    cap = ctx.source.v2x_links_max_rows
    data = {**links_to_columns(rows[:cap]), "matching": len(rows), "total_links": len(frame.links),
            "max_rows": cap, "truncated": len(rows) > cap}
    pub = ctx.cache[key] = Published(frame.frame, frame.frame, frame.sim_time, data)
    return pub


def _inbox(ctx: ProviderContext) -> Optional[Published]:
    frame = ctx.source.get_v2x_frame()
    if frame is None:
        return None
    sid = ctx.filter["station_id"]
    key = ("v2x.inbox", frame.frame, sid)
    hit = ctx.cache.get(key)
    if hit is None:
        msgs = [{"tx_id": m.tx_id, "msg_type": m.msg_type, "message": message_to_dict(m.message),
                 "link": link_to_dict(m.link)} for m in frame.inbox.get(sid, ())]
        threats = [t._asdict() for t in frame.threats.get(sid, ())]
        sent = [message_to_dict(m) for m in frame.sent.get(sid, ())]          # what this station transmitted
        data = {"station_id": sid, "known_station": sid in frame.stations, "messages": msgs,
                "threats": to_jsonable(threats), "sent": sent}
        hit = ctx.cache[key] = Published(frame.frame, frame.frame, frame.sim_time, data)
    return hit


def _v2x_stats(ctx: ProviderContext) -> Optional[Published]:
    return ctx.source.get_v2x_stats()


def _lidar_sensors(ctx: ProviderContext) -> Optional[Published]:
    mgr = ctx.source.lidar
    if mgr is None:
        return None
    return Published(mgr.info_version, None, None, {"sensors": mgr.list_sensors(), "budget": mgr.budget()})


def _events(ctx: ProviderContext) -> Optional[Published]:
    log = ctx.source.events
    cursor = ctx.state.get("cursor")
    if cursor is None:
        items = log.tail(50)
        dropped = False
    else:
        items, dropped = log.since(cursor)
    if not items:
        return None
    ctx.state["cursor"] = items[-1]["seq"]
    return Published(items[-1]["seq"], None, None, {"events": to_jsonable(items), "dropped": dropped})


def _run_status(ctx: ProviderContext) -> Optional[Published]:
    return ctx.source.get_run_status()


_AO = frozenset({ADMIN_R, OBSERVER_R})
_ALL = frozenset({ADMIN_R, OBSERVER_R, VEHICLE_R})

TOPICS: Dict[str, TopicSpec] = {t.name: t for t in [
    TopicSpec("world.state", _AO, 10.0, 5.0, _world_state, "Ground-truth state of every vehicle and pedestrian",
              validate_filter=validate_world_state_filter, ground_truth=True),
    TopicSpec("session.self", frozenset({VEHICLE_R}), 10.0, 5.0, _session_self,
              "Own telemetry of the joined vehicle", validate_filter=validate_no_filter),
    TopicSpec("v2x.frame", _AO, 2.0, 2.0, _v2x_frame,
              "Summary of the last V2X update: stations, channel counts, DENMs, RSU states",
              validate_filter=validate_no_filter, ground_truth=True),
    TopicSpec("v2x.links", _AO, 2.0, 2.0, _links, "Link results of the last V2X update (a filter is required)",
              filter_required=True, validate_filter=validate_links_filter, ground_truth=True),
    TopicSpec("v2x.inbox", _ALL, 2.0, 2.0, _inbox, "What one station actually received in the last V2X update",
              filter_required=True, validate_filter=validate_inbox_filter),
    TopicSpec("v2x.stats", _AO, 1.0, 1.0, _v2x_stats, "Pre-binned channel statistics over a sliding window",
              validate_filter=validate_no_filter, ground_truth=True),
    TopicSpec("lidar.sensors", _ALL, 10.0, 2.0, _lidar_sensors, "LiDAR sensors and the budget (push on change)",
              validate_filter=validate_no_filter),
    TopicSpec("events", _AO, 2.0, 1.0, _events, "Admin actions, session changes, LiDAR transitions, errors",
              validate_filter=validate_no_filter, ground_truth=True),
    TopicSpec("run.status", _AO, 1.0, 1.0, _run_status, "Frame, real-time factor, tick durations, queues",
              validate_filter=validate_no_filter, ground_truth=True),
]}
