"""Immutable published views and JSON conversion for the WebSocket hub.

The simulation thread builds a ``Published`` and replaces one reference; the hub (event loop thread) only reads
it. Nothing here calls CARLA.
"""

from __future__ import annotations

import dataclasses
import enum
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

from src.v2x.dsrc_channel import LinkResult
from src.v2x.frame import V2XFrame


@dataclass(frozen=True)
class Published:
    """One published snapshot. ``version`` increases with every publication of that kind; the hub sends it as
    ``seq``, so a gap tells a client how many updates it skipped."""
    version: int
    frame: Optional[int]
    sim_time: Optional[float]
    data: Any


def to_jsonable(obj: Any) -> Any:
    """Plain JSON types: dataclasses -> dict, enums -> name, tuples -> lists, non-finite floats -> None."""
    if isinstance(obj, enum.Enum):          # before int: IntEnum is an int
        return obj.name
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_jsonable(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset)):
        return [to_jsonable(v) for v in obj]
    item = getattr(obj, "item", None)
    if callable(item) and getattr(obj, "shape", None) == ():     # numpy scalar
        return to_jsonable(item())
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        return to_jsonable(tolist())
    return str(obj)


def message_to_dict(msg: Any) -> Dict[str, Any]:
    d = to_jsonable(msg)
    if isinstance(d, dict):
        return {"type": type(msg).__name__, **d}
    return {"type": type(msg).__name__, "value": d}


LINK_FIELDS = tuple(f.name for f in dataclasses.fields(LinkResult))


def link_to_dict(link: LinkResult) -> Dict[str, Any]:
    return {k: to_jsonable(getattr(link, k)) for k in LINK_FIELDS}


def links_to_columns(links) -> Dict[str, Any]:
    """Compact form for many links: ``{"columns": [...], "rows": [[...], ...]}`` (key names once, not per row)."""
    return {"columns": list(LINK_FIELDS),
            "rows": [[to_jsonable(getattr(l, k)) for k in LINK_FIELDS] for l in links]}


def station_to_dict(s: Any) -> Dict[str, Any]:
    return to_jsonable(s)


def build_v2x_summary(frame: V2XFrame) -> Dict[str, Any]:
    """Ground-truth summary of one V2X update (topic ``v2x.frame``): per station what was sent / received and
    how many threats it sees, channel counts per link type, active DENMs and the signal state of each RSU."""
    received: Dict[int, int] = {sid: len(msgs) for sid, msgs in frame.inbox.items()}
    stations = []
    for sid, st in frame.stations.items():
        sent = frame.sent.get(sid, ())
        threats = frame.threats.get(sid, ())
        stations.append({
            "station_id": sid, "kind": st.kind, "x": st.x_m, "y": st.y_m, "z": st.z_m,
            "heading_deg": st.heading_deg, "speed_mps": st.speed_mps, "tx_power_dbm": st.tx_power_dbm,
            "sent": len(sent), "received": received.get(sid, 0), "threats": len(threats),
            "max_threat_level": max((t.level for t in threats), default=0),
        })
    denms, rsu_states = [], {}
    for sid, msgs in frame.sent.items():
        for m in msgs:
            name = type(m).__name__
            if name == "DENM":
                denms.append(message_to_dict(m))
            elif name == "SPATMessage":
                rsu_states.setdefault(str(sid), {})["spat"] = message_to_dict(m)
    return {
        "stations": stations,
        "channel": {lt: c._asdict() for lt, c in frame.stats.by_link_type.items()},
        "skipped_not_in_snapshot": frame.stats.skipped_not_in_snapshot,
        "denms": denms,
        "rsu_states": rsu_states,
        "links": len(frame.links),
    }
