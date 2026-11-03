"""The immutable result of one V2X update (REFACTOR_HANDOFF section 5.3).

A ``V2XFrame`` is built once per update from the network's state and never changed afterwards, so any thread (the WebSocket
hub, the recorder) can hold and read it without locks. Ground truth (``stations``) and what each station actually received
(``inbox``) are separate on purpose: a vehicle's V2X view is its inbox, not the ground truth.

Station ids in a frame: vehicles and pedestrians use their CARLA actor id; an RSU uses ``-intersection_id`` and an admin hazard's
virtual transmitter ``-(hazard station id + 1)`` (both negative, so they cannot collide with actor ids). The network itself keeps
the raw positive ids (the recorder writes those); ``build_v2x_frame`` converts, and counts a link's drop cause as "channel_or_mac"
because the channel model decides channel and MAC loss with ONE random draw (they cannot be told apart).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, NamedTuple, Optional, Tuple

from .dsrc_channel import LinkResult

KIND_OBU, KIND_RSU, KIND_VRU = "OBU", "RSU", "VRU"
DROP_RADIO = "channel_or_mac"
DROP_AWARENESS = "out_of_awareness_range"
HAZARD_STATION_BASE = 1_000_000


def rsu_station_id(intersection_id: int) -> int:
    return -int(intersection_id)


def hazard_station_id(raw_station_id: int) -> int:
    return -(int(raw_station_id) + 1)


@dataclass(frozen=True)
class StationState:
    """Ground truth of one station at the frame's simulation time."""
    station_id: int
    kind: str                       # "OBU" | "RSU" | "VRU"
    x_m: float
    y_m: float
    z_m: float
    heading_deg: float = 0.0
    speed_mps: float = 0.0
    tx_power_dbm: Optional[float] = None


class ReceivedMessage(NamedTuple):
    """A message that reached a receiver's application, with the link outcome that delivered it."""
    tx_id: int
    msg_type: str
    message: Any
    link: LinkResult


class Threat(NamedTuple):
    """Collision threat of ``other_id`` as seen by a receiver, computed from the receiver's inbox."""
    other_id: int
    level: int
    ttc_s: float
    distance_m: float


class LinkCounts(NamedTuple):
    total: int
    delivered: int
    channel_drops: int        # radio drops: channel and MAC together (one draw)
    mac_drops: int            # always 0 here, kept for the wire format
    awareness_drops: int


@dataclass(frozen=True)
class ChannelStats:
    """Link outcomes of THIS frame (not cumulative), per link type."""
    by_link_type: Mapping[str, LinkCounts]
    skipped_not_in_snapshot: int = 0


@dataclass(frozen=True)
class V2XFrame:
    frame: int
    sim_time: float
    stations: Mapping[int, StationState]               # ground truth
    sent: Mapping[int, Tuple[Any, ...]]                # tx station -> messages it transmitted
    inbox: Mapping[int, Tuple[ReceivedMessage, ...]]   # rx station -> what it received
    links: Tuple[LinkResult, ...]                      # every attempt of this frame
    threats: Mapping[int, Tuple[Threat, ...]]          # rx station -> threats from its inbox
    stats: ChannelStats


def count_links(links) -> Dict[str, LinkCounts]:
    """Group link outcomes by link type."""
    acc: Dict[str, List[int]] = {}
    for l in links:
        c = acc.setdefault(l.link_type, [0, 0, 0, 0, 0])
        c[0] += 1
        if l.delivered:
            c[1] += 1
        elif l.drop_cause == DROP_RADIO:
            c[2] += 1
        elif l.drop_cause == DROP_AWARENESS:
            c[4] += 1
    return {k: LinkCounts(*v) for k, v in acc.items()}


def _frozen(d: dict) -> Mapping:
    return MappingProxyType(d)


def build_v2x_frame(net, frame: int, sim_time: float) -> V2XFrame:
    """Snapshot the state of a ``V2XNetwork`` right after an update (simulation thread)."""
    stations: Dict[int, StationState] = {}
    sent: Dict[int, List[Any]] = {}
    for vid, b in net.bsm_messages.items():
        stations[vid] = StationState(vid, KIND_OBU, b.x_m, b.y_m, b.z_m, b.heading, b.speed, net._dsrc_channel.config.tx_power_dbm)
        sent.setdefault(vid, []).append(b)
    for iid, rsu in net.get_rsus().items():
        sid = rsu_station_id(iid)
        x, y, z = rsu.config.position
        stations[sid] = StationState(sid, KIND_RSU, x, y, z, 0.0, 0.0, rsu.config.tx_power_dbm)
        for m in (net.map_messages.get(iid), net.spat_messages.get(iid)):
            if m is not None:
                sent.setdefault(sid, []).append(m)
    for vid, m in net.get_all_vruam().items():
        stations[vid] = StationState(vid, KIND_VRU, m.position[0], m.position[1], m.position[2], m.heading, m.speed,
                                     net._dsrc_channel.config.tx_power_dbm)
        sent.setdefault(vid, []).append(m)
    for (station_id, _action), denm in tuple(net.denm_store.items()):
        sid = hazard_station_id(station_id) if station_id >= HAZARD_STATION_BASE else station_id
        if station_id >= HAZARD_STATION_BASE and sid not in stations:
            x, y, z = denm.event_position
            stations[sid] = StationState(sid, KIND_RSU, x, y, z, 0.0, 0.0, net._hazard_tx_power.get(station_id))
        sent.setdefault(sid, []).append(denm)

    def tx_of(l: LinkResult) -> int:
        if l.link_type == "I2V":
            return hazard_station_id(l.tx_id) if l.tx_id >= HAZARD_STATION_BASE else rsu_station_id(l.tx_id)
        return l.tx_id

    links: List[LinkResult] = []
    inbox: Dict[int, List[ReceivedMessage]] = {}
    for l in net.get_link_results():
        tx = tx_of(l)
        if tx != l.tx_id:
            l = dataclasses.replace(l, tx_id=tx)          # the network's own record keeps the raw id
        links.append(l)
        if not l.delivered:
            continue
        msg = _message_for(net, l, tx)
        if msg is not None:
            inbox.setdefault(l.rx_id, []).append(ReceivedMessage(tx, l.msg_type, msg, l))

    threats: Dict[int, List[Threat]] = {}
    for (rx, other), t in tuple(net.threats.items()):
        threats.setdefault(rx, []).append(Threat(other, t["level"], t["ttc"], t["distance"]))

    return V2XFrame(
        frame=frame, sim_time=sim_time, stations=_frozen(stations),
        sent=_frozen({k: tuple(v) for k, v in sent.items()}),
        inbox=_frozen({k: tuple(v) for k, v in inbox.items()}),
        links=tuple(links),
        threats=_frozen({k: tuple(v) for k, v in threats.items()}),
        stats=ChannelStats(_frozen(count_links(links)), int(net.stats.get("bsm_skipped_not_in_snapshot", 0))),
    )


def _message_for(net, link: LinkResult, tx: int) -> Any:
    mt = link.msg_type
    if mt == "BSM":
        return net.bsm_messages.get(tx)
    if mt == "MAP":
        return net.map_messages.get(-tx)
    if mt == "SPAT":
        return net.spat_messages.get(-tx)
    if mt == "VRUAM":
        return net.vru_messages.get(tx)
    if mt == "DENM":
        for (station_id, _a), d in tuple(net.denm_store.items()):
            sid = hazard_station_id(station_id) if station_id >= HAZARD_STATION_BASE else station_id
            if sid == tx:
                return d
    return None
