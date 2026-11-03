"""
Road Side Unit (RSU) — V2I Infrastructure Node

Implements a stationary ITS station that:
  - Generates MAP messages describing intersection geometry.
  - Reads CARLA TrafficLight state and produces live SPAT messages.

The RSU transmits at the same 2 Hz cadence as V2V BSM beacons.
Per ETSI TR 101 607, RSUs use 33 dBm TX power (vs. 23 dBm for OBUs)
and cover a 300 m intersection radius.

IEEE / ETSI references:
  ETSI TS 103 301 v2.1.1  — MAP + SPAT (infrastructure services)
  SAE J2735-202007 §7.18  — SPaTIntersectionState
  ETSI TR 101 607 v1.1.1  — ITS-G5 RSU deployment guidelines
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

from .messages import MAPMessage, SPATMessage, SPATMovementState

logger = logging.getLogger(__name__)


# ── CARLA traffic-light state → SAE J2735 MovementPhaseState ────────────────

_CARLA_STATE_MAP: Dict[Any, str] = {}


def _build_carla_state_map() -> Dict[Any, str]:
    """
    Lazily build the CARLA TrafficLightState → SAE J2735 MovementPhaseState map.

    Imported lazily to avoid a hard CARLA dependency at module import time
    (important for unit tests that run without CARLA installed).
    """
    if _CARLA_STATE_MAP:
        return _CARLA_STATE_MAP
    try:
        import carla
        _CARLA_STATE_MAP.update({
            carla.TrafficLightState.Green:   "protectedMovementAllowed",
            carla.TrafficLightState.Yellow:  "protectedClearance",
            carla.TrafficLightState.Red:     "stop-And-Remain",
            carla.TrafficLightState.Off:     "caution-Conflicting-Traffic",
            carla.TrafficLightState.Unknown: "unavailable",
        })
    except (ImportError, AttributeError):
        pass
    return _CARLA_STATE_MAP


# ── RSU configuration ─────────────────────────────────────────────────────────

@dataclass
class RSUConfig:
    """
    Road Side Unit configuration.

    Defaults follow ETSI TR 101 607 v1.1.1 for ITS-G5 RSU deployments:
      TX power 33 dBm (higher than OBU 23 dBm to cover the full intersection).
      Broadcast range 300 m — typical urban intersection sight distance.
    """
    intersection_id: int
    position: tuple                       # (x, y, z) CARLA world coordinates
    tx_power_dbm: float = 33.0            # ETSI TR 101 607 RSU max
    broadcast_range_m: float = 300.0      # Intersection coverage radius (m)
    map_revision: int = 1


# ── Road Side Unit ─────────────────────────────────────────────────────────────

class RoadSideUnit:
    """
    Stationary V2I infrastructure node.

    Wraps one or more CARLA TrafficLight actors at an intersection and produces
    MAP / SPAT messages that are injected into the V2X network each 2 Hz tick
    via V2XNetwork.register_rsu().

    Usage::

        rsu = RoadSideUnit.from_traffic_light(tl_actor, intersection_id=1)
        v2x_network.register_rsu(rsu)
        # V2XNetwork.update() broadcasts MAP/SPAT automatically.
    """

    def __init__(
        self,
        config: RSUConfig,
        traffic_lights: Optional[List] = None,
    ) -> None:
        self.config = config
        self._traffic_lights: List = traffic_lights or []
        self._map_cache: Optional[MAPMessage] = None

        logger.debug(
            "RSU created: intersection_id=%d, pos=(%.1f, %.1f, %.1f), "
            "TX=%.0f dBm, range=%.0f m",
            config.intersection_id,
            *config.position,
            config.tx_power_dbm,
            config.broadcast_range_m,
        )

    # ── Factory ───────────────────────────────────────────────────────────────

    @classmethod
    def from_traffic_light(
        cls,
        traffic_light,
        intersection_id: int,
        broadcast_range_m: float = 300.0,
    ) -> "RoadSideUnit":
        """
        Build an RSU positioned at a single CARLA TrafficLight actor.

        Args:
            traffic_light:    CARLA TrafficLight actor.
            intersection_id:  Unique intersection identifier.
            broadcast_range_m: RSU coverage radius in metres.
        """
        loc = traffic_light.get_transform().location
        config = RSUConfig(
            intersection_id=intersection_id,
            position=(loc.x, loc.y, loc.z),
            broadcast_range_m=broadcast_range_m,
        )
        return cls(config, traffic_lights=[traffic_light])

    # ── MAP generation ─────────────────────────────────────────────────────────

    def get_map(self, timestamp: float) -> MAPMessage:
        """
        Return the static MAP message for this intersection.

        The MAP is generated once and cached; only the timestamp is refreshed
        each call.  Geometry changes would require a map_revision bump (not
        modelled dynamically in CARLA scenarios).
        """
        if self._map_cache is None:
            self._map_cache = MAPMessage(
                intersection_id=self.config.intersection_id,
                revision=self.config.map_revision,
                ref_point=self.config.position,
                timestamp=timestamp,
            )
        self._map_cache.timestamp = timestamp
        return self._map_cache

    # ── SPAT generation ────────────────────────────────────────────────────────

    def get_spat(self, timestamp: float) -> SPATMessage:
        """
        Build a live SPAT from current CARLA TrafficLight state(s).

        Each registered TrafficLight actor contributes one SPATMovementState.
        Phase timing is derived from CARLA's elapsed / phase-duration API;
        when timing is unavailable the window defaults to ±1 s around now.
        """
        state_map = _build_carla_state_map()
        states: List[SPATMovementState] = []

        for sg_id, tl in enumerate(self._traffic_lights):
            try:
                tl_state = tl.get_state()
                movement_event = state_map.get(tl_state, "unavailable")
                remaining = self._remaining_time(tl)

                states.append(SPATMovementState(
                    signal_group_id=sg_id,
                    movement_event=movement_event,
                    min_end_time_s=timestamp + max(remaining - 1.0, 0.0),
                    max_end_time_s=timestamp + remaining + 1.0,
                    likely_time_s=timestamp + remaining,
                ))
            except Exception as exc:
                logger.debug(
                    "RSU %d: failed to read TL state for signal group %d: %s",
                    self.config.intersection_id, sg_id, exc,
                )

        return SPATMessage(
            intersection_id=self.config.intersection_id,
            intersection_name=f"intersection_{self.config.intersection_id}",
            timestamp=timestamp,
            states=states,
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _remaining_time(traffic_light) -> float:
        """Best-effort remaining phase time from the CARLA API (seconds)."""
        try:
            import carla
            state = traffic_light.get_state()
            elapsed = traffic_light.get_elapsed_time()
            if state == carla.TrafficLightState.Green:
                return max(traffic_light.get_green_time() - elapsed, 0.0)
            elif state == carla.TrafficLightState.Yellow:
                return max(traffic_light.get_yellow_time() - elapsed, 0.0)
            elif state == carla.TrafficLightState.Red:
                return max(traffic_light.get_red_time() - elapsed, 0.0)
        except Exception:
            pass
        return 5.0  # safe fallback
