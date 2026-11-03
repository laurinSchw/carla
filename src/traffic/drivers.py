"""Heterogeneous, reproducible driver behaviour for background traffic (no CARLA import).

CARLA's Traffic Manager (TM) drives every autopilot vehicle the same way by default: lane keeping, traffic lights,
collision avoidance, a global speed offset. Real traffic is not like that, and a V2X study needs vehicles that differ
in speed, following distance and lane-change habits. This module gives each vehicle a *profile* drawn from a seeded
mix, and turns it into per-vehicle TM settings.

All numbers below are PROJECT DEFAULTS chosen to be plausible, not calibrated to a measured driver population.
Change them in ``DriverMix`` / ``PROFILES`` and record the values (they go into the run manifest).

TM semantics used (CARLA 0.9.x Traffic Manager API, to be confirmed against 0.9.16 on a live server):
  * ``vehicle_percentage_speed_difference(actor, pct)``: positive = slower than the speed limit by pct %, NEGATIVE = faster.
  * ``distance_to_leading_vehicle(actor, m)``: minimum gap to the vehicle ahead in metres.
  * ``random_left/right_lanechange_percentage(actor, pct)``: chance of an unprompted lane change.
  * ``keep_right_rule_percentage``, ``ignore_lights_percentage``, ``ignore_signs_percentage``: probabilities in %.
"""

from __future__ import annotations

import logging
import random
from dataclasses import asdict, dataclass
from typing import Dict, List, Mapping

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DriverProfile:
    name: str
    speed_pct: float            # % BELOW the speed limit (negative = above it)
    follow_distance_m: float    # minimum gap to the leader
    lane_change_left_pct: float
    lane_change_right_pct: float
    keep_right_pct: float
    ignore_lights_pct: float = 0.0
    ignore_signs_pct: float = 0.0

    def tm_settings(self) -> Dict[str, float]:
        """TM method name -> value, applied per vehicle as ``getattr(tm, name)(actor, value)``."""
        return {
            "vehicle_percentage_speed_difference": self.speed_pct,
            "distance_to_leading_vehicle": self.follow_distance_m,
            "random_left_lanechange_percentage": self.lane_change_left_pct,
            "random_right_lanechange_percentage": self.lane_change_right_pct,
            "keep_right_rule_percentage": self.keep_right_pct,
            "ignore_lights_percentage": self.ignore_lights_pct,
            "ignore_signs_percentage": self.ignore_signs_pct,
        }


#: archetypes (project defaults, uncalibrated)
PROFILES: Mapping[str, DriverProfile] = {
    "cautious": DriverProfile("cautious", speed_pct=15.0, follow_distance_m=4.5, lane_change_left_pct=2.0,
                              lane_change_right_pct=2.0, keep_right_pct=80.0),
    "normal": DriverProfile("normal", speed_pct=0.0, follow_distance_m=2.5, lane_change_left_pct=8.0,
                            lane_change_right_pct=8.0, keep_right_pct=50.0),
    "aggressive": DriverProfile("aggressive", speed_pct=-25.0, follow_distance_m=1.2, lane_change_left_pct=30.0,
                                lane_change_right_pct=20.0, keep_right_pct=15.0),
}

#: per-vehicle jitter applied to the speed offset so that vehicles of one archetype still differ (uniform, +- this many %)
SPEED_JITTER_PCT = 5.0


@dataclass(frozen=True)
class DriverMix:
    """Share of each archetype in the background traffic (normalised internally)."""
    cautious: float = 0.2
    normal: float = 0.6
    aggressive: float = 0.2

    def weights(self) -> List[float]:
        w = [self.cautious, self.normal, self.aggressive]
        total = sum(w)
        if total <= 0 or min(w) < 0:
            raise ValueError("driver mix weights must be non-negative and not all zero")
        return [x / total for x in w]


class DriverAssigner:
    """Deterministic profile draw: same seed + same call order => same drivers."""

    def __init__(self, mix: DriverMix, seed: int) -> None:
        self.mix = mix
        self._weights = mix.weights()
        self._rng = random.Random(seed)

    def next_profile(self) -> DriverProfile:
        name = self._rng.choices(("cautious", "normal", "aggressive"), weights=self._weights)[0]
        base = PROFILES[name]
        jitter = self._rng.uniform(-SPEED_JITTER_PCT, SPEED_JITTER_PCT)
        return DriverProfile(**{**asdict(base), "speed_pct": base.speed_pct + jitter})


def apply_profile(tm, actor, profile: DriverProfile) -> List[str]:
    """Apply ``profile`` to one vehicle through the Traffic Manager. Returns the setting names that failed.

    A TM build without one of the methods must not break spawning, so failures are logged and reported, not raised.
    """
    failed = []
    for name, value in profile.tm_settings().items():
        try:
            getattr(tm, name)(actor, value)
        except Exception as exc:
            failed.append(name)
            logger.warning("TM setting %s failed for actor %s: %s", name, getattr(actor, "id", "?"), exc)
    return failed
