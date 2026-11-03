"""V2X-assisted driving advisory (optional, off by default; no CARLA import).

A vehicle reacts only to what it RECEIVED through the channel, never to ground truth:

* **Threat advisory** - received BSMs of other vehicles give a threat level (TTC based, 0-4). At or above ``trigger_level``
  the vehicle slows by ``extra_slowdown_pct`` for ``hold_s`` seconds.
* **Hazard advisory** - a received DENM (admin-injected hazard or another vehicle's warning) that is relevant to the vehicle
  (it is within the DENM's relevance distance and moving towards the event) slows it by ``hazard_slowdown_pct`` while the
  warning keeps arriving, and for ``hold_s`` seconds after.

With a lossy or short-range channel fewer warnings arrive, so the advisory protects fewer vehicles: that coupling between
channel quality and driving behaviour is what this module is for. It is a deliberately small, explainable cooperative-driving
model, not a collision-avoidance controller: the Traffic Manager's own collision avoidance keeps running underneath.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple


@dataclass(frozen=True)
class AdvisoryConfig:
    enabled: bool = False
    trigger_level: int = 3            # threat level (0-4, see calculate_threat_level) that triggers the slow-down
    extra_slowdown_pct: float = 40.0  # added to the vehicle's own speed offset while a threat advisory is active
    hazard_slowdown_pct: float = 60.0 # added while a relevant received hazard warning is active
    hold_s: float = 3.0               # simulation seconds a slow-down is kept after its last trigger


def hazard_is_relevant(
    vehicle_xy: Tuple[float, float],
    vehicle_velocity_xy: Tuple[float, float],
    event_xy: Tuple[float, float],
    relevance_distance_m: float,
    min_speed_mps: float = 0.5,
) -> bool:
    """A received hazard matters to a vehicle that is within the DENM's relevance distance of the event and is moving
    towards it (positive component of its velocity along the line to the event). A stopped vehicle is not slowed further."""
    dx, dy = event_xy[0] - vehicle_xy[0], event_xy[1] - vehicle_xy[1]
    distance = math.hypot(dx, dy)
    if distance > relevance_distance_m:
        return False
    speed = math.hypot(*vehicle_velocity_xy)
    if speed < min_speed_mps:
        return False
    return distance < 1e-6 or (vehicle_velocity_xy[0] * dx + vehicle_velocity_xy[1] * dy) / (speed * distance) > 0.0


class V2XAdvisory:
    def __init__(self, config: AdvisoryConfig) -> None:
        self.config = config
        self._until: Dict[int, float] = {}
        self._pct: Dict[int, float] = {}
        self.activations = 0

    def update(
        self,
        sim_time: float,
        threat_level_by_vehicle: Mapping[int, int],
        hazard_relevant: Optional[Mapping[int, bool]] = None,
    ) -> List[Tuple[int, str]]:
        """Return ``[(vehicle_id, "slow" | "restore")]`` for vehicles whose advisory state changes now.

        ``"slow"`` is also returned when an active slow-down gets stronger (a hazard arrives while a threat slow-down runs);
        read the amount with ``slowdown_pct(vehicle_id)``.
        """
        if not self.config.enabled:
            return []
        hazard_relevant = hazard_relevant or {}
        actions: List[Tuple[int, str]] = []
        for vid in set(threat_level_by_vehicle) | set(hazard_relevant):
            want = 0.0
            if threat_level_by_vehicle.get(vid, 0) >= self.config.trigger_level:
                want = self.config.extra_slowdown_pct
            if hazard_relevant.get(vid, False):
                want = max(want, self.config.hazard_slowdown_pct)
            if want > 0.0:
                running = vid in self._until
                new = max(want, self._pct[vid]) if running else want   # an episode keeps its strongest request
                if not running:
                    self.activations += 1
                    actions.append((vid, "slow"))
                elif new > self._pct[vid]:
                    actions.append((vid, "slow"))                       # got stronger: apply the new amount
                self._pct[vid] = new
                self._until[vid] = sim_time + self.config.hold_s
        for vid, until in list(self._until.items()):
            threatened = threat_level_by_vehicle.get(vid, 0) >= self.config.trigger_level or hazard_relevant.get(vid, False)
            if sim_time >= until and not threatened:
                del self._until[vid]
                self._pct.pop(vid, None)
                actions.append((vid, "restore"))
        return actions

    def slowdown_pct(self, vehicle_id: int) -> float:
        """Extra speed reduction (percentage points of the speed limit) currently asked of ``vehicle_id``."""
        return self._pct.get(vehicle_id, 0.0)

    def forget(self, vehicle_id: int) -> None:
        self._until.pop(vehicle_id, None)
        self._pct.pop(vehicle_id, None)

    @property
    def active(self) -> int:
        return len(self._until)
