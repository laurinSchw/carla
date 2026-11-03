"""V2X-assisted driving advisory (optional, off by default; no CARLA import).

A vehicle that RECEIVED BSMs through the channel (not ground truth) and sees a threat level at or above ``trigger_level``
slows down for ``hold_s`` seconds, then returns to its own profile speed. This couples driver behaviour to packet loss:
with a lossy channel fewer threats are seen, so the advisory helps less. It is a deliberately small, explainable
cooperative-driving model, not a collision-avoidance controller: the TM's own collision avoidance still runs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Tuple


@dataclass(frozen=True)
class AdvisoryConfig:
    enabled: bool = False
    trigger_level: int = 3        # threat level (0-4, see calculate_threat_level) that triggers the slow-down
    extra_slowdown_pct: float = 40.0  # added to the vehicle's own speed offset while the advisory is active
    hold_s: float = 3.0           # simulation seconds the slow-down is kept after the last trigger


class V2XAdvisory:
    def __init__(self, config: AdvisoryConfig) -> None:
        self.config = config
        self._until: Dict[int, float] = {}
        self.activations = 0

    def update(self, sim_time: float, threat_level_by_vehicle: Mapping[int, int]) -> List[Tuple[int, str]]:
        """Return ``[(vehicle_id, "slow" | "restore")]`` for vehicles whose advisory state changes now."""
        if not self.config.enabled:
            return []
        actions: List[Tuple[int, str]] = []
        for vid, level in threat_level_by_vehicle.items():
            if level >= self.config.trigger_level:
                if vid not in self._until:
                    actions.append((vid, "slow"))
                    self.activations += 1
                self._until[vid] = sim_time + self.config.hold_s
        for vid, until in list(self._until.items()):
            if sim_time >= until and threat_level_by_vehicle.get(vid, 0) < self.config.trigger_level:
                del self._until[vid]
                actions.append((vid, "restore"))
        return actions

    def forget(self, vehicle_id: int) -> None:
        self._until.pop(vehicle_id, None)

    @property
    def active(self) -> int:
        return len(self._until)
