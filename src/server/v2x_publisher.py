"""What the simulation thread publishes after each V2X update (summary, channel statistics, the frame itself).

``on_update`` runs on the simulation thread; the three attributes are replaced, never mutated, so the hub reads
them from the event loop without locks.
"""

from __future__ import annotations

from typing import Optional

from src.server.published import Published, build_v2x_summary
from src.v2x.frame import V2XFrame
from src.v2x.stats import V2XStats


class V2XPublisher:
    def __init__(self, window_s: float = 60.0):
        self._stats = V2XStats(window_s=window_s)
        self.frame: Optional[V2XFrame] = None
        self.summary: Optional[Published] = None
        self.stats: Optional[Published] = None

    def on_update(self, frame: V2XFrame) -> None:
        self._stats.add(frame)
        self.summary = Published(frame.frame, frame.frame, frame.sim_time, build_v2x_summary(frame))
        self.stats = Published(frame.frame, frame.frame, frame.sim_time, self._stats.snapshot())
        self.frame = frame            # last: a reader that sees the frame also finds its summary
