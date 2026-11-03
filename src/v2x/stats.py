"""Pre-binned channel statistics over a sliding window of simulation time and over the whole run (REFACTOR_HANDOFF 5.7.2, topic v2x.stats).

The browser gets bins, never raw links: with 100 equipped vehicles one update has ~10,000 links. Everything here
is computed from the same ``LinkResult`` rows the recorder writes, so a plot can be reproduced from a run.

Definitions (also shown in the UI):
  * a link is *counted* in a PRR bin unless the application-layer awareness filter dropped it
    (``drop_cause == "out_of_awareness_range"``): that decision is not the radio's;
  * PRR of a bin = delivered / counted (None when nothing was counted), with its Wilson 95 % interval;
  * distance bins are ``bin_m`` wide, the last one collects everything beyond ``max_distance_m``.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from .frame import DROP_AWARENESS, DROP_RADIO, V2XFrame

DROP_CHANNEL = DROP_RADIO   # channel and MAC loss are decided by one draw here: one cause, "channel_or_mac"
DROP_MAC = "mac"            # kept for the wire format; never produced

MARGIN_EDGES_DB = list(range(-30, 65, 5))      # rx margin above sensitivity (not an SNR)


def wilson_interval(k: int, n: int, z: float = 1.959963984540054) -> Tuple[Optional[float], Optional[float]]:
    """Wilson score interval (default 95 %) for k successes in n trials; (None, None) when n == 0.
    Better behaved than the normal approximation near 0 and 1 and for small n, which is where link
    statistics live (a distance bin with 8 packets)."""
    if n <= 0:
        return None, None
    p = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / denom
    lo = 0.0 if k == 0 else max(0.0, centre - half)
    hi = 1.0 if k == n else min(1.0, centre + half)
    return lo, hi


class _Agg:
    """Counts of one V2X update (or the sum of several): per series (link type / LOS|NLOS) and distance bin the
    number of counted and delivered links, outcome counts per link type, receive-margin histograms."""

    def __init__(self, n_bins: int):
        self.n_bins = n_bins
        self.series: Dict[str, List[np.ndarray]] = {}
        self.drops: Dict[str, Dict[str, int]] = {}
        self.hist_deliv = np.zeros(len(MARGIN_EDGES_DB) - 1, dtype=np.int64)
        self.hist_drop = np.zeros(len(MARGIN_EDGES_DB) - 1, dtype=np.int64)
        self.updates = 0

    def merge(self, other: "_Agg") -> None:
        for k, (n, d) in other.series.items():
            mine = self.series.setdefault(k, [np.zeros(self.n_bins, dtype=np.int64), np.zeros(self.n_bins, dtype=np.int64)])
            mine[0] = mine[0] + n
            mine[1] = mine[1] + d
        for lt, c in other.drops.items():
            acc = self.drops.setdefault(lt, {"delivered": 0, DROP_CHANNEL: 0, DROP_MAC: 0, DROP_AWARENESS: 0})
            for kk, vv in c.items():
                acc[kk] += vv
        self.hist_deliv = self.hist_deliv + other.hist_deliv
        self.hist_drop = self.hist_drop + other.hist_drop
        self.updates += other.updates


class V2XStats:
    """Aggregates over the last ``window_s`` seconds of SIMULATION time and over the whole run (since the first
    update this object saw)."""

    def __init__(self, window_s: float = 60.0, bin_m: float = 10.0, max_distance_m: float = 400.0):
        self.window_s = float(window_s)
        self.bin_m = float(bin_m)
        self.n_bins = int(round(max_distance_m / bin_m)) + 1       # +1: overflow bin
        self._window: Deque[Tuple[int, float, _Agg]] = deque()     # (frame, sim_time, aggregate)
        self._run = _Agg(self.n_bins)
        self._run_first: Optional[Tuple[int, float]] = None
        self._run_last: Optional[Tuple[int, float]] = None

    def add(self, frame: V2XFrame) -> None:
        agg = self._aggregate(frame)
        if self._window and frame.sim_time < self._window[-1][1]:       # the simulation restarted
            self._window.clear()
            self._run = _Agg(self.n_bins)
            self._run_first = None
        self._window.append((frame.frame, frame.sim_time, agg))
        while len(self._window) > 1 and self._window[0][1] < frame.sim_time - self.window_s:
            self._window.popleft()
        self._run.merge(agg)
        if self._run_first is None:
            self._run_first = (frame.frame, frame.sim_time)
        self._run_last = (frame.frame, frame.sim_time)

    def _aggregate(self, frame: V2XFrame) -> _Agg:
        links = frame.links
        n = len(links)
        dist = np.fromiter((l.distance_m for l in links), dtype=np.float64, count=n)
        deliv = np.fromiter((l.delivered for l in links), dtype=bool, count=n)
        los = np.fromiter((l.los for l in links), dtype=bool, count=n)
        margin = np.fromiter((l.rx_margin_db for l in links), dtype=np.float64, count=n)
        cause = np.array([l.drop_cause or "" for l in links], dtype="<U32")
        ltype = np.array([l.link_type for l in links]) if n else np.array([], dtype="<U3")
        counted = cause != DROP_AWARENESS
        bins = np.minimum((dist // self.bin_m).astype(np.int64), self.n_bins - 1)

        agg = _Agg(self.n_bins)
        agg.updates = 1
        for lt in sorted(set(ltype.tolist())):
            m_lt = ltype == lt
            for los_flag, name in ((True, "LOS"), (False, "NLOS")):
                m = m_lt & counted & (los == los_flag)
                if not m.any():
                    continue
                agg.series[f"{lt}/{name}"] = [np.bincount(bins[m], minlength=self.n_bins),
                                              np.bincount(bins[m & deliv], minlength=self.n_bins)]
            agg.drops[lt] = {
                "delivered": int((m_lt & deliv).sum()),
                DROP_CHANNEL: int((m_lt & (cause == DROP_CHANNEL)).sum()),
                DROP_MAC: int((m_lt & (cause == DROP_MAC)).sum()),
                DROP_AWARENESS: int((m_lt & (cause == DROP_AWARENESS)).sum()),
            }
        lo, hi = MARGIN_EDGES_DB[0], MARGIN_EDGES_DB[-1]
        agg.hist_deliv = np.histogram(np.clip(margin[counted & deliv], lo, hi), bins=MARGIN_EDGES_DB)[0]
        agg.hist_drop = np.histogram(np.clip(margin[counted & ~deliv], lo, hi), bins=MARGIN_EDGES_DB)[0]
        return agg

    def _render(self, agg: _Agg, first: Optional[Tuple[int, float]], last: Optional[Tuple[int, float]]) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "updates": agg.updates,
            "first_frame": first[0] if first else None, "last_frame": last[0] if last else None,
            "sim_time_span": [first[1], last[1]] if first and last else None,
            "prr_by_distance": {}, "drops": {}, "rx_margin_hist": None,
        }
        if not agg.updates:
            return out
        for k in sorted(agg.series):
            n, d = agg.series[k]
            prr: List[Optional[float]] = [None if c == 0 else float(x) / float(c) for x, c in zip(d, n)]
            ci = [wilson_interval(int(x), int(c)) for x, c in zip(d, n)]
            out["prr_by_distance"][k] = {"counted": n.tolist(), "delivered": d.tolist(), "prr": prr,
                                         "ci95_low": [lo for lo, _ in ci], "ci95_high": [hi for _, hi in ci]}
        out["drops"] = {lt: dict(c) for lt, c in sorted(agg.drops.items())}
        out["rx_margin_hist"] = {"edges_db": MARGIN_EDGES_DB, "note": "values are clipped into the first/last bin",
                                 "delivered": agg.hist_deliv.tolist(), "dropped": agg.hist_drop.tolist()}
        return out

    def snapshot(self) -> Dict[str, Any]:
        win = _Agg(self.n_bins)
        for _f, _t, a in self._window:
            win.merge(a)
        first = (self._window[0][0], self._window[0][1]) if self._window else None
        last = (self._window[-1][0], self._window[-1][1]) if self._window else None
        return {
            "bin_m": self.bin_m, "distance_bin_starts_m": [i * self.bin_m for i in range(self.n_bins)],
            "last_bin_is_overflow": True, "window_s": self.window_s,
            "definition": "PRR = delivered / counted links (Wilson 95 % interval); links dropped by the "
                          "awareness-range filter are not counted",
            "window": self._render(win, first, last),
            "run": self._render(self._run, self._run_first, self._run_last),
        }
