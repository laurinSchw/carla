"""Bounded, thread-safe log of things that happened (admin actions, session changes, LiDAR transitions, errors).

Written from any thread (the simulation thread, request handlers); read by the WebSocket ``events`` topic and
(later) the run recorder. Entries are immutable dicts; ``seq`` increases by one per entry, so a reader that
remembers the last ``seq`` it saw gets exactly the entries it missed (or learns that some were dropped).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple


class EventLog:
    def __init__(self, max_size: int = 1000):
        self._items: Deque[Dict[str, Any]] = deque(maxlen=max_size)
        self._lock = threading.Lock()
        self._seq = 0
        self._listeners: list = []

    def add(self, kind: str, *, frame: Optional[int] = None, sim_time: Optional[float] = None, **data: Any) -> int:
        with self._lock:
            self._seq += 1
            entry = {"seq": self._seq, "kind": kind, "frame": frame, "sim_time": sim_time, "wall_time": time.time(), "data": data}
            self._items.append(entry)
            listeners = list(self._listeners)
            seq = self._seq
        for fn in listeners:                     # outside the lock: a listener may take its time (it must not block, though)
            try:
                fn(entry)
            except Exception:                    # a broken listener must not break whoever logged the event
                pass
        return seq

    def add_listener(self, fn) -> None:
        with self._lock:
            self._listeners.append(fn)

    def remove_listener(self, fn) -> None:
        with self._lock:
            if fn in self._listeners:
                self._listeners.remove(fn)

    @property
    def last_seq(self) -> int:
        with self._lock:
            return self._seq

    def since(self, seq: int, limit: int = 200) -> Tuple[List[Dict[str, Any]], bool]:
        """Entries with ``seq`` greater than the given one, oldest first; second value: some were dropped
        (older than the log keeps) or cut by ``limit``."""
        with self._lock:
            items = [e for e in self._items if e["seq"] > seq]
            first_kept = self._items[0]["seq"] if self._items else self._seq + 1
        lost = seq + 1 < first_kept
        if len(items) > limit:
            return items[-limit:], True
        return items, lost

    def tail(self, n: int = 50) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._items)[-n:]
