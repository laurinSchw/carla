"""Run callables on the single simulation thread that owns CARLA.

CARLA's Python client is only touched by one thread (the tick thread). Everything else
(HTTP handlers, WebSocket tasks) submits a callable and waits on the returned Future.
The tick thread calls ``drain()`` once per frame, so submitted work always runs between
two ticks, at a frame boundary.

No CARLA import here, so it is unit-testable anywhere.
"""

import queue
import threading
from concurrent.futures import Future
from typing import Any, Callable, Optional


class SimThreadExecutor:
    def __init__(self) -> None:
        self._queue: "queue.Queue[tuple[Future, Callable[..., Any], tuple, dict]]" = queue.Queue()
        self._sim_thread_id: Optional[int] = None
        self._closed = False
        self._lock = threading.Lock()

    # --- sim-thread side -------------------------------------------------
    def bind_current_thread(self) -> None:
        """Declare the calling thread as the simulation thread."""
        with self._lock:
            self._sim_thread_id = threading.get_ident()
            self._closed = False

    def is_sim_thread(self) -> bool:
        return self._sim_thread_id == threading.get_ident()

    def drain(self, max_items: int = 1000) -> int:
        """Run queued callables (call from the sim thread). Returns how many ran."""
        ran = 0
        while ran < max_items:
            try:
                fut, fn, args, kwargs = self._queue.get_nowait()
            except queue.Empty:
                break
            ran += 1
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                fut.set_result(fn(*args, **kwargs))
            except BaseException as exc:  # noqa: BLE001 - delivered to the waiting caller
                fut.set_exception(exc)
        return ran

    def close(self, reason: str = "simulation stopped") -> None:
        """Reject new work and fail everything still queued."""
        with self._lock:
            self._closed = True
        while True:
            try:
                fut, *_ = self._queue.get_nowait()
            except queue.Empty:
                break
            if fut.set_running_or_notify_cancel():
                fut.set_exception(RuntimeError(reason))

    # --- caller side -----------------------------------------------------
    def submit(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Future:
        """Schedule ``fn`` on the sim thread. Runs inline if already on it."""
        fut: Future = Future()
        if self.is_sim_thread():
            if fut.set_running_or_notify_cancel():
                try:
                    fut.set_result(fn(*args, **kwargs))
                except BaseException as exc:  # noqa: BLE001
                    fut.set_exception(exc)
            return fut
        with self._lock:
            if self._closed or self._sim_thread_id is None:
                fut.set_exception(RuntimeError("simulation is not running"))
                return fut
            self._queue.put((fut, fn, args, kwargs))
        return fut
