"""SimThreadExecutor: work submitted from other threads runs on the sim thread only."""

import threading
import time

import pytest

from src.server.sim_executor import SimThreadExecutor


def _start_sim_thread(ex, ticks=None):
    stop = threading.Event()
    seen = {}

    def loop():
        ex.bind_current_thread()
        seen["thread"] = threading.get_ident()
        seen["ready"] = True
        while not stop.is_set():
            ex.drain()
            time.sleep(0.001)

    t = threading.Thread(target=loop, name="sim", daemon=True)
    t.start()
    while "ready" not in seen:
        time.sleep(0.001)
    return t, stop, seen


def test_submitted_work_runs_on_sim_thread_not_caller():
    ex = SimThreadExecutor()
    t, stop, seen = _start_sim_thread(ex)
    try:
        ran_on = ex.submit(threading.get_ident).result(timeout=2)
        assert ran_on == seen["thread"] != threading.get_ident()
    finally:
        stop.set(); t.join()


def test_results_and_exceptions_are_delivered():
    ex = SimThreadExecutor()
    t, stop, _ = _start_sim_thread(ex)
    try:
        assert ex.submit(lambda a, b=1: a + b, 2, b=3).result(timeout=2) == 5
        with pytest.raises(ValueError, match="boom"):
            ex.submit(lambda: (_ for _ in ()).throw(ValueError("boom"))).result(timeout=2)
        assert ex.submit(lambda: 1).result(timeout=2) == 1  # still alive after an error
    finally:
        stop.set(); t.join()


def test_fifo_order_from_many_threads():
    ex = SimThreadExecutor()
    t, stop, _ = _start_sim_thread(ex)
    order = []
    try:
        futs = [ex.submit(order.append, i) for i in range(200)]
        [f.result(timeout=5) for f in futs]
        assert order == list(range(200))
        with_threads = []
        def worker(n):
            with_threads.append(ex.submit(lambda n=n: n).result(timeout=5))
        ths = [threading.Thread(target=worker, args=(n,)) for n in range(20)]
        [th.start() for th in ths]; [th.join() for th in ths]
        assert sorted(with_threads) == list(range(20))
    finally:
        stop.set(); t.join()


def test_submit_from_sim_thread_runs_inline_without_deadlock():
    ex = SimThreadExecutor()
    t, stop, _ = _start_sim_thread(ex)
    try:
        assert ex.submit(lambda: ex.submit(lambda: 7).result(timeout=1)).result(timeout=2) == 7
    finally:
        stop.set(); t.join()


def test_submit_before_sim_thread_or_after_close_fails_fast():
    ex = SimThreadExecutor()
    with pytest.raises(RuntimeError):
        ex.submit(lambda: 1).result(timeout=1)
    t, stop, _ = _start_sim_thread(ex)
    stop.set(); t.join()
    ex.close("stopped")
    with pytest.raises(RuntimeError, match="not running"):
        ex.submit(lambda: 1).result(timeout=1)


def test_close_fails_pending_work():
    ex = SimThreadExecutor()
    ex.bind_current_thread()  # no one drains: work stays queued
    other = threading.Thread(target=lambda: None)
    other.start(); other.join()
    futs = []
    def submit_from_other():
        futs.append(ex.submit(lambda: 1))
    th = threading.Thread(target=submit_from_other); th.start(); th.join()
    ex.close("shutting down")
    with pytest.raises(RuntimeError, match="shutting down"):
        futs[0].result(timeout=1)
