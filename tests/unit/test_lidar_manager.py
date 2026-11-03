"""LiDAR manager: lifecycle, budget, ownership, auto-release, frame-exact collection, stream profiles."""

import numpy as np
import pytest

from src.lidar import protocol
from src.lidar.config import LidarConfig, LidarLimits, Mount
from src.lidar.manager import (
    LidarBudgetExceeded, LidarError, LidarManager, LidarNotFound, LidarPermissionError, Measurement,
    State, StreamProfile,
)

DT = 0.05
PT = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"), ("object_idx", "u4"), ("tag", "u4")])


class FakeHandle:
    def __init__(self, backend, actor_id, config):
        self.backend, self.actor_id, self.config = backend, actor_id, config
        self.destroyed = False
        self.requested_frames = []

    def get_measurement(self, frame, timeout_s):
        self.requested_frames.append(frame)
        if frame in self.backend.drop_frames:
            return None
        n = self.config.points_per_frame(DT)
        pts = np.zeros(n, dtype=PT)
        pts["x"] = np.arange(n)
        pts["object_idx"] = np.arange(n)
        return Measurement(frame, pts, protocol.SEMANTIC_FIELDS, (1.0, 2.0, 3.0), (0.0, 90.0, 0.0))

    def destroy(self):
        self.destroyed = True
        self.backend.live.discard(self)


class FakeBackend:
    def __init__(self):
        self.live, self.spawned, self.drop_frames, self.fail = set(), [], set(), None

    def spawn(self, actor_id, config):
        if self.fail:
            raise RuntimeError(self.fail)
        h = FakeHandle(self, actor_id, config)
        self.live.add(h)
        self.spawned.append(h)
        return h


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture
def env():
    backend, clock = FakeBackend(), Clock()
    limits = LidarLimits(max_sensors=2, max_total_points_per_second=1_000_000, keep_alive_s=10.0)
    mgr = LidarManager(backend, limits, DT, wall_clock=clock, monotonic=clock)
    mgr.frame = 100

    def tick(n=1):
        for _ in range(n):
            mgr.frame += 1
            mgr.on_tick(mgr.frame, mgr.frame * DT)
    mgr.tick = tick
    return mgr, backend, clock


def cfg(pps=300_000, channels=32, **kw):
    return LidarConfig(points_per_second=pps, channels=channels, **kw)


def states(mgr):
    return [s["state"] for s in mgr.list_sensors()]


# ── lifecycle ────────────────────────────────────────────────────────────────

def test_request_to_active_follows_the_state_machine_at_tick_boundaries(env):
    mgr, backend, _ = env
    s = mgr.request("sess-a", 7, cfg(), own_actor_id=7)
    assert states(mgr) == ["requested"] and not backend.live          # nothing touches CARLA before the tick
    mgr.tick()
    assert states(mgr) == ["warming_up"] and len(backend.live) == 1   # spawned at the boundary
    mgr.tick()
    assert states(mgr) == ["active"]
    assert mgr.get_info(s.sensor_id)["last_frame"] == mgr.frame
    assert [e["state"] for e in mgr.events] == ["requested", "spawning", "warming_up", "active"]
    assert all(e["frame"] >= 100 for e in mgr.events[1:])             # events are stamped with the frame


def test_each_tick_collects_exactly_its_own_frame(env):
    mgr, backend, _ = env
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(4)
    handle = backend.spawned[0]
    assert handle.requested_frames == [102, 103, 104]                 # spawned in 101, then one request per tick
    assert mgr.get_info(1)["last_frame"] == 104


def test_missed_measurement_is_counted_not_fatal(env):
    mgr, backend, _ = env
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    backend.drop_frames.add(103)
    mgr.tick()
    info = mgr.get_info(1)
    assert info["state"] == "active" and info["missed_frames"] == 1 and info["last_frame"] == 102


def test_spawn_failure_goes_to_error_with_a_reason_and_frees_the_budget(env):
    mgr, backend, _ = env
    backend.fail = "actor gone"
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick()
    info = mgr.list_sensors()[0]
    assert info["state"] == "error" and "actor gone" in info["error"]
    assert mgr.budget()["used_pps"] == 0 and not backend.live


def test_release_destroys_the_sensor_and_leaves_no_actor(env):
    mgr, backend, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    mgr.release(s.sensor_id, requester="a")
    assert states(mgr) == ["stopping"]
    mgr.tick()
    assert mgr.list_sensors() == [] and not backend.live
    assert mgr.events[-1]["state"] == "destroyed"


def test_owner_session_end_destroys_its_sensors(env):
    mgr, backend, _ = env
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.request("b", 8, cfg(), own_actor_id=8)
    mgr.tick(2)
    assert mgr.release_owner("a") == 1
    mgr.tick()
    assert [s["owner"] for s in mgr.list_sensors()] == ["b"] and len(backend.live) == 1


def test_reconfigure_respawns_with_new_config_at_a_tick_boundary(env):
    mgr, backend, _ = env
    s = mgr.request("a", 7, cfg(channels=32), own_actor_id=7)
    mgr.tick(2)
    old = backend.spawned[0]
    mgr.reconfigure(s.sensor_id, cfg(channels=64), requester="a")
    assert states(mgr) == ["reconfiguring"] and not old.destroyed     # not before the boundary
    mgr.tick()
    assert old.destroyed and states(mgr) == ["warming_up"]
    assert backend.spawned[1].config.channels == 64 and len(backend.live) == 1
    mgr.tick()
    assert states(mgr) == ["active"] and mgr.get_info(s.sensor_id)["config"]["channels"] == 64


def test_reconfigure_rejected_while_not_running_and_for_other_owners(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    with pytest.raises(LidarError):
        mgr.reconfigure(s.sensor_id, cfg(), requester="a")            # still 'requested'
    mgr.tick(2)
    with pytest.raises(LidarPermissionError):
        mgr.reconfigure(s.sensor_id, cfg(), requester="someone-else")
    mgr.reconfigure(s.sensor_id, cfg(channels=16), requester="admin", is_admin=True)


def test_shutdown_destroys_everything(env):
    mgr, backend, _ = env
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    mgr.shutdown()
    assert not backend.live and mgr.list_sensors() == []


# ── budget and ownership ─────────────────────────────────────────────────────

def test_over_budget_request_gets_a_409_with_numbers(env):
    mgr, _, _ = env
    mgr.request("a", 7, cfg(pps=700_000), own_actor_id=7)
    with pytest.raises(LidarBudgetExceeded) as err:
        mgr.request("b", 8, cfg(pps=400_000), own_actor_id=8)
    assert err.value.status == 409
    assert err.value.detail == {"error": "lidar_budget_exceeded", "message": "LiDAR budget exceeded",
                                "requested_pps": 400_000, "available_pps": 300_000, "sensors": 1, "max_sensors": 2}


def test_sensor_count_limit_and_budget_meter(env):
    mgr, _, _ = env
    mgr.request("a", 7, cfg(pps=100_000), own_actor_id=7)
    mgr.request("b", 8, cfg(pps=100_000), own_actor_id=8)
    with pytest.raises(LidarBudgetExceeded):
        mgr.request("c", 9, cfg(pps=100_000), own_actor_id=9)
    assert mgr.budget() == {"used_pps": 200_000, "max_pps": 1_000_000, "sensors": 2, "max_sensors": 2}


def test_release_frees_budget(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(pps=900_000), own_actor_id=7)
    mgr.tick(2)
    mgr.release(s.sensor_id, requester="a")
    mgr.tick()
    mgr.request("b", 8, cfg(pps=900_000), own_actor_id=8)


def test_reconfigure_budget_counts_only_the_difference(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(pps=900_000), own_actor_id=7)
    mgr.tick(2)
    mgr.reconfigure(s.sensor_id, cfg(pps=1_000_000), requester="a")   # replaces its own 900k: fits exactly
    mgr.tick()
    with pytest.raises(LidarBudgetExceeded):
        mgr.reconfigure(s.sensor_id, cfg(pps=1_100_000), requester="a")
    with pytest.raises(LidarBudgetExceeded):                          # nothing left for a second sensor
        mgr.request("b", 8, cfg(pps=1_000), own_actor_id=8)


def test_vehicle_may_only_attach_to_its_own_actor_admin_to_any(env):
    mgr, _, _ = env
    with pytest.raises(LidarPermissionError):
        mgr.request("a", 99, cfg(), own_actor_id=7)
    s = mgr.request("admin", 99, cfg(), is_admin=True)
    assert s.actor_id == 99


def test_only_admin_may_pin(env):
    mgr, _, _ = env
    with pytest.raises(LidarPermissionError):
        mgr.request("a", 7, cfg(), own_actor_id=7, pinned=True)


def test_release_of_unknown_sensor_is_404(env):
    mgr, _, _ = env
    with pytest.raises(LidarNotFound):
        mgr.release(42, requester="a")


# ── auto-release ─────────────────────────────────────────────────────────────

def test_sensor_without_viewers_is_released_after_keep_alive_and_the_frame_is_logged(env):
    mgr, backend, clock = env
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    clock.t += 9.9
    mgr.tick()
    assert states(mgr) == ["active"]
    clock.t += 0.2
    mgr.tick()
    assert states(mgr) == ["stopping"]
    mgr.tick()
    assert mgr.list_sensors() == [] and not backend.live
    reasons = [e for e in mgr.events if e["state"] == "stopping"]
    assert "no viewers" in reasons[0]["reason"] and reasons[0]["frame"] == mgr.frame - 1


def test_a_viewer_keeps_the_sensor_alive_and_leaving_restarts_the_clock(env):
    mgr, _, clock = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    mgr.viewer_attached(s.sensor_id)
    clock.t += 100
    mgr.tick()
    assert states(mgr) == ["active"]
    mgr.viewer_detached(s.sensor_id)
    clock.t += 5
    mgr.tick()
    assert states(mgr) == ["active"]
    clock.t += 6
    mgr.tick()
    assert states(mgr) == ["stopping"]


def test_pinned_and_recording_sensors_are_never_auto_released(env):
    mgr, _, clock = env
    mgr.request("admin", 7, cfg(pps=100_000), is_admin=True, pinned=True)
    mgr.request("b", 8, cfg(pps=100_000), own_actor_id=8, record_raw=True)
    mgr.tick(2)
    clock.t += 1000
    mgr.tick(3)
    assert states(mgr) == ["active", "active"]


# ── streaming and profiles ───────────────────────────────────────────────────

def test_two_viewers_on_one_profile_get_the_identical_bytes_object(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(pps=200_000), own_actor_id=7)
    mgr.tick(2)
    prof = StreamProfile(max_points=None)
    f1, b1 = mgr.get_stream_frame(s.sensor_id, prof)
    f2, b2 = mgr.get_stream_frame(s.sensor_id, prof)
    assert f1 == f2 == mgr.frame and b1 is b2                          # encoded once, shared


def test_stream_frames_carry_the_ticked_frame_pose_and_ids(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    frame, data = mgr.get_stream_frame(s.sensor_id)
    h, pts = protocol.decode(data)
    assert h.frame == frame == mgr.frame and h.sensor_id == s.sensor_id and h.ego_actor_id == 7
    assert h.location == (1.0, 2.0, 3.0) and h.rotation == (0.0, 90.0, 0.0) and h.sensor_frame
    assert len(pts) == cfg().points_per_frame(DT)


def test_no_frame_is_returned_twice_and_none_before_the_first(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    assert mgr.get_stream_frame(s.sensor_id) is None
    mgr.tick(2)
    frame, _ = mgr.get_stream_frame(s.sensor_id)
    assert mgr.get_stream_frame(s.sensor_id, after_frame=frame) is None


def test_profile_quantisation():
    q = StreamProfile.quantise
    assert q(8_000, 20, False, 20.0).max_points == 10_000
    assert q(30_000, None, False, 20.0).max_points == 50_000
    assert q(60_000, None, False, 20.0).max_points is None
    assert q(None, 10, False, 20.0).every_n_frames == 2
    assert q(None, 7, False, 20.0).every_n_frames == 3                # 20/3 = 6.7 Hz <= 7
    assert q(None, 100, False, 20.0).every_n_frames == 1


def test_max_hz_profile_only_gets_due_frames(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    prof = StreamProfile(every_n_frames=2)
    got = []
    for _ in range(6):
        mgr.tick()
        r = mgr.get_stream_frame(s.sensor_id, prof)
        if r:
            got.append(r[0])
    assert got and all(f % 2 == 0 for f in got)


def test_subsampling_is_seeded_random_not_a_stride_and_stable(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(pps=400_000), own_actor_id=7)     # 20k points per frame
    mgr.tick(2)
    prof = StreamProfile(max_points=10_000)
    _, data = mgr.get_stream_frame(s.sensor_id, prof)
    _, pts = protocol.decode(data)
    idx = pts["object_idx"].astype(int)                              # the fake sets object_idx = original index
    assert len(idx) == 10_000 and len(set(idx)) == 10_000 and idx.max() < 20_000
    assert np.all(np.diff(idx) > 0)                                  # original order kept
    assert not np.array_equal(idx, np.arange(0, 20_000, 2))          # not a stride
    mgr.tick()
    _, pts2 = protocol.decode(mgr.get_stream_frame(s.sensor_id, prof)[1])
    assert np.array_equal(pts2["object_idx"], pts["object_idx"])     # same selection every frame


def test_subsampling_is_reproducible_across_managers(env):
    def run():
        backend, clock = FakeBackend(), Clock()
        m = LidarManager(backend, LidarLimits(), DT, seed=5, wall_clock=clock, monotonic=clock)
        s = m.request("a", 7, cfg(pps=400_000), own_actor_id=7)
        m.on_tick(1, 0.05); m.on_tick(2, 0.1)
        return protocol.decode(m.get_stream_frame(s.sensor_id, StreamProfile(max_points=10_000))[1])[1]["object_idx"]
    assert np.array_equal(run(), run())


def test_raw_listener_receives_full_resolution_frames_while_streams_are_subsampled(env):
    mgr, _, _ = env
    seen = []
    mgr.add_raw_listener(lambda info, frame: seen.append((info["record_raw"], len(frame.measurement.points))))
    s = mgr.request("a", 7, cfg(pps=400_000), own_actor_id=7, record_raw=True)
    mgr.tick(3)
    assert seen and all(n == 20_000 for _, n in seen) and all(r for r, _ in seen)
    _, pts = protocol.decode(mgr.get_stream_frame(s.sensor_id, StreamProfile(max_points=10_000))[1])
    assert len(pts) == 10_000


# ── config validation ────────────────────────────────────────────────────────

def test_rotation_frequency_must_match_the_simulation_step():
    assert LidarConfig().resolve(0.05).rotation_frequency_hz == 20.0
    assert LidarConfig(rotation_frequency_hz=20.0).resolve(0.05).rotation_frequency_hz == 20.0
    with pytest.raises(ValueError, match="1/fixed_delta_seconds"):
        LidarConfig(rotation_frequency_hz=10.0).resolve(0.05)


def test_semantic_sensor_rejects_raycast_only_attributes():
    with pytest.raises(ValueError, match="ray_cast"):
        LidarConfig(sensor_type="ray_cast_semantic", noise_stddev=0.1)
    c = LidarConfig(sensor_type="ray_cast", noise_stddev=0.1).resolve(0.05)
    assert c.blueprint_id == "sensor.lidar.ray_cast" and c.blueprint_attributes()["noise_stddev"] == "0.1"


@pytest.mark.parametrize("kwargs", [{"channels": 0}, {"range_m": 0}, {"points_per_second": 10},
                                    {"upper_fov_deg": -40.0, "lower_fov_deg": -30.0}, {"unknown": 1},
                                    {"mount": {"z_m": 99}}])
def test_invalid_configs_are_rejected(kwargs):
    with pytest.raises(ValueError):
        LidarConfig(**kwargs)


def test_points_per_frame_follows_the_step():
    assert LidarConfig(points_per_second=500_000).points_per_frame(0.05) == 25_000
    assert Mount().z_m == 2.4


# ── fixes from the independent review ────────────────────────────────────────

def test_a_10_hz_stream_delivers_every_second_frame_whatever_the_polling_phase(env):
    """Review finding: polling after odd ticks used to deliver NOTHING for max_hz=10 (the latest frame was odd)."""
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    prof, got, last = StreamProfile(every_n_frames=2), [], -1
    for _ in range(40):
        mgr.tick(1)
        if mgr.frame % 2 == 1:                           # the poll always lands right after an odd tick
            r = mgr.get_stream_frame(s.sensor_id, prof, last)
            if r:
                got.append(r[0]); last = r[0]
    assert got and all(f % 2 == 0 for f in got) and len(got) >= 15
    assert all(b - a == 2 for a, b in zip(got, got[1:]))   # steady 10 Hz: every second simulation frame


def test_two_polling_phases_share_the_same_encoded_frame_per_profile(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(4)
    prof = StreamProfile(every_n_frames=4)
    mgr.tick(1)
    a = mgr.get_stream_frame(s.sensor_id, prof)
    mgr.tick(1)
    b = mgr.get_stream_frame(s.sensor_id, prof)
    assert a[0] == b[0] and a[1] is b[1]


def test_a_pending_reconfigure_counts_at_its_new_size_in_the_budget(env):
    """Review finding: reconfigure 200k -> 900k then a request for 800k were both accepted (1.7 M against a 1 M budget)."""
    mgr, _, _ = env
    a = mgr.request("a", 7, cfg(pps=200_000), own_actor_id=7)
    mgr.tick(2)
    mgr.reconfigure(a.sensor_id, cfg(pps=900_000), requester="a")
    assert mgr.budget()["used_pps"] == 900_000
    with pytest.raises(LidarBudgetExceeded):
        mgr.request("b", 8, cfg(pps=800_000), own_actor_id=8)


def test_budget_can_be_read_from_other_threads_while_sensors_come_and_go(env):
    """Review finding: budget() iterated the live dict and raised 'dictionary changed size' on the HTTP thread."""
    import threading
    mgr, _, _ = env
    errors, stop = [], threading.Event()

    def reader():
        while not stop.is_set():
            try:
                mgr.budget(); mgr.list_sensors()
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

    t = threading.Thread(target=reader)
    t.start()
    for i in range(300):
        s = mgr.request(f"o{i}", i, cfg(pps=10_000), own_actor_id=i)
        mgr.tick(1)
        mgr.release(s.sensor_id, requester=f"o{i}")
        mgr.tick(1)
    stop.set(); t.join()
    assert errors == []


def test_a_sensor_that_stops_delivering_is_declared_dead_and_freed(env):
    """Review finding: it stayed ACTIVE forever and stalled every tick for frame_timeout_s."""
    mgr, backend, _ = env
    mgr.limits = LidarLimits(max_sensors=2, max_total_points_per_second=1_000_000, max_consecutive_misses=5)
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    backend.drop_frames.update(range(mgr.frame + 1, mgr.frame + 50))
    mgr.tick(6)
    info = mgr.list_sensors()[0]
    assert info["state"] == "error" and "consecutive" in info["error"]
    assert mgr.budget()["used_pps"] == 0 and not backend.live


def test_a_single_missed_frame_does_not_kill_the_sensor_and_the_counter_resets(env):
    mgr, backend, _ = env
    mgr.limits = LidarLimits(max_sensors=2, max_total_points_per_second=1_000_000, max_consecutive_misses=3)
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    for gap in (1, 2, 1, 2):                              # never 3 in a row
        backend.drop_frames.update(range(mgr.frame + 1, mgr.frame + 1 + gap))
        mgr.tick(gap + 1)
    assert states(mgr) == ["active"]


def test_hold_all_keeps_viewerless_sensors_alive_for_raw_recording(env):
    mgr, _, clock = env
    mgr.hold_all = True
    mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    clock.t += 1000
    mgr.tick(3)
    assert states(mgr) == ["active"]
    mgr.hold_all = False
    mgr.tick(1)
    assert states(mgr) == ["stopping"]


def test_reconfigure_of_a_busy_sensor_is_a_409_busy_not_a_400(env):
    from src.lidar.manager import LidarBusy
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    with pytest.raises(LidarBusy) as err:
        mgr.reconfigure(s.sensor_id, cfg(), requester="a")
    assert err.value.status == 409 and err.value.detail["error"] == "lidar_busy"


def test_a_stopping_sensor_does_not_count_as_the_owners_sensor(env):
    mgr, _, _ = env
    s = mgr.request("a", 7, cfg(), own_actor_id=7)
    mgr.tick(2)
    mgr.release(s.sensor_id, requester="a")
    assert mgr.sensor_of_owner("a") is None               # a new request may replace it
    mgr.request("a", 7, cfg(), own_actor_id=7)            # budget is free again too


def test_subsampling_barely_changes_when_the_point_count_varies_between_frames(env):
    """Review finding: a permutation redrawn at a larger size changed ~40 % of the selection."""
    mgr, _, _ = env
    a = mgr._subsample_indices(1, 20_000, 10_000)
    b = mgr._subsample_indices(1, 20_001, 10_000)
    assert len(set(a) & set(b)) >= 9_990
    assert len(a) == 10_000 and len(set(a)) == 10_000 and a.max() < 20_000


def test_subsampling_is_independent_between_sensors_and_seeds(env):
    mgr, _, _ = env
    assert not np.array_equal(mgr._subsample_indices(1, 5_000, 1_000), mgr._subsample_indices(2, 5_000, 1_000))
    other = LidarManager(FakeBackend(), LidarLimits(), DT, seed=99)
    assert not np.array_equal(mgr._subsample_indices(1, 5_000, 1_000), other._subsample_indices(1, 5_000, 1_000))


def test_encode_cache_does_not_thrash_between_sensors_at_different_frames(env):
    mgr, backend, _ = env
    a = mgr.request("a", 7, cfg(pps=100_000), own_actor_id=7)
    b = mgr.request("b", 8, cfg(pps=100_000), own_actor_id=8)
    mgr.tick(3)
    backend.drop_frames.add(mgr.frame + 1)               # both miss one frame, then both catch up: latest frames can differ per sensor
    mgr.tick(1)
    p = StreamProfile()
    fa = mgr.get_stream_frame(a.sensor_id, p)
    fb = mgr.get_stream_frame(b.sensor_id, p)
    again_a = mgr.get_stream_frame(a.sensor_id, p)
    again_b = mgr.get_stream_frame(b.sensor_id, p)
    assert (fa is None) == (again_a is None) and (fb is None) == (again_b is None)
    if fa and again_a:
        assert fa[1] is again_a[1]                       # still the cached bytes object
