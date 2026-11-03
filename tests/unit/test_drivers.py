"""Driver profiles and the V2X advisory (pure logic, no CARLA)."""

from collections import Counter
from types import SimpleNamespace as NS

import pytest

from src.traffic.advisory import AdvisoryConfig, V2XAdvisory
from src.traffic.drivers import PROFILES, DriverAssigner, DriverMix, SPEED_JITTER_PCT, apply_profile


def draw(seed, n=300, mix=DriverMix()):
    a = DriverAssigner(mix, seed)
    return [a.next_profile() for _ in range(n)]


def test_same_seed_gives_the_same_drivers_and_a_different_seed_does_not():
    assert draw(7) == draw(7)
    assert draw(7) != draw(8)


def test_mix_proportions_follow_the_weights():
    c = Counter(p.name for p in draw(1, n=4000, mix=DriverMix(cautious=0.2, normal=0.6, aggressive=0.2)))
    assert c["normal"] / 4000 == pytest.approx(0.6, abs=0.03)
    assert c["cautious"] / 4000 == pytest.approx(0.2, abs=0.03) and c["aggressive"] / 4000 == pytest.approx(0.2, abs=0.03)


def test_mix_can_be_a_single_archetype_and_weights_are_validated():
    assert {p.name for p in draw(1, n=50, mix=DriverMix(0.0, 1.0, 0.0))} == {"normal"}
    with pytest.raises(ValueError):
        DriverMix(0.0, 0.0, 0.0).weights()
    with pytest.raises(ValueError):
        DriverMix(-1.0, 1.0, 1.0).weights()


def test_vehicles_of_one_archetype_still_differ_within_the_jitter():
    normals = [p for p in draw(3, n=500) if p.name == "normal"]
    speeds = {p.speed_pct for p in normals}
    assert len(speeds) > 50
    assert all(abs(p.speed_pct - PROFILES["normal"].speed_pct) <= SPEED_JITTER_PCT for p in normals)


def test_archetypes_are_ordered_as_a_driver_would_expect():
    c, n, a = PROFILES["cautious"], PROFILES["normal"], PROFILES["aggressive"]
    assert c.speed_pct > n.speed_pct > a.speed_pct          # positive = slower than the limit
    assert c.follow_distance_m > n.follow_distance_m > a.follow_distance_m
    assert c.lane_change_left_pct < n.lane_change_left_pct < a.lane_change_left_pct


class TM:
    def __init__(self, fail=()):
        self.calls, self.fail = [], set(fail)

    def __getattr__(self, name):
        def call(actor, value):
            if name in self.fail:
                raise RuntimeError("not in this CARLA build")
            self.calls.append((name, actor.id, value))
        return call


def test_apply_profile_sets_every_setting_on_that_vehicle():
    tm = TM()
    failed = apply_profile(tm, NS(id=5), PROFILES["aggressive"])
    assert failed == [] and {c[0] for c in tm.calls} == set(PROFILES["aggressive"].tm_settings())
    assert all(c[1] == 5 for c in tm.calls)
    assert ("vehicle_percentage_speed_difference", 5, -25.0) in tm.calls


def test_a_missing_tm_method_is_reported_not_raised():
    failed = apply_profile(TM(fail={"keep_right_rule_percentage"}), NS(id=1), PROFILES["normal"])
    assert failed == ["keep_right_rule_percentage"]


# ── advisory ─────────────────────────────────────────────────────────────────

def adv(**kw):
    return V2XAdvisory(AdvisoryConfig(enabled=True, trigger_level=3, hold_s=3.0, **kw))


def test_advisory_is_silent_when_disabled():
    a = V2XAdvisory(AdvisoryConfig(enabled=False))
    assert a.update(0.0, {1: 4}) == [] and a.active == 0


def test_threat_triggers_one_slowdown_and_then_a_restore_after_the_hold_time():
    a = adv()
    assert a.update(10.0, {1: 4, 2: 1}) == [(1, "slow")]
    assert a.update(10.5, {1: 4}) == []                              # still threatened: no duplicate action
    assert a.update(12.0, {1: 0}) == []                              # inside the hold window
    assert a.update(13.6, {1: 0}) == [(1, "restore")]                # 3 s after the last trigger (10.5)
    assert a.active == 0 and a.activations == 1


def test_a_renewed_threat_extends_the_hold_and_a_new_threat_after_restore_slows_again():
    a = adv()
    a.update(0.0, {1: 3})
    a.update(2.5, {1: 3})
    assert a.update(5.0, {1: 0}) == []                               # hold now runs to 5.5
    assert a.update(5.6, {1: 0}) == [(1, "restore")]
    assert a.update(6.0, {1: 3}) == [(1, "slow")] and a.activations == 2


def test_below_trigger_level_does_nothing_and_forget_clears_state():
    a = adv()
    assert a.update(0.0, {1: 2}) == []
    a.update(1.0, {1: 4})
    a.forget(1)
    assert a.active == 0
