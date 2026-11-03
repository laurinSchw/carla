"""S4: MAP, SPAT and VRUAM go through the same DSRC channel as BSMs (REFACTOR_HANDOFF §3.2)."""

from types import SimpleNamespace as NS

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig
from src.v2x.network import V2XNetwork
from src.v2x.rsu import RoadSideUnit, RSUConfig


def _vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _vehicle(vid, x, y=0.0):
    snap = NS(
        get_transform=lambda: NS(location=_vec(x, y), rotation=NS(yaw=0.0)),
        get_velocity=lambda: _vec(5.0),
        get_angular_velocity=lambda: _vec(),
        get_acceleration=lambda: _vec(),
    )
    actor = NS(
        id=vid, type_id="vehicle.tesla.model3", attributes={"number_of_wheels": "4"},
        bounding_box=NS(extent=_vec(2.3, 0.9, 0.75)),
        get_transform=snap.get_transform, get_velocity=snap.get_velocity,
        get_angular_velocity=snap.get_angular_velocity,
        get_control=lambda: NS(steer=0.0, brake=0.0, throttle=0.2, reverse=False),
        get_physics_control=lambda: NS(wheels=[NS(max_steer_angle=70.0)]),
        get_world=lambda: NS(),
    )
    return actor, snap


class _Snapshot:
    def __init__(self, snaps, t=1.0, frame=7):
        self._snaps, self.frame, self.timestamp = snaps, frame, NS(elapsed_seconds=t)

    def find(self, i):
        return self._snaps.get(i)


class _Walker:
    id = 900

    def get_transform(self):
        return NS(location=_vec(30.0, 0.0), rotation=NS(yaw=0.0))

    def get_velocity(self):
        return _vec(1.0)


def _quiet_channel(**kw):
    # no shadowing: link outcomes depend on distance and tx power only
    return DSRCConfig(shadowing_std_los_db=0.0, shadowing_std_nlos_db=0.0, enable_nlos_model=False, **kw)


def _world(max_range=None, rsu_xy=(0.0, 0.0), rsu_power=33.0, vehicle_x=(20.0,)):
    net = V2XNetwork(max_range=max_range, dsrc_config=_quiet_channel())
    snaps = {}
    for i, x in enumerate(vehicle_x, start=1):
        actor, snap = _vehicle(i, x)
        net.register(i, actor)
        snaps[i] = snap
    rsu = RoadSideUnit(RSUConfig(intersection_id=5, position=(*rsu_xy, 0.0), tx_power_dbm=rsu_power))
    net.register_rsu(rsu)
    return net, _Snapshot(snaps)


def test_map_and_spat_arrive_in_the_vehicle_inbox_with_link_records():
    net, snap = _world()
    net.update(snap, force=True)
    assert 5 in net.map_inbox[1] and 5 in net.spat_inbox[1]
    links = [l for l in net.get_link_results() if l.link_type == "I2V"]
    assert {l.msg_type for l in links} == {"MAP", "SPAT"}
    assert all(l.tx_id == 5 and l.rx_id == 1 and l.frame == 7 and l.sim_time == 1.0 for l in links)


def test_rsu_uses_its_own_tx_power():
    net, snap = _world(rsu_power=33.0)
    net.update(snap, force=True)
    rsu_link = next(l for l in net.get_link_results() if l.msg_type == "SPAT")
    bsm_links = [l for l in net.get_link_results() if l.msg_type == "BSM"]
    assert rsu_link.tx_power_dbm == 33.0
    assert all(l.tx_power_dbm == 23.0 for l in bsm_links) or bsm_links == []
    # +10 dB vs the 23 dBm default at the same distance (no shadowing)
    other, snap2 = _world(rsu_power=23.0)
    other.update(snap2, force=True)
    weak = next(l for l in other.get_link_results() if l.msg_type == "SPAT")
    assert abs((rsu_link.rx_power_dbm - weak.rx_power_dbm) - 10.0) < 1e-9


def test_far_rsu_is_lost_by_the_radio_not_by_a_radius_test():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20_000.0,))
    net.update(snap, force=True)
    spat = next(l for l in net.get_link_results() if l.msg_type == "SPAT")
    assert not spat.delivered and spat.drop_cause == "channel_or_mac" and spat.rx_margin_db < -20
    assert net.spat_inbox[1] == {}


def test_awareness_range_filters_after_the_radio_and_is_logged():
    net, snap = _world(max_range=10.0, vehicle_x=(20.0,))  # radio is fine at 20 m, filter says 10 m
    net.update(snap, force=True)
    spat = next(l for l in net.get_link_results() if l.msg_type == "SPAT")
    assert spat.p_total > 0.9 and not spat.delivered and spat.drop_cause == "out_of_awareness_range"
    assert net.spat_inbox[1] == {}


def test_vru_beacons_go_through_the_channel_as_p2v():
    net, snap = _world(vehicle_x=(0.0,))
    net.register_vru(900, _Walker())
    net.update(snap, force=True)
    link = next(l for l in net.get_link_results() if l.msg_type == "VRUAM")
    assert (link.link_type, link.tx_id, link.rx_id) == ("P2V", 900, 1)
    assert 900 in net.vruam_inbox[1] or not link.delivered


def test_inboxes_do_not_keep_messages_from_earlier_updates():
    net, snap = _world()
    net.update(snap, force=True)
    assert net.spat_inbox[1]
    net.unregister_rsu(5)
    net.update(snap, force=True)
    assert net.spat_inbox[1] == {} and net.map_inbox[1] == {}


def test_every_message_type_has_the_same_record_shape_and_is_reproducible():
    def run():
        net, snap = _world(vehicle_x=(20.0, 120.0, 400.0))
        net.register_vru(900, _Walker())
        net.update(snap, force=True)
        return [(l.msg_type, l.tx_id, l.rx_id, l.delivered, round(l.rx_power_dbm, 9)) for l in net.get_link_results()]

    assert run() == run()
    assert {r[0] for r in run()} == {"BSM", "MAP", "SPAT", "VRUAM"}


def test_channel_evaluate_link_matches_broadcast_all_statistics():
    ch = DSRCChannel(_quiet_channel())
    link = ch.evaluate_link(1, 2, (0.0, 0.0), (50.0, 0.0), False, 1)
    assert link.p_total == link.p_channel * link.p_mac
    assert link.rx_margin_db == link.rx_power_dbm - ch.config.rx_sensitivity_dbm
    assert ch.stats["total_broadcasts"] == 1


def test_vectorised_nlos_matches_the_scalar_rule_on_random_and_degenerate_layouts():
    """nlos_to_receivers() replaced a per-pair Python loop (O(N^3) per update); it must give the same answers."""
    import random

    ch = DSRCChannel(DSRCConfig())
    rnd = random.Random(7)
    for trial in range(30):
        n = rnd.randint(2, 40)
        pos = {i: (rnd.uniform(-30, 30), rnd.uniform(-30, 30)) for i in range(n)}
        if trial % 3 == 0:                       # degenerate: co-located stations and stations on a line
            pos[1] = pos[0]
            for i in range(2, min(n, 8)):
                pos[i] = (float(i) * 3.0, 0.0)
        for sender in range(n):
            fast = ch.nlos_to_receivers(sender, pos[sender], pos)
            for rx in range(n):
                if rx == sender:
                    continue
                assert fast[rx] == ch._is_nlos(sender, rx, pos[sender], pos[rx], pos), (trial, sender, rx)


def test_infrastructure_sender_with_a_colliding_id_does_not_exclude_a_vehicle_blocker():
    ch = DSRCChannel(DSRCConfig())
    vehicles = {1: (50.0, 0.0), 2: (25.0, 0.0), 3: (100.0, 0.0)}   # vehicle 2 sits exactly between RSU "1" and vehicle 3
    fast = ch.nlos_to_receivers(None, (0.0, 0.0), vehicles)
    assert fast[3] is True and fast[2] is False


# ── hazard warnings (DENM) through the channel ───────────────────────────────

from src.v2x.network import HAZARD_STATION_BASE  # noqa: E402
from src.v2x.safety_messages import DENM, DENMCauseCode  # noqa: E402


def test_an_admin_hazard_is_sent_through_the_channel_and_lands_in_the_inbox_of_vehicles_that_received_it():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 80.0))
    net.update(snap, force=True)                                           # establishes sim time
    hz = net.publish_hazard((50.0, 0.0, 0.0), DENMCauseCode.ACCIDENT, duration_s=30.0, relevance_distance_m=120.0)
    assert hz.station_id >= HAZARD_STATION_BASE and net.get_hazards() == [hz]
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    links = [l for l in net.get_link_results() if l.msg_type == "DENM"]
    assert {(l.link_type, l.tx_id) for l in links} == {("I2V", hz.station_id)} and {l.rx_id for l in links} == {1, 2}
    assert all(l.tx_power_dbm == 33.0 for l in links)
    for rx in (1, 2):
        assert (hz.station_id, 0) in net.denm_inbox[rx]


def test_a_hazard_far_beyond_the_radio_range_is_lost_by_the_radio_not_delivered():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20_000.0,))
    net.update(snap, force=True)
    net.publish_hazard((0.0, 0.0, 0.0))
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    (link,) = [l for l in net.get_link_results() if l.msg_type == "DENM"]
    assert not link.delivered and link.drop_cause == "channel_or_mac" and net.denm_inbox[1] == {}


def test_a_hazard_outside_the_awareness_range_is_logged_as_filtered():
    net, snap = _world(max_range=10.0, rsu_xy=(500.0, 500.0), vehicle_x=(60.0,))
    net.update(snap, force=True)
    net.publish_hazard((0.0, 0.0, 0.0))
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    (link,) = [l for l in net.get_link_results() if l.msg_type == "DENM"]
    assert link.drop_cause == "out_of_awareness_range" and net.denm_inbox[1] == {}


def test_hazards_expire_after_their_validity_and_cancel_removes_them_at_once():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0,))
    net.update(snap, force=True)
    short = net.publish_hazard((10.0, 0.0, 0.0), duration_s=2.0)
    keep = net.publish_hazard((10.0, 0.0, 0.0), duration_s=100.0)
    net.update(_Snapshot(snap._snaps, t=2.5), force=True)
    assert (short.station_id, 0) in net.denm_inbox[1]
    net.update(_Snapshot(snap._snaps, t=4.0), force=True)                  # 1.0 + 2.0 + ... : short is over
    assert [h.station_id for h in net.get_hazards()] == [keep.station_id]
    assert (short.station_id, 0) not in net.denm_inbox[1]
    assert net.cancel_denm(keep.station_id, 0) and net.get_hazards() == []
    net.update(_Snapshot(snap._snaps, t=5.0), force=True)
    assert net.denm_inbox[1] == {}


def test_a_vehicle_originated_denm_goes_through_the_channel_as_v2v_to_the_others_but_not_back_to_itself():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 60.0))
    net.update(snap, force=True)
    net.publish_denm(DENM(station_id=1, action_id=5, detection_time=1.0, reference_time=1.0, event_position=(20.0, 0.0, 0.0),
                          cause_code=DENMCauseCode.VEHICLE_BREAKDOWN))
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    links = [l for l in net.get_link_results() if l.msg_type == "DENM"]
    assert [(l.link_type, l.tx_id, l.rx_id) for l in links] == [("V2V", 1, 2)]
    assert (1, 5) in net.denm_inbox[2] and net.denm_inbox[1] == {}


def test_denms_without_validity_never_expire_so_existing_users_are_unaffected():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0,))
    net.update(snap, force=True)
    net.publish_denm(DENM(station_id=1, action_id=1, detection_time=0.0, reference_time=0.0, event_position=(0, 0, 0)))
    net.update(_Snapshot(snap._snaps, t=10_000.0), force=True)
    assert (1, 1) in net.denm_store


def test_hazard_relevance_needs_range_and_movement_towards_the_event():
    from src.traffic.advisory import hazard_is_relevant as rel
    assert rel((0, 0), (10, 0), (50, 0), 100)                       # approaching, in range
    assert not rel((0, 0), (-10, 0), (50, 0), 100)                  # driving away
    assert not rel((0, 0), (10, 0), (150, 0), 100)                  # too far
    assert not rel((0, 0), (0.1, 0), (50, 0), 100)                  # stopped
    assert not rel((0, 0), (0, 10), (50, 0), 100)                   # perpendicular
    assert rel((0, 0), (5, 5), (30, 20), 100)


def test_every_attempt_appears_exactly_once_in_the_link_results():
    """Regression: MAP/SPAT/VRUAM links were listed twice (the channel's list was aliased and the infrastructure links added again)."""
    net, snap = _world(vehicle_x=(20.0, 60.0, 90.0))
    net.register_vru(900, _Walker())
    net.update(snap, force=True)
    keys = [(l.msg_type, l.tx_id, l.rx_id) for l in net.get_link_results()]
    assert len(keys) == len(set(keys)), "duplicate link records"
    assert sum(1 for k in keys if k[0] == "BSM") == 3 * 2            # 3 vehicles, each sends to the 2 others
    for msg in ("MAP", "SPAT", "VRUAM"):
        assert sum(1 for k in keys if k[0] == msg) == 3              # one RSU/VRU to 3 vehicles
    net.update(_Snapshot(snap._snaps, t=2.0), force=True)           # the second update starts fresh
    assert len(net.get_link_results()) == len(keys)


def test_link_results_are_complete_even_when_the_channel_does_not_record_them():
    net = V2XNetwork(max_range=None, dsrc_config=DSRCConfig(shadowing_std_los_db=0.0, enable_nlos_model=False, record_links=False))
    actor, snap = _vehicle(1, 20.0)
    net.register(1, actor)
    net.register_rsu(RoadSideUnit(RSUConfig(intersection_id=5, position=(0.0, 0.0, 0.0))))
    net.update(_Snapshot({1: snap}), force=True)
    assert sorted(l.msg_type for l in net.get_link_results()) == ["MAP", "SPAT"]
