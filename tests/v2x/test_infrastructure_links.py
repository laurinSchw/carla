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
