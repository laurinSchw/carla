"""V2XFrame: the immutable snapshot of one update that the hub and recorder read (ids, inbox, threats, counts)."""

from src.v2x.frame import HAZARD_STATION_BASE, build_v2x_frame, count_links, hazard_station_id, rsu_station_id
from tests.v2x.test_infrastructure_links import _Snapshot, _Walker, _vehicle, _world


def _frame(net, snap, f=7, t=1.5):
    net.update(_Snapshot(snap._snaps, t=t, frame=f), force=True)
    return build_v2x_frame(net, f, t)


def test_stations_are_ground_truth_with_negative_ids_for_rsus_and_hazards():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20.0, 60.0))
    net.update(snap, force=True)
    hz = net.publish_hazard((30.0, 5.0, 0.0), relevance_distance_m=100.0)
    fr = _frame(net, snap)
    assert fr.stations[1].kind == "OBU" and fr.stations[1].x_m == 20.0
    assert fr.stations[rsu_station_id(5)].kind == "RSU" and rsu_station_id(5) == -5
    assert fr.stations[hazard_station_id(hz.station_id)].kind == "RSU" and hazard_station_id(hz.station_id) < 0
    assert all(sid < 0 for sid in fr.stations if sid not in (1, 2))


def test_inbox_holds_what_was_received_with_the_link_that_delivered_it():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20.0, 60.0))
    fr = _frame(net, snap)
    got = {(m.tx_id, m.msg_type) for m in fr.inbox[1]}
    assert (2, "BSM") in got and (rsu_station_id(5), "SPAT") in got and (rsu_station_id(5), "MAP") in got
    for m in fr.inbox[1]:
        assert m.link.delivered and m.link.rx_id == 1 and m.link.tx_id == m.tx_id and m.message is not None


def test_links_carry_the_converted_ids_but_the_network_keeps_its_raw_records():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20.0,))
    fr = _frame(net, snap)
    assert {l.tx_id for l in fr.links if l.msg_type == "SPAT"} == {-5}
    assert {l.tx_id for l in net.get_link_results() if l.msg_type == "SPAT"} == {5}


def test_hazard_warning_reaches_the_inbox_under_its_virtual_station_id():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0,))
    net.update(snap, force=True)
    hz = net.publish_hazard((30.0, 0.0, 0.0), relevance_distance_m=100.0)
    fr = _frame(net, snap)
    vs = hazard_station_id(hz.station_id)
    assert any(m.tx_id == vs and m.msg_type == "DENM" for m in fr.inbox[1])
    assert hz.station_id >= HAZARD_STATION_BASE and fr.sent[vs][0].station_id == hz.station_id


def test_channel_counts_group_outcomes_per_link_type_and_add_up():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20.0, 60.0, 5000.0))
    fr = _frame(net, snap)
    for lt, c in fr.stats.by_link_type.items():
        assert c.total == c.delivered + c.channel_drops + c.mac_drops + c.awareness_drops, lt
    assert sum(c.total for c in fr.stats.by_link_type.values()) == len(fr.links)
    assert fr.stats.by_link_type["V2V"].channel_drops + fr.stats.by_link_type["V2V"].awareness_drops > 0   # vehicle 3 is far away


def test_threats_are_listed_per_receiver_from_the_networks_assessment():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 40.0))
    net.update(snap, force=True)
    net.threats[(1, 2)] = {"level": 3, "ttc": 2.5, "distance": 20.0}
    fr = build_v2x_frame(net, 7, 1.5)
    (t,) = fr.threats[1]
    assert (t.other_id, t.level, t.ttc_s, t.distance_m) == (2, 3, 2.5, 20.0)


def test_a_frame_is_immutable_and_independent_of_later_updates():
    net, snap = _world(rsu_xy=(0.0, 0.0), vehicle_x=(20.0, 60.0))
    fr = _frame(net, snap)
    n = len(fr.links)
    net.update(_Snapshot({1: snap._snaps[1]}, t=3.0, frame=9), force=True)         # vehicle 2 disappears
    assert len(fr.links) == n and 2 in fr.stations
    try:
        fr.stations[99] = None
        raise AssertionError("frame stations must be read-only")
    except TypeError:
        pass


def test_count_links_on_nothing_is_empty():
    assert count_links([]) == {}
