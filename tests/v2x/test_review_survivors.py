"""Tests for behaviours the independent review broke on purpose (mutation testing) without any test noticing."""

import random
from types import SimpleNamespace as NS

import pytest

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig
from src.v2x.network import V2XNetwork
from src.v2x.rsu import RoadSideUnit, RSUConfig
from src.v2x.safety_messages import DENM, DENMCauseCode
from tests.v2x.test_infrastructure_links import _Snapshot, _Walker, _vehicle, _world


def _two_updates(net, snaps, t1=1.0, t2=2.0):
    net.update(_Snapshot(snaps, t=t1, frame=10), force=True)
    net.update(_Snapshot(snaps, t=t2, frame=20), force=True)


def test_a_vehicle_that_vanishes_from_the_snapshot_loses_its_old_bsm():
    """S10: the previous BSM must not stay in the network (nobody would notice, it is only ever replaced)."""
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 50.0))
    net.update(snap, force=True)
    assert net.get_bsm(2) is not None
    net.update(_Snapshot({1: snap._snaps[1]}, t=2.0), force=True)            # vehicle 2 is gone from this tick's snapshot
    assert net.get_bsm(2) is None and 2 not in net.get_all_bsm()
    assert net.stats["bsm_skipped_not_in_snapshot"] == 1


def test_bsm_link_records_carry_the_snapshot_frame_and_sim_time():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 50.0))
    net.update(_Snapshot(snap._snaps, t=3.5, frame=777), force=True)
    bsm = [l for l in net.get_link_results() if l.msg_type == "BSM"]
    assert bsm and all(l.frame == 777 and l.sim_time == 3.5 for l in bsm)


def test_a_vehicle_between_an_rsu_and_a_receiver_blocks_the_infrastructure_link_too():
    """The LOS/NLOS check applied to MAP/SPAT/DENM was never tested (switching it off survived)."""
    cfg = DSRCConfig(shadowing_std_los_db=0.0, shadowing_std_nlos_db=0.0, enable_nlos_model=True)
    net = V2XNetwork(max_range=None, dsrc_config=cfg)
    snaps = {}
    for vid, x in ((1, 100.0), (2, 50.0)):                                # vehicle 2 sits on the line RSU -> vehicle 1
        actor, s = _vehicle(vid, x)
        net.register(vid, actor)
        snaps[vid] = s
    net.register_rsu(RoadSideUnit(RSUConfig(intersection_id=5, position=(0.0, 0.0, 0.0))))
    net.update(_Snapshot(snaps), force=True)
    spat = {l.rx_id: l for l in net.get_link_results() if l.msg_type == "SPAT"}
    assert spat[1].los is False and spat[2].los is True
    assert spat[1].path_loss_db > spat[2].path_loss_db                    # NLOS exponent 3.2 vs 2.3 (distance differs, loss still higher)


def test_a_vehicles_own_denm_is_sent_with_the_default_obu_power_and_hazards_use_the_rsu_power():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 60.0))
    net.update(snap, force=True)
    net.publish_denm(DENM(station_id=1, action_id=1, detection_time=1.0, reference_time=1.0, event_position=(20.0, 0.0, 0.0)))
    hz = net.publish_hazard((30.0, 0.0, 0.0), tx_power_dbm=40.0)
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    by_tx = {l.tx_id: l for l in net.get_link_results() if l.msg_type == "DENM"}
    assert by_tx[1].tx_power_dbm == 23.0 and by_tx[1].link_type == "V2V"
    assert by_tx[hz.station_id].tx_power_dbm == 40.0 and by_tx[hz.station_id].link_type == "I2V"


def test_active_hazards_take_part_in_the_contention_domain():
    net, snap = _world(rsu_xy=(500.0, 500.0), vehicle_x=(20.0, 60.0))
    net.update(snap, force=True)
    before = next(l for l in net.get_link_results() if l.msg_type == "BSM").p_mac
    for k in range(10):
        net.publish_hazard((float(k), 3.0, 0.0))
    net.update(_Snapshot(snap._snaps, t=1.5), force=True)
    after = next(l for l in net.get_link_results() if l.msg_type == "BSM").p_mac
    assert after < before


def test_the_monte_carlo_channel_term_matches_the_documented_table():
    """The table test re-implements the formula; this one goes through the production evaluate_link (300 m NLOS: 0.3489)."""
    ch = DSRCChannel(DSRCConfig(enable_nlos_model=False, random_seed=3))
    p = [ch.evaluate_link(1, 2, (0.0, 0.0), (300.0, 0.0), True, 1).p_channel for _ in range(6000)]
    assert sum(p) / len(p) == pytest.approx(0.3489, abs=0.012)
    p = [ch.evaluate_link(1, 2, (0.0, 0.0), (100.0, 0.0), True, 1).p_channel for _ in range(6000)]
    assert sum(p) / len(p) == pytest.approx(0.9342, abs=0.012)


def test_broadcast_all_updates_the_channel_statistics_it_reports():
    ch = DSRCChannel(DSRCConfig(enable_nlos_model=False))
    from src.v2x.safety_messages import BSMCore
    bsms = {i: BSMCore(timestamp=0.0, msg_count=0, vehicle_id=i, x_m=float(i * 30), y_m=0.0) for i in range(4)}
    ch.broadcast_all(bsms, {i: (b.x_m, b.y_m) for i, b in bsms.items()})
    st = ch.get_stats()
    assert st["total_broadcasts"] == 12 == len(ch.last_links)
    assert st["total_deliveries"] + st["total_drops"] == 12 and st["prr"] == st["total_deliveries"] / 12
