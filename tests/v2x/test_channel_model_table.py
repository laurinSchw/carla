"""Pins the numbers published in docs/science/model.md (handoff S1): docs and code must not drift apart."""

import math

import pytest

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig, _L0_DB


def _prx(cfg, d, nlos):
    n = cfg.path_loss_exponent_nlos if nlos else cfg.path_loss_exponent_los
    return cfg.tx_power_dbm + 2 * cfg.antenna_gain_dbi - (_L0_DB + 10 * n * math.log10(d / 10.0))


def _expected_p_channel(cfg, d, nlos):
    """E over the shadowing Gaussian of the logistic channel term (trapezoid on +-6 sigma)."""
    sigma = cfg.shadowing_std_nlos_db if nlos else cfg.shadowing_std_los_db
    num = den = 0.0
    for i in range(2001):
        z = -6 + 12 * i / 2000
        w = math.exp(-z * z / 2)
        margin = _prx(cfg, d, nlos) - z * sigma - cfg.rx_sensitivity_dbm
        num += w / (1 + math.exp(-cfg.prr_sigmoid_k * margin))
        den += w
    return num / den


# d, P_rx LOS, P_rx NLOS, E[p_channel] LOS, E[p_channel] NLOS  — copied from docs/science/model.md section 3
TABLE = [
    (10, -40.86, -40.86, 1.0000, 1.0000),
    (50, -56.94, -63.23, 1.0000, 0.9966),
    (100, -63.86, -72.86, 0.9999, 0.9342),
    (150, -67.91, -78.49, 0.9989, 0.7905),
    (200, -70.78, -82.49, 0.9946, 0.6223),
    (300, -74.83, -88.13, 0.9659, 0.3489),
    (500, -79.94, -95.23, 0.8182, 0.1020),
    (800, -84.63, -101.76, 0.5265, 0.0187),
    (1000, -86.86, -104.86, 0.3694, 0.0068),
]


@pytest.mark.parametrize("d,los,nlos,e_los,e_nlos", TABLE)
def test_documented_table_matches_the_code(d, los, nlos, e_los, e_nlos):
    cfg = DSRCConfig()
    assert _prx(cfg, d, False) == pytest.approx(los, abs=0.006)
    assert _prx(cfg, d, True) == pytest.approx(nlos, abs=0.006)
    assert _expected_p_channel(cfg, d, False) == pytest.approx(e_los, abs=0.00006)
    assert _expected_p_channel(cfg, d, True) == pytest.approx(e_nlos, abs=0.00006)


def test_link_result_matches_the_documented_equations_without_shadowing():
    cfg = DSRCConfig(shadowing_std_los_db=0.0, enable_nlos_model=False)
    link = DSRCChannel(cfg).evaluate_link(1, 2, (0.0, 0.0), (100.0, 0.0), False, 1)
    assert link.rx_power_dbm == pytest.approx(-63.86, abs=0.006)
    assert link.rx_margin_db == pytest.approx(-63.86 + 85.0, abs=0.006)
    assert link.p_channel == pytest.approx(1 / (1 + math.exp(-2.0 * link.rx_margin_db)))
    assert link.shadowing_db == 0.0 and link.los is True


def test_documented_mac_values():
    ch = DSRCChannel()
    got = [round(ch._csma_success_prob(n), 3) for n in (1, 20, 50, 100, 200, 400)]
    assert got == [1.0, 0.909, 0.783, 0.610, 0.370, 0.350]


def test_range_at_sensitivity_floor():
    cfg = DSRCConfig()
    for nlos, expected in ((False, 830), (True, 240)):
        n = cfg.path_loss_exponent_nlos if nlos else cfg.path_loss_exponent_los
        d = 10 * 10 ** ((cfg.tx_power_dbm + 2 * cfg.antenna_gain_dbi - _L0_DB - cfg.rx_sensitivity_dbm) / (10 * n))
        assert d == pytest.approx(expected, abs=5)
