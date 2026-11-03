"""Real asserts replacing the root-level test_scientific_fixes.py (which asserted nothing).

The CSMA values are pinned to what the code computes today (tolerance 1e-9). The CSMA
term is EXPERIMENTAL / uncalibrated (REFACTOR_HANDOFF S6); these pins detect drift, they
do not validate the model.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig

CSMA_PINNED = {
    1: 1.0,
    5: 0.9801986733067553,
    15: 0.9323938199059483,
    20: 0.9093729344682314,
    50: 0.7827045382418681,
    100: 0.6095709072963093,
    200: 0.369723444544059,
}


@pytest.mark.parametrize("n,expected", sorted(CSMA_PINNED.items()))
def test_csma_success_prob_pinned(n, expected):
    assert DSRCChannel()._csma_success_prob(n) == pytest.approx(expected, abs=1e-9)


def test_csma_bounds_and_monotonic():
    ch = DSRCChannel()
    probs = [ch._csma_success_prob(n) for n in range(1, 400)]
    assert probs[0] == 1.0
    assert all(a >= b for a, b in zip(probs, probs[1:]))
    assert min(probs) >= 0.35  # floor


def test_light_traffic_prr_above_090():
    ch = DSRCChannel()
    assert all(ch._csma_success_prob(n) > 0.90 for n in range(1, 21))

