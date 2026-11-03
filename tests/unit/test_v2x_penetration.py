"""S3 / D2: background vehicles become V2X stations with probability ``v2x_penetration_rate``."""

import importlib
import sys
from types import SimpleNamespace as NS

import random

import pytest

from tests.unit.carla_stub import Recorder, make_carla_module


@pytest.fixture
def pw(monkeypatch):
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(Recorder()))
    sys.modules.pop("src.server.persistent_world", None)
    return importlib.import_module("src.server.persistent_world")


class _Net:
    def __init__(self):
        self.registered = []

    def register(self, vid, actor):
        self.registered.append(vid)


def _server(rate, net, seed=42):
    return NS(v2x_network=net, config=NS(v2x_penetration_rate=rate), _v2x_rng=random.Random(seed + 1))


def _register_all(pw, srv, n=1000):
    for i in range(n):
        pw.PersistentCARLAServer._maybe_register_background_v2x(srv, NS(id=i))


def test_full_penetration_registers_every_vehicle(pw):
    net = _Net()
    _register_all(pw, _server(1.0, net), 50)
    assert net.registered == list(range(50))


def test_zero_penetration_registers_none(pw):
    net = _Net()
    _register_all(pw, _server(0.0, net), 50)
    assert net.registered == []


def test_partial_penetration_is_close_to_rate_and_reproducible(pw):
    a, b = _Net(), _Net()
    _register_all(pw, _server(0.3, a))
    _register_all(pw, _server(0.3, b))
    assert a.registered == b.registered
    assert 250 <= len(a.registered) <= 350


def test_no_v2x_network_is_a_no_op(pw):
    _register_all(pw, _server(1.0, None), 5)
