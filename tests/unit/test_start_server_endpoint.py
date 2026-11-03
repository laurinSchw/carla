"""CARLA host/port resolution of start_server.py: flag > environment > (port only) 2000; no built-in host."""

import pytest

from start_server import resolve_carla_endpoint


def test_flags_win_over_environment():
    assert resolve_carla_endpoint("h1", 2002, {"CARLA_HOST": "h2", "CARLA_PORT": "9"}) == ("h1", 2002)


def test_environment_is_used_without_flags():
    assert resolve_carla_endpoint(None, None, {"CARLA_HOST": "h2", "CARLA_PORT": "2002"}) == ("h2", 2002)


def test_port_defaults_to_carla_default_and_host_has_no_default():
    assert resolve_carla_endpoint(None, None, {}) == (None, 2000)
    assert resolve_carla_endpoint(None, None, {"CARLA_HOST": ""}) == (None, 2000)


@pytest.mark.parametrize("port", ["abc", "0", "70000"])
def test_bad_port_is_rejected(port):
    with pytest.raises(ValueError):
        resolve_carla_endpoint("h", None, {"CARLA_PORT": port})


def test_live_without_host_exits_with_a_clear_message(monkeypatch):
    import start_server
    monkeypatch.delenv("CARLA_HOST", raising=False)
    monkeypatch.setattr("sys.argv", ["start_server.py", "--live"])
    with pytest.raises(SystemExit) as e:
        start_server.main()
    assert "CARLA_HOST" in str(e.value)
