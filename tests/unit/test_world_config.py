import pytest
from pydantic import ValidationError

from src.server.world_config import WorldConfig


def test_defaults_match_pre_refactor_behaviour():
    c = WorldConfig(host="h")
    assert (c.port, c.fixed_delta_seconds, c.tm_port, c.seed) == (2000, 0.05, 8001, 42)
    assert c.global_speed_pct == -20.0 and c.safety_distance_m == 2.5
    assert c.v2x_range_m == 75.0 and c.v2x_every_n_frames == 10


@pytest.mark.parametrize("kwargs", [
    {"fixed_delta_seconds": 0.2},                       # > 0.1 cap
    {"fixed_delta_seconds": 0.05, "max_substeps": 2},   # substepping rule: 0.05 > 0.01*2
    {"tm_port": 2000}, {"tm_port": 2001}, {"tm_port": 8000},
    {"v2x_update_rate_hz": 3.0},                        # 1/(3*0.05) = 6.67 frames
    {"v2x_update_rate_hz": 0.0},
    {"port": 0}, {"v2x_range_m": 5.0}, {"unknown_field": 1},
])
def test_invalid_configs_are_rejected(kwargs):
    with pytest.raises(ValidationError):
        WorldConfig(host="h", **kwargs)


@pytest.mark.parametrize("rate,n", [(2.0, 10), (4.0, 5), (10.0, 2), (1.0, 20), (20.0, 1)])
def test_v2x_every_n_frames(rate, n):
    assert WorldConfig(host="h", v2x_update_rate_hz=rate).v2x_every_n_frames == n


def test_config_is_immutable():
    with pytest.raises(ValidationError):
        WorldConfig(host="h").port = 1
