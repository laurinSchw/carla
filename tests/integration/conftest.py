"""Fixtures for tests that need a live CARLA 0.9.16 server (marker: carla).

Select with ``pytest -m carla``. Server address comes from the environment:
    CARLA_HOST (default localhost)   CARLA_PORT (default 2000)
Streaming port is RPC port + 1; it must be reachable too.
"""

import os

import pytest

pytestmark = pytest.mark.carla


def pytest_collection_modifyitems(items):
    for item in items:
        if "tests/integration" in str(item.fspath).replace("\\", "/"):
            item.add_marker(pytest.mark.carla)


@pytest.fixture(scope="session")
def carla_client():
    carla = pytest.importorskip("carla")
    host = os.environ.get("CARLA_HOST", "localhost")
    port = int(os.environ.get("CARLA_PORT", "2000"))
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    try:
        client.get_server_version()
    except RuntimeError as exc:
        pytest.fail(f"Cannot reach CARLA at {host}:{port}: {exc}")
    return client
