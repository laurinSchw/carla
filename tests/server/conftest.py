"""Fixtures for tests against a RUNNING, deployed server (docker or bare metal).

Configuration (environment):
    SERVER_URL    base URL of the web server        e.g. http://192.168.0.50:8000
    ADMIN_TOKEN   the server's admin token          (admin tests skip without it)
    CARLA_HOST    optional: if the server is not yet connected to CARLA, connect it to
    CARLA_PORT    this host/port (default port 2000) before the tests that need a world

Without SERVER_URL every test here is skipped, so the normal offline run is unaffected.
Tests that need a running CARLA simulation are also marked ``carla``.
"""

import http.client
import json
import os
import time
import urllib.error
import urllib.request

import pytest


def pytest_collection_modifyitems(items):
    for item in items:
        if "tests/server" in str(item.fspath).replace("\\", "/"):
            item.add_marker(pytest.mark.server)


class Server:
    def __init__(self, base_url: str, token: str):
        self.base = base_url.rstrip("/")
        self.token = token

    def request(self, method, path, body=None, admin=False, token=None, timeout=15):
        data = json.dumps(body).encode() if body is not None else None
        headers = {"Content-Type": "application/json"} if data else {}
        if admin:
            headers["X-Admin-Token"] = token if token is not None else self.token
        req = urllib.request.Request(self.base + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                raw = r.read()
                return r.status, r.headers.get("content-type", ""), raw
        except urllib.error.HTTPError as e:
            return e.code, e.headers.get("content-type", ""), e.read()
        except (http.client.RemoteDisconnected, ConnectionResetError) as exc:
            pytest.fail(f"{method} {path}: server closed the connection without answering "
                        f"({exc.__class__.__name__}). The server probably crashed or hung in that "
                        f"handler; check its logs (docker compose logs --tail=100 frontend).")

    def json(self, method, path, body=None, admin=False, **kw):
        status, _, raw = self.request(method, path, body, admin, **kw)
        try:
            return status, json.loads(raw or b"null")
        except ValueError:
            return status, raw

    @property
    def ws_base(self):
        return self.base.replace("https://", "wss://").replace("http://", "ws://")


@pytest.fixture(scope="session")
def server():
    url = os.environ.get("SERVER_URL")
    if not url:
        pytest.skip("SERVER_URL not set (e.g. SERVER_URL=http://localhost:8000)")
    srv = Server(url, os.environ.get("ADMIN_TOKEN", ""))
    try:
        status, _, _ = srv.request("GET", "/", timeout=10)
    except OSError as exc:
        pytest.fail(f"Cannot reach server at {url}: {exc}")
    assert status == 200, f"GET / returned {status}"
    return srv


@pytest.fixture(scope="session")
def admin(server):
    if not server.token:
        pytest.skip("ADMIN_TOKEN not set")
    status, _ = server.json("GET", "/admin/status", admin=True)
    if status == 403:
        pytest.fail("ADMIN_TOKEN does not match the server's token (403)")
    assert status == 200
    return server


@pytest.fixture(scope="session")
def live_world(admin):
    """A server whose CARLA simulation is running. Connects it if CARLA_HOST is given."""
    status, body = admin.json("GET", "/admin/status", admin=True)
    if not body.get("running"):
        host = os.environ.get("CARLA_HOST")
        if not host:
            pytest.skip("server is not connected to CARLA (set CARLA_HOST/CARLA_PORT to connect it)")
        port = int(os.environ.get("CARLA_PORT", "2000"))
        code, resp = admin.json("POST", "/admin/connect",
                                {"carla_host": host, "carla_port": port}, admin=True)
        assert code == 200, f"/admin/connect failed: {code} {resp}"
        deadline = time.time() + 120
        while time.time() < deadline:
            _, body = admin.json("GET", "/admin/status", admin=True)
            if body.get("running"):
                break
            time.sleep(2)
        else:
            pytest.fail(f"server did not reach running state connecting to {host}:{port} within 120 s")
    return admin
