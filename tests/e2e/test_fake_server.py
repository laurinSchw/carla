"""``start_server.py --fake``: the real server (HTTP, hub WebSocket, V2X, LiDAR) runs on the synthetic world, no CARLA.

A subprocess on a free port, driven over HTTP and WebSocket, then stopped with SIGINT (as AGENTS.md asks for servers).
"""

import asyncio
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

httpx = pytest.importorskip("httpx")
websockets = pytest.importorskip("websockets")

ROOT = Path(__file__).resolve().parents[2]
TOKEN = "fake-e2e-token"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    port = _free_port()
    log = tmp_path_factory.mktemp("fake_server") / "server.log"
    with open(log, "w") as out:
        proc = subprocess.Popen([sys.executable, "start_server.py", "--fake", "--fake-vehicles", "6", "--fake-pedestrians", "3",
                                 "--host", "127.0.0.1", "--port", str(port), "--admin-token", TOKEN],
                                cwd=ROOT, stdout=out, stderr=subprocess.STDOUT, env={**os.environ, "PYTHONUNBUFFERED": "1"})
    base = f"http://127.0.0.1:{port}"
    try:
        deadline = time.monotonic() + 60
        while True:
            try:
                if httpx.get(base + "/api/status", timeout=2).json().get("connected"):
                    break
            except Exception:
                pass
            assert proc.poll() is None and time.monotonic() < deadline, log.read_text()[-3000:]
            time.sleep(0.3)
        yield base, port, log
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            proc.kill()
    assert proc.returncode == 0, log.read_text()[-3000:]
    text = log.read_text()
    assert "Traceback" not in text and "TM setting" not in text, text[-3000:]


def test_status_says_fake_and_the_population_spawned(server):
    base, _, _ = server
    st = httpx.get(base + "/api/status", timeout=5).json()
    assert st["connected"] and st["fake"] is True and st["map"] == "Carla/Maps/FakeTown"
    adm = httpx.get(base + "/admin/status", headers={"X-Admin-Token": TOKEN}, timeout=5).json()
    assert adm["background_vehicles"] == 6 and adm["pedestrians"] == 3 and adm["rsu_count"] == 9


def test_join_hub_websocket_v2x_and_lidar(server):
    base, port, _ = server

    async def drive():
        async with httpx.AsyncClient(base_url=base, timeout=10) as c:
            j = (await c.post("/client/join", json={"client_name": "e2e", "blueprint": "vehicle.audi.a2"})).json()
            aid, sid, tok = j["actor_id"], j["session_id"], j["session_token"]
            got = {}
            async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as ws:
                await ws.send(json.dumps({"op": "auth", "admin_token": TOKEN}))
                for topic in ("world.state", "v2x.frame"):
                    await ws.send(json.dumps({"op": "subscribe", "topic": topic}))
                end = time.monotonic() + 15
                while time.monotonic() < end and not ({"world.state", "v2x.frame"} <= set(got)
                                                       and aid in {s["station_id"] for s in got["v2x.frame"]["stations"]}):
                    m = json.loads(await asyncio.wait_for(ws.recv(), 5))
                    if m.get("data") is not None:
                        got[m["topic"]] = m["data"]
            async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as ws:   # the vehicle may not see ground truth
                await ws.send(json.dumps({"op": "auth", "session_token": tok}))
                await ws.send(json.dumps({"op": "subscribe", "topic": "world.state"}))
                replies = [json.loads(await asyncio.wait_for(ws.recv(), 5)) for _ in range(2)]
            r = await c.put(f"/client/lidar/{sid}", json={"sensor_type": "ray_cast_semantic", "points_per_second": 40000})
            assert r.status_code == 202, r.text
            state = None
            for _ in range(50):
                state = (await c.get(f"/client/lidar/{sid}")).json()["sensor"]["state"]
                if state == "active":
                    break
                await asyncio.sleep(0.1)
            assert (await c.delete(f"/client/leave/{sid}")).status_code == 200
            return aid, got, replies, state

    aid, got, replies, lidar_state = asyncio.run(drive())
    frame = got["v2x.frame"]
    assert aid in {s["station_id"] for s in frame["stations"]}
    assert sum(v["total"] for v in frame["channel"].values()) > 0
    assert {s["kind"] for s in frame["stations"]} >= {"OBU", "RSU", "VRU"}
    assert [r["topic"] for r in replies] == ["auth", "error"] and replies[1]["code"] == "forbidden"
    assert lidar_state == "active"
