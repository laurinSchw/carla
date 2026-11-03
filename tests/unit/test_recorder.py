"""Run recorder: layout, stamping, precision, determinism (handoff §4 and §5.6)."""

import json
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from src.lidar import protocol
from src.recording.recorder import RunRecorder, dumps, list_runs
from src.v2x.dsrc_channel import DSRCChannel, DSRCConfig

NOW = datetime(2026, 11, 3, 12, 0, 0, tzinfo=timezone.utc)


def make(tmp_path, name="My Run!", **kw):
    return RunRecorder(tmp_path, name, {"config": {"seed": 42}}, start_frame=100, start_sim_time=5.0,
                       git_sha="abcdef123456", utcnow=lambda: NOW, wall_clock=lambda: 1.7e9, **kw)


def lines(path):
    return [json.loads(l) for l in Path(path).read_text().splitlines()]


def test_directory_name_manifest_and_files(tmp_path):
    r = make(tmp_path)
    assert r.run_id == "20261103T120000Z_abcdef12_My-Run"
    m = json.loads((r.path / "manifest.json").read_text())
    assert m["start_frame"] == 100 and m["config"] == {"seed": 42} and "complete" not in m
    for f in ("events.jsonl", "stations.jsonl", "v2x_links.jsonl"):
        assert (r.path / f).exists()


def test_every_record_is_stamped_and_files_have_no_wall_clock(tmp_path):
    r = make(tmp_path)
    r.event(101, 5.05, "spawn", actor_id=7)
    r.stations(110, 5.5, [{"id": 7, "x_m": 1.0}])
    r.links(110, 5.5, [{"tx_id": 1}])
    r.stop(120, 6.0)
    for f in ("events.jsonl", "stations.jsonl", "v2x_links.jsonl"):
        for row in lines(r.path / f):
            assert "frame" in row and "sim_time" in row
            assert not any("wall" in k for k in row)


def test_values_are_written_at_full_precision_and_nonfinite_become_null(tmp_path):
    r = make(tmp_path)
    x = 0.1 + 0.2                                    # 0.30000000000000004: must not be rounded
    r.event(1, 0.05, "t", x=x, ttc=float("inf"), n=np.float32(1.5), v=(1, 2))
    r.stop(2, 0.1)
    row = lines(r.path / "events.jsonl")[0]
    assert row["x"] == x and row["ttc"] is None and row["n"] == 1.5 and row["v"] == [1, 2]
    assert dumps({"b": 1, "a": 2}) == '{"a":2,"b":1}'              # stable key order


def test_stop_writes_final_manifest_with_counts_and_is_idempotent(tmp_path):
    r = make(tmp_path)
    r.event(1, 0.05, "a"); r.event(2, 0.1, "b")
    m = r.stop(150, 7.5)
    assert m["complete"] is True and m["end_frame"] == 150 and m["row_counts"]["events.jsonl"] == 2
    r.event(3, 0.15, "late")                                       # ignored after stop
    assert r.stop(999, 99.0)["end_frame"] == 150
    assert len(lines(r.path / "events.jsonl")) == 2
    assert json.loads((r.path / "manifest.json").read_text())["complete"] is True


def test_links_can_be_switched_off_and_runs_are_listed_newest_first(tmp_path):
    r = make(tmp_path, record_links=False)
    r.links(1, 0.05, [{"a": 1}])
    r.stop(2, 0.1)
    assert (r.path / "v2x_links.jsonl").read_text() == ""
    r2 = RunRecorder(tmp_path, "later", {}, start_frame=1, start_sim_time=0.0, git_sha="x",
                     utcnow=lambda: datetime(2026, 11, 4, tzinfo=timezone.utc))
    r2.stop(2, 0.1)
    assert [x["name"] for x in list_runs(tmp_path)] == ["later", "My Run!"]


def test_lidar_frames_are_recorded_at_full_resolution_and_decode(tmp_path):
    r = make(tmp_path, record_lidar=True)
    pts = np.zeros(5000, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"), ("object_idx", "u4"), ("tag", "u4")])
    pts["x"] = np.arange(5000)
    lf = NS(frame=105, sim_time=5.25, sensor_id=3, ego_actor_id=9,
            measurement=NS(points=pts, field_mask=protocol.SEMANTIC_FIELDS, location=(1.0, 2.0, 3.0), rotation=(0.0, 1.0, 2.0)))
    r.lidar_frame({"config": {"channels": 32}}, lf)
    r.stop(106, 5.3)
    header, got = protocol.decode((r.path / "lidar" / "3" / "105.bin").read_bytes())
    assert header.num_points == 5000 and np.array_equal(got["x"], pts["x"]) and header.frame == 105
    meta = lines(r.path / "lidar_meta.jsonl")[0]
    assert meta["num_points"] == 5000 and meta["file"] == "lidar/3/105.bin"


def test_lidar_is_not_recorded_unless_asked(tmp_path):
    r = make(tmp_path)
    r.lidar_frame({}, NS(frame=1))
    assert not (r.path / "lidar").exists() and not (r.path / "lidar_meta.jsonl").exists()


def _record_channel_run(root, seed):
    """Same inputs + same seed through the real channel model -> the data files must be byte-identical."""
    ch = DSRCChannel(DSRCConfig(random_seed=seed))
    r = RunRecorder(root, "det", {}, start_frame=0, start_sim_time=0.0, git_sha="x", utcnow=lambda: NOW)
    from src.v2x.safety_messages import BSMCore
    bsms = {i: BSMCore(timestamp=0.0, msg_count=0, vehicle_id=i, x_m=float(i * 40), y_m=float((i * 13) % 17)) for i in range(8)}
    pos = {i: (b.x_m, b.y_m) for i, b in bsms.items()}
    for frame in range(10, 60, 10):
        ch.broadcast_all(bsms, pos, frame=frame, sim_time=frame * 0.05)
        r.stations(frame, frame * 0.05, [{"id": i, "x_m": p[0], "y_m": p[1]} for i, p in pos.items()])
        r.links(frame, frame * 0.05, ch.last_links)
    r.stop(60, 3.0)
    return {f: (r.path / f).read_bytes() for f in ("stations.jsonl", "v2x_links.jsonl")}


def test_same_seed_gives_byte_identical_data_files_and_a_different_seed_does_not(tmp_path):
    a = _record_channel_run(tmp_path / "a", 42)
    b = _record_channel_run(tmp_path / "b", 42)
    c = _record_channel_run(tmp_path / "c", 43)
    assert a["stations.jsonl"] == b["stations.jsonl"] and a["v2x_links.jsonl"] == b["v2x_links.jsonl"]
    assert a["v2x_links.jsonl"] != c["v2x_links.jsonl"]
    assert len(a["v2x_links.jsonl"].splitlines()) == 5


def test_rows_are_readable_on_disk_while_the_run_is_still_open(tmp_path):
    r = make(tmp_path)
    r.event(1, 0.05, "first")
    assert [x["kind"] for x in lines(r.path / "events.jsonl")] == ["first"]      # no stop() needed
    r.stations(2, 0.1, [])
    assert len(lines(r.path / "stations.jsonl")) == 1
    r.stop(3, 0.15)
