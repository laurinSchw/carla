"""Headless scenario runs on the FAKE world (no CARLA): same scenario twice -> byte-identical data files.

This checks the server's own determinism (seeds, frame-scheduled commands, recorder) on a deterministic world. It says
nothing about CARLA's physics / Traffic Manager determinism, which is not bit-identical live (AGENTS.md section 6a).
Each run is a subprocess: the fake must be ``sys.modules['carla']`` before the server imports it, and must not leak
into the other tests (some compare against the real carla module).
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

from src.lidar import protocol
from src.recording.recorder import iter_link_rows

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "scenarios" / "crossing.yaml"
DATA_FILES = ("stations.jsonl", "v2x_links.jsonl.gz", "lidar_meta.jsonl")


def run(tmp_path, name, scenario=SCENARIO, extra=()):
    out = tmp_path / name
    res = subprocess.run([sys.executable, "-m", "src.cli", "run", str(scenario), "--runs-dir", str(out), *extra], cwd=ROOT,
                         capture_output=True, text=True, timeout=300)
    assert res.returncode == 0, res.stdout + res.stderr
    (d,) = list(out.iterdir())
    return d, json.loads((d / "manifest.json").read_text())


@pytest.fixture(scope="module")
def two_runs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("repro")
    return run(tmp, "a"), run(tmp, "b")


def test_the_same_scenario_twice_gives_byte_identical_data_files(two_runs):
    (da, ma), (db, mb) = two_runs
    for f in DATA_FILES:
        assert (da / f).read_bytes() == (db / f).read_bytes(), f"{f} differs between two runs of the same scenario"
    ra, rb = sorted((da / "lidar").rglob("*.bin")), sorted((db / "lidar").rglob("*.bin"))
    assert len(ra) > 100 and [p.name for p in ra] == [p.name for p in rb]
    assert all(a.read_bytes() == b.read_bytes() for a, b in zip(ra, rb)), "raw LiDAR frames must be reproducible too"
    assert ma["run_id"] != mb["run_id"] and ma["row_counts"] == mb["row_counts"]
    from src.cli import compare_runs
    assert compare_runs(da, db) == []


def test_a_different_seed_gives_different_data(tmp_path, two_runs):
    sc = tmp_path / "seed8.yaml"
    sc.write_text(SCENARIO.read_text().replace("seed: 7", "seed: 8"))
    d8, m8 = run(tmp_path, "c", sc)
    (da, ma), _ = two_runs
    assert (d8 / "v2x_links.jsonl.gz").read_bytes() != (da / "v2x_links.jsonl.gz").read_bytes()
    assert m8["config"]["seed"] == 8 and m8["seeds"]["dsrc"] == 8


def test_manifest_says_what_produced_the_run(two_runs):
    (_, m), _ = two_runs
    assert m["status"] == "finished" and m["complete"] is True and m["end_frame"] == m["start_frame"] + 400
    assert m["fake_carla"] is True and m["versions"]["carla_server"] == "0.9.16-fake"
    assert m["config"]["realtime"] is False and m["reproducibility"]["realtime_pacing"] is False and m["config"]["seed"] == 7
    assert m["scenario"]["name"] == "crossing" and len(m["scenario"]["commands"]) == 5
    assert m["lidar_protocol"] == {"version": protocol.VERSION, "field_layout": protocol.FIELD_LAYOUT_ID}


def test_scripted_commands_ran_at_their_frames(two_runs):
    (d, m), _ = two_runs
    s = m["start_frame"]
    ev = [json.loads(line) for line in (d / "events.jsonl").read_text().splitlines()]
    first = {}
    for e in ev:
        first.setdefault((e["kind"], e.get("state") or e.get("reason")), e)
    assert first[("hazard_publish", None)]["frame"] == s + 60
    assert first[("lidar", "requested")]["frame"] == s + 100
    assert first[("rsu_changed", None)]["frame"] == s + 200 and first[("rsu_changed", None)]["enabled"] is False
    cc = first[("config_changed", None)]
    assert cc["frame"] == s + 300 and cc["changes"]["awareness_range_m"]["new"] == 50.0
    assert first[("lidar", "stopping")]["frame"] == s + 340
    assert ev[0]["kind"] == "run_start" and ev[-1]["kind"] == "run_stop"
    assert not any(e["kind"] == "scheduled_command_failed" for e in ev)


def test_the_recorded_links_show_the_denm_the_disabled_rsu_and_the_new_awareness_range(two_runs):
    (d, m), _ = two_runs
    s = m["start_frame"]
    cols = m["formats"]["v2x_links"]["columns"]
    links = [row for _, _, row in iter_link_rows(d / "v2x_links.jsonl.gz", cols)]
    denm = [r for r in links if r["msg_type"] == "DENM"]
    # injected before tick s+60, valid 5 s = 100 ticks; the V2X update runs every 10 ticks
    assert denm and all(s + 60 < r["frame"] <= s + 60 + 100 + 10 for r in denm)
    assert {r["link_type"] for r in denm} == {"I2V"}
    # the recording keeps the network's raw RSU id (intersection 5, link type I2V); the API / hub call it station -5
    rsu5 = [r for r in links if r["tx_id"] == 5 and r["link_type"] == "I2V" and r["msg_type"] in ("MAP", "SPAT")]
    assert any(r["frame"] < s + 200 for r in rsu5) and not any(r["frame"] > s + 200 for r in rsu5)
    late_v2v = [r for r in links if r["frame"] > s + 300 and r["delivered"] and r["link_type"] == "V2V"]
    assert late_v2v and max(r["distance_m"] for r in late_v2v) <= 50.0


def test_raw_lidar_frames_decode_and_match_the_meta(two_runs):
    (d, _), _ = two_runs
    meta = [json.loads(line) for line in (d / "lidar_meta.jsonl").read_text().splitlines()]
    f = meta[10]
    header, pts = protocol.decode((d / f["file"]).read_bytes())
    assert header.frame == f["frame"] and header.num_points == f["num_points"] == len(pts)
    assert header.field_mask == protocol.SEMANTIC_FIELDS and header.wall_time == 0.0
    assert (pts["object_idx"] == f["actor_id"]).any(), "a roof LiDAR sees its own vehicle (CARLA does too)"
    assert [x["frame"] for x in meta] == sorted(x["frame"] for x in meta) and len({x["frame"] for x in meta}) == len(meta)


def test_a_bad_scenario_is_refused_with_a_message(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\nduration_frames: 0\nbogus: 1\n")
    res = subprocess.run([sys.executable, "-m", "src.cli", "run", str(bad), "--fake"], cwd=ROOT, capture_output=True,
                         text=True, timeout=60)
    assert res.returncode != 0 and "duration_frames" in res.stderr and "bogus" in res.stderr
