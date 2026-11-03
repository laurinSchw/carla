"""Run recorder: layout, stamping, precision, determinism (handoff §4 and §5.6)."""

import gzip
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
    path = Path(path)
    if path.name.endswith(".gz"):
        return [json.loads(l) for l in gzip.decompress(path.read_bytes()).decode().splitlines()]
    return [json.loads(l) for l in path.read_text().splitlines()]


def links_file(r):
    return r.path / "v2x_links.jsonl.gz"


def test_directory_name_manifest_and_files(tmp_path):
    r = make(tmp_path)
    assert r.run_id == "20261103T120000Z_abcdef12_My-Run"
    m = json.loads((r.path / "manifest.json").read_text())
    assert m["start_frame"] == 100 and m["config"] == {"seed": 42} and "complete" not in m
    for f in ("events.jsonl", "stations.jsonl", "v2x_links.jsonl.gz"):
        assert (r.path / f).exists()


def test_every_record_is_stamped_and_files_have_no_wall_clock(tmp_path):
    r = make(tmp_path)
    r.event(101, 5.05, "spawn", actor_id=7)
    r.stations(110, 5.5, [{"id": 7, "x_m": 1.0}])
    r.links(110, 5.5, [{"tx_id": 1}])
    r.stop(120, 6.0)
    for f in ("events.jsonl", "stations.jsonl", "v2x_links.jsonl.gz"):
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
    assert lines(links_file(r)) == []
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
    return {f: (r.path / f).read_bytes() for f in ("stations.jsonl", "v2x_links.jsonl.gz")}


def test_same_seed_gives_byte_identical_data_files_and_a_different_seed_does_not(tmp_path):
    a = _record_channel_run(tmp_path / "a", 42)
    b = _record_channel_run(tmp_path / "b", 42)
    c = _record_channel_run(tmp_path / "c", 43)
    assert a["stations.jsonl"] == b["stations.jsonl"] and a["v2x_links.jsonl.gz"] == b["v2x_links.jsonl.gz"]   # raw bytes
    assert a["v2x_links.jsonl.gz"] != c["v2x_links.jsonl.gz"]
    assert len(gzip.decompress(a["v2x_links.jsonl.gz"]).splitlines()) == 5


def test_rows_are_readable_on_disk_while_the_run_is_still_open(tmp_path):
    r = make(tmp_path)
    r.event(1, 0.05, "first")
    assert [x["kind"] for x in lines(r.path / "events.jsonl")] == ["first"]      # no stop() needed
    r.stations(2, 0.1, [])
    assert len(lines(r.path / "stations.jsonl")) == 1
    r.stop(3, 0.15)


def test_links_are_columnar_with_the_column_names_in_the_manifest_and_nothing_rounded(tmp_path):
    from src.v2x.dsrc_channel import LinkResult
    r = make(tmp_path)
    link = LinkResult(7, 0.35, 1, 2, "BSM", "V2V", 12.345678901234567, True, 23.0, 70.1, -1.5, -60.25, 24.75, 1.0, 0.9, 0.9, True, None)
    r.links(7, 0.35, [link, link])
    m = r.stop(8, 0.4)
    cols = m["formats"]["v2x_links"]["columns"]
    assert cols[:6] == ["frame", "sim_time", "tx_id", "rx_id", "msg_type", "link_type"] and len(cols) == 18
    (line,) = lines(links_file(r))
    assert line["frame"] == 7 and len(line["rows"]) == 2
    row = dict(zip(cols, line["rows"][0]))
    assert row["distance_m"] == 12.345678901234567 and row["drop_cause"] is None and row["delivered"] is True
    assert json.loads((r.path / "manifest.json").read_text())["formats"]["v2x_links"]["file"] == "v2x_links.jsonl.gz"


def test_columnar_gzip_is_much_smaller_than_one_json_object_per_link(tmp_path):
    from src.v2x.dsrc_channel import LinkResult
    r = make(tmp_path)
    rows = [LinkResult(10, 0.5, i % 40, (i * 7) % 40, "BSM", "V2V", 10.0 + i * 0.37, True, 23.0, 70.0 + i * 0.01, -1.0,
                       -60.0 - i * 0.01, 24.0, 1.0, 0.9, 0.9, i % 3 != 0, None if i % 3 else "channel_or_mac") for i in range(3000)]
    r.links(10, 0.5, rows)
    r.stop(11, 0.55)
    naive = sum(len(dumps(l)) + 1 for l in rows)
    assert links_file(r).stat().st_size < naive / 5


# ── fixes from the independent review ────────────────────────────────────────

def test_two_runs_with_the_same_name_in_the_same_second_get_distinct_directories(tmp_path):
    """Review finding: stop + immediate restart (or names that slug alike) raised FileExistsError -> an unmapped HTTP 500."""
    a, b, c = make(tmp_path), make(tmp_path), make(tmp_path)
    assert len({a.run_id, b.run_id, c.run_id}) == 3 and b.run_id.endswith("-2") and c.run_id.endswith("-3")
    for r in (a, b, c):
        r.stop(1, 0.1)
    assert len(list_runs(tmp_path)) == 3
    assert RunRecorder(tmp_path, "a b", {}, start_frame=0, start_sim_time=0.0, git_sha="x", utcnow=lambda: NOW).run_id != \
        RunRecorder(tmp_path, "a/b", {}, start_frame=0, start_sim_time=0.0, git_sha="x", utcnow=lambda: NOW).run_id


def test_a_write_error_is_reported_in_the_manifest_and_stops_further_writing(tmp_path):
    """Review finding: after a transient ENOSPC the gzip stream was damaged but the manifest still said complete=true."""
    r = make(tmp_path)
    r.event(1, 0.05, "ok")
    real = r._files["events.jsonl"]

    class Failing:
        def write(self, *_a):
            raise OSError(28, "No space left on device")

        def flush(self):
            pass

        def close(self):
            real.close()

    r._files["events.jsonl"] = Failing()
    r.event(2, 0.1, "lost")
    assert r.write_error and "No space" in r.write_error
    r.stations(3, 0.15, [])                                  # no further writes once the run is damaged
    assert r.counts["stations.jsonl"] == 0
    m = r.stop(4, 0.2)
    assert m["complete"] is False and "No space" in m["write_error"]
    assert json.loads((r.path / "manifest.json").read_text())["complete"] is False


def test_stop_writes_the_manifest_even_if_closing_a_file_fails(tmp_path):
    r = make(tmp_path)

    class BadClose:
        def close(self):
            raise OSError("boom")

    r._files["stations.jsonl"] = BadClose()
    m = r.stop(5, 0.25)
    assert (r.path / "manifest.json").exists() and m["complete"] is False and "boom" in m["write_error"]
    assert r._links_raw.closed                                 # the other handles were still closed


def test_a_failing_init_does_not_leak_open_files(tmp_path, monkeypatch):
    import builtins
    opened = []
    real_open = builtins.open

    def flaky(file, *a, **k):
        if str(file).endswith("stations.jsonl"):
            raise OSError("cannot open")
        f = real_open(file, *a, **k)
        opened.append(f)
        return f

    monkeypatch.setattr(builtins, "open", flaky)
    with pytest.raises(OSError):
        make(tmp_path)
    assert opened and all(f.closed for f in opened)


def test_links_can_be_read_while_the_run_is_open_and_after_a_crash_cut_the_file(tmp_path):
    """Review finding: the documented gzip.open loader raised EOFError mid-run; flush was never tested either."""
    from src.recording.recorder import iter_link_rows
    r = make(tmp_path)
    r.links(1, 0.05, [{"a": 1, "b": 2}, {"a": 3, "b": 4}])
    got = list(iter_link_rows(links_file(r), ["a", "b"]))                     # no stop(): the stream has no trailer yet
    assert got == [(1, 0.05, {"a": 1, "b": 2}), (1, 0.05, {"a": 3, "b": 4})]
    r.links(2, 0.1, [{"a": 5, "b": 6}])
    raw = links_file(r).read_bytes()
    r.stop(3, 0.15)
    links_file(r).write_bytes(raw[:-9])                                       # a crash cuts the file short
    assert len(list(iter_link_rows(links_file(r)))) >= 2
    with pytest.raises(EOFError):
        gzip.decompress(links_file(r).read_bytes())                           # which is why the tolerant reader exists


def test_the_gzip_header_has_no_timestamp_so_runs_recorded_at_different_times_are_byte_identical(tmp_path):
    import time
    from src.v2x.dsrc_channel import LinkResult
    link = LinkResult(1, 0.05, 1, 2, "BSM", "V2V", 5.0, True, 23.0, 70.0, 0.0, -50.0, 35.0, 1.0, 1.0, 1.0, True, None)

    def run(name, root):
        r = RunRecorder(root, name, {}, start_frame=0, start_sim_time=0.0, git_sha="x", utcnow=lambda: NOW)
        r.links(1, 0.05, [link])
        r.stop(2, 0.1)
        return links_file(r).read_bytes()
    a = run("t", tmp_path / "a")
    time.sleep(1.1)                                           # a gzip header with the current time would now differ
    b = run("t", tmp_path / "b")
    assert a == b
