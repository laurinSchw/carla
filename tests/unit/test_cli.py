"""The command line: 'serve' hands its options to start_server.py untouched; 'compare' finds differing data files;
scenarios are validated before anything starts."""

import sys

import pytest

import start_server
from src import cli
from src.scenario import Scenario, load_scenario


def test_serve_forwards_leading_options(monkeypatch):
    seen = []
    monkeypatch.setattr(start_server, "main", lambda: seen.append(list(sys.argv)))
    monkeypatch.setattr(sys, "argv", ["keep"])
    assert cli.main(["serve", "--fake", "--port", "9999"]) == 0
    assert seen == [["start_server.py", "--fake", "--port", "9999"]]


def test_fake_options_parse(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["start_server.py", "--fake", "--fake-vehicles", "3", "--seed", "5"])
    a = start_server._parse_args()
    assert (a.fake, a.fake_vehicles, a.fake_pedestrians, a.seed) == (True, 3, 6, 5)


def test_compare_reports_identical_and_different_runs(tmp_path, capsys):
    a, b = tmp_path / "a", tmp_path / "b"
    for d in (a, b):
        (d / "lidar" / "1").mkdir(parents=True)
        (d / "stations.jsonl").write_text("x\n")
        (d / "lidar" / "1" / "5.bin").write_bytes(b"\x01")
        (d / "events.jsonl").write_text(d.name)           # events carry wall times: never compared
    assert cli.compare_runs(a, b) == [] and cli.main(["compare", str(a), str(b)]) == 0
    (b / "lidar" / "1" / "5.bin").write_bytes(b"\x02")
    (b / "stations.jsonl").write_text("y\n")
    assert cli.compare_runs(a, b) == ["stations.jsonl", "lidar/ (1 of 1 frames)"]
    assert cli.main(["compare", str(a), str(b)]) == 1


def test_scenario_validation(tmp_path):
    p = tmp_path / "s.json"
    p.write_text('{"name": "x", "duration_frames": 5, "commands": [{"frame": 1, "action": "rsu", "station_id": -1}]}')
    sc = load_scenario(p)
    assert sc.commands[0].model_extra == {"station_id": -1} and sc.record == ["links"]
    for bad in ({"name": "x", "duration_frames": 0}, {"name": "x", "duration_frames": 1, "bogus": 1},
                {"name": "x", "duration_frames": 1, "record": ["everything"]},
                {"name": "x", "duration_frames": 1, "config": {"realtime": True}},
                {"name": "x", "duration_frames": 1, "commands": [{"frame": 1, "action": "explode"}]}):
        with pytest.raises(ValueError):
            Scenario.model_validate(bad)
