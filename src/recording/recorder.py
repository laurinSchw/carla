"""Run directory writer (no CARLA import).

    runs/<UTC>_<git sha>_<name>/
        manifest.json      who/what/with which settings; start and end frame; row counts per file; versions
        events.jsonl       one line per event: {"frame", "sim_time", "kind", ...}  (spawns, sessions, LiDAR states, advisory ...)
        stations.jsonl     one line per V2X update: ground truth of every vehicle / RSU / VRU
        v2x_links.jsonl    one line per transmission attempt: the full LinkResult (channel decision)
        lidar_meta.jsonl   (optional) one line per recorded LiDAR frame
        lidar/<sensor>/<frame>.bin   (optional) protocol-v3 frames at FULL resolution (never subsampled)

Rules (handoff §4): every record carries ``frame`` and ``sim_time``; wall-clock time appears only in the manifest;
values are written at full precision, never rounded or decimated; non-finite numbers become ``null``. With the same
seed, configuration and command script the data files (not the manifest) are byte-identical — that is what
``tests/unit/test_recorder.py`` checks for the parts that do not depend on CARLA.
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional

from src.lidar import protocol

class RunAlreadyRecording(Exception):
    """Only one run can be recorded at a time (HTTP 409)."""


FILES = ("events.jsonl", "stations.jsonl", "v2x_links.jsonl")


def _clean(value: Any) -> Any:
    """JSON-safe copy: dataclasses -> dicts, non-finite floats -> None, tuples -> lists."""
    if is_dataclass(value) and not isinstance(value, type):
        value = asdict(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if hasattr(value, "item"):  # numpy scalar
        return _clean(value.item())
    return value


def dumps(obj: Any) -> str:
    return json.dumps(_clean(obj), separators=(",", ":"), sort_keys=True, allow_nan=False)


def _slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", name.strip()).strip("-")[:48] or "run"


class RunRecorder:
    def __init__(
        self,
        root: Path,
        name: str,
        manifest: Dict[str, Any],
        *,
        start_frame: int,
        start_sim_time: float,
        git_sha: str = "nogit",
        record_links: bool = True,
        record_lidar: bool = False,
        utcnow: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self._wall = wall_clock
        started = utcnow()
        self.run_id = f"{started:%Y%m%dT%H%M%SZ}_{git_sha[:8]}_{_slug(name)}"
        self.path = Path(root) / self.run_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.record_links = record_links
        self.record_lidar = record_lidar
        self.counts: Dict[str, int] = {f: 0 for f in FILES}
        self.counts["lidar_frames"] = 0
        self._files = {f: open(self.path / f, "w", encoding="utf-8", newline="\n") for f in FILES}
        self._lidar_meta = None
        self.closed = False
        self._manifest = {
            "run_id": self.run_id, "name": name, "git_sha": git_sha,
            "started_utc": started.isoformat(), "started_wall_time_unix": wall_clock(),
            "start_frame": start_frame, "start_sim_time": start_sim_time,
            "record_links": record_links, "record_lidar": record_lidar,
            **manifest,
        }
        self._write_manifest(final=False)

    # -- writers (simulation thread) -------------------------------------------------------------
    def event(self, frame: int, sim_time: float, kind: str, **data: Any) -> None:
        self._line("events.jsonl", {"frame": frame, "sim_time": sim_time, "kind": kind, **data})

    def stations(self, frame: int, sim_time: float, stations: List[dict]) -> None:
        self._line("stations.jsonl", {"frame": frame, "sim_time": sim_time, "stations": stations})

    def links(self, frame: int, sim_time: float, links: Iterable[Any]) -> None:
        if not self.record_links:
            return
        rows = [_clean(l) for l in links]
        self._line("v2x_links.jsonl", {"frame": frame, "sim_time": sim_time, "links": rows})

    def lidar_frame(self, info: dict, lf: Any) -> None:
        """Raw-listener hook for ``LidarManager``: the frame is written exactly as delivered (full resolution)."""
        if not self.record_lidar or self.closed:
            return
        m = lf.measurement
        d = self.path / "lidar" / str(lf.sensor_id)
        d.mkdir(parents=True, exist_ok=True)
        data = protocol.encode(m.points, frame=lf.frame, sim_time=lf.sim_time, wall_time=0.0,
                               ego_actor_id=lf.ego_actor_id, sensor_id=lf.sensor_id,
                               location=m.location, rotation=m.rotation, field_mask=m.field_mask)
        (d / f"{lf.frame}.bin").write_bytes(data)
        if self._lidar_meta is None:
            self._lidar_meta = open(self.path / "lidar_meta.jsonl", "w", encoding="utf-8", newline="\n")
        self._lidar_meta.write(dumps({"frame": lf.frame, "sim_time": lf.sim_time, "sensor_id": lf.sensor_id,
                                      "actor_id": lf.ego_actor_id, "num_points": len(m.points),
                                      "file": f"lidar/{lf.sensor_id}/{lf.frame}.bin", "config": info.get("config")}) + "\n")
        self.counts["lidar_frames"] += 1

    def stop(self, frame: int, sim_time: float) -> dict:
        """Close the files and write the final manifest. Safe to call twice."""
        if not self.closed:
            self.closed = True
            for f in self._files.values():
                f.close()
            if self._lidar_meta is not None:
                self._lidar_meta.close()
            self._manifest.update({"end_frame": frame, "end_sim_time": sim_time, "ended_wall_time_unix": self._wall(),
                                   "row_counts": dict(self.counts), "complete": True})
            self._write_manifest(final=True)
        return dict(self._manifest)

    def summary(self) -> dict:
        return {"run_id": self.run_id, "path": str(self.path), "row_counts": dict(self.counts),
                "record_links": self.record_links, "record_lidar": self.record_lidar, "closed": self.closed}

    # -- internals -------------------------------------------------------------------------------
    def _line(self, name: str, obj: dict) -> None:
        if self.closed:
            return
        f = self._files[name]
        f.write(dumps(obj) + "\n")
        f.flush()  # a run can be read (tail -f, plots) while it is still being recorded; a crash loses at most one row
        self.counts[name] += 1

    def _write_manifest(self, final: bool) -> None:
        tmp = self.path / "manifest.json.tmp"
        tmp.write_text(json.dumps(_clean(self._manifest), indent=1, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        tmp.replace(self.path / "manifest.json")
        if not final:  # let readers see the data written so far
            for f in self._files.values():
                f.flush()


def list_runs(root: Path) -> List[dict]:
    """Manifests of all runs under ``root``, newest first."""
    out = []
    for d in sorted(Path(root).glob("*/manifest.json"), reverse=True):
        try:
            m = json.loads(d.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        out.append({k: m.get(k) for k in ("run_id", "name", "started_utc", "start_frame", "end_frame", "complete", "row_counts")})
    return out
