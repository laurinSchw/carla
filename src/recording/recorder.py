"""Run directory writer (no CARLA import).

    runs/<UTC>_<git sha>_<name>/
        manifest.json      who/what/with which settings; start and end frame; row counts per file; versions
        events.jsonl       one line per event: {"frame", "sim_time", "kind", ...}  (spawns, sessions, LiDAR states, advisory ...)
        stations.jsonl     one line per V2X update: ground truth of every vehicle / RSU / VRU
        v2x_links.jsonl    one line per transmission attempt: the full LinkResult (channel decision)
        lidar_meta.jsonl   (optional) one line per recorded LiDAR frame
        lidar/<sensor>/<frame>.bin   (optional) protocol-v3 frames at FULL resolution (never subsampled)

``v2x_links.jsonl.gz`` is gzip-compressed and COLUMNAR (the first live recording of 25 vehicles was 28 MB in 30 s as one
JSON object per link): each line is ``{"frame", "sim_time", "rows": [[...], ...]}`` and the column names are in
``manifest.json`` under ``formats.v2x_links.columns``. No value is rounded. The file is flushed after every update, so it
can be read while the run is in progress with ``iter_link_rows`` (plain ``gzip.open`` raises EOFError on an unfinished stream).

Rules (handoff §4): every record carries ``frame`` and ``sim_time``; wall-clock time appears only in the manifest;
values are written at full precision, never rounded or decimated; non-finite numbers become ``null``. With the same
seed, configuration and command script the data files (not the manifest) are byte-identical — that is what
``tests/unit/test_recorder.py`` checks for the parts that do not depend on CARLA.
"""

from __future__ import annotations

import gzip
import io
import json
import logging
import math
import re
import time
import zlib
from dataclasses import asdict, fields, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional

from src.lidar import protocol

class RunAlreadyRecording(Exception):
    """Only one run can be recorded at a time (HTTP 409)."""


logger = logging.getLogger(__name__)

FILES = ("events.jsonl", "stations.jsonl", "v2x_links.jsonl")
LINKS_FILE = "v2x_links.jsonl.gz"  # logical name "v2x_links.jsonl" in row_counts


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
        base = f"{started:%Y%m%dT%H%M%SZ}_{git_sha[:8]}_{_slug(name)}"
        self.run_id, n = base, 1
        Path(root).mkdir(parents=True, exist_ok=True)
        while True:  # two runs in the same second (stop + immediate restart) or names that slug alike must not collide
            self.path = Path(root) / self.run_id
            try:
                self.path.mkdir(exist_ok=False)
                break
            except FileExistsError:
                n += 1
                self.run_id = f"{base}-{n}"
        self.write_error: Optional[str] = None
        self._files: Dict[str, Any] = {}
        self._links_raw = None
        self._lidar_meta = None
        self.record_links = record_links
        self.record_lidar = record_lidar
        self.counts: Dict[str, int] = {f: 0 for f in FILES}
        self.counts["lidar_frames"] = 0
        try:
            # gzip header: no file name, mtime 0 -> the compressed bytes depend only on the data (reproducible runs)
            self._links_raw = open(self.path / LINKS_FILE, "wb")
            for f in FILES:
                self._files[f] = (
                    io.TextIOWrapper(gzip.GzipFile(filename="", mode="wb", fileobj=self._links_raw, compresslevel=3, mtime=0),
                                     encoding="utf-8", newline="\n")
                    if f == "v2x_links.jsonl" else open(self.path / f, "w", encoding="utf-8", newline="\n"))
        except BaseException:
            self._close_handles()   # do not leak the handles that were opened before the failure
            raise
        self._link_columns: Optional[List[str]] = None
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
        """One line per update; rows are lists in the fixed column order recorded in the manifest."""
        if not self.record_links:
            return
        links = list(links)
        if links and self._link_columns is None:
            first = links[0]
            self._link_columns = [f.name for f in fields(first)] if is_dataclass(first) else list(first)
            self._manifest.setdefault("formats", {})["v2x_links"] = {
                "file": LINKS_FILE, "encoding": "gzip, JSON lines, columnar",
                "line": {"frame": "int", "sim_time": "float", "rows": "list of rows; each row is a list in `columns` order"},
                "columns": self._link_columns,
            }
            self._write_manifest(final=False)
        cols = self._link_columns or []
        rows = [[_clean(getattr(l, c) if is_dataclass(l) else l[c]) for c in cols] for l in links]
        self._line("v2x_links.jsonl", {"frame": frame, "sim_time": sim_time, "rows": rows})

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
            close_errors = self._close_handles()
            if close_errors and self.write_error is None:
                self.write_error = "; ".join(close_errors)
            self._manifest.update({"end_frame": frame, "end_sim_time": sim_time, "ended_wall_time_unix": self._wall(),
                                   "row_counts": dict(self.counts), "complete": self.write_error is None})
            if self.write_error is not None:
                self._manifest["write_error"] = self.write_error  # the files are incomplete or damaged: say so
            self._write_manifest(final=True)
        return dict(self._manifest)

    def _close_handles(self) -> List[str]:
        """Close every open file, whatever fails, so one bad handle cannot leave the others (and the manifest) behind."""
        errors = []
        for f in list(self._files.values()) + [self._links_raw, self._lidar_meta]:
            if f is None:
                continue
            try:
                f.close()
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{type(exc).__name__}: {exc}")
        return errors

    def summary(self) -> dict:
        return {"run_id": self.run_id, "path": str(self.path), "row_counts": dict(self.counts),
                "record_links": self.record_links, "record_lidar": self.record_lidar, "closed": self.closed}

    # -- internals -------------------------------------------------------------------------------
    def _line(self, name: str, obj: dict) -> None:
        if self.closed or self.write_error is not None:
            return
        try:
            f = self._files[name]
            f.write(dumps(obj) + "\n")
            f.flush()  # a run can be read (tail -f, plots) while it is still being recorded; a crash loses at most one row
            if name == "v2x_links.jsonl":
                self._links_raw.flush()
            self.counts[name] += 1
        except OSError as exc:
            # A failed write can leave half a row or a damaged gzip stream behind. Stop writing altogether and say so in the
            # manifest, instead of carrying on and reporting a complete run (disk full, permissions, ...).
            self.write_error = f"{type(exc).__name__}: {exc}"
            logger.error("run %s: writing %s failed, recording stopped: %s", self.run_id, name, self.write_error)

    def _write_manifest(self, final: bool) -> None:
        tmp = self.path / "manifest.json.tmp"
        tmp.write_text(json.dumps(_clean(self._manifest), indent=1, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        tmp.replace(self.path / "manifest.json")
        if not final:  # let readers see the data written so far
            for f in self._files.values():
                f.flush()
            self._links_raw.flush()


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


def iter_link_rows(path, columns: Optional[List[str]] = None):
    """Yield ``(frame, sim_time, row)`` for every link record in a ``v2x_links.jsonl.gz`` file, tolerating a file that is still
    being written or was cut off by a crash (``gzip.open`` raises ``EOFError`` there; this reads everything that is complete).

    With ``columns`` (``manifest["formats"]["v2x_links"]["columns"]``) rows come as dicts.
    """
    data = Path(path).read_bytes()
    text = zlib.decompressobj(31).decompress(data).decode("utf-8", errors="ignore")  # 31 = gzip wrapper, stops at the end of data
    for line in text.split("\n"):
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
        except ValueError:
            break  # a half-written last line
        for row in obj["rows"]:
            yield obj["frame"], obj["sim_time"], (dict(zip(columns, row)) if columns else row)
