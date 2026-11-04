"""Command line: ``python -m src.cli run scenario.yaml [--fake] [--runs-dir runs]`` runs a scenario headless and records it;
``python -m src.cli compare RUN_A RUN_B`` compares the data files of two runs; ``serve`` starts the web server (every option
after ``serve`` goes to start_server.py)."""

from __future__ import annotations

import argparse
import hashlib
import logging
import sys
from pathlib import Path
from typing import List, Optional

#: data files of a run that must be identical for a reproducible run (events carry wall times, the manifest wall times + run id)
DATA_FILES = ("v2x_links.jsonl.gz", "stations.jsonl", "messages.jsonl", "lidar_meta.jsonl")


def _digest(path: Path) -> Optional[str]:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def compare_runs(a: Path, b: Path) -> List[str]:
    """Names of the data files (and raw LiDAR frames) that differ between two run directories; empty = identical."""
    diff = [f for f in DATA_FILES if _digest(a / f) != _digest(b / f)]
    la = sorted(p.relative_to(a) for p in (a / "lidar").rglob("*.bin")) if (a / "lidar").is_dir() else []
    lb = sorted(p.relative_to(b) for p in (b / "lidar").rglob("*.bin")) if (b / "lidar").is_dir() else []
    if la != lb:
        diff.append("lidar/ (different frame files)")
    else:
        n = sum(_digest(a / p) != _digest(b / p) for p in la)
        if n:
            diff.append(f"lidar/ ({n} of {len(la)} frames)")
    return diff


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="python -m src.cli", description="CARLA V2X research server: headless runs")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run a scenario headless and record it")
    r.add_argument("scenario")
    r.add_argument("--runs-dir", default="runs")
    r.add_argument("--fake", action="store_true", help="use the deterministic synthetic world instead of CARLA")
    r.add_argument("-v", "--verbose", action="store_true")
    c = sub.add_parser("compare", help="compare the data files of two recorded runs (exit 0 = identical)")
    c.add_argument("run_a")
    c.add_argument("run_b")
    sub.add_parser("serve", help="start the web server; every option after 'serve' goes to start_server.py")
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["serve"]:               # argparse cannot forward leading --options, so hand them over untouched
        import start_server
        sys.argv = ["start_server.py", *argv[1:]]
        start_server.main()
        return 0
    args = p.parse_args(argv)
    if args.cmd == "compare":
        diff = compare_runs(Path(args.run_a), Path(args.run_b))
        for name in DATA_FILES:
            print(f"  {name:22s} {_digest(Path(args.run_a) / name) or '-':.16} {_digest(Path(args.run_b) / name) or '-':.16}")
        print("identical" if not diff else "DIFFERENT: " + ", ".join(diff))
        return 0 if not diff else 1
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from src.scenario import load_scenario, run_scenario
    m = run_scenario(load_scenario(args.scenario), args.runs_dir, fake=True if args.fake else None)
    print(f"run {m['run_id']}: {m['status']}, frames {m['start_frame']}..{m['end_frame']}, "
          f"{m['row_counts']} in {m['_wall_s']} s -> {m['_dir']}")
    d = Path(m["_dir"])
    for name in DATA_FILES:
        if (d / name).is_file():
            print(f"  {name:22s} {(d / name).stat().st_size:>10} bytes  sha256 {_digest(d / name)[:16]}")
    for f in m["_failed_commands"]:
        print(f"  FAILED command at frame {f['frame']} ({f['action']}): {f['error']}")
    return 0 if m["status"] == "finished" and not m["_failed_commands"] else 2


if __name__ == "__main__":
    sys.exit(main())
