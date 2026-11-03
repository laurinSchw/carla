"""Golden-file helper: compare a JSON-serialisable value with a stored file.

Regenerate deliberately with ``UPDATE_GOLDEN=1 pytest tests/golden`` and review the diff.
A golden change is either a bug in a refactor or a deliberate model change that needs a
docs/science/CHANGELOG.md entry with before/after numbers.
"""

import json
import os
from pathlib import Path

GOLDEN_DIR = Path(__file__).parent / "expected"

# Floats may differ in the last digits between Python/libm builds (3.10 image vs 3.12 host).
# 1e-9 relative is far below any model change but above that noise.
_REL_TOL = 1e-9


def _same(a, b) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, float) or isinstance(b, float):
        if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
            return False
        return a == b or abs(a - b) <= _REL_TOL * max(abs(a), abs(b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def assert_matches_golden(name: str, actual) -> None:
    path = GOLDEN_DIR / f"{name}.json"
    # round-trip so tuples/enums compare like the stored JSON
    actual = json.loads(json.dumps(actual))
    if os.environ.get("UPDATE_GOLDEN") == "1":
        GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(actual, indent=1, sort_keys=True) + "\n")
        return
    assert path.exists(), f"missing golden file {path}; run with UPDATE_GOLDEN=1 to create it"
    expected = json.loads(path.read_text())
    assert _same(actual, expected), f"output differs from golden file {path}"
