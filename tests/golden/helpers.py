"""Golden-file helper: compare a JSON-serialisable value with a stored file.

Regenerate deliberately with ``UPDATE_GOLDEN=1 pytest tests/golden`` and review the diff.
A golden change is either a bug in a refactor or a deliberate model change that needs a
docs/science/CHANGELOG.md entry with before/after numbers.
"""

import json
import os
from pathlib import Path

GOLDEN_DIR = Path(__file__).parent / "expected"


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
    assert actual == expected, f"output differs from golden file {path}"
