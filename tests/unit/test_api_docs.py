"""docs/openapi.json and docs/api.md must match the code: an API change has to show up in the diff."""

import json

import pytest

pytest.importorskip("httpx")

from src.server import api_docs  # noqa: E402


def test_committed_api_docs_are_current():
    spec = api_docs.openapi()
    committed = api_docs.DOCS / "openapi.json"
    if committed.exists():
        recorded = json.loads(committed.read_text()).get("info", {}).get("x-generated-with")
        if recorded != spec["info"]["x-generated-with"]:
            pytest.skip(f"docs were generated with {recorded}, this environment has {spec['info']['x-generated-with']}: "
                        "the OpenAPI text differs between FastAPI/Pydantic versions")
    expected = {committed: api_docs.render_openapi_json(spec), api_docs.DOCS / "api.md": api_docs.render_markdown(spec)}
    stale = [p.name for p, text in expected.items() if not p.exists() or p.read_text() != text]
    assert not stale, f"{stale} out of date: run `python -m src.server.api_docs` and commit the result"


def test_every_route_has_a_summary_and_every_topic_is_documented():
    spec = api_docs.openapi()
    missing = [f"{m.upper()} {path}" for path, ops in spec["paths"].items() for m, op in ops.items()
               if not op.get("summary")]
    assert not missing, f"routes without a summary: {missing}"
    md = api_docs.render_markdown(spec)
    from src.server.topics import TOPICS
    assert all(f"`{name}`" in md for name in TOPICS)
    assert "/ws/lidar/{sensor_id}" in md and "4401" in md
    for path in ("/api/lidar/schema", "/api/lidar/sensors", "/api/lidar/sensors/{sensor_id}", "/api/sessions/{session_id}/lidar"):
        assert path in spec["paths"], path
    assert "patch" in spec["paths"]["/api/lidar/sensors/{sensor_id}"]
    assert "### `PATCH /api/lidar/sensors/{sensor_id}`" in md        # PATCH routes are listed too
