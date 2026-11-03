"""Generates ``docs/openapi.json`` and ``docs/api.md`` from the app definition.

    python -m src.server.api_docs           # write both files
    python -m src.server.api_docs --check   # exit 1 if they are out of date (tests/unit/test_api_docs.py does this)

The OpenAPI file is committed so that a change of the API shows up as a diff in review. REST is described by
OpenAPI; the WebSocket protocols (which OpenAPI cannot express) are written out below, the hub topics from the topic
table. The generated text depends on the FastAPI / Pydantic versions; ``info.x-generated-with`` records them.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

DOCS = Path(__file__).resolve().parents[2] / "docs"

METHODS = ("get", "post", "put", "patch", "delete")

WS_INTRO = """\
## WebSocket `/ws` (JSON, one connection per browser tab)

Client to server (text frames, at most 16 KiB):

| message | meaning |
|---|---|
| `{"op": "auth", "admin_token": "..."}` or `{"op": "auth", "session_token": "..."}` | become admin / the vehicle of that session. Without an auth message the connection is an *observer* (if `observer_public`). A wrong credential closes the socket with code 4401. Tokens are never put in a URL. |
| `{"op": "subscribe", "topic": "...", "max_hz": 2, "filter": {...}}` | start (or replace) a subscription. `max_hz` is capped by the topic. |
| `{"op": "unsubscribe", "topic": "..."}` | stop it |
| `{"op": "ping"}` | answered with a `pong` message |

Server to client:

* `{"topic": "<name>", "frame": <CARLA frame>, "sim_time": <s>, "seq": <n>, "data": {...}}`.
  `seq` is the publication counter of that topic: if it jumps by more than one the client skipped updates (delivery
  is *latest wins per connection and topic*; a slow client never slows the simulation, and a client that cannot
  accept a message for `send_timeout_s` is disconnected with code 1013).
* `{"topic": "error", "code": "...", "message": "...", "request": {...}}`: `bad_message`, `bad_request`,
  `unknown_topic`, `forbidden`, `filter_required`, `bad_filter`, `auth_failed`, `admin_not_configured`, `internal`.
* control replies on the topics `auth`, `subscribed`, `unsubscribed`, `pong`.

### Topics
"""

TOPIC_NOTES = """\
Notes on the data:

* `v2x.links` carries `{"columns": [...], "rows": [[...]], "matching": n, "total_links": N, "max_rows": cap,
  "truncated": bool}`; `columns` are the fields of a link result (see docs/science/model.md). A filter with at
  least one of `station_ids`, `msg_types`, `link_types`, `outcome` is required, otherwise the subscription is
  refused (`filter_required`): one V2X update with 100 equipped vehicles has about 10,000 links.
* `v2x.inbox` needs `{"station_id": n}`; a vehicle may only ask for its own station (the default). `messages` are
  what the station received (with the link result that delivered each), `sent` what it transmitted in that update,
  `threats` are computed from its inbox.
* `v2x.stats` has two blocks, `window` (the last `window_s` = 60 s of simulation time) and `run` (everything since
  the world started), both with PRR per 10 m distance bin (LOS and NLOS apart, Wilson 95 % interval), outcome counts
  per link type and receive-margin histograms.
* `world.state` takes an optional `{"kinds": ["vehicle", "pedestrian"]}`.
* `events` starts with the last 50 events, then sends only new ones (`dropped: true` if the log overflowed).
* Station ids: vehicles and pedestrians use the CARLA actor id, an RSU `-intersection_id`, a fixed DENM
  transmitter `-1000001` and below.
"""

LIDAR_WS = """\
## WebSocket `/ws/lidar/{sensor_id}` (binary protocol v3)

Views one sensor. Binary messages are LiDAR frames (header, sensor pose, points; see `src/lidar/protocol.py` and
`src/visualization/web/static/js/lidar-decoder.js`). Text messages are JSON:
`{"type": "state", "state": "...", "frame": n, "error": ...}`, `{"type": "stream_profile", ...}` (what the viewer
actually gets: quantised `max_points`, `max_hz`, `every_n_frames`, `compress`, `points_total`, `points_sent`,
`subsampled`), `{"type": "auth", "role": "..."}`, `{"type": "pong"}` and `{"type": "error", "code": "...",
"message": "..."}`. The viewer may send `{"op": "profile", "max_points": 25000, "max_hz": 10, "compress": true}`,
`{"op": "auth", "admin_token" | "session_token": "..."}` and `"ping"` / `{"op": "ping"}`.

Who may view: the admin always; anyone while `observer_public` is on; otherwise a vehicle only the sensors on its
own vehicle. With `observer_public` off a viewer must send the auth message within 5 s. Close codes: 4401 wrong
credential, 4403 not allowed to view, 4404 no such sensor; the socket closes (1000) once the sensor is destroyed or
in error. Closing the socket does not release the sensor: it is released `keep_alive_s` after its last viewer left
unless it is pinned or recorded.

The same path with a session id (`/ws/lidar/{session_id}`, the `lidar_ws_url` of `POST /client/join`) is the older
per-session stream of the `/client` pages: query `max_points`, `max_hz`, `compress`; no authentication.
"""


def openapi() -> Dict[str, Any]:
    import fastapi
    import pydantic

    from src.server.app import app
    spec = copy.deepcopy(app.openapi())
    spec["info"]["x-generated-with"] = {"fastapi": fastapi.__version__, "pydantic": pydantic.VERSION}
    return spec


def render_openapi_json(spec: Dict[str, Any]) -> str:
    return json.dumps(spec, indent=2, sort_keys=True) + "\n"


def _schema_summary(schema: Dict[str, Any], spec: Dict[str, Any]) -> str:
    if not schema:
        return ""
    if "$ref" in schema:
        name = schema["$ref"].rsplit("/", 1)[-1]
        props = spec["components"]["schemas"].get(name, {}).get("properties", {})
        return f"`{name}` ({', '.join(props)})" if props else f"`{name}`"
    if "anyOf" in schema:   # Optional[Model]
        parts = [_schema_summary(s, spec) for s in schema["anyOf"] if s.get("type") != "null"]
        return " or ".join(p for p in parts if p) + " (optional)"
    t = schema.get("type", "")
    return f"`{t}`" if t else ""


def render_markdown(spec: Dict[str, Any]) -> str:
    from src.server.topics import TOPICS
    lines: List[str] = [
        "# API",
        "",
        "Generated by `python -m src.server.api_docs`; do not edit by hand. The machine-readable source is "
        "`docs/openapi.json`.",
        "",
        "Authentication: admin endpoints take the header `X-Admin-Token`; vehicle endpoints take `X-Session-Token` "
        "(the `session_token` from `POST /client/join`). Read-only endpoints without a credential work for "
        "observers unless the server runs with `observer_public = false`.",
        "",
    ]
    by_tag: Dict[str, List[str]] = {}
    for path in sorted(spec["paths"]):
        for method in METHODS:
            op = spec["paths"][path].get(method)
            if not op:
                continue
            tag = (op.get("tags") or ["other"])[0]
            params = list(op.get("parameters", []))
            hdrs = [p["name"] for p in params if p["in"] == "header"]
            others = [f"{p['name']} ({p['in']})" for p in params if p["in"] != "header"]
            body = op.get("requestBody", {}).get("content", {}).get("application/json", {}).get("schema", {})
            ok = [c for c in op.get("responses", {}) if c.startswith("2") or c == "304"]
            entry = [f"### `{method.upper()} {path}`", ""]
            if op.get("summary"):
                entry += [op["summary"], ""]
            if hdrs:
                entry.append(f"* headers: {', '.join('`' + h + '`' for h in hdrs)}")
            if others:
                entry.append(f"* parameters: {', '.join(others)}")
            if body:
                entry.append(f"* body: {_schema_summary(body, spec)}")
            entry.append(f"* success: {', '.join(ok)}")
            entry.append("")
            by_tag.setdefault(tag, []).extend(entry)
    for tag in sorted(by_tag):
        lines += [f"## REST: {tag}", ""] + by_tag[tag]
    lines += [WS_INTRO]
    lines += ["| topic | roles | max Hz | filter | ground truth | description |", "|---|---|---|---|---|---|"]
    for t in TOPICS.values():
        lines.append(f"| `{t.name}` | {', '.join(sorted(t.roles))} | {t.max_hz:g} | "
                     f"{'required' if t.filter_required else 'optional' if t.name == 'world.state' else 'none'} | "
                     f"{'yes' if t.ground_truth else 'no'} | {t.description} |")
    lines += ["", TOPIC_NOTES, "", LIDAR_WS]
    return "\n".join(lines).rstrip("\n") + "\n"


def expected_files() -> Dict[Path, str]:
    spec = openapi()
    return {DOCS / "openapi.json": render_openapi_json(spec), DOCS / "api.md": render_markdown(spec)}


def main(argv: List[str]) -> int:
    files = expected_files()
    if "--check" in argv:
        stale = [str(p) for p, text in files.items() if not p.exists() or p.read_text() != text]
        if stale:
            print("out of date (run: python -m src.server.api_docs): " + ", ".join(stale))
            return 1
        return 0
    for path, text in files.items():
        path.write_text(text)
        print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
