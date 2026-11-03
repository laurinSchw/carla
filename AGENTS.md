# AGENTS.md — CARLA V2X Research Platform

Single entry point for AI agents working in this repo. It replaces `.instructions.md` and
`docs/agents/*`. Last verified against branch `developer` after the independent review fixes (2026-11-03).
If something here disagrees with the code, the code wins; fix this file.

## 1. What this is
A **persistent CARLA 0.9.16 world** runs in a background thread. A **FastAPI server** (port 8000)
lets clients join (each controls one "leading vehicle"), lets an admin spawn traffic, pedestrians and hazard
warnings, records runs, and streams telemetry, V2X state and LiDAR. Inside the tick loop an in-process
**V2X network** models BSM broadcast over a simplified IEEE 802.11p (DSRC) channel, plus V2I (RSU: MAP/SPAT),
V2P (VRU awareness) and DENM hazard warnings; every message type goes through the same channel model and leaves
one `LinkResult` per attempt. Background vehicles are driven by CARLA's Traffic Manager with seeded driver
profiles, optionally slowed by what they RECEIVED over V2X. It is a research instrument: scientific accuracy
and reproducibility come before convenience.

## 2. Status (read before planning work)
Driven by `REFACTOR_HANDOFF.md` (rev 2: plan, defect list S1–S20 / C1–C13 / L1–L9, target architecture).

| Phase | State |
|-------|-------|
| 0 Safety net, 1 Remove monolithic plane, 2 Core skeleton (one simulation thread owns CARLA) | done |
| 3 V2X | done except the open model items below. All S-defects fixed or documented honestly in `docs/science/model.md` |
| 4 LiDAR on request (`src/lidar/`) | done and live-verified. Tick p95 with 3 sessions + 70 actors was 79.6 ms (target 50 ms, not met: CARLA's `world.tick()` dominates on the lab PC) |
| 7 Recording (`src/recording/`, `POST /admin/runs`) | done in a first version (see `docs/science/recording.md`); frame-scheduled commands / bit-identical repeats are not |
| Background drivers + V2X advisory (`src/traffic/`) | done; effect size of the advisory not measured |
| 5 WS hub + roles (`/ws`, topics, session tokens, grace D4), 6 new frontend (`/`, `/observer`, `/vehicle`, `/admin`, Three.js LiDAR view), `/api/*` + `/api/lidar/*` routes, `docs/api.md` | ported from the archived branch (tag `archive/refactor-handoff`); not live-verified on CARLA |
| Fake CARLA (`start_server.py --fake`), headless scenarios (`python -m src.cli run scenarios/crossing.yaml`), JS Node tests (`tests/js`, runner `tests/unit/test_js_modules.py`) | done; same-seed determinism is shown on the fake world only (`docs/science/CHANGELOG.md`) |

Open model items (need decision D5 or a reference dataset): i.i.d. shadowing (S5), the MAC term is a placeholder (S6), no building
occlusion (S20), CPM generation, ETSI CAM triggering rules, cooperative perception (`enable_bidirectional_sharing` is a stub that
shares nothing). Decisions D1–D13 in the handoff (§7) belong to the human; ask, don't guess.

## 3. Layout
```
start_server.py                 CLI entry (argparse → uvicorn). Run from repo root.
src/server/
  app.py                        FastAPI app; pages / (index), /observer, /vehicle, /admin, /client (redirect to /vehicle),
                                /static; shuts the world down on exit
  admin_routes.py               /admin/*: connect (seed, drivers, advisory, ...), status (+timing_ms), sessions, traffic,
                                pedestrians, stop, runs, v2x/{rsus,vrus,spat,links,hazards}   (token: ADMIN_TOKEN)
  client_routes.py              /client/{blueprints,join,leave,lidar/...}, WS /ws/client/{id}, WS /ws/lidar/{id} (binary v3)
  persistent_world.py           PersistentCARLAServer: connect, tick thread, spawn/despawn, V2X update every 10 frames,
                                LiDAR, recording, driver profiles, advisory, hazards
  session_manager.py, sim_executor.py, route_utils.py, world_config.py (validated WorldConfig)
src/v2x/                        dsrc_channel.py (DSRCConfig, DSRCChannel, LinkResult), network.py (V2XNetwork: registries,
                                inboxes, hazards, link log), safety_messages.py (BSM/CAM/DENM, threat level), messages.py
                                (MAP/SPAT/VRUAM/CPM data), rsu.py
src/lidar/                      protocol.py (v3 wire format), config.py, manager.py (lifecycle, budget, profiles), carla_backend.py
src/recording/recorder.py       run directory writer + tolerant link reader
src/traffic/                    drivers.py (seeded profiles → Traffic Manager settings), advisory.py (V2X-assisted slow-down)
src/visualization/web/          index.html, observer.html, vehicle.html, admin.html (+ static/js ES modules, static/css, static/vendor)
tests/                          golden/ unit/ v2x/ contract/ integration/ (carla) server/ (server+carla)
docs/science/                   model.md (what the channel does), recording.md (run formats), CHANGELOG.md (every result-changing
                                change with numbers), audit.md (superseded)
```
There is no `v2v` package and no `src/config.py`. Tunables live in dataclasses next to their code
(`DSRCConfig`, `RSUConfig`, `LidarLimits`, `DriverMix`, `AdvisoryConfig`, `WorldConfig`).

## 4. Run
Docker is the normal way to run this.
```bash
cp .env.example .env            # set ADMIN_TOKEN (compose refuses to start without it)
GIT_SHA=$(git rev-parse --short HEAD) docker compose up -d --build frontend    # the sha names recorded runs
# http://localhost:8000   /admin (token)   /observer, /vehicle (open)    recorded runs appear in ./runs/
```
Local: `python start_server.py [--live --carla-host H --carla-port 2002 --admin-token T --port 8000 --no-v2x --v2x-range 75
--v2x-rate 2 --v2x-penetration 1.0 --seed 42 --no-drivers --v2x-advisory --tm-port 8001]`. Without `--live` only the web UI runs;
the admin connects CARLA from `/admin` or `POST /admin/connect`. Run from the repo root. The image is Python 3.10.

## 5. Invariants — do not break
- **Fixed timestep** 0.05 s (20 FPS), synchronous mode, substepping `0.05 <= max_substep_delta_time * max_substeps`; `use_hybrid_physics` stays False.
- **Traffic Manager port 8001** ≠ web port 8000. **Seeds are explicit** (default 42); every random stream derives from the master seed.
- **One thread owns CARLA** (the simulation thread). HTTP/WebSocket threads only read published snapshots (copies / replaced objects) — never iterate a
  dict the simulation thread mutates (three such races were found and fixed in review).
- **Fresh data:** read velocity/transform/acceleration from the tick's `snapshot.find(id)`; a vehicle missing from the snapshot is skipped, not queried.
- **Never call blocking RPCs per update per vehicle** (`bounding_box`, `get_physics_control()` ≈ 1 ms each over a LAN): cache at registration.
- **Import `carla` lazily** in `src/v2x`, `src/lidar`, `src/traffic`, `src/recording` so unit tests run without CARLA.
- **Scientific honesty:** never invent citations or standard section numbers — write "unverified". Known unverified items are listed in
  `docs/science/model.md` (23 dBm / 33 dBm attributions, DENM cause-code numbers, MAC constants, VRU TX power).
- **Any change that can alter results** needs a `docs/science/CHANGELOG.md` entry with before/after numbers and a deliberate `UPDATE_GOLDEN=1` run.
- **Verify any CARLA API call against the live server** (`dir(tm)`): e.g. the TM method is `keep_slow_lane_rule_percentage`, not `keep_right_rule_percentage`.
- **A request that is valid but impossible in the current state is a 409 via `ConflictError` / a dedicated exception**, not a `RuntimeError`
  (`on_sim_thread` maps `RuntimeError` to 503).

## 6. Tests
From the repo root: `python -m pytest tests -q -m "not carla"` (venv or container). Markers: `carla` (live CARLA), `server` (deployed server, `SERVER_URL`).
Baseline: 591 passed, 13 skipped, 5 xfailed (host, with Node); 9 of those passes come from the untracked `tests/test_lidar_enhanced.py`. `docs/api.md`/`openapi.json` are generated: run `python -m src.server.api_docs` after changing routes (a test fails when stale). The 5 strict xfails are stale by
design (CSMA drops at N=100; neighbour tests that never call `broadcast_all`). Node-based tests skip without Node (e.g. in the image).
- `tests/golden/` — pin `DSRCChannel` (seed 42) and `create_bsm_from_carla` in `expected/*.json`; floats compare at 1e-9 relative. Regenerate only on purpose.
- `tests/v2x/` — channel, messages, infrastructure/DENM links, model fixes, the channel table pinned against `docs/science/model.md`.
- `tests/unit/` — LiDAR (protocol, manager, backend queue, geometry vs `carla.Transform`, JS decoder under Node), recorder, drivers/advisory, WorldConfig, and the
  real tick loop against the recording CARLA stub (`test_persistent_world_*.py`).
- `tests/contract/` — HTTP/WebSocket contract against a `FakeWorld` (no CARLA), including `/admin/connect` → WorldConfig, runs, hazards, LiDAR.
- Live: `SERVER_URL=http://127.0.0.1:8000 ADMIN_TOKEN=... CARLA_HOST=192.168.0.104 CARLA_PORT=2002 pytest tests/server tests/integration -m carla -s`
  (`LIVE_TRAFFIC` / `LIVE_PEDESTRIANS` size the load test; default 25 vehicles + 20 pedestrians).
- An independent review (Opus 5.5, 2026-11-03) broke 74 behaviours on purpose: the ones no test caught now have tests (`tests/v2x/test_review_survivors.py`, `test_persistent_world_science.py`).

## 6a. Science use
Runs are recorded with `POST /admin/runs` (`docs/science/recording.md`: formats, `iter_link_rows` loader, what is and is not reproducible). Set the seed per run with
`/admin/connect {"seed": N}`. Live reproducibility (15 vehicles, same seed twice): same drivers and spawn points, trajectories within ~cm for most vehicles over 20 s, one vehicle
3 m off — **not bit-identical**; average over several seeds. Hazards: `POST /admin/v2x/hazards`; vehicles react only to what they received (`advisory_enabled`).

## 6b. Live CARLA
The lab CARLA 0.9.16 PC is `192.168.0.104`, RPC **2002**, streaming 2003 (2000 is taken by something else). **Only one server may be connected to a CARLA world** (two servers both
tick it; observed 2x frame rate and timeouts): check the Docker container's `/admin/status` before starting a local server, and stop servers with SIGINT so world settings are restored.
`GET /admin/status` → `timing_ms` (tick, world_tick, lidar, v2x_update p50/p95/max) and `v2x_phase_ms` show where a tick goes. The lab world is shared: other people's actors and
sensors appear; live tests count only their own.

## 7. Known gaps
- Channel: zero latency, i.i.d. shadowing, placeholder MAC term (the ≈ 14 % loss at short range in live data is that term), NLOS ignores buildings — `docs/science/model.md` §4.
- V2X-aware driver: the mechanism is verified live; its effect on speeds/collisions is not measured (needs same-seed A/B over several seeds).
- Drivers: profiles are project defaults, not calibrated; no destination-driven routing (`set_route` / `set_path` exist in the live TM but need a route planner).
- Recording: gzip links grow with the square of the V2X stations (≈ 6 MB/minute at 25 vehicles + 5 RSUs); runs are owned by root when written by the container.
- LiDAR v3 field-mask bits were changed to match the UI decoder without bumping the version: raw LiDAR recorded before 2026-11-03 uses the old bits (new manifests carry `lidar_protocol`). Open: bump to v4?
- Open decisions: hub vs `/ws/client` behaviour at grace 0; whether `/client/*` should require the session token; which sensors a vehicle may view with `observer_public=false` (now: its own only).
- Untracked work files of the owner (hidden from `git add <dir>` by `.git/info/exclude`; never commit them without asking): `collector_enhanced.py` (depends on the removed
  collector), `tests/test_lidar_enhanced.py` (tests its own mocks), root stress/accuracy scripts, `STRESS_*`/`LIDAR_*` summaries, `docs/LIDAR_SERVER_IMPLEMENTATION.md` (old LiDAR design).

## 8. Conventions
- Absolute imports from `src`; relative imports inside a subpackage. Type hints on signatures; Pydantic models for REST bodies; `logging.getLogger(__name__)`.
- Frontend is static HTML + plain JS, no build step.
- Docstrings cite SAE J2735 / ETSI / IEEE 802.11p sources only when verified.
- Branches: `developer` is the dev line (tracks `gitlab/developer`; `origin` is a GitHub fork). Commits are authored by the human; do not add AI co-author trailers unless asked.
  Stage explicit paths (`git add <file>`), never a whole directory: the owner has untracked work files.
- Before editing, check `REFACTOR_HANDOFF.md` — a file may already be marked DELETE/REPLACE.
- `docs/V2V_*.md` and `docs/science/audit.md` predate the merge and use old names (`v2v/…`); don't copy their paths.
