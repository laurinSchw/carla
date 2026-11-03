# AGENTS.md — CARLA V2X Research Platform

Single entry point for AI agents working in this repo. It replaces `.instructions.md` and
`docs/agents/*`. Last verified against branch `developer` @ the V2V→V2X merge (2026-11-03).
If something here disagrees with the code, the code wins; fix this file.

## 1. What this is
A **persistent CARLA 0.9.16 world** runs in a background thread. A **FastAPI server** (port 8000)
lets clients join (each controls one "leading vehicle"), lets an admin spawn traffic and
pedestrians, and streams telemetry, V2X state and LiDAR. Inside the tick loop an in-process
**V2X network** models SAE J2735 BSM broadcast over an IEEE 802.11p (DSRC) channel, plus V2I
(RSU: MAP/SPAT) and V2P (VRU awareness). It is a research instrument: scientific accuracy and
reproducibility come before convenience.

## 2. Status (read before planning work)
The repo is mid-refactor, driven by `REFACTOR_HANDOFF.md` (rev 2, 891 lines — the plan, defect
list S1–S20/C1–C13, target architecture and phase acceptance criteria).

| Phase | State | Where |
|-------|-------|-------|
| 0 Safety net (golden tests, lazy `import carla`) | done | `developer` |
| 1 Remove monolithic V2V demo plane | done | `developer` |
| 2 Core skeleton (single sim thread owns CARLA via `SimThreadExecutor`, `WorldConfig`, locked `SessionManager`, 5 Hz WS, contract tests) | **done and merged into `developer`** (2026-11-03). Still open from the Phase 2 list: package layout `sim/`, `api/`, `config.py` (§5.2) — the code still lives in `src/server/` | `developer` |
| 3 V2X unification | **step 1 done** (`src/v2v` merged into `src/v2x`, one `V2XNetwork`, golden output identical). Model-change steps (S2, S4, S7 …), channel docs and `LinkResult` are **not started** | `developer` |
| 4–8 LiDAR rework, WS hub, new frontend, recording, docs | not started | — |

Remaining defects from the handoff list are in `REFACTOR_HANDOFF.md` §3.1 (check which are fixed by Phase 2 before touching them).
Decisions D1–D13 in the handoff (§7) belong to the human; ask, don't guess.

## 3. Layout
```
start_server.py                 CLI entry (argparse → uvicorn). Run from repo root.
src/server/
  app.py                        FastAPI app; serves /, /admin, /client; mounts routers
  admin_routes.py               /admin/* (token via ADMIN_TOKEN): connect, status, sessions,
                                traffic, pedestrians, stop, v2x/{rsus,vrus,spat}
  client_routes.py              /client/{blueprints,join,leave,vehicle}, WS /ws/client/{id}
                                (telemetry + v2x), WS /ws/lidar/{id} (binary)
  persistent_world.py           PersistentCARLAServer: connect, tick thread, spawn/despawn,
                                V2X update every 10 frames, per-client LiDAR
  session_manager.py            ClientSession + SessionManager
src/v2x/
  dsrc_channel.py               DSRCConfig, DSRCChannel (802.11p: path loss, shadowing, LOS/NLOS,
                                CSMA term [EXPERIMENTAL], PRR sigmoid), seeded RNG
  safety_messages.py            BSMCore/BSMPartII, CAM, DENM, threat level, create_bsm_from_carla
  messages.py                   MAP, SPAT, CPM (data only), VRUAM, VRUProfile
  rsu.py                        RoadSideUnit (MAP/SPAT from CARLA traffic lights)
  network.py                    V2XNetwork: vehicle/RSU/VRU registries, 2 Hz update, neighbours,
                                threats, DENM store
src/visualization/lidar/collector.py    LiDARDataCollector (per-client)
src/visualization/web/                  admin_panel.html, client_join.html (stopgap UIs, replaced in Phase 6)
src/utils/binary_protocol.py            LiDAR wire format
tests/                          golden/ unit/ v2x/ + loose lidar tests (see §6)
docs/                           science/CHANGELOG.md (model changes), science/audit.md (stale backlog),
                                V2V_*.md (older human docs, partly stale), ER_DATABASE_MODEL.*
```
There is no `v2v` package any more and no `src/config.py`. Tunables live in dataclasses next to
their code (`DSRCConfig`, `RSUConfig`); a single validated config object is planned (handoff §5.2).

## 4. Run
Docker is the normal way to run this.
```bash
cp .env.example .env            # set ADMIN_TOKEN (compose refuses to start without it)
docker compose up -d --build frontend
docker compose logs -f
# http://localhost:8000   /admin (token)   /client (open)
```
Local: `python start_server.py [--live --carla-host H --carla-port 2000 --admin-token T
--port 8000 --no-v2x --v2x-range 75 --tm-port 8001]`. Without `--live` only the web UI runs;
the admin connects CARLA from `/admin`. Run from the repo root (`src.` imports).
CARLA itself runs elsewhere (`CarlaUE4 -carla-rpc-port=2000`); compose sets `CARLA_HOST`/`CARLA_PORT`.
The container image is Python 3.10 (`Dockerfile`, `requirements.docker.txt`).

## 5. Invariants — do not break
- **Fixed timestep** `fixed_delta_seconds = 0.05` (20 FPS), synchronous mode, substepping
  `0.05 <= max_substep_delta_time * max_substeps` (0.01 × 10). `use_hybrid_physics` stays `False`.
- **Traffic Manager port 8001** ≠ web port 8000. Seeds explicit (default 42).
- **Fresh data:** read velocity/transform from the tick's `snapshot.find(id)`, not
  `actor.get_velocity()` (cached from the previous tick). Label any fallback as inaccurate.
- **V2X rate 2 Hz** = every 10 frames at 20 FPS; the network also gates on sim time.
- **Thread safety:** actor lifecycle changes go through `PersistentCARLAServer._lock` (and, after
  Phase 2, only on the simulation thread). Don't block the tick loop (LiDAR cleanup runs in a daemon thread).
- **Import `carla` lazily** (`TYPE_CHECKING` / inside functions) in `src/v2x` and the collector so
  unit tests run without CARLA.
- **Scientific honesty:** never invent citations or standard section numbers — write "unverified".
  Known unverified/contradicted items: CSMA constants vs. their docstring (code gives PRR factor
  0.78/0.61/0.37 at N≈50/100/200, docstring says 0.85/0.70/0.50); "ETSI EN 302 663 for 23 dBm";
  NLOS = vehicles within 2.5 m of the link (buildings are ignored); BSM `latitude/longitude`
  carry CARLA x/y (S8).
- **Any change that can alter results** needs a `docs/science/CHANGELOG.md` entry with before/after
  numbers and a deliberate `UPDATE_GOLDEN=1` regeneration. Pure refactors must leave golden output identical.

## 6. Tests
Run in the venv or container, from the repo root: `python -m pytest tests -q -m "not carla"`. Markers: `carla` (live CARLA), `server` (deployed server, needs `SERVER_URL`).
- `tests/golden/` — pin `DSRCChannel` (seed 42) and `create_bsm_from_carla` output in
  `tests/golden/expected/*.json` (tracked). They detect change, not correctness; they pin known defects.
  Regenerate only on purpose: `UPDATE_GOLDEN=1 pytest tests/golden`.
  **In the Docker image (Python 3.10) 3 golden DSRC cases (`grid100`, `line10_no_nlos_model`,
  `pair_30m`) fail even on the pre-refactor commit** — the files were generated on Python 3.12.
- `tests/v2x/` — channel, BSM/DENM/CAM, V2X messages/registries, neighbour discovery, scientific integrity.
- `tests/unit/` — CSMA pins, LiDAR protocol. `tests/test_v2v_lidar.py` — collector transforms.
- `tests/contract/` — API contract tests against `FakeWorld` (no CARLA). `tests/unit/test_sim_executor.py`, `test_persistent_world_threading.py`, `test_world_config.py` — Phase 2 threading.
  `tests/integration/` (`carla`) and `tests/server/` (`server`) — live/deployed smoke tests.
- Baseline (host, `.venv`, `-m "not carla"`): 258 passed, 11 skipped, 5 xfailed (strict, stale by design: CSMA drops at N=100,
  neighbour tests that never call `broadcast_all`).
- Known weak tests: `test_lidar_enhanced.py` (untracked; path bug, tests its own mocks),
  rotation/JSON tests in `test_v2v_lidar.py`, several "stored value" asserts in
  `test_v2x_messages.py`, mock vehicle/world classes duplicated across four files (needs a conftest).

## 7. Known gaps (v2x)
- `CPMMessage` is defined but nothing generates it.
- MAP/SPAT/VRUAM bypass the DSRC channel (no loss/latency); only BSM goes through it.
- `admin_routes.py` / `persistent_world.py` reach into private `_rsus`/`_vrus` via `getattr`.
- `binary_protocol.py` is still the V1/V2-style format (v3 planned, handoff §5.4).
- Untracked, unreviewed: `collector_enhanced.py`, root stress/accuracy scripts, `STRESS_*`/`LIDAR_*` summaries.

## 8. Conventions
- Absolute imports from `src`; relative imports inside a subpackage.
- Type hints on signatures; Pydantic models for REST bodies; `logging.getLogger(__name__)`.
- Frontend is static HTML, no build step.
- Docstrings cite SAE J2735 / ETSI / IEEE 802.11p sources only when verified.
- Branches: `developer` is the dev line (tracks `gitlab/developer`, GitLab is the team source of truth;
  `origin` is a GitHub fork). Refactor phases go on `refactor/pN-*` branches per the handoff.
  Commits are authored by the human; do not add AI co-author trailers unless asked.
- Before editing, check `REFACTOR_HANDOFF.md` — a file may already be marked DELETE/REPLACE.
- Docs: `docs/V2V_*.md` and `docs/science/audit.md` predate the merge and use old names (`v2v/…`); don't copy their paths.
