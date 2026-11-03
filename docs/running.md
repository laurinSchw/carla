# Running, testing, recording

Run everything from the repository root. Commands marked **executed** were run on the development host (2026-11-03,
Python 3.12 venv, Node 22) while writing this page; the others are marked **not executed here**. `AGENTS.md` is the
reference for invariants and the live lab setup.

## Install

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt              # includes the carla==0.9.16 client and uvicorn[standard] (brings PyYAML)
pip install -e ".[dev]"                      # pytest, httpx, ruff, pyright, pre-commit
```

The `carla` package is only needed for a real simulation. The fake world, the unit/contract tests and the Node tests
run without it (some geometry tests compare against `carla.Transform` and are skipped without it).

## Start the server

Docker is the normal way (`AGENTS.md` section 4, `DOCKER.md`):

```bash
cp .env.example .env            # set ADMIN_TOKEN (compose refuses to start without it)
GIT_SHA=$(git rev-parse --short HEAD) docker compose up -d --build frontend
```

Locally (**not executed here** against CARLA):

```bash
python start_server.py --live --carla-host 192.168.0.104 --carla-port 2002 --admin-token t0k --port 8000
```

`--live` connects to CARLA at start; without it only the web server runs and the admin connects CARLA from `/admin` or
`POST /admin/connect`. `python start_server.py --help` lists every option (`--seed`, `--v2x-range`, `--v2x-rate`,
`--v2x-penetration`, `--no-v2x`, `--no-drivers`, `--v2x-advisory`, `--tm-port`, ...). `python -m src.cli serve <same options>`
does the same. Only one server may be connected to a CARLA world (`AGENTS.md` section 6b); stop servers with SIGINT.

### Without CARLA: the fake world (executed)

```bash
python start_server.py --fake --port 8765 --admin-token t0k --host 127.0.0.1
curl -s 127.0.0.1:8765/api/status          # {"connected":true,"map":"Carla/Maps/FakeTown",...,"fake":true}
```

`--fake` implies `--live`, registers `src/testing/fake_carla` as the `carla` module before anything imports it, and spawns
`--fake-vehicles` (12) background vehicles and `--fake-pedestrians` (6) pedestrians. The fake is a deterministic synthetic
city (3 x 3 grid, 9 signalised junctions, vehicles on fixed rings, walkers on sidewalks, a ray-cast LiDAR against boxes) behind
the subset of the CARLA API this server uses. It is **not a simulator**: no physics, no collisions, vehicles ignore traffic
lights and each other, only the Traffic Manager speed offset changes their motion. It says so in `/api/status` (`"fake": true`)
and in every run manifest (`"fake_carla": true`). Never use its numbers as results; use it for frontend work, smoke tests and
the reproducibility check of the server's own code.

## Record a run

### Headless and scripted (executed on the fake world)

```bash
python -m src.cli run scenarios/crossing.yaml --runs-dir /tmp/runs        # the scenario has fake: true
python -m src.cli run scenarios/crossing.yaml --runs-dir /tmp/runs        # a second time
python -m src.cli compare /tmp/runs/<first-run-id> /tmp/runs/<second-run-id>   # exit 0 = identical data files
```

`src/scenario.py` is the scenario model (pydantic; YAML or JSON): seed, `WorldConfig` fields, population, what to record
(`links`, `raw_lidar`) and commands with the tick at which each runs (`inject_denm`, `cancel_denm`, `lidar_request`,
`lidar_release`, `set_config`, `rsu`, `spawn_traffic`, `clear_traffic`, `spawn_pedestrians`, `clear_pedestrians`). The world
starts paused; spawning and the start of the recording happen on the simulation thread before the first tick, then the loop
runs without real-time pacing (`realtime: false`) and each command runs just before its tick (`PersistentCARLAServer.at_tick`).
A failing command is logged, recorded as event `scheduled_command_failed`, and makes the CLI exit with 2.

On the fake world two runs of the same scenario give byte-identical `v2x_links.jsonl.gz`, `stations.jsonl`, `lidar_meta.jsonl`
and raw LiDAR frames (`tests/e2e/test_scenarios.py`); a different seed gives different files. On real CARLA (drop `fake: true`
or set `host`/`port`) this is **not verified**: see `docs/science/limitations.md`. The run formats are in
`docs/science/recording.md`. The Docker image does not contain `scenarios/`.

### Interactive, on a running server (executed on the fake world)

```bash
curl -s -X POST -H "X-Admin-Token: t0k" -H 'content-type: application/json' \
     -d '{"name":"manual","record":["links","stations"]}' 127.0.0.1:8765/api/runs
curl -s -X POST -H "X-Admin-Token: t0k" 127.0.0.1:8765/api/runs/current/stop
curl -s -H "X-Admin-Token: t0k" 127.0.0.1:8765/api/runs
```

The older `/admin/runs` routes do the same. Runs go to `runs/` (git- and docker-ignored). A `runs/` directory written by the
container is owned by root; a local server then cannot create runs there and `/api/runs` answers 503 `{"error": "storage"}`:
pass another directory (`WorldConfig.runs_dir`, `--runs-dir` for scenarios).

## Tests

```bash
python -m pytest tests -q -m "not carla"        # offline: unit, contract, golden, v2x, e2e on the fake world, Node tests
node --test tests/js/*.test.mjs                 # the JavaScript unit tests alone; fixture tests skip without JS_FIXTURES
ruff check src tests start_server.py
```

* `tests/unit`, `tests/contract`, `tests/v2x`, `tests/golden`: no CARLA, no network. Golden tests pin the channel and BSM
  outputs; regenerate only deliberately with `UPDATE_GOLDEN=1` and describe why in `docs/science/CHANGELOG.md`.
* `tests/unit/test_js_modules.py` writes fixtures from the Python side (poses and points from `src/lidar/transforms.py`,
  LiDAR v3 frames from `src/lidar/protocol.py`, `config_schema()`, `lidar_schema()`, `DENMCauseCode`) and runs
  `tests/js/*.test.mjs` under Node with them; it also decodes Python-encoded frames with the browser decoder. Skipped
  without Node (e.g. in the Docker image).
* `tests/e2e`: headless scenarios and `start_server.py --fake` as subprocesses (HTTP, hub WebSocket, LiDAR). No browser tests.
* `-m carla` selects tests that need a live CARLA 0.9.16 (`tests/integration`, `tests/server`); see `AGENTS.md` section 6 for
  the command line with `SERVER_URL`, `CARLA_HOST=192.168.0.104`, `CARLA_PORT=2002`. **Not executed here.**

## Manual check against live CARLA (not executed here)

After a change that touches the simulation thread, LiDAR or the V2X pipeline, on the lab machine:

1. `curl -s localhost:8000/api/status` shows `"connected": true`, a map name, `"fake": false` and a growing `frame`.
2. Spawn 25 vehicles and 20 pedestrians; `GET /admin/status` → `timing_ms` shows where a tick goes.
3. Join a vehicle (`POST /client/join`), subscribe on `/ws` with its `session_token` to `session.self` and `v2x.inbox`: the inbox
   holds only what the radio delivered.
4. Request a LiDAR (`PUT /client/lidar/{session_id}`): the state goes `requested → spawning → warming_up → active`.
5. Inject a DENM (`POST /api/v2x/denm`): it appears in the receivers' inboxes and in the link records with its channel outcome.
6. Disable an RSU (`PATCH /api/v2x/rsus/{station_id}`): its MAP/SPAT stop at the next V2X update.
7. Record a run for 30 s and check `runs/<id>/manifest.json`: `status` is `finished`, `fake_carla` is `false`.
8. Run a scenario twice against CARLA (`fake: false`, `host`/`port` of the lab PC) and `python -m src.cli compare` the two runs;
   write the result (identical or the first differing file) into `docs/science/limitations.md` and the CHANGELOG.
