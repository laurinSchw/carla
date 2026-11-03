# Refactor Handoff — CARLA V2X + LiDAR Research Server

> **Audience:** the coding agent doing the refactor.
> **Author context:** this document comes from a full read of the `developer` branch at
> commit `29bca10` (2026-10-08). Every defect below cites `file:line` at that commit.
> **Revision 2** (2026-10-09): scope narrowed to the V2X server structure only; frontend spec added (§5.7).
> **Purpose of the codebase:** a *research* platform. Numbers it produces may end up in
> publications. Precision and traceability beat features and beat speed.

> **Revision 2 — scope decision by the researcher (overrides anything below that conflicts):**
> **Remove the V2V/monolithic structure entirely. Keep only the V2X server structure.**
> - The legacy monolithic demo (plane A in §2), and everything that exists only to serve it, is
>   **deleted, not ported**. Nothing from it stays at runtime. Its code may be read as reference
>   while you write the new modules, and that's all.
> - "V2V" stops being a package, module, API path, class name or UI label. Vehicle-to-vehicle
>   messaging (BSM/CAM/DENM between OBUs) **stays**, as one *link type* inside the single V2X
>   stack. **Do not delete the BSM, DSRC channel or threat code.** It moves into `v2x/` (§2.1).
> - The frontend is redesigned around the V2X server: §5.7 is the binding spec (LiDAR on request,
>   V2X views, data transport).

---

## 0. Read this first: rules of engagement

1. **This is research code. Wrong numbers are worse than no numbers.** If you are unsure about a
   unit, frame convention, standard clause, or CARLA API behaviour, don't guess. Write a test
   that settles it, or write `TODO(verify): …` and list it in your final report.
2. **Never invent citations.** Several existing citations are wrong or unverifiable (see §3.2).
   When you touch a docstring that cites a standard or paper, either check it against a primary
   source you can actually read or rewrite it as `"<claim> — source requires verification"`.
3. **Separate refactors from behaviour changes.** A commit is *either* a pure refactor (outputs
   are bit-identical, proven by golden tests) *or* a deliberate model/behaviour change (explained
   in `docs/science/CHANGELOG.md` with before/after numbers). Never mix them. The researcher must
   be able to attribute every change in results to a specific commit.
4. **Do not trust the existing docs.** `docs/agents/*.md`, `.instructions.md` and
   `docs/agents/audit.md` are partly wrong (for example, audit "P2 RESOLVED" is false, see §3.2-S1).
   Trust this handoff where it cites code, and verify it anyway.
5. **Work in phases (§6), one branch per phase, small commits.** Each phase has acceptance
   criteria. Don't start a phase until the previous one meets them.
6. **Decisions marked 🟨 DECISION in §7 belong to the human researcher.** Ask before acting
   on them. Use the stated default only if told to proceed without asking.
7. **Before deleting anything, tag the current state:**
   `git tag archive/pre-refactor-2026-10 29bca10 && git push origin archive/pre-refactor-2026-10`.
   Nothing is lost, so you can delete aggressively afterwards.

---

## 1. What the researcher wants (target)

> "V2X communication on a server level with LiDAR, controlled via frontend."

Concretely, the end state is:

- **One server process** connects to one CARLA 0.9.16 instance and owns the simulation
  (synchronous mode, fixed Δt). There is **one control plane**, not two (see §2).
- **Server-side V2X simulation:** every V2X-equipped station (vehicle OBUs, RSUs, VRUs) exchanges
  messages through **one** configurable channel model. Each delivery decision is computed and
  logged per link.
- **Server-side LiDAR, on request:** a LiDAR sensor exists only while someone requested it
  (or a run records it). The frontend can **attach, view, reconfigure and detach** it at runtime
  (sensor type, channels, range, points/s, FoV, mount pose). Each viewer picks its own stream
  decimation. Point clouds are **frame-synchronised** with the simulation and V2X state.
- **Frontend control** (full spec in §5.7):
  - *Admin page:* connect to CARLA, start/stop runs, traffic/pedestrians, V2X parameters,
    recording, LiDAR on any vehicle.
  - *Vehicle page:* join → own vehicle → the V2X view **as received by that vehicle's radio**
    (not ground truth) + LiDAR on request for the own vehicle.
  - *Observer page:* map with all stations, links (delivered/dropped and why), station inspector,
    channel analytics, and a view of any LiDAR stream that is already active.
- **Recording and reproducibility:** every run produces a run directory with a manifest (config,
  seeds, git SHA, CARLA map, start time) and per-frame data (V2X per-link results, station states,
  LiDAR frame metadata, optional raw point clouds), plus an event log of every admin/client action
  stamped with the **simulation frame** at which it was applied.

---

## 2. Current state: two control planes in one process

The repo grew two independent architectures that share one FastAPI app
(`src/visualization/lidar/server.py`) but **don't share state**:

| | **A. Legacy "demo scenario"** | **B. Persistent world (current direction)** |
|---|---|---|
| Entry | `POST /api/simulation/start` (`server.py:780`) → thread → `src/scenarios/v2v_complete_demo.py:run_simulation_headless` | `start_server.py --live` or `POST /admin/connect` (`admin_routes.py:96`) → `src/server/persistent_world.py` |
| V2X | `V2VNetworkEnhanced`/`V2XNetworkEnhanced`. **All** vehicles are registered | `V2XNetworkEnhanced`. **Only client vehicles** are registered (`persistent_world.py:495-496`) |
| LiDAR | Global `_collector` → `/ws` broadcast to everyone (`server.py:414-511`) → rendered by `viewer.html` (Three.js) | Per-session collector → `/ws/lidar/{sid}` (`client_routes.py:258`). **Not rendered anywhere:** `client_join.html:283-286` drops the binary frames |
| V2X REST | `/api/v2v/*` read the module global `_v2v_network` (`server.py:579-747`) | `set_persistent_world()` (`server.py:72-79`) **never sets `_v2v_network`**, so every `/api/v2v/*` endpoint and the pages `/v2v`, `/map`, `/vehicles`, `/view` show **nothing** in persistent mode |
| Frontend | `/` → `unified_viewer.html` → iframes `/control`, `/lidar`, `/v2v`, `/map` | `/admin`, `/client` |
| Logging | CSV observers, RunDirectory, seeded RNG (`v2v_complete_demo.py:115-116`) | **No data recording at all.** Unseeded `random` |
| Driving | BehaviorAgent (vendored `src/agents/`) or TM autopilot | TM autopilot only. The client cannot control anything about its vehicle |

Both planes can be started at the same time against the same CARLA server, and they will fight
over synchronous mode and ticking.

**Target (Revision 2):** **B is the only structure.** **A is deleted outright**: the route
handlers, the demo scenario, observers, utils, the standalone V2V API app and all A pages. A few
*capabilities* that A had and B lacks get **re-implemented fresh inside B's V2X server**, not
ported:

| Capability A had | Where it lives in the new V2X server |
|---|---|
| Seeded RNG | `config.seed` → per-subsystem `numpy.random.Generator` (§3.1-C8) |
| Run directory + CSV logs | `recording/` (§5.6), with a new schema. A's CSV columns are only a checklist |
| All vehicles transmit | `v2x.penetration_rate` (🟨 D2) |
| Three.js point-cloud renderer | Rewritten as `web/static/js/lidar-view.js` (§5.7.5). `viewer.html` is reference only: it allocates a new `BufferGeometry` every frame (`viewer.html:~600`) and its semantic palette likely uses pre-0.9.14 tag ids (§3.4-F8, verify) |
| Canvas top-down map | Rewritten as `web/static/js/map-view.js` (§5.7.4). The projection in `v2v_map.html:372-391` is correct and worth keeping (see §5.7.4) |

Naming rule: after Phase 3 step 1 (the move), `git grep -in "v2v"` matches only `docs/science/CHANGELOG.md`
history, the archive tag, and the string `"V2V"` as a link-type label (`link_type: "V2V" | "V2I" | "I2V" | "V2P" | "P2V"`).

### 2.1 File inventory and verdicts

Legend: **KEEP** (refactor in place) · **MOVE/MERGE** (same code, new location, golden-tested) ·
**SPLIT** (keep the plane-B part, delete the rest) · **DELETE** · 🟨 (needs a human decision, §7).
Revision 2: there is no "port" category. Plane-A code is deleted, never carried over.

| Path | Lines | Verdict | Notes |
|---|---:|---|---|
| `start_server.py` | 147 | KEEP→rewrite | Becomes a thin CLI (`serve`, `run`) |
| `src/config.py` | 135 | KEEP→rewrite | Duplicates `DSRCConfig` (also in `dsrc_channel.py:58`). Many values ignored (§3.1-C9) |
| `src/server/persistent_world.py` | 776 | KEEP→split | Core of the new `sim/` package |
| `src/server/session_manager.py` | 134 | KEEP | Add a lock. Docstring claims thread safety but there is no lock (`:64-74`) |
| `src/server/admin_routes.py` | 292 | KEEP→rewrite | Module-global `_world`, all `async def` doing blocking RPC |
| `src/server/client_routes.py` | 320 | KEEP→rewrite | Same problems. Wrong stream rates (§3.1-C6) |
| `src/v2v/messages.py` | 524 | **MOVE**→`v2x/messages/{bsm,cam,denm}.py` + `v2x/extract.py` | BSM/DENM/CAM dataclasses and the CARLA extractor (bugs §3.2). Pure move first (golden tests), then fixes |
| `src/v2v/dsrc_channel.py` | 418 | **MOVE**→`v2x/channel/dsrc_logdistance.py` | Physics core. Must expose per-link results (§5.3). Delete the duplicate `DSRCConfig` |
| `src/v2v/network_enhanced.py` | 526 | **MERGE**→`v2x/network.py` | Merged with `v2x/network.py` into one `V2XNetwork`. The V2V/V2X class hierarchy disappears |
| `src/v2v/__init__.py` | 20 | DELETE | Package `v2v` ceases to exist after the moves |
| `src/v2x/messages.py` | 233 | KEEP→split | SPAT/MAP/VRUAM/CPM → `v2x/messages/*.py` |
| `src/v2x/network.py` | 238 | KEEP→merge | Target of the merge above |
| `src/v2x/rsu.py` | 211 | KEEP | RSU grouping is wrong (§3.2-S7) |
| `src/visualization/lidar/collector.py` | 345 | KEEP→rewrite | Becomes `lidar/sensor.py`. Transform bug (§3.3-L1). It's used by plane B (`persistent_world.py:672`), so it's not a plane-A file |
| `src/utils/binary_protocol.py` | 237 | KEEP→v3 | Add frame number and sim time (§5.4) |
| `src/visualization/lidar/server.py` | 952 | SPLIT then DELETE | Only the plane-B wiring (router mounting, static assets) moves to `api/app.py`. **Delete outright:** `/api/simulation/*`, `/api/system/info`, `/ws` global stream + `stream_lidar_data`, every `/api/v2v/*`, `/v2v` HTML patching (`:278-361`), `set_collector`, `set_v2v_network`, the Pydantic `SimulationConfig` |
| `src/visualization/web/*.html` (all 8 pages) | ~3750 | DELETE (after Phase 6) | Replaced by the §5.7 frontend. Until Phase 6 lands, keep **only** `admin_panel.html` and `client_join.html` so the server stays usable. `viewer.html` and `v2v_map.html` are reference only (see §2) |
| `src/visualization/web/assets/*` | — | KEEP one logo | Five logo variants exist. Keep the one the pages use and move it to `web/static/img/` |
| `src/v2v/dashboard.html` | 549 | DELETE | Hardcodes port 8001 (= Traffic Manager port!), gets string-patched at runtime |
| `src/v2v/api.py` | 576 | DELETE | A second, standalone FastAPI app for port 8001. Nothing is ported |
| `src/visualization/lidar/api.py` | 240 | DELETE | `LiDARStreamingAPI` starts its own uvicorn. Plane A only |
| `src/lidar/*.py` | 31 | DELETE | `import *` shims around `src/visualization/lidar` |
| `src/visualization/__init__.py`, `src/web/__init__.py` | 31 | DELETE | Re-export shims, unused |
| `src/scenarios/` (incl. `v2v_complete_demo.py`, 1075 lines) | 1076 | DELETE | Monolithic demo. The headless runner (Phase 7) is new code on top of the V2X server, not a port |
| `src/utils/` (all 12 files, incl. `observers.py`, `__init__.py`) | ~3000 | DELETE | Plane A only, except `binary_protocol.py` (moves to `lidar/protocol.py`) and `potholes.py` (🟨 D1). `octree.py` is unused everywhere. `utils/__init__.py` eagerly imports `carla` |
| `src/agents/**` (vendored CARLA agents) | ~2000 | DELETE | Only the monolithic demo used them (`v2v_complete_demo.py:49`). The V2X server drives with TM autopilot. **D3 is resolved by Revision 2** |
| `src/utils/potholes.py`, `/api/potholes*`, `scripts/lidar_pothole_detection_check.py`, README pothole section, pothole drawing in tick loop | ~550 | 🟨 D1 | Use case **cancelled** in HEAD commit `29bca10`. Default: delete (it's in the archive tag) |
| `tests/**` | ~5300 | KEEP→reorganise | See §3.5 |
| `test_scientific_fixes.py` (root) | 164 | DELETE | Prints warnings but **asserts nothing**, so it always "passes". Turn the checks into real pytest asserts |
| `test_windows_access.sh`, `start_web_viewer.sh`, `run_v2v_lidar.sh` | — | DELETE | `start_web_viewer.sh:7` hardcodes `/home/workstation/carla`. Others are legacy |
| `.instructions.md`, `docs/agents/*`, `.github/agents/*.agent.md` | — | REPLACE | Stale. References the nonexistent `AGENTS.md` and git-ignored `LaurinNotes.md`. Replace with one `AGENTS.md` (and a `CLAUDE.md` that points to it) written in Phase 8 |
| `docs/ER_DATABASE_MODEL.*`, `docs/er_diagram.*` | — | 🟨 D6 | Describes a database that **doesn't exist** (no DB code in the repo). Either implement it as the recording schema (§5.6) or delete it |
| `docs/V2V_API_REFERENCE.md` | 388 | REPLACE | Documents the legacy API. Replace with an auto-generated OpenAPI export and a short hand-written guide |
| `DLR_KI_Logo_inverted_transparent_v2.png` (root, 1.4 MB) | — | DELETE | Duplicate of `src/visualization/web/assets/…`. Keep **one** logo in assets (there are 5 variants) |
| `requirements.txt` | — | REPLACE | Lists unused `open3d`, `opencv-python`, `matplotlib`, `pandas`, `selenium` (nothing in `src/` imports them). Move to `pyproject.toml` with extras `[dev]`, `[analysis]` |
| `pyrightconfig.json` | — | REPLACE | Disables almost every check. Enable basic mode on the new package |
| `Dockerfile`, `docker-compose.yml` | — | KEEP | `docker-compose.yml:15` hardcodes `ADMIN_TOKEN=201295`. Move it to `.env` (git-ignored) with `.env.example` |

---

## 3. Verified defects

Severity: 🔴 produces wrong data or corrupts the simulation · 🟠 broken feature or race ·
🟡 misleading or inconsistent · ⚪ hygiene.

"Evidence" says how each item was confirmed. Items marked **verify** rest on CARLA or standards
knowledge that you must confirm before fixing.

### 3.1 Concurrency, control flow, simulation integrity

| ID | Sev | Where | Defect | Evidence / fix direction |
|---|---|---|---|---|
| C1 | 🔴 | `persistent_world.py:335,356,388,404,448,770` | `client.apply_batch_sync(batch, True)` — the second argument is `do_tick`. In sync mode it **ticks the world from the HTTP thread** while `_tick_loop` also ticks. Extra unaccounted frames: `self._frame` drifts from the real frame, the 2 Hz gate misaligns, runs become irreproducible. | CARLA API: `apply_batch_sync(commands, do_tick=False)`. Fix: only the sim thread calls CARLA, with `do_tick=False`. All mutations go through a command queue (§5.1) |
| C2 | 🔴 | `persistent_world.py:403` | `spawn_pedestrians` calls `self.world.tick()` directly from the HTTP thread | Same as C1 |
| C3 | 🔴 | `persistent_world.py:243-276` | The tick loop **doesn't hold `_lock`**, so the "all mutations under lock" claim (`:15-19`) is false. HTTP threads mutate `_client_vehicles` and V2X registries while the tick thread iterates them (`network_enhanced.py:194`). Risks: `RuntimeError: dictionary changed size during iteration`, BSMs for destroyed actors | Single-owner thread model (§5.1) removes the need for most locks |
| C4 | 🔴 | `persistent_world.py:283-285` | `_check_expired_sessions` calls `_despawn_client_vehicle_locked` **without** the lock. The docstring at `:517` handwaves "or from tick thread" | — |
| C5 | 🟠 | `admin_routes.py`, `client_routes.py` (all handlers) | Every handler is `async def` but does blocking CARLA RPC (spawn, destroy, `apply_batch_sync`). That **blocks the asyncio event loop**, which freezes every WebSocket (telemetry and LiDAR) during a spawn | Make handlers enqueue a command and `await` an `asyncio.Future` resolved by the sim thread |
| C6 | 🟠 | `client_routes.py:234,240` / `:298,304` | Telemetry loop waits `wait_for(receive, 0.2)` **plus** `sleep(0.2)`, so it runs at ≤2.5 Hz, not the documented 5 Hz. LiDAR: 0.1 + 0.1 gives ≤5 Hz, not 10 Hz | Push model: the sim thread publishes per-frame data, WS tasks forward the latest data. A separate receive task handles pings |
| C7 | 🟠 | `persistent_world.py:255` + `network_enhanced.py:171-177` | The V2X update is gated **twice**: `frame % 10 == 0` **and** sim-time Δ ≥ 0.5 s. With float accumulation, Δ can be 0.4999… and the update is skipped. Simulated with accumulated `+= 0.05`: **1638 of 7200 updates skipped (23%)**, an actual rate of ≈1.54 Hz instead of 2 Hz | Script in §8. **Verify** on live CARLA: log `total_update_count / (frames/10)`. Fix: one gate, frame-count based (`frame % round(1/(rate*dt)) == 0`), and assert `1/(rate*dt)` is an integer at config validation |
| C8 | 🟠 | `persistent_world.py:310,323,331,383,732-733` | Module-global `random` is **never seeded** in plane B, so spawn points and blueprints aren't reproducible | One `numpy.random.Generator` per subsystem, derived from a master seed recorded in the manifest |
| C9 | 🟡 | `persistent_world.py:63-71,141-153` vs `config.py` | `src/config.py` is **ignored** by plane B: Δt, substeps, TM seed, speed are hardcoded or re-defaulted. Contradiction: `config.py:36` says `global_speed_difference = 30.0  # % slower`, plane B uses `-20.0` (20% **faster**, `persistent_world.py:67`) | One validated config object passed everywhere (§5.2) |
| C10 | 🟡 | `persistent_world.py:561-571` | Telemetry reads `actor.get_transform()/get_velocity()` from an HTTP thread and stamps it with `self._frame` **at read time**, not at the frame the data belongs to. Telemetry, V2X (2 Hz) and LiDAR carry no common frame id | Every published datum carries `frame` and `sim_time` from the `WorldSnapshot` |
| C11 | 🟡 | `client_routes.py:246-250` | Any WebSocket close (browser refresh, Wi-Fi blip) **despawns the vehicle and ends the session**. This contradicts `duration_seconds` sessions and the REST-only path | 🟨 D4 |
| C12 | 🟡 | `admin_routes.py:96-132` | `/admin/connect` swaps the module-global `_world` from a daemon thread with no synchronisation. Two quick clicks start two worlds against one CARLA | Single `WorldController` with explicit state machine `disconnected→connecting→running→stopping` |
| C13 | 🟡 | `session_manager.py:30,40,46,88` | `datetime.utcnow()` is deprecated in Python 3.12+. Use `datetime.now(timezone.utc)`. `server.py:400` `@app.on_event` is deprecated (use lifespan) | — |
| C14 | ⚪ | `server.py:117` | A Pydantic model named `SimulationConfig` shadows the dataclass `config.SimulationConfig` | Goes away with plane A |

### 3.2 V2X science and correctness

| ID | Sev | Where | Defect | Evidence / fix direction |
|---|---|---|---|---|
| S1 | 🔴 | `dsrc_channel.py:84-86`, `config.py:107-109`, `docs/agents/audit.md` "P2 RESOLVED" | **The documented calibration is numerically false.** It claims P_rx ≈ −71 dBm at 50 m LOS, −77 at 100 m LOS, −87 at 150 m NLOS. The code computes **−56.9, −63.9, −78.5 dBm** (L0 = 67.86 dB at d0 = 10 m, P_tx + 2G = 27 dBm). Expected channel PRR (shadowing averaged): 50 m LOS 1.000 · 100 m LOS 0.9999 · 150 m LOS 0.999 · 150 m NLOS 0.79 · **500 m LOS still 0.82**. Combined with the 75 m app-layer cutoff (S3), the radio model is **effectively lossless** for LOS in the operating region. The audit's "meaningful 10–150 m variation" claim is wrong | Reproduce with the §8 script. Fix: recalibrate against a cited measurement dataset, **or** state honestly what the model does. Add a pytest that pins P_rx(d) and E[PRR](d) to the documented table so the docs can't drift again |
| S2 | 🔴 | `v2v/messages.py:375` | `yaw_rate = math.degrees(angular_velocity.z)`. CARLA's `get_angular_velocity()` already returns **deg/s** (**verify** in the 0.9.16 Python API reference), so yaw rate is inflated ×57.3 | Fix and add a unit test with a mocked snapshot |
| S3 | 🔴 | `persistent_world.py:495-496`, `:420-423` | In plane B only client vehicles (typically 1–5) are V2X stations. Background traffic **never transmits**, so: (a) CSMA contender count N ≈ number of clients, so P_csma ≈ 1; (b) NLOS blockers are only other clients; (c) "neighbours" only exist between clients. Plane A registered all vehicles, so results differ between planes | 🟨 D2: market penetration rate. Make it config (`v2x.penetration_rate ∈ [0,1]`, seeded selection) |
| S4 | 🔴 | `v2x/network.py`, `persistent_world.py:598-632` | **V2I and V2P bypass the channel model.** SPAT "reception" is a geometric radius test (`dist <= broadcast_range_m`, 300 m). VRUAM "reception" is a hardcoded 100 m radius (`:625`). `RSUConfig.tx_power_dbm` (33 dBm) is **never used**. Only V2V BSMs go through `DSRCChannel` | All message types go through the same channel abstraction with per-station TX power and antenna height |
| S5 | 🟠 | `dsrc_channel.py:245-264,289-290` | Shadowing is drawn **i.i.d. per packet per tick** (no spatial or temporal correlation), then a logistic "PRR" is applied to that already-random P_rx and a second Bernoulli draw is made. Two problems: (1) uncorrelated shadowing overstates link volatility vs. measurement-based models (for example Gudmundson-type decorrelation, **verify** a source); (2) the sigmoid-on-a-sample double-randomises. The quantity called "SNR margin" is actually `P_rx − sensitivity`: no noise floor, no interference, so it's not an SNR | Pick one documented model. Expose the choice as config (`shadowing: iid / correlated`, `decision: threshold / sigmoid`). Rename `snr_margin` → `rx_margin_db` |
| S6 | 🟠 | `dsrc_channel.py:295-327` | CSMA term `max(0.35, exp(−0.005·(N−1)))`. λ = 0.005 and the 0.35 floor are **made up** ("calibrated" without data). The docstring table (`:303-306`: 0.95/0.85/0.70/0.50) contradicts the code comment (`:322`: 0.91/0.78/0.61/0.37). Citations: "Campolo et al. (2011)" has no title/venue. **"ETSI TR 102 863"** appears to be the *Local Dynamic Map* rationale report, not a congestion study (**verify**). `Bianchi (2000)` is still cited at `:26-27` and `:278-279` although the audit says it was removed. `_CARRIER_SENSE_RANGE_M = 300` hardcoded (`:55`), uncited | Treat as a placeholder. Mark it `EXPERIMENTAL — uncalibrated` in code and docs until replaced by a cited model (for example a Bianchi-type analytical model for broadcast with stated assumptions, or a lookup table from a cited measurement/simulation study) |
| S7 | 🟠 | `persistent_world.py:202-210` | RSU creation: `jid = id(tl) % 10000`. That's the **Python object id**, so every traffic light becomes its own "intersection" (no grouping), and only the first 10 lights in arbitrary order get RSUs. Each SPAT therefore has exactly one signal group | Group by `tl.get_group_traffic_lights()` (or junction id via `tl.get_affected_lane_waypoints()[0].get_junction()`), deterministic ordering (sort by junction id), configurable count (all by default). **Verify** the API in 0.9.16 |
| S8 | 🟠 | `v2v/messages.py:129-132`, `:404-406`, everywhere | BSM `latitude/longitude/elevation` hold **CARLA x/y/z in metres**. Every consumer must "know". JSON outputs relabel them as `x/y` in some places and not others | Rename fields to `x_m, y_m, z_m` in the internal model. If geodetic output is wanted, convert with `world.get_map().transform_to_geolocation()` into separate fields |
| S9 | 🟠 | `v2v/messages.py:345-346` | "Longitudinal acceleration" = Δ(speed magnitude)/Δt over 0.5 s. It isn't longitudinal (magnitude includes lateral and vertical components) and is heavily low-passed | Use `actor_snapshot.get_acceleration()` projected onto the forward vector (`transform.get_forward_vector()`). Same for lateral (right vector). Note the audit P8 (lateral from steer angle) at `:362-372` |
| S10 | 🟡 | `v2v/messages.py:320-321` | `create_bsm_from_carla` first calls `vehicle.get_transform()` (RPC) and `time.time()` (**wall clock**) as defaults, then overrides from the snapshot. If the actor is missing from the snapshot, the BSM silently gets a **wall-clock timestamp** mixed with sim-time BSMs | Missing from snapshot → skip the station this frame and count it in stats. Never mix clocks |
| S11 | 🟡 | `v2v/messages.py:351-356` | `get_control()` and `get_physics_control()` per vehicle per update. `get_physics_control()` is a blocking RPC, and `max_steer_angle` never changes | Cache physics constants at registration |
| S12 | 🟡 | `network_enhanced.py:148,179,345-351` | Wall clock used for `should_update` fallback and `measured_update_hz` (audit P10, still open) | Sim time only. Report wall-clock real-time factor separately |
| S13 | 🟡 | Ranges | Five different "range" defaults: `V2VConfig.max_range = 50` (`config.py:70`), CLI `--v2x-range 75`, `ConnectRequest.v2x_range = 75`, `V2XNetworkEnhanced(max_range=150)`, `V2V_RANGE_MEDIUM = 150`, plus RSU 300 m and VRU 100 m. The "max_range" is an **application-layer cutoff applied after the radio model**, so with S1 the radio never matters inside it | One config value with explicit semantics: `awareness_range_m` (app filter, can be `null` = radio decides). Document the split |
| S14 | 🟡 | `v2v/messages.py:244-297` | TTC uses centre-to-centre distance (ignores vehicle extents, overestimates TTC). Velocity direction is taken from `heading` (yaw) instead of the actual velocity vector (wrong when reversing or sliding). Thresholds 10/5/2 s uncited | Use velocity vectors from the BSM (add `vx, vy`). Optionally bounding-box gap. Cite or label thresholds as project-defined |
| S15 | 🟡 | `v2v/messages.py:427-452` | DENM cause codes need a cross-check against ETSI EN 302 637-3 (audit P7, open) | **verify** |
| S16 | 🟡 | `v2x/rsu.py:9,15,64,70` | "33 dBm per ETSI TR 101 607" (audit P4, open). `dsrc_channel.py:77` "Class A max 23 dBm ERP": IEEE 802.11p/DSRC power classes and ETSI limits need checking (**verify**, the class labels look wrong) | **verify** |
| S17 | 🟡 | `v2x/rsu.py:177-179,211` | SPAT min/max end time = ±1 s around the predicted end (audit P9). Fallback `remaining = 5.0` silently invents timing when the CARLA query fails | On failure emit `likely_time = null`, never a fabricated number |
| S18 | 🟡 | `network_enhanced.py:53-54` | Docstring: "2 Hz per SAE J2735 §7.1 minimum beacon interval". J2735 is a message dictionary. Transmission rates come from SAE J2945/1 (BSM, nominally 10 Hz) or ETSI EN 302 637-2 (CAM, 1–10 Hz, dynamics-triggered) (**verify** both) | Make the rate configurable. Optionally implement ETSI CAM generation rules as a mode |
| S19 | 🟡 | `network_enhanced.py:68`, `server.py:556-576` | `enhanced_messages` (transmission/reception time, link quality, hop count) is **never written**, so the "enhanced metadata" in the API is always null | Replace with real per-link results (§5.3) |
| S20 | ⚪ | `dsrc_channel.py:331-352` | NLOS = any registered vehicle within 2.5 m of the TX→RX segment. Buildings are ignored; docstring oversells it (audit P5, open). With S3, blockers are only client vehicles | Phase 3 option: building occlusion via `world.cast_ray()` or a precomputed 2-D building footprint (**verify** API cost at 2–10 Hz × N²). At minimum fix the docstring |
| S21 | ⚪ | Latency | All messages are delivered in the same tick, with zero latency. Not wrong, but it must be **documented as a modelling assumption** | — |

### 3.3 LiDAR

| ID | Sev | Where | Defect | Evidence / fix direction |
|---|---|---|---|---|
| L1 | 🔴 | `collector.py:156-173` | Hand-rolled sensor→world rotation matrix. Its third row is `[-sin p, cos p sin r, cos p cos r]`. CARLA's own `Transform::GetMatrix()` (LibCarla `geom/Transform.h`) uses `[sin p, -cos p sin r, cos p cos r]` with matching sign flips in column 3 (UE4 left-handed convention). For pitch = roll = 0 they agree. **On slopes, or for a pitched or rolled vehicle, points are transformed wrongly** | **Verify:** unit test comparing against `carla.Transform.transform(carla.Location)` for (yaw, pitch, roll) = (30, 10, 5). Fix: `M = np.array(transform.get_matrix())` |
| L2 | 🟠 | `collector.py:294-300`, `binary_protocol.py` | Header timestamp = `time.time()` (wall clock). No `frame`, no sim time. Points are world-axis-aligned but translated to the sensor origin, and the header carries yaw/pitch/roll. Neither the frame of reference nor that fact is documented in the protocol | Protocol v3 (§5.4) |
| L3 | 🟠 | `collector.py:91-130` | Sensor callbacks arrive on CARLA's thread whenever they like. The streamer reads "latest" data, so a frame can mix with V2X state from another frame | CARLA sync-mode pattern: sensor `listen` → `queue.Queue`. After `world.tick()` returns frame F, `get(timeout)` until `measurement.frame == F`. Then the data is frame-exact |
| L4 | 🟠 | `persistent_world.py:684-691`, `client_routes.py:130-137` | LiDAR can only be enabled **at join**. Config is hardcoded (`channels="32"`, `points_per_second="500000"`, `downsample_factor=2`). No disable, no reconfigure. The frontend doesn't render it (§2) | §5.5 runtime LiDAR control |
| L5 | 🟡 | `collector.py:111-112` | `points[::downsample_factor]` decimates **before** storage, so recorded data would be lossy | Never decimate recorded data. Decimate only the network stream, as a per-client setting |
| L6 | 🟡 | `collector.py:215,225`, `persistent_world.py:691` | Ego detection is hardcoded `vehicle_id == 0`. Works only because each per-session collector registers its vehicle as 0 | Explicit `ego_actor_id` per sensor |
| L7 | 🟡 | `collector.py:71` vs `config.py:14` | Default `rotation_frequency = 20.0` matches Δt = 0.05 s only by coincidence. If Δt changes, one measurement ≠ one full sweep | Validate: `rotation_frequency == 1/fixed_delta_seconds`, or warn and record it |
| L8 | ⚪ | `binary_protocol.py:367-368` | `object_idx` sent as float32 is exact only below 2²⁴. In a long-running persistent server actor ids keep increasing | Send as `uint32` |
| L9 | ⚪ | `collector.py:320-322` | `time.sleep(0.1)` inside cleanup | Destruction happens on the sim thread after `sensor.stop()`. No sleep needed |

Note for the LiDAR UI: `sensor.lidar.ray_cast_semantic` has **no** noise, dropoff or intensity
attributes. `sensor.lidar.ray_cast` has them (`noise_stddev`, `dropoff_*`, `atmosphere_attenuation_rate`)
but no semantic tags. Offer the sensor type as a choice and show only the attributes valid for
that type (**verify** attribute lists for 0.9.16). CARLA sensor attributes are fixed at spawn,
so "reconfigure" means destroying and respawning at a tick boundary.

### 3.4 API and frontend

| ID | Sev | Where | Defect |
|---|---|---|---|
| F1 | 🟠 | `server.py:72-79` | Persistent mode never wires `_v2v_network`, so `/api/v2v/*`, `/v2v`, `/map`, `/vehicles`, `/view` show nothing (see §2) |
| F2 | 🟠 | `client_join.html:283-286` | The LiDAR stream is received and discarded. No point-cloud view for clients |
| F3 | 🟡 | `server.py:278-361` | `/v2v` serves `dashboard.html` after **runtime string-replacement of JS source**. It breaks silently on any edit |
| F4 | 🟡 | `server.py:241-397` | Seven near-identical "read file, return HTMLResponse" routes. Use `StaticFiles` |
| F5 | 🟡 | `client_routes.py:9` | "No authentication": anyone on the LAN can spawn vehicles. Acceptable for a lab, but rate-limit joins and cap sessions (config) |
| F6 | ⚪ | `viewer.html:233-235` | Three.js r128 from CDN. Pin with SRI hashes or vendor locally (labs often run offline) |
| F7 | ⚪ | All responses | Untyped `dict` responses. No OpenAPI schema for most endpoints |
| F8 | 🟡 | `viewer.html:256-…` | Semantic colour table uses the **pre-0.9.14 tag ids** (`1 = Building`, `2 = Fence`). CARLA 0.9.14+ switched to Cityscapes-aligned ids (`1 = Roads`, `2 = SideWalks`, `3 = Building`, …, up to 28). `binary_protocol.py:367` already assumes "max tag = 28". **Verify** against the 0.9.16 sensor reference and generate the palette from one table shared by server and client (`GET /api/lidar/schema`) |
| F9 | 🟡 | `viewer.html:~600` | A new `THREE.BufferGeometry` is allocated for every frame at 10–20 Hz, which causes GC stalls. Allocate once at max size, then update attributes and `setDrawRange` |
| F10 | 🟡 | Plane B | No way to inject a DENM. DENM/CAM endpoints exist only in the deleted standalone V2V app (`src/v2v/api.py`). If hazard warnings are part of the research, the admin needs an "inject DENM at map position" action (§5.7.6) |

### 3.5 Tests and tooling (baseline at `29bca10`)

- **Without `carla` installed, all 12 test modules fail to import**, because
  `network_enhanced.py:10` does a top-level `import carla` and `src/v2x/__init__.py` pulls it in.
  The repo's own rule ("unit tests should not require CARLA") is violated.
- With a stub `carla` module (MagicMock) and `fastapi pydantic numpy pytest httpx` installed:
  **190 passed, 10 failed**, plus 2 modules need `requests`, and 2 frontend modules need Selenium.
  - `tests/test_v2v_neighbor_discovery.py` (3 tests): inject BSMs and call `_discover_neighbors()`
    without `broadcast_all()`, so the channel inbox is empty and there are 0 neighbours. The tests
    predate the channel model.
  - `tests/v2v/test_v2v_basic.py::test_scalability_100_vehicles_all_neighbors` (56 ≠ 99) and
    `tests/v2v/test_scientific_integrity.py::test_cumulative_avg_neighbors_converges`:
    assume lossless delivery, but the CSMA term drops packets at N = 100.
  - `tests/test_v2v_lidar.py` (5 tests): may be stub artefacts. Re-run with real `carla==0.9.16`
    (Python ≤ 3.12 wheels, **verify**) before judging.
- `test_scientific_fixes.py` asserts nothing (see §2.1).

**Test verdicts under Revision 2:**

| Test module | Verdict | Reason |
|---|---|---|
| `tests/test_complete_system.py`, `tests/test_frontend_integration.py` | DELETE | Drive `/api/simulation/*` (plane A) against a live `localhost:8000` |
| `tests/test_frontend.py`, `tests/test_frontend_visual.py` | DELETE | Selenium tests of the plane-A `viewer.html`. Replaced by Playwright tests in Phase 6 |
| `tests/test_reproducibility.py` | DELETE | Standalone CARLA script with its own scenario, not a pytest suite. Replaced by the Phase 7 reproducibility test |
| `tests/test_lidar_api.py` | DELETE | Tests `LiDARStreamingAPI` (plane A). Keep its `BinaryProtocol` cases → `tests/unit/test_lidar_protocol.py` |
| `tests/v2v/test_api.py` | DELETE | Tests the standalone `V2VAPI` app |
| `tests/v2v/test_denm_cam.py` | SPLIT | Keep `TestDENMDataclass`, `TestDENMNetworkStore`, `TestCreateCamFromBsm` → `tests/unit/v2x/`. Delete `TestCAMEndpoints`, `TestDENMEndpoints` (V2VAPI) |
| `tests/v2v/test_dsrc_channel.py`, `test_scientific_integrity.py`, `test_v2v_basic.py`, `tests/test_v2v_neighbor_discovery.py` | MOVE→`tests/unit/v2x/` | Valuable model tests. Fix the stale ones (above) as part of Phase 0 |
| `tests/test_v2v_lidar.py` | SPLIT | Collector/transform cases → `tests/unit/lidar/`. Drop the `ConnectionManager` cases (plane-A `/ws`) |
| `tests/v2x/test_v2x_messages.py` | MOVE→`tests/unit/v2x/` | |

---

## 4. Non-negotiable invariants for the new code

1. **One thread owns CARLA.** Only the sim thread calls any `carla.*` method that talks to the
   server (tick, spawn, destroy, settings, snapshot, sensors). Everything else sends commands.
2. **Every datum is stamped** with `frame: int` and `sim_time: float` from the `WorldSnapshot`
   it was derived from. Wall-clock time appears only as a separately named field
   (`wall_time_unix`).
3. **One clock for scheduling:** frame counts. No `time.time()` in model code.
4. **Determinism:** with `realtime=false`, same config + same seed + same command script gives
   bit-identical V2X logs. Commands are applied at frame boundaries and logged with their frame.
   Real-time pacing is allowed (`realtime=true`) but logged in the manifest as non-reproducible
   input timing.
5. **No `import carla` outside** `sim/`, `lidar/sensor.py` and `v2x/extract.py`. Models,
   messages, channel, protocol and API schemas are importable and testable without CARLA.
6. **Units in names** where ambiguous (`_m`, `_mps`, `_deg`, `_dps`, `_dbm`, `_db`, `_s`).
   Frame conventions documented once (`docs/science/conventions.md`): CARLA is left-handed,
   x forward, y right, z up, yaw in degrees from +x towards +y.
7. **Ground truth and received view are separate.** A vehicle's V2X view = what its radio
   received. Ground truth is only exposed to admin/observer and the recorder.
8. **Recorded data is never decimated or rounded.** Rounding and decimation are presentation
   concerns in the API/stream layer.

---

## 5. Target architecture

### 5.0 Layout

```
pyproject.toml                 # package `carla_v2x`, extras: [dev], [analysis]
carla_v2x/
  cli.py                       # `carla-v2x serve`, `carla-v2x run <scenario.yaml>`
  config.py                    # Pydantic models + YAML/env loading + cross-field validation
  sim/
    controller.py              # WorldController: state machine, sim thread, tick loop
    commands.py                # Command dataclasses + queue; each returns a concurrent.futures.Future
    actors.py                  # traffic, pedestrians, client vehicles (seeded Generator)
    sessions.py                # SessionManager (lock-protected; no CARLA)
    map_geometry.py            # road polylines + junctions for the map view, cached per map name
  v2x/
    messages/                  # bsm.py cam.py denm.py spat.py map.py vruam.py cpm.py (pure)
    extract.py                 # WorldSnapshot → StationState (the ONLY carla import in v2x)
    stations.py                # OBU / RSU / VRU station definitions (tx power, antenna height)
    channel/
      base.py                  # ChannelModel protocol → LinkResult
      dsrc_logdistance.py      # current model, refactored, every term explicit
    network.py                 # V2XNetwork (single class; V2V is a link type), produces V2XFrame
    threat.py                  # TTC / threat level
    stats.py                   # aggregations (PRR vs distance bins, margins…) shared by UI and offline analysis
  lidar/
    sensor.py                  # LidarManager: sensor lifecycle state machine, viewer refcount, budget, frame sync
    stream.py                  # per-sensor stream profiles, encode-once fan-out, display subsampling
    config.py                  # LidarConfig + per-sensor-type attribute validation
    protocol.py                # binary protocol v3
    transforms.py              # thin wrappers over carla.Transform.get_matrix / get_inverse_matrix
  recording/
    manifest.py                # run manifest (config, seeds, git SHA + dirty flag, CARLA map, versions)
    recorder.py                # per-frame writers (JSONL first; Parquet optional extra)
    events.py                  # admin/client command log with frame
  api/
    app.py                     # create_app(controller) — app factory, lifespan, NO module globals
    deps.py                    # auth, controller dependency
    schemas.py                 # all Pydantic request/response models
    routes/{admin,sessions,world,map,v2x,lidar,runs,config}.py
    ws.py                      # topic hub (§5.7.3): sim thread → loop.call_soon_threadsafe → latest-wins per (client, topic)
    roles.py                   # admin / vehicle / observer permissions per endpoint and per WS topic
  testing/
    fake.py                    # FakeController: deterministic synthetic world + synthetic LiDAR (`carla-v2x serve --fake`)
  web/                         # layout in §5.7.8
tests/
  unit/        # no carla, no network
  contract/    # API tests against a FakeController
  golden/      # pinned channel outputs for refactor-equivalence
  integration/ # @pytest.mark.carla — needs a live CARLA 0.9.16
docs/
  science/{model.md, conventions.md, CHANGELOG.md, limitations.md}
  api.md  running.md
AGENTS.md  CLAUDE.md
```

### 5.1 Tick loop (sim thread)

```
while state == RUNNING:
    t0 = monotonic()
    drain command queue → execute each (do_tick=False) → resolve futures; log (frame+1, command)
    frame = world.tick()                         # returns frame id
    snap  = world.get_snapshot();  assert snap.frame == frame
    lidar.collect(frame)                         # queue.get(timeout) per active sensor, match frame
    if frame % v2x_every_n == 0:
        v2x_frame = network.update(snap)         # immutable V2XFrame
    publish(FrameBundle(frame, sim_time, telemetry, v2x_frame?, lidar_frames))  # atomic ref swap
    recorder.write(bundle)                       # own writer thread via bounded queue; never blocks tick
    if realtime: sleep(max(0, dt - (monotonic()-t0)))
```

- `publish` swaps one reference. Readers (HTTP/WS) only read the last published immutable bundle,
  so they need no locks.
- Expose and record `real_time_factor` and `tick_duration_ms` (p50/p95) per run.
- If the recorder queue is full, **stop the run with an error**. Silently dropping research data
  is not allowed. Make that policy explicit in config.

### 5.2 Configuration

One Pydantic model tree loaded from `config/default.yaml`, overridden by run YAML, overridden by
env/CLI. Cross-field validators (fail fast):

- `fixed_delta_seconds <= max_substep_delta_time * max_substeps` (CARLA substepping rule)
- `1 / (v2x.update_rate_hz * fixed_delta_seconds)` is an integer → `v2x_every_n`
- `tm_port != server.port`
- LiDAR: `rotation_frequency == 1 / fixed_delta_seconds` → else warning recorded in manifest
- `use_hybrid_physics` must stay `False` (known zero-velocity issue, `docs/agents/gotchas.md`)

The resolved config is written verbatim into the run manifest. `GET /api/config` returns it.

### 5.3 V2X data contract

```python
@dataclass(frozen=True)
class LinkResult:            # one per (tx, rx, message) attempt
    frame: int; sim_time: float
    tx_id: int; rx_id: int; msg_type: str      # "BSM" | "SPAT" | "MAP" | "VRUAM" | "DENM" | "CPM"
    distance_m: float; los: bool
    path_loss_db: float; shadowing_db: float; rx_power_dbm: float; rx_margin_db: float
    p_channel: float; p_mac: float; p_total: float
    delivered: bool
    drop_cause: str | None   # "channel" | "mac" | "out_of_awareness_range" | None
    link_type: str           # "V2V" | "V2I" | "I2V" | "V2P" | "P2V"  (V2V lives on only as this label)

@dataclass(frozen=True)
class V2XFrame:
    frame: int; sim_time: float
    stations: Mapping[int, StationState]        # ground truth
    sent: Mapping[int, tuple[Message, ...]]     # by tx station
    inbox: Mapping[int, Mapping[int, Message]]  # rx → tx → message (what rx actually received)
    links: tuple[LinkResult, ...]
    threats: Mapping[int, tuple[Threat, ...]]   # computed from the rx's *inbox*, not ground truth
    stats: ChannelStats
```

All V2X message types go through the same `ChannelModel.evaluate(tx, rx, msg, ctx) -> LinkResult`.
Station TX power and antenna height come from the station type (OBU/RSU/VRU device).

### 5.4 LiDAR binary protocol v3

Little-endian, versioned, documented in `lidar/protocol.py` **and** mirrored in
`web/static/js/lidar-decoder.js`, with a cross-language test (Python encodes, Node decodes a fixture).

```
Header:
  4s  magic 'LIDR' | H version=3 | H flags (bit0 zlib, bit1 world-frame, bit2 sensor-frame)
  Q   frame        | d sim_time  | d wall_time_unix
  I   num_points   | I ego_actor_id | I sensor_id | H point_stride_bytes | H field_mask
Sensor pose (world): 3d location (m), 3d rotation (deg: pitch, yaw, roll — CARLA order)
Points: packed per field_mask (x,y,z f32; tag u8/u32; object_idx u32; intensity f32; cos_inc f32)
```

Decide one stream frame (recommended: **sensor frame**, raw as CARLA delivers it, with the pose in
the header). The browser applies the pose. That is exact, cheaper on the server, and keeps
recorded data raw.

### 5.5 LiDAR on request (server side)

A LiDAR sensor is a **costly, explicitly requested resource**. *Sensors* and *viewers* are
separate things: a sensor is a CARLA actor attached to a vehicle, and a viewer is a WebSocket
subscribed to that sensor's stream. Many viewers can share one sensor.

**Lifecycle (one state machine per sensor, owned by the sim thread; every transition is an event
stamped with `frame`):**

```
requested ──(next tick boundary)──► spawning ──► warming_up ──(first measurement for frame F)──► active
active ──PUT config──► reconfiguring (destroy + respawn at a tick boundary) ──► warming_up ──► active
active ──DELETE / owner session ended / auto-release──► stopping ──► destroyed
any ──(spawn failure, actor gone, budget revoked)──► error {reason}
```

- **Auto-release:** a sensor with `viewers == 0` for `keep_alive_s` (config, default 10 s,
  measured in wall time because it's a UI resource policy, not model data) is destroyed, unless
  `record_raw = true` or `pinned = true` (admin only). The destroy frame is logged.
- **Ownership:** a vehicle session may create sensors only on its own actor. The admin can create
  them on any vehicle. Observers can't create sensors, only view active ones (🟨 D9). When a
  session ends, its sensors are destroyed.
- **Budget:** `lidar.max_sensors` and `lidar.max_total_points_per_second` (config). A request
  over budget returns `409 {"error": "lidar_budget_exceeded", "requested_pps": …, "available_pps": …}`.
  The UI shows the budget meter from `GET /api/lidar/sensors`.
- **Points per frame** shown to the user = `points_per_second × fixed_delta_seconds`, computed on
  the server. With `rotation_frequency_hz == 1/Δt` (validated, §5.2) each frame is one full sweep.
- **Recording is raw:** recorded frames are never subsampled (§4.8). Subsampling exists only in
  stream profiles.
- **Stream profiles (display only):** a viewer asks for `{max_points, max_hz, compress}`. The
  server quantises `max_points` to a fixed set (e.g. 10k / 25k / 50k / all) and `max_hz` to
  divisors of the sim rate, so **each frame is encoded once per profile** and the same bytes fan
  out to every viewer on that profile. Subsampling uses a fixed, seeded random permutation per
  sensor, not a stride: CARLA orders points by channel, so a stride makes ring artefacts.
- **Sensor type:** `sensor.lidar.ray_cast_semantic` (tags + `object_idx` = hit actor id, no
  intensity/noise) or `sensor.lidar.ray_cast` (intensity, noise, dropoff, no tags). The schema
  endpoint returns only the attributes valid for each type (**verify** lists against 0.9.16).

```
GET    /api/lidar/schema                  → JSON schema per sensor type, valid ranges, semantic tag table (id → name, colour)
GET    /api/lidar/sensors                 → [{sensor_id, actor_id, owner, state, config, viewers, pinned, record_raw,
                                              points_per_frame, created_frame, last_frame}], budget {used_pps, max_pps}
POST   /api/lidar/sensors                 body {actor_id, config, record_raw?, pinned?}  → 202 {sensor_id, state: "requested"}
PUT    /api/lidar/sensors/{sensor_id}     body LidarConfig → 202 (reconfigure)
DELETE /api/lidar/sensors/{sensor_id}     → 202
POST   /api/sessions/{sid}/lidar          convenience for vehicles: create on own actor
WS     /ws/lidar/{sensor_id}              see §5.7.3 (auth message first, then stream profile; binary v3 frames;
                                          JSON text frames {type: "state", state, frame} on lifecycle changes)
```

`LidarConfig`: `sensor_type`, `channels`, `range_m`, `points_per_second`, `rotation_frequency_hz`,
`upper_fov_deg`, `lower_fov_deg`, `horizontal_fov_deg`, `mount: {x_m, y_m, z_m, pitch_deg, yaw_deg, roll_deg}`,
plus type-specific noise/dropoff fields.

### 5.6 Recording

Run directory `runs/<UTC-timestamp>_<short-sha>_<name>/`:
`manifest.json`, `events.jsonl`, `v2x_links.jsonl` (or `.parquet`), `stations.jsonl`,
`messages.jsonl`, `lidar_meta.jsonl`, optional `lidar/<sensor_id>/<frame>.bin` (protocol v3) and
optional CARLA recorder file (`client.start_recorder`, **verify** it's useful with sync mode
and TM). Endpoints: `POST /api/runs` (start with name + config overrides), `POST /api/runs/current/stop`,
`GET /api/runs`, `GET /api/runs/{id}/manifest`. If 🟨 D6 says to keep the ER model, this run
layout **is** its physical schema. Update the ER doc to match the real fields.

### 5.7 Frontend specification (V2X server only)

Binding spec for Phase 6. The frontend is a **research instrument**, not a dashboard. It must
show exactly what the server computed, labelled with frame and units, and never invent or
smooth data.

#### 5.7.1 Roles, pages, permissions

| Page | Who | Purpose |
|---|---|---|
| `/` | anyone | Server status (CARLA connected, map, run active, frame, sim time, real-time factor) and links to the three role pages |
| `/observer` | anyone (if `server.observer_public`, default true) | **Main V2X research view:** map + station inspector + link log + channel analytics. Can open any *active* LiDAR stream |
| `/vehicle` | anyone who joins (gets a session token) | Own vehicle: "my radio" (received view only), LiDAR on request for the own vehicle, mini-map from the own perspective |
| `/admin` | admin token | Everything on `/observer`, plus world/run/population/V2X config/RSU/DENM/sessions/LiDAR management |

Permission rules (enforced in `api/roles.py`, on REST **and** per WS topic, with contract tests):

- Ground truth (`world.state`, unfiltered `v2x.links`, other stations' inboxes) → observer and
  admin only. A vehicle sees its own ground-truth telemetry and its own inbox, and nothing else
  (§4.7, 🟨 D10).
- Creating/reconfiguring LiDAR → vehicle: own actor. Admin: any actor. Observer: never (🟨 D9).
- Browsers can't set headers on WebSockets, so WS auth is the **first message**
  `{"op": "auth", "admin_token" | "session_token": …}`. Tokens never go in URLs (they leak into logs).
  The session token is a separate secret, not the session id the admin list shows.

#### 5.7.2 Data the frontend needs and where it comes from

| Need | Source | Rate |
|---|---|---|
| Road network for the map | `GET /api/map/geometry` → `{map_name, version, lanes: [[x,y]…], junctions: [{id, bbox}]}` from `world.get_map().get_topology()` + waypoints every 2 m (**verify** cost; cache per map name) | once per map |
| All station states (ground truth) | WS topic `world.state` | ≤ 10 Hz, per client `max_hz` |
| Own telemetry | WS topic `session.self` | ≤ 10 Hz |
| V2X update summary | WS topic `v2x.frame` (per-station sent/received counts, threats count, stats) | per V2X update (2 Hz default) |
| Link results | WS topic `v2x.links` with a **required filter** (station ids, msg types, delivered/dropped). The server caps rows per message (config, e.g. 2000) and reports `truncated: true` | per V2X update |
| One station's received messages (full fields) | WS topic `v2x.inbox` `{station_id}` | per V2X update |
| Channel analytics | WS topic `v2x.stats` → pre-binned aggregates from `v2x/stats.py` | 1 Hz |
| SPAT/MAP, VRUAM, DENM | inside `v2x.inbox` (received view) and `v2x.frame` (ground truth: active DENMs, RSU states) | per V2X update |
| LiDAR sensors list and budget | WS topic `lidar.sensors` (push on change) + `GET /api/lidar/sensors` | on change |
| Point clouds | `WS /ws/lidar/{sensor_id}` binary v3 | ≤ sim rate, per viewer `max_hz` |
| Events (admin actions, config changes, run start/stop, errors) | WS topic `events` | on change |
| Run/recording status | WS topic `run.status` (frames written, writer queue fill, disk usage) | 1 Hz |
| Config + JSON schema (form generation) | `GET /api/config`, `GET /api/config/schema` | on load / on change event |

Why links are filtered: with 100 equipped vehicles at 2 Hz there are 9,900 BSM links per update,
before V2I/V2P. That's far too much to broadcast to every browser. Complete link data belongs to
the recording (§5.6). Live views show a filtered slice and the server-side aggregates.

#### 5.7.3 Transport

- **One multiplexed JSON WebSocket `/ws`** per browser tab for all topics, plus **one binary
  WebSocket per LiDAR stream** (`/ws/lidar/{sensor_id}`). Big binary frames must not delay JSON
  updates (head-of-line blocking).
- Client → server: `{"op": "auth", …}`, `{"op": "subscribe", "topic": "v2x.links", "max_hz": 2, "filter": {…}}`,
  `{"op": "unsubscribe", "topic": …}`, `{"op": "ping"}`.
- Server → client envelope: `{"topic", "frame", "sim_time", "seq", "data"}`, or
  `{"topic": "error", "code", "message"}` for permission or validation errors.
- Server side: **latest-wins per (client, topic)**. A slow browser skips updates and never
  slows the sim thread (Phase 5 acceptance). `seq` gaps let the client display "skipped N updates".
- Client side (`ws.js`): reconnect with exponential backoff + jitter, then re-auth and
  re-subscribe automatically. Show the connection state in the UI permanently.

#### 5.7.4 Map view (`map-view.js`) — observer, admin, vehicle (perspective mode only)

- **Projection:** `screen_x = (x − cx)·s + W/2`, `screen_y = (y − cy)·s + H/2`, heading drawn
  with `ctx.rotate(yaw_deg·π/180)`. CARLA's top-down view (z up, left-handed) and the canvas
  (y down) have the same handedness, so this is **not mirrored**. This is what `v2v_map.html:372-391,480`
  already does. Put it in `coords.js` with tests: a station moving +y moves *down* the screen,
  and yaw 90° points *down*.
- **Layers (each toggleable):** roads/junctions; stations by kind (V2X-equipped vehicle,
  non-equipped vehicle, RSU, VRU), each with its own glyph; labels (id, session name); trails
  (last N s of sim time); awareness-range ring and RSU coverage; traffic-light state per junction;
  **links** of the selected station (incoming and/or outgoing), coloured by outcome (delivered /
  `channel` / `mac` / `out_of_awareness_range`, distinct colour *and* dash pattern so it isn't
  colour-only), line opacity ∝ `p_total`; DENM event markers with relevance radius; LiDAR-active
  marker with range ring and horizontal FoV wedge.
- **Two modes:** *Ground truth* (observer/admin) and *Perspective of station X*. The second shows
  only what X received, drawn at the positions **from the received messages**, with message age.
  A station X hasn't heard from in this update is drawn as a ghost at its last received position
  with its age.
- **Interaction:** click a station → inspector (§5.7.6). Click a link → every `LinkResult` field.
  Pan/zoom, follow the selected station, north-up. **"Show LiDAR"** on a selected vehicle: opens
  the viewer if a sensor exists, creates one if the role allows, otherwise is disabled with the reason.
- **No interpolation or extrapolation** of positions between updates: it would display states
  that never existed. Draw the latest state and show its frame.

#### 5.7.5 LiDAR view (`lidar-view.js` + `lidar-worker.js`)

- **Decoding in a Web Worker** (`lidar-decoder.js`, protocol v3), with the `ArrayBuffer`
  transferred, not copied. The worker returns positions in the Three.js frame plus a colour array.
  Latest-wins: if the worker is busy, keep only the newest frame and count drops.
- **Renderer:** one preallocated `BufferGeometry` sized from the schema's max points. Update
  attributes in place and use `setDrawRange` (fixes F9). Three.js is vendored at a pinned version
  under `static/vendor/three/` (module build + matching `OrbitControls`). Record the version in
  `static/vendor/VERSIONS.md`.
- **Coordinates:** points arrive in the sensor frame (§5.4). Apply the header pose with the same
  matrix CARLA uses (`Transform.get_matrix()` semantics, ported to `coords.js`), then convert
  CARLA → Three.js by mirroring y (`(x, y, z) → (x, −y, z)`, `camera.up = (0, 0, 1)`). A Node
  test compares `coords.js` against a fixture produced in Python by the verified transform (L1),
  for a pitched + rolled pose.
- **Colour modes:** semantic (palette from `/api/lidar/schema`, fixes F8), range from sensor,
  height relative to sensor, intensity (ray_cast only), **highlight actor**: points whose
  `object_idx` equals the selected station's actor id (semantic only).
- **Ego self-hits:** filter `object_idx == ego_actor_id` (toggle, default on). Show how many
  points were filtered.
- **Overlays (each toggleable):** own bounding box; **V2X-received stations** as wireframe boxes
  built from the latest *received* message (position, heading, length/width), coloured by message
  age; ground-truth boxes (observer/admin only); RSUs, VRUs, DENM markers; LiDAR range ring and
  awareness ring. This shows "what the LiDAR sees vs what V2X reports", the core research
  picture (🟨 D13 for turning it into a logged metric).
- **Synchronisation display:** the HUD shows LiDAR frame `F_l`, V2X frame `F_v` and their gap in
  seconds, `(F_l − F_v)·Δt`. Overlays use the newest V2X frame with `F_v ≤ F_l` from a client-side
  ring buffer (≈ 2 s). They never use a V2X frame newer than the point cloud.
- **HUD:** sensor state, points received/displayed/filtered, stream profile, received fps vs
  requested, dropped frames, kB/s.
- **Camera:** orbit, follow (heading-locked or north-up), top-down, free-fly. Keyboard shortcuts.
- **Freeze & export:** pause rendering while the stream keeps running. "Export frame"
  downloads the raw v3 bytes as received plus `frame_<F>_v2x.json` (the V2X frame used for the
  overlays). The JSON states whether the cloud was subsampled (`stream_profile`). Full-resolution
  data comes only from recordings.
- An observer may open at most `ui.max_lidar_views` (default 2) streams at once.

#### 5.7.6 Panels

- **Station inspector:** identity (id, kind, blueprint, equipped, owner session, LiDAR status).
  Ground-truth kinematics next to the **last transmitted message** fields, full precision, with
  units and age. Sent counts and rate per message type. **Inbox table:** `tx_id, link_type,
  msg_type, age_s, distance_m, los, rx_power_dbm, rx_margin_db, p_total`, delivery ratio over
  the last N updates. Threats computed from the inbox (`other_id, ttc_s, level, distance_m`).
  SPAT per intersection in coverage with countdown `likely_time − sim_time`, or "unknown" when
  null (never a fallback number, S17). VRUAM and DENM lists. A raw-JSON toggle for every message.
- **Link log:** live table from `v2x.links` with the current filter. Pause; client ring buffer
  (e.g. 5000 rows); "download CSV of this view" (labelled *live view, filtered*). If a run is
  recording, a link to the run id.
- **Channel analytics** (from `v2x.stats`, computed by `v2x/stats.py`, the **same code** offline
  analysis uses): PRR vs distance in 10 m bins, LOS and NLOS as separate series, with *n* per bin
  and a Wilson 95 % interval. Rx-margin histogram. Neighbours per station over time. Achieved V2X
  update rate vs target. Real-time factor and tick p95. Window switch: "this run" or "last 60 s of
  sim time". Charts: vendored uPlot (**verify** licence) or hand-rolled canvas, no build step.
- **Admin-only cards:**
  - *Connection:* state machine display, CARLA host/port/TM port, connect/disconnect.
  - *Run:* name, seed, duration in frames or open-ended, `realtime` toggle, what to record
    (links, stations, messages, raw LiDAR); start/stop; writer status.
  - *Population:* background vehicles, pedestrians, `penetration_rate` (D2), clear all.
  - *V2X config:* generated from the JSON schema. Each field is marked *hot* (applied at the next
    V2X update, confirmed with the applying frame) or *restart* (needs a new run). Every change
    becomes an event and goes into the manifest's `config_changes`.
  - *RSUs:* list (auto-created per junction, S7), enable/disable, TX power, coverage.
  - *DENM injection* (🟨 D11): click a map position, then pick cause code (enum), relevance
    radius, validity and originating station (RSU or vehicle). The DENM goes through the channel
    like any other message (fixes F10).
  - *Sessions:* table, kick, their LiDAR sensors.
  - *LiDAR:* all sensors, budget meter, attach to any vehicle, pin, detach.
  - *Health:* tick p50/p95, real-time factor, WS clients, outbound kB/s.
- **Vehicle page:** join (name, blueprint from the server list, duration, "request LiDAR on
  join"). Layout: LiDAR view (with a **Request LiDAR** button and config form when off; state
  badge `requested / spawning / warming_up / active / …` when on), "my radio" (inbox summary,
  threats, SPAT, VRUs, DENMs), mini-map in *own perspective* mode. Reconnect behaviour per 🟨 D4,
  with the grace-period countdown visible.

#### 5.7.7 Display rules

- Every live panel shows the **frame and sim time** of its data. Data older than 2 V2X update
  periods is visibly marked stale.
- Rounding happens only in display (`units.js`). Hover or raw view shows the full value. Units
  are always shown. Distances in m, speeds in m/s with km/h as secondary, power in dBm, angles in
  degrees.
- Colour is never the only carrier of meaning (dash patterns and glyphs too). Both light and dark
  themes are readable.
- No silent fallbacks: if a value is missing, show "—" with a tooltip giving the reason.

#### 5.7.8 Web layout (no build step, plain ES modules)

```
carla_v2x/web/
  index.html  observer.html  vehicle.html  admin.html
  static/
    vendor/three/…  vendor/uplot/…  vendor/VERSIONS.md
    css/app.css
    img/logo.png                    # the one kept logo
    js/
      api.js          fetch wrapper (auth header, typed errors → toasts)
      ws.js           multiplexed topic socket: auth, subscribe, backoff + jitter reconnect, resubscribe
      store.js        latest value per topic + V2X frame ring buffer
      coords.js       CARLA ↔ canvas, CARLA ↔ Three.js, CARLA Transform matrix (tested)
      units.js        display formatting only
      schema-form.js  forms from JSON schema (V2X config, LiDAR config)
      map-view.js  lidar-view.js  lidar-worker.js  lidar-decoder.js
      inspector.js  link-log.js  charts.js  toasts.js
      tests/          node --test: decoder vs Python-encoded fixture, coords vs Python fixture
```

#### 5.7.9 Developing without CARLA

`carla-v2x serve --fake` runs `testing/fake.py`: a deterministic synthetic world (stations on
fixed loops, one RSU with a cycling signal, VRUs) through the **real** V2X network and channel
code, plus a synthetic LiDAR (ground plane + boxes for stations with correct `object_idx`). All
frontend work and all Playwright tests run against it. Node 22 and Chromium are available in the
dev container.

---

## 6. Phased plan with acceptance criteria

Each phase is a branch off `developer` (`refactor/pN-<name>`) merged after review.
Run `pytest -m "not carla"` green at the end of **every** phase.

### Phase 0 — Safety net (no behaviour change)
- Push the archive tag (§0.7).
- Make the package importable without CARLA: lazy `import carla` in `network_enhanced.py` (inside
  functions, or `TYPE_CHECKING`), and drop eager imports from `__init__.py` files.
- Add `tests/conftest.py` with a minimal `carla` stub **only** for tests that need types. Register
  `@pytest.mark.carla`.
- **Golden tests:** for `DSRCChannel` with seed 42, fixed position sets (2, 10, 50, 100 vehicles;
  LOS/NLOS geometries), pin the full `_received` matrix and stats. For `create_bsm_from_carla`
  with a fixed mock snapshot, pin every field. These golden files gate every later "pure refactor".
- Convert `test_scientific_fixes.py` into real pytest asserts under `tests/unit/`.
- Record the baseline (§3.5) in `docs/science/CHANGELOG.md`.
- **Accept:** `pytest -m "not carla"` runs **without** `carla` installed. The 10 known failures are
  either fixed-as-stale (with a note) or `xfail(strict=True, reason=…)` with an issue reference.

### Phase 1 — Repo cleanup
- **Remove the monolithic/V2V plane completely** (Revision 2): every DELETE row in §2.1 and every
  DELETE row in the §3.5 test table, after the archive tag. Keep `admin_panel.html` and
  `client_join.html` (and the plane-B routes they call) as the only UI until Phase 6.
  🟨 D1/D6 per human answers.
- Strip from `src/visualization/lidar/server.py` everything listed in its §2.1 row. What remains
  is just the app wiring for plane B. `/` currently serves the deleted `unified_viewer.html`. Until
  Phase 6, make it a tiny page linking `/admin` and `/client`. The Docker healthcheck
  (`Dockerfile`, `docker-compose.yml`) requests `/`, so it must keep returning 200.
- `pyproject.toml`, `ruff` (lint + format), `pyright` basic on the new package, `pre-commit`.
- `.env.example`. Remove the hardcoded admin token from compose.
- **Accept:** `ruff check`, `pyright`, `pytest -m "not carla"` clean. These greps are empty:
  `git grep -nE "api/simulation|api/v2v|v2v_complete_demo|V2VAPI|LiDARStreamingAPI|set_v2v_network|set_collector|8001/"`.
  `src/scenarios/`, `src/utils/` (except the two kept files), `src/agents/`, `src/lidar/` are gone.
  No remaining file is unused (check with `vulture` or an import graph). A live smoke test still
  passes: admin connect → client join → telemetry.

### Phase 2 — Core skeleton (behaviour-preserving)
- `config.py` (§5.2), `sim/controller.py` + `commands.py` (§5.1), `sim/sessions.py`, `api/app.py`
  factory with lifespan, routers ported from `admin_routes.py`/`client_routes.py` (same URLs for now).
- Fix C1–C6, C8–C10, C12–C13 as part of this phase (they are architecture, not model changes).
- `testing/fake.py` `FakeController` for contract tests (extended with synthetic LiDAR in Phase 6).
  Every endpoint has a request/response schema.
- **Accept:** contract tests cover every endpoint. A live smoke test (`@carla`) joins 3 clients,
  spawns 50 vehicles and 20 pedestrians, runs 2 min: `frames_ticked == world frame delta`
  (no foreign ticks), WS telemetry measured at the configured rate ±5%, no exceptions in the log.

### Phase 3 — V2X: unify, make honest, make observable
1. *Pure refactor commits:* move `src/v2v/*` into `v2x/` (§2.1) and merge into one `V2XNetwork`.
   Delete the `v2v` package. Split messages, extractor and channel. Rename BSM position fields (S8).
   Golden tests must stay identical. After this step the §2 naming rule holds (`git grep -in v2v`
   finds only the allowed matches).
2. *Model-change commits, one per item, each with a CHANGELOG entry and before/after numbers:*
   S2 (yaw rate), S7 (RSU grouping), S9/S10/S11 (kinematics from snapshot), S4 (all messages via
   channel), S3 (penetration rate, per D2), S13 (one range semantics), S14 (TTC from velocity
   vectors), S17 (no fabricated SPAT timing), S18 (configurable rate).
3. *Channel honesty (S1, S5, S6):* add `docs/science/model.md` with every equation, parameter,
   unit and source. For each source, mark *verified (page/table)* or *unverified*. Recalibrate only
   if the human supplies or approves a reference (🟨 D5). Otherwise keep the parameters, **fix the
   docs and audit to match the code**, and label CSMA as `EXPERIMENTAL`.
4. Per-link `LinkResult` (§5.3) and `V2XFrame` publishing.
- **Accept:** a unit test reproduces the `model.md` P_rx/PRR table within 0.01 dB / 0.001.
  Changing any channel parameter in config changes the logged `LinkResult`s accordingly
  (property test). `docs/agents/audit.md` is superseded by `docs/science/model.md` +
  `docs/science/limitations.md`.

### Phase 4 — LiDAR
- `LidarManager` with frame-synced collection (L3), `get_matrix()` transforms (L1) with the
  pitched/rolled test, protocol v3 (L2/L8) plus the JS decoder and cross-language fixture test,
  sensor lifecycle state machine, auto-release, ownership and budget (§5.5), stream profiles with
  encode-once fan-out, stream-only subsampling (L5), validation (L7).
- **Accept:** unit tests (FakeController) for every lifecycle transition, auto-release after
  `keep_alive_s`, the over-budget 409, and the ownership rules. `@carla` test: request → active →
  reconfigure (32→64 channels) → viewer disconnect → auto-release, on a live session, with no
  leaked sensor actors (`world.get_actors().filter('sensor.*')` count back to baseline).
  Two viewers on the same profile get byte-identical frames (encoded once).
  Every streamed frame's `frame` equals a ticked frame. At 20 FPS with 3 LiDAR sessions at default
  config, tick p95 ≤ 50 ms on the lab machine. Record the measured number in the CHANGELOG
  either way.

### Phase 5 — API and WebSocket hub
- Topic hub (§5.7.3): multiplexed `/ws` with auth-first-message, subscribe/unsubscribe, per-topic
  `max_hz` and filters, latest-wins per (client, topic), `seq` numbering. Separate binary
  `/ws/lidar/{sensor_id}`. Decide C11 per D4.
- `api/roles.py`: admin / vehicle / observer permissions on every endpoint and topic (§5.7.1).
- REST from §5.5 and §5.7.2 (`/api/map/geometry`, `/api/config/schema`, `/api/lidar/*`, …).
  OpenAPI export committed to `docs/api.md`.
- **Accept:** a slow WS client (artificially throttled) never slows `tick_duration_ms`. Contract
  tests prove every ground-truth topic is refused for a vehicle token, and that `v2x.links` without
  a filter is refused. OpenAPI schema diff is reviewed.

### Phase 6 — Frontend (§5.7 is the spec)
- **6a Foundations:** `serve --fake` with synthetic LiDAR (§5.7.9). Vendored Three.js/uPlot.
  `api.js`, `ws.js`, `store.js`, `coords.js`, `units.js`, `schema-form.js`, `lidar-decoder.js`
  + Node tests (decoder fixture, coords fixture incl. pitched/rolled pose, map projection
  handedness).
- **6b Observer:** map view (both modes), inspector, link log, channel analytics, LiDAR viewer for
  active sensors.
- **6c Vehicle:** join, "my radio", own-perspective mini-map, LiDAR request/config/view with the
  state badge.
- **6d Admin:** all admin cards (§5.7.6). Then delete `admin_panel.html`, `client_join.html` and
  the remaining `src/visualization/`.
- **Accept:** Playwright against `serve --fake`: every page loads with no console errors;
  vehicle requests LiDAR → badge goes `requested → … → active` → points render (count > 0) →
  reconfigure → still renders → leave → sensor destroyed; an observer clicks a station → inspector
  shows its inbox; a link click shows every `LinkResult` field; the vehicle page has no ground-truth
  topic subscriptions (checked on the server). With 3 LiDAR views at 50k points/frame and 20 Hz,
  the browser sustains ≥ 15 rendered fps on the lab machine (report the measured number). The
  manual script in `docs/running.md` passes against live CARLA.

### Phase 7 — Recording and headless runs
- `recording/` (§5.6), `carla-v2x run scenario.yaml` (seeded, `realtime=false`, scripted commands
  at frames, fixed duration in frames). This is new code on the V2X server. The deleted monolithic
  demo is not resurrected.
- **Accept:** `@carla` reproducibility test: the same scenario twice gives byte-identical
  `v2x_links.jsonl` and `stations.jsonl` (if CARLA/TM physics prove non-deterministic, document the
  exact divergence frame and cause in `limitations.md`, and don't paper over it).

### Phase 8 — Docs
- `AGENTS.md` (replaces `.instructions.md`, `docs/agents/*`, `.github/agents/*`), `CLAUDE.md`
  → "see AGENTS.md", `README.md` (setup, run, architecture diagram), `docs/running.md`,
  `docs/api.md`, `docs/science/*`.
- **Accept:** every command in the docs was executed during this phase. No doc mentions a file
  that doesn't exist.

---

## 7. 🟨 Decisions for the human researcher (ask before acting)

| ID | Question | Default if told to proceed |
|---|---|---|
| D1 | Potholes: the use case was cancelled in `29bca10`. Delete all pothole code, endpoints, map instructions? | Delete (preserved by the archive tag) |
| D2 | Should background traffic be V2X-equipped? If yes, with what **penetration rate**? This dominates channel load and neighbour statistics | Configurable, default `1.0` (all equipped, matches legacy plane A) |
| D3 | ~~Keep BehaviorAgent driving?~~ **Resolved by Revision 2:** the monolithic demo is removed, so TM autopilot only and `src/agents/` is deleted. Still open: is **manual driving** of the own vehicle from the vehicle page in scope? | No manual driving |
| D4 | WebSocket disconnect semantics: end the session immediately (current) or after a grace period? | Grace period 30 s (config), then despawn |
| D5 | Channel model calibration: which reference dataset/paper should P_rx(d), shadowing σ and the MAC model be calibrated against? Keep "DSRC 802.11p" framing or also model C-V2X (PC5)? | Keep current parameters, fix the documentation to match the code, label CSMA experimental. No C-V2X |
| D6 | ER database model: implement it (SQLite/Parquet recording schema) or delete it? | Treat the run directory (§5.6) as the schema and update the ER doc to match |
| D7 | Git remotes: docs say team work lives on `gitlab` and `origin` is a GitHub fork. Which remote/branch receives the refactor? | Feature branches on `origin`, human merges |
| D8 | Default V2X rate: 2 Hz (current) vs 10 Hz (J2945/1-style) vs ETSI CAM triggering? | 2 Hz configurable, the others as options |
| D9 | Who may **create** LiDAR sensors? Can LiDAR go on non-client (background) vehicles? | Vehicle → own vehicle only; admin → any vehicle incl. background; observer → view active streams only |
| D10 | May vehicle clients see ground truth (other stations' true positions), or strictly what their radio received? | Strictly received view (§4.7) |
| D11 | Is admin **DENM injection** (hazard at a map position, through the channel) in scope? | Yes, admin only |
| D12 | Is the observer page public on the LAN, or does it need a token too? | Public (`server.observer_public: true`), read-only |
| D13 | Should "LiDAR sees vs V2X reports" become a **logged metric** (per pair and frame: LiDAR-visible = ≥ k points with `object_idx` = that actor, vs V2X-received), or stay a visual overlay? | Overlay now. Metric as an optional Phase 7 item after the human defines k and the semantics |

---

## 8. How to reproduce the key findings

```bash
# Channel calibration (S1): pure Python, no deps
python3 - <<'EOF'
import math, random
L0 = 20*math.log10(4*math.pi*10/(3e8/5.9e9))                 # 67.86 dB at d0 = 10 m
prx = lambda d, n: 23 + 2*2 - (L0 + 10*n*math.log10(d/10))    # P_tx + 2·G − PL
sig = lambda m: 1/(1+math.exp(-2.0*m))
rng = random.Random(0)
for d, n, s in [(50,2.3,5.5), (100,2.3,5.5), (150,2.3,5.5), (150,3.2,8.0), (500,2.3,5.5)]:
    m = prx(d, n) + 85
    e = sum(sig(m - rng.gauss(0, s)) for _ in range(100000)) / 100000
    print(f"{d} m n={n}: P_rx={prx(d,n):.1f} dBm margin={m:.1f} dB E[PRR_channel]={e:.4f}")
EOF

# Double-gate drop (C7)
python3 -c "
t=0.0; last=None; skip=0
for f in range(1,72001):
    t+=0.05
    if f%10==0:
        if last is not None and t-last<0.5: skip+=1; continue
        last=t
print('skipped', skip, 'of 7200')"

# Test baseline without CARLA (§3.5): expect 12 collection errors on 'import carla'
python -m pytest tests/ -q
```

---

## 9. Report back

At the end of each phase, report:

1. What changed (commits, files deleted, behaviour changes with CHANGELOG links)
2. Measured numbers (tick p50/p95, real-time factor, update Hz, PRR table)
3. Every `TODO(verify)` you added, with what source would settle it
4. Anything in this handoff you found to be **wrong**. Correct this file in the same PR.
