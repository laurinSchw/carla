# Science / model changelog

Every change that can alter results gets an entry with before/after numbers.
Pure refactors (bit-identical outputs) are listed only as such.

## Phase 0 — baseline (2026-10-09, branch `claude/refactor-handoff`, base `29bca10`)

Pure refactor, no behaviour change:
- `carla` is imported lazily (`TYPE_CHECKING` / inside functions) in
  `src/v2v/network_enhanced.py`, `src/visualization/lidar/collector.py`,
  `src/visualization/lidar/api.py`; `src/utils/__init__.py` exports are lazy (PEP 562).
- Added `pytest.ini` with the `carla` marker.

Test baseline without `carla` installed (`pytest tests/v2v tests/v2x tests/test_v2v_neighbor_discovery.py tests/test_v2v_lidar.py tests/test_lidar_api.py -m "not carla"`):
187 passed, 8 skipped, 5 xfailed (strict).

Known-stale tests, now `xfail(strict=True)` (REFACTOR_HANDOFF §3.5):
- `tests/test_v2v_neighbor_discovery.py` (3): inject BSMs without `broadcast_all()`.
- `tests/v2v/test_v2v_basic.py::test_scalability_100_vehicles_all_neighbors`,
  `tests/v2v/test_scientific_integrity.py::test_cumulative_avg_neighbors_converges`:
  assume lossless delivery, but the CSMA term drops packets at N=100.

Golden tests added (`tests/golden/`, regenerate only deliberately with `UPDATE_GOLDEN=1`):
- `DSRCChannel`, seed 42, 5 consecutive ticks: 2-vehicle pairs (30/250/400 m), 10-vehicle line
  (NLOS blockers, with and without the NLOS model), 50- and 100-vehicle grids.
- `create_bsm_from_carla` with a mock snapshot (normal, and braking/reverse ambulance).
- These pin current behaviour including known defects (S2 yaw rate, S8 x/y in lat/lon fields).
- `test_scientific_fixes.py` (root, asserted nothing) replaced by `tests/unit/test_scientific_fixes.py`
  with real asserts; its pothole test is dropped (D1: pothole code is to be deleted).

Not yet run: modules slated for deletion (`test_complete_system`, `test_frontend*`,
`test_reproducibility`), which need a live server, Selenium or `websocket`.

## Phase 1 — repo cleanup (2026-10-10, no behaviour change in the V2X/DSRC models)

- Removed the monolithic/V2V plane: `src/scenarios`, `src/agents`, `src/lidar`, `src/web`,
  `src/visualization/lidar/{server,api}.py`, `src/v2v/{api.py,dashboard.html}`, all of `src/utils`
  except `binary_protocol.py`, six plane-A HTML pages, spare logo variants, legacy shell scripts,
  `docs/V2V_API_REFERENCE.md`, and the plane-A / Selenium / live-server tests.
- D1 (default): pothole code, README section and `scripts/lidar_pothole_detection_check.py` removed.
- New `src/server/app.py` holds the app wiring (`/`, `/admin`, `/client`, routers, `/assets`);
  `start_server.py` imports from it.
- Removed unused requirements (selenium, open3d, opencv-python, matplotlib, pandas).
- `docker-compose.yml` reads `ADMIN_TOKEN` from `.env` (`.env.example` added).
- Tests split per handoff §3.5: protocol/collector-schema cases moved to
  `tests/unit/test_lidar_protocol.py`; DENM/CAM endpoint tests, `ConnectionManager` tests and
  `LiDARStreamingAPI` tests dropped with their code.
- Test result without `carla`: 161 passed, 8 skipped, 5 xfailed.
- Not done yet from the Phase 1 list: `pyproject.toml`, ruff, pyright, pre-commit.

## Server tests (2026-10-10, no model change)

- `tests/contract/` (53 tests, no CARLA): the real FastAPI app with a `FakeWorld` that uses the
  real `SessionManager`. Pages, admin auth (missing/wrong/unconfigured token), join -> telemetry
  -> leave, kick, WebSocket telemetry/ping/expiry/disconnect cleanup, traffic/pedestrian
  endpoints, request validation. Mutation-checked: breaking auth or `leave` makes them fail.
- `tests/server/` (marker `server`, skipped without `SERVER_URL`): the same life cycle against a
  RUNNING server over real HTTP/WebSocket. Needs `SERVER_URL`, `ADMIN_TOKEN`; simulation tests
  (marker `carla`) also use `CARLA_HOST`/`CARLA_PORT` and connect the server if it is idle.
- Measured against the real app code path with a fake world: telemetry WebSocket rate 2.48 Hz
  (documented 5 Hz) - confirms handoff defect C6. Not yet verified against live CARLA.

## Phase 1 tooling (2026-10-10, no model change)

- `pyproject.toml` (deps, extras `dev`/`analysis`, ruff config), `.pre-commit-config.yaml`
  (ruff on commit, offline pytest on push), `pyrightconfig.json` (basic mode on `src/` and
  `start_server.py`).
- Ruff (`E9`,`F`) clean: removed 36 unused imports and unused variables. Rule set is
  deliberately conservative; widen during the restructuring.
- Pyright basic: 0 errors. `reportOptionalMemberAccess` is switched off in the three server files
  (42 findings from module-global/None-initialised state) until Phase 2 replaces that design.
- Test fix: `test_different_seeds_may_differ` computed `all_same` but asserted nothing, so it could
  never fail; it now asserts (seeds 1 and 999 give different delivery sets, deterministic).
- `vulture` (confidence 80): one hit, the unused `sensor_data` parameter of `V2VNetworkEnhanced.enable_bidirectional_sharing` (nothing calls it; goes away with the Phase 3 network merge). Every remaining source file is imported somewhere.
- Still open from Phase 1: live smoke test by a human (admin connect -> client join -> telemetry; `tests/server` covers it when pointed at a live server); archive tag push (HTTP 403 from this environment).

## Phase 2 - single simulation thread (2026-10-10, behaviour-preserving architecture change)

No V2X/channel model change (golden tests unchanged). Fixes handoff defects:
- C1/C2/C3/C4/C5: only the "carla-tick" thread calls CARLA after `start()`. Handlers use
  `world.submit(fn)` (`SimThreadExecutor`) and await the result off the event loop. All
  `apply_batch_sync` calls use `do_tick=False`; the one explicit tick (pedestrian controllers)
  goes through `_tick()`. Mutating methods raise if called from another thread.
- C1: `_frame` is CARLA's own frame id (previously a counter that drifted with foreign ticks).
- C10: telemetry (per frame) and the per-client V2X payload (per V2X update) are published by the
  sim thread, stamped with the snapshot frame (+ `sim_time`); readers make no RPC.
- C6: separate receive task, no double wait. Measured telemetry WebSocket rate 2.48 Hz -> 4.97 Hz
  (target 5 Hz) with the real app on a recording `carla` stub. Not yet measured on live CARLA.
- C8: spawn points / blueprints use a seeded `random.Random(config.seed)` (was unseeded global).
- C9: `WorldConfig` (pydantic, cross-field validation: substepping rule, tm_port clashes, whole-frame
  V2X rate). Defaults reproduce earlier behaviour, including `global_speed_pct = -20.0` (20 % FASTER
  than the limit) which contradicts `src/config.py` ("30 % slower"); contradiction recorded, not changed.
- C12: `/admin/connect` validates the config (422), rejects a concurrent connect (409), and reports
  `connect_state` / `connect_error` in `/admin/status`.
- C13: timezone-aware datetimes; `SessionManager` has a lock.
- Robustness (new): an exception in the V2X update or session expiry is logged and the simulation
  keeps ticking (previously any non-RuntimeError ended the tick thread and destroyed all actors).
- V2X update gate is now `frame - last >= every_n` (was `frame % 10`), so extra ticks cannot skip it.
  The second, time-based gate inside the network (defect C7) is Phase 3.
- LiDAR collector cleanup now runs on the sim thread (was a helper thread making CARLA calls); its
  internal `time.sleep(0.1)` (L9) briefly stalls the tick loop when a LiDAR session ends (Phase 4).
- Verification: stub-`carla` tests record the calling thread of every CARLA call (before: 33 of 38
  calls off-thread, 4 batches ticked implicitly, frame 6 vs real 11; after: 0, 0, equal). Mutation
  checks confirm the tests fail if `do_tick=True` or the thread guard is restored.
- NOT verified against a real CARLA server: that actors spawned by `apply_batch_sync(..., False)` are
  visible to `world.get_actor` before the next tick, and that destroy batches without a tick behave.
  TODO(verify) - run `tests/server` with CARLA_HOST/CARLA_PORT (traffic/pedestrian/join tests).

## Phase 3 — model fixes (2026-11-03, branch `developer`)

Each item changes results; golden BSM files regenerated deliberately (`UPDATE_GOLDEN=1`).
Regression tests: `tests/v2x/test_model_fixes.py`.

### S2 — BSM yaw rate was inflated ×57.3
`create_bsm_from_carla` applied `math.degrees()` to `get_angular_velocity().z`, which CARLA already
returns in deg/s. Fix: use the value as is.
- Golden `bsm_from_snapshot` / `bsm_braking_reverse` (mock angular velocity 0.5 deg/s):
  `yaw_rate` 28.64788975654116 → 0.5. No other BSM field changed.
- Affects every consumer of `BSMCore.yaw_rate` (threat/telemetry). Not yet verified against live CARLA 0.9.16.

### C7 — 2 Hz V2X update was skipped by a float-accumulation gate
The server gates on frames (every 10th) and the network gated again on sim-time Δ ≥ 0.5 s; with
accumulated `+= 0.05` the Δ is sometimes 0.4999… and the update was dropped.
Fix: 1e-6 s tolerance in `V2XNetwork` gate.
- Reproduction in the new test (7200 frames, dt 0.05, update on every 10th frame, `update_rate_hz=2`):
  618 of 720 updates performed (≈1.72 Hz effective) → 720 of 720 (2.00 Hz).
- Effect: more BSM/channel ticks per run; neighbour statistics and DSRC RNG consumption differ for the
  same seed compared with earlier runs.

### S17 — SPAT no longer invents phase timing
When CARLA's phase timers could not be read, `RoadSideUnit` emitted `remaining = 5.0 s` with a ±1 s
window. Now `min_end_time_s`, `max_end_time_s`, `likely_time_s` are `None` (admin route and
`get_vehicle_v2x_data` already pass `None` through; the admin route's min/max rounding was made None-safe).
- Before: unreadable timer → likely end = now + 5.0 s. After: `null`.

### S8 — BSM position fields renamed `latitude/longitude/elevation` → `x_m/y_m/z_m`
They always held CARLA world x/y/z in metres, never geodetic degrees. Pure rename, bit-identical values
(golden BSM files: only the keys changed). No geodetic output exists; add one separately with
`world.get_map().transform_to_geolocation()` if needed.

### S9 — BSM accelerations come from the snapshot's acceleration vector
Longitudinal acceleration was Δ|v|/Δt over the 0.5 s update (not longitudinal, heavily low-passed);
lateral came from a steering-angle bicycle estimate. Now, when the actor snapshot offers
`get_acceleration()`, both are the world acceleration projected on the heading (forward = (cos yaw, sin yaw),
right = (−sin yaw, cos yaw); right is positive). Without it the old estimates remain as fallback.
- Unit test: heading +90°, accel (1.5, 2.0, 0) → longitudinal 2.0, lateral −1.5 m/s².
- Golden BSMs unchanged (their mocks have no `get_acceleration`). **Not yet verified on live CARLA 0.9.16**
  (check that `ActorSnapshot.get_acceleration()` is what the Python API documents and the sign convention).

### S10 — no mixed-clock or stale BSMs
`create_bsm_from_carla` returns `None` if a snapshot is given but the actor is not in it (it used to fall back
to RPC reads and a wall-clock timestamp). `V2XNetwork.update` skips that station, drops its previous BSM and
counts it in `stats['bsm_skipped_not_in_snapshot']`. Without any snapshot the old wall-clock fallback remains,
documented as inaccurate.

### S11 — physics constants cached
`max_steer_angle` is read once in `V2XNetwork.register` instead of a blocking `get_physics_control()` RPC per
vehicle per update.

### S3 / D2 — background vehicles are V2X stations (penetration rate)
In the server, only client vehicles were V2X stations, so the channel saw N ≈ number of clients: the CSMA term
was ≈ 1 and NLOS blockers/neighbours existed only between clients. Now every spawned background vehicle is
registered with probability `v2x_penetration_rate` (new `WorldConfig` field, admin `/admin/connect`
`v2x_penetration_rate`, CLI `--v2x-penetration`; default **1.0** = all equipped, handoff D2 default). The draw uses a
separate seeded stream (`seed + 1`), so spawn choices stay reproducible. Unregistered on despawn.
- **Behaviour change:** with background traffic, N (CSMA contenders, N² channel pairs, neighbours) is larger than
  before. Cost grows with N²; no live-CARLA timing measured yet — measure before running with > ~100 stations.

### S7 — one RSU per signalised intersection
RSUs used `id(tl) % 10000` (the Python object id), so every traffic light became its own "intersection", and only
10 arbitrary lights got an RSU. Now lights are grouped with CARLA's `get_group_traffic_lights()`, one RSU per
group at the centroid of its heads (`RoadSideUnit.from_traffic_lights`), ids assigned by ascending smallest
actor id (stable between runs), no arbitrary cap. **Not yet verified on live CARLA** (group semantics).

### D6 — ER model
Decision: no database; the run directory (handoff §5.6) is the schema. `docs/ER_DATABASE_MODEL.md` carries a
status note; the diagram is regenerated when Phase 7 lands.

### S4 / S13 / LinkResult — every message type goes through the same channel
Before, only BSMs used `DSRCChannel`. SPAT "reception" was `dist <= 300 m` (RSU) and VRUAM `dist <= 100 m`
(hard-coded in `persistent_world.py`); `RSUConfig.tx_power_dbm` (33 dBm) was never used.
Now `DSRCChannel.evaluate_link()` decides every attempt (BSM, MAP, SPAT, VRUAM) and returns a frozen
`LinkResult` (frame, sim_time, tx/rx id, msg_type, link_type V2V|I2V|P2V, distance, LOS, TX power, path loss,
shadowing, rx power, rx margin, p_channel, p_mac, p_total, delivered, drop_cause). RSUs transmit with their own
`tx_power_dbm`; VRU devices use the channel default (23 dBm, project-defined, unverified).
`V2XNetwork` keeps per-vehicle inboxes (`spat_inbox`, `map_inbox`, `vruam_inbox`) — what a vehicle received, not
ground truth — and `get_link_results()`; admin `GET /admin/v2x/links?msg_type=&rx_id=&delivered=&limit=`.
- **S13:** `max_range` is now explicitly the *awareness range*: one application filter applied after the radio to
  all message types (`None` = radio decides). A packet the radio delivered but from beyond it is logged
  `delivered=False, drop_cause="out_of_awareness_range"`. The RSU 300 m and VRU 100 m cut-offs are gone
  (`RSUConfig.broadcast_range_m` is informational). With the server default 75 m an RSU's SPAT is therefore received
  within 75 m of its position (was 300 m), VRUAMs within 75 m (was 100 m).
- **BSM path is bit-identical** (golden delivery matrices unchanged). Only change: stats key
  `avg_snr_margin_db` → `avg_rx_margin_db` (values identical) — the quantity is `P_rx − sensitivity`, not an SNR.
- **Cost:** with `record_links=True` (default) one record per (tx, rx, message) is kept per update (N·(N−1) BSM records);
  set `DSRCConfig.record_links=False` for very large runs.
- **RNG:** V2I/V2P draws come after the BSM draws of the same update, so runs that have RSUs or VRUs consume the
  seeded stream differently from before; runs without them are unchanged.
- Drop causes: one Bernoulli draw decides `p_channel · p_mac`, so a radio drop is `"channel_or_mac"` (not separable).

### S14 — threat model uses velocity vectors and vehicle extents
`BSMCore` gained `velocity_x`, `velocity_y` (world-frame m/s; golden BSMs: only these two fields added).
`calculate_threat_level` takes the vehicles' velocity vectors (fallback: speed along heading when absent), so a
reversing or sliding vehicle is handled; TTC is the closing speed along the line of sight divided by the *gap*
between the two bounding circles (centre distance − both half-diagonals, floor 0), not the centre distance.
Effect: TTC is shorter than before by (r₁+r₂)/closing speed (≈ 0.5 s at 10 m/s for two cars). The 100 m / 10 s / 5 s /
2 s thresholds stay and are now labelled project-defined (no citation).

### S18 — update rate: honest docs, configurable everywhere
Removed the claim that SAE J2735 sets a 2 Hz beacon interval (J2735 defines contents; BSM rate is nominally 10 Hz in
SAE J2945/1, CAM 1–10 Hz dynamics-triggered in ETSI EN 302 637-2 — both unverified here). Rate is `WorldConfig.v2x_update_rate_hz`
(default 2.0), now also `--v2x-rate` and `POST /admin/connect {"v2x_update_rate_hz": ...}`; it must divide the simulation rate.
ETSI CAM triggering rules are not implemented.

### S12 — reproducible update-rate statistic
`stats['measured_sim_update_hz']` is computed from simulation time (equals the configured rate on a correct gate);
`measured_update_hz` stays wall-clock and depends on the real-time factor.

### S1 / S5 / S6 / S15 / S16 / S20 — channel honesty (documentation and labels; no output change)
`docs/science/model.md` now lists every equation, default and its status, and a table generated from the code,
pinned by `tests/v2x/test_channel_model_table.py`. Removed or corrected: the "P_rx ≈ −71/−77/−87 dBm" calibration claim
(code: −56.9 / −63.9 / −78.5 dBm), the "meaningful 10–150 m PRR variation" claim, the Bianchi / Campolo / "ETSI TR 102 863"
citations for the MAC term, "ETSI EN 302 663 Class A 23 dBm" and "ETSI TR 101 607 33 dBm" as established facts (now *unverified project
defaults*), and the NLOS docstring (vehicles only, no buildings). The MAC term is labelled EXPERIMENTAL / uncalibrated; `channel_busy_ratio` is
documented as unused. DENM cause-code numbers are marked unverified. **Not changed (open):** i.i.d. shadowing and sigmoid-then-Bernoulli (S5),
the MAC placeholder constants (S6), building occlusion (S20), a calibration against a cited dataset (needs your decision D5).

## Phase 4 — LiDAR on request (2026-11-03, branch `developer`)

New package `src/lidar/` (protocol v3, validated config, sensor manager, CARLA backend); the old
`src/visualization/lidar/collector.py`, `src/utils/binary_protocol.py` (v1/v2 format) and their tests are removed.
LiDAR data is sensor-frame and not part of the V2X model; no V2X output changes.

- **L1 (wrong sensor→world matrix) — verified against CARLA's own transform** (`carla.Transform.get_matrix()`, available
  in the venv, no server needed). For a vehicle at pitch 10°, roll 5°, yaw 30° the removed matrix put the point (4, 5, 6)
  at (3.15, 8.39, 8.62); CARLA gives (0.83, 8.26, 9.15): **2.3 m off**. For flat ground (pitch = roll = 0) both agree.
  Fix: the server no longer transforms points; it streams raw sensor-frame points plus the sensor's world pose, and
  `src/lidar/geometry.sensor_to_world` uses `get_matrix()` (pinned for 5 poses against CARLA in `test_lidar_geometry_and_backend.py`).
- **L2/L8 protocol v3:** header carries `frame`, `sim_time`, `wall_time`, `ego_actor_id`, `sensor_id`, frame-of-reference flags and
  the sensor pose; `object_idx` is `uint32` (float32 lost ids > 2²⁴). 21 B/point (semantic) vs 20 B before. Wire layout pinned by test.
- **L3 frame-exact collection:** the CARLA callback only enqueues; each tick takes the measurement of exactly the frame it just ticked
  (older ones discarded, a missing one counted in `missed_frames`). Every streamed frame id is a ticked frame (stub-server test).
- **L4 runtime control:** `PUT/DELETE /client/lidar/{session}` request, reconfigure (destroy + respawn at a tick boundary) and
  release; join with `enable_lidar` still works and now reports why LiDAR was refused (`lidar_error`). Lifecycle
  `requested → spawning → warming_up → active → reconfiguring/stopping → destroyed | error`, every transition logged with its frame.
- **Budget/ownership/auto-release:** `WorldConfig.lidar` (`max_sensors` 4, `max_total_points_per_second` 2 M, `keep_alive_s` 10, `frame_timeout_s` 0.25).
  Over budget → HTTP 409 `{"error":"lidar_budget_exceeded","requested_pps","available_pps",...}`. A vehicle may only attach to its own actor; only an admin may pin.
  A sensor without viewers for `keep_alive_s` is destroyed unless pinned or recording raw. Session end destroys its sensors *before* the vehicle.
- **L5/L6/L7/L9:** streaming subsamples with a fixed seeded permutation per sensor (not a stride) and quantised profiles
  (10k/25k/50k/all points, `max_hz` a divisor of the sim rate, optional zlib), each frame encoded once per profile and shared;
  raw listeners get full-resolution frames. `ego_actor_id` is explicit. `rotation_frequency_hz` is validated to equal 1/Δt (HTTP 422 otherwise).
  No `sleep` in sensor teardown (`stop()` then `destroy()`).
- **Measured cost (host, numpy):** encoding one 25 k-point frame 1.2 ms (5.1 ms with zlib, 148 kB instead of 513 kB), 100 k points 5.2 ms (20.8 ms zlib).
  Encoding runs on the WebSocket reader thread, not on the simulation thread; the tick loop only dequeues and wraps the raw buffer.
  **Not measured:** tick p95 with 3 LiDAR sessions on a live CARLA server (the handoff's Phase 4 acceptance number) — needs a server.
- **Unverified against CARLA 0.9.16:** the ray_cast-only attribute names (`noise_stddev`, `dropoff_*`, `atmosphere_attenuation_rate`) and that
  `data.frame` equals the ticked frame in synchronous mode. `tests/server` (`-m carla`) contains the end-to-end check.

### Phase 4 addendum — LiDAR display (client page)
`/client` now has a LiDAR card: request/reconfigure/stop a sensor (channels, points/s, range), stream settings (points, rate, zlib),
top-down and 3-D (drag) views, colour by semantic class / height / distance / intensity, hide-own-vehicle, live stats (frame, points,
kB/frame, fps). Code: `web/static/js/lidar-decoder.js` (mirrors `src/lidar/protocol.py`, checked against the Python encoder under Node)
and `lidar-view.js` (canvas, no dependencies). Verified in Firefox headless with a synthetic scene streamed through the real server path.
Not verified: behaviour with real CARLA data, fps/CPU of the 2-D canvas renderer at 50 k points on a slow device (software projection, one
`fillRect` per point).

### Live verification against CARLA 0.9.16 (Windows PC 192.168.0.104, Town10HD) and a V2X performance fix
CARLA on that PC answers on RPC port **2002** (streaming 2003), not 2000.

- **Passed live:** `tests/integration` (3), `tests/server/test_deployed_server.py` (18): telemetry measured at 4.99 Hz (documented 5 Hz),
  LiDAR protocol-v3 frames decode (≈24.6 k points/frame at 500 k pps × 0.05 s, ≈505 kB/frame), 5 RSUs from 15 traffic lights (S7 junction
  grouping works), and `tests/server/test_live_lidar.py`: sensor reaches *active*, **reconfigure 32 → 64 channels replaces (not adds) the
  sensor**, frame ids are ticked frames, leaving destroys the sensor, **no leaked sensor actors** (counted on the CARLA side).
- **Simulation rate (target 20 FPS), measured from the frame counter:** idle 19.9 · 50 vehicles + 20 pedestrians **15.1** · 1 LiDAR 20.0 ·
  3 LiDAR (1.5 M points/s) **19.7** · 3 LiDAR + 50 vehicles + 20 pedestrians **14.4**. LiDAR was not the bottleneck; the traffic was.
  (Not measured: per-tick p95, which the server does not expose.)
- **Cause and fix:** with `v2x_penetration_rate = 1.0` the V2X update handles ≈ 50 stations, and the line-of-sight check made one update
  O(N³) in pure Python (every pair tested every station). `DSRCChannel.nlos_to_receivers` now does one numpy pass per transmitter (same rule, same
  random-draw order: golden delivery matrices unchanged, equivalence test against the scalar rule), and link records are built once instead of
  being rewritten (`LinkResult` is no longer `frozen`). Update time on the host, N stations + 5 RSUs, with link records:
  | N | before | after |
  |---:|---:|---:|
  | 25 | 21.8 ms | 9.0 ms |
  | 50 | 113.7 ms | 30.6 ms |
  | 100 | 615.5 ms | 142.0 ms |
  | 200 | — | 726 ms |
  The update runs every 10 frames, so N = 50 now costs ≈ 3 ms per frame on average. Still ~O(N²): above ≈ 100 stations lower
  `v2x_penetration_rate` or set `DSRCConfig.record_links=False` (saves ≈ 15 %). **Not re-measured live after the fix** (see below).
- **Memory bug found:** the CARLA LiDAR callback queued frames without limit; when the tick loop stalled the server grew to 8.4 GB. The queue now
  keeps the newest 8 frames (`MAX_QUEUED_FRAMES`) and counts drops.
- **Operational lesson:** two servers connected to one CARLA world both tick it (observed: idle rate doubled to 39.7 FPS, requests timed out).
  Run exactly one server per CARLA world; the Docker container and a local `start_server.py` must not both be connected.

### Live performance follow-up (2026-11-03, CARLA PC 192.168.0.104:2002)
`GET /admin/status` now has `timing_ms` (p50 / p95 / max over the last ~1000 samples of `tick`, `world_tick`, `lidar`, `v2x_update`,
`v2x_publish`; `tick` is the work per frame without the pacing sleep, 50 ms is the 20 FPS budget) and `v2x_phase_ms` (bsm / channel /
discover / threats / infrastructure of the last V2X update, plus the station count).

- **Root cause of the remaining slowdown with traffic: blocking CARLA RPCs per vehicle.** Measured on the real server: `vehicle.bounding_box`
  **1.1 ms** and `get_physics_control()` **1.0 ms** per access; `get_control()`, `get_transform()`, `snapshot.find()` and the snapshot
  accessors are local (≈ 0.001 ms). `create_bsm_from_carla` read the bounding box for every vehicle on every update: BSM phase **104 ms for
  50 vehicles**. Now cached at registration (like the steering limit): **1.0 ms**.
- **V2X update with 50 stations, live:** p95 140 ms → **54 ms** (channel ≈ 46 ms, infrastructure ≈ 15 ms, BSM 1 ms, discovery/threats ≈ 3 ms).
  The update runs every 10th frame, so ≈ 5 ms per frame on average.
- **Simulation rate (frame counter; target 20):** idle 20.0 · traffic only with V2X penetration 0 → **19.9**, V2X off → **20.0**,
  penetration 1.0 → 17.1 (before the BSM fix) · 3 LiDAR 19.4 · 3 LiDAR + 50 vehicles + 20 pedestrians **14.3** (after all fixes).
- **Phase 4 acceptance number (tick p95 ≤ 50 ms), 3 LiDAR sessions + 50 vehicles + 20 pedestrians: p95 = 79.6 ms, p50 = 15.3 ms — NOT met.**
  Breakdown at p95: `world_tick` 75.9 ms (CARLA's own `world.tick()` including sensor generation on the lab PC), `lidar` 0.25 ms,
  `v2x_update` 54.5 ms (one frame in ten). Our own per-frame work is small; the tail is CARLA's `world.tick()` under the combined load
  (3 semantic LiDARs ≈ 1.5 M rays per frame + 70 actors, on a PC that other people also use). Lower `points_per_second`/`channels`,
  fewer sessions or traffic, or a dedicated CARLA machine will help; this is a capacity limit of the lab PC, not a defect found in the server code.
- **Test hygiene:** `tests/server/test_live_lidar.py` counts only sensors attached to its own vehicles (the shared lab world gets foreign
  LiDARs that appear and disappear), and reports rather than asserts the tick p95.
- **Lighter traffic (the lab PC is modest):** the live test now defaults to `LIVE_TRAFFIC=25` vehicles + `LIVE_PEDESTRIANS=20`. With 3 LiDAR sessions:
  **17.5 FPS, tick p95 61.9 ms** (p50 16.1 ms; `world_tick` p95 59.2 ms, `lidar` p95 0.28 ms, `v2x_update` max 20.7 ms). Still above the 50 ms p95 target,
  and again almost entirely CARLA's `world.tick()`. For the 20 FPS target on this PC use fewer LiDAR sessions or lower `points_per_second`.
