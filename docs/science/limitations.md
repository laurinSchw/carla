# Known limitations and modelling assumptions

What a reader of results from this platform must know, in one place. S/L/C/D numbers refer to `REFACTOR_HANDOFF.md`;
what was changed for each, with numbers, is in `CHANGELOG.md`; the channel equations are in `model.md`. Nothing here has
been validated against measurement data. Status 2026-11-03; if this page disagrees with the code, the code wins.

## Radio / V2X

* **Parameters are project defaults.** Path-loss exponents, shadowing, sensitivity, the logistic steepness and the
  contention term are not calibrated against data; the sources named in the code are unverified (`model.md` section 2).
  Inside the default 75 m awareness range a LOS link is effectively lossless on the radio side (`model.md` section 3).
* **The MAC term is a placeholder** (S6): `p_mac = max(0.35, exp(-0.005 (N-1)))`. The ≈ 14 % loss within 50 m seen in live
  data is this term at ≈ 30 stations, not radio physics. Packet-loss conclusions are about the placeholder until it is calibrated (D5).
* **Shadowing is i.i.d. per packet** and a sigmoid is applied to an already random received power before a second draw (S5).
* **No noise floor, interference, hidden terminals, half-duplex or collisions**; "margin" is power above sensitivity.
* **Zero latency** (S21): every delivered message arrives in the update in which it was sent.
* **NLOS = another registered vehicle within 2.5 m of the tx-rx line** (S20). Buildings are ignored; vehicles that are not
  V2X stations do not block.
* **Every message type goes through the same channel** (BSM, MAP, SPAT, VRUAM, DENM; S4/S13). The RSU `broadcast_range_m`
  (300 m) is informational only; reception is decided by the channel plus the awareness range (BSM, MAP, SPAT, VRUAM,
  measured from the transmitter) or, for a DENM, its own relevance distance measured from the event.
* **Fixed powers, no antenna heights:** 23 dBm for vehicles and VRU devices, 33 dBm for RSUs and admin hazards; the
  attributions to ETSI EN 302 663 / ETSI TR 101 607 are unverified (S16). Omnidirectional 2 dBi antennas.
* **Which vehicles transmit** is decided by `v2x_penetration_rate` (default 1.0) (S3, D2). Contender counts, and with them the
  MAC term, depend on that rate; stations that are not registered are invisible to the model.
* **RSUs:** one per group of traffic lights as reported by CARLA's `get_group_traffic_lights()` (S7); live on Town10HD this gave
  5 RSUs from 15 lights.
* **Threat levels** (S14): velocity vectors and vehicle extents; thresholds (100 m; TTC 10 / 5 / 2 s) are project-defined.
* **Update rate:** all messages once per V2X update, 2 Hz by default (`v2x_update_rate_hz`). SAE J2945/1 (BSM) and ETSI
  EN 302 637-2 (CAM) use other rates and triggering rules; ETSI CAM triggering is not implemented (S18; rates unverified).
* **Unverified standard attributions:** DENM cause-code numbers (S15), SPAT message structure, the power classes above.
* **Not implemented:** CPM generation, cooperative perception (`enable_bidirectional_sharing` is a stub that shares nothing),
  C-V2X (PC5).

## Scale

* The V2X update runs on the simulation thread every 10th frame and is ~O(N²) in the number of stations. Host measurements with
  link records: 9 ms (25 stations), 31 ms (50), 142 ms (100), 726 ms (200), against a 50 ms frame (CHANGELOG, "Live verification").
  Live with 50 stations: p95 54 ms. Above ≈ 100 stations lower `v2x_penetration_rate` or switch link recording off.
* Phase 4 target (tick p95 ≤ 50 ms with 3 LiDAR sessions and traffic) is **not met** on the lab PC (79.6 ms; CARLA's own `world.tick()` dominates).
* Recorded link files grow with the square of the stations (≈ 6 MB/minute at 25 vehicles + 5 RSUs).

## Simulation and drivers

* Synchronous mode, fixed 0.05 s step, physics substepping (10 x 0.01 s); `use_hybrid_physics` stays off.
* **Only the Traffic Manager drives.** Client vehicles are on autopilot too; clients cannot steer them (D3).
* **Driver profiles** (cautious / normal / aggressive, seeded) are project defaults, not calibrated; no destination routing.
* **V2X advisory** (`advisory_enabled`): the mechanism is verified live; its effect on speeds or collisions is **not measured**
  (needs a same-seed A/B over several seeds).
* **A disconnected client** keeps its session for `session_disconnect_grace_s` (default 30 s, D4); reconnecting in time keeps it.

## LiDAR

* Sensors exist only on request, within a global budget (`lidar_max_sensors` 4, `lidar_max_points_per_second` 2 000 000);
  both limits are project defaults.
* Verified live (CHANGELOG, Phase 4): frames decode, reconfigure replaces the sensor, no leaked sensors. Not separately verified:
  the semantic tag table against CARLA 0.9.16 and the attribute ranges in `lidar_schema()`.
* Streams to viewers are subsampled (seeded, stable per point); recordings never are.
* **Wire format:** protocol v3; on 2026-11-03 the field bits changed to the web UI's layout (`xyz=1, tag_u8=2, tag_u32=4, object_idx=8,
  intensity=16, cos_inc=32`) under the same version number. Raw LiDAR recorded before that date uses the old bits (`tag=2, object_idx=4,
  intensity=8, cos_inc=16`); their manifests have no `lidar_protocol` key. Do not decode them with the current decoder.

## The fake world (`src/testing/fake_carla`)

* A deterministic test double, **not a simulator**: vehicles follow fixed rings at a fixed speed (only the TM speed offset changes it),
  no physics, no collisions, no car following, no traffic-light obedience; walkers walk straight lines on sidewalks; the LiDAR casts
  rays against boxes and a flat ground. Other per-vehicle TM settings are stored and ignored.
* Runs on it are marked `"fake_carla": true` in the manifest and `"fake": true` in `/api/status`. Its numbers are never results.

## Reproducibility of runs

* `python -m src.cli run scenario.yaml` starts the world paused, spawns and starts the recording on the simulation thread, then ticks
  without real-time pacing; each scripted command runs just before the tick with its index (`at_tick`). On the **fake world** two runs of
  one scenario give byte-identical `v2x_links.jsonl.gz`, `stations.jsonl`, `lidar_meta.jsonl` and raw LiDAR frames
  (`tests/e2e/test_scenarios.py`; executed 2026-11-03: identical sha256 for all, 239 LiDAR frames identical). This checks the
  server's own determinism (seeds, command timing, recorder), not CARLA's.
* On **real CARLA** this is **not verified**. Measured earlier with real-time pacing (15 vehicles, same seed twice): same drivers and
  spawn points, trajectories within ~cm for most vehicles over 20 s, one vehicle 3 m off; not bit-identical. Headless same-seed runs
  against the lab CARLA have not been made yet; until they have, treat CARLA runs as seeded but **not** reproducible and average over seeds.
* Interactive runs (`/api/runs` on a real-time server) execute admin and client commands at whatever frame they arrive; their
  timing relative to frames is not reproducible (`manifest.reproducibility.realtime_pacing: true`).
* Scripted commands count ticks from the release of the paused loop. A command whose target does not exist (e.g. `vehicle: first`
  with none spawned) fails loudly: event `scheduled_command_failed`, a log line, CLI exit code 2; the run continues and the manifest
  keeps the scenario.
* LiDAR sensors requested by a scenario are pinned by default: the wall-clock keep-alive release would make the run depend on wall time.
* Even without bit-identical repeats, every V2X record was computed from the stations recorded next to it (`stations.jsonl` is the
  ground truth of the same update), so an analysis never needs the simulator again.
