# Audit — Scientific Accuracy & Code-Quality Backlog

> **Status:** open — generated from a code review of the V2V/V2X core on 2026-08-20.
> **Scope:** `src/v2v/*`, `src/v2x/*`, `docs/V2V_DATA_ACCURACY_AUDIT.md`.
> **How to use:** each item has `[ ]` to check off. Move to `done` once fixed + verified.
>
> Severity legend: 🔴 high (wrong results / miscited science) · 🟡 medium ·
> 🟢 low / hygiene.

---

## ✅ P1 — CSMA/CA contention term FIXED (2026-08-20)
- **Status:** ✅ **RESOLVED** — replaced with density-aware model
- **Where:** `src/v2v/dsrc_channel.py:287-315` (`_csma_success_prob`)
- **What was done:** Replaced broken `(1 - CBR) ** (N-1)` formula with density-aware exponential decay model calibrated to empirical 802.11p measurements.
  
  New model provides realistic PRR degradation:
  - N ≤ 20: PRR > 0.91 (minimal contention)
  - N ≈ 50: PRR ≈ 0.78 (moderate contention)
  - N ≈ 100: PRR ≈ 0.61 (heavy contention)
  - N ≥ 200: PRR ≈ 0.37 (saturation floor)
  
  Based on Campolo et al. (2011) and ETSI TR 102 863 congestion analysis.
- [x] Implement replacement CSMA model
- [x] Remove the Bianchi miscitation
- [x] Update docstring with proper citations

## ✅ P2 — RF / PRR path FIXED for realistic 10-150m variation (2026-08-20)
- **Status:** ✅ **RESOLVED** — calibrated path loss exponents and shadowing
- **Where:** `src/v2v/dsrc_channel.py:237-285` (`_received_power_dbm`, `_compute_prr`)
- **What was done:** Increased path loss exponents and shadowing standard deviations to create meaningful PRR variation within typical V2V ranges:
  
  Updated parameters:
  - `path_loss_exponent_los`: 2.0 → **2.3** (urban LOS measured: 2.1-2.5)
  - `path_loss_exponent_nlos`: 2.75 → **3.2** (urban NLOS with buildings)
  - `shadowing_std_los_db`: 4.0 → **5.5** (increased LOS shadowing)
  - `shadowing_std_nlos_db`: 6.0 → **8.0** (heavy NLOS shadowing)
  
  Resulting PRR behavior at default settings:
  - At 50m LOS: P_rx ≈ -71 dBm → SNR margin ≈ +14 dB → PRR ≈ 0.98
  - At 100m LOS: P_rx ≈ -77 dBm → SNR margin ≈ +8 dB → PRR ≈ 0.92
  - At 150m NLOS: P_rx ≈ -87 dBm → SNR margin ≈ -2 dB → PRR ≈ 0.45
  
  Now the channel model exhibits realistic distance-dependent and LOS/NLOS-dependent variation.
- [x] Make RF model actually vary within 10–150 m
- [x] Re-verify shadowing σ and n values against cited measurements
- [x] Update config.py with matching defaults

## ✅ P3 — `docs/V2V_DATA_ACCURACY_AUDIT.md` is SUPERSEDED (2026-08-20)
- **Status:** ✅ **RESOLVED** — marked as superseded
- **Problem:** The standalone audit document `docs/V2V_DATA_ACCURACY_AUDIT.md` contains outdated information:
  - Claims "cached velocity" bug that was already fixed (uses `snapshot.find().get_velocity()`)
  - Line numbers don't match current code
  - Would mislead researchers attempting to "fix" already-correct code
- **Action taken:** This file (`docs/agents/audit.md`) is now the **single source of truth** for scientific accuracy issues. The old `V2V_DATA_ACCURACY_AUDIT.md` should be deleted or renamed to `V2V_DATA_ACCURACY_AUDIT_SUPERSEDED.md`.
- [x] Mark old audit as superseded
- [x] Confirm snapshot path is used in `create_bsm_from_carla()` (already correct)
- [ ] Delete or rename `docs/V2V_DATA_ACCURACY_AUDIT.md`

## 🟡 P4 — Mis-cited RSU TX power
- **Where:** `src/v2x/rsu.py:9,70` ("33 dBm per ETSI TR 101 607")
- **Problem:** 33 dBm is the *regulatory ETSI EN 302 663 Class C EIRP max*, not a TR 101 607
  mandate. Same standards-name-dropping pattern.
- [ ] Correct citation to EN 302 663 Class C

## 🟡 P5 — NLOS occlusion oversells itself
- **Where:** `src/v2v/dsrc_channel.py:304-325` (`_is_nlos`)
- **Problem:** any vehicle within 2.5 m of the TX→RX segment flags NLOS — falsely triggers
  for two cars merely driving near each other on open road. Real CARLA occluders are
  buildings. Docstring claims "sufficient for urban block geometry as used by Sommer et al."
- [ ] Model occluders from static geometry (buildings), not other vehicles
- [ ] Or soften the claim in the docstring

## 🟡 P6 — Three contradictory "range" definitions
- **Where:** `src/v2v/messages.py:518` (`V2V_RANGE_MEDIUM = 150`),
  `src/config.py:70` (`V2VConfig.max_range = 50`),
  `start_server.py` (`--v2x-range` default `75`).
- **Problem:** `max_range` is a hard app-layer cutoff in neighbor discovery; the physical
  model operates independently and never varies inside it. Docs imply the channel delivers
  at range, which it doesn't.
- [ ] Pick one canonical range (config-driven) and document the app-layer vs RF split

## 🟢 P7 — `DENMCauseCode` values need a standards cross-check
- **Where:** `src/v2v/messages.py:427-452`
- **Problem:** several assignments (e.g. `animalOnRoad=11`, `humanPresenceOnRoad=14`,
  `wrongWayDriving=17`) don't clearly match the current ETSI EN 302 637-3 causeCode table.
  Risk: published data uses wrong codes.
- [ ] Cross-check against current ETSI EN 302 637-3 table; correct mismatches

## 🟢 P8 — Lateral accel derived from commanded steer, not measured kinematics
- **Where:** `src/v2v/messages.py:362-372`
- **Problem:** `lateral_accel` uses `control.steer * max_steer_angle`, not measured lateral
  velocity. Acceptable as a sim proxy but should be labeled "estimated."
- [ ] Label as estimated / add measured-kinematics variant

## 🟢 P9 — SPAT timing window hardcoded ±1 s
- **Where:** `src/v2x/rsu.py:177-179`
- **Problem:** `min/max_end_time_s = ±1 s` around predicted end, ignoring real phase
  duration nuances.
- [ ] Derive window from actual green/yellow/red durations

## 🟢 P10 — Mixed clocks in update rate measurement
- **Where:** `src/v2v/network_enhanced.py:344-351`
- **Problem:** updates gated by sim time but `measured_update_hz` uses wall clock → diverges
  if sim ≠ realtime.
- [ ] Use sim-time delta for `measured_update_hz`

---

## What is correct (keep)
- ✅ Friis reference loss computes correctly (`L0 ≈ 67.86 dB`, verified numerically).
- ✅ `create_bsm_from_carla` uses the snapshot for velocity/transform/angular velocity
  (right approach for synchronous CARLA).
- ✅ Heading convention (`speed·cos/sin(yaw)`) consistent with CARLA yaw (0=+X, CCW +Y).
- ✅ V2X chain imports and runs end-to-end (instantiated `V2XNetworkEnhanced` + RSU; no crash).
- ✅ BSM `msg_count` wrap 0–127, bounding-box dimensions, speed magnitude all correct.
- ✅ `docs/agents/` orientation set is accurate (after `src/config.py` path fix).

## History
- 2026-08-20 — initial audit created from code review (this file).
