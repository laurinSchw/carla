# V2X channel and message model — what the code actually does

Status 2026-11-03. Replaces the claims in `docs/science/audit.md` (stale). The numbers in section 3 are
pinned by `tests/v2x/test_channel_model_table.py`, so this file cannot silently drift from the code.

**Verification status used below:** *project* = a project default or modelling choice, not fitted to or
checked against a published dataset · *unverified* = a standard/paper is named in the code, but nobody has
checked that it supports the number · *verified* = checked (none yet).

## 1. Scope
One in-process packet-delivery model (`src/v2x/dsrc_channel.py`) decides **every** transmission attempt —
BSM (V2V), MAP and SPAT (RSU→vehicle, `I2V`), VRUAM (VRU→vehicle, `P2V`). Each attempt yields one `LinkResult`.
Messages are delivered in the same tick (zero latency) — an assumption, not a measurement (S21).
The application filter *awareness range* (`WorldConfig.v2x_range_m`, default 75 m) is applied after the radio.

## 2. Equations and parameters
For a link of distance `d` (m) between transmitter and receiver:

1. **Path loss.** `PL(d) = L0 + 10·n·log10(d / d0) + X`, `d0 = 10 m`, `L0 = 20·log10(4π·d0/λ) = 67.86 dB`
   (free space, 5.9 GHz, λ = 0.0508 m). `X ~ N(0, σ²)` is drawn **independently for every packet**.
2. **Received power.** `P_rx = P_tx + 2·G − PL(d)` with antenna gain `G = 2 dBi` at each end (omnidirectional).
3. **Receive margin.** `m = P_rx − S`, sensitivity `S = −85 dBm`. (Earlier code called this "SNR margin": it is not an SNR —
   there is no noise floor and no interference.)
4. **Channel term.** `p_channel = 1 / (1 + exp(−k·m))`, `k = 2.0 per dB`.
5. **MAC term (EXPERIMENTAL).** `p_mac = max(0.35, exp(−0.005·(N−1)))`, `N` = stations (vehicles, RSUs, VRUs)
   within 300 m of the transmitter, including itself.
6. **Decision.** One uniform draw: `delivered = U < p_channel · p_mac`. Co-located stations (< 0.5 m) are always delivered.
7. **LOS/NLOS.** NLOS if another registered **vehicle** lies within 2.5 m of the segment between the two stations.
   Buildings are not modelled, so in sparse traffic nearly every link is LOS (S20).

| Parameter | Default | Status |
|---|---|---|
| `tx_power_dbm` (OBU / VRU device) | 23 dBm | project (ETSI EN 302 663 attribution unverified) |
| `RSUConfig.tx_power_dbm` | 33 dBm | project (ETSI TR 101 607 attribution unverified, S16) |
| `antenna_gain_dbi` | 2 dBi | project |
| `rx_sensitivity_dbm` | −85 dBm | project |
| `path_loss_exponent_los` / `_nlos` | 2.3 / 3.2 | project |
| `shadowing_std_los_db` / `_nlos_db` | 5.5 / 8.0 dB | project |
| `prr_sigmoid_k` | 2.0 /dB | project |
| MAC `λ = 0.005`, floor `0.35`, carrier-sense range 300 m | — | project, **uncalibrated** (S6) |
| `channel_busy_ratio` | 0.15 | **unused** by the model |
| seed | 42 | — |

References named in the code, none of them checked against these numbers: Paier et al. (V2V channel
measurements at 5.2 GHz), Sommer et al. 2011 (empirical 802.11p shadowing), ETSI EN 302 663, ETSI TR 101 607.
The earlier citations of Bianchi (2000), Campolo et al. (2011) and "ETSI TR 102 863" for the MAC term were removed:
Bianchi's model is for saturated unicast DCF, and the other two could not be confirmed to say what was claimed.

## 3. What the defaults produce (generated from the code)
Received power without shadowing, and the shadowing-averaged channel term `E[p_channel]` (N = 1, i.e. `p_mac = 1`):

| d (m) | P_rx LOS (dBm) | P_rx NLOS (dBm) | E[p_channel] LOS | E[p_channel] NLOS |
|---:|---:|---:|---:|---:|
| 10 | −40.86 | −40.86 | 1.0000 | 1.0000 |
| 50 | −56.94 | −63.23 | 1.0000 | 0.9966 |
| 100 | −63.86 | −72.86 | 0.9999 | 0.9342 |
| 150 | −67.91 | −78.49 | 0.9989 | 0.7905 |
| 200 | −70.78 | −82.49 | 0.9946 | 0.6223 |
| 300 | −74.83 | −88.13 | 0.9659 | 0.3489 |
| 500 | −79.94 | −95.23 | 0.8182 | 0.1020 |
| 800 | −84.63 | −101.76 | 0.5265 | 0.0187 |
| 1000 | −86.86 | −104.86 | 0.3694 | 0.0068 |

`p_mac` for N = 1, 20, 50, 100, 200, ≥ 400: 1.000, 0.909, 0.783, 0.610, 0.370, 0.350.
Without shadowing, P_rx reaches the sensitivity floor at ≈ 830 m (LOS) and ≈ 240 m (NLOS).

**Consequence:** inside the default 75 m awareness range a LOS link is effectively lossless, and losses
mostly come from NLOS beyond ~100 m, from far LOS links (hundreds of metres), or from the MAC term at high N.
Earlier documentation claimed P_rx ≈ −71 / −77 / −87 dBm at 50 m LOS / 100 m LOS / 150 m NLOS and "meaningful
10–150 m PRR variation"; the code gives −56.9 / −63.9 / −78.5 dBm (S1).

## 4. Known limitations (open)
- **S5** Shadowing is i.i.d. per packet — no spatial/temporal correlation (e.g. Gudmundson-type decorrelation) — and a
  sigmoid is applied to an already random `P_rx` before a second random draw. A threshold decision or a correlated
  shadowing process are not implemented.
- **S6** The MAC term is a placeholder; congestion conclusions need a cited model or measurement-based table.
- **S20** No building occlusion; only registered vehicles block.
- **S21** Zero latency; no half-duplex, no hidden-terminal effect, no channel-busy feedback, no interference.
- **S3** Only registered stations contend and block. With `v2x_penetration_rate < 1` the others are invisible to the model.
- Fixed 23 dBm for VRU devices and omnidirectional antennas at the same height for all stations (no antenna heights).
- No C-V2X (PC5) model.

## 5. Message rates and contents
All message types are produced once per update (default 2 Hz, `v2x_update_rate_hz`). Real rates differ (BSM nominally
10 Hz per SAE J2945/1; CAM 1–10 Hz, ETSI EN 302 637-2 — both unverified here; MAP/SPAT typically 1–10 Hz). BSM contents
follow SAE J2735 fields but position fields are CARLA metres (`x_m, y_m, z_m`), not geodetic. DENM cause-code numbers
are unverified against ETSI EN 302 637-3 (S15). CPM types exist but nothing generates them.
