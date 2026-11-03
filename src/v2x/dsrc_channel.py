"""
DSRC/WAVE (IEEE 802.11p) Channel Model for V2X Simulation

A deliberately simple, seeded, in-process packet-delivery model used for every message
type (BSM, MAP, SPAT, VRUAM). See docs/science/model.md for every equation, parameter,
its status (project-defined / unverified) and the known limitations. Summary:

  - P_rx = P_tx + 2·G − [L0 + 10·n·log10(d/d0) + X], X ~ N(0, σ²) drawn i.i.d. per packet
    (no spatial or temporal correlation). L0 is free-space loss at d0 = 10 m.
  - rx margin = P_rx − sensitivity (NOT an SNR: no noise floor, no interference).
  - p_channel = logistic(k · margin);  p_mac = EXPERIMENTAL contention term (uncalibrated);
    delivered = Bernoulli(p_channel · p_mac), one draw per (tx, rx, message).
  - LOS/NLOS: NLOS if another registered vehicle lies within 2.5 m of the tx→rx segment.
    Buildings are ignored.
  - Everything is delivered in the same tick (zero latency) — a modelling assumption.

The default exponents, shadowing σ, sensitivity, TX power and sigmoid steepness are PROJECT
DEFAULTS, not values fitted to or verified against a published measurement set. The works
below are the intended background; which numbers they actually support has NOT been checked:
  - Paier et al.: V2V channel measurements at 5.2 GHz (Wireless Personal Communications).
  - Sommer et al. (2011): empirical 802.11p shadowing model, WONS.
  - ETSI EN 302 663 (ITS-G5 access layer): the 23 dBm default is not verified against it.

All computation is in-process and deterministic (seeded RNG). No threads, no external
broker. Call broadcast_all() once per update tick, then query get_received() per vehicle.
"""

import math
import logging

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import random

from .safety_messages import BSMCore

logger = logging.getLogger(__name__)

# ── Physical constants ─────────────────────────────────────────────────────────
_C_SPEED_OF_LIGHT = 3.0e8       # m/s
_DSRC_FREQ_HZ = 5.9e9           # 5.9 GHz DSRC center frequency (ETSI EN 302 663)
_DSRC_LAMBDA = _C_SPEED_OF_LIGHT / _DSRC_FREQ_HZ  # wavelength ≈ 0.0508 m

# Friis free-space path loss at reference distance d0 = 10 m:
#   L0 = 20·log10(4π·d0 / λ)
#   L0 = 20·log10(4π·10 / 0.05085) ≈ 67.9 dB
# (Note: the Friis loss at d0=1 m would be ≈ 47.8 dB; this model uses d0=10 m.)
_REFERENCE_DIST_M = 10.0
_L0_DB = 20.0 * math.log10(4.0 * math.pi * _REFERENCE_DIST_M / _DSRC_LAMBDA)
# _L0_DB ≈ 67.9 dB

# Carrier sensing range for CSMA/CA contention counting (IEEE 802.11 CS threshold)
_CARRIER_SENSE_RANGE_M = 300.0


@dataclass(slots=True)
class LinkResult:
    """Outcome of one (transmitter, receiver, message) attempt through the channel.

    ``p_channel`` is the logistic reception probability of the received power over the
    sensitivity floor; ``p_mac`` is the EXPERIMENTAL contention term; ``p_total`` is their
    product and the probability of the single Bernoulli draw that decides the radio outcome.
    The two causes of a radio drop cannot be told apart after one combined draw, so it is
    labelled ``"channel_or_mac"``. ``delivered`` means "reached the receiver's application":
    when the radio delivered the packet but the network's awareness-range filter discards it,
    the record says ``delivered=False, drop_cause="out_of_awareness_range"``. Records are not changed after
    creation (not ``frozen``: frozen dataclasses cost about twice as much to build, and N stations make N² of them).
    """
    frame: int
    sim_time: float
    tx_id: int
    rx_id: int
    msg_type: str        # "BSM" | "SPAT" | "MAP" | "VRUAM" (| "DENM" | "CPM" when generated)
    link_type: str       # "V2V" | "I2V" | "P2V"
    distance_m: float
    los: bool
    tx_power_dbm: float
    path_loss_db: float
    shadowing_db: float
    rx_power_dbm: float
    rx_margin_db: float  # rx_power_dbm - rx_sensitivity_dbm (not an SNR: no noise floor, no interference)
    p_channel: float
    p_mac: float
    p_total: float
    delivered: bool
    drop_cause: Optional[str] = None


@dataclass
class DSRCConfig:
    """
    DSRC/WAVE channel model parameters.

    ALL defaults are project defaults — plausible for 5.9 GHz urban V2V but not fitted to or
    verified against a cited dataset (docs/science/model.md). Tune them to explore path loss,
    shadowing, TX power or congestion without changing simulation logic.
    """

    # ── Transmitter / receiver ────────────────────────────────────────────────
    tx_power_dbm: float = 23.0        # project default OBU power (not verified against ETSI EN 302 663)
    antenna_gain_dbi: float = 2.0     # Omnidirectional OBU antenna gain (TX + RX each)
    rx_sensitivity_dbm: float = -85.0 # 802.11p typical receiver sensitivity floor

    # ── Path loss model (log-distance + log-normal shadowing) ────────────────
    # What these defaults actually give (no shadowing; reproduced by tests/v2x/test_channel_model_table.py):
    #   P_rx at 50 m LOS = -56.9 dBm, 100 m LOS = -63.9 dBm, 150 m NLOS = -78.5 dBm
    #   LOS reaches the -85 dBm floor at ~830 m, NLOS at ~240 m. Inside the 75 m awareness range
    #   the LOS link is effectively lossless; losses appear for NLOS beyond ~100 m and for
    #   LOS only at several hundred metres. (Earlier comments claiming -71/-77/-87 dBm were wrong.)
    path_loss_exponent_los: float = 2.3    # project default (urban LOS literature range is roughly 2-2.5; unverified)
    path_loss_exponent_nlos: float = 3.2   # project default
    shadowing_std_los_db: float = 5.5      # project default, i.i.d. per packet
    shadowing_std_nlos_db: float = 8.0     # project default, i.i.d. per packet

    # ── Channel busy ratio (UNUSED) ───────────────────────────────────────────
    # Not read by the model. The contention term depends only on the number of stations
    # within carrier-sense range (see _csma_success_prob, EXPERIMENTAL). Kept so existing
    # configs still construct; reported in log lines only.
    channel_busy_ratio: float = 0.15

    # ── LOS/NLOS detection ────────────────────────────────────────────────────
    # When True, vehicles lying on the TX→RX line segment cause NLOS penalty.
    # When False, LOS path loss and shadowing parameters are always used.
    enable_nlos_model: bool = True

    # ── PRR sigmoid steepness ─────────────────────────────────────────────────
    # Larger values produce a sharper PRR transition around the sensitivity floor.
    # k = 2.0 per dB is a project choice (not verified against Sommer et al. 2011).
    prr_sigmoid_k: float = 2.0

    # ── Reproducibility ───────────────────────────────────────────────────────
    random_seed: int = 42

    # ── Observability ─────────────────────────────────────────────────────────
    # Keep one LinkResult per attempt of the last tick in DSRCChannel.last_links.
    # N stations create N*(N-1) records per tick: switch off for very large runs.
    record_links: bool = True


class DSRCChannel:
    """
    In-process DSRC/WAVE (IEEE 802.11p) channel model.

    Simple in-process, seeded packet-delivery model with a probabilistic, broadcast and contention-sensitive
    character (log-distance path loss, i.i.d. shadowing, an EXPERIMENTAL contention term). It is a deliberate
    simplification, not a validated model of the DSRC/WAVE medium: see docs/science/model.md for what it does and
    does not represent. No external broker or network infrastructure is required.

    Usage inside V2XNetwork.update():

        positions = {
            vid: (bsm.x_m, bsm.y_m)
            for vid, bsm in self.bsm_messages.items()
        }
        self._dsrc_channel.broadcast_all(self.bsm_messages, positions)

    Then in _discover_neighbors():

        received = self._dsrc_channel.get_received(receiver_id)
        # received[sender_id] = BSMCore if packet was delivered, absent if dropped

    No shutdown or teardown is required — there is no external state to release.
    """

    def __init__(self, config: Optional[DSRCConfig] = None) -> None:
        self.config = config or DSRCConfig()
        # Private RNG — seeded for reproducibility; isolated from global random state
        self._rng = random.Random(self.config.random_seed)

        # _received[receiver_id][sender_id] = BSMCore for this tick
        self._received: Dict[int, Dict[int, BSMCore]] = {}

        # Rolling statistics for research analysis
        self.stats: Dict[str, float] = {
            "total_broadcasts": 0.0,
            "total_deliveries": 0.0,
            "total_drops": 0.0,
            "prr": 1.0,
            "avg_rx_margin_db": 0.0,
        }
        self._rx_margin_ring: List[float] = []  # rolling rx-margin samples (capped at 500)

        # LinkResult per attempt of the last broadcast_all() tick (+ anything appended by the network)
        self.last_links: List[LinkResult] = []

        logger.debug(
            "DSRCChannel initialized: TX=%.0f dBm, sensitivity=%.0f dBm, "
            "n_LOS=%.2f, n_NLOS=%.2f, CBR=%.2f",
            self.config.tx_power_dbm,
            self.config.rx_sensitivity_dbm,
            self.config.path_loss_exponent_los,
            self.config.path_loss_exponent_nlos,
            self.config.channel_busy_ratio,
        )

    # ── Public API ─────────────────────────────────────────────────────────────

    def broadcast_all(
        self,
        bsm_messages: Dict[int, BSMCore],
        positions: Dict[int, Tuple[float, float]],
        frame: int = 0,
        sim_time: float = 0.0,
        awareness_range_m: Optional[float] = None,
        contention_positions: Optional[Dict] = None,
    ) -> None:
        """
        Compute channel delivery for every (sender, receiver) pair this tick.

        Args:
            bsm_messages: Current BSMCore per registered vehicle.
            positions: {vehicle_id: (x_m, y_m)} in CARLA world space.
                       Typically derived from bsm.x_m / bsm.y_m (CARLA metres),
                       which store CARLA world coordinates (not geodetic).
            frame, sim_time: stamped on the LinkResult records only.
            awareness_range_m: application filter applied after the radio; affects the LinkResult
                outcome only (``get_received`` still reports radio delivery).
            contention_positions: every station that contends for the channel (vehicles AND RSUs, VRUs, hazard sources),
                keyed so that ids cannot collide; defaults to ``positions``. A BSM and a MAP sent in the same tick must see
                the same contention domain.

        Call once per V2X update tick. Results are stable until the next call.
        """
        self._received = {vid: {} for vid in bsm_messages}
        self.last_links = []
        vehicle_ids = list(bsm_messages.keys())

        for sender_id, bsm in bsm_messages.items():
            tx_pos = positions.get(sender_id)
            if tx_pos is None:
                continue

            n_contenders = self._count_contenders(sender_id, tx_pos, contention_positions or positions)
            nlos_by_rx = (
                self.nlos_to_receivers(sender_id, tx_pos, positions)
                if self.config.enable_nlos_model else {}
            )

            for receiver_id in vehicle_ids:
                if receiver_id == sender_id:
                    continue

                rx_pos = positions.get(receiver_id)
                if rx_pos is None:
                    continue

                nlos = nlos_by_rx.get(receiver_id, False) and _euclidean(tx_pos, rx_pos) >= 0.5
                link, radio_delivered = self._evaluate(
                    sender_id, receiver_id, tx_pos, rx_pos, nlos, n_contenders,
                    "BSM", "V2V", None, frame, sim_time, awareness_range_m,
                )
                if radio_delivered:
                    self._received[receiver_id][sender_id] = bsm
                if self.config.record_links:
                    self.last_links.append(link)

        self._flush_stats()

    def evaluate_link(
        self,
        tx_id: int,
        rx_id: int,
        tx_pos: Tuple[float, float],
        rx_pos: Tuple[float, float],
        nlos: bool,
        n_contenders: int,
        msg_type: str = "BSM",
        link_type: str = "V2V",
        tx_power_dbm: Optional[float] = None,
        frame: int = 0,
        sim_time: float = 0.0,
        awareness_range_m: Optional[float] = None,
        awareness_ref_xy: Optional[Tuple[float, float]] = None,
    ) -> LinkResult:
        """Decide one transmission attempt and update the channel statistics.

        Every message type (BSM, SPAT, MAP, VRUAM) uses this one path. ``tx_power_dbm`` is the
        transmitter's own power (OBU, RSU, VRU device); None means ``config.tx_power_dbm``.
        ``awareness_range_m`` (None = no filter) turns a radio delivery from farther away into
        ``delivered=False, drop_cause="out_of_awareness_range"`` in the record. The range is measured from the transmitter,
        or from ``awareness_ref_xy`` (a DENM: its event position) when given.
        Consumes the seeded RNG in a fixed order (one Gaussian, then one uniform draw).
        """
        link, _ = self._evaluate(tx_id, rx_id, tx_pos, rx_pos, nlos, n_contenders, msg_type, link_type,
                                 tx_power_dbm, frame, sim_time, awareness_range_m, awareness_ref_xy)
        if self.config.record_links:
            self.last_links.append(link)
        return link

    def _evaluate(self, tx_id, rx_id, tx_pos, rx_pos, nlos, n_contenders, msg_type, link_type,
                  tx_power_dbm, frame, sim_time, awareness_range_m, awareness_ref_xy=None):
        """Core of ``evaluate_link``: returns ``(record, radio_delivered)`` without storing the record."""
        cfg = self.config
        tx_power = cfg.tx_power_dbm if tx_power_dbm is None else tx_power_dbm
        distance = _euclidean(tx_pos, rx_pos)
        self.stats["total_broadcasts"] += 1

        if distance < 0.5:
            # Co-located stations: deterministic delivery, no radio draw
            self.stats["total_deliveries"] += 1
            gain = tx_power + 2.0 * cfg.antenna_gain_dbi
            return LinkResult(
                frame, sim_time, tx_id, rx_id, msg_type, link_type, distance, True, tx_power,
                0.0, 0.0, gain, gain - cfg.rx_sensitivity_dbm, 1.0, 1.0, 1.0, True, None,
            ), True

        p_rx, path_loss, shadowing = self._rx_power_parts(distance, nlos, tx_power)
        margin = p_rx - cfg.rx_sensitivity_dbm
        self._record_margin(margin)
        p_channel = self._p_channel(margin)
        p_mac = self._csma_success_prob(n_contenders)
        p_total = p_channel * p_mac
        radio_delivered = self._rng.random() < p_total
        self.stats["total_deliveries" if radio_delivered else "total_drops"] += 1
        if not radio_delivered:
            delivered, cause = False, "channel_or_mac"
        elif awareness_range_m is not None and (
                distance if awareness_ref_xy is None else _euclidean(awareness_ref_xy, rx_pos)) > awareness_range_m:
            delivered, cause = False, "out_of_awareness_range"
        else:
            delivered, cause = True, None
        return LinkResult(
            frame, sim_time, tx_id, rx_id, msg_type, link_type, distance, not nlos, tx_power,
            path_loss, shadowing, p_rx, margin, p_channel, p_mac, p_total, delivered, cause,
        ), radio_delivered

    def get_received(self, receiver_id: int) -> Dict[int, BSMCore]:
        """
        Return the BSMs successfully received by receiver_id this tick.

        Args:
            receiver_id: Vehicle querying its inbox.

        Returns:
            Mapping of sender_id → BSMCore for every packet that survived
            the channel model. Absent senders had their packets dropped.
        """
        return self._received.get(receiver_id, {})

    def get_stats(self) -> dict:
        """Channel statistics for research analysis and reporting."""
        return self.stats.copy()

    # ── Channel physics ─────────────────────────────────────────────────────────

    def _received_power_dbm(self, distance_m: float, nlos: bool) -> float:
        """Received signal power in dBm (one shadowing draw); see ``_rx_power_parts``."""
        return self._rx_power_parts(distance_m, nlos)[0]

    def _rx_power_parts(
        self, distance_m: float, nlos: bool, tx_power_dbm: Optional[float] = None
    ) -> Tuple[float, float, float]:
        """
        Received power using log-distance path loss + log-normal shadowing.

            P_rx = P_tx + 2·G_ant − [L0 + 10·n·log10(d/d0) + X_σ]

        where X_σ ~ N(0, σ²) is drawn i.i.d. per call (no spatial or temporal correlation).
        Returns ``(p_rx_dbm, path_loss_db, shadowing_db)``. The exponents and σ are project
        defaults, not a fit to a cited dataset (see docs/science/model.md).
        """
        cfg = self.config
        n = cfg.path_loss_exponent_nlos if nlos else cfg.path_loss_exponent_los
        sigma = cfg.shadowing_std_nlos_db if nlos else cfg.shadowing_std_los_db

        path_loss = _L0_DB + 10.0 * n * math.log10(distance_m / _REFERENCE_DIST_M)
        shadowing = self._rng.gauss(0.0, sigma)

        # Both TX and RX carry the same omnidirectional antenna
        tx_power = cfg.tx_power_dbm if tx_power_dbm is None else tx_power_dbm
        system_gain = tx_power + 2.0 * cfg.antenna_gain_dbi
        return system_gain - path_loss - shadowing, path_loss, shadowing

    def _compute_prr(
        self, distance_m: float, nlos: bool, n_contenders: int
    ) -> float:
        """
        Packet reception probability = P_channel × P_mac for one fresh shadowing draw.

        P_channel: logistic function of the receive margin (P_rx − sensitivity), steepness
          ``prr_sigmoid_k``; 0.5 exactly at the sensitivity floor. The shape is a project
          choice; Sommer et al. (2011) is NOT verified as its source (docs/science/model.md).
        P_mac: ``_csma_success_prob`` — EXPERIMENTAL, uncalibrated.
        """
        p_rx = self._received_power_dbm(distance_m, nlos)
        margin = p_rx - self.config.rx_sensitivity_dbm
        self._record_margin(margin)

        return self._p_channel(margin) * self._csma_success_prob(n_contenders)

    def _p_channel(self, margin_db: float) -> float:
        """Logistic channel term of the receive margin: the ONE formula used by evaluate_link and _compute_prr."""
        return 1.0 / (1.0 + math.exp(-self.config.prr_sigmoid_k * margin_db))

    def _record_margin(self, margin_db: float) -> None:
        """Keep the last 500 receive margins for the rolling average (bounded memory)."""
        self._rx_margin_ring.append(margin_db)
        if len(self._rx_margin_ring) > 500:
            self._rx_margin_ring = self._rx_margin_ring[-500:]

    def _csma_success_prob(self, n_contenders: int) -> float:
        """
        MAC / contention success probability for N stations in carrier-sense range.

        EXPERIMENTAL — UNCALIBRATED. The functional form ``max(0.35, exp(-0.005·(N-1)))`` and
        both constants are project placeholders. They are not derived from a cited measurement or
        an analytical broadcast model (Bianchi's model is for saturated unicast DCF, and the
        references this function used to cite were not verified). Values produced:
        N=1 → 1.00, 20 → 0.91, 50 → 0.78, 100 → 0.61, 200 → 0.37, ≥ 400 → 0.35 (floor).
        Replace with a cited model before drawing conclusions about congestion.
        """
        if n_contenders <= 1:
            return 1.0
        lambda_contention = 0.005  # placeholder
        return max(0.35, math.exp(-lambda_contention * (n_contenders - 1)))

    # ── Geometry helpers ────────────────────────────────────────────────────────

    def _is_nlos(
        self,
        sender_id: int,
        receiver_id: int,
        tx: Tuple[float, float],
        rx: Tuple[float, float],
        all_positions: Dict[int, Tuple[float, float]],
    ) -> bool:
        """
        Determine LOS/NLOS by 2-D geometric occlusion by VEHICLES only.

        A third registered station causes NLOS if its position lies within 2.5 m (about a
        vehicle width) of the TX→RX segment. Buildings, vegetation and unregistered vehicles are
        ignored, so in open or building-lined maps with few stations the link is almost always LOS.
        This is a placeholder, not the Sommer et al. building-shadowing model.
        """
        for vid, pos in all_positions.items():
            if vid == sender_id or vid == receiver_id:
                continue
            if _point_to_segment_dist(pos, tx, rx) < 2.5:
                return True
        return False

    def nlos_to_receivers(
        self,
        sender_id,
        tx: Tuple[float, float],
        positions: Dict[int, Tuple[float, float]],
        blockers: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> Dict[int, bool]:
        """LOS/NLOS from one transmitter to every station in ``positions``, in one vectorised pass.

        Same rule as ``_is_nlos`` (another station within 2.5 m of the tx→rx segment, excluding the two ends),
        but numpy instead of a Python loop per pair: the pairwise loop made one update cost O(N^3).
        ``sender_id`` may be any key not present in ``positions`` (RSU, VRU). ``blockers`` defaults to ``positions``.
        """
        rx_ids = [i for i in positions if i != sender_id]
        if not rx_ids:
            return {}
        blk = positions if blockers is None else blockers
        blk_ids = list(blk)
        R = np.array([positions[i] for i in rx_ids], dtype=np.float64)          # (M, 2) receivers
        Q = np.array([blk[i] for i in blk_ids], dtype=np.float64)               # (N, 2) possible blockers
        A = np.asarray(tx, dtype=np.float64)
        seg = R - A                                                             # (M, 2) segment vectors
        seg_len_sq = np.einsum("ij,ij->i", seg, seg)                            # (M,)
        rel = Q[:, None, :] - A                                                 # (N, 1->M, 2)
        dot = np.einsum("nmj,mj->nm", np.broadcast_to(rel, (len(Q), len(R), 2)), seg)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(seg_len_sq > 0.0, dot / seg_len_sq, 0.0)
        t = np.clip(t, 0.0, 1.0)
        proj = A + t[:, :, None] * seg[None, :, :]                              # (N, M, 2)
        dist = np.hypot(Q[:, None, 0] - proj[:, :, 0], Q[:, None, 1] - proj[:, :, 1])
        close = dist < 2.5
        # the two ends of the link never block it
        blk_index = {i: k for k, i in enumerate(blk_ids)}
        for m, rid in enumerate(rx_ids):
            k = blk_index.get(rid)
            if k is not None:
                close[k, m] = False
        sk = blk_index.get(sender_id)
        if sk is not None:
            close[sk, :] = False
        blocked = close.any(axis=0)
        return {rid: bool(blocked[m]) for m, rid in enumerate(rx_ids)}

    def _count_contenders(
        self,
        sender_id: int,
        tx_pos: Tuple[float, float],
        all_positions: Dict[int, Tuple[float, float]],
    ) -> int:
        """
        Count vehicles within IEEE 802.11 carrier-sense range of the sender.

        These are the stations that contend for the channel medium during
        this sender's transmission slot (CSMA/CA contention domain).
        """
        count = 1  # the sender itself
        for vid, pos in all_positions.items():
            if vid == sender_id:
                continue
            if _euclidean(tx_pos, pos) <= _CARRIER_SENSE_RANGE_M:
                count += 1
        return count

    # ── Internal stats ──────────────────────────────────────────────────────────

    def _flush_stats(self) -> None:
        """Refresh derived stats after a broadcast_all() call."""
        total = self.stats["total_broadcasts"]
        if total > 0:
            self.stats["prr"] = self.stats["total_deliveries"] / total
        if self._rx_margin_ring:
            self.stats["avg_rx_margin_db"] = (
                sum(self._rx_margin_ring) / len(self._rx_margin_ring)
            )


# ── Module-level geometry utilities ────────────────────────────────────────────

def _euclidean(
    p1: Tuple[float, float], p2: Tuple[float, float]
) -> float:
    """Euclidean distance between two 2-D points."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def _point_to_segment_dist(
    p: Tuple[float, float],
    a: Tuple[float, float],
    b: Tuple[float, float],
) -> float:
    """
    Perpendicular distance from point p to the finite line segment a–b.
    Returns the distance to the nearest endpoint when the projection falls
    outside the segment.
    """
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)
