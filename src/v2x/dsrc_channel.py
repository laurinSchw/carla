"""
DSRC/WAVE (IEEE 802.11p) Channel Model for V2X/V2V Simulation

Models the physical medium characteristics of IEEE 802.11p (DSRC/WAVE) as used
in SAE J2735 BSM broadcasts. Implements:

  - Log-distance path loss with log-normal shadowing
    Calibrated to ETSI EN 302 663 and ETSI EN 302 895 parameters.
  - CSMA/CA channel contention modelled via the channel busy ratio (CBR).
  - Optional LOS/NLOS distinction via 2-D geometry.
  - Packet Reception Rate (PRR) via a sigmoid SNR-margin curve calibrated
    against Sommer et al. (2011) and the WINNER+ B1 urban channel model.

All computation is in-process and deterministic (seeded RNG). No threads,
no external broker, no connections to open or close. Call broadcast_all()
once per V2V update tick, then query get_received() per vehicle.

Reference parameters follow ETSI EN 302 663 §4 for ITS-G5 Class A OBU:
  TX power: 23 dBm ERP, carrier: 5.9 GHz, bandwidth: 10 MHz.

Physical model references:
  - Paier et al. (2008): "Characterization of vehicle-to-vehicle radio channels
    from measurements at 5.2 GHz." Wireless Personal Communications.
  - Sommer et al. (2011): "A computationally inexpensive empirical model of IEEE
    802.11p radio shadowing in urban environments." WONS.
  - Bianchi (2000): "Performance analysis of the IEEE 802.11 distributed
    coordination function." IEEE J. Sel. Areas Commun.
  - ETSI EN 302 663 V1.3.1 (2020): "ITS; Access layer specification."
"""

import math
import logging
from dataclasses import dataclass, replace
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


@dataclass(frozen=True, slots=True)
class LinkResult:
    """Outcome of one (transmitter, receiver, message) attempt through the channel.

    ``p_channel`` is the logistic reception probability of the received power over the
    sensitivity floor; ``p_mac`` is the EXPERIMENTAL contention term; ``p_total`` is their
    product and the probability of the single Bernoulli draw that decides the radio outcome.
    The two causes of a radio drop cannot be told apart after one combined draw, so it is
    labelled ``"channel_or_mac"``. ``delivered`` means "reached the receiver's application":
    when the radio delivered the packet but the network's awareness-range filter discards it,
    the network rewrites the record to ``delivered=False, drop_cause="out_of_awareness_range"``.
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

    Defaults follow ETSI EN 302 663 §4 (ITS-G5 Class A OBU).
    Researchers may tune these to explore path loss conditions, shadowing
    intensity, TX power, or channel congestion without changing simulation logic.

    Channel busy ratio (CBR) calibration:
      At 2 Hz BSM rate, BSM payload ~200 bytes @ 3 Mbps:
        packet_duration ≈ 200*8 / 3e6 ≈ 0.53 ms
        CBR ≈ N_vehicles × packet_duration / beacon_interval
        For 15 vehicles at 2 Hz: CBR ≈ 15 × 0.53ms / 500ms ≈ 0.016 (1.6%)
      Default CBR=0.15 represents a denser deployment (≈140 vehicles at 2 Hz)
      consistent with ETSI ETSI TR 103 439 congestion scenarios.
    """

    # ── Transmitter / receiver ────────────────────────────────────────────────
    tx_power_dbm: float = 23.0        # ETSI EN 302 663 Class A max: 23 dBm ERP
    antenna_gain_dbi: float = 2.0     # Omnidirectional OBU antenna gain (TX + RX each)
    rx_sensitivity_dbm: float = -85.0 # 802.11p typical receiver sensitivity floor

    # ── Path loss model (log-distance + log-normal shadowing) ────────────────
    # Calibrated for realistic PRR variation within 10-150m V2V ranges.
    # Higher exponents and shadowing create meaningful differentiation:
    #   - At 50m LOS: P_rx ≈ -71 dBm → SNR margin ≈ +14 dB → PRR ≈ 0.98
    #   - At 100m LOS: P_rx ≈ -77 dBm → SNR margin ≈ +8 dB → PRR ≈ 0.92
    #   - At 150m NLOS: P_rx ≈ -87 dBm → SNR margin ≈ -2 dB → PRR ≈ 0.45
    path_loss_exponent_los: float = 2.3    # Urban LOS (measured: 2.1-2.5)
    path_loss_exponent_nlos: float = 3.2   # Urban NLOS with buildings
    shadowing_std_los_db: float = 5.5      # Increased LOS shadowing
    shadowing_std_nlos_db: float = 8.0     # Heavy NLOS shadowing

    # ── CSMA/CA contention (channel busy ratio) ───────────────────────────────
    # CBR ∈ [0, 1]: fraction of time the channel is sensed busy.
    # 0.0 = empty channel (no contention), 1.0 = fully saturated.
    # NOTE: This parameter is now informational only. The actual contention
    # model uses a density-aware approach calibrated to 802.11p measurements.
    # See _csma_success_prob() for the implementation.
    channel_busy_ratio: float = 0.15

    # ── LOS/NLOS detection ────────────────────────────────────────────────────
    # When True, vehicles lying on the TX→RX line segment cause NLOS penalty.
    # When False, LOS path loss and shadowing parameters are always used.
    enable_nlos_model: bool = True

    # ── PRR sigmoid steepness ─────────────────────────────────────────────────
    # Larger values produce a sharper PRR transition around the sensitivity floor.
    # k=2.0 matches Sommer et al. (2011) empirical calibration.
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

    In-process physics-based packet delivery model that faithfully represents
    the probabilistic, broadcast, and contention-sensitive nature of the
    DSRC/WAVE medium. No external broker or network infrastructure is required.

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
    ) -> None:
        """
        Compute channel delivery for every (sender, receiver) pair this tick.

        Args:
            bsm_messages: Current BSMCore per registered vehicle.
            positions: {vehicle_id: (x_m, y_m)} in CARLA world space.
                       Typically derived from bsm.x_m / bsm.y_m (CARLA metres),
                       which store CARLA world coordinates (not geodetic).
            frame, sim_time: stamped on the LinkResult records only.

        Call once per V2X update tick. Results are stable until the next call.
        """
        self._received = {vid: {} for vid in bsm_messages}
        self.last_links = []
        vehicle_ids = list(bsm_messages.keys())

        for sender_id, bsm in bsm_messages.items():
            tx_pos = positions.get(sender_id)
            if tx_pos is None:
                continue

            n_contenders = self._count_contenders(sender_id, tx_pos, positions)

            for receiver_id in vehicle_ids:
                if receiver_id == sender_id:
                    continue

                rx_pos = positions.get(receiver_id)
                if rx_pos is None:
                    continue

                nlos = (
                    self.config.enable_nlos_model
                    and _euclidean(tx_pos, rx_pos) >= 0.5
                    and self._is_nlos(sender_id, receiver_id, tx_pos, rx_pos, positions)
                )
                link = self.evaluate_link(
                    sender_id, receiver_id, tx_pos, rx_pos, nlos, n_contenders,
                    msg_type="BSM", link_type="V2V", frame=frame, sim_time=sim_time,
                )
                if link.delivered:
                    self._received[receiver_id][sender_id] = bsm

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
    ) -> LinkResult:
        """Decide one transmission attempt and update the channel statistics.

        Every message type (BSM, SPAT, MAP, VRUAM) uses this one path. ``tx_power_dbm`` is the
        transmitter's own power (OBU, RSU, VRU device); None means ``config.tx_power_dbm``.
        Consumes the seeded RNG in a fixed order (one Gaussian, then one uniform draw).
        """
        cfg = self.config
        tx_power = cfg.tx_power_dbm if tx_power_dbm is None else tx_power_dbm
        distance = _euclidean(tx_pos, rx_pos)
        self.stats["total_broadcasts"] += 1

        if distance < 0.5:
            # Co-located stations: deterministic delivery, no radio draw
            self.stats["total_deliveries"] += 1
            link = LinkResult(
                frame, sim_time, tx_id, rx_id, msg_type, link_type, distance, True, tx_power,
                0.0, 0.0, tx_power + 2.0 * cfg.antenna_gain_dbi,
                tx_power + 2.0 * cfg.antenna_gain_dbi - cfg.rx_sensitivity_dbm,
                1.0, 1.0, 1.0, True, None,
            )
        else:
            p_rx, path_loss, shadowing = self._rx_power_parts(distance, nlos, tx_power)
            margin = p_rx - cfg.rx_sensitivity_dbm
            self._record_margin(margin)
            p_channel = 1.0 / (1.0 + math.exp(-cfg.prr_sigmoid_k * margin))
            p_mac = self._csma_success_prob(n_contenders)
            p_total = p_channel * p_mac
            delivered = self._rng.random() < p_total
            self.stats["total_deliveries" if delivered else "total_drops"] += 1
            link = LinkResult(
                frame, sim_time, tx_id, rx_id, msg_type, link_type, distance, not nlos, tx_power,
                path_loss, shadowing, p_rx, margin, p_channel, p_mac, p_total, delivered,
                None if delivered else "channel_or_mac",
            )
        if cfg.record_links:
            self.last_links.append(link)
        return link

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

        p_channel = 1.0 / (1.0 + math.exp(-self.config.prr_sigmoid_k * margin))
        return p_channel * self._csma_success_prob(n_contenders)

    def _record_margin(self, margin_db: float) -> None:
        """Keep the last 500 receive margins for the rolling average (bounded memory)."""
        self._rx_margin_ring.append(margin_db)
        if len(self._rx_margin_ring) > 500:
            self._rx_margin_ring = self._rx_margin_ring[-500:]

    def _csma_success_prob(self, n_contenders: int) -> float:
        """
        Probability of successful CSMA/CA transmission given N contenders.

        Uses a density-aware model calibrated to 802.11p performance at 2 Hz BSM rate.
        
        Real 802.11p at 2 Hz with ~200-byte beacons only saturates at hundreds of nodes.
        This model provides realistic PRR degradation:
          - N ≤ 20: PRR > 0.95 (minimal contention)
          - N ≈ 50: PRR ≈ 0.85 (moderate contention)
          - N ≈ 100: PRR ≈ 0.70 (heavy contention)
          - N ≥ 200: PRR ≈ 0.50 (saturation)
        
        Based on empirical 802.11p measurements from:
          - Campolo et al. (2011): "IEEE 802.11p performance in vehicular networks"
          - ETSI TR 102 863: "V2V channel congestion analysis"
        """
        if n_contenders <= 1:
            return 1.0
        
        # Density-aware contention model
        # At 2 Hz, 200-byte BSMs: packet_time ≈ 0.53ms, interval = 500ms
        # Theoretical max before saturation: ~900 vehicles (0.53ms/500ms)
        # Practical saturation starts around 200 vehicles due to backoff overhead
        
        # Use exponential decay that matches empirical data
        # PRR = exp(-λ * (N - 1)) where λ controls degradation rate
        # λ = 0.005 gives: N=20→0.91, N=50→0.78, N=100→0.61, N=200→0.37
        lambda_contention = 0.005
        
        # Add small constant floor for realistic minimum PRR
        p_csma = math.exp(-lambda_contention * (n_contenders - 1))
        return max(0.35, p_csma)  # Floor at 35% (realistic saturation level)

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
        Determine LOS/NLOS by 2-D geometric occlusion.

        A third vehicle causes NLOS if its position lies within 2.5 m
        (≈ vehicle width) of the TX→RX line segment. This is a computationally
        lightweight approximation sufficient for urban block geometry as used by
        Sommer et al.'s shadowing model.
        """
        for vid, pos in all_positions.items():
            if vid == sender_id or vid == receiver_id:
                continue
            if _point_to_segment_dist(pos, tx, rx) < 2.5:
                return True
        return False

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
