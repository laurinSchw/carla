"""
Client session tracking for the persistent CARLA live server.

Each device that connects (laptop, raspi, workstation…) gets one session,
and one "leading vehicle" in the simulation.
"""

import uuid
import logging
import threading
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Optional, Dict, List

logger = logging.getLogger(__name__)


@dataclass
class ClientSession:
    """State for a single connected client and their leading vehicle."""

    session_id: str
    client_name: str
    client_ip: str
    vehicle_blueprint: str
    spawn_time: datetime
    expires_at: Optional[datetime]         # None = run until the client disconnects
    vehicle_actor_id: Optional[int] = None # CARLA actor ID (set after spawn)
    is_active: bool = True
    lidar_enabled: bool = False
    last_seen: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    # ------------------------------------------------------------------ #
    # Computed properties                                                  #
    # ------------------------------------------------------------------ #

    @property
    def is_expired(self) -> bool:
        if self.expires_at is None:
            return False
        return datetime.now(timezone.utc) > self.expires_at

    @property
    def remaining_seconds(self) -> Optional[float]:
        if self.expires_at is None:
            return None
        return max(0.0, (self.expires_at - datetime.now(timezone.utc)).total_seconds())

    def to_dict(self) -> dict:
        return {
            "session_id": self.session_id,
            "client_name": self.client_name,
            "client_ip": self.client_ip,
            "vehicle_blueprint": self.vehicle_blueprint,
            "spawn_time": self.spawn_time.isoformat(),
            "expires_at": self.expires_at.isoformat() if self.expires_at else None,
            "vehicle_actor_id": self.vehicle_actor_id,
            "is_active": self.is_active,
            "lidar_enabled": self.lidar_enabled,
            "remaining_seconds": self.remaining_seconds,
            "last_seen": self.last_seen.isoformat(),
        }


class SessionManager:
    """
    Thread-safe registry of active client sessions.

    The persistent world holds a single instance and shares it with the
    HTTP/WebSocket routes so they can look up sessions without acquiring
    CARLA-level locks.
    """

    def __init__(self) -> None:
        self._sessions: Dict[str, ClientSession] = {}
        self._lock = threading.RLock()

    # ------------------------------------------------------------------ #
    # CRUD                                                                 #
    # ------------------------------------------------------------------ #

    def create_session(
        self,
        client_name: str,
        client_ip: str,
        vehicle_blueprint: str,
        duration_seconds: Optional[float],
    ) -> ClientSession:
        session_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc)
        expires_at = now + timedelta(seconds=duration_seconds) if duration_seconds else None

        session = ClientSession(
            session_id=session_id,
            client_name=client_name,
            client_ip=client_ip,
            vehicle_blueprint=vehicle_blueprint,
            spawn_time=now,
            expires_at=expires_at,
        )
        with self._lock:
            self._sessions[session_id] = session
        logger.info("Session created: %s for '%s' from %s", session_id, client_name, client_ip)
        return session

    def get_session(self, session_id: str) -> Optional[ClientSession]:
        with self._lock:
            return self._sessions.get(session_id)

    def end_session(self, session_id: str) -> bool:
        with self._lock:
            session = self._sessions.get(session_id)
            if not session:
                return False
            session.is_active = False
        logger.info("Session ended: %s (%s)", session_id, session.client_name)
        return True

    def remove_session(self, session_id: str) -> bool:
        with self._lock:
            return self._sessions.pop(session_id, None) is not None

    # ------------------------------------------------------------------ #
    # Queries                                                              #
    # ------------------------------------------------------------------ #

    def get_active_sessions(self) -> List[ClientSession]:
        with self._lock:
            return [s for s in self._sessions.values() if s.is_active]

    def get_expired_sessions(self) -> List[ClientSession]:
        with self._lock:
            return [s for s in self._sessions.values() if s.is_active and s.is_expired]

    def all_sessions(self) -> List[ClientSession]:
        with self._lock:
            return list(self._sessions.values())

    def touch(self, session_id: str) -> None:
        """Update last_seen timestamp (called on every WS heartbeat)."""
        with self._lock:
            session = self._sessions.get(session_id)
        if session:
            session.last_seen = datetime.now(timezone.utc)
