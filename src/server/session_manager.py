"""
Client session tracking for the persistent CARLA live server.

Each device that connects (laptop, raspi, workstation…) gets one session,
and one "leading vehicle" in the simulation.
"""

import secrets
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
    # Secret that identifies the session on the WebSocket hub and in LiDAR requests. It is NOT the session id:
    # the id is shown in the admin list, the token only goes to the client that joined (never in a URL).
    session_token: str = field(default_factory=lambda: secrets.token_urlsafe(24), repr=False)
    ws_connections: int = 0
    ended_at: Optional[datetime] = None

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
            "connected": self.ws_connections > 0,
            "ended_at": self.ended_at.isoformat() if self.ended_at else None,
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
            if session.is_active:
                session.ended_at = datetime.now(timezone.utc)
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

    def get_by_token(self, token: str) -> Optional[ClientSession]:
        """The active session owning ``token`` (constant-time comparison), else None."""
        with self._lock:
            sessions = list(self._sessions.values())
        found = None
        for s in sessions:
            if secrets.compare_digest(s.session_token, token) and s.is_active:
                found = s
        return found

    def mark_connected(self, session_id: str) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                session.ws_connections += 1
                session.last_seen = datetime.now(timezone.utc)

    def mark_disconnected(self, session_id: str) -> None:
        """A WebSocket of the session closed. The session is kept for the grace period (decision D4)."""
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                session.ws_connections = max(0, session.ws_connections - 1)
                session.last_seen = datetime.now(timezone.utc)

    def prune_ended(self, older_than_s: float = 600.0) -> int:
        """Forget sessions that ended more than ``older_than_s`` ago (they are listed for a while, then dropped)."""
        cutoff = datetime.now(timezone.utc) - timedelta(seconds=older_than_s)
        with self._lock:
            stale = [k for k, s in self._sessions.items() if not s.is_active and s.ended_at and s.ended_at < cutoff]
            for k in stale:
                del self._sessions[k]
        return len(stale)

    def get_grace_expired(self, grace_s: float) -> List[ClientSession]:
        """Active sessions with no WebSocket and no sign of life (``touch``) for longer than ``grace_s``."""
        now = datetime.now(timezone.utc)
        with self._lock:
            return [s for s in self._sessions.values()
                    if s.is_active and s.ws_connections == 0 and (now - s.last_seen).total_seconds() > grace_s]

    def touch(self, session_id: str) -> None:
        """Update last_seen timestamp (called on every WS heartbeat)."""
        with self._lock:
            session = self._sessions.get(session_id)
        if session:
            session.last_seen = datetime.now(timezone.utc)
