"""Who is asking, and what each role may see (REFACTOR_HANDOFF 5.7.1).

Three roles: *admin* (``ADMIN_TOKEN``), *vehicle* (the ``session_token`` returned by join) and *observer* (nobody
in particular; read-only, only when ``observer_public``). Tokens are accepted in the first WebSocket message or
in a request header, never in a URL.

The topic permission table lives in ``topics.py``; this module only turns credentials into a ``Principal``.
"""

from __future__ import annotations

import os
import secrets
from typing import Optional

from dataclasses import dataclass


@dataclass(frozen=True)
class Principal:
    """Admin: any actor. Vehicle: only the actor of its own session. Observer: read-only."""
    kind: str                         # "admin" | "vehicle" | "observer"
    session_id: Optional[str] = None
    actor_id: Optional[int] = None
    label: Optional[str] = None       # shown as the owner (client name); the session id is credential-adjacent and is
                                      # never put in anything an observer can read

    @property
    def owner_label(self) -> str:
        return "admin" if self.kind == "admin" else (self.label or self.kind)


ADMIN = Principal("admin")
OBSERVER = Principal("observer")


class AuthError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code, self.message = code, message


def admin_token_ok(token: str) -> bool:
    """True if ``token`` equals ADMIN_TOKEN (constant-time). AuthError if the server has none configured."""
    expected = os.environ.get("ADMIN_TOKEN", "")
    if not expected:
        raise AuthError("admin_not_configured", "ADMIN_TOKEN is not configured on the server")
    return secrets.compare_digest(token.encode(), expected.encode())


def authenticate(sessions, admin_token: Optional[str] = None, session_token: Optional[str] = None) -> Principal:
    """Credentials -> Principal. No credential: observer. A wrong credential is an error, never a downgrade."""
    if admin_token is not None:
        if not isinstance(admin_token, str) or not admin_token_ok(admin_token):
            raise AuthError("auth_failed", "invalid admin token")
        return ADMIN
    if session_token is not None:
        session = sessions.get_by_token(session_token) if isinstance(session_token, str) else None
        if session is None:
            raise AuthError("auth_failed", "unknown or ended session")
        return Principal("vehicle", session.session_id, session.vehicle_actor_id, session.client_name)
    return OBSERVER
