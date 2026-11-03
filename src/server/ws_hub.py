"""Multiplexed JSON WebSocket ``/ws`` (REFACTOR_HANDOFF 5.7.3).

Client -> server (JSON text):
    {"op": "auth", "admin_token": "..."}  or  {"op": "auth", "session_token": "..."}
    {"op": "subscribe", "topic": "v2x.links", "max_hz": 2, "filter": {...}}
    {"op": "unsubscribe", "topic": "..."}
    {"op": "ping"}
Server -> client:
    {"topic": "<name>", "frame": 12, "sim_time": 0.6, "seq": 7, "data": {...}}
    {"topic": "error", "code": "forbidden", "message": "...", "request": {"op": ..., "topic": ...}}
    control replies on topics "auth", "subscribed", "unsubscribed", "pong".

Without an auth message a connection is an *observer* (if the server allows public observers). Credentials are
accepted only in the message, never in the URL.

Delivery is *latest wins per (connection, topic)*: a connection's sender builds the newest published state only
when its previous send has completed, so a slow browser skips updates (``seq`` jumps tell it how many) and never
holds anything back. The simulation thread does not know the hub exists: providers only read published snapshots.
A send that does not complete within ``send_timeout_s`` closes the connection.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from src.server.roles import OBSERVER, AuthError, Principal, authenticate
from src.server.topics import TOPICS, FilterError, ProviderContext, TopicSpec

logger = logging.getLogger(__name__)

MAX_MESSAGE_BYTES = 16 * 1024
POLL_STEP_S = 0.02            # how soon a subscription that found nothing new looks again
IDLE_WAIT_S = 0.25


@dataclass
class Subscription:
    spec: TopicSpec
    filter: Dict[str, Any]
    max_hz: float
    last_sent: float = 0.0
    next_poll: float = 0.0
    last_version: Optional[int] = None
    state: Dict[str, Any] = field(default_factory=dict)

    @property
    def interval(self) -> float:
        return 1.0 / self.max_hz


class Hub:
    """Shared by all connections: the data source, the topic table and a per-frame cache."""

    def __init__(self, source: Any, topics: Optional[Dict[str, TopicSpec]] = None, *, send_timeout_s: float = 10.0,
                 clock=time.monotonic):
        self.source = source
        self.topics = TOPICS if topics is None else topics
        self.send_timeout_s = send_timeout_s
        self.clock = clock
        self.cache: Dict[Any, Any] = {}
        self.connections = 0
        self.conns: set = set()                    # live Connection objects (for the admin health card and tests)
        self.bytes_sent = 0
        self.messages_sent = 0

    def _new_round(self) -> None:
        """Bound the shared cache (its keys carry the frame, so an entry is never wrong, only old)."""
        if len(self.cache) > 256:
            self.cache.clear()

    async def serve(self, ws) -> None:
        await ws.accept()
        conn = Connection(self, ws)
        self.connections += 1
        self.conns.add(conn)
        try:
            await conn.run()
        finally:
            self.connections -= 1
            self.conns.discard(conn)
            conn.on_close()

    def describe(self) -> Dict[str, Any]:
        """Who is connected and what they subscribed to (ground truth of the permission model, for admins)."""
        return {"connections": [c.info() for c in list(self.conns)], "messages_sent": self.messages_sent,
                "bytes_sent": self.bytes_sent}


class Connection:
    def __init__(self, hub: Hub, ws):
        self.hub, self.ws = hub, ws
        self.principal: Principal = OBSERVER
        self.subs: Dict[str, Subscription] = {}
        self.wake = asyncio.Event()
        self.closing = False
        self._session_id: Optional[str] = None       # session whose connection count we hold
        self.connected_at = time.time()
        self.sent_messages = 0
        self.sent_bytes = 0

    def info(self) -> Dict[str, Any]:
        return {"role": self.principal.kind, "client_name": self.principal.label,
                "topics": {name: {"max_hz": s.max_hz, "filter": s.filter} for name, s in self.subs.items()},
                "connected_s": round(time.time() - self.connected_at, 1), "messages_sent": self.sent_messages,
                "bytes_sent": self.sent_bytes}

    # ── output ──────────────────────────────────────────────────────────────────────────────────
    async def _send(self, obj: Dict[str, Any]) -> None:
        try:
            text = json.dumps(obj, allow_nan=False, separators=(",", ":"))
        except ValueError as exc:                     # NaN / inf must never reach a browser
            logger.error("topic %s produced non-JSON data: %s", obj.get("topic"), exc)
            text = json.dumps({"topic": "error", "code": "internal", "message": "server produced invalid data for "
                               f"{obj.get('topic')}"}, separators=(",", ":"))
        await asyncio.wait_for(self.ws.send_text(text), timeout=self.hub.send_timeout_s)
        self.sent_messages += 1
        self.sent_bytes += len(text)
        self.hub.messages_sent += 1
        self.hub.bytes_sent += len(text)

    async def _error(self, code: str, message: str, request: Optional[Dict[str, Any]] = None) -> None:
        await self._send({"topic": "error", "code": code, "message": message, "request": request or {}})

    # ── input ───────────────────────────────────────────────────────────────────────────────────
    async def _receive_loop(self) -> None:
        while True:
            raw = await self.ws.receive_text()
            if len(raw) > MAX_MESSAGE_BYTES:
                await self._error("bad_message", f"message larger than {MAX_MESSAGE_BYTES} bytes")
                continue
            try:
                msg = json.loads(raw)
            except ValueError:
                await self._error("bad_message", "not JSON")
                continue
            if not isinstance(msg, dict) or not isinstance(msg.get("op"), str):
                await self._error("bad_message", 'expected {"op": ...}')
                continue
            await self._handle(msg)

    async def _handle(self, msg: Dict[str, Any]) -> None:
        op = msg["op"]
        req = {"op": op, "topic": msg.get("topic")}
        if op == "ping":
            await self._send({"topic": "pong", "data": {"t": time.time()}})
        elif op == "auth":
            await self._auth(msg)
        elif op == "subscribe":
            await self._subscribe(msg, req)
        elif op == "unsubscribe":
            self.subs.pop(str(msg.get("topic")), None)
            await self._send({"topic": "unsubscribed", "data": {"topic": msg.get("topic")}})
        else:
            await self._error("bad_request", f"unknown op {op!r}", req)

    async def _auth(self, msg: Dict[str, Any]) -> None:
        try:
            principal = authenticate(self.hub.source.sessions, msg.get("admin_token"), msg.get("session_token"))
        except AuthError as exc:
            await self._error(exc.code, exc.message, {"op": "auth"})
            self.closing = True
            await self.ws.close(code=4401)
            return
        if principal.kind == "observer":
            await self._error("bad_request", "auth needs admin_token or session_token", {"op": "auth"})
            return
        self._release_session()
        self.principal = principal
        if principal.kind == "vehicle" and principal.session_id:
            self.hub.source.sessions.mark_connected(principal.session_id)
            self._session_id = principal.session_id
        # subscriptions made before auth are re-checked against the new role
        for name in list(self.subs):
            try:
                self._check_access(self.subs[name].spec, self.subs[name].filter)
            except FilterError as exc:
                del self.subs[name]
                await self._error(exc.code, f"subscription {name} dropped: {exc.message}", {"topic": name})
        await self._send({"topic": "auth", "data": {"role": principal.kind, "client_name": principal.label,
                                                    "station_id": principal.actor_id if principal.kind == "vehicle" else None}})

    def _check_access(self, spec: TopicSpec, filt: Dict[str, Any]) -> Dict[str, Any]:
        """Role check, then the topic's own filter validation (which also enforces a required filter)."""
        p = self.principal
        if p.kind == "observer" and not getattr(self.hub.source, "observer_public", True):
            raise FilterError("forbidden", "observer access is disabled on this server; authenticate first")
        if p.kind not in spec.roles:
            raise FilterError("forbidden", f"role {p.kind!r} may not subscribe to {spec.name}")
        return spec.validate_filter(filt, p) if spec.validate_filter is not None else filt

    async def _subscribe(self, msg: Dict[str, Any], req: Dict[str, Any]) -> None:
        name = msg.get("topic")
        spec = self.hub.topics.get(name) if isinstance(name, str) else None
        if spec is None:
            await self._error("unknown_topic", f"unknown topic {name!r}", req)
            return
        filt = msg.get("filter", {})
        if filt is None:
            filt = {}
        if not isinstance(filt, dict):
            await self._error("bad_filter", "filter must be an object", req)
            return
        hz = msg.get("max_hz", spec.default_hz)
        if not isinstance(hz, (int, float)) or isinstance(hz, bool) or not hz > 0:
            await self._error("bad_request", "max_hz must be a positive number", req)
            return
        try:
            filt = self._check_access(spec, filt)
        except FilterError as exc:
            await self._error(exc.code, exc.message, req)
            return
        hz = min(float(hz), spec.max_hz)
        self.subs[spec.name] = Subscription(spec, filt, hz)
        await self._send({"topic": "subscribed", "data": {"topic": spec.name, "max_hz": hz, "filter": filt,
                                                          "role": self.principal.kind}})
        self.wake.set()

    # ── sender ──────────────────────────────────────────────────────────────────────────────────
    async def _send_loop(self) -> None:
        hub = self.hub
        while not self.closing:
            now = hub.clock()
            hub._new_round()
            soonest = now + IDLE_WAIT_S
            for sub in list(self.subs.values()):
                due = max(sub.last_sent + sub.interval, sub.next_poll)
                if now < due:
                    soonest = min(soonest, due)
                    continue
                ctx = ProviderContext(hub.source, self.principal, sub.filter, sub.state, hub.cache)
                try:
                    pub = sub.spec.provide(ctx)
                except Exception:
                    logger.exception("topic %s provider failed", sub.spec.name)
                    sub.next_poll = now + 1.0
                    await self._error("internal", f"{sub.spec.name} is temporarily unavailable", {"topic": sub.spec.name})
                    continue
                if pub is None or pub.version == sub.last_version:
                    sub.next_poll = now + min(POLL_STEP_S, sub.interval)
                    soonest = min(soonest, sub.next_poll)
                    continue
                await self._send({"topic": sub.spec.name, "frame": pub.frame, "sim_time": pub.sim_time,
                                  "seq": pub.version, "data": pub.data})
                sent_at = hub.clock()
                sub.last_sent, sub.last_version = sent_at, pub.version
                sub.next_poll = 0.0
            wait = max(0.001, soonest - hub.clock())
            try:
                await asyncio.wait_for(self.wake.wait(), timeout=wait)
            except asyncio.TimeoutError:
                pass
            self.wake.clear()

    async def run(self) -> None:
        receiver = asyncio.create_task(self._receive_loop())
        sender = asyncio.create_task(self._send_loop())
        try:
            done, _pending = await asyncio.wait({receiver, sender}, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                exc = task.exception()
                if isinstance(exc, asyncio.TimeoutError):
                    logger.warning("closing a slow WebSocket client (send timed out)")
                    try:
                        await self.ws.close(code=1013)
                    except Exception:
                        pass
                elif exc is not None and not _is_disconnect(exc):
                    logger.error("WebSocket hub error: %r", exc)
        finally:
            self.closing = True
            for task in (receiver, sender):
                task.cancel()
            await asyncio.gather(receiver, sender, return_exceptions=True)

    def _release_session(self) -> None:
        if self._session_id is not None:
            self.hub.source.sessions.mark_disconnected(self._session_id)
            self._session_id = None

    def on_close(self) -> None:
        self._release_session()


def _is_disconnect(exc: BaseException) -> bool:
    name = type(exc).__name__
    return name in ("WebSocketDisconnect", "ConnectionClosed", "ConnectionClosedOK", "ConnectionClosedError", "CancelledError") \
        or (isinstance(exc, RuntimeError) and "close message" in str(exc))
