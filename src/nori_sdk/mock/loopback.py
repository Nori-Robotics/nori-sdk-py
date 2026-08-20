"""In-process signaling: two transports wired to each other, no network.

Lets a test exercise the REAL handshake path — the same on_open -> ready -> robot_here ->
offer -> answer sequence RemoteTeleop runs against Supabase — with deterministic ordering
and no credentials. Pair it with aiortc on both ends for a full loopback session, or with
MockRobot for protocol-only tests.
"""

from __future__ import annotations

import asyncio
from typing import Any

from ..signaling import (
    IcePayload,
    NackPayload,
    ReadyTurn,
    SdpPayload,
    SignalingHandlers,
    SignalingState,
    SignalingTransport,
    dispatch,
)


class LoopbackSignaling(SignalingTransport):
    """One end of a loopback pair. Whatever this end sends is delivered to the peer's
    handlers on the next event-loop turn — asynchronously, deliberately, so a test can't
    accidentally depend on synchronous reentrancy that a real transport would never give."""

    def __init__(self, name: str = "operator") -> None:
        self.name = name
        self.peer: LoopbackSignaling | None = None
        self.handlers: SignalingHandlers | None = None
        self.sent: list[tuple[str, Any]] = []
        self._closed = False
        self._open_on_connect = True

    async def connect(self, handlers: SignalingHandlers) -> None:
        self.handlers = handlers
        if self._open_on_connect:
            # A real transport signals readiness after the room subscribes, never inside
            # connect(); schedule it so ordering matches.
            asyncio.get_running_loop().call_soon(lambda: self._spawn(dispatch(handlers.on_open)))

    def send_ready(self, turn: ReadyTurn | None = None) -> None:
        self._deliver("ready", {"turn": turn.to_wire() if turn else None})

    def send_sdp(self, payload: SdpPayload) -> None:
        self._deliver("sdp", payload)

    def send_ice(self, payload: IcePayload) -> None:
        self._deliver("ice", payload)

    def send_bye(self) -> None:
        try:
            self._deliver("bye", {})
        except Exception:
            pass

    async def close(self) -> None:
        self._closed = True
        self._notify_state("closed")

    # --- test-facing helpers -----------------------------------------------------------------

    def announce_robot(self, payload: dict[str, Any] | None = None) -> None:
        """Simulate the robot joining the room."""
        self._to_peer(lambda h: dispatch(h.on_robot_here, payload or {}))

    def nack(self, reason: str) -> None:
        self._to_peer(lambda h: dispatch(h.on_nack, NackPayload(reason=reason)))

    def fail(self, state: SignalingState = "error") -> None:
        """Simulate losing the signaling service."""
        self._notify_state(state)

    # --- internals ---------------------------------------------------------------------------

    def _deliver(self, event: str, payload: Any) -> None:
        if self._closed:
            return
        self.sent.append((event, payload))
        if event == "sdp":
            self._to_peer(lambda h: dispatch(h.on_sdp, payload))
        elif event == "ice":
            self._to_peer(lambda h: dispatch(h.on_ice, payload))
        elif event == "ready":
            # `ready` has no handler on the SignalingHandlers contract (only the robot cares),
            # so a test reads it from peer.sent.
            pass

    def _to_peer(self, make: Any) -> None:
        peer = self.peer
        if peer is None or peer.handlers is None or peer._closed:
            return
        peer._spawn(make(peer.handlers))

    def _notify_state(self, state: str) -> None:
        if self.handlers is not None and self.handlers.on_state is not None:
            self._spawn(dispatch(self.handlers.on_state, state))

    def _spawn(self, coro: Any) -> None:
        try:
            asyncio.ensure_future(coro)
        except RuntimeError:
            pass  # no running loop (test teardown)


def loopback_pair() -> tuple[LoopbackSignaling, LoopbackSignaling]:
    """(operator, robot) transports wired to each other."""
    operator, robot = LoopbackSignaling("operator"), LoopbackSignaling("robot")
    operator.peer, robot.peer = robot, operator
    return operator, robot


__all__ = ["LoopbackSignaling", "loopback_pair"]
