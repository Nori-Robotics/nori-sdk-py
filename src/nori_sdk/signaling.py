"""The signaling transport contract — the small out-of-band pipe carrying SDP/ICE and the
room handshake BEFORE the peer connection exists.

Ported from @nori/sdk's src/signaling.ts, and it exists for the same reason: the surface
actually used is tiny (a broadcast room with a handful of named events), so hiding it behind
an interface keeps the SDK from being welded to Nori's cloud. Ship the Supabase adapter
(signaling_supabase.py) or bring your own — a plain WebSocket, an MQTT topic, even manual
copy/paste — without touching RemoteTeleop.

ROLE: the operator is always the ANSWERER. The robot offers; we answer. That asymmetry is
protocol, not implementation detail — a transport that reverses it will not connect.

SAFETY: this carries signaling only. The robot defends itself regardless (clamp / watchdog /
E-STOP / torque lifecycle are all robot-side), so a buggy or hostile transport can at worst
fail to connect — it can never move a robot unsafely.
"""

from __future__ import annotations

import abc
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from typing import Any, Literal

# Transport health, as distinct from robot health. "open" = the room is live and we can
# speak; "error"/"timeout" = we cannot reach the signaling service at all. Without this
# distinction a caller cannot tell "my internet is broken" from "my robot is off" — both
# look like an unbounded silent wait.
SignalingState = Literal["open", "error", "timeout", "closed"]


@dataclass(frozen=True)
class SdpPayload:
    type: Literal["offer", "answer"]
    sdp: str


@dataclass(frozen=True)
class IcePayload:
    candidate: str
    sdp_mline_index: int | None = None
    sdp_mid: str | None = None

    def to_wire(self) -> dict[str, Any]:
        # The wire key is the browser's camelCase name — the robot and the TS SDK both
        # expect `sdpMLineIndex`. Do not "fix" this to snake_case.
        return {"candidate": self.candidate, "sdpMLineIndex": self.sdp_mline_index}

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> IcePayload | None:
        cand = obj.get("candidate")
        if not isinstance(cand, str) or not cand:
            return None
        idx = obj.get("sdpMLineIndex")
        return cls(
            candidate=cand,
            sdp_mline_index=idx if isinstance(idx, int) and not isinstance(idx, bool) else None,
            sdp_mid=obj.get("sdpMid") if isinstance(obj.get("sdpMid"), str) else None,
        )


@dataclass(frozen=True)
class ReadyTurn:
    """Session TURN credentials forwarded to the robot inside `ready`.

    The robot cannot hold static relay creds (coturn runs use-auth-secret, so creds are
    short-lived and backend-minted per session), and this is its ONLY way to gather relay
    candidates. Without it a robot behind a symmetric NAT is unreachable even though our own
    relay path is fine. Rides the RLS-gated private room, i.e. the same trust level as the
    SDP that follows. Robots predating the field ignore it."""

    urls: list[str] = field(default_factory=list)
    username: str = ""
    credential: str = ""

    def to_wire(self) -> dict[str, Any]:
        return {"urls": list(self.urls), "username": self.username, "credential": self.credential}


@dataclass(frozen=True)
class NackPayload:
    """The robot refused our `ready`. TRUST: room membership is enforced by the robot (via
    Supabase RLS), but a nack itself can be forged by anyone already in the room. It is a
    HINT used only to pick better error copy — never a security decision."""

    reason: str | None = None


@dataclass
class SignalingHandlers:
    """Inbound events RemoteTeleop reacts to. Registered once, by connect().

    Handlers may be sync or async; a transport must accept either (see `dispatch`)."""

    # The robot published an SDP OFFER. We build a fresh peer and answer. Fires again on
    # every robot restart, so it must be idempotent.
    on_sdp: Callable[[SdpPayload], Any]
    # A remote ICE candidate from the robot.
    on_ice: Callable[[IcePayload], Any]
    # The robot (re)joined the room — re-announce `ready` so it (re)offers. May fire
    # repeatedly.
    on_robot_here: Callable[[dict[str, Any]], Any]
    # The transport is live — safe to announce `ready`. May fire more than once across
    # reconnects; RemoteTeleop is idempotent on it.
    on_open: Callable[[], Any]
    # Optional: the robot rejected our `ready`. Robots older than 2026-07-12 never send it,
    # so its ABSENCE must never be read as success.
    on_nack: Callable[[NackPayload], Any] | None = None
    # Optional: every transport state change, including failures on_open can't express.
    on_state: Callable[[SignalingState], Any] | None = None


class SignalingTransport(abc.ABC):
    """The operator side of the signaling room. One instance per teleop session;
    RemoteTeleop owns its lifecycle (connect on start, close on stop)."""

    @abc.abstractmethod
    async def connect(self, handlers: SignalingHandlers) -> None:
        """Join the room and wire the inbound handlers. Returns once wiring is registered —
        NOT once the room is open. Readiness is signalled via handlers.on_open."""

    @abc.abstractmethod
    def send_ready(self, turn: ReadyTurn | None = None) -> None:
        """Announce that the operator is present and wants an offer."""

    @abc.abstractmethod
    def send_sdp(self, payload: SdpPayload) -> None:
        """Publish our SDP ANSWER back to the robot."""

    @abc.abstractmethod
    def send_ice(self, payload: IcePayload) -> None:
        """Publish one local ICE candidate."""

    @abc.abstractmethod
    def send_bye(self) -> None:
        """Best-effort "operator leaving" so the robot restarts cleanly. MUST NOT raise."""

    @abc.abstractmethod
    async def close(self) -> None:
        """Tear down the room subscription. Idempotent."""


async def dispatch(handler: Callable[..., Any] | None, *args: Any) -> None:
    """Call a handler that may be sync or async, swallowing nothing but awaiting when
    needed. Transports use this so users can register either flavor."""
    if handler is None:
        return
    result = handler(*args)
    if isinstance(result, Awaitable):
        await result


__all__ = [
    "IcePayload",
    "NackPayload",
    "ReadyTurn",
    "SdpPayload",
    "SignalingHandlers",
    "SignalingState",
    "SignalingTransport",
    "dispatch",
]
