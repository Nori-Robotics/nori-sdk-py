"""nori-sdk — Python operator client for Nori robots.

Speaks nori-protocol over a WebRTC data channel: the same wire dialect as the TypeScript
@nori/sdk the web app uses, and the same one the robot's gateway implements. Three
implementations, one protocol — see README "Staying in sync" for how they are kept honest.

Layered so you only pay for what you use, mirroring the TS package's subpath exports:

    nori_sdk.protocol / .types / .motion   zero dependencies — build and parse every frame,
                                           bring your own transport
    nori_sdk.teleop                        the live session          [needs: aiortc]
    nori_sdk.signaling_supabase            the reference transport   [needs: websocket-client]
    nori_sdk.mock                          hardware-free test doubles

The heavy imports are lazy: `import nori_sdk` never pulls in aiortc or websocket-client, so a
frame-parsing consumer stays dependency-free. A missing extra surfaces as a plain ImportError
naming the install command, raised at the point where the dependency is actually required —
`RemoteTeleop.start()` and `SupabaseSignaling(...)` — not at import. Constructing a
RemoteTeleop and feeding it frames deliberately works without any of it, which is what keeps
the test suite hardware-free.

SAFETY: no message any client can send makes a robot unsafe. Clamping, the watchdog, E-STOP
and the torque lifecycle all live on the robot, which defends itself against every client.
The corollary matters for scripts: a jog stream that STOPS is a stop command, because the
robot treats silence as an absent operator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from . import motion, protocol, types
from .auth import AuthError, DeviceAuth, UserAuth
from .signaling import (
    IcePayload,
    NackPayload,
    ReadyTurn,
    SdpPayload,
    SignalingHandlers,
    SignalingState,
    SignalingTransport,
)
from .types import (
    RECOVERY_ERROR_CODES,
    TERMINAL_ACTION_STATES,
    ActionStatus,
    CameraLayout,
    ConnectStatus,
    DaemonStatus,
    Perception,
    PolicyStreamStatus,
    RecordState,
    RobotDescriptor,
    RobotError,
    RobotInfo,
    Telemetry,
    WatchdogProfile,
)
from .version import NORI_PROTOCOL_VERSION

__version__ = "1.0.0"

if TYPE_CHECKING:  # import for type checkers only; runtime stays lazy
    from .signaling_supabase import SupabaseSignaling
    from .teleop import RemoteTeleop, TeleopError

_LAZY = {
    "RemoteTeleop": (".teleop", "webrtc"),
    "TeleopError": (".teleop", "webrtc"),
    "SupabaseSignaling": (".signaling_supabase", "supabase"),
}


def __getattr__(name: str) -> Any:
    """Resolve the optional-extra symbols on first touch, and turn a missing dependency into
    a message that names the install command rather than an aiortc traceback."""
    target = _LAZY.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, extra = target
    import importlib

    try:
        module = importlib.import_module(module_name, __name__)
    except ImportError as e:
        raise ImportError(
            f'{name} needs the "{extra}" extra: pip install "nori-sdk[{extra}]" ({e})'
        ) from e
    return getattr(module, name)


def __dir__() -> list[str]:
    """Include the lazily-resolved names.

    Without this, Python's default `dir()` reports only what has already been imported, so
    RemoteTeleop — the class this package exists to provide — was missing from tab-completion,
    `help(nori_sdk)` and every IDE's introspection until something touched it first. A symbol
    you cannot discover is, for most users, a symbol that does not exist."""
    return sorted(set(globals()) | set(_LAZY))


__all__ = [
    "NORI_PROTOCOL_VERSION",
    "RECOVERY_ERROR_CODES",
    "TERMINAL_ACTION_STATES",
    "ActionStatus",
    "AuthError",
    "CameraLayout",
    "ConnectStatus",
    "DaemonStatus",
    "DeviceAuth",
    "IcePayload",
    "NackPayload",
    "Perception",
    "PolicyStreamStatus",
    "ReadyTurn",
    "RecordState",
    "RemoteTeleop",
    "RobotDescriptor",
    "RobotError",
    "RobotInfo",
    "SdpPayload",
    "SignalingHandlers",
    "SignalingState",
    "SignalingTransport",
    "SupabaseSignaling",
    "Telemetry",
    "TeleopError",
    "UserAuth",
    "WatchdogProfile",
    "__version__",
    "motion",
    "protocol",
    "types",
]
