"""Wire types: the robot -> operator half of nori-protocol, as dataclasses.

Every class here is a pure, dependency-free parser for one inbound frame kind. They are
deliberately TOLERANT: a field the robot omits becomes None/default rather than an error,
because the fleet runs mixed firmware and a newer robot may add keys this SDK has never
heard of. The rule, in both SDKs, is "ignore what you don't understand" — never raise on an
unknown or missing field, only on structurally invalid JSON.

These mirror the TypeScript interfaces in @nori/sdk (src/teleop.ts). Where the wire uses
snake_case the dataclass field keeps snake_case; the TS side camel-cases at its boundary and
that difference is cosmetic, not protocol.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from .version import NORI_PROTOCOL_VERSION

# The daemon's externally visible safety states. Kept as a plain str (not an enum) on the
# dataclasses so a newer robot's unknown value renders instead of crashing.
#   "ok"        — normal
#   "safe_hold" — motion refused, no latch: the robot is protecting itself (thermal hold, or
#                 control-frame silence past the watchdog stop threshold). Self-clears.
#   "latched"   — E-STOP latched. Motion blocked until command("reset_latch").
SafetyState = Literal["ok", "safe_hold", "latched"]

ArmSide = Literal["left", "right"]
ControlMode = Literal["cylindrical", "joint"]
LinkMode = Literal["lan", "wan"]

# The `action_status` states that end an action. "accepted" and "active" are explicitly NOT
# here: they are progress reports. See ActionStatus.done.
TERMINAL_ACTION_STATES = frozenset({"done", "blocked", "clamped", "timeout"})


def _f(obj: dict[str, Any], key: str) -> float | None:
    v = obj.get(key)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _s(obj: dict[str, Any], key: str) -> str | None:
    v = obj.get(key)
    return v if isinstance(v, str) else None


def _i(obj: dict[str, Any], key: str, default: int = 0) -> int:
    v = obj.get(key)
    return v if isinstance(v, int) and not isinstance(v, bool) else default


def _num_map(obj: dict[str, Any], key: str) -> dict[str, float]:
    v = obj.get(key)
    if not isinstance(v, dict):
        return {}
    return {
        k: float(x)
        for k, x in v.items()
        if isinstance(x, (int, float)) and not isinstance(x, bool)
    }


# --- handshake ---------------------------------------------------------------------------


@dataclass(frozen=True)
class WatchdogProfile:
    """How long the robot tolerates control-frame silence. The robot picks the profile from
    the link mode we report (LAN 150/500 ms vs WAN 300/1000 ms) — see RemoteTeleop.send_link.
    A client streaming jog must resend inside t_warn_ms or the robot ramps to a stop."""

    t_warn_ms: float
    t_stop_ms: float

    @classmethod
    def from_wire(cls, obj: Any) -> WatchdogProfile | None:
        if not isinstance(obj, dict):
            return None
        warn, stop = _f(obj, "t_warn_ms"), _f(obj, "t_stop_ms")
        if warn is None or stop is None:
            return None
        return cls(t_warn_ms=warn, t_stop_ms=stop)


@dataclass(frozen=True)
class RobotDescriptor:
    """What the robot physically is, straight from the robot. This is what makes the SDK
    model-agnostic: never hard-code a joint list, read it from here.

    `ranges` is the authoritative [min, max] per "<name>.pos" key. Values outside it are
    CLAMPED robot-side (never rejected), so use it to scale your own inputs rather than to
    pre-validate."""

    buses: list[str] = field(default_factory=list)
    joints: list[str] = field(default_factory=list)  # every drivable "<motor>.pos" key
    base: list[str] = field(default_factory=list)  # base DOFs ("x.vel", "theta.vel")
    aux: list[str] = field(default_factory=list)  # extra actuators (e.g. "left_lift")
    cameras: list[str] = field(default_factory=list)  # roles; match the CameraLayout tiles
    ranges: dict[str, tuple[float, float]] = field(default_factory=dict)

    @classmethod
    def from_wire(cls, obj: Any) -> RobotDescriptor | None:
        if not isinstance(obj, dict):
            return None

        def strs(key: str) -> list[str]:
            v = obj.get(key)
            return [x for x in v if isinstance(x, str)] if isinstance(v, list) else []

        ranges: dict[str, tuple[float, float]] = {}
        raw_ranges = obj.get("ranges")
        if isinstance(raw_ranges, dict):
            for k, pair in raw_ranges.items():
                if isinstance(pair, (list, tuple)) and len(pair) == 2:
                    lo, hi = pair
                    if isinstance(lo, (int, float)) and isinstance(hi, (int, float)):
                        ranges[k] = (float(lo), float(hi))
        return cls(
            buses=strs("buses"),
            joints=strs("joints"),
            base=strs("base"),
            aux=strs("aux"),
            cameras=strs("cameras"),
            ranges=ranges,
        )


@dataclass(frozen=True)
class RobotInfo:
    """The parsed `ack` handshake.

    accepted=False means the robot refused the session (`error` says why) — the connection
    stays up but control frames are ignored. version_mismatch is advisory only."""

    accepted: bool = True
    protocol_version: int | None = None
    norm_mode: str | None = None  # "range_m100_100" | "degrees" — units of all .pos values
    watchdog_profile: WatchdogProfile | None = None
    descriptor: RobotDescriptor | None = None
    initial_state: dict[str, float] = field(default_factory=dict)
    error: str | None = None
    version_mismatch: bool = False

    # ADVISORY ONLY: a label for logs, dataset provenance and bug reports ("L2", "A3"). Never
    # branch behaviour on it — branch on `descriptor` and `capabilities`, so a model this SDK
    # has never heard of works against an unmodified client. Absent on older robots.
    model: str | None = None

    # None means the robot did not tell us — NOT "supports nothing". The distinction is load-
    # bearing: an empty list is a robot explicitly declaring no optional verbs, while None is
    # a robot predating the field, which must be probed or assumed legacy. Collapsing the two
    # would make every legacy robot look like it supports nothing at all.
    capabilities: list[str] | None = None

    @classmethod
    def from_wire(
        cls, obj: dict[str, Any], sdk_protocol_version: int = NORI_PROTOCOL_VERSION
    ) -> RobotInfo:
        # Tolerant of old robots that send a bare {"type":"ack"}: an absent `accepted`
        # counts as accepted, everything else is optional.
        pv = obj.get("protocol_version")
        protocol_version = int(pv) if isinstance(pv, int) and not isinstance(pv, bool) else None
        raw_caps = obj.get("capabilities")
        capabilities = (
            [c for c in raw_caps if isinstance(c, str)] if isinstance(raw_caps, list) else None
        )
        return cls(
            accepted=obj.get("accepted") is not False,
            protocol_version=protocol_version,
            norm_mode=_s(obj, "norm_mode"),
            watchdog_profile=WatchdogProfile.from_wire(obj.get("watchdog_profile")),
            descriptor=RobotDescriptor.from_wire(obj.get("descriptor")),
            initial_state=_num_map(obj, "initial_state"),
            error=_s(obj, "error"),
            version_mismatch=(
                protocol_version is not None and protocol_version != sdk_protocol_version
            ),
            model=_s(obj, "model"),
            capabilities=capabilities,
        )

    def supports(self, capability: str) -> bool | None:
        """Does this robot honour an optional verb? True / False / None for "it didn't say".

        Three-valued on purpose. A robot predating the capabilities field reports None, and a
        caller must decide whether to probe or assume legacy — folding that into False would
        silently disable working features on every older robot in the fleet. An unrecognised
        capability name is not an error; treat it as one this SDK does not model.

        Known values: task_jog, leader_action_deg, lift_targets, record, policy_stream, call,
        perception. The list is a UNION built along the path — a daemon lists what it honours
        and a bridge appends what it adds on top — so never infer WHICH component provides a
        capability from the fact that it is present."""
        if self.capabilities is None:
            return None
        return capability in self.capabilities


# --- periodic state ----------------------------------------------------------------------


@dataclass(frozen=True)
class Telemetry:
    """One `telemetry` frame (~15 Hz). `state` is the lerobot-native dict: every
    "<motor>.pos" plus base "x.vel"/"theta.vel".

    The robot sends the `status` block at a lower rate than the frame itself, so a frame
    without it leaves the previous safety reading standing — RemoteTeleop merges that for
    you and hands out the merged view; this dataclass is the raw frame."""

    ts_ns: int = 0
    state: dict[str, float] = field(default_factory=dict)
    safety: str | None = None
    watchdog: str | None = None
    latch_reason: str | None = None
    motor_faults: dict[str, str] = field(default_factory=dict)
    servo_temps: dict[str, float] = field(default_factory=dict)
    currents: dict[str, float] = field(default_factory=dict)
    loop_hz: float | None = None
    pi_temp_c: float | None = None

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> Telemetry:
        status = obj.get("status") if isinstance(obj.get("status"), dict) else {}
        faults = status.get("motor_faults") if isinstance(status, dict) else None
        temp = _f(obj, "pi_temp_c")
        ts = obj.get("ts_ns")
        return cls(
            ts_ns=int(ts) if isinstance(ts, int) and not isinstance(ts, bool) else 0,
            state=_num_map(obj, "state"),
            safety=_s(status, "safety") if isinstance(status, dict) else None,
            watchdog=_s(status, "watchdog") if isinstance(status, dict) else None,
            latch_reason=_s(status, "latch_reason") if isinstance(status, dict) else None,
            motor_faults=(
                {k: str(v) for k, v in faults.items()} if isinstance(faults, dict) else {}
            ),
            servo_temps=_num_map(obj, "servo_temps"),
            currents=_num_map(obj, "currents"),
            loop_hz=_f(obj, "loop_hz"),
            # A 0 reading means "no sensor", not "freezing" — drop it like the TS SDK does.
            pi_temp_c=temp if temp and temp > 0 else None,
        )


@dataclass(frozen=True)
class CameraLayout:
    """The composite video tiling. One H.264 track carries every camera as tiles of a grid;
    this says how to slice it. Sent repeatedly on connect because the control channel is
    unreliable — treat repeats as idempotent.

    Single-camera robots send NO layout at all; absence means "the whole frame is the one
    camera"."""

    cols: int
    rows: int
    tiles: list[str] = field(default_factory=list)

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> CameraLayout | None:
        """None means REJECT this frame and keep whatever layout you already had.

        A layout with no tiles is invalid per the schema, and adopting one would blank the
        grid for the rest of the session — the robot repeats the layout several times on
        open, so a single malformed repeat must not be able to poison a good one. Note this
        is the opposite of an ABSENT layout, which is how a single-camera robot says "the
        whole frame is the one camera"."""
        cols, rows = obj.get("cols"), obj.get("rows")
        if not isinstance(cols, int) or not isinstance(rows, int) or cols < 1 or rows < 1:
            return None
        raw_tiles = obj.get("tiles")
        if not isinstance(raw_tiles, list):
            return None
        tiles = [t for t in raw_tiles if isinstance(t, str)]
        if not tiles:
            return None
        return cls(cols=cols, rows=rows, tiles=tiles)

    def rect(self, role: str) -> tuple[float, float, float, float] | None:
        """Normalized (x, y, w, h) of one camera's tile within the composite frame, or None
        if this robot has no such role. Multiply by the decoded frame size to crop."""
        if role not in self.tiles:
            return None
        i = self.tiles.index(role)
        w, h = 1.0 / self.cols, 1.0 / self.rows
        return ((i % self.cols) * w, (i // self.cols) * h, w, h)


@dataclass(frozen=True)
class DaemonStatus:
    """Robot-side motion health, as distinct from transport health. "offline" with reason
    "unauthorized" means the motion stack refused this gateway authority; "unreachable"
    means motion is disabled or the daemon is down. Control frames are dropped while
    offline — check this before blaming your jog values."""

    state: str  # "online" | "offline"
    reason: str | None = None
    detail: str | None = None
    robot_local_mic_muted: bool | None = None

    @property
    def online(self) -> bool:
        return self.state == "online"

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> DaemonStatus | None:
        """None means DROP THIS FRAME and keep the health you already had.

        A frame with a missing or empty `state` used to be coerced to "offline", which
        invents an outage: the bridge rebroadcasts while offline, so one malformed repeat
        would flip a healthy robot to offline in every UI and log watching this. The spec
        makes the drop mandatory. Callers must handle None — that is the whole point of the
        signal, and the reason this returns an optional at all."""
        state = _s(obj, "state")
        if not state:
            return None
        mic = obj.get("robot_local_mic_muted")
        return cls(
            state=state,
            reason=_s(obj, "reason"),
            detail=_s(obj, "detail"),
            robot_local_mic_muted=mic if isinstance(mic, bool) else None,
        )


# --- request/response ---------------------------------------------------------------------


@dataclass(frozen=True)
class ActionStatus:
    """Reply to an absolute-pose `action` that carried an action_id.

    The lifecycle is accepted -> active -> done | blocked | clamped | timeout. Only those
    last four are TERMINAL: "accepted" means the robot received and validated the target, not
    that anything moved, and "active" means it is still moving. Treating "accepted" as the
    end is how a client returns success before the motion has happened and never sees the
    real verdict — which is exactly what this SDK used to do.

      done     settled within tolerance
      clamped  the target saturated a range or soft limit; the robot moved somewhere else
      blocked  refused — `reason` names it, e.g. "stall:<joint>", "estop:button". A stall is
               soft: torque drops on that one joint and clears when you jog off the obstruction
      timeout  no terminal state within the robot's own per-action deadline
    """

    action_id: str
    state: str
    reason: str | None = None
    ts_ns: int = 0

    @property
    def done(self) -> bool:
        """True once the robot has finished with this action, however it ended.

        An unrecognised state counts as NOT done, deliberately: a caller then falls through
        to its own timeout instead of being told a move completed that this SDK cannot
        actually interpret."""
        return self.state in TERMINAL_ACTION_STATES

    @property
    def succeeded(self) -> bool:
        """Terminal AND the robot reached the target. `clamped` is a completed move to a
        DIFFERENT pose than you asked for, so it is done but not successful."""
        return self.state == "done"

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> ActionStatus:
        ts = obj.get("ts_ns")
        return cls(
            action_id=_s(obj, "action_id") or "",
            state=_s(obj, "state") or "",
            reason=_s(obj, "reason"),
            ts_ns=int(ts) if isinstance(ts, int) and not isinstance(ts, bool) else 0,
        )


# `error` codes that are NOT failures: the robot telling you a previous fault has cleared.
# Surprising enough to be worth a constant -- an "error" frame saying arm_recovered arriving
# as a red banner is a bug that ships easily.
RECOVERY_ERROR_CODES = frozenset(
    {"obstruction_cleared", "arm_recovered", "motor_recovered"}
)


@dataclass(frozen=True)
class RobotError:
    """An `error` frame. `fatal=True` ends the session; everything else is a notice.

    `code` is an OPEN set — newer robots add codes freely, so never switch exhaustively on it
    and never gate display on recognising it. Show `msg`, which is written for an operator.

    Not every error is a failure. `obstruction` is a soft stall: torque drops on one joint and
    clears when you jog away from whatever it hit. And three codes are recoveries — see
    RECOVERY_ERROR_CODES — so a client that renders every `error` frame as a fault will show a
    red banner for the news that the fault went away."""

    code: str = ""
    msg: str = ""
    fatal: bool = False
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def recovered(self) -> bool:
        """This frame reports a fault CLEARING, not a new one."""
        return self.code in RECOVERY_ERROR_CODES

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> RobotError:
        return cls(
            code=_s(obj, "code") or "",
            msg=_s(obj, "msg") or "",
            # Absent means false, per the schema default.
            fatal=obj.get("fatal") is True,
            raw=dict(obj),
        )


@dataclass(frozen=True)
class RecordState:
    """Reply to every `record` verb. `episode` is the robot's DISPLAY id ("episode-0001");
    the on-disk UUID is internal and never crosses the wire."""

    ok: bool = False
    recording: bool = False
    session_open: bool = False
    episodes_kept: int = 0
    episode: str | None = None
    free_gb: float = 0.0
    error: str | None = None

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> RecordState:
        kept = obj.get("episodes_kept")
        return cls(
            ok=bool(obj.get("ok")),
            recording=bool(obj.get("recording")),
            session_open=bool(obj.get("session_open")),
            episodes_kept=int(kept) if isinstance(kept, int) and not isinstance(kept, bool) else 0,
            episode=_s(obj, "episode"),
            free_gb=_f(obj, "free_gb") or 0.0,
            error=_s(obj, "error"),
        )


@dataclass(frozen=True)
class PolicyStreamStatus:
    """Reply to `policy_stream` verbs (the robot's always-on policy streamer).

    `ok` and `streaming` default to FALSE when absent, which is the opposite of this SDK's
    usual tolerance and is deliberate: a partial or malformed reply must not read as success.

    LIVENESS: there is no unsolicited death notification. The L2 streamer is a ZMQ REP socket
    — it can only answer a request — and its end-of-run result is discarded rather than
    pushed. A stream that dies mid-run (sink timeout, camera silence) is observable ONLY by
    POLLING with action "status". Do not wait for a failure frame that never arrives."""

    ok: bool = False
    streaming: bool = False
    dest: str | None = None
    fps_out: float | None = None
    frames_sent: int = 0
    dropped: int = 0
    error: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> PolicyStreamStatus:
        return cls(
            ok=obj.get("ok") is True,
            streaming=obj.get("streaming") is True,
            dest=_s(obj, "dest"),
            fps_out=_f(obj, "fps_out"),
            frames_sent=_i(obj, "frames_sent"),
            dropped=_i(obj, "dropped"),
            error=_s(obj, "error"),
            raw=dict(obj),
        )


@dataclass(frozen=True)
class Perception:
    """A `perception` frame: what the robot's vision stack currently believes is in front of
    it. Shape is robot-defined and evolving, so the raw dict is preserved verbatim
    alongside the fields this SDK version knows."""

    ts_ns: int = 0
    objects: list[dict[str, Any]] = field(default_factory=list)
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> Perception:
        objs = obj.get("objects")
        ts = obj.get("ts_ns")
        return cls(
            ts_ns=int(ts) if isinstance(ts, int) and not isinstance(ts, bool) else 0,
            objects=[o for o in objs if isinstance(o, dict)] if isinstance(objs, list) else [],
            raw=dict(obj),
        )


# --- session-level state (SDK-side, not a wire frame) --------------------------------------

ConnectPhase = Literal[
    "idle", "joining", "waiting", "negotiating", "connected", "failed", "closed"
]


@dataclass(frozen=True)
class ConnectStatus:
    """Where the connection attempt actually is, and why it stopped if it did. Failures name
    a cause so a script can tell "my network is broken" from "the robot is off":

      signaling_unreachable — can't reach the signaling room at all
      robot_absent          — room is live, robot never announced itself
      session_rejected      — the robot refused us (see detail)
      negotiation_failed    — SDP/ICE exchange broke
      ice_failed            — no working network path (NAT/firewall/TURN)
    """

    phase: ConnectPhase = "idle"
    failure: str | None = None
    detail: str | None = None


__all__ = [
    "RECOVERY_ERROR_CODES",
    "TERMINAL_ACTION_STATES",
    "ActionStatus",
    "ArmSide",
    "CameraLayout",
    "ConnectPhase",
    "ConnectStatus",
    "ControlMode",
    "DaemonStatus",
    "LinkMode",
    "Perception",
    "PolicyStreamStatus",
    "RecordState",
    "RobotDescriptor",
    "RobotError",
    "RobotInfo",
    "SafetyState",
    "Telemetry",
    "WatchdogProfile",
]
