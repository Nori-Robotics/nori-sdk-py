"""Wire types: the robot -> operator half of nori-protocol, as dataclasses.

Every class here is a pure, dependency-free parser for one inbound frame kind. They are
deliberately TOLERANT: a field the robot omits becomes None/default rather than an error,
because the fleet runs mixed firmware and a newer robot may add keys this SDK has never
heard of. The rule, in both SDKs, is "ignore what you don't understand" — never raise on an
unknown or missing field, only on structurally invalid JSON.

These mirror the TypeScript interfaces in @nori/sdk (src/teleop.ts). Where the wire uses
snake_case the dataclass field keeps snake_case; the TS side camel-cases at its boundary and
that difference is cosmetic, not protocol.

REACHING FIELDS THIS SDK DOES NOT MODEL. Three classes carry a `raw` dict — Perception,
PolicyStreamStatus and RobotError — and the others do not. That is a rule, not an oversight:
`raw` is for frames whose shape is robot-defined and still evolving, where a caller is
expected to read something this version has never heard of. For the rest, the escape hatch is
`protocol.decode()`, which always returns the untouched dict as its third element.

The gap worth knowing: `RemoteTeleop.on()` and `.stream()` hand you the PARSED object, so a
field this SDK does not model is not reachable through the session API. If a robot starts
sending a telemetry key you need before this SDK models it, read it off the channel with
`protocol.decode()` rather than waiting for a release.

A "parses to None" return is never a parse failure to paper over — it is a decision the spec
requires. DaemonStatus.from_wire and CameraLayout.from_wire both use it to mean DROP THIS
FRAME AND KEEP WHAT YOU HAD, because in both cases adopting the malformed frame would
manufacture a state the robot never reported.
"""

from __future__ import annotations

import math
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
# The wire value for task-space control stays "cylindrical" for compatibility; UIs display
# it as "cartesian" on robots that advertise descriptor.jog_scale.task (verbs x/y/z/pitch/
# yaw, with "shoulder_pan" as the deprecated alias of "yaw"). Only the label changed.
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


def _fd(obj: dict[str, Any], key: str, default: float = 0.0) -> float:
    v = _f(obj, key)
    return default if v is None else v


def _ob(obj: dict[str, Any], key: str) -> bool | None:
    v = obj.get(key)
    return v if isinstance(v, bool) else None


def _oi(obj: dict[str, Any], key: str) -> int | None:
    v = obj.get(key)
    return v if isinstance(v, int) and not isinstance(v, bool) else None


def _opt_nums(
    obj: dict[str, Any], key: str, length: int | None = None
) -> tuple[float | None, ...]:
    """A ROS numeric array as floats, with every non-finite reading as None.

    JSON has no NaN/Infinity, but Python's json.loads accepts both, and a ROS scan uses them
    for "no reading". None is therefore a MEASUREMENT GAP, never a distance -- collapsing it
    to 0.0 would invent an obstacle at the sensor origin. When `length` is given the result is
    padded/truncated to it, so a fixed-size covariance is always indexable."""
    raw = obj.get(key)
    items = raw if isinstance(raw, list) else []
    out: list[float | None] = []
    for item in items:
        if isinstance(item, (int, float)) and not isinstance(item, bool):
            value = float(item)
            out.append(value if math.isfinite(value) else None)
        else:
            out.append(None)
    if length is not None:
        out = out[:length] + [None] * max(0, length - len(out))
    return tuple(out)


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
class JogScale:
    """The multiplier from a normalized jog rate to real motion, per namespace.

    NOMINAL COMMANDED scale, not achieved velocity. Three things routinely make the real rate
    lower and a caller must expect all three: the watchdog's `warn` state scales all motion to
    ZERO until control frames resume (watch `telemetry.watchdog`), an acceleration limit means
    a short jog never reaches the rate, and stacks running MoveIt Servo scale it down near
    singularities and collisions.

    UNITS ARE PER NAMESPACE, each matching the thing it addresses so you can verify what you
    got against telemetry:

        joints  norm_mode units/s  — matches telemetry `state` and descriptor `ranges`
        lift    mm/s               — matches the mm of `<side>_lift.pos` targets
        task    m/s (x, y, z), rad/s (pitch, yaw). "yaw" is the canonical angular-z verb;
                "shoulder_pan" is its DEPRECATED alias and may appear alongside it.
                L2 descriptors never carry `task` at all.
        base    m/s (linear), rad/s (angular)

    Joints are deliberately NOT rad/s: telemetry reports normalized positions, so a rad/s rate
    could not be checked against anything you can see. Expect per-joint values to differ even
    where the robot uses one internal constant — the normalized-to-physical scale is per joint.

    An empty mapping means the robot advertised that namespace but named no keys; a MISSING
    key means unknown. Neither ever means zero."""

    joints: dict[str, float] = field(default_factory=dict)
    task: dict[str, float] = field(default_factory=dict)
    base: dict[str, float] = field(default_factory=dict)
    lift: float | None = None

    @classmethod
    def from_wire(cls, obj: Any) -> JogScale | None:
        if not isinstance(obj, dict):
            return None

        def rates(key: str) -> dict[str, float]:
            v = obj.get(key)
            if not isinstance(v, dict):
                return {}
            # A non-positive rate is invalid per the schema: "cannot be jogged" is expressed
            # by omitting the key, so a 0 here is a broken robot, not a slow one. Dropping it
            # keeps "missing means unknown" true rather than handing back a zero that would
            # silently scale every command to nothing.
            return {
                k: float(x)
                for k, x in v.items()
                if isinstance(x, (int, float)) and not isinstance(x, bool) and x > 0
            }

        lift = obj.get("lift")
        return cls(
            joints=rates("joints"),
            task=rates("task"),
            base=rates("base"),
            lift=(
                float(lift)
                if isinstance(lift, (int, float)) and not isinstance(lift, bool) and lift > 0
                else None
            ),
        )


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

    # The same bounds as `ranges`, in SI units (radians for revolute, metres for prismatic).
    # Empty when the robot does not publish them -- the L-series fleet never will. These are
    # the robot's own CALIBRATED bounds, not nominal URDF limits, which is what makes a
    # normalized->radian conversion exact rather than approximate. See motion.to_si().
    #
    # Only keys whose `ranges` entry is NORMALIZED appear here. The A-series lift is absent on
    # purpose: ranges["lift.pos"] is already millimetres, so an entry would mean converting a
    # value that is already physical.
    ranges_si: dict[str, tuple[float, float]] = field(default_factory=dict)

    # How a normalized jog rate converts to real motion on THIS robot. None means the robot
    # did not advertise it — the L2 fleet is frozen and never will, so None is the common
    # case. See motion.jog_rate()/normalized_for(). NOT achieved velocity: the watchdog's
    # warn state scales motion to zero, acceleration limits mean short jogs never reach the
    # rate, and Servo scales near singularities.
    jog_scale: JogScale | None = None

    @classmethod
    def from_wire(cls, obj: Any) -> RobotDescriptor | None:
        if not isinstance(obj, dict):
            return None

        def strs(key: str) -> list[str]:
            v = obj.get(key)
            return [x for x in v if isinstance(x, str)] if isinstance(v, list) else []

        def spans(key: str) -> dict[str, tuple[float, float]]:
            out: dict[str, tuple[float, float]] = {}
            raw = obj.get(key)
            if not isinstance(raw, dict):
                return out
            for k, pair in raw.items():
                if isinstance(pair, (list, tuple)) and len(pair) == 2:
                    lo, hi = pair
                    if isinstance(lo, (int, float)) and isinstance(hi, (int, float)):
                        # NOT sorted: an inverted span means the calibration reverses that
                        # axis, and normalizing the order away would flip the joint.
                        out[k] = (float(lo), float(hi))
            return out

        ranges = spans("ranges")
        return cls(
            buses=strs("buses"),
            joints=strs("joints"),
            base=strs("base"),
            aux=strs("aux"),
            cameras=strs("cameras"),
            ranges=ranges,
            ranges_si=spans("ranges_si"),
            jog_scale=JogScale.from_wire(obj.get("jog_scale")),
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

        Known values: task_jog, pose_targets, leader_action_deg, lift_targets, record,
        policy_stream, call, perception. The list is a UNION built along the path — a daemon
        lists what it honours
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


# --- named navigation ------------------------------------------------------------------------

# The robot's navigation lifecycle. `unavailable` means the robot cannot navigate at all right
# now (no map, not localized, Nav2 down, software E-stop unknown or active) -- distinct from
# `failed`, which is a goal that was attempted and did not finish.
NavigationState = Literal[
    "idle",
    "starting",
    "navigating",
    "canceling",
    "succeeded",
    "canceled",
    "aborted",
    "failed",
    "unavailable",
]

# States a goal never leaves. await_navigation() resolves on exactly these.
TERMINAL_NAVIGATION_STATES: frozenset[str] = frozenset(
    {"succeeded", "canceled", "aborted", "failed", "unavailable"}
)


@dataclass(frozen=True)
class WaypointSummary:
    """One saved destination on the robot's active map."""

    name: str = ""
    saved_at_unix: float = 0.0


@dataclass(frozen=True)
class NavigationStatus:
    """A `navigation_status` frame: either the correlated reply to one request, or an
    unsolicited lifecycle snapshot.

    Snapshots are SELF-CONTAINED and keyed by `goal_id`, so loss, duplication and reordering
    are all tolerable -- act on the latest one for a goal rather than accumulating deltas.
    `request_id` is present on a direct reply and absent on an unsolicited update.

    Unlike the TypeScript SDK this class is never synthesized by the client: when the robot
    does not answer, this SDK raises `RobotUnreachable` instead of returning a status, so a
    transport failure can never be misread as the robot reporting that it stopped."""

    ok: bool = False
    # A plain string at runtime: a newer robot's unfamiliar state renders as itself rather
    # than being forced onto one this build knows. Test `terminal`, not equality, to ask
    # whether a goal is over.
    state: NavigationState = "unavailable"
    active: bool = False
    request_id: str | None = None
    goal_id: str | None = None
    name: str | None = None
    map_sha256: str | None = None
    distance_remaining_m: float | None = None
    estimated_time_remaining_s: float | None = None
    number_of_recoveries: int | None = None
    error_code: int | None = None
    error: str | None = None
    replaced: bool | None = None
    deleted: bool | None = None
    waypoints: tuple[WaypointSummary, ...] | None = None

    @property
    def terminal(self) -> bool:
        """True once this goal can no longer change state."""
        return self.state in TERMINAL_NAVIGATION_STATES

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> NavigationStatus:
        # A state this build has never heard of is kept VERBATIM, like SafetyState — never
        # coerced onto a known one. Coercing to "failed" would make it terminal, and
        # await_navigation() would then report a finished goal while the robot drove on: the
        # unknown state is exactly the case where we must not claim the robot stopped.
        # Terminality is membership in the known terminal set, so an unrecognised state is
        # simply not terminal, which is the safe default and needs no invention.
        state: NavigationState = _s(obj, "state") or "unavailable"  # type: ignore[assignment]
        waypoints: tuple[WaypointSummary, ...] | None = None
        listed = obj.get("waypoints")
        if isinstance(listed, list):
            found: list[WaypointSummary] = []
            for item in listed:
                if not isinstance(item, dict):
                    continue
                name = _s(item, "name")
                saved = _f(item, "saved_at_unix")
                if name and saved is not None:
                    found.append(WaypointSummary(name=name, saved_at_unix=saved))
            waypoints = tuple(found)
        return cls(
            ok=obj.get("ok") is True,
            state=state,
            active=obj.get("active") is True,
            request_id=_s(obj, "request_id"),
            goal_id=_s(obj, "goal_id"),
            name=_s(obj, "name") or None,
            map_sha256=_s(obj, "map_sha256") or None,
            distance_remaining_m=_f(obj, "distance_remaining_m"),
            estimated_time_remaining_s=_f(obj, "estimated_time_remaining_s"),
            number_of_recoveries=_oi(obj, "number_of_recoveries"),
            error_code=_oi(obj, "error_code"),
            error=_s(obj, "error") or None,
            replaced=_ob(obj, "replaced"),
            deleted=_ob(obj, "deleted"),
            waypoints=waypoints,
        )


# --- LiDAR and IMU ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RosStamp:
    """A ROS header stamp, preserved as-is so a caller can align frames against other ROS
    data. It is the ROBOT's clock, not the client's -- do not compare it to local time."""

    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_wire(cls, obj: Any) -> RosStamp:
        stamp = obj if isinstance(obj, dict) else {}
        return cls(sec=_i(stamp, "sec"), nanosec=_i(stamp, "nanosec"))


@dataclass(frozen=True)
class SensorStreamStatus:
    """Reply to a `sensor_stream` request: the EFFECTIVE settings after the robot clamped
    them, plus whether ROS currently sees a publisher on each topic.

    `lidar_available` / `imu_available` are a point-in-time publisher check. Receipt of a
    fresh sample is the authoritative liveness signal -- a topic can have a publisher that
    has stopped producing."""

    ok: bool = False
    request_id: str = ""
    lidar_hz: float = 0.0
    imu_hz: float = 0.0
    lidar_max_points: int = 360
    lidar_available: bool = False
    imu_available: bool = False
    error: str | None = None

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> SensorStreamStatus:
        return cls(
            ok=obj.get("ok") is True,
            request_id=_s(obj, "request_id") or "",
            lidar_hz=_fd(obj, "lidar_hz"),
            imu_hz=_fd(obj, "imu_hz"),
            lidar_max_points=_i(obj, "lidar_max_points", 360),
            lidar_available=obj.get("lidar_available") is True,
            imu_available=obj.get("imu_available") is True,
            error=_s(obj, "error") or None,
        )


@dataclass(frozen=True)
class LidarScan:
    """One sampled frame of the filtered `/scan` topic.

    The angle for `ranges_m[i]` is `angle_min_rad + i * angle_increment_rad`, where the
    increment ALREADY accounts for the robot's sampling stride -- do not re-derive it from
    `source_points`. `source_points` is how many readings the source scan had before
    sampling, so `len(ranges_m) < source_points` means you asked for fewer points, not that
    readings were lost.

    A `None` in `ranges_m` is a non-finite ROS reading (no return), NOT a distance."""

    stamp: RosStamp = field(default_factory=RosStamp)
    frame_id: str = ""
    angle_min_rad: float = 0.0
    angle_max_rad: float = 0.0
    angle_increment_rad: float = 0.0
    time_increment_s: float = 0.0
    scan_time_s: float = 0.0
    range_min_m: float = 0.0
    range_max_m: float = 0.0
    source_points: int = 0
    ranges_m: tuple[float | None, ...] = ()
    intensities: tuple[float | None, ...] | None = None

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> LidarScan:
        ranges = _opt_nums(obj, "ranges_m")
        return cls(
            stamp=RosStamp.from_wire(obj.get("stamp")),
            frame_id=_s(obj, "frame_id") or "",
            angle_min_rad=_fd(obj, "angle_min_rad"),
            angle_max_rad=_fd(obj, "angle_max_rad"),
            angle_increment_rad=_fd(obj, "angle_increment_rad"),
            time_increment_s=_fd(obj, "time_increment_s"),
            scan_time_s=_fd(obj, "scan_time_s"),
            range_min_m=_fd(obj, "range_min_m"),
            range_max_m=_fd(obj, "range_max_m"),
            source_points=_i(obj, "source_points", len(ranges)),
            ranges_m=ranges,
            intensities=(
                _opt_nums(obj, "intensities")
                if isinstance(obj.get("intensities"), list)
                else None
            ),
        )


@dataclass(frozen=True)
class ImuSample:
    """One `/imu/data` sample, with ROS covariance arrays preserved.

    Every vector keeps its ROS length (4 for the quaternion, 3 for the vectors, 9 for each
    covariance) so indexing is always safe; a missing or non-finite component is None. ROS
    signals "this quantity is not provided" with a leading covariance of -1."""

    stamp: RosStamp = field(default_factory=RosStamp)
    frame_id: str = ""
    orientation_xyzw: tuple[float | None, ...] = (None, None, None, None)
    orientation_covariance: tuple[float | None, ...] = (None,) * 9
    angular_velocity_rad_s: tuple[float | None, ...] = (None, None, None)
    angular_velocity_covariance: tuple[float | None, ...] = (None,) * 9
    linear_acceleration_m_s2: tuple[float | None, ...] = (None, None, None)
    linear_acceleration_covariance: tuple[float | None, ...] = (None,) * 9

    @classmethod
    def from_wire(cls, obj: dict[str, Any]) -> ImuSample:
        return cls(
            stamp=RosStamp.from_wire(obj.get("stamp")),
            frame_id=_s(obj, "frame_id") or "",
            orientation_xyzw=_opt_nums(obj, "orientation_xyzw", 4),
            orientation_covariance=_opt_nums(obj, "orientation_covariance", 9),
            angular_velocity_rad_s=_opt_nums(obj, "angular_velocity_rad_s", 3),
            angular_velocity_covariance=_opt_nums(obj, "angular_velocity_covariance", 9),
            linear_acceleration_m_s2=_opt_nums(obj, "linear_acceleration_m_s2", 3),
            linear_acceleration_covariance=_opt_nums(obj, "linear_acceleration_covariance", 9),
        )


__all__ = [
    "RECOVERY_ERROR_CODES",
    "TERMINAL_ACTION_STATES",
    "TERMINAL_NAVIGATION_STATES",
    "ActionStatus",
    "ArmSide",
    "CameraLayout",
    "ConnectPhase",
    "ConnectStatus",
    "ControlMode",
    "DaemonStatus",
    "ImuSample",
    "JogScale",
    "LidarScan",
    "LinkMode",
    "NavigationState",
    "NavigationStatus",
    "Perception",
    "PolicyStreamStatus",
    "RecordState",
    "RobotDescriptor",
    "RobotError",
    "RobotInfo",
    "RosStamp",
    "SafetyState",
    "SensorStreamStatus",
    "Telemetry",
    "WatchdogProfile",
    "WaypointSummary",
]
