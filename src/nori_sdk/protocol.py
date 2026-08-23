"""The nori-protocol wire vocabulary, operator side. Pure functions, zero dependencies.

This module is the Python counterpart of the message literals scattered through @nori/sdk's
teleop.ts, and the mirror image of the robot's nori_gateway/protocol.py: what that module
`handle()`s, this module builds, and what it emits, this module parses.

Keeping it separate from the session (teleop.py) is the point — the vocabulary is the part
that must stay byte-identical across three implementations (TS SDK, this SDK, the robot
gateway), so it lives somewhere it can be tested against shared conformance vectors with no
WebRTC stack in the room. See tests/vectors/ and README "Staying in sync".

SAFETY: nothing here can make a robot unsafe. Clamping, the watchdog, E-STOP and the torque
lifecycle all live on the robot, which defends itself against any client — a malformed or
hostile frame can at worst be ignored.
"""

from __future__ import annotations

import json
from typing import Any, Literal

from .types import (
    ActionStatus,
    CameraLayout,
    DaemonStatus,
    Perception,
    PolicyStreamStatus,
    RecordState,
    RobotError,
    RobotInfo,
    Telemetry,
)

# Frames the robot may send us. Anything outside this set is ignored (forward compat).
INBOUND_KINDS = frozenset(
    {
        "ack",
        "telemetry",
        "camera_layout",
        "daemon_status",
        "action_status",
        "record_status",
        "policy_stream_status",
        "perception",
        "error",
    }
)

# Frames we may send. The robot ignores what it doesn't know, and so must we.
OUTBOUND_KINDS = frozenset(
    {"control", "command", "video", "link", "record", "policy_stream", "call"}
)

# A `jog` payload: normalized rates in [-1, 1] per DOF. The robot scales by its own per-tick
# step, so these are intent, not units. Nested per group:
#   {"left_arm": {"shoulder_pan": 0.5}, "base": {"linear": 1.0}, "left_lift": -0.3}
# NOTE the base keys: "linear"/"angular", NOT the descriptor's "x.vel"/"theta.vel".
# A robot reads linear/angular as absent -- i.e. STOP -- from an x/theta payload.
Jog = dict[str, Any]

# An `action` payload: ABSOLUTE targets, flat "<motor>.pos" -> value, in the units named by
# RobotInfo.norm_mode (normally "range_m100_100"; grippers are [0, 100]).
Action = dict[str, float]

# Recording verbs. Grouped by what they do to DATA, because that is the axis that matters and
# the names do not reliably signal it.
#
#   KEEPS DATA
#     session_start    open a collection
#     episode_start    open an episode (opens a session too, if none is open)
#     episode_stop     finalise and KEEP the open episode
#     session_end      close the session and keep it for shipping
#     status           read-only
#
#   DESTROYS DATA -- irreversibly, on disk
#     episode_discard  finalise and DROP the open episode
#     session_discard  drop the ENTIRE open session (or, straight after session_end, the most
#                      recent one). NOT a synonym for session_end -- they are opposites, and
#                      despite sitting beside the aliases below this one is canonical.
#
#   LEGACY ALIASES -- deployed clients still send these and robots still accept them, so this
#   SDK must be able to name them. Prefer the canonical verbs above in new code.
#     start            open a session AND an episode
#     stop             finalise the episode -- and on L2 ALSO END THE SESSION. A client using
#                      `stop` between episodes on an L2 silently gets a different session
#                      than it expects.
#     discard          DESTROYS on L2 (alias for session_discard); KEEPS on A3/L3 (folded into
#                      session_end). The same verb means opposite things per stack. This
#                      divergence is real and unresolved upstream -- do not build a UI on it.
#     discard_last     alias for session_discard on L2, targeting the most recent session.
RecordVerb = Literal[
    "session_start",
    "episode_start",
    "episode_stop",
    "episode_discard",
    "session_end",
    "session_discard",
    "status",
    "start",
    "stop",
    "discard",
    "discard_last",
]

# The verbs that delete recorded data. `discard` is deliberately absent: it destroys on L2 and
# keeps on A3, so it belongs to neither set and a caller must resolve it per robot.
DESTRUCTIVE_RECORD_VERBS = frozenset({"episode_discard", "session_discard", "discard_last"})

CommandName = Literal["estop", "reset_latch"]


# --- outbound builders ---------------------------------------------------------------------
# Each returns a plain dict; the session serializes. Split this way so tests can assert on
# structure without a transport, and so a caller can inspect/log a frame before it flies.


def control_jog(seq: int, jog: Jog) -> dict[str, Any]:
    """The continuous velocity stream. Send it at ~20 Hz while the operator is holding an
    input: the robot's watchdog ramps to a stop if frames stop arriving inside
    WatchdogProfile.t_warn_ms, which is the intended dead-man behavior, not a bug to work
    around. Send an all-zero jog to stop cleanly."""
    return {"type": "control", "seq": seq, "jog": jog}


def control_action(seq: int, action: Action, action_id: str = "") -> dict[str, Any]:
    """An absolute move. With an action_id the robot replies `action_status` (accepted or
    blocked); without one it is fire-and-forget."""
    frame: dict[str, Any] = {"type": "control", "seq": seq, "action": action}
    if action_id:
        frame["action_id"] = action_id
    return frame


POSE_FRAME = "base_footprint"


def control_pose(
    seq: int,
    side: str,
    position_m: list[float] | tuple[float, ...],
    orientation_xyzw: list[float] | tuple[float, ...] | None = None,
    action_id: str = "",
) -> dict[str, Any]:
    """An absolute Cartesian pose target for one arm's gripper TCP, solved to joints ON THE
    ROBOT (spec control.json `pose`; gate on the "pose_targets" capability).

    The frame is named in every message and is always base_footprint today: fixed to the
    robot, stable across lift travel. Metres, REP-103 (+x forward, +y left, +z up), and the
    orientation — when you send one — is a ROS-order quaternion [x, y, z, w]. Omit it for
    "get the gripper to this point, any wrist angle": the robot solves at its current
    wrist, and forcing a full pose turns those tasks into avoidable IK failures.

    ONE arm per frame: a control frame carries one action_id and arms fail independently,
    so a dual-arm pose is two calls. Malformed vectors raise here rather than flying —
    the robot would refuse them as `bad_pose` after a round trip."""
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    if len(position_m) != 3:
        raise ValueError(f"position_m needs [x, y, z] metres, got {position_m!r}")
    if orientation_xyzw is not None and len(orientation_xyzw) != 4:
        raise ValueError(
            f"orientation_xyzw needs [x, y, z, w], got {orientation_xyzw!r}")
    target: dict[str, Any] = {
        "frame": POSE_FRAME,
        "position_m": [float(v) for v in position_m],
    }
    if orientation_xyzw is not None:
        target["orientation_xyzw"] = [float(v) for v in orientation_xyzw]
    frame: dict[str, Any] = {
        "type": "control", "seq": seq, "pose": {f"{side}_arm": target}}
    if action_id:
        frame["action_id"] = action_id
    return frame


def control_leader(seq: int, leader_action_deg: dict[str, float]) -> dict[str, Any]:
    """Absolute pose from a physical leader arm. Keys are flat "<side>_arm_<joint>.pos";
    body joints are DEGREES around the calibrated leader zero, grippers normalized [0, 100].
    The robot does calibration-normalize + IK + a server-side slew clamp.

    Note: L2 leader keys are not mappable to a 7-DOF A3 arm — the A3 gateway logs and
    ignores them. Check RobotInfo.descriptor (or capabilities["leader_action_deg"]) before
    driving a robot this way."""
    return {"type": "control", "seq": seq, "leader_action_deg": leader_action_deg}


def control_pose(seq: int, side: str, position_m: list[float],
                 orientation_xyzw: list[float] | None = None,
                 action_id: str = "") -> dict[str, Any]:
    """A Cartesian pose target (capability `pose_targets`), solved on-robot by
    IK into the same latched-target path as `action`. ONE arm per frame; the
    reply lifecycle rides action_status (accepted -> active -> done | blocked
    | timeout, terminal states from OBSERVED joint motion). Frame is pinned to
    base_footprint (REP-103). Omitted orientation = keep the CURRENT tcp
    orientation. Reference implementation: nori_gateway 3b55c78."""
    target: dict[str, Any] = {"frame": "base_footprint",
                              "position_m": [float(v) for v in position_m]}
    if orientation_xyzw is not None:
        target["orientation_xyzw"] = [float(v) for v in orientation_xyzw]
    frame: dict[str, Any] = {"type": "control", "seq": seq,
                             "pose": {f"{side}_arm": target}}
    if action_id:
        frame["action_id"] = action_id
    return frame


def control_reset(arm: str) -> dict[str, Any]:
    """Clear the per-arm latch/target state. Note this rides `control`, not `command`."""
    return {"type": "control", "reset": {arm: True}}


def command(name: CommandName) -> dict[str, Any]:
    """E-STOP or latch reset. Wire shape is a flag, not a name: {"estop": true}."""
    return {"type": "command", name: True}


def video_state(paused: bool) -> dict[str, Any]:
    """Pause/resume the inbound video encoder on the robot (power saving). The robot
    defaults to flowing, so only send this when you want it paused."""
    return {"type": "video", "state": "pause" if paused else "resume"}


def video_bitrate(kbps: int) -> dict[str, Any]:
    """Target encoder bitrate. Intercepted by the robot's WebRTC bridge — it never reaches
    the motion daemon. The robot applies its own thermal/load ceiling on top."""
    return {"type": "video", "bitrate": int(kbps)}


def video_quality(quality: str) -> dict[str, Any]:
    """Named quality preset ("low" | "normal")."""
    return {"type": "video", "quality": quality}


def link(mode: str) -> dict[str, Any]:
    """Tell the robot which network path we resolved to. This SELECTS ITS WATCHDOG PROFILE
    (LAN 150/500 ms vs WAN 300/1000 ms), so reporting "lan" on a WAN link gives you a
    tighter dead-man window than your latency can hold. RemoteTeleop derives it from the
    selected ICE candidate pair; only override if you know better."""
    return {"type": "link", "mode": mode}


def record(action: RecordVerb, task: str = "", **extra: Any) -> dict[str, Any]:
    """Dataset recording verbs. Every verb draws exactly one `record_status` reply, so a
    client can await each one. Order is session_start -> episode_start -> episode_stop
    (or episode_discard) -> ... -> session_end."""
    frame: dict[str, Any] = {"type": "record", "action": action}
    if task:
        frame["task"] = task
    frame.update(extra)
    return frame


def policy_stream(action: str, **extra: Any) -> dict[str, Any]:
    """Drive the robot's policy streamer (start/stop/status)."""
    frame: dict[str, Any] = {"type": "policy_stream", "action": action}
    frame.update(extra)
    return frame


def call(
    state: str | None = None, mic_muted: bool | None = None, clip: bool = False
) -> dict[str, Any]:
    """Two-way audio call control. Intercepted by the robot's bridge like `video`."""
    frame: dict[str, Any] = {"type": "call"}
    if state is not None:
        frame["state"] = state
    if mic_muted is not None:
        frame["mic_muted"] = mic_muted
    if clip:
        frame["clip"] = True
    return frame


def encode(frame: dict[str, Any]) -> str:
    """Serialize one outbound frame. Compact separators because the control channel is
    per-message and we send it at 20 Hz."""
    return json.dumps(frame, separators=(",", ":"))


# --- inbound parsing -----------------------------------------------------------------------

Inbound = (
    RobotInfo
    | Telemetry
    | CameraLayout
    | DaemonStatus
    | ActionStatus
    | RecordState
    | PolicyStreamStatus
    | Perception
    | RobotError
)


def decode(raw: str | bytes) -> tuple[str, Inbound | None, dict[str, Any]]:
    """Parse one inbound datachannel message.

    Returns (kind, parsed, raw_dict). `parsed` is None for a frame kind this SDK version
    doesn't model — the raw dict is still handed back so a caller can act on a newer robot's
    vocabulary without waiting for an SDK release. Unparseable JSON yields ("", None, {}):
    the channel is unreliable and lossy by design, so garbage is dropped, never raised.
    """
    try:
        obj = json.loads(raw)
    except (ValueError, TypeError):
        return "", None, {}
    if not isinstance(obj, dict):
        return "", None, {}
    kind = obj.get("type")
    if not isinstance(kind, str):
        return "", None, obj

    parsed: Inbound | None
    if kind == "ack":
        parsed = RobotInfo.from_wire(obj)
    elif kind == "telemetry":
        parsed = Telemetry.from_wire(obj)
    elif kind == "camera_layout":
        parsed = CameraLayout.from_wire(obj)
    elif kind == "daemon_status":
        parsed = DaemonStatus.from_wire(obj)
    elif kind == "action_status":
        parsed = ActionStatus.from_wire(obj)
    elif kind == "record_status":
        parsed = RecordState.from_wire(obj)
    elif kind == "policy_stream_status":
        parsed = PolicyStreamStatus.from_wire(obj)
    elif kind == "perception":
        parsed = Perception.from_wire(obj)
    elif kind == "error":
        parsed = RobotError.from_wire(obj)
    else:
        parsed = None
    return kind, parsed, obj


__all__ = [
    "DESTRUCTIVE_RECORD_VERBS",
    "INBOUND_KINDS",
    "OUTBOUND_KINDS",
    "POSE_FRAME",
    "Action",
    "CommandName",
    "Inbound",
    "Jog",
    "RecordVerb",
    "call",
    "command",
    "control_action",
    "control_jog",
    "control_leader",
    "control_pose",
    "control_reset",
    "decode",
    "encode",
    "link",
    "policy_stream",
    "record",
    "video_bitrate",
    "video_quality",
    "video_state",
]
