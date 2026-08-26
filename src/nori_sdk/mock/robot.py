"""A robot that speaks nori-protocol into a list, with no WebRTC and no hardware.

Mirrors nori_gateway/protocol.py's observable behavior — the same frames, in the same order,
with the same tolerances. Where the two disagree, the GATEWAY is right and this is a bug:
this exists to catch client regressions, not to define new semantics.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from typing import Any

from ..version import NORI_PROTOCOL_VERSION

# A small two-arm robot with L2-SHAPED arms (5 DOF a side). The descriptor a client sees
# drives everything it can do, so tests using this are testing descriptor-driven behaviour
# for real. L2 rather than A3 on purpose: the 5-DOF arm is the legacy shape still in the
# field, and a client that works against the smaller descriptor works against the larger.
# Pass your own `descriptor=` to model an A3 (7 DOF a side, one central lift).
DEFAULT_DESCRIPTOR: dict[str, Any] = {
    "buses": ["left", "right"],
    "joints": [
        f"{side}_arm_{joint}.pos"
        for side in ("left", "right")
        for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "gripper")
    ],
    "base": ["x.vel", "theta.vel"],
    "aux": ["left_lift", "right_lift"],
    "cameras": ["left_wrist", "right_wrist", "overhead", "front"],
    "ranges": {
        f"{side}_arm_{joint}.pos": [-100.0, 100.0]
        for side in ("left", "right")
        for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    },
}

# The A3: 7-DoF arms + gripper per side, ONE central telescoping lift (bare "lift" jog key,
# "lift.pos" telemetry in MILLIMETERS). This is not a guess at the shape — it is transcribed
# from the LIVE descriptor an A3 gateway (NORI-A3-0000, nori_ws 7805a46) served over the
# wire on 2026-08-21, the same session that first drove the hardware through this SDK.
_A3_ARM_JOINTS = (
    "shoulder_pitch",
    "shoulder_roll",
    "bicep_yaw",
    "elbow_pitch",
    "forearm_yaw",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
)
A3_DESCRIPTOR: dict[str, Any] = {
    "buses": ["left", "right"],
    "joints": [
        f"{side}_arm_{joint}.pos" for side in ("left", "right") for joint in _A3_ARM_JOINTS
    ],
    "base": ["x.vel", "theta.vel"],
    "aux": ["lift"],
    "cameras": ["left_wrist", "right_wrist", "overhead", "front"],
    "ranges": {
        **{
            f"{side}_arm_{joint}.pos": (
                [0.0, 100.0] if joint == "gripper" else [-100.0, 100.0]
            )
            for side in ("left", "right")
            for joint in _A3_ARM_JOINTS
        },
        "lift.pos": [0.0, 720.0],  # millimeters; the 0-0.72 m prismatic column
    },
}

LAYOUT_REPEATS = 5  # the control channel is unreliable, so one-shot metadata is repeated

# Normalized-units per second at full jog rate. A round number chosen so a 1 s full-rate jog
# moves a visible 40 units on a [-100, 100] joint; the real per-tick step is robot-side and
# model-specific, and nothing should depend on this matching it.
JOG_SCALE = 40.0

# Central-lift advance at full rate, in mm/s — "lift.pos" telemetry is millimeters, so it
# cannot ride JOG_SCALE's normalized units. 50 mm/s matches the real column's normal-PWM
# speed (~0.05 m/s), but as with JOG_SCALE, nothing should depend on the match.
CENTRAL_LIFT_MM_PER_S = 50.0

# Task-space arm jog verbs the gateway accepts alongside joint shorts (nori_ws motion.py
# TASK_SHORTS). Integrated under their own name -- see step() -- because resolving them into
# joints would be inventing kinematics this SDK has no business claiming to know.
TASK_SHORTS = ("x", "y", "z", "pitch", "shoulder_pan")

# (t_warn_ms, t_stop_ms) per link mode. The ROBOT owns these -- the client only reports which
# network it is on, via `link`, and the robot picks. Hard-coded in both real stacks too.
WATCHDOG_PROFILES = {"lan": (150.0, 500.0), "wan": (300.0, 1000.0)}
# What a robot assumes before any `link` frame arrives: the more forgiving profile, because
# guessing LAN on a WAN client would stop a healthy session.
DEFAULT_LINK_MODE = "wan"


class MockRobot:
    """Feed it outbound client frames; read back what a robot would say.

        robot = MockRobot()
        for frame in robot.on_channel_open():
            client._handle_frame(frame)
        replies = robot.handle(sent_frame)
    """

    def __init__(
        self,
        *,
        descriptor: dict[str, Any] | None = DEFAULT_DESCRIPTOR,
        online: bool = True,
        cameras: bool = True,
        accepted: bool = True,
        protocol_version: int = NORI_PROTOCOL_VERSION,
        on_send: Callable[[str], None] | None = None,
        action_outcome: str = "done",
        capabilities: list[str] | None = None,
        tiles: list[str] | None = None,
    ) -> None:
        self.descriptor = descriptor
        # The composite video tiling this robot will ANNOUNCE. Defaults to the descriptor's
        # camera list because that is the common case, but it is a separate input on purpose
        # -- see _layout(). Pass it explicitly to rehearse a layout that disagrees with the
        # descriptor, which is a real thing robots do and a client must not assume away.
        self.tiles = (
            list(tiles)
            if tiles is not None
            else list((descriptor or {}).get("cameras", []) if descriptor else [])
        )
        self.online = online
        self.cameras = cameras
        self.accepted = accepted
        self.protocol_version = protocol_version
        # How an accepted action ENDS: "done", "clamped" or "timeout". Set it to rehearse the
        # outcomes a script has to handle but a bench robot rarely produces on demand.
        self.action_outcome = action_outcome
        # The optional verbs this double HONOURS -- and it honours exactly these, no more:
        # handle() checks this list before acting, so the double can never quietly serve a
        # verb it did not advertise. The default is what a healthy A3 gateway sends with
        # motion and the recorder up (nori_ws protocol.py on_channel_open: task_jog,
        # pose_targets, record) -- the mock used to omit pose_targets on the claim that the
        # gateway advertised nothing, which stopped being true and made pose() raise here
        # while working on hardware. Trim the list to rehearse the capability gate:
        #     MockRobot(capabilities=["task_jog", "record"])   # pose() refuses pre-flight
        self.capabilities = (
            ["task_jog", "pose_targets", "record"]
            if capabilities is None
            else list(capabilities)
        )
        self._on_send = on_send
        self.received: list[dict[str, Any]] = []
        self.link_mode = ""
        self.estopped = False
        self.jog: dict[str, Any] = {}
        self.action: dict[str, float] = {}
        # Integrated by step(); what telemetry() reports. Seeded with every joint
        # the descriptor advertises (at 0.0, mid-range by norm convention) plus a
        # mid-travel central lift when one is advertised — because that is what a
        # REAL gateway does: it reports every calibrated joint from the first
        # telemetry frame, commanded or not. The unseeded mock taught a policy
        # exactly the wrong lesson (hardware-found 2026-08-22: a policy's
        # telemetry-ready poll passed on the base keys alone, then KeyError'd on
        # an arm joint the real robot would always have sent).
        self.pose: dict[str, float] = {}
        for key in (descriptor or {}).get("joints", []) or []:
            self.pose[key] = 0.0
        if "lift" in ((descriptor or {}).get("aux", []) or []):
            low, high = (descriptor or {}).get("ranges", {}).get("lift.pos", (0.0, 720.0))
            self.pose["lift.pos"] = round((float(low) + float(high)) / 2.0, 1)
        self.streaming = False            # policy streamer state
        self.stream_dest: str | None = None
        self._frames_sent = 0
        self.watchdog = "ok"  # "ok" | "warn" | "stop", driven by control-frame silence
        self._elapsed = 0.0  # seconds accumulated via step(); the double's whole clock
        self._last_control_at = 0.0
        self.dropped_motion_frames = 0
        self.dropped_pose_frames = 0
        self._layout_sends = 0
        self._session_open = False
        self._episode_counter = 0
        self._episodes_kept = 0
        self._open_episode = ""

    # --- outbound ---------------------------------------------------------------------------

    def on_channel_open(self) -> list[str]:
        """ack -> camera_layout -> daemon_status, the gateway's order. A single-camera robot
        sends NO layout at all, which is why `cameras=False` is worth testing against."""
        frames = [self._ack()]
        if self.cameras:
            frames.append(self._layout())
            self._layout_sends = 1
        frames.append(self._daemon_status())
        return [self._emit(f) for f in frames]

    @property
    def watchdog_profile(self) -> tuple[float, float]:
        """(t_warn_ms, t_stop_ms) for the link mode the client selected. The robot picks the
        profile — the client only reports which network it is on."""
        return WATCHDOG_PROFILES.get(self.link_mode, WATCHDOG_PROFILES[DEFAULT_LINK_MODE])

    def step(self, dt: float) -> None:
        """Advance `dt` seconds: run the watchdog, then integrate whatever jog is held.

        Enough to make telemetry respond to commands — a script can drive and watch numbers
        move — and no more. This is NOT a simulator: no dynamics, no collision, no IK. Task-
        space arm keys ("x", "y", "pitch") are integrated as-is under their own name rather
        than resolved into joints, because resolving them would be inventing kinematics this
        SDK has no business claiming to know.

        The clock is `dt` accumulated here, NOT wall time, so a test can advance two seconds
        instantly and deterministically."""
        self._elapsed += dt

        # THE WATCHDOG. The single most important behaviour this double can teach, because it
        # is the one a script gets wrong in a way that works locally and fails on hardware:
        # the robot treats control-frame SILENCE as an absent operator and stops. Without this
        # a mock happily holds a jog forever, so "send one frame and sleep" passes here and
        # dies on a real robot. Any control frame resets it -- safe_hold self-clears, it does
        # not latch.
        silence_ms = (self._elapsed - self._last_control_at) * 1000.0
        t_warn, t_stop = self.watchdog_profile
        if silence_ms > t_stop:
            self.watchdog = "stop"
            self.jog = {}  # motion ceases; the robot has decided nobody is driving
        elif silence_ms > t_warn:
            self.watchdog = "warn"
        else:
            self.watchdog = "ok"

        if self.estopped or not self.online:
            self._set_base_velocity(None)
            return

        # The base reports VELOCITY, not an accumulated position, so it is SET from the
        # current jog every tick rather than integrated. Read outside the loop below on
        # purpose: an ABSENT `base` object means STOP, not "hold the last velocity" (see
        # control.json). Deriving it only from keys that are present would leave a base
        # coasting forever after any later jog that omitted it -- which is what this mock did
        # on its first draft, and it would have taught a script exactly the wrong lesson.
        self._set_base_velocity(self.jog.get("base"))

        # Unknown jog vocabulary is dropped in SILENCE, exactly as the gateway drops it:
        # apply_jog only ever reads the groups its model has, and _integrate_arm_jog skips
        # any short its keymap lacks (jog has no reply channel, so there is nothing louder
        # to do). The mock used to integrate ANY group and invent telemetry keys for it,
        # which let a typo'd group read as working motion. No descriptor = no vocabulary
        # to check, so the legacy-robot mock stays permissive.
        joints = self._joint_vocab()
        aux = self.descriptor.get("aux", []) if self.descriptor else None
        for group, value in self.jog.items():
            if group in ("left_lift", "right_lift", "lift"):
                if aux is not None and group not in aux:
                    continue  # a rail this robot doesn't have: the gateway never reads it
                # Bare-scalar lifts. "lift" is the A-series central column and its
                # telemetry unit is MILLIMETERS, so it advances on its own scale —
                # 50 mm/s at full rate, matching the real Pico's normal-PWM speed —
                # while the per-arm rails stay on the normalized JOG_SCALE.
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    scale = CENTRAL_LIFT_MM_PER_S if group == "lift" else JOG_SCALE
                    self._advance(f"{group}.pos", float(value) * dt * scale)
            elif group != "base" and isinstance(value, dict):
                if joints is not None and not any(
                    key.startswith(group + "_") for key in joints
                ):
                    continue  # not an arm group on this robot: never read, no motion
                for dof, rate in value.items():
                    if joints is not None and (
                        f"{group}_{dof}.pos" not in joints and dof not in TASK_SHORTS
                    ):
                        continue  # unknown short: skipped in silence, like the keymap miss
                    if isinstance(rate, (int, float)) and not isinstance(rate, bool):
                        self._advance(f"{group}_{dof}.pos", float(rate) * dt * JOG_SCALE)

    def _set_base_velocity(self, base: Any) -> None:
        rates = base if isinstance(base, dict) else {}
        self.pose["x.vel"] = float(rates.get("linear", 0.0) or 0.0)
        self.pose["theta.vel"] = float(rates.get("angular", 0.0) or 0.0)

    def _advance(self, key: str, delta: float) -> None:
        ranges = self.descriptor.get("ranges", {}) if self.descriptor else {}
        low, high = ranges.get(key, (-100.0, 100.0))
        # Clamped, never rejected -- the same contract the real robot offers.
        self.pose[key] = min(max(self.pose.get(key, 0.0) + delta, low), high)

    def _joint_vocab(self) -> list[str] | None:
        """The joint keys motion vocabulary is validated against; None means "no vocabulary".

        None ONLY when there is no descriptor at all (the legacy-robot mock): the client was
        told nothing, so both motion paths stay permissive, honestly. A descriptor WITHOUT a
        "joints" key is a vocabulary — an empty one — and is strict. One definition for the
        jog, action and pose paths: reading it three different ways once left the same robot
        permissive about an action and strict about a jog for the same missing key."""
        if self.descriptor is None:
            return None
        return self.descriptor.get("joints") or []

    def telemetry(self, state: dict[str, float] | None = None, with_status: bool = True) -> str:
        """One telemetry frame. Defaults to the integrated pose; pass `state` to force it."""
        frame: dict[str, Any] = {
            "type": "telemetry",
            "ts_ns": time.time_ns(),
            "state": dict(self.pose if state is None else state),
        }
        if with_status:
            if self.estopped:
                safety, reason = "latched", "operator estop"
            elif self.watchdog == "stop":
                # Motion refused, but NO latch: safe_hold clears itself the moment control
                # frames resume. Only an E-STOP needs reset_latch().
                safety, reason = "safe_hold", "control silence"
            else:
                safety, reason = "ok", None
            frame["status"] = {
                "safety": safety,
                "watchdog": self.watchdog,
                "latch_reason": reason,
            }
        return self._emit(frame)

    def repeat_layout(self) -> str | None:
        """The gateway repeats the layout LAYOUT_REPEATS times, then stops."""
        if not self.cameras or self._layout_sends >= LAYOUT_REPEATS:
            return None
        self._layout_sends += 1
        return self._emit(self._layout())

    def _ack(self) -> dict[str, Any]:
        ack: dict[str, Any] = {
            "type": "ack",
            "accepted": self.accepted,
            "protocol_version": self.protocol_version,
            # Advisory label so logs name the double honestly (never branch on it).
            "model": "MOCK",
            # The TRUTHFUL set of optional verbs this double honours (spec ack.json), and
            # handle() enforces it -- advertise and it is served, omit and it is dropped, so
            # the two can never disagree.
            "capabilities": list(self.capabilities),
        }
        if self.descriptor is not None:
            ack["norm_mode"] = "range_m100_100"
            ack["descriptor"] = self.descriptor
            t_warn, t_stop = self.watchdog_profile
            ack["watchdog_profile"] = {"t_warn_ms": t_warn, "t_stop_ms": t_stop}
        return ack

    def _layout(self) -> dict[str, Any]:
        """The composite tiling.

        Derived from `self.tiles`, NOT from descriptor.cameras. The schema calls this out
        directly: the layout frame is the only authoritative description of the tiling, and
        descriptor.cameras is diagnostic metadata that may legitimately differ. A mock that
        generates one from the other can never fail the way a real robot does, so it hides
        exactly the layout bugs it exists to catch. Set `tiles` to rehearse a disagreement."""
        cols = 2 if len(self.tiles) > 1 else 1
        rows = max(1, -(-len(self.tiles) // cols))  # ceil, so no tile falls off the grid
        return {"type": "camera_layout", "cols": cols, "rows": rows, "tiles": list(self.tiles)}

    def _daemon_status(self) -> dict[str, Any]:
        if self.online:
            return {"type": "daemon_status", "state": "online"}
        return {
            "type": "daemon_status",
            "state": "offline",
            "reason": "unreachable",
            "detail": "gateway: motion disabled (enable_motion: false)",
        }

    def _emit(self, frame: dict[str, Any]) -> str:
        raw = json.dumps(frame)
        if self._on_send is not None:
            self._on_send(raw)
        return raw

    # --- inbound ------------------------------------------------------------------------------

    def handle(self, raw: str | dict[str, Any]) -> list[str]:
        """Apply one client frame; return whatever the robot would say back."""
        message = json.loads(raw) if isinstance(raw, str) else dict(raw)
        self.received.append(message)
        kind = message.get("type")

        if kind == "control":
            # Feeds the dead-man clock BEFORE the online check: the operator demonstrably sent
            # a frame, whether or not the motion stack was up to act on it.
            self._last_control_at = self._elapsed
            if not self.online:
                self.dropped_motion_frames += 1
                return []
            if isinstance(message.get("jog"), dict):
                self.jog = message["jog"]
            replies = []
            pose = message.get("pose")
            # Dropped in SILENCE when unadvertised -- which is what a real robot does with a
            # verb it does not implement, and the behaviour the ack above promises. Serving it
            # anyway would make the double MORE capable than it claims, so a client that
            # ignored the capability gate would pass here and fail on hardware.
            if "pose_targets" not in self.capabilities:
                self.dropped_pose_frames += 1
                pose = None
            if isinstance(pose, dict):
                # pose (capability pose_targets): the double has no IK — it
                # accepts the shape and teleports the arm's telemetry a
                # plausible step so scripts see motion, then runs the same
                # action lifecycle. Geometry truth lives in the SIM stage;
                # this stage rehearses PROTOCOL flow only.
                action_id = message.get("action_id")
                sides = [k[:-4] for k in pose if k.endswith("_arm")]
                if self.estopped:
                    # The gateway refuses a latched pose with ONE terminal frame
                    # (apply_pose: refuse("estop_latched")). Dropping it in silence
                    # stranded a wait=True client into its own 10-15 s timeout — the
                    # wrong estop shape to teach.
                    if action_id:
                        replies.append(self._emit({
                            "type": "action_status", "action_id": action_id,
                            "state": "blocked", "reason": "estop_latched",
                            "ts_ns": time.time_ns()}))
                elif action_id and sides:
                    side = sides[0]
                    # The joints this robot actually HAS for that side. Teleporting a
                    # hard-coded shoulder_pitch invented a telemetry key for an absent
                    # arm -- the same class of lie the vocabulary checks exist to stop.
                    side_joints = [
                        key
                        for key in (self._joint_vocab() or [])
                        if key.startswith(f"{side}_arm_")
                    ]
                    target = pose.get(f"{side}_arm") or {}
                    ok = (target.get("frame") == "base_footprint"
                          and isinstance(target.get("position_m"), list)
                          and len(target["position_m"]) == 3)
                    if self.descriptor is not None and not side_joints:
                        # The gateway draws sides from the arms it has and refuses a
                        # pose for one it doesn't (apply_pose: refuse("empty_pose")).
                        replies.append(self._emit({
                            "type": "action_status", "action_id": action_id,
                            "state": "blocked", "reason": "empty_pose",
                            "ts_ns": time.time_ns()}))
                    elif not ok:
                        replies.append(self._emit({
                            "type": "action_status", "action_id": action_id,
                            "state": "blocked", "reason": "bad_pose",
                            "ts_ns": time.time_ns()}))
                    else:
                        # No descriptor = no vocabulary: the legacy-robot mock stays
                        # permissive and nudges the conventional first joint.
                        self._advance(
                            side_joints[0] if side_joints
                            else f"{side}_arm_shoulder_pitch.pos",
                            -5.0,
                        )
                        for state in self._action_lifecycle():
                            pose_frame: dict[str, Any] = {
                                "type": "action_status",
                                "action_id": action_id, "state": state,
                                "ts_ns": time.time_ns()}
                            if state == "blocked":
                                pose_frame["reason"] = "no_ik:-31"
                            replies.append(self._emit(pose_frame))
            if isinstance(message.get("action"), dict):
                # The gateway checks every key against its keymap -- the calibrated arm
                # joints -- and applies the ones it knows; keys it doesn't (or non-numeric
                # values) collect into `unknown` (nori_ws motion.py apply_action). The mock
                # used to accept ANY spelling and invent a range for it, so a client speaking
                # a different robot's vocabulary stayed green here and failed on hardware.
                # The vocabulary is the descriptor's joint list, the same thing the keymap
                # represents on the wire — see _joint_vocab() for the no-descriptor case.
                joints = self._joint_vocab()
                applied: dict[str, float] = {}
                unknown: list[str] = []
                for key, target in message["action"].items():
                    if (joints is not None and key not in joints) or not isinstance(
                        target, (int, float)
                    ):
                        unknown.append(str(key))
                    else:
                        applied[key] = float(target)
                if not self.estopped:
                    self.action.update(applied)
                    # An absolute target lands in the pose, clamped. Without this a script
                    # that commands a position and then reads telemetry sees nothing move.
                    for key, target in applied.items():
                        self._advance(key, target - self.pose.get(key, 0.0))
                action_id = message.get("action_id")
                if action_id and not self.estopped and not applied:
                    # Nothing matched this robot's key space: refuse loudly, ONE terminal
                    # frame, same reason string as the gateway. Silently accepting used to
                    # strand a real client in a 12 s await-done no-op whenever it spoke a
                    # different robot's vocabulary -- the mock must refuse the same way.
                    refusal: dict[str, Any] = {
                        "type": "action_status",
                        "action_id": action_id,
                        "state": "blocked",
                        "reason": (
                            "unknown_joint:" + ",".join(sorted(unknown))
                            if unknown
                            else "empty_action"
                        ),
                        "ts_ns": time.time_ns(),
                    }
                    replies.append(self._emit(refusal))
                elif action_id:
                    # The full lifecycle, not just the first frame. A real robot answers
                    # accepted -> active -> done; emitting only "accepted" made this double
                    # agree with a client that treated acceptance as completion, so the two
                    # bugs cancelled and the suite stayed green. A test double that models
                    # the bug cannot catch it.
                    for state in self._action_lifecycle():
                        frame: dict[str, Any] = {
                            "type": "action_status",
                            "action_id": action_id,
                            "state": state,
                            "ts_ns": time.time_ns(),
                        }
                        if state == "blocked":
                            # The gateway's string (apply_action: "estop_latched"), not
                            # the telemetry safety STATE "latched" — two vocabularies.
                            frame["reason"] = "estop_latched"
                        replies.append(self._emit(frame))
            return replies
        if kind == "command":
            if message.get("estop"):
                self.estopped = True
            elif message.get("reset_latch"):
                self.estopped = False
            return []
        if kind == "link":
            mode = message.get("mode")
            if mode in ("lan", "wan"):
                self.link_mode = mode
            return []
        if kind == "record":
            return [self._emit(self._record(message))]
        if kind == "policy_stream":
            return [self._emit(self._policy_stream(message))]
        return []  # video/call are bridge-intercepted; no audio device in a double

    def _action_lifecycle(self) -> list[str]:
        """The action_status states this robot would emit, in order.

        A latched robot refuses outright — one terminal frame, no progress. Otherwise it
        acknowledges, moves, and settles, and only that last frame is terminal."""
        if self.estopped:
            return ["blocked"]
        return ["accepted", "active", self.action_outcome]

    def _policy_stream(self, message: dict[str, Any]) -> dict[str, Any]:
        """Answer a policy_stream verb.

        Modelled as REQUEST/REPLY only, deliberately: the real streamer is a ZMQ REP socket
        that cannot push, so it never announces its own death. A double that helpfully emitted
        a failure frame would teach a client to wait for something no robot sends."""
        action = message.get("action")
        if action == "start":
            self.streaming = True
            self.stream_dest = message.get("dest")
            self._frames_sent = 0
        elif action == "stop":
            self.streaming = False
        elif action != "status":
            # Unknown verb: ok:false is ordinary state for this reply, not an error.
            return {"type": "policy_stream_status", "ok": False,
                    "error": f"unknown action {action!r}"}
        if self.streaming:
            self._frames_sent += 30
        return {
            "type": "policy_stream_status",
            "ok": True,
            "streaming": self.streaming,
            "dest": self.stream_dest,
            "fps_out": 14.8 if self.streaming else 0.0,
            "frames_sent": self._frames_sent,
            "dropped": 0,
        }

    def die_mid_stream(self) -> None:
        """Rehearse the failure mode that has no notification: the stream stops and the robot
        says NOTHING. The only way a client finds out is by polling "status"."""
        self.streaming = False

    def perception(self, objects: list[dict[str, Any]] | None = None) -> str:
        """One `perception` frame. Not emitted on a timer -- a robot with no detector running
        sends none at all, which is the common case a client must handle."""
        return self._emit({
            "type": "perception",
            "ts_ns": time.time_ns(),
            "objects": list(objects or []),
        })

    def _record(self, message: dict[str, Any]) -> dict[str, Any]:
        action = message.get("action")
        error = ""
        if action in ("session_start", "start"):
            self._session_open = True
            self._episodes_kept = 0
        elif action == "episode_start":
            if self._open_episode:
                error = "already recording an episode"
            else:
                self._session_open = True
                self._episode_counter += 1
                self._open_episode = f"episode-{self._episode_counter:04d}"
        elif action in ("episode_stop", "stop", "episode_discard"):
            if not self._open_episode:
                error = "not recording"
            else:
                if action != "episode_discard":
                    self._episodes_kept += 1
                self._open_episode = ""
        elif action in ("session_end", "session_discard", "discard", "discard_last"):
            # All four just close the session, ok:true — gateway-verbatim. Episode-as-unit
            # means each finalized episode already shipped independently, so "discard" here
            # deletes nothing on this stack (it DOES destroy data on the L2 stack, which is
            # why DESTRUCTIVE_RECORD_VERBS cannot classify it statically).
            self._session_open = False
        elif action != "status":
            error = f"unknown action {action}"
        frame: dict[str, Any] = {
            "type": "record_status",
            "ok": not error,
            "recording": bool(self._open_episode),
            "session_open": self._session_open,
            "episodes_kept": self._episodes_kept,
            "episode": self._open_episode or None,
            "free_gb": 42.0,
        }
        if error:
            frame["error"] = error
        return frame


__all__ = [
    "A3_DESCRIPTOR",
    "CENTRAL_LIFT_MM_PER_S",
    "DEFAULT_DESCRIPTOR",
    "DEFAULT_LINK_MODE",
    "JOG_SCALE",
    "LAYOUT_REPEATS",
    "TASK_SHORTS",
    "WATCHDOG_PROFILES",
    "MockRobot",
]
