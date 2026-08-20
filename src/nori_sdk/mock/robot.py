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

LAYOUT_REPEATS = 5  # the control channel is unreliable, so one-shot metadata is repeated

# Normalized-units per second at full jog rate. A round number chosen so a 1 s full-rate jog
# moves a visible 40 units on a [-100, 100] joint; the real per-tick step is robot-side and
# model-specific, and nothing should depend on this matching it.
JOG_SCALE = 40.0

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
        self._on_send = on_send
        self.received: list[dict[str, Any]] = []
        self.link_mode = ""
        self.estopped = False
        self.jog: dict[str, Any] = {}
        self.action: dict[str, float] = {}
        self.pose: dict[str, float] = {}  # integrated by step(); what telemetry() reports
        self.watchdog = "ok"  # "ok" | "warn" | "stop", driven by control-frame silence
        self._elapsed = 0.0  # seconds accumulated via step(); the double's whole clock
        self._last_control_at = 0.0
        self.dropped_motion_frames = 0
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

        for group, value in self.jog.items():
            if group in ("left_lift", "right_lift"):
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    self._advance(f"{group}.pos", float(value) * dt * JOG_SCALE)
            elif group != "base" and isinstance(value, dict):
                for dof, rate in value.items():
                    if isinstance(rate, (int, float)) and not isinstance(rate, bool):
                        self._advance(f"{group}_{dof}.pos", float(rate) * dt * JOG_SCALE)

    def _set_base_velocity(self, base: Any) -> None:
        rates = base if isinstance(base, dict) else {}
        self.pose["x.vel"] = float(rates.get("linear", 0.0) or 0.0)
        self.pose["theta.vel"] = float(rates.get("angular", 0.0) or 0.0)

    def _advance(self, key: str, delta: float) -> None:
        ranges = (self.descriptor or {}).get("ranges", {}) if self.descriptor else {}
        low, high = ranges.get(key, (-100.0, 100.0))
        # Clamped, never rejected -- the same contract the real robot offers.
        self.pose[key] = min(max(self.pose.get(key, 0.0) + delta, low), high)

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
            if isinstance(message.get("action"), dict):
                self.action.update(message["action"])
                if not self.estopped:
                    # An absolute target lands in the pose, clamped. Without this a script
                    # that commands a position and then reads telemetry sees nothing move.
                    for key, target in message["action"].items():
                        if isinstance(target, (int, float)) and not isinstance(target, bool):
                            self._advance(key, float(target) - self.pose.get(key, 0.0))
                action_id = message.get("action_id")
                if action_id:
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
                            frame["reason"] = "latched"
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
        return []  # video/call/policy_stream are bridge-intercepted or out of scope

    def _action_lifecycle(self) -> list[str]:
        """The action_status states this robot would emit, in order.

        A latched robot refuses outright — one terminal frame, no progress. Otherwise it
        acknowledges, moves, and settles, and only that last frame is terminal."""
        if self.estopped:
            return ["blocked"]
        return ["accepted", "active", self.action_outcome]

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
        elif action == "session_end":
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


__all__ = ["DEFAULT_DESCRIPTOR", "LAYOUT_REPEATS", "MockRobot"]
