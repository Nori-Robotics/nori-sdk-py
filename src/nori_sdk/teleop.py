"""RemoteTeleop — the operator session: signaling handshake, WebRTC peer, control channel.

The Python counterpart of @nori/sdk's RemoteTeleop. Same protocol, same role (we are always
the ANSWERER), same defensive posture; different concurrency model — this is asyncio-native
because aiortc is. (No synchronous facade yet — see README "Not built yet".)

    from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth

    auth = UserAuth(URL, ANON, "me@example.com", "pw")
    sig = SupabaseSignaling(URL, ANON, room="NORI-A3-0001", token_provider=auth.token)

    async with RemoteTeleop(sig) as robot:
        await robot.wait_ready()
        print(robot.info.descriptor.joints)
        await robot.jog({"base": {"linear": 0.5}}, duration=1.0)
        await robot.action({"left_arm_gripper.pos": 30}, wait=True)

TWO DIVERGENCES FROM THE BROWSER SDK, both forced by aiortc and both load-bearing:

 1. ICE is NOT trickled outbound. aiortc completes gathering inside setLocalDescription, so
    our candidates ride inside the answer SDP and send_ice() is never called on the way out.
    We still accept trickled candidates FROM the robot (GStreamer's webrtcbin trickles), so
    the transport must still deliver on_ice. Connection setup is therefore a touch slower
    than the browser's but functionally identical.
 2. There is no ABR loop yet. The browser adapts the robot's encoder bitrate from live
    getStats; a script usually wants a fixed quality, so set_video_bitrate() is manual. See
    README "Not built yet".

SAFETY: nothing this class sends can make a robot unsafe. Clamping, the watchdog, E-STOP and
the torque lifecycle are all robot-side. Note the corollary: the watchdog means a jog stream
that stops IS a stop command — see jog() and stream_jog().
"""

from __future__ import annotations

import asyncio
import contextlib
import importlib.util
import logging
import math
import time
import uuid
from collections import OrderedDict
from collections.abc import AsyncIterator, Callable, Iterable
from typing import Any, Self

from . import protocol, webrtc_compat
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
    ActionStatus,
    CameraLayout,
    ConnectStatus,
    DaemonStatus,
    ImuSample,
    LidarScan,
    NavigationStatus,
    Perception,
    PolicyStreamStatus,
    RecordState,
    RobotInfo,
    SensorStreamStatus,
    Telemetry,
    TERMINAL_NAVIGATION_STATES,
)
from .version import NORI_PROTOCOL_VERSION

log = logging.getLogger("nori_sdk.teleop")

# How often a held jog is re-sent. The robot's watchdog warns at 150 ms (LAN) / 300 ms (WAN),
# so 20 Hz leaves ~3 frames of headroom on the tighter profile.
JOG_HZ = 20.0

# How many per-action verdicts to remember for action_status(). Bounds memory on a long
# unattended run; nothing needs more than the most recent handful.
ACTION_HISTORY = 256
# How long we wait for the robot to announce itself before calling it absent. The robot
# announces on join and we re-ask every RETRY_S, so this is several missed chances.
ROBOT_WAIT_S = 20.0
RETRY_S = 2.0

# What stop() pushes into every live stream() queue: the consumer's await lives in the
# caller's own task, which stop() cannot cancel, so it must be WOKEN to find out.
_STREAM_CLOSED = object()


class TeleopError(RuntimeError):
    """A session-level failure: refused, unreachable, or torn down mid-call."""


class RemoteTeleop:
    def __init__(
        self,
        signaling: SignalingTransport,
        *,
        stun: Iterable[str] = ("stun:stun.l.google.com:19302",),
        turn_urls: Iterable[str] = (),
        turn_user: str = "",
        turn_credential: str = "",
        on_log: Callable[[str], None] | None = None,
        strict: bool = False,
    ) -> None:
        self._signaling = signaling
        self._stun = list(stun)
        self._turn_urls = list(turn_urls)
        self._turn_user = turn_user
        self._turn_credential = turn_credential
        self._log_cb = on_log
        # strict=True makes motion verbs RAISE instead of commanding into the void.
        # Two silent-drop paths exist by default: _send() drops when the channel is
        # closed, and a robot whose motion stack is down drops control frames with no
        # error (daemon_status.online=False). Both defaults are right for interactive
        # drivers (a UI shows the state) and wrong for an unattended policy, which
        # would otherwise "succeed" against a dead robot. Policies/harnesses set it.
        self._strict = strict

        # --- live state (all read-only to callers, via properties) ---
        self._status = ConnectStatus()
        self._info: RobotInfo | None = None
        self._telemetry: Telemetry | None = None
        self._daemon: DaemonStatus | None = None
        self._layout: CameraLayout | None = None
        self._link_mode: str = ""
        self._perception: Perception | None = None
        self._perception_at: float = 0.0        # monotonic; staleness is the useful reading
        self._policy: PolicyStreamStatus | None = None
        self._record_state: RecordState | None = None
        self._navigation_status: NavigationStatus | None = None
        self._sensor_stream_status: SensorStreamStatus | None = None
        self._lidar_scan: LidarScan | None = None
        self._imu_sample: ImuSample | None = None
        # Last status per action_id, so a fire-and-forget caller can poll instead of awaiting.
        # BOUNDED: a policy issuing thousands of actions must not grow this without limit, and
        # nothing needs the history -- only the most recent verdict per id.
        self._action_states: OrderedDict[str, ActionStatus] = OrderedDict()

        # --- plumbing ---
        self._pc: Any = None  # RTCPeerConnection; typed Any so the core stays import-free
        self._control: Any = None  # RTCDataChannel opened BY THE ROBOT
        self._video_track: Any = None
        self._pending_ice: list[Any] = []
        self._remote_set = False
        self._seq = 0
        self._stopped = False
        self._loop: asyncio.AbstractEventLoop | None = None

        self._ready = asyncio.Event()  # `ack` seen
        self._connected = asyncio.Event()  # peer connection up
        self._listeners: dict[str, list[Callable[[Any], Any]]] = {}
        self._pending_actions: dict[str, asyncio.Future[ActionStatus]] = {}
        self._record_waiters: list[asyncio.Future[RecordState]] = []
        self._policy_waiters: list[asyncio.Future[PolicyStreamStatus]] = []
        self._navigation_waiters: dict[
            str, asyncio.Future[NavigationStatus]
        ] = {}
        self._navigation_goal_waiters: dict[
            str, asyncio.Future[NavigationStatus]
        ] = {}
        self._sensor_stream_waiters: dict[
            str, asyncio.Future[SensorStreamStatus]
        ] = {}
        self._stream_queues: set[asyncio.Queue[Any]] = set()
        self._tasks: set[asyncio.Task[Any]] = set()
        self._jog_payload: dict[str, Any] | None = None
        self._jog_task: asyncio.Task[Any] | None = None

    # --- lifecycle -------------------------------------------------------------------------

    async def __aenter__(self) -> Self:
        await self.start()
        return self

    async def __aexit__(self, *exc: object) -> None:
        await self.stop()

    async def start(self) -> None:
        """Join the signaling room and begin handshaking. Returns as soon as the room is
        wired — await wait_connected() or wait_ready() for an actually usable session."""
        # Checked HERE, not in __init__: building frames and running the mock needs no
        # WebRTC stack (that's what keeps the test suite hardware-free), but starting a
        # session without one would otherwise fail ~20 s later, mid-negotiation, as a raw
        # ModuleNotFoundError inside a transport callback.
        if importlib.util.find_spec("aiortc") is None:
            raise ImportError(
                'a live session needs the "webrtc" extra: pip install "nori-sdk[webrtc]"'
            )
        self._stopped = False
        self._loop = asyncio.get_running_loop()
        self._set_phase("joining")
        await self._signaling.connect(
            SignalingHandlers(
                on_sdp=self._on_sdp,
                on_ice=self._on_ice,
                on_robot_here=self._on_robot_here,
                on_open=self._on_signaling_open,
                on_nack=self._on_nack,
                on_state=self._on_signaling_state,
            )
        )

    async def stop(self) -> None:
        """Tear everything down. Idempotent, and safe to call from a failure path."""
        if self._stopped:
            return
        self._stopped = True
        await self.stop_jog()
        for task in list(self._tasks):
            task.cancel()
        # Wake every stream() consumer: their awaits live in CALLER tasks, which the
        # cancellations above cannot reach. _put_drop_oldest guarantees the sentinel
        # lands even in a full queue a slow consumer may never drain.
        for queue in list(self._stream_queues):
            _put_drop_oldest(queue, _STREAM_CLOSED)
        for request_id, future in list(self._navigation_waiters.items()):
            if not future.done():
                future.set_result(NavigationStatus(
                    state="unavailable",
                    request_id=request_id,
                    error="navigation session closed",
                ))
        self._navigation_waiters.clear()
        for goal_id, future in list(self._navigation_goal_waiters.items()):
            if not future.done():
                future.set_result(NavigationStatus(
                    state="failed",
                    goal_id=goal_id,
                    error="navigation session closed",
                ))
        self._navigation_goal_waiters.clear()
        self._navigation_status = None
        for request_id, future in list(self._sensor_stream_waiters.items()):
            if not future.done():
                future.set_result(SensorStreamStatus(
                    request_id=request_id,
                    error="sensor stream session closed",
                ))
        self._sensor_stream_waiters.clear()
        self._sensor_stream_status = None
        self._lidar_scan = None
        self._imu_sample = None
        with contextlib.suppress(Exception):
            self._signaling.send_bye()
        with contextlib.suppress(Exception):
            await self._signaling.close()
        if self._pc is not None:
            with contextlib.suppress(Exception):
                await self._pc.close()
            self._pc = None
        self._control = None
        self._connected.clear()
        self._ready.clear()
        self._set_phase("closed")

    async def wait_connected(self, timeout: float = ROBOT_WAIT_S + 15.0) -> None:
        """Block until the peer connection is up. Raises TeleopError with a NAMED cause on
        failure — a script should be able to log "robot_absent" vs "ice_failed" without
        parsing prose."""
        await self._wait_or_fail(self._connected, timeout, "connection")

    async def wait_ready(self, timeout: float = ROBOT_WAIT_S + 20.0) -> RobotInfo:
        """Block until the robot's `ack` arrives, i.e. until we know what robot this is.
        Every descriptor-driven helper needs this first."""
        await self._wait_or_fail(self._ready, timeout, "handshake")
        assert self._info is not None
        if not self._info.accepted:
            raise TeleopError(f"robot refused the session: {self._info.error or 'no reason given'}")
        if self._info.version_mismatch:
            self._log(
                f"protocol version mismatch: robot speaks {self._info.protocol_version}, "
                f"this SDK targets {NORI_PROTOCOL_VERSION} - "
                "proceeding; expect vocabulary gaps, not danger"
            )
        return self._info

    async def _wait_or_fail(self, event: asyncio.Event, timeout: float, what: str) -> None:
        try:
            await asyncio.wait_for(event.wait(), timeout)
        except TimeoutError:
            status = self._status
            if status.failure:
                raise TeleopError(
                    f"{what} failed: {status.failure}"
                    + (f" ({status.detail})" if status.detail else "")
                ) from None
            raise TeleopError(
                f"{what} timed out after {timeout:.0f}s in phase {status.phase!r} — "
                "the robot never completed it"
            ) from None
        if self._status.failure:
            raise TeleopError(f"{what} failed: {self._status.failure}")

    # --- read-only state -------------------------------------------------------------------

    @property
    def status(self) -> ConnectStatus:
        """Where the connection attempt is, and why it stopped if it did. `failure` names a
        cause — signaling_unreachable / robot_absent / session_rejected / negotiation_failed /
        ice_failed — so a script can log "my network is broken" separately from "the robot is
        off" without parsing prose."""
        return self._status

    @property
    def info(self) -> RobotInfo | None:
        """The `ack` handshake: descriptor, ranges, watchdog profile. None until ready."""
        return self._info

    @property
    def telemetry(self) -> Telemetry | None:
        """The most recent telemetry frame, with the safety block merged forward (the robot
        sends `status` less often than the frame itself, so a bare frame must not read as
        "safety unknown")."""
        return self._telemetry

    @property
    def daemon_status(self) -> DaemonStatus | None:
        """Robot-side motion health. If this is offline, your control frames are being
        dropped and no amount of jogging will move anything."""
        return self._daemon

    @property
    def camera_layout(self) -> CameraLayout | None:
        """How to slice the single composited video track into per-camera tiles.

        None has TWO meanings and they are not interchangeable: the robot has one camera (so
        the whole frame is that camera), or no valid layout has arrived yet. A malformed
        layout is rejected rather than adopted, so this never goes from good to blank."""
        return self._layout

    def perceive(self) -> Perception | None:
        """The robot's latest world-state from its vision stack, or None if none has arrived.

        None is the common case and does NOT mean "nothing is in front of the robot": the
        detector may simply not be running. Use perception_age to tell a fresh read from a
        stale one — a policy that acts on a 30-second-old detection is acting on fiction."""
        return self._perception

    @property
    def perception_age(self) -> float | None:
        """Seconds since the last `perception` frame, or None if none has arrived.

        Measured on the monotonic clock, not the frame's ts_ns: that timestamp is the ROBOT's
        clock, so differencing it against ours would fold in clock skew."""
        if self._perception is None:
            return None
        return time.monotonic() - self._perception_at

    @property
    def policy_stream_status(self) -> PolicyStreamStatus | None:
        """The last `policy_stream_status` seen, or None if none has arrived.

        POLLING IS THE ONLY WAY to notice a dead stream. There is no unsolicited death
        notification — the robot's streamer can only answer a request, and its end-of-run
        result is discarded rather than pushed. A stream that dies mid-run (sink timeout,
        camera silence) is visible only by calling policy_stream("status")."""
        return self._policy

    @property
    def record_state(self) -> RecordState | None:
        """The last `record_status` seen, or None if no record verb has been issued yet.

        Every reply carries the WHOLE recording state rather than a delta, so this is a
        complete snapshot and a client that missed one frame is not left inconsistent."""
        return self._record_state

    @property
    def navigation_status(self) -> NavigationStatus | None:
        """Latest named-navigation reply or unsolicited lifecycle snapshot."""
        return self._navigation_status

    @property
    def sensor_stream_status(self) -> SensorStreamStatus | None:
        """Latest effective sensor stream settings and publisher presence."""
        return self._sensor_stream_status

    @property
    def lidar_scan(self) -> LidarScan | None:
        """Latest delivered filtered LiDAR scan, or None before subscribing."""
        return self._lidar_scan

    @property
    def imu_sample(self) -> ImuSample | None:
        """Latest delivered IMU sample, or None before subscribing."""
        return self._imu_sample

    def action_status(self, action_id: str) -> ActionStatus | None:
        """The latest verdict for one action_id, or None if none has arrived yet.

        For the fire-and-forget-then-poll shape: `action(..., wait=False)` with your own id,
        then poll. `action(wait=True)` is simpler when you can block. Only the most recent
        ACTION_HISTORY ids are retained, so poll while the action is live rather than
        auditing an old run from this."""
        return self._action_states.get(action_id)

    @staticmethod
    def next_action_id() -> str:
        """A fresh action_id for correlating a fire-and-forget `action` with its status."""
        return uuid.uuid4().hex[:12]

    @property
    def is_connected(self) -> bool:
        """The peer connection is up. NOT the same as "the robot will move": motion can be
        offline behind a perfectly healthy transport — check `daemon_status` for that."""
        return self._connected.is_set()

    # --- outbound: motion ------------------------------------------------------------------

    async def jog(self, payload: dict[str, Any], duration: float = 0.0, hz: float = JOG_HZ) -> None:
        """Send a velocity jog. With `duration`, streams it at `hz` for that long and then
        sends an explicit zero — the shape a script wants. With duration=0 it sends ONE
        frame, which the robot will act on for one watchdog window and then ramp down.

        Streaming is not an optimization: the robot treats silence as "the operator is
        gone" and stops. That is the dead-man behavior, so never try to hold a motion by
        sending a single frame and sleeping."""
        self._require_live("jog")
        if duration <= 0:
            self._send(protocol.control_jog(self._next_seq(), payload))
            return
        deadline = time.monotonic() + duration
        interval = 1.0 / max(1.0, hz)
        while time.monotonic() < deadline and not self._stopped:
            self._send(protocol.control_jog(self._next_seq(), payload))
            await asyncio.sleep(min(interval, max(0.0, deadline - time.monotonic())))
        self._send(protocol.control_jog(self._next_seq(), _zeroed(payload)))

    async def hold(self, ms: float) -> None:
        """Hold position for `ms` milliseconds while keeping the watchdog fed.

        This is the pause primitive a policy needs between commands. Plain
        `asyncio.sleep` is WRONG here: control-frame silence past the watchdog's
        t_stop_ms (500 ms LAN / 1000 ms WAN) puts the robot in safe_hold and DROPS
        all commanded intent — a stale target never self-resumes, so the move you
        commanded before the sleep quietly dies. hold() streams an empty jog at
        JOG_HZ instead: an explicit all-stop for rate motion that leaves latched
        `action` targets in place (the robot pins that: a zero jog must not cancel
        an action), so the arm keeps holding pose while the dead-man clock is fed.

        Like jog(), this owns the repetition for its duration — do not run it
        concurrently with set_jog()."""
        await self.jog({}, duration=ms / 1000.0)

    def _require_live(self, verb: str) -> None:
        """strict-mode gate: motion must never command into the void undetected.

        Raises only in strict mode, and only on the two KNOWN-dead states: the
        session is not connected (frames would be silently dropped client-side) or
        the robot has said its motion stack is down (daemon_status.online=False —
        frames arrive and are silently dropped robot-side). A daemon_status that
        simply hasn't arrived yet (None) passes: the first commands of a session
        race the first status frame, and "unknown" is not "known dead"."""
        if not self._strict:
            return
        if self._stopped or not self.is_connected:
            raise TeleopError(f"{verb}: session is not connected (strict mode)")
        daemon = self._daemon
        if daemon is not None and not daemon.online:
            raise TeleopError(
                f"{verb}: robot motion stack is offline (strict mode): "
                f"{getattr(daemon, 'detail', '') or 'daemon_status.online=false'}"
            )

    def _require_connected(self, verb: str) -> None:
        """strict-mode gate for the BRIDGE-side verbs: policy_stream, record, video, call.

        Connection only — deliberately NOT daemon_status. Those verbs are served by the
        bridge in front of the motion daemon, so they work perfectly well while motion is
        offline: you can poll a running policy stream on a robot whose arms are disabled.
        Gating them on motion health would make strict mode refuse valid operations, which is
        worse than not having the gate."""
        if not self._strict:
            return
        if self._stopped or not self.is_connected:
            raise TeleopError(f"{verb}: session is not connected (strict mode)")

    def set_jog(self, payload: dict[str, Any] | None) -> None:
        """Set the continuously streamed jog — the keyboard-held model.

        THIS SDK DOES THE RESENDING FOR YOU. A background task repeats the payload at JOG_HZ
        until you change it or clear it with set_jog(None), which sends one explicit zero
        frame. You do not need to run your own timer to keep the watchdog fed, and you should
        not: two loops racing means the robot sees whichever won.

        That is worth stating plainly because the rule elsewhere in these docs — "resend
        inside t_warn_ms or the robot stops" — describes the WIRE, not this method. It applies
        to you only if you drive the channel yourself via protocol.control_jog().

        Three entry points, and the difference is who owns the repetition:
            jog(payload, duration=…)  this SDK repeats, for a fixed time, then zeroes
            set_jog(payload)          this SDK repeats, indefinitely, until you clear it
            protocol.control_jog(…)   YOU repeat, inside t_warn_ms, or the robot stops

        Use set_jog for interactive drivers, jog() for scripts."""
        if payload is not None:
            self._require_live("set_jog")
        self._jog_payload = payload
        if payload is None:
            return
        if self._jog_task is None or self._jog_task.done():
            self._jog_task = self._spawn(self._jog_loop())

    async def stop_jog(self) -> None:
        """Stop the streaming jog and send a zero frame."""
        last = self._jog_payload
        self._jog_payload = None
        if self._jog_task is not None:
            self._jog_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._jog_task
            self._jog_task = None
        if last is not None and self._control is not None:
            with contextlib.suppress(Exception):
                self._send(protocol.control_jog(self._next_seq(), _zeroed(last)))

    async def _jog_loop(self) -> None:
        interval = 1.0 / JOG_HZ
        while not self._stopped:
            payload = self._jog_payload
            if payload is None:
                return
            self._send(protocol.control_jog(self._next_seq(), payload))
            await asyncio.sleep(interval)

    async def action(
        self, targets: dict[str, float], wait: bool = False, timeout: float = 10.0
    ) -> ActionStatus | None:
        """Command absolute joint targets ("<motor>.pos" -> value in norm_mode units).

        With wait=True this attaches an action_id and awaits a TERMINAL verdict — done,
        clamped, blocked or timeout. The intermediate "accepted" and "active" frames are not
        it: "accepted" only means the robot validated the target. Check `.succeeded` for
        "reached what I asked for"; `.done` is true for clamped and blocked too.
        In strict mode this raises up front when the session is disconnected or the robot's
        motion stack is offline, instead of timing out against a robot that never heard you.

        The timeout here is the CLIENT's patience. The robot runs its own per-action deadline
        and answers "timeout" when it expires, so a TeleopError from this call means no reply
        arrived at all — a dead channel or an offline motion stack — not a slow move."""
        self._require_live("action")
        if not wait:
            self._send(protocol.control_action(self._next_seq(), targets))
            return None
        action_id = uuid.uuid4().hex[:12]
        future: asyncio.Future[ActionStatus] = asyncio.get_running_loop().create_future()
        self._pending_actions[action_id] = future
        self._send(protocol.control_action(self._next_seq(), targets, action_id))

        # Feed the watchdog while the arm PHYSICALLY TRAVELS. One frame then
        # silence starves the robot's dead-man past t_stop (500 ms LAN / 1 s
        # WAN): it drops the latched target mid-flight and answers "timeout"
        # for any move slower than that — which is MOST real arm moves.
        # Hardware-found 2026-08-22, first live agent0 run: the first slow
        # untuck move died at exactly the WAN t_stop. An empty jog is the
        # robot-pinned keep-alive that cancels nothing (same one hold() uses).
        async def _feed() -> None:
            interval = 1.0 / JOG_HZ
            while True:
                await asyncio.sleep(interval)
                self._send(protocol.control_jog(self._next_seq(), {}))

        feeder = self._spawn(_feed())
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            raise TeleopError(
                f"no action_status for {action_id} within {timeout:.0f}s "
                f"(daemon status: {self._daemon.state if self._daemon else 'unknown'})"
            ) from None
        finally:
            feeder.cancel()
            self._pending_actions.pop(action_id, None)

    async def goto_pose(self, side: str, position_m: list[float],
                        orientation_xyzw: list[float] | None = None,
                        wait: bool = True,
                        timeout: float = 15.0) -> ActionStatus | None:
        """Alias for pose() with await-the-move defaults (wait=True, a
        patience sized for real arm travel). Kept for callers written against
        the pre-0b52e81 name; new code calls pose() directly."""
        return await self.pose(side, position_m, orientation_xyzw,
                               wait=wait, timeout=timeout)

    async def pose(
        self,
        side: str,
        position_m: list[float] | tuple[float, ...],
        orientation_xyzw: list[float] | tuple[float, ...] | None = None,
        wait: bool = False,
        timeout: float = 10.0,
    ) -> ActionStatus | None:
        """Command an absolute Cartesian pose for one arm's gripper TCP (metres, in
        base_footprint). The robot solves IK on-board — the wire never carries joint
        solutions — and tracks the result through the same latch `action` uses: a zero jog
        does not cancel it, the watchdog drops it on control silence.

        Capability-gated: this raises TeleopError when the robot EXPLICITLY does not
        advertise `pose_targets` — a robot without it silently ignores the payload, and
        silently accepting reads as a hung move. An ack predating capabilities entirely
        (supports() is None) is allowed through: probe-and-see is the legacy contract.

        Failure is a modelled reply on the awaited status, not an exception: `blocked`
        with reason "no_ik_solution" (for a FULL pose, not retriable at this lift
        height; position-only failures are wrist-dependent — retry with an explicit
        orientation), "ik_timeout" (worth retrying), "limit:<joint>", "singularity",
        "collision", "lift_moved" (the lift moved — re-send to re-solve at the new
        height), "frame:<name>", "config_jump" (a numerical-IK configuration flip;
        split the move into waypoints), "superseded" (a newer pose or an operator jog
        took the arm), or `timeout` with "ik_no_reply" (solver never answered; retry).
        `reason` is an OPEN string — render unknown values verbatim, never fail on one.
        The intermediate "active" frame means solved-and-tracking; `.done` stays False
        until a terminal state, and `.succeeded` is the "reached it" check.

        With wait=True the empty-jog keep-alive streams for the whole await —
        see the comment in action(); a pose move is exactly the shape that
        starves the dead-man (one frame, then seconds of physical travel)."""
        self._require_live("pose")
        info = self._info
        if info is not None and info.supports("pose_targets") is False:
            raise TeleopError(
                "this robot does not advertise the pose_targets capability — "
                "a pose frame would be silently ignored")
        if not wait:
            self._send(protocol.control_pose(
                self._next_seq(), side, position_m, orientation_xyzw))
            return None
        action_id = uuid.uuid4().hex[:12]
        future: asyncio.Future[ActionStatus] = asyncio.get_running_loop().create_future()
        self._pending_actions[action_id] = future
        self._send(protocol.control_pose(
            self._next_seq(), side, position_m, orientation_xyzw, action_id))

        # Same dead-man arithmetic as action(wait=True): the single pose frame
        # latches a target the arm then takes SECONDS to reach, and silence
        # past t_stop (500 ms LAN / 1 s WAN) makes the watchdog drop it
        # mid-flight — the move dies as a phantom "timeout". The empty jog is
        # the robot-pinned keep-alive that commands nothing and, per the
        # gateway contract, does not cancel a pose latch.
        async def _feed() -> None:
            interval = 1.0 / JOG_HZ
            while True:
                await asyncio.sleep(interval)
                self._send(protocol.control_jog(self._next_seq(), {}))

        feeder = self._spawn(_feed())
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            raise TeleopError(
                f"no action_status for pose {action_id} within {timeout:.0f}s "
                f"(daemon status: {self._daemon.state if self._daemon else 'unknown'})"
            ) from None
        finally:
            feeder.cancel()
            self._pending_actions.pop(action_id, None)

    def estop(self) -> None:
        """Latch E-STOP. Motion stays blocked until reset_latch(). Deliberately synchronous
        and un-awaited: it must not be able to block behind anything.

        Raises TeleopError in EVERY mode — not just strict — when the frame could not be
        handed to an open channel. Ordinary verbs drop silently on a dead channel because
        the watchdog makes the drop meaningless; an E-STOP that went nowhere is the one
        drop a caller must not be able to mistake for success — they need to reach for
        the physical button instead. The check is a local readyState read, so this still
        cannot block. Delivery is not execution: the channel is lossy and the robot drops
        command frames without reply while its motion stack is down — an unattended caller
        should use estop_confirmed()."""
        if not self._send(protocol.command("estop")):
            raise TeleopError(
                "estop: control channel is not open — the frame went NOWHERE. This session "
                "cannot stop the robot; use the physical E-STOP or the robot's face button."
            )

    async def estop_confirmed(self, timeout: float = 5.0) -> None:
        """estop(), then await the robot REPORTING the latch in telemetry.

        Closes the two drop paths estop() cannot see: the channel is unreliable by design
        (a sent frame can still vanish in flight) and the robot drops command frames with
        no reply while its motion stack is down. Confirmation is OBSERVED STATE, not an
        ack — the safety block rides telemetry at ~5 Hz, so a real latch is visible well
        inside the default timeout. Raises TeleopError when no latch is seen in time, and
        the only safe reading of that is "the robot is NOT stopped".

        Only a report observed AFTER the send counts. The cached merged frame is
        deliberately not consulted: _merge_telemetry carries the safety block forward, so
        a stale "latched" from minutes ago — telemetry stalled, latch since cleared at the
        robot — would confirm an estop that went nowhere."""
        latched: asyncio.Future[None] = asyncio.get_running_loop().create_future()

        def _check(frame: Any) -> None:
            if getattr(frame, "safety", None) == "latched" and not latched.done():
                latched.set_result(None)

        # Subscribed BEFORE the send, so a fast robot cannot reply into the gap.
        unsubscribe = self.on("telemetry", _check)
        try:
            self.estop()
            await asyncio.wait_for(latched, timeout)
        except TimeoutError:
            raise TeleopError(
                f"estop sent but the robot never reported the latch within {timeout:.1f}s — "
                "assume it is NOT stopped (motion stack down, or the frame was lost)"
            ) from None
        finally:
            unsubscribe()

    def reset_latch(self) -> None:
        """Clear a latched E-STOP, re-enabling motion. The counterpart to estop().

        Only a LATCH needs this. `safe_hold` — the watchdog's response to control-frame
        silence — clears itself the moment frames resume, so calling this for a safe_hold is
        unnecessary. Check `telemetry.safety` to tell them apart."""
        self._send(protocol.command("reset_latch"))

    def reset_arm(self, arm: str) -> None:
        """Clear one arm's latch/target state ("left_arm" / "right_arm").

        Narrower than reset_latch(): this clears a per-arm stall or target, not a session-wide
        E-STOP. Note it rides the `control` frame rather than `command`."""
        self._send(protocol.control_reset(arm))

    # --- outbound: video / recording -------------------------------------------------------

    def set_video_bitrate(self, kbps: int) -> None:
        """Ask the robot's encoder for a target bitrate. It applies its own thermal/load
        ceiling on top, so this is a request, not a setting."""
        self._send(protocol.video_bitrate(kbps))

    def set_video_paused(self, paused: bool) -> None:
        """Pause or resume the robot's video encoder, to save power and bandwidth.

        The robot defaults to flowing, so only send this when you want it paused. Video is
        independent of control: pausing it does not affect motion, telemetry or the watchdog."""
        self._send(protocol.video_state(paused))

    async def record(
        self, action: protocol.RecordVerb, task: str = "", timeout: float = 10.0
    ) -> RecordState:
        """Issue one dataset-recording verb and await its `record_status`.

        Replies carry no correlation id, so this resolves the NEXT status frame — do not
        issue record verbs concurrently from two tasks."""
        future: asyncio.Future[RecordState] = asyncio.get_running_loop().create_future()
        self._record_waiters.append(future)
        self._send(protocol.record(action, task))
        try:
            state = await asyncio.wait_for(future, timeout)
        except TimeoutError:
            with contextlib.suppress(ValueError):
                self._record_waiters.remove(future)
            raise TeleopError(f"no record_status for {action!r} within {timeout:.0f}s") from None
        if not state.ok:
            raise TeleopError(f"record {action!r} refused: {state.error or 'no reason given'}")
        return state

    async def _navigation_request(
        self,
        action: protocol.NavigationAction,
        *,
        name: str = "",
        goal_id: str = "",
        timeout: float = 5.0,
    ) -> NavigationStatus:
        if self._info is not None and self._info.supports("named_navigation") is False:
            raise TeleopError(
                "this robot does not advertise the named_navigation capability"
            )
        request_id = str(uuid.uuid4())
        future: asyncio.Future[NavigationStatus] = (
            asyncio.get_running_loop().create_future()
        )
        self._navigation_waiters[request_id] = future
        frame = protocol.navigation(
            action,
            request_id,
            name=name,
            goal_id=goal_id,
        )
        sent = self._send(frame)
        if not sent:
            self._navigation_waiters.pop(request_id, None)
            return NavigationStatus(
                state="unavailable",
                request_id=request_id,
                goal_id=goal_id or None,
                name=name or None,
                error="navigation control channel is not open",
            )
        async def retry() -> None:
            while not future.done():
                await asyncio.sleep(0.75)
                if not future.done():
                    self._send(frame)

        retry_task = asyncio.create_task(retry())
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            self._navigation_waiters.pop(request_id, None)
            return NavigationStatus(
                state="unavailable",
                request_id=request_id,
                goal_id=goal_id or None,
                name=name or None,
                error=f"navigation {action}: no reply in {timeout:.1f}s",
            )
        finally:
            retry_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await retry_task

    async def list_waypoints(self, timeout: float = 5.0) -> NavigationStatus:
        """List named destinations bound to the robot's active saved map."""
        return await self._navigation_request("list_waypoints", timeout=timeout)

    async def remember_waypoint(
        self, name: str, timeout: float = 5.0
    ) -> NavigationStatus:
        """Save the robot's current localized pose under ``name``."""
        return await self._navigation_request(
            "remember_waypoint", name=name, timeout=timeout
        )

    async def delete_waypoint(
        self, name: str, timeout: float = 5.0
    ) -> NavigationStatus:
        """Delete ``name`` on the active map; refuses during navigation."""
        return await self._navigation_request(
            "delete_waypoint", name=name, timeout=timeout
        )

    async def navigate_to_waypoint(
        self, name: str, timeout: float = 5.0
    ) -> NavigationStatus:
        """Start one map-matched Nav2 goal to a remembered destination."""
        return await self._navigation_request(
            "start",
            name=name,
            goal_id=str(uuid.uuid4()),
            timeout=timeout,
        )

    async def cancel_navigation(
        self, goal_id: str = "", timeout: float = 5.0
    ) -> NavigationStatus:
        """Cancel this SDK session's goal, optionally requiring ``goal_id``."""
        return await self._navigation_request(
            "cancel", goal_id=goal_id, timeout=timeout
        )

    async def get_navigation_status(
        self, timeout: float = 5.0
    ) -> NavigationStatus:
        """Read the robot's latest global named-navigation snapshot."""
        return await self._navigation_request("status", timeout=timeout)

    async def await_navigation(
        self, goal_id: str, timeout: float = 120.0
    ) -> NavigationStatus:
        """Wait for a terminal snapshot for ``goal_id`` without polling."""
        current = self._navigation_status
        if (
            current is not None
            and current.goal_id == goal_id
            and current.state in TERMINAL_NAVIGATION_STATES
        ):
            return current
        previous = self._navigation_goal_waiters.pop(goal_id, None)
        if previous is not None and not previous.done():
            previous.set_result(NavigationStatus(
                state="failed",
                goal_id=goal_id,
                error="await_navigation replaced by a newer waiter for this goal",
            ))
        future: asyncio.Future[NavigationStatus] = (
            asyncio.get_running_loop().create_future()
        )
        self._navigation_goal_waiters[goal_id] = future
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            self._navigation_goal_waiters.pop(goal_id, None)
            return NavigationStatus(
                state="failed",
                goal_id=goal_id,
                error=f"navigation goal did not finish in {timeout:.1f}s",
            )

    async def _sensor_stream_request(
        self,
        action: protocol.SensorStreamAction,
        *,
        lidar_hz: float | None = None,
        imu_hz: float | None = None,
        lidar_max_points: int | None = None,
        timeout: float = 5.0,
    ) -> SensorStreamStatus:
        if self._info is not None and self._info.supports("sensor_streams") is False:
            raise TeleopError(
                "this robot does not advertise the sensor_streams capability"
            )
        def valid_rate(value: object, maximum: float) -> bool:
            return (
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                and 0.0 <= float(value) <= maximum
            )

        if lidar_hz is not None and not valid_rate(lidar_hz, 10.0):
            raise ValueError("lidar_hz must be between 0 and 10")
        if imu_hz is not None and not valid_rate(imu_hz, 50.0):
            raise ValueError("imu_hz must be between 0 and 50")
        if lidar_max_points is not None and (
            isinstance(lidar_max_points, bool)
            or not isinstance(lidar_max_points, int)
            or not 16 <= lidar_max_points <= 1440
        ):
            raise ValueError(
                "lidar_max_points must be an integer between 16 and 1440"
            )
        if (
            action == "configure"
            and lidar_hz is None
            and imu_hz is None
            and lidar_max_points is None
        ):
            raise ValueError(
                "configure_sensor_streams requires at least one setting"
            )
        request_id = str(uuid.uuid4())
        frame = protocol.sensor_stream(
            action,
            request_id,
            lidar_hz=lidar_hz,
            imu_hz=imu_hz,
            lidar_max_points=lidar_max_points,
        )
        future: asyncio.Future[SensorStreamStatus] = (
            asyncio.get_running_loop().create_future()
        )
        self._sensor_stream_waiters[request_id] = future
        if not self._send(frame):
            self._sensor_stream_waiters.pop(request_id, None)
            return SensorStreamStatus(
                request_id=request_id,
                error="sensor stream control channel is not open",
            )

        async def retry() -> None:
            while not future.done():
                await asyncio.sleep(0.75)
                if not future.done():
                    self._send(frame)

        retry_task = asyncio.create_task(retry())
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            self._sensor_stream_waiters.pop(request_id, None)
            current = self._sensor_stream_status
            return SensorStreamStatus(
                request_id=request_id,
                lidar_hz=current.lidar_hz if current else 0.0,
                imu_hz=current.imu_hz if current else 0.0,
                lidar_max_points=current.lidar_max_points if current else 360,
                lidar_available=current.lidar_available if current else False,
                imu_available=current.imu_available if current else False,
                error=f"sensor_stream {action}: no reply in {timeout:.1f}s",
            )
        finally:
            retry_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await retry_task

    async def configure_sensor_streams(
        self,
        *,
        lidar_hz: float | None = None,
        imu_hz: float | None = None,
        lidar_max_points: int | None = None,
        timeout: float = 5.0,
    ) -> SensorStreamStatus:
        """Configure opt-in sensor delivery; zero disables a stream.

        Omitted settings retain their current robot-side values. The robot caps
        LiDAR at 10 Hz, IMU at 50 Hz and LiDAR payloads at 1440 readings.
        """
        return await self._sensor_stream_request(
            "configure",
            lidar_hz=lidar_hz,
            imu_hz=imu_hz,
            lidar_max_points=lidar_max_points,
            timeout=timeout,
        )

    async def get_sensor_stream_status(
        self, timeout: float = 5.0
    ) -> SensorStreamStatus:
        """Read effective stream settings and current ROS publisher presence."""
        return await self._sensor_stream_request("status", timeout=timeout)

    # --- outbound: policy stream / leader / call --------------------------------------------

    async def policy_stream(
        self, action: str, timeout: float = 10.0, **extra: Any
    ) -> PolicyStreamStatus:
        """Drive the robot's policy streamer and await its reply.

            await robot.policy_stream("start", dest="laptop")
            status = await robot.policy_stream("status")     # the ONLY way to check liveness

        Replies carry no correlation id, so this resolves the NEXT status frame — do not issue
        policy_stream verbs concurrently from two tasks.

        There is NO unsolicited death notification: the streamer can only answer a request and
        its end-of-run result is discarded rather than pushed. A stream that dies mid-run (sink
        timeout, camera silence) is observable only by polling "status". Do not build a loop
        that waits for a failure frame — it never arrives.

        Unlike record(), an `ok: false` reply is RETURNED rather than raised: for this verb a
        refusal is ordinary state a caller inspects (a stream that is not running answers
        ok:false to "status" routinely), not an error."""
        self._require_connected(f"policy_stream({action!r})")
        future: asyncio.Future[PolicyStreamStatus] = (
            asyncio.get_running_loop().create_future()
        )
        self._policy_waiters.append(future)
        self._send(protocol.policy_stream(action, **extra))
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            with contextlib.suppress(ValueError):
                self._policy_waiters.remove(future)
            raise TeleopError(
                f"no policy_stream_status for {action!r} within {timeout:.0f}s"
            ) from None

    def set_leader_action(self, targets: dict[str, float]) -> None:
        """Absolute pose from a physical leader arm — ONE frame, not a stream.

        Keys are flat "<side>_arm_<joint>.pos"; body joints are DEGREES around the calibrated
        leader zero, grippers normalized [0, 100]. The robot calibration-normalizes, runs IK
        and applies its own slew clamp.

        Gated on the `leader_action_deg` capability: a robot that does not advertise it drops
        these frames silently, and the 5-DOF L-series key set does not map onto a 7-DOF arm at
        all. Check `info.supports("leader_action_deg")` first.

        Like a jog this is level-triggered — the robot acts on the latest frame and the
        watchdog still applies, so send it at your control rate, not once."""
        self._require_live("set_leader_action")
        self._send(protocol.control_leader(self._next_seq(), targets))

    def set_video_quality(self, quality: str) -> None:
        """Named encoder preset ("low" | "normal"). Coarser than set_video_bitrate and
        intercepted by the robot's bridge before it reaches the motion daemon."""
        self._send(protocol.video_quality(quality))

    def call(self, *, state: str | None = None, mic_muted: bool | None = None) -> None:
        """Two-way audio control: join/leave the call, and mute/unmute our microphone.

        This SDK sends the verb and nothing more. Unlike the browser client there is no local
        audio device handling here — no capture, no playback, no echo cancellation — so on its
        own this changes the robot's view of the call without giving you an audio path. Wire
        your own aiortc audio track if you need one."""
        self._send(protocol.call(state=state, mic_muted=mic_muted))

    # --- video frames ----------------------------------------------------------------------

    async def frames(self, track_timeout: float = ROBOT_WAIT_S) -> AsyncIterator[Any]:
        """Yield decoded video frames (av.VideoFrame) from the composite track.

        One H.264 track carries every camera as tiles; use camera_layout.rect(role) to crop
        one. Await wait_connected() first. Raises TeleopError when no track arrives within
        `track_timeout`: a session is perfectly healthy with video down (control and
        telemetry are independent of it), and an unattended caller needs that as a NAMED
        failure, not an infinite poll — snapshot()/snapshot_png() inherit the same raise."""
        deadline = time.monotonic() + track_timeout
        while self._video_track is None and not self._stopped:
            if time.monotonic() >= deadline:
                raise TeleopError(
                    f"no video track within {track_timeout:.0f}s — is the robot's video "
                    "pipeline up? (motion and telemetry work without it)"
                )
            await asyncio.sleep(0.1)
        track = self._video_track
        while track is not None and not self._stopped:
            try:
                yield await track.recv()
            except Exception:  # track ended (robot restart / teardown)
                return

    async def snapshot(
        self, role: str | None = None, settle: float = 0.3,
        track_timeout: float = ROBOT_WAIT_S,
    ) -> Any:
        """One frame, optionally cropped to a camera role. `settle` discards frames first so
        an auto-exposing camera isn't captured mid-adjust. `track_timeout` bounds the wait
        for the video track itself and raises the named no-track error — see frames()."""
        deadline = time.monotonic() + settle
        frame = None
        async for frame in self.frames(track_timeout=track_timeout):
            if time.monotonic() >= deadline:
                break
        if frame is None:
            raise TeleopError("no video frame available")
        if role is None or self._layout is None:
            return frame
        rect = self._layout.rect(role)
        if rect is None:
            raise TeleopError(f"this robot has no camera {role!r}")
        image = frame.to_ndarray(format="rgb24")
        height, width = image.shape[:2]
        x, y, w, h = rect
        return image[
            int(y * height) : int((y + h) * height), int(x * width) : int((x + w) * width)
        ]

    async def snapshot_png(
        self, role: str | None = None, settle: float = 0.3,
        track_timeout: float = ROBOT_WAIT_S,
    ) -> bytes:
        """One frame as PNG file bytes — the shape a trial-artifact writer wants.

        Same semantics as snapshot(); the whole composite is encoded when `role`
        is None or the robot has no layout. Save with a lowercase `.png` suffix —
        downstream consumers map media type from the extension. Prefer per-role
        crops for VLM consumption (a full composite is bigger than any judgment
        needs; API-side image caps are per-image)."""
        from . import _png

        frame = await self.snapshot(role=role, settle=settle, track_timeout=track_timeout)
        image = frame if hasattr(frame, "shape") else frame.to_ndarray(format="rgb24")
        return _png.encode_rgb24(image)

    # --- events ----------------------------------------------------------------------------

    def on(self, kind: str, callback: Callable[[Any], Any]) -> Callable[[], None]:
        """Subscribe to an inbound frame kind ("telemetry", "daemon_status", "perception",
        ...). Returns an unsubscribe callable. Callbacks may be sync or async; an exception
        in one is logged and swallowed so a bad listener can't kill the session."""
        self._listeners.setdefault(kind, []).append(callback)

        def unsubscribe() -> None:
            with contextlib.suppress(ValueError):
                self._listeners[kind].remove(callback)

        return unsubscribe

    async def stream(self, kind: str, maxsize: int = 16) -> AsyncIterator[Any]:
        """Async-iterate one frame kind. The queue drops the OLDEST frame when a slow
        consumer falls behind — for telemetry, stale data is worse than missing data.

        Ends when the session stops. That needs a sentinel rather than a flag check:
        the consumer's `await` lives in the CALLER's task, which stop() cannot cancel,
        so without the wake-up a loop parked on an idle stream would sleep forever."""
        queue: asyncio.Queue[Any] = asyncio.Queue(maxsize=maxsize)
        unsubscribe = self.on(kind, lambda value: _put_drop_oldest(queue, value))
        self._stream_queues.add(queue)
        try:
            while not self._stopped:
                value = await queue.get()
                if value is _STREAM_CLOSED:
                    return
                yield value
        finally:
            self._stream_queues.discard(queue)
            unsubscribe()

    def _emit(self, kind: str, value: Any) -> None:
        for callback in list(self._listeners.get(kind, ())):
            try:
                result = callback(value)
                if asyncio.iscoroutine(result):
                    self._spawn(result)
            except Exception:
                log.exception("listener for %s raised", kind)

    # --- signaling handlers ----------------------------------------------------------------

    def _on_signaling_open(self) -> None:
        self._log("signaling room open — announcing 'ready'")
        self._send_ready()
        # Only (re)enter `waiting` from a pre-connection phase: on_open also fires on a
        # mid-session signaling reconnect, and that must not knock a live session back.
        if self._status.phase in ("joining", "failed"):
            self._set_phase("waiting")
            self._spawn(self._wait_deadline())
        self._spawn(self._ready_retry_loop())

    def _on_signaling_state(self, state: SignalingState) -> None:
        if state in ("error", "timeout"):
            if self._connected.is_set():
                return  # a live session rides out a signaling blip; media is peer-to-peer
            self._set_phase("failed", "signaling_unreachable", state)

    def _on_robot_here(self, _payload: dict[str, Any]) -> None:
        # Do NOT clear _connected here. The gateway broadcasts robot_here on EVERY room
        # join — including its own signaling auto-reconnect mid-session — and it ignores
        # re-readys while a session exists, so clearing would mark a healthy peer
        # connection disconnected FOREVER (strict mode then refuses every verb — fatal
        # to an unattended run). Like the two handlers above, a live session rides it
        # out: _connected is owned by the connection-state callback alone.
        #
        # The ready is still sent, and unconditionally on purpose — NOT the sibling
        # early-return-when-connected guard: a gateway that already has us ignores
        # re-readys, and a gateway that RESTARTED (no session) needs one to offer. Its
        # robot_here usually beats aiortc noticing the dead peer, and nothing else
        # re-sends ready after that (_ready_retry_loop exits for good on first connect),
        # so gating the send on !connected would stall restart recovery indefinitely.
        self._log("robot announced — sending 'ready'")
        self._send_ready()

    def _on_nack(self, payload: NackPayload) -> None:
        if self._connected.is_set():
            return  # a live session ignores late/stray nacks
        if payload.reason and payload.reason != "unauthorized":
            self._log(f"robot refused the session: {payload.reason}")
            self._set_phase("failed", "session_rejected", payload.reason)
            return
        # Room-token auth is retired (the robot gates access via RLS), so a reasonless or
        # "unauthorized" nack is a stray/forged artifact rather than a wrong access code.
        # Keep retrying: an authorized operator in the room gets an offer regardless.
        self._log("ignoring unauthorized nack (room-token auth is retired)")

    async def _on_sdp(self, payload: SdpPayload) -> None:
        if payload.type != "offer":
            return  # we are the answerer; an answer arriving here is someone else's traffic
        self._set_phase("negotiating")
        try:
            from aiortc import RTCSessionDescription

            self._log("offer received; building fresh peer and answering")
            pc = await self._fresh_peer()
            # webrtcbin interop: a fresh gateway can offer H264 without its fmtp
            # line, which aiortc mis-defaults to packetization-mode=0 and then
            # rejects outright (hardware-confirmed 2026-08-21; see webrtc_compat).
            await pc.setRemoteDescription(
                RTCSessionDescription(
                    sdp=webrtc_compat.ensure_h264_fmtp(payload.sdp), type="offer"
                )
            )
            self._remote_set = True
            for candidate in self._pending_ice:
                with contextlib.suppress(Exception):
                    await pc.addIceCandidate(candidate)
            self._pending_ice.clear()
            answer = await pc.createAnswer()
            # aiortc completes ICE gathering inside setLocalDescription, so by the time
            # this returns our candidates are already in the SDP...
            await pc.setLocalDescription(answer)
            self._signaling.send_sdp(
                SdpPayload(type="answer", sdp=pc.localDescription.sdp)
            )
            # ...but webrtcbin IGNORES in-SDP candidates — it only consumes ones
            # delivered via the signaling `ice` event. Without this trickle the
            # robot never learns a single operator candidate and ICE fails
            # (hardware-confirmed 2026-08-21).
            for mline, cand in webrtc_compat.local_candidates(pc.localDescription.sdp):
                self._signaling.send_ice(
                    IcePayload(candidate=cand, sdp_mline_index=mline)
                )
            self._log("answer sent")
        except Exception as e:
            # This runs inside a transport callback, where a raise would become an
            # unhandled task exception and the session would just silently stop.
            self._log(f"negotiation failed: {e}")
            self._set_phase("failed", "negotiation_failed", str(e))

    async def _on_ice(self, payload: IcePayload) -> None:
        try:
            from aiortc.sdp import candidate_from_sdp

            candidate = candidate_from_sdp(payload.candidate.replace("candidate:", "", 1))
            candidate.sdpMLineIndex = payload.sdp_mline_index
            candidate.sdpMid = payload.sdp_mid
        except Exception as e:
            self._log(f"ignoring unparseable ICE candidate: {e}")
            return
        if self._pc is not None and self._remote_set:
            with contextlib.suppress(Exception):
                await self._pc.addIceCandidate(candidate)
        else:
            # Candidates can beat the offer; hold them until there's a remote description.
            self._pending_ice.append(candidate)

    def _send_ready(self) -> None:
        # Forward this session's TURN creds so the ROBOT can gather relay candidates too.
        # Without them a host-only robot is unreachable through a relay: the relay can't
        # route to LAN addresses, and coturn drops the robot's inbound checks because its
        # public address was never signaled, so no permission exists.
        turn = (
            ReadyTurn(
                urls=self._turn_urls, username=self._turn_user, credential=self._turn_credential
            )
            if self._turn_urls
            else None
        )
        with contextlib.suppress(Exception):
            self._signaling.send_ready(turn)

    async def _ready_retry_loop(self) -> None:
        while not self._stopped and not self._connected.is_set():
            await asyncio.sleep(RETRY_S)
            if self._stopped or self._connected.is_set():
                return
            self._send_ready()

    async def _wait_deadline(self) -> None:
        await asyncio.sleep(ROBOT_WAIT_S)
        if not self._stopped and not self._connected.is_set() and self._status.phase == "waiting":
            self._set_phase(
                "failed",
                "robot_absent",
                f"no offer within {ROBOT_WAIT_S:.0f}s — the room is live but the robot "
                "never answered (is it powered and online?)",
            )

    # --- peer connection -------------------------------------------------------------------

    async def _fresh_peer(self) -> Any:
        """A NEW RTCPeerConnection per offer. The robot restarts its pipeline on every
        session, so reusing a peer across offers leaves stale transceivers behind."""
        from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection

        # webrtcbin interop: GStreamer's dtls cert is RSA; aiortc's default cipher
        # list is ECDSA-only and the handshake dies instantly without this
        # (hardware-confirmed 2026-08-21; see webrtc_compat). Idempotent.
        webrtc_compat.widen_dtls_ciphers()

        if self._pc is not None:
            with contextlib.suppress(Exception):
                await self._pc.close()
        servers = [RTCIceServer(urls=list(self._stun))] if self._stun else []
        if self._turn_urls:
            servers.append(
                RTCIceServer(
                    urls=list(self._turn_urls),
                    username=self._turn_user,
                    credential=self._turn_credential,
                )
            )
        pc = RTCPeerConnection(RTCConfiguration(iceServers=servers))
        self._pc = pc
        self._remote_set = False

        @pc.on("datachannel")
        def _on_datachannel(channel: Any) -> None:
            # The ROBOT opens the control channel; we never create it.
            self._setup_control(channel)

        @pc.on("track")
        def _on_track(track: Any) -> None:
            self._log(f"{track.kind} track received")
            if track.kind == "video":
                self._video_track = track

        @pc.on("connectionstatechange")
        def _on_state() -> None:
            self._spawn(self._handle_conn_state(pc.connectionState))

        return pc

    async def _handle_conn_state(self, state: str) -> None:
        self._log(f"connection: {state}")
        if state == "connected":
            self._connected.set()
            self._set_phase("connected")
            await self._detect_link_mode()
        elif state in ("failed", "closed"):
            self._connected.clear()
            if not self._stopped and state == "failed":
                # "failed" = ICE found no working path (NAT/firewall/TURN): a real fault,
                # unlike "disconnected", which is often a blip that heals itself.
                self._set_phase("failed", "ice_failed")

    async def _detect_link_mode(self) -> None:
        """Report the resolved network path so the robot picks its watchdog profile. Both
        nominated candidates 'host' on non-tunnel addresses means a direct LAN link;
        anything else (STUN srflx, TURN relay, a VPN/overlay address) is WAN, which buys
        a looser dead-man window — the safe direction.

        Read from aioice's nominated pairs, NOT getStats: aiortc implements no
        candidate-pair stats at all, so the standards-shaped getStats loop this replaces
        matched nothing and answered "wan" unconditionally (hardware-found 2026-08-26,
        SDK 1.0 bench). The walk touches private attributes and is wrapped accordingly:
        an aiortc upgrade that breaks it degrades to "wan", never to a false "lan"."""
        mode = "wan"
        try:
            ice = self._pc.sctp.transport.transport  # sctp -> dtls -> ice transport
            for pair in ice._connection._nominated.values():  # aioice internals
                local, remote = pair.local_candidate, pair.remote_candidate
                types = {getattr(local, "type", ""), getattr(remote, "type", "")}
                hosts = (getattr(local, "host", ""), getattr(remote, "host", ""))
                if types == {"host"} and not any(_tunnel_address(h) for h in hosts):
                    mode = "lan"
                break
        except Exception:
            pass  # best-effort; defaulting to "wan" is the safe direction
        self._link_mode = mode
        if self._send(protocol.link(mode)):
            self._log(f"link -> {mode}")
        else:
            # Connection state can beat the datachannel event; the stored mode is
            # re-sent by _setup_control the moment the robot's channel arrives.
            self._log(f"link resolved -> {mode} (channel not up yet; sent on channel open)")

    def _setup_control(self, channel: Any) -> None:
        self._control = channel

        def _announce_open() -> None:
            self._log("control channel open")
            if self._link_mode:
                self._send(protocol.link(self._link_mode))

        @channel.on("open")
        def _on_open() -> None:
            _announce_open()

        @channel.on("close")
        def _on_close() -> None:
            if self._control is channel:
                self._control = None

        @channel.on("message")
        def _on_message(raw: Any) -> None:
            self._handle_frame(raw)

        # The ROBOT opens this channel, so aiortc hands it to us already open — its "open"
        # event fired before we could subscribe and will never fire again. Run the open
        # logic now, or the link-mode resend above is dead code: when _detect_link_mode
        # raced ahead of the channel its send dropped silently, and this is the only retry.
        if getattr(channel, "readyState", "") == "open":
            _announce_open()

    # --- inbound ---------------------------------------------------------------------------

    def _handle_frame(self, raw: str | bytes) -> None:
        kind, parsed, obj = protocol.decode(raw)
        if not kind:
            return
        if kind == "ack" and isinstance(parsed, RobotInfo):
            self._info = parsed
            self._ready.set()
            joints = len(parsed.descriptor.joints) if parsed.descriptor else 0
            self._log(f"ack: accepted={parsed.accepted} joints={joints}")
        elif kind == "telemetry" and isinstance(parsed, Telemetry):
            self._telemetry = _merge_telemetry(self._telemetry, parsed)
            parsed = self._telemetry
        elif kind == "camera_layout" and isinstance(parsed, CameraLayout):
            # Sent repeatedly because the channel is unreliable; treat repeats as idempotent.
            self._layout = parsed
        elif kind == "daemon_status":
            if not isinstance(parsed, DaemonStatus):
                # Stateless frame: the spec says drop it and keep the health we have. Return
                # rather than fall through, so subscribers see no event at all instead of an
                # untyped raw dict where every other daemon_status hands them a DaemonStatus.
                return
            if self._daemon is None or self._daemon.state != parsed.state:
                self._log(f"daemon: {parsed.state}" + (f" ({parsed.detail})" if parsed.detail else ""))
            self._daemon = parsed
        elif kind == "action_status" and isinstance(parsed, ActionStatus):
            # Resolve ONLY on a terminal state. "accepted"/"active" are progress reports, and
            # completing the future on the first of them is what made action(wait=True) return
            # before the robot had moved -- reporting success while the watchdog was still
            # free to abort the move. Non-terminal frames still reach subscribers via _emit.
            # Remembered so a fire-and-forget caller can poll action_status(id) instead of
            # awaiting. Bounded LRU: a policy issuing thousands of actions must not grow this
            # forever, and only the latest verdict per id is ever useful.
            self._action_states[parsed.action_id] = parsed
            self._action_states.move_to_end(parsed.action_id)
            while len(self._action_states) > ACTION_HISTORY:
                self._action_states.popitem(last=False)
            action_future = self._pending_actions.get(parsed.action_id)
            if action_future is not None and parsed.done and not action_future.done():
                action_future.set_result(parsed)
        elif kind == "record_status" and isinstance(parsed, RecordState):
            self._record_state = parsed
            if self._record_waiters:
                record_future = self._record_waiters.pop(0)
                if not record_future.done():
                    record_future.set_result(parsed)
        elif kind == "perception" and isinstance(parsed, Perception):
            # Monotonic, not the frame's ts_ns: that is the ROBOT's clock, so differencing it
            # against ours would fold in clock skew and report nonsense staleness.
            self._perception = parsed
            self._perception_at = time.monotonic()
        elif kind == "policy_stream_status" and isinstance(parsed, PolicyStreamStatus):
            self._policy = parsed
            if self._policy_waiters:
                policy_future = self._policy_waiters.pop(0)
                if not policy_future.done():
                    policy_future.set_result(parsed)
        elif kind == "navigation_status" and isinstance(parsed, NavigationStatus):
            if parsed.request_id:
                request_future = self._navigation_waiters.pop(
                    parsed.request_id, None
                )
                if request_future is not None and not request_future.done():
                    request_future.set_result(parsed)
            previous = self._navigation_status
            stale_regression = bool(
                previous is not None
                and previous.goal_id
                and previous.goal_id == parsed.goal_id
                and previous.state in TERMINAL_NAVIGATION_STATES
                and parsed.state not in TERMINAL_NAVIGATION_STATES
            )
            if not stale_regression:
                self._navigation_status = parsed
            if parsed.goal_id and parsed.state in TERMINAL_NAVIGATION_STATES:
                goal_future = self._navigation_goal_waiters.pop(
                    parsed.goal_id, None
                )
                if goal_future is not None and not goal_future.done():
                    goal_future.set_result(parsed)
            if stale_regression:
                return
        elif kind == "sensor_stream_status" and isinstance(
            parsed, SensorStreamStatus
        ):
            self._sensor_stream_status = parsed
            future = self._sensor_stream_waiters.pop(parsed.request_id, None)
            if future is not None and not future.done():
                future.set_result(parsed)
        elif kind == "lidar_scan" and isinstance(parsed, LidarScan):
            self._lidar_scan = parsed
        elif kind == "imu" and isinstance(parsed, ImuSample):
            self._imu_sample = parsed
        self._emit(kind, parsed if parsed is not None else obj)

    # --- internals -------------------------------------------------------------------------

    def _send(self, frame: dict[str, Any]) -> bool:
        """True when the frame was handed to an open channel, False when it was dropped.

        Dropping is correct: the control channel is unreliable by design and the robot is
        watchdogged, so a frame sent into a dead channel has no meaning to preserve. But a
        caller must not claim delivery it didn't get — log on the return value, not on
        having called this."""
        channel = self._control
        if channel is None or getattr(channel, "readyState", "") != "open":
            return False
        try:
            channel.send(protocol.encode(frame))
        except Exception:
            return False
        return True

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _set_phase(self, phase: Any, failure: str | None = None, detail: str | None = None) -> None:
        self._status = ConnectStatus(phase=phase, failure=failure, detail=detail)
        if failure:
            self._log(f"phase {phase}: {failure}" + (f" ({detail})" if detail else ""))
        self._emit("status", self._status)

    def _spawn(self, coro: Any) -> asyncio.Task[Any]:
        task: asyncio.Task[Any] = asyncio.ensure_future(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task

    def _log(self, message: str) -> None:
        log.info("%s", message)
        if self._log_cb is not None:
            with contextlib.suppress(Exception):
                self._log_cb(message)


def _tunnel_address(host: str) -> bool:
    """True for addresses that mean a VPN/overlay carried the candidate even though its
    ICE type is "host". A "lan" verdict over a tunnel hands the robot the tight watchdog
    profile on a path with tunnel latency and a 1280-byte MTU — the exact pairing that
    silently ate every fragmented frame on the 2026-08-26 bench (Tailscale: its CGNAT
    IPv4 range and IPv6 ULA prefix). Not a general tunnel detector — it names the
    overlay networks we have actually been bitten by."""
    import ipaddress

    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return False
    if isinstance(ip, ipaddress.IPv4Address):
        return ip in ipaddress.ip_network("100.64.0.0/10")  # RFC 6598 CGNAT (Tailscale)
    return ip in ipaddress.ip_network("fd7a:115c:a1e0::/48")  # Tailscale ULA


def _put_drop_oldest(queue: asyncio.Queue[Any], value: Any) -> None:
    """Insert unconditionally: a full queue sheds its OLDEST entry first. One definition,
    shared by stream()'s push path and stop()'s sentinel, so the overflow policy cannot
    diverge between the path that fills a queue and the one that must always wake it."""
    if queue.full():
        with contextlib.suppress(asyncio.QueueEmpty):
            queue.get_nowait()
    queue.put_nowait(value)


def _zeroed(payload: dict[str, Any]) -> dict[str, Any]:
    """The same jog shape with every rate at 0 — an explicit stop for exactly the DOFs that
    were moving, rather than a blanket zero that would fight another controller."""
    out: dict[str, Any] = {}
    for key, value in payload.items():
        out[key] = {dof: 0.0 for dof in value} if isinstance(value, dict) else 0.0
    return out


def _merge_telemetry(previous: Telemetry | None, incoming: Telemetry) -> Telemetry:
    """Carry the safety block forward across frames that omit it.

    The robot sends `status` at ~5 Hz inside a ~15 Hz telemetry stream, so two frames in
    three have no safety fields. Reading those as "safety unknown" would make a latched
    E-STOP banner flicker off twice a second."""
    if previous is None:
        return incoming
    if incoming.safety is not None:
        return incoming
    from dataclasses import replace

    return replace(
        incoming,
        safety=previous.safety,
        watchdog=previous.watchdog,
        latch_reason=previous.latch_reason,
        motor_faults=previous.motor_faults,
        servo_temps=incoming.servo_temps or previous.servo_temps,
    )


# JOG_HZ, ROBOT_WAIT_S and RETRY_S are exported because they are already reachable and a
# caller legitimately needs to reason about them -- JOG_HZ to size its own control loop,
# ROBOT_WAIT_S to set a sensible wait_ready() timeout. Reachable-but-undeclared is the
# worst of both: people depend on it anyway, and nothing stops it changing.
__all__ = [
    "ACTION_HISTORY",
    "JOG_HZ",
    "RETRY_S",
    "ROBOT_WAIT_S",
    "RemoteTeleop",
    "TeleopError",
]
