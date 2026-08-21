"""RemoteTeleop — the operator session: signaling handshake, WebRTC peer, control channel.

The Python counterpart of @nori/sdk's RemoteTeleop. Same protocol, same role (we are always
the ANSWERER), same defensive posture; different concurrency model — this is asyncio-native
because aiortc is, with a synchronous facade in sync.py for scripts that don't want an event
loop.

    from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth

    auth = UserAuth(URL, ANON, "me@example.com", "pw")
    sig = SupabaseSignaling(URL, ANON, room="NORI-A3-0001", token_provider=auth.token)

    async with RemoteTeleop(sig) as robot:
        await robot.wait_ready()
        print(robot.info.descriptor.joints)
        await robot.jog({"base": {"x": 0.5}}, duration=1.0)
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
import time
import uuid
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
    RecordState,
    RobotInfo,
    Telemetry,
)
from .version import NORI_PROTOCOL_VERSION

log = logging.getLogger("nori_sdk.teleop")

# How often a held jog is re-sent. The robot's watchdog warns at 150 ms (LAN) / 300 ms (WAN),
# so 20 Hz leaves ~3 frames of headroom on the tighter profile.
JOG_HZ = 20.0
# How long we wait for the robot to announce itself before calling it absent. The robot
# announces on join and we re-ask every RETRY_S, so this is several missed chances.
ROBOT_WAIT_S = 20.0
RETRY_S = 2.0


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
        protocol_version: int | None = None,
        on_log: Callable[[str], None] | None = None,
    ) -> None:
        self._signaling = signaling
        self._stun = list(stun)
        self._turn_urls = list(turn_urls)
        self._turn_user = turn_user
        self._turn_credential = turn_credential
        self._protocol_version = protocol_version
        self._log_cb = on_log

        # --- live state (all read-only to callers, via properties) ---
        self._status = ConnectStatus()
        self._info: RobotInfo | None = None
        self._telemetry: Telemetry | None = None
        self._daemon: DaemonStatus | None = None
        self._layout: CameraLayout | None = None
        self._link_mode: str = ""

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
                f"this SDK targets {self._protocol_version or NORI_PROTOCOL_VERSION} - "
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
        if duration <= 0:
            self._send(protocol.control_jog(self._next_seq(), payload))
            return
        deadline = time.monotonic() + duration
        interval = 1.0 / max(1.0, hz)
        while time.monotonic() < deadline and not self._stopped:
            self._send(protocol.control_jog(self._next_seq(), payload))
            await asyncio.sleep(min(interval, max(0.0, deadline - time.monotonic())))
        self._send(protocol.control_jog(self._next_seq(), _zeroed(payload)))

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

        The timeout here is the CLIENT's patience. The robot runs its own per-action deadline
        and answers "timeout" when it expires, so a TeleopError from this call means no reply
        arrived at all — a dead channel or an offline motion stack — not a slow move."""
        if not wait:
            self._send(protocol.control_action(self._next_seq(), targets))
            return None
        action_id = uuid.uuid4().hex[:12]
        future: asyncio.Future[ActionStatus] = asyncio.get_running_loop().create_future()
        self._pending_actions[action_id] = future
        self._send(protocol.control_action(self._next_seq(), targets, action_id))
        try:
            return await asyncio.wait_for(future, timeout)
        except TimeoutError:
            raise TeleopError(
                f"no action_status for {action_id} within {timeout:.0f}s "
                f"(daemon status: {self._daemon.state if self._daemon else 'unknown'})"
            ) from None
        finally:
            self._pending_actions.pop(action_id, None)

    def estop(self) -> None:
        """Latch E-STOP. Motion stays blocked until reset_latch(). Deliberately synchronous
        and un-awaited: it must not be able to block behind anything."""
        self._send(protocol.command("estop"))

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

    # --- video frames ----------------------------------------------------------------------

    async def frames(self) -> AsyncIterator[Any]:
        """Yield decoded video frames (av.VideoFrame) from the composite track.

        One H.264 track carries every camera as tiles; use camera_layout.rect(role) to crop
        one. Yields nothing until the track arrives, so await wait_connected() first."""
        while self._video_track is None and not self._stopped:
            await asyncio.sleep(0.1)
        track = self._video_track
        while track is not None and not self._stopped:
            try:
                yield await track.recv()
            except Exception:  # track ended (robot restart / teardown)
                return

    async def snapshot(self, role: str | None = None, settle: float = 0.3) -> Any:
        """One frame, optionally cropped to a camera role. `settle` discards frames first so
        an auto-exposing camera isn't captured mid-adjust."""
        deadline = time.monotonic() + settle
        frame = None
        async for frame in self.frames():
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
        consumer falls behind — for telemetry, stale data is worse than missing data."""
        queue: asyncio.Queue[Any] = asyncio.Queue(maxsize=maxsize)

        def push(value: Any) -> None:
            if queue.full():
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
            queue.put_nowait(value)

        unsubscribe = self.on(kind, push)
        try:
            while not self._stopped:
                yield await queue.get()
        finally:
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
        self._connected.clear()
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
        candidates 'host' means a direct same-subnet LAN link; anything else (STUN srflx,
        TURN relay) is WAN, which buys a looser dead-man window."""
        mode = "wan"
        try:
            stats = await self._pc.getStats()
            for report in stats.values():
                if getattr(report, "type", "") == "candidate-pair" and getattr(
                    report, "nominated", False
                ):
                    local = stats.get(getattr(report, "localCandidateId", ""))
                    remote = stats.get(getattr(report, "remoteCandidateId", ""))
                    types = {
                        getattr(local, "candidateType", ""),
                        getattr(remote, "candidateType", ""),
                    }
                    if types == {"host"}:
                        mode = "lan"
                    break
        except Exception:
            pass  # getStats is best-effort; defaulting to "wan" is the safe direction
        self._link_mode = mode
        self._send(protocol.link(mode))
        self._log(f"link -> {mode}")

    def _setup_control(self, channel: Any) -> None:
        self._control = channel

        @channel.on("open")
        def _on_open() -> None:
            self._log("control channel open")
            if self._link_mode:
                self._send(protocol.link(self._link_mode))

        @channel.on("close")
        def _on_close() -> None:
            if self._control is channel:
                self._control = None

        @channel.on("message")
        def _on_message(raw: Any) -> None:
            self._handle_frame(raw)

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
            action_future = self._pending_actions.get(parsed.action_id)
            if action_future is not None and parsed.done and not action_future.done():
                action_future.set_result(parsed)
        elif kind == "record_status" and isinstance(parsed, RecordState):
            if self._record_waiters:
                record_future = self._record_waiters.pop(0)
                if not record_future.done():
                    record_future.set_result(parsed)
        self._emit(kind, parsed if parsed is not None else obj)

    # --- internals -------------------------------------------------------------------------

    def _send(self, frame: dict[str, Any]) -> None:
        channel = self._control
        if channel is None or getattr(channel, "readyState", "") != "open":
            # Dropping is correct: the control channel is unreliable by design and the robot
            # is watchdogged, so a frame sent into a dead channel has no meaning to preserve.
            return
        with contextlib.suppress(Exception):
            channel.send(protocol.encode(frame))

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
__all__ = ["JOG_HZ", "RETRY_S", "ROBOT_WAIT_S", "RemoteTeleop", "TeleopError"]
