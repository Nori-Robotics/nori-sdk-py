"""Session behavior without a WebRTC stack.

RemoteTeleop imports aiortc lazily (only when an offer actually arrives), so everything that
happens ON the control channel — frame handling, action correlation, the jog stream, the
watchdog-shaped stop semantics — is testable with a fake channel and no peer connection.
That covers the logic most likely to be wrong; the parts that need real hardware are called
out in the README.
"""

import asyncio
import json

import pytest

from nori_sdk.mock.loopback import loopback_pair
from nori_sdk.mock.robot import MockRobot
from nori_sdk.teleop import RemoteTeleop, TeleopError


class FakeChannel:
    """Stands in for the RTCDataChannel the robot opens."""

    def __init__(self, robot: MockRobot | None = None, ready: str = "open") -> None:
        self.readyState = ready
        self.sent: list[dict] = []
        self._robot = robot
        self._sink = None

    def send(self, raw: str) -> None:
        self.sent.append(json.loads(raw))
        if self._robot is not None and self._sink is not None:
            for reply in self._robot.handle(raw):
                self._sink(reply)


def session(robot: MockRobot | None = None, ready: str = "open"):
    """A RemoteTeleop wired to a fake channel, as if a peer had connected."""
    operator, _robot_side = loopback_pair()
    teleop = RemoteTeleop(operator)
    channel = FakeChannel(robot, ready)
    channel._sink = teleop._handle_frame
    teleop._control = channel
    return teleop, channel


def open_with(robot: MockRobot, teleop: RemoteTeleop) -> None:
    for frame in robot.on_channel_open():
        teleop._handle_frame(frame)


async def test_handshake_populates_descriptor_and_daemon_state():
    robot = MockRobot()
    teleop, _channel = session(robot)
    open_with(robot, teleop)
    assert teleop.info is not None and len(teleop.info.descriptor.joints) == 10
    assert teleop.daemon_status.online
    assert teleop.camera_layout.rect("overhead") == (0.0, 0.5, 0.5, 0.5)


async def test_a_stateless_daemon_status_leaves_the_session_health_alone():
    """The bridge rebroadcasts every few seconds while offline, so a malformed repeat is not
    a one-off: it must neither flip a healthy robot to offline nor reach subscribers at all.
    Emitting it would hand them a raw dict where every other daemon_status is a DaemonStatus,
    which is worse than silence — a callback that reads `.online` would raise."""
    robot = MockRobot()
    teleop, _channel = session(robot)
    open_with(robot, teleop)
    assert teleop.daemon_status.online

    seen: list[object] = []
    teleop.on("daemon_status", seen.append)
    teleop._handle_frame(json.dumps({"type": "daemon_status"}))

    assert seen == [], "a dropped frame was emitted to subscribers"
    assert teleop.daemon_status.online, "a stateless frame invented an outage"


async def test_telemetry_carries_safety_forward_across_bare_frames():
    # The robot sends `status` at ~5 Hz inside a ~15 Hz stream. Reading a bare frame as
    # "safety unknown" would flicker a latched E-STOP off twice a second.
    robot = MockRobot()
    teleop, _channel = session(robot)
    robot.estopped = True
    teleop._handle_frame(robot.telemetry({"a.pos": 1.0}, with_status=True))
    assert teleop.telemetry.safety == "latched"
    teleop._handle_frame(robot.telemetry({"a.pos": 2.0}, with_status=False))
    assert teleop.telemetry.safety == "latched"  # carried forward
    assert teleop.telemetry.state == {"a.pos": 2.0}  # but state is the new frame's


async def test_action_wait_resolves_on_the_TERMINAL_status_not_the_first_one():
    """The robot answers accepted -> active -> done. Returning on "accepted" is returning
    before the move has happened: the watchdog is still free to abort it, and the caller has
    been told it succeeded. That defect was live in this SDK, and the mock reproduced it, so
    the two agreed and the suite stayed green."""
    robot = MockRobot()
    teleop, _channel = session(robot)
    status = await teleop.action({"left_arm_gripper.pos": 30}, wait=True, timeout=1.0)
    assert status.state == "done"
    assert status.done and status.succeeded
    assert robot.action == {"left_arm_gripper.pos": 30}


async def test_action_wait_reports_a_clamp_as_finished_but_not_successful():
    """The robot completed a move to somewhere other than the target. A caller that only
    checks `.done` must not read this as "we are where I asked"."""
    teleop, _channel = session(MockRobot(action_outcome="clamped"))
    status = await teleop.action({"left_arm_gripper.pos": 999}, wait=True, timeout=1.0)
    assert status.state == "clamped"
    assert status.done and not status.succeeded


async def test_non_terminal_action_frames_still_reach_subscribers():
    """Only the future waits for terminality. A progress UI still sees every transition."""
    teleop, _channel = session(MockRobot())
    seen: list[str] = []
    teleop.on("action_status", lambda s: seen.append(s.state))
    await teleop.action({"left_arm_gripper.pos": 30}, wait=True, timeout=1.0)
    assert seen == ["accepted", "active", "done"]


async def test_action_wait_reports_a_block_rather_than_raising():
    # "blocked" is a real answer, not an error — the caller decides what to do about it.
    robot = MockRobot()
    robot.estopped = True
    teleop, _channel = session(robot)
    status = await teleop.action({"a.pos": 1}, wait=True, timeout=1.0)
    assert status.state == "blocked" and status.reason == "latched"


async def test_action_wait_times_out_with_the_daemon_state_in_the_message():
    teleop, _channel = session(MockRobot(online=False))
    teleop._daemon = None
    with pytest.raises(TeleopError, match="no action_status"):
        await teleop.action({"a.pos": 1}, wait=True, timeout=0.05)


async def test_record_verbs_resolve_in_order_and_refusals_raise():
    robot = MockRobot()
    teleop, _channel = session(robot)
    assert (await teleop.record("session_start")).ok
    started = await teleop.record("episode_start", "fold towel")
    assert started.episode == "episode-0001"
    with pytest.raises(TeleopError, match="already recording"):
        await teleop.record("episode_start")


async def test_jog_with_duration_streams_then_zeroes():
    teleop, channel = session()
    await teleop.jog({"base": {"x": 0.5}}, duration=0.12, hz=50)
    jogs = [f["jog"] for f in channel.sent if f["type"] == "control"]
    assert len(jogs) > 2, "a held jog must be RESENT — silence is a stop command"
    assert jogs[-1] == {"base": {"x": 0.0}}, "must end with an explicit zero"


async def test_jog_seq_is_monotonic():
    teleop, channel = session()
    await teleop.jog({"base": {"x": 0.5}}, duration=0.06, hz=50)
    seqs = [f["seq"] for f in channel.sent if "seq" in f]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)


async def test_stop_jog_zeroes_only_the_dofs_that_were_moving():
    # A blanket zero would fight another controller driving a different group.
    teleop, channel = session()
    teleop.set_jog({"left_arm": {"shoulder_pan": 0.5}, "left_lift": 0.2})
    await asyncio.sleep(0.12)
    await teleop.stop_jog()
    assert channel.sent[-1]["jog"] == {"left_arm": {"shoulder_pan": 0.0}, "left_lift": 0.0}


async def test_sends_are_dropped_when_the_channel_is_closed():
    # The control channel is unreliable by design and the robot is watchdogged, so a frame
    # sent into a dead channel has no meaning to preserve — dropping beats raising.
    teleop, channel = session(ready="closed")
    teleop.estop()
    assert channel.sent == []


async def test_listeners_receive_parsed_frames_and_a_raising_one_is_contained():
    robot = MockRobot()
    teleop, _channel = session(robot)
    seen = []
    teleop.on("telemetry", lambda t: (_ for _ in ()).throw(RuntimeError("bad listener")))
    teleop.on("telemetry", seen.append)
    teleop._handle_frame(robot.telemetry({"a.pos": 3.0}))
    assert len(seen) == 1 and seen[0].state == {"a.pos": 3.0}


async def test_unknown_frame_kinds_reach_listeners_as_raw_dicts():
    teleop, _channel = session()
    seen = []
    teleop.on("weather", seen.append)
    teleop._handle_frame('{"type":"weather","sunny":true}')
    assert seen == [{"type": "weather", "sunny": True}]


async def test_wait_ready_reports_a_refusal_with_its_reason():
    teleop, _channel = session()
    teleop._handle_frame('{"type":"ack","accepted":false,"error":"another operator"}')
    with pytest.raises(TeleopError, match="another operator"):
        await teleop.wait_ready(timeout=0.5)


async def test_start_without_aiortc_names_the_install_command():
    # Failing here beats failing 20 s later, mid-negotiation, inside a transport callback.
    import importlib.util

    if importlib.util.find_spec("aiortc") is not None:
        pytest.skip("aiortc is installed")
    operator, _robot = loopback_pair()
    with pytest.raises(ImportError, match=r'nori-sdk\[webrtc\]'):
        await RemoteTeleop(operator).start()


async def test_wait_connected_surfaces_the_named_failure():
    teleop, _channel = session()
    teleop._set_phase("failed", "robot_absent", "no offer within 20s")
    with pytest.raises(TeleopError, match="robot_absent"):
        await teleop.wait_connected(timeout=0.05)
