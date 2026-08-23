"""The policy-loop gap features: hold(), A3 descriptor/central lift, snapshot_png, strict.

These exist for one consumer: an unattended synthesized policy running against a real
robot. Every test here rehearses the failure that motivated the feature — a watchdog
starving between commands, a lift verb that silently goes nowhere, an artifact encoder
that mangles pixels, a policy commanding a dead robot and reporting success.
"""

import asyncio
import struct
import zlib

import pytest

from nori_sdk._png import encode_rgb24
from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session
from nori_sdk.mock.robot import CENTRAL_LIFT_MM_PER_S, WATCHDOG_PROFILES
from nori_sdk.motion import CENTRAL_LIFT_GROUP, JogBuilder
from nori_sdk.teleop import TeleopError
from nori_sdk.types import RobotDescriptor

# --- hold(ms) --------------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_hold_feeds_the_watchdog_across_t_stop() -> None:
    """A pause longer than t_stop must NOT trip safe_hold when spent inside hold()."""
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        t_stop = WATCHDOG_PROFILES["wan"][1] / 1000.0
        await robot.hold((t_stop + 0.5) * 1000.0)
        # the mock's watchdog flags safe_hold in telemetry on control silence; a fed
        # watchdog stays "ok"/"warn", never "stop"
        assert bot.watchdog != "stop"
        t = robot.telemetry
        assert t is None or t.safety != "safe_hold"


@pytest.mark.asyncio
async def test_hold_does_not_disturb_latched_action_targets() -> None:
    """hold() streams an EMPTY jog: rate-stop + keep-alive, never an action cancel."""
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        await robot.action({"left_arm_shoulder_pan.pos": 30.0})
        await robot.hold(200.0)
        assert bot.action.get("left_arm_shoulder_pan.pos") == 30.0


# --- A3 descriptor + central lift ------------------------------------------------------------


def test_a3_descriptor_matches_live_ground_truth() -> None:
    d = RobotDescriptor.from_wire(A3_DESCRIPTOR)
    assert d is not None
    assert len(d.joints) == 16  # 2 sides x (7 DoF + gripper)
    assert "left_arm_bicep_yaw.pos" in d.joints and "right_arm_wrist_pitch.pos" in d.joints
    assert d.aux == ["lift"]
    assert d.ranges["lift.pos"] == (0.0, 720.0)  # millimeters
    assert d.ranges["right_arm_gripper.pos"] == (0.0, 100.0)
    assert d.ranges["right_arm_elbow_pitch.pos"] == (-100.0, 100.0)


def test_central_lift_builder_strict_paths() -> None:
    a3 = RobotDescriptor.from_wire(A3_DESCRIPTOR)
    payload = JogBuilder(a3).central_lift(0.5).build()
    assert payload == {CENTRAL_LIFT_GROUP: 0.5}
    # an L2-shaped robot must refuse central_lift, and the error must point at lift(side)
    l2 = RobotDescriptor.from_wire(
        {"joints": [], "aux": ["left_lift", "right_lift"], "base": [], "ranges": {}}
    )
    with pytest.raises(ValueError, match="per-arm"):
        JogBuilder(l2).central_lift(0.5)
    # and the reverse: per-arm lift on an A3 points at central_lift()
    with pytest.raises(ValueError, match="central"):
        JogBuilder(a3).lift("left", 0.5)


def test_central_lift_stop_shape_is_scalar() -> None:
    payload = JogBuilder.stop(groups=(CENTRAL_LIFT_GROUP,))
    assert payload == {"lift": 0.0}  # bare scalar — {} would be parsed as no-op


@pytest.mark.asyncio
async def test_mock_a3_integrates_central_lift_in_millimeters() -> None:
    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        info = await robot.wait_ready()
        assert info.descriptor is not None and "lift" in info.descriptor.aux
        start = bot.pose["lift.pos"]  # seeded mid-travel (360), like a real robot
        await robot.jog(JogBuilder(info.descriptor).central_lift(1.0).build(), duration=0.5)
        # ~0.5 s at full rate = ~CENTRAL_LIFT_MM_PER_S/2 mm upward from the seed
        moved = bot.pose["lift.pos"] - start
        assert 0.0 < moved <= CENTRAL_LIFT_MM_PER_S  # moved up, in mm scale


# --- snapshot_png ----------------------------------------------------------------------------


def test_encode_rgb24_roundtrip() -> None:
    np = pytest.importorskip("numpy")
    image = np.zeros((4, 3, 3), dtype=np.uint8)
    image[0, 0] = (255, 0, 0)
    image[3, 2] = (0, 128, 255)
    png = encode_rgb24(image)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    # IHDR: width 3, height 4, bit depth 8, color type 2 (truecolor)
    assert struct.unpack("!II", png[16:24]) == (3, 4)
    assert png[24] == 8 and png[25] == 2
    # inflate IDAT and check the exact pixels survive (filter byte 0 per scanline)
    idat_start = png.index(b"IDAT") + 4
    idat_len = struct.unpack("!I", png[idat_start - 8 : idat_start - 4])[0]
    raw = zlib.decompress(png[idat_start : idat_start + idat_len])
    stride = 3 * 3 + 1
    assert raw[0] == 0 and raw[1:4] == b"\xff\x00\x00"          # (0,0) red, filter 0
    assert raw[3 * stride + 1 + 2 * 3 : 3 * stride + 1 + 3 * 3] == b"\x00\x80\xff"


def test_encode_rgb24_rejects_wrong_shapes() -> None:
    np = pytest.importorskip("numpy")
    with pytest.raises(ValueError, match="rgb24"):
        encode_rgb24(np.zeros((4, 3), dtype=np.uint8))          # missing channels
    with pytest.raises(ValueError, match="uint8"):
        encode_rgb24(np.zeros((4, 3, 3), dtype=np.float32))     # wrong dtype


# --- strict mode -----------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_strict_raises_when_daemon_offline() -> None:
    """The exact trap: a policy commanding a robot whose motion stack is down would
    otherwise look identical to a robot ignoring it."""
    bot = MockRobot(online=False)
    async with mock_session(bot, strict=True) as robot:
        await robot.wait_ready()
        for _ in range(50):  # let daemon_status land
            if robot.daemon_status is not None:
                break
            await asyncio.sleep(0.01)
        with pytest.raises(TeleopError, match="offline"):
            await robot.action({"left_arm_shoulder_pan.pos": 10.0})
        with pytest.raises(TeleopError, match="offline"):
            await robot.jog({"base": {"linear": 0.2}}, duration=0.05)


@pytest.mark.asyncio
async def test_strict_default_off_preserves_silent_behavior() -> None:
    bot = MockRobot(online=False)
    async with mock_session(bot) as robot:  # strict not set
        await robot.wait_ready()
        await robot.action({"left_arm_shoulder_pan.pos": 10.0})  # must not raise


@pytest.mark.asyncio
async def test_strict_allows_motion_on_a_healthy_session() -> None:
    async with mock_session(strict=True) as robot:
        await robot.wait_ready()
        await robot.action({"left_arm_shoulder_pan.pos": 10.0})  # no raise
        await robot.hold(100.0)


def test_mock_pose_seeded_from_descriptor() -> None:
    """A real gateway reports every calibrated joint from the first telemetry
    frame; the mock must too, or a policy's readiness poll passes on the base
    keys alone and then KeyErrors on an arm joint (hardware-found 2026-08-22)."""
    from nori_sdk.mock import A3_DESCRIPTOR
    from nori_sdk.mock.robot import DEFAULT_DESCRIPTOR, MockRobot

    a3 = MockRobot(descriptor=A3_DESCRIPTOR)
    for key in A3_DESCRIPTOR["joints"]:
        assert a3.pose[key] == 0.0
    assert a3.pose["lift.pos"] == 360.0  # mid-travel of [0, 720]

    l2 = MockRobot(descriptor=DEFAULT_DESCRIPTOR)
    for key in DEFAULT_DESCRIPTOR["joints"]:
        assert l2.pose[key] == 0.0
    assert "lift.pos" not in l2.pose  # no central lift advertised

    bare = MockRobot(descriptor=None)  # legacy robot: nothing to seed
    assert bare.pose == {}


@pytest.mark.asyncio
async def test_action_wait_feeds_the_watchdog_while_traveling(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """One action frame then silence starves the robot's dead-man during any
    move slower than t_stop — the robot drops the target mid-flight and
    answers timeout (hardware-found 2026-08-22, first live agent0 run).
    action(wait=True) must stream the empty-jog keep-alive until terminal."""
    from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session

    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        await robot.wait_ready()

        # Hold the action_status back so the wait genuinely spans time.
        real_handle = robot._handle_frame
        parked: list = []

        def gate_frames(frame: str) -> None:
            if '"action_status"' in frame:
                parked.append(frame)
                return
            real_handle(frame)

        robot._handle_frame = gate_frames  # type: ignore[method-assign]
        sent_before = len(bot.received)
        task = asyncio.get_running_loop().create_task(
            robot.action({"right_arm_elbow_pitch.pos": 20.0}, wait=True, timeout=5.0)
        )
        await asyncio.sleep(1.2)  # > WAN t_stop
        keepalives = [
            f for f in bot.received[sent_before:]
            if f.get("type") == "control" and f.get("jog") == {}
        ]
        assert len(keepalives) >= 10, f"only {len(keepalives)} keep-alives in 1.2s"
        assert bot.watchdog != "stop"
        robot._handle_frame = real_handle  # type: ignore[method-assign]
        for frame in parked:
            real_handle(frame)
        status = await task
        assert status is not None and status.done


def test_control_pose_builder_shape() -> None:
    from nori_sdk.protocol import control_pose

    frame = control_pose(7, "right", [0.55, -0.45, 0.98], action_id="p1")
    assert frame["pose"]["right_arm"] == {"frame": "base_footprint",
                                          "position_m": [0.55, -0.45, 0.98]}
    assert "orientation_xyzw" not in frame["pose"]["right_arm"]  # omitted = keep current
    full = control_pose(8, "left", [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], "p2")
    assert full["pose"]["left_arm"]["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]


@pytest.mark.asyncio
async def test_goto_pose_waits_for_terminal_status() -> None:
    from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session

    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        status = await robot.goto_pose("right", [0.55, -0.45, 0.98])
        assert status is not None and status.succeeded


@pytest.mark.asyncio
async def test_goto_pose_bad_pose_is_structured() -> None:
    from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session

    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        # malformed by hand (the typed API can't produce this shape)
        fut = asyncio.get_running_loop().create_future()
        robot._pending_actions["px"] = fut
        robot._send({"type": "control", "seq": 999, "action_id": "px",
                     "pose": {"right_arm": {"frame": "wrong"}}})
        status = await asyncio.wait_for(fut, 5.0)
        assert status.state == "blocked" and status.reason == "bad_pose"


@pytest.mark.asyncio
async def test_pose_wait_feeds_the_watchdog_while_traveling() -> None:
    """pose(wait=True) is the exact shape that starves the dead-man: ONE
    control frame latching a target the arm takes seconds to reach, then an
    await. Without the empty-jog keep-alive the watchdog hits t_stop and
    drops the move mid-flight (the same hardware-found failure action() had,
    2026-08-22). The keep-alive must stream until the terminal status."""
    from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session

    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        await robot.wait_ready()

        real_handle = robot._handle_frame
        parked: list = []

        def gate_frames(frame: str) -> None:
            if '"action_status"' in frame:
                parked.append(frame)
                return
            real_handle(frame)

        robot._handle_frame = gate_frames  # type: ignore[method-assign]
        sent_before = len(bot.received)
        task = asyncio.get_running_loop().create_task(
            robot.pose("right", [0.55, -0.45, 0.98], wait=True, timeout=5.0)
        )
        await asyncio.sleep(1.2)  # > WAN t_stop
        keepalives = [
            f for f in bot.received[sent_before:]
            if f.get("type") == "control" and f.get("jog") == {}
        ]
        assert len(keepalives) >= 10, f"only {len(keepalives)} keep-alives in 1.2s"
        assert bot.watchdog != "stop"
        robot._handle_frame = real_handle  # type: ignore[method-assign]
        for frame in parked:
            real_handle(frame)
        status = await task
        assert status is not None and status.done


@pytest.mark.asyncio
async def test_goto_pose_is_a_pose_alias() -> None:
    """goto_pose survives as the awaited-defaults alias; both names must ride
    the SAME implementation (one feeder, one gate, no drift)."""
    from nori_sdk.mock import A3_DESCRIPTOR, MockRobot, mock_session

    bot = MockRobot(descriptor=A3_DESCRIPTOR)
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        status = await robot.goto_pose("right", [0.55, -0.45, 0.98])
        assert status is not None and status.done
        pose_frames = [f for f in bot.received if f.get("type") == "control"
                       and "pose" in f]
        assert pose_frames, "alias sent no pose frame"
