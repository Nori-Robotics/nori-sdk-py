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
        await robot.jog(JogBuilder(info.descriptor).central_lift(1.0).build(), duration=0.5)
        # ~0.5 s at full rate = ~CENTRAL_LIFT_MM_PER_S/2 mm, clamped to [0, 720]
        pos = bot.pose.get("lift.pos", 0.0)
        assert 0.0 < pos <= CENTRAL_LIFT_MM_PER_S  # moved, in mm scale, within range


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
