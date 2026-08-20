"""The mock robot's behavior, pinned against the real gateway (nori_gateway/protocol.py).

If one of these fails after a gateway change, the mock is wrong — fix it here rather than
teaching a client to tolerate both.
"""

import json

import pytest

from nori_sdk import protocol
from nori_sdk.mock.robot import LAYOUT_REPEATS, MockRobot


def kinds(frames):
    return [json.loads(f)["type"] for f in frames]


def test_channel_open_order_matches_the_gateway():
    assert kinds(MockRobot().on_channel_open()) == ["ack", "camera_layout", "daemon_status"]


def test_single_camera_robot_sends_no_layout():
    assert kinds(MockRobot(cameras=False).on_channel_open()) == ["ack", "daemon_status"]


def test_layout_repeats_then_stops():
    robot = MockRobot()
    robot.on_channel_open()  # counts as the first send
    repeats = [robot.repeat_layout() for _ in range(LAYOUT_REPEATS + 2)]
    assert sum(r is not None for r in repeats) == LAYOUT_REPEATS - 1
    assert repeats[-1] is None


def test_ack_carries_the_descriptor_and_watchdog():
    _kind, info, _raw = protocol.decode(MockRobot().on_channel_open()[0])
    assert info.accepted and info.norm_mode == "range_m100_100"
    assert info.descriptor is not None and len(info.descriptor.joints) == 10
    assert info.watchdog_profile is not None and info.watchdog_profile.t_stop_ms == 1000


def test_offline_robot_drops_motion_frames():
    robot = MockRobot(online=False)
    robot.handle(protocol.control_jog(1, {"base": {"x": 1.0}}))
    assert robot.dropped_motion_frames == 1 and robot.jog == {}


def test_action_with_an_id_draws_a_status():
    robot = MockRobot()
    replies = robot.handle(protocol.control_action(1, {"left_arm_gripper.pos": 30}, "abc"))
    _kind, status, _raw = protocol.decode(replies[0])
    assert status.action_id == "abc" and status.state == "accepted"


def test_estop_latches_and_blocks_later_actions():
    robot = MockRobot()
    robot.handle(protocol.command("estop"))
    assert robot.estopped
    _kind, status, _raw = protocol.decode(
        robot.handle(protocol.control_action(2, {"a.pos": 1}, "xyz"))[0]
    )
    assert status.state == "blocked" and status.reason == "latched"
    robot.handle(protocol.command("reset_latch"))
    assert not robot.estopped


def test_record_lifecycle():
    robot = MockRobot()

    def verb(action):
        _kind, state, _raw = protocol.decode(robot.handle(protocol.record(action))[0])
        return state

    assert verb("session_start").ok
    started = verb("episode_start")
    assert started.ok and started.recording and started.episode == "episode-0001"
    assert verb("episode_start").error == "already recording an episode"
    stopped = verb("episode_stop")
    assert stopped.ok and not stopped.recording and stopped.episodes_kept == 1
    assert verb("episode_stop").error == "not recording"


def test_link_mode_is_recorded():
    robot = MockRobot()
    robot.handle(protocol.link("lan"))
    assert robot.link_mode == "lan"
    robot.handle(protocol.link("bogus"))
    assert robot.link_mode == "lan"  # unknown values ignored, not adopted


# --- the pose integrator ---------------------------------------------------------------


def _jog(robot, payload):
    robot.handle(protocol.control_jog(1, payload))


def test_a_held_jog_integrates_into_the_pose():
    robot = MockRobot()
    _jog(robot, {"left_arm": {"gripper": 1.0}})
    robot.step(0.5)
    robot.step(0.5)
    # 1.0 rate * 1.0 s * JOG_SCALE, and telemetry reports the integrated pose by default.
    assert robot.pose["left_arm_gripper.pos"] == pytest.approx(40.0)
    _kind, telemetry, _raw = protocol.decode(robot.telemetry())
    assert telemetry.state["left_arm_gripper.pos"] == pytest.approx(40.0)


def test_the_pose_is_clamped_to_the_descriptor_range_never_rejected():
    robot = MockRobot()
    # Resend every tick, as a real jog stream does. Holding the jog by NOT resending is what
    # the watchdog exists to punish -- the first draft of this test did exactly that and only
    # moved for one watchdog window before the robot stopped itself.
    for _ in range(40):  # 40 x 0.1 s x rate 1.0 x JOG_SCALE = 160, well past the limit
        _jog(robot, {"left_arm": {"elbow_flex": 1.0}})
        robot.step(0.1)
    assert robot.pose["left_arm_elbow_flex.pos"] == 100.0  # range is [-100, 100]


def test_an_absent_base_object_means_stop_not_hold_the_last_velocity():
    """control.json is explicit about this, and it is the difference between a base that
    coasts forever and one that stops. A mock that held the last velocity would teach a
    script the opposite of what the robot does."""
    robot = MockRobot()
    _jog(robot, {"base": {"linear": 0.5}})
    robot.step(0.1)
    assert robot.pose["x.vel"] == 0.5

    _jog(robot, {"left_arm": {"gripper": 0.2}})  # a later jog that omits `base` entirely
    robot.step(0.1)
    assert robot.pose["x.vel"] == 0.0, "the base kept coasting after `base` went absent"


def test_a_latched_robot_does_not_move():
    robot = MockRobot()
    robot.handle(protocol.command("estop"))
    _jog(robot, {"left_arm": {"gripper": 1.0}, "base": {"linear": 1.0}})
    robot.step(1.0)
    assert robot.pose.get("left_arm_gripper.pos", 0.0) == 0.0
    assert robot.pose["x.vel"] == 0.0


def test_an_absolute_action_lands_in_the_pose():
    robot = MockRobot()
    robot.handle(protocol.control_action(1, {"left_arm_gripper.pos": 30.0}))
    assert robot.pose["left_arm_gripper.pos"] == 30.0


# --- the watchdog: the behaviour a script gets wrong locally and pays for on hardware ----


def _status(robot):
    _kind, telemetry, _raw = protocol.decode(robot.telemetry())
    return telemetry


def test_silence_ramps_through_warn_to_stop():
    robot = MockRobot()  # no link frame yet -> WAN profile, 300/1000 ms
    _jog(robot, {"left_arm": {"gripper": 1.0}})
    robot.step(0.1)
    assert _status(robot).watchdog == "ok"
    robot.step(0.25)  # 350 ms of silence: past t_warn, not yet t_stop
    assert _status(robot).watchdog == "warn"
    robot.step(0.7)  # 1050 ms: past t_stop
    assert _status(robot).watchdog == "stop"


def test_a_stopped_watchdog_actually_stops_the_motion():
    """The point. A mock that reported "stop" while still integrating the jog would be worse
    than one with no watchdog at all -- it would look correct and teach the wrong lesson."""
    robot = MockRobot()
    for _ in range(5):  # drive properly first, so there is motion to freeze
        _jog(robot, {"left_arm": {"gripper": 1.0}, "base": {"linear": 1.0}})
        robot.step(0.1)
    assert robot.pose["left_arm_gripper.pos"] > 0 and robot.pose["x.vel"] == 1.0

    robot.step(1.5)  # then go silent past t_stop
    frozen = robot.pose["left_arm_gripper.pos"]
    robot.step(1.0)
    assert robot.pose["left_arm_gripper.pos"] == frozen, "kept moving with nobody driving"
    assert robot.pose["x.vel"] == 0.0


def test_safe_hold_self_clears_and_never_needs_a_latch_reset():
    """safe_hold is not an E-STOP. Only a latch needs reset_latch(); silence just needs the
    operator to come back."""
    robot = MockRobot()
    robot.step(2.0)
    assert _status(robot).safety == "safe_hold"
    _jog(robot, {"left_arm": {"gripper": 0.5}})
    robot.step(0.05)
    assert _status(robot).safety == "ok"
    assert not robot.estopped


def test_the_link_mode_selects_the_profile_the_robot_enforces():
    """LAN is the tighter profile, so the same silence that is survivable on WAN is not on
    LAN. A client that reports the wrong link mode gets stopped for no visible reason."""
    lan = MockRobot()
    lan.handle(protocol.link("lan"))
    assert lan.watchdog_profile == (150.0, 500.0)
    lan.step(0.6)
    assert _status(lan).watchdog == "stop"

    wan = MockRobot()
    wan.handle(protocol.link("wan"))
    wan.step(0.6)  # same silence, more forgiving profile
    assert _status(wan).watchdog == "warn"


def test_the_ack_advertises_the_profile_it_will_actually_enforce():
    """A client reads t_warn_ms from the ack and paces its stream by it. If the advertised
    numbers and the enforced ones disagree, every client pacing itself correctly still gets
    stopped."""
    robot = MockRobot()
    robot.handle(protocol.link("lan"))
    _kind, info, _raw = protocol.decode(robot.on_channel_open()[0])
    assert info.watchdog_profile is not None
    advertised = (info.watchdog_profile.t_warn_ms, info.watchdog_profile.t_stop_ms)
    assert advertised == robot.watchdog_profile
