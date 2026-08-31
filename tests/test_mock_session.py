"""`mock_session` — the supported no-hardware path, and the examples that depend on it.

These matter more than their size suggests: this is the first thing a new developer runs, so
a break here is a break in the on-ramp. The example is executed rather than imported, because
a README snippet that is never run is a README snippet that is wrong.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from nori_sdk.mock import MockRobot, mock_session
from nori_sdk.motion import JogBuilder

REPO_ROOT = Path(__file__).resolve().parent.parent


async def test_a_mock_session_hands_back_a_ready_connected_robot():
    async with mock_session() as robot:
        info = await robot.wait_ready()
        assert robot.is_connected
        assert info.descriptor is not None and info.descriptor.joints
        # The gateway's open order is ack -> camera_layout -> daemon_status; all three must
        # have landed before the caller gets control, or a script's first check races.
        assert robot.camera_layout is not None
        assert robot.daemon_status is not None and robot.daemon_status.online


async def test_a_script_written_against_the_mock_uses_only_public_api():
    """The point of the helper. Everything below is documented surface -- if this test needs
    an underscore to pass, the helper has failed at its job."""
    async with mock_session() as robot:
        info = await robot.wait_ready()
        await robot.jog(JogBuilder(info.descriptor).base(linear=0.5).build(), duration=0.1)
        result = await robot.action({"left_arm_gripper.pos": 30.0}, wait=True, timeout=1.0)
        assert result.succeeded
        assert (await robot.record("session_start")).ok
        robot.estop()


async def test_pose_works_against_a_plain_mock_session():
    """The default mock advertises what a healthy A3 gateway advertises -- pose_targets
    included -- so pose() must work out of the box, exactly as it does on hardware. It
    raising against the plain mock was audit finding A3: a double LESS capable than the
    fleet teaches a client to route around a verb the robot honours."""
    async with mock_session() as robot:
        info = await robot.wait_ready()
        assert info.supports("pose_targets") is True
        status = await robot.pose("left", [0.4, 0.0, 0.9], wait=True, timeout=1.0)
        assert status is not None and status.done


async def test_named_navigation_works_against_a_plain_mock_session():
    async with mock_session(telemetry_hz=60.0) as robot:
        info = await robot.wait_ready()
        assert info.supports("named_navigation") is True
        remembered = await robot.remember_waypoint("Dock")
        assert remembered.ok and remembered.name == "Dock"
        listed = await robot.list_waypoints()
        assert [waypoint.name for waypoint in listed.waypoints] == ["Dock"]
        started = await robot.navigate_to_waypoint("Dock")
        assert started.ok and started.goal_id
        finished = await robot.await_navigation(started.goal_id, timeout=2.0)
        assert finished.state == "succeeded"


async def test_telemetry_streams_and_reflects_what_was_commanded():
    async with mock_session(telemetry_hz=60.0) as robot:
        info = await robot.wait_ready()
        await robot.jog(JogBuilder(info.descriptor).arm("left", "gripper", 1.0).build(),
                        duration=0.2)
        async for telemetry in robot.stream("telemetry"):
            if telemetry.state.get("left_arm_gripper.pos", 0.0) > 0:
                break


async def test_mock_session_streams_lidar_and_imu_when_requested():
    async with mock_session(telemetry_hz=60.0) as robot:
        status = await robot.configure_sensor_streams(
            lidar_hz=5,
            imu_hz=20,
            lidar_max_points=32,
        )
        assert status.ok
        lidar = await anext(robot.stream("lidar_scan"))
        imu = await anext(robot.stream("imu"))
        assert len(lidar.ranges_m) == 32
        assert lidar.source_points == 360
        assert imu.frame_id == "imu_link"
        assert imu.linear_acceleration_m_s2[-1] == 9.81


async def test_the_awkward_robots_are_reachable_through_the_helper():
    """A developer's real value here is rehearsing the cases hardware won't produce on demand."""
    async with mock_session(MockRobot(online=False)) as robot:
        await robot.wait_ready()
        assert robot.daemon_status is not None and not robot.daemon_status.online
        assert robot.daemon_status.reason == "unreachable"

    async with mock_session(MockRobot(descriptor=None)) as robot:
        info = await robot.wait_ready()
        assert info.descriptor is None, "an absent descriptor is the legacy-robot signal"

    async with mock_session(MockRobot(cameras=False)) as robot:
        await robot.wait_ready()
        assert robot.camera_layout is None, "single-camera robots send no layout at all"

    async with mock_session(MockRobot(action_outcome="clamped")) as robot:
        await robot.wait_ready()
        result = await robot.action({"left_arm_gripper.pos": 999.0}, wait=True, timeout=1.0)
        assert result.done and not result.succeeded


async def test_the_session_stops_cleanly_and_leaves_no_task_running():
    async with mock_session() as robot:
        await robot.wait_ready()
    assert robot.status.phase == "closed"


@pytest.mark.parametrize("script", ["mock_pick_place.py"])
def test_the_runnable_example_still_runs(script):
    """Executed as a subprocess, exactly as the README tells a developer to run it. An
    example that no longer runs is worse than no example: it is the first thing tried."""
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "examples" / script)],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0, f"{script} failed:\n{result.stdout}\n{result.stderr}"
    assert "protocol v" in result.stdout
