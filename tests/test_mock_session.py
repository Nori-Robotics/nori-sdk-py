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


async def test_telemetry_streams_and_reflects_what_was_commanded():
    async with mock_session(telemetry_hz=60.0) as robot:
        info = await robot.wait_ready()
        await robot.jog(JogBuilder(info.descriptor).arm("left", "gripper", 1.0).build(),
                        duration=0.2)
        async for telemetry in robot.stream("telemetry"):
            if telemetry.state.get("left_arm_gripper.pos", 0.0) > 0:
                break


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
