"""Named navigation and the opt-in LiDAR/IMU streams, end to end against the double.

The failure paths carry the weight here. Navigation is the only verb in this SDK that makes
the robot drive itself, so the interesting question is never "does the happy path work" but
"what does a caller believe when the robot does not answer" -- and the answer must never be
"it stopped". See RobotUnreachable.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

import pytest

from nori_sdk import ImuSample, LidarScan, RobotUnreachable, TeleopError
from nori_sdk.mock import MockRobot, mock_session

GOAL_LIKE = "90b234d9-9582-4d5b-9792-7f81080a4dcb"


class SilentNavigation(MockRobot):
    """Receives navigation requests and never answers — a dropped reply, which is precisely
    what the retry and the unreachable contract exist for."""

    def handle(self, raw: str | dict[str, Any]) -> list[str]:
        message = json.loads(raw) if isinstance(raw, str) else dict(raw)
        if message.get("type") == "navigation":
            self.received.append(message)
            return []
        return super().handle(raw)


class NeverFinishes(MockRobot):
    """Reports a goal as navigating and never lets it finish.

    The double runs its lifecycle off `step()`, so the honest way to model a goal that never
    completes is to stop the clock advancing for it — not to filter the terminal frame out
    of a reply."""

    def step(self, dt: float) -> None:
        held = self._navigation_active
        super().step(dt)
        if held is not None and self._navigation_active is None:
            self._navigation_active = held  # never let it settle


# --- the lifecycle ---------------------------------------------------------------------------


async def test_a_waypoint_can_be_saved_listed_and_navigated():
    async with mock_session() as robot:
        await robot.wait_ready()
        saved = await robot.remember_waypoint("charging station")
        assert saved.ok and saved.replaced is False

        listed = await robot.list_waypoints()
        assert [w.name for w in (listed.waypoints or ())] == ["charging station"]

        started = await robot.navigate_to_waypoint("charging station")
        assert started.ok and started.goal_id
        finished = await robot.await_navigation(started.goal_id, timeout=2.0)
        assert finished.state == "succeeded"
        assert finished.terminal and finished.active is False


async def test_reusing_a_name_replaces_rather_than_duplicating():
    async with mock_session() as robot:
        await robot.wait_ready()
        assert (await robot.remember_waypoint("dock")).replaced is False
        assert (await robot.remember_waypoint("dock")).replaced is True
        listed = await robot.list_waypoints()
        assert len(listed.waypoints or ()) == 1


async def test_a_refusal_is_returned_not_raised():
    """ "waypoint not found" is an ordinary answer to inspect, not an exception — the same
    call that refuses here succeeds once the waypoint exists."""
    async with mock_session() as robot:
        await robot.wait_ready()
        refused = await robot.navigate_to_waypoint("nowhere")
        assert refused.ok is False
        assert refused.error == "waypoint not found"


async def test_a_latched_robot_refuses_to_start_a_goal():
    """The software E-stop is a robot-side gate: it must refuse the goal, not merely stop it
    afterwards."""
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        await robot.remember_waypoint("dock")
        robot.estop()
        refused = await robot.navigate_to_waypoint("dock")
        assert refused.ok is False
        assert refused.state == "unavailable"
        assert "E-stop" in (refused.error or "")


async def test_a_robot_without_the_capability_is_refused_before_the_wire():
    bot = MockRobot(capabilities=["task_jog", "record"])
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        with pytest.raises(TeleopError, match="named_navigation"):
            await robot.get_navigation_status()
        assert not [f for f in bot.received if f.get("type") == "navigation"]


# --- the robot does not answer ----------------------------------------------------------------


async def test_a_silent_robot_raises_rather_than_returning_a_status():
    """A returned status would have to invent `state` and `active`; a caller reading
    active=False off it would read a dropped reply as a halted robot."""
    async with mock_session(SilentNavigation()) as robot:
        await robot.wait_ready()
        with pytest.raises(RobotUnreachable, match="no reply within"):
            await robot.list_waypoints(timeout=0.3)


async def test_an_unanswered_request_is_retried_under_one_request_id():
    """Retrying is safe only because the id never changes: a fresh id per retry would start a
    second physical goal."""
    bot = SilentNavigation()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        with pytest.raises(RobotUnreachable):
            await robot.navigate_to_waypoint("dock", timeout=2.0)
    sent = [f for f in bot.received if f.get("type") == "navigation"]
    assert len(sent) > 1, "the request was never retried"
    assert len({f["request_id"] for f in sent}) == 1
    assert len({f["goal_id"] for f in sent}) == 1


async def test_await_navigation_timing_out_does_not_claim_the_robot_stopped():
    """The goal did not finish in time. The robot was last seen DRIVING, and a client-side
    timeout is not evidence that it stopped — so the last known state must survive on the
    exception rather than being replaced by an invented one."""
    async with mock_session(NeverFinishes()) as robot:
        await robot.wait_ready()
        await robot.remember_waypoint("dock")
        started = await robot.navigate_to_waypoint("dock")
        assert started.goal_id

        with pytest.raises(RobotUnreachable) as caught:
            await robot.await_navigation(started.goal_id, timeout=0.3)
        last = caught.value.last_known
        assert last is not None
        assert last.state == "navigating"
        assert last.active is True
        assert last.terminal is False


async def test_teardown_fails_an_in_flight_goal_wait_immediately():
    """stop() must not leave a caller parked for the full await_navigation timeout."""
    async with mock_session(NeverFinishes()) as robot:
        await robot.wait_ready()
        await robot.remember_waypoint("dock")
        started = await robot.navigate_to_waypoint("dock")
        assert started.goal_id
        waiter = asyncio.create_task(robot.await_navigation(started.goal_id, timeout=120.0))
        await asyncio.sleep(0)  # let the waiter register before tearing down
        await robot.stop()
        with pytest.raises(RobotUnreachable, match="session closed"):
            await waiter


# --- LiDAR and IMU -----------------------------------------------------------------------------


async def test_sensor_streams_are_off_until_asked_for():
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        # A feed nobody asked for produces nothing, however long the robot runs.
        bot.step(1.0)
        assert bot.drain_events() == []
        assert robot.lidar_scan is None
        assert robot.imu_sample is None

        status = await robot.configure_sensor_streams(lidar_hz=5, imu_hz=20)
        assert status.ok and status.lidar_hz == 5 and status.imu_hz == 20
        scan = await anext(robot.stream("lidar_scan"))
        sample = await anext(robot.stream("imu"))
        assert isinstance(scan, LidarScan) and isinstance(sample, ImuSample)
        assert scan.frame_id == "laser"
        assert sample.linear_acceleration_m_s2[-1] == 9.81


async def test_a_zero_rate_stops_a_feed_without_stopping_the_other():
    bot = MockRobot()
    async with mock_session(bot, telemetry_hz=0) as robot:
        await robot.wait_ready()
        await robot.configure_sensor_streams(lidar_hz=5, imu_hz=20)
        await robot.configure_sensor_streams(lidar_hz=0)
        bot.step(1.0)
        kinds = {json.loads(raw)["type"] for raw in bot.drain_events()}
        assert kinds == {"imu"}, "a zero rate must silence only its own feed"


async def test_impossible_rates_never_reach_the_wire():
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        for kwargs, message in (
            ({"lidar_hz": 11}, "between 0 and 10"),
            ({"imu_hz": 51}, "between 0 and 50"),
            ({"lidar_max_points": 8}, "between 16 and 1440"),
            ({}, "at least one setting"),
        ):
            with pytest.raises(ValueError, match=message):
                await robot.configure_sensor_streams(**kwargs)
        assert not [f for f in bot.received if f.get("type") == "sensor_stream"]


async def test_a_sensor_capability_a_robot_lacks_is_refused_preflight():
    bot = MockRobot(capabilities=["task_jog"])
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        with pytest.raises(TeleopError, match="sensor_streams"):
            await robot.configure_sensor_streams(lidar_hz=5)


# --- the double's own idempotency contract ------------------------------------------------------


def test_a_retry_inside_the_window_replays_rather_than_re_running():
    bot = MockRobot()
    request = {
        "type": "navigation",
        "request_id": GOAL_LIKE,
        "action": "remember_waypoint",
        "name": "Dock",
    }
    first = json.loads(bot.handle(request)[0])
    second = json.loads(bot.handle(request)[0])
    assert first["replaced"] is False
    assert second == first, "a retried request_id must replay, not re-run"


def test_a_retry_re_runs_once_the_reply_has_aged_out():
    """The window is finite, so eviction is reachable — and this is what a real robot does
    on the far side of it. An unbounded cache would hide the case entirely."""
    bot = MockRobot()
    request = {
        "type": "navigation",
        "request_id": GOAL_LIKE,
        "action": "remember_waypoint",
        "name": "Dock",
    }
    assert json.loads(bot.handle(request)[0])["replaced"] is False
    for index in range(MockRobot.REQUEST_HISTORY):
        bot.handle(
            {
                "type": "navigation",
                "request_id": f"{index:08d}-0000-4000-8000-000000000000",
                "action": "status",
            }
        )
    assert json.loads(bot.handle(request)[0])["replaced"] is True


def test_a_request_id_that_is_not_a_uuid_is_dropped_silently():
    """The gateway drops it with no reply. A lenient double would let code that mints its own
    ids pass here and fail on hardware."""
    bot = MockRobot()
    assert (
        bot.handle({"type": "navigation", "request_id": "not-a-uuid", "action": "status"}) == []
    )
