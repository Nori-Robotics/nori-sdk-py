"""The verbs that had frame builders but no session method, and the state that was decoded
but never surfaced.

Every one of these passed conformance for weeks — `protocol.policy_stream` built a valid
frame, `Perception.from_wire` parsed a valid frame — while being unreachable from
`RemoteTeleop`. A builder with no call site is not a feature; it is a promise the API does not
keep, and conformance cannot see the difference.
"""

from __future__ import annotations

import pytest

from nori_sdk.mock import MockRobot, mock_session
from nori_sdk.teleop import ACTION_HISTORY, TeleopError

# --- policy stream ---------------------------------------------------------------------


async def test_policy_stream_start_status_stop_round_trips():
    async with mock_session() as robot:
        await robot.wait_ready()

        started = await robot.policy_stream("start", dest="laptop")
        assert started.ok and started.streaming
        assert started.dest == "laptop"

        polled = await robot.policy_stream("status")
        assert polled.streaming and polled.frames_sent > 0

        stopped = await robot.policy_stream("stop")
        assert stopped.ok and not stopped.streaming


async def test_the_cached_status_matches_the_last_reply():
    async with mock_session() as robot:
        await robot.wait_ready()
        assert robot.policy_stream_status is None, "nothing polled yet is not 'not streaming'"
        reply = await robot.policy_stream("start", dest="laptop")
        assert robot.policy_stream_status == reply


async def test_a_stream_that_dies_is_only_visible_by_POLLING():
    """The liveness rule, pinned. The robot's streamer is request/reply — it cannot push, and
    its end-of-run result is discarded. A client that waits for a failure frame waits forever;
    the only way to notice is to ask."""
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        assert (await robot.policy_stream("start", dest="laptop")).streaming

        deaths: list[object] = []
        robot.on("policy_stream_status", deaths.append)
        bot.die_mid_stream()

        # No frame arrives. The cached status still says "streaming" — it is a memory of the
        # last answer, not a live reading.
        assert deaths == []
        assert robot.policy_stream_status is not None
        assert robot.policy_stream_status.streaming is True

        # Polling is what reveals it.
        assert (await robot.policy_stream("status")).streaming is False


async def test_an_unknown_verb_is_returned_as_state_not_raised():
    """Unlike record(), ok:false here is ordinary — a stream that is not running answers
    ok:false to "status" routinely, so raising would make normal polling throw."""
    async with mock_session() as robot:
        await robot.wait_ready()
        reply = await robot.policy_stream("teleport")
        assert reply.ok is False and reply.error


async def test_policy_stream_works_while_MOTION_is_offline():
    """The streamer is served by the bridge in FRONT of the motion daemon, so it runs fine on
    a robot whose arms are disabled. My first cut gated this on daemon_status and strict mode
    would then have refused a valid operation — worse than having no gate."""
    async with mock_session(MockRobot(online=False), strict=True) as robot:
        await robot.wait_ready()
        assert robot.daemon_status is not None and not robot.daemon_status.online
        assert (await robot.policy_stream("start", dest="laptop")).streaming


async def test_policy_stream_times_out_rather_than_hanging_forever():
    async with mock_session() as robot:
        await robot.wait_ready()
        # A robot that never answers: drop the reply on the floor.
        robot._policy_waiters.clear()
        original = robot._send
        robot._send = lambda frame: None  # type: ignore[method-assign]
        try:
            with pytest.raises(TeleopError, match="no policy_stream_status"):
                await robot.policy_stream("status", timeout=0.05)
        finally:
            robot._send = original  # type: ignore[method-assign]
    # And the abandoned waiter is cleaned up, so a later reply cannot resolve a dead future.
    assert robot._policy_waiters == []


async def test_a_motion_verb_IS_still_gated_on_motion_health():
    """The other half: strict mode must still catch commanding motion into the void."""
    async with mock_session(MockRobot(online=False), strict=True) as robot:
        await robot.wait_ready()
        with pytest.raises(TeleopError, match="offline"):
            robot.set_leader_action({"left_arm_shoulder_pitch.pos": 1.0})


# --- perception ------------------------------------------------------------------------


async def test_perception_is_absent_until_a_frame_arrives():
    """None does NOT mean "nothing is in front of the robot" — it means no detector frame has
    arrived, which is the common case and a different claim entirely."""
    async with mock_session() as robot:
        await robot.wait_ready()
        assert robot.perceive() is None
        assert robot.perception_age is None


async def test_perception_surfaces_and_reports_its_own_staleness():
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        robot._handle_frame(bot.perception([{"label": "cup", "conf": 0.9}]))

        seen = robot.perceive()
        assert seen is not None and seen.objects[0]["label"] == "cup"
        age = robot.perception_age
        assert age is not None and age < 1.0, "a just-arrived frame must not read as stale"


async def test_perception_age_uses_our_clock_not_the_robots():
    """ts_ns is the ROBOT's monotonic clock. Differencing it against ours would fold in clock
    skew and report a fresh detection as minutes old — or negative."""
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        # A frame stamped in the far future on the robot's clock.
        robot._handle_frame(
            '{"type":"perception","ts_ns":99999999999999999,"objects":[]}'
        )
        age = robot.perception_age
        assert age is not None and 0 <= age < 1.0


# --- action introspection ---------------------------------------------------------------


async def test_a_fire_and_forget_action_can_be_polled_by_id():
    async with mock_session() as robot:
        await robot.wait_ready()
        action_id = robot.next_action_id()
        assert robot.action_status(action_id) is None

        await robot.action({"left_arm_gripper.pos": 30.0}, wait=True, timeout=1.0)
        # wait=True mints its own id; poll the one the mock actually answered.
        answered = [k for k in robot._action_states]
        assert answered, "no action verdict was retained"
        assert robot.action_status(answered[-1]) is not None


async def test_action_ids_are_unique():
    ids = {mock_id for mock_id in (__import__("nori_sdk").RemoteTeleop.next_action_id()
                                   for _ in range(200))}
    assert len(ids) == 200


async def test_the_action_verdict_map_is_bounded():
    """A policy issuing thousands of actions must not grow this forever. Only the most recent
    verdict per id is ever useful, so old ones are evicted rather than accumulated."""
    async with mock_session(telemetry_hz=0) as robot:
        await robot.wait_ready()
        for i in range(ACTION_HISTORY + 50):
            robot._handle_frame(
                f'{{"type":"action_status","action_id":"a{i}","state":"done"}}'
            )
        assert len(robot._action_states) == ACTION_HISTORY
        assert robot.action_status("a0") is None, "oldest should have been evicted"
        assert robot.action_status(f"a{ACTION_HISTORY + 49}") is not None


# --- record state ----------------------------------------------------------------------


async def test_record_state_caches_the_last_whole_snapshot():
    """Each reply carries the WHOLE recording state, not a delta, so the cached value is a
    complete snapshot and a client that missed a frame is not left inconsistent."""
    async with mock_session() as robot:
        await robot.wait_ready()
        assert robot.record_state is None
        await robot.record("session_start")
        await robot.record("episode_start", task="demo")
        cached = robot.record_state
        assert cached is not None and cached.recording and cached.episode == "episode-0001"


# --- leader / video quality / call -------------------------------------------------------


async def test_leader_action_reaches_the_wire_in_the_flat_key_shape():
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        robot.set_leader_action({"left_arm_shoulder_pitch.pos": 12.5})
        sent = [m for m in bot.received if "leader_action_deg" in m]
        assert sent and sent[-1]["leader_action_deg"] == {"left_arm_shoulder_pitch.pos": 12.5}
        assert "seq" in sent[-1], "leader frames are motion-bearing and carry a seq"


async def test_video_quality_and_call_reach_the_wire():
    bot = MockRobot()
    async with mock_session(bot) as robot:
        await robot.wait_ready()
        robot.set_video_quality("low")
        robot.call(state="join", mic_muted=True)
        kinds = [m.get("type") for m in bot.received]
        assert "video" in kinds and "call" in kinds
