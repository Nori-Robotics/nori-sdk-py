"""A first task against the mock robot: no hardware, no network, no credentials.

    python examples/mock_pick_place.py

Needs nothing installed beyond the package itself -- not even the `webrtc` extra, because
there is no peer connection here.

The task: discover what robot we're attached to, drive the base, close a gripper to an
absolute target, record the whole thing as an episode, then stop safely. Every line of it
runs unchanged against a real robot; the only difference is which context manager opens the
session. Swap the marked line and this is a production script.

What the mock does NOT prove: ICE, TURN, bandwidth, video, or real timing. A green run here
means the LOGIC is right.
"""

import asyncio

from nori_sdk.mock import MockRobot, mock_session
from nori_sdk.motion import JogBuilder, joints_by_group


async def main() -> None:
    # --- swap these two lines for hardware -------------------------------------------------
    # from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth
    # auth = UserAuth.from_env().require()
    # signaling = SupabaseSignaling(url, anon_key, room="NORI-A3-0001", token_provider=auth.token)
    # async with RemoteTeleop(signaling) as robot:
    async with mock_session(MockRobot()) as robot:
        # 1. WHAT AM I ATTACHED TO? Always first. Never hard-code a joint list: the same
        #    script has to work on a 5-DOF arm and a 7-DOF one, and the robot is the
        #    authority on which it is.
        info = await robot.wait_ready()
        groups = joints_by_group(info.descriptor)
        print(f"protocol v{info.protocol_version}, units={info.norm_mode}")
        for group, dofs in groups.items():
            print(f"  {group}: {', '.join(dofs)}")

        # 2. IS MOTION ACTUALLY LIVE? The peer connection can be perfect while the motion
        #    stack is down, and in that state the robot DROPS control frames silently. A
        #    script that skips this check looks like a robot ignoring it, with no error.
        status = robot.daemon_status
        if status is not None and not status.online:
            print(f"motion offline ({status.reason}) -- nothing would move")
            return

        # 3. DRIVE THE BASE. jog() streams at 20 Hz for the duration and then sends an
        #    explicit zero. That matters: the robot's watchdog treats SILENCE as an absent
        #    operator, so a jog stream that stops IS a stop command. The keys are
        #    linear/angular -- NOT the x.vel/theta.vel the descriptor lists, which are the
        #    telemetry namespace and would read as a full stop.
        await robot.jog(JogBuilder(info.descriptor).base(linear=0.4).build(), duration=1.0)
        print(f"after driving: {robot.telemetry.state.get('x.vel')} forward")

        # 4. AN ABSOLUTE MOVE, and wait for the real verdict. wait=True returns only on a
        #    TERMINAL state -- "accepted" just means the robot took the target. `clamped`
        #    means it finished somewhere other than asked, so check .succeeded, not .done.
        side = "left_arm" if "left_arm" in groups else next(iter(groups), None)
        if side and "gripper" in groups.get(side, []):
            result = await robot.action({f"{side}_gripper.pos": 30.0}, wait=True)
            print(f"gripper -> 30: {result.state} (reached={result.succeeded})")

        # 5. RECORD AN EPISODE. session_start opens the dataset, episode_start begins a take.
        #    Note episode_discard deletes it from disk -- it is not an undo.
        await robot.record("session_start")
        episode = await robot.record("episode_start", task="demo pick")
        print(f"recording {episode.episode}, {episode.free_gb} GB free")

        if side:
            for dof in groups[side][:1]:
                await robot.jog(
                    JogBuilder(info.descriptor).arm(side, dof, 0.5).build(), duration=0.5
                )

        stopped = await robot.record("episode_stop")
        print(f"kept {stopped.episodes_kept} episode(s)")
        await robot.record("session_end")

        # 6. READ THE STREAM. Same API shape as any other frame kind.
        async for telemetry in robot.stream("telemetry"):
            moved = {k: round(v, 1) for k, v in telemetry.state.items() if abs(v) > 0.01}
            print(f"safety={telemetry.safety} moved={moved}")
            break

        # 7. STOP. estop() latches: motion stays blocked until reset_latch(). It is
        #    deliberately synchronous and un-awaited so it can never queue behind anything.
        robot.estop()
        print("estopped -- latched until reset_latch()")


if __name__ == "__main__":
    asyncio.run(main())
