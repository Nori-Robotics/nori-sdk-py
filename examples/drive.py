"""Minimal end-to-end script: connect, inspect the robot, drive it, record an episode.

    export NORI_SUPABASE_URL=... NORI_SUPABASE_ANON_KEY=... NORI_EMAIL=... NORI_PASSWORD=...
    python examples/drive.py NORI-A3-0001

Nothing here is model-specific: every joint and camera name comes from the robot's own
descriptor, so the same script runs against a different Nori without edits.
"""

import asyncio
import logging
import os
import sys

from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth
from nori_sdk.motion import JogBuilder, joints_by_group


async def main(room: str) -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    auth = UserAuth.from_env()
    if auth is None:
        sys.exit("set NORI_SUPABASE_URL / NORI_SUPABASE_ANON_KEY / NORI_EMAIL / NORI_PASSWORD")
    auth.require()

    signaling = SupabaseSignaling(
        os.environ["NORI_SUPABASE_URL"],
        os.environ.get("NORI_SUPABASE_ANON_KEY", ""),
        room=room,
        token_provider=auth.token,
    )

    async with RemoteTeleop(signaling, on_log=print) as robot:
        info = await robot.wait_ready()
        print(f"connected to {room}: {info.norm_mode}, joints={joints_by_group(info.descriptor)}")
        print(f"cameras: {info.descriptor.cameras if info.descriptor else 'unknown'}")

        # Motion is dropped while the daemon is offline, so say so rather than looking hung.
        if robot.daemon_status is not None and not robot.daemon_status.online:
            print(f"motion offline ({robot.daemon_status.detail}) — nothing will move")
            return

        # Nudge the base forward. jog() streams at 20 Hz for the duration and then sends an
        # explicit zero: the robot's watchdog would stop it on silence anyway, but an
        # explicit stop is immediate rather than a ramp-down.
        # linear/angular, NOT the descriptor's x.vel/theta.vel — see JogBuilder.base().
        await robot.jog(JogBuilder(info.descriptor).base(linear=0.3).build(), duration=1.0)

        # Absolute move on the first joint this robot happens to have. wait=True returns only
        # on a terminal verdict; `clamped` means it finished somewhere other than asked.
        if info.descriptor and info.descriptor.joints:
            joint = info.descriptor.joints[0]
            status = await robot.action({joint: 0.0}, wait=True)
            verdict = "reached" if status.succeeded else f"ended {status.state}"
            print(f"{joint} -> 0.0: {verdict}{' (' + status.reason + ')' if status.reason else ''}")

        # One recorded episode.
        await robot.record("session_start")
        episode = await robot.record("episode_start", task="demo")
        print(f"recording {episode.episode}, {episode.free_gb} GB free")
        await robot.jog(JogBuilder(info.descriptor).base(angular=0.3).build(), duration=1.0)
        print(f"kept {(await robot.record('episode_stop')).episodes_kept} episode(s)")
        await robot.record("session_end")

        telemetry = robot.telemetry
        if telemetry is not None:
            print(f"safety={telemetry.safety} temp={telemetry.pi_temp_c}")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "NORI-A3-0001"))
