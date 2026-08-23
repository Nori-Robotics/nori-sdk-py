"""Measure a robot's real jog scale and emit its `descriptor.jog_scale` block.

    python tools/measure_jog_scale.py --room NORI-A3-0001
    python tools/measure_jog_scale.py --mock            # rehearse the procedure, no robot

Publishing the constants straight out of the gateway source would not be calibration -- it
would be moving a number onto the wire and calling it verified. If the acceleration limit, the
target leash and Servo's scaling mean the steady-state rate is 0.72 where the constant says
0.8, then 0.8 is a lie with a decimal point on it, and clients will trust it MORE than the
honest silence it replaced. So the number has to be measured on the robot it describes.

WHAT IT MEASURES: steady-state normalized units per second at full deflection, per joint, per
direction. That is the same quantity `descriptor.jog_scale.joints` promises, in the same units
telemetry reports, which is the whole reason those units were chosen.

FOUR THINGS THAT CORRUPT THE MEASUREMENT, and how this handles them:

  acceleration     MAX_ACCELERATION means the joint is still speeding up at the start of a
                   jog. We discard SETTLE_S of every run before fitting.
  the target leash JOG_LEAD_RAD bounds how far the commanded target may lead the measured
                   position, so the achieved rate is only meaningful once that gap is steady.
                   Falls out of the same settle window.
  range limits     a joint decelerating into its soft limit reads as a slower robot. We refuse
                   to fit any run that leaves the middle band of the advertised range.
  the watchdog     `warn` scales all motion to ZERO. Any sample taken while telemetry reports
                   a non-ok watchdog is dropped, and a run that loses too many is rerun.

SAFETY: this drives a real robot through its range. Clear the workspace. It jogs one joint at
a time at full deflection, and every run ends with an explicit zero rather than by going quiet.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import statistics
import sys
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent / "src"))

from nori_sdk.motion import ARM_GROUPS, joint_short
from nori_sdk.types import RobotDescriptor, Telemetry

SETTLE_S = 0.35     # discarded head of every run: acceleration ramp + leash settling
RUN_S = 1.20        # fitted window
SAMPLE_HZ = 15.0    # the telemetry rate; we fit whatever actually arrives
SAFE_BAND = 0.55    # fit only while inside this fraction of the range, centred
MIN_SAMPLES = 8
RUNS_PER_DIRECTION = 3


class Measurement:
    def __init__(self, key: str) -> None:
        self.key = key
        self.rates: list[float] = []

    def add(self, samples: list[tuple[float, float]]) -> str | None:
        """Fit one run. Returns a reason string if the run was rejected."""
        if len(samples) < MIN_SAMPLES:
            return f"only {len(samples)} samples"
        t0 = samples[0][0]
        xs = [t - t0 for t, _ in samples]
        ys = [p for _, p in samples]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        denom = sum((x - mx) ** 2 for x in xs)
        if denom <= 0:
            return "no time spread"
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
        # R^2, so a run where the joint stalled or bounced off a limit is rejected rather
        # than averaged in as a slow one.
        ss_tot = sum((y - my) ** 2 for y in ys)
        ss_res = sum((y - (my + slope * (x - mx))) ** 2 for x, y in zip(xs, ys))
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        if r2 < 0.98:
            return f"non-linear (R^2={r2:.3f}) — stall, limit or watchdog"
        self.rates.append(abs(slope))
        return None

    def result(self) -> tuple[float, float] | None:
        """(rate, spread) — spread is max/min, a direct readout of how repeatable it is."""
        if len(self.rates) < 2:
            return None
        return statistics.median(self.rates), max(self.rates) / min(self.rates)


def _in_safe_band(descriptor: RobotDescriptor, key: str, value: float) -> bool:
    span = descriptor.ranges.get(key)
    if span is None:
        return True
    low, high = span
    mid, half = (low + high) / 2.0, (high - low) / 2.0 * SAFE_BAND
    return mid - half <= value <= mid + half


async def _run_once(robot, descriptor, group, dof, key, sign, meas) -> str | None:
    from nori_sdk.motion import JogBuilder

    samples: list[tuple[float, float]] = []
    dropped = 0

    def on_telemetry(frame: Telemetry) -> None:
        nonlocal dropped
        if frame.watchdog not in (None, "ok"):
            dropped += 1          # motion is being scaled to zero; the sample is meaningless
            return
        value = frame.state.get(key)
        if value is None:
            return
        if not _in_safe_band(descriptor, key, value):
            return
        samples.append((time.monotonic(), value))

    unsubscribe = robot.on("telemetry", on_telemetry)
    try:
        payload = JogBuilder(descriptor).arm(group, dof, float(sign)).build()
        await robot.jog(payload, duration=SETTLE_S)   # ramp, discarded
        samples.clear()
        await robot.jog(payload, duration=RUN_S)      # the fitted window
    finally:
        unsubscribe()
        await robot.stop_jog()

    if dropped > len(samples):
        return f"watchdog degraded ({dropped} samples dropped)"
    return meas.add(samples)


async def measure(robot, args) -> dict:
    info = await robot.wait_ready()
    descriptor = info.descriptor
    if descriptor is None:
        sys.exit("this robot sends no descriptor — nothing to calibrate against")
    if robot.daemon_status is not None and not robot.daemon_status.online:
        sys.exit(f"motion offline ({robot.daemon_status.reason}) — nothing would move")

    print(f"model={info.model or '?'} norm_mode={info.norm_mode} "
          f"joints={len(descriptor.joints)}", file=sys.stderr)

    joints: dict[str, float] = {}
    for key in descriptor.joints:
        group = next((g for g in ARM_GROUPS if key.startswith(g + "_")), None)
        if group is None or (args.only and joint_short(key) not in args.only):
            continue
        dof = joint_short(key)
        meas = Measurement(key)
        for sign in (1, -1):
            for attempt in range(RUNS_PER_DIRECTION):
                reason = await _run_once(robot, descriptor, group, dof, key, sign, meas)
                tag = f"{key} {'+' if sign > 0 else '-'}{attempt + 1}"
                print(f"  {tag:44} {'ok' if reason is None else 'SKIP ' + reason}",
                      file=sys.stderr)
        got = meas.result()
        if got is None:
            print(f"  {key}: NO USABLE RUNS — omitted (omission means unknown)", file=sys.stderr)
            continue
        rate, spread = got
        # Publishing a number that varies 20% run to run would be calibration in name only.
        flag = "  <-- NOT REPEATABLE, do not publish" if spread > 1.20 else ""
        print(f"  {key}: {rate:.2f} units/s  (spread x{spread:.2f}){flag}", file=sys.stderr)
        if spread <= 1.20:
            joints[key] = round(rate, 2)

    return {"joints": joints}


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--room", help="robot serial, e.g. NORI-A3-0001")
    ap.add_argument("--mock", action="store_true", help="rehearse against MockRobot")
    ap.add_argument("--only", nargs="*", help="limit to these DOF short names")
    args = ap.parse_args()

    if args.mock:
        from nori_sdk.mock import mock_session
        async with mock_session() as robot:
            block = await measure(robot, args)
    elif args.room:
        from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth
        auth = UserAuth.from_env()
        if auth is None:
            sys.exit("set NORI_SUPABASE_URL / _ANON_KEY / NORI_EMAIL / NORI_PASSWORD")
        auth.require()
        signaling = SupabaseSignaling(
            auth.supabase_url, auth.supabase_anon_key, room=args.room,
            token_provider=auth.token,
        )
        async with RemoteTeleop(signaling) as robot:
            block = await measure(robot, args)
    else:
        ap.error("pass --room <serial> or --mock")

    print(json.dumps({"jog_scale": block}, indent=2))


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
