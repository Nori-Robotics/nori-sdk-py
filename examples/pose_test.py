"""Runbook harness: docs/runbooks/pose-targets-sdk-hardware-test.md (nori_ws).

Executes steps 3-8 of the pose-targets hardware runbook against a live robot and
prints a per-step verdict table. Steps 1, 2 and 9 are robot-side (motion stack,
gateway, teardown) and are NOT done here.

    export NORI_SUPABASE_URL=... NORI_SUPABASE_ANON_KEY=... NORI_EMAIL=... NORI_PASSWORD=...
    python examples/pose_test.py noriA3-0 --tcp 0.31,-0.18,0.62

--tcp is the arm's CURRENT gripper TCP in base_footprint metres. The runbook says to
take it "from live telemetry FK", but telemetry carries no TCP field -- the gateway
resolves TCP internally via tf and never puts it on the wire. Read it on the robot:

    ros2 run tf2_ros tf2_echo base_footprint right_gripper_tcp_link

and pass the translation here. Without it every pose target is a guess, and a blind
guess near the dangling rest pose refuses as no_ik_solution for reasons that have
nothing to do with the code under test.

Safety: --side defaults to right. The left bus on noriA3-0 has the servo-10
early-reply fault and the runbook puts it off-limits; passing --side left is
therefore an explicit, deliberate act.
"""

import argparse
import asyncio
import contextlib
import os
import sys
import time
import uuid

from nori_sdk import RemoteTeleop, SupabaseSignaling, TeleopError, UserAuth, protocol

# ---------------------------------------------------------------- result recording

PASS, FAIL, SKIP, INFO = "PASS", "FAIL", "SKIP", "----"
_results: list[tuple[str, str, str]] = []


def record(step: str, verdict: str, detail: str = "") -> None:
    _results.append((step, verdict, detail))
    print(f"  [{verdict}] {step}: {detail}", flush=True)


def banner(text: str) -> None:
    print(f"\n{'=' * 78}\n{text}\n{'=' * 78}", flush=True)


def summary() -> int:
    banner("SUMMARY")
    width = max((len(s) for s, _, _ in _results), default=10)
    for step, verdict, detail in _results:
        print(f"{step.ljust(width)}  {verdict}  {detail}")
    failed = [s for s, v, _ in _results if v == FAIL]
    print(f"\n{len(_results)} checks, {len(failed)} failed"
          + (f": {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


def fmt(status, elapsed: float) -> str:
    """One-line rendering of an ActionStatus plus how long it took."""
    if status is None:
        return f"no status (fire-and-forget) [{elapsed:.2f}s]"
    reason = f" reason={status.reason}" if status.reason else ""
    return f"state={status.state}{reason} [{elapsed:.2f}s]"


async def timed(coro):
    """Await, returning (result_or_exception, elapsed_seconds)."""
    t0 = time.monotonic()
    try:
        return await coro, time.monotonic() - t0
    except Exception as exc:
        return exc, time.monotonic() - t0


async def ask(prompt: str) -> str:
    """input() without blocking the event loop -- the jog keepalive must keep running."""
    return (await asyncio.to_thread(input, prompt)).strip()


# ---------------------------------------------------------------- steps

async def step3_capabilities(robot, room: str) -> object:
    banner("STEP 3 - connect and discover capabilities")
    info = await robot.wait_ready()
    print(f"  model={getattr(info, 'model', None)} norm_mode={getattr(info, 'norm_mode', None)}")
    print(f"  capabilities={info.capabilities}")

    if info.capabilities is None:
        record("3-capabilities", FAIL,
               "capabilities is None -- gateway predates 2026-08-23; wrong deploy")
    elif not info.capabilities:
        record("3-capabilities", FAIL,
               "capabilities is empty -- motion adapter absent; check enable_motion "
               "and the gateway log")
    else:
        record("3-capabilities", PASS, f"{len(info.capabilities)} advertised")

    supports = info.supports("pose_targets")
    record("3-pose_targets", PASS if supports else FAIL, f"supports()={supports}")
    record("3-task_jog", PASS if info.supports("task_jog") else FAIL,
           f"supports()={info.supports('task_jog')}")

    descriptor = getattr(info, "descriptor", None)
    jog_scale = getattr(descriptor, "jog_scale", None) if descriptor else None
    aux = getattr(descriptor, "aux", None) if descriptor else None
    record("3-descriptor", PASS if jog_scale is not None else FAIL,
           f"jog_scale={jog_scale} aux={aux}")

    daemon = robot.daemon_status
    if daemon is not None and not daemon.online:
        record("3-daemon", FAIL,
               f"daemon offline: {getattr(daemon, 'detail', '') or daemon.state} "
               "-- motion frames will be dropped; fix before step 4")
    else:
        record("3-daemon", PASS, f"online (state={getattr(daemon, 'state', '?')})")
    return info


async def step3b_lift_state(robot, interactive: bool) -> None:
    """The fabricated-lift trap, checked from the operator side.

    With no active (homed) Pico component the joint_state_broadcaster publishes
    lift_extension_joint = 0.0 as a DEFAULT, which is fiction indistinguishable from a
    measurement. Everything downstream inherits it: every base_footprint z is off by the
    true lift height, and near-floor poses refuse as `collision` against a wheel the
    gripper is nowhere near. Steps 6a and 8 are the ones that eat this.

    A constant 0.0 is only SUSPICIOUS, not proof -- a genuinely retracted lift also reads
    0.0. The disambiguation is movement, which needs a human, so it is offered rather
    than assumed."""
    banner("STEP 3b - is the lift state live or fabricated?")
    samples = []
    for _ in range(12):
        state = getattr(robot.telemetry, "state", None) or {}
        if "lift.pos" in state:
            samples.append(float(state["lift.pos"]))
        await asyncio.sleep(0.25)

    if not samples:
        record("3b-lift", FAIL,
               "no lift.pos in telemetry at all -- expected on an A3 (mm, range 0-720). "
               "Check the Pico component is present.")
        return

    lo, hi = min(samples), max(samples)
    print(f"  lift.pos over {len(samples)} frames: min={lo:.1f} max={hi:.1f} mm")

    if lo == hi == 0.0:
        record("3b-lift", INFO,
               "lift.pos is a constant 0.0 -- either genuinely retracted OR the "
               "fabricated default from an inactive Pico. Cannot tell from here.")
        if not interactive:
            print("  re-run with --check-lift to disambiguate, or verify on the robot:")
            print("    ros2 control list_hardware_components   # expect an ACTIVE Pico")
            return
        await ask("    move the lift a few cm by hand/jog, then press Enter: ")
        after = []
        for _ in range(12):
            state = getattr(robot.telemetry, "state", None) or {}
            if "lift.pos" in state:
                after.append(float(state["lift.pos"]))
            await asyncio.sleep(0.25)
        moved = after and (max(after) - min(after) > 1.0 or abs(max(after) - hi) > 1.0)
        record("3b-lift", PASS if moved else FAIL,
               f"lift.pos now min={min(after):.1f} max={max(after):.1f} mm -- "
               + ("tracks the physical lift, state is live"
                  if moved else
                  "did NOT follow the physical lift: the 0.0 is FABRICATED. Every z is "
                  "wrong and near-floor collisions are phantom. Do not trust 6a or 8."))
    else:
        record("3b-lift", PASS,
               f"lift.pos non-zero/varying ({lo:.1f}-{hi:.1f} mm) -- a real measurement")


async def step4_first_pose(robot, side: str, tcp: list[float]) -> None:
    banner("STEP 4 - heartbeat + first pose (the end-to-end gap check)")
    robot.set_jog({})  # SDK repeats this at JOG_HZ for the whole session
    await asyncio.sleep(0.5)
    record("4-heartbeat", PASS, "empty jog streaming (SDK-owned repeat)")

    print(f"  commanding current TCP {tcp} -- position-only, current wrist")
    outcome, elapsed = await timed(robot.pose(side, tcp, wait=True))

    if isinstance(outcome, TeleopError):
        record("4-first-pose", FAIL,
               f"TeleopError after {elapsed:.1f}s -- THE END-TO-END GAP. "
               f"Capture gateway logs and stop here. ({outcome})")
    elif isinstance(outcome, Exception):
        record("4-first-pose", FAIL, f"{type(outcome).__name__}: {outcome}")
    elif outcome.state in ("done", "clamped"):
        record("4-first-pose", PASS, fmt(outcome, elapsed))
    elif outcome.state == "blocked":
        record("4-first-pose", PASS,
               f"honest refusal, wire path works -- {fmt(outcome, elapsed)}")
    else:
        record("4-first-pose", FAIL, f"non-terminal verdict {fmt(outcome, elapsed)}")


async def step5_posture(robot, side: str) -> None:
    banner("STEP 5 - working posture (lift off the near-singular rest pose)")
    targets = {f"{side}_arm_elbow_pitch.pos": 8.0, f"{side}_arm_wrist_pitch.pos": -10.0}
    print(f"  action {targets}")
    outcome, elapsed = await timed(robot.action(targets, wait=True))
    if isinstance(outcome, Exception):
        record("5-posture", FAIL, f"{type(outcome).__name__}: {outcome}")
    elif outcome.state in ("done", "clamped"):
        record("5-posture", PASS, fmt(outcome, elapsed) + " -- arm should have visibly lifted")
    else:
        record("5-posture", FAIL, fmt(outcome, elapsed))


async def pose_step(robot, side, label, position, *, expect: str, note: str = "",
                    orientation=None, timeout: float = 12.0) -> object:
    """One rung of the step-6 ladder. `expect` is prose for the report, not an assertion:
    the runbook's own point is that several rungs have two legitimate outcomes."""
    print(f"  {label}: pose {[round(v, 3) for v in position]}  (expect: {expect})")
    outcome, elapsed = await timed(
        robot.pose(side, position, orientation, wait=True, timeout=timeout))

    if isinstance(outcome, TeleopError):
        record(label, FAIL,
               f"no terminal state in {elapsed:.1f}s -- CRITICAL: the no-progress guard "
               f"should have answered. Capture /{side}_servo/status and the gateway log.")
    elif isinstance(outcome, Exception):
        record(label, FAIL, f"{type(outcome).__name__}: {outcome}")
    else:
        detail = fmt(outcome, elapsed)
        if note:
            detail += f" -- {note}"
        record(label, PASS if outcome.done else FAIL, detail)
    return outcome


async def step6_ladder(robot, side: str, tcp: list[float], far_reach: float,
                       far_target: list[float] | None = None) -> None:
    banner("STEP 6 - the pose ladder")
    x, y, z = tcp

    # 6a - 3 cm above the current TCP.
    up = [x, y, z + 0.03]
    got = await pose_step(robot, side, "6a-up-3cm", up,
                          expect="done in a few seconds, arm moves up",
                          note="if 'blocked collision' with no motion: known spurious-SRDF "
                               "debt -- record the target and retry 10 cm higher")

    # 6b - back to where 6a started, only meaningful if 6a actually moved.
    if got is not None and not isinstance(got, Exception) and got.state == "done":
        await pose_step(robot, side, "6b-return", [x, y, z],
                        expect="done, settles within ~2 cm")
    else:
        record("6b-return", SKIP, "6a did not reach its target; nothing to return from")

    # 6c - 2 m away: unreachable, must refuse fast with zero motion.
    await pose_step(robot, side, "6c-2m-away", [x + 2.0, y, z],
                    expect="blocked no_ik_solution in ~0.2 s, zero motion", timeout=6.0)

    # 6d - a frame the robot must reject. The SDK builder pins frame=base_footprint,
    # so this one is constructed by hand and sent raw; there is no public API for an
    # invalid frame, by design.
    print('  6d: raw control frame with frame="tool" (SDK builder cannot express it)')
    action_id = uuid.uuid4().hex[:12]
    raw = protocol.control_pose(robot._next_seq(), side, [x, y, z], None, action_id)
    raw["pose"][f"{side}_arm"]["frame"] = "tool"
    future = asyncio.get_running_loop().create_future()
    robot._pending_actions[action_id] = future
    t0 = time.monotonic()
    try:
        robot._send(raw)
        status = await asyncio.wait_for(future, 6.0)
        elapsed = time.monotonic() - t0
        ok = status.state == "blocked" and (status.reason or "").startswith("frame")
        record("6d-bad-frame", PASS if ok else FAIL, fmt(status, elapsed))
    except TimeoutError:
        record("6d-bad-frame", FAIL,
               "no reply in 6 s -- an unknown frame must be refused, not ignored")
    except Exception as exc:
        record("6d-bad-frame", FAIL, f"{type(exc).__name__}: {exc}")
    finally:
        robot._pending_actions.pop(action_id, None)

    # 6e - far but reachable: expect config_jump, then reach it by waypointing.
    # An UNREACHABLE target returns no_ik_solution and tests nothing, so this is
    # explicitly overridable once the workspace is known.
    far = list(far_target) if far_target else [x, y + far_reach, z]
    got = await pose_step(robot, side, "6e-far-jump", far,
                          expect=f"blocked config_jump ({far_reach:+.2f} m in y)")
    reason = getattr(got, "reason", None) if not isinstance(got, Exception) else None
    if reason == "config_jump":
        print("  6e: refused as designed -- now the same point in 3 waypoints")
        ok = True
        for i in range(1, 4):
            way = [x + (far[0] - x) * i / 3.0,
                   y + (far[1] - y) * i / 3.0,
                   z + (far[2] - z) * i / 3.0]
            leg = await pose_step(robot, side, f"6e-waypoint-{i}", way, expect="done")
            if isinstance(leg, Exception) or leg.state != "done":
                ok = False
                break
        record("6e-waypointed", PASS if ok else FAIL,
               "reached by waypoints" if ok else "waypointing did not reach the target")
    else:
        record("6e-waypointed", SKIP, f"6e did not refuse as config_jump (reason={reason})")

    # 6f - a task jog must supersede an in-flight pose. Built by hand rather than with
    # JogBuilder: the task verbs (x/y/z/pitch/shoulder_pan) are not joint names, so
    # strict-mode JogBuilder.arm() rejects them.
    print("  6f: pose in flight, then a task jog (+z) on the same arm")
    tracked = uuid.uuid4().hex[:12]
    future = asyncio.get_running_loop().create_future()
    robot._pending_actions[tracked] = future
    t0 = time.monotonic()
    try:
        robot._send(protocol.control_pose(
            robot._next_seq(), side, [x, y, z + 0.05], None, tracked))
        await asyncio.sleep(0.3)
        robot.set_jog({f"{side}_arm": {"z": 0.3}})
        status = await asyncio.wait_for(future, 6.0)
        elapsed = time.monotonic() - t0
        ok = status.state == "blocked" and status.reason == "superseded"
        record("6f-superseded", PASS if ok else FAIL, fmt(status, elapsed))
    except TimeoutError:
        record("6f-superseded", FAIL, "pose never answered after a jog took the arm")
    except Exception as exc:
        record("6f-superseded", FAIL, f"{type(exc).__name__}: {exc}")
    finally:
        robot._pending_actions.pop(tracked, None)
        robot.set_jog({})  # back to bare keepalive
        await asyncio.sleep(0.5)
    print("  6f: confirm by eye -- the arm should NOT snap back to the pose target")


async def step7_safety(robot, side: str, tcp: list[float],
                       reach: float = 0.13) -> None:
    banner("STEP 7 - safety honesty (watchdog, then E-stop)")
    x, y, z = tcp
    # The interruption must land while the pose is still tracking. A target the arm
    # has already reached returns `done` in ~0.3 s and the test proves nothing.
    far_z = z + reach
    print(f"  using a {reach*100:.0f} cm move (z -> {far_z:.3f}) so there is a"
          f" move in flight to interrupt")

    # 7.1 watchdog: a pose in flight, then total control silence.
    print("  7.1: pose in flight, then killing the heartbeat entirely")
    action_id = uuid.uuid4().hex[:12]
    future = asyncio.get_running_loop().create_future()
    robot._pending_actions[action_id] = future
    t0 = time.monotonic()
    try:
        robot._send(protocol.control_pose(
            robot._next_seq(), side, [x, y, far_z], None, action_id))
        await asyncio.sleep(0.15)
        await robot.stop_jog()          # one zero frame, then silence
        status = await asyncio.wait_for(future, 8.0)
        elapsed = time.monotonic() - t0
        ok = status.state in ("timeout", "blocked")
        record("7.1-watchdog", PASS if ok else FAIL, fmt(status, elapsed))
    except TimeoutError:
        record("7.1-watchdog", FAIL,
               "control silence did not terminate the pose -- the watchdog is the "
               "last line of defence here")
    except Exception as exc:
        record("7.1-watchdog", FAIL, f"{type(exc).__name__}: {exc}")
    finally:
        robot._pending_actions.pop(action_id, None)

    # t_stop is 1000 ms on the WAN profile and stop_jog() itself sends one zero frame,
    # so safety cannot be read until well past a full second of real silence.
    await asyncio.sleep(2.5)
    safety = getattr(robot.telemetry, "safety", None)
    record("7.1-safe_hold", PASS if safety == "safe_hold" else FAIL,
           f"telemetry.safety={safety!r} (expected 'safe_hold')")

    robot.set_jog({})   # resume frames; must NOT resume the dead pose
    await asyncio.sleep(1.0)
    print("  7.1: frames resumed -- confirm by eye the arm did NOT continue the old pose")

    # 7.2 E-stop.
    print("  7.2: pose in flight, then estop()")
    action_id = uuid.uuid4().hex[:12]
    future = asyncio.get_running_loop().create_future()
    robot._pending_actions[action_id] = future
    t0 = time.monotonic()
    try:
        robot._send(protocol.control_pose(
            robot._next_seq(), side, [x, y, far_z], None, action_id))
        await asyncio.sleep(0.15)
        robot.estop()
        status = await asyncio.wait_for(future, 8.0)
        elapsed = time.monotonic() - t0
        ok = status.state == "blocked" and "estop" in (status.reason or "")
        record("7.2-estop", PASS if ok else FAIL, fmt(status, elapsed))
    except TimeoutError:
        record("7.2-estop", FAIL, "E-stop did not terminate the in-flight pose")
    except Exception as exc:
        record("7.2-estop", FAIL, f"{type(exc).__name__}: {exc}")
    finally:
        robot._pending_actions.pop(action_id, None)

    # Motion must stay refused until the latch is cleared.
    outcome, elapsed = await timed(robot.pose(side, [x, y, z], wait=True, timeout=6.0))
    if isinstance(outcome, Exception):
        record("7.2-latched", FAIL,
               f"pose while latched raised instead of refusing: {outcome}")
    else:
        ok = outcome.state == "blocked"
        record("7.2-latched", PASS if ok else FAIL,
               fmt(outcome, elapsed) + " (must refuse while latched)")

    robot.reset_latch()
    await asyncio.sleep(1.0)
    outcome, elapsed = await timed(robot.pose(side, [x, y, z], wait=True, timeout=8.0))
    if isinstance(outcome, Exception):
        record("7.2-reset", FAIL, f"{type(outcome).__name__}: {outcome}")
    else:
        record("7.2-reset", PASS if outcome.done else FAIL,
               fmt(outcome, elapsed) + " (motion restored after reset_latch)")


async def step8_accuracy(robot, side: str, tcp: list[float]) -> None:
    banner("STEP 8 - world-accuracy annex (interactive: needs a ruler)")
    print("  base_footprint origin is on the FLOOR under the axle centre.")
    print("  Enter measured x,y,z in metres, or blank to skip a target.\n")
    x, y, z = tcp
    targets = [
        [x + 0.10, y, z],
        [x, y + 0.10, z],
        [x, y, z + 0.12],
        [x + 0.08, y - 0.08, z + 0.08],
        [x, y, max(z - 0.10, 0.10)],
    ]
    rows = []
    for i, target in enumerate(targets, 1):
        outcome, elapsed = await timed(robot.pose(side, target, wait=True, timeout=15.0))
        if isinstance(outcome, Exception) or outcome.state != "done":
            record(f"8-target-{i}", SKIP,
                   f"not reached: {fmt(outcome if not isinstance(outcome, Exception) else None, elapsed)}"
                   f"{outcome if isinstance(outcome, Exception) else ''}")
            continue
        raw = await ask(f"    target {i} commanded {[round(v, 3) for v in target]} -> measured: ")
        if not raw:
            record(f"8-target-{i}", SKIP, "no measurement taken")
            continue
        try:
            measured = [float(v) for v in raw.replace(",", " ").split()]
            if len(measured) != 3:
                raise ValueError
        except ValueError:
            record(f"8-target-{i}", SKIP, f"unparseable measurement {raw!r}")
            continue
        err = [m - c for m, c in zip(measured, target)]
        rows.append((target, measured, err))
        record(f"8-target-{i}", INFO,
               f"error dx={err[0]*1000:+.0f} dy={err[1]*1000:+.0f} dz={err[2]*1000:+.0f} mm")

    if rows:
        banner("STEP 8 RESULTS - paste into the runbook's results section")
        print("| commanded (m) | measured (m) | error (mm) |")
        print("|---|---|---|")
        for target, measured, err in rows:
            c = ", ".join(f"{v:.3f}" for v in target)
            m = ", ".join(f"{v:.3f}" for v in measured)
            e = ", ".join(f"{v*1000:+.0f}" for v in err)
            print(f"| {c} | {m} | {e} |")


# ---------------------------------------------------------------- driver

async def run(args) -> int:
    url = os.environ.get("NORI_SUPABASE_URL", "")
    anon = os.environ.get("NORI_SUPABASE_ANON_KEY", "")
    auth = UserAuth.from_env()

    if auth is not None:
        auth.require()
        token_provider = auth.token
        print("auth: private room (paired user)")
    elif args.public:
        # Bench-only. The token_provider's PRESENCE is what flips the join to
        # private, so passing None joins a public channel with the anon key. This
        # does NOT exercise the pairing RLS a fleet client goes through -- the
        # robot must also be running with private_room: false or it will never
        # see us.
        if not url:
            sys.exit("--public still needs NORI_SUPABASE_URL (and the anon key)")
        token_provider = None
        print("auth: PUBLIC room (bench only) -- pairing/RLS is NOT under test here")
    else:
        sys.exit("set NORI_SUPABASE_URL / NORI_SUPABASE_ANON_KEY / NORI_EMAIL / "
                 "NORI_PASSWORD, or pass --public for a bench public room")

    signaling = SupabaseSignaling(url, anon, room=args.room,
                                  token_provider=token_provider)

    async with RemoteTeleop(signaling, on_log=print) as robot:
        info = await step3_capabilities(robot, args.room)
        if info.supports("pose_targets") is False:
            print("\npose_targets not advertised -- steps 4-8 cannot run.")
            return summary()

        try:
            await step3b_lift_state(robot, args.check_lift)
            await step4_first_pose(robot, args.side, args.tcp)
            if not args.no_posture:
                await step5_posture(robot, args.side)
            await step6_ladder(robot, args.side, args.tcp, args.far_reach,
                               args.far_target)
            if not args.no_safety:
                await step7_safety(robot, args.side, args.tcp, args.safety_reach)
            if args.accuracy:
                await step8_accuracy(robot, args.side, args.tcp)
        finally:
            with contextlib.suppress(Exception):
                await robot.stop_jog()

    return summary()


def parse_tcp(text: str) -> list[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--tcp wants three numbers: x,y,z in metres")
    return [float(p) for p in parts]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("room", help="signaling room = robot hostname, e.g. noriA3-0")
    ap.add_argument("--tcp", type=parse_tcp, required=True,
                    help="current gripper TCP x,y,z in base_footprint metres "
                         "(tf2_echo base_footprint <side>_gripper_tcp_link)")
    ap.add_argument("--side", default="right", choices=["left", "right"],
                    help="left is off-limits on noriA3-0 (servo-10 fault)")
    ap.add_argument("--far-reach", type=float, default=0.35,
                    help="step 6e lateral offset in metres (default 0.35)")
    ap.add_argument("--far-target", type=parse_tcp, default=None,
                    help="explicit step-6e target x,y,z (must be REACHABLE but far in "
                         "joint space, or it returns no_ik_solution and tests nothing)")
    ap.add_argument("--safety-reach", type=float, default=0.13,
                    help="step-7 move size in metres; must be big enough that the pose "
                         "is still in flight when it is interrupted (default 0.13)")
    ap.add_argument("--no-posture", action="store_true", help="skip step 5")
    ap.add_argument("--no-safety", action="store_true", help="skip step 7")
    ap.add_argument("--public", action="store_true",
                    help="join a PUBLIC signaling room (no user login). Bench only, "
                         "and the robot must have private_room: false")
    ap.add_argument("--check-lift", action="store_true",
                    help="interactively disambiguate a constant lift.pos of 0.0 "
                         "(you move the lift, the harness watches telemetry)")
    ap.add_argument("--accuracy", action="store_true",
                    help="run the step-8 accuracy annex (interactive, needs a ruler)")
    args = ap.parse_args()
    raise SystemExit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
