# nori-sdk (Python)

Python operator client for Nori robots. Speaks **nori-protocol** over a WebRTC data channel —
the same wire dialect as the TypeScript `@nori/sdk` the web app uses, and the same one the
robot's ROS 2 gateway implements.

It exists for the clients a browser SDK can't serve: headless scripts, policy and agent
drivers, dataset tooling, CI that drives a robot (or a mock) without a browser.

> **Status: v1.0.0.** The pure layers (protocol, types, motion helpers, mock) are complete,
> tested and spec-conformant, and `RemoteTeleop` has driven real hardware — bench A3 sessions
> (2026-08-21/22) shook out the WebRTC interop path and two watchdog bugs. See
> [Status](#status) for exactly what is and isn't hardware-verified.

## Install

```bash
pip install "nori-sdk[all]"        # session + Supabase signaling
pip install nori-sdk               # protocol only, zero dependencies
```

## Start here: no hardware, no credentials

```bash
pip install -e .            # no extras needed — there is no peer connection
python examples/mock_pick_place.py
```

That drives a `MockRobot` through a real session: discover the descriptor, check motion
health, jog the base, command an absolute move and wait for the verdict, record an episode,
read telemetry, E-STOP. Every line runs unchanged against a real robot — **one line differs**:

```python
async with mock_session() as robot:                          # development
async with RemoteTeleop(SupabaseSignaling(...)) as robot:    # hardware
```

`mock_session()` is the supported way to develop against the double; don't reach for
`teleop._control` or `_handle_frame`, which the test suite uses and which carry no
compatibility promise. Pass a configured robot to rehearse what hardware won't produce on
demand — `MockRobot(online=False)` (motion stack down), `accepted=False` (session refused),
`descriptor=None` (legacy robot, no descriptor), `cameras=False` (no camera layout),
`action_outcome="clamped"` (a move that lands somewhere other than commanded).

The mock **enforces the watchdog**: control-frame silence past `t_stop_ms` stops the motion
and reports `safe_hold`, and `link("lan"|"wan")` selects which profile it enforces. That is
deliberate — it is the one rule a script can violate and still appear to work locally, so the
double has to punish it here rather than let hardware do it. It also integrates a pose, so
telemetry responds to what you commanded.

**What a green mock run does not prove:** ICE, TURN, bandwidth, video, or real timing. It
means your logic is right, not that your network is. Also not produced by the mock:
`perception` and `error` frames (both are modelled and parse — the double just never
emits them), motor faults, thermals, and a daemon that goes offline mid-session.

## Quickstart (against a robot)

```python
import asyncio
from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth
from nori_sdk.motion import JogBuilder

async def main():
    auth = UserAuth(SUPABASE_URL, ANON_KEY, "me@example.com", "password")
    signaling = SupabaseSignaling(
        SUPABASE_URL, ANON_KEY, room="NORI-A3-0001", token_provider=auth.token
    )

    async with RemoteTeleop(signaling) as robot:
        info = await robot.wait_ready()
        print(info.descriptor.joints)          # never hard-code a joint list

        jog = JogBuilder(info.descriptor).base(linear=0.4).build()
        await robot.jog(jog, duration=1.5)     # streams at 20 Hz, then stops cleanly

        await robot.action({"left_arm_gripper.pos": 30}, wait=True)

        async for telemetry in robot.stream("telemetry"):
            print(telemetry.state)
            break

asyncio.run(main())
```

**The one thing to internalize:** the robot's watchdog treats silence as an absent operator.
A jog stream that stops *is* a stop command. `jog(payload, duration=...)` handles the
repetition for you; if you drive the stream yourself, resend inside
`info.watchdog_profile.t_warn_ms` (150 ms on LAN, 300 ms on WAN).

## Layering

Mirrors the TypeScript package's subpath exports, so you only pay for what you use. The heavy
imports are lazy — `import nori_sdk` never pulls in aiortc.

| Module | Needs | What it is |
|---|---|---|
| `nori_sdk.protocol`, `.types`, `.motion` | nothing | Build and parse every frame; descriptor-driven jog/action helpers |
| `nori_sdk.signaling` | nothing | The transport contract — bring your own |
| `nori_sdk.teleop` | `aiortc` | The live session (`RemoteTeleop`) |
| `nori_sdk.signaling_supabase` | `websocket-client` | Reference Supabase Realtime transport |
| `nori_sdk.mock` | nothing | `mock_session()`, `MockRobot`, loopback signaling — hardware-free development and CI |

## API reference

Everything below is public and covered by `tests/test_public_api.py`, which pins the surface
so it cannot drift by accident. Anything with a leading underscore is internal and may change
in a patch release — including `teleop._control` and `teleop._handle_frame`, which the test
suite uses and `mock_session()` exists to replace.

### The session — `RemoteTeleop`

| | |
|---|---|
| **Lifecycle** | `start()` · `stop()` · `async with` · `wait_connected()` · `wait_ready() -> RobotInfo` |
| **State** (properties) | `status` · `info` · `telemetry` · `daemon_status` · `camera_layout` · `is_connected` |
| **Motion** | `jog(payload, duration=)` · `set_jog(payload)` · `stop_jog()` · `action(targets, wait=)` · `pose(side, position_m, orientation_xyzw=, wait=)` |
| **Safety** | `estop()` · `estop_confirmed(timeout=)` · `reset_latch()` · `reset_arm(arm)` |
| **Recording** | `record(verb, task=)` |
| **Video** | `set_video_bitrate(kbps)` · `set_video_paused(bool)` · `frames()` · `snapshot(role=)` |
| **Events** | `on(kind, cb) -> unsubscribe` · `stream(kind)` |

Three ways to jog, and the difference is **who owns the repetition** — the thing worth
getting right, because the robot stops when frames stop:

| Call | Who resends | Use for |
|---|---|---|
| `jog(payload, duration=…)` | the SDK, for a fixed time, then zeroes | scripts |
| `set_jog(payload)` | the SDK, until you clear it | interactive drivers |
| `protocol.control_jog(…)` | **you**, inside `t_warn_ms` | your own transport |

`estop()` is the one verb that **raises** on a dead control channel — in every mode, not just
strict — because an E-STOP that silently went nowhere must not read as success (every other
verb drops silently there, correctly: the watchdog makes the drop meaningless). Delivery is
still not execution, so unattended runs use `estop_confirmed()`, which awaits the robot
*reporting* the latch in telemetry and raises if it never does.

`frames()` and `snapshot()` return `Any` because their type comes from `av`, an optional
dependency — they yield `av.VideoFrame` when the `webrtc` extra is installed. Both raise a
named `TeleopError` when no video track arrives within `track_timeout`: a session is
perfectly healthy with video down, and an unattended caller needs an error, not a hang.

#### Cartesian pose targets — `pose()`

`pose(side, position_m, orientation_xyzw=None, wait=False)` commands an absolute
gripper-TCP pose and the **robot solves the IK on-board** — the wire never carries joint
solutions, so every client shares one IK implementation instead of each shipping its own
(the architecture the rpi4 era moved away from). Metres in `base_footprint` (fixed to the
robot, stable across lift travel), REP-103 axes, optional ROS-order quaternion — omit it
for "get the gripper to this point, any wrist angle" (v1 solves at the current wrist, so a
position-only failure is worth retrying with an explicit orientation).

```python
if robot.info.supports("pose_targets"):
    status = await robot.pose("right", [0.42, -0.18, 0.95], wait=True)
    print(status.state, status.reason)   # e.g. "done" "" — or "blocked" "no_ik_solution"
```

Three things distinguish it from `action()`:

- **Capability-gated.** Gate on `info.supports("pose_targets")` — a robot without it
  ignores the frame silently, so `pose()` raises on an *explicit* absence rather than
  letting a script hang to its timeout. A legacy ack (no capabilities field) passes
  through, per the probe-or-assume-legacy contract.
- **Failure is a modelled reply, not an exception.** The awaited status ends `blocked`
  with a reason that tells you what to do next: `no_ik_solution` (full pose: don't retry
  at this lift height), `ik_timeout` / `ik_no_reply` (retry), `config_jump` (waypoint the
  move), `lift_moved` (re-send to re-solve), `limit:<joint>`, `singularity`, `collision`,
  `frame:<name>`. The set is open — render unknown reasons, never fail on one.
- **Terminal states are driven by observed motion**, never by the solver returning: the
  intermediate `active` means solved-and-tracking, and a pose that stops progressing ends
  `blocked` with the live Servo status named — there is no accepted-then-nothing state.

One arm per call (arms fail independently); the gripper stays on `action()`; the lift
never moves implicitly — a pose out of reach at the current lift height is a refusal.

### Wire types — `nori_sdk.types`

`RobotInfo` · `RobotDescriptor` · `WatchdogProfile` · `Telemetry` · `CameraLayout` ·
`DaemonStatus` · `ActionStatus` · `RecordState` · `PolicyStreamStatus` · `Perception` ·
`RobotError` · `ConnectStatus`, plus `TERMINAL_ACTION_STATES` and `RECOVERY_ERROR_CODES`.

Four have sharp edges worth knowing before you use them:

- **`DaemonStatus.from_wire` and `CameraLayout.from_wire` can return `None`**, meaning *drop
  this frame and keep what you had*. Adopting the malformed frame would invent a state the
  robot never reported — a fake outage, or a blanked camera grid.
- **`ActionStatus.done` is not success.** Terminal is `done | blocked | clamped | timeout`;
  `clamped` finished somewhere other than you asked. Check `.succeeded`.
- **`RobotInfo.capabilities` is three-valued.** `None` means the robot did not say, which is
  not "supports nothing" — use `.supports(verb)`, which returns `True`/`False`/`None`.
- **`RobotInfo.model` is advisory.** Branch on `descriptor` and `capabilities` so a model this
  SDK has never heard of still works.

### Frame vocabulary — `nori_sdk.protocol`

Builders (`control_jog`, `control_action`, `control_leader`, `control_reset`, `command`,
`video_*`, `link`, `record`, `policy_stream`, `call`) plus `encode` / `decode`,
`INBOUND_KINDS` / `OUTBOUND_KINDS`, `RecordVerb` and `DESTRUCTIVE_RECORD_VERBS`.

Reach for this to drive your own transport, or to read a field this SDK does not model yet:
`decode()` always returns the untouched dict as its third element, whereas `on()` and
`stream()` hand you the parsed object.

`DESTRUCTIVE_RECORD_VERBS` deliberately omits `discard`, which **destroys data on L2 and
keeps it on A3** — no static set can classify a verb whose meaning inverts per stack.

### Motion helpers — `nori_sdk.motion`

`JogBuilder` · `joints_by_group` · `joint_group` · `joint_short` · `scale_to_range` · `clamp`.
All descriptor-driven: pass `info.descriptor` and a DOF the robot lacks raises instead of
being silently dropped robot-side.

### Mock — `nori_sdk.mock`

`mock_session()` · `MockRobot` · `LoopbackSignaling` · `loopback_pair`, plus
`WATCHDOG_PROFILES`, `JOG_SCALE` and `DEFAULT_DESCRIPTOR` for tests that need the numbers.

### Auth — `nori_sdk.auth`

`UserAuth` · `DeviceAuth` · `AuthError`. Both are token providers for `SupabaseSignaling`.

## Design decisions

**Asyncio-native.** aiortc is asyncio, so the core is too. Callbacks may be sync or async;
`session.on(kind, cb)` and `async for x in session.stream(kind)` both work.

**The operator is always the answerer.** The robot offers, we answer, and a fresh peer
connection is built per offer because the robot restarts its pipeline each session. This is
protocol, not implementation detail.

**Nothing is hard-coded per robot model.** Joints, base DOFs, lifts, cameras and ranges all
come from the `ack` descriptor. The TypeScript side learned this the hard way: its DOF
vocabulary ended up re-derived in four places, so adding the 7-DOF arm meant finding all
of them.

**Failures are named.** `ConnectStatus.failure` distinguishes `signaling_unreachable`,
`robot_absent`, `session_rejected`, `negotiation_failed` and `ice_failed`, so a script can log
"my network is broken" separately from "the robot is off" without parsing prose.

**Two divergences from the browser SDK**, both forced by aiortc:

1. **Outbound ICE is not trickled.** aiortc completes gathering inside
   `setLocalDescription`, so our candidates ride in the answer SDP and `send_ice()` is never
   called outbound. Inbound trickled candidates from the robot are still accepted. Setup is
   slightly slower; the result is identical.
2. **No adaptive bitrate loop.** The browser adapts the robot's encoder from live `getStats`.
   A script usually wants a fixed quality, so `set_video_bitrate()` is manual.

## Staying in sync with the TypeScript SDK

There are now **three** implementations of one protocol: `@nori/sdk` (TS), this package, and
the robot's `nori_gateway/protocol.py`. They are hand-written and will stay hand-written — the
interesting parts (watchdog handling, descriptor-driven keymaps, ABR) are behavior, not types,
and codegen wouldn't produce them.

What must *not* stay hand-verified is the wire contract, and it no longer is. The spec lives
in its own repo — [`Nori-Robotics/Nori-Protocol`](https://github.com/Nori-Robotics/Nori-Protocol) —
as JSON Schema plus golden fixtures, in two layers: `daemon/` (what a motion daemon speaks over
its control port) and `session/` (what a client speaks over the data channel — this SDK's layer).

`tests/test_conformance.py` runs this SDK against those files in both directions: every golden
frame a robot can send must decode, and **every frame this SDK builds must validate against the
schema**. The second direction is the one that earns its keep — it is what catches "we invented
a field name", which is exactly how this SDK's base jog ended up addressing DOFs no robot reads.

### Mounting the spec

```bash
git submodule add git@github.com:Nori-Robotics/Nori-Protocol.git spec/nori-protocol
```

Until that exists, point the suite at a local checkout — also how you test a spec change
before pushing it:

```bash
NORI_PROTOCOL_DIR=/path/to/nori-protocol pytest
```

With neither, the conformance tests **skip** rather than fail, so a contributor without the
submodule initialised can still run everything else.

### The xfail policy

Known divergences are marked `xfail(strict=True)` with a reason naming the consequence — not
deleted, and not left red. The suite stays green, each gap is documented where it will be
found, and the moment somebody fixes one the test XPASSes and *fails the build*, forcing the
marker off. A self-retiring TODO list — and it is currently **empty**. All eight divergences
have been fixed; each former `xfail` is now an ordinary test guarding the fix, kept rather
than deleted because the consequence each one documents is the part worth preserving.

They shared a shape worth naming, because it is the one this policy exists to catch: each
made the SDK **report success while doing something else** — a base jog that was silently a
full stop, a completed action that had not happened, a malformed policy reply that read as a
running stream, a healthy robot reported offline, a fatal error arriving as an untyped dict,
one bad repeat blanking a good camera layout. None would have surfaced as an error anywhere.
A fix here is not done until a mutation reverting it fails exactly one named test;
`tools/mutate.py` runs all 19.

Still to do, and unchanged: **move policy into data** (the robot-ops manifest already lives in
`robot-tools.json`; DOF tables and ABR tuning should join it), and codegen only if the protocol
outgrows what fixtures cover.

## Status

The whole suite runs with no hardware, no network and no WebRTC stack — `pytest` is the
authority on the count. **Zero `xfail`s remain** — all eight known divergences from the spec
are fixed, each pinned by a mutation that fails exactly one named test. See
[the xfail policy](#the-xfail-policy) for why that number is worth quoting.

Verified against the spec:

- **Every frame-building function in `protocol.py` is exercised by a conformance test** and
  validates against `nori-protocol`, the base jog and the all-stop frame included. That
  coverage is measured, not assumed: an earlier version of this line claimed full validation
  while two builders had no test at all, and one of them was producing an invalid frame.
- Every golden fixture from both spec layers decodes, including the legacy no-descriptor ack
  and the L2-shaped frames (bridge-injected telemetry fields, `<session>/episode-NNNN`)
- `pose()` / `control_pose()` validate against the spec's `control.pose` fixtures; the
  robot side (A3 gateway) is bench-verified through the full lifecycle including the
  observed-motion failure guards — but this SDK's pose path has NOT yet run end-to-end
  over a live WebRTC session (see `nori_ws` docs/runbooks/pose-targets-sdk-hardware-test.md)
- Descriptor-driven motion helpers
- `MockRobot` — pinned against the real gateway's frame order and record lifecycle
- `LoopbackSignaling` — in-process transport pair for handshake tests
- `UserAuth` / `DeviceAuth` — Supabase token providers with refresh, skew clamping and backoff

Hardware-verified — live bench A3 sessions (NORI-A3-0000, 2026-08-21/22) drove the robot
through this SDK end-to-end:

- `RemoteTeleop`'s WebRTC path: offer/answer against GStreamer's webrtcbin (the RSA-cipher,
  H.264-fmtp and ICE-trickle interop fixes in `webrtc_compat` are each hardware-confirmed),
  the control channel, and live jog/action driving. The watchdog keep-alives inside
  `action(wait=True)` and `pose(wait=True)` exist *because* those sessions found their
  absence — both were hardware-found bugs, now fixed and pinned.
- `SupabaseSignaling` against the live Realtime service, in the same sessions.

Not yet verified against hardware — expect to check these first:

- The **LAN verdict of link-mode detection**: the handshake delivery itself is
  bench-verified (2026-08-26 — the robot received and applied the mode), but the "lan"
  classification was rewritten the same day (aiortc implements no candidate-pair stats,
  so the old getStats loop answered "wan" unconditionally; detection now reads aioice's
  nominated pairs and refuses to call a VPN/tunnel path "lan"). Unit-tested against fakes;
  a live LAN session has not yet confirmed the robot adopts the tighter watchdog profile.
- `estop_confirmed()`, `frames(track_timeout=)` and the stream shutdown wake-up (all
  2026-08-25) are unit-tested against the mock, not yet exercised on hardware.

Not built yet:

- Adaptive bitrate; per-camera decode helpers beyond `snapshot(role=...)`
- Two-way audio (`call` verbs are in the vocabulary, not in the session)
- A synchronous facade for scripts that don't want an event loop
- VR mapping, the robot-ops/agent-tool manifest, the 3D model helpers — TS-only for now
- Nickname / robot-list / REST: deliberately absent, exactly as in the TS SDK. Those live in
  the app and backend, and the robot's gateway owns its own nickname courier.

## Development

```bash
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q
```

Every test runs without a robot, a network or a WebRTC stack. That is a deliberate property —
keep it.
