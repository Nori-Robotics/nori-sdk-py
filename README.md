# nori-sdk (Python)

Python operator client for Nori robots. Speaks **nori-protocol** over a WebRTC data channel —
the same wire dialect as the TypeScript `@nori/sdk` the web app uses, and the same one the
robot's ROS 2 gateway implements.

It exists for the clients a browser SDK can't serve: headless scripts, policy and agent
drivers, dataset tooling, CI that drives a robot (or a mock) without a browser.

> **Status: sketch.** The pure layers (protocol, types, motion helpers, mock) are complete and
> tested. `RemoteTeleop` is written but has **not yet been run against a real robot** — see
> [Status](#status) for exactly what is and isn't verified.

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
means your logic is right, not that your network is. Also unmodelled: `perception` and
`error` frames, motor faults, thermals, and a daemon that goes offline mid-session.

## Quickstart (against a robot)

```python
import asyncio
from nori_sdk import RemoteTeleop, SupabaseSignaling, UserAuth
from nori_sdk.motion import JogBuilder

async def main():
    auth = UserAuth(SUPABASE_URL, ANON_KEY, "me@example.com", "password")
    signaling = SupabaseSignaling(
        SUPABASE_URL, ANON_KEY, room="NORI-L3-0001", token_provider=auth.token
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

## Design decisions

**Asyncio-native.** aiortc is asyncio, so the core is too. Callbacks may be sync or async;
`session.on(kind, cb)` and `async for x in session.stream(kind)` both work.

**The operator is always the answerer.** The robot offers, we answer, and a fresh peer
connection is built per offer because the robot restarts its pipeline each session. This is
protocol, not implementation detail.

**Nothing is hard-coded per robot model.** Joints, base DOFs, lifts, cameras and ranges all
come from the `ack` descriptor. The TypeScript side learned this the hard way: its DOF
vocabulary ended up re-derived in four places, so adding the L3 arm meant finding all of them.

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
marker off. A self-retiring TODO list. There are currently **4**; each is a real bug in this
SDK, not a spec disagreement: the missing legacy `record` verb aliases, unmodelled `error`
frames, a tile-less `camera_layout` being accepted, and `model`/`capabilities` absent from
`RobotInfo`.

Four more have been retired. They shared a shape worth naming, because it is the one this
policy exists to catch: each made the SDK **report success while doing something else** — a
base jog that was silently a full stop, a completed action that had not happened, a malformed
policy reply that read as a running stream, and a healthy robot reported offline. None would
have surfaced as an error anywhere. A fix here is not considered done until a mutation
reverting it fails exactly one named test.

Still to do, and unchanged: **move policy into data** (the robot-ops manifest already lives in
`robot-tools.json`; DOF tables and ABR tuning should join it), and codegen only if the protocol
outgrows what fixtures cover.

## Status

The whole suite runs with no hardware, no network and no WebRTC stack — `pytest` is the
authority on the count. Everything passes except **4 `xfail`s, each marking a real bug in this
SDK** rather than a spec disagreement; see [the xfail policy](#the-xfail-policy). That number
is worth quoting because it only moves deliberately: an `xfail` that starts passing fails the
build. It was 8; the four that made the SDK actively lie to a caller are fixed, and each fix
is pinned by a mutation that fails exactly one test.

Verified against the spec:

- **Every frame-building function in `protocol.py` is exercised by a conformance test** and
  validates against `nori-protocol`, the base jog and the all-stop frame included. That
  coverage is measured, not assumed: an earlier version of this line claimed full validation
  while two builders had no test at all, and one of them was producing an invalid frame.
- Every golden fixture from both spec layers decodes, including the legacy no-descriptor ack
  and the L2-shaped frames (bridge-injected telemetry fields, `<session>/episode-NNNN`)
- Descriptor-driven motion helpers
- `MockRobot` — pinned against the real gateway's frame order and record lifecycle
- `LoopbackSignaling` — in-process transport pair for handshake tests
- `UserAuth` / `DeviceAuth` — Supabase token providers with refresh, skew clamping and backoff

Written but **not yet verified against hardware** — expect to fix things here first:

- `RemoteTeleop`'s WebRTC path: offer/answer, ICE, control channel, video track
- `SupabaseSignaling` against the live Realtime service (the threading and reconnect logic is
  a port of the gateway's proven implementation, but the operator role is new)
- Link-mode detection from `getStats` (aiortc's stats shape differs from the browser's)

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
