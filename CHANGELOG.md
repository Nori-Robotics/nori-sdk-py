# Changelog

Newest first. This package targets **nori-protocol v1** — see the Status section of
`README.md` for what is and isn't hardware-verified.

## 1.1.0 — unreleased

Additive: thirteen new methods and nine new exports, nothing removed or changed. Targets
**nori-protocol v1** still — the navigation and sensor frames are additive to the spec and
carry no version bump.

### Named navigation

`remember_waypoint()`, `list_waypoints()`, `delete_waypoint()`, `navigate_to_waypoint()`,
`cancel_navigation()`, `get_navigation_status()`, `await_navigation()` and the
`navigation_status` property, gated on the robot advertising `named_navigation`.

The API takes names, never coordinates: the robot owns localization, active-map matching,
motion safety and the single-goal rule. A refusal (`ok=False` — "waypoint not found",
"navigation is active", "software E-stop is active") is RETURNED for the caller to inspect,
matching `policy_stream()` rather than `record()`.

### `RobotUnreachable` — a lost reply is not a lost command

When the robot does not answer a correlated request, this SDK **raises** rather than
returning a status. Returning one would mean inventing `state` and `active`, and a caller
reading `active=False` off an invented status would read a transport failure as a halted
robot — for the one verb in this SDK that makes the robot drive itself. The exception
carries the robot's last real snapshot on `.last_known`, which is stale by definition and
never evidence of the present. It subclasses `TeleopError`, so existing handlers still work.

Unanswered requests are retried under the SAME `request_id`, which the gateway treats as
idempotent — that is what lets a one-shot command survive a dropped frame without ever
starting a second goal.

### An unknown lifecycle state is never treated as finished

A `state` this build has not heard of is kept verbatim rather than coerced onto a known one,
because coercing it would make it terminal and `await_navigation()` would then report a
finished goal while the robot drove on. Terminality is membership in
`TERMINAL_NAVIGATION_STATES`, so an unrecognised state is simply not terminal. Same rule as
`SafetyState`. (The TypeScript SDK carried the same bug and was fixed in lockstep.)

### Opt-in LiDAR and IMU streams

`configure_sensor_streams()` and `get_sensor_stream_status()`, plus the `lidar_scan`,
`imu_sample` and `sensor_stream_status` properties, gated on `sensor_streams`. Both feeds are off until asked for; a rate
of 0 stops one. Bounds (LiDAR 0–10 Hz / 16–1440 points, IMU 0–50 Hz) are the spec's, pinned
to the schema by a conformance test, and revalidated on the robot. A non-finite ROS reading
parses to `None` — a measurement gap, never a distance.

### `MockRobot` grew the navigation and sensor vocabulary

Waypoint storage, the goal lifecycle, `navigation_outcome` for rehearsing an abort, and
`emit_sensor_samples()`. It now mirrors the gateway's request-id idempotency window — 256
replies, evicted oldest-written-first — so a retry landing past the window re-runs here
exactly as it would on a robot. Non-UUID request ids are dropped, as the gateway drops them.
`mock_session()` can now deliver unsolicited robot frames, not only replies.

## 1.0.1 — 2026-08-26

### `JogBuilder.arm()` accepts task-space verbs in strict mode

Robots that advertise `descriptor.jog_scale.task` take task-space verbs ("x", "y", "z",
"pitch", "yaw") through the same arm() call, and strict mode now knows that instead of
rejecting them as unknown DOFs. "yaw" is the canonical angular-z verb; "shoulder_pan" is
its deprecated alias, and each is accepted whenever the other is advertised. Robots with
no `jog_scale.task` still reject task verbs, as before.

### The mock accepts the discard record verbs

`session_discard`, `discard` and `discard_last` are in the SDK's own `RecordVerb` type and
the gateway accepts all three (they close the session; episode-as-unit means nothing is
deleted on this stack) — but the mock refused them as unknown, so correct code raised
against the sandbox. It now answers them gateway-verbatim.

## 1.0.0 — 2026-08-25

First public release. Everything below lands together; the headline items:

### Removed: the inert `RemoteTeleop(protocol_version=...)` kwarg

It was stored and used only to format the mismatch log line — it never changed what the SDK
emits or how mismatch is detected (`RobotInfo.from_wire` compares the robot's ack against
this build's `NORI_PROTOCOL_VERSION`, which is what an SDK build actually speaks). The
TypeScript SDK never had the option, for the same reason. Removed *before* the 1.0 API
freeze; if a real override need appears, a working kwarg can return compatibly in a minor.

### `estop()` now raises on a dead channel — a deliberate contract change

Previously `estop()` returned `None` whether or not the frame flew, like every other verb.
That is right for ordinary verbs (the watchdog makes a dropped frame meaningless) and wrong
for an E-stop — a caller should never mistake a silently dropped stop for a delivered one.
`estop()` now raises `TeleopError` in every mode — not just strict — when the channel is not open,
so the caller knows to reach for the physical button. **Migration:** a bare
`finally: robot.estop()` cleanup should become `try: robot.estop() except TeleopError: ...`
(or log-and-continue), or the raise will mask the original exception.

### `estop_confirmed()` — delivery is not execution

New awaitable: sends the estop, then awaits the robot *reporting* the latch in telemetry.
Only a report observed after the send counts — the cached merged frame is deliberately not
consulted, since the safety block is carried forward and a stale "latched" would confirm an
estop that went nowhere. Raises when no latch is seen; the only safe reading is "not stopped".

### Base sign convention pinned against the spec fixture

The SDK has always emitted raw REP-103 (+angular = LEFT, no client negation). The gateway
formerly un-negated angular to compensate for a TypeScript-era L2 quirk, inverting this SDK's
turns on A3/L3; that gateway change ships coordinated with this release, and a conformance
test now pins this SDK's emission byte-for-byte against `control_jog_base.json` so the
convention can never drift silently again.

### Link-mode LAN detection actually works now (2026-08-26 bench findings)

aiortc implements no candidate-pair stats, so the getStats-based LAN detection matched
nothing and reported "wan" unconditionally — found while bench-testing this release. Detection now reads
aioice's nominated pairs directly (private-attribute walk that degrades to "wan" if an
aiortc upgrade changes it), and deliberately refuses to call a VPN/overlay path "lan"
even when its candidates are ICE-type host: a tunnel's 1280-byte MTU with the tight
watchdog profile is the worst pairing (the same bench found Tailscale-carried sessions
silently dropping every fragmented frame — if you run a VPN on the operator machine,
prefer disabling it while driving). `estop_confirmed()`'s default timeout also widened
2 s → 5 s: the latch report crosses gateway → safety node → telemetry, and 2 s proved
tight on a busy stack.

### Session robustness for unattended runs

- A mid-session `robot_here` (the gateway rebroadcasts it on every signaling rejoin) no
  longer marks a healthy session disconnected forever; `ready` is still sent, which is what
  a genuinely restarted gateway needs to re-offer.
- The link-mode handshake actually fires: the robot-opened channel arrives already open, so
  the open logic now runs immediately instead of waiting for an `open` event that fired
  before we could subscribe. LAN sessions get LAN watchdog windows.
- `frames()` / `snapshot()` / `snapshot_png()` raise a named error (`track_timeout`,
  default `ROBOT_WAIT_S`) instead of polling forever when no video track arrives; `stream()`
  consumers are woken when the session stops instead of parking on an idle queue.

### The mock now refuses what the gateway refuses

Unknown action keys answer `blocked/"unknown_joint:<keys>"` (gateway-verbatim, incl.
`empty_action`); unknown jog vocabulary is dropped in the same silence; a latched robot
refuses a pose with `blocked/"estop_latched"` instead of silence, and action refusals say
`"estop_latched"` (previously `"latched"`, a string no gateway emits); a pose for an arm the
robot lacks refuses `blocked/"empty_pose"`; default capabilities now match a healthy A3
gateway (`task_jog`, `pose_targets`, `record`), so `pose()` works against a plain
`mock_session()` exactly as it does on hardware.

### Protocol: `control.pose` and `action_id` are finalized

Both graduated from PROPOSED in nori-protocol v1 alongside this release (older changelog
entries below describe them as proposed — that was true at their date).

### The doc examples now work

The module headline example commanded `{"base": {"x": ...}}` — the telemetry-namespace
spelling a robot reads as an explicit stop — and the `JogBuilder` docstring showed
`.base(x=1.0)`, which raises. Both now use `linear`, as do the tests that pinned the old
spelling.

## Pre-release — 2026-08-23

### `pose(wait=True)` no longer starves the watchdog; `goto_pose` is now an alias

A pose frame latches a target the arm takes SECONDS to reach, and `pose(wait=True)` sent that
one frame and then awaited in silence — control silence the gateway dead-man reads as "the
operator is gone" (warn scales motion to ZERO at 300 ms WAN, stop drops the latched target at
1 s). Every awaited move slower than t_stop died mid-flight as a phantom timeout — the same
hardware-found failure `action(wait=True)` had (2026-08-22). `pose(wait=True)` now streams the
empty-jog keep-alive (commands nothing, cancels nothing) until the terminal status, and gains
the same strict-mode liveness guard as every other motion verb.

`goto_pose` — the parallel-built name for the same verb — is now a thin alias with its
awaited-move defaults (`wait=True`, 15 s patience): one implementation, one feeder, one
capability gate, no drift. New code calls `pose()`.

### Cartesian pose targets — `RemoteTeleop.pose()` (spec: `control.pose`, PROPOSED)

`pose(side, position_m, orientation_xyzw=None, wait=False)` commands an absolute gripper-TCP
pose in `base_footprint` (metres, REP-103; optional ROS-order quaternion — omit it for "any
wrist angle"). The robot solves IK on-board and tracks through the same latch `action` uses,
answering on the shared `action_status` lifecycle — including the intermediate `active`
(solved, tracking) and a modelled failure vocabulary (`no_ik_solution`, `ik_timeout`,
`limit:<joint>`, `singularity`, `collision`, `lift_moved`, `frame:<name>`).

Gated on the `pose_targets` capability: explicitly-unsupported robots raise `TeleopError`
instead of a silent 10 s no-op (the payload would be ignored on the wire); a legacy ack with
no capabilities field is allowed through, per the probe-or-assume-legacy contract. New
builder `protocol.control_pose()` + `protocol.POSE_FRAME`; additive, no version bump.

## Pre-release — 2026-08-23

### The unwired verbs are wired

Four frame builders passed conformance for weeks with **zero session call sites**, and two
decoded frame types were parsed and then discarded. A builder with no call site is not a
feature — it is a promise the API does not keep, and conformance cannot tell the difference.

- **`policy_stream(action, **extra)`** — the headline Python use case could not start a
  stream. Returns the status rather than raising on `ok: false`: unlike `record()`, a refusal
  here is ordinary state (a stopped stream answers `ok:false` to `"status"` routinely), so
  raising would make normal polling throw.
- **`policy_stream_status`** — the cached last reply, with the liveness rule documented: there
  is NO unsolicited death notification, so a stream that dies mid-run is visible only by
  polling. `MockRobot.die_mid_stream()` rehearses exactly that.
- **`perceive()` + `perception_age`** — perception decoded to a dataclass nothing surfaced.
  Age is measured on OUR monotonic clock, not the frame's `ts_ns`, which is the robot's clock
  and would fold in skew.
- **`action_status(id)` / `next_action_id()`** — the fire-and-forget-then-poll shape. The
  verdict map is a bounded LRU (`ACTION_HISTORY = 256`): a policy issuing thousands of actions
  must not grow it forever, and only the latest verdict per id is useful.
- **`record_state`**, **`set_leader_action()`**, **`set_video_quality()`**, **`call()`**.
- **A design error caught by its own test.** I first gated `policy_stream` on
  `_require_live`, which checks `daemon_status.online` — i.e. MOTION health. The streamer is
  served by the bridge in FRONT of the motion daemon and runs fine on a robot whose arms are
  disabled, so strict mode would have refused a valid operation. Split out
  `_require_connected` for the bridge-side verbs; motion verbs still get the full gate.
- The mock now answers `policy_stream` and can emit `perception`. A verb the double cannot
  answer is a verb nobody can develop against.

### Real angles from a normalized wire — `descriptor.ranges_si`

`ranges` is in `norm_mode` units, and the normalized-to-physical mapping is the robot's own
**per-unit calibration** — not a nominal figure from the URDF. So a client wanting real angles
(to pose a URDF, run FK, feed a simulator) had to substitute the URDF's nominal joint limits,
wrong by that unit's calibration offset and wrong **silently**.

- **Protocol**: `descriptor.ranges_si`, optional and additive — radians for revolute, metres
  for prismatic. Two fixtures (a full A3 with per-side calibration skew, and an inverted span).
  50 fixtures / 18 schemas / 0 failures.
- **No gripper special case.** `ranges` already encodes the convention difference (body joint
  `[-100,100]`, gripper `[0,100]`), so one linear map between the two entries covers both. A
  hand-written gripper branch is the thing that rots when a robot changes convention.
- **Only normalized keys appear.** The A-series lift is deliberately absent: `ranges["lift.pos"]`
  is already millimetres, so an SI entry would mean converting twice.
- **Inverted bounds are honoured, not sorted.** A calibration can reverse an axis and the
  ORDER carries that; sorting it ascending would flip the joint.
- **SDK**: `motion.to_si()`, `from_si()`, `state_to_si()`. All return `None`/omit rather than
  guessing — the frozen L-series never publishes this, so absence is the common case.
  `state_to_si()` OMITS what it cannot convert rather than passing it through: a dict silently
  mixing radians and normalized units is worse than a smaller one, because nothing downstream
  can tell which key is in which unit.

## Pre-release — 2026-08-20

### Calibrated jog rates — `descriptor.jog_scale`

A jog is normalized `[-1,1]` and the robot owns what full deflection means, which is what
keeps one client working across models. The cost was that a script could not ask for a
REPEATABLE speed or discover what it just asked for. This closes that without adding a
velocity command — the robot still owns the envelope.

- **Protocol** (`Nori-Protocol`): `descriptor.jog_scale`, optional and additive, so no version
  bump and no client breaks. Namespaced `joints` / `task` / `base` / `lift`, because a flat map
  cannot express that `x` and `pitch` are task-space VERBS rather than joints — `shoulder_pan`
  is not even a joint name on every model. Two fixtures (full A3, arms-only partial); 44
  fixtures / 17 schemas / 0 failures.
- **Units are per namespace, each matching the thing it addresses**, so a client can verify
  what it got: joints in norm_mode units/s (matching `telemetry.state` and `ranges`), lift in
  mm/s (matching `<side>_lift.pos`), task and base in SI. **Joints are deliberately NOT
  rad/s** — telemetry reports normalized positions, so a rad/s figure could not be checked
  against anything a client can see, and verifiability was the entire point.
- **It promises the NOMINAL COMMANDED scale, not achieved velocity**, and says so in the
  schema. Three things routinely make the real rate lower: the watchdog's `warn` state scales
  all motion to ZERO, an acceleration limit means short jogs never reach the rate, and MoveIt
  Servo scales near singularities.
- **Omission means UNKNOWN at every level** — missing block, missing namespace, missing key. A
  rate of `0` is schema-invalid rather than meaning "cannot move"; that is expressed by leaving
  the key out. The parser drops a non-positive rate rather than believing it, since a zero that
  survived would silently scale every command to nothing.
- **SDK**: `JogScale` on `RobotDescriptor`, plus `motion.jog_rate()` and
  `motion.normalized_for()`. Both return `None` rather than guessing — the L2 fleet is frozen
  and will never publish this, so `None` is the common answer and callers must handle it.
- **`tools/measure_jog_scale.py`** — the part that makes the numbers true. Publishing the
  gateway's constants would be moving a number onto the wire and calling it verified; if the
  accel limit, the target leash and Servo mean steady-state is 0.72 where the constant says
  0.8, then 0.8 is a lie with a decimal point on it. The tool fits steady-state rate per joint
  per direction, discards the acceleration ramp, refuses runs that leave the middle of the
  advertised range or that arrive while the watchdog is degraded, rejects non-linear runs by
  R², takes a median across runs and **refuses to publish a joint whose spread exceeds 20%**.
  Rehearsed against the mock, where it recovers the known `JOG_SCALE = 40.0` as 39.33.

### Public API surface audited and pinned

The surface an SDK promises drifts by accident — a helper loses its underscore, a name is
dropped from `__all__` while callers still import it, a new method ships undocumented. None of
that fails a normal suite, and all of it reaches users. It is now data, in
`tests/test_public_api.py`, with a snapshot of the top-level surface that has to be edited
deliberately.

- **`dir(nori_sdk)` omitted `RemoteTeleop`.** The lazy-import `__getattr__` resolved the three
  optional-extra names on access but never listed them, so the class this package exists to
  provide was missing from tab-completion, `help()` and IDE introspection until something
  touched it first. Added `__dir__`.
- **Six public members of `RemoteTeleop` had no docstring**: `status`, `camera_layout`,
  `is_connected`, `reset_latch`, `reset_arm`, `set_video_paused`. These are exactly the ones
  carrying behaviour a signature cannot convey — that `is_connected` does not mean the robot
  will move, that `reset_latch` is for a latch and not for `safe_hold`, that `camera_layout`
  being `None` has two different meanings.
- **`set_jog()` was documented misleadingly**, reported from the first external review. The
  README's "resend inside `t_warn_ms`" rule describes the WIRE, and `set_jog` does the
  resending for you; read together they implied a caller should run its own timer, which would
  race the SDK's. All three jog entry points now state who owns the repetition, in a table.
- **Five constants were reachable but undeclared** (`RETRY_S`, `ROBOT_WAIT_S`, `JOG_SCALE`,
  `WATCHDOG_PROFILES`, `DEFAULT_LINK_MODE`). Reachable-but-undeclared is the worst of both:
  people depend on it anyway and nothing stops it changing. Now in `__all__`.
- **`nori_sdk.types` is exported** alongside `motion` and `protocol`, which it should have
  been all along.
- **Documented the forward-compatibility rule.** Three classes carry `raw` and the rest do
  not, which is a rule rather than an oversight — and the gap it leaves is now stated: `on()`
  and `stream()` hand you the PARSED object, so a field this SDK does not model is reachable
  only via `protocol.decode()`.
- **`README.md` gained an API reference** — the actual question a new developer has after the
  quickstart, which the layering table did not answer.

### The xfail list is now empty

All four remaining divergences fixed. Each is mutation-pinned; `tools/mutate.py` now runs 19.

- **`error` frames are modelled** (`RobotError`, and `"error"` added to `INBOUND_KINDS`). A
  fatal robot error previously reached the caller only as an untyped dict. `fatal` defaults
  FALSE per the schema — defaulting it true would tear down a live session over a soft stall.
  Added `RECOVERY_ERROR_CODES` and `.recovered`, because three codes
  (`obstruction_cleared`, `arm_recovered`, `motor_recovered`) report a fault *clearing*: a
  client rendering every `error` as a fault shows a red banner for the good news.
- **A tile-less `camera_layout` is rejected.** It was accepted, and adopting one blanks the
  grid for the rest of the session — the robot repeats the layout on open, so a single
  malformed repeat could poison a good one. Note this is the opposite of an ABSENT layout,
  which is how a single-camera robot says "the whole frame is the one camera".
- **`RecordVerb` carries all eleven spec verbs**, including the legacy aliases deployed
  clients still send. Grouped in the source by what they do to DATA, since the names do not
  signal it: `session_discard` is canonical and destructive (not a synonym for `session_end`
  — they are opposites), `stop` also ends the session on L2, and `discard` **destroys on L2
  but keeps on A3**. Added `DESTRUCTIVE_RECORD_VERBS`, from which `discard` is deliberately
  absent: no static set can classify a verb whose meaning inverts per stack.
- **`RobotInfo` exposes `model` and `capabilities`**, plus `supports()`. `capabilities` is
  three-valued — `None` means the robot did not say, which is NOT "supports nothing".
  Collapsing absent into False would silently disable working features on every robot
  predating the field. `model` is advisory only; branch on `descriptor`/`capabilities`.

### Mock and docs

- **The mock no longer derives its camera layout from `descriptor.cameras`.** The schema names
  this as an antipattern in as many words: the layout frame is the only authoritative
  description of the tiling, the descriptor is diagnostic metadata, and a mock that generates
  one from the other can never reproduce a disagreement — hiding exactly the layout bugs it
  exists to catch. `MockRobot(tiles=[...])` now rehearses that case.
- `DEFAULT_DESCRIPTOR` was commented "shaped like an L3". It is 5 DOF per arm, which is the
  **L2** shape; A3 arms are 7 DOF. Corrected, with a note on why the smaller descriptor is
  the right default.
- **L3 → A3 throughout.** L3 is retired; room names, the leader-arm note and the mock comment
  referenced it. Historical statements about how the TS SDK's DOF vocabulary drifted now say
  "the 7-DOF arm" rather than naming a dead model.

### The mock became usable for development, not just for tests

Driving `MockRobot` previously required private API (`teleop._control`, `teleop._handle_frame`).
That is fine inside this package and wrong to hand anybody else, since those names carry no
compatibility promise.

- **`nori_sdk.mock.mock_session()`** — an async context manager yielding a connected
  `RemoteTeleop` backed by a `MockRobot`. No WebRTC, no network, no credentials; needs no
  extras. A script written against it runs unchanged on hardware, with one line different.
  Replies are delivered via `call_soon` rather than inline, so a client cannot accidentally
  depend on synchronous reentrancy a real data channel would never give.
- **`MockRobot` enforces the watchdog.** Silence past `t_stop_ms` stops the motion and reports
  `safe_hold`; `link("lan"|"wan")` selects the profile (150/500 vs 300/1000 ms), and the `ack`
  advertises the profile it will actually enforce. This was the largest gap: the README calls
  the watchdog "the one thing to internalize", the TypeScript mock has emulated it since
  `sim.ts`, and this one hard-coded `"watchdog": "ok"`. A script that held a jog by sending one
  frame and sleeping therefore passed locally and would have stopped dead on a robot.
  `safe_hold` self-clears on the next control frame — only an E-STOP needs `reset_latch()`.
- **`MockRobot.step(dt)` integrates a pose**, clamped to `descriptor.ranges`, so telemetry
  responds to commands. The clock is accumulated `dt`, not wall time, so a test can advance
  two seconds instantly. Absolute `action` targets land in the pose too. Not a simulator: no
  dynamics, no collision, no IK, and task-space arm keys are not resolved into joints.
  - Caught while writing it: the first draft **held the last base velocity** when a later jog
    omitted `base`. `control.json` says an absent `base` means STOP, so the mock would have
    taught a script the opposite of what the robot does. Pinned by a test.
- **`examples/mock_pick_place.py`** — a runnable first task (discover, check motion health,
  jog, absolute move, record an episode, stream telemetry, E-STOP), executed by the test suite
  as a subprocess so it cannot rot.


### Four xfails retired — every one of them a silent lie to the caller

These shared a shape, which is why they were taken as a batch: each made the SDK **report
success while doing something else**, with no error raised anywhere. Each fix is pinned by a
mutation that reverts it and fails exactly one named test; nine mutations were run in total.

- **`JogBuilder.base()` emitted the telemetry namespace.** It built `{"base": {"x": ...,
  "theta": ...}}` from `descriptor.base`'s `x.vel`/`theta.vel`. The jog namespace is
  `linear`/`angular`, and a robot reads `x`/`theta` there as `linear=0, angular=0` — **an
  explicit stop**. Any script driving the base did nothing and reported nothing. The signature
  is now `base(linear=..., angular=...)`, and the old spelling **raises** rather than being
  aliased: a quiet translation would leave every caller believing the two namespaces are
  interchangeable, and the next DOF added under one name only would fail the same way again.
  `JogBuilder.stop()` now writes the base zeros explicitly. `base()` also validates against
  `descriptor.base`, as `arm()` and `lift()` already did.
- **`action(wait=True)` returned before the move happened.** `ActionStatus.done` counted
  `accepted` as terminal (and `failed`, which is not in the spec's enum at all, while missing
  `clamped` and `timeout`). Worse, `_handle_frame` resolved the future on the *first*
  `action_status` regardless — so a caller was told the action was complete while the watchdog
  was still free to abort it. Terminal is now exactly `done | blocked | clamped | timeout`,
  pinned against the schema's enum rather than a hand-typed list, and the future waits for
  one. Added `.succeeded`, because `clamped` is a finished move to a *different* pose than
  requested. An unknown state counts as non-terminal, so a newer robot's vocabulary falls
  through to the caller's timeout instead of being reported as a completed move.
  **`MockRobot` was reproducing this bug** — it emitted only `accepted` — so the double and
  the client agreed and the suite stayed green. It now emits the full `accepted -> active ->
  done` lifecycle, with `action_outcome` to rehearse `clamped`/`timeout`.
- **`PolicyStreamStatus` modelled invented fields.** `state`/`detail` do not exist on the
  wire; the real fields are `streaming`/`dest`/`fps_out`/`frames_sent`/`dropped`/`error`. And
  `ok` defaulted **true**, so a truncated or malformed reply read as a running stream. Both
  `ok` and `streaming` now default false — deliberately inverted from this SDK's usual
  tolerance. A test asserts every field in the schema is modelled, so one added later cannot
  quietly live only in `raw`. The docstring now records that there is **no unsolicited death
  notification**: a stream that dies mid-run is observable only by polling `status`.
- **A stateless `daemon_status` invented an outage.** A missing `state` was coerced to
  "offline". The bridge rebroadcasts every few seconds while offline, so one malformed repeat
  would flip a healthy robot to offline in every watching UI and log. `from_wire` now returns
  `None` — **a signature change: callers must handle it** — and `RemoteTeleop` drops the frame
  without emitting, rather than handing subscribers a raw dict where every other
  `daemon_status` gives them a `DaemonStatus`. Also picked up `robot_local_mic_muted`, which
  rides here rather than on telemetry precisely because telemetry stops when the daemon does.

### Packaging and CI (new)

- **`py.typed` added — and `mypy` was inert without it.** The `strict = true` config has been
  in `pyproject.toml` since the first commit; mypy refuses to check an installed package that
  does not advertise type information, so it exited 0 having examined nothing. With the marker
  in place it found 15 errors, one of them real (a `Future[ActionStatus]` variable reused to
  hold a `Future[RecordState]`). Third-party stub noise from aiortc/websocket-client is
  suppressed by module override, not by relaxing strictness. A CI step asserts `py.typed` is
  actually in the built wheel — a package that ships without it silently untypes every
  consumer.
- **`.github/workflows/ci.yml`**: ruff, mypy and pytest on 3.11/3.12/3.13, plus a build job
  running `twine check`. `NORI_PROTOCOL_DIR` is set explicitly so a broken spec checkout is a
  hard failure (`tests/_spec.py` raises on a set-but-invalid path) rather than a green run
  that skipped all of conformance, and a separate step asserts the conformance suite collected
  something. A nightly cron catches spec changes landing in Nori-Protocol without a commit
  here. **Requires a `NORI_PROTOCOL_TOKEN` secret** with read access to the private spec repo.

## Pre-release — 2026-08-13

### Conformance against the spec (new)

`tests/test_conformance.py` runs this SDK against the real `nori-protocol` schemas and golden
fixtures, in both directions: every frame a robot can send must decode, and **every frame this
SDK builds must validate against the schema**. The second direction is the one that earns its
keep — it is what catches "we invented a field name".

- The spec is resolved from `NORI_PROTOCOL_DIR`, then `spec/nori-protocol` (the intended
  submodule path), then a sibling checkout. **An explicitly-set-but-invalid `NORI_PROTOCOL_DIR`
  is now a hard error, not a skip.** It previously returned `None`, which skipped the entire
  conformance suite and exited **0** — a CI job with a typo would have been green while
  testing nothing, which is precisely the failure this spec exists to prevent.
- Known divergences are `xfail(strict=True)` with a reason naming the consequence, rather than
  deleted or left red: the suite stays green, each gap is documented where it will be found,
  and fixing one makes the test XPASS and *fail the build*, forcing the marker off. There are
  8, each a real bug in this SDK.
- Added coverage for `control_reset` and `control_leader`, which had none. That gap was hiding
  a real defect: `control_reset()` built a frame with no `seq`, which the schema then required.
  (The schema has since been relaxed — the L2 daemon defaults `seq` to `-1` and both clients
  omit it there — so this is now a pinned agreement rather than a divergence.)

### Auth — clock-skew and credential fixes

Ported from the robot's own device-auth implementation; the two are deliberate twins and
must not drift.

- The refresh deadline runs on `time.monotonic()`, so no NTP step can move it.
- The cache hold is bounded on both sides: a `_MIN_CACHE_S` **floor** (bounds the grant rate)
  and a **cap** at the server's own `expires_in` (a duration, so a wrong clock cannot distort
  it). **Order matters** — the floor is the outermost bound. Applied the other way round the
  cap undid the floor, and a server answering `expires_in: 1` set the grant rate to 3600/hour.
- Both bounds carry the same refresh lead, so we re-grant *before* expiry rather than at the
  instant of death.
- `expires_in <= 0` means the server declared the token dead: the cap collapses to zero and
  only the floor remains, instead of falling back to the default lifetime.
- A refresh token could reach logs: `"grant returned no access_token: {data}"` embedded the
  whole grant response. It now names response **keys** only.
- `AuthError` carries `.status`; the refresh token is discarded only on a 4xx, not on a
  network blip that says nothing about its validity.
- `tests/test_auth.py` (new, 15 cases). Every fix is pinned by a mutation that fails exactly
  one test — the first version of two of these tests passed with the guard deleted.

### Fixes

- `protocol.py`'s documented `Jog` example used `{"base": {"x": 1.0}}` — the telemetry
  namespace, not the jog namespace. A robot parses that as `linear=0, angular=0`, an explicit
  **stop**, with no error. The example now shows `linear`/`angular` and says why.
- `README.md`'s status claim ("every frame this SDK builds validates") was asserted rather
  than measured, and was false while two builders had no test. It now states what the
  conformance suite actually covers.

### Known gaps

The 8 `xfail`s are the live list: the base jog namespace in `JogBuilder.base()`, missing legacy
`record` verbs, `PolicyStreamStatus`'s invented field names and true-defaulting `ok`, inverted
`ActionStatus.done` terminality, unmodelled `error` frames, tile-less `camera_layout` being
accepted, stateless `daemon_status` inventing an outage, and `model`/`capabilities` not being
exposed on `RobotInfo`.

Not addressed and larger than a bug fix: the session layer's blocking-send-on-the-event-loop
problem, `close()`/`connect()` thread lifecycle, and the absence of any hardware validation.
