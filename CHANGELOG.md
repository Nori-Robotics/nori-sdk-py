# Changelog

Newest first. This package is pre-release: it targets **nori-protocol v1** and is not yet
verified against a physical robot — see the Status section of `README.md` for what that means
in practice.

## Unreleased — 2026-08-20

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

## Unreleased — 2026-08-13

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

Ported from `nori_identity/device_auth.py`; the two modules are deliberate twins and must not
drift. Full rationale in `nori_ws/docs/known_issues.md` entry 18.

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
