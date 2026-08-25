"""Conformance against the nori-protocol spec.

These are the tests that make this SDK's wire behaviour checkable instead of asserted. They
run against the SAME golden fixtures and schemas the robot gateway and the TypeScript SDK
are meant to validate against, so a divergence fails here rather than on a robot.

Two directions, and the second is the one that matters:

  INBOUND  every golden frame a robot can send must decode without error, and the fields
           this SDK models must survive the parse.
  OUTBOUND every frame this SDK builds must validate against the schema. This is what
           catches "we invented a field name", which is exactly how the base jog ended up
           addressing DOFs no robot has ever read.

Known divergences are marked `xfail(strict=True)` rather than deleted or left red. That way
the suite stays green, each gap is documented with its consequence, and the moment somebody
fixes one the test XPASSes and fails the build -- forcing the marker off. A self-retiring
TODO list, in other words. As of this revision the list is EMPTY: all eight divergences have
been fixed, and each former xfail is now an ordinary test guarding the fix.
"""

from __future__ import annotations

import json

import pytest

from _spec import SPEC_DIR, load_fixtures, load_schema
from nori_sdk import protocol
from nori_sdk.motion import JogBuilder
from nori_sdk.types import (
    TERMINAL_ACTION_STATES,
    ActionStatus,
    CameraLayout,
    DaemonStatus,
    PolicyStreamStatus,
    RobotError,
    RobotInfo,
)

jsonschema = pytest.importorskip("jsonschema")

pytestmark = pytest.mark.skipif(
    SPEC_DIR is None,
    reason="nori-protocol spec not found; set NORI_PROTOCOL_DIR or init the submodule",
)

FIXTURES = load_fixtures()

# Robot -> client. Everything else is client -> robot.
INBOUND_TYPES = {
    "ack",
    "telemetry",
    "action_status",
    "error",
    "perception",
    "camera_layout",
    "daemon_status",
    "record_status",
    "policy_stream_status",
}


def assert_valid(frame: dict) -> None:
    schema = load_schema(frame["type"])
    assert schema is not None, f"no schema for type={frame['type']!r}"
    errors = sorted(
        jsonschema.Draft202012Validator(schema).iter_errors(frame), key=lambda e: list(e.path)
    )
    if errors:
        detail = "\n".join(
            f"  {'/'.join(map(str, e.path)) or '<root>'}: {e.message}" for e in errors
        )
        pytest.fail(f"{frame['type']} frame does not match the spec:\n{detail}")


def ids(items):
    return [name for name, _ in items]


# --- sanity ---------------------------------------------------------------------------


def test_the_spec_was_actually_found():
    assert FIXTURES, "no fixtures loaded — the spec directory resolved but is empty"


def test_sdk_targets_the_spec_version():
    """A version skew here means every other assertion in this file is checking the SDK
    against a contract it does not claim to implement."""
    version = int((SPEC_DIR / "VERSION").read_text().strip())
    from nori_sdk.version import NORI_PROTOCOL_VERSION

    assert NORI_PROTOCOL_VERSION == version, (
        f"this SDK targets protocol v{NORI_PROTOCOL_VERSION} but the spec is v{version}"
    )


# --- inbound: every golden frame must decode ------------------------------------------


@pytest.mark.parametrize("name,frame", FIXTURES, ids=ids(FIXTURES))
def test_every_golden_frame_decodes(name, frame):
    kind, parsed, raw = protocol.decode(json.dumps(frame))
    assert kind == frame["type"], f"{name}: decoded as {kind!r}"
    assert raw == frame, f"{name}: raw dict was altered during decode"
    if kind in INBOUND_TYPES and kind in protocol.INBOUND_KINDS:
        assert parsed is not None, f"{name}: modelled type produced no parsed object"


@pytest.mark.parametrize(
    "name,frame", [f for f in FIXTURES if f[1]["type"] == "ack"], ids=lambda v: v
)
def test_ack_descriptor_survives_the_parse(name, frame):
    info = RobotInfo.from_wire(frame)
    assert info.accepted is (frame.get("accepted") is not False)
    wire_desc = frame.get("descriptor")
    if wire_desc is None:
        assert info.descriptor is None, "an absent descriptor must stay absent (legacy signal)"
    else:
        assert info.descriptor is not None
        assert info.descriptor.joints == wire_desc.get("joints", [])
        assert info.descriptor.aux == wire_desc.get("aux", [])


def test_legacy_ack_is_distinguishable_from_a_descriptor_robot():
    """The whole model-discovery rule rests on this being detectable."""
    by_name = dict(FIXTURES)
    legacy = RobotInfo.from_wire(by_name["daemon/ack_l2_legacy.json"])
    modern = RobotInfo.from_wire(by_name["daemon/ack_l3.json"])
    assert legacy.descriptor is None
    assert modern.descriptor is not None and len(modern.descriptor.joints) == 16


# --- outbound: everything this SDK builds must validate --------------------------------


def test_control_action_validates():
    assert_valid(protocol.control_action(1, {"left_arm_gripper.pos": 30.0}, "abc123"))


def test_control_pose_validates():
    assert_valid(protocol.control_pose(2, "right", [0.4, -0.1, 0.9], action_id="p-1"))
    assert_valid(protocol.control_pose(
        3, "left", [0.3, 0.2, 0.7], [0.0, 0.7071068, 0.0, 0.7071068]))


def test_command_validates():
    assert_valid(protocol.command("estop"))
    assert_valid(protocol.command("reset_latch"))


def test_video_verbs_validate():
    assert_valid(protocol.video_state(True))
    assert_valid(protocol.video_state(False))
    assert_valid(protocol.video_bitrate(800))
    assert_valid(protocol.video_quality("low"))


def test_link_validates():
    assert_valid(protocol.link("lan"))
    assert_valid(protocol.link("wan"))


def test_record_verbs_validate():
    for action in ("session_start", "episode_start", "episode_stop", "episode_discard",
                   "session_end", "status"):
        assert_valid(protocol.record(action, "demo task"))


def test_call_validates():
    assert_valid(protocol.call(state="join", mic_muted=True))
    assert_valid(protocol.call(mic_muted=False))


def test_policy_stream_validates():
    assert_valid(protocol.policy_stream("start", dest="laptop"))


def test_arm_jog_validates():
    assert_valid(protocol.control_jog(1, {"left_arm": {"shoulder_pitch": 0.5, "gripper": -1.0}}))


def test_control_reset_validates():
    """Had no coverage at all, which is how it went unnoticed that it builds a frame with no
    `seq`. The schema no longer requires seq (the L2 daemon defaults it to -1 and both
    clients omit it here), so this now pins the agreed shape rather than a divergence."""
    assert_valid(protocol.control_reset("left_arm"))
    assert_valid(protocol.control_reset("right_arm"))


def test_control_leader_validates():
    assert_valid(protocol.control_leader(1, {"left_arm_shoulder_pan.pos": 12.5}))


def test_lift_jog_validates():
    assert_valid(protocol.control_jog(1, {"left_lift": -0.25, "right_lift": 0.0}))


# --- former divergences: each of these was an xfail marking a real bug in this SDK ------
# They are kept as ordinary tests rather than deleted. The bug each one describes was live,
# and the comment above each explains the consequence, which is the part worth preserving:
# every one of them made the SDK report success while doing something else.


def test_base_jog_uses_the_jog_namespace():
    """The jog namespace is linear/angular; descriptor.base's x.vel/theta.vel is the
    TELEMETRY namespace. A robot reads x/theta here as linear=0, angular=0 -- an explicit
    STOP, with no error. See MODELS.md."""
    frame = protocol.control_jog(1, JogBuilder().base(linear=1.0, angular=0.5).build())
    assert_valid(frame)
    assert frame["jog"]["base"] == {"linear": 1.0, "angular": 0.5}


def test_base_signs_match_the_spec_fixture_verbatim():
    """The sign convention crosses three implementations (this SDK, @nori/sdk, the gateway)
    and the audit found all three disagreeing — this SDK was the only one already on the
    spec. The fixture IS the convention: linear 1.0 = full forward, angular 0.5 = half
    LEFT (REP-103), and the builder must reproduce it byte-for-byte with NO negation
    anywhere between base() and the wire. The L2 legacy flip is @nori/sdk's problem
    (RemoteTeleop.wireJog, positive-L2-gated); this SDK never negates."""
    fixture = dict(FIXTURES).get("daemon/control_jog_base.json")
    assert fixture is not None, "spec lost its base-jog fixture"
    built = protocol.control_jog(fixture["seq"], JogBuilder().base(linear=1.0, angular=0.5).build())
    assert built["jog"] == fixture["jog"]
    assert built["jog"]["base"]["angular"] == 0.5  # +left in, +left on the wire


def test_the_old_base_spelling_is_rejected_rather_than_translated():
    """Silently mapping x->linear would be worse than the bug: it would leave every caller
    believing the telemetry namespace is jogable, and the next DOF added under one name and
    not the other would fail silently again."""
    with pytest.raises(ValueError, match="telemetry namespace"):
        JogBuilder().base(x=1.0, theta=0.5)


def test_an_all_stop_jog_validates():
    """JogBuilder.stop() is the frame a script sends to halt now rather than waiting out the
    watchdog ramp. If it did not validate, the robot would drop it and keep moving."""
    assert_valid(protocol.control_jog(1, JogBuilder.stop()))


def test_record_accepts_every_verb_the_spec_allows():
    """Including the legacy aliases. Deployed clients still send them and robots still accept
    them, so an SDK that cannot even name them cannot describe what a fleet is doing."""
    schema = load_schema("record")
    for action in schema["properties"]["action"]["enum"]:
        assert action in protocol.RecordVerb.__args__, f"{action} missing from RecordVerb"


def test_no_invented_record_verbs():
    """The other direction. A verb this SDK offers but no robot accepts is a silent no-op."""
    allowed = set(load_schema("record")["properties"]["action"]["enum"])
    assert set(protocol.RecordVerb.__args__) <= allowed


def test_the_destructive_verb_set_matches_what_the_spec_says_destroys_data():
    """Data loss is the one thing here worth a guard rail. `discard` is deliberately excluded
    -- it destroys on L2 and keeps on A3, so no static set can classify it and a caller has to
    resolve it per robot."""
    assert "session_discard" in protocol.DESTRUCTIVE_RECORD_VERBS
    assert "episode_discard" in protocol.DESTRUCTIVE_RECORD_VERBS
    assert "session_end" not in protocol.DESTRUCTIVE_RECORD_VERBS
    assert "discard" not in protocol.DESTRUCTIVE_RECORD_VERBS
    assert protocol.DESTRUCTIVE_RECORD_VERBS <= set(protocol.RecordVerb.__args__)


def test_every_record_verb_builds_a_valid_frame():
    for action in protocol.RecordVerb.__args__:
        assert_valid(protocol.record(action))


def test_policy_stream_status_models_the_real_fields():
    frame = dict(dict(FIXTURES)["session/policy_stream_status.json"])
    parsed = PolicyStreamStatus.from_wire(frame)
    assert parsed.streaming is True
    assert parsed.dest == "laptop"
    assert parsed.fps_out == 14.8
    assert (parsed.frames_sent, parsed.dropped) == (4412, 7)


def test_a_partial_policy_stream_reply_does_not_read_as_success():
    """Inverted from this SDK's usual tolerance, on purpose: absent means false here, so a
    truncated or malformed reply cannot be mistaken for a running stream."""
    parsed = PolicyStreamStatus.from_wire({"type": "policy_stream_status"})
    assert parsed.ok is False
    assert parsed.streaming is False


def test_every_policy_stream_status_field_in_the_spec_is_modelled():
    """Guards the direction the fixture cannot: a field ADDED to the schema later should
    show up as a modelled attribute, not silently live only in `raw`."""
    schema = load_schema("policy_stream_status")
    modelled = set(PolicyStreamStatus.__dataclass_fields__) | {"type", "raw"}
    missing = set(schema["properties"]) - modelled
    assert not missing, f"unmodelled policy_stream_status fields: {sorted(missing)}"


def test_action_status_terminality_matches_the_spec():
    """accepted -> active -> done | blocked | clamped | timeout. Only the last four end it."""
    for state in ("accepted", "active"):
        assert ActionStatus.from_wire({"state": state}).done is False, state
    for state in ("done", "blocked", "clamped", "timeout"):
        assert ActionStatus.from_wire({"state": state}).done is True, state


def test_terminal_action_states_are_exactly_the_non_progress_states_in_the_schema():
    """Pins the split to the schema rather than to a list I typed. A state added to the enum
    later lands here as a failure instead of being silently treated as non-terminal forever."""
    schema = load_schema("action_status")
    enum = set(schema["properties"]["state"]["enum"])
    assert TERMINAL_ACTION_STATES | {"accepted", "active"} == enum


def test_a_clamped_action_is_done_but_not_successful():
    """The distinction a caller acts on: the robot finished, somewhere other than asked."""
    clamped = ActionStatus.from_wire({"state": "clamped", "reason": "clamp:left_arm_gripper"})
    assert clamped.done is True and clamped.succeeded is False
    assert ActionStatus.from_wire({"state": "done"}).succeeded is True


def test_an_unknown_action_state_is_not_treated_as_finished():
    """A newer robot's state this SDK has never heard of must fall through to the caller's
    own timeout, never be reported as a completed move."""
    assert ActionStatus.from_wire({"state": "recalibrating"}).done is False


def test_error_frames_are_modelled():
    assert "error" in protocol.INBOUND_KINDS
    _kind, parsed, _raw = protocol.decode('{"type":"error","code":"x","msg":"y","fatal":true}')
    assert isinstance(parsed, RobotError)
    assert parsed.code == "x" and parsed.msg == "y" and parsed.fatal is True


def test_a_non_fatal_error_is_not_reported_as_fatal():
    """`fatal` defaults FALSE per the schema, and fatal is what ends a session. Defaulting it
    true would tear down a live connection over a soft stall notice."""
    _kind, parsed, _raw = protocol.decode('{"type":"error","code":"obstruction","msg":"hit"}')
    assert isinstance(parsed, RobotError) and parsed.fatal is False


def test_recovery_codes_are_distinguishable_from_faults():
    """Three `error` codes report a fault CLEARING. A client that renders every error frame as
    a fault shows a red banner for the news that the problem went away."""
    for code in ("obstruction_cleared", "arm_recovered", "motor_recovered"):
        assert RobotError.from_wire({"type": "error", "code": code, "msg": ""}).recovered
    assert not RobotError.from_wire({"type": "error", "code": "overtemp", "msg": ""}).recovered


def test_an_unknown_error_code_still_parses_and_keeps_its_message():
    """`code` is an open set -- newer robots add codes freely. The operator-readable `msg` is
    what a client shows, so it must survive a code this SDK has never seen."""
    err = RobotError.from_wire(
        {"type": "error", "code": "flux_capacitor_desync", "msg": "see manual", "fatal": True}
    )
    assert err.code == "flux_capacitor_desync" and err.msg == "see manual" and err.fatal


def test_tile_less_layout_is_rejected():
    """The robot re-sends the layout several times on open, so one malformed repeat must not
    be able to blank a good layout for the rest of the session."""
    for bad in ({"cols": 2, "rows": 2, "tiles": []}, {"cols": 2, "rows": 2}):
        assert CameraLayout.from_wire({"type": "camera_layout", **bad}) is None


@pytest.mark.parametrize(
    "name,frame", [f for f in FIXTURES if f[1]["type"] == "camera_layout"], ids=lambda v: v
)
def test_every_golden_layout_survives_the_rejection_rule(name, frame):
    """The other half: rejecting the malformed must not reject the real."""
    parsed = CameraLayout.from_wire(frame)
    assert parsed is not None, f"{name}: a valid layout was rejected"
    assert parsed.tiles == frame["tiles"]


def test_stateless_daemon_status_is_dropped():
    """Coercing a missing state to "offline" invents an outage. The bridge rebroadcasts while
    offline, so one malformed repeat would flip a healthy robot in every watching UI."""
    assert DaemonStatus.from_wire({"type": "daemon_status"}) is None
    assert DaemonStatus.from_wire({"type": "daemon_status", "state": ""}) is None


@pytest.mark.parametrize(
    "name,frame", [f for f in FIXTURES if f[1]["type"] == "daemon_status"], ids=lambda v: v
)
def test_every_golden_daemon_status_survives_the_drop_rule(name, frame):
    """The other half: the drop must not swallow real frames."""
    parsed = DaemonStatus.from_wire(frame)
    assert parsed is not None, f"{name}: a valid frame was dropped"
    assert parsed.state == frame["state"]
    assert parsed.online is (frame["state"] == "online")


def test_ack_exposes_model_and_capabilities():
    info = RobotInfo.from_wire(dict(FIXTURES)["daemon/ack_l3.json"])
    assert info.model == "L3"
    assert info.capabilities is not None and "record" in info.capabilities
    assert info.supports("record") is True
    assert info.supports("call") is False


def test_an_absent_capabilities_list_is_UNKNOWN_not_none_supported():
    """The distinction the spec insists on. A robot predating the field must be probed or
    assumed legacy; folding absent into "supports nothing" would silently disable working
    features on every older robot in the fleet."""
    silent = RobotInfo.from_wire({"type": "ack"})
    assert silent.capabilities is None
    assert silent.supports("record") is None, "absent was collapsed into False"

    explicit = RobotInfo.from_wire({"type": "ack", "capabilities": []})
    assert explicit.capabilities == []
    assert explicit.supports("record") is False, "an explicit empty list is not unknown"


def test_an_unrecognised_capability_is_ignored_not_an_error():
    info = RobotInfo.from_wire({"type": "ack", "capabilities": ["record", "teleportation"]})
    assert info.supports("teleportation") is True
    assert info.supports("record") is True
