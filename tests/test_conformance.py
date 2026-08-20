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
TODO list, in other words.
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


# --- known divergences, self-retiring --------------------------------------------------


def test_base_jog_uses_the_jog_namespace():
    """The jog namespace is linear/angular; descriptor.base's x.vel/theta.vel is the
    TELEMETRY namespace. A robot reads x/theta here as linear=0, angular=0 -- an explicit
    STOP, with no error. See MODELS.md."""
    frame = protocol.control_jog(1, JogBuilder().base(linear=1.0, angular=0.5).build())
    assert_valid(frame)
    assert frame["jog"]["base"] == {"linear": 1.0, "angular": 0.5}


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


@pytest.mark.xfail(
    strict=True,
    reason="BUG: RecordVerb omits the legacy aliases (start/stop/discard/discard_last/"
    "session_discard) that deployed clients still send and robots still accept.",
)
def test_record_accepts_every_verb_the_spec_allows():
    schema = load_schema("record")
    for action in schema["properties"]["action"]["enum"]:
        assert action in protocol.RecordVerb.__args__, f"{action} missing from RecordVerb"


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


@pytest.mark.xfail(
    strict=True,
    reason="BUG: `error` frames are unmodelled -- not in INBOUND_KINDS, no dataclass. A "
    "fatal robot error reaches the caller only as an untyped raw dict.",
)
def test_error_frames_are_modelled():
    assert "error" in protocol.INBOUND_KINDS
    _kind, parsed, _raw = protocol.decode('{"type":"error","code":"x","msg":"y","fatal":true}')
    assert parsed is not None


@pytest.mark.xfail(
    strict=True,
    reason="BUG: a tile-less camera_layout is accepted and would blank a good layout. The "
    "spec requires minItems 1; the robot re-sends the layout several times, so one "
    "malformed repeat must not poison the session.",
)
def test_tile_less_layout_is_rejected():
    assert CameraLayout.from_wire({"type": "camera_layout", "cols": 2, "rows": 2, "tiles": []}) is None


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


@pytest.mark.xfail(
    strict=True,
    reason="GAP: RobotInfo models neither `model` nor `capabilities`, so this SDK cannot "
    "gate optional verbs and cannot name the hardware in a log.",
)
def test_ack_exposes_model_and_capabilities():
    info = RobotInfo.from_wire(dict(FIXTURES)["daemon/ack_l3.json"])
    assert getattr(info, "model", None) == "L3"
    assert "record" in getattr(info, "capabilities", [])
