"""Descriptor-driven motion helpers. The point of these tests is that NO joint name is
hard-coded in the SDK — swap the descriptor and the same code drives a different robot."""

import pytest

from nori_sdk.mock.robot import DEFAULT_DESCRIPTOR
from nori_sdk.motion import (
    JogBuilder,
    from_si,
    jog_rate,
    joint_short,
    joints_by_group,
    normalized_for,
    scale_to_range,
    state_to_si,
    to_si,
)
from nori_sdk.types import RobotDescriptor

DESCRIPTOR = RobotDescriptor.from_wire(DEFAULT_DESCRIPTOR)


def test_joint_short_strips_group_and_suffix():
    assert joint_short("left_arm_shoulder_pan.pos") == "shoulder_pan"
    assert joint_short("right_arm_gripper.pos") == "gripper"
    assert joint_short("shoulder_pan") == "shoulder_pan"  # idempotent


def test_joints_by_group_comes_from_the_descriptor():
    groups = joints_by_group(DESCRIPTOR)
    assert set(groups) == {"left_arm", "right_arm"}
    assert "shoulder_pan" in groups["left_arm"]
    assert len(groups["left_arm"]) == 5


def test_joints_by_group_follows_a_different_robot():
    other = RobotDescriptor.from_wire({"joints": ["left_arm_tentacle_curl.pos"]})
    assert joints_by_group(other) == {"left_arm": ["tentacle_curl"]}


def test_jog_builder_nests_and_clamps():
    payload = (
        JogBuilder(DESCRIPTOR)
        .arm("left", "shoulder_pan", 5.0)  # over range: clamped, not rejected
        .lift("left", -0.25)
        .base(linear=1.0)
        .build()
    )
    assert payload == {
        "left_arm": {"shoulder_pan": 1.0},
        "left_lift": -0.25,
        "base": {"linear": 1.0},
    }


def test_base_rejects_the_telemetry_namespace():
    # x.vel/theta.vel is what the descriptor lists; a robot reads x/theta in a JOG as
    # linear=0, angular=0 — a silent stop. Refusing beats translating: a quiet alias would
    # leave every caller believing the two namespaces are interchangeable.
    with pytest.raises(ValueError, match="telemetry namespace"):
        JogBuilder(DESCRIPTOR).base(x=1.0)


def test_base_omits_the_axis_you_did_not_set():
    # Absence is zero for that axis, so a turn-in-place needs no explicit linear=0.
    assert JogBuilder(DESCRIPTOR).base(angular=-0.5).build() == {"base": {"angular": -0.5}}


def test_base_never_negates_either_axis():
    # REP-103 in, REP-103 out: +linear forward, +angular LEFT, verbatim on the wire (the
    # spec fixture pins the same thing when the spec is present; this copy runs without it).
    # The one robot that turns opposite (deployed L2, angular only) is compensated in the
    # TS SDK's L2 model gate — never here, never for any other model.
    assert JogBuilder(DESCRIPTOR).base(linear=0.4, angular=0.3).build() == {
        "base": {"linear": 0.4, "angular": 0.3}
    }


def test_base_is_refused_on_a_robot_that_has_none():
    armonly = RobotDescriptor.from_wire({"joints": ["left_arm_gripper.pos"], "base": []})
    with pytest.raises(ValueError, match="no base"):
        JogBuilder(armonly).base(linear=0.5)


def test_jog_builder_rejects_a_dof_this_robot_lacks():
    # Silently dropping it robot-side would look like "the SDK is broken".
    with pytest.raises(ValueError, match="no DOF 'tentacle_curl'"):
        JogBuilder(DESCRIPTOR).arm("left", "tentacle_curl", 1.0)


def test_jog_builder_permissive_mode_skips_the_check():
    payload = JogBuilder(DESCRIPTOR, strict=False).arm("left", "tentacle_curl", 1.0).build()
    assert payload == {"left_arm": {"tentacle_curl": 1.0}}


def test_stop_zeroes_every_group():
    stop = JogBuilder.stop()
    # Arms stay {} — their DOF names are per-robot, so there is no model-agnostic zero to
    # write. The base names are fixed, so it says the zeros out loud.
    assert stop["left_arm"] == {} and stop["left_lift"] == 0.0
    assert stop["base"] == {"linear": 0.0, "angular": 0.0}


def test_scale_to_range_refuses_to_guess():
    key = "left_arm_elbow_flex.pos"
    assert scale_to_range(DESCRIPTOR, key, 0.5) == 0.0  # midpoint of [-100, 100]
    assert scale_to_range(DESCRIPTOR, key, 1.0) == 100.0
    # Grippers carry no advertised range in this descriptor: None beats a plausible guess.
    assert scale_to_range(DESCRIPTOR, "left_arm_gripper.pos", 0.5) is None


# --- calibrated rates: descriptor.jog_scale ---------------------------------------------

SCALED = RobotDescriptor.from_wire({
    "joints": ["left_arm_elbow_pitch.pos", "left_arm_gripper.pos"],
    "base": ["x.vel", "theta.vel"],
    "aux": ["lift"],
    "jog_scale": {
        "joints": {"left_arm_elbow_pitch.pos": 12.2, "left_arm_gripper.pos": 22.0},
        "task": {"x": 0.08, "pitch": 0.5},
        "base": {"linear": 0.15, "angular": 0.60},
        "lift": 50.0,
    },
})


def test_jog_rate_reads_each_namespace_in_its_own_units():
    assert jog_rate(SCALED, "left_arm", "elbow_pitch") == 12.2   # norm units/s
    assert jog_rate(SCALED, "base", "linear") == 0.15            # m/s
    assert jog_rate(SCALED, "base", "angular") == 0.60           # rad/s
    assert jog_rate(SCALED, "lift") == 50.0                      # mm/s


# A3-shaped descriptor: full task table with canonical "yaw" plus deprecated "shoulder_pan".
A3 = RobotDescriptor.from_wire({
    "joints": ["left_arm_elbow_pitch.pos", "left_arm_gripper.pos"],
    "base": ["x.vel", "theta.vel"],
    "aux": ["lift"],
    "jog_scale": {
        "joints": {"left_arm_elbow_pitch.pos": 12.2},
        "task": {"x": 0.08, "y": 0.08, "z": 0.06, "pitch": 0.5, "yaw": 0.5,
                 "shoulder_pan": 0.5},
        "base": {"linear": 0.15, "angular": 0.60},
        "lift": 50.0,
    },
})


def test_strict_jog_builder_accepts_task_verbs_when_advertised():
    payload = (
        JogBuilder(A3)
        .arm("left", "x", 0.3)
        .arm("left", "yaw", -0.2)
        .arm("left", "z", 0.1)
        .build()
    )
    assert payload == {"left_arm": {"x": 0.3, "yaw": -0.2, "z": 0.1}}


def test_strict_jog_builder_still_rejects_task_verbs_without_jog_scale_task():
    # DESCRIPTOR carries no jog_scale at all -- behavior unchanged.
    for verb in ("x", "yaw", "z"):
        with pytest.raises(ValueError, match="no DOF"):
            JogBuilder(DESCRIPTOR).arm("left", verb, 0.5)


def test_yaw_and_shoulder_pan_alias_each_other_in_strict_mode():
    yaw_only = RobotDescriptor.from_wire({
        "joints": ["left_arm_elbow_pitch.pos"],
        "jog_scale": {"task": {"yaw": 0.5}},
    })
    pan_only = RobotDescriptor.from_wire({
        "joints": ["left_arm_elbow_pitch.pos"],
        "jog_scale": {"task": {"shoulder_pan": 0.5}},
    })
    # Deprecated alias accepted where only the canonical verb is advertised, and vice versa.
    assert JogBuilder(yaw_only).arm("left", "shoulder_pan", 0.5).build() == {
        "left_arm": {"shoulder_pan": 0.5}
    }
    assert JogBuilder(pan_only).arm("left", "yaw", 0.5).build() == {
        "left_arm": {"yaw": 0.5}
    }


def test_jog_rate_reads_yaw_and_z_from_the_task_table():
    assert jog_rate(A3, "left_arm", "yaw") == 0.5
    assert jog_rate(A3, "left_arm", "z") == 0.06
    assert jog_rate(A3, "left_arm", "shoulder_pan") == 0.5  # deprecated alias, advertised


def test_a_task_verb_is_not_looked_up_as_a_joint():
    """"pitch" and "x" are task-space VERBS with their own table -- on some models
    "shoulder_pan" is a task verb and not a joint name at all, so resolving verbs through the
    joint map would return the wrong number or silently nothing."""
    assert jog_rate(SCALED, "left_arm", "pitch") == 0.5
    assert jog_rate(SCALED, "left_arm", "x") == 0.08


def test_an_unadvertised_rate_is_None_never_a_guess():
    # The L2 fleet is frozen and will never publish jog_scale, so None is the common answer.
    assert jog_rate(DESCRIPTOR, "left_arm", "shoulder_pan") is None
    assert jog_rate(None, "base", "linear") is None
    assert jog_rate(SCALED, "left_arm", "tentacle_curl") is None
    assert jog_rate(SCALED, "right_arm", "elbow_pitch") is None  # scale is per joint KEY


def test_normalized_for_converts_a_real_rate_into_a_jog_value():
    assert normalized_for(SCALED, "base", "linear", rate=0.075) == pytest.approx(0.5)
    assert normalized_for(SCALED, "left_arm", "elbow_pitch", rate=6.1) == pytest.approx(0.5)
    assert normalized_for(SCALED, "lift", rate=-25.0) == pytest.approx(-0.5)


def test_asking_for_more_than_full_deflection_clamps():
    # Full deflection, not an out-of-range frame the robot would have to clamp anyway.
    assert normalized_for(SCALED, "base", "linear", rate=99.0) == 1.0
    assert normalized_for(SCALED, "base", "linear", rate=-99.0) == -1.0


def test_normalized_for_is_None_when_the_robot_published_no_scale():
    """Returning a guess here would command a speed that means something else on the next
    model -- the exact failure the normalized jog namespace exists to prevent."""
    assert normalized_for(DESCRIPTOR, "left_arm", "shoulder_pan", rate=1.0) is None
    assert normalized_for(None, "base", "linear", rate=0.1) is None


def test_a_nonpositive_published_rate_is_dropped_rather_than_believed():
    """0 is invalid per the schema -- "cannot be jogged" is expressed by OMITTING the key. A
    zero that survived parsing would scale every command to nothing, silently."""
    broken = RobotDescriptor.from_wire({
        "joints": ["a.pos"], "jog_scale": {"joints": {"a.pos": 0.0}, "base": {"linear": -1.0}},
    })
    assert broken.jog_scale is not None
    assert broken.jog_scale.joints == {} and broken.jog_scale.base == {}
    assert jog_rate(broken, "base", "linear") is None


def test_a_robot_with_no_jog_scale_parses_to_None_not_an_empty_scale():
    """None and an empty JogScale are different claims: "did not say" vs "said, named nothing"."""
    assert DESCRIPTOR.jog_scale is None


# --- physical units: descriptor.ranges_si ------------------------------------------------

SI = RobotDescriptor.from_wire({
    "joints": ["left_arm_shoulder_pitch.pos", "left_arm_gripper.pos", "lift.pos"],
    "aux": ["lift"],
    "ranges": {
        "left_arm_shoulder_pitch.pos": [-100.0, 100.0],   # body: symmetric convention
        "left_arm_gripper.pos": [0.0, 100.0],             # gripper: 0..100 convention
        "lift.pos": [0.0, 720.0],                         # ALREADY physical (mm)
    },
    "ranges_si": {
        "left_arm_shoulder_pitch.pos": [-1.92, 1.88],
        "left_arm_gripper.pos": [0.0, 0.043],
        # no lift entry: converting a value already in mm would convert twice
    },
})


def test_to_si_maps_a_body_joint_onto_its_calibrated_radians():
    assert to_si(SI, "left_arm_shoulder_pitch.pos", -100.0) == pytest.approx(-1.92)
    assert to_si(SI, "left_arm_shoulder_pitch.pos", 100.0) == pytest.approx(1.88)
    # Mid-normalized is mid-CALIBRATED, which is not zero on an asymmetric joint -- exactly
    # the offset that using nominal URDF limits would have silently discarded.
    assert to_si(SI, "left_arm_shoulder_pitch.pos", 0.0) == pytest.approx(-0.02)


def test_the_gripper_needs_no_special_case():
    """`ranges` already encodes the convention difference, so one linear map handles both. A
    hand-written gripper branch is the thing that rots when a robot changes convention."""
    assert to_si(SI, "left_arm_gripper.pos", 0.0) == pytest.approx(0.0)
    assert to_si(SI, "left_arm_gripper.pos", 100.0) == pytest.approx(0.043)
    assert to_si(SI, "left_arm_gripper.pos", 50.0) == pytest.approx(0.0215)


def test_a_key_with_no_si_bounds_is_None_never_a_guess():
    # The lift: `ranges` is already mm, so there is no SI entry and nothing to convert.
    assert to_si(SI, "lift.pos", 360.0) is None
    # And a robot that publishes no SI bounds at all -- the frozen L-series.
    assert to_si(DESCRIPTOR, "left_arm_shoulder_pan.pos", 0.0) is None
    assert to_si(None, "left_arm_shoulder_pitch.pos", 0.0) is None


def test_out_of_range_input_clamps_like_the_robot_does():
    assert to_si(SI, "left_arm_shoulder_pitch.pos", 999.0) == pytest.approx(1.88)
    assert to_si(SI, "left_arm_shoulder_pitch.pos", -999.0) == pytest.approx(-1.92)


def test_from_si_round_trips():
    for norm in (-100.0, -37.5, 0.0, 61.25, 100.0):
        rad = to_si(SI, "left_arm_shoulder_pitch.pos", norm)
        assert rad is not None
        assert from_si(SI, "left_arm_shoulder_pitch.pos", rad) == pytest.approx(norm)


def test_an_inverted_calibration_span_is_honoured_not_sorted():
    """A calibration can reverse an axis, and the ORDER of the span carries that. Sorting it
    into ascending order would flip the joint -- the arm would move the wrong way."""
    flipped = RobotDescriptor.from_wire({
        "joints": ["left_arm_wrist_roll.pos"],
        "ranges": {"left_arm_wrist_roll.pos": [-100.0, 100.0]},
        "ranges_si": {"left_arm_wrist_roll.pos": [2.79, -2.81]},
    })
    assert flipped.ranges_si["left_arm_wrist_roll.pos"] == (2.79, -2.81)
    assert to_si(flipped, "left_arm_wrist_roll.pos", -100.0) == pytest.approx(2.79)
    assert to_si(flipped, "left_arm_wrist_roll.pos", 100.0) == pytest.approx(-2.81)
    # and the inverse still round-trips through the reversed span
    assert from_si(flipped, "left_arm_wrist_roll.pos", 2.79) == pytest.approx(-100.0)


def test_state_to_si_omits_what_it_cannot_convert():
    """A dict silently mixing radians and normalized units is worse than a smaller one:
    nothing downstream can tell which key is in which unit."""
    out = state_to_si(SI, {
        "left_arm_shoulder_pitch.pos": 0.0,
        "lift.pos": 360.0,        # no SI bounds -> omitted, not passed through
        "x.vel": 0.4,             # not a joint at all
    })
    assert set(out) == {"left_arm_shoulder_pitch.pos"}
    assert out["left_arm_shoulder_pitch.pos"] == pytest.approx(-0.02)


def test_a_robot_without_si_bounds_yields_an_empty_conversion():
    assert state_to_si(DESCRIPTOR, {"left_arm_shoulder_pan.pos": 10.0}) == {}
