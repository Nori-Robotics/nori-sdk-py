"""Descriptor-driven motion helpers. The point of these tests is that NO joint name is
hard-coded in the SDK — swap the descriptor and the same code drives a different robot."""

import pytest

from nori_sdk.mock.robot import DEFAULT_DESCRIPTOR
from nori_sdk.motion import JogBuilder, joint_short, joints_by_group, scale_to_range
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
