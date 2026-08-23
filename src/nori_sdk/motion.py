"""Descriptor-driven motion helpers: build valid jog and action payloads for whatever robot
you are actually connected to.

The lesson from the TypeScript side is here in the negative: over there the DOF vocabulary
got re-derived in four separate places (the SDK's key tables, robot-ops, the script driver,
and the app's own keymap), and adding the 7-DOF arm meant finding all of them. So this module
hard-codes NO joint names. Everything derives from RobotInfo.descriptor, which the robot
sends in its `ack`, and a robot with joints this SDK has never heard of works unchanged.

Units:
  jog     — normalized RATES in [-1, 1] per DOF. The robot scales by its per-tick step.
  action  — ABSOLUTE targets in RobotInfo.norm_mode units (normally "range_m100_100");
            grippers are [0, 100]. Out-of-range values are CLAMPED robot-side, never
            rejected, so `descriptor.ranges` is for scaling your input, not for validating.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from .types import RobotDescriptor

# Wire group names for the nested `jog` payload. Arms and base take a dict of DOF -> rate;
# the lifts take a bare float because a robot has one lift per arm, not a joint group.
ARM_GROUPS = ("left_arm", "right_arm")
LIFT_GROUPS = ("left_lift", "right_lift")
# A3 carries ONE central telescoping lift rather than L2's pair, and addresses it with a bare
# "lift". The jog vocabulary for it is not settled upstream yet, so nothing here builds an A3
# lift frame -- this name exists only so jog_rate() can answer for a robot that publishes a
# single lift scale. See MODELS.md.
LIFT_GROUP = "lift"
BASE_GROUP = "base"


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return low if value < low else min(value, high)


def joint_group(joint_key: str) -> str | None:
    """Which jog group a descriptor joint key belongs to. "left_arm_shoulder_pan.pos" ->
    "left_arm". Returns None for a key that isn't an arm joint."""
    for group in ARM_GROUPS:
        if joint_key.startswith(group + "_"):
            return group
    return None


def joint_short(joint_key: str) -> str:
    """The DOF name a jog payload uses, from a descriptor joint key:
    "left_arm_shoulder_pan.pos" -> "shoulder_pan". Idempotent on already-short names."""
    name = joint_key.removesuffix(".pos")
    for group in ARM_GROUPS:
        if name.startswith(group + "_"):
            return name[len(group) + 1 :]
    return name


def joints_by_group(descriptor: RobotDescriptor | None) -> dict[str, list[str]]:
    """Every arm DOF the connected robot actually has, grouped for jogging:
    {"left_arm": ["shoulder_pan", "elbow_flex", ...], "right_arm": [...]}.

    This is the call that makes a client model-agnostic — build your UI, your key bindings
    or your action space from it rather than from a constant."""
    out: dict[str, list[str]] = {}
    if descriptor is None:
        return out
    for key in descriptor.joints:
        group = joint_group(key)
        if group is None:
            continue
        out.setdefault(group, []).append(joint_short(key))
    return out


class JogBuilder:
    """Accumulates one `jog` frame across several DOFs, then emits it.

        jog = (JogBuilder(info.descriptor)
               .arm("left", "shoulder_pan", 0.5)
               .lift("left", -0.2)
               .base(x=1.0)
               .build())

    Setting a DOF the robot doesn't have raises ValueError — a typo'd joint name would
    otherwise be silently dropped by the robot and read as "the SDK is broken". Pass
    strict=False to skip the check (useful against a robot whose descriptor you know is
    incomplete)."""

    def __init__(self, descriptor: RobotDescriptor | None = None, strict: bool = True) -> None:
        self._descriptor = descriptor
        self._strict = strict and descriptor is not None
        self._known = joints_by_group(descriptor)
        self._payload: dict[str, Any] = {}

    def arm(self, side: str, dof: str, rate: float) -> JogBuilder:
        group = f"{side}_arm" if not side.endswith("_arm") else side
        if self._strict and dof not in self._known.get(group, []):
            available = ", ".join(self._known.get(group, [])) or "none"
            raise ValueError(f"{group} has no DOF {dof!r} on this robot (has: {available})")
        self._payload.setdefault(group, {})[dof] = clamp(rate)
        return self

    def lift(self, side: str, rate: float) -> JogBuilder:
        group = f"{side}_lift" if not side.endswith("_lift") else side
        if (
            self._strict
            and self._descriptor is not None
            and group not in self._descriptor.aux
            and not any(group in key for key in self._descriptor.joints)
        ):
            raise ValueError(f"this robot has no {group}")
        self._payload[group] = clamp(rate)
        return self

    def base(
        self, linear: float | None = None, angular: float | None = None, **rejected: float
    ) -> JogBuilder:
        """Base drive intent as normalized rates: base(linear=0.4, angular=-0.2).

        The jog namespace is `linear`/`angular`. It is NOT the "x.vel"/"theta.vel" pair the
        ack descriptor lists under `base` — those are the TELEMETRY namespace. Sending x/theta
        here builds an object the robot parses as linear=0, angular=0: an explicit STOP, with
        no error raised anywhere. This SDK shipped that bug, which is why the old spelling is
        rejected loudly instead of being quietly translated — a silent mistranslation here is
        indistinguishable from a robot that ignores you.

        SIGN CONVENTION (normative, shared by every client): +linear drives the robot FORWARD,
        +angular turns it LEFT (counter-clockwise, ROS REP-103). Never negate on the client
        side; a robot whose hardware turns the other way compensates in its own actuator
        adapter. Omitting a key means zero for that axis, and omitting the whole `base` object
        means STOP — never "hold the last velocity"."""
        if rejected:
            raise ValueError(
                f"base() takes linear/angular, not {'/'.join(sorted(rejected))}. "
                "x.vel/theta.vel is the telemetry namespace: a robot reads x/theta in a jog "
                "as linear=0, angular=0 — a silent stop."
            )
        if self._strict and self._descriptor is not None and not self._descriptor.base:
            raise ValueError("this robot has no base (descriptor.base is empty)")
        group = self._payload.setdefault(BASE_GROUP, {})
        if linear is not None:
            group["linear"] = clamp(linear)
        if angular is not None:
            group["angular"] = clamp(angular)
        return self

    def build(self) -> dict[str, Any]:
        return dict(self._payload)

    @staticmethod
    def stop(groups: Iterable[str] = ARM_GROUPS + LIFT_GROUPS + (BASE_GROUP,)) -> dict[str, Any]:
        """An explicit all-zero jog. Prefer this to simply stopping the stream: the watchdog
        WILL ramp the robot down on silence, but a zero frame stops it now and keeps the
        session's dead-man clock fed."""
        payload: dict[str, Any] = {}
        for group in groups:
            if group in LIFT_GROUPS:
                payload[group] = 0.0
            elif group == BASE_GROUP:
                # Explicit zeros rather than {}. Both stop the robot — absence of a base key
                # reads as zero — but the arms cannot do this (their DOF names are per-robot,
                # so {} is the only model-agnostic spelling) and the base can, so it should.
                payload[group] = {"linear": 0.0, "angular": 0.0}
            else:
                payload[group] = {}
        return payload


def scale_to_range(
    descriptor: RobotDescriptor | None, joint_key: str, fraction: float
) -> float | None:
    """Map a 0..1 fraction onto a joint's advertised [min, max]. Returns None when the robot
    didn't advertise a range for that joint, which is the honest answer — guessing a range
    is how a client ends up commanding a number that means something else on the next model.
    """
    if descriptor is None:
        return None
    span = descriptor.ranges.get(joint_key)
    if span is None:
        return None
    low, high = span
    return low + (high - low) * clamp(fraction, 0.0, 1.0)


# --- calibrated rates ----------------------------------------------------------------------
# The jog wire is normalized [-1, 1] and the robot owns what "full deflection" means. That is
# what keeps one client working across models -- but it also means a script cannot ask for a
# REPEATABLE speed, or discover what it just asked for. A robot that publishes
# descriptor.jog_scale closes that without any new command.


def jog_rate(descriptor: RobotDescriptor | None, group: str, dof: str = "") -> float | None:
    """The rate this robot commands at FULL deflection, or None if it didn't advertise one.

        jog_rate(d, "left_arm", "elbow_pitch")  -> 12.2   (norm units/s)
        jog_rate(d, "base", "linear")           -> 0.15   (m/s)
        jog_rate(d, "lift")                     -> 50.0   (mm/s)

    None is the common answer and must be handled: the L2 fleet is frozen and will never
    publish this. Units are per namespace -- see JogScale, and do not mix them up.

    NOMINAL COMMANDED, not achieved. The watchdog's warn state scales motion to zero, an
    acceleration limit means short jogs never reach the rate, and Servo scales near limits."""
    if descriptor is None or descriptor.jog_scale is None:
        return None
    scale = descriptor.jog_scale
    if group == LIFT_GROUP or group in LIFT_GROUPS:
        return scale.lift
    if group == BASE_GROUP:
        return scale.base.get(dof)
    if group in ARM_GROUPS:
        # A task-space VERB ("x", "pitch") is not a joint and lives in its own table -- on
        # some models "shoulder_pan" is a task verb and not a joint name at all.
        if dof in scale.task:
            return scale.task[dof]
        return scale.joints.get(f"{group}_{dof}.pos")
    return None


def normalized_for(
    descriptor: RobotDescriptor | None, group: str, dof: str = "", *, rate: float = 0.0
) -> float | None:
    """The normalized jog value that asks for `rate` real units per second.

        normalized_for(d, "base", "linear", rate=0.075)  -> 0.5   (half of 0.15 m/s)

    None when the robot published no scale for that DOF -- never a guess. Guessing is how a
    client ends up commanding a number that means something else on the next model, which is
    the entire reason the jog namespace is normalized in the first place.

    The result is CLAMPED to [-1, 1]: asking for more than the robot's full deflection gives
    you full deflection, not an out-of-range frame. Compare what you asked for against what
    telemetry reports to find out whether you actually got it."""
    full = jog_rate(descriptor, group, dof)
    if full is None or full <= 0:
        return None
    return clamp(rate / full)


__all__ = [
    "ARM_GROUPS",
    "BASE_GROUP",
    "LIFT_GROUP",
    "LIFT_GROUPS",
    "JogBuilder",
    "clamp",
    "jog_rate",
    "joint_group",
    "joint_short",
    "joints_by_group",
    "normalized_for",
    "scale_to_range",
]
