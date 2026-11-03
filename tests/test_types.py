"""Parser tolerance. Every one of these is a mixed-fleet scenario, not a hypothetical: the
SDK must keep working against a robot older or newer than itself."""

from nori_sdk.types import CameraLayout, DaemonStatus, RecordState, RobotInfo, Telemetry


def test_bare_ack_counts_as_accepted():
    # An old robot sends {"type":"ack"} and nothing else.
    info = RobotInfo.from_wire({"type": "ack"})
    assert info.accepted
    assert info.descriptor is None
    assert not info.version_mismatch  # unknown version is not a mismatch


def test_refused_ack_carries_the_reason():
    info = RobotInfo.from_wire({"type": "ack", "accepted": False, "error": "busy"})
    assert not info.accepted and info.error == "busy"


def test_version_mismatch_is_advisory_not_fatal():
    info = RobotInfo.from_wire({"type": "ack", "protocol_version": 2}, sdk_protocol_version=1)
    assert info.version_mismatch and info.accepted


def test_descriptor_ranges_survive_and_bad_ones_drop():
    info = RobotInfo.from_wire(
        {
            "type": "ack",
            "descriptor": {
                "joints": ["left_arm_elbow_flex.pos"],
                "ranges": {"left_arm_elbow_flex.pos": [-100, 100], "bogus": ["a", "b"]},
            },
        }
    )
    assert info.descriptor is not None
    assert info.descriptor.ranges == {"left_arm_elbow_flex.pos": (-100.0, 100.0)}


def test_telemetry_drops_a_zero_temperature():
    # 0 means "no sensor", not "freezing" — matching the TS SDK.
    assert Telemetry.from_wire({"type": "telemetry", "pi_temp_c": 0}).pi_temp_c is None
    assert Telemetry.from_wire({"type": "telemetry", "pi_temp_c": 51.2}).pi_temp_c == 51.2


def test_telemetry_ignores_non_numeric_state_entries():
    tel = Telemetry.from_wire(
        {"type": "telemetry", "state": {"a.pos": 1.5, "b.pos": "nope", "c.pos": True}}
    )
    assert tel.state == {"a.pos": 1.5}  # bools are not numbers here


def test_camera_layout_rect_slices_the_composite():
    layout = CameraLayout.from_wire(
        {"type": "camera_layout", "cols": 2, "rows": 2, "tiles": ["a", "b", "c", "d"]}
    )
    assert layout is not None
    assert layout.rect("a") == (0.0, 0.0, 0.5, 0.5)
    assert layout.rect("d") == (0.5, 0.5, 0.5, 0.5)
    assert layout.rect("nope") is None


def test_camera_layout_rejects_a_degenerate_grid():
    assert CameraLayout.from_wire({"type": "camera_layout", "cols": 0, "rows": 2}) is None


def test_daemon_status_drops_a_stateless_frame_rather_than_inventing_an_outage():
    # None means "drop this and keep the health you had". Coercing to offline would let one
    # malformed repeat — and the bridge repeats while offline — flip a healthy robot.
    assert DaemonStatus.from_wire({"type": "daemon_status"}) is None
    assert DaemonStatus.from_wire({"type": "daemon_status", "state": ""}) is None

    online = DaemonStatus.from_wire({"type": "daemon_status", "state": "online"})
    assert online is not None and online.online
    offline = DaemonStatus.from_wire(
        {"type": "daemon_status", "state": "offline", "reason": "connection_lost"}
    )
    assert offline is not None and not offline.online and offline.reason == "connection_lost"


def test_record_state_parses_a_refusal():
    state = RecordState.from_wire(
        {"type": "record_status", "ok": False, "error": "not recording"}
    )
    assert not state.ok and state.error == "not recording"


def test_telemetry_carries_stalled_joints_per_frame():
    from nori_sdk.types import Telemetry
    assert Telemetry.from_wire({"type": "telemetry", "stalled": ["right_wrist_roll_joint", 3]}).stalled == ["right_wrist_roll_joint"]
    assert Telemetry.from_wire({"type": "telemetry"}).stalled == []            # absent on the wire = none this frame
    assert Telemetry.from_wire({"type": "telemetry", "currents": {"right_elbow_pitch_joint": 212, "x": "no"}}).currents == {"right_elbow_pitch_joint": 212.0}


def test_alert_parses_the_no_ik_jog_limit():
    from nori_sdk.types import Alert
    a = Alert.from_wire({"type": "alert", "code": "task_jog_limit", "severity": "warning", "active": True,
                         "message": "right arm cannot reach further with the tool held at this angle.", "ts_ns": 5})
    assert a is not None and a.code == "task_jog_limit" and a.active and "right arm" in a.message and a.ts_ns == 5
    assert Alert.from_wire({"type": "alert", "active": True}) is None             # no code: not an alert
