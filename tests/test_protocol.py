"""Wire-vocabulary tests. These are the ones that must never be relaxed to make a client
pass — they pin the shapes the robot gateway and the TypeScript SDK also implement.

When conformance vectors land (see README "Staying in sync"), the literals here become the
Python half of a shared corpus rather than a hand-maintained second opinion.
"""

import json

from nori_sdk import protocol
from nori_sdk.types import CameraLayout, DaemonStatus, RobotInfo, Telemetry


def test_control_jog_shape():
    frame = protocol.control_jog(7, {"left_arm": {"shoulder_pan": 0.5}})
    assert frame == {"type": "control", "seq": 7, "jog": {"left_arm": {"shoulder_pan": 0.5}}}


def test_control_action_omits_empty_action_id():
    assert "action_id" not in protocol.control_action(1, {"left_arm_gripper.pos": 30})
    assert protocol.control_action(1, {"a.pos": 1}, "abc")["action_id"] == "abc"


def test_command_is_a_flag_not_a_name():
    # The robot accepts {"name": "estop"} too, but the SDK dialect is the flag form; the
    # gateway checks `message.get("estop")` first.
    assert protocol.command("estop") == {"type": "command", "estop": True}
    assert protocol.command("reset_latch") == {"type": "command", "reset_latch": True}


def test_reset_rides_control_not_command():
    # A real trap: reset is a `control` frame with a per-arm dict, NOT a command.
    assert protocol.control_reset("left_arm") == {
        "type": "control",
        "reset": {"left_arm": True},
    }


def test_video_verbs():
    assert protocol.video_state(True) == {"type": "video", "state": "pause"}
    assert protocol.video_state(False) == {"type": "video", "state": "resume"}
    assert protocol.video_bitrate(800) == {"type": "video", "bitrate": 800}


def test_record_omits_empty_task():
    assert protocol.record("episode_start") == {"type": "record", "action": "episode_start"}
    assert protocol.record("episode_start", "fold towel")["task"] == "fold towel"


def test_encode_is_compact():
    assert protocol.encode({"type": "command", "estop": True}) == '{"type":"command","estop":true}'


def test_decode_dispatches_every_known_kind():
    cases = {
        "ack": RobotInfo,
        "telemetry": Telemetry,
        "camera_layout": CameraLayout,
        "daemon_status": DaemonStatus,
    }
    for kind, expected in cases.items():
        payload = {"type": kind}
        if kind == "camera_layout":
            payload |= {"cols": 2, "rows": 2, "tiles": ["a"]}
        if kind == "daemon_status":
            # A stateless frame parses to None on purpose (see types.DaemonStatus.from_wire),
            # so dispatch has to be checked with a frame that carries a state.
            payload |= {"state": "online"}
        parsed_kind, parsed, _raw = protocol.decode(json.dumps(payload))
        assert parsed_kind == kind
        assert isinstance(parsed, expected), kind


def test_decode_survives_garbage():
    # The control channel is unreliable and lossy by design: garbage is dropped, never raised.
    for junk in ("", "not json", "[1,2,3]", "null", b"\xff\xfe"):
        assert protocol.decode(junk) == ("", None, {})


def test_decode_keeps_unknown_kinds_reachable():
    # Forward compat: a newer robot's vocabulary must reach the caller unparsed rather than
    # being silently swallowed.
    kind, parsed, raw = protocol.decode('{"type":"weather","sunny":true}')
    assert kind == "weather"
    assert parsed is None
    assert raw["sunny"] is True
