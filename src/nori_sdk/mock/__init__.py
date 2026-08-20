"""Test doubles: a robot that speaks nori-protocol without hardware, and a loopback
signaling transport.

The TypeScript SDK's mock robot is the declared SPEC for the real robot gateway — the
gateway's own tests are written against it. This package is the Python peer of that idea, so
a client can be exercised end-to-end in CI with no robot, no network and no WebRTC stack.

Keep the frame ORDER here honest: the real gateway emits ack -> camera_layout ->
daemon_status on channel open, and a client that only works against a mock with a different
order is a client that breaks on hardware.
"""

from .loopback import LoopbackSignaling, loopback_pair
from .robot import MockRobot
from .session import mock_session

__all__ = ["LoopbackSignaling", "MockRobot", "loopback_pair", "mock_session"]
