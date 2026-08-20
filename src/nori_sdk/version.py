"""The nori-protocol version this SDK's wire vocabulary targets.

Mirrors @nori/sdk's src/version.ts, deliberately: the TypeScript and Python SDKs are two
implementations of ONE protocol, and they must agree on which version of it they speak.

Compat policy: nori-sdk targeting version N is compatible with a robot speaking
nori-protocol version N. A mismatch is ADVISORY on the client side — the robot's `ack`
carries its own `protocol_version`, we surface the difference on RobotInfo.version_mismatch
and keep going, because mixed-version fleets are normal and unknown frames are ignored by
both ends. When the protocol makes a breaking change, this integer bumps in lockstep with a
new SDK major, in both languages.
"""

NORI_PROTOCOL_VERSION = 1

__all__ = ["NORI_PROTOCOL_VERSION"]
