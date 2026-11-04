"""The V1 robot-session grant rides in `ready`, and a fixed DTLS certificate lets a grant be bound to the
connection's fingerprint before the peer connection exists (what the browser app does; the gateway's
session_grant.verify compares the `fp` claim with the SDP answer's fingerprint)."""
import re

import pytest

from nori_sdk import RemoteTeleop
from nori_sdk.mock.loopback import loopback_pair


def _readies(side):
    return [p for kind, p in side.sent if kind == "ready"]


@pytest.mark.asyncio
async def test_ready_carries_the_grant_from_a_string_or_a_provider():
    operator, _robot_side = loopback_pair()
    teleop = RemoteTeleop(operator, grant="grant-1")
    teleop._send_ready()
    assert _readies(operator)[-1]["grant"] == "grant-1"

    calls = []
    def provider():
        calls.append(1); return f"grant-{len(calls)}"
    operator2, _ = loopback_pair()
    teleop2 = RemoteTeleop(operator2, grant=provider)
    teleop2._send_ready(); teleop2._send_ready()
    assert [p["grant"] for p in _readies(operator2)] == ["grant-1", "grant-2"]   # re-asked at every ready

    operator3, _ = loopback_pair()
    RemoteTeleop(operator3)._send_ready()
    assert _readies(operator3)[-1]["grant"] is None                              # unchanged default: no grant


def test_generate_certificate_gives_a_bindable_fingerprint():
    pytest.importorskip("aiortc")
    cert, fp = RemoteTeleop.generate_certificate()
    assert re.fullmatch(r"sha-256:[0-9a-f]{64}", fp), fp
    assert cert.getFingerprints()[0].value.replace(":", "").lower() == fp.split(":", 1)[1]


@pytest.mark.asyncio
async def test_the_peer_connection_uses_the_fixed_certificate():
    pytest.importorskip("aiortc")
    cert, fp = RemoteTeleop.generate_certificate()
    operator, _ = loopback_pair()
    teleop = RemoteTeleop(operator, certificate=cert)
    await teleop._fresh_peer()
    try:
        assert teleop._pc._RTCPeerConnection__certificates == [cert]
        # the fingerprint the robot will see in our SDP is the one the grant was bound to
        assert any(f.value.replace(":", "").lower() == fp.split(":", 1)[1]
                   for c in teleop._pc._RTCPeerConnection__certificates for f in c.getFingerprints())
    finally:
        await teleop._pc.close()
