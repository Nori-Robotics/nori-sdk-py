"""aiortc <-> GStreamer webrtcbin interop shims.

The robot's gateway builds its peer with GStreamer's webrtcbin; this SDK
answers with aiortc. Three incompatibilities were found (and root-caused)
during the first real-hardware handshake against an A3 gateway, 2026-08-21.
Each function here fixes exactly one, and each is a no-op when the peer is
already well-behaved — so none of this can break a session with a future,
stricter robot build.

1. `ensure_h264_fmtp` — webrtcbin's offer sometimes omits the `a=fmtp`
   line for H264 (observed on a freshly started gateway before its encoder
   pipeline warms). aiortc then defaults the missing `packetization-mode`
   to 0, supports only mode 1, finds zero common video codecs, and
   setRemoteDescription raises. Browsers are lenient about the missing
   fmtp; aiortc is strict. The robot's encoder genuinely emits
   packetization-mode=1 (confirmed from a warm-gateway offer), so stating
   it on the model's behalf is honest, not hopeful.

2. `local_candidates` — aiortc finishes ICE gathering inside
   setLocalDescription and embeds its candidates in the answer SDP, so this
   SDK originally trickled nothing outbound. But webrtcbin only consumes
   candidates delivered through the signaling `ice` event
   (`add-ice-candidate`); the ones inline in the answer are ignored. Result:
   the robot never learned a single operator candidate and ICE failed every
   time. The session must trickle the answer's candidates explicitly.

3. `widen_dtls_ciphers` — aiortc's DTLS cipher list is ECDHE-ECDSA-only
   (its own certificate is ECDSA). GStreamer's dtls element generates an
   RSA certificate, so the robot-side server cannot select any ECDSA-auth
   suite and the handshake dies instantly in a handshake_failure alert
   (surfacing as an empty `SSL.Error`). Browsers offer both cipher
   families, which is why the TS app never hit this. Widening the client
   list with ECDHE-RSA suites keeps every ECDSA suite first, so nothing
   changes against an ECDSA robot.
"""

from __future__ import annotations

import re
from typing import Any

__all__ = ["ensure_h264_fmtp", "local_candidates", "widen_dtls_ciphers"]

# Every H264 fmtp the gateway has been observed to send uses these values;
# they are also the profile aiortc negotiates against browsers.
_H264_FMTP = "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f"

# aiortc's four default ECDSA suites, in aiortc's own order, then their RSA
# twins, then plain-RSA fallbacks for a server that cannot do ECDHE at all.
_DTLS_CIPHERS = (
    b"ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:"
    b"ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:"
    b"ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES128-SHA:"
    b"ECDHE-ECDSA-AES256-SHA:ECDHE-RSA-AES256-SHA:"
    b"AES128-GCM-SHA256:AES128-SHA:AES256-SHA"
)

_RTPMAP_H264 = re.compile(r"a=rtpmap:(\d+) H264/90000")
_FMTP = re.compile(r"a=fmtp:(\d+)")

_ciphers_widened = False


def ensure_h264_fmtp(sdp: str) -> str:
    """Return `sdp` with an `a=fmtp` line added after any H264 rtpmap that lacks one.

    Pure string transform; no aiortc import. An offer that already carries
    fmtp for every H264 payload type is returned byte-identical.
    """
    lines = sdp.splitlines()
    have = {m.group(1) for ln in lines if (m := _FMTP.match(ln))}
    out: list[str] = []
    for ln in lines:
        out.append(ln)
        m = _RTPMAP_H264.match(ln)
        if m and m.group(1) not in have:
            out.append(f"a=fmtp:{m.group(1)} {_H264_FMTP}")
    return "\r\n".join(out) + "\r\n"


def local_candidates(sdp: str) -> list[tuple[int, str]]:
    """Extract `(mline_index, candidate)` pairs from a local SDP for trickling.

    The candidate string is returned without the leading ``a=`` (i.e. in the
    ``candidate:...`` wire form `IcePayload` expects). Duplicate candidate
    lines (BUNDLE repeats the same set on every m-section) are emitted once,
    with the first m-line index they appear under.
    """
    pairs: list[tuple[int, str]] = []
    seen: set[str] = set()
    mline = -1
    for ln in sdp.splitlines():
        if ln.startswith("m="):
            mline += 1
        elif ln.startswith("a=candidate:") and ln not in seen:
            seen.add(ln)
            pairs.append((mline, ln[2:]))
    return pairs


def widen_dtls_ciphers() -> None:
    """Widen aiortc's DTLS cipher list with RSA-auth suites. Idempotent.

    Patches `RTCCertificate._create_ssl_context` so every context built for
    a session offers both ECDSA- and RSA-auth suites. Called by
    `RemoteTeleop` before building a peer; safe to call repeatedly and safe
    when the `webrtc` extra is absent (import happens here, lazily).
    """
    global _ciphers_widened
    if _ciphers_widened:
        return
    from aiortc.rtcdtlstransport import RTCCertificate

    original = RTCCertificate._create_ssl_context

    def _widened(self: Any, srtp_profiles: Any) -> Any:
        ctx = original(self, srtp_profiles)
        ctx.set_cipher_list(_DTLS_CIPHERS)
        return ctx

    RTCCertificate._create_ssl_context = _widened  # type: ignore[method-assign]
    _ciphers_widened = True
