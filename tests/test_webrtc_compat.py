"""webrtcbin interop shims — pure-function tests (no aiortc, no network).

Each fixture shape below is taken from a REAL offer captured from an A3
gateway (GStreamer webrtcbin) on 2026-08-21 — the session where all three
incompatibilities were found on hardware.
"""

from nori_sdk.webrtc_compat import ensure_h264_fmtp, local_candidates

# The failing shape: H264 rtpmap with NO fmtp line (fresh-gateway offer).
OFFER_NO_FMTP = (
    "v=0\r\n"
    "o=- 1 1 IN IP4 0.0.0.0\r\n"
    "s=-\r\n"
    "t=0 0\r\n"
    "m=video 9 UDP/TLS/RTP/SAVPF 96 97\r\n"
    "a=rtpmap:96 H264/90000\r\n"
    "a=rtpmap:97 rtx/90000\r\n"
    "a=fmtp:97 apt=96\r\n"
    "m=application 0 UDP/DTLS/SCTP webrtc-datachannel"
)

# The healthy shape: fmtp present (warm-gateway offer) — must pass untouched.
OFFER_WITH_FMTP = (
    "v=0\r\n"
    "m=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
    "a=rtpmap:96 H264/90000\r\n"
    "a=fmtp:96 packetization-mode=1;profile-level-id=42c016"
)


def test_fmtp_added_when_missing() -> None:
    out = ensure_h264_fmtp(OFFER_NO_FMTP)
    lines = out.splitlines()
    i = lines.index("a=rtpmap:96 H264/90000")
    added = lines[i + 1]
    assert added.startswith("a=fmtp:96 ")
    assert "packetization-mode=1" in added
    # the rtx fmtp (apt=96) must not have suppressed the insertion — it is
    # payload 97's fmtp, not 96's... and yet 97 needs no new line:
    assert sum(ln.startswith("a=fmtp:97") for ln in lines) == 1


def test_fmtp_rtx_apt_does_not_mask_missing_h264_fmtp() -> None:
    # regression pin for the exact trap: `a=fmtp:97 apt=96` exists, H264 is
    # payload 96 — the check must key on the H264 payload type, not "any fmtp".
    out = ensure_h264_fmtp(OFFER_NO_FMTP)
    assert "a=fmtp:96 " in out


def test_fmtp_untouched_when_present() -> None:
    out = ensure_h264_fmtp(OFFER_WITH_FMTP)
    # exactly one fmtp for 96, the original one
    assert out.count("a=fmtp:96") == 1
    assert "42c016" in out


def test_local_candidates_dedupes_bundle_repeats() -> None:
    sdp = (
        "v=0\r\n"
        "m=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
        "a=candidate:1 1 udp 2130706431 192.168.68.78 52622 typ host\r\n"
        "a=candidate:2 1 udp 1694498815 76.237.102.143 52622 typ srflx\r\n"
        "m=application 9 UDP/DTLS/SCTP webrtc-datachannel\r\n"
        "a=candidate:1 1 udp 2130706431 192.168.68.78 52622 typ host\r\n"
        "a=candidate:2 1 udp 1694498815 76.237.102.143 52622 typ srflx"
    )
    pairs = local_candidates(sdp)
    # BUNDLE repeats the same candidates per m-section; trickle each once,
    # attributed to the first m-line they appear under.
    assert len(pairs) == 2
    assert all(m == 0 for m, _ in pairs)
    assert pairs[0][1].startswith("candidate:1 ")
    assert pairs[1][1].startswith("candidate:2 ")


def test_local_candidates_empty_sdp() -> None:
    assert local_candidates("v=0\r\ns=-") == []


def test_dtls_cipher_list_is_forward_secret_only() -> None:
    # Every suite must be ECDHE (forward secrecy); plain-RSA key exchange is
    # deliberately absent — Chrome<->webrtcbin interop proves the GStreamer
    # side does ECDHE-RSA, so non-PFS fallbacks would only weaken sessions.
    from nori_sdk.webrtc_compat import _DTLS_CIPHERS

    for suite in _DTLS_CIPHERS.decode().split(":"):
        assert suite.startswith("ECDHE-"), suite


def test_widen_dtls_ciphers_patches_and_is_idempotent(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    import pytest

    pytest.importorskip("aiortc")
    from aiortc.rtcdtlstransport import RTCCertificate

    from nori_sdk import webrtc_compat

    monkeypatch.setattr(webrtc_compat, "_ciphers_widened", False)
    before = RTCCertificate._create_ssl_context
    try:
        assert webrtc_compat.widen_dtls_ciphers() is True
        assert RTCCertificate._create_ssl_context is not before
        patched = RTCCertificate._create_ssl_context
        # idempotent: a second call must not stack another wrapper
        assert webrtc_compat.widen_dtls_ciphers() is True
        assert RTCCertificate._create_ssl_context is patched
    finally:
        RTCCertificate._create_ssl_context = before  # type: ignore[method-assign]
        webrtc_compat._ciphers_widened = False


def test_widen_dtls_ciphers_degrades_without_the_seam(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    # If a future aiortc moves _create_ssl_context, the patch must degrade to
    # a warning + unpatched behavior, never break session start.
    import pytest

    pytest.importorskip("aiortc")
    from aiortc.rtcdtlstransport import RTCCertificate

    from nori_sdk import webrtc_compat

    monkeypatch.setattr(webrtc_compat, "_ciphers_widened", False)
    monkeypatch.delattr(RTCCertificate, "_create_ssl_context")
    assert webrtc_compat.widen_dtls_ciphers() is False
