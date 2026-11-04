"""The robot's per-session token (offer -> answer / ICE / bye echo), 2026-11-01: room broadcasts carry no sender,
so the gateway mints a token per session and only honours messages that echo it."""
import asyncio

from nori_sdk.mock.loopback import LoopbackSignaling
from nori_sdk.signaling import IcePayload, SdpPayload


def test_ice_wire_carries_the_token_only_when_set():
    assert IcePayload(candidate="c", sdp_mline_index=0).to_wire() == {"candidate": "c", "sdpMLineIndex": 0}
    assert IcePayload(candidate="c", sdp_mline_index=0, session="tok").to_wire()["session"] == "tok"
    assert IcePayload.from_wire({"candidate": "c", "sdpMLineIndex": 0, "session": "tok"}).session == "tok"
    assert IcePayload.from_wire({"candidate": "c", "sdpMLineIndex": 0, "session": 7}).session is None


def test_sdp_payload_defaults_to_no_token():
    assert SdpPayload(type="offer", sdp="v=0").session is None
    assert SdpPayload(type="answer", sdp="v=0", session="tok").session == "tok"


def test_loopback_bye_carries_the_token():
    a, b = LoopbackSignaling("operator"), LoopbackSignaling("robot")
    a.peer, b.peer = b, a
    a.send_bye("tok"); a.send_bye()
    assert [p for ev, p in a.sent if ev == "bye"] == [{"session": "tok"}, {}]


def test_supabase_transport_parses_and_sends_the_token():
    from nori_sdk import signaling_supabase as ss
    got = []

    class H:
        on_open = on_robot_here = on_ice = on_nack = on_state = staticmethod(lambda *a, **k: None)

        @staticmethod
        def on_sdp(p):
            got.append(p)

    t = ss.SupabaseSignaling.__new__(ss.SupabaseSignaling)
    t._handlers = H()
    t._call = lambda fn: asyncio.run(fn(t._handlers))   # dispatch() is a coroutine
    t._on_broadcast("sdp", {"type": "offer", "sdp": "v=0", "session": "tok-A"})
    t._on_broadcast("sdp", {"type": "offer", "sdp": "v=0"})
    assert [p.session for p in got] == ["tok-A", None]
    sent = []
    t._broadcast = lambda ev, p: sent.append((ev, p))
    t.send_sdp(SdpPayload(type="answer", sdp="v=0", session="tok-A")); t.send_sdp(SdpPayload(type="answer", sdp="v=0"))
    t.send_bye("tok-A"); t.send_bye()
    assert sent == [("sdp", {"type": "answer", "sdp": "v=0", "session": "tok-A"}), ("sdp", {"type": "answer", "sdp": "v=0"}),
                    ("bye", {"session": "tok-A"}), ("bye", {})]


def test_stop_sends_bye_with_the_offers_token():
    from nori_sdk.teleop import RemoteTeleop
    sent = []

    class Sig:
        def send_bye(self, session=None): sent.append(session)
        async def close(self): pass

    t = RemoteTeleop.__new__(RemoteTeleop)
    t._stopped = False; t._tasks = []; t._stream_queues = []; t._navigation_waiters = {}; t._navigation_goal_waiters = {}
    t._sensor_waiters = {}; t._pc = None; t._control = None; t._session_token = "tok-A"
    t._connected = asyncio.Event(); t._ready = asyncio.Event(); t._signaling = Sig()
    t._set_phase = lambda *a, **k: None

    async def stop_jog(): pass
    t.stop_jog = stop_jog
    asyncio.run(t.stop())
    assert sent == ["tok-A"]
