"""snapshot_raw: one full-resolution camera still over the control channel (robot verb `frame`), chunked, with
re-requests for what the unreliable channel dropped."""
import asyncio
import base64
import json

import pytest

from nori_sdk.mock.loopback import loopback_pair
from nori_sdk.teleop import RemoteTeleop, TeleopError


class Channel:
    readyState = "open"

    def __init__(self):
        self.sent = []

    def send(self, raw):
        self.sent.append(json.loads(raw))


def session():
    signaling, _ = loopback_pair()
    robot = RemoteTeleop(signaling)
    channel = Channel()
    robot._control = channel
    return robot, channel


def _chunks(rid, jpeg, size=10):
    b64 = base64.b64encode(jpeg).decode()
    parts = [b64[i:i + size] for i in range(0, len(b64), size)]
    return [{"type": "frame", "request_id": rid, "camera": "overhead", "ok": True, "i": i, "n": len(parts),
             "bytes": len(jpeg), "data": d} for i, d in enumerate(parts)]


@pytest.mark.asyncio
async def test_all_chunks_reassemble_to_the_bytes():
    robot, ch = session()
    jpeg = bytes(range(200))
    task = asyncio.ensure_future(robot.snapshot_raw("overhead", timeout=2.0))
    await asyncio.sleep(0)
    req = ch.sent[-1]; assert req["type"] == "frame" and req["camera"] == "overhead" and "want" not in req
    for c in reversed(_chunks(req["request_id"], jpeg)):                 # unordered delivery
        robot._handle_frame(json.dumps(c))
    assert await task == jpeg


@pytest.mark.asyncio
async def test_missing_chunks_are_re_requested_by_index_and_the_same_frame_completes():
    robot, ch = session()
    jpeg = bytes(range(256)) * 2
    task = asyncio.ensure_future(robot.snapshot_raw("overhead", timeout=2.4, rounds=4))
    await asyncio.sleep(0)
    rid = ch.sent[-1]["request_id"]
    cs = _chunks(rid, jpeg)
    for c in cs:
        if c["i"] not in (2, 5): robot._handle_frame(json.dumps(c))
    await asyncio.sleep(0.8)                                             # one round passes without 2 and 5
    again = [m for m in ch.sent if m.get("type") == "frame" and "want" in m]
    assert again and again[-1]["want"] == [2, 5]
    for c in cs:
        if c["i"] in (2, 5): robot._handle_frame(json.dumps(c))
    assert await task == jpeg


@pytest.mark.asyncio
async def test_refusal_and_timeout_are_named():
    robot, ch = session()
    task = asyncio.ensure_future(robot.snapshot_raw("overhead", timeout=1.0))
    await asyncio.sleep(0)
    rid = ch.sent[-1]["request_id"]
    robot._handle_frame(json.dumps({"type": "frame", "request_id": rid, "camera": "overhead", "ok": False, "error": "no frame for camera 'overhead'"}))
    with pytest.raises(TeleopError, match="refused"):
        await task
    robot2, _ = session()
    with pytest.raises(TeleopError, match="chunks within"):
        await robot2.snapshot_raw("overhead", timeout=0.6, rounds=1)
    assert robot2._frame_waiters == {}


@pytest.mark.asyncio
async def test_no_channel_is_a_named_error():
    signaling, _ = loopback_pair()
    robot = RemoteTeleop(signaling)
    with pytest.raises(TeleopError, match="no control channel"):
        await robot.snapshot_raw("overhead", timeout=0.2)
