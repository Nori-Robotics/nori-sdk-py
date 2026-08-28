"""A live-looking RemoteTeleop wired to a MockRobot: no WebRTC, no network, no credentials.

This exists so that developing against the mock is a SUPPORTED path rather than a private
one. The test suite drives the mock by assigning `teleop._control` and calling
`teleop._handle_frame` directly; that is fine inside this package and wrong to hand anybody
else, because those names carry no compatibility promise and would break silently on an
internal refactor.

The point is that a script written here runs unchanged against hardware. One line differs:

    async with mock_session() as robot:                  # development
    async with RemoteTeleop(SupabaseSignaling(...)) as robot:   # the real thing

Everything after it -- wait_ready, the descriptor, jog, action, telemetry, record, estop --
is the same object and the same protocol. What the mock CANNOT tell you is anything about the
transport: no ICE, no TURN, no bandwidth, no video track, and no real timing. A green mock
run means your logic is right, not that your network is.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from typing import Any

from ..teleop import RemoteTeleop
from .loopback import loopback_pair
from .robot import MockRobot

TELEMETRY_HZ = 15.0  # what the real gateway streams at


class _MockChannel:
    """Stands in for the RTCDataChannel the robot opens.

    Deliveries are scheduled on the event loop rather than made inline. A real data channel is
    never synchronously reentrant, and a client that accidentally depends on the reply landing
    before `send()` returns would pass here and deadlock -- or worse, silently reorder -- on
    hardware."""

    readyState = "open"

    def __init__(self, robot: MockRobot, teleop: RemoteTeleop) -> None:
        self._robot = robot
        self._teleop = teleop
        self.sent: list[dict[str, Any]] = []

    def send(self, raw: str) -> None:
        self.sent.append(json.loads(raw))
        replies = self._robot.handle(raw)
        loop = asyncio.get_running_loop()
        for reply in replies:
            loop.call_soon(self._teleop._handle_frame, reply)


@asynccontextmanager
async def mock_session(
    robot: MockRobot | None = None,
    *,
    telemetry_hz: float = TELEMETRY_HZ,
    strict: bool = False,
) -> AsyncIterator[RemoteTeleop]:
    """A connected RemoteTeleop backed by `robot` (a fresh MockRobot by default).

        async with mock_session() as robot:
            info = await robot.wait_ready()
            await robot.jog(JogBuilder(info.descriptor).base(linear=0.4).build(), duration=1)

    Pass a configured MockRobot to rehearse the awkward cases -- `MockRobot(online=False)` for
    a robot whose motion stack is down, `accepted=False` for a refused session,
    `descriptor=None` for a legacy robot that sends no descriptor at all, `cameras=False` for
    one that sends no camera layout, `action_outcome="clamped"` for a move that lands
    somewhere other than commanded.

    Telemetry is pumped in the background at `telemetry_hz`, integrating whatever jog is
    currently held, so `stream("telemetry")` behaves like the real thing. Set
    `telemetry_hz=0` to drive it yourself with `robot_double.telemetry(...)`."""
    bot = MockRobot() if robot is None else robot
    operator, _robot_side = loopback_pair()
    # strict flows through so a policy developed here rehearses the same raise-on-dead
    # behavior the harness runs with on hardware (see RemoteTeleop(strict=...)).
    teleop = RemoteTeleop(operator, strict=strict)

    # Deliberately NOT teleop.start(): that requires aiortc and would try to negotiate. We
    # assemble the post-negotiation state directly, which is the whole point of the double.
    teleop._loop = asyncio.get_running_loop()
    teleop._control = _MockChannel(bot, teleop)
    teleop._set_phase("connected")
    teleop._connected.set()

    for frame in bot.on_channel_open():
        teleop._handle_frame(frame)

    async def pump() -> None:
        interval = 1.0 / telemetry_hz
        while True:
            await asyncio.sleep(interval)
            bot.step(interval)
            for event in bot.drain_events():
                teleop._handle_frame(event)
            teleop._handle_frame(bot.telemetry())

    task = asyncio.create_task(pump()) if telemetry_hz > 0 else None
    try:
        yield teleop
    finally:
        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        await teleop.stop()


__all__ = ["TELEMETRY_HZ", "mock_session"]
