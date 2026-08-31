"""Supabase Realtime broadcast signaling, operator side — no supabase SDK.

Speaks just enough of the Phoenix websocket protocol that Supabase Realtime uses to join one
broadcast channel and exchange `sdp`/`ice` with a robot. The robot's gateway
(nori_gateway/signaling.py) and the browser's supabase-js client join the SAME channel with
the SAME payloads; this is the third implementation of that one wire format, ported from the
gateway so its hard-won behaviors survive:

  * BLOCKING recv on a dedicated thread — a socket read timeout kills an idle link in
    seconds, so the heartbeat keeps it alive instead.
  * AUTO-RECONNECT with exponential backoff AND jitter. A script may sit waiting for a robot
    that is still booting; failures must not hammer DNS, and a fleet that lost its sockets
    together must not rejoin in lockstep.
  * The device/user JWT rides IN the phx_join payload, not only in the post-join push:
    Realtime evaluates the private-channel RLS AT JOIN, so a tokenless join is denied even
    for a legitimate identity.

Threading: the websocket loop runs on its own thread; every handler is marshalled back onto
the asyncio loop that called connect(), so user callbacks always run where they expect to.

Requires `websocket-client` (pip install "nori-sdk[supabase]").
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import random
import ssl
import threading
import time
from collections.abc import Callable
from typing import Any

from .signaling import (
    IcePayload,
    NackPayload,
    ReadyTurn,
    SdpPayload,
    SignalingHandlers,
    SignalingTransport,
    dispatch,
)

log = logging.getLogger("nori_sdk.signaling")

try:  # pragma: no cover - import guard
    import websocket  # websocket-client
except ImportError:  # pragma: no cover
    # Sentinel for "the supabase extra is not installed"; SupabaseSignaling.__init__ turns it
    # into an ImportError naming the install command. mypy sees the name as a Module once the
    # missing-stub override applies, so the rebind needs the ignore.
    websocket = None  # type: ignore[assignment]


class SupabaseSignaling(SignalingTransport):
    HEARTBEAT_S = 8  # under Supabase's idle timeout
    BACKOFF_MAX_S = float(os.environ.get("NORI_SIG_BACKOFF_MAX_S", "180"))

    def __init__(
        self,
        supabase_url: str,
        anon_key: str,
        room: str,
        *,
        token_provider: Callable[[], str] | None = None,
        debug: bool = False,
    ) -> None:
        """`room` is the robot's serial (e.g. "NORI-A3-0001") — the channel name both ends
        agree on.

        `token_provider` returns a Supabase JWT (see auth.UserAuth). Its PRESENCE is what
        flips the join to `private: true`: RLS then admits only the robot and the customer
        paired to it. Without one you join a public channel, which is bench-only — the
        fleet's robots join private rooms and will never see you."""
        if websocket is None:
            raise ImportError(
                'this transport needs the "supabase" extra: '
                'pip install "nori-sdk[supabase]" (websocket-client)'
            )
        self._base = (supabase_url or "").rstrip("/")
        self._anon = anon_key or ""
        self.room = room
        self._topic = f"realtime:{room}"
        self._token_provider = token_provider
        self._debug = debug

        self._handlers: SignalingHandlers | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws: Any = None
        self._ref = 0
        self._join_ref: str | None = None
        self._last_pushed_token: str | None = None
        self._reflock = threading.Lock()  # separate from _sendlock: _next_ref is called
        self._sendlock = threading.Lock()  # INSIDE `with _sendlock`
        self._stop = threading.Event()
        self._joined = threading.Event()
        self._thread: threading.Thread | None = None

    # --- SignalingTransport ----------------------------------------------------------------

    async def connect(self, handlers: SignalingHandlers) -> None:
        self._handlers = handlers
        self._loop = asyncio.get_running_loop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._manage, daemon=True, name="nori-signaling")
        self._thread.start()

    def send_ready(self, turn: ReadyTurn | None = None) -> None:
        payload: dict[str, Any] = {}
        if turn is not None and turn.urls:
            payload["turn"] = turn.to_wire()
        self._broadcast("ready", payload)

    def send_sdp(self, payload: SdpPayload) -> None:
        self._broadcast("sdp", {"type": payload.type, "sdp": payload.sdp})

    def send_ice(self, payload: IcePayload) -> None:
        self._broadcast("ice", payload.to_wire())

    def send_bye(self) -> None:
        try:
            self._broadcast("bye", {})
        except Exception:
            pass  # contract: must not raise

    async def close(self) -> None:
        self._stop.set()
        try:
            if self._ws is not None:
                self._ws.close()
        except Exception:
            pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            await asyncio.get_running_loop().run_in_executor(None, thread.join, 3.0)
        self._thread = None
        self._notify_state("closed")

    # --- websocket plumbing ----------------------------------------------------------------

    def _url(self) -> str:
        base = self._base.replace("https://", "wss://").replace("http://", "ws://").rstrip("/")
        return f"{base}/realtime/v1/websocket?apikey={self._anon}&vsn=1.0.0"

    def _next_ref(self) -> str:
        with self._reflock:  # NOT _sendlock — the heartbeat would self-deadlock
            self._ref += 1
            return str(self._ref)

    def _send_raw(self, message: dict[str, Any]) -> None:
        data = json.dumps(message)
        if self._debug:
            log.debug("-> %s", data[:200])
        try:
            with self._sendlock:
                if self._ws is not None:
                    self._ws.send(data)
        except Exception as e:
            log.warning("send failed: %s", e)

    def _broadcast(self, event: str, payload: dict[str, Any]) -> None:
        self._send_raw(
            {
                "topic": self._topic,
                "event": "broadcast",
                "payload": {"type": "broadcast", "event": event, "payload": payload},
                "ref": self._next_ref(),
                "join_ref": self._join_ref,
            }
        )

    def _current_token(self) -> str | None:
        """The channel access_token: our JWT in private mode, else the anon key. None when a
        private-mode provider fails — the caller skips rather than tearing the process down,
        and the reconnect backoff retries."""
        if self._token_provider is None:
            return self._anon
        try:
            return self._token_provider()
        except Exception as e:
            log.warning("token unavailable (private room): %s", e)
            return None

    def _manage(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            joined = self._session()
            if self._stop.is_set():
                break
            if joined:
                # We HAD joined (server closed it / network blip): retry promptly, but
                # spread 1-3 s so a fleet-wide drop isn't a lockstep rejoin.
                backoff = 1.0
                wait = random.uniform(1.0, 3.0)
            else:
                # Never joined: exponential with equal jitter — half fixed, half random, so
                # the worst case stays bounded AND clients stay decorrelated.
                backoff = min(backoff * 2, self.BACKOFF_MAX_S)
                wait = backoff / 2 + random.uniform(0, backoff / 2)
            log.info("reconnecting in %.1fs", wait)
            self._stop.wait(wait)

    def _session(self) -> bool:
        """One connect+join+recv lifetime. True if it joined at least once."""
        joined_ok = False
        try:
            # enable_multithread so the heartbeat thread can send while this thread blocks
            # in recv. A recv timeout proved unreliable on the SSL socket, so recv BLOCKS
            # and the heartbeat is what keeps the link alive.
            ws = websocket.create_connection(
                self._url(),
                sslopt={"cert_reqs": ssl.CERT_REQUIRED},
                timeout=10,
                enable_multithread=True,
            )
        except Exception as e:
            log.warning("connect failed: %s", e)
            self._notify_state("error")
            return False
        log.info("connected room=%s", self.room)
        ws.settimeout(None)
        sendlock = threading.Lock()  # PER-SESSION: a stuck old send can't block a new one
        self._ws = ws
        self._sendlock = sendlock
        self._joined.clear()
        self._join_ref = self._next_ref()

        join_token = self._current_token()
        if self._token_provider is not None and join_token is None:
            log.warning("token unavailable - deferring join (will back off)")
            try:
                ws.close()
            except Exception:
                pass
            return False
        join_payload: dict[str, Any] = {
            "config": {
                "broadcast": {"self": False, "ack": False},
                "presence": {"key": ""},
                "private": bool(self._token_provider),
            }
        }
        if self._token_provider is not None:
            join_payload["access_token"] = join_token
            self._last_pushed_token = join_token
        self._send_raw(
            {
                "topic": self._topic,
                "event": "phx_join",
                "payload": join_payload,
                "ref": self._join_ref,
                "join_ref": self._join_ref,
            }
        )

        stop_hb = threading.Event()
        threading.Thread(
            target=self._heartbeat, args=(ws, sendlock, stop_hb), daemon=True
        ).start()
        started = time.monotonic()
        try:
            while not self._stop.is_set():
                raw = ws.recv()
                if not raw:  # "" / None => the server closed the socket
                    log.info("socket closed by server after %.1fs", time.monotonic() - started)
                    break
                if self._debug:
                    log.debug("<- %s", str(raw)[:200])
                self._handle(raw)
                if not joined_ok and self._joined.is_set():
                    joined_ok = True
        except Exception as e:
            if not self._stop.is_set():
                log.warning("recv ended after %.1fs: %s", time.monotonic() - started, e)
        finally:
            stop_hb.set()
            self._joined.clear()
            try:
                ws.close()
            except Exception:
                pass
        if not joined_ok:
            self._notify_state("error")
        return joined_ok

    def _heartbeat(self, ws: Any, sendlock: threading.Lock, stop_hb: threading.Event) -> None:
        # Uses the PER-SESSION lock/socket (not self._*) so a send blocking on a dead socket
        # can't freeze the next session. On failure, close the socket to unblock recv() so
        # the session ends and _manage reconnects.
        interval = 0.5  # first heartbeat almost immediately
        while not stop_hb.wait(interval):
            interval = self.HEARTBEAT_S
            if self._stop.is_set():
                return
            try:
                with sendlock:
                    ws.send(
                        json.dumps(
                            {
                                "topic": "phoenix",
                                "event": "heartbeat",
                                "payload": {},
                                "ref": self._next_ref(),
                            }
                        )
                    )
            except Exception as e:
                log.warning("heartbeat failed -> reconnecting: %s", e)
                try:
                    ws.close()
                except Exception:
                    pass
                return
            # Keep the channel's access_token current: the provider refreshes near expiry,
            # and Realtime closes an idle channel when the ~1 h JWT lapses. Gated on a
            # confirmed join — the post-join push is the intended first token, this only
            # REFRESHES it. A fetch failure is non-fatal; try again next tick.
            if self._token_provider is not None and self._joined.is_set():
                token = self._current_token()
                if token and token != self._last_pushed_token:
                    try:
                        with sendlock:
                            ws.send(
                                json.dumps(
                                    {
                                        "topic": self._topic,
                                        "event": "access_token",
                                        "payload": {"access_token": token},
                                        "ref": self._next_ref(),
                                        "join_ref": self._join_ref,
                                    }
                                )
                            )
                        self._last_pushed_token = token
                    except Exception as e:
                        log.warning("access_token refresh failed: %s", e)
                        try:
                            ws.close()
                        except Exception:
                            pass
                        return

    def _handle(self, raw: str | bytes) -> None:
        # `ws.recv()` is typed `str | bytes` (a binary frame is legal websocket, even though
        # Phoenix only ever sends text), and json.loads takes either. Widening the annotation
        # is the honest fix -- `raw` is never used as a str beyond this parse.
        try:
            message = json.loads(raw)
        except Exception:
            return
        event = message.get("event")
        if event == "phx_reply" and message.get("ref") == self._join_ref:
            payload = message.get("payload") or {}
            if payload.get("status") == "ok":
                self._joined.set()
                log.info("channel joined")
                token = self._current_token()
                if token is not None:
                    self._send_raw(
                        {
                            "topic": self._topic,
                            "event": "access_token",
                            "payload": {"access_token": token},
                            "ref": self._next_ref(),
                            "join_ref": self._join_ref,
                        }
                    )
                    self._last_pushed_token = token
                self._notify_state("open")
                self._call(lambda h: dispatch(h.on_open))
            else:
                log.error("JOIN FAILED: %s", payload)
                if self._token_provider is not None:
                    log.error(
                        "  hint: private room denied - is this account paired with %s? "
                        "(RLS admits only the device and its paired customer)",
                        self.room,
                    )
                self._notify_state("error")
        elif event == "broadcast":
            payload = message.get("payload") or {}
            self._on_broadcast(payload.get("event"), payload.get("payload") or {})
        elif event == "phx_error":
            log.warning("phx_error: %s", message.get("payload"))
            self._notify_state("error")

    def _on_broadcast(self, event: Any, payload: dict[str, Any]) -> None:
        if event == "sdp":
            kind = payload.get("type")
            sdp = payload.get("sdp")
            if kind in ("offer", "answer") and isinstance(sdp, str):
                self._call(lambda h: dispatch(h.on_sdp, SdpPayload(type=kind, sdp=sdp)))
        elif event == "ice":
            ice = IcePayload.from_wire(payload)
            if ice is not None:
                self._call(lambda h: dispatch(h.on_ice, ice))
        elif event == "robot_here":
            self._call(lambda h: dispatch(h.on_robot_here, payload))
        elif event == "nack":
            reason = payload.get("reason")
            self._call(
                lambda h: dispatch(
                    h.on_nack, NackPayload(reason=reason if isinstance(reason, str) else None)
                )
            )

    # --- thread -> event loop --------------------------------------------------------------

    def _call(self, make: Callable[[SignalingHandlers], Any]) -> None:
        """Run a handler coroutine on the caller's event loop. Everything inbound arrives on
        the websocket thread, so this is the ONLY way user code should be reached."""
        handlers, loop = self._handlers, self._loop
        if handlers is None or loop is None or loop.is_closed():
            return
        try:
            asyncio.run_coroutine_threadsafe(_wrap(make(handlers)), loop)
        except RuntimeError:
            pass  # loop shut down mid-flight

    def _notify_state(self, state: str) -> None:
        handlers = self._handlers
        if handlers is not None and handlers.on_state is not None:
            self._call(lambda h: dispatch(h.on_state, state))


async def _wrap(awaitable: Any) -> None:
    try:
        await awaitable
    except Exception:
        log.exception("signaling handler raised")


__all__ = ["SupabaseSignaling"]
