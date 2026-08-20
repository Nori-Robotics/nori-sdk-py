"""Supabase Auth token providers — zero dependencies, urllib only.

A private signaling room is gated by Supabase RLS, which admits exactly two identities: the
robot (its device user) and the customer paired to it. So a token provider here answers the
question "who is this client?" and nothing else — it is not an authorization system, it just
presents a JWT the signaling transport pushes to Realtime.

Two flavors, same interface (`.token()` returning a valid JWT, cached and auto-refreshed):

  UserAuth   — a human operator's Supabase account. This is the normal one for a Python
               client: a script driving a robot acts as the customer who owns it.
  DeviceAuth — a ROBOT's own identity, from provisioning. Only useful for on-robot tools;
               a laptop should not be holding device credentials.

Ported from the robot's nori_identity.device_auth (same refresh/skew logic, same tolerance
for transient failures) so the two codebases behave identically under an auth outage.
"""

from __future__ import annotations

import base64
import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

# Hand a token back only if it has more than this left before `exp`, so a channel never gets
# a JWT that expires mid-use.
_REFRESH_SKEW_S = 600
_TIMEOUT_S = 10
_DEFAULT_TTL_S = 3600  # Supabase JWTs are 1 h; used when `exp` is unparseable
# Floor on how long a freshly granted token is served from cache, whatever its
# `exp` claims. Guards against a grant storm when the local clock runs ahead of the auth
# server: see _grant(). Mirrors nori_identity/device_auth.py and docs/known_issues.md #18.
_MIN_CACHE_S = 30


class AuthError(RuntimeError):
    """A token grant failed (network, or a 4xx). Carries the server's error body.

    Callers treat this like any other connect failure — the signaling backoff loop retries —
    so a transient auth outage is self-healing rather than fatal.

    `status` is the HTTP status when the server answered and refused us, and None when we
    never got an answer (DNS/TCP/TLS failure, or a non-JSON body). Only a 4xx means the
    credentials themselves were rejected, and only then is a refresh token worth discarding.
    """

    def __init__(self, message: str, status: int | None = None) -> None:
        super().__init__(message)
        self.status = status


class _SupabaseTokenSource:
    """Shared password-grant + refresh machinery. Subclasses only supply credentials."""

    def __init__(self, supabase_url: str, anon_key: str, email: str, password: str) -> None:
        self._base = (supabase_url or "").rstrip("/")
        self._anon = anon_key or ""
        self._email = email or ""
        self._password = password or ""
        self._lock = threading.Lock()  # token() is called from the signaling threads
        self._access = ""
        self._refresh = ""
        self._exp = 0.0  # wall-clock exp, for TTL maths and logs only
        # Refresh deadline on the MONOTONIC clock: a machine can step its wall clock by
        # hours (NTP settling after boot, sleep/wake), which breaks a wall-clock deadline
        # in both directions — forward looks permanently overdue, backward pins an expired
        # token forever. Monotonic is immune to both.
        self._refresh_after_mono = 0.0

    # --- public ---------------------------------------------------------------------------
    def token(self) -> str:
        """A valid access JWT, (re)fetching when missing or near expiry.

        Cheap on the hot path: no network call while the cached token is comfortably
        unexpired. Serialized so concurrent callers never double-grant."""
        with self._lock:
            if self._access and time.monotonic() < self._refresh_after_mono:
                return self._access
            # Prefer a refresh (no password on the wire) when we hold one; fall back to a
            # password grant if it is missing or has been revoked/rotated.
            if self._refresh:
                try:
                    self._grant("refresh_token", {"refresh_token": self._refresh})
                    return self._access
                except AuthError as e:
                    # Only a 4xx means the refresh token was actually refused; dropping it
                    # on a network blip would downgrade this process to password grants —
                    # i.e. the secret on the wire — for the rest of its life.
                    if e.status is not None and 400 <= e.status < 500:
                        self._refresh = ""
            self._grant("password", {"email": self._email, "password": self._password})
            return self._access

    def require(self) -> None:
        """Fail fast at startup when credentials are incomplete, instead of letting the
        client look like an unreachable robot after an RLS denial."""
        missing = [
            name
            for name, value in (
                ("supabase_url", self._base),
                ("supabase_anon_key", self._anon),
                ("email", self._email),
                ("password", self._password),
            )
            if not value
        ]
        if missing:
            raise AuthError("auth is missing: " + ", ".join(missing))

    # --- internals ------------------------------------------------------------------------
    def _grant(self, grant_type: str, body: dict[str, str]) -> None:
        data = self._post(f"/auth/v1/token?grant_type={grant_type}", body)
        access = data.get("access_token", "")
        if not access:
            # Keys only: a partial grant response can still carry a refresh_token, and
            # this text is logged at WARNING by the signaling layer.
            raise AuthError(
                "grant returned no access_token "
                f"(response keys: {sorted(data)})")
        self._access = access
        # Both grant types return a refresh token; keep the old one if a response omits it,
        # so we never lose the ability to refresh.
        self._refresh = data.get("refresh_token") or self._refresh
        self._exp = _jwt_exp(access, data.get("expires_in"))
        # Refresh at exp - skew, but never before the token's own half-life: a short token
        # (TTL <= 2*skew) would otherwise sit permanently inside the skew window and force a
        # fresh grant on EVERY call — a grant storm.
        # Two readings of this token's life; trust the shorter. `ttl` comes from the JWT's
        # `exp` against OUR clock, so a wrong clock distorts it (behind inflates it). `srv`
        # is the server's `expires_in`, a DURATION, so no clock error touches it. Both carry
        # the same refresh lead, so whichever wins we re-grant BEFORE expiry.
        ttl = max(0.0, self._exp - time.time())
        srv = _server_ttl(data.get("expires_in"))
        hold_s = min(ttl - _refresh_lead(ttl), srv - _refresh_lead(srv))
        # The floor is the OUTERMOST bound: its job is to cap the grant RATE, so nothing --
        # not a skewed clock, not a server issuing 1-second tokens -- may push us under it.
        # Applied before the cap it stopped being a floor and the server set our grant rate.
        # See nori_ws docs/known_issues.md #18.
        hold_s = max(_MIN_CACHE_S, hold_s)
        self._refresh_after_mono = time.monotonic() + hold_s

    def _post(self, path: str, body: dict[str, str]) -> dict[str, Any]:
        req = urllib.request.Request(
            self._base + path,
            data=json.dumps(body).encode(),
            headers={"apikey": self._anon, "Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT_S) as resp:
                result = json.loads(resp.read().decode())
                return result if isinstance(result, dict) else {}
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode()
            except Exception:
                pass
            raise AuthError(f"auth {e.code} on {path}: {detail}", status=e.code) from e
        except (urllib.error.URLError, OSError) as e:
            raise AuthError(f"auth unreachable on {path}: {e}") from e
        except ValueError as e:
            raise AuthError(f"auth sent non-JSON on {path}: {e}") from e


class UserAuth(_SupabaseTokenSource):
    """A human operator's Supabase session. The identity a Python script normally runs as.

        auth = UserAuth(SUPABASE_URL, ANON_KEY, "me@example.com", "hunter2")
        sig = SupabaseSignaling(SUPABASE_URL, ANON_KEY, room=serial, token_provider=auth.token)

    Nothing is persisted: on process restart we simply grant again."""

    @classmethod
    def from_env(cls, prefix: str = "NORI") -> UserAuth | None:
        """Build from environment: <prefix>_SUPABASE_URL, _SUPABASE_ANON_KEY, _EMAIL,
        _PASSWORD. Returns None when incomplete, so a caller can fall back to a public room
        or prompt interactively."""
        url = os.environ.get(f"{prefix}_SUPABASE_URL", "")
        anon = os.environ.get(f"{prefix}_SUPABASE_ANON_KEY", "")
        email = os.environ.get(f"{prefix}_EMAIL", "")
        password = os.environ.get(f"{prefix}_PASSWORD", "")
        if not (url and email and password):
            return None
        return cls(url, anon, email, password)


class DeviceAuth(_SupabaseTokenSource):
    """A ROBOT's identity, minted at provisioning. Wire-identical to the robot's
    nori_identity.device_auth — this exists so on-robot Python tools can reuse this SDK
    rather than carrying their own copy.

    The device secret IS the device user's password; unlike a service-role key it is safe on
    the robot because it authenticates only that one device, whose reach is bounded by RLS.
    It has no business on a laptop."""

    DEFAULT_CREDENTIALS_FILE = Path("/etc/nori/device.env")

    @classmethod
    def from_file(cls, path: Path | str | None = None) -> DeviceAuth | None:
        """Build from the standard robot credential file, or None if it is incomplete."""
        env = read_env_file(Path(path) if path else cls.DEFAULT_CREDENTIALS_FILE)
        email = env.get("NORI_DEVICE_EMAIL", "")
        secret = env.get("NORI_DEVICE_SECRET", "")
        url = env.get("SUPABASE_URL", "")
        anon = env.get("SUPABASE_ANON_KEY", "")
        if not (email and secret and url):
            return None
        return cls(url, anon, email, secret)


def read_env_file(path: Path) -> dict[str, str]:
    """Parse a KEY=VALUE file (the provisioning device.env format). A missing file yields {}
    — the caller decides whether the credentials were required."""
    out: dict[str, str] = {}
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        out[key.strip()] = value.strip().strip('"').strip("'")
    return out


def _refresh_lead(lifetime: float) -> float:
    """How far BEFORE expiry to re-grant: the skew, clamped to half the lifetime so a short
    token is still served for its first half rather than re-granted on every call."""
    return min(_REFRESH_SKEW_S, lifetime / 2)


def _server_ttl(expires_in: Any) -> float:
    """The token's lifetime as the SERVER stated it, in seconds. A duration, so it is immune
    to our clock being wrong -- which is why it is the cap in _grant.

    Absent/unparseable means the server told us NOTHING -> fall back to the default rather
    than inventing a tighter bound. Zero or negative means the server told us the token is
    ALREADY DEAD -> return 0 so only the _MIN_CACHE_S floor remains; falling back to the
    default there would cache a token the server just declared expired for a full hour."""
    try:
        value = float(expires_in)
    except (TypeError, ValueError):
        return float(_DEFAULT_TTL_S)
    return value if value > 0 else 0.0


def _jwt_exp(token: str, fallback_ttl: Any = None) -> float:
    """`exp` (unix seconds) from a JWT WITHOUT verifying it — we made the request, we only
    need to know when to refresh. Falls back to now+expires_in, then now+1 h."""
    try:
        seg = token.split(".")[1]
        seg += "=" * (-len(seg) % 4)  # restore base64url padding
        payload = json.loads(base64.urlsafe_b64decode(seg).decode())
        exp = float(payload["exp"])
        if exp > 0:
            return exp
    except Exception:
        pass
    ttl = float(_DEFAULT_TTL_S)
    try:
        if fallback_ttl is not None:
            ttl = float(fallback_ttl)
    except (TypeError, ValueError):
        pass
    return time.time() + ttl


__all__ = ["AuthError", "DeviceAuth", "UserAuth", "read_env_file"]
