"""Token-cache behavior, with no network.

The scenarios that matter are the ones a real machine hits: a clock that has not met NTP
yet, a clock that steps mid-session, and a flaky auth endpoint. In those cases a token cache
stops being an optimization and becomes the reason a client can or cannot sign in at all.

These mirror nori_ws/src/nori_identity/test/test_device_auth.py, deliberately — this module
is a port of that one, and the two must not drift. See docs/known_issues.md #18 in nori_ws.
"""

import base64
import json

import pytest

from nori_sdk import auth as auth_mod
from nori_sdk.auth import AuthError, UserAuth


class FakeClock:
    """Independent wall and monotonic clocks — the only way to express "NTP stepped"."""

    def __init__(self, wall=1_700_000_000.0, mono=1000.0):
        self.wall = wall
        self.mono = mono

    def step_wall(self, seconds):
        self.wall += seconds

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds


@pytest.fixture
def clock(monkeypatch):
    fake = FakeClock()
    monkeypatch.setattr(auth_mod.time, "time", lambda: fake.wall)
    monkeypatch.setattr(auth_mod.time, "monotonic", lambda: fake.mono)
    return fake


def _jwt(exp):
    payload = base64.urlsafe_b64encode(json.dumps({"exp": exp}).encode()).decode()
    return f"header.{payload.rstrip('=')}.signature"


_UNSET = object()


def make_auth(clock, ttl_s=3600, exp_offset=None, expires_in=_UNSET):
    a = UserAuth("https://auth.example", "anon", "me@example.com", "pw")
    calls = []

    def fake_post(path, body):
        calls.append(path)
        offset = ttl_s if exp_offset is None else exp_offset
        return {
            "access_token": _jwt(clock.wall + offset),
            "refresh_token": f"refresh-{len(calls)}",
            "expires_in": ttl_s if expires_in is _UNSET else expires_in,
        }

    a._post = fake_post
    return a, calls


def test_a_healthy_token_is_served_from_cache(clock):
    a, calls = make_auth(clock)
    first = a.token()
    for _ in range(50):
        assert a.token() == first
    assert len(calls) == 1


def test_a_clock_running_ahead_does_not_cause_a_grant_storm(clock):
    # Every freshly minted token looks already-expired to us, so without a floor on the
    # cache hold, every caller would perform a full grant and rate-limit us out.
    a, calls = make_auth(clock, exp_offset=0)
    for _ in range(100):
        a.token()
    assert len(calls) == 1


def test_a_skewed_clock_still_refreshes_eventually(clock):
    a, calls = make_auth(clock, exp_offset=0)
    a.token()
    clock.advance(31)  # literal, not _MIN_CACHE_S: see the nori_ws twin
    a.token()
    assert len(calls) == 2


def test_a_backward_wall_clock_step_does_not_pin_an_expired_token(clock):
    a, calls = make_auth(clock, ttl_s=3600)
    a.token()
    clock.step_wall(-7200)  # NTP corrects a fast clock
    clock.advance(3001)
    a.token()
    assert len(calls) == 2


def test_a_forward_wall_clock_step_does_not_force_an_early_grant(clock):
    a, calls = make_auth(clock, ttl_s=3600)
    a.token()
    clock.step_wall(7200)
    a.token()
    assert len(calls) == 1


def test_a_short_token_is_still_served_for_its_first_half(clock):
    a, calls = make_auth(clock, ttl_s=60)
    a.token()
    clock.advance(29)
    a.token()
    assert len(calls) == 1


def _grant_type_of(path):
    return path.split("grant_type=")[1]


def _fail_both_grants(a, error):
    """Make BOTH grants fail. That is the only window in which the 4xx-vs-network distinction
    is observable: on a successful password fallthrough `_grant` rewrites `_refresh` from the
    response, so a test that lets it succeed cannot tell "kept the token" from "discarded it
    and got a new one". The previous versions of these two tests made exactly that mistake
    and stayed green with the guard deleted."""
    original = a._post

    def failing(path, body):
        raise error

    a._post = failing
    return original


def test_a_network_failure_on_refresh_keeps_the_refresh_token(clock):
    a, calls = make_auth(clock)
    a.token()
    clock.advance(3001)
    original = _fail_both_grants(a, AuthError("auth unreachable", status=None))
    with pytest.raises(AuthError):
        a.token()
    a._post = original
    calls.clear()
    a.token()
    assert _grant_type_of(calls[0]) == "refresh_token", (
        "a blip discarded the refresh token; this process now password-grants forever"
    )


def test_a_4xx_on_refresh_discards_the_refresh_token(clock):
    a, calls = make_auth(clock)
    a.token()
    clock.advance(3001)
    original = _fail_both_grants(a, AuthError("auth 401", status=401))
    with pytest.raises(AuthError):
        a.token()
    a._post = original
    calls.clear()
    a.token()
    assert _grant_type_of(calls[0]) == "password", (
        "a refused refresh token was retained and will be retried forever"
    )


def test_a_clock_running_BEHIND_never_serves_a_token_past_its_life(clock):
    """The likelier field case: a Pi with a flat RTC battery boots BEHIND, not ahead."""
    for behind_s in (0, 600, 7200, 86400):
        a, _calls = make_auth(clock, ttl_s=3600, exp_offset=3600 + behind_s)
        a.token()
        held = a._refresh_after_mono - clock.mono
        # <= 3600 would pass at the instant of death; require a real refresh margin.
        assert held <= 3000, (
            f"clock {behind_s}s behind: holding a 3600 s token for {held:.0f} s "
            f"-- only {3600 - held:.0f} s of margin"
        )


def test_an_absurd_exp_cannot_pin_the_token(clock):
    a, _calls = make_auth(clock, ttl_s=3600, exp_offset=1_700_000_000_000)
    a.token()
    assert a._refresh_after_mono - clock.mono <= 3600


def test_from_env_returns_none_when_incomplete(monkeypatch):
    for var in ("NORI_SUPABASE_URL", "NORI_SUPABASE_ANON_KEY", "NORI_EMAIL", "NORI_PASSWORD"):
        monkeypatch.delenv(var, raising=False)
    assert UserAuth.from_env() is None


def test_require_names_every_missing_credential():
    with pytest.raises(AuthError) as excinfo:
        UserAuth("", "", "", "").require()
    for field in ("supabase_url", "supabase_anon_key", "email", "password"):
        assert field in str(excinfo.value)


def test_a_short_server_lifetime_cannot_drive_the_grant_rate(clock):
    """The floor must be the OUTERMOST bound. Applied before the cap it stopped being one:
    a server answering `expires_in: 1` set the grant rate to 3600/hour."""
    for expires_in in (1, 5, 29):
        a, _calls = make_auth(clock, exp_offset=0, expires_in=expires_in)
        a.token()
        held = a._refresh_after_mono - clock.mono
        assert held >= 30, f"expires_in={expires_in} -> {3600 / held:.0f} grants/hour"


def test_a_server_declared_dead_token_is_not_cached_for_an_hour(clock):
    for expires_in in (0, -100):
        a, _calls = make_auth(clock, ttl_s=3600, expires_in=expires_in)
        a.token()
        assert a._refresh_after_mono - clock.mono == 30


def test_an_absent_expires_in_still_falls_back_to_the_default(clock):
    """Absent is NOT zero: the server told us nothing, so do not invent a tighter bound."""
    a, _calls = make_auth(clock, ttl_s=3600, expires_in=None)
    a.token()
    assert a._refresh_after_mono - clock.mono == 3000


def test_a_failed_grant_never_puts_a_credential_in_the_error_text():
    """The exception text is logged at WARNING by the signaling layer, so anything
    interpolated into it lands in journald and in any support log bundle. A partial grant
    response can carry a refresh_token while lacking an access_token -- name the KEYS, never
    the values."""
    auth = UserAuth("https://auth.example", "anon", "me@example.com", "pw")
    auth._post = lambda path, body: {
        "refresh_token": "SECRET-REFRESH-VALUE", "token_type": "bearer"}
    with pytest.raises(AuthError) as excinfo:
        auth.token()
    message = str(excinfo.value)
    assert "SECRET-REFRESH-VALUE" not in message, f"credential leaked into: {message}"
    assert "refresh_token" in message, "should still name the keys, for diagnosis"
