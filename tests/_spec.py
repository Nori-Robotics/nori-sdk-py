"""Locate the nori-protocol spec.

Plain module, not a conftest: test_conformance parametrises over the fixtures at import
time, which needs a real import rather than a pytest fixture.

The spec is a separate repo (Nori-Robotics/Nori-Protocol) and is meant to be mounted here
as a submodule at `spec/nori-protocol`. Until that happens it can be pointed at a local
checkout, which is also how you test a spec change before pushing it:

    NORI_PROTOCOL_DIR=../NoriTeleop/rpi5/nori_core_agent/external/nori-protocol pytest

Resolution order: the env var, then the submodule path, then the known sibling checkout.
If none exists the conformance suite SKIPS rather than fails -- a contributor without the
submodule initialised should still be able to run the unit tests.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_CANDIDATES = [
    REPO_ROOT / "spec" / "nori-protocol",
    # Dev fallback: the spec is currently vendored inside the L2 repo.
    REPO_ROOT.parent / "NoriTeleop" / "rpi5" / "nori_core_agent" / "external" / "nori-protocol",
]


def find_spec_dir() -> Path | None:
    """The spec directory, or None if it genuinely is not present.

    An explicitly-set NORI_PROTOCOL_DIR that does not resolve RAISES rather than returning
    None. Returning None would skip the conformance suite, and a CI job with a typo'd path
    would then pass green while testing nothing -- which is precisely the failure mode this
    whole spec exists to prevent."""
    env = os.environ.get("NORI_PROTOCOL_DIR")
    if env:
        p = Path(env).expanduser()
        if not (p / "VERSION").is_file():
            raise RuntimeError(
                f"NORI_PROTOCOL_DIR={env!r} is not a nori-protocol checkout (no VERSION "
                f"file at {p / 'VERSION'}). Unset it to fall back to the submodule, or "
                f"point it at a real checkout -- refusing to silently skip conformance."
            )
        return p
    for p in _CANDIDATES:
        if (p / "VERSION").is_file():
            return p
    return None


SPEC_DIR = find_spec_dir()


def load_fixtures() -> list[tuple[str, dict]]:
    """(relative name, frame) for every golden fixture, both layers."""
    if SPEC_DIR is None:
        return []
    out = []
    for path in sorted((SPEC_DIR / "fixtures").rglob("*.json")):
        out.append((path.relative_to(SPEC_DIR / "fixtures").as_posix(), json.loads(path.read_text())))
    return out


def load_schema(message_type: str) -> dict | None:
    """The schema for a message type, from a SESSION client's point of view.

    Session first, then daemon. Most messages exist only under schema/daemon/ and ride the
    data channel unchanged, so the fallback covers them -- but where both layers define a
    type, the session dialect is the one this SDK speaks. `command` is the case that makes
    the order matter: the daemon form is {name, token}, the session form is a bare {estop:
    true} flag, and the bridge translates. Checking daemon first would validate our frames
    against a contract we never speak.
    """
    if SPEC_DIR is None:
        return None
    for layer in ("session", "daemon"):
        path = SPEC_DIR / "schema" / layer / f"{message_type}.json"
        if path.is_file():
            return json.loads(path.read_text())
    return None
