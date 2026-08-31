"""The public API surface, pinned.

An SDK's surface is a promise, and promises drift by accident: a helper loses its underscore,
a name is dropped from `__all__` while callers still import it, a new public method ships with
no docstring. None of that fails a normal test suite, and all of it reaches users.

So these tests treat the surface as data. The snapshot below is the ONE place a change to the
public API has to be made deliberately — if you meant it, update the list in the same commit
and the diff shows a reviewer exactly what consumers will see.
"""

from __future__ import annotations

import ast
import inspect
import pathlib

import pytest

import nori_sdk
from nori_sdk.teleop import RemoteTeleop

SRC = pathlib.Path(__file__).resolve().parent.parent / "src" / "nori_sdk"

# The top-level surface. Adding or removing a line here is the deliberate act.
EXPECTED_TOP_LEVEL = {
    # protocol version + constant sets
    "NORI_PROTOCOL_VERSION", "RECOVERY_ERROR_CODES", "TERMINAL_ACTION_STATES",
    "TERMINAL_NAVIGATION_STATES",
    # wire types
    "ActionStatus", "CameraLayout", "ConnectStatus", "DaemonStatus", "Perception",
    "PolicyStreamStatus", "RecordState", "RobotDescriptor", "RobotError", "RobotInfo",
    "Telemetry", "WatchdogProfile",
    # wire types: named navigation + opt-in LiDAR/IMU streams
    "ImuSample", "LidarScan", "NavigationState", "NavigationStatus", "RosStamp",
    "SensorStreamStatus", "WaypointSummary",
    # session
    "RemoteTeleop", "RobotUnreachable", "TeleopError",
    # signaling
    "IcePayload", "NackPayload", "ReadyTurn", "SdpPayload", "SignalingHandlers",
    "SignalingState", "SignalingTransport", "SupabaseSignaling",
    # auth
    "AuthError", "DeviceAuth", "UserAuth",
    # submodules + dunder
    "motion", "protocol", "types", "__version__",
}

# Needs an optional extra, so it cannot be resolved by a bare getattr in every environment.
LAZY = {"RemoteTeleop", "TeleopError", "RobotUnreachable", "SupabaseSignaling"}


def test_the_top_level_surface_is_exactly_what_we_promised():
    actual = set(nori_sdk.__all__)
    added, removed = actual - EXPECTED_TOP_LEVEL, EXPECTED_TOP_LEVEL - actual
    assert not added, f"new public API not recorded in this test: {sorted(added)}"
    assert not removed, f"public API removed — this BREAKS callers: {sorted(removed)}"


def test_every_promised_name_actually_resolves():
    """Catches the failure `__all__` invites: a typo or a rename leaves a name that only
    explodes at `from nori_sdk import *`, which no unit test does."""
    for name in nori_sdk.__all__:
        assert getattr(nori_sdk, name, None) is not None, f"{name} is promised but unresolvable"


def test_star_import_yields_the_whole_surface():
    ns: dict[str, object] = {}
    exec("from nori_sdk import *", ns)  # noqa: S102 - exercising the real import path
    for name in nori_sdk.__all__:
        assert name in ns, f"{name} is in __all__ but star-import did not bind it"


def test_the_lazy_names_are_discoverable():
    """RemoteTeleop is the class this package exists to provide. Without __dir__ it was
    missing from tab-completion, help() and IDE introspection until something touched it —
    a symbol you cannot discover is, for most users, a symbol that does not exist."""
    listed = dir(nori_sdk)
    for name in LAZY:
        assert name in listed, f"{name} is invisible to dir()/help()/tab-completion"


def test_an_unknown_attribute_still_raises_attribute_error():
    """__getattr__ must not turn a typo into something stranger than AttributeError."""
    with pytest.raises(AttributeError, match="RemoteTeleopp"):
        nori_sdk.RemoteTeleopp  # noqa: B018


def _public_names(path: pathlib.Path) -> tuple[set[str], set[str]]:
    """(declared in __all__, defined at module level and public by convention)."""
    tree = ast.parse(path.read_text())
    declared: set[str] = set()
    defined: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if not node.name.startswith("_"):
                defined.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    if target.id == "__all__":
                        declared = {e.value for e in node.value.elts}  # type: ignore[attr-defined]
                    elif not target.id.startswith("_"):
                        defined.add(target.id)
    return declared, defined


MODULES = sorted(SRC.rglob("*.py"))


@pytest.mark.parametrize("path", MODULES, ids=lambda p: p.name)
def test_no_module_leaks_an_undeclared_public_name(path):
    """Reachable-but-undeclared is the worst of both worlds: people depend on it anyway, and
    nothing stops it changing. Either name it in __all__ or prefix it with an underscore."""
    declared, defined = _public_names(path)
    # `log` is a module logger — conventional, and not part of the promised surface.
    leaked = defined - declared - {"log"}
    assert not leaked, f"{path.name}: public but not in __all__: {sorted(leaked)}"


@pytest.mark.parametrize("path", MODULES, ids=lambda p: p.name)
def test_every_module_declares_a_surface(path):
    declared, _ = _public_names(path)
    assert declared or path.name == "py.typed", f"{path.name} has no __all__"


def _documented(obj) -> bool:
    return bool((obj.__doc__ or "").strip())


def test_every_public_member_of_the_session_is_documented():
    """RemoteTeleop is the surface almost every user touches, and its methods carry the
    behaviour that is NOT guessable from a signature — which side owns the jog repetition,
    that reset_latch is only for a latch, that is_connected does not mean the robot will
    move. An undocumented one here is a trap, not a gap."""
    undocumented = []
    for name, member in vars(RemoteTeleop).items():
        if name.startswith("_"):
            continue
        target = member.fget if isinstance(member, property) else member
        if callable(target) and not _documented(target):
            undocumented.append(name)
    assert not undocumented, f"public but undocumented: {sorted(undocumented)}"


@pytest.mark.parametrize(
    "name", sorted(n for n in EXPECTED_TOP_LEVEL if n[0].isupper() and n not in LAZY)
)
def test_every_exported_type_is_documented(name):
    obj = getattr(nori_sdk, name)
    if inspect.isclass(obj):
        assert _documented(obj), f"{name} has no docstring"


def test_the_package_advertises_its_types():
    """Without py.typed, mypy refuses to check this package at all and silently reports
    success — and every downstream consumer gets no type information despite full
    annotations. It is one empty file and it was missing."""
    assert (SRC / "py.typed").is_file()
