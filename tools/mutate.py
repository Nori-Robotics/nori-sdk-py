"""Revert each fix one at a time; report which tests catch it.

A fix with no failing mutation is a fix with no test. Bytecode writing is off because a
previous round of this got fooled by a stale .pyc: two versions of a line were the same
length and the edit/run/restore cycle finished inside one second, so Python's (mtime, size)
staleness check served the old code.
"""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

MUTATIONS = [
    (
        "1. base jog emits the telemetry namespace again",
        "src/nori_sdk/motion.py",
        '            group["linear"] = clamp(linear)',
        '            group["x"] = clamp(linear)',
    ),
    (
        "1b. base() silently accepts x/theta instead of raising",
        "src/nori_sdk/motion.py",
        "        if rejected:\n            raise ValueError(",
        "        if False:\n            raise ValueError(",
    ),
    (
        "2. 'accepted' is terminal again",
        "src/nori_sdk/types.py",
        'TERMINAL_ACTION_STATES = frozenset({"done", "blocked", "clamped", "timeout"})',
        'TERMINAL_ACTION_STATES = frozenset({"accepted", "done", "blocked", "clamped", "timeout"})',
    ),
    (
        "2b. action(wait) resolves on the first frame, not a terminal one",
        "src/nori_sdk/teleop.py",
        "if action_future is not None and parsed.done and not action_future.done():",
        "if action_future is not None and not action_future.done():",
    ),
    (
        "2c. the mock emits only 'accepted' again (the double that hid the bug)",
        "src/nori_sdk/mock/robot.py",
        'return ["accepted", "active", self.action_outcome]',
        'return ["accepted"]',
    ),
    (
        "3. policy_stream_status ok defaults true again",
        "src/nori_sdk/types.py",
        'ok=obj.get("ok") is True,',
        'ok=obj.get("ok") is not False,',
    ),
    (
        "3b. streaming defaults true",
        "src/nori_sdk/types.py",
        'streaming=obj.get("streaming") is True,',
        'streaming=obj.get("streaming") is not False,',
    ),
    (
        "4. stateless daemon_status is coerced to offline again",
        "src/nori_sdk/types.py",
        "        if not state:\n            return None",
        '        if not state:\n            state = "offline"',
    ),
    (
        "4b. teleop keeps the dropped frame instead of returning",
        "src/nori_sdk/teleop.py",
        "            if not isinstance(parsed, DaemonStatus):\n                # Stateless",
        "            if False:\n                # Stateless",
    ),
    (
        "5. legacy record aliases dropped from RecordVerb again",
        "src/nori_sdk/protocol.py",
        '    "session_discard",\n    "status",\n    "start",\n    "stop",\n    "discard",\n    "discard_last",\n]',
        '    "status",\n]',
    ),
    (
        "5b. a verb no robot accepts is offered",
        "src/nori_sdk/protocol.py",
        '    "discard_last",\n]',
        '    "discard_last",\n    "obliterate",\n]',
    ),
    (
        "5c. session_end misclassified as destructive",
        "src/nori_sdk/protocol.py",
        'DESTRUCTIVE_RECORD_VERBS = frozenset({"episode_discard", "session_discard", "discard_last"})',
        'DESTRUCTIVE_RECORD_VERBS = frozenset({"episode_discard", "session_end", "discard_last"})',
    ),
    (
        "6. error frames unmodelled again",
        "src/nori_sdk/protocol.py",
        '    elif kind == "error":\n        parsed = RobotError.from_wire(obj)',
        '    elif kind == "error" and False:\n        parsed = RobotError.from_wire(obj)',
    ),
    (
        "6b. error.fatal defaults TRUE (would tear down a session on a soft stall)",
        "src/nori_sdk/types.py",
        'fatal=obj.get("fatal") is True,',
        'fatal=obj.get("fatal") is not False,',
    ),
    (
        "6c. recovery codes indistinguishable from faults",
        "src/nori_sdk/types.py",
        '{"obstruction_cleared", "arm_recovered", "motor_recovered"}',
        'set()',
    ),
    (
        "7. tile-less camera_layout accepted again",
        "src/nori_sdk/types.py",
        "        if not tiles:\n            return None",
        "        if False:\n            return None",
    ),
    (
        "8. capabilities: absent collapsed into 'supports nothing'",
        "src/nori_sdk/types.py",
        "        if self.capabilities is None:\n            return None",
        "        if self.capabilities is None:\n            return False",
    ),
    (
        "8b. ack model dropped",
        "src/nori_sdk/types.py",
        'model=_s(obj, "model"),',
        "model=None,",
    ),
    (
        "9. the mock derives its layout from the descriptor (the antipattern)",
        "src/nori_sdk/mock/robot.py",
        '"tiles": list(self.tiles)}',
        '"tiles": list((self.descriptor or {}).get("cameras", []))}',
    ),
    (
        '10. __dir__ removed: RemoteTeleop invisible to tab-completion again',
        'src/nori_sdk/__init__.py',
        'def __dir__() -> list[str]:',
        'def _disabled__dir__() -> list[str]:',
    ),
    (
        '10b. a name silently dropped from the public surface',
        'src/nori_sdk/__init__.py',
        '    "RobotError",\n    "RobotInfo",',
        '    "RobotInfo",',
    ),
    (
        '10c. a public session method loses its docstring',
        'src/nori_sdk/teleop.py',
        '    def reset_latch(self) -> None:\n        """',
        '    def reset_latch(self) -> None:\n        pass_doc = """',
    ),
    (
        '10d. a constant goes public without being declared',
        'src/nori_sdk/mock/robot.py',
        'JOG_SCALE = 40.0',
        'JOG_SCALE = 40.0\nSNEAKY_PUBLIC_CONSTANT = 1',
    ),
]


def run() -> list[str]:
    p = subprocess.run(
        [str(ROOT / ".venv/bin/pytest"), "-q", "--no-header", "-x", "--tb=no", "-p", "no:cacheprovider"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,  # a non-zero exit IS the result we are measuring
        env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1", "HOME": str(pathlib.Path.home())},
    )
    return [ln for ln in p.stdout.splitlines() if ln.startswith(("FAILED", "ERROR"))] or (
        ["<<< NO TEST FAILED >>>"] if p.returncode == 0 else ["<<< failed, no FAILED line >>>"]
    )


failures = 0
for label, relpath, old, new in MUTATIONS:
    path = ROOT / relpath
    original = path.read_text()
    if old not in original:
        print(f"{label}\n    !! mutation target not found in {relpath}\n")
        failures += 1
        continue
    assert original.count(old) == 1, f"{label}: target is not unique"
    path.write_text(original.replace(old, new))
    try:
        caught = run()
    finally:
        path.write_text(original)
    ok = not caught[0].startswith("<<<")
    failures += 0 if ok else 1
    print(f"{'PASS' if ok else 'FAIL'}  {label}")
    for line in caught[:3]:
        print(f"        {line}")
    print()

print("clean run after restore:", run())
sys.exit(1 if failures else 0)
