"""The rate fit used to calibrate `descriptor.jog_scale`.

The measurement tool drives a real robot, so the suite cannot run it end to end. What it CAN
pin is the part that decides whether a number gets published: the fit and its rejection rules.
A fit that quietly accepts a stalled or saturated run produces a confident wrong number, which
is worse than the honest silence of publishing nothing.

An end-to-end rehearsal against the mock is a manual step and worth doing before any hardware
session — the mock's JOG_SCALE is a known ground truth, so the tool should recover it:

    python tools/measure_jog_scale.py --mock --only gripper
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

_spec = importlib.util.spec_from_file_location(
    "measure_jog_scale",
    pathlib.Path(__file__).resolve().parent.parent / "tools" / "measure_jog_scale.py",
)
assert _spec and _spec.loader
measure_jog_scale = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(measure_jog_scale)

Measurement = measure_jog_scale.Measurement


def ramp(rate: float, n: int = 20, dt: float = 1 / 15, noise: float = 0.0) -> list:
    return [(i * dt, i * dt * rate + (noise if i % 2 else -noise)) for i in range(n)]


def test_a_clean_run_recovers_the_true_rate():
    m = Measurement("j")
    assert m.add(ramp(12.5)) is None
    assert m.rates[0] == pytest.approx(12.5)


def test_the_sign_of_the_jog_does_not_change_the_measured_rate():
    """Rate is a magnitude. A joint driven negative moves just as fast."""
    m = Measurement("j")
    m.add(ramp(12.5))
    m.add(ramp(-12.5))
    assert m.rates == pytest.approx([12.5, 12.5])


def test_a_saturated_run_is_rejected_not_averaged_in():
    """A joint that hits its limit mid-run reads as a slower robot. Averaging it in would
    publish a number that is wrong in the dangerous direction: a client asking for half speed
    would get more than half."""
    samples = ramp(12.5, n=10) + [(i / 15, 100.0) for i in range(10, 20)]
    assert "non-linear" in (Measurement("j").add(samples) or "")


def test_a_stalled_run_is_rejected():
    m = Measurement("j")
    assert m.add([(i / 15, 42.0) for i in range(20)]) is not None
    assert m.rates == []


def test_a_run_with_too_few_samples_is_rejected():
    """A short window is not a fast robot — it is no evidence."""
    assert "samples" in (Measurement("j").add(ramp(12.5, n=3)) or "")


def test_small_sensor_noise_still_fits():
    m = Measurement("j")
    assert m.add(ramp(12.5, noise=0.05)) is None
    assert m.rates[0] == pytest.approx(12.5, rel=0.02)


def test_one_run_is_never_enough_to_publish():
    """Repeatability is the claim being made, and a single run cannot support it."""
    m = Measurement("j")
    m.add(ramp(12.5))
    assert m.result() is None
    m.add(ramp(12.5))
    assert m.result() is not None


def test_the_result_reports_spread_so_an_unstable_joint_is_visible():
    """The caller refuses to publish above a spread threshold. That decision needs the spread
    to be real, not a summary that hides it."""
    m = Measurement("j")
    for rate in (10.0, 12.0, 14.0):
        m.add(ramp(rate))
    rate, spread = m.result()
    assert rate == pytest.approx(12.0)          # median, not mean — one bad run cannot drag it
    assert spread == pytest.approx(1.4)          # 14 / 10


def test_the_median_resists_a_single_outlier_run():
    m = Measurement("j")
    for rate in (12.4, 12.5, 12.6, 40.0):
        m.add(ramp(rate))
    rate, _spread = m.result()
    assert rate == pytest.approx(12.55)
