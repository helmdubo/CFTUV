"""SIDE_CUT_NATIVE_V1: the side cuts of the drag certificate in Rust (`side_cut.rs`): the points, contours and facts of a side cut (`parametric._side_inputs`)
and the slopes of the clip by the exact second differences of three cuts (`parametric._clip_slope_values`), against the oracle.

Every call of the cases is replayed on both: the same containers in the same order, a point without a slope the same object; a second difference that is not zero is the
oracle's refusal by its name; a key the oracle would raise `KeyError` on is declined by the port.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "clip_slope_values"), reason="расширение cftuv_native не собрано (или собрано без SIDE_CUT_NATIVE_V1)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op
    from cftuv_native import pin

    op.refresh()
    pin.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _captured():
    """`(side input calls, slope calls)` of the cases (the oracle's run)."""

    from cftuv_envelope import backend
    from test_native_surface_lift import _cases, _step

    sides, slopes = [], []
    saved_sides, saved_slopes = backend.side_inputs_compute, backend.clip_slope_values_compute

    def side(oracle, *arguments):
        sides.append(arguments)
        return saved_sides(oracle, *arguments)

    def slope(oracle, *arguments):
        slopes.append(arguments)
        return saved_slopes(oracle, *arguments)

    backend.side_inputs_compute, backend.clip_slope_values_compute = side, slope
    try:
        for prepare, laws, alpha in _cases():
            _step(prepare, laws, alpha, "PYTHON")
    finally:
        backend.side_inputs_compute, backend.clip_slope_values_compute = saved_sides, saved_slopes
    return sides, slopes


def _outcome(function, *arguments):
    try:
        return "answer", function(*arguments)
    except Exception as raised:  # noqa: BLE001 - the oracle's outcome is part of the answer
        return type(raised).__name__, getattr(raised, "name", str(raised))


def _flat(found):
    kind, value = found
    if kind != "answer":
        return found
    return kind, tuple(list(item.items()) if isinstance(item, dict) else item for item in value)


def test_the_certificate_routes_its_side_cuts_through_the_dispatcher():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/parametric.py").read_text(encoding="utf-8")
    assert "backend.side_inputs_compute(_side_inputs, inputs.points_before, inputs.cycles, inputs.facts_before, key_slope, fact_slope, shift)" in text
    assert "backend.clip_slope_values_compute(\n        _clip_slope_values," in text.replace("\r\n", "\n")


@needs_native
def test_every_side_cut_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope.materialize import parametric

    sides, slopes = _captured()
    assert sides and slopes, "the cases cut no side: the test would prove nothing"
    moved = 0
    for arguments in sides:
        expected = parametric._side_inputs(*arguments)
        found = cftuv_native.side_inputs(*arguments)
        assert list(found[0].items()) == list(expected[0].items())
        assert found[1] == expected[1]
        assert list(found[2].items()) == list(expected[2].items())
        points_before, key_slope = arguments[0], arguments[3]
        for key, point in points_before.items():
            if key_slope.get(key) is None:
                assert found[0][key] is point
            else:
                moved += 1
    assert moved, "no point moved: the test would prove nothing"
    answered = 0
    for arguments in slopes:
        expected = _flat(_outcome(parametric._clip_slope_values, *arguments))
        assert _flat(_outcome(cftuv_native.clip_slope_values, *arguments)) == expected
        answered += expected[0] == "answer" and bool(expected[1][0] or expected[1][1])
    assert answered, "no slope was found: the test would prove nothing"


@needs_native
def test_a_bent_side_cut_is_the_oracles_refusal_by_name(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize import parametric

    _sides, slopes = _captured()
    refused = set()
    for arguments in slopes:
        cut, up, down, facts_before, facts_after, facts_up, facts_down, delta = arguments
        if up.points:
            key = next(iter(up.points))
            bent = dict(up.points)
            bent[key] = (bent[key][0] + SqrtSumV1.rational(1), bent[key][1])
            forced = (cut, replace(up, points=bent), down, facts_before, facts_after, facts_up, facts_down, delta)
            expected = _outcome(parametric._clip_slope_values, *forced)
            assert expected == ("ParametricRefusal", parametric.PARAM_CLIP_NODE)
            assert _outcome(cftuv_native.clip_slope_values, *forced) == expected
            refused.add(expected[1])
        added = [slot for slot in facts_after if slot not in facts_before]
        if added:
            bent = dict(facts_up)
            bent[added[0]] = (bent[added[0]][0], bent[added[0]][1] + SqrtSumV1.rational(1))
            forced = (cut, up, down, facts_before, facts_after, bent, facts_down, delta)
            expected = _outcome(parametric._clip_slope_values, *forced)
            assert expected == ("ParametricRefusal", parametric.PARAM_CLIP_FACT)
            assert _outcome(cftuv_native.clip_slope_values, *forced) == expected
            refused.add(expected[1])
    assert refused == {parametric.PARAM_CLIP_NODE, parametric.PARAM_CLIP_FACT}, refused


@needs_native
def test_a_missing_key_is_declined_and_the_oracle_raises(approved):
    from cftuv_envelope.materialize import parametric

    sides, slopes = _captured()
    points_before, cycles_before, facts_before, key_slope, fact_slope, shift = sides[0]
    broken = [[("no such key", None)]] + list(cycles_before)
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.side_inputs(points_before, broken, facts_before, key_slope, fact_slope, shift)
    with pytest.raises(KeyError):
        parametric._side_inputs(points_before, broken, facts_before, key_slope, fact_slope, shift)
    for arguments in slopes:
        cut, up, down, *rest = arguments
        if cut.points:
            forced = (cut, replace(up, points={}), down, *rest)
            with pytest.raises(cftuv_native.NATIVE_REFUSALS):
                cftuv_native.clip_slope_values(*forced)
            with pytest.raises(KeyError):
                parametric._clip_slope_values(*forced)
            return
    pytest.fail("no cut had a clip node")


@needs_native
def test_the_cases_answer_and_certify_the_same_under_both_backends(approved, monkeypatch):
    from test_native_surface_lift import _cases, _certificate_fields, _step

    from cftuv_envelope.materialize.step import answer_differences

    calls = []
    for name in ("side_inputs", "clip_slope_values"):
        real = getattr(cftuv_native, name)
        monkeypatch.setattr(cftuv_native, name, lambda *arguments, real=real, name=name: calls.append(name) or real(*arguments))
    compared = 0
    for prepare, laws, alpha in _cases():
        for (expected, expected_signs), (found, found_signs) in zip(_step(prepare, laws, alpha, "PYTHON"), _step(prepare, laws, alpha, "NATIVE")):
            assert answer_differences(expected, found) == ()
            assert expected.counters == found.counters
            assert _certificate_fields(expected.parametric) == _certificate_fields(found.parametric)
            assert expected_signs == found_signs
            compared += 1
    assert compared >= 10
    assert {"side_inputs", "clip_slope_values"} <= set(calls), calls
