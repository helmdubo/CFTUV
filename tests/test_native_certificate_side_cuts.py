"""CERTIFICATE_SIDE_CUTS_NATIVE_V1: the side cuts of the drag certificate in ONE native call (`certificate.rs`): from the inputs of the domain's clip stage to the
slopes of the clip nodes and of the facts the clip added (`parametric._clip_slopes_at`), with no Python object in between.

The contract is the product's (NATIVE_PATH_PRODUCT_CONTRACT_V1): the same slopes as values (the order of the two dicts is not part of the answer), the same refusal by
name, the same certificate and the same sign counts of the whole step; a decline of the port applies nothing and the dispatcher runs the oracle on the same state.
CLIP_MEMO_MODE_PER_CALL_V1: the side cuts pass by the clip memory by their own argument, so no process-wide switch and no lock is left in the certificate.
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
    cftuv_native is None or not hasattr(cftuv_native, "certificate_side_slopes"), reason="расширение cftuv_native не собрано (или собрано без CERTIFICATE_SIDE_CUTS_NATIVE_V1)"
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


def _step(prepare, laws, alpha):
    """Two widths of one fresh preparation, everything native: `[(result, sign counts gained)]`."""

    from decimal import Decimal

    from cftuv_envelope import backend, exact_sqrt_sum
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.clip_memo import memo_disabled
    from cftuv_envelope.materialize.step import step_domain

    prepared = prepare()
    text = str(prepared.requested_alpha.value) if alpha is None else alpha
    found = []
    with backend.use_backend("NATIVE", materialize_backend="NATIVE"), memo_disabled():
        for width in (text, str(Decimal(text) * Decimal("1.001"))):
            before = dict(exact_sqrt_sum.SIGN_COUNTS)
            stepped = step_domain(
                prepared, width, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=laws[0], decal_topology_law=laws[1],
                digests=False, parametric=True,
            )
            after = dict(exact_sqrt_sum.SIGN_COUNTS)
            found.append((stepped.result, {key: after.get(key, 0) - before.get(key, 0) for key in after}))
    return found


def _captured():
    """The `(inputs, step, key_slope, fact_slope)` of every `_clip_slopes_at` of the cases (the oracle's run)."""

    from cftuv_envelope import backend
    from test_native_surface_lift import _cases
    from test_native_surface_lift import _step as python_step

    calls = []
    saved = backend.certificate_side_slopes_compute

    def capture(oracle, *arguments):
        calls.append(arguments)
        return saved(oracle, *arguments)

    backend.certificate_side_slopes_compute = capture
    try:
        for prepare, laws, alpha in _cases():
            python_step(prepare, laws, alpha, "PYTHON")
    finally:
        backend.certificate_side_slopes_compute = saved
    return calls


def _outcome(function, *arguments):
    try:
        point_slopes, fact_slopes = function(*arguments)
    except Exception as raised:  # noqa: BLE001 - the oracle's outcome is part of the answer
        return type(raised).__name__, getattr(raised, "name", str(raised)), getattr(raised, "detail", "")
    # the order of the dicts is not part of the answer (the certificate looks slopes up by key)
    return "answer", dict(point_slopes), dict(fact_slopes)


def test_the_certificate_routes_its_side_cuts_through_one_dispatcher_call():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/parametric.py").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert "return backend.certificate_side_slopes_compute(_clip_slopes_at, inputs, step, key_slope, fact_slope)" in text
    # CLIP_MEMO_MODE_PER_CALL_V1: no process-wide memo switch, no lock around the side cut
    assert "_SIDE_CUT_LOCK" not in text and "memo_disabled" not in text
    assert "use_memo=False," in text


def test_the_certificate_routes_its_arrays_through_one_dispatcher_call():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/parametric.py").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert "return backend.certificate_core_compute(_certificate_tail, inputs, batch, interval, request, key_slope, fact_slope, fact_floats, source, developable)" in text


def test_a_cut_without_memory_leaves_the_process_switch_alone(monkeypatch):
    from cftuv_envelope.materialize import clip_memo

    seen = []
    assert clip_memo.MEMO.enabled
    found, status = clip_memo.run_clip(lambda plane, budget, **inputs: seen.append(clip_memo.MEMO.enabled) or "cut", None, None, (), use_memo=False, points={})
    assert (found, status, seen) == ("cut", clip_memo.OFF, [True])
    assert clip_memo.MEMO.enabled


@needs_native
def test_every_side_cut_chain_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope.materialize import parametric

    calls = _captured()
    assert calls, "the cases certify no clip: the test would prove nothing"
    answered = 0
    for arguments in calls:
        expected = _outcome(parametric._clip_slopes_at, *arguments)
        assert _outcome(cftuv_native.certificate_side_slopes, *arguments) == expected
        answered += expected[0] == "answer" and bool(expected[1] or expected[2])
    assert answered, "no slope was found: the test would prove nothing"


@needs_native
def test_a_changed_cut_is_the_oracles_refusal_by_name(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize import parametric

    refused = set()
    for inputs, step, key_slope, fact_slope in _captured():
        cut = inputs.cut
        if cut.points:
            key = next(iter(cut.points))
            bent = dict(cut.points)
            bent[key] = (bent[key][0] + SqrtSumV1.rational(1), bent[key][1])
            forced = (replace(inputs, cut=replace(cut, points=bent)), step, key_slope, fact_slope)
            expected = _outcome(parametric._clip_slopes_at, *forced)
            assert expected[:2] == ("ParametricRefusal", parametric.PARAM_CLIP_NODE)
            assert _outcome(cftuv_native.certificate_side_slopes, *forced) == expected
            refused.add(expected[1])
        if cut.polygons:
            forced = (replace(inputs, cut=replace(cut, polygons=list(cut.polygons)[1:])), step, key_slope, fact_slope)
            expected = _outcome(parametric._clip_slopes_at, *forced)
            assert expected[:2] == ("ParametricRefusal", parametric.PARAM_CLIP_STRUCTURE_CHANGED)
            assert _outcome(cftuv_native.certificate_side_slopes, *forced) == expected
            refused.add(expected[1])
    assert refused == {parametric.PARAM_CLIP_NODE, parametric.PARAM_CLIP_STRUCTURE_CHANGED}, refused


@needs_native
def test_a_decline_applies_nothing_and_the_dispatcher_runs_the_oracle(approved):
    from cftuv_envelope import backend, exact_sqrt_sum
    from cftuv_envelope.materialize import parametric

    def state():
        return (
            dict(exact_sqrt_sum.SIGN_COUNTS), list(exact_sqrt_sum._KNOWN_PRIMES), dict(exact_sqrt_sum._FACTORIZATION_MEMO), dict(exact_sqrt_sum._SQUAREFREE_MEMO),
            dict(exact_sqrt_sum._PRIME_SUPPORT_MEMO),
        )

    arguments = _captured()[0]
    mirror = cftuv_native.default_mirror()
    before = state()
    mirror.force_refusal("unsupported")
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.certificate_side_slopes(*arguments)
    assert state() == before
    expected = _outcome(parametric._clip_slopes_at, *arguments)
    with backend.use_backend("NATIVE", materialize_backend="NATIVE") as ledger:
        mirror.force_refusal("unsupported")
        assert _outcome(backend.certificate_side_slopes_compute, parametric._clip_slopes_at, *arguments) == expected
    assert "certificate_side_slopes" in repr(ledger.fallbacks), ledger.fallbacks


@needs_native
def test_the_cases_answer_and_certify_the_same_with_and_without_the_fused_calls(approved, monkeypatch):
    from test_native_surface_lift import _cases, _certificate_fields

    from cftuv_envelope.materialize.step import answer_differences

    fused = []
    for name in ("certificate_side_slopes", "certificate_core"):
        real = getattr(cftuv_native, name)
        monkeypatch.setattr(cftuv_native, name, lambda *arguments, real=real, name=name: fused.append(name) or real(*arguments))
    with_fused = [_step(prepare, laws, alpha) for prepare, laws, alpha in _cases()]
    assert "certificate_core" in fused, "no certificate went through the fused call: the test would prove nothing"
    monkeypatch.delattr(cftuv_native, "certificate_side_slopes")
    monkeypatch.delattr(cftuv_native, "certificate_core")
    chain = [_step(prepare, laws, alpha) for prepare, laws, alpha in _cases()]
    compared = 0
    for one, other in zip(with_fused, chain):
        for (found, found_signs), (expected, expected_signs) in zip(one, other):
            assert answer_differences(expected, found) == ()
            assert expected.counters == found.counters
            assert _certificate_fields(expected.parametric) == _certificate_fields(found.parametric)
            assert expected_signs == found_signs
            compared += 1
    assert compared >= 10


def _captured_tails():
    """The arguments of every `_certificate_tail` of the cases (the oracle's run)."""

    from cftuv_envelope import backend
    from test_native_surface_lift import _cases
    from test_native_surface_lift import _step as python_step

    calls = []
    saved = backend.certificate_core_compute

    def capture(oracle, *arguments):
        calls.append(arguments)
        return saved(oracle, *arguments)

    backend.certificate_core_compute = capture
    try:
        for prepare, laws, alpha in _cases():
            python_step(prepare, laws, alpha, "PYTHON")
    finally:
        backend.certificate_core_compute = saved
    return calls


def _uniform_tails(monkeypatch):
    import materialize_factories as factories
    import developable_factories as df
    from test_parametric_certificate import developable, HOST_LAWS, CERTIFIED_LAWS
    from cftuv_envelope import backend
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.step import step_domain
    from cftuv_envelope.materialize.uv_phase import compile_uniform_uv_context

    calls = []
    def capture(oracle, *arguments):
        calls.append(arguments)
        return oracle(*arguments)
    monkeypatch.setattr(backend, "certificate_core_compute", capture)
    cases = ((lambda: factories.field_domain("building_002_point_contact_v1")[0], HOST_LAWS, "0.25"),
             (lambda: factories.field_domain("building_002_full_selection_v1")[0], HOST_LAWS, "0.25"),
             (lambda: factories.field_domain("mesh2_patch0_cut_fans_v1")[0], HOST_LAWS, "0.25"),
             (lambda: developable(df.fold_strip), CERTIFIED_LAWS, "0.3"))
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        for build, laws, alpha in cases:
            prepared = build()
            context = compile_uniform_uv_context((prepared,))
            result = step_domain(prepared, alpha, request=materialization_request(prepared, uv_policy_id="UV_UNIFORM_TRIM_FAN_V1"),
                                 near_planar_lift_law=laws[0], decal_topology_law=laws[1], digests=False, parametric=True, uv_context=context).result
            assert result.is_materialized and result.parametric is not None and not result.parametric.markers, (result.detail, result.parametric)
    return calls


def _certificate_outcome(function, *arguments):
    from test_native_surface_lift import _certificate_fields

    try:
        return "answer", _certificate_fields(function(*arguments))
    except Exception as raised:  # noqa: BLE001 - the oracle's outcome is part of the answer
        return type(raised).__name__, getattr(raised, "name", str(raised)), getattr(raised, "detail", "")


def _both(arguments):
    """`(oracle outcome, port outcome)` of one tail; the oracle's slope memory is a fresh one (the port does not take positions from it)."""

    from cftuv_envelope.materialize import parametric

    inputs, batch, interval, request, key_slope, fact_slope, fact_floats, _source, developable = arguments
    expected = _certificate_outcome(parametric._certificate_tail, inputs, batch, interval, request, key_slope, fact_slope, fact_floats, parametric._SlopeSource(None), developable)
    found = _certificate_outcome(cftuv_native.certificate_core, inputs, batch, interval, request, key_slope, fact_slope, fact_floats, developable)
    return expected, found


@needs_native
def test_every_certificate_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    compared = moving = 0
    for arguments in _captured_tails():
        if type(arguments[0].plane) is not BoundSurfaceLiftV1 or arguments[3].exact:
            with pytest.raises(cftuv_native.NATIVE_REFUSALS):
                cftuv_native.certificate_core(*arguments[:7], arguments[8])
            continue
        expected, found = _both(arguments)
        assert found == expected
        compared += 1
        moving += expected[0] == "answer" and dict(expected[1][-1]).get("PARAMETRIC_MOVING_VERTICES", 0) > 0
    assert compared and moving, "no certificate with moving vertices was compared: the test would prove nothing"


@needs_native
def test_uniform_certificate_transforms_physical_clip_slopes_and_names_missing_uv_forms(approved, monkeypatch):
    from cftuv_envelope.materialize import parametric
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    calls = _uniform_tails(monkeypatch)
    assert len(calls) == 4
    clipped = compared = declined = 0
    for arguments in calls:
        assert arguments[0].uv_forms and arguments[0].uv_numerators
        if type(arguments[0].plane) is not BoundSurfaceLiftV1:
            with pytest.raises(cftuv_native.NATIVE_REFUSALS):
                cftuv_native.certificate_core(*arguments[:7], arguments[8])
            declined += 1
            continue
        expected, found = _both(arguments)
        assert found == expected and expected[0] == "answer", (expected, found)
        clipped += arguments[0].cut is not None and bool(arguments[0].cut.points)
        compared += 1
        inputs = replace(arguments[0], uv_forms={})
        expected, found = _both((inputs, *arguments[1:]))
        assert found == expected and expected[:2] == ("ParametricRefusal", parametric.PARAM_FACT_UNRESOLVED)
        assert "UV_UNIFORM_SLOPE_FORM_MISSING" in expected[2]
    assert compared == 1 and declined == 3 and clipped, "the bound-surface physical clip and the plane declines were not exercised"


@needs_native
def test_a_vertex_without_its_triangle_or_a_loop_without_its_fact_is_the_oracles_refusal_by_name(approved):
    from cftuv_envelope.materialize import parametric
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    refused = set()
    for arguments in _captured_tails():
        inputs, batch, interval, request, key_slope, fact_slope, fact_floats, source, developable = arguments
        if type(inputs.plane) is not BoundSurfaceLiftV1 or request.exact:
            continue
        moving = [key for key, slope in key_slope.items() if slope is not None and key in inputs.lifted_names]
        if moving:
            names = dict(inputs.lifted_names)
            del names[moving[0]]
            expected, found = _both((replace(inputs, lifted_names=names), *arguments[1:]))
            assert found == expected and expected[0] == "ParametricRefusal", expected
            refused.add(expected[1])
        if fact_floats:
            # Факты петель читает физическая таблица: у карты UV (Straight strips) своя, поэтому отказ по имени проверяется без неё.
            expected, found = _both((replace(inputs, uv_numerators=None, uv_forms=None), *arguments[1:6], {}, source, developable))
            assert found == expected, expected
            refused.add(expected[1])
    assert {parametric.PARAM_LIFT_UNSUPPORTED, parametric.PARAM_FACT_UNRESOLVED} <= refused, refused


@needs_native
def test_a_declined_certificate_applies_nothing_and_the_dispatcher_runs_the_oracle(approved):
    from cftuv_envelope import backend, exact_sqrt_sum
    from cftuv_envelope.materialize import parametric
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    arguments = next(item for item in _captured_tails() if type(item[0].plane) is BoundSurfaceLiftV1 and not item[3].exact)
    inputs, batch, interval, request, key_slope, fact_slope, fact_floats, _source, developable = arguments
    mirror = cftuv_native.default_mirror()
    before = dict(exact_sqrt_sum.SIGN_COUNTS), dict(exact_sqrt_sum._FACTORIZATION_MEMO)
    mirror.force_refusal("unsupported")
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.certificate_core(*arguments[:7], developable)
    assert (dict(exact_sqrt_sum.SIGN_COUNTS), dict(exact_sqrt_sum._FACTORIZATION_MEMO)) == before
    expected = _certificate_outcome(parametric._certificate_tail, *arguments[:7], parametric._SlopeSource(None), developable)
    with backend.use_backend("NATIVE", materialize_backend="NATIVE") as ledger:
        mirror.force_refusal("unsupported")
        found = _certificate_outcome(backend.certificate_core_compute, parametric._certificate_tail, *arguments[:7], parametric._SlopeSource(None), developable)
    assert found == expected
    assert "certificate_core" in repr(ledger.fallbacks), ledger.fallbacks
