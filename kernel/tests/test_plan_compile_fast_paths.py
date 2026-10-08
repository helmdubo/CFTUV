"""Кэши компиляции сохраняют прежние выражения, отказы и цену точного поиска.

Oracle содержит прежние функции afc989c, а не новый код с выключенным кэшем.
Предел 2**-200 — контрпример к удалённому рациональному shortcut: его точный
ответ True/False менял прежний именованный отказ интервального предиката.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path
import pickle
from types import SimpleNamespace

from mpmath import iv
import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.reference import angular, adaptive_density_fan as fan_module
from cftuv_envelope.reference.adaptive_density_atlas import DensityExactWorkBudget
from cftuv_envelope.reference.compile import _density_ideal_is_subturn_feasible
from cftuv_envelope.reference.metric import ExactPlanarMetric

import plan_compile_legacy_oracle as legacy


IDENTITY = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
SKEW = ((sp.Integer(2), sp.Rational(1, 3)), (sp.Rational(1, 3), sp.Integer(3)))


def _metric(gram=IDENTITY, orientation=1):
    matrix = sp.Matrix(gram).inv()
    inverse = tuple(tuple(matrix[row, column] for column in range(2)) for row in range(2))
    return ExactPlanarMetric(gram, inverse, orientation)


def _vector(x, y):
    return angular._density_runtime_vector(x, y)


def _outcome(call):
    try:
        return ("ok", call())
    except Exception as error:
        return ("refused", type(error).__name__, getattr(error, "outcome", None), str(error))


def _fan_value(function, metric, incoming, outgoing, count, orientation, canonical=None, rotation=None):
    return tuple(
        tuple(sp.srepr(value) for value in vector.expressions())
        for vector in function(metric, incoming, outgoing, count, orientation, canonical, rotation)
    )


@pytest.mark.parametrize("gram", (IDENTITY, SKEW), ids=("identity", "skew"))
@pytest.mark.parametrize("orientation", (-1, 1))
@pytest.mark.parametrize("count", range(6))
@pytest.mark.parametrize("canonical", (None, Fraction(1, 2)))
def test_cold_and_reused_turn_atoms_keep_exact_fan_expressions(gram, orientation, count, canonical):
    incoming, outgoing = _vector(3, 2), _vector(-2, 4 * orientation)
    expected = _outcome(lambda: _fan_value(
        legacy._huber_density_interpolated_normals, _metric(gram), incoming, outgoing,
        count, orientation, canonical,
    ))
    metric = _metric(gram)
    for prior_count in (0, 2, 4, count, count):
        actual = _outcome(lambda: _fan_value(
            angular._huber_density_interpolated_normals, metric, incoming, outgoing,
            prior_count, orientation, canonical,
        ))
        if prior_count == count:
            assert actual == expected


@pytest.mark.parametrize("incoming,outgoing,count,orientation,rotation", (
    (_vector(1, 0), _vector(0, 1), -1, 1, None),
    (_vector(1, 0), _vector(0, 1), 6, 1, None),
    (_vector(1, 0), _vector(0, 1), 1, -1, None),
    (_vector(0, 0), _vector(0, 1), 1, 0, None),
    (_vector(1, 0), _vector(1, 0), 1, 0, None),
    (_vector(1, 0), _vector(sp.sqrt(2) + 1, 1), 1, 1, None),
    (_vector(1, 0), _vector(sp.cos(sp.pi / 8), sp.sin(sp.pi / 8)), 1, 1, None),
    (_vector(1, 0), _vector(0, 1), 1, 1, ((1, 1),)),
    (_vector(1, 0), _vector(0, 1), 2, 1, ((1, 1),)),
))
def test_stage_failures_and_rational_rotation_keep_the_old_result(incoming, outgoing, count, orientation, rotation):
    expected = _outcome(lambda: _fan_value(
        legacy._huber_density_interpolated_normals, _metric(), incoming, outgoing,
        count, orientation, rotation=rotation,
    ))
    metric = _metric()
    for _ in range(2):
        assert _outcome(lambda: _fan_value(
            angular._huber_density_interpolated_normals, metric, incoming, outgoing,
            count, orientation, rotation=rotation,
        )) == expected


@pytest.mark.parametrize("sign", (-1, 1))
def test_near_limit_keeps_the_named_refusal_instead_of_using_exact_turn_facts(sign):
    incoming, outgoing = _vector(1, 0), _vector(sp.Rational(sign, 2**200), 1)

    def evaluate(function):
        metric = _metric()
        normals = function(metric, incoming, outgoing, 1, 1)
        return _density_ideal_is_subturn_feasible(metric, normals, 4)

    expected = _outcome(lambda: evaluate(legacy._huber_density_interpolated_normals))
    assert expected[0:2] == ("refused", "ReferenceGeometryError")
    assert expected[-1] == "Density A exact sign is not certified without generic factorization"
    assert _outcome(lambda: evaluate(angular._huber_density_interpolated_normals)) == expected


def test_nonpositive_norm_still_refuses_before_reading_vector_expressions():
    class BrokenVector:
        def expressions(self, *args):
            raise ValueError("vector cannot be decoded")

    for squared in (sp.Integer(0), sp.Integer(-1)):
        expected = _outcome(lambda: legacy._density_unit_from_squared(BrokenVector(), squared, _metric()))
        assert expected[0:2] == ("refused", "ReferenceGeometryError")
        assert _outcome(lambda: angular._density_unit_from_squared(BrokenVector(), squared, _metric())) == expected


def test_cached_fan_is_reused_and_every_new_memo_slot_is_process_local():
    metric = _metric()
    incoming, outgoing = _vector(1, 0), _vector(1, 2)
    first = angular._huber_density_interpolated_normals(metric, incoming, outgoing, 2, 1)
    assert angular._huber_density_interpolated_normals(metric, incoming, outgoing, 2, 1) is first
    fan_module._covectors(metric, first)
    memo = metric._density_exact_memo
    for name in ("dot_expressions", "unit_vectors", "turn_atoms", "fans", "covector_rows"):
        assert getattr(memo, name)
        assert not getattr(pickle.loads(pickle.dumps(memo)), name)
    assert not _metric()._density_exact_memo.turn_atoms


@pytest.mark.parametrize("precision", (53, 160, 256))
@pytest.mark.parametrize("expression", (
    sp.sqrt(2) + sp.sqrt(3) - 3,
    sp.cos(sp.atan2(sp.sqrt(3), sp.Rational(1, 7)) / 3),
    sp.Add(sp.sqrt(2), -sp.sqrt(2), evaluate=False),
    sp.sqrt(2) - sp.Rational(1414213562373095048801688724209698078569671875376948, 10**51),
    sp.exp(1),
))
def test_interval_precision_and_quadratic_fallback_keep_old_outcomes(expression, precision):
    saved = iv.prec
    try:
        iv.prec = precision
        expected = _outcome(lambda: legacy._density_exact_sign(expression, _metric()))
        assert iv.prec == precision
        metric = _metric()
        for _ in range(2):
            assert _outcome(lambda: angular._density_exact_sign(expression, metric)) == expected
            assert iv.prec == precision
    finally:
        iv.prec = saved


FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
COMPILE_CASES = (
    ("binding_noise_canonical_v1/building_patch114", "decal_request_density4.json"),
    ("density4_exact_limit_v1/mesh2_patch2", "decal_request_density4.json"),
    ("wall_noise_top_join_density4_v1/wall_noise_top_patch1", "decal_request.json"),
)


def test_strip_index_preserves_first_match_failures_and_compilation_replacement():
    path = FIXTURES / "density4_exact_limit_v1/mesh2_patch2"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((path / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((path / "decal_request_density4.json").read_bytes())
    compilation = kernel.compile_reference_envelopes(snapshot, request).compilation
    strip = next(item for item in compilation.envelope_specs if isinstance(item, kernel.StripEnvelopeSpec))
    seed = next(item for item in compilation.seeds if item.seed_id == strip.source_seed_id)
    alternate = replace(strip, envelope_spec_id=kernel.EnvelopeSpecId("other-strip"))
    first = replace(compilation, envelope_specs=(strip, alternate), seeds=(seed,))
    context = SimpleNamespace(
        compilation=first, strip_index_cache={},
        support_segments_for_use=lambda use, identity: [SimpleNamespace(
            source_vertex_start_id="anchor", source_vertex_end_id="end",
            owner_normal=_vector(1, 0), support_id=identity,
        )],
    )
    for changed in (
        first,
        replace(first, envelope_specs=(alternate, strip)),
        replace(first, seeds=()),
        first,
    ):
        context.compilation = changed
        expected = _outcome(lambda: legacy._incident_normal(context, seed.chain_use_id, "anchor"))
        assert _outcome(lambda: angular._incident_normal(context, seed.chain_use_id, "anchor")) == expected
        assert _outcome(lambda: angular._incident_normal(context, seed.chain_use_id, "anchor")) == expected


def _legacy_functions(monkeypatch):
    for name in (
        "_incident_normal", "_density_dot_expression", "_density_unit_from_squared",
        "_density_left_unit_normal", "_huber_density_interpolated_normals",
    ):
        monkeypatch.setattr(angular, name, getattr(legacy, name))
    monkeypatch.setattr(fan_module, "_covectors", legacy._covectors)


@pytest.mark.parametrize("cap", (None, 0, 1))
def test_nonempty_search_budgets_and_cap_refusals_match_the_previous_path(cap, monkeypatch):
    budgets = []
    initialize = DensityExactWorkBudget.__init__

    def remember(self, exhausted, limit):
        initialize(self, exhausted, limit)
        budgets.append(self)

    monkeypatch.setattr(DensityExactWorkBudget, "__init__", remember)
    if cap is not None:
        monkeypatch.setattr(fan_module, "_DENSITY_EXACT_WORK_CAP", cap)

    def result():
        budgets.clear()
        metric = _metric()
        ideal = angular._huber_density_interpolated_normals(metric, _vector(1, 0), _vector(3, 1), 1, 1)
        answer = _outcome(lambda: kernel.canonical_json_bytes(fan_module.certify_adaptive_density_fan(
            metric, ideal, kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 2,
            binding_reasons=(None,),
        )))
        assert budgets and sum(item.spent for item in budgets) > 0
        return answer, tuple((b.cap, b.counters()) for b in budgets)

    with monkeypatch.context() as scope:
        _legacy_functions(scope)
        expected = result()
    assert expected[0][0] == ("ok" if cap is None else "refused")
    assert result() == expected


@pytest.mark.parametrize("folder,request_name", COMPILE_CASES)
def test_fixture_compilation_bytes_and_exact_search_budgets_match_old_functions(folder, request_name, monkeypatch):
    path = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((path / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((path / request_name).read_bytes())
    budgets = []
    initialize = DensityExactWorkBudget.__init__

    def remember(self, exhausted, cap):
        initialize(self, exhausted, cap)
        budgets.append(self)

    monkeypatch.setattr(DensityExactWorkBudget, "__init__", remember)

    def result():
        budgets.clear()
        compiled = kernel.compile_reference_envelopes(snapshot, request)
        return kernel.canonical_json_bytes(compiled), tuple((b.cap, b.counters()) for b in budgets)

    with monkeypatch.context() as scope:
        _legacy_functions(scope)
        expected = result()
    assert result() == expected
    assert result() == expected
