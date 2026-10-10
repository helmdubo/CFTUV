"""PLAN-COMPILE-DEDUP: повторы и выражения, построенные только ради знака, не меняют ни ответа, ни отказа.

Каждая правка здесь сравнивается с ПРЕЖНИМ кодом, скопированным в этот файл как oracle (`legacy_*`), а не с самой
собой при выключенной оптимизации:

* `squared` - тот же объект, что `value * value`, без обхода `Add._eval_power`;
* `residual_sign` - тот же знак (и тот же названный отказ), что `_density_exact_sign` над построенным остатком;
  фильтр молчит там, где оболочка остатка не отстоит от нуля с запасом;
* `bind_density_fan` - память по значению входа: успех берётся из неё, отказ повторяется как был;
* `settled_result` / `station_plans_of` / `plan_errors(recompute=)` - память по тождеству неизменяемых входов;
* `_search` - пустая высота платит ровно один probe, как платила `_farey_shell_candidates`.
"""

from __future__ import annotations

from fractions import Fraction
from math import gcd
from pathlib import Path
import pickle

from mpmath import iv
import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import _chain_station
from cftuv_envelope.contracts.envelopes import DirectionBindingReasonV1
from cftuv_envelope.reference import (
    adaptive_density_band as band,
    adaptive_density_fan as fan,
    angular,
    density_fan_binding as binding,
    density_residual as residual_module,
)
from cftuv_envelope.reference.adaptive_density_atlas import DensityExactWorkBudget
from cftuv_envelope.reference.common import ReferenceGeometryError, settled_result, station_plans_of
from cftuv_envelope.reference.metric import ExactPlanarMetric, _DensityExactMemo

IDENTITY = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
SKEW = ((sp.Integer(2), sp.Rational(1, 3)), (sp.Rational(1, 3), sp.Integer(3)))
CCW = kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION


def _metric(gram=IDENTITY):
    matrix = sp.Matrix(gram).inv()
    inverse = tuple(tuple(matrix[row, column] for column in range(2)) for row in range(2))
    return ExactPlanarMetric(gram, inverse, 1)


def _vector(x, y):
    return angular._density_runtime_vector(x, y)


def _outcome(call):
    try:
        return ("ok", call())
    except Exception as error:  # noqa: BLE001 - исход сравнивается вместе с типом и текстом отказа
        return ("refused", type(error).__name__, getattr(error, "outcome", None), str(error))


# --------------------------------------------------------------------------------------------
# legacy oracle: тело `_subturn` / `_subturn_boundary` до правки (умножение выражений, знак по построенному остатку)
# --------------------------------------------------------------------------------------------


def _legacy_residual(dot, norm_left, norm_right, q):
    norm_product = norm_left * norm_right
    dot_squared = dot * dot
    if q == 3:
        return 4 * dot_squared - norm_product
    if q == 4:
        return 2 * dot_squared - norm_product
    if q == 5:
        return 8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
    if q == 6:
        return 4 * dot_squared - 3 * norm_product
    raise fan.AdaptiveDensityFanInvalid("unsupported Density q")


def _legacy_subturn(metric, left, right, q):
    dot = fan._dual_dot(metric, left, right)
    dot_sign = fan._sign(dot, metric)
    if q == 2:
        return dot_sign >= 0
    if dot_sign <= 0:
        return False
    residual = _legacy_residual(
        dot, fan._dual_dot(metric, left, left), fan._dual_dot(metric, right, right), q
    )
    return fan._sign(residual, metric) >= 0


def _legacy_subturn_boundary(metric, left, right, q):
    dot = fan._dual_dot(metric, left, right)
    if fan._sign(dot, metric) < 0:
        return False
    norm_product = fan._dual_dot(metric, left, left) * fan._dual_dot(metric, right, right)
    dot_squared = dot * dot
    if q == 2:
        residual = dot
    elif q == 3:
        residual = 4 * dot_squared - norm_product
    elif q == 4:
        residual = 2 * dot_squared - norm_product
    elif q == 5:
        residual = 8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
    elif q == 6:
        residual = 4 * dot_squared - 3 * norm_product
    else:
        raise fan.AdaptiveDensityFanInvalid("unsupported Density q")
    return fan._sign(residual, metric) == 0


def _covector_pairs(gram, outgoing, count, extra_offset=None):
    """Пары соседних ковекторов идеального веера одного угла - то, что `_subturn` получает в компиляции."""

    metric = _metric(gram)
    ideal = angular._huber_density_interpolated_normals(metric, _vector(1, 0), _vector(*outgoing), count, 1)
    covectors = fan._covectors(metric, ideal)
    return metric, [(covectors[index], covectors[index + 1]) for index in range(len(covectors) - 1)]


DECIDED: list = []
ANGLES = (
    (0, 1),  # 90 градусов: q=4 ровно на пределе
    (-1, 1),  # 135
    (-3, 1),
    (1, 1),  # 45
    (1, 3),
    (2, 5),
    (-7, 2),
    (-1000, 3),
    (3, 4),
    (-1, 2),
    (1, 2),
)


# --------------------------------------------------------------------------------------------
# squared
# --------------------------------------------------------------------------------------------


def _sample_sums():
    root = sp.sqrt(14256196423567529)
    half = sp.cos(sp.atan(sp.Rational(119399315, 1048)) / 2)
    other = sp.sin(sp.atan(sp.Rational(119399315, 1048)) / 2)
    return (
        sp.Rational(3, 7),
        root,
        half,
        root * half,
        half + other,
        sp.Rational(1, 3) * root * half - sp.Rational(2, 5) * other,
        half + other + sp.sqrt(2),
        sp.Add(sp.sqrt(2), sp.Rational(1, 5), evaluate=False),
        sp.sqrt(2) + sp.sqrt(3),
        half * sp.sqrt(5) - other,
        sp.cos(sp.atan2(sp.sqrt(3), sp.Rational(1, 7)) / 3) + sp.Rational(1, 9),
    )


@pytest.mark.parametrize("value", _sample_sums(), ids=lambda item: sp.srepr(item)[:40])
def test_squared_is_the_object_the_product_builds(value):
    expected = value * value
    actual = band.squared(value)
    assert actual == expected
    assert sp.srepr(actual) == sp.srepr(expected)
    assert actual.func is expected.func and actual.args == expected.args
    assert hash(actual) == hash(expected)
    assert band.squared(value) == actual


@pytest.mark.parametrize("infinite", (sp.oo, -sp.oo, sp.zoo, sp.nan))
def test_squared_leaves_sums_with_a_non_rational_number_to_the_old_product(infinite):
    value = sp.Add(infinite, sp.sqrt(2), evaluate=False)
    assert sp.srepr(band.squared(value)) == sp.srepr(value * value)


# --------------------------------------------------------------------------------------------
# residual_sign
# --------------------------------------------------------------------------------------------


@pytest.mark.parametrize("gram", (IDENTITY, SKEW), ids=("identity", "skew"))
@pytest.mark.parametrize("q", (3, 4, 5, 6))
@pytest.mark.parametrize("count", (1, 2))
def test_residual_sign_is_the_sign_of_the_built_residual(gram, q, count):
    decided = 0
    for outgoing in ANGLES:
        metric, pairs = _covector_pairs(gram, outgoing, count)
        reference = _metric(gram)
        for left, right in pairs:
            dot = fan._dual_dot(metric, left, right)
            norm_left = fan._dual_dot(metric, left, left)
            norm_right = fan._dual_dot(metric, right, right)
            expected = _outcome(
                lambda: angular._density_exact_sign(_legacy_residual(dot, norm_left, norm_right, q), reference)
            )
            actual = _outcome(lambda: residual_module.residual_sign(metric, dot, norm_left, norm_right, q))
            assert actual == expected
            filtered = residual_module._filtered_sign(metric, dot, norm_left, norm_right, q)
            if filtered is not None:
                decided += 1
                assert expected == ("ok", filtered)
    DECIDED.append(decided)


@pytest.mark.parametrize("q", (3, 4, 5, 6))
def test_residual_filter_stays_silent_where_the_enclosure_cannot_clear_zero(q):
    """Остаток ровно ноль и остаток на 2^-200 от нуля: фильтр молчит, ответ или отказ - прежние."""

    cos_limit = {3: sp.Rational(1, 2), 4: sp.sqrt(2) / 2, 5: (1 + sp.sqrt(5)) / 4, 6: sp.sqrt(3) / 2}[q]
    metric = _metric()
    for offset in (0, sp.Rational(1, 2**20), sp.Rational(1, 2**100), sp.Rational(1, 2**200), -sp.Rational(1, 2**200)):
        dot = sp.Add(cos_limit, offset, evaluate=False) if offset != 0 else sp.Add(cos_limit, sp.sqrt(7) - sp.sqrt(7), evaluate=False)
        one = sp.Integer(1)
        expected = _outcome(lambda: angular._density_exact_sign(_legacy_residual(dot, one, one, q), _metric()))
        actual = _outcome(lambda: residual_module.residual_sign(metric, dot, one, one, q))
        assert actual == expected
        verdict = residual_module._filtered_sign(metric, dot, one, one, q)
        if verdict is not None:
            assert expected == ("ok", verdict)
        if offset in (0, sp.Rational(1, 2**200), -sp.Rational(1, 2**200)):
            assert verdict is None


@pytest.mark.parametrize("gram", (IDENTITY, SKEW), ids=("identity", "skew"))
@pytest.mark.parametrize("q", (2, 3, 4, 5, 6, 7))
def test_subturn_and_boundary_keep_the_legacy_answers(gram, q):
    for outgoing in ANGLES:
        metric, pairs = _covector_pairs(gram, outgoing, 1)
        reference, _ = _covector_pairs(gram, outgoing, 1)
        for left, right in pairs:
            assert _outcome(lambda: fan._subturn(metric, left, right, q)) == _outcome(
                lambda: _legacy_subturn(reference, left, right, q)
            )
            assert _outcome(lambda: fan._subturn_boundary(metric, left, right, q)) == _outcome(
                lambda: _legacy_subturn_boundary(reference, left, right, q)
            )


def test_residual_filter_leaves_the_working_precision_as_it_found_it():
    metric, pairs = _covector_pairs(IDENTITY, (-3, 1), 1)
    left, right = pairs[0]
    saved = iv.prec
    try:
        for precision in (53, 160, 256):
            iv.prec = precision
            fan._subturn(metric, left, right, 4)
            assert iv.prec == precision
    finally:
        iv.prec = saved


# --------------------------------------------------------------------------------------------
# bind_density_fan
# --------------------------------------------------------------------------------------------


def _ideal(metric, outgoing=(1, 2)):
    return angular._huber_density_interpolated_normals(metric, _vector(1, 0), _vector(*outgoing), 1, 1)


def test_bind_density_fan_takes_a_repeat_from_memory_and_recomputes_every_other_input(monkeypatch):
    calls = []
    real = binding.certify_huber_density_bindings_with_adaptive_fallback

    def counted(*args, **kwargs):
        calls.append(args[3])
        return real(*args, **kwargs)

    monkeypatch.setattr(binding, "certify_huber_density_bindings_with_adaptive_fallback", counted)
    metric = _metric()
    ideal = _ideal(metric)
    reasons = (None,)

    def bind(ideal=ideal, q=4, reasons=reasons, law=band.WINDOW_LAW_VORONOI, lifted=False, refusals=None):
        return binding.bind_density_fan(
            metric, ideal, CCW, q, reasons, lifted=lifted, law=law, spec_id="spec", refusals=[] if refusals is None else refusals
        )

    first = bind()
    assert bind() is first
    assert bind(ideal=tuple(list(ideal))) is first  # значения те же, объект кортежа другой
    assert len(calls) == 1
    other = bind(q=5)
    assert other is not first and len(calls) == 2
    assert bind(ideal=_ideal(metric, (2, 3))) is not first and len(calls) == 3
    assert bind(reasons=(DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL,)) is not first and len(calls) == 4


def test_bind_density_fan_memory_separates_the_turn_orientation():
    clockwise = kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION

    def bind(metric, orientation):
        return _outcome(lambda: binding.bind_density_fan(
            metric, _ideal(metric), orientation, 4, (None,), lifted=True, law=band.WINDOW_LAW_VORONOI, spec_id="s", refusals=[]
        ))

    metric = _metric()
    counter_clockwise = bind(metric, CCW)
    assert bind(metric, clockwise) == bind(_metric(), clockwise)  # память не отдаёт ответ другой ориентации
    assert bind(metric, CCW) == counter_clockwise


def test_bind_density_fan_does_not_remember_a_refusal(monkeypatch):
    attempts = []

    def narrow(metric, ideal, orientation, q, reasons, window_law):
        attempts.append(window_law)
        raise fan.AdaptiveDensityFanInvalid("band refused")

    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", narrow)
    monkeypatch.setattr(
        binding,
        "certify_huber_density_bindings_with_adaptive_fallback",
        lambda *args, **kwargs: ((None,), "authority"),
    )
    metric = _metric()
    ideal = _ideal(metric)
    refusals = []
    results = [
        binding.bind_density_fan(
            metric, ideal, CCW, 4, (None,), lifted=False, law=band.WINDOW_LAW_NARROW_BAND, spec_id=name, refusals=refusals
        )
        for name in ("a", "b")
    ]
    assert results[0] is results[1]
    assert len(attempts) == 2  # отказ полосы повторяется для каждого угла
    assert [item[0] for item in refusals] == ["a", "b"]  # и записывается за каждым
    assert all(item[1] == "AdaptiveDensityFanInvalid: band refused" for item in refusals)


def test_bind_density_fan_result_equals_the_direct_certification():
    metric = _metric()
    ideal = _ideal(metric)
    direct = _outcome(lambda: kernel.canonical_json_bytes(fan.certify_adaptive_density_fan(
        _metric(), ideal, CCW, 4, binding_reasons=(None,)
    )))
    bound = binding.bind_density_fan(
        metric, ideal, CCW, 4, (None,), lifted=True, law=band.WINDOW_LAW_VORONOI, spec_id="s", refusals=[]
    )
    assert _outcome(lambda: kernel.canonical_json_bytes(bound[1])) == direct
    assert bound[0] == (None,) * (len(ideal) - 2)


# --------------------------------------------------------------------------------------------
# settled_result / station plans
# --------------------------------------------------------------------------------------------


def test_settled_result_is_keyed_by_identity_and_keeps_its_inputs_alive():
    memo = _DensityExactMemo()
    first, second = object(), object()
    calls = []

    def compute(value):
        return lambda: (calls.append(value), value)[1]

    assert settled_result(memo, "n", (first,), compute("a")) == "a"
    assert settled_result(memo, "n", (first,), compute("b")) == "a"  # тот же вход - из памяти
    assert settled_result(memo, "n", (second,), compute("c")) == "c"  # другой объект - заново
    assert settled_result(memo, "other", (first,), compute("d")) == "d"  # другое имя - заново
    assert settled_result(None, "n", (first,), compute("e")) == "e"  # без памяти - прямое вычисление
    assert calls == ["a", "c", "d", "e"]
    assert all(entry[0][0] in (first, second) for entry in memo.settled.values())


def test_settled_result_does_not_remember_a_refusal():
    memo = _DensityExactMemo()
    subject = object()

    def refuse():
        raise ReferenceGeometryError(kernel.ReferenceOutcome.CORNER_TREATMENT_INVALID, "no")

    for _ in range(2):
        with pytest.raises(ReferenceGeometryError):
            settled_result(memo, "n", (subject,), refuse)
    assert not memo.settled


def test_new_transaction_memos_are_process_local():
    memo = _DensityExactMemo()
    memo.fan_bindings[("key",)] = ("value",)
    memo.settled[("n", 1)] = ((object(),), "result")
    restored = pickle.loads(pickle.dumps(memo))
    assert not restored.fan_bindings and not restored.settled


FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
COMPILE_CASES = (
    ("binding_noise_canonical_v1/building_patch114", "decal_request_density4.json"),
    ("density4_exact_limit_v1/mesh2_patch2", "decal_request_density4.json"),
    ("wall_noise_top_join_density4_v1/wall_noise_top_patch1", "decal_request.json"),
)


def _fixture(folder, request_name):
    path = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((path / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((path / request_name).read_bytes())
    return snapshot, request


@pytest.mark.parametrize("folder,request_name", COMPILE_CASES)
def test_station_plan_errors_with_a_shared_plan_equal_the_recomputation(folder, request_name):
    snapshot, request = _fixture(folder, request_name)
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id
    plans = _chain_station.chain_station_plans(snapshot, domain_id)
    memo = _DensityExactMemo()
    shared = station_plans_of(snapshot, domain_id, memo)
    assert shared == plans and station_plans_of(snapshot, domain_id, memo) is shared
    assert _chain_station.plan_errors(snapshot, domain_id, plans) == ()
    assert _chain_station.plan_errors(snapshot, domain_id, plans, recompute=lambda: shared) == ()
    if len(plans) > 1:  # один оставшийся план - уже не пустой: недостающий обязан быть назван, а не пропущен
        forged = frozenset(sorted(plans, key=lambda item: item.physical_chain_id.value)[1:])
        mutated = _chain_station.plan_errors(snapshot, domain_id, forged)
        assert mutated and _chain_station.plan_errors(snapshot, domain_id, forged, recompute=lambda: shared) == mutated


@pytest.mark.parametrize("folder,request_name", COMPILE_CASES)
def test_fixture_compilation_is_the_same_with_every_dedup_switched_off(folder, request_name, monkeypatch):
    snapshot, request = _fixture(folder, request_name)
    enabled = kernel.canonical_json_bytes(kernel.compile_reference_envelopes(snapshot, request))
    assert enabled == kernel.canonical_json_bytes(kernel.compile_reference_envelopes(snapshot, request))

    class Forgetful(dict):
        def __setitem__(self, key, value):
            return None

    original = _DensityExactMemo.__init__

    def forgetful(self):
        original(self)
        self.fan_bindings = Forgetful()
        self.settled = Forgetful()

    monkeypatch.setattr(_DensityExactMemo, "__init__", forgetful)
    monkeypatch.setattr(residual_module, "_filtered_sign", lambda *args: None)
    monkeypatch.setattr(band, "squared", lambda value: value * value)
    monkeypatch.setattr(residual_module, "squared", lambda value: value * value)
    disabled = kernel.canonical_json_bytes(kernel.compile_reference_envelopes(snapshot, request))
    assert disabled == enabled


# --------------------------------------------------------------------------------------------
# _search: пустая высота
# --------------------------------------------------------------------------------------------


def _legacy_search(records, sealed_intervals, stop_height, budget, shell, best_fan):
    candidates = [set() for _ in records]
    previous_counts = tuple(0 for _ in records)
    for height in range(1, stop_height + 1):
        for ordinal, record in enumerate(records, start=1):
            candidates[ordinal - 1].update(shell(ordinal, record, sealed_intervals[ordinal - 1], height, budget))
        fan_value = best_fan(candidates, budget) if all(candidates) else None
        if fan_value is not None:
            return height, fan_value, previous_counts, tuple(len(items) for items in candidates)
        previous_counts = tuple(len(items) for items in candidates)
    return None, None, previous_counts, tuple(len(items) for items in candidates)


def _stub_shell(ordinal, record, sealed_interval, height, budget):
    """Построчная копия платы `_farey_shell_candidates`: границы, probe, кандидаты внутри внутреннего окна."""

    outer_lower, inner_lower, inner_upper, outer_upper = sealed_interval
    first, last = fan._strict_numerator_bounds(outer_lower, outer_upper, height)
    budget.spend_shell_probes(max(last - first + 1, 1))
    if first > last:
        return ()
    return tuple(
        (numerator, height)
        for numerator in range(first, last + 1)
        if gcd(abs(numerator), height) == 1 and inner_lower < Fraction(numerator, height) < inner_upper
    )


def _stub_best_fan(candidates, budget):
    budget.spend_order_steps(sum(len(items) for items in candidates) + 1)
    return None if sum(len(items) for items in candidates) < 3 else tuple(sorted(items)[0] for items in map(sorted, candidates))


INTERVALS = (
    (Fraction(1, 3), Fraction(5, 12), Fraction(9, 20), Fraction(1, 2)),
    (Fraction(-1, 1000), Fraction(0), Fraction(1, 997), Fraction(1, 500)),
    (Fraction(7, 3), Fraction(7, 3) + Fraction(1, 100), Fraction(7, 3) + Fraction(1, 80), Fraction(7, 3) + Fraction(1, 50)),
)


@pytest.mark.parametrize("cap", (1 << 17, 400, 37))
@pytest.mark.parametrize("stop", (0, 1, 25, 300))
def test_search_pays_for_an_empty_height_exactly_as_before(cap, stop, monkeypatch):
    records = [(True, 1), (False, -1), (True, 1)]

    def run(search):
        budget = DensityExactWorkBudget(fan.DensityRationalAuthorityExhausted, cap)
        outcome = _outcome(lambda: search(budget))
        return outcome, budget.counters()

    monkeypatch.setattr(fan, "_farey_shell_candidates", lambda metric, ideal, ordinal, q, record, sealed, height, budget: _stub_shell(ordinal, record, sealed, height, budget))
    monkeypatch.setattr(fan, "_best_fan", lambda metric, ideal, orientation, q, candidates, budget: _stub_best_fan(candidates, budget))
    expected = run(lambda budget: _legacy_search(records, INTERVALS, stop, budget, _stub_shell, _stub_best_fan))
    actual = run(lambda budget: fan._search(None, None, CCW, 4, records, INTERVALS, stop, budget))
    assert actual == expected
    if cap == 37 and stop >= 25:
        assert expected[0][0] == "refused"  # кап пересечён: текст отказа несёт счётчики в точке пересечения


def test_the_residual_filter_decided_something_on_the_irrational_corners():
    assert sum(DECIDED) > 0
