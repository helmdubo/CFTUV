"""EFFECTIVE_ALPHA без круга через sympy и текст (срез S1 плана RadicalSumV2): тот же ответ, те же счета, иная проволока.

Контакты пары `(источник, отрезок границы)` считались так: `point_sub` (разбор текста в sympy, вычитание, `factor(cancel(...))` на
иррациональном, обратно в текст), затем `natives()` читал этот текст. На рациональной паре (поле: 100 % пар) весь круг давал те же
`Fraction`, что их вычитание. Здесь проверяется, что быстрый путь ДАЁТ ТЕ ЖЕ ОБЪЕКТЫ, а не «равные по значению»:

1. рациональная разность точек — те же члены `terms`, что у текстовой; иррациональная и вне поля идёт прежним `point_sub` (его текст
   назначает представителя класса, и порядок вычитания его бы изменил: `terms` → оболочка → окно шага);
2. левый рациональный множитель `scaled` — те же члены, что у `*`; равенство значений `_same_value` — тот же предикат, что и разность;
3. приращение счётчиков пары ложится в `BACKEND_COUNTS` в прежнем порядке и при отказе, и при прерывании пары исключением вне `NativeExactError`;
4. память подготовки пиклится целыми и строками (никаких `Fraction`, `RadicalSumV1`, `ExactPlanarPoint` в потоке) и читается обратно в те же
   члены; пикл прежнего формата (объекты) читается как раньше;
5. резервный sympy-путь обходит кандидатов в порядке вставки: порядок равных по alpha контактов не зависит от `PYTHONHASHSEED`.
"""

from __future__ import annotations

import json
import pickle
import pickletools
import random
import sys
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope.numeric import LocalLengthV1
from cftuv_envelope.reference import boundary as boundary_module
from cftuv_envelope.reference import boundary_native
from cftuv_envelope.reference import native_exact as nx
from cftuv_envelope.reference import symbolic_backend as sb
from cftuv_envelope.reference.boundary import (
    ContactCandidatesMemoV1,
    _contact_candidates,
    _entries_wire,
    _length_of,
    _not_above,
    resolve_component_alphas,
)
from cftuv_envelope.reference.boundary_native import SourceContactFrame, _same_value, points_difference
from cftuv_envelope.reference.metric import _product
from cftuv_envelope.reference.planar_types import (
    SYMBOLIC_FALLBACK_COUNTS,
    ExactPlanarPoint,
    ExactScalar,
    exact_sign,
    point_sub,
)

from test_contact_candidates_memo import ALPHAS, CONCAVE, _concave, _geometry, _hole_ring, _sources
from test_contact_prefilter import FIELD_CASES, _field_preparation, _lattice_point, _segment
from test_exact_scalar_text_canon import _run_kernel_child
from reference_factories import straight_snapshot

R = nx.RadicalSumV1


@pytest.fixture(autouse=True)
def _own_counters():
    saved = dict(sb.BACKEND_COUNTS), dict(SYMBOLIC_FALLBACK_COUNTS)
    yield
    sb.BACKEND_COUNTS.clear()
    sb.BACKEND_COUNTS.update(saved[0])
    SYMBOLIC_FALLBACK_COUNTS.update(saved[1])


def _snapshot():
    return dict(sb.BACKEND_COUNTS), dict(SYMBOLIC_FALLBACK_COUNTS)


def _delta(before):
    backend = {k: v - before[0].get(k, 0) for k, v in sb.BACKEND_COUNTS.items() if v != before[0].get(k, 0)}
    symbolic = {k: v - before[1][k] for k, v in SYMBOLIC_FALLBACK_COUNTS.items() if v != before[1][k]}
    return backend, symbolic


def _corpus(seed=20261011, size=120):
    """Величины с разными представителями класса, нуль, рациональные, двух- и трёхклассовые."""

    rng = random.Random(seed)
    values = [R.zero(), R.rational(1), R.rational(-3), R.rational(Fraction(5, 7))]
    for _ in range(size):
        terms = {}
        for _ in range(rng.choice((1, 1, 2, 3))):
            radicand = rng.choice((1, 1, 2, 3, 5, 6, 8, 12, 18, 50, 2 * 49, 3 * 25, 10 ** 12 + 39))
            terms[radicand] = terms.get(radicand, Fraction(0)) + Fraction(rng.randint(-9, 9), rng.randint(1, 6))
        value = R.zero()
        for radicand, coefficient in terms.items():
            value = value + R.sqrt_of_rational(radicand).scaled(coefficient)
        values.append(value)
    return values


# --------------------------------------------------------------------------
# 1. Разность точек, произведение, равенство значений
# --------------------------------------------------------------------------


def test_the_rational_difference_has_the_terms_of_the_text_route_and_the_rest_is_the_text_route_itself():
    rng = random.Random(7)

    def point(x, y):
        return ExactPlanarPoint.from_values(x, y)

    for _ in range(200):
        left = point(Fraction(rng.randint(-50, 50), rng.randint(1, 9)), Fraction(rng.randint(-50, 50), rng.randint(1, 9)))
        right = point(Fraction(rng.randint(-50, 50), rng.randint(1, 9)), Fraction(rng.randint(-50, 50), rng.randint(1, 9)))
        before = _snapshot()
        got = points_difference(left, right)
        assert _delta(before) == ({}, {}), "the rational difference touches no counter"
        want = point_sub(left, right).natives()
        assert type(got) is tuple
        assert tuple(item.terms for item in got) == tuple(item.terms for item in want)
    irrational = point(sp.sqrt(2), sp.Rational(1, 3))
    rational = point(sp.Rational(1, 2), sp.Rational(-2))
    for pair in ((irrational, rational), (rational, irrational), (irrational, irrational)):
        before = _snapshot()
        got = points_difference(*pair)
        after_new = _delta(before)
        before = _snapshot()
        want = point_sub(*pair)
        assert got == want
        assert after_new == _delta(before), "the irrational route keeps the counters of the old route"


def test_the_difference_of_points_outside_the_native_field_is_the_old_vector():
    outside = ExactPlanarPoint.from_values(sp.pi, sp.Integer(1))
    inside = ExactPlanarPoint.from_values(sp.Integer(2), sp.Integer(3))
    assert points_difference(outside, inside) == point_sub(outside, inside)


def test_a_rational_left_factor_is_scaled_with_the_same_terms_as_the_product():
    values = _corpus()
    checked = 0
    for left in values:
        for right in values[:40]:
            assert _product(left, right).terms == (left * right).terms
            checked += 1
    assert checked > 4000


def test_same_value_is_the_difference_test():
    values = _corpus(size=70)
    alternates = [R(tuple((radicand * 49, coefficient / 7) for radicand, coefficient in value.terms if radicand != 1) + tuple(item for item in value.terms if item[0] == 1)) for value in values]
    different = same = 0
    for left in values:
        for right in values + alternates:
            expected = not (left - right).terms
            assert _same_value(left, right) is expected
            different += not expected
            same += expected
    assert same > 70 and different > 1000, "the corpus must contain both equal and different pairs, and other representatives"


# --------------------------------------------------------------------------
# 2. Лист и длина источника: быстрый путь против прежнего текстового
# --------------------------------------------------------------------------


def _outputs(context, domain, sources, extra_boundaries=()):
    """Все контакты всех пар `(источник, граница)`: члены `terms` без пересчёта в значения, тексты точек, приращения счетчиков."""

    out = []
    for source in sources:
        frame = SourceContactFrame(context, source)
        for boundary in (*domain.blocking_segments, *extra_boundaries):
            before = _snapshot()
            try:
                result = _contact_candidates(context, source, boundary, frame)
                shown = tuple(
                    (a.terms, s.terms, p.x.expression, p.y.expression) if type(a) is R else (sp.srepr(a), sp.srepr(s), p.x.expression, p.y.expression)
                    for a, s, p in result
                )
            except ValueError as error:
                shown = ("raised", type(error).__name__, str(error))
            out.append((shown, _delta(before)))
    return out


def _geometries():
    yield "hole-ring", _geometry(*_hole_ring())
    yield "concave", _geometry(*_concave())
    for a, b, c, d in ((1, 0, 1, 1), (2, 1, 1, 2)):
        def transform(point, a=a, b=b, c=c, d=d):
            x, y = point
            return a * x + b * y + 0.125, c * x + d * y - 0.25

        snapshot, request = straight_snapshot(
            faces=tuple(tuple(map(transform, face)) for face in CONCAVE),
            source_routes=({"name": "source", "points": tuple(map(transform, ((0, 0), (10, 0))))},),
            alpha="20",
        )
        yield f"skewed-{a}{b}{c}{d}", _geometry(snapshot, request)
    for name, request_name in (FIELD_CASES[0], FIELD_CASES[3]):
        prepared = _field_preparation(name, request_name)
        yield name, (prepared.context, prepared.domain)


def test_the_fast_leaf_answers_with_the_same_objects_and_counters_as_the_text_route(monkeypatch):
    compared = pairs_with_contacts = irrational_frames = 0
    for name, (context, domain) in _geometries():
        sources = list(_sources(context))
        # отрезки решётки вокруг источника: их концы иррациональны там, где иррациональна нормаль
        lattice = []
        rng = random.Random(11)
        for source in sources[:2]:
            points = [
                _lattice_point(source, Fraction(rng.randint(-3, 5), rng.randint(1, 3)), Fraction(rng.randint(-2, 4), rng.randint(1, 3)))
                for _ in range(10)
            ]
            lattice += [_segment(points[i], points[(i * 3 + 1) % 10]) for i in range(10) if points[i] != points[(i * 3 + 1) % 10]]
        modes = (sb.SymbolicBackendV1.NATIVE_EXACT, sb.SymbolicBackendV1.SHADOW) if name in ("hole-ring", "concave") else (sb.SymbolicBackendV1.NATIVE_EXACT,)
        for mode in modes:
            with sb.symbolic_backend(mode, sb.DisagreementPolicyV1.RECORD):
                fast = _outputs(context, domain, sources, lattice)
                with monkeypatch.context() as patch:
                    patch.setattr(boundary_native, "_rational_pair", lambda point: None)
                    text = _outputs(context, domain, sources, lattice)
            assert fast == text, (name, mode.value)
            compared += len(fast)
            pairs_with_contacts += sum(1 for shown, _ in fast if shown and shown[0] != "raised")
        irrational_frames += any(SourceContactFrame(context, s).ready().length.as_rational() is None for s in sources)
    assert compared > 800 and pairs_with_contacts > 100, (compared, pairs_with_contacts)
    assert irrational_frames >= 2, "the skewed metrics give irrational lengths: the frames are not all rational"


def test_the_source_length_is_the_old_value_and_the_old_counters_on_rational_and_irrational_ends():
    def old_length_of(context, direction):
        """`boundary._length_of` до S1 дословно."""

        if sb.backend_mode() is sb.SymbolicBackendV1.NATIVE_EXACT:
            try:
                return context.metric.length_g_native(direction)
            except nx.NativeExactError as refusal:
                sb.count("source_length", "sign_undecided" if isinstance(refusal, nx.NativeSignUndecided) else "outside_field")
        return context.metric.length_g(direction)

    checked = 0
    for name, (context, domain) in _geometries():
        for source in _sources(context):
            ends = [source]
            for boundary in domain.blocking_segments[:6]:
                ends.append(replace(source, start=boundary.segment.start, end=boundary.segment.end))
            for variant in ends:
                for mode in sb.SymbolicBackendV1:
                    with sb.symbolic_backend(mode, sb.DisagreementPolicyV1.RECORD):
                        before = _snapshot()
                        try:
                            old = old_length_of(context, point_sub(variant.end, variant.start))
                        except ValueError as error:
                            old = ("raised", str(error))
                        want = _delta(before)
                        before = _snapshot()
                        try:
                            new = _length_of(context, variant)
                        except ValueError as error:
                            new = ("raised", str(error))
                        assert _delta(before) == want, (name, mode.value)
                        if type(old) is R:
                            assert type(new) is R and new.terms == old.terms
                        elif isinstance(old, tuple):
                            assert new == old
                        else:
                            assert old == new
                        checked += 1
    assert checked > 100


# --------------------------------------------------------------------------
# 3. Приращение счётчиков пары
# --------------------------------------------------------------------------


def _pair_that_passes_the_prefilter():
    prepared = _field_preparation(*FIELD_CASES[0])
    context, domain = prepared.context, prepared.domain
    for source in _sources(context):
        frame = SourceContactFrame(context, source)
        for boundary in domain.blocking_segments:
            if not frame.excludes(boundary):
                return context, source, frame, boundary, domain
    raise AssertionError("no pair passes the prefilter")


def _pair_that_the_prefilter_rejects():
    prepared = _field_preparation(*FIELD_CASES[0])
    context, domain = prepared.context, prepared.domain
    for source in _sources(context):
        frame = SourceContactFrame(context, source)
        for boundary in domain.blocking_segments:
            if frame.excludes(boundary):
                return context, source, frame, boundary
    raise AssertionError("no pair is rejected by the prefilter")


@pytest.mark.parametrize(
    "error,key",
    (
        (nx.NativeSignUndecided("injected"), "contact_candidates.sign_undecided"),
        (nx.OutsideNativeField("injected"), "contact_candidates.outside_field"),
    ),
)
def test_a_refusal_after_the_prefilter_counts_passed_then_the_refusal_before_the_sympy_path(monkeypatch, error, key):
    context, source, frame, boundary, _ = _pair_that_passes_the_prefilter()
    sb.reset_backend_counts()
    expected = boundary_module._contact_candidates_sympy(context, source, boundary)
    fallback = dict(sb.BACKEND_COUNTS)

    def refuse(*args, **kwargs):
        raise error

    monkeypatch.setattr(boundary_module, "contact_candidates_native", refuse)
    sb.reset_backend_counts()
    got = _contact_candidates(context, source, boundary, frame)
    assert got == expected
    assert list(sb.BACKEND_COUNTS)[:2] == ["contact_candidates.prefilter_passed", key], "the keys appear in the old order, before the sympy path"
    assert dict(sb.BACKEND_COUNTS) == {"contact_candidates.prefilter_passed": 1, key: 1, **fallback}


def test_an_abort_by_another_exception_leaves_the_prefilter_count_like_the_old_code(monkeypatch):
    context, source, frame, boundary, _ = _pair_that_passes_the_prefilter()

    def crash(*args, **kwargs):
        raise AssertionError("injected")

    monkeypatch.setattr(boundary_module, "contact_candidates_native", crash)
    sb.reset_backend_counts()
    with pytest.raises(AssertionError):
        _contact_candidates(context, source, boundary, frame)
    assert dict(sb.BACKEND_COUNTS) == {"contact_candidates.prefilter_passed": 1}


def test_a_rejected_pair_counts_the_rejection_then_native_and_never_calls_the_leaf(monkeypatch):
    context, source, frame, boundary = _pair_that_the_prefilter_rejects()

    def forbidden(*args, **kwargs):
        raise AssertionError("the leaf must not run")

    monkeypatch.setattr(boundary_module, "contact_candidates_native", forbidden)
    sb.reset_backend_counts()
    assert _contact_candidates(context, source, boundary, frame) == ()
    assert list(sb.BACKEND_COUNTS) == ["contact_candidates.prefilter_rejected", "contact_candidates.native"]
    assert dict(sb.BACKEND_COUNTS) == {"contact_candidates.prefilter_rejected": 1, "contact_candidates.native": 1}


def test_a_successful_pair_counts_passed_then_native():
    context, source, frame, boundary, _ = _pair_that_passes_the_prefilter()
    sb.reset_backend_counts()
    _contact_candidates(context, source, boundary, frame)
    assert list(sb.BACKEND_COUNTS) == ["contact_candidates.prefilter_passed", "contact_candidates.native"]


def test_injected_signum_faults_leave_the_same_counters_in_the_leaf_and_in_the_text_route(monkeypatch):
    """Прерывание ЛЮБОГО `signum` пары (отказ родной арифметики или иное исключение) оставляет одни счета у быстрого и у текстового пути."""

    context, domain = _geometry(*_hole_ring())
    real = R.signum
    outcomes = {}
    for route in ("fast", "text"):
        results = []
        with monkeypatch.context() as patch:
            if route == "text":
                patch.setattr(boundary_native, "_rational_pair", lambda point: None)
            for n in range(1, 25):
                for error in (nx.NativeSignUndecided, nx.OutsideNativeField, AssertionError):
                    calls = [0]

                    def failing(self, n=n, error=error, calls=calls):
                        calls[0] += 1
                        if calls[0] == n:
                            raise error("injected")
                        return real(self)

                    with monkeypatch.context() as inner:
                        inner.setattr(R, "signum", failing)
                        for source in list(_sources(context))[:2]:
                            frame = SourceContactFrame(context, source)
                            for boundary in domain.blocking_segments[:8]:
                                calls[0] = 0
                                before = _snapshot()
                                try:
                                    shown = tuple((a.terms, s.terms, p.x.expression) if type(a) is R else sp.srepr(a) for a, s, p in _contact_candidates(context, source, boundary, frame))
                                except (AssertionError, ValueError) as caught:
                                    shown = ("raised", type(caught).__name__)
                                results.append((n, error.__name__, shown, _delta(before)))
        outcomes[route] = results
    assert outcomes["fast"] == outcomes["text"]
    assert len(outcomes["fast"]) > 800
    raised = sum(1 for item in outcomes["fast"] if item[2][:1] == ("raised",))
    refused = sum(1 for item in outcomes["fast"] if "contact_candidates.outside_field" in item[3][0] or "contact_candidates.sign_undecided" in item[3][0])
    assert raised > 0 and refused > 0, "the injection must reach both kinds of abort"


# --------------------------------------------------------------------------
# 4. Память подготовки: целые и строки
# --------------------------------------------------------------------------

_OBJECT_OPCODES = {"GLOBAL", "STACK_GLOBAL", "REDUCE", "NEWOBJ", "NEWOBJ_EX", "BUILD", "INST", "OBJ"}


def _filled_memo(geometry, alphas=ALPHAS):
    context, domain = geometry
    memo = ContactCandidatesMemoV1()
    for alpha in alphas:
        resolve_component_alphas(context, LocalLengthV1(Decimal(alpha)), domain, memo)
    return memo


def _shown(entries):
    out = {}
    for key, value in entries.items():
        kind = key[0]
        if kind == "contacts":
            value = tuple((a.terms, s.terms, p.x.expression, p.y.expression) if type(a) is R else (sp.srepr(a), sp.srepr(s), p.x.expression, p.y.expression) for a, s, p in value)
        elif kind == "length":
            value = value.terms if type(value) is R else sp.srepr(value)
        elif kind == "blocking":
            value = len(value)
        out[key] = value
    return out


def test_the_memo_pickle_carries_ints_and_strings_only_for_contacts_bounds_and_lengths():
    for name, geometry in (("hole", _geometry(*_hole_ring())), ("field", (lambda p: (p.context, p.domain))(_field_preparation(*FIELD_CASES[1])))):
        memo = _filled_memo(geometry, ("0.5", "2", "4"))
        wire = _entries_wire(memo.entries)
        algebraic = {key: value for key, value in wire.items() if key[0] in ("contacts", "bounds", "length")}
        kinds = {key[0] for key in algebraic}
        assert kinds == {"contacts", "bounds", "length"}, (name, kinds)
        opcodes = {opcode.name for opcode, _, _ in pickletools.genops(pickle.dumps(algebraic, protocol=pickle.HIGHEST_PROTOCOL))}
        assert not opcodes & _OBJECT_OPCODES, (name, sorted(opcodes & _OBJECT_OPCODES))


@pytest.mark.parametrize("mode", (sb.SymbolicBackendV1.NATIVE_EXACT, sb.SymbolicBackendV1.SYMPY))
def test_the_memo_round_trips_through_pickle_with_the_same_terms_and_answer(mode):
    for geometry in (_geometry(*_hole_ring()), _geometry(*_concave())):
        with sb.symbolic_backend(mode):
            memo = _filled_memo(geometry, ("1", "4"))
            clone = pickle.loads(pickle.dumps(memo, protocol=pickle.HIGHEST_PROTOCOL))
            assert list(clone.entries) == list(memo.entries)
            assert _shown(clone.entries) == _shown(memo.entries)
            for key, value in memo.entries.items():
                if key[0] == "bounds":
                    assert clone.entries[key] == value
            context, domain = geometry
            for alpha in ALPHAS:
                value = LocalLengthV1(Decimal(alpha))
                assert resolve_component_alphas(context, value, domain, clone) == resolve_component_alphas(context, value, domain)
            assert (clone.computed, clone.reused) != (0, 0)


def test_a_memo_pickled_in_the_old_format_still_loads_and_answers():
    context, domain = _geometry(*_hole_ring())
    memo = _filled_memo((context, domain), ("1", "4"))

    class _OldFormat:
        def __reduce__(self):
            return (ContactCandidatesMemoV1, (memo.entries,))

    old = pickle.loads(pickle.dumps(_OldFormat()))
    assert _shown(old.entries) == _shown(memo.entries)
    for alpha in ALPHAS:
        value = LocalLengthV1(Decimal(alpha))
        assert resolve_component_alphas(context, value, domain, old) == resolve_component_alphas(context, value, domain)


def test_the_wire_of_a_value_keeps_the_representative_of_every_class():
    for value in _corpus():
        assert nx.value_of_wire(nx.terms_wire(value)).terms == value.terms
    assert nx.terms_wire(R.zero()) == ()
    assert nx.value_of_wire(()).terms == ()


# --------------------------------------------------------------------------
# 5. Сравнение с эффективной alpha без sympy
# --------------------------------------------------------------------------


def test_the_comparison_with_the_effective_alpha_is_the_sign_of_the_sympy_difference():
    rng = random.Random(5)
    checked = equal = 0
    for _ in range(300):
        square = Fraction(rng.choice((2, 3, 8, 14, 65537 ** 2 * 100003)), rng.choice((1, 4, 9, 49)))
        sign = rng.choice((-1, 1))
        alpha = rng.choice((R.sqrt_of_rational(square).scaled(sign), R.rational(Fraction(rng.randint(-9, 9), rng.randint(1, 5)))))
        effective_value = rng.choice((alpha, R.sqrt_of_rational(rng.choice((2, 3, 18, 50))), R.rational(Fraction(rng.randint(0, 9), rng.randint(1, 5)))))
        effective = ExactScalar.canonical(effective_value)
        assert _not_above(alpha, effective) is (exact_sign(alpha - effective.as_expr()) <= 0)
        checked += 1
        equal += not (alpha - effective_value).terms
    assert equal > 20 and checked == 300
    alpha_in_sympy = sp.sqrt(2)
    assert _not_above(alpha_in_sympy, ExactScalar.canonical(R.rational(2))) is True
    assert _not_above(alpha_in_sympy, ExactScalar.canonical(R.rational(1))) is False


# --------------------------------------------------------------------------
# 6. Порядок кандидатов резервного пути не зависит от PYTHONHASHSEED
# --------------------------------------------------------------------------

_TIE_PROBE = """
import json
from types import SimpleNamespace

import sympy as sp

from cftuv_envelope.reference import planar_types as PT, symbolic_backend as SB
from cftuv_envelope.reference.boundary import _contact_candidates
from cftuv_envelope.reference.metric import ExactPlanarMetric

one, zero = sp.Integer(1), sp.Integer(0)
context = SimpleNamespace(metric=ExactPlanarMetric(((one, zero), (zero, one)), ((one, zero), (zero, one)), 1))
point = lambda x, y: PT.ExactPlanarPoint(PT.ExactScalar.from_value(x), PT.ExactScalar.from_value(y))
start, end = point(0, 0), point(4, 0)
tangent = context.metric.unit_g(PT.point_sub(end, start))
source = SimpleNamespace(start=start, end=end, tangent=tangent, owner_normal=context.metric.owner_normal_g(tangent, owner_left=True), support_id="s")
report = []
for root in (2, 3, 5, 6, 7):
    along = sp.sqrt(root) - 3
    boundary = SimpleNamespace(segment=SimpleNamespace(start=point(along, one), end=point(along + 8, one), segment_id="tie"))
    for mode in (SB.SymbolicBackendV1.SYMPY, SB.SymbolicBackendV1.SHADOW, SB.SymbolicBackendV1.NATIVE_EXACT):
        with SB.symbolic_backend(mode, SB.DisagreementPolicyV1.RECORD):
            contacts = _contact_candidates(context, source, boundary)
            report.append([root, mode.value, [p.x.expression for _, _, p in contacts]])
print("RESULT " + json.dumps(report))
"""


def test_tied_contacts_of_the_sympy_path_come_in_insertion_order_for_every_hash_seed(monkeypatch):
    runs = {}
    for seed in ("0", "1", "2", "12345", "99991"):
        monkeypatch.setenv("PYTHONHASHSEED", seed)
        done = _run_kernel_child(_TIE_PROBE)
        line = next(item for item in done.stdout.splitlines() if item.startswith("RESULT "))
        runs[seed] = json.loads(line[len("RESULT "):])
    first = runs["0"]
    assert len(first) == 15
    for _root, _mode, order in first:
        # two valid parameters with equal alpha: the stations 0 and 4, in the order the candidates were inserted
        assert order == ["Integer(0)", "Integer(4)"], (_root, _mode, order)
    for seed, report in runs.items():
        assert report == first, f"PYTHONHASHSEED={seed} changes the order of tied contacts"
