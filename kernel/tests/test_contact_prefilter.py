"""Предфильтр «контактов нет» и данные источника, посчитанные один раз: ответ резолвера тот же, фильтр лишь доказывает пустоту.

Профиль холодной кнопки (`building`, d2): стадия EFFECTIVE_ALPHA — 5.4 с из них 92 % `contact_candidates_native`
(8689 пар «источник, отрезок границы», по ~0.55 мс на дроби); у 57-85 % пар (по мешам) контактов нет вовсе, и считать их
точно — трата. `SourceContactFrame.excludes` доказывает пустоту в binary64 с границей ошибки (оба конца отрезка строго
по недопустимую сторону одной из прямых `станция = 0`, `станция = длина`, `alpha = 0`) и иначе уступает точному пути.
Длина источника и ковекторы `G·t`, `G·n` — функции только источника — считаются один раз на источник, а не на пару.

Что проверяется:

1. ФИЛЬТР НИКОГДА НЕ ОТВЕРГАЕТ ПАРУ С КОНТАКТОМ — на полевых фикстурах (все пары источник x граница), а отвергнутые пары
   пусты и в `sympy`-эталоне. Контроль различения: фильтр отвергает хотя бы половину пустых пар (иначе «ни разу не ошибся»
   значило бы «ни разу не сработал»);
2. то же на решётке точек ВОКРУГ границ полосы (станция 0 и длина, alpha 0), с возмущениями от 1e-2 до 1e-30 и точными
   касаниями, на источниках с иррациональными кадрами: там фильтр обязан уступать, а не угадывать (ему вменяется доказать
   или уступить; число уступок с пустым ответом больше нуля — тест лежит в области шума binary64);
3. ответ `resolve_component_alphas` (резолюции с диагностиками и память контактов) побитово тот же с предфильтром и без;
4. счётчики называют каждую пару: `prefilter_rejected + prefilter_passed == native == число контактных записей памяти`;
5. длина источника считается один раз на источник, а не на пару;
6. число вне binary64 фильтр не берёт (уступка), а точный путь отвечает.
"""

from __future__ import annotations

import random
from decimal import Decimal
from fractions import Fraction

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.numeric import LocalLengthV1
from cftuv_envelope.reference import symbolic_backend as sb
from cftuv_envelope.reference.boundary import (
    ContactCandidatesMemoV1,
    _contact_candidates,
    _contact_candidates_sympy,
    resolve_component_alphas,
)
from cftuv_envelope.reference.boundary_native import (
    SourceContactFrame,
    contact_candidates_native,
)
from cftuv_envelope.reference.domain_geometry import BlockingBoundarySegment, BoundaryRole
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import BoundedSupportSegment, ExactPlanarPoint
from cftuv_envelope.wavefront import ConveyorOutcome, prepare_conveyor

from test_contact_candidates_memo import ALPHAS, _concave, _geometry, _hole_ring, _sources
from wavefront_cases import KERNEL_ROOT

FIXTURES = KERNEL_ROOT / "fixtures"
FIELD_CASES = (
    ("building_002_point_contact_v1", "decal_request.json"),
    ("wall_noise_top_rung_clip_v1", "decal_request.json"),
    ("sagging_wall_convex_partition_v1", "decal_request.json"),
    ("sagging_wall_rung_chord_v1", "decal_request_alpha_0.4.json"),
    ("building_patch89_fold_miter_v1", "decal_request_density1.json"),
    ("building_002_full_selection_v1", "decal_request.json"),
)


def _field_preparation(name: str, request_name: str):
    root = FIXTURES / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / request_name).read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    return prepared


# --------------------------------------------------------------------------
# 1. Полевые фикстуры: ни одной пары с контактом не отвергнуто
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name,request_name", FIELD_CASES, ids=[case[0] for case in FIELD_CASES])
def test_the_prefilter_never_rejects_a_pair_in_contact_on_the_field_fixtures(name, request_name):
    prepared = _field_preparation(name, request_name)
    context = prepared.context
    blocking = prepared.domain.blocking_segments
    pairs = empty = rejected = checked_against_sympy = 0
    for source in _sources(context):
        frame = SourceContactFrame(context, source)
        for boundary in blocking:
            pairs += 1
            exact = contact_candidates_native(context, source, boundary)
            proved = frame.excludes(boundary)
            empty += not exact
            rejected += proved
            assert not (proved and exact), (name, boundary.segment.segment_id, exact)
            if proved and checked_against_sympy < 40:
                # Эталон `sympy` тоже пуст: отвергнутая пара пуста по значению, а не по совпадению двух родных путей.
                assert _contact_candidates_sympy(context, source, boundary) == ()
                checked_against_sympy += 1
    assert pairs >= 30
    assert empty > 0 and rejected > 0, (name, pairs, empty, rejected)
    assert 2 * rejected >= empty, f"{name}: the prefilter proves only {rejected} of {empty} empty pairs"


# --------------------------------------------------------------------------
# 2. Решётка вокруг границ полосы: доказать или уступить
# --------------------------------------------------------------------------

#: Станция `lam * длина` и alpha `mu` точки `S + lam*(E - S) + mu*N` точны (`N` — единичная нормаль в метрике, `(E - S)` —
#: длина на касательную): отсюда точные касания с тремя прямыми и возмущения заданной малости по обе стороны.
_BASE = (Fraction(-2), Fraction(-1), Fraction(0), Fraction(1, 3), Fraction(1), Fraction(2), Fraction(3))
_NOISE = tuple(Fraction(sign, 10 ** power) for power in (2, 8, 13, 16, 30) for sign in (1, -1)) + (Fraction(0),) * 3


def _lattice_point(source, lam: Fraction, mu: Fraction) -> ExactPlanarPoint:
    sx, sy = source.start.expressions()
    ex, ey = source.end.expressions()
    nx, ny = source.owner_normal.expressions()
    lam_s, mu_s = sp.Rational(lam.numerator, lam.denominator), sp.Rational(mu.numerator, mu.denominator)
    return ExactPlanarPoint.from_values(sx + lam_s * (ex - sx) + mu_s * nx, sy + lam_s * (ey - sy) + mu_s * ny)


def _segment(start: ExactPlanarPoint, end: ExactPlanarPoint) -> BlockingBoundarySegment:
    return BlockingBoundarySegment(
        BoundedSupportSegment("lattice", start, end, frozenset(), frozenset(), None, frozenset(), frozenset()),
        BoundaryRole.OUTER,
    )


def _scan_lattice(context):
    """`(отвергнуто, контактов, пустых без доказательства, ложных отвержений)` по случайным отрезкам решётки на двух источниках."""

    rng = random.Random(20261006)
    rejected = contact = yielded_empty = 0
    false_rejections = []
    for source in list(_sources(context))[:2]:
        frame = SourceContactFrame(context, source)
        pool = []
        for _ in range(26):
            pool.append(
                _lattice_point(source, rng.choice(_BASE) + rng.choice(_NOISE), rng.choice(_BASE) + rng.choice(_NOISE))
            )
        # Точные касания: оба конца на одной из трёх прямых.
        for edge in (Fraction(0), Fraction(1)):
            pool.append(_lattice_point(source, edge, Fraction(1, 2)))
            pool.append(_lattice_point(source, edge, Fraction(3)))
        pool.append(_lattice_point(source, Fraction(-1, 2), Fraction(0)))
        pool.append(_lattice_point(source, Fraction(3, 2), Fraction(0)))
        for _ in range(90):
            first, second = rng.sample(pool, 2)
            boundary = _segment(first, second)
            exact = contact_candidates_native(context, source, boundary)
            proved = frame.excludes(boundary)
            if proved and exact:
                false_rejections.append(exact)
            rejected += proved
            contact += bool(exact)
            yielded_empty += (not exact) and not proved
    return rejected, contact, yielded_empty, false_rejections


@pytest.mark.parametrize(
    "name,request_name",
    (FIELD_CASES[0], FIELD_CASES[3]),
    ids=("building_002_point_contact_v1", "sagging_wall_rung_chord_v1"),
)
def test_the_prefilter_proves_or_yields_around_the_strip_boundaries(name, request_name):
    context = _field_preparation(name, request_name).context
    rejected, contact, yielded_empty, false_rejections = _scan_lattice(context)
    assert not false_rejections, (name, false_rejections[:1])
    # Все три исхода встречены: доказанная пустота, контакт, пустота без доказательства (шум binary64 и угловые случаи).
    assert rejected > 20 and contact > 10 and yielded_empty > 0, (name, rejected, contact, yielded_empty)


def test_the_lattice_scan_has_the_power_to_catch_a_filter_without_error_bounds(monkeypatch):
    """Отрицательный контроль: тот же обход ловит фильтр, у которого граница ошибки координат нулевая, - иначе «ни одной ошибки» ничего не доказывало бы."""

    from cftuv_envelope.reference import boundary_native

    real = boundary_native.centre_and_bound

    def without_bound(value):
        found = real(value)
        return None if found is None else (found[0], 0.0)

    monkeypatch.setattr(boundary_native, "centre_and_bound", without_bound)
    context = _field_preparation(*FIELD_CASES[0]).context
    *_, false_rejections = _scan_lattice(context)
    assert false_rejections, "a filter that trusts the rounded centres must reject some pair that has a contact"


def test_the_prefilter_yields_on_exact_ties_with_the_strip_lines():
    prepared = _field_preparation(*FIELD_CASES[0])
    context = prepared.context
    source = next(_sources(context))
    frame = SourceContactFrame(context, source)
    # alpha = 0 у обоих концов (контакт есть: alpha >= 0), станция = 0 у обоих, станция = длина у обоих.
    for lam_a, mu_a, lam_b, mu_b in (
        (Fraction(-1, 2), Fraction(0), Fraction(3, 2), Fraction(0)),
        (Fraction(0), Fraction(1), Fraction(0), Fraction(2)),
        (Fraction(1), Fraction(1), Fraction(1), Fraction(2)),
    ):
        boundary = _segment(_lattice_point(source, lam_a, mu_a), _lattice_point(source, lam_b, mu_b))
        assert contact_candidates_native(context, source, boundary), (lam_a, mu_a, lam_b, mu_b)
        assert frame.excludes(boundary) is False


# --------------------------------------------------------------------------
# 3. Ответ резолвера побитово тот же с предфильтром и без
# --------------------------------------------------------------------------


def _resolve_all(context, domain, alphas):
    memo = ContactCandidatesMemoV1()
    answers = [resolve_component_alphas(context, LocalLengthV1(Decimal(alpha)), domain, memo) for alpha in alphas]
    return answers, memo


def test_the_resolver_answer_is_bitwise_the_same_with_and_without_the_prefilter(monkeypatch):
    cases = [_geometry(*_hole_ring()), _geometry(*_concave())]
    prepared = _field_preparation(*FIELD_CASES[1])
    cases.append((prepared.context, prepared.domain))
    for context, domain in cases:
        with_filter, memo_with = _resolve_all(context, domain, ALPHAS)
        monkeypatch.setattr(SourceContactFrame, "excludes", lambda self, boundary: False)
        without, memo_without = _resolve_all(context, domain, ALPHAS)
        monkeypatch.undo()
        assert with_filter == without
        assert memo_with.entries.keys() == memo_without.entries.keys()
        for key, value in memo_with.entries.items():
            assert value == memo_without.entries[key], key


def test_the_prefilter_changes_only_the_price_in_every_backend_mode(monkeypatch):
    prepared = _field_preparation(*FIELD_CASES[0])
    context, domain = prepared.context, prepared.domain
    alphas = ("0.25", "1", "4")
    for mode in sb.SymbolicBackendV1:
        sb.reset_backend_counts()
        with sb.symbolic_backend(mode, sb.DisagreementPolicyV1.RECORD):
            with_filter, _ = _resolve_all(context, domain, alphas)
            counts = dict(sb.BACKEND_COUNTS)
            monkeypatch.setattr(SourceContactFrame, "excludes", lambda self, boundary: False)
            without, _ = _resolve_all(context, domain, alphas)
            monkeypatch.undo()
        assert with_filter == without, mode.value
        assert not sb.DISAGREEMENTS, (mode.value, sb.DISAGREEMENTS[:2])
        if mode is sb.SymbolicBackendV1.SYMPY:
            # Откат на прежний путь целиком: ни предфильтра, ни родных контактов.
            assert not any(key.startswith("contact_candidates.prefilter") for key in counts)
        else:
            assert counts["contact_candidates.prefilter_rejected"] > 0, mode.value
        if mode is sb.SymbolicBackendV1.SHADOW:
            # Пустота, доказанная фильтром, сверена с `sympy`-эталоном этой же пары.
            assert counts["contact_candidates.shadow_checked"] > 0


# --------------------------------------------------------------------------
# 4. Счётчики называют каждую пару
# --------------------------------------------------------------------------


def test_the_counters_name_every_pair_and_the_price_is_a_function_of_the_input():
    prepared = _field_preparation(*FIELD_CASES[3])
    context, domain = prepared.context, prepared.domain
    runs = []
    for _ in range(2):
        sb.reset_backend_counts()
        _, memo = _resolve_all(context, domain, ("0.4", "0.2"))
        rejected = sb.BACKEND_COUNTS["contact_candidates.prefilter_rejected"]
        passed = sb.BACKEND_COUNTS["contact_candidates.prefilter_passed"]
        native = sb.BACKEND_COUNTS["contact_candidates.native"]
        pairs = sum(1 for key in memo.entries if key[0] == "contacts")
        assert rejected > 0 and passed > 0
        assert rejected + passed == native == pairs
        runs.append((rejected, passed))
    # Не зависит от истории процесса: тот же вход - те же числа.
    assert runs[0] == runs[1]


# --------------------------------------------------------------------------
# 5. Длина источника: один раз на источник
# --------------------------------------------------------------------------


def test_the_source_length_is_computed_per_source_and_not_per_pair(monkeypatch):
    prepared = _field_preparation(*FIELD_CASES[2])
    context, domain = prepared.context, prepared.domain
    sources = list(_sources(context))
    calls = []
    real = ExactPlanarMetric.length_g_native

    def counting(self, vector):
        calls.append(1)
        return real(self, vector)

    monkeypatch.setattr(ExactPlanarMetric, "length_g_native", counting)
    _, memo = _resolve_all(context, domain, ("0.4",))
    pairs = sum(1 for key in memo.entries if key[0] == "contacts")
    # Один раз кадр источника и (когда есть контакт) один раз память длины (`_source_length`).
    assert 0 < len(calls) <= 2 * len(sources)
    assert len(calls) < pairs // 4, (len(calls), pairs)


# --------------------------------------------------------------------------
# 6. Вне binary64 фильтр уступает
# --------------------------------------------------------------------------


def test_the_prefilter_yields_outside_binary64_and_the_exact_path_answers():
    context, domain = _geometry(*_hole_ring())
    source = next(_sources(context))
    frame = SourceContactFrame(context, source)
    huge = sp.Integer(10) ** 400
    first = ExactPlanarPoint.from_values(huge, huge)
    second = ExactPlanarPoint.from_values(huge + 1, huge)
    boundary = _segment(first, second)
    assert frame.excludes(boundary) is False
    assert contact_candidates_native(context, source, boundary) == _contact_candidates(context, source, boundary)
