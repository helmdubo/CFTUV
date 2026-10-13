"""Адаптивная минимальная рациональная власть Density-веера."""

from __future__ import annotations

from .density_fan_types import (
    DensityRationalAuthorityExhausted,
    AdaptiveDensityFanInvalid,
    DensityWindowChartUnrepresentable,
    DensityTerminationBoxesExhausted,
)
from .density_projective import (
    _DensityVector,
    _ratio,
    _fraction,
    _height,
    _expected_orientation,
    _dual_dot,
    _oriented_cross,
    _sign,
    _vector,
    _inside_ordinal,
    _slope,
    _vector_from_slope,
    _chart_for_window,
    _subturn,
    _subturn_boundary,
    _quarter_turn,
    _exact_q_trig,
    _rotate_q,
    _subturn_boundary_vectors,
)

from dataclasses import replace
from functools import partial

from .density_controller_v1 import resolve_density_laws
from decimal import Decimal
from fractions import Fraction
from math import gcd, isqrt

import sympy as sp

from .._cpython311 import sorted_as_cpython311
from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import (
    AdaptiveFareyHeightRangeWitnessV2,
    AdaptiveMinimalRationalFanAuthorityV2,
    AdaptiveRationalFanOrdinalWindowAtlasV1,
    AdaptiveRationalFanOrdinalWindowV2,
    DirectionBindingReasonV1,
    DirectionBindingCertificateV1,
    EvaluationGeometryDirectionBindingCertificateV1,
)
from ..numeric import CertifiedDecimalIntervalV1
from .adaptive_density_atlas import (
    AtlasWindowPrepared,
    DensityExactWorkBudget,
    SegmentPlanLaws,
    build_atlas_window_record,
    global_termination_height,
    piece_contains_vector,
    piece_ordered_endpoints,
    preparation_budget,
    search_global_height,
    search_segments,
    termination_piece,
    validate_atlas_structure,
)
from . import adaptive_density_band as _band
from .common import ReferenceGeometryError, stable_id
from .metric import ExactPlanarMetric
from .planar_types import ExactPlanarVector


ADAPTIVE_FAN_MONOTONE = "ADAPTIVE_FAN_MONOTONE"
ADAPTIVE_FAN_ORDINAL_WINDOWS = "ADAPTIVE_FAN_ORDINAL_WINDOWS"
ADAPTIVE_FAN_SUBTURN = "ADAPTIVE_FAN_SUBTURN"
ADAPTIVE_FAN_MINIMAL_COMMON_HEIGHT = (
    "ADAPTIVE_FAN_MINIMAL_COMMON_HEIGHT"
)
ADAPTIVE_FAN_GRAM_OBJECTIVE = "ADAPTIVE_FAN_GRAM_OBJECTIVE"
ADAPTIVE_FAN_POSITIVE_TERMINATION_WIDTH = (
    "ADAPTIVE_FAN_POSITIVE_TERMINATION_WIDTH"
)
ADAPTIVE_FAN_FAREY_HEIGHT_RANGE = "ADAPTIVE_FAN_FAREY_HEIGHT_RANGE"
ADAPTIVE_FAN_INNER_OUTWARD_ENVELOPE = (
    "ADAPTIVE_FAN_INNER_OUTWARD_ENVELOPE"
)

ADAPTIVE_FAN_PROVEN_PREDICATES = frozenset(
    {
        ADAPTIVE_FAN_MONOTONE,
        ADAPTIVE_FAN_ORDINAL_WINDOWS,
        ADAPTIVE_FAN_SUBTURN,
        ADAPTIVE_FAN_MINIMAL_COMMON_HEIGHT,
        ADAPTIVE_FAN_GRAM_OBJECTIVE,
        ADAPTIVE_FAN_POSITIVE_TERMINATION_WIDTH,
        ADAPTIVE_FAN_FAREY_HEIGHT_RANGE,
        ADAPTIVE_FAN_INNER_OUTWARD_ENVELOPE,
    }
)

_RESOURCE_HEIGHT_CAP = 1_000_000
_BOX_REFINEMENT_CAP = 96

# Объявленный кап точной работы поиска минимального D* (реестр допусков:
# `DENSITY_EXACT_WORK_CAP_V1`). Высота останова Фарея `floor(1/w) + 2` на поле
# достигает 4.3e7, то есть бюджетом НЕ является. Стоимость прохода известна точно:
# одна карта — около `w*h + 1` пробников на высоту h и окно; atlas — примитивных
# ковекторов высоты h не больше 8h, проход по высотам 1..H для k окон не больше
# `4*H*(H+1)*k`. Литерал выведен из худшего случая ЗАМОРОЖЕННОГО корпуса:
# building.002, explicit density 4, corner relation 60fc81c8…, общая высота 4492
# при k = 2, ровно 9 035 единиц (тест
# `test_deepest_green_field_authority_stays_far_below_the_work_cap`); запас 14x,
# степень двойки: 9 035 * 14.5 ~ 131 000 -> `1 << 17`. Кап тратится на все k окон
# сразу: больший k уменьшает достижимую высоту, а не удорожает прогон.
_DENSITY_EXACT_WORK_CAP = 1 << 17


def _work_budget(cap: int | None = None) -> DensityExactWorkBudget:
    """Свежий счётчик точной работы одной транзакции поиска D*."""
    return DensityExactWorkBudget(
        DensityRationalAuthorityExhausted,
        _DENSITY_EXACT_WORK_CAP if cap is None else cap,
    )


def _preparation_budget(cap: int | None = None) -> DensityExactWorkBudget:
    """Счётчик точной работы подготовки власти. Закон капа — в atlas."""
    return preparation_budget(DensityRationalAuthorityExhausted, cap)


def _window_envelopes(ideal, *, laws=None):
    laws = resolve_density_laws(laws)

    windows = []
    records = []
    metric = ideal.metric
    for ordinal in range(1, len(ideal) - 1):
        previous, following = laws.window_neighbours(ideal, ordinal)
        left_values = metric.density_expressions(previous)
        center_values = metric.density_expressions(ideal[ordinal])
        right_values = metric.density_expressions(following)
        # Для projective slope нормирующий множитель midpoint не нужен.
        left = laws.vector(
            left_values[0] + center_values[0],
            left_values[1] + center_values[1],
            metric,
        )
        right = laws.vector(
            center_values[0] + right_values[0],
            center_values[1] + right_values[1],
            metric,
        )
        try:
            use_x, denominator_sign = laws.chart_for_window(
                metric,
                ideal[ordinal],
                left,
                right,
            )
        except DensityWindowChartUnrepresentable:
            windows.append((left, right))
            records.append(
                build_atlas_window_record(
                    metric,
                    ideal[ordinal],
                    left,
                    right,
                    sign=laws.sign,
                    slope=laws.slope,
                    vector=laws.vector,
                    decimal_envelope=laws.envelope,
                    invalid=DensityWindowChartUnrepresentable,
                    strict_between=laws.strict_between,
                    literal_unit=laws.literal_unit,
                )
            )
            continue
        slopes = (
            laws.slope(left, use_x, metric),
            laws.slope(right, use_x, metric),
        )
        lower_exact, upper_exact = (
            slopes
            if laws.sign(slopes[1] - slopes[0], metric) > 0
            else (slopes[1], slopes[0])
        )
        lower = laws.envelope(lower_exact, metric)
        upper = laws.envelope(upper_exact, metric)
        if lower.upper >= upper.lower:
            raise AdaptiveDensityFanInvalid(
                "ordinal rational envelopes overlap"
            )
        windows.append((left, right))
        records.append((use_x, denominator_sign, lower, upper))
    return tuple(windows), tuple(records)


class _IdealTuple(tuple):
    metric: ExactPlanarMetric
    window_law: str
    band_cache: dict
    band_orientation: int | None


def _covectors(
    metric,
    ideal_unit_normals,
    window_law: str = _band.WINDOW_LAW_VORONOI,
    orientation: TurnOrientation | None = None,
    *, laws=None,
):
    laws = resolve_density_laws(laws)
    remembered = metric._density_exact_memo.covector_rows
    covectors = []
    for normal in ideal_unit_normals:
        nx, ny = metric.density_expressions(normal)
        row = remembered.get((nx, ny))
        if row is None:
            # Ковектор нормали — чистая функция её выражений; повтор (а нормаль входит и в веер своего
            # счёта, и в веера соседних счетов, и в обе геометрии) берётся из памяти транзакции.
            row = remembered[(nx, ny)] = laws.vector(
                metric.gram[0][0] * nx + metric.gram[0][1] * ny,
                metric.gram[1][0] * nx + metric.gram[1][1] * ny,
                metric,
            )
        covectors.append(row)
    values = _IdealTuple(covectors)
    values.metric = metric
    values.window_law = window_law
    values.band_cache = {}
    values.band_orientation = (
        None if orientation is None else _expected_orientation(orientation)
    )
    return values


def _candidate_vector(
    slope: Fraction,
    use_x: bool,
    denominator_sign: int,
    metric: ExactPlanarMetric | None = None,
    *, laws=None,
) -> ExactPlanarVector:
    laws = resolve_density_laws(laws)
    return laws.vector(
        *_vector_from_slope(slope, use_x, denominator_sign),
        metric,
    )


def _sealed_local_interval(
    metric,
    ideal,
    ordinal,
    orientation,
    q,
    record,
    *, laws=None,
):
    """Один раз запечатать inner/outward рациональную оболочку окна."""
    laws = resolve_density_laws(laws)


    use_x, _, ordinal_lower, ordinal_upper = record
    boundary_vectors = laws.subturn_boundary_vectors(
        metric,
        ideal,
        ordinal,
        orientation,
        q,
    )
    boundaries = []
    for boundary_vector in boundary_vectors:
        denominator = metric.density_expressions(
            boundary_vector
        )[0 if use_x else 1]
        if laws.sign(denominator, metric) == 0:
            continue
        boundaries.append(
            laws.envelope(
                laws.slope(boundary_vector, use_x, metric),
                metric,
            )
        )
    center_envelope = laws.envelope(
        laws.slope(ideal[ordinal], use_x, metric),
        metric,
    )
    center = (
        Fraction(center_envelope.lower)
        + Fraction(center_envelope.upper)
    ) / 2
    lower_boundaries = [ordinal_lower]
    upper_boundaries = [ordinal_upper]
    for boundary in boundaries:
        if Fraction(boundary.upper) < center:
            lower_boundaries.append(boundary)
        elif Fraction(boundary.lower) > center:
            upper_boundaries.append(boundary)
        else:
            raise AdaptiveDensityFanInvalid(
                "subturn boundary overlaps the ideal direction"
            )
    lower = max(lower_boundaries, key=lambda item: item.upper)
    upper = min(upper_boundaries, key=lambda item: item.lower)
    lower_outward = Fraction(lower.lower)
    lower_inward = Fraction(lower.upper)
    upper_inward = Fraction(upper.lower)
    upper_outward = Fraction(upper.upper)
    if lower_outward == lower_inward:
        lower_inward = (lower_inward + center) / 2
    if upper_inward == upper_outward:
        upper_inward = (upper_inward + center) / 2
    sealed = (
        lower_outward,
        lower_inward,
        upper_inward,
        upper_outward,
    )
    if not (
        sealed[0] <= sealed[1] < sealed[2] <= sealed[3]
    ):
        raise AdaptiveDensityFanInvalid(
            "sealed local admissible interval is empty"
        )
    use_x, denominator_sign, *_ = record
    if not (
        _local_candidate(
            metric,
            ideal,
            ordinal,
            _vector_from_slope(
                sealed[1],
                use_x,
                denominator_sign,
            ),
            q,
            laws=laws,
        )
        and _local_candidate(
            metric,
            ideal,
            ordinal,
            _vector_from_slope(
                sealed[2],
                use_x,
                denominator_sign,
            ),
            q,
            laws=laws,
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "inner rational envelope is not admissible"
        )
    return sealed


def _box_is_feasible(metric, ideal, orientation, q, boxes, *, laws=None) -> bool:
    laws = resolve_density_laws(laws)
    expected = _expected_orientation(orientation)
    endpoints = []
    for ordinal, (lower, upper, use_x, sign) in enumerate(boxes, start=1):
        pair = (
            _candidate_vector(lower, use_x, sign, metric, laws=laws),
            _candidate_vector(upper, use_x, sign, metric, laws=laws),
        )
        previous, following = laws.window_neighbours(ideal, ordinal)
        if any(
            not laws.inside_ordinal(
                metric,
                item,
                previous,
                ideal[ordinal],
                following,
            )
            for item in pair
        ):
            return False
        endpoints.append(pair)
    sequences = [()]
    for pair in endpoints:
        sequences = [(*prefix, item) for prefix in sequences for item in pair]
    for hidden in sequences:
        sequence = (ideal[0], *hidden, ideal[-1])
        if any(
            laws.sign(
                laws.oriented_cross(metric, left, right),
                metric,
            )
            != expected
            for left, right in zip(sequence, sequence[1:])
        ):
            return False
        if any(
            not laws.subturn(metric, left, right, q)
            for left, right in zip(sequence, sequence[1:])
        ):
            return False
    return True


def _termination_boxes(metric, ideal, orientation, q, records, *, laws=None):
    laws = resolve_density_laws(laws)
    centers = []
    gaps = []
    for ordinal, (use_x, _, lower, upper) in enumerate(records, start=1):

        ideal_envelope = laws.envelope(
            laws.slope(ideal[ordinal], use_x, metric),
            metric,
        )
        center = (
            Fraction(ideal_envelope.lower)
            + Fraction(ideal_envelope.upper)
        ) / 2
        gap = min(
            center - Fraction(lower.upper),
            Fraction(upper.lower) - center,
        )
        if gap <= 0:
            raise AdaptiveDensityFanInvalid(
                "ideal is not strictly inside its ordinal window"
            )
        centers.append(center)
        gaps.append(gap)
    # Termination-box не участвует в поиске D*: это отдельный позитивный
    # свидетель существования. Начинаем сразу с узкой dyadic окрестности,
    # чтобы не повторять тяжёлые algebraic predicates на каждом удвоении.
    #
    # Каждое уточнение стоит ровно 2^k проверенных угловых последовательностей
    # (`_box_is_feasible` строит их полный декартов набор), поэтому работа
    # оплачивается вперёд и по точному числу, а не по факту.
    budget = laws.preparation_budget(_preparation_budget)
    divisor = 1 << 24
    refinement_cap = _BOX_REFINEMENT_CAP if laws.box_cap is None else laws.box_cap
    for _ in range(refinement_cap):
        budget.spend_order_steps(1 << len(centers))
        boxes = tuple(
            (
                center - gap / divisor,
                center + gap / divisor,
                records[index][0],
                records[index][1],
            )
            for index, (center, gap) in enumerate(zip(centers, gaps))
        )
        if _box_is_feasible(metric, ideal, orientation, q, boxes, laws=laws):
            return boxes
        divisor *= 2
    # Подсказка в сообщении не вправе заменить собой честное имя отказа: сам
    # предикат может быть не решён, и тогда так и сказано.
    try:
        at_limit = str(laws.subturn_boundary(metric, ideal[0], ideal[1], q))
    except ReferenceGeometryError:
        at_limit = "undecided"
    raise DensityTerminationBoxesExhausted(
        f"termination box refinement exhausted after {refinement_cap} "
        f"halvings for {len(centers)} hidden rays: no feasible box was "
        f"witnessed (ideal fan exactly at the subturn limit: {at_limit})"
    )


def _termination_height(boxes, *, laws=None) -> int:
    laws = resolve_density_laws(laws)
    bounds = []
    for lower, upper, _, _ in boxes:
        width = upper - lower
        if width <= 0:
            raise AdaptiveDensityFanInvalid(
                "fan termination width is not positive"
            )
        bounds.append(width.denominator // width.numerator + 2)
    return max(bounds, default=0)


def _local_record(record, *, laws=None):
    laws = resolve_density_laws(laws)
    return record.local_record if type(record) is AtlasWindowPrepared else record


def _local_candidate(metric, ideal, ordinal, vector, q, *, laws=None) -> bool:
    laws = resolve_density_laws(laws)
    candidate = laws.vector(*vector, metric)
    previous, following = laws.window_neighbours(ideal, ordinal)
    return (
        laws.inside_ordinal(
            metric,
            candidate,
            previous,
            ideal[ordinal],
            following,
        )
        and laws.subturn(
            metric,
            ideal[ordinal - 1],
            candidate,
            q,
        )
        and laws.subturn(
            metric,
            candidate,
            ideal[ordinal + 1],
            q,
        )
    )


def _strict_numerator_bounds(
    lower: Fraction,
    upper: Fraction,
    height: int,
    *, laws=None,
) -> tuple[int, int]:
    laws = resolve_density_laws(laws)
    first = (
        lower.numerator * height // lower.denominator
    ) + 1
    last = -(
        (-upper.numerator * height) // upper.denominator
    ) - 1
    return first, last


def _farey_shell_candidates(
    metric,
    ideal,
    ordinal,
    q,
    record,
    sealed_interval,
    height,
    budget,
    *, laws=None,
):
    """Высота Farey: integer-only внутри inner, exact только в shell."""
    laws = resolve_density_laws(laws)

    use_x, denominator_sign, *_ = record
    outer_lower, inner_lower, inner_upper, outer_upper = (
        sealed_interval
    )
    first, last = _strict_numerator_bounds(
        outer_lower,
        outer_upper,
        height,
        laws=laws,
    )
    # Пустая высота тоже стоит один probe: иначе последовательность пустых
    # высот прошла бы под капом без единой единицы работы.
    budget.spend_shell_probes(max(last - first + 1, 1))
    if first > last:
        return ()
    candidates = []
    for numerator in range(first, last + 1):
        if gcd(abs(numerator), height) != 1:
            continue
        slope = Fraction(numerator, height)
        if inner_lower < slope < inner_upper:
            valid = True
        elif (
            outer_lower < slope <= inner_lower
            or inner_upper <= slope < outer_upper
        ):
            valid = _local_candidate(
                metric,
                ideal,
                ordinal,
                _vector_from_slope(
                    slope,
                    use_x,
                    denominator_sign,
                ),
                q,
                laws=laws,
            )
        else:
            valid = False
        if valid:
            candidates.append(
                _vector_from_slope(
                    slope,
                    use_x,
                    denominator_sign,
                )
            )
    return tuple(candidates)


def _segment_plan_laws(*, laws=None) -> SegmentPlanLaws:
    """Точные предикаты, которыми sibling-модуль atlas не владеет."""
    laws = resolve_density_laws(laws)


    return SegmentPlanLaws(
        sign=laws.sign,
        slope=laws.slope,
        decimal_envelope=laws.envelope,
        local_candidate=partial(_local_candidate, laws=laws),
        vector_from_slope=_vector_from_slope,
        subturn_boundary_vectors=laws.subturn_boundary_vectors,
        invalid=DensityWindowChartUnrepresentable,
    )


def _objective(metric, ideal, ordinal, vector, *, laws=None):
    """Cos² Gram-score; его максимум равен минимуму старого расстояния."""
    laws = resolve_density_laws(laws)

    candidate = laws.vector(*vector, metric)
    dot = laws.dual_dot(metric, candidate, ideal[ordinal])
    norm = laws.dual_dot(metric, candidate, candidate)
    return dot * dot / norm


def _candidate_compare(metric, ideal, ordinal, *, laws=None):
    laws = resolve_density_laws(laws)

    scores = {}
    envelopes = {}

    def score(vector):
        if vector not in scores:
            scores[vector] = _objective(metric, ideal, ordinal, vector, laws=laws)
        return scores[vector]

    def envelope(vector):
        if vector not in envelopes:
            envelopes[vector] = laws.envelope(
                score(vector),
                metric,
            )
        return envelopes[vector]

    def compare(left, right):
        left_envelope = envelope(left)
        right_envelope = envelope(right)
        if left_envelope.lower > right_envelope.upper:
            return -1
        if right_envelope.lower > left_envelope.upper:
            return 1
        sign = laws.sign(
            score(right) - score(left),
            metric,
        )
        if sign:
            return sign
        use_x = abs(left[0]) >= abs(left[1])
        left_slope = Fraction(
            left[1] if use_x else left[0],
            left[0] if use_x else left[1],
        )
        right_slope = Fraction(
            right[1] if use_x else right[0],
            right[0] if use_x else right[1],
        )
        return -1 if left_slope > right_slope else 1 if left_slope < right_slope else 0

    return compare


def _legacy_authority_budget(width: Fraction, *, laws=None) -> int:
    laws = resolve_density_laws(laws)
    quotient_ceiling = (
        width.denominator + width.numerator - 1
    ) // width.numerator
    budget = isqrt(quotient_ceiling)
    while budget * budget < quotient_ceiling:
        budget += 1
    while budget > 1 and (
        (budget - 1) * (budget - 1) * width.numerator
        >= width.denominator
    ):
        budget -= 1
    return budget


def _legacy_density_certificate(
    metric,
    ideal,
    ordinal,
    q,
    record,
    *, laws=None,
) -> DirectionBindingCertificateV1 | None:
    laws = resolve_density_laws(laws)
    from .density_binding_types import _PROVEN_PREDICATES

    if type(record) is AtlasWindowPrepared:
        return None
    use_x, denominator_sign, lower_envelope, upper_envelope = record
    lower = Fraction(lower_envelope.upper)
    upper = Fraction(upper_envelope.lower)
    width = upper - lower
    if width <= 0:
        raise AdaptiveDensityFanInvalid("legacy Density window is empty")
    budget = _legacy_authority_budget(width, laws=laws)
    # Длина развёртки известна ТОЧНО до первого шага, поэтому она оплачивается
    # вперёд: узкое окно отказывает именем за микросекунды, а не после часа
    # перебора. Сам перебор ниже не тронут — на доменах под капом он побитово
    # прежний.
    laws.preparation_budget(_preparation_budget).spend_shell_probes(budget)
    winner = None
    for denominator in range(1, budget + 1):
        scaled_lower = lower * denominator
        scaled_upper = upper * denominator
        first = scaled_lower.numerator // scaled_lower.denominator + 1
        last = -(
            (-scaled_upper.numerator) // scaled_upper.denominator
        ) - 1
        for numerator in range(first, last + 1):
            if gcd(abs(numerator), denominator) != 1:
                continue
            slope = Fraction(numerator, denominator)
            vector = _vector_from_slope(
                slope,
                use_x,
                denominator_sign,
            )
            candidate = laws.vector(*vector, metric)
            if not (
                laws.subturn(metric, ideal[ordinal - 1], candidate, q)
                and laws.subturn(metric, candidate, ideal[ordinal + 1], q)
            ):
                continue
            score = _objective(metric, ideal, ordinal, vector, laws=laws)
            if (
                winner is None
                or laws.sign(score - winner[0], metric) > 0
                or (
                    laws.sign(score - winner[0], metric) == 0
                    and slope > winner[1]
                )
            ):
                winner = score, slope, vector
    if winner is None:
        return None
    return DirectionBindingCertificateV1(
        bound_primitive_integer_vector=winner[2],
        ideal_window_lower_slope_envelope=lower_envelope,
        ideal_window_upper_slope_envelope=upper_envelope,
        certified_window_width_lower_bound=(
            laws.width(lower_envelope.upper, upper_envelope.lower)
        ),
        proven_predicates=_PROVEN_PREDICATES,
    )


def _legacy_full_fan_valid(
    metric,
    ideal,
    orientation,
    q,
    records,
    certificates,
    *, laws=None,
) -> bool:
    laws = resolve_density_laws(laws)
    from .density_binding_types import _PROVEN_PREDICATES

    expected = _expected_orientation(orientation)
    resolved = tuple(
        ideal[ordinal]
        if certificate is None
        else laws.vector(
            *certificate.bound_primitive_integer_vector,
            metric,
        )
        for ordinal, certificate in enumerate(certificates, start=1)
    )
    sequence = (ideal[0], *resolved, ideal[-1])
    if any(
        certificate is not None
        and certificate.proven_predicates != _PROVEN_PREDICATES
        for certificate in certificates
    ):
        return False
    if any(
        laws.sign(laws.oriented_cross(metric, left, right), metric) != expected
        or not laws.subturn(metric, left, right, q)
        for left, right in zip(sequence, sequence[1:])
    ):
        return False
    return all(
        certificate is None
        or (
            type(record) is not AtlasWindowPrepared
            and laws.inside_ordinal(
                metric,
                candidate,
                *laws.window_triple(ideal, ordinal),
            )
            and (
                Fraction(record[2].upper)
                < laws.fraction(laws.slope(candidate, record[0], metric))
                < Fraction(record[3].lower)
            )
        )
        for ordinal, (certificate, candidate, record) in enumerate(
            zip(certificates, resolved, records, strict=True),
            start=1,
        )
    )


def certify_legacy_density_bindings_factor_free(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    max_subturn_q: int,
) -> tuple[DirectionBindingCertificateV1 | None, ...]:
    """Byte-compatible B(w) без generic symbolic normalization."""

    if max_subturn_q not in range(2, 7):
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    if len(ideal_unit_normals) < 3:
        return ()
    ideal = _covectors(metric, ideal_unit_normals)
    _, records = _window_envelopes(ideal)
    certificates = tuple(
        _legacy_density_certificate(
            metric,
            ideal,
            ordinal,
            max_subturn_q,
            records[ordinal - 1],
        )
        for ordinal in range(1, len(ideal) - 1)
    )
    if not _legacy_full_fan_valid(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        certificates,
    ):
        raise AdaptiveDensityFanInvalid("legacy Density fan is not valid")
    return certificates


def certify_density_bindings_and_adaptive_fallback(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    max_subturn_q: int,
    *,
    binding_reasons: tuple[DirectionBindingReasonV1 | None, ...],
    resource_height_cap: int = _RESOURCE_HEIGHT_CAP,
    laws=None,
) -> tuple[
    tuple[DirectionBindingCertificateV1 | None, ...],
    AdaptiveMinimalRationalFanAuthorityV2 | None,
]:
    """Одна транзакция B(w) и, при пустоте, adaptive-власти.

    Обе ветви закона используют одни production covectors и ordinal windows.
    Повторное их построение здесь было бы не независимой проверкой, а только
    повтором той же власти внутри одной compile-транзакции.
    """
    laws = resolve_density_laws(laws)

    if max_subturn_q not in range(2, 7):
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    if len(ideal_unit_normals) < 3:
        return (), None
    ideal = _covectors(metric, ideal_unit_normals, laws=laws)
    _, records = _window_envelopes(ideal, laws=laws)
    certificates = tuple(
        _legacy_density_certificate(
            metric,
            ideal,
            ordinal,
            max_subturn_q,
            records[ordinal - 1],
            laws=laws,
        )
        for ordinal in range(1, len(ideal) - 1)
    )
    legacy_full_fan_valid = _legacy_full_fan_valid(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        certificates,
        laws=laws,
    )
    if (
        legacy_full_fan_valid
        and all(
            reason is None or certificate is not None
            for reason, certificate in zip(
                binding_reasons,
                certificates,
                strict=True,
            )
        )
    ):
        return certificates, None
    authority = _certify_adaptive_density_fan_prepared(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        binding_reasons=binding_reasons,
        resource_height_cap=resource_height_cap,
        laws=laws,
    )
    return certificates, authority


def verify_legacy_density_bindings_factor_free(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
        | None,
        ...,
    ],
    max_subturn_q: int,
) -> None:
    """Независимо перевывести byte-compatible B(w) и полный веер."""

    ideal = _covectors(metric, ideal_unit_normals)
    _, records = _window_envelopes(ideal)
    canonical = certify_legacy_density_bindings_factor_free(
        metric,
        ideal_unit_normals,
        orientation,
        max_subturn_q,
    )
    if len(certificates) != len(canonical):
        raise AdaptiveDensityFanInvalid("legacy Density cardinality invalid")
    for ordinal, (supplied, expected) in enumerate(
        zip(certificates, canonical, strict=True),
        start=1,
    ):
        if supplied is None:
            x, y = metric.density_expressions(ideal[ordinal])
            ratio = (
                None
                if _sign(x, metric) == 0
                else sp.cancel(y / x)
            )
            projectively_rational = (
                _sign(x, metric) == 0
                and _sign(y, metric) != 0
            ) or (ratio is not None and ratio.is_Rational is True)
            if expected is not None and not projectively_rational:
                raise AdaptiveDensityFanInvalid(
                    "legacy Density EMPTY authority mismatch"
                )
            continue
        if expected is None:
            raise AdaptiveDensityFanInvalid(
                "legacy Density supplied a false nonempty authority"
            )
        if (
            supplied.bound_primitive_integer_vector
            != expected.bound_primitive_integer_vector
            or supplied.ideal_window_lower_slope_envelope
            != expected.ideal_window_lower_slope_envelope
            or supplied.ideal_window_upper_slope_envelope
            != expected.ideal_window_upper_slope_envelope
            or supplied.certified_window_width_lower_bound
            != expected.certified_window_width_lower_bound
            or supplied.proven_predicates != expected.proven_predicates
        ):
            raise AdaptiveDensityFanInvalid(
                "legacy Density certificate is not canonical"
            )
    if not _legacy_full_fan_valid(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        certificates,
    ):
        raise AdaptiveDensityFanInvalid(
            "legacy Density supplied fan is not valid"
        )


def _best_fan(metric, ideal, orientation, q, candidates, budget, *, laws=None):
    laws = resolve_density_laws(laws)
    expected = _expected_orientation(orientation)
    budget.spend_order_steps(sum(len(items) for items in candidates))
    ordered = tuple(
        tuple(
            sorted_as_cpython311(
                items,
                _candidate_compare(metric, ideal, ordinal, laws=laws),
            )
        )
        for ordinal, items in enumerate(candidates, start=1)
    )

    def visit(ordinal, prefix):
        if ordinal == len(ordered):
            last = (
                ideal[0]
                if not prefix
                else laws.vector(*prefix[-1], metric)
            )
            return (
                prefix
                if laws.sign(
                    laws.oriented_cross(metric, last, ideal[-1]),
                    metric,
                )
                == expected
                and laws.subturn(
                    metric,
                    last,
                    ideal[-1],
                    q,
                )
                else None
            )
        previous = (
            ideal[0]
            if not prefix
            else laws.vector(*prefix[-1], metric)
        )
        for vector in ordered[ordinal]:
            budget.spend_order_steps(1)
            candidate = laws.vector(*vector, metric)
            if (
                laws.sign(
                    laws.oriented_cross(metric, previous, candidate),
                    metric,
                )
                != expected
                or not laws.subturn(
                    metric,
                    previous,
                    candidate,
                    q,
                )
            ):
                continue
            result = visit(ordinal + 1, (*prefix, vector))
            if result is not None:
                return result
        return None

    return visit(0, ())


def _search(
    metric,
    ideal,
    orientation,
    q,
    records,
    sealed_intervals,
    stop_height,
    budget,
    *, laws=None,
):
    laws = resolve_density_laws(laws)
    candidates = [set() for _ in records]
    previous_counts = tuple(0 for _ in records)
    # Внешние границы окон целыми: на большинстве высот оболочка пуста, и пустая высота стоит ровно один probe, как
    # `_farey_shell_candidates` платит за неё (`max(last - first + 1, 1)`), - вызывать её ради этого незачем.
    outer = [
        (item[0].numerator, item[0].denominator, item[3].numerator, item[3].denominator)
        for item in sealed_intervals
    ]
    for height in range(1, stop_height + 1):
        for ordinal, record in enumerate(records, start=1):
            lower_n, lower_d, upper_n, upper_d = outer[ordinal - 1]
            if lower_n * height // lower_d + 1 > -(-upper_n * height // upper_d) - 1:
                budget.spend_shell_probes(1)
                continue
            candidates[ordinal - 1].update(
                _farey_shell_candidates(
                    metric,
                    ideal,
                    ordinal,
                    q,
                    record,
                    sealed_intervals[ordinal - 1],
                    height,
                    budget,
                    laws=laws,
                )
            )
        fan = (
            _best_fan(metric, ideal, orientation, q, candidates, budget, laws=laws)
            if all(candidates)
            else None
        )
        if fan is not None:
            return height, fan, previous_counts, tuple(
                len(items) for items in candidates
            )
        previous_counts = tuple(len(items) for items in candidates)
    return None, None, previous_counts, tuple(
        len(items) for items in candidates
    )


def _authority_id(q, height, fan, windows, *, laws=None) -> str:
    laws = resolve_density_laws(laws)
    if any(
        type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1
        for item in windows
    ):
        return stable_id(
            "adaptive-density-fan-authority-atlas-v1",
            q,
            height,
            fan,
            windows,
        )
    return stable_id(
        "adaptive-density-fan-authority-v2",
        q,
        height,
        fan,
        tuple(
            (
                item.use_x_denominator,
                item.denominator_sign,
                item.full_lower_slope_envelope,
                item.full_upper_slope_envelope,
                item.termination_lower_slope,
                item.termination_upper_slope,
                item.admissible_lower_outward,
                item.admissible_lower_inward,
                item.admissible_upper_inward,
                item.admissible_upper_outward,
            )
            for item in windows
        ),
    )


def certify_adaptive_density_fan(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    max_subturn_q: int,
    *,
    binding_reasons: tuple[DirectionBindingReasonV1 | None, ...],
    resource_height_cap: int = _RESOURCE_HEIGHT_CAP,
    window_law: str = _band.WINDOW_LAW_VORONOI,
    laws=None,
) -> AdaptiveMinimalRationalFanAuthorityV2:
    """Построить одну V2-власть полного coupled Density-веера."""
    laws = resolve_density_laws(laws)

    ideal = _covectors(metric, ideal_unit_normals, window_law, orientation, laws=laws)
    _, records = _window_envelopes(ideal, laws=laws)
    return _certify_adaptive_density_fan_prepared(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        binding_reasons=binding_reasons,
        resource_height_cap=resource_height_cap,
        laws=laws,
    )


def _authority_windows(records, boxes, sealed_intervals, *, laws=None):
    """Собрать wire-окна власти: atlas и одно-картное — по своей форме."""
    laws = resolve_density_laws(laws)

    windows = []
    for ordinal, (record, box, sealed) in enumerate(
        zip(records, boxes, sealed_intervals, strict=True),
        start=1,
    ):
        common = {
            "ordinal": ordinal,
            "termination_lower_slope": _ratio(box[0]),
            "termination_upper_slope": _ratio(box[1]),
            "certified_termination_width": _ratio(box[1] - box[0]),
            "admissible_lower_outward": _ratio(sealed[0]),
            "admissible_lower_inward": _ratio(sealed[1]),
            "admissible_upper_inward": _ratio(sealed[2]),
            "admissible_upper_outward": _ratio(sealed[3]),
        }
        if type(record) is AtlasWindowPrepared:
            windows.append(
                AdaptiveRationalFanOrdinalWindowAtlasV1(
                    pieces=record.pieces,
                    termination_piece_index=record.termination_piece_index,
                    **common,
                )
            )
        else:
            windows.append(
                AdaptiveRationalFanOrdinalWindowV2(
                    use_x_denominator=record[0],
                    denominator_sign=record[1],
                    full_lower_slope_envelope=record[2],
                    full_upper_slope_envelope=record[3],
                    **common,
                )
            )
    return tuple(windows)


def _certify_adaptive_density_fan_prepared(
    metric,
    ideal,
    orientation,
    max_subturn_q,
    records,
    *,
    binding_reasons,
    resource_height_cap,
    laws=None,
) -> AdaptiveMinimalRationalFanAuthorityV2:
    """Достроить V2 из фактов, уже выведенных в этой транзакции."""
    laws = resolve_density_laws(laws)

    local_records = tuple(_local_record(record, laws=laws) for record in records)
    uses_atlas = any(type(record) is AtlasWindowPrepared for record in records)
    boxes = _termination_boxes(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        local_records,
        laws=laws,
    )
    sealed_intervals = tuple(
        _sealed_local_interval(
            metric,
            ideal,
            ordinal,
            orientation,
            max_subturn_q,
            local_record,
            laws=laws,
        )
        for ordinal, local_record in enumerate(local_records, start=1)
    )
    termination_height = (
        global_termination_height(
            boxes,
            local_height=partial(_termination_height, laws=laws),
        )
        if uses_atlas
        else _termination_height(boxes, laws=laws)
    )
    stop_height = min(termination_height, resource_height_cap)
    height, fan, previous_counts, _ = (
        search_global_height(
            metric,
            ideal,
            orientation,
            max_subturn_q,
            stop_height,
            segments=search_segments(
                metric,
                ideal,
                orientation,
                max_subturn_q,
                records,
                local_records,
                sealed_intervals,
                _segment_plan_laws(laws=laws),
            ),
            local_candidate=partial(_local_candidate, laws=laws),
            vector_from_slope=_vector_from_slope,
            best_fan=partial(_best_fan, laws=laws),
            budget=laws.search_budget(_work_budget),
        )
        if uses_atlas
        else _search(
            metric,
            ideal,
            orientation,
            max_subturn_q,
            records,
            sealed_intervals,
            stop_height,
            laws.search_budget(_work_budget),
            laws=laws,
        )
    )
    if fan is None:
        raise DensityRationalAuthorityExhausted(
            "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
        )
    windows = _authority_windows(records, boxes, sealed_intervals, laws=laws)
    authority_id = _authority_id(max_subturn_q, height, fan, windows, laws=laws)
    return AdaptiveMinimalRationalFanAuthorityV2(
        authority_id=authority_id,
        max_subturn_q=max_subturn_q,
        minimal_common_height=height,
        exhaustive_previous_height=height - 1,
        termination_height_upper_bound=termination_height,
        bound_primitive_integer_vectors=fan,
        binding_reasons=binding_reasons,
        ordinal_windows=windows,
        previous_height_witness=AdaptiveFareyHeightRangeWitnessV2(
            first_height=1,
            last_height=height - 1,
            primitive_candidate_counts=previous_counts,
        ),
        proven_predicates=_band.window_law_predicates(
            ideal.window_law, ADAPTIVE_FAN_PROVEN_PREDICATES
        ),
    )


def _verify_authority_structure(supplied, ideal_count) -> None:
    if (
        type(supplied) is not AdaptiveMinimalRationalFanAuthorityV2
        or type(supplied.max_subturn_q) is not int
        or supplied.max_subturn_q not in range(2, 7)
        or type(supplied.minimal_common_height) is not int
        or supplied.minimal_common_height < 1
        or type(supplied.exhaustive_previous_height) is not int
        or type(supplied.termination_height_upper_bound) is not int
        or supplied.proven_predicates
        not in _band.known_predicate_sets(ADAPTIVE_FAN_PROVEN_PREDICATES)
        or supplied.exhaustive_previous_height
        != supplied.minimal_common_height - 1
        or len(supplied.binding_reasons)
        != len(supplied.bound_primitive_integer_vectors)
        or type(supplied.previous_height_witness)
        is not AdaptiveFareyHeightRangeWitnessV2
        or len(
            supplied.previous_height_witness.primitive_candidate_counts
        )
        != len(supplied.bound_primitive_integer_vectors)
        or supplied.previous_height_witness.first_height != 1
        or supplied.previous_height_witness.last_height
        != supplied.minimal_common_height - 1
        or any(
            type(value) is not int or value < 0
            for value in supplied.previous_height_witness.primitive_candidate_counts
        )
        or any(
            len(vector) != 2
            or any(type(value) is not int for value in vector)
            or vector == (0, 0)
            or gcd(abs(vector[0]), abs(vector[1])) != 1
            for vector in supplied.bound_primitive_integer_vectors
        )
    ):
        raise AdaptiveDensityFanInvalid("adaptive fan tag/cardinality invalid")
    if (
        len(supplied.ordinal_windows) != ideal_count - 2
        or len(supplied.bound_primitive_integer_vectors) != ideal_count - 2
        or any(
            type(item.ordinal) is not int
            or type(item)
            not in {
                AdaptiveRationalFanOrdinalWindowV2,
                AdaptiveRationalFanOrdinalWindowAtlasV1,
            }
            for item in supplied.ordinal_windows
        )
        or tuple(item.ordinal for item in supplied.ordinal_windows)
        != tuple(range(1, ideal_count - 1))
    ):
        raise AdaptiveDensityFanInvalid("adaptive fan ordinal identity invalid")


def _decode_authority_windows(supplied):
    records = tuple(
        (
            (
                item.use_x_denominator,
                item.denominator_sign,
                item.full_lower_slope_envelope,
                item.full_upper_slope_envelope,
            )
            if type(item) is AdaptiveRationalFanOrdinalWindowV2
            else (
                termination_piece(
                    item,
                    invalid=AdaptiveDensityFanInvalid,
                ).use_x_denominator,
                termination_piece(
                    item,
                    invalid=AdaptiveDensityFanInvalid,
                ).denominator_sign,
                termination_piece(
                    item,
                    invalid=AdaptiveDensityFanInvalid,
                ).lower_slope_envelope,
                termination_piece(
                    item,
                    invalid=AdaptiveDensityFanInvalid,
                ).upper_slope_envelope,
            )
        )
        for item in supplied.ordinal_windows
    )
    boxes = tuple(
        (
            _fraction(item.termination_lower_slope),
            _fraction(item.termination_upper_slope),
            record[0],
            record[1],
        )
        for item, record in zip(
            supplied.ordinal_windows,
            records,
            strict=True,
        )
    )
    sealed_intervals = tuple(
        (
            _fraction(item.admissible_lower_outward),
            _fraction(item.admissible_lower_inward),
            _fraction(item.admissible_upper_inward),
            _fraction(item.admissible_upper_outward),
        )
        for item in supplied.ordinal_windows
    )
    return records, boxes, sealed_intervals


def verify_sealed_adaptive_density_fan(
    supplied: AdaptiveMinimalRationalFanAuthorityV2,
    *,
    ideal_count: int,
) -> None:
    """Проверить wire-власть рационально, не повторяя root certification."""

    _verify_authority_structure(supplied, ideal_count)
    records, boxes, sealed_intervals = _decode_authority_windows(supplied)
    for item, record, vector, box, sealed in zip(
        supplied.ordinal_windows,
        records,
        supplied.bound_primitive_integer_vectors,
        boxes,
        sealed_intervals,
        strict=True,
    ):
        use_x, denominator_sign, lower_envelope, upper_envelope = record
        denominator = vector[0 if use_x else 1]
        numerator = vector[1 if use_x else 0]
        slope = Fraction(numerator, denominator)
        atlas = type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1
        if atlas:
            validate_atlas_structure(
                item,
                invalid=AdaptiveDensityFanInvalid,
            )
        if (
            denominator_sign not in (-1, 1)
            or (
                not atlas
                and (
                    (1 if denominator > 0 else -1) != denominator_sign
                    or not (
                        Fraction(lower_envelope.upper)
                        < slope
                        < Fraction(upper_envelope.lower)
                    )
                )
            )
            or (
                atlas
                and sum(
                    piece_contains_vector(piece, vector)
                    for piece in item.pieces
                )
                != 1
            )
            or not (sealed[0] <= sealed[1] < sealed[2] <= sealed[3])
            or (not atlas and not (sealed[0] < slope < sealed[3]))
            or box[0] >= box[1]
            or _fraction(item.certified_termination_width)
            != box[1] - box[0]
        ):
            raise AdaptiveDensityFanInvalid(
                "sealed adaptive rational authority is false"
            )
    uses_atlas = any(
        type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1
        for item in supplied.ordinal_windows
    )
    if (
        (
            global_termination_height(
                boxes,
                local_height=_termination_height,
            )
            if uses_atlas
            else _termination_height(boxes)
        )
        != supplied.termination_height_upper_bound
        or max(
            (
                (
                    _height(vector)
                    if uses_atlas
                    else abs(vector[0 if record[0] else 1])
                )
                for vector, record in zip(
                    supplied.bound_primitive_integer_vectors,
                    records,
                    strict=True,
                )
            ),
            default=0,
        )
        != supplied.minimal_common_height
        or supplied.authority_id
        != _authority_id(
            supplied.max_subturn_q,
            supplied.minimal_common_height,
            supplied.bound_primitive_integer_vectors,
            supplied.ordinal_windows,
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "sealed adaptive rational identity is false"
        )


def _verify_adaptive_density_fan(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    supplied: AdaptiveMinimalRationalFanAuthorityV2,
) -> None:
    """Независимо связать sealed V2 с production metric/ideal facts."""

    ideal = _covectors(
        metric, ideal_unit_normals, _band.authority_window_law(supplied), orientation
    )
    _verify_authority_structure(supplied, len(ideal))
    records, boxes, sealed_intervals = _decode_authority_windows(supplied)
    _verify_window_envelopes(metric, ideal, supplied.ordinal_windows)
    if not _box_is_feasible(
        metric,
        ideal,
        orientation,
        supplied.max_subturn_q,
        boxes,
    ):
        raise AdaptiveDensityFanInvalid(
            "adaptive termination box is not inside the full feasible fan"
        )
    for item, box in zip(supplied.ordinal_windows, boxes, strict=True):
        if (
            _fraction(item.certified_termination_width)
            != box[1] - box[0]
            or box[0] >= box[1]
        ):
            raise AdaptiveDensityFanInvalid(
                "adaptive termination width is false"
            )
    for ordinal, (record, sealed) in enumerate(
        zip(records, sealed_intervals, strict=True),
        start=1,
    ):
        _verify_sealed_local_interval(
            metric,
            ideal,
            ordinal,
            supplied.max_subturn_q,
            record,
            sealed,
        )
    uses_atlas = any(
        type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1
        for item in supplied.ordinal_windows
    )
    termination_height = (
        global_termination_height(
            boxes,
            local_height=_termination_height,
        )
        if uses_atlas
        else _termination_height(boxes)
    )
    if termination_height != supplied.termination_height_upper_bound:
        raise AdaptiveDensityFanInvalid(
            "adaptive termination height is false"
        )
    height, fan, previous_counts, _ = (
        search_global_height(
            metric,
            ideal,
            orientation,
            supplied.max_subturn_q,
            supplied.minimal_common_height,
            segments=search_segments(
                metric,
                ideal,
                orientation,
                supplied.max_subturn_q,
                supplied.ordinal_windows,
                records,
                sealed_intervals,
                _segment_plan_laws(),
            ),
            local_candidate=_local_candidate,
            vector_from_slope=_vector_from_slope,
            best_fan=_best_fan,
            budget=_work_budget(),
        )
        if uses_atlas
        else _search(
            metric,
            ideal,
            orientation,
            supplied.max_subturn_q,
            records,
            sealed_intervals,
            supplied.minimal_common_height,
            _work_budget(),
        )
    )
    if (
        height != supplied.minimal_common_height
        or fan != supplied.bound_primitive_integer_vectors
        or previous_counts
        != supplied.previous_height_witness.primitive_candidate_counts
        or supplied.authority_id
        != _authority_id(
            supplied.max_subturn_q,
            supplied.minimal_common_height,
            supplied.bound_primitive_integer_vectors,
            supplied.ordinal_windows,
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "adaptive fan minimality or Gram winner is false"
        )


def verify_adaptive_density_fan(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    supplied: AdaptiveMinimalRationalFanAuthorityV2,
) -> None:
    """Нормализовать любой malformed record в один именованный отказ."""

    try:
        _verify_adaptive_density_fan(
            metric,
            ideal_unit_normals,
            orientation,
            supplied,
        )
    except AdaptiveDensityFanInvalid:
        raise
    except (
        ArithmeticError,
        AttributeError,
        IndexError,
        TypeError,
        ValueError,
    ) as exc:
        raise AdaptiveDensityFanInvalid(
            "adaptive fan record is malformed"
        ) from exc


def _boundary_scores(metric, ideal, ordinal, slope, use_x, sign):
    vector = _candidate_vector(slope, use_x, sign, metric)
    previous, following = _band.window_neighbours(ideal, ordinal)
    px, py = metric.density_expressions(previous)
    ix, iy = metric.density_expressions(ideal[ordinal])
    fx, fy = metric.density_expressions(following)
    return (
        _sign(
            _dual_dot(
                metric,
                vector,
                _vector(ix - px, iy - py, metric),
            ),
            metric,
        ),
        _sign(
            _dual_dot(
                metric,
                vector,
                _vector(ix - fx, iy - fy, metric),
            ),
            metric,
        ),
    )


def _brackets_boundary(left_scores, right_scores) -> bool:
    changes = 0
    for index in range(2):
        if left_scores[index] <= 0 <= right_scores[index] or (
            right_scores[index] <= 0 <= left_scores[index]
        ):
            changes += 1
    return changes == 1


def _atlas_boundary_envelope(piece, *, first: bool):
    start, end = piece_ordered_endpoints(piece)
    return start if first else end


def _verify_atlas_window(metric, ideal, item) -> None:
    """Связать atlas-piece свойства с chart-free production predicates."""

    validate_atlas_structure(
        item,
        invalid=AdaptiveDensityFanInvalid,
    )
    ordinal = item.ordinal
    previous, following = _band.window_neighbours(ideal, ordinal)
    left_values = metric.density_expressions(previous)
    center_values = metric.density_expressions(ideal[ordinal])
    right_values = metric.density_expressions(following)
    left = _vector(
        left_values[0] + center_values[0],
        left_values[1] + center_values[1],
        metric,
    )
    right = _vector(
        center_values[0] + right_values[0],
        center_values[1] + right_values[1],
        metric,
    )
    try:
        _chart_for_window(metric, ideal[ordinal], left, right)
    except DensityWindowChartUnrepresentable:
        pass
    else:
        raise AdaptiveDensityFanInvalid(
            "adaptive fan atlas used for a single-chart window"
        )
    for piece, first in (
        (item.pieces[0], True),
        (item.pieces[-1], False),
    ):
        envelope = _atlas_boundary_envelope(piece, first=first)
        scores = tuple(
            _boundary_scores(
                metric,
                ideal,
                ordinal,
                Fraction(value),
                piece.use_x_denominator,
                piece.denominator_sign,
            )
            for value in (envelope.lower, envelope.upper)
        )
        if not _brackets_boundary(*scores):
            raise AdaptiveDensityFanInvalid(
                "atlas outward ordinal envelope is not production-bound"
            )
    for piece in item.pieces:
        lower = Fraction(piece.lower_slope_envelope.upper)
        upper = Fraction(piece.upper_slope_envelope.lower)
        midpoint = (lower + upper) / 2
        vector = _candidate_vector(
            midpoint,
            piece.use_x_denominator,
            piece.denominator_sign,
            metric,
        )
        if not _inside_ordinal(
            metric,
            vector,
            *_band.window_triple(ideal, ordinal),
        ):
            raise AdaptiveDensityFanInvalid(
                "atlas piece is outside the production ordinal"
            )
    selected_piece = termination_piece(
        item,
        invalid=AdaptiveDensityFanInvalid,
    )
    denominator = metric.density_expressions(ideal[ordinal])[
        0 if selected_piece.use_x_denominator else 1
    ]
    if (
        _sign(denominator, metric) != selected_piece.denominator_sign
        or not (
            Fraction(selected_piece.lower_slope_envelope.upper)
            < _slope(
                ideal[ordinal],
                selected_piece.use_x_denominator,
                metric,
            )
            < Fraction(selected_piece.upper_slope_envelope.lower)
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "atlas termination piece does not own the ideal direction"
        )


def _verify_window_envelopes(metric, ideal, windows) -> None:
    """Связать outward envelopes с двумя production Voronoi predicates."""

    for item in windows:
        if type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1:
            _verify_atlas_window(metric, ideal, item)
            continue
        if (
            type(item.use_x_denominator) is not bool
            or item.denominator_sign not in (-1, 1)
            or item.full_lower_slope_envelope.upper
            >= item.full_upper_slope_envelope.lower
        ):
            raise AdaptiveDensityFanInvalid("adaptive fan chart invalid")
        lower = item.full_lower_slope_envelope
        upper = item.full_upper_slope_envelope
        lower_scores = (
            _boundary_scores(
                metric,
                ideal,
                item.ordinal,
                Fraction(lower.lower),
                item.use_x_denominator,
                item.denominator_sign,
            ),
            _boundary_scores(
                metric,
                ideal,
                item.ordinal,
                Fraction(lower.upper),
                item.use_x_denominator,
                item.denominator_sign,
            ),
        )
        upper_scores = (
            _boundary_scores(
                metric,
                ideal,
                item.ordinal,
                Fraction(upper.lower),
                item.use_x_denominator,
                item.denominator_sign,
            ),
            _boundary_scores(
                metric,
                ideal,
                item.ordinal,
                Fraction(upper.upper),
                item.use_x_denominator,
                item.denominator_sign,
            ),
        )
        if not (
            _brackets_boundary(*lower_scores)
            and _brackets_boundary(*upper_scores)
        ):
            raise AdaptiveDensityFanInvalid(
                "outward ordinal envelope is not production-bound"
            )


def _verify_sealed_local_interval(
    metric,
    ideal,
    ordinal,
    q,
    record,
    sealed,
) -> None:
    """Связать envelope exact predicates без повторного root construction."""

    if not (sealed[0] <= sealed[1] < sealed[2] <= sealed[3]):
        raise AdaptiveDensityFanInvalid(
            "adaptive admissible envelope order is false"
        )
    use_x, denominator_sign, *_ = record
    primitive_vectors = tuple(
        _vector_from_slope(
            slope,
            use_x,
            denominator_sign,
        )
        for slope in sealed
    )
    if not (
        _local_candidate(
            metric,
            ideal,
            ordinal,
            primitive_vectors[1],
            q,
        )
        and _local_candidate(
            metric,
            ideal,
            ordinal,
            primitive_vectors[2],
            q,
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "adaptive inner envelope is not exact-admissible"
        )
    for primitive in (primitive_vectors[0], primitive_vectors[3]):
        if not _local_candidate(
            metric,
            ideal,
            ordinal,
            primitive,
            q,
        ):
            continue
        vector = _vector(*primitive, metric)
        scores = _boundary_scores(
            metric,
            ideal,
            ordinal,
            _slope(vector, use_x, metric),
            use_x,
            denominator_sign,
        )
        if 0 in scores:
            continue
        if any(
            _subturn_boundary(metric, neighbor, vector, q)
            for neighbor in (
                ideal[ordinal - 1],
                ideal[ordinal + 1],
            )
        ):
            continue
        raise AdaptiveDensityFanInvalid(
            "adaptive outward endpoint lies strictly inside"
        )


def with_binding_reasons(
    authority: AdaptiveMinimalRationalFanAuthorityV2,
    reasons: tuple[DirectionBindingReasonV1 | None, ...],
) -> AdaptiveMinimalRationalFanAuthorityV2:
    return replace(authority, binding_reasons=reasons)
