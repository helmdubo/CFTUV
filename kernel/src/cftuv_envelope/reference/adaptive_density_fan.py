"""Адаптивная минимальная рациональная власть Density-веера."""

from __future__ import annotations

from dataclasses import dataclass, replace
from decimal import Decimal
from fractions import Fraction
from functools import cmp_to_key
from math import gcd, isqrt

import sympy as sp

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
    search_global_height,
    search_segments,
    termination_piece,
    validate_atlas_structure,
)
from .common import stable_id
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

# Объявленный кап точной работы поиска минимального D*.
#
# Высота одна не является ресурсом: исчерпывающий проход по высотам стоит
# квадратично, поэтому `_RESOURCE_HEIGHT_CAP` ограничивает не работу, а только
# номер последней высоты, и при узком окне фронт уходит в счёт без исхода.
#
# Закон бюджета D*. Доказанная высота останова (закон Фарея,
# `_termination_height`) для окна ширины w равна floor(1/w) + 2, а atlas
# поднимает её в chart-invariant норму множителем модуля коробки; на поле
# она достигает 4.3e7 и 1.0e8, то есть высота останова НЕ является бюджетом.
# Стоимость же прохода известна точно:
#   * одна карта — внутри sealed-интервала перечисляются только числители,
#     то есть примерно w*h + 1 пробников на высоту h и окно;
#   * atlas — перебирается вся оболочка max-нормы, а примитивных ковекторов
#     ровно высоты h не больше 8h (периметр квадрата [-h, h]^2), то есть
#     проход по высотам 1..H для веера из k окон стоит не больше
#     sum_{h<=H} 8*h*k = 4*H*(H+1)*k пробников.
#
# Литерал выведен из худшего случая ЗАМОРОЖЕННОГО корпуса, а не из потолка:
# самая глубокая зелёная власть поля (building.002, explicit density 4,
# corner relation 60fc81c8…) имеет минимальную общую высоту 4492 при k = 2 и
# тратит ровно 9 035 единиц (см. тест
# `test_deepest_green_field_authority_stays_far_below_the_work_cap`).
# Кап — эта величина с запасом в 14 раз, округлённая до степени двойки:
#   9 035 * 14.5 ~ 131 000  ->  1 << 17 = 131 072.
# В atlas-ветви тот же литерал по закону 4*H*(H+1) означает исчерпывающий
# chart-invariant проход примерно до высоты 180.
#
# Кап тратится на все k окон сразу: больший k честно уменьшает достижимую
# высоту, а не удорожает прогон.
_DENSITY_EXACT_WORK_CAP = 1 << 17


class DensityRationalAuthorityExhausted(ValueError):
    """Именованный отказ только при превышении доказанного resource cap."""


class AdaptiveDensityFanInvalid(ValueError):
    """Запись V2 не следует из production metric/ideal facts."""


class DensityWindowChartUnrepresentable(AdaptiveDensityFanInvalid):
    """Допустимое projective-окно не представимо принятым chart-контрактом."""


def _work_budget(cap: int | None = None) -> DensityExactWorkBudget:
    """Свежий счётчик точной работы одной транзакции поиска D*."""

    return DensityExactWorkBudget(
        DensityRationalAuthorityExhausted,
        _DENSITY_EXACT_WORK_CAP if cap is None else cap,
    )


@dataclass(frozen=True, slots=True)
class _DensityVector:
    """Транзакционный exact-вектор без wire srepr/reparse."""

    x: sp.Expr
    y: sp.Expr

    def expressions(self, transaction_memo=None):
        del transaction_memo
        return self.x, self.y


def _ratio(value: Fraction) -> tuple[int, int]:
    return value.numerator, value.denominator


def _fraction(value: tuple[int, int]) -> Fraction:
    numerator, denominator = value
    if (
        type(numerator) is not int
        or type(denominator) is not int
        or denominator <= 0
        or gcd(abs(numerator), denominator) != 1
    ):
        raise AdaptiveDensityFanInvalid("fan rational is not reduced")
    return Fraction(numerator, denominator)


def _height(vector: tuple[int, int]) -> int:
    return max(abs(vector[0]), abs(vector[1]))


def _expected_orientation(orientation: TurnOrientation) -> int:
    return (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )


def _dual_dot(metric, left, right):
    lx, ly = metric.density_expressions(left)
    rx, ry = metric.density_expressions(right)
    key = (lx, ly, rx, ry)
    cached = metric._density_exact_memo.dual_dots.get(key)
    if cached is not None:
        return cached
    inverse = metric.inverse_gram
    result = (
        lx * (inverse[0][0] * rx + inverse[0][1] * ry)
        + ly * (inverse[1][0] * rx + inverse[1][1] * ry)
    )
    metric._density_exact_memo.dual_dots[key] = result
    metric._density_exact_memo.dual_dots[(rx, ry, lx, ly)] = result
    return result


def _oriented_cross(metric, left, right):
    lx, ly = metric.density_expressions(left)
    rx, ry = metric.density_expressions(right)
    key = (lx, ly, rx, ry)
    cached = metric._density_exact_memo.crosses.get(key)
    if cached is not None:
        return cached
    result = metric.owner_orientation_sign * (lx * ry - ly * rx)
    metric._density_exact_memo.crosses[key] = result
    metric._density_exact_memo.crosses[(rx, ry, lx, ly)] = -result
    return result


def _sign(expression, metric: ExactPlanarMetric | None = None) -> int:
    from .angular import _density_exact_sign

    return _density_exact_sign(expression, metric)


def _vector(
    x,
    y,
    metric: ExactPlanarMetric | None = None,
) -> _DensityVector:
    del metric
    return _DensityVector(sp.sympify(x), sp.sympify(y))


def _inside_ordinal(metric, candidate, previous, ideal, following) -> bool:
    """Voronoi-окно ordinal без материализации двух midpoint-лучей."""

    px, py = metric.density_expressions(previous)
    ix, iy = metric.density_expressions(ideal)
    fx, fy = metric.density_expressions(following)
    toward_ideal_from_previous = _vector(
        ix - px,
        iy - py,
        metric,
    )
    toward_ideal_from_following = _vector(
        ix - fx,
        iy - fy,
        metric,
    )
    return (
        _sign(
            _dual_dot(metric, candidate, toward_ideal_from_previous),
            metric,
        )
        > 0
        and _sign(
            _dual_dot(metric, candidate, toward_ideal_from_following),
            metric,
        )
        > 0
    )


def _slope(
    vector,
    use_x: bool,
    metric: ExactPlanarMetric | None = None,
):
    x, y = (
        vector.expressions()
        if metric is None
        else metric.density_expressions(vector)
    )
    denominator = x if use_x else y
    if _sign(denominator, metric) == 0:
        raise AdaptiveDensityFanInvalid("fan chart denominator is zero")
    return (y if use_x else x) / denominator


def _vector_from_slope(
    slope: Fraction,
    use_x: bool,
    denominator_sign: int,
) -> tuple[int, int]:
    numerator = slope.numerator * denominator_sign
    denominator = slope.denominator * denominator_sign
    return (
        (denominator, numerator)
        if use_x
        else (numerator, denominator)
    )


def _chart_for_window(
    metric,
    ideal,
    left,
    right,
) -> tuple[bool, int]:
    """Выбрать один неособый projective chart всего ordinal-окна."""

    ix, iy = metric.density_expressions(ideal)
    preferred = _sign(ix * ix - iy * iy, metric) >= 0
    for use_x in (preferred, not preferred):
        denominator_index = 0 if use_x else 1
        values = (
            metric.density_expressions(left)[denominator_index],
            metric.density_expressions(ideal)[denominator_index],
            metric.density_expressions(right)[denominator_index],
        )
        signs = tuple(_sign(value, metric) for value in values)
        if 0 in signs or len(set(signs)) != 1:
            continue
        return use_x, signs[0]
    raise DensityWindowChartUnrepresentable(
        "DENSITY_WINDOW_CHART_UNREPRESENTABLE"
    )


def _window_envelopes(ideal):
    from .direction_binding import _density_decimal_envelope

    windows = []
    records = []
    metric = ideal.metric
    for ordinal in range(1, len(ideal) - 1):
        left_values = metric.density_expressions(ideal[ordinal - 1])
        center_values = metric.density_expressions(ideal[ordinal])
        right_values = metric.density_expressions(ideal[ordinal + 1])
        # Для projective slope нормирующий множитель midpoint не нужен.
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
            use_x, denominator_sign = _chart_for_window(
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
                    sign=_sign,
                    slope=_slope,
                    vector=_vector,
                    decimal_envelope=_density_decimal_envelope,
                    invalid=DensityWindowChartUnrepresentable,
                )
            )
            continue
        slopes = (
            _slope(left, use_x, metric),
            _slope(right, use_x, metric),
        )
        lower_exact, upper_exact = (
            slopes
            if _sign(slopes[1] - slopes[0], metric) > 0
            else (slopes[1], slopes[0])
        )
        lower = _density_decimal_envelope(lower_exact, metric)
        upper = _density_decimal_envelope(upper_exact, metric)
        if lower.upper >= upper.lower:
            raise AdaptiveDensityFanInvalid(
                "ordinal rational envelopes overlap"
            )
        windows.append((left, right))
        records.append((use_x, denominator_sign, lower, upper))
    return tuple(windows), tuple(records)


class _IdealTuple(tuple):
    metric: ExactPlanarMetric


def _covectors(metric, ideal_unit_normals):
    covectors = []
    for normal in ideal_unit_normals:
        nx, ny = metric.density_expressions(normal)
        covectors.append(
            _vector(
                metric.gram[0][0] * nx + metric.gram[0][1] * ny,
                metric.gram[1][0] * nx + metric.gram[1][1] * ny,
                metric,
            )
        )
    values = _IdealTuple(covectors)
    values.metric = metric
    return values


def _candidate_vector(
    slope: Fraction,
    use_x: bool,
    denominator_sign: int,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    return _vector(
        *_vector_from_slope(slope, use_x, denominator_sign),
        metric,
    )


def _subturn(metric, left, right, q: int) -> bool:
    left_values = metric.density_expressions(left)
    right_values = metric.density_expressions(right)
    key = (
        *left_values,
        *right_values,
        q,
    )
    reverse_key = (
        *right_values,
        *left_values,
        q,
    )
    cached = metric._density_exact_memo.subturns.get(key)
    if cached is not None:
        return cached
    dot = _dual_dot(metric, left, right)
    dot_sign = _sign(dot, metric)
    if q == 2:
        result = dot_sign >= 0
        metric._density_exact_memo.subturns[key] = result
        metric._density_exact_memo.subturns[reverse_key] = result
        return result
    if dot_sign <= 0:
        metric._density_exact_memo.subturns[key] = False
        metric._density_exact_memo.subturns[reverse_key] = False
        return False
    norm_product = _dual_dot(metric, left, left) * _dual_dot(
        metric,
        right,
        right,
    )
    dot_squared = dot * dot
    if q == 3:
        residual = 4 * dot_squared - norm_product
    elif q == 4:
        residual = 2 * dot_squared - norm_product
    elif q == 5:
        residual = (
            8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
        )
    elif q == 6:
        residual = 4 * dot_squared - 3 * norm_product
    else:
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    result = _sign(residual, metric) >= 0
    metric._density_exact_memo.subturns[key] = result
    metric._density_exact_memo.subturns[reverse_key] = result
    return result


def _subturn_boundary(metric, left, right, q: int) -> bool:
    dot = _dual_dot(metric, left, right)
    if _sign(dot, metric) < 0:
        return False
    norm_product = _dual_dot(metric, left, left) * _dual_dot(
        metric,
        right,
        right,
    )
    dot_squared = dot * dot
    if q == 2:
        residual = dot
    elif q == 3:
        residual = 4 * dot_squared - norm_product
    elif q == 4:
        residual = 2 * dot_squared - norm_product
    elif q == 5:
        residual = (
            8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
        )
    elif q == 6:
        residual = 4 * dot_squared - 3 * norm_product
    else:
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    return _sign(residual, metric) == 0


def _quarter_turn(metric, vector, orientation: int):
    """Dual-Gram перпендикуляр с заданным oriented-cross знаком."""

    x, y = metric.density_expressions(vector)
    inverse = metric.inverse_gram
    dual_x = inverse[0][0] * x + inverse[0][1] * y
    dual_y = inverse[1][0] * x + inverse[1][1] * y
    determinant = (
        inverse[0][0] * inverse[1][1]
        - inverse[0][1] * inverse[1][0]
    )
    scale = (
        orientation
        * metric.owner_orientation_sign
        / sp.sqrt(determinant)
    )
    return _vector(
        -scale * dual_y,
        scale * dual_x,
        metric,
    )


def _exact_q_trig(q: int):
    if q == 2:
        return sp.Integer(0), sp.Integer(1)
    if q == 3:
        return sp.Rational(1, 2), sp.sqrt(3) / 2
    if q == 4:
        value = sp.sqrt(2) / 2
        return value, value
    if q == 5:
        return (
            (1 + sp.sqrt(5)) / 4,
            sp.sqrt(10 - 2 * sp.sqrt(5)) / 4,
        )
    if q == 6:
        return sp.sqrt(3) / 2, sp.Rational(1, 2)
    raise AdaptiveDensityFanInvalid("unsupported Density q")


def _rotate_q(metric, vector, orientation: int, q: int):
    cosine, sine = _exact_q_trig(q)
    perpendicular = _quarter_turn(metric, vector, orientation)
    x, y = metric.density_expressions(vector)
    px, py = metric.density_expressions(perpendicular)
    return _vector(
        cosine * x + sine * px,
        cosine * y + sine * py,
        metric,
    )


def _subturn_boundary_vectors(metric, ideal, ordinal, orientation, q):
    """Два крайних луча subturn-ограничений ordinal — без всякой карты."""

    expected = _expected_orientation(orientation)
    return (
        _rotate_q(metric, ideal[ordinal - 1], expected, q),
        _rotate_q(metric, ideal[ordinal + 1], -expected, q),
    )


def _sealed_local_interval(
    metric,
    ideal,
    ordinal,
    orientation,
    q,
    record,
):
    """Один раз запечатать inner/outward рациональную оболочку окна."""

    from .direction_binding import _density_decimal_envelope

    use_x, _, ordinal_lower, ordinal_upper = record
    boundary_vectors = _subturn_boundary_vectors(
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
        if _sign(denominator, metric) == 0:
            continue
        boundaries.append(
            _density_decimal_envelope(
                _slope(boundary_vector, use_x, metric),
                metric,
            )
        )
    center_envelope = _density_decimal_envelope(
        _slope(ideal[ordinal], use_x, metric),
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
        )
    ):
        raise AdaptiveDensityFanInvalid(
            "inner rational envelope is not admissible"
        )
    return sealed


def _box_is_feasible(metric, ideal, orientation, q, boxes) -> bool:
    expected = _expected_orientation(orientation)
    endpoints = []
    for ordinal, (lower, upper, use_x, sign) in enumerate(boxes, start=1):
        pair = (
            _candidate_vector(lower, use_x, sign, metric),
            _candidate_vector(upper, use_x, sign, metric),
        )
        if any(
            not _inside_ordinal(
                metric,
                item,
                ideal[ordinal - 1],
                ideal[ordinal],
                ideal[ordinal + 1],
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
            _sign(
                _oriented_cross(metric, left, right),
                metric,
            )
            != expected
            for left, right in zip(sequence, sequence[1:])
        ):
            return False
        if any(
            not _subturn(metric, left, right, q)
            for left, right in zip(sequence, sequence[1:])
        ):
            return False
    return True


def _termination_boxes(metric, ideal, orientation, q, records):
    centers = []
    gaps = []
    for ordinal, (use_x, _, lower, upper) in enumerate(records, start=1):
        from .direction_binding import _density_decimal_envelope

        ideal_envelope = _density_decimal_envelope(
            _slope(ideal[ordinal], use_x, metric),
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
    divisor = 1 << 24
    for _ in range(_BOX_REFINEMENT_CAP):
        boxes = tuple(
            (
                center - gap / divisor,
                center + gap / divisor,
                records[index][0],
                records[index][1],
            )
            for index, (center, gap) in enumerate(zip(centers, gaps))
        )
        if _box_is_feasible(metric, ideal, orientation, q, boxes):
            return boxes
        divisor *= 2
    raise AdaptiveDensityFanInvalid(
        "positive full-fan termination width is not proven"
    )


def _termination_height(boxes) -> int:
    bounds = []
    for lower, upper, _, _ in boxes:
        width = upper - lower
        if width <= 0:
            raise AdaptiveDensityFanInvalid(
                "fan termination width is not positive"
            )
        bounds.append(width.denominator // width.numerator + 2)
    return max(bounds, default=0)


def _local_record(record):
    return record.local_record if type(record) is AtlasWindowPrepared else record


def _local_candidate(metric, ideal, ordinal, vector, q) -> bool:
    candidate = _vector(*vector, metric)
    return (
        _inside_ordinal(
            metric,
            candidate,
            ideal[ordinal - 1],
            ideal[ordinal],
            ideal[ordinal + 1],
        )
        and _subturn(
            metric,
            ideal[ordinal - 1],
            candidate,
            q,
        )
        and _subturn(
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
) -> tuple[int, int]:
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
):
    """Высота Farey: integer-only внутри inner, exact только в shell."""

    use_x, denominator_sign, *_ = record
    outer_lower, inner_lower, inner_upper, outer_upper = (
        sealed_interval
    )
    first, last = _strict_numerator_bounds(
        outer_lower,
        outer_upper,
        height,
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


def _segment_plan_laws() -> SegmentPlanLaws:
    """Точные предикаты, которыми sibling-модуль atlas не владеет."""

    from .direction_binding import _density_decimal_envelope

    return SegmentPlanLaws(
        sign=_sign,
        slope=_slope,
        decimal_envelope=_density_decimal_envelope,
        local_candidate=_local_candidate,
        vector_from_slope=_vector_from_slope,
        subturn_boundary_vectors=_subturn_boundary_vectors,
        invalid=DensityWindowChartUnrepresentable,
    )


def _objective(metric, ideal, ordinal, vector):
    """Cos² Gram-score; его максимум равен минимуму старого расстояния."""

    candidate = _vector(*vector, metric)
    dot = _dual_dot(metric, candidate, ideal[ordinal])
    norm = _dual_dot(metric, candidate, candidate)
    return dot * dot / norm


def _candidate_compare(metric, ideal, ordinal):
    from .direction_binding import _density_decimal_envelope

    scores = {}
    envelopes = {}

    def score(vector):
        if vector not in scores:
            scores[vector] = _objective(metric, ideal, ordinal, vector)
        return scores[vector]

    def envelope(vector):
        if vector not in envelopes:
            envelopes[vector] = _density_decimal_envelope(
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
        sign = _sign(
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


def _legacy_authority_budget(width: Fraction) -> int:
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
) -> DirectionBindingCertificateV1 | None:
    from .direction_binding import _PROVEN_PREDICATES

    if type(record) is AtlasWindowPrepared:
        return None
    use_x, denominator_sign, lower_envelope, upper_envelope = record
    lower = Fraction(lower_envelope.upper)
    upper = Fraction(upper_envelope.lower)
    width = upper - lower
    if width <= 0:
        raise AdaptiveDensityFanInvalid("legacy Density window is empty")
    budget = _legacy_authority_budget(width)
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
            candidate = _vector(*vector, metric)
            if not (
                _subturn(metric, ideal[ordinal - 1], candidate, q)
                and _subturn(metric, candidate, ideal[ordinal + 1], q)
            ):
                continue
            score = _objective(metric, ideal, ordinal, vector)
            if (
                winner is None
                or _sign(score - winner[0], metric) > 0
                or (
                    _sign(score - winner[0], metric) == 0
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
            upper_envelope.lower - lower_envelope.upper
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
) -> bool:
    from .direction_binding import _PROVEN_PREDICATES

    expected = _expected_orientation(orientation)
    resolved = tuple(
        ideal[ordinal]
        if certificate is None
        else _vector(
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
        _sign(_oriented_cross(metric, left, right), metric) != expected
        or not _subturn(metric, left, right, q)
        for left, right in zip(sequence, sequence[1:])
    ):
        return False
    return all(
        certificate is None
        or (
            type(record) is not AtlasWindowPrepared
            and _inside_ordinal(
                metric,
                candidate,
                ideal[ordinal - 1],
                ideal[ordinal],
                ideal[ordinal + 1],
            )
            and (
                Fraction(record[2].upper)
                < Fraction(_slope(candidate, record[0], metric))
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
) -> tuple[
    tuple[DirectionBindingCertificateV1 | None, ...],
    AdaptiveMinimalRationalFanAuthorityV2 | None,
]:
    """Одна транзакция B(w) и, при пустоте, adaptive-власти.

    Обе ветви закона используют одни production covectors и ordinal windows.
    Повторное их построение здесь было бы не независимой проверкой, а только
    повтором той же власти внутри одной compile-транзакции.
    """

    if max_subturn_q not in range(2, 7):
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    if len(ideal_unit_normals) < 3:
        return (), None
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
    legacy_full_fan_valid = _legacy_full_fan_valid(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        certificates,
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


def _best_fan(metric, ideal, orientation, q, candidates, budget):
    expected = _expected_orientation(orientation)
    budget.spend_order_steps(sum(len(items) for items in candidates))
    ordered = tuple(
        tuple(
            sorted(
                items,
                key=cmp_to_key(
                    _candidate_compare(metric, ideal, ordinal)
                ),
            )
        )
        for ordinal, items in enumerate(candidates, start=1)
    )

    def visit(ordinal, prefix):
        if ordinal == len(ordered):
            last = (
                ideal[0]
                if not prefix
                else _vector(*prefix[-1], metric)
            )
            return (
                prefix
                if _sign(
                    _oriented_cross(metric, last, ideal[-1]),
                    metric,
                )
                == expected
                and _subturn(
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
            else _vector(*prefix[-1], metric)
        )
        for vector in ordered[ordinal]:
            budget.spend_order_steps(1)
            candidate = _vector(*vector, metric)
            if (
                _sign(
                    _oriented_cross(metric, previous, candidate),
                    metric,
                )
                != expected
                or not _subturn(
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
):
    candidates = [set() for _ in records]
    previous_counts = tuple(0 for _ in records)
    for height in range(1, stop_height + 1):
        for ordinal, record in enumerate(records, start=1):
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
                )
            )
        fan = (
            _best_fan(metric, ideal, orientation, q, candidates, budget)
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


def _authority_id(q, height, fan, windows) -> str:
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
) -> AdaptiveMinimalRationalFanAuthorityV2:
    """Построить одну V2-власть полного coupled Density-веера."""

    ideal = _covectors(metric, ideal_unit_normals)
    _, records = _window_envelopes(ideal)
    return _certify_adaptive_density_fan_prepared(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        records,
        binding_reasons=binding_reasons,
        resource_height_cap=resource_height_cap,
    )


def _authority_windows(records, boxes, sealed_intervals):
    """Собрать wire-окна власти: atlas и одно-картное — по своей форме."""

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
) -> AdaptiveMinimalRationalFanAuthorityV2:
    """Достроить V2 из фактов, уже выведенных в этой транзакции."""

    local_records = tuple(_local_record(record) for record in records)
    uses_atlas = any(type(record) is AtlasWindowPrepared for record in records)
    boxes = _termination_boxes(
        metric,
        ideal,
        orientation,
        max_subturn_q,
        local_records,
    )
    sealed_intervals = tuple(
        _sealed_local_interval(
            metric,
            ideal,
            ordinal,
            orientation,
            max_subturn_q,
            local_record,
        )
        for ordinal, local_record in enumerate(local_records, start=1)
    )
    termination_height = (
        global_termination_height(
            boxes,
            local_height=_termination_height,
        )
        if uses_atlas
        else _termination_height(boxes)
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
            max_subturn_q,
            records,
            sealed_intervals,
            stop_height,
            _work_budget(),
        )
    )
    if fan is None:
        raise DensityRationalAuthorityExhausted(
            "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
        )
    windows = _authority_windows(records, boxes, sealed_intervals)
    authority_id = _authority_id(max_subturn_q, height, fan, windows)
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
        proven_predicates=ADAPTIVE_FAN_PROVEN_PREDICATES,
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
        or supplied.proven_predicates != ADAPTIVE_FAN_PROVEN_PREDICATES
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

    ideal = _covectors(metric, ideal_unit_normals)
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
    px, py = metric.density_expressions(ideal[ordinal - 1])
    ix, iy = metric.density_expressions(ideal[ordinal])
    fx, fy = metric.density_expressions(ideal[ordinal + 1])
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
    left_values = metric.density_expressions(ideal[ordinal - 1])
    center_values = metric.density_expressions(ideal[ordinal])
    right_values = metric.density_expressions(ideal[ordinal + 1])
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
            ideal[ordinal - 1],
            ideal[ordinal],
            ideal[ordinal + 1],
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
