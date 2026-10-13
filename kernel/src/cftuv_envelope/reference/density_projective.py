"""Независимые projective-предикаты и повороты Density-веера."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import gcd

import sympy as sp

from ..contracts.analysis import TurnOrientation
from . import density_residual as _residual
from .density_fan_types import AdaptiveDensityFanInvalid, DensityWindowChartUnrepresentable
from .metric import ExactPlanarMetric


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
    from .density_sign import _density_exact_sign

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
    result = _residual.residual_sign(
        metric,
        dot,
        _dual_dot(metric, left, left),
        _dual_dot(metric, right, right),
        q,
    ) >= 0
    metric._density_exact_memo.subturns[key] = result
    metric._density_exact_memo.subturns[reverse_key] = result
    return result


def _subturn_boundary(metric, left, right, q: int) -> bool:
    dot = _dual_dot(metric, left, right)
    if _sign(dot, metric) < 0:
        return False
    norm_left = _dual_dot(metric, left, left)
    norm_right = _dual_dot(metric, right, right)
    if q == 2:
        return _sign(dot, metric) == 0
    return _residual.residual_sign(metric, dot, norm_left, norm_right, q) == 0


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
