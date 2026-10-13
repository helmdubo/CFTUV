"""Транзакционные векторы и Gram-операции Density без generic normalization."""

from __future__ import annotations

from dataclasses import dataclass

import sympy as sp

from .common import ReferenceGeometryError
from .contracts import ReferenceOutcome
from .density_sign import _density_exact_sign
from .metric import ExactPlanarMetric
from .planar_types import ExactPlanarVector, ExactScalar


@dataclass(frozen=True, slots=True)
class _DensityRuntimeVector:
    """Транзакционный exact-вектор без wire srepr/reparse."""

    x: sp.Expr
    y: sp.Expr

    def expressions(self, transaction_memo=None):
        del transaction_memo
        return self.x, self.y


def _density_runtime_vector(x: sp.Expr, y: sp.Expr) -> _DensityRuntimeVector:
    return _DensityRuntimeVector(sp.sympify(x), sp.sympify(y))


def _density_exact_vector(
    x: sp.Expr,
    y: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    """Записать bounded Density-expression без global factor-canonicalizer."""

    def scalar(expression: sp.Expr) -> ExactScalar:
        expression = sp.sympify(expression)
        if isinstance(expression, sp.Rational):
            return ExactScalar.from_value(expression)
        return ExactScalar(_density_srepr(expression, metric))

    return ExactPlanarVector(scalar(x), scalar(y))


def _density_srepr(
    expression: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> str:
    """Сериализовать expression с memo конкретной metric-транзакции."""

    expression = sp.sympify(expression)
    if metric is None:
        return sp.srepr(expression)
    cached = metric._density_exact_memo.sreprs.get(expression)
    if cached is not None:
        return cached
    result = sp.srepr(expression)
    metric._density_exact_memo.sreprs[expression] = result
    metric._density_exact_memo.expressions[result] = expression
    return result


def _density_dot_expression(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    """Dual-free Gram dot без generic normalization/factor."""

    lx, ly = metric.density_expressions(left)
    rx, ry = metric.density_expressions(right)
    remembered = metric._density_exact_memo.dot_expressions
    key = (lx, ly, rx, ry)
    cached = remembered.get(key)
    if cached is None:
        cached = remembered[key] = (
            lx * (metric.gram[0][0] * rx + metric.gram[0][1] * ry)
            + ly * (metric.gram[1][0] * rx + metric.gram[1][1] * ry)
        )
    return cached


def _density_unit_from_squared(
    vector: ExactPlanarVector,
    squared: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    if _density_exact_sign(squared, metric) <= 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A support direction has non-positive Gram norm",
        )
    if metric is not None:
        # Чистая функция `(x, y, squared)`: единичный вектор из квадрата. Повтор берётся из памяти
        # транзакции; отказ (неположительная норма) в память не попадает и повторяется как был.
        x, y = metric.density_expressions(vector)
        remembered = metric._density_exact_memo.unit_vectors
        key = (x, y, squared)
        cached = remembered.get(key)
        if cached is not None:
            return cached
    if metric is None:
        x, y = vector.expressions()
    length = sp.sqrt(squared)
    unit = _density_runtime_vector(x / length, y / length)
    if metric is not None:
        remembered[key] = unit
    return unit


def _density_left_unit_normal(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
) -> ExactPlanarVector:
    tx, ty = metric.density_expressions(tangent)
    covector_x = metric.gram[0][0] * tx + metric.gram[0][1] * ty
    covector_y = metric.gram[1][0] * tx + metric.gram[1][1] * ty
    sign = metric.owner_orientation_sign
    raw = _density_runtime_vector(
        -sign * covector_y,
        sign * covector_x,
    )
    orthogonality = sp.expand(
        _density_dot_expression(metric, tangent, raw)
    )
    if orthogonality != 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A Gram-orthogonal basis construction is not exact",
        )
    return _density_unit_from_squared(
        raw,
        sp.expand(_density_dot_expression(metric, raw, raw)),
        metric,
    )
