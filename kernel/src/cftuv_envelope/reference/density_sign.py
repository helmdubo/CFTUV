"""Точный знак Density-фактов с памятью одной metric-транзакции."""

from __future__ import annotations

import sympy as sp
from mpmath import iv

from .._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
from .common import ReferenceGeometryError
from .contracts import ReferenceOutcome
from .metric import ExactPlanarMetric
from .planar_types import ExactQuadraticFieldUnsupported, exact_quadratic_value


def _density_exact_sign(
    expression: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> int:
    """Знак Density-факта с memo конкретной metric-транзакции."""

    expression = sp.sympify(expression)
    if metric is not None:
        cached = metric._density_exact_memo.signs.get(expression)
        if cached is not None:
            return cached
    if isinstance(expression, sp.Rational):
        return (expression.p > 0) - (expression.p < 0)
    if expression == 0:
        return 0
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = density_interval_enclosure(
            expression,
            (
                None
                if metric is None
                else metric._density_exact_memo.intervals
            ),
        )
    except (
        DensityIntervalEnclosureUnsupported,
        ArithmeticError,
        TypeError,
        ValueError,
    ):
        enclosure = None
    finally:
        iv.prec = saved
    if enclosure is not None:
        if enclosure.a > 0:
            result = 1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
        if enclosure.b < 0:
            result = -1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
    try:
        result = exact_quadratic_value(sp.expand(expression)).sign()
        if metric is not None:
            metric._density_exact_memo.signs[expression] = result
        return result
    except ExactQuadraticFieldUnsupported:
        pass
    raise ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
        "Density A exact sign is not certified without generic factorization",
    )
