"""Density-рациональность, восстановление нормали и замкнутая decimal-огибающая."""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction

from mpmath import iv, libmp

from .._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from .density_binding_types import (
    BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    DirectionBindingCertificateUnproven,
    _DECIMAL_DIGITS,
    _DECIMAL_SCALE,
)
from .metric import ExactPlanarMetric
from .planar_types import ExactPlanarVector
from .radical_rationality import radical_ratio_is_rational


def density_support_direction_rationality(
    metric: ExactPlanarMetric,
    unit_normal: ExactPlanarVector,
) -> bool | None:
    """Рациональна ли проективная запись Density-ковектора: ТОЧНО или «не решено».

    `True` / `False` — решение в точном поле корней из рациональных
    (`radical_rationality`, без факторизации и потому без неоплаченной
    работы), а не по виду выражения: `(1 + sqrt 3, 2 + 2 sqrt 3)` — направление
    `(1, 2)`, хотя `ratio.is_Rational` у SymPy для него `False` (признак класса
    выражения, не значения).

    `None` — выражение вне поля (нераскрытый `cos`/`sin` угла, корень из
    суммы): оно не называется ни рациональным, ни иррациональным. Потребитель,
    которому нужна ДОКАЗАННАЯ иррациональность (лифт «на пределе»), на `None`
    не срабатывает.
    """

    from .density_sign import _density_exact_sign

    x, y = metric.density_expressions(unit_normal)
    gram = metric.gram
    covector_x = gram[0][0] * x + gram[0][1] * y
    covector_y = gram[1][0] * x + gram[1][1] * y
    if _density_exact_sign(covector_x, metric) == 0:
        return _density_exact_sign(covector_y, metric) != 0
    if (covector_y / covector_x).is_Rational:
        return True
    return radical_ratio_is_rational(covector_y, covector_x)


def has_rational_density_support_direction(
    metric: ExactPlanarMetric,
    unit_normal: ExactPlanarVector,
) -> bool:
    """Доказанно ли направление Density-ковектора рациональным (иначе — привязка)."""

    return density_support_direction_rationality(metric, unit_normal) is True


def bound_unit_normal_from_vector(
    metric: ExactPlanarMetric,
    vector: tuple[int, int],
) -> ExactPlanarVector:
    """Восстановить G-единичную нормаль из примитивного ковектора V2."""

    from .density_vectors import _density_runtime_vector, _density_unit_from_squared

    x, y = vector
    inverse = metric.inverse_gram
    normal = _density_runtime_vector(
        inverse[0][0] * x + inverse[0][1] * y,
        inverse[1][0] * x + inverse[1][1] * y,
    )
    nx, ny = metric.density_expressions(normal)
    return _density_unit_from_squared(
        normal,
        x * nx + y * ny,
        metric,
    )


def _density_decimal_envelope(expression, metric=None):
    """Decimal-envelope только через замкнутую Density authority."""

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
        ValueError,
        TypeError,
    ):
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    finally:
        iv.prec = saved
    lower, upper = (
        Fraction(*libmp.to_rational(item)) for item in enclosure._mpi_
    )
    lower_units = lower.numerator * _DECIMAL_SCALE // lower.denominator
    upper_units = -(
        (-upper.numerator * _DECIMAL_SCALE) // upper.denominator
    )
    lower_decimal = Decimal(f"{lower_units}E-{_DECIMAL_DIGITS}")
    upper_decimal = Decimal(f"{upper_units}E-{_DECIMAL_DIGITS}")
    return CertifiedDecimalIntervalV1(
        lower_decimal,
        upper_decimal,
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        upper_decimal - lower_decimal,
    )
