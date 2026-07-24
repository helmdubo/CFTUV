"""Мера рефлексного угла в долях π: точная дробь либо доказанная оболочка.

Хост остаётся измерительным органом для `CertifiedReflexAngleMeasureV1`, но
сама арифметика точная и потому принадлежит ядру.

Путей два, и первый не менялся:

1. `atan2(|sin|, cos)/π` вычисляется точно и оказывается рациональным
   (90° → 1/2, 45° → 1/4). Интервал вырождается в саму дробь. Дайджесты
   принятого корпуса зависят от этой ветки побитово — вплоть до того, что
   `Decimal.scaleb` округляет по точности контекста; выражения сохранены
   дословно.

2. Дробь не рациональна. Так выглядит почти любой реальный прямой угол:
   в binary64 координатах 90° приходит как 89.9996°, и доля π равна
   0.4999980324. Раньше это отвергало патч целиком
   (`EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`), хотя ниже по конвейеру значение
   всё равно превращалось в интервал. Теперь строится строгая интервальная
   оболочка того же выражения: `mpmath.iv` округляет наружу на каждой
   операции, поэтому истинное значение гарантированно внутри.

Оболочка — сертификат, а не допуск: порога здесь нет. Она либо доказывает
`π < φ < 2π`, либо даёт именованный отказ. Тот же приём, которым
`planar_types._certified_interval_sign` доказывает знак.
"""

from __future__ import annotations

import math
from decimal import Decimal
from fractions import Fraction

import sympy as sp
from mpmath import iv, libmp

from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from .planar_types import IntervalEnclosureUnsupported, interval_enclosure


class CertifiedAngleUnavailable(ValueError):
    """Именованный отказ: доля π не точна и не заключена в доказанную оболочку."""


# Столько десятичных знаков записывается в сертификат. Число унаследовано от
# точного пути и не меняется: от него зависят дайджесты принятого корпуса.
DECIMALS = 28
_SCALE = 10**DECIMALS

# Битовая точность mpmath. 28 десятичных знаков — это ~93 бита; запас нужен,
# чтобы оболочка сходилась заведомо тоньше последнего записываемого знака,
# а не упиралась в него.
_ENCLOSURE_PRECISION = 256


def angular_fraction_of_pi(sine: sp.Expr, cosine: sp.Expr) -> sp.Expr:
    """`atan2(|sin|, cos)/π` в том же виде, в каком его считал хост."""

    return sp.factor(sp.atan2(abs(sine), cosine) / sp.pi)


def reflex_angle_intervals_over_pi(
    sine: sp.Expr, cosine: sp.Expr
) -> tuple[CertifiedDecimalIntervalV1, CertifiedDecimalIntervalV1]:
    """`(φ/π, δ/π)`: точная дробь, если она есть, иначе доказанная оболочка.

    Оба интервала всегда приходят из одного пути. Смешивать их нельзя: сверка
    «δ равна φ минус единица» перестала бы что-либо проверять, начав сравнивать
    результаты разных вычислений.
    """

    exact = rational_intervals_over_pi(angular_fraction_of_pi(sine, cosine))
    if exact is not None:
        return exact
    return certified_intervals_over_pi(sine, cosine)


def rational_intervals_over_pi(
    fraction: sp.Expr,
) -> tuple[CertifiedDecimalIntervalV1, CertifiedDecimalIntervalV1] | None:
    """`(φ/π, δ/π)` для точной рациональной доли, либо None — «отвечать не мне».

    Это фильтр: он отвечает только тогда, когда 28 десятичных знаков
    представляют φ ровно, и уступает оболочке во всех остальных случаях.
    Ошибиться он не может — расхождение φ и δ ловится сверкой ниже.
    """

    if fraction.is_Rational is not True:
        return None
    phi = sp.factor(1 + fraction)
    interval_phi = _rational_interval(phi)
    # Сравнение с числом безопасно только здесь: ветка достижима лишь для
    # Rational, где SymPy возвращает настоящий bool. На символьном выражении
    # это была бы `Relational` — объект, истинный в `if` всегда.
    if phi >= 2:
        raise CertifiedAngleUnavailable(
            "exact 2*pi corner must be Terminal/Junction, not Angular"
        )
    interval_delta = CertifiedDecimalIntervalV1(
        interval_phi.lower - Decimal(1),
        interval_phi.upper - Decimal(1),
        interval_phi.lower_kind,
        interval_phi.upper_kind,
        interval_phi.absolute_error_bound,
    )
    exact_delta_interval = _rational_interval(fraction)
    if (
        interval_delta.lower != exact_delta_interval.lower
        or interval_delta.upper != exact_delta_interval.upper
    ):
        # φ и δ посчитаны по отдельности и разошлись, значит точная запись
        # границу не удержала: у φ ∈ (1, 2) на значащую цифру больше, чем
        # оставляет `scaleb`, и δ = 1/3 отбрасывает последний знак. Отвечает
        # оболочка — она эти углы заключает без потери разряда. Отказом это
        # быть не может: угол существует и измерим, не измерим он только так.
        return None
    return interval_phi, interval_delta


def certified_intervals_over_pi(
    sine: sp.Expr, cosine: sp.Expr
) -> tuple[CertifiedDecimalIntervalV1, CertifiedDecimalIntervalV1]:
    """`(φ/π, δ/π)` из строгой интервальной оболочки `atan2(|sin|, cos)/π`."""

    bounds = _enclose_fraction_of_pi(sine, cosine)
    if bounds is None:
        raise CertifiedAngleUnavailable(
            "angular ratio has no certified interval enclosure: "
            f"atan2(|{sine}|, {cosine})/pi"
        )
    lower, upper = bounds
    lower_units = math.floor(lower * _SCALE)
    upper_units = math.ceil(upper * _SCALE)
    # `π < φ < 2π` решается оболочкой, а не сравнением символьного выражения:
    # δ/π обязана лежать строго внутри (0, 1). Граница, которую оболочка не
    # разделила, — это неразличимые π и 2π, то есть Terminal/Junction.
    if lower_units <= 0 or upper_units >= _SCALE:
        raise CertifiedAngleUnavailable(
            "certified angular enclosure does not prove pi < phi < 2*pi: "
            f"phi/pi in [{_scaled(lower_units + _SCALE)}, "
            f"{_scaled(upper_units + _SCALE)}]"
        )
    return (
        _interval(_scaled(lower_units + _SCALE), _scaled(upper_units + _SCALE)),
        _interval(_scaled(lower_units), _scaled(upper_units)),
    )


def _interval(lower: Decimal, upper: Decimal) -> CertifiedDecimalIntervalV1:
    return CertifiedDecimalIntervalV1(
        lower,
        upper,
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        upper - lower,
    )


def _rational_interval(value: sp.Expr) -> CertifiedDecimalIntervalV1:
    numerator = int(value.p)
    denominator = int(value.q)
    lower_units = numerator * _SCALE // denominator
    upper_units = -((-numerator * _SCALE) // denominator)
    # `scaleb` — операция контекста Decimal и округляет до 28 значащих цифр.
    # Для φ ∈ (1, 2) это на цифру больше, чем помещается, поэтому граница может
    # уехать внутрь. Поведение унаследованное и намеренно сохранено дословно:
    # от него зависят дайджесты, а расхождение ловит сверка выше.
    return _interval(
        Decimal(lower_units).scaleb(-DECIMALS),
        Decimal(upper_units).scaleb(-DECIMALS),
    )


def _scaled(units: int) -> Decimal:
    # Строковый конструктор `Decimal` не округляется по контексту, в отличие от
    # `scaleb`. Оболочке это обязательно: округление внутрь на последнем знаке
    # превратило бы сертификат в оценку.
    return Decimal(f"{units}E-{DECIMALS}")


def _enclose_fraction_of_pi(
    sine: sp.Expr, cosine: sp.Expr
) -> tuple[Fraction, Fraction] | None:
    saved = iv.prec
    iv.prec = _ENCLOSURE_PRECISION
    try:
        enclosure = (
            iv.atan2(
                abs(interval_enclosure(sine)),
                interval_enclosure(cosine),
            )
            / iv.pi
        )
    except (
        IntervalEnclosureUnsupported,
        ArithmeticError,
        ValueError,
        TypeError,
    ):
        return None
    finally:
        iv.prec = saved
    # `_mpi_` — пара границ в двоичном представлении mpmath. Публичные `.a`/`.b`
    # отдают вырожденный интервал, а не число, и точного рационального
    # представления не дают; через `float()` сертификат потерял бы разряды.
    lower, upper = enclosure._mpi_
    return _endpoint(lower), _endpoint(upper)


def _endpoint(value) -> Fraction:
    """Точное рациональное значение границы mpmath, без потери разряда."""

    return Fraction(*libmp.to_rational(value))
