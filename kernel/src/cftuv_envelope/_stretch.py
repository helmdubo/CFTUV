"""Сертификат растяжения развёртки: точный суд по Грамам, без корней и допусков вычисления.

Модуль внутренний, как `_width_distortion`: допуск ему не принадлежит (его владеет
`contracts.metric.DEVELOPABLE_STRETCH_BUDGET`), он ничего не округляет и возвращает
запись. Судит ли запись, решает вызывающий.

МАТЕМАТИКА. Треугольник источника задан рёбрами `e1 = p1 - p0`, `e2 = p2 - p0`
(рациональный Грам `G_s`); его образ в карте — рёбрами `f1`, `f2` (Грам `G_c`).
Линейное отображение `e_i -> f_i` растягивает квадрат длины в `x^T G_c x / x^T G_s x`
раз, а крайние значения этого отношения — корни `λ` уравнения

    q(λ) = det(G_c - λ G_s) = a λ² - B λ + C = 0,
    a = det G_s,  C = det G_c,  B = c00 s11 + c11 s00 - 2 c01 s01.

Оба корня лежат в `[l, u]` (`l = 1/(1+b)²`, `u = (1+b)²`) тогда и только тогда,
когда `q(l) >= 0`, `q(u) >= 0` и `l <= B/(2a) <= u`: ведущий коэффициент положителен,
значит парабола неотрицательна в концах и её вершина внутри. Это три знака
рациональных чисел — предикат ТОЧЕН. Именно он судит; число `worst_band_squared_upper`
в записи — сертифицированная верхняя граница `max(λ_max, 1/λ_min)`, читаемая, но не
судящая: `λ_max` ограничен целочисленным корнем, а `1/λ_min = λ_max · a / C`
(произведение корней `C/a`), поэтому вычитания близких чисел нет.

ОРИЕНТАЦИЯ. Грам знак площади не видит: отражённый треугольник имеет тот же `G_c`.
Знак площади карты считается отдельно, точно, и треугольник с обратным обходом
считается и называется (`chart_flipped_triangle_count`).
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import isqrt

from .contracts.metric import (
    DEVELOPABLE_STRETCH_BUDGET,
    DevelopableStretchCertificateV1,
    DevelopableStretchLawV1,
    ExactRationalV1,
)
from .outcomes import NamedOutcome

#: Число двоичных разрядов, с которыми берётся целочисленный корень границы.
_ROOT_BITS = 64


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def source_gram(corners) -> tuple[Fraction, Fraction, Fraction]:
    """Грам пары рёбер `(p0->p1, p0->p2)` ТОЧНОЙ 3D-тройки: `(g00, g01, g11)`."""

    first = tuple(a - b for a, b in zip(corners[1], corners[0], strict=True))
    second = tuple(a - b for a, b in zip(corners[2], corners[0], strict=True))
    dot = lambda left, right: sum(  # noqa: E731
        (a * b for a, b in zip(left, right, strict=True)), Fraction(0)
    )
    return dot(first, first), dot(first, second), dot(second, second)


def chart_gram(points) -> tuple[Fraction, Fraction, Fraction]:
    """Грам пары рёбер ТОЧНОЙ 2D-тройки (метры)."""

    first = (points[1][0] - points[0][0], points[1][1] - points[0][1])
    second = (points[2][0] - points[0][0], points[2][1] - points[0][1])
    return (
        first[0] * first[0] + first[1] * first[1],
        first[0] * second[0] + first[1] * second[1],
        second[0] * second[0] + second[1] * second[1],
    )


def chart_twice_area(points) -> Fraction:
    """Ориентированная удвоенная площадь тройки карты: знак — обход."""

    return (points[1][0] - points[0][0]) * (points[2][1] - points[0][1]) - (
        points[1][1] - points[0][1]
    ) * (points[2][0] - points[0][0])


def _quadratic(source, chart):
    s00, s01, s11 = source
    c00, c01, c11 = chart
    leading = s00 * s11 - s01 * s01
    middle = c00 * s11 + c11 * s00 - 2 * c01 * s01
    constant = c00 * c11 - c01 * c01
    return leading, middle, constant


def band_bounds(budget: Fraction) -> tuple[Fraction, Fraction]:
    """`(l, u)` = `(1/(1+b)², (1+b)²)`: границы квадрата растяжения."""

    scale = 1 + Fraction(budget)
    return Fraction(1) / (scale * scale), scale * scale


def in_stretch_band(source, chart, budget: Fraction) -> bool:
    """ТОЧНОЕ условие приёма: оба квадрата сингулярных чисел в `[l, u]`.

    Три знака рациональных чисел, ни корня, ни допуска. Вырожденный в карте
    треугольник (`C = 0`) имеет нулевое сингулярное число и условию не удовлетворяет.
    """

    leading, middle, constant = _quadratic(source, chart)
    low, high = band_bounds(budget)
    at = lambda lam: leading * lam * lam - middle * lam + constant  # noqa: E731
    return (
        at(low) >= 0
        and at(high) >= 0
        and 2 * leading * low <= middle <= 2 * leading * high
    )


def _sqrt_upper(value: Fraction) -> Fraction:
    """Рациональная верхняя граница `√value`, точная целочисленным корнем."""

    if value <= 0:
        return Fraction(0)
    scale = 1 << _ROOT_BITS
    numerator = value.numerator * value.denominator * scale * scale
    return Fraction(isqrt(numerator) + 1, value.denominator * scale)


def band_squared_upper(source, chart) -> Fraction | None:
    """Сертифицированная верхняя граница `max(λ_max, 1/λ_min)`; `None` при `C = 0`."""

    leading, middle, constant = _quadratic(source, chart)
    if constant <= 0:
        return None
    discriminant = middle * middle - 4 * leading * constant
    largest = (middle + _sqrt_upper(discriminant)) / (2 * leading)
    return largest * max(Fraction(1), leading / constant)


@dataclass(frozen=True, slots=True)
class StretchFactsV1:
    """Итог измерения: запись и именованные треугольники для диагностики."""

    certificate: DevelopableStretchCertificateV1
    outside: tuple
    flipped: tuple
    degenerate: tuple


def measure_stretch(triangles, positions, chart, budget: Fraction) -> StretchFactsV1:
    """Растяжение каждого треугольника источника в карту `chart` (точные метры).

    `triangles` — треугольники владельца, `positions` — точные привязанные 3D-позиции
    их вершин, `chart` — позиции карты (дроби). Порядок — по имени треугольника.
    """

    outside, flipped, degenerate = [], [], []
    worst = None
    measured = 0
    for item in sorted(triangles, key=lambda entry: entry.triangle_id.value):
        measured += 1
        points = tuple(chart[vertex] for vertex in item.vertex_ids)
        source = source_gram(tuple(positions[vertex] for vertex in item.vertex_ids))
        area = chart_twice_area(points)
        if area < 0:
            flipped.append(item.triangle_id)
        gram = chart_gram(points)
        if not in_stretch_band(source, gram, budget):
            outside.append(item.triangle_id)
        band = band_squared_upper(source, gram)
        if band is None:
            degenerate.append(item.triangle_id)
        elif worst is None or band > worst[0]:
            worst = (band, item.triangle_id)
    certificate = DevelopableStretchCertificateV1(
        law=DevelopableStretchLawV1.EXACT_GRAM_SINGULAR_VALUE_BAND_V1,
        stretch_budget=_rational(budget),
        triangles_measured=measured,
        triangles_outside_budget=len(outside),
        first_outside_triangle_id=outside[0] if outside else None,
        worst_triangle_id=None if worst is None else worst[1],
        worst_band_squared_upper=_rational(Fraction(1) if worst is None else worst[0]),
        chart_degenerate_triangle_count=len(degenerate),
        chart_flipped_triangle_count=len(flipped),
        first_flipped_triangle_id=flipped[0] if flipped else None,
    )
    return StretchFactsV1(
        certificate=certificate,
        outside=tuple(outside),
        flipped=tuple(flipped),
        degenerate=tuple(degenerate),
    )


def stretch_violations(certificate) -> tuple[NamedOutcome, ...]:
    """Именованные отказы сертификата. Порядок: бюджет растяжения, переворот.

    Растяжение идёт первым: перевёрнутый треугольник почти всегда ещё и сильно
    растянут (веер не замкнулся), и честное имя причины — бюджет; переворот без
    нарушения бюджета называется своим именем.
    """

    result = []
    if certificate.triangles_outside_budget:
        result.append(NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED)
    if certificate.chart_flipped_triangle_count:
        result.append(NamedOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED)
    return tuple(result)


def _shown(value: ExactRationalV1) -> float:
    return value.numerator / value.denominator


def stretch_refusal_text(certificate, *, worst_vertex=None) -> str:
    """Отказ несёт числа, из которых сложилось решение (binary64 — для чтения)."""

    low, high = band_bounds(
        Fraction(certificate.stretch_budget.numerator, certificate.stretch_budget.denominator)
    )
    first = "none" if certificate.first_outside_triangle_id is None else (
        certificate.first_outside_triangle_id.value
    )
    worst = "none" if certificate.worst_triangle_id is None else (
        certificate.worst_triangle_id.value
    )
    flipped = "none" if certificate.first_flipped_triangle_id is None else (
        certificate.first_flipped_triangle_id.value
    )
    vertex = "none" if worst_vertex is None else worst_vertex
    return (
        "developable stretch: "
        f"worst_band_squared<={_shown(certificate.worst_band_squared_upper):.9e} "
        f"allowed=[{float(low):.9e}, {float(high):.9e}] "
        f"(stretch_budget={_shown(certificate.stretch_budget):.6e}, "
        f"law={certificate.law.value}, "
        f"triangles_measured={certificate.triangles_measured}, "
        f"worst_triangle={worst}); "
        f"outside_budget={certificate.triangles_outside_budget} (first={first}); "
        f"chart_degenerate={certificate.chart_degenerate_triangle_count}; "
        f"chart_flipped={certificate.chart_flipped_triangle_count} (first={flipped}); "
        f"worst_vertex={vertex}"
    )


__all__ = (
    "DEVELOPABLE_STRETCH_BUDGET",
    "StretchFactsV1",
    "band_bounds",
    "band_squared_upper",
    "chart_gram",
    "chart_twice_area",
    "in_stretch_band",
    "measure_stretch",
    "source_gram",
    "stretch_refusal_text",
    "stretch_violations",
)
