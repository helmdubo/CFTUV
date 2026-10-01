"""Подъём точки карты в локальные 3D-координаты: ТОЧНО, одно округление.

Плоскость домена задана аффинным репером метрики: `origin + a*x + b*y`, где
`(origin, a, b)` — точные дроби (`exact_origin`, `exact_basis_a`,
`exact_basis_b` дескриптора). Точка покрытия — пара `SqrtSumV1` в единицах
решётки, поэтому подъём — рациональная линейная комбинация, а значит ТОЧНАЯ
величина `SqrtSumV1` по каждой оси. Во float она переводится ОДИН раз, на
выходе, серединой строгой оболочки (`sqrt_sum_binary64`): ни промежуточных
округлений, ни порогов.

NEAR_PLANAR. Реперы такого домена построены по вершинам, СПРОЕЦИРОВАННЫМ на
точную плоскость, поэтому подъём лежит на сертифицированной плоскости, а не на
исходном меше: расстояние между ними — невязка сертификата, и материализатор
называет её диагностикой (`NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE`), а не прячет.
Смещение декали над поверхностью (z-fighting) — политика ХОСТА, не ядра.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from ..contracts.metric import AffineChartOrientationV1, DevelopableUnfoldCertificateV1
from ..exact_sqrt_sum import SqrtSumV1
from ..numeric import LocalPoint3V1, LocalVector3V1
from ..planar_metric import fraction_from_exact

#: Разрядность целочисленной оболочки при переводе `SqrtSumV1` в число. Читать
#: величину по частям правило проекта запрещает; оболочка — объявленный способ,
#: и её середина отличается от истинного значения не больше чем на половину
#: ширины, то есть на 2^-64 в единицах величины.
ENCLOSURE_BITS = 64


def sqrt_sum_binary64(value: SqrtSumV1, *, bits: int = ENCLOSURE_BITS) -> float:
    """Число из `SqrtSumV1` — серединой строгой оболочки, а не по членам.

    Это ТОТ ЖЕ перевод, которым пользуется отладочный хост (`sqrt_sum_float`):
    у картинки и у меша одно округление, а не два разных.
    """

    low, high = value.enclosure(bits)
    return float((low + high) / 2)


@dataclass(frozen=True, slots=True)
class PlaneLiftV1:
    """Точный подъём карты домена на плоскость: репер и масштаб решётки."""

    origin: tuple[Fraction, Fraction, Fraction]
    basis_a: tuple[Fraction, Fraction, Fraction]
    basis_b: tuple[Fraction, Fraction, Fraction]
    scale: int

    def lift_exact(self, point) -> tuple[SqrtSumV1, SqrtSumV1, SqrtSumV1]:
        """`origin + a*(X/scale) + b*(Y/scale)` по осям, без единого округления."""

        scale = Fraction(self.scale)
        return tuple(
            point[0].scaled(self.basis_a[axis] / scale)
            + point[1].scaled(self.basis_b[axis] / scale)
            + SqrtSumV1.rational(self.origin[axis])
            for axis in range(3)
        )

    def lift(self, point) -> LocalPoint3V1:
        x, y, z = self.lift_exact(point)
        return LocalPoint3V1(
            sqrt_sum_binary64(x), sqrt_sum_binary64(y), sqrt_sum_binary64(z)
        )

    def counters(self) -> tuple[tuple[str, int], ...]:
        """Подъём на плоскость точки не ищет и бюджета не тратит: счётчиков нет."""

        return ()

    def note(self) -> str:
        """Пояснение укладки для диагностики: у плоскости его нет."""

        return ""

    def gap_note(self) -> str:
        """Зазор смещения по нормалям вершин: у плоскости одна нормаль на домен, записи нет."""

        return ""


def _triple(value) -> tuple[Fraction, Fraction, Fraction]:
    return (
        fraction_from_exact(value.x),
        fraction_from_exact(value.y),
        fraction_from_exact(value.z),
    )


def _refuse_unfolded(descriptor) -> None:
    """Репер развёртки — репер КАРТЫ: плоскости источника у него нет, класть не на что."""

    if type(descriptor.planarity_certificate) is DevelopableUnfoldCertificateV1:
        raise ValueError(
            "an unfolded chart has no source plane: the domain lifts onto the "
            "source triangles, and its offset normal is per vertex"
        )


def plane_lift_of(descriptor, scale: int) -> PlaneLiftV1:
    """Подъём по дескриптору `RationalAffinePlanarMetricV2` и масштабу решётки."""

    _refuse_unfolded(descriptor)
    return PlaneLiftV1(
        origin=_triple(descriptor.exact_origin),
        basis_a=_triple(descriptor.exact_basis_a),
        basis_b=_triple(descriptor.exact_basis_b),
        scale=int(scale),
    )


def plane_normal_binary64(descriptor) -> LocalVector3V1:
    """Единичная нормаль плоскости домена — той стороны, куда смотрит сетка батча.

    Публичный помощник для хоста: смещение декали над поверхностью (z-fighting)
    — политика хоста, а направление смещения он не должен выводить из
    приватных функций отладочной сцены. Нормаль — векторное произведение
    `a x b` ТОЧНЫХ реперных векторов (рациональных), а знак берётся из
    ориентации карты: триангуляция идёт против часовой стрелки в координатах
    карты, и когда карта совпадает с владельцем по часовой
    (`COORDINATE_CW_MATCHES_OWNER_PATCH`), материализатор разворачивает
    треугольники, поэтому лицевая сторона — `-(a x b)`. Одно округление на
    выходе: компоненты и длина переводятся во float по одному разу.
    """

    _refuse_unfolded(descriptor)
    a = _triple(descriptor.exact_basis_a)
    b = _triple(descriptor.exact_basis_b)
    cross = (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
    mirrored = (
        descriptor.chart_orientation
        is AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )
    if mirrored:
        cross = tuple(-item for item in cross)
    squared = sum(item * item for item in cross)
    if not squared:
        raise ValueError("the plane basis of the domain is degenerate")
    length = math.sqrt(float(squared))
    return LocalVector3V1(*(float(item) / length for item in cross))
