"""Родной двойник `boundary._contact_candidates`: контакты опорного отрезка с границей без sympy.

Функция та же — геометрия пары (опорный отрезок источника, отрезок границы домена) в метрике
контекста, alpha запроса в неё не входит. Здесь она считается в `native_exact`: четыре скалярных
произведения, длина источника и параметры пересечения — суммы корней по квадратным классам, а не
выражения `sympy`, которые `exact_normalize` приводил `factor`'ом на каждом шаге.

Что остаётся прежним ПО ПОСТРОЕНИЮ: набор кандидатов параметра (0, 1 и три отношения), три
фильтра допустимости (параметр в `[0, 1]`, станция в `[0, длина]`, alpha не отрицательна) и
порядок по alpha. Что может отличаться и названо: ПОРЯДОК равных по alpha контактов (прежний
код брал его из итерации множества `sympy`, зависящей от хэша имени типа и потому не воспроизводимого
между процессами; здесь порядок — вставки), и дубликаты (прежний код держал структурно разные, но
равные по значению параметры раздельно; здесь равные по значению сливаются). Теневая сверка
сравнивает множества различных контактов, а не последовательности.

Уступка sympy — исключением `NativeExactError` (вне поля, знак не решён): вызывающий считает
прежним путём и записывает уступку в `symbolic_backend.BACKEND_COUNTS`.
"""

from __future__ import annotations

from functools import cmp_to_key

from . import symbolic_backend as _backend
from .native_exact import (
    OutsideNativeField,
    RadicalSumV1,
    _rational_text,
    from_sympy,
    to_sympy,
)
from .planar_types import (
    ExactPlanarPoint,
    ExactScalar,
    point_add,
    point_sub,
    vector_scale,
)

_ZERO = RadicalSumV1.zero()
_ONE = RadicalSumV1.rational(1)


def _distinct(values: list[RadicalSumV1]) -> list[RadicalSumV1]:
    unique: list[RadicalSumV1] = []
    for value in values:
        if not any(not (value - kept).terms for kept in unique):
            unique.append(value)
    return unique


def _point_at(barrier, direction, parameter: RadicalSumV1) -> ExactPlanarPoint:
    """`barrier.start + direction * parameter`; рациональный случай — прямо в строки `Rational`."""

    rational = parameter.as_rational()
    if rational is not None:
        start = [item.as_rational() for item in barrier.start.natives()]
        step = [item.as_rational() for item in direction.natives()]
        if None not in start and None not in step:
            return ExactPlanarPoint(
                ExactScalar(_rational_text(start[0] + step[0] * rational)),
                ExactScalar(_rational_text(start[1] + step[1] * rational)),
            )
    return point_add(barrier.start, vector_scale(direction, to_sympy(parameter)))


def contact_candidates_native(context, source, boundary) -> tuple:
    """`((alpha, station, point), ...)` по возрастанию alpha; alpha и station — `RadicalSumV1`."""

    metric = context.metric
    barrier = boundary.segment
    barrier_direction = point_sub(barrier.end, barrier.start)
    length = metric.length_g_native(point_sub(source.end, source.start))
    start_offset = point_sub(barrier.start, source.start)
    s0 = metric.dot_g_native(start_offset, source.tangent)
    ds = metric.dot_g_native(barrier_direction, source.tangent)
    a0 = metric.dot_g_native(start_offset, source.owner_normal)
    da = metric.dot_g_native(barrier_direction, source.owner_normal)
    candidates = [_ZERO, _ONE]
    if ds.signum() != 0:
        candidates.append((-s0) / ds)
        candidates.append((length - s0) / ds)
    if da.signum() != 0:
        candidates.append((-a0) / da)
    result = []
    for parameter in _distinct(candidates):
        if parameter.signum() < 0 or (parameter - _ONE).signum() > 0:
            continue
        station = s0 + parameter * ds
        alpha = a0 + parameter * da
        if station.signum() < 0 or (station - length).signum() > 0:
            continue
        if alpha.signum() < 0:
            continue
        result.append((alpha, station, _point_at(barrier, barrier_direction, parameter)))
    result.sort(key=cmp_to_key(lambda left, right: (left[0] - right[0]).signum()))
    return tuple(result)


def _as_native(value) -> RadicalSumV1:
    return value if type(value) is RadicalSumV1 else from_sympy(value)


def _contact_values(contact) -> tuple[RadicalSumV1, RadicalSumV1, RadicalSumV1, RadicalSumV1]:
    alpha, station, point = contact
    x, y = point.natives()
    return _as_native(alpha), _as_native(station), x, y


def _same(left, right) -> bool:
    return all(not (a - b).terms for a, b in zip(left, right))


def compare_contacts(legacy: tuple, native: tuple) -> None:
    """Теневая сверка: множества различных контактов совпадают по значению, порядок по alpha — тоже."""

    site = "contact_candidates"
    try:
        old = [_contact_values(item) for item in legacy]
    except OutsideNativeField:
        _backend.count(site, "shadow_outside_field")
        return
    new = [_contact_values(item) for item in native]
    _backend.count(site, "shadow_checked")
    distinct_old: list = []
    for item in old:
        if not any(_same(item, kept) for kept in distinct_old):
            distinct_old.append(item)
    if len(old) != len(distinct_old):
        _backend.count(site, "legacy_duplicates", len(old) - len(distinct_old))
    ok = len(distinct_old) == len(new) and all(
        any(_same(item, other) for other in new) for item in distinct_old
    )
    # Оба списка отсортированы по alpha: значения alpha по позициям различных контактов совпадают.
    if ok:
        ok = all(
            not (a[0] - b[0]).terms
            for a, b in zip(sorted(distinct_old, key=cmp_to_key(_by_alpha)), new)
        )
    if not ok:
        _backend.disagreement(
            site,
            f"{len(distinct_old)} distinct legacy contacts vs {len(new)} native",
        )


def _by_alpha(left, right) -> int:
    return (left[0] - right[0]).signum()
