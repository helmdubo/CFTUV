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

ЧТО СЧИТАЕТСЯ ОДИН РАЗ НА ИСТОЧНИК, А НЕ НА ПАРУ (`SourceContactFrame`). Длина опорного отрезка и ковекторы
`G·t`, `G·n` — функции только источника, а пар у источника сотни: длина (вычитание точек через `sympy`, скалярный
квадрат, корень) и два применения грамма стоили около трети цены пары. Те же операции над теми же значениями,
выполненные один раз, дают те же значения: ответ побитово прежний, меняется цена.

ПРЕДФИЛЬТР «КОНТАКТОВ НЕТ» (`SourceContactFrame.excludes`). Контакт пары — точка отрезка границы с
`0 <= станция <= длина` и `alpha >= 0`; станция и alpha линейны по параметру отрезка, поэтому, если ОБА конца
отрезка лежат строго по одну сторону одной из трёх прямых (`станция = 0`, `станция = длина`, `alpha = 0`) с
недопустимой стороны, ни одна точка отрезка не годится и результат пуст. Фильтр считает станцию и alpha обоих
концов в binary64 с границей ошибки (тот же порядок оценки, что `float_filter.orientation_sign`: центр и граница
каждой координаты из таблицы `float_filter`, ошибки разностей и произведений по первому порядку с запасом вдвое)
и объявляет «контактов нет», только если сторона доказана строго. Нуль, касание, число вне binary64, значение вне
поля — не доказано: пара идёт точным путём без изменений. Фильтр может лишь ДОКАЗАТЬ пустоту, поэтому не меняет
ответ; он не вводит допуска.
"""

from __future__ import annotations

from .._cpython311 import sorted_as_cpython311
from ..float_filter import _FLOOR, _MARGIN, _SLACK, centre_and_bound
from . import symbolic_backend as _backend
from .native_exact import (
    NativeExactError,
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


class SourceContactFrame:
    """Данные контактов, зависящие только от опорного отрезка источника: один раз на источник, лениво.

    Точная часть (`length`, `tangent_covector`, `normal_covector`) нужна `contact_candidates_native` на каждой паре,
    float-часть (центры и границы ошибки начала источника, ковекторов и длины) — только предфильтру. Отказ родной
    арифметики на источнике (`refusal`) повторяется на каждой паре как тот же класс исхода, что дал бы пересчёт.
    """

    __slots__ = ("_context", "_source", "_built", "refusal", "length", "tangent_covector", "normal_covector", "_floats")

    def __init__(self, context, source) -> None:
        self._context = context
        self._source = source
        self._built = False
        self.refusal: NativeExactError | None = None
        self.length: RadicalSumV1 | None = None
        self.tangent_covector: tuple[RadicalSumV1, RadicalSumV1] | None = None
        self.normal_covector: tuple[RadicalSumV1, RadicalSumV1] | None = None
        self._floats = None

    def ready(self) -> "SourceContactFrame":
        """Строит точную часть при первом обращении; нулевой вектор источника (`ValueError`) всплывает как раньше."""

        if self._built:
            return self
        metric = self._context.metric
        source = self._source
        try:
            self.length = metric.length_g_native(point_sub(source.end, source.start))
            self.tangent_covector = metric.covector_g_native(source.tangent)
            self.normal_covector = metric.covector_g_native(source.owner_normal)
        except NativeExactError as refusal:
            self.refusal = refusal
        self._built = True
        return self

    def _float_parts(self):
        """Семь пар `(центр, граница)`: начало (x, y), ковектор касательной (x, y), ковектор нормали (x, y), длина; `False` — не берётся."""

        floats = self._floats
        if floats is None:
            floats = self._floats = self._measure()
        return floats

    def _measure(self):
        try:
            start = self._source.start.natives()
        except NativeExactError:
            return False
        entries = []
        for value in (*start, *self.tangent_covector, *self.normal_covector, self.length):
            entry = centre_and_bound(value)
            if entry is None:
                return False
            entries.append(entry)
        return tuple(entries)

    def excludes(self, boundary) -> bool:
        """`True` ТОЛЬКО когда доказано: контактов нет. `False` — не доказано (пара идёт точным путём)."""

        if self.ready().refusal is not None:
            return False
        floats = self._float_parts()
        if floats is False:
            return False
        segment = boundary.segment
        try:
            first = segment.start.natives()
            second = segment.end.natives()
        except NativeExactError:
            return False
        near = _station_and_alpha(floats, first)
        if near is None:
            return False
        far = _station_and_alpha(floats, second)
        if far is None:
            return False
        (station0, station_error0, alpha0, alpha_error0) = near
        (station1, station_error1, alpha1, alpha_error1) = far
        if alpha0 < -alpha_error0 and alpha1 < -alpha_error1:
            return True
        if station0 < -station_error0 and station1 < -station_error1:
            return True
        length, length_error = floats[6]
        beyond0 = station0 - length
        beyond1 = station1 - length
        return (
            beyond0 > (station_error0 + length_error + _SLACK * abs(beyond0)) * _MARGIN
            and beyond1 > (station_error1 + length_error + _SLACK * abs(beyond1)) * _MARGIN
        )


def _station_and_alpha(floats, point):
    """`(станция, граница, alpha, граница)` точки в binary64 или `None`: `|точное - значение| <= граница`.

    Станция и alpha точки `P` — `(P - S) · G t` и `(P - S) · G n`: тот же порядок оценки, что `float_filter.orientation_sign`
    (граница координаты из таблицы, ошибка разности и произведений по первому порядку с запасом вдвое, `_MARGIN`).
    """

    (sx, sx_error), (sy, sy_error), (gx, gx_error), (gy, gy_error), (hx, hx_error), (hy, hy_error), _ = floats
    entry_x = centre_and_bound(point[0])
    entry_y = centre_and_bound(point[1])
    if entry_x is None or entry_y is None:
        return None
    dx = entry_x[0] - sx
    dx_error = entry_x[1] + sx_error + _SLACK * abs(dx)
    dy = entry_y[0] - sy
    dy_error = entry_y[1] + sy_error + _SLACK * abs(dy)
    left = dx * gx
    right = dy * gy
    station = left + right
    station_error = (
        abs(dx) * gx_error + abs(gx) * dx_error + dx_error * gx_error + _SLACK * abs(left) + _FLOOR
        + abs(dy) * gy_error + abs(gy) * dy_error + dy_error * gy_error + _SLACK * abs(right) + _FLOOR
        + _SLACK * abs(station)
    ) * _MARGIN
    left = dx * hx
    right = dy * hy
    alpha = left + right
    alpha_error = (
        abs(dx) * hx_error + abs(hx) * dx_error + dx_error * hx_error + _SLACK * abs(left) + _FLOOR
        + abs(dy) * hy_error + abs(hy) * dy_error + dy_error * hy_error + _SLACK * abs(right) + _FLOOR
        + _SLACK * abs(alpha)
    ) * _MARGIN
    return station, station_error, alpha, alpha_error


def contact_candidates_native(context, source, boundary, frame: SourceContactFrame | None = None) -> tuple:
    """`((alpha, station, point), ...)` по возрастанию alpha; alpha и station — `RadicalSumV1`.

    `frame` — данные источника, посчитанные один раз на источник (без него считаются здесь, на этой паре).
    """

    metric = context.metric
    if frame is None:
        frame = SourceContactFrame(context, source)
    if frame.ready().refusal is not None:
        raise type(frame.refusal)(frame.refusal.detail)
    length = frame.length
    barrier = boundary.segment
    barrier_direction = point_sub(barrier.end, barrier.start)
    start_offset = point_sub(barrier.start, source.start)
    s0 = metric.dot_covector_native(start_offset, frame.tangent_covector)
    ds = metric.dot_covector_native(barrier_direction, frame.tangent_covector)
    a0 = metric.dot_covector_native(start_offset, frame.normal_covector)
    da = metric.dot_covector_native(barrier_direction, frame.normal_covector)
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
    result[:] = sorted_as_cpython311(result, lambda left, right: (left[0] - right[0]).signum())
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
            for a, b in zip(sorted_as_cpython311(distinct_old, _by_alpha), new)
        )
    if not ok:
        _backend.disagreement(
            site,
            f"{len(distinct_old)} distinct legacy contacts vs {len(new)} native",
        )


def _by_alpha(left, right) -> int:
    return (left[0] - right[0]).signum()
