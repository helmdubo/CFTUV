"""Фильтр знака в binary64: либо доказывает знак, либо уступает точному пути.

ЗАЧЕМ. Предикат `faces.orientation` спрашивает знак `(b - a) x (c - a)` у точек из
сумм корней. Точный ответ строит произведение двух сумм по четыре-восемь корней
(до шестнадцати членов, `gcd` на каждую пару) и только потом читает знак, а читает
он его почти всегда целочисленной оболочкой. Замер материализатора (Blender 4.5,
`2` домен 0, DECISIONS «PERF MATERIALIZE-SPEED»): `orientation` — 8117
вызовов, 57% всего времени материализации. Знак с запасом (определитель далеко от
нуля) виден и в binary64.

КАК. Каждая координата — `sum c_i * sqrt(m_i)` — получает центр и ВЕРХНЮЮ ГРАНИЦУ
абсолютной ошибки: `(n + 6) * 2^-52 * sum |c_i * sqrt(m_i)|`. Честная оценка
накопленного округления — `(n + 3.8) * 2^-53 * sum |...|` (преобразования
`float(Fraction)`, `float(int)`, `sqrt`, произведение и последовательная сумма
по `n` членам, по `u = 2^-53` на действие), поэтому граница вдвое шире нужной.
Ошибки разностей, произведений и суммы определителя считаются по первому порядку
вместе со вторым (`e1 * e2`) и с тем же двойным запасом на собственное
округление каждого действия. Знак объявляется, только если `|det|` строго больше
границы ошибки (ещё с запасом `1 + 1e-9`). Всё, что не доказано, — нуль, касание,
коллинеарность, переполнение, радикант за пределами `float` — возвращает `None`, и
вызывающий идёт прежним точным путём. Порога в смысле допуска здесь нет: фильтр
не меняет ответ, а только решает, платить ли за него точной арифметикой.

ПРЯМАЯ. `line_estimate` — тот же фильтр для знака точки у прямой с ЦЕЛЫМИ концами (`dx * (y - y0) - dy * (x - x0)`): резка спрашивает его
на каждую пару «узел, ребро области» (`materialize.clip._slot`) и строит точное значение, только когда оно нужно.

ПАМЯТЬ. Центр и граница координаты считаются один раз на объект: таблица ключится
`id(объект)` и держит САМ объект, поэтому занятый ключ не может достаться другому
живому значению. Таблицу сбрасывает `reset_factorization_memory` на границе домена, а предел записей — целиком. Результат от таблицы не
зависит (она кэширует чистую функцию значения), поэтому порядок задач и воркеров на
ответ не влияет.
"""

from __future__ import annotations

from math import sqrt
from sys import float_info

#: Единица округления binary64 и вдвое более широкий запас, которым считаются все границы.
_UNIT = 2.0 ** -53
_SLACK = 2.0 ** -52
#: Абсолютная добавка на денормалы: недобор произведения даёт абсолютную ошибку не больше `2^-1075` на действие, а
#: наименьшее нормальное число binary64 на много порядков больше. Она входит в границу каждого произведения (иначе у
#: пары малых множителей граница исчезает вместе с самим произведением), а координата с коэффициентом меньше неё
#: фильтру не отдаётся вовсе (`_measure`). Не допуск: значение — свойство формата.
_FLOOR = float_info.min
#: Граница ошибки, которой заведомо не хватает значения: определитель должен её ПЕРЕБИТЬ.
_MARGIN = 1.0 + 1e-9
#: Предел таблицы центров: целая таблица сбрасывается, когда она переросла.
_TABLE_LIMIT = 1 << 17

_TABLE: dict[int, tuple[object, float | None, float]] = {}


def _measure(value) -> tuple[object, float | None, float]:
    terms = value.terms
    total = 0.0
    weight = 0.0
    try:
        for radicand, coefficient in terms:
            centre = float(coefficient)
            # Коэффициент меньше наименьшего нормального числа (и нуль — недобор): ошибка `float(Fraction)` у него
            # АБСОЛЮТНА (до 2^-1075), а не относительна, и умножение на `sqrt(m)` разгоняет её сколь угодно далеко
            # за `_FLOOR`. Такую координату binary64 не берёт: точный путь.
            if abs(centre) < _FLOOR:
                raise OverflowError
            term = centre * sqrt(radicand)
            total += term
            weight += abs(term)
    except OverflowError:
        entry = (value, None, 0.0)
    else:
        bound = (len(terms) + 6) * _SLACK * weight + _FLOOR
        # `inf - inf` и друзья: центр и граница обязаны быть конечны.
        entry = (value, total, bound) if bound - bound == 0.0 and total - total == 0.0 else (value, None, 0.0)
    if len(_TABLE) >= _TABLE_LIMIT:
        _TABLE.clear()
    _TABLE[id(value)] = entry
    return entry


def clear_table() -> None:
    """Сбрасывает таблицу центров (тесты и измерения; на ответ не влияет)."""

    _TABLE.clear()


def centre_and_bound(value) -> tuple[float, float] | None:
    """`(центр, граница)`: `|value - центр| <= граница`; `None` — binary64 величину не берёт."""

    entry = _TABLE.get(id(value))
    if entry is None:
        entry = _measure(value)
    return None if entry[1] is None else (entry[1], entry[2])


def orientation_sign(first, second, third) -> int | None:
    """Знак удвоенной ориентированной площади `(b - a) x (c - a)` либо `None` (не доказан)."""

    table = _TABLE
    get = table.get
    entries = []
    for point in (first, second, third):
        for coordinate in point:
            entry = get(id(coordinate))
            if entry is None:
                entry = _measure(coordinate)
            if entry[1] is None:
                return None
            entries.append(entry)
    (_, ax, eax), (_, ay, eay), (_, bx, ebx), (_, by, eby), (_, cx, ecx), (_, cy, ecy) = entries
    slack = _SLACK
    # (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    d1 = bx - ax
    e1 = ebx + eax + slack * abs(d1)
    d2 = cy - ay
    e2 = ecy + eay + slack * abs(d2)
    d3 = by - ay
    e3 = eby + eay + slack * abs(d3)
    d4 = cx - ax
    e4 = ecx + eax + slack * abs(d4)
    left = d1 * d2
    left_bound = abs(d1) * e2 + abs(d2) * e1 + e1 * e2 + slack * abs(left) + _FLOOR
    right = d3 * d4
    right_bound = abs(d3) * e4 + abs(d4) * e3 + e3 * e4 + slack * abs(right) + _FLOOR
    value = left - right
    bound = (left_bound + right_bound + slack * abs(value)) * _MARGIN
    if abs(value) > bound:
        return 1 if value > 0.0 else -1
    return None


def line_estimate(point, start_x: float, start_y: float, step_x: float, step_y: float) -> tuple[float, float] | None:
    """`(значение, граница)` ориентации точки у прямой: `step_x * (y - start_y) - step_y * (x - start_x)`, либо `None`.

    `|точное - значение| <= граница`: тот же порядок оценки, что у `orientation_sign` (центр и граница каждой координаты
    из таблицы, ошибки разности, произведений и суммы по первому порядку с запасом вдвое, `_MARGIN`). Концы прямой и
    шаг — float, ТОЧНО представляющие целые (вызывающий не отдаёт сюда ничего другого): ошибки у них нет. Знак доказан,
    если `|значение| > граница`; это же значение решает «дальше допуска от прямой» (`|значение| > граница + допуск`).
    `None` — координату binary64 не берёт: вызывающий идёт точным путём.
    """

    get = _TABLE.get
    entry_x = get(id(point[0]))
    if entry_x is None:
        entry_x = _measure(point[0])
    centre_x = entry_x[1]
    if centre_x is None:
        return None
    entry_y = get(id(point[1]))
    if entry_y is None:
        entry_y = _measure(point[1])
    centre_y = entry_y[1]
    if centre_y is None:
        return None
    slack = _SLACK
    along_y = centre_y - start_y
    error_y = entry_y[2] + slack * abs(along_y)
    along_x = centre_x - start_x
    error_x = entry_x[2] + slack * abs(along_x)
    first = step_x * along_y
    second = step_y * along_x
    value = first - second
    bound = (
        abs(step_x) * error_y
        + abs(step_y) * error_x
        + slack * (abs(first) + abs(second) + abs(value))
        + _FLOOR
    ) * _MARGIN
    return value, bound


def polygon_sign(points) -> int | None:
    """Знак удвоенной ориентированной площади многоугольника (веер от первой точки) либо `None`."""

    count = len(points)
    if count < 3:
        return None
    flat = []
    for point in points:
        for coordinate in point:
            entry = _TABLE.get(id(coordinate))
            if entry is None:
                entry = _measure(coordinate)
            if entry[1] is None:
                return None
            flat.append(entry)
    slack = _SLACK
    _, px, epx = flat[0]
    _, py, epy = flat[1]
    ux = flat[2][1] - px
    eux = flat[2][2] + epx + slack * abs(ux)
    uy = flat[3][1] - py
    euy = flat[3][2] + epy + slack * abs(uy)
    total = 0.0
    total_bound = 0.0
    for index in range(2, count):
        vx = flat[2 * index][1] - px
        evx = flat[2 * index][2] + epx + slack * abs(vx)
        vy = flat[2 * index + 1][1] - py
        evy = flat[2 * index + 1][2] + epy + slack * abs(vy)
        left = ux * vy
        left_bound = abs(ux) * evy + abs(vy) * eux + eux * evy + slack * abs(left) + _FLOOR
        right = uy * vx
        right_bound = abs(uy) * evx + abs(vx) * euy + euy * evx + slack * abs(right) + _FLOOR
        area = left - right
        total += area
        total_bound += left_bound + right_bound + slack * abs(area) + slack * abs(total)
        ux, eux, uy, euy = vx, evx, vy, evy
    bound = total_bound * _MARGIN
    if abs(total) > bound:
        return 1 if total > 0.0 else -1
    return None
