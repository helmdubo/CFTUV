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

ТОЖДЕСТВО АФФИННОСТИ. `affine_map_violated` — тот же фильтр для точного тождества `uv_vertex_on_affine_map` (вершина лежит на аффинной
карте значений, решённой по трём вершинам основы): доказывает только НАРУШЕНИЕ (невязка строго больше границы ошибки), а равенство
и всё неопределённое уступает точному пути.

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


def affine_map_violated(points, values, base, index) -> bool:
    """Доказано ли в binary64, что вершина `index` НЕ лежит на аффинной карте значений, решённой по основе `base` (три ключа).

    Невязка тождества `det (f(q) - f0) - [(q' x v) (f1 - f0) + (u x q') (f2 - f0)]` (`u`, `v` — стороны основы, `q'` — вершина от
    начала основы; `tessellate.uv_vertex_on_affine_map`) считается с границей ошибки тем же способом, что `orientation_sign`.
    `True` — по какой-то из двух компонент `|невязка|` строго больше границы: нарушение доказано, точный путь вывод
    подтвердит. `False` — не доказано (нуль, касание, число вне binary64): вызывающий идёт точным путём. Ответ не меняется,
    меняется цена: вершины, чьё нарушение велико, не платят за произведение сумм корней.
    """

    slack = _SLACK
    found = []
    for key in (base[0], base[1], base[2], index):
        for coordinate in points[key]:
            entry = centre_and_bound(coordinate)
            if entry is None:
                return False
            found.append(entry)
        for value in values[key]:
            entry = centre_and_bound(value)
            if entry is None:
                return False
            found.append(entry)

    def sub(first, second):
        value = first[0] - second[0]
        return value, first[1] + second[1] + slack * abs(value)

    def add(first, second):
        value = first[0] + second[0]
        return value, first[1] + second[1] + slack * abs(value)

    def mul(first, second):
        value = first[0] * second[0]
        return value, abs(first[0]) * second[1] + abs(second[0]) * first[1] + first[1] * second[1] + slack * abs(value) + _FLOOR

    # по четыре записи на вершину: x, y, f0, f1 (`values` — пара компонент, `points` — пара координат)
    x0, y0, a0, b0 = found[0:4]
    x1, y1, a1, b1 = found[4:8]
    x2, y2, a2, b2 = found[8:12]
    xq, yq, aq, bq = found[12:16]
    ux, uy = sub(x1, x0), sub(y1, y0)
    vx, vy = sub(x2, x0), sub(y2, y0)
    qx, qy = sub(xq, x0), sub(yq, y0)
    det = sub(mul(ux, vy), mul(uy, vx))
    first_weight = sub(mul(qx, vy), mul(qy, vx))
    second_weight = sub(mul(ux, qy), mul(uy, qx))
    for f0, f1, f2, fq in ((a0, a1, a2, aq), (b0, b1, b2, bq)):
        left = mul(det, sub(fq, f0))
        right = add(mul(first_weight, sub(f1, f0)), mul(second_weight, sub(f2, f0)))
        residual = sub(left, right)
        if abs(residual[0]) > residual[1] * _MARGIN:
            return True
    return False
