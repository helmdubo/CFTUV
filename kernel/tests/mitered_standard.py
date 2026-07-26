"""Митрованный эталон покрытия для ПРЯМОУГОЛЬНОГО семейства. Считается вне кода покрытия.

Зачем он существует. До него ни одна проверка проекта не сравнивала покрытие с
множеством, посчитанным НЕЗАВИСИМО от кода покрытия. Проверка на
`interactions/policy_b.py:969-976` сравнивает результат кроя с `RawCoverage`, то
есть покрытие с самим собой до кроя, а `test_wavefront_coverage.py` проверяет
границы, объявленные самой `coverage.py`, на её же результате. Дыра `alpha^2` у
вогнутой вершины прожила до этого среза ровно поэтому: вопрос «а верно ли само
множество» никто не задавал.

**ВЫБРАННАЯ ТОЧКА СЕМЕЙСТВА НАЗВАНА ЯВНО, потому что перепутать её стоит дорого.**
У вопроса «какое множество покрыто к моменту alpha» ответов не два, а ТРИ, и
различаются они только профилем вогнутой вершины (`ell` 12x12 без правого
верхнего квадранта, площадь 108, alpha = 1):

| множество                      | вогнутая вершина      | площадь        |
|--------------------------------|-----------------------|---------------:|
| полосы (юбки попарного пути)   | не накрыта вовсе      | 43             |
| **МИТРОВАННЫЙ отступ**         | ОСТРЫЙ угол           | **44**         |
| круглый отступ (Минковский)    | дуга радиуса alpha    | 43 + pi/4      |

Эталон здесь — МИТРОВАННЫЙ, и это ВЫБОР точки семейства, а не единственная
возможность. Причина выбора: straight skeleton равноудалён от несущих ПРЯМЫХ, а
не от отрезков, поэтому вогнутая вершина уходит внутрь по биссектрисе и угол у
неё остаётся острым. Круглый отступ — это то, что обычно называют «эрозией», и
он даёт ТРЕТЬЕ число; приняв его за эталон, пришлось бы объявить дефектом верный
митрованный ответ 44. Параметризация по профилю вогнутого угла (`ROUND_k`,
`LINEAR_REFLEX_EQUAL_V1`) — отдельная работа; здесь митрованность НАЗВАНА, а не
зашита молча, и `alpha^2` — верхняя граница семейства, а не допуск.

**Почему семейство именно прямоугольное.** У фигуры, все рёбра которой осевые,
митрованный отступ снова осевой: каждое ребро уходит внутрь ровно на alpha по
своей нормали, а вершина садится в пересечение двух сдвинутых прямых. Значит
эталон живёт в `Fraction` целиком — ни одного радикала, ни одного float, ни
одного порога, ни одной строки из `wavefront/`. Это и делает его независимым.

**Как считается митрованный отступ, и почему это теорема, а не приём.** Отступ
есть множество точек, вокруг которых квадрат `[-alpha, alpha]^2` целиком лежит в
фигуре:

    I(alpha) = { p : p + [-alpha, alpha]^2 лежит в P }

У осевого ребра расстояние по нормали равно расстоянию до его несущей прямой,
поэтому опорная функция квадрата в направлении нормали равна ровно alpha: квадрат
срезает каждое ребро на alpha, а у вогнутой вершины две сдвинутых прямых
пересекаются в ОСТРОМ углу — том самом, который отличает митрованный ответ от
круглого. Дальше одна строка доказательства: если `p + K` лежит в `P`, то это
осевой прямоугольник внутри `P`, значит он лежит в каком-то МАКСИМАЛЬНОМ осевом
прямоугольнике `R` из `P`, значит `p` лежит в `R`, сжатом на alpha. И обратно.
Поэтому

    I(alpha) = объединение (R, сжатый на alpha) по вписанным осевым R

Для `ell` это буквально `[a,12-a]x[a,6-a] U [a,6-a]x[a,12-a]` — та самая замкнутая
форма, которой митрованный ответ был проверен независимо от обеих реализаций.
Максимальный вписанный прямоугольник упирается каждой стороной в ребро фигуры,
поэтому его координаты — координаты фигуры, и перебор по сжатой сетке полон.

Покрытие тогда `P \\ I(alpha)`.

**Полосы считаются здесь же и тем же способом**, чтобы разность двух путей была
числом, а не рассказом. Юбка ребра — перпендикулярная полоса глубиной alpha,
обрезанная КОНЦАМИ своего ребра; отсюда у вогнутой вершины остаётся квадрат
`alpha^2`, которого не накрывает ни одна юбка. Модель полос сверена с настоящим
`RawCoverage` конвейера в `test_wavefront_mitered_standard.py`: `ell` даёт 43,
`staircase` 42, `u_shape` 58, `double_notch` 72 — числа, посчитанные попарным
путём, а не здесь.

**Сетка здесь точная, а не численная.** Все три индикатора (`P`, отступ, полосы)
меняются только на прямых `x = x_i`, `x = x_i +- alpha` и таких же по `y`, поэтому
на каждой клетке сжатой сетки они ПОСТОЯННЫ, и сумма площадей клеток равна
площади множества точно. Это разбиение, а не выборка: узлов конечное число, и они
выведены из входа, а не подобраны. Своя проверка на это есть — сумма клеток
внутри `P` обязана совпасть с целой площадью многоугольника, иначе исход
`CELLS_DO_NOT_REPRODUCE_THE_POLYGON`.

Тихого исчезновения нет: не осевое ребро, отрицательная alpha и несошедшаяся
сетка — каждое даёт названный исход, а не пустой результат.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from cftuv_envelope.wavefront.polygon import PolygonV1, signed_double_area


# Осевой прямоугольник `(x_low, x_high, y_low, y_high)` в точных дробях.
Rect = tuple[Fraction, Fraction, Fraction, Fraction]


class StandardOutcome(str, Enum):
    """Чем кончился счёт эталона. Пустого ответа без имени здесь нет."""

    EXACT = "EXACT"
    # Ребро не осевое: у такой фигуры митрованный отступ уже не прямоугольный, и
    # в рациональных числах он не считается. `comb` корпуса — ровно этот случай
    # (зубья скошены), и он назван, а не обойдён.
    EDGE_IS_NOT_AXIS_PARALLEL = "EDGE_IS_NOT_AXIS_PARALLEL"
    ALPHA_IS_NEGATIVE = "ALPHA_IS_NEGATIVE"
    # Сумма клеток сжатой сетки не дала площадь многоугольника. Это отказ
    # эталона от собственного результата, а не повод его подправить.
    CELLS_DO_NOT_REPRODUCE_THE_POLYGON = "CELLS_DO_NOT_REPRODUCE_THE_POLYGON"
    # Отступ запрошен КОНТУРОМ, а не площадью, но при этой alpha он уже не
    # односвязен либо выродился: сдвиг рёбер даёт петлю другой площади.
    INSET_IS_NOT_ONE_LOOP = "INSET_IS_NOT_ONE_LOOP"
    # Контур отступа запрошен у фигуры с дырами. Дыра при росте alpha даёт свою
    # петлю, и «один контур» перестаёт быть верным описанием.
    INSET_LOOP_NEEDS_A_FIGURE_WITHOUT_HOLES = (
        "INSET_LOOP_NEEDS_A_FIGURE_WITHOUT_HOLES"
    )


@dataclass(frozen=True, slots=True)
class RectilinearStandardV1:
    """Эталон покрытия прямоугольной фигуры: три площади и вогнутые вершины.

    Все площади — `Fraction`, поэтому сравнимы точно. `mitered_covered` — та
    величина, с которой обязана совпасть очередь; `strip_covered` — та, которая
    обязана разойтись, и `strip_deficit` — разность, ради которой эталон и нужен.
    """

    outcome: StandardOutcome
    alpha: Fraction
    polygon_area: Fraction
    inset_area: Fraction
    mitered_covered: Fraction
    strip_covered: Fraction
    reflex_count: int
    reflex_square_union: Fraction
    detail: str = ""

    @property
    def strip_deficit(self) -> Fraction:
        """Насколько полосы накрыли МЕНЬШЕ митрованного эталона."""

        return self.mitered_covered - self.strip_covered

    @property
    def reflex_square_budget(self) -> Fraction:
        """Сумма `alpha^2` по вогнутым вершинам. Верхняя граница разности.

        Разложение разности на три числа существует затем, чтобы «меньше суммы»
        не превратилось в «непонятно почему меньше»:

            strip_deficit <= reflex_square_union <= reflex_square_budget

        Первое неравенство строгое, когда до квадрата вогнутой вершины дошла
        ЧУЖАЯ юбка. Второе — когда квадраты соседних вершин ПЕРЕСЕКАЮТСЯ либо
        квадрат вылез из фигуры. Оба случая законны и оба обязаны быть названы
        числом, а не оговоркой.
        """

        return self.reflex_count * self.alpha * self.alpha

    @property
    def deficit_is_the_full_budget(self) -> bool:
        return self.strip_deficit == self.reflex_square_budget


def axis_edges(
    polygon: PolygonV1,
) -> tuple[tuple[tuple[int, int], tuple[int, int]], ...] | None:
    """Рёбра всех петель, если КАЖДОЕ осевое. Иначе `None` — решает вызывающий."""

    edges = []
    for loop in polygon.loops:
        points = loop.points
        for index in range(len(points)):
            start = points[index]
            end = points[(index + 1) % len(points)]
            if start[0] != end[0] and start[1] != end[1]:
                return None
            edges.append((start, end))
    return tuple(edges)


def strip_of(
    start: tuple[int, int], end: tuple[int, int], alpha: Fraction
) -> Rect:
    """Юбка ребра: полоса глубиной alpha ВНУТРЬ, обрезанная концами ребра.

    Внутрь — это «влево от хода»: внешний контур CCW, дыры CW, и это нормировано
    `PolygonV1.build`. Поэтому направление ребра само говорит, куда полоса
    растёт, и параметра ориентации не нужно.
    """

    (x0, y0), (x1, y1) = start, end
    if y0 == y1:
        low, high = min(x0, x1), max(x0, x1)
        if x1 > x0:
            return (Fraction(low), Fraction(high), Fraction(y0), y0 + alpha)
        return (Fraction(low), Fraction(high), y0 - alpha, Fraction(y0))
    low, high = min(y0, y1), max(y0, y1)
    if y1 > y0:
        return (x0 - alpha, Fraction(x0), Fraction(low), Fraction(high))
    return (Fraction(x0), x0 + alpha, Fraction(low), Fraction(high))


def _inside(polygon: PolygonV1, x: Fraction, y: Fraction) -> bool:
    """Точка в фигуре. Луч вправо, чётность пересечений, точные дроби.

    Вырождений у луча быть не может по построению вызова: `y` берётся строго
    между двумя соседними РАЗЛИЧНЫМИ ординатами входа, поэтому ни одна вершина и
    ни одно горизонтальное ребро на луч не попадают.
    """

    crossings = 0
    for loop in polygon.loops:
        points = loop.points
        for index in range(len(points)):
            (x0, y0) = points[index]
            (x1, y1) = points[(index + 1) % len(points)]
            if y0 == y1:
                continue
            if min(y0, y1) < y < max(y0, y1) and x0 > x:
                crossings += 1
    return crossings % 2 == 1


def _in_any(rects: tuple[Rect, ...], x: Fraction, y: Fraction) -> bool:
    return any(x0 < x < x1 and y0 < y < y1 for x0, x1, y0, y1 in rects)


def _maximal_rectangles(
    polygon: PolygonV1, xs: tuple[int, ...], ys: tuple[int, ...]
) -> tuple[tuple[int, int, int, int], ...]:
    """Максимальные вписанные осевые прямоугольники по сжатой сетке фигуры.

    Максимальность не украшение: без неё объединение то же, но прямоугольников в
    разы больше, а счёт площади потом линеен по их числу. Вписанность решается
    префиксной суммой по клеткам, то есть за постоянное время на прямоугольник, —
    иначе перебор был бы шестой степенью по числу координат.
    """

    columns, rows = len(xs) - 1, len(ys) - 1
    prefix = [[0] * (rows + 1) for _ in range(columns + 1)]
    for i in range(columns):
        for j in range(rows):
            filled = _inside(
                polygon,
                Fraction(xs[i] + xs[i + 1], 2),
                Fraction(ys[j] + ys[j + 1], 2),
            )
            prefix[i + 1][j + 1] = (
                prefix[i][j + 1] + prefix[i + 1][j] - prefix[i][j] + int(filled)
            )

    def fits(i0: int, i1: int, j0: int, j1: int) -> bool:
        if i0 < 0 or j0 < 0 or i1 > columns or j1 > rows:
            return False
        count = (
            prefix[i1][j1] - prefix[i0][j1] - prefix[i1][j0] + prefix[i0][j0]
        )
        return count == (i1 - i0) * (j1 - j0)

    result = []
    for i0 in range(columns):
        for i1 in range(i0 + 1, columns + 1):
            for j0 in range(rows):
                for j1 in range(j0 + 1, rows + 1):
                    if not fits(i0, i1, j0, j1):
                        continue
                    if (
                        fits(i0 - 1, i1, j0, j1)
                        or fits(i0, i1 + 1, j0, j1)
                        or fits(i0, i1, j0 - 1, j1)
                        or fits(i0, i1, j0, j1 + 1)
                    ):
                        continue
                    result.append((xs[i0], xs[i1], ys[j0], ys[j1]))
    return tuple(result)


def mitered_inset_rectangles(
    polygon: PolygonV1, alpha: Fraction
) -> tuple[Rect, ...]:
    """Митрованный отступ как объединение сжатых вписанных прямоугольников.

    Ровно та замкнутая форма, которой митрованный ответ был проверен вне обеих
    реализаций: у `ell` возвращается две штуки, и это `[a,12-a]x[a,6-a]` и
    `[a,6-a]x[a,12-a]`.
    """

    lattice_x = tuple(sorted({x for loop in polygon.loops for x, _ in loop.points}))
    lattice_y = tuple(sorted({y for loop in polygon.loops for _, y in loop.points}))
    inset = []
    for x0, x1, y0, y1 in _maximal_rectangles(polygon, lattice_x, lattice_y):
        left, right, low, high = x0 + alpha, x1 - alpha, y0 + alpha, y1 - alpha
        if left < right and low < high:
            inset.append((left, right, low, high))
    return tuple(inset)


def mitered_inset_loop(
    polygon: PolygonV1, alpha: Fraction
) -> tuple[tuple[Fraction, Fraction], ...] | StandardOutcome:
    """Контур митрованного отступа: каждое ребро сдвинуто внутрь на alpha.

    Нужен не для площади — её даёт `rectilinear_standard` — а для того, чтобы
    эталон можно было предъявить закону сохранения `policy_b.py` как МНОЖЕСТВО
    ПЕТЕЛЬ, а не как одно число. Вершина отступа есть пересечение двух сдвинутых
    прямых, и у прямоугольной фигуры они всегда перпендикулярны, поэтому
    пересечение берётся покоординатно, без деления и без радикалов.

    Построение НЕ доказывает себя само: сдвиг рёбер верен только пока отступ
    остаётся односвязным, а при большой alpha он рвётся либо вырождается. Поэтому
    площадь полученной петли сверяется с площадью отступа, посчитанной
    независимо, и расхождение даёт исход `INSET_IS_NOT_ONE_LOOP`, а не молчаливо
    неверный контур. Возвращается либо контур, либо исход — третьего нет.
    """

    if polygon.holes:
        return StandardOutcome.INSET_LOOP_NEEDS_A_FIGURE_WITHOUT_HOLES
    standard = rectilinear_standard(polygon, alpha)
    if standard.outcome is not StandardOutcome.EXACT:
        return standard.outcome

    points = polygon.outer.points
    size = len(points)
    loop: list[tuple[Fraction, Fraction]] = []
    for index in range(size):
        previous = points[index - 1]
        current = points[index]
        following = points[(index + 1) % size]
        shifted_x = shifted_y = None
        for start, end in ((previous, current), (current, following)):
            if start[1] == end[1]:
                shifted_y = (
                    current[1] + alpha if end[0] > start[0] else current[1] - alpha
                )
            else:
                shifted_x = (
                    current[0] - alpha if end[1] > start[1] else current[0] + alpha
                )
        if shifted_x is None or shifted_y is None:
            return StandardOutcome.EDGE_IS_NOT_AXIS_PARALLEL
        loop.append((Fraction(shifted_x), Fraction(shifted_y)))

    doubled = Fraction(0)
    for index in range(size):
        x0, y0 = loop[index]
        x1, y1 = loop[(index + 1) % size]
        doubled += x0 * y1 - x1 * y0
    # Три разных способа не быть контуром, и все три — один исход, потому что
    # различать их нечем: сдвиг рёбер перестал описывать отступ. Совпавшие
    # соседние вершины (два ребра слились), нулевая площадь (ядро выродилось) и
    # несошедшаяся площадь (отступ распался) — каждый из них делает петлю
    # неверной, а не приблизительной.
    if len(set(loop)) != size or standard.inset_area == 0:
        return StandardOutcome.INSET_IS_NOT_ONE_LOOP
    if doubled / 2 != standard.inset_area:
        return StandardOutcome.INSET_IS_NOT_ONE_LOOP
    return tuple(loop)


def reflex_squares(polygon: PolygonV1, alpha: Fraction) -> tuple[Rect, ...]:
    """Квадраты `alpha x alpha` у вогнутых вершин — то, чего не накрывает ни юбка.

    Квадрат растёт из вершины по ВНУТРЕННИМ нормалям обоих её рёбер: юбка каждого
    ребра обрезана концом ребра, поэтому за концом остаётся незакрытый угол, а
    митрованный отступ его накрывает целиком. Сторона выводится из ориентации
    («влево от хода» — внутрь, это нормировано `PolygonV1.build`), а не задаётся
    параметром, иначе выбор стороны стал бы второй точкой принятия решения.
    """

    squares: list[Rect] = []
    for loop in polygon.loops:
        points = loop.points
        for index, is_reflex in enumerate(loop.reflex_flags()):
            if not is_reflex:
                continue
            previous = points[index - 1]
            current = points[index]
            following = points[(index + 1) % len(points)]
            incoming = (
                -(current[1] - previous[1]),
                current[0] - previous[0],
            )
            outgoing = (
                -(following[1] - current[1]),
                following[0] - current[0],
            )
            towards_x = incoming[0] + outgoing[0]
            towards_y = incoming[1] + outgoing[1]
            x_low, x_high = (
                (Fraction(current[0]), current[0] + alpha)
                if towards_x > 0
                else (current[0] - alpha, Fraction(current[0]))
            )
            y_low, y_high = (
                (Fraction(current[1]), current[1] + alpha)
                if towards_y > 0
                else (current[1] - alpha, Fraction(current[1]))
            )
            squares.append((x_low, x_high, y_low, y_high))
    return tuple(squares)


def _refined_axis(values: tuple[int, ...], alpha: Fraction) -> tuple[Fraction, ...]:
    """Ось сжатой сетки: сами координаты входа и они же, сдвинутые на +-alpha.

    Больше узлов не нужно и меньше нельзя: индикаторы фигуры, отступа и полос
    ломаются только на этих прямых.
    """

    return tuple(
        sorted(
            {Fraction(value) for value in values}
            | {value + alpha for value in values}
            | {value - alpha for value in values}
        )
    )


def _refused(
    outcome: StandardOutcome,
    polygon: PolygonV1,
    alpha: Fraction,
    area: Fraction,
    detail: str,
) -> RectilinearStandardV1:
    """Отказ эталона: имя исхода и причина, а не нули без объяснения."""

    return RectilinearStandardV1(
        outcome,
        alpha,
        area,
        Fraction(0),
        Fraction(0),
        Fraction(0),
        polygon.reflex_count,
        Fraction(0),
        detail,
    )


def rectilinear_standard(
    polygon: PolygonV1, alpha: Fraction
) -> RectilinearStandardV1:
    """Три площади прямоугольной фигуры к моменту alpha, в точных дробях."""

    alpha = Fraction(alpha)
    area = Fraction(
        sum(signed_double_area(loop.points) for loop in polygon.loops), 2
    )
    edges = axis_edges(polygon)
    if edges is None:
        return _refused(
            StandardOutcome.EDGE_IS_NOT_AXIS_PARALLEL,
            polygon,
            alpha,
            area,
            "у фигуры есть не осевое ребро",
        )
    if alpha < 0:
        return _refused(
            StandardOutcome.ALPHA_IS_NEGATIVE, polygon, alpha, area, str(alpha)
        )

    inset = mitered_inset_rectangles(polygon, alpha)
    strips = tuple(strip_of(start, end, alpha) for start, end in edges)
    squares = reflex_squares(polygon, alpha)
    grid_x = _refined_axis(
        tuple(sorted({x for loop in polygon.loops for x, _ in loop.points})), alpha
    )
    grid_y = _refined_axis(
        tuple(sorted({y for loop in polygon.loops for _, y in loop.points})), alpha
    )

    cells_area = Fraction(0)
    inset_area = Fraction(0)
    strip_area = Fraction(0)
    square_area = Fraction(0)
    for i in range(len(grid_x) - 1):
        centre_x = (grid_x[i] + grid_x[i + 1]) / 2
        width = grid_x[i + 1] - grid_x[i]
        for j in range(len(grid_y) - 1):
            centre_y = (grid_y[j] + grid_y[j + 1]) / 2
            if not _inside(polygon, centre_x, centre_y):
                continue
            size = width * (grid_y[j + 1] - grid_y[j])
            cells_area += size
            if _in_any(inset, centre_x, centre_y):
                inset_area += size
            if _in_any(strips, centre_x, centre_y):
                strip_area += size
            if _in_any(squares, centre_x, centre_y):
                square_area += size
    if cells_area != area:
        return _refused(
            StandardOutcome.CELLS_DO_NOT_REPRODUCE_THE_POLYGON,
            polygon,
            alpha,
            area,
            f"клетки дали {cells_area}, площадь {area}",
        )
    return RectilinearStandardV1(
        StandardOutcome.EXACT,
        alpha,
        area,
        inset_area,
        area - inset_area,
        strip_area,
        polygon.reflex_count,
        square_area,
    )
