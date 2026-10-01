"""Тесселяция грани отсечением ушей: ТОЧНО, детерминированно, без дыр.

Вход — контур одной слитой грани покрытия (`SqrtSumV1`-точки). Грань скелета
проста по границе 1 `build_faces`, слитый контур доказан простым тем же
предикатом (`coalesce.merge_same_chain_faces`), а дыр у грани нет: поэтому
отсечение ушей без дыр — не упрощение, а точная постановка.

ПОРЯДОК И ДЕТЕРМИНИЗМ. Вершины идут в порядке контура; ухом выбирается ПЕРВОЕ
допустимое в текущем списке. Ни случайности, ни зависимости от порядка
хеширования: один контур — одна триангуляция. Ей это и не нужно быть
единственной: `TessellationDigestEquivalence` говорит, что все допустимые
тесселяции одного упорядочения дают один семантический дайджест, а граница
(вершины контура) триангуляция не меняет — новых вершин она не вводит.

ВСЕ ВЕРШИНЫ КОНТУРА ВХОДЯТ В ТРЕУГОЛЬНИКИ. Прямая вершина на границе (середина
слитой цепи) не может быть кончиком уха — вырожденный треугольник, — но
остаётся вершиной соседнего уха. Потерять её значило бы дать T-стык с соседней
гранью, которая её несёт.

ЗНАКИ — ТОЧНЫЕ И ПОД БЮДЖЕТОМ: `faces.orientation(..., budget)`. Исчерпание
бюджета поднимается наружу как есть (`ExactCanonicalizationWorkBudgetExhausted`)
и названо материализатором отдельным исходом.

Не сложилось — `None`, и материализатор называет это
`TESSELLATION_DID_NOT_CLOSE`: молчаливого запасного разбиения нет.

ЧЕТЫРЁХГРАННИК. Закон `QUAD_STRIPS_V1` оставляет грань целым четырёхугольником,
если контур СТРОГО выпуклый (`convex_quad_ring`): все четыре поворота строго
влево на кольце против часовой. Строгость точная, под бюджетом, тем же
предикатом `orientation`: прямой угол вершины (плоский, `0`) и невыпуклая
вершина (`< 0`) одинаково выводят контур из закона.

МНОГОУГОЛЬНИК. Закон `PLANAR_POLYGONS_V1` оставляет целым контур любой длины от
четырёх, у которого нет ПРАВОГО поворота (`convex_polygon_ring`): повороты `>= 0`,
площадь не нуль. Вершина на прямой допустима — она несёт T-стык соседней грани, а
потерять её значило бы дать дыру; триангуляция такого контура (`triangulate_exact`)
всё равно даёт `n - 2` треугольников, каждый невырожденный. Простота контура
доказана выше по конвейеру (граница 1), поэтому «нет правых поворотов» здесь
означает выпуклость, а не звезду.
"""

from __future__ import annotations

from ..wavefront.faces import doubled_shoelace, orientation


def _ear_contains_vertex(points, budget, a: int, b: int, c: int, w: int) -> bool:
    """Лежит ли вершина `w` в ЗАМКНУТОМ треугольнике `(a, b, c)` (против часовой)."""

    return (
        orientation(points[a], points[b], points[w], budget) >= 0
        and orientation(points[b], points[c], points[w], budget) >= 0
        and orientation(points[c], points[a], points[w], budget) >= 0
    )


def triangulate_exact(points, budget):
    """Треугольники `((i, j, k), ...)` по индексам контура либо `None`.

    Треугольники ориентированы ПРОТИВ часовой (в системе координат карты), чем
    бы ни был ориентирован вход. Площадь каждого строго положительна, число
    треугольников `n - 2`, сумма удвоенных площадей равна площади контура.
    """

    count = len(points)
    if count < 3:
        return None
    total = doubled_shoelace(tuple(points)).sign(budget=budget)
    if total == 0:
        return None
    ring = list(range(count))
    if total < 0:
        ring.reverse()

    def convex(position: int) -> bool:
        size = len(ring)
        return (
            orientation(
                points[ring[position - 1]],
                points[ring[position]],
                points[ring[(position + 1) % size]],
                budget,
            )
            > 0
        )

    flags = [convex(position) for position in range(count)]
    triangles: list[tuple[int, int, int]] = []
    while len(ring) > 3:
        size = len(ring)
        for position in range(size):
            if not flags[position]:
                continue
            previous, current = ring[position - 1], ring[position]
            following = ring[(position + 1) % size]
            # Внутри уха может лежать только невыпуклая (развёрнутая или
            # плоская) вершина: если внутри ушной вершины окажется выпуклая,
            # то и невыпуклая найдётся. Поэтому проверяются только они.
            if any(
                not flags[other]
                and ring[other] not in (previous, current, following)
                and _ear_contains_vertex(
                    points, budget, previous, current, following, ring[other]
                )
                for other in range(size)
            ):
                continue
            triangles.append((previous, current, following))
            del ring[position]
            del flags[position]
            size -= 1
            before = (position - 1) % size
            flags[before] = convex(before)
            flags[position % size] = convex(position % size)
            break
        else:
            return None
    last = tuple(ring)
    if orientation(points[last[0]], points[last[1]], points[last[2]], budget) <= 0:
        return None
    triangles.append(last)
    return tuple(triangles)


def convex_quad_ring(points, budget):
    """Кольцо индексов `(i0, i1, i2, i3)` против часовой, если 4 точки — СТРОГО выпуклый контур.

    Иначе `None`: не четыре точки, нулевая площадь, либо хоть один поворот не
    строго влево. Все четыре поворота влево у замкнутого четырёхугольника
    означают простой выпуклый контур (сумма внешних углов меньше `4π`, поэтому
    обход один), и у такого `triangulate_exact` режет по первому уху, а
    `fan_out` повторяет это разбиение по ключам.
    """

    if len(points) != 4:
        return None
    total = doubled_shoelace(tuple(points)).sign(budget=budget)
    if total == 0:
        return None
    ring = (0, 1, 2, 3) if total > 0 else (3, 2, 1, 0)
    for position in range(4):
        turn = orientation(
            points[ring[position - 1]],
            points[ring[position]],
            points[ring[(position + 1) % 4]],
            budget,
        )
        if turn <= 0:
            return None
    return ring


def convex_polygon_ring(points, budget):
    """Кольцо индексов против часовой, если контур из `>= 4` точек выпуклый (вершины на прямой допускаются).

    Иначе `None`: меньше четырёх точек, нулевая площадь либо хоть один поворот
    строго вправо. Знаки точные, под бюджетом (`orientation`).
    """

    count = len(points)
    if count < 4:
        return None
    total = doubled_shoelace(tuple(points)).sign(budget=budget)
    if total == 0:
        return None
    ring = tuple(range(count)) if total > 0 else tuple(range(count - 1, -1, -1))
    for position in range(count):
        turn = orientation(
            points[ring[position - 1]],
            points[ring[position]],
            points[ring[(position + 1) % count]],
            budget,
        )
        if turn < 0:
            return None
    return ring


def fan_out(keys):
    """Грань как треугольники закона `TRIANGLES_V1`: `((a, b, c), ...)` по ключам вершин.

    Треугольник — он сам. Четырёхугольник `(q0, q1, q2, q3)` в порядке кольца
    (против часовой в координатах карты) — ровно ПЕРВОЕ ухо `triangulate_exact`:
    у строго выпуклого контура все четыре вершины выпуклы, ухом идёт первая в
    списке (предыдущая `q3`, текущая `q0`, следующая `q1`), остаток `(q1, q2, q3)`
    замыкает разбиение. Диагональ — `q1 q3`. Четырёхгранью грань закона бывает
    только строго выпуклая, поэтому у неё это разбиение допустимо, а не
    выбрано наугад.

    Другой длины у грани закона нет: больше четырёх вершин — слитые пробеги,
    они остаются треугольниками закона, а не многоугольниками.
    """

    if len(keys) == 3:
        return (tuple(keys),)
    if len(keys) != 4:
        raise ValueError(f"a face of {len(keys)} vertices has no canonical split")
    first, second, third, fourth = keys
    return ((fourth, first, second), (second, third, fourth))
