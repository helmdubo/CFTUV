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
