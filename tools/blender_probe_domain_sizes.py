"""Показать размер и форму каждого домена: сколько вершин и сколько вогнутых.

Зачем это нужно, одним абзацем. Скорость очереди событий зависит от формы
контура сильнее, чем от его длины: на гребёнке (вогнутые вершины в ряд) она
идёт как O(n^1.05) и стоит ~2100 мкс на вершину, а на контуре с вогнутыми
вершинами В ОБЩЕМ ПОЛОЖЕНИИ — как O(n^2.0) и ~163 000 мкс на вершину, то есть
в 77 раз дороже при сравнимом размере. Замерено одним и тем же кодом на 128-256
вершинах. Значит вопрос «успеет ли очередь в поле» решается не догадкой про
алгоритм, а двумя числами с настоящего меша: СКОЛЬКО вершин у домена и сколько
из них вогнутых.

Единственная полевая фикстура, которая у нас есть, даёт 12 вершин на домен, и
при таком размере цена очереди 0.02-0.07 с — то есть запас огромный. Но фикстура
одна, а в поле доменов было тринадцать, и их размеры никто не мерил. Этот скрипт
их и меряет.

Что делать с результатом:
- максимум вершин на домен до ~30 — очереди хватает с запасом при любой форме;
- 30-100 и вогнутые в общем положении — надо смотреть, счёт пойдёт на секунды;
- больше 100 в общем положении — считать так нельзя, и это надо знать ДО того,
  как строить дальше, а не после.

Скрипт ничего не меняет: только читает выделение и печатает таблицу.

Запуск: выделить меш → Edit Mode → выделить рёбра-швы, для которых считается
decal → `Scripting` → `Run Script`. Результат в системной консоли
(Window → Toggle System Console на Windows).
"""

from __future__ import annotations

import math

import bmesh
import bpy


# Вогнутость меряется знаком поворота относительно СРЕДНЕЙ нормали петли, а не
# нормали грани: петля шва может идти по нескольким граням с разными нормалями,
# и тогда локальная нормаль дала бы разный знак на одном и том же повороте.
def _loop_normal(points) -> tuple[float, float, float]:
    nx = ny = nz = 0.0
    count = len(points)
    for index in range(count):
        ax, ay, az = points[index]
        bx, by, bz = points[(index + 1) % count]
        nx += ay * bz - az * by
        ny += az * bx - ax * bz
        nz += ax * by - ay * bx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0.0:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def _reflex_count(points, normal) -> int:
    count = len(points)
    reflex = 0
    for index in range(count):
        previous = points[index - 1]
        here = points[index]
        following = points[(index + 1) % count]
        ux, uy, uz = (here[i] - previous[i] for i in range(3))
        vx, vy, vz = (following[i] - here[i] for i in range(3))
        cx = uy * vz - uz * vy
        cy = uz * vx - ux * vz
        cz = ux * vy - uy * vx
        if cx * normal[0] + cy * normal[1] + cz * normal[2] < 0.0:
            reflex += 1
    return reflex


def _loops_of_selected_edges(mesh) -> list[list[int]]:
    """Замкнутые цепочки из выделенных рёбер. Незамкнутые тоже возвращаются."""

    selected = [edge for edge in mesh.edges if edge.select]
    if not selected:
        return []
    neighbours: dict[int, list[int]] = {}
    for edge in selected:
        a, b = edge.verts[0].index, edge.verts[1].index
        neighbours.setdefault(a, []).append(b)
        neighbours.setdefault(b, []).append(a)

    seen: set[int] = set()
    loops: list[list[int]] = []
    for start in neighbours:
        if start in seen:
            continue
        chain = [start]
        seen.add(start)
        current, previous = start, None
        while True:
            following = [
                item for item in neighbours[current] if item != previous
            ]
            following = [item for item in following if item not in seen]
            if not following:
                break
            previous, current = current, following[0]
            seen.add(current)
            chain.append(current)
        loops.append(chain)
    return loops


def main() -> None:
    obj = bpy.context.active_object
    if obj is None or obj.type != "MESH" or obj.mode != "EDIT":
        print("Нужен меш в Edit Mode с выделенными рёбрами швов.")
        return

    mesh = bmesh.from_edit_mesh(obj.data)
    mesh.verts.ensure_lookup_table()
    mesh.edges.ensure_lookup_table()

    loops = _loops_of_selected_edges(mesh)
    if not loops:
        print("Не выделено ни одного ребра.")
        return

    print()
    print(f"Меш: {obj.name}")
    print(f"{'цепочка':>9} {'вершин':>7} {'вогнутых':>9} {'замкнута':>9} "
          f"{'оценка очереди':>16}")
    worst = 0
    for number, chain in enumerate(loops):
        points = [tuple(mesh.verts[index].co) for index in chain]
        closed = len(chain) > 2 and any(
            edge.select
            and {edge.verts[0].index, edge.verts[1].index}
            == {chain[0], chain[-1]}
            for edge in mesh.edges
        )
        normal = _loop_normal(points)
        reflex = _reflex_count(points, normal) if len(points) >= 3 else 0
        size = len(chain)
        worst = max(worst, size)
        # Оценка по замеренным константам: гребёнка ~2100 мкс/вершину при
        # O(n^1.05), общее положение ~163 000 мкс/вершину при O(n^2.0).
        # Печатается ВИЛКА, а не одно число: форму по счёту вершин не угадать.
        low = 2.1e-3 * size
        high = 1.63e-1 * size * (size / 129.0)
        print(f"{number:>9} {size:>7} {reflex:>9} {str(closed):>9} "
              f"{low:>7.2f}-{high:<8.2f} с")

    print()
    print(f"Максимум вершин на цепочку: {worst}")
    if worst <= 30:
        print("  -> очереди хватает с запасом при ЛЮБОЙ форме контура.")
    elif worst <= 100:
        print("  -> пограничная зона: если вогнутые в общем положении, счёт "
              "пойдёт на секунды. Нужен замер, а не оценка.")
    else:
        print("  -> при вогнутых в общем положении так считать нельзя. "
              "Это надо знать ДО того, как строить дальше.")
    print()
    print("Вилка широкая намеренно: 77-кратная разница между гребёнкой и общим "
          "положением — это форма, а не размер, и по счёту вершин её не видно.")


if __name__ == "__main__":
    main()
