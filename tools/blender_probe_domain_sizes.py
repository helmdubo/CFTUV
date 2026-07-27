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


def _components_of_selected_edges(mesh):
    """Связные куски выделенного графа швов, с их СТЕПЕНЯМИ.

    Почему не «цепочки». Первая версия зонда шла по рёбрам и на развилке брала
    первого попавшегося соседа, а остальные ветки теряла. Для простой цепочки
    это верно, а для графа с развилками — нет: весь выделенный граф выходил
    одной «цепочкой», и колонки «вогнутых» и «замкнута» у неё смысла не имели,
    хотя печатались. Поймано полем: владелец выделил все швы и получил строку
    «24 вершины, 6 вогнутых, не замкнута», где верно было только число вершин.

    Теперь возвращается компонента целиком, а форма НЕ утверждается: считаются
    вершины степени 1 (концы) и степени 3+ (развилки). Простая цепочка — это
    ровно «развилок ноль», и только для неё имеет смысл говорить про вогнутость
    и замкнутость.
    """

    selected = [edge for edge in mesh.edges if edge.select]
    if not selected:
        return []
    neighbours: dict[int, set[int]] = {}
    for edge in selected:
        a, b = edge.verts[0].index, edge.verts[1].index
        neighbours.setdefault(a, set()).add(b)
        neighbours.setdefault(b, set()).add(a)

    seen: set[int] = set()
    components = []
    for start in neighbours:
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        members: list[int] = []
        while stack:
            current = stack.pop()
            members.append(current)
            for following in neighbours[current]:
                if following not in seen:
                    seen.add(following)
                    stack.append(following)
        ends = [v for v in members if len(neighbours[v]) == 1]
        forks = [v for v in members if len(neighbours[v]) >= 3]
        # Рёбра нужны для проверки Эйлера: у замкнутой сети швов на замкнутой
        # поверхности число доменов равно `2 - V + E`, и это единственный способ
        # узнать ЧИСЛО доменов, не строя их. Без него зонд говорит про размер
        # сети, но молчит про то, на сколько кусков она эту поверхность делит.
        edge_count = sum(len(neighbours[v]) for v in members) // 2
        components.append(
            {
                "members": members,
                "ends": len(ends),
                "forks": len(forks),
                "edges": edge_count,
                "neighbours": neighbours,
            }
        )
    return components


def _ordered_simple_chain(component) -> list[int] | None:
    """Порядок обхода, если компонента — ПРОСТАЯ цепочка; иначе None."""

    if component["forks"]:
        return None
    neighbours = component["neighbours"]
    members = component["members"]
    ends = [v for v in members if len(neighbours[v]) == 1]
    if len(ends) not in (0, 2):
        return None
    start = ends[0] if ends else members[0]
    chain = [start]
    previous, current = None, start
    while True:
        following = [v for v in neighbours[current] if v != previous]
        following = [v for v in following if v not in chain]
        if not following:
            break
        previous, current = current, following[0]
        chain.append(current)
    return chain if len(chain) == len(members) else None


def main() -> None:
    obj = bpy.context.active_object
    if obj is None or obj.type != "MESH" or obj.mode != "EDIT":
        print("Нужен меш в Edit Mode с выделенными рёбрами швов.")
        return

    mesh = bmesh.from_edit_mesh(obj.data)
    mesh.verts.ensure_lookup_table()
    mesh.edges.ensure_lookup_table()

    components = _components_of_selected_edges(mesh)
    if not components:
        print("Не выделено ни одного ребра.")
        return

    print()
    print(f"Меш: {obj.name}")
    print(f"{'кусок':>6} {'вершин':>7} {'рёбер':>6} {'развилок':>9} "
          f"{'концов':>7} {'вогнутых':>9} {'замкнут':>8} {'оценка очереди':>16}")
    worst = 0
    for number, component in enumerate(components):
        size = len(component["members"])
        worst = max(worst, size)
        chain = _ordered_simple_chain(component)
        if chain is None:
            # Форма не утверждается: у графа с развилками «вогнутых» и
            # «замкнут» смысла не имеют, и печатать туда число значило бы
            # соврать точно так же, как печатала первая версия зонда.
            reflex_text, closed_text = "—", "—"
        else:
            points = [tuple(mesh.verts[index].co) for index in chain]
            closed_text = str(component["ends"] == 0)
            normal = _loop_normal(points)
            reflex_text = str(
                _reflex_count(points, normal) if len(points) >= 3 else 0
            )
        # Оценка по замеренным константам: гребёнка ~2100 мкс/вершину при
        # O(n^1.05), общее положение ~163 000 мкс/вершину при O(n^2.0).
        # Печатается ВИЛКА, а не одно число: форму по счёту вершин не угадать.
        low = 2.1e-3 * size
        high = 1.63e-1 * size * (size / 129.0)
        print(f"{number:>6} {size:>7} {component['edges']:>6} "
              f"{component['forks']:>9} {component['ends']:>7} "
              f"{reflex_text:>9} {closed_text:>8} "
              f"{low:>7.2f}-{high:<8.2f} с")
        if component["ends"] == 0 and component["forks"]:
            # Замкнутая сеть швов делит поверхность на грани, и это ДОМЕНЫ.
            # Формула Эйлера для сферы: V - E + F = 2. На поверхности другого
            # рода число выйдет меньше — расхождение с числом доменов полевого
            # прогона тогда и есть измерение рода, а не ошибка.
            faces = 2 - size + component["edges"]
            print(f"       замкнутая сеть: доменов по Эйлеру (сфера) = "
                  f"2 - {size} + {component['edges']} = {faces}, "
                  f"в среднем {2 * component['edges'] / faces:.1f} шва на домен")

    print()
    print(f"Максимум вершин на кусок сети: {worst}")
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
