"""Показать углы поворота на углах патчей и найти нерациональные.

Хост-адаптер требует, чтобы `atan2(sin, cos) / pi` был **точной рациональной
дробью** (`envelope_request_export.py:_decimal_interval_from_rational`).
Угол 90° даёт 1/2, 45° — 1/4, 60° — 1/3. Любой другой угол не выражается
дробью, и весь патч отвергается с
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`.

Это ограничение представления, а не семантики: сертификат в итоге всё равно
превращается в интервал. Скрипт показывает, какие именно углы не проходят,
чтобы не гадать.

Запуск: Edit Mode, выделены нужные рёбра → `Scripting` → `Run Script`.
Скрипт ничего не меняет.
"""

from __future__ import annotations

import math

import bmesh
import bpy


def _chord_angle(previous_chord, next_chord) -> float:
    ax, ay, az = previous_chord
    bx, by, bz = next_chord
    dot = ax * bx + ay * by + az * bz
    cross = (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )
    sine = math.sqrt(sum(item * item for item in cross))
    return math.atan2(sine, dot)


def main() -> None:
    obj = bpy.context.active_object
    if obj is None or obj.type != "MESH" or obj.mode != "EDIT":
        print("Нужен меш в Edit Mode.")
        return

    mesh = bmesh.from_edit_mesh(obj.data)
    mesh.verts.ensure_lookup_table()
    mesh.faces.ensure_lookup_table()

    print(f"\n=== углы поворота, объект {obj.name} ===")
    print("  доля от pi: 0.5 = 90°, 0.25 = 45°, 0.333… = 60°")
    print("  «дробь?» = проходит ли текущее требование хост-адаптера\n")

    suspicious = 0
    total = 0
    for face in mesh.faces:
        if not face.select:
            continue
        loop_count = len(face.verts)
        for index, loop in enumerate(face.loops):
            previous_vertex = face.loops[index - 1].vert.co
            current_vertex = loop.vert.co
            next_vertex = face.loops[(index + 1) % loop_count].vert.co
            incoming = tuple(current_vertex[axis] - previous_vertex[axis] for axis in range(3))
            outgoing = tuple(next_vertex[axis] - current_vertex[axis] for axis in range(3))
            if not any(incoming) or not any(outgoing):
                continue
            total += 1
            angle = _chord_angle(incoming, outgoing)
            ratio = angle / math.pi
            # Нас интересует, ложится ли доля на «удобную» дробь со знаменателем
            # до 12 — именно такие sympy выражает точно.
            nice = any(
                abs(ratio * denominator - round(ratio * denominator)) < 1e-9
                for denominator in range(1, 13)
            )
            if not nice:
                suspicious += 1
                print(
                    f"  face {face.index:4d} vert {loop.vert.index:5d}  "
                    f"{math.degrees(angle):8.4f}°  доля={ratio:.10f}  дробь? НЕТ"
                )

    print(f"\n  всего углов: {total}")
    print(f"  не выражаются дробью: {suspicious}")
    if suspicious:
        print(
            "\n  Эти углы и валят патч. Ограничение снимается переходом на\n"
            "  сертифицированный интервал вместо точной дроби — тот же приём,\n"
            "  что уже применён для знака в ядре."
        )
    else:
        print("\n  Все углы «удобные» — причина отказа в чём-то другом.")


main()
