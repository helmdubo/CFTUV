"""Показать углы поворота на углах патчей и найти неразрешимые.

Требования «доля от π обязана быть точной дробью» больше нет: угол измеряется
сертифицированной интервальной оболочкой
(`kernel/src/cftuv_envelope/reference/angle_measure.py`), и 89.9996° проходит
так же, как ровные 90°.

Осталось одно место, где угол может не дать ответа: выбор профиля берёт k из
полосы `δ ∈ (k/3, (k+1)/3]`, и угол ровно на границе полосы не доказывает ни
k, ни k+1. Ответ на это — `ANGULAR_PROFILE_SELECTION_UNCERTAIN`. Скрипт
показывает, какие именно углы стоят у границы, чтобы не гадать.

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
    print("  границы полос выбора профиля: 1/3 = 60°, 2/3 = 120°\n")

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
            # Оболочка угла имеет ширину ~1e-28, так что k не разрешается
            # только у самой границы полосы. Порог здесь щедрый намеренно:
            # скрипт показывает, на что смотреть, а решает ядро.
            margin = min(abs(ratio - bound / 3.0) for bound in (0, 1, 2, 3))
            if margin < 1e-6:
                suspicious += 1
                print(
                    f"  face {face.index:4d} vert {loop.vert.index:5d}  "
                    f"{math.degrees(angle):8.4f}°  доля={ratio:.10f}  "
                    f"до границы полосы={margin:.2e}"
                )

    print(f"\n  всего углов: {total}")
    print(f"  стоят у границы полосы выбора k: {suspicious}")
    if total == 0:
        # Из нуля образцов нельзя заключить «всё хорошо». Прошлая версия
        # скрипта именно это и делала — и ввела в заблуждение.
        print(
            "\n  ВЫВОДА НЕТ: не выделено ни одной грани.\n"
            "  Tab (Edit Mode) -> клавиша 3 (Face Select) -> кликнуть патч."
        )
        return
    print(
        "\n  Замечание: скрипт меряет углы КАЖДОЙ грани, включая внутренние\n"
        "  диагонали патча. Адаптер меряет только углы граничных цепочек, так\n"
        "  что часть строк выше к отказу отношения не имеет. Значимы те, что\n"
        "  почти-но-не-ровно 90°: это шум координат на прямых углах."
    )
    if suspicious:
        print(
            "\n  По этим углам профиль может не выбраться: интервал накрывает\n"
            "  границу полосы и не доказывает ни k, ни k+1. Отказ будет\n"
            "  именованным — ANGULAR_PROFILE_SELECTION_UNCERTAIN."
        )
    else:
        print(
            "\n  Все углы далеко от границ полос — причина отказа, если он\n"
            "  есть, не в измерении угла."
        )


main()
