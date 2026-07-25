"""Таблица «масштаб решётки → что он сливает и насколько двигает».

Выбор шага — решение владельца по картинке в Blender. Этот стенд не выбирает,
он показывает натяжение, которое иначе остаётся на словах:

- мелкий шаг — задуманное вырождение не восстанавливается, точки, которые
  должны были совпасть, остаются разными, и центральный патч остаётся как был;
- крупный шаг — вырождение восстанавливается, но декали заметно едут.

Модель входа: контур патча плюс группы точек, которые ЗАДУМАНЫ совпадающими и
разнесены шумом binary64. Именно так выглядит `building.002`: полевые углы
отклоняются от прямого на ~7e-6 рад, что на патче в метры даёт расхождение
порядка 1e-4 м между точками, обязанными совпасть.

Стенд ничего не меняет и ни от чего не зависит, кроме ядра.

Запуск:
    PYTHONPATH=kernel/src python3 tools/benchmark_grid_scale.py
    PYTHONPATH=kernel/src python3 tools/benchmark_grid_scale.py --json out.json
"""

from __future__ import annotations

import argparse
import json
from fractions import Fraction

from cftuv_envelope.robust.grid import (
    SNAP_COUNTS,
    SNAP_DISPLACEMENT_MAX_SQUARED,
    reset_snap_counts,
)
from cftuv_envelope.robust.snapping import grid_for_extent, snap_offset


# Габарит патча в метрах и шум, разносящий точки, которые обязаны совпасть.
PATCH_EXTENT = 4
FIELD_NOISE = Fraction(1, 10**4)


def _intended_meetings(groups: int, per_group: int):
    """Точки, задуманные совпадающими, разнесённые полевым шумом."""

    points = []
    for group in range(groups):
        centre_x = Fraction(PATCH_EXTENT * (group + 1), groups + 1)
        centre_y = Fraction(PATCH_EXTENT, 2)
        for index in range(per_group):
            offset = FIELD_NOISE * Fraction(index - per_group // 2, max(per_group, 1))
            points.append((centre_x + offset, centre_y - offset))
    return points


def _contour():
    return [
        (Fraction(0), Fraction(0)),
        (Fraction(PATCH_EXTENT), Fraction(0)),
        (Fraction(PATCH_EXTENT), Fraction(PATCH_EXTENT)),
        (Fraction(0), Fraction(PATCH_EXTENT)),
    ]


def measure(node_counts: tuple[int, ...], groups: int = 5, per_group: int = 3):
    contour = _contour()
    meetings = _intended_meetings(groups, per_group)
    intended = groups
    rows = []
    for nodes_across in node_counts:
        grid = grid_for_extent(contour, nodes_across=nodes_across)
        reset_snap_counts()
        snapped = [snap_offset(x, y, grid) for x, y in meetings]
        distinct = len({(item.x, item.y) for item in snapped})
        step = Fraction(1, grid.scale)
        rows.append(
            {
                "nodes_across": nodes_across,
                "scale": grid.scale,
                "step_metres": float(step),
                "merged_points": SNAP_COUNTS["merged_points"],
                "points_moved": SNAP_COUNTS["points_moved"],
                "distinct_nodes": distinct,
                "intended_nodes": intended,
                "restored": distinct == intended,
                "displacement_max": float(SNAP_DISPLACEMENT_MAX_SQUARED[0]) ** 0.5,
            }
        )
    reset_snap_counts()
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=str, default=None)
    arguments = parser.parse_args()

    rows = measure((64, 256, 1024, 4096, 16384, 65536))

    print(f"\n=== габарит патча {PATCH_EXTENT} м, шум {float(FIELD_NOISE):g} м ===")
    print(
        "  узлов  масштаб    шаг, м   слито  сдвинуто  узлов/задумано  "
        "макс.сдвиг, м  вырождение"
    )
    for row in rows:
        verdict = "восстановлено" if row["restored"] else "НЕ восстановлено"
        print(
            f"  {row['nodes_across']:5d}  {row['scale']:7d}  {row['step_metres']:8.2e}"
            f"  {row['merged_points']:5d}  {row['points_moved']:8d}"
            f"  {row['distinct_nodes']:6d}/{row['intended_nodes']:<8d}"
            f"  {row['displacement_max']:12.2e}  {verdict}"
        )

    print(
        "\n  Шаг обязан быть КРУПНЕЕ шума, иначе задуманное совпадение не\n"
        "  восстанавливается. Он же обязан быть мельче того, что видно глазом\n"
        "  на декали. Между этими двумя границами и выбирается значение —\n"
        "  выбор владельца по картинке в Blender, не стенда."
    )

    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as handle:
            json.dump(
                {"schema": "cftuv.envelope.grid_scale.v1", "rows": rows},
                handle,
                ensure_ascii=False,
                indent=2,
            )


if __name__ == "__main__":
    main()
