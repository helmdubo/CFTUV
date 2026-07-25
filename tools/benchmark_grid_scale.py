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

# Расстояния между РАЗЛИЧНЫМИ деталями декали, которые обязаны выжить.
#
# Без этой оси стенд оптимистичен по построению: группы «задуманных совпадений»
# разнесены на 0.67 м, поэтому даже шаг 6 см показывает «вырождение
# восстановлено» — хотя на нём две настоящие детали в 1 см сливаются в одну.
# Замерено: при шаге 6.25e-2 сливается всё вплоть до 1 см, при 3.91e-3 — 1 мм.
# Одной границы недостаточно, чтобы выбрать шаг; нужны обе.
FEATURE_SEPARATIONS = (
    Fraction(1, 1000),
    Fraction(5, 1000),
    Fraction(1, 100),
)


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
                "destroyed": _destroyed_separations(grid),
            }
        )
    reset_snap_counts()
    return rows


def _destroyed_separations(grid) -> list[float]:
    """Расстояния из `FEATURE_SEPARATIONS`, которые этот шаг СХЛОПЫВАЕТ.

    Две точки на расстоянии d — не задуманное совпадение, а настоящая деталь.
    Если решётка кладёт их в один узел, деталь уничтожена, и никакой счётчик
    «вырождение восстановлено» об этом не скажет.
    """

    destroyed = []
    anchor = Fraction(PATCH_EXTENT, 2)
    for separation in FEATURE_SEPARATIONS:
        left = snap_offset(anchor, anchor, grid)
        right = snap_offset(anchor + separation, anchor, grid)
        if (left.x, left.y) == (right.x, right.y):
            destroyed.append(float(separation))
    return destroyed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=str, default=None)
    arguments = parser.parse_args()

    rows = measure((64, 256, 1024, 4096, 16384, 65536))

    print(f"\n=== габарит патча {PATCH_EXTENT} м, шум {float(FIELD_NOISE):g} м ===")
    print(
        "     шаг, м  узлов/задумано  макс.сдвиг, м  вырождение        "
        "уничтожает детали"
    )
    usable = []
    for row in rows:
        verdict = "восстановлено   " if row["restored"] else "НЕ восстановлено"
        if row["destroyed"]:
            damage = ", ".join(f"{item * 1000:g} мм" for item in row["destroyed"])
        else:
            damage = "—"
        if row["restored"] and not row["destroyed"]:
            usable.append(row["step_metres"])
        print(
            f"  {row['step_metres']:9.2e}  {row['distinct_nodes']:6d}/"
            f"{row['intended_nodes']:<8d}  {row['displacement_max']:12.2e}"
            f"  {verdict}  {damage}"
        )

    print(
        "\n  Границ ДВЕ, и обе меряются здесь. Снизу: шаг обязан быть крупнее\n"
        "  шума, иначе задуманное совпадение не восстанавливается. Сверху: шаг\n"
        "  обязан быть мельче самой мелкой настоящей детали декали, иначе она\n"
        "  схлопывается — а счётчик «вырождение восстановлено» об этом молчит,\n"
        "  потому что меряет другое."
    )
    if usable:
        print(
            f"\n  Пригодное окно по этим двум границам: "
            f"{min(usable):.2e} … {max(usable):.2e} м."
        )
    else:
        print(
            "\n  Пригодного окна НЕТ: самая мелкая деталь мельче шума, и\n"
            "  привязка не может одновременно сливать одно и сохранять другое.\n"
            "  Это не настройка, а именованный отказ."
        )
    print("  Выбор внутри окна — владельца по картинке в Blender, не стенда.")

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
