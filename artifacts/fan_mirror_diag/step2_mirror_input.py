"""FAN-MIRROR-DIAG, шаг 2: зеркальность ВХОДА до и после привязки к решётке.

Отражение — вертикальная ось карты `u -> S - u` (v не трогается). Ось не
подбирается «на глаз»: она задаётся парой окон, и её СОГЛАСОВАННОСТЬ
(`P.left + Q.right == P.right + Q.left`) сама и есть проверка зеркальности.
Расхождение меряется в ШАГАХ РЕШЁТКИ.

Ответ на альтернативу (б): если после снапа зеркальность пары точная, снап из
подозреваемых выбывает; если нет — величина поломки записана числом.
"""

from __future__ import annotations

import json
from fractions import Fraction
from pathlib import Path

OUT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/fan_diag")
import sys
SCALE = int(sys.argv[1]) if len(sys.argv) > 1 else 32768


def load():
    payload = json.loads((OUT / f"fixture_wall_2_001_lattice_{SCALE}.json").read_text())
    order = payload["loop_order"]
    nodes = [(item["u"], item["v"]) for item in payload["vertices_snapped"]]
    exact_pts = [
        (Fraction(item["u"]), Fraction(item["v"]))
        for item in payload["vertices_chart_exact"]
    ]
    return payload, order, nodes, exact_pts


def find_windows(order, nodes, exact_pts):
    """Проёмы контура: ход вниз-вверх относительно нижней кромки.

    Опознаются структурно, без порогов: вершина контура, у которой v — нижняя
    кромка, и следующая по контуру вершина с тем же u и большим v, открывает
    проём; закрывает его симметричная пара. Промежуточные вершины на
    вертикальных сторонах (T-вершины маршрута) проёму не мешают.
    """

    size = len(order)
    bottom = min(v for _, v in nodes)
    windows = []
    index = 0
    while index < size:
        here = nodes[index]
        if here[1] != bottom:
            index += 1
            continue
        # идём вверх по вертикали, пропуская T-вершины
        walk = index
        while (
            nodes[(walk + 1) % size][0] == here[0]
            and nodes[(walk + 1) % size][1] > nodes[walk][1]
        ):
            walk = (walk + 1) % size
        if walk == index:
            index += 1
            continue
        top_left = nodes[walk]
        following = nodes[(walk + 1) % size]
        if following[1] != top_left[1]:
            index += 1
            continue
        # вниз по другой вертикали
        down = (walk + 1) % size
        while (
            nodes[(down + 1) % size][0] == following[0]
            and nodes[(down + 1) % size][1] < nodes[down][1]
        ):
            down = (down + 1) % size
        if nodes[down][1] != bottom:
            index += 1
            continue
        left_u, right_u = here[0], nodes[down][0]
        windows.append(
            {
                "loop_indices": [index, walk, (walk + 1) % size, down],
                "corner_vertices": [
                    order[index],
                    order[walk],
                    order[(walk + 1) % size],
                    order[down],
                ],
                "u_outer_side": left_u,
                "u_other_side": right_u,
                "u_left": min(left_u, right_u),
                "u_right": max(left_u, right_u),
                "v_bottom": bottom,
                "v_top": top_left[1],
                "width_lattice": abs(right_u - left_u),
                "height_lattice": top_left[1] - bottom,
                "u_left_exact": str(
                    min(exact_pts[index][0], exact_pts[down][0])
                ),
                "u_right_exact": str(
                    max(exact_pts[index][0], exact_pts[down][0])
                ),
                "width_exact": str(abs(exact_pts[down][0] - exact_pts[index][0])),
            }
        )
        index = (down + 1) if down > index else size
    return windows


def mirror_gap(first, second, key_left="u_left", key_right="u_right"):
    """Несогласованность оси пары в единицах решётки (0 = точное зеркало)."""

    return (first[key_left] + second[key_right]) - (
        first[key_right] + second[key_left]
    )


def main() -> None:
    payload, order, nodes, exact_pts = load()
    windows = find_windows(order, nodes, exact_pts)
    windows.sort(key=lambda w: w["u_right"], reverse=True)
    for ordinal, window in enumerate(windows):
        window["label"] = "ABCDEFGH"[ordinal]

    report = {
        "schema": "cftuv.fan_diag.mirror_input.v1",
        "scale": SCALE,
        "window_count": len(windows),
        "windows": windows,
        "window_widths_lattice": sorted({w["width_lattice"] for w in windows}),
        "window_widths_exact": sorted({w["width_exact"] for w in windows}),
        "windows_congruent_after_snap": len(
            {(w["width_lattice"], w["height_lattice"]) for w in windows}
        )
        == 1,
        "windows_congruent_before_snap": len({w["width_exact"] for w in windows})
        == 1,
    }

    # 1. Глобальная зеркальность ВСЕЙ фигуры.
    node_set = set(nodes)
    axes = sorted({a[0] + b[0] for a in nodes for b in nodes})
    global_hits = [
        doubled
        for doubled in axes
        if {(doubled - u, v) for u, v in nodes} == node_set
    ]
    report["global_mirror_axes_doubled"] = global_hits
    report["figure_is_globally_mirror_symmetric"] = bool(global_hits)

    # 2. Все пары окон: точность зеркала окно-в-окно, в шагах решётки.
    pair_rows = []
    for i, first in enumerate(windows):
        for second in windows[i + 1 :]:
            gap = mirror_gap(first, second)
            doubled = first["u_left"] + second["u_right"]
            imaged = {
                (doubled - first["u_left"], first["v_top"]),
                (doubled - first["u_right"], first["v_top"]),
                (doubled - first["u_left"], first["v_bottom"]),
                (doubled - first["u_right"], first["v_bottom"]),
            }
            target = {
                (second["u_left"], second["v_top"]),
                (second["u_right"], second["v_top"]),
                (second["u_left"], second["v_bottom"]),
                (second["u_right"], second["v_bottom"]),
            }
            exact_gap = (
                Fraction(first["u_left_exact"]) + Fraction(second["u_right_exact"])
            ) - (
                Fraction(first["u_right_exact"]) + Fraction(second["u_left_exact"])
            )
            whole = {(doubled - u, v) for u, v in nodes}
            pair_rows.append(
                {
                    "pair": f"{first['label']}<->{second['label']}",
                    "doubled_axis": doubled,
                    "axis_gap_lattice_steps": gap,
                    "window_maps_exactly": gap == 0 and imaged == target,
                    "axis_gap_before_snap_exact": str(exact_gap),
                    "axis_gap_before_snap_lattice_steps": float(exact_gap * SCALE),
                    "whole_figure_symmetric": whole == node_set,
                    "whole_figure_unmatched": len(node_set - whole),
                }
            )
    report["window_pairs"] = pair_rows

    # 3. Локальная зеркальность через ПРОСТЕНОК: соседняя пара окон.
    pier_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        doubled = right_window["u_left"] + left_window["u_right"]
        gap = mirror_gap(right_window, left_window)
        bands = []
        for radius in (10648, 21299, 31949, 42598, 63897, 10 ** 9):
            centre = Fraction(doubled, 2)
            band = {p for p in nodes if abs(Fraction(p[0]) - centre) <= radius}
            imaged = {(doubled - u, v) for u, v in band}
            bands.append(
                {
                    "radius_lattice": radius,
                    "radius_metres": float(Fraction(radius, SCALE))
                    if radius < 10 ** 9
                    else None,
                    "band_nodes": len(band),
                    "symmetric": imaged == band,
                    "unmatched": sorted(band - imaged),
                }
            )
        pier_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "doubled_axis": doubled,
                "axis_gap_lattice_steps": gap,
                "pier_width_lattice": left_window["u_right"] - right_window["u_left"],
                "inner_top_corner_pair": {
                    "right_window_inner_top": [
                        right_window["u_left"],
                        right_window["v_top"],
                    ],
                    "left_window_inner_top": [
                        left_window["u_right"],
                        left_window["v_top"],
                    ],
                },
                "two_windows_map_exactly": gap == 0,
                "bands": bands,
                "symmetry_radius_lattice": max(
                    (b["radius_lattice"] for b in bands if b["symmetric"]),
                    default=0,
                ),
            }
        )
    report["piers"] = pier_rows

    (OUT / f"mirror_input_{SCALE}.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1)
    )

    digest = {
        "window_count": report["window_count"],
        "windows": [
            {
                k: w[k]
                for k in (
                    "label",
                    "corner_vertices",
                    "u_left",
                    "u_right",
                    "width_lattice",
                    "height_lattice",
                    "width_exact",
                )
            }
            for w in windows
        ],
        "windows_congruent_after_snap": report["windows_congruent_after_snap"],
        "windows_congruent_before_snap": report["windows_congruent_before_snap"],
        "figure_is_globally_mirror_symmetric": report[
            "figure_is_globally_mirror_symmetric"
        ],
        "window_pairs": [
            {
                k: row[k]
                for k in (
                    "pair",
                    "doubled_axis",
                    "axis_gap_lattice_steps",
                    "window_maps_exactly",
                    "axis_gap_before_snap_lattice_steps",
                    "whole_figure_symmetric",
                    "whole_figure_unmatched",
                )
            }
            for row in pair_rows
        ],
        "piers": [
            {
                "pier": row["pier"],
                "doubled_axis": row["doubled_axis"],
                "axis_gap_lattice_steps": row["axis_gap_lattice_steps"],
                "two_windows_map_exactly": row["two_windows_map_exactly"],
                "symmetry_radius_lattice": row["symmetry_radius_lattice"],
                "bands": [
                    {
                        "radius": b["radius_lattice"],
                        "nodes": b["band_nodes"],
                        "symmetric": b["symmetric"],
                        "unmatched": len(b["unmatched"]),
                    }
                    for b in row["bands"]
                ],
            }
            for row in pier_rows
        ],
    }
    print(json.dumps(digest, ensure_ascii=False, indent=1))
    _ = payload


if __name__ == "__main__":
    main()
