"""FAN-MIRROR-DIAG, шаг 5: посев вееров, ничьи владения и контрфакт растворения.

Четыре вопроса:

S1 ПОСЕВ      опоры вееров зеркальной пары после отражения `(a, b) -> (-a, b)`.
S2 НИЧЬИ      `ambiguous_owner_spans` моста: где они, зеркальны ли, каким
              правилом ядро/хост выбирает владельца при ничьей.
S3 СИРОТА     точные координаты узла, у которого нет зеркального двойника,
              и вершина маршрута, породившая его.
S4 КОНТРФАКТ  та же фигура БЕЗ прямых T-вершин маршрута (коллинеарные рёбра
              слиты в одно): становится ли пара зеркальной.
"""

from __future__ import annotations

import json
from collections import Counter
from fractions import Fraction
from pathlib import Path
import sys

from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.wavefront.bridge import bridge_arrival_laws
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.polygon import signed_double_area

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step3_queue import (  # noqa: E402
    GRID,
    OUT,
    SCALE,
    build_input,
    load_chart,
    primitive,
    sq,
)
from step4_probes import (  # noqa: E402
    approx,
    lattice_fans,
    node_signature,
    pipeline,
)


def collinear_indices(points):
    size = len(points)
    return [
        index
        for index in range(size)
        if (points[index][0] - points[(index - 1) % size][0])
        * (points[(index + 1) % size][1] - points[index][1])
        - (points[index][1] - points[(index - 1) % size][1])
        * (points[(index + 1) % size][0] - points[index][0])
        == 0
    ]


def main() -> None:
    payload, order, exact_points, lattice_points = load_chart()
    (
        loop,
        lattice_loop,
        labels,
        laws,
        fan_laws,
        reflex,
        _pre,
    ) = build_input(exact_points, tuple(order), lattice_points)
    base_points = tuple((int(x), int(y)) for x, y in lattice_loop)
    windows = json.loads((OUT / f"mirror_input_{SCALE}.json").read_text())[
        "windows"
    ]
    windows.sort(key=lambda w: w["u_right"], reverse=True)

    result = {"schema": "cftuv.fan_diag.seeds_owners.v1", "scale": SCALE}

    # ---- S1: посев вееров зеркальной пары -----------------------------------
    polygon, skeleton, partition = pipeline(base_points)
    fans = {fan.point: fan for fan in polygon.vertex_fans}
    seed_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        doubled = right_window["u_left"] + left_window["u_right"]
        first = (right_window["u_left"], right_window["v_top"])
        second = (left_window["u_right"], left_window["v_top"])
        fan_a, fan_b = fans.get(first), fans.get(second)
        image = (
            tuple(
                (-support.normal_x, support.normal_y, support.speed_squared)
                for support in fan_a.supports
            )
            if fan_a
            else None
        )
        actual = (
            tuple(
                (support.normal_x, support.normal_y, support.speed_squared)
                for support in fan_b.supports
            )
            if fan_b
            else None
        )
        seed_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "doubled_axis": doubled,
                "anchor_right_window": list(first),
                "anchor_left_window": list(second),
                "anchor_mirror_exact": doubled - first[0] == second[0]
                and first[1] == second[1],
                "anchor_gap_lattice_steps": (doubled - first[0]) - second[0],
                "supports_right": [list(map(str, s)) for s in (image or ())],
                "supports_left": [list(map(str, s)) for s in (actual or ())],
                "supports_mirror_equal": image == actual,
                "hidden_edge_count_right": len(fan_a.supports) if fan_a else 0,
                "hidden_edge_count_left": len(fan_b.supports) if fan_b else 0,
            }
        )
    result["S1_fan_seeds"] = {
        "fan_count": len(polygon.vertex_fans),
        "hidden_edges_per_fan": sorted(
            {len(fan.supports) for fan in polygon.vertex_fans}
        ),
        "all_pairs_mirror_equal": all(
            row["supports_mirror_equal"] for row in seed_rows
        ),
        "all_anchors_mirror_exact": all(
            row["anchor_mirror_exact"] for row in seed_rows
        ),
        "rows": seed_rows,
    }

    # ---- S2: ничьи владения -------------------------------------------------
    direct = bridge_arrival_laws(
        (tuple(loop),), laws, lattice=GRID, vertex_fans=fan_laws
    )
    mirror_axis = 0
    mirrored_loop = tuple((mirror_axis - x, y) for x, y in loop)
    mirrored_labels = tuple(labels)
    (
        m_loop,
        _m_lattice,
        m_labels,
        m_laws,
        m_fan_laws,
        _m_reflex,
        _m_pre,
    ) = build_input(
        list(mirrored_loop),
        mirrored_labels,
        [(mirror_axis - x, y) for x, y in lattice_loop],
    )
    mirrored = bridge_arrival_laws(
        (tuple(m_loop),), m_laws, lattice=GRID, vertex_fans=m_fan_laws
    )

    def span_set(report):
        return {tuple(span) for span in report.ambiguous_owner_spans}

    def mirror_span(span):
        x0, y0, x1, y1 = span
        return tuple(sorted(((mirror_axis - x0, y0), (mirror_axis - x1, y1))))

    direct_spans = {mirror_span(span) for span in span_set(direct)}
    mirrored_spans = {
        tuple(sorted(((x0, y0), (x1, y1))))
        for x0, y0, x1, y1 in span_set(mirrored)
    }
    line_groups = Counter()
    for x0, y0, x1, y1 in span_set(direct):
        a, b = y0 - y1, x1 - x0
        scale = abs(a) if a else abs(b)
        line_groups[(a // scale, b // scale, (a * x0 + b * y0) // scale)] += 1
    result["S2_ownership_ties"] = {
        "ambiguous_span_count_direct": len(span_set(direct)),
        "ambiguous_span_count_mirrored": len(span_set(mirrored)),
        "forced_owner_count_direct": len(direct.owner_by_edge),
        "ambiguous_set_is_mirror_covariant": direct_spans == mirrored_spans,
        "ambiguous_spans_by_support_line": [
            {"line_class": list(key), "edges": value}
            for key, value in sorted(line_groups.items())
        ],
        "tie_break_rule": {
            "kernel": (
                "НЕТ правила: cftuv_envelope/wavefront/bridge.py::"
                "_match_by_line_class перечисляет пролёты в ambiguous и НЕ "
                "кладёт их в forced_owner; wavefront/conveyor.py::"
                "_region_coverage делает owner_names.get(face.owner, ''), "
                "то есть у грани ничьей envelope_spec_id ПУСТОЙ"
            ),
            "host": (
                "cftuv/envelope_queue_export.py::merge_same_chain_faces: "
                "primary = min(block, key=lambda item: item.owner) — "
                "лексикографический минимум решёточного пролёта (x0,y0,x1,y1); "
                "палитра слоёв берётся у primary "
                "(_palette/slot_of), пустой spec_id даёт слот 0"
            ),
            "mirror_covariance_of_host_rule": (
                "НЕТ: min по (x0,y0,...) при отражении u -> S-u меняет порядок "
                "на обратный, поэтому primary слитого блока перескакивает на "
                "противоположный конец блока"
            ),
        },
        "ambiguous_spans": [list(span) for span in sorted(span_set(direct))],
    }

    # ---- S3: узел-сирота ----------------------------------------------------
    straight = collinear_indices(base_points)
    orphan_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        doubled = right_window["u_left"] + left_window["u_right"]
        centre = doubled / 2
        radius = right_window["width_lattice"]
        band = [
            node
            for node in skeleton.nodes
            if abs(approx(node.point.x) - centre) <= radius
        ]
        present = Counter(node_signature(node) for node in band)
        for node in band:
            if node_signature(node, doubled) in present:
                continue
            x_terms, y_terms = sq(node.point.x), sq(node.point.y)
            orphan_rows.append(
                {
                    "pier": f"{right_window['label']}|{left_window['label']}",
                    "kind": node.kind.value,
                    "x_exact": x_terms,
                    "y_exact": y_terms,
                    "x_lattice": approx(node.point.x),
                    "y_lattice": approx(node.point.y),
                    "x_metres": approx(node.point.x) / SCALE,
                    "y_metres": approx(node.point.y) / SCALE,
                    "time": [
                        [
                            node.time.canonical().dividend.numerator,
                            node.time.canonical().dividend.denominator,
                        ],
                        sq(node.time.canonical().divisor),
                    ],
                    "converging_vertices": node.converging_vertices,
                    "distance_from_pier_axis_lattice": approx(node.point.x)
                    - centre,
                    "distance_from_pier_axis_metres": (
                        approx(node.point.x) - centre
                    )
                    / SCALE,
                }
            )
    result["S3_orphan_nodes"] = {
        "collinear_route_vertices": [
            {
                "loop_index": index,
                "point": list(base_points[index]),
                "point_metres": [
                    base_points[index][0] / SCALE,
                    base_points[index][1] / SCALE,
                ],
            }
            for index in straight
        ],
        "collinear_route_vertex_count": len(straight),
        "rows": orphan_rows,
    }

    # ---- S4: контрфакт растворения -----------------------------------------
    dissolved = tuple(
        point
        for index, point in enumerate(base_points)
        if index not in set(straight)
    )
    d_polygon, d_skeleton, d_partition = pipeline(dissolved)
    d_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        doubled = right_window["u_left"] + left_window["u_right"]
        centre = doubled / 2
        radius = right_window["width_lattice"]
        band = [
            node
            for node in d_skeleton.nodes
            if abs(approx(node.point.x) - centre) <= radius
        ]
        present = Counter(node_signature(node) for node in band)
        imaged = Counter(node_signature(node, doubled) for node in band)
        d_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "nodes_in_band": len(band),
                "band_is_mirror_symmetric": present == imaged,
                "orphans": sum(
                    1
                    for node in band
                    if node_signature(node, doubled) not in present
                ),
            }
        )
    result["S4_dissolved_counterfactual"] = {
        "vertex_count": len(dissolved),
        "outcome": d_skeleton.outcome.value,
        "node_count": len(d_skeleton.nodes),
        "face_count": len(d_partition.faces),
        "fan_count": d_polygon.fan_edge_count,
        "semantic_digest": semantic_digest(d_skeleton),
        "signed_double_area_unchanged": signed_double_area(
            d_polygon.outer.points
        )
        == signed_double_area(polygon.outer.points),
        "piers": d_rows,
        "all_piers_mirror_symmetric": all(
            row["band_is_mirror_symmetric"] for row in d_rows
        ),
    }

    (OUT / "seeds_owners.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=1)
    )
    print(
        json.dumps(
            {
                "S1": {
                    k: result["S1_fan_seeds"][k]
                    for k in (
                        "fan_count",
                        "hidden_edges_per_fan",
                        "all_pairs_mirror_equal",
                        "all_anchors_mirror_exact",
                    )
                },
                "S1_rows": [
                    {
                        k: row[k]
                        for k in (
                            "pier",
                            "anchor_mirror_exact",
                            "anchor_gap_lattice_steps",
                            "supports_right",
                            "supports_left",
                            "supports_mirror_equal",
                        )
                    }
                    for row in seed_rows
                ],
                "S2": {
                    k: result["S2_ownership_ties"][k]
                    for k in (
                        "ambiguous_span_count_direct",
                        "ambiguous_span_count_mirrored",
                        "forced_owner_count_direct",
                        "ambiguous_set_is_mirror_covariant",
                        "ambiguous_spans_by_support_line",
                    )
                },
                "S3": {
                    "collinear_route_vertex_count": result["S3_orphan_nodes"][
                        "collinear_route_vertex_count"
                    ],
                    "collinear_route_vertices": result["S3_orphan_nodes"][
                        "collinear_route_vertices"
                    ],
                    "rows": [
                        {
                            k: row[k]
                            for k in (
                                "pier",
                                "kind",
                                "x_metres",
                                "y_metres",
                                "distance_from_pier_axis_metres",
                                "converging_vertices",
                            )
                        }
                        for row in orphan_rows
                    ],
                },
                "S4": {
                    k: result["S4_dissolved_counterfactual"][k]
                    for k in (
                        "vertex_count",
                        "outcome",
                        "node_count",
                        "face_count",
                        "fan_count",
                        "signed_double_area_unchanged",
                        "all_piers_mirror_symmetric",
                        "piers",
                    )
                },
            },
            ensure_ascii=False,
            indent=1,
        )
    )
    _ = payload, order, reflex, partition


if __name__ == "__main__":
    main()
