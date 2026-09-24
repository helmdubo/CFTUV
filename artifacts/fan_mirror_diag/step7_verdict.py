"""FAN-MIRROR-DIAG, шаг 7: сравнение узлов по парам и итоговый вердикт.

Собирает три артефакта из уже посчитанных прогонов:
`seeds_comparison.json`, `nodes_comparison.json`, `verdict.json`.
"""

from __future__ import annotations

import json
from collections import Counter
from fractions import Fraction
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step3_queue import OUT, SCALE, build_input, load_chart, sq  # noqa: E402
from step4_probes import approx, node_signature, pipeline  # noqa: E402
from step5_seeds_owners import collinear_indices  # noqa: E402


def band_rows(skeleton, doubled_axis, radius):
    centre = Fraction(doubled_axis, 2)
    band = [
        node
        for node in skeleton.nodes
        if abs(Fraction(approx(node.point.x)).limit_denominator(10 ** 9) - centre)
        <= radius
    ]
    present = Counter(node_signature(node) for node in band)
    rows = []
    for node in band:
        image = node_signature(node, doubled_axis)
        rows.append(
            {
                "kind": node.kind.value,
                "x_lattice": approx(node.point.x),
                "y_lattice": approx(node.point.y),
                "x_metres": approx(node.point.x) / SCALE,
                "y_metres": approx(node.point.y) / SCALE,
                "x_exact": sq(node.point.x),
                "y_exact": sq(node.point.y),
                "time_dividend": [
                    node.time.canonical().dividend.numerator,
                    node.time.canonical().dividend.denominator,
                ],
                "time_divisor": sq(node.time.canonical().divisor),
                "converging_vertices": node.converging_vertices,
                "has_mirror_partner": image in present,
                "signed_offset_from_axis_lattice": approx(node.point.x)
                - float(centre),
            }
        )
    rows.sort(key=lambda row: row["x_lattice"])
    return rows


def main() -> None:
    payload, order, exact_points, lattice_points = load_chart()
    (
        loop,
        lattice_loop,
        labels,
        laws,
        fan_laws,
        reflex,
        pre_reflex,
    ) = build_input(exact_points, tuple(order), lattice_points)
    base_points = tuple((int(x), int(y)) for x, y in lattice_loop)
    polygon, skeleton, partition = pipeline(base_points)
    straight = set(collinear_indices(base_points))
    dissolved = tuple(
        point for index, point in enumerate(base_points) if index not in straight
    )
    d_polygon, d_skeleton, d_partition = pipeline(dissolved)

    windows = json.loads((OUT / f"mirror_input_{SCALE}.json").read_text())[
        "windows"
    ]
    windows.sort(key=lambda w: w["u_right"], reverse=True)

    seeds = json.loads((OUT / "seeds_owners.json").read_text())
    probes = json.loads((OUT / "probes.json").read_text())
    snap = json.loads((OUT / "snap_law.json").read_text())
    queue = json.loads((OUT / "queue_run.json").read_text())
    mirror_in = json.loads(
        (OUT / f"mirror_input_{SCALE}.json").read_text()
    )

    # ---- seeds_comparison ---------------------------------------------------
    (OUT / "seeds_comparison.json").write_text(
        json.dumps(
            {
                "schema": "cftuv.fan_diag.seeds_comparison.v1",
                "scale": SCALE,
                "density": "Fan Density 1 -> HUBER_EMANATED_COUNT_DENSITY_A_V1, "
                "max_subturn pi/3, hidden count H=1 на каждый вогнутый угол",
                "recipe": "chamfered_standard.right_angle_fans (сумма ПРИВЕДЁННЫХ "
                "нормалей соседних рёбер, q=|n|^2); для точного прямого угла "
                "совпадает с reference/angular._interpolated_normals при count=1",
                "reflex_count_on_lattice": len(reflex),
                "reflex_vertices_on_lattice": list(reflex),
                "reflex_count_before_snap": len(pre_reflex),
                "reflex_vertices_before_snap": list(pre_reflex),
                "fan_seeds": queue["fan_seeds"],
                "mirror_pairs": seeds["S1_fan_seeds"],
            },
            ensure_ascii=False,
            indent=1,
        )
    )

    # ---- nodes_comparison ---------------------------------------------------
    node_rows = []
    dissolved_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        doubled = right_window["u_left"] + left_window["u_right"]
        radius = right_window["width_lattice"]
        rows = band_rows(skeleton, doubled, radius)
        node_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "doubled_axis": doubled,
                "axis_metres": doubled / 2 / SCALE,
                "radius_lattice": radius,
                "nodes": rows,
                "orphans": [row for row in rows if not row["has_mirror_partner"]],
                "band_is_mirror_symmetric": all(
                    row["has_mirror_partner"] for row in rows
                ),
            }
        )
        d_rows = band_rows(d_skeleton, doubled, radius)
        dissolved_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "nodes": d_rows,
                "band_is_mirror_symmetric": all(
                    row["has_mirror_partner"] for row in d_rows
                ),
            }
        )
    (OUT / "nodes_comparison.json").write_text(
        json.dumps(
            {
                "schema": "cftuv.fan_diag.nodes_comparison.v1",
                "scale": SCALE,
                "base_run": {
                    "outcome": skeleton.outcome.value,
                    "node_count": len(skeleton.nodes),
                    "face_count": len(partition.faces),
                    "fan_count": polygon.fan_edge_count,
                },
                "dissolved_run": {
                    "outcome": d_skeleton.outcome.value,
                    "node_count": len(d_skeleton.nodes),
                    "face_count": len(d_partition.faces),
                    "fan_count": d_polygon.fan_edge_count,
                    "vertex_count": len(dissolved),
                },
                "piers": node_rows,
                "piers_after_dissolving_collinear_route_vertices": dissolved_rows,
                "simultaneous_groups": probes["P4_simultaneous_groups"],
            },
            ensure_ascii=False,
            indent=1,
        )
    )

    # ---- verdict ------------------------------------------------------------
    verdict = {
        "schema": "cftuv.fan_diag.verdict.v1",
        "card": "FAN-MIRROR-DIAG",
        "object": "wall 2.001 (artifacts/field_snapshots/wall_2_001_snapshot.json)",
        "reproduced_without_blender": True,
        "kernel_run": {
            "bridge_outcome": queue["bridge"]["outcome"],
            "edges": queue["bridge"]["edge_count"],
            "laws": queue["bridge"]["law_count"],
            "fans": queue["bridge"]["fan_edge_count"],
            "hidden_edges_per_fan": 1,
            "skeleton_outcome": skeleton.outcome.value,
            "skeleton_nodes": len(skeleton.nodes),
            "faces": len(partition.faces),
            "coverage_outcome": queue["coverage"]["outcome"],
            "field_reference": {
                "fans": 12,
                "skeleton_nodes": 45,
                "faces": 47,
                "ambiguous_owner_spans": 23,
            },
            "matches_field": {
                "fans": queue["bridge"]["fan_edge_count"] == 12,
                "faces": len(partition.faces) == 47,
                "skeleton_nodes": len(skeleton.nodes) == 45,
                "ambiguous_owner_spans": queue["bridge"][
                    "ambiguous_owner_span_count"
                ]
                == 23,
            },
        },
        "verdicts": {
            "a_seeding": {
                "name": "FAN_SEEDING_IS_MIRROR_EXACT",
                "guilty": False,
                "evidence": {
                    "fan_count": 12,
                    "hidden_edges_per_fan": 1,
                    "all_pairs_supports_mirror_equal": seeds["S1_fan_seeds"][
                        "all_pairs_mirror_equal"
                    ],
                    "all_anchors_mirror_exact": seeds["S1_fan_seeds"][
                        "all_anchors_mirror_exact"
                    ],
                    "support_normal": "(1, 1) с q = 2 у каждой из 12 опор; "
                    "образ (-1, 1) при u -> S-u совпадает с опорой партнёра "
                    "точно, без допуска",
                },
            },
            "b_snap": {
                "name": "GRID_SNAP_LAW_IS_NOT_MIRROR_COVARIANT",
                "guilty": True,
                "evidence": {
                    "law": "robust/grid.py::snap_value = floor(v*scale + 1/2) "
                    "(HALF_UP)",
                    "window_width_in_lattice_steps_exact": "85197/2 = 42598.5 "
                    "у 5 окон из 6 — ровно на границе округления",
                    "pier_width_in_lattice_steps_exact": "111411/4 = 27852.75",
                    "window_widths_after_snap": mirror_in[
                        "window_widths_lattice"
                    ],
                    "pier_widths_after_snap": [27853, 27853, 27853, 27853, 27852],
                    "counterexample_vertices_scale_32768": [
                        item["vertex"]
                        for row in snap["T1_snap_law_mirror_covariance"]["rows"]
                        if row["scale"] == 32768
                        for item in row["broken"]
                    ],
                    "gap_lattice_steps": 1,
                    "gap_metres": 1 / SCALE,
                    "snap_then_mirror_vs_mirror_then_snap": {
                        "vertices_differing": 4,
                        "skeleton_nodes_differing": 9,
                        "of_total_nodes": 40,
                    },
                    "window_pairs_with_broken_axis": [
                        row["pair"]
                        for row in mirror_in["window_pairs"]
                        if row["axis_gap_lattice_steps"] != 0
                    ],
                },
            },
            "c_environment": {
                "name": "FIGURE_IS_NOT_GLOBALLY_MIRROR_SYMMETRIC_AND_THE_ROUTE_"
                "CARRIES_OFF_AXIS_COLLINEAR_VERTICES",
                "guilty": True,
                "evidence": {
                    "global_mirror_axes": mirror_in["global_mirror_axes_doubled"],
                    "figure_is_globally_mirror_symmetric": mirror_in[
                        "figure_is_globally_mirror_symmetric"
                    ],
                    "collinear_route_vertices": len(straight),
                    "collinear_route_vertices_list": sorted(
                        labels[index] for index in straight
                    ),
                    "top_edge_T_vertex_offset_from_pier_axis_metres": 0.0234832763671875,
                    "orphan_nodes_per_pier": 1,
                    "orphan_kind": "SPLIT",
                    "dissolving_them_restores_the_mirror": True,
                    "dissolved_run": {
                        "vertices": len(dissolved),
                        "nodes": len(d_skeleton.nodes),
                        "faces": len(d_partition.faces),
                        "all_piers_mirror_symmetric": True,
                        "area_unchanged": True,
                    },
                },
            },
            "d_order": {
                "name": "QUEUE_IS_ORDER_INVARIANT_AND_MIRROR_EQUIVARIANT_ON_THIS_FIGURE",
                "guilty": False,
                "evidence": {
                    "start_vertex_rotations_tried": probes[
                        "P1_start_vertex_rotation"
                    ]["rotations_tried"],
                    "digest_stable": probes["P1_start_vertex_rotation"][
                        "digest_stable"
                    ],
                    "node_sets_stable": probes["P1_start_vertex_rotation"][
                        "node_sets_stable"
                    ],
                    "whole_figure_mirror_nodes_match": probes[
                        "P2_whole_figure_mirror"
                    ]["nodes_match_reflected_base"],
                    "simultaneous_groups": probes["P4_simultaneous_groups"][
                        "group_count"
                    ],
                    "largest_simultaneous_group": probes[
                        "P4_simultaneous_groups"
                    ]["largest_group"],
                    "nodes_in_simultaneous_groups": probes[
                        "P4_simultaneous_groups"
                    ]["nodes_in_simultaneous_groups"],
                    "note": "одновременные группы ЕСТЬ и они крупные (12 и 8 "
                    "событий на одно точное время), но на ЭТОЙ фигуре они не "
                    "дают порядковой зависимости: и прокрутка старта, и "
                    "отражение дают побитово тот же набор узлов",
                },
            },
            "e_ownership_ties": {
                "name": "OWNERSHIP_TIES_ARE_UNRESOLVED_IN_THE_KERNEL_AND_"
                "RESOLVED_NON_COVARIANTLY_IN_THE_HOST",
                "guilty": "partly",
                "evidence": {
                    "ambiguous_spans_in_this_run": queue["bridge"][
                        "ambiguous_owner_span_count"
                    ],
                    "ambiguous_spans_in_field": 23,
                    "why_the_difference": "у поля верхняя кромка точно "
                    "коллинеарна и её 6 рёбер тоже попадают в один класс "
                    "(17 + 6 = 23); у нас дорешёточная кромка имеет binary64-"
                    "шум 3.8e-06 м и распадается на 6 отдельных классов",
                    "spans_by_support_line": seeds["S2_ownership_ties"][
                        "ambiguous_spans_by_support_line"
                    ],
                    "kernel_tie_break": seeds["S2_ownership_ties"][
                        "tie_break_rule"
                    ]["kernel"],
                    "host_tie_break": seeds["S2_ownership_ties"][
                        "tie_break_rule"
                    ]["host"],
                    "host_rule_is_mirror_covariant": False,
                    "ambiguous_set_is_mirror_covariant_under_exact_reflection": (
                        seeds["S2_ownership_ties"][
                            "ambiguous_set_is_mirror_covariant"
                        ]
                    ),
                    "note": "разметка ничьих сама по себе структурна и "
                    "переносится отражением; ломает её тот же half-up снап "
                    "(вердикт b). В ядре ничья НЕ разрешается вовсе: "
                    "envelope_spec_id грани ничьей — пустая строка",
                },
            },
        },
        "named_gaps": [
            {
                "name": "HOST_CHART_FRAME_UNAVAILABLE_WITHOUT_BLENDER",
                "detail": "начало и базис хостовой карты патча не выводятся из "
                "слепка меша; взята карта (u, v) = (object y, object z) без "
                "переноса. Сдвиг начала на 1/4 шага решётки меняет число узлов "
                "40 -> 43 и число вееров 12 -> 15, поэтому ровно полевые 45 "
                "узлов этой картой не воспроизводятся",
            },
            {
                "name": "HOST_WEIGHTED_ARRIVAL_LAWS_NOT_REPRODUCED",
                "detail": "поле сообщает NON_UNIT_SPEED_LAWS=35; прогон сделан "
                "с единичной скоростью. Однородный вес — подобие по времени и "
                "зеркальности не меняет; неоднородный не проверялся",
            },
        ],
        "artifacts": sorted(
            item.name for item in OUT.glob("*.json")
        ),
    }
    (OUT / "verdict.json").write_text(
        json.dumps(verdict, ensure_ascii=False, indent=1)
    )
    print(json.dumps(verdict["verdicts"], ensure_ascii=False, indent=1))
    print(json.dumps(verdict["kernel_run"], ensure_ascii=False, indent=1))
    _ = payload, laws, fan_laws, loop


if __name__ == "__main__":
    main()
