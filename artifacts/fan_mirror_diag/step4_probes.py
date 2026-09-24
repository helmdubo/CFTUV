"""FAN-MIRROR-DIAG, шаг 4: кто именно ломает зеркальность — вход или очередь.

Четыре пробы на ОДНОЙ и той же полевой фигуре:

P1 ПОРЯДОК   петля прокручена стартовой вершиной (`LoopV1.rotated`). Геометрия
             та же побитово, меняется только порядок посева. Расхождение узлов
             здесь — чистая зависимость ответа от порядка обработки.
P2 ЗЕРКАЛО   вся фигура отражена `u -> S - u`. Точное ядро обязано дать
             отражённый ответ; расхождение — неэквивариантность.
P3 ПАРА      узлы скелета в полосе вокруг оси простенка, сверенные отражением.
P4 ВРЕМЯ     группы событий одного ТОЧНОГО времени (одновременные группы).
"""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from fractions import Fraction
from pathlib import Path

from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.wavefront.bridge import (
    PlainArrivalLawV1,
    VertexFanLawV1,
    bridge_arrival_laws,
)
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.polygon import (
    FanSupportV1,
    LoopV1,
    PolygonV1,
    VertexFanV1,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step3_queue import (  # noqa: E402
    GRID,
    LATTICE_ALPHA,
    OUT,
    SCALE,
    build_input,
    ccw_loop,
    load_chart,
    primitive,
    sq,
)


def lattice_fans(points):
    """Вееры `k = 1` во всех вогнутых вершинах решёточной петли (корпусной рецепт)."""

    size = len(points)
    fans = []
    for index in range(size):
        before, here, after = (
            points[(index - 1) % size],
            points[index],
            points[(index + 1) % size],
        )
        if (here[0] - before[0]) * (after[1] - here[1]) - (
            here[1] - before[1]
        ) * (after[0] - here[0]) >= 0:
            continue
        incoming = primitive((-(here[1] - before[1]), here[0] - before[0]))
        outgoing = primitive((-(after[1] - here[1]), after[0] - here[0]))
        normal = (incoming[0] + outgoing[0], incoming[1] + outgoing[1])
        fans.append(
            VertexFanV1(
                here,
                (
                    FanSupportV1(
                        normal[0], normal[1], normal[0] ** 2 + normal[1] ** 2
                    ),
                ),
            )
        )
    return tuple(fans)


def pipeline(points):
    """Полигон -> скелет -> грани на ЦЕЛОЙ петле. Один рецепт на все пробы.

    Вееры считаются по УЖЕ НОРМИРОВАННОЙ петле полигона, а не по поданному
    порядку: `PolygonV1.build` разворачивает CW-петлю, и вогнутость, снятая до
    разворота, указала бы на выпуклые вершины. Отражение как раз меняет знак
    площади, поэтому ошибка проявилась бы только на зеркальной колонке.
    """

    built = PolygonV1.build(tuple(points))
    polygon = with_vertex_fans(built, lattice_fans(built.outer.points))
    skeleton = build_skeleton(polygon)
    partition = build_faces(polygon, skeleton)
    return polygon, skeleton, partition


def node_signature(node, doubled_axis=None):
    record = node_record(node)
    x = node.point.x
    if doubled_axis is not None:
        x = SqrtSumV1.rational(Fraction(doubled_axis)) - x
    return json.dumps(
        [
            record["time_dividend"],
            record["time_divisor"],
            sq(x),
            record["point_y"],
            record["kind"],
        ],
        sort_keys=True,
    )


def approx(value: SqrtSumV1) -> float:
    return sum(float(coefficient) * radicand ** 0.5 for radicand, coefficient in value.terms)


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

    polygon, skeleton, partition = pipeline(base_points)
    base_nodes = Counter(node_signature(node) for node in skeleton.nodes)
    base_digest = semantic_digest(skeleton)

    result = {
        "schema": "cftuv.fan_diag.probes.v1",
        "scale": SCALE,
        "base": {
            "node_count": len(skeleton.nodes),
            "face_count": len(partition.faces),
            "semantic_digest": base_digest,
            "outcome": skeleton.outcome.value,
        },
    }

    # ---- P1: порядок посева -------------------------------------------------
    rotations = []
    for shift in range(1, len(base_points)):
        rotated = base_points[shift:] + base_points[:shift]
        _, rotated_skeleton, rotated_partition = pipeline(rotated)
        nodes = Counter(node_signature(node) for node in rotated_skeleton.nodes)
        rotations.append(
            {
                "shift": shift,
                "outcome": rotated_skeleton.outcome.value,
                "node_count": len(rotated_skeleton.nodes),
                "face_count": len(rotated_partition.faces),
                "digest": semantic_digest(rotated_skeleton),
                "digest_matches_base": semantic_digest(rotated_skeleton)
                == base_digest,
                "nodes_match_base": nodes == base_nodes,
                "only_in_base": sorted(
                    (base_nodes - nodes).elements()
                )[:4],
                "only_in_rotated": sorted((nodes - base_nodes).elements())[:4],
            }
        )
    result["P1_start_vertex_rotation"] = {
        "rotations_tried": len(rotations),
        "digest_stable": all(row["digest_matches_base"] for row in rotations),
        "node_sets_stable": all(row["nodes_match_base"] for row in rotations),
        "diverging_shifts": [
            row["shift"] for row in rotations if not row["nodes_match_base"]
        ],
        "rows": [
            {
                k: row[k]
                for k in (
                    "shift",
                    "outcome",
                    "node_count",
                    "face_count",
                    "digest_matches_base",
                    "nodes_match_base",
                )
            }
            for row in rotations
        ],
    }

    # ---- P2: зеркало всей фигуры -------------------------------------------
    doubled_axis = 0
    mirrored_points = tuple((doubled_axis - x, y) for x, y in base_points)
    _, mirror_skeleton, mirror_partition = pipeline(mirrored_points)
    mirror_nodes = Counter(
        node_signature(node, doubled_axis) for node in mirror_skeleton.nodes
    )
    result["P2_whole_figure_mirror"] = {
        "doubled_axis": doubled_axis,
        "outcome": mirror_skeleton.outcome.value,
        "node_count": len(mirror_skeleton.nodes),
        "face_count": len(mirror_partition.faces),
        "nodes_match_reflected_base": mirror_nodes == base_nodes,
        "only_in_base": sorted((base_nodes - mirror_nodes).elements()),
        "only_in_mirror": sorted((mirror_nodes - base_nodes).elements()),
    }

    # ---- P3: локальная пара через простенок ---------------------------------
    windows = json.loads(
        (OUT / f"mirror_input_{SCALE}.json").read_text()
    )["windows"]
    windows.sort(key=lambda w: w["u_right"], reverse=True)
    node_points = [
        (approx(node.point.x), approx(node.point.y), node) for node in skeleton.nodes
    ]
    pier_rows = []
    for right_window, left_window in zip(windows, windows[1:]):
        pier_axis_doubled = right_window["u_left"] + left_window["u_right"]
        centre = pier_axis_doubled / 2
        radius = right_window["width_lattice"]
        band = [
            item for item in node_points if abs(item[0] - centre) <= radius
        ]
        direct = Counter(node_signature(item[2]) for item in band)
        imaged = Counter(
            node_signature(item[2], pier_axis_doubled) for item in band
        )
        pier_rows.append(
            {
                "pier": f"{right_window['label']}|{left_window['label']}",
                "doubled_axis": pier_axis_doubled,
                "radius_lattice": radius,
                "nodes_in_band": len(band),
                "band_is_mirror_symmetric": direct == imaged,
                "nodes_without_mirror_partner": [
                    {
                        "x": round(item[0], 3),
                        "y": round(item[1], 3),
                        "x_metres": round(item[0] / SCALE, 6),
                        "y_metres": round(item[1] / SCALE, 6),
                        "kind": item[2].kind.value,
                    }
                    for item in band
                    if node_signature(item[2], pier_axis_doubled) not in direct
                ],
            }
        )
    result["P3_pier_pair"] = pier_rows

    # ---- P4: группы одновременных событий -----------------------------------
    groups = defaultdict(list)
    for node in skeleton.nodes:
        record = node_record(node)
        groups[
            json.dumps([record["time_dividend"], record["time_divisor"]])
        ].append(node)
    simultaneous = []
    for key, members in groups.items():
        if len(members) < 2:
            continue
        time = json.loads(key)
        simultaneous.append(
            {
                "time_dividend": time[0],
                "time_divisor": time[1],
                "time_float": round(
                    (time[0][0] / time[0][1])
                    / sum(
                        (c[0] / c[1]) * radicand ** 0.5
                        for radicand, c in time[1]
                    ),
                    6,
                ),
                "member_count": len(members),
                "members": [
                    {
                        "kind": node.kind.value,
                        "x_lattice": round(approx(node.point.x), 3),
                        "y_lattice": round(approx(node.point.y), 3),
                        "converging_vertices": node.converging_vertices,
                    }
                    for node in members
                ],
            }
        )
    simultaneous.sort(key=lambda row: -row["member_count"])
    result["P4_simultaneous_groups"] = {
        "group_count": len(simultaneous),
        "largest_group": simultaneous[0]["member_count"] if simultaneous else 0,
        "nodes_in_simultaneous_groups": sum(
            row["member_count"] for row in simultaneous
        ),
        "groups": simultaneous,
    }

    (OUT / "probes.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=1)
    )
    print(
        json.dumps(
            {
                "base": result["base"],
                "P1": {
                    k: result["P1_start_vertex_rotation"][k]
                    for k in (
                        "rotations_tried",
                        "digest_stable",
                        "node_sets_stable",
                        "diverging_shifts",
                    )
                },
                "P2": {
                    k: result["P2_whole_figure_mirror"][k]
                    for k in (
                        "outcome",
                        "node_count",
                        "face_count",
                        "nodes_match_reflected_base",
                    )
                },
                "P3": [
                    {
                        "pier": row["pier"],
                        "nodes_in_band": row["nodes_in_band"],
                        "band_is_mirror_symmetric": row[
                            "band_is_mirror_symmetric"
                        ],
                        "orphans": len(row["nodes_without_mirror_partner"]),
                        "orphan_points_metres": [
                            [item["x_metres"], item["y_metres"], item["kind"]]
                            for item in row["nodes_without_mirror_partner"]
                        ],
                    }
                    for row in pier_rows
                ],
                "P4": {
                    k: result["P4_simultaneous_groups"][k]
                    for k in (
                        "group_count",
                        "largest_group",
                        "nodes_in_simultaneous_groups",
                    )
                },
            },
            ensure_ascii=False,
            indent=1,
        )
    )
    _ = payload, laws, fan_laws, reflex, labels, polygon, LATTICE_ALPHA, GRID


if __name__ == "__main__":
    main()
