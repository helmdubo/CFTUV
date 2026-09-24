"""FAN-MIRROR-DIAG, шаг 3: прогон очереди ядра на полевом входе стены 2.001.

Вход собирается ТЕМ ЖЕ фасадом, что и полевой прогон:

    точные дроби домена + PlainArrivalLawV1 на каждое ребро + VertexFanLawV1
    на каждый вогнутый угол  ->  bridge_arrival_laws(lattice=GridSpecV1(32768))
    ->  build_skeleton  ->  build_faces  ->  coverage_at(lattice_alpha)

Рецепт веера — корпусной `chamfered_standard.right_angle_fans` (`k = 1`,
направление = сумма ПРИВЕДЁННЫХ нормалей соседних рёбер, `q = |n|^2`). Для
точного прямого угла он совпадает с `reference/angular._interpolated_normals`
при `count = 1`: там скрытая опора есть `unit_g(incoming + outgoing)`, то есть
та же биссектриса, и `test_the_conveyor_builds_the_fan_with_the_reference_recipe`
эту связь уже держит.
"""

from __future__ import annotations

import json
from fractions import Fraction
from math import gcd
from pathlib import Path
import sys

from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.wavefront.bridge import (
    BridgeOutcome,
    PlainArrivalLawV1,
    VertexFanLawV1,
    bridge_arrival_laws,
)
from cftuv_envelope.wavefront.conveyor import computation_loop_supports
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.polygon import PolygonV1, signed_double_area
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

OUT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/fan_diag")
SCALE = 32768
GRID = GridSpecV1(scale=SCALE)
# Полевой sidecar стены 2.001.
ALPHA_METRES = Fraction(2540000081062317, 10000000000000000)
LATTICE_ALPHA = Fraction(2540000081062317, 305175781250)


def load_chart():
    payload = json.loads(
        (OUT / f"fixture_wall_2_001_lattice_{SCALE}.json").read_text()
    )
    order = payload["loop_order"]
    exact = [
        (Fraction(item["u"]), Fraction(item["v"]))
        for item in payload["vertices_chart_exact"]
    ]
    lattice = [
        (Fraction(item["u"]), Fraction(item["v"]))
        for item in payload["vertices_snapped"]
    ]
    return payload, order, exact, lattice


def primitive(vector):
    """Приведённый целый вектор. Дробные компоненты сначала обезнаменателены."""

    x, y = Fraction(vector[0]), Fraction(vector[1])
    denominator = x.denominator * y.denominator // gcd(
        x.denominator, y.denominator
    )
    ix, iy = int(x * denominator), int(y * denominator)
    common = gcd(abs(ix), abs(iy))
    return (ix // common, iy // common)


def ccw_loop(points):
    """Петля в порядке CCW. Ровно та нормировка, что делает `PolygonV1.build`."""

    doubled = Fraction(0)
    size = len(points)
    for index in range(size):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % size]
        doubled += x0 * y1 - x1 * y0
    return (points if doubled > 0 else tuple(reversed(points))), doubled


def build_input(exact_points, labels, lattice_points):
    """Законы прихода на каждое ребро и вееры `k = 1` на каждый вогнутый угол.

    ВОГНУТОСТЬ РЕШАЕТСЯ НА РЕШЁТКЕ, а не на дорешёточных дробях, и это
    названный выбор `FAN_ANCHORS_ARE_THE_LATTICE_REFLEX_VERTICES_V1`. Причина
    измерена: на дорешёточных координатах binary64-шум верхней кромки делает
    ТРИ прямые T-вершины маршрута (v1, v12, v13) слабо вогнутыми, и веер сел бы
    на них — 15 вееров вместо полевых 12. На решётке те же вершины дают ровно
    нулевой поворот и вееров не получают.
    """

    loop, doubled = ccw_loop(tuple(exact_points))
    lattice_loop, lattice_doubled = ccw_loop(tuple(lattice_points))
    assert (doubled > 0) == (lattice_doubled > 0)
    if doubled <= 0:
        labels = tuple(reversed(labels))
    size = len(loop)
    laws = []
    for index in range(size):
        start = loop[index]
        end = loop[(index + 1) % size]
        nx, ny = primitive((-(end[1] - start[1]), end[0] - start[0]))
        laws.append(
            PlainArrivalLawV1(
                name=f"law:{labels[index]}->{labels[(index + 1) % size]}",
                normal_x=Fraction(nx),
                normal_y=Fraction(ny),
                constant=Fraction(nx) * start[0] + Fraction(ny) * start[1],
                speed_squared=Fraction(nx * nx + ny * ny),
            )
        )
    fans = []
    reflex = []
    pre_lattice_reflex = []
    for index in range(size):
        for source, sink in ((loop, pre_lattice_reflex), (lattice_loop, reflex)):
            before = source[(index - 1) % size]
            here = source[index]
            after = source[(index + 1) % size]
            if (here[0] - before[0]) * (after[1] - here[1]) - (
                here[1] - before[1]
            ) * (after[0] - here[0]) < 0:
                sink.append(labels[index])
        if labels[index] not in reflex:
            continue
        before = lattice_loop[(index - 1) % size]
        here = lattice_loop[index]
        after = lattice_loop[(index + 1) % size]
        incoming = primitive((-(here[1] - before[1]), here[0] - before[0]))
        outgoing = primitive((-(after[1] - here[1]), after[0] - here[0]))
        here = loop[index]
        normal = (incoming[0] + outgoing[0], incoming[1] + outgoing[1])
        fans.append(
            VertexFanLawV1(
                name=f"angular-spec:v{labels[index]}",
                point=(here[0], here[1]),
                supports=computation_loop_supports(
                    (
                        (
                            normal[0],
                            normal[1],
                            Fraction(normal[0] ** 2 + normal[1] ** 2),
                        ),
                    ),
                    1,
                ),
            )
        )
    return (
        loop,
        lattice_loop,
        labels,
        tuple(laws),
        tuple(fans),
        tuple(reflex),
        tuple(pre_lattice_reflex),
    )


def sq(value: SqrtSumV1):
    return [[radicand, [c.numerator, c.denominator]] for radicand, c in value.terms]


def node_key(node, mirror_doubled_axis=None):
    """Каноническая запись узла; при отражении — с отзеркаленной абсциссой."""

    record = node_record(node)
    x = node.point.x
    if mirror_doubled_axis is not None:
        x = SqrtSumV1.rational(Fraction(mirror_doubled_axis)) - x
    return json.dumps(
        [
            record["time_dividend"],
            record["time_divisor"],
            sq(x),
            record["point_y"],
        ],
        sort_keys=True,
    )


def run(loop_points, tag):
    loops = (tuple((Fraction(x), Fraction(y)) for x, y in loop_points),)
    return loops


def main() -> None:
    payload, order, exact_points, lattice_points = load_chart()
    (
        loop,
        lattice_loop,
        labels,
        laws,
        fans,
        reflex,
        pre_lattice_reflex,
    ) = build_input(exact_points, tuple(order), lattice_points)

    report = bridge_arrival_laws(
        (tuple(loop),),
        laws,
        lattice=GRID,
        vertex_fans=fans,
    )
    result = {
        "schema": "cftuv.fan_diag.queue_run.v1",
        "scale": SCALE,
        "alpha_metres": str(ALPHA_METRES),
        "lattice_alpha": str(LATTICE_ALPHA),
        "bridge": {
            "outcome": report.outcome.value,
            "findings": [item.value for item in report.findings],
            "edge_count": report.edge_count,
            "law_count": report.law_count,
            "matched_edge_count": report.matched_edge_count,
            "wall_edge_count": report.wall_edge_count,
            "fan_edge_count": report.fan_edge_count,
            "ambiguous_owner_span_count": len(report.ambiguous_owner_spans),
            "owner_by_edge_count": len(report.owner_by_edge),
            "snap_residual": str(report.snap_residual),
            "snap_residual_lattice_steps": float(report.snap_residual * SCALE),
            "lattice_scale": report.lattice_scale,
            "weighted_edge_count": report.weighted_edge_count,
        },
        "reflex_vertices": list(reflex),
        "reflex_count": len(reflex),
        "reflex_vertices_before_snap": list(pre_lattice_reflex),
        "reflex_count_before_snap": len(pre_lattice_reflex),
        "reflex_law": "FAN_ANCHORS_ARE_THE_LATTICE_REFLEX_VERTICES_V1",
        "fan_count": len(fans),
        "fan_seeds": [
            {
                "name": fan.name,
                "point_exact": [str(fan.point[0]), str(fan.point[1])],
                "supports": [
                    [a, b, str(q)] for a, b, q in fan.supports
                ],
            }
            for fan in fans
        ],
        "ambiguous_owner_spans": [list(span) for span in report.ambiguous_owner_spans],
        "owner_by_edge": [[list(key), name] for key, name in report.owner_by_edge],
    }

    if report.polygon is None:
        result["skeleton"] = {"outcome": "BRIDGE_DID_NOT_MAP"}
        (OUT / "queue_run.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=1)
        )
        print(json.dumps(result["bridge"], ensure_ascii=False, indent=1))
        return

    polygon = report.polygon
    skeleton = build_skeleton(polygon)
    partition = build_faces(polygon, skeleton)
    covered = coverage_at(partition, LATTICE_ALPHA)
    result["polygon"] = {
        "outer_points": len(polygon.outer.points),
        "reflex_count": polygon.reflex_count,
        "fan_edge_count": polygon.fan_edge_count,
        "signed_double_area": str(signed_double_area(polygon.outer.points)),
    }
    result["skeleton"] = {
        "outcome": skeleton.outcome.value,
        "node_count": len(skeleton.nodes),
        "levels": skeleton.levels,
        "semantic_digest": semantic_digest(skeleton),
        "proof_status": skeleton.proof_status.value,
    }
    result["faces"] = {
        "outcome": partition.outcome.value,
        "face_count": len(partition.faces),
        "fan_faces": sum(1 for face in partition.faces if face.is_fan_support),
        "area_reproduces_polygon": partition.area_reproduces_polygon,
    }
    result["coverage"] = {
        "outcome": covered.outcome.value,
        "face_count": len(covered.faces),
    }
    result["nodes"] = [
        {
            "kind": node.kind.value,
            "time": [
                [node.time.canonical().dividend.numerator,
                 node.time.canonical().dividend.denominator],
                sq(node.time.canonical().divisor),
            ],
            "x": sq(node.point.x),
            "y": sq(node.point.y),
            "x_float": float(sum(
                Fraction(c) * Fraction(int(r ** 0.5 * 2 ** 40), 2 ** 40)
                for r, c in node.point.x.terms
            )),
            "y_float": float(sum(
                Fraction(c) * Fraction(int(r ** 0.5 * 2 ** 40), 2 ** 40)
                for r, c in node.point.y.terms
            )),
            "converging_vertices": node.converging_vertices,
        }
        for node in skeleton.nodes
    ]

    (OUT / "queue_run.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=1)
    )
    print(json.dumps(
        {
            "bridge": result["bridge"],
            "reflex_count": result["reflex_count"],
            "reflex_vertices": result["reflex_vertices"],
            "reflex_count_before_snap": result["reflex_count_before_snap"],
            "reflex_vertices_before_snap": result["reflex_vertices_before_snap"],
            "polygon": result["polygon"],
            "skeleton": result["skeleton"],
            "faces": result["faces"],
            "coverage": result["coverage"],
        },
        ensure_ascii=False,
        indent=1,
    ))
    _ = payload, sys


if __name__ == "__main__":
    main()
