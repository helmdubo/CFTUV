"""FAN-MIRROR-DIAG, шаг 1: слепок стены -> вход очереди на решётке.

Запуск из каталога worktree:

    PYTHONPATH=kernel/src:kernel/tests python3 <путь>/step1_fixture.py

Закон привязки — корпусной: `cftuv_envelope.robust.grid.snap_value` при
`GridSpecV1(scale=16384)`. Это floor(v*scale + 1/2), то есть half-UP, а не
half-even; масштаб 16384 взят из полевого прогона walls.001
(`CONVEYOR_LATTICE_SCALE 16384`, grid_step 6.103516e-05).
"""

from __future__ import annotations

import json
from fractions import Fraction
from pathlib import Path
import sys

from cftuv_envelope.robust.grid import GridSpecV1, snap_value

ROOT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/wt-fandiag")
OUT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/fan_diag")
SCALE = int(sys.argv[1]) if len(sys.argv) > 1 else 32768
GRID = GridSpecV1(scale=SCALE)


def exact(value: float) -> Fraction:
    """Точная двоичная дробь binary64. Без округления и без nsimplify."""

    return Fraction(value)


def signed_double_area(points) -> Fraction:
    total = Fraction(0)
    size = len(points)
    for index in range(size):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % size]
        total += x0 * y1 - x1 * y0
    return total


def main() -> None:
    snapshot = json.loads(
        (ROOT / "artifacts/field_snapshots/wall_2_001_snapshot.json").read_text()
    )
    angles = json.loads(
        (ROOT / "artifacts/field_snapshots/wall_2_001_corner_angles.json").read_text()
    )
    vertices = snapshot["raw"]["vertices"]
    order = angles["loop_order"]

    xs = {exact(v[0]) for v in vertices}
    assert len(xs) == 1, "стена объявлена точно плоской x=const"
    plane_x = next(iter(xs))

    # Карта (u, v) = (y, z). Базис правый: y x z = +x, поэтому CCW в карте
    # означает материал со стороны +x объекта.
    chart = [(exact(vertices[i][1]), exact(vertices[i][2])) for i in order]
    area2 = signed_double_area(chart)

    snapped = [(snap_value(u, GRID), snap_value(v, GRID)) for u, v in chart]
    area2_snapped = signed_double_area(
        [(Fraction(u), Fraction(v)) for u, v in snapped]
    )

    residuals = [
        (
            abs(Fraction(su, SCALE) - u),
            abs(Fraction(sv, SCALE) - v),
        )
        for (u, v), (su, sv) in zip(chart, snapped)
    ]
    max_residual = max(max(pair) for pair in residuals)

    # Половинные случаи: там, где v*scale ровно на середине между узлами,
    # half-up и half-even расходятся. Их наличие — единственная причина, по
    # которой выбор закона округления вообще может сломать зеркальность.
    half_cases = []
    for index, (u, v) in enumerate(chart):
        for axis, value in (("u", u), ("v", v)):
            scaled = value * SCALE
            if scaled.denominator == 2:
                half_cases.append(
                    {
                        "loop_index": index,
                        "vertex": order[index],
                        "axis": axis,
                        "scaled": str(scaled),
                    }
                )

    collisions = {}
    for index, node in enumerate(snapped):
        collisions.setdefault(node, []).append(order[index])
    merged = {str(k): v for k, v in collisions.items() if len(v) > 1}

    payload = {
        "schema": "cftuv.fan_diag.fixture.v1",
        "source_snapshot": "artifacts/field_snapshots/wall_2_001_snapshot.json",
        "object_name": snapshot["object_name"],
        "plane": {"law": "x = const", "x_exact": str(plane_x)},
        "chart": {
            "law": "CHART_IS_THE_OBJECT_PLANE_AXES_V1",
            "u_axis": "object y",
            "v_axis": "object z",
            "origin": "object origin (no translation applied)",
            "named_gap": "HOST_CHART_FRAME_UNAVAILABLE_WITHOUT_BLENDER",
        },
        "snap_law": {
            "function": "cftuv_envelope.robust.grid.snap_value",
            "formula": "(2*num + den) // (2*den) on v*scale  ==  floor(v*scale + 1/2)",
            "rounding": "HALF_UP (не half-even; корпусной закон)",
            "scale": SCALE,
            "grid_step": str(Fraction(1, SCALE)),
            "scale_provenance": "wall 2.001 owner sidecar: CONVEYOR_LATTICE_SCALE 32768; walls.001 earlier run: 16384",
            "magnitude_bound": GRID.magnitude_bound,
            "max_abs_node": max(max(abs(a), abs(b)) for a, b in snapped),
        },
        "orientation": {
            "signed_double_area_chart_exact": str(area2),
            "signed_double_area_chart_float": float(area2),
            "is_counter_clockwise": area2 > 0,
            "material_side": (
                "+x side of the wall plane" if area2 > 0 else "-x side of the wall plane"
            ),
            "signed_double_area_snapped": str(area2_snapped),
        },
        "snap_report": {
            "max_residual_exact": str(max_residual),
            "max_residual_float": float(max_residual),
            "budget_of_field_run": str(Fraction(1, 80)),
            "half_way_cases": half_cases,
            "merged_nodes": merged,
        },
        "loop_order": order,
        "vertices_chart_exact": [
            {"vertex": order[i], "u": str(chart[i][0]), "v": str(chart[i][1])}
            for i in range(len(order))
        ],
        "vertices_snapped": [
            {"vertex": order[i], "u": snapped[i][0], "v": snapped[i][1]}
            for i in range(len(order))
        ],
    }
    (OUT / f"fixture_wall_2_001_lattice_{SCALE}.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=1, sort_keys=False)
    )
    print(json.dumps(
        {
            k: payload[k]
            for k in ("plane", "chart", "snap_law", "orientation", "snap_report")
        },
        ensure_ascii=False,
        indent=1,
    ))


if __name__ == "__main__":
    main()
