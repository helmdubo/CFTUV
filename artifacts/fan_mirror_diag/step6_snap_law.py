"""FAN-MIRROR-DIAG, шаг 6: ковариантность ЗАКОНА ПРИВЯЗКИ и чувствительность к карте.

T1 ЗАКОН   `snap_value` есть floor(v*scale + 1/2) — HALF-UP. Половинное
           значение при отражении переходит в половинное же, и half-up
           округляет их В ОДНУ СТОРОНУ. Поэтому `snap(S - v) != S - snap(v)`
           ровно на половинных узлах. Проверяется на всех 35 вершинах.
T2 КАРТА   начало отсчёта хостовой карты недоступно без Blender
           (`HOST_CHART_FRAME_UNAVAILABLE_WITHOUT_BLENDER`). Сдвиг начала на
           долю шага меняет, какие вырождения решётка восстановит. Мерится
           числом узлов скелета по сетке сдвигов.
"""

from __future__ import annotations

import json
from fractions import Fraction
from pathlib import Path
import sys

from cftuv_envelope.robust.grid import GridSpecV1, snap_value
from cftuv_envelope.wavefront.digest import semantic_digest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step3_queue import OUT, load_chart  # noqa: E402
from step4_probes import pipeline  # noqa: E402


def snap_pair(value: Fraction, scale: int) -> int:
    return snap_value(value, GridSpecV1(scale=scale))


def main() -> None:
    payload, order, exact_points, lattice_points = load_chart()

    result = {"schema": "cftuv.fan_diag.snap_law.v1"}

    # ---- T1: ковариантность закона привязки ---------------------------------
    rows = []
    for scale in (16384, 32768, 65536):
        broken = []
        for label, (u, v) in zip(order, exact_points):
            direct = snap_pair(u, scale)
            reflected = snap_pair(-u, scale)
            if reflected != -direct:
                broken.append(
                    {
                        "vertex": label,
                        "u_exact": str(u),
                        "snap_u": direct,
                        "snap_minus_u": reflected,
                        "minus_snap_u": -direct,
                        "gap_lattice_steps": reflected + direct,
                    }
                )
        rows.append(
            {
                "scale": scale,
                "vertices": len(order),
                "law_is_mirror_covariant": not broken,
                "broken_vertex_count": len(broken),
                "broken": broken,
            }
        )
    result["T1_snap_law_mirror_covariance"] = {
        "law": "snap_value = floor(v*scale + 1/2)  (HALF_UP)",
        "counterexample_form": "snap(-v) != -snap(v) при v*scale ровно полуцелом",
        "rows": rows,
    }

    # ---- T2: чувствительность к началу карты --------------------------------
    scale = 32768
    sweep = []
    for numerator in range(0, 8):
        shift = Fraction(numerator, 8 * scale)
        points = tuple(
            (
                snap_pair(u + shift, scale),
                snap_pair(v + shift, scale),
            )
            for u, v in exact_points
        )
        if len(set(points)) != len(points):
            sweep.append(
                {
                    "shift_lattice_steps": float(shift * scale),
                    "outcome": "SNAP_MERGED_TWO_VERTICES",
                }
            )
            continue
        polygon, skeleton, partition = pipeline(points)
        widths = sorted(
            {
                abs(points[i][0] - points[j][0])
                for i, j in ()
            }
        )
        sweep.append(
            {
                "shift_lattice_steps": float(shift * scale),
                "outcome": skeleton.outcome.value,
                "node_count": len(skeleton.nodes),
                "face_count": len(partition.faces),
                "fan_count": polygon.fan_edge_count,
                "digest": semantic_digest(skeleton),
            }
        )
        _ = widths
    result["T2_chart_origin_sensitivity"] = {
        "named_gap": "HOST_CHART_FRAME_UNAVAILABLE_WITHOUT_BLENDER",
        "field_node_count": 45,
        "field_face_count": 47,
        "field_fan_count": 12,
        "scale": scale,
        "sweep": sweep,
        "node_counts_seen": sorted(
            {row.get("node_count") for row in sweep if "node_count" in row}
        ),
        "face_counts_seen": sorted(
            {row.get("face_count") for row in sweep if "face_count" in row}
        ),
    }

    (OUT / "snap_law.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=1)
    )
    print(json.dumps(result, ensure_ascii=False, indent=1))
    _ = payload, lattice_points


if __name__ == "__main__":
    main()
