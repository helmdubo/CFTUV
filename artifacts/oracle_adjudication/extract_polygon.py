"""Извлечение PolygonV1, фактически поданного в build_skeleton() для wall 2.001.

Переиспользует field_route.py / run_domain.py / env.py 1:1 (та же цепочка
вызовов, что и в полевой заморозке), только вместо сводной записи достаёт
region.bridge.polygon и сериализует его в JSON для tools/surfer2_oracle.py.
"""
from __future__ import annotations

import json
import sys
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
WT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/wt-oracle")
DIAG = WT / "artifacts" / "perf_prepare_diag"
FGF = WT / "artifacts" / "field_gate_freeze"
SNAPSHOTS = WT / "artifacts" / "field_snapshots"

sys.path.insert(0, str(DIAG))
sys.path.insert(0, str(FGF))
sys.path.insert(0, str(WT / "tools"))

import env  # noqa: E402,F401
from run_domain import bundle_from_field_snapshot  # noqa: E402
from snapshot_bmesh import selected_edge_ids  # noqa: E402
from field_route import staged_domains  # noqa: E402


def main():
    alpha = 0.254
    density = 0
    payload, bm, bundle = bundle_from_field_snapshot(SNAPSHOTS / "wall_2_001_snapshot.json")
    selected = selected_edge_ids(payload)

    from cftuv.envelope_queue_export import run_queue_domain

    out = {"snapshot": "wall_2_001_snapshot.json", "domains": []}
    for entry in staged_domains(bundle, selected, alpha=alpha, density=density):
        if entry["ready"] is None:
            out["domains"].append({
                "patch_id": entry["patch_id"],
                "domain_id": entry["domain_id"],
                "stage": entry["stage"],
                "outcome": entry["outcome"],
            })
            continue
        patch_id, domain_id, snapshot, request = entry["ready"]
        prepared, domain = run_queue_domain(patch_id, domain_id, snapshot, request, str(alpha))
        rec = {
            "patch_id": int(patch_id),
            "domain_id": domain_id,
            "preparation_outcome": domain.preparation_outcome,
            "regions": [],
        }
        for region in prepared.regions:
            polygon = region.bridge.polygon if region.bridge is not None else None
            region_rec = {
                "region_id": region.region_id,
                "bridge_outcome": region.bridge_outcome.value if region.bridge_outcome else None,
                "skeleton_outcome": region.skeleton_outcome.value if region.skeleton_outcome else None,
                "skeleton_nodes": None if region.skeleton is None else len(region.skeleton.nodes),
                "has_polygon": polygon is not None,
            }
            if polygon is not None:
                def frac_str(v):
                    # ЧЕСТНО: q -- рациональное число (Fraction), не int.
                    # int(Fraction) ТРУНКИРУЕТ к нулю для 0<q<1 -- была бы
                    # ложная запись "speed_squared: 0" для q=1328.../3943...
                    return str(Fraction(v))

                region_rec["vertex_fans_count"] = len(polygon.vertex_fans)
                region_rec["vertex_fans"] = [
                    {
                        "point": list(fan.point),
                        "supports": [
                            {
                                "normal": list(s.normal),
                                "speed_squared": frac_str(s.speed_squared),
                                "speed_squared_float": float(s.speed_squared),
                            }
                            for s in fan.supports
                        ],
                    }
                    for fan in polygon.vertex_fans
                ]
                loops = []
                for loop in polygon.loops:
                    loops.append({
                        "points": [list(p) for p in loop.points],
                        "edge_speeds_squared": [frac_str(v) for v in loop.edge_speeds_squared],
                        "edge_speeds_squared_float": [float(v) for v in loop.edge_speeds_squared],
                        "source_flags": list(loop.source_flags),
                    })
                region_rec["loops"] = loops
            out["domains"].append({**rec, "region": region_rec} if False else rec)
            rec["regions"].append(region_rec)
    Path(sys.argv[1] if len(sys.argv) > 1 else str(HERE / "polygon_dump.json")).write_text(
        json.dumps(out, ensure_ascii=False, indent=1)
    )
    print("wrote", sys.argv[1] if len(sys.argv) > 1 else str(HERE / "polygon_dump.json"))


if __name__ == "__main__":
    main()
