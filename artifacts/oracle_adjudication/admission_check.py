"""Официальный допуск: та же PolygonV1, что реально течёт в build_skeleton(),
через rings_of_polygon() из tools/surfer2_oracle.py -- НЕ переизобретённый.
Пишет JSON-расписку допуска.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/wt-oracle")
DIAG = WT / "artifacts" / "perf_prepare_diag"
FGF = WT / "artifacts" / "field_gate_freeze"
SNAPSHOTS = WT / "artifacts" / "field_snapshots"
TOOLS = WT / "tools"
OUT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/oracle_adjudication")

sys.path.insert(0, str(DIAG))
sys.path.insert(0, str(FGF))
sys.path.insert(0, str(TOOLS))

import env  # noqa: E402,F401
from run_domain import bundle_from_field_snapshot  # noqa: E402
from snapshot_bmesh import selected_edge_ids  # noqa: E402
from field_route import staged_domains  # noqa: E402

from surfer2_oracle import rings_of_polygon  # noqa: E402


def main():
    alpha = 0.254
    density = 0
    payload, bm, bundle = bundle_from_field_snapshot(SNAPSHOTS / "wall_2_001_snapshot.json")
    selected = selected_edge_ids(payload)

    from cftuv.envelope_queue_export import run_queue_domain

    records = []
    for entry in staged_domains(bundle, selected, alpha=alpha, density=density):
        if entry["ready"] is None:
            continue
        patch_id, domain_id, snapshot, request = entry["ready"]
        prepared, domain = run_queue_domain(patch_id, domain_id, snapshot, request, str(alpha))
        for region in prepared.regions:
            polygon = region.bridge.polygon if region.bridge is not None else None
            if polygon is None:
                records.append({
                    "region_id": region.region_id,
                    "domain_id": domain_id,
                    "polygon": None,
                    "note": "bridge did not produce a polygon",
                })
                continue
            outcome, detail, rings = rings_of_polygon(polygon)
            records.append({
                "region_id": region.region_id,
                "domain_id": domain_id,
                "skeleton_nodes_kernel": None if region.skeleton is None else len(region.skeleton.nodes),
                "vertex_fans_count": len(polygon.vertex_fans),
                "vertex_fan_supports_all_speed_zero": all(
                    s.speed_squared == 0 for fan in polygon.vertex_fans for s in fan.supports
                ),
                "loop_edge_count": sum(len(loop.points) for loop in polygon.loops),
                "loop_edges_with_zero_speed": sum(
                    1 for loop in polygon.loops for s in loop.edge_speeds_squared if s == 0
                ),
                "AdmissionOutcome": outcome.value,
                "detail": detail,
                "rings_produced": len(rings),
            })
    result = {
        "purpose": (
            "Официальный допуск через уже объявленную границу пересечения моделей "
            "(tools/surfer2_oracle.py:rings_of_polygon), не переизобретённую здесь."
        ),
        "snapshot": "artifacts/field_snapshots/wall_2_001_snapshot.json",
        "alpha": alpha,
        "density": density,
        "records": records,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "admission_check.json").write_text(json.dumps(result, indent=1, ensure_ascii=False))
    print(json.dumps(result, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
