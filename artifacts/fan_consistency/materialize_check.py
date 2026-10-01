"""Материализованный меш: сходится ли число веерных граней меша с суммой граней вееров в покрытии.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/materialize_check.py <mesh> <density> [patch,patch...]

Маршрут продуктовой материализации (как `artifacts/materialize_sweep/sweep.py`): prepare_conveyor ->
conveyor_coverage(0.45) -> materialize_domain под UV_DIRECT_STRIP_V1. На домен печатается: сумма граней
вееров по перечню (`coverage_fan_faces`), `MATERIALIZE_FAN_FACES` меша, число треугольников и кадров
слияния. Равенство «перечень = меш» доказывает, что число скрытых опор на перечне и есть число
веерных граней в материализованной геометрии (слияния вееров между собой нет).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

from cftuv.surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1  # noqa: E402
from cftuv_envelope.materialize.admit import materialization_request  # noqa: E402
from cftuv_envelope.materialize.domain import materialize_domain  # noqa: E402
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor  # noqa: E402


def main() -> int:
    mesh, density = sys.argv[1], int(sys.argv[2])
    manifest = json.loads((_paths.mesh_dir(mesh) / "manifest.json").read_text(encoding="utf-8"))
    patches = sorted(manifest["domains"], key=int)
    if len(sys.argv) > 3:
        patches = [p for p in patches if p in sys.argv[3].split(",")]
    enumerated = {
        r["patch"]: sum(c["coverage_fan_faces"] for c in r["corners"])
        for r in json.loads(
            (_paths.RESULTS / f"fans_{mesh.replace('.', '_')}_d{density}.json").read_text(encoding="utf-8")
        )
    }
    bad = 0
    total_enum = total_mesh = 0
    for patch in patches:
        if not enumerated.get(int(patch)):
            continue
        base = _paths.mesh_dir(mesh) / f"patch_{patch}"
        snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        prep = prepare_conveyor(snap, req)
        coverage = conveyor_coverage(prep, "0.45")
        request = materialization_request(prep, uv_policy_id="UV_DIRECT_STRIP_V1")
        result = materialize_domain(
            prep,
            coverage,
            request=request,
            near_planar_lift_law=NearPlanarLiftLawV1(HOST_NEAR_PLANAR_LIFT_POLICY.value),
        )
        counters = dict(result.counters)
        mesh_fans = counters.get("MATERIALIZE_FAN_FACES")
        total_enum += enumerated[int(patch)]
        total_mesh += mesh_fans or 0
        verdict = "OK" if mesh_fans == enumerated[int(patch)] else "DIFF"
        bad += verdict != "OK"
        print(
            f"{mesh} d{density} patch {patch}: {result.outcome.value} enumerated_fan_faces={enumerated[int(patch)]} "
            f"mesh_fan_faces={mesh_fans} triangles={counters.get('MATERIALIZE_TRIANGLES')} "
            f"faces_merged={counters.get('MATERIALIZE_FACES_MERGED')} {verdict}"
        )
    print(f"TOTAL enumerated={total_enum} mesh={total_mesh} mismatching_domains={bad}")
    return 1 if bad else 0


raise SystemExit(main())
