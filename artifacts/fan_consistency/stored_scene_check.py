"""Независимость от маршрута: веера `building` по СОХРАНЁННОЙ сцене (без Blender) против экспорта из Blender.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/stored_scene_check.py <density> [patch,patch...]

Маршрут сохранённой сцены — `artifacts/perf_prepare_diag/big_scene.py` (заглушки bpy/bmesh, подмена
многопетлевой классификации вложенностью); экспорт Blender — `export_domains.py` (настоящий bmesh).
Сравниваются по вершине: итоговый H спеки и закон лифта. Расхождение — отказ с кодом 1.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "artifacts" / "perf_prepare_diag"))
import _paths  # noqa: E402

import env  # noqa: E402,F401  (заглушки и пути харнесса)
import big_scene  # noqa: E402
import fanlib  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    _typed_value,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
)
from cftuv.envelope_topology_export import stage_domain_inputs  # noqa: E402
from cftuv_envelope.wavefront import prepare_conveyor  # noqa: E402


def main() -> int:
    density = int(sys.argv[1])
    wanted = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else None
    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected)
    exported = {
        r["patch"]: {c["vertex"]: (c["spec_H"], c["lift_law"], round(c["eval_turn_deg"], 9)) for c in r["corners"]}
        for r in json.loads((_paths.RESULTS / f"fans_building_d{density}.json").read_text(encoding="utf-8"))
    }
    bad = 0
    checked = 0
    for patch_id in sorted(patch_ids):
        if wanted is not None and patch_id not in wanted:
            continue
        if not exported.get(patch_id):
            continue
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = build_envelope_analysis_snapshot(bundle, included_patch_ids=frozenset({patch_id}))
        request = build_envelope_decal_request(
            snapshot, frozenset(by_domain[domain_id]), 0.45, decal_request_id_value=request_id, density=density
        )
        prep = prepare_conveyor(snapshot, request)
        view = fanlib.DomainView(snapshot, prep)
        for spec in view.specs:
            vertex, prev, nxt = view.corner_neighbours(spec)
            _d, _c, _, _, ang = view.turn(view.eval_xy, vertex, prev, nxt)
            lift = getattr(spec, "evaluation_subturn_count_lift", None)
            ours = (spec.resolved_hidden_edge_count, None if lift is None else lift.lift_law.name, round(ang, 9))
            theirs = exported[patch_id].get(fanlib.vertex_index(vertex))
            checked += 1
            if theirs is None or tuple(theirs) != ours:
                bad += 1
                print("MISMATCH patch", patch_id, "v", fanlib.vertex_index(vertex), "stored", ours, "blender", theirs)
    print(f"d{density}: checked {checked} fans, mismatches {bad}")
    return 1 if bad else 0


raise SystemExit(main())
