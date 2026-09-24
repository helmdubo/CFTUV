"""Ворота карточки EXACT-WORK-BUDGET: ЧТО домен отвечает, без единой секунды.

Отпечаток намеренно не содержит времён: приёмка требует «БАЙТ-В-БАЙТ тот же
ответ», а время меняется от машины и не является ответом. В отпечаток идут
исходы обеих ступеней, деталь, ВСЕ счётчики подготовки, семантический дайджест
скелета каждого региона и число граней покрытия.

Прогон: `python3 artifacts/exact_work_budget/field_gate.py <куда.json>`
из каталога `artifacts/perf_prepare_diag` (там живёт `env.py` харнесса).
"""

from __future__ import annotations

import json
import sys

import env  # noqa: F401


def _skeleton_digests(prepared):
    from cftuv_envelope.wavefront.digest import semantic_digest

    out = []
    for region in prepared.regions:
        if region.skeleton is None:
            out.append(None)
            continue
        out.append(semantic_digest(region.skeleton))
    return out


def _fingerprint(prepared, domain):
    return {
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "counters": dict(prepared.counters),
        "lattice_scale": domain.lattice_scale,
        "faces": len(domain.faces),
        "skeleton_digests": _skeleton_digests(prepared),
    }


def _snapshot_domains(name, density):
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot

    path = env.SNAPSHOTS / name
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    return snapshot_and_request(bundle, selected, density=density)


def _building_domains(patches, density):
    import big_scene
    from cftuv.envelope_request_export import (
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected
    )
    rows = []
    for patch_id in patches:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = build_envelope_analysis_snapshot(
            bundle, included_patch_ids=frozenset({patch_id})
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            0.45,
            decal_request_id_value=request_id,
            density=density,
        )
        rows.append((patch_id, domain_id, snapshot, request))
    return rows


def main():
    out = sys.argv[1]
    from cftuv.envelope_queue_export import run_queue_domain

    results = {}
    for label, rows in (
        ("walls_012_d0", _snapshot_domains("walls_012_snapshot.json", 0)),
        ("wall_2_001_d0", _snapshot_domains("wall_2_001_snapshot.json", 0)),
        ("building_109_121_d0", _building_domains([109, 121], 0)),
    ):
        for patch_id, domain_id, snapshot, request in rows:
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, "0.45"
            )
            key = f"{label}:patch{patch_id}"
            results[key] = _fingerprint(prepared, domain)
            print(key, results[key]["preparation_outcome"],
                  results[key]["coverage_outcome"], flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
