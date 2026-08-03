"""Полевой прогон домена из слепка без Blender: staged QUEUE + тайминги."""

from __future__ import annotations

import json
import time

import env  # noqa: F401  (ставит пути и заглушки)
from snapshot_bmesh import (
    FakeObject,
    bmesh_from_snapshot,
    load_snapshot,
    selected_edge_ids,
)

ALPHA = 0.45
DENSITY = 2


def bundle_from_field_snapshot(path, *, vertex_override=None):
    from cftuv.analysis import build_analysis_bundle

    payload = load_snapshot(path)
    bm = bmesh_from_snapshot(payload, vertex_override=vertex_override)
    obj = FakeObject(payload["object_name"])
    bundle = build_analysis_bundle(bm, tuple(range(len(bm.faces))), obj)
    return payload, bm, bundle


def snapshot_and_request(bundle, selected, *, alpha=ALPHA, density=DENSITY):
    """Тот же пролог, что у `evaluate_envelope_queue_staged`, но по доменам."""

    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    (
        topology_scene,
        revision,
        patch_ids,
        request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(bundle, frozenset(selected))
    from cftuv.envelope_request_export import _typed_value

    out = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = build_envelope_analysis_snapshot(
            bundle, included_patch_ids=frozenset({patch_id})
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(selected_edges_by_domain[domain_id]),
            alpha,
            decal_request_id_value=request_id,
            density=density,
        )
        out.append((patch_id, domain_id, snapshot, request))
    return out


def timed_queue(patch_id, domain_id, snapshot, request, *, alpha_text=None):
    from cftuv.envelope_queue_export import run_queue_domain

    alpha_text = alpha_text or str(float(ALPHA))
    started = time.perf_counter()
    prepared, domain = run_queue_domain(
        patch_id, domain_id, snapshot, request, alpha_text
    )
    total = time.perf_counter() - started
    return prepared, domain, total


def report(prepared, domain, total):
    return {
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "prepare_ms": round(domain.prepare_seconds * 1000.0, 1),
        "coverage_ms": round(domain.coverage_seconds * 1000.0, 1),
        "contour_ms": round(domain.contour_seconds * 1000.0, 1),
        "total_ms": round(total * 1000.0, 1),
        "counters": dict(prepared.counters),
        "faces": len(domain.faces),
        "lattice_scale": domain.lattice_scale,
    }


def main():
    path = env.SNAPSHOTS / "walls_012_snapshot.json"
    payload, bm, bundle = bundle_from_field_snapshot(path)
    selected = selected_edge_ids(payload)
    print("selected edges:", sorted(selected))
    rows = snapshot_and_request(bundle, selected)
    print("domains:", [(p, d) for p, d, _, _ in rows])
    results = {}
    for patch_id, domain_id, snapshot, request in rows:
        prepared, domain, total = timed_queue(patch_id, domain_id, snapshot, request)
        results[str(patch_id)] = report(prepared, domain, total)
        print(json.dumps(results[str(patch_id)], ensure_ascii=False, indent=2))
    (env.OUT / "walls_012_staged.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
