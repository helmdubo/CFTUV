"""Один домен (patch) хранимой сцены building на заданных плотностях.

Запуск из каталога artifacts/perf_prepare_diag:
  PYTHONPATH=. python ../faces_chain_refusal/run17.py <patch_id> <d,d,..> [outjson]
"""
from __future__ import annotations
import json, sys, time
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon


def main():
    patch_id = int(sys.argv[1])
    densities = [int(x) for x in sys.argv[2].split(",")]
    out = sys.argv[3] if len(sys.argv) > 3 else None
    import big_scene
    from cftuv.envelope_queue_export import run_queue_domain
    from cftuv.envelope_request_export import (
        _typed_value, build_envelope_analysis_snapshot, build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected)
    domain_id = _typed_value("patch-domain", revision, patch_id)
    print("domain_id", domain_id)
    rows = {}
    for d in densities:
        getattr(canon, "reset_factorization_memory", lambda: None)()
        getattr(canon, "reset_unbudgeted_work", lambda: None)()
        t = time.perf_counter()
        snapshot = build_envelope_analysis_snapshot(bundle, included_patch_ids=frozenset({patch_id}))
        request = build_envelope_decal_request(
            snapshot, frozenset(by_domain[domain_id]), 0.45,
            decal_request_id_value=request_id, density=d)
        prepared, domain = run_queue_domain(patch_id, domain_id, snapshot, request, "0.45")
        row = {
            "seconds": round(time.perf_counter() - t, 2),
            "prep": str(domain.preparation_outcome),
            "cov": str(domain.coverage_outcome),
            "detail": domain.detail,
            "counters": [list(c) for c in prepared.counters] if getattr(prepared, "counters", None) else None,
        }
        rows[d] = row
        print("D", d, row["seconds"], row["prep"], row["cov"], (row["detail"] or "")[:200], flush=True)
    if out:
        json.dump(rows, open(out, "w"), ensure_ascii=False, indent=1)


main()
