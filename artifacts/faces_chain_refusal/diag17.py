"""Диагностика FACE_CHAIN_DOES_NOT_CLOSE на домене patch17 хранимой сцены.
Запуск из artifacts/perf_prepare_diag: PYTHONPATH=. python ../faces_chain_refusal/diag17.py <patch> <density,..>
"""
from __future__ import annotations
import json, sys, collections
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon


def stage(patch_id, d):
    import big_scene
    from cftuv.envelope_request_export import (
        _typed_value, build_envelope_analysis_snapshot, build_envelope_decal_request)
    from cftuv.envelope_topology_export import stage_domain_inputs
    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected)
    domain_id = _typed_value("patch-domain", revision, patch_id)
    snap = build_envelope_analysis_snapshot(bundle, included_patch_ids=frozenset({patch_id}))
    return snap, (lambda dd: build_envelope_decal_request(
        snap, frozenset(by_domain[domain_id]), 0.45, decal_request_id_value=request_id, density=dd))


def main():
    patch_id = int(sys.argv[1])
    dens = [int(x) for x in sys.argv[2].split(",")]
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    snap, mk = stage(patch_id, 0)
    for d in dens:
        canon.reset_factorization_memory(); canon.reset_unbudgeted_work()
        prepared = prepare_conveyor(snap, mk(d))
        print("=== density", d, "outcome", prepared.outcome, "detail", prepared.detail[:160])
        print(" counters", {k: v for k, v in prepared.counters if k.startswith("CONVEYOR") or "FAN" in k})
        for r in prepared.regions:
            poly = r.bridge.polygon
            print(" region", r.region_id, "bridge", r.bridge_outcome.value, "skel", r.skeleton_outcome, "face", r.face_outcome,
                  "partition.detail=", (r.partition.detail if r.partition else None))
            if poly is None: continue
            loops = poly.loops
            print("  loops", [len(l.points) for l in loops], "fans", len(poly.vertex_fans),
                  "fan supports", [len(f.supports) for f in poly.vertex_fans],
                  "wall_edges", r.wall_edge_count)
            sk = r.skeleton
            kinds = collections.Counter(n.kind.value for n in sk.nodes)
            print("  skeleton nodes", len(sk.nodes), dict(kinds), "levels", sk.levels)
            print("  skel counters", {k: v for k, v in sk.counters if v})
        if d == dens[-1]: pass


main()
