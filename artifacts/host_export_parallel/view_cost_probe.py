"""Цена отдельных кусков выгрузки хоста вне Blender: вид патча, снапшот, запрос.

    python artifacts/host_export_parallel/view_cost_probe.py
"""

from __future__ import annotations

import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "perf_prepare_diag"))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    _typed_value,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_analysis_bundle_id_view,
    build_envelope_topology_export,
    stage_domain_inputs,
)


def main() -> None:
    _, bundle, selected, _ = big_scene.survey()
    topology = build_envelope_topology_export(bundle)
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected, topology_export=topology
    )
    started = time.perf_counter()
    views = {
        pid: build_analysis_bundle_id_view(bundle, frozenset({pid}))
        for pid in patch_ids
    }
    print("views", round(time.perf_counter() - started, 3))
    surface = bundle.patch_surface
    print(
        "surface sizes: vertices",
        len(surface.vertices),
        "edges",
        len(surface.edges),
        "faces",
        len(surface.faces),
        "triangles",
        len(surface.triangles),
    )
    snapshots = {}
    requests = {}
    snap_total = 0.0
    req_total = 0.0
    for pid in patch_ids:
        domain_id = _typed_value("patch-domain", revision, pid)
        started = time.perf_counter()
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle,
                included_patch_ids=frozenset({pid}),
                topology_export=topology,
                analysis_view=views[pid],
            )
        except EnvelopeHostAdapterError:
            snap_total += time.perf_counter() - started
            continue
        snap_total += time.perf_counter() - started
        started = time.perf_counter()
        requests[pid] = build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            0.45,
            decal_request_id_value=request_id,
            density=2,
        )
        req_total += time.perf_counter() - started
        snapshots[pid] = snapshot
    print("snapshots", round(snap_total, 3), "requests", round(req_total, 3))
    started = time.perf_counter()
    blobs = {pid: pickle.dumps((snapshots[pid], requests[pid]), 5) for pid in snapshots}
    print("pickle snapshot+request", round(time.perf_counter() - started, 3))
    print("pickle bytes total", sum(len(b) for b in blobs.values()), "max", max(len(b) for b in blobs.values()))
    started = time.perf_counter()
    for blob in blobs.values():
        pickle.loads(blob)
    print("unpickle snapshot+request", round(time.perf_counter() - started, 3))
    # the light input that would be shipped: surface slice + chains
    started = time.perf_counter()
    inputs = {
        pid: pickle.dumps(
            (
                views[pid].patch_surface,
                tuple(r for r in topology.host_chains if r.patch_id == pid),
            ),
            5,
        )
        for pid in snapshots
    }
    print("pickle light input", round(time.perf_counter() - started, 3), "bytes", sum(len(b) for b in inputs.values()))


if __name__ == "__main__":
    main()
