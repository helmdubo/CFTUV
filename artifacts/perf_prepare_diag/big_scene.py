"""Большая сцена: бандл из building_full_snapshot.json и три тяжёлых домена.

ИМЕНОВАННАЯ ПОДМЕНА (только для этого меша): у части патчей несколько
граничных петель, и продакшн-классификация OUTER/HOLE идёт через ВРЕМЕННЫЙ
UV-unwrap внутри Blender (`_classify_multi_loop_patches_via_uv` ->
`bpy.ops.uv.unwrap`). Без Blender этот вызов недоступен, поэтому харнесс
подставляет ДРУГОЙ ЖЕ МОДУЛЬНЫЙ путь того же файла —
`_classify_multi_loop_patches_by_nesting` (вложенность в плоскости чарта),
объявленный в продакшн-коде диагностическим. `walls.012` и `walls.001` этой
подмены НЕ ТРЕБУЮТ: их патчи однопетлевые, и там измерен продакшн-путь.
"""

from __future__ import annotations

import json
import sys
import time
from fractions import Fraction

import env  # noqa: F401
from snapshot_bmesh import FakeObject, bmesh_from_snapshot, load_snapshot

from cftuv import analysis_classification as classification

SUBSTITUTION = "HOST_MULTI_LOOP_UV_CLASSIFICATION_SUBSTITUTED_BY_NESTING"


def _install_substitution():
    def by_nesting(bm, classification_inputs, obj):
        classification._classify_multi_loop_patches_by_nesting(
            classification_inputs
        )

    classification._classify_multi_loop_patches_via_uv = by_nesting


_install_substitution()

SNAPSHOT = env.SNAPSHOTS / "building_full_snapshot.json"


def build():
    payload = load_snapshot(SNAPSHOT)
    bm = bmesh_from_snapshot(payload)
    from cftuv.analysis import build_analysis_bundle

    started = time.perf_counter()
    bundle = build_analysis_bundle(
        bm, tuple(range(len(bm.faces))), FakeObject(payload["object_name"])
    )
    return payload, bm, bundle, time.perf_counter() - started


def out_of_plane(node, bm):
    """Грубая непланарность патча: максимальное отклонение от плоскости Ньюэлла."""

    ids = sorted({int(i) for face in node.face_indices for i in
                  (v.index for v in bm.faces[int(face)].verts)})
    if len(ids) < 3:
        return 0.0
    points = [tuple(float(c) for c in bm.verts[i].co) for i in ids]
    nx = ny = nz = 0.0
    for face in node.face_indices:
        cycle = [tuple(float(c) for c in v.co) for v in bm.faces[int(face)].verts]
        for index in range(len(cycle)):
            a = cycle[index]
            b = cycle[(index + 1) % len(cycle)]
            nx += (a[1] - b[1]) * (a[2] + b[2])
            ny += (a[2] - b[2]) * (a[0] + b[0])
            nz += (a[0] - b[0]) * (a[1] + b[1])
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length == 0.0:
        return 0.0
    normal = (nx / length, ny / length, nz / length)
    anchor = points[0]
    return max(
        abs(sum((p[a] - anchor[a]) * normal[a] for a in range(3))) for p in points
    )


def survey():
    payload, bm, bundle, seconds = build()
    print(f"bundle built in {seconds:.2f}s, patches={len(bundle.patch_graph.nodes)}")
    selected = frozenset(int(v) for v in payload["raw"]["selected_edges"])
    rows = []
    for patch_id, node in bundle.patch_graph.nodes.items():
        loop_edges = [
            int(e) for loop in node.boundary_loops for e in loop.edge_indices
        ]
        mine = [e for e in loop_edges if e in selected]
        rows.append(
            {
                "patch_id": int(patch_id),
                "faces": len(node.face_indices),
                "boundary_edges": len(loop_edges),
                "loops": len(node.boundary_loops),
                "selected_edges": len(mine),
                "out_of_plane_m": round(out_of_plane(node, bm), 6),
            }
        )
    rows = [item for item in rows if item["selected_edges"] > 0]
    rows.sort(
        key=lambda item: (
            item["out_of_plane_m"] > 0,
            item["out_of_plane_m"],
            item["boundary_edges"],
        ),
        reverse=True,
    )
    return payload, bundle, selected, rows


def main():
    limit = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    budget = float(sys.argv[3]) if len(sys.argv) > 3 else 600.0
    explicit = (
        [int(value) for value in sys.argv[4].split(",")]
        if len(sys.argv) > 4
        else None
    )
    payload, bundle, selected, rows = survey()
    if explicit is not None:
        order = {value: index for index, value in enumerate(explicit)}
        rows = sorted(
            (item for item in rows if item["patch_id"] in order),
            key=lambda item: order[item["patch_id"]],
        )
        limit = len(rows)
    print(f"domains carrying selected edges: {len(rows)}")
    for item in rows[:12]:
        print("   ", json.dumps(item))

    from cftuv.envelope_topology_export import stage_domain_inputs
    from cftuv.envelope_request_export import (
        EnvelopeHostAdapterError,
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_queue_export import run_queue_domain
    from cftuv_envelope import exact_sqrt_sum as ess

    started = time.perf_counter()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected
    )
    stage_seconds = time.perf_counter() - started
    print(f"stage_domain_inputs: {stage_seconds:.2f}s, domains={len(patch_ids)}")

    factor_calls = []
    original = ess._factorize

    def spy(n):
        mark = time.perf_counter()
        result = original(n)
        factor_calls.append((int(n).bit_length(), time.perf_counter() - mark))
        return result

    ess._factorize = spy

    results = []
    for item in rows[:limit]:
        patch_id = item["patch_id"]
        if patch_id not in patch_ids:
            print(f"patch {patch_id}: not in staged domains, skipped")
            continue
        domain_id = _typed_value("patch-domain", revision, patch_id)
        record = dict(item)
        record["domain_id"] = domain_id[-24:]
        started = time.perf_counter()
        try:
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
        except EnvelopeHostAdapterError as exc:
            record["host_export_s"] = round(time.perf_counter() - started, 2)
            record["outcome"] = f"HOST_REJECT:{exc.outcome.value}"
            record["detail"] = str(exc)[:200]
            results.append(record)
            print(json.dumps(record, ensure_ascii=False))
            continue
        record["host_export_s"] = round(time.perf_counter() - started, 2)
        factor_calls.clear()
        started = time.perf_counter()
        prepared, domain = run_queue_domain(
            patch_id, domain_id, snapshot, request, "0.45"
        )
        record["queue_s"] = round(time.perf_counter() - started, 2)
        record["prepare_s"] = round(domain.prepare_seconds, 2)
        record["coverage_s"] = round(domain.coverage_seconds, 2)
        record["contour_s"] = round(domain.contour_seconds, 2)
        record["outcome"] = domain.preparation_outcome
        record["coverage_outcome"] = domain.coverage_outcome
        record["detail"] = (domain.detail or "")[:120]
        record["factorization_s"] = round(sum(t for _, t in factor_calls), 2)
        record["factorization_calls"] = len(factor_calls)
        record["radicand_bits_max"] = (
            max((b for b, _ in factor_calls), default=0)
        )
        record["counters"] = {
            k: v
            for k, v in dict(prepared.counters).items()
            if k
            in (
                "CONVEYOR_SOURCE_EDGES",
                "CONVEYOR_WALL_EDGES",
                "CONVEYOR_DOMAIN_EDGES",
                "CONVEYOR_LATTICE_SCALE",
                "CONVEYOR_SKELETON_NODES",
                "CONVEYOR_FACES",
            )
        }
        results.append(record)
        print(json.dumps(record, ensure_ascii=False))
        if time.perf_counter() - started > budget:
            print("budget exceeded, stopping")
            break
    (env.OUT / "big_scene.json").write_text(
        json.dumps({"survey": rows, "runs": results, "substitution": SUBSTITUTION},
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
