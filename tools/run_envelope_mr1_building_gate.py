"""Read-only Blender field gate for M-R1 on building.002."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import time

import bmesh
import bpy


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "kernel" / "src"))
for module_name in tuple(sys.modules):
    if (
        module_name == "cftuv"
        or module_name.startswith("cftuv.")
        or module_name == "cftuv_envelope"
        or module_name.startswith("cftuv_envelope.")
    ):
        del sys.modules[module_name]

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.envelope_debug_profile import (  # noqa: E402
    EnvelopeDebugProfileBuilderV1,
)
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeDebugHostOutcome,
    EnvelopeHostAdapterError,
    _typed_value,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
    build_envelope_topology_debug_scene,
)
from cftuv.envelope_topology_debug import (  # noqa: E402
    EnvelopeTopologyPathKind,
)
import cftuv_envelope as kernel  # noqa: E402


def _mesh_fingerprint(obj) -> str:
    mesh = obj.data
    payload = {
        "vertices": [
            (vertex.index, tuple(float(value) for value in vertex.co))
            for vertex in mesh.vertices
        ],
        "edges": [
            (
                edge.index,
                tuple(int(value) for value in edge.vertices),
                bool(edge.use_seam),
            )
            for edge in mesh.edges
        ],
        "faces": [
            (polygon.index, tuple(int(value) for value in polygon.vertices))
            for polygon in mesh.polygons
        ],
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _runtime_payload(
    compilation, request, reference_metric_build_seconds: float
) -> tuple[dict, object | None]:
    result = kernel.evaluate_filtered_runtime_raw_coverage(
        compilation,
        request.requested_alpha,
        reference_metric_build_seconds=reference_metric_build_seconds,
    )
    raw = result.raw_result.raw_coverage
    return {
        "outcome": result.raw_result.outcome.value,
        "diagnostics": [
            {
                "outcome": item.outcome.value,
                "message": item.message,
            }
            for item in result.raw_result.diagnostics
        ],
        "semantic_digest": raw.semantic_digest if raw is not None else None,
        "construction_authority": "RationalAffinePlanarMetricV2",
        "performance": {
            "filtered_predicate_count": (
                result.performance.filtered_predicate_count
            ),
            "fast_path_certified_count": (
                result.performance.fast_path_certified_count
            ),
            "exact_fallback_count": (
                result.performance.exact_fallback_count
            ),
            "fallback_fraction": result.performance.fallback_fraction,
            "reference_metric_build_seconds": (
                result.performance.reference_metric_build_seconds
            ),
            "runtime_view_build_seconds": (
                result.performance.runtime_view_build_seconds
            ),
            "raw_coverage_seconds": (
                result.performance.raw_coverage_seconds
            ),
            "expression_size": result.performance.expression_size,
            "peak_memory_bytes": result.performance.peak_memory_bytes,
        },
    }, raw


def _queue_payload(patch_id: int, domain_id: str, snapshot, request) -> dict:
    """Стадия QUEUE рядом с RAW: подготовка, покрытие, их цена и исход.

    Столбцы RAW не трогаются: очередь считается ДОПОЛНИТЕЛЬНО, на тех же
    снапшоте и запросе, и её числа лежат отдельным разделом. Иначе сравнивать
    было бы нечего — осталась бы одна колонка, и та новая.
    """

    from cftuv.envelope_queue_export import run_queue_domain

    alpha_text = str(request.requested_alpha.value)
    prepared, domain = run_queue_domain(
        patch_id,
        domain_id,
        snapshot,
        request,
        alpha_text,
    )
    if domain.preparation_outcome != "EXACT":
        stage = "QUEUE_PREPARE_REJECTED"
    elif domain.coverage_outcome != "EXACT":
        stage = "QUEUE_COVERAGE_REJECTED"
    else:
        stage = "QUEUE_RESOLVED"
    return {
        "stage": stage,
        "alpha": alpha_text,
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "prepare_seconds": domain.prepare_seconds,
        "coverage_seconds": domain.coverage_seconds,
        "contour_seconds": domain.contour_seconds,
        "faces": len(domain.faces),
        "wall_edges": sum(item.wall_edge_count for item in domain.regions),
        "owners": sorted({face.envelope_spec_id for face in domain.faces}),
        "lattice_scale": domain.lattice_scale,
        "region_outcomes": [
            {
                "region_id": item.region_id,
                "bridge": item.bridge_outcome,
                "skeleton": item.skeleton_outcome,
                "faces": item.face_outcome,
                "coverage": item.coverage_outcome,
                "findings": list(item.findings),
            }
            for item in domain.regions
        ],
        "preparation": prepared,
    }


def _queue_alpha_change(entries, alpha_text: str) -> dict:
    """Цена смены alpha на ТЁПЛЫХ подготовках, по всем доменам сразу.

    Это и есть число приёмки: подготовка alpha-независима, поэтому ползунок
    обязан платить только за покрытие. Меряется здесь, а не в Blender, потому
    что фоновый прогон воспроизводим.
    """

    from cftuv.envelope_queue_export import recompute_queue_coverage

    if not entries:
        return {"domains": 0, "elapsed_seconds": 0.0, "outcomes": []}
    started = time.perf_counter()
    scene = recompute_queue_coverage(entries, alpha_text)
    elapsed = time.perf_counter() - started
    return {
        "domains": len(scene.domains),
        "alpha": alpha_text,
        "elapsed_seconds": elapsed,
        "faces": sum(len(item.faces) for item in scene.domains),
        "owner_palette_slots": [
            {"envelope_spec_id": spec_id, "palette_slot": slot}
            for spec_id, slot in scene.palette
        ],
        "palette_wrapped": scene.palette_wrapped,
        "outcomes": [item.coverage_outcome for item in scene.domains],
    }


def _scope_inputs(bundle, edge_ids: tuple[int, ...], profile):
    """Топология и адресация доменов одного scope. Вынесено из `_run_scope`."""

    topology = build_envelope_topology_debug_scene(
        bundle,
        frozenset(edge_ids),
        profile=profile,
    )
    revision = f"host-source:{bundle.source_revision.digest}:{bundle.source_revision.source_name}"
    patch_ids = tuple(
        sorted(
            int(patch_id)
            for patch_id in bundle.patch_graph.nodes
            if _typed_value("patch-domain", revision, int(patch_id))
            in topology.patch_domain_ids
        )
    )
    selected_edges_by_domain = {
        domain_id: set() for domain_id in topology.patch_domain_ids
    }
    for path in topology.paths:
        if (
            path.kind is EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE
            and path.selected
            and path.patch_domain_id is not None
        ):
            selected_edges_by_domain[path.patch_domain_id].update(
                path.host_edge_ids
            )
    request_id = _typed_value(
        "decal-request",
        revision,
        tuple(
            sorted(
                path.physical_chain_id
                for path in topology.paths
                if path.kind is EnvelopeTopologyPathKind.SELECTED_SOURCE
                and path.physical_chain_id is not None
            )
        ),
    )
    return revision, patch_ids, selected_edges_by_domain, request_id


def _run_scope(bundle, name: str, edge_ids: tuple[int, ...]) -> dict:
    profile = EnvelopeDebugProfileBuilderV1("building.002", "M_R1_FIELD")
    started = time.perf_counter()
    (
        revision,
        patch_ids,
        selected_edges_by_domain,
        request_id,
    ) = _scope_inputs(bundle, edge_ids, profile)
    elapsed = time.perf_counter() - started
    domains = []
    queue_entries = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        try:
            with profile.measure("SNAPSHOT_EXPORT", domain_id):
                snapshot = build_envelope_analysis_snapshot(
                    bundle,
                    included_patch_ids=frozenset({patch_id}),
                    profile=profile,
                )
                request = build_envelope_decal_request(
                    snapshot,
                    frozenset(selected_edges_by_domain[domain_id]),
                    0.3,
                    decal_request_id_value=request_id,
                )
        except EnvelopeHostAdapterError as exc:
            stage = (
                "METRIC_REJECTED"
                if exc.outcome
                in {
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                    EnvelopeDebugHostOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
                }
                else "COMPILE_REJECTED"
            )
            domains.append(
                {
                    "patch_id": patch_id,
                    "patch_domain_id": domain_id,
                    "stage": stage,
                    "outcome": exc.outcome.value,
                    "message": str(exc),
                    "raw_semantic_digest": None,
                    "runtime": None,
                    "queue": None,
                }
            )
            continue
        with profile.measure("QUEUE_PREPARE", domain_id):
            queue = _queue_payload(patch_id, domain_id, snapshot, request)
        preparation = queue.pop("preparation")
        if queue["stage"] == "QUEUE_RESOLVED":
            queue_entries.append((patch_id, domain_id, preparation))
        with profile.measure("COMPILE", domain_id):
            compiled = kernel.compile_reference_envelopes(snapshot, request)
        if compiled.compilation is None:
            domains.append(
                {
                    "patch_id": patch_id,
                    "patch_domain_id": domain_id,
                    "stage": "COMPILE_REJECTED",
                    "outcome": compiled.outcome.value,
                    "message": "; ".join(
                        item.message for item in compiled.diagnostics
                    ),
                    "raw_semantic_digest": None,
                    "runtime": None,
                    "queue": queue,
                }
            )
            continue
        metric_seconds = sum(
            item.elapsed_seconds
            for item in profile.snapshot().timings
            if item.stage == "FRAME_ADMISSION"
            and item.patch_domain_id == domain_id
        )
        with profile.measure("RAW_UNION", domain_id):
            runtime_payload, raw = _runtime_payload(
                compiled.compilation,
                request,
                metric_seconds,
            )
        raw_result = runtime_payload["outcome"]
        stage = "RAW_READY" if raw is not None else "RAW_REJECTED"
        domains.append(
            {
                "patch_id": patch_id,
                "patch_domain_id": domain_id,
                "stage": stage,
                "outcome": raw_result,
                "message": (
                    "authoritative exact RawCoverage available"
                    if raw is not None
                    else "authoritative exact RawCoverage unavailable"
                ),
                "raw_semantic_digest": (
                    raw.semantic_digest
                    if raw is not None
                    else None
                ),
                "runtime": runtime_payload,
                "queue": queue,
            }
        )
    return {
        "scope": name,
        "selected_physical_edge_ids": list(edge_ids),
        "topology_elapsed_seconds": elapsed,
        "profile": profile.snapshot().to_payload(),
        "domains": domains,
        # Смена alpha на тёплых подготовках: ровно то число, которым срез
        # принимается. Вторая alpha намеренно отличается от alpha запроса.
        "queue_alpha_change": _queue_alpha_change(queue_entries, "0.5"),
    }


def main() -> None:
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    output = (
        Path(arguments[0])
        if arguments
        else ROOT / "artifacts" / "envelope_runtime_r1" / "building_002.json"
    )
    obj = bpy.data.objects.get("building.002")
    if obj is None or obj.type != "MESH":
        raise RuntimeError("building.002 mesh object is unavailable")
    before = _mesh_fingerprint(obj)
    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bundle = build_analysis_bundle(
            bm,
            tuple(face.index for face in bm.faces),
            obj,
        )
    finally:
        bm.free()

    seam_edges = tuple(
        edge.index for edge in obj.data.edges if edge.use_seam
    )
    scopes = (
        ("gate_edge_12", (12,)),
        ("one_chain_l0", (2,)),
        ("three_chains_l0", (2, 3, 7)),
        ("ten_chains_l0", tuple(range(1, 11))),
        ("all_seam_chains_l0", seam_edges),
    )
    if len(arguments) > 1:
        requested_scopes = frozenset(arguments[1].split(","))
        scopes = tuple(
            item for item in scopes if item[0] in requested_scopes
        )
        if not scopes:
            raise ValueError("requested M-R1 field scopes are unknown")
    runs = []
    output.parent.mkdir(parents=True, exist_ok=True)
    for name, edges in scopes:
        print(f"M-R1 field scope start: {name}", flush=True)
        runs.append(_run_scope(bundle, name, edges))
        checkpoint = {
            "schema": "cftuv.envelope.runtime_metric_building_gate.v1",
            "source_file": bpy.data.filepath,
            "source_object": obj.name,
            "status": "IN_PROGRESS",
            "runs": runs,
        }
        output.write_text(
            json.dumps(
                checkpoint,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"M-R1 field scope complete: {name}", flush=True)
    after = _mesh_fingerprint(obj)
    payload = {
        "schema": "cftuv.envelope.runtime_metric_building_gate.v1",
        "source_file": bpy.data.filepath,
        "source_object": obj.name,
        "status": "COMPLETE",
        "mesh_counts": {
            "vertices": len(obj.data.vertices),
            "edges": len(obj.data.edges),
            "faces": len(obj.data.polygons),
        },
        "mesh_fingerprint_before": before,
        "mesh_fingerprint_after": after,
        "source_mesh_unchanged": before == after,
        "runs": runs,
    }
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
