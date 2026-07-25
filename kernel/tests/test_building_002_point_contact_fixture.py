from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from pathlib import Path

import cftuv_envelope as kernel


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "kernel" / "fixtures" / "building_002_point_contact_v1"
EVIDENCE = ROOT / "artifacts" / "envelope_c_r2c_fixture"
HISTORICAL_SHA = "df587ed166cfb0e0b615148f08c583b4477c5ac4"
SELECTED_SHA = "c2622d07020338e5231b81f41655fe6c74cdca72"


def _canonical_json(payload) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_contracts():
    snapshot_bytes = (FIXTURE / "analysis_snapshot.json").read_bytes()
    request_bytes = (FIXTURE / "decal_request.json").read_bytes()
    return (
        snapshot_bytes,
        request_bytes,
        kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes),
        kernel.DecalRequestCodecV1.loads(request_bytes),
    )


def _point_topology_from_historical(report: dict) -> list[dict]:
    result = []
    for vertex in report["evaluation"]["rejected_topology"]["rejected_vertices"]:
        chains = {
            chain_id
            for edge in vertex["outgoing_edges"]
            for chain_id in edge["provenance"]["coverage_contributors"][
                "chain_use_ids"
            ]
        }
        result.append(
            {
                "point": (
                    vertex["exact_point"]["x"]["expression"],
                    vertex["exact_point"]["y"]["expression"],
                ),
                "chain_use_ids": tuple(sorted(chains)),
            }
        )
    return sorted(result, key=lambda item: item["point"])


def _point_topology_from_selected(report: dict) -> list[dict]:
    raw = report["evaluation"]["raw_coverage"]
    occurrences_by_point = defaultdict(list)
    for occurrence in raw["shared_boundary_occurrences"]:
        occurrences_by_point[occurrence["arrangement_point_id"]].append(occurrence)
    result = []
    for contact in raw["point_contacts"]:
        chains = {
            chain_id
            for occurrence in occurrences_by_point[contact["arrangement_point_id"]]
            for chain_id in occurrence["provenance"]["coverage_contributors"][
                "chain_use_ids"
            ]
        }
        result.append(
            {
                "point": (
                    contact["exact_point"]["x"]["expression"],
                    contact["exact_point"]["y"]["expression"],
                ),
                "chain_use_ids": tuple(sorted(chains)),
            }
        )
    return sorted(result, key=lambda item: item["point"])


def test_fixture_contracts_hashes_and_ids_are_stable():
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))
    snapshot_bytes, request_bytes, snapshot, request = _load_contracts()
    host_mesh_bytes = (FIXTURE / "host_mesh.json").read_bytes()

    assert kernel.AnalysisSnapshotCodecV1.dumps(snapshot) == snapshot_bytes
    assert kernel.DecalRequestCodecV1.dumps(request) == request_bytes
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    assert kernel.validate_decal_request(request) == ()
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()

    actual_hashes = {
        "analysis_snapshot.json": _sha256(snapshot_bytes),
        "decal_request.json": _sha256(request_bytes),
        "host_mesh.json": _sha256(host_mesh_bytes),
    }
    assert actual_hashes == manifest["files"]
    assert manifest["fixture_hash"] == _sha256(
        _canonical_json(
            [[name, actual_hashes[name]] for name in sorted(actual_hashes)]
        )
    )
    assert manifest["source_mesh_fingerprint"] == (
        "3205834b7ead22cf1eb8985902144ca1c7295a88b8015556b364be17d8ac9021"
    )
    assert sorted(item.value for item in request.selected_chain_use_ids) == (
        manifest["selected_chain_use_ids"]
    )
    assert sorted(item.physical_chain_id.value for item in snapshot.physical_chains) == (
        manifest["physical_chain_ids"]
    )


def test_the_metric_descriptor_is_a_rebuild_of_the_fixtures_own_source():
    """Дескриптор метрики — не отдельные данные, а функция от вершин и граней.

    Проверяется побитово, и это же делает фикстуру перегенерируемой без
    Blender: когда контракт метрики меняется, дескриптор пересобирается ровно
    этим вызовом, а не правится руками.
    """

    _, _, snapshot, _ = _load_contracts()
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == descriptor.patch_domain_id
    )
    rebuilt = kernel.build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=descriptor.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        planarity_policy=kernel.PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=kernel.GridSnappingLawV1.UNSNAPPED_EXACT_V1,
        source_lineage=descriptor.source_lineage,
    )
    assert kernel.canonical_json_bytes(rebuilt) == kernel.canonical_json_bytes(
        descriptor
    )


def test_selected_kernel_reaches_accepted_raw_coverage_v2():
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))
    _, _, snapshot, request = _load_contracts()
    compiled = kernel.compile_reference_envelopes(snapshot, request)

    assert compiled.compilation is not None
    result = kernel.evaluate_reference_raw_coverage(
        compiled.compilation,
        request.requested_alpha,
    )
    raw = result.raw_coverage
    expected = manifest["expected_kernel_results"][SELECTED_SHA]

    assert result.outcome.value == expected["outcome"]
    assert raw is not None
    assert raw.schema_version == expected["raw_schema_version"]
    assert raw.semantic_digest == expected["semantic_digest"]
    assert raw.exact_area_expression == expected["exact_area_expression"]
    assert len(raw.boundary_vertex_occurrences) == expected[
        "boundary_occurrence_count"
    ]
    assert len(raw.point_contacts) == expected["point_contact_count"]
    assert len(raw.loops) == expected["loop_count"]
    assert len(raw.regions) == expected["region_count"]


def test_historical_failure_and_selected_contacts_are_the_same_exact_topology():
    historical = json.loads(
        (EVIDENCE / "historical_df587ed_result.json").read_text(encoding="utf-8")
    )
    selected = json.loads(
        (EVIDENCE / "selected_c262_result.json").read_text(encoding="utf-8")
    )
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))

    assert historical["kernel_revision"] == HISTORICAL_SHA
    assert historical["evaluation"]["stage"] == "RAW_REJECTED"
    assert historical["evaluation"]["outcome"] == manifest[
        "expected_kernel_results"
    ][HISTORICAL_SHA]["outcome"]
    assert historical["evaluation"]["diagnostics"][0]["message"] == (
        "exact union boundary is non-manifold at a construction vertex"
    )
    assert selected["kernel_revision"] == SELECTED_SHA
    assert selected["evaluation"]["stage"] == "RAW_READY"
    assert _point_topology_from_historical(historical) == (
        _point_topology_from_selected(selected)
    )
