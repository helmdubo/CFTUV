#!/usr/bin/env python3
"""Воспроизводимая валидация текстового/JSON-корпуса EC0-P."""

from __future__ import annotations

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
CORPUS = REPO / "artifacts" / "envelope_ec0" / "corpus"
CASE_DIR = CORPUS / "cases"
CANONICAL_AUTHORITY = {
    "USER_REQUIRED",
    "FIELD_PROVEN",
    "MATHEMATICALLY_REQUIRED",
}
REVIEW_ONLY_AUTHORITY = {
    "LEGACY_COMPATIBILITY",
    "IMPLEMENTATION_ACCIDENT",
    "OPEN_RESEARCH",
}
AM3_REQUIRED_TRANSFORMS = {
    "chain_storage_reverse_preserve_semantics",
    "surface_winding_reverse_preserve_domain",
    "rigid_translation",
    "coherent_uniform_scale",
    "source_retriangulation",
    "data_chain_split_merge",
    "simultaneous_event_permutation",
    "small_perturbation_without_topology_change",
}
FORBIDDEN_TOP_LEVEL = {"skeleton", "envelopes", "region_graph", "visuals", "pivot_digest_projection"}
RUNTIME_SNAPSHOT_KEYS = {
    "seeds",
    "front_components",
    "coverage_contributions",
    "active_interval_model",
    "boundary_events",
    "capacity_states",
    "effective_alpha",
    "topology_events",
    "interactions",
}
COORDINATE_KEYS = {"x", "y", "z", "point", "points", "vector", "vectors", "coordinate", "coordinates", "tolerance", "epsilon"}


class ValidationError(RuntimeError):
    pass


def fail(case_id: str, message: str) -> None:
    raise ValidationError(f"{case_id}: {message}")


def read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValidationError(f"{path.relative_to(REPO)}: invalid JSON: {exc}") from exc


def walk_keys(value, path=""):
    if isinstance(value, dict):
        for key, child in value.items():
            yield key, f"{path}.{key}" if path else key
            yield from walk_keys(child, f"{path}.{key}" if path else key)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from walk_keys(child, f"{path}[{index}]")


def index(records, case_id, label):
    result = {}
    for record in records:
        identifier = record.get("id")
        if not identifier:
            fail(case_id, f"{label} record has no id")
        if identifier in result:
            fail(case_id, f"duplicate {label} id {identifier}")
        result[identifier] = record
    return result


def validate_case(case, manifest_contract_ids):
    case_id = case.get("case_id", "<missing-case-id>")
    required = {
        "schema", "case_id", "slug", "title", "status", "global_contract_ids",
        "semantic_contracts", "analysis_snapshot", "decal_request",
        "expected_compiled_plan", "acceptance", "metamorphic_expectations",
        "canonical_geometry_digest",
    }
    if set(case) != required:
        fail(case_id, f"top-level keys differ: missing={sorted(required-set(case))}, extra={sorted(set(case)-required)}")
    if case["schema"] != "cftuv.envelope.ec0.case.v3":
        fail(case_id, "wrong schema")
    if case["status"] not in {"DEFINED", "UNSUPPORTED_NAMED_FAILURE", "BLOCKED_PENDING_USER_DECISION"}:
        fail(case_id, "invalid status")
    if set(case["global_contract_ids"]) != manifest_contract_ids:
        fail(case_id, "global contract references do not match manifest")
    for item in case["semantic_contracts"]:
        if item.get("authority") not in CANONICAL_AUTHORITY or not item.get("rule"):
            fail(case_id, f"invalid SemanticAuthority record {item}")
    if FORBIDDEN_TOP_LEVEL & set(case):
        fail(case_id, "parallel v1 semantic graph is forbidden")

    for key, key_path in walk_keys(case):
        if key.lower() in COORDINATE_KEYS:
            fail(case_id, f"coordinate/tolerance key is forbidden: {key_path}")
        if key in {"visuals", "svg", "png", "html_presentation"}:
            fail(case_id, f"presentation visual field is forbidden: {key_path}")

    snapshot = case["analysis_snapshot"]
    snapshot_keys = {key for key, _ in walk_keys(snapshot)}
    leak = RUNTIME_SNAPSHOT_KEYS & snapshot_keys
    if leak:
        fail(case_id, f"kernel-owned runtime state leaked into AnalysisSnapshot: {sorted(leak)}")

    patches = index(snapshot.get("patches", []), case_id, "Patch")
    domains = index(snapshot.get("patch_domains", []), case_id, "PatchDomain")
    physical = index(snapshot.get("physical_chains", []), case_id, "PhysicalChain")
    uses = index(snapshot.get("chain_uses", []), case_id, "ChainUse")
    boundaries = index(snapshot.get("boundary_constraints", []), case_id, "BoundaryConstraint")
    vertices = index(snapshot.get("source_vertices", []), case_id, "SourceVertex")
    corners = index(snapshot.get("corner_relations", []), case_id, "CornerRelation")
    junctions = index(snapshot.get("junction_relations", []), case_id, "JunctionRelation")

    sector_records = {}
    sectors_by_use = defaultdict(list)
    for domain in domains.values():
        if domain.get("owner_patch_id") not in patches:
            fail(case_id, f"domain {domain['id']} references missing Patch")
        for boundary_id in domain.get("boundary_constraint_ids", []):
            if boundary_id not in boundaries:
                fail(case_id, f"domain {domain['id']} references missing boundary {boundary_id}")
        for sector in domain.get("sectors", []):
            sector_id = sector.get("id")
            if not sector_id or sector_id in sector_records:
                fail(case_id, f"invalid or duplicate sector id {sector_id}")
            if not sector.get("analysis_proven"):
                fail(case_id, f"sector {sector_id} lacks analysis provenance")
            if not sector.get("multiplicity_reason"):
                fail(case_id, f"sector {sector_id} lacks multiplicity reason")
            sector_records[sector_id] = sector
            sectors_by_use[sector.get("chain_use_id")].append(sector)

    for use in uses.values():
        if use.get("physical_chain_id") not in physical:
            fail(case_id, f"use {use['id']} references missing PhysicalChain")
        if use.get("owner_patch_id") not in patches:
            fail(case_id, f"use {use['id']} references missing Patch")
        if use.get("patch_domain_id") not in domains:
            fail(case_id, f"use {use['id']} references missing PatchDomain")
        launch = use.get("launch_locus", {})
        boundary = boundaries.get(launch.get("boundary_id"))
        if not boundary or boundary.get("constraint_kind") != "SOURCE_LAUNCH_BOUNDARY":
            fail(case_id, f"use {use['id']} has no SOURCE_LAUNCH_BOUNDARY")
        if not launch.get("source_support_non_blocking") or boundary.get("blocks_originating_seed") is not False:
            fail(case_id, f"use {use['id']} launch support blocks its own seed")
        if not sectors_by_use[use["id"]]:
            fail(case_id, f"use {use['id']} has no analysis-proven sector")
        if len(sectors_by_use[use["id"]]) > 1:
            reasons = {item["multiplicity_reason"] for item in sectors_by_use[use["id"]]}
            if reasons == {"SINGLE_OWNER_INTERIOR_SECTOR"} or "DEFAULT_LEFT_RIGHT" in reasons:
                fail(case_id, f"use {use['id']} has unproven automatic sector multiplicity")

    request = case["decal_request"]
    request_id = request.get("decal_request_id")
    selected_uses = set(request.get("selected_chain_use_ids", []))
    if not request_id or not selected_uses or not selected_uses <= set(uses):
        fail(case_id, "DecalRequest has invalid selected ChainUses")

    relation_incidents = {}
    for relation in corners.values():
        incident = relation.get("incident_chain_use_ids", [])
        if len(incident) < 2 or not set(incident) <= set(uses):
            fail(case_id, f"CornerRelation {relation['id']} must have at least two valid uses")
        if relation.get("source_vertex_id") not in vertices:
            fail(case_id, f"CornerRelation {relation['id']} references missing vertex")
        relation_incidents[relation["id"]] = set(incident)
    for relation in junctions.values():
        incident = relation.get("incident_chain_use_ids", [])
        if len(incident) < 2 or not set(incident) <= set(uses):
            fail(case_id, f"JunctionRelation {relation['id']} must have at least two valid uses")
        if relation.get("source_vertex_id") not in vertices:
            fail(case_id, f"JunctionRelation {relation['id']} references missing vertex")
        relation_incidents[relation["id"]] = set(incident)

    evaluations = case["expected_compiled_plan"].get("patch_evaluations", [])
    if not evaluations:
        fail(case_id, "CompiledPatchEvaluationPlan is empty")
    plan_keys = set()
    covered_uses = set()
    all_components_by_use = defaultdict(list)
    for plan in evaluations:
        plan_key = plan.get("plan_key", {})
        key = (plan_key.get("decal_request_id"), plan_key.get("patch_domain_id"))
        if key in plan_keys:
            fail(case_id, f"duplicate evaluation key {key}")
        plan_keys.add(key)
        if key[0] != request_id or key[1] not in domains:
            fail(case_id, f"invalid evaluation key {key}")
        domain_id = key[1]
        owner_patch = domains[domain_id]["owner_patch_id"]
        if plan.get("owner_patch_id") != owner_patch:
            fail(case_id, f"evaluation {plan.get('id')} owner Patch mismatch")

        seeds = index(plan.get("seeds", []), case_id, "Seed")
        components = index(plan.get("front_components", []), case_id, "FrontComponent")
        contributions = index(plan.get("coverage_contributions", []), case_id, "Contribution")
        if set(plan.get("source_seed_ids", [])) != set(seeds):
            fail(case_id, f"evaluation {plan.get('id')} source_seed_ids mismatch")

        plan_use_ids = {use_id for use_id, use in uses.items() if use["patch_domain_id"] == domain_id and use_id in selected_uses}
        covered_uses |= plan_use_ids
        for seed in seeds.values():
            kind = seed.get("kind")
            if kind == "FrontSeed":
                use_id = seed.get("chain_use_id")
                if use_id not in plan_use_ids:
                    fail(case_id, f"FrontSeed {seed['id']} is outside evaluation domain")
                if set(seed.get("sector_ids", [])) != {item["id"] for item in sectors_by_use[use_id]}:
                    fail(case_id, f"FrontSeed {seed['id']} sector set mismatch")
            elif kind == "CornerSeed":
                relation_id = seed.get("source_relation_id")
                incident = set(seed.get("incident_chain_use_ids", []))
                if relation_id not in corners or incident != relation_incidents[relation_id]:
                    fail(case_id, f"CornerSeed {seed['id']} incident uses differ from relation")
            elif kind == "JunctionSeed":
                relation_id = seed.get("source_relation_id")
                incident = set(seed.get("incident_chain_use_ids", []))
                if relation_id not in junctions:
                    fail(case_id, f"JunctionSeed {seed['id']} references missing relation")
                if seed.get("projection_role") == "PER_PATCH_PROJECTION":
                    if not incident or not incident <= relation_incidents[relation_id]:
                        fail(case_id, f"Junction projection {seed['id']} has invalid incident subset")
                elif incident != relation_incidents[relation_id]:
                    fail(case_id, f"JunctionSeed {seed['id']} incident uses differ from relation")
            elif kind not in {"CapSeed", "EndpointClaimSeed"}:
                fail(case_id, f"unsupported seed kind {kind}")

        for component in components.values():
            use_id = component.get("chain_use_id")
            sector_id = component.get("sector_id")
            if use_id not in plan_use_ids or sector_id not in sector_records:
                fail(case_id, f"component {component['id']} has invalid use/sector")
            if sector_records[sector_id]["chain_use_id"] != use_id:
                fail(case_id, f"component {component['id']} sector belongs to another use")
            if component.get("branch_count_policy") != "NON_INCREASING" or component.get("initial_branch_count") != 1:
                fail(case_id, f"component {component['id']} violates branch-count policy")
            all_components_by_use[use_id].append(component["id"])

        component_ids = set(components)
        capacity_ids = {state.get("front_component_id") for state in plan.get("capacity_states", [])}
        if capacity_ids != component_ids:
            fail(case_id, f"evaluation {plan.get('id')} capacity states are incomplete")
        for contribution in contributions.values():
            if contribution.get("source_seed_id") not in seeds:
                fail(case_id, f"contribution {contribution['id']} references missing seed")
            if contribution.get("decal_request_id") != request_id or contribution.get("patch_domain_id") != domain_id:
                fail(case_id, f"contribution {contribution['id']} request/domain mismatch")
            if not set(contribution.get("front_component_ids", [])) <= component_ids:
                fail(case_id, f"contribution {contribution['id']} references missing component")

        coverage = plan.get("patch_coverage", {})
        if coverage.get("decal_request_id") != request_id or coverage.get("patch_domain_id") != domain_id:
            fail(case_id, "PatchCoverage request/domain mismatch")
        if coverage.get("operation") != "EXACT_SINGLE_COVER_UNION" or coverage.get("reachability") != "PROVEN_FROM_SOURCE_SEEDS":
            fail(case_id, "PatchCoverage does not prove exact reachable single cover")
        if set(coverage.get("contribution_ids", [])) != set(contributions):
            fail(case_id, "PatchCoverage contribution set mismatch")
        if plan.get("ownership", {}).get("coverage_id") != coverage.get("id"):
            fail(case_id, "ownership references another coverage")
        provenance = plan.get("geometry_batch_provenance", {})
        if provenance.get("decal_request_id") != request_id or provenance.get("patch_domain_id") != domain_id:
            fail(case_id, "GeometryBatch provenance lacks request/domain key")
        for interaction in plan.get("interactions", []):
            if interaction.get("decal_request_id") != request_id or interaction.get("patch_domain_id") != domain_id:
                fail(case_id, f"interaction {interaction.get('id')} request/domain mismatch")
            if not set(interaction.get("participant_contribution_ids", [])) <= set(contributions):
                fail(case_id, f"interaction {interaction.get('id')} references missing contribution")

    if covered_uses != selected_uses:
        fail(case_id, f"selected uses are not covered exactly once by request/domain plans: {selected_uses-covered_uses}")
    for use_id in selected_uses:
        expected = len(sectors_by_use[use_id])
        actual = len(all_components_by_use[use_id])
        if actual != expected:
            fail(case_id, f"ChainUse {use_id} has {actual} components for {expected} proven sectors")

    # Review-specific regression assertions.
    if case_id == "EC0-C12":
        plan = evaluations[0]
        claim_b = [
            item for item in plan["coverage_contributions"]
            if item.get("source_seed_id") == "corner_b_seed"
        ]
        owner_claim_b = [
            item for item in plan["ownership"]["claims"]
            if item.get("id") == "claim_b_owner"
        ]
        if (
            len(claim_b) != 1
            or claim_b[0]["id"] != "corner_b_contribution"
            or claim_b[0]["kind"] != "CornerEnvelope"
            or len(owner_claim_b) != 1
            or owner_claim_b[0].get("contribution_id") != "corner_b_contribution"
        ):
            fail(case_id, "claim B must have exactly one CornerEnvelope identity")
    if case_id == "EC0-C13":
        seed = next(item for item in evaluations[0]["seeds"] if item["id"] == "bevel_corner_seed")
        if set(seed["incident_chain_use_ids"]) != {"bevel_in_use", "bevel_out_use"}:
            fail(case_id, "other_wing_use leaked into BEVEL CornerSeed")
    if case_id == "EC0-C11":
        endpoint_claims = [
            item for item in evaluations[0]["seeds"]
            if item["kind"] == "EndpointClaimSeed"
        ]
        if any(item["kind"] == "CornerSeed" for item in evaluations[0]["seeds"]):
            fail(case_id, "short endpoint claims must not be CornerSeed")
        if len(endpoint_claims) != 2:
            fail(case_id, "short segment must have two EndpointClaimSeeds")
    if case_id == "EC0-C16":
        interactions = evaluations[0]["interactions"]
        if (
            len(interactions) != 1
            or interactions[0].get("kind") != "INTRAPATCH_WING_COVERAGE_CLIP"
            or interactions[0].get("coverage_effect")
            != "CLIP_EXISTING_CONTRIBUTIONS_AT_EQUALITY_LOCUS"
            or interactions[0].get("creates_new_matter") is not False
        ):
            fail(case_id, "case 16 must encode selected same-request policy B")
    if case_id == "EC0-P05":
        if len(junctions) != 1 or len(evaluations) < 2:
            fail(case_id, "cross-Patch junction needs one global relation and multiple Patch plans")
        relation = next(iter(junctions.values()))
        anchor = relation.get("shared_semantic_anchor_id")
        if not anchor or relation.get("relation_kind") != "CROSS_PATCH":
            fail(case_id, "cross-Patch junction lacks shared anchor")
        for plan in evaluations:
            projections = [
                item for item in plan["coverage_contributions"]
                if item["kind"] == "JunctionEnvelope"
            ]
            if (
                len(projections) != 1
                or projections[0].get("shared_semantic_anchor_id") != anchor
                or plan.get("interactions")
            ):
                fail(case_id, "cross-Patch projection lost shared anchor or added collision")
    if case_id in {"EC0-P06", "EC0-P07"}:
        shared = [item for plan in evaluations for item in plan["coverage_contributions"] if item["kind"] in {"CornerEnvelope", "JunctionEnvelope"}]
        if len(shared) != 1 or "incident_effective_alpha_values" not in shared[0] or shared[0].get("mixed_alpha_failure") != "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN":
            fail(case_id, "mixed-alpha shared envelope policy is incomplete")


def validate_matrices(main_ids, pivot_ids):
    specs = [
        (MATRIX := CORPUS / "matrices" / "metamorphic_matrix.json", "cases", "by_case", main_ids),
        (CORPUS / "matrices" / "pivot_metamorphic_matrix.json", "cases", "by_case", pivot_ids),
    ]
    for path, cases_key, verdict_key, expected_ids in specs:
        data = read_json(path)
        ids = set(data.get(cases_key, []))
        if ids != expected_ids:
            raise ValidationError(f"{path.relative_to(REPO)}: case set mismatch")
        transforms = {row.get("transform") for row in data.get("verdicts", [])}
        if not AM3_REQUIRED_TRANSFORMS <= transforms:
            missing = sorted(AM3_REQUIRED_TRANSFORMS - transforms)
            raise ValidationError(f"{path.relative_to(REPO)}: missing AM3 transforms {missing}")
        for row in data.get("verdicts", []):
            verdicts = row.get(verdict_key, {})
            if set(verdicts) != ids or not set(verdicts.values()) <= {"INVARIANT", "SEMANTIC_CHANGE"}:
                raise ValidationError(f"{path.relative_to(REPO)}: invalid row {row.get('transform')}")
    boundary = read_json(CORPUS / "policies" / "boundary_limited_propagation.json")
    boundary_matrix = read_json(CORPUS / "matrices" / "boundary_metamorphic_matrix.json")
    scenario_ids = {item["id"] for item in boundary.get("scenarios", [])}
    if len(scenario_ids) != 10 or scenario_ids != set(boundary_matrix.get("scenarios", [])):
        raise ValidationError("boundary policy/matrix scenario mismatch")
    by_id = {item["id"]: item for item in boundary["scenarios"]}
    if by_id["BLP-04"]["contact_topology"] != "ENDPOINT_CONTACT" or by_id["BLP-05"]["contact_topology"] != "INTERIOR_CONTACT":
        raise ValidationError("hole endpoint/interior contact is not separated")
    if by_id["BLP-05"]["exact_event"] != "BARRIER_SPLIT_REQUIRED":
        raise ValidationError("interior hole contact must saturate at exact contact")


def main() -> int:
    manifest = read_json(CORPUS / "manifest.json")
    if manifest.get("status") != "CORRECTION_CANDIDATE_READY_FOR_EXTERNAL_REVIEW":
        raise ValidationError("manifest has unexpected review status")
    if manifest.get("canonical_format") != "JSON":
        raise ValidationError("manifest canonical format must be JSON")
    if manifest.get("presentation_visual_policy") != "FORBIDDEN":
        raise ValidationError("manifest must forbid presentation visuals")
    allowed_visuals = {
        "BLENDER_VIEWPORT_SCREENSHOT",
        "BLENDER_UV_EDITOR_SCREENSHOT",
        "BLENDER_DEBUG_OVERLAY_SCREENSHOT",
    }
    if set(manifest.get("allowed_visual_evidence", [])) != allowed_visuals:
        raise ValidationError("only Blender diagnostic screenshots may be visual evidence")
    expected_policy_files = {
        str(CORPUS / "policies" / "boundary_limited_propagation.json")
    }
    expected_matrix_files = {
        str(CORPUS / "matrices" / "metamorphic_matrix.json"),
        str(CORPUS / "matrices" / "pivot_metamorphic_matrix.json"),
        str(CORPUS / "matrices" / "boundary_metamorphic_matrix.json"),
    }
    policy_files = {str((REPO / item).resolve()) for item in manifest.get("policy_files", [])}
    matrix_files = {str((REPO / item).resolve()) for item in manifest.get("matrix_files", [])}
    if policy_files != {str(Path(item).resolve()) for item in expected_policy_files}:
        raise ValidationError("manifest policy inventory differs from canonical policy")
    if matrix_files != {str(Path(item).resolve()) for item in expected_matrix_files}:
        raise ValidationError("manifest matrix inventory differs from canonical matrices")
    schema = read_json(CORPUS / "schema" / "case.schema.json")
    if schema.get("$id") != "cftuv.envelope.ec0.case.v3":
        raise ValidationError("case JSON Schema id mismatch")
    global_contracts = manifest.get("global_semantic_contracts", [])
    global_ids = {item.get("id") for item in global_contracts}
    if len(global_ids) != len(global_contracts) or any(item.get("authority") not in CANONICAL_AUTHORITY for item in global_contracts):
        raise ValidationError("manifest has invalid global SemanticAuthority records")
    decision_record = read_json(CORPUS / "decision_record.json")
    decisions = {item.get("id"): item for item in decision_record.get("decisions", [])}
    if decisions.get("AM4", {}).get("selected") != "B_COVERAGE_CLIP":
        raise ValidationError("decision record lost user-selected case 16 policy B")
    if decisions.get("VISUAL_POLICY", {}).get("selected") != "TEXT_JSON_VALIDATOR":
        raise ValidationError("decision record lost text/JSON artifact policy")
    review_groups = decision_record.get("review_only_not_semantic", [])
    if {item.get("authority") for item in review_groups} != REVIEW_ONLY_AUTHORITY:
        raise ValidationError("decision record must enumerate all review-only authorities")
    if any(not item.get("items") for item in review_groups):
        raise ValidationError("decision record has an empty review-only list")
    case_paths = sorted(CASE_DIR.glob("*.json"))
    if len(case_paths) != 23:
        raise ValidationError(f"expected 23 cases, found {len(case_paths)}")
    manifest_paths = {str((REPO / item).resolve()) for item in manifest.get("case_files", [])}
    if manifest_paths != {str(path.resolve()) for path in case_paths}:
        raise ValidationError("manifest case inventory differs from filesystem")

    cases = [read_json(path) for path in case_paths]
    blocked = [case.get("case_id") for case in cases if case.get("status") == "BLOCKED_PENDING_USER_DECISION"]
    if blocked:
        raise ValidationError(f"unresolved user decisions remain: {blocked}")
    counts = Counter(case.get("case_id") for case in cases)
    duplicates = [case_id for case_id, count in counts.items() if count != 1]
    if duplicates:
        raise ValidationError(f"duplicate case ids: {duplicates}")
    for case in cases:
        validate_case(case, global_ids)

    main_ids = {case["case_id"] for case in cases if case["case_id"].startswith("EC0-C")}
    pivot_ids = {case["case_id"] for case in cases if case["case_id"].startswith("EC0-P")}
    if len(main_ids) != 16 or len(pivot_ids) != 7:
        raise ValidationError(f"expected 16 main and 7 pivot cases, found {len(main_ids)} and {len(pivot_ids)}")
    validate_matrices(main_ids, pivot_ids)

    presentation_files = []
    for suffix in ("*.svg", "*.png", "*.html"):
        presentation_files.extend((REPO / "artifacts" / "envelope_ec0").rglob(suffix))
    if presentation_files:
        joined = ", ".join(str(path.relative_to(REPO)) for path in presentation_files[:5])
        raise ValidationError(f"presentation visuals remain in active EC0 artifacts: {joined}")

    print("EC0 corpus validation OK")
    print(f"  canonical JSON cases: {len(cases)} (main={len(main_ids)}, pivot={len(pivot_ids)})")
    print("  ChainUse/sector/FrontComponent cardinality: OK")
    print("  AnalysisSnapshot / DecalRequest / CompiledPlan separation: OK")
    print("  request/domain provenance and canonical ID references: OK")
    print("  review regressions C11/C12/C13, cross-Patch junction, mixed alpha: OK")
    print("  legacy/implementation-accident/open-research review lists: explicit")
    print("  presentation artifacts: none; Blender screenshots remain policy-allowed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValidationError as exc:
        print(f"EC0 corpus validation FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1)
