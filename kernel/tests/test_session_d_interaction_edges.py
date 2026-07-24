from __future__ import annotations

from dataclasses import replace
from itertools import permutations
import json
from pathlib import Path

import cftuv_envelope as kernel
import sympy as sp

from cftuv_envelope import (
    ArrivalModelId,
    ArrivalModelKind,
    CapArrivalModelV1,
    DecalRequestId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    ExactFrontArrivalLawV1,
    FrontComponentId,
    FrontReadingId,
    InteractionCandidateKind,
    InteractionCandidateV1,
    InteractionComponentV1,
    InteractionComponentId,
    InteractionRelationId,
    InteractionOutcome,
    LawId,
    LineageId,
    PatchDomainId,
    ReachabilityCertificateV1,
    SameAlphaInteractionBatchId,
    SourceSupportId,
    StripArrivalModelV1,
    UnsupportedJunctionArrivalModelV1,
    generate_interaction_candidates,
    prove_mutual_arrivals,
)
from cftuv_envelope.interactions.arrival import (
    _select_cap_incident_reading,
)
from cftuv_envelope.interactions.equality_locus import (
    clip_equality_locus_to_active_domains,
)
from cftuv_envelope.interactions.policy_b import apply_policy_b
from cftuv_envelope.interactions.policy_b import _component_intersection_area
from cftuv_envelope.interactions.contracts import (
    ATOMIC_OWNERLESS_POINT_CONTACT_V1,
    InteractionCoverageEffect,
)
from cftuv_envelope.interactions.provenance import InteractionProvenanceV1
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import (
    GeometryContext,
    make_region,
    make_segment,
    stable_id,
)
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
)
from cftuv_envelope.reference.provenance import make_reference_provenance
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from session_d_factories import opposing_strip_state, self_contact_state


def _rectangle(
    name: str,
    x0,
    y0,
    x1,
    y1,
    *,
    hole: tuple | None = None,
) -> PlanarRegion:
    provenance = make_reference_provenance(
        envelope_spec_ids=frozenset({f"spec:{name}"}),
        envelope_instance_ids=frozenset({f"instance:{name}"}),
        patch_domain_ids=frozenset({"domain"}),
        lineage_ids=frozenset({f"lineage:{name}"}),
    )
    certificate = ConstructionCertificate(
        ConstructionKind.EVENT_ANCHOR,
        event_key=f"fixture:{name}",
    )

    def loop(loop_name, coordinates):
        points = tuple(
            ExactPlanarPoint.from_values(x, y) for x, y in coordinates
        )
        return PlanarLoop(
            loop_name,
            tuple(
                make_segment(
                    f"{loop_name}:{index}",
                    start,
                    end,
                    support_ids=frozenset({f"support:{name}"}),
                    provenance=provenance,
                    start_certificates=frozenset({certificate}),
                    end_certificates=frozenset({certificate}),
                )
                for index, (start, end) in enumerate(
                    zip(points, points[1:] + points[:1])
                )
            ),
        )

    outer = loop(
        f"outer:{name}",
        ((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
    )
    holes = ()
    if hole is not None:
        hx0, hy0, hx1, hy1 = hole
        holes = (
            loop(
                f"hole:{name}",
                ((hx0, hy0), (hx0, hy1), (hx1, hy1), (hx1, hy0)),
            ),
        )
    return PlanarRegion(
        f"region:{name}",
        outer,
        holes,
        frozenset({f"instance:{name}"}),
        frozenset({f"spec:{name}"}),
    )


def _component(name: str, *, request="request", domain="domain"):
    return InteractionComponentV1(
        interaction_component_id=InteractionComponentId(
            f"component:{name}"
        ),
        decal_request_id=DecalRequestId(request),
        patch_domain_id=PatchDomainId(domain),
        envelope_spec_ids=frozenset(
            {EnvelopeSpecId(f"spec:{name}")}
        ),
        envelope_instance_ids=frozenset(
            {EnvelopeInstanceId(f"instance:{name}")}
        ),
        front_component_ids=frozenset(
            {FrontComponentId(f"front:{name}")}
        ),
        relation_ids=frozenset(),
        source_lineage_ids=frozenset(
            {LineageId(f"lineage:{name}")}
        ),
    )


def _strip_model(
    name: str,
    region: PlanarRegion,
    normal: tuple,
    constant,
    effective=6,
):
    normal_value = ExactPlanarVector.from_values(*normal)
    law = ExactFrontArrivalLawV1(
        law_id=LawId(f"law:{name}"),
        support_id=SourceSupportId(f"support:{name}"),
        normal=normal_value,
        source_constant=ExactScalar.from_value(constant),
        normal_speed=ExactScalar.from_value(1),
    )
    front = region.outer.segments[2:3]
    return StripArrivalModelV1(
        arrival_model_id=ArrivalModelId(f"model:{name}"),
        model_kind=ArrivalModelKind.STRIP,
        interaction_component_id=InteractionComponentId(
            f"component:{name}"
        ),
        decal_request_id=DecalRequestId("request"),
        patch_domain_id=PatchDomainId("domain"),
        envelope_spec_id=EnvelopeSpecId(f"spec:{name}"),
        envelope_instance_id=EnvelopeInstanceId(
            f"instance:{name}"
        ),
        front_component_id=FrontComponentId(f"front:{name}"),
        front_reading_id=FrontReadingId(f"reading:{name}"),
        source_lineage_ids=frozenset(
            {LineageId(f"lineage:{name}")}
        ),
        arrival_law=law,
        active_segments=front,
        active_regions=(region,),
        effective_alpha=ExactScalar.from_value(effective),
        reachability=ReachabilityCertificateV1(
            frozenset({f"front:{name}"}),
            True,
            1,
            1,
            (),
            False,
        ),
    )


def test_same_geometry_different_request_or_patch_has_no_candidate():
    left = _component("left")
    assert generate_interaction_candidates(
        (
            left,
            replace(
                _component("right"),
                decal_request_id=DecalRequestId("other"),
            ),
        ),
        (),
    ) == ()
    assert generate_interaction_candidates(
        (
            left,
            replace(
                _component("right"),
                patch_domain_id=PatchDomainId("other-domain"),
            ),
        ),
        (),
    ) == ()


def test_equality_locus_terminates_exactly_at_hole():
    left = _rectangle(
        "left-hole", 0, 0, 6, 10, hole=(sp.Rational(9, 2), 4, sp.Rational(11, 2), 6)
    )
    right = _rectangle(
        "right-hole", 4, 0, 10, 10, hole=(sp.Rational(9, 2), 4, sp.Rational(11, 2), 6)
    )
    left_law = ExactFrontArrivalLawV1(
        LawId("left-law"),
        SourceSupportId("left-support"),
        ExactPlanarVector.from_values(1, 0),
        ExactScalar.from_value(0),
        ExactScalar.from_value(1),
    )
    right_law = ExactFrontArrivalLawV1(
        LawId("right-law"),
        SourceSupportId("right-support"),
        ExactPlanarVector.from_values(-1, 0),
        ExactScalar.from_value(-10),
        ExactScalar.from_value(1),
    )
    locus = clip_equality_locus_to_active_domains(
        left_law, right_law, (left,), (right,)
    )
    assert locus is not None
    assert len(locus.segments) == 2
    endpoints = {
        point.y.as_expr()
        for segment in locus.segments
        for point in segment
    }
    assert endpoints == {0, 4, 6, 10}


def test_one_sided_arrival_and_saturated_component_do_not_interact():
    left_region = _rectangle("left", 0, 0, 6, 2)
    right_region = _rectangle("right", 4, 0, 10, 2)
    left = _strip_model("left", left_region, (1, 0), 0, effective=4)
    right = _strip_model("right", right_region, (-1, 0), -10)
    components = (_component("left"), _component("right"))
    candidates = generate_interaction_candidates(components, (left, right))
    proofs, diagnostics = prove_mutual_arrivals(
        candidates, (left, right)
    )
    assert proofs == ()
    assert diagnostics == ()
    assert left.effective_alpha.as_expr() == 4
    assert right.effective_alpha.as_expr() == 6


def test_two_independent_same_alpha_interactions_are_atomic_not_multiway():
    regions = {
        "a": _rectangle("a", 0, 0, 6, 2),
        "b": _rectangle("b", 4, 0, 10, 2),
        "c": _rectangle("c", 0, 10, 6, 12),
        "d": _rectangle("d", 4, 10, 10, 12),
    }
    models = (
        _strip_model("a", regions["a"], (1, 0), 0),
        _strip_model("b", regions["b"], (-1, 0), -10),
        _strip_model("c", regions["c"], (1, 0), 0),
        _strip_model("d", regions["d"], (-1, 0), -10),
    )
    components = tuple(_component(name) for name in "abcd")
    proofs, diagnostics = prove_mutual_arrivals(
        generate_interaction_candidates(components, models),
        models,
    )
    assert len(proofs) == 2
    assert diagnostics == ()
    assert {
        proof.mutual_arrival_certificate.same_alpha_batch_identity
        for proof in proofs
    } == {
        SameAlphaInteractionBatchId(
            stable_id(
                "same-alpha-interaction-batch",
                DecalRequestId("request"),
                PatchDomainId("domain"),
                ExactScalar.from_value(5).expression,
            )
        )
    }


def test_triple_meeting_fails_named_without_pair_order():
    regions = {
        "a": _rectangle("a", 0, 0, 6, 2),
        "b": _rectangle("b", 4, 0, 10, 2),
        "c": _rectangle("c", 4, -5, 6, 1),
    }
    models = (
        _strip_model("a", regions["a"], (1, 0), 0),
        _strip_model("b", regions["b"], (-1, 0), -10),
        _strip_model("c", regions["c"], (0, 1), -5),
    )
    components = tuple(_component(name) for name in "abc")
    proofs, diagnostics = prove_mutual_arrivals(
        generate_interaction_candidates(components, models),
        models,
    )
    assert proofs == ()
    assert {
        item.outcome for item in diagnostics
    } == {InteractionOutcome.MULTIWAY_INTERACTION_POLICY_UNPROVEN}


def test_coincident_fronts_fail_named_and_never_choose_first():
    region = _rectangle("same", 0, 0, 6, 2)
    left = _strip_model("left", region, (1, 0), 0)
    right = replace(
        _strip_model("right", region, (-1, 0), -10),
        arrival_law=left.arrival_law,
    )
    components = (_component("left"), _component("right"))
    proofs, diagnostics = prove_mutual_arrivals(
        generate_interaction_candidates(components, (left, right)),
        (left, right),
    )
    assert proofs == ()
    assert {
        item.outcome for item in diagnostics
    } == {
        InteractionOutcome.INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN
    }


def test_cap_arrival_inherits_strip_law_and_is_endpoint_bounded():
    left_region = _rectangle("left", 0, 0, 6, 2)
    right_region = _rectangle("right", 4, 0, 10, 2)
    left_strip = _strip_model("left", left_region, (1, 0), 0)
    right = _strip_model("right", right_region, (-1, 0), -10)
    cap = CapArrivalModelV1(
        arrival_model_id=ArrivalModelId("model:left-cap"),
        model_kind=ArrivalModelKind.CAP,
        interaction_component_id=left_strip.interaction_component_id,
        decal_request_id=DecalRequestId("request"),
        patch_domain_id=PatchDomainId("domain"),
        envelope_spec_id=EnvelopeSpecId("spec:left-cap"),
        envelope_instance_id=EnvelopeInstanceId("instance:left-cap"),
        incident_strip_spec_id=left_strip.envelope_spec_id,
        front_component_id=left_strip.front_component_id,
        front_reading_id=left_strip.front_reading_id,
        source_lineage_ids=left_strip.source_lineage_ids,
        arrival_law=left_strip.arrival_law,
        active_segments=(left_region.outer.segments[0],),
        active_regions=left_strip.active_regions,
        effective_alpha=left_strip.effective_alpha,
        reachability=left_strip.reachability,
        inherits_incident_strip_reading=True,
    )
    candidate = generate_interaction_candidates(
        (_component("left"), _component("right")), (cap, right)
    )
    proofs, diagnostics = prove_mutual_arrivals(candidate, (cap, right))
    assert diagnostics == ()
    assert len(proofs) == 1
    assert proofs[0].mutual_arrival_certificate.exact_alpha.as_expr() == 5
    assert proofs[0].clipped_locus.segments == ()
    assert len(proofs[0].clipped_locus.anchors) == 1


def test_cap_incident_reading_is_exactly_one_or_fails_named():
    region = _rectangle("cap-source", 0, 0, 6, 2)
    strip = _strip_model("cap-source", region, (1, 0), 0)
    selected, diagnostic = _select_cap_incident_reading(
        cap_spec_id=EnvelopeSpecId("spec:cap"),
        component_id=strip.interaction_component_id,
        source_support_id=strip.arrival_law.support_id,
        incident_models=(strip,),
    )
    assert selected is strip
    assert diagnostic is None

    selected, diagnostic = _select_cap_incident_reading(
        cap_spec_id=EnvelopeSpecId("spec:cap"),
        component_id=strip.interaction_component_id,
        source_support_id=SourceSupportId("support:absent"),
        incident_models=(strip,),
    )
    assert selected is None
    assert diagnostic is not None
    assert (
        diagnostic.outcome
        is InteractionOutcome.INTERACTION_MISSING_FRONT_READING
    )

    duplicate = replace(
        strip,
        arrival_model_id=ArrivalModelId("model:cap-source:duplicate"),
        front_reading_id=FrontReadingId("reading:cap-source:duplicate"),
    )
    selected, diagnostic = _select_cap_incident_reading(
        cap_spec_id=EnvelopeSpecId("spec:cap"),
        component_id=strip.interaction_component_id,
        source_support_id=strip.arrival_law.support_id,
        incident_models=(strip, duplicate),
    )
    assert selected is None
    assert diagnostic is not None
    assert (
        diagnostic.outcome
        is InteractionOutcome.INTERACTION_CAP_READING_AMBIGUOUS
    )


def test_equivalent_locus_with_distinct_reading_provenance_fails_named():
    left_region = _rectangle("equivalent-left", 0, 0, 6, 2)
    right_region = _rectangle("equivalent-right", 4, 0, 10, 2)
    left = _strip_model("left", left_region, (1, 0), 0)
    duplicate = replace(
        left,
        arrival_model_id=ArrivalModelId("model:left:other-provenance"),
        front_reading_id=FrontReadingId("reading:left:other-provenance"),
    )
    right = _strip_model("right", right_region, (-1, 0), -10)
    models = (left, duplicate, right)
    candidates = generate_interaction_candidates(
        (_component("left"), _component("right")),
        models,
    )
    proofs, diagnostics = prove_mutual_arrivals(candidates, models)
    assert proofs == ()
    assert {
        item.outcome for item in diagnostics
    } == {
        InteractionOutcome.INTERACTION_EQUIVALENT_LOCUS_PROVENANCE_UNPROVEN
    }


def _unsupported_junction(component_name: str):
    return UnsupportedJunctionArrivalModelV1(
        arrival_model_id=ArrivalModelId(
            f"model:{component_name}:unsupported-junction"
        ),
        model_kind=ArrivalModelKind.UNSUPPORTED_JUNCTION,
        interaction_component_id=InteractionComponentId(
            f"component:{component_name}"
        ),
        decal_request_id=DecalRequestId("request"),
        patch_domain_id=PatchDomainId("domain"),
        envelope_spec_id=EnvelopeSpecId(
            f"spec:{component_name}:junction"
        ),
        envelope_instance_id=None,
        source_relation_id=InteractionRelationId(
            f"relation:{component_name}:junction"
        ),
        outcome=InteractionOutcome.INTERACTION_JUNCTION_ARRIVAL_UNPROVEN,
        message="fixture unsupported Junction law",
    )


def test_unsupported_junction_is_fatal_only_when_candidate_requires_it():
    left_region = _rectangle("junction-left", 0, 0, 6, 2)
    right_region = _rectangle("junction-right", 4, 0, 10, 2)
    left = _strip_model("left", left_region, (1, 0), 0)
    right = _strip_model("right", right_region, (-1, 0), -10)
    unsupported = _unsupported_junction("left")
    components = (_component("left"), _component("right"))
    candidates = generate_interaction_candidates(
        components,
        (left, right, unsupported),
    )
    proofs, diagnostics = prove_mutual_arrivals(
        candidates,
        (left, right, unsupported),
    )
    assert len(proofs) == 1
    assert diagnostics == ()

    candidates = generate_interaction_candidates(
        components,
        (right, unsupported),
    )
    proofs, diagnostics = prove_mutual_arrivals(
        candidates,
        (right, unsupported),
    )
    assert proofs == ()
    assert {
        item.outcome for item in diagnostics
    } == {InteractionOutcome.INTERACTION_JUNCTION_ARRIVAL_UNPROVEN}


def test_explicit_self_contact_preserves_two_readings_after_domain_clip():
    compilation, raw, result = self_contact_state("6")
    assert result.outcome is InteractionOutcome.EXACT
    assert len(compilation.front_reading_declarations) == 2
    assert len(compilation.self_contact_pair_declarations) == 1
    components_by_id = {
        item.front_component_id: item
        for item in compilation.front_components
    }
    for reading in compilation.front_reading_declarations:
        component = components_by_id[reading.front_component_id]
        assert reading.chain_use_id == component.chain_use_id
        assert reading.owner_sector_id == component.sector_id
        assert reading.source_support_id.value
        assert reading.physical_interval.physical_edge_id.value
        assert reading.arrival_law_id.value
    pair = next(iter(compilation.self_contact_pair_declarations))
    assert pair.source_lineage_ids == frozenset(
        {LineageId("self-front-lineage")}
    )
    candidate = result.candidates[0]
    assert (
        candidate.candidate_kind
        is InteractionCandidateKind.EXPLICIT_SELF_CONTACT
    )
    assert candidate.participant_front_reading_ids is not None
    assert set(candidate.participant_front_reading_ids) == {
        item.front_reading_id
        for item in compilation.front_reading_declarations
    }
    resolved = result.resolved_coverage
    assert resolved is not None
    assert resolved.exact_area_expression == raw.exact_area_expression
    assert len(resolved.interaction_applications) == 1
    retained_readings = frozenset(
        reading
        for contribution in resolved.resolved_contributions
        for reading in contribution.retained_front_reading_ids
    )
    assert retained_readings == frozenset(
        item.front_reading_id
        for item in compilation.front_reading_declarations
    )


def test_public_self_contact_pipeline_is_exact_before_at_and_after():
    for alpha, expected_applications in (("4", 0), ("5", 1), ("6", 1)):
        compilation, raw, result = self_contact_state(alpha)
        assert result.outcome is InteractionOutcome.EXACT
        assert len(compilation.self_contact_pair_declarations) == 1
        resolved = result.resolved_coverage
        assert resolved is not None
        assert len(resolved.interaction_applications) == expected_applications
        assert resolved.exact_area_expression == raw.exact_area_expression


def test_internalized_angular_model_cannot_create_false_candidate():
    left = _component("angular")
    right = _component("third")
    third_region = _rectangle("third", 4, 0, 10, 2)
    third = _strip_model("third", third_region, (-1, 0), -10)
    candidates = generate_interaction_candidates((left, right), (third,))
    proofs, diagnostics = prove_mutual_arrivals(candidates, (third,))
    assert proofs == ()
    assert diagnostics == ()


def test_cross_patch_junction_projections_never_form_collision_candidate():
    projection_a = replace(
        _component("projection-a"),
        patch_domain_id=PatchDomainId("domain-a"),
    )
    projection_b = replace(
        _component("projection-b"),
        patch_domain_id=PatchDomainId("domain-b"),
    )
    assert generate_interaction_candidates(
        (projection_a, projection_b), ()
    ) == ()


_D_R2_FIXTURE = (
    Path(__file__).parents[1]
    / "fixtures"
    / "session_d_interactions_v1"
    / "d_r2_atomic_point_contact_cases.json"
)
_BUILDING_002_FIXTURE = (
    Path(__file__).parents[1]
    / "fixtures"
    / "building_002_point_contact_v1"
)


def _d_r2_anchor_count(hyperedge: dict) -> int:
    if "anchor_ids" in hyperedge:
        return len(hyperedge["anchor_ids"])
    return int("anchor" in hyperedge)


def _d_r2_hypergraph_is_connected(case: dict) -> bool:
    participants = frozenset(case["participants"])
    adjacency = {item: set() for item in participants}
    for hyperedge in case["hyperedges"]:
        edge_participants = tuple(hyperedge["participant_component_ids"])
        if len(edge_participants) < 2:
            return False
        if not set(edge_participants).issubset(participants):
            return False
        for left in edge_participants:
            adjacency[left].update(
                right for right in edge_participants if right != left
            )
    if not participants:
        return False
    visited = set()
    pending = [next(iter(participants))]
    while pending:
        current = pending.pop()
        if current in visited:
            continue
        visited.add(current)
        pending.extend(adjacency[current] - visited)
    return visited == set(participants)


def _d_r2_contract_projection(case: dict) -> dict:
    requested_alpha = sp.sympify(case["requested_alpha"])
    event_alpha = sp.sympify(case["event_alpha"])
    if requested_alpha < event_alpha:
        return {
            "outcome": InteractionOutcome.EXACT.value,
            "application_count": 0,
            "equality_anchor_count": 0,
            "coverage_effect": (
                InteractionCoverageEffect.NONE_BEFORE_MUTUAL_ARRIVAL.value
            ),
            "resolved_union_digest": case["raw_union_digest"],
        }

    point_only = all(
        hyperedge["locus_dimension"] == 0
        and _d_r2_anchor_count(hyperedge) > 0
        for hyperedge in case["hyperedges"]
    )
    interiors_disjoint = all(
        sp.sympify(value) == 0
        for value in case["pairwise_positive_area_intersections"]
    )
    admitted = (
        len(case["participants"]) >= 3
        and _d_r2_hypergraph_is_connected(case)
        and point_only
        and interiors_disjoint
        and case["point_contact_records_complete"]
        and case["provenance_complete"]
        and not case["ownership_required"]
    )
    if not admitted:
        return {
            "outcome": InteractionOutcome.MULTIWAY_INTERACTION_POLICY_UNPROVEN.value,
            "application_count": 0,
            "equality_anchor_count": 0,
            "resolved_union_digest": None,
        }
    return {
        "outcome": InteractionOutcome.EXACT.value,
        "application_count": len(case["hyperedges"]),
        "equality_anchor_count": sum(
            _d_r2_anchor_count(hyperedge)
            for hyperedge in case["hyperedges"]
        ),
        "equality_owner": "NONE",
        "coverage_effect": (
            InteractionCoverageEffect
            .ATOMIC_OWNERLESS_POINT_CONTACT_IDENTITY
            .value
        ),
        "resolved_union_digest": case["raw_union_digest"],
    }


def test_d_r2_contract_fixture_before_at_after_and_unsupported_boundary():
    payload = json.loads(_D_R2_FIXTURE.read_text(encoding="utf-8"))

    assert payload["contract_id"] == ATOMIC_OWNERLESS_POINT_CONTACT_V1
    assert payload["pairwise_sequential_application"] == "FORBIDDEN"
    assert payload["supported_effect"] == (
        InteractionCoverageEffect
        .ATOMIC_OWNERLESS_POINT_CONTACT_IDENTITY
        .value
    )
    assert len(payload["cases"]) == 6
    for case in payload["cases"]:
        assert _d_r2_contract_projection(case) == case["expected"]


def test_d_r2_contract_is_invariant_under_all_participant_and_edge_permutations():
    payload = json.loads(_D_R2_FIXTURE.read_text(encoding="utf-8"))

    for original in (*payload["cases"], payload["field_case"]):
        expected = _d_r2_contract_projection(original)
        projections = set()
        for participant_order in permutations(original["participants"]):
            for edge_order in permutations(original["hyperedges"]):
                permuted = {
                    **original,
                    "participants": list(participant_order),
                    "hyperedges": list(edge_order),
                }
                projections.add(
                    json.dumps(
                        _d_r2_contract_projection(permuted),
                        sort_keys=True,
                        separators=(",", ":"),
                    )
                )
        assert projections == {
            json.dumps(expected, sort_keys=True, separators=(",", ":"))
        }


def test_d_r2_field_fixture_matches_the_accepted_point_only_hypergraph():
    payload = json.loads(_D_R2_FIXTURE.read_text(encoding="utf-8"))
    field = payload["field_case"]
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (_BUILDING_002_FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (_BUILDING_002_FIXTURE / "decal_request.json").read_bytes()
    )
    compiled_result = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled_result.compilation is not None
    compilation = compiled_result.compilation
    raw_result = kernel.evaluate_reference_raw_coverage(
        compilation, request.requested_alpha.value
    )
    assert raw_result.raw_coverage is not None
    raw = raw_result.raw_coverage
    boundary_resolved = tuple(raw.boundary_resolved_envelopes)
    components = kernel.compile_interaction_components(
        compilation, boundary_resolved
    )
    models, model_diagnostics = kernel.compile_arrival_models(
        compilation, components, boundary_resolved
    )
    assert model_diagnostics == ()
    candidates = kernel.generate_interaction_candidates(
        components, models, compilation
    )

    pair_proofs = []
    for candidate in candidates:
        proofs, diagnostics = kernel.prove_mutual_arrivals(
            (candidate,), models
        )
        assert diagnostics == ()
        pair_proofs.extend(proofs)
    assert len(pair_proofs) == 2
    assert {
        proof.mutual_arrival_certificate.same_alpha_batch_identity.value
        for proof in pair_proofs
    } == {field["same_alpha_batch_id"]}
    assert {
        proof.mutual_arrival_certificate.exact_alpha.expression
        for proof in pair_proofs
    } == {field["event_alpha"]}

    observed_edges = []
    for proof in pair_proofs:
        assert proof.clipped_locus.segments == ()
        assert len(proof.clipped_locus.anchors) == 1
        anchor = proof.clipped_locus.anchors[0]
        observed_edges.append(
            {
                "candidate_id": proof.candidate.candidate_id.value,
                "participant_component_ids": sorted(
                    (
                        proof.candidate.left_component_id.value,
                        proof.candidate.right_component_id.value,
                    )
                ),
                "participant_front_reading_ids": sorted(
                    (
                        proof.mutual_arrival_certificate
                        .left_front_reading
                        .front_reading_id
                        .value,
                        proof.mutual_arrival_certificate
                        .right_front_reading
                        .front_reading_id
                        .value,
                    )
                ),
                "mutual_arrival_certificate_id": (
                    proof.mutual_arrival_certificate.certificate_id.value
                ),
                "equality_locus_id": proof.equality_locus.locus_id.value,
                "locus_dimension": 0,
                "anchor": {
                    "x": anchor.x.expression,
                    "y": anchor.y.expression,
                },
            }
        )
    expected_edges = [
        {
            **item,
            "participant_component_ids": sorted(
                item["participant_component_ids"]
            ),
            "participant_front_reading_ids": sorted(
                item["participant_front_reading_ids"]
            ),
        }
        for item in field["hyperedges"]
    ]
    assert sorted(
        observed_edges, key=lambda item: item["candidate_id"]
    ) == sorted(expected_edges, key=lambda item: item["candidate_id"])

    resolved_by_instance = {
        item.envelope_instance.envelope_instance_id: item
        for item in boundary_resolved
    }
    reachability = {
        instance_id: item.reachability
        for instance_id, item in resolved_by_instance.items()
    }
    regions_by_component = {
        component.interaction_component_id: tuple(
            region
            for instance_id in component.envelope_instance_ids
            for region in resolved_by_instance[
                instance_id.value
            ].envelope_instance.regions
        )
        for component in components
    }
    intersection_areas = []
    for index, left in enumerate(components):
        for right in components[index + 1:]:
            intersection_areas.append(
                _component_intersection_area(
                    regions_by_component[left.interaction_component_id],
                    regions_by_component[right.interaction_component_id],
                    reachability,
                )
            )
    assert intersection_areas == [0, 0, 0]
    assert raw.semantic_digest == field["raw_union_digest"]
    assert raw.exact_area_expression == field["exact_area"]
    assert len(raw.point_contacts) == field["point_contact_count"]
    assert _d_r2_contract_projection(field) == field["expected"]

    current = kernel.resolve_coverage_interactions(
        compilation, boundary_resolved, raw
    )
    assert current.outcome is (
        InteractionOutcome.MULTIWAY_INTERACTION_POLICY_UNPROVEN
    )
