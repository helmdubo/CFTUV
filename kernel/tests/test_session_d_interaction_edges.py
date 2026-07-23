from __future__ import annotations

from dataclasses import replace

import sympy as sp

from cftuv_envelope import (
    ArrivalModelKind,
    CapArrivalModelV1,
    ExactFrontArrivalLawV1,
    InteractionCandidateKind,
    InteractionCandidateV1,
    InteractionComponentV1,
    InteractionOutcome,
    ReachabilityCertificateV1,
    StripArrivalModelV1,
    generate_interaction_candidates,
    prove_mutual_arrivals,
)
from cftuv_envelope.interactions.equality_locus import (
    clip_equality_locus_to_active_domains,
)
from cftuv_envelope.interactions.policy_b import apply_policy_b
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
from session_d_factories import opposing_strip_state


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
        interaction_component_id=f"component:{name}",
        decal_request_id=request,
        patch_domain_id=domain,
        envelope_spec_ids=frozenset({f"spec:{name}"}),
        envelope_instance_ids=frozenset({f"instance:{name}"}),
        front_component_ids=frozenset({f"front:{name}"}),
        relation_ids=frozenset(),
        source_lineage_ids=frozenset({f"lineage:{name}"}),
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
        law_id=f"law:{name}",
        support_id=f"support:{name}",
        normal=normal_value,
        source_constant=ExactScalar.from_value(constant),
        normal_speed=ExactScalar.from_value(1),
    )
    front = region.outer.segments[2:3]
    return StripArrivalModelV1(
        arrival_model_id=f"model:{name}",
        model_kind=ArrivalModelKind.STRIP,
        interaction_component_id=f"component:{name}",
        decal_request_id="request",
        patch_domain_id="domain",
        envelope_spec_id=f"spec:{name}",
        envelope_instance_id=f"instance:{name}",
        front_component_id=f"front:{name}",
        front_reading_id=f"reading:{name}",
        source_lineage_ids=frozenset({f"lineage:{name}"}),
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
        (left, replace(_component("right"), decal_request_id="other")),
        (),
    ) == ()
    assert generate_interaction_candidates(
        (left, replace(_component("right"), patch_domain_id="other-domain")),
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
        "left-law",
        "left-support",
        ExactPlanarVector.from_values(1, 0),
        ExactScalar.from_value(0),
        ExactScalar.from_value(1),
    )
    right_law = ExactFrontArrivalLawV1(
        "right-law",
        "right-support",
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
        stable_id(
            "same-alpha-interaction-batch",
            "request",
            "domain",
            ExactScalar.from_value(5).expression,
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
        arrival_model_id="model:left-cap",
        model_kind=ArrivalModelKind.CAP,
        interaction_component_id=left_strip.interaction_component_id,
        decal_request_id="request",
        patch_domain_id="domain",
        envelope_spec_id="spec:left-cap",
        envelope_instance_id="instance:left-cap",
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


def test_explicit_self_contact_preserves_two_readings_after_domain_clip():
    compilation, raw, result = opposing_strip_state("6")
    assert result.outcome is InteractionOutcome.EXACT
    strip_models = tuple(
        item
        for item in result.arrival_models
        if isinstance(item, StripArrivalModelV1)
    )
    components = result.interaction_components
    merged = InteractionComponentV1(
        interaction_component_id="component:self",
        decal_request_id=components[0].decal_request_id,
        patch_domain_id=components[0].patch_domain_id,
        envelope_spec_ids=frozenset().union(
            *(item.envelope_spec_ids for item in components)
        ),
        envelope_instance_ids=frozenset().union(
            *(item.envelope_instance_ids for item in components)
        ),
        front_component_ids=frozenset().union(
            *(item.front_component_ids for item in components)
        ),
        relation_ids=frozenset(),
        source_lineage_ids=frozenset().union(
            *(item.source_lineage_ids for item in components)
        ),
    )
    self_models = tuple(
        replace(
            model,
            interaction_component_id=merged.interaction_component_id,
            front_reading_id=f"self-reading:{index}",
        )
        for index, model in enumerate(strip_models)
    )
    provenance = InteractionProvenanceV1(
        decal_request_id=merged.decal_request_id,
        patch_domain_id=merged.patch_domain_id,
        interaction_component_ids=frozenset(
            {merged.interaction_component_id}
        ),
        envelope_spec_ids=merged.envelope_spec_ids,
        envelope_instance_ids=merged.envelope_instance_ids,
        front_component_ids=merged.front_component_ids,
        front_reading_ids=frozenset(
            item.front_reading_id for item in self_models
        ),
        source_lineage_ids=merged.source_lineage_ids,
    )
    candidate = InteractionCandidateV1(
        "candidate:self",
        merged.interaction_component_id,
        merged.interaction_component_id,
        InteractionCandidateKind.EXPLICIT_SELF_CONTACT,
        merged.decal_request_id,
        merged.patch_domain_id,
        provenance,
    )
    proofs, diagnostics = prove_mutual_arrivals((candidate,), self_models)
    assert diagnostics == ()
    assert len(proofs) == 1
    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    assert frame is not None
    domain = build_domain_geometry(
        GeometryContext.build(compilation, frame)
    )
    policy = apply_policy_b(
        proofs,
        (merged,),
        tuple(raw.boundary_resolved_envelopes),
        raw,
        domain.face_regions,
    )
    assert policy.diagnostics == ()
    assert policy.resolved_union.exact_area_expression == raw.exact_area_expression
    retained_readings = frozenset(
        reading
        for contribution in policy.resolved_contributions
        for reading in contribution.retained_front_reading_ids
    )
    assert retained_readings == frozenset(
        item.front_reading_id for item in self_models
    )


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
        _component("projection-a"), patch_domain_id="domain-a"
    )
    projection_b = replace(
        _component("projection-b"), patch_domain_id="domain-b"
    )
    assert generate_interaction_candidates(
        (projection_a, projection_b), ()
    ) == ()
