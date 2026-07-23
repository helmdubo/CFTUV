from __future__ import annotations

from dataclasses import dataclass, replace

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
    InteractionOutcome,
    LawId,
    LineageId,
    PatchDomainId,
    ReachabilityCertificateV1,
    SourceSupportId,
    StripArrivalModelV1,
    generate_interaction_candidates,
    prove_mutual_arrivals,
    validate_resolved_coverage_digest,
)
from cftuv_envelope.interactions.equality_locus import (
    clip_equality_locus_to_active_domains,
)
from cftuv_envelope.interactions.policy_b import apply_policy_b
from cftuv_envelope.interactions.provenance import InteractionProvenanceV1
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext, make_segment
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
from session_d_factories import (
    angular_third_strip_state,
    opposing_strip_state,
    self_contact_state,
)


@dataclass(frozen=True, slots=True)
class ExecutedSessionDCase:
    observed: dict[str, object]


def _component(
    name: str,
    *,
    request: str = "request",
    domain: str = "domain",
) -> InteractionComponentV1:
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


def _strip_model(
    name: str,
    region: PlanarRegion,
    normal: tuple,
    constant,
    *,
    effective=6,
) -> StripArrivalModelV1:
    law = ExactFrontArrivalLawV1(
        law_id=LawId(f"law:{name}"),
        support_id=SourceSupportId(f"support:{name}"),
        normal=ExactPlanarVector.from_values(*normal),
        source_constant=ExactScalar.from_value(constant),
        normal_speed=ExactScalar.from_value(1),
    )
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
        active_segments=region.outer.segments[2:3],
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


def _resolved_observation(
    *,
    alpha: str,
    width: float = 10.0,
    scale: float = 1.0,
    translation: tuple[float, float] = (0.0, 0.0),
    reverse_sources: bool = False,
) -> dict[str, object]:
    _, raw, result = opposing_strip_state(
        alpha,
        width=width,
        scale=scale,
        translation=translation,
        reverse_sources=reverse_sources,
    )
    resolved = result.resolved_coverage
    return {
        "outcome": result.outcome.value,
        "application_count": (
            len(resolved.interaction_applications) if resolved else 0
        ),
        "certificate_alphas": (
            sorted(
                str(item.exact_alpha.as_expr())
                for item in resolved.mutual_arrival_certificates
            )
            if resolved
            else []
        ),
        "exact_area": resolved.exact_area_expression if resolved else None,
        "raw_area": raw.exact_area_expression,
        "digest_valid": bool(
            resolved and validate_resolved_coverage_digest(resolved)
        ),
        "creates_new_matter": (
            any(
                item.creates_new_matter
                for item in resolved.interaction_applications
            )
            if resolved
            else False
        ),
    }


def _angular_observation(hidden_count: int, alpha: str) -> dict[str, object]:
    compilation, raw, result = angular_third_strip_state(hidden_count, alpha)
    resolved = result.resolved_coverage
    angular_spec = next(
        item
        for item in compilation.envelope_specs
        if type(item).__name__ == "AngularEnvelopeSpec"
    )
    third_front = next(
        item.front_component_id
        for item in compilation.front_components
        if item.chain_use_id.value == "third-use"
    )
    angular_models = tuple(
        item
        for item in result.arrival_models
        if type(item).__name__ == "AngularProfileArrivalModelV1"
    )
    return {
        "outcome": result.outcome.value,
        "application_count": (
            len(resolved.interaction_applications) if resolved else 0
        ),
        "certificate_alphas": (
            sorted(
                {
                    str(item.exact_alpha.as_expr())
                    for item in resolved.mutual_arrival_certificates
                }
            )
            if resolved
            else []
        ),
        "profile_law_count": (
            len(angular_models[0].component_profile_arrival_laws)
            if angular_models
            else 0
        ),
        "third_front_excluded_from_angular_seed": (
            third_front not in angular_spec.incident_front_component_ids
        ),
        "area_preserved": bool(
            resolved
            and resolved.exact_area_expression == raw.exact_area_expression
        ),
        "creates_new_matter": (
            any(
                item.creates_new_matter
                for item in resolved.interaction_applications
            )
            if resolved
            else False
        ),
    }


def _candidate_filter_observation(kind: str) -> dict[str, object]:
    left = _component("left")
    if kind == "request":
        right = _component("right", request="other-request")
    elif kind == "patch":
        right = _component("right", domain="other-domain")
    elif kind == "cross_patch_projection":
        left = _component("projection-a", domain="domain-a")
        right = _component("projection-b", domain="domain-b")
    else:
        right = _component("third")
        model = _strip_model(
            "third",
            _rectangle("third", 4, 0, 10, 2),
            (-1, 0),
            -10,
        )
        candidates = generate_interaction_candidates(
            (left, right), (model,)
        )
        proofs, diagnostics = prove_mutual_arrivals(candidates, (model,))
        return {
            "candidate_count": len(candidates),
            "proof_count": len(proofs),
            "diagnostics": sorted(
                item.outcome.value for item in diagnostics
            ),
        }
    return {
        "candidate_count": len(
            generate_interaction_candidates((left, right), ())
        )
    }


def _proof_edge_observation(kind: str) -> dict[str, object]:
    if kind == "hole":
        left = _rectangle(
            "left-hole",
            0,
            0,
            6,
            10,
            hole=(sp.Rational(9, 2), 4, sp.Rational(11, 2), 6),
        )
        right = _rectangle(
            "right-hole",
            4,
            0,
            10,
            10,
            hole=(sp.Rational(9, 2), 4, sp.Rational(11, 2), 6),
        )
        locus = clip_equality_locus_to_active_domains(
            ExactFrontArrivalLawV1(
                LawId("left-law"),
                SourceSupportId("left-support"),
                ExactPlanarVector.from_values(1, 0),
                ExactScalar.from_value(0),
                ExactScalar.from_value(1),
            ),
            ExactFrontArrivalLawV1(
                LawId("right-law"),
                SourceSupportId("right-support"),
                ExactPlanarVector.from_values(-1, 0),
                ExactScalar.from_value(-10),
                ExactScalar.from_value(1),
            ),
            (left,),
            (right,),
        )
        assert locus is not None
        return {
            "locus_segment_count": len(locus.segments),
            "endpoint_y": sorted(
                {
                    str(point.y.as_expr())
                    for segment in locus.segments
                    for point in segment
                },
                key=sp.sympify,
            ),
        }

    regions = {
        "a": _rectangle("a", 0, 0, 6, 2),
        "b": _rectangle("b", 4, 0, 10, 2),
        "c": _rectangle("c", 0, 10, 6, 12),
        "d": _rectangle("d", 4, 10, 10, 12),
    }
    if kind == "saturated":
        models = (
            _strip_model("a", regions["a"], (1, 0), 0, effective=4),
            _strip_model("b", regions["b"], (-1, 0), -10),
        )
        components = (_component("a"), _component("b"))
    elif kind == "simultaneous":
        models = (
            _strip_model("a", regions["a"], (1, 0), 0),
            _strip_model("b", regions["b"], (-1, 0), -10),
            _strip_model("c", regions["c"], (1, 0), 0),
            _strip_model("d", regions["d"], (-1, 0), -10),
        )
        components = tuple(_component(name) for name in "abcd")
    elif kind == "triple":
        regions["c"] = _rectangle("c", 4, -5, 6, 1)
        models = (
            _strip_model("a", regions["a"], (1, 0), 0),
            _strip_model("b", regions["b"], (-1, 0), -10),
            _strip_model("c", regions["c"], (0, 1), -5),
        )
        components = tuple(_component(name) for name in "abc")
    elif kind == "coincident":
        models = (
            _strip_model("a", regions["a"], (1, 0), 0),
            replace(
                _strip_model("b", regions["a"], (-1, 0), -10),
                arrival_law=_strip_model(
                    "a", regions["a"], (1, 0), 0
                ).arrival_law,
            ),
        )
        components = (_component("a"), _component("b"))
    elif kind == "cap":
        left = _strip_model("a", regions["a"], (1, 0), 0)
        right = _strip_model("b", regions["b"], (-1, 0), -10)
        cap = CapArrivalModelV1(
            arrival_model_id=ArrivalModelId("model:a-cap"),
            model_kind=ArrivalModelKind.CAP,
            interaction_component_id=left.interaction_component_id,
            decal_request_id=DecalRequestId("request"),
            patch_domain_id=PatchDomainId("domain"),
            envelope_spec_id=EnvelopeSpecId("spec:a-cap"),
            envelope_instance_id=EnvelopeInstanceId("instance:a-cap"),
            incident_strip_spec_id=left.envelope_spec_id,
            front_component_id=left.front_component_id,
            front_reading_id=left.front_reading_id,
            source_lineage_ids=left.source_lineage_ids,
            arrival_law=left.arrival_law,
            active_segments=(regions["a"].outer.segments[0],),
            active_regions=left.active_regions,
            effective_alpha=left.effective_alpha,
            reachability=left.reachability,
            inherits_incident_strip_reading=True,
        )
        models = (cap, right)
        components = (_component("a"), _component("b"))
    else:
        raise AssertionError(f"unknown proof edge case: {kind}")

    proofs, diagnostics = prove_mutual_arrivals(
        generate_interaction_candidates(components, models),
        models,
    )
    return {
        "proof_count": len(proofs),
        "diagnostics": sorted(item.outcome.value for item in diagnostics),
        "batch_count": len(
            {
                item.mutual_arrival_certificate.same_alpha_batch_identity
                for item in proofs
            }
        ),
        "anchor_count": sum(
            len(item.clipped_locus.anchors) for item in proofs
        ),
    }


def _self_contact_observation() -> dict[str, object]:
    compilation, raw, result = self_contact_state("6")
    resolved = result.resolved_coverage
    assert resolved is not None
    return {
        "proof_count": len(resolved.mutual_arrival_certificates),
        "diagnostics": sorted(
            item.outcome.value for item in result.diagnostics
        ),
        "front_reading_count": len(
            compilation.front_reading_declarations
        ),
        "area_preserved": (
            resolved.exact_area_expression
            == raw.exact_area_expression
        ),
        "retained_front_reading_count": len(
            {
                reading
                for contribution in resolved.resolved_contributions
                for reading in contribution.retained_front_reading_ids
            }
        ),
    }


def execute_session_d_case(case: dict) -> ExecutedSessionDCase:
    scenario = case["scenario"]
    parameters = case.get("parameters", {})
    if scenario == "opposing":
        observed = _resolved_observation(**parameters)
    elif scenario == "angular_c13":
        observed = _angular_observation(**parameters)
    elif scenario == "candidate_filter":
        observed = _candidate_filter_observation(**parameters)
    elif scenario == "proof_edge":
        observed = _proof_edge_observation(**parameters)
    elif scenario == "explicit_self":
        observed = _self_contact_observation()
    elif scenario == "reversed_order":
        forward = _resolved_observation(alpha="6")
        reversed_order = _resolved_observation(
            alpha="6", reverse_sources=True
        )
        observed = {
            "forward_outcome": forward["outcome"],
            "reversed_outcome": reversed_order["outcome"],
            "same_digest": (
                opposing_strip_state("6")[2]
                .resolved_coverage.semantic_digest
                == opposing_strip_state(
                    "6", reverse_sources=True
                )[2].resolved_coverage.semantic_digest
            ),
        }
    elif scenario == "transform_scale":
        translated = _resolved_observation(
            alpha="6", translation=(17.0, -3.0)
        )
        scaled = _resolved_observation(alpha="12", scale=2.0)
        observed = {
            "translated_event": translated["certificate_alphas"],
            "scaled_event": scaled["certificate_alphas"],
            "scaled_area": scaled["exact_area"],
            "both_exact": (
                translated["outcome"] == scaled["outcome"] == "EXACT"
            ),
        }
    else:
        raise AssertionError(f"unknown Session D scenario: {scenario}")
    return ExecutedSessionDCase(observed)
