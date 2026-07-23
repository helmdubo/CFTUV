"""Executable registry for the canonical Session C planar case declarations."""

from __future__ import annotations

from dataclasses import dataclass, replace
from decimal import Decimal

import sympy as sp

from cftuv_envelope import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    ChainUseOrientation,
    JunctionEnvelopeSpec,
    JunctionRelationKind,
    SurfaceTriangleId,
    SurfaceTriangleV1,
    TerminalEndpointRole,
)
from cftuv_envelope.reference import (
    RawCoverageLoopKind,
    ReferenceOutcome,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)
from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.common import make_region, make_segment
from cftuv_envelope.reference.contracts import ReachabilityCertificateV1
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    exact_sign,
)
from cftuv_envelope.reference.provenance import make_reference_provenance

from reference_factories import (
    angular_snapshot,
    junction_snapshot,
    physical_seam_snapshot,
    straight_snapshot,
)


SQUARE = (
    ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
)
HOLE_RING = (
    ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
    ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
    ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
    ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
)


@dataclass(frozen=True, slots=True)
class ExecutedCase:
    observed: dict[str, object]
    profile_k: int | None
    contributor_count: int
    named_outcomes: tuple[str, ...]
    lineage_verified: bool


def _execute_plans(snapshot, request, alphas, domain_ids=(None,)):
    compilations = []
    evaluations = []
    for domain_id, alpha in zip(domain_ids, alphas, strict=True):
        compiled = compile_reference_envelopes(snapshot, request, domain_id)
        compilations.append(compiled)
        if compiled.compilation is not None:
            evaluations.append(
                evaluate_reference_raw_coverage(compiled.compilation, Decimal(alpha))
            )
    return tuple(compilations), tuple(evaluations)


def _receipt(
    compilations,
    evaluations,
    *,
    extra: dict[str, object] | None = None,
) -> ExecutedCase:
    plans = tuple(
        item.compilation for item in compilations if item.compilation is not None
    )
    raws = tuple(
        item.raw_coverage for item in evaluations if item.raw_coverage is not None
    )
    observed: dict[str, object] = {
        "patch_plans": len(plans),
        "regions": sum(len(item.regions) for item in raws) if raws else None,
        "holes": (
            sum(
                loop.kind is RawCoverageLoopKind.HOLE
                for raw in raws
                for loop in raw.loops
            )
            if raws
            else None
        ),
        "regions_per_plan": (
            len(raws[0].regions)
            if raws and len({len(item.regions) for item in raws}) == 1
            else None
        ),
        "area": raws[0].exact_area_expression if len(raws) == 1 else None,
        "cap_specs": sum(
            isinstance(spec, CapEnvelopeSpec)
            for plan in plans
            for spec in plan.envelope_specs
        ),
        "compile_only": bool(plans) and not raws,
    }
    if extra:
        observed.update(extra)

    angular_specs = tuple(
        spec
        for plan in plans
        for spec in plan.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
    )
    profile_k = (
        angular_specs[0].resolved_hidden_edge_count if angular_specs else None
    )
    non_cap_counts = [
        sum(not isinstance(spec, CapEnvelopeSpec) for spec in plan.envelope_specs)
        for plan in plans
    ]
    named_outcomes = tuple(
        sorted(
            {
                item.outcome.value
                for item in (*compilations, *evaluations)
                if item.outcome is not ReferenceOutcome.EXACT
            }
        )
    )
    lineage_verified = bool(plans) and all(
        (
            all(
                (
                    edge.provenance.boundary_generator.support_ids
                    or edge.provenance.boundary_generator.physical_edge_ids
                    or edge.provenance.boundary_generator.boundary_constraint_ids
                )
                and edge.provenance.coverage_contributors.envelope_spec_ids
                for edge in raw.edges
            )
            and all(
                region.provenance.coverage_contributors.envelope_spec_ids
                for region in raw.regions
            )
        )
        for raw in raws
    )
    if not raws:
        lineage_verified = bool(plans) and all(
            item.provenance.coverage_contributors.chain_use_ids
            for plan in plans
            for item in plan.source_provenance
        )
    return ExecutedCase(
        observed=observed,
        profile_k=profile_k,
        contributor_count=max(non_cap_counts, default=0),
        named_outcomes=named_outcomes,
        lineage_verified=lineage_verified,
    )


def _straight_case(case):
    case_id = case["id"]
    if case_id in {"SC-P01-STRAIGHT-STRIP", "SC-P02-PHYSICAL-CAPS"}:
        snapshot, request = straight_snapshot(
            faces=SQUARE,
            source_routes=(
                {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
            ),
            alpha=case["decal_request"]["alpha"],
        )
        compilations, evaluations = _execute_plans(
            snapshot, request, (case["decal_request"]["alpha"],)
        )
        return _receipt(compilations, evaluations)

    if case_id == "SC-P07-DATA-CHAIN-SPLIT-MERGE":
        baseline_snapshot, baseline_request = straight_snapshot(
            faces=SQUARE,
            source_routes=(
                {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
            ),
            alpha="2",
        )
        split_face = (
            (
                (0.0, 0.0),
                (4.0, 0.0),
                (6.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
            ),
        )
        split_snapshot, split_request = straight_snapshot(
            faces=split_face,
            source_routes=(
                {
                    "name": "source",
                    "points": (
                        (0.0, 0.0),
                        (4.0, 0.0),
                        (6.0, 0.0),
                        (10.0, 0.0),
                    ),
                },
            ),
            alpha="2",
        )
        first_c, first_e = _execute_plans(
            baseline_snapshot, baseline_request, ("2",)
        )
        second_c, second_e = _execute_plans(split_snapshot, split_request, ("2",))
        first = first_e[0].raw_coverage
        second = second_e[0].raw_coverage
        invariant = (
            sp.sympify(first.exact_area_expression)
            == sp.sympify(second.exact_area_expression)
            and len(first.regions) == len(second.regions)
            and len(first.loops) == len(second.loops)
        )
        return _receipt(
            first_c,
            first_e,
            extra={"silhouette_invariant": invariant},
        )

    if case_id == "SC-P19-VERY-SHORT-SOURCE":
        face = (
            (
                (0.0, 0.0),
                (0.001, 0.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (0.0, 1.0),
            ),
        )
        snapshot, request = straight_snapshot(
            faces=face,
            source_routes=(
                {"name": "short", "points": ((0.0, 0.0), (0.001, 0.0))},
            ),
            alpha="0.5",
        )
        compilations, evaluations = _execute_plans(
            snapshot, request, ("0.5",)
        )
        return _receipt(
            compilations,
            evaluations,
            extra={
                "positive_exact_area": exact_sign(
                    evaluations[0].raw_coverage.exact_area_expression
                )
                > 0
            },
        )

    if case_id == "SC-P21-RETRIANGULATED-SURFACE":
        snapshot, request = straight_snapshot(
            faces=SQUARE,
            source_routes=(
                {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
            ),
            alpha="2",
        )
        first_c, first_e = _execute_plans(snapshot, request, ("2",))
        face = next(iter(snapshot.surface_ir.source_faces))
        v0, v1, v2, v3 = face.vertex_cycle
        e0, e1, e2, e3 = face.edge_cycle
        alternate = frozenset(
            {
                SurfaceTriangleV1(
                    SurfaceTriangleId("alternate-a"),
                    face.face_id,
                    (v0, v1, v3),
                    (e0, None, e3),
                    face.polygon_normal,
                ),
                SurfaceTriangleV1(
                    SurfaceTriangleId("alternate-b"),
                    face.face_id,
                    (v1, v2, v3),
                    (e1, e2, None),
                    face.polygon_normal,
                ),
            }
        )
        changed_face = replace(
            face, triangle_ids=tuple(item.triangle_id for item in alternate)
        )
        changed = replace(
            snapshot,
            surface_ir=replace(
                snapshot.surface_ir,
                source_faces=frozenset({changed_face}),
                surface_triangles=alternate,
            ),
        )
        _, second_e = _execute_plans(changed, request, ("2",))
        return _receipt(
            first_c,
            first_e,
            extra={
                "semantic_digest_invariant": (
                    first_e[0].raw_coverage.semantic_digest
                    == second_e[0].raw_coverage.semantic_digest
                )
            },
        )

    if case_id == "SC-P22-UNIFORM-SCALE-TRANSLATION":
        snapshot, request = straight_snapshot(
            faces=SQUARE,
            source_routes=(
                {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
            ),
            alpha="2",
        )
        first_c, first_e = _execute_plans(snapshot, request, ("2",))
        transform = lambda point: (3 * point[0] + 7, 3 * point[1] - 11)
        scaled_snapshot, scaled_request = straight_snapshot(
            faces=tuple(
                tuple(transform(point) for point in face) for face in SQUARE
            ),
            source_routes=(
                {
                    "name": "source",
                    "points": tuple(
                        transform(point)
                        for point in ((0.0, 0.0), (10.0, 0.0))
                    ),
                },
            ),
            alpha="6",
            revision_name="corpus-scaled",
        )
        _, second_e = _execute_plans(scaled_snapshot, scaled_request, ("6",))
        scale = sp.factor(
            sp.sympify(second_e[0].raw_coverage.exact_area_expression)
            / sp.sympify(first_e[0].raw_coverage.exact_area_expression)
        )
        return _receipt(
            first_c,
            first_e,
            extra={"area_scale": int(scale)},
        )

    if case_id == "SC-P23-REVERSED-STORAGE-SEMANTIC-ORIENTATION":
        snapshot, request = straight_snapshot(
            faces=SQUARE,
            source_routes=(
                {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
            ),
            alpha="2",
        )
        first_c, first_e = _execute_plans(snapshot, request, ("2",))
        use = next(
            item
            for item in snapshot.chain_uses
            if item.chain_use_id in request.selected_chain_use_ids
        )
        chain = next(
            item
            for item in snapshot.physical_chains
            if item.physical_chain_id == use.physical_chain_id
        )
        reversed_snapshot = replace(
            snapshot,
            physical_chains=frozenset(
                replace(
                    item,
                        ordered_source_vertex_ids=tuple(
                            reversed(chain.ordered_source_vertex_ids)
                        ),
                        ordered_physical_edge_ids=tuple(
                            reversed(chain.ordered_physical_edge_ids)
                        ),
                    )
                if item.physical_chain_id == chain.physical_chain_id
                else item
                for item in snapshot.physical_chains
            ),
            chain_uses=frozenset(
                replace(item, orientation=ChainUseOrientation.B_START_TO_END)
                if item.chain_use_id == use.chain_use_id
                else item
                for item in snapshot.chain_uses
            ),
            terminal_relations=frozenset(
                replace(
                    item,
                    endpoint_role=(
                        TerminalEndpointRole.END
                        if item.endpoint_role is TerminalEndpointRole.START
                        else TerminalEndpointRole.START
                    ),
                )
                for item in snapshot.terminal_relations
            ),
        )
        _, second_e = _execute_plans(reversed_snapshot, request, ("2",))
        return _receipt(
            first_c,
            first_e,
            extra={
                "semantic_digest_invariant": (
                    first_e[0].raw_coverage.semantic_digest
                    == second_e[0].raw_coverage.semantic_digest
                )
            },
        )
    raise AssertionError(f"unhandled straight fixture {case_id}")


def _angular_case(case):
    hidden_count = int(case["id"].rsplit("K", 1)[1])
    snapshot, request = angular_snapshot(hidden_count)
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))
    selection = next(iter(compilations[0].compilation.profile_selection_certificates))
    return _receipt(
        compilations,
        evaluations,
        extra={"local_profile_segments": selection.local_profile_segment_count},
    )


def _angular_internalized_case(case):
    """Run the public K0 evaluator and its exact arrangement internalization oracle."""

    snapshot, request = angular_snapshot(0)
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))

    def declared_region(name, coordinates, spec_id, instance_id):
        points = tuple(
            ExactPlanarPoint.from_values(*item) for item in coordinates
        )
        certificates = tuple(
            ConstructionCertificate(
                ConstructionKind.SOURCE_VERTEX,
                source_vertex_ids=frozenset({f"vertex:{name}:{index}"}),
            )
            for index in range(len(points))
        )
        provenance = make_reference_provenance(
            envelope_spec_ids=frozenset({spec_id}) if spec_id else frozenset(),
            envelope_instance_ids=(
                frozenset({instance_id}) if instance_id else frozenset()
            ),
            support_ids=frozenset({f"support:{name}"}),
            physical_edge_ids=frozenset({f"edge:{name}"}),
            chain_use_ids=(
                frozenset({f"use:{name}"}) if spec_id else frozenset()
            ),
            patch_domain_ids=frozenset({"domain"}),
        )
        segments = tuple(
            make_segment(
                f"segment:{name}:{index}",
                points[index],
                points[(index + 1) % len(points)],
                support_ids=provenance.support_ids,
                provenance=provenance,
                start_certificates=frozenset({certificates[index]}),
                end_certificates=frozenset(
                    {certificates[(index + 1) % len(points)]}
                ),
            )
            for index in range(len(points))
        )
        return make_region(
            f"region:{name}",
            segments,
            spec_id=spec_id,
            instance_id=instance_id,
        )

    outer = case["analysis_snapshot"]["outer"]
    angular_coordinates = case["analysis_snapshot"]["angular"]
    covering = declared_region(
        "covering-strip", outer, "strip-spec", "strip-instance"
    )
    angular = declared_region(
        "angular", angular_coordinates, "angular-spec", "angular-instance"
    )
    domain = declared_region(
        "domain",
        (
            (outer[0][0] - 1, outer[0][1] - 1),
            (outer[1][0] + 1, outer[1][1] - 1),
            (outer[2][0] + 1, outer[2][1] + 1),
            (outer[3][0] - 1, outer[3][1] + 1),
        ),
        "",
        "",
    )
    reachability = {
        instance_id: ReachabilityCertificateV1(
            frozenset({f"front:{instance_id}"}), True, 1, 1, (), False
        )
        for instance_id in ("strip-instance", "angular-instance")
    }
    union = ExactSegmentArrangementBackend().exact_union(
        (covering, angular), (domain,), reachability
    )
    region = next(iter(union.regions))
    lineage_verified = (
        region.provenance.coverage_contributors.envelope_spec_ids
        == frozenset({"strip-spec", "angular-spec"})
        and all(
            "support:angular"
            not in edge.provenance.boundary_generator.support_ids
            for edge in union.edges
        )
    )
    return ExecutedCase(
        observed={
            "regions": len(union.regions),
            "holes": sum(
                item.kind is RawCoverageLoopKind.HOLE for item in union.loops
            ),
            "angular_exposed_edges": sum(
                "angular-spec"
                in edge.provenance.coverage_contributors.envelope_spec_ids
                for edge in union.edges
            ),
        },
        profile_k=next(
            item.resolved_hidden_edge_count
            for item in compilations[0].compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        ),
        contributor_count=len(region.contributor_envelope_spec_ids),
        named_outcomes=tuple(
            item.outcome.value
            for item in evaluations
            if item.outcome is not ReferenceOutcome.EXACT
        ),
        lineage_verified=lineage_verified,
    )


def _triangle_case(case):
    triangle = (((-1.0, 0.0), (1.0, 0.0), (0.0, 1.0)),)
    snapshot, request = straight_snapshot(
        faces=triangle,
        source_routes=(
            {"name": "base", "points": ((-1.0, 0.0), (1.0, 0.0))},
        ),
        alpha="1",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))
    raw = evaluations[0].raw_coverage
    event_count = len(
        {
            event
            for edge in raw.edges
            for event in edge.reachability.boundary_event_keys
        }
    )
    key = (
        "same_alpha_event_count"
        if "SIMULTANEOUS" in case["id"]
        else "simultaneous_contacts"
    )
    return _receipt(compilations, evaluations, extra={key: event_count})


def _hole_case(case):
    snapshot, request = straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {
                "name": "bottom",
                "points": (
                    (0.0, 0.0),
                    (4.0, 0.0),
                    (6.0, 0.0),
                    (10.0, 0.0),
                ),
            },
            {
                "name": "top",
                "points": (
                    (10.0, 10.0),
                    (6.0, 10.0),
                    (4.0, 10.0),
                    (0.0, 10.0),
                ),
            },
        ),
        alpha="4",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("4",))
    plan = compilations[0].compilation
    raw = evaluations[0].raw_coverage
    use_by_component = {
        item.front_component_id.value: item.chain_use_id.value
        for item in plan.front_components
    }
    alpha_by_use = {
        use_by_component[item.front_component_id]: item.effective_alpha.expression
        for item in raw.component_effective_alphas
    }
    return _receipt(
        compilations,
        evaluations,
        extra={
            "bottom_effective_alpha": next(
                value for key, value in alpha_by_use.items() if "bottom" in key
            ),
            "top_effective_alpha": next(
                value for key, value in alpha_by_use.items() if "top" in key
            ),
        },
    )


def _concave_case(case):
    concave = (
        (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (6.0, 10.0),
            (6.0, 3.0),
            (4.0, 3.0),
            (4.0, 10.0),
            (0.0, 10.0),
        ),
    )
    snapshot, request = straight_snapshot(
        faces=concave,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
        alpha="4",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("4",))
    effective = next(
        iter(evaluations[0].raw_coverage.component_effective_alphas)
    ).effective_alpha.expression
    return _receipt(
        compilations, evaluations, extra={"effective_alpha": effective}
    )


def _overlap_case(case):
    snapshot, request = straight_snapshot(
        faces=SQUARE,
        source_routes=(
            {"name": "source-a", "points": ((0.0, 0.0), (10.0, 0.0))},
            {"name": "source-b", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
        alpha="2",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("2",))
    return _receipt(
        compilations,
        evaluations,
        extra={"single_cover": len(evaluations[0].raw_coverage.regions) == 1},
    )


def _seam_self_case(case):
    faces = (
        ((0.0, 0.0), (5.0, 0.0), (5.0, 10.0), (0.0, 10.0)),
        ((5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (5.0, 10.0)),
    )
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=(
            {
                "name": "self-seam",
                "points": ((5.0, 0.0), (5.0, 10.0)),
                "kind": "SEAM_SELF",
                "uses": (
                    ("a", "A_START_TO_END"),
                    ("b", "B_START_TO_END"),
                ),
            },
        ),
        alpha="1",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))
    return _receipt(compilations, evaluations)


def _physical_seam_case(case):
    cross = bool(case["analysis_snapshot"].get("cross_patch_junction"))
    snapshot, request, domains = physical_seam_snapshot(cross_junction=cross)
    compilations, evaluations = _execute_plans(
        snapshot, request, ("1", "1"), domains
    )
    extra: dict[str, object] = {"cross_patch_collision": False}
    if cross:
        junction_specs = tuple(
            spec
            for item in compilations
            for spec in item.compilation.envelope_specs
            if isinstance(spec, JunctionEnvelopeSpec)
        )
        extra.update(
            {
                "shared_anchor_count": len(
                    {item.shared_semantic_anchor_id for item in junction_specs}
                ),
                "projection_count": len(
                    {item.per_patch_projection_id for item in junction_specs}
                ),
            }
        )
    return _receipt(compilations, evaluations, extra=extra)


def _junction_case(case):
    kind = JunctionRelationKind(case["analysis_snapshot"]["kind"])
    snapshot, request = junction_snapshot(kind)
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))
    spec = next(
        item
        for item in compilations[0].compilation.envelope_specs
        if isinstance(item, JunctionEnvelopeSpec)
    )
    return _receipt(
        compilations,
        evaluations,
        extra={"route_pairs": len(spec.route_pairing_ids)},
    )


def _closed_case(case):
    snapshot, request = straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {
                "name": "closed-source",
                "points": ((4.0, 3.0), (4.0, 5.0), (6.0, 5.0), (6.0, 3.0)),
                "is_closed": True,
            },
        ),
        alpha="1",
    )
    compilations, evaluations = _execute_plans(snapshot, request, ("1",))
    return _receipt(
        compilations,
        evaluations,
        extra={"requires_declared_segment_corner_relations": True},
    )


BUILDERS = {
    "straight_snapshot": _straight_case,
    "angular_snapshot": _angular_case,
    "arrangement_internalization_fixture": _angular_internalized_case,
    "triangle_endpoint_contact": _triangle_case,
    "hole_ring_snapshot": _hole_case,
    "concave_outer_snapshot": _concave_case,
    "overlap_sources_snapshot": _overlap_case,
    "seam_self_snapshot": _seam_self_case,
    "physical_seam_snapshot": _physical_seam_case,
    "junction_snapshot": _junction_case,
    "closed_hole_chain_snapshot": _closed_case,
}


def execute_session_c_case(case: dict) -> ExecutedCase:
    builder_name = case["analysis_snapshot"]["builder"]
    return BUILDERS[builder_name](case)
