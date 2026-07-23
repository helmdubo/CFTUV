"""Compile exact front-arrival laws from Session C source supports."""

from __future__ import annotations

from collections import defaultdict

from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from ..reference.angular import _incident_normal, _interpolated_normals
from ..reference.arrangement import ExactSegmentArrangementBackend
from ..reference.boundary import build_domain_geometry
from ..reference.common import GeometryContext, make_segment, stable_id
from ..reference.contracts import (
    BoundaryResolvedEnvelopeV1,
    ReferenceEnvelopeCompilationV1,
)
from ..reference.planar_types import (
    BoundedSupportSegment,
    ExactScalar,
    exact_sign,
)
from ..reference.validation import validate_reference_geometry_payload
from .contracts import (
    ActiveDomainCertificateV1,
    AngularProfileArrivalModelV1,
    ArrivalModelKind,
    ArrivalModelV1,
    CapArrivalModelV1,
    ExactFrontArrivalLawV1,
    FrontArrivalReadingV1,
    InteractionComponentV1,
    InteractionDiagnosticSeverity,
    InteractionDiagnosticV1,
    InteractionOutcome,
    StripArrivalModelV1,
    UnsupportedJunctionArrivalModelV1,
)


_ARRANGEMENT = ExactSegmentArrangementBackend()


def _line_constant(normal, point) -> ExactScalar:
    nx, ny = normal.expressions()
    px, py = point.expressions()
    return ExactScalar.from_value(nx * px + ny * py)


def _segment_on_front(
    segment: BoundedSupportSegment,
    law: ExactFrontArrivalLawV1,
    effective_alpha: ExactScalar,
) -> bool:
    nx, ny = law.normal.expressions()
    expected = (
        law.source_constant.as_expr()
        + law.normal_speed.as_expr() * effective_alpha.as_expr()
    )
    for point in (segment.start, segment.end):
        px, py = point.expressions()
        if exact_sign(nx * px + ny * py - expected) != 0:
            return False
    return True


def _component_boundary_segments(
    component: InteractionComponentV1,
    resolved_by_instance: dict[str, BoundaryResolvedEnvelopeV1],
    domain_regions,
) -> tuple[BoundedSupportSegment, ...]:
    contribution_regions = tuple(
        region
        for instance_id in component.envelope_instance_ids
        if instance_id in resolved_by_instance
        for region in resolved_by_instance[instance_id].envelope_instance.regions
    )
    if not contribution_regions:
        return ()
    reachability = {
        instance_id: resolved_by_instance[instance_id].reachability
        for instance_id in component.envelope_instance_ids
        if instance_id in resolved_by_instance
    }
    union = _ARRANGEMENT.exact_union(
        contribution_regions, domain_regions, reachability
    )
    vertices = {item.vertex_id: item for item in union.vertices}
    return tuple(
        make_segment(
            item.edge_id,
            vertices[item.start_vertex_id].point,
            vertices[item.end_vertex_id].point,
            support_ids=item.provenance.support_ids,
            boundary_constraint_ids=item.provenance.boundary_constraint_ids,
            provenance=item.provenance,
            start_certificates=vertices[
                item.start_vertex_id
            ].construction_certificates,
            end_certificates=vertices[item.end_vertex_id].construction_certificates,
        )
        for item in sorted(union.edges, key=lambda edge: edge.edge_id)
    )


def _front_reading_ids(
    compilation: ReferenceEnvelopeCompilationV1,
    front_component_id: str,
    law_keys: tuple[str, ...],
) -> tuple[tuple[str, ...], InteractionDiagnosticV1 | None]:
    front_component = next(
        item
        for item in compilation.front_components
        if item.front_component_id.value == front_component_id
    )
    declared = tuple(sorted(item.value for item in front_component.front_reading_ids))
    if declared and len(declared) != len(law_keys):
        return (), InteractionDiagnosticV1(
            outcome=InteractionOutcome.SELF_CONTACT_READING_IDENTITY_UNPROVEN,
            severity=InteractionDiagnosticSeverity.UNSUPPORTED,
            message=(
                f"{front_component_id} declares {len(declared)} readings for "
                f"{len(law_keys)} distinct exact arrival laws"
            ),
            arrival_model_ids=frozenset(law_keys),
        )
    if declared:
        return declared, None
    return tuple(
        stable_id("front-reading", front_component_id, law_key)
        for law_key in law_keys
    ), None


def _law_key(normal, constant: ExactScalar) -> str:
    return stable_id(
        "arrival-law-key",
        normal.x.expression,
        normal.y.expression,
        constant.expression,
        "1",
    )


def _angular_support_data(context: GeometryContext, spec: AngularEnvelopeSpec):
    relation = next(
        item
        for item in context.snapshot.corner_relations
        if item.corner_relation_id == spec.source_relation_id
    )
    sector = next(
        item
        for item in context.snapshot.angular_owner_sectors
        if item.owner_sector_id == spec.owner_sector_id
    )
    anchor = context.points_by_id[relation.source_vertex_id]
    incoming, incoming_support_id = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[0],
        relation.source_vertex_id,
    )
    outgoing, outgoing_support_id = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[-1],
        relation.source_vertex_id,
    )
    normals = _interpolated_normals(
        incoming,
        outgoing,
        spec.resolved_hidden_edge_count,
        sector.turn_orientation,
    )
    hidden_by_ordinal = {item.ordinal: item for item in spec.hidden_supports}
    support_ids = [incoming_support_id]
    support_ids.extend(
        hidden_by_ordinal[index].hidden_support_id.value
        for index in range(1, spec.resolved_hidden_edge_count + 1)
    )
    support_ids.append(outgoing_support_id)
    return relation, anchor, tuple(support_ids), normals


def compile_arrival_models(
    compilation: ReferenceEnvelopeCompilationV1,
    components: tuple[InteractionComponentV1, ...],
    boundary_resolved_envelopes: tuple[BoundaryResolvedEnvelopeV1, ...],
) -> tuple[tuple[ArrivalModelV1, ...], tuple[InteractionDiagnosticV1, ...]]:
    """Build laws from source supports; emitted regions only certify active reach."""

    frame, payload_diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    if frame is None:
        return (), (
            InteractionDiagnosticV1(
                InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
                InteractionDiagnosticSeverity.UNSUPPORTED,
                payload_diagnostics[0].message,
            ),
        )
    context = GeometryContext.build(compilation, frame)
    domain = build_domain_geometry(context)
    resolved_by_instance = {
        item.envelope_instance.envelope_instance_id: item
        for item in boundary_resolved_envelopes
    }
    resolved_by_spec = {
        item.envelope_instance.envelope_spec_id: item
        for item in boundary_resolved_envelopes
    }
    component_by_spec = {
        spec_id: component
        for component in components
        for spec_id in component.envelope_spec_ids
    }
    boundary_by_component = {
        component.interaction_component_id: _component_boundary_segments(
            component, resolved_by_instance, domain.face_regions
        )
        for component in components
    }
    models: list[ArrivalModelV1] = []
    diagnostics: list[InteractionDiagnosticV1] = []
    strip_models_by_spec: dict[str, list[StripArrivalModelV1]] = defaultdict(list)

    variant_order = {
        StripEnvelopeSpec: 0,
        AngularEnvelopeSpec: 1,
        CapEnvelopeSpec: 2,
        JunctionEnvelopeSpec: 3,
    }
    for spec in sorted(
        compilation.envelope_specs,
        key=lambda item: (
            variant_order[type(item)],
            item.envelope_spec_id.value,
        ),
    ):
        spec_id = spec.envelope_spec_id.value
        component = component_by_spec[spec_id]
        component_boundary = boundary_by_component[
            component.interaction_component_id
        ]
        resolved = resolved_by_spec.get(spec_id)
        if isinstance(spec, JunctionEnvelopeSpec):
            model = UnsupportedJunctionArrivalModelV1(
                arrival_model_id=stable_id(
                    "junction-arrival-unproven", component.interaction_component_id, spec_id
                ),
                model_kind=ArrivalModelKind.UNSUPPORTED_JUNCTION,
                interaction_component_id=component.interaction_component_id,
                decal_request_id=spec.decal_request_id.value,
                patch_domain_id=spec.patch_domain_id.value,
                envelope_spec_id=spec_id,
                envelope_instance_id=(
                    resolved.envelope_instance.envelope_instance_id
                    if resolved is not None
                    else None
                ),
                source_relation_id=spec.source_relation_id.value,
                outcome=InteractionOutcome.INTERACTION_JUNCTION_ARRIVAL_UNPROVEN,
                message="Session C-R1 has no accepted exact Junction support law",
            )
            models.append(model)
            diagnostics.append(
                InteractionDiagnosticV1(
                    model.outcome,
                    InteractionDiagnosticSeverity.UNSUPPORTED,
                    model.message,
                    frozenset({component.interaction_component_id}),
                    frozenset({model.arrival_model_id}),
                )
            )
            continue
        if resolved is None:
            diagnostics.append(
                InteractionDiagnosticV1(
                    InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
                    InteractionDiagnosticSeverity.UNSUPPORTED,
                    f"BoundaryResolvedEnvelope is missing for {spec_id}",
                    frozenset({component.interaction_component_id}),
                )
            )
            continue

        instance = resolved.envelope_instance
        if isinstance(spec, StripEnvelopeSpec):
            seed = next(
                item
                for item in compilation.seeds
                if getattr(item, "seed_id", None) == spec.source_seed_id
            )
            source_segments = context.support_segments_for_use(
                seed.chain_use_id, spec_id
            )
            grouped: dict[str, list] = defaultdict(list)
            law_by_key = {}
            source_by_key = {}
            for source in source_segments:
                constant = _line_constant(source.owner_normal, source.start)
                key = _law_key(source.owner_normal, constant)
                law = ExactFrontArrivalLawV1(
                    law_id=stable_id("strip-arrival-law", spec_id, key),
                    support_id=source.support_id,
                    normal=source.owner_normal,
                    source_constant=constant,
                    normal_speed=ExactScalar.from_value(1),
                )
                moved_support_id = stable_id("moving-support", source.support_id)
                grouped[key].extend(
                    segment
                    for segment in component_boundary
                    if moved_support_id in segment.support_ids
                    and _segment_on_front(segment, law, resolved.effective_alpha)
                )
                law_by_key[key] = law
                source_by_key[key] = source
            law_keys = tuple(sorted(key for key, active in grouped.items() if active))
            reading_ids, issue = _front_reading_ids(
                compilation, next(iter(source_by_key.values())).front_component_id, law_keys
            )
            if issue is not None:
                diagnostics.append(issue)
                continue
            for key, reading_id in zip(law_keys, reading_ids, strict=True):
                source = source_by_key[key]
                model = StripArrivalModelV1(
                    arrival_model_id=stable_id(
                        "strip-arrival-model",
                        component.interaction_component_id,
                        spec_id,
                        key,
                    ),
                    model_kind=ArrivalModelKind.STRIP,
                    interaction_component_id=component.interaction_component_id,
                    decal_request_id=spec.decal_request_id.value,
                    patch_domain_id=spec.patch_domain_id.value,
                    envelope_spec_id=spec_id,
                    envelope_instance_id=instance.envelope_instance_id,
                    front_component_id=source.front_component_id,
                    front_reading_id=reading_id,
                    source_lineage_ids=frozenset(
                        item.value for item in spec.source_lineage_ids
                    ),
                    arrival_law=law_by_key[key],
                    active_segments=tuple(
                        sorted(grouped[key], key=lambda item: item.segment_id)
                    ),
                    active_regions=instance.regions,
                    effective_alpha=resolved.effective_alpha,
                    reachability=resolved.reachability,
                )
                models.append(model)
                strip_models_by_spec[spec_id].append(model)
            continue

        if isinstance(spec, AngularEnvelopeSpec):
            relation, anchor, support_ids, normals = _angular_support_data(
                context, spec
            )
            profile_laws = tuple(
                ExactFrontArrivalLawV1(
                    law_id=stable_id(
                        "angular-profile-arrival-law", spec_id, ordinal, support_id
                    ),
                    support_id=support_id,
                    normal=normal,
                    source_constant=_line_constant(normal, anchor),
                    normal_speed=ExactScalar.from_value(1),
                )
                for ordinal, (support_id, normal) in enumerate(
                    zip(support_ids, normals, strict=True)
                )
            )
            for ordinal, law in enumerate(profile_laws):
                support_id = law.support_id
                active = tuple(
                    segment
                    for segment in component_boundary
                    if support_id in segment.support_ids
                    and _segment_on_front(segment, law, resolved.effective_alpha)
                )
                if not active:
                    continue
                reading_id = stable_id(
                    "angular-profile-front-reading",
                    component.interaction_component_id,
                    support_id,
                )
                models.append(
                    AngularProfileArrivalModelV1(
                        arrival_model_id=stable_id(
                            "angular-profile-arrival-model",
                            component.interaction_component_id,
                            spec_id,
                            ordinal,
                        ),
                        model_kind=ArrivalModelKind.ANGULAR_PROFILE,
                        interaction_component_id=component.interaction_component_id,
                        decal_request_id=spec.decal_request_id.value,
                        patch_domain_id=spec.patch_domain_id.value,
                        envelope_spec_id=spec_id,
                        envelope_instance_id=instance.envelope_instance_id,
                        front_component_ids=frozenset(
                            item.value
                            for item in spec.incident_front_component_ids
                        ),
                        front_reading_id=reading_id,
                        source_relation_id=relation.corner_relation_id.value,
                        profile_support_ordinal=ordinal,
                        source_lineage_ids=frozenset(
                            item.value for item in spec.source_lineage_ids
                        ),
                        arrival_law=law,
                        component_profile_arrival_laws=profile_laws,
                        active_segments=tuple(
                            sorted(active, key=lambda item: item.segment_id)
                        ),
                        active_regions=instance.regions,
                        effective_alpha=resolved.effective_alpha,
                        reachability=resolved.reachability,
                        exposed_on_component_boundary=True,
                    )
                )
            continue

        if isinstance(spec, CapEnvelopeSpec):
            incident_models = strip_models_by_spec.get(
                spec.incident_strip_spec_id.value, ()
            )
            if not incident_models:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_MISSING_FRONT_READING,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        f"Cap {spec_id} has no incident Strip arrival reading",
                        frozenset({component.interaction_component_id}),
                    )
                )
                continue
            source_segments = context.support_segments_for_use(
                spec.incident_chain_use_id, spec.incident_strip_spec_id.value
            )
            source = (
                source_segments[0]
                if spec.endpoint_role.value == "START"
                else source_segments[-1]
            )
            constant = _line_constant(source.owner_normal, source.start)
            law_key = _law_key(source.owner_normal, constant)
            incident = next(
                (
                    item
                    for item in incident_models
                    if _law_key(
                        item.arrival_law.normal,
                        item.arrival_law.source_constant,
                    )
                    == law_key
                ),
                incident_models[0],
            )
            terminal_support_id = stable_id(
                "terminal-support",
                source.support_id,
                spec.endpoint_role.value.lower(),
            )
            active = tuple(
                segment
                for segment in component_boundary
                if terminal_support_id in segment.support_ids
            )
            if not active:
                continue
            models.append(
                CapArrivalModelV1(
                    arrival_model_id=stable_id(
                        "cap-arrival-model",
                        component.interaction_component_id,
                        spec_id,
                    ),
                    model_kind=ArrivalModelKind.CAP,
                    interaction_component_id=component.interaction_component_id,
                    decal_request_id=spec.decal_request_id.value,
                    patch_domain_id=spec.patch_domain_id.value,
                    envelope_spec_id=spec_id,
                    envelope_instance_id=instance.envelope_instance_id,
                    incident_strip_spec_id=spec.incident_strip_spec_id.value,
                    front_component_id=incident.front_component_id,
                    front_reading_id=incident.front_reading_id,
                    source_lineage_ids=frozenset(
                        item.value for item in spec.source_lineage_ids
                    ),
                    arrival_law=incident.arrival_law,
                    active_segments=tuple(
                        sorted(active, key=lambda item: item.segment_id)
                    ),
                    active_regions=incident.active_regions,
                    effective_alpha=incident.effective_alpha,
                    reachability=incident.reachability,
                    inherits_incident_strip_reading=True,
                )
            )

    return (
        tuple(sorted(models, key=lambda item: item.arrival_model_id)),
        tuple(diagnostics),
    )


def front_arrival_reading(model) -> FrontArrivalReadingV1:
    return FrontArrivalReadingV1(
        front_reading_id=model.front_reading_id,
        arrival_model_id=model.arrival_model_id,
        interaction_component_id=model.interaction_component_id,
        envelope_spec_id=model.envelope_spec_id,
        envelope_instance_id=model.envelope_instance_id,
        arrival_law=model.arrival_law,
        effective_alpha=model.effective_alpha,
        active_segment_ids=tuple(
            sorted(segment.segment_id for segment in model.active_segments)
        ),
    )


def active_domain_certificate(model, locus_reachable: bool) -> ActiveDomainCertificateV1:
    return ActiveDomainCertificateV1(
        certificate_id=stable_id(
            "active-domain-certificate",
            model.arrival_model_id,
            *(sorted(segment.segment_id for segment in model.active_segments)),
        ),
        arrival_model_id=model.arrival_model_id,
        active_region_ids=tuple(
            sorted(region.region_id for region in model.active_regions)
        ),
        active_segment_ids=tuple(
            sorted(segment.segment_id for segment in model.active_segments)
        ),
        locus_reachable=locus_reachable,
    )
