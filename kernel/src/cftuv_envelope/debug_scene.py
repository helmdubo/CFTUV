"""Immutable Blender-free diagnostic projection of accepted Envelope results."""

from __future__ import annotations

import hashlib
import math
from typing import Iterable

from .contracts.analysis import (
    AnalysisSnapshotV1,
    BoundaryLoopConstraintTargetV1,
    ChainUseConstraintTargetV1,
    ChainUseOrientation,
    PhysicalEdgeSequenceConstraintTargetV1,
    PlanarPatchFrameV1,
)
from .contracts.envelopes import AngularEnvelopeSpec, StripEnvelopeSpec
from .contracts.metric import RationalAffinePlanarMetricV2
from .contracts.request import DecalRequestV1
from .contracts.debug import (
    ENVELOPE_DEBUG_SCENE_SCHEMA_V1,
    DebugDiagnosticSeverity,
    DebugDiagnosticV1,
    DebugExactPoint2V1,
    DebugLabelV1,
    DebugLoopV1,
    DebugPathV1,
    DebugPatchFrameV1,
    DebugPointV1,
    DebugPrimitiveKind,
    DebugRegionV1,
    EnvelopeDebugSceneV1,
    EnvelopeDebugStage,
)
from .ids import (
    DebugDiagnosticId,
    DebugPrimitiveId,
    DecalRequestId,
    LineageId,
    PatchDomainId,
    SourceRevision,
)
from .numeric import LocalPoint3V1, LocalVector3V1
from .planar_metric import build_runtime_planar_metric
from .interactions.contracts import (
    AngularProfileArrivalModelV1,
    CapArrivalModelV1,
    InteractionResolutionResultV1,
    ResolvedCoverageResultV1,
    StripArrivalModelV1,
)
from .reference.contracts import (
    RawCoverageResultV1,
    ReferenceEnvelopeCompilationV1,
)
from .reference.planar_types import (
    ExactPlanarPoint,
    ExactScalar,
    PlanarRegion,
)


def _stable_id(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(part) for part in parts)))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _primitive_id(kind: str, *parts: object) -> DebugPrimitiveId:
    return DebugPrimitiveId(f"debug:{kind}:{_stable_id(kind, *parts)[:24]}")


def _diagnostic_id(kind: str, *parts: object) -> DebugDiagnosticId:
    return DebugDiagnosticId(
        f"debug-diagnostic:{kind}:{_stable_id(kind, *parts)[:24]}"
    )


def _debug_point(point: ExactPlanarPoint) -> DebugExactPoint2V1:
    return DebugExactPoint2V1(point.x.expression, point.y.expression)


def _local_debug_point(point) -> DebugExactPoint2V1:
    return DebugExactPoint2V1(
        ExactScalar.from_value(point.x).expression,
        ExactScalar.from_value(point.y).expression,
    )


def _lineage(values: Iterable[str]) -> frozenset[LineageId]:
    return frozenset(LineageId(value) for value in values)


def _region_record(
    *,
    semantic_key: str,
    patch_domain_id: PatchDomainId,
    stage: EnvelopeDebugStage,
    kind: DebugPrimitiveKind,
    region: PlanarRegion,
    lineage: frozenset[LineageId],
    style_key: str,
    label: str,
) -> DebugRegionV1:
    return DebugRegionV1(
        semantic_id=_primitive_id(kind.value, semantic_key, region.region_id),
        patch_domain_id=patch_domain_id,
        stage=stage,
        kind=kind,
        outer_exact_points=tuple(_debug_point(point) for point in region.outer.points),
        hole_exact_point_loops=tuple(
            tuple(_debug_point(point) for point in hole.points)
            for hole in region.holes
        ),
        closed=True,
        source_lineage_ids=lineage,
        style_key=style_key,
        label=label,
    )


def _raw_loop_points(result, loop) -> tuple[DebugExactPoint2V1, ...]:
    edge_by_id = {edge.edge_id: edge for edge in result.edges}
    vertex_by_id = {vertex.vertex_id: vertex for vertex in result.vertices}
    return tuple(
        _debug_point(vertex_by_id[edge_by_id[edge_id].start_vertex_id].point)
        for edge_id in loop.ordered_edge_ids
    )


def _frame_points(snapshot: AnalysisSnapshotV1):
    result = {}
    for frame in snapshot.surface_metric_descriptors:
        if isinstance(frame, PlanarPatchFrameV1):
            result[frame.patch_domain_id] = {
                item.source_vertex_id: _local_debug_point(
                    item.domain_coordinate
                )
                for item in frame.source_vertex_coordinates
            }
        elif isinstance(frame, RationalAffinePlanarMetricV2):
            result[frame.patch_domain_id] = {
                item.source_vertex_id: DebugExactPoint2V1(
                    str(item.domain_coordinate.x.numerator)
                    if item.domain_coordinate.x.denominator == 1
                    else (
                        f"{item.domain_coordinate.x.numerator}/"
                        f"{item.domain_coordinate.x.denominator}"
                    ),
                    str(item.domain_coordinate.y.numerator)
                    if item.domain_coordinate.y.denominator == 1
                    else (
                        f"{item.domain_coordinate.y.numerator}/"
                        f"{item.domain_coordinate.y.denominator}"
                    ),
                )
                for item in frame.exact_source_vertex_coordinates
            }
    return result


def _debug_patch_frame(frame) -> DebugPatchFrameV1:
    if isinstance(frame, PlanarPatchFrameV1):
        return DebugPatchFrameV1(
            frame.patch_domain_id,
            frame.origin,
            frame.axis_u,
            frame.axis_v,
            frame.normal,
        )
    view = build_runtime_planar_metric(frame).derived_binary64_view
    a = view.basis_a
    b = view.basis_b
    normal = (
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
    length = math.sqrt(sum(item * item for item in normal))
    return DebugPatchFrameV1(
        frame.patch_domain_id,
        LocalPoint3V1(view.origin.x, view.origin.y, view.origin.z),
        LocalVector3V1(a.x, a.y, a.z),
        LocalVector3V1(b.x, b.y, b.z),
        LocalVector3V1(*(item / length for item in normal)),
    )


def _directed_chain_points(snapshot, frame_points, chain_use):
    chain = next(
        item
        for item in snapshot.physical_chains
        if item.physical_chain_id == chain_use.physical_chain_id
    )
    vertex_ids = chain.ordered_source_vertex_ids
    if chain_use.orientation is ChainUseOrientation.B_START_TO_END:
        vertex_ids = tuple(reversed(vertex_ids))
    return tuple(frame_points[chain_use.patch_domain_id][item] for item in vertex_ids)


def _loop_points(snapshot, frame_points, loop):
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    points: list[DebugExactPoint2V1] = []
    for use_id in loop.ordered_chain_use_ids:
        use_points = list(
            _directed_chain_points(snapshot, frame_points, use_by_id[use_id])
        )
        if points and use_points and points[-1] == use_points[0]:
            use_points = use_points[1:]
        points.extend(use_points)
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return tuple(points)


def _constraint_paths(snapshot, frame_points, constraint):
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    loop_by_id = {item.boundary_loop_id: item for item in snapshot.boundary_loops}
    edge_by_id = {item.edge_id: item for item in snapshot.surface_ir.source_edges}
    target = constraint.target
    if isinstance(target, ChainUseConstraintTargetV1):
        use = use_by_id[target.chain_use_id]
        return (_directed_chain_points(snapshot, frame_points, use),)
    if isinstance(target, BoundaryLoopConstraintTargetV1):
        return ((_loop_points(snapshot, frame_points, loop_by_id[target.boundary_loop_id])),)
    if isinstance(target, PhysicalEdgeSequenceConstraintTargetV1):
        result = []
        points = frame_points[constraint.patch_domain_id]
        for edge_id in target.ordered_physical_edge_ids:
            edge = edge_by_id[edge_id]
            result.append((points[edge.vertex_a_id], points[edge.vertex_b_id]))
        return tuple(result)
    return ()


def _diagnostic_severity(value: str) -> DebugDiagnosticSeverity:
    if value == "INFO":
        return DebugDiagnosticSeverity.INFO
    if value == "CAPACITY":
        return DebugDiagnosticSeverity.CAPACITY
    return DebugDiagnosticSeverity.UNSUPPORTED


def validate_envelope_debug_scene(scene: EnvelopeDebugSceneV1) -> tuple[str, ...]:
    issues: list[str] = []
    if scene.schema_version != ENVELOPE_DEBUG_SCENE_SCHEMA_V1:
        issues.append("ENVELOPE_DEBUG_SCENE_SCHEMA_UNSUPPORTED")
    domains = set(scene.patch_domain_ids)
    frame_domains = [item.patch_domain_id for item in scene.patch_frames]
    if len(frame_domains) != len(set(frame_domains)):
        issues.append("ENVELOPE_DEBUG_SCENE_DUPLICATE_PATCH_FRAME")
    if set(frame_domains) != domains:
        issues.append("ENVELOPE_DEBUG_SCENE_PATCH_FRAME_DOMAIN_MISMATCH")
    records = (*scene.points, *scene.paths, *scene.loops, *scene.regions, *scene.labels)
    identities = [record.semantic_id for record in records]
    if len(identities) != len(set(identities)):
        issues.append("ENVELOPE_DEBUG_SCENE_DUPLICATE_SEMANTIC_ID")
    for record in records:
        if record.patch_domain_id not in domains:
            issues.append("ENVELOPE_DEBUG_SCENE_DOMAIN_REFERENCE_MISSING")
    diagnostic_ids = [item.diagnostic_id for item in scene.diagnostics]
    if len(diagnostic_ids) != len(set(diagnostic_ids)):
        issues.append("ENVELOPE_DEBUG_SCENE_DUPLICATE_DIAGNOSTIC_ID")
    for diagnostic in scene.diagnostics:
        if (
            diagnostic.patch_domain_id is not None
            and diagnostic.patch_domain_id not in domains
        ):
            issues.append("ENVELOPE_DEBUG_SCENE_DIAGNOSTIC_DOMAIN_MISSING")
    return tuple(sorted(set(issues)))


def build_envelope_debug_scene(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    compilations: tuple[ReferenceEnvelopeCompilationV1, ...],
    raw_results: tuple[RawCoverageResultV1, ...],
    interaction_results: tuple[InteractionResolutionResultV1, ...],
    diagnostics: tuple[DebugDiagnosticV1, ...] = (),
) -> EnvelopeDebugSceneV1:
    """Project ready semantic results without solving or repairing geometry."""

    points: list[DebugPointV1] = []
    paths: list[DebugPathV1] = []
    loops: list[DebugLoopV1] = []
    regions: list[DebugRegionV1] = []
    labels: list[DebugLabelV1] = []
    scene_diagnostics = list(diagnostics)
    frame_points = _frame_points(snapshot)
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    selected_domain_ids = {
        use_by_id[use_id].patch_domain_id
        for use_id in request.selected_chain_use_ids
    }
    chain_by_id = {
        item.physical_chain_id: item for item in snapshot.physical_chains
    }

    for loop in sorted(
        (
            item
            for item in snapshot.boundary_loops
            if item.patch_domain_id in selected_domain_ids
        ),
        key=lambda item: item.boundary_loop_id.value,
    ):
        exact_points = _loop_points(snapshot, frame_points, loop)
        if len(exact_points) < 3:
            continue
        kind = (
            DebugPrimitiveKind.PATCH_DOMAIN_OUTER
            if loop.kind.value == "OUTER"
            else DebugPrimitiveKind.PATCH_DOMAIN_HOLE
        )
        loops.append(
            DebugLoopV1(
                _primitive_id(kind.value, loop.boundary_loop_id),
                loop.patch_domain_id,
                EnvelopeDebugStage.ANALYSIS,
                kind,
                exact_points,
                True,
                frozenset(),
                "ENV_00_PATCH_DOMAIN" if kind is DebugPrimitiveKind.PATCH_DOMAIN_OUTER else "ENV_01_HOLES",
                loop.boundary_loop_id.value,
            )
        )

    physical_emitted: set[tuple[PatchDomainId, object]] = set()
    for use in sorted(
        (
            item
            for item in snapshot.chain_uses
            if item.patch_domain_id in selected_domain_ids
        ),
        key=lambda item: item.chain_use_id.value,
    ):
        chain = chain_by_id[use.physical_chain_id]
        exact_points = _directed_chain_points(snapshot, frame_points, use)
        physical_key = (use.patch_domain_id, chain.physical_chain_id)
        if physical_key not in physical_emitted:
            physical_emitted.add(physical_key)
            physical_points = tuple(
                frame_points[use.patch_domain_id][vertex_id]
                for vertex_id in chain.ordered_source_vertex_ids
            )
            paths.append(
                DebugPathV1(
                    _primitive_id(
                        DebugPrimitiveKind.PHYSICAL_CHAIN.value,
                        use.patch_domain_id,
                        chain.physical_chain_id,
                    ),
                    use.patch_domain_id,
                    EnvelopeDebugStage.ANALYSIS,
                    DebugPrimitiveKind.PHYSICAL_CHAIN,
                    physical_points,
                    chain.is_closed,
                    chain.source_lineage,
                    "ENV_10_PHYSICAL_CHAINS",
                    chain.physical_chain_id.value,
                )
            )
        paths.append(
            DebugPathV1(
                _primitive_id(DebugPrimitiveKind.CHAIN_USE.value, use.chain_use_id),
                use.patch_domain_id,
                EnvelopeDebugStage.ANALYSIS,
                DebugPrimitiveKind.CHAIN_USE,
                exact_points,
                chain.is_closed,
                chain.source_lineage,
                "ENV_11_CHAIN_USES",
                use.chain_use_id.value,
            )
        )

    for constraint in sorted(
        (
            item
            for item in snapshot.boundary_constraints
            if item.patch_domain_id in selected_domain_ids
        ),
        key=lambda item: item.boundary_constraint_id.value,
    ):
        if constraint.constraint_kind.value != "PHYSICAL_DOMAIN_BARRIER":
            continue
        for ordinal, exact_points in enumerate(
            _constraint_paths(snapshot, frame_points, constraint)
        ):
            if len(exact_points) < 2:
                continue
            paths.append(
                DebugPathV1(
                    _primitive_id(
                        DebugPrimitiveKind.PHYSICAL_BARRIER.value,
                        constraint.boundary_constraint_id,
                        ordinal,
                    ),
                    constraint.patch_domain_id,
                    EnvelopeDebugStage.ANALYSIS,
                    DebugPrimitiveKind.PHYSICAL_BARRIER,
                    exact_points,
                    False,
                    frozenset(),
                    "ENV_02_BARRIERS",
                    constraint.boundary_constraint_id.value,
                )
            )

    raw_by_domain = {item.plan_key.patch_domain_id: item for item in raw_results}
    interaction_by_domain = {
        item.resolved_coverage.plan_key.patch_domain_id: item
        for item in interaction_results
        if item.resolved_coverage is not None
    }

    for compilation in compilations:
        domain_id = compilation.plan_key.patch_domain_id
        hidden_support_ids = {
            item.hidden_support_id.value
            for spec in compilation.envelope_specs
            if isinstance(spec, AngularEnvelopeSpec)
            for item in spec.hidden_supports
        }
        for spec in sorted(
            (
                item
                for item in compilation.envelope_specs
                if isinstance(item, StripEnvelopeSpec)
            ),
            key=lambda item: item.envelope_spec_id.value,
        ):
            seed = next(
                item
                for item in compilation.seeds
                if getattr(item, "seed_id", None) == spec.source_seed_id
            )
            use = use_by_id[seed.chain_use_id]
            chain = chain_by_id[use.physical_chain_id]
            paths.append(
                DebugPathV1(
                    _primitive_id(
                        DebugPrimitiveKind.SOURCE_SUPPORT.value,
                        spec.envelope_spec_id,
                    ),
                    domain_id,
                    EnvelopeDebugStage.COMPILE,
                    DebugPrimitiveKind.SOURCE_SUPPORT,
                    _directed_chain_points(snapshot, frame_points, use),
                    chain.is_closed,
                    chain.source_lineage,
                    "ENV_20_SOURCE_SUPPORTS",
                    spec.envelope_spec_id.value,
                )
            )
        raw = raw_by_domain.get(domain_id)
        if raw is None:
            continue
        for instance in sorted(
            raw.envelope_instances, key=lambda item: item.envelope_instance_id
        ):
            lineage = _lineage(instance.provenance.lineage_ids)
            for ordinal, segment in enumerate(instance.exposed_segments):
                segment_kind = (
                    DebugPrimitiveKind.HIDDEN_SUPPORT
                    if set(segment.support_ids) & hidden_support_ids
                    else DebugPrimitiveKind.MOVING_FRONT
                )
                paths.append(
                    DebugPathV1(
                        _primitive_id(
                            segment_kind.value,
                            instance.envelope_instance_id,
                            segment.segment_id,
                            ordinal,
                        ),
                        domain_id,
                        EnvelopeDebugStage.ENVELOPE_INSTANCE,
                        segment_kind,
                        (_debug_point(segment.start), _debug_point(segment.end)),
                        False,
                        lineage,
                        "ENV_22_HIDDEN_SUPPORTS"
                        if segment_kind is DebugPrimitiveKind.HIDDEN_SUPPORT
                        else "ENV_21_MOVING_FRONTS",
                        segment.segment_id,
                    )
                )
            for region in instance.regions:
                regions.append(
                    _region_record(
                        semantic_key=instance.envelope_instance_id,
                        patch_domain_id=domain_id,
                        stage=EnvelopeDebugStage.ENVELOPE_INSTANCE,
                        kind=DebugPrimitiveKind.ENVELOPE_INSTANCE,
                        region=region,
                        lineage=lineage,
                        style_key="ENV_30_ENVELOPE_INSTANCES",
                        label=instance.envelope_variant,
                    )
                )

        raw_loop_by_id = {item.loop_id: item for item in raw.loops}
        for loop in sorted(raw.loops, key=lambda item: item.loop_id):
            exact_points = _raw_loop_points(raw, loop)
            if len(exact_points) < 3:
                continue
            loops.append(
                DebugLoopV1(
                    _primitive_id(DebugPrimitiveKind.RAW_COVERAGE.value, loop.loop_id),
                    domain_id,
                    EnvelopeDebugStage.RAW_COVERAGE,
                    DebugPrimitiveKind.RAW_COVERAGE,
                    exact_points,
                    True,
                    frozenset(),
                    "ENV_40_RAW_COVERAGE",
                    loop.loop_id,
                )
            )
        for region in raw.regions:
            outer = raw_loop_by_id[region.outer_loop_id]
            hole_points = tuple(
                _raw_loop_points(raw, raw_loop_by_id[item])
                for item in region.hole_loop_ids
            )
            regions.append(
                DebugRegionV1(
                    _primitive_id(DebugPrimitiveKind.RAW_COVERAGE.value, region.region_id),
                    domain_id,
                    EnvelopeDebugStage.RAW_COVERAGE,
                    DebugPrimitiveKind.RAW_COVERAGE,
                    _raw_loop_points(raw, outer),
                    hole_points,
                    True,
                    _lineage(region.provenance.lineage_ids),
                    "ENV_40_RAW_COVERAGE",
                    region.region_id,
                )
            )
        for diagnostic in raw.diagnostics:
            scene_diagnostics.append(
                DebugDiagnosticV1(
                    _diagnostic_id(
                        diagnostic.outcome.value,
                        domain_id,
                        diagnostic.envelope_spec_id,
                        diagnostic.front_component_id,
                        diagnostic.message,
                    ),
                    domain_id,
                    EnvelopeDebugStage.DIAGNOSTIC,
                    DebugPrimitiveKind.DIAGNOSTIC,
                    diagnostic.outcome.value,
                    _diagnostic_severity(diagnostic.severity.value),
                    diagnostic.message,
                    None,
                    frozenset(),
                    "ENV_80_DIAGNOSTICS",
                    diagnostic.outcome.value,
                )
            )

        interaction = interaction_by_domain.get(domain_id)
        if interaction is None or interaction.resolved_coverage is None:
            continue
        resolved = interaction.resolved_coverage
        instance_by_id = {
            item.envelope_instance_id: item for item in raw.envelope_instances
        }
        for component in interaction.interaction_components:
            for instance_id in component.envelope_instance_ids:
                instance = instance_by_id.get(instance_id.value)
                if instance is None:
                    continue
                for region in instance.regions:
                    regions.append(
                        _region_record(
                            semantic_key=component.interaction_component_id.value,
                            patch_domain_id=domain_id,
                            stage=EnvelopeDebugStage.INTERACTION,
                            kind=DebugPrimitiveKind.INTERACTION_COMPONENT,
                            region=region,
                            lineage=component.source_lineage_ids,
                            style_key="ENV_50_INTERACTION_COMPONENTS",
                            label=component.interaction_component_id.value,
                        )
                    )
        for model in interaction.arrival_models:
            if not isinstance(
                model,
                (
                    StripArrivalModelV1,
                    AngularProfileArrivalModelV1,
                    CapArrivalModelV1,
                ),
            ):
                continue
            for ordinal, segment in enumerate(model.active_segments):
                paths.append(
                    DebugPathV1(
                        _primitive_id(
                            DebugPrimitiveKind.FRONT_READING.value,
                            model.front_reading_id,
                            segment.segment_id,
                            ordinal,
                        ),
                        domain_id,
                        EnvelopeDebugStage.INTERACTION,
                        DebugPrimitiveKind.FRONT_READING,
                        (_debug_point(segment.start), _debug_point(segment.end)),
                        False,
                        model.source_lineage_ids,
                        "ENV_51_FRONT_READINGS",
                        model.front_reading_id.value,
                    )
                )
        for locus in resolved.equality_loci:
            for segment in locus.ordered_exact_segments:
                paths.append(
                    DebugPathV1(
                        _primitive_id(
                            DebugPrimitiveKind.EQUALITY_LOCUS.value,
                            locus.locus_id,
                            segment.segment_id,
                        ),
                        domain_id,
                        EnvelopeDebugStage.INTERACTION,
                        DebugPrimitiveKind.EQUALITY_LOCUS,
                        (_debug_point(segment.start), _debug_point(segment.end)),
                        False,
                        frozenset(),
                        "ENV_52_EQUALITY_LOCI",
                        locus.locus_id.value,
                    )
                )
            for ordinal, anchor in enumerate(locus.event_anchor_points):
                exact_anchor = _debug_point(anchor)
                points.append(
                    DebugPointV1(
                        _primitive_id(
                            DebugPrimitiveKind.EVENT_ANCHOR.value,
                            locus.locus_id,
                            ordinal,
                        ),
                        domain_id,
                        EnvelopeDebugStage.INTERACTION,
                        DebugPrimitiveKind.EVENT_ANCHOR,
                        exact_anchor,
                        frozenset(),
                        "ENV_70_EVENT_ANCHORS",
                        locus.locus_id.value,
                    )
                )
                labels.append(
                    DebugLabelV1(
                        _primitive_id(
                            DebugPrimitiveKind.LABEL.value,
                            locus.locus_id,
                            ordinal,
                        ),
                        domain_id,
                        EnvelopeDebugStage.INTERACTION,
                        DebugPrimitiveKind.LABEL,
                        exact_anchor,
                        frozenset(),
                        "ENV_LABELS",
                        locus.locus_id.value,
                    )
                )
        resolved_loop_by_id = {item.loop_id: item for item in resolved.loops}
        for loop in sorted(resolved.loops, key=lambda item: item.loop_id):
            exact_points = _raw_loop_points(resolved, loop)
            if len(exact_points) < 3:
                continue
            loops.append(
                DebugLoopV1(
                    _primitive_id(
                        DebugPrimitiveKind.RESOLVED_COVERAGE.value, loop.loop_id
                    ),
                    domain_id,
                    EnvelopeDebugStage.RESOLVED_COVERAGE,
                    DebugPrimitiveKind.RESOLVED_COVERAGE,
                    exact_points,
                    True,
                    frozenset(),
                    "ENV_60_RESOLVED_COVERAGE",
                    loop.loop_id,
                )
            )
        for region in resolved.regions:
            regions.append(
                DebugRegionV1(
                    _primitive_id(
                        DebugPrimitiveKind.RESOLVED_COVERAGE.value, region.region_id
                    ),
                    domain_id,
                    EnvelopeDebugStage.RESOLVED_COVERAGE,
                    DebugPrimitiveKind.RESOLVED_COVERAGE,
                    _raw_loop_points(resolved, resolved_loop_by_id[region.outer_loop_id]),
                    tuple(
                        _raw_loop_points(resolved, resolved_loop_by_id[item])
                        for item in region.hole_loop_ids
                    ),
                    True,
                    _lineage(region.provenance.lineage_ids),
                    "ENV_60_RESOLVED_COVERAGE",
                    region.region_id,
                )
            )
        for contribution in resolved.resolved_contributions:
            lineage = _lineage(
                contribution.original_envelope_instance.provenance.lineage_ids
            )
            for retained in contribution.retained_exact_regions:
                regions.append(
                    _region_record(
                        semantic_key=contribution.resolved_contribution_id.value,
                        patch_domain_id=domain_id,
                        stage=EnvelopeDebugStage.RESOLVED_COVERAGE,
                        kind=DebugPrimitiveKind.RETAINED_REGION,
                        region=retained,
                        lineage=lineage,
                        style_key="ENV_61_RETAINED_REGIONS",
                        label=contribution.resolved_contribution_id.value,
                    )
                )
            for removed in contribution.removed_exact_regions:
                regions.append(
                    _region_record(
                        semantic_key=contribution.resolved_contribution_id.value,
                        patch_domain_id=domain_id,
                        stage=EnvelopeDebugStage.RESOLVED_COVERAGE,
                        kind=DebugPrimitiveKind.REMOVED_REGION,
                        region=removed,
                        lineage=lineage,
                        style_key="ENV_62_REMOVED_REGIONS",
                        label=contribution.resolved_contribution_id.value,
                    )
                )
        for diagnostic in resolved.diagnostics:
            scene_diagnostics.append(
                DebugDiagnosticV1(
                    _diagnostic_id(
                        diagnostic.outcome.value,
                        domain_id,
                        diagnostic.message,
                    ),
                    domain_id,
                    EnvelopeDebugStage.DIAGNOSTIC,
                    DebugPrimitiveKind.DIAGNOSTIC,
                    diagnostic.outcome.value,
                    _diagnostic_severity(diagnostic.severity.value),
                    diagnostic.message,
                    None,
                    frozenset(),
                    "ENV_80_DIAGNOSTICS",
                    diagnostic.outcome.value,
                )
            )

    resolved_results: tuple[ResolvedCoverageResultV1, ...] = tuple(
        item.resolved_coverage
        for item in interaction_results
        if item.resolved_coverage is not None
    )
    scene = EnvelopeDebugSceneV1(
        schema_version=ENVELOPE_DEBUG_SCENE_SCHEMA_V1,
        source_revision=snapshot.source_revision,
        request_ids=(request.decal_request_id,),
        patch_domain_ids=tuple(
            sorted(selected_domain_ids, key=lambda item: item.value)
        ),
        requested_alpha=request.requested_alpha,
        patch_frames=tuple(
            _debug_patch_frame(frame)
            for frame in sorted(
                (
                    item
                    for item in snapshot.surface_metric_descriptors
                    if isinstance(
                        item,
                        (
                            PlanarPatchFrameV1,
                            RationalAffinePlanarMetricV2,
                        ),
                    )
                    and item.patch_domain_id in selected_domain_ids
                ),
                key=lambda item: item.patch_domain_id.value,
            )
        ),
        points=tuple(sorted(points, key=lambda item: item.semantic_id.value)),
        paths=tuple(sorted(paths, key=lambda item: item.semantic_id.value)),
        loops=tuple(sorted(loops, key=lambda item: item.semantic_id.value)),
        regions=tuple(sorted(regions, key=lambda item: item.semantic_id.value)),
        labels=tuple(sorted(labels, key=lambda item: item.semantic_id.value)),
        diagnostics=tuple(
            sorted(scene_diagnostics, key=lambda item: item.diagnostic_id.value)
        ),
        raw_coverage_digests=tuple(
            item.semantic_digest
            for item in sorted(
                raw_results, key=lambda item: item.plan_key.patch_domain_id.value
            )
        ),
        resolved_coverage_digests=tuple(
            item.semantic_digest
            for item in sorted(
                resolved_results,
                key=lambda item: item.plan_key.patch_domain_id.value,
            )
        ),
    )
    issues = validate_envelope_debug_scene(scene)
    if issues:
        raise ValueError("; ".join(issues))
    return scene
