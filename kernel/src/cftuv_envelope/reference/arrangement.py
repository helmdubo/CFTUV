"""Deterministic exact segment arrangement with curve history.

This module performs no polygon Boolean followed by provenance recovery.  It
splits every supplied segment at exact intersections, merges coincident atomic
histories, classifies both infinitesimal sides from oriented input boundaries,
and emits precisely those atomic edges separating coverage from non-coverage.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cmp_to_key

import sympy as sp

from .arrangement_protocol import ArrangementUnionV1
from .common import stable_id
from .contracts import (
    RawCoverageEdgeV1,
    RawCoverageLoopKind,
    RawCoverageLoopV1,
    RawCoverageRegionV1,
    RawCoverageVertexV1,
    ReachabilityCertificateV1,
)
from .planar_types import (
    BoundedSupportSegment,
    CertifiedPredicateUndecidable,
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
    cross,
    dot,
    exact_sign,
    point_add,
    point_key,
    point_sub,
    points_equal,
    polygon_signed_area,
    vector_scale,
)
from .provenance import (
    ReferenceProvenanceV1,
    merge_boundary_generator_provenance,
    merge_coverage_contributor_provenance,
    merge_provenance,
)


class ExactArrangementNonManifold(CertifiedPredicateUndecidable):
    """Exact construction history proves a non-manifold output boundary."""


@dataclass(frozen=True, slots=True)
class _InputBoundary:
    segment: BoundedSupportSegment
    region: PlanarRegion
    is_domain: bool
    semantic_key: tuple[str, ...]
    aabb: ExactSegmentAabbV1


@dataclass(frozen=True, slots=True)
class ExactSegmentAabbV1:
    """Exact closed AABB used only to reject impossible segment pairs."""

    min_x: ExactScalar
    max_x: ExactScalar
    min_y: ExactScalar
    max_y: ExactScalar


@dataclass(frozen=True, slots=True)
class ArrangementBuildCountersV1:
    input_segments: int
    all_possible_pairs: int
    broadphase_candidate_pairs: int
    narrowphase_tests: int
    actual_intersections: int
    atomic_edges: int


@dataclass(frozen=True, slots=True)
class _ArrangementBuildState:
    boundaries: tuple[_InputBoundary, ...]
    atomic: dict[tuple[tuple[str, str], tuple[str, str]], _AtomicHistory]
    counters: ArrangementBuildCountersV1


@dataclass(slots=True)
class _PointHistory:
    point: ExactPlanarPoint
    certificates: set[ConstructionCertificate]


@dataclass(slots=True)
class _AtomicHistory:
    start: ExactPlanarPoint
    end: ExactPlanarPoint
    source_boundaries: list[_InputBoundary]
    start_certificates: set[ConstructionCertificate]
    end_certificates: set[ConstructionCertificate]


@dataclass(frozen=True, slots=True)
class _OrientedOutput:
    start: ExactPlanarPoint
    end: ExactPlanarPoint
    provenance: ReferenceProvenanceV1
    start_certificates: frozenset[ConstructionCertificate]
    end_certificates: frozenset[ConstructionCertificate]
    reachability: ReachabilityCertificateV1


def _compare(left: sp.Expr, right: sp.Expr) -> int:
    return exact_sign(left - right)


def _exact_ordered_pair(left: sp.Expr, right: sp.Expr) -> tuple[sp.Expr, sp.Expr]:
    return (left, right) if _compare(left, right) <= 0 else (right, left)


def _segment_aabb(segment: BoundedSupportSegment) -> ExactSegmentAabbV1:
    start_x, start_y = segment.start.expressions()
    end_x, end_y = segment.end.expressions()
    min_x, max_x = _exact_ordered_pair(start_x, end_x)
    min_y, max_y = _exact_ordered_pair(start_y, end_y)
    return ExactSegmentAabbV1(
        min_x=ExactScalar.from_value(min_x),
        max_x=ExactScalar.from_value(max_x),
        min_y=ExactScalar.from_value(min_y),
        max_y=ExactScalar.from_value(max_y),
    )


def _aabbs_overlap(left: ExactSegmentAabbV1, right: ExactSegmentAabbV1) -> bool:
    return (
        _compare(left.max_x.as_expr(), right.min_x.as_expr()) >= 0
        and _compare(right.max_x.as_expr(), left.min_x.as_expr()) >= 0
        and _compare(left.max_y.as_expr(), right.min_y.as_expr()) >= 0
        and _compare(right.max_y.as_expr(), left.min_y.as_expr()) >= 0
    )


def _between(value: sp.Expr, lower: sp.Expr, upper: sp.Expr) -> bool:
    return _compare(value, lower) >= 0 and _compare(value, upper) <= 0


def _point_on_segment(point: ExactPlanarPoint, segment: BoundedSupportSegment) -> bool:
    direction = point_sub(segment.end, segment.start)
    offset = point_sub(point, segment.start)
    if exact_sign(cross(direction, offset)) != 0:
        return False
    parameter_numerator = dot(offset, direction)
    parameter_denominator = dot(direction, direction)
    return _between(parameter_numerator, sp.Integer(0), parameter_denominator)


def _intersection_certificate(
    left: BoundedSupportSegment, right: BoundedSupportSegment
) -> ConstructionCertificate:
    left_boundary = bool(left.boundary_constraint_ids)
    right_boundary = bool(right.boundary_constraint_ids)
    if left_boundary and right_boundary:
        kind = ConstructionKind.BOUNDARY_BOUNDARY_INTERSECTION
    elif left_boundary or right_boundary:
        kind = ConstructionKind.SUPPORT_BOUNDARY_INTERSECTION
    else:
        kind = ConstructionKind.SUPPORT_SUPPORT_INTERSECTION
    left_provenance = left.provenance
    right_provenance = right.provenance
    return ConstructionCertificate(
        kind=kind,
        support_ids=left.support_ids | right.support_ids,
        boundary_constraint_ids=left.boundary_constraint_ids
        | right.boundary_constraint_ids,
        physical_edge_ids=frozenset(
            left_provenance.physical_edge_ids | right_provenance.physical_edge_ids
        ),
    )


def segment_intersections(
    left: BoundedSupportSegment, right: BoundedSupportSegment
) -> tuple[ExactPlanarPoint, ...]:
    p = left.start
    q = right.start
    r = point_sub(left.end, left.start)
    s = point_sub(right.end, right.start)
    denominator = cross(r, s)
    q_minus_p = point_sub(q, p)
    if exact_sign(denominator) != 0:
        t = sp.factor(cross(q_minus_p, s) / denominator)
        u = sp.factor(cross(q_minus_p, r) / denominator)
        if _between(t, sp.Integer(0), sp.Integer(1)) and _between(
            u, sp.Integer(0), sp.Integer(1)
        ):
            return (point_add(p, vector_scale(r, t)),)
        return ()
    if exact_sign(cross(q_minus_p, r)) != 0:
        return ()
    candidates = []
    for point in (left.start, left.end, right.start, right.end):
        if _point_on_segment(point, left) and _point_on_segment(point, right):
            if not any(points_equal(point, existing) for existing in candidates):
                candidates.append(point)
    return tuple(candidates)


def _parameter(segment: BoundedSupportSegment, point: ExactPlanarPoint) -> sp.Expr:
    direction = point_sub(segment.end, segment.start)
    return sp.factor(dot(point_sub(point, segment.start), direction) / dot(direction, direction))


def _point_in_loop(point: ExactPlanarPoint, loop: PlanarLoop) -> int:
    """Return 1 inside, 0 on boundary, -1 outside using exact winding."""

    winding = 0
    px, py = point.expressions()
    for segment in loop.segments:
        if _point_on_segment(point, segment):
            return 0
        sx, sy = segment.start.expressions()
        ex, ey = segment.end.expressions()
        upward = _compare(sy, py) <= 0 and _compare(ey, py) > 0
        downward = _compare(ey, py) <= 0 and _compare(sy, py) > 0
        side = exact_sign(cross(point_sub(segment.end, segment.start), point_sub(point, segment.start)))
        if upward and side > 0:
            winding += 1
        elif downward and side < 0:
            winding -= 1
    return 1 if winding else -1


def _point_in_region(point: ExactPlanarPoint, region: PlanarRegion) -> int:
    outer = _point_in_loop(point, region.outer)
    if outer <= 0:
        return outer
    for hole in region.holes:
        state = _point_in_loop(point, hole)
        if state == 0:
            return 0
        if state > 0:
            return -1
    return 1


def _same_direction(
    start: ExactPlanarPoint,
    end: ExactPlanarPoint,
    segment: BoundedSupportSegment,
) -> bool:
    return exact_sign(dot(point_sub(end, start), point_sub(segment.end, segment.start))) > 0


def _side_membership(
    region: PlanarRegion, start: ExactPlanarPoint, end: ExactPlanarPoint
) -> tuple[bool, bool]:
    midpoint = ExactPlanarPoint.from_values(
        (start.x.as_expr() + end.x.as_expr()) / 2,
        (start.y.as_expr() + end.y.as_expr()) / 2,
    )
    matching = [
        segment
        for loop in (region.outer, *region.holes)
        for segment in loop.segments
        if _point_on_segment(midpoint, segment)
        and exact_sign(cross(point_sub(end, start), point_sub(segment.end, segment.start))) == 0
    ]
    if matching:
        directions = {_same_direction(start, end, item) for item in matching}
        if len(directions) != 1:
            raise CertifiedPredicateUndecidable("coincident region boundaries disagree in orientation")
        interior_left = next(iter(directions))
        return interior_left, not interior_left
    inside = _point_in_region(midpoint, region) > 0
    return inside, inside


def _merge_reachability(
    instance_ids: frozenset[str],
    reachability_by_instance: dict[str, ReachabilityCertificateV1],
) -> ReachabilityCertificateV1:
    items = [reachability_by_instance[item] for item in instance_ids if item in reachability_by_instance]
    if not items:
        return ReachabilityCertificateV1(frozenset(), True, 0, 0, (), False)
    return ReachabilityCertificateV1(
        front_component_ids=frozenset().union(*(item.front_component_ids for item in items)),
        source_launch_reachable=all(item.source_launch_reachable for item in items),
        initial_branch_count=sum(item.initial_branch_count for item in items),
        effective_branch_count=sum(item.effective_branch_count for item in items),
        boundary_event_keys=tuple(
            sorted({event for item in items for event in item.boundary_event_keys})
        ),
        bypass_used=any(item.bypass_used for item in items),
    )


def _certificate_key(certificate: ConstructionCertificate) -> str:
    parts = (
        certificate.kind.value,
        *sorted(certificate.source_vertex_ids),
        *sorted(certificate.support_ids),
        *sorted(certificate.boundary_constraint_ids),
        *sorted(certificate.physical_edge_ids),
        certificate.event_key or "",
    )
    return "|".join(parts)


def _vertex_id(certificates: frozenset[ConstructionCertificate]) -> str:
    if not certificates:
        raise CertifiedPredicateUndecidable("arrangement vertex has no construction history")
    return stable_id("raw-vertex", *sorted(_certificate_key(item) for item in certificates))


class ExactSegmentArrangementBackend:
    backend_identity = "CFTUV_EXACT_SEGMENT_ARRANGEMENT_WITH_HISTORY"
    backend_version = "2.0.0"

    def _input_boundaries(
        self,
        contribution_regions: tuple[PlanarRegion, ...],
        domain_regions: tuple[PlanarRegion, ...],
    ) -> tuple[_InputBoundary, ...]:
        boundaries = []
        semantic_keys = set()
        for is_domain, regions in (
            (False, contribution_regions),
            (True, domain_regions),
        ):
            for region in regions:
                for loop in (region.outer, *region.holes):
                    for segment in loop.segments:
                        semantic_key = (
                            segment.segment_id,
                            "DOMAIN" if is_domain else "CONTRIBUTION",
                            region.region_id,
                            loop.loop_id,
                            *point_key(segment.start),
                            *point_key(segment.end),
                        )
                        if semantic_key in semantic_keys:
                            raise ValueError(
                                "duplicate semantic arrangement segment identity: "
                                + "|".join(semantic_key)
                            )
                        semantic_keys.add(semantic_key)
                        boundaries.append(
                            _InputBoundary(
                                segment=segment,
                                region=region,
                                is_domain=is_domain,
                                semantic_key=semantic_key,
                                aabb=_segment_aabb(segment),
                            )
                        )
        return tuple(sorted(boundaries, key=lambda item: item.semantic_key))

    @staticmethod
    def _sweep_order_compare(
        left: tuple[int, _InputBoundary],
        right: tuple[int, _InputBoundary],
    ) -> int:
        comparison = _compare(
            left[1].aabb.min_x.as_expr(),
            right[1].aabb.min_x.as_expr(),
        )
        if comparison:
            return comparison
        return (left[1].semantic_key > right[1].semantic_key) - (
            left[1].semantic_key < right[1].semantic_key
        )

    def _candidate_pairs(
        self,
        boundaries: tuple[_InputBoundary, ...],
    ) -> tuple[tuple[int, int], ...]:
        ordered = sorted(
            enumerate(boundaries),
            key=cmp_to_key(self._sweep_order_compare),
        )
        active: list[tuple[int, _InputBoundary]] = []
        candidates = []
        for current_index, current in ordered:
            active = [
                item
                for item in active
                if _compare(
                    item[1].aabb.max_x.as_expr(),
                    current.aabb.min_x.as_expr(),
                )
                >= 0
            ]
            for active_index, active_boundary in active:
                if not _aabbs_overlap(active_boundary.aabb, current.aabb):
                    continue
                pair = tuple(sorted((active_index, current_index)))
                candidates.append(pair)
            active.append((current_index, current))
        return tuple(
            sorted(
                candidates,
                key=lambda pair: (
                    boundaries[pair[0]].semantic_key,
                    boundaries[pair[1]].semantic_key,
                ),
            )
        )

    def build_arrangement(
        self,
        contribution_regions: tuple[PlanarRegion, ...],
        domain_regions: tuple[PlanarRegion, ...],
    ) -> _ArrangementBuildState:
        boundaries = self._input_boundaries(
            contribution_regions,
            domain_regions,
        )
        split_points: dict[int, list[_PointHistory]] = {}
        for index, boundary in enumerate(boundaries):
            split_points[index] = [
                _PointHistory(boundary.segment.start, set(boundary.segment.start_constructions)),
                _PointHistory(boundary.segment.end, set(boundary.segment.end_constructions)),
            ]
        intersection_count = 0
        candidate_pairs = self._candidate_pairs(boundaries)
        for left_index, right_index in candidate_pairs:
            left = boundaries[left_index]
            right = boundaries[right_index]
            intersections = segment_intersections(left.segment, right.segment)
            if intersections:
                intersection_count += len(intersections)
            certificate = _intersection_certificate(left.segment, right.segment)
            for point in intersections:
                for index in (left_index, right_index):
                    existing = next(
                        (
                            item
                            for item in split_points[index]
                            if points_equal(item.point, point)
                        ),
                        None,
                    )
                    if existing is None:
                        split_points[index].append(_PointHistory(point, {certificate}))
                    else:
                        existing.certificates.add(certificate)

        atomic: dict[tuple[tuple[str, str], tuple[str, str]], _AtomicHistory] = {}
        for index, boundary in enumerate(boundaries):
            points = sorted(
                split_points[index],
                key=cmp_to_key(
                    lambda left, right: _compare(
                        _parameter(boundary.segment, left.point),
                        _parameter(boundary.segment, right.point),
                    )
                ),
            )
            for left, right in zip(points, points[1:]):
                if points_equal(left.point, right.point):
                    continue
                left_key = point_key(left.point)
                right_key = point_key(right.point)
                key = tuple(sorted((left_key, right_key)))
                history = atomic.get(key)
                if history is None:
                    if left_key <= right_key:
                        history = _AtomicHistory(
                            left.point,
                            right.point,
                            [],
                            set(left.certificates),
                            set(right.certificates),
                        )
                    else:
                        history = _AtomicHistory(
                            right.point,
                            left.point,
                            [],
                            set(right.certificates),
                            set(left.certificates),
                        )
                    atomic[key] = history
                history.source_boundaries.append(boundary)
                if points_equal(history.start, left.point):
                    history.start_certificates.update(left.certificates)
                    history.end_certificates.update(right.certificates)
                else:
                    history.start_certificates.update(right.certificates)
                    history.end_certificates.update(left.certificates)
        input_segment_count = len(boundaries)
        counters = ArrangementBuildCountersV1(
            input_segments=input_segment_count,
            all_possible_pairs=(
                input_segment_count * (input_segment_count - 1) // 2
            ),
            broadphase_candidate_pairs=len(candidate_pairs),
            narrowphase_tests=len(candidate_pairs),
            actual_intersections=intersection_count,
            atomic_edges=len(atomic),
        )
        return _ArrangementBuildState(
            boundaries=boundaries,
            atomic=atomic,
            counters=counters,
        )

    def exact_union(
        self,
        contribution_regions: tuple[PlanarRegion, ...],
        domain_regions: tuple[PlanarRegion, ...],
        reachability_by_instance: dict[str, ReachabilityCertificateV1],
    ) -> ArrangementUnionV1:
        build = self.build_arrangement(
            contribution_regions, domain_regions
        )
        atomic = build.atomic
        outputs = []
        for history in atomic.values():
            contribution_left = []
            contribution_right = []
            domain_left = []
            domain_right = []
            for region in contribution_regions:
                left, right = _side_membership(region, history.start, history.end)
                if left:
                    contribution_left.append(region)
                if right:
                    contribution_right.append(region)
            for region in domain_regions:
                left, right = _side_membership(region, history.start, history.end)
                if left:
                    domain_left.append(region)
                if right:
                    domain_right.append(region)
            coverage_left = bool(contribution_left) and bool(domain_left)
            coverage_right = bool(contribution_right) and bool(domain_right)
            if coverage_left == coverage_right:
                continue
            inside_contributions = contribution_left if coverage_left else contribution_right
            inside_domains = domain_left if coverage_left else domain_right
            boundary_generator = merge_boundary_generator_provenance(
                *(
                    boundary.segment.provenance.boundary_generator
                    for boundary in history.source_boundaries
                )
            )
            coverage_contributors = merge_coverage_contributor_provenance(
                *(
                    segment.provenance.coverage_contributors
                    for region in (*inside_contributions, *inside_domains)
                    for segment in region.outer.segments[:1]
                )
            )
            provenance = ReferenceProvenanceV1(
                boundary_generator=boundary_generator,
                coverage_contributors=coverage_contributors,
            )
            instance_ids = frozenset(
                item
                for region in inside_contributions
                for item in region.contributor_instance_ids
            )
            output = _OrientedOutput(
                start=history.start if coverage_left else history.end,
                end=history.end if coverage_left else history.start,
                provenance=provenance,
                start_certificates=frozenset(
                    history.start_certificates
                    if coverage_left
                    else history.end_certificates
                ),
                end_certificates=frozenset(
                    history.end_certificates
                    if coverage_left
                    else history.start_certificates
                ),
                reachability=_merge_reachability(
                    instance_ids, reachability_by_instance
                ),
            )
            outputs.append(output)

        point_certificates: dict[tuple[str, str], set[ConstructionCertificate]] = {}
        point_provenance: dict[tuple[str, str], list[ReferenceProvenanceV1]] = {}
        for output in outputs:
            for point, certificates in (
                (output.start, output.start_certificates),
                (output.end, output.end_certificates),
            ):
                key = point_key(point)
                point_certificates.setdefault(key, set()).update(certificates)
                point_provenance.setdefault(key, []).append(output.provenance)
        vertex_id_by_key = {
            key: _vertex_id(frozenset(certificates))
            for key, certificates in point_certificates.items()
        }
        vertices = frozenset(
            RawCoverageVertexV1(
                vertex_id=vertex_id_by_key[key],
                point=next(
                    point
                    for output in outputs
                    for point in (output.start, output.end)
                    if point_key(point) == key
                ),
                construction_certificates=frozenset(point_certificates[key]),
                provenance=merge_provenance(*point_provenance[key]),
            )
            for key in vertex_id_by_key
        )
        raw_edges = []
        output_by_edge_id = {}
        for output in outputs:
            start_id = vertex_id_by_key[point_key(output.start)]
            end_id = vertex_id_by_key[point_key(output.end)]
            edge_id = stable_id(
                "raw-edge",
                start_id,
                end_id,
                *sorted(output.provenance.support_ids),
                *sorted(output.provenance.boundary_constraint_ids),
            )
            edge = RawCoverageEdgeV1(
                edge_id=edge_id,
                start_vertex_id=start_id,
                end_vertex_id=end_id,
                provenance=output.provenance,
                construction_certificates=output.start_certificates
                | output.end_certificates,
                reachability=output.reachability,
            )
            raw_edges.append(edge)
            output_by_edge_id[edge_id] = output

        outgoing = {}
        edge_by_start_end = {}
        for edge in raw_edges:
            outgoing.setdefault(edge.start_vertex_id, []).append(edge)
            edge_by_start_end[(edge.start_vertex_id, edge.end_vertex_id)] = edge
        if any(len(items) != 1 for items in outgoing.values()):
            raise ExactArrangementNonManifold(
                "exact union boundary is non-manifold at a construction vertex"
            )
        unvisited = {edge.edge_id for edge in raw_edges}
        loops = []
        loop_points = {}
        while unvisited:
            first_id = min(unvisited)
            first = next(item for item in raw_edges if item.edge_id == first_id)
            ordered = []
            points = []
            current = first
            while current.edge_id in unvisited:
                unvisited.remove(current.edge_id)
                ordered.append(current.edge_id)
                output = output_by_edge_id[current.edge_id]
                points.append(output.start)
                candidates = outgoing.get(current.end_vertex_id, ())
                if len(candidates) != 1:
                    raise CertifiedPredicateUndecidable(
                        "open/non-manifold exact union boundary"
                    )
                current = candidates[0]
            if current.edge_id != first.edge_id:
                raise CertifiedPredicateUndecidable("boundary traversal did not close")
            area = polygon_signed_area(points)
            sign = exact_sign(area)
            if sign == 0:
                raise CertifiedPredicateUndecidable("zero-area output loop")
            loop_id = stable_id("raw-loop", *ordered)
            loop = RawCoverageLoopV1(
                loop_id=loop_id,
                kind=RawCoverageLoopKind.OUTER if sign > 0 else RawCoverageLoopKind.HOLE,
                ordered_edge_ids=tuple(ordered),
                signed_area=ExactScalar.from_value(area).expression,
            )
            loops.append(loop)
            loop_points[loop_id] = tuple(points)

        outer_loops = [item for item in loops if item.kind is RawCoverageLoopKind.OUTER]
        hole_loops = [item for item in loops if item.kind is RawCoverageLoopKind.HOLE]
        holes_by_outer = {item.loop_id: [] for item in outer_loops}
        for hole in hole_loops:
            probe = loop_points[hole.loop_id][0]
            containing = [
                outer
                for outer in outer_loops
                if _point_in_loop(
                    probe,
                    PlanarLoop(
                        outer.loop_id,
                        tuple(
                            BoundedSupportSegment(
                                edge.edge_id,
                                output_by_edge_id[edge.edge_id].start,
                                output_by_edge_id[edge.edge_id].end,
                                frozenset(),
                                frozenset(),
                                edge.provenance,
                                edge.construction_certificates,
                                edge.construction_certificates,
                            )
                            for edge_id in outer.ordered_edge_ids
                            for edge in raw_edges
                            if edge.edge_id == edge_id
                        ),
                    ),
                )
                > 0
            ]
            if not containing:
                raise CertifiedPredicateUndecidable("hole loop has no containing outer loop")
            containing.sort(
                key=lambda item: abs(
                    sp.sympify(item.signed_area)
                )
            )
            holes_by_outer[containing[0].loop_id].append(hole)

        edge_by_id = {item.edge_id: item for item in raw_edges}
        regions = []
        total_area = sp.Integer(0)
        for outer in outer_loops:
            holes = holes_by_outer[outer.loop_id]
            edge_ids = (*outer.ordered_edge_ids, *(edge for hole in holes for edge in hole.ordered_edge_ids))
            provenance = merge_provenance(*(edge_by_id[item].provenance for item in edge_ids))
            spec_ids = set(provenance.envelope_spec_ids)
            instance_ids = set(provenance.envelope_instance_ids)
            probe = loop_points[outer.loop_id][0]
            region_contributor_provenance = []
            for contribution in contribution_regions:
                if _point_in_region(probe, contribution) >= 0 or any(
                    _point_in_region(
                        point,
                        PlanarRegion(
                            "probe",
                            PlanarLoop(
                                outer.loop_id,
                                tuple(
                                    BoundedSupportSegment(
                                        edge_by_id[edge_id].edge_id,
                                        output_by_edge_id[edge_id].start,
                                        output_by_edge_id[edge_id].end,
                                        frozenset(),
                                        frozenset(),
                                        edge_by_id[edge_id].provenance,
                                        edge_by_id[
                                            edge_id
                                        ].construction_certificates,
                                        edge_by_id[
                                            edge_id
                                        ].construction_certificates,
                                    )
                                    for edge_id in outer.ordered_edge_ids
                                ),
                            ),
                        ),
                    )
                    >= 0
                    for point in contribution.outer.points
                ):
                    spec_ids.update(contribution.contributor_spec_ids)
                    instance_ids.update(contribution.contributor_instance_ids)
                    region_contributor_provenance.extend(
                        segment.provenance.coverage_contributors
                        for segment in contribution.outer.segments[:1]
                    )
            provenance = ReferenceProvenanceV1(
                boundary_generator=provenance.boundary_generator,
                coverage_contributors=merge_coverage_contributor_provenance(
                    provenance.coverage_contributors,
                    *region_contributor_provenance,
                ),
            )
            regions.append(
                RawCoverageRegionV1(
                    region_id=stable_id("raw-region", outer.loop_id),
                    outer_loop_id=outer.loop_id,
                    hole_loop_ids=tuple(sorted(item.loop_id for item in holes)),
                    contributor_envelope_spec_ids=frozenset(spec_ids),
                    contributor_envelope_instance_ids=frozenset(instance_ids),
                    provenance=provenance,
                )
            )
            total_area += sp.sympify(outer.signed_area)
            total_area += sum(sp.sympify(item.signed_area) for item in holes)

        return ArrangementUnionV1(
            vertices=vertices,
            edges=frozenset(raw_edges),
            loops=frozenset(loops),
            regions=frozenset(regions),
            exact_area_expression=ExactScalar.from_value(total_area).expression,
            input_segment_count=build.counters.input_segments,
            all_possible_pair_count=build.counters.all_possible_pairs,
            broadphase_candidate_pair_count=(
                build.counters.broadphase_candidate_pairs
            ),
            narrowphase_test_count=build.counters.narrowphase_tests,
            intersection_count=build.counters.actual_intersections,
            atomic_edge_count=build.counters.atomic_edges,
        )


class ExhaustiveExactArrangementBackend(ExactSegmentArrangementBackend):
    """Permanent differential oracle using canonical exhaustive pair tests."""

    backend_identity = "CFTUV_EXACT_SEGMENT_ARRANGEMENT_EXHAUSTIVE_ORACLE"
    backend_version = "1.0.0"

    def _candidate_pairs(
        self,
        boundaries: tuple[_InputBoundary, ...],
    ) -> tuple[tuple[int, int], ...]:
        return tuple(
            (left_index, right_index)
            for left_index in range(len(boundaries))
            for right_index in range(left_index + 1, len(boundaries))
        )
