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

from .arrangement_protocol import ArrangementUnionV2
from .common import stable_id
from .contracts import (
    BoundaryVertexOccurrenceV1,
    PointContactRecordV1,
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
    ExactPlanarVector,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
    cross,
    dot,
    exact_normalize,
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


class ExactArrangementRotationSystemUnproven(CertifiedPredicateUndecidable):
    """Exact embedded boundary does not prove one face-successor per half-edge."""


class ExactArrangementCollinearBranchUnproven(
    ExactArrangementRotationSystemUnproven
):
    """Collinear incident rays remain ambiguous after exact atomic splitting."""


class ExactTouchingHoleTopologyUnproven(ExactArrangementRotationSystemUnproven):
    """Touching outer/hole or hole/hole topology has no accepted v1 policy."""


class ExactArrangementNonManifold(ExactArrangementRotationSystemUnproven):
    """Deprecated compatibility spelling for the former one-outgoing failure."""


@dataclass(frozen=True, slots=True)
class _InputBoundary:
    segment: BoundedSupportSegment
    region: PlanarRegion
    loop_id: str
    is_hole: bool
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
    # Длина канонической записи самой длинной координаты. Мерится строкой, а не
    # разрядностью: строка уже построена, а `int(p).bit_length()` потребовал бы
    # разбора выражения на каждую точку — наблюдение не должно стоить дороже
    # наблюдаемого.
    max_coordinate_chars: int


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


@dataclass(slots=True)
class _RotationTelemetry:
    comparisons: int = 0


@dataclass(frozen=True, slots=True)
class _BoundaryPairing:
    arrangement_point_id: str
    incoming_edge_id: str
    outgoing_edge_id: str
    covered_sector_id: str


@dataclass(frozen=True, slots=True)
class _BoundaryTopology:
    successors: dict[str, str]
    pairings: tuple[_BoundaryPairing, ...]
    branch_point_count: int
    max_incident_degree: int
    rotation_comparison_count: int


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
        generator_segment_ids=frozenset(
            {left.segment_id, right.segment_id}
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
        t = exact_normalize(cross(q_minus_p, s) / denominator)
        u = exact_normalize(cross(q_minus_p, r) / denominator)
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
    return exact_normalize(dot(point_sub(point, segment.start), direction) / dot(direction, direction))


# Сегменты, предложенные локализации точки. Полевые счётчики показали, что ни
# одна считаемая величина не объясняет время: входы выросли в 5-7 раз, пары в
# 15-29, а RAW_UNION в 656. Локализация точки — линейный проход по всей петле на
# каждый запрос, и её работа до сих пор не считалась ничем.
#
# Считается модульным счётчиком, а не возвратом из предиката: `_point_in_loop`
# зовут из шести мест, и протаскивать аккумулятор через каждое значило бы менять
# сигнатуры ради наблюдения. Инкремент один на вызов, а не на сегмент: на горячем
# цикле это разница между наблюдением и его стоимостью. Величина — верхняя
# оценка: выход по `_point_on_segment` считается целиком, и это ровно то `O(грани
# x сегменты)`, которое надо подтвердить или опровергнуть.
POINT_LOCATION_SEGMENT_SCANS = {"count": 0}


def _point_in_loop(point: ExactPlanarPoint, loop: PlanarLoop) -> int:
    """Return 1 inside, 0 on boundary, -1 outside using exact winding."""

    POINT_LOCATION_SEGMENT_SCANS["count"] += len(loop.segments)
    winding = 0
    _, py = point.expressions()
    for segment in loop.segments:
        if _point_on_segment(point, segment):
            return 0
        _, sy = segment.start.expressions()
        _, ey = segment.end.expressions()
        upward = _compare(sy, py) <= 0 and _compare(ey, py) > 0
        downward = _compare(ey, py) <= 0 and _compare(sy, py) > 0
        # Ориентация нужна только сегменту, который пересекает горизонтальный
        # луч: ниже она читается исключительно под `upward`/`downward`. Замерено
        # на реальных петлях — луч пересекают 19.8% сегментов, так что раньше
        # четыре из пяти точных cross-предикатов вычислялись и выбрасывались.
        # Побочный эффект намеренный: неразрешимый знак у сегмента, не влияющего
        # на winding, больше не поднимает CertifiedPredicateUndecidable — мы не
        # требуем доказательства там, где ответ не используется.
        if not (upward or downward):
            continue
        side = exact_sign(
            cross(point_sub(segment.end, segment.start), point_sub(point, segment.start))
        )
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
    POINT_LOCATION_SEGMENT_SCANS["count"] += sum(
        len(loop.segments) for loop in (region.outer, *region.holes)
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
        *sorted(certificate.generator_segment_ids),
    )
    return "|".join(parts)


def _vertex_id(certificates: frozenset[ConstructionCertificate]) -> str:
    if not certificates:
        raise CertifiedPredicateUndecidable("arrangement vertex has no construction history")
    return stable_id("raw-vertex", *sorted(_certificate_key(item) for item in certificates))


def _vector_sign_on_collinear_axis(
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> int:
    """Return same/opposite ray sign without assuming an orthonormal chart."""

    left_x, left_y = left.expressions()
    right_x, right_y = right.expressions()
    if exact_sign(left_x) != 0:
        if exact_sign(right_x) == 0:
            raise ExactArrangementRotationSystemUnproven(
                "collinear ray lost its non-zero affine coordinate"
            )
        return exact_sign(left_x * right_x)
    if exact_sign(left_y) == 0 or exact_sign(right_y) == 0:
        raise ExactArrangementRotationSystemUnproven(
            "zero-length boundary ray in rotation system"
        )
    return exact_sign(left_y * right_y)


def _same_ray(left: ExactPlanarVector, right: ExactPlanarVector) -> bool:
    return (
        exact_sign(cross(left, right)) == 0
        and _vector_sign_on_collinear_axis(left, right) > 0
    )


def _ray_half(direction: ExactPlanarVector) -> int:
    x, y = direction.expressions()
    y_sign = exact_sign(y)
    if y_sign > 0:
        return 0
    if y_sign < 0:
        return 1
    x_sign = exact_sign(x)
    if x_sign > 0:
        return 0
    if x_sign < 0:
        return 1
    raise ExactArrangementRotationSystemUnproven(
        "zero-length boundary ray in rotation system"
    )


def _compare_rays_ccw(
    left: ExactPlanarVector,
    right: ExactPlanarVector,
    telemetry: _RotationTelemetry,
) -> int:
    """Exact cyclic order in the oriented affine chart."""

    telemetry.comparisons += 1
    left_half = _ray_half(left)
    right_half = _ray_half(right)
    if left_half != right_half:
        return -1 if left_half < right_half else 1
    orientation = exact_sign(cross(left, right))
    if orientation > 0:
        return -1
    if orientation < 0:
        return 1
    if _vector_sign_on_collinear_axis(left, right) > 0:
        return 0
    # Opposite rays are separated by _ray_half; reaching this branch means
    # the affine direction classifier is internally inconsistent.
    raise ExactArrangementRotationSystemUnproven(
        "opposite rays received the same exact polar half"
    )


def _boundary_rotation_system(
    raw_edges: tuple[RawCoverageEdgeV1, ...],
    output_by_edge_id: dict[str, _OrientedOutput],
) -> _BoundaryTopology:
    """Pair every incoming edge with its exact covered-left face successor."""

    incoming: dict[str, list[RawCoverageEdgeV1]] = {}
    outgoing: dict[str, list[RawCoverageEdgeV1]] = {}
    for edge in raw_edges:
        outgoing.setdefault(edge.start_vertex_id, []).append(edge)
        incoming.setdefault(edge.end_vertex_id, []).append(edge)
    point_ids = sorted(set(incoming) | set(outgoing))
    telemetry = _RotationTelemetry()
    pairings: list[_BoundaryPairing] = []
    successors: dict[str, str] = {}
    branch_point_count = 0
    max_incident_degree = 0

    for point_id in point_ids:
        incoming_edges = tuple(
            sorted(incoming.get(point_id, ()), key=lambda item: item.edge_id)
        )
        outgoing_edges = tuple(
            sorted(outgoing.get(point_id, ()), key=lambda item: item.edge_id)
        )
        incident_degree = len(incoming_edges) + len(outgoing_edges)
        max_incident_degree = max(max_incident_degree, incident_degree)
        if len(incoming_edges) != len(outgoing_edges) or not incoming_edges:
            raise ExactArrangementRotationSystemUnproven(
                "boundary point does not have balanced incoming/outgoing half-edges"
            )
        if len(incoming_edges) > 1:
            branch_point_count += 1

        outgoing_with_rays = [
            (
                edge,
                point_sub(
                    output_by_edge_id[edge.edge_id].end,
                    output_by_edge_id[edge.edge_id].start,
                ),
            )
            for edge in outgoing_edges
        ]
        outgoing_with_rays.sort(
            key=cmp_to_key(
                lambda left, right: _compare_rays_ccw(
                    left[1], right[1], telemetry
                )
            )
        )
        for left, right in zip(
            outgoing_with_rays,
            outgoing_with_rays[1:],
        ):
            if _same_ray(left[1], right[1]):
                raise ExactArrangementCollinearBranchUnproven(
                    "multiple outgoing atomic edges occupy one exact ray"
                )

        chosen_outgoing_ids: set[str] = set()
        for incoming_edge in incoming_edges:
            incoming_output = output_by_edge_id[incoming_edge.edge_id]
            reverse_incoming = point_sub(
                incoming_output.start,
                incoming_output.end,
            )
            lower = 0
            upper = len(outgoing_with_rays)
            while lower < upper:
                middle = (lower + upper) // 2
                comparison = _compare_rays_ccw(
                    outgoing_with_rays[middle][1],
                    reverse_incoming,
                    telemetry,
                )
                if comparison < 0:
                    lower = middle + 1
                else:
                    upper = middle
            if lower < len(outgoing_with_rays) and _compare_rays_ccw(
                outgoing_with_rays[lower][1],
                reverse_incoming,
                telemetry,
            ) == 0:
                raise ExactArrangementCollinearBranchUnproven(
                    "boundary continuation would reverse on the same exact ray"
                )
            outgoing_edge = outgoing_with_rays[
                lower - 1 if lower else -1
            ][0]
            if outgoing_edge.edge_id in chosen_outgoing_ids:
                raise ExactArrangementRotationSystemUnproven(
                    "exact face walk does not prove a bijective boundary pairing"
                )
            chosen_outgoing_ids.add(outgoing_edge.edge_id)
            successors[incoming_edge.edge_id] = outgoing_edge.edge_id
            pairings.append(
                _BoundaryPairing(
                    arrangement_point_id=point_id,
                    incoming_edge_id=incoming_edge.edge_id,
                    outgoing_edge_id=outgoing_edge.edge_id,
                    covered_sector_id=stable_id(
                        "raw-covered-sector",
                        point_id,
                        incoming_edge.edge_id,
                        outgoing_edge.edge_id,
                    ),
                )
            )
        if chosen_outgoing_ids != {item.edge_id for item in outgoing_edges}:
            raise ExactArrangementRotationSystemUnproven(
                "not every outgoing boundary half-edge has one predecessor"
            )

    return _BoundaryTopology(
        successors=successors,
        pairings=tuple(
            sorted(pairings, key=lambda item: item.incoming_edge_id)
        ),
        branch_point_count=branch_point_count,
        max_incident_degree=max_incident_degree,
        rotation_comparison_count=telemetry.comparisons,
    )


def _planar_loop_from_raw(
    loop: RawCoverageLoopV1,
    edge_by_id: dict[str, RawCoverageEdgeV1],
    output_by_edge_id: dict[str, _OrientedOutput],
) -> PlanarLoop:
    return PlanarLoop(
        loop.loop_id,
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
            for edge_id in loop.ordered_edge_ids
            for edge in (edge_by_id[edge_id],)
        ),
    )


def _edge_midpoint(
    edge: RawCoverageEdgeV1,
    output_by_edge_id: dict[str, _OrientedOutput],
) -> ExactPlanarPoint:
    output = output_by_edge_id[edge.edge_id]
    return ExactPlanarPoint.from_values(
        (output.start.x.as_expr() + output.end.x.as_expr()) / 2,
        (output.start.y.as_expr() + output.end.y.as_expr()) / 2,
    )


def _certified_region_interior_witness(region: PlanarRegion) -> ExactPlanarPoint:
    """Construct a strict interior witness using exact predicates only."""

    for segment in sorted(region.outer.segments, key=lambda item: item.segment_id):
        midpoint = ExactPlanarPoint.from_values(
            (segment.start.x.as_expr() + segment.end.x.as_expr()) / 2,
            (segment.start.y.as_expr() + segment.end.y.as_expr()) / 2,
        )
        direction = point_sub(segment.end, segment.start)
        dx, dy = direction.expressions()
        left_direction = ExactPlanarVector.from_values(-dy, dx)
        denominator = sp.Integer(1)
        while True:
            left_candidate = point_add(
                midpoint,
                vector_scale(left_direction, sp.Integer(1) / denominator),
            )
            right_candidate = point_add(
                midpoint,
                vector_scale(left_direction, -sp.Integer(1) / denominator),
            )
            left_inside = _point_in_region(left_candidate, region) > 0
            right_inside = _point_in_region(right_candidate, region) > 0
            if left_inside != right_inside:
                return left_candidate if left_inside else right_candidate
            denominator *= 2
    raise ExactArrangementRotationSystemUnproven(
        "positive-area region has no certified interior witness"
    )


class ExactSegmentArrangementBackend:
    backend_identity = "CFTUV_EXACT_SEGMENT_ARRANGEMENT_WITH_HISTORY"
    backend_version = "3.0.0"

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
                for loop, is_hole in (
                    (region.outer, False),
                    *((item, True) for item in region.holes),
                ):
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
                                loop_id=loop.loop_id,
                                is_hole=is_hole,
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
                if (
                    left.region.region_id == right.region.region_id
                    and left.loop_id != right.loop_id
                    and (left.is_hole or right.is_hole)
                ):
                    raise ExactTouchingHoleTopologyUnproven(
                        "input outer/hole or hole/hole boundaries touch exactly"
                    )
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
        max_coordinate_chars = max(
            (
                len(scalar.expression)
                for history in atomic.values()
                for point in (history.start, history.end)
                for scalar in (point.x, point.y)
            ),
            default=0,
        )
        counters = ArrangementBuildCountersV1(
            input_segments=input_segment_count,
            all_possible_pairs=(
                input_segment_count * (input_segment_count - 1) // 2
            ),
            broadphase_candidate_pairs=len(candidate_pairs),
            narrowphase_tests=len(candidate_pairs),
            actual_intersections=intersection_count,
            atomic_edges=len(atomic),
            max_coordinate_chars=max_coordinate_chars,
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
    ) -> ArrangementUnionV2:
        scans_before = POINT_LOCATION_SEGMENT_SCANS["count"]
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

        raw_edges = tuple(sorted(raw_edges, key=lambda item: item.edge_id))
        if len(raw_edges) != len({item.edge_id for item in raw_edges}):
            raise ExactArrangementRotationSystemUnproven(
                "distinct atomic boundaries received one semantic edge identity"
            )
        topology = _boundary_rotation_system(raw_edges, output_by_edge_id)
        edge_by_id = {item.edge_id: item for item in raw_edges}
        unvisited = {edge.edge_id for edge in raw_edges}
        loops = []
        loop_id_by_incoming_edge: dict[str, str] = {}
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
                successor_id = topology.successors.get(current.edge_id)
                if successor_id is None:
                    raise ExactArrangementRotationSystemUnproven(
                        "boundary half-edge has no exact face-successor"
                    )
                current = edge_by_id[successor_id]
            if current.edge_id != first.edge_id:
                raise ExactArrangementRotationSystemUnproven(
                    "boundary face-walk entered another closed cycle"
                )
            area = polygon_signed_area(points)
            sign = exact_sign(area)
            if sign == 0:
                raise ExactArrangementRotationSystemUnproven(
                    "boundary face-walk produced a zero-area loop"
                )
            loop_id = stable_id("raw-loop", *ordered)
            loop = RawCoverageLoopV1(
                loop_id=loop_id,
                kind=RawCoverageLoopKind.OUTER if sign > 0 else RawCoverageLoopKind.HOLE,
                ordered_edge_ids=tuple(ordered),
                signed_area=ExactScalar.from_value(area).expression,
            )
            loops.append(loop)
            for edge_id in ordered:
                loop_id_by_incoming_edge[edge_id] = loop_id

        pairing_by_incoming = {
            item.incoming_edge_id: item for item in topology.pairings
        }
        vertex_by_id = {item.vertex_id: item for item in vertices}
        occurrences = []
        for incoming_edge_id, loop_id in sorted(
            loop_id_by_incoming_edge.items()
        ):
            pairing = pairing_by_incoming[incoming_edge_id]
            incoming_edge = edge_by_id[pairing.incoming_edge_id]
            outgoing_edge = edge_by_id[pairing.outgoing_edge_id]
            point = vertex_by_id[pairing.arrangement_point_id]
            occurrences.append(
                BoundaryVertexOccurrenceV1(
                    boundary_occurrence_id=stable_id(
                        "raw-boundary-occurrence",
                        pairing.arrangement_point_id,
                        pairing.incoming_edge_id,
                        pairing.outgoing_edge_id,
                    ),
                    arrangement_point_id=pairing.arrangement_point_id,
                    loop_id=loop_id,
                    incoming_boundary_edge_id=pairing.incoming_edge_id,
                    outgoing_boundary_edge_id=pairing.outgoing_edge_id,
                    covered_sector_id=pairing.covered_sector_id,
                    construction_certificates=point.construction_certificates,
                    provenance=merge_provenance(
                        incoming_edge.provenance,
                        outgoing_edge.provenance,
                    ),
                )
            )

        occurrences_by_point: dict[
            str, list[BoundaryVertexOccurrenceV1]
        ] = {}
        for occurrence in occurrences:
            occurrences_by_point.setdefault(
                occurrence.arrangement_point_id, []
            ).append(occurrence)
        point_contacts = []
        for point_id, point_occurrences in sorted(occurrences_by_point.items()):
            if len(point_occurrences) < 2:
                continue
            contact_edges = {
                edge_id
                for occurrence in point_occurrences
                for edge_id in (
                    occurrence.incoming_boundary_edge_id,
                    occurrence.outgoing_boundary_edge_id,
                )
            }
            contact_provenance = merge_provenance(
                *(edge_by_id[item].provenance for item in sorted(contact_edges))
            )
            point_contacts.append(
                PointContactRecordV1(
                    point_contact_id=stable_id(
                        "raw-point-contact",
                        point_id,
                        *sorted(
                            item.boundary_occurrence_id
                            for item in point_occurrences
                        ),
                    ),
                    arrangement_point_id=point_id,
                    participating_loop_ids=tuple(
                        sorted({item.loop_id for item in point_occurrences})
                    ),
                    participating_envelope_instance_ids=(
                        contact_provenance.envelope_instance_ids
                    ),
                    participating_front_component_ids=frozenset().union(
                        *(
                            edge_by_id[item].reachability.front_component_ids
                            for item in contact_edges
                        )
                    ),
                    construction_certificates=vertex_by_id[
                        point_id
                    ].construction_certificates,
                    provenance=contact_provenance,
                )
            )

        outer_loops = [item for item in loops if item.kind is RawCoverageLoopKind.OUTER]
        hole_loops = [item for item in loops if item.kind is RawCoverageLoopKind.HOLE]
        vertex_ids_by_loop = {
            loop.loop_id: frozenset(
                vertex_id
                for edge_id in loop.ordered_edge_ids
                for edge in (edge_by_id[edge_id],)
                for vertex_id in (edge.start_vertex_id, edge.end_vertex_id)
            )
            for loop in loops
        }
        for left_index, left in enumerate(loops):
            for right in loops[left_index + 1 :]:
                if (
                    left.kind is RawCoverageLoopKind.OUTER
                    and right.kind is RawCoverageLoopKind.OUTER
                ):
                    continue
                if vertex_ids_by_loop[left.loop_id] & vertex_ids_by_loop[
                    right.loop_id
                ]:
                    raise ExactTouchingHoleTopologyUnproven(
                        "touching outer/hole or hole/hole boundary requires "
                        "an explicit topology policy"
                    )

        holes_by_outer = {item.loop_id: [] for item in outer_loops}
        planar_loop_by_id = {
            loop.loop_id: _planar_loop_from_raw(
                loop, edge_by_id, output_by_edge_id
            )
            for loop in loops
        }
        for hole in hole_loops:
            first_edge = edge_by_id[hole.ordered_edge_ids[0]]
            witness = _edge_midpoint(first_edge, output_by_edge_id)
            containing = [
                outer
                for outer in outer_loops
                if _point_in_loop(witness, planar_loop_by_id[outer.loop_id])
                > 0
            ]
            if not containing:
                if any(
                    _point_in_loop(
                        witness, planar_loop_by_id[outer.loop_id]
                    )
                    == 0
                    for outer in outer_loops
                ):
                    raise ExactTouchingHoleTopologyUnproven(
                        "hole witness lies on an outer boundary"
                    )
                raise ExactArrangementRotationSystemUnproven(
                    "hole loop has no containing outer face"
                )
            containing.sort(
                key=cmp_to_key(
                    lambda left, right: _compare(
                        abs(sp.sympify(left.signed_area)),
                        abs(sp.sympify(right.signed_area)),
                    )
                )
            )
            if len(containing) > 1 and _compare(
                abs(sp.sympify(containing[0].signed_area)),
                abs(sp.sympify(containing[1].signed_area)),
            ) == 0:
                raise ExactArrangementRotationSystemUnproven(
                    "hole containment has no unique minimum-area outer face"
                )
            holes_by_outer[containing[0].loop_id].append(hole)

        regions = []
        total_area = sp.Integer(0)
        contribution_witnesses = tuple(
            (
                contribution,
                _certified_region_interior_witness(contribution),
            )
            for contribution in sorted(
                contribution_regions, key=lambda item: item.region_id
            )
        )
        for outer in outer_loops:
            holes = holes_by_outer[outer.loop_id]
            edge_ids = (*outer.ordered_edge_ids, *(edge for hole in holes for edge in hole.ordered_edge_ids))
            provenance = merge_provenance(*(edge_by_id[item].provenance for item in edge_ids))
            spec_ids = set(provenance.envelope_spec_ids)
            instance_ids = set(provenance.envelope_instance_ids)
            output_region = PlanarRegion(
                region_id=stable_id("raw-planar-region", outer.loop_id),
                outer=planar_loop_by_id[outer.loop_id],
                holes=tuple(
                    planar_loop_by_id[item.loop_id] for item in holes
                ),
            )
            region_contributor_provenance = []
            for contribution, witness in contribution_witnesses:
                if _point_in_region(witness, output_region) > 0:
                    spec_ids.update(contribution.contributor_spec_ids)
                    instance_ids.update(contribution.contributor_instance_ids)
                    region_contributor_provenance.extend(
                        segment.provenance.coverage_contributors
                        for loop in (contribution.outer, *contribution.holes)
                        for segment in loop.segments
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

        return ArrangementUnionV2(
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
            boundary_vertex_occurrences=frozenset(occurrences),
            point_contacts=frozenset(point_contacts),
            branch_point_count=topology.branch_point_count,
            max_incident_degree=topology.max_incident_degree,
            rotation_comparison_count=topology.rotation_comparison_count,
            face_walk_count=len(loops),
            point_location_segment_scan_count=(
                POINT_LOCATION_SEGMENT_SCANS["count"] - scans_before
            ),
            max_coordinate_chars=build.counters.max_coordinate_chars,
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
