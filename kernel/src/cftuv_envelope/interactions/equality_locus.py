"""Exact equality-line construction and active-domain clipping."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cmp_to_key

import sympy as sp

from ..reference.arrangement import (
    _point_in_region,
    _point_on_segment,
    segment_intersections,
)
from ..reference.common import stable_id
from ..reference.planar_types import (
    BoundedSupportSegment,
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    PlanarRegion,
    exact_sign,
    point_key,
    points_equal,
)
from ..reference.provenance import ReferenceProvenanceV1
from .contracts import (
    EqualityLocusOwner,
    EqualityLocusSegmentV1,
    EqualityLocusV1,
    ExactFrontArrivalLawV1,
)


@dataclass(frozen=True, slots=True)
class ExactEqualityLineV1:
    normal_x: ExactScalar
    normal_y: ExactScalar
    constant: ExactScalar

    def expressions(self) -> tuple[sp.Expr, sp.Expr, sp.Expr]:
        return (
            self.normal_x.as_expr(),
            self.normal_y.as_expr(),
            self.constant.as_expr(),
        )


@dataclass(frozen=True, slots=True)
class ClippedEqualityLocusV1:
    line: ExactEqualityLineV1
    segments: tuple[tuple[ExactPlanarPoint, ExactPlanarPoint], ...]
    anchors: tuple[ExactPlanarPoint, ...]
    exact_first_alpha: ExactScalar


def equality_line(
    left: ExactFrontArrivalLawV1,
    right: ExactFrontArrivalLawV1,
) -> ExactEqualityLineV1 | None:
    left_speed = left.normal_speed.as_expr()
    right_speed = right.normal_speed.as_expr()
    if exact_sign(left_speed) <= 0 or exact_sign(right_speed) <= 0:
        raise ValueError("arrival normal speed must be strictly positive")
    left_x, left_y = left.normal.expressions()
    right_x, right_y = right.normal.expressions()
    normal_x = sp.factor(left_x / left_speed - right_x / right_speed)
    normal_y = sp.factor(left_y / left_speed - right_y / right_speed)
    constant = sp.factor(
        left.source_constant.as_expr() / left_speed
        - right.source_constant.as_expr() / right_speed
    )
    if exact_sign(normal_x) == 0 and exact_sign(normal_y) == 0:
        if exact_sign(constant) == 0:
            return ExactEqualityLineV1(
                ExactScalar.from_value(0),
                ExactScalar.from_value(0),
                ExactScalar.from_value(0),
            )
        return None
    return ExactEqualityLineV1(
        ExactScalar.from_value(normal_x),
        ExactScalar.from_value(normal_y),
        ExactScalar.from_value(constant),
    )


def _all_points(
    left_regions: tuple[PlanarRegion, ...],
    right_regions: tuple[PlanarRegion, ...],
) -> tuple[ExactPlanarPoint, ...]:
    return tuple(
        point
        for region in (*left_regions, *right_regions)
        for loop in (region.outer, *region.holes)
        for point in loop.points
    )


def _extreme(values: tuple[sp.Expr, ...], *, maximum: bool) -> sp.Expr:
    chosen = values[0]
    for value in values[1:]:
        comparison = exact_sign(value - chosen)
        if (maximum and comparison > 0) or (not maximum and comparison < 0):
            chosen = value
    return chosen


def _line_bbox_segment(
    line: ExactEqualityLineV1,
    points: tuple[ExactPlanarPoint, ...],
) -> BoundedSupportSegment | None:
    if not points:
        return None
    xs = tuple(point.x.as_expr() for point in points)
    ys = tuple(point.y.as_expr() for point in points)
    min_x = _extreme(xs, maximum=False)
    max_x = _extreme(xs, maximum=True)
    min_y = _extreme(ys, maximum=False)
    max_y = _extreme(ys, maximum=True)
    span = (max_x - min_x) + (max_y - min_y) + 1
    min_x -= span
    max_x += span
    min_y -= span
    max_y += span
    a, b, d = line.expressions()
    candidates = []

    def add(x, y) -> None:
        if (
            exact_sign(x - min_x) >= 0
            and exact_sign(max_x - x) >= 0
            and exact_sign(y - min_y) >= 0
            and exact_sign(max_y - y) >= 0
        ):
            point = ExactPlanarPoint.from_values(x, y)
            if not any(points_equal(point, item) for item in candidates):
                candidates.append(point)

    if exact_sign(b) != 0:
        add(min_x, (d - a * min_x) / b)
        add(max_x, (d - a * max_x) / b)
    if exact_sign(a) != 0:
        add((d - b * min_y) / a, min_y)
        add((d - b * max_y) / a, max_y)
    if len(candidates) < 2:
        return None
    candidates.sort(key=lambda point: point_key(point))
    certificate = ConstructionCertificate(
        ConstructionKind.EVENT_ANCHOR,
        event_key=stable_id(
            "equality-line",
            line.normal_x.expression,
            line.normal_y.expression,
            line.constant.expression,
        ),
    )
    return BoundedSupportSegment(
        segment_id=certificate.event_key or "equality-line",
        start=candidates[0],
        end=candidates[-1],
        support_ids=frozenset(),
        boundary_constraint_ids=frozenset(),
        provenance=ReferenceProvenanceV1(),
        start_constructions=frozenset({certificate}),
        end_constructions=frozenset({certificate}),
    )


def _parameter(
    segment: BoundedSupportSegment, point: ExactPlanarPoint
) -> sp.Expr:
    sx, sy = segment.start.expressions()
    ex, ey = segment.end.expressions()
    px, py = point.expressions()
    dx = ex - sx
    dy = ey - sy
    return sp.factor(((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy))


def _point_in_any_closure(
    point: ExactPlanarPoint, regions: tuple[PlanarRegion, ...]
) -> bool:
    return any(_point_in_region(point, region) >= 0 for region in regions)


def active_domains_share_closure(
    left_regions: tuple[PlanarRegion, ...],
    right_regions: tuple[PlanarRegion, ...],
) -> bool:
    for left in left_regions:
        for right in right_regions:
            if any(
                _point_in_region(point, right) >= 0
                for point in left.outer.points
            ) or any(
                _point_in_region(point, left) >= 0
                for point in right.outer.points
            ):
                return True
            if any(
                segment_intersections(left_segment, right_segment)
                for left_loop in (left.outer, *left.holes)
                for right_loop in (right.outer, *right.holes)
                for left_segment in left_loop.segments
                for right_segment in right_loop.segments
            ):
                return True
    return False


def _arrival_value(
    law: ExactFrontArrivalLawV1, point: ExactPlanarPoint
) -> sp.Expr:
    nx, ny = law.normal.expressions()
    px, py = point.expressions()
    return sp.factor(
        (nx * px + ny * py - law.source_constant.as_expr())
        / law.normal_speed.as_expr()
    )


def clip_equality_locus_to_active_domains(
    left_law: ExactFrontArrivalLawV1,
    right_law: ExactFrontArrivalLawV1,
    left_regions: tuple[PlanarRegion, ...],
    right_regions: tuple[PlanarRegion, ...],
    left_dominance_laws: tuple[ExactFrontArrivalLawV1, ...] = (),
    right_dominance_laws: tuple[ExactFrontArrivalLawV1, ...] = (),
) -> ClippedEqualityLocusV1 | None:
    line = equality_line(left_law, right_law)
    if line is None:
        return None
    a, b, _ = line.expressions()
    if exact_sign(a) == 0 and exact_sign(b) == 0:
        raise ValueError("coincident arrival laws have no unique equality locus")
    carrier = _line_bbox_segment(line, _all_points(left_regions, right_regions))
    if carrier is None:
        return None
    points = [carrier.start, carrier.end]
    for region in (*left_regions, *right_regions):
        for loop in (region.outer, *region.holes):
            for boundary in loop.segments:
                points.extend(segment_intersections(carrier, boundary))
    all_domain_points = _all_points(left_regions, right_regions)
    for current_law, sibling_laws in (
        (left_law, left_dominance_laws),
        (right_law, right_dominance_laws),
    ):
        for sibling in sibling_laws:
            if sibling.law_id == current_law.law_id:
                continue
            dominance_line = equality_line(current_law, sibling)
            if dominance_line is None:
                continue
            da, db, _ = dominance_line.expressions()
            if exact_sign(da) == 0 and exact_sign(db) == 0:
                continue
            dominance_carrier = _line_bbox_segment(
                dominance_line, all_domain_points
            )
            if dominance_carrier is not None:
                points.extend(
                    segment_intersections(carrier, dominance_carrier)
                )
    unique = []
    for point in points:
        if not any(points_equal(point, item) for item in unique):
            unique.append(point)
    unique.sort(
        key=cmp_to_key(
            lambda left, right: exact_sign(
                _parameter(carrier, left) - _parameter(carrier, right)
            )
        )
    )
    segments = []

    def dominates(
        current_law: ExactFrontArrivalLawV1,
        siblings: tuple[ExactFrontArrivalLawV1, ...],
        point: ExactPlanarPoint,
    ) -> bool:
        current = _arrival_value(current_law, point)
        return all(
            exact_sign(current - _arrival_value(sibling, point)) >= 0
            for sibling in siblings
        )

    for start, end in zip(unique, unique[1:]):
        midpoint = ExactPlanarPoint.from_values(
            (start.x.as_expr() + end.x.as_expr()) / 2,
            (start.y.as_expr() + end.y.as_expr()) / 2,
        )
        if _point_in_any_closure(
            midpoint, left_regions
        ) and _point_in_any_closure(
            midpoint, right_regions
        ) and dominates(
            left_law, left_dominance_laws, midpoint
        ) and dominates(
            right_law, right_dominance_laws, midpoint
        ):
            segments.append((start, end))
    anchors = tuple(
        point
        for point in unique
        if _point_in_any_closure(point, left_regions)
        and _point_in_any_closure(point, right_regions)
        and dominates(left_law, left_dominance_laws, point)
        and dominates(right_law, right_dominance_laws, point)
        and not any(
            points_equal(point, start) or points_equal(point, end)
            for start, end in segments
        )
    )
    if not segments and not anchors:
        return None
    arrival_values = [
        _arrival_value(left_law, point)
        for segment in segments
        for point in segment
    ]
    arrival_values.extend(_arrival_value(left_law, point) for point in anchors)
    first_alpha = _extreme(tuple(arrival_values), maximum=False)
    return ClippedEqualityLocusV1(
        line=line,
        segments=tuple(segments),
        anchors=anchors,
        exact_first_alpha=ExactScalar.from_value(first_alpha),
    )


def public_equality_locus(
    clipped: ClippedEqualityLocusV1,
    left_reading_id: str,
    right_reading_id: str,
    certificate_id: str,
) -> EqualityLocusV1:
    locus_id = stable_id(
        "equality-locus",
        certificate_id,
        clipped.line.normal_x.expression,
        clipped.line.normal_y.expression,
        clipped.line.constant.expression,
    )
    construction_id = stable_id(
        "equality-locus-construction",
        certificate_id,
        locus_id,
    )
    segments = tuple(
        EqualityLocusSegmentV1(
            segment_id=stable_id(
                "equality-locus-segment",
                locus_id,
                ordinal,
                point_key(start),
                point_key(end),
            ),
            start=start,
            end=end,
            left_front_reading_id=left_reading_id,
            right_front_reading_id=right_reading_id,
            construction_certificate_id=construction_id,
        )
        for ordinal, (start, end) in enumerate(clipped.segments)
    )
    return EqualityLocusV1(
        locus_id=locus_id,
        ordered_exact_segments=segments,
        event_anchor_points=clipped.anchors,
        participant_front_reading_ids=(
            left_reading_id,
            right_reading_id,
        ),
        construction_certificate_ids=(construction_id,),
        owner=EqualityLocusOwner.NONE,
    )


def restrict_clipped_locus_to_active_segments(
    clipped: ClippedEqualityLocusV1,
    law: ExactFrontArrivalLawV1,
    active_segments: tuple[BoundedSupportSegment, ...],
) -> ClippedEqualityLocusV1 | None:
    """Restrict Cap closure authority to its physical swept terminal segment."""

    retained_segments = []
    anchors = list(clipped.anchors)
    for start, end in clipped.segments:
        locus_segment = BoundedSupportSegment(
            segment_id="temporary-equality-locus",
            start=start,
            end=end,
            support_ids=frozenset(),
            boundary_constraint_ids=frozenset(),
            provenance=ReferenceProvenanceV1(),
            start_constructions=frozenset(),
            end_constructions=frozenset(),
        )
        for active in active_segments:
            intersections = segment_intersections(locus_segment, active)
            if len(intersections) >= 2:
                ordered = sorted(
                    intersections,
                    key=cmp_to_key(
                        lambda left, right: exact_sign(
                            _parameter(locus_segment, left)
                            - _parameter(locus_segment, right)
                        )
                    ),
                )
                retained_segments.append((ordered[0], ordered[-1]))
            else:
                anchors.extend(intersections)
    anchors = [
        point
        for point in anchors
        if any(_point_on_segment(point, segment) for segment in active_segments)
    ]
    unique_anchors = []
    for point in anchors:
        if not any(points_equal(point, item) for item in unique_anchors):
            unique_anchors.append(point)
    if not retained_segments and not unique_anchors:
        return None
    values = [
        _arrival_value(law, point)
        for segment in retained_segments
        for point in segment
    ]
    values.extend(_arrival_value(law, point) for point in unique_anchors)
    return ClippedEqualityLocusV1(
        line=clipped.line,
        segments=tuple(retained_segments),
        anchors=tuple(unique_anchors),
        exact_first_alpha=ExactScalar.from_value(
            _extreme(tuple(values), maximum=False)
        ),
    )
