"""INTRAPATCH_POLICY_B_V1 exact clipping and proof gates."""

from __future__ import annotations

from dataclasses import dataclass

import sympy as sp

from ..ids import (
    InteractionApplicationId,
    ResolvedContributionId,
)
from ..reference.arrangement import (
    ExactSegmentArrangementBackend,
    _point_on_segment,
    segment_intersections,
)
from ..reference.arrangement_protocol import ArrangementUnionV1
from ..reference.common import make_segment, stable_id
from ..reference.contracts import (
    BoundaryResolvedEnvelopeV1,
    RawCoverageResultV1,
    ReachabilityCertificateV1,
)
from ..reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
    cross,
    dot,
    exact_normalize,
    exact_sign,
    point_key,
    point_sub,
    points_equal,
    polygon_signed_area,
)
from ..reference.provenance import (
    ReferenceProvenanceV1,
    merge_coverage_contributor_provenance,
    merge_provenance,
)
from ..reference.raw_coverage import _arrangement_regions
from .contracts import (
    AngularProfileArrivalModelV1,
    FreezeState,
    INTRAPATCH_POLICY_B_V1,
    SELF_CONTACT_POLICY_B_V1,
    InteractionApplicationV1,
    InteractionCandidateKind,
    InteractionCoverageEffect,
    InteractionDiagnosticSeverity,
    InteractionDiagnosticV1,
    InteractionOutcome,
    ResolvedContributionV1,
)
from .mutual_arrival import ProvenInteractionV1
from .equality_locus import ExactEqualityLineV1


_ARRANGEMENT = ExactSegmentArrangementBackend()


@dataclass(frozen=True, slots=True)
class PolicyBResultV1:
    resolved_contributions: tuple[ResolvedContributionV1, ...]
    resolved_union: ArrangementUnionV1
    applications: tuple[InteractionApplicationV1, ...]
    diagnostics: tuple[InteractionDiagnosticV1, ...]


def _extreme(values: tuple[sp.Expr, ...], *, maximum: bool) -> sp.Expr:
    chosen = values[0]
    for value in values[1:]:
        sign = exact_sign(value - chosen)
        if (maximum and sign > 0) or (not maximum and sign < 0):
            chosen = value
    return chosen


def _enclosing_rectangle(
    resolved: tuple[BoundaryResolvedEnvelopeV1, ...],
) -> tuple[ExactPlanarPoint, ...]:
    points = tuple(
        point
        for item in resolved
        for region in item.envelope_instance.regions
        for loop in (region.outer, *region.holes)
        for point in loop.points
    )
    if not points:
        return ()
    xs = tuple(point.x.as_expr() for point in points)
    ys = tuple(point.y.as_expr() for point in points)
    min_x = _extreme(xs, maximum=False)
    max_x = _extreme(xs, maximum=True)
    min_y = _extreme(ys, maximum=False)
    max_y = _extreme(ys, maximum=True)
    margin = (max_x - min_x) + (max_y - min_y) + 1
    return (
        ExactPlanarPoint.from_values(min_x - margin, min_y - margin),
        ExactPlanarPoint.from_values(max_x + margin, min_y - margin),
        ExactPlanarPoint.from_values(max_x + margin, max_y + margin),
        ExactPlanarPoint.from_values(min_x - margin, max_y + margin),
    )


def _halfplane_polygon(
    rectangle: tuple[ExactPlanarPoint, ...],
    line,
    *,
    keep_nonpositive: bool,
    proof_id: str,
) -> PlanarRegion | None:
    a, b, d = line.expressions()

    def value(point: ExactPlanarPoint) -> sp.Expr:
        x, y = point.expressions()
        return exact_normalize(a * x + b * y - d)

    def inside(point: ExactPlanarPoint) -> bool:
        sign = exact_sign(value(point))
        return sign <= 0 if keep_nonpositive else sign >= 0

    output = list(rectangle)
    clipped = []
    for start, end in zip(output, output[1:] + output[:1]):
        start_inside = inside(start)
        end_inside = inside(end)
        start_value = value(start)
        end_value = value(end)
        if start_inside and end_inside:
            clipped.append(end)
        elif start_inside != end_inside:
            denominator = exact_normalize(start_value - end_value)
            if exact_sign(denominator) == 0:
                raise ValueError("halfplane edge crossing has zero denominator")
            parameter = exact_normalize(start_value / denominator)
            sx, sy = start.expressions()
            ex, ey = end.expressions()
            crossing = ExactPlanarPoint.from_values(
                sx + parameter * (ex - sx),
                sy + parameter * (ey - sy),
            )
            clipped.append(crossing)
            if end_inside:
                clipped.append(end)
    unique = []
    for point in clipped:
        if not unique or point_key(point) != point_key(unique[-1]):
            unique.append(point)
    if len(unique) > 1 and point_key(unique[0]) == point_key(unique[-1]):
        unique.pop()
    if len(unique) < 3:
        return None
    certificate = ConstructionCertificate(
        ConstructionKind.EVENT_ANCHOR,
        event_key=proof_id,
    )
    provenance = ReferenceProvenanceV1()
    segments = tuple(
        make_segment(
            stable_id(
                "interaction-halfplane-edge",
                proof_id,
                int(keep_nonpositive),
                ordinal,
            ),
            start,
            end,
            support_ids=frozenset(),
            provenance=provenance,
            start_certificates=frozenset({certificate}),
            end_certificates=frozenset({certificate}),
        )
        for ordinal, (start, end) in enumerate(
            zip(unique, unique[1:] + unique[:1])
        )
    )
    loop_id = stable_id(
        "interaction-halfplane-loop",
        proof_id,
        int(keep_nonpositive),
    )
    return PlanarRegion(
        region_id=stable_id("interaction-halfplane-region", loop_id),
        outer=PlanarLoop(loop_id, segments),
    )


def _clip_regions(
    regions: tuple[PlanarRegion, ...],
    halfplane: PlanarRegion | None,
    reachability: ReachabilityCertificateV1,
) -> tuple[PlanarRegion, ...]:
    if not regions or halfplane is None:
        return ()
    direct = []
    for region in regions:
        clipped = _clip_region_direct(region, halfplane)
        if clipped is None:
            break
        if clipped is not False:
            direct.append(clipped)
    else:
        return tuple(direct)
    instance_ids = frozenset(
        instance_id
        for region in regions
        for instance_id in region.contributor_instance_ids
    )
    union = _ARRANGEMENT.exact_union(
        regions,
        (halfplane,),
        {instance_id: reachability for instance_id in instance_ids},
    )
    return _arrangement_regions(union)


def _is_convex_region(region: PlanarRegion) -> bool:
    if region.holes:
        return False
    points = region.outer.points
    signs = set()
    for first, second, third in zip(
        points,
        points[1:] + points[:1],
        points[2:] + points[:2],
    ):
        sign = exact_sign(
            cross(point_sub(second, first), point_sub(third, second))
        )
        if sign:
            signs.add(sign)
    return len(signs) <= 1


def _clip_region_direct(
    region: PlanarRegion,
    halfplane: PlanarRegion,
) -> PlanarRegion | bool | None:
    """Clip one convex exact region; return None to request arrangement fallback."""

    if not _is_convex_region(region):
        return None
    clip_edge = next(
        (
            segment
            for segment in halfplane.outer.segments
            if segment.start_constructions
            and next(iter(segment.start_constructions)).event_key
        ),
        None,
    )
    if clip_edge is None:
        return None
    event_key = next(iter(clip_edge.start_constructions)).event_key or "clip"
    source_points = list(region.outer.points)
    # The halfplane polygon is already exact; membership is equivalent to
    # point-in-closure and avoids reconstructing its oriented line.
    from ..reference.arrangement import _point_in_region

    def inside(point: ExactPlanarPoint) -> bool:
        return _point_in_region(point, halfplane) >= 0

    output = []
    for start, end in zip(
        source_points[-1:] + source_points[:-1],
        source_points,
    ):
        start_inside = inside(start)
        end_inside = inside(end)
        if start_inside != end_inside:
            source_segment = next(
                segment
                for segment in region.outer.segments
                if _point_on_segment(start, segment)
                and _point_on_segment(end, segment)
            )
            intersections = tuple(
                point
                for boundary in halfplane.outer.segments
                for point in segment_intersections(source_segment, boundary)
            )
            if not intersections:
                return None
            crossing = intersections[0]
            output.append(crossing)
        if end_inside:
            output.append(end)
    unique = []
    for point in output:
        if not unique or not points_equal(point, unique[-1]):
            unique.append(point)
    if len(unique) > 1 and points_equal(unique[0], unique[-1]):
        unique.pop()
    if len(unique) < 3:
        return False
    area = polygon_signed_area(unique)
    if exact_sign(area) == 0:
        return False
    if exact_sign(area) < 0:
        unique.reverse()
    event_certificate = ConstructionCertificate(
        ConstructionKind.EVENT_ANCHOR,
        event_key=event_key,
    )

    def point_certificates(point: ExactPlanarPoint):
        certificates = set()
        for segment in region.outer.segments:
            if points_equal(point, segment.start):
                certificates.update(segment.start_constructions)
            if points_equal(point, segment.end):
                certificates.update(segment.end_constructions)
        if not certificates:
            certificates.add(event_certificate)
        return frozenset(certificates)

    coverage_provenance = ReferenceProvenanceV1(
        coverage_contributors=merge_coverage_contributor_provenance(
            *(
                segment.provenance.coverage_contributors
                for segment in region.outer.segments
            )
        )
    )
    segments = []
    for ordinal, (start, end) in enumerate(
        zip(unique, unique[1:] + unique[:1])
    ):
        midpoint = ExactPlanarPoint.from_values(
            (start.x.as_expr() + end.x.as_expr()) / 2,
            (start.y.as_expr() + end.y.as_expr()) / 2,
        )
        original = tuple(
            segment
            for segment in region.outer.segments
            if _point_on_segment(midpoint, segment)
        )
        provenance = (
            merge_provenance(*(item.provenance for item in original))
            if original
            else coverage_provenance
        )
        segments.append(
            make_segment(
                stable_id(
                    "interaction-direct-clip-edge",
                    event_key,
                    region.region_id,
                    ordinal,
                    point_key(start),
                    point_key(end),
                ),
                start,
                end,
                support_ids=frozenset().union(
                    *(item.support_ids for item in original)
                ),
                boundary_constraint_ids=frozenset().union(
                    *(item.boundary_constraint_ids for item in original)
                ),
                provenance=provenance,
                start_certificates=point_certificates(start),
                end_certificates=point_certificates(end),
            )
        )
    loop_id = stable_id(
        "interaction-direct-clip-loop",
        event_key,
        region.region_id,
        *(point_key(point) for point in unique),
    )
    return PlanarRegion(
        region_id=stable_id("interaction-direct-clip-region", loop_id),
        outer=PlanarLoop(loop_id, tuple(segments)),
        contributor_instance_ids=region.contributor_instance_ids,
        contributor_spec_ids=region.contributor_spec_ids,
    )


def _union_regions(
    regions: tuple[PlanarRegion, ...],
    domain_regions: tuple[PlanarRegion, ...],
    reachability: dict[str, ReachabilityCertificateV1],
) -> tuple[PlanarRegion, ...]:
    if not regions:
        return ()
    union = _ARRANGEMENT.exact_union(
        regions,
        domain_regions,
        reachability,
    )
    return _arrangement_regions(union)


def _edge_halfplane(
    rectangle: tuple[ExactPlanarPoint, ...],
    segment,
    *,
    keep_interior: bool,
    proof_id: str,
) -> PlanarRegion | None:
    sx, sy = segment.start.expressions()
    ex, ey = segment.end.expressions()
    dx = ex - sx
    dy = ey - sy
    normal_x = dy
    normal_y = -dx
    constant = exact_normalize(normal_x * sx + normal_y * sy)
    line = ExactEqualityLineV1(
        ExactScalar.from_value(normal_x),
        ExactScalar.from_value(normal_y),
        ExactScalar.from_value(constant),
    )
    # CCW polygon interior is left of its edge, hence non-positive for
    # the right-facing normal above.
    return _halfplane_polygon(
        rectangle,
        line,
        keep_nonpositive=keep_interior,
        proof_id=proof_id,
    )


def _intersect_with_convex_region(
    subject: tuple[PlanarRegion, ...],
    clip: PlanarRegion,
    rectangle: tuple[ExactPlanarPoint, ...],
    reachability: ReachabilityCertificateV1,
    proof_id: str,
) -> tuple[PlanarRegion, ...]:
    if not _is_convex_region(clip):
        raise ValueError("Policy B exact clip region is not convex")
    retained = subject
    for ordinal, segment in enumerate(clip.outer.segments):
        retained = _clip_regions(
            retained,
            _edge_halfplane(
                rectangle,
                segment,
                keep_interior=True,
                proof_id=stable_id(
                    "interaction-convex-intersection",
                    proof_id,
                    ordinal,
                ),
            ),
            reachability,
        )
        if not retained:
            break
    return retained


def _subtract_convex_regions(
    subject: tuple[PlanarRegion, ...],
    clips: tuple[PlanarRegion, ...],
    rectangle: tuple[ExactPlanarPoint, ...],
    reachability: ReachabilityCertificateV1,
    all_reachability: dict[str, ReachabilityCertificateV1],
    domain_regions: tuple[PlanarRegion, ...],
    proof_id: str,
) -> tuple[PlanarRegion, ...]:
    pieces = subject
    for clip_index, clip in enumerate(clips):
        if not _is_convex_region(clip):
            raise ValueError("Policy B subtraction clip is not convex")
        exterior_pieces = []
        for edge_index, segment in enumerate(clip.outer.segments):
            halfplane = _edge_halfplane(
                rectangle,
                segment,
                keep_interior=False,
                proof_id=stable_id(
                    "interaction-convex-difference",
                    proof_id,
                    clip_index,
                    edge_index,
                ),
            )
            exterior_pieces.extend(
                region
                for region in _clip_regions(
                    pieces,
                    halfplane,
                    reachability,
                )
            )
        pieces = tuple(exterior_pieces)
        if not pieces:
            break
    return pieces


def _loop_set_signature(vertices, edges, loops) -> tuple:
    vertex_by_id = {item.vertex_id: item.point for item in vertices}
    edge_by_id = {item.edge_id: item for item in edges}
    signatures = []
    for loop in loops:
        points = [
            vertex_by_id[edge_by_id[edge_id].start_vertex_id]
            for edge_id in loop.ordered_edge_ids
        ]
        changed = True
        while changed and len(points) > 3:
            changed = False
            for index, point in enumerate(points):
                previous = points[index - 1]
                following = points[(index + 1) % len(points)]
                incoming = point_sub(point, previous)
                outgoing = point_sub(following, point)
                if (
                    exact_sign(cross(incoming, outgoing)) == 0
                    and exact_sign(dot(incoming, outgoing)) > 0
                ):
                    points.pop(index)
                    changed = True
                    break
        keys = tuple(point_key(point) for point in points)
        rotations = tuple(keys[index:] + keys[:index] for index in range(len(keys)))
        signatures.append((loop.kind.value, min(rotations)))
    return tuple(sorted(signatures))


def _set_signature(union: ArrangementUnionV1) -> tuple:
    return _loop_set_signature(union.vertices, union.edges, union.loops)


def _raw_set_signature(raw: RawCoverageResultV1) -> tuple:
    return _loop_set_signature(raw.vertices, raw.edges, raw.loops)


def _component_intersection_area(
    left_regions: tuple[PlanarRegion, ...],
    right_regions: tuple[PlanarRegion, ...],
    reachability: dict[str, ReachabilityCertificateV1],
) -> sp.Expr:
    if not left_regions or not right_regions:
        return sp.Integer(0)
    intersection = _ARRANGEMENT.exact_union(
        left_regions,
        right_regions,
        reachability,
    )
    return ExactScalar(intersection.exact_area_expression).as_expr()


# Почему у эталона сохранения появился параметр, а сам закон не тронут.
#
# Проверка ниже — ЗАКОН СОХРАНЕНИЯ: разбиение обязано воспроизвести то же
# точечное множество, что дал производитель покрытия. Инвариант верен, и менять
# его логику не нужно. Неверна была только ЖЁСТКАЯ привязка эталона к
# `RawCoverage`: тот сам построен попарным путём, поэтому вопрос «а верно ли
# само множество» здесь не задавался никогда — сравнивалось покрытие с самим
# собой до кроя. Ровно поэтому дыра `alpha^2` у вогнутой вершины прожила
# незамеченной (`DECISIONS.md`, 2026-07-26).
#
# Логика уже была общей: `_set_signature` и `_raw_set_signature` обе сводятся к
# `_loop_set_signature`, и к `RawCoverage` был привязан только АРГУМЕНТ. Здесь
# он стал параметром со значением по умолчанию, поэтому поведение конвейера не
# изменилось ни на бит: `resolved_coverage.py` вызывает функцию без него.
# Живой потребитель параметра — независимый митрованный эталон
# (`kernel/tests/mitered_standard.py`), и он ловит дыру `alpha^2` там, где
# привязка к `RawCoverage` молчит.
def apply_policy_b(
    proofs: tuple[ProvenInteractionV1, ...],
    components,
    boundary_resolved_envelopes: tuple[BoundaryResolvedEnvelopeV1, ...],
    raw_coverage: RawCoverageResultV1,
    domain_regions: tuple[PlanarRegion, ...],
    conservation: RawCoverageResultV1 | None = None,
) -> PolicyBResultV1:
    """Clip existing contributions only, then prove union and disjointness.

    `conservation` — эталон закона сохранения; по умолчанию это `raw_coverage`,
    и тогда поведение прежнее. Требуется от него ровно то же, что берётся у
    `RawCoverage`: `vertices`, `edges`, `loops` и `exact_area_expression`.
    """

    component_by_id = {
        item.interaction_component_id: item for item in components
    }
    resolved_by_instance = {
        item.envelope_instance.envelope_instance_id: item
        for item in boundary_resolved_envelopes
    }
    component_by_instance = {
        instance_id.value: component
        for component in components
        for instance_id in component.envelope_instance_ids
    }
    reachability = {
        instance_id: item.reachability
        for instance_id, item in resolved_by_instance.items()
    }
    rectangle = _enclosing_rectangle(boundary_resolved_envelopes)
    current = {
        instance_id: item.envelope_instance.regions
        for instance_id, item in resolved_by_instance.items()
    }
    removed = {instance_id: [] for instance_id in current}
    interaction_ids = {instance_id: [] for instance_id in current}
    reading_ids = {instance_id: set() for instance_id in current}
    applications = []
    self_outputs = []
    diagnostics = []
    proof_groups = {}
    for proof in proofs:
        proof_groups.setdefault(proof.candidate.candidate_id, []).append(proof)

    for group in proof_groups.values():
        group = sorted(
            group,
            key=lambda proof: (
                proof.clipped_locus.line.normal_x.expression,
                proof.clipped_locus.line.normal_y.expression,
                proof.clipped_locus.line.constant.expression,
            ),
        )
        candidate = group[0].candidate
        prepared = []
        for proof in group:
            interaction_id = InteractionApplicationId(stable_id(
                "interaction-application",
                proof.candidate.candidate_id,
                proof.mutual_arrival_certificate.certificate_id,
            ))
            prepared.append(
                (
                    proof,
                    interaction_id,
                    _halfplane_polygon(
                        rectangle,
                        proof.clipped_locus.line,
                        keep_nonpositive=True,
                        proof_id=interaction_id.value,
                    ),
                    _halfplane_polygon(
                        rectangle,
                        proof.clipped_locus.line,
                        keep_nonpositive=False,
                        proof_id=interaction_id.value,
                    ),
                )
            )
        if (
            candidate.candidate_kind
            is InteractionCandidateKind.EXPLICIT_SELF_CONTACT
            and candidate.left_component_id
            == candidate.right_component_id
        ):
            if len(prepared) != 1:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        "self-contact v1 requires one exact equality law",
                        frozenset({candidate.left_component_id}),
                    )
                )
                continue
            proof, interaction_id, left_halfplane, right_halfplane = prepared[0]
            component = component_by_id[candidate.left_component_id]
            for typed_instance_id in component.envelope_instance_ids:
                instance_id = typed_instance_id.value
                if instance_id not in current:
                    continue
                source = current[instance_id]
                self_outputs.extend(
                    (
                        (
                            instance_id,
                            _clip_regions(
                                source,
                                left_halfplane,
                                reachability[instance_id],
                            ),
                            _clip_regions(
                                source,
                                right_halfplane,
                                reachability[instance_id],
                            ),
                            proof.left_model.front_reading_id,
                        ),
                        (
                            instance_id,
                            _clip_regions(
                                source,
                                right_halfplane,
                                reachability[instance_id],
                            ),
                            _clip_regions(
                                source,
                                left_halfplane,
                                reachability[instance_id],
                            ),
                            proof.right_model.front_reading_id,
                        ),
                    )
                )
                current[instance_id] = ()
                interaction_ids[instance_id].append(interaction_id)
        else:
            left_is_profile = len(prepared) > 1 and all(
                isinstance(item[0].left_model, AngularProfileArrivalModelV1)
                for item in prepared
            )
            right_is_profile = len(prepared) > 1 and all(
                isinstance(item[0].right_model, AngularProfileArrivalModelV1)
                for item in prepared
            )
            if len(prepared) > 1 and (left_is_profile ^ right_is_profile):
                profile_component_id = (
                    candidate.left_component_id
                    if left_is_profile
                    else candidate.right_component_id
                )
                other_component_id = (
                    candidate.right_component_id
                    if left_is_profile
                    else candidate.left_component_id
                )
                profile_keep_index = 2 if left_is_profile else 3
                profile_model_side = (
                    "left_model" if left_is_profile else "right_model"
                )
                other_model_side = (
                    "right_model" if left_is_profile else "left_model"
                )
                group_interaction_ids = tuple(item[1] for item in prepared)
                profile_component = component_by_id[profile_component_id]
                for typed_instance_id in profile_component.envelope_instance_ids:
                    instance_id = typed_instance_id.value
                    if instance_id not in current:
                        continue
                    retained = current[instance_id]
                    for item in prepared:
                        keep = item[profile_keep_index]
                        opposite = item[
                            3 if profile_keep_index == 2 else 2
                        ]
                        removed[instance_id].extend(
                            _clip_regions(
                                retained,
                                opposite,
                                reachability[instance_id],
                            )
                        )
                        retained = _clip_regions(
                            retained,
                            keep,
                            reachability[instance_id],
                        )
                    current[instance_id] = retained
                    interaction_ids[instance_id].extend(
                        group_interaction_ids
                    )
                    reading_ids[instance_id].update(
                        getattr(item[0], profile_model_side).front_reading_id
                        for item in prepared
                    )
                profile_retained = tuple(
                    region
                    for instance_id in profile_component.envelope_instance_ids
                    for region in current.get(instance_id.value, ())
                )
                other_component = component_by_id[other_component_id]
                for typed_instance_id in other_component.envelope_instance_ids:
                    instance_id = typed_instance_id.value
                    if instance_id not in current:
                        continue
                    source = current[instance_id]
                    current[instance_id] = _subtract_convex_regions(
                        source,
                        profile_retained,
                        rectangle,
                        reachability[instance_id],
                        reachability,
                        domain_regions,
                        candidate.candidate_id.value,
                    )
                    for clip_index, clip in enumerate(profile_retained):
                        removed[instance_id].extend(
                            _intersect_with_convex_region(
                                source,
                                clip,
                                rectangle,
                                reachability[instance_id],
                                stable_id(
                                    "interaction-removed-overlap",
                                    candidate.candidate_id,
                                    clip_index,
                                ),
                            )
                        )
                    interaction_ids[instance_id].extend(
                        group_interaction_ids
                    )
                    reading_ids[instance_id].update(
                        getattr(item[0], other_model_side).front_reading_id
                        for item in prepared
                    )
            else:
                for component_id, is_profile, keep_index, model_side in (
                    (
                        candidate.left_component_id,
                        left_is_profile,
                        2,
                        "left_model",
                    ),
                    (
                        candidate.right_component_id,
                        right_is_profile,
                        3,
                        "right_model",
                    ),
                ):
                    component = component_by_id[component_id]
                    keep_halfplanes = tuple(
                        item[keep_index] for item in prepared
                    )
                    opposite_halfplanes = tuple(
                        item[3 if keep_index == 2 else 2]
                        for item in prepared
                    )
                    models = tuple(
                        getattr(item[0], model_side) for item in prepared
                    )
                    group_interaction_ids = tuple(
                        item[1] for item in prepared
                    )
                    for typed_instance_id in component.envelope_instance_ids:
                        instance_id = typed_instance_id.value
                        if instance_id not in current:
                            continue
                        source = current[instance_id]
                        if len(prepared) == 1 or is_profile:
                            retained = source
                            for keep, opposite in zip(
                                keep_halfplanes,
                                opposite_halfplanes,
                                strict=True,
                            ):
                                removed[instance_id].extend(
                                    _clip_regions(
                                        retained,
                                        opposite,
                                        reachability[instance_id],
                                    )
                                )
                                retained = _clip_regions(
                                    retained,
                                    keep,
                                    reachability[instance_id],
                                )
                        else:
                            retained_pieces = tuple(
                                region
                                for keep in keep_halfplanes
                                for region in _clip_regions(
                                    source,
                                    keep,
                                    reachability[instance_id],
                                )
                            )
                            retained = _union_regions(
                                retained_pieces,
                                domain_regions,
                                reachability,
                            )
                            removed_region = source
                            for opposite in opposite_halfplanes:
                                removed_region = _clip_regions(
                                    removed_region,
                                    opposite,
                                    reachability[instance_id],
                                )
                            removed[instance_id].extend(removed_region)
                        current[instance_id] = retained
                        interaction_ids[instance_id].extend(
                            group_interaction_ids
                        )
                        reading_ids[instance_id].update(
                            model.front_reading_id for model in models
                        )
        effect = (
            InteractionCoverageEffect.EXACT_SELF_CONTACT_POLICY_B_CLIP
            if candidate.candidate_kind
            is InteractionCandidateKind.EXPLICIT_SELF_CONTACT
            else InteractionCoverageEffect.EXACT_POLICY_B_CLIP
        )
        policy_id = (
            SELF_CONTACT_POLICY_B_V1
            if candidate.candidate_kind
            is InteractionCandidateKind.EXPLICIT_SELF_CONTACT
            else INTRAPATCH_POLICY_B_V1
        )
        for proof, interaction_id, _, _ in prepared:
            applications.append(
                InteractionApplicationV1(
                    interaction_id=interaction_id,
                    policy_id=policy_id,
                    candidate_id=proof.candidate.candidate_id,
                    mutual_arrival_certificate_id=(
                        proof.mutual_arrival_certificate.certificate_id
                    ),
                    equality_locus_id=proof.equality_locus.locus_id,
                    participant_component_ids=(
                        proof.candidate.left_component_id,
                        proof.candidate.right_component_id,
                    ),
                    coverage_effect=effect,
                    creates_new_matter=False,
                    exact_alpha=proof.mutual_arrival_certificate.exact_alpha,
                    same_alpha_batch_identity=(
                        proof.mutual_arrival_certificate.same_alpha_batch_identity
                    ),
                )
            )

    contributions = []
    for instance_id, item in sorted(resolved_by_instance.items()):
        component = component_by_instance[instance_id]
        if current[instance_id] or not any(
            output[0] == instance_id for output in self_outputs
        ):
            freeze_state = FreezeState.UNCHANGED
            if interaction_ids[instance_id]:
                freeze_state = (
                    FreezeState.BOUNDARY_FROZEN_PRESERVED
                    if item.capacity_outcome is not None
                    else FreezeState.FROZEN_AT_MUTUAL_EQUALITY_LOCUS
                )
            contributions.append(
                ResolvedContributionV1(
                    resolved_contribution_id=ResolvedContributionId(
                        stable_id(
                            "resolved-contribution",
                            instance_id,
                            "aggregate",
                        )
                    ),
                    original_envelope_instance=item.envelope_instance,
                    interaction_component_id=component.interaction_component_id,
                    retained_front_reading_ids=frozenset(
                        reading_ids[instance_id]
                    ),
                    retained_exact_regions=current[instance_id],
                    removed_exact_regions=tuple(removed[instance_id]),
                    interaction_ids=tuple(interaction_ids[instance_id]),
                    freeze_state=freeze_state,
                )
            )
        for output_instance, retained, removed_regions, reading_id in self_outputs:
            if output_instance != instance_id:
                continue
            contributions.append(
                ResolvedContributionV1(
                    resolved_contribution_id=ResolvedContributionId(
                        stable_id(
                            "resolved-contribution",
                            instance_id,
                            reading_id,
                        )
                    ),
                    original_envelope_instance=item.envelope_instance,
                    interaction_component_id=component.interaction_component_id,
                    retained_front_reading_ids=frozenset({reading_id}),
                    retained_exact_regions=retained,
                    removed_exact_regions=removed_regions,
                    interaction_ids=tuple(interaction_ids[instance_id]),
                    freeze_state=FreezeState.FROZEN_AT_MUTUAL_EQUALITY_LOCUS,
                )
            )

    retained_regions = tuple(
        region
        for contribution in contributions
        for region in contribution.retained_exact_regions
    )
    resolved_union = _ARRANGEMENT.exact_union(
        retained_regions,
        domain_regions,
        reachability,
    )
    standard = raw_coverage if conservation is None else conservation
    raw_area = ExactScalar(standard.exact_area_expression).as_expr()
    resolved_area = ExactScalar(resolved_union.exact_area_expression).as_expr()
    if (
        exact_sign(raw_area - resolved_area) != 0
        or _raw_set_signature(standard) != _set_signature(resolved_union)
    ):
        diagnostics.append(
            InteractionDiagnosticV1(
                InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN,
                InteractionDiagnosticSeverity.UNSUPPORTED,
                "Policy B clipping did not reproduce the exact RawCoverage set",
                frozenset(
                    component_id
                    for proof in proofs
                    for component_id in (
                        proof.candidate.left_component_id,
                        proof.candidate.right_component_id,
                    )
                ),
            )
        )
    regions_by_component = {
        component.interaction_component_id: tuple(
            region
            for contribution in contributions
            if contribution.interaction_component_id
            == component.interaction_component_id
            for region in contribution.retained_exact_regions
        )
        for component in components
    }
    participating_pairs = {
        tuple(
            sorted(
                (
                    proof.candidate.left_component_id,
                    proof.candidate.right_component_id,
                ),
                key=lambda item: item.value,
            )
        )
        for proof in proofs
        if proof.candidate.left_component_id
        != proof.candidate.right_component_id
    }
    for left_id, right_id in participating_pairs:
        area = _component_intersection_area(
            regions_by_component[left_id],
            regions_by_component[right_id],
            reachability,
        )
        if exact_sign(area) != 0:
            diagnostics.append(
                InteractionDiagnosticV1(
                    InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN,
                    InteractionDiagnosticSeverity.UNSUPPORTED,
                    "Policy B participant interiors still overlap",
                    frozenset({left_id, right_id}),
                )
            )
    return PolicyBResultV1(
        resolved_contributions=tuple(contributions),
        resolved_union=resolved_union,
        applications=tuple(applications),
        diagnostics=tuple(diagnostics),
    )
