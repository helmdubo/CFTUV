"""Exact O(n^2) embedding certificates for source snap and projection.

This module is intentionally internal.  It owns no tolerance and performs no
repair: it only recomputes exact predicates over ``Fraction`` coordinates and
returns immutable public evidence records.  Callers decide whether a non-zero
failure count is an admission refusal or a validation issue.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from functools import cmp_to_key
from hashlib import sha256
import json

from .contracts.metric import (
    GridSnappingLawV1,
    NearPlanarProjectionEmbeddingCertificateV1,
    ProjectionAnchorSelectionLawV1,
    SourceSnapEmbeddingCertificateV1,
)
from .outcomes import NamedOutcome
from .numeric import canonical_primitive_normal
from .robust.predicates import on_segment, orient2d, point_in_loop


_NONE = 0
_POINT = 1
_OVERLAP = 2


@dataclass(frozen=True, slots=True)
class _EdgeOccurrence:
    face_key: str
    index: int
    edge_id: object
    start: object
    end: object

    @property
    def key(self) -> tuple[str, int, str, str, str]:
        return (
            self.face_key,
            self.index,
            self.edge_id.value,
            self.start.value,
            self.end.value,
        )


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _cross3(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left, right):
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def patch_plane_normal(positions, faces):
    """Exact canonical Newell normal over every owner-patch face."""

    x = y = z = Fraction(0)
    for face in faces:
        cycle = face.vertex_cycle
        for index, head_id in enumerate(cycle):
            head = positions[head_id]
            tail = positions[cycle[(index + 1) % len(cycle)]]
            x += (head[1] - tail[1]) * (head[2] + tail[2])
            y += (head[2] - tail[2]) * (head[0] + tail[0])
            z += (head[0] - tail[0]) * (head[1] + tail[1])
    return canonical_primitive_normal((x, y, z))


def _ordered(value: Fraction, denominator: Fraction) -> Fraction:
    return value / denominator


def _segment_relation3(a, b, c, d) -> int:
    """Exact relation of two closed 3D segments."""

    r = _sub(b, a)
    s = _sub(d, c)
    r_cross_s = _cross3(r, s)
    offset = _sub(c, a)
    if not any(r_cross_s):
        if any(_cross3(offset, r)):
            return _NONE
        axis = next((index for index, value in enumerate(r) if value), None)
        if axis is None:
            return _POINT if a == c or a == d else _NONE
        left = sorted((a[axis], b[axis]))
        right = sorted((c[axis], d[axis]))
        low, high = max(left[0], right[0]), min(left[1], right[1])
        if low > high:
            return _NONE
        return _POINT if low == high else _OVERLAP
    if _dot(offset, r_cross_s) != 0:
        return _NONE
    axis = next(index for index, value in enumerate(r_cross_s) if value)
    t = _ordered(_cross3(offset, s)[axis], r_cross_s[axis])
    u = _ordered(_cross3(offset, r)[axis], r_cross_s[axis])
    return _POINT if 0 <= t <= 1 and 0 <= u <= 1 else _NONE


def _segment_relation2(a, b, c, d) -> int:
    """Exact 2D relation, reusing the kernel's exact orientation predicates."""

    signs = (
        orient2d(a, b, c),
        orient2d(a, b, d),
        orient2d(c, d, a),
        orient2d(c, d, b),
    )
    if signs == (0, 0, 0, 0):
        axis = 0 if a[0] != b[0] else 1
        left = sorted((a[axis], b[axis]))
        right = sorted((c[axis], d[axis]))
        low, high = max(left[0], right[0]), min(left[1], right[1])
        if low > high:
            return _NONE
        return _POINT if low == high else _OVERLAP
    if (
        (signs[0] == 0 and on_segment(c, a, b))
        or (signs[1] == 0 and on_segment(d, a, b))
        or (signs[2] == 0 and on_segment(a, c, d))
        or (signs[3] == 0 and on_segment(b, c, d))
        or (signs[0] * signs[1] < 0 and signs[2] * signs[3] < 0)
    ):
        return _POINT
    return _NONE


def _all_occurrences(faces) -> tuple[_EdgeOccurrence, ...]:
    result = []
    for face in faces:
        cycle = face.vertex_cycle
        for index, start in enumerate(cycle):
            result.append(
                _EdgeOccurrence(
                    face.face_id.value,
                    index,
                    face.edge_cycle[index],
                    start,
                    cycle[(index + 1) % len(cycle)],
                )
            )
    return tuple(result)


def _edge_occurrence_groups(faces):
    groups: dict[object, list[_EdgeOccurrence]] = {}
    for occurrence in _all_occurrences(faces):
        groups.setdefault(occurrence.edge_id, []).append(occurrence)
    for edge_id, occurrences in groups.items():
        endpoints = {
            frozenset((occurrence.start, occurrence.end))
            for occurrence in occurrences
        }
        if len(endpoints) != 1:
            raise ValueError(
                f"physical edge {edge_id.value!r} has inconsistent endpoints"
            )
    return groups


def _boundary_occurrences(faces) -> tuple[_EdgeOccurrence, ...]:
    """Boundary half-edges by exact physical-edge incidence."""

    groups = _edge_occurrence_groups(faces)
    return tuple(
        sorted(
            (
                occurrences[0]
                for occurrences in groups.values()
                if len(occurrences) == 1
            ),
            key=lambda item: item.key,
        )
    )


def _source_edge_occurrences(faces) -> tuple[_EdgeOccurrence, ...]:
    """One canonical occurrence of every physical source edge."""

    return tuple(
        sorted(
            (
                min(occurrences, key=lambda item: item.key)
                for occurrences in _edge_occurrence_groups(faces).values()
            ),
            key=lambda item: item.key,
        )
    )


def _boundary_loops(edges) -> tuple[tuple[_EdgeOccurrence, ...], ...]:
    outgoing: dict[object, list[_EdgeOccurrence]] = {}
    incoming: dict[object, list[_EdgeOccurrence]] = {}
    for edge in edges:
        outgoing.setdefault(edge.start, []).append(edge)
        incoming.setdefault(edge.end, []).append(edge)
    vertices = set(outgoing) | set(incoming)
    malformed = tuple(
        sorted(
            (
                vertex
                for vertex in vertices
                if len(outgoing.get(vertex, ())) != 1
                or len(incoming.get(vertex, ())) != 1
            ),
            key=lambda item: item.value,
        )
    )
    if malformed:
        raise ValueError(
            "boundary incidence is not a disjoint union of directed cycles: "
            + ", ".join(item.value for item in malformed)
        )
    unused = set(edges)
    loops = []
    while unused:
        first = min(unused, key=lambda item: item.key)
        loop = []
        current = first
        while True:
            if current not in unused:
                if current != first:
                    raise ValueError("boundary walk re-entered a different cycle")
                break
            unused.remove(current)
            loop.append(current)
            current = outgoing[current.end][0]
        if not loop or loop[-1].end != first.start:
            raise ValueError("boundary walk is not closed")
        loops.append(tuple(loop))
    return tuple(sorted(loops, key=lambda loop: tuple(item.key for item in loop)))


def _nonadjacent_pairs(edges):
    for left_index, left in enumerate(edges):
        left_ids = {left.start, left.end}
        for right in edges[left_index + 1 :]:
            if left_ids.isdisjoint((right.start, right.end)):
                yield left, right


def _digest(value) -> str:
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return sha256(payload.encode("utf-8")).hexdigest()


def _canonical_cycle(values):
    sequence = tuple(values)
    return min(sequence[index:] + sequence[:index] for index in range(len(sequence)))


def _cyclic_digest(loops) -> str:
    records = sorted(
        _canonical_cycle(tuple(edge.edge_id.value for edge in loop))
        for loop in loops
    )
    return _digest(records)


def _fan_incident_edges(faces):
    incident: dict[object, list[tuple[str, object]]] = {}
    for edge in _source_edge_occurrences(faces):
        incident.setdefault(edge.start, []).append((edge.edge_id.value, edge.end))
        incident.setdefault(edge.end, []).append((edge.edge_id.value, edge.start))
    return incident


def _ray_half(ray) -> int:
    return 0 if ray[1] > 0 or (ray[1] == 0 and ray[0] >= 0) else 1


def _fan_rotation(rays):
    vectors = {edge_id: vector for edge_id, vector in rays}
    relations = {}
    edge_ids = tuple(sorted(vectors))
    ambiguous = sum(not any(vectors[edge_id]) for edge_id in edge_ids)
    for index, left in enumerate(edge_ids):
        for right in edge_ids[index + 1 :]:
            left_ray, right_ray = vectors[left], vectors[right]
            cross = orient2d((0, 0), left_ray, right_ray)
            dot = left_ray[0] * right_ray[0] + left_ray[1] * right_ray[1]
            relations[(left, right)] = (cross, dot)

    def relation(left, right):
        cross, dot = relations[tuple(sorted((left, right)))]
        return (cross, dot) if left < right else (-cross, dot)

    def compare(left, right):
        left_half, right_half = _ray_half(vectors[left]), _ray_half(vectors[right])
        if left_half != right_half:
            return -1 if left_half < right_half else 1
        cross, _ = relation(left, right)
        if cross:
            return -1 if cross > 0 else 1
        return (left > right) - (left < right)

    same_direction = []
    for index, left in enumerate(edge_ids):
        for right in edge_ids[index + 1 :]:
            cross, dot = relation(left, right)
            if cross == 0 and dot > 0:
                same_direction.append((left, right))
    ambiguous += len(same_direction)
    if len(edge_ids) <= 2:
        rotation = edge_ids
    else:
        angular = tuple(sorted(edge_ids, key=cmp_to_key(compare)))
        rotation = _canonical_cycle(angular)
    record = {
        "rotation": rotation,
        "same_direction_pairs": same_direction,
        "zero_ray_edge_ids": tuple(
            edge_id for edge_id in edge_ids if not any(vectors[edge_id])
        ),
    }
    pair_tests = len(edge_ids) * (len(edge_ids) - 1) // 2
    return record, pair_tests, ambiguous


def _fan_digest(faces, positions) -> tuple[str, int, int]:
    records = {}
    pair_tests = 0
    ambiguous = 0
    for vertex, incident in sorted(
        _fan_incident_edges(faces).items(), key=lambda item: item[0].value
    ):
        rays = tuple(
            (edge_id, _sub(positions[other], positions[vertex]))
            for edge_id, other in incident
        )
        record, tests, failures = _fan_rotation(rays)
        records[vertex.value] = record
        pair_tests += tests
        ambiguous += failures
    return _digest(records), pair_tests, ambiguous


def _source_chart(before, normal, faces, expected_sign):
    dropped = max(range(3), key=lambda axis: abs(normal[axis]))
    kept = tuple(axis for axis in range(3) if axis != dropped)
    chart = {
        vertex: (point[kept[0]], point[kept[1]])
        for vertex, point in before.items()
    }
    first_sign = 0
    for face in faces:
        area = _signed_twice_area(tuple(chart[item] for item in face.vertex_cycle))
        first_sign = (area > 0) - (area < 0)
        if first_sign:
            break
    negated = bool(first_sign and first_sign != expected_sign)
    if negated:
        chart = {vertex: (-point[0], point[1]) for vertex, point in chart.items()}
    return chart, dropped, negated


def _anchor_ids(positions):
    vertex_ids = tuple(sorted(positions, key=lambda item: item.value))
    origin = vertex_ids[0]
    first = next(
        (item for item in vertex_ids[1:] if positions[item] != positions[origin]),
        None,
    )
    if first is None:
        return (origin,)
    second = next(
        (
            item
            for item in vertex_ids[1:]
            if orient2d(positions[origin], positions[first], positions[item]) != 0
        ),
        None,
    )
    return (origin, first) if second is None else (origin, first, second)


def _corner_degenerated(positions, corner) -> bool:
    previous, vertex, following = corner
    if any(item not in positions for item in corner):
        return True
    left = _sub(positions[previous], positions[vertex])
    right = _sub(positions[following], positions[vertex])
    return not any(_cross3(left, right))


def _corner_unchanged(before, after, corner) -> bool:
    return all(
        item in before and item in after and before[item] == after[item]
        for item in corner
    )


def build_source_snap_embedding_certificate(
    *,
    before,
    after,
    faces,
    intended_corners,
    unclassifiable_corners=(),
    snapping_law,
):
    vertex_ids = tuple(sorted(before, key=lambda item: item.value))
    edges = _source_edge_occurrences(faces)
    new_coincidences = 0
    pair_tests = 0
    for index, left in enumerate(vertex_ids):
        for right in vertex_ids[index + 1 :]:
            pair_tests += 1
            if before[left] != before[right] and after[left] == after[right]:
                new_coincidences += 1
    collapsed = sum(
        before[edge.start] != before[edge.end]
        and after[edge.start] == after[edge.end]
        for edge in edges
    )
    new_intersections = 0
    for left, right in _nonadjacent_pairs(edges):
        pair_tests += 1
        prior = _segment_relation3(
            before[left.start], before[left.end], before[right.start], before[right.end]
        )
        snapped = _segment_relation3(
            after[left.start], after[left.end], after[right.start], after[right.end]
        )
        if prior == _NONE and snapped != _NONE:
            new_intersections += 1
    degenerated = sum(
        _corner_degenerated(after, corner) for corner in intended_corners
    )
    unchanged_unclassifiable = sum(
        _corner_unchanged(before, after, corner)
        for corner in unclassifiable_corners
    )
    return SourceSnapEmbeddingCertificateV1(
        snapping_law=snapping_law,
        source_vertex_ids=vertex_ids,
        source_vertex_count=len(vertex_ids),
        source_edge_count=len(edges),
        intended_right_corner_count=len(intended_corners),
        newly_coincident_vertex_pair_count=new_coincidences,
        collapsed_nonzero_source_edge_count=collapsed,
        new_nonadjacent_edge_intersection_count=new_intersections,
        unclassifiable_source_corner_count=len(unclassifiable_corners),
        unchanged_unclassifiable_source_corner_count=unchanged_unclassifiable,
        degenerated_intended_right_corner_count=degenerated,
        exact_pair_test_count=pair_tests,
    )


def source_snap_violations(certificate):
    checks = (
        (
            certificate.newly_coincident_vertex_pair_count,
            NamedOutcome.SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED,
        ),
        (
            certificate.collapsed_nonzero_source_edge_count,
            NamedOutcome.SOURCE_SNAP_NONZERO_EDGE_COLLAPSED,
        ),
        (
            certificate.new_nonadjacent_edge_intersection_count,
            NamedOutcome.SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION,
        ),
        (
            certificate.degenerated_intended_right_corner_count,
            NamedOutcome.SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED,
        ),
    )
    return tuple(outcome for count, outcome in checks if count)


def source_snap_violation(certificate):
    failures = source_snap_violations(certificate)
    return failures[0] if failures else None


def _signed_twice_area(points) -> Fraction:
    return sum(
        (
            points[index][0] * points[(index + 1) % len(points)][1]
            - points[index][1] * points[(index + 1) % len(points)][0]
            for index in range(len(points))
        ),
        Fraction(0),
    )


def _geometric_component_count(loops, positions) -> tuple[int, int]:
    parents = list(range(len(loops)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(left, right):
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parents[right_root] = left_root

    tests = 0
    for left_index, left in enumerate(loops):
        for right_index in range(left_index + 1, len(loops)):
            right = loops[right_index]
            related = False
            for left_edge in left:
                for right_edge in right:
                    tests += 1
                    if _segment_relation2(
                        positions[left_edge.start],
                        positions[left_edge.end],
                        positions[right_edge.start],
                        positions[right_edge.end],
                    ) != _NONE:
                        related = True
                        break
                if related:
                    break
            if related:
                union(left_index, right_index)
    return len({find(index) for index in range(len(loops))}), tests


def _nesting_depths(loops, positions) -> tuple[tuple[int, ...], int]:
    points = [tuple(positions[edge.start] for edge in loop) for loop in loops]
    tests = 0
    depths = []
    for index, loop in enumerate(points):
        if not loop:
            depths.append(0)
            continue
        depth = 0
        for other_index, other in enumerate(points):
            if index == other_index or not other:
                continue
            tests += 1
            if point_in_loop(loop[0], other) == 1:
                depth += 1
        depths.append(depth)
    return tuple(depths), tests


@dataclass(frozen=True, slots=True)
class _ProjectionOrientationFacts:
    source_face_signs: tuple[int, ...]
    projected_face_signs: tuple[int, ...]
    source_loop_signs: tuple[int, ...]
    projected_loop_signs: tuple[int, ...]
    source_depths: tuple[int, ...]
    projected_depths: tuple[int, ...]
    orientation_mismatches: int
    nesting_mismatches: int
    pair_tests: int


def _orientation_facts(
    *, faces, loops, source_chart, projected, expected_orientation_sign
):
    def signs_for(cycles, positions):
        return tuple(
            (
                lambda area: (area > 0) - (area < 0)
            )(_signed_twice_area(tuple(positions[item] for item in cycle)))
            for cycle in cycles
        )

    face_cycles = tuple(face.vertex_cycle for face in faces)
    loop_cycles = tuple(tuple(edge.start for edge in loop) for loop in loops)
    source_face_signs = signs_for(face_cycles, source_chart)
    projected_face_signs = signs_for(face_cycles, projected)
    source_loop_signs = signs_for(loop_cycles, source_chart)
    projected_loop_signs = signs_for(loop_cycles, projected)
    source_depths, source_tests = _nesting_depths(loops, source_chart)
    projected_depths, projected_tests = _nesting_depths(loops, projected)
    face_mismatches = sum(
        source != expected_orientation_sign
        or target != expected_orientation_sign
        or source != target
        for source, target in zip(
            source_face_signs, projected_face_signs, strict=True
        )
    )
    loop_mismatches = sum(
        source != expected_orientation_sign * (-1 if source_depth % 2 else 1)
        or target
        != expected_orientation_sign * (-1 if target_depth % 2 else 1)
        or source != target
        for source, target, source_depth, target_depth in zip(
            source_loop_signs,
            projected_loop_signs,
            source_depths,
            projected_depths,
            strict=True,
        )
    )
    nesting_mismatches = sum(
        source != target
        for source, target in zip(source_depths, projected_depths, strict=True)
    )
    return _ProjectionOrientationFacts(
        source_face_signs,
        projected_face_signs,
        source_loop_signs,
        projected_loop_signs,
        source_depths,
        projected_depths,
        face_mismatches + loop_mismatches,
        nesting_mismatches,
        source_tests + projected_tests,
    )


def build_projection_embedding_certificate(
    *, before, projected, faces, normal, expected_orientation_sign
):
    edges = _boundary_occurrences(faces)
    loops = _boundary_loops(edges)
    source_chart, dropped_axis, first_axis_negated = _source_chart(
        before, normal, faces, expected_orientation_sign
    )
    occurrences = tuple(edge.start for edge in edges)
    pair_tests = 0
    coincidences = 0
    for index, left in enumerate(occurrences):
        for right in occurrences[index + 1 :]:
            pair_tests += 1
            if projected[left] == projected[right]:
                coincidences += 1
    collapsed = sum(projected[edge.start] == projected[edge.end] for edge in edges)
    new_crossings = 0
    new_overlaps = 0
    for left, right in _nonadjacent_pairs(edges):
        pair_tests += 1
        prior = _segment_relation3(
            before[left.start],
            before[left.end],
            before[right.start],
            before[right.end],
        )
        result = _segment_relation2(
            projected[left.start],
            projected[left.end],
            projected[right.start],
            projected[right.end],
        )
        if result == _OVERLAP and prior != _OVERLAP:
            new_overlaps += 1
        elif result == _POINT and prior == _NONE:
            new_crossings += 1
    orientation = _orientation_facts(
        faces=faces,
        loops=loops,
        source_chart=source_chart,
        projected=projected,
        expected_orientation_sign=expected_orientation_sign,
    )
    pair_tests += orientation.pair_tests
    projected_components, projected_component_tests = _geometric_component_count(
        loops, projected
    )
    pair_tests += projected_component_tests
    source_cyclic = _cyclic_digest(loops)
    projected_cyclic = _cyclic_digest(loops)
    source_fan, source_fan_pair_tests, source_fan_ambiguities = _fan_digest(
        faces, source_chart
    )
    projected_fan, projected_fan_pair_tests, projected_fan_ambiguities = (
        _fan_digest(faces, projected)
    )
    pair_tests += source_fan_pair_tests + projected_fan_pair_tests
    source_ids = tuple(sorted(before, key=lambda item: item.value))
    resolved_anchor = _anchor_ids(projected)
    anchor_law = ProjectionAnchorSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_ON_RESOLVED_PLANE_V1
    return NearPlanarProjectionEmbeddingCertificateV1(
        source_vertex_ids=source_ids,
        source_chart_dropped_axis=dropped_axis,
        source_chart_first_axis_negated=first_axis_negated,
        source_boundary_occurrence_count=len(occurrences),
        source_boundary_component_count=len(loops),
        projected_boundary_component_count=projected_components,
        coincident_boundary_occurrence_pair_count=coincidences,
        collapsed_boundary_edge_occurrence_count=collapsed,
        new_nonadjacent_edge_intersection_count=new_crossings,
        new_nonadjacent_collinear_overlap_count=new_overlaps,
        source_face_orientation_signs=orientation.source_face_signs,
        projected_face_orientation_signs=orientation.projected_face_signs,
        source_boundary_loop_orientation_signs=orientation.source_loop_signs,
        projected_boundary_loop_orientation_signs=orientation.projected_loop_signs,
        source_boundary_loop_nesting_depths=orientation.source_depths,
        projected_boundary_loop_nesting_depths=orientation.projected_depths,
        orientation_mismatch_count=orientation.orientation_mismatches,
        nesting_mismatch_count=orientation.nesting_mismatches,
        source_cyclic_order_sha256=source_cyclic,
        projected_cyclic_order_sha256=projected_cyclic,
        anchor_selection_law=anchor_law,
        source_anchor_vertex_ids=resolved_anchor,
        projected_anchor_vertex_ids=resolved_anchor,
        resolved_plane_basis_unavailable_count=int(len(resolved_anchor) != 3),
        source_fan_identity_sha256=source_fan,
        projected_fan_identity_sha256=projected_fan,
        source_fan_ambiguity_count=source_fan_ambiguities,
        projected_fan_ambiguity_count=projected_fan_ambiguities,
        exact_pair_test_count=pair_tests,
    )


def projection_violations(certificate):
    checks = (
        (
            certificate.coincident_boundary_occurrence_pair_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED,
        ),
        (
            certificate.collapsed_boundary_edge_occurrence_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED,
        ),
        (
            certificate.new_nonadjacent_edge_intersection_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION,
        ),
        (
            certificate.new_nonadjacent_collinear_overlap_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP,
        ),
        (
            certificate.orientation_mismatch_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED,
        ),
        (
            certificate.source_boundary_component_count
            != certificate.projected_boundary_component_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED,
        ),
        (
            certificate.nesting_mismatch_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED,
        ),
        (
            certificate.source_cyclic_order_sha256
            != certificate.projected_cyclic_order_sha256,
            NamedOutcome.NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED,
        ),
        (
            certificate.source_anchor_vertex_ids
            != certificate.projected_anchor_vertex_ids,
            NamedOutcome.NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED,
        ),
        (
            certificate.resolved_plane_basis_unavailable_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE,
        ),
        (
            certificate.source_fan_identity_sha256
            != certificate.projected_fan_identity_sha256
            or certificate.source_fan_ambiguity_count
            or certificate.projected_fan_ambiguity_count,
            NamedOutcome.NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED,
        ),
    )
    return tuple(outcome for failed, outcome in checks if failed)


def projection_violation(certificate):
    failures = projection_violations(certificate)
    return failures[0] if failures else None
