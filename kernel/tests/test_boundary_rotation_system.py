from __future__ import annotations

from dataclasses import replace

import pytest
import sympy as sp

from cftuv_envelope.reference.arrangement import (
    ExactArrangementCollinearBranchUnproven,
    ExactSegmentArrangementBackend,
    ExactTouchingHoleTopologyUnproven,
    ExhaustiveExactArrangementBackend,
    _OrientedOutput,
    _boundary_rotation_system,
)
from cftuv_envelope.reference.common import make_segment
from cftuv_envelope.reference.contracts import (
    RawCoverageEdgeV1,
    ReachabilityCertificateV1,
)
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    PlanarLoop,
    PlanarRegion,
)
from cftuv_envelope.reference.provenance import make_reference_provenance


def _region(
    name: str,
    coordinates: tuple[tuple[object, object], ...],
    *,
    holes: tuple[tuple[tuple[object, object], ...], ...] = (),
    domain: bool = False,
) -> PlanarRegion:
    provenance = make_reference_provenance(
        envelope_spec_ids=(
            frozenset() if domain else frozenset({f"spec:{name}"})
        ),
        envelope_instance_ids=(
            frozenset() if domain else frozenset({f"instance:{name}"})
        ),
        support_ids=frozenset({f"support:{name}"}),
        physical_edge_ids=frozenset({f"edge:{name}"}),
        chain_use_ids=frozenset({f"use:{name}"}),
        patch_domain_ids=frozenset({"domain"}),
        boundary_constraint_ids=(
            frozenset({f"constraint:{name}"}) if domain else frozenset()
        ),
    )

    def make_loop(
        suffix: str,
        loop_coordinates: tuple[tuple[object, object], ...],
    ) -> PlanarLoop:
        points = tuple(
            ExactPlanarPoint.from_values(*item) for item in loop_coordinates
        )
        certificates = tuple(
            ConstructionCertificate(
                ConstructionKind.SOURCE_VERTEX,
                source_vertex_ids=frozenset(
                    {f"vertex:{name}:{suffix}:{index}"}
                ),
            )
            for index in range(len(points))
        )
        return PlanarLoop(
            f"loop:{name}:{suffix}",
            tuple(
                make_segment(
                    f"segment:{name}:{suffix}:{index}",
                    points[index],
                    points[(index + 1) % len(points)],
                    support_ids=provenance.support_ids,
                    provenance=provenance,
                    start_certificates=frozenset({certificates[index]}),
                    end_certificates=frozenset(
                        {certificates[(index + 1) % len(points)]}
                    ),
                    boundary_constraint_ids=(
                        provenance.boundary_constraint_ids
                    ),
                )
                for index in range(len(points))
            ),
        )

    return PlanarRegion(
        region_id=f"region:{name}",
        outer=make_loop("outer", coordinates),
        holes=tuple(
            make_loop(f"hole:{index}", item)
            for index, item in enumerate(holes)
        ),
        contributor_instance_ids=(
            frozenset() if domain else frozenset({f"instance:{name}"})
        ),
        contributor_spec_ids=(
            frozenset() if domain else frozenset({f"spec:{name}"})
        ),
    )


def _reachability(
    regions: tuple[PlanarRegion, ...],
) -> dict[str, ReachabilityCertificateV1]:
    return {
        instance_id: ReachabilityCertificateV1(
            front_component_ids=frozenset({f"front:{instance_id}"}),
            source_launch_reachable=True,
            initial_branch_count=1,
            effective_branch_count=1,
            boundary_event_keys=(),
            bypass_used=False,
        )
        for region in regions
        for instance_id in region.contributor_instance_ids
    }


def _union(
    regions: tuple[PlanarRegion, ...],
    *,
    backend=None,
):
    domain = _region(
        "domain",
        ((-20, -20), (20, -20), (20, 20), (-20, 20)),
        domain=True,
    )
    return (backend or ExactSegmentArrangementBackend()).exact_union(
        regions,
        (domain,),
        _reachability(regions),
    )


def _assert_point_contact(result, expected_loops: int) -> None:
    assert len(result.regions) == expected_loops
    assert len(result.loops) == expected_loops
    assert len(result.point_contacts) == 1
    contact = next(iter(result.point_contacts))
    occurrences = {
        item.boundary_occurrence_id
        for item in result.boundary_vertex_occurrences
        if item.arrangement_point_id == contact.arrangement_point_id
    }
    assert len(occurrences) == expected_loops
    assert len(contact.participating_loop_ids) == expected_loops


def test_two_rectangles_touching_at_one_corner_remain_two_area_regions():
    regions = (
        _region("a", ((0, 0), (2, 0), (2, 2), (0, 2))),
        _region("b", ((2, 2), (4, 2), (4, 4), (2, 4))),
    )

    result = _union(regions)

    _assert_point_contact(result, 2)
    assert sp.sympify(result.exact_area_expression) == 8
    assert result.branch_point_count == 1
    assert result.max_incident_degree == 4


@pytest.mark.parametrize(
    ("name", "coordinates"),
    (
        ("east", ((0, 0), (4, 0), (3, 1))),
        ("north-west", ((0, 0), (-3, 2), (-4, 1))),
        ("south", ((0, 0), (1, -4), (2, -3))),
    ),
)
def test_three_triangles_fixture_parts_are_positive_area(name, coordinates):
    region = _region(name, coordinates)
    assert sp.sympify(
        sum(
            (
                segment.start.x.as_expr()
                * segment.end.y.as_expr()
                - segment.end.x.as_expr()
                * segment.start.y.as_expr()
            )
            for segment in region.outer.segments
        )
        / 2
    ) > 0


def test_three_components_touching_one_exact_point_have_three_occurrences():
    regions = (
        _region("east", ((0, 0), (4, 0), (3, 1))),
        _region("north-west", ((0, 0), (-3, 2), (-4, 1))),
        _region("south", ((0, 0), (1, -4), (2, -3))),
    )

    result = _union(regions)

    _assert_point_contact(result, 3)


def test_three_exact_rotated_rectangles_touch_only_at_one_point():
    root_three = sp.sqrt(3)
    regions = (
        _region("rect-east", ((0, 0), (2, 0), (2, 1), (0, 1))),
        _region(
            "rect-north-west",
            (
                (0, 0),
                (-1, root_three),
                (-1 - root_three / 2, root_three - sp.Rational(1, 2)),
                (-root_three / 2, -sp.Rational(1, 2)),
            ),
        ),
        _region(
            "rect-south-west",
            (
                (0, 0),
                (-1, -root_three),
                (-1 + root_three / 2, -root_three - sp.Rational(1, 2)),
                (root_three / 2, -sp.Rational(1, 2)),
            ),
        ),
    )

    result = _union(regions)

    _assert_point_contact(result, 3)


def test_four_sided_point_touch_keeps_four_topological_occurrences():
    regions = (
        _region("east", ((0, 0), (4, 0), (3, 1))),
        _region("north", ((0, 0), (0, 4), (-1, 3))),
        _region("west", ((0, 0), (-4, 0), (-3, -1))),
        _region("south", ((0, 0), (0, -4), (1, -3))),
    )

    result = _union(regions)

    _assert_point_contact(result, 4)
    assert result.max_incident_degree == 8


def test_before_at_after_contact_changes_only_at_exact_area_event():
    first = _region("first", ((0, 0), (2, 0), (2, 2), (0, 2)))
    before = _union(
        (first, _region("second", ((3, 3), (5, 3), (5, 5), (3, 5))))
    )
    at = _union(
        (first, _region("second", ((2, 2), (4, 2), (4, 4), (2, 4))))
    )
    after = _union(
        (first, _region("second", ((1, 1), (3, 1), (3, 3), (1, 3))))
    )

    assert (len(before.regions), len(before.point_contacts)) == (2, 0)
    assert (len(at.regions), len(at.point_contacts)) == (2, 1)
    assert (len(after.regions), len(after.point_contacts)) == (1, 0)
    assert sp.sympify(after.exact_area_expression) == 7


def test_tangential_vertex_to_edge_contact_does_not_weld_components():
    regions = (
        _region("base", ((0, 0), (4, 0), (4, 2), (0, 2))),
        _region("tangent", ((2, 2), (3, 4), (1, 4))),
    )

    result = _union(regions)

    _assert_point_contact(result, 2)


def test_internalized_equivalent_generators_merge_provenance_not_edges():
    regions = (
        _region("generator-a", ((0, 0), (4, 0), (4, 4), (0, 4))),
        _region("generator-b", ((0, 0), (4, 0), (4, 4), (0, 4))),
    )

    result = _union(regions)

    assert len(result.regions) == 1
    assert len(result.loops) == 1
    assert not result.point_contacts
    assert next(iter(result.regions)).contributor_envelope_spec_ids == {
        "spec:generator-a",
        "spec:generator-b",
    }


def test_concave_component_can_point_touch_an_independent_component():
    regions = (
        _region(
            "concave",
            (
                (0, 0),
                (4, 0),
                (4, 4),
                (3, 4),
                (3, 1),
                (1, 1),
                (1, 4),
                (0, 4),
            ),
        ),
        _region("tip", ((4, 4), (6, 4), (6, 6), (4, 6))),
    )

    result = _union(regions)

    _assert_point_contact(result, 2)


def test_touching_hole_and_touching_holes_fail_with_named_exact_boundary():
    touching_outer = _region(
        "touching-outer",
        ((0, 0), (6, 0), (6, 6), (0, 6)),
        holes=(((3, 0), (2, 2), (4, 2)),),
    )
    touching_holes = _region(
        "touching-holes",
        ((0, 0), (8, 0), (8, 8), (0, 8)),
        holes=(
            ((2, 2), (2, 4), (4, 3)),
            ((4, 3), (6, 4), (6, 2)),
        ),
    )

    for region in (touching_outer, touching_holes):
        with pytest.raises(ExactTouchingHoleTopologyUnproven):
            _union((region,))


def test_input_permutation_and_cyclic_loop_shift_preserve_rotation_system():
    regions = (
        _region("a", ((0, 0), (2, 0), (2, 2), (0, 2))),
        _region("b", ((2, 2), (4, 2), (4, 4), (2, 4))),
    )
    baseline = _union(regions)
    permuted = _union(tuple(reversed(regions)))
    shifted = tuple(
        replace(
            region,
            outer=replace(
                region.outer,
                segments=(
                    region.outer.segments[1:]
                    + region.outer.segments[:1]
                ),
            ),
        )
        for region in regions
    )
    shifted_result = _union(shifted)
    reversed_storage = tuple(
        replace(
            region,
            outer=replace(
                region.outer,
                segments=tuple(reversed(region.outer.segments)),
            ),
        )
        for region in regions
    )
    reversed_storage_result = _union(reversed_storage)

    for result in (permuted, shifted_result, reversed_storage_result):
        assert result.vertices == baseline.vertices
        assert result.edges == baseline.edges
        assert result.loops == baseline.loops
        assert result.regions == baseline.regions
        assert (
            result.boundary_vertex_occurrences
            == baseline.boundary_vertex_occurrences
        )
        assert result.point_contacts == baseline.point_contacts


def test_rigid_rotation_and_uniform_scale_preserve_semantic_topology():
    coordinates = (
        ((0, 0), (2, 0), (2, 2), (0, 2)),
        ((2, 2), (4, 2), (4, 4), (2, 4)),
    )
    baseline = _union(
        tuple(_region(name, item) for name, item in zip(("a", "b"), coordinates))
    )

    def transform(point):
        x, y = point
        return (-3 * y + 7, 3 * x - 11)

    transformed = _union(
        tuple(
            _region(name, tuple(transform(point) for point in item))
            for name, item in zip(("a", "b"), coordinates)
        )
    )

    assert transformed.edges == baseline.edges
    assert {
        (item.loop_id, item.kind, item.ordered_edge_ids)
        for item in transformed.loops
    } == {
        (item.loop_id, item.kind, item.ordered_edge_ids)
        for item in baseline.loops
    }
    assert transformed.regions == baseline.regions
    assert (
        transformed.boundary_vertex_occurrences
        == baseline.boundary_vertex_occurrences
    )
    assert transformed.point_contacts == baseline.point_contacts
    assert sp.sympify(transformed.exact_area_expression) == 9 * sp.sympify(
        baseline.exact_area_expression
    )


def test_broadphase_and_exhaustive_share_one_rotation_face_walk():
    regions = (
        _region("a", ((0, 0), (2, 0), (2, 2), (0, 2))),
        _region("b", ((2, 2), (4, 2), (4, 4), (2, 4))),
    )
    broadphase = _union(regions)
    exhaustive = _union(regions, backend=ExhaustiveExactArrangementBackend())

    assert broadphase.vertices == exhaustive.vertices
    assert broadphase.edges == exhaustive.edges
    assert broadphase.loops == exhaustive.loops
    assert broadphase.regions == exhaustive.regions
    assert (
        broadphase.boundary_vertex_occurrences
        == exhaustive.boundary_vertex_occurrences
    )
    assert broadphase.point_contacts == exhaustive.point_contacts


def test_duplicate_collinear_outgoing_rays_fail_without_id_tie_break():
    provenance = make_reference_provenance(
        envelope_spec_ids=frozenset({"spec"}),
        envelope_instance_ids=frozenset({"instance"}),
        support_ids=frozenset({"support"}),
        patch_domain_ids=frozenset({"domain"}),
    )
    certificate = ConstructionCertificate(
        ConstructionKind.SOURCE_VERTEX,
        source_vertex_ids=frozenset({"vertex"}),
    )
    reachability = ReachabilityCertificateV1(
        frozenset({"front"}), True, 1, 1, (), False
    )
    point_by_id = {
        "a": ExactPlanarPoint.from_values(0, 0),
        "z1": ExactPlanarPoint.from_values(-1, 0),
        "z2": ExactPlanarPoint.from_values(0, -1),
        "z3": ExactPlanarPoint.from_values(1, 0),
        "z4": ExactPlanarPoint.from_values(2, 0),
    }
    edge_specs = (
        ("in-a", "z1", "a"),
        ("in-b", "z2", "a"),
        ("out-a", "a", "z3"),
        ("out-b", "a", "z4"),
    )
    edges = tuple(
        RawCoverageEdgeV1(
            edge_id,
            start_id,
            end_id,
            provenance,
            frozenset({certificate}),
            reachability,
        )
        for edge_id, start_id, end_id in edge_specs
    )
    outputs = {
        edge_id: _OrientedOutput(
            point_by_id[start_id],
            point_by_id[end_id],
            provenance,
            frozenset({certificate}),
            frozenset({certificate}),
            reachability,
        )
        for edge_id, start_id, end_id in edge_specs
    }

    with pytest.raises(ExactArrangementCollinearBranchUnproven):
        _boundary_rotation_system(edges, outputs)
