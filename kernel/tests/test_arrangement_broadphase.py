from __future__ import annotations

from dataclasses import replace
from hashlib import sha256

import pytest
import sympy as sp

from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.reference import arrangement as arrangement_module
from cftuv_envelope.reference.arrangement import (
    ExactSegmentAabbV1,
    ExactSegmentArrangementBackend,
    ExhaustiveExactArrangementBackend,
)
from cftuv_envelope.reference.arrangement_protocol import ArrangementUnionV1
from cftuv_envelope.reference.common import make_region, make_segment
from cftuv_envelope.reference.contracts import ReachabilityCertificateV1
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
)
from cftuv_envelope.reference.provenance import make_reference_provenance


def _region(
    name: str,
    coordinates: tuple[tuple[object, object], ...],
    *,
    domain: bool = False,
) -> PlanarRegion:
    points = tuple(ExactPlanarPoint.from_values(*item) for item in coordinates)
    certificates = tuple(
        ConstructionCertificate(
            ConstructionKind.SOURCE_VERTEX,
            source_vertex_ids=frozenset({f"vertex:{name}:{index}"}),
        )
        for index in range(len(points))
    )
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
            boundary_constraint_ids=provenance.boundary_constraint_ids,
        )
        for index in range(len(points))
    )
    result = make_region(
        f"region:{name}",
        segments,
        spec_id="" if domain else f"spec:{name}",
        instance_id="" if domain else f"instance:{name}",
    )
    assert result is not None
    if domain:
        result = replace(
            result,
            contributor_instance_ids=frozenset(),
            contributor_spec_ids=frozenset(),
        )
    return result


def _reachability(
    contribution_regions: tuple[PlanarRegion, ...],
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
        for region in contribution_regions
        for instance_id in region.contributor_instance_ids
    }


def _semantic_digest(result: ArrangementUnionV1) -> str:
    semantic_projection = {
        "vertices": result.vertices,
        "atomic_edges": result.edges,
        "loops": result.loops,
        "regions": result.regions,
        "boundary_vertex_occurrences": result.boundary_vertex_occurrences,
        "point_contacts": result.point_contacts,
        "exact_area_expression": result.exact_area_expression,
    }
    return sha256(canonical_json_bytes(semantic_projection)).hexdigest()


def _assert_differential_equivalence(
    contribution_regions: tuple[PlanarRegion, ...],
    domain_regions: tuple[PlanarRegion, ...],
) -> tuple[ArrangementUnionV1, ArrangementUnionV1]:
    reachability = _reachability(contribution_regions)
    broadphase = ExactSegmentArrangementBackend().exact_union(
        contribution_regions,
        domain_regions,
        reachability,
    )
    exhaustive = ExhaustiveExactArrangementBackend().exact_union(
        contribution_regions,
        domain_regions,
        reachability,
    )

    assert broadphase.vertices == exhaustive.vertices
    assert broadphase.edges == exhaustive.edges
    assert broadphase.loops == exhaustive.loops
    assert broadphase.regions == exhaustive.regions
    assert (
        broadphase.boundary_vertex_occurrences
        == exhaustive.boundary_vertex_occurrences
    )
    assert broadphase.point_contacts == exhaustive.point_contacts
    assert broadphase.exact_area_expression == exhaustive.exact_area_expression
    assert broadphase.intersection_count == exhaustive.intersection_count
    assert broadphase.atomic_edge_count == exhaustive.atomic_edge_count
    assert _semantic_digest(broadphase) == _semantic_digest(exhaustive)
    return broadphase, exhaustive


@pytest.fixture
def arrangement_cases():
    domain = (_region("domain", ((-20, -20), (20, -20), (20, 20), (-20, 20)), domain=True),)
    grid = tuple(
        _region(
            f"grid-{row}-{column}",
            (
                (-8 + column * 4, -8 + row * 4),
                (-4 + column * 4, -8 + row * 4),
                (-4 + column * 4, -4 + row * 4),
                (-8 + column * 4, -4 + row * 4),
            ),
        )
        for row in range(4)
        for column in range(4)
    )
    collinear = (
        _region("collinear-a", ((-8, -2), (3, -2), (3, 2), (-8, 2))),
        _region("collinear-b", ((-3, -2), (8, -2), (8, 2), (-3, 2))),
        _region("coincident-partial", ((-1, 2), (5, 2), (5, 6), (-1, 6))),
    )
    star = tuple(
        _region(
            f"star-{index}",
            (
                (0, 0),
                (
                    12 * sp.cos(sp.Rational(index, 6) * sp.pi),
                    12 * sp.sin(sp.Rational(index, 6) * sp.pi),
                ),
                (
                    12 * sp.cos(sp.Rational(index + 1, 6) * sp.pi),
                    12 * sp.sin(sp.Rational(index + 1, 6) * sp.pi),
                ),
            ),
        )
        for index in range(12)
    )
    sparse_many = tuple(
        _region(
            f"sparse-envelope-{row}-{column}",
            (
                (-12 + column * 3, -12 + row * 3),
                (-10 + column * 3, -12 + row * 3),
                (-10 + column * 3, -10 + row * 3),
                (-12 + column * 3, -10 + row * 3),
            ),
        )
        for row in range(7)
        for column in range(7)
    )
    return (
        pytest.param(grid, domain, id="grid"),
        pytest.param(collinear, domain, id="collinear-and-coincident-partial"),
        pytest.param(star, domain, id="high-intersection-star"),
        pytest.param(sparse_many, domain, id="sparse-domain-many-envelopes"),
    )


def test_broadphase_matches_exhaustive_on_exact_fixture_matrix(arrangement_cases):
    for case in arrangement_cases:
        contribution_regions, domain_regions = case.values
        broadphase, exhaustive = _assert_differential_equivalence(
            contribution_regions,
            domain_regions,
        )
        assert broadphase.input_segment_count == exhaustive.input_segment_count
        assert (
            broadphase.all_possible_pair_count
            == exhaustive.all_possible_pair_count
        )
        assert (
            broadphase.broadphase_candidate_pair_count
            <= exhaustive.broadphase_candidate_pair_count
        )
        assert broadphase.narrowphase_test_count <= exhaustive.narrowphase_test_count


def test_exact_aabb_contains_only_exact_scalar_expressions():
    region = _region(
        "algebraic-aabb",
        (
            (sp.sqrt(2), -sp.sqrt(3)),
            (-sp.sqrt(5), sp.sqrt(7)),
            (sp.sqrt(11), sp.sqrt(13)),
        ),
    )
    boundary = ExactSegmentArrangementBackend()._input_boundaries((region,), ())[0]

    assert isinstance(boundary.aabb, ExactSegmentAabbV1)
    assert all(
        isinstance(value, ExactScalar)
        for value in (
            boundary.aabb.min_x,
            boundary.aabb.max_x,
            boundary.aabb.min_y,
            boundary.aabb.max_y,
        )
    )
    assert sp.sympify(boundary.aabb.min_x.expression) == -sp.sqrt(5)
    assert sp.sympify(boundary.aabb.max_y.expression) == sp.sqrt(7)


def test_pair_tests_follow_semantic_segment_ids_not_input_iteration(monkeypatch):
    regions = (
        _region("z", ((0, 0), (4, 0), (4, 4), (0, 4))),
        _region("a", ((2, -1), (3, -1), (3, 5), (2, 5))),
    )
    tested_pairs = []
    exact_narrowphase = arrangement_module.segment_intersections

    def traced(left, right):
        tested_pairs.append(tuple(sorted((left.segment_id, right.segment_id))))
        return exact_narrowphase(left, right)

    monkeypatch.setattr(arrangement_module, "segment_intersections", traced)
    ExactSegmentArrangementBackend().build_arrangement(tuple(reversed(regions)), ())

    assert tested_pairs == sorted(tested_pairs)


def test_disjoint_2000_segment_fixture_has_nonquadratic_narrowphase():
    provenance = make_reference_provenance(
        support_ids=frozenset({"support:disjoint"}),
        patch_domain_ids=frozenset({"domain"}),
    )
    certificate = ConstructionCertificate(
        ConstructionKind.SOURCE_VERTEX,
        source_vertex_ids=frozenset({"vertex:disjoint"}),
    )
    segments = tuple(
        make_segment(
            f"segment:disjoint:{index:04d}",
            ExactPlanarPoint.from_values(index * 3, 0),
            ExactPlanarPoint.from_values(index * 3 + 1, 1),
            support_ids=provenance.support_ids,
            provenance=provenance,
            start_certificates=frozenset({certificate}),
            end_certificates=frozenset({certificate}),
        )
        for index in range(2000)
    )
    region = PlanarRegion(
        region_id="region:disjoint-2000",
        outer=PlanarLoop("loop:disjoint-2000", segments),
        contributor_instance_ids=frozenset({"instance:disjoint"}),
        contributor_spec_ids=frozenset({"spec:disjoint"}),
    )

    broadphase = ExactSegmentArrangementBackend().build_arrangement((region,), ())

    assert broadphase.counters.input_segments == 2000
    assert broadphase.counters.all_possible_pairs == 1_999_000
    assert broadphase.counters.broadphase_candidate_pairs == 0
    assert broadphase.counters.narrowphase_tests == 0
    assert broadphase.counters.actual_intersections == 0
    assert broadphase.counters.atomic_edges == 2000


def test_exhaustive_oracle_observes_all_pairs_on_disjoint_sample():
    provenance = make_reference_provenance(
        support_ids=frozenset({"support:oracle"}),
        patch_domain_ids=frozenset({"domain"}),
    )
    certificate = ConstructionCertificate(
        ConstructionKind.SOURCE_VERTEX,
        source_vertex_ids=frozenset({"vertex:oracle"}),
    )
    segments = tuple(
        make_segment(
            f"segment:oracle:{index:03d}",
            ExactPlanarPoint.from_values(index * 3, 0),
            ExactPlanarPoint.from_values(index * 3 + 1, 1),
            support_ids=provenance.support_ids,
            provenance=provenance,
            start_certificates=frozenset({certificate}),
            end_certificates=frozenset({certificate}),
        )
        for index in range(80)
    )
    region = PlanarRegion(
        region_id="region:oracle-sample",
        outer=PlanarLoop("loop:oracle-sample", segments),
    )

    broadphase = ExactSegmentArrangementBackend().build_arrangement((region,), ())
    exhaustive = ExhaustiveExactArrangementBackend().build_arrangement((region,), ())

    assert broadphase.atomic.keys() == exhaustive.atomic.keys()
    assert broadphase.counters.actual_intersections == 0
    assert exhaustive.counters.actual_intersections == 0
    assert broadphase.counters.narrowphase_tests == 0
    assert exhaustive.counters.narrowphase_tests == 3160
