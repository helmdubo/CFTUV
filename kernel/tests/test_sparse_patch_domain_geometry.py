from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

import pytest
import sympy as sp

from cftuv_envelope import (
    BoundaryConstraintId,
    BoundaryConstraintKind,
    BoundaryConstraintV1,
    PhysicalEdgeSequenceConstraintTargetV1,
    ReferenceOutcome,
    SurfaceTriangleV1,
    SurfaceTriangleId,
)
from cftuv_envelope.reference import (
    build_sparse_patch_domain_geometry,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.domain_oracle import (
    build_source_face_union_oracle,
    compare_sparse_domain_to_face_union,
)

from reference_factories import physical_seam_snapshot, straight_snapshot


SQUARE = (
    ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
)
CONCAVE = (
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
ONE_HOLE = (
    ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
    ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
    ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
    ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
)


def _several_holes_faces():
    xs = (0.0, 2.0, 4.0, 6.0, 8.0, 10.0)
    ys = (0.0, 2.0, 4.0, 6.0)
    holes = {(1, 1), (3, 1)}
    return tuple(
        (
            (xs[x], ys[y]),
            (xs[x + 1], ys[y]),
            (xs[x + 1], ys[y + 1]),
            (xs[x], ys[y + 1]),
        )
        for y in range(len(ys) - 1)
        for x in range(len(xs) - 1)
        if (x, y) not in holes
    )


def _context(snapshot, request, domain_id=None):
    compiled = compile_reference_envelopes(snapshot, request, domain_id)
    assert compiled.outcome is ReferenceOutcome.EXACT, compiled.diagnostics
    frame = next(
        item
        for item in snapshot.surface_metric_descriptors
        if item.patch_domain_id == compiled.compilation.plan_key.patch_domain_id
    )
    return compiled, GeometryContext.build(compiled.compilation, frame)


@pytest.mark.parametrize(
    ("faces", "expected_holes"),
    (
        (SQUARE, 0),
        (CONCAVE, 0),
        (ONE_HOLE, 1),
        (_several_holes_faces(), 2),
    ),
)
def test_sparse_domain_matches_independent_face_union_oracle(
    faces, expected_holes
):
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=(
            {"name": "source", "points": (faces[0][0], faces[0][1])},
        ),
    )
    _, context = _context(snapshot, request)

    sparse = build_sparse_patch_domain_geometry(context)
    report = compare_sparse_domain_to_face_union(context, sparse)

    assert report.exact_equivalent
    assert len(sparse.outer_loops) == 1
    assert len(sparse.hole_loops) == expected_holes
    assert len(sparse.domain_regions) == 1
    assert report.sparse_domain_segment_count <= report.face_boundary_segment_count
    assert sparse.source_face_contributor_ids
    assert all(
        not provenance.source_face_ids for provenance in sparse.boundary_provenance
    )
    assert all(
        provenance.physical_edge_ids
        and provenance.physical_chain_ids
        and provenance.boundary_loop_ids
        and provenance.chain_use_ids
        and provenance.patch_domain_ids
        and provenance.boundary_constraint_ids
        and provenance.lineage_ids
        for provenance in sparse.boundary_provenance
    )


def test_disconnected_patch_domain_is_rejected_instead_of_becoming_two_regions():
    faces = (
        ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)),
        ((5.0, 0.0), (7.0, 0.0), (7.0, 2.0), (5.0, 2.0)),
    )
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (2.0, 0.0))},
        ),
    )
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT

    evaluated = evaluate_reference_raw_coverage(
        compiled.compilation, Decimal("1")
    )

    assert evaluated.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert evaluated.raw_coverage is None
    assert "exactly one connected OUTER" in evaluated.diagnostics[0].message


def test_explicit_internal_barrier_is_sparse_and_not_a_face_boundary():
    filled = _several_holes_faces() + (
        ((2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)),
    )
    snapshot, request = straight_snapshot(
        faces=filled,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (2.0, 0.0))},
        ),
    )
    frame = next(iter(snapshot.surface_metric_descriptors))
    coordinates = {
        item.source_vertex_id: (
            item.domain_coordinate.x,
            item.domain_coordinate.y,
        )
        for item in frame.source_vertex_coordinates
    }
    barrier_edge = next(
        edge.edge_id
        for edge in snapshot.surface_ir.source_edges
        if {
            coordinates[edge.vertex_a_id],
            coordinates[edge.vertex_b_id],
        }
        == {(2.0, 2.0), (4.0, 2.0)}
    )
    domain = next(iter(snapshot.patch_domains))
    barrier_id = BoundaryConstraintId("r2a-explicit-barrier")
    barrier = BoundaryConstraintV1(
        barrier_id,
        domain.patch_domain_id,
        BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
        PhysicalEdgeSequenceConstraintTargetV1((barrier_edge,), False),
        None,
        False,
        True,
    )
    snapshot = replace(
        snapshot,
        patch_domains=frozenset(
            {
                replace(
                    domain,
                    boundary_constraint_ids=domain.boundary_constraint_ids
                    | {barrier_id},
                )
            }
        ),
        boundary_constraints=snapshot.boundary_constraints | {barrier},
    )
    _, context = _context(snapshot, request)

    sparse = build_sparse_patch_domain_geometry(context)
    report = compare_sparse_domain_to_face_union(context, sparse)

    assert report.exact_equivalent
    assert len(sparse.explicit_barriers) == 1
    assert sparse.explicit_barriers[0].segment.provenance.physical_edge_ids == {
        barrier_edge.value
    }
    assert sparse.sparse_segment_count == sparse.boundary_segment_count + 1


def test_seam_self_uses_remain_distinct_without_becoming_runtime_face_edges():
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
    )
    _, context = _context(snapshot, request)
    sparse = build_sparse_patch_domain_geometry(context)
    selected_uses = tuple(
        item
        for item in snapshot.chain_uses
        if item.chain_use_id in request.selected_chain_use_ids
    )
    self_chain_ids = {item.physical_chain_id for item in selected_uses}
    self_edge_ids = frozenset(
        edge.value
        for chain in snapshot.physical_chains
        if chain.physical_chain_id in self_chain_ids
        for edge in chain.ordered_physical_edge_ids
    )

    assert len(selected_uses) == 2
    assert len(self_chain_ids) == 1
    assert self_edge_ids.isdisjoint(
        frozenset(
            edge_id
            for provenance in sparse.boundary_provenance
            for edge_id in provenance.physical_edge_ids
        )
    )


def test_common_refined_seam_sides_keep_one_physical_lineage_per_domain():
    snapshot, request, domain_ids = physical_seam_snapshot()
    selected_uses = tuple(
        item
        for item in snapshot.chain_uses
        if item.chain_use_id in request.selected_chain_use_ids
    )
    assert len({item.physical_chain_id for item in selected_uses}) == 1

    reports = []
    for domain_id in domain_ids:
        _, context = _context(snapshot, request, domain_id)
        sparse = build_sparse_patch_domain_geometry(context)
        reports.append(compare_sparse_domain_to_face_union(context, sparse))

    assert all(item.exact_equivalent for item in reports)
    assert all(item.sparse_domain_segment_count == 5 for item in reports)


def test_retriangulation_does_not_change_sparse_boundary_or_oracle_coverage():
    snapshot, request = straight_snapshot(
        faces=SQUARE,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
    )
    _, context = _context(snapshot, request)
    baseline = build_sparse_patch_domain_geometry(context)
    face = next(iter(snapshot.surface_ir.source_faces))
    v0, v1, v2, v3 = face.vertex_cycle
    e0, e1, e2, e3 = face.edge_cycle
    alternate = frozenset(
        {
            SurfaceTriangleV1(
                SurfaceTriangleId("r2a-alternate-a"),
                face.face_id,
                (v0, v1, v3),
                (e0, None, e3),
                face.polygon_normal,
            ),
            SurfaceTriangleV1(
                SurfaceTriangleId("r2a-alternate-b"),
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
    changed_surface = replace(
        snapshot.surface_ir,
        source_faces=frozenset({changed_face}),
        surface_triangles=alternate,
    )
    changed_snapshot = replace(snapshot, surface_ir=changed_surface)
    _, changed_context = _context(changed_snapshot, request)
    changed = build_sparse_patch_domain_geometry(changed_context)

    assert baseline.outer_loops == changed.outer_loops
    assert baseline.hole_loops == changed.hole_loops
    assert compare_sparse_domain_to_face_union(
        changed_context, changed
    ).exact_equivalent
    assert face.face_id in changed.source_face_contributor_ids


def test_rigid_translation_and_scale_preserve_oracle_topology_and_scale_area():
    base_snapshot, base_request = straight_snapshot(
        faces=ONE_HOLE,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (4.0, 0.0))},
        ),
    )
    transform = lambda point: (3 * point[0] + 7, 3 * point[1] - 11)
    changed_faces = tuple(
        tuple(transform(point) for point in face) for face in ONE_HOLE
    )
    changed_snapshot, changed_request = straight_snapshot(
        faces=changed_faces,
        source_routes=(
            {
                "name": "source",
                "points": tuple(
                    transform(point) for point in ((0.0, 0.0), (4.0, 0.0))
                ),
            },
        ),
        revision_name="r2a-scaled",
    )
    _, base_context = _context(base_snapshot, base_request)
    _, changed_context = _context(changed_snapshot, changed_request)
    base_sparse = build_sparse_patch_domain_geometry(base_context)
    changed_sparse = build_sparse_patch_domain_geometry(changed_context)
    base_oracle = build_source_face_union_oracle(base_context)
    changed_oracle = build_source_face_union_oracle(changed_context)

    assert compare_sparse_domain_to_face_union(
        base_context, base_sparse
    ).exact_equivalent
    assert compare_sparse_domain_to_face_union(
        changed_context, changed_sparse
    ).exact_equivalent
    assert len(base_sparse.hole_loops) == len(changed_sparse.hole_loops) == 1
    assert sp.sympify(changed_oracle.exact_area_expression) == 9 * sp.sympify(
        base_oracle.exact_area_expression
    )
