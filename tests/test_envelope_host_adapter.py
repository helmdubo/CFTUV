from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

from mathutils import Vector
import pytest


KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_host_adapter import (  # noqa: E402
    EnvelopeDebugHostOutcome,
    _canonical_chain,
    _host_edge_number,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
    build_envelope_topology_debug_scene,
    evaluate_envelope_debug,
    evaluate_envelope_debug_staged,
)
from cftuv.envelope_debug_profile import (  # noqa: E402
    EnvelopeDebugProfileBuilderV1,
    EnvelopeDomainStage,
)
from cftuv.envelope_topology_debug import (  # noqa: E402
    EnvelopeTopologyPairKind,
    EnvelopeTopologyPathKind,
)
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryCorner,
    BoundaryLoop,
    CornerKind,
    LoopKind,
    PatchGraph,
    PatchNode,
    PatchType,
    WorldFacing,
)
from cftuv.surface_ir import (  # noqa: E402
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)
from cftuv_envelope import compile_reference_envelopes  # noqa: E402


def _single_patch_bundle(*, approximate_frame: bool = False, combined_chain: bool = False):
    revision = SourceRevision("v0-plane", "sha256:v0-plane")
    graph = PatchGraph(source_revision=revision)
    root = 2.0**-0.5
    basis_u = Vector((root, root, 0.0)) if approximate_frame else Vector((1.0, 0.0, 0.0))
    basis_v = Vector((-root, root, 0.0)) if approximate_frame else Vector((0.0, 1.0, 0.0))
    chains = (
        [
            BoundaryChain(
                vert_indices=[0, 1, 2],
                vert_cos=[Vector((0, 0, 0)), Vector((4, 0, 0)), Vector((4, 3, 0))],
                edge_indices=[0, 1],
                side_face_indices=[0, 0],
                side_face_normals=[Vector((0, 0, 1)), Vector((0, 0, 1))],
            ),
            BoundaryChain(
                vert_indices=[2, 3],
                vert_cos=[Vector((4, 3, 0)), Vector((0, 3, 0))],
                edge_indices=[2],
                side_face_indices=[0],
                side_face_normals=[Vector((0, 0, 1))],
            ),
            BoundaryChain(
                vert_indices=[3, 0],
                vert_cos=[Vector((0, 3, 0)), Vector((0, 0, 0))],
                edge_indices=[3],
                side_face_indices=[0],
                side_face_normals=[Vector((0, 0, 1))],
            ),
        ]
        if combined_chain
        else [
            BoundaryChain(
                vert_indices=[index, (index + 1) % 4],
                vert_cos=[
                    Vector(((0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0))[index]),
                    Vector(((0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0))[(index + 1) % 4]),
                ],
                edge_indices=[index],
                side_face_indices=[0],
                side_face_normals=[Vector((0, 0, 1))],
            )
            for index in range(4)
        ]
    )
    loop = BoundaryLoop(
        vert_indices=[0, 1, 2, 3],
        vert_cos=[
            Vector((0, 0, 0)),
            Vector((4, 0, 0)),
            Vector((4, 3, 0)),
            Vector((0, 3, 0)),
        ],
        edge_indices=[0, 1, 2, 3],
        side_face_indices=[0, 0, 0, 0],
        kind=LoopKind.OUTER,
        chains=chains,
    )
    graph.add_node(
        PatchNode(
            patch_id=0,
            face_indices=[0],
            centroid=Vector((2, 1.5, 0)),
            normal=Vector((0, 0, 1)),
            basis_u=basis_u,
            basis_v=basis_v,
            patch_type=PatchType.FLOOR,
            world_facing=WorldFacing.UP,
            boundary_loops=[loop],
        )
    )
    surface = PatchSurfaceIR(
        revision,
        vertices=(
            SourceVertex(0, (0.0, 0.0, 0.0)),
            SourceVertex(1, (4.0, 0.0, 0.0)),
            SourceVertex(2, (4.0, 3.0, 0.0)),
            SourceVertex(3, (0.0, 3.0, 0.0)),
        ),
        edges=(
            SourceEdge(0, (0, 1), (0,)),
            SourceEdge(1, (1, 2), (0,)),
            SourceEdge(2, (2, 3), (0,)),
            SourceEdge(3, (3, 0), (0,)),
        ),
        faces=(
            SourceFace(
                0,
                0,
                (0, 1, 2, 3),
                (0, 1, 2, 3),
                (0.0, 0.0, 1.0),
                (0, 1),
            ),
        ),
        triangles=(
            SurfaceTriangle(
                0,
                0,
                (0, 1, 2),
                (1, None, 0),
                (0.0, 0.0, 1.0),
            ),
            SurfaceTriangle(
                1,
                0,
                (0, 2, 3),
                (2, 3, None),
                (0.0, 0.0, 1.0),
            ),
        ),
    )
    return AnalysisBundle(revision, graph, surface)


def _two_patch_seam_bundle():
    revision = SourceRevision("v0-seam", "sha256:v0-seam")
    graph = PatchGraph(source_revision=revision)
    coordinates = {
        0: (0.0, 0.0, 0.0),
        1: (2.0, 0.0, 0.0),
        2: (2.0, 2.0, 0.0),
        3: (0.0, 2.0, 0.0),
        4: (4.0, 0.0, 0.0),
        5: (4.0, 2.0, 0.0),
    }

    def chain(start, end, edge_id, neighbor=-1):
        return BoundaryChain(
            vert_indices=[start, end],
            vert_cos=[Vector(coordinates[start]), Vector(coordinates[end])],
            edge_indices=[edge_id],
            side_face_indices=[],
            side_face_normals=[Vector((0, 0, 1))],
            neighbor_patch_id=neighbor,
        )

    loops = (
        BoundaryLoop(
            vert_indices=[0, 1, 2, 3],
            vert_cos=[Vector(coordinates[item]) for item in (0, 1, 2, 3)],
            edge_indices=[0, 1, 2, 3],
            side_face_indices=[0, 0, 0, 0],
            kind=LoopKind.OUTER,
            chains=[
                chain(0, 1, 0),
                chain(1, 2, 1, 1),
                chain(2, 3, 2),
                chain(3, 0, 3),
            ],
        ),
        BoundaryLoop(
            vert_indices=[1, 4, 5, 2],
            vert_cos=[Vector(coordinates[item]) for item in (1, 4, 5, 2)],
            edge_indices=[4, 5, 6, 1],
            side_face_indices=[1, 1, 1, 1],
            kind=LoopKind.OUTER,
            chains=[
                chain(1, 4, 4),
                chain(4, 5, 5),
                chain(5, 2, 6),
                chain(2, 1, 1, 0),
            ],
        ),
    )
    for patch_id, loop in enumerate(loops):
        graph.add_node(
            PatchNode(
                patch_id=patch_id,
                face_indices=[patch_id],
                normal=Vector((0, 0, 1)),
                basis_u=Vector((1, 0, 0)),
                basis_v=Vector((0, 1, 0)),
                patch_type=PatchType.FLOOR,
                world_facing=WorldFacing.UP,
                boundary_loops=[loop],
            )
        )
    surface = PatchSurfaceIR(
        revision,
        vertices=tuple(
            SourceVertex(vertex_id, coordinates[vertex_id])
            for vertex_id in sorted(coordinates)
        ),
        edges=(
            SourceEdge(0, (0, 1), (0,)),
            SourceEdge(1, (1, 2), (0, 1)),
            SourceEdge(2, (2, 3), (0,)),
            SourceEdge(3, (3, 0), (0,)),
            SourceEdge(4, (1, 4), (1,)),
            SourceEdge(5, (4, 5), (1,)),
            SourceEdge(6, (5, 2), (1,)),
        ),
        faces=(
            SourceFace(0, 0, (0, 1, 2, 3), (0, 1, 2, 3), (0, 0, 1), (0, 1)),
            SourceFace(1, 1, (1, 4, 5, 2), (4, 5, 6, 1), (0, 0, 1), (2, 3)),
        ),
        triangles=(
            SurfaceTriangle(0, 0, (0, 1, 2), (1, None, 0), (0, 0, 1)),
            SurfaceTriangle(1, 0, (0, 2, 3), (2, 3, None), (0, 0, 1)),
            SurfaceTriangle(2, 1, (1, 4, 5), (5, None, 4), (0, 0, 1)),
            SurfaceTriangle(3, 1, (1, 5, 2), (6, 1, None), (0, 0, 1)),
        ),
    )
    return AnalysisBundle(revision, graph, surface)


def _mismatched_seam_partition_bundle():
    revision = SourceRevision(
        "v0-seam-partition",
        "sha256:v0-seam-partition",
    )
    graph = PatchGraph(source_revision=revision)
    coordinates = {
        0: (0.0, 0.0, 0.0),
        1: (2.0, 0.0, 0.0),
        2: (2.0, 1.0, 0.0),
        3: (2.0, 2.0, 0.0),
        4: (0.0, 2.0, 0.0),
        5: (4.0, 0.0, 0.0),
        6: (4.0, 2.0, 0.0),
    }

    def chain(vertices, edges, neighbor=-1):
        return BoundaryChain(
            vert_indices=list(vertices),
            vert_cos=[Vector(coordinates[item]) for item in vertices],
            edge_indices=list(edges),
            side_face_indices=[],
            side_face_normals=[Vector((0, 0, 1)) for _ in edges],
            neighbor_patch_id=neighbor,
        )

    left_loop = BoundaryLoop(
        vert_indices=[0, 1, 2, 3, 4],
        vert_cos=[Vector(coordinates[item]) for item in (0, 1, 2, 3, 4)],
        edge_indices=[0, 1, 2, 3, 4],
        side_face_indices=[0, 0, 0, 0, 0],
        kind=LoopKind.OUTER,
        chains=[
            chain((0, 1), (0,)),
            chain((1, 2), (1,), 1),
            chain((2, 3), (2,), 1),
            chain((3, 4), (3,)),
            chain((4, 0), (4,)),
        ],
    )
    right_loop = BoundaryLoop(
        vert_indices=[1, 5, 6, 3, 2],
        vert_cos=[Vector(coordinates[item]) for item in (1, 5, 6, 3, 2)],
        edge_indices=[5, 6, 7, 2, 1],
        side_face_indices=[1, 1, 1, 1, 1],
        kind=LoopKind.OUTER,
        chains=[
            chain((1, 5), (5,)),
            chain((5, 6), (6,)),
            chain((6, 3), (7,)),
            chain((3, 2, 1), (2, 1), 0),
        ],
    )
    for patch_id, loop in enumerate((left_loop, right_loop)):
        graph.add_node(
            PatchNode(
                patch_id=patch_id,
                face_indices=[patch_id],
                normal=Vector((0, 0, 1)),
                basis_u=Vector((1, 0, 0)),
                basis_v=Vector((0, 1, 0)),
                patch_type=PatchType.FLOOR,
                world_facing=WorldFacing.UP,
                boundary_loops=[loop],
            )
        )
    surface = PatchSurfaceIR(
        revision,
        vertices=tuple(
            SourceVertex(vertex_id, coordinates[vertex_id])
            for vertex_id in sorted(coordinates)
        ),
        edges=(
            SourceEdge(0, (0, 1), (0,)),
            SourceEdge(1, (1, 2), (0, 1)),
            SourceEdge(2, (2, 3), (0, 1)),
            SourceEdge(3, (3, 4), (0,)),
            SourceEdge(4, (4, 0), (0,)),
            SourceEdge(5, (1, 5), (1,)),
            SourceEdge(6, (5, 6), (1,)),
            SourceEdge(7, (6, 3), (1,)),
        ),
        faces=(
            SourceFace(
                0,
                0,
                (0, 1, 2, 3, 4),
                (0, 1, 2, 3, 4),
                (0, 0, 1),
                (0, 1, 2),
            ),
            SourceFace(
                1,
                1,
                (1, 5, 6, 3, 2),
                (5, 6, 7, 2, 1),
                (0, 0, 1),
                (3, 4, 5),
            ),
        ),
        triangles=(
            SurfaceTriangle(0, 0, (0, 1, 2), (1, None, 0), (0, 0, 1)),
            SurfaceTriangle(1, 0, (0, 2, 3), (2, None, None), (0, 0, 1)),
            SurfaceTriangle(2, 0, (0, 3, 4), (3, 4, None), (0, 0, 1)),
            SurfaceTriangle(3, 1, (1, 5, 6), (6, None, 5), (0, 0, 1)),
            SurfaceTriangle(4, 1, (1, 6, 3), (7, None, None), (0, 0, 1)),
            SurfaceTriangle(5, 1, (1, 3, 2), (2, 1, None), (0, 0, 1)),
        ),
    )
    return AnalysisBundle(revision, graph, surface)


def _self_seam_bundle():
    bundle = _single_patch_bundle()
    loop = bundle.patch_graph.nodes[0].boundary_loops[0]
    first_use = loop.chains[0]
    first_use.neighbor_patch_id = -2
    loop.chains.append(
        BoundaryChain(
            vert_indices=[1, 0],
            vert_cos=[Vector((4, 0, 0)), Vector((0, 0, 0))],
            edge_indices=[0],
            side_face_indices=[0],
            side_face_normals=[Vector((0, 0, 1))],
            neighbor_patch_id=-2,
        )
    )
    return bundle


def _reflex_bundle(hidden_count: int):
    coordinates_by_k = {
        0: (
            (0.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (4.0, 2.0, 0.0),
            (3.0, 2.0, 0.0),
            (2.0, 2.0, 0.0),
            (1.0, 3.0, 0.0),
            (0.0, 3.0, 0.0),
        ),
        1: (
            (0.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (4.0, 2.0, 0.0),
            (3.0, 2.0, 0.0),
            (2.0, 2.0, 0.0),
            (2.0, 4.0, 0.0),
            (0.0, 4.0, 0.0),
        ),
        2: (
            (0.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (4.0, 2.0, 0.0),
            (3.0, 2.0, 0.0),
            (2.0, 2.0, 0.0),
            (3.0, 3.0, 0.0),
            (3.0, 4.0, 0.0),
            (0.0, 4.0, 0.0),
        ),
    }
    turn_angle_by_k = {0: 225.0, 1: 270.0, 2: 315.0}
    coordinates = coordinates_by_k[hidden_count]
    revision = SourceRevision(
        f"v0-angular-k{hidden_count}",
        f"sha256:v0-angular-k{hidden_count}",
    )
    vertex_count = len(coordinates)
    chains = [
        BoundaryChain(
            vert_indices=[index, (index + 1) % vertex_count],
            vert_cos=[
                Vector(coordinates[index]),
                Vector(coordinates[(index + 1) % vertex_count]),
            ],
            edge_indices=[index],
            side_face_indices=[0],
            side_face_normals=[Vector((0, 0, 1))],
        )
        for index in range(vertex_count)
    ]
    loop = BoundaryLoop(
        vert_indices=list(range(vertex_count)),
        vert_cos=[Vector(item) for item in coordinates],
        edge_indices=list(range(vertex_count)),
        side_face_indices=[0] * vertex_count,
        kind=LoopKind.OUTER,
        chains=chains,
        corners=[
            BoundaryCorner(
                loop_vert_index=4,
                vert_index=4,
                vert_co=Vector(coordinates[4]),
                prev_chain_index=3,
                next_chain_index=4,
                corner_kind=CornerKind.GEOMETRIC,
                turn_angle_deg=turn_angle_by_k[hidden_count],
            )
        ],
    )
    graph = PatchGraph(source_revision=revision)
    graph.add_node(
        PatchNode(
            patch_id=0,
            face_indices=[0],
            normal=Vector((0, 0, 1)),
            basis_u=Vector((1, 0, 0)),
            basis_v=Vector((0, 1, 0)),
            patch_type=PatchType.FLOOR,
            world_facing=WorldFacing.UP,
            boundary_loops=[loop],
        )
    )
    triangle_count = vertex_count - 2
    triangles = []
    for ordinal in range(1, vertex_count - 1):
        triangles.append(
            SurfaceTriangle(
                ordinal - 1,
                0,
                (0, ordinal, ordinal + 1),
                (
                    ordinal,
                    vertex_count - 1 if ordinal + 1 == vertex_count - 1 else None,
                    0 if ordinal == 1 else None,
                ),
                (0.0, 0.0, 1.0),
            )
        )
    surface = PatchSurfaceIR(
        revision,
        vertices=tuple(
            SourceVertex(index, coordinate)
            for index, coordinate in enumerate(coordinates)
        ),
        edges=tuple(
            SourceEdge(
                index,
                (index, (index + 1) % vertex_count),
                (0,),
            )
            for index in range(vertex_count)
        ),
        faces=(
            SourceFace(
                0,
                0,
                tuple(range(vertex_count)),
                tuple(range(vertex_count)),
                (0.0, 0.0, 1.0),
                tuple(range(triangle_count)),
            ),
        ),
        triangles=tuple(triangles),
    )
    return AnalysisBundle(revision, graph, surface)


def test_closed_physical_chain_canonicalization_is_rotation_and_reversal_invariant():
    variants = (
        BoundaryChain(
            vert_indices=[0, 1, 2, 3],
            edge_indices=[10, 11, 12, 13],
            is_closed=True,
        ),
        BoundaryChain(
            vert_indices=[2, 3, 0, 1],
            edge_indices=[12, 13, 10, 11],
            is_closed=True,
        ),
        BoundaryChain(
            vert_indices=[0, 3, 2, 1],
            edge_indices=[13, 12, 11, 10],
            is_closed=True,
        ),
    )
    canonical = [_canonical_chain(item)[:2] for item in variants]
    assert canonical[0] == canonical[1] == canonical[2]


def test_real_analysis_bundle_runs_public_static_pipeline():
    bundle = _single_patch_bundle()
    snapshot = build_envelope_analysis_snapshot(bundle)
    request = build_envelope_decal_request(snapshot, frozenset({0}), 1.0)
    assert request.selected_chain_use_ids

    evaluation = evaluate_envelope_debug(bundle, frozenset({0}), 1.0)
    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    assert evaluation.request is not None
    assert len(evaluation.compilations) == 1
    assert len(evaluation.raw_results) == 1
    assert len(evaluation.interaction_results) == 1
    assert evaluation.debug_scene is not None
    assert evaluation.debug_scene.raw_coverage_digests
    assert evaluation.debug_scene.resolved_coverage_digests
    assert all(
        not item.self_contact_pair_declarations
        for item in evaluation.compilations
    )


@pytest.mark.parametrize("hidden_count", (0, 1, 2))
def test_real_analysis_bundle_compiles_exact_k0_k1_k2_angular(hidden_count):
    snapshot = build_envelope_analysis_snapshot(_reflex_bundle(hidden_count))
    request = build_envelope_decal_request(
        snapshot,
        frozenset({3, 4}),
        0.25,
    )
    result = compile_reference_envelopes(snapshot, request)
    assert result.compilation is not None
    angular_specs = [
        item
        for item in result.compilation.envelope_specs
        if type(item).__name__ == "AngularEnvelopeSpec"
    ]
    assert len(angular_specs) == 1
    assert angular_specs[0].resolved_hidden_edge_count == hidden_count


def test_partial_chain_selection_fails_named_without_expansion():
    evaluation = evaluate_envelope_debug(
        _single_patch_bundle(combined_chain=True),
        frozenset({0}),
        1.0,
    )
    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED
    )


def test_exact_plane_derives_frame_without_trusting_approximate_host_basis():
    evaluation = evaluate_envelope_debug(
        _single_patch_bundle(approximate_frame=True),
        frozenset({0}),
        1.0,
    )
    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    assert evaluation.debug_scene is not None
    frame = next(iter(evaluation.snapshot.surface_metric_descriptors))
    assert (frame.normal.x, frame.normal.y, frame.normal.z) == (0.0, 0.0, 1.0)


def test_axis_frame_uses_canonical_plane_origin_without_float_subtraction():
    bundle = _single_patch_bundle()
    positions = (
        (0.12345678901234566, 0.23456789012345677, 0.0),
        (0.9876543210987654, 0.23456789012345677, 0.0),
        (0.9876543210987654, 1.8765432109876542, 0.0),
        (0.12345678901234566, 1.8765432109876542, 0.0),
    )
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            SourceVertex(vertex_id, position)
            for vertex_id, position in enumerate(positions)
        ),
    )
    offset_bundle = AnalysisBundle(
        bundle.source_revision,
        bundle.patch_graph,
        surface,
        bundle.capabilities,
    )

    evaluation = evaluate_envelope_debug(
        offset_bundle,
        frozenset({0}),
        0.25,
    )

    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    frame = next(iter(evaluation.snapshot.surface_metric_descriptors))
    assert (frame.origin.x, frame.origin.y, frame.origin.z) == (0.0, 0.0, 0.0)


def test_exact_plane_with_irrational_unit_normal_fails_named():
    bundle = _single_patch_bundle()
    root = 2.0**-0.5
    bundle.patch_graph.nodes[0].normal = Vector((0.0, -root, root))
    bundle.patch_graph.nodes[0].basis_u = Vector((1.0, 0.0, 0.0))
    bundle.patch_graph.nodes[0].basis_v = Vector((0.0, root, root))
    positions = (
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 1.0, 1.0),
        (0.0, 1.0, 1.0),
    )
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            SourceVertex(vertex_id, position)
            for vertex_id, position in enumerate(positions)
        ),
    )
    bundle = AnalysisBundle(
        bundle.source_revision,
        bundle.patch_graph,
        surface,
        bundle.capabilities,
    )

    evaluation = evaluate_envelope_debug(
        bundle,
        frozenset({0}),
        0.25,
    )

    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE
    )
    assert "no rational unit normal" in evaluation.diagnostics[0].message


def test_one_physical_seam_maps_to_two_domains_without_cross_patch_collision():
    evaluation = evaluate_envelope_debug(
        _two_patch_seam_bundle(),
        frozenset({1}),
        0.5,
    )
    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    seam_chains = [
        item
        for item in evaluation.snapshot.physical_chains
        if item.kind.value == "PHYSICAL_SEAM"
    ]
    assert len(seam_chains) == 1
    seam_uses = [
        item
        for item in evaluation.snapshot.chain_uses
        if item.physical_chain_id == seam_chains[0].physical_chain_id
    ]
    assert len(seam_uses) == 2
    assert len({item.patch_domain_id for item in seam_uses}) == 2
    assert len(evaluation.compilations) == 2
    assert len(evaluation.raw_results) == 2
    assert evaluation.debug_scene is not None
    assert len(evaluation.debug_scene.patch_domain_ids) == 2
    assert len(evaluation.debug_scene.raw_coverage_digests) == 2
    assert len(evaluation.debug_scene.resolved_coverage_digests) == 2


def test_patch_side_seam_partitions_use_exact_common_refinement():
    evaluation = evaluate_envelope_debug(
        _mismatched_seam_partition_bundle(),
        frozenset({1}),
        0.25,
    )

    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    selected_edges = {
        _host_edge_number(edge_id)
        for chain in evaluation.snapshot.physical_chains
        if any(
            _host_edge_number(edge_id) == 1
            for edge_id in chain.ordered_physical_edge_ids
        )
        for edge_id in chain.ordered_physical_edge_ids
    }
    assert selected_edges == {1}
    selected_chain = next(
        chain
        for chain in evaluation.snapshot.physical_chains
        if tuple(
            _host_edge_number(edge_id)
            for edge_id in chain.ordered_physical_edge_ids
        )
        == (1,)
    )
    selected_uses = [
        use
        for use in evaluation.snapshot.chain_uses
        if use.physical_chain_id == selected_chain.physical_chain_id
    ]
    assert len(selected_uses) == 2
    assert len({use.patch_domain_id for use in selected_uses}) == 2


def test_patch_side_seam_refinement_rejects_different_edge_coverage():
    bundle = _mismatched_seam_partition_bundle()
    coarse_use = bundle.patch_graph.nodes[1].boundary_loops[0].chains[-1]
    coarse_use.vert_indices = [3, 2]
    coarse_use.edge_indices = [2]

    evaluation = evaluate_envelope_debug(
        bundle,
        frozenset({1}),
        0.25,
    )

    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE
    )
    assert "cover different physical edges" in evaluation.diagnostics[0].message


def test_unselected_patch_domain_is_not_projected_into_debug_scene():
    evaluation = evaluate_envelope_debug(
        _two_patch_seam_bundle(),
        frozenset({0}),
        0.25,
    )
    assert evaluation.diagnostics == ()
    assert evaluation.debug_scene is not None
    assert len(evaluation.debug_scene.patch_domain_ids) == 1
    assert {
        item.patch_domain_id
        for item in (
            *evaluation.debug_scene.paths,
            *evaluation.debug_scene.loops,
            *evaluation.debug_scene.regions,
        )
    } == set(evaluation.debug_scene.patch_domain_ids)


def test_unselected_nonexact_patch_does_not_block_selected_exact_domain():
    bundle = _two_patch_seam_bundle()
    root = 2.0**-0.5
    bundle.patch_graph.nodes[1].basis_u = Vector((root, root, 0.0))
    bundle.patch_graph.nodes[1].basis_v = Vector((-root, root, 0.0))

    evaluation = evaluate_envelope_debug(
        bundle,
        frozenset({0}),
        0.25,
    )

    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    assert len(evaluation.snapshot.patch_domains) == 1
    assert evaluation.debug_scene is not None
    assert len(evaluation.debug_scene.patch_domain_ids) == 1


def test_selected_non_coplanar_patch_still_fails_exact_frame_admission():
    bundle = _two_patch_seam_bundle()
    root = 2.0**-0.5
    bundle.patch_graph.nodes[1].basis_u = Vector((root, root, 0.0))
    bundle.patch_graph.nodes[1].basis_v = Vector((-root, root, 0.0))
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            replace(vertex, position=(4.0, 2.0, 0.000001))
            if vertex.vertex_id == 5
            else vertex
            for vertex in bundle.patch_surface.vertices
        ),
    )
    bundle = AnalysisBundle(
        bundle.source_revision,
        bundle.patch_graph,
        surface,
        bundle.capabilities,
    )

    evaluation = evaluate_envelope_debug(
        bundle,
        frozenset({4}),
        0.25,
    )

    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE
    )
    assert "not exactly coplanar" in evaluation.diagnostics[0].message


def test_seam_self_maps_to_two_uses_in_one_domain_without_self_contact_guessing():
    snapshot = build_envelope_analysis_snapshot(_self_seam_bundle())
    self_chains = [
        item
        for item in snapshot.physical_chains
        if item.kind.value == "SEAM_SELF"
    ]
    assert len(self_chains) == 1
    self_uses = [
        item
        for item in snapshot.chain_uses
        if item.physical_chain_id == self_chains[0].physical_chain_id
    ]
    assert len(self_uses) == 2
    assert len({item.patch_domain_id for item in self_uses}) == 1

    request = build_envelope_decal_request(snapshot, frozenset({0}), 0.25)
    result = compile_reference_envelopes(snapshot, request)
    assert result.compilation is not None
    assert result.compilation.self_contact_pair_declarations == frozenset()


def test_topology_scene_uses_host_facts_without_loading_exact_kernel(monkeypatch):
    def reject_kernel_load():
        raise AssertionError("topology debug must not load kernel or SymPy")

    monkeypatch.setattr(
        "cftuv.envelope_host_adapter._load_kernel",
        reject_kernel_load,
    )
    profile = EnvelopeDebugProfileBuilderV1("v0-seam", "TOPOLOGY")

    scene = build_envelope_topology_debug_scene(
        _two_patch_seam_bundle(),
        frozenset({1}),
        profile=profile,
    )

    assert len(scene.patch_domain_ids) == 2
    assert {
        item.kind for item in scene.paths
    } >= {
        EnvelopeTopologyPathKind.PATCH_OUTER_LOOP,
        EnvelopeTopologyPathKind.PHYSICAL_CHAIN,
        EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE,
        EnvelopeTopologyPathKind.SELECTED_SOURCE,
    }
    selected_sources = [
        item
        for item in scene.paths
        if item.kind is EnvelopeTopologyPathKind.SELECTED_SOURCE
    ]
    assert len(selected_sources) == 1
    assert selected_sources[0].host_edge_ids == (1,)
    patch_pairs = [
        item
        for item in scene.pairs
        if item.kind is EnvelopeTopologyPairKind.PATCH
        and item.host_edge_ids == (1,)
    ]
    assert len(patch_pairs) == 1
    assert len(patch_pairs[0].chain_use_ids) == 2
    assert {
        item.stage for item in profile.snapshot().receipts
    } == {EnvelopeDomainStage.TOPOLOGY_READY}


def test_staged_exact_keeps_topology_when_one_domain_rejects_metric():
    bundle = _two_patch_seam_bundle()
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            replace(vertex, position=(4.0, 2.0, 0.001))
            if vertex.vertex_id == 5
            else vertex
            for vertex in bundle.patch_surface.vertices
        ),
    )
    bundle = AnalysisBundle(
        bundle.source_revision,
        bundle.patch_graph,
        surface,
        bundle.capabilities,
    )
    profile = EnvelopeDebugProfileBuilderV1(
        "v0-seam",
        "EXACT_REFERENCE",
    )

    evaluation = evaluate_envelope_debug_staged(
        bundle,
        frozenset({1}),
        0.25,
        profile=profile,
    )

    assert len(evaluation.topology_scene.patch_domain_ids) == 2
    receipts = {item.patch_id: item for item in evaluation.receipts}
    assert receipts[0].stage in {
        EnvelopeDomainStage.INTERACTION_REJECTED,
        EnvelopeDomainStage.RESOLVED,
    }
    assert receipts[1].stage is EnvelopeDomainStage.METRIC_REJECTED
    assert "not exactly coplanar" in receipts[1].message
    assert any(
        scene.patch_domain_ids
        for scene in evaluation.exact_debug_scenes
    )
    summary = profile.snapshot().stage_summary()
    assert summary["topology"] == 2
    assert summary["metric"] == 1
    assert summary["raw"] == 1
    assert summary["resolved"] in {0, 1}


def test_exact_profile_exposes_named_stage_timings_and_counters():
    profile = EnvelopeDebugProfileBuilderV1(
        "v0-plane",
        "EXACT_REFERENCE",
    )
    evaluation = evaluate_envelope_debug_staged(
        _single_patch_bundle(),
        frozenset({0}),
        0.25,
        profile=profile,
    )
    assert evaluation.topology_scene is not None

    snapshot = profile.snapshot()
    stages = {item.stage for item in snapshot.timings}
    assert {
        "SELECTION_SCOPE",
        "HOST_CHAIN_COLLECTION",
        "SEAM_PARTITION_NORMALIZATION",
        "BUNDLE_SLICE",
        "TOPOLOGY_SCENE",
        "SNAPSHOT_EXPORT",
        "FRAME_ADMISSION",
        "ANGULAR_RELATIONS",
        "SNAPSHOT_VALIDATION",
        "COMPILE",
        "DOMAIN_BUILD",
        "ENVELOPE_INSTANCE_BUILD",
        "DOMAIN_CLIP",
        "RAW_UNION",
        "INTERACTION",
        "DEBUG_SCENE",
    } <= stages
    counter_names = {item.name for item in snapshot.counters}
    assert {
        "MESH_FACES",
        "MESH_EDGES",
        "PATCH_COUNT",
        "SELECTED_CHAINS",
        "SELECTED_DOMAINS",
        "FACES_PER_DOMAIN",
        "PHYSICAL_EDGES_PER_DOMAIN",
        "DOMAIN_FACE_REGIONS",
        "DOMAIN_BOUNDARY_SEGMENTS",
        "ENVELOPE_SPECS",
        "ARRANGEMENT_INPUT_SEGMENTS",
        "ARRANGEMENT_PAIR_TESTS",
        "ARRANGEMENT_INTERSECTIONS",
        "ARRANGEMENT_ATOMIC_SEGMENTS",
    } <= counter_names


def test_staged_multi_domain_selection_slices_edges_but_keeps_request_identity():
    evaluation = evaluate_envelope_debug_staged(
        _two_patch_seam_bundle(),
        frozenset({0, 5}),
        0.25,
    )

    assert len(evaluation.domains) == 2
    assert all(item.request is not None for item in evaluation.domains)
    request_ids = {
        item.request.decal_request_id
        for item in evaluation.domains
    }
    assert len(request_ids) == 1
    assert all(
        len(item.request.selected_chain_use_ids) == 1
        for item in evaluation.domains
    )
