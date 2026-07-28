from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
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
from cftuv.envelope_debug_session import (  # noqa: E402
    EnvelopeDebugSessionController,
)
from cftuv.envelope_topology_debug import (  # noqa: E402
    EnvelopeTopologyPairKind,
    EnvelopeTopologyPathKind,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    SELECTION_COMPLETED_DIAGNOSTIC_CODE,
    SELECTION_COMPLETION_COUNTERS,
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
    HOST_GRID_POLICY,
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)
from cftuv_envelope import (  # noqa: E402
    PlanarityAdmissionLawV1,
    compile_reference_envelopes,
)


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
    # Host analysis stores the unsigned turn between ordered boundary
    # tangents, not the interior reflex angle.
    turn_angle_by_k = {0: 45.0, 1: 90.0, 2: 135.0}
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


def test_the_declared_grid_policy_reaches_the_metric_certificate():
    """Политика хоста — не комментарий: она видна в сертификате метрики.

    Без этой проверки `HOST_GRID_POLICY` можно было бы переключить и не
    заметить, что экспорт его не читает; ровно так однажды «установленная»
    правка оказалась неустановленной.
    """

    from cftuv.surface_ir import HOST_GRID_POLICY

    snapshot = build_envelope_analysis_snapshot(_single_patch_bundle())
    descriptors = [
        item
        for item in snapshot.surface_metric_descriptors
        if hasattr(item, "grid_certificate")
    ]
    assert descriptors
    for descriptor in descriptors:
        assert descriptor.grid_certificate.snapping_law.value == (
            HOST_GRID_POLICY.value
        )
        # Обе границы окна названы — обе и записаны.
        assert descriptor.grid_certificate.window_lower_bound.numerator > 0
        assert descriptor.grid_certificate.window_upper_bound.numerator > 0


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
    cap_specs = [
        item
        for item in result.compilation.envelope_specs
        if type(item).__name__ == "CapEnvelopeSpec"
    ]
    assert len(angular_specs) == 1
    assert angular_specs[0].resolved_hidden_edge_count == hidden_count
    assert len(snapshot.corner_relations) == 1
    assert len(cap_specs) == 2
    assert all(
        angular_specs[0].envelope_spec_id
        in item.terminal_interface_spec_ids
        for item in result.compilation.envelope_specs
        if type(item).__name__ == "StripEnvelopeSpec"
    )


def test_partial_chain_selection_is_completed_to_the_whole_chain():
    """Частичное выделение цепочки больше не гасит билд, а достраивается.

    Полевой факт, которым это оплачено: на мешах с многорёберными швами
    (`wall_noise_top`, `sagging_wall`, `CFTUV_D_PERIODIC_CYLINDER`) попадание
    мышью в одно ребро шва валило ВЕСЬ прогон именованным отказом, и владелец
    видел warning вместо результата.
    """

    evaluation = evaluate_envelope_debug(
        _single_patch_bundle(combined_chain=True),
        frozenset({0}),
        1.0,
    )
    # Фикстура даёт ещё и `PLANAR_CHAIN_SUPPORT_NOT_LINEAR` (её цепочка гнётся
    # в вершине 1), и это утверждение — про отсутствие ОТКАЗА ВЫДЕЛЕНИЯ, а не
    # про пустой список диагностик.
    assert not {
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED,
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN,
    } & {item.outcome for item in evaluation.diagnostics}
    assert evaluation.debug_scene is not None
    assert evaluation.request is not None
    # Цепочка `[0, 1]` выделена целиком, хотя владелец указал только ребро 0.
    chain_edges = {
        _host_edge_number(edge_id)
        for chain in evaluation.snapshot.physical_chains
        for edge_id in chain.ordered_physical_edge_ids
        if chain.physical_chain_id
        in {
            item.physical_chain_id
            for item in evaluation.snapshot.chain_uses
            if item.chain_use_id in evaluation.request.selected_chain_use_ids
        }
    }
    assert chain_edges == {0, 1}


def test_selection_completion_is_counted_and_named_in_the_scene():
    profile = EnvelopeDebugProfileBuilderV1("v0-plane", "TOPOLOGY")

    scene = build_envelope_topology_debug_scene(
        _single_patch_bundle(combined_chain=True),
        frozenset({0}),
        profile=profile,
    )

    assert scene.selected_physical_edge_ids == (0, 1)
    codes = {item.code for item in scene.selection_diagnostics}
    assert SELECTION_COMPLETED_DIAGNOSTIC_CODE in codes
    completed = next(
        item
        for item in scene.selection_diagnostics
        if item.code == SELECTION_COMPLETED_DIAGNOSTIC_CODE
    )
    assert "[1]" in completed.message
    assert completed.physical_chain_id is not None
    counters = {
        item.name: item.value for item in profile.snapshot().counters
    }
    assert counters["SELECTION_COMPLETED_CHAINS"] == 1
    assert counters["SELECTION_COMPLETED_EDGES"] == 1


def test_whole_chain_selection_counts_zero_completion_rather_than_silence():
    """Нулевое дополнение — объявленный ноль, а не отсутствие измерения."""

    profile = EnvelopeDebugProfileBuilderV1("v0-plane", "TOPOLOGY")

    scene = build_envelope_topology_debug_scene(
        _single_patch_bundle(combined_chain=True),
        frozenset({0, 1}),
        profile=profile,
    )

    assert scene.selected_physical_edge_ids == (0, 1)
    assert all(
        item.code != SELECTION_COMPLETED_DIAGNOSTIC_CODE
        for item in scene.selection_diagnostics
    )
    counters = {
        item.name: item.value for item in profile.snapshot().counters
    }
    for name in SELECTION_COMPLETION_COUNTERS:
        assert counters[name] == 0


def test_edge_outside_every_physical_chain_is_refused_by_its_own_name():
    """Дополнение НЕВОЗМОЖНО ровно тогда, когда ребро не лежит ни в одной цепочке.

    Внутреннее ребро патча принадлежит поверхности, но не входит ни в одну
    граничную цепочку: достраивать его не до чего. Это и есть единственный
    случай, в котором жёсткий отказ на этом слое остаётся достижимым.
    """

    evaluation = evaluate_envelope_debug(
        _two_patch_seam_bundle(),
        frozenset({99}),
        1.0,
    )
    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN
    )

    bundle = _single_patch_bundle()
    interior = replace(
        bundle.patch_surface,
        edges=bundle.patch_surface.edges
        + (SourceEdge(11, (0, 2), (0,)),),
    )
    evaluation = evaluate_envelope_debug(
        replace(bundle, patch_surface=interior),
        frozenset({11}),
        1.0,
    )
    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN
    )


def test_kernel_request_still_refuses_a_hand_made_partial_chain():
    """Отказ V0 на слое запроса ядра остаётся достижимым и остаётся нужным.

    Хост дополняет выделение владельца, но `build_envelope_decal_request` —
    публичная точка входа: вызывающий подаёт номера рёбер напрямую (так делают
    инструменты и тесты), и там частичная цепочка обязана называться отказом, а
    не молча доезжать до ядра.
    """

    snapshot = build_envelope_analysis_snapshot(
        _single_patch_bundle(combined_chain=True)
    )
    with pytest.raises(Exception) as error:
        build_envelope_decal_request(snapshot, frozenset({0}), 1.0)
    assert error.value.outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED
    )


def test_exact_plane_derives_affine_metric_without_trusting_host_basis():
    evaluation = evaluate_envelope_debug(
        _single_patch_bundle(approximate_frame=True),
        frozenset({0}),
        1.0,
    )
    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    assert evaluation.debug_scene is not None
    metric = next(iter(evaluation.snapshot.surface_metric_descriptors))
    normal = metric.planarity_certificate.exact_plane_normal
    assert (normal.x.numerator, normal.y.numerator) == (0, 0)
    assert normal.z.numerator > 0


def test_affine_metric_uses_canonical_source_vertex_origin_exactly():
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
    metric = next(iter(evaluation.snapshot.surface_metric_descriptors))
    exact_origin = metric.exact_origin
    # Начало — вершина источника ТОЧНО, а не подгонка. «Точно» означает
    # позицию под ОБЪЯВЛЕННЫМ законом решётки, а не сырую binary64: ровно так
    # эту же сверку делает `validation._position_under_grid_law`
    # (`DECISIONS.md` за 2026-07-25). Узел пересчитывается здесь заново из
    # масштаба, который называет сертификат, поэтому проверяется ещё и то, что
    # метрика привязана к ТОМУ масштабу, который объявляет.
    from fractions import Fraction

    from cftuv_envelope.robust.grid import GridSpecV1, snap_value

    certificate = metric.grid_certificate
    raw = tuple(Fraction(*value.as_integer_ratio()) for value in positions[0])
    if certificate.snapping_law.snaps_source:
        grid = GridSpecV1(scale=certificate.source_scale)
        expected_origin = tuple(
            Fraction(snap_value(item, grid), certificate.source_scale)
            for item in raw
        )
    else:
        expected_origin = raw
    assert (
        Fraction(exact_origin.x.numerator, exact_origin.x.denominator),
        Fraction(exact_origin.y.numerator, exact_origin.y.denominator),
        Fraction(exact_origin.z.numerator, exact_origin.z.denominator),
    ) == expected_origin
    # Закон обязан быть тем, который объявил хост, иначе сверка выше
    # проверяет саму себя.
    assert certificate.snapping_law.value == HOST_GRID_POLICY.value


def test_exact_plane_with_irrational_unit_normal_uses_rational_affine_metric():
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

    assert evaluation.diagnostics == ()
    assert evaluation.debug_scene is not None
    metric = next(iter(evaluation.snapshot.surface_metric_descriptors))
    assert metric.planarity_certificate.exact


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


def _seam_bundle_with_off_plane_vertex(z: float):
    bundle = _two_patch_seam_bundle()
    root = 2.0**-0.5
    bundle.patch_graph.nodes[1].basis_u = Vector((root, root, 0.0))
    bundle.patch_graph.nodes[1].basis_v = Vector((-root, root, 0.0))
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            replace(vertex, position=(4.0, 2.0, z))
            if vertex.vertex_id == 5
            else vertex
            for vertex in bundle.patch_surface.vertices
        ),
    )
    return AnalysisBundle(
        bundle.source_revision,
        bundle.patch_graph,
        surface,
        bundle.capabilities,
    )


def test_selected_non_coplanar_patch_still_fails_exact_frame_admission():
    """Отклонение, ПЕРЕЖИВШЕЕ решётку, по-прежнему отвергается бюджетом.

    Отклонение здесь 1e-2, а не прежние 1e-6, и число сменилось не для
    удобства: с `HOST_GRID_POLICY = SOURCE_ONLY_GRID_SNAP_V1` вершины источника
    привязываются ДО проверки планарности, а выбранный на этом патче шаг —
    1/256, то есть половина ячейки 1.95e-03. Всё, что ближе к плоскости,
    привязка кладёт в неё точно, и отвергать становится нечего; это не обход
    проверки, а её механизм — соседний тест держит именно этот факт.
    1e-2 больше половины ячейки, поэтому переживает привязку и обязано быть
    отвергнутым бюджетом невязки, как и прежде.
    """

    evaluation = evaluate_envelope_debug(
        _seam_bundle_with_off_plane_vertex(0.01),
        frozenset({4}),
        0.25,
    )

    assert evaluation.debug_scene is None
    assert evaluation.diagnostics[0].outcome is (
        EnvelopeDebugHostOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
    )
    # Хост объявляет NEAR_PLANAR_PROJECTION_V1, поэтому отказ приходит от
    # бюджета невязки, а не от требования побитовой компланарности.
    assert "beyond the declared near-planar budget" in (
        evaluation.diagnostics[0].message
    )


def test_a_deviation_below_half_a_cell_is_absorbed_by_the_source_snap():
    """Новое следствие привязки источника, записанное как факт, а не как побочка.

    Вершина в 1 мкм от плоскости при половине ячейки 1.53e-05 ложится в
    плоскость ТОЧНО, и патч проходит `EXACT_SOURCE_PLANE_V1` — не «в пределах
    бюджета», а побитово. Это ровно тот механизм, ради которого решётка и
    вводилась (задуманное отношение восстанавливается структурно, а не
    допуском), но у него есть цена, которую видно только отсюда: порогом
    планарности на практике становится половина ячейки, а она на четыре
    порядка крупнее бюджета невязки. Прежде этот вход отвергался.

    Тест держит обе стороны: и что отклонение исчезло, и что исчезло оно
    ТОЧНО, а не было прощено.
    """

    evaluation = evaluate_envelope_debug(
        _seam_bundle_with_off_plane_vertex(0.000001),
        frozenset({4}),
        0.25,
    )

    assert evaluation.diagnostics == ()
    assert evaluation.snapshot is not None
    for descriptor in evaluation.snapshot.surface_metric_descriptors:
        certificate = descriptor.planarity_certificate
        assert certificate.admission_law is (
            PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
        )
        assert certificate.exact is True
        grid = descriptor.grid_certificate
        assert grid.snapping_law.snaps_source
        # Половина ячейки — та величина, которая отклонение и поглотила.
        assert Fraction(1, 2 * grid.source_scale) > Fraction(1, 10**6)


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
    """Отклонение 1e-2, а не прежнее 1e-3, по той же причине, что и выше.

    Выбранный на этом патче шаг — 1/256, половина ячейки 1.95e-03, поэтому
    прежний 1 мм привязка кладёт в плоскость точно и отвергать становится
    нечего: домен доходит до `RESOLVED`, и тест перестаёт проверять то, ради
    чего написан, — что отказ ОДНОГО домена не уносит топологию остальных.
    1e-2 переживает привязку, поэтому отказ остаётся настоящим.
    """

    bundle = _two_patch_seam_bundle()
    surface = replace(
        bundle.patch_surface,
        vertices=tuple(
            replace(vertex, position=(4.0, 2.0, 0.01))
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
    assert "beyond the declared near-planar budget" in receipts[1].message
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
        "DOMAIN_REGIONS",
        "DOMAIN_OUTER_LOOPS",
        "DOMAIN_HOLE_LOOPS",
        "DOMAIN_EXPLICIT_BARRIERS",
        "DOMAIN_SOURCE_FACE_CONTRIBUTORS",
        "DOMAIN_FACE_BOUNDARY_SEGMENTS_ORACLE",
        "DOMAIN_INPUT_SEGMENTS",
        "DOMAIN_SPARSE_SEGMENTS",
        "DOMAIN_BOUNDARY_SEGMENTS",
        "ARRANGEMENT_DOMAIN_SEGMENTS",
        "BOUNDARY_RESOLVER_BARRIER_SEGMENTS",
        "ENVELOPE_SPECS",
        "ARRANGEMENT_INPUT_SEGMENTS",
        "ARRANGEMENT_ALL_POSSIBLE_PAIRS",
        "ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS",
        "ARRANGEMENT_NARROWPHASE_TESTS",
        "ARRANGEMENT_PAIR_TESTS",
        "ARRANGEMENT_INTERSECTIONS",
        "ARRANGEMENT_ATOMIC_SEGMENTS",
        # Выход arrangement и обе стороны клипа. Без них по счётчикам не
        # отличить «пришло много» от «расплодилось на пересечениях».
        "ARRANGEMENT_OUTPUT_VERTICES",
        "ARRANGEMENT_OUTPUT_EDGES",
        "ARRANGEMENT_OUTPUT_LOOPS",
        "ARRANGEMENT_OUTPUT_REGIONS",
        "ENVELOPE_INSTANCES",
        "ENVELOPE_INSTANCE_SEGMENTS",
        "CLIP_SEGMENTS_IN",
        "CLIP_SEGMENTS_OUT",
        # INTERACTION — самая дорогая стадия в поле.
        "INTERACTION_COMPONENTS",
        "INTERACTION_ARRIVAL_MODELS",
        "INTERACTION_CANDIDATES",
        "INTERACTION_APPLICATIONS",
        "INTERACTION_EQUALITY_LOCI",
        "RESOLVED_REGIONS",
        "RESOLVED_EDGES",
    } <= counter_names
    assert {item.patch_domain_id for item in snapshot.counters} != {None}, (
        "счётчики обязаны быть привязаны к домену: без этого нельзя сравнить "
        "большой патч с мелким, ради чего они и заведены"
    )


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


def _profile_counter(profile, name, patch_domain_id=None):
    return next(
        item.value
        for item in profile.snapshot().counters
        if item.name == name and item.patch_domain_id == patch_domain_id
    )


def _bundle_with_revision(bundle, digest):
    revision = SourceRevision(bundle.source_revision.source_name, digest)
    graph = PatchGraph(source_revision=revision)
    for patch in bundle.patch_graph.nodes.values():
        graph.add_node(patch)
    for seam in bundle.patch_graph.edges.values():
        graph.add_edge(seam)
    return AnalysisBundle(
        revision,
        graph,
        replace(bundle.patch_surface, source_revision=revision),
        bundle.capabilities,
    )


def test_session_reuses_analysis_topology_metric_and_domain_for_alpha_changes():
    bundle = _single_patch_bundle()
    controller = EnvelopeDebugSessionController()
    build_calls = 0
    evaluations = []
    profiles = []

    def build():
        nonlocal build_calls
        build_calls += 1
        return bundle

    for alpha in (0.2, 0.3, 0.4):
        profile = EnvelopeDebugProfileBuilderV1(
            bundle.source_revision.source_name,
            "EXACT_REFERENCE",
        )
        cached_bundle = controller.get_analysis_bundle(
            "object",
            "mesh",
            bundle.source_revision,
            build,
            profile=profile,
        )
        evaluations.append(
            evaluate_envelope_debug_staged(
                cached_bundle,
                frozenset({0}),
                alpha,
                profile=profile,
                controller=controller,
                source_object_key="object",
                source_data_key="mesh",
            )
        )
        profiles.append(profile)

    assert build_calls == 1
    assert controller.build_counts == {
        "ANALYSIS_BUNDLE": 1,
        "TOPOLOGY_EXPORT": 1,
        "PATCH_METRIC": 1,
        "DOMAIN_GEOMETRY": 1,
        "COMPILED_ENVELOPE": 0,
        # Движок LEGACY подготовку очереди не строит ни разу — и ноль здесь
        # утверждение, а не умолчание.
        "CONVEYOR_PREPARATION": 0,
    }
    assert [
        str(item.domains[0].request.requested_alpha.value)
        for item in evaluations
    ] == ["0.2", "0.3", "0.4"]
    domain_id = evaluations[0].domains[0].patch_domain_id
    for profile in profiles[1:]:
        assert _profile_counter(
            profile,
            "ANALYSIS_BUNDLE_CACHE_HIT",
        ) == 1
        assert _profile_counter(
            profile,
            "TOPOLOGY_EXPORT_CACHE_HIT",
        ) == 1
        assert _profile_counter(
            profile,
            "PATCH_METRIC_CACHE_HIT",
            domain_id,
        ) == 1
        assert _profile_counter(
            profile,
            "DOMAIN_GEOMETRY_CACHE_HIT",
            domain_id,
        ) == 1
        assert _profile_counter(
            profile,
            "COMPILED_ENVELOPE_CACHE_BYPASS_ALPHA_DEPENDENT",
        ) == 1
        stage_names = {item.stage for item in profile.snapshot().timings}
        assert "HOST_CHAIN_COLLECTION" not in stage_names
        assert "SEAM_PARTITION_NORMALIZATION" not in stage_names
        assert "FRAME_ADMISSION" not in stage_names
        assert "ANGULAR_RELATIONS" not in stage_names
        assert "DOMAIN_GEOMETRY_EXPORT" not in stage_names


def test_selection_change_rebuilds_request_but_reuses_source_caches():
    bundle = _single_patch_bundle()
    controller = EnvelopeDebugSessionController()
    cached = controller.get_analysis_bundle(
        "object",
        "mesh",
        bundle.source_revision,
        lambda: bundle,
    )
    first = evaluate_envelope_debug_staged(
        cached,
        frozenset({0}),
        0.25,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
    )
    second = evaluate_envelope_debug_staged(
        cached,
        frozenset({1}),
        0.25,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
    )

    assert (
        first.domains[0].request.selected_chain_use_ids
        != second.domains[0].request.selected_chain_use_ids
    )
    assert controller.build_counts["ANALYSIS_BUNDLE"] == 1
    assert controller.build_counts["TOPOLOGY_EXPORT"] == 1
    assert controller.build_counts["PATCH_METRIC"] == 1
    assert controller.build_counts["DOMAIN_GEOMETRY"] == 1


def test_source_revision_and_data_replacement_invalidate_all_dependent_caches():
    first_bundle = _single_patch_bundle()
    second_bundle = _bundle_with_revision(
        first_bundle,
        "sha256:v0-plane-edited",
    )
    controller = EnvelopeDebugSessionController()

    for bundle, data_key in (
        (first_bundle, "mesh-a"),
        (second_bundle, "mesh-a"),
        (second_bundle, "mesh-b"),
    ):
        cached = controller.get_analysis_bundle(
            "object",
            data_key,
            bundle.source_revision,
            lambda bundle=bundle: bundle,
        )
        evaluate_envelope_debug_staged(
            cached,
            frozenset({0}),
            0.2,
            controller=controller,
            source_object_key="object",
            source_data_key=data_key,
        )

    assert controller.invalidation_count == 2
    assert controller.build_counts["ANALYSIS_BUNDLE"] == 3
    assert controller.build_counts["TOPOLOGY_EXPORT"] == 3
    assert controller.build_counts["PATCH_METRIC"] == 3
    assert controller.build_counts["DOMAIN_GEOMETRY"] == 3


def test_console_profile_shows_counters_for_each_domain(capsys):
    """Счётчики обязаны быть видны в консоли, а не только в sidecar JSON.

    Приёмка задачи — прогон в Blender: владелец читает консоль. Счётчик,
    который попал в JSON и не попал на экран, для этого прогона не существует.
    """

    from cftuv.envelope_debug_renderer import _print_profile

    profile = EnvelopeDebugProfileBuilderV1("v0-plane", "EXACT_REFERENCE")
    evaluate_envelope_debug_staged(
        _two_patch_seam_bundle(),
        frozenset({0, 5}),
        0.25,
        profile=profile,
    )
    snapshot = profile.snapshot()
    capsys.readouterr()
    _print_profile(snapshot)
    printed = capsys.readouterr().out

    domains = sorted(
        {
            item.patch_domain_id
            for item in snapshot.counters
            if item.patch_domain_id is not None
        }
    )
    assert len(domains) == 2
    for domain in domains:
        # Домен печатается хвостом: у всех общий префикс, и обрезка слева
        # давала одинаковые строки. Живой след стадий печатает так же.
        assert domain[-24:] in printed
    for name in (
        "ARRANGEMENT_INPUT_SEGMENTS",
        "ARRANGEMENT_OUTPUT_REGIONS",
        "CLIP_SEGMENTS_IN",
        "INTERACTION_CANDIDATES",
    ):
        assert name in printed
    # Сводка масштабирования — то, ради чего счётчики и заведены.
    assert "Scaling" in printed


def test_non_fatal_kernel_diagnostics_reach_the_host_evaluation(monkeypatch):
    """INFO-диагностика ядра доходит до хоста на УСПЕШНОМ домене.

    Раньше `diagnostics.extend(...)` стоял только в ветках отказа, поэтому
    именованное событие на разрешившемся домене исчезало молча — печать в
    рендерере его бы всё равно не увидела.

    Проверяется тракт, а не геометрия: фикстуры INFO-диагностик не порождают, и
    доказывать проброс полевой случайностью нельзя. Поэтому исход подменяется
    здесь явно, а тест падает ровно тогда, когда проброс исчезнет.
    """

    import cftuv_envelope as kernel
    from dataclasses import replace as _replace

    from cftuv.envelope_request_export import EnvelopeDebugHostSeverity

    original = kernel.resolve_coverage_interactions
    marker = kernel.InteractionDiagnosticV1(
        kernel.InteractionOutcome.MULTIWAY_MEET_RESOLVED_AS_ONE_EVENT,
        kernel.InteractionDiagnosticSeverity.INFO,
        "probe: multiway meet resolved as one event",
    )

    def with_marker(*args, **kwargs):
        result = original(*args, **kwargs)
        if result.resolved_coverage is None:
            return result
        return _replace(result, diagnostics=(*result.diagnostics, marker))

    monkeypatch.setattr(kernel, "resolve_coverage_interactions", with_marker)

    evaluation = evaluate_envelope_debug_staged(
        _single_patch_bundle(),
        frozenset({0}),
        0.25,
    )

    outcomes = {
        item.outcome.value if hasattr(item.outcome, "value") else str(item.outcome)
        for item in evaluation.diagnostics
    }
    assert "MULTIWAY_MEET_RESOLVED_AS_ONE_EVENT" in outcomes, (
        "ненадзорная диагностика ядра не дошла до хоста на успешном домене"
    )
    # Severity обязана сохраниться: иначе INFO неотличим от отказа в выводе.
    assert any(
        item.severity is not EnvelopeDebugHostSeverity.ERROR
        for item in evaluation.diagnostics
    )
