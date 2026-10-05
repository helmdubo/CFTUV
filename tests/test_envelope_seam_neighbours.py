"""Грани соседа шва в снапшоте домена: вторая сторона цепи для закона `CHAIN_STATION_PLAN_V1`.

Снапшот запроса несёт поверхность ТОЛЬКО патчей запроса, а общую цепь шва читают оба домена; решение по её внутренней вершине обязано
видеть поверхность обеих сторон, иначе домены решат по-разному и шов получит вершину с одной стороны. Хост кладёт в снапшот грани
патча-соседа у внутренних вершин цепей шва (`seam_neighbour_faces`), а ядро берёт чужую сторону оттуда. Здесь: две ленты по два
четырёхгранника с общим швом из трёх вершин (внутренняя — средняя), домен каждой ленты видит чужую.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import sympy  # noqa: F401  (экспортёр снапшота требует sympy 1.14)

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from mathutils import Vector  # noqa: E402

from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_host_adapter import build_envelope_analysis_snapshot  # noqa: E402
from cftuv.envelope_metric_export import build_envelope_patch_metric_export  # noqa: E402
from cftuv.envelope_topology_export import EnvelopeTopologyExportV1, build_envelope_topology_export  # noqa: E402
from cftuv.model import BoundaryChain, BoundaryLoop, LoopKind, PatchGraph, PatchNode, PatchType, WorldFacing  # noqa: E402
from cftuv.surface_ir import (  # noqa: E402
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)
from cftuv_envelope import _chain_station  # noqa: E402
from cftuv_envelope.canonical import canonical_json_bytes  # noqa: E402
from cftuv_envelope.contracts.chain_station import ChainStationDispositionV1  # noqa: E402

# вершины: шов `c0 c1 c2` (0, 1, 2), сторона A `a0 a1 a2` (3, 4, 5) при y = -2, сторона B `b0 b1 b2` (6, 7, 8) при y = +2
SEAM = (0, 1, 2)


def _positions(lift=0.0):
    found = {}
    for column in range(3):
        found[column] = (2.0 * column, 0.0, 0.0)
        found[3 + column] = (2.0 * column, -2.0, 0.0)
        found[6 + column] = (2.0 * column, 2.0, lift if column == 1 else 0.0)
    return found


def two_strips(lift_b=0.0):
    """Две ленты по два четырёхгранника (патчи 0 и 1); шов `c0 c1 c2` общий, средняя вершина — внутренняя."""

    revision = SourceRevision("v0-two-strips", "sha256:v0-two-strips")
    graph = PatchGraph(source_revision=revision)
    pos = _positions(lift_b)

    def chain(vertices, edges, neighbour=-1):
        return BoundaryChain(
            vert_indices=list(vertices),
            vert_cos=[Vector(pos[v]) for v in vertices],
            edge_indices=list(edges),
            side_face_indices=[],
            side_face_normals=[Vector((0, 0, 1))] * len(edges),
            neighbor_patch_id=neighbour,
        )

    def loop(vertices, edges, chains, owner):
        return BoundaryLoop(
            vert_indices=list(vertices),
            vert_cos=[Vector(pos[v]) for v in vertices],
            edge_indices=list(edges),
            side_face_indices=[owner] * len(edges),
            kind=LoopKind.OUTER,
            chains=chains,
        )

    loop_a = loop(
        (3, 4, 5, 2, 1, 0),
        (2, 3, 6, 1, 0, 4),
        [chain((3, 4, 5), (2, 3)), chain((5, 2), (6,)), chain((2, 1, 0), (1, 0), 1), chain((0, 3), (4,))],
        0,
    )
    loop_b = loop(
        (0, 1, 2, 8, 7, 6),
        (0, 1, 11, 8, 7, 9),
        [chain((0, 1, 2), (0, 1), 0), chain((2, 8), (11,)), chain((8, 7, 6), (8, 7)), chain((6, 0), (9,))],
        1,
    )
    for patch_id, boundary, faces in ((0, loop_a, [0, 1]), (1, loop_b, [2, 3])):
        graph.add_node(
            PatchNode(
                patch_id=patch_id,
                face_indices=faces,
                normal=Vector((0, 0, 1)),
                basis_u=Vector((1, 0, 0)),
                basis_v=Vector((0, 1, 0)),
                patch_type=PatchType.FLOOR,
                world_facing=WorldFacing.UP,
                boundary_loops=[boundary],
            )
        )
    edges = (
        SourceEdge(0, (0, 1), (0, 2)),
        SourceEdge(1, (1, 2), (1, 3)),
        SourceEdge(2, (3, 4), (0,)),
        SourceEdge(3, (4, 5), (1,)),
        SourceEdge(4, (3, 0), (0,)),
        SourceEdge(5, (4, 1), (0, 1)),
        SourceEdge(6, (5, 2), (1,)),
        SourceEdge(7, (6, 7), (2,)),
        SourceEdge(8, (7, 8), (3,)),
        SourceEdge(9, (6, 0), (2,)),
        SourceEdge(10, (7, 1), (2, 3)),
        SourceEdge(11, (8, 2), (3,)),
    )
    faces = (
        SourceFace(0, 0, (3, 4, 1, 0), (2, 5, 0, 4), (0, 0, 1), (0, 1)),
        SourceFace(1, 0, (4, 5, 2, 1), (3, 6, 1, 5), (0, 0, 1), (2, 3)),
        SourceFace(2, 1, (0, 1, 7, 6), (0, 10, 7, 9), (0, 0, 1), (4, 5)),
        SourceFace(3, 1, (1, 2, 8, 7), (1, 11, 8, 10), (0, 0, 1), (6, 7)),
    )
    triangles = (
        SurfaceTriangle(0, 0, (3, 4, 1), (5, None, 2), (0, 0, 1)),
        SurfaceTriangle(1, 0, (3, 1, 0), (0, 4, None), (0, 0, 1)),
        SurfaceTriangle(2, 1, (4, 5, 2), (6, None, 3), (0, 0, 1)),
        SurfaceTriangle(3, 1, (4, 2, 1), (1, 5, None), (0, 0, 1)),
        SurfaceTriangle(4, 2, (0, 1, 7), (10, None, 0), (0, 0, 1)),
        SurfaceTriangle(5, 2, (0, 7, 6), (7, 9, None), (0, 0, 1)),
        SurfaceTriangle(6, 3, (1, 2, 8), (11, None, 1), (0, 0, 1)),
        SurfaceTriangle(7, 3, (1, 8, 7), (8, 10, None), (0, 0, 1)),
    )
    surface = PatchSurfaceIR(
        revision,
        vertices=tuple(SourceVertex(vertex_id, pos[vertex_id]) for vertex_id in sorted(pos)),
        edges=edges,
        faces=faces,
        triangles=triangles,
    )
    return AnalysisBundle(revision, graph, surface)


def _snapshot(bundle, patches, topology=None):
    return build_envelope_analysis_snapshot(
        bundle,
        included_patch_ids=frozenset(patches),
        topology_export=topology if topology is not None else build_envelope_topology_export(bundle),
    )


def _numbers(items):
    return {int(item.value.rsplit(":", 1)[1]) for item in items}


def test_a_domain_snapshot_carries_the_neighbour_faces_at_the_interior_vertex_of_a_seam_chain():
    bundle = two_strips()

    first, second = _snapshot(bundle, {0}), _snapshot(bundle, {1})

    # внутренняя вершина шва — `1`: грани патча-соседа, которые её касаются (обе)
    assert _numbers(item.face_id for item in first.seam_neighbour_faces) == {2, 3}
    assert {item.patch_id.value.rsplit(":", 1)[1] for item in first.seam_neighbour_faces} == {"1"}
    assert _numbers(item.face_id for item in second.seam_neighbour_faces) == {0, 1}
    positions = {item.vertex_ids[0].value.rsplit(":", 1)[1]: item.positions[0] for item in first.seam_neighbour_faces}
    assert positions["0"].x == 0.0 and positions["0"].y == 0.0
    # домен, у которого обе стороны в срезе, чужих граней не несёт
    assert _snapshot(bundle, {0, 1}).seam_neighbour_faces == frozenset()


def test_both_domains_of_the_shared_seam_chain_read_one_station_decision_from_their_snapshots():
    for lift in (0.0, 0.5):
        bundle = two_strips(lift_b=lift)
        first, second = _snapshot(bundle, {0}), _snapshot(bundle, {1})
        plans = [
            _chain_station.chain_station_plans(snapshot, next(iter(snapshot.patch_domains)).patch_domain_id)
            for snapshot in (first, second)
        ]
        seam = [
            [item for item in plan if len(item.stations) == 1 and item.stations[0].source_vertex_id.value.endswith(":1")]
            for plan in plans
        ]

        assert [len(items) for items in seam] == [1, 1]
        assert seam[0] == seam[1]
        (record,) = seam[0]
        free = record.stations[0].disposition is ChainStationDispositionV1.FREE
        # B поднята у шва: вершина `7` (над внутренней вершиной шва) выше — складка поперёк, у обоих доменов решение одно
        assert free == (lift == 0.0)


def test_the_worker_export_of_a_domain_is_bit_equal_to_the_parent_snapshot_with_the_neighbour_faces():
    bundle = two_strips()
    topology = build_envelope_topology_export(bundle)
    parent = _snapshot(bundle, {0}, topology)
    export = build_host_export_input(topology, 0, alpha=0.3, request_id="request", density=1)
    worker_topology = EnvelopeTopologyExportV1(
        export.source_revision_value,
        export.bundle,
        export.host_chains,
        {0: topology.patch_domain_id_by_patch[0]},
        export.developable_stretch_budget,
    )
    worker = build_envelope_patch_metric_export(worker_topology, 0).snapshot

    assert parent.seam_neighbour_faces
    assert canonical_json_bytes(worker) == canonical_json_bytes(parent)
    # лёгкий вход несёт только то, что снапшот прочтёт: грани, касающиеся внутренней вершины шва
    assert {item.face_id for item in export.bundle.patch_surface.neighbour_faces} == {2, 3}


def test_the_view_ring_names_the_faces_outside_the_request_that_touch_its_vertices():
    bundle = two_strips()

    ring = build_envelope_topology_export(bundle).analysis_bundle.patch_surface
    view = build_host_export_input(build_envelope_topology_export(bundle), 0, alpha=0.3, request_id="r", density=1).bundle.patch_surface

    assert ring is not None
    assert {item.face_id for item in view.neighbour_faces} == {2, 3}
    assert all(item.patch_id == 1 and len(item.vertex_cycle) == len(item.positions) == 4 for item in view.neighbour_faces)
