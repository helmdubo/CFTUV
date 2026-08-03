"""Host `AnalysisBundle` фикстуры для закона разреза физической цепочки.

Три источника фактов, ни один из которых не требует Blender:

* `planar_quad_bundle` — конструируемый патч с одной трёхвершинной цепочкой;
  смещение средней вершины задаёт величину и род излома;
* `u_route_bundle` — ПОЛЕВОЙ якорь `walls.001` (маршрут 1→8→9→10→11→0,
  четыре подлинных излома по 45°), координаты читаются из слепка владельца,
  а не переписываются здесь руками;
* `bundle_from_exported_snapshot` — обратное отображение выпущенного
  `AnalysisSnapshotV1` в факты хоста. Корпусные фикстуры несут ЗАМОРОЖЕННЫЕ
  байты снапшота и пересобираются только из Blender, поэтому иначе закон
  экспортёра на них не проверить вовсе.

Про обратное отображение честно: снапшот, срезанный под один домен, НЕ несёт
номера противоположного патча шва (его сторона в срез не попала). Род соседа
(`PATCH`) — факт снапшота и восстанавливается, номер — нет; на его место
встаёт заведомо отсутствующий в графе номер. Он ни на что не влияет: пара
сторон всё равно не собирается, и разбиение по объявленной противоположной
стороне для такого шва не запускается ни до, ни после восстановления.
"""

from __future__ import annotations

import json
from pathlib import Path

from mathutils import Vector

import cftuv_envelope as kernel
from cftuv.model import (
    BoundaryChain,
    BoundaryCorner,
    BoundaryLoop,
    LoopKind,
    PatchGraph,
    PatchNode,
    PatchType,
    WorldFacing,
)
from cftuv.surface_ir import (
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_ROOT = REPO_ROOT / "kernel" / "fixtures"
SEM_CLB_CASE_ROOT = FIXTURE_ROOT / "sem_clb_02_lost_domains_v1" / "cases"
FIELD_ANCHOR = (
    REPO_ROOT
    / "artifacts"
    / "field_snapshots"
    / "walls_001_u_route_snapshot.json"
)


def host_exported_snapshot_paths() -> tuple[Path, ...]:
    """Все фикстуры корпуса, чьи байты выпустил ИМЕННО этот экспортёр."""

    paths = [
        case / "analysis_snapshot.json"
        for case in sorted(SEM_CLB_CASE_ROOT.iterdir())
    ]
    paths += [
        FIXTURE_ROOT / name / "analysis_snapshot.json"
        for name in (
            "building_002_full_selection_v1",
            "building_002_point_contact_v1",
            "building_002_weighted_normals_v1",
            "building_patch10_density4_v1",
        )
    ]
    return tuple(path for path in paths if path.exists())


# --------------------------------------------------------------------------
# Конструируемый планарный патч
# --------------------------------------------------------------------------


def planar_quad_bundle(midpoint: tuple[float, float, float] = (2.0, 0.0, 0.0)):
    """Квадрат, нижняя сторона которого — одна цепочка из двух рёбер.

    `midpoint` лежит между `(0, 0, 0)` и `(4, 0, 0)`: ровно на отрезке — цепочка
    прямая; смещённый — цепочка изломана, и величина смещения задаёт величину
    излома. Смещение по нормали патча даёт излом, который проекция чарта
    вырождает.
    """

    revision = SourceRevision("v0-quad", "sha256:v0-quad")
    coordinates = {
        0: (0.0, 0.0, 0.0),
        4: tuple(float(value) for value in midpoint),
        1: (4.0, 0.0, 0.0),
        2: (4.0, 3.0, 0.0),
        3: (0.0, 3.0, 0.0),
    }
    graph = PatchGraph(source_revision=revision)

    def chain(vertices, edges):
        return BoundaryChain(
            vert_indices=list(vertices),
            vert_cos=[Vector(coordinates[item]) for item in vertices],
            edge_indices=list(edges),
            side_face_indices=[0 for _ in edges],
            side_face_normals=[Vector((0, 0, 1)) for _ in edges],
        )

    chains = [
        chain((0, 4, 1), (0, 4)),
        chain((1, 2), (1,)),
        chain((2, 3), (2,)),
        chain((3, 0), (3,)),
    ]
    loop = BoundaryLoop(
        vert_indices=[0, 4, 1, 2, 3],
        vert_cos=[Vector(coordinates[item]) for item in (0, 4, 1, 2, 3)],
        edge_indices=[0, 4, 1, 2, 3],
        side_face_indices=[0, 0, 0, 0, 0],
        kind=LoopKind.OUTER,
        chains=chains,
        corners=[
            BoundaryCorner(
                vert_index=item.vert_indices[-1],
                vert_co=Vector(coordinates[item.vert_indices[-1]]),
                prev_chain_index=index,
                next_chain_index=(index + 1) % len(chains),
            )
            for index, item in enumerate(chains)
        ],
    )
    graph.add_node(
        PatchNode(
            patch_id=0,
            face_indices=[0],
            centroid=Vector((2, 1.5, 0)),
            normal=Vector((0, 0, 1)),
            basis_u=Vector((1.0, 0.0, 0.0)),
            basis_v=Vector((0.0, 1.0, 0.0)),
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
            SourceEdge(0, (0, 4), (0,)),
            SourceEdge(1, (1, 2), (0,)),
            SourceEdge(2, (2, 3), (0,)),
            SourceEdge(3, (3, 0), (0,)),
            SourceEdge(4, (4, 1), (0,)),
        ),
        faces=(
            SourceFace(
                0,
                0,
                (0, 4, 1, 2, 3),
                (0, 4, 1, 2, 3),
                (0.0, 0.0, 1.0),
                (0, 1, 2),
            ),
        ),
        triangles=(
            SurfaceTriangle(0, 0, (0, 4, 1), (4, None, 0), (0.0, 0.0, 1.0)),
            SurfaceTriangle(1, 0, (0, 1, 2), (1, None, None), (0.0, 0.0, 1.0)),
            SurfaceTriangle(2, 0, (0, 2, 3), (2, 3, None), (0.0, 0.0, 1.0)),
        ),
    )
    return AnalysisBundle(revision, graph, surface)


# --------------------------------------------------------------------------
# Полевой якорь walls.001
# --------------------------------------------------------------------------

U_ROUTE_NGON = (5, 4, 3, 2, 1, 8, 9, 10, 11, 0)
U_ROUTE = (1, 8, 9, 10, 11, 0)


def u_route_positions() -> dict[int, tuple[float, float, float]]:
    payload = json.loads(FIELD_ANCHOR.read_text(encoding="utf-8"))
    return {
        index: tuple(float(value) for value in position)
        for index, position in enumerate(payload["raw"]["vertices"])
    }


def u_route_bundle(*, split_route: bool = False):
    """Стена `walls.001` целиком: изломанный U-маршрут объявлен ОДНОЙ цепочкой.

    Это и есть полевой дефект: у хоста нет представления излома внутри
    `BoundaryChain`, поэтому маршрут с четырьмя подлинными поворотами по 45°
    едет в ядро как одна прямая цепь. `split_route=True` даёт ту же стену с
    маршрутом, заранее разобранным на рёбра, — контроль того, что закон
    экспортёра приходит к тому же набору цепочек.
    """

    position = u_route_positions()
    revision = SourceRevision("walls.001", "sha256:walls-001-u-route")
    graph = PatchGraph(source_revision=revision)
    edge_of = {
        (U_ROUTE_NGON[index], U_ROUTE_NGON[(index + 1) % len(U_ROUTE_NGON)]): index
        for index in range(len(U_ROUTE_NGON))
    }

    def chain(vertices):
        return BoundaryChain(
            vert_indices=list(vertices),
            vert_cos=[Vector(position[item]) for item in vertices],
            edge_indices=[
                edge_of[(a, b)] for a, b in zip(vertices, vertices[1:])
            ],
            side_face_indices=[0 for _ in vertices[1:]],
            side_face_normals=[Vector((1.0, 0.0, 0.0)) for _ in vertices[1:]],
        )

    route = (
        [chain(U_ROUTE[index : index + 2]) for index in range(len(U_ROUTE) - 1)]
        if split_route
        else [chain(U_ROUTE)]
    )
    chains = [
        chain((5, 4)),
        chain((4, 3)),
        chain((3, 2)),
        chain((2, 1)),
        *route,
        chain((0, 5)),
    ]
    loop = BoundaryLoop(
        vert_indices=list(U_ROUTE_NGON),
        vert_cos=[Vector(position[item]) for item in U_ROUTE_NGON],
        edge_indices=list(range(len(U_ROUTE_NGON))),
        side_face_indices=[0] * len(U_ROUTE_NGON),
        kind=LoopKind.OUTER,
        chains=chains,
        corners=[
            BoundaryCorner(
                vert_index=item.vert_indices[-1],
                vert_co=Vector(position[item.vert_indices[-1]]),
                prev_chain_index=index,
                next_chain_index=(index + 1) % len(chains),
            )
            for index, item in enumerate(chains)
        ],
    )
    graph.add_node(
        PatchNode(
            patch_id=0,
            face_indices=[0],
            normal=Vector((1.0, 0.0, 0.0)),
            basis_u=Vector((0.0, 1.0, 0.0)),
            basis_v=Vector((0.0, 0.0, 1.0)),
            patch_type=PatchType.WALL,
            world_facing=WorldFacing.SIDE,
            boundary_loops=[loop],
        )
    )
    triangles = tuple(
        SurfaceTriangle(
            index - 1,
            0,
            (U_ROUTE_NGON[0], U_ROUTE_NGON[index], U_ROUTE_NGON[index + 1]),
            (
                edge_of.get((U_ROUTE_NGON[index], U_ROUTE_NGON[index + 1])),
                None,
                None,
            ),
            (1.0, 0.0, 0.0),
        )
        for index in range(1, len(U_ROUTE_NGON) - 1)
    )
    surface = PatchSurfaceIR(
        revision,
        vertices=tuple(
            SourceVertex(item, position[item]) for item in sorted(U_ROUTE_NGON)
        ),
        edges=tuple(
            SourceEdge(edge_of[key], key, (0,))
            for key in sorted(edge_of, key=edge_of.__getitem__)
        ),
        faces=(
            SourceFace(
                0,
                0,
                U_ROUTE_NGON,
                tuple(range(len(U_ROUTE_NGON))),
                (1.0, 0.0, 0.0),
                tuple(item.triangle_id for item in triangles),
            ),
        ),
        triangles=triangles,
    )
    return AnalysisBundle(revision, graph, surface)


# --------------------------------------------------------------------------
# Обратное отображение выпущенного снапшота в факты хоста
# --------------------------------------------------------------------------

_TAG_TO_PATCH_TYPE = {
    kernel.PatchContextTag.WALL: PatchType.WALL,
    kernel.PatchContextTag.FLOOR: PatchType.FLOOR,
    kernel.PatchContextTag.SLOPE: PatchType.SLOPE,
}
_CHAIN_KIND_TO_NEIGHBOR = {
    kernel.PhysicalChainKind.PHYSICAL_DECAL_SOURCE: -1,
    kernel.PhysicalChainKind.SEAM_SELF: -2,
}


def host_number(identity) -> int:
    return int(identity.value.rsplit(":", 1)[1])


def _vector(value) -> tuple[float, float, float]:
    return (float(value.x), float(value.y), float(value.z))


def _patch_surface(snapshot, revision, position_by_id):
    surface = snapshot.surface_ir
    faces = sorted(surface.source_faces, key=lambda item: host_number(item.face_id))
    face_ids_by_edge: dict[int, list[int]] = {}
    for face in faces:
        for edge_id in face.edge_cycle:
            face_ids_by_edge.setdefault(host_number(edge_id), []).append(
                host_number(face.face_id)
            )

    def host_edge(value):
        return None if value is None else host_number(value)

    return PatchSurfaceIR(
        revision,
        vertices=tuple(
            SourceVertex(vertex_id, position_by_id[vertex_id])
            for vertex_id in sorted(position_by_id)
        ),
        edges=tuple(
            SourceEdge(
                host_number(item.edge_id),
                (host_number(item.vertex_a_id), host_number(item.vertex_b_id)),
                tuple(sorted(face_ids_by_edge.get(host_number(item.edge_id), ()))),
            )
            for item in sorted(
                surface.source_edges, key=lambda item: host_number(item.edge_id)
            )
        ),
        faces=tuple(
            SourceFace(
                host_number(item.face_id),
                host_number(item.patch_id),
                tuple(host_number(value) for value in item.vertex_cycle),
                tuple(host_number(value) for value in item.edge_cycle),
                _vector(item.polygon_normal),
                tuple(host_number(value) for value in item.triangle_ids),
            )
            for item in faces
        ),
        triangles=tuple(
            SurfaceTriangle(
                host_number(item.triangle_id),
                host_number(item.source_face_id),
                tuple(host_number(value) for value in item.vertex_ids),
                # Экспортёр переставляет тройку при записи; здесь она
                # возвращается на место, а не пересобирается заново.
                (
                    host_edge(item.physical_edge_ids[1]),
                    host_edge(item.physical_edge_ids[2]),
                    host_edge(item.physical_edge_ids[0]),
                ),
                _vector(item.triangle_normal),
            )
            for item in sorted(
                surface.surface_triangles,
                key=lambda item: host_number(item.triangle_id),
            )
        ),
    )


def bundle_from_exported_snapshot(snapshot) -> tuple[AnalysisBundle, int]:
    """Вернуть `(bundle, patch_id)` по выпущенному хостом снапшоту."""

    _, digest, source_name = snapshot.source_revision.value.split(":", 2)
    revision = SourceRevision(source_name, digest)
    position_by_id = {
        host_number(item.vertex_id): _vector(item.position)
        for item in snapshot.source_vertices
    }
    patch_surface = _patch_surface(snapshot, revision, position_by_id)

    chains_by_id = {
        item.physical_chain_id: item for item in snapshot.physical_chains
    }
    uses_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    peers_by_chain: dict[object, set[int]] = {}
    for use in snapshot.chain_uses:
        peers_by_chain.setdefault(use.physical_chain_id, set()).add(
            host_number(use.owner_patch_id)
        )

    descriptor_by_patch = {
        host_number(item.patch_id): item for item in snapshot.patches
    }
    loops_by_domain: dict[object, list] = {}
    for loop in snapshot.boundary_loops:
        loops_by_domain.setdefault(loop.patch_domain_id, []).append(loop)
    domain_by_patch = {
        host_number(item.owner_patch_id): item.patch_domain_id
        for item in snapshot.patch_domains
    }

    graph = PatchGraph(source_revision=revision)
    for patch_id in sorted(domain_by_patch):
        (tag,) = descriptor_by_patch[patch_id].context_tags
        boundary_loops = []
        for loop in sorted(
            loops_by_domain[domain_by_patch[patch_id]],
            key=lambda item: item.boundary_loop_id.value,
        ):
            chains = []
            for use_id in loop.ordered_chain_use_ids:
                use = uses_by_id[use_id]
                chain = chains_by_id[use.physical_chain_id]
                vertices = [
                    host_number(item) for item in chain.ordered_source_vertex_ids
                ]
                edges = [
                    host_number(item) for item in chain.ordered_physical_edge_ids
                ]
                if use.orientation is kernel.ChainUseOrientation.B_START_TO_END:
                    vertices.reverse()
                    edges.reverse()
                neighbor = _CHAIN_KIND_TO_NEIGHBOR.get(chain.kind)
                if neighbor is None:
                    peers = peers_by_chain[use.physical_chain_id] - {patch_id}
                    neighbor = min(peers) if peers else patch_id + 1
                chains.append(
                    BoundaryChain(
                        vert_indices=vertices,
                        vert_cos=[
                            Vector(position_by_id[item]) for item in vertices
                        ],
                        edge_indices=edges,
                        side_face_indices=[],
                        side_face_normals=[],
                        is_closed=bool(chain.is_closed),
                        neighbor_patch_id=neighbor,
                    )
                )
            corners = [
                BoundaryCorner(
                    vert_index=item.vert_indices[-1],
                    vert_co=Vector(position_by_id[item.vert_indices[-1]]),
                    prev_chain_index=index,
                    next_chain_index=(index + 1) % len(chains),
                )
                for index, item in enumerate(chains)
                if item.vert_indices[-1]
                == chains[(index + 1) % len(chains)].vert_indices[0]
            ]
            loop_vertices = [
                vertex for item in chains for vertex in item.vert_indices[:-1]
            ]
            boundary_loops.append(
                BoundaryLoop(
                    vert_indices=loop_vertices,
                    vert_cos=[
                        Vector(position_by_id[item]) for item in loop_vertices
                    ],
                    edge_indices=[
                        edge for item in chains for edge in item.edge_indices
                    ],
                    side_face_indices=[],
                    kind=LoopKind(loop.kind.value),
                    chains=chains,
                    corners=corners,
                )
            )
        graph.add_node(
            PatchNode(
                patch_id=patch_id,
                face_indices=[
                    face.face_id
                    for face in patch_surface.faces
                    if face.patch_id == patch_id
                ],
                patch_type=_TAG_TO_PATCH_TYPE[tag],
                boundary_loops=boundary_loops,
            )
        )
    (only_patch_id,) = sorted(domain_by_patch)
    return AnalysisBundle(revision, graph, patch_surface), only_patch_id
