"""Build and validate the decal-facing PatchSurfaceIR in analysis."""

from __future__ import annotations

import importlib
from hashlib import sha256

from .surface_ir import (
    AnalysisBundle,
    AnalysisCapabilities,
    AnalysisCrossIrError,
    AnalysisSchemaError,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)


def _vec3(value) -> tuple[float, float, float]:
    return tuple(float(value[index]) for index in range(3))


def source_revision_from_bmesh(bm, obj=None, face_indices=None) -> SourceRevision:
    """Exact analysis-input fingerprint; float.hex is representation only."""

    digest = sha256()
    requested_faces = (
        None if face_indices is None else {int(value) for value in face_indices}
    )
    scope = tuple(
        sorted(
            int(face.index) for face in bm.faces
            if requested_faces is None or int(face.index) in requested_faces
        )
    )
    digest.update(("scope:" + ",".join(map(str, scope)) + ";").encode("ascii"))
    for vertex in sorted(bm.verts, key=lambda item: int(item.index)):
        digest.update(f"v:{int(vertex.index)}:".encode("ascii"))
        digest.update(
            ",".join(float(vertex.co[index]).hex() for index in range(3)).encode(
                "ascii"
            )
        )
        digest.update(b";")
    for edge in sorted(bm.edges, key=lambda item: int(item.index)):
        endpoints = tuple(sorted(int(vertex.index) for vertex in edge.verts))
        digest.update(
            (
                f"e:{int(edge.index)}:{endpoints[0]},{endpoints[1]}:"
                f"seam={int(bool(getattr(edge, 'seam', False)))};"
            ).encode("ascii")
        )
    for face in sorted(bm.faces, key=lambda item: int(item.index)):
        cycle = ",".join(str(int(loop.vert.index)) for loop in face.loops)
        digest.update(f"f:{int(face.index)}:{cycle};".encode("ascii"))
    return SourceRevision(
        source_name=str(getattr(obj, "name", "") or ""),
        digest=digest.hexdigest(),
    )


def _triangle_normal(vertex_ids, vertex_positions):
    first, second, third = (vertex_positions[index] for index in vertex_ids)
    ab = tuple(second[index] - first[index] for index in range(3))
    ac = tuple(third[index] - first[index] for index in range(3))
    normal = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length_squared = sum(component * component for component in normal)
    if length_squared <= 0.0:
        return (0.0, 0.0, 0.0)
    inverse_length = length_squared ** -0.5
    return tuple(component * inverse_length for component in normal)


def _physical_triangle_edges(face, vertex_ids):
    edge_by_pair = {
        frozenset((int(edge.verts[0].index), int(edge.verts[1].index))): int(
            edge.index
        )
        for edge in face.edges
    }
    return tuple(
        edge_by_pair.get(
            frozenset(
                (
                    vertex_ids[(opposite + 1) % 3],
                    vertex_ids[(opposite + 2) % 3],
                )
            )
        )
        for opposite in range(3)
    )


def build_patch_surface_ir(bm, patch_graph, source_revision):
    """Serialize original face cycles and Blender loop-triangle tessellation."""

    selected_face_ids = set(int(face_id) for face_id in patch_graph.face_to_patch)
    selected_vertices = {
        int(vertex.index)
        for face in bm.faces
        if int(face.index) in selected_face_ids
        for vertex in face.verts
    }
    vertex_positions = {
        int(vertex.index): _vec3(vertex.co)
        for vertex in bm.verts
        if int(vertex.index) in selected_vertices
    }
    face_by_id = {
        int(face.index): face
        for face in bm.faces
        if int(face.index) in selected_face_ids
    }
    triangles_by_face = {face_id: [] for face_id in selected_face_ids}
    triangles = []
    for loop_triangle in bm.calc_loop_triangles():
        face = loop_triangle[0].face
        face_id = int(face.index)
        if face_id not in selected_face_ids:
            continue
        vertex_ids = tuple(int(loop.vert.index) for loop in loop_triangle)
        triangle_id = len(triangles)
        triangles_by_face[face_id].append(triangle_id)
        triangles.append(
            SurfaceTriangle(
                triangle_id=triangle_id,
                source_face_id=face_id,
                vertex_ids=vertex_ids,
                physical_edge_ids=_physical_triangle_edges(face, vertex_ids),
                triangle_normal=_triangle_normal(vertex_ids, vertex_positions),
            )
        )

    faces = []
    for face_id in sorted(face_by_id):
        face = face_by_id[face_id]
        faces.append(
            SourceFace(
                face_id=face_id,
                patch_id=int(patch_graph.face_to_patch[face_id]),
                vertex_cycle=tuple(int(loop.vert.index) for loop in face.loops),
                edge_cycle=tuple(int(loop.edge.index) for loop in face.loops),
                polygon_normal=_vec3(face.normal),
                triangle_ids=tuple(triangles_by_face[face_id]),
            )
        )

    edge_face_ids = {}
    for face in faces:
        for edge_id in face.edge_cycle:
            edge_face_ids.setdefault(edge_id, set()).add(face.face_id)
    edges = []
    for edge_id in sorted(edge_face_ids):
        edge = bm.edges[edge_id]
        edges.append(
            SourceEdge(
                edge_id=edge_id,
                vertex_ids=tuple(sorted(int(vertex.index) for vertex in edge.verts)),
                source_face_ids=tuple(sorted(edge_face_ids[edge_id])),
            )
        )
    return PatchSurfaceIR(
        source_revision=source_revision,
        vertices=tuple(
            SourceVertex(vertex_id, vertex_positions[vertex_id])
            for vertex_id in sorted(vertex_positions)
        ),
        edges=tuple(edges),
        faces=tuple(faces),
        triangles=tuple(triangles),
    )


def _fail(invariant, details):
    raise AnalysisCrossIrError(invariant, details)


def validate_analysis_bundle(bundle: AnalysisBundle) -> None:
    """Enforce the mandatory PatchGraph/PatchSurfaceIR cross-contract."""

    bundle.capabilities.require_supported()
    graph = bundle.patch_graph
    surface = bundle.patch_surface
    vertices = surface.vertex_by_id
    edges = surface.edge_by_id
    faces = surface.face_by_id
    triangles = surface.triangle_by_id

    if set(graph.face_to_patch) != set(faces):
        _fail(
            "FACE_SET",
            f"graph={sorted(graph.face_to_patch)} surface={sorted(faces)}",
        )
    for face_id, patch_id in graph.face_to_patch.items():
        if faces[face_id].patch_id != int(patch_id):
            _fail(
                "FACE_TO_PATCH",
                f"face={face_id} graph={patch_id} surface={faces[face_id].patch_id}",
            )
    for face in surface.faces:
        if len(face.vertex_cycle) != len(face.edge_cycle) or len(face.vertex_cycle) < 3:
            _fail("FACE_CYCLE", f"face={face.face_id}")
        if any(vertex_id not in vertices for vertex_id in face.vertex_cycle):
            _fail("FACE_VERTEX", f"face={face.face_id}")
        if any(edge_id not in edges for edge_id in face.edge_cycle):
            _fail("FACE_EDGE", f"face={face.face_id}")
        for cycle_index, edge_id in enumerate(face.edge_cycle):
            expected_endpoints = frozenset(
                (
                    face.vertex_cycle[cycle_index],
                    face.vertex_cycle[(cycle_index + 1) % len(face.vertex_cycle)],
                )
            )
            if frozenset(edges[edge_id].vertex_ids) != expected_endpoints:
                _fail(
                    "FACE_EDGE_CYCLE",
                    f"face={face.face_id} edge={edge_id}",
                )
        for triangle_id in face.triangle_ids:
            triangle = triangles.get(triangle_id)
            if triangle is None or triangle.source_face_id != face.face_id:
                _fail(
                    "TRIANGLE_FACE_PROVENANCE",
                    f"face={face.face_id} triangle={triangle_id}",
                )
    for triangle in surface.triangles:
        if triangle.source_face_id not in faces:
            _fail("TRIANGLE_FACE_PROVENANCE", f"triangle={triangle.triangle_id}")
        if any(vertex_id not in vertices for vertex_id in triangle.vertex_ids):
            _fail("TRIANGLE_VERTEX", f"triangle={triangle.triangle_id}")
        if any(
            edge_id is not None and edge_id not in edges
            for edge_id in triangle.physical_edge_ids
        ):
            _fail("TRIANGLE_EDGE", f"triangle={triangle.triangle_id}")
        for opposite, edge_id in enumerate(triangle.physical_edge_ids):
            if edge_id is None:
                continue
            expected_endpoints = frozenset(
                (
                    triangle.vertex_ids[(opposite + 1) % 3],
                    triangle.vertex_ids[(opposite + 2) % 3],
                )
            )
            if frozenset(edges[edge_id].vertex_ids) != expected_endpoints:
                _fail(
                    "TRIANGLE_EDGE_PROVENANCE",
                    f"triangle={triangle.triangle_id} edge={edge_id}",
                )

    declared_triangle_ids = tuple(
        triangle_id
        for face in surface.faces
        for triangle_id in face.triangle_ids
    )
    if (
        len(set(declared_triangle_ids)) != len(declared_triangle_ids)
        or set(declared_triangle_ids) != set(triangles)
    ):
        _fail("TRIANGLE_PARTITION", "SourceFace triangle_ids are not total")
    actual_edge_faces = {edge_id: set() for edge_id in edges}
    for face in surface.faces:
        for edge_id in face.edge_cycle:
            actual_edge_faces[edge_id].add(face.face_id)
    for edge_id, edge in edges.items():
        if tuple(sorted(actual_edge_faces[edge_id])) != edge.source_face_ids:
            _fail("EDGE_FACE_INCIDENCE", f"edge={edge_id}")

    for patch_id, node in graph.nodes.items():
        if set(int(face_id) for face_id in node.face_indices) != {
            face.face_id for face in surface.patch_faces(patch_id)
        }:
            _fail("PATCH_FACE_SET", f"patch={patch_id}")
        for boundary_loop in node.boundary_loops:
            loop_vertex_ids = tuple(int(value) for value in boundary_loop.vert_indices)
            loop_edge_sequence = tuple(
                int(value) for value in boundary_loop.edge_indices
            )
            loop_edge_ids = set(loop_edge_sequence)
            if len(loop_vertex_ids) != len(boundary_loop.vert_cos):
                _fail("BOUNDARY_COORDINATES", f"patch={patch_id}")
            if len(loop_vertex_ids) != len(loop_edge_sequence):
                _fail("BOUNDARY_CYCLE", f"patch={patch_id}")
            if any(vertex_id not in vertices for vertex_id in loop_vertex_ids):
                _fail("BOUNDARY_VERTEX", f"patch={patch_id}")
            if any(edge_id not in edges for edge_id in loop_edge_ids):
                _fail("BOUNDARY_EDGE", f"patch={patch_id}")
            for vertex_id, coordinate in zip(loop_vertex_ids, boundary_loop.vert_cos):
                if _vec3(coordinate) != vertices[vertex_id].position:
                    _fail(
                        "VERTEX_COORDINATE",
                        f"patch={patch_id} vertex={vertex_id}",
                    )
            for chain in boundary_loop.chains:
                chain_edges = set(int(value) for value in chain.edge_indices)
                if not chain_edges.issubset(loop_edge_ids):
                    _fail("CHAIN_BOUNDARY_CYCLE", f"patch={patch_id}")
                # Open chains keep an endpoint-side face id in addition to the
                # per-segment prefix. Normals remain strictly segment-aligned.
                if (
                    len(chain.side_face_normals) != len(chain.edge_indices)
                    or len(chain.side_face_indices) < len(chain.edge_indices)
                ):
                    raise AnalysisSchemaError(
                        "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
                        "per-segment chain owner normals/faces are required: "
                        f"patch={patch_id}",
                    )
                if len(chain.vert_indices) != len(chain.vert_cos):
                    _fail("CHAIN_COORDINATES", f"patch={patch_id}")
                expected_vertex_count = len(chain.edge_indices) + (
                    0 if getattr(chain, "is_closed", False) else 1
                )
                if len(chain.vert_indices) != expected_vertex_count:
                    _fail("CHAIN_CYCLE", f"patch={patch_id}")
                for vertex_id, coordinate in zip(chain.vert_indices, chain.vert_cos):
                    if (
                        int(vertex_id) not in vertices
                        or _vec3(coordinate) != vertices[int(vertex_id)].position
                    ):
                        _fail(
                            "VERTEX_COORDINATE",
                            f"patch={patch_id} vertex={vertex_id}",
                        )


def build_analysis_bundle(bm, face_indices, obj, build_patch_graph):
    """Build both IRs from one source snapshot and validate before publish."""

    face_indices = tuple(int(value) for value in face_indices)
    revision = source_revision_from_bmesh(bm, obj, face_indices)
    graph = build_patch_graph(bm, face_indices, obj, source_revision=revision)
    surface = build_patch_surface_ir(bm, graph, revision)
    bundle = AnalysisBundle(
        source_revision=revision,
        patch_graph=graph,
        patch_surface=surface,
        capabilities=AnalysisCapabilities(),
    )
    validate_analysis_bundle(bundle)
    return bundle


# --------------------------------------------------------------------------
# Смежность треугольников поверхности (карточка S0)
#
# Заполняется ОТДЕЛЬНОЙ функцией и ничего не меняет в `PatchSurfaceIR`: байты
# снапшота неподвижны, таблица живёт рядом. Идентичность сюда ПЕРЕДАЁТСЯ, а не
# минтится: функция физически не может выдумать `PhysicalEdgeId` диагонали,
# потому что у неё нет ни одного id, которого ей не дали.
#
# Blender нужен ровно одной строчке — `surface_edge_face_counts`. Всё
# остальное работает от `PatchSurfaceIR` (модуль `surface_ir` Blender-free),
# поэтому тестируется без `bpy`.
# --------------------------------------------------------------------------


class SurfaceAdjacencyBuildRejected(ValueError):
    """Хост не может назвать смежность честно и отказывается её называть."""

    def __init__(self, reason: str, details: str = ""):
        self.reason = str(reason)
        self.details = str(details)
        message = self.reason
        if self.details:
            message += f": {self.details}"
        super().__init__(message)


def surface_edge_face_counts(bm, edge_indices=None) -> dict:
    """Сколько граней ПОЛНОГО меша держит каждое ребро.

    Единственное место, читающее bmesh. Без этого числа сосед вне среза
    неотличим от границы меша — ровно та неоднозначность, ради которой
    заводится таблица.
    """

    requested = None if edge_indices is None else {int(v) for v in edge_indices}
    return {
        int(edge.index): len(edge.link_faces)
        for edge in bm.edges
        if requested is None or int(edge.index) in requested
    }


def _triangle_side_records(patch_surface):
    """`(triangle_id, ordinal) -> (from, to, physical_edge_id)` по всем сторонам.

    `ordinal` здесь ЯДЕРНЫЙ: сторона `i` — пара `(v[i], v[(i + 1) % 3])`. У
    хоста тот же кортеж индексируется противолежащей вершиной, поэтому
    физическое ребро берётся со сдвигом `(i + 2) % 3` — тем же самым, который
    делает экспортёр (`envelope_request_export.py`: `[2], [0], [1]`). Сдвиг
    записан ЗДЕСЬ, а не подразумевается: два закона нумерации одной величины
    уже живут в конвейере, и третьего молчаливого быть не должно.
    """

    records = {}
    for triangle in patch_surface.triangles:
        for ordinal in range(3):
            records[(int(triangle.triangle_id), ordinal)] = (
                int(triangle.vertex_ids[ordinal]),
                int(triangle.vertex_ids[(ordinal + 1) % 3]),
                triangle.physical_edge_ids[(ordinal + 2) % 3],
            )
    return records


def _sides_by_endpoint_pair(records):
    grouped = {}
    for key, (first, second, _edge_id) in records.items():
        grouped.setdefault(frozenset((first, second)), []).append(key)
    return {pair: tuple(sorted(keys)) for pair, keys in grouped.items()}


def _lone_side_reason(edge_id, scope_is_full, edge_face_counts):
    """Чем кончается сторона без пары. Ни одна ветка не выбирается молча."""

    if edge_id is None:
        # Диагональ тесселяции лежит ВНУТРИ грани и пару обязана иметь.
        return "HOST_DECLARED_UNKNOWN"
    if edge_face_counts is None:
        return "MESH_BOUNDARY" if scope_is_full else "HOST_DECLARED_UNKNOWN"
    count = edge_face_counts.get(int(edge_id))
    if count is None:
        return "HOST_DECLARED_UNKNOWN"
    if count >= 3:
        return "NON_MANIFOLD_FAN"
    if count <= 1:
        return "MESH_BOUNDARY"
    if scope_is_full:
        raise SurfaceAdjacencyBuildRejected(
            "SURFACE_ADJACENCY_SCOPE_DECLARATION_INVALID",
            f"ребро {int(edge_id)} держат {count} граней полного меша, но в "
            "payload попала одна: срез объявлен полным мешем",
        )
    return "OUTSIDE_REQUESTED_SCOPE"


def _opposite_for_pair(keys, records, scope_is_full, edge_face_counts):
    """Что стоит по другую сторону каждой стороны одной пары концов."""

    if len(keys) > 2:
        return {key: ("BOUNDARY", "NON_MANIFOLD_FAN") for key in keys}
    if len(keys) == 1:
        key = keys[0]
        reason = _lone_side_reason(
            records[key][2], scope_is_full, edge_face_counts
        )
        return {key: ("BOUNDARY", reason)}
    left, right = keys
    if records[left][0] != records[right][1] or records[left][1] != records[right][0]:
        # Две стороны проходят общую пару концов в ОДНУ сторону: цикл граней
        # источника не согласован, и склеивать их значило бы объявить
        # ориентацию, которой нет.
        return {key: ("BOUNDARY", "HOST_DECLARED_UNKNOWN") for key in keys}
    return {left: ("SIDE", right), right: ("SIDE", left)}


def _vertex_incidence(patch_surface):
    incidence = {}
    for triangle in patch_surface.triangles:
        for vertex_id in triangle.vertex_ids:
            incidence.setdefault(int(vertex_id), set()).add(int(triangle.triangle_id))
    return incidence


def _fan_links(vertex_id, triangle_ids, records, opposites, named):
    """Граф склеек при вершине В ЯДЕРНЫХ id: соседи через стороны при ней.

    Ключи — уже ядерные `SurfaceTriangleId`, потому что канонический порядок
    веера определён контрактом ИМЕННО на них. Сортировка по хостовому целому
    дала бы другой конец цепи (числа и строки упорядочены по-разному), и два
    вывода одной смежности разошлись бы на печати, совпав по существу.
    """

    links = {named[triangle_id]: set() for triangle_id in triangle_ids}
    reasons = set()
    for triangle_id in triangle_ids:
        for ordinal in range(3):
            key = (triangle_id, ordinal)
            first, second, _edge = records[key]
            if vertex_id not in (first, second):
                continue
            kind, payload = opposites[key]
            if kind == "SIDE" and payload[0] in named:
                links[named[triangle_id]].add(named[payload[0]])
            elif kind != "SIDE":
                reasons.add(payload)
    return links, reasons


def _fan_unavailable_reason(reasons):
    for candidate in (
        "NON_MANIFOLD_FAN",
        "OUTSIDE_REQUESTED_SCOPE",
        "HOST_DECLARED_UNKNOWN",
    ):
        if candidate in reasons:
            return candidate
    return "NON_MANIFOLD_FAN"


def _named_id(table, key, kind):
    value = table.get(int(key))
    if value is None:
        raise SurfaceAdjacencyBuildRejected(
            "SURFACE_ADJACENCY_IDENTITY_MISSING",
            f"{kind} {int(key)} не назван вызывающим",
        )
    return value


def _adjacency_sides(contracts, records, opposites, identity):
    """Стороны таблицы. Каждый id БЕРЁТСЯ из переданной идентичности."""

    triangle_ids, vertex_ids, edge_ids = identity
    sides = []
    for key, (first, second, edge_id) in records.items():
        kind, payload = opposites[key]
        if kind == "SIDE":
            opposite = contracts.TriangleSideRefV1(
                triangle_id=_named_id(triangle_ids, payload[0], "треугольник"),
                side_ordinal=payload[1],
            )
        else:
            opposite = contracts.SideBoundaryV1(
                reason=contracts.SideBoundaryReasonV1(payload)
            )
        sides.append(
            contracts.TriangleSideV1(
                side=contracts.TriangleSideRefV1(
                    triangle_id=_named_id(triangle_ids, key[0], "треугольник"),
                    side_ordinal=key[1],
                ),
                from_vertex_id=_named_id(vertex_ids, first, "вершина"),
                to_vertex_id=_named_id(vertex_ids, second, "вершина"),
                kind=(
                    contracts.TriangleSideKindV1.FACE_DIAGONAL
                    if edge_id is None
                    else contracts.TriangleSideKindV1.SOURCE_EDGE
                ),
                physical_edge_id=(
                    None if edge_id is None else _named_id(edge_ids, edge_id, "ребро")
                ),
                opposite=opposite,
            )
        )
    return frozenset(sides)


def _adjacency_fans(contracts, patch_surface, records, opposites, identity):
    triangle_ids, vertex_ids, _edge_ids = identity
    fans = []
    for vertex_id, incident in sorted(_vertex_incidence(patch_surface).items()):
        links, reasons = _fan_links(
            vertex_id, sorted(incident), records, opposites, triangle_ids
        )
        ordered = contracts.canonical_fan_order(links)
        named_vertex = _named_id(vertex_ids, vertex_id, "вершина")
        if ordered is None:
            fans.append(
                contracts.VertexFanUnavailableV1(
                    vertex_id=named_vertex,
                    reason=contracts.VertexFanUnavailableReasonV1(
                        _fan_unavailable_reason(reasons)
                    ),
                )
            )
            continue
        order, closed = ordered
        fans.append(
            contracts.VertexFanV1(
                vertex_id=named_vertex,
                ordered_triangle_ids=order,
                is_closed=closed,
            )
        )
    return frozenset(fans)


def build_surface_adjacency_ir(
    patch_surface,
    *,
    source_revision,
    triangle_ids,
    vertex_ids,
    edge_ids,
    scope,
    scope_patch_ids=(),
    edge_face_counts=None,
):
    """Собрать `SurfaceAdjacencyIRV1` из уже построенного `PatchSurfaceIR`.

    Идентичность (`triangle_ids`, `vertex_ids`, `edge_ids`, `source_revision`)
    ПЕРЕДАЁТСЯ вызывающим: смежность её не минтит и потому не может заразить
    `PhysicalEdgeId` синтетикой. `edge_face_counts` — единственная величина,
    которую даёт bmesh, и единственная, которая отличает границу меша от
    границы запроса; без неё срез честно отвечает `HOST_DECLARED_UNKNOWN`.
    """

    kernel_module = importlib.import_module(
        "cftuv_envelope.contracts.surface_adjacency"
    )
    scope_value = kernel_module.SurfaceAdjacencyScopeV1(str(scope))
    scope_is_full = scope_value is kernel_module.SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    records = _triangle_side_records(patch_surface)
    opposites = {}
    for keys in _sides_by_endpoint_pair(records).values():
        opposites.update(
            _opposite_for_pair(keys, records, scope_is_full, edge_face_counts)
        )
    identity = (triangle_ids, vertex_ids, edge_ids)
    return kernel_module.SurfaceAdjacencyIRV1(
        schema_version=kernel_module.SURFACE_ADJACENCY_IR_SCHEMA_V1,
        source_revision=source_revision,
        scope=scope_value,
        scope_patch_ids=frozenset(scope_patch_ids),
        orientation_law=(
            kernel_module.SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
        ),
        triangle_sides=_adjacency_sides(
            kernel_module, records, opposites, identity
        ),
        vertex_fans=_adjacency_fans(
            kernel_module, patch_surface, records, opposites, identity
        ),
    )


__all__ = (
    "SurfaceAdjacencyBuildRejected",
    "build_analysis_bundle",
    "build_patch_surface_ir",
    "build_surface_adjacency_ir",
    "source_revision_from_bmesh",
    "surface_edge_face_counts",
    "validate_analysis_bundle",
)
