"""Headless BMesh-shaped view over `cftuv.mesh_snapshot.v1` field snapshots.

Диагностический харнесс карточки PERF-PREPARE-DIAG. Никакого продакшн-кода не
трогает: реализует ровно тот срез API bmesh, который читает
`cftuv.analysis.build_analysis_bundle` (см. `analysis_surface.py`,
`analysis_topology.py`), чтобы полевой слепок можно было прогнать без Blender.

Список читаемых атрибутов снят грепом по хост-анализу:
  bm.verts/edges/faces (+ ensure_lookup_table, индексация, итерация),
  bm.calc_loop_triangles()
  vert.index/co/link_edges/link_loops
  edge.index/verts/link_faces/seam/smooth/calc_length()
  face.index/loops/verts/edges/normal/select/calc_area()/calc_center_median()
  loop.vert/edge/face/link_loop_next/link_loop_prev
"""

from __future__ import annotations

import json
from pathlib import Path

from mathutils import Vector


class _Seq(list):
    def ensure_lookup_table(self):
        return None

    def index_update(self):
        return None


class FVert:
    __slots__ = (
        "index",
        "co",
        "link_edges",
        "link_loops",
        "link_faces",
        "select",
        "hide",
        "_normal",
    )

    def __init__(self, index, co):
        self.index = index
        self.co = Vector(co)
        self.link_edges = []
        self.link_loops = []
        self.link_faces = []
        self.select = False
        self.hide = False
        self._normal = None

    @property
    def normal(self):
        """Угловзвешенная нормаль вершины — тот же закон, что у bmesh."""

        if self._normal is None:
            total = Vector((0.0, 0.0, 0.0))
            for loop in self.link_loops:
                previous = loop.link_loop_prev.vert.co - self.co
                following = loop.link_loop_next.vert.co - self.co
                if previous.length <= 0.0 or following.length <= 0.0:
                    continue
                total = total + loop.face.normal * previous.angle(following, 0.0)
            self._normal = total.normalized() if total.length > 0.0 else Vector((0.0, 0.0, 1.0))
        return self._normal


class FEdge:
    __slots__ = (
        "index",
        "verts",
        "link_faces",
        "link_loops",
        "seam",
        "smooth",
        "select",
        "hide",
    )

    def __init__(self, index, verts, seam=False, sharp=False, select=False):
        self.index = index
        self.verts = list(verts)
        self.link_faces = []
        self.link_loops = []
        self.seam = bool(seam)
        # bmesh: `smooth == False` означает помеченное sharp ребро.
        self.smooth = not bool(sharp)
        self.select = bool(select)
        self.hide = False

    def calc_length(self):
        return (self.verts[0].co - self.verts[1].co).length

    def other_vert(self, vert):
        a, b = self.verts
        return b if vert is a else a

    def is_boundary(self):
        return len(self.link_faces) == 1

    def is_manifold(self):
        return len(self.link_faces) in (1, 2)


class FLoop:
    __slots__ = (
        "vert",
        "edge",
        "face",
        "link_loop_next",
        "link_loop_prev",
        "link_loop_radial_next",
        "link_loop_radial_prev",
        "index",
    )

    def __init__(self, vert, face):
        self.vert = vert
        self.face = face
        self.edge = None
        self.link_loop_next = None
        self.link_loop_prev = None
        self.link_loop_radial_next = self
        self.link_loop_radial_prev = self
        self.index = 0


class FFace:
    __slots__ = (
        "index",
        "loops",
        "verts",
        "edges",
        "normal",
        "select",
        "smooth",
        "hide",
        "material_index",
    )

    def __init__(self, index):
        self.index = index
        self.loops = []
        self.verts = []
        self.edges = []
        self.normal = Vector((0.0, 0.0, 1.0))
        self.select = False
        self.smooth = False
        self.hide = False
        self.material_index = 0

    def calc_center_median(self):
        total = Vector((0.0, 0.0, 0.0))
        for vert in self.verts:
            total = total + vert.co
        return total / float(len(self.verts))

    def calc_area(self):
        # Ньюэлл: половина модуля векторной площади многоугольника.
        total = Vector((0.0, 0.0, 0.0))
        count = len(self.verts)
        for i in range(count):
            a = self.verts[i].co
            b = self.verts[(i + 1) % count].co
            total = total + a.cross(b)
        return total.length * 0.5

    def calc_normal(self):
        return self.normal


def _newell_normal(points):
    nx = ny = nz = 0.0
    count = len(points)
    for i in range(count):
        ax, ay, az = points[i]
        bx, by, bz = points[(i + 1) % count]
        nx += (ay - by) * (az + bz)
        ny += (az - bz) * (ax + bx)
        nz += (ax - bx) * (ay + by)
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length == 0.0:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


class FakeBMesh:
    def __init__(self, vertices, edges, faces, loop_triangles=None):
        self.verts = _Seq(FVert(i, co) for i, co in enumerate(vertices))
        self.edges = _Seq()
        edge_by_pair = {}
        for i, row in enumerate(edges):
            a, b = int(row[0]), int(row[1])
            flags = row[2] if len(row) > 2 and isinstance(row[2], dict) else {}
            edge = FEdge(
                i,
                (self.verts[a], self.verts[b]),
                seam=bool(flags.get("seam", 0)),
                sharp=bool(flags.get("sharp", 0)),
                select=bool(flags.get("selected", 0)),
            )
            self.edges.append(edge)
            edge_by_pair[frozenset((a, b))] = edge
            self.verts[a].link_edges.append(edge)
            self.verts[b].link_edges.append(edge)
        self.faces = _Seq()
        self.loops = _Seq()
        for i, cycle in enumerate(faces):
            face = FFace(i)
            cycle = [int(value) for value in cycle]
            face.verts = [self.verts[index] for index in cycle]
            loops = [FLoop(vert, face) for vert in face.verts]
            for offset, loop in enumerate(loops):
                loop.link_loop_next = loops[(offset + 1) % len(loops)]
                loop.link_loop_prev = loops[(offset - 1) % len(loops)]
                pair = frozenset((cycle[offset], cycle[(offset + 1) % len(cycle)]))
                edge = edge_by_pair[pair]
                loop.edge = edge
                loop.index = len(self.loops)
                self.loops.append(loop)
                face.edges.append(edge)
                edge.link_faces.append(face)
                edge.link_loops.append(loop)
                loop.vert.link_loops.append(loop)
            for vert in face.verts:
                vert.link_faces.append(face)
            face.loops = loops
            face.normal = Vector(_newell_normal([tuple(v.co) for v in face.verts]))
            self.faces.append(face)
        for edge in self.edges:
            loops = edge.link_loops
            for offset, loop in enumerate(loops):
                loop.link_loop_radial_next = loops[(offset + 1) % len(loops)]
                loop.link_loop_radial_prev = loops[(offset - 1) % len(loops)]
        self._loop_triangles = tuple(
            tuple(int(value) for value in row) for row in (loop_triangles or ())
        )

    def calc_loop_triangles(self):
        """Тройки петель, как их отдаёт bmesh: по одной петле на вершину."""

        result = []
        for row in self._loop_triangles:
            a, b, c, face_index = row
            face = self.faces[face_index]
            loop_by_vert = {loop.vert.index: loop for loop in face.loops}
            result.append((loop_by_vert[a], loop_by_vert[b], loop_by_vert[c]))
        return result


class FakeObject:
    """Ровно то, что читает анализ: `.name`, `.type`, `.data`, `.matrix_world`."""

    type = "MESH"

    def __init__(self, name):
        self.name = name
        self.data = None
        self.matrix_world = None


def load_snapshot(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def bmesh_from_snapshot(payload, *, layer="raw", vertex_override=None):
    block = payload[layer]
    vertices = [tuple(float(v) for v in row) for row in block["vertices"]]
    if vertex_override is not None:
        for index, position in vertex_override.items():
            vertices[index] = tuple(float(v) for v in position)
    triangles = payload.get("evaluated", {}).get("loop_triangles")
    return FakeBMesh(vertices, block["edges"], block["faces"], triangles)


def selected_edge_ids(payload):
    return frozenset(int(v) for v in payload["raw"]["selected_edges"])
