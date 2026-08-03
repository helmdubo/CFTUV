"""Крошечные синтетические поверхности для отрицательных контролей S0."""

from __future__ import annotations

from cftuv_envelope.contracts.surface import (
    PatchSurfaceIRV1,
    SourceEdgeV1,
    SourceFaceV1,
    SurfacePayloadMode,
    SurfaceTriangleV1,
)
from cftuv_envelope.ids import (
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from cftuv_envelope.numeric import LocalVector3V1


SCHEMA = "cftuv.envelope.patch_surface_ir.v1"
REVISION = SourceRevision("synthetic:S0")
PATCH = PatchId("synthetic:patch")
NORMAL = LocalVector3V1(0.0, 0.0, 1.0)


def vertex(index: int) -> SourceVertexId:
    return SourceVertexId(f"synthetic:vertex:{index:04d}")


def edge(index: int) -> PhysicalEdgeId:
    return PhysicalEdgeId(f"synthetic:edge:{index:04d}")


def face(index: int) -> SourceFaceId:
    return SourceFaceId(f"synthetic:face:{index:04d}")


def triangle(index: int) -> SurfaceTriangleId:
    return SurfaceTriangleId(f"synthetic:triangle:{index:04d}")


def _edge_table(triangles):
    table: dict = {}
    for corners in triangles:
        for index in range(3):
            pair = frozenset((corners[index], corners[(index + 1) % 3]))
            table.setdefault(pair, len(table))
    return table


def triangulated_surface(triangles, *, repeat_vertex_in_cycle=False):
    """Каждый треугольник — своя грань; все стороны физические рёбра."""

    table = _edge_table(triangles)
    source_edges = frozenset(
        SourceEdgeV1(
            edge_id=edge(index),
            vertex_a_id=vertex(min(pair)),
            vertex_b_id=vertex(max(pair)),
        )
        for pair, index in table.items()
    )
    records, faces = [], []
    for number, corners in enumerate(triangles):
        vertex_ids = tuple(vertex(value) for value in corners)
        edge_cycle = tuple(
            edge(table[frozenset((corners[index], corners[(index + 1) % 3]))])
            for index in range(3)
        )
        records.append(
            SurfaceTriangleV1(
                triangle_id=triangle(number),
                source_face_id=face(number),
                vertex_ids=vertex_ids,
                physical_edge_ids=edge_cycle,
                triangle_normal=NORMAL,
            )
        )
        cycle = vertex_ids
        cycle_edges = edge_cycle
        if repeat_vertex_in_cycle and number == 0:
            # Дверь fail-open: треугольники повтор вершины запрещают, а цикл
            # грани — нет. Ребро-петля здесь не заводится намеренно: повтор
            # берётся ровно тот, который пропускает нынешний валидатор.
            cycle = vertex_ids + (vertex_ids[0],)
            cycle_edges = edge_cycle + (edge_cycle[0],)
        faces.append(
            SourceFaceV1(
                face_id=face(number),
                patch_id=PATCH,
                vertex_cycle=cycle,
                edge_cycle=cycle_edges,
                polygon_normal=NORMAL,
                triangle_ids=(triangle(number),),
            )
        )
    used = sorted({value for corners in triangles for value in corners})
    return PatchSurfaceIRV1(
        schema_version=SCHEMA,
        source_revision=REVISION,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(vertex(value) for value in used),
        source_edges=source_edges,
        source_faces=frozenset(faces),
        surface_triangles=frozenset(records),
    )


def two_triangle_quad():
    """Две грани, склеенные по диагонали, с согласованной ориентацией."""

    return triangulated_surface([(0, 1, 2), (0, 2, 3)])


def flipped_quad():
    """То же, но вторая грань обходится в ту же сторону: ориентации нет."""

    return triangulated_surface([(0, 1, 2), (0, 3, 2)])


def non_manifold_fin():
    """Три треугольника на одном ребре: «противоположной» стороны не бывает."""

    return triangulated_surface([(0, 1, 2), (1, 0, 3), (0, 1, 4)])


def bowtie_pinch():
    """Два треугольника, встречающиеся ТОЛЬКО в одной вершине.

    Ни одна сторона не общая, поэтому склеек нет вовсе — а веер вершины 0 всё
    равно не складывается: он распадается на две несвязные половины. Это
    защемление, а не край, и называться оно обязано немногообразием.
    """

    return triangulated_surface([(0, 1, 2), (0, 3, 4)])


def three_coincident_components():
    """Три РАЗДЕЛЬНЫХ треугольника с совпадающими позициями.

    Позиции совпадают побитово, идентичности различны. Склейка по совпадению
    координат означала бы, что близко лежащие листы сливаются в один — ровно
    та ошибка, против которой стоит фикстура `close_parallel_sheets`.
    """

    return triangulated_surface([(0, 1, 2), (3, 4, 5), (6, 7, 8)])


def coincident_positions():
    """Позиции трёх совпадающих компонент: три копии одного треугольника."""

    from fractions import Fraction

    corners = ((Fraction(0), Fraction(0), Fraction(0)),
               (Fraction(1), Fraction(0), Fraction(0)),
               (Fraction(0), Fraction(1), Fraction(0)))
    return {
        vertex(index): corners[index % 3] for index in range(9)
    }


__all__ = (
    "PATCH",
    "REVISION",
    "bowtie_pinch",
    "coincident_positions",
    "edge",
    "face",
    "flipped_quad",
    "non_manifold_fin",
    "three_coincident_components",
    "triangle",
    "triangulated_surface",
    "two_triangle_quad",
    "vertex",
)
