"""Входы тестов ступени DEVELOPABLE: поверхности в 3D из точек и циклов граней.

Поверхность задаётся словарём `имя -> (x, y, z)` и списком циклов граней по именам;
грань триангулируется веером `(v0, vi, vi+1)` в порядке цикла, как у хоста. Физические
рёбра — по паре концов (общая пара — общее ребро), поэтому граница патча считается
по рёбрам граней так же, как у настоящего снапшота. Нормали грубые: ядро их не читает
(власть — позиции и обход).

Фикстуры (все на двоичной сетке, кроме тех, где это названо): складка 90°, фаска,
четверть цилиндра, конус, купол, седло, спираль.
"""

from __future__ import annotations

import math

from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import GridSnappingLawV1
from cftuv_envelope.contracts.surface import SourceFaceV1, SurfaceTriangleV1
from cftuv_envelope.ids import (
    PatchDomainId,
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1

REVISION = SourceRevision("developable-revision")
DOMAIN = PatchDomainId("developable-domain")
PATCH = PatchId("developable-patch")


def surface(points: dict, cycles: list, *, patch: PatchId = PATCH):
    """`(вершины, грани, треугольники)` по именованным точкам и циклам граней."""

    ids = {name: SourceVertexId(f"v:{name}") for name in points}
    vertices = tuple(
        SourceVertexV1(ids[name], LocalPoint3V1(*(float(a) for a in position)))
        for name, position in points.items()
    )

    def edge(first, second):
        return PhysicalEdgeId("e:" + ":".join(sorted((first.value, second.value))))

    faces, triangles = [], []
    for index, cycle in enumerate(cycles):
        vertex_cycle = tuple(ids[name] for name in cycle)
        face_id = SourceFaceId(f"face{index:03d}")
        faces.append(
            SourceFaceV1(
                face_id=face_id,
                patch_id=patch,
                vertex_cycle=vertex_cycle,
                edge_cycle=tuple(
                    edge(vertex_cycle[k], vertex_cycle[(k + 1) % len(vertex_cycle)])
                    for k in range(len(vertex_cycle))
                ),
                polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
                triangle_ids=(),
            )
        )
        for k in range(1, len(vertex_cycle) - 1):
            triangles.append(
                SurfaceTriangleV1(
                    triangle_id=SurfaceTriangleId(f"{face_id.value}:t{k:02d}"),
                    source_face_id=face_id,
                    vertex_ids=(vertex_cycle[0], vertex_cycle[k], vertex_cycle[k + 1]),
                    physical_edge_ids=(None, None, None),
                    triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
                )
            )
    return vertices, tuple(faces), tuple(triangles)


def strip_cycles(rows: int) -> list:
    """Полоса из `rows - 1` квадов между кольцами `r0 .. r{rows-1}` по две точки (`a`, `b`)."""

    return [
        [f"r{k}a", f"r{k}b", f"r{k + 1}b", f"r{k + 1}a"] for k in range(rows - 1)
    ]


def strip_points(rings) -> dict:
    """Точки полосы: `rings[k] = (точка_a, точка_b)`."""

    points = {}
    for k, (first, second) in enumerate(rings):
        points[f"r{k}a"] = first
        points[f"r{k}b"] = second
    return points


def fold_strip(width=1.0):
    """Ступень: плоский квад, вертикальный квад (складка 90°), плоский квад."""

    rings = [
        ((0.0, 0.0, 0.0), (0.0, width, 0.0)),
        ((1.0, 0.0, 0.0), (1.0, width, 0.0)),
        ((1.0, 0.0, 1.0), (1.0, width, 1.0)),
        ((2.0, 0.0, 1.0), (2.0, width, 1.0)),
    ]
    return surface(strip_points(rings), strip_cycles(len(rings)))


def bevel_strip(segments: int, *, step_degrees=15.0, length=1.0, width=1.0):
    """Фаска: `segments` плоских квадов, каждый повёрнут относительно предыдущего на `step`."""

    rings = [((0.0, 0.0, 0.0), (0.0, width, 0.0))]
    x = z = 0.0
    angle = 0.0
    for _ in range(segments + 1):
        x += length * math.cos(angle)
        z += length * math.sin(angle)
        rings.append(((x, 0.0, z), (x, width, z)))
        angle += math.radians(step_degrees)
    return surface(strip_points(rings), strip_cycles(len(rings)))


def quarter_cylinder(segments=16, radius=1.0, height=1.0):
    """Четверть цилиндра: `segments` плоских квадов по дуге, образующая вдоль `y`."""

    rings = []
    for k in range(segments + 1):
        theta = (math.pi / 2) * k / segments
        x, z = radius * math.sin(theta), radius * (1.0 - math.cos(theta))
        rings.append(((x, 0.0, z), (x, height, z)))
    return surface(strip_points(rings), strip_cycles(len(rings)))


def cone(sides=8, radius=1.0, rise=0.5, *, boundary_apex: bool):
    """Конус: веер треугольников вокруг вершины `apex`.

    `boundary_apex=True` — веер РАЗОМКНУТ (сектор в 270°, вершина на границе
    домена); иначе замкнут (вершина внутри, полная сумма углов меньше `2π`).
    """

    points = {"apex": (0.0, 0.0, rise)}
    sweep = 1.5 * math.pi if boundary_apex else 2.0 * math.pi
    count = sides + 1 if boundary_apex else sides
    for k in range(count):
        theta = sweep * k / sides
        points[f"b{k}"] = (radius * math.cos(theta), radius * math.sin(theta), 0.0)
    cycles = [["apex", f"b{k}", f"b{(k + 1) % count}"] for k in range(sides)]
    return surface(points, cycles)


def fold_grid(rows=4):
    """Ступень сеткой: колонки `x = 0, 1` плоско, `x = 1` вверх (складка 90°), `x = 2` плоско.

    У вершин колонок складки есть замкнутые веера: сумма углов `2π` ТОЧНО, хотя веер
    не плоский.
    """

    columns = [
        lambda y: (0.0, y, 0.0),
        lambda y: (1.0, y, 0.0),
        lambda y: (1.0, y, 1.0),
        lambda y: (2.0, y, 1.0),
    ]
    points = {
        f"g{i}_{j}": columns[i](float(j)) for i in range(4) for j in range(rows + 1)
    }
    cycles = [
        [f"g{i}_{j}", f"g{i + 1}_{j}", f"g{i + 1}_{j + 1}", f"g{i}_{j + 1}"]
        for i in range(3)
        for j in range(rows)
    ]
    return surface(points, cycles)


def dome(rings=4, sides=8, radius=1.0, *, saddle=False):
    """Купол (или седло) над кругом: центр и кольца; вершины внутри — не развёртываются."""

    points = {"c": (0.0, 0.0, radius if not saddle else 0.0)}
    cycles = []
    for ring in range(1, rings + 1):
        phi = (math.pi / 2) * ring / rings
        for k in range(sides):
            theta = 2.0 * math.pi * k / sides
            rho = radius * math.sin(phi)
            height = radius * math.cos(phi)
            if saddle:
                height = 0.5 * rho * rho * math.cos(2.0 * theta)
            points[f"p{ring}_{k}"] = (rho * math.cos(theta), rho * math.sin(theta), height)
    for k in range(sides):
        cycles.append(["c", f"p1_{k}", f"p1_{(k + 1) % sides}"])
    for ring in range(1, rings):
        for k in range(sides):
            cycles.append(
                [
                    f"p{ring}_{k}",
                    f"p{ring + 1}_{k}",
                    f"p{ring + 1}_{(k + 1) % sides}",
                    f"p{ring}_{(k + 1) % sides}",
                ]
            )
    return surface(points, cycles)


def spiral_strip(steps=40, step_degrees=10.0, inner=1.0, outer=1.5, pitch=0.02):
    """Винтовая лента: радиальные кольца, поворот за шаг `step`, подъём `pitch` за оборот.

    Лента 400°: её развёртка (плоский кольцевой сектор) накрывает себя.
    """

    rings = []
    for k in range(steps + 1):
        theta = math.radians(step_degrees * k)
        z = pitch * theta / (2.0 * math.pi)
        rings.append(
            (
                (inner * math.cos(theta), inner * math.sin(theta), z),
                (outer * math.cos(theta), outer * math.sin(theta), z),
            )
        )
    return surface(strip_points(rings), strip_cycles(len(rings)))


def closed_cylinder(segments=8, radius=1.0, height=1.0):
    """Замкнутая колонна: кольцо квадов (последнее кольцо склеено с первым) - носитель-кольцо."""

    points = {}
    for k in range(segments):
        theta = 2.0 * math.pi * k / segments
        x, y = radius * math.cos(theta), radius * math.sin(theta)
        points[f"c{k}a"] = (x, y, 0.0)
        points[f"c{k}b"] = (x, y, height)
    cycles = [
        [f"c{k}a", f"c{(k + 1) % segments}a", f"c{(k + 1) % segments}b", f"c{k}b"]
        for k in range(segments)
    ]
    return surface(points, cycles)


def fold_fan(sides=5, radius=1.0):
    """Вершина `c` на складке: `sides` треугольников в плоскости и `sides` под углом 90°.

    Веер замкнут и НЕ плоский, сумма углов `2π` точно (складка прямая).
    """

    points = {"c": (0.0, 0.0, 0.0)}
    count = 2 * sides
    for k in range(count + 1):
        theta = math.pi * k / sides
        if k <= sides:
            points[f"p{k}"] = (radius * math.cos(theta), radius * math.sin(theta), 0.0)
        else:
            points[f"p{k}"] = (radius * math.cos(theta), 0.0, -radius * math.sin(theta))
    names = [f"p{k}" for k in range(count)]
    cycles = [["c", names[k], names[(k + 1) % count]] for k in range(count)]
    return surface(points, cycles)


def developable_chart(parts, *, grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1, **overrides):
    """Карта и сертификат развёртки НАПРЯМУЮ (без лестницы): привязка источника, затем развёртка."""

    from cftuv_envelope._developable import build_developable_chart
    from cftuv_envelope.planar_metric import _source_scope
    from cftuv_envelope.source_grid import resolve_source_grid

    vertices, faces, triangles = parts
    scope_faces, required_ids, positions = _source_scope(
        owner_patch_id=PATCH, source_vertices=vertices, source_faces=faces
    )
    grid = resolve_source_grid(
        positions=positions,
        faces=scope_faces,
        snapping_law=grid_policy,
        enforce_embedding=True,
    )
    options = dict(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        snapped=dict(grid.positions),
        owner_triangles=triangles,
        required_ids=required_ids,
        source_scale=grid.certificate.source_scale if grid.certificate.snapping_law.snaps_source else None,
    )
    options.update(overrides)
    return build_developable_chart(**options)

