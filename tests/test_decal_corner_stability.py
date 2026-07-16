"""S1 Static Interior: стабильность внутренних швов при width drag.

Oracle: docs/decal_corner_bands.md §5.1. Внутренние швы обязаны лежать
на ширино-независимых опорных прямых и только удлиняться вдоль них;
переезжает исключительно фронтир покрытия. Грани с неизменной геометрией
обязаны сохранять UV (текстура не «плывёт» на разрешённых областях).

Fixture — силуэт стены с углами трёх band'ов: MITER ~131°, FAN ~62°
(острый пик), KITE ~111°, плюс CAP на концах цепочки.
"""

import pytest

pytest.importorskip("pyvoronoi")

from mathutils import Vector

from cftuv.decal_voronoi import (
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
)
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


_WIDTHS = (1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 14.0, 18.0, 24.0, 30.0)


def _ear_clip(points2d):
    """Наивный ear clipping простого полигона; порядок вершин любой."""

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    indices = list(range(len(points2d)))
    area = sum(
        cross((0.0, 0.0), points2d[i], points2d[(i + 1) % len(points2d)])
        for i in range(len(points2d))
    )
    if area < 0.0:
        indices.reverse()
    tris = []
    guard = 0
    while len(indices) > 3 and guard < 10000:
        guard += 1
        for k in range(len(indices)):
            i0 = indices[(k - 1) % len(indices)]
            i1 = indices[k]
            i2 = indices[(k + 1) % len(indices)]
            a, b, c = points2d[i0], points2d[i1], points2d[i2]
            if cross(a, b, c) <= 1e-12:
                continue
            contains = False
            for j in indices:
                if j in (i0, i1, i2):
                    continue
                p = points2d[j]
                if (
                    cross(a, b, p) >= -1e-12
                    and cross(b, c, p) >= -1e-12
                    and cross(c, a, p) >= -1e-12
                ):
                    contains = True
                    break
            if contains:
                continue
            tris.append((i0, i1, i2))
            indices.pop(k)
            break
    tris.append(tuple(indices))
    return tris


def _silhouette_graph():
    pts2d = [
        (-12.0, 0.0),
        (12.0, 0.0),
        (12.0, 4.0),   # 2 = E (CAP)
        (4.0, 4.0),    # 3 = D (~111° KITE)
        (2.5, 8.0),    # 4 = C (пик ~62° FAN)
        (-1.0, 4.0),   # 5 = B (~131° MITER)
        (-12.0, 4.0),  # 6 = A (CAP)
    ]
    points = [Vector((x, y, 0.0)) for x, y in pts2d]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = _ear_clip(pts2d)
    chains = [([2, 3], 60), ([3, 4], 61), ([4, 5], 62), ([5, 6], 63)]
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=pair,
                    vert_cos=[points[pair[0]], points[pair[1]]],
                    edge_indices=[edge],
                )
                for pair, edge in chains
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, [60, 61, 62, 63]


def _vert_key(position):
    return (round(position.x, 5), round(position.y, 5), round(position.z, 5))


def _edge_key(a, b):
    ka, kb = _vert_key(a), _vert_key(b)
    return (ka, kb) if ka <= kb else (kb, ka)


def _line_key(a, b):
    """Опорная прямая ребра: quantized (unit normal, signed offset).

    Округления согласованы: normal 3 знака, offset 2 знака — рельсы,
    отстоящие на полшага width, различимы, а дрожание последнего знака
    одной и той же прямой не даёт ложных срабатываний.
    """

    dx, dy = b[0] - a[0], b[1] - a[1]
    length = (dx * dx + dy * dy) ** 0.5
    if length <= 1e-12:
        return None
    nx, ny = -dy / length, dx / length
    offset = nx * a[0] + ny * a[1]
    if nx < 0 or (abs(nx) < 1e-9 and ny < 0):
        nx, ny, offset = -nx, -ny, -offset
    return (round(nx, 3), round(ny, 3), round(offset, 2))


def _interior_edges(faces):
    counts = {}
    for face in faces:
        n = len(face.positions)
        for i in range(n):
            key = _edge_key(face.positions[i], face.positions[(i + 1) % n])
            counts[key] = counts.get(key, 0) + 1
    return {key for key, count in counts.items() if count >= 2}


def _face_signature(face):
    verts = tuple(sorted(_vert_key(position) for position in face.positions))
    return (face.component_kind, face.component_side, verts)


def _face_uv_payload(face, alpha):
    """UV-инварианты S1: латеральная дистанция u*alpha и v_length.

    U нормируется на текущую полуширину (тримовая семантика rails ±1),
    поэтому сравнивается не сам u_frac, а `u_frac * alpha`.
    """

    order = sorted(
        range(len(face.positions)),
        key=lambda index: _vert_key(face.positions[index]),
    )
    return tuple(
        (
            round(face.u_fracs[index] * alpha, 5),
            round(face.v_lengths[index], 6),
        )
        for index in order
    )


@pytest.fixture(scope="module")
def width_sweep():
    graph, edges = _silhouette_graph()
    plan = compile_patch_voronoi_plan(graph, edges, offset=0.01)
    return [
        (
            width,
            evaluate_patch_voronoi_plan(plan, width=width, preview=True),
        )
        for width in _WIDTHS
    ]


@pytest.mark.xfail(
    strict=True,
    reason=(
        "S1 нарушен: FAN/ACUTE границы якорены на alpha-точках и переезжают "
        "с шириной (задача A12); после переякорения снять маркер"
    ),
)
def test_s1_interior_supporting_lines_never_move(width_sweep):
    """Опорные прямые внутренних швов не исчезают при росте ширины."""

    previous_lines = None
    for width, faces in width_sweep:
        lines = {
            _line_key(a[:2], b[:2]) for a, b in _interior_edges(faces)
        } - {None}
        if previous_lines is not None:
            vanished = previous_lines - lines
            assert not vanished, (
                f"width {width}: опорные прямые внутренних швов переехали "
                f"(нарушение S1 §5.1): {sorted(vanished)}"
            )
        previous_lines = lines


@pytest.mark.xfail(
    strict=True,
    reason=(
        "S1 §5.1 п.4-5: грани в разрешённой области пересобираются при "
        "drag (задача A12); после переякорения снять маркер"
    ),
)
def test_s1_resolved_faces_keep_geometry_and_uv(width_sweep):
    """Грань, замершая геометрически, не меняет UV на следующем кадре.

    Прокси локальной заморозки: доля граней, доживших без изменений от
    кадра к кадру, должна монотонно приближаться к 1 задолго до полного
    насыщения; у совпавших граней UV обязаны быть идентичными.
    """

    for (w1, faces1), (w2, faces2) in zip(width_sweep, width_sweep[1:]):
        signatures1 = {
            _face_signature(face): _face_uv_payload(face, w1 * 0.5)
            for face in faces1
        }
        stable = 0
        for face in faces2:
            signature = _face_signature(face)
            if signature not in signatures1:
                continue
            stable += 1
            assert signatures1[signature] == _face_uv_payload(
                face, w2 * 0.5
            ), (
                f"width {w2}: UV поплыл на геометрически неизменной грани "
                f"{signature[:2]} (нарушение S1 §5.1 п.5)"
            )
        if w2 >= 6.0:
            # К этой ширине фронтир покинул окрестности всех углов
            # fixture; большинство граней обязаны дожить без изменений.
            assert stable >= len(faces2) // 2, (
                f"width {w2}: только {stable}/{len(faces2)} граней "
                f"стабильны — разрешённая область пересобирается"
            )


def test_interior_frozen_after_saturation(width_sweep):
    """После полного насыщения покрытия интерьер идентичен (guard).

    Этот инвариант выполняется уже сейчас и защищает strict runtime от
    регрессии при переякорении A12.
    """

    (wa, faces_a), (wb, faces_b) = width_sweep[-2], width_sweep[-1]
    assert _interior_edges(faces_a) == _interior_edges(faces_b)
    signatures_a = sorted(
        (_face_signature(face), _face_uv_payload(face, wa * 0.5))
        for face in faces_a
    )
    signatures_b = sorted(
        (_face_signature(face), _face_uv_payload(face, wb * 0.5))
        for face in faces_b
    )
    assert signatures_a == signatures_b
