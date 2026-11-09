"""Копия граней бевела декали Both (BEVEL_FACES_BOTH_V2): топология, смещение, UV в метрах и домен сборки меша без Blender.

Меши - аналитические: куб с фаской в один сегмент (полосы и треугольные узлы), сгиб с профилем в два сегмента (одна колонка), плоское
кольцо из двух рядов вокруг отверстия (кривая полоса, замкнутая и разомкнутая). Каждое решение - число с именем.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from cftuv import bevel_copy as bc
from cftuv import bevel_faces as bf

from test_bevel_faces import _cube_view, _fold, _view


def _area(points) -> float:
    return 0.5 * sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points)))


def _corners(spec):
    """`[(кольцо вершин исходника, UV углов)]` по граням копии."""

    found, at = [], 0
    for ring in spec.faces:
        found.append((tuple(spec.vertex_ids[local] for local in ring), spec.uvs[at:at + len(ring)]))
        at += len(ring)
    return found


def _split(strip=frozenset(), node=frozenset()):
    return bf.BevelSplitV1(frozenset(strip), frozenset(node), frozenset(), "TEST")


def _annulus(columns, closed, r_in=1.0, width=0.2):
    """Плоское кольцо (z = 0): ряд внутри, два ряда полосы (выделены), ряд снаружи; `closed=False` - сектор в полкруга."""

    radii = (r_in - 0.3, r_in, r_in + width / 2, r_in + width, r_in + width + 0.3)
    count = columns if closed else columns + 1
    span = 2 * math.pi if closed else math.pi
    coords = {}
    for row, radius in enumerate(radii):
        for column in range(count):
            angle = span * column / columns
            coords[row * count + column] = (radius * math.cos(angle), radius * math.sin(angle), 0.0)
    faces, selected = {}, set()
    for row in range(len(radii) - 1):
        for column in range(columns):
            nxt = (column + 1) % count
            face = row * columns + column
            faces[face] = (row * count + column, row * count + nxt, (row + 1) * count + nxt, (row + 1) * count + column)
            if row in (1, 2):
                selected.add(face)
    return coords, faces, selected


def test_the_copy_of_the_cube_selection_is_its_topology_with_unit_area_weighted_normals():
    view, kinds = _cube_view()
    split = bf.split_bevel_faces(view)
    spec = bc.copy_spec(view, split)
    selected = sorted(face for face, kind in kinds.items() if kind != "flat")
    assert spec.face_ids == tuple(selected)
    assert [ring for ring, _uv in _corners(spec)] == [view.faces[face] for face in selected], "same vertices, same winding"
    assert spec.positions == tuple(view.coords[vertex] for vertex in spec.vertex_ids)
    assert all(abs(math.sqrt(sum(value * value for value in normal)) - 1.0) <= 1e-12 for normal in spec.normals)
    corner = spec.vertex_ids.index(view.faces[selected[-1]][0])  # вершина узла: средняя нормаль трёх граней (полосы и угол)
    assert all(value > 0.0 for value in (abs(item) for item in spec.normals[corner]))
    assert len(spec.boundary) == 24, "every copy vertex touches a flat face"
    assert len(set(spec.owners)) == 12 and dict(spec.events) == {bc.BEVEL_COPY_NODE_CONTINUES_STRIP: 8}
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec)), "the copy is counter-clockwise in UV"
    assert spec.seams, "faces of different frames do not share UV"


def test_a_profile_column_gets_the_half_width_across_and_one_along_per_rung():
    coords, faces, selected = _fold(2)
    view = _view(coords, faces, selected)
    spec = bc.copy_spec(view, _split(strip=selected))
    width = sum(math.dist(coords[2 * row], coords[2 * row + 2]) for row in range(1, 3))
    values = {}
    for ring, uv in _corners(spec):
        for vertex, (u, s) in zip(ring, uv):
            values.setdefault(vertex, set()).add((round(u, 12), round(s, 12)))
    assert all(len(item) == 1 for item in values.values()), "one frame: a vertex has one UV"
    across = {vertex: next(iter(item))[1] for vertex, item in values.items()}
    along = {vertex: next(iter(item))[0] for vertex, item in values.items()}
    assert sorted({abs(round(across[vertex], 9)) for vertex in across}) == [0.0, round(width / 2, 9)]
    assert across[2] == -across[6] and abs(across[2]) == pytest.approx(width / 2) and across[4] == 0.0
    assert {along[vertex] for vertex in (2, 4, 6)} == {along[2]} and abs(along[3] - along[2]) == pytest.approx(1.0)
    assert not spec.seams and dict(spec.events) == {}


@pytest.mark.parametrize("closed", [False, True])
def test_a_curved_strip_has_one_along_per_rung_from_the_centre_line(closed):
    columns = 12
    coords, faces, selected = _annulus(columns, closed)
    spec = bc.copy_spec(_view(coords, faces, selected), _split(strip=selected))
    centre = 1.1 * 2 * math.sin((2 * math.pi if closed else math.pi) / columns / 2)
    rows = {}
    for ring, uv in _corners(spec):
        for vertex, (u, s) in zip(ring, uv):
            rows.setdefault(round(u, 9), set()).add(round(s, 9))
    us = sorted(rows)
    assert all(values == {-0.1, 0.0, 0.1} for values in rows.values()), "the two rails mirror at the same U"
    assert [b - a for a, b in zip(us, us[1:])] == pytest.approx([centre] * (len(us) - 1), abs=1e-8)
    events = dict(spec.events)
    if closed:
        assert events == {bc.BEVEL_COPY_RING: 1} and len(us) == columns + 1 and spec.seams, "the ring closes with a UV seam"
    else:
        assert events == {} and len(us) == columns + 1 and not spec.seams
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec))


def test_nodes_without_a_strip_are_projected_by_name():
    view, kinds = _cube_view()
    nodes = {face for face, kind in kinds.items() if kind == "node"}
    spec = bc.copy_spec(_view(view.coords, view.faces, nodes), _split(node=nodes))
    assert dict(spec.events) == {bc.BEVEL_COPY_NODE_NO_STRIP: 8} and len(set(spec.owners)) == 8
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec))


def test_the_copy_domain_names_the_kernel_source_vertices_and_travels_as_view_bytes():
    from cftuv.envelope_view_bytes import decode_view

    view, _kinds = _cube_view()
    spec = bc.copy_spec(view, bf.split_bevel_faces(view))
    revision = SimpleNamespace(source_name="cube", digest="d" * 64)
    result = bc.copy_result(spec, 6, revision)
    assert result.patch_id == 6 and result.is_materialized and result.domain_id.startswith("bevel-copy:" + "d" * 16)
    assert result.view.vertices.refs[0] == f"location:src:host-vertex:host-source:{'d' * 64}:cube:{spec.vertex_ids[0]}"
    decoded = decode_view(result.view_bytes)
    assert decoded.faces == result.view.faces and decoded.uvs == result.view.uvs and decoded.owners == result.view.owners
    assert decoded.vertices.positions == spec.positions and decoded.vertices.refs == result.view.vertices.refs and decoded.token == result.view.token


def test_the_registry_finds_the_copy_of_the_press_and_counts_unpaired_rail_vertices():
    view, _kinds = _cube_view()
    spec = bc.copy_spec(view, bf.split_bevel_faces(view))
    result = bc.copy_result(spec, 6, SimpleNamespace(source_name="cube", digest="e" * 64))
    bc.note_copy(result, ("a", "b"))
    assert bc.copy_for([SimpleNamespace(domain_id="b")]) is result
    assert bc.copy_for([SimpleNamespace(domain_id="z")]) is None
    count = len(spec.vertex_ids)
    shared = list(range(count))
    shared[0] = 999  # первая вершина копии без пары снаружи
    arrays = SimpleNamespace(bevel_copy=result, domain_keys=((0, "a"), (6, result.domain_id)), buffers=None,
                             domain_vertex_index=(tuple(range(1, count)), tuple(shared)))
    assert bc.rail_unpaired(arrays) == (1 if 0 in spec.boundary else 0)
    assert bc.rail_unpaired(SimpleNamespace(bevel_copy=None)) == 0
