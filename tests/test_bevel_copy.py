"""Копия граней бевела декали Both (BEVEL_FACES_BOTH_V3): топология, смещение, UV сегмента в метрах, присоединение узла, деление рёбер рельса
и домен сборки меша без Blender.

Меши - аналитические: куб с фаской в один сегмент (полосы и треугольные узлы), сгиб с профилем в два сегмента (одна колонка), плоское
кольцо из двух рядов вокруг отверстия (кривая полоса, замкнутая и разомкнутая), плоская лента «узел | полоса | узел | полоса | узел»
(узел между полосами разной длины) и стена-колонка, у которой внешняя грань ядра одна на всю высоту, а рельс копии - с промежуточными
вершинами (T-стык). Каждое решение - число с именем.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

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
    segments = [owner // bc.OWNER_SHIFT for owner in spec.owners]
    nodes = {face for face, owner in zip(spec.face_ids, spec.owners) if owner % bc.OWNER_SHIFT == bc.OWNER_NODE}
    assert len(set(segments)) == 12 and nodes == {face for face, kind in kinds.items() if kind == "node"}
    assert dict(spec.events) == {bc.BEVEL_COPY_NODE_CONTINUES_STRIP: 8, bc.BEVEL_COPY_SEGMENTS: 12}
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec)), "the copy is counter-clockwise in UV"
    assert spec.seams, "faces of different segments do not share UV"
    of = dict(zip(spec.face_ids, segments))
    table = {}
    for face in spec.face_ids:
        for at in range(len(view.faces[face])):
            table.setdefault(frozenset((view.faces[face][at], view.faces[face][(at + 1) % len(view.faces[face])])), []).append(face)
    for node in nodes:  # BEVEL_NODE_ATTACH_MOST_EDGES_LONGEST_V3: три полосы по ребру, длины равны - меньший номер сегмента
        touching = {of[other] for faces in table.values() if node in faces for other in faces if other != node}
        assert of[node] == min(touching), (node, touching)


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
    assert not spec.seams and dict(spec.events) == {bc.BEVEL_COPY_SEGMENTS: 1}


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
        assert events == {bc.BEVEL_COPY_RING: 1, bc.BEVEL_COPY_SEGMENTS: 1} and len(us) == columns + 1 and spec.seams, "the ring closes with a UV seam"
    else:
        assert events == {bc.BEVEL_COPY_SEGMENTS: 1} and len(us) == columns + 1 and not spec.seams
    assert us[0] == 0.0, "U starts at the first rung"
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec))


def test_nodes_without_a_strip_are_projected_by_name():
    view, kinds = _cube_view()
    nodes = {face for face, kind in kinds.items() if kind == "node"}
    spec = bc.copy_spec(_view(view.coords, view.faces, nodes), _split(node=nodes))
    assert dict(spec.events) == {bc.BEVEL_COPY_NODE_NO_STRIP: 8, bc.BEVEL_COPY_SEGMENTS: 8} and len(set(spec.owners)) == 8
    assert all(owner % bc.OWNER_SHIFT == bc.OWNER_NODE for owner in spec.owners)
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
    outer = [tuple(range(1, count))]  # грань снаружи на всех вершинах копии, кроме первой
    arrays = SimpleNamespace(bevel_copy=result, domain_keys=((0, "a"), (6, result.domain_id)), buffers=None,
                             domain_vertex_index=(tuple(range(1, count)), tuple(shared)), faces=(*outer, *spec.faces), face_domain=(0, *([6] * len(spec.faces))))
    assert bc.rail_unpaired(arrays) == (1 if 0 in spec.boundary else 0)
    assert bc.rail_unpaired(SimpleNamespace(bevel_copy=None)) == 0


def _ribbon(lengths=(1, 3, 1, 1, 1)):
    """Плоская лента (z = 0) вдоль x: ряд под ней и над ней не выделен, средний ряд - узел | полоса A | узел | полоса B | узел по колонкам."""

    xs = [0.0]
    for width in lengths:
        xs.append(xs[-1] + float(width))
    count = len(xs)
    coords = {row * count + column: (x, float(row), 0.0) for row in range(4) for column, x in enumerate(xs)}
    faces, kinds = {}, {}
    for row in range(3):
        for column in range(count - 1):
            face = len(faces)
            faces[face] = (row * count + column, row * count + column + 1, (row + 1) * count + column + 1, (row + 1) * count + column)
            kinds[face] = "flat" if row != 1 else ("node" if column % 2 == 0 else "strip")
    selected = {face for face, kind in kinds.items() if kind != "flat"}
    return coords, faces, selected, kinds


def test_a_node_between_strips_attaches_whole_to_the_longer_strip_and_u_runs_up_the_world_axis():
    coords, faces, selected, kinds = _ribbon()
    split = _split(strip={face for face in selected if kinds[face] == "strip"}, node={face for face in selected if kinds[face] == "node"})
    spec = bc.copy_spec(_view(coords, faces, selected), split)
    owner = dict(zip(spec.face_ids, spec.owners))
    strip_a, middle, strip_b = 6, 7, 8  # средний ряд: узел 5, полоса A 6 (длина 3), узел 7, полоса B 8 (длина 1), узел 9
    assert owner[middle] == owner[strip_a] + bc.OWNER_NODE, "the node between A and B (one edge each) goes to the longer strip A"
    assert owner[5] == owner[strip_a] + bc.OWNER_NODE and owner[9] == owner[strip_b] + bc.OWNER_NODE
    assert owner[strip_a] != owner[strip_b] and dict(spec.events)[bc.BEVEL_COPY_SEGMENTS] == 2
    values = {}
    for (ring, uv), face in zip(_corners(spec), spec.face_ids):
        for vertex, (u, s) in zip(ring, uv):
            values.setdefault((face, vertex), (u, s))
    for face in (strip_a, strip_b):
        us = {vertex: values[(face, vertex)][0] for vertex in faces[face]}
        low = min(us, key=us.get)
        assert us[low] == 0.0 and all((coords[vertex][0] - coords[low][0]) == pytest.approx(us[vertex]) for vertex in us), "U = arc length up +x from 0"
    ring = faces[middle]  # узел продолжает U полосы A за её конец
    assert {round(values[(middle, vertex)][0], 9) for vertex in ring} == {3.0, 4.0}
    assert all(_area(uv) > 0.0 for _ring, uv in _corners(spec))


def _column(rows=3):
    """Стена-колонка: полоса копии из `rows` квадов вдоль z (x от 0 до 1), справа грань ядра снаружи одна на всю высоту (x от 1 до 2):
    рельс копии x = 1 с промежуточными вершинами, у грани ядра - только концы. Плоскость y = 0, нормаль -y."""

    coords = {}
    for row in range(rows + 1):
        coords[row * 2] = (0.0, 0.0, float(row))
        coords[row * 2 + 1] = (1.0, 0.0, float(row))
    faces = {row: (row * 2, row * 2 + 1, row * 2 + 3, row * 2 + 2) for row in range(rows)}
    outer = rows
    top, bottom = rows * 2 + 1, 1
    coords[100], coords[101] = (2.0, 0.0, 0.0), (2.0, 0.0, float(rows))
    faces[outer] = (bottom, 100, 101, *[row * 2 + 1 for row in range(rows, 0, -1)])
    selected = set(range(rows))
    return coords, faces, selected, outer, top, bottom


@dataclass(frozen=True)
class _Arrays:
    positions: tuple
    faces: tuple
    uvs: tuple
    face_domain: tuple
    face_owner: tuple
    seam_edges: tuple
    domain_keys: tuple
    domain_vertex_index: tuple
    domain_loop_counts: tuple
    domain_wall_splits: tuple
    weld_counters: tuple
    digest: str
    offset: float
    bevel_copy: object = None
    buffers: object = None


def _column_arrays(offset=0.1):
    """Массивы меша Both колонки: домен 0 - грань ядра на концах рельса (кольцо без промежуточных вершин), домен 7 - копия."""

    rows = 3
    coords, faces, selected, _outer, top, bottom = _column(rows)
    spec = bc.copy_spec(_view(coords, faces, selected), _split(strip=selected))
    result = bc.copy_result(spec, 7, SimpleNamespace(source_name="wall", digest="f" * 64))
    mesh_of = {vertex: number for number, vertex in enumerate(spec.vertex_ids)}
    outer_ring = (mesh_of[bottom], 8, 9, mesh_of[top])
    positions = [tuple(spec.positions[local][axis] - offset * (axis == 1) for axis in range(3)) for local in range(len(spec.vertex_ids))]
    positions += [(2.0, -offset, 0.0), (2.0, -offset, float(rows))]
    outer_uv = ((0.0, 0.0), (0.0, 1.0), (3.0, 1.0), (3.0, 0.0))
    copy_faces = tuple(spec.faces)
    arrays = _Arrays(
        positions=tuple(positions), faces=(outer_ring, *copy_faces), uvs=(*outer_uv, *spec.uvs), face_domain=(0, *([7] * len(copy_faces))),
        face_owner=(0, *spec.owners), seam_edges=(), domain_keys=((0, "outer"), (7, result.domain_id)),
        domain_vertex_index=((mesh_of[bottom], 8, 9, mesh_of[top]), tuple(range(len(spec.vertex_ids)))), domain_loop_counts=(4, len(spec.uvs)),
        domain_wall_splits=((), ()), weld_counters=(("ADAPTER_WELD_GROUPS", 2),), digest="d", offset=offset, bevel_copy=result,
    )
    return arrays, spec, mesh_of


def test_a_one_face_wing_over_a_column_gets_the_rail_vertices_of_the_copy():
    arrays, spec, mesh_of = _column_arrays()
    assert bc.rail_unpaired(arrays) == 2, "the T-junction: two copy rail vertices have no outward face"
    split = bc.split_rails(arrays)
    middle = [mesh_of[5], mesh_of[3]]  # вершины рельса z = 2 и z = 1 по пути от верха к низу
    assert split.faces[0] == (mesh_of[1], 8, 9, mesh_of[7], *middle), "the wing edge carries the copy rail vertices in rail order"
    assert np.allclose(split.uvs[4:6], ((2.0, 0.0), (1.0, 0.0))), "the new loops share the edge's UV by length"
    assert split.domain_loop_counts == (6, len(spec.uvs)) and split.domain_wall_splits[0] == ((4, 3, 0, 1.0 / 3.0), (4, 3, 0, 2.0 / 3.0))
    assert bc.rail_unpaired(split) == 0 and dict(split.weld_counters)[bc.BEVEL_COPY_RAIL_SPLIT] == 2
    assert dict(split.weld_counters)[bc.BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK] == 0 and split.digest != arrays.digest and split.buffers is None
    for vertex in middle:  # нормали копии и грани снаружи совпадают (-y): митра - то же смещение
        assert np.allclose(split.positions[vertex], np.asarray(spec.positions[vertex]) + (0.0, -0.1, 0.0))
    assert bc.split_rails(split) is split, "nothing left to split: the same arrays"


def test_a_rail_vertex_off_the_wing_edge_is_refused_by_name():
    arrays, spec, mesh_of = _column_arrays()
    positions = list(spec.positions)
    positions[mesh_of[3]] = (1.0, 0.0, 3.5)  # вершина рельса вне отрезка концов: доли не по возрастанию
    bent = SimpleNamespace(faces=spec.faces, boundary=spec.boundary, positions=tuple(positions))
    entries, refused = bc.rail_splits(arrays.faces, arrays.face_domain, 7, arrays.domain_vertex_index[1], bent)
    assert entries == [] and refused == 1
