"""Разрез отказанного патча Both на куски (BOTH_PATCH_SPLIT_V1): закон хорды, пара накрытия из текста отказа, отпечаток нажатия.

Меши аналитические, без Blender: полоса квадов (ровная и с узкой щелью от края), один треугольник (хорды нет).
"""

from __future__ import annotations

from cftuv import bevel_patch_split as split
from cftuv.bevel_faces import BOTH, BevelFacesPlanV1, scope_digest
from cftuv.bevel_faces_blender import FaceBuildV1
from cftuv.surface_ir import SourceRevision


def _strip(columns: int, rows: int = 2, width: float = 1.0):
    """Полоса `columns x rows` квадов на плоскости z = 0: вершина `(i, j)` - номер `i * (rows + 1) + j`."""

    coords = {i * (rows + 1) + j: (float(i), width * j / rows, 0.0) for i in range(columns + 1) for j in range(rows + 1)}
    faces = {}
    for i in range(columns):
        for j in range(rows):
            a = i * (rows + 1) + j
            faces[i * rows + j] = (a, a + rows + 1, a + rows + 2, a + 1)
    return faces, coords


def _parts(faces, cut):
    return sorted(len(part) for part in split._parts(faces, split._edge_faces(faces), frozenset(cut)))


def test_a_plain_strip_is_cut_across_in_the_middle_into_two_equal_parts():
    faces, coords = _strip(8)
    cut = split.patch_cut(faces, coords)
    assert cut == ((12, 13), (13, 14))  # колонка i = 4: вершины 12-13-14, поперёк полосы
    assert _parts(faces, cut) == [8, 8]


def test_the_cut_runs_from_the_bottom_of_a_slit_to_the_other_rail():
    """Щель от нижнего рельса (две грани у края вынуты) - узкая шейка: хорда идёт от её дна к другому рельсу, а не посередине."""

    faces, coords = _strip(9, rows=3)
    for face_id in (4 * 3, 5 * 3):  # грани (i=4, j=0) и (i=5, j=0): щель шириной в две грани и глубиной в одну
        del faces[face_id]
    coords = {vertex: coords[vertex] for cycle in faces.values() for vertex in cycle}
    cut = split.patch_cut(faces, coords)
    vertices = {vertex for edge in cut for vertex in edge}
    assert len(cut) == 2 and {vertex % 4 for vertex in vertices} == {1, 2, 3}  # от дна щели (j = 1) до рельса j = 3
    assert vertices & {17, 21, 25}  # дно щели: вершины (4..6, 1)
    assert sum(_parts(faces, cut)) == len(faces) and len(_parts(faces, cut)) == 2


def test_a_named_overlap_pair_is_put_into_different_pieces():
    faces, coords = _strip(8)
    left_rail, right_rail = (0, 3), (24, 27)  # нижние рёбра у двух концов: в одном куске после хорды посередине они не останутся
    near = ((0, 3), (3, 6))  # два соседних нижних ребра у левого конца: развести их можно только хордой между ними
    cut = split.patch_cut(faces, coords, near)
    assert cut == ((3, 4), (4, 5))
    assert split.patch_cut(faces, coords, (left_rail, right_rail)) == split.patch_cut(faces, coords)
    # пара, которой нет на границе патча, не мешает закону
    assert split.patch_cut(faces, coords, ((100, 101), (102, 103))) == split.patch_cut(faces, coords)


def test_the_cut_does_not_depend_on_the_order_of_the_faces():
    faces, coords = _strip(7, rows=3)
    shuffled = dict(sorted(faces.items(), reverse=True))
    assert split.patch_cut(faces, coords) == split.patch_cut(shuffled, dict(sorted(coords.items(), reverse=True)))


def test_a_patch_without_two_separable_boundary_vertices_has_no_cut():
    assert split.patch_cut({0: (0, 1, 2)}, {0: (0.0, 0.0, 0.0), 1: (1.0, 0.0, 0.0), 2: (0.0, 1.0, 0.0)}) == ()


def test_the_overlap_pair_is_read_from_the_kernel_text_even_with_colons_and_spaces_in_the_name():
    name = "host-vertex:host-source:abc:my wall: v2"
    detail = (
        "the plane-start ARAP unfolding covers itself before any snapping: 4 boundary edge pairs of the chart meet or overlap "
        f"(first: {name}:148->{name}:153 vs {name}:370->{name}:155) [ARAP_FROM_PLANE ...]"
    )
    assert split.overlap_pair(detail) == ((148, 153), (155, 370))
    assert split.overlap_pair("developable stretch: worst_band_squared<=2.4") is None
    assert split.overlap_pair("") is None


def test_the_summary_names_every_count():
    assert split.split_summary(1, 3, 2, {}, 0) == "BOTH_PATCH_SPLIT 1 -> 3 pieces (2 rounds)"
    assert split.split_summary(2, 5, 4, {"DEVELOPABLE_CHART_SELF_OVERLAP": 1}, 1) == (
        "BOTH_PATCH_SPLIT 2 -> 5 pieces (4 rounds), BOTH_PATCH_SPLIT_PIECE_REFUSED DEVELOPABLE_CHART_SELF_OVERLAP 1, BOTH_PATCH_SPLIT_NO_CUT 1"
    )
    assert split.SPLIT_TRIGGER_OUTCOMES == {"DEVELOPABLE_CHART_SELF_OVERLAP", "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED"}


def test_cuts_enter_the_revision_of_the_press_and_only_add_up():
    plan = BevelFacesPlanV1(BOTH, frozenset({9}), (1, 2, 3), outward_sources=frozenset({5}))
    mesh = SourceRevision(source_name="wall", digest="m" * 64)
    build = FaceBuildV1(plan, mesh, SourceRevision(source_name="wall", digest=scope_digest(mesh.digest, plan)))
    assert build.with_cuts(()).revision == build.revision
    once = build.with_cuts({7, 8})
    assert once.plan.cut_edges == {7, 8} and once.revision != build.revision and once.mesh_revision == mesh
    assert once.with_cuts({8, 11}).plan.cut_edges == {7, 8, 11}
    assert once.revision == build.with_cuts({8, 7}).revision  # отпечаток - от множества рёбер, не от порядка раундов
    assert once.plan.outward_sources == plan.outward_sources  # рёбра разреза не источники


def test_a_named_face_is_cut_off_along_its_edges_shared_with_the_patch():
    """BOTH_PATCH_ISOLATE_V1: шов - рёбра названной грани, общие с другими гранями патча; её рёбра на границе патча не режутся."""

    faces, _coords = _strip(4)  # грани 0..7, колонка i: грани 2i (низ) и 2i+1 (верх)
    cut = split.isolating_cut(faces, {2})  # грань (i=1, j=0): соседи - грань 0 слева, 4 справа, 3 сверху
    assert cut == ((3, 4), (4, 7), (6, 7))
    assert _parts(faces, cut) == [1, 7]
    assert split.isolating_cut(faces, {2, 3}) == ((3, 4), (4, 5), (6, 7), (7, 8))  # две соседние названные грани - один кусок
    assert split.isolating_cut({0: faces[0]}, {0}) == ()  # патч из одной названной грани: отделять нечего
    assert split.isolating_cut(faces, {100}) == ()  # грани нет в патче


def test_the_named_faces_are_read_from_both_refusals():
    crossing = (
        "SOURCE_FACE_SELF_INTERSECTION: face 123 crosses itself, its edges 87-208 and 210-157 pass 0 µm apart — rebuild the face"
        " (patch 2: 2 such faces: 123, 140)\nrecord: patch 2, extent 4.3 m, contact gap 1e-06 m; 0 T-vertices, 2 face self-intersections"
        "\n  face 123: edges 87-208 and 210-157 pass 0 m apart\n  face 140: edges 1-2 and 3-4 pass 0 m apart"
    )
    assert split.named_faces("SOURCE_FACE_SELF_INTERSECTION", crossing, {}) == {123, 140}
    name = "host-triangle:host-source:abc:my wall"
    opposed = (
        f"the offset normal of vertex host-vertex:host-source:abc:my wall:120 opposes the normal of its triangle {name}:133 (depth bound "
        f"0.00328 m at the reference offset 0.02 m against the tolerance 0.0001 m) [SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1: refused again ... "
        f"its triangle {name}:133 (depth ...)]"
    )
    assert split.named_faces("SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", opposed, {133: 167, 9: 1}) == {167}
    assert split.named_faces("SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", opposed, {}) == frozenset()  # треугольника нет в пакете
    assert split.named_faces("DEVELOPABLE_CHART_SELF_OVERLAP", crossing, {}) == frozenset()
    assert split.SPLIT_ISOLATE_OUTCOMES == {"SOURCE_FACE_SELF_INTERSECTION", "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE"}
