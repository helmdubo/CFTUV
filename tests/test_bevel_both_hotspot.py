"""BEVEL_FACES_BOTH_HOTSPOT_V1 без Blender: декаль Both под Hotspot - остров СЕГМЕНТ (копия бевела с юбками), одним куском в прямоугольнике метки.

Рамка сегмента (BEVEL_HOTSPOT_SEGMENT_FRAME_V1): `V` крыльев 0..1 - вся толщина метки снизу вверх, `U = 0` - левый край метки, `U` вправо,
единица `U` - толщина (изотропно), trim и corner - одна формула (у corner средняя линия бевела `V = 0.5` - средняя линия метки), только поворот;
Swap у corner - поворот на 180 градусов вокруг центра прямоугольника метки. Острова - сегменты (`segments` записи), якоря - центры граней
копии (BEVEL_SEGMENT_ISLAND_ANCHOR_COPY_V1), запись крыльев есть и под Hotspot. Путь Hotspot без Both - побитово прежний (дайджест).
"""

from __future__ import annotations

import hashlib
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cftuv import bevel_wings as bw
from cftuv import envelope_production_mesh, hotspot_blender
from cftuv.bevel_faces import BOTH
from cftuv.bevel_wings_blender import BEVEL_WINGS_OFF_UNDER_UNIFORM, press_record
from cftuv.envelope_request_policy import ENVELOPE_UV_POLICY_DIRECT_STRIP
from cftuv.hotspot_islands import (
    COUNTER_UNMATCHED,
    GROUP_DOMAIN_SCALE,
    NO_MARK,
    HotspotRecordV2,
    anchor_tolerance,
    carry_islands,
    chosen_islands,
    code_for,
    decode,
    face_groups,
    island_codes,
    map_islands,
    paired_corner_groups,
    record_to_json,
    remap_atlas,
    remember_islands,
    segment_points,
    strip_from_atlas,
    transfer_frame,
)
from cftuv.hotspot_mapping import HotspotMappingV1, segment_atlas_uvs, segment_frame
from cftuv.hotspot_markup import HotspotMarkV1
from test_bevel_wings import COPY, _strip
from test_hotspot_mesh import _Attribute, _Attributes, _Data, _Layer

ROOT = Path(__file__).resolve().parents[1]
UPS = ("+y", "-y", "+x", "-x")


def _marks(up="+y"):
    vertical = up in ("+x", "-x")  # «вверх» вдоль x - толщина метки по x, длина по y
    trim = HotspotMarkV1(0, "drips", "trim", 64, 0, 128, 512, up) if vertical else HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, up)
    corner = HotspotMarkV1(1, "edge", "corner", 256, 0, 96, 512, up, 12) if vertical else HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, up, 12)
    return trim, corner


def _mapping(mark, swap=False):
    return HotspotMappingV1("Sheet", mark, 1024, 512, swap)


def _pixels(mapping, uvs):
    return segment_atlas_uvs(uvs, mapping) * np.array([1024.0, 512.0])


def _signed_area(points) -> float:
    return 0.5 * sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points)))


def _inside(mark, pixels, tolerance=1e-9) -> bool:
    return bool(np.all((pixels[:, 0] >= mark.x - tolerance) & (pixels[:, 0] <= mark.x + mark.w + tolerance)
                       & (pixels[:, 1] >= mark.y - tolerance) & (pixels[:, 1] <= mark.y + mark.h + tolerance)))


@pytest.mark.parametrize("up", UPS)
@pytest.mark.parametrize("kind", (0, 1))
def test_the_segment_fills_the_mark_across_its_thickness_with_u_from_the_left_end_isotropic_and_never_mirrored(up, kind):
    mark = _marks(up)[kind]
    mapping = _mapping(mark)
    thickness = float(mark.thickness)
    pixels = _pixels(mapping, np.array([(0.0, 0.0), (0.0, 0.5), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0), (2.0, 0.5)]))
    bottom_left, centre_left = np.array(mapping.left_end(-thickness / 2.0)), np.array(mapping.left_end(0.0))
    assert np.allclose(pixels[0], bottom_left) and np.allclose(pixels[1], centre_left), "V = 0 - the bottom edge, V = 0.5 - the centre line"
    assert np.allclose(pixels[2], np.array(mapping.left_end(thickness / 2.0))), "V = 1 - the top edge"
    assert np.allclose(pixels[3] - pixels[0], np.array(mapping.right) * thickness), "U grows to the right, one unit of U is the thickness"
    assert np.isclose(np.linalg.norm(pixels[3] - pixels[0]), np.linalg.norm(pixels[2] - pixels[0])), "isotropic: a unit of U and V is one thickness"
    whole = np.array([(u, v) for u in np.linspace(0.0, mark.length / thickness, 9) for v in (0.0, 0.25, 0.5, 0.75, 1.0)])
    assert _inside(mark, _pixels(mapping, whole)), "V 0..1 and U along the mark length lie inside the mark rectangle"
    triangle = np.array([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])
    assert _signed_area(_pixels(mapping, triangle)) > 0.0, "a counter-clockwise segment face stays counter-clockwise: rotation only"


@pytest.mark.parametrize("up", UPS)
def test_swap_of_a_corner_turns_the_segment_half_a_turn_about_the_centre_line_and_a_trim_has_no_swap(up):
    trim, corner = _marks(up)
    points = np.array([(0.3, 0.1), (1.7, 0.9), (2.2, 0.5), (0.0, 0.0)])
    plain, swapped = _pixels(_mapping(corner), points), _pixels(_mapping(corner, swap=True), points)
    centre = np.array((corner.x + corner.w / 2.0, corner.y + corner.h / 2.0))
    assert np.allclose(swapped - centre, -(plain - centre)), "Swap wings: 180 degrees about the centre of the mark rectangle"
    assert _signed_area(_pixels(_mapping(corner, True), np.array([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]))) > 0.0, "a turn, never a mirror"
    assert decode(code_for(trim, True)) == (0, False)
    assert np.array_equal(_pixels(_mapping(trim, True), points), _pixels(_mapping(trim), points)), "a trim mark maps the segment one way"


@pytest.mark.parametrize("up", UPS)
@pytest.mark.parametrize("swap", (False, True))
def test_the_segment_frame_inverts_exactly(up, swap):
    mapping = _mapping(_marks(up)[1], swap)
    points = np.random.default_rng(3).uniform((-1.0, -0.2), (6.0, 1.2), size=(40, 2))
    assert np.max(np.abs(segment_frame(segment_atlas_uvs(points, mapping), mapping) - points)) <= 1e-12


class _Both:
    """Декаль Both из `test_bevel_wings._strip(split=2)`: полоса копии (домен COPY) в два сегмента и юбки ядра (домены 1 и 2) снаружи рельсов."""

    def __init__(self):
        self.uvs, self.loop_vertex, self.starts, self.domains, self.owners, _points, self.positions = _strip(split=2)
        self.totals = np.diff(self.starts)
        self.wings, _numbers, self.plan = bw.wing_uvs(self.uvs, self.loop_vertex, self.starts, self.domains, self.owners, COPY, self.positions)
        kernel = face_groups(self.domains, self.owners)
        self.face_group = np.where(self.plan.face_segment >= 0, COPY * GROUP_DOMAIN_SCALE + self.plan.face_segment, kernel)
        self.loop_group = np.repeat(self.face_group, self.totals)
        self.copy = self.domains == COPY


def _record(default, marks, segments=True, islands=()):
    return HotspotRecordV2("Sheet", 1024, 512, default, tuple(marks), islands, segments)


def test_both_segments_map_one_piece_each_into_its_own_mark_and_no_mark_keeps_the_wing_uvs():
    both = _Both()
    trim, corner = _marks()
    groups = np.unique(both.face_group)
    assert groups.tolist() == [COPY * GROUP_DOMAIN_SCALE, COPY * GROUP_DOMAIN_SCALE + 1], "an island is a segment: its copy and its skirts"
    codes = np.where(both.loop_group == groups[0], code_for(trim, False), code_for(corner, True))
    record = _record(code_for(trim, False), (trim, corner))
    atlas, counters = map_islands(both.wings, both.loop_vertex, both.loop_group, codes, record)
    first = both.loop_group == groups[0]
    assert np.array_equal(atlas[first], segment_atlas_uvs(both.wings[first], _mapping(trim)))
    assert np.array_equal(atlas[~first], segment_atlas_uvs(both.wings[~first], _mapping(corner, True)))
    assert "HOTSPOT_CORNER_SIDE_PAIRS" not in counters, "a segment carries both halves: no corner pairing of pChain sides"
    assert _inside(trim, atlas[first] * [1024.0, 512.0]) and _inside(corner, atlas[~first] * [1024.0, 512.0])
    for start, stop in zip(both.starts, both.starts[1:]):
        assert _signed_area(atlas[start:stop] * [1024.0, 512.0]) > 0.0, "no UV face is flipped"
    pairs = both.plan.pair_outer, both.plan.pair_copy
    assert np.array_equal(atlas[pairs[0]], atlas[pairs[1]]), "UV is continuous across the rail: the skirt corner is the copy corner"
    unmapped = _record(NO_MARK, ())
    assert np.array_equal(map_islands(both.wings, both.loop_vertex, both.loop_group, np.full(both.loop_group.shape, NO_MARK), unmapped)[0], both.wings)


def test_a_segment_starts_at_the_left_end_of_the_mark_from_its_smallest_u_whatever_the_origin_of_its_wing_u():
    """Узел, присоединённый к сегменту, продолжает `U` за первую перекладину (`U < 0`): остров всё равно от левого края метки, сдвиг `U` крыльев
    острова его места в метке не меняет."""

    both = _Both()
    trim, _corner = _marks()
    record = _record(code_for(trim, False), (trim,))
    codes = np.full(both.loop_group.shape, code_for(trim, False))
    atlas, _ = map_islands(both.wings, both.loop_vertex, both.loop_group, codes, record)
    moved = both.wings.copy()
    second = both.loop_group == COPY * GROUP_DOMAIN_SCALE + 1
    moved[second, 0] -= 0.7
    again, _ = map_islands(moved, both.loop_vertex, both.loop_group, codes, record)
    assert np.allclose(again, atlas, atol=1e-12)
    for group in np.unique(both.loop_group).tolist():
        rows = both.loop_group == group
        assert np.isclose(np.min(atlas[rows, 0]) * 1024.0, trim.x), "every segment island starts at the left end of its mark"


def test_apply_inverts_the_segment_frame_exactly_keeps_other_segments_bit_for_bit_and_never_expands_a_corner():
    both = _Both()
    trim, corner = _marks()
    record = _record(code_for(trim, False), (trim, corner))
    old = np.full(both.loop_group.shape, code_for(trim, False))
    atlas, _ = map_islands(both.wings, both.loop_vertex, both.loop_group, old, record)
    strip = strip_from_atlas(atlas, both.loop_group, old, record)
    assert np.max(np.abs(strip - both.wings)) <= 1e-12, "the segment frame inverts to the wing UVs"
    selected = np.zeros(both.face_group.shape, dtype=bool)
    selected[np.flatnonzero(both.copy)[0]] = True  # одна грань копии выделена - весь её сегмент, без соседей по шву
    picked, expanded = chosen_islands(both.face_group, selected, corner.kind, both.loop_vertex, both.loop_group, strip[:, 1], segments=True)
    assert picked.tolist() == [both.face_group[np.flatnonzero(both.copy)[0]]] and expanded == 0
    assert paired_corner_groups(both.loop_vertex, both.loop_group, old, strip[:, 1], record).shape == (0,)
    new = np.where(np.isin(both.loop_group, picked), code_for(corner, False), old)
    remapped, _ = remap_atlas(atlas, both.loop_vertex, both.loop_group, old, new, record, record)
    kept = ~np.isin(both.loop_group, picked)
    assert np.array_equal(remapped[kept], atlas[kept]), "the untouched segment keeps its UVs bit for bit"
    assert np.allclose(remapped[~kept], segment_atlas_uvs(both.wings[~kept], _mapping(corner)), atol=1e-12)
    moved = replace(trim, y=300)
    assert np.allclose(transfer_frame(_mapping(trim), _mapping(moved), atlas, segments=True), segment_atlas_uvs(both.wings, _mapping(moved)), atol=1e-12)


def test_a_segment_island_mark_survives_a_rebuild_at_another_width_with_new_face_order_and_owners():
    """BEVEL_SEGMENT_ISLAND_ANCHOR_COPY_V1: якоря - центры граней копии; юбки другой ширины, другой порядок граней и номера владельцев ядра."""

    both = _Both()
    trim, corner = _marks()
    record = _record(code_for(trim, False), (trim, corner))
    face_code = np.where(both.face_group == COPY * GROUP_DOMAIN_SCALE + 1, code_for(corner, True), code_for(trim, False))
    points, groups = segment_points(both.loop_vertex, both.totals, both.face_group, both.copy, both.positions)
    assert set(groups.tolist()) == set(np.unique(both.face_group).tolist())
    remembered = remember_islands(record, both.face_group, face_code, points, groups)
    assert [(island.domain, island.code) for island in remembered.islands] == [(COPY, code_for(corner, True))]
    wider = both.positions.copy()
    wider[:, 1] = np.where(wider[:, 1] < 0.0, wider[:, 1] * 1.7, np.where(wider[:, 1] > 0.6, 0.6 + (wider[:, 1] - 0.6) * 1.7, wider[:, 1]))
    order = np.arange(both.totals.shape[0])[::-1]
    loops = np.concatenate([np.arange(both.starts[face], both.starts[face + 1]) for face in order])
    vertex, totals, copy = both.loop_vertex[loops], both.totals[order], both.copy[order]
    group = both.face_group[order]  # грань юбки - группа своего сегмента: номера владельцев ядра (они несут ширину) в группу не входят
    new_points, new_groups = segment_points(vertex, totals, group, copy, wider)
    codes, counters = island_codes(remembered, group, None, new_points, new_groups, anchor_tolerance(wider))
    assert counters[COUNTER_UNMATCHED] == 0
    assert codes.tolist() == face_code[order].tolist(), "the whole segment keeps its mark: copy faces and skirts"


def test_island_marks_of_pchain_sides_and_of_segments_never_carry_into_each_other():
    trim, corner = _marks()
    from cftuv.hotspot_islands import IslandMarkV1

    old = _record(code_for(trim, False), (trim, corner), segments=False, islands=(IslandMarkV1(3, code_for(corner, False), ((0.0, 0.0, 0.0),)),))
    carried, dropped = carry_islands(old, replace(old, islands=(), segments=True))
    assert dropped == 1 and carried.islands == ()
    same, none_dropped = carry_islands(replace(old, segments=True), replace(old, islands=(), segments=True))
    assert none_dropped == 0 and len(same.islands) == 1
    assert record_to_json(replace(old, segments=True)) == record_to_json(old), "the island kind is the mesh's, not a field of the JSON"


def test_the_wing_record_is_made_under_hotspot_too_and_no_off_under_hotspot_path_is_left():
    face_build = SimpleNamespace(plan=SimpleNamespace(side=BOTH))
    record, outcome = press_record(face_build, 7, ENVELOPE_UV_POLICY_DIRECT_STRIP)
    assert outcome == "" and bw.wings_from_json(record).copy_domain == 7
    assert press_record(face_build, 7, "UV_UNIFORM_TRIM_V1") == (None, BEVEL_WINGS_OFF_UNDER_UNIFORM)
    for path in (ROOT / "cftuv").glob("*.py"):
        assert "BEVEL_WINGS_OFF_UNDER_HOTSPOT" not in path.read_text(encoding="utf-8"), (path.name, "Hotspot is the last UV stage: Both never turns off under it")


class _Edges:
    def __init__(self, pairs):
        self.pairs = np.asarray(pairs, dtype=np.int32)
        self.seam = np.zeros(self.pairs.shape[0], dtype=bool)

    def __len__(self):
        return self.pairs.shape[0]

    def foreach_get(self, _name, buffer):
        buffer[:] = self.pairs.reshape(-1)

    def foreach_set(self, name, values):
        assert name == "use_seam"
        self.seam = np.asarray(values, dtype=bool).copy()


class _BothMesh(dict):
    """Меш Blender в объёме писателя для декали Both: свойства ID, углы, грани, вершины, рёбра (швы), атрибуты граней и слой UV."""

    name = "Both.CFTUV_Decal"
    session_uid = 11

    def __init__(self, both: _Both):
        super().__init__()
        self.attributes = _Attributes(self)
        self.loops = list(both.loop_vertex)
        self.polygons = _Data(both.totals.astype(np.int32), 1)
        self.vertices = _Data(both.positions.astype(np.float32).reshape(-1), 3)
        following = np.arange(1, len(self.loops) + 1)
        following[both.starts[1:] - 1] = both.starts[:-1]
        pairs = {tuple(sorted((int(a), int(b)))) for a, b in zip(both.loop_vertex, both.loop_vertex[following])}
        self.edges = _Edges(sorted(pairs))
        self.attributes["UVMap"] = _Attribute(both.uvs.astype(np.float32).reshape(-1), "FLOAT2", "CORNER", 2)
        self.attributes[".corner_vert"] = _Attribute(both.loop_vertex.astype(np.int32), "INT", "CORNER")
        self.attributes["cftuv_domain"] = _Attribute(both.domains.astype(np.int32), "INT", "FACE")
        self.attributes["cftuv_owner"] = _Attribute(both.owners.astype(np.int32), "INT", "FACE")
        self.attributes["position"] = _Attribute(both.positions.astype(np.float32).reshape(-1), "FLOAT_VECTOR", "POINT", 3)
        self.uv_layers = {"UVMap": _Layer(self.attributes["UVMap"])}

    def uvs(self):
        return self.attributes["UVMap"].data.values.reshape(-1, 2).copy()


def test_the_writer_maps_the_wings_first_and_then_every_segment_into_its_mark_and_seams_stay_the_wing_seams():
    both = _Both()
    trim, corner = _marks()
    wings_only = _BothMesh(both)
    wings_only[bw.MESH_PROPERTY] = bw.wings_to_json(bw.WingsRecordV3(COPY))
    envelope_production_mesh._map_hotspot(wings_only)
    mesh = _BothMesh(both)
    mesh[bw.MESH_PROPERTY] = bw.wings_to_json(bw.WingsRecordV3(COPY))
    mesh[hotspot_blender.MESH_PROPERTY] = record_to_json(_record(code_for(corner, False), (trim, corner), segments=False))
    envelope_production_mesh._map_hotspot(mesh)
    segment = mesh.attributes[bw.SEGMENT_ATTRIBUTE].data.values
    assert segment.tolist() == both.plan.face_segment.tolist(), "the wings write the segment of every face"
    record, _error = hotspot_blender.mesh_record(mesh)
    assert record.segments, "a Both mesh reads its islands as segments"
    _vertices, _totals, group = hotspot_blender.mesh_arrays(mesh)
    assert np.array_equal(group, both.face_group)
    wing32 = wings_only.uvs()
    assert np.array_equal(mesh.uvs(), segment_atlas_uvs(wing32.astype(np.float64), _mapping(corner)).astype(np.float32)), "Hotspot maps the wing UVs"
    assert np.array_equal(mesh.edges.seam, wings_only.edges.seam) and mesh.edges.seam.any(), "the seams are the wing seams: where UV splits"
    assert np.array_equal(hotspot_blender.KERNEL_UVS[mesh.session_uid], wing32), "Apply maps from the wing UVs of the session"
    assert _inside(corner, mesh.uvs().astype(np.float64) * [1024.0, 512.0], 1e-3)


def test_the_hotspot_path_without_both_is_bit_for_bit_the_previous_one():
    """Острова - стороны pChain: дайджест `map_islands`, `strip_from_atlas`, `remap_atlas` и `chosen_islands` равен дайджесту main 77af6ef2."""

    faces = [
        ((0, 1, 11, 10), [(0.0, 0.0), (1.5, 0.0), (1.5, 1.0), (0.0, 1.0)], 3, 0),
        ((1, 2, 12, 11), [(1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0)], 3, 0),
        ((2, 1, 21, 22), [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], 5, 1),
        ((1, 0, 20, 21), [(1.0, 0.0), (2.5, 0.0), (2.5, 1.0), (1.0, 1.0)], 5, 1),
        ((40, 41, 51, 50), [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], 3, 4),
        ((41, 42, 52, 51), [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0)], 3, 4),
    ]
    loop_vertex = np.array([v for face in faces for v in face[0]], dtype=np.int64)
    uvs = np.array([uv for face in faces for uv in face[1]], dtype=np.float64)
    totals = np.array([4] * len(faces))
    face_group = face_groups([face[2] for face in faces], [face[3] for face in faces])
    loop_group = np.repeat(face_group, totals)
    digest = hashlib.sha256()
    for up in UPS:
        trim = HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, up)
        corner = HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, up, 12)
        for swap in (False, True):
            record = HotspotRecordV2("Sheet", 1024, 512, code_for(corner, swap), (trim, corner))
            for codes in ([code_for(corner, swap)] * 6, [code_for(trim, False)] * 2 + [code_for(corner, swap)] * 4, [code_for(trim, False)] * 6):
                loop_code = np.repeat(np.array(codes, dtype=np.int64), totals)
                atlas, counters = map_islands(uvs, loop_vertex, loop_group, loop_code, record)
                strip = strip_from_atlas(atlas, loop_group, loop_code, record)
                other = np.repeat(np.array([code_for(trim, False)] * 6, dtype=np.int64), totals)
                remapped, numbers = remap_atlas(atlas, loop_vertex, loop_group, loop_code, other, record, record)
                chosen, expanded = chosen_islands(face_group, np.array([1, 0, 0, 0, 0, 0], dtype=bool), "corner", loop_vertex, loop_group, strip[:, 1])
                for item in (atlas, strip, remapped, chosen):
                    digest.update(np.ascontiguousarray(item).tobytes())
                digest.update(repr(sorted(counters.items())).encode() + repr(sorted(numbers.items())).encode() + repr(expanded).encode())
    assert digest.hexdigest()[:16] == "91ffabd4572f1883"
