"""HOTSPOT_ISLAND_MARKS_V1 без Blender: выделение -> острова (corner - с соседом по шву), запись v2 (и чтение v1), отображение по островам от
UV ядра (повтор не копит ошибку), правило пары corner и якоря шва, которые переживают пересборку граней с новыми номерами владельцев."""

from __future__ import annotations

import json

import numpy as np
import pytest

from cftuv.hotspot_islands import (
    COUNTER_DROPPED,
    COUNTER_MARK_MISSING,
    COUNTER_MIXED,
    COUNTER_OFF_DEFAULT,
    COUNTER_UNMATCHED,
    GROUP_DOMAIN_SCALE,
    HotspotRecordV2,
    IslandMarkV1,
    anchor_tolerance,
    assign,
    carry_islands,
    chosen_islands,
    code_for,
    decode,
    encode,
    face_groups,
    island_codes,
    NO_MARK,
    loop_next,
    map_islands,
    record_from_json,
    record_of,
    record_to_json,
    refresh_marks,
    paired_corner_groups,
    remap_atlas,
    remember_islands,
    seam_points,
    selection_text,
    strip_from_atlas,
    unmapped_record,
    unwarp_paired,
)
from cftuv.hotspot_mapping import COUNTER_PAIRS, COUNTER_UNPAIRED, HotspotMappingV1, atlas_uvs, mapping_to_json
from cftuv.hotspot_markup import HotspotMarkV1, HotspotMarkupV1

TRIM = HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y")
CORNER = HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, "+y", 12)
OTHER = HotspotMarkV1(2, "rust", "trim", 0, 384, 1024, 64, "+y")


def _record(default=TRIM, swap=False, islands=()):
    return HotspotRecordV2("Sheet", 1024, 512, code_for(default, swap), (TRIM, CORNER, OTHER), islands)


class _Decal:
    """Декаль из трёх островов: A (домен 3, владелец 0) и B (домен 5, владелец 1) - две стороны шва 0-1-2 (станции навстречу), C (домен 3,
    владелец 4) - полоса на своём шве 40-41-42 без пары. `width` масштабирует внешний ряд (позиции и `u` в единицах ширины), как живая ширина."""

    def __init__(self, width=1.0, owners=(0, 1, 4), order=None):
        a, b, c = owners
        faces = [
            ((0, 1, 11, 10), [(0.0, 0.0), (1.5, 0.0), (1.5, 1.0), (0.0, 1.0)], 3, a),
            ((1, 2, 12, 11), [(1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0)], 3, a),
            ((2, 1, 21, 22), [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], 5, b),
            ((1, 0, 20, 21), [(1.0, 0.0), (2.5, 0.0), (2.5, 1.0), (1.0, 1.0)], 5, b),
            ((40, 41, 51, 50), [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], 3, c),
            ((41, 42, 52, 51), [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0)], 3, c),
        ]
        order = range(len(faces)) if order is None else order
        faces = [faces[index] for index in order]
        self.loop_vertex = np.array([v for face in faces for v in face[0]], dtype=np.int64)
        self.uvs = np.array([uv for face in faces for uv in face[1]], dtype=np.float64) / np.array([width, 1.0])
        self.totals = np.array([len(face[0]) for face in faces], dtype=np.int64)
        self.face_group = face_groups([face[2] for face in faces], [face[3] for face in faces])
        self.loop_group = np.repeat(self.face_group, self.totals)
        positions = np.zeros((64, 3))
        stations = np.array([0.0, 0.6, 1.0])
        positions[0:3, 0] = stations  # шов A/B: от ширины не зависит
        positions[10:13] = np.stack((stations, np.full(3, width), np.zeros(3)), axis=1)  # внешний ряд A (пол)
        positions[20:23] = np.stack((stations, np.zeros(3), np.full(3, width)), axis=1)  # внешний ряд B (стена)
        positions[40:43] = np.stack((stations, np.full(3, -2.0), np.full(3, 0.5)), axis=1)  # шов C
        positions[50:53] = np.stack((stations, np.full(3, -2.0 - width), np.full(3, 0.5)), axis=1)
        self.positions = positions

    def points(self):
        return seam_points(self.uvs[:, 1], self.loop_vertex, loop_next(self.totals), self.loop_group, self.positions)

    def group(self, domain, owner):
        return domain * GROUP_DOMAIN_SCALE + owner


def test_codes_carry_mark_and_swap_and_a_trim_mark_has_no_wings():
    assert decode(encode(7, True)) == (7, True) and decode(encode(7, False)) == (7, False)
    assert code_for(TRIM, True) == code_for(TRIM, False) == encode(0, False)
    assert code_for(CORNER, True) == encode(1, True)


def test_the_v1_record_reads_as_v2_without_islands_and_v2_round_trips_with_islands():
    old = mapping_to_json(HotspotMappingV1("Sheet", CORNER, 1024, 512, True))
    record = record_from_json(old)
    assert (record.markup, record.width, record.height, decode(record.default), record.marks, record.islands) == ("Sheet", 1024, 512, (1, True), (CORNER,), ())
    island = IslandMarkV1(3, encode(1, True), ((0.1, 0.2, 0.3), (0.4, 0.5, 0.6)))
    full = _record(islands=(island,))
    again = record_from_json(record_to_json(full))
    assert again == full
    assert json.loads(record_to_json(full))["version"] == 2


@pytest.mark.parametrize(
    "payload",
    [
        "[]",
        json.dumps({"format": "CFTUV_HOTSPOT_MAPPING", "version": 3}),
        json.dumps({"format": "OTHER", "version": 2}),
        record_to_json(HotspotRecordV2("Sheet", 1024, 512, encode(9, False), (TRIM,))),
    ],
    ids=["not-a-dict", "unknown-version", "other-format", "default-not-in-table"],
)
def test_a_record_that_does_not_read_is_a_value_error(payload):
    with pytest.raises(ValueError):
        record_from_json(payload)


def test_a_face_selection_expands_to_its_island_and_a_corner_mark_to_the_seam_partner():
    decal = _Decal()
    one_face = np.zeros(decal.face_group.shape[0], dtype=bool)
    one_face[0] = True
    trim, added = chosen_islands(decal.face_group, one_face, "trim", decal.loop_vertex, decal.loop_group, decal.uvs[:, 1])
    assert trim.tolist() == [decal.group(3, 0)] and added == 0
    corner, added = chosen_islands(decal.face_group, one_face, "corner", decal.loop_vertex, decal.loop_group, decal.uvs[:, 1])
    assert corner.tolist() == [decal.group(3, 0), decal.group(5, 1)] and added == 1
    lonely = np.zeros_like(one_face)
    lonely[5] = True
    alone, added = chosen_islands(decal.face_group, lonely, "corner", decal.loop_vertex, decal.loop_group, decal.uvs[:, 1])
    assert alone.tolist() == [decal.group(3, 4)] and added == 0


def test_one_code_for_the_whole_mesh_is_the_single_mark_mapping_bit_for_bit():
    decal = _Decal()
    record = _record(CORNER)
    codes = np.full(decal.loop_group.shape[0], record.default)
    atlas, counters = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, codes, record)
    expected, numbers = atlas_uvs(decal.uvs, decal.loop_vertex, decal.loop_group, record.mapping(record.default))
    assert np.array_equal(atlas, expected) and counters[COUNTER_PAIRS] == numbers[COUNTER_PAIRS] == 1


def test_islands_map_into_their_own_marks_and_the_corner_pair_meets_on_the_seam():
    decal = _Decal()
    record = _record(TRIM)
    face_code = assign(np.full(decal.face_group.shape[0], record.default), decal.face_group, [decal.group(3, 0), decal.group(5, 1)], code_for(CORNER, False))
    loop_code = np.repeat(face_code, decal.totals)
    atlas, counters = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, loop_code, record)
    pixels = atlas * (1024, 512)
    c_rows = decal.loop_group == decal.group(3, 4)
    alone, _numbers = atlas_uvs(decal.uvs[c_rows], decal.loop_vertex[c_rows], decal.loop_group[c_rows], record.mapping(record.default))
    assert np.array_equal(atlas[c_rows], alone), "a trim island maps as it would alone"
    seam = decal.uvs[:, 1] == 0.0
    assert np.max(np.abs(pixels[seam & ~c_rows, 1] - 304.0)) <= 1e-9, "the corner pair has its seam on the centre line"
    for vertex in (0, 1, 2):
        here = atlas[seam & (decal.loop_vertex == vertex)]
        assert np.max(np.abs(here - here[0])) <= 1e-12
    assert counters[COUNTER_PAIRS] == 1 and counters[COUNTER_UNPAIRED] == 0 and counters[COUNTER_MARK_MISSING] == 0


def test_a_corner_side_whose_partner_has_another_mark_maps_unpaired_and_is_named():
    decal = _Decal()
    record = _record(TRIM)
    face_code = assign(np.full(decal.face_group.shape[0], record.default), decal.face_group, [decal.group(3, 0)], code_for(CORNER, False))
    _atlas, counters = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, np.repeat(face_code, decal.totals), record)
    assert counters[COUNTER_UNPAIRED] == 1 and counters[COUNTER_PAIRS] == 0


def test_a_code_whose_mark_left_the_record_takes_the_default_and_is_counted_per_island():
    decal = _Decal()
    record = _record(TRIM)
    face_code = assign(np.full(decal.face_group.shape[0], record.default), decal.face_group, [decal.group(3, 4)], encode(9, False))
    atlas, counters = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, np.repeat(face_code, decal.totals), record)
    expected, _numbers = atlas_uvs(decal.uvs, decal.loop_vertex, decal.loop_group, record.mapping(record.default))
    assert counters[COUNTER_MARK_MISSING] == 1 and np.array_equal(atlas, expected)


@pytest.mark.parametrize("up", ["+y", "-y", "+x", "-x"])
def test_a_trim_mark_inverts_exactly_back_to_the_kernel_strip_uv(up):
    decal = _Decal()
    mark = HotspotMarkV1(3, "side", "trim", 100, 0, 128, 512, up) if up in ("+x", "-x") else HotspotMarkV1(3, "side", "trim", 0, 100, 1024, 128, up)
    record = HotspotRecordV2("Sheet", 1024, 512, code_for(mark, False), (mark,))
    codes = np.full(decal.loop_group.shape[0], record.default)
    atlas, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, codes, record)
    assert np.max(np.abs(strip_from_atlas(atlas, decal.loop_group, codes, record) - decal.uvs)) <= 1e-12
    single = atlas.astype(np.float32)
    back = strip_from_atlas(single, decal.loop_group, codes, record)
    assert np.all(back[decal.uvs[:, 1] == 0.0, 1] == 0.0), "the float32 seam row snaps to v = 0"


def test_a_corner_mark_inverts_to_the_aligned_frame_with_the_wings_turned_back_and_the_seam_on_zero():
    decal = _Decal()
    for swap in (False, True):
        record = _record(CORNER, swap)
        codes = np.full(decal.loop_group.shape[0], record.default)
        atlas, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, codes, record)
        frame = strip_from_atlas(atlas.astype(np.float32), decal.loop_group, codes, record)
        assert np.max(np.abs(frame[:, 1] - decal.uvs[:, 1])) <= 1e-6, "v of both wings comes back (no mirror: the - wing is turned 180 degrees back)"
        again, _ = map_islands(frame, decal.loop_vertex, decal.loop_group, codes, record)
        residual = float(np.max(np.abs(again - atlas)))
        assert residual <= 1e-6, residual  # выравнивание corner на выровненном along - тождество до float32
        for group in np.unique(decal.loop_group):
            rows = decal.loop_group == group
            slope = np.polyfit(decal.uvs[rows, 0], frame[rows, 0], 1)[0]
            assert slope > 0.0, "the recovered along runs with the kernel u: the texture keeps its orientation"


def test_no_mark_is_the_kernel_strip_uv_and_an_unmapped_decal_maps_only_what_was_applied():
    decal = _Decal()
    record = unmapped_record("Sheet", 1024, 512).with_marks([OTHER])
    codes = np.full(decal.face_group.shape[0], NO_MARK)
    new_codes = assign(codes, decal.face_group, [decal.group(3, 4)], code_for(OTHER, False))
    kernel = decal.uvs.astype(np.float32)
    atlas, counters = remap_atlas(kernel, decal.loop_vertex, decal.loop_group, np.repeat(codes, decal.totals), np.repeat(new_codes, decal.totals), record, record)
    c_rows = decal.loop_group == decal.group(3, 4)
    assert np.array_equal(atlas[~c_rows], kernel[~c_rows]), "islands without a mark keep their kernel UV bit for bit"
    alone, _ = atlas_uvs(decal.uvs[c_rows], decal.loop_vertex[c_rows], decal.loop_group[c_rows], record.mapping(code_for(OTHER, False)))
    assert np.max(np.abs(atlas[c_rows] - alone)) <= 1e-7 and counters[COUNTER_MARK_MISSING] == 0
    assert record_from_json(record_to_json(record)).default == NO_MARK and selection_text(record, [NO_MARK]) == "no mark (kernel strip UV)"


def test_remap_keeps_untouched_islands_bit_for_bit_and_re_maps_a_corner_pair_whose_partner_changed():
    decal = _Decal()
    record = _record(CORNER)
    old = np.full(decal.face_group.shape[0], record.default)
    atlas, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, np.repeat(old, decal.totals), record)
    atlas = atlas.astype(np.float32)
    new = assign(old, decal.face_group, [decal.group(3, 0)], code_for(OTHER, False))
    out, counters = remap_atlas(atlas, decal.loop_vertex, decal.loop_group, np.repeat(old, decal.totals), np.repeat(new, decal.totals), record, record)
    b_rows = decal.loop_group == decal.group(5, 1)
    c_rows = decal.loop_group == decal.group(3, 4)
    assert counters[COUNTER_UNPAIRED] == 2, "B lost its partner; C never had one"
    assert not np.array_equal(out[b_rows], atlas[b_rows]), "the corner partner of a changed island is re-mapped (now unpaired)"
    same = assign(old, decal.face_group, [], 0)
    kept, _ = remap_atlas(atlas, decal.loop_vertex, decal.loop_group, np.repeat(old, decal.totals), np.repeat(same, decal.totals), record, record)
    assert np.array_equal(kept, atlas) and kept[c_rows].dtype == np.float64


def test_fifty_alternating_remaps_drift_less_than_a_thousandth_of_a_pixel():
    """Метка острова A trim <-> corner 50 раз, каждый раз через float32 слоя UV: ошибка против одного отображения от UV ядра."""

    decal = _Decal()
    record = _record(TRIM)
    trim = np.full(decal.face_group.shape[0], record.default)
    corner = assign(trim, decal.face_group, [decal.group(3, 0), decal.group(5, 1)], code_for(CORNER, True))
    loop = lambda codes: np.repeat(codes, decal.totals)  # noqa: E731
    atlas, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, loop(trim), record)
    atlas = atlas.astype(np.float32)
    single = atlas.copy()
    direct, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, loop(corner), record)
    codes, seen = trim, []
    for step in range(50):
        following = corner if step % 2 == 0 else trim
        atlas, _ = remap_atlas(atlas, decal.loop_vertex, decal.loop_group, loop(codes), loop(following), record, record)
        atlas, codes = atlas.astype(np.float32), following
        seen.append(atlas.astype(np.float64))
    px = lambda a, b: float(np.max(np.abs(a - b))) * 1024  # noqa: E731
    corner_vs_direct = px(seen[0], direct)  # trim -> corner против corner от UV ядра: обращение trim точно
    shift = px(seen[1], single)  # первый круг trim -> corner -> trim: along выровнен (сдвиг по длине, не ошибка float)
    drift = max(px(seen[-1], seen[1]), px(seen[-2], seen[0]))  # 50 нажатий против первого круга: накопление float32
    print(f"HOTSPOT ISLANDS remap: corner vs direct {corner_vs_direct:.2e} px, first round-trip along shift {shift:.2e} px, drift after 50 {drift:.2e} px (of 1024)")
    assert corner_vs_direct < 1e-3 and drift < 1e-3, (corner_vs_direct, drift)


def test_island_marks_survive_a_rebuild_with_new_owner_numbers_new_face_order_and_another_width():
    built = _Decal()
    record = _record(TRIM)
    face_code = assign(np.full(built.face_group.shape[0], record.default), built.face_group, [built.group(3, 0), built.group(5, 1)], code_for(CORNER, True))
    points, groups = built.points()
    remembered = remember_islands(record, built.face_group, face_code, points, groups)
    assert sorted(island.domain for island in remembered.islands) == [3, 5] and all(island.code == encode(1, True) for island in remembered.islands)
    remembered = record_from_json(record_to_json(remembered))  # через свойство меша
    rebuilt = _Decal(width=1.7, owners=(6, 2, 0), order=(5, 3, 1, 0, 4, 2))
    points, groups = rebuilt.points()
    codes, counters = island_codes(remembered, rebuilt.face_group, None, points, groups, anchor_tolerance(rebuilt.positions))
    by_group = dict(zip(rebuilt.face_group.tolist(), codes.tolist()))
    assert by_group[rebuilt.group(3, 6)] == by_group[rebuilt.group(5, 2)] == encode(1, True)
    assert by_group[rebuilt.group(3, 0)] == record.default, "the island on its own seam keeps the default even though it took an old owner number"
    assert counters[COUNTER_UNMATCHED] == 0 and counters[COUNTER_OFF_DEFAULT] == 2


def test_a_remembered_island_without_an_island_now_is_named_and_its_faces_take_the_default():
    built = _Decal()
    record = _record(TRIM)
    face_code = assign(np.full(built.face_group.shape[0], record.default), built.face_group, [built.group(3, 4)], code_for(OTHER, False))
    points, groups = built.points()
    remembered = remember_islands(record, built.face_group, face_code, points, groups)
    rebuilt = _Decal(order=(0, 1, 2, 3))  # острова C больше нет
    points, groups = rebuilt.points()
    codes, counters = island_codes(remembered, rebuilt.face_group, None, points, groups, anchor_tolerance(rebuilt.positions))
    assert counters[COUNTER_UNMATCHED] == 1 and set(codes.tolist()) == {record.default}


def test_the_face_attribute_is_the_truth_of_a_kept_mesh_and_a_mixed_island_takes_its_majority():
    decal = _Decal()
    record = _record(TRIM, islands=(IslandMarkV1(3, encode(2, False), ((99.0, 99.0, 99.0),)),))
    face_code = np.full(decal.face_group.shape[0], record.default)
    face_code[0] = encode(2, False)  # одна грань острова A из двух: ничья - меньший код
    face_code[4] = face_code[5] = encode(2, False)
    codes, counters = island_codes(record, decal.face_group, face_code)
    by_group = dict(zip(decal.face_group.tolist(), codes.tolist()))
    assert by_group[decal.group(3, 0)] == min(record.default, encode(2, False)) and by_group[decal.group(3, 4)] == encode(2, False)
    assert counters[COUNTER_MIXED] == 1 and counters[COUNTER_UNMATCHED] == 0


def test_the_selection_line_names_one_mark_or_mixed():
    record = _record(TRIM)
    assert selection_text(record, []) == ""
    assert selection_text(record, [encode(1, True), encode(1, True)]) == "1: edge (corner, swapped)"
    assert selection_text(record, [encode(0, False), encode(1, False)]) == "mixed: 0: drips (trim), 1: edge (corner)"
    assert "HOTSPOT_ISLAND_MARK_NOT_IN_RECORD" in selection_text(record, [encode(5, False)])


def test_the_press_carries_the_island_marks_of_the_same_markup_and_drops_them_by_name_otherwise():
    island = IslandMarkV1(3, encode(2, False), ((0.0, 0.0, 0.0),))
    old = _record(TRIM, islands=(island,))
    moved = HotspotMarkV1(2, "rust", "trim", 0, 320, 1024, 64, "+y")
    new = record_of(HotspotMappingV1("Sheet", CORNER, 1024, 512, False))
    carried, dropped = carry_islands(old, new, (moved,))
    assert dropped == 0 and carried.islands == (island,) and carried.mark(2) == moved and decode(carried.default) == (1, False)
    assert carried.mark(0) is None, "marks no island uses are not carried"
    other = record_of(HotspotMappingV1("Other", CORNER, 1024, 512, False))
    assert carry_islands(old, other) == (other, 1) and COUNTER_DROPPED == "HOTSPOT_ISLAND_MARKS_DROPPED"
    same = record_of(HotspotMappingV1("Sheet", OTHER, 1024, 512, False))
    assert carry_islands(old, same)[0].islands == (), "an island whose mark is the new default is the default"


def test_refresh_takes_the_rectangles_of_the_current_markup_and_keeps_a_gone_mark_as_its_snapshot():
    record = _record(TRIM)
    moved = HotspotMarkV1(1, "edge", "corner", 0, 200, 1024, 96, "+y", 12)
    refreshed = refresh_marks(record, HotspotMarkupV1(1024, 512, "sheet.png", (TRIM, moved)))
    assert refreshed.mark(1) == moved and refreshed.mark(2) == OTHER and refreshed.mark(0) == TRIM


ALPHA = 0.4  # станции шва фикстуры: u = длина ряда / 0.4 у обеих сторон, каждая от своего начала (A от вершины 0, B от вершины 2)


@pytest.mark.parametrize("up", ["+y", "+x"])
def test_the_seam_arc_unwarps_the_aligned_corner_pair_back_to_the_kernel_u_and_corner_to_trim_has_no_pop(up):
    decal = _Decal()
    corner = HotspotMarkV1(1, "edge", "corner", 300, 0, 96, 512, up, 12) if up == "+x" else CORNER
    record = HotspotRecordV2("Sheet", 1024, 512, code_for(corner, False), (TRIM, corner))
    codes = np.full(decal.loop_group.shape[0], record.default)
    atlas, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, codes, record)
    atlas = atlas.astype(np.float32)
    frame = strip_from_atlas(atlas, decal.loop_group, codes, record)
    paired = paired_corner_groups(decal.loop_vertex, decal.loop_group, codes, frame[:, 1], record)
    assert sorted(paired.tolist()) == [decal.group(3, 0), decal.group(5, 1)], "C has no ridge partner"
    assert np.max(np.abs(frame[:, 0] - decal.uvs[:, 0])) > 0.5, "the plain inversion gives the aligned along, not the kernel u"
    kernel, done = unwarp_paired(frame, decal.loop_vertex, decal.loop_group, paired, decal.positions, ALPHA)
    residual = float(np.max(np.abs(kernel - decal.uvs)))
    print(f"HOTSPOT ISLANDS seam-arc unwarp residual {residual:.2e} widths (up {up})")
    assert done == 2 and residual <= 1e-4, residual
    trim = assign(np.full(decal.face_group.shape[0], record.default), decal.face_group, decal.face_group, code_for(TRIM, False))
    moved, _ = remap_atlas(atlas, decal.loop_vertex, decal.loop_group, codes, np.repeat(trim, decal.totals), record, record, kernel)
    fresh, _ = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, np.repeat(trim, decal.totals), record)
    assert float(np.max(np.abs(moved - fresh))) * 1024 <= 0.1, "corner -> trim lands where a rebuild puts it (no pop on the next width change)"


def test_a_one_sided_island_takes_one_wing_of_a_corner_mark_and_swap_flips_it_the_build_rule():
    decal = _Decal()
    for swap, half in ((False, (304.0, 352.0)), (True, (256.0, 304.0))):
        record = _record(TRIM)
        face_code = assign(np.full(decal.face_group.shape[0], record.default), decal.face_group, [decal.group(3, 4)], code_for(CORNER, swap))
        atlas, counters = map_islands(decal.uvs, decal.loop_vertex, decal.loop_group, np.repeat(face_code, decal.totals), record)
        rows = decal.loop_group == decal.group(3, 4)
        v = atlas[rows, 1] * 512
        assert counters[COUNTER_UNPAIRED] == 1 and v.min() >= half[0] - 1e-6 and v.max() <= half[1] + 1e-6, (swap, v.min(), v.max())
        seam = rows & (decal.uvs[:, 1] == 0.0)
        assert np.max(np.abs(atlas[seam, 1] * 512 - 304.0)) <= 1e-9, "the seam is on the band centre line"
        lone, _ = atlas_uvs(decal.uvs[rows], decal.loop_vertex[rows], decal.loop_group[rows], record.mapping(code_for(CORNER, swap)))
        assert np.array_equal(atlas[rows], lone), "Apply and the press map a one-sided island with the same function"
