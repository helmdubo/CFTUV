"""HOTSPOT_ATLAS_MAPPING_V1: UV ядра (Straight strips) -> атлас метки `trim` и `corner`, без Blender."""

from __future__ import annotations

import numpy as np
import pytest

from cftuv.hotspot_markup import HotspotMarkV1, HotspotMarkupV1
from cftuv.hotspot_mapping import (
    COUNTER_FOLDS,
    COUNTER_MIRRORED,
    COUNTER_NOT_TWO_SIDED,
    COUNTER_PAIRS,
    COUNTER_SHIFT,
    COUNTER_UNPAIRED,
    HotspotMappingV1,
    atlas_uvs,
    mapping_for,
    mapping_from_json,
    mapping_to_json,
)

W, H = 1024, 512


def _trim(rect, up):
    return HotspotMappingV1("Sheet", HotspotMarkV1(3, "drips", "trim", *rect, up), W, H)


def _pixels(atlas):
    return atlas * np.array([W, H], dtype=np.float64)


# (прямоугольник, up, пиксель (u=0, v=0), пиксель (u=0, v=1), пиксель (u=1, v=0))
TRIM_CORNERS = (
    ((0, 0, 1024, 128), "+y", (0, 0), (0, 128), (128, 0)),
    ((0, 384, 1024, 128), "-y", (1024, 512), (1024, 384), (896, 512)),
    ((0, 0, 128, 512), "+x", (0, 512), (128, 512), (0, 384)),
    ((896, 0, 128, 512), "-x", (1024, 0), (896, 0), (1024, 128)),
)


@pytest.mark.parametrize("rect, up, seam, outer, step", TRIM_CORNERS, ids=[item[1] for item in TRIM_CORNERS])
def test_trim_puts_the_seam_on_the_bottom_edge_and_the_outer_row_on_the_top_edge_at_the_exact_mark_corners(rect, up, seam, outer, step):
    mapping = _trim(rect, up)
    uvs = np.array([(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.25, 0.5)])
    atlas, counters = atlas_uvs(uvs, np.arange(4), np.zeros(4, dtype=int), mapping)
    pixels = _pixels(atlas)
    assert pixels[0].tolist() == list(map(float, seam))
    assert pixels[1].tolist() == list(map(float, outer))
    assert pixels[2].tolist() == list(map(float, step)), "one unit of u (one decal width) is one mark thickness along the length"
    x, y, w, h = rect
    assert x <= pixels[3][0] <= x + w and y <= pixels[3][1] <= y + h, "an inner point stays inside the mark"
    du, dv = pixels[2] - pixels[0], pixels[1] - pixels[0]
    assert np.hypot(*du) == np.hypot(*dv) and float(np.dot(du, dv)) == 0.0, "isotropic texels: the thickness maps to the width"
    assert du[0] * dv[1] - du[1] * dv[0] > 0.0, "the chirality of the kernel island is kept (a rotation, never a mirror)"
    assert counters == {"HOTSPOT_LOOPS_MAPPED": 4}


def test_trim_repeats_along_the_length_through_texture_wrap():
    mapping = _trim((0, 0, 1024, 128), "+y")
    atlas, _ = atlas_uvs(np.array([(0.0, 0.3), (8.0, 0.3), (19.5, 0.3)]), np.arange(3), np.zeros(3, dtype=int), mapping)
    assert atlas[1, 0] - atlas[0, 0] == 1.0, "8 widths of a 128 px mark are one 1024 px texture length"
    assert atlas[2, 1] == atlas[0, 1]


def _strip(group, stations, top_stations, seam_ids, top_ids):
    """Полоса одной стороны шва: четырёхугольники между соседними станциями; UV, вершины и группа по углам."""

    uvs, vertices, groups = [], [], []
    for k in range(len(stations) - 1):
        quad = ((stations[k], 0.0, seam_ids[k]), (stations[k + 1], 0.0, seam_ids[k + 1]), (top_stations[k + 1], 1.0, top_ids[k + 1]), (top_stations[k], 1.0, top_ids[k]))
        for u, v, vertex in quad:
            uvs.append((u, v))
            vertices.append(vertex)
            groups.append(group)
    return uvs, vertices, groups


def _butterfly(drift=0.0, origin=0.004, same_direction=False, top_offset=0.0):
    """Цепь из 9 вершин между доменом 1 (группа 1000) и доменом 4 (группа 4000): станции сторон навстречу, начало сдвинуто, длина в карте стороны B
    растянута на `drift` (сумма станций НЕ константа)."""

    steps = np.array([0.0, 0.3, 0.45, 1.2, 1.9, 2.0, 2.75, 3.1, 3.6])
    seam_ids = list(range(9))
    a = steps
    b_raw = steps * (1.0 + drift) + drift * np.sin(steps)
    b = b_raw + origin if same_direction else (b_raw[-1] - b_raw) + origin
    side_a = _strip(1000, a, a + top_offset, seam_ids, [100 + k for k in range(9)])
    order = slice(None) if same_direction else slice(None, None, -1)
    side_b = _strip(4000, b[order], b[order], seam_ids[order], [200 + k for k in range(9)][order])
    uvs = np.array(side_a[0] + side_b[0], dtype=np.float64)
    return uvs, np.array(side_a[1] + side_b[1]), np.array(side_a[2] + side_b[2])


def _corner(swap=False, up="+y"):
    rect = (0, 128, 1024, 96) if up in ("+y", "-y") else (128, 0, 96, 512)
    return HotspotMappingV1("Sheet", HotspotMarkV1(1, "edge", "corner", *rect, up, 12), W, H, swap)


def _signed_area(points):
    x, y = points[:, 0], points[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


@pytest.mark.parametrize("drift", (0.0, 0.08), ids=("lengths-equal", "lengths-drift"))
def test_corner_wings_meet_at_every_shared_seam_vertex_across_opposite_stations_with_an_origin_offset(drift):
    uvs, vertices, groups = _butterfly(drift=drift)
    sums = {}
    for (u, v), vertex in zip(uvs, vertices):
        if v == 0.0:
            sums.setdefault(int(vertex), set()).add(float(u))
    spread = np.ptp([sum(item) for item in sums.values()])
    assert (spread > 0.1) == bool(drift), "with drift the station sum of the two sides is not a constant"
    atlas, counters = atlas_uvs(uvs, vertices, groups, _corner())
    pixels = _pixels(atlas)
    seam = uvs[:, 1] == 0.0
    for vertex in range(9):
        here = pixels[seam & (vertices == vertex)]
        assert here.shape[0] >= 2 and np.max(np.abs(here - here[0])) <= 1e-9, f"seam vertex {vertex}: the wings meet"
    assert np.all(pixels[seam, 1] == 128 + 48), "the seam lies on the centre line of the mark"
    top_a = (uvs[:, 1] == 1.0) & (groups == 1000)
    top_b = (uvs[:, 1] == 1.0) & (groups == 4000)
    assert np.all(pixels[top_a, 1] == 224.0), "the lower domain takes the +up wing, its outer row on the top edge"
    assert np.all(pixels[top_b, 1] == 128.0), "the other domain's outer row is on the bottom edge"
    assert counters[COUNTER_PAIRS] == 1 and counters[COUNTER_FOLDS] == 0 and counters[COUNTER_MIRRORED] == 0
    assert (counters[COUNTER_SHIFT] > 0) == bool(drift), "the alignment shift is named, zero when the two sides agree"
    for start in range(0, uvs.shape[0], 4):
        assert np.sign(_signed_area(pixels[start:start + 4])) == np.sign(_signed_area(uvs[start:start + 4])), "both wings keep the chirality"


def test_corner_along_is_continuous_with_the_plus_side_station_and_the_rungs_stay_rungs():
    uvs, vertices, groups = _butterfly(drift=0.0, origin=0.0)
    atlas, _ = atlas_uvs(uvs, vertices, groups, _corner())
    pixels = _pixels(atlas)
    plus = groups == 1000
    assert np.allclose(pixels[plus, 0], uvs[plus, 0] * 48.0, atol=1e-9), "equal lengths and origins: along is the plus side's station, wing 48 px per width"
    for vertex in range(9):
        rung = pixels[(vertices == vertex) | (vertices == 100 + vertex)]
        assert np.ptp(rung[:, 0]) <= 1e-9, "a rung (one u) stays vertical"


def test_swap_exchanges_the_wings_and_keeps_the_meeting_and_the_chirality():
    uvs, vertices, groups = _butterfly(drift=0.05)
    atlas, counters = atlas_uvs(uvs, vertices, groups, _corner(swap=True))
    pixels = _pixels(atlas)
    assert np.all(pixels[(uvs[:, 1] == 1.0) & (groups == 1000), 1] == 128.0)
    assert np.all(pixels[(uvs[:, 1] == 1.0) & (groups == 4000), 1] == 224.0)
    seam = uvs[:, 1] == 0.0
    for vertex in range(9):
        here = pixels[seam & (vertices == vertex)]
        assert np.max(np.abs(here - here[0])) <= 1e-9
    for start in range(0, uvs.shape[0], 4):
        assert np.sign(_signed_area(pixels[start:start + 4])) == np.sign(_signed_area(uvs[start:start + 4]))
    assert counters[COUNTER_MIRRORED] == 0


def test_a_vertical_corner_mark_puts_the_seam_on_its_vertical_centre_line():
    uvs, vertices, groups = _butterfly()
    pixels = _pixels(atlas_uvs(uvs, vertices, groups, _corner(up="-x"))[0])
    assert np.all(pixels[uvs[:, 1] == 0.0, 0] == 128 + 48)
    assert np.all(pixels[(uvs[:, 1] == 1.0) & (groups == 1000), 0] == 128.0), "+ wing goes towards up = -x"


def test_same_direction_stations_are_aligned_and_named_as_a_mirrored_wing():
    uvs, vertices, groups = _butterfly(same_direction=True)
    atlas, counters = atlas_uvs(uvs, vertices, groups, _corner())
    pixels = _pixels(atlas)
    seam = uvs[:, 1] == 0.0
    for vertex in range(9):
        here = pixels[seam & (vertices == vertex)]
        assert np.max(np.abs(here - here[0])) <= 1e-9
    assert counters[COUNTER_MIRRORED] == 1


def test_a_lone_side_is_named_unpaired_and_maps_to_the_plus_wing_by_its_own_station():
    uvs, vertices, groups = _butterfly()
    alone = groups == 1000
    atlas, counters = atlas_uvs(uvs[alone], vertices[alone], groups[alone], _corner())
    assert counters[COUNTER_UNPAIRED] == 1 and counters[COUNTER_PAIRS] == 0
    assert np.allclose(_pixels(atlas)[:, 0], uvs[alone, 0] * 48.0)


def test_two_chains_meeting_at_a_corner_vertex_are_aligned_each_on_its_own():
    first = _butterfly(drift=0.03)
    second_uvs, second_vertices, second_groups = _butterfly(drift=-0.02, origin=0.5)
    # Вторая цепь начинается в последней вершине первой (угол): её вершины шва - 8 и далее, группы - другие домены.
    second_vertices = np.where(second_vertices < 9, second_vertices + 8, second_vertices + 1000)
    uvs = np.vstack((first[0], second_uvs))
    vertices = np.concatenate((first[1], second_vertices))
    groups = np.concatenate((first[2], second_groups + 1))
    atlas, counters = atlas_uvs(uvs, vertices, groups, _corner())
    assert counters[COUNTER_PAIRS] == 2 and counters[COUNTER_NOT_TWO_SIDED] == 0 and counters[COUNTER_FOLDS] == 0
    seam = uvs[:, 1] == 0.0
    for chain in ((1000, 4000), (1001, 4001)):
        mine = seam & np.isin(groups, chain)
        for vertex in np.unique(vertices[mine]):
            here = _pixels(atlas)[mine & (vertices == vertex)]
            assert np.max(np.abs(here - here[0])) <= 1e-9, f"chain {chain}, vertex {vertex}"


def test_the_mapping_record_round_trips_and_names_its_source():
    markup = HotspotMarkupV1(W, H, "trim.png", (HotspotMarkV1(1, "edge", "corner", 0, 128, 1024, 96, "+y", 12),))
    mapping = mapping_for("Sheet", markup, 1, swap=True)
    assert mapping_from_json(mapping_to_json(mapping)) == mapping
    assert mapping_for("Sheet", markup, 7) is None
    with pytest.raises(ValueError):
        mapping_from_json('{"format": "OTHER", "version": 1}')
