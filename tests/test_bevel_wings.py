"""Крылья UV декали Both (BEVEL_WINGS_V1, BEVEL_FACES_BOTH_V1): закон отображения UV ядра без Blender.

Синтетика в UV ядра Straight strips (`v = r / alpha`, `u` вдоль цепи, все карты против часовой): прямая полоса ширины `2 H` из двух
областей рельсов, встречающихся на средней линии, и две полосы снаружи от рельсов. Ответ закона известен: развёртка всей ленты на
плоскость поворотом на 180 градусов, средняя линия на `V = 0.5`, единица `u` и `v` одна. Плюс укладка по соседям, закон сварки, строка
частей и запись меша.
"""

from __future__ import annotations

import numpy as np
import pytest

from cftuv import bevel_wings as bw

L, H, ROWS = 4.0, 0.4, 5


def _strip(outer: bool = True, centre: bool = True):
    """`(UV ядра (L, 2), вершина угла, группа угла, домены бевела, xy вершин)`: полоса домена 1 вдоль x, рельс A на y = 0, B на y = 2H; снаружи
    домены 2 (под A) и 3 (над B). Вершины - сетка столбцов x = 0..L и строк y = -1, 0, H, 2H, 2H + 1."""

    ys = (-1.0, 0.0, H, 2 * H, 2 * H + 1.0)
    vertex = {(column, row): row * ROWS + column for row in range(len(ys)) for column in range(ROWS)}
    xy = {vertex[(c, r)]: (float(c), ys[r]) for (c, r) in vertex}
    regions = [
        (1, 0, 1, 2, lambda x, y: (x, y)),  # рельс A: u вдоль +x, v от A
        (1, 1, 2, 3, lambda x, y: (L - x, 2 * H - y)),  # рельс B: станции навстречу, v от B
    ]
    if outer:
        regions += [
            (2, 0, 0, 1, lambda x, y: (L - x, -y)),  # снаружи A: станции навстречу внутренней стороне
            (3, 0, 3, 4, lambda x, y: (x, y - 2 * H)),  # снаружи B
        ]
    uvs, loops, groups = [], [], []
    for domain, owner, low, high, chart in regions:
        for column in range(ROWS - 1):
            cycle = (vertex[(column, low)], vertex[(column + 1, low)], vertex[(column + 1, high)], vertex[(column, high)])
            for item in cycle:
                uvs.append(chart(*xy[item]))
                # фронты не встретились: у стороны B свои вершины на средней строке (общих вершин сторон нет)
                loops.append(item + 1000 if not centre and (domain, owner) == (1, 1) and xy[item][1] == H else item)
                groups.append(domain * (1 << bw.GROUP_SHIFT) + owner)
    return np.array(uvs), np.array(loops), np.array(groups), (1,), xy


def _area(points) -> float:
    return 0.5 * sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points)))


def test_the_kernel_charts_of_the_synthetic_strip_are_counter_clockwise():
    uvs, _loops, _groups, _inner, _xy = _strip()
    assert all(_area(uvs[start:start + 4]) > 0.0 for start in range(0, uvs.shape[0], 4))


def test_a_straight_strip_and_its_outer_bands_unfold_into_one_mirrored_trim():
    uvs, loops, groups, inner, xy = _strip()
    out, numbers = bw.wing_uvs(uvs, loops, groups, inner)
    scale = 1.0 / (2.0 * (1.0 + H))
    expected = np.array([(-xy[item][0] * scale, 0.5 + (H - xy[item][1]) * scale) for item in loops])
    assert np.allclose(out, expected, atol=1e-12)
    assert numbers[bw.COUNTER_LANES] == 1 and numbers[bw.COUNTER_RAIL_PAIRS] == 2 and numbers[bw.COUNTER_CENTRE_PAIRS] == 1
    for name in (bw.COUNTER_CENTRE_UNMET, bw.COUNTER_OUTER_UNPAIRED, bw.COUNTER_NOT_TWO_SIDED, bw.COUNTER_FOLDS, bw.COUNTER_ALONG_AGAINST):
        assert numbers[name] == 0, name
    assert numbers[bw.COUNTER_HALF_BEVEL] == round(H * 1e6)
    # средняя линия - V = 0.5, рельсы двух сторон зеркальны, обе стороны рельса одной точкой, ни одна грань не вывернута
    by_vertex: dict = {}
    for index, item in enumerate(loops.tolist()):
        by_vertex.setdefault(item, set()).add(tuple(np.round(out[index], 12)))
    assert all(len(values) == 1 for values in by_vertex.values())
    rail_a = {round(out[i][1] - 0.5, 12) for i, item in enumerate(loops.tolist()) if xy[item][1] == 0.0}
    rail_b = {round(out[i][1] - 0.5, 12) for i, item in enumerate(loops.tolist()) if xy[item][1] == 2 * H}
    assert rail_a == {-value for value in rail_b} and {round(out[i][1], 12) for i, item in enumerate(loops.tolist()) if xy[item][1] == H} == {0.5}
    assert all(_area(out[start:start + 4]) > 0.0 for start in range(0, out.shape[0], 4))


def test_the_live_frames_reuse_one_topology_plan():
    uvs, loops, groups, inner, _xy = _strip()
    bw._PLANS.clear()
    first, _numbers = bw.wing_uvs(uvs, loops, groups, inner)
    second, _numbers = bw.wing_uvs(uvs * np.array([1.0, 1.0]), loops, groups, inner)
    assert len(bw._PLANS) == 1 and np.array_equal(first, second)


def test_a_strip_whose_fronts_do_not_meet_is_named_and_an_unpaired_outer_band_too():
    uvs, loops, groups, inner, _xy = _strip(centre=False)
    _out, numbers = bw.wing_uvs(uvs, loops, groups, inner)
    assert numbers[bw.COUNTER_CENTRE_UNMET] == 2 and numbers[bw.COUNTER_CENTRE_PAIRS] == 0
    uvs, loops, groups, _inner, _xy = _strip()
    keep = (groups >> bw.GROUP_SHIFT) != 1  # домен полосы отказан: внешние полосы без рельсовой стороны
    _out, numbers = bw.wing_uvs(uvs[keep], loops[keep], groups[keep], (1,))
    assert numbers[bw.COUNTER_OUTER_UNPAIRED] == 2 and numbers[bw.COUNTER_RAIL_PAIRS] == 0


def test_the_fit_is_affine_without_reflection_and_follows_a_line_along_it():
    source = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
    shear = np.array([[1.0, 0.3], [0.0, 1.2]])
    target = source @ shear.T + np.array([2.0, -1.0])
    matrix, shift, gap, fallback = bw._fit(source, target)
    assert np.allclose(matrix, shear) and np.allclose(shift, [2.0, -1.0]) and gap < 1e-12 and not fallback
    mirrored = source * np.array([-1.0, 1.0])
    matrix, _shift, _gap, fallback = bw._fit(source, mirrored)
    assert fallback and np.linalg.det(matrix) > 0.0
    line = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    image = np.array([[5.0, 1.0], [5.0, 3.0], [5.0, 5.0]])  # повёрнута на 90 градусов и растянута вдвое вдоль себя
    matrix, shift, gap, fallback = bw._fit(line, image)
    assert gap < 1e-12 and not fallback and np.linalg.det(matrix) > 0.0
    assert np.allclose(np.array([1.0, 1.0]) @ matrix.T + shift, [4.0, 3.0])  # поперёк прямой масштаб 1, ориентация сохранена


def test_the_wing_record_round_trips_and_a_foreign_record_is_refused():
    record = bw.WingsRecordV1((7, 3, 5))
    assert bw.wings_from_json(bw.wings_to_json(record)) == bw.WingsRecordV1((3, 5, 7))
    with pytest.raises(ValueError):
        bw.wings_from_json('{"format": "OTHER", "version": 1, "inner": []}')


class _Result:
    def __init__(self, domain_id):
        self.domain_id = domain_id


def test_the_both_weld_law_is_known_by_the_domains_of_the_press():
    law = (4.0, 1e-9, 1e-9, 1e-2, 1e-6, 1.0, 1.0)
    relaxed = bw.both_weld_law(law)
    assert relaxed[2] == bw.BOTH_WELD_RESIDUAL and relaxed[:2] == law[:2] and relaxed[3:] == law[3:]
    bw.note_both_domains(("domain-a", "domain-b"))
    assert bw.is_both_run([_Result("domain-b")]) and not bw.is_both_run([_Result("domain-z")])


def test_the_parts_line_names_the_refused_part_and_keeps_the_other():
    skipped = ((5, "d5", "DEVELOPABLE_CHART_SELF_OVERLAP", ""), (2, "d2", "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED", ""))
    text = bw.parts_summary((1, 2, 3), (1, 3, 4, 6), skipped)
    assert text == "both: inward 2/3 refused (DEVELOPABLE_STRETCH_BUDGET_EXCEEDED), outward 2/3 refused (DEVELOPABLE_CHART_SELF_OVERLAP)"
    assert bw.parts_summary((1,), (1, 2), ()) == "both: inward 1/1, outward 1/1"
