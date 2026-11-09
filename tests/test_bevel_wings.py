"""Крылья UV декали Both (BEVEL_WINGS_V2, BEVEL_FACES_BOTH_V2): закон отображения UV копии и ядра без Blender.

Синтетика: прямая полоса копии ширины `2 h` (UV в метрах: `U` вдоль средней линии, `S` от неё) и две полосы ядра Straight strips снаружи от
рельсов (`v = r / alpha`, `u` вдоль цепи в единицах alpha, карты против часовой). Ответ закона известен: вся лента - одно аффинное
отображение плоскости, `(x k / alpha, 0.5 + (y - h) k / alpha)`, средняя линия на `V = 0.5`, единица `u` и `v` одна. Плюс согласование кадров
(второй кадр копии повёрнут на 180 градусов и сдвинут), именованные отступления, строка частей и запись меша.
"""

from __future__ import annotations

import numpy as np
import pytest

from cftuv import bevel_wings as bw

L, H, ALPHA, COLUMNS = 4.0, 0.3, 0.5, 5
COPY = 9


def _strip(outer=(True, True), split_frame=False):
    """`(uv, вершина угла, начала граней, домен грани, владелец грани, xy вершины угла)`: копия на `0 <= y <= 2h`, снаружи A под ней, B над ней."""

    ys = (-ALPHA, 0.0, H, 2 * H, 2 * H + ALPHA)
    xs = np.linspace(0.0, L, COLUMNS)
    vertex = {(c, r): r * COLUMNS + c for r in range(len(ys)) for c in range(COLUMNS)}
    bands = []
    if outer[0]:
        bands.append((0, 1, 1, 0, lambda x, y: ((L - x) / ALPHA, -y / ALPHA)))
    bands.append((1, 2, COPY, None, None))
    bands.append((2, 3, COPY, None, None))
    if outer[1]:
        bands.append((3, 4, 2, 0, lambda x, y: (x / ALPHA, (y - 2 * H) / ALPHA)))
    uvs, loops, starts, domains, owners, points = [], [], [0], [], [], []
    for low, high, domain, owner, chart in bands:
        for c in range(COLUMNS - 1):
            ring = (vertex[(c, low)], vertex[(c + 1, low)], vertex[(c + 1, high)], vertex[(c, high)])
            cells = ((c, low), (c + 1, low), (c + 1, high), (c, high))
            frame = 1 if split_frame and c >= (COLUMNS - 1) // 2 else 0
            for item, (column, row) in zip(ring, cells):
                x, y = float(xs[column]), ys[row]
                if chart is not None:
                    uvs.append(chart(x, y))
                elif frame:
                    uvs.append((7.0 - x, -(y - H)))  # второй кадр: поворот на 180 градусов и свой сдвиг
                else:
                    uvs.append((x, y - H))
                loops.append(item)
                points.append((x, y))
            starts.append(starts[-1] + 4)
            domains.append(domain)
            owners.append(frame if chart is None else owner)
    return np.array(uvs), np.array(loops), np.array(starts), np.array(domains), np.array(owners), np.array(points)


def _expected(points):
    scale = 1.0 / (2.0 * (1.0 + H / ALPHA))
    return np.column_stack([points[:, 0] * scale / ALPHA, 0.5 + (points[:, 1] - H) * scale / ALPHA])


def _area(points) -> float:
    return 0.5 * sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points)))


def test_the_synthetic_kernel_charts_and_the_copy_are_counter_clockwise():
    uvs, _loops, starts, domains, _owners, _points = _strip()
    assert all(_area(uvs[a:b]) > 0.0 for a, b in zip(starts, starts[1:]))
    assert set(domains.tolist()) == {1, 2, COPY}


def test_a_straight_copy_and_its_outer_bands_unfold_into_one_mirrored_trim():
    uvs, loops, starts, domains, owners, points = _strip()
    out, numbers = bw.wing_uvs(uvs, loops, starts, domains, owners, COPY)
    assert np.allclose(out, _expected(points), atol=1e-12)
    assert numbers[bw.COUNTER_RAIL_PAIRS] == 2 * 2 * (COLUMNS - 1) and numbers[bw.COUNTER_RAIL_GAP] == 0
    assert numbers[bw.COUNTER_ALPHA_SPREAD] == 0 and bw.COUNTER_ALPHA_UNKNOWN not in numbers
    assert numbers[bw.COUNTER_HALF_BEVEL] == round(1e6 * H / ALPHA)
    for name in (bw.COUNTER_OUTER_UNPAIRED, bw.COUNTER_NOT_TWO_SIDED, bw.COUNTER_OUTER_MIRRORED, bw.COUNTER_FOLDS, bw.COUNTER_FRAME_CONFLICTS):
        assert numbers[name] == 0, (name, numbers)
    assert min(out[:, 1]) == pytest.approx(0.0) and max(out[:, 1]) == pytest.approx(1.0)
    assert all(_area(out[a:b]) > 0.0 for a, b in zip(starts, starts[1:]))


def test_a_rotated_second_copy_frame_is_aligned_through_the_shared_rail_vertices():
    uvs, loops, starts, domains, owners, points = _strip(split_frame=True)
    out, numbers = bw.wing_uvs(uvs, loops, starts, domains, owners, COPY)
    assert np.allclose(out, _expected(points), atol=1e-12), "the second frame is turned and shifted onto the first"
    assert numbers[bw.COUNTER_FRAME_CONFLICTS] == 0 and numbers[bw.COUNTER_RAIL_GAP] == 0 and numbers[bw.COUNTER_NOT_TWO_SIDED] == 0


def test_the_plan_is_cached_by_topology_and_an_island_away_from_the_copy_is_named():
    uvs, loops, starts, domains, owners, _points = _strip()
    args = (loops, starts, domains, owners, np.abs(uvs[:, 1]) <= bw.SEAM_V_TOLERANCE, COPY)
    assert bw.cached_wings_plan(*args) is bw.cached_wings_plan(*args)
    far = np.concatenate([uvs, [(0.0, 0.5), (1.0, 0.5), (1.0, 1.0), (0.0, 1.0)]])
    out, numbers = bw.wing_uvs(far, np.concatenate([loops, [500, 501, 502, 503]]), np.concatenate([starts, [starts[-1] + 4]]),
                               np.concatenate([domains, [3]]), np.concatenate([owners, [0]]), COPY)
    assert numbers[bw.COUNTER_OUTER_UNPAIRED] == 1
    scale = 1.0 / (2.0 * (1.0 + H / ALPHA))
    assert np.allclose(out[-4:], [(0.0, 0.5 + (H / ALPHA + 0.5) * scale), (scale, 0.5 + (H / ALPHA + 0.5) * scale),
                                  (scale, 0.5 + (H / ALPHA + 1.0) * scale), (0.0, 0.5 + (H / ALPHA + 1.0) * scale)])


def test_without_outer_bands_alpha_is_unknown_by_name_and_the_copy_stays_in_metres():
    uvs, loops, starts, domains, owners, _points = _strip(outer=(False, False))
    out, numbers = bw.wing_uvs(uvs, loops, starts, domains, owners, COPY)
    assert numbers[bw.COUNTER_ALPHA_UNKNOWN] == 1 and numbers[bw.COUNTER_ISLANDS] == 0
    scale = 1.0 / (2.0 * (1.0 + H))
    assert np.allclose(out, np.column_stack([uvs[:, 0] * scale, 0.5 + uvs[:, 1] * scale]))


def test_the_wing_record_roundtrips_and_refuses_another_version():
    record = bw.WingsRecordV2(17)
    assert bw.wings_from_json(bw.wings_to_json(record)) == record
    with pytest.raises(ValueError):
        bw.wings_from_json('{"format": "CFTUV_BEVEL_WINGS", "version": 1, "inner": [1]}')


def test_the_parts_line_names_the_copy_and_every_refused_outward_domain():
    assert bw.parts_summary(48, (0, 1, 2), ()) == "both: copy 48 faces, outward 3/3"
    skipped = ((3, "d3", "DEVELOPABLE_CHART_SELF_OVERLAP", ""), (4, "d4", "DEVELOPABLE_CHART_SELF_OVERLAP", ""), (5, "d5", "ADAPTER_X", ""))
    assert bw.parts_summary(7, (0,), skipped) == "both: copy 7 faces, outward 1/4 refused (ADAPTER_X 1, DEVELOPABLE_CHART_SELF_OVERLAP 2)"
