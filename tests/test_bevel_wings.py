"""Крылья UV декали Both (BEVEL_WINGS_V3, BEVEL_FACES_BOTH_V3): закон UV сегмента без Blender.

Синтетика: прямая полоса копии ширины `2 h` (UV в метрах одной системой сегмента: `U` вдоль средней линии, `S` от неё) и две полосы ядра
Straight strips снаружи от рельсов (`v = r / alpha`, `u` вдоль цепи в единицах alpha, карты против часовой). Ответ закона известен: сегмент -
одно аффинное отображение плоскости, `(x k, 0.5 + (y - h) k)`, `k = 0.5 / (h + alpha)` UV на метр, средняя линия на `V = 0.5`, края юбок ровно 0
и 1. Плюс: два сегмента - два острова каждый в своей системе и шов между ними, грань веера у вершины узла берёт сегмент узла, веер своей
карты ядра раскрывается от юбки своего сегмента по общему ребру, оценка alpha по рёбрам рельса, именованные отступления, строка частей и
запись меша.
"""

from __future__ import annotations

import numpy as np
import pytest

from cftuv import bevel_wings as bw

L, H, ALPHA, COLUMNS = 4.0, 0.3, 0.5, 5
COPY = 9
SCALE = 0.5 / (H + ALPHA)


def _strip(outer=(True, True), split=None):
    """`(uv, вершина угла, начала граней, домен грани, владелец грани, xy угла, позиции вершин)`: копия на `0 <= y <= 2h`, снаружи A под ней,
    B над ней. `split` - колонка, с которой начинается второй сегмент копии (своя система: `U` от `x = L` к `-x`, `S` повёрнута)."""

    ys = (-ALPHA, 0.0, H, 2 * H, 2 * H + ALPHA)
    xs = np.linspace(0.0, L, COLUMNS)
    vertex = {(c, r): r * COLUMNS + c for r in range(len(ys)) for c in range(COLUMNS)}
    positions = np.zeros((len(ys) * COLUMNS, 3))
    for (c, r), number in vertex.items():
        positions[number] = (xs[c], ys[r], 0.0)
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
            second = split is not None and c >= split
            for item, (column, row) in zip(ring, cells):
                x, y = float(xs[column]), ys[row]
                if chart is not None:
                    uvs.append(chart(x, y))
                elif second:
                    uvs.append((L - x, -(y - H)))
                else:
                    uvs.append((x, y - H))
                loops.append(item)
                points.append((x, y))
            starts.append(starts[-1] + 4)
            domains.append(domain)
            owners.append((2 if second else 0) if chart is None else owner)
    return np.array(uvs), np.array(loops), np.array(starts), np.array(domains), np.array(owners), np.array(points), positions


def _expected(points):
    return np.column_stack([points[:, 0] * SCALE, 0.5 + (points[:, 1] - H) * SCALE])


def _area(points) -> float:
    return 0.5 * sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points)))


def _map(uvs, loops, starts, domains, owners, positions):
    return bw.wing_uvs(uvs, loops, starts, domains, owners, COPY, positions)


def test_the_synthetic_kernel_charts_and_the_copy_are_counter_clockwise():
    uvs, _loops, starts, domains, _owners, _points, _positions = _strip()
    assert all(_area(uvs[a:b]) > 0.0 for a, b in zip(starts, starts[1:]))
    assert set(domains.tolist()) == {1, 2, COPY}


def test_one_segment_is_one_affine_trim_island_with_the_skirt_edges_at_zero_and_one():
    uvs, loops, starts, domains, owners, points, positions = _strip()
    out, numbers, plan = _map(uvs, loops, starts, domains, owners, positions)
    assert np.allclose(out, _expected(points), atol=1e-12), "U along the centre line, V across the profile and on along the decal width"
    assert numbers[bw.COUNTER_RAIL_PAIRS] == 2 * 2 * (COLUMNS - 1) and numbers[bw.COUNTER_RAIL_GAP] == 0
    assert numbers[bw.COUNTER_ALPHA] == round(1e6 * ALPHA) and numbers[bw.COUNTER_ALPHA_SPREAD] == 0, "alpha = rail edge length / du"
    assert numbers[bw.COUNTER_HALF_BEVEL] == round(1e6 * H) and numbers[bw.COUNTER_SEGMENTS] == 1 and numbers[bw.COUNTER_GROUPS] == 2
    for name in (bw.COUNTER_OUTER_UNPAIRED, bw.COUNTER_NOT_TWO_SIDED, bw.COUNTER_OUTER_MIRRORED, bw.COUNTER_FOLDS, bw.COUNTER_COPY_FLIPPED,
                 bw.COUNTER_KNOT_BREAKS, bw.COUNTER_KNOT_CONFLICTS, bw.COUNTER_NODE_SKIRTS, bw.COUNTER_SKIRT_STRETCH, bw.COUNTER_SEGMENT_SEAMS,
                 bw.COUNTER_KERNEL_FLIPPED, bw.COUNTER_SKIRT_FOOT_MOVED):
        assert numbers[name] == 0, (name, numbers)
    assert min(out[:, 1]) == 0.0 and max(out[:, 1]) == 1.0, "the full-width skirt edges are exactly 0 and 1"
    assert all(_area(out[a:b]) > 0.0 for a, b in zip(starts, starts[1:]))
    assert bw.seam_pairs(out, plan).shape == (0, 2), "one segment: no seam inside the island"


@pytest.mark.parametrize("kernel_u", [1.4, 0.8, -0.3])
def test_a_short_segment_skirt_follows_its_rail_in_3d_where_the_kernel_parameter_points_elsewhere(kernel_u):
    """BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4: сегмент в одну колонку между узлами (копия `0 <= x <= 0.4`, `0 <= y <= 2h`), над ним юбка - треугольник
    с вершиной над серединой рельса на всю ширину декали. Ядро даёт вершине `u` чужой точки опоры (за углом узла, `1.4` и `-0.3` - за краями
    рельса; `0.8` - на его конце, как веер выпуклого угла). Вершина берёт `(u, v)` по рельсу в 3D: грань ложится как на меше, край - ровно 1."""

    width = 0.4
    positions = np.array([(0.0, 0.0, 0.0), (width, 0.0, 0.0), (width, H, 0.0), (0.0, H, 0.0), (width, 2 * H, 0.0), (0.0, 2 * H, 0.0),
                          (width / 2, 2 * H + ALPHA, 0.0)])
    rings = ((0, 1, 2, 3), (3, 2, 4, 5), (5, 4, 6))
    uvs = [(x, y - H) for ring in rings[:2] for x, y, _z in positions[list(ring)]]
    uvs += [(0.0, 0.0), (width / ALPHA, 0.0), (kernel_u, 1.0)]
    loops = np.array([vertex for ring in rings for vertex in ring])
    starts = np.array([0, 4, 8, 11])
    out, numbers, _plan = bw.wing_uvs(np.array(uvs), loops, starts, np.array([COPY, COPY, 2]), np.array([0, 0, 0]), COPY, positions)
    points = positions[loops][:, :2]
    assert np.allclose(out, _expected(points), atol=1e-12), "the whole segment is one isometry in the trim frame"
    assert numbers[bw.COUNTER_SKIRT_FOOT_MOVED] == 1 and numbers[bw.COUNTER_FOLDS] == 0


def test_a_piece_the_rail_geometry_would_fold_keeps_the_kernel_parameter_by_name():
    """BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4, откат: квад юбки над сегментом в одну колонку, у ядра он против часовой, а по геометрии рельса его
    внешние вершины меняются местами (грань вывернулась бы). Кусок целиком берёт параметр ядра - `BEVEL_WINGS_SKIRT_FOOT_REVERTED`."""

    width = 0.4
    positions = np.array([(0.0, 0.0, 0.0), (width, 0.0, 0.0), (width, H, 0.0), (0.0, H, 0.0), (width, 2 * H, 0.0), (0.0, 2 * H, 0.0),
                          (-0.05, 2 * H + ALPHA, 0.0), (width + 0.05, 2 * H + ALPHA, 0.0)])
    rings = ((0, 1, 2, 3), (3, 2, 4, 5), (5, 4, 6, 7))
    kernel = [(0.0, 0.0), (width / ALPHA, 0.0), (1.5, 1.0), (1.4, 1.0)]
    uvs = [(x, y - H) for ring in rings[:2] for x, y, _z in positions[list(ring)]] + kernel
    loops = np.array([vertex for ring in rings for vertex in ring])
    out, numbers, _plan = bw.wing_uvs(np.array(uvs), loops, np.array([0, 4, 8, 12]), np.array([COPY, COPY, 2]), np.array([0, 0, 0]), COPY,
                                      positions)
    assert numbers[bw.COUNTER_SKIRT_FOOT_REVERTED] == 1 and numbers[bw.COUNTER_SKIRT_FOOT_MOVED] == 0 and numbers[bw.COUNTER_FOLDS] == 0
    outer = np.array([(width + ALPHA * (u - width / ALPHA)) * SCALE for u, _v in kernel[2:]])
    assert np.allclose(out[10:12, 0], outer, atol=1e-12) and np.allclose(out[10:12, 1], 1.0, atol=1e-12), "the kernel law, unchanged"


def test_two_segments_are_two_islands_each_in_its_own_frame_with_a_seam_between_them():
    split = 2
    uvs, loops, starts, domains, owners, points, positions = _strip(split=split)
    out, numbers, plan = _map(uvs, loops, starts, domains, owners, positions)
    first = np.repeat(np.arange(len(starts) - 1) % (COLUMNS - 1) < split, 4)
    assert np.allclose(out[first], _expected(points)[first], atol=1e-12), "the first segment is the plain trim"
    turned = np.column_stack([(L - points[:, 0]) * SCALE, 0.5 - (points[:, 1] - H) * SCALE])
    assert np.allclose(out[~first], turned[~first], atol=1e-12), "the second segment keeps its own frame: no fitting between frames"
    assert numbers[bw.COUNTER_SEGMENTS] == 2 and numbers[bw.COUNTER_GROUPS] == 4 and numbers[bw.COUNTER_RAIL_GAP] == 0
    for name in (bw.COUNTER_FOLDS, bw.COUNTER_OUTER_MIRRORED, bw.COUNTER_COPY_FLIPPED, bw.COUNTER_SEGMENT_SEAMS):
        assert numbers[name] == 0, (name, numbers)
    seams = {tuple(pair) for pair in bw.seam_pairs(out, plan).tolist()}
    assert seams == {(r * COLUMNS + split, (r + 1) * COLUMNS + split) for r in range(4)}, "the seam is the cut between the two segments"


def test_a_fan_touching_only_a_node_vertex_takes_the_node_segment_and_its_apex():
    uvs, loops, starts, domains, owners, _points, positions = _strip(outer=(False, True))
    node_face = int(np.flatnonzero(domains == COPY)[-2])  # верхний ряд копии, колонка 2: грань узла, присоединённого к сегменту 1
    owners = owners.copy()
    owners[node_face] = bw.OWNER_SHIFT * 1 + bw.OWNER_NODE
    apex = 3 * COLUMNS + 3  # вершина рельса между гранью сегмента 0 и гранью узла
    positions = np.vstack([positions, np.zeros((202 - len(positions), 3))])
    positions[200], positions[201] = (3.5, 1.2, 0.0), (2.5, 1.2, 0.0)
    out, numbers, plan = _map(np.vstack([uvs, [(0.0, 0.0), (0.4, 0.9), (-0.4, 0.9)]]), np.concatenate([loops, [apex, 200, 201]]),
                              np.concatenate([starts, [starts[-1] + 3]]), np.concatenate([domains, [5]]), np.concatenate([owners, [0]]), positions)
    assert plan.face_segment[-1] == 1, "the fan at a node vertex belongs to the node's segment"
    assert numbers[bw.COUNTER_VERTEX_PAIRED] == 1
    node_loop = int(starts[node_face]) + list(loops[starts[node_face]:starts[node_face + 1]]).index(apex)
    assert np.allclose(out[-3], out[node_loop]), "the fan apex is exactly the node corner"
    assert _area(out[-3:]) > 0.0


def test_the_plan_is_cached_by_topology_and_a_face_away_from_the_copy_is_named():
    uvs, loops, starts, domains, owners, _points, positions = _strip()
    args = (loops, starts, domains, owners, np.abs(uvs[:, 1]) <= bw.SEAM_V_TOLERANCE, COPY)
    assert bw.cached_wings_plan(*args) is bw.cached_wings_plan(*args)
    far = np.concatenate([uvs, [(0.0, 0.5), (1.0, 0.5), (1.0, 1.0), (0.0, 1.0)]])
    positions = np.vstack([positions, np.zeros((504 - len(positions), 3))])
    out, numbers, _plan = _map(far, np.concatenate([loops, [500, 501, 502, 503]]), np.concatenate([starts, [starts[-1] + 4]]),
                               np.concatenate([domains, [3]]), np.concatenate([owners, [0]]), positions)
    assert numbers[bw.COUNTER_OUTER_UNPAIRED] == 1
    rail = 0.5 + H * SCALE
    middle = rail + 0.5 * (0.5 - H * SCALE)
    assert np.allclose(out[-4:], [(0.0, middle), (ALPHA * SCALE, middle), (ALPHA * SCALE, 1.0), (0.0, 1.0)])


def test_without_outer_bands_alpha_is_unknown_by_name_and_the_copy_keeps_its_metres():
    uvs, loops, starts, domains, owners, _points, positions = _strip(outer=(False, False))
    out, numbers, _plan = _map(uvs, loops, starts, domains, owners, positions)
    assert numbers[bw.COUNTER_ALPHA_UNKNOWN] == 1 and numbers[bw.COUNTER_GROUPS] == 0
    scale = 0.5 / (H + 1.0)
    assert np.allclose(out, np.column_stack([uvs[:, 0] * scale, 0.5 + uvs[:, 1] * scale]))


def test_the_wing_record_roundtrips_and_refuses_another_version():
    record = bw.WingsRecordV3(17)
    assert bw.wings_from_json(bw.wings_to_json(record)) == record
    with pytest.raises(ValueError):
        bw.wings_from_json('{"format": "CFTUV_BEVEL_WINGS", "version": 2, "copy": 1}')


def test_the_parts_line_names_the_copy_and_every_refused_outward_domain():
    assert bw.parts_summary(48, (0, 1, 2), ()) == "both: copy 48 faces, outward 3/3"
    skipped = ((3, "d3", "DEVELOPABLE_CHART_SELF_OVERLAP", ""), (4, "d4", "DEVELOPABLE_CHART_SELF_OVERLAP", ""), (5, "d5", "ADAPTER_X", ""))
    assert bw.parts_summary(7, (0,), skipped) == "both: copy 7 faces, outward 1/4 refused (ADAPTER_X 1, DEVELOPABLE_CHART_SELF_OVERLAP 2)"


def test_a_fan_of_its_own_kernel_chart_unfolds_from_the_skirt_of_its_segment():
    uvs, loops, starts, domains, owners, _points, positions = _strip(outer=(False, True))
    apex, far = 3 * COLUMNS + COLUMNS - 1, 4 * COLUMNS + COLUMNS - 1  # правый конец верхнего рельса и угол юбки B над ним
    positions = np.vstack([positions, np.zeros((201 - len(positions), 3))])
    positions[200] = (L + ALPHA, 2 * H + ALPHA, 0.0)
    out, numbers, plan = _map(np.vstack([uvs, [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0)]]), np.concatenate([loops, [apex, 200, far]]),
                              np.concatenate([starts, [starts[-1] + 3]]), np.concatenate([domains, [5]]), np.concatenate([owners, [0]]), positions)
    assert plan.face_segment[-1] == 0 and numbers[bw.COUNTER_FANS_UNFOLDED] == 1, "the fan joins the skirt it shares an edge with"
    skirt = [int(item) for item in np.flatnonzero((domains == 2))]
    last = skirt[-1]
    ring = list(loops[starts[last]:starts[last + 1]])
    for vertex, at in ((apex, -3), (far, -1)):
        assert np.allclose(out[at], out[starts[last] + ring.index(vertex)]), "UV is continuous along the shared edge"
    assert _area(out[-3:]) > 0.0 and numbers[bw.COUNTER_FOLDS] == 0
