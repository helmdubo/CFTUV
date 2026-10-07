"""Конец стены на линии сгиба (`WALL_MITER_OFFSET_V1`): две области, сходящиеся вдоль невыделенной цепи, не оставляют клина.

ПОЛЕВАЯ БЕДА (владелец, `rounded_wall.001`, выделены цепи патча верхней крышки, декаль «юбкой» идёт на соседние патчи). Вертикальный
шов между длинной гранью и торцом не выделен: его касается конец выделенной цепи, и с обеих сторон шва стоит стена соседнего домена. Вдоль
шва два полотна расходились клином: от общей митровой вершины у цепи до 8 мм (смещение сцены 5 мм; 28 мм при 20 мм) у конца стены, не зависимо
от ширины. Геометрия батча ни при чём (до смещения обе стены лежат на ребре источника, глубины различаются на 0.3 %): клин делало смещение хоста,
которое узел стены (`node:`, домен-локальный) вело по нормали одного домена, а вершину `src:` — митрой.

Здесь закон проверяется Blender-free: синтетическая складка (точные числа), красный контроль (без цепей стен всё как прежде и клин есть), полевая
фикстура (`artifacts/wall_miter/rounded_wall_001_walls.json`, выгрузка `export_wall_fixture.py`: четыре домена, ширины 0.2239 и 0.5) и сквозной путь
через `build_mesh_arrays` (вид домена несёт цепи стен из батча).
"""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from cftuv.envelope_production_export import MATERIALIZED, ProductionDomainResultV1
from cftuv.envelope_production_mesh import build_mesh_arrays
from cftuv.envelope_production_view import build_domain_view
from cftuv.envelope_production_weld import (
    MITER_LIMIT,
    WALL_DIRECTION_SINE,
    WELD_LOCATION_PREFIX,
    DomainVerticesV1,
    miter_offset,
    plan_wall_nodes,
    weld_vertices,
)

ROOT = Path(__file__).resolve().parents[1]
FIELD = ROOT / "artifacts" / "wall_miter" / "rounded_wall_001_walls.json"

UP = (0.0, 0.0, 1.0)
SIDE = (-1.0, 0.0, 0.0)
AWAY = (1.0, 0.0, 0.0)
ANCHOR = "location:src:host:A"
D = 0.02


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _unit(vector):
    length = math.sqrt(_dot(vector, vector))
    return tuple(item / length for item in vector)


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _domain(patch_id, depth, normal, *, anchor=(1.0, 0.0, 0.0), node_normal=None, walls=True, direction=(0.0, 1.0, 0.0), ref=ANCHOR):
    """Стена одного домена: вершина источника `anchor` и узел на глубине `depth` вдоль `direction` (цепь стены `[anchor, узел]`)."""

    node = tuple(a + depth * d for a, d in zip(anchor, direction))
    return DomainVerticesV1(
        patch_id,
        (anchor, node),
        (normal, node_normal or normal),
        (ref, f"location:node:{patch_id}"),
        ((0, 1),) if walls else (),
    )


def _final(weld, ordinal, local):
    return weld.positions[weld.index[ordinal][local]]


def _line_distance(point, first, second):
    line = _sub(second, first)
    along = _dot(_sub(point, first), line) / _dot(line, line)
    return math.dist(point, tuple(a + along * b for a, b in zip(first, line)))


def lateral_gap(weld, domains) -> float:
    """Наибольшее расстояние конца стены от ПРОДОЛЖЕННОЙ линии стены соседа (глубину, на которую один домен длиннее, щелью не считаем)."""

    worst = 0.0
    for ordinal in range(len(domains)):
        other = 1 - ordinal
        first, second = _final(weld, other, 0), _final(weld, other, 1)
        worst = max(worst, _line_distance(_final(weld, ordinal, 1), first, second))
    return worst


def _counters(weld):
    return dict(weld.counters)


# --------------------------------------------------------------------------
# Синтетическая складка: точные числа
# --------------------------------------------------------------------------


@pytest.mark.parametrize("wall_normal", [SIDE, AWAY], ids=["concave", "convex"])
def test_symmetric_wall_ends_weld_into_one_vertex_on_the_fold_line(wall_normal):
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, wall_normal)]

    weld = weld_vertices(domains, D)

    assert weld.index[0][1] == weld.index[1][1]
    corner = tuple(1.0 + D * wall_normal[0] if axis == 0 else (0.4 if axis == 1 else D) for axis in range(3))
    assert _final(weld, 0, 1) == pytest.approx(corner, abs=1e-12)
    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_WELDED"] == 1 and counters["ADAPTER_WALL_NODES_LIFTED"] == 2
    assert counters["ADAPTER_WALL_NODES_ALONE"] == counters["ADAPTER_WALL_MITER_FALLBACK"] == 0
    assert lateral_gap(weld, domains) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("wall_normal", [SIDE, AWAY], ids=["concave", "convex"])
def test_unequal_depths_put_both_ends_on_the_same_line_and_leave_no_lateral_gap(wall_normal):
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4012, wall_normal)]

    weld = weld_vertices(domains, D)

    assert weld.index[0][1] != weld.index[1][1]
    first, second = _final(weld, 0, 1), _final(weld, 1, 1)
    assert first[0] == pytest.approx(second[0], abs=1e-12) and first[2] == pytest.approx(second[2], abs=1e-12)
    assert lateral_gap(weld, domains) == pytest.approx(0.0, abs=1e-12)
    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 2 and counters["ADAPTER_WALL_NODES_WELDED"] == 0


def test_without_wall_chains_the_two_edges_still_spread_apart_by_d_times_the_normal_difference():
    """Красный контроль: домен без цепей стен ведёт себя как прежде, и клин есть."""

    domains = [_domain(0, 0.4, UP, walls=False), _domain(1, 0.4, SIDE, walls=False)]

    weld = weld_vertices(domains, D)

    assert lateral_gap(weld, domains) == pytest.approx(D * math.sqrt(2.0), rel=1e-3)
    assert weld.index[0][1] != weld.index[1][1]
    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == counters["ADAPTER_WALL_NODES_ALONE"] == 0


def test_every_face_keeps_to_the_offset_plane_of_its_own_domain_after_the_lift():
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4012, SIDE)]

    weld = weld_vertices(domains, D)

    assert _final(weld, 0, 1)[2] == pytest.approx(D, abs=1e-12)
    assert _final(weld, 1, 1)[0] == pytest.approx(1.0 - D, abs=1e-12)


def test_a_node_without_a_neighbour_keeps_the_plain_offset_and_is_counted_alone():
    domains = [_domain(0, 0.4, UP)]

    weld = weld_vertices(domains, D)

    assert _final(weld, 0, 1) == pytest.approx((1.0, 0.4, D), abs=1e-15)
    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_ALONE"] == 1 and counters["ADAPTER_WALL_NODES_LIFTED"] == 0


def test_a_wall_from_the_same_vertex_along_another_edge_is_not_the_neighbour():
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, SIDE, direction=(0.0, 0.0, 1.0))]

    weld = weld_vertices(domains, D)

    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 0 and counters["ADAPTER_WALL_NODES_ALONE"] == 2


def test_a_direction_off_the_edge_by_more_than_the_tolerance_is_not_the_neighbour():
    slanted = _unit((0.0, 1.0, 2.0 * WALL_DIRECTION_SINE))
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, SIDE, direction=slanted)]

    assert _counters(weld_vertices(domains, D))["ADAPTER_WALL_NODES_LIFTED"] == 0
    near = _unit((0.0, 1.0, 0.5 * WALL_DIRECTION_SINE))
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, SIDE, direction=near)]
    assert _counters(weld_vertices(domains, D))["ADAPTER_WALL_NODES_LIFTED"] == 2


def test_anchors_that_are_not_bitwise_the_same_point_are_not_neighbours():
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, SIDE, anchor=(1.0, 1e-9, 0.0))]

    counters = _counters(weld_vertices(domains, D))

    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 0 and counters["ADAPTER_WELD_POSITION_MISMATCH"] == 1


def test_the_neighbour_normal_is_blended_along_the_wall_between_its_anchor_and_its_end():
    turned = _unit((-math.cos(0.3), -math.sin(0.3), 0.0))
    domains = [_domain(0, 0.2, UP), _domain(1, 0.4, SIDE, node_normal=turned)]

    weld = weld_vertices(domains, D)

    blend = _unit(tuple(0.5 * a + 0.5 * b for a, b in zip(SIDE, turned)))
    expected, _factor, _reason = miter_offset([UP, blend])
    assert _final(weld, 0, 1) == pytest.approx(tuple(p + D * o for p, o in zip((1.0, 0.2, 0.0), expected)), abs=1e-12)
    assert _dot(_sub(_final(weld, 0, 1), (1.0, 0.2, 0.0)), UP) == pytest.approx(D, abs=1e-12)
    assert _dot(_sub(_final(weld, 0, 1), (1.0, 0.2, 0.0)), blend) == pytest.approx(D, abs=1e-12)


def test_beyond_the_neighbour_end_its_end_normal_is_used_not_an_extrapolation():
    turned = _unit((-math.cos(0.3), -math.sin(0.3), 0.0))
    domains = [_domain(0, 0.6, UP), _domain(1, 0.4, SIDE, node_normal=turned)]

    weld = weld_vertices(domains, D)

    expected, _factor, _reason = miter_offset([UP, turned])
    assert _final(weld, 0, 1) == pytest.approx(tuple(p + D * o for p, o in zip((1.0, 0.6, 0.0), expected)), abs=1e-12)


def test_a_knife_fold_keeps_the_own_offsets_and_names_the_fallback():
    knife = _unit((math.sin(3.1), 0.0, math.cos(3.1)))
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, knife)]

    weld = weld_vertices(domains, D)

    counters = _counters(weld)
    assert counters["ADAPTER_WALL_MITER_FALLBACK"] == 2, "each of the two ends is tried and refused"
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 0
    assert _final(weld, 0, 1) == pytest.approx((1.0, 0.4, D), abs=1e-15)
    named = [item for item in weld.warnings if item[1] == "ADAPTER_WALL_MITER_FALLBACK"]
    assert len(named) == 1 and "the miter exceeds the limit" in named[0][2] and f"limit {MITER_LIMIT:g}" in named[0][2]


def test_the_plan_reads_only_nodes_and_leaves_source_vertices_to_the_weld():
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4, SIDE)]

    plan = plan_wall_nodes(domains)

    assert {key[1] for key in (*plan.lifts, *plan.classes)} == {1}


def test_a_zero_offset_moves_nothing():
    domains = [_domain(0, 0.4, UP), _domain(1, 0.4012, SIDE)]

    weld = weld_vertices(domains, 0.0)

    assert _final(weld, 0, 1) == (1.0, 0.4, 0.0) and _final(weld, 1, 1) == (1.0, 0.4012, 0.0)


# --------------------------------------------------------------------------
# Полевая фикстура: rounded_wall.001, цепи патча крышки
# --------------------------------------------------------------------------


def _field_runs():
    document = json.loads(FIELD.read_text(encoding="utf-8"))
    runs = []
    for run in document["runs"]:
        domains = [
            DomainVerticesV1(
                item["patch_id"],
                tuple(tuple(p) for p in item["positions"]),
                tuple(tuple(n) for n in item["normals"]),
                tuple(item["refs"]),
                tuple(tuple(chain) for chain in item["walls"]),
            )
            for item in run["domains"]
        ]
        runs.append((run["alpha"], document["offset"], domains))
    return runs


def _wall_pairs(domains):
    """Пары `(домен, вершина-узел, домен соседа, якорь соседа, узел соседа)` вдоль общих стен (по общей ссылке вершины источника)."""

    pairs = []
    for ordinal, domain in enumerate(domains):
        for chain in domain.walls:
            for at, local in enumerate(chain):
                ref = domain.refs[local]
                if ref.startswith(WELD_LOCATION_PREFIX):
                    continue
                anchor = next(chain[at + step] for step in (-1, 1) if 0 <= at + step < len(chain) and domain.refs[chain[at + step]].startswith(WELD_LOCATION_PREFIX))
                for other_ordinal, other in enumerate(domains):
                    if other_ordinal == ordinal:
                        continue
                    for other_chain in other.walls:
                        for other_at, other_local in enumerate(other_chain):
                            if other.refs[other_local] != domain.refs[anchor]:
                                continue
                            heading = _sub(domain.positions[local], domain.positions[anchor])
                            for step in (-1, 1):
                                if not 0 <= other_at + step < len(other_chain):
                                    continue
                                beyond = other_chain[other_at + step]
                                there = _sub(other.positions[beyond], other.positions[other_local])
                                if _dot(heading, there) > 0.0:
                                    pairs.append((ordinal, local, other_ordinal, other_local, beyond))
    return pairs


@pytest.mark.parametrize("alpha, offset, domains", _field_runs(), ids=lambda value: f"{value}" if isinstance(value, float) else "")
def test_field_corner_of_rounded_wall_001_has_no_lateral_gap_along_the_unselected_seams(alpha, offset, domains):
    pairs = _wall_pairs(domains)
    assert len(pairs) == 8, "the four unselected vertical seams have a wall of two domains each"

    after = weld_vertices(domains, offset)
    before = weld_vertices([replace(item, walls=()) for item in domains], offset)

    def worst(weld):
        gap = 0.0
        for ordinal, local, other, anchor, beyond in pairs:
            gap = max(gap, _line_distance(_final(weld, ordinal, local), _final(weld, other, anchor), _final(weld, other, beyond)))
        return gap

    counters = dict(after.counters)
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 8 and counters["ADAPTER_WALL_MITER_FALLBACK"] == 0
    assert counters["ADAPTER_WELD_POSITION_MISMATCH"] == 0 and counters["ADAPTER_WELD_MITER_FALLBACK"] == 0
    assert worst(before) > 5e-3, "the wedge of the offset policy: 5.7-8.4 mm at the 5 mm offset"
    # Остаток — шум подъёма самих узлов в батче (до одной ячейки решётки, 0.43 мм у изогнутой грани), не политика смещения.
    assert worst(after) < 6e-4


def test_field_domains_carry_four_corner_walls_with_two_domains_on_each_seam():
    for _alpha, _offset, domains in _field_runs():
        assert sorted(item.patch_id for item in domains) == [0, 2, 3, 5]
        assert all(len(item.walls) == 2 for item in domains)


# --------------------------------------------------------------------------
# Сквозной путь: батч с цепью `boundary:WALL:*` -> вид домена -> меш
# --------------------------------------------------------------------------


def _batch_domain(patch_id, depth, normal):
    """Домен минимального вида (как `_fake_domain` писателя меша): грань `[anchor, узел, p, q]`, стена `[anchor, узел]`."""

    anchor = (1.0, 0.0, 0.0)
    node = (1.0, depth, 0.0)
    plane = {0: ((0.0, depth, 0.0), (0.0, 0.0, 0.0)), 1: ((1.0, depth, 1.0), (1.0, 0.0, 1.0))}[patch_id]
    keys = {"src:a": anchor, "node:n": node, "p": plane[0], "q": plane[1]}
    refs = {"src:a": ANCHOR, "node:n": f"location:node:{patch_id}", "p": f"location:node:p{patch_id}", "q": f"location:node:q{patch_id}"}

    def vertex(key):
        return SimpleNamespace(
            vert_key=SimpleNamespace(value=key),
            position=SimpleNamespace(x=keys[key][0], y=keys[key][1], z=keys[key][2]),
            semantic_location_ref=SimpleNamespace(value=refs[key]),
        )

    order = ("src:a", "node:n", "p", "q") if patch_id == 0 else ("src:a", "q", "p", "node:n")
    face = SimpleNamespace(
        face_id=SimpleNamespace(value=f"face:{patch_id}"),
        ordered_vert_keys=tuple(SimpleNamespace(value=key) for key in order),
        uv_facts=tuple(SimpleNamespace(uv=SimpleNamespace(u=float(i), v=0.5)) for i, _key in enumerate(order)),
        ownership_claim_id=SimpleNamespace(value="claim:0"),
    )
    batch = SimpleNamespace(
        vertices=tuple(vertex(key) for key in keys),
        faces=(face,),
        interface_chains=(),
        boundary_chains=(
            SimpleNamespace(
                semantic_boundary_id=SimpleNamespace(value="boundary:WALL:0:0"),
                ordered_vert_keys=(SimpleNamespace(value="src:a"), SimpleNamespace(value="node:n")),
            ),
        ),
        source_revision=SimpleNamespace(value="rev"),
    )
    return ProductionDomainResultV1(patch_id, f"domain{patch_id}", MATERIALIZED, batch, normal=normal, source_normal=normal)


def test_the_domain_view_carries_the_wall_chains_of_the_batch_in_local_numbers():
    view = build_domain_view(_batch_domain(0, 0.4, UP))

    ordered = sorted(("src:a", "node:n", "p", "q"))
    assert view.vertices.walls == ((ordered.index("src:a"), ordered.index("node:n")),)


def test_the_mesh_welds_the_wall_ends_of_two_domains_and_reports_the_law():
    results = [_batch_domain(0, 0.4, UP), _batch_domain(1, 0.4, SIDE)]

    arrays = build_mesh_arrays(results, D)

    counters = dict(arrays.weld_counters)
    assert counters["ADAPTER_WALL_NODES_WELDED"] == 1 and counters["ADAPTER_WALL_NODES_LIFTED"] == 2
    shared = set(arrays.faces[0]) & set(arrays.faces[1])
    assert len(shared) == 2, "the anchor and the wall end are the two vertices both domains use"
    assert all(abs(arrays.positions[index][0] - (1.0 - D)) < 1e-12 and abs(arrays.positions[index][2] - D) < 1e-12 for index in shared)
    assert not any(item[1].startswith("ADAPTER_WALL") for item in arrays.warnings)


def test_the_mesh_of_unequal_wall_depths_puts_both_ends_on_one_fold_line():
    results = [_batch_domain(0, 0.4, UP), _batch_domain(1, 0.4012, SIDE)]

    arrays = build_mesh_arrays(results, D)

    first, second = arrays.faces[0][1], arrays.faces[1][3]
    assert first != second
    assert arrays.positions[first][0] == pytest.approx(arrays.positions[second][0], abs=1e-12)
    assert arrays.positions[first][2] == pytest.approx(arrays.positions[second][2], abs=1e-12)


def test_the_console_names_the_wall_law_only_when_it_acted():
    from cftuv.envelope_production_export import receipt_console_lines

    arrays = build_mesh_arrays([_batch_domain(0, 0.4, UP), _batch_domain(1, 0.4012, SIDE)], D)
    receipt = SimpleNamespace(skipped=(), warnings=arrays.warnings, domains=(), weld_counters=arrays.weld_counters, offset_counters=())

    lines = receipt_console_lines(receipt, [])

    wall = [line for line in lines if "] WALL:" in line]
    assert wall == [
        "[CFTUV][Production] WALL: 2 wall end nodes lifted onto the fold line (0 welded), 0 without a neighbour wall, miter fallbacks 0"
    ]
    empty = SimpleNamespace(skipped=(), warnings=(), domains=(), weld_counters=(), offset_counters=())
    assert not any("] WALL:" in line for line in receipt_console_lines(empty, []))


def test_a_domain_of_the_previous_layout_has_no_walls_and_the_law_does_not_act_on_it():
    """Воркер пула, поднятый до правки, присылает вершины без поля `walls`: закон молчит, а не падает."""

    new = _domain(1, 0.4, SIDE)
    previous = SimpleNamespace(**{name: getattr(new, name) for name in ("patch_id", "positions", "normals", "refs")})

    weld = weld_vertices([_domain(0, 0.4, UP), previous], D)

    counters = _counters(weld)
    assert counters["ADAPTER_WALL_NODES_LIFTED"] == 0 and counters["ADAPTER_WALL_NODES_ALONE"] == 1
    assert _final(weld, 0, 1) == pytest.approx((1.0, 0.4, D), abs=1e-15)
