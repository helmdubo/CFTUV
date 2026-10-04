"""Мгновенное превью ширины декали и автомат «Adjust Decal Width» (хост без Blender).

Утверждения среза DECAL-WIDTH-LIVE (мгновенная часть), каждое стоит на числе:

1. ПЛОСКИЙ ПАТЧ: линия отступа лежит ровно на ширине от исходной прямой (до 1e-9) в произвольной плоскости,
   угол пути — митра (расстояние до ОБЕИХ сторон равно ширине), прямая через много граней — одна линия;
2. ИЗОГНУТЫЙ ПАТЧ (граненый цилиндр, развёртываемая поверхность): точки лежат НА гранях меша (до 1e-9), а их
   положение отличается от аналитического отступа по окружности не больше допуска растяжения запроса;
3. ГРАНИЦА ПАТЧА: ширина больше патча обрывает луч на границе и называется `PREVIEW_CLIPPED_AT_PATCH_BOUNDARY`;
   острый угол режется пределом митры (`PREVIEW_MITRE_LIMITED`), сторона без грани — `PREVIEW_SIDE_WITHOUT_FACE`;
   ничего из этого не молчит;
4. ДВА ПАТЧА через шов: у каждого патча своя линия, по одной с каждой стороны шва;
5. ЦЕНА: превью на границе патча из сотен рёбер считается за миллисекунды (замер на `building` — в смоке);
6. АВТОМАТ ИНСТРУМЕНТА: радиальное смещение, Ctrl, Shift, число с клавиатуры, подтверждение, отмена возвращает
   прежнюю ширину, после исхода события не принимаются.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_request_policy import DEFAULT_ENVELOPE_STRETCH_BUDGET  # noqa: E402
from cftuv.envelope_width_adjust import (  # noqa: E402
    KIND_BACKSPACE,
    KIND_CANCEL,
    KIND_CONFIRM,
    KIND_DIGIT,
    KIND_MOVE,
    KIND_POINT,
    PHASE_ACTIVE,
    PHASE_CANCELLED,
    PHASE_CONFIRMED,
    PRECISION,
    WidthAdjustSessionV1,
    WidthEventV1,
)
from cftuv.envelope_width_preview import (  # noqa: E402
    OUTCOME_BEVEL_JOIN,
    OUTCOME_CLIPPED,
    OUTCOME_MITRE_LIMITED,
    OUTCOME_NO_FACE,
    PREVIEW_BINARY64_V1,
    build_preview_inputs,
    chain_centroid,
    compute_width_preview,
)
from cftuv.surface_ir import (  # noqa: E402
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
)

TOLERANCE = 1e-9


# --------------------------------------------------------------------------
# Поверхности
# --------------------------------------------------------------------------


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(a, k):
    return tuple(x * k for x in a)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _unit(a):
    size = math.sqrt(_dot(a, a))
    return tuple(x / size for x in a)


class _Mesh:
    """Собиратель `PatchSurfaceIR`: вершины, грани по циклам, рёбра с гранями."""

    def __init__(self):
        self.vertices = []
        self.faces = []
        self._edge_of = {}
        self._edge_ends = {}
        self._edge_faces = {}

    def vertex(self, position):
        self.vertices.append(SourceVertex(len(self.vertices), tuple(float(item) for item in position)))
        return len(self.vertices) - 1

    def edge(self, a, b):
        key = tuple(sorted((a, b)))
        if key not in self._edge_of:
            self._edge_of[key] = len(self._edge_of)
            self._edge_ends[self._edge_of[key]] = key
        return self._edge_of[key]

    def face(self, patch_id, cycle):
        points = [self.vertices[item].position for item in cycle]
        normal = _unit(_cross(_sub(points[2], points[0]), _sub(points[3 % len(points)], points[1])))
        edges = tuple(self.edge(cycle[i], cycle[(i + 1) % len(cycle)]) for i in range(len(cycle)))
        face_id = len(self.faces)
        self.faces.append(SourceFace(face_id, patch_id, tuple(cycle), edges, normal, ()))
        for edge in edges:
            self._edge_faces.setdefault(edge, []).append(face_id)
        return face_id

    def surface(self):
        edges = tuple(
            SourceEdge(edge, self._edge_ends[edge], tuple(sorted(self._edge_faces[edge])))
            for edge in sorted(self._edge_ends)
        )
        return PatchSurfaceIR(
            SourceRevision("preview", "digest"), tuple(self.vertices), edges, tuple(self.faces), ()
        )


def _grid(nx, ny, size, *, origin=(0.0, 0.0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0), patch=lambda i, j: 0):
    mesh = _Mesh()
    ids = {}
    for j in range(ny + 1):
        for i in range(nx + 1):
            ids[i, j] = mesh.vertex(_add(origin, _add(_scale(u, i * size), _scale(v, j * size))))
    for j in range(ny):
        for i in range(nx):
            mesh.face(patch(i, j), (ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]))
    return mesh, ids


def _edges_between(mesh, ids, cells):
    return [mesh.edge(ids[a], ids[b]) for a, b in cells]


def _bottom(mesh, ids, nx):
    return _edges_between(mesh, ids, [((i, 0), (i + 1, 0)) for i in range(nx)])


def _line_distance(point, origin, direction):
    """Расстояние от точки до ПРЯМОЙ."""

    along = _dot(_sub(point, origin), direction)
    foot = _add(origin, _scale(direction, along))
    return math.sqrt(_dot(_sub(point, foot), _sub(point, foot)))


TILTED = dict(
    origin=(0.3, -1.2, 2.0),
    u=_unit((1.0, 0.4, 0.2)),
    v=_unit(_cross((0.0, 0.0, 1.0), _unit((1.0, 0.4, 0.2)))),
)
TILTED["v"] = _unit(_cross(_cross((0.3, 0.2, 1.0), TILTED["u"]), TILTED["u"]))


# --------------------------------------------------------------------------
# 1. Плоский патч
# --------------------------------------------------------------------------


@pytest.mark.parametrize("frame", ({}, TILTED), ids=("axis", "tilted"))
@pytest.mark.parametrize("width", (0.05, 0.31, 1.0, 2.75))
def test_a_straight_chain_offsets_by_exactly_the_width_and_merges_into_one_line(frame, width):
    mesh, ids = _grid(12, 8, 0.5, **frame)
    inputs = build_preview_inputs(mesh.surface(), [(0, _bottom(mesh, ids, 12))])

    preview = compute_width_preview(inputs, width)

    assert preview.method == PREVIEW_BINARY64_V1 == "PREVIEW_BINARY64_V1"
    assert preview.outcomes == ()
    line = preview.polylines[0]
    assert len(line) == 2  # двенадцать рёбер на одной прямой: промежуточных вершин нет
    u = frame.get("u", (1.0, 0.0, 0.0))
    v = frame.get("v", (0.0, 1.0, 0.0))
    origin = frame.get("origin", (0.0, 0.0, 0.0))
    for point in line:
        assert abs(_line_distance(point, origin, u) - width) <= TOLERANCE
        assert abs(_dot(_sub(point, origin), v) - width) <= TOLERANCE
    assert abs(math.dist(line[0], line[1]) - 12 * 0.5) <= TOLERANCE  # параллельный перенос, длина цепи
    # Торцы: от исходной точки цепи к концу линии отступа, длиной в ширину.
    caps = preview.polylines[1:]
    assert len(caps) == 2
    assert all(abs(math.dist(*cap) - width) <= TOLERANCE for cap in caps)


@pytest.mark.parametrize("frame", ({}, TILTED), ids=("axis", "tilted"))
def test_a_corner_of_the_chain_is_a_mitre_at_the_width_from_both_sides(frame):
    mesh, ids = _grid(10, 10, 0.5, **frame)
    cells = [((i, 0), (i + 1, 0)) for i in range(10)] + [((10, j), (10, j + 1)) for j in range(10)]
    inputs = build_preview_inputs(mesh.surface(), [(0, _edges_between(mesh, ids, cells))])
    u = frame.get("u", (1.0, 0.0, 0.0))
    v = frame.get("v", (0.0, 1.0, 0.0))
    origin = frame.get("origin", (0.0, 0.0, 0.0))
    width = 0.8

    preview = compute_width_preview(inputs, width)

    line = preview.polylines[0]
    assert len(line) == 3  # два прямых участка и один угол
    corner = line[1]
    right = _add(origin, _scale(u, 5.0))  # нижняя сторона: прямая по u, правая: прямая по v через (5, 0)
    assert abs(_line_distance(corner, origin, u) - width) <= TOLERANCE
    assert abs(_line_distance(corner, right, v) - width) <= TOLERANCE
    assert preview.outcome(OUTCOME_BEVEL_JOIN) == 0


def test_the_four_sides_of_a_patch_make_one_closed_loop_inward():
    mesh, ids = _grid(6, 4, 0.5)
    cells = (
        [((i, 0), (i + 1, 0)) for i in range(6)]
        + [((6, j), (6, j + 1)) for j in range(4)]
        + [((i, 4), (i + 1, 4)) for i in range(6)]
        + [((0, j), (0, j + 1)) for j in range(4)]
    )
    inputs = build_preview_inputs(mesh.surface(), [(0, _edges_between(mesh, ids, cells))])

    preview = compute_width_preview(inputs, 0.25)

    closed = [line for line in preview.polylines if len(line) > 2]
    assert len(closed) == 1 and closed[0][0] == closed[0][-1]
    xs = sorted({round(point[0], 12) for point in closed[0]})
    ys = sorted({round(point[1], 12) for point in closed[0]})
    assert xs == [0.25, 2.75] and ys == [0.25, 1.75]


def test_the_lift_moves_every_point_along_the_face_normal_and_nothing_else():
    mesh, ids = _grid(4, 4, 1.0, **TILTED)
    inputs = build_preview_inputs(mesh.surface(), [(0, _bottom(mesh, ids, 4))])
    plain = compute_width_preview(inputs, 0.5)
    lifted = compute_width_preview(inputs, 0.5, lift=0.02)
    normal = _unit(_cross(TILTED["u"], TILTED["v"]))

    for line, higher in zip(plain.polylines, lifted.polylines):
        for point, up in zip(line, higher):
            assert abs(math.dist(point, up) - 0.02) <= TOLERANCE
            assert abs(abs(_dot(_sub(up, point), normal)) - 0.02) <= TOLERANCE


def test_the_same_input_gives_the_same_answer():
    mesh, ids = _grid(6, 6, 0.5)
    inputs = build_preview_inputs(mesh.surface(), [(0, _bottom(mesh, ids, 6))])

    first = compute_width_preview(inputs, 0.4)
    second = compute_width_preview(inputs, 0.4)

    assert first.polylines == second.polylines and first.outcomes == second.outcomes


# --------------------------------------------------------------------------
# 2. Изогнутый патч
# --------------------------------------------------------------------------


def _cylinder(segments, stacks, *, radius=1.0, height=0.5):
    """Полуцилиндр `theta` в `[0, pi]`: развёртываемая поверхность из четырёхугольников."""

    mesh = _Mesh()
    ids = {}
    for j in range(stacks + 1):
        for i in range(segments + 1):
            angle = math.pi * i / segments
            ids[i, j] = mesh.vertex((radius * math.cos(angle), radius * math.sin(angle), height * j))
    for j in range(stacks):
        for i in range(segments):
            mesh.face(0, (ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]))
    return mesh, ids


@pytest.mark.parametrize("width", (0.1, 0.4, 1.3, 2.9))
def test_on_a_curved_patch_the_points_lie_on_the_mesh_within_the_stretch_tolerance(width):
    segments, stacks, radius = 48, 3, 1.0
    mesh, ids = _cylinder(segments, stacks, radius=radius)
    chain = _edges_between(mesh, ids, [((0, j), (0, j + 1)) for j in range(stacks)])
    inputs = build_preview_inputs(mesh.surface(), [(0, chain)])

    preview = compute_width_preview(inputs, width)

    assert preview.outcomes == ()
    line = preview.polylines[0]
    step = math.pi / segments
    inner = radius * math.cos(step / 2.0)
    tolerance = float(DEFAULT_ENVELOPE_STRETCH_BUDGET) * width
    for point in line:
        angle = math.atan2(point[1], point[0])
        facet = min(segments - 1, int(angle / step))
        middle = (facet + 0.5) * step
        normal = (math.cos(middle), math.sin(middle), 0.0)
        assert abs(_dot(point, normal) - inner) <= TOLERANCE  # точка лежит на плоскости грани меша
        assert abs(radius * angle - width) <= tolerance  # а её положение — в допуске растяжения запроса
    assert abs(line[0][2]) <= TOLERANCE and abs(line[-1][2] - 3 * 0.5) <= TOLERANCE


def test_a_width_beyond_the_curved_patch_is_clipped_at_its_boundary_and_named():
    segments, radius = 24, 1.0
    mesh, ids = _cylinder(segments, 2, radius=radius)
    chain = _edges_between(mesh, ids, [((0, j), (0, j + 1)) for j in range(2)])
    inputs = build_preview_inputs(mesh.surface(), [(0, chain)])

    preview = compute_width_preview(inputs, math.pi * radius * 1.5)

    assert preview.outcome(OUTCOME_CLIPPED) >= 2
    last = preview.polylines[0][-1]
    assert abs(last[0] + radius) <= 1e-6 and abs(last[1]) <= 1e-6  # край патча при theta = pi


# --------------------------------------------------------------------------
# 3. Ничего не молчит
# --------------------------------------------------------------------------


def test_a_width_beyond_a_planar_patch_is_clipped_and_named():
    mesh, ids = _grid(4, 4, 0.5)
    inputs = build_preview_inputs(mesh.surface(), [(0, _bottom(mesh, ids, 4))])

    preview = compute_width_preview(inputs, 50.0)

    assert preview.outcome(OUTCOME_CLIPPED) > 0
    for line in preview.polylines:
        for point in line:
            assert -1e-9 <= point[1] <= 2.0 + 1e-9  # не дальше противоположного края патча
    assert "PREVIEW_CLIPPED_AT_PATCH_BOUNDARY" in preview.status_text()
    assert "preview, not final" in preview.status_text()


def test_a_very_sharp_corner_is_cut_by_the_mitre_limit_and_named():
    """Путь ломается под 20 градусов, а грани лежат С ВНЕШНЕЙ стороны излома: митра ушла бы в шип."""

    turn = math.radians(20.0)
    mesh = _Mesh()
    a = mesh.vertex((4.0, 0.0, 0.0))
    b = mesh.vertex((0.0, 0.0, 0.0))
    p = mesh.vertex((0.0, -1.0, 0.0))
    q = mesh.vertex((4.0, -1.0, 0.0))
    c = mesh.vertex((4.0 * math.cos(turn), 4.0 * math.sin(turn), 0.0))
    away = (-math.sin(turn), math.cos(turn), 0.0)
    r = mesh.vertex(_add((4.0 * math.cos(turn), 4.0 * math.sin(turn), 0.0), away))
    s = mesh.vertex(away)
    mesh.face(0, (a, b, p, q))
    mesh.face(0, (b, c, r, s))
    inputs = build_preview_inputs(mesh.surface(), [(0, [mesh.edge(a, b), mesh.edge(b, c)])])

    preview = compute_width_preview(inputs, 0.05)

    assert preview.outcome(OUTCOME_MITRE_LIMITED) == 1
    assert preview.outcome(OUTCOME_BEVEL_JOIN) == 1
    assert len(preview.polylines[0]) == 4  # два конца и две точки фаски вместо шипа
    assert OUTCOME_MITRE_LIMITED in preview.status_text()


def test_a_selected_edge_without_a_face_in_the_patch_is_named_and_never_dropped_silently():
    mesh, ids = _grid(3, 3, 1.0, patch=lambda i, j: 0)
    other = _bottom(mesh, ids, 3)
    inputs = build_preview_inputs(mesh.surface(), [(7, other)])  # у патча 7 граней нет вовсе

    preview = compute_width_preview(inputs, 0.3)

    assert preview.polylines == ()
    assert preview.outcome(OUTCOME_NO_FACE) == 3
    assert OUTCOME_NO_FACE in preview.status_text()


# --------------------------------------------------------------------------
# 4. Два патча
# --------------------------------------------------------------------------


def test_a_seam_between_two_patches_gets_one_line_on_each_side():
    mesh, ids = _grid(4, 2, 1.0, patch=lambda i, j: 0 if i < 2 else 1)
    seam = _edges_between(mesh, ids, [((2, 0), (2, 1)), ((2, 1), (2, 2))])
    inputs = build_preview_inputs(mesh.surface(), [(0, seam), (1, seam)])

    preview = compute_width_preview(inputs, 0.4)

    lines = [line for line in preview.polylines if len(line) == 2 and line[0][0] == line[1][0]]
    assert sorted(round(line[0][0], 12) for line in lines) == [1.6, 2.4]
    assert all(abs(abs(line[0][1] - line[1][1]) - 2.0) <= TOLERANCE for line in lines)
    assert chain_centroid(inputs) == (2.0, 1.0, 0.0)


def test_the_marching_does_not_cross_into_another_patch():
    mesh, ids = _grid(4, 1, 1.0, patch=lambda i, j: 0 if i < 2 else 1)
    chain = _edges_between(mesh, ids, [((0, 0), (0, 1))])
    inputs = build_preview_inputs(mesh.surface(), [(0, chain)])

    preview = compute_width_preview(inputs, 3.0)

    assert preview.outcome(OUTCOME_CLIPPED) > 0
    assert max(point[0] for line in preview.polylines for point in line) <= 2.0 + 1e-9


# --------------------------------------------------------------------------
# 5. Цена
# --------------------------------------------------------------------------


def test_the_preview_of_a_patch_boundary_with_hundreds_of_edges_costs_milliseconds():
    nx = 150
    mesh, ids = _grid(nx, 40, 0.1)
    cells = (
        [((i, 0), (i + 1, 0)) for i in range(nx)]
        + [((nx, j), (nx, j + 1)) for j in range(40)]
        + [((i, 40), (i + 1, 40)) for i in range(nx)]
        + [((0, j), (0, j + 1)) for j in range(40)]
    )
    inputs = build_preview_inputs(mesh.surface(), [(0, _edges_between(mesh, ids, cells))])
    assert inputs.edges == 2 * nx + 80

    compute_width_preview(inputs, 0.2)  # прогрев
    best = min(compute_width_preview(inputs, 0.2).seconds for _ in range(5))

    assert best < 0.05, f"{best * 1000:.1f} ms"


# --------------------------------------------------------------------------
# 6. Автомат инструмента
# --------------------------------------------------------------------------


def _session(**overrides):
    values = dict(pivot=(100.0, 100.0), start_mouse=(200.0, 100.0), metres_per_pixel=0.002)
    values.update(overrides)
    return WidthAdjustSessionV1(0.25, **values)


def _move(x, y=100.0, **modifiers):
    return WidthEventV1(KIND_MOVE, x, y, **modifiers)


def test_moving_away_from_the_pivot_widens_in_proportion_to_the_path_and_back_narrows():
    session = _session()

    assert session.handle(_move(250.0)).width == pytest.approx(0.25 + 50 * 0.002)
    assert session.handle(_move(300.0)).width == pytest.approx(0.25 + 100 * 0.002)
    assert session.handle(_move(200.0)).width == pytest.approx(0.25)
    assert session.handle(_move(0.0)).width == pytest.approx(0.25)  # по другую сторону оси путь снова растёт
    assert session.phase == PHASE_ACTIVE and session.events == 4


def test_the_width_never_goes_below_the_minimum_and_does_not_hide_a_dead_zone():
    session = _session(metres_per_pixel=0.01)

    session.handle(_move(105.0))  # путь к оси на 95 px по 0.01: ширина ушла бы ниже нуля
    assert session.width == 0.0
    # мёртвой зоны нет: уход от оси на 10 px растёт сразу, а не после «долга» в 0.7
    assert session.handle(_move(115.0)).width == pytest.approx(10 * 0.01)


def test_control_snaps_to_the_step_and_shift_makes_it_precise():
    session = _session()

    assert session.handle(_move(203.0, ctrl=True)).width == pytest.approx(0.26)  # 0.256 -> шаг 0.01
    fine = _session()
    fine.handle(_move(300.0, shift=True))
    assert fine.width == pytest.approx(0.25 + 100 * 0.002 * PRECISION)


def test_a_typed_number_replaces_the_drag_until_the_outcome():
    session = _session()
    session.handle(_move(300.0))
    for char in "0":
        session.handle(WidthEventV1(KIND_DIGIT, char=char))
    session.handle(WidthEventV1(KIND_POINT))
    for char in "75":
        session.handle(WidthEventV1(KIND_DIGIT, char=char))

    assert session.width == pytest.approx(0.75) and session.numeric
    assert "0.75|" in session.header()
    session.handle(_move(900.0))  # мышь ширину не меняет, пока набирается число
    assert session.width == pytest.approx(0.75)
    session.handle(WidthEventV1(KIND_BACKSPACE))
    assert session.width == pytest.approx(0.7)
    session.handle(WidthEventV1(KIND_POINT))
    assert "ignored" in session.header()  # вторая точка названа в заголовке, а не проглочена


def test_confirm_and_cancel_end_the_session_and_cancel_restores_the_start_width():
    confirmed = _session()
    confirmed.handle(_move(300.0))
    done = confirmed.handle(WidthEventV1(KIND_CONFIRM))
    assert done.phase == PHASE_CONFIRMED and confirmed.changed
    assert confirmed.handle(_move(500.0)).width == pytest.approx(0.25 + 100 * 0.002)  # события после исхода не принимаются

    cancelled = _session()
    cancelled.handle(_move(400.0))
    assert cancelled.width > 0.25
    result = cancelled.handle(WidthEventV1(KIND_CANCEL))
    assert result.phase == PHASE_CANCELLED and result.width == 0.25 and not cancelled.changed


def test_the_header_names_the_width_a_preview_and_the_unit():
    session = _session()

    assert session.header().startswith("Decal width: 0.250 m (preview)")
    with pytest.raises(ValueError):
        session.handle(WidthEventV1("DRAG"))
    with pytest.raises(ValueError):
        _session(metres_per_pixel=0.0)
