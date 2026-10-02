"""Предполёт источника кнопок Envelope: ребро нулевой длины называется одним именем.

Полевой случай (`wall_noise_top`): вершины 6 и 19 лежат в одной точке и
соединены швом. Расчёт видел это тремя разными отказами — нулевая площадь
треугольника владельца (лестница метрики встаёт), нулевая касательная угла и
холостой веер, — и каждый вёл не туда. Здесь меш — маленький аналог этой беды
(пятиугольник с удвоенной вершиной и сосед по шву), без Blender: BMesh заменён
утиной копией ровно с теми полями, что читает предполёт.

Закреплено: имя и строка владельцу; точное равенство (один ulp — не ноль);
область — патчи при выделении; предполёт не чинит и не двигает ничего;
виновные рёбра остаются выделенными; обе кнопки звали предполёт ДО анализа.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import pytest

from cftuv.analysis_topology import (
    ZERO_LENGTH_EDGE,
    faces_of_patches_touching_edges,
    find_zero_length_edges,
    format_solver_input_preflight_report,
    validate_solver_input_mesh,
)
from cftuv.envelope_source_preflight import reject_source, zero_length_edge_refusal


class _Vert:
    def __init__(self, index, co):
        self.index = index
        self.co = co
        self.select = False


class _Edge:
    def __init__(self, index, verts):
        self.index = index
        self.verts = verts
        self.seam = False
        self.select = False
        self.link_faces = []


class _Face:
    def __init__(self, index, verts, edges):
        self.index = index
        self.verts = verts
        self.edges = edges
        self.select = False

    def calc_area(self):
        total = [0.0, 0.0, 0.0]
        for current, following in zip(self.verts, self.verts[1:] + self.verts[:1]):
            a, b = current.co, following.co
            total[0] += a[1] * b[2] - a[2] * b[1]
            total[1] += a[2] * b[0] - a[0] * b[2]
            total[2] += a[0] * b[1] - a[1] * b[0]
        return 0.5 * math.sqrt(sum(item * item for item in total))


class _Seq(list):
    def ensure_lookup_table(self):
        pass


class _Mesh:
    def __init__(self, coordinates, face_cycles, seams):
        self.verts = _Seq(_Vert(i, co) for i, co in enumerate(coordinates))
        self.edges = _Seq()
        self.faces = _Seq()
        by_pair = {}
        for index, cycle in enumerate(face_cycles):
            edges = []
            for a, b in zip(cycle, cycle[1:] + cycle[:1]):
                key = frozenset((a, b))
                if key not in by_pair:
                    by_pair[key] = _Edge(len(self.edges), (self.verts[a], self.verts[b]))
                    self.edges.append(by_pair[key])
                edges.append(by_pair[key])
            face = _Face(index, [self.verts[i] for i in cycle], edges)
            for edge in edges:
                edge.link_faces.append(face)
            self.faces.append(face)
        for pair in seams:
            by_pair[frozenset(pair)].seam = True
        self.edge_of = lambda a, b: by_pair[frozenset((a, b))].index
        self.select_flush_mode = lambda: None


def _wall(nudge=False):
    """Патч A (пятиугольник, 0 и 4 совпали), патч B (сосед по шву 0-4), патч C (чистый).

        A = [4, 0, 1, 2, 3]   ребро 4-0 нулевой длины и шов между A и B
        B = [0, 4, 5, 6]
        C = [7, 8, 9, 10]     далеко, ни одного нулевого ребра
    """

    twin = (math.nextafter(0.0, 1.0), 0.0, 0.0) if nudge else (0.0, 0.0, 0.0)
    coordinates = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        twin,
        (-1.0, 0.0, 0.0),
        (-1.0, 1.0, 0.0),
        (5.0, 0.0, 0.0),
        (6.0, 0.0, 0.0),
        (6.0, 1.0, 0.0),
        (5.0, 1.0, 0.0),
    ]
    cycles = [[4, 0, 1, 2, 3], [0, 4, 5, 6], [7, 8, 9, 10]]
    seams = [(0, 4), (3, 4), (0, 1), (1, 2), (2, 3), (4, 5), (5, 6), (6, 0)]
    seams += [(7, 8), (8, 9), (9, 10), (10, 7)]
    return _Mesh(coordinates, cycles, seams)


def test_the_zero_length_edge_is_found_once_with_ordered_ends():
    mesh = _wall()
    found = find_zero_length_edges(mesh, [0, 1, 2])
    assert found == ((mesh.edge_of(0, 4), 0, 4),)


def test_one_ulp_apart_is_not_zero_length():
    """Допуск нулевой: «почти ноль» — другой класс, и эвристики тут нет."""

    assert find_zero_length_edges(_wall(nudge=True), [0, 1, 2]) == ()


def test_the_refusal_names_the_class_the_count_and_an_example_and_the_fix():
    mesh = _wall()
    refusal = zero_length_edge_refusal(mesh, [mesh.edge_of(0, 1)])

    assert refusal is not None
    assert refusal.outcome == ZERO_LENGTH_EDGE == "ZERO_LENGTH_EDGE"
    assert refusal.message == (
        "ZERO_LENGTH_EDGE: 1 edges (e.g. vertices 0–4); run Merge by Distance"
    )
    assert refusal.edge_indices == (mesh.edge_of(0, 4),)
    assert refusal.vert_pairs == ((0, 4),)


def test_the_scope_is_the_patches_at_the_selection_and_a_remote_patch_is_not_in_it():
    mesh = _wall()
    # Выделение у одного патча: в области он один, и нулевое ребро на его границе названо.
    assert faces_of_patches_touching_edges(mesh, [mesh.edge_of(5, 6)]) == (1,)
    assert zero_length_edge_refusal(mesh, [mesh.edge_of(5, 6)]) is not None
    # Шов между двумя патчами тянет в область обоих.
    assert faces_of_patches_touching_edges(mesh, [mesh.edge_of(0, 4)]) == (0, 1)
    # Выделен только чистый патч C: мусор вне выделения не мешает.
    assert faces_of_patches_touching_edges(mesh, [mesh.edge_of(7, 8)]) == (2,)
    assert zero_length_edge_refusal(mesh, [mesh.edge_of(7, 8)]) is None


def test_the_preflight_repairs_nothing():
    mesh = _wall()
    before = [vert.co for vert in mesh.verts]
    counts = (len(mesh.verts), len(mesh.edges), len(mesh.faces))

    zero_length_edge_refusal(mesh, [mesh.edge_of(0, 1)])

    assert [vert.co for vert in mesh.verts] == before
    assert (len(mesh.verts), len(mesh.edges), len(mesh.faces)) == counts


def test_the_solver_preflight_carries_the_same_name():
    mesh = _wall()
    report = validate_solver_input_mesh(mesh, [0, 1, 2])
    named = [item for item in report.issues if item.code == ZERO_LENGTH_EDGE]
    assert [(item.edge_indices, item.vert_indices) for item in named] == [
        ((mesh.edge_of(0, 4),), (0, 4))
    ]
    assert "ZERO_LENGTH_EDGE:1" in format_solver_input_preflight_report(report).summary
    clean = validate_solver_input_mesh(_wall(nudge=True), [0, 1, 2])
    assert not [item for item in clean.issues if item.code == ZERO_LENGTH_EDGE]


def test_rejecting_leaves_exactly_the_offending_edges_selected_and_reports_an_error(monkeypatch):
    mesh = _wall()
    for element in (*mesh.verts, *mesh.edges, *mesh.faces):
        element.select = True
    updated = []
    stub = type(sys)("bmesh")
    stub.update_edit_mesh = lambda data: updated.append(data)
    monkeypatch.setitem(sys.modules, "bmesh", stub)

    class _Tool:
        mesh_select_mode = (True, False, False)

    class _Context:
        tool_settings = _Tool()

    class _Obj:
        data = object()

    class _Operator:
        reports = []

        def report(self, kinds, message):
            self.reports.append((kinds, message))

    refusal = zero_length_edge_refusal(mesh, [mesh.edge_of(0, 1)])
    result = reject_source(_Operator(), _Context(), _Obj(), mesh, refusal)

    assert result == {"CANCELLED"}
    assert [edge.index for edge in mesh.edges if edge.select] == [mesh.edge_of(0, 4)]
    assert not any(face.select for face in mesh.faces)
    assert _Context.tool_settings.mesh_select_mode == (False, True, False)
    assert _Operator.reports == [({"ERROR"}, refusal.message)]
    assert updated == [_Obj.data]


# --------------------------------------------------------------------------
# Обе кнопки зовут предполёт ДО анализа (порядок в тексте, как у проверки UNDO)
# --------------------------------------------------------------------------


def _first_call_lines(path: Path, owner: str, names: tuple[str, ...]) -> dict[str, int]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == owner:
            lines: dict[str, int] = {}
            for call in ast.walk(node):
                if isinstance(call, ast.Call):
                    target = call.func
                    name = target.attr if isinstance(target, ast.Attribute) else getattr(target, "id", "")
                    if name in names:
                        lines[name] = min(lines.get(name, call.lineno), call.lineno)
            return lines
    raise AssertionError(f"{owner} not found in {path}")


@pytest.mark.parametrize(
    ("file", "owner"),
    (
        ("envelope_production_operator.py", "HOTSPOTUV_OT_BuildEnvelopeDecalMesh"),
        ("operators.py", "_EnvelopeDebugBuildBase"),
    ),
)
def test_the_buttons_that_call_the_kernel_preflight_before_any_analysis(file, owner):
    package = Path(__file__).resolve().parents[1] / "cftuv"
    lines = _first_call_lines(
        package / file,
        owner,
        ("zero_length_edge_refusal", "reject_source", "get_analysis_bundle"),
    )
    assert set(lines) == {"zero_length_edge_refusal", "reject_source", "get_analysis_bundle"}
    assert lines["zero_length_edge_refusal"] < lines["reject_source"] < lines["get_analysis_bundle"]
