"""Предполёт источника кнопок Envelope: ребро нулевой длины называется ДО расчёта.

Полевой случай (`wall_noise_top`): вершины 6 и 19 лежат в одной точке и соединены
швом (`Merge by Distance` не выполнен). Расчёт видел это тремя разными отказами —
нулевая площадь треугольника владельца (`NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE`,
лестница метрики встаёт, не дойдя до `DEVELOPABLE`), нулевая касательная угла,
холостой веер, — и каждый вёл не туда: чинить надо ребро. Здесь оно называется
одним именем `ZERO_LENGTH_EDGE`, до анализа и до единой ступени, а виновные рёбра
остаются выделенными для осмотра. Ядро называет то же самое своим именем
(`SOURCE_EDGE_ZERO_LENGTH`, `validation_source_edges.py`) на снапшоте, дойди дело
до него другим путём.

Чего здесь НЕТ, намеренно:

* молчаливой правки. Хост не сваривает вершины сам: слияние меняет топологию
  источника, и решает его владелец;
* допуска. Нулевая длина — ТОЧНОЕ равенство координат; «короче шага решётки»
  известно только ядру после выбора закона решётки и называется там
  (`SOURCE_SNAP_NONZERO_EDGE_COLLAPSED`);
* Blender на верхнем уровне модуля: проверка идёт по BMesh через утиную
  типизацию, выделение — единственная функция, которая зовёт `bmesh`.

Область проверки — грани тех патчей (разбиение по швам), что граничат с
выделенными рёбрами: вне выделения мусор не мешает. Остальное ловит ядро —
по снапшоту каждого домена, в который ребро попадёт через дополнение цепей.
"""

from __future__ import annotations

from dataclasses import dataclass

from .analysis_topology import (
    ZERO_LENGTH_EDGE,
    faces_of_patches_touching_edges,
    find_zero_length_edges,
)

ENVELOPE_SOURCE_ZERO_LENGTH_FIX = "run Merge by Distance"


@dataclass(frozen=True, slots=True)
class SourcePreflightRefusalV1:
    """Отказ предполёта: имя, строка владельцу и виновные рёбра (по возрастанию)."""

    outcome: str
    message: str
    edge_indices: tuple[int, ...]
    vert_pairs: tuple[tuple[int, int], ...]


def zero_length_edge_refusal(bm, selected_edge_indices) -> SourcePreflightRefusalV1 | None:
    """`None`, если у патчей выделения нет ребра нулевой длины; иначе — отказ с именем."""

    faces = faces_of_patches_touching_edges(bm, selected_edge_indices)
    found = find_zero_length_edges(bm, faces)
    if not found:
        return None
    first_a, first_b = found[0][1], found[0][2]
    return SourcePreflightRefusalV1(
        ZERO_LENGTH_EDGE,
        f"{ZERO_LENGTH_EDGE}: {len(found)} edges (e.g. vertices {first_a}–{first_b}); "
        f"{ENVELOPE_SOURCE_ZERO_LENGTH_FIX}",
        tuple(item[0] for item in found),
        tuple((item[1], item[2]) for item in found),
    )


def reject_source(operator, context, obj, bm, refusal: SourcePreflightRefusalV1) -> set:
    """Отказ оператору: виновные рёбра остаются выделенными (режим рёбер), отчёт ERROR.

    Шаблон `NON_MANIFOLD_EDGE` (`_highlight_solver_preflight_issues`): снять
    выделение со всего, выделить рёбра, протолкнуть в вершины и грани.
    """

    import bmesh

    bm.edges.ensure_lookup_table()
    context.tool_settings.mesh_select_mode = (False, True, False)
    for element in (*bm.faces, *bm.edges, *bm.verts):
        element.select = False
    for index in refusal.edge_indices:
        bm.edges[index].select = True
    bm.select_flush_mode()
    bmesh.update_edit_mesh(obj.data)
    operator.report({"ERROR"}, refusal.message)
    return {"CANCELLED"}


__all__ = (
    "ENVELOPE_SOURCE_ZERO_LENGTH_FIX",
    "SourcePreflightRefusalV1",
    "reject_source",
    "zero_length_edge_refusal",
)
