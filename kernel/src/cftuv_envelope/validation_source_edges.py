"""Ребро источника нулевой длины: именованный отказ снапшота.

Отдельный модуль по той же причине, что `validation_metric.py`: `validation.py`
стоит на потолке в `tests/test_architecture.py`, и потолок не поднимается.

Что ловится. Две вершины источника лежат в ОДНОЙ локальной точке и соединены
ребром (`Merge by Distance` не выполнен). Такое ребро не геометрия, а дефект
меша, и каждая ступень ниже читает его по-своему: вырожденный треугольник
владельца у измерения ширины (`NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE`, из-за
него лестница метрики встаёт, не дойдя до `DEVELOPABLE`), нулевая касательная
у угла (`BoundaryCorner incident support is degenerate`), нулевое плечо у
веера. Каждая из этих причин верна и каждая вводит в заблуждение: чинить надо
ребро, а не плоскость и не угол. Поэтому снапшот с таким ребром называет его
сам, ДО любой ступени.

Допуск нулевой намеренно: «короче шага решётки» — другой класс (его шаг
известен только после выбора закона решётки, то есть ПОСЛЕ снапшота) и
называется своим исходом (`SOURCE_SNAP_NONZERO_EDGE_COLLAPSED`). Сравнение —
точное равенство `float`: координаты хоста уже двоичные числа, никакого «почти»
здесь нет, и эвристики, меняющей ответ, тоже.

Без координат (`UnavailableSourcePositionV1`, фикстуры EC0) проверять нечего.
"""

from __future__ import annotations

from .contracts.analysis import AnalysisSnapshotV1
from .numeric import LocalPoint3V1
from .validation_issues import ValidationCode, ValidationIssue, add_issue


SOURCE_EDGE_ZERO_LENGTH = ValidationCode.SOURCE_EDGE_ZERO_LENGTH.value


def _coordinates(position: LocalPoint3V1) -> tuple[float, float, float]:
    return (position.x, position.y, position.z)


def source_edge_zero_length_issues(
    snapshot: AnalysisSnapshotV1,
) -> tuple[ValidationIssue, ...]:
    """По одной записи на ребро, чьи концы совпадают позицией; порядок — по ID."""

    positions = {
        vertex.vertex_id: _coordinates(vertex.position)
        for vertex in snapshot.source_vertices
        if isinstance(vertex.position, LocalPoint3V1)
    }
    issues: list[ValidationIssue] = []
    for edge in sorted(snapshot.surface_ir.source_edges, key=lambda item: str(item.edge_id)):
        first = positions.get(edge.vertex_a_id)
        if first is None or first != positions.get(edge.vertex_b_id):
            continue
        add_issue(
            issues,
            ValidationCode.SOURCE_EDGE_ZERO_LENGTH,
            ("surface_ir", "source_edges", str(edge.edge_id), "length"),
            f"{SOURCE_EDGE_ZERO_LENGTH}: edge joins vertices {edge.vertex_a_id} and "
            f"{edge.vertex_b_id} at one local position; weld them (Merge by "
            "Distance) before evaluation",
        )
    return tuple(issues)
