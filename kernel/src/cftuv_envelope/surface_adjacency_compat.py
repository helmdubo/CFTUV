"""Совместимый вывод смежности из снапшота БЕЗ таблицы. Никогда не власть.

Зачем. Одиннадцать полевых снапшотов записаны до `SurfaceAdjacencyIRV1`, их
байты неподвижны, а пересборка требует Blender. Чтобы читать их новым кодом,
нужен путь, выводящий смежность из того, что в них уже есть: троек вершин
треугольников и `physical_edge_ids`.

Границы власти взяты с AUTH P0-4-Q-06 (compat-исключение чтения):

1. Путь ТОЛЬКО на чтении. Единственный производитель таблицы — хост, у
   которого есть bmesh; здесь нет и не будет ни одной величины, которой нет в
   payload.
2. Срабатывание — ПОД СЧЁТЧИКОМ (`compat_counters`), плюс храповик
   «снапшотов без таблицы осталось N» только вниз; при нуле путь подлежит
   изъятию. Sunset записан здесь же и в `DECISIONS.md`.
3. Три конфигурации подлинно неоднозначны, и на них путь ОТКАЗЫВАЕТ
   ИМЕНОВАННО, а не выбирает:
   * patch-scoped срез — сосед вне скоупа неотличим от границы меша;
   * немногообразный веер — «противоположной» стороны не существует;
   * повтор вершины в `vertex_cycle` грани — дверь fail-open, названная в
     AUTH развилок S0/S2: треугольники повтор запрещают, циклы граней нет.
4. Вывод здесь — НЕЗАВИСИМАЯ ПЕРЕВЫВОДКА того же факта, что строит хост.
   Совпадение двух выводов и есть приёмка; расхождение — дефект, а не выбор
   одной из версий.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .contracts.surface import PatchSurfaceIRV1
from .contracts.surface_adjacency import (
    SURFACE_ADJACENCY_IR_SCHEMA_V1,
    SideBoundaryReasonV1,
    SideBoundaryV1,
    SurfaceAdjacencyIRV1,
    SurfaceAdjacencyScopeV1,
    SurfaceOrientationLawV1,
    TriangleSideKindV1,
    TriangleSideRefV1,
    TriangleSideV1,
    VertexFanUnavailableReasonV1,
    VertexFanUnavailableV1,
    VertexFanV1,
    canonical_fan_order,
)


class SurfaceAdjacencyCompatOutcome(str, Enum):
    """Именованные отказы совместимого вывода. Молчаливых веток нет."""

    COMPAT_REPEATED_VERTEX_IN_SOURCE_FACE_CYCLE = (
        "COMPAT_REPEATED_VERTEX_IN_SOURCE_FACE_CYCLE"
    )
    COMPAT_NON_MANIFOLD_TRIANGLE_SIDE = "COMPAT_NON_MANIFOLD_TRIANGLE_SIDE"
    COMPAT_PATCH_SCOPED_BOUNDARY_UNDECIDABLE = (
        "COMPAT_PATCH_SCOPED_BOUNDARY_UNDECIDABLE"
    )
    COMPAT_ORIENTATION_NOT_CONSISTENT_WITH_SOURCE_FACE_CYCLE = (
        "COMPAT_ORIENTATION_NOT_CONSISTENT_WITH_SOURCE_FACE_CYCLE"
    )


class SurfaceAdjacencyCompatRefused(ValueError):
    """Совместимый вывод не может назвать смежность честно."""

    def __init__(self, outcome: SurfaceAdjacencyCompatOutcome, details: str) -> None:
        self.outcome = outcome
        self.details = str(details)
        super().__init__(f"{outcome.value}: {details}")


@dataclass
class _Counters:
    reads: int = 0
    accepted: int = 0
    refused: int = 0

    def reset(self) -> None:
        self.reads = 0
        self.accepted = 0
        self.refused = 0


_COUNTERS = _Counters()
_REFUSALS: dict[str, int] = {}


def compat_counters() -> dict:
    """Снимок счётчиков совместимого пути. Читается приёмкой и храповиком."""

    return {
        "reads": _COUNTERS.reads,
        "accepted": _COUNTERS.accepted,
        "refused": _COUNTERS.refused,
        "refusals_by_outcome": dict(sorted(_REFUSALS.items())),
    }


def reset_compat_counters() -> None:
    _COUNTERS.reset()
    _REFUSALS.clear()


def _refuse(outcome: SurfaceAdjacencyCompatOutcome, details: str):
    _COUNTERS.refused += 1
    _REFUSALS[outcome.value] = _REFUSALS.get(outcome.value, 0) + 1
    return SurfaceAdjacencyCompatRefused(outcome, details)


def _check_face_cycles(surface: PatchSurfaceIRV1) -> None:
    """Повтор вершины в цикле грани — дверь fail-open, а не рабочий случай."""

    for face in surface.source_faces:
        if len(set(face.vertex_cycle)) != len(face.vertex_cycle):
            raise _refuse(
                SurfaceAdjacencyCompatOutcome.COMPAT_REPEATED_VERTEX_IN_SOURCE_FACE_CYCLE,
                f"face={face.face_id.value}",
            )


def _side_records(surface: PatchSurfaceIRV1) -> dict:
    records = {}
    for triangle in surface.surface_triangles:
        for ordinal in range(3):
            # Ядерный закон нумерации: сторона `i` — пара `(v[i], v[i + 1])`.
            records[(triangle.triangle_id, ordinal)] = (
                triangle.vertex_ids[ordinal],
                triangle.vertex_ids[(ordinal + 1) % 3],
                triangle.physical_edge_ids[ordinal],
            )
    return records


def _grouped_by_pair(records: dict) -> dict:
    grouped: dict = {}
    for key, (first, second, _edge) in records.items():
        grouped.setdefault(frozenset((first, second)), []).append(key)
    return {
        pair: tuple(sorted(keys, key=lambda item: (item[0].value, item[1])))
        for pair, keys in grouped.items()
    }


def _opposites(records, grouped, scope) -> dict:
    result: dict = {}
    for keys in grouped.values():
        if len(keys) > 2:
            raise _refuse(
                SurfaceAdjacencyCompatOutcome.COMPAT_NON_MANIFOLD_TRIANGLE_SIDE,
                f"пара концов держит {len(keys)} сторон треугольников",
            )
        if len(keys) == 1:
            if scope is not SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH:
                raise _refuse(
                    SurfaceAdjacencyCompatOutcome.COMPAT_PATCH_SCOPED_BOUNDARY_UNDECIDABLE,
                    f"сторона {keys[0][0].value}:{keys[0][1]} без пары: в срезе "
                    "граница меша неотличима от границы запроса",
                )
            result[keys[0]] = SideBoundaryV1(reason=SideBoundaryReasonV1.MESH_BOUNDARY)
            continue
        left, right = keys
        if (
            records[left][0] != records[right][1]
            or records[left][1] != records[right][0]
        ):
            raise _refuse(
                SurfaceAdjacencyCompatOutcome.COMPAT_ORIENTATION_NOT_CONSISTENT_WITH_SOURCE_FACE_CYCLE,
                f"стороны {left[0].value}:{left[1]} и {right[0].value}:{right[1]} "
                "проходят общую пару концов в одну сторону",
            )
        result[left] = TriangleSideRefV1(triangle_id=right[0], side_ordinal=right[1])
        result[right] = TriangleSideRefV1(triangle_id=left[0], side_ordinal=left[1])
    return result


def _sides(records, opposites) -> frozenset:
    return frozenset(
        TriangleSideV1(
            side=TriangleSideRefV1(triangle_id=key[0], side_ordinal=key[1]),
            from_vertex_id=first,
            to_vertex_id=second,
            kind=(
                TriangleSideKindV1.FACE_DIAGONAL
                if edge_id is None
                else TriangleSideKindV1.SOURCE_EDGE
            ),
            physical_edge_id=edge_id,
            opposite=opposites[key],
        )
        for key, (first, second, edge_id) in records.items()
    )


def _incidence(surface: PatchSurfaceIRV1) -> dict:
    incidence: dict = {}
    for triangle in surface.surface_triangles:
        for vertex_id in triangle.vertex_ids:
            incidence.setdefault(vertex_id, set()).add(triangle.triangle_id)
    return incidence


def _links(vertex_id, triangle_ids, records, opposites):
    links = {triangle_id: set() for triangle_id in triangle_ids}
    reasons: set[str] = set()
    for triangle_id in triangle_ids:
        for ordinal in range(3):
            key = (triangle_id, ordinal)
            first, second, _edge = records[key]
            if vertex_id not in (first, second):
                continue
            opposite = opposites[key]
            if isinstance(opposite, TriangleSideRefV1):
                links[triangle_id].add(opposite.triangle_id)
            else:
                reasons.add(opposite.reason.value)
    return links, reasons


def _unavailable_reason(reasons) -> VertexFanUnavailableReasonV1:
    """Причина неупорядоченного веера по объявленному старшинству.

    `MESH_BOUNDARY` в этот список не входит НАМЕРЕННО: край меша веер не
    ломает, он его только размыкает. Веер, не сложившийся при одних лишь
    краевых сторонах, — защемлённая вершина («бабочка»), то есть
    немногообразие, и называется им, а не краем.
    """

    for candidate in (
        VertexFanUnavailableReasonV1.NON_MANIFOLD_FAN,
        VertexFanUnavailableReasonV1.OUTSIDE_REQUESTED_SCOPE,
        VertexFanUnavailableReasonV1.HOST_DECLARED_UNKNOWN,
    ):
        if candidate.value in reasons:
            return candidate
    return VertexFanUnavailableReasonV1.NON_MANIFOLD_FAN


def _fans(surface, records, opposites) -> frozenset:
    fans = []
    for vertex_id, incident in _incidence(surface).items():
        links, reasons = _links(
            vertex_id,
            sorted(incident, key=lambda item: item.value),
            records,
            opposites,
        )
        ordered = canonical_fan_order(links)
        if ordered is None:
            fans.append(
                VertexFanUnavailableV1(
                    vertex_id=vertex_id,
                    reason=_unavailable_reason(reasons),
                )
            )
            continue
        order, closed = ordered
        fans.append(
            VertexFanV1(
                vertex_id=vertex_id,
                ordered_triangle_ids=order,
                is_closed=closed,
            )
        )
    return frozenset(fans)


def read_surface_adjacency(
    surface: PatchSurfaceIRV1,
    *,
    scope: SurfaceAdjacencyScopeV1,
    scope_patch_ids=frozenset(),
) -> SurfaceAdjacencyIRV1:
    """Вывести смежность из снапшота без таблицы — или отказать именованно."""

    _COUNTERS.reads += 1
    _check_face_cycles(surface)
    records = _side_records(surface)
    opposites = _opposites(records, _grouped_by_pair(records), scope)
    table = SurfaceAdjacencyIRV1(
        schema_version=SURFACE_ADJACENCY_IR_SCHEMA_V1,
        source_revision=surface.source_revision,
        scope=scope,
        scope_patch_ids=frozenset(scope_patch_ids),
        orientation_law=(
            SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
        ),
        triangle_sides=_sides(records, opposites),
        vertex_fans=_fans(surface, records, opposites),
    )
    _COUNTERS.accepted += 1
    return table


# Храповик sunset. Число полевых снапшотов, которые ещё не несут собственной
# таблицы смежности и потому требуют этого пути. Двигается ТОЛЬКО вниз — по
# мере пересборки фикстур экспортёром, умеющим писать `SurfaceAdjacencyIRV1`.
# При нуле совместимый путь подлежит изъятию вместе с этим модулем.
SNAPSHOTS_WITHOUT_ADJACENCY_TABLE = 11


__all__ = (
    "SNAPSHOTS_WITHOUT_ADJACENCY_TABLE",
    "SurfaceAdjacencyCompatOutcome",
    "SurfaceAdjacencyCompatRefused",
    "compat_counters",
    "read_surface_adjacency",
    "reset_compat_counters",
)
