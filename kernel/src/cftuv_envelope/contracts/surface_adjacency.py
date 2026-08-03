"""Таблица смежности треугольников поверхности — обёртка, а не расширение.

Почему обёртка. `PatchSurfaceIRV1` несёт треугольники и физические рёбра, но
не несёт того, ЧТО лежит по другую сторону стороны треугольника. Расширение
`SourceEdgeV1` было отвергнуто (AUTH развилок S0/S2, 2026-08-03): оно ломает
codec и одиннадцать полевых фикстур, пересборка которых требует Blender, а
triangle adjacency всё равно не даёт — сторона треугольника и физическое ребро
это разные вещи, и диагонали тесселяции физических рёбер не имеют вовсе.

Что здесь названо и чего нет в снапшоте:

* СКОУП. `FULL_SOURCE_MESH` против `REQUEST_SCOPED_PATCH_SET`. Без него сосед
  вне среза неотличим от границы меша: срезанный `SourceEdgeV1` называет грани
  вне среза, и читатель не может сказать, кончился меш или кончился запрос.
  Поэтому `OUTSIDE_REQUESTED_SCOPE` — отдельная причина, а под
  `FULL_SOURCE_MESH` она запрещена вовсе (иначе полный меш объявлялся бы
  полным, будучи срезом).
* ОРИЕНТАЦИЯ. Единственный носитель ориентации в снапшоте — binary64-нормаль,
  то есть число с плавающей точкой, которое ничего не доказывает.
  `CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1` — закон КОМБИНАТОРНЫЙ: две стороны,
  склеенные друг с другом, обязаны проходить общую пару концов в
  ПРОТИВОПОЛОЖНЫХ направлениях. Это проверяется на идентификаторах вершин и не
  зависит ни от одной координаты.
* НЕМНОГООБРАЗНЫЕ ВЕЕРА. `NON_MANIFOLD_FAN` называется, а не выбирается: у
  стороны с тремя и более носителями «противоположной» стороны нет, и веер
  вершины в таком месте не упорядочивается. Дверь fail-closed, а не молчаливый
  выбор одного из соседей.
* ДИАГОНАЛИ. `FACE_DIAGONAL` НИКОГДА не получает `PhysicalEdgeId`. Синтетика в
  нормативном тождестве отвергнута (AUTH P0-4-Q-02): диагональ тесселяции —
  свойство разбиения, а не источника, и id источника ей не принадлежит.

Дайджест и кросс-валидатор живут здесь же: таблица объявляет себя властью
смежности, поэтому и её согласие со снапшотом обязано быть исполняемым в том
же модуле, а не обещанием в документе.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from hashlib import sha256

from ..codec import canonical_json_bytes
from ..ids import (
    OpaqueId,
    PatchId,
    PhysicalEdgeId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from .surface import PatchSurfaceIRV1


SURFACE_ADJACENCY_IR_SCHEMA_V1 = "cftuv.envelope.surface_adjacency_ir.v1"
SURFACE_COMPLETE_IR_SCHEMA_V1 = "cftuv.envelope.surface_complete_ir.v1"


@dataclass(frozen=True, slots=True)
class SurfaceAdjacencyDigestValue(OpaqueId):
    """sha256 канонических байт таблицы смежности."""


class SurfaceAdjacencyScopeV1(str, Enum):
    """Что именно покрывает таблица. Третьего значения нет."""

    FULL_SOURCE_MESH = "FULL_SOURCE_MESH"
    REQUEST_SCOPED_PATCH_SET = "REQUEST_SCOPED_PATCH_SET"


class TriangleSideKindV1(str, Enum):
    """Сторона треугольника лежит на ребре источника либо на диагонали."""

    SOURCE_EDGE = "SOURCE_EDGE"
    FACE_DIAGONAL = "FACE_DIAGONAL"


class SideBoundaryReasonV1(str, Enum):
    """Почему у стороны нет противоположной. Причины не сливаются."""

    MESH_BOUNDARY = "MESH_BOUNDARY"
    OUTSIDE_REQUESTED_SCOPE = "OUTSIDE_REQUESTED_SCOPE"
    NON_MANIFOLD_FAN = "NON_MANIFOLD_FAN"
    HOST_DECLARED_UNKNOWN = "HOST_DECLARED_UNKNOWN"


class SurfaceOrientationLawV1(str, Enum):
    """Комбинаторный закон ориентации. binary64-нормаль властью не является."""

    CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1 = "CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1"


class VertexFanUnavailableReasonV1(str, Enum):
    """Почему веер вершины не упорядочен."""

    NON_MANIFOLD_FAN = "NON_MANIFOLD_FAN"
    OUTSIDE_REQUESTED_SCOPE = "OUTSIDE_REQUESTED_SCOPE"
    HOST_DECLARED_UNKNOWN = "HOST_DECLARED_UNKNOWN"


@dataclass(frozen=True, slots=True)
class TriangleSideRefV1:
    """Адрес стороны: треугольник и порядковый номер стороны.

    `side_ordinal` — ТОТ ЖЕ индекс, что у `SurfaceTriangleV1.physical_edge_ids`:
    сторона `i` соединяет `vertex_ids[i]` и `vertex_ids[(i + 1) % 3]`. Закон
    взят не из комментария, а из исполняемой проверки —
    `validation.py` сверяет `physical_edge_ids[index]` ровно с парами
    `(0,1), (1,2), (2,0)`.

    ВНИМАНИЕ, счёт заведён. У ХОСТА (`cftuv/surface_ir.py`) тот же кортеж
    индексируется ПРОТИВОЛЕЖАЩЕЙ вершиной, и экспортёр молча поворачивает
    тройку (`envelope_request_export.py`: `[2], [0], [1]`). Два закона
    нумерации одной величины живут в одном конвейере, и ни один из них не
    записан в контракте, который его несёт. Здесь фиксируется ЯДЕРНЫЙ: таблица
    смежности адресует стороны так же, как их адресует снапшот.
    """

    triangle_id: SurfaceTriangleId
    side_ordinal: int

    def __post_init__(self) -> None:
        if type(self.side_ordinal) is not int:
            raise ValueError("side_ordinal — целое")
        if not 0 <= self.side_ordinal <= 2:
            raise ValueError("side_ordinal треугольника лежит в {0, 1, 2}")


@dataclass(frozen=True, slots=True)
class SideBoundaryV1:
    """Противоположной стороны нет, и причина названа."""

    reason: SideBoundaryReasonV1


@dataclass(frozen=True, slots=True)
class TriangleSideV1:
    """Одна сторона одного треугольника со своей парой концов.

    Концы записаны НАПРАВЛЕННО, в порядке обхода этого треугольника. Иначе
    закон ориентации нечем проверить: неупорядоченная пара концов одинакова у
    обеих сторон склейки независимо от того, согласованы они или нет.
    """

    side: TriangleSideRefV1
    from_vertex_id: SourceVertexId
    to_vertex_id: SourceVertexId
    kind: TriangleSideKindV1
    physical_edge_id: PhysicalEdgeId | None
    opposite: TriangleSideRefV1 | SideBoundaryV1

    def __post_init__(self) -> None:
        if self.from_vertex_id == self.to_vertex_id:
            raise ValueError("концы стороны треугольника различны")
        if self.kind is TriangleSideKindV1.SOURCE_EDGE:
            if self.physical_edge_id is None:
                raise ValueError("SOURCE_EDGE обязана назвать PhysicalEdgeId")
        elif self.physical_edge_id is not None:
            # Синтетический id диагонали заражает нормативное тождество.
            raise ValueError("FACE_DIAGONAL не имеет PhysicalEdgeId")


@dataclass(frozen=True, slots=True)
class VertexFanV1:
    """Упорядоченный веер треугольников вокруг вершины.

    `is_closed` — веер замкнут (вершина внутренняя), то есть последний
    треугольник склеен с первым. Открытый веер — вершина на границе скоупа или
    меша; порядок в нём идёт от одного края к другому.
    """

    vertex_id: SourceVertexId
    ordered_triangle_ids: tuple[SurfaceTriangleId, ...]
    is_closed: bool

    def __post_init__(self) -> None:
        if not self.ordered_triangle_ids:
            raise ValueError("веер вершины непуст")
        if len(set(self.ordered_triangle_ids)) != len(self.ordered_triangle_ids):
            raise ValueError("треугольник входит в веер вершины один раз")


@dataclass(frozen=True, slots=True)
class VertexFanUnavailableV1:
    """Веер вершины не упорядочивается, и это записано, а не выбрано."""

    vertex_id: SourceVertexId
    reason: VertexFanUnavailableReasonV1


SurfaceVertexFanV1 = VertexFanV1 | VertexFanUnavailableV1


# Закон канонического порядка веера. Порядок обхода веера — ВЫБОР, и два
# независимых вывода одной и той же смежности выбрали разные концы цепи, пока
# закон не был назван (полевая сверка на `building_001`: двенадцать вееров
# совпали как множества и разошлись как кортежи). Поэтому закон один и живёт в
# контракте, а не по копии у каждого производителя: ключ сортировки — ЗНАЧЕНИЕ
# `SurfaceTriangleId`, потому что это единственное, что есть у обоих концов
# конвейера (хостовый номер до ядра не доходит).
VERTEX_FAN_ORDER_LAW_V1 = "SMALLEST_TRIANGLE_ID_VALUE_FIRST_V1"


def canonical_fan_order(links: dict):
    """Канонический порядок веера по графу склеек при вершине.

    `links` — `{SurfaceTriangleId: set[SurfaceTriangleId]}`, соседи считаются
    только по сторонам, содержащим эту вершину. Возвращает `(порядок, замкнут)`
    либо `None`, если это не веер: степень выше двух, разрыв цепи или
    вырожденный «замкнутый» веер короче трёх треугольников.
    """

    if any(len(near) > 2 for near in links.values()):
        return None
    ends = sorted(
        (node for node, near in links.items() if len(near) <= 1),
        key=lambda item: item.value,
    )
    closed = not ends
    start = min(links, key=lambda item: item.value) if closed else ends[0]
    order = [start]
    previous = None
    current = start
    while True:
        visited = set() if previous is None else {previous}
        following = sorted(links[current] - visited, key=lambda item: item.value)
        if not following or following[0] == start:
            break
        order.append(following[0])
        previous, current = current, following[0]
    if len(order) != len(links) or (closed and len(order) < 3):
        return None
    return tuple(order), closed


@dataclass(frozen=True, slots=True)
class SurfaceAdjacencyIRV1:
    """Смежность сторон треугольников и веера вершин одной ревизии."""

    schema_version: str
    source_revision: SourceRevision
    scope: SurfaceAdjacencyScopeV1
    scope_patch_ids: frozenset[PatchId]
    orientation_law: SurfaceOrientationLawV1
    triangle_sides: frozenset[TriangleSideV1]
    vertex_fans: frozenset[SurfaceVertexFanV1]

    def __post_init__(self) -> None:
        if self.scope is SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH:
            if self.scope_patch_ids:
                raise ValueError(
                    "FULL_SOURCE_MESH не перечисляет патчи: срез, названный "
                    "полным мешем, и есть отказ N-4"
                )
        elif not self.scope_patch_ids:
            raise ValueError("REQUEST_SCOPED_PATCH_SET обязан назвать патчи")
        _forbid_scope_boundary_in_full_mesh(self)


def _forbid_scope_boundary_in_full_mesh(table: SurfaceAdjacencyIRV1) -> None:
    """N-4: полный меш не имеет сторон, упирающихся в границу запроса."""

    if table.scope is not SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH:
        return
    for side in table.triangle_sides:
        opposite = side.opposite
        if (
            isinstance(opposite, SideBoundaryV1)
            and opposite.reason is SideBoundaryReasonV1.OUTSIDE_REQUESTED_SCOPE
        ):
            raise ValueError(
                "OUTSIDE_REQUESTED_SCOPE под FULL_SOURCE_MESH: payload срезан, "
                "а объявлен полным"
            )
    for fan in table.vertex_fans:
        if (
            isinstance(fan, VertexFanUnavailableV1)
            and fan.reason is VertexFanUnavailableReasonV1.OUTSIDE_REQUESTED_SCOPE
        ):
            raise ValueError(
                "веер OUTSIDE_REQUESTED_SCOPE под FULL_SOURCE_MESH: payload "
                "срезан, а объявлен полным"
            )


def surface_adjacency_digest(
    table: SurfaceAdjacencyIRV1,
) -> SurfaceAdjacencyDigestValue:
    """Собственный канонический дайджест таблицы.

    Отдельный от `snapshot_digest` намеренно: таблица не входит в байты
    снапшота и не должна их двигать. Смежность обязана быть адресуемой сама по
    себе — `SurfaceMetricDescriptorV2` ссылается на НЕЁ, а не на снапшот.
    """

    return SurfaceAdjacencyDigestValue(
        sha256(canonical_json_bytes(table)).hexdigest()
    )


@dataclass(frozen=True, slots=True)
class SurfaceCompleteIRV1:
    """Неподвижный `PatchSurfaceIRV1` плюс таблица смежности и её дайджест.

    Байты `PatchSurfaceIRV1` не меняются: обёртка ДОБАВЛЯЕТ запись рядом, а не
    переписывает существующую. Дайджест хранится, а не выводится при чтении,
    чтобы ссылка на таблицу из метрики проверялась сравнением, а не доверием.
    """

    schema_version: str
    surface_ir: PatchSurfaceIRV1
    adjacency: SurfaceAdjacencyIRV1
    adjacency_digest: SurfaceAdjacencyDigestValue


class SurfaceAdjacencyContractError(ValueError):
    """Кросс-проверка таблицы смежности и снапшота не сошлась."""

    def __init__(self, invariant: str, details: str) -> None:
        self.invariant = str(invariant)
        self.details = str(details)
        super().__init__(f"SURFACE_ADJACENCY_INVALID:{self.invariant}: {details}")


def _fail(invariant: str, details: str) -> None:
    raise SurfaceAdjacencyContractError(invariant, details)


def _side_endpoints(triangle, ordinal: int):
    """Ядерный закон нумерации: сторона `i` — пара `(v[i], v[(i + 1) % 3])`."""

    return (
        triangle.vertex_ids[ordinal],
        triangle.vertex_ids[(ordinal + 1) % 3],
    )


def _check_identity(table: SurfaceAdjacencyIRV1, surface: PatchSurfaceIRV1) -> None:
    if table.schema_version != SURFACE_ADJACENCY_IR_SCHEMA_V1:
        _fail("SCHEMA_VERSION", table.schema_version)
    if table.source_revision != surface.source_revision:
        _fail(
            "SOURCE_REVISION",
            f"table={table.source_revision.value} "
            f"surface={surface.source_revision.value}",
        )
    if table.orientation_law is not (
        SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
    ):
        _fail("ORIENTATION_LAW", table.orientation_law.value)


def _check_side_membership(sides, triangles, surface) -> None:
    """Множества id таблицы обязаны лежать ВНУТРИ снапшота."""

    edge_ids = {edge.edge_id for edge in surface.source_edges}
    for key, side in sides.items():
        triangle_id = key[0]
        if triangle_id not in triangles:
            _fail("TRIANGLE_SCOPE", f"triangle={triangle_id.value}")
        for vertex_id in (side.from_vertex_id, side.to_vertex_id):
            if vertex_id not in surface.source_vertex_ids:
                _fail("VERTEX_SCOPE", f"vertex={vertex_id.value}")
        if side.physical_edge_id is not None and side.physical_edge_id not in edge_ids:
            _fail("EDGE_SCOPE", f"edge={side.physical_edge_id.value}")


def _check_side_totality(sides) -> None:
    """Названный треугольник несёт ровно три стороны с номерами 0, 1, 2."""

    ordinals_by_triangle: dict[SurfaceTriangleId, set[int]] = {}
    for triangle_id, ordinal in sides:
        ordinals_by_triangle.setdefault(triangle_id, set()).add(ordinal)
    for triangle_id, ordinals in ordinals_by_triangle.items():
        if ordinals != {0, 1, 2}:
            _fail(
                "SIDE_TOTALITY",
                f"triangle={triangle_id.value} ordinals={sorted(ordinals)}",
            )


def _check_side_facts(sides, triangles) -> None:
    """Концы, вид и физическое ребро стороны против самого снапшота."""

    for (triangle_id, ordinal), side in sides.items():
        triangle = triangles[triangle_id]
        expected = _side_endpoints(triangle, ordinal)
        if (side.from_vertex_id, side.to_vertex_id) != expected:
            _fail(
                "SIDE_ENDPOINTS",
                f"triangle={triangle_id.value} ordinal={ordinal}",
            )
        declared = triangle.physical_edge_ids[ordinal]
        if side.physical_edge_id != declared:
            _fail(
                "PHYSICAL_EDGE_AGREEMENT",
                f"triangle={triangle_id.value} ordinal={ordinal} "
                f"table={side.physical_edge_id} surface={declared}",
            )
        expected_kind = (
            TriangleSideKindV1.FACE_DIAGONAL
            if declared is None
            else TriangleSideKindV1.SOURCE_EDGE
        )
        if side.kind is not expected_kind:
            _fail(
                "SIDE_KIND",
                f"triangle={triangle_id.value} ordinal={ordinal} "
                f"kind={side.kind.value}",
            )


def _check_opposite(sides) -> None:
    """Инволюция склейки, общая пара концов и комбинаторная ориентация."""

    for key, side in sides.items():
        opposite = side.opposite
        if isinstance(opposite, SideBoundaryV1):
            continue
        other_key = (opposite.triangle_id, opposite.side_ordinal)
        if other_key == key:
            _fail("OPPOSITE_INVOLUTION", f"side={key[0].value}:{key[1]} сама себе")
        other = sides.get(other_key)
        if other is None:
            _fail(
                "OPPOSITE_INVOLUTION",
                f"side={key[0].value}:{key[1]} ссылается на отсутствующую",
            )
        back = other.opposite
        if not isinstance(back, TriangleSideRefV1) or (
            back.triangle_id,
            back.side_ordinal,
        ) != key:
            _fail(
                "OPPOSITE_INVOLUTION",
                f"side={key[0].value}:{key[1]} склейка не взаимна",
            )
        if {side.from_vertex_id, side.to_vertex_id} != {
            other.from_vertex_id,
            other.to_vertex_id,
        }:
            _fail(
                "OPPOSITE_ENDPOINTS",
                f"side={key[0].value}:{key[1]} склеена с чужой парой концов",
            )
        if side.from_vertex_id != other.to_vertex_id:
            _fail(
                "ORIENTATION_CONSISTENCY",
                f"side={key[0].value}:{key[1]} проходит общую пару в ту же "
                "сторону, что и склеенная",
            )
        if side.physical_edge_id != other.physical_edge_id:
            _fail(
                "OPPOSITE_EDGE_AGREEMENT",
                f"side={key[0].value}:{key[1]} и её склейка называют разные "
                "физические рёбра",
            )


def _fan_neighbours(sides, triangle_id, vertex_id):
    """Стороны треугольника, СОДЕРЖАЩИЕ вершину: их ровно две."""

    return tuple(
        side
        for ordinal in range(3)
        for side in (sides[(triangle_id, ordinal)],)
        if vertex_id in (side.from_vertex_id, side.to_vertex_id)
    )


def _check_vertex_fans(table: SurfaceAdjacencyIRV1, sides, triangles) -> None:
    """Веер — цепь треугольников, склеенных сторонами при этой вершине."""

    seen: set[SourceVertexId] = set()
    for fan in table.vertex_fans:
        if fan.vertex_id in seen:
            _fail("VERTEX_FAN_UNIQUE", f"vertex={fan.vertex_id.value}")
        seen.add(fan.vertex_id)
        if isinstance(fan, VertexFanUnavailableV1):
            continue
        for triangle_id in fan.ordered_triangle_ids:
            triangle = triangles.get(triangle_id)
            if triangle is None or fan.vertex_id not in triangle.vertex_ids:
                _fail(
                    "VERTEX_FAN_INCIDENCE",
                    f"vertex={fan.vertex_id.value} triangle={triangle_id}",
                )
        _check_fan_chain(fan, sides)


def _fan_links(fan: VertexFanV1, sides) -> dict:
    links = {triangle_id: set() for triangle_id in fan.ordered_triangle_ids}
    for triangle_id in fan.ordered_triangle_ids:
        for side in _fan_neighbours(sides, triangle_id, fan.vertex_id):
            opposite = side.opposite
            if isinstance(opposite, TriangleSideRefV1):
                links[triangle_id].add(opposite.triangle_id)
    return {
        node: {near for near in neighbours if near in links}
        for node, neighbours in links.items()
    }


def _check_fan_chain(fan: VertexFanV1, sides) -> None:
    """Порядок веера обязан быть ТЕМ САМЫМ каноническим, а не любым связным."""

    canonical = canonical_fan_order(_fan_links(fan, sides))
    if canonical is None:
        _fail(
            "VERTEX_FAN_ORDER",
            f"vertex={fan.vertex_id.value}: склейки при вершине веером не "
            "являются",
        )
    order, closed = canonical
    if (order, closed) != (fan.ordered_triangle_ids, fan.is_closed):
        _fail(
            "VERTEX_FAN_ORDER",
            f"vertex={fan.vertex_id.value}: порядок не канонический "
            f"({VERTEX_FAN_ORDER_LAW_V1})",
        )


def validate_surface_adjacency(
    table: SurfaceAdjacencyIRV1,
    surface: PatchSurfaceIRV1,
) -> None:
    """Кросс-проверка таблицы смежности против снапшота поверхности.

    Проверяется ровно то, что таблица утверждает сверх снапшота, и ровно то, в
    чём она обязана со снапшотом совпасть: ревизия, вхождение множеств id,
    согласие `physical_edge_id` со стороной, взаимность склейки, общая пара
    концов, комбинаторная ориентация и связность вееров. Отказ — именованный.
    """

    _check_identity(table, surface)
    triangles = {item.triangle_id: item for item in surface.surface_triangles}
    sides = {
        (side.side.triangle_id, side.side.side_ordinal): side
        for side in table.triangle_sides
    }
    if len(sides) != len(table.triangle_sides):
        _fail("SIDE_UNIQUE", "две записи на один адрес стороны")
    _check_side_membership(sides, triangles, surface)
    _check_side_totality(sides)
    _check_side_facts(sides, triangles)
    _check_opposite(sides)
    _check_vertex_fans(table, sides, triangles)


def validate_surface_complete(complete: SurfaceCompleteIRV1) -> None:
    """То же плюс сверка хранимого дайджеста с пересчитанным."""

    if complete.schema_version != SURFACE_COMPLETE_IR_SCHEMA_V1:
        _fail("SCHEMA_VERSION", complete.schema_version)
    validate_surface_adjacency(complete.adjacency, complete.surface_ir)
    recomputed = surface_adjacency_digest(complete.adjacency)
    if recomputed != complete.adjacency_digest:
        _fail(
            "ADJACENCY_DIGEST",
            f"stored={complete.adjacency_digest.value} "
            f"recomputed={recomputed.value}",
        )


__all__ = (
    "SURFACE_ADJACENCY_IR_SCHEMA_V1",
    "SURFACE_COMPLETE_IR_SCHEMA_V1",
    "VERTEX_FAN_ORDER_LAW_V1",
    "canonical_fan_order",
    "SideBoundaryReasonV1",
    "SideBoundaryV1",
    "SurfaceAdjacencyContractError",
    "SurfaceAdjacencyDigestValue",
    "SurfaceAdjacencyIRV1",
    "SurfaceAdjacencyScopeV1",
    "SurfaceCompleteIRV1",
    "SurfaceOrientationLawV1",
    "SurfaceVertexFanV1",
    "TriangleSideKindV1",
    "TriangleSideRefV1",
    "TriangleSideV1",
    "VertexFanUnavailableReasonV1",
    "VertexFanUnavailableV1",
    "VertexFanV1",
    "surface_adjacency_digest",
    "validate_surface_adjacency",
    "validate_surface_complete",
)
