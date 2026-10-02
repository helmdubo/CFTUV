"""Закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1`: внутренняя вершина прямой цепи стоит на хорде там, где её источник.

ЗАЧЕМ. Хорда объявленной прямой цепи лежит на узлах решётки домена, а внутренние вершины цепи привязка
(`evaluation_geometry`) садит на узлы той же хорды в строгом порядке. Шаг хорды — приращение примитивного
вектора, и на крупной хорде (`gcd = 1`) соседние узлы стоят далеко друг от друга: вершина источника
оказывается на узле в метрах от своей позиции вдоль хорды (поле `building`: до 1.13 м). Ключ `src:<id>`
в соседнем домене стоит в другом месте, сварка по позициям не сходится, и бюджет закона положения
(`source_lift`: одна ячейка источника) вершину не берёт — `SOURCE_VERTEX_DISPLACED_BY_LATTICE`.

ЗАКОН. Внутренняя вершина прямой цепи — точка на отрезке: её положение вдоль хорды не меняет ни
арранжемент, ни фронты, ни регионы покрытия, а меняет только то, где стоит перекладина, станция `s` и
позиция `src:`. Поэтому ядро держит хорду на узлах (покрытие не тронуто ни на бит), а материализатор ставит
вершину в ТОЧНУЮ рациональную точку хорды `anchor * 2^r + t * d`, где `t = projection_k_gram` из привязки
(проекция исходной вершины по Граму, точная дробь, знаменатели в десятки бит), `d` — примитивный вектор
хорды. Вершина сохраняет имя `src:<id>` (карта имён `names` — второй источник имени для точки, которая
уже не узел решётки), `(s, r)` и UV считаются от новой точки (`s` — точная проекция Грама на пробег, `r`
ноль), подъём (`lift`) и закон положения хоста (`source_lift`) берут её как любую точку контура.

ЧТО ЗАКОН ДЕЛАЕТ С ГРАНЯМИ. Точка встаёт во ВСЕ контуры, где стоял её узел: в грани, в слитые грани и в их
части (перекладины слитого пробега сохраняются, части режутся теми же ключами). Удвоенная площадь
сдвинутой грани пересчитывается точно (`doubled_shoelace`), и сумма сдвигов по региону обязана быть
НУЛЕМ точно, иначе отказ `COVERAGE_FACE_LOST`: сдвиг вдоль хорды отдаёт площадь соседу, а не теряет её.

ЧТО ЗАКОН НЕ ДЕЛАЕТ МОЛЧА (счётчики `MATERIALIZE_CHORD_STATIONS_*`, их сумма равна `TOTAL`).
* `PLACED` — вершина сдвинута с узла на станцию; `AT_NODE` — станция целая, сдвига нет;
  `NOT_IN_COVERAGE` — узел вершины ни в одной грани домена, ставить нечего.
* `SKIPPED_NOT_MONOTONE` — станции цепи не строго возрастают между концами хорды (точная ничья на
  полушаге): цепь остаётся на узлах, диагностика называет её; `SKIPPED_NODE_NAMES_ANOTHER_VERTEX` — узел
  вершины назван в привязке или в таблице станций другой вершиной: ставить под чужим именем нельзя.
* Допусков закон не вводит: все сравнения точные, реестр допусков не меняется.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from fractions import Fraction
from typing import NamedTuple

from ..contracts.plan import ChainStraightEvaluationGeometryBindingV2
from ..exact_sqrt_sum import SqrtSumV1
from ..wavefront.faces import doubled_shoelace
from .admit import MaterializationOutcome
from .coalesce import lattice_node, point_key
from .frames import MaterializationRefusal

SOURCE_VERTEX_CHORD_STATION_LAW = "SOURCE_VERTEX_STATIONED_ON_CHORD_V1"

TOTAL = "MATERIALIZE_CHORD_STATIONS_TOTAL"
PLACED = "MATERIALIZE_CHORD_STATIONS_PLACED"
AT_NODE = "MATERIALIZE_CHORD_STATIONS_AT_NODE"
NOT_IN_COVERAGE = "MATERIALIZE_CHORD_STATIONS_NOT_IN_COVERAGE"
SKIPPED_NOT_MONOTONE = "MATERIALIZE_CHORD_STATIONS_SKIPPED_NOT_MONOTONE"
SKIPPED_NODE_SHARED = "MATERIALIZE_CHORD_STATIONS_SKIPPED_NODE_NAMES_ANOTHER_VERTEX"
FACES_RESTATIONED = "MATERIALIZE_CHORD_STATIONS_FACES_RESTATIONED"

REASON_NOT_MONOTONE = "NOT_MONOTONE"
REASON_NODE_SHARED = "NODE_NAMES_ANOTHER_VERTEX"


@dataclass(frozen=True, slots=True)
class ChordStationsV1:
    """Итог закона: имена вершин на станциях и всё, что не сдвинуто, с числами."""

    #: `{(регион, point_key станции): id вершины источника}` — имя для точки, не лежащей на узле.
    names: dict = field(default_factory=dict)
    total: int = 0
    placed: int = 0
    at_node: int = 0
    not_in_coverage: int = 0
    #: Грани (слитые), у которых сдвинулась хоть одна точка контура либо контура части.
    faces: int = 0
    #: `((id цепи, причина, вершин), ...)` — цепи, оставленные на узлах.
    skipped: tuple = ()
    #: Наибольший сдвиг среди поставленных, в шагах хорды (`Fraction`, не больше 1/2).
    largest_slide: Fraction = Fraction(0)

    def _skipped_vertices(self, reason: str) -> int:
        return sum(count for _chain, name, count in self.skipped if name == reason)

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            (TOTAL, self.total),
            (PLACED, self.placed),
            (AT_NODE, self.at_node),
            (NOT_IN_COVERAGE, self.not_in_coverage),
            (SKIPPED_NOT_MONOTONE, self._skipped_vertices(REASON_NOT_MONOTONE)),
            (SKIPPED_NODE_SHARED, self._skipped_vertices(REASON_NODE_SHARED)),
            (FACES_RESTATIONED, self.faces),
        )

    def placed_note(self) -> str:
        return (
            f"{self.placed} of {self.total} chain-internal source vertices stationed on their "
            f"chord at the exact Gram projection (law {SOURCE_VERTEX_CHORD_STATION_LAW}); "
            f"{self.faces} faces restationed; largest slide {float(self.largest_slide):.6g} "
            "chord steps"
        )

    def skipped_note(self) -> str:
        shown = ", ".join(
            f"{chain}:{reason}({count})" for chain, reason, count in self.skipped[:6]
        )
        more = len(self.skipped) - 6
        return (
            f"{len(self.skipped)} straight chains keep their internal vertices on lattice "
            f"nodes: {shown}" + (f" (+{more} more)" if more > 0 else "")
        )


class _Target(NamedTuple):
    """Станция вершины: имя, точка хорды (`SqrtSumV1` пара) и сдвиг от узла в шагах хорды."""

    vertex_id: str
    point: tuple
    slide: Fraction


def _station(item) -> Fraction:
    """`t` вершины вдоль хорды в шагах примитивного вектора: точная дробь привязки."""

    value = item.projection_k_gram
    return Fraction(value.numerator, value.denominator)


def _skip_reason(chain, stations, claims) -> str | None:
    """Причина оставить цепь на узлах либо `None`: порядок станций строгий, узлы названы одной вершиной."""

    sequence = (Fraction(0), *stations, Fraction(chain.refined_endpoint_span_k))
    if any(after <= before for before, after in zip(sequence, sequence[1:])):
        return REASON_NOT_MONOTONE
    if any(
        len(claims[tuple(item.assigned_refined_node)]) != 1
        for item in chain.internal_assignments
    ):
        return REASON_NODE_SHARED
    return None


def _claims(chains, table) -> dict:
    """`{узел: {id вершин, которые его называют}}`: привязка и таблица станций (все регионы)."""

    claims: dict = {}
    for chain in chains:
        for item in chain.internal_assignments:
            claims.setdefault(tuple(item.assigned_refined_node), set()).add(
                item.source_vertex_id.value
            )
    for (_region, node), vertex_id in table.node_vertex_ids.items():
        if vertex_id is not None and node in claims:
            claims[node].add(vertex_id)
    return claims


def _plan(binding, table):
    """`(цели {узел: _Target}, всего, на узле, пропущенные цепи)` по привязке цепей."""

    factor = 1 << binding.refinement_power
    chains = sorted(binding.straight_chain_bindings, key=lambda item: item.physical_chain_id.value)
    claims = _claims(chains, table)
    targets: dict = {}
    skipped: list = []
    total = at_node = 0
    for chain in chains:
        assignments = sorted(chain.internal_assignments, key=lambda item: item.ordinal)
        stations = [_station(item) for item in assignments]
        total += len(assignments)
        reason = _skip_reason(chain, stations, claims)
        if reason is not None:
            skipped.append((chain.physical_chain_id.value, reason, len(assignments)))
            continue
        anchor = tuple(axis * factor for axis in chain.base_start_node)
        direction = chain.primitive_direction
        for item, station in zip(assignments, stations):
            if station == item.selected_k:
                at_node += 1
                continue
            point = tuple(
                SqrtSumV1.rational(anchor[axis] + station * direction[axis]) for axis in (0, 1)
            )
            targets[tuple(item.assigned_refined_node)] = _Target(
                item.source_vertex_id.value, point, abs(station - item.selected_k)
            )
    return targets, total, at_node, tuple(skipped)


def _slide(points, targets, found: set):
    """`(точки, сдвинуто?)`: точки, стоявшие на узле цели, встают на её станцию."""

    moved = False
    result = []
    for point in points:
        node = lattice_node(point)
        target = None if node is None else targets.get(node)
        if target is None:
            result.append(point)
            continue
        found.add(node)
        result.append(target.point)
        moved = True
    return tuple(result), moved


def _restation(face, targets, found: set):
    """`(грань, сдвинута?)`: контур грани и контуры её частей с точными площадями."""

    points, moved = _slide(face.points, targets, found)
    parts = [_restation(part, targets, found) for part in face.parts]
    if not moved and not any(part_moved for _part, part_moved in parts):
        return face, False
    area = face.doubled_area
    if moved:
        area = area + doubled_shoelace(points) - doubled_shoelace(face.points)
    return (
        replace(
            face,
            points=points,
            doubled_area=area,
            parts=tuple(part for part, _moved in parts),
        ),
        True,
    )


def _names(targets, found, table) -> dict:
    """`{(регион, point_key станции): id}` для поставленных вершин, которых регион называет сам."""

    names = {}
    for (region_id, node), vertex_id in table.node_vertex_ids.items():
        target = targets.get(node)
        if target is not None and node in found and vertex_id == target.vertex_id:
            names[(region_id, point_key(target.point))] = vertex_id
    return names


def station_chord_vertices(prepared, items, table):
    """`(грани, ChordStationsV1)`: закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1` над слитыми гранями домена.

    `items` — `[(регион, CoveredFaceV1, линия, ключи источника)]` стадии `_covered_regions`; порядок и
    состав сохраняются. Домен без привязки с прямыми цепями (`EvaluationGeometryBindingV1`) проходит без
    изменений: счётчики нулевые, и это измерение, а не умолчание.
    """

    binding = prepared.compilation.evaluation_geometry_binding
    if not isinstance(binding, ChainStraightEvaluationGeometryBindingV2):
        return items, ChordStationsV1()
    targets, total, at_node, skipped = _plan(binding, table)
    found: set = set()
    drift: dict = {}
    faces = 0
    result = []
    for region_id, face, line, source_keys in items:
        moved_face, moved = _restation(face, targets, found) if targets else (face, False)
        if moved:
            faces += 1
            drift[region_id] = drift.get(region_id, SqrtSumV1.zero()) + (
                moved_face.doubled_area - face.doubled_area
            )
        result.append((region_id, moved_face, line, source_keys))
    stats = ChordStationsV1(
        names=_names(targets, found, table),
        total=total,
        placed=len(found),
        at_node=at_node,
        not_in_coverage=len(targets) - len(found),
        faces=faces,
        skipped=skipped,
        largest_slide=max((targets[node].slide for node in found), default=Fraction(0)),
    )
    open_regions = sorted(name for name, value in drift.items() if not value.is_zero)
    if open_regions:
        raise MaterializationRefusal(
            MaterializationOutcome.COVERAGE_FACE_LOST,
            "; ".join(f"{name}:CHORD_STATION_AREA_DOES_NOT_CLOSE" for name in open_regions[:4]),
            stats.counters(),
        )
    return result, stats
