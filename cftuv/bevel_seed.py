"""Группа бевела от выделенных граней (BEVEL_SELECT_SEED_V1): владелец выделяет одну или несколько граней бевела, кнопка
«Select Bevel Faces» восстанавливает их группу - связную сеть полос и узлов, достижимую от семян, и ничего вне её.

Чистый Python, без Blender: вход - вид всего меша (`BevelMeshViewV1`) и семена, выход - предложение выделения `BevelSelectionV1`
(исход `BEVEL_SELECT_FROM_SEED`, семена в нём). Порядок источников тот же, что у поиска по всему мешу:
1. атрибут Face Strength со STRONG (режим All) - компонента связности граней бевела (значение <= 0) по рёбрам, в которой лежит семя:
   точно, без геометрии;
2. иначе геометрия (атрибут без STRONG - под именем неоднозначности, узлы WEAK, связные с группой, добавляются числом). Параметры -
   из ряда семени, а не из предела ширины кнопки: ряд семени - узкие квады не шире `BEVEL_SEED_FACE_FACTOR` семени; его ширина задаёт
   предел поиска (`BEVEL_SEED_WIDTH_FACTOR`), а отступ бевела ряда (ширина, поворот профиля и число сегментов дают отступ дуги,
   `bevel_offset`) - мерило продолжения. Группа растёт от ряда семени по рёбрам через ряды и узлы, найденные тем же поиском, что и по
   всему мешу. Шаг вдоль полосы (за концевым ребром ряда - та же полоса дальше) берётся всегда: clamp overlap и Percent меняют отступ
   вдоль полосы. Шаг через стык (через узел или рельс) берётся, только если отступ ряда в допуске от ряда, через который дошли, и от
   ряда семени, либо ряд - частичный (не все грани профиля стали кандидатами: хорда и поворот на грань как у мерила,
   `BEVEL_SEED_PARTIAL_ROW`); иначе ряд не взят и дальше не ведёт (`BEVEL_SEED_OFFSET_MISMATCH`, число рядов): так своя фаска
   45 градусов торца исходного меша (отступ 0.029 при бевеле 0.045), до которой бевел обода доходит через узел, остаётся вне группы.
Семя, которое не грань бевела (большая плоская грань, грань вне найденного), - `BEVEL_SEED_NOT_BEVEL` числом: оно остаётся выделенным,
от него ничего не растёт; ни одного годного семени - исход `BEVEL_SEED_NOT_BEVEL`, выделение не тронуто.
"""

from __future__ import annotations

import math
from collections import deque

from . import bevel_faces as bf
from .bevel_faces import (
    BEVEL_SELECT_FACE_STRENGTH,
    BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS,
    BEVEL_SELECT_GEOMETRIC,
    BEVEL_SELECT_NODE_FROM_ATTRIBUTE,
    FACE_STRENGTH_STRONG,
    FACE_STRENGTH_WEAK,
    BevelMeshViewV1,
    BevelSelectionV1,
)

BEVEL_SELECT_FROM_SEED = "BEVEL_SELECT_FROM_SEED"
# Семя не грань бевела (число семян); все семена такие - исход нажатия, выделение не тронуто.
BEVEL_SEED_NOT_BEVEL = "BEVEL_SEED_NOT_BEVEL"
# Ряд за стыком, чей отступ бевела не продолжает группу (другая фаска, своя фаска исходного меша), - не взят, число рядов.
BEVEL_SEED_OFFSET_MISMATCH = "BEVEL_SEED_OFFSET_MISMATCH"
# Ряд не всех сегментов профиля (грань профиля не стала кандидатом), взятый за стыком по хорде и повороту на грань, - число рядов.
BEVEL_SEED_PARTIAL_ROW = "BEVEL_SEED_PARTIAL_ROW"

#: Предел ширины полосы поиска от семени: этот множитель ширины ряда семени (ширина ряда одной фаски на шумном верху гуляет вдвое:
#: хорда зависит от угла ребра, отступ - нет).
BEVEL_SEED_WIDTH_FACTOR = 2.5
#: Ряд семени - из узких квадов не шире этого множителя самого семени (сегменты одной дуги - равные хорды; без предела ряд узкой
#: фаски забирает узкие грани скруглённой стены рядом).
BEVEL_SEED_FACE_FACTOR = 2.0
#: Шаг через стык: отступ ряда не дальше этого множителя от ряда, через который дошли (шумный верх: 0.039-0.049 у одной фаски 0.045;
#: своя фаска торца - 0.029)...
BEVEL_SEED_STEP_FACTOR = 1.3
#: ... и не дальше этого множителя от ряда семени (дрейф по цепочке стыков).
BEVEL_SEED_DRIFT_FACTOR = 1.6
#: Частичный ряд: поворот на грань к повороту полного ряда на сегмент - не больше этого множителя (крайний рельс частичного ряда -
#: полный поворот сегмента: грань из двух сегментов дуги 96 градусов гнёт 72 при 48 на сегмент) и не меньше обратного.
BEVEL_SEED_PARTIAL_TURN_FACTOR = 1.75
#: Семя не в ряду (узел, кусок у торца): ряд-мерило ищется в стольких кольцах соседей по рёбрам.
BEVEL_SEED_NODE_RINGS = 3


def bevel_offset(width: float, turn_degrees: float, segments: int) -> float:
    """Отступ бевела ряда: дуга окружности с поворотом `turn_degrees` в `segments` хорд суммарной ширины `width` - отступ от ребра
    `r * tan(turn / 2)` при `r = width / (2 * segments * sin(turn / (2 * segments)))`. Хорда зависит от угла ребра, отступ (режимы
    Offset и Width с одним углом) - нет: у одной фаски на шумном верху ширина ряда 0.050-0.094, отступ 0.039-0.049."""

    turn = math.radians(max(float(turn_degrees), 1e-3))
    return width * math.tan(turn / 2.0) / (2.0 * segments * math.sin(turn / (2.0 * segments)))


def _ratio(a: float, b: float) -> float:
    return max(a, b) / max(min(a, b), 1e-12)


def _neighbours(table, faces, face_id):
    return {other for key in bf._edges_of(faces[face_id]) for other in table.get(key, ()) if other != face_id}


class _Probe:
    """Узкие квады без предела ширины (тот же `_rail_candidate`); ряд грани - из граней не шире `BEVEL_SEED_FACE_FACTOR` её самой
    (тот же путь колонки и `_accept_row`)."""

    def __init__(self, view: BevelMeshViewV1):
        self.geometry = bf._MeshGeometry(view, math.inf)
        self.candidates = {}
        for face_id in sorted(view.faces):
            found = bf._rail_candidate(self.geometry, face_id)
            if found is not None and not isinstance(found, str):
                self.candidates[face_id] = found

    def row(self, face_id):
        """`(ряд, ширина, поворот)` грани либо `None` (не узкий квад или ряд не принят)."""

        if face_id not in self.candidates:
            return None
        limit = BEVEL_SEED_FACE_FACTOR * self.candidates[face_id][2]
        near = {other: found for other, found in self.candidates.items() if found[2] <= limit}
        bands, rail_faces = bf._bands(near)
        band = next(band for band in bands if face_id in band)
        for row, width, turn in bf._band_rows(self.geometry, near, band, rail_faces, {}):
            if face_id in row:
                return tuple(row), width, turn
        return None

    def rails(self, face_id) -> frozenset:
        """Рельсы грани ряда (узкий квад либо кусок у торца); у прочих - пусто."""

        found = bf._rail_candidate(self.geometry, face_id) or bf._rail_candidate(self.geometry, face_id, bf.BEVEL_SHORT_STRIP_RATIO)
        return frozenset(found[:2]) if found is not None and not isinstance(found, str) else frozenset()


def _reference(table, faces, probe: _Probe, seed):
    """Ряд-мерило семени: его ряд; у узла или куска у торца - самый узкий ряд ближайшего кольца соседей (до `BEVEL_SEED_NODE_RINGS`;
    самый широкий бывает гранями скруглённой стены рядом с фаской); нет - `None`."""

    found = probe.row(seed)
    if found is not None:
        return found
    ring, seen = {seed}, {seed}
    for _step in range(BEVEL_SEED_NODE_RINGS):
        ring = {other for face_id in ring for other in _neighbours(table, faces, face_id)} - seen
        seen |= ring
        rows = [row for row in (probe.row(face_id) for face_id in sorted(ring)) if row is not None]
        if rows:
            return min(rows, key=lambda item: item[1])
    return None


def _measure(row, width, turn):
    """Мерило ряда: `(сегментов, ширина, поворот, отступ)`."""

    return len(row), width, turn, bevel_offset(width, turn, len(row))


def _across(measure, reference, seed, origin) -> str:
    """Продолжает ли ряд `measure` за стыком группу, до которой дошли через ряд `reference` (семя - `seed`, ряд самого семени -
    `origin`): `"full"` - отступ в допуске шага и дрейфа; `"partial"` - частичный ряд (хорда и поворот на грань как у мерила); `""` - нет."""

    segments, _width, _turn, offset = measure
    if _ratio(offset, reference[3]) <= BEVEL_SEED_STEP_FACTOR and _ratio(offset, seed[3]) <= BEVEL_SEED_DRIFT_FACTOR:
        return "full"
    if segments < reference[0]:
        part, whole, least = measure, reference, 1.0 / BEVEL_SEED_PARTIAL_TURN_FACTOR
    elif segments > reference[0] and reference is origin:  # семя - частичный ряд: первый шаг к полному
        # Частичный ряд гнёт на грань не меньше, чем полный на сегмент (его внутренний рельс - полный поворот сегмента): ряд семени
        # с наклонной гранью шумного верха рядом гнёт на сегмент меньше семени - это не полный ряд семени.
        part, whole, least = reference, measure, 1.0
    else:
        return ""
    bend = (part[2] / part[0]) / max(whole[2] / whole[0], 1e-9)
    if _ratio(part[1] / part[0], whole[1] / whole[0]) <= BEVEL_SEED_STEP_FACTOR and least <= bend <= BEVEL_SEED_PARTIAL_TURN_FACTOR:
        return "partial"
    return ""


def _edge_neighbours(table, faces, members):
    """`(грань, ребро, соседка)` по рёбрам граней `members` в устойчивом порядке."""

    for face_id in members:
        for key in bf._edges_of(faces[face_id]):
            for other in table.get(key, ()):
                if other != face_id:
                    yield face_id, key, other


class _Growth:
    """Найденное поиском от семени: ряды (`row_of`, мерила), рельсы граней рядов; итог всех семян и отказы (номера рядов)."""

    def __init__(self, view, table, detection, probe):
        self.faces, self.table, self.detection = view.faces, table, detection
        self.row_of = {face_id: index for index, (row, _width, _turn) in enumerate(detection.rows) for face_id in row}
        self.measures = [_measure(*row) for row in detection.rows]
        self.rails = {face_id: probe.rails(face_id) for face_id in self.row_of}
        self.rows, self.nodes, self.refused, self.partial = set(), set(), set(), set()

    def along(self, face_id, key, other) -> bool:
        """Тот же бевел, а не стык: ребро - концевое (не рельс) у обеих граней рядов (за ним та же полоса дальше) либо рельс у обеих
        (куски одного профиля, который `_accept_row` разрезал: на шумном верху сегмент семени - отдельный ряд). Стык - узел или
        ребро, что у одной грани рельс, а у другой - конец."""

        here, there = self.rails.get(face_id), self.rails.get(other)
        return bool(here) and bool(there) and (key in here) == (key in there)

    def grow(self, seed, seed_measure) -> None:
        """Ряды и узлы группы от семени обходом по рёбрам через найденное (`_across` на стыках); дальше мерилом идёт ряд с большим
        числом сегментов (частичный ряд не мерило), ряд семени-частичного уступает мерило дрейфа первому полному ряду."""

        detection, measures, origin = self.detection, self.measures, seed_measure
        queue = deque()
        if seed in self.row_of:
            self.rows.add(self.row_of[seed])
            queue.append((detection.rows[self.row_of[seed]][0], measures[self.row_of[seed]]))
        else:
            self.nodes.add(seed)
            queue.append(((seed,), seed_measure))
        while queue:
            members, reference = queue.popleft()
            for face_id, key, other in _edge_neighbours(self.table, self.faces, members):
                if other in self.row_of:
                    index = self.row_of[other]
                    if index in self.rows:
                        continue
                    verdict = "full" if self.along(face_id, key, other) else _across(measures[index], reference, seed_measure, origin)
                    if not verdict:
                        self.refused.add(index)
                        continue
                    self.rows.add(index)
                    if verdict == "partial":
                        self.partial.add(index)
                    if measures[index][0] > seed_measure[0]:
                        seed_measure = measures[index]
                    queue.append((detection.rows[index][0], measures[index] if measures[index][0] >= reference[0] else reference))
                elif other in detection.node and other not in self.nodes:
                    self.nodes.add(other)
                    queue.append(((other,), reference))


def _geometric(view: BevelMeshViewV1, seeds: frozenset[int]):
    """`(полосы, узлы, события, годные семена)` геометрии от семян."""

    table = bf._edge_faces(view)
    probe = _Probe(view)
    references = {seed: _reference(table, view.faces, probe, seed) for seed in sorted(seeds)}
    usable = {seed: found for seed, found in references.items() if found is not None}
    if not usable:
        return frozenset(), frozenset(), {}, frozenset()
    detection = bf.detect_bevel_rows(view, BEVEL_SEED_WIDTH_FACTOR * max(width for _row, width, _turn in usable.values()))
    growth = _Growth(view, table, detection, probe)
    grown = set()
    for seed, reference in usable.items():
        if seed in growth.row_of or seed in detection.node:
            growth.grow(seed, growth.measures[growth.row_of[seed]] if seed in growth.row_of else _measure(*reference))
            grown.add(seed)
    strip = frozenset(face_id for index in growth.rows for face_id in detection.rows[index][0])
    events = dict(detection.events)
    for name, count in ((BEVEL_SEED_OFFSET_MISMATCH, len(growth.refused - growth.rows)), (BEVEL_SEED_PARTIAL_ROW, len(growth.partial))):
        if count:
            events[name] = count
    return strip, frozenset(growth.nodes), events, frozenset(grown)


def _component(faces, table, members: frozenset[int], seeds) -> frozenset[int]:
    """Компонента связности `members` по рёбрам, в которой лежат семена."""

    found, stack = set(seeds) & members, sorted(set(seeds) & members)
    while stack:
        face_id = stack.pop()
        for other in _neighbours(table, faces, face_id):
            if other in members and other not in found:
                found.add(other)
                stack.append(other)
    return frozenset(found)


def select_from_seeds(view: BevelMeshViewV1, seeds=None) -> BevelSelectionV1:
    """Группа бевела от семян (по умолчанию - выделенные грани вида): атрибут All - точно, иначе геометрия от ряда семени; семя не
    грань бевела - по имени, ни одного годного - исход `BEVEL_SEED_NOT_BEVEL`."""

    seeds = frozenset(int(item) for item in (view.selected if seeds is None else seeds) if int(item) in view.faces)
    strength = view.face_strength
    events: dict[str, int] = {}
    if strength is not None and any(int(value) == FACE_STRENGTH_STRONG for value in strength.values()):
        source = BEVEL_SELECT_FACE_STRENGTH
        bevel = frozenset(face_id for face_id, value in strength.items() if int(value) <= 0)
        group = _component(view.faces, bf._edge_faces(view), bevel, seeds)
        strip = frozenset(face_id for face_id in group if int(strength[face_id]) == 0)
        node, grown = group - strip, seeds & bevel
    else:
        source = BEVEL_SELECT_GEOMETRIC if strength is None else BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS
        strip, node, events, grown = _geometric(view, seeds)
        if strength is not None and grown:
            weak = frozenset(face_id for face_id, value in strength.items() if int(value) == FACE_STRENGTH_WEAK)
            added = (_component(view.faces, bf._edge_faces(view), strip | node | weak, grown) & weak) - node
            if added:
                events[BEVEL_SELECT_NODE_FROM_ATTRIBUTE] = len(added)
            node, strip = node | added, strip - added
    if seeds - grown:
        events[BEVEL_SEED_NOT_BEVEL] = len(seeds - grown)
    events = tuple(sorted(events.items()))
    if not grown:
        return BevelSelectionV1(BEVEL_SEED_NOT_BEVEL, source, events=events, seeds=seeds)
    return BevelSelectionV1(BEVEL_SELECT_FROM_SEED, source, strip, node, events, seeds)


__all__ = (
    "BEVEL_SEED_NOT_BEVEL",
    "BEVEL_SEED_OFFSET_MISMATCH",
    "BEVEL_SEED_PARTIAL_ROW",
    "BEVEL_SELECT_FROM_SEED",
    "bevel_offset",
    "select_from_seeds",
)
