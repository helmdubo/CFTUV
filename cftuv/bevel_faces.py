"""Декаль от выделенных граней бевела (BEVEL_FACES_INWARD_OUTWARD_V1): сторона, разрез «узел | полоса», именованные исходы.

Чистый Python, без Blender: вход - локальный вид меша (циклы вершин выделенных граней и их соседей по рёбрам, координаты, номера рёбер,
значения атрибута Face Strength), выход - план анализа. Адаптер Blender (`bevel_faces_blender`) только читает bmesh и строит пакет анализа.

ПРАВИЛО ВЛАДЕЛЬЦА: обе стороны считаются «как будто другой части нет».
- INWARD: анализ ТОЛЬКО выделенных граней. Граница выделения становится открытой границей меша; полосы и узлы разведены разрезом
  «узел | полоса», который существует только внутри анализа (меш владельца не получает ни шва, ни атрибута). Каждый патч (полоса или узел)
  заливается от всей своей границы внутрь; у разреза «узел | полоса» источники - обе стороны.
- OUTWARD: анализ ТОЛЬКО невыделенных граней; источники - рёбра, которые оставляет выделение. Разбиение не нужно и не считается.

РАЗБИЕНИЕ «узел | полоса» (BEVEL_NODE_STRIP_SPLIT_V1), порядок источников:
1. есть атрибут `__mod_weightednormals_faceweight` (модификатор Bevel с Face Strength, любой режим кроме None): узел - выделенная грань
   со значением != 0, полоса - со значением 0. Геометрия считается тоже; расхождение - событие `BEVEL_CLASS_DISAGREES` с числом граней;
2. атрибута нет - геометрическая эвристика (`BEVEL_SPLIT_GEOMETRIC`, колонки профиля в духе Unbevel: путь по квадам от ребра с
   плоской гранью A до ребра с плоской гранью B, рельсы вдоль A x B), каждый отброс колонки - именованное событие;
3. грани, которые ни один путь не разрешил чисто (не полоса и не узел при полосе), - именованный отказ `BEVEL_SPLIT_UNRESOLVED`:
   они уходят из анализа и названы числом, а не молчат.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from hashlib import sha256
from typing import Mapping

FACE_STRENGTH_ATTRIBUTE = "__mod_weightednormals_faceweight"

INWARD = "INWARD"
OUTWARD = "OUTWARD"
FACE_SIDES = (INWARD, OUTWARD)

#: Закон ревизии анализа по граням: меняется, если меняется то, что входит в отпечаток.
BEVEL_REVISION_LAW = "BEVEL_FACES_SCOPE_V1"

# Отказы всего нажатия (меш не трогается).
BEVEL_FACES_EMPTY_SELECTION = "BEVEL_FACES_EMPTY_SELECTION"
BEVEL_FACES_NOTHING_OUTSIDE = "BEVEL_FACES_NOTHING_OUTSIDE"
BEVEL_FACES_NO_SOURCE = "BEVEL_FACES_NO_SOURCE"
# Отказ граней (они вне анализа, число названо) и отказ нажатия, если не осталось ни одной.
BEVEL_SPLIT_UNRESOLVED = "BEVEL_SPLIT_UNRESOLVED"
# Чем получено разбиение.
BEVEL_SPLIT_FACE_STRENGTH = "BEVEL_SPLIT_FACE_STRENGTH"
BEVEL_SPLIT_GEOMETRIC = "BEVEL_SPLIT_GEOMETRIC"
# Атрибут и геометрия не согласны (атрибут выигрывает).
BEVEL_CLASS_DISAGREES = "BEVEL_CLASS_DISAGREES"
# События геометрической эвристики: почему путь от ребра границы не стал колонкой профиля.
BEVEL_COLUMN_NOT_QUAD = "BEVEL_COLUMN_NOT_QUAD"
BEVEL_COLUMN_OPEN_END = "BEVEL_COLUMN_OPEN_END"
BEVEL_COLUMN_WRAPS = "BEVEL_COLUMN_WRAPS"
BEVEL_COLUMN_SAME_SIDE = "BEVEL_COLUMN_SAME_SIDE"
BEVEL_COLUMN_RAILS_NOT_PARALLEL = "BEVEL_COLUMN_RAILS_NOT_PARALLEL"
BEVEL_COLUMN_OVERLAPS = "BEVEL_COLUMN_OVERLAPS"

#: Допуск рельса колонки к линии A x B (Unbevel RAIL_TOL: перекос митры коротких полос до ~16 градусов).
RAIL_TOLERANCE_DEGREES = 20.0
#: Плоские стороны колонки, чьи нормали ближе этого, не образуют профиль (путь вдоль полосы, а не поперёк).
SAME_SIDE_DEGREES = 5.0


@dataclass(frozen=True, slots=True)
class BevelFacesViewV1:
    """Локальный вид меша вокруг выделения.

    `faces` - циклы вершин выделенных граней и ВСЕХ их соседей по рёбрам (у ребра выделенной грани все грани ребра в виде);
    `edge_ids` - номер ребра меша по паре вершин; `face_strength` - значения атрибута у выделенных граней либо `None` (атрибута нет).
    """

    coords: Mapping[int, tuple[float, float, float]]
    faces: Mapping[int, tuple[int, ...]]
    selected: frozenset[int]
    edge_ids: Mapping[frozenset, int]
    face_strength: Mapping[int, int] | None = None


@dataclass(frozen=True, slots=True)
class BevelSplitV1:
    """Разбиение выделения: полосы, узлы, неразрешённые грани, источник и события (имя, число)."""

    strip: frozenset[int]
    node: frozenset[int]
    unresolved: frozenset[int]
    source: str
    events: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True, slots=True)
class BevelFacesPlanV1:
    """План анализа одного нажатия по граням: сторона, грани анализа, рёбра разреза, события и отказ (пусто - нет)."""

    side: str
    selected: frozenset[int]
    scope: tuple[int, ...]
    cut_edges: frozenset[int] = frozenset()
    outward_sources: frozenset[int] = frozenset()
    split: BevelSplitV1 | None = None
    refusal: str = ""
    detail: str = ""
    events: tuple[tuple[str, int], ...] = field(default=())


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _length(a) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a):
    length = _length(a)
    return None if length <= 0.0 else (a[0] / length, a[1] / length, a[2] / length)


def face_normal(coords, cycle):
    """Нормаль Ньюэлла (устойчива к неплоским граням); `None` у вырожденной."""

    nx = ny = nz = 0.0
    count = len(cycle)
    for index in range(count):
        a = coords[cycle[index]]
        b = coords[cycle[(index + 1) % count]]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return _unit((nx, ny, nz))


def _edges_of(cycle):
    count = len(cycle)
    return [frozenset((cycle[index], cycle[(index + 1) % count])) for index in range(count)]


def _edge_faces(view: BevelFacesViewV1) -> dict[frozenset, list[int]]:
    table: dict[frozenset, list[int]] = {}
    for face_id in sorted(view.faces):
        for key in _edges_of(view.faces[face_id]):
            table.setdefault(key, []).append(face_id)
    return table


def _other(table, key, face_id):
    others = [item for item in table.get(key, ()) if item != face_id]
    return others[0] if len(others) == 1 else None


def _opposite(cycle, key):
    edges = _edges_of(cycle)
    return edges[(edges.index(key) + 2) % 4]


def _walk_column(view, table, start_key, start_face):
    """Путь по квадам выделения от ребра границы до ребра, за которым невыделенная грань: `(грани, конечное ребро, грань B)` либо имя отброса."""

    path: list[int] = []
    key, face_id = start_key, start_face
    while True:
        cycle = view.faces[face_id]
        if face_id in path:
            return BEVEL_COLUMN_WRAPS
        if len(cycle) != 4:
            return BEVEL_COLUMN_NOT_QUAD
        path.append(face_id)
        key = _opposite(cycle, key)
        beyond = _other(table, key, face_id)
        if beyond is None:
            return BEVEL_COLUMN_OPEN_END
        if beyond not in view.selected:
            return tuple(path), key, beyond
        face_id = beyond


def _rail_deviation(view, start_key, end_key, side_a, side_b):
    """Синус большего угла рельса к линии A x B, если хоть один рельс в допуске; иначе имя отброса."""

    na = face_normal(view.coords, view.faces[side_a])
    nb = face_normal(view.coords, view.faces[side_b])
    if na is None or nb is None:
        return BEVEL_COLUMN_SAME_SIDE
    direction = _cross(na, nb)
    if _length(direction) < math.sin(math.radians(SAME_SIDE_DEGREES)):
        return BEVEL_COLUMN_SAME_SIDE
    direction = _unit(direction)
    deviations = []
    for key in (start_key, end_key):
        first, second = sorted(key)
        rail = _unit(_sub(view.coords[second], view.coords[first]))
        if rail is None:
            return BEVEL_COLUMN_RAILS_NOT_PARALLEL
        deviations.append(_length(_cross(rail, direction)))
    if min(deviations) > math.sin(math.radians(RAIL_TOLERANCE_DEGREES)):
        return BEVEL_COLUMN_RAILS_NOT_PARALLEL
    return max(deviations)


def _midpoint(view, key):
    first, second = sorted(key)
    a, b = view.coords[first], view.coords[second]
    return tuple(round((a[index] + b[index]) * 0.5, 9) for index in range(3))


def geometric_split(view: BevelFacesViewV1) -> BevelSplitV1:
    """Колонки профиля и узлы по геометрии (эвристика `BEVEL_SPLIT_GEOMETRIC`): каждое решение названо событием."""

    table = _edge_faces(view)
    events: dict[str, int] = {}
    candidates = []
    for face_id in sorted(view.selected):
        for key in _edges_of(view.faces[face_id]):
            side_a = _other(table, key, face_id)
            if side_a is None or side_a in view.selected:
                continue
            walked = _walk_column(view, table, key, face_id)
            if isinstance(walked, str):
                events[walked] = events.get(walked, 0) + 1
                continue
            faces, end_key, side_b = walked
            deviation = _rail_deviation(view, key, end_key, side_a, side_b)
            if isinstance(deviation, str):
                events[deviation] = events.get(deviation, 0) + 1
                continue
            candidates.append((deviation, _midpoint(view, key), faces))
    candidates.sort(key=lambda item: item[:2])
    strip: set[int] = set()
    accepted: set[tuple[int, ...]] = set()
    for _deviation, _mid, faces in candidates:
        if frozenset(faces) & strip:
            if tuple(sorted(faces)) not in accepted:  # та же колонка с другого конца - не событие
                events[BEVEL_COLUMN_OVERLAPS] = events.get(BEVEL_COLUMN_OVERLAPS, 0) + 1
            continue
        strip.update(faces)
        accepted.add(tuple(sorted(faces)))
    node, unresolved = _nodes_and_unresolved(view, table, frozenset(strip))
    return BevelSplitV1(frozenset(strip), node, unresolved, BEVEL_SPLIT_GEOMETRIC, tuple(sorted(events.items())))


def _nodes_and_unresolved(view, table, strip: frozenset[int]):
    """Остаток выделения по компонентам: компонента, касающаяся полосы ребром, - узел; иная не разрешена."""

    rest = set(view.selected) - strip
    node: set[int] = set()
    unresolved: set[int] = set()
    seen: set[int] = set()
    for start in sorted(rest):
        if start in seen:
            continue
        component, stack, touches = [], [start], False
        seen.add(start)
        while stack:
            face_id = stack.pop()
            component.append(face_id)
            for key in _edges_of(view.faces[face_id]):
                for other in table.get(key, ()):
                    if other in strip:
                        touches = True
                    elif other in rest and other not in seen:
                        seen.add(other)
                        stack.append(other)
        (node if touches else unresolved).update(component)
    return frozenset(node), frozenset(unresolved)


def split_bevel_faces(view: BevelFacesViewV1) -> BevelSplitV1:
    """Разбиение по порядку владельца: атрибут Face Strength, иначе геометрия; расхождение и отказ - по имени."""

    geometric = geometric_split(view)
    if view.face_strength is None:
        return geometric
    node = frozenset(face_id for face_id in view.selected if int(view.face_strength.get(face_id, 0)) != 0)
    strip = frozenset(view.selected) - node
    disagree = sum(
        1
        for face_id in view.selected
        if (face_id in geometric.node and face_id not in node)
        or (face_id in geometric.strip and face_id not in strip)
        or face_id in geometric.unresolved
    )
    events = (*geometric.events, (BEVEL_CLASS_DISAGREES, disagree)) if disagree else geometric.events
    return BevelSplitV1(strip, node, frozenset(), BEVEL_SPLIT_FACE_STRENGTH, tuple(sorted(events)))


def cut_edges(view: BevelFacesViewV1, split: BevelSplitV1) -> frozenset[int]:
    """Рёбра разреза «узел | полоса» (только анализ): у ребра одна грань полосы и одна грань узла."""

    table = _edge_faces(view)
    cuts = set()
    for key, faces in table.items():
        if len(faces) != 2:
            continue
        first, second = faces
        if (first in split.strip and second in split.node) or (first in split.node and second in split.strip):
            cuts.add(int(view.edge_ids[key]))
    return frozenset(cuts)


def outward_sources(view: BevelFacesViewV1) -> frozenset[int]:
    """Рёбра, которые выделение оставляет невыделенным граням: у ребра выделенная грань и невыделенная."""

    table = _edge_faces(view)
    return frozenset(
        int(view.edge_ids[key])
        for key, faces in table.items()
        if any(item in view.selected for item in faces) and any(item not in view.selected for item in faces)
    )


def plan_faces(view: BevelFacesViewV1, side: str, all_faces) -> BevelFacesPlanV1:
    """План нажатия по граням: грани анализа, разрез, источники OUTWARD и именованный отказ, если считать нечего."""

    if side not in FACE_SIDES:
        raise ValueError(f"unknown face side {side!r}")
    selected = frozenset(int(item) for item in view.selected)
    if not selected:
        return BevelFacesPlanV1(side, selected, (), refusal=BEVEL_FACES_EMPTY_SELECTION, detail="select the bevel faces")
    if side == OUTWARD:
        scope = tuple(sorted(int(item) for item in all_faces if int(item) not in selected))
        if not scope:
            return BevelFacesPlanV1(side, selected, (), refusal=BEVEL_FACES_NOTHING_OUTSIDE, detail="every face is selected")
        sources = outward_sources(view)
        if not sources:
            return BevelFacesPlanV1(side, selected, scope, refusal=BEVEL_FACES_NO_SOURCE, detail="the selection leaves no edge to the other faces")
        return BevelFacesPlanV1(side, selected, scope, outward_sources=sources)
    split = split_bevel_faces(view)
    events = ((split.source, len(selected)), *split.events)
    if split.unresolved:
        events = (*events, (BEVEL_SPLIT_UNRESOLVED, len(split.unresolved)))
    scope = tuple(sorted(selected - split.unresolved))
    if not scope:
        return BevelFacesPlanV1(
            side, selected, (), split=split, events=events, refusal=BEVEL_SPLIT_UNRESOLVED,
            detail=f"no selected face splits into strip and node faces ({len(selected)} faces); add Face Strength to the Bevel modifier",
        )
    return BevelFacesPlanV1(side, selected, scope, cut_edges=cut_edges(view, split), split=split, events=events)


def scope_digest(base_digest: str, plan: BevelFacesPlanV1) -> str:
    """Отпечаток входа анализа по граням: отпечаток меша, закон, сторона, грани анализа и рёбра разреза (шов в отпечатке - разрез)."""

    digest = sha256()
    digest.update(f"{base_digest}|{BEVEL_REVISION_LAW}|{plan.side}|".encode("ascii"))
    digest.update(("scope:" + ",".join(map(str, plan.scope)) + ";").encode("ascii"))
    digest.update(("cuts:" + ",".join(map(str, sorted(plan.cut_edges))) + ";").encode("ascii"))
    return digest.hexdigest()


def plan_summary(plan: BevelFacesPlanV1) -> str:
    """Короткая строка панели: сторона, грани, полосы и узлы, неразрешённые."""

    head = f"faces {plan.side.lower()}: {len(plan.selected)} selected"
    if plan.split is None:
        return head
    split = plan.split
    text = f"{head}, {len(split.strip)} strip / {len(split.node)} node ({split.source})"
    if split.unresolved:
        text += f", {BEVEL_SPLIT_UNRESOLVED} {len(split.unresolved)}"
    return text


def plan_console_lines(plan: BevelFacesPlanV1) -> list[str]:
    """Каждое событие плана - строкой консоли (имя и число): ни одно решение эвристики не молчит."""

    lines = [f"[CFTUV][BevelFaces] {plan_summary(plan)}"]
    lines.extend(f"[CFTUV][BevelFaces] {name} {count}" for name, count in plan.events)
    if plan.refusal:
        lines.append(f"[CFTUV][BevelFaces] REFUSED {plan.refusal}: {plan.detail}")
    return lines


__all__ = (
    "BEVEL_CLASS_DISAGREES",
    "BEVEL_FACES_EMPTY_SELECTION",
    "BEVEL_FACES_NOTHING_OUTSIDE",
    "BEVEL_FACES_NO_SOURCE",
    "BEVEL_SPLIT_FACE_STRENGTH",
    "BEVEL_SPLIT_GEOMETRIC",
    "BEVEL_SPLIT_UNRESOLVED",
    "BevelFacesPlanV1",
    "BevelFacesViewV1",
    "BevelSplitV1",
    "FACE_SIDES",
    "FACE_STRENGTH_ATTRIBUTE",
    "INWARD",
    "OUTWARD",
    "cut_edges",
    "geometric_split",
    "outward_sources",
    "plan_console_lines",
    "plan_faces",
    "plan_summary",
    "scope_digest",
    "split_bevel_faces",
)
