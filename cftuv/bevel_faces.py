"""Декаль от выделенных граней бевела (BEVEL_FACES_INWARD_OUTWARD_V1): сторона, разрез «узел | полоса», именованные исходы.

Чистый Python, без Blender: вход - локальный вид меша (циклы вершин выделенных граней и их соседей по рёбрам, координаты, номера рёбер,
значения атрибута Face Strength), выход - план анализа. Адаптер Blender (`bevel_faces_blender`) только читает bmesh и строит пакет анализа.

ПРАВИЛО ВЛАДЕЛЬЦА: обе стороны считаются «как будто другой части нет».
- INWARD: анализ ТОЛЬКО выделенных граней. Граница выделения становится открытой границей меша; полосы и узлы разведены разрезом
  «узел | полоса», который существует только внутри анализа (меш владельца не получает ни шва, ни атрибута). Каждый патч (полоса или узел)
  заливается от всей своей границы внутрь; у разреза «узел | полоса» источники - обе стороны.
- OUTWARD: анализ ТОЛЬКО невыделенных граней; источники - рёбра, которые оставляет выделение. Разбиение не нужно и не считается.
- BOTH (по умолчанию, BEVEL_FACES_BOTH_V2, заменяет V1 с заливкой граней бевела ядром): анализ и источники - те же, что у OUTWARD;
  выделенные грани не заливаются, а КОПИРУЮТСЯ над собой (`bevel_copy`: те же вершины и грани, смещение декали), и копия сваривается
  с внешней декалью по вершинам рельса. Разбиение «узел | полоса» считается, но в анализ не идёт: по нему строятся только UV копии
  (крылья, `bevel_wings`); неразрешённые грани копируются тоже (их UV продолжает соседнюю полосу), их число - событие.

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

BOTH = "BOTH"
INWARD = "INWARD"
OUTWARD = "OUTWARD"
FACE_SIDES = (BOTH, INWARD, OUTWARD)

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
#: Колонка принята только по плоскости плоской стороны У РЕЛЬСА (нормали её углов при концах рельса), а нормаль всей грани её
#: отбросила бы (BEVEL_SPLIT_RAIL_LOCAL_PLANE_V1): шумная неплоская грань верха с углами вразброс на 40 градусов уводит нормаль Ньюэлла
#: от ребра бевела, и кусок полосы у узла уходил в узел.
BEVEL_COLUMN_RAIL_LOCAL_PLANE = "BEVEL_COLUMN_RAIL_LOCAL_PLANE"
#: Seam Bevel без атрибута (BEVEL_SEAM_NODE_AGREEMENT_V1): несогласия обхода колонок и классификатора выделения, ставшие полосой, и
#: грани выделения вне группы классификатора (разбиение обхода как есть).
BEVEL_SEAM_WALK_NODE_IS_STRIP = "BEVEL_SEAM_WALK_NODE_IS_STRIP"
BEVEL_SEAM_CLASSIFIER_NODE_IS_COLUMN = "BEVEL_SEAM_CLASSIFIER_NODE_IS_COLUMN"
BEVEL_SEAM_UNRESOLVED_IS_STRIP = "BEVEL_SEAM_UNRESOLVED_IS_STRIP"
BEVEL_SEAM_OUTSIDE_CLASSIFIER = "BEVEL_SEAM_OUTSIDE_CLASSIFIER"

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


def _newell(coords, cycle):
    """Вектор Ньюэлла: направление нормали, длина - удвоенная площадь."""

    nx = ny = nz = 0.0
    count = len(cycle)
    for index in range(count):
        a = coords[cycle[index]]
        b = coords[cycle[(index + 1) % count]]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return nx, ny, nz


def face_normal(coords, cycle):
    """Нормаль Ньюэлла (устойчива к неплоским граням); `None` у вырожденной."""

    return _unit(_newell(coords, cycle))


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


def _side_normal_at_rail(view, face_id, key):
    """Нормаль плоской стороны у её рельса: сумма нормалей двух углов грани при концах рельса, повёрнутых по нормали всей грани
    (угол при вогнутой вершине переворачивается); у треугольника - сама грань; вырожденные углы - нормаль всей грани."""

    cycle = view.faces[face_id]
    whole = face_normal(view.coords, cycle)
    count = len(cycle)
    position = next((index for index in range(count) if frozenset((cycle[index], cycle[(index + 1) % count])) == key), None)
    if whole is None or position is None:
        return whole
    total = (0.0, 0.0, 0.0)
    for start in (position - 1, position):
        corner = face_normal(view.coords, tuple(cycle[(start + offset) % count] for offset in range(3)))
        if corner is not None:
            sign = 1.0 if _dot(corner, whole) >= 0.0 else -1.0
            total = tuple(total[axis] + sign * corner[axis] for axis in range(3))
    return _unit(total) or whole


def _rail_deviation(view, start_key, end_key, side_a, side_b, at_rail=False):
    """Синус большего угла рельса к линии A x B, если хоть один рельс в допуске; иначе имя отброса. `at_rail` - нормали сторон у их
    рельсов (`_side_normal_at_rail`), иначе нормали всех граней."""

    if at_rail:
        na, nb = _side_normal_at_rail(view, side_a, start_key), _side_normal_at_rail(view, side_b, end_key)
    else:
        na, nb = face_normal(view.coords, view.faces[side_a]), face_normal(view.coords, view.faces[side_b])
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
            at_rail = False
            if isinstance(deviation, str) and deviation == BEVEL_COLUMN_RAILS_NOT_PARALLEL:
                local = _rail_deviation(view, key, end_key, side_a, side_b, at_rail=True)
                if not isinstance(local, str):  # BEVEL_SPLIT_RAIL_LOCAL_PLANE_V1: принятая прежде колонка не меняется ни в чём
                    deviation, at_rail = local, True
            if isinstance(deviation, str):
                events[deviation] = events.get(deviation, 0) + 1
                continue
            candidates.append((deviation, _midpoint(view, key), faces, at_rail))
    candidates.sort(key=lambda item: item[:2])
    strip: set[int] = set()
    accepted: set[tuple[int, ...]] = set()
    for _deviation, _mid, faces, at_rail in candidates:
        if frozenset(faces) & strip:
            if tuple(sorted(faces)) not in accepted:  # та же колонка с другого конца - не событие
                events[BEVEL_COLUMN_OVERLAPS] = events.get(BEVEL_COLUMN_OVERLAPS, 0) + 1
            continue
        strip.update(faces)
        accepted.add(tuple(sorted(faces)))
        if at_rail:
            events[BEVEL_COLUMN_RAIL_LOCAL_PLANE] = events.get(BEVEL_COLUMN_RAIL_LOCAL_PLANE, 0) + 1
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


def reconcile_with_selection(split: BevelSplitV1, classified: "BevelSelectionV1 | None") -> BevelSplitV1:
    """BEVEL_SEAM_NODE_AGREEMENT_V1 (только геометрия, без атрибута): узел - грань, которую узлом зовут ОБА - обход колонок
    (`geometric_split`) и классификатор кнопки «Select Bevel» / «Grow Bevel» (`classified`, его полосы и узлы); их несогласие -
    полоса, число по имени: `BEVEL_SEAM_WALK_NODE_IS_STRIP` (обход - узел, классификатор - полоса), `BEVEL_SEAM_CLASSIFIER_NODE_IS_COLUMN`
    (классификатор - узел, обход нашёл колонку профиля), `BEVEL_SEAM_UNRESOLVED_IS_STRIP` (обход не разрешил, классификатор - полоса
    или узел). Грани выделения вне группы классификатора остаются за обходом - `BEVEL_SEAM_OUTSIDE_CLASSIFIER` числом."""

    if classified is None or split.source != BEVEL_SPLIT_GEOMETRIC:
        return split
    events = dict(split.events)
    known = (classified.strip | classified.node) & (split.strip | split.node | split.unresolved)
    node = set(split.node - known)
    for face_id in known:
        walk_node, class_node = face_id in split.node, face_id in classified.node
        if walk_node and class_node:
            node.add(face_id)
        elif face_id in split.unresolved:
            _count(events, BEVEL_SEAM_UNRESOLVED_IS_STRIP)
        elif walk_node:
            _count(events, BEVEL_SEAM_WALK_NODE_IS_STRIP)
        elif class_node:
            _count(events, BEVEL_SEAM_CLASSIFIER_NODE_IS_COLUMN)
    outside = (split.strip | split.node | split.unresolved) - known
    if outside:
        events[BEVEL_SEAM_OUTSIDE_CLASSIFIER] = len(outside)
    strip = (split.strip | known) - node
    return BevelSplitV1(frozenset(strip), frozenset(node), split.unresolved - known, split.source, tuple(sorted(events.items())))


def seam_edges(view: BevelFacesViewV1, classified: "BevelSelectionV1 | None" = None) -> tuple[frozenset[int], BevelSplitV1]:
    """BEVEL_SEAM_MARK_V1: швы разметки по выделению - граница выделения (рельсы и края узлов к исходным граням) и разрез «узел |
    полоса»; узел становится своим патчем, швы рельсов сходятся с его краями в точках стыка. Разбиение - атрибут Face Strength, иначе
    геометрия по имени, согласованная с классификатором выделения `classified` (`reconcile_with_selection`)."""

    split = reconcile_with_selection(split_bevel_faces(view), classified)
    return outward_sources(view) | cut_edges(view, split), split


def plan_faces(view: BevelFacesViewV1, side: str, all_faces) -> BevelFacesPlanV1:
    """План нажатия по граням: грани анализа, разрез, источники OUTWARD и именованный отказ, если считать нечего."""

    if side not in FACE_SIDES:
        raise ValueError(f"unknown face side {side!r}")
    selected = frozenset(int(item) for item in view.selected)
    if not selected:
        return BevelFacesPlanV1(side, selected, (), refusal=BEVEL_FACES_EMPTY_SELECTION, detail="select the bevel faces")
    if side == BOTH:
        return _plan_both(view, selected, all_faces)
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


def _plan_both(view: BevelFacesViewV1, selected: frozenset[int], all_faces) -> BevelFacesPlanV1:
    """BOTH: анализ и источники - как у OUTWARD (невыделенные грани, рёбра, которые оставляет выделение); разбиение - только для UV копии.
    Невыделенных граней нет - отказ `BEVEL_FACES_NOTHING_OUTSIDE` (внешней части нет, а копия без неё - не декаль Both)."""

    scope = tuple(sorted(int(item) for item in all_faces if int(item) not in selected))
    if not scope:
        return BevelFacesPlanV1(BOTH, selected, (), refusal=BEVEL_FACES_NOTHING_OUTSIDE, detail="every face is selected: Both grows outward from the selection")
    sources = outward_sources(view)
    if not sources:
        return BevelFacesPlanV1(BOTH, selected, scope, refusal=BEVEL_FACES_NO_SOURCE, detail="the selection leaves no edge to the other faces")
    split = split_bevel_faces(view)
    events = ((split.source, len(selected)), *split.events)
    if split.unresolved:
        events = (*events, (BEVEL_SPLIT_UNRESOLVED, len(split.unresolved)))
    return BevelFacesPlanV1(BOTH, selected, scope, outward_sources=sources, split=split, events=events)


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


# --- Выделение граней бевела по всему мешу (BEVEL_SELECT_FACES_V1) -------------------------------------------------------------
#
# Подсказка владельцу перед нажатием по граням: кнопка выделяет грани бевела, владелец правит выделение руками, нажатие по граням
# по-прежнему считает выделение правдой. Порядок источников:
# 1. атрибут Face Strength есть и в нём есть STRONG (16384, режим All: так помечены ВСЕ исходные грани) - грани со значением <= 0
#    (полоса 0, узел -16384), точный ответ `BEVEL_SELECT_FACE_STRENGTH`;
# 2. атрибут есть, STRONG нет (режим New: полоса 0 неотличима от исходной 0) - не гадать молча: геометрия с именем
#    `BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS`; узлы WEAK (-16384) точны и добавляются к геометрии, расхождение - числом;
# 3. атрибута нет - геометрия `BEVEL_SELECT_GEOMETRIC`; ничего не найдено - `BEVEL_SELECT_NONE_FOUND`, выделение не тронуто.
# Геометрия (порт classify.py исследования бевела): узкий квад с параллельными рельсами, которые гнут поверхность, - кандидат; ряд
# кандидатов, связанных общими рельсами, проходится тем же путём колонки (`_walk_column`), что и разбиение «узел | полоса»; ряд принят,
# если профиль поворачивает в одну сторону на 10-170 градусов и уже `BEVEL_SELECT_MAX_WIDTH`; узел - малая грань, где сходятся
# полосы. Каждое решение эвристики - событие с числом.
# Угол бевелей (BEVEL_SELECT_CORNERS_V1, поле: фаски у торцов шумного верха стены пропускались): угол рельса читается сквозь
# вырожденный квад, ряд режется там, где рельс уходит от первого рельса куска на допуск рельса, крайний рельс может слегка гнуть
# против профиля, кусок у торца (короткий или трапеция) - полоса, если продолжает найденное не шире его, узел - и компонента малых
# граней с одним касанием каждая; малая грань у бевела, не ставшая узлом, названа числом по причине.

BEVEL_SELECT_FACE_STRENGTH = "BEVEL_SELECT_FACE_STRENGTH"
BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS = "BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS"
BEVEL_SELECT_GEOMETRIC = "BEVEL_SELECT_GEOMETRIC"
BEVEL_SELECT_NONE_FOUND = "BEVEL_SELECT_NONE_FOUND"
# Узлы атрибута против геометрии (путь 2): WEAK-грань, которую геометрия не нашла (добавлена), и узел геометрии без WEAK (оставлен).
BEVEL_SELECT_NODE_FROM_ATTRIBUTE = "BEVEL_SELECT_NODE_FROM_ATTRIBUTE"
BEVEL_SELECT_NODE_NOT_WEAK = "BEVEL_SELECT_NODE_NOT_WEAK"
# События геометрии: почему ряд узких квадов не стал (целиком) полосой.
BEVEL_BAND_NOT_PARALLEL = "BEVEL_BAND_NOT_PARALLEL"
BEVEL_ROW_WRAPS = "BEVEL_ROW_WRAPS"
BEVEL_ROW_TRIMMED_FLAT_END = "BEVEL_ROW_TRIMMED_FLAT_END"
BEVEL_ROW_SPLIT_AT_WIDEST_FACE = "BEVEL_ROW_SPLIT_AT_WIDEST_FACE"
BEVEL_ROW_REJECTED = "BEVEL_ROW_REJECTED"
# Узкий квад с параллельными рельсами шире предела ширины (но не вдвое): не кандидат, число названо (поднять «Max Bevel Width»).
BEVEL_FACE_WIDER_THAN_MAX = "BEVEL_FACE_WIDER_THAN_MAX"
# Концы профиля у угла бевелей (BEVEL_SELECT_CORNERS_V1), каждое решение - числом:
# ребро, чей угол прочитан сквозь вырожденный квад (щель, которую сжал clamp overlap между двумя полосами), - число рёбер;
BEVEL_EDGE_THROUGH_DEGENERATE_FACE = "BEVEL_EDGE_THROUGH_DEGENERATE_FACE"
# ряд принят, хотя крайний рельс слегка гнёт против профиля (шумная плоская грань у края фаски), - число рядов;
BEVEL_ROW_END_COUNTER_TURN = "BEVEL_ROW_END_COUNTER_TURN"
# полоса-трапеция (рельсы сходятся на 5-20 градусов: clamp overlap срезал отступ у вершины), принятая в ряд, - число граней;
BEVEL_FACE_TAPERED = "BEVEL_FACE_TAPERED"
# короткий кусок полосы у торца (квад фаски шире половины длины), принятый, потому что концом касается бевела, - число граней;
BEVEL_ROW_SHORT_STRIP = "BEVEL_ROW_SHORT_STRIP"
# ряд кусков у торца шире ряда, который он продолжал бы, - не полоса, число рядов.
BEVEL_END_PIECE_WIDER = "BEVEL_END_PIECE_WIDER"
# грань ряда - не сегмент профиля (BEVEL_SELECT_SEED_V1), число граней: крайняя вдвое шире остальных без выполаживания профиля к ней
# (наклонная грань шумного верха рядом с фаской в один сегмент) - ряд без неё; средняя шире обеих соседок (своя фаска торца между
# бевелями её рёбер) - ряд режется по ней.
BEVEL_ROW_UNEVEN_FACE = "BEVEL_ROW_UNEVEN_FACE"
# узел из компоненты малых граней, каждая из которых касается одной полосы (угол трёх бевелей, пара треугольников), - число граней;
BEVEL_NODE_CLUSTER = "BEVEL_NODE_CLUSTER"
# малая грань у бевела не стала узлом: ребро к не-бевелу плоское (грань лежит в плоскости соседа) либо касание одно - число граней.
BEVEL_NODE_FLAT_EDGE = "BEVEL_NODE_FLAT_EDGE"
BEVEL_NODE_ONE_CONTACT = "BEVEL_NODE_ONE_CONTACT"

FACE_STRENGTH_STRONG = 16384
FACE_STRENGTH_WEAK = -16384

#: Наибольшая ширина полосы бевела по умолчанию (метры мира; кнопка даёт её править): шире - стена или уступ, а не фаска.
#: Фикстуры Unbevel владельца: 0.25 - P 0.977 R 0.985, 0.10 - P 0.991 R 0.983; 0.10 уже фаски 0.08 в один сегмент (полоса 0.113).
BEVEL_SELECT_MAX_WIDTH = 0.25
#: Рельсы квада и рельсы ряда параллельны в этом допуске.
BEVEL_PARALLEL_DEGREES = 5.0
#: Ребро, повёрнутое меньше этого, - плоское (конец ряда обрезается).
BEVEL_FLAT_RAIL_DEGREES = 0.5
#: Ребро узла не к полосе и не к узлу гнёт поверхность хотя бы на столько. classify.py пускал и плоское ребро к малой грани:
#: на кубе с сеткой это 24 лишних угловых грани (плоские обрезки сетки у фаски), а убрать допуск ничего не стоило ни синтетике,
#: ни фикстурам Unbevel.
BEVEL_NODE_MIN_TURN_DEGREES = 2.0
#: Профиль ряда поворачивает не меньше и не больше этого (больше - тонкая стенка или уступ).
BEVEL_ROW_MIN_TURN_DEGREES = 10.0
BEVEL_ROW_MAX_TURN_DEGREES = 170.0
#: Узел помещается в этот множитель ширины полосы, которой касается (с запасом 1.5).
BEVEL_NODE_DIAMETER_FACTOR = 2.5
#: Крайний рельс ряда может гнуть против профиля не больше этого (шумная грань у края фаски; уступ гнёт на 90).
BEVEL_ROW_END_COUNTER_DEGREES = 15.0
#: Компонента малых граней больше этого - мелкая гнутая поверхность, а не угол бевелей (угол трёх бевелей в 4 сегмента - 12 граней).
BEVEL_NODE_CLUSTER_MAX_FACES = 64
#: Грань вырождена, если её площадь не больше этой доли квадрата поперечника (щель 20 мкм на рельсе 2 м).
BEVEL_DEGENERATE_AREA = 1e-5
#: Короткий кусок полосы у торца: ширина не больше этой доли длины (узкий квад - не больше половины).
BEVEL_SHORT_STRIP_RATIO = 1.0
#: Кусок у торца не шире этого множителя ряда, который он продолжает (наклонная грань шумного верха у фаски - втрое шире).
BEVEL_END_PIECE_WIDTH_FACTOR = 1.5
#: Крайняя грань ряда не шире этого множителя остальных (наклонная грань шумного верха рядом с фаской в один сегмент - вдвое шире;
#: эллипс Percent - втрое, но профиль к широкой хорде выполаживается).
BEVEL_ROW_UNEVEN_FACTOR = 1.75


@dataclass(frozen=True, slots=True)
class BevelMeshViewV1:
    """Весь меш: координаты (мир), циклы вершин всех граней, значения Face Strength всех граней либо `None` (атрибута нет), выделенные
    грани (семена `bevel_seed`, BEVEL_SELECT_SEED_V1)."""

    coords: Mapping[int, tuple[float, float, float]]
    faces: Mapping[int, tuple[int, ...]]
    face_strength: Mapping[int, int] | None = None
    selected: frozenset[int] = frozenset()


@dataclass(frozen=True, slots=True)
class BevelSelectionV1:
    """Предложение выделения: исход (`BEVEL_SELECT_*`, от семян - `bevel_seed`), каким путём искали, полосы, узлы, события (имя, число),
    семена (выделение владельца, от которого росла группа; пусто - поиск по всему мешу)."""

    outcome: str
    source: str
    strip: frozenset[int] = frozenset()
    node: frozenset[int] = frozenset()
    events: tuple[tuple[str, int], ...] = ()
    seeds: frozenset[int] = frozenset()

    @property
    def faces(self) -> frozenset[int]:
        return self.strip | self.node


@dataclass(frozen=True, slots=True)
class BevelDetectionV1:
    """Геометрия всего меша: ряды полос `(грани поперёк профиля по порядку, ширина, поворот профиля в градусах)`, узлы, события."""

    rows: tuple[tuple[tuple[int, ...], float, float], ...]
    node: frozenset[int]
    events: tuple[tuple[str, int], ...] = ()

    @property
    def strip(self) -> frozenset[int]:
        return frozenset(face_id for row, _width, _turn in self.rows for face_id in row)


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _angle_degrees(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(a, b)))))


def _count(events: dict[str, int], name: str) -> None:
    events[name] = events.get(name, 0) + 1


class _MeshGeometry:
    """Нормали, центры, поперечники граней и знаковые двугранные углы рёбер (+ выпуклое), лениво и с памятью."""

    def __init__(self, view: BevelMeshViewV1, max_width: float):
        self.view = view
        self.max_width = float(max_width)
        self.table = _edge_faces(view)
        self._normals: dict[int, tuple | None] = {}
        self._dihedrals: dict[frozenset, float | None] = {}
        self._diameters: dict[int, float] = {}
        self.through: set[frozenset] = set()  # рёбра, чей угол прочитан сквозь вырожденный квад (`BEVEL_EDGE_THROUGH_DEGENERATE_FACE`)
        self.node_width: dict[int, float] = {}  # ширина, от которой мерялся принятый узел (`_node_width`)

    def normal(self, face_id):
        """Нормаль либо `None` у вырожденной грани: площадь меньше `BEVEL_DEGENERATE_AREA` квадрата поперечника (clamp overlap
        сжимает щель не в точный ноль, и нормаль остатка - шум, который дал бы рельсу ложный угол против профиля)."""

        if face_id not in self._normals:
            vector = _newell(self.view.coords, self.view.faces[face_id])
            degenerate = 0.5 * _length(vector) <= BEVEL_DEGENERATE_AREA * self.diameter(face_id) ** 2
            self._normals[face_id] = None if degenerate else _unit(vector)
        return self._normals[face_id]

    def traversal(self, face_id, key):
        """Ребро `key` в обходе грани: вектор от первой вершины ко второй."""

        cycle = self.view.faces[face_id]
        for position, first in enumerate(cycle):
            second = cycle[(position + 1) % len(cycle)]
            if frozenset((first, second)) == key:
                return _sub(self.view.coords[second], self.view.coords[first])
        raise KeyError(key)

    def diameter(self, face_id) -> float:
        if face_id not in self._diameters:
            points = [self.view.coords[index] for index in self.view.faces[face_id]]
            self._diameters[face_id] = max(_length(_sub(a, b)) for a in points for b in points)
        return self._diameters[face_id]

    def dihedral(self, key):
        """Градусы, + выпуклое, - вогнутое; `None` у ребра не двух граней или у вырожденной грани."""

        if key not in self._dihedrals:
            faces = self.table.get(key, ())
            value = None
            if len(faces) == 2:
                first, second = faces
                if self.normal(second) is None and self.normal(first) is not None:
                    second = self._beyond_degenerate(key, second)
                elif self.normal(first) is None and self.normal(second) is not None:
                    first, second = second, self._beyond_degenerate(key, first)
                n1 = self.normal(first)
                n2 = None if second is None else self.normal(second)
                if n1 is not None and n2 is not None:
                    # Выпукло, если n1 x n2 смотрит по обходу ребра первой гранью (согласованные нормали). Центр второй грани
                    # (classify.py) ошибается у невыпуклой грани: центр L-образной грани лежит по другую сторону ребра внутреннего угла.
                    angle = _angle_degrees(n1, n2)
                    value = angle if _dot(_cross(n1, n2), self.traversal(first, key)) > 0.0 else -angle
                    if second not in faces:
                        self.through.add(key)
            self._dihedrals[key] = value
        return self._dihedrals[key]

    def _beyond_degenerate(self, key, face_id):
        """Грань за вырожденным квадом `face_id` напротив ребра `key` либо `None`: clamp overlap сжимает до нуля плоскую щель между
        двумя полосами, и угол ребра - угол между гранями по обе стороны щели (иначе рельс без угла, и полоса пропадает молча)."""

        cycle = self.view.faces[face_id]
        if len(cycle) != 4:
            return None
        beyond = [other for other in self.table.get(_opposite(cycle, key), ()) if other != face_id]
        return beyond[0] if len(beyond) == 1 and self.normal(beyond[0]) is not None else None


def _rail_candidate(geometry: _MeshGeometry, face_id, ratio=0.5):
    """Узкий квад (ширина не больше `ratio` длины): `(рельс, рельс, ширина, трапеция ли)`; `None` - не квад, рельсы не параллельны,
    не узок, рельс не двух граней; узкий квад шире предела, но не вдвое, - имя `BEVEL_FACE_WIDER_THAN_MAX`."""

    cycle = geometry.view.faces[face_id]
    if len(cycle) != 4 or geometry.normal(face_id) is None:  # вырожденный квад - щель, сквозь которую читается угол, а не полоса
        return None
    coords = geometry.view.coords
    edges = _edges_of(cycle)
    best = None
    for k in (0, 1):
        d1 = _sub(coords[cycle[k + 1]], coords[cycle[k]])
        d2 = _sub(coords[cycle[(k + 3) % 4]], coords[cycle[k + 2]])
        u1, u2 = _unit(d1), _unit(d2)
        if u1 is None or u2 is None:
            continue
        parallel = min(_angle_degrees(u1, u2), _angle_degrees(u1, (-u2[0], -u2[1], -u2[2])))
        widths = []
        for index in (cycle[k + 2], cycle[(k + 3) % 4]):
            offset = _sub(coords[index], coords[cycle[k]])
            along = _dot(offset, u1)
            widths.append(_length(_sub(offset, (u1[0] * along, u1[1] * along, u1[2] * along))))
        width = sum(widths) / 2.0
        length = min(_length(d1), _length(d2))
        if best is None or width / max(length, 1e-9) < best[1] / max(best[2], 1e-9):
            best = (parallel, width, length, edges[k], edges[k + 2])
    if best is None:
        return None
    parallel, width, length, first, second = best
    if parallel > (BEVEL_PARALLEL_DEGREES if ratio <= 0.5 else RAIL_TOLERANCE_DEGREES) or width > ratio * length:
        return None
    if geometry.dihedral(first) is None or geometry.dihedral(second) is None:
        return None
    if width > geometry.max_width:  # близкий промах (до двух пределов) назван: фаска чуть шире предела; дальше - просто не фаска
        return BEVEL_FACE_WIDER_THAN_MAX if width <= 2.0 * geometry.max_width else None
    # Кусок у торца (`ratio` > 0.5, `_end_pieces`): рельсы могут сходиться до допуска рельса Unbevel (clamp overlap срезал отступ
    # у одной вершины полосы - трапеция), квад - быть шире половины длины (митра съела длину куска).
    return first, second, width, parallel > BEVEL_PARALLEL_DEGREES


def _bands(candidates):
    """Ряды кандидатов: компоненты связности по общим рельсам (порядок - по наименьшей грани)."""

    rail_faces: dict[frozenset, list[int]] = {}
    for face_id in sorted(candidates):
        for key in candidates[face_id][:2]:
            rail_faces.setdefault(key, []).append(face_id)
    seen: set[int] = set()
    bands = []
    for start in sorted(candidates):
        if start in seen:
            continue
        band, stack = [], [start]
        seen.add(start)
        while stack:
            face_id = stack.pop()
            band.append(face_id)
            for key in candidates[face_id][:2]:
                for other in rail_faces[key]:
                    if other not in seen:
                        seen.add(other)
                        stack.append(other)
        bands.append(sorted(band))
    return bands, rail_faces


def _accept_row(geometry, candidates, row, rails, events):
    """Ряд граней между рельсами `rails[i]`, `rails[i + 1]`: плоские концы обрезаются, неудачный ряд режется по широчайшей грани."""

    while row and abs(geometry.dihedral(rails[0])) < BEVEL_FLAT_RAIL_DEGREES:
        row, rails = row[1:], rails[1:]
        _count(events, BEVEL_ROW_TRIMMED_FLAT_END)
    while row and abs(geometry.dihedral(rails[-1])) < BEVEL_FLAT_RAIL_DEGREES:
        row, rails = row[:-1], rails[:-1]
        _count(events, BEVEL_ROW_TRIMMED_FLAT_END)
    if not row:
        return []
    turns = [geometry.dihedral(key) for key in rails]
    total = sum(turns)
    # Крайний рельс, слегка гнущий против профиля, - шумная плоская грань у края фаски (верх стены с шумом), а не уступ: он не голосует
    # за знак (`BEVEL_ROW_END_COUNTER_TURN`); раньше ряд резался по широчайшей грани, и у угла терялись две полосы из трёх.
    counter = {index for index in (0, len(turns) - 1) if total * turns[index] < 0.0 and abs(turns[index]) <= BEVEL_ROW_END_COUNTER_DEGREES}
    signs = {turn > 0.0 for index, turn in enumerate(turns) if abs(turn) >= BEVEL_FLAT_RAIL_DEGREES and index not in counter}
    width = sum(candidates[face_id][2] for face_id in row)
    turn = abs(total)
    for end, others, outer, shared in ((0, row[1:], 0, 1), (-1, row[:-1], -1, -2)) if len(row) > 1 else ():
        # Крайняя грань не уже `BEVEL_ROW_UNEVEN_FACTOR` любой другой грани ряда (щель, сжатая clamp overlap, - не мерило), и её внешний
        # рельс гнёт по профилю не меньше общего: профиль к ней не выполаживается (эллипс Percent кладёт широкую хорду туда, где поворот
        # мал) - это соседняя грань поверхности, а не сегмент.
        wide = candidates[row[end]][2] >= BEVEL_ROW_UNEVEN_FACTOR * max(candidates[face_id][2] for face_id in others)
        if wide and total * turns[outer] >= total * turns[shared]:
            _count(events, BEVEL_ROW_UNEVEN_FACE)
            keep = slice(1, None) if end == 0 else slice(None, -1)
            return _accept_row(geometry, candidates, row[keep], rails[keep], events)
    for index in range(1, len(row) - 1):
        # Средняя грань шире обеих соседок либо уже обеих: хорды одной дуги не горбятся и не проваливаются (эллипс меняет их монотонно) -
        # это своя фаска исходного меша между двумя бевелями её рёбер (торец стены: 2 + 1 + 2; clamp overlap сжимает её в щель), ряд
        # режется по ней: иначе два бевеля по 45 градусов читаются одной дугой 90 с вдвое большим отступом.
        width_here = candidates[row[index]][2]
        widths = [candidates[row[index + step]][2] for step in (-1, 1)]
        if all(width_here >= BEVEL_ROW_UNEVEN_FACTOR * width for width in widths) or \
                all(width >= BEVEL_ROW_UNEVEN_FACTOR * width_here for width in widths):
            _count(events, BEVEL_ROW_UNEVEN_FACE)
            return (
                _accept_row(geometry, candidates, row[:index], rails[: index + 1], events)
                + _accept_row(geometry, candidates, row[index + 1 :], rails[index + 1 :], events)
            )
    if BEVEL_ROW_MIN_TURN_DEGREES <= turn <= BEVEL_ROW_MAX_TURN_DEGREES and width <= geometry.max_width and len(signs) == 1:
        if counter:  # плоские концы уже обрезаны: крайний рельс против профиля гнёт заметно, решение поменяло ответ
            _count(events, BEVEL_ROW_END_COUNTER_TURN)
        return [(row, width, turn)]
    if len(row) == 1:
        _count(events, BEVEL_ROW_REJECTED)
        return []
    widest = max(range(len(row)), key=lambda index: candidates[row[index]][2])
    _count(events, BEVEL_ROW_SPLIT_AT_WIDEST_FACE)
    return (
        _accept_row(geometry, candidates, row[:widest], rails[: widest + 1], events)
        + _accept_row(geometry, candidates, row[widest + 1 :], rails[widest + 1 :], events)
    )


def _parallel_degrees(a, b) -> float:
    return min(_angle_degrees(a, b), _angle_degrees(a, (-b[0], -b[1], -b[2])))


def _rail_direction(geometry, key):
    first, second = sorted(key)
    return _unit(_sub(geometry.view.coords[second], geometry.view.coords[first])) or (0.0, 0.0, 0.0)


def _band_rows(geometry, candidates, band, rail_faces, events):
    """Ряды полосы из ряда кандидатов: путь колонки (`_walk_column`) от конца ряда; ряд режется там, где рельс уходит от первого рельса
    куска на `RAIL_TOLERANCE_DEGREES` (`BEVEL_BAND_NOT_PARALLEL`). Раньше ряд, где хоть одна грань отходила от первой на
    `BEVEL_PARALLEL_DEGREES`, отбрасывался целиком: у угла митра и clamp overlap поворачивают рельсы на ~3 градуса за грань."""

    members = frozenset(band)

    def lonely(key):
        return sum(1 for face_id in rail_faces[key] if face_id in members) == 1

    ends = [face_id for face_id in band if any(lonely(key) for key in candidates[face_id][:2])]
    if not ends:
        _count(events, BEVEL_ROW_WRAPS)
        return []
    start = ends[0]
    entry = next(key for key in candidates[start][:2] if lonely(key))
    walk_view = BevelFacesViewV1(geometry.view.coords, geometry.view.faces, members, {})
    walked = _walk_column(walk_view, geometry.table, entry, start)
    if isinstance(walked, str):
        _count(events, walked)
        return []
    row = list(walked[0])
    rails, key = [entry], entry
    for face_id in row:
        key = _opposite(geometry.view.faces[face_id], key)
        rails.append(key)
    rows, begin = [], 0
    for index in range(1, len(row) + 1):
        if index < len(row):
            if _parallel_degrees(_rail_direction(geometry, rails[begin]), _rail_direction(geometry, rails[index + 1])) < RAIL_TOLERANCE_DEGREES:
                continue
            _count(events, BEVEL_BAND_NOT_PARALLEL)
        rows += _accept_row(geometry, candidates, row[begin:index], rails[begin : index + 1], events)
        begin = index
    return rows


def _node_edge_ok(geometry, face_id, key, taken) -> bool:
    """Ребро узла: за ним полоса или узел, либо оно гнёт поверхность."""

    others = [other for other in geometry.table.get(key, ()) if other != face_id]
    if not others:
        return False
    if others[0] in taken:
        return True
    turn = geometry.dihedral(key)
    return turn is not None and abs(turn) >= BEVEL_NODE_MIN_TURN_DEGREES


def _touching(geometry, face_id, taken) -> set[int]:
    return {other for key in _edges_of(geometry.view.faces[face_id]) for other in geometry.table.get(key, ()) if other != face_id and other in taken}


def _node_width(geometry, band_width, touch) -> float:
    """Ширина, от которой меряется узел: самая широкая полоса, которой он касается; узел, касающийся только узлов, наследует наименьшую
    их ширину (`geometry.node_width`: широкий узел не раздувает соседей; BEVEL_SELECT_SEED_V1: раньше он мерялся долей предела ширины,
    и большая n-угольная грань исходного меша у угла торца, зажатая между узлами, проходила в узлы); ни полосы, ни узла - доля предела."""

    strips = [band_width[other] for other in touch if other in band_width]
    if strips:
        return max(strips)
    nodes = [geometry.node_width[other] for other in touch if other in geometry.node_width]
    return min(nodes) if nodes else geometry.max_width / BEVEL_NODE_DIAMETER_FACTOR


def _node_limit(geometry, band_width, touch) -> float:
    """Наибольший поперечник узла: множитель ширины `_node_width` (с запасом 1.5)."""

    return BEVEL_NODE_DIAMETER_FACTOR * max(_node_width(geometry, band_width, touch), 1e-6) * 1.5


def _frontier(geometry, taken) -> list[int]:
    """Грани у полос и узлов; вырожденная грань (без нормали) не узел - сквозь неё читается угол ребра."""

    table = geometry.table
    found = {other for face_id in taken for key in _edges_of(geometry.view.faces[face_id]) for other in table.get(key, ())}
    return sorted(face_id for face_id in found - taken if geometry.normal(face_id) is not None)


def _node_cluster(geometry, start, taken, limit):
    """Компонента малых граней от `start` через рёбра, которые гнут поверхность: угол трёх бевелей в 2+ сегмента, пара треугольников
    у вершины полосы - каждая грань касается одной полосы. `(компонента, узел ли)`: узел, если вся компонента помещается в предел узла
    `limit` (гнутая мелочь, что тянется дальше, - шум или кривизна поверхности, а не узел), касается >= 2 граней полос или узлов и
    ни одно её ребро к не-бевелу не плоское."""

    table, coords = geometry.table, geometry.view.coords
    cluster, stack = {start}, [start]
    points = [coords[index] for index in geometry.view.faces[start]]
    while stack:
        face_id = stack.pop()
        for key in _edges_of(geometry.view.faces[face_id]):
            turn = geometry.dihedral(key)
            if turn is None or abs(turn) < BEVEL_NODE_MIN_TURN_DEGREES:
                continue
            for other in table.get(key, ()):
                if other in cluster or other in taken or geometry.diameter(other) > limit or geometry.normal(other) is None:
                    continue  # большая грань - граница узла
                cluster.add(other)
                fresh = [coords[index] for index in geometry.view.faces[other]]
                if len(cluster) > BEVEL_NODE_CLUSTER_MAX_FACES or any(_length(_sub(a, b)) > limit for a in fresh for b in points):
                    return cluster, False
                points.extend(fresh)
                stack.append(other)
    if len(set().union(*(_touching(geometry, face_id, taken) for face_id in cluster))) < 2:
        return cluster, False
    inside = taken | cluster
    return cluster, all(_node_edge_ok(geometry, face_id, key, inside) for face_id in cluster for key in _edges_of(geometry.view.faces[face_id]))


def _single_nodes(geometry, taken, band_width) -> set[int]:
    """Малые грани фронта, касающиеся >= 2 граней полос или узлов, каждое иное ребро которых гнёт поверхность (по одной за проход)."""

    found = set()
    for face_id in _frontier(geometry, taken):
        touch = _touching(geometry, face_id, taken)
        if len(touch) < 2 or geometry.diameter(face_id) > _node_limit(geometry, band_width, touch):
            continue
        if all(_node_edge_ok(geometry, face_id, key, taken) for key in _edges_of(geometry.view.faces[face_id])):
            geometry.node_width[face_id] = _node_width(geometry, band_width, touch)
            found.add(face_id)
            taken.add(face_id)
    return found


def _grow_nodes(geometry, taken_before: frozenset[int], band_width, events) -> frozenset[int]:
    """Новые узлы у граней `taken_before`: малая грань между >= 2 гранями полос или узлов; когда такие кончились - компоненты малых
    граней с одним касанием каждая (`BEVEL_NODE_CLUSTER`)."""

    taken = set(taken_before)
    node: set[int] = set()
    while True:
        found = _single_nodes(geometry, taken, band_width)
        if not found:
            tried: set[int] = set()
            for face_id in _frontier(geometry, taken):
                limit = _node_limit(geometry, band_width, _touching(geometry, face_id, taken))
                if face_id in tried or face_id in taken or geometry.diameter(face_id) > limit:
                    continue
                cluster, accepted = _node_cluster(geometry, face_id, taken, limit)
                if not accepted:  # компонента, оборванная на пределе, зависит от старта: от другой грани она может уложиться
                    tried.add(face_id)
                    continue
                width = _node_width(geometry, band_width, _touching(geometry, face_id, taken))
                geometry.node_width.update((member, width) for member in cluster)
                tried |= cluster
                found |= cluster
                taken |= cluster
                events[BEVEL_NODE_CLUSTER] = events.get(BEVEL_NODE_CLUSTER, 0) + len(cluster)
        if not found:
            break
        node |= found
    return frozenset(node)


def _count_node_misses(geometry, taken, band_width, events) -> None:
    """Малая грань у бевела, которая узлом не стала, - числом по причине: плоское ребро к не-бевелу либо открытая граница
    (`BEVEL_NODE_FLAT_EDGE`, грань лежит в плоскости соседа: веер бевела на шумном верху, конец полосы у неснятого ребра, плоская
    обрезка сетки у фаски) либо одно касание (`BEVEL_NODE_ONE_CONTACT`)."""

    for face_id in _frontier(geometry, taken):
        if geometry.diameter(face_id) <= _node_limit(geometry, band_width, _touching(geometry, face_id, taken)):
            flat = not all(_node_edge_ok(geometry, face_id, key, taken) for key in _edges_of(geometry.view.faces[face_id]))
            _count(events, BEVEL_NODE_FLAT_EDGE if flat else BEVEL_NODE_ONE_CONTACT)


def _end_pieces(geometry, candidates, taken, band_width, events):
    """Куски полосы у торца, которые не узкий квад с параллельными рельсами: митра съела длину куска (квад фаски в 1 сегмент шире
    половины длины, но не шире длины - `BEVEL_ROW_SHORT_STRIP`) либо clamp overlap срезал отступ у одной вершины (рельсы сходятся
    на 5-20 градусов - `BEVEL_FACE_TAPERED`). Ряд таких кусков - полоса, только если он концом (ребром не-рельсом) продолжает полосу
    или узел и проходит тот же `_accept_row`: сам по себе такой квад - и наклонная грань шумного верха; ряд, бевела не касающийся,
    не рассматривается. Число - граней."""

    pieces = {}
    for face_id in sorted(geometry.view.faces):
        if face_id not in candidates and face_id not in taken:
            found = _rail_candidate(geometry, face_id, BEVEL_SHORT_STRIP_RATIO)
            if found is not None and not isinstance(found, str):
                pieces[face_id] = found
    bands, rail_faces = _bands(pieces)
    rows = []
    for band in bands:
        ends = [key for face_id in band for key in _edges_of(geometry.view.faces[face_id]) if key not in pieces[face_id][:2]]
        reference = max((_continued_width(geometry, other, taken, band_width) for key in ends for other in geometry.table.get(key, ())
                         if other in taken), default=0.0)
        for row, width, turn in _band_rows(geometry, pieces, band, rail_faces, events) if reference > 0.0 else ():
            if width > BEVEL_END_PIECE_WIDTH_FACTOR * reference:  # шире полосы, которую продолжает: наклонная грань шумного верха
                _count(events, BEVEL_END_PIECE_WIDER)
                continue
            rows.append((row, width, turn))
            for face_id in row:
                _count(events, BEVEL_FACE_TAPERED if pieces[face_id][3] else BEVEL_ROW_SHORT_STRIP)
    return rows


def _continued_width(geometry, face_id, taken, band_width) -> float:
    """Ширина ряда, который продолжает кусок у торца: у полосы - её ряд, у узла - самый широкий ряд полос, которых узел касается."""

    if face_id in band_width:
        return band_width[face_id]
    return max((band_width.get(other, 0.0) for other in _touching(geometry, face_id, taken)), default=0.0)


def detect_bevel_rows(view: BevelMeshViewV1, max_width: float = BEVEL_SELECT_MAX_WIDTH) -> BevelDetectionV1:
    """Геометрия всего меша: ряды полос, узлы, события; атрибут не читается. `max_width` - наибольшая ширина полосы (метры мира)."""

    geometry = _MeshGeometry(view, max_width)
    candidates = {}
    events: dict[str, int] = {}
    for face_id in sorted(view.faces):
        found = _rail_candidate(geometry, face_id)
        if isinstance(found, str):
            _count(events, found)
        elif found is not None:
            candidates[face_id] = found
    bands, rail_faces = _bands(candidates)
    strip: set[int] = set()
    band_width: dict[int, float] = {}
    rows = [row for band in bands for row in _band_rows(geometry, candidates, band, rail_faces, events)]
    found_rows = []
    node: frozenset[int] = frozenset()
    while rows:  # полосы, их узлы, затем куски у торца, что продолжают найденное, и их узлы
        for row, width, turn in rows:
            strip.update(row)
            band_width.update((face_id, width) for face_id in row)
            found_rows.append((tuple(row), width, turn))
        node = node | _grow_nodes(geometry, frozenset(strip) | node, band_width, events)
        rows = _end_pieces(geometry, candidates, strip | node, band_width, events)
    _count_node_misses(geometry, strip | node, band_width, events)
    if geometry.through:
        events[BEVEL_EDGE_THROUGH_DEGENERATE_FACE] = len(geometry.through)
    return BevelDetectionV1(tuple(found_rows), node, tuple(sorted(events.items())))


def detect_bevel_faces(view: BevelMeshViewV1, max_width: float = BEVEL_SELECT_MAX_WIDTH):
    """Геометрия всего меша: `(полосы, узлы, события)` (`detect_bevel_rows` без рядов)."""

    found = detect_bevel_rows(view, max_width)
    return found.strip, found.node, found.events


def select_bevel_faces(view: BevelMeshViewV1, max_width: float = BEVEL_SELECT_MAX_WIDTH) -> BevelSelectionV1:
    """Предложение выделения граней бевела по порядку источников (атрибут All, иначе геометрия по имени); ничего - по имени."""

    strength = view.face_strength
    if strength is not None and any(int(value) == FACE_STRENGTH_STRONG for value in strength.values()):
        strip = frozenset(face_id for face_id, value in strength.items() if int(value) == 0)
        node = frozenset(face_id for face_id, value in strength.items() if int(value) < 0)
        source, events = BEVEL_SELECT_FACE_STRENGTH, ()
    else:
        strip, node, events = detect_bevel_faces(view, max_width)
        source = BEVEL_SELECT_GEOMETRIC
        if strength is not None:
            source = BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS
            weak = frozenset(face_id for face_id, value in strength.items() if int(value) == FACE_STRENGTH_WEAK)
            extra = ((BEVEL_SELECT_NODE_FROM_ATTRIBUTE, len(weak - node)), (BEVEL_SELECT_NODE_NOT_WEAK, len(node - weak)))
            events = tuple(sorted((*events, *(item for item in extra if item[1]))))
            node = node | weak
            strip = strip - node
    if not strip and not node:
        return BevelSelectionV1(BEVEL_SELECT_NONE_FOUND, source, events=events)
    return BevelSelectionV1(source, source, strip, node, events)


_SOURCE_WORD = {
    BEVEL_SELECT_FACE_STRENGTH: "face strength",
    BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS: "face strength not All, geometric",
    BEVEL_SELECT_GEOMETRIC: "geometric",
}


def selection_line(name: str, selection: BevelSelectionV1) -> str:
    """Строка объекта: исход и числа (от семян - и число семян); неоднозначный атрибут называет, что даёт точный ответ."""

    if selection.seeds:
        text = f"{name}: {selection.outcome} {len(selection.seeds)} seeds"
        if selection.faces:
            text += f" -> {len(selection.strip)} strip / {len(selection.node)} node"
        text += f" ({_SOURCE_WORD[selection.source]})"
    elif selection.outcome == BEVEL_SELECT_NONE_FOUND:
        text = f"{name}: {BEVEL_SELECT_NONE_FOUND} ({_SOURCE_WORD[selection.source]})"
    else:
        text = f"{name}: {selection.outcome} {len(selection.strip)} strip / {len(selection.node)} node"
    if selection.source == BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS:
        text += " - set the Bevel Face Strength mode to All for the exact answer"
    return text


def selection_console_lines(name: str, selection: BevelSelectionV1) -> list[str]:
    """Строка объекта и каждое событие эвристики - строкой консоли с префиксом `[CFTUV]`."""

    lines = [f"[CFTUV][BevelSelect] {selection_line(name, selection)}"]
    lines.extend(f"[CFTUV][BevelSelect] {name}: {event} {count}" for event, count in selection.events)
    return lines


__all__ = (
    "BEVEL_BAND_NOT_PARALLEL",
    "BEVEL_CLASS_DISAGREES",
    "BEVEL_EDGE_THROUGH_DEGENERATE_FACE",
    "BEVEL_FACES_EMPTY_SELECTION",
    "BEVEL_FACES_NOTHING_OUTSIDE",
    "BEVEL_FACES_NO_SOURCE",
    "BEVEL_END_PIECE_WIDER",
    "BEVEL_FACE_TAPERED",
    "BEVEL_FACE_WIDER_THAN_MAX",
    "BEVEL_NODE_CLUSTER",
    "BEVEL_NODE_FLAT_EDGE",
    "BEVEL_NODE_ONE_CONTACT",
    "BEVEL_ROW_END_COUNTER_TURN",
    "BEVEL_ROW_REJECTED",
    "BEVEL_ROW_SHORT_STRIP",
    "BEVEL_ROW_SPLIT_AT_WIDEST_FACE",
    "BEVEL_ROW_TRIMMED_FLAT_END",
    "BEVEL_ROW_UNEVEN_FACE",
    "BEVEL_ROW_WRAPS",
    "BEVEL_SELECT_FACE_STRENGTH",
    "BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS",
    "BEVEL_SELECT_GEOMETRIC",
    "BEVEL_SELECT_MAX_WIDTH",
    "BEVEL_SELECT_NODE_FROM_ATTRIBUTE",
    "BEVEL_SELECT_NODE_NOT_WEAK",
    "BEVEL_SELECT_NONE_FOUND",
    "BEVEL_SPLIT_FACE_STRENGTH",
    "BEVEL_SPLIT_GEOMETRIC",
    "BEVEL_SPLIT_UNRESOLVED",
    "BevelDetectionV1",
    "BevelFacesPlanV1",
    "BevelFacesViewV1",
    "BevelMeshViewV1",
    "BevelSelectionV1",
    "BevelSplitV1",
    "FACE_SIDES",
    "FACE_STRENGTH_ATTRIBUTE",
    "FACE_STRENGTH_STRONG",
    "FACE_STRENGTH_WEAK",
    "INWARD",
    "OUTWARD",
    "cut_edges",
    "detect_bevel_faces",
    "detect_bevel_rows",
    "geometric_split",
    "outward_sources",
    "plan_console_lines",
    "plan_faces",
    "plan_summary",
    "reconcile_with_selection",
    "scope_digest",
    "seam_edges",
    "select_bevel_faces",
    "selection_console_lines",
    "selection_line",
    "split_bevel_faces",
)
