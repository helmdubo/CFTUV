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


@dataclass(frozen=True, slots=True)
class BevelMeshViewV1:
    """Весь меш: координаты (мир), циклы вершин всех граней, значения Face Strength всех граней либо `None` (атрибута нет)."""

    coords: Mapping[int, tuple[float, float, float]]
    faces: Mapping[int, tuple[int, ...]]
    face_strength: Mapping[int, int] | None = None


@dataclass(frozen=True, slots=True)
class BevelSelectionV1:
    """Предложение выделения: исход (один из четырёх `BEVEL_SELECT_*`), каким путём искали, полосы, узлы, события (имя, число)."""

    outcome: str
    source: str
    strip: frozenset[int] = frozenset()
    node: frozenset[int] = frozenset()
    events: tuple[tuple[str, int], ...] = ()

    @property
    def faces(self) -> frozenset[int]:
        return self.strip | self.node


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

    def normal(self, face_id):
        if face_id not in self._normals:
            self._normals[face_id] = face_normal(self.view.coords, self.view.faces[face_id])
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
                n1, n2 = self.normal(first), self.normal(second)
                if n1 is not None and n2 is not None:
                    # Выпукло, если n1 x n2 смотрит по обходу ребра первой гранью (согласованные нормали). Центр второй грани
                    # (classify.py) ошибается у невыпуклой грани: центр L-образной грани лежит по другую сторону ребра внутреннего угла.
                    angle = _angle_degrees(n1, n2)
                    value = angle if _dot(_cross(n1, n2), self.traversal(first, key)) > 0.0 else -angle
            self._dihedrals[key] = value
        return self._dihedrals[key]


def _rail_candidate(geometry: _MeshGeometry, face_id):
    """Узкий квад: `(рельс, рельс, ширина, направление)`; `None` - не квад, рельсы не параллельны, не узок, рельс не двух граней;
    узкий квад шире предела, но не вдвое, - имя `BEVEL_FACE_WIDER_THAN_MAX`."""

    cycle = geometry.view.faces[face_id]
    if len(cycle) != 4:
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
            best = (parallel, width, length, edges[k], edges[k + 2], u1)
    if best is None:
        return None
    parallel, width, length, first, second, direction = best
    if parallel > BEVEL_PARALLEL_DEGREES or width > 0.5 * length:
        return None
    if geometry.dihedral(first) is None or geometry.dihedral(second) is None:
        return None
    if width > geometry.max_width:  # близкий промах (до двух пределов) назван: фаска чуть шире предела; дальше - просто не фаска
        return BEVEL_FACE_WIDER_THAN_MAX if width <= 2.0 * geometry.max_width else None
    return first, second, width, direction


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
    signs = {turn > 0.0 for turn in turns if abs(turn) >= BEVEL_FLAT_RAIL_DEGREES}
    width = sum(candidates[face_id][2] for face_id in row)
    turn = abs(sum(turns))
    if BEVEL_ROW_MIN_TURN_DEGREES <= turn <= BEVEL_ROW_MAX_TURN_DEGREES and width <= geometry.max_width and len(signs) == 1:
        return [(row, width)]
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


def _band_rows(geometry, candidates, band, rail_faces, events):
    """Ряды полосы из ряда кандидатов: рельсы параллельны, путь колонки (`_walk_column`) от конца ряда; иначе событие с числом."""

    first = candidates[band[0]][3]
    if any(_parallel_degrees(first, candidates[face_id][3]) >= BEVEL_PARALLEL_DEGREES for face_id in band):
        _count(events, BEVEL_BAND_NOT_PARALLEL)
        return []
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
    return _accept_row(geometry, candidates, row, rails, events)


def _node_edge_ok(geometry, face_id, key, taken) -> bool:
    """Ребро узла: за ним полоса или узел, либо оно гнёт поверхность."""

    others = [other for other in geometry.table.get(key, ()) if other != face_id]
    if not others:
        return False
    if others[0] in taken:
        return True
    turn = geometry.dihedral(key)
    return turn is not None and abs(turn) >= BEVEL_NODE_MIN_TURN_DEGREES


def _grow_nodes(geometry, strip: frozenset[int], band_width) -> frozenset[int]:
    """Узлы: малая грань, касающаяся ребром >= 2 граней полос или узлов; каждое её иное ребро гнёт поверхность."""

    table = geometry.table
    taken = set(strip)
    node: set[int] = set()
    changed = True
    while changed:
        changed = False
        frontier = {other for face_id in taken for key in _edges_of(geometry.view.faces[face_id]) for other in table.get(key, ())}
        for face_id in sorted(frontier - taken):
            edges = _edges_of(geometry.view.faces[face_id])
            touch = {other for key in edges for other in table.get(key, ()) if other != face_id and other in taken}
            if len(touch) < 2:
                continue
            width = max(band_width.get(other, 0.0) for other in touch)
            if width <= 0.0:
                width = geometry.max_width / BEVEL_NODE_DIAMETER_FACTOR
            limit = BEVEL_NODE_DIAMETER_FACTOR * max(width, 1e-6) * 1.5
            if geometry.diameter(face_id) > limit:
                continue
            if all(_node_edge_ok(geometry, face_id, key, taken) for key in edges):
                node.add(face_id)
                taken.add(face_id)
                changed = True
    return frozenset(node)


def detect_bevel_faces(view: BevelMeshViewV1, max_width: float = BEVEL_SELECT_MAX_WIDTH):
    """Геометрия всего меша: `(полосы, узлы, события)`; атрибут не читается. `max_width` - наибольшая ширина полосы (метры мира)."""

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
    for band in bands:
        for row, width in _band_rows(geometry, candidates, band, rail_faces, events):
            for face_id in row:
                strip.add(face_id)
                band_width[face_id] = width
    node = _grow_nodes(geometry, frozenset(strip), band_width)
    return frozenset(strip), node, tuple(sorted(events.items()))


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
    """Строка объекта: исход и числа; неоднозначный атрибут называет, что даёт точный ответ."""

    if selection.outcome == BEVEL_SELECT_NONE_FOUND:
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
    "BEVEL_FACES_EMPTY_SELECTION",
    "BEVEL_FACES_NOTHING_OUTSIDE",
    "BEVEL_FACES_NO_SOURCE",
    "BEVEL_FACE_WIDER_THAN_MAX",
    "BEVEL_ROW_REJECTED",
    "BEVEL_ROW_SPLIT_AT_WIDEST_FACE",
    "BEVEL_ROW_TRIMMED_FLAT_END",
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
    "geometric_split",
    "outward_sources",
    "plan_console_lines",
    "plan_faces",
    "plan_summary",
    "scope_digest",
    "select_bevel_faces",
    "selection_console_lines",
    "selection_line",
    "split_bevel_faces",
)
