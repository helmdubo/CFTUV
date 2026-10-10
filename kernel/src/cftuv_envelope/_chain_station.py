"""Закон `CHAIN_STATION_PLAN_V1`: какие вершины физической цепи декаль оставляет, решённое ОДИН РАЗ при компиляции.

Чистый лист без `reference` (как `_corner_fold`, `_corner_treatment`): законом пользуются компиляция, проверяющий плана
(`validation_chain_station`) и материализатор; ответ зависит только от снапшота.

ЗАЧЕМ. Вершина цепи, лежащая на прямой между соседями по цепи, своей формой силуэт не рисует. Рисует её то, что к ней прикреплено:
поперечное ребро источника (ребро меша, уходящее от вершины внутрь патча) режет грани декали, и вершина становится концом этого реза.
Если поверхность поперёк такого ребра плоская в допуске, рез ничего не меняет и вершина лишняя. Раньше её растворял проход по готовым
батчам всех доменов прогона (вердикт на каждое место по сетке каждого домена; проход удалён); этот закон решает то же ДО построения и по
поверхности источника, а не по сетке: резка не режет по инертным рёбрам, проход силуэта растворяет сами вершины (`materialize.silhouette`).

ДВА ВИДА ВЕРШИН. Хост режет цепь в каждом ТОЧНОМ изломе (`float32` почти никогда не даёт точной прямой, поэтому ровная стена с вершинами
на ней — цепочка кусков по одному ребру, а не один кусок). (1) ВНУТРЕННЯЯ вершина куска (`INTERIOR_VERTEX`): точно на прямой. (2) СТЫК двух
кусков (`CHAIN_JOINT`): вершина, где кончается один кусок и начинается другой, и больше цепей в ней нет; излом между кусками — доли
градуса. На `building` стыки — 11 из 38 мест, которые растворял прежний проход, на меше `2` — 5 из 17: закон без них оставил бы эти
точки в мешах. Стык с изломом или хордой за допуском — настоящий угол цепи, записи у него нет (решать нечего: вершина несётся как всегда).

РЕШЕНИЕ. `FREE`, когда в вершине (а) излом цепи не больше `CANONICAL_RESTORATION_ARTIST_ERROR` (тот же художественный допуск «это прямая»,
что у канонического угла и у прежнего прохода), (б) вершина отстоит от прямой между соседями по цепи не дальше допуска хорды, (в) в КАЖДОМ
патче цепи (обе стороны шва) все поперечные рёбра инертны, (г) грани, которые эти рёбра склеивают, плоски вместе. `REQUIRED` иначе, с
именованной причиной (`ChainStationReasonV1`). Решение не зависит от выбора цепей запроса: иначе два домена общей цепи (кэш по содержимому
домена не видит соседа) решили бы вершину по-разному и дали T-стык шва. Поэтому вход закона — поверхность вокруг вершины на ОБЕИХ сторонах
цепи: своя берётся из `surface_ir`, чужая — из `seam_neighbour_faces` снапшота (факт хоста).

ПОПЕРЕЧНОЕ РЕБРО. Ребро источника в вершине, не принадлежащее цепи, с ровно двумя гранями ОДНОГО патча. Другой шов или край меша
в вершине — `JUNCTION`; ребро не с двумя гранями, грань без площади — `NOT_MANIFOLD`.

ИНЕРТНОСТЬ. Ребро инертно, когда все вершины каждой из двух его граней отстоят от плоскости другой не дальше допуска хорды
`CLIP_DIAGONAL_CHORD_BUDGET` (5 мм, запись реестра `CLIP_DIAGONAL_CHORD_DEPTH_V1`: одно число, одно место). Плоскость грани — Ньюэлла
через центроид, точная (`Fraction(float)`: позиции снапшота рациональны точно); сравнение — квадратов, `d^2 <= B^2 |n|^2`, ни корня, ни
чисел с плавающей точкой. Точная копланарность непригодна: позиции float32, и ровная стена даёт ненулевые отклонения.

ПЛОСКОСТЬ ВМЕСТЕ. Грани, склеенные инертными рёбрами вершин-кандидатов ОДНОЙ цепи, образуют группы (связные компоненты по патчу); группа
плоска, когда все вершины всех её граней отстоят не дальше допуска от плоскости её наибольшей грани (по модулю нормали, затем по имени). Без
этого цепочка рёбер, каждое из которых в допуске, дрейфовала бы на десятки миллиметров (прецедент — «без дрейфа цепочки слияний» закона
`SILHOUETTE_TOPOLOGY_V1`). Вершины неплоской группы — `REQUIRED:GROUP_NOT_FLAT` все разом (порядок обхода на ответ не влияет).

ДРЕЙФ ЛИНИИ. Вершина проверена по своим соседям, но подряд идущие `FREE`-вершины вместе уводят ломаную от прямой между несомыми вершинами
(малые изломы копятся: большая дуга мелкими кусками). Линия цепей — куски, соединённые стыками (`CHAIN_JOINT`); окно идёт по ней слева направо
жадно: вершина остаётся `FREE`, пока ВСЕ пропущенные с начала окна отстоят от отрезка «последняя несомая — следующая вершина» не дальше допуска
хорды (расстояние до ОТРЕЗКА, точно), иначе она несётся (`REQUIRED:RUN_CHORD_BEYOND_BUDGET`) и открывает новое окно. Концы линии несутся всегда.
Так растворение всех `FREE`-вершин цепи не выводит силуэт за допуск, а решение остаётся функцией снапшота: оба домена общей цепи видят одни и те же
куски и стыки.

ЧЕГО ЗАКОН НЕ ЗНАЕТ, И ЭТО НАЗВАНО. Положений нет (координатно-свободная выгрузка) — `POSITIONS_UNAVAILABLE`; другая сторона шва не выгружена —
`NEIGHBOUR_SIDE_UNKNOWN`; цепь замкнута — `CLOSED_CHAIN`. Карта домена — развёртка: решение то же, причина `FREE_UNDER_CHART`
(UV вдоль выпрямленного ребра совпадает с прежней в пределах растяжения карты, а не точно). Эвристика меняет ответ, поэтому ни одно решение
не молчит: запись плана есть у каждой внутренней вершины и каждого стыка каждой цепи домена.
"""

from __future__ import annotations

from collections import defaultdict
from fractions import Fraction

from ._authoring_intent import CANONICAL_RESTORATION_ARTIST_ERROR
from .contracts.analysis import PhysicalChainKind
from .contracts.chain_station import (
    ChainStationDispositionV1,
    ChainStationKindV1,
    ChainStationLawV1,
    ChainStationPlanV1,
    ChainStationReasonV1,
    ChainStationV1,
)
from .contracts.metric import RationalAffinePlanarMetricV2, is_unfolded_certificate
from .contracts.surface import SurfacePayloadMode
from .numeric import ExactRatioV1, LocalPoint3V1

LAW = ChainStationLawV1.CHAIN_STATION_PLAN_V1
FREE = ChainStationDispositionV1.FREE
REQUIRED = ChainStationDispositionV1.REQUIRED
INTERIOR = ChainStationKindV1.INTERIOR_VERTEX
JOINT = ChainStationKindV1.CHAIN_JOINT
R = ChainStationReasonV1

#: Излом цепи в вершине, до которого вершина — точка на прямой: художественная точность канонического угла, одна запись реестра.
#: Сравнение квадратов синуса (`sin^2(a) <= L^2`, `L` — радианы): синус не больше угла, поэтому допуск — на `L^2 / 6` шире угла, то есть до 1e-12.
BEND_LIMIT: Fraction = CANONICAL_RESTORATION_ARTIST_ERROR


def flatness_budget() -> Fraction:
    """Допуск закона, метры: запись реестра `CLIP_DIAGONAL_CHORD_DEPTH_V1`, объявленная в `materialize.clip_cells` (одно число, одно место).

    Читается при вызове, а не при импорте: `materialize` читает `reference` (через `lift`), а `reference.compile` читает этот модуль, и
    импорт числа наверху замкнул бы цикл.
    """

    from .materialize.clip_cells import CLIP_DIAGONAL_CHORD_BUDGET

    return CLIP_DIAGONAL_CHORD_BUDGET


def budget_record() -> ExactRatioV1:
    """Допуск закона как запись плана."""

    budget = flatness_budget()
    return ExactRatioV1(budget.numerator, budget.denominator)


def _sub(first, second):
    return (first[0] - second[0], first[1] - second[1], first[2] - second[2])


def _dot(first, second):
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2]


def _cross(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


class _Face:
    """Грань источника для закона: имя, патч, обход вершин, точные положения (`None` — положений нет) и плоскость Ньюэлла (лениво)."""

    __slots__ = ("face_id", "patch_id", "cycle", "points", "_plane")

    def __init__(self, face_id, patch_id, cycle, points) -> None:
        self.face_id = face_id
        self.patch_id = patch_id
        self.cycle = cycle
        self.points = points
        self._plane = False

    @property
    def plane(self):
        """`(центроид, нормаль Ньюэлла, |нормаль|^2)` либо `None`: положений нет или у грани нет площади."""

        if self._plane is False:
            self._plane = None if self.points is None else _newell(self.points)
        return self._plane


def _newell(points):
    origin = points[0]
    normal = (Fraction(0), Fraction(0), Fraction(0))
    for index in range(1, len(points) - 1):
        normal = tuple(
            axis + part
            for axis, part in zip(normal, _cross(_sub(points[index], origin), _sub(points[index + 1], origin)))
        )
    squared = _dot(normal, normal)
    if not squared:
        return None
    count = len(points)
    centroid = tuple(sum(point[axis] for point in points) / count for axis in range(3))
    return centroid, normal, squared


def _within(plane, points, budget) -> bool:
    """Все `points` отстоят от плоскости не дальше допуска: `(n . (p - c))^2 <= B^2 |n|^2`, точно."""

    centroid, normal, squared = plane
    limit = budget * budget * squared
    for point in points:
        height = _dot(normal, _sub(point, centroid))
        if height * height > limit:
            return False
    return True


class StationFacts:
    """Грани вокруг вершин снапшота и решения по ним; один экземпляр на снапшот и на дверь, состояния вне снапшота нет."""

    def __init__(self, snapshot) -> None:
        self.budget = flatness_budget()
        full = snapshot.surface_ir.payload_mode is SurfacePayloadMode.FULL_HOST_SURFACE
        self.position: dict = {}
        for item in snapshot.source_vertices:
            if isinstance(item.position, LocalPoint3V1):
                self.position[item.vertex_id] = (Fraction(item.position.x), Fraction(item.position.y), Fraction(item.position.z))
        self.faces: dict = {}
        for face in snapshot.surface_ir.source_faces:
            points = tuple(self.position.get(vertex) for vertex in face.vertex_cycle) if full else None
            if points is not None and None in points:
                points = None
            self.faces[face.face_id] = _Face(face.face_id, face.patch_id, tuple(face.vertex_cycle), points)
        for face in snapshot.seam_neighbour_faces:
            if face.face_id not in self.faces:
                points = tuple((Fraction(item.x), Fraction(item.y), Fraction(item.z)) for item in face.positions)
                self.faces[face.face_id] = _Face(face.face_id, face.patch_id, tuple(face.vertex_ids), points)
                for vertex, point in zip(face.vertex_ids, points):
                    self.position.setdefault(vertex, point)
        self.at: dict = defaultdict(list)
        for face in self.faces.values():
            for vertex in face.cycle:
                self.at[vertex].append(face)
        for faces in self.at.values():
            faces.sort(key=lambda item: item.face_id.value)
        self.corner_vertices = frozenset(
            item.source_vertex_id for relations in (snapshot.corner_relations, snapshot.junction_relations) for item in relations
        )
        self._inert: dict = {}
        self._floats: dict = {}
        self._float_limit = float(self.budget * self.budget)

    def inert(self, first: _Face, second: _Face) -> bool:
        """Ребро между гранями инертно: вершины каждой отстоят от плоскости другой не дальше допуска (кэш по паре)."""

        key = (first.face_id, second.face_id) if first.face_id.value <= second.face_id.value else (second.face_id, first.face_id)
        found = self._inert.get(key)
        if found is None:
            found = self._inert[key] = _within(first.plane, second.points, self.budget) and _within(
                second.plane, first.points, self.budget
            )
        return found

    def flat_together(self, faces) -> bool:
        """Группа граней плоска: вершины всех отстоят не дальше допуска от плоскости наибольшей (нормаль, затем имя)."""

        biggest = max(faces, key=lambda item: (item.plane[2], item.face_id.value))
        return all(_within(biggest.plane, face.points, self.budget) for face in faces)

    def beyond_segment(self, first, second, point) -> bool:
        """Точка `point` дальше допуска хорды от ОТРЕЗКА `first - second` (положения вершин): точно; ясные случаи решает binary64 с запасом 1e-6.

        Квадрат расстояния до отрезка: у концов — квадрат расстояния до конца, внутри — квадрат расстояния до прямой; сравнение с `B^2`.
        """

        fast = self._beyond_by_floats(first, second, point)
        if fast is not None:
            return fast
        start, end, middle = self.position[first], self.position[second], self.position[point]
        whole, along = _sub(end, start), _sub(middle, start)
        reach = _dot(along, whole)
        if reach <= 0:
            return _dot(along, along) > self.budget * self.budget
        if reach >= _dot(whole, whole):
            far = _sub(middle, end)
            return _dot(far, far) > self.budget * self.budget
        cross = _cross(along, whole)
        return _dot(cross, cross) > self.budget * self.budget * _dot(whole, whole)

    def _beyond_by_floats(self, first, second, point):
        """`True`/`False`, если binary64 решает с запасом, иначе `None` (решает точный путь)."""

        floats = self._floats
        for name in (first, second, point):
            if name not in floats:
                floats[name] = tuple(float(axis) for axis in self.position[name])
        a, b, p = floats[first], floats[second], floats[point]
        w = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
        reach, whole = v[0] * w[0] + v[1] * w[1] + v[2] * w[2], w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
        if reach <= 0.0:
            square, limit = v[0] * v[0] + v[1] * v[1] + v[2] * v[2], self._float_limit
        elif reach >= whole:
            square, limit = (p[0] - b[0]) ** 2 + (p[1] - b[1]) ** 2 + (p[2] - b[2]) ** 2, self._float_limit
        else:
            cross = (v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0])
            square, limit = cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2, self._float_limit * whole
        if square <= limit * (1.0 - 1e-6):
            return False
        if square >= limit * (1.0 + 1e-6):
            return True
        return None

    def not_straight(self, before, vertex, after):
        """Причина, по которой цепь `before - vertex - after` не прямая в вершине, либо `None`: излом в допуске, хорда в допуске.

        Излом: `sin^2 <= BEND_LIMIT^2` и вперёд (`a . b > 0`), точно. Хорда: расстояние вершины от прямой `before - after` не больше допуска.
        """

        first, second = self.position.get(before), self.position.get(after)
        middle = self.position.get(vertex)
        if first is None or second is None or middle is None:
            return R.POSITIONS_UNAVAILABLE
        one, two, whole = _sub(middle, first), _sub(second, middle), _sub(second, first)
        one_square, two_square, whole_square = _dot(one, one), _dot(two, two), _dot(whole, whole)
        if not one_square or not two_square or not whole_square:
            return R.NOT_MANIFOLD
        turn = _cross(one, two)
        if _dot(one, two) <= 0 or _dot(turn, turn) > BEND_LIMIT * BEND_LIMIT * one_square * two_square:
            return R.BEND_BEYOND_STRAIGHT
        off = _cross(one, whole)
        if _dot(off, off) > self.budget * self.budget * whole_square:
            return R.CHORD_BEYOND_BUDGET
        return None


def _expected_faces(kind) -> int:
    """Граней на ребре цепи: край меша — одна, шов (двух патчей либо одного патча с двух сторон) — две."""

    return 1 if kind is PhysicalChainKind.PHYSICAL_DECAL_SOURCE else 2


def _station(facts: StationFacts, vertex, arms) -> tuple:
    """`(решение, причина, пары)` одной вершины цепи; `FREE` здесь — кандидат (группы решает `_demote_deep_groups`).

    `arms` — `((сосед по цепи, вид цепи), (сосед по цепи, вид цепи))`: два ребра цепи в вершине (у внутренней вершины куска оба
    из одной цепи, у стыка — по одному из каждого куска).
    """

    if vertex in facts.corner_vertices:
        return REQUIRED, R.JUNCTION, ()
    faces = facts.at.get(vertex, ())
    if not faces or any(face.points is None for face in faces):
        return REQUIRED, R.POSITIONS_UNAVAILABLE, ()
    if any(face.cycle.count(vertex) != 1 or face.plane is None for face in faces):
        return REQUIRED, R.NOT_MANIFOLD, ()
    edges: dict = defaultdict(list)
    for face in faces:
        at = face.cycle.index(vertex)
        for other in (face.cycle[at - 1], face.cycle[(at + 1) % len(face.cycle)]):
            edges[other].append(face)
    ends = tuple(other for other, _kind in arms)
    for other, kind in arms:
        found, wanted = len(edges.get(other, ())), _expected_faces(kind)
        if found > wanted:
            return REQUIRED, R.NOT_MANIFOLD, ()
        if found < wanted:
            return REQUIRED, R.NEIGHBOUR_SIDE_UNKNOWN if kind is PhysicalChainKind.PHYSICAL_SEAM else R.NOT_MANIFOLD, ()
    pairs, folded = [], False
    for other in sorted(edges, key=lambda item: item.value):
        if other in ends:
            continue
        sides = edges[other]
        if len(sides) > 2:
            return REQUIRED, R.NOT_MANIFOLD, ()
        if len(sides) < 2 or sides[0].patch_id != sides[1].patch_id:
            return REQUIRED, R.JUNCTION, ()
        first, second = sorted(sides, key=lambda item: item.face_id.value)
        if facts.inert(first, second):
            pairs.append((first.face_id, second.face_id))
        else:
            folded = True
    bent = facts.not_straight(ends[0], vertex, ends[1])
    if bent is not None:
        return REQUIRED, bent, ()
    if folded:
        return REQUIRED, R.FOLD, ()
    return FREE, R.TRANSVERSE_EDGES_INERT, tuple(sorted(pairs, key=lambda pair: (pair[0].value, pair[1].value)))


def _demote_deep_groups(facts: StationFacts, found: dict) -> None:
    """Группы граней, склеенных инертными рёбрами вершин-кандидатов ОДНОЙ цепи, плоские вместе; вершины неплоской группы — `GROUP_NOT_FLAT`.

    Группы считаются по цепи, а не по всему домену: решение обязано быть одним у двух доменов общей цепи, а каждый видит поверхность
    вокруг ЕЁ вершин (своя сторона целиком, чужая — грани у вершин цепи), но не остальные цепи чужого патча. Грани, лежащие у двух цепей
    сразу (тонкий патч), группой не сливаются в законе; резка домена склеивает их по парам, как они записаны. Стык кусков решается один
    раз, а записан в обоих; группа считается в плане каждого куска по его вершинам, и вердикт стыка берётся из той группы, что неплоская.
    """

    for stations in found.values():
        parent: dict = {}

        def root(item):
            parent.setdefault(item, item)
            while parent[item] != item:
                parent[item] = parent[parent[item]]
                item = parent[item]
            return item

        for _kind, disposition, _reason, pairs in stations.values():
            if disposition is FREE:
                for first, second in pairs:
                    parent[root(first)] = root(second)
        groups: dict = defaultdict(list)
        for face_id in list(parent):
            groups[root(face_id)].append(facts.faces[face_id])
        deep = {key for key, faces in groups.items() if not facts.flat_together(faces)}
        for ordinal, (kind, disposition, _reason, pairs) in list(stations.items()):
            if disposition is FREE and any(root(first) in deep for first, _second in pairs):
                stations[ordinal] = (kind, REQUIRED, R.GROUP_NOT_FLAT, ())


def _lines(found: dict, vertices_of: dict) -> list:
    """Линии цепей: куски цепей домена, соединённые стыками (`CHAIN_JOINT`), в порядке обхода: `[[(вершина, ((цепь, порядковый), ...)), ...], ...]`.

    У стыка две записи (по одной в каждом из двух кусков): вершина линии одна, ссылок у неё две. Линия без свободного конца (кольцо из
    кусков) режется в куске с наименьшим именем: концы линии несутся всегда. Порядок и начало линии — функции снапшота, поэтому два домена
    общей цепи строят одни линии.
    """

    link: dict = {}
    joined: dict = defaultdict(list)
    for chain_id, stations in found.items():
        vertices = vertices_of[chain_id]
        for ordinal, (kind, *_rest) in stations.items():
            if kind is JOINT:
                joined[vertices[ordinal]].append(chain_id)
    for vertex, ids in joined.items():
        if len(ids) == 2 and ids[0] != ids[1]:
            one, two = ids
            end_one, end_two = int(vertices_of[one][0] != vertex), int(vertices_of[two][0] != vertex)
            link[(one, end_one)], link[(two, end_two)] = (two, end_two), (one, end_one)
    visited: set = set()
    lines = []
    for start in sorted(found, key=lambda item: item.value):
        if start in visited or len(vertices_of[start]) < 2:
            continue
        # налево до свободного конца либо до замыкания кольца
        chain, flip, seen = start, False, {start}
        while True:
            ahead = link.get((chain, 1 if flip else 0))
            if ahead is None or ahead[0] in seen:
                break
            chain, flip = ahead[0], ahead[1] == 0
            seen.add(chain)
        if ahead is not None:  # кольцо: начало — кусок с наименьшим именем, ориентация как записана
            chain, flip = min(seen, key=lambda item: item.value), False
        line: list = []
        while chain not in visited:
            visited.add(chain)
            vertices = vertices_of[chain]
            order = range(len(vertices) - 1, -1, -1) if flip else range(len(vertices))
            for ordinal in order:
                ref = (chain, ordinal)
                if line and ordinal == (len(vertices) - 1 if flip else 0) and line[-1][0] == vertices[ordinal]:
                    line[-1] = (line[-1][0], (*line[-1][1], ref))
                else:
                    line.append((vertices[ordinal], (ref,)))
            ahead = link.get((chain, 0 if flip else 1))
            if ahead is None:
                break
            chain, flip = ahead[0], ahead[1] == 1
        lines.append(line)
    return lines


def _demote_runs(facts: StationFacts, found: dict, vertices_of: dict) -> None:
    """Дрейф линии (см. модуль): окно слева направо; вершина, которую окно не вмещает в допуск, несётся и открывает новое окно."""

    for line in _lines(found, vertices_of):
        carried = [
            any(found[chain].get(ordinal, (None, REQUIRED))[1] is REQUIRED for chain, ordinal in refs) for _vertex, refs in line
        ]
        anchor = 0
        for index in range(1, len(line) - 1):
            if carried[index]:
                anchor = index
                continue
            first, second = line[anchor][0], line[index + 1][0]
            if any(facts.beyond_segment(first, second, line[at][0]) for at in range(anchor + 1, index + 1)):
                for chain, ordinal in line[index][1]:
                    kind = found[chain][ordinal][0]
                    found[chain][ordinal] = (kind, REQUIRED, R.RUN_CHORD_BEYOND_BUDGET, ())
                carried[index] = True
                anchor = index


def _chart_is_developed(snapshot, patch_domain_id) -> bool:
    return any(
        isinstance(item, RationalAffinePlanarMetricV2)
        and item.patch_domain_id == patch_domain_id
        and is_unfolded_certificate(item.planarity_certificate)
        for item in snapshot.surface_metric_descriptors
    )


def _joint_arms(chain, other, vertex, ends) -> tuple | None:
    """Два ребра цепи в стыке `vertex` кусков `chain` и `other` либо `None`: `((сосед, вид), (сосед, вид))`, в порядке «до» и «после»."""

    first, second = chain.ordered_source_vertex_ids, other.ordered_source_vertex_ids
    if len(ends[vertex]) != 2 or chain.is_closed or other.is_closed:
        return None
    near_first = first[1] if first[0] == vertex else first[-2]
    near_second = second[1] if second[0] == vertex else second[-2]
    if near_first == near_second:
        return None
    return (near_first, chain.kind), (near_second, other.kind)


def chain_station_plans(snapshot, patch_domain_id, facts: StationFacts | None = None) -> frozenset:
    """Планы станций всех цепей домена: по записи на цепь с вершинами-кандидатами (внутренние вершины куска и стыки кусков).

    Цепь домена — цепь, у которой есть вход (`ChainUse`) в домене, выделена она запросом или нет: стена домена делит вершины с
    соседом так же, как источник. Стык — вершина, где кончаются ровно два куска, оба входят в домен.
    """

    facts = StationFacts(snapshot) if facts is None else facts
    in_domain = {item.physical_chain_id for item in snapshot.chain_uses if item.patch_domain_id == patch_domain_id}
    chains = sorted(
        (item for item in snapshot.physical_chains if item.physical_chain_id in in_domain),
        key=lambda item: item.physical_chain_id.value,
    )
    ends: dict = defaultdict(list)
    for chain in snapshot.physical_chains:
        if not chain.is_closed and len(chain.ordered_source_vertex_ids) >= 2:
            for vertex in {chain.ordered_source_vertex_ids[0], chain.ordered_source_vertex_ids[-1]}:
                ends[vertex].append(chain)
    found: dict = {}
    joints: dict = {}
    for chain in chains:
        vertices = chain.ordered_source_vertex_ids
        if chain.is_closed:
            closed = vertices[:-1] if len(vertices) > 1 and vertices[0] == vertices[-1] else vertices
            found[chain.physical_chain_id] = {ordinal: (INTERIOR, REQUIRED, R.CLOSED_CHAIN, ()) for ordinal in range(len(closed))}
            continue
        stations = {}
        for ordinal in range(1, len(vertices) - 1):
            arms = ((vertices[ordinal - 1], chain.kind), (vertices[ordinal + 1], chain.kind))
            stations[ordinal] = (INTERIOR, *_station(facts, vertices[ordinal], arms))
        for ordinal in {0, len(vertices) - 1} if len(vertices) >= 2 else ():
            vertex = vertices[ordinal]
            partner = next((item for item in ends.get(vertex, ()) if item is not chain and item.physical_chain_id in in_domain), None)
            arms = None if partner is None else _joint_arms(chain, partner, vertex, ends)
            if arms is not None and facts.not_straight(arms[0][0], vertex, arms[1][0]) is None:
                if vertex not in joints:
                    joints[vertex] = _station(facts, vertex, arms)
                stations[ordinal] = (JOINT, *joints[vertex])
        if stations:
            found[chain.physical_chain_id] = stations
    _demote_deep_groups(facts, found)
    _agree_on_joints(found, chains)
    _demote_runs(facts, found, {item.physical_chain_id: item.ordered_source_vertex_ids for item in chains})
    named = R.FREE_UNDER_CHART if _chart_is_developed(snapshot, patch_domain_id) else R.TRANSVERSE_EDGES_INERT
    by_id = {item.physical_chain_id: item for item in chains}
    plans = []
    for chain_id, stations in found.items():
        vertices = by_id[chain_id].ordered_source_vertex_ids
        plans.append(
            ChainStationPlanV1(
                LAW,
                chain_id,
                budget_record(),
                tuple(
                    ChainStationV1(
                        vertices[ordinal],
                        ordinal,
                        kind,
                        disposition,
                        named if reason is R.TRANSVERSE_EDGES_INERT else reason,
                        pairs,
                    )
                    for ordinal, (kind, disposition, reason, pairs) in sorted(stations.items())
                ),
            )
        )
    return frozenset(plans)


def _agree_on_joints(found: dict, chains) -> None:
    """Стык записан в планах обоих кусков и решён одним ответом: неплоская группа у любого куска делает вершину `REQUIRED` у обоих."""

    by_id = {item.physical_chain_id: item.ordered_source_vertex_ids for item in chains}
    demoted: set = set()
    for chain_id, stations in found.items():
        for ordinal, (kind, disposition, reason, _pairs) in stations.items():
            if kind is JOINT and reason is R.GROUP_NOT_FLAT:
                demoted.add(by_id[chain_id][ordinal])
    if not demoted:
        return
    for chain_id, stations in found.items():
        for ordinal, (kind, disposition, _reason, _pairs) in list(stations.items()):
            if kind is JOINT and by_id[chain_id][ordinal] in demoted and disposition is FREE:
                stations[ordinal] = (kind, REQUIRED, R.GROUP_NOT_FLAT, ())


_FREE_REASONS = (R.TRANSVERSE_EDGES_INERT, R.FREE_UNDER_CHART)


def structure_errors(plans) -> tuple:
    """Структура записей без снапшота: имена, порядок, согласие решения и причины; пусто — записи собраны честно."""

    errors, seen = [], set()
    for plan in sorted(plans, key=lambda item: item.physical_chain_id.value):
        name = plan.physical_chain_id.value
        if plan.physical_chain_id in seen:
            errors.append(f"two chain station plans for {name}")
        seen.add(plan.physical_chain_id)
        if plan.law is not LAW:
            errors.append(f"{name}: plan names another law")
        if plan.flatness_budget != budget_record():
            errors.append(f"{name}: plan records a flatness budget other than the registered chord depth")
        ordinals = [station.ordinal for station in plan.stations]
        if ordinals != sorted(set(ordinals)):
            errors.append(f"{name}: station ordinals are not strictly increasing")
        if len({station.source_vertex_id for station in plan.stations}) != len(plan.stations):
            errors.append(f"{name}: a source vertex has two station records")
        for station in plan.stations:
            free = station.disposition is FREE
            if free != (station.reason in _FREE_REASONS):
                errors.append(f"{name}: station {station.ordinal} has a reason that does not belong to its disposition")
            if not free and station.inert_face_pairs:
                errors.append(f"{name}: required station {station.ordinal} lists inert edges")
            if station.kind is JOINT and station.ordinal != 0 and station.ordinal != plan.stations[-1].ordinal:
                errors.append(f"{name}: joint station {station.ordinal} is not at an end of the chain")
    return tuple(errors)


def plan_errors(snapshot, patch_domain_id, plans, facts: StationFacts | None = None, recompute=None) -> tuple:
    """Пересчёт плана по сырому снапшоту: имена расхождений; пусто — каждая запись доказана.

    План без единой записи — прежний план (решений нет, все вершины несутся): пересчитывать нечем и не нужно.
    `recompute` - вызываемое, отдающее ТЕ ЖЕ планы, что `chain_station_plans(snapshot, patch_domain_id)` (транзакция уже посчитала
    их из этого снапшота); без него план пересчитывается здесь.
    """

    plans = tuple(plans)
    if not plans:
        return ()
    errors = list(structure_errors(plans))
    recomputed = chain_station_plans(snapshot, patch_domain_id, facts) if recompute is None else recompute()
    expected = {item.physical_chain_id: item for item in recomputed}
    found = {item.physical_chain_id: item for item in plans}
    for chain_id in sorted(set(expected) | set(found), key=lambda item: item.value):
        if chain_id not in found:
            errors.append(f"{chain_id.value}: the raw snapshot demands a station plan the plan lacks")
        elif chain_id not in expected:
            errors.append(f"{chain_id.value}: station plan names a chain with no stations in the raw snapshot")
        elif found[chain_id] != expected[chain_id]:
            errors.append(f"{chain_id.value}: station plan differs from the raw snapshot")
    return tuple(errors)


def free_vertices(plans) -> frozenset:
    """Значения `SourceVertexId` вершин, которые декаль не несёт (`FREE`)."""

    return frozenset(
        station.source_vertex_id.value
        for plan in plans
        for station in plan.stations
        if station.disposition is FREE
    )


def inert_face_pairs(plans) -> frozenset:
    """Пары граней `frozenset({имя, имя})` по инертным поперечным рёбрам `FREE`-вершин: рёбра, по которым резка не режет."""

    return frozenset(
        frozenset((first.value, second.value))
        for plan in plans
        for station in plan.stations
        if station.disposition is FREE
        for first, second in station.inert_face_pairs
    )
