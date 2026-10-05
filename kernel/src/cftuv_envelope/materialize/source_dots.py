"""Закон `SILHOUETTE_SOURCE_DOTS_V1` (срез S4 закона `SILHOUETTE_TOPOLOGY_V1`): точка на прямой цепи источника или стены растворяется.

ЗАПРОС ВЛАДЕЛЬЦА: «только вершины и рёбра силуэта»; жалоба — точки: вершины степени два на прямых рёбрах границы. Проход по
доменам (`silhouette`) вершин цепей источника и стены не трогает: они общие с соседним доменом по `location:src:`, и решить такую
вершину в одном домене значит открыть шов (T-стык: у соседа на ребре `a - b` вершина `v` осталась бы, а здесь ребро стало бы
`a - b`; `ADAPTER_SEAM_T_JUNCTIONS`). Поэтому их решает ЭТОТ закон, ОДИН РАЗ НА МЕСТО (`location:src:<id>`) и по всем доменам
прогона разом: одно решение для всех доменов, где место есть, а не по решению домена.

ПОЧЕМУ НЕ В ПРОХОДЕ ПО ДОМЕНУ. Решение зависит от того, что домен знает про СОСЕДА: «вершина степени два» и «UV вдоль выпрямленного
ребра» у соседа — свойства его меша (в нём свои слитые перекладины, свой разбор регионов), и в одном домене их не вычислить.
Результат домена при этом остаётся чистой функцией его входа (он лежит в кэшах по содержимому прогона), а общее решение — чистая
функция набора результатов, и считается над ГОТОВЫМИ батчами: ни кэши, ни пул, ни другие законы его не видят.

ЗАКОН. Место растворяется, когда ВО ВСЕХ доменах, где оно есть, выполнено всё:

1. ТОЧКА ПО ФОРМЕ: вершина `src:` — внутренняя вершина цепи источника или стены (`boundary:SOURCE`, `boundary:WALL`), излом цепи в ней
   не больше `CANONICAL_RESTORATION_ARTIST_ERROR` (1745e-6 рад, 0.1 градуса: тот же художественный допуск «это прямая», что у
   канонического угла), положение 3D — по батчу. Вершины-углы цепи законом не считаются: решать нечего.
2. ВЕРШИНА ДВУХ РЁБЕР: степень два, ровно одна грань, не на интерфейсной цепи, без двойника-копии разреза кольца. Иначе — `KEPT_ATTACHED`:
   к вершине прикреплено внутреннее ребро (перекладина, шов регионов, срез), и это ребро силуэт рисует.
3. ТОТ ЖЕ СОСЕДСТВЕННЫЙ ОТРЕЗОК: соседи по цепи (`location:src:` либо промежуточные вершины) во всех доменах одни и те же
   (`KEPT_NEIGHBOURS_DIFFER`). Грани исходника «по обе стороны вершины» вдоль цепи компланарны в пределах глубины хорды: это условие
   слияния перекладины проходом по домену (`silhouette`, хорда 5 мм), и вершина степени два ОДНОЙ грани — его следствие, отдельной
   проверки нет. Сгиб между доменами условием не является: линия излома двух стен — прямая цепь, и точка на ней ничего не рисует
   (на `building` все 13 общих мест лежат на сгибах стен: проверка компланарности соседей оставила бы их все).
4. ХОРДА И UV: все растворённые между соседями вершины отстоят от выпрямленного ребра не дальше `CLIP_DIAGONAL_CHORD_BUDGET`
   (`KEPT_CHORD`), а UV вдоль ребра отличается от прежней в каждом домене не больше `DecalRequestV1.silhouette_uv_slide`
   (`KEPT_UV`, тот же допуск запроса, что у растворения вершин и рёбер).
5. ГРАНЬ ОСТАЁТСЯ ПРОСТОЙ: в окрестности (в пределах глубины хорды) выпрямленного ребра нет ни одной чужой вершины домена и ни одного
   чужого ребра его грани (`KEPT_NOT_SIMPLE`); проверка консервативная и в 3D, без выбора плоскости.

Порядок жадный и детерминированный: меньший излом первым (глубина хорды по возрастанию, затем место). Соседи растворённых вершин —
следующие вершины цепи, и глубина хорды считается по ВСЕЙ цепочке растворённых между ними (как у прохода по доменам).

ЧТО ПИШЕТСЯ. Каждая точка по форме названа исходом на домен: растворена (`SOURCE_DOTS_DISSOLVED`, из них места, общие с другим доменом —
`..._SHARED_DISSOLVED`) либо оставлена под одним из `KEPT_*`; `KEPT_OTHER_DOMAIN` — в этом домене точка прошла бы, но в другом домене места
не прошла (решение одно на место, и оно «оставить»). Наибольшие хорда, сдвиг UV и излом растворённых записаны (нанометры, тысячные alpha,
микроградусы). Нулевые числа не пишутся.

ПРОВЕРКА. `verify_source_dots` пересчитывает независимо от прохода: растворены только внутренние вершины `src:` цепей источника и стены
без интерфейса; место растворено во ВСЕХ доменах, где оно есть, либо ни в одном; кольца и UV граней, факты станций и цепи итога — исходные без
растворённых (цепи пересчитаны тем же `paths_of`); хорда, сдвиг UV и излом каждой растворённой вершины относительно ребра итога не больше
записанных максимумов, а те не больше допусков; итог проходит `validate_geometry_batch` и `audit_batch`, числа батча пересчитаны. Любое
расхождение — НИ ОДНО место не растворено, проблемы названы (правило 4: молчаливого исчезновения нет).
"""

from __future__ import annotations

import math
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from fractions import Fraction
from hashlib import sha256
from typing import NamedTuple

from .._authoring_intent import CANONICAL_RESTORATION_ARTIST_ERROR
from ..canonical import geometry_batch_semantic_digest
from ..codec import canonical_json_bytes
from ..contracts.geometry_batch import GeometryBoundaryChainV1
from ..ids import SemanticBoundaryId, SemanticDigestValue, VertexKey
from ..validation import validate_geometry_batch
from .assemble import paths_of
from .audit import audit_batch, batch_shape_counters
from .offset_normal import offset_normals_digest
from .silhouette import CHORD_BUDGET, _along, _milli_alpha, _nanometres, _within

LAW = "SILHOUETTE_SOURCE_DOTS_V1"

#: Имена чисел закона (они же ключи счётчиков домена). Нулевые числа не пишутся.
_PREFIX = "MATERIALIZE_SILHOUETTE_SOURCE_DOTS_"
DISSOLVED = _PREFIX + "DISSOLVED"
SHARED_DISSOLVED = _PREFIX + "SHARED_DISSOLVED"
KEPT_ATTACHED = _PREFIX + "KEPT_ATTACHED"
KEPT_NEIGHBOURS_DIFFER = _PREFIX + "KEPT_NEIGHBOURS_DIFFER"
KEPT_CHORD = _PREFIX + "KEPT_CHORD"
KEPT_UV = _PREFIX + "KEPT_UV"
KEPT_NOT_SIMPLE = _PREFIX + "KEPT_NOT_SIMPLE"
KEPT_OTHER_DOMAIN = _PREFIX + "KEPT_OTHER_DOMAIN"
MAX_CHORD_NM = _PREFIX + "MAX_CHORD_NM"
MAX_UV_SLIDE_MILLI_ALPHA = _PREFIX + "MAX_UV_SLIDE_MILLI_ALPHA"
MAX_BEND_MICRODEGREES = _PREFIX + "MAX_BEND_MICRODEGREES"
SKIPPED_UNVERIFIED = _PREFIX + "SKIPPED_UNVERIFIED"
COUNTER_NAMES = (
    DISSOLVED,
    SHARED_DISSOLVED,
    KEPT_ATTACHED,
    KEPT_NEIGHBOURS_DIFFER,
    KEPT_CHORD,
    KEPT_UV,
    KEPT_NOT_SIMPLE,
    KEPT_OTHER_DOMAIN,
    MAX_CHORD_NM,
    MAX_UV_SLIDE_MILLI_ALPHA,
    MAX_BEND_MICRODEGREES,
    SKIPPED_UNVERIFIED,
)

SOURCE_PREFIX = "location:src:"
#: Излом цепи, до которого вершина — точка на прямой: художественная точность канонического угла, одна запись реестра.
BEND_LIMIT = float(CANONICAL_RESTORATION_ARTIST_ERROR)
MICRODEGREES_PER_RADIAN = 180.0 * 10**6 / math.pi
_CHAIN_KINDS = ("SOURCE", "WALL")


class SourceDotInputV1(NamedTuple):
    """Один материализованный домен прогона: ключ (номер патча), батч, нормаль источника и нормали смещения вершин."""

    key: object
    batch: object
    source_normal: object = None
    vertex_normals: tuple = ()


@dataclass(frozen=True, slots=True)
class SourceDotsDomainV1:
    """Итог закона по одному домену: новый батч (тот же, если ничего не растворено) и числа."""

    key: object
    batch: object
    content_digest: str
    removed: frozenset
    #: Числа закона домена, только ненулевые.
    counters: tuple
    #: Пересчитанные числа самого батча (`MATERIALIZE_VERTICES`, ...): хост подставляет их по имени в счётчики домена.
    overrides: tuple
    vertex_normals: tuple
    offset_normals_digest: str
    note: str


@dataclass(frozen=True, slots=True)
class SourceDotsV1:
    domains: tuple
    #: Хоть одна вершина растворена.
    changed: bool
    #: Имена нарушений независимой проверки (тогда ничего не растворено).
    problems: tuple = ()


def _sub(first, second):
    return (first[0] - second[0], first[1] - second[1], first[2] - second[2])


def _dot(first, second) -> float:
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2]


def _bend(before, middle, after) -> float:
    """Излом цепи в `middle` (радианы): угол между направлениями `before -> middle` и `middle -> after`; `inf`, если ребро нулевое."""

    one, two = _sub(middle, before), _sub(after, middle)
    cross = (
        one[1] * two[2] - one[2] * two[1],
        one[2] * two[0] - one[0] * two[2],
        one[0] * two[1] - one[1] * two[0],
    )
    length = math.sqrt(_dot(cross, cross))
    along = _dot(one, two)
    return math.inf if not (_dot(one, one) > 0.0 and _dot(two, two) > 0.0) else math.atan2(length, along)


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def _segment_distance(p1, q1, p2, q2) -> float:
    """Наименьшее расстояние между отрезками `p1 - q1` и `p2 - q2` в 3D (замкнутыми)."""

    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    if a <= 0.0 and e <= 0.0:
        s = t = 0.0
    elif a <= 0.0:
        s, t = 0.0, _clamp(f / e)
    else:
        c = _dot(d1, r)
        if e <= 0.0:
            s, t = _clamp(-c / a), 0.0
        else:
            b = _dot(d1, d2)
            denominator = a * e - b * b
            s = _clamp((b * f - c * e) / denominator) if denominator > 0.0 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = _clamp(-c / a), 0.0
            elif t > 1.0:
                s, t = _clamp((b - c) / a), 1.0
    near = tuple(p1[axis] + d1[axis] * s - p2[axis] - d2[axis] * t for axis in range(3))
    return math.sqrt(_dot(near, near))


class _Domain:
    """Рабочая копия сетки одного домена: позиции, ссылки мест, кольца граней, UV, соседи и цепи источника и стены."""

    def __init__(self, item: SourceDotInputV1) -> None:
        batch = item.batch
        self.item = item
        self.pos = {v.vert_key.value: (v.position.x, v.position.y, v.position.z) for v in batch.vertices}
        self.ref = {v.vert_key.value: v.semantic_location_ref.value for v in batch.vertices}
        self.rings = [[key.value for key in face.ordered_vert_keys] for face in batch.faces]
        self.fact = {(index, fact.vert_key.value): fact for index, face in enumerate(batch.faces) for fact in face.uv_facts}
        self.incident: dict = defaultdict(set)
        self.neighbours: dict = defaultdict(set)
        for index, ring in enumerate(self.rings):
            for at, key in enumerate(ring):
                self.incident[key].add(index)
                self.neighbours[key].update((ring[at - 1], ring[(at + 1) % len(ring)]))
        self.interface = {key.value for chain in batch.interface_chains for key in chain.ordered_vert_keys}
        places = Counter(self.ref.values())
        self.twinned = {key for key, ref in self.ref.items() if places[ref] > 1}
        self.live = set(self.pos)
        self.prv: dict = {}
        self.nxt: dict = {}
        self.covered: dict = {}
        #: `{вершина: чистая ли точка}` — точки по форме (пункт 1); чистая — ещё и вершина двух рёбер (пункт 2).
        self.dots: dict = {}
        #: Излом цепи в вершине между её ИСХОДНЫМИ соседями (то, что закон судит и записывает; соседи растворённых потом меняются).
        self.bends: dict = {}
        self.removed: set = set()
        self._scan(batch)

    def _scan(self, batch) -> None:
        seen: Counter = Counter()
        for chain in sorted(batch.boundary_chains, key=lambda item: item.semantic_boundary_id.value):
            if chain.semantic_boundary_id.value.split(":")[1] not in _CHAIN_KINDS:
                continue
            path = [key.value for key in chain.ordered_vert_keys]
            closed = len(path) > 3 and path[0] == path[-1]
            cycle = path[:-1] if closed else path
            size = len(cycle)
            for at, key in enumerate(cycle):
                if not closed and at in (0, size - 1):
                    continue
                seen[key] += 1
                before, after = cycle[at - 1], cycle[(at + 1) % size]
                self.prv[key], self.nxt[key] = before, after
                if seen[key] == 1 and self.ref[key].startswith(SOURCE_PREFIX):
                    self.bends[key] = self.bend(key)
                    if self.bends[key] <= BEND_LIMIT:
                        self.dots[key] = self._pure(key, before, after)
        for key, count in seen.items():
            if count > 1:
                self.dots.pop(key, None)

    def bend(self, key) -> float:
        return _bend(self.pos[self.prv[key]], self.pos[key], self.pos[self.nxt[key]])

    def _pure(self, key, before, after) -> bool:
        """Вершина двух рёбер: степень два, одна грань (кольцо без повтора), не на интерфейсе, без двойника-копии."""

        if key in self.twinned or key in self.interface or self.neighbours[key] != {before, after}:
            return False
        faces = self.incident[key]
        return len(faces) == 1 and self.rings[next(iter(faces))].count(key) == 1 and len(self.rings[next(iter(faces))]) >= 4

    def face_of(self, key) -> int:
        return next(iter(self.incident[key]))

    def run_of(self, key) -> tuple:
        """Растворённые между соседями вершины вместе с `key`: цепочка, которую ребро итога заменит."""

        before, after = self.prv[key], self.nxt[key]
        return (*self.covered.get(frozenset((before, key)), ()), *self.covered.get(frozenset((key, after)), ()), key)

    def slide(self, face, before, after, run):
        """Наибольший сдвиг UV (доли alpha) растворённых `run` относительно выпрямленного ребра; `None` — фактов UV нет."""

        low, high = sorted((before, after))  # концы по ключам: одни числа в обоих доменах, где цепь идёт в разные стороны
        start, end, worst = self.pos[low], self.pos[high], 0.0
        if any((face, item) not in self.fact for item in (low, high, *run)):
            return None
        (u0, v0), (u1, v1) = ((self.fact[(face, item)].uv.u, self.fact[(face, item)].uv.v) for item in (low, high))
        for item in run:
            share = _along(start, end, self.pos[item])[0]
            u, v = self.fact[(face, item)].uv.u, self.fact[(face, item)].uv.v
            worst = max(worst, math.hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share)))
        return worst

    def sag(self, before, after, run) -> float:
        low, high = sorted((before, after))
        start, end = self.pos[low], self.pos[high]
        return max(_along(start, end, self.pos[item])[1] for item in run)

    def simple(self, face, key, before, after) -> bool:
        """Консервативно: рядом (в глубине хорды) с выпрямленным ребром нет чужих вершин домена и чужих рёбер этой грани."""

        low, high = sorted((before, after))
        start, end = self.pos[low], self.pos[high]
        budget = float(CHORD_BUDGET)
        skip = {before, key, after}
        for other in self.live:
            if other not in skip and _along(start, end, self.pos[other])[1] <= budget:
                return False
        ring = self.rings[face]
        for at, first in enumerate(ring):
            second = ring[(at + 1) % len(ring)]
            if skip & {first, second}:
                continue
            if _segment_distance(start, end, self.pos[first], self.pos[second]) <= budget:
                return False
        return True

    def commit(self, key, run) -> None:
        before, after = self.prv[key], self.nxt[key]
        face = self.face_of(key)
        self.rings[face].remove(key)
        for item in (before, after):
            self.neighbours[item].discard(key)
        self.neighbours[before].add(after)
        self.neighbours[after].add(before)
        if before in self.nxt:
            self.nxt[before] = after
        if after in self.prv:
            self.prv[after] = before
        self.covered.pop(frozenset((before, key)), None)
        self.covered.pop(frozenset((key, after)), None)
        self.covered[frozenset((before, after))] = run
        self.live.discard(key)
        self.removed.add(key)
        self.dots.pop(key, None)


class _Pass:
    """Решение по местам: общий жадный проход над всеми доменами прогона."""

    def __init__(self, inputs, slide: Fraction) -> None:
        self.domains = [_Domain(item) for item in inputs]
        self.slide = slide
        self.members: dict = defaultdict(list)
        for index, state in enumerate(self.domains):
            for key, ref in sorted(state.ref.items()):
                if ref.startswith(SOURCE_PREFIX):
                    self.members[ref].append((index, key))
        self.outcome: dict = {}
        self.peaks = [{"chord": 0.0, "slide": 0.0, "bend": 0.0} for _ in self.domains]
        self.shared = [0] * len(self.domains)

    def _name(self, found, reason, own=None) -> None:
        """Исход каждой точки по форме места `found`: `reason` у тех, кто в `own` (`None` — у всех), у остальных — `KEPT_OTHER_DOMAIN`."""

        for index, key in found:
            if key in self.domains[index].dots and (index, key) not in self.outcome:
                self.outcome[(index, key)] = reason if own is None or (index, key) in own else KEPT_OTHER_DOMAIN

    def run(self) -> None:
        order = []
        for ref, found in sorted(self.members.items()):
            shaped = [(index, key) for index, key in found if key in self.domains[index].dots]
            if not shaped:
                continue
            attached = {(index, key) for index, key in shaped if not self.domains[index].dots[key]}
            if attached or len(shaped) != len(found):
                # у места есть участник, который не чистая точка: решение одно на место, и оно «оставить»
                self._name(found, KEPT_ATTACHED, attached)
                continue
            index, key = found[0]
            state = self.domains[index]
            order.append((_along(state.pos[state.prv[key]], state.pos[state.nxt[key]], state.pos[key])[1], ref))
        for _sag, ref in sorted(order):
            self._attempt(self.members[ref])

    def _attempt(self, found) -> None:
        pairs = set()
        for index, key in found:
            state = self.domains[index]
            pairs.add(frozenset((state.ref[state.prv[key]], state.ref[state.nxt[key]])))
        if len(pairs) != 1:
            self._name(found, KEPT_NEIGHBOURS_DIFFER)
            return
        failures, plans = {}, []
        for index, key in found:
            state = self.domains[index]
            before, after = state.prv[key], state.nxt[key]
            run = state.run_of(key)
            sag = state.sag(before, after, run)
            face = state.face_of(key)
            reason = slide = None
            if not _within(sag, CHORD_BUDGET):
                reason = KEPT_CHORD
            else:
                slide = state.slide(face, before, after, run)
                if slide is None or not _within(slide, self.slide):
                    reason = KEPT_UV
                elif not state.simple(face, key, before, after):
                    reason = KEPT_NOT_SIMPLE
            if reason is not None:
                failures[(index, key)] = reason
            plans.append((index, key, run, sag, slide))
        if failures:
            self.outcome.update(failures)
            self._name(found, KEPT_OTHER_DOMAIN, frozenset())
            return
        for index, key, run, sag, slide in plans:
            state = self.domains[index]
            peaks = self.peaks[index]
            peaks["chord"], peaks["slide"] = max(peaks["chord"], sag), max(peaks["slide"], slide)
            peaks["bend"] = max(peaks["bend"], state.bends[key])
            state.commit(key, run)
            self.outcome[(index, key)] = DISSOLVED
            self.shared[index] += int(len(found) > 1)

    def counters(self, index: int) -> tuple:
        tally = Counter(reason for (domain, _key), reason in self.outcome.items() if domain == index)
        peaks = self.peaks[index]
        values = {name: tally[name] for name in COUNTER_NAMES}
        values[SHARED_DISSOLVED] = self.shared[index]
        if tally[DISSOLVED]:
            values[MAX_CHORD_NM] = _nanometres(peaks["chord"])
            values[MAX_UV_SLIDE_MILLI_ALPHA] = _milli_alpha(peaks["slide"])
            values[MAX_BEND_MICRODEGREES] = math.ceil(peaks["bend"] * MICRODEGREES_PER_RADIAN)
        return tuple((name, values[name]) for name in COUNTER_NAMES if values[name])


def _closed(path) -> bool:
    return len(path) > 3 and path[0] == path[-1]


def _bypassed(chain, removed) -> list:
    """Рёбра цепи `((a, b), ...)` по её ключам без `removed`: соседи растворённых вершин соединены прямо (направление цепи то же)."""

    path = [key.value for key in chain.ordered_vert_keys]
    cycle = path[:-1] if _closed(path) else path
    kept = [key for key in cycle if key not in removed]
    pairs = list(zip(kept, kept[1:]))
    return pairs + [(kept[-1], kept[0])] if _closed(path) else pairs


def _rebuilt_chains(batch, removed) -> frozenset:
    """Цепи итога: граничные цепи групп `(вид, регион)` с растворёнными вершинами пересобраны тем же `paths_of`, что у сборки батча."""

    groups: dict = defaultdict(list)
    touched = set()
    for chain in batch.boundary_chains:
        _boundary, kind, region, _number = chain.semantic_boundary_id.value.split(":")
        groups[(kind, region)].append(chain)
        if any(key.value in removed for key in chain.ordered_vert_keys):
            touched.add((kind, region))
    found = [chain for group, chains in groups.items() if group not in touched for chain in chains]
    for (kind, region) in sorted(touched):
        edges = [edge for chain in groups[(kind, region)] for edge in _bypassed(chain, removed)]
        for number, path in enumerate(paths_of(edges)):
            found.append(
                GeometryBoundaryChainV1(SemanticBoundaryId(f"boundary:{kind}:{region}:{number}"), tuple(VertexKey(key) for key in path))
            )
    return frozenset(found)


def _rebuilt(item: SourceDotInputV1, state: _Domain, removed: set):
    """Батч без `removed`: вершины, кольца и UV граней, факты станций, цепи; дайджесты пересчитаны. Нормали смещения без растворённых."""

    batch = item.batch
    faces = []
    for index, face in enumerate(batch.faces):
        ring = state.rings[index]
        if len(ring) == len(face.ordered_vert_keys):
            faces.append(face)
            continue
        faces.append(
            replace(
                face,
                ordered_vert_keys=tuple(VertexKey(key) for key in ring),
                uv_facts=tuple(state.fact[(index, key)] for key in ring),
            )
        )
    shaped = replace(
        batch,
        vertices=frozenset(v for v in batch.vertices if v.vert_key.value not in removed),
        faces=tuple(faces),
        station_facts=frozenset(f for f in batch.station_facts if f.vert_key.value not in removed),
        boundary_chains=_rebuilt_chains(batch, removed),
    )
    shaped = replace(shaped, semantic_digest=SemanticDigestValue(geometry_batch_semantic_digest(shaped).sha256_hex))
    normals = tuple(entry for entry in item.vertex_normals if entry[0] not in removed)
    return shaped, normals


def _audit(item: SourceDotInputV1, batch, normals):
    source = item.source_normal if item.source_normal is not None else (0.0, 0.0, 0.0)
    return audit_batch(batch, source, dict(normals)) if normals else audit_batch(batch, source)


def _finished(item: SourceDotInputV1, state: _Domain, counters: tuple) -> SourceDotsDomainV1:
    removed = state.removed
    if not removed:
        return SourceDotsDomainV1(item.key, item.batch, "", frozenset(), counters, (), item.vertex_normals, "", "")
    batch, normals = _rebuilt(item, state, removed)
    audit = _audit(item, batch, normals)
    note = f"{LAW}: {len(removed)} source-chain vertices dissolved " + " ".join(f"{name.removeprefix(_PREFIX).lower()}={value}" for name, value in counters)
    return SourceDotsDomainV1(
        item.key,
        batch,
        sha256(canonical_json_bytes(batch)).hexdigest(),
        frozenset(removed),
        counters,
        (*batch_shape_counters(batch), *audit.counters()),
        normals,
        offset_normals_digest(normals),
        note,
    )


def _unchanged(inputs, counters, problems) -> SourceDotsV1:
    domains = tuple(
        SourceDotsDomainV1(item.key, item.batch, "", frozenset(), counters_of, (), item.vertex_normals, "", "")
        for item, counters_of in zip(inputs, counters)
    )
    return SourceDotsV1(domains, False, problems)


def verify_source_dots(inputs, result: SourceDotsV1, slide: Fraction) -> tuple:
    """Независимый пересчёт: имена нарушений (пусто — закон выполнен). Читает входные батчи и итог, проход не зовёт."""

    problems: list = []
    removed_refs: dict = defaultdict(set)
    refs_present: dict = defaultdict(set)
    for number, (item, out) in enumerate(zip(inputs, result.domains)):
        refs = {v.vert_key.value: v.semantic_location_ref.value for v in item.batch.vertices}
        for key, ref in refs.items():
            refs_present[ref].add(number)
        for key in out.removed:
            removed_refs[refs.get(key, "")].add(number)
        if out.removed:
            problems.extend(_verify_domain(item, out, slide, refs))
    for ref, numbers in removed_refs.items():
        if numbers != refs_present[ref]:
            problems.append("PLACE_DISSOLVED_IN_SOME_DOMAINS_ONLY")
            break
    return tuple(dict.fromkeys(problems))


def _verify_domain(item, out, slide, refs) -> list:
    """Один домен: растворены только внутренние вершины `src:` цепей без интерфейса; итог — вход без них; хорда, сдвиг и излом в допусках."""

    found: list = []
    batch, after = item.batch, out.batch
    removed = set(out.removed)
    interface = {key.value for chain in batch.interface_chains for key in chain.ordered_vert_keys}
    position = {v.vert_key.value: (v.position.x, v.position.y, v.position.z) for v in batch.vertices}
    uv_of = {(face.face_id.value, fact.vert_key.value): (fact.uv.u, fact.uv.v) for face in batch.faces for fact in face.uv_facts}
    face_of = {key.value: face.face_id.value for face in batch.faces for key in face.ordered_vert_keys}
    inside, worst_chord, worst_slide, worst_bend = set(), 0.0, 0.0, 0.0
    for chain in batch.boundary_chains:
        if chain.semantic_boundary_id.value.split(":")[1] not in _CHAIN_KINDS:
            continue
        path = [key.value for key in chain.ordered_vert_keys]
        closed = _closed(path)
        cycle = path[:-1] if closed else path
        size = len(cycle)
        for at in range(size) if closed else range(1, size - 1):
            if cycle[at] in removed:
                inside.add(cycle[at])
                worst_bend = max(worst_bend, _bend(position[cycle[at - 1]], position[cycle[at]], position[cycle[(at + 1) % size]]))
        worst_chord, worst_slide = _verify_runs(cycle, closed, removed, position, uv_of, face_of, worst_chord, worst_slide)
    if inside != removed:
        found.append("DISSOLVED_VERTEX_IS_NOT_AN_INTERIOR_VERTEX_OF_A_SOURCE_OR_WALL_CHAIN")
    if any(not refs[key].startswith(SOURCE_PREFIX) or key in interface for key in removed):
        found.append("DISSOLVED_VERTEX_IS_NOT_A_SOURCE_VERTEX_OFF_THE_INTERFACE")
    recorded = dict(out.counters)
    if worst_bend > BEND_LIMIT:
        found.append("BEND_BEYOND_THE_LIMIT")
    if _nanometres(worst_chord) > recorded.get(MAX_CHORD_NM, 0) or not _within(worst_chord, CHORD_BUDGET):
        found.append("CHORD_DEPTH_BEYOND_RECORDED_MAXIMUM")
    if _milli_alpha(worst_slide) > recorded.get(MAX_UV_SLIDE_MILLI_ALPHA, 0) or not _within(worst_slide, slide):
        found.append("UV_SLIDE_BEYOND_RECORDED_MAXIMUM")
    if math.ceil(worst_bend * MICRODEGREES_PER_RADIAN) > recorded.get(MAX_BEND_MICRODEGREES, 0):
        found.append("BEND_BEYOND_RECORDED_MAXIMUM")
    found.extend(_verify_batch(item, out, removed))
    return found


def _verify_runs(cycle, closed, removed, position, uv_of, face_of, worst_chord, worst_slide):
    """Хорда и сдвиг UV растворённых вершин относительно ребра итога: по цепочке растворённых между двумя уцелевшими соседями."""

    size = len(cycle)
    kept = [at for at, key in enumerate(cycle) if key not in removed]
    for left, right in zip(kept, kept[1:] + ([kept[0] + size] if closed and kept else [])):
        run = [cycle[at % size] for at in range(left + 1, right)]
        if not run:
            continue
        low, high = sorted((cycle[left % size], cycle[right % size]))
        start, end = position[low], position[high]
        face = face_of[run[0]]
        for key in run:
            share, distance = _along(start, end, position[key])
            worst_chord = max(worst_chord, distance)
            if all((face, item) in uv_of for item in (low, high, key)):
                (u0, v0), (u1, v1), (u, v) = uv_of[(face, low)], uv_of[(face, high)], uv_of[(face, key)]
                worst_slide = max(worst_slide, math.hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share)))
            else:
                worst_slide = math.inf
    return worst_chord, worst_slide


def _verify_batch(item, out, removed) -> list:
    """Итог — вход без растворённых (вершины, кольца и UV граней, факты, цепи), проходит валидатор и аудит; числа батча пересчитаны."""

    found = []
    before, after = item.batch, out.batch
    if {v.vert_key.value for v in after.vertices} != {v.vert_key.value for v in before.vertices} - removed:
        found.append("VERTICES_ARE_NOT_THE_INPUT_WITHOUT_THE_DISSOLVED")
    if {f for f in after.station_facts} != {f for f in before.station_facts if f.vert_key.value not in removed}:
        found.append("STATION_FACTS_ARE_NOT_THE_INPUT_WITHOUT_THE_DISSOLVED")
    if len(after.faces) != len(before.faces) or any(
        [key.value for key in new.ordered_vert_keys] != [key.value for key in old.ordered_vert_keys if key.value not in removed]
        or [(fact.vert_key, fact.uv) for fact in new.uv_facts] != [(fact.vert_key, fact.uv) for fact in old.uv_facts if fact.vert_key.value not in removed]
        for new, old in zip(after.faces, before.faces)
    ):
        found.append("FACES_ARE_NOT_THE_INPUT_WITHOUT_THE_DISSOLVED")
    if after.interface_chains != before.interface_chains:
        found.append("INTERFACE_CHAINS_CHANGED")
    if {frozenset(edge) for chain in after.boundary_chains for edge in _bypassed(chain, set())} != {
        frozenset(edge) for chain in before.boundary_chains for edge in _bypassed(chain, removed)
    }:
        found.append("BOUNDARY_CHAINS_ARE_NOT_THE_INPUT_WITH_THE_DISSOLVED_BYPASSED")
    if out.content_digest != sha256(canonical_json_bytes(after)).hexdigest():
        found.append("CONTENT_DIGEST_IS_STALE")
    if validate_geometry_batch(after):
        found.append("BATCH_DOES_NOT_VALIDATE")
    audit = _audit(item, after, out.vertex_normals)
    if audit.problems():
        found.append("AUDIT:" + ",".join(audit.problems()))
    if out.overrides != (*batch_shape_counters(after), *audit.counters()):
        found.append("BATCH_NUMBERS_ARE_STALE")
    return found


def reconcile_source_dots(inputs, slide: Fraction) -> SourceDotsV1:
    """Закон `SILHOUETTE_SOURCE_DOTS_V1`: точки на прямых цепях источника и стены по всем доменам прогона разом, с независимой проверкой.

    `inputs` — `SourceDotInputV1` материализованных доменов; `slide` — допуск UV запроса (доля alpha). Порядок доменов на ответ не влияет.
    Проверка не прошла — ничего не растворено (`problems`, и в счётчиках каждого домена `SKIPPED_UNVERIFIED`): проход либо верен целиком, либо его нет.
    """

    inputs = tuple(inputs)
    run = _Pass(inputs, slide)
    run.run()
    counters = [run.counters(index) for index in range(len(inputs))]
    domains = tuple(_finished(item, state, counters[index]) for index, (item, state) in enumerate(zip(inputs, run.domains)))
    result = SourceDotsV1(domains, any(domain.removed for domain in domains))
    problems = verify_source_dots(inputs, result, slide) if result.changed else ()
    if problems:
        return _unchanged(inputs, [((SKIPPED_UNVERIFIED, 1),) for _ in inputs], problems)
    return result
