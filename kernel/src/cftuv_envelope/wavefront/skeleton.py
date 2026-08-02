"""Цикл событий волнового фронта: LAV, уровни, кластеры, цепи.

Структура заимствована как СПЕЦИФИКАЦИЯ у Kendzi (`akashskypatel/StraightSkeleton`,
BSD-3-Clause, C++, код не переносился) и совпадает с Felkel–Obdrzalek:

    while queue:
        level = queue.pop_level()        # всё, что на этом уровне
        clusters = cluster_by_point(...) # кластеры по совпадению точки
        for cluster: chain -> MultiEdge / MultiSplit

Одно отличие от всех трёх изученных реализаций, и оно единственное: и «то же
время», и «та же точка» решаются ТОЧНО на целочисленной решётке (`sqrt_sum`,
`event_time`), а не допуском `1e-10` и не квантованием к 1 мм.

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ две, и обе проверяются тестом на собственном
результате этой функции):

1. Время выдаваемых узлов НЕ УБЫВАЕТ. Это не пожелание, а определение очереди.
2. Число обработанных уровней не превосходит `level_budget(polygon)`. Бюджет —
   не эвристика: каждый уровень либо убивает хотя бы одну вершину фронта, либо
   разрезает reflex-вершину, а разрезов не больше, чем reflex-вершин, умноженных
   на число рёбер (наивный поиск). Выход за бюджет — ИМЕНОВАННЫЙ исход, а не
   `return`, как `bestt == 1e100` у Трики.

ПОИСК SPLIT-КАНДИДАТОВ. Способов два, и оба живые — второй существует потому,
что первый обязан быть чем-то проверен.

| способ | кандидатов при 2002 вершинах | зачем он есть |
|---|---:|---|
| `EXHAUSTIVE` | 1 998 000 (ровно `reflex * рёбра`) | эталон дифференциального теста |
| `MOTORCYCLE` | см. `split_candidates_examined` | рабочий путь |

`MOTORCYCLE` опирается на теорему 2.11 Huber'а (`motorcycle.py`): reflex-вершина
волны не уходит за свою трассу, поэтому расстояние от точки split до несущей
прямой рассекаемого ребра не больше времени крушения, и кандидаты берутся из
ячеек вокруг трассы. Наивная проверка «точка попала в текущий отрезок фронта»
остаётся на месте: она необходима, но НЕ достаточна — это измерено.

`EXHAUSTIVE` не «старый код, который забыли убрать»: оптимизация законна ровно
настолько, насколько даёт тот же ответ, а сравнивать не с чем, если эталон
удалить.

ВЫРОЖДЕННАЯ ТОЧКА. Разрезов у уровня на самом деле два вида, и второй до сих пор
разбирался первым — неверно. Кандидат, чья точка совпала с КОНЦОМ рассекаемого
отрезка фронта, означает, что вершина встретила не ребро, а другую ВЕРШИНУ.
Разрезом такая встреча даёт отрезок-близнец нулевой длины, тот остаётся жить в
LAV и уводит соседей по ложной траектории; на кресте из-за этого терялось
пересечение гребней. Теперь встреча разбирается пересоединением: все
встретившиеся вершины умирают, а их концы сшиваются заново по ЛУЧАМ — концы на
одном луче аннигилируют (схлопнувшийся по всей длине рукав), остаток сшивается в
единственную оставшуюся пару.

Оттуда же берётся третий вид вершины фронта. У стыка двух коллинеарных
СОНАПРАВЛЕННЫХ рёбер угол ровно развёрнутый, точки пересечения нет — но
биссектриса развёрнутого угла перпендикулярна сторонам, поэтому вершина идёт
поперёк общей прямой, не сдвигаясь вдоль неё. Она хранит свою проекцию вдоль
прямой (`_Vertex.sliding`), а место и времена ей считают `sliding_point` и
`sliding_time`, а не тройка прямых: тройке тут задан неверный вопрос, две её
прямые — одна и та же движущаяся прямая.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from fractions import Fraction
from functools import cmp_to_key
from math import gcd

from ..exact_sqrt_sum import _prime_universe_from_q_values
from .event_time import (
    ZERO_TIME,
    _event_point_with_prime_universe,
    EventPointV1,
    EventTimeV1,
    EventTimeOutcome,
    SupportLineV1,
    compare_times,
    concurrency_time,
    sliding_point,
    sliding_time,
)
from .events import (
    CandidateEventV1,
    EventKind,
    EventQueueV1,
    cluster_by_point,
    point_sort_key as _point_sort_key,
)
from .motorcycle import (
    MotorcycleGraphV1,
    TraceCandidateIndexV1,
    TraceV1,
    build_motorcycle_graph,
)
from .polygon import PolygonV1
from .sqrt_sum import SqrtSumV1


class SplitSearch(str, Enum):
    """Чем ищутся split-кандидаты. Оба пути обязаны давать один ответ."""

    MOTORCYCLE = "MOTORCYCLE"
    EXHAUSTIVE = "EXHAUSTIVE"


class SkeletonOutcome(str, Enum):
    """Чем кончилось построение. Тихого выхода нет ни одного."""

    EXACT = "EXACT"
    LEVEL_BUDGET_EXHAUSTED = "LEVEL_BUDGET_EXHAUSTED"
    MULTIWAY_SPLIT_UNPROVEN = "MULTIWAY_SPLIT_UNPROVEN"
    WAVEFRONT_LEFT_UNRESOLVED = "WAVEFRONT_LEFT_UNRESOLVED"


class CandidateRefusal(str, Enum):
    """Почему кандидат в события не родился. Две категории, и они РАЗНЫЕ.

    Отклонить кандидата — штатная работа фильтра, а не потеря: событие в
    прошлом фронта, точка вне текущего отрезка, кандидат за трассой (теорема
    2.11), тройка прямых, которая не сходится никогда. У всего этого правило
    ЕСТЬ, и правило говорит «события нет». Такие члены помечены `FILTER`.

    Тихая потеря — это когда правила нет вовсе: конфигурация встретилась, а
    алгоритм вместо названного исхода просто ничего не делает. Такие члены
    помечены `NO_RULE`, и каждый из них — открытый счёт, а не оговорка.
    Именно ради этого различения перечисление заведено: без него счётчик
    отказов утонул бы в штатном шуме и потеря в нём не была бы видна.
    """

    # -- FILTER: правило есть, и оно говорит «события нет» -----------------
    FILTER_SOLO_VERTEX = "FILTER_SOLO_VERTEX"
    FILTER_TRIPLE_NEVER_CONCURRENT = "FILTER_TRIPLE_NEVER_CONCURRENT"
    FILTER_EVENT_IN_THE_PAST = "FILTER_EVENT_IN_THE_PAST"
    FILTER_POINT_OUTSIDE_FRONT = "FILTER_POINT_OUTSIDE_FRONT"
    FILTER_BEYOND_TRACE = "FILTER_BEYOND_TRACE"
    FILTER_EDGE_IS_OWN = "FILTER_EDGE_IS_OWN"
    # Схлопываемый отрезок фронта к назначенному моменту НЕ сжался в точку:
    # его концы стоят на разных проекциях, и это доказано точным равенством, а
    # не оценено. Edge-событие означает исчезновение отрезка; отрезок
    # положительной длины исчезнуть не может, и «три прямые сошлись» этого не
    # заменяет — тройка отвечает про ПРЯМЫЕ, а гаснет ОТРЕЗОК.
    FILTER_SPAN_DOES_NOT_COLLAPSE = "FILTER_SPAN_DOES_NOT_COLLAPSE"
    # Схлопываемый отрезок фронта РОЖДЁН нулевым: событие назначено ровно на
    # момент рождения ОБОИХ его концов, и длина в этот момент равна нулю точно.
    # Положение вершины фронта аффинно по времени, значит и проекционная длина
    # отрезка аффинна; аффинная функция, равная нулю в начале своего
    # существования, положительной ДО него не была. Схлопываться такому отрезку
    # не из чего: он открывается, а не закрывается.
    #
    # Конфигурация рождается ровно веером вогнутой вершины: три его прямые
    # проходят через саму вершину, поэтому `concurrency_time` честно отвечает
    # `t = 0`, и без этого фильтра ребро веера гасло бы на нулевом уровне — то
    # есть мягкий угол молча превращался бы обратно в митрованный.
    FILTER_SPAN_IS_BORN_ZERO = "FILTER_SPAN_IS_BORN_ZERO"

    # -- NO_RULE: конфигурация без правила ---------------------------------
    # Тройка прямых сошлась НАВСЕГДА: определитель равен нулю тождественно.
    # Формула `t = -D0/S` тут не отвечает не потому, что события нет, а потому,
    # что вопрос ей задан неверный: две из трёх прямых — одна и та же движущаяся
    # прямая. Что при этом происходит с фронтом, формула не знает.
    NO_RULE_TRIPLE_ALWAYS_CONCURRENT = "NO_RULE_TRIPLE_ALWAYS_CONCURRENT"
    # Соседние рёбра вершины лежат на одной движущейся прямой и смотрят В РАЗНЫЕ
    # стороны: фронт локально вывернулся, два его отрезка совпали. У вершины нет
    # позиции — не «её трудно посчитать», а её нет.
    NO_RULE_JOINT_IS_ANTIPARALLEL = "NO_RULE_JOINT_IS_ANTIPARALLEL"
    # Соседние рёбра вершины лежат на одной движущейся прямой и смотрят В ОДНУ
    # сторону: угол ровно развёрнутый. Пересечения двух прямых нет, и позиция
    # вершины ВДОЛЬ прямой парой рёбер не определяется вовсе.
    NO_RULE_JOINT_IS_CODIRECTIONAL = "NO_RULE_JOINT_IS_CODIRECTIONAL"
    # То же коллинеарное сонаправленное соседство, но скорости РАЗНЫЕ: прямые
    # совпали на одно мгновение и уже расходятся. Развёрнутым углом это не
    # является, закреплённой проекции у такой вершины нет, и правила для неё в
    # модуле тоже нет. Рождается конфигурация только с приходом стен и весов:
    # при единичной скорости коллинеарность соседей влекла совпадение прямых.
    NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS = (
        "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS"
    )
    # Отрезок, который собирались резать, к моменту применения потерял концы.
    NO_RULE_SPAN_VANISHED = "NO_RULE_SPAN_VANISHED"
    # Вершины фронта встретились в одной точке, а сшить их концы однозначно
    # нечем: два конца одного рода легли на один луч (фронт сложился вдвое) либо
    # среди встретившихся оказались соседи по LAV. Кандидаты возвращаются
    # прежнему разбору — разрезу, — и это признанный долг, а не решение.
    NO_RULE_MEETING_NOT_RECONNECTABLE = "NO_RULE_MEETING_NOT_RECONNECTABLE"


class ProofObligationDisposition(str, Enum):
    """Что стало с явно названной, но ещё не доказанной ветвью."""

    OBSERVED = "OBSERVED"
    DISCHARGED_BY_PROVEN_SAME_TIME_EVENT = (
        "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT"
    )
    UNPROVEN_FALLBACK_APPLIED = "UNPROVEN_FALLBACK_APPLIED"
    EVENT_ACCEPTED_WITH_UNPROVEN_SPAN = "EVENT_ACCEPTED_WITH_UNPROVEN_SPAN"
    SURVIVED_PAST_EVENT_TIME = "SURVIVED_PAST_EVENT_TIME"
    UNSUPPORTED_EVENT_KIND_DROPPED = "UNSUPPORTED_EVENT_KIND_DROPPED"


class ProofStatus(str, Enum):
    COMPLETE = "COMPLETE"
    INCOMPLETE = "INCOMPLETE"


class ProofObligationBranch(str, Enum):
    EDGE_COLLAPSE_SPAN_UNPROVEN = "EDGE_COLLAPSE_SPAN_UNPROVEN"
    UNSUPPORTED_EVENT_KIND = "UNSUPPORTED_EVENT_KIND"


@dataclass(frozen=True, slots=True)
class ProofObligationV1:
    """Стабильное тождество одного непогашенного доказательного долга."""

    cause: CandidateRefusal | ProofObligationBranch
    disposition: ProofObligationDisposition
    vertex_ids: tuple[int, ...]
    participant_edge_keys: tuple[tuple[int, ...], ...]
    target_edge_keys: tuple[tuple[int, ...], ...]
    level: EventTimeV1
    event_kind: EventKind | None


_ObligationIdentity = tuple[
    tuple[int, ...],
    tuple[tuple[int, ...], ...],
    tuple[tuple[int, ...], ...],
]


_NO_RULE_DISPOSITIONS = {
    CandidateRefusal.NO_RULE_TRIPLE_ALWAYS_CONCURRENT: (
        ProofObligationDisposition.OBSERVED
    ),
    CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL: (
        ProofObligationDisposition.OBSERVED
    ),
    CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL: (
        ProofObligationDisposition.OBSERVED
    ),
    CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS: (
        ProofObligationDisposition.OBSERVED
    ),
    CandidateRefusal.NO_RULE_SPAN_VANISHED: (
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED
    ),
    CandidateRefusal.NO_RULE_MEETING_NOT_RECONNECTABLE: (
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED
    ),
}

_INCOMPLETE_DISPOSITIONS = frozenset(
    {
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED,
        ProofObligationDisposition.EVENT_ACCEPTED_WITH_UNPROVEN_SPAN,
        ProofObligationDisposition.SURVIVED_PAST_EVENT_TIME,
        ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED,
    }
)


def _proof_obligation_sort_key(obligation: ProofObligationV1) -> tuple:
    level = obligation.level.canonical()
    dividend = level.dividend
    divisor = tuple(
        (radicand, coefficient.numerator, coefficient.denominator)
        for radicand, coefficient in level.divisor.terms
    )
    return (
        dividend.numerator,
        dividend.denominator,
        divisor,
        obligation.cause.value,
        obligation.disposition.value,
        obligation.vertex_ids,
        obligation.participant_edge_keys,
        obligation.target_edge_keys,
        obligation.event_kind.value if obligation.event_kind is not None else "",
    )


def refusal_counter(reason: CandidateRefusal) -> str:
    """Имя счётчика отказов данного вида. Одна функция на весь репозиторий.

    Имя выводится из члена, а не пишется рядом с ним: иначе перечисление и
    отчёт можно было бы рассогласовать правкой одного из двух, и отказ, у
    которого счётчик назвали иначе, исчез бы из диагностики молча.
    """

    return f"refused_{reason.value.lower()}"


#: Счётчик на каждый член `CandidateRefusal`.
REFUSAL_COUNTERS = tuple(refusal_counter(member) for member in CandidateRefusal)


@dataclass(slots=True)
class _Edge:
    ident: int
    line: SupportLineV1
    #: Вхождение ИСХОДНОГО ребра входа: `(x0, y0, x1, y1)`, а у скрытой опоры
    #: веера — `(x, y, x, y, ordinal)`. У близнецов разреза оно то же, что у
    #: рассечённого: отрезки фронта — куски одного ребра, и грань у них одна.
    span: tuple[int, ...]

    @property
    def key(self) -> tuple[int, ...]:
        """Тождество УЧАСТНИКА события: вхождение ребра, а не несущая прямая.

        Различие не терминологическое, и оно оплачено. Ключом участника была
        `(a, b, c, q)` — несущая прямая с ненормированной нормалью. Такой ключ
        совпадал у двух коллинеарных сонаправленных рёбер ОДНОЙ длины
        (`holes_2`: рёбра соседних дыр, длина 8, два ключа из двенадцати
        сталкивались), узлы двух РАЗНЫХ граней сливались в один список, и
        `build_faces` отказывал до всякого счёта.

        Довод в пользу прямой — что биссектриса в straight skeleton равноудалена
        от несущих ПРЯМЫХ, а не от отрезков, — верен и не отменяется: вся
        арифметика времени и положения по-прежнему идёт через `self.line`.
        Прямая нужна ГЕОМЕТРИИ, вхождение — ТОЖДЕСТВУ. Роли разные, и раньше их
        исполнял один ключ.

        Ключ не зависит от порядка обхода контура: это координаты концов, а не
        номер ребра.
        """

        return self.span

    @property
    def line_key(self) -> tuple[int, int, int, int]:
        """Ключ несущей ПРЯМОЙ. Индекс кандидатов опрашивается по ней.

        Здесь совпадение у коллинеарных рёбер — не дефект, а смысл: теорема 2.11
        `motorcycle.py` фильтрует по расстоянию до прямой, и двум отрезкам на
        одной прямой полагается один вход в индексе.
        """

        return (self.line.a, self.line.b, self.line.c, self.line.q)


@dataclass(slots=True)
class _Vertex:
    ident: int
    prev_edge: int
    next_edge: int
    prev: int
    next: int
    birth: EventTimeV1
    point: EventPointV1
    reflex: bool
    alive: bool = True
    # Проекция ВДОЛЬ общей прямой соседних рёбер, если те коллинеарны и
    # сонаправлены. У такой вершины угол ровно развёрнутый, точки пересечения
    # двух прямых нет, а есть прямая плюс это сохраняющееся число: биссектриса
    # развёрнутого угла перпендикулярна сторонам, поэтому вершина скользит по
    # прямой, не сдвигаясь вдоль неё. `None` — обычная вершина либо стык
    # антипараллельный, у которого позиции нет вовсе.
    sliding: SqrtSumV1 | None = None


@dataclass(frozen=True, slots=True)
class SkeletonNodeV1:
    """Узел скелета: момент, место и ВСЕ участники сразу.

    `participants` — ключи ВХОЖДЕНИЙ исходных рёбер `(x0, y0, x1, y1)`,
    отсортированные. Три фронта, сошедшиеся в одной точке, дают ОДИН такой узел
    с тремя ключами, а не три попарных.

    Ключ здесь именно вхождение, а не несущая прямая: два коллинеарных
    сонаправленных ребра — два РАЗНЫХ участника, и грани у них две. См.
    `_Edge.key`.
    """

    kind: EventKind
    time: EventTimeV1
    point: EventPointV1
    participants: tuple[tuple[int, ...], ...]
    converging_vertices: int


@dataclass(frozen=True, slots=True)
class SkeletonV1:
    outcome: SkeletonOutcome
    nodes: tuple[SkeletonNodeV1, ...]
    levels: int
    counters: tuple[tuple[str, int], ...]
    proof_status: ProofStatus = ProofStatus.COMPLETE
    proof_obligations: tuple[ProofObligationV1, ...] = ()

    def counter(self, name: str) -> int:
        return dict(self.counters).get(name, 0)


def level_budget(polygon: PolygonV1) -> int:
    """Объявленная граница числа уровней. Превышение — именованный исход.

    Веер добавляет по одной вершине фронта и одному ребру на каждую скрытую
    опору, и каждая такая подвершина вогнута по построению. Без этой поправки
    бюджет считался бы по полигону БЕЗ веера, то есть был бы границей не той
    задачи, которая решается.
    """

    fan = polygon.fan_edge_count
    n = polygon.vertex_count + fan
    r = polygon.reflex_count + fan
    return 4 * n + 4 * n * r + 16


def build_skeleton(
    polygon: PolygonV1,
    *,
    split_search: SplitSearch = SplitSearch.MOTORCYCLE,
) -> SkeletonV1:
    """Скелет области как последовательность событий по возрастанию времени."""

    return _Builder(polygon, split_search).run()


class _Builder:
    """Носитель состояния цикла. Разбит на короткие этапы конвейера."""

    def __init__(
        self,
        polygon: PolygonV1,
        split_search: SplitSearch = SplitSearch.MOTORCYCLE,
    ) -> None:
        self.polygon = polygon
        self._prime_universe = _prime_universe_from_q_values(
            tuple(speed for _, _, speed in polygon.edges())
            + tuple(line.q for _, _, line in polygon.fan_edges())
        )
        self.split_search = split_search
        self.edges: list[_Edge] = []
        self.vertices: list[_Vertex] = []
        self.queue = EventQueueV1()
        self.edge_start: dict[int, int] = {}
        self.edge_end: dict[int, int] = {}
        self.nodes: list[SkeletonNodeV1] = []
        self.proof_obligations: list[ProofObligationV1] = []
        self.refusal: SkeletonOutcome | None = None
        # Трассы мотоциклов и двусторонний индекс. `None` — путь полного
        # перебора: эталон, с которым сверяется рабочий путь.
        self.graph: MotorcycleGraphV1 | None = None
        self.index: TraceCandidateIndexV1 | None = None
        self.traces: dict[int, TraceV1] = {}
        self.line_id: dict[tuple[int, int, int, int], int] = {}
        self.edges_by_line: dict[int, list[int]] = {}
        # Reflex-вершины, за которые индекс НЕ отвечает: их кандидаты всегда
        # перебираются полностью. Забыть их означало бы потерять события молча.
        self.unindexed_reflex: set[int] = set()
        # Вершины с развёрнутым углом. Индекс трасс за них не отвечает (трассы
        # определены на reflex-вершинах ВХОДА), а кандидаты им нужны: развёрнутая
        # вершина встречает встречный фронт ровно так же, как вогнутая.
        self.sliding_vertices: set[int] = set()
        # Подвершины вееров: у них биссектрисы ВХОДА нет, значит нет и трассы
        # motorcycle graph. Ограничиваются слабейшей стенной границей, как и
        # вершины, рождённые во время счёта.
        self.fan_vertices: set[int] = set()
        # Вершина входа `i` (нумерация motorcycle graph) — это вершина LAV
        # `origin_vertex[i]`. Совпадают они тождественно ровно тогда, когда
        # вееров нет; с веерами LAV длиннее, и без карты трасса уехала бы на
        # чужую вершину МОЛЧА.
        self.origin_vertex: dict[int, int] = {}
        self.origin_count = 0
        # Время уже снятого уровня. Кандидат раньше него — событие в прошлом
        # фронта, а не будущее: очередь его уже прошла. Без этой отсечки
        # ВЫДАВАЕМОЕ время переставало быть неубывающим на входах, где фронт
        # вывернулся, — то есть объявленная граница нарушалась собственным
        # результатом. Найдено тестом, а не рассуждением.
        self.now: EventTimeV1 = ZERO_TIME
        self.counters = {
            "edge_events": 0,
            "split_events": 0,
            "start_events": 0,
            "switch_events": 0,
            "multi_participant_nodes": 0,
            "discarded_stale_candidates": 0,
            "split_candidates_examined": 0,
            "split_candidates_beyond_trace": 0,
            "split_search_exhaustive_vertices": 0,
            "split_search_exhaustive_segments": 0,
            "coincident_split_targets": 0,
            "peaks": 0,
            "ridges": 0,
            # Встречи ВЕРШИН фронта, разобранные пересоединением. До среза они
            # молча обрабатывались как разрезы и рождали отрезки нулевой длины.
            "vertex_meeting_events": 0,
            "edge_collapse_span_unproven_but_accepted": 0,
            "unsupported_event_kind_dropped": 0,
        }
        self.counters.update({name: 0 for name in REFUSAL_COUNTERS})
        self._seed()

    def _refuse(
        self,
        reason: CandidateRefusal,
        *,
        vertex_ids: tuple[int, ...] = (),
        participant_edge_keys: tuple[tuple[int, ...], ...] = (),
        target_edge_keys: tuple[tuple[int, ...], ...] = (),
    ) -> None:
        """Отказ кандидата ПОД ИМЕНЕМ. Ни один `return` не проходит мимо.

        Возвращает `None`, чтобы место отказа читалось одной строкой
        `return self._refuse(...)`, и чтобы забыть счётчик было труднее, чем
        его поставить.
        """

        self.counters[refusal_counter(reason)] += 1
        disposition = _NO_RULE_DISPOSITIONS.get(reason)
        if disposition is not None:
            self._record_obligation(
                cause=reason,
                disposition=disposition,
                vertex_ids=vertex_ids,
                participant_edge_keys=participant_edge_keys,
                target_edge_keys=target_edge_keys,
                level=self.now,
            )
        return None

    def _record_obligation(
        self,
        *,
        cause: CandidateRefusal | ProofObligationBranch,
        disposition: ProofObligationDisposition,
        vertex_ids: tuple[int, ...] = (),
        participant_edge_keys: tuple[tuple[int, ...], ...] = (),
        target_edge_keys: tuple[tuple[int, ...], ...] = (),
        level: EventTimeV1,
        event_kind: EventKind | None = None,
    ) -> None:
        """Нормализовать identity, не теряя кратность самих obligations."""

        self.proof_obligations.append(
            ProofObligationV1(
                cause=cause,
                disposition=disposition,
                vertex_ids=tuple(sorted(set(vertex_ids))),
                participant_edge_keys=tuple(
                    sorted(set(participant_edge_keys))
                ),
                target_edge_keys=tuple(sorted(set(target_edge_keys))),
                level=level.canonical(),
                event_kind=event_kind,
            )
        )

    def _edge_keys(self, *edge_ids: int) -> tuple[tuple[int, ...], ...]:
        return tuple(
            self.edges[ident].key
            for ident in edge_ids
            if 0 <= ident < len(self.edges)
        )

    def _proof_edge_endpoint_ids(self, edge_id: int) -> tuple[int, ...]:
        """Runtime ID концов без live-фильтра, только для proof identity."""

        return tuple(
            ident
            for ident in (
                self.edge_start.get(edge_id),
                self.edge_end.get(edge_id),
            )
            if ident is not None and 0 <= ident < len(self.vertices)
        )

    def _edge_obligation_identity(
        self, vertex: _Vertex, peer: _Vertex
    ) -> _ObligationIdentity:
        shared = vertex.next_edge
        return (
            (vertex.ident, peer.ident),
            self._edge_keys(vertex.prev_edge, shared, peer.next_edge),
            self._edge_keys(shared),
        )

    def _split_obligation_identity(
        self, vertex: _Vertex, edge: _Edge
    ) -> _ObligationIdentity:
        endpoints = self._proof_edge_endpoint_ids(edge.ident)
        return (
            (vertex.ident, *endpoints),
            self._edge_keys(vertex.prev_edge, vertex.next_edge, edge.ident),
            self._edge_keys(edge.ident),
        )

    def counter_of(self, reason: CandidateRefusal) -> int:
        """Сколько раз отказ этого вида случился. Для стендов и отчётов."""

        return self.counters[refusal_counter(reason)]

    # ---- инициализация ---------------------------------------------------

    def _seed(self) -> None:
        """`InitSlav` на каждый контур, включая каждую дыру."""

        self._seed_loops()
        self._seed_traces()
        for vertex in self.vertices:
            self._enqueue_for(vertex)

    def _seed_loops(self) -> None:
        for loop in self.polygon.loops:
            self._seed_one_loop(loop)
        for edge in self.edges:
            self._register_line(edge)

    def _seed_one_loop(self, loop) -> None:
        """LAV одного контура: его рёбра, его вееры и вершины между ними.

        ВЕЕР ВСТАВЛЯЕТСЯ ЗДЕСЬ И БОЛЬШЕ НИГДЕ. Вершина с веером из `k` опор
        рождает не одну вершину фронта, а `k + 1`: все в ОДНОЙ точке, все с
        нулевой длиной разделяющих их рёбер. Дальше эти рёбра ничем не
        отличаются от обычных движущихся — ни одного правила событий под них не
        заведено, и это проверяемое утверждение, а не намерение.

        Вогнутость подвершины веера считается ПО ПРЯМЫМ (`_is_reflex`), а не по
        тройке точек контура: точки у всех `k + 1` подвершин одинаковы, и
        `orient2d` на них ответил бы нулём для каждой. У вершины без веера
        по-прежнему берётся `reflex_flags()` — то же самое число, посчитанное
        целочисленно, и путь этот остаётся побитово прежним.
        """

        points = loop.points
        reflex = loop.reflex_flags()
        speeds = loop.edge_speeds_squared
        size = len(points)
        first_edge = len(self.edges)
        for index in range(size):
            start = points[index]
            end = points[(index + 1) % size]
            # Скорость берётся из петли, а не из `through`: иначе стена
            # молча стала бы источником единичной скорости.
            self.edges.append(
                _Edge(
                    first_edge + index,
                    SupportLineV1.with_speed(start, end, speeds[index]),
                    (start[0], start[1], end[0], end[1]),
                )
            )
        first_vertex = len(self.vertices)
        stops: list[tuple[int, int, tuple[int, int], bool | None]] = []
        for index in range(size):
            chain = self._seed_fan_edges(points[index])
            ring = (first_edge + (index - 1) % size, *chain, first_edge + index)
            for prev_edge, next_edge in zip(ring, ring[1:]):
                stops.append(
                    (
                        prev_edge,
                        next_edge,
                        points[index],
                        None if chain else reflex[index],
                    )
                )
            if not chain:
                # Нумерация motorcycle graph идёт по вершинам ВХОДА, а LAV с
                # веерами их размножает. Соответствие пишется здесь и только
                # для вершин БЕЗ веера: у вершины с веером биссектрисы входа
                # больше не существует, значит и трасса её не про неё.
                self.origin_vertex[self.origin_count] = first_vertex + len(stops) - 1
            self.origin_count += 1
        total = len(stops)
        for offset, (prev_edge, next_edge, point, corner) in enumerate(stops):
            vertex = _Vertex(
                ident=first_vertex + offset,
                prev_edge=prev_edge,
                next_edge=next_edge,
                prev=first_vertex + (offset - 1) % total,
                next=first_vertex + (offset + 1) % total,
                birth=ZERO_TIME,
                point=EventPointV1(
                    SqrtSumV1.rational(point[0]),
                    SqrtSumV1.rational(point[1]),
                ),
                reflex=(
                    _is_reflex(
                        self.edges[prev_edge].line, self.edges[next_edge].line
                    )
                    if corner is None
                    else corner
                ),
            )
            self._classify_sliding(vertex)
            self.vertices.append(vertex)
            self._register(vertex)
            if corner is None:
                self.fan_vertices.add(vertex.ident)

    def _seed_fan_edges(self, point: tuple[int, int]) -> tuple[int, ...]:
        """Рёбра НУЛЕВОЙ ДЛИНЫ веера этой вершины, в порядке входа.

        Вхождение такого ребра — вырожденный отрезок `(x, y, x, y)` плюс ординал
        опоры. Ординал в ключе обязателен: без него `k` опор одной вершины были
        бы ОДНИМ участником события и ОДНОЙ гранью на всех, а грани у них `k`.
        """

        fan = self.polygon.fan_at(point)
        if fan is None:
            return ()
        idents: list[int] = []
        for ordinal, support in enumerate(fan.supports, start=1):
            ident = len(self.edges)
            self.edges.append(
                _Edge(
                    ident,
                    SupportLineV1(
                        support.normal_x,
                        support.normal_y,
                        support.constant_at(point),
                        support.speed_squared,
                    ),
                    (point[0], point[1], point[0], point[1], ordinal),
                )
            )
            idents.append(ident)
        return tuple(idents)

    def _register_line(self, edge: _Edge) -> int:
        """Ребро под свою НЕСУЩУЮ ПРЯМУЮ. Близнецы разреза делят одну прямую.

        Индекс опрашивается по прямым, а не по рёбрам: рассечённое ребро
        размножается в счёте на несколько отрезков фронта с одной и той же
        несущей прямой, и разметка от этого не меняется — меняется только
        список владельцев. Здесь берётся `line_key`, а НЕ `key`: под одной
        прямой законно стоят и близнецы разреза, и два разных коллинеарных
        ребра входа, потому что фильтровать кандидатов по расстоянию до прямой
        им положено сообща.
        """

        ident = self.line_id.get(edge.line_key)
        if ident is None:
            ident = len(self.line_id)
            self.line_id[edge.line_key] = ident
            self.edges_by_line[ident] = []
        self.edges_by_line[ident].append(edge.ident)
        return ident

    def _seed_traces(self) -> None:
        """Motorcycle graph по входу и двусторонний индекс по его трассам."""

        if self.split_search is not SplitSearch.MOTORCYCLE:
            return
        self.graph = build_motorcycle_graph(self.polygon)
        self.index = TraceCandidateIndexV1.covering(self.polygon, self.graph)
        for line_key, ident in self.line_id.items():
            self.index.register_line(
                ident, SupportLineV1(*line_key)
            )
        for origin, ident in sorted(self.origin_vertex.items()):
            vertex = self.vertices[ident]
            if not vertex.reflex:
                continue
            self._adopt_trace(vertex, self.graph.traces.get(origin))
        for ident in sorted(self.fan_vertices):
            vertex = self.vertices[ident]
            if not vertex.reflex:
                continue
            # У подвершины веера трассы входа нет: motorcycle graph строит их по
            # биссектрисе между соседними рёбрами ВХОДА, а здесь одно из двух
            # рёбер — скрытая опора. Берётся та же слабейшая стенная граница,
            # что и у вершин, рождённых во время счёта: волна не выходит за
            # границу области, и это верно без всякой теоремы.
            self._adopt_trace(
                vertex,
                self.graph.trace_for(
                    self.edges[vertex.prev_edge].line,
                    self.edges[vertex.next_edge].line,
                    vertex.birth,
                    vertex.point,
                ),
            )

    def _adopt_trace(self, vertex: _Vertex, trace: TraceV1 | None) -> None:
        """Принять трассу вершины либо честно объявить её неиндексируемой."""

        if trace is None or trace.crash_time is None:
            self.unindexed_reflex.add(vertex.ident)
            return
        self.traces[vertex.ident] = trace
        if not self.index.register_trace(vertex.ident, trace):
            self.unindexed_reflex.add(vertex.ident)

    # ---- порождение кандидатов ------------------------------------------

    def _enqueue_for(self, vertex: _Vertex) -> None:
        self._enqueue_edge_event(vertex)
        if vertex.reflex or vertex.sliding is not None:
            self._enqueue_split_events(vertex)

    def _edge_event_time(
        self, vertex: _Vertex, peer: _Vertex
    ) -> tuple[EventTimeV1 | None, EventTimeOutcome]:
        """Момент встречи двух соседей фронта. Три случая, и они РАЗНЫЕ.

        Обычная пара — тройка прямых (`prev` вершины, общее ребро, `next`
        соседа) сходится в точке, и это `concurrency_time`.

        Если один из двух — вершина с развёрнутым углом, тройка вырождается:
        две её прямые оказываются ОДНОЙ движущейся прямой, определитель равен
        нулю тождественно, и формула отвечает «сошлись навсегда» вместо времени.
        Вопрос ей задан неверный. Верный вопрос — когда обычный сосед доедет до
        закреплённой проекции скользящего, и на него отвечает `sliding_time`.

        Если развёрнуты ОБА, они стоят на одной прямой и идут вместе: встреча
        либо была всегда (проекции совпали), либо не будет никогда. Оба ответа
        названы, и ни один не молчит.
        """

        if vertex.sliding is not None and peer.sliding is not None:
            here = self._position(vertex, self.now)
            there = self._position(peer, self.now)
            if (here.x - there.x).is_zero and (here.y - there.y).is_zero:
                # Оба уже в одной точке и идут вместе: отрезок между ними имеет
                # нулевую длину, и событие не «когда-нибудь», а СЕЙЧАС.
                return self.now, EventTimeOutcome.EXACT
            return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
        if vertex.sliding is not None:
            return sliding_time(
                self.edges[vertex.prev_edge].line,
                vertex.sliding,
                self.edges[peer.next_edge].line,
            )
        if peer.sliding is not None:
            return sliding_time(
                self.edges[peer.prev_edge].line,
                peer.sliding,
                self.edges[vertex.prev_edge].line,
            )
        return concurrency_time(
            self.edges[vertex.prev_edge].line,
            self.edges[vertex.next_edge].line,
            self.edges[peer.next_edge].line,
        )

    def _enqueue_edge_event(self, vertex: _Vertex) -> None:
        peer = self.vertices[vertex.next]
        if peer.ident == vertex.ident:
            return self._refuse(CandidateRefusal.FILTER_SOLO_VERTEX)
        time, outcome = self._edge_event_time(vertex, peer)
        if outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT:
            return self._refuse(CandidateRefusal.FILTER_TRIPLE_NEVER_CONCURRENT)
        if outcome is not EventTimeOutcome.EXACT or time is None:
            identity = self._edge_obligation_identity(vertex, peer)
            return self._refuse(
                CandidateRefusal.NO_RULE_TRIPLE_ALWAYS_CONCURRENT,
                vertex_ids=identity[0],
                participant_edge_keys=identity[1],
                target_edge_keys=identity[2],
            )
        if not self._is_future(time, vertex, peer):
            return self._refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
        # ФРОНТ НЕ ГАСИТ ТОГО, ЧЕГО НЕ КАСАЕТСЯ ПО МЕРЕ. `concurrency_time`
        # отвечает про три ПРЯМЫЕ, а гаснет ОТРЕЗОК, и это не одно и то же:
        # когда движущаяся прямая источника наезжает на прямую коллинеарной
        # СТЕНЫ, тройка честно сходится, хотя отрезки перекрываются в одной
        # точке и стена не сжимается ни на шаг. Отрезок положительной длины
        # исчезнуть не может, поэтому проверяется он, а не тройка.
        span = self._collapsing_span(vertex, peer, time)
        if span is not None and not span.is_zero:
            return self._refuse(CandidateRefusal.FILTER_SPAN_DOES_NOT_COLLAPSE)
        if (
            span is not None
            and compare_times(time, vertex.birth) == 0
            and compare_times(time, peer.birth) == 0
        ):
            # Отрезок нулевой в момент рождения обоих концов: он ОТКРЫВАЕТСЯ.
            return self._refuse(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
        point = self._vertex_position(vertex, time, peer=peer)
        if point is None:
            return
        self.queue.push(
            CandidateEventV1(
                EventKind.EDGE,
                time,
                point,
                vertex.ident,
                peer.ident,
                -1,
                span_unproven=span is None,
            )
        )

    def _enqueue_split_events(self, vertex: _Vertex) -> None:
        """Кандидаты с трассы, если индекс за вершину отвечает; иначе перебор."""

        if (
            vertex.sliding is None
            and self.index is not None
            and self.index.knows_vertex(vertex.ident)
        ):
            self._enqueue_splits_from_trace(vertex)
            return
        self.counters["split_search_exhaustive_vertices"] += 1
        for edge in self.edges:
            self._try_split(vertex, edge)

    def _enqueue_splits_from_trace(self, vertex: _Vertex) -> None:
        """Только рёбра тех прямых, что заходят в рамку трассы этой вершины."""

        for line_ident in self.index.lines_near(vertex.ident):
            for edge_ident in self.edges_by_line[line_ident]:
                self._try_split(vertex, self.edges[edge_ident])

    def _try_split(self, vertex: _Vertex, edge: _Edge) -> None:
        if edge.ident in (vertex.prev_edge, vertex.next_edge):
            return self._refuse(CandidateRefusal.FILTER_EDGE_IS_OWN)
        self.counters["split_candidates_examined"] += 1
        candidate = self._split_candidate(vertex, edge)
        if candidate is not None:
            self.queue.push(candidate)

    def _split_candidate(
        self, vertex: _Vertex, edge: _Edge
    ) -> CandidateEventV1 | None:
        if vertex.sliding is None:
            time, outcome = concurrency_time(
                self.edges[vertex.prev_edge].line,
                self.edges[vertex.next_edge].line,
                edge.line,
            )
        else:
            # У развёрнутой вершины двух прямых нет — есть одна и закреплённая
            # проекция вдоль неё. Тройка тут вырождена, и спрашивать её незачем.
            time, outcome = sliding_time(
                self.edges[vertex.prev_edge].line, vertex.sliding, edge.line
            )
        if outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT:
            return self._refuse(CandidateRefusal.FILTER_TRIPLE_NEVER_CONCURRENT)
        if outcome is not EventTimeOutcome.EXACT or time is None:
            identity = self._split_obligation_identity(vertex, edge)
            return self._refuse(
                CandidateRefusal.NO_RULE_TRIPLE_ALWAYS_CONCURRENT,
                vertex_ids=identity[0],
                participant_edge_keys=identity[1],
                target_edge_keys=identity[2],
            )
        if time.sign <= 0 or compare_times(time, vertex.birth) <= 0:
            return self._refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
        if compare_times(time, self.now) < 0:
            return self._refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
        # ТЕОРЕМА 2.11: вершина не уходит за свою трассу, значит событие позже
        # крушения невозможно. Это и есть достаточное условие, которого наивной
        # проверке попадания в отрезок не хватало.
        trace = self.traces.get(vertex.ident)
        if trace is not None and not trace.bounds_time(time):
            self.counters["split_candidates_beyond_trace"] += 1
            return self._refuse(CandidateRefusal.FILTER_BEYOND_TRACE)
        point = self._vertex_position(vertex, time, target_edge=edge)
        if point is None:
            return None
        if not self._edge_span_contains(edge, point, time):
            return self._refuse(CandidateRefusal.FILTER_POINT_OUTSIDE_FRONT)
        return CandidateEventV1(
            EventKind.SPLIT, time, point, vertex.ident, -1, edge.ident
        )

    def _is_future(
        self, time: EventTimeV1, vertex: _Vertex, peer: _Vertex
    ) -> bool:
        if time.sign < 0:
            return False
        return (
            compare_times(time, vertex.birth) >= 0
            and compare_times(time, peer.birth) >= 0
            and compare_times(time, self.now) >= 0
        )

    def _position(
        self, vertex: _Vertex, time: EventTimeV1
    ) -> EventPointV1 | None:
        """Место вершины в момент `time`. Без счётчика — для внутренних опросов.

        Параллельность соседних прямых — не «трудный случай», а ДВА разных
        вырождения, и путать их нельзя.

        Сонаправленный стык: угол развёрнутый, вершина скользит по общей прямой
        с сохраняющейся проекцией вдоль неё — место есть, и оно точное.
        Антипараллельный: фронт вывернулся, два его отрезка совпали — места нет
        не потому, что его трудно посчитать, а потому, что его нет.
        """

        first = self.edges[vertex.prev_edge].line
        second = self.edges[vertex.next_edge].line
        if first.a * second.b - second.a * first.b:
            return _event_point_with_prime_universe(
                first,
                second,
                time,
                self._prime_universe,
            )
        if vertex.sliding is None:
            return None
        return sliding_point(first, vertex.sliding, time)

    def _vertex_position(
        self,
        vertex: _Vertex,
        time: EventTimeV1,
        *,
        peer: _Vertex | None = None,
        target_edge: _Edge | None = None,
    ) -> EventPointV1 | None:
        """То же место, но с ИМЕНОВАННЫМ отказом, когда места нет.

        Разделение на две функции не косметика: счётчик обязан считать
        конфигурации, встреченные ЦИКЛОМ СОБЫТИЙ, а не опросы, которыми стенд и
        внутренние проверки перебирают вершины. Иначе число отказов зависело бы
        от того, сколько раз мы посмотрели, а не от того, что случилось.
        """

        place = self._position(vertex, time)
        if place is None:
            if peer is not None:
                identity = self._edge_obligation_identity(vertex, peer)
            elif target_edge is not None:
                identity = self._split_obligation_identity(vertex, target_edge)
            else:
                identity = (
                    (vertex.ident,),
                    self._edge_keys(vertex.prev_edge, vertex.next_edge),
                    (),
                )
            self._refuse(
                _joint_kind(
                    self.edges[vertex.prev_edge].line,
                    self.edges[vertex.next_edge].line,
                ),
                vertex_ids=identity[0],
                participant_edge_keys=identity[1],
                target_edge_keys=identity[2],
            )
        return place

    # ---- геометрические проверки ----------------------------------------

    def _edge_span_contains(
        self, edge: _Edge, point: EventPointV1, time: EventTimeV1
    ) -> bool:
        """Точка лежит внутри ТЕКУЩЕГО отрезка фронта этого ребра.

        Проекция на направление ребра — целочисленная линейная форма, поэтому
        сравнение остаётся точным знаком `SqrtSumV1` и порога не требует.
        """

        start = self._edge_start_vertex(edge.ident)
        end = self._edge_end_vertex(edge.ident)
        if start is None or end is None:
            return False
        here = _project(edge.line, point)
        low = self._span_end(start, edge, time, at_start=True)
        high = self._span_end(end, edge, time, at_start=False)
        if low is not None and (here - low).sign() < 0:
            return False
        if high is not None and (high - here).sign() < 0:
            return False
        return True

    def _span_end(
        self,
        vertex: _Vertex,
        edge: _Edge,
        time: EventTimeV1,
        *,
        at_start: bool,
    ) -> SqrtSumV1 | None:
        """Проекция конца отрезка фронта, либо `None`, если конца нет.

        Конец отрезка — это сама вершина, поэтому спрашивается её место, а не
        пересечение двух прямых заново: для обычной вершины это одно и то же
        число, а для скользящей пересечения не существует и второй способ
        отвечал бы «граница отсутствует» там, где она есть. Отсутствует она
        ровно у антипараллельного стыка — у вывернувшегося фронта.

        У НЕПОДВИЖНОГО ребра (`q = 0`) есть второй ответ, и он точный.
        Отрезок фронта такого ребра лежит на прямой, которая не двигается
        вовсе, поэтому он есть ПОДОТРЕЗОК самого ребра при любом `t`: расти ему
        некуда, у него нет собственной скорости. Значит даже когда вершина на
        конце позиции не имеет, конец отрезка ограничен концом РЕБРА, и `None`
        там означал бы «границы нет» — то есть неограниченный отрезок фронта у
        ребра, которое не сдвинулось ни на шаг.

        Различие «неподвижное против движущегося» здесь не подгонка: у
        движущегося ребра отрезок фронта РАСТЁТ за пределы своего ребра всякий
        раз, когда рядом стоит вогнутая вершина, и подставить ему концы входа
        было бы неверно. У неподвижного расти нечему — прямая та же самая.

        `at_start` говорит, КОТОРЫЙ из двух концов ребра берётся: проекция
        вдоль направления `(b, -a)` монотонно растёт от начала ребра к концу
        (разность равна `|d|^2 > 0`), поэтому начало отрезка — это начало
        ребра, а конец — конец.
        """

        place = self._position(vertex, time)
        if place is not None:
            return _project(edge.line, place)
        if not edge.line.is_stationary:
            return None
        x0, y0, x1, y1 = edge.span
        node_x, node_y = (x0, y0) if at_start else (x1, y1)
        return SqrtSumV1.rational(
            node_x * edge.line.b - node_y * edge.line.a
        )

    def _collapsing_span(
        self, vertex: _Vertex, peer: _Vertex, time: EventTimeV1
    ) -> SqrtSumV1 | None:
        """Длина схлопываемого отрезка в проекции, либо `None`, если её нет.

        Edge-событие означает ровно одно: отрезок фронта между двумя соседними
        вершинами СЖАЛСЯ В ТОЧКУ. Проекция на направление ребра монотонна вдоль
        него, поэтому «сжался в точку» — это точное равенство двух проекций, а
        не «стало мало». Обе вершины лежат на прямой этого ребра, поэтому
        равенство проекций влечёт совпадение точек.

        `None` означает, что длину доказать нечем: у конца нет позиции, и ребро
        при этом движется. Такой случай НЕ отвергается — отсутствие
        доказательства не есть доказательство отсутствия.
        """

        edge = self.edges[vertex.next_edge]
        low = self._span_end(vertex, edge, time, at_start=True)
        high = self._span_end(peer, edge, time, at_start=False)
        if low is None or high is None:
            return None
        return high - low

    def _register(self, vertex: _Vertex) -> None:
        self.edge_start[vertex.next_edge] = vertex.ident
        self.edge_end[vertex.prev_edge] = vertex.ident

    def _edge_start_vertex(self, edge_id: int) -> _Vertex | None:
        """Живая вершина, с которой начинается фронт этого ребра, либо None.

        Запись в словаре может устареть: ребро исчезает вместе со схлопнутой
        цепью. Поэтому найденное сверяется с текущим состоянием, а не берётся
        на веру.
        """

        ident = self.edge_start.get(edge_id)
        if ident is None:
            return None
        vertex = self.vertices[ident]
        return vertex if vertex.alive and vertex.next_edge == edge_id else None

    def _edge_end_vertex(self, edge_id: int) -> _Vertex | None:
        ident = self.edge_end.get(edge_id)
        if ident is None:
            return None
        vertex = self.vertices[ident]
        return vertex if vertex.alive and vertex.prev_edge == edge_id else None

    # ---- цикл ------------------------------------------------------------

    def run(self) -> SkeletonV1:
        budget = level_budget(self.polygon)
        levels = 0
        while len(self.queue):
            if levels >= budget:
                return self._finish(SkeletonOutcome.LEVEL_BUDGET_EXHAUSTED, levels)
            level = self.queue.pop_level()
            self.now = level[0].time
            levels += 1
            self._apply_level(level)
            if self.refusal is not None:
                return self._finish(self.refusal, levels)
            self._close_short_lavs()
            self._discharge_observed_obligations()
        outcome = (
            SkeletonOutcome.EXACT
            if not any(vertex.alive for vertex in self.vertices)
            else SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
        )
        return self._finish(outcome, levels)

    def _finish(self, outcome: SkeletonOutcome, levels: int) -> SkeletonV1:
        self._discharge_observed_obligations()
        self.proof_obligations = [
            replace(
                obligation,
                disposition=ProofObligationDisposition.SURVIVED_PAST_EVENT_TIME,
            )
            if obligation.disposition is ProofObligationDisposition.OBSERVED
            else obligation
            for obligation in self.proof_obligations
        ]
        obligations = tuple(
            sorted(self.proof_obligations, key=_proof_obligation_sort_key)
        )
        proof_status = (
            ProofStatus.INCOMPLETE
            if any(
                obligation.disposition in _INCOMPLETE_DISPOSITIONS
                for obligation in obligations
            )
            else ProofStatus.COMPLETE
        )
        counters = dict(self.counters)
        if self.graph is not None:
            # Цена графа входит в отчёт. Иначе выигрыш в кандидатах мерился бы
            # против базы, у которой этой статьи расхода просто нет.
            counters.update(self.graph.counters)
        return SkeletonV1(
            outcome=outcome,
            nodes=tuple(self.nodes),
            levels=levels,
            counters=tuple(sorted(counters.items())),
            proof_status=proof_status,
            proof_obligations=obligations,
        )

    def _discharge_observed_obligations(self) -> None:
        """Погасить долг только смертью всех названных вершин этого уровня."""

        self.proof_obligations = [
            replace(
                obligation,
                disposition=(
                    ProofObligationDisposition.DISCHARGED_BY_PROVEN_SAME_TIME_EVENT
                ),
            )
            if (
                obligation.disposition is ProofObligationDisposition.OBSERVED
                and obligation.vertex_ids
                and all(
                    0 <= ident < len(self.vertices)
                    and not self.vertices[ident].alive
                    for ident in obligation.vertex_ids
                )
            )
            else obligation
            for obligation in self.proof_obligations
        ]

    def _apply_level(self, level: tuple[CandidateEventV1, ...]) -> None:
        """Весь уровень разом: сначала ВСЕ разрезы, потом ВСЕ схлопывания.

        Живость всех кандидатов уровня определяется ДО первого применения.
        Иначе порядок применения решал бы, какие события уровня выживут, — то
        есть ровно тот дефект, ради которого очередь и строится: последовательный
        попарный крой не композируется.

        Ни одно снятое событие не теряется молча: не прошедшее проверку идёт в
        счётчик устаревших, а неразрешимая одновременность — в именованный
        исход, а не в выбор «первого попавшегося».
        """

        supported = {EventKind.SPLIT, EventKind.EDGE}
        for event in level:
            if event.kind in supported:
                continue
            self.counters["unsupported_event_kind_dropped"] += 1
            valid_vertices = tuple(
                ident
                for ident in (event.vertex, event.peer)
                if 0 <= ident < len(self.vertices)
            )
            carried_edge = self._edge_keys(event.edge)
            self._record_obligation(
                cause=ProofObligationBranch.UNSUPPORTED_EVENT_KIND,
                disposition=(
                    ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED
                ),
                vertex_ids=valid_vertices,
                participant_edge_keys=carried_edge,
                target_edge_keys=carried_edge,
                level=event.time,
                event_kind=event.kind,
            )

        splits = [event for event in level if event.kind is EventKind.SPLIT]
        collapses = [event for event in level if event.kind is EventKind.EDGE]
        live_splits = [event for event in splits if self._split_is_live(event)]
        self.counters["discarded_stale_candidates"] += len(splits) - len(
            live_splits
        )
        meetings, cuts = self._separate_vertex_meetings(live_splits)
        cuts += self._apply_vertex_meetings(meetings)
        cuts = [event for event in cuts if self._split_is_live(event)]
        separated = self._dedupe_by_vertex(cuts)
        if separated is None:
            return
        for edge_id, group in self._group_splits(separated):
            self._apply_multi_split(edge_id, group)

        live_collapses = [
            event for event in collapses if self._edge_event_is_live(event)
        ]
        self.counters["discarded_stale_candidates"] += len(collapses) - len(
            live_collapses
        )
        for cluster in cluster_by_point(tuple(live_collapses)):
            self._apply_multi_edge(list(cluster))

    def _separate_vertex_meetings(
        self, splits: list[CandidateEventV1]
    ) -> tuple[dict, list[CandidateEventV1]]:
        """Отделить встречи ВЕРШИН от настоящих разрезов.

        Кандидат, чья точка совпала с концом рассекаемого отрезка фронта, — не
        разрез: вершина встретила не ребро, а другую вершину. Разрезом такая
        встреча обрабатывается неверно, и неверность видна числом — рождается
        отрезок-близнец нулевой длины, он остаётся жить в LAV и уводит соседей
        по ложной траектории. Именно так у креста терялось пересечение гребней.

        Спрашивается ЗДЕСЬ, до первого применения, и это не вкусовщина: разрез,
        применённый раньше, переписывает концы отрезка соседа, и та же вершина
        со второго взгляда уже не видна — на кресте видно 2 встречи вместо 4.

        Соседство по LAV пересоединением не описывается — там между вершинами
        схлопывается ребро, и сосед не остаётся, а умирает. Такой кандидат идёт
        ПРЕЖНИМ путём, разрезом, и считается под именем; отбросить его нельзя,
        это измерено (см. `_apply_vertex_meetings`).
        """

        meetings: dict[tuple, dict] = {}
        cuts: list[CandidateEventV1] = []
        for event in splits:
            met, adjacent = self._front_vertex_met_by(event)
            if met is None:
                cuts.append(event)
                continue
            if adjacent:
                # Встреченная вершина — сосед по LAV. Пересоединение её не
                # описывает (у пересоединения сосед считается остающимся, а он
                # умирает), поэтому кандидат идёт прежним путём — разрезом.
                # Отбрасывать его нельзя: измерено, что на фигурах общего
                # положения это теряет до шести настоящих событий из восьми.
                vertex = self.vertices[event.vertex]
                self._refuse(
                    CandidateRefusal.NO_RULE_MEETING_NOT_RECONNECTABLE,
                    vertex_ids=(
                        vertex.ident,
                        met.ident,
                        *self._proof_edge_endpoint_ids(event.edge),
                    ),
                    participant_edge_keys=self._edge_keys(
                        vertex.prev_edge,
                        vertex.next_edge,
                        met.prev_edge,
                        met.next_edge,
                        event.edge,
                    ),
                    target_edge_keys=self._edge_keys(event.edge),
                )
                cuts.append(event)
                continue
            key = (event.point.x.terms, event.point.y.terms)
            entry = meetings.setdefault(
                key, {"event": event, "vertices": set(), "events": []}
            )
            entry["vertices"].add(event.vertex)
            entry["vertices"].add(met.ident)
            entry["events"].append(event)
        return meetings, cuts

    def _front_vertex_met_by(
        self, event: CandidateEventV1
    ) -> tuple[_Vertex | None, bool]:
        """Вершина фронта, стоящая ровно в точке кандидата, и её соседство.

        Второе значение — «эта вершина является соседом по LAV». Различать
        обязательно: у соседа встреча означает схлопывание ребра между ними,
        то есть штатное edge-событие, а у не-соседа — пересоединение.
        """

        vertex = self.vertices[event.vertex]
        for other in (
            self._edge_start_vertex(event.edge),
            self._edge_end_vertex(event.edge),
        ):
            if other is None or other.ident == vertex.ident:
                continue
            place = self._position(other, event.time)
            if place is None:
                continue
            if (
                (place.x - event.point.x).is_zero
                and (place.y - event.point.y).is_zero
            ):
                return other, (
                    vertex.next == other.ident or other.next == vertex.ident
                )
        return None, False

    def _apply_vertex_meetings(
        self, meetings: dict
    ) -> list[CandidateEventV1]:
        """Все встречи вершин этого уровня. Возвращает НЕразобранные кандидаты.

        Порядок обхода — по каноническому ключу точки, значит от порядка входа
        не зависит. Встречи разных точек независимы: они трогают разные участки
        LAV.

        Встреча, для которой сшивка не доказана, НЕ отбрасывается и не отказывает
        всё построение: её кандидаты возвращаются прежнему разбору — разрезу — и
        считаются под именем. Это признанный долг, а не решение: разрезом такая
        конфигурация обрабатывается неверно (рождается отрезок нулевой длины), но
        измерено, что на фигурах общего положения прежний разбор даёт точный
        ответ, а отказ отнял бы его. Число в счётчике — размер этого долга.
        """

        deferred: list[CandidateEventV1] = []
        for key in sorted(meetings, key=_point_sort_key):
            entry = meetings[key]
            live = sorted(
                ident
                for ident in entry["vertices"]
                if self.vertices[ident].alive
            )
            if len(live) < 2:
                self.counters["discarded_stale_candidates"] += 1
                continue
            meeting = [self.vertices[ident] for ident in live]
            pairs = _reconnect_by_rays(
                tuple(
                    (
                        _incoming_ray(self.edges[vertex.prev_edge].line),
                        _outgoing_ray(self.edges[vertex.next_edge].line),
                        vertex,
                    )
                    for vertex in meeting
                )
            )
            if pairs is None or self._meeting_touches_itself(meeting):
                target_edges = tuple(
                    event.edge for event in entry["events"]
                )
                incident_edges = tuple(
                    edge_id
                    for vertex in meeting
                    for edge_id in (vertex.prev_edge, vertex.next_edge)
                )
                target_vertices = tuple(
                    ident
                    for edge_id in target_edges
                    for ident in self._proof_edge_endpoint_ids(edge_id)
                )
                self._refuse(
                    CandidateRefusal.NO_RULE_MEETING_NOT_RECONNECTABLE,
                    vertex_ids=tuple(vertex.ident for vertex in meeting)
                    + target_vertices,
                    participant_edge_keys=self._edge_keys(
                        *incident_edges, *target_edges
                    ),
                    target_edge_keys=self._edge_keys(*target_edges),
                )
                deferred.extend(entry["events"])
                continue
            self._apply_vertex_meeting(meeting, pairs, entry["event"])
        return deferred

    def _meeting_touches_itself(self, meeting: list[_Vertex]) -> bool:
        """Есть ли среди встретившихся пара СОСЕДЕЙ по LAV.

        Соседи, пришедшие в одну точку, — это схлопнувшееся ребро, то есть
        штатное edge-событие, и разбирать его пересоединением нельзя: у
        пересоединения сосед считается остающимся, а он умирает.
        """

        idents = {vertex.ident for vertex in meeting}
        return any(
            vertex.next in idents or vertex.prev in idents for vertex in meeting
        )

    def _apply_vertex_meeting(
        self,
        meeting: list[_Vertex],
        pairs: tuple[tuple[_Vertex, _Vertex], ...],
        event: CandidateEventV1,
    ) -> None:
        """Вершины фронта встретились в одной точке: ПЕРЕСОЕДИНЕНИЕ по лучам.

        Вокруг точки сходятся концы рёбер — по два от каждой встретившейся
        вершины. Все вершины умирают, и на их месте рождаются новые: каждая
        сшивает ВХОДЯЩЕЕ ребро одной с ИСХОДЯЩИМ ребром другой. Какое с каким —
        решают лучи (`_reconnect_by_rays`), а не порядок в списке, поэтому от
        перестановки входа результат не зависит.

        Узел скелета один, и участников у него столько, сколько различных рёбер
        сошлось, — четыре при встрече двух вершин, до восьми при встрече
        четырёх. Ровно такого узла у креста и не хватало: на пересечении гребней
        встречаются четыре стенки блока, и грань каждой обязана его знать.
        """

        participants = sorted(
            {self.edges[vertex.prev_edge].key for vertex in meeting}
            | {self.edges[vertex.next_edge].key for vertex in meeting}
        )
        self._emit(EventKind.SPLIT, event, tuple(participants), len(meeting))
        self.counters["vertex_meeting_events"] += 1
        if len(participants) > 3:
            self.counters["multi_participant_nodes"] += 1

        # Соседи снимаются ДО первой правки: пересоединение переписывает ссылки,
        # и вторая пара увидела бы уже переписанные.
        joints = tuple(
            (
                self.vertices[incoming.prev],
                incoming.prev_edge,
                self.vertices[outgoing.next],
                outgoing.next_edge,
            )
            for incoming, outgoing in pairs
        )
        for vertex in meeting:
            vertex.alive = False
        born: list[_Vertex] = []
        for before, prev_edge, after, next_edge in joints:
            merged = self._new_vertex(
                prev_edge=prev_edge,
                next_edge=next_edge,
                prev=before.ident,
                next=after.ident,
                birth=event.time,
                point=event.point,
            )
            before.next = merged.ident
            after.prev = merged.ident
            born.append(merged)
        for merged in born:
            self._enqueue_for(merged)
        for before, _, _, _ in joints:
            # У предшественника сменился сосед, значит сменилась и третья прямая
            # его собственного edge-события. Не пересчитать — потерять событие.
            self._enqueue_edge_event(before)

    def _dedupe_by_vertex(
        self, splits: list[CandidateEventV1]
    ) -> list[CandidateEventV1] | None:
        """Одна reflex-вершина — один разрез за уровень.

        Два случая, и они РАЗНЫЕ, поэтому и обходятся по-разному.

        Одна и та же точка у нескольких кандидатов означает, что вершина
        пришла в УГОЛ фронта — туда, где два отрезка уже сходятся. Это одно
        событие, а не два; лишние цели считаются `coincident_split_targets`,
        и цель выбирается по геометрическому ключу, то есть от порядка входа
        не зависит.

        Разные точки означают развилку: вершина одновременно достигает двух
        разных мест. Правила для этого в срезе НЕТ, и брать первое попавшееся
        значило бы вернуть порядковую зависимость чёрным ходом. Исход
        именованный.
        """

        grouped: dict[int, list[CandidateEventV1]] = {}
        for event in splits:
            grouped.setdefault(event.vertex, []).append(event)
        chosen: list[CandidateEventV1] = []
        for vertex_id in sorted(grouped):
            candidates = grouped[vertex_id]
            points = {
                (event.point.x.terms, event.point.y.terms) for event in candidates
            }
            if len(points) > 1:
                self.refusal = SkeletonOutcome.MULTIWAY_SPLIT_UNPROVEN
                return None
            candidates.sort(key=lambda event: (self.edges[event.edge].key, event.edge))
            self.counters["coincident_split_targets"] += len(candidates) - 1
            chosen.append(candidates[0])
        return chosen

    def _group_splits(
        self, splits: list[CandidateEventV1]
    ) -> list[tuple[int, list[CandidateEventV1]]]:
        """Разрезы одного уровня, сгруппированные по рассекаемому отрезку.

        Внутри группы точки упорядочены ВДОЛЬ ребра точным сравнением проекций.
        Порядок групп между собой не влияет ни на что: отрезки независимы.
        """

        grouped: dict[int, list[CandidateEventV1]] = {}
        for event in splits:
            grouped.setdefault(event.edge, []).append(event)
        ordered: list[tuple[int, list[CandidateEventV1]]] = []
        for edge_id in sorted(grouped, key=lambda ident: self.edges[ident].key):
            line = self.edges[edge_id].line
            group = sorted(
                grouped[edge_id],
                key=cmp_to_key(
                    lambda left, right: (
                        _project(line, left.point) - _project(line, right.point)
                    ).sign()
                ),
            )
            ordered.append((edge_id, group))
        return ordered

    def _edge_event_is_live(self, event: CandidateEventV1) -> bool:
        vertex = self.vertices[event.vertex]
        peer = self.vertices[event.peer]
        return (
            vertex.alive
            and peer.alive
            and vertex.next == peer.ident
            and peer.prev == vertex.ident
        )

    def _split_is_live(self, event: CandidateEventV1) -> bool:
        vertex = self.vertices[event.vertex]
        if not vertex.alive:
            return False
        edge = self.edges[event.edge]
        if edge.ident in (vertex.prev_edge, vertex.next_edge):
            return False
        return self._edge_span_contains(edge, event.point, event.time)

    # ---- применение событий ---------------------------------------------

    def _apply_multi_edge(self, events: list[CandidateEventV1]) -> None:
        """Кластер сходящихся в ОДНОЙ точке рёбер — одна цепь, один узел."""

        for event in events:
            if not event.span_unproven:
                continue
            vertex = self.vertices[event.vertex]
            peer = self.vertices[event.peer]
            identity = self._edge_obligation_identity(vertex, peer)
            self.counters["edge_collapse_span_unproven_but_accepted"] += 1
            self._record_obligation(
                cause=ProofObligationBranch.EDGE_COLLAPSE_SPAN_UNPROVEN,
                disposition=(
                    ProofObligationDisposition.EVENT_ACCEPTED_WITH_UNPROVEN_SPAN
                ),
                vertex_ids=identity[0],
                participant_edge_keys=identity[1],
                target_edge_keys=identity[2],
                level=event.time,
            )

        chains = _chains(
            {event.vertex for event in events} | {event.peer for event in events},
            self.vertices,
        )
        sample = events[0]
        participants = sorted(
            {
                self.edges[self.vertices[ident].next_edge].key
                for chain in chains
                for ident in chain
            }
            | {
                self.edges[self.vertices[ident].prev_edge].key
                for chain in chains
                for ident in chain
            }
        )
        converging = sum(len(chain) for chain in chains)
        self._emit(EventKind.EDGE, sample, tuple(participants), converging)
        self.counters["edge_events"] += 1
        if converging > 2:
            self.counters["multi_participant_nodes"] += 1
        for chain in chains:
            self._collapse_chain(chain, sample)

    def _collapse_chain(
        self, chain: tuple[int, ...], event: CandidateEventV1
    ) -> None:
        head, tail = self.vertices[chain[0]], self.vertices[chain[-1]]
        closed = self.vertices[tail.next].ident == head.ident
        for ident in chain:
            self.vertices[ident].alive = False
        if closed:
            self.counters["peaks"] += 1
            return
        before, after = self.vertices[head.prev], self.vertices[tail.next]
        merged = self._new_vertex(
            prev_edge=head.prev_edge,
            next_edge=tail.next_edge,
            prev=before.ident,
            next=after.ident,
            birth=event.time,
            point=event.point,
        )
        before.next = merged.ident
        after.prev = merged.ident
        self._enqueue_for(merged)
        # У предшественника сменился сосед, значит сменилась и третья прямая
        # его собственного edge-события. Не пересчитать её — потерять событие.
        self._enqueue_edge_event(before)

    def _apply_multi_split(
        self, edge_id: int, group: list[CandidateEventV1]
    ) -> None:
        """`MultiSplitEvent`: отрезок фронта режется СРАЗУ во всех точках.

        Это тот случай, ради которого срез: у квадрата с двумя дырами четыре
        верхних угла дыр приходят к верхнему ребру в один и тот же момент.
        Последовательное применение дало бы разный ответ на разном порядке
        входа — измерено, дайджесты расходились. Разрез сразу во всех точках
        от порядка не зависит: точки упорядочены геометрией, а не очередью.

        Раскладка отрезков вдоль ребра: `start .. p_1 .. p_2 .. p_m .. end`.
        Отрезок, примыкающий к `end`, сохраняет исходный идентификатор ребра,
        остальные получают близнецов с ТЕМ ЖЕ геометрическим ключом.
        """

        edge = self.edges[edge_id]
        span_start = self._edge_start_vertex(edge_id)
        span_end = self._edge_end_vertex(edge_id)
        if span_start is None or span_end is None:
            self.counters["discarded_stale_candidates"] += len(group)
            emitters = tuple(self.vertices[event.vertex] for event in group)
            incident_edges = tuple(
                edge_ident
                for vertex in emitters
                for edge_ident in (vertex.prev_edge, vertex.next_edge)
            )
            endpoints = self._proof_edge_endpoint_ids(edge_id)
            self._refuse(
                CandidateRefusal.NO_RULE_SPAN_VANISHED,
                vertex_ids=tuple(
                    vertex.ident for vertex in emitters
                )
                + endpoints,
                participant_edge_keys=self._edge_keys(
                    *incident_edges, edge_id
                ),
                target_edge_keys=self._edge_keys(edge_id),
            )
            return
        # segments[i] — отрезок между p_{i-1} и p_i; последний примыкает к
        # `span_end` и сохраняет исходный идентификатор ребра.
        segments = [self._twin(edge) for _ in range(len(group))] + [edge]
        lefts: list[_Vertex] = []
        rights: list[_Vertex] = []
        for index, event in enumerate(group):
            vertex = self.vertices[event.vertex]
            self._emit_split_node(vertex, edge, event)
            vertex.alive = False
            lefts.append(
                self._new_vertex(
                    prev_edge=vertex.prev_edge,
                    next_edge=segments[index + 1].ident,
                    prev=vertex.prev,
                    next=vertex.ident,
                    birth=event.time,
                    point=event.point,
                )
            )
            rights.append(
                self._new_vertex(
                    prev_edge=segments[index].ident,
                    next_edge=vertex.next_edge,
                    prev=vertex.ident,
                    next=vertex.next,
                    birth=event.time,
                    point=event.point,
                )
            )
        self._relink_multi_split(group, segments, lefts, rights, span_start, span_end)
        for segment in segments:
            self._enqueue_splits_against(segment)

    def _relink_multi_split(
        self,
        group: list[CandidateEventV1],
        segments: list[_Edge],
        lefts: list[_Vertex],
        rights: list[_Vertex],
        span_start: _Vertex,
        span_end: _Vertex,
    ) -> None:
        """Сшивание LAV после многоточечного разреза. Только ссылки."""

        span_start.next_edge = segments[0].ident
        span_start.next = rights[0].ident
        rights[0].prev = span_start.ident
        self._register(span_start)
        for index in range(len(group) - 1):
            lefts[index].next = rights[index + 1].ident
            rights[index + 1].prev = lefts[index].ident
        lefts[-1].next = span_end.ident
        span_end.prev = lefts[-1].ident
        span_end.prev_edge = segments[-1].ident
        self._register(span_end)
        for index, event in enumerate(group):
            vertex = self.vertices[event.vertex]
            predecessor = self.vertices[vertex.prev]
            successor = self.vertices[vertex.next]
            predecessor.next = lefts[index].ident
            lefts[index].prev = predecessor.ident
            successor.prev = rights[index].ident
            rights[index].next = successor.ident
            self._enqueue_edge_event(predecessor)
        for vertex in lefts + rights:
            self._enqueue_for(vertex)
        self._enqueue_edge_event(span_start)

    def _enqueue_splits_against(self, edge: _Edge) -> None:
        """Кандидаты живых reflex-вершин против ВНОВЬ ПОЯВИВШЕГОСЯ отрезка.

        Без этого срез теряет события молча, и потеря видна не сразу: у
        квадрата с двумя дырами четыре верхних угла дыр целят в одно и то же
        верхнее ребро в один и тот же момент, первый разрез рассекает это ребро
        надвое, и оставшиеся кандидаты перестают попадать в СВОЙ отрезок. Рёбра
        во время счёта размножаются, поэтому перепорождение обязательно.

        Запрос обратный к `lines_near` и по построению согласован с ним: пара
        попадает в кандидаты тогда и только тогда, когда у вершины и прямой есть
        общая ячейка. Несогласованность двух направлений теряла бы ровно те
        события, которые нашёл первый проход.
        """

        for ident in self._split_partners(edge):
            vertex = self.vertices[ident]
            if not vertex.alive or not (
                vertex.reflex or vertex.sliding is not None
            ):
                continue
            self.counters["split_candidates_examined"] += 1
            candidate = self._split_candidate(vertex, edge)
            if candidate is not None:
                self.queue.push(candidate)

    def _split_partners(self, edge: _Edge) -> tuple[int, ...]:
        line_ident = self.line_id.get(edge.line_key)
        if (
            self.index is None
            or line_ident is None
            or not self.index.knows_line(line_ident)
        ):
            self.counters["split_search_exhaustive_segments"] += 1
            return tuple(range(len(self.vertices)))
        near = set(self.index.vertices_near(line_ident))
        return tuple(
            sorted(near | self.unindexed_reflex | self.sliding_vertices)
        )

    def _twin(self, edge: _Edge) -> _Edge:
        """Копия ребра под новым идентификатором и с ТЕМ ЖЕ ключом.

        Рассечённое ребро становится несколькими отрезками фронта, у каждого
        свой владелец. Без отдельных идентификаторов вторая reflex-вершина,
        целящая в то же ребро, не нашла бы своего отрезка. На участников
        события и на дайджест раздвоение не влияет: близнец наследует `span`
        рассечённого, то есть остаётся ТЕМ ЖЕ участником. Грань у кусков одного
        ребра одна, и разделить их значило бы придумать грань, которой нет.
        """

        twin = _Edge(len(self.edges), edge.line, edge.span)
        self.edges.append(twin)
        self._register_line(twin)
        return twin

    def _emit_split_node(
        self, vertex: _Vertex, edge: _Edge, event: CandidateEventV1
    ) -> None:
        participants = sorted(
            {
                self.edges[vertex.prev_edge].key,
                self.edges[vertex.next_edge].key,
                edge.key,
            }
        )
        self._emit(EventKind.SPLIT, event, tuple(participants), 1)
        self.counters["split_events"] += 1

    def _classify_sliding(self, vertex: _Vertex) -> None:
        """Развёрнутый стык одной физической прямой: точный общий закон."""

        first = self.edges[vertex.prev_edge].line
        second = self.edges[vertex.next_edge].line
        if not (
            first.a * second.b - second.a * first.b == 0
            and first.a * second.a + first.b * second.b > 0
            # Равны физические скорости, а не сырые `q`: нормали одного
            # направления могут иметь разный целочисленный масштаб.
            and first.q * second.normal_squared
            == second.q * first.normal_squared
        ):
            return
        vertex.sliding = _project(first, vertex.point)
        self.sliding_vertices.add(vertex.ident)

    def _new_vertex(self, **fields) -> _Vertex:
        ident = len(self.vertices)
        vertex = _Vertex(ident=ident, reflex=False, **fields)
        first = self.edges[vertex.prev_edge].line
        second = self.edges[vertex.next_edge].line
        vertex.reflex = _is_reflex(first, second)
        self._classify_sliding(vertex)
        self.vertices.append(vertex)
        self._register(vertex)
        if vertex.reflex and self.graph is not None:
            # У рождённой вершины motorcycle graph трассы не имеет: граф
            # определён на reflex-вершинах ВХОДА. Её граница — стенная, то есть
            # слабее теоремы 2.11, но верна без всякой теоремы.
            self._adopt_trace(
                vertex,
                self.graph.trace_for(
                    self.edges[vertex.prev_edge].line,
                    self.edges[vertex.next_edge].line,
                    vertex.birth,
                    vertex.point,
                ),
            )
        return vertex

    def _emit(
        self,
        kind: EventKind,
        event: CandidateEventV1,
        participants: tuple[tuple[int, ...], ...],
        converging: int,
    ) -> None:
        self.nodes.append(
            SkeletonNodeV1(
                kind, event.time, event.point, participants, converging
            )
        )

    def _close_short_lavs(self) -> None:
        """LAV из двух вершин — конёк крыши: событий больше не будет."""

        for vertex in self.vertices:
            if not vertex.alive:
                continue
            peer = self.vertices[vertex.next]
            if peer.ident == vertex.ident or self.vertices[peer.next].ident != vertex.ident:
                continue
            vertex.alive = False
            peer.alive = False
            self.counters["ridges"] += 1


def _project(line: SupportLineV1, point: EventPointV1) -> SqrtSumV1:
    """Проекция на направление ребра `(b, -a)`. Целые коэффициенты."""

    return point.x.scaled(line.b) - point.y.scaled(line.a)


def _direction(line: SupportLineV1) -> tuple[int, int]:
    """Направление ребра по его несущей прямой, приведённое к примитивному.

    Приведение обязательно: у двух коллинеарных рёбер разной длины `(b, -a)`
    отличается множителем, а луч у них один и тот же. Сравнивать надо лучи, а не
    их масштаб.
    """

    dx, dy = line.b, -line.a
    common = gcd(abs(dx), abs(dy))
    return (dx // common, dy // common)


def _incoming_ray(line: SupportLineV1) -> tuple[int, int]:
    """Куда от точки встречи уходит отрезок фронта, который в неё ВХОДИТ."""

    dx, dy = _direction(line)
    return (-dx, -dy)


def _outgoing_ray(line: SupportLineV1) -> tuple[int, int]:
    """Куда от точки встречи уходит отрезок фронта, который из неё ВЫХОДИТ."""

    return _direction(line)


def _reconnect_by_rays(
    ends: tuple[tuple[tuple[int, int], tuple[int, int], _Vertex], ...]
) -> tuple[tuple[_Vertex, _Vertex], ...] | None:
    """Кого с кем сшивать после встречи вершин. Решают ЛУЧИ, а не порядок.

    Вокруг точки встречи каждая вершина оставляет два конца отрезка фронта:
    входящий (её `prev_edge`) и исходящий (`next_edge`). Каждый конец — это
    луч, и весь вопрос в том, какой входящий продолжается каким исходящим.

    Первое условие — РАЗЛИЧИМОСТЬ: два конца одного рода на одном луче означают
    два отрезка фронта, идущих от точки в одну сторону, то есть фронт сложился
    вдвое. Такую конфигурацию пересоединение не описывает вовсе.

    Двум вершинам сшивка навязана арифметикой: двух входящих и двух исходящих
    концов хватает ровно на два полных паросочетания, и второе — «каждая со
    своими», то есть отсутствие события. Значит остаётся крест-накрест.

    Троим и более решают лучи. Входящий и исходящий концы на ОДНОМ луче — это
    два совпавших отрезка, идущих навстречу: рукав, схлопнувшийся по всей длине.
    Они аннигилируют и сшиваются друг с другом; всё, что осталось, сшивается в
    единственную оставшуюся пару. Проверено на квадратном блоке креста, где в
    одной точке встречаются все четыре вогнутые вершины: четыре аннигилирующие
    пары и ни одного остатка.

    `None` означает, что правила НЕТ: концы неразличимы по лучам либо остатка
    больше одной пары. Взять любую из возможных сшивок значило бы вернуть
    зависимость от порядка входа чёрным ходом.
    """

    incoming: dict[tuple[int, int], list[_Vertex]] = {}
    outgoing: dict[tuple[int, int], list[_Vertex]] = {}
    for ray_in, ray_out, vertex in ends:
        incoming.setdefault(ray_in, []).append(vertex)
        outgoing.setdefault(ray_out, []).append(vertex)
    if len(incoming) != len(ends) or len(outgoing) != len(ends):
        return None
    if len(ends) == 2:
        (_, _, first), (_, _, second) = ends
        return ((first, second), (second, first))

    pairs: list[tuple[_Vertex, _Vertex]] = []
    for ray in sorted(set(incoming) & set(outgoing)):
        pairs.append((incoming.pop(ray)[0], outgoing.pop(ray)[0]))
    rest_in = [vertex for group in incoming.values() for vertex in group]
    rest_out = [vertex for group in outgoing.values() for vertex in group]
    if len(rest_in) != len(rest_out):
        return None
    if len(rest_in) > 1:
        return None
    if rest_in:
        pairs.append((rest_in[0], rest_out[0]))
    return tuple(pairs)


def _joint_kind(
    first: SupportLineV1, second: SupportLineV1
) -> CandidateRefusal:
    """Какое именно вырождение у стыка двух ПАРАЛЛЕЛЬНЫХ прямых фронта.

    Направление ребра — `(b, -a)`. Скалярное произведение направлений решает
    вопрос точно и целочисленно: положительное — рёбра смотрят в одну сторону
    (угол развёрнутый, вершина скользит вдоль прямой), отрицательное — навстречу
    (фронт вывернулся, два его отрезка совпали).

    У сонаправленного стыка спрашивается ещё и равенство скоростей `q1/N1` и
    `q2/N2`: прямые разных скоростей совпадают на одно мгновение, и называть
    такой стык развёрнутым углом было бы неверным диагнозом, а не огрублением.
    """

    dot = first.b * second.b + first.a * second.a
    if dot <= 0:
        return CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL
    if first.q * second.normal_squared != second.q * first.normal_squared:
        return CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS
    return CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL


def _is_reflex(first: SupportLineV1, second: SupportLineV1) -> bool:
    """Вогнутость стыка двух рёбер — знак векторного произведения направлений."""

    cross = first.b * (-second.a) - (-first.a) * second.b
    return cross < 0


def _chains(
    idents: set[int], vertices: list[_Vertex]
) -> tuple[tuple[int, ...], ...]:
    """Разложить множество вершин на СВЯЗНЫЕ участки LAV.

    Это `CreateChains` у Kendzi. Кластер одного уровня может задевать
    несколько несмежных участков фронта; каждый схлопывается отдельно.
    """

    live = {ident for ident in idents if vertices[ident].alive}
    chains: list[tuple[int, ...]] = []
    seen: set[int] = set()
    for ident in sorted(live):
        if ident in seen:
            continue
        start = ident
        guard = 0
        while (
            vertices[start].prev in live
            and vertices[start].prev != ident
            and guard <= len(live)
        ):
            start = vertices[start].prev
            guard += 1
        chain = [start]
        seen.add(start)
        cursor = vertices[start].next
        while cursor in live and cursor not in seen:
            chain.append(cursor)
            seen.add(cursor)
            cursor = vertices[cursor].next
        chains.append(tuple(chain))
    return tuple(chains)
