"""Таблица станций: `(s, r)` каждой точки покрытия, ТОЧНО и без sympy.

Что такое станция. Закон полосы `PLANAR_LINEAR_NORMAL_OFFSET_V1` называет две
координаты: продольную `s` — вдоль `ChainUse` источника, в его ориентации, и
поперечную `r` — знаковое расстояние до несущей прямой источника внутрь
владельца. Всё остальное (UV, провенанс, шов) выводится из этих двух чисел и
ничего больше не спрашивает у геометрии.

Что здесь есть.

1. `source_chain_by_span` — переехало из хоста (отладка и продукт читают один
   код): какая `PhysicalChain` стоит под решёточным отрезком региона.
2. `chain_station_table` — по каждому `ChainUse` домена копится ДЛИНА в метрике
   Грама: `s0` — длина всех рёбер ДО данного. Рёбра собираются в ПРОБЕГИ —
   подряд идущие коллинеарные рёбра одного направления, — потому что внутри
   пробега `s` и `r` одна формула, а на изломе цепи у соседних рёбер разные
   системы координат, и вершина на их общей биссектрисе законно имеет два
   набора `(s, r)`.
3. `station_of` / `transverse_of` — сами числа: `s` точки относительно пробега
   и `r` относительно несущей прямой грани.

ЕДИНИЦЫ. Всё считается в единицах РЕШЁТКИ (`GridSpecV1.scale` целых узлов на
единицу карты); метры источника получаются делением на `scale` ОДИН раз, на
выходе (`uv_law`, `domain`). Метрика — Грам `G` аффинной карты домена
(`exact_gram_matrix`, дроби); карта НЕ ортонормальна, поэтому длина решёточного
вектора `d` — это `sqrt(d^T G d)`, а не евклидова.

ЧТО НЕ ДЕЛАЕТСЯ. Ни `sympy`, ни float: радикалы — `SqrtSumV1.radical`, и каждый
из них идёт под бюджетом (новый радиканд — это факторизация). `r` — это время
прихода `(a x + b y - c) / sqrt(q)` несущей прямой `FaceV1.line`: при единичной
нормальной скорости оно равно расстоянию, а на фронте равно `alpha` ТОЧНО.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction

from .._corner_treatment import shared_source_lineage
from ..contracts.envelopes import CornerTreatmentV1
from ..exact_sqrt_sum import SqrtSumV1
from ..planar_metric import fraction_from_exact
from ..reference.planar_types import ConstructionKind
from ..wavefront.bridge import _lattice_image
from ..wavefront.conveyor import _region_loops
from .coalesce import integer_line_class, undirected_span


#: Почему станция чего-то НЕ вывелась. Каждая причина — отдельное число таблицы
#: (`STATION_SKIP_<ПРИЧИНА>`, с нулями) и строка в `ChainStationTableV1.skips`:
#: до этого пропуск приходил позже отказом `STATION_CHAIN_UNNAMED` без причины.
SKIP_DOMAIN_MISSING = "DOMAIN_MISSING"
SKIP_REGION_LOOPS_UNREADABLE = "REGION_LOOPS_UNREADABLE"
SKIP_REGION_LATTICE_IMAGE_FAILED = "REGION_LATTICE_IMAGE_FAILED"
SKIP_REGION_NODE_COUNT_MISMATCH = "REGION_NODE_COUNT_MISMATCH"
SKIP_LOOP_SEGMENT_COUNT_MISMATCH = "LOOP_SEGMENT_COUNT_MISMATCH"
SKIP_EDGE_PROVENANCE_AMBIGUOUS = "EDGE_PROVENANCE_AMBIGUOUS"
SKIP_USE_NOT_IN_SNAPSHOT = "CHAIN_USE_NOT_IN_SNAPSHOT"
SKIP_USE_EDGE_VERTEX_UNNAMED = "USE_EDGE_VERTEX_UNNAMED"
SKIP_USE_EDGE_PAIR_UNRESOLVED = "USE_EDGE_PAIR_UNRESOLVED"
#: Угол JOIN, у которого вхождения не стыкуются «конец -> начало» в направлении
#: `ChainUse` в вершине угла: поток через него не продолжается, оба вхождения
#: остаются своими кадрами (как без JOIN), и это названо.
SKIP_JOIN_CORNER_NOT_ADJACENT = "JOIN_CORNER_NOT_ADJACENT"
#: Угол JOIN, у которого ровно одно из двух вхождений имеет рёбра в петлях домена:
#: поток пересёк бы границу домена, и он не продолжается (оба — как без JOIN).
SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS = "JOIN_USE_NOT_IN_DOMAIN_LOOPS"
#: Угол JOIN замкнутой цепи из ОДНОГО вхождения: потока нет (его пробеги — по кадру на
#: пробег, как без JOIN), и это названо, а не потеряно.
SKIP_JOIN_CYCLE_OF_ONE_USE = "JOIN_CYCLE_OF_ONE_USE"
#: Стык двух кусков ОДНОЙ цепи хоста без записи угла (хост пишет запись каждому вогнутому стыку, то есть стык
#: выпуклый либо вырожденный в карте), чей изгиб в карте НЕ МЕНЬШЕ четверти оборота (`dot_G(вход, выход) <= 0`; имя
#: «beyond» включает саму четверть): тот же СТРОГИЙ предел, что у JOIN вогнутого угла (`δ/π < 1/2`). Поток через него
#: не идёт, угол остаётся углом (шов): у прямого угла излом билинейной UV доходил до 1.0–1.15 alpha.
SKIP_JOIN_BEND_BEYOND_QUARTER_TURN = "JOIN_BEND_BEYOND_QUARTER_TURN"
#: Стык потока (закон `CORNER_JOIN_SAME_PCHAIN_V1` либо JOIN плана), снятый доменом после конфликта станций
#: (`STATION_VALUE_CONFLICT`), который не свёлся ни перекладиной (`RUNG_STATION_FROM_CHAIN_VERTEX_V1`), ни ребром
#: (`RUNG_CHORD_STATION_V1`): вершина события скелета (короткий кусок между двумя выпуклыми стыками вырождается, и две
#: биссектрисы сходятся в узле) получила бы три разных `s` от трёх пробегов одного потока; единой станции узла у закона
#: нет. Стык остаётся углом (шов): домен не отказывает из-за стыка. Первыми снимаются стыки без записи угла, JOIN плана —
#: последним средством (его угол и митра остаются: снимается только непрерывность `u`).
SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT = "JOIN_WITHDRAWN_AT_STATION_CONFLICT"
#: Имя кадра первого вхождения замкнутого потока (`FLOW_CYCLE_OPENED`): суффикс к ключу потока.
OPENING_FRAME_SUFFIX = "@opening"
STATION_SKIP_REASONS = (
    SKIP_DOMAIN_MISSING,
    SKIP_REGION_LOOPS_UNREADABLE,
    SKIP_REGION_LATTICE_IMAGE_FAILED,
    SKIP_REGION_NODE_COUNT_MISMATCH,
    SKIP_LOOP_SEGMENT_COUNT_MISMATCH,
    SKIP_EDGE_PROVENANCE_AMBIGUOUS,
    SKIP_USE_NOT_IN_SNAPSHOT,
    SKIP_USE_EDGE_VERTEX_UNNAMED,
    SKIP_USE_EDGE_PAIR_UNRESOLVED,
    SKIP_JOIN_CORNER_NOT_ADJACENT,
    SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS,
    SKIP_JOIN_CYCLE_OF_ONE_USE,
    SKIP_JOIN_BEND_BEYOND_QUARTER_TURN,
    SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT,
)


def region_lattice_loops(prepared, skips: list | None = None):
    """Регионы домена с их решёточными петлями: `(регион, источники, узлы)`.

    Решёточный образ берётся ТЕМИ ЖЕ функциями ядра (`_region_loops` и
    `_lattice_image`), которыми мост строил полигон. Второй способ его
    посчитать разошёлся бы с первым молча — а разойтись ему есть на чём:
    привязка к решётке двигает вершины, и повторять её округление на глаз
    означало бы получать другой ответ на другом масштабе.

    Регион, у которого петли не читаются либо число узлов не совпало с числом
    источников, пропускается — и пропуск НАЗЫВАЕТСЯ: если задан `skips`, в него
    ложится `(регион, причина)`. Без `skips` поведение прежнее (так зовёт
    отладочный хост): молчаливым пропуск быть перестаёт только там, где есть
    кому его прочесть.
    """

    domain = getattr(prepared, "domain", None)
    if domain is None:
        if skips is not None:
            skips.append(("domain", SKIP_DOMAIN_MISSING))
        return
    lattice = getattr(prepared, "lattice", None)
    for region in domain.domain_regions:
        loops, _issue = _region_loops(region)
        if loops is None:
            if skips is not None:
                skips.append((region.region_id, SKIP_REGION_LOOPS_UNREADABLE))
            continue
        lattice_loops, _off_lattice, _residual = _lattice_image(loops, lattice)
        sources = (region.outer, *region.holes)
        if lattice_loops is None:
            if skips is not None:
                skips.append((region.region_id, SKIP_REGION_LATTICE_IMAGE_FAILED))
            continue
        if len(lattice_loops) != len(sources):
            if skips is not None:
                skips.append((region.region_id, SKIP_REGION_NODE_COUNT_MISMATCH))
            continue
        yield region, sources, lattice_loops


def source_chain_by_span(prepared) -> dict:
    """`region_id -> (ненаправленный решёточный отрезок -> PhysicalChain)`.

    АДДИТИВНАЯ выгрузка поверх готовой подготовки ядра: ни одного байта ядра
    она не двигает и ни одного его числа не пересчитывает. Ядру цепь не нужна —
    владельцем грани у него служит вхождение отрезка (`EdgeKey`), — а домену
    она известна: `_loop_segment` кладёт `physical_chain_ids` в провенанс
    КАЖДОГО сегмента граничной петли.

    Сегмент, у которого цепь названа не единственным именем, и отрезок, на
    который два сегмента ответили по-разному, остаются БЕЗ цепи: два разных
    ответа — не ответ, и выбирать между ними здесь нечем.
    """

    result: dict[str, dict] = {}
    for region, sources, lattice_loops in region_lattice_loops(prepared):
        by_span: dict[tuple, str | None] = {}
        for loop_index, source in enumerate(sources):
            nodes = lattice_loops[loop_index]
            segments = source.segments
            if len(segments) != len(nodes):
                continue
            for index, segment in enumerate(segments):
                names = tuple(
                    sorted(segment.provenance.physical_chain_ids)
                )
                if len(names) != 1:
                    continue
                start = nodes[index]
                end = nodes[(index + 1) % len(nodes)]
                span = undirected_span(
                    (start[0], start[1], end[0], end[1])
                )
                if span is None:
                    continue
                if span in by_span and by_span[span] != names[0]:
                    by_span[span] = None
                    continue
                by_span[span] = names[0]
        result[region.region_id] = {
            span: name for span, name in by_span.items() if name is not None
        }
    return result


# --------------------------------------------------------------------------
# Таблица станций
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class StationRunV1:
    """Пробег цепи: подряд идущие коллинеарные рёбра ОДНОГО направления.

    Внутри пробега станция точки — одна формула
    `s = s_origin + <p - origin, direction>_G / |direction|_G`, а `r` — до
    прямой пробега. `direction` — целый вектор первого ребра В НАПРАВЛЕНИИ
    `ChainUse`; `covector = G * direction` лежит готовым, потому что считать его
    на каждой точке значило бы переумножать одни и те же дроби.
    """

    run_id: str
    chain_id: str
    chain_use_id: str
    run_index: int
    origin: tuple[int, int]
    direction: tuple[int, int]
    #: Длина (в единицах решётки) всех рёбер цепи ДО начала пробега.
    s_origin: SqrtSumV1
    #: `1 / |direction|_G`. Радикал, поэтому живёт `SqrtSumV1`, а не дробью.
    inverse_length: SqrtSumV1
    covector: tuple[Fraction, Fraction]
    physical_edge_ids: tuple[str, ...]
    lineage_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class StationEdgeV1:
    """Одно ребро петли домена: к какому пробегу оно относится и чьё оно."""

    run_id: str
    chain_id: str
    chain_use_id: str
    physical_edge_id: str
    #: Концы в направлении ПЕТЛИ домена (так же, как ключ владельца у ядра).
    start: tuple[int, int]
    end: tuple[int, int]
    start_vertex_id: str | None
    end_vertex_id: str | None
    #: `True`, если направление петли совпадает с направлением `ChainUse`.
    along_chain_use: bool


@dataclass(frozen=True, slots=True)
class StationCornerV1:
    """Вершина петли: ребро, входящее в неё, и ребро, выходящее из неё."""

    region_id: str
    node: tuple[int, int]
    vertex_id: str | None
    incoming: tuple[int, int, int, int]
    outgoing: tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class ChainStationTableV1:
    """Станции всех цепей домена и всё, что нужно, чтобы по узлу найти цепь.

    `unnamed_chain_ids` — цепи, у которых станцию вывести НЕ из чего (ребро
    цепи вне петли, неоднозначная пара вершин): грани на них материализатор
    отдаёт именованным отказом, а не придуманным `s`.
    """

    scale: int
    gram: tuple[Fraction, Fraction, Fraction]
    runs: dict
    edges: dict
    corners: dict
    node_vertex_ids: dict
    unnamed_chain_ids: frozenset[str]
    #: Цепи, у которых накопление длины пришлось начать заново из-за ребра вне
    #: петли домена. Не отказ, а диагностика `U_RESTARTS_AT_DOMAIN_BORDER`.
    restart_chain_ids: frozenset[str] = frozenset()
    counters: tuple[tuple[str, int], ...] = field(default=(), compare=False)
    #: Всё, что станция НЕ вывела, с причиной: `(где, причина)`. «Где» — регион,
    #: петля либо `ChainUse`. Нужны затем, что отказ `STATION_CHAIN_UNNAMED`
    #: приходит с грани и сам причины не знает.
    skips: tuple[tuple[str, str], ...] = ()
    #: ПОТОКИ (закон `CORNER_JOIN_SOFT_BEND_V1`): `run_id -> ключ потока` для
    #: пробегов, лежащих в потоке из двух и более `ChainUse`; у одиночного
    #: вхождения записи нет, его кадр — прежний пробег.
    flow_of_run: dict = field(default_factory=dict, compare=False)
    #: Стык JOIN: `(пробег до угла, пробег после угла) -> s вершины угла`.
    #: По нему `assemble.station_values` даёт перекладине на биссектрисе
    #: станцию вершины цепи (`RUNG_STATION_FROM_CHAIN_VERTEX_V1`).
    joins: dict = field(default_factory=dict, compare=False)
    #: Кадр каждого пробега потока: ключ потока либо `<ключ>@opening` у вхождения, с которого
    #: ЗАМКНУТЫЙ поток размыкается (`FLOW_CYCLE_OPENED`). Регион — это кадр, а у кольца из
    #: мягких изломов вершина разреза обязана иметь два набора `(s, r)`, то есть два региона.
    frame_of_run: dict = field(default_factory=dict, compare=False)
    #: Разрезы замкнутых потоков: `(ключ потока, вхождение-замыкатель, вхождение-открыватель,
    #: цепь открывателя)`. Каждый — диагностика `U_RESTARTS_AT_CLOSED_FLOW_OPENING`.
    cuts: tuple = field(default=(), compare=False)
    #: Стыки закона `CORNER_JOIN_SAME_PCHAIN_V1` БЕЗ записи угла (выпуклые и вырожденные в карте): `(вершина,
    #: вхождение до, вхождение после, `COLLINEAR` | `CONVEX`)`. Вогнутые стыки несёт план (`CornerTreatmentRecordV1`).
    same_chain_joins: tuple = field(default=(), compare=False)
    #: Стыки JOIN ПЛАНА (вогнутый угол с записью `CornerTreatmentRecordV1`), ставшие потоком: `(вершина, вхождение до,
    #: вхождение после)`. Домен снимает такой стык только последним средством (`junction_to_withdraw`).
    plan_joins: tuple = field(default=(), compare=False)
    #: Вершины углов `MITER_SEAM` плана (закон `CORNER_MITER_ON_FOLD_V1`) в петлях домена: митра `k = 0` со швом на
    #: биссектрисе, потока нет. Угол только назван и посчитан (`STATION_FOLD_MITER_CORNERS`).
    fold_miters: tuple = field(default=(), compare=False)

    def join_station(self, run_a: str, run_b: str):
        """`s` вершины угла JOIN между двумя пробегами (в любом порядке) либо `None`."""

        found = self.joins.get((run_a, run_b))
        return self.joins.get((run_b, run_a)) if found is None else found

    def cut_join_station(self, run_a: str, run_b: str):
        """`s` вершины угла JOIN, чьи пробеги лежат в РАЗНЫХ кадрах одного разомкнутого кольца, либо `None`.

        Стык двух регионов, где UV обязана не рваться (`RUNG_STATION_FROM_CHAIN_VERTEX_V1`
        через границу кадров): станция вершины цепи одна для обеих сторон.
        """

        if self.frame_of_run.get(run_a) == self.frame_of_run.get(run_b):
            return None
        return self.join_station(run_a, run_b)

    def skip_text(self) -> str:
        """Хвост для детали отказа: первые пропуски поимённо, либо пустая строка."""

        if not self.skips:
            return ""
        shown = ", ".join(f"{where}:{reason}" for where, reason in self.skips[:6])
        more = len(self.skips) - 6
        return f"; station skips: {shown}" + (f" (+{more} more)" if more > 0 else "")

    def edge_of_owner(self, region_id: str, owner):
        """Ребро петли по ключу владельца либо `None`.

        Ключ владельца задан в обходе ПОЛИГОНА, а петля домена может прийти в
        полигон развёрнутой, поэтому ищется и обратный ключ. Если нашлись ОБА
        (щель: одно и то же ребро с двух сторон), ответ неоднозначен и честно
        `None` — решать между двумя станциями здесь нечем.
        """

        if len(owner) != 4:
            return None
        key = tuple(int(item) for item in owner)
        direct = self.edges.get((region_id, key))
        reverse = self.edges.get((region_id, (key[2], key[3], key[0], key[1])))
        if direct is not None and reverse is not None:
            return None
        return direct if direct is not None else reverse

    def run_of(self, edge: StationEdgeV1) -> StationRunV1:
        return self.runs[edge.run_id]


def _gram_of(frame) -> tuple[Fraction, Fraction, Fraction]:
    gram = frame.exact_gram_matrix
    return (
        fraction_from_exact(gram.m00),
        fraction_from_exact(gram.m01),
        fraction_from_exact(gram.m11),
    )


def length_squared_g(
    gram: tuple[Fraction, Fraction, Fraction], vector: tuple[int, int]
) -> Fraction:
    """`d^T G d`: квадрат длины решёточного вектора в метрике Грама."""

    g00, g01, g11 = gram
    dx, dy = vector
    return g00 * dx * dx + 2 * g01 * dx * dy + g11 * dy * dy


def _segment_vertex_id(segment, *, at_start: bool) -> str | None:
    """Исходная вершина конца сегмента по его сертификатам построения."""

    certificates = (
        segment.start_constructions if at_start else segment.end_constructions
    )
    names = {
        name
        for item in certificates
        if item.kind is ConstructionKind.SOURCE_VERTEX
        for name in item.source_vertex_ids
    }
    return next(iter(names)) if len(names) == 1 else None


def _single(values) -> str | None:
    names = tuple(sorted(values))
    return names[0] if len(names) == 1 else None


@dataclass(frozen=True, slots=True)
class _LoopEdge:
    """Ребро петли домена, как его читает сборщик таблицы."""

    region_id: str
    key: tuple[int, int, int, int]
    start: tuple[int, int]
    end: tuple[int, int]
    start_vertex_id: str | None
    end_vertex_id: str | None
    chain_id: str
    chain_use_id: str
    physical_edge_id: str


def _collect_loops(prepared, skips: list | None = None):
    """Рёбра петель с их `ChainUse`, углы петель и узлы с именами вершин.

    `skips` — куда ложатся названные пропуски (регион, петля, ребро без
    единственного провенанса): станции для них не будет, и причина должна
    остаться читаемой, а не раствориться в позднем `STATION_CHAIN_UNNAMED`.
    """

    by_use: dict[str, list[_LoopEdge]] = {}
    corners: dict = {}
    node_ids: dict = {}
    for region, sources, lattice_loops in region_lattice_loops(prepared, skips):
        rid = region.region_id
        for loop_index, source in enumerate(sources):
            nodes = [tuple(node) for node in lattice_loops[loop_index]]
            segments = source.segments
            size = len(nodes)
            if len(segments) != size:
                if skips is not None:
                    skips.append(
                        (f"{rid}/loop{loop_index}", SKIP_LOOP_SEGMENT_COUNT_MISMATCH)
                    )
                continue
            keys = [
                (
                    nodes[i][0],
                    nodes[i][1],
                    nodes[(i + 1) % size][0],
                    nodes[(i + 1) % size][1],
                )
                for i in range(size)
            ]
            for index, segment in enumerate(segments):
                start_id = _segment_vertex_id(segment, at_start=True)
                end_id = _segment_vertex_id(segment, at_start=False)
                for node, vertex_id in (
                    (nodes[index], start_id),
                    (nodes[(index + 1) % size], end_id),
                ):
                    seen = node_ids.get((rid, node), vertex_id)
                    node_ids[(rid, node)] = vertex_id if seen == vertex_id else None
                corners.setdefault((rid, nodes[index]), []).append(
                    StationCornerV1(
                        rid,
                        nodes[index],
                        start_id,
                        keys[(index - 1) % size],
                        keys[index],
                    )
                )
                provenance = segment.provenance
                use_id = _single(provenance.chain_use_ids)
                chain_id = _single(provenance.physical_chain_ids)
                edge_id = _single(provenance.physical_edge_ids)
                if use_id is None or chain_id is None or edge_id is None:
                    # Ребро стены `ChainUse` не называет вовсе, и это штатно.
                    # Пропуском считается только ребро, у которого `ChainUse`
                    # ЕСТЬ, а единственной тройки (использование, цепь, ребро)
                    # из провенанса не вышло: станцию такому ребру взять негде.
                    if skips is not None and provenance.chain_use_ids:
                        skips.append(
                            (f"{rid}/loop{loop_index}/edge{index}",
                             SKIP_EDGE_PROVENANCE_AMBIGUOUS)
                        )
                    continue
                by_use.setdefault(use_id, []).append(
                    _LoopEdge(
                        rid,
                        keys[index],
                        nodes[index],
                        nodes[(index + 1) % size],
                        start_id,
                        end_id,
                        chain_id,
                        use_id,
                        edge_id,
                    )
                )
    return by_use, corners, node_ids


def _pair_index(vertices, closed: bool):
    """`(начало, конец) -> (индекс ребра в обходе ChainUse, вперёд?)` и число рёбер.

    Пара встречается в словаре ОДИН раз: цепь, у которой одна пара вершин даёт
    два ребра, неоднозначна, и запись такой пары снимается (`None`), а не
    выбирается наугад.
    """

    names = [item.value for item in vertices]
    pairs = list(zip(names, names[1:]))
    if closed:
        pairs.append((names[-1], names[0]))
    index: dict = {}
    for position, (first, second) in enumerate(pairs):
        for key, forward in (((first, second), True), ((second, first), False)):
            index[key] = None if key in index else (position, forward)
    return index, len(pairs)


def _order_or_reason(context, chain_use, edges):
    """Рёбра `ChainUse` в порядке обхода и причина отказа: `(ordered, reason)`.

    `ordered` — `[(ребро, вперёд?, рестарт?), ...]`; `None` — пара вершин
    неоднозначна либо ребро названо дважды, и тогда `reason` называет, какое из
    двух: станцию из такого входа выдавать нельзя. Ребро цепи, которого В ПЕТЛЕ
    ДОМЕНА НЕТ (цепь выходит за границу домена), НЕ отказ: накопление длины на
    таком месте начинается заново, и это названо флагом `рестарт` —
    материализатор выставляет диагностику `U_RESTARTS_AT_DOMAIN_BORDER`, а не
    молчит.
    """

    chain = context.chains_by_id[chain_use.physical_chain_id]
    vertices = context.directed_chain_vertices(chain_use)
    index, _count = _pair_index(vertices, chain.is_closed)
    placed: dict[int, tuple] = {}
    for edge in edges:
        if edge.start_vertex_id is None or edge.end_vertex_id is None:
            return None, SKIP_USE_EDGE_VERTEX_UNNAMED
        slot = index.get((edge.start_vertex_id, edge.end_vertex_id))
        if slot is None or slot[0] in placed:
            return None, SKIP_USE_EDGE_PAIR_UNRESOLVED
        placed[slot[0]] = (edge, slot[1])
    ordered = []
    previous = -1
    for position in sorted(placed):
        edge, forward = placed[position]
        ordered.append((edge, forward, position != previous + 1))
        previous = position
    return ordered, ""


def _use_runs(chain_use, chain, ordered, gram, budget):
    """Пробеги и записи рёбер одного `ChainUse` с нуля: `(пробеги, записи)`."""

    return _use_runs_from(chain_use, chain, ordered, gram, budget, None)[:2]


def _use_runs_from(chain_use, chain, ordered, gram, budget, accumulated):
    """Пробеги и записи рёбер одного `ChainUse`. Копит длину `s0`.

    `accumulated` — длина, с которой начинается счёт: `None` (нуль) у начала
    потока, длина предыдущих вхождений потока после угла JOIN. Третьим
    возвращается длина на конце вхождения — она станет началом следующего.
    Рестарт (ребро цепи вне петли домена) обнуляет накопленную длину и
    обязательно начинает новый пробег: у него другое начало отсчёта.
    """

    lineage = tuple(sorted(item.value for item in chain.source_lineage))
    use_id = chain_use.chain_use_id.value
    g00, g01, g11 = gram
    raw_runs: list[tuple] = []
    records: list = []
    accumulated = SqrtSumV1.zero() if accumulated is None else accumulated
    previous_class = None
    run_edges: list[str] = []
    for edge, along, restart in ordered:
        if restart:
            accumulated = SqrtSumV1.zero()
            previous_class = None
        use_start, use_end = (
            (edge.start, edge.end) if along else (edge.end, edge.start)
        )
        vector = (use_end[0] - use_start[0], use_end[1] - use_start[1])
        line = integer_line_class(
            (use_start[0], use_start[1], use_end[0], use_end[1])
        )
        squared = length_squared_g(gram, vector)
        if line is None or line != previous_class:
            run_edges = []
            inverse = (
                SqrtSumV1.radical(1, Fraction(1) / squared, budget)
                if squared
                else SqrtSumV1.zero()
            )
            covector = (
                g00 * vector[0] + g01 * vector[1],
                g01 * vector[0] + g11 * vector[1],
            )
            raw_runs.append(
                (use_start, vector, accumulated, inverse, covector, run_edges)
            )
            previous_class = line
        run_edges.append(edge.physical_edge_id)
        records.append(
            (
                (edge.region_id, edge.key),
                StationEdgeV1(
                    run_id=f"{use_id}#run{len(raw_runs) - 1}",
                    chain_id=edge.chain_id,
                    chain_use_id=use_id,
                    physical_edge_id=edge.physical_edge_id,
                    start=edge.start,
                    end=edge.end,
                    start_vertex_id=edge.start_vertex_id,
                    end_vertex_id=edge.end_vertex_id,
                    along_chain_use=along,
                ),
            )
        )
        if squared:
            accumulated = accumulated + SqrtSumV1.radical(1, squared, budget)
    runs = tuple(
        StationRunV1(
            run_id=f"{use_id}#run{number}",
            chain_id=ordered[0][0].chain_id,
            chain_use_id=use_id,
            run_index=number,
            origin=origin,
            direction=vector,
            s_origin=origin_station,
            inverse_length=inverse,
            covector=covector,
            physical_edge_ids=tuple(names),
            lineage_ids=lineage,
        )
        for number, (origin, vector, origin_station, inverse, covector, names)
        in enumerate(raw_runs)
    )
    return runs, records, accumulated


def _join_successors(context, uses: dict, present, skips: list, withdrawn=frozenset(), joined: list | None = None):
    """`(вхождение до угла -> вхождение после угла, углов вне домена)` по записям JOIN плана.

    Порядок — В НАПРАВЛЕНИИ `ChainUse`, а не петли: поток продолжает счёт `s`
    того вхождения, которое КОНЧАЕТСЯ в вершине угла, тем, которое в ней
    НАЧИНАЕТСЯ. Что не получилось, названо поимённо и в потоки не идёт:
    вхождения смотрят врозь либо их нет в снапшоте (`JOIN_CORNER_NOT_ADJACENT`);
    у ровно одного из двух нет рёбер в петлях домена (`JOIN_USE_NOT_IN_DOMAIN_LOOPS`);
    цепь замкнута на одно вхождение (`JOIN_CYCLE_OF_ONE_USE`). Угол, у которого НЕТ рёбер
    в петлях домена ни у одного из двух вхождений, домену не принадлежит и считается
    числом (`STATION_JOIN_CORNERS_OUT_OF_DOMAIN`), а не пропуском: записи плана идут по
    ВСЕМ углам патча, и угол чужой цепи не есть потерянный стык.

    Записи читаются прямо (`context.compilation`, `context.snapshot`): отсутствующее поле
    — исключение, а не молчаливое «потоков нет».

    `withdrawn` — значения вершин, чей стык домен снял после конфликта станций (`junction_to_withdraw`): такой угол
    остаётся углом, пропуск назван (`JOIN_WITHDRAWN_AT_STATION_CONFLICT`). `joined` — если задан, в него ложится
    `(значение вершины, вхождение до, вхождение после)` каждого стыка, ставшего потоком.
    """

    relations = {item.corner_relation_id: item for item in context.snapshot.corner_relations}
    successors: dict[str, str] = {}
    outside = 0
    for record in sorted(
        context.compilation.corner_treatments,
        key=lambda item: item.corner_relation_id.value,
    ):
        if record.treatment is not CornerTreatmentV1.JOIN_CONTINUATION:
            continue
        where = record.corner_relation_id.value
        first_id, second_id = record.incoming_chain_use_id.value, record.outgoing_chain_use_id.value
        inside = (first_id in present, second_id in present)
        if not any(inside):
            outside += 1
            continue
        first, second = uses.get(first_id), uses.get(second_id)
        relation = relations.get(record.corner_relation_id)
        if first_id == second_id:
            skips.append((where, SKIP_JOIN_CYCLE_OF_ONE_USE))
            continue
        if not all(inside):
            skips.append((where, SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS))
            continue
        pair = None
        if relation is not None and first is not None and second is not None:
            vertex = relation.source_vertex_id
            for before, after in ((first, second), (second, first)):
                ends = context.directed_chain_vertices(before)
                starts = context.directed_chain_vertices(after)
                if ends[-1] == vertex and starts[0] == vertex:
                    pair = (before.chain_use_id.value, after.chain_use_id.value)
        if pair is None or pair[0] in successors or pair[1] in successors.values():
            skips.append((where, SKIP_JOIN_CORNER_NOT_ADJACENT))
            continue
        vertex_value = getattr(relation.source_vertex_id, "value", relation.source_vertex_id)
        if vertex_value in withdrawn:
            skips.append((where, SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT))
            continue
        successors[pair[0]] = pair[1]
        if joined is not None:
            joined.append((vertex_value, pair[0], pair[1]))
    return successors, outside


def _fold_miters(context, present) -> tuple:
    """Вершины углов `MITER_SEAM` плана, у которых хоть одно вхождение имеет рёбра в петлях домена.

    Записи плана идут по ВСЕМ углам патча, и угол чужой цепи домену не принадлежит. Потока у митры нет: угол остаётся
    `k = 0` с швом на биссектрисе (так же, как JOIN, снятый после конфликта станций), `_join_successors` её не берёт.
    """

    relations = {item.corner_relation_id: item for item in context.snapshot.corner_relations}
    vertices = []
    for record in sorted(
        context.compilation.corner_treatments,
        key=lambda item: item.corner_relation_id.value,
    ):
        if record.treatment is not CornerTreatmentV1.MITER_SEAM:
            continue
        if record.incoming_chain_use_id.value in present or record.outgoing_chain_use_id.value in present:
            vertex = relations[record.corner_relation_id].source_vertex_id
            vertices.append(getattr(vertex, "value", vertex))
    return tuple(vertices)


def _dot_g(gram, left, right) -> Fraction:
    """`<left, right>_G` решёточных векторов, точно."""

    g00, g01, g11 = gram
    return g00 * left[0] * right[0] + g01 * (left[0] * right[1] + left[1] * right[0]) + g11 * left[1] * right[1]


def _arm(edge, vertex_id: str):
    """`(узел, вектор от узла вдоль ребра)` конца ребра петли, лежащего в вершине, либо `None`."""

    if edge.start_vertex_id == vertex_id and edge.end_vertex_id != vertex_id:
        return edge.start, (edge.end[0] - edge.start[0], edge.end[1] - edge.start[1])
    if edge.end_vertex_id == vertex_id and edge.start_vertex_id != vertex_id:
        return edge.end, (edge.start[0] - edge.end[0], edge.start[1] - edge.end[1])
    return None


def _same_chain_successors(context, uses: dict, present, skips: list, successors: dict, gram, withdrawn=frozenset()):
    """Стыки закона `CORNER_JOIN_SAME_PCHAIN_V1` БЕЗ записи угла: `[(вершина, до, после, вид), ...]`; пополняет `successors`.

    Решение владельца (2026-10-04): «одна цепь» — факт хоста, а не порог угла. Любой стык двух кусков ОДНОЙ цепи
    ВЛАДЕЛЬЦА (общая запись `chain-source` его патча) продолжает полосу: `u` течёт сквозь стык, регион один, шва нет.
    Вогнутые стыки (хост пишет им запись угла) решает план — `_join_successors`; здесь остальные: у них записи нет,
    потому что стык выпуклый либо вырожденный в карте, и перекладина на биссектрисе та же, что у митры вогнутого
    JOIN (`RUNG_STATION_FROM_CHAIN_VERTEX_V1`: `s_a + s_b = 2 s_v`, равноскоростная биссектриса симметрична).
    Предел изгиба один и тот же и СТРОГИЙ: меньше четверти оборота в карте (`dot_G(вход, выход) > 0`), четверть и
    шире — названный пропуск `JOIN_BEND_BEYOND_QUARTER_TURN`, угол остаётся углом (шов). Вид стыка (`COLLINEAR` | `CONVEX`) читается в той же
    карте решётки, что у всей таблицы, точной арифметикой.

    Стык — это вершина, где вхождение домена КОНЧАЕТСЯ, а другое НАЧИНАЕТСЯ (в направлении `ChainUse`). Куски разных
    цепей и снапшоты без записей `chain-source` (запись не доказана) сюда не попадают: закон инертен, как у вогнутого
    JOIN. Вхождение без рёбер в петлях домена стыка не образует (поток не выходит за границу домена).
    """

    snapshot = context.snapshot
    sectors = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    reflex_pairs = set()
    for relation in snapshot.corner_relations:
        sector = sectors.get(relation.owner_sector_id)
        if sector is not None and sector.ordered_incident_chain_use_ids:
            ids = sector.ordered_incident_chain_use_ids
            reflex_pairs.add((ids[0].value, ids[-1].value))
            reflex_pairs.add((ids[-1].value, ids[0].value))
    ends: dict[str, list[str]] = {}
    starts: dict[str, list[str]] = {}
    for use_id in sorted(present):
        use = uses.get(use_id)
        if use is None:
            continue
        vertices = context.directed_chain_vertices(use)
        ends.setdefault(vertices[-1].value, []).append(use_id)
        starts.setdefault(vertices[0].value, []).append(use_id)
    found = []
    for where in sorted(ends):
        for first_id in ends[where]:
            first = uses[first_id]
            eligible = [
                second_id
                for second_id in starts.get(where, ())
                if second_id != first_id
                and (first_id, second_id) not in reflex_pairs
                and shared_source_lineage(
                    context.chains_by_id[first.physical_chain_id],
                    context.chains_by_id[uses[second_id].physical_chain_id],
                    first.owner_patch_id,
                )
            ]
            if not eligible:
                continue
            if len(eligible) != 1 or len(ends[where]) != 1 or first_id in successors or eligible[0] in successors.values():
                skips.append((where, SKIP_JOIN_CORNER_NOT_ADJACENT))
                continue
            second_id = eligible[0]
            if where in withdrawn:
                skips.append((where, SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT))
                continue
            arms_a = [item for item in (_arm(edge, where) for edge in present[first_id]) if item]
            arms_b = [item for item in (_arm(edge, where) for edge in present[second_id]) if item]
            if len(arms_a) != 1 or len(arms_b) != 1 or arms_a[0][0] != arms_b[0][0]:
                skips.append((where, SKIP_JOIN_CORNER_NOT_ADJACENT))
                continue
            left, right = arms_a[0][1], arms_b[0][1]
            if _dot_g(gram, left, right) >= 0:
                skips.append((where, SKIP_JOIN_BEND_BEYOND_QUARTER_TURN))
                continue
            successors[first_id] = second_id
            kind = "COLLINEAR" if left[0] * right[1] - left[1] * right[0] == 0 else "CONVEX"
            found.append((where, first_id, second_id, kind))
    return found


def _flows(use_ids, successors: dict) -> list:
    """Вхождения, выстроенные в потоки: `[([u1, u2, ...], замкнут), ...]`, детерминированно.

    Потоки без предшественника начинаются с него; остаток (замкнутая цепь,
    нарезанная сплошь на мягкие изломы) — цикл. Цикл размыкается в наименьшем по имени
    вхождении (`замкнут` — истина): счёт `s` обязан где-то начаться заново, как у замкнутой
    цепи без изломов, и угол между последним вхождением и первым — РАЗРЕЗ потока
    (`FLOW_CYCLE_OPENED`), а не потерянный стык: там два набора `(s, r)`, то есть два
    кадра (`OPENING_FRAME_SUFFIX`), и шов один.
    """

    predecessors = {after: before for before, after in successors.items()}
    placed: set = set()
    flows = []
    for start in sorted(use_ids):
        if start in placed or start in predecessors:
            continue
        flow = []
        cursor = start
        while cursor is not None and cursor not in placed:
            placed.add(cursor)
            flow.append(cursor)
            cursor = successors.get(cursor)
        flows.append((flow, False))
    for start in sorted(use_ids):
        if start in placed:
            continue
        flow = []
        cursor = start
        while cursor not in placed:
            placed.add(cursor)
            flow.append(cursor)
            cursor = successors[cursor]
        flows.append((flow, True))
    return flows


def _use_order(context, uses, use_id, loop_edges, skips, unnamed):
    """Рёбра вхождения в порядке обхода либо `None` с названным пропуском."""

    chain_use = uses.get(use_id)
    if chain_use is None:
        ordered, reason = None, SKIP_USE_NOT_IN_SNAPSHOT
    else:
        ordered, reason = _order_or_reason(context, chain_use, loop_edges)
    if ordered is None:
        unnamed.update(item.chain_id for item in loop_edges)
        skips.append((use_id, reason))
    return chain_use, ordered


def chain_station_table(prepared, budget, withdrawn=frozenset()) -> ChainStationTableV1:
    """Таблица станций домена по ГОТОВОЙ подготовке очереди. Точно, под бюджетом.

    Порядок работы: (1) все рёбра всех петель всех регионов собираются со своим
    `ChainUse`, парой вершин и решёточными узлами; (2) вхождения выстраиваются
    в ПОТОКИ — цепочки `ChainUse`, связанные углами JOIN плана
    (`CORNER_JOIN_SOFT_BEND_V1`, `_join_successors`); (3) по каждому вхождению
    рёбра выстраиваются в порядке его обхода, и копится длина `s0` — СКВОЗЬ
    углы JOIN одного потока; (4) подряд идущие рёбра с одним классом прямой
    (`integer_line_class` в направлении `ChainUse`) складываются в пробег. Замкнутая цепь из
    одних мягких изломов — цикл потока: он размыкается в одном месте и имеет ДВА кадра
    (`frame_of_run`), чтобы вершина разреза получила два набора `(s, r)`; разрез назван
    (`cuts`, `STATION_FLOW_CYCLES_OPENED`).

    Бюджет нужен потому, что каждый РАДИКАЛ длины — новый радиканд, то есть
    факторизация; исчерпание поднимает `ExactCanonicalizationWorkBudgetExhausted`
    наверх, и материализатор называет его своим исходом.

    `withdrawn` — вершины стыков потока (закон `CORNER_JOIN_SAME_PCHAIN_V1` и JOIN плана), которые домен снял после
    конфликта станций (`junction_to_withdraw`): такой стык остаётся углом, пропуск назван.
    """

    context = prepared.context
    gram = _gram_of(context.frame)
    scale = int(prepared.lattice.scale) if prepared.lattice is not None else 1
    skips: list[tuple[str, str]] = []
    by_use, corners, node_ids = _collect_loops(prepared, skips)
    uses = {key.value: use for key, use in context.uses_by_id.items()}
    runs: dict[str, StationRunV1] = {}
    edges: dict = {}
    unnamed: set[str] = set()
    restarted: set[str] = set()
    flow_of_run: dict[str, str] = {}
    frame_of_run: dict[str, str] = {}
    joins: dict = {}
    cuts: list = []
    plan_joined: list = []
    successors, outside = _join_successors(context, uses, by_use, skips, frozenset(withdrawn), plan_joined)
    same_chain = _same_chain_successors(context, uses, by_use, skips, successors, gram, frozenset(withdrawn))
    fold_miters = _fold_miters(context, by_use)
    flows = _flows(by_use, successors)
    for flow, closed in flows:
        accumulated = None
        previous_run = None
        flow_key = f"flow:{flow[0]}"
        if closed:
            cuts.append((flow_key, flow[-1], flow[0], uses[flow[0]].physical_chain_id.value))
        for position, use_id in enumerate(flow):
            chain_use, ordered = _use_order(
                context, uses, use_id, by_use[use_id], skips, unnamed
            )
            if ordered is None:
                accumulated, previous_run = None, None
                continue
            chain = context.chains_by_id[chain_use.physical_chain_id]
            if any(item[2] for item in ordered[1:]) or ordered[0][2]:
                restarted.add(ordered[0][0].chain_id)
            use_runs, records, accumulated = _use_runs_from(
                chain_use, chain, ordered, gram, budget, accumulated
            )
            runs.update((item.run_id, item) for item in use_runs)
            edges.update(records)
            if len(flow) > 1:
                frame = flow_key + OPENING_FRAME_SUFFIX if closed and position == 0 else flow_key
                flow_of_run.update((item.run_id, flow_key) for item in use_runs)
                frame_of_run.update((item.run_id, frame) for item in use_runs)
            if previous_run is not None and not ordered[0][2]:
                joins[(previous_run, use_runs[0].run_id)] = use_runs[0].s_origin
            previous_run = use_runs[-1].run_id
    return ChainStationTableV1(
        scale=scale,
        gram=gram,
        runs=runs,
        edges=edges,
        corners={key: tuple(value) for key, value in corners.items()},
        node_vertex_ids=node_ids,
        unnamed_chain_ids=frozenset(unnamed),
        restart_chain_ids=frozenset(restarted),
        counters=(
            ("STATION_RUNS", len(runs)),
            ("STATION_EDGES", len(edges)),
            ("STATION_UNNAMED_CHAINS", len(unnamed)),
            ("STATION_RESTART_CHAINS", len(restarted)),
            ("STATION_SKIPS", len(skips)),
            *(
                (
                    f"STATION_SKIP_{reason}",
                    sum(1 for _where, name in skips if name == reason),
                )
                for reason in STATION_SKIP_REASONS
            ),
            ("STATION_FLOWS", sum(1 for flow, _closed in flows if len(flow) > 1)),
            ("STATION_JOIN_CORNERS", len(joins)),
            ("STATION_FLOW_CYCLES_OPENED", len(cuts)),
            ("STATION_JOIN_CORNERS_OUT_OF_DOMAIN", outside),
            ("STATION_SAME_PCHAIN_JOINS", len(same_chain)),
            ("STATION_FOLD_MITER_CORNERS", len(fold_miters)),
        ),
        skips=tuple(skips),
        flow_of_run=flow_of_run,
        joins=joins,
        frame_of_run=frame_of_run,
        cuts=tuple(cuts),
        same_chain_joins=tuple(same_chain),
        plan_joins=tuple(plan_joined),
        fold_miters=fold_miters,
    )


def junction_to_withdraw(table: ChainStationTableV1, run_ids) -> str | None:
    """Вершина стыка потока (закон `CORNER_JOIN_SAME_PCHAIN_V1` либо JOIN плана), которую снимает конфликт станций пробегов `run_ids`, либо `None`.

    Конфликт: вершина региона получила от нескольких пробегов ОДНОГО потока ответы, которые не сводятся
    ни перекладиной стыка, ни ребром (узел события скелета, где сошлись биссектрисы трёх кусков). Снимается один стык
    за раз, детерминированно: сначала входящий в вхождение НОВОГО пробега, затем исходящий из него, затем те же у
    прежних. Сперва — стыки без записи угла (`same_chain_joins`), и только если такого стыка у пробегов конфликта
    нет, — стык JOIN плана (`plan_joins`): у него угол и митра остаются, снимается лишь непрерывность `u`, как у
    любого пропуска плана (`JOIN_CORNER_NOT_ADJACENT`). Домен не отказывает из-за стыка потока.
    """

    for joins in (table.same_chain_joins, table.plan_joins):
        entering = {item[2]: item[0] for item in joins}
        leaving = {item[1]: item[0] for item in joins}
        for run_id in run_ids:
            run = table.runs.get(run_id)
            if run is None:
                continue
            for index in (entering, leaving):
                if run.chain_use_id in index:
                    return index[run.chain_use_id]
    return None


def station_of(run: StationRunV1, point) -> SqrtSumV1:
    """`s` точки относительно пробега, в единицах решётки. ТОЧНО.

    `s = s_origin + <p - origin, direction>_G / |direction|_G`. Всё, кроме
    последнего произведения, — рациональная линейная комбинация координат;
    произведение двух `SqrtSumV1` замкнуто (`__mul__` на gcd) и новых радикандов
    не рождает, поэтому бюджет здесь не нужен.
    """

    relative_x = point[0] - SqrtSumV1.rational(run.origin[0])
    relative_y = point[1] - SqrtSumV1.rational(run.origin[1])
    projection = relative_x.scaled(run.covector[0]) + relative_y.scaled(
        run.covector[1]
    )
    return run.s_origin + projection * run.inverse_length


def transverse_root(line, budget) -> SqrtSumV1:
    """`sqrt(q) / q` несущей прямой: множитель времени прихода. `q > 0`."""

    return SqrtSumV1.radical(1, Fraction(1) / Fraction(line.q), budget)


def transverse_of(line, point, root: SqrtSumV1) -> SqrtSumV1:
    """`r = (a x + b y - c) / sqrt(q)` точки грани, в единицах решётки. ТОЧНО.

    Это ВРЕМЯ ПРИХОДА фронта несущей прямой: на самой прямой оно ноль, а на
    фронте при alpha равно `lattice_alpha` ровно (`coverage_at` режет той же
    формулой). Деление заменено умножением на `root = sqrt(q)/q`, чтобы не
    звать сопряжение.
    """

    numerator = (
        point[0].scaled(line.a)
        + point[1].scaled(line.b)
        - SqrtSumV1.rational(line.c)
    )
    return numerator * root
