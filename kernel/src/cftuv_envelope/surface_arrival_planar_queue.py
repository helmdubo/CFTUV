"""Первый бэкенд-адаптер: планарный результат — в surface-контракте, без потерь.

Зачем он существует. `SurfaceArrivalComplexV1` объявляет форму ответа для
непланарных бэкендов, которых ещё нет. Форма, у которой нет ни одного
производителя, не проверяется ничем и потому является слухом. Единственный
производитель, который УЖЕ доказан, — планарная очередь (`wavefront/**`,
P0-1..P0-3), и требование к контракту поэтому формулируется исполняемо:

    планарный ответ обязан выражаться в surface-контракте так, чтобы из
    ОДНИХ ТОЛЬКО БАЙТ КОМПЛЕКСА восстанавливались все девять осей P0-3.

Отсюда `rebuild_face_partition`: комплекс возвращается в `FacePartitionV1`, и
покрытие считается ТЕМ ЖЕ `coverage_at`, что у эталонной колонки. Если бы
адаптер считал покрытие своим кодом, сверка проверяла бы согласие двух
резаков, а не полноту записи. Круг замыкается на контракте, а не на арифметике.

ЧТО ЗДЕСЬ НАЗВАНО ЯВНО, потому что иначе выбор выглядел бы фактом:

* ЯЧЕЙКА ПЛАНАРНОЙ РЕДУКЦИИ — ОБЛАСТЬ ПОСТОЯННОГО ВЛАДЕЛЬЦА, а не треугольник.
  Планарный домен не триангулирован, и придумывать ему триангуляцию значило бы
  вносить данные, которых нет. Поэтому `cell_law` объявлен
  `PLANAR_SINGLE_CHART_REGION_V1`, а адрес ячейки выводится из владельца: в
  одной плоской карте это ОДНО И ТО ЖЕ множество, и ссылаться на него двумя
  разными именами было бы вымыслом. На меше адрес — треугольник, и владельцев
  в нём несколько; именно это различие `cell_law` и называет.
* СЧЁТЧИКИ КОМПЛЕКСА — О КОМПЛЕКСЕ, А НЕ О ПОИСКЕ. Копировать счётчики скелета
  нельзя: `split_candidates_examined` у MOTORCYCLE и EXHAUSTIVE РАЗНЫЕ по
  построению (на этом стоит тест строгого отсечения P0-3), и два доказанно
  равных ответа различались бы числом попыток. Здесь считается содержимое
  ответа, и оно у обоих режимов совпадает.
* ПОРЯДОК КАНОНИЧЕН, А НЕ ВХОДНОЙ. События и доказательные долги
  упорядочиваются по собственной канонической записи, потому что порядок узлов
  у очереди — следствие обхода контура. Множества (посевы, ячейки, фрагменты,
  cut locus) хранятся `frozenset` и порядка не имеют вовсе. Ровно этим ворота
  G-N3 («перестановка входа даёт байт-в-байт тот же комплекс») становятся
  утверждением о комплексе, а не о везении.
* СТАНЦИИ КАНДИДАТА — ТОЛЬКО КОНТАКТЫ CUT LOCUS. Концы пролёта посева лежат в
  `station_interval` самого посева, и второй раз в свидетелях они не
  записываются: одна величина — одно место.
* ВРЕМЯ ХРАНИТСЯ КАНОНИЧЕСКИМ. `EventTimeV1.canonical()` снимает рациональный
  произвол пары `(dividend, divisor)`; без него одно и то же время, найденное
  разными тройками рёбер, дало бы разные байты.

Стены (`q = 0`) объявлены посевами вида `BOUNDARY_CONDITION`, но граней не
получают — их не получает и планарная очередь: неподвижный фронт не заметает
ничего. Посевом стена остаётся потому, что она УЧАСТВУЕТ в событиях и делит
cut locus; выбросить её из посевов значило бы получить владельца, которого нет
среди источников.
"""

from __future__ import annotations

from fractions import Fraction
import json

from .contracts.metric import ExactPoint2V1, ExactRationalV1
from .contracts.surface_metric_v2 import SurfaceMetricPrecisionTierV1
from .contracts.surface_arrival import (
    PLANAR_DIRECT_PATH_HISTORY,
    SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
    ArrivalCandidateV1,
    ArrivalCellId,
    ArrivalCellLawV1,
    ArrivalCellV1,
    ArrivalCounterV1,
    ArrivalDomainLawV1,
    ArrivalDomainV1,
    ArrivalEventV1,
    ArrivalLawV1,
    ArrivalMetricDigestValue,
    ArrivalOwnerKeyV1,
    ArrivalPathHistoryKeyV1,
    ArrivalProofObligationV1,
    ArrivalSeedKindV1,
    ArrivalSeedV1,
    ArrivalSupportLineV1,
    ArrivalTieResolutionV1,
    CutLocusPointV1,
    ExactAlgebraicPointV1,
    ExactAlgebraicSumV1,
    ExactAlgebraicTimeV1,
    ExactArrivalPrecisionV1,
    ExactRadicalTermV1,
    OwnerFragmentV1,
    PlanarChartRefV1,
    SeedStationIntervalV1,
    StationWitnessKindV1,
    StationWitnessV1,
    SurfaceArrivalBackendV1,
    SurfaceArrivalComplexV1,
    arrival_metric_digest,
    exact_rational,
)
from .ids import LawId, PatchDomainId, SourceRevision
from .wavefront.event_time import EventPointV1, EventTimeV1, SupportLineV1
from .wavefront.events import EventKind
from .wavefront.faces import (
    FaceOutcome,
    FacePartitionV1,
    FaceV1,
    edge_key,
    fan_edge_key,
)
from .wavefront.skeleton import SkeletonNodeV1
from .wavefront.sqrt_sum import SqrtSumV1


#: Префикс адреса ячейки планарной карты. Он же — обещание, что за адресом НЕ
#: стоит треугольник меша: `cell_law` говорит то же самое типом, префикс —
#: глазами читателя лога.
PLANAR_CHART_CELL_PREFIX = "planar_chart_region:"

#: Планарная очередь точна по построению: ни одного порога, ни одного float
#: (P0-1..P0-3). Класс точности поэтому `REFERENCE_EXACT_SMALL`, а ε — `None`,
#: и это ровно та же семантика, что у `SurfaceMetricDescriptorV2`.
PLANAR_QUEUE_PRECISION_TIER = SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL


# --------------------------------------------------------------------------
# Перевод чисел: ядерная арифметика <-> записи контракта
# --------------------------------------------------------------------------


def to_algebraic_sum(value: SqrtSumV1) -> ExactAlgebraicSumV1:
    """`SqrtSumV1` в запись контракта. Канонический набор сохраняется как есть."""

    return ExactAlgebraicSumV1(
        tuple(
            ExactRadicalTermV1(radicand, exact_rational(coefficient))
            for radicand, coefficient in value.terms
        )
    )


def from_algebraic_sum(record: ExactAlgebraicSumV1) -> SqrtSumV1:
    """Обратный перевод. Биекция: канонический набор единствен у обоих."""

    return SqrtSumV1(
        tuple(
            (
                term.radicand,
                Fraction(term.coefficient.numerator, term.coefficient.denominator),
            )
            for term in record.terms
        )
    )


def to_algebraic_point(point) -> ExactAlgebraicPointV1:
    return ExactAlgebraicPointV1(
        to_algebraic_sum(point[0]), to_algebraic_sum(point[1])
    )


def from_algebraic_point(record: ExactAlgebraicPointV1):
    return (from_algebraic_sum(record.x), from_algebraic_sum(record.y))


def to_algebraic_time(time: EventTimeV1) -> ExactAlgebraicTimeV1:
    """Время КАНОНИЧЕСКИМ: одна величина — один набор байт."""

    canonical = time.canonical()
    return ExactAlgebraicTimeV1(
        exact_rational(canonical.dividend), to_algebraic_sum(canonical.divisor)
    )


def from_algebraic_time(record: ExactAlgebraicTimeV1) -> EventTimeV1:
    return EventTimeV1(
        Fraction(record.dividend.numerator, record.dividend.denominator),
        from_algebraic_sum(record.divisor),
    )


def _rational_point(point: tuple[int, int]) -> ExactPoint2V1:
    return ExactPoint2V1(exact_rational(point[0]), exact_rational(point[1]))


def _integer_point(record: ExactPoint2V1) -> tuple[int, int]:
    for item in (record.x, record.y):
        if item.denominator != 1:
            raise ValueError("узел планарной карты — целочисленный")
    return (record.x.numerator, record.y.numerator)


# --------------------------------------------------------------------------
# Ключи владельцев
# --------------------------------------------------------------------------


def owner_key_of(key: tuple[int, ...]) -> ArrivalOwnerKeyV1:
    """Ключ вхождения ребра — в непрозрачный ключ владельца, обратимо."""

    return ArrivalOwnerKeyV1(json.dumps(list(key), separators=(",", ":")))


def edge_key_of(owner: ArrivalOwnerKeyV1) -> tuple[int, ...]:
    return tuple(json.loads(owner.value))


def _canonical_loop(points: tuple[tuple[int, int], ...]) -> list[list[int]]:
    """Петля с канонического начала. Циклический сдвиг — НЕ другая петля.

    Закон назван, а не подразумевается, и это прямой урок S0: два независимых
    вывода одной величины совпали как множества и разошлись как кортежи ровно
    потому, что начало обхода никто не объявлял. Ориентацию нормирует сама
    `PolygonV1.build`, начало нормируется здесь — наименьшей точкой.
    """

    start = points.index(min(points))
    return [list(point) for point in points[start:] + points[:start]]


def chart_digest(polygon) -> ArrivalMetricDigestValue:
    """Дайджест планарной карты: ровно тот вход, который фиксирует ответ.

    Считается по петлям, скоростям рёбер и опорам вееров — трём величинам, от
    которых зависит каждый узел скелета. Дайджест снапшота здесь не при чём:
    планарная карта в него не входит вовсе.

    Внешняя петля и дыры записаны ПОРОЗНЬ и дыры отсортированы: дыра не
    становится внешней границей от того, что её записали первой, а порядок
    перечисления дыр входом не задан вовсе.
    """

    payload = {
        "outer": _canonical_loop(polygon.outer.points),
        "holes": sorted(
            _canonical_loop(hole.points) for hole in polygon.holes
        ),
        "speeds": sorted(
            [
                [
                    list(start),
                    list(end),
                    [Fraction(speed).numerator, Fraction(speed).denominator],
                ]
                for start, end, speed in polygon.edges()
            ]
        ),
        "fans": sorted(
            [
                [
                    list(point),
                    ordinal,
                    [line.a, line.b, line.c],
                    [Fraction(line.q).numerator, Fraction(line.q).denominator],
                ]
                for point, ordinal, line in polygon.fan_edges()
            ]
        ),
    }
    return arrival_metric_digest(
        json.dumps(payload, sort_keys=True, separators=(",", ":"))
    )


# --------------------------------------------------------------------------
# Сборка комплекса
# --------------------------------------------------------------------------


def _seeds(polygon) -> frozenset[ArrivalSeedV1]:
    seeds: list[ArrivalSeedV1] = []
    for start, end, speed in polygon.edges():
        kind = (
            ArrivalSeedKindV1.PARALLEL_SEGMENT
            if speed > 0
            else ArrivalSeedKindV1.BOUNDARY_CONDITION
        )
        seeds.append(
            ArrivalSeedV1(
                kind,
                owner_key_of(edge_key(start, end)),
                SeedStationIntervalV1(
                    _rational_point(start), _rational_point(end)
                ),
            )
        )
    for point, ordinal, _line in polygon.fan_edges():
        seeds.append(
            ArrivalSeedV1(
                ArrivalSeedKindV1.DIRECTIONAL_PARALLEL,
                owner_key_of(fan_edge_key(point, ordinal)),
                SeedStationIntervalV1(
                    _rational_point(point), _rational_point(point)
                ),
            )
        )
    return frozenset(seeds)


def _cell_id(owner: ArrivalOwnerKeyV1) -> ArrivalCellId:
    return ArrivalCellId(PLANAR_CHART_CELL_PREFIX + owner.value)


def _cell_of(face: FaceV1) -> ArrivalCellV1:
    owner = owner_key_of(face.owner)
    witnesses = tuple(
        StationWitnessV1(
            StationWitnessKindV1.CUT_LOCUS_CONTACT, to_algebraic_point(point)
        )
        for point in face.points[2:]
    )
    candidate = ArrivalCandidateV1(
        owner,
        ArrivalPathHistoryKeyV1(PLANAR_DIRECT_PATH_HISTORY),
        ArrivalLawV1.PARALLEL_OFFSET_V1,
        witnesses,
        ArrivalDomainV1(ArrivalDomainLawV1.WHOLE_CELL_V1, ()),
        ExactArrivalPrecisionV1(),
    )
    return ArrivalCellV1(
        _cell_id(owner), (candidate,), ArrivalTieResolutionV1.RESOLVED_EXACT
    )


def _fragment_of(face: FaceV1) -> OwnerFragmentV1:
    line = face.line
    if line is None:
        raise ValueError(f"у грани {face.owner} нет несущей прямой")
    owner = owner_key_of(face.owner)
    return OwnerFragmentV1(
        owner,
        _cell_id(owner),
        ArrivalSupportLineV1(line.a, line.b, line.c, exact_rational(line.q)),
        _rational_point(face.source_start),
        _rational_point(face.source_end),
        tuple(to_algebraic_point(point) for point in face.points),
        to_algebraic_sum(face.doubled_area),
    )


def _cut_locus(skeleton) -> frozenset[CutLocusPointV1]:
    """Место И МОМЕНТ встречи, с объединением владельцев всех записей в нём.

    Группировка обязательна: в одной точке креста стоит до пяти записей, а
    делят её всё те же владельцы. Без объединения cut locus считал бы записи
    очереди, а не места встречи.
    """

    grouped: dict[tuple, set[tuple[int, ...]]] = {}
    places: dict[tuple, tuple] = {}
    for node in skeleton.nodes:
        time = node.time.canonical()
        key = (
            time.dividend,
            time.divisor.terms,
            node.point.x.terms,
            node.point.y.terms,
        )
        grouped.setdefault(key, set()).update(node.participants)
        places[key] = (node.point, time)
    return frozenset(
        CutLocusPointV1(
            to_algebraic_point((places[key][0].x, places[key][0].y)),
            to_algebraic_time(places[key][1]),
            tuple(
                sorted(
                    (owner_key_of(item) for item in owners),
                    key=lambda owner: owner.value,
                )
            ),
        )
        for key, owners in grouped.items()
    )


def _event_of(node: SkeletonNodeV1) -> ArrivalEventV1:
    return ArrivalEventV1(
        LawId(node.kind.value),
        to_algebraic_time(node.time),
        to_algebraic_point((node.point.x, node.point.y)),
        tuple(owner_key_of(item) for item in node.participants),
        node.converging_vertices,
        tuple(LawId(item.value) for item in node.kinds),
        tuple(
            tuple(owner_key_of(item) for item in incidence)
            for incidence in node.incidences
        ),
    )


def _event_sort_key(event: ArrivalEventV1) -> str:
    return json.dumps(
        [
            event.kind.value,
            [event.time.dividend.numerator, event.time.dividend.denominator],
            _sum_record(event.time.divisor),
            _sum_record(event.place.x),
            _sum_record(event.place.y),
            [item.value for item in event.participants],
            event.converging_vertices,
            [item.value for item in event.kinds],
            [[item.value for item in row] for row in event.incidences],
        ],
        sort_keys=True,
        separators=(",", ":"),
    )


def _sum_record(value: ExactAlgebraicSumV1) -> list:
    return [
        [term.radicand, term.coefficient.numerator, term.coefficient.denominator]
        for term in value.terms
    ]


def _obligation_of(obligation) -> ArrivalProofObligationV1:
    return ArrivalProofObligationV1(
        LawId(obligation.cause.value),
        obligation.disposition,
        to_algebraic_time(obligation.level),
        tuple(owner_key_of(item) for item in obligation.participant_edge_keys),
        tuple(owner_key_of(item) for item in obligation.target_edge_keys),
        None if obligation.event_kind is None else LawId(obligation.event_kind.value),
    )


def _obligation_sort_key(obligation: ArrivalProofObligationV1) -> str:
    return json.dumps(
        [
            [
                obligation.level.dividend.numerator,
                obligation.level.dividend.denominator,
            ],
            _sum_record(obligation.level.divisor),
            obligation.cause.value,
            obligation.disposition.value,
            [item.value for item in obligation.participant_edge_keys],
            [item.value for item in obligation.target_edge_keys],
            "" if obligation.event_kind is None else obligation.event_kind.value,
        ],
        sort_keys=True,
        separators=(",", ":"),
    )


def _counters(
    seeds, cells, fragments, cut_locus, events, obligations
) -> tuple[ArrivalCounterV1, ...]:
    """Счёт СОДЕРЖИМОГО ответа. О том, как его искали, здесь нет ничего."""

    values = {
        "ARRIVAL_CANDIDATES": sum(len(cell.candidates) for cell in cells),
        "ARRIVAL_CELLS": len(cells),
        "ARRIVAL_CUT_LOCUS_POINTS": len(cut_locus),
        "ARRIVAL_EVENTS": len(events),
        "ARRIVAL_MULTIWAY_CELLS": sum(
            cell.tie_resolution is ArrivalTieResolutionV1.MULTIWAY_PRESERVED
            for cell in cells
        ),
        "ARRIVAL_OWNER_FRAGMENTS": len(fragments),
        "ARRIVAL_PROOF_OBLIGATIONS": len(obligations),
        "ARRIVAL_SEEDS": len(seeds),
        "ARRIVAL_STATION_WITNESSES": sum(
            len(candidate.station_witnesses)
            for cell in cells
            for candidate in cell.candidates
        ),
    }
    return tuple(
        ArrivalCounterV1(LawId(name), values[name]) for name in sorted(values)
    )


def build_planar_queue_arrival_complex(
    polygon,
    skeleton,
    partition: FacePartitionV1,
    *,
    patch_domain_id: PatchDomainId,
    source_revision: SourceRevision,
    alpha_horizon: ExactRationalV1,
) -> SurfaceArrivalComplexV1:
    """Комплекс прибытия ИЗ результата планарной очереди. Ничего не считает.

    Функция только ПЕРЕКЛАДЫВАЕТ уже доказанный ответ в форму контракта: ни
    одного геометрического предиката, ни одной новой величины. Именно поэтому
    расхождение колонок в воротах означало бы потерю записи, а не второй
    алгоритм, который «посчитал иначе».
    """

    seeds = _seeds(polygon)
    cells = frozenset(_cell_of(face) for face in partition.faces)
    fragments = frozenset(_fragment_of(face) for face in partition.faces)
    cut_locus = _cut_locus(skeleton)
    events = tuple(
        sorted(
            (_event_of(node) for node in skeleton.nodes), key=_event_sort_key
        )
    )
    obligations = tuple(
        sorted(
            (_obligation_of(item) for item in skeleton.proof_obligations),
            key=_obligation_sort_key,
        )
    )
    return SurfaceArrivalComplexV1(
        schema_version=SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        backend=SurfaceArrivalBackendV1.PLANAR_QUEUE,
        metric_ref=PlanarChartRefV1(
            source_revision, patch_domain_id, chart_digest(polygon)
        ),
        adjacency_ref=None,
        cell_law=ArrivalCellLawV1.PLANAR_SINGLE_CHART_REGION_V1,
        alpha_horizon=alpha_horizon,
        seeds=seeds,
        cells=cells,
        cut_locus=cut_locus,
        owner_fragments=fragments,
        events=events,
        domain_doubled_area=to_algebraic_sum(
            SqrtSumV1.rational(partition.polygon_doubled_area)
        ),
        front_outcome=skeleton.outcome,
        ownership_outcome=LawId(partition.outcome.value),
        proof_status=skeleton.proof_status,
        proof_obligations=obligations,
        precision_tier=PLANAR_QUEUE_PRECISION_TIER,
        named_epsilon=None,
        counters=_counters(
            seeds, cells, fragments, cut_locus, events, obligations
        ),
    )


# --------------------------------------------------------------------------
# Обратный путь: комплекс -> разбиение граней
# --------------------------------------------------------------------------


def _face_of(fragment: OwnerFragmentV1) -> FaceV1:
    q = Fraction(fragment.support.q.numerator, fragment.support.q.denominator)
    return FaceV1(
        edge_key_of(fragment.owner_key),
        _integer_point(fragment.span_start),
        _integer_point(fragment.span_end),
        tuple(from_algebraic_point(point) for point in fragment.contour),
        from_algebraic_sum(fragment.doubled_area),
        SupportLineV1(
            fragment.support.a,
            fragment.support.b,
            fragment.support.c,
            int(q) if q.denominator == 1 else q,
        ),
    )


def rebuild_face_partition(
    complex_: SurfaceArrivalComplexV1,
) -> FacePartitionV1:
    """Разбиение граней ОБРАТНО из байт комплекса, без единого нового числа.

    Порядок граней канонический (по ключу владельца), а не входной: множество
    фрагментов порядка не имеет, а все три величины ниже — сумма площадей,
    покрытие и счёт владельцев — от порядка не зависят. `polygon_doubled_area`
    берётся из `domain_doubled_area`: у планарной карты это целое, и дробный
    остаток означал бы, что область записана не той величиной.
    """

    faces = tuple(
        _face_of(fragment)
        for fragment in sorted(
            complex_.owner_fragments, key=lambda item: item.owner_key.value
        )
    )
    total = SqrtSumV1.zero()
    for face in faces:
        total = total + face.doubled_area
    area = from_algebraic_sum(complex_.domain_doubled_area)
    rational = area.as_rational()
    if rational is None or rational.denominator != 1:
        raise ValueError("площадь планарной области — целое число")
    return FacePartitionV1(
        FaceOutcome(complex_.ownership_outcome.value),
        faces,
        total,
        int(rational),
    )


def rebuild_skeleton_node(event: ArrivalEventV1) -> SkeletonNodeV1:
    """Узел скелета обратно из события комплекса — для `digest.node_record`.

    Восстановление нужно затем, чтобы ось `node_record` считалась ТОЙ ЖЕ
    функцией ядра, а не её копией: копия сверяла бы две копии, а не запись.
    """

    return SkeletonNodeV1(
        EventKind(event.kind.value),
        from_algebraic_time(event.time),
        EventPointV1(
            from_algebraic_sum(event.place.x), from_algebraic_sum(event.place.y)
        ),
        tuple(edge_key_of(item) for item in event.participants),
        event.converging_vertices,
        tuple(EventKind(item.value) for item in event.kinds),
        tuple(
            tuple(edge_key_of(item) for item in incidence)
            for incidence in event.incidences
        ),
    )


__all__ = (
    "PLANAR_CHART_CELL_PREFIX",
    "build_planar_queue_arrival_complex",
    "chart_digest",
    "edge_key_of",
    "from_algebraic_point",
    "from_algebraic_sum",
    "from_algebraic_time",
    "owner_key_of",
    "rebuild_face_partition",
    "rebuild_skeleton_node",
    "to_algebraic_point",
    "to_algebraic_sum",
    "to_algebraic_time",
)
