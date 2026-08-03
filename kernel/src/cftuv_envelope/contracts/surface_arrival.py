"""Комплекс прибытия на поверхности: что посчитано ДО alpha и чем это доказано.

Власть записана в `DECISIONS.md` за 2026-08-03 («AUTH развилок S0/S2») и её
приёмкой того же дня. Комплекс — ПРЕДВЫЧИСЛЕНИЕ до `alpha_horizon`; drag только
РЕЖЕТ уже посчитанное, а запрос сверх горизонта — именованный отказ, а не
молчаливое досчитывание. Здесь описано, ЧТО записано; считают бэкенды.

Четыре решения S2 живут в формах ниже, и каждое стоило бы иначе:

* ТОЧНОСТЬ — ОТДЕЛЬНАЯ ОСЬ. `ProofStatus` остаётся двоичным (тип P0-1, взят
  импортом и НЕ расширен), а «чем доказано» едет в `precision_tier` +
  `named_epsilon`. Оба поля — ТЕ ЖЕ, что у `SurfaceMetricDescriptorV2`: одна
  семантика `None ⇔ REFERENCE_EXACT_SMALL`, один тип ε, одна таблица классов.
  Второй набор значений означал бы, что «точно» у метрики и «точно» у комплекса
  выясняются по разным правилам.
* НИЧЬЯ СОХРАНЯЕТСЯ. `MULTIWAY_PRESERVED` несёт ДВУХ И БОЛЕЕ кандидатов до
  атомарного резолвера; float-политика 2014 года отвергнута ранее, и выбор
  «одного из равных» здесь не делается вовсе. `UNDECIDED_FAIL_CLOSED` — тоже
  минимум двое: «не разделилось» между одним кандидатом бессмысленно.
* СТАНЦИИ — КОРТЕЖ СВИДЕТЕЛЕЙ. Место, где владение кончилось встречей, входит
  в кандидата кортежем `station_witnesses`, а не одним «ближайшим»: у прихода,
  обогнувшего конусную вершину, свидетелей столько, сколько станций на пути.
* ССЫЛКИ ПО ДАЙДЖЕСТУ. `metric_ref` и `adjacency_ref` — не доверие, а
  сравнение. Комплекс, построенный на одной таблице смежности и приложенный к
  другой, есть неверный ответ без отказа, поэтому расхождение ссылок —
  именованный отказ (`METRIC_REF_ADJACENCY_DISAGREEMENT`).

ДВЕ ВЛАСТИ МЕТРИКИ, А НЕ ОДНА, и это следствие закона S0, а не послабление.
`SurfaceMetricDescriptorV2.__post_init__` ОТКАЗЫВАЕТ на `SurfaceRegime.PLANAR`:
у планарного режима своя решётко-точная `RationalAffinePlanarMetricV2`. Значит
планарный домен не может сослаться на V2 вовсе, и `metric_ref` обязан быть
объединением: `SurfaceMetricRefV1` для меша, `PlanarChartRefV1` для одной
плоской карты. У планарной карты таблицы смежности треугольников НЕ
СУЩЕСТВУЕТ — не «пока нет», а нет предмета, — поэтому `adjacency_ref` под ней
обязан быть `None`, и непустая ссылка там такой же именованный отказ, как
пустая под мешем. Обе двери fail-closed, ни одной молчаливой.

ЧУЖИЕ СЛОВАРИ НЕ РАЗДВАИВАЮТСЯ. Правило одно и держится по всему файлу:
словарь, которым владеет ЭТОТ контракт, — перечисление; словарь, которым
владеет производящий движок, несётся ДОСЛОВНО в `LawId`. Поэтому виды событий,
исход материализации владения и причина доказательного долга — `LawId`
(их значения принадлежат `wavefront/**`, который эта карточка не двигает ни
строкой), а `ProofStatus`, `ProofObligationDisposition` и `SkeletonOutcome`
взяты импортом: они уже перечисления и уже принадлежат P0-1/P0-3. Копия чужого
перечисления здесь означала бы два места, где решается, что такое `EDGE`.

ЧИСЛА ЗАПИСАНЫ СВОИМИ ЗАПИСЯМИ, А НЕ `Fraction`. Схема контракта рендерится
`json_schema_for`, а он `Fraction` не описывает вовсе; `ProofObligationV1` не
рендерится и подавно (его `cause` объявлен под `TYPE_CHECKING`, и
`get_type_hints` падает `NameError`). Починка потребовала бы правки
`wavefront/**`, запрещённой P0-3. Поэтому долг едет СТАБИЛЬНОЙ ПРОЕКЦИЕЙ — той
самой, которую P0-3 уже объявил стабильной, — а алгебраические величины лежат
в `ExactAlgebraicSumV1`: тот же канонический набор `(радиканд, коэффициент)`,
что у `SqrtSumV1`, только записями контракта.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from hashlib import sha256
from typing import Any

from ..codec import canonical_json_bytes
from ..ids import (
    LawId,
    OpaqueId,
    PatchDomainId,
    SourceRevision,
)
from ..numeric import CertifiedDecimalIntervalV1
from ..wavefront.proof import ProofObligationDisposition, ProofStatus
from ..wavefront.superlevel import SkeletonOutcome
from .analysis import SurfaceRegime
from .metric import ExactPoint2V1, ExactRationalV1
from .surface_metric_v2 import (
    NamedEpsilonV1,
    SurfaceAdjacencyRefV1,
    SurfaceMetricPrecisionTierV1,
)


SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA = "cftuv.envelope.surface_arrival_complex.v1"

#: Путь прихода в ОДНОЙ плоской карте. Разворотов нет, обходов нет, история
#: пути пуста — и записана именем, а не пустой строкой: «истории нет» и «историю
#: забыли записать» обязаны различаться.
PLANAR_DIRECT_PATH_HISTORY = "PLANAR_DIRECT_V1"


@dataclass(frozen=True, slots=True)
class ArrivalMetricDigestValue(OpaqueId):
    """sha256 канонических байт объявленной власти метрики."""


@dataclass(frozen=True, slots=True)
class ArrivalOwnerKeyV1(OpaqueId):
    """Ключ владельца: чей фронт пришёл. Он же ключ участника события.

    Владелец и участник — ОДНО, и типов поэтому один, а не два. У планарного
    движка участник узла скелета есть ключ вхождения ребра, а грань этого же
    вхождения — его владение; развести их значило бы завести два имени одной
    величины и потом сверять их между собой.
    """


@dataclass(frozen=True, slots=True)
class ArrivalPathHistoryKeyV1(OpaqueId):
    """Каким путём этот посев сюда пришёл.

    Ключ обязателен, а не вычислим из владельца: на поверхности ОДИН посев
    приходит в одну ячейку несколькими развёртками, и это РАЗНЫЕ кандидаты с
    разными значениями. Без истории пути они слились бы в одного, и минимум
    брался бы по обрезанному множеству.
    """


@dataclass(frozen=True, slots=True)
class ArrivalCellId(OpaqueId):
    """Адрес ячейки комплекса. Что это за адрес — говорит `cell_law`."""


class SurfaceArrivalBackendV1(str, Enum):
    """Чем посчитан комплекс. Бэкенд объявляется, а не угадывается по данным."""

    PLANAR_QUEUE = "PLANAR_QUEUE"
    DEVELOPABLE_UNFOLD = "DEVELOPABLE_UNFOLD"
    POLYHEDRAL_WINDOW = "POLYHEDRAL_WINDOW"
    MULTILABEL_FMM = "MULTILABEL_FMM"


class ArrivalCellLawV1(str, Enum):
    """Что именно адресует `ArrivalCellId`. Молчаливой проекции здесь нет.

    `MESH_TRIANGLE_V1` — значение адреса ЕСТЬ значение `SurfaceTriangleId`
    таблицы смежности; кросс-валидатор проверяет это сравнением.
    `PLANAR_SINGLE_CHART_REGION_V1` — область ОДНОЙ плоской карты, треугольника
    за ней нет вовсе. Назвать её треугольником значило бы выдать выбор
    представления за факт меша — ровно то «молчаливое проецирование», против
    которого заведены ворота плоской редукции.
    """

    MESH_TRIANGLE_V1 = "MESH_TRIANGLE_V1"
    PLANAR_SINGLE_CHART_REGION_V1 = "PLANAR_SINGLE_CHART_REGION_V1"


class ArrivalSeedKindV1(str, Enum):
    """Чем задан посев. Четвёртого способа задать источник фронта нет."""

    PARALLEL_SEGMENT = "PARALLEL_SEGMENT"
    DIRECTIONAL_PARALLEL = "DIRECTIONAL_PARALLEL"
    POINT = "POINT"
    BOUNDARY_CONDITION = "BOUNDARY_CONDITION"


class ArrivalLawV1(str, Enum):
    """Каким законом кандидат приходит в свою область.

    Закон и вид посева НЕ связаны взаимно однозначно, и связывать их проверкой
    было бы ошибкой: посев-отрезок после разворота через конусную вершину даёт
    РАДИАЛЬНЫЙ приход вокруг её образа. Проверяется поэтому только то, что
    владелец кандидата действительно объявлен посевом.
    """

    PARALLEL_OFFSET_V1 = "PARALLEL_OFFSET_V1"
    RADIAL_FROM_STATION_V1 = "RADIAL_FROM_STATION_V1"
    STATIONARY_BOUNDARY_V1 = "STATIONARY_BOUNDARY_V1"


class ArrivalTieResolutionV1(str, Enum):
    """Что стало с ничьёй в ячейке. Тихого выбора одного из равных нет."""

    RESOLVED_EXACT = "RESOLVED_EXACT"
    MULTIWAY_PRESERVED = "MULTIWAY_PRESERVED"
    UNDECIDED_FAIL_CLOSED = "UNDECIDED_FAIL_CLOSED"


class ArrivalDomainLawV1(str, Enum):
    """Чем задана область кандидата внутри его ячейки."""

    WHOLE_CELL_V1 = "WHOLE_CELL_V1"
    BARYCENTRIC_REGION_V1 = "BARYCENTRIC_REGION_V1"


class StationWitnessKindV1(str, Enum):
    """Почему станция названа станцией. Причины не сливаются.

    `SEED_SPAN_CONTACT` — путь вышел из самого посева. `SIDE_CROSSING` — путь
    пересёк сторону треугольника и продолжился в соседнем. `CONE_VERTEX` — путь
    обогнул вершину с конусным углом, то есть переизлучился. `CUT_LOCUS_CONTACT`
    — владение кончилось встречей с чужим фронтом.
    """

    SEED_SPAN_CONTACT = "SEED_SPAN_CONTACT"
    SIDE_CROSSING = "SIDE_CROSSING"
    CONE_VERTEX = "CONE_VERTEX"
    CUT_LOCUS_CONTACT = "CUT_LOCUS_CONTACT"


class SurfaceArrivalContractError(ValueError):
    """Инвариант комплекса прибытия нарушен, и нарушение НАЗВАНО."""

    def __init__(self, invariant: str, details: str) -> None:
        self.invariant = str(invariant)
        self.details = str(details)
        super().__init__(f"SURFACE_ARRIVAL_INVALID:{self.invariant}: {details}")


def _fail(invariant: str, details: str) -> None:
    raise SurfaceArrivalContractError(invariant, details)


# --------------------------------------------------------------------------
# Точные числа контракта
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ExactRadicalTermV1:
    """`coefficient * sqrt(radicand)`; радиканд бесквадратный и положительный."""

    radicand: int
    coefficient: ExactRationalV1

    def __post_init__(self) -> None:
        if type(self.radicand) is not int or self.radicand < 1:
            raise ValueError("радиканд — целое не меньше единицы")
        if self.coefficient.numerator == 0:
            raise ValueError("нулевой член в каноническом наборе не хранится")


@dataclass(frozen=True, slots=True)
class ExactAlgebraicSumV1:
    """`sum c_m * sqrt(m)` записью контракта, каноническим набором.

    Тот же канонический набор, что у ядерной `SqrtSumV1`, и это не копия
    арифметики, а её ТРАНСПОРТ: `Fraction` схемой не описывается вовсе, а без
    схемы контракт не переезжает. Каноничность (радиканды строго возрастают,
    нулевых коэффициентов нет) держит побитовую воспроизводимость: одна
    величина — один набор байт.
    """

    terms: tuple[ExactRadicalTermV1, ...]

    def __post_init__(self) -> None:
        radicands = [term.radicand for term in self.terms]
        if radicands != sorted(set(radicands)):
            raise ValueError(
                "канонический набор строго возрастает по радиканду и не "
                "повторяет его"
            )


@dataclass(frozen=True, slots=True)
class ExactAlgebraicPointV1:
    """Точка в карте ячейки, обе координаты — суммы корней."""

    x: ExactAlgebraicSumV1
    y: ExactAlgebraicSumV1


@dataclass(frozen=True, slots=True)
class ExactAlgebraicTimeV1:
    """`t = dividend / divisor`. Значение НЕ вычисляется, как и в P0-1."""

    dividend: ExactRationalV1
    divisor: ExactAlgebraicSumV1

    def __post_init__(self) -> None:
        if not self.divisor.terms:
            raise ValueError("знаменатель времени доказанно нулевой")


@dataclass(frozen=True, slots=True)
class ArrivalSupportLineV1:
    """`a*x + b*y = c + t*sqrt(q)` — фронт посева в карте ячейки.

    `a`, `b`, `c` ЦЕЛЫЕ, и это не удобство: карта ячейки построена на решётке
    (`IntegerGridCertificateV1` из S0), прямая через два узла решётки имеет
    целые коэффициенты, и пока они целые, каждый предикат — целочисленный
    определитель. `q` рационально и неотрицательно: оно входит только под
    корень, и рациональный множитель иррациональности не добавляет.
    """

    a: int
    b: int
    c: int
    q: ExactRationalV1

    def __post_init__(self) -> None:
        if not all(type(item) is int for item in (self.a, self.b, self.c)):
            raise ValueError("коэффициенты несущей прямой целые")
        if self.q.numerator < 0:
            raise ValueError("квадрат скорости неотрицателен")


# --------------------------------------------------------------------------
# Ссылки на власть
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SurfaceMetricRefV1:
    """Ссылка на `SurfaceMetricDescriptorV2` ПО ДАЙДЖЕСТУ.

    `surface_regime` хранится здесь, а не выводится: без него запрет
    «GENERAL_CURVED не бывает REFERENCE_EXACT_SMALL» невозможно проверить по
    самому комплексу, и он остался бы обещанием в документе.
    """

    source_revision: SourceRevision
    patch_domain_id: PatchDomainId
    surface_regime: SurfaceRegime
    metric_digest: ArrivalMetricDigestValue
    adjacency_ref: SurfaceAdjacencyRefV1

    def __post_init__(self) -> None:
        if self.surface_regime is SurfaceRegime.PLANAR:
            raise ValueError(
                "V2-метрика не описывает PLANAR (закон S0); планарная власть "
                "называется PlanarChartRefV1"
            )


@dataclass(frozen=True, slots=True)
class PlanarChartRefV1:
    """Ссылка на планарную власть: одна карта, таблицы смежности НЕТ.

    Отсутствие таблицы здесь — не пробел поставки, а отсутствие предмета: у
    одной плоской карты нет сторон треугольников, которые нужно было бы
    склеивать. Именно поэтому `adjacency_ref` комплекса под этой ссылкой обязан
    быть `None`: непустая ссылка означала бы, что склейка откуда-то взялась.
    """

    source_revision: SourceRevision
    patch_domain_id: PatchDomainId
    chart_digest: ArrivalMetricDigestValue


ArrivalMetricRefV1 = SurfaceMetricRefV1 | PlanarChartRefV1


def arrival_metric_digest(record: Any) -> ArrivalMetricDigestValue:
    """sha256 канонических байт власти метрики — той, что реально приложена."""

    return ArrivalMetricDigestValue(
        sha256(canonical_json_bytes(record)).hexdigest()
    )


# --------------------------------------------------------------------------
# Посевы, кандидаты, ячейки
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SeedStationIntervalV1:
    """Отрезок посева: где именно он излучает.

    У точечного посева и у направленной опоры концы СОВПАДАЮТ, и это не
    вырождение записи, а факт: у скрытой опоры веера пролёт нулевой, а нормаль
    задана входом. Отдельной записи «точка» здесь нет намеренно — иначе один и
    тот же посев описывался бы двумя формами.
    """

    start: ExactPoint2V1
    end: ExactPoint2V1


@dataclass(frozen=True, slots=True)
class ArrivalSeedV1:
    """Один посев: чем задан, чей ключ и на каком отрезке он стоит."""

    kind: ArrivalSeedKindV1
    owner_key: ArrivalOwnerKeyV1
    station_interval: SeedStationIntervalV1

    def __post_init__(self) -> None:
        degenerate = self.station_interval.start == self.station_interval.end
        if self.kind is ArrivalSeedKindV1.PARALLEL_SEGMENT and degenerate:
            raise ValueError(
                "PARALLEL_SEGMENT с совпавшими концами — это не отрезок; "
                "направленная опора называется DIRECTIONAL_PARALLEL"
            )
        if self.kind is ArrivalSeedKindV1.POINT and not degenerate:
            raise ValueError("POINT занимает одну точку, а не отрезок")


@dataclass(frozen=True, slots=True)
class BarycentricHalfPlaneV1:
    """`w0*b0 + w1*b1 + w2*b2 <= bound` — одна грань bary-области."""

    w0: ExactRationalV1
    w1: ExactRationalV1
    w2: ExactRationalV1
    bound: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ArrivalDomainV1:
    """Область кандидата внутри ячейки — bary-регионом либо всей ячейкой.

    `WHOLE_CELL_V1` существует не ради планарной карты, а ради честности: у
    ячейки с единственным кандидатом область РАВНА ячейке, и записывать её тремя
    тривиальными полуплоскостями значило бы притворяться, что резать было чем.
    """

    law: ArrivalDomainLawV1
    half_planes: tuple[BarycentricHalfPlaneV1, ...]

    def __post_init__(self) -> None:
        whole = self.law is ArrivalDomainLawV1.WHOLE_CELL_V1
        if whole and self.half_planes:
            raise ValueError("WHOLE_CELL_V1 не режется полуплоскостями")
        if not whole and not self.half_planes:
            raise ValueError("BARYCENTRIC_REGION_V1 обязан назвать полуплоскости")


@dataclass(frozen=True, slots=True)
class StationWitnessV1:
    """Станция на пути кандидата: где именно путь переизлучился или кончился."""

    kind: StationWitnessKindV1
    place: ExactAlgebraicPointV1


@dataclass(frozen=True, slots=True)
class ExactArrivalPrecisionV1:
    """Закон прихода задан ТОЧНО. Параметров у точности нет.

    Пустая запись намеренна: «точно» — это отсутствие оболочки, а не оболочка
    нулевой ширины. Нулевая ширина была бы утверждением о числе, которого никто
    не считал.
    """


@dataclass(frozen=True, slots=True)
class CertifiedArrivalPrecisionV1:
    """Приход известен сертифицированной оболочкой, и её ε ИМЕНОВАН.

    Оболочка относится к значению прихода на СОБСТВЕННОЙ области кандидата:
    нижняя граница — там, где фронт в неё входит, верхняя — там, где владение
    кончается. Безымянной погрешности не бывает, поэтому `named_epsilon`
    обязателен здесь, а не только у комплекса целиком.
    """

    enclosure: CertifiedDecimalIntervalV1
    named_epsilon: NamedEpsilonV1


ArrivalPrecisionV1 = ExactArrivalPrecisionV1 | CertifiedArrivalPrecisionV1


@dataclass(frozen=True, slots=True)
class ArrivalCandidateV1:
    """Один кандидат на владение куском ячейки."""

    owner_key: ArrivalOwnerKeyV1
    path_history_key: ArrivalPathHistoryKeyV1
    arrival_law: ArrivalLawV1
    station_witnesses: tuple[StationWitnessV1, ...]
    domain: ArrivalDomainV1
    precision: ArrivalPrecisionV1


@dataclass(frozen=True, slots=True)
class ArrivalCellV1:
    """Ячейка комплекса: кортеж кандидатов и судьба их ничьей."""

    cell_id: ArrivalCellId
    candidates: tuple[ArrivalCandidateV1, ...]
    tie_resolution: ArrivalTieResolutionV1

    def __post_init__(self) -> None:
        if not self.candidates:
            raise ValueError(
                "ячейка без кандидатов не записывается: недостигнутая ячейка "
                "отсутствует, а не присутствует пустой"
            )
        keys = {
            (item.owner_key, item.path_history_key) for item in self.candidates
        }
        if len(keys) != len(self.candidates):
            raise ValueError(
                "два кандидата с одной парой (владелец, история пути) "
                "неразличимы — минимум брался бы по обрезанному множеству"
            )


@dataclass(frozen=True, slots=True)
class CutLocusPointV1:
    """Точка cut locus: место, где владение делят двое и более.

    Владельцы — МНОЖЕСТВОМ на место, а не по записи на событие: в одной точке
    планарного скелета стоит до шести записей, а делят её всё те же владельцы.
    """

    place: ExactAlgebraicPointV1
    time: ExactAlgebraicTimeV1
    owner_keys: tuple[ArrivalOwnerKeyV1, ...]

    def __post_init__(self) -> None:
        if len(self.owner_keys) < 2:
            raise ValueError("cut locus — место ВСТРЕЧИ; владельцев минимум двое")
        values = [item.value for item in self.owner_keys]
        if values != sorted(set(values)):
            raise ValueError("владельцы точки cut locus упорядочены и уникальны")


@dataclass(frozen=True, slots=True)
class OwnerFragmentV1:
    """Кусок владения одним посевом: точный контур и его удвоенная площадь.

    Фрагмент — ГЕОМЕТРИЯ владения, тогда как ячейка — его комбинаторика. Развод
    не косметический: комбинаторика отвечает «кто спорит за эту ячейку», а
    фрагмент — «сколько именно ему досталось», и второе обязано быть точным
    числом, иначе владение нечем сверить с площадью области.
    """

    owner_key: ArrivalOwnerKeyV1
    cell_id: ArrivalCellId
    support: ArrivalSupportLineV1
    span_start: ExactPoint2V1
    span_end: ExactPoint2V1
    contour: tuple[ExactAlgebraicPointV1, ...]
    doubled_area: ExactAlgebraicSumV1

    def __post_init__(self) -> None:
        if len(self.contour) < 3:
            raise ValueError("контур фрагмента владения — не менее трёх точек")


@dataclass(frozen=True, slots=True)
class ArrivalEventV1:
    """Событие комплекса, формой `SkeletonNodeV1`: когда, где, кто и сколько.

    `kind` и `kinds` — `LawId`, а не перечисление: виды событий принадлежат
    производящему движку (`wavefront/**`), и копия его словаря здесь означала бы
    два места, где решается, что такое `EDGE`.
    """

    kind: LawId
    time: ExactAlgebraicTimeV1
    place: ExactAlgebraicPointV1
    participants: tuple[ArrivalOwnerKeyV1, ...]
    converging_vertices: int
    kinds: tuple[LawId, ...]
    incidences: tuple[tuple[ArrivalOwnerKeyV1, ...], ...]

    def __post_init__(self) -> None:
        if type(self.converging_vertices) is not int:
            raise ValueError("число сошедшихся вершин — целое")
        if self.converging_vertices < 0:
            raise ValueError("число сошедшихся вершин неотрицательно")


@dataclass(frozen=True, slots=True)
class ArrivalProofObligationV1:
    """Стабильная проекция долга P0-1. НЕ новый долг и не новый его вид.

    Проекция — та же, что P0-3 объявил стабильной: runtime-идентификаторы вершин
    исключены намеренно, потому что они не переживают ни одного переупорядочения
    входа. `disposition` — перечисление P0-1 импортом (расширять его нечем и
    незачем), `cause` — `LawId` со значением словаря того же P0-1: словарь
    отказов принадлежит движку и растёт вместе с ним, а замороженный в схеме
    транспорта он превратил бы каждую новую причину в ломку контракта.
    """

    cause: LawId
    disposition: ProofObligationDisposition
    level: ExactAlgebraicTimeV1
    participant_edge_keys: tuple[ArrivalOwnerKeyV1, ...]
    target_edge_keys: tuple[ArrivalOwnerKeyV1, ...]
    event_kind: LawId | None


@dataclass(frozen=True, slots=True)
class ArrivalCounterV1:
    """Счётчик комплекса. Ноль записывается, а не отсутствует."""

    name: LawId
    value: int

    def __post_init__(self) -> None:
        if type(self.value) is not int:
            raise ValueError("счётчик — целое")


# --------------------------------------------------------------------------
# Комплекс
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SurfaceArrivalComplexV1:
    """Предвычисленный комплекс прибытия до `alpha_horizon`.

    В union `SurfaceMetricDescriptorV1` и в фасад `cftuv_envelope` НЕ входит:
    проводка транспорта — отдельная карточка после интеграционной вершины, до
    неё замороженные дайджесты фасада не двигаются вовсе.
    """

    schema_version: str
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    backend: SurfaceArrivalBackendV1
    metric_ref: ArrivalMetricRefV1
    adjacency_ref: SurfaceAdjacencyRefV1 | None
    cell_law: ArrivalCellLawV1
    alpha_horizon: ExactRationalV1
    seeds: frozenset[ArrivalSeedV1]
    cells: frozenset[ArrivalCellV1]
    cut_locus: frozenset[CutLocusPointV1]
    owner_fragments: frozenset[OwnerFragmentV1]
    events: tuple[ArrivalEventV1, ...]
    front_outcome: SkeletonOutcome
    ownership_outcome: LawId
    proof_status: ProofStatus
    proof_obligations: tuple[ArrivalProofObligationV1, ...]
    precision_tier: SurfaceMetricPrecisionTierV1
    named_epsilon: NamedEpsilonV1 | None
    counters: tuple[ArrivalCounterV1, ...]

    def __post_init__(self) -> None:
        validate_surface_arrival_complex(self)


def _check_identity(complex_: SurfaceArrivalComplexV1) -> None:
    if complex_.schema_version != SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA:
        _fail("SCHEMA_VERSION", complex_.schema_version)
    ref = complex_.metric_ref
    if ref.source_revision != complex_.source_revision:
        _fail(
            "SOURCE_REVISION",
            f"complex={complex_.source_revision.value} "
            f"metric_ref={ref.source_revision.value}",
        )
    if ref.patch_domain_id != complex_.patch_domain_id:
        _fail(
            "PATCH_DOMAIN",
            f"complex={complex_.patch_domain_id.value} "
            f"metric_ref={ref.patch_domain_id.value}",
        )


def _check_refs(complex_: SurfaceArrivalComplexV1) -> None:
    """G-N8: комплекс без совпавших ссылок бессмыслен, и это отказ."""

    ref = complex_.metric_ref
    declared = complex_.adjacency_ref
    if isinstance(ref, PlanarChartRefV1):
        if declared is not None:
            _fail(
                "PLANAR_CHART_HAS_NO_ADJACENCY",
                f"adjacency_digest={declared.adjacency_digest.value}",
            )
        if complex_.cell_law is not ArrivalCellLawV1.PLANAR_SINGLE_CHART_REGION_V1:
            _fail("CELL_LAW_DISAGREES_WITH_METRIC_REF", complex_.cell_law.value)
        return
    if declared is None:
        _fail(
            "SURFACE_METRIC_REQUIRES_ADJACENCY",
            "комплекс на меше без таблицы смежности неразрешим",
        )
    if declared != ref.adjacency_ref:
        _fail(
            "METRIC_REF_ADJACENCY_DISAGREEMENT",
            f"complex={declared.adjacency_digest.value} "
            f"metric={ref.adjacency_ref.adjacency_digest.value}",
        )
    if complex_.cell_law is not ArrivalCellLawV1.MESH_TRIANGLE_V1:
        _fail("CELL_LAW_DISAGREES_WITH_METRIC_REF", complex_.cell_law.value)


def _check_horizon(complex_: SurfaceArrivalComplexV1) -> None:
    if complex_.alpha_horizon.numerator <= 0:
        _fail(
            "ALPHA_HORIZON_NOT_POSITIVE",
            f"{complex_.alpha_horizon.numerator}/"
            f"{complex_.alpha_horizon.denominator}",
        )


def _check_precision(complex_: SurfaceArrivalComplexV1) -> None:
    """G-N7 и запрет точного класса на кривой метрике — одним разрезом."""

    exact_tier = (
        complex_.precision_tier
        is SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL
    )
    if exact_tier and complex_.named_epsilon is not None:
        _fail("EPSILON_UNDER_EXACT_TIER", complex_.named_epsilon.name.value)
    if not exact_tier and complex_.named_epsilon is None:
        _fail("CERTIFIED_WITHOUT_EPSILON", complex_.precision_tier.value)
    ref = complex_.metric_ref
    if (
        isinstance(ref, SurfaceMetricRefV1)
        and ref.surface_regime is SurfaceRegime.GENERAL_CURVED
        and exact_tier
    ):
        _fail(
            "GENERAL_CURVED_IS_NEVER_EXACT",
            "цепочки разворотов компонуют радикалы, решётка их не вмещает",
        )
    if exact_tier:
        certified = [
            candidate
            for cell in complex_.cells
            for candidate in cell.candidates
            if isinstance(candidate.precision, CertifiedArrivalPrecisionV1)
        ]
        if certified:
            _fail(
                "CERTIFIED_CANDIDATE_UNDER_EXACT_TIER",
                f"кандидатов с оболочкой: {len(certified)}",
            )


def _check_cells(complex_: SurfaceArrivalComplexV1) -> None:
    """Ничья сохраняется, владелец объявлен посевом, адрес ячейки единствен."""

    seed_keys = {seed.owner_key for seed in complex_.seeds}
    seen: set[str] = set()
    for cell in complex_.cells:
        if cell.cell_id.value in seen:
            _fail("CELL_ID_UNIQUE", cell.cell_id.value)
        seen.add(cell.cell_id.value)
        _check_tie(cell)
        for candidate in cell.candidates:
            if candidate.owner_key not in seed_keys:
                _fail(
                    "CANDIDATE_OWNER_IS_NOT_A_SEED",
                    f"cell={cell.cell_id.value} "
                    f"owner={candidate.owner_key.value}",
                )


def _check_tie(cell: ArrivalCellV1) -> None:
    single = len(cell.candidates) == 1
    if cell.tie_resolution is ArrivalTieResolutionV1.RESOLVED_EXACT and not single:
        _fail(
            "RESOLVED_EXACT_KEEPS_ONE_CANDIDATE",
            f"cell={cell.cell_id.value} candidates={len(cell.candidates)}",
        )
    if (
        cell.tie_resolution
        in (
            ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
            ArrivalTieResolutionV1.UNDECIDED_FAIL_CLOSED,
        )
        and single
    ):
        _fail(
            "TIE_NEEDS_TWO_CANDIDATES",
            f"cell={cell.cell_id.value} "
            f"tie_resolution={cell.tie_resolution.value}",
        )


def _check_seeds(complex_: SurfaceArrivalComplexV1) -> None:
    keys = {seed.owner_key.value for seed in complex_.seeds}
    if len(keys) != len(complex_.seeds):
        _fail("SEED_OWNER_UNIQUE", "один ключ владельца — один посев")


def _check_fragments(complex_: SurfaceArrivalComplexV1) -> None:
    seed_keys = {seed.owner_key for seed in complex_.seeds}
    places: set[tuple[str, str]] = set()
    for fragment in complex_.owner_fragments:
        if fragment.owner_key not in seed_keys:
            _fail(
                "FRAGMENT_OWNER_IS_NOT_A_SEED", fragment.owner_key.value
            )
        place = (fragment.owner_key.value, fragment.cell_id.value)
        if place in places:
            _fail("FRAGMENT_PLACE_UNIQUE", f"{place[0]}@{place[1]}")
        places.add(place)


def _check_proof(complex_: SurfaceArrivalComplexV1) -> None:
    """Двоичность P0-1 не трогается: статус — тип P0-1, а не наш."""

    if not isinstance(complex_.proof_status, ProofStatus):
        _fail("PROOF_STATUS_TYPE", type(complex_.proof_status).__name__)
    for obligation in complex_.proof_obligations:
        if not isinstance(obligation.disposition, ProofObligationDisposition):
            _fail(
                "OBLIGATION_DISPOSITION_TYPE",
                type(obligation.disposition).__name__,
            )


def _check_counters(complex_: SurfaceArrivalComplexV1) -> None:
    names = [item.name.value for item in complex_.counters]
    if names != sorted(set(names)):
        _fail(
            "COUNTERS_ORDER",
            "счётчики упорядочены по имени и не повторяются",
        )


def validate_surface_arrival_complex(complex_: SurfaceArrivalComplexV1) -> None:
    """Все инварианты комплекса одним проходом. Отказ — именованный.

    Функция вызывается самим `__post_init__`, поэтому невалидного комплекса не
    существует вовсе: проверка, которую можно забыть вызвать, — это слух.
    Отдельно она вызывается ещё и потому, что читатель обязан иметь право
    перепроверить принятые байты, не строя объект заново.
    """

    _check_identity(complex_)
    _check_refs(complex_)
    _check_horizon(complex_)
    _check_precision(complex_)
    _check_seeds(complex_)
    _check_cells(complex_)
    _check_fragments(complex_)
    _check_proof(complex_)
    _check_counters(complex_)


def exact_rational(value: Fraction | int) -> ExactRationalV1:
    """`Fraction` в запись контракта. Приведение единственно и обратимо."""

    ratio = Fraction(value)
    return ExactRationalV1(ratio.numerator, ratio.denominator)


def require_alpha_within_horizon(
    complex_: SurfaceArrivalComplexV1, alpha: ExactRationalV1
) -> None:
    """G-N9: запрос сверх горизонта — ИМЕНОВАННЫЙ отказ, а не досчёт.

    Комплекс предвычислен до `alpha_horizon`; drag только режет посчитанное.
    Молча ответить на больший alpha значило бы выдать обрезанный ответ за
    полный — ровно то тихое исчезновение, которое запрещено.
    """

    requested = Fraction(alpha.numerator, alpha.denominator)
    horizon = Fraction(
        complex_.alpha_horizon.numerator, complex_.alpha_horizon.denominator
    )
    if requested < 0:
        _fail("ALPHA_IS_NEGATIVE", str(requested))
    if requested > horizon:
        _fail(
            "ALPHA_BEYOND_HORIZON",
            f"requested={requested} horizon={horizon}",
        )


def validate_arrival_against_metric(
    complex_: SurfaceArrivalComplexV1,
    *,
    metric: Any,
    adjacency: Any,
) -> None:
    """Кросс-проверка комплекса против ПРИЛОЖЕННЫХ метрики и таблицы.

    Сверяется ровно то, что комплекс утверждает о чужих байтах: пересчитанный
    дайджест метрики, ссылка на смежность и вхождение адресов ячеек в множество
    треугольников таблицы. Доверие здесь не участвует — только сравнение.
    """

    ref = complex_.metric_ref
    if not isinstance(ref, SurfaceMetricRefV1):
        _fail("METRIC_REF_KIND", type(ref).__name__)
    recomputed = arrival_metric_digest(metric)
    if recomputed != ref.metric_digest:
        _fail(
            "METRIC_DIGEST",
            f"stored={ref.metric_digest.value} recomputed={recomputed.value}",
        )
    if metric.adjacency_ref != ref.adjacency_ref:
        _fail(
            "METRIC_REF_ADJACENCY_DISAGREEMENT",
            "приложенная метрика ссылается на другую таблицу смежности",
        )
    triangles = {
        side.side.triangle_id.value for side in adjacency.triangle_sides
    }
    for cell in complex_.cells:
        if cell.cell_id.value not in triangles:
            _fail("CELL_IS_NOT_A_MESH_TRIANGLE", cell.cell_id.value)


__all__ = (
    "PLANAR_DIRECT_PATH_HISTORY",
    "SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA",
    "ArrivalCandidateV1",
    "ArrivalCellId",
    "ArrivalCellLawV1",
    "ArrivalCellV1",
    "ArrivalCounterV1",
    "ArrivalDomainLawV1",
    "ArrivalDomainV1",
    "ArrivalEventV1",
    "ArrivalLawV1",
    "ArrivalMetricDigestValue",
    "ArrivalMetricRefV1",
    "ArrivalOwnerKeyV1",
    "ArrivalPathHistoryKeyV1",
    "ArrivalPrecisionV1",
    "ArrivalProofObligationV1",
    "ArrivalSeedKindV1",
    "ArrivalSeedV1",
    "ArrivalSupportLineV1",
    "ArrivalTieResolutionV1",
    "BarycentricHalfPlaneV1",
    "CertifiedArrivalPrecisionV1",
    "CutLocusPointV1",
    "ExactAlgebraicPointV1",
    "ExactAlgebraicSumV1",
    "ExactAlgebraicTimeV1",
    "ExactArrivalPrecisionV1",
    "ExactRadicalTermV1",
    "OwnerFragmentV1",
    "PlanarChartRefV1",
    "SeedStationIntervalV1",
    "StationWitnessKindV1",
    "StationWitnessV1",
    "SurfaceArrivalBackendV1",
    "SurfaceArrivalComplexV1",
    "SurfaceArrivalContractError",
    "SurfaceMetricRefV1",
    "arrival_metric_digest",
    "exact_rational",
    "require_alpha_within_horizon",
    "validate_arrival_against_metric",
    "validate_surface_arrival_complex",
)
