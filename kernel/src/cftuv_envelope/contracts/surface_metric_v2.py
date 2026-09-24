"""Власть метрики непланарной поверхности: piecewise-flat после снапа.

Закон записан в `DECISIONS.md` за 2026-08-02 («Модель точности поверхности») и
уточнён 2026-08-03 («AUTH развилок S0/S2»):

* Власть general-curved — ВЫЧИСЛЕННЫЙ триангулированный меш ПОСЛЕ привязки
  источника к решётке. Не гладкий предел subdivision, не касательная
  аппроксимация: набор плоских треугольников с рациональными квадратами длин.
* general-curved НИКОГДА не называется EXACT. Цепочки разворотов компонуют
  радикалы и наращивают алгебраическую степень; решётка `SqrtSumV1` их не
  вмещает. Честная граница лучше притворного exact, поэтому класс точности —
  ОТДЕЛЬНАЯ ось (`precision_tier`) с именованным ε, а не ярлык на ответе.
* `snapped_source_positions` хранятся ЯВНО. Это закрывает счёт ROADMAP
  «результаты воспроизводимы только из допроекционных позиций, которых метрика
  не несёт»: без них ни один читатель не может перевывести ни длины, ни углы.
* Периодика — поле С РОЖДЕНИЯ. Единственное допустимое значение до S1 —
  `NON_PERIODIC_V1`; всё прочее fail-closed. Поле, заведённое позже, означало
  бы, что сегодняшние записи молча считались непериодическими.

Чего здесь НЕТ намеренно. Вычислений: контракт описывает, что записано, а
считает `surface_metric_build.py` — тот же разрез, что у планарной метрики
(`contracts/metric.py` против `planar_metric.py`). И членства в union
`SurfaceMetricDescriptorV1`: проводка транспорта — отдельная карточка после
интеграционной вершины, до неё байты снапшота не двигаются вовсе.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum

from ..ids import (
    LawId,
    PatchDomainId,
    PhysicalEdgeId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from ..numeric import CertifiedDecimalIntervalV1
from .analysis import SurfaceRegime
from .metric import (
    ExactMatrix2V1,
    ExactPoint3V1,
    ExactRationalV1,
    IntegerGridCertificateV1,
)
from .surface_adjacency import (
    SurfaceAdjacencyDigestValue,
    SurfaceAdjacencyScopeV1,
)


SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA = "cftuv.envelope.surface_metric_descriptor.v2"


class SurfaceMetricEvaluationLawV1(str, Enum):
    """Что именно объявлено властью метрики. Второго значения нет."""

    EVALUATED_TRIANGULATED_MESH_AFTER_SOURCE_SNAP_V1 = (
        "EVALUATED_TRIANGULATED_MESH_AFTER_SOURCE_SNAP_V1"
    )


class SurfaceMetricPrecisionTierV1(str, Enum):
    """Класс точности — ОТДЕЛЬНАЯ ось от двоичного ProofStatus (AUTH S2).

    Слить оси значило бы перекроить принятую P0-1 и девять осей P0-3. Поэтому
    ответ остаётся «доказан / не доказан», а «чем доказан» живёт здесь.
    """

    REFERENCE_EXACT_SMALL = "REFERENCE_EXACT_SMALL"
    REFERENCE_CERTIFIED = "REFERENCE_CERTIFIED"
    RUNTIME_APPROXIMATE = "RUNTIME_APPROXIMATE"


class SurfacePeriodicTopologyV1(str, Enum):
    """Периодика домена. До S1 допустимо ровно одно значение.

    Второе заведено НЕ для употребления, а чтобы дверь fail-closed была
    исполняемой: закон, у которого нельзя построить нарушение, не проверяется
    ничем и потому является слухом.
    """

    NON_PERIODIC_V1 = "NON_PERIODIC_V1"
    PERIODIC_UNIVERSAL_COVER_V1 = "PERIODIC_UNIVERSAL_COVER_V1"


class SurfaceBarrierRoleV1(str, Enum):
    """Стена на поверхности раздваивается, и третья роль — источник прибытия.

    `BARRIER` — непроницаемое ограничение домена, участником конкуренции
    расстояний НЕ являющееся. `BOUNDARY_CONDITION` — стационарная опора
    топологии. `ARRIVAL_SOURCE` — носитель посева. Роль объявляется, а не
    выводится из геометрии: одно и то же ребро в разных запросах играет разные
    роли, и молчаливый выбор здесь стоил бы неверного ответа без отказа.
    """

    BARRIER = "BARRIER"
    BOUNDARY_CONDITION = "BOUNDARY_CONDITION"
    ARRIVAL_SOURCE = "ARRIVAL_SOURCE"


class DegenerateTriangleReasonV1(str, Enum):
    """Почему треугольник вырожден ПОСЛЕ привязки. Не отказ, а именование."""

    COINCIDENT_SNAPPED_VERTICES = "COINCIDENT_SNAPPED_VERTICES"
    COLLINEAR_SNAPPED_VERTICES = "COLLINEAR_SNAPPED_VERTICES"


class ConeAngleVerdictV1(str, Enum):
    """Вердикт по конусному углу вершины. Четвёртый исход — отказ."""

    EXACT_TWO_PI = "EXACT_TWO_PI"
    STRICTLY_LESS = "STRICTLY_LESS"
    STRICTLY_GREATER = "STRICTLY_GREATER"
    UNDECIDED_FAIL_CLOSED = "UNDECIDED_FAIL_CLOSED"


class ConeAngleMeasureLawV1(str, Enum):
    """Чем получен вердикт. Оболочка и точный вывод — РАЗНЫЕ законы.

    `EXACT_PLANAR_CLOSED_FAN_V1` — веер замкнут и все его треугольники ТОЧНО
    компланарны в привязанных рациональных координатах; сумма углов равна 2π по
    построению, а не по числу. Оболочка при этом всё равно записывается и
    обязана содержать 2π: расхождение точного вывода с сертифицированной
    оболочкой означает дефект, а не выбор одного из них.

    `CERTIFIED_INTERVAL_ENCLOSURE_V1` — вердикт получен РАЗДЕЛЕНИЕМ оболочки и
    2π. Не разделилось — `UNDECIDED_FAIL_CLOSED`, а не «примерно равно».

    `FAN_UNAVAILABLE_V1` — веера нет (немногообразие или граница скоупа);
    мерить нечего, и это записано, а не спрятано под «граничная вершина».
    """

    EXACT_PLANAR_CLOSED_FAN_V1 = "EXACT_PLANAR_CLOSED_FAN_V1"
    CERTIFIED_INTERVAL_ENCLOSURE_V1 = "CERTIFIED_INTERVAL_ENCLOSURE_V1"
    FAN_UNAVAILABLE_V1 = "FAN_UNAVAILABLE_V1"


@dataclass(frozen=True, slots=True)
class NamedEpsilonV1:
    """Именованная граница погрешности. Безымянной погрешности не бывает."""

    name: LawId
    absolute_bound: Decimal

    def __post_init__(self) -> None:
        if not isinstance(self.absolute_bound, Decimal):
            raise ValueError("NamedEpsilonV1 требует Decimal")
        if not self.absolute_bound.is_finite() or self.absolute_bound < 0:
            raise ValueError("граница погрешности конечна и неотрицательна")


@dataclass(frozen=True, slots=True)
class SnappedSourcePositionV1:
    """Позиция вершины ПОСЛЕ привязки, точной рациональной тройкой."""

    source_vertex_id: SourceVertexId
    position: ExactPoint3V1


@dataclass(frozen=True, slots=True)
class TriangleSquaredLengthsV1:
    """Квадраты длин сторон треугольника, по ЯДЕРНОЙ нумерации сторон.

    Хранятся квадраты, а не длины: квадрат рационален всегда, длина —
    почти никогда. Это и есть piecewise-flat метрика: всё, что нужно для
    внутренней геометрии треугольника, лежит в трёх рациональных числах.
    """

    triangle_id: SurfaceTriangleId
    side_0: ExactRationalV1
    side_1: ExactRationalV1
    side_2: ExactRationalV1


@dataclass(frozen=True, slots=True)
class TriangleGramV1:
    """Матрица Грама пары рёбер `(v0→v1, v0→v2)` треугольника."""

    triangle_id: SurfaceTriangleId
    gram: ExactMatrix2V1


@dataclass(frozen=True, slots=True)
class DegenerateTriangleV1:
    """Вырожденный после привязки треугольник — именованный, не отброшенный."""

    triangle_id: SurfaceTriangleId
    reason: DegenerateTriangleReasonV1


@dataclass(frozen=True, slots=True)
class IntrinsicSurfaceGeometryV2:
    """Внутренняя геометрия: квадраты длин, Грам и вырожденные множеством."""

    triangle_squared_lengths: frozenset[TriangleSquaredLengthsV1]
    triangle_gram_matrices: frozenset[TriangleGramV1]
    degenerate_triangles: frozenset[DegenerateTriangleV1]

    def __post_init__(self) -> None:
        lengths = {item.triangle_id for item in self.triangle_squared_lengths}
        grams = {item.triangle_id for item in self.triangle_gram_matrices}
        if len(lengths) != len(self.triangle_squared_lengths):
            raise ValueError("на треугольник приходится одна тройка квадратов")
        if len(grams) != len(self.triangle_gram_matrices):
            raise ValueError("на треугольник приходится одна матрица Грама")
        if lengths != grams:
            raise ValueError(
                "квадраты длин и матрицы Грама названы для разных множеств "
                "треугольников"
            )
        degenerate = {item.triangle_id for item in self.degenerate_triangles}
        if len(degenerate) != len(self.degenerate_triangles):
            raise ValueError("вырожденный треугольник называется один раз")
        if not degenerate <= lengths:
            raise ValueError(
                "вырожденным назван треугольник, которого нет в метрике"
            )


@dataclass(frozen=True, slots=True)
class SurfaceBarrierV1:
    """Роль физического ребра источника. Диагоналей здесь быть не может.

    Тип поля — `PhysicalEdgeId`, и это не украшение: барьер живёт на ребре
    ИСТОЧНИКА, а диагональ тесселяции физического ребра не имеет вовсе. Стена,
    привязанная к диагонали, означала бы, что разбиение поменяло физику.
    """

    physical_edge_id: PhysicalEdgeId
    role: SurfaceBarrierRoleV1


@dataclass(frozen=True, slots=True)
class VertexConeAngleV1:
    """Конусный угол вершины: оболочка, вердикт и закон измерения врозь.

    Три поля, а не одно, потому что это три разных утверждения: сколько
    получилось (оболочка), что из этого следует (вердикт) и чем это получено
    (закон). Слить их значило бы потерять возможность сказать «измерено, но не
    разделилось» — а именно это и есть честный ответ на гладкой поверхности с
    мелкой сеткой.
    """

    vertex_id: SourceVertexId
    enclosure: CertifiedDecimalIntervalV1
    verdict: ConeAngleVerdictV1
    measure_law: ConeAngleMeasureLawV1

    def __post_init__(self) -> None:
        if self.measure_law is ConeAngleMeasureLawV1.FAN_UNAVAILABLE_V1:
            if self.verdict is not ConeAngleVerdictV1.UNDECIDED_FAIL_CLOSED:
                raise ValueError(
                    "без веера вердикт может быть только UNDECIDED_FAIL_CLOSED"
                )
        if (
            self.measure_law is ConeAngleMeasureLawV1.EXACT_PLANAR_CLOSED_FAN_V1
            and self.verdict is not ConeAngleVerdictV1.EXACT_TWO_PI
        ):
            raise ValueError(
                "точный плоский замкнутый веер даёт ровно EXACT_TWO_PI"
            )
        if (
            self.verdict is ConeAngleVerdictV1.EXACT_TWO_PI
            and self.measure_law
            is not ConeAngleMeasureLawV1.EXACT_PLANAR_CLOSED_FAN_V1
        ):
            raise ValueError(
                "EXACT_TWO_PI не выводится из оболочки: 2π иррационально, и "
                "интервал его равенства не доказывает"
            )


@dataclass(frozen=True, slots=True)
class SurfaceAdjacencyRefV1:
    """Ссылка на таблицу смежности ПО ДАЙДЖЕСТУ, а не по доверию."""

    source_revision: SourceRevision
    scope: SurfaceAdjacencyScopeV1
    adjacency_digest: SurfaceAdjacencyDigestValue


@dataclass(frozen=True, slots=True)
class SurfaceMetricDescriptorV2:
    """Метрика непланарного домена. В union `SurfaceMetricDescriptorV1` НЕ входит."""

    schema_version: str
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    surface_regime: SurfaceRegime
    evaluation_law: SurfaceMetricEvaluationLawV1
    precision_tier: SurfaceMetricPrecisionTierV1
    named_epsilon: NamedEpsilonV1 | None
    grid_certificate: IntegerGridCertificateV1
    snapped_source_positions: frozenset[SnappedSourcePositionV1]
    intrinsic: IntrinsicSurfaceGeometryV2
    adjacency_ref: SurfaceAdjacencyRefV1
    barriers: frozenset[SurfaceBarrierV1]
    vertex_cone_angles: frozenset[VertexConeAngleV1]
    periodic_topology: SurfacePeriodicTopologyV1

    def __post_init__(self) -> None:
        _check_regime(self)
        _check_precision(self)
        _check_periodicity(self)
        _check_identity(self)


def _check_regime(metric: SurfaceMetricDescriptorV2) -> None:
    """N-7: у PLANAR своя точная метрика, и подменять её эта не имеет права."""

    if metric.surface_regime is SurfaceRegime.PLANAR:
        raise ValueError(
            "SurfaceMetricDescriptorV2 не описывает PLANAR: у планарного "
            "режима есть решётко-точная RationalAffinePlanarMetricV2, и "
            "вычисленный меш её подменять не может"
        )
    if metric.evaluation_law is not (
        SurfaceMetricEvaluationLawV1.EVALUATED_TRIANGULATED_MESH_AFTER_SOURCE_SNAP_V1
    ):
        raise ValueError("закон вычисления V2 единственный")


def _check_precision(metric: SurfaceMetricDescriptorV2) -> None:
    """N-8: general-curved никогда не EXACT, и ε всегда именован."""

    exact_tier = (
        metric.precision_tier is SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL
    )
    if exact_tier and metric.named_epsilon is not None:
        raise ValueError("exact-класс не несёт ε: None ⇔ exact-tier")
    if not exact_tier and metric.named_epsilon is None:
        raise ValueError("неточный класс обязан назвать ε именем и границей")
    if metric.surface_regime is SurfaceRegime.GENERAL_CURVED and exact_tier:
        raise ValueError(
            "GENERAL_CURVED не бывает REFERENCE_EXACT_SMALL: цепочки "
            "разворотов компонуют радикалы, решётка их не вмещает"
        )


def _check_periodicity(metric: SurfaceMetricDescriptorV2) -> None:
    if metric.periodic_topology is not SurfacePeriodicTopologyV1.NON_PERIODIC_V1:
        raise ValueError(
            f"{metric.periodic_topology.value} не поддержан до S1: universal "
            "cover не построен, и молча считать домен непериодическим нельзя"
        )


def _check_identity(metric: SurfaceMetricDescriptorV2) -> None:
    if metric.schema_version != SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA:
        raise ValueError("чужая версия схемы метрики V2")
    if metric.adjacency_ref.source_revision != metric.source_revision:
        raise ValueError(
            "метрика и таблица смежности объявляют разные ревизии источника"
        )
    positions = {item.source_vertex_id for item in metric.snapped_source_positions}
    if len(positions) != len(metric.snapped_source_positions):
        raise ValueError("на вершину приходится одна привязанная позиция")
    angles = {item.vertex_id for item in metric.vertex_cone_angles}
    if len(angles) != len(metric.vertex_cone_angles):
        raise ValueError("на вершину приходится один конусный угол")
    if not angles <= positions:
        raise ValueError(
            "конусный угол назван у вершины без привязанной позиции"
        )
    barriers = {item.physical_edge_id for item in metric.barriers}
    if len(barriers) != len(metric.barriers):
        raise ValueError(
            "одно ребро — одна роль: две роли на ребре означают, что власть "
            "выбирается читателем"
        )


__all__ = (
    "SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA",
    "ConeAngleMeasureLawV1",
    "ConeAngleVerdictV1",
    "DegenerateTriangleReasonV1",
    "DegenerateTriangleV1",
    "IntrinsicSurfaceGeometryV2",
    "NamedEpsilonV1",
    "SnappedSourcePositionV1",
    "SurfaceAdjacencyRefV1",
    "SurfaceBarrierRoleV1",
    "SurfaceBarrierV1",
    "SurfaceMetricDescriptorV2",
    "SurfaceMetricEvaluationLawV1",
    "SurfaceMetricPrecisionTierV1",
    "SurfacePeriodicTopologyV1",
    "TriangleGramV1",
    "TriangleSquaredLengthsV1",
    "VertexConeAngleV1",
)
