"""Public planar metric contracts for exact reference and filtered runtime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import gcd, isfinite

from ..ids import (
    ReferenceMetricId,
    LineageId,
    PatchDomainId,
    PlanarityCertificateId,
    RuntimeMetricId,
    SourceRevision,
    SourceVertexId,
)


REFERENCE_PLANAR_METRIC_SCHEMA_V2 = (
    "cftuv.envelope.rational_affine_planar_metric.v2"
)
RUNTIME_PLANAR_METRIC_SCHEMA_V1 = "cftuv.envelope.runtime_planar_metric.v1"


class PlanarityAdmissionLawV1(str, Enum):
    EXACT_SOURCE_PLANE_V1 = "EXACT_SOURCE_PLANE_V1"
    # Источник не компланарен побитово, но укладывается в объявленный бюджет
    # невязки. Вершины проецируются на плоскость ТОЧНО, в рациональных числах:
    # ниже по конвейеру арифметика остаётся точной, приблизительным является
    # только выбор входа, и он записан в сертификате.
    NEAR_PLANAR_PROJECTION_V1 = "NEAR_PLANAR_PROJECTION_V1"


class GridSnappingLawV1(str, Enum):
    """Как координаты источника попали в эту метрику.

    Форма взята с `PlanarityAdmissionLawV1`: закон существует в контракте
    отдельно от того, какое значение объявляет хост, поэтому переключение
    политики видно в сертификате, а не только в поведении.

    Привязок в конвейере ДВЕ, и они независимы: вершины ИСТОЧНИКА (до базиса,
    механизм детерминированный) и точки КОНСТРУКЦИЙ (`offset_support_g`,
    `segment_intersections` — механизм вероятностный, карточка R1b сама числит
    его лотереей). Поэтому и законов три, а не два: разрез между ними
    измерен — на `building.002` привязка одного источника даёт `EXACT` и ту же
    топологию, а привязка конструкций поверх неё сливает три вычисленные точки
    и оставляет две висячие полурёбра.
    """

    # Нынешнее поведение: координаты binary64 читаются как точные дроби и
    # больше ничем не трогаются.
    UNSNAPPED_EXACT_V1 = "UNSNAPPED_EXACT_V1"
    # Вершины источника привязаны к целочисленной решётке ДО построения
    # базиса, конструкции — НЕ привязаны и остаются точными. Восстановленное
    # задуманное отношение живёт в метрике, а вырождение вычисленных точек
    # по-прежнему не сливается.
    SOURCE_ONLY_GRID_SNAP_V1 = "SOURCE_ONLY_GRID_SNAP_V1"
    # Вершины источника привязаны к целочисленной решётке ДО построения
    # базиса, поэтому базис, матрица Грама и все углы считаются уже от
    # привязанных координат; сверх того привязаны и конструкции.
    INTEGER_GRID_SNAP_V1 = "INTEGER_GRID_SNAP_V1"
    # Привязывается только источник, и решётка — та же мировая решётка, но
    # ПЕРЕСЕЧЁННАЯ с собственной плоскостью патча: две координаты садятся в узел
    # как прежде, третья решается из уравнения плоскости. Планарность при этом
    # сохраняется тождественно, а не с точностью до бюджета.
    #
    # Причина замены `SOURCE_ONLY_GRID_SNAP_V1` замерена. Осевая решётка
    # сохраняет планарность точно только у плоскостей, соизмеримых с осями. На
    # наклонном скате (нормаль (0,−0.4525,−0.8918), габарит 3.3125) она двигает
    # вершины на полклетки и оставляет невязку к СОБСТВЕННОЙ плоскости патча
    # 1.27e-06 … 3.69e-03 при бюджете 3.31e-07 — от 3.8 до 11000 бюджетов, и
    # НИ ОДИН допустимый масштаб окна её не спасает (проверено на всех восьми).
    # То есть осевая привязка и near-planar бюджет несовместимы размерностно:
    # бюджет описывает шум ПРЕДСТАВЛЕНИЯ (1e-7 относительно габарита), а
    # привязка есть объявленный сдвиг ГЕОМЕТРИИ на два порядка крупнее.
    #
    # Решётка КАРТЫ (узлы `origin + u·A + v·B`) на эту роль замерена и
    # отвергнута: она тоже сохраняет планарность, но базис карты — два ребра
    # патча, на полевом меше сходящиеся под 12.6°, и в такой косой решётке
    # задуманно прямые углы не восстанавливаются вовсе — 0 из 3 на ВСЕХ
    # масштабах окна против 3 из 3 у мировой. Восстановление прямых углов есть
    # факт МЕТРИКИ, а не целочисленное отношение в координатах карты.
    SOURCE_ONLY_PATCH_PLANE_GRID_SNAP_V2 = "SOURCE_ONLY_PATCH_PLANE_GRID_SNAP_V2"

    @property
    def snaps_source(self) -> bool:
        """Двигает ли закон вершины источника до построения базиса."""

        return self is not GridSnappingLawV1.UNSNAPPED_EXACT_V1

    @property
    def snaps_in_patch_plane(self) -> bool:
        """Живёт ли решётка закона в собственной плоскости патча.

        Отдельным свойством, а не сравнением с именем: от него зависит, обязан
        ли закон оставить источник компланарным, и это обязательство
        проверяется в построителе метрики. Закон, который двигает источник
        мировыми осями, компланарность наклонного патча не сохраняет — и это
        не мнение, а замер (см. комментарий у `SOURCE_ONLY_CHART_GRID_SNAP_V2`).
        """

        return self is GridSnappingLawV1.SOURCE_ONLY_PATCH_PLANE_GRID_SNAP_V2

    @property
    def snaps_constructions(self) -> bool:
        """Двигает ли закон вычисленные точки (`offset_support_g` и пересечения).

        Отдельным свойством, а не сравнением с именем в четырёх местах: разрез
        между двумя привязками — это то, чем закон `SOURCE_ONLY_GRID_SNAP_V1`
        отличается от `INTEGER_GRID_SNAP_V1`, и он обязан быть назван один раз.
        """

        return self is GridSnappingLawV1.INTEGER_GRID_SNAP_V1


class GridWindowOutcomeV1(str, Enum):
    """Существует ли шаг решётки, который чинит вход и не съедает деталь.

    Границ у окна две, поэтому и исходов закрытия два: «границы разошлись» и
    «между границами нет степени двойки» лечатся разным, и сливать их в один
    отказ значило бы потерять причину.
    """

    WINDOW_AVAILABLE = "WINDOW_AVAILABLE"
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"
    NO_POWER_OF_TWO_STEP_IN_WINDOW = "NO_POWER_OF_TWO_STEP_IN_WINDOW"


class GridScaleSearchOrderV1(str, Enum):
    """С какого конца окна перебираются степени двойки.

    Порядок — часть закона, а не деталь реализации: он однозначно определяет,
    какой масштаб будет выбран, поэтому обязан быть записан рядом с выбором.
    Два конца окна названы оба, потому что оба осмысленны и замерены; какой из
    них объявляет ядро — сказано в `source_grid.GRID_SCALE_SEARCH_ORDER`.
    """

    # От самого МЕЛКОГО допустимого шага (наибольший масштаб) к крупному.
    FINEST_ADMISSIBLE_FIRST_V1 = "FINEST_ADMISSIBLE_FIRST_V1"
    # От самого КРУПНОГО допустимого шага (наименьший масштаб) к мелкому.
    COARSEST_ADMISSIBLE_FIRST_V1 = "COARSEST_ADMISSIBLE_FIRST_V1"


class GridScaleTrialOutcomeV1(str, Enum):
    """Чем кончилась проба одного масштаба. Третьего исхода нет."""

    RELATIONS_RESTORED = "RELATIONS_RESTORED"
    RELATIONS_NOT_RESTORED = "RELATIONS_NOT_RESTORED"


@dataclass(frozen=True, slots=True)
class GridScaleTrialV1:
    """Одна проба перебора: что пробовали и что из этого вышло.

    Записывается каждая проба, а не только победившая. Иначе выбор
    невоспроизводим: по картинке в Blender видно, какая геометрия получилась,
    но не видно, почему выбран именно этот шаг, — а он выбран потому, что
    предыдущие пробы отказали, и отказ каждой из них здесь назван числом.
    """

    scale: int
    step: ExactRationalV1
    restored_right_corners: int
    outcome: GridScaleTrialOutcomeV1

    def __post_init__(self) -> None:
        if self.scale <= 0 or self.scale & (self.scale - 1):
            raise ValueError("масштаб пробы — положительная степень двойки")
        if self.restored_right_corners < 0:
            raise ValueError("счётчик восстановленных углов неотрицателен")
        if self.step != ExactRationalV1(1, self.scale):
            raise ValueError("шаг пробы обязан быть обратным её масштабу")


@dataclass(frozen=True, slots=True)
class IntegerGridCertificateV1:
    """Решётка домена: обе границы окна, весь перебор и доказательство.

    Записывается ВСЕГДА, в том числе когда привязки не было: тот же вход с
    другим шагом даёт другой результат, и дайджест обязан их различать.

    `intended_right_corners` — сколько углов патча объявленная авторская
    ошибка числит задуманно прямыми, не будучи таковыми точно.
    `restored_right_corners` — сколько из них дают точно рациональную долю π
    в тех координатах, которые метрика реально несёт. Две названные величины —
    два поля: расхождение между числом названного и числом измеренного само по
    себе дефект.

    `window_step` и `source_scale` у всякого закона, который двигает источник
    (`SOURCE_ONLY_GRID_SNAP_V1`, `INTEGER_GRID_SNAP_V1`), — ВЫБРАННЫЙ
    перебором шаг, то есть последняя проба в `scale_trials`. При
    `UNSNAPPED_EXACT_V1` перебора не было, и это первый кандидат окна: шаг,
    который окно предлагает у своей нижней границы.

    Привязку конструкций сертификат не описывает и описывать не обязан: она
    происходит ниже, в карте, и её решётка выводится из `window_step`
    (`source_grid.chart_grid_for`). Два закона со снятой привязкой источника
    здесь неразличимы намеренно — различие между ними живёт там, где оно
    действует.
    """

    snapping_law: GridSnappingLawV1
    window_outcome: GridWindowOutcomeV1
    patch_extent: ExactRationalV1
    author_angular_error: ExactRationalV1
    decal_detail: ExactRationalV1
    window_lower_bound: ExactRationalV1
    window_upper_bound: ExactRationalV1
    window_step: ExactRationalV1 | None
    source_scale: int | None
    magnitude_bound: int | None
    intended_right_corners: int
    restored_right_corners: int
    search_order: GridScaleSearchOrderV1
    scale_trials: tuple[GridScaleTrialV1, ...]

    def __post_init__(self) -> None:
        if self.intended_right_corners < 0 or self.restored_right_corners < 0:
            raise ValueError("угловые счётчики решётки неотрицательны")
        if self.restored_right_corners > self.intended_right_corners:
            raise ValueError(
                "восстановлено не может быть больше, чем задумано прямыми"
            )
        for trial in self.scale_trials:
            if trial.restored_right_corners > self.intended_right_corners:
                raise ValueError(
                    "проба восстановила больше углов, чем задумано прямыми"
                )
            restored = (
                trial.restored_right_corners == self.intended_right_corners
            )
            if restored is not (
                trial.outcome is GridScaleTrialOutcomeV1.RELATIONS_RESTORED
            ):
                raise ValueError(
                    "исход пробы расходится с её же числом восстановленных"
                )
        # Проверки ниже принадлежат привязке ИСТОЧНИКА, а не одному закону:
        # окно, перебор и доказательство восстановления одинаковы у
        # `SOURCE_ONLY_GRID_SNAP_V1` и `INTEGER_GRID_SNAP_V1`, потому что
        # источник они двигают одинаково, а различаются ниже — привязкой
        # конструкций, которой в этом сертификате нет.
        if not self.snapping_law.snaps_source:
            if self.scale_trials:
                raise ValueError(
                    "UNSNAPPED_EXACT_V1 не перебирает масштабы: проб быть не может"
                )
            return
        if self.window_outcome is not GridWindowOutcomeV1.WINDOW_AVAILABLE:
            raise ValueError(
                f"{self.snapping_law.value} требует открытого окна шага"
            )
        if self.window_step is None or self.source_scale is None:
            raise ValueError("привязка без объявленного шага невозможна")
        # Сертификат, объявляющий восстановление, обязан ему удовлетворять.
        # Иначе «привязка сработала» существует как заявление, которого никто
        # не проверил, — а проверять его больше негде: дальше по конвейеру
        # исходные координаты уже недоступны.
        if self.restored_right_corners != self.intended_right_corners:
            raise ValueError(
                f"{self.snapping_law.value} не восстановил все задуманно "
                "прямые углы"
            )
        self._check_trials()

    def _check_trials(self) -> None:
        """Перебор обязан быть тем самым, который объявлен законом.

        Проверяется не «что-то записано», а четыре свойства объявленного
        закона: перебор непуст, победил ПЕРВЫЙ прошедший, победитель — это и
        есть выбранный шаг, и все пробы шли объявленным порядком внутри окна.
        Без этих проверок запись перебора была бы украшением, а не
        доказательством выбора.
        """

        if not self.scale_trials:
            raise ValueError("привязка без записанного перебора невозможна")
        winner = self.scale_trials[-1]
        if winner.outcome is not GridScaleTrialOutcomeV1.RELATIONS_RESTORED:
            raise ValueError("последняя проба перебора обязана быть прошедшей")
        if winner.scale != self.source_scale or winner.step != self.window_step:
            raise ValueError("выбранный шаг не совпадает с прошедшей пробой")
        if any(
            trial.outcome is GridScaleTrialOutcomeV1.RELATIONS_RESTORED
            for trial in self.scale_trials[:-1]
        ):
            raise ValueError(
                "закон берёт ПЕРВЫЙ прошедший масштаб: прошедшая проба до "
                "победителя означает, что выбран не он"
            )
        scales = [trial.scale for trial in self.scale_trials]
        descending = (
            self.search_order is GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1
        )
        expected = sorted(scales, reverse=descending)
        if scales != expected or len(set(scales)) != len(scales):
            raise ValueError("перебор идёт не тем порядком, который объявлен")
        for trial in self.scale_trials:
            if not (
                self.window_lower_bound.numerator
                * trial.step.denominator
                <= trial.step.numerator * self.window_lower_bound.denominator
                and trial.step.numerator * self.window_upper_bound.denominator
                <= self.window_upper_bound.numerator * trial.step.denominator
            ):
                raise ValueError("проба перебора вышла за границы окна")


class NearPlanarResidualBudgetLawV1(str, Enum):
    # max(relative_extent_factor * max(planar_extent, minimum_extent),
    #     coordinate_ulp_multiplier * max_coordinate_ulp)
    # Исследовано в M-R0 как RUNTIME_PLANAR_RESIDUAL_BUDGET_CANDIDATE_V1.
    RELATIVE_EXTENT_OR_ULP_V1 = "RELATIVE_EXTENT_OR_ULP_V1"


class AffineFrameSelectionLawV1(str, Enum):
    CANONICAL_SOURCE_VERTEX_BASIS_V1 = (
        "CANONICAL_SOURCE_VERTEX_BASIS_V1"
    )


class AffineReconstructionLawV1(str, Enum):
    O_PLUS_U_A_PLUS_V_B_V1 = "O_PLUS_U_A_PLUS_V_B_V1"


class AffineChartOrientationV1(str, Enum):
    COORDINATE_CCW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CCW_MATCHES_OWNER_PATCH"
    )
    COORDINATE_CW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CW_MATCHES_OWNER_PATCH"
    )


class RuntimePredicateFilterLawV1(str, Enum):
    BINARY64_OUTWARD_INTERVAL_V1 = "BINARY64_OUTWARD_INTERVAL_V1"


class RuntimePredicateResultV1(str, Enum):
    CERTIFIED_NEGATIVE = "CERTIFIED_NEGATIVE"
    CERTIFIED_ZERO = "CERTIFIED_ZERO"
    CERTIFIED_POSITIVE = "CERTIFIED_POSITIVE"
    EXACT_FALLBACK_REQUIRED = "EXACT_FALLBACK_REQUIRED"


class RuntimeMetricFallbackLawV1(str, Enum):
    AUTHORITATIVE_REFERENCE_METRIC_V2 = (
        "AUTHORITATIVE_REFERENCE_METRIC_V2"
    )


class MetricSemanticIdentityLawV1(str, Enum):
    EXACT_CONSTRUCTION_CERTIFICATES_ONLY = (
        "EXACT_CONSTRUCTION_CERTIFICATES_ONLY"
    )


@dataclass(frozen=True, slots=True)
class ExactRationalV1:
    numerator: int
    denominator: int

    def __post_init__(self) -> None:
        if type(self.numerator) is not int or type(self.denominator) is not int:
            raise TypeError("ExactRationalV1 requires integer components")
        if self.denominator <= 0:
            raise ValueError("ExactRationalV1 denominator must be positive")
        if gcd(abs(self.numerator), self.denominator) != 1:
            raise ValueError("ExactRationalV1 must be reduced")


@dataclass(frozen=True, slots=True)
class ExactPoint2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactPoint3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactMatrix2V1:
    m00: ExactRationalV1
    m01: ExactRationalV1
    m10: ExactRationalV1
    m11: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactSourceVertexCoordinateV2:
    source_vertex_id: SourceVertexId
    domain_coordinate: ExactPoint2V1


@dataclass(frozen=True, slots=True)
class CertifiedAffineSupportDirectionV2:
    direction_in_domain: ExactVector2V1
    reference_metric_id: ReferenceMetricId


@dataclass(frozen=True, slots=True)
class ExactSourcePlaneCertificateV1:
    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1

    def __post_init__(self) -> None:
        if not self.exact:
            raise ValueError(
                "ExactSourcePlaneCertificateV1 must be exact"
            )


@dataclass(frozen=True, slots=True)
class NearPlanarProjectionCertificateV1:
    """Запись о том, что вход был спроецирован, и на сколько он отклонялся.

    Карта N0 требует: ни один солвер не сглаживает и не проецирует молча, и
    каждая неточная политика записывает масштаб, метод, бюджет невязки и
    ревизию источника. Проекция здесь точная (рациональная); приблизителен
    выбор плоскости, и именно он зафиксирован этими полями.
    """

    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1
    residual_budget_law: NearPlanarResidualBudgetLawV1
    relative_extent_factor: ExactRationalV1
    minimum_extent: ExactRationalV1
    coordinate_ulp_multiplier: int
    planar_extent: ExactRationalV1
    residual_budget: ExactRationalV1
    max_residual: ExactRationalV1
    projected_source_vertex_ids: frozenset[SourceVertexId]

    def __post_init__(self) -> None:
        if self.exact:
            raise ValueError(
                "NearPlanarProjectionCertificateV1 describes a non-exact plane"
            )
        if self.admission_law is not PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1:
            raise ValueError(
                "NearPlanarProjectionCertificateV1 requires "
                "NEAR_PLANAR_PROJECTION_V1"
            )
        if not self.projected_source_vertex_ids:
            raise ValueError(
                "near-planar certificate must name the projected vertices"
            )


@dataclass(frozen=True, slots=True)
class RationalAffinePlanarMetricV2:
    reference_metric_id: ReferenceMetricId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    exact_origin: ExactPoint3V1
    exact_basis_a: ExactVector3V1
    exact_basis_b: ExactVector3V1
    exact_gram_matrix: ExactMatrix2V1
    exact_inverse_gram_matrix: ExactMatrix2V1
    exact_source_vertex_coordinates: frozenset[
        ExactSourceVertexCoordinateV2
    ]
    chart_orientation: AffineChartOrientationV1
    frame_selection_law: AffineFrameSelectionLawV1
    planarity_certificate: (
        ExactSourcePlaneCertificateV1 | NearPlanarProjectionCertificateV1
    )
    source_lineage: frozenset[LineageId]
    grid_certificate: IntegerGridCertificateV1


@dataclass(frozen=True, slots=True)
class Binary64Point2V1:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y)):
            raise ValueError("Binary64Point2V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Point3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Point3V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Vector3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Vector3V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64Matrix2V1:
    m00: float
    m01: float
    m10: float
    m11: float

    def __post_init__(self) -> None:
        if not all(
            isfinite(item)
            for item in (self.m00, self.m01, self.m10, self.m11)
        ):
            raise ValueError("Binary64Matrix2V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64SourceVertexCoordinateV1:
    source_vertex_id: SourceVertexId
    domain_coordinate: Binary64Point2V1


@dataclass(frozen=True, slots=True)
class DerivedBinary64AffineViewV1:
    origin: Binary64Point3V1
    basis_a: Binary64Vector3V1
    basis_b: Binary64Vector3V1
    gram_matrix: Binary64Matrix2V1
    inverse_gram_matrix: Binary64Matrix2V1
    source_vertex_coordinates: frozenset[
        Binary64SourceVertexCoordinateV1
    ]


@dataclass(frozen=True, slots=True)
class RuntimePredicateFilterContractV1:
    filter_law: RuntimePredicateFilterLawV1
    uncertain_result: RuntimePredicateResultV1
    exact_zero_requires_fallback: bool
    semantic_identity_law: MetricSemanticIdentityLawV1

    def __post_init__(self) -> None:
        if (
            self.uncertain_result
            is not RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED
        ):
            raise ValueError(
                "uncertain runtime predicates must require exact fallback"
            )
        if not self.exact_zero_requires_fallback:
            raise ValueError(
                "binary64 zero cannot be semantic authority"
            )


@dataclass(frozen=True, slots=True)
class RuntimeMetricFallbackContractV1:
    fallback_law: RuntimeMetricFallbackLawV1
    authoritative_reference_metric_id: ReferenceMetricId
    rounded_runtime_reconstruction_forbidden: bool

    def __post_init__(self) -> None:
        if not self.rounded_runtime_reconstruction_forbidden:
            raise ValueError(
                "runtime fallback cannot reconstruct exact facts from floats"
            )


@dataclass(frozen=True, slots=True)
class RuntimePlanarMetricV1:
    runtime_metric_id: RuntimeMetricId
    reference_metric_id: ReferenceMetricId
    derived_binary64_view: DerivedBinary64AffineViewV1
    predicate_filter_contract: RuntimePredicateFilterContractV1
    fallback_contract: RuntimeMetricFallbackContractV1

    def __post_init__(self) -> None:
        if (
            self.fallback_contract.authoritative_reference_metric_id
            != self.reference_metric_id
        ):
            raise ValueError(
                "RuntimePlanarMetricV1 must fallback to its reference metric"
            )
