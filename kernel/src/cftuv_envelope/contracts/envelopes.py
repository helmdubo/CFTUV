"""Typed EnvelopeSpec union и alpha-bound EnvelopeInstance records."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum

from ..ids import (
    AngleCertificateId,
    CapSeedId,
    ChainUseId,
    CornerRelationId,
    CornerSeedId,
    DecalRequestId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    FrontSeedId,
    HiddenSupportId,
    JunctionRelationId,
    JunctionSeedId,
    LineageId,
    OwnerSectorId,
    PatchDomainId,
    PerPatchProjectionId,
    RoutePairingId,
    SelectionCertificateId,
    SharedSemanticAnchorId,
    SourceVertexId,
    TerminalRelationId,
)
from ..numeric import CertifiedDecimalIntervalV1, ExactRatioV1
from .metric import ExactRationalV1
from .request import (
    AngularProfileFamilyId,
    AngularProfileSelectionPolicyId,
    MaxSubturnValueId,
)
from .seeds import EndpointRole


class EnvelopeSpecVariant(str, Enum):
    STRIP = "StripEnvelopeSpec"
    ANGULAR = "AngularEnvelopeSpec"
    JUNCTION = "JunctionEnvelopeSpec"
    CAP = "CapEnvelopeSpec"


class StripSupportLawId(str, Enum):
    PLANAR_LINEAR_NORMAL_OFFSET_V1 = "PLANAR_LINEAR_NORMAL_OFFSET_V1"


class StationModelId(str, Enum):
    SEMANTIC_CHAIN_USE_S = "SEMANTIC_CHAIN_USE_S"
    CONSTANT_PHYSICAL_ENDPOINT_S = "CONSTANT_PHYSICAL_ENDPOINT_S"
    PLAN_STATION_FLOW = "PLAN_STATION_FLOW"


class TransverseModelId(str, Enum):
    SIGNED_OWNER_INTERIOR_R = "SIGNED_OWNER_INTERIOR_R"


class TerminalInterfacePolicy(str, Enum):
    REFERENCED_ENVELOPE_SPECS = "REFERENCED_ENVELOPE_SPECS"
    DOMAIN_OR_REQUEST_TERMINAL = "DOMAIN_OR_REQUEST_TERMINAL"


class AngularSubdivisionPolicy(str, Enum):
    EQUAL_REFLEX_EXCESS = "EQUAL_REFLEX_EXCESS"


class HiddenSupportDirectionLaw(str, Enum):
    ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN = (
        "ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN"
    )


class CertifiedBoundHiddenSupportDirectionLawV1(str, Enum):
    CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1 = (
        "CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1"
    )


class AdaptiveBoundHiddenSupportDirectionLawV2(str, Enum):
    """Чем закреплено направление лифтованной скрытой опоры.

    `ADAPTIVE_MINIMAL_RATIONAL_FAN_V2` — минимальная общая высота рационального
    веера в окнах идеального веера (`AdaptiveMinimalRationalFanAuthorityV2`).

    `CANONICAL_RATIONAL_ROTATION_FAN_V1` — фиксированный рациональный поворот
    входящей опоры по таблице (`CanonicalRationalRotationFanAuthorityV1`): луч
    не ищется в окне, а вычисляется, поэтому одинаковые канонические углы
    получают одинаковые веера независимо от шума привязки.
    """

    ADAPTIVE_MINIMAL_RATIONAL_FAN_V2 = (
        "ADAPTIVE_MINIMAL_RATIONAL_FAN_V2"
    )
    CANONICAL_RATIONAL_ROTATION_FAN_V1 = (
        "CANONICAL_RATIONAL_ROTATION_FAN_V1"
    )


class CanonicalFanRaysLawV1(str, Enum):
    """Закон, по которому лучи веера канонического угла — точные рациональные повороты.

    Лифт счёта на каноническом прямом угле при чётном `q` даёт равноугольный
    идеал с иррациональным подшагом (`pi/8` при `H + 1 = 4`, `q = 6`), и
    адаптивный атлас искал рациональный веер на ШУМНОЙ вычислительной
    геометрии: сорок восемь конгруэнтных углов одной стены получали десять
    разных наборов шагов. Закон заменяет поиск вычислением: луч ординала `j` —
    рациональный поворот входящей опоры по таблице (для `pi/8` при `H + 1 = 4`
    — лучи на 22.62, 45 и 67.38 градуса: пары `(12, 5)`, `(1, 1)`, `(5, 12)`),
    знак поворота — из ориентации угла, а остаток шума остаётся в ПОСЛЕДНЕМ
    секторе. Ряд симметричен, поэтому зеркальные углы получают один веер. Гарантия
    `подшаг <= pi/q` проверяется точно на каждом секторе вычислительной
    геометрии, включая последний, и не ослаблена. Нет рационального луча в
    карте или нет записи таблицы — именованный отказ и прежний путь.
    """

    CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1 = (
        "CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1"
    )


class EvaluationGeometrySubturnCountLiftLawV1(str, Enum):
    """Почему счёт скрытых рёбер evaluation-веера выше счёта селекции.

    `EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1` — исходный закон: подшаг на
    предшествующем счёте СТРОГО больше `pi/q` в геометрии вычисления.

    `EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1` — подшаг на
    предшествующем счёте РОВНО `pi/q` (остаток подшага точно ноль), а хотя бы
    один скрытый луч этого веера иррационален. Веер равношаговый, поэтому
    все `H + 1` шагов равны `pi/q` и каждый скрытый луч закреплён поворотом:
    допустимая область — точка, а рационального направления в точке с
    иррациональным лучом нет. Счёт неосуществим точно, без допуска, и
    записан под этим именем, а не растворён в отказе поиска окон.

    `EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1` —
    тот же предел, но на КАНОНИЧЕСКОМ угле: селектор увидел ровно канонический
    интервал (сырой точный либо восстановленный), предшествующий счёт стоит на
    `pi/q` на каноническом веере и его скрытый луч иррационален, а привязка к
    решётке сдвинула вычислительный угол на шум (знак и `cos^2` шума — в самой
    записи лифта, точная граница смещений привязки — в записи
    `EvaluationBindingNoiseOnCanonicalAngleV1`). Решает канонический угол, а
    не знак шума привязки: один и тот же прямой угол получает один счёт,
    округлила его решётка вверх, вниз или никак.
    """

    EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1 = (
        "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1"
    )
    EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1 = (
        "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1"
    )
    EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1 = (
        "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1"
    )


class ExactTurnSignV1(str, Enum):
    NEGATIVE = "NEGATIVE"
    ZERO = "ZERO"
    POSITIVE = "POSITIVE"


class AdaptiveProjectivePoleOwnershipV1(str, Enum):
    NONE = "NONE"
    X_ZERO = "X_ZERO"
    Y_ZERO = "Y_ZERO"


class DirectionBindingReasonV1(str, Enum):
    SOURCE_DIRECTION_IRRATIONAL = "SOURCE_DIRECTION_IRRATIONAL"
    EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL = (
        "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL"
    )


class HiddenSupportScope(str, Enum):
    ANGULAR_ENVELOPE_SPEC_LOCAL = "ANGULAR_ENVELOPE_SPEC_LOCAL"


class AngularExposurePolicy(str, Enum):
    EXPOSED_ONLY_WHEN_ON_RESOLVED_COVERAGE_SILHOUETTE = (
        "EXPOSED_ONLY_WHEN_ON_RESOLVED_COVERAGE_SILHOUETTE"
    )
    MAY_BECOME_INTERNAL_AFTER_EXACT_UNION = "MAY_BECOME_INTERNAL_AFTER_EXACT_UNION"


class JunctionSupportLawId(str, Enum):
    DECLARED_JUNCTION_RELATION_SUPPORTS_V1 = "DECLARED_JUNCTION_RELATION_SUPPORTS_V1"


class MixedAlphaPolicy(str, Enum):
    INCIDENT_EFFECTIVE_ALPHA_VECTOR = "INCIDENT_EFFECTIVE_ALPHA_VECTOR"


class CapClosureLawId(str, Enum):
    PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1 = "PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1"


class ExactTwoPiHandling(str, Enum):
    TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE = (
        "TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE"
    )


class AngularRegressionFixtureId(str, Enum):
    K0 = "LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1"
    K1 = "LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1"


class SelectionLaw(str, Enum):
    MIN_K_FOR_MAX_SUBTURN = "K_EQUALS_MAX_ZERO_CEIL_DELTA_OVER_DELTA_MAX_MINUS_ONE"
    HUBER_EMANATED_DENSITY_FLOOR_V1 = "HUBER_EMANATED_DENSITY_FLOOR_V1"
    # Мягкий излом (δ < 30°) ОДНОЙ цепи источника: `k = 0`, митра прямого
    # скелета, полоса продолжается через вершину. Счёт по плотности этому углу
    # не задаётся; решение и его причина — `CornerTreatmentRecordV1`.
    CORNER_JOIN_SOFT_BEND_V1 = "CORNER_JOIN_SOFT_BEND_V1"
    # Митра на изломе окрестности (`CORNER_MITER_ON_FOLD_V1`, `_corner_fold`): угол, чья окрестность в патче владельца
    # СЛОЖЕНА (`sin^2` двугранного угла кольца-1 свыше бюджета), получает `k = 0` — митру прямого скелета со ШВОМ на
    # биссектрисе, без потока. Счёт по плотности такому углу не задаётся; решение и причина — `CornerTreatmentRecordV1`.
    CORNER_MITER_ON_FOLD_V1 = "CORNER_MITER_ON_FOLD_V1"


#: Законы, под которыми счёт угла `k = 0` решён ЗАКОНОМ УГЛА, а не плотностью: у них нет веера и нет скрытых опор, а
#: интервальная запись сертификата — прежняя запись политики. Одно место перечня на все двери (контракт сертификата,
#: проверяющий плана, компиляция, очередь): закон, добавленный в одно из них и забытый в другом, остался бы углом с
#: `k = 0`, принятым на слово.
ZERO_SUPPORT_SELECTION_LAWS = frozenset(
    {SelectionLaw.CORNER_JOIN_SOFT_BEND_V1, SelectionLaw.CORNER_MITER_ON_FOLD_V1}
)


class MinimalityLowerBound(str, Enum):
    K_ZERO_OR_STRICT_LOWER = "K_EQ_ZERO_OR_K_TIMES_DELTA_MAX_LT_DELTA"
    HUBER_DENSITY_BUCKET_OPEN_LOWER = "HUBER_DENSITY_BUCKET_OPEN_LOWER"


class AdmissibilityUpperBound(str, Enum):
    CLOSED_UPPER = "DELTA_LEQ_K_PLUS_ONE_TIMES_DELTA_MAX"
    HUBER_DENSITY_BUCKET_CLOSED_UPPER = "HUBER_DENSITY_BUCKET_CLOSED_UPPER"


class SelectionCertificateAuthority(str, Enum):
    EXACT_OR_CERTIFIED_ANGLE_COMPARISON = "EXACT_OR_CERTIFIED_ANGLE_COMPARISON"


class CanonicalReflexAngleRelationV1(str, Enum):
    """Каноническое авторское отношение, выраженное символом, а не числом.

    Значение символа — точная доля π, которую несёт рефлексный избыток
    `u = δ/π`. Множество намеренно минимально: оплачен полем ровно прямой
    угол (интерьер 270°, поворот π/2, `u = 1/2`). Граница расширения —
    в `_canonical_angle.CANONICAL_REFLEX_EXCESS_RELATIONS`.
    """

    CANONICAL_REFLEX_EXCESS_PI_OVER_2 = "CANONICAL_REFLEX_EXCESS_PI_OVER_2"


class CanonicalAngleRestorationLawV1(str, Enum):
    AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1 = (
        "AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1"
    )


class AngleTolerancePolicyIdV1(str, Enum):
    """Имя допуска И его категории одним значением.

    Категория `AUTHORING_INTENT` означает: эпсилон применяется У ДВЕРИ, на
    стадии восстановления задуманного отношения, и дальше все решения идут по
    канонизированному факту. Имя — ключ типизированного реестра допусков
    (`TolerancePolicyV1`); сверка идёт тестом реестра. Допуск восстановления
    масштаба художника (0.1 градуса) отделён от `AUTHOR_ANGULAR_ERROR`
    привязки источника к решётке (7e-6 рад) и называется своим именем.
    """

    CANONICAL_RESTORATION_ARTIST_SCALE_V1 = (
        "CANONICAL_RESTORATION_ARTIST_SCALE_V1"
    )


class SubturnGuaranteeLawV1(str, Enum):
    """Против КАКИХ опор доказан жёсткий максимум подшага.

    Законов два, и это разные обещания, а не оттенки одного.

    `SUBTURN_ON_SOURCE_SUPPORTS_V1` — исходный и по-прежнему единственный для
    всех невосстановленных углов: `подшаг <= pi/q` доказан на ФАКТИЧЕСКИХ
    опорах, равношаговым делением измеренного угла. Их поведение эта карточка
    не трогает ни байтом.

    `SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1` — новая власть, и она честно
    слабее на сырых опорах. Обязательство целиком:

    * цель — `pi/q` на КАНОНИЧЕСКОМ угле. Первые `H` лучей веера ставятся
      точными поворотами на `u_канон * pi / (H + 1)`, и это значение не
      превосходит `pi/q` целочисленно (`u_канон * q <= H + 1`), без единого
      сравнения с порогом;
    * на сырых опорах превышение возникает РОВНО в одном, последнем секторе и
      равно остатку `δ_сырое - H * u_канон * pi / (H + 1)` минус канонический
      подшаг, то есть в точности отклонению восстановленного угла
      `Δ <= CANONICAL_RESTORATION_ARTIST_ERROR` (1745e-6 рад, 0.1 градуса с
      недобором). Шум не размазывается по вееру и не усиливается: он остаётся
      там, где и был, — между последним каноническим лучом и сырой опорой;
    * закон применяется ТОЛЬКО там, где старый отказал: если равношаговый
      веер сырого угла уже удовлетворяет `подшаг <= pi/q`, ничего не
      происходит и байты прежние. Это та же форма, что у
      `EvaluationGeometrySubturnCountLiftV1`: осуществимо — молчим,
      неосуществимо — именованная власть с записью.

    Почему это смена власти, а не расширение старой: старое обещание
    буквально ложно на сырых опорах восстановленного угла, и молча
    переопределять его смыслом «ну почти» — ровно то, от чего предостерегает
    аудит. Поэтому имя новое, запись отдельная, а старое имя остаётся за
    старым обещанием.
    """

    SUBTURN_ON_SOURCE_SUPPORTS_V1 = "SUBTURN_ON_SOURCE_SUPPORTS_V1"
    SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1 = (
        "SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1"
    )


@dataclass(frozen=True, slots=True)
class CanonicalSubturnFanAuthorityV1:
    """Именованная власть канонического веера — по углу и по плотности.

    Пишется ТОЛЬКО когда старый закон отказал на сырых опорах, поэтому само
    её присутствие и есть ответ на вопрос «почему этот веер построен иначе».
    Отсутствие записи означает старый закон и прежние байты.
    """

    guarantee_law: SubturnGuaranteeLawV1
    envelope_spec_id: EnvelopeSpecId
    selection_certificate_id: SelectionCertificateId
    canonical_relation: CanonicalReflexAngleRelationV1
    canonical_reflex_excess_over_pi: ExactRatioV1
    hidden_edge_count: int
    max_subturn_q: int
    canonical_subturn_over_pi: ExactRatioV1
    raw_residual_upper_bound_radians: ExactRationalV1
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class CanonicalAngleRestorationCertificateV1:
    """Что именно восстановлено, из чего и на каком основании.

    Восстановление — ИМЕНОВАННОЕ изменение входа селектора, а не молчаливое
    округление, поэтому запись обязана позволять перепроверить решение целиком:
    сырой интервал (`source_reflex_excess_over_pi`) записан рядом с
    канонической долей, а `deviation_upper_bound_radians` — доказанная сверху
    величина отклонения в радианах, которую допуск обязан покрывать.

    Подделка ловится сверкой этих полей с сырым углом снапшота: запись,
    которой не соответствует угол, отвергается именованным исходом.
    """

    restoration_law: CanonicalAngleRestorationLawV1
    selection_certificate_id: SelectionCertificateId
    corner_relation_id: CornerRelationId
    reflex_angle_certificate_id: AngleCertificateId
    canonical_relation: CanonicalReflexAngleRelationV1
    canonical_reflex_excess_over_pi: ExactRatioV1
    source_reflex_excess_over_pi: CertifiedDecimalIntervalV1
    deviation_upper_bound_radians: ExactRationalV1
    tolerance_radians: ExactRationalV1
    tolerance_policy_id: AngleTolerancePolicyIdV1
    proven_predicates: frozenset[str]


class IntervalBoundKind(str, Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"


class BoundaryResolutionState(str, Enum):
    REQUIRED_BEFORE_PATCH_UNION = "REQUIRED_BEFORE_PATCH_UNION"
    BOUNDARY_RESOLVED = "BOUNDARY_RESOLVED"


class EffectiveAlphaBindingKind(str, Enum):
    INCIDENT_FRONT_COMPONENT_VECTOR = "INCIDENT_FRONT_COMPONENT_VECTOR"
    INCIDENT_STRIP_EFFECTIVE_ALPHA = "INCIDENT_STRIP_EFFECTIVE_ALPHA"


@dataclass(frozen=True, slots=True)
class SelectionIntervalCertificateV1:
    lower_bound_kind: IntervalBoundKind
    lower_bound_integer: int
    upper_bound_kind: IntervalBoundKind
    upper_bound_integer: int


@dataclass(frozen=True, slots=True)
class HuberDensitySelectionIntervalCertificateV1:
    """Именованная ячейка `(C-1)/q < u <= C/q` политики Density A."""

    q: int
    bucket_c: int
    lower_bound_kind: IntervalBoundKind
    lower_bound_numerator: int
    upper_bound_kind: IntervalBoundKind
    upper_bound_numerator: int


@dataclass(frozen=True, slots=True)
class AngularProfileSelectionCertificateV1:
    certificate_id: SelectionCertificateId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    corner_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    reflex_angle_certificate_id: AngleCertificateId
    profile_family_id: AngularProfileFamilyId
    selection_policy_id: AngularProfileSelectionPolicyId
    max_subturn_value_id: MaxSubturnValueId
    resolved_hidden_edge_count: int
    resolved_subturn_count: int
    local_profile_support_count: int
    local_profile_segment_count: int
    selection_law: SelectionLaw
    minimality_lower_bound: MinimalityLowerBound
    admissibility_upper_bound: AdmissibilityUpperBound
    selection_interval_certificate: (
        SelectionIntervalCertificateV1
        | HuberDensitySelectionIntervalCertificateV1
    )
    certificate_authority: SelectionCertificateAuthority
    regression_fixture_id: AngularRegressionFixtureId | None


class CornerTreatmentV1(str, Enum):
    """Что делает полоса в вогнутой вершине: продолжается либо идёт профилем."""

    JOIN_CONTINUATION = "CORNER_JOIN_CONTINUATION"
    ANGULAR_PROFILE = "CORNER_ANGULAR_PROFILE"
    # Митра `k = 0` со швом на биссектрисе, потока нет (`CORNER_MITER_ON_FOLD_V1`): складка окрестности угла.
    MITER_SEAM = "CORNER_MITER_SEAM"


class CornerTreatmentReasonV1(str, Enum):
    """Почему угол получил свою обработку. Ровно одна причина на угол."""

    SOFT_BEND_IN_ONE_SOURCE_CHAIN = "SOFT_BEND_IN_ONE_SOURCE_CHAIN"
    REFLEX_EXCESS_NOT_SOFT = "REFLEX_EXCESS_NOT_SOFT"
    REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD = (
        "REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD"
    )
    # Хост не доказал, что два куска — одна цепь ВЛАДЕЛЬЦА угла (нет общей записи
    # `chain-source` его патча): ядро знает «не доказано», а не «цепи разные».
    SOURCE_CHAIN_UNPROVEN = "SOURCE_CHAIN_UNPROVEN"
    # Закон `CORNER_MITER_ON_FOLD_V1`: окрестность угла сложена свыше бюджета, изгиб не шире четверти оборота
    # (ЗАМКНУТЫЙ предел: прямой угол берёт митру) — `MITER_SEAM`.
    FOLDED_NEIGHBOURHOOD_MITER = "FOLDED_NEIGHBOURHOOD_MITER"
    # Окрестность сложена свыше бюджета, но изгиб шире четверти оборота либо не доказан в её пределах: митра
    # неприменима, угол остаётся веером под счётом плотности (на здании `building` вершина 34 патча 89, изгиб ~178 градусов).
    BEND_BEYOND_MITER_BOUND = "BEND_BEYOND_MITER_BOUND"


@dataclass(frozen=True, slots=True)
class CornerTreatmentRecordV1:
    """Запись закона `CORNER_TREATMENT_V1` на один `CornerRelation`.

    `incoming`/`outgoing` — вхождения цепей в порядке сектора владельца;
    `shared_source_lineage_ids` — общие записи `data_record_lineage` двух
    цепей (факт хоста «один кусок одной цепи источника»); `reflex_excess_over_pi`
    — сырой сертифицированный интервал, по которому решался предел изгиба;
    `threshold_over_pi` — сам ПРЕДЕЛ ИЗГИБА закона `CORNER_JOIN_SAME_PCHAIN_V1` (доля π, 1/2 = четверть оборота, исключительный;
    имя поля прежнее, из закона с порогом выбора: переименование сдвинуло бы схему плана).
    """

    treatment_law: str
    corner_relation_id: CornerRelationId
    selection_certificate_id: SelectionCertificateId
    incoming_chain_use_id: ChainUseId
    outgoing_chain_use_id: ChainUseId
    treatment: CornerTreatmentV1
    reason: CornerTreatmentReasonV1
    threshold_over_pi: ExactRatioV1
    reflex_excess_over_pi: CertifiedDecimalIntervalV1
    shared_source_lineage_ids: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class HiddenSupportSpecV1:
    hidden_support_id: HiddenSupportId
    ordinal: int
    turn_fraction: ExactRatioV1
    direction_law: HiddenSupportDirectionLaw
    zero_length_at_alpha_zero: bool
    scope: HiddenSupportScope
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    selection_certificate_id: SelectionCertificateId


@dataclass(frozen=True, slots=True)
class DirectionBindingCertificateV1:
    bound_primitive_integer_vector: tuple[int, int]
    ideal_window_lower_slope_envelope: CertifiedDecimalIntervalV1
    ideal_window_upper_slope_envelope: CertifiedDecimalIntervalV1
    certified_window_width_lower_bound: Decimal
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class EvaluationGeometryDirectionBindingCertificateV1:
    """Сертификат направления, доказанный на записанной evaluation-геометрии."""

    bound_primitive_integer_vector: tuple[int, int]
    ideal_window_lower_slope_envelope: CertifiedDecimalIntervalV1
    ideal_window_upper_slope_envelope: CertifiedDecimalIntervalV1
    certified_window_width_lower_bound: Decimal
    proven_predicates: frozenset[str]
    binding_reason: DirectionBindingReasonV1


@dataclass(frozen=True, slots=True)
class AdaptiveRationalFanOrdinalWindowV2:
    """Рациональная оболочка полного окна и внутренний termination-box."""

    ordinal: int
    use_x_denominator: bool
    denominator_sign: int
    full_lower_slope_envelope: CertifiedDecimalIntervalV1
    full_upper_slope_envelope: CertifiedDecimalIntervalV1
    termination_lower_slope: tuple[int, int]
    termination_upper_slope: tuple[int, int]
    certified_termination_width: tuple[int, int]
    admissible_lower_outward: tuple[int, int]
    admissible_lower_inward: tuple[int, int]
    admissible_upper_inward: tuple[int, int]
    admissible_upper_outward: tuple[int, int]


@dataclass(frozen=True, slots=True)
class AdaptiveRationalFanProjectiveChartPieceV1:
    """Один непересекающийся кусок канонического projective-атласа."""

    piece_index: int
    use_x_denominator: bool
    denominator_sign: int
    lower_slope_envelope: CertifiedDecimalIntervalV1
    upper_slope_envelope: CertifiedDecimalIntervalV1
    lower_endpoint_included: bool
    upper_endpoint_included: bool
    slope_increases_in_ordinal_order: bool
    pole_ownership: AdaptiveProjectivePoleOwnershipV1


@dataclass(frozen=True, slots=True)
class AdaptiveRationalFanOrdinalWindowAtlasV1:
    """Tagged atlas только для окна, пересекающего coordinate-chart poles."""

    ordinal: int
    pieces: tuple[AdaptiveRationalFanProjectiveChartPieceV1, ...]
    termination_piece_index: int
    termination_lower_slope: tuple[int, int]
    termination_upper_slope: tuple[int, int]
    certified_termination_width: tuple[int, int]
    admissible_lower_outward: tuple[int, int]
    admissible_lower_inward: tuple[int, int]
    admissible_upper_inward: tuple[int, int]
    admissible_upper_outward: tuple[int, int]


@dataclass(frozen=True, slots=True)
class AdaptiveFareyHeightRangeWitnessV2:
    """Сжатый integer-свидетель всех высот до победителя."""

    first_height: int
    last_height: int
    primitive_candidate_counts: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class EvaluationGeometrySubturnCountLiftV1:
    """Минимальный evaluation-only подъём H без изменения selection."""

    lift_law: EvaluationGeometrySubturnCountLiftLawV1
    source_selection_certificate_id: SelectionCertificateId
    source_hidden_edge_count: int
    effective_hidden_edge_count: int
    max_subturn_q: int
    evaluation_turn_sign: ExactTurnSignV1
    evaluation_turn_cosine_squared: ExactRatioV1
    minimality_predecessor_hidden_edge_count: int
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class AdaptiveMinimalRationalFanAuthorityV2:
    """Единственная sealed-власть всего Density-веера."""

    authority_id: str
    max_subturn_q: int
    minimal_common_height: int
    exhaustive_previous_height: int
    termination_height_upper_bound: int
    bound_primitive_integer_vectors: tuple[tuple[int, int], ...]
    binding_reasons: tuple[DirectionBindingReasonV1 | None, ...]
    ordinal_windows: tuple[
        AdaptiveRationalFanOrdinalWindowV2
        | AdaptiveRationalFanOrdinalWindowAtlasV1,
        ...,
    ]
    previous_height_witness: AdaptiveFareyHeightRangeWitnessV2
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class CanonicalRationalRotationFanAuthorityV1:
    """Власть веера лифтованного канонического угла: лучи — рациональные повороты.

    Запись НЕ доверяется: проверяющий заново берёт канонический факт из
    интервала угла снапшота, запись таблицы по `(u, H + 1, q)`, входящую опору
    вычислительной геометрии и ориентацию угла, строит веер и сверяет каждый
    `bound_primitive_integer_vectors[j]` с направлением луча `j` на точное
    равенство (нулевой cross и положительный dot), а затем точно проверяет
    `подшаг <= pi/q` на каждом секторе, включая последний. Поля, которых
    проверяющий не может вывести из геометрии, не хранятся.
    """

    authority_id: str
    ray_law: CanonicalFanRaysLawV1
    selection_certificate_id: SelectionCertificateId
    canonical_relation: CanonicalReflexAngleRelationV1
    canonical_reflex_excess_over_pi: ExactRatioV1
    hidden_edge_count: int
    max_subturn_q: int
    ray_rotation_pairs: tuple[tuple[int, int], ...]
    bound_primitive_integer_vectors: tuple[tuple[int, int], ...]
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class CertifiedBoundHiddenSupportSpecV1:
    hidden_support_id: HiddenSupportId
    ordinal: int
    turn_fraction: ExactRatioV1
    direction_law: CertifiedBoundHiddenSupportDirectionLawV1
    zero_length_at_alpha_zero: bool
    scope: HiddenSupportScope
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    selection_certificate_id: SelectionCertificateId
    direction_binding: (
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
    )


@dataclass(frozen=True, slots=True)
class AdaptiveBoundHiddenSupportSpecV2:
    hidden_support_id: HiddenSupportId
    ordinal: int
    turn_fraction: ExactRatioV1
    direction_law: AdaptiveBoundHiddenSupportDirectionLawV2
    zero_length_at_alpha_zero: bool
    scope: HiddenSupportScope
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    selection_certificate_id: SelectionCertificateId
    direction_fan_authority_id: str
    bound_primitive_integer_vector: tuple[int, int]


@dataclass(frozen=True, slots=True)
class StripEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: FrontSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    front_component_ids: frozenset[FrontComponentId]
    support_law_id: StripSupportLawId
    station_model_id: StationModelId
    transverse_model_id: TransverseModelId
    terminal_interface_spec_ids: frozenset[EnvelopeSpecId]
    terminal_interface_policy: TerminalInterfacePolicy


@dataclass(frozen=True, slots=True)
class AngularEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: CornerSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    angle_certificate_id: AngleCertificateId
    selection_certificate_id: SelectionCertificateId
    profile_family_id: AngularProfileFamilyId
    resolved_hidden_edge_count: int
    subdivision_policy: AngularSubdivisionPolicy
    hidden_supports: frozenset[
        HiddenSupportSpecV1 | CertifiedBoundHiddenSupportSpecV1
    ]
    incident_front_component_ids: tuple[FrontComponentId, ...]
    all_support_normal_speed: int
    exposure_policy: AngularExposurePolicy
    mixed_alpha_policy: MixedAlphaPolicy


@dataclass(frozen=True, slots=True)
class AdaptiveDensityAngularEnvelopeSpecV2(AngularEnvelopeSpec):
    """Density Angular spec с одной общей властью рационального веера."""

    hidden_supports: frozenset[AdaptiveBoundHiddenSupportSpecV2]
    direction_fan_authority: (
        AdaptiveMinimalRationalFanAuthorityV2
        | CanonicalRationalRotationFanAuthorityV1
    )
    evaluation_subturn_count_lift: (
        EvaluationGeometrySubturnCountLiftV1 | None
    )


@dataclass(frozen=True, slots=True)
class JunctionEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: JunctionSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    source_relation_id: JunctionRelationId
    incident_front_component_ids: frozenset[FrontComponentId]
    support_law_id: JunctionSupportLawId
    route_pairing_ids: frozenset[RoutePairingId]
    shared_semantic_anchor_id: SharedSemanticAnchorId
    per_patch_projection_id: PerPatchProjectionId | None
    mixed_alpha_policy: MixedAlphaPolicy


@dataclass(frozen=True, slots=True)
class CapEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: CapSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    physical_terminal_source_vertex_id: SourceVertexId
    terminal_relation_id: TerminalRelationId | None
    incident_chain_use_id: ChainUseId
    incident_strip_spec_id: EnvelopeSpecId
    closure_law_id: CapClosureLawId
    endpoint_role: EndpointRole
    station_law: StationModelId
    effective_alpha_binding: EffectiveAlphaBindingKind
    exact_two_pi_handling: ExactTwoPiHandling


EnvelopeSpec = (
    StripEnvelopeSpec
    | AdaptiveDensityAngularEnvelopeSpecV2
    | AngularEnvelopeSpec
    | JunctionEnvelopeSpec
    | CapEnvelopeSpec
)


@dataclass(frozen=True, slots=True)
class EffectiveAlphaBindingV1:
    kind: EffectiveAlphaBindingKind
    front_component_ids: frozenset[FrontComponentId]


@dataclass(frozen=True, slots=True)
class EnvelopeInstanceV1:
    instance_id: EnvelopeInstanceId
    spec_id: EnvelopeSpecId
    spec_variant: EnvelopeSpecVariant
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    effective_alpha_binding: EffectiveAlphaBindingV1
    boundary_resolution_state: BoundaryResolutionState
