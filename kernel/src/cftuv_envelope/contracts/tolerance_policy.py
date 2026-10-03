"""Типизированный реестр ИМЕНОВАННЫХ допусков ядра. Ни одного нового числа.

Почему реестр, а не список чисел
--------------------------------

Верховное правило владельца (`DECISIONS.md`, 2026-08-03) легализовало
эпсилон-допуски как инструмент DCC-ядра и потребовало дисциплины, которая
отличает нас от рассыпанных magic numbers: **у каждого допуска есть ИМЯ, одно
значение и одно место**. Второй этап внешнего аудита (та же дата, пункт 5)
уточнил форму обязательства: не «единый список чисел», а ТИПИЗИРОВАННЫЙ
реестр — потому что список чисел отвечает на вопрос «сколько», а поле и ревью
спрашивают «что этому числу вообще дозволено сделать с ответом».

Отсюда обязательное поле `allowed_effect`. Допуск near-planar-приёма вправе
только принять или отвергнуть проекцию; числовой фильтр — только выбрать между
быстрым и точным путём; кап работы — только превратить бесконечный счёт в
именованный отказ. Запись, которая объявляет одно, а делает другое, — это то
самое «тихое исчезновение», запрещённое п. 4 `AGENTS.md`; реестр делает
расхождение читаемым до того, как оно доедет до поля.

Что в реестр НЕ ходит, и это часть вердикта
-------------------------------------------

* **Производные границы ошибки фильтров** — ширина конкретной оболочки,
  `filter_bits`, `iv.prec`, измеренный `named_epsilon` конуса. Они не
  объявляются, а ВЫЧИСЛЯЮТСЯ на каждом вызове и записываются в сертификат.
  Реестр называет САМ ФИЛЬТР (запись с `bound_law` и `value=None`), потому что
  политика здесь — «доказать знак или уступить точному пути», а не число.
* **Архитектурные бюджеты и потолки** (`MODULE_LINE_ALLOWANCE`, счёт
  публичного API, число md-файлов). Это процессные храповики против роста
  долга, а не геометрические допуски: они ничего не решают о геометрии и
  ничего не могут сделать с ответом ядра. Именованная граница карточки.
* **Нейтральные элементы** (`Fraction(0)`, `Fraction(1)`, `iv.mpf(0)`) —
  инициализаторы накопителей, а не допуски.

Как реестр исполняется
----------------------

`kernel/tests/test_tolerance_policy_registry.py`:

1. каждая запись полна, её `positive_fixture`/`negative_fixture` указывают на
   СУЩЕСТВУЮЩИЕ тесты (проверяется разбором файла, а не строкой);
2. каждая константа «формы допуска» в `kernel/src` (float-литерал либо
   `Fraction`/`Decimal` от литералов) либо объявлена `declaration_sites`
   какой-то записи, либо стоит в замороженном списке исключений с причиной;
3. ни одного ГОЛОГО числового порога в сравнениях геометрических модулей.

Реестр — данные, а не поведение: ни один модуль ядра его не импортирует, и
импортировать не должен. Числа живут там, где они применяются (иначе цикл
импортов и второй экземпляр величины); реестр ССЫЛАЕТСЯ на них по имени и
сверяется с ними тестом. Поэтому появление этого файла не двигает ни байта
поведения — что и требовалось карточкой.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .metric import ExactRationalV1


class TolerancePolicyCategoryV1(str, Enum):
    """Категории вердикта аудита §11 плюс `WORK_BUDGET` от принципала.

    Категория отвечает на вопрос «чем это число вообще является», и от неё
    зависит, какой `allowed_effect` для записи законен.

    `BROAD_PHASE_MARGIN` и `UI_HYSTERESIS` объявлены и пусты, и это
    зафиксировано тестом, а не умолчанием: широкая фаза ядра (AABB пар
    треугольников в `_embedding`, кандидатные пары аранжировки) работает БЕЗ
    запаса — целочисленным точным тестом, — а гистерезиса в v1 нет вовсе
    (`docs/decal_corner_bands.md`). Пустая категория с исполняемым «пусто»
    честнее отсутствующей: первая же попытка завести запас найдёт себе имя.
    """

    PRODUCT_ADMISSION = "PRODUCT_ADMISSION"
    AUTHORING_INTENT = "AUTHORING_INTENT"
    STRUCTURAL_QUANTIZATION = "STRUCTURAL_QUANTIZATION"
    NUMERIC_ERROR_BOUND = "NUMERIC_ERROR_BOUND"
    BROAD_PHASE_MARGIN = "BROAD_PHASE_MARGIN"
    UI_HYSTERESIS = "UI_HYSTERESIS"
    WORK_BUDGET = "WORK_BUDGET"


class TolerancePolicyIdV1(str, Enum):
    """Имена допусков. Ключ реестра и единственная законная ссылка на допуск.

    `CANONICAL_RESTORATION_ARTIST_SCALE_V1` намеренно совпадает буква в
    букву с членом `AngleTolerancePolicyIdV1`: докстринг того перечисления
    обещал, что имя станет ключом реестра, когда реестр появится. Обещание
    исполняется сверкой в тесте, а не совпадением по памяти.
    """

    PRODUCT_SKIRT_ABSOLUTE_V1 = "PRODUCT_SKIRT_ABSOLUTE_V1"
    NEAR_PLANAR_WIDTH_DISTORTION_RELATIVE_V1 = (
        "NEAR_PLANAR_WIDTH_DISTORTION_RELATIVE_V1"
    )
    DEVELOPABLE_STRETCH_RELATIVE_V1 = "DEVELOPABLE_STRETCH_RELATIVE_V1"
    SURFACE_LIFT_EXTRAPOLATION_CELLS_V1 = "SURFACE_LIFT_EXTRAPOLATION_CELLS_V1"
    NEAR_PLANAR_REPRESENTATION_NOISE_V1 = "NEAR_PLANAR_REPRESENTATION_NOISE_V1"
    CANONICAL_RESTORATION_ARTIST_SCALE_V1 = (
        "CANONICAL_RESTORATION_ARTIST_SCALE_V1"
    )
    AUTHOR_ANGULAR_ERROR_SOURCE_GRID_INTENT_V1 = (
        "AUTHOR_ANGULAR_ERROR_SOURCE_GRID_INTENT_V1"
    )
    SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1 = (
        "SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1"
    )
    PI_RATIONAL_UPPER_BOUND_V1 = "PI_RATIONAL_UPPER_BOUND_V1"
    DECAL_DETAIL_V1 = "DECAL_DETAIL_V1"
    SOURCE_GRID_STEP_V1 = "SOURCE_GRID_STEP_V1"
    SQRT_SUM_INTERVAL_PREFILTER_V1 = "SQRT_SUM_INTERVAL_PREFILTER_V1"
    SYMBOLIC_INTERVAL_PREFILTER_V1 = "SYMBOLIC_INTERVAL_PREFILTER_V1"
    DENSITY_FAN_PREPARATION_WORK_CAP_V1 = "DENSITY_FAN_PREPARATION_WORK_CAP_V1"
    DENSITY_EXACT_WORK_CAP_V1 = "DENSITY_EXACT_WORK_CAP_V1"
    FACTORIZATION_MEMO_ENTRIES_V1 = "FACTORIZATION_MEMO_ENTRIES_V1"
    KNOWN_PRIME_REGISTRY_ENTRIES_V1 = "KNOWN_PRIME_REGISTRY_ENTRIES_V1"
    COPRIME_BASIS_SPLIT_BUDGET_V1 = "COPRIME_BASIS_SPLIT_BUDGET_V1"
    EXACT_CANONICALIZATION_WORK_CAP_V1 = "EXACT_CANONICALIZATION_WORK_CAP_V1"
    EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1 = (
        "EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1"
    )
    SOURCE_VERTEX_LIFT_BUDGET_V1 = "SOURCE_VERTEX_LIFT_BUDGET_V1"
    CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1 = (
        "CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1"
    )
    CLIP_DIAGONAL_CHORD_DEPTH_V1 = "CLIP_DIAGONAL_CHORD_DEPTH_V1"
    CORNER_JOIN_SOFT_BEND_THRESHOLD_V1 = "CORNER_JOIN_SOFT_BEND_THRESHOLD_V1"
    ADAPTIVE_FAN_NARROW_ROTATION_BAND_V1 = "ADAPTIVE_FAN_NARROW_ROTATION_BAND_V1"
    CLIP_SOURCE_VERTEX_CORNER_SNAP_CELLS_V1 = "CLIP_SOURCE_VERTEX_CORNER_SNAP_CELLS_V1"
    CLIP_NODE_SOURCE_EDGE_GAP_CELLS_V1 = "CLIP_NODE_SOURCE_EDGE_GAP_CELLS_V1"


class TolerancePolicyUnitsV1(str, Enum):
    """Единицы. `NONE` — у записей, чья политика есть закон, а не число."""

    METRES = "METRES"
    RADIANS = "RADIANS"
    RADIANS_PER_HALF_TURN = "RADIANS_PER_HALF_TURN"
    DIMENSIONLESS = "DIMENSIONLESS"
    CHART_LATTICE_CELLS = "CHART_LATTICE_CELLS"
    EXACT_WORK_UNITS = "EXACT_WORK_UNITS"
    MEMO_ENTRIES = "MEMO_ENTRIES"
    NONE = "NONE"


class TolerancePolicyCoordinateSpaceV1(str, Enum):
    """В каком пространстве число вообще имеет смысл.

    Допуск в метрах источника и допуск в единицах карты — разные величины, и
    сравнивать их нельзя. `NOT_A_COORDINATE` — у капов работы и у фильтров:
    они живут в счёте предикатов, а не в геометрии.
    """

    SOURCE_LOCAL_INTRINSIC = "SOURCE_LOCAL_INTRINSIC"
    SOURCE_ANGLE_MEASURE = "SOURCE_ANGLE_MEASURE"
    CHART_LATTICE = "CHART_LATTICE"
    NOT_A_COORDINATE = "NOT_A_COORDINATE"


class TolerancePolicyScalingLawV1(str, Enum):
    """Как величина ведёт себя при изменении размера входа."""

    ABSOLUTE_INDEPENDENT_OF_EXTENT = "ABSOLUTE_INDEPENDENT_OF_EXTENT"
    RELATIVE_TO_PATCH_EXTENT_WITH_FLOOR = "RELATIVE_TO_PATCH_EXTENT_WITH_FLOOR"
    DERIVED_FROM_PATCH_EXTENT_AND_DECAL_DETAIL = (
        "DERIVED_FROM_PATCH_EXTENT_AND_DECAL_DETAIL"
    )
    NOT_SCALED = "NOT_SCALED"


class TolerancePolicyAppliedStageV1(str, Enum):
    """ОДНА дверь, у которой допуск применяется.

    Смысл поля — исполнить правило владельца «допуски применяются У ДВЕРИ,
    внутри — согласованные решения на очищенных данных». Две двери у одного
    числа — это две записи реестра, а не одна с оговоркой.
    """

    NEAR_PLANAR_ADMISSION = "NEAR_PLANAR_ADMISSION"
    DEVELOPABLE_ADMISSION = "DEVELOPABLE_ADMISSION"
    SURFACE_LIFT_POINT_LOCATION = "SURFACE_LIFT_POINT_LOCATION"
    NEAR_PLANAR_CERTIFICATE_RECOMPUTATION = (
        "NEAR_PLANAR_CERTIFICATE_RECOMPUTATION"
    )
    SOURCE_GRID_WINDOW = "SOURCE_GRID_WINDOW"
    SOURCE_GRID_SCALE_SELECTION = "SOURCE_GRID_SCALE_SELECTION"
    CANONICAL_ANGLE_RESTORATION = "CANONICAL_ANGLE_RESTORATION"
    CANONICAL_SUBTURN_FAN_CONSTRUCTION = "CANONICAL_SUBTURN_FAN_CONSTRUCTION"
    CANONICAL_COUNT_DECISION_ON_BINDING_NOISE = (
        "CANONICAL_COUNT_DECISION_ON_BINDING_NOISE"
    )
    EXACT_SIGN_PREFILTER = "EXACT_SIGN_PREFILTER"
    DENSITY_FAN_PREPARATION = "DENSITY_FAN_PREPARATION"
    DENSITY_MINIMAL_HEIGHT_SEARCH = "DENSITY_MINIMAL_HEIGHT_SEARCH"
    EXACT_CANONICALIZATION_MEMORY = "EXACT_CANONICALIZATION_MEMORY"
    EXACT_CANONICALIZATION_TRANSACTION = "EXACT_CANONICALIZATION_TRANSACTION"
    SOURCE_VERTEX_LIFT_AT_HOST_POSITION = "SOURCE_VERTEX_LIFT_AT_HOST_POSITION"
    SOURCE_FACE_CLIP_AT_DIAGONALS = "SOURCE_FACE_CLIP_AT_DIAGONALS"
    CORNER_TREATMENT_BEFORE_COUNT_LAW = "CORNER_TREATMENT_BEFORE_COUNT_LAW"
    DENSITY_NARROW_BAND_RAY_BINDING = "DENSITY_NARROW_BAND_RAY_BINDING"
    CLIP_SOURCE_VERTEX_AT_TRIANGULATION_CORNER = "CLIP_SOURCE_VERTEX_AT_TRIANGULATION_CORNER"
    CLIP_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE = "CLIP_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE"


class TolerancePolicyAllowedEffectV1(str, Enum):
    """ПРИНУДИТЕЛЬНОЕ поле: что именно допуску дозволено сделать с ответом.

    Читать так: всё, чего здесь не написано, допуску запрещено. Проверка —
    негативная фикстура записи: она обязана показывать ГРАНИЦУ дозволенного,
    а не просто ещё один зелёный путь.
    """

    ADMIT_OR_REJECT_PROJECTION = "ADMIT_OR_REJECT_PROJECTION"
    ADMIT_OR_REJECT_UNFOLDED_CHART = "ADMIT_OR_REJECT_UNFOLDED_CHART"
    EXTEND_NEAREST_TRIANGLE_WITHIN_BOUND = "EXTEND_NEAREST_TRIANGLE_WITHIN_BOUND"
    RECOMPUTE_DECLARED_CERTIFICATE_ONLY = "RECOMPUTE_DECLARED_CERTIFICATE_ONLY"
    ADMIT_OR_REJECT_GRID_SCALE = "ADMIT_OR_REJECT_GRID_SCALE"
    BOUND_GRID_WINDOW_OR_NAME_IT_CLOSED = "BOUND_GRID_WINDOW_OR_NAME_IT_CLOSED"
    REPLACE_MEASURE_WITH_CANONICAL_FACT = "REPLACE_MEASURE_WITH_CANONICAL_FACT"
    BUILD_FAN_ON_CANONICAL_SUPPORTS = "BUILD_FAN_ON_CANONICAL_SUPPORTS"
    DECIDE_COUNT_ON_CANONICAL_ANGLE = "DECIDE_COUNT_ON_CANONICAL_ANGLE"
    NARROW_ADMITTED_SET_TOWARD_REFUSAL = "NARROW_ADMITTED_SET_TOWARD_REFUSAL"
    CHOOSE_FAST_OR_EXACT_PATH = "CHOOSE_FAST_OR_EXACT_PATH"
    NAMED_REFUSAL_ONLY = "NAMED_REFUSAL_ONLY"
    CHANGE_COST_NEVER_THE_ANSWER = "CHANGE_COST_NEVER_THE_ANSWER"
    LIFT_SOURCE_VERTEX_TO_HOST_POSITION_WITHIN_BOUND = (
        "LIFT_SOURCE_VERTEX_TO_HOST_POSITION_WITHIN_BOUND"
    )
    KEEP_SOURCE_FACE_WHOLE_ACROSS_DIAGONAL_WITHIN_CHORD_DEPTH = (
        "KEEP_SOURCE_FACE_WHOLE_ACROSS_DIAGONAL_WITHIN_CHORD_DEPTH"
    )
    JOIN_SOFT_BEND_OF_ONE_SOURCE_CHAIN = "JOIN_SOFT_BEND_OF_ONE_SOURCE_CHAIN"
    BIND_FAN_RAYS_WITHIN_NARROW_BAND_OF_THE_EQUAL_STEP_IDEAL = (
        "BIND_FAN_RAYS_WITHIN_NARROW_BAND_OF_THE_EQUAL_STEP_IDEAL"
    )
    SNAP_SOURCE_VERTEX_TO_TRIANGULATION_CORNER_WITHIN_GAP = (
        "SNAP_SOURCE_VERTEX_TO_TRIANGULATION_CORNER_WITHIN_GAP"
    )
    ZERO_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE_WITHIN_GAP = (
        "ZERO_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE_WITHIN_GAP"
    )


class TolerancePolicyPipelineStageV1(str, Enum):
    """Продакшн-путь или только предпросмотр.

    У допуска, который живёт лишь в превью, цена ошибки другая; смешивать их в
    одном списке — значит потерять это различие. Сегодня все записи ядра —
    `FINAL_PRODUCT_PATH`: превью ядра считает тем же кодом.
    """

    FINAL_PRODUCT_PATH = "FINAL_PRODUCT_PATH"
    PREVIEW_ONLY = "PREVIEW_ONLY"


@dataclass(frozen=True, slots=True)
class TolerancePolicyV1:
    """Одна именованная политика допуска.

    Поля `value` и `bound_law` — взаимоисключающие по построению (проверяется
    тестом): либо политика ЕСТЬ число, либо она есть закон, дающий границу на
    каждом вызове. Третьего не бывает: запись без числа и без закона ничего не
    объявляет.

    `declaration_sites` — точки, где значение физически лежит. Поле не
    украшение: без него реестр нельзя СВЕРИТЬ с кодом, и он снова становится
    прозой. Архитектурный тест идёт от кода к реестру именно по этим строкам.
    """

    id: TolerancePolicyIdV1
    category: TolerancePolicyCategoryV1
    value: ExactRationalV1 | None
    bound_law: str | None
    units: TolerancePolicyUnitsV1
    coordinate_space: TolerancePolicyCoordinateSpaceV1
    scaling_law: TolerancePolicyScalingLawV1
    scope: str
    authority: str
    applied_stage: TolerancePolicyAppliedStageV1
    allowed_effect: TolerancePolicyAllowedEffectV1
    changes_topology: bool
    preview_or_final: TolerancePolicyPipelineStageV1
    telemetry_counters: tuple[str, ...]
    declaration_sites: tuple[str, ...]
    positive_fixture: str
    negative_fixture: str


@dataclass(frozen=True, slots=True)
class TolerancePolicyRegistryV1:
    """Реестр целиком — одна перечислимая запись под схемой."""

    registry_law: str
    policies: tuple[TolerancePolicyV1, ...]


TOLERANCE_POLICY_REGISTRY_SCHEMA_V1 = "cftuv.envelope.tolerance_policy_registry.v1"

TOLERANCE_POLICY_REGISTRY_LAW_V1 = "NAMED_TOLERANCE_POLICY_REGISTRY_V1"


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


_KERNEL_TESTS = "kernel/tests"


TOLERANCE_POLICIES_V1: tuple[TolerancePolicyV1, ...] = (
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.PRODUCT_SKIRT_ABSOLUTE_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=_rational(Fraction(1, 80)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.METRES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Насколько источник патча вправе отклониться от собственной "
            "плоскости и всё ещё получить near-planar проекцию. Один и тот же "
            "допуск на оба закона решётки: продуктовый вопрос «сколько "
            "кривизны стерпит юбка декали» не зависит от того, снапился "
            "источник или нет."
        ),
        authority=(
            "NearPlanarResidualBudgetLawV1.PRODUCT_SKIRT_ABSOLUTE_V1; "
            "DECISIONS.md 2026-08-01 (власть допуска объявлена продуктовой) и "
            "2026-08-03 (ПРОДУКТОВЫЙ ФАКТ владельца: юбка декали, 1.25 см)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.NEAR_PLANAR_ADMISSION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.ADMIT_OR_REJECT_PROJECTION
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.contracts.metric.PRODUCT_SKIRT_ABSOLUTE_BUDGET",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_policy.py"
            "::test_the_product_budget_admits_roof_curvature_and_still_refuses_real_bends"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_policy.py"
            "::test_source_beyond_the_budget_is_refused_by_name"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.NEAR_PLANAR_REPRESENTATION_NOISE_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=None,
        bound_law="RELATIVE_EXTENT_OR_ULP_V1",
        units=TolerancePolicyUnitsV1.METRES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=(
            TolerancePolicyScalingLawV1.RELATIVE_TO_PATCH_EXTENT_WITH_FLOOR
        ),
        scope=(
            "Шум ПРЕДСТАВЛЕНИЯ: сколько binary64 врёт о задуманной плоскости. "
            "Закон вытеснен из ВЫБОРА допуска решением владельца, но остался "
            "членом перечисления и остался пересчитываемым: три его величины "
            "пишутся в каждый near-planar сертификат, и валидатор обязан "
            "воспроизвести объявленное число из записанных полей. Числа три, "
            "поэтому политика объявлена законом, а не одним значением."
        ),
        authority=(
            "NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1; "
            "validation_metric._recomputed_budget"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.NEAR_PLANAR_CERTIFICATE_RECOMPUTATION
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.RECOMPUTE_DECLARED_CERTIFICATE_ONLY
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.planar_metric._RELATIVE_EXTENT_FACTOR",
            "cftuv_envelope.planar_metric._MINIMUM_EXTENT",
            "cftuv_envelope.planar_metric._COORDINATE_ULP_MULTIPLIER",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_snapshot_validation.py"
            "::test_the_validator_admits_what_its_own_builder_writes"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_snapshot_validation.py"
            "::test_a_budget_law_that_does_not_reproduce_the_number_is_refused"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CANONICAL_RESTORATION_ARTIST_SCALE_V1,
        category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
        value=_rational(Fraction(1745, 10**6)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Восстановление задуманного ОТНОШЕНИЯ сторон до селектора "
            "плотности: угол, отклонившийся от прямого не более чем на "
            "допуск масштаба художника (1745e-6 рад = 0.099985 градуса, "
            "недобор до 0.1), заменяется СИМВОЛИЧЕСКИМ каноническим фактом. "
            "Ниже по конвейеру решения идут на точной рациональной доле π, а "
            "не на интервале. Поле: шум моделирования 0.005–0.03 градуса "
            "(вершина 245 двери `building` патча 10, 90.0053 градуса, за "
            "прежним допуском 0.0004 градуса переходила границу замкнутой "
            "ячейки d2: H=2 вместо H=1), честные почти прямые углы начинаются "
            "с 0.4 градуса (`building.004` патч 0, 90.56) и допуском не "
            "покрываются: они идут сырым числом и называются в записях "
            "корня счёта. ГАРАНТИЯ ПОДШАГА МЯГКАЯ: на восстановленном тугом "
            "угле (d2 `H = 1`, d4) последний сектор может превысить `pi/q` до "
            "0.1 градуса плюс шум привязки к решётке (граница шума — запись "
            "EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1, 1/400); остальные "
            "секторы — точные повороты и проверяются точно."
        ),
        authority=(
            "AngleTolerancePolicyIdV1.CANONICAL_RESTORATION_ARTIST_SCALE_V1; "
            "CanonicalAngleRestorationLawV1."
            "AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1; "
            "DECISIONS.md 2026-08-03 (восстановление канонического авторского "
            "угла вместо сдвига границ ячеек) и 2026-10-03 (RIGHT-ANGLE-STABLE: "
            "допуск восстановления поднят с 7e-6 рад до масштаба художника)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CANONICAL_ANGLE_RESTORATION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.REPLACE_MEASURE_WITH_CANONICAL_FACT
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope._authoring_intent.CANONICAL_RESTORATION_ARTIST_ERROR",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_wall_2_001_noise_is_inside_the_authoring_intent_tolerance"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_an_honest_near_right_corner_is_outside_the_tolerance"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.AUTHOR_ANGULAR_ERROR_SOURCE_GRID_INTENT_V1,
        category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
        value=_rational(Fraction(7, 10**6)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Тот же допуск у ВТОРОЙ двери: восстановление задуманного "
            "ПОЛОЖЕНИЯ вершины. Угол, который авторская ошибка числит "
            "задуманно прямым, после привязки к решётке обязан дать точно "
            "рациональную долю π — иначе масштаб отвергается. Две двери у "
            "одного числа — две записи; значение и место при этом одно, "
            "поэтому копии величины не существует."
        ),
        authority=(
            "source_grid.select_grid_scale, GridScaleSearchOrderV1."
            "FINEST_ADMISSIBLE_FIRST_V1; DECISIONS.md 2026-07-25 (решение "
            "владельца о величине авторской ошибки и детали декали)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.SOURCE_GRID_SCALE_SELECTION,
        allowed_effect=TolerancePolicyAllowedEffectV1.ADMIT_OR_REJECT_GRID_SCALE,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope._authoring_intent.AUTHOR_ANGULAR_ERROR",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_grid_wiring.py"
            "::test_the_field_mesh_restores_every_intended_right_corner"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_grid_wiring.py"
            "::test_an_axis_drift_of_one_millimetre_is_not_an_intended_right_corner"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1,
        category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
        value=_rational(Fraction(1745, 10**6)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "ВТОРАЯ дверь допуска восстановления: лучи веера ставятся точными "
            "поворотами на канонический подшаг, а невязка между канонической "
            "опорой и сырой ограничена тем же допуском восстановления "
            "(предикат RESIDUAL_IS_BOUNDED_BY_THE_AUTHORING_INTENT_TOLERANCE). "
            "Власть применяется ТОЛЬКО там, где закон на сырых опорах отказал; "
            "где старый проходит, не меняется ни байта."
        ),
        authority=(
            "SubturnGuaranteeLawV1."
            "SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1; "
            "CanonicalSubturnFanAuthorityV1; DECISIONS.md 2026-08-03 (мандат "
            "выпуска: система аудитора утверждена владельцем)"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.CANONICAL_SUBTURN_FAN_CONSTRUCTION
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.BUILD_FAN_ON_CANONICAL_SUPPORTS
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope._authoring_intent.CANONICAL_RESTORATION_ARTIST_ERROR",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_canonical_fan_authority_is_recorded_only_where_the_source_law_failed"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_canonical_fan_authority_on_a_feasible_source_fan_is_a_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.PI_RATIONAL_UPPER_BOUND_V1,
        category=TolerancePolicyCategoryV1.NUMERIC_ERROR_BOUND,
        value=_rational(Fraction(3141592653589793239, 10**18)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS_PER_HALF_TURN,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Допуск объявлен в радианах, а мера приходит в долях π — сравнение "
            "требует π. Берётся ДОКАЗАННАЯ рациональная верхняя граница, "
            "поэтому принимаемое множество строго ВНУТРИ объявленного допуска: "
            "ошибаемся в сторону отказа, не в сторону восстановления. Это "
            "объявленная константа геометрического модуля, а не производная "
            "граница фильтра, поэтому она в реестре, а `filter_bits` и "
            "`iv.prec` — нет."
        ),
        authority=(
            "_canonical_angle.PI_RATIONAL_UPPER_BOUND; граница доказывается "
            "интервальной оболочкой в тесте, а не объявляется"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CANONICAL_ANGLE_RESTORATION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.NARROW_ADMITTED_SET_TOWARD_REFUSAL
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope._canonical_angle.PI_RATIONAL_UPPER_BOUND",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_pi_upper_bound_is_proven_strictly_above_pi"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_an_honest_near_right_corner_is_outside_the_tolerance"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.DECAL_DETAIL_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(Fraction(1, 100)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.METRES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Объявленная мельчайшая деталь декали. Задаёт ВЕРХНЮЮ границу окна "
            "шага решётки: шаг крупнее детали стирал бы то, ради чего декаль "
            "существует. Нижнюю границу задаёт авторская угловая ошибка на "
            "габарите патча; окно может закрыться, и это именованный отказ."
        ),
        authority=(
            "DECISIONS.md 2026-07-25 (решение владельца: деталей мельче "
            "сантиметра в игровом пространстве нет); robust.snapping."
            "grid_window_for_patch"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.SOURCE_GRID_WINDOW,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.BOUND_GRID_WINDOW_OR_NAME_IT_CLOSED
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=("cftuv_envelope._authoring_intent.DECAL_DETAIL",),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_robust_snapping.py"
            "::test_building_002_scale_has_a_window"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_robust_snapping.py"
            "::test_a_large_patch_closes_the_window_and_that_is_a_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SOURCE_GRID_STEP_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=None,
        bound_law="POWER_OF_TWO_STEP_IN_WINDOW_FINEST_ADMISSIBLE_FIRST_V1",
        units=TolerancePolicyUnitsV1.METRES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=(
            TolerancePolicyScalingLawV1.DERIVED_FROM_PATCH_EXTENT_AND_DECAL_DETAIL
        ),
        scope=(
            "Шаг решётки источника — допуск РАВЕНСТВА ТОЧЕК, применённый "
            "согласованно (снап = epsilon у двери). Числа у политики нет по "
            "построению: шаг выбирается на каждый патч перебором степеней "
            "двойки внутри окна объявленным порядком, и весь перебор идёт в "
            "сертификат. Ни одной подходящей степени — именованный отказ "
            "NO_GRID_SCALE_RESTORES_RELATIONS, а не «взять ближайший»."
        ),
        authority=(
            "GridSnappingLawV1 + GridScaleSearchOrderV1."
            "FINEST_ADMISSIBLE_FIRST_V1; IntegerGridCertificateV1 записывает "
            "каждую пробу"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.SOURCE_GRID_SCALE_SELECTION,
        allowed_effect=TolerancePolicyAllowedEffectV1.ADMIT_OR_REJECT_GRID_SCALE,
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "SNAP_COUNTS.points_total",
            "SNAP_COUNTS.points_moved",
            "SNAP_COUNTS.merged_points",
            "SNAP_DISPLACEMENT_MAX_SQUARED",
        ),
        declaration_sites=(
            "cftuv_envelope.source_grid.GRID_SCALE_SEARCH_ORDER",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_grid_wiring.py"
            "::test_the_field_search_records_every_scale_it_tried_not_only_the_winner"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_grid_wiring.py"
            "::test_no_scale_in_the_window_is_a_named_refusal_not_a_silent_choice"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SQRT_SUM_INTERVAL_PREFILTER_V1,
        category=TolerancePolicyCategoryV1.NUMERIC_ERROR_BOUND,
        value=None,
        bound_law="EXACT_INTEGER_ENCLOSURE_ISQRT_V1",
        units=TolerancePolicyUnitsV1.NONE,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Знак суммы корней. `sqrt(m)` заключается между `isqrt(m<<2b)/2^b` "
            "и следующим узлом — граница вычислена ТОЧНО, порога в ней нет. "
            "Фильтр либо доказывает знак, либо уступает точному сопряжению; "
            "тождества он не доказывает никогда. Производная ширина оболочки и "
            "`filter_bits` в реестр не ходят — они вычисляются, а не "
            "объявляются."
        ),
        authority=(
            "exact_sqrt_sum.SqrtSumV1.enclosure/certified_sign; AGENTS.md "
            "«фильтр, который либо доказывает, либо уступает точному пути, не "
            "требует церемоний»"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.EXACT_SIGN_PREFILTER,
        allowed_effect=TolerancePolicyAllowedEffectV1.CHOOSE_FAST_OR_EXACT_PATH,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "SIGN_COUNTS.closed_by_enclosure",
            "SIGN_COUNTS.closed_by_conjugation",
            "SIGN_COUNTS.total",
        ),
        declaration_sites=(),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_wavefront_motorcycle_graph.py"
            "::test_the_graph_never_falls_through_to_the_expensive_sign_solver"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_wavefront_exact_time.py"
            "::test_conjugation_decides_when_the_enclosure_gives_up"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SYMBOLIC_INTERVAL_PREFILTER_V1,
        category=TolerancePolicyCategoryV1.NUMERIC_ERROR_BOUND,
        value=None,
        bound_law="MPMATH_IV_STRICT_ENCLOSURE_V1",
        units=TolerancePolicyUnitsV1.NONE,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Знак символьного выражения. Каждая операция расширяет интервал "
            "наружу, поэтому истинное значение гарантированно внутри: если "
            "оболочка не содержит нуля, знак ДОКАЗАН. Настоящий ноль и всё, "
            "что оболочка не разделяет, уходит в точный символьный путь без "
            "изменений."
        ),
        authority=(
            "reference.planar_types.interval_enclosure и "
            "_certified_interval_sign"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.EXACT_SIGN_PREFILTER,
        allowed_effect=TolerancePolicyAllowedEffectV1.CHOOSE_FAST_OR_EXACT_PATH,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=("SYMBOLIC_FALLBACK_COUNTS.sign",),
        declaration_sites=(),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_exact_numeric_fast_path.py"
            "::test_interval_filter_never_contradicts_the_exact_path"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_exact_numeric_fast_path.py"
            "::test_interval_filter_refuses_to_decide_a_true_zero"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.DENSITY_FAN_PREPARATION_WORK_CAP_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 16),
        bound_law=None,
        units=TolerancePolicyUnitsV1.EXACT_WORK_UNITS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Точная работа ПОДГОТОВКИ власти веера — всё, что считается до "
            "поиска D*. Без капа legacy-развёртка ordinal-окна и позитивный "
            "свидетель termination-box уходят в счёт без исхода, а это хуже "
            "отказа (AGENTS.md п.4). Измерено: зелёный корпус ядра — максимум "
            "32 единицы, полевые слепки — максимум 36; литерал выбран по ЦЕНЕ "
            "ПОТОЛКА (~0.3 s), а не по марже."
        ),
        authority=(
            "reference.adaptive_density_atlas.DENSITY_FAN_PREPARATION_WORK_CAP; "
            "именованный отказ DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.DENSITY_FAN_PREPARATION,
        allowed_effect=TolerancePolicyAllowedEffectV1.NAMED_REFUSAL_ONLY,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "DENSITY_FAN_SHELL_PROBES",
            "DENSITY_FAN_ORDER_STEPS",
        ),
        declaration_sites=(
            "cftuv_envelope.reference.adaptive_density_atlas."
            "DENSITY_FAN_PREPARATION_WORK_CAP",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_adaptive_density_fan_authority.py"
            "::test_fan_preparation_cap_leaves_the_domain_below_it_untouched"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_adaptive_density_fan_authority.py"
            "::test_fan_preparation_work_is_capped_by_a_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.DENSITY_EXACT_WORK_CAP_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 17),
        bound_law=None,
        units=TolerancePolicyUnitsV1.EXACT_WORK_UNITS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Точная работа одной транзакции поиска минимального D*. Считается "
            "только точная работа: wall-clock запрещён, потому что время не "
            "воспроизводимо побитово, а число выполненных exact-предикатов "
            "воспроизводимо на любой машине. Кап — самая глубокая зелёная "
            "полевая власть (9 035 единиц) с запасом в 14 раз."
        ),
        authority=(
            "reference.adaptive_density_fan._DENSITY_EXACT_WORK_CAP и "
            "DensityExactWorkBudget; именованный отказ "
            "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.DENSITY_MINIMAL_HEIGHT_SEARCH
        ),
        allowed_effect=TolerancePolicyAllowedEffectV1.NAMED_REFUSAL_ONLY,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "DENSITY_FAN_SHELL_PROBES",
            "DENSITY_FAN_ORDER_STEPS",
        ),
        declaration_sites=(
            "cftuv_envelope.reference.adaptive_density_fan."
            "_DENSITY_EXACT_WORK_CAP",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_adaptive_density_fan_authority.py"
            "::test_deepest_green_field_authority_stays_far_below_the_work_cap"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_adaptive_density_fan_authority.py"
            "::test_exact_work_cap_turns_endless_density_search_into_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.FACTORIZATION_MEMO_ENTRIES_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 13),
        bound_law=None,
        units=TolerancePolicyUnitsV1.MEMO_ENTRIES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Ёмкость памяти разложений. Разложение на простые ЕДИНСТВЕННО, "
            "поэтому мемоизация не может изменить ответ — она меняет только "
            "путь. Переполнение вытесняет запись и возвращает прежний дорогой "
            "путь; сброс памяти обязан давать те же кортежи."
        ),
        authority="exact_sqrt_sum._FACTORIZATION_MEMO_ENTRIES",
        applied_stage=(
            TolerancePolicyAppliedStageV1.EXACT_CANONICALIZATION_MEMORY
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.CHANGE_COST_NEVER_THE_ANSWER
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.exact_sqrt_sum._FACTORIZATION_MEMO_ENTRIES",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_basis_of_the_field_universe_removes_the_expensive_factorizations"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_memory_reset_returns_the_same_answers_again"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.KNOWN_PRIME_REGISTRY_ENTRIES_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 13),
        bound_law=None,
        units=TolerancePolicyUnitsV1.MEMO_ENTRIES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Ёмкость реестра доказанных простых. Переполнение очищает реестр "
            "ЦЕЛИКОМ: половинчатый реестр остался бы корректным, но "
            "непредсказуемым по цене, а полный сброс воспроизводим."
        ),
        authority="exact_sqrt_sum._KNOWN_PRIME_REGISTRY_ENTRIES",
        applied_stage=(
            TolerancePolicyAppliedStageV1.EXACT_CANONICALIZATION_MEMORY
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.CHANGE_COST_NEVER_THE_ANSWER
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.exact_sqrt_sum._KNOWN_PRIME_REGISTRY_ENTRIES",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_field_prime_universe_is_bit_for_bit_identical_with_and_without_basis"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_memory_reset_returns_the_same_answers_again"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.EXACT_CANONICALIZATION_WORK_CAP_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 23),
        bound_law=None,
        units=TolerancePolicyUnitsV1.EXACT_WORK_UNITS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Точная работа ОДНОЙ транзакции домена — подготовки и покрытия "
            "вместе. Единицы только детерминированные (модульные возведения, "
            "gcd, раунды Миллера—Рабина, попытки Полларда, материализации "
            "канонических радикалов, гидратации точных позиций); wall-clock "
            "запрещён, потому что секунда есть свойство машины, а не входа. "
            "До капа факторизация радиканда с большим простым делителем "
            "уходила в счёт без исхода — полевые десять минут без имени. "
            "Превышение не меняет ни одного ответа: оно превращает "
            "бесконечный счёт в именованный отказ. Литерал ограничен с двух "
            "сторон измерением: снизу — walls.012 density 1, вход, который до "
            "PERF-CANON-1 не возвращался 900+ s (запас 249x); сверху — цена "
            "самого потолка, 4.5 s на радиканде полевой ширины 246 бит. Запас "
            "над худшим измеренным ЗДОРОВЫМ доменом (walls.012 d0, 554 580 "
            "единиц на 6ce0227) — 15.1x."
        ),
        authority=(
            "exact_sqrt_sum._EXACT_CANONICALIZATION_WORK_CAP и "
            "ExactWorkBudgetV1; именованный отказ "
            "EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED, доезжающий до "
            "ConveyorOutcome одноимённым членом; DECISIONS.md 2026-08-04 "
            "(открытый счёт Q-FACTORIZATION-WORK-BUDGET приёмки PERF-CANON-1)"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.EXACT_CANONICALIZATION_TRANSACTION
        ),
        allowed_effect=TolerancePolicyAllowedEffectV1.NAMED_REFUSAL_ONLY,
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "EXACT_WORK_MODULAR_SQUARINGS",
            "EXACT_WORK_GCD_OPERATIONS",
            "EXACT_WORK_MILLER_RABIN_ROUNDS",
            "EXACT_WORK_POLLARD_ATTEMPTS",
            "EXACT_WORK_RADICAL_MATERIALIZATIONS",
            "EXACT_WORK_EXACT_POSITION_HYDRATIONS",
            "EXACT_WORK_SPENT",
        ),
        declaration_sites=(
            "cftuv_envelope.exact_sqrt_sum._EXACT_CANONICALIZATION_WORK_CAP",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_exact_work_budget.py"
            "::test_the_budget_does_not_move_a_single_answer"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_exact_work_budget.py"
            "::test_a_lowered_cap_turns_endless_factorization_into_a_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.COPRIME_BASIS_SPLIT_BUDGET_V1,
        category=TolerancePolicyCategoryV1.WORK_BUDGET,
        value=_rational(1 << 12),
        bound_law=None,
        units=TolerancePolicyUnitsV1.EXACT_WORK_UNITS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.NOT_A_COORDINATE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Бюджет расщеплений при построении взаимно простого базиса. Цикл "
            "конечен и без бюджета (каждое расщепление строго уменьшает сумму "
            "элементов); бюджет — явно названная верхняя граница. По его "
            "исчерпании базис перестаёт быть взаимно простым, но его "
            "назначение — только УЗНАТЬ простые, и ответ от этого не зависит."
        ),
        authority="exact_sqrt_sum._COPRIME_BASIS_SPLIT_BUDGET",
        applied_stage=(
            TolerancePolicyAppliedStageV1.EXACT_CANONICALIZATION_MEMORY
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.CHANGE_COST_NEVER_THE_ANSWER
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.exact_sqrt_sum._COPRIME_BASIS_SPLIT_BUDGET",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_coprime_basis_generates_exactly_the_same_primes"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_exact_canonicalization_memory.py"
            "::test_field_radicands_are_bit_for_bit_identical_with_and_without_basis"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.NEAR_PLANAR_WIDTH_DISTORTION_RELATIVE_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=_rational(Fraction(1, 50)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Во сколько раз декаль на поверхности источника вправе быть шире, "
            "чем на плоскости карты: `1 + b`, `b = 1/50`. Условие приёма "
            "near-planar при укладке на треугольники источника: наименьший "
            "`cos²` наклона треугольника к плоскости карты не меньше "
            "`1/(1+b)²`. Относительный допуск свойства поверхности, а не "
            "сантиметры: он не зависит от размера патча."
        ),
        authority=(
            "NearPlanarWidthDistortionLawV1.INTRINSIC_WIDTH_RELATIVE_V1; "
            "DECISIONS.md 2026-10-02 (КРИВИЗНА, ПЕРВАЯ СТУПЕНЬ: решение "
            "владельца — 2 % ширины, относительный)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.NEAR_PLANAR_ADMISSION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.ADMIT_OR_REJECT_PROJECTION
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.contracts.metric.NEAR_PLANAR_WIDTH_BUDGET",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_width_distortion.py"
            "::test_a_gentle_slope_is_within_the_width_budget"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_near_planar_width_distortion.py"
            "::test_a_steep_slope_is_beyond_the_width_budget_by_name"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.DEVELOPABLE_STRETCH_RELATIVE_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=_rational(Fraction(1, 5)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Во сколько раз длина вдоль поверхности источника вправе отличаться от "
            "длины на привязанной к решётке карте развёртки, В ОБЕ СТОРОНЫ: `1 + b`, "
            "`b = 1/5` (20 %). Условие приёма развёртки: оба квадрата сингулярных чисел "
            "отображения треугольник источника -> треугольник карты лежат в "
            "`[1/(1+b)^2, (1+b)^2]`, что решается тремя знаками рациональных "
            "чисел (корни `det(G_c - lambda G_s)`), без корней и допуска "
            "вычисления. Невязка веера недевелопабельной вершины и шум привязки "
            "решётки карты входят в растяжение, а не обходят его. Тот же бюджет и "
            "тот же суд применяются к карте ЛЮБОГО из двух предложений положений: "
            "шарнирного и (только после его именованного отказа) ARAP "
            "`ARAP_LOCAL_GLOBAL_80_BINARY64_V1`; бюджет не зависит от того, "
            "какое предложение дало карту. Относительный "
            "допуск свойства поверхности, а не сантиметры."
        ),
        authority=(
            "DevelopableStretchLawV1.EXACT_GRAM_SINGULAR_VALUE_BAND_V1; DECISIONS.md "
            "2026-10-03 (КРИВИЗНА, СТУПЕНЬ 2: S1 DEVELOPABLE_UNFOLDED_V1; бюджет "
            "1/50 -> 1/5 решением владельца «Устраивают растяжения до 20%»)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.DEVELOPABLE_ADMISSION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.ADMIT_OR_REJECT_UNFOLDED_CHART
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope.contracts.metric.DEVELOPABLE_STRETCH_BUDGET",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_developable_unfold.py"
            "::test_a_gentle_bevel_is_within_the_stretch_budget"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_developable_unfold.py"
            "::test_a_cone_with_an_interior_apex_is_beyond_the_stretch_budget_by_name"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SURFACE_LIFT_EXTRAPOLATION_CELLS_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=_rational(2),
        bound_law=None,
        units=TolerancePolicyUnitsV1.CHART_LATTICE_CELLS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.CHART_LATTICE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "На сколько ячеек решётки карты точка покрытия вправе выйти за "
            "привязанную триангуляцию источника, чтобы подняться ПРОДОЛЖЕНИЕМ "
            "ближайшего треугольника (барицентрические веса с отрицательным "
            "значением). Две ячейки: вершина триангуляции и вершина полигона "
            "покрытия — два независимых округления одной точки (каждое не "
            "больше половины ячейки по оси), расходятся не больше чем на √2 "
            "ячейки. Ячейка — шаг решётки в ЕДИНИЦАХ КАРТЫ; физически при "
            "приведённом репере 0.0006-0.067 мм (`building`, `building.004`), "
            "то есть допуск до 0.13 мм. Измерено: наибольший выход 1.0186 ячейки. Дальше — "
            "именованный отказ SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION. "
            "Если допустимых треугольников больше одного, побеждает ближайший по "
            "ТОЧНОМУ квадрату выхода, равенство решает меньшее имя; оба случая "
            "считаются и называются в диагностике."
        ),
        authority=(
            "materialize.lift_surface.EXTRAPOLATION_CELL_BOUND; DECISIONS.md "
            "2026-10-03 (NEAR-PLANAR-V2, КОММИТ 4: приведённый базис и допуск "
            "выхода за триангуляцию)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.SURFACE_LIFT_POINT_LOCATION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.EXTEND_NEAREST_TRIANGLE_WITHIN_BOUND
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "MATERIALIZE_SURFACE_LIFT_EXTRAPOLATED_POINTS",
            "MATERIALIZE_SURFACE_LIFT_CONTINUATION_AMBIGUOUS_CANDIDATES",
            "MATERIALIZE_SURFACE_LIFT_CONTINUATION_EXACT_TIES",
        ),
        declaration_sites=(
            "cftuv_envelope.materialize.lift_surface.EXTRAPOLATION_CELL_BOUND",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_materialize_lift_surface.py"
            "::test_a_point_within_the_bound_outside_is_extended_and_counted"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_materialize_lift_surface.py"
            "::test_a_point_beyond_the_bound_outside_is_a_named_refusal"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(Fraction(1, 400)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Шум привязки вершин к решётке на КАНОНИЧЕСКОМ угле. На тугом "
            "пороге (`u*q == H+1`, d2 и d4) знак этого шума решал счёт "
            "точного прямого угла; закон решает канон: счёт идёт по "
            "каноническому вееру, лучи — точные повороты. Число — ОБЪЯВЛЕННЫЙ "
            "синус углового шума: sin поворота направления каждого из двух "
            "рёбер при привязке и отклонение поворота между опорами от "
            "канонического угла не больше 1/400 (около 0.143 градуса; "
            "проверяется точно, в квадратах, без корней). Граница накрывает "
            "допуск восстановления (синус 0.1 градуса — 1.745e-3) плюс шум "
            "привязки к решётке поля (до 3e-4). Рядом структурное условие: боковой сдвиг "
            "каждого ребра не больше одной ячейки решётки — сам по себе он "
            "угол не ограничивает, поэтому угловое условие отдельное. Шум вне "
            "границ закону не принадлежит: ответ решает прежний закон, а "
            "отказ называется в диагностике и в счётчике."
        ),
        authority=(
            "EvaluationBindingNoiseLawV1.EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1; "
            "EvaluationBindingNoiseOnCanonicalAngleV1; DECISIONS.md 2026-10-03 "
            "(FAN-CANONICAL-COUNT: визуальная согласованность канонических "
            "углов важнее строгого шага на шуме привязки)"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.CANONICAL_COUNT_DECISION_ON_BINDING_NOISE
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.DECIDE_COUNT_ON_CANONICAL_ANGLE
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "CONVEYOR_EXACT_LIMIT_LIFTED_FANS",
            "CONVEYOR_BINDING_NOISE_LAW_REFUSED",
        ),
        declaration_sites=(
            "cftuv_envelope.reference.evaluation_binding_noise."
            "NOISE_DIRECTION_SINE_BOUND",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_evaluation_binding_noise.py"
            "::test_a_right_angle_gets_one_count_whatever_the_sign_of_the_binding_noise"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_evaluation_binding_noise.py"
            "::test_a_short_edge_passes_the_cell_gate_and_is_refused_by_the_angle"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.SOURCE_VERTEX_LIFT_BUDGET_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(1),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=(
            TolerancePolicyScalingLawV1.DERIVED_FROM_PATCH_EXTENT_AND_DECAL_DETAIL
        ),
        scope=(
            "На сколько ЯЧЕЕК ИСТОЧНИКА домена (`grid_certificate.window_step`, "
            "метры) подъём узла вершины `src:` вправе отстоять от позиции вершины "
            "исходника, чтобы вершина была положена В ЭТУ ПОЗИЦИЮ (один binary64 на "
            "вершину во всех доменах — условие сварки меша соседних доменов). Одна "
            "ячейка: привязка источника двигает точку не больше чем на половину "
            "ячейки по оси, то есть на `h*sqrt(3)/2 < h`, а решётка карты не крупнее "
            "(`chart_grid_for`); дальше ячейки расхождение — не шум привязки, а "
            "другое положение вершины (внутренность объявленной прямой цепи, "
            "сдвинутая вдоль хорды), и она остаётся на узле под именем "
            "SOURCE_VERTEX_DISPLACED_BY_LATTICE. Положенная вершина вправе сойти с "
            "носителя подъёма на величину бюджета: грани от четырёх вершин плоские с "
            "точностью до одной ячейки (а их UV аффинен по карте, не по подвинутым "
            "позициям), и наибольшее отклонение пишется счётчиком. Контур, "
            "который положенная вершина перевернула бы, остаётся на узлах под именем "
            "SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION; выпущенная грань с "
            "перевернувшимся ухом режется на уши под своим счётчиком."
        ),
        authority=(
            "materialize.source_lift.SOURCE_VERTEX_LIFT_BUDGET_CELLS; закон "
            "SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1; DECISIONS.md 2026-10-03 "
            "(DECAL-WELD C2: вершины общей цепи соседних доменов сшиваются по "
            "семантической ссылке при побитовом равенстве позиций)"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.SOURCE_VERTEX_LIFT_AT_HOST_POSITION
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.LIFT_SOURCE_VERTEX_TO_HOST_POSITION_WITHIN_BOUND
        ),
        changes_topology=False,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "MATERIALIZE_SOURCE_VERTICES_LIFTED_AT_HOST",
            "MATERIALIZE_SOURCE_VERTICES_DISPLACED_BY_LATTICE",
            "MATERIALIZE_SOURCE_VERTICES_HOST_POSITION_UNAVAILABLE",
            "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION",
            "MATERIALIZE_FACES_MAX_OFF_PLANE_NANOMETRES",
            "MATERIALIZE_FACES_TRIANGULATED_AFTER_SOURCE_LIFT",
            "MATERIALIZE_TRIANGLES_FLIPPED_BY_SOURCE_LIFT",
        ),
        declaration_sites=(
            "cftuv_envelope.materialize.source_lift.SOURCE_VERTEX_LIFT_BUDGET_CELLS",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_materialize_source_lift.py"
            "::test_a_vertex_within_the_budget_is_lifted_at_the_exact_host_position"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_materialize_source_lift.py"
            "::test_a_vertex_beyond_the_budget_stays_and_is_named"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=None,
        bound_law="CANONICAL_ROTATION_TABLE_ROW_EXACTLY_WITHIN_MAX_SUBTURN_V1",
        units=TolerancePolicyUnitsV1.NONE,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Лучи веера ЛИФТОВАННОГО канонического угла (d4: `u = 1/2`, "
            "`H + 1 = 4`, `q = 6`). Равноугольный идеал `pi/8` иррационален, "
            "адаптивный атлас искал рациональный веер на шумной геометрии, и "
            "конгруэнтные углы получали разные лучи. Закон ставит лучи по "
            "таблице рациональных поворотов входящей опоры: углы 22.62, 45 и "
            "67.38 градуса вместо 22.5, 45 и 67.5 — квантование канонического "
            "подшага в рациональное направление. Числа в законе нет: каждая "
            "строка таблицы проверена тестом точно (порядок, симметрия, потолок "
            "сектора `pi/q`), а на КАЖДОМ веере точно проверяются подшаг и "
            "порядок лучей на вычислительной геометрии, включая последний "
            "сектор с шумом привязки. Нет записи таблицы, луч иррационален в "
            "карте или проверка провалилась — отказ назван, веер ищет прежний "
            "атлас."
        ),
        authority=(
            "CanonicalFanRaysLawV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1; "
            "CanonicalRationalRotationFanAuthorityV1; DECISIONS.md 2026-10-03 "
            "(CANONICAL-FAN-RAYS: конгруэнтные канонические углы получают один "
            "веер, решение владельца)"
        ),
        applied_stage=(
            TolerancePolicyAppliedStageV1.CANONICAL_SUBTURN_FAN_CONSTRUCTION
        ),
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.BUILD_FAN_ON_CANONICAL_SUPPORTS
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "CONVEYOR_CANONICAL_FAN_RAYS_FANS",
            "CONVEYOR_CANONICAL_FAN_RAYS_LAW_REFUSED",
        ),
        declaration_sites=(
            "cftuv_envelope._density_policy.CANONICAL_ROTATION_TABLE",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_fan_rays.py"
            "::test_every_lifted_canonical_corner_carries_the_table_fan"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_fan_rays.py"
            "::test_a_fan_that_breaks_the_subturn_guarantee_is_named_and_not_placed"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CLIP_DIAGONAL_CHORD_DEPTH_V1,
        category=TolerancePolicyCategoryV1.PRODUCT_ADMISSION,
        value=_rational(Fraction(1, 200)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.METRES,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Насколько кусок грани декали, лежащий в ОДНОЙ грани источника поперёк диагонали её "
            "триангуляции, вправе отстоять от поверхности источника над той же точкой карты, чтобы "
            "диагональ его не резала. Диагональ четырёхгранья — ребро триангуляции хоста, в меше "
            "источника её нет, и закон SOURCE_FACES_CLIPPED_V1 режет только по рёбрам меша; но "
            "непланарная грань даёт кусок со складкой, и его глубина (звучная оценка по выпуклой "
            "оболочке вершин куска: излом `|k|·a·b/(a+b)` у четырёхгранья, `2ρ` у выпуклой грани из "
            "трёх и более треугольников) не должна превышать допуск. Четверть смещения хоста "
            "(0.02 м -> 5 мм). УМОЛЧАНИЕ, РЕШЕНИЕ ВЛАДЕЛЬЦА ЖДЁТ: число меняется одной строкой "
            "константы. Ячейка, у которой хоть один кусок глубже допуска, режется по диагонали, как "
            "под SOURCE_TRIANGLES_CLIPPED_V1 (названа счётчиком, наибольшая глубина записана); точно "
            "планарная грань диагональю не режется никогда."
        ),
        authority=(
            "materialize.clip_cells.CLIP_DIAGONAL_CHORD_BUDGET; NearPlanarLiftLawV1."
            "SOURCE_FACES_CLIPPED_V1; DECISIONS.md 2026-10-03 (CLIP_BY_SOURCE_FACES_V1: «лишние рёбра» "
            "на кривых декалях, диагонали четырёхгранья)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.SOURCE_FACE_CLIP_AT_DIAGONALS,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.KEEP_SOURCE_FACE_WHOLE_ACROSS_DIAGONAL_WITHIN_CHORD_DEPTH
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "MATERIALIZE_CLIP_DIAGONAL_FACES_KEPT_WHOLE",
            "MATERIALIZE_CLIP_DIAGONAL_PIECES_ACROSS",
            "MATERIALIZE_CLIP_DIAGONAL_CUTS_AVOIDED",
            "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_NOT_PLANAR",
            "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_UNMERGEABLE",
            "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_KEPT_NANOMETRES",
            "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_OVER_BUDGET_NANOMETRES",
        ),
        declaration_sites=(
            "cftuv_envelope.materialize.clip_cells.CLIP_DIAGONAL_CHORD_BUDGET",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_clip_faces_law.py"
            "::test_a_piece_within_the_chord_budget_stays_one_face_across_the_diagonal"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_clip_faces_law.py"
            "::test_a_piece_beyond_the_chord_budget_cuts_the_face_by_its_triangles_and_names_it"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CORNER_JOIN_SOFT_BEND_THRESHOLD_V1,
        category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
        value=_rational(Fraction(1, 4)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Порог мягкого излома ОДНОЙ цепи источника, доля π рефлексного избытка: 1/4 = 45° (решение "
            "владельца 2026-10-03: сначала 1/6 = 30°, тот же CORNER_ANGLE_THRESHOLD_DEG главного UV-солвера, "
            "затем 1/4 по его жалобе на веера на изломах 31–36° плоской стены). Вогнутый угол "
            "между двумя кусками одной цепи хоста (общая запись `chain-source` ЕГО патча), чей СЕРТИФИЦИРОВАННЫЙ "
            "интервал δ/π лежит строго ниже порога, получает `k = 0` (митра прямого скелета) под законом "
            "CORNER_JOIN_SOFT_BEND_V1, и материализатор ведёт полосу сквозь угол одним потоком (u "
            "непрерывна, шва нет). Интервал поверх порога и угол от порога идут прежним законом счёта "
            "под именами REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD / REFLEX_EXCESS_NOT_SOFT; каждый угол "
            "несёт запись CornerTreatmentRecordV1, проверяющий пересчитывает её по сырому снапшоту."
        ),
        authority=(
            "_corner_treatment.JOIN_THRESHOLD_OVER_PI; SelectionLaw.CORNER_JOIN_SOFT_BEND_V1; "
            "DECISIONS.md 2026-10-03 (JOIN-FLOW: мягкий излом < 30° одной цепи — митра вместо фаски)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CORNER_TREATMENT_BEFORE_COUNT_LAW,
        allowed_effect=TolerancePolicyAllowedEffectV1.JOIN_SOFT_BEND_OF_ONE_SOURCE_CHAIN,
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "STATION_JOIN_CORNERS",
            "STATION_FLOWS",
            "STATION_SKIP_JOIN_CORNER_NOT_ADJACENT",
            "STATION_FLOW_CYCLES_OPENED",
            "MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX",
            "MATERIALIZE_QUADS_UV_BILINEAR",
        ),
        declaration_sites=(
            "cftuv_envelope._corner_treatment.JOIN_THRESHOLD_OVER_PI",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_corner_join.py"
            "::test_a_soft_bend_in_one_source_chain_joins"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_corner_join.py"
            "::test_hard_and_uncertain_bends_keep_the_profile_by_name"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.ADAPTIVE_FAN_NARROW_ROTATION_BAND_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(Fraction(1, 57)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.DIMENSIONLESS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "Полуширина окна ординала адаптивного веера вокруг луча РАВНОУГОЛЬНОГО идеала, "
            "тангенс: 1/57 (1.005 градуса). Прежнее окно — окно Вороного между серединами углов "
            "с соседними лучами — шириной в полшага веера, а поиск минимальной общей высоты ставил "
            "луч на прямую наименьшей высоты решётки карты вне зависимости от равного шага (d1 "
            "`building`: 23 набора шагов у 167 прямых углов, 57.8/32.2, 46.8/43.2, 36/54…). "
            "Полоса — то же окно при ФАНТОМНЫХ соседях идеала на ±2·omega (рациональный поворот "
            "3248/114/3250, радикалов не добавляет): луч привязывается к рациональному направлению "
            "в пределах omega от равного шага, победитель тот же (минимальная общая высота, затем "
            "ближайший к идеалу). Подшаг `<= pi/q` проверяется ТОЧНО по настоящим соседям идеала и "
            "полосой не ослаблен. Власть называет закон окна предикатом "
            "ADAPTIVE_FAN_NARROW_ROTATION_BAND; байты власти окна Вороного заморожены. Отказ полосы "
            "(исчерпание точной работы, неустановимый чарт, пустое окно) назван диагностикой и "
            "счётчиком, веер ищет прежнее окно Вороного."
        ),
        authority=(
            "adaptive_density_band.ADAPTIVE_FAN_NARROW_ROTATION_BAND; compile.FAN_WINDOW_LAW; "
            "DECISIONS.md 2026-10-03 (RIGHT-ANGLE-STABLE: лучи равны шагу идеала, а не ближайшей "
            "простой прямой решётки; решение владельца, принятое оркестратором)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.DENSITY_NARROW_BAND_RAY_BINDING,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.BIND_FAN_RAYS_WITHIN_NARROW_BAND_OF_THE_EQUAL_STEP_IDEAL
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=("CONVEYOR_FAN_NARROW_BAND_REFUSED",),
        declaration_sites=(
            "cftuv_envelope.reference.adaptive_density_band."
            "ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_fan_narrow_band.py"
            "::test_a_bound_ray_stays_within_the_band_of_the_equal_step_ideal"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_fan_narrow_band.py"
            "::test_a_band_that_cannot_hold_a_ray_is_a_named_refusal_and_the_voronoi_window_decides"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CLIP_SOURCE_VERTEX_CORNER_SNAP_CELLS_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(4),
        bound_law=None,
        units=TolerancePolicyUnitsV1.CHART_LATTICE_CELLS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.CHART_LATTICE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "На сколько ячеек решётки карты вершина `src:` многоугольника домена вправе отстоять от угла "
            "привязанной триангуляции подъёма, чтобы резка (ClipStageV1) поставила её В угол до первой стадии. "
            "Образ вершины источника привязывается к решётке дважды независимо (мост домена и карта подъёма), "
            "у вершины объявленной прямой цепи ещё и сдвиг вдоль хорды: замер `sagging_wall` — до 2.83 ячейки "
            "(две по оси). Резка принимает многоугольник, только если площади кусков сходятся ТОЧНО, а такая "
            "вершина площадь не сводит: 47 из 49 не-секторных треугольников домена — уши свеса, шовные уши и "
            "уши «вершина не в углу». Четыре ячейки — запас над измеренным; дальше — другая геометрия, и "
            "вершина остаётся с прежним путём. Привязка точная (квадрат расстояния на SqrtSumV1 под бюджетом); "
            "отказы названы: угол занят другой вершиной либо двумя `src:` (слияние вершин), в допуск попали два "
            "угла и больше. Привязанные точки идут во все последующие шаги домена (подъём, уши выпущенных "
            "кусков, положение вершин `src:`), поэтому сварка с соседом по `location:src:` не расходится."
        ),
        authority=(
            "materialize.clip_snap.SOURCE_VERTEX_CORNER_SNAP_CELLS; DECISIONS.md 2026-10-03 (SNAP-NOISE: "
            "привязка шума решётки перед резкой)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CLIP_SOURCE_VERTEX_AT_TRIANGULATION_CORNER,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.SNAP_SOURCE_VERTEX_TO_TRIANGULATION_CORNER_WITHIN_GAP
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "MATERIALIZE_CLIP_SOURCE_VERTICES_SNAPPED_TO_CORNER",
            "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_NANOMETRES",
            "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_MILLICELLS",
            "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNER_TAKEN",
            "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNERS_AMBIGUOUS",
        ),
        declaration_sites=(
            "cftuv_envelope.materialize.clip_snap.SOURCE_VERTEX_CORNER_SNAP_CELLS",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_clip_snap.py"
            "::test_a_source_vertex_within_the_gap_of_a_corner_snaps_to_it"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_clip_snap.py"
            "::test_a_source_vertex_beyond_the_gap_keeps_its_point"
        ),
    ),
    TolerancePolicyV1(
        id=TolerancePolicyIdV1.CLIP_NODE_SOURCE_EDGE_GAP_CELLS_V1,
        category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
        value=_rational(1),
        bound_law=None,
        units=TolerancePolicyUnitsV1.CHART_LATTICE_CELLS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.CHART_LATTICE,
        scaling_law=TolerancePolicyScalingLawV1.NOT_SCALED,
        scope=(
            "На сколько ячеек решётки карты вершина `node:` вправе отстоять от прямой ВНУТРЕННЕГО ребра источника "
            "(общего у двух областей резки), чтобы её знак относительно этого ребра считался нулевым. Перекладина "
            "полосы вдоль сетки меша лежит на ребре источника точно по построению, а на карте конец перекладины "
            "отстоит от прямой на доли ячейки (замер `rounded_wall.001`: 0.011-0.29): резка отрезала от грани иглу "
            "между перекладиной и ребром — площадь нулевая по смыслу, положительная точно (52 на `rounded_wall.001`, "
            "12 на `sagging_wall`: длина в полосу, высота в микроны). Допуск стоит в ЗНАКЕ, вершина не двигается: "
            "факты (s, r) и классы рёбер (источник, фронт, стена) остаются точными. Знак у вершины один для обоих "
            "треугольников ребра, поэтому куски по-прежнему покрывают многоугольник точно, а доказательство «кусок в "
            "замкнутом треугольнике» читает тот же знак: вершина куска лежит в его треугольнике с точностью до "
            "допуска по нормали к ребру. Знаки вершин `src:` и `clip:`, а также неинтерьерных рёбер — точные."
        ),
        authority=(
            "materialize.clip_snap.NODE_EDGE_SNAP_CELLS; DECISIONS.md 2026-10-03 (SNAP-NOISE: привязка шума "
            "решётки перед резкой)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CLIP_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.ZERO_NODE_SIGN_AT_INTERIOR_SOURCE_EDGE_WITHIN_GAP
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(
            "MATERIALIZE_CLIP_NODE_SIGNS_ZEROED_BY_EDGE_GAP",
            "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_NANOMETRES",
            "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_MILLICELLS",
        ),
        declaration_sites=(
            "cftuv_envelope.materialize.clip_snap.NODE_EDGE_SNAP_CELLS",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_clip_snap.py"
            "::test_a_node_within_the_gap_of_an_interior_edge_makes_no_needle"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_clip_snap.py"
            "::test_a_node_beyond_the_gap_of_an_interior_edge_still_cuts_a_needle"
        ),
    ),
)


TOLERANCE_POLICY_REGISTRY_V1 = TolerancePolicyRegistryV1(
    registry_law=TOLERANCE_POLICY_REGISTRY_LAW_V1,
    policies=TOLERANCE_POLICIES_V1,
)


def tolerance_policy(policy_id: TolerancePolicyIdV1) -> TolerancePolicyV1:
    """Запись по имени. Отсутствие имени — ошибка, а не `None`."""

    for policy in TOLERANCE_POLICIES_V1:
        if policy.id is policy_id:
            return policy
    raise KeyError(policy_id)


def tolerance_policies_in_category(
    category: TolerancePolicyCategoryV1,
) -> tuple[TolerancePolicyV1, ...]:
    """Все записи категории в порядке реестра. Пустой кортеж — законный ответ."""

    return tuple(
        policy for policy in TOLERANCE_POLICIES_V1 if policy.category is category
    )


__all__ = (
    "TOLERANCE_POLICIES_V1",
    "TOLERANCE_POLICY_REGISTRY_LAW_V1",
    "TOLERANCE_POLICY_REGISTRY_SCHEMA_V1",
    "TOLERANCE_POLICY_REGISTRY_V1",
    "TolerancePolicyAllowedEffectV1",
    "TolerancePolicyAppliedStageV1",
    "TolerancePolicyCategoryV1",
    "TolerancePolicyCoordinateSpaceV1",
    "TolerancePolicyIdV1",
    "TolerancePolicyPipelineStageV1",
    "TolerancePolicyRegistryV1",
    "TolerancePolicyScalingLawV1",
    "TolerancePolicyUnitsV1",
    "TolerancePolicyV1",
    "tolerance_policies_in_category",
    "tolerance_policy",
)
