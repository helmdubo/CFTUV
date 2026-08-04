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

    `AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1` намеренно совпадает буква в
    букву с членом `AngleTolerancePolicyIdV1`: докстринг того перечисления
    обещал, что имя станет ключом реестра, когда реестр появится. Обещание
    исполняется сверкой в тесте, а не совпадением по памяти.
    """

    PRODUCT_SKIRT_ABSOLUTE_V1 = "PRODUCT_SKIRT_ABSOLUTE_V1"
    NEAR_PLANAR_REPRESENTATION_NOISE_V1 = "NEAR_PLANAR_REPRESENTATION_NOISE_V1"
    AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1 = (
        "AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1"
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


class TolerancePolicyUnitsV1(str, Enum):
    """Единицы. `NONE` — у записей, чья политика есть закон, а не число."""

    METRES = "METRES"
    RADIANS = "RADIANS"
    RADIANS_PER_HALF_TURN = "RADIANS_PER_HALF_TURN"
    DIMENSIONLESS = "DIMENSIONLESS"
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
    NEAR_PLANAR_CERTIFICATE_RECOMPUTATION = (
        "NEAR_PLANAR_CERTIFICATE_RECOMPUTATION"
    )
    SOURCE_GRID_WINDOW = "SOURCE_GRID_WINDOW"
    SOURCE_GRID_SCALE_SELECTION = "SOURCE_GRID_SCALE_SELECTION"
    CANONICAL_ANGLE_RESTORATION = "CANONICAL_ANGLE_RESTORATION"
    CANONICAL_SUBTURN_FAN_CONSTRUCTION = "CANONICAL_SUBTURN_FAN_CONSTRUCTION"
    EXACT_SIGN_PREFILTER = "EXACT_SIGN_PREFILTER"
    DENSITY_FAN_PREPARATION = "DENSITY_FAN_PREPARATION"
    DENSITY_MINIMAL_HEIGHT_SEARCH = "DENSITY_MINIMAL_HEIGHT_SEARCH"
    EXACT_CANONICALIZATION_MEMORY = "EXACT_CANONICALIZATION_MEMORY"


class TolerancePolicyAllowedEffectV1(str, Enum):
    """ПРИНУДИТЕЛЬНОЕ поле: что именно допуску дозволено сделать с ответом.

    Читать так: всё, чего здесь не написано, допуску запрещено. Проверка —
    негативная фикстура записи: она обязана показывать ГРАНИЦУ дозволенного,
    а не просто ещё один зелёный путь.
    """

    ADMIT_OR_REJECT_PROJECTION = "ADMIT_OR_REJECT_PROJECTION"
    RECOMPUTE_DECLARED_CERTIFICATE_ONLY = "RECOMPUTE_DECLARED_CERTIFICATE_ONLY"
    ADMIT_OR_REJECT_GRID_SCALE = "ADMIT_OR_REJECT_GRID_SCALE"
    BOUND_GRID_WINDOW_OR_NAME_IT_CLOSED = "BOUND_GRID_WINDOW_OR_NAME_IT_CLOSED"
    REPLACE_MEASURE_WITH_CANONICAL_FACT = "REPLACE_MEASURE_WITH_CANONICAL_FACT"
    BUILD_FAN_ON_CANONICAL_SUPPORTS = "BUILD_FAN_ON_CANONICAL_SUPPORTS"
    NARROW_ADMITTED_SET_TOWARD_REFUSAL = "NARROW_ADMITTED_SET_TOWARD_REFUSAL"
    CHOOSE_FAST_OR_EXACT_PATH = "CHOOSE_FAST_OR_EXACT_PATH"
    NAMED_REFUSAL_ONLY = "NAMED_REFUSAL_ONLY"
    CHANGE_COST_NEVER_THE_ANSWER = "CHANGE_COST_NEVER_THE_ANSWER"


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
        id=TolerancePolicyIdV1.AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1,
        category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
        value=_rational(Fraction(7, 10**6)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "Восстановление задуманного ОТНОШЕНИЯ сторон до селектора "
            "плотности: угол, отклонившийся от прямого не более чем на "
            "объявленную авторскую ошибку, заменяется СИМВОЛИЧЕСКИМ "
            "каноническим фактом. Ниже по конвейеру решения идут на точной "
            "рациональной доле π, а не на интервале."
        ),
        authority=(
            "AngleTolerancePolicyIdV1.AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1; "
            "CanonicalAngleRestorationLawV1."
            "AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1; "
            "DECISIONS.md 2026-08-03 (продуктовый путь аудита: восстановление "
            "канонического авторского угла вместо сдвига границ ячеек)"
        ),
        applied_stage=TolerancePolicyAppliedStageV1.CANONICAL_ANGLE_RESTORATION,
        allowed_effect=(
            TolerancePolicyAllowedEffectV1.REPLACE_MEASURE_WITH_CANONICAL_FACT
        ),
        changes_topology=True,
        preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
        telemetry_counters=(),
        declaration_sites=(
            "cftuv_envelope._authoring_intent.AUTHOR_ANGULAR_ERROR",
        ),
        positive_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_wall_2_001_noise_is_inside_the_authoring_intent_tolerance"
        ),
        negative_fixture=(
            f"{_KERNEL_TESTS}/test_canonical_angle_restoration.py"
            "::test_building_003_drift_is_honestly_outside_the_tolerance"
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
        value=_rational(Fraction(7, 10**6)),
        bound_law=None,
        units=TolerancePolicyUnitsV1.RADIANS,
        coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_ANGLE_MEASURE,
        scaling_law=TolerancePolicyScalingLawV1.ABSOLUTE_INDEPENDENT_OF_EXTENT,
        scope=(
            "ТРЕТЬЯ дверь того же допуска: лучи веера ставятся точными "
            "поворотами на канонический подшаг, а невязка между канонической "
            "опорой и сырой ограничена той же авторской ошибкой "
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
            "cftuv_envelope._authoring_intent.AUTHOR_ANGULAR_ERROR",
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
            "::test_building_003_drift_is_honestly_outside_the_tolerance"
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
