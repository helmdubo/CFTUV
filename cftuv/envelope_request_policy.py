"""Host-owned selection of the exact angular fan policy."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from fractions import Fraction


# Измеренное значение alpha приёмки — ОДИН узел на оба инструмента.
# Прежде гейт нёс литерал `0.3` в двух местах, а свип не пиннил alpha вовсе и
# наследовал её из ползунка сцены. Владелец накрутил ползунок на 0.78, и
# 9-кнопочный свип упал `BUTTON_PARITY_MISMATCH`: `DecalRequestId` — контентный
# хэш запроса, alpha в него входит, поэтому идентичность прямой расписки и
# кнопки расходилась ПО ПОСТРОЕНИЮ при любом положении ползунка ≠ измеренного.
# Свип при этом нарушал собственное правило `REQUEST_POLICY_KNOBS_MEASURED_V1`:
# density он пиннит и перечитывает, alpha — нет. Одна константа делает
# расхождение двух инструментов непредставимым; наследование из сцены —
# измеримым.
#
# 0.3 -> 0.25, решение принципала. Основания по весу:
# (1) 0.25 — СОБСТВЕННЫЙ дефолт ползунка (`envelope_debug_alpha`), поэтому
#     кнопка на нетронутой сцене несёт ровно измеренный узел без выставлений;
# (2) величина диадическая — точна в binary32, в binary64 и в
#     `Decimal(str(float(...)))` одновременно, поэтому круг «записал в ползунок
#     → прочитал» переживает ТОЖДЕСТВЕННО. 0.3 не переживала: `FloatProperty`
#     хранит binary32, и 0.3 возвращается как 0.30000001192092896, то есть
#     запрос кнопки нёс бы `Decimal('0.30000001192092896')` против
#     `Decimal('0.3')` у гейта — паритет был недостижим по построению никаким
#     пиннингом;
# (3) непрерывность с историческим 0.3 не стоит ничего: расписки ребейзятся с
#     каждой правкой сцены владельцем, а сравнение чисел разных alpha и так
#     запрещено по построению.
MEASURED_REQUEST_ALPHA = 0.25


def request_alpha_decimal(alpha) -> Decimal:
    """Та величина alpha, которую понесёт ЗАПРОС, а не та, что стоит в ползунке.

    Повторяет правило `build_envelope_decal_request`:
    `alpha_decimal = Decimal(str(float(alpha)))`. Живёт здесь, чтобы сверять
    ползунок можно было в ТОЙ ЖЕ величине, которая попадает в контентный хэш
    запроса. Сверять сырые float значило бы проверять не то, что расходится:
    в идентичность запроса входит именно эта Decimal.

    Оговорка, которую обязан знать вызывающий: `FloatProperty` Blender хранит
    binary32, и не всякое десятичное значение переживает круг «записал в
    ползунок → прочитал». Поэтому проверка round-trip у инструмента и нужна.
    """

    return Decimal(str(float(alpha)))


# UV-законы запроса. Реестр один: хост объявляет, какие идентификаторы вообще
# существуют, а материализатор ядра (`materialize.uv_law.SUPPORTED_UV_POLICIES`)
# решает, какие умеет. Расхождение имён ловит исполняемая проверка
# (`tests/test_envelope_request_policy.py`), а не договорённость.
#
# `ENVELOPE_DEBUG_NO_UV_V1` — закон отладочного пути: UV нет, меш не строится.
# Он остаётся ЗНАЧЕНИЕМ ПО УМОЛЧАНИЮ, поэтому запрос отладки (и его контентный
# хэш `DecalRequestId`) не сдвинулся ни на байт. `UV_DIRECT_STRIP_V1` — первый
# продуктовый закон: `(source_s, source_r) -> (u, v)` без атласа.
ENVELOPE_UV_POLICY_DEBUG_NO_UV = "ENVELOPE_DEBUG_NO_UV_V1"
ENVELOPE_UV_POLICY_DIRECT_STRIP = "UV_DIRECT_STRIP_V1"
ENVELOPE_UV_POLICIES = (
    ENVELOPE_UV_POLICY_DEBUG_NO_UV,
    ENVELOPE_UV_POLICY_DIRECT_STRIP,
)


ENVELOPE_FAN_DENSITY_ITEMS = (
    ("0", "0", "Minimum angular fan segment density"),
    ("1", "1", "Default angular fan segment density"),
    ("2", "2", "Higher angular fan segment density"),
    ("3", "3", "High angular fan segment density"),
    ("4", "4", "Maximum angular fan segment density"),
)
DEFAULT_ENVELOPE_FAN_DENSITY = "1"
_DENSITY_IDENTIFIERS = frozenset(item[0] for item in ENVELOPE_FAN_DENSITY_ITEMS)


# Допуск растяжения развёртки — политика запроса (`DecalRequestV1.developable_stretch_budget`), а на панели —
# целые проценты («Max stretch»): процент / 100 есть точная дробь, без двоичного `FloatProperty`. Умолчание панели
# (20 %) — решение владельца; совпадение с умолчанием ядра (1/5) сверяет исполняемая проверка
# (`tests/test_envelope_request_policy.py`), а не договорённость.
DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT = 20
ENVELOPE_MAX_STRETCH_PERCENT_RANGE = (1, 50)
DEFAULT_ENVELOPE_STRETCH_BUDGET = Fraction(DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT, 100)


def envelope_stretch_budget(percent) -> Fraction | None:
    """`None` (запрос несёт умолчание ядра) либо точная дробь `percent / 100`.

    Принимает только точный int в границах панели; бул и float — ошибка, а не округление.
    Значение, равное умолчанию, возвращается как `None`: запрос с умолчанием побитово прежний.
    """

    if percent is None:
        return None
    low, high = ENVELOPE_MAX_STRETCH_PERCENT_RANGE
    if type(percent) is not int or not low <= percent <= high:
        raise ValueError(f"Max stretch must be an exact int percent in {low}..{high}")
    budget = Fraction(percent, 100)
    return None if budget == DEFAULT_ENVELOPE_STRETCH_BUDGET else budget


def _budget_text(budget) -> str:
    return f"{budget.numerator}/{budget.denominator}"


@dataclass(frozen=True, slots=True)
class EnvelopeAngularPolicyV1:
    """Канонические поля запроса, выбранные одной host-властью.

    `developable_stretch_budget` — допуск растяжения развёртки запроса (`None`: умолчание ядра).
    Он едет в этой же записи, чтобы идентичность запроса и сам запрос получали его от одного владельца.
    """

    density: int | None
    selection_policy_id: object
    parameter_id: object
    value_id: object
    exact_value: object
    developable_stretch_budget: Fraction | None = None

    @property
    def signature(self) -> tuple[str, ...]:
        return tuple(
            _enum_value(item)
            for item in (
                self.selection_policy_id,
                self.parameter_id,
                self.value_id,
                self.exact_value.symbol,
            )
        )


def _enum_value(value) -> str:
    return str(value.value) if hasattr(value, "value") else str(value)


def normalize_envelope_fan_density(value) -> int | None:
    """Принимает только None, точный int 0..4 или канонический UI-id."""

    if value is None:
        return None
    if type(value) is int:
        if 0 <= value <= 4:
            return value
        raise ValueError("Fan Density integer must be in the closed range 0..4")
    if type(value) is str:
        if value in _DENSITY_IDENTIFIERS:
            return int(value)
        raise ValueError(
            "Fan Density string must be one canonical EnumProperty id 0..4"
        )
    raise TypeError("Fan Density must be None, exact int, or exact str")


def envelope_angular_policy(kernel, density, developable_stretch_budget=None) -> EnvelopeAngularPolicyV1:
    """Возвращает старый закон для None и Huber Density A для 0..4; допуск растяжения едет вместе."""

    normalized = normalize_envelope_fan_density(density)
    if developable_stretch_budget == DEFAULT_ENVELOPE_STRETCH_BUDGET:
        developable_stretch_budget = None
    if normalized is None:
        return EnvelopeAngularPolicyV1(
            None,
            kernel.AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1,
            kernel.MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,
            kernel.MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,
            kernel.ExactAngleV1(kernel.ExactAngleSymbol.PI_OVER_3),
            developable_stretch_budget,
        )
    value_contracts = (
        (
            kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
            kernel.ExactAngleSymbol.PI_OVER_2,
        ),
        (
            kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
            kernel.ExactAngleSymbol.PI_OVER_3,
        ),
        (
            kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
            kernel.ExactAngleSymbol.PI_OVER_4,
        ),
        (
            kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1,
            kernel.ExactAngleSymbol.PI_OVER_5,
        ),
        (
            kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1,
            kernel.ExactAngleSymbol.PI_OVER_6,
        ),
    )
    value_id, symbol = value_contracts[normalized]
    return EnvelopeAngularPolicyV1(
        normalized,
        kernel.AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1,
        kernel.MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
        value_id,
        kernel.ExactAngleV1(symbol),
        developable_stretch_budget,
    )


def envelope_request_policy_signature(request) -> tuple[str, ...]:
    """Канонический ключ только тех полей, от которых зависит подготовка: веер и допуск растяжения."""

    exact_value = request.max_subturn_exact_value
    return tuple(
        _enum_value(item)
        for item in (
            request.angular_profile_selection_policy_id,
            request.max_subturn_parameter_id,
            request.max_subturn_value_id,
            exact_value.symbol,
        )
    ) + (_budget_text(request.developable_stretch_budget),)


def envelope_decal_request_id_value(
    typed_value,
    source_revision_value: str,
    selected_chain_ids,
    explicit_value: str | None,
    policy: EnvelopeAngularPolicyV1,
) -> str:
    """Сохраняет V1-id для None и добавляет policy signature для Density и допуска растяжения."""

    base = explicit_value or typed_value(
        "decal-request",
        source_revision_value,
        tuple(sorted(item.value for item in selected_chain_ids)),
    )
    if policy.density is not None:
        base = typed_value("decal-request-density", base, policy.signature)
    if policy.developable_stretch_budget is None:
        return base
    return typed_value(
        "decal-request-stretch", base, _budget_text(policy.developable_stretch_budget)
    )


def build_envelope_request_contract(
    kernel,
    request_id,
    selected_use_ids,
    alpha_decimal,
    angular_policy: EnvelopeAngularPolicyV1,
    uv_policy_id: str = ENVELOPE_UV_POLICY_DEBUG_NO_UV,
):
    """Материализует один request из уже проверенных host-фактов.

    `uv_policy_id` — из `ENVELOPE_UV_POLICIES`; незнакомое имя — ошибка, а не
    молчаливая подмена закона.
    """

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")

    budget = angular_policy.developable_stretch_budget
    stretch = (
        {}
        if budget is None
        else {
            "developable_stretch_budget": kernel.ExactRationalV1(
                budget.numerator, budget.denominator
            )
        }
    )
    return kernel.DecalRequestV1(
        schema_version=kernel.DECAL_REQUEST_SCHEMA_V1,
        decal_request_id=request_id,
        selected_chain_use_ids=selected_use_ids,
        requested_alpha=kernel.LocalLengthV1(alpha_decimal),
        metric_space=kernel.MetricSpace.SOURCE_LOCAL_INTRINSIC,
        angular_profile_family_id=kernel.AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1,
        angular_profile_selection_policy_id=angular_policy.selection_policy_id,
        max_subturn_parameter_id=angular_policy.parameter_id,
        max_subturn_value_id=angular_policy.value_id,
        max_subturn_exact_value=angular_policy.exact_value,
        cap_policy_id=kernel.CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,
        boundary_policy_id=kernel.BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION,
        interaction_policy_id=kernel.InteractionPolicyId.INTRAPATCH_POLICY_B_V1,
        ownership_policy_id=kernel.OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1,
        material_policy_id=kernel.PolicyId("ENVELOPE_DEBUG_NO_MATERIAL_V1"),
        uv_policy_id=kernel.PolicyId(uv_policy_id),
        **stretch,
    )


__all__ = (
    "DEFAULT_ENVELOPE_FAN_DENSITY",
    "DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT",
    "DEFAULT_ENVELOPE_STRETCH_BUDGET",
    "ENVELOPE_FAN_DENSITY_ITEMS",
    "ENVELOPE_MAX_STRETCH_PERCENT_RANGE",
    "ENVELOPE_UV_POLICIES",
    "ENVELOPE_UV_POLICY_DEBUG_NO_UV",
    "ENVELOPE_UV_POLICY_DIRECT_STRIP",
    "MEASURED_REQUEST_ALPHA",
    "EnvelopeAngularPolicyV1",
    "build_envelope_request_contract",
    "envelope_angular_policy",
    "envelope_decal_request_id_value",
    "envelope_request_policy_signature",
    "envelope_stretch_budget",
    "normalize_envelope_fan_density",
    "request_alpha_decimal",
)
