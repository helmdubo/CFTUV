"""Host-owned selection of the exact angular fan policy."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal


# Измеренное значение alpha приёмки — ОДИН узел на оба инструмента.
# Прежде гейт нёс литерал `0.3` в двух местах, а свип не пиннил alpha вовсе и
# наследовал её из ползунка сцены. Владелец накрутил ползунок на 0.78, и
# 9-кнопочный свип упал `BUTTON_PARITY_MISMATCH`: `DecalRequestId` — контентный
# хэш запроса, alpha в него входит, поэтому идентичность прямой расписки и
# кнопки расходилась ПО ПОСТРОЕНИЮ при любом положении ползунка ≠ 0.3. Свип при
# этом нарушал собственное правило `REQUEST_POLICY_KNOBS_MEASURED_V1`: density
# он пиннит и перечитывает, alpha — нет. Одна константа делает расхождение
# двух инструментов непредставимым; наследование из сцены — измеримым.
MEASURED_REQUEST_ALPHA = 0.3


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


ENVELOPE_FAN_DENSITY_ITEMS = (
    ("0", "0", "Minimum angular fan segment density"),
    ("1", "1", "Default angular fan segment density"),
    ("2", "2", "Higher angular fan segment density"),
    ("3", "3", "High angular fan segment density"),
    ("4", "4", "Maximum angular fan segment density"),
)
DEFAULT_ENVELOPE_FAN_DENSITY = "1"
_DENSITY_IDENTIFIERS = frozenset(item[0] for item in ENVELOPE_FAN_DENSITY_ITEMS)


@dataclass(frozen=True, slots=True)
class EnvelopeAngularPolicyV1:
    """Канонические поля запроса, выбранные одной host-властью."""

    density: int | None
    selection_policy_id: object
    parameter_id: object
    value_id: object
    exact_value: object

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


def envelope_angular_policy(kernel, density) -> EnvelopeAngularPolicyV1:
    """Возвращает старый закон для None и Huber Density A для 0..4."""

    normalized = normalize_envelope_fan_density(density)
    if normalized is None:
        return EnvelopeAngularPolicyV1(
            None,
            kernel.AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1,
            kernel.MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,
            kernel.MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,
            kernel.ExactAngleV1(kernel.ExactAngleSymbol.PI_OVER_3),
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
    )


def envelope_request_policy_signature(request) -> tuple[str, ...]:
    """Канонический ключ только тех полей, от которых зависит fan preparation."""

    exact_value = request.max_subturn_exact_value
    return tuple(
        _enum_value(item)
        for item in (
            request.angular_profile_selection_policy_id,
            request.max_subturn_parameter_id,
            request.max_subturn_value_id,
            exact_value.symbol,
        )
    )


def envelope_decal_request_id_value(
    typed_value,
    source_revision_value: str,
    selected_chain_ids,
    explicit_value: str | None,
    policy: EnvelopeAngularPolicyV1,
) -> str:
    """Сохраняет V1-id для None и добавляет policy signature для Density."""

    base = explicit_value or typed_value(
        "decal-request",
        source_revision_value,
        tuple(sorted(item.value for item in selected_chain_ids)),
    )
    if policy.density is None:
        return base
    return typed_value("decal-request-density", base, policy.signature)


def build_envelope_request_contract(
    kernel,
    request_id,
    selected_use_ids,
    alpha_decimal,
    angular_policy: EnvelopeAngularPolicyV1,
):
    """Материализует один request из уже проверенных host-фактов."""

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
        uv_policy_id=kernel.PolicyId("ENVELOPE_DEBUG_NO_UV_V1"),
    )


__all__ = (
    "DEFAULT_ENVELOPE_FAN_DENSITY",
    "ENVELOPE_FAN_DENSITY_ITEMS",
    "MEASURED_REQUEST_ALPHA",
    "EnvelopeAngularPolicyV1",
    "build_envelope_request_contract",
    "envelope_angular_policy",
    "envelope_decal_request_id_value",
    "envelope_request_policy_signature",
    "normalize_envelope_fan_density",
    "request_alpha_decimal",
)
