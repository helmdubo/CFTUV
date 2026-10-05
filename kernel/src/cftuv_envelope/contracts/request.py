"""DecalRequestV1: request authority отделена от host snapshot."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import ChainUseId, DecalRequestId, PolicyId
from ..numeric import ExactAngleV1, MetricLengthV1, MetricSpace
from ..schema import wire_default_field
from .metric import (
    DEFAULT_CHART_REACH_CAP_V1,
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET_V1,
    DEFAULT_SILHOUETTE_UV_SLIDE_V1,
    ExactRationalV1,
)


DECAL_REQUEST_SCHEMA_V1 = "cftuv.envelope.decal_request.v1"


class AngularProfileFamilyId(str, Enum):
    LINEAR_REFLEX_EQUAL_V1 = "LINEAR_REFLEX_EQUAL_V1"


class AngularProfileSelectionPolicyId(str, Enum):
    MIN_K_FOR_MAX_SUBTURN_V1 = "MIN_K_FOR_MAX_SUBTURN_V1"
    HUBER_EMANATED_COUNT_DENSITY_A_V1 = "HUBER_EMANATED_COUNT_DENSITY_A_V1"


class MaxSubturnParameterId(str, Enum):
    LINEAR_REFLEX_MAX_SUBTURN_V1 = "LINEAR_REFLEX_MAX_SUBTURN_V1"
    LINEAR_REFLEX_DENSITY_A_V1 = "LINEAR_REFLEX_DENSITY_A_V1"


class MaxSubturnValueId(str, Enum):
    LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1 = (
        "LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1"
    )
    LINEAR_REFLEX_DENSITY_0_V1 = "LINEAR_REFLEX_DENSITY_0_V1"
    LINEAR_REFLEX_DENSITY_1_V1 = "LINEAR_REFLEX_DENSITY_1_V1"
    LINEAR_REFLEX_DENSITY_2_V1 = "LINEAR_REFLEX_DENSITY_2_V1"
    LINEAR_REFLEX_DENSITY_3_V1 = "LINEAR_REFLEX_DENSITY_3_V1"
    LINEAR_REFLEX_DENSITY_4_V1 = "LINEAR_REFLEX_DENSITY_4_V1"


class CapPolicyId(str, Enum):
    PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1 = "PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1"


class BoundaryPolicyId(str, Enum):
    BOUNDARY_LIMITED_PROPAGATION = "BOUNDARY_LIMITED_PROPAGATION"


class InteractionPolicyId(str, Enum):
    INTRAPATCH_POLICY_B_V1 = "INTRAPATCH_POLICY_B_V1"


class OwnershipPolicyId(str, Enum):
    TOTAL_DISJOINT_RESOLVED_COVERAGE_V1 = "TOTAL_DISJOINT_RESOLVED_COVERAGE_V1"


@dataclass(frozen=True, slots=True)
class DecalRequestV1:
    """Политика одного запроса. Ровно один владелец допуска растяжения развёртки.

    `developable_stretch_budget` — допуск растяжения развёртки (точная дробь из
    `(0, MAX_DEVELOPABLE_STRETCH_BUDGET]`; законность — в `validate_decal_request`). Запись без
    поля читается как `DEFAULT_DEVELOPABLE_STRETCH_BUDGET` (1/5), а на проводе значение по
    умолчанию опущено, поэтому прежние запросы, их хэши и фикстуры побитово те же. Допуск
    входит в идентичность исполнения, и метрика снапшота обязана нести сертификат,
    записанный ПОД ЭТИМ допуском (иначе запрос не компилируется, а отказ назван).

    `chart_reach_cap` — досягаемость полосовой карты, метры (точная дробь из `(0, MAX_CHART_REACH_CAP]`; законность — в
    `validate_decal_request`). Запись без поля читается как `DEFAULT_CHART_REACH_CAP` (полметра), на проводе значение по
    умолчанию опущено, поэтому прежние запросы, их хэши и фикстуры побитово те же. Это политика запроса, как и допуск
    растяжения: полосовая карта (`DevelopableBandChartCertificateV1`) записана под ЭТОЙ досягаемостью, и alpha запроса
    выше неё получает именованный отказ `REQUEST_ALPHA_EXCEEDS_CHART_REACH`. На домены, которые развёрнуты целиком,
    поле не влияет.

    `silhouette_uv_slide` — допуск UV закона топологии `SILHOUETTE_TOPOLOGY_V1` на растворение вершины и ребра (доля alpha, точная
    дробь из `[0, MAX_SILHOUETTE_UV_SLIDE]`, нуль — точная аффинность; законность — в `validate_decal_request`). Запись без
    поля читается как `DEFAULT_SILHOUETTE_UV_SLIDE` (1/256), на проводе значение по умолчанию опущено, поэтому прежние
    запросы, их хэши и фикстуры побитово те же. Поле читает только материализатор под этим законом; остальные законы топологии
    его не видят.
    """

    schema_version: str
    decal_request_id: DecalRequestId
    selected_chain_use_ids: frozenset[ChainUseId]
    requested_alpha: MetricLengthV1
    metric_space: MetricSpace
    angular_profile_family_id: AngularProfileFamilyId
    angular_profile_selection_policy_id: AngularProfileSelectionPolicyId
    max_subturn_parameter_id: MaxSubturnParameterId
    max_subturn_value_id: MaxSubturnValueId
    max_subturn_exact_value: ExactAngleV1
    cap_policy_id: CapPolicyId
    boundary_policy_id: BoundaryPolicyId
    interaction_policy_id: InteractionPolicyId
    ownership_policy_id: OwnershipPolicyId
    material_policy_id: PolicyId
    uv_policy_id: PolicyId
    developable_stretch_budget: ExactRationalV1 = wire_default_field(
        DEFAULT_DEVELOPABLE_STRETCH_BUDGET_V1
    )
    chart_reach_cap: ExactRationalV1 = wire_default_field(DEFAULT_CHART_REACH_CAP_V1)
    silhouette_uv_slide: ExactRationalV1 = wire_default_field(DEFAULT_SILHOUETTE_UV_SLIDE_V1)

