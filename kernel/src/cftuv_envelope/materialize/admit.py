"""Допуск домена к материализации: именованные отказы ДО любой работы.

Материализатор не чинит и не угадывает вход. Он принимает домен, у которого

1. покрытие и подготовка доказаны (`EXACT`) — иначе `COVERAGE_IS_NOT_EXACT`;
2. запрос и покрытие принадлежат ЭТОЙ подготовке: ключ исполнения батча
   `(DecalRequestId, PatchDomainId)` берётся у подготовки, а переданный запрос
   обязан с ней совпасть везде, кроме законов выхода (`OUTPUT_POLICY_FIELDS`) —
   иначе `REQUEST_DOES_NOT_MATCH_PREPARATION`;
3. метрика — точный аффинный дескриптор плоскости (`RationalAffinePlanarMetricV2`
   с сертификатом точной плоскости или near-planar проекции). Кривой домен до
   сюда не доходит (его отвергает бюджет near-planar при подготовке), а вход с
   иным дескриптором — `DOMAIN_IS_NOT_PLANAR_ADMITTED`;
4. UV-закон запроса — из `SUPPORTED_UV_POLICIES`, иначе `UV_POLICY_UNSUPPORTED`.

NEAR_PLANAR допущен (решение 2026-10-01): меш строится на СЕРТИФИЦИРОВАННОЙ
плоскости, а не на исходных вершинах, и этот факт именован диагностикой
`NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE`; смещение над поверхностью — политика
хоста.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from enum import Enum

from ..contracts.metric import (
    ExactSourcePlaneCertificateV1,
    NearPlanarProjectionCertificateV1,
    RationalAffinePlanarMetricV2,
)
from ..ids import PolicyId
from .uv_law import SUPPORTED_UV_POLICIES


class MaterializationOutcome(str, Enum):
    """Чем кончилась материализация. Тихого пустого результата нет."""

    MATERIALIZED = "MATERIALIZED"
    COVERAGE_IS_NOT_EXACT = "COVERAGE_IS_NOT_EXACT"
    REQUEST_DOES_NOT_MATCH_PREPARATION = "REQUEST_DOES_NOT_MATCH_PREPARATION"
    DOMAIN_IS_NOT_PLANAR_ADMITTED = "DOMAIN_IS_NOT_PLANAR_ADMITTED"
    UV_POLICY_UNSUPPORTED = "UV_POLICY_UNSUPPORTED"
    STATION_CHAIN_UNNAMED = "STATION_CHAIN_UNNAMED"
    STATION_FRAME_IS_AMBIGUOUS = "STATION_FRAME_IS_AMBIGUOUS"
    TESSELLATION_DID_NOT_CLOSE = "TESSELLATION_DID_NOT_CLOSE"
    BATCH_DID_NOT_VALIDATE = "BATCH_DID_NOT_VALIDATE"
    EXACT_WORK_BUDGET_EXHAUSTED = "EXACT_WORK_BUDGET_EXHAUSTED"


class PlanarityKind(str, Enum):
    PLANAR_EXACT = "PLANAR_EXACT"
    NEAR_PLANAR = "NEAR_PLANAR"


@dataclass(frozen=True, slots=True)
class AdmissionV1:
    """Исход допуска: отказ с деталью либо вид плоскости."""

    outcome: MaterializationOutcome | None
    detail: str = ""
    planarity: PlanarityKind | None = None


#: Поля запроса, которые материализатор берёт у ПЕРЕДАННОГО запроса, а не у
#: подготовки: alpha (покрытие несёт собственную, и в батч она входит через
#: него), закон UV и политика материала (законы ВЫХОДА: ни план, ни
#: подготовка от них не зависят — ядро доказало это тем, что ключ кэша
#: подготовки хоста их не содержит). Всё остальное обязано совпасть с запросом,
#: по которому подготовка скомпилирована: ключ исполнения батча —
#: `(DecalRequestId, PatchDomainId)`, и чужой запрос дал бы батч с чужим ключом.
OUTPUT_POLICY_FIELDS = frozenset(
    {"requested_alpha", "uv_policy_id", "material_policy_id"}
)


def materialization_request(prepared, *, uv_policy_id):
    """Скомпилированный запрос подготовки, в котором заменён ТОЛЬКО закон UV.

    Закон UV — явный параметр материализации: личность запроса (`DecalRequestId`
    и всё, что влияет на план) остаётся ровно той, с которой подготовка
    скомпилирована, а применённый закон лежит в `contract_versions` батча.
    Запрос, собранный хостом заново и подставленный `dataclasses.replace`, давал
    бы то же при удаче и чужой ключ при расхождении — этот путь расхождению
    места не оставляет.
    """

    compiled = getattr(getattr(prepared, "compilation", None), "decal_request", None)
    if compiled is None:
        raise ValueError("the preparation carries no compiled DecalRequestV1")
    return replace(
        compiled, uv_policy_id=PolicyId(getattr(uv_policy_id, "value", uv_policy_id))
    )


def _shown(value) -> str:
    text = repr(value)
    return text if len(text) <= 90 else text[:87] + "..."


def request_mismatch(prepared, coverage, request) -> str:
    """Первое расхождение запроса с подготовкой и покрытия с подготовкой, либо `""`.

    Сверка по ПОЛЯМ, не по `==` целого запроса: `OUTPUT_POLICY_FIELDS`
    законно отличаются. Покрытие, посчитанное на другой подготовке (другой ключ
    плана), — то же нарушение ключа исполнения, только с другой стороны.
    """

    compilation = getattr(prepared, "compilation", None)
    compiled = getattr(compilation, "decal_request", None)
    if compiled is None:
        return "the preparation carries no compiled DecalRequestV1"
    for field in fields(compiled):
        if field.name in OUTPUT_POLICY_FIELDS:
            continue
        left = getattr(compiled, field.name)
        right = getattr(request, field.name, _MISSING)
        if left != right:
            return f"{field.name}: compiled {_shown(left)}, passed {_shown(right)}"
    key = compilation.plan_key
    if key.decal_request_id != compiled.decal_request_id:
        return (
            f"plan_key.decal_request_id: {_shown(key.decal_request_id)}, "
            f"compiled request {_shown(compiled.decal_request_id)}"
        )
    covered = getattr(getattr(coverage, "preparation", None), "compilation", None)
    if covered is not None and covered.plan_key != key:
        return (
            f"coverage plan_key {_shown(covered.plan_key)} "
            f"differs from the preparation's {_shown(key)}"
        )
    return ""


_MISSING = object()


def admit_domain(prepared, coverage, request) -> AdmissionV1:
    """Допуск по порядку: точность, ключ исполнения, плоскость, закон UV.

    Первая не прошедшая проверка называется; до любой работы материализатора.
    """

    if prepared.outcome.value != "EXACT":
        return AdmissionV1(
            MaterializationOutcome.COVERAGE_IS_NOT_EXACT,
            f"preparation:{prepared.outcome.value}",
        )
    if coverage.outcome.value != "EXACT":
        return AdmissionV1(
            MaterializationOutcome.COVERAGE_IS_NOT_EXACT,
            f"coverage:{coverage.outcome.value}",
        )
    mismatch = request_mismatch(prepared, coverage, request)
    if mismatch:
        return AdmissionV1(
            MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION, mismatch
        )
    context = getattr(prepared, "context", None)
    frame = getattr(context, "frame", None)
    if not isinstance(frame, RationalAffinePlanarMetricV2):
        return AdmissionV1(
            MaterializationOutcome.DOMAIN_IS_NOT_PLANAR_ADMITTED,
            type(frame).__name__,
        )
    certificate = frame.planarity_certificate
    if isinstance(certificate, ExactSourcePlaneCertificateV1):
        planarity = PlanarityKind.PLANAR_EXACT
    elif isinstance(certificate, NearPlanarProjectionCertificateV1):
        planarity = PlanarityKind.NEAR_PLANAR
    else:
        return AdmissionV1(
            MaterializationOutcome.DOMAIN_IS_NOT_PLANAR_ADMITTED,
            type(certificate).__name__,
        )
    if request.uv_policy_id not in SUPPORTED_UV_POLICIES:
        return AdmissionV1(
            MaterializationOutcome.UV_POLICY_UNSUPPORTED,
            str(request.uv_policy_id.value),
        )
    return AdmissionV1(None, "", planarity)
