"""Допуск домена к материализации: три именованных отказа ДО любой работы.

Материализатор не чинит и не угадывает вход. Он принимает домен, у которого

1. покрытие и подготовка доказаны (`EXACT`) — иначе `COVERAGE_IS_NOT_EXACT`;
2. метрика — точный аффинный дескриптор плоскости (`RationalAffinePlanarMetricV2`
   с сертификатом точной плоскости или near-planar проекции). Кривой домен до
   сюда не доходит (его отвергает бюджет near-planar при подготовке), а вход с
   иным дескриптором — `DOMAIN_IS_NOT_PLANAR_ADMITTED`;
3. UV-закон запроса — из `SUPPORTED_UV_POLICIES`, иначе `UV_POLICY_UNSUPPORTED`.

NEAR_PLANAR допущен (решение 2026-10-01): меш строится на СЕРТИФИЦИРОВАННОЙ
плоскости, а не на исходных вершинах, и этот факт именован диагностикой
`NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE`; смещение над поверхностью — политика
хоста.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..contracts.metric import (
    ExactSourcePlaneCertificateV1,
    NearPlanarProjectionCertificateV1,
    RationalAffinePlanarMetricV2,
)
from .uv_law import SUPPORTED_UV_POLICIES


class MaterializationOutcome(str, Enum):
    """Чем кончилась материализация. Тихого пустого результата нет."""

    MATERIALIZED = "MATERIALIZED"
    COVERAGE_IS_NOT_EXACT = "COVERAGE_IS_NOT_EXACT"
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


def admit_domain(prepared, coverage, request) -> AdmissionV1:
    """Три проверки по порядку; первая не прошедшая называется."""

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
