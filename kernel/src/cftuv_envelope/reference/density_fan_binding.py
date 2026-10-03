"""Привязка лучей плотностного веера продуктовым путём: закон окна и названный отказ полосы.

Закон окна луча (`adaptive_density_band.py`): основной — узкая полоса поворота
вокруг равноугольного идеала, запасной — окно Вороного. Полоса строит ТОЛЬКО
адаптивную власть: построчные сертификаты B(w) записывают лишь окно Вороного (у них
нет поля закона окна), и на полосе они не строятся. Если полоса не построила власть
(исчерпание точной работы, неустановимый чарт, пустое окно, неразрешённый предикат),
отказ ЗАПИСЫВАЕТСЯ, и веер ищет прежнее окно Вороного прежним путём: молча окно не
меняется (п. 4 `AGENTS.md`). Отказ прежнего окна идёт наверх как раньше.
"""

from __future__ import annotations

from .adaptive_density_band import WINDOW_LAW_VORONOI
from .adaptive_density_fan import (
    AdaptiveDensityFanInvalid,
    DensityRationalAuthorityExhausted,
    DensityTerminationBoxesExhausted,
    DensityWindowChartUnrepresentable,
)
from .common import ReferenceGeometryError
from .contracts import (
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .direction_binding import (
    DirectionBindingCertificateUnproven,
    certify_adaptive_huber_density_direction_fan,
    certify_huber_density_bindings_with_adaptive_fallback,
)

_BAND_REFUSALS = (
    AdaptiveDensityFanInvalid,
    DensityRationalAuthorityExhausted,
    DensityTerminationBoxesExhausted,
    DensityWindowChartUnrepresentable,
    DirectionBindingCertificateUnproven,
    ReferenceGeometryError,
)


def bind_density_fan(
    metric,
    ideal,
    orientation,
    q: int,
    binding_reasons,
    *,
    lifted: bool,
    law: str,
    spec_id: str,
    refusals: list,
):
    """`(построчные сертификаты, власть)` под законом окна `law`, отказ полосы — в `refusals`."""

    def build(window_law: str):
        if lifted or window_law != WINDOW_LAW_VORONOI:
            return (None,) * (len(ideal) - 2), certify_adaptive_huber_density_direction_fan(
                metric, ideal, orientation, q, binding_reasons, window_law=window_law
            )
        return certify_huber_density_bindings_with_adaptive_fallback(
            metric, ideal, orientation, q, binding_reasons
        )

    if law == WINDOW_LAW_VORONOI:
        return build(law)
    try:
        return build(law)
    except _BAND_REFUSALS as exc:
        refusals.append((spec_id, f"{type(exc).__name__}: {exc}"))
    return build(WINDOW_LAW_VORONOI)


def band_refusal_diagnostics(refusals) -> tuple:
    """Именованный отказ полосы на каждый веер, ушедший в окно Вороного."""

    return tuple(
        ReferenceEvaluationDiagnosticV1(
            outcome=ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED,
            severity=ReferenceDiagnosticSeverity.INFO,
            message=(
                f"{reason}: the rays are bound in the Voronoi window of the "
                "neighbouring ideal rays, not in the narrow rotation band of the "
                "equal-step ideal"
            ),
            envelope_spec_id=spec_id,
        )
        for spec_id, reason in refusals
    )
