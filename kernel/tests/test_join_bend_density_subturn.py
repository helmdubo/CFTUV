"""JOIN-излом шире подшага плотности: домен строится на любом Fan Density, отказ называет причину.

Полевая беда (`wall_noise_top`, ребро нулевой длины 6-19 слито, четыре домена): при Fan Density 4
патчи 1 и 2 отказывали `COVERAGE_IS_NOT_EXACT: preparation:DOMAIN_GEOMETRY_REFUSED`, а при Fan
Density 2 строились. Причина — не веер: излом контура 35.96 градуса (в геометрии вычисления) решён
законом JOIN (`k = 0`, порог 45 градусов по сырому углу), а проверка опор Density A требовала от
этого излома подшаг `<= pi/q`, то есть 30 градусов при `q = 6`. Потолок плотности относится к шагам
веера; у JOIN шагов нет. Фикстура — выгрузка двух доменов меша владельца (`manifest.json`).

| что проверяется                                                                  | тест |
|----------------------------------------------------------------------------------|------|
| домен готовится на d4: излом JOIN шире 30 градусов, но уже 45                     | `test_the_exported_domain_prepares_at_density_4` |
| покрытие и материализация идут так же, как у кнопки                              | `test_the_exported_domain_materializes_like_the_button` |
| JOIN-углы одни и те же на всех плотностях (плотность закон угла не меняет)        | `test_the_join_corners_do_not_depend_on_the_density` |
| КРАСНЫЙ КОНТРОЛЬ: старый потолок `pi/q` у JOIN возвращает отказ, и он называет причину | `test_the_old_density_ceiling_refuses_and_the_refusal_names_its_reason` |
| отрицательный контроль: четверть оборота и направление поворота у JOIN по-прежнему проверяются | `test_the_join_bound_still_refuses_a_bend_beyond_a_quarter_turn` |
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec, CornerTreatmentV1, SelectionLaw
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import (
    MaterializationOutcome,
    admit_domain,
    materialization_request,
)
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope._corner_treatment import JOIN_EVALUATION_SUBTURN_Q
from cftuv_envelope.reference import adaptive_density_fan as density_fan
from cftuv_envelope.reference import angular
from cftuv_envelope.reference.direction_binding import (
    DirectionBindingCertificateUnproven,
    verify_huber_density_direction_bindings,
)
from cftuv_envelope.reference.planar_types import vector_scale
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "wall_noise_top_join_density4_v1"
PATCHES = ("wall_noise_top_patch1", "wall_noise_top_patch2")
UV = PolicyId("UV_DIRECT_STRIP_V1")
_DENSITY_VALUES = {
    1: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, kernel.ExactAngleSymbol.PI_OVER_3),
    2: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, kernel.ExactAngleSymbol.PI_OVER_4),
    3: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1, kernel.ExactAngleSymbol.PI_OVER_5),
    4: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1, kernel.ExactAngleSymbol.PI_OVER_6),
}


def _load(name: str, density: int = 4):
    folder = FIXTURE / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((folder / "decal_request.json").read_bytes())
    if density != 4:
        value, symbol = _DENSITY_VALUES[density]
        request = dataclasses.replace(
            request, max_subturn_value_id=value, max_subturn_exact_value=kernel.ExactAngleV1(symbol)
        )
    return snapshot, request


def _prepared(name: str, density: int = 4):
    snapshot, request = _load(name, density)
    prepared = prepare_conveyor(snapshot, request)
    return prepared, request


def _join_bends(prepared) -> dict[str, float]:
    """`{спека: изгиб в градусах}` каждого JOIN-угла, в геометрии вычисления (между входящей и исходящей опорой)."""

    context = prepared.context
    selections = {item.certificate_id: item for item in context.compilation.profile_selection_certificates}
    bends = {}
    for spec in context.compilation.envelope_specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        if selections[spec.selection_certificate_id].selection_law is not SelectionLaw.CORNER_JOIN_SOFT_BEND_V1:
            continue
        *_, ideal = angular._ideal_angular_support_data(context, spec)
        covectors = density_fan._covectors(context.metric, ideal)
        left, right = covectors[0], covectors[-1]
        dot = float(density_fan._dual_dot(context.metric, left, right))
        norms = float(density_fan._dual_dot(context.metric, left, left)) * float(
            density_fan._dual_dot(context.metric, right, right)
        )
        bends[spec.envelope_spec_id.value] = math.degrees(math.acos(dot / math.sqrt(norms)))
    return bends


@pytest.mark.parametrize("name", PATCHES)
def test_the_exported_domain_prepares_at_density_4(name):
    prepared, _request = _prepared(name)

    assert prepared.outcome.value == "EXACT", (prepared.outcome, prepared.detail)
    bends = _join_bends(prepared)
    assert bends, "the fixture must keep a JOIN corner"
    # Ради этого излома фикстура и снята: он шире потолка d4 (30 градусов) и уже порога JOIN (45).
    assert any(30.0 < value < 45.0 for value in bends.values()), bends
    assert dict(prepared.counters)["CONVEYOR_MITERED_CORNERS"] >= 1


@pytest.mark.parametrize("name", PATCHES)
def test_the_exported_domain_materializes_like_the_button(name):
    prepared, request = _prepared(name)
    coverage = conveyor_coverage(prepared, "0.25")
    assert coverage.outcome.value == "EXACT", coverage.detail

    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )

    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    assert result.batch is not None


@pytest.mark.parametrize("name", PATCHES)
def test_the_join_corners_do_not_depend_on_the_density(name):
    """Плотность — политика веера; закон угла JOIN её не читает, и те же углы остаются JOIN на d1..d4."""

    joined = {}
    for density in (1, 2, 3, 4):
        prepared, _request = _prepared(name, density)
        assert prepared.outcome.value == "EXACT", (density, prepared.outcome, prepared.detail)
        joined[density] = frozenset(
            item.corner_relation_id.value
            for item in prepared.compilation.corner_treatments
            if item.treatment is CornerTreatmentV1.JOIN_CONTINUATION
        )
    assert joined[1] and len(set(joined.values())) == 1, joined


@pytest.mark.parametrize("name", PATCHES)
def test_the_old_density_ceiling_refuses_and_the_refusal_names_its_reason(name, monkeypatch):
    """Красный контроль: потолок `pi/q` у JOIN (`q = 6`) возвращает полевой отказ, и отказ несёт причину.

    Без правки кода именно так и выглядит d4 на меше владельца; текст отказа обязан называть имя предиката
    (`BINDING_INSIDE_OWN_ORDINAL_WINDOW`), а не только «DOMAIN_GEOMETRY_REFUSED» на всё.
    """

    monkeypatch.setattr(angular, "JOIN_EVALUATION_SUBTURN_Q", 6)
    snapshot, request = _load(name)

    prepared = prepare_conveyor(snapshot, request)

    assert prepared.outcome.value == "DOMAIN_GEOMETRY_REFUSED"
    assert prepared.detail.startswith("REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE: ")
    assert "BINDING_INSIDE_OWN_ORDINAL_WINDOW" in prepared.detail
    admission = admit_domain(prepared, None, None)
    assert admission.outcome is MaterializationOutcome.COVERAGE_IS_NOT_EXACT
    assert admission.detail.startswith("preparation:DOMAIN_GEOMETRY_REFUSED: REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE")
    assert "BINDING_INSIDE_OWN_ORDINAL_WINDOW" in admission.detail


def test_the_join_bound_still_refuses_a_bend_beyond_a_quarter_turn():
    """Отрицательный контроль: JOIN не освобождён от проверки, он просто не читает потолок плотности.

    Настоящая пара опор проходит с границей JOIN (`q = 2`); та же пара с исходящей опорой, перевёрнутой
    (`180 - изгиб`: больше четверти оборота и против поворота владельца), отказывает тем же именем,
    каким отказывал полевой домен.
    """

    prepared, _request = _prepared("wall_noise_top_patch1")
    context = prepared.context
    selections = {item.certificate_id: item for item in context.compilation.profile_selection_certificates}
    checked = 0
    for spec in context.compilation.envelope_specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        if selections[spec.selection_certificate_id].selection_law is not SelectionLaw.CORNER_JOIN_SOFT_BEND_V1:
            continue
        _relation, sector, _anchor, _hidden, _ids, ideal = angular._ideal_angular_support_data(context, spec)
        orientation = sector.turn_orientation
        verify_huber_density_direction_bindings(context.metric, ideal, orientation, (), JOIN_EVALUATION_SUBTURN_Q)
        turned = (ideal[0], vector_scale(ideal[-1], -1))
        with pytest.raises(DirectionBindingCertificateUnproven, match="BINDING_INSIDE_OWN_ORDINAL_WINDOW"):
            verify_huber_density_direction_bindings(context.metric, turned, orientation, (), JOIN_EVALUATION_SUBTURN_Q)
        checked += 1
    assert checked >= 1
