"""STRETCH-BUDGET-POLICY: допуск растяжения развёртки — политика ЗАПРОСА, а не константа ядра.

Здесь проверяется: у `DecalRequestV1` есть поле `developable_stretch_budget` (точная дробь), его
значение по умолчанию — прежние 1/5, а на проводе оно опущено, пока равно умолчанию (прежние
запросы, их байты и хранимые фикстуры побитово те же; явно названное умолчание отвергается кодеком);
границы `(0, 1/2]` держит `validate_decal_request` именованным отказом `POLICY_MISMATCH`;
снапшот, чей сертификат записан под другим допуском, не компилируется с таким запросом
(`REFERENCE_INPUT_CONTRACT_INVALID`, имя поля названо); построитель и валидатор судят карту под
допуском запроса: `rounded_wall_noise_top`, патч 2, отказан при 20 % и принят при 35 % (ARAP).
"""

from __future__ import annotations

import dataclasses
import json
from fractions import Fraction
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope._stretch import stretch_violations
from cftuv_envelope.codec import ContractCodecError
from cftuv_envelope.contracts.metric import (
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET_V1,
    MAX_DEVELOPABLE_STRETCH_BUDGET,
    CurvatureLadderPolicyV1,
    DevelopableProposalSelectionLawV1 as Selection,
    ExactRationalV1,
    developable_stretch_budget_is_lawful,
)
from cftuv_envelope.contracts.request import DecalRequestV1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.schema import json_schema_for
from cftuv_envelope.validation_issues import ValidationCode
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
import developable_noise_fixtures as noise
from developable_factories import DOMAIN, PATCH, REVISION
from developable_route import build_metric, developable_domain

ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1
FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "building_patch10_density4_v1"
THIRTY_FIVE_PERCENT = Fraction(7, 20)


def _request(budget=None):
    base = kernel.DecalRequestCodecV1.loads((FIXTURE / "decal_request.json").read_bytes())
    if budget is None:
        return base
    return dataclasses.replace(base, developable_stretch_budget=ExactRationalV1(budget.numerator, budget.denominator))


def _budget_issues(request):
    return [
        item
        for item in kernel.validate_decal_request(request)
        if item.path == ("developable_stretch_budget",)
    ]


# --------------------------------------------------------------------------
# Поле запроса, умолчание и провод
# --------------------------------------------------------------------------


def test_the_kernel_default_is_one_fifth_and_a_request_without_the_field_carries_it():
    assert DEFAULT_DEVELOPABLE_STRETCH_BUDGET == Fraction(1, 5)
    assert DEFAULT_DEVELOPABLE_STRETCH_BUDGET_V1 == ExactRationalV1(1, 5)
    assert _request().developable_stretch_budget == ExactRationalV1(1, 5)
    assert MAX_DEVELOPABLE_STRETCH_BUDGET == Fraction(1, 2)


def test_a_stored_request_without_the_field_is_byte_for_byte_what_the_codec_writes_now():
    stored = (FIXTURE / "decal_request.json").read_bytes()
    request = kernel.DecalRequestCodecV1.loads(stored)
    assert kernel.DecalRequestCodecV1.dumps(request) == stored
    assert b"developable_stretch_budget" not in stored


def test_a_non_default_budget_is_written_on_the_wire_and_survives_the_round_trip():
    request = _request(THIRTY_FIVE_PERCENT)
    payload = kernel.DecalRequestCodecV1.dumps(request)
    assert json.loads(payload)["developable_stretch_budget"] == {
        "$type": "ExactRationalV1",
        "numerator": 7,
        "denominator": 20,
    }
    assert kernel.DecalRequestCodecV1.loads(payload) == request
    assert payload != (FIXTURE / "decal_request.json").read_bytes()


def test_the_default_spelled_out_on_the_wire_is_not_canonical_and_is_rejected():
    document = json.loads((FIXTURE / "decal_request.json").read_bytes())
    document["developable_stretch_budget"] = {"$type": "ExactRationalV1", "numerator": 1, "denominator": 5}
    with pytest.raises(ContractCodecError, match="equals its default"):
        kernel.DecalRequestCodecV1.loads(json.dumps(document))


def test_the_schema_lists_the_budget_as_an_optional_property():
    schema = json_schema_for(DecalRequestV1, "cftuv.envelope.decal_request.v1")
    assert "developable_stretch_budget" in schema["$defs"]["DecalRequestV1"]["properties"]
    assert "developable_stretch_budget" not in schema["$defs"]["DecalRequestV1"]["required"]
    assert "uv_policy_id" in schema["$defs"]["DecalRequestV1"]["required"]


# --------------------------------------------------------------------------
# Границы запроса: именованный отказ
# --------------------------------------------------------------------------


@pytest.mark.parametrize("budget", (Fraction(1, 100), Fraction(1, 50), Fraction(1, 5), THIRTY_FIVE_PERCENT, Fraction(1, 2)))
def test_a_lawful_budget_is_a_lawful_request(budget):
    assert developable_stretch_budget_is_lawful(budget)
    assert not _budget_issues(_request(budget))


@pytest.mark.parametrize("budget", (Fraction(0), Fraction(-1, 5), Fraction(51, 100), Fraction(3, 5), Fraction(1)))
def test_an_out_of_range_budget_is_a_named_request_refusal(budget):
    assert not developable_stretch_budget_is_lawful(budget)
    request = _request(budget)
    issues = _budget_issues(request)
    assert [item.code for item in issues] == [ValidationCode.POLICY_MISMATCH]
    assert "(0, 1/2]" in issues[0].message
    # Отказ доходит до входа компиляции: поле названо в сообщении.
    assert any(
        item.path == ("developable_stretch_budget",)
        for item in kernel.validate_snapshot_request_references(
            developable_domain(factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8")[0], request
        )
    )


# --------------------------------------------------------------------------
# Метрика снапшота обязана быть записана под допуском запроса
# --------------------------------------------------------------------------


def _bound_domain(budget):
    snapshot, request = developable_domain(
        factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8", developable_stretch_budget=budget
    )
    return snapshot, request


def test_a_snapshot_recorded_under_the_requests_budget_validates_with_it():
    snapshot, request = _bound_domain(THIRTY_FIVE_PERCENT)
    certificate = next(iter(snapshot.surface_metric_descriptors)).planarity_certificate
    assert certificate.stretch.stretch_budget == ExactRationalV1(7, 20)
    bound = dataclasses.replace(request, developable_stretch_budget=ExactRationalV1(7, 20))
    assert kernel.validate_snapshot_request_references(snapshot, bound) == ()
    # Снапшот без запроса судится под собственным записанным допуском (законным).
    assert kernel.validate_analysis_snapshot(snapshot) == ()


def test_a_snapshot_recorded_under_another_budget_than_the_requests_does_not_compile():
    snapshot, request = _bound_domain(THIRTY_FIVE_PERCENT)
    issues = kernel.validate_snapshot_request_references(snapshot, request)
    assert any("stretch_budget" in ".".join(item.path) and "request" in item.message for item in issues)
    result = kernel.compile_reference_envelopes(snapshot, request)
    assert result.compilation is None
    assert result.diagnostics[0].outcome.value == "REFERENCE_INPUT_CONTRACT_INVALID"
    assert "stretch_budget" in result.diagnostics[0].message


def test_the_default_snapshot_and_the_default_request_still_agree():
    snapshot, request = developable_domain(factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8")
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()
    assert kernel.compile_reference_envelopes(snapshot, request).compilation is not None


# --------------------------------------------------------------------------
# rounded_wall_noise_top, патч 2: отказан при 20 %, принят при 35 %
# --------------------------------------------------------------------------


def _rounded_wall_noise_top():
    return noise.noise_surface(noise.ROUNDED_WALL_NOISE_TOP_PATCH_2, patch=PATCH)


def test_the_owners_noisy_wall_is_refused_at_twenty_percent_and_accepted_at_thirty_five():
    parts = _rounded_wall_noise_top()
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_metric(parts, ladder=ON)
    assert failure.value.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "stretch_budget=2.000000e-01" in str(failure.value)

    record = build_metric(parts, ladder=ON, developable_stretch_budget=THIRTY_FIVE_PERCENT)
    certificate = record.metric.planarity_certificate
    assert certificate.stretch.stretch_budget == ExactRationalV1(7, 20)
    assert certificate.proposal_selection_law is Selection.ARAP_AFTER_HINGE_REFUSED_V1
    assert not stretch_violations(certificate.stretch)
    band = certificate.stretch.worst_band_squared_upper
    assert (band.numerator / band.denominator) ** 0.5 - 1.0 < 0.35
    vertices, faces, triangles = parts
    issues = validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
        developable_stretch_budget=THIRTY_FIVE_PERCENT,
    )
    assert issues == ()
    # Та же запись под допуском 20 % запроса не принимается.
    assert validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
        developable_stretch_budget=DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
    )
