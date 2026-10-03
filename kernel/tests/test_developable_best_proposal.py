"""BEST-PROPOSAL: карта шарнира выше порога изометрии соревнуется с ARAP, победитель и числа записаны.

До закона шарнир, принятый в бюджете запроса, уходил в сертификат, даже если ARAP растянул бы
карту меньше (при бюджете 20 % шарнир в 15 % побеждал ARAP в 2 %). Теперь, когда сертифицированная
граница квадрата растяжения принятой карты шарнира выше `(1 + DEVELOPABLE_ISOMETRIC_ENOUGH)^2`, ARAP
тоже строит карту (тот же суд, тот же бюджет, та же решётка), и остаётся карта с меньшей границей;
равенство решает шарнир. Здесь проверяется: карта шарнира не выше порога побитово прежняя и ARAP не
пробуется; выше порога ARAP пробуется и побеждает, когда растянул меньше; равенство и проигрыш
оставляют шарнир; ARAP без положений и ARAP с отказанной картой оставляют карту шарнира с именем
причины; валидатор пересчитывает оба предложения и ловит подмену закона, числа и победителя; четыре
новых поля сертификата — единственные байты, которые изменились против прежних золотых дайджестов.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from fractions import Fraction
from types import SimpleNamespace

import pytest

import cftuv_envelope._developable as developable
from cftuv_envelope._arap import ArapProposalUnavailable
from cftuv_envelope._stretch import band_bounds
from cftuv_envelope.codec import canonical_json_bytes, to_canonical_data
from cftuv_envelope.contracts.metric import (
    DEVELOPABLE_ISOMETRIC_ENOUGH,
    CurvatureLadderPolicyV1,
    DevelopableProposalLawV1,
    DevelopableProposalSelectionLawV1 as Selection,
    ExactRationalV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
from developable_factories import DOMAIN, PATCH, REVISION, developable_chart
from developable_route import build_metric
from test_developable_arap import ARAP_FIXTURE_DROP, _perturbed_fold_grid

ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1
HINGE = DevelopableProposalLawV1.BINARY64_HINGE_V1
ARAP = DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1

#: Невязка складки сеткой, при которой шарнир в бюджете 20 % (5.84 %), но выше порога изометрии, а ARAP (1.87 %) лучше.
BETWEEN_THRESHOLD_AND_BUDGET = 0.05

#: Четыре поля сертификата, добавленные законом «лучшее предложение».
NEW_FIELDS = (
    "proposal_selection_law",
    "hinge_chart_worst_band_squared_upper",
    "arap_chart_worst_band_squared_upper",
    "arap_refusal",
)


def _value(rational: ExactRationalV1) -> Fraction:
    return Fraction(rational.numerator, rational.denominator)


def _threshold() -> Fraction:
    return band_bounds(DEVELOPABLE_ISOMETRIC_ENOUGH)[1]


def _contested():
    return _perturbed_fold_grid(BETWEEN_THRESHOLD_AND_BUDGET)


def _issues(record, parts, **options):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
        **options,
    )


def _tampered(record, **changes):
    certificate = replace(record.metric.planarity_certificate, **changes)
    return replace(record, metric=replace(record.metric, planarity_certificate=certificate))


def _patch_arap(monkeypatch, replacement):
    """Подмена второго предложения вместе со сбросом памяти построителя: память ключуется входами, а не кодом."""

    monkeypatch.setattr(developable, "arap_proposal", replacement)
    developable.clear_developable_chart_memory()


def _rival_positions(monkeypatch, transform):
    """Подставляет ARAP, чьи положения — преобразованные положения шарнира (иное — только они)."""

    def rival(topology, proposal, snapped):
        return SimpleNamespace(coordinates=transform(proposal.coordinates))

    _patch_arap(monkeypatch, rival)


def _scaled(factor: float):
    return lambda coordinates: {
        key: tuple(axis * factor for axis in point) for key, point in coordinates.items()
    }


# --------------------------------------------------------------------------
# Порог изометрии: ниже него ARAP не пробуется
# --------------------------------------------------------------------------


def test_a_hinge_chart_within_the_isometric_threshold_never_tries_arap(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("ARAP was tried for a hinge chart that is isometric enough")

    _patch_arap(monkeypatch, forbidden)
    for parts in (factories.fold_strip(), factories.bevel_strip(4), factories.quarter_cylinder(8)):
        certificate = developable_chart(parts).certificate
        assert certificate.proposal_law is HINGE
        assert certificate.proposal_selection_law is Selection.HINGE_ISOMETRIC_ENOUGH_V1
        assert _value(certificate.hinge_chart_worst_band_squared_upper) <= _threshold()
        assert certificate.hinge_chart_worst_band_squared_upper == certificate.stretch.worst_band_squared_upper
        assert certificate.arap_chart_worst_band_squared_upper is None
        assert certificate.arap_refusal == ""
        assert certificate.previous_refusals == ()


def test_a_hinge_chart_beyond_the_isometric_threshold_also_tries_arap(monkeypatch):
    tried = []
    original = developable.arap_proposal

    def spy(*arguments):
        tried.append(True)
        return original(*arguments)

    _patch_arap(monkeypatch, spy)
    certificate = developable_chart(_contested()).certificate
    assert tried == [True]
    hinge = _value(certificate.hinge_chart_worst_band_squared_upper)
    arap = _value(certificate.arap_chart_worst_band_squared_upper)
    assert hinge > _threshold()
    assert arap < hinge
    assert certificate.proposal_selection_law is Selection.BEST_ARAP_WON_V1
    assert certificate.proposal_law is ARAP
    assert certificate.stretch.worst_band_squared_upper == certificate.arap_chart_worst_band_squared_upper
    assert certificate.arap_refusal == ""
    # Шарнир не отказан, поэтому в следе лестницы записи об его отказе нет.
    assert certificate.previous_refusals == ()


def test_the_hinge_chart_is_accepted_in_the_budget_and_arap_still_beats_it():
    """Именно ради этого закон: шарнир 5.8 % в бюджете 20 %, ARAP 1.9 % — раньше побеждал шарнир."""

    certificate = developable_chart(_contested()).certificate
    percent = lambda rational: (float(_value(rational)) ** 0.5 - 1.0) * 100.0  # noqa: E731
    assert 2.0 < percent(certificate.hinge_chart_worst_band_squared_upper) < 20.0
    assert percent(certificate.arap_chart_worst_band_squared_upper) < 2.0
    assert certificate.stretch.triangles_outside_budget == 0


def test_a_tie_of_the_certified_numbers_goes_to_the_hinge(monkeypatch):
    _rival_positions(monkeypatch, lambda coordinates: coordinates)
    certificate = developable_chart(_contested()).certificate
    assert certificate.proposal_selection_law is Selection.BEST_HINGE_WON_V1
    assert certificate.proposal_law is HINGE
    assert certificate.hinge_chart_worst_band_squared_upper == certificate.arap_chart_worst_band_squared_upper
    assert certificate.stretch.worst_band_squared_upper == certificate.hinge_chart_worst_band_squared_upper


def test_a_worse_arap_chart_loses_and_both_numbers_are_recorded(monkeypatch):
    _rival_positions(monkeypatch, _scaled(1.1))
    certificate = developable_chart(_contested()).certificate
    assert certificate.proposal_selection_law is Selection.BEST_HINGE_WON_V1
    assert certificate.proposal_law is HINGE
    assert _value(certificate.hinge_chart_worst_band_squared_upper) < _value(
        certificate.arap_chart_worst_band_squared_upper
    )
    assert certificate.stretch.worst_band_squared_upper == certificate.hinge_chart_worst_band_squared_upper


def test_arap_without_positions_keeps_the_hinge_chart_by_name(monkeypatch):
    def unavailable(*_args, **_kwargs):
        raise ArapProposalUnavailable("the work cap")

    _patch_arap(monkeypatch, unavailable)
    chart = developable_chart(_contested())
    certificate = chart.certificate
    assert certificate.proposal_selection_law is Selection.HINGE_KEPT_ARAP_UNAVAILABLE_V1
    assert certificate.proposal_law is HINGE
    assert certificate.arap_chart_worst_band_squared_upper is None
    assert certificate.arap_refusal == ""
    assert _value(certificate.hinge_chart_worst_band_squared_upper) > _threshold()


def test_a_refused_arap_chart_keeps_the_hinge_chart_and_names_the_refusal(monkeypatch):
    _rival_positions(monkeypatch, _scaled(3.0))
    certificate = developable_chart(_contested()).certificate
    assert certificate.proposal_selection_law is Selection.HINGE_KEPT_ARAP_REFUSED_V1
    assert certificate.proposal_law is HINGE
    assert certificate.arap_chart_worst_band_squared_upper is None
    assert certificate.arap_refusal == NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value


def test_the_kept_hinge_chart_is_the_same_chart_whatever_the_second_proposal_did(monkeypatch):
    def unavailable(*_args, **_kwargs):
        raise ArapProposalUnavailable("the work cap")

    _patch_arap(monkeypatch, unavailable)
    kept = developable_chart(_contested())
    _rival_positions(monkeypatch, _scaled(3.0))
    refused = developable_chart(_contested())
    _rival_positions(monkeypatch, _scaled(1.1))
    lost = developable_chart(_contested())
    assert kept.nodes == refused.nodes == lost.nodes
    assert kept.chart_scale == refused.chart_scale == lost.chart_scale
    assert kept.certificate.stretch == refused.certificate.stretch == lost.certificate.stretch


def test_a_hinge_refused_before_the_snap_keeps_the_previous_arap_only_law():
    certificate = developable_chart(_perturbed_fold_grid(ARAP_FIXTURE_DROP)).certificate
    assert certificate.proposal_selection_law is Selection.ARAP_AFTER_HINGE_REFUSED_V1
    assert certificate.proposal_law is ARAP
    assert certificate.hinge_chart_worst_band_squared_upper is None
    assert certificate.arap_chart_worst_band_squared_upper == certificate.stretch.worst_band_squared_upper
    assert certificate.previous_refusals == (NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value,)


def test_the_competing_proposal_never_changes_the_hinge_unfolding():
    parts = _contested()
    first = developable_chart(parts)
    second = developable_chart(parts)
    assert first.nodes == second.nodes
    assert first.certificate == second.certificate


# --------------------------------------------------------------------------
# Побайтово: четыре новых поля — единственное, что изменилось
# --------------------------------------------------------------------------


def _without_new_fields(value):
    data = to_canonical_data(value)

    def strip(node):
        if isinstance(node, dict):
            if node.get("$type") == "DevelopableUnfoldCertificateV1":
                for name in NEW_FIELDS:
                    node.pop(name)
            for item in node.values():
                strip(item)
        elif isinstance(node, list):
            for item in node:
                strip(item)

    strip(data)
    return hashlib.sha256(
        json.dumps(data, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def test_the_new_certificate_fields_are_the_only_bytes_that_changed():
    """Вычеркнув четыре поля, получаем ровно прежние золотые дайджесты (складка 1/5, складка 1/50, запись ARAP)."""

    assert (
        _without_new_fields(developable_chart(factories.fold_strip()).certificate)
        == "f492bc8c1476dc46f20111c64bf9a8c8d9df20848d5f024c18fc69f97a059f78"
    )
    assert (
        _without_new_fields(developable_chart(factories.fold_strip(), budget=Fraction(1, 50)).certificate)
        == "ef2baa0b4b8960afc567974cb3d6cf2da03f3cfbbaf0affec574281e42c53670"
    )
    assert (
        _without_new_fields(build_metric(_perturbed_fold_grid(ARAP_FIXTURE_DROP), ladder=ON))
        == "0f5562b4075975f8025776f36ac4b52d6b3c85773f9c4ed06443f79ff807e6b2"
    )


def test_a_hinge_chart_within_the_threshold_has_the_same_nodes_with_and_without_the_law(monkeypatch):
    parts = factories.bevel_strip(8)
    with_law = developable_chart(parts)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("no second proposal below the threshold")

    _patch_arap(monkeypatch, forbidden)
    assert developable_chart(parts).nodes == with_law.nodes


# --------------------------------------------------------------------------
# Валидатор: пересчёт обоих предложений и красные контроли
# --------------------------------------------------------------------------


def test_the_validator_accepts_every_selection_law_the_builder_writes(monkeypatch):
    parts = _contested()
    record = build_metric(parts, ladder=ON)
    assert record.metric.planarity_certificate.proposal_selection_law is Selection.BEST_ARAP_WON_V1
    assert _issues(record, parts) == ()
    fold = factories.fold_strip()
    assert _issues(build_metric(fold, ladder=ON), fold) == ()
    arap_only = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    assert _issues(build_metric(arap_only, ladder=ON), arap_only) == ()

    _rival_positions(monkeypatch, lambda coordinates: coordinates)
    assert build_metric(parts, ladder=ON).metric.planarity_certificate.proposal_selection_law is (
        Selection.BEST_HINGE_WON_V1
    )
    assert _issues(build_metric(parts, ladder=ON), parts) == ()

    _rival_positions(monkeypatch, _scaled(3.0))
    refused = build_metric(parts, ladder=ON)
    assert refused.metric.planarity_certificate.proposal_selection_law is Selection.HINGE_KEPT_ARAP_REFUSED_V1
    assert _issues(refused, parts) == ()

    def unavailable(*_args, **_kwargs):
        raise ArapProposalUnavailable("the work cap")

    _patch_arap(monkeypatch, unavailable)
    kept = build_metric(parts, ladder=ON)
    assert kept.metric.planarity_certificate.proposal_selection_law is Selection.HINGE_KEPT_ARAP_UNAVAILABLE_V1
    assert _issues(kept, parts) == ()



def test_the_validator_recomputes_both_proposals_and_catches_a_forged_number():
    parts = _contested()
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    for forged in (
        _tampered(record, arap_chart_worst_band_squared_upper=certificate.hinge_chart_worst_band_squared_upper),
        _tampered(record, hinge_chart_worst_band_squared_upper=certificate.arap_chart_worst_band_squared_upper),
        _tampered(record, arap_chart_worst_band_squared_upper=None),
        _tampered(record, arap_refusal=NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP.value),
    ):
        assert _issues(forged, parts), forged.metric.planarity_certificate


def test_the_validator_catches_a_law_that_contradicts_its_own_numbers():
    parts = _contested()
    record = build_metric(parts, ladder=ON)
    messages = lambda forged: [item.message for item in _issues(forged, parts)]  # noqa: E731
    # Победитель ARAP по числам, закон называет шарнир.
    assert any(
        "selection law disagrees" in text
        for text in messages(_tampered(record, proposal_selection_law=Selection.BEST_HINGE_WON_V1))
    )
    # Шарнир выше порога изометрии не вправе называться «достаточно изометричным».
    assert any(
        "selection law disagrees" in text
        for text in messages(_tampered(record, proposal_selection_law=Selection.HINGE_ISOMETRIC_ENOUGH_V1))
    )
    # Закон «после отказа шарнира» требует в следе отказ шарнира и числа без карты шарнира.
    assert messages(_tampered(record, proposal_selection_law=Selection.ARAP_AFTER_HINGE_REFUSED_V1))


def test_the_validator_catches_a_winner_swapped_without_the_law(monkeypatch):
    parts = _contested()
    record = build_metric(parts, ladder=ON)
    assert _issues(_tampered(record, proposal_law=HINGE), parts)


def test_the_validator_catches_a_selection_record_on_a_chart_within_the_threshold():
    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    forged = _tampered(
        record,
        proposal_selection_law=Selection.BEST_HINGE_WON_V1,
        arap_chart_worst_band_squared_upper=certificate.hinge_chart_worst_band_squared_upper,
    )
    assert any("selection law disagrees" in item.message for item in _issues(forged, parts))


# --------------------------------------------------------------------------
# Допуск запроса: тот же закон при другом бюджете
# --------------------------------------------------------------------------


def test_the_best_proposal_law_runs_under_the_requests_budget_and_not_a_constant():
    """Шарнир 5.84 % при допуске 5 % отказан до привязки (ARAP единственный), при 20 % — соперник ARAP."""

    parts = _contested()
    tight = developable_chart(parts, budget=Fraction(1, 20)).certificate
    assert tight.proposal_selection_law is Selection.ARAP_AFTER_HINGE_REFUSED_V1
    assert tight.stretch.stretch_budget == ExactRationalV1(1, 20)
    wide = developable_chart(parts, budget=Fraction(1, 5)).certificate
    assert wide.proposal_selection_law is Selection.BEST_ARAP_WON_V1
    assert wide.stretch.stretch_budget == ExactRationalV1(1, 5)


def test_a_public_builder_refuses_a_budget_outside_the_lawful_range():
    parts = _contested()
    for budget in (Fraction(0), Fraction(-1, 5), Fraction(3, 5), Fraction(5, 2)):
        with pytest.raises(ValueError):
            build_metric(parts, ladder=ON, developable_stretch_budget=budget)
    record = build_metric(parts, ladder=ON, developable_stretch_budget=Fraction(1, 2))
    assert record.metric.planarity_certificate.stretch.stretch_budget == ExactRationalV1(1, 2)


def test_the_isometric_threshold_is_the_old_budget_and_is_named_in_the_registry():
    from cftuv_envelope.contracts.tolerance_policy import TolerancePolicyIdV1, tolerance_policy

    assert DEVELOPABLE_ISOMETRIC_ENOUGH == Fraction(1, 50)
    policy = tolerance_policy(TolerancePolicyIdV1.DEVELOPABLE_ISOMETRIC_ENOUGH_V1)
    assert _value(policy.value) == DEVELOPABLE_ISOMETRIC_ENOUGH


def test_a_refusal_of_the_hinge_chart_does_not_try_the_competition():
    """Купол вне бюджета обоих предложений: отказ прежний, имя то же (`DEVELOPABLE_STRETCH_BUDGET_EXCEEDED`)."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(factories.dome())
    assert failure.value.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED


# --------------------------------------------------------------------------
# Память построителя: карта, собранная пять раз, платит соперника ARAP один раз
# --------------------------------------------------------------------------


def test_a_second_identical_build_comes_from_the_memory_and_costs_no_second_arap(monkeypatch):
    calls = []
    original = developable.arap_proposal

    def spy(*arguments):
        calls.append(True)
        return original(*arguments)

    _patch_arap(monkeypatch, spy)
    parts = _contested()
    first = developable_chart(parts)
    second = developable_chart(parts)
    assert calls == [True]
    assert first.certificate == second.certificate and first.nodes == second.nodes
    assert first is not second and first.nodes is not second.nodes


def test_the_validator_recomputation_reuses_the_chart_the_builder_just_made(monkeypatch):
    calls = []
    original = developable.arap_proposal

    def spy(*arguments):
        calls.append(True)
        return original(*arguments)

    _patch_arap(monkeypatch, spy)
    parts = _contested()
    record = build_metric(parts, ladder=ON)
    assert _issues(record, parts) == ()
    assert _issues(record, parts) == ()
    assert calls == [True]


def test_every_input_of_the_builder_is_part_of_the_memory_key():
    parts = _contested()
    base = developable_chart(parts)
    other_budget = developable_chart(parts, budget=Fraction(1, 4))
    other_trace = developable_chart(parts, previous_refusals=("NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED",))
    assert other_budget.certificate.stretch.stretch_budget == ExactRationalV1(1, 4)
    assert other_budget.certificate != base.certificate
    assert other_trace.certificate.previous_refusals == ("NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED",)
    assert base.certificate.previous_refusals == ()


def test_a_caller_that_edits_the_returned_nodes_does_not_corrupt_the_memory():
    parts = _contested()
    first = developable_chart(parts)
    expected = dict(first.nodes)
    first.nodes.clear()
    assert developable_chart(parts).nodes == expected


def test_a_refusal_is_not_remembered_and_stays_a_refusal():
    for _ in range(2):
        with pytest.raises(PlanarMetricAdmissionError) as failure:
            developable_chart(factories.dome())
        assert failure.value.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
