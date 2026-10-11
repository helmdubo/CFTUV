"""`compile_reference_envelopes` и `prepare_conveyor` принимают готовую проверку снапшота (WORKER_REUSES_ITS_SNAPSHOT_CHECK_V1).

Компиляция плана звала `validate_snapshot_request_references(snapshot, request)` без готовых замечаний, то есть проверяла снапшот ещё раз, хотя хост уже
проверил ЭТОТ снапшот под допуском ЭТОГО запроса при сборке запроса. Параметр `snapshot_issues` у обеих функций - то же, что третий аргумент
`validate_snapshot_request_references`: готовый кортеж `validate_analysis_snapshot(снапшот, developable_stretch_budget=<допуск запроса>)`.

1. Без параметра ничего не изменилось: умолчание `None`, снапшот проверяется внутри ровно один раз под допуском запроса.
2. С верным кортежем результат РАВЕН результату без него (принятый вход и отказанный), а снапшот заново не проверяется.
3. Параметр читается, а не игнорируется: переданное замечание попадает в отказ; кортеж чужого допуска даёт другой итог (допуск - за вызывающим).
4. `prepare_conveyor` ничего не добавляет и не теряет: подготовка с кортежем равна подготовке без него.

Кто имеет право отдать кортеж, решает хост (тот же объект, тот же допуск, то же исполнение): `tests/test_envelope_worker_reuses_snapshot_check.py`.
"""

from __future__ import annotations

import dataclasses
import inspect
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
import cftuv_envelope.validation as kernel_validation
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.reference.compile import compile_reference_envelopes
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.conveyor import prepare_conveyor as conveyor_prepare

import developable_factories as factories
from developable_route import developable_domain
from wavefront_cases import FIELD_FIXTURE


def _field_inputs():
    root = FIELD_FIXTURE.parent
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request.json").read_bytes())
    return snapshot, request


def _developable_inputs():
    return developable_domain(factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8")


INPUTS = pytest.mark.parametrize("inputs", [_field_inputs, _developable_inputs], ids=["field", "developable"])


def _budget(request) -> Fraction:
    asked = request.developable_stretch_budget
    return Fraction(asked.numerator, asked.denominator)


def _spy(monkeypatch):
    """`[допуск]` каждого вызова `validate_analysis_snapshot` изнутри `validate_snapshot_request_references`."""

    calls: list = []
    real = kernel_validation.validate_analysis_snapshot

    def spy(snapshot, **kwargs):
        calls.append(kwargs.get("developable_stretch_budget"))
        return real(snapshot, **kwargs)

    monkeypatch.setattr(kernel_validation, "validate_analysis_snapshot", spy)
    return calls


def _answer(prepared):
    return (prepared.outcome, prepared.detail, prepared.law_names, prepared.counters, prepared.lattice, prepared.regions)


def test_the_parameter_is_optional_and_none_keeps_the_old_behaviour():
    for function in (compile_reference_envelopes, prepare_conveyor, conveyor_prepare):
        assert inspect.signature(function).parameters["snapshot_issues"].default is None
    assert prepare_conveyor is conveyor_prepare


@INPUTS
def test_without_the_parameter_the_snapshot_is_validated_once_under_the_requests_budget(monkeypatch, inputs):
    snapshot, request = inputs()
    calls = _spy(monkeypatch)
    assert compile_reference_envelopes(snapshot, request).compilation is not None
    assert calls == [_budget(request)]


@INPUTS
def test_with_the_right_issues_the_result_equals_the_default_and_the_snapshot_is_not_validated_again(monkeypatch, inputs):
    snapshot, request = inputs()
    issues = tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=_budget(request)))
    assert issues == ()
    expected = compile_reference_envelopes(snapshot, request)
    calls = _spy(monkeypatch)

    result = compile_reference_envelopes(snapshot, request, snapshot_issues=issues)

    assert result == expected and result.compilation is not None
    assert calls == []


@INPUTS
def test_a_refused_input_is_refused_alike_with_the_precomputed_issues(inputs):
    """Снапшот с замечаниями: итог с готовым кортежем равен итогу без него (тот же исход, тот же текст)."""

    snapshot, request = inputs()
    capabilities = sorted(snapshot.analysis_capabilities, key=lambda item: item.value)
    spoiled = dataclasses.replace(snapshot, analysis_capabilities=frozenset(capabilities[1:]))
    issues = tuple(kernel.validate_analysis_snapshot(spoiled, developable_stretch_budget=_budget(request)))
    assert issues
    expected = compile_reference_envelopes(spoiled, request)
    assert expected.compilation is None
    assert expected.diagnostics[0].outcome.value == "REFERENCE_INPUT_CONTRACT_INVALID"

    assert compile_reference_envelopes(spoiled, request, snapshot_issues=issues) == expected


def test_the_parameter_is_read_and_the_issues_belong_to_one_budget():
    snapshot, request = _field_inputs()
    sentinel = kernel.ValidationIssue(kernel.ValidationCode.CAPABILITY, ("sentinel",), "precomputed")
    refused = compile_reference_envelopes(snapshot, request, snapshot_issues=(sentinel,))
    assert refused.compilation is None
    assert refused.diagnostics[0].outcome.value == "REFERENCE_INPUT_CONTRACT_INVALID"
    assert "sentinel: precomputed" in refused.diagnostics[0].message

    # Снапшот, записанный под допуском 7/20, с запросом на умолчание 1/5: расхождение допусков называет только проверка под допуском ЗАПРОСА.
    recorded, mismatching = developable_domain(
        factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8", developable_stretch_budget=Fraction(7, 20)
    )
    own = tuple(kernel.validate_analysis_snapshot(recorded, developable_stretch_budget=_budget(mismatching)))
    blind = tuple(kernel.validate_analysis_snapshot(recorded))
    default = compile_reference_envelopes(recorded, mismatching)
    assert default.compilation is None and "stretch_budget" in default.diagnostics[0].message
    assert own and blind == ()
    assert compile_reference_envelopes(recorded, mismatching, snapshot_issues=own) == default
    assert compile_reference_envelopes(recorded, mismatching, snapshot_issues=blind) != default


@INPUTS
def test_the_preparation_with_the_precomputed_issues_equals_the_preparation_without_them(monkeypatch, inputs):
    snapshot, request = inputs()
    plain = prepare_conveyor(snapshot, request)
    issues = tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=_budget(request)))
    calls = _spy(monkeypatch)

    prepared = prepare_conveyor(snapshot, request, snapshot_issues=issues)

    assert _answer(prepared) == _answer(plain)
    assert prepared.outcome == plain.outcome and prepared.regions
    assert calls == [], "the compilation inside the preparation does not validate the snapshot again"
    calls.clear()
    prepare_conveyor(snapshot, request)
    assert calls == [_budget(request)]
