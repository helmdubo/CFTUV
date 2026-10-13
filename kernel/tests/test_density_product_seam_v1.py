"""Product seam: одна batch-операция и явный вход замороженного oracle."""

from dataclasses import replace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import backend
from cftuv_envelope.reference import compile as compiler
from cftuv_envelope.reference.metric import _DensityExactMemo
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch_with_memo
from test_density_shell_events import compile_inputs


def test_default_whole_compile_calls_python_batch_once_and_shared_memo_charges_four_then_zero(monkeypatch):
    snapshot, request = compile_inputs()
    calls = []
    original = backend.density_compute
    def compute(batch, *, memo_snapshot):
        result = original(batch, memo_snapshot=memo_snapshot)
        calls.append((batch, memo_snapshot, result))
        return result
    monkeypatch.setattr(backend, "density_compute", compute)
    memo = _DensityExactMemo()
    cold = kernel.compile_reference_envelopes(snapshot, request, _density_exact_memo=memo)
    warm = kernel.compile_reference_envelopes(snapshot, request, _density_exact_memo=memo)
    assert cold.outcome.value == warm.outcome.value == "EXACT" and cold == warm
    assert len(calls) == 2 and calls[0][1].entries == () and len(calls[1][1].entries) == 2
    assert [sum(len(result.ordered_work_articles) for result in call[2].results) for call in calls] == [4, 0]
    assert backend.active_density_backend() is backend.KernelBackendV1.PYTHON
    old = kernel.compile_reference_envelopes(snapshot, request,
        _direction_binding_attachment=compiler._attach_direction_bindings_legacy_v1)
    assert old.outcome == cold.outcome and old.compilation.envelope_specs == cold.compilation.envelope_specs


def test_product_hook_observes_native_density_ledger_once(monkeypatch):
    from types import SimpleNamespace
    snapshot, request = compile_inputs()
    calls = []
    def compute(batch, memo_snapshot):
        calls.append((batch, memo_snapshot))
        return evaluate_raw_density_batch_with_memo(batch, memo_snapshot)
    native = SimpleNamespace(density_compute_with_memo=compute, NATIVE_REFUSALS=())
    monkeypatch.setattr(backend, "_native", lambda: (native, ""))
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        result = kernel.compile_reference_envelopes(snapshot, request)
    assert result.outcome.value == "EXACT" and len(calls) == 1
    assert ledger.record().density_native_calls == 1 and ledger.record().density_python_calls == 0
    assert ledger.record().native_calls == 0


def test_non_density_attachment_preserves_legacy_dispatch(monkeypatch):
    from cftuv_envelope.contracts.request import AngularProfileSelectionPolicyId
    snapshot, request = compile_inputs()
    original = kernel.compile_reference_envelopes(snapshot, request).compilation
    policy = next(value for value in AngularProfileSelectionPolicyId
                  if value is not AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1)
    changed = replace(original, decal_request=replace(original.decal_request, angular_profile_selection_policy_id=policy))
    calls, answer = [], object()
    def legacy(compilation, **kwargs):
        calls.append(compilation)
        return answer
    monkeypatch.setattr(compiler, "_attach_direction_bindings_legacy_v1", legacy)
    monkeypatch.setattr(backend, "density_compute", lambda *_args, **_kwargs: pytest.fail("non-Density reached batch"))
    assert compiler._attach_direction_bindings(changed) is answer and calls == [changed]


def test_explicit_oracle_callback_is_threaded_to_finalize_without_product_dispatch(monkeypatch):
    snapshot, request = compile_inputs()
    calls = []
    def oracle(compilation, **kwargs):
        calls.append(compilation)
        return compiler._attach_direction_bindings_legacy_v1(compilation, **kwargs)
    monkeypatch.setattr(backend, "density_compute", lambda *_args, **_kwargs: pytest.fail("explicit oracle reached batch"))
    result = kernel.compile_reference_envelopes(snapshot, request, _direction_binding_attachment=oracle)
    assert result.outcome.value == "EXACT" and len(calls) == 1


@pytest.mark.parametrize("density", range(5))
def test_real_high_angle_float_input_fullcompile_matches_current_oracle_q2_through_q6(density):
    from test_density_selection_policy import _narrow_high_angle_snapshot, _density_request, _DENSITY_VALUES
    from cftuv_envelope.reference.density_shell_events import DENSITY_CANON_EVENT_OUTCOMES
    snapshot, request = _narrow_high_angle_snapshot()
    value, symbol, _ = _DENSITY_VALUES[density]
    request = _density_request(request, value, symbol)
    old = kernel.compile_reference_envelopes(snapshot, request,
        _direction_binding_attachment=compiler._attach_direction_bindings_legacy_v1)
    current = kernel.compile_reference_envelopes(snapshot, request)
    assert old.outcome.value == current.outcome.value == "EXACT"
    def without_canons(result):
        def selected(diagnostics):
            return tuple(item for item in diagnostics if item.outcome not in DENSITY_CANON_EVENT_OUTCOMES)
        return replace(result, diagnostics=selected(result.diagnostics),
            compilation=replace(result.compilation, diagnostics=selected(result.compilation.diagnostics)))
    assert kernel.canonical_json_bytes(without_canons(current)) == kernel.canonical_json_bytes(without_canons(old))
