"""Shell outcome transport: реальные fan memo, compile и materialization границы."""

from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from types import SimpleNamespace
import copy
import pickle

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.reference import adaptive_density_band as band, adaptive_density_fan as fan
from cftuv_envelope.reference import compile as compile_module, density_fan_binding as binding
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceCompileResultV1, ReferenceOutcome
from cftuv_envelope.reference.density_shell_events import (
    DensityShellEventCollector, SHELL_EVENT_OUTCOMES, capture_density_shell_candidates,
    density_shell_materialization_facts, record_density_shell_candidate,
    with_density_shell_diagnostics,
)
from cftuv_envelope.reference.density_shell_v2 import density_shell_v2
from cftuv_envelope.reference.density_vectors import _density_runtime_vector
from cftuv_envelope.reference.metric import ExactPlanarMetric, _DensityExactMemo


IDENTITY = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
CCW = kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
STRAW = sp.Rational(1657, 5120)


def metric():
    return ExactPlanarMetric(IDENTITY, IDENTITY, 1)


def bind(m, collector=None, spec="spec", law=band.WINDOW_LAW_VORONOI, refusals=None):
    ideal = tuple(_density_runtime_vector(*v) for v in ((1, 0), (1, 1), (0, 1)))
    return binding.bind_density_fan(
        m, ideal, CCW, 4, (None,), lifted=False, law=law, spec_id=spec,
        refusals=[] if refusals is None else refusals, shell_event_collector=collector,
    )


def recording_certifier(monkeypatch, values=(STRAW, sp.Rational(1, 2))):
    calls = []
    result = ((None,), "authority")
    def certify(m, *args, **kwargs):
        calls.append(args)
        for value in (*values, values[0]):
            record_density_shell_candidate(m, density_shell_v2(value))
        return result
    monkeypatch.setattr(binding, "certify_huber_density_bindings_with_adaptive_fallback", certify)
    return calls, result


def test_cold_and_memo_hits_replay_per_spec_value_and_keep_result_identity(monkeypatch):
    calls, expected = recording_certifier(monkeypatch)
    m, first = metric(), DensityShellEventCollector()
    assert bind(m, first) is expected
    assert bind(m, first) is expected
    assert len(calls) == 1
    assert len(first.diagnostics()) == 5
    second = DensityShellEventCollector()
    assert bind(m, second) is expected
    assert second.diagnostics() == first.diagnostics()
    assert bind(m, second, "another-spec") is expected
    assert len(second.diagnostics()) == 10
    assert {item.envelope_spec_id for item in second.diagnostics()} == {"spec", "another-spec"}
    memo_entry = next(iter(m._density_exact_memo.fan_bindings.values()))
    assert memo_entry.result is expected
    assert isinstance(memo_entry.shell_outcomes, tuple)
    with pytest.raises(FrozenInstanceError):
        memo_entry.result = ((), None)
    assert m._density_exact_memo.shell_event_capture is None


def test_validation_without_collector_has_no_frozen_uses_but_future_hit_replays(monkeypatch):
    calls, expected = recording_certifier(monkeypatch)
    m = metric()
    assert bind(m) is expected
    record_density_shell_candidate(m, object())  # нет active capture: candidate не читается
    record_density_shell_candidate(None, object())
    collector = DensityShellEventCollector()
    assert collector.diagnostics() == ()
    assert bind(m, collector) is expected
    assert len(calls) == 1 and len(collector.diagnostics()) == 5
    frozen = collector.diagnostics()
    bind(m)  # verifier reuse не меняет uses этого compiler collector
    assert collector.diagnostics() == frozen


def test_failure_attempts_and_band_fallback_preserve_both_bundles(monkeypatch):
    _, expected = recording_certifier(monkeypatch, values=(sp.Rational(1, 2),))
    attempts = []
    def refused(m, *args, **kwargs):
        attempts.append(kwargs["window_law"])
        record_density_shell_candidate(m, density_shell_v2(STRAW))
        raise fan.AdaptiveDensityFanInvalid("band refused")
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", refused)
    m, collector, refusals = metric(), DensityShellEventCollector(), []
    assert bind(m, collector, law=band.WINDOW_LAW_NARROW_BAND, refusals=refusals) is expected
    assert len(collector.diagnostics()) == 5
    assert refusals == [("spec", "AdaptiveDensityFanInvalid: band refused")]
    other = DensityShellEventCollector()
    assert bind(m, other, law=band.WINDOW_LAW_NARROW_BAND) is expected
    assert len(attempts) == 2 and other.diagnostics() == collector.diagnostics()
    assert len(m._density_exact_memo.fan_bindings) == 1
    assert m._density_exact_memo.shell_event_capture is None
    def failed(m, *args, **kwargs):
        record_density_shell_candidate(m, density_shell_v2(STRAW))
        raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, "exact refusal")
    monkeypatch.setattr(binding, "certify_huber_density_bindings_with_adaptive_fallback", failed)
    fresh, failed_events = metric(), DensityShellEventCollector()
    with pytest.raises(ReferenceGeometryError, match="exact refusal"):
        bind(fresh, failed_events)
    assert len(failed_events.diagnostics()) == 3
    assert not fresh._density_exact_memo.fan_bindings
    assert fresh._density_exact_memo.shell_event_capture is None


def test_nested_capture_and_pickle_keep_only_frozen_compilation_diagnostics(monkeypatch):
    _, expected = recording_certifier(monkeypatch)
    m, collector = metric(), DensityShellEventCollector()
    with capture_density_shell_candidates(m) as outer:
        assert bind(m, collector) is expected
        assert len(outer) == 2
    diagnostics = collector.diagnostics()
    for restored in (copy.deepcopy(m._density_exact_memo), pickle.loads(pickle.dumps(m._density_exact_memo))):
        assert restored.shell_event_capture is None and not restored.fan_bindings
    immutable = SimpleNamespace(diagnostics=diagnostics)
    assert pickle.loads(pickle.dumps(immutable)).diagnostics == diagnostics
    clean_metric = replace(m, _density_exact_memo=pickle.loads(pickle.dumps(m._density_exact_memo)))
    fresh = DensityShellEventCollector()
    assert bind(clean_metric, fresh) is expected
    assert fresh.diagnostics() == diagnostics


def test_value_wire_rendering_no_sympy_repr_and_three_filtered_counters(monkeypatch):
    m, collector = metric(), DensityShellEventCollector()
    values = [density_shell_v2(x) for x in (STRAW, sp.Rational(1, 2), sp.sqrt(2), sp.Rational(1, 3))]
    def forbidden(*args):
        raise AssertionError("event не читает SymPy repr")
    monkeypatch.setattr(sp, "srepr", forbidden)
    with capture_density_shell_candidates(m) as captured:
        for candidate in values:
            record_density_shell_candidate(m, candidate)
    collector.replay("spec", tuple(sorted(captured.items())))
    diagnostics = collector.diagnostics()
    counters, lines = density_shell_materialization_facts(diagnostics)
    assert dict(counters) == {SHELL_EVENT_OUTCOMES[0].value: 4, SHELL_EVENT_OUTCOMES[1].value: 2, SHELL_EVENT_OUTCOMES[2].value: 1}
    assert len(lines) == 7
    assert all("value_wire=" in item.message and "Radical" not in item.message for item in diagnostics)
    assert not any(name.startswith("EXACT_WORK") for name, _ in counters)
    unrelated = replace(diagnostics[0], outcome=ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED)
    assert density_shell_materialization_facts((*diagnostics, unrelated)) == (counters, lines)


def test_finalize_keeps_events_with_early_or_late_named_failure(monkeypatch):
    _, expected = recording_certifier(monkeypatch)
    m, collector = metric(), DensityShellEventCollector()
    sentinel = SimpleNamespace(diagnostics=())
    monkeypatch.setattr(compile_module, "_attach_evaluation_geometry", lambda value, **kwargs: value)
    def attach(value, *, shell_event_collector, **kwargs):
        assert bind(m, shell_event_collector) is expected
        return value, None
    refused = compile_module._failure(ReferenceOutcome.DENSITY_SEALED_FAN_INVALID, "named late failure")
    monkeypatch.setattr(compile_module, "_seal_evaluation_geometry_authority", lambda *args, **kwargs: refused)
    result = compile_module._finalize_compilation(sentinel, shell_event_collector=collector, direction_binding_attachment=attach)
    assert result.outcome is refused.outcome and result.compilation is None
    assert result.diagnostics[0] == refused.diagnostics[0]
    assert result.diagnostics[1:] == collector.diagnostics()
    assert with_density_shell_diagnostics(result, collector) == result
    def failed_attach(value, *, shell_event_collector, **kwargs):
        bind(m, shell_event_collector)
        return refused, None
    again = compile_module._finalize_compilation(sentinel, shell_event_collector=DensityShellEventCollector(), direction_binding_attachment=failed_attach)
    assert again == result


def compile_inputs():
    root = Path(__file__).parents[1] / "fixtures" / "building_patch10_density4_v1"
    decoded = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        (root / "analysis_snapshot.json").read_bytes())
    assert decoded.compatibility_receipt.positive_scaled_normal_compat_hits == 1
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request.json").read_bytes())
    return decoded.record, request


def test_real_compile_memo_hit_and_pickle_new_memo_replay_same_semantic_events(monkeypatch):
    snapshot, request = compile_inputs()
    baseline = compile_module.compile_reference_envelopes(snapshot, request,
        _direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    assert baseline.compilation is not None
    attempts = []
    for name in ("certify_huber_density_bindings_with_adaptive_fallback", "certify_adaptive_huber_density_direction_fan"):
        original = getattr(binding, name)
        def recorded(m, *args, _original=original, **kwargs):
            attempts.append(args)
            record_density_shell_candidate(m, density_shell_v2(STRAW))
            return _original(m, *args, **kwargs)
        monkeypatch.setattr(binding, name, recorded)
    memo, collector = _DensityExactMemo(), DensityShellEventCollector()
    def run(memo):
        return compile_module.compile_reference_envelopes(snapshot, request, _density_exact_memo=memo,
            _density_shell_event_collector=collector,
            _direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    first = run(memo)
    assert attempts and first.compilation is not None
    count = len(attempts)
    second = run(memo)
    assert len(attempts) == count and second == first
    restored = run(pickle.loads(pickle.dumps(memo)))
    assert len(attempts) > count and restored == first
    events = collector.diagnostics()
    assert events and first.diagnostics == events
    stripped = replace(first, diagnostics=(), compilation=replace(first.compilation, diagnostics=tuple(
        item for item in first.compilation.diagnostics if item.outcome not in SHELL_EVENT_OUTCOMES)))
    assert kernel.canonical_json_bytes(stripped) == kernel.canonical_json_bytes(baseline)
    assert pickle.loads(pickle.dumps(first)) == first
    # На новом compile scope даже ранний отказ не оставляет uses предыдущего запроса.
    invalid = replace(request, selected_chain_use_ids=frozenset())
    rejected = compile_module.compile_reference_envelopes(snapshot, invalid, _density_shell_event_collector=collector)
    assert rejected.compilation is None and collector.diagnostics() == ()


def test_materialization_success_emits_only_shell_events_without_changing_geometry_or_exact_work():
    import materialize_factories as factories
    from cftuv_envelope.ids import PolicyId
    from cftuv_envelope.materialize.domain import materialize_domain
    prepared, coverage, request = factories.two_edge_chain_domain()
    request = replace(request, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1"))
    baseline = materialize_domain(prepared, coverage, request=request)
    assert baseline.is_materialized
    collector = DensityShellEventCollector()
    candidate = density_shell_v2(STRAW)
    collector.replay("spec", ((candidate.value.wire(), candidate.outcomes),))
    result = ReferenceCompileResultV1(ReferenceOutcome.EXACT, prepared.compilation, ())
    enriched = with_density_shell_diagnostics(result, collector).compilation
    revised_prepared = replace(prepared, compilation=enriched)
    revised = materialize_domain(revised_prepared, coverage, request=request)
    assert revised.is_materialized
    assert kernel.canonical_json_bytes(revised.batch) == kernel.canonical_json_bytes(baseline.batch)
    assert revised.content_digest == baseline.content_digest
    assert tuple(item for item in revised.counters if item[0].startswith("EXACT_WORK")) == tuple(
        item for item in baseline.counters if item[0].startswith("EXACT_WORK"))
    new_counters, new_lines = density_shell_materialization_facts(enriched.diagnostics)
    assert revised.counters == baseline.counters + new_counters
    assert revised.diagnostics == baseline.diagnostics + new_lines
    assert dict(new_counters) == {outcome.value: 1 for outcome in SHELL_EVENT_OUTCOMES}


def test_actual_compile_refusal_keeps_band_attempt_and_fallback_events(monkeypatch):
    snapshot, request = compile_inputs()
    enabled, attempted = [], []
    def failed(value):
        def certify(m, *args, **kwargs):
            attempted.append(value)
            if enabled:
                record_density_shell_candidate(m, density_shell_v2(value))
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, "named compiler predicate refusal")
        return certify
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", failed(STRAW))
    monkeypatch.setattr(binding, "certify_huber_density_bindings_with_adaptive_fallback", failed(sp.Rational(1, 2)))
    baseline = compile_module.compile_reference_envelopes(snapshot, request,
        _direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    assert baseline.compilation is None
    assert attempted == [STRAW, sp.Rational(1, 2)]
    enabled.append(True)
    attempted.clear()
    memo, collector = _DensityExactMemo(), DensityShellEventCollector()
    revised = compile_module.compile_reference_envelopes(
        snapshot, request, _density_exact_memo=memo, _density_shell_event_collector=collector,
        _direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    assert attempted == [STRAW, sp.Rational(1, 2)]
    assert revised.compilation is None and revised.outcome is baseline.outcome
    assert revised.diagnostics == (*baseline.diagnostics, *collector.diagnostics())
    assert len(collector.diagnostics()) == 5 and not memo.fan_bindings
    assert pickle.loads(pickle.dumps(revised)) == revised


def _actual_compile_diff():
    import sys
    tools = str(Path(__file__).resolve().parents[2] / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import density_shell_compile_diff
    return density_shell_compile_diff


@pytest.mark.parametrize("folder,request_name,event_rows", (
    # Три прежних COMPILE_CASES: каноническая таблица обходится без bind/search.
    ("binding_noise_canonical_v1/building_patch114", "decal_request_density4.json", 0),
    ("density4_exact_limit_v1/mesh2_patch2", "decal_request_density4.json", 0),
    ("wall_noise_top_join_density4_v1/wall_noise_top_patch1", "decal_request.json", 0),
    ("binding_noise_canonical_v1/building004_patch0", "decal_request_density2.json", 2),
    ("binding_noise_canonical_v1/building_patch114", "decal_request_density2.json", 2),
    ("building_patch17_crowded_v1", "decal_request_density2.json", 8),
    ("building_patch17_crowded_v1", "decal_request_density4.json", 2),
    ("building_patch10_density4_v1", "decal_request.json", 4),
))
def test_actual_a1_adapter_compile_keeps_full_outputs_quantum_prices_and_replayed_events(folder, request_name, event_rows):
    report = _actual_compile_diff().check_case(folder, request_name,
        direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    assert all(report["stable"].values())
    assert all(phase["budgets_equal"] for phase in report["comparisons"])
    assert all(phase["canonical_equal"] and phase["raw_equal"] for phase in report["comparisons"]), report
    assert report["decimal_deltas"] == [] and report["full_output_deltas"] == []
    old, new = report["legacy"], report["candidate"]
    assert all(phase["events"] == [] for phase in old)
    assert len(new[0]["events"]) == event_rows
    assert new[0]["events"] == new[1]["events"] == new[2]["events"]
    if not event_rows:
        assert new[0]["budgets"] == []  # это table path, а не потерянный event bundle
        assert new[0]["event_classification"] == "NO_ADAPTER_CALL"
        assert new[0]["adapter_calls"]["calls"] == 0
    else:
        assert new[0]["adapter_calls"]["recorded_inside_capture"] > 0
        assert new[1]["event_classification"] == "MEMO_BUNDLE_REPLAY"
        assert new[1]["adapter_calls"]["recorded_inside_capture"] == 0
    assert all(row["outcome"] in {code.value for code in SHELL_EVENT_OUTCOMES} for row in new[0]["events"])


@pytest.mark.parametrize("cap_name,expected_outcome,event_rows", (
    ("search_cap", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", 2),
    ("preparation_cap", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", 0),
    ("box_cap", "DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED", 0),
))
def test_actual_a1_adapter_kernel_budget_refusal_replays_events_and_keeps_named_failure(cap_name, expected_outcome, event_rows):
    report = _actual_compile_diff().check_case("building_patch10_density4_v1", "decal_request.json", {cap_name: 0},
        direction_binding_attachment=compile_module._attach_direction_bindings_legacy_v1)
    assert all(report["stable"].values())
    assert all(p["budgets_equal"] and p["canonical_equal"] and p["raw_equal"] for p in report["comparisons"]), report
    assert all(p["outcome"] == expected_outcome and not p["compilation_present"] for p in report["candidate"])
    assert len(report["candidate"][0]["events"]) == event_rows
    assert report["candidate"][0]["events"] == report["candidate"][1]["events"] == report["candidate"][2]["events"]
    if not event_rows:
        phase = report["candidate"][0]
        assert phase["event_classification"] == "ONLY_NOT_APPLICABLE_VALUES"
        assert phase["adapter_calls"]["applied"] == phase["adapter_calls"]["recorded_inside_capture"] == 0
        assert phase["adapter_calls"]["calls"] == sum(phase["adapter_calls"]["not_applicable"].values()) > 0
    if cap_name != "box_cap":
        assert any(budget["shell_probes"] + budget["order_steps"] > budget["cap"]
                   for budget in report["candidate"][0]["budgets"])


def test_actual_a1_adapter_context_restores_aliases_caps_and_preserves_legacy_symbol():
    diff = _actual_compile_diff()
    before = (diff.binding._density_decimal_envelope, diff.values._density_decimal_envelope,
              diff.adapter._legacy_density_decimal_envelope, fan._DENSITY_EXACT_WORK_CAP)
    with pytest.raises(RuntimeError, match="scope ends"):
        with diff.adapter_policy(True, {"search_cap": 0}):
            assert diff.binding._density_decimal_envelope is diff.values._density_decimal_envelope is diff.adapter._density_decimal_envelope_v2
            assert diff.adapter._legacy_density_decimal_envelope is before[2]
            raise RuntimeError("scope ends")
    assert before == (diff.binding._density_decimal_envelope, diff.values._density_decimal_envelope,
                      diff.adapter._legacy_density_decimal_envelope, fan._DENSITY_EXACT_WORK_CAP)
