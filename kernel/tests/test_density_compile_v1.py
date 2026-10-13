"""Candidate wrapper проверяет carrier/lifecycle, реальная математика — offline A/B."""

from dataclasses import replace

import pytest

from test_density_attach_v1 import FixedFrontend, _forbidden, _result
from test_density_raw_v1 import CASES, _case_inputs, _selections
from cftuv_envelope.reference import compile as compiler, density_compile_v1 as candidate
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.contracts import (
    ReferenceCompileResultV1, ReferenceDiagnosticSeverity, ReferenceEvaluationDiagnosticV1, ReferenceOutcome,
)
from cftuv_envelope.reference.density_oracle_v1 import DensityFailureFactsV1
from cftuv_envelope.reference.density_raw_v1 import RawDensityExtractionErrorV1, extract_raw_density_spec_input
from cftuv_envelope.reference.metric import _DensityExactMemo


@pytest.fixture
def inputs(monkeypatch):
    source, context = _case_inputs(monkeypatch, *CASES[0])
    raw = tuple(extract_raw_density_spec_input(source, context, spec, selection)
                for spec, selection in _selections(context))
    return context.compilation, raw


def test_same_context_flags_sorted_shared_frontend_and_ordered_work(inputs, monkeypatch):
    compilation, raw = inputs
    frontend = FixedFrontend({item.spec_id: _result(item) for item in raw})
    descriptor, flags = GeometryContext.__dict__["build"], []
    def build(cls, compilation, frame, **kwargs):
        flags.append(kwargs)
        return descriptor.__func__(cls, compilation, frame, **kwargs)
    monkeypatch.setattr(GeometryContext, "build", classmethod(build))
    for name in ("_evaluation_density_spec", "bind_density_fan", "canonical_fan_rays_decision",
                 "has_rational_density_support_direction"):
        monkeypatch.setattr(compiler, name, _forbidden)
    memo, work = _DensityExactMemo(), []
    updated, context = candidate.attach_density_compilation_v1(compilation, frontend,
                                                              density_exact_memo=memo, ordered_work=work)
    assert frontend.calls == sorted(item.spec_id for item in raw)
    assert updated.envelope_specs == compilation.envelope_specs and len(work) == len(raw)
    assert context.compilation is compilation
    assert flags == [dict(require_certified_bound_supports=False, require_canonical_fan_authority=False,
                          density_exact_memo=memo), dict(require_evaluation_binding=False,
                          require_certified_bound_supports=False, require_canonical_fan_authority=False,
                          density_exact_memo=memo)]


def test_frame_refusal_stops_before_raw_and_frontend(inputs, monkeypatch):
    compilation, _ = inputs
    diagnostic = ReferenceEvaluationDiagnosticV1(ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
        ReferenceDiagnosticSeverity.UNSUPPORTED, "frame guard")
    monkeypatch.setattr(compiler, "validate_reference_geometry_payload", lambda *args, **kwargs: (None, (diagnostic,)))
    monkeypatch.setattr(candidate, "_raw_inputs", _forbidden)
    frontend = FixedFrontend({})
    result, context = candidate.attach_density_compilation_v1(compilation, frontend)
    assert result == ReferenceCompileResultV1(diagnostic.outcome, None, (diagnostic,))
    assert context is None and frontend.calls == []


def test_failure_keeps_prior_work_atomic_and_extraction_error_is_internal(inputs, monkeypatch):
    compilation, raw = inputs
    first = min(raw, key=lambda item: item.spec_id)
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    frontend = FixedFrontend({first.spec_id: _result(first, outcome=failure.outcome, failure=failure)})
    work = []
    result, context = candidate.attach_density_compilation_v1(compilation, frontend, ordered_work=work)
    assert result.compilation is None and context is None and len(work) == 1
    assert frontend.calls == [first.spec_id]
    def malformed(*args):
        raise RawDensityExtractionErrorV1("anchor segment is not unique")
    monkeypatch.setattr(candidate, "extract_raw_density_spec_input", malformed)
    frontend = FixedFrontend({})
    with pytest.raises(RawDensityExtractionErrorV1):
        candidate.attach_density_compilation_v1(compilation, frontend)
    assert frontend.calls == []


def test_non_density_request_is_explicit_contract_gap(inputs):
    compilation, _ = inputs
    from cftuv_envelope.contracts.request import AngularProfileSelectionPolicyId
    policy = next(value for value in AngularProfileSelectionPolicyId
                  if value is not AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1)
    changed = replace(compilation, decal_request=replace(compilation.decal_request, angular_profile_selection_policy_id=policy))
    with pytest.raises(ValueError, match="explicit Density"):
        candidate.attach_density_compilation_v1(changed, FixedFrontend({}))


def _memo_kwargs():
    from test_density_raw_frontend_v1 import _frontend
    frontend = _frontend()
    return dict(canon_ids=frontend.canon_ids, limits=frontend.limits, window_law="NARROW_ROTATION_BAND")


def test_batch_wrapper_is_one_call_and_does_not_replay_budget_articles_twice(inputs):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityMemoBatchResultV1, DensityAttemptMemoSnapshotV1
    compilation, raw = inputs
    calls, work = [], []
    def compute(batch, *, memo_snapshot):
        calls.append((batch, memo_snapshot))
        return DensityMemoBatchResultV1(tuple(_result(item) for item in batch.specs), DensityAttemptMemoSnapshotV1())
    updated, _ = candidate.attach_density_compilation_batch_v1(compilation, compute, ordered_work=work, **_memo_kwargs())
    assert updated.envelope_specs == compilation.envelope_specs and len(calls) == 1
    assert len(work) == len(raw) and calls[0][1].entries == ()


def test_empty_batch_still_calls_once_and_returns_empty_delta(inputs):
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityMemoBatchResultV1, DensityAttemptMemoSnapshotV1
    compilation, _ = inputs
    empty = replace(compilation, envelope_specs=frozenset(spec for spec in compilation.envelope_specs
                                                         if not isinstance(spec, AngularEnvelopeSpec)))
    calls = []
    def compute(batch, *, memo_snapshot):
        calls.append(batch)
        return DensityMemoBatchResultV1((), DensityAttemptMemoSnapshotV1())
    updated, _ = candidate.attach_density_compilation_batch_v1(empty, compute, **_memo_kwargs())
    assert len(calls) == 1 and calls[0].specs == () and updated.envelope_specs == empty.envelope_specs


def test_named_failure_prefix_keeps_work_once_and_stops_attachment(inputs):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityMemoBatchResultV1, DensityAttemptMemoSnapshotV1
    compilation, _ = inputs
    calls, work = [], []
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    def compute(batch, *, memo_snapshot):
        calls.append(batch)
        results = (_result(batch.specs[0]), _result(batch.specs[1], outcome=failure.outcome, failure=failure))
        return DensityMemoBatchResultV1(results, DensityAttemptMemoSnapshotV1())
    result, context = candidate.attach_density_compilation_batch_v1(compilation, compute, ordered_work=work, **_memo_kwargs())
    assert len(calls) == 1 and result.compilation is None and context is None and len(work) == 2


@pytest.mark.parametrize("shape", ("oldtuple", "list", "incomplete", "untyped", "overlong", "afterfailure"))
def test_malformed_memo_results_do_not_pollute_host_memo(inputs, shape):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityMemoBatchResultV1, DensityAttemptMemoSnapshotV1
    from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch_with_memo
    from test_density_attempt_memo_v1 import _batch
    compilation, raw = inputs
    delta = evaluate_raw_density_batch_with_memo(_batch(), DensityAttemptMemoSnapshotV1()).delta
    memo = _DensityExactMemo()
    def compute(batch, *, memo_snapshot):
        values = tuple(_result(item) for item in batch.specs)
        if shape == "oldtuple":
            return values
        if shape == "list":
            values = list(values)
        elif shape == "incomplete":
            values = values[:-1]
        elif shape == "untyped":
            values = (object(),) * len(values)
        elif shape == "overlong":
            values += values[:1]
        elif shape == "afterfailure":
            failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
            values = (_result(batch.specs[0], outcome=failure.outcome, failure=failure), values[1])
        return DensityMemoBatchResultV1(values, delta)
    with pytest.raises((ValueError, TypeError)):
        candidate.attach_density_compilation_batch_v1(compilation, compute, density_exact_memo=memo, **_memo_kwargs())
    assert not memo.density_attempt_memo


def test_internal_exception_merges_success_delta_and_preserves_completed_and_partial_facts(inputs):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1
    from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch_with_memo
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector
    from test_density_attempt_memo_v1 import _batch
    compilation, raw = inputs
    raw = tuple(sorted(raw, key=lambda item: item.spec_id))
    cold = evaluate_raw_density_batch_with_memo(_batch(), DensityAttemptMemoSnapshotV1())
    error = ZeroDivisionError("same internal exception")
    error.density_attempt_memo_delta = cold.delta
    error.density_completed_results = (_result(raw[0]),)
    error.density_partial_work_articles = _result(raw[1]).ordered_work_articles
    error.density_partial_shell_events = cold.results[0].shell_events
    error.density_partial_canon_usage = cold.results[0].canon_usage
    def compute(batch, *, memo_snapshot):
        raise error
    work, memo, collector = [], _DensityExactMemo(), DensityShellEventCollector()
    with pytest.raises(ZeroDivisionError) as caught:
        candidate.attach_density_compilation_batch_v1(compilation, compute, density_exact_memo=memo,
            ordered_work=work, shell_event_collector=collector, **_memo_kwargs())
    assert caught.value is error and len(memo.density_attempt_memo) == len(cold.delta.entries)
    assert tuple(work) == error.density_completed_results[0].ordered_work_articles + error.density_partial_work_articles
    assert len([item for item in collector.diagnostics() if item.outcome.value == "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED"]) == 2


def test_malformed_internal_prefix_cannot_merge_success_delta(inputs):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1
    from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch_with_memo
    from test_density_attempt_memo_v1 import _batch
    compilation, _ = inputs
    delta = evaluate_raw_density_batch_with_memo(_batch(), DensityAttemptMemoSnapshotV1()).delta
    error = ZeroDivisionError("internal error with malformed protocol prefix")
    error.density_attempt_memo_delta = delta
    error.density_completed_results = (object(),)
    def compute(*args, **kwargs):
        raise error
    memo = _DensityExactMemo()
    with pytest.raises(TypeError, match="unknown type"):
        candidate.attach_density_compilation_batch_v1(compilation, compute, density_exact_memo=memo, **_memo_kwargs())
    assert not memo.density_attempt_memo


def test_real_shared_compile_matches_current_oracle_cold_four_warm_zero(monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.reference import adaptive_density_atlas as atlas
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector
    source, context = _case_inputs(monkeypatch, "building_patch10_density4_v1", "decal_request.json")
    compilation = context.compilation
    budgets = []
    original = atlas.DensityExactWorkBudget.__init__
    def observe(budget, exhausted, cap):
        original(budget, exhausted, cap)
        budgets.append(budget)
    monkeypatch.setattr(atlas.DensityExactWorkBudget, "__init__", observe)
    old_memo, new_memo = _DensityExactMemo(), _DensityExactMemo()
    old, new, counts = [], [], []
    for _ in range(2):
        budgets.clear()
        previous, _ = compiler._attach_direction_bindings_legacy_v1(compilation, density_exact_memo=old_memo)
        old.append(previous)
        old_prices = tuple((budget.cap, budget.counters()) for budget in budgets)
        budgets.clear()
        work = []
        current, _ = candidate.attach_density_compilation_batch_v1(compilation, backend.density_compute,
            density_exact_memo=new_memo, ordered_work=work, **_memo_kwargs())
        new.append(current)
        assert tuple((article.cap, article.counters) for article in work) == old_prices
        assert tuple((budget.cap, budget.counters()) for budget in budgets) == old_prices
        counts.append(len(work))
        assert current.envelope_specs == previous.envelope_specs
        assert current.canonical_subturn_fan_authorities == previous.canonical_subturn_fan_authorities
        assert current.evaluation_binding_noise_records == previous.evaluation_binding_noise_records
    assert counts == [4, 0] and len(new_memo.density_attempt_memo) == len(old_memo.fan_bindings) == 2
    assert new[0] == new[1] and old[0] == old[1]


def _later_invalid_anchor_compilation(monkeypatch):
    from cftuv_envelope.validation import validate_analysis_snapshot, validate_snapshot_request_references
    source, context = _case_inputs(monkeypatch, "building_patch10_density4_v1", "decal_request.json")
    compilation = context.compilation
    ordered = sorted(_selections(context), key=lambda row: row[0].envelope_spec_id.value)
    snapshot = compilation.analysis_snapshot
    relation = next(item for item in snapshot.corner_relations if item.corner_relation_id == ordered[-1][0].source_relation_id)
    sector = next(item for item in snapshot.angular_owner_sectors if item.owner_sector_id == relation.owner_sector_id)
    incident = {vertex for use in sector.ordered_incident_chain_use_ids
                for vertex in context.chains_by_id[context.uses_by_id[use].physical_chain_id].ordered_source_vertex_ids}
    vertex = next(item.vertex_id for item in snapshot.source_vertices if item.vertex_id not in incident)
    changed = replace(snapshot, corner_relations=frozenset(replace(item, source_vertex_id=vertex) if item == relation
                                                          else item for item in snapshot.corner_relations))
    assert not validate_analysis_snapshot(changed)
    assert not validate_snapshot_request_references(changed, compilation.decal_request)
    return replace(compilation, analysis_snapshot=changed)


def test_validated_later_raw_error_does_not_preempt_current_oracle_first_named_failure(monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.reference import adaptive_density_atlas as atlas
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector
    compilation = _later_invalid_anchor_compilation(monkeypatch)
    budgets = []
    original = atlas.DensityExactWorkBudget.__init__
    def observe(budget, exhausted, cap):
        original(budget, exhausted, cap)
        budgets.append(budget)
    monkeypatch.setattr(atlas.DensityExactWorkBudget, "__init__", observe)
    monkeypatch.setattr(atlas, "DENSITY_FAN_PREPARATION_WORK_CAP", 0)
    expected, _ = compiler._attach_direction_bindings_legacy_v1(compilation)
    old_prices = tuple((budget.cap, budget.counters()) for budget in budgets)
    budgets.clear()
    calls, work = [], []
    def compute(batch, *, memo_snapshot):
        calls.append(batch)
        return backend.density_compute(batch, memo_snapshot=memo_snapshot)
    kwargs = _memo_kwargs()
    kwargs["limits"] = replace(kwargs["limits"], preparation_cap=0)
    result, context = candidate.attach_density_compilation_batch_v1(compilation, compute,
        ordered_work=work, shell_event_collector=DensityShellEventCollector(), **kwargs)
    assert len(calls) == 1 and 0 < len(calls[0].specs) < len(tuple(candidate._eligible_specs(compilation)))
    assert result.outcome == expected.outcome and result.diagnostics[0] == expected.diagnostics[0]
    assert context is None and result.compilation is None
    assert tuple((article.cap, article.counters) for article in work) == old_prices
    assert tuple((budget.cap, budget.counters()) for budget in budgets) == old_prices


def test_successful_prefix_retains_memo_prices_events_then_rethrows_exact_raw_exception(monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector
    source, context = _case_inputs(monkeypatch, "building_patch10_density4_v1", "decal_request.json")
    compilation = context.compilation
    last = max(spec.envelope_spec_id.value for spec, _ in _selections(context))
    error = RawDensityExtractionErrorV1("later precise host carrier failure")
    original = candidate.extract_raw_density_spec_input
    def extract(source, context, spec, selection):
        if spec.envelope_spec_id.value == last:
            raise error
        return original(source, context, spec, selection)
    monkeypatch.setattr(candidate, "extract_raw_density_spec_input", extract)
    calls, work, memo, collector = [], [], _DensityExactMemo(), DensityShellEventCollector()
    def compute(batch, *, memo_snapshot):
        calls.append(batch)
        return backend.density_compute(batch, memo_snapshot=memo_snapshot)
    with pytest.raises(RawDensityExtractionErrorV1, match="later precise host carrier failure") as caught:
        candidate.attach_density_compilation_batch_v1(compilation, compute, density_exact_memo=memo,
            ordered_work=work, shell_event_collector=collector, **_memo_kwargs())
    assert caught.value is error and len(calls) == 1 and len(calls[0].specs) == len(_selections(context))-1
    assert len(work) == 2 and len(memo.density_attempt_memo) == 1
    assert len(error.density_completed_results) == len(calls[0].specs) and error.density_attempt_memo_delta.entries
    assert len([item for item in collector.diagnostics() if item.outcome.value == "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED"]) == len(calls[0].specs)


def test_first_raw_extraction_exception_is_raised_before_any_batch(inputs, monkeypatch):
    compilation, _ = inputs
    error = RawDensityExtractionErrorV1("first exact host carrier failure")
    def fail(*_):
        raise error
    monkeypatch.setattr(candidate, "extract_raw_density_spec_input", fail)
    with pytest.raises(RawDensityExtractionErrorV1) as caught:
        candidate.attach_density_compilation_batch_v1(compilation, _forbidden, **_memo_kwargs())
    assert caught.value is error


def test_private_failed_prefix_rejects_missing_misordered_and_nonterminal_results(inputs):
    from cftuv_envelope.reference.density_attach_v1 import _attach_density_failed_prefix_v1
    compilation, raw = inputs
    raw = tuple(sorted(raw, key=lambda item: item.spec_id))
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    failed = _result(raw[0], outcome=failure.outcome, failure=failure)
    for declarations in (raw[1:], tuple(reversed(raw))):
        with pytest.raises(ValueError, match="exact eligible prefix"):
            _attach_density_failed_prefix_v1(compilation, FixedFrontend.canon_ids, declarations,
                                            ((declarations[0].spec_id, failed),))
    with pytest.raises(ValueError, match="terminal named failure"):
        _attach_density_failed_prefix_v1(compilation, FixedFrontend.canon_ids, raw[:1], ((raw[0].spec_id, _result(raw[0])),))
