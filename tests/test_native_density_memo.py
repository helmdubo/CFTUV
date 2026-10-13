"""Живой закрытый memo boundary: Python/Rust receipts и ordered Density work."""
from dataclasses import replace
from fractions import Fraction
import importlib
import os
from pathlib import Path
import sys

import pytest
from native_gate import skip_unless_available

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel/src'))
from cftuv_envelope.reference import density_attempt_memo_v1 as memo
from cftuv_envelope.reference import density_oracle_v1 as facts
from cftuv_envelope.reference import adaptive_density_atlas as atlas
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch_with_memo
from test_native_density_boundary import _raw, _batch, _observed


@pytest.fixture
def native():
    path = os.environ.get('CFTUV_DENSITY_CANDIDATE_PACKAGE')
    if path:
        sys.path.insert(0, path)
    module = importlib.import_module('cftuv_native')
    if path:
        assert Path(module.__file__).is_relative_to(Path(path))
    else:
        skip_unless_available(module, 'density')
    return module


def _call(native, batch, snapshot):
    method = native.density_compute_with_memo_candidate if os.environ.get('CFTUV_DENSITY_CANDIDATE_PACKAGE') else native.density_compute_with_memo
    return method(batch, snapshot)


def _budgets(monkeypatch):
    budgets = []
    original = atlas.DensityExactWorkBudget.__init__
    def remember(budget, exhausted, cap):
        original(budget, exhausted, cap)
        budgets.append(budget)
    monkeypatch.setattr(atlas.DensityExactWorkBudget, '__init__', remember)
    return budgets


@pytest.mark.parametrize('q,search', [(4, 1), (5, 131072)])
def test_cross_backend_receipts_and_all_observable_facts_roundtrip(native, q, search):
    batch = _batch((_raw(q=q),), search=search)
    empty = memo.DensityAttemptMemoSnapshotV1()
    python = evaluate_raw_density_batch_with_memo(batch, empty)
    rust = _call(native, batch, empty)
    assert tuple(map(_observed, python.results)) == tuple(map(_observed, rust.results))
    assert rust.delta == python.delta
    assert rust.delta.entries
    for producer, consumer in ((python, lambda b, s: _call(native, b, s)), (rust, evaluate_raw_density_batch_with_memo)):
        warm = consumer(batch, producer.delta)
        expected = evaluate_raw_density_batch_with_memo(batch, python.delta)
        assert tuple(map(_observed, warm.results)) == tuple(map(_observed, expected.results))
        assert warm.delta == empty


def test_metric_alias_failed_narrow_prices_are_replayed_once_per_batch(native, monkeypatch):
    batch = _batch((_raw(),), search=1)
    cold = _call(native, batch, memo.DensityAttemptMemoSnapshotV1())
    raw = batch.specs[0]
    alias = replace(raw, spec_id='alias',
        source_geometry=replace(raw.source_geometry, metric=replace(raw.source_geometry.metric, metric_id='alias-metric')),
        evaluation_geometry=replace(raw.evaluation_geometry, metric=replace(raw.evaluation_geometry.metric, metric_id='alias-metric')))
    budgets = _budgets(monkeypatch)
    warm = _call(native, replace(batch, specs=(alias,)), cold.delta)
    assert len(cold.results[0].ordered_work_articles) == 3
    assert warm.results[0].ordered_work_articles == cold.results[0].ordered_work_articles[:2]
    assert warm.results[0].shell_events == cold.results[0].shell_events
    assert warm.results[0].canon_usage == cold.results[0].canon_usage
    assert [(b.cap, b.counters()) for b in budgets] == [(a.cap, a.counters) for a in warm.results[0].ordered_work_articles]
    assert warm.delta.entries == ()


def test_success_hit_ignores_lower_caps_and_declaration_flag_without_any_budget(native, monkeypatch):
    raw = _raw(q=5)
    batch = replace(_batch((raw,)), window_law='VORONOI_OF_NEIGHBOUR_MIDPOINTS')
    cold = _call(native, batch, memo.DensityAttemptMemoSnapshotV1())
    altered = replace(raw, spec_id='different', canonical=replace(raw.canonical, noise_policy_declaration=(('binding_present', True),)))
    lowered = replace(batch, specs=(altered,), limits=replace(batch.limits, preparation_cap=0, search_cap=0, height_cap=0, box_cap=0))
    budgets = _budgets(monkeypatch)
    warm = _call(native, lowered, cold.delta)
    assert warm.results[0].failure is None
    assert warm.results[0].ordered_work_articles == ()
    assert warm.results[0].shell_events == cold.results[0].shell_events
    assert warm.results[0].canon_usage == cold.results[0].canon_usage
    assert warm.delta.entries == () and budgets == []


def test_internal_prefix_retains_closed_delta_completed_and_partial_aliases(native, monkeypatch):
    raw = _raw(q=5)
    bad = replace(raw, spec_id='b', source_geometry=replace(raw.source_geometry, metric=replace(raw.source_geometry.metric, gram=(Fraction(0),)*3)))
    batch = _batch((raw, bad))
    budgets = _budgets(monkeypatch)
    with pytest.raises(native.DensityNativeInternalErrorV1) as caught:
        _call(native, batch, memo.DensityAttemptMemoSnapshotV1())
    error = caught.value
    assert error.spec_index == 1
    assert error.density_completed_results == error.completed_results and len(error.completed_results) == 1
    assert error.density_partial_work_articles == error.partial_effects.ordered_work_articles
    assert error.density_partial_shell_events == error.partial_effects.shell_events
    assert error.density_partial_canon_usage == error.partial_effects.canon_usage
    assert error.density_attempt_memo_delta.entries
    articles = (*error.completed_results[0].ordered_work_articles, *error.partial_effects.ordered_work_articles)
    assert [(b.cap, b.counters()) for b in budgets] == [(a.cap, a.counters) for a in articles]
    assert _call(native, replace(batch, specs=(raw,)), error.density_attempt_memo_delta).delta.entries == ()


def _unchecked_snapshot(entries):
    result = object.__new__(memo.DensityAttemptMemoSnapshotV1)
    object.__setattr__(result, 'entries', entries)
    return result


@pytest.mark.parametrize('mutation', ['duplicate', 'canon', 'bool_q', 'literal', 'decimal', 'shell'])
def test_malformed_snapshot_is_loud_before_native_or_any_budget(native, monkeypatch, mutation):
    batch = _batch((_raw(),), search=1)
    receipt = _call(native, batch, memo.DensityAttemptMemoSnapshotV1()).delta.entries[0]
    if mutation == 'duplicate':
        entries = (receipt, receipt)
    else:
        if mutation == 'canon':
            receipt = replace(receipt, key=replace(receipt.key, canon_ids=replace(receipt.key.canon_ids, angle='other')))
        elif mutation == 'bool_q':
            receipt = replace(receipt, key=replace(receipt.key, q=True))
        elif mutation == 'literal':
            receipt = replace(receipt, key=replace(receipt.key, ideal_primal_literals=((('sympy',), ('Q',0,1)),)*len(receipt.key.ideal_primal_literals)))
        elif mutation == 'decimal':
            cert = receipt.binding_certificates[0]
            bad = object.__new__(facts.DecimalValueFactsV1)
            object.__setattr__(bad,'coefficient',True)
            object.__setattr__(bad,'exponent',-27)
            receipt = replace(receipt, binding_certificates=(replace(cert, certified_window_width_lower_bound=bad),))
        else:
            receipt = replace(receipt, shell_events=(facts.DensityShellEventFactsV1((1,), ('invalid-code',), None, None, None, None),))
        entries = (receipt,)
    snapshot = _unchecked_snapshot(entries)
    budgets = _budgets(monkeypatch)
    # Прямая closed decoder граница обязана отказать без Rust evaluation.
    with pytest.raises((TypeError, ValueError, native.NativePortUnsupported)):
        native._core.density_compute_with_memo(batch, snapshot, Fraction)
    assert budgets == []
    with pytest.raises((TypeError, ValueError)):
        _call(native,batch,snapshot)
    assert budgets == []


@pytest.mark.parametrize('mutation',['bool_q','shell'])
def test_malformed_delta_is_rejected_before_replay_of_valid_completed_results(native, monkeypatch, mutation):
    batch = _batch((_raw(),), search=1)
    good = _call(native, batch, memo.DensityAttemptMemoSnapshotV1())
    receipt = good.delta.entries[0]
    if mutation=='bool_q':
        receipt = replace(receipt,key=replace(receipt.key,q=True))
    else:
        receipt = replace(receipt,shell_events=(facts.DensityShellEventFactsV1((1,),('invalid-code',),None,None,None,None),))
    budgets = _budgets(monkeypatch)
    with pytest.raises((TypeError,ValueError)):
        native.density_op.density_compute_with_memo(lambda *args: ((good.results,None),_unchecked_snapshot((receipt,))), batch, memo.DensityAttemptMemoSnapshotV1(), candidate=True)
    assert budgets == []


def test_empty_batch_has_empty_delta_and_public_pinned_call_is_available(native):
    batch = replace(_batch((_raw(),)), specs=())
    answer = _call(native,batch,memo.DensityAttemptMemoSnapshotV1())
    assert answer == memo.DensityMemoBatchResultV1((),memo.DensityAttemptMemoSnapshotV1())
    assert 'density' in native.pin.OPERATION_FILES
    assert native.density_compute_with_memo(batch,memo.DensityAttemptMemoSnapshotV1()) == answer


def test_named_failure_preserves_success_delta_and_stops_at_failed_spec(native):
    raw = _raw(q=5)
    bad = replace(raw,spec_id='b',source_geometry=replace(raw.source_geometry,incoming=replace(raw.source_geometry.incoming,owner_left=None)))
    result = _call(native,_batch((raw,bad,replace(raw,spec_id='c'))),memo.DensityAttemptMemoSnapshotV1())
    assert len(result.results)==2 and result.results[1].failure is not None
    assert result.delta.entries
    assert _call(native,_batch((raw,)),result.delta).delta.entries==()


def test_successful_receipt_replays_nonempty_shell_events_across_python_rust(native):
    raw = _raw(q=6)
    geometry = replace(raw.source_geometry,outgoing=replace(raw.source_geometry.outgoing,end=facts.RawDensityPointV1(2,-1)))
    raw = replace(raw,source_geometry=geometry,evaluation_geometry=geometry,
        selection=replace(raw.selection,source_hidden_count=3,bucket_c=4),
        canonical=replace(raw.canonical,source_reflex_interval=(Fraction(7,12),)*2))
    batch = replace(_batch((raw,)),window_law='VORONOI_OF_NEIGHBOUR_MIDPOINTS')
    cold = _call(native,batch,memo.DensityAttemptMemoSnapshotV1())
    assert cold.results[0].shell_events
    assert cold.delta.entries[0].shell_events
    python = evaluate_raw_density_batch_with_memo(batch,memo.DensityAttemptMemoSnapshotV1())
    assert cold.delta == python.delta
    for receipt in (python.delta,cold.delta):
        warm = _call(native,batch,receipt)
        assert warm.results[0].shell_events == cold.results[0].shell_events
        assert warm.results[0].ordered_work_articles == ()
