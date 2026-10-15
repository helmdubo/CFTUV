"""Переносимый success-only memo сохраняет старую оплату реальных попыток."""

from dataclasses import fields, is_dataclass, replace
from fractions import Fraction
import pickle

import pytest

from cftuv_envelope.reference import density_oracle_v1 as facts
from cftuv_envelope.reference.density_attempt_memo_v1 import (
    DensityAttemptMemoSnapshotV1, DensityAttemptMemoV1, density_attempt_key, merge_density_attempt_delta,
)
from cftuv_envelope.reference.density_raw_frontend_v1 import (
    RawDensityFrontendV1, evaluate_raw_density_batch, evaluate_raw_density_batch_with_memo,
)
from cftuv_envelope.reference.metric import _DensityExactMemo
from test_density_raw_frontend_v1 import _frontend, _forty_five_degree_raw


def _batch(*, search_cap=None, law="NARROW_ROTATION_BAND"):
    template = _frontend()
    limits = template.limits if search_cap is None else replace(template.limits, search_cap=search_cap)
    return facts.RawDensityBatchInputV1((_forty_five_degree_raw(),), template.canon_ids, limits, law)


def _closed(value):
    if type(value) in (str, int, bool, type(None), Fraction):
        return
    if type(value) is tuple:
        for item in value:
            _closed(item)
        return
    assert is_dataclass(value) and value.__dataclass_params__.frozen, type(value)
    for field in fields(value):
        _closed(getattr(value, field.name))


def test_cold_and_warm_across_new_arenas_preserve_all_facts_without_repricing():
    batch = _batch()
    cold = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    assert cold.results == evaluate_raw_density_batch(batch)
    assert cold.results[0].ordered_work_articles and cold.delta.entries
    _closed(cold)
    restored = pickle.loads(pickle.dumps(cold.delta))
    warm = evaluate_raw_density_batch_with_memo(batch, restored)
    assert warm.results == (replace(cold.results[0], ordered_work_articles=()),)
    assert warm.delta == DensityAttemptMemoSnapshotV1()
    # Старый вызов остаётся stateless и оплачивает те же холодные попытки.
    assert evaluate_raw_density_batch(batch) == cold.results


def test_success_survives_lower_caps_without_putting_limits_in_the_key():
    batch = _batch()
    cold = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    lowered = replace(batch, limits=replace(batch.limits, preparation_cap=0, search_cap=0, box_cap=0, height_cap=0))
    warm = evaluate_raw_density_batch_with_memo(lowered, cold.delta)
    assert warm.results == (replace(cold.results[0], ordered_work_articles=()),)
    assert evaluate_raw_density_batch(lowered)[0].failure is not None


def test_metric_identity_and_declaration_flag_do_not_partition_success_attempts():
    batch = _batch()
    raw = batch.specs[0]
    alias = replace(raw.source_geometry, metric=replace(raw.source_geometry.metric, metric_id="same-gram-other-id"))
    second = replace(raw, spec_id="z-other-spec", source_geometry=alias, evaluation_geometry=alias,
        canonical=replace(raw.canonical, noise_policy_declaration=(("binding_present", True),)))
    cold = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    warm = evaluate_raw_density_batch_with_memo(replace(batch, specs=(second,)), cold.delta)
    assert warm.results[0].failure is None and not warm.results[0].ordered_work_articles
    assert warm.results[0].authority == cold.results[0].authority
    assert not warm.delta.entries
    together = evaluate_raw_density_batch(replace(batch, specs=(raw, second)))
    assert together[1] == warm.results[0]


def test_failed_narrow_attempt_retries_and_repays_with_warm_voronoi():
    batch = _batch(search_cap=1)
    cold = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    warm = evaluate_raw_density_batch_with_memo(batch, cold.delta)
    assert cold.results[0].failure is None and warm.results[0].failure is None
    assert len(cold.results[0].ordered_work_articles) == 3
    assert warm.results == (replace(cold.results[0], ordered_work_articles=cold.results[0].ordered_work_articles[:2]),)
    assert cold.results[0].named_refusals
    assert tuple(entry.key.window_law for entry in cold.delta.entries) == ("VORONOI_OF_NEIGHBOUR_MIDPOINTS",)
    assert not warm.delta.entries


def test_success_prefix_survives_later_named_failure_and_batch_stops():
    batch = _batch()
    raw = batch.specs[0]
    ambiguous = replace(raw.source_geometry, incoming=replace(raw.source_geometry.incoming, owner_left=None))
    second = replace(raw, spec_id="z-ambiguous", source_geometry=ambiguous)
    third = replace(raw, spec_id="zz-unreached")
    result = evaluate_raw_density_batch_with_memo(replace(batch, specs=(raw, second, third)), DensityAttemptMemoSnapshotV1())
    first = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    assert len(result.results) == 2 and result.results[1].failure is not None
    assert result.delta == first.delta
    assert evaluate_raw_density_batch_with_memo(batch, result.delta).results[0].ordered_work_articles == ()


def test_success_prefix_and_partial_prices_survive_later_internal_failure(monkeypatch):
    batch = _batch()
    second = replace(batch.specs[0], spec_id="z-internal-failure")
    original = RawDensityFrontendV1._compute
    error = ZeroDivisionError("named internal partial-domain failure")

    def compute(frontend, raw):
        if raw.spec_id == second.spec_id:
            frontend.laws.preparation_budget(lambda cap: __import__(
                "cftuv_envelope.reference.adaptive_density_atlas", fromlist=["DensityExactWorkBudget"]
            ).DensityExactWorkBudget("internal-phase", cap))
            raise error
        return original(frontend, raw)

    monkeypatch.setattr(RawDensityFrontendV1, "_compute", compute)
    with pytest.raises(ZeroDivisionError, match="named internal partial-domain failure") as caught:
        evaluate_raw_density_batch_with_memo(replace(batch, specs=(batch.specs[0], second)), DensityAttemptMemoSnapshotV1())
    assert caught.value is error
    assert len(error.density_completed_results) == 1
    assert error.density_attempt_memo_delta.entries
    assert len(error.density_partial_work_articles) == 1
    warm = evaluate_raw_density_batch_with_memo(batch, error.density_attempt_memo_delta)
    assert not warm.results[0].ordered_work_articles


def test_host_memo_is_separate_from_legacy_and_cleared_on_pickle():
    result = evaluate_raw_density_batch_with_memo(_batch(), DensityAttemptMemoSnapshotV1())
    memo = _DensityExactMemo()
    merge_density_attempt_delta(memo.density_attempt_memo, result.delta)
    assert memo.density_attempt_memo and not memo.fan_bindings
    restored = pickle.loads(pickle.dumps(memo))
    assert not restored.density_attempt_memo and not restored.fan_bindings


def test_snapshot_duplicates_fail_before_work_and_empty_batch_has_empty_delta(monkeypatch):
    batch = _batch()
    cold = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())

    def forbidden(*args, **kwargs):
        raise AssertionError("duplicate snapshot reached geometry work")

    monkeypatch.setattr(RawDensityFrontendV1, "_compute", forbidden)
    with pytest.raises(ValueError, match="duplicate attempt keys"):
        DensityAttemptMemoSnapshotV1(cold.delta.entries * 2)
    with pytest.raises(TypeError, match="malformed result facts"):
        DensityAttemptMemoSnapshotV1((replace(cold.delta.entries[0], shell_events=[]),))
    empty = evaluate_raw_density_batch_with_memo(replace(batch, specs=()), cold.delta)
    assert empty.results == () and empty.delta.entries == ()


def test_delta_retains_first_success_order_for_different_attempt_keys():
    batch = _batch()
    raw = batch.specs[0]
    scaled = replace(raw.source_geometry, metric=replace(raw.source_geometry.metric,
        gram=(4, 0, 4), inverse_gram=(Fraction(1, 4), 0, Fraction(1, 4))))
    second = replace(raw, spec_id="z-scaled", source_geometry=scaled, evaluation_geometry=scaled)
    first_alone = evaluate_raw_density_batch_with_memo(batch, DensityAttemptMemoSnapshotV1())
    second_alone = evaluate_raw_density_batch_with_memo(replace(batch, specs=(second,)), DensityAttemptMemoSnapshotV1())
    together = evaluate_raw_density_batch_with_memo(replace(batch, specs=(raw, second)), DensityAttemptMemoSnapshotV1())
    assert together.delta.entries == first_alone.delta.entries + second_alone.delta.entries
    assert len(together.delta.entries) == 2


def test_key_is_literal_identity_not_mathematical_value_or_arena_id():
    from cftuv_envelope.contracts.analysis import TurnOrientation
    template = _frontend()
    laws = template.laws
    metric = laws.metric((1, 0, 1), (1, 0, 1), 1)
    raw = laws.arena.raw_cos(laws.arena.rational(0))
    fan_literal = laws.vector(1, 0), laws.vector(raw, 1), laws.vector(0, 1)
    fan_one = laws.vector(1, 0), laws.vector(1, 1), laws.vector(0, 1)
    arguments = (TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 2, (None,), False,
                 "VORONOI_OF_NEIGHBOUR_MIDPOINTS", template.canon_ids)
    literal = density_attempt_key(laws, metric, fan_literal, *arguments)
    one = density_attempt_key(laws, metric, fan_one, *arguments)
    assert literal != one
    assert literal.ideal_primal_literals[1][0] == ("cos", ("Q", 0, 1))
    _closed(literal)
    assert DensityAttemptMemoV1().lookup(literal) is None
