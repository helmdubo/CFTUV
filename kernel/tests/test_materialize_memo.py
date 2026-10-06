"""Память alpha-независимой работы материализатора (`materialize.memo`): попадание побитово равно промаху.

Утверждения (каждое стоит на проверке, а не на словах докстроки модуля):

1. ПОПАДАНИЕ == ПРОМАХ == БЕЗ ПАМЯТИ. Домен на ОДНОЙ подготовке, посчитанный без памяти, с промахом и с попаданием, а
   также на другой alpha: батч (канонические байты), дайджесты, нормали смещения, диагностики и ВСЕ счётчики (в том числе
   цена `EXACT_WORK_*`) равны. Память держит таблицу станций и подъём домена; от alpha они не зависят.
2. ЦЕНА ВОСПРОИЗВЕДЕНА. После попадания бюджет и память канонизации равны тем, что оставил счёт.
3. ПОТОЛОК - АВТОРИТЕТ ОТКАЗА. Записанная цена, не влезающая в остаток потолка, не повторяется: домен считается заново и
   отказывает там же и с теми же числами, что без памяти.
4. ПИКЛ ПУСТОЙ. Память не едет с подготовкой: байты пикла не зависят от того, что процесс успел посчитать (ключ пикла
   в памяти воркера - хеш самой подготовки), а распакованная копия начинает с пустой памяти.
5. ГРАНИЦА. Записи ограничены, смена отпечатка кода не отдаёт записей прежнего кода, память выключается, подготовка без памяти
   (собранная мимо `prepare_conveyor`) считается как прежде.
"""

from __future__ import annotations

import pickle

import pytest

import developable_factories as df
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope import exact_sqrt_sum as esq
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.materialize import memo as memo_module
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.memo import MaterializeMemoV1, memo_disabled, memo_of
from cftuv_envelope.wavefront import conveyor_coverage

ROUTE = ("r0a", "r0b")
FIXTURES = {
    "fold": (df.fold_strip, "3.5", "3.0"),
    "quarter": (df.quarter_cylinder, "1.6", "1.2"),
}
LAW = DecalTopologyLawV1.PLANAR_POLYGONS_V1
LIFT = NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1


def domain(name):
    make, first, _other = FIXTURES[name]
    snapshot, request = developable_domain(make(), ROUTE, alpha=first)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def materialize(prepared, alpha, **kwargs):
    coverage = conveyor_coverage(prepared, alpha)
    assert coverage.outcome.value == "EXACT", coverage.detail
    return materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=LIFT,
        decal_topology_law=LAW,
        **kwargs,
    )


def answer(result):
    """Всё, что составляет ответ домена, включая цену `EXACT_WORK_*`."""

    return (
        result.outcome.value,
        result.detail,
        tuple(result.counters),
        tuple(result.diagnostics),
        result.content_digest,
        result.offset_normals_digest,
        tuple(result.vertex_normals),
        None if result.batch is None else canonical_json_bytes(result.batch),
    )


@pytest.mark.parametrize("name", FIXTURES)
def test_a_hit_is_byte_identical_to_a_miss_and_to_a_build_without_the_memo(name):
    _make, first, other = FIXTURES[name]
    prepared = domain(name)
    with memo_disabled():
        off_first = materialize(prepared, first)
        off_other = materialize(prepared, other)
    assert off_first.is_materialized and off_other.is_materialized
    memo = memo_of(prepared)
    assert len(memo) == 0, "a build without the memo must not write it"
    miss = materialize(prepared, first)
    assert (memo.hits, memo.misses) == (0, 2)  # станции и подъём домена: один промах на каждое
    hit = materialize(prepared, first)
    assert memo.hits == 2
    wide = materialize(prepared, other)
    assert memo.hits == 4, "the memo is alpha-independent: another width hits it too"
    assert answer(off_first) == answer(miss) == answer(hit)
    assert answer(off_other) == answer(wide)
    assert answer(miss) != answer(wide), "two widths must differ or the equality discriminates nothing"


def test_a_remembered_value_is_never_changed_by_the_materializations_that_read_it():
    """Таблица станций и подъём общие у всех материализаций подготовки: потребитель, правящий их, испортил бы следующий домен."""

    prepared = domain("quarter")
    _make, first, other = FIXTURES["quarter"]
    materialize(prepared, first)
    memo = memo_of(prepared)
    snapshot = {key: pickle.dumps(entry, protocol=5) for key, entry in memo.entries.items()}
    assert len(snapshot) >= 2
    for width in (first, other, first):
        materialize(prepared, width)
    assert {key: pickle.dumps(entry, protocol=5) for key, entry in memo.entries.items()} == snapshot


def test_the_price_and_the_factorization_memory_after_a_hit_equal_those_of_a_build():
    prepared = domain("quarter")
    _make, first, _other = FIXTURES["quarter"]
    marks = {}

    def measure(label):
        result = materialize(prepared, first)
        assert result.is_materialized
        # Память канонизации ПОСЛЕ материализации (её ключи) и цена: то, что стадии после станций читают как историю.
        marks[label] = (
            tuple(sorted(item for item in result.counters if item[0].startswith("EXACT_WORK_"))),
            esq.factorization_memory_marker(),
        )

    with memo_disabled():
        measure("off")
    measure("miss")
    measure("hit")
    assert marks["off"] == marks["miss"] == marks["hit"]
    assert dict(marks["hit"][0])["EXACT_WORK_SPENT"] > 0


def test_a_recorded_price_beyond_the_ceiling_is_not_replayed_and_the_domain_fails_as_without_the_memo():
    prepared = domain("quarter")
    _make, first, _other = FIXTURES["quarter"]
    full = materialize(prepared, first)
    spent = dict(full.counters)["EXACT_WORK_SPENT"]
    assert spent > 4
    for cap in (spent - 1, spent // 2, 3):
        results = {}
        for label, switch in (("off", True), ("memo", False)):
            budget = exact_work_budget(stage="MATERIALIZE", domain_id="d", cap=cap)
            if switch:
                with memo_disabled():
                    results[label] = materialize(prepared, first, work_budget=budget)
            else:
                results[label] = materialize(prepared, first, work_budget=budget)
        assert results["off"].outcome.value == "EXACT_WORK_BUDGET_EXHAUSTED", cap
        assert answer(results["off"]) == answer(results["memo"]), cap


def test_the_pickle_of_a_preparation_does_not_carry_the_memo():
    prepared = domain("fold")
    _make, first, _other = FIXTURES["fold"]
    blob_before = pickle.dumps(prepared, protocol=5)
    materialize(prepared, first)
    assert len(memo_of(prepared)) > 0
    assert pickle.dumps(prepared, protocol=5) == blob_before, "the memo must not move the pickle (and its worker-store key)"
    clone = pickle.loads(blob_before)
    assert len(memo_of(clone)) == 0 and (memo_of(clone).hits, memo_of(clone).misses) == (0, 0)
    assert answer(materialize(clone, first)) == answer(materialize(prepared, first))


def test_the_memo_is_bounded_and_keyed_by_the_code_fingerprint(monkeypatch):
    memo = MaterializeMemoV1()
    for number in range(memo_module.MEMO_ENTRY_LIMIT * 3):
        memo.remembered(("probe", number), lambda number=number: number)
    assert len(memo) == memo_module.MEMO_ENTRY_LIMIT
    assert memo.remembered(("probe", memo_module.MEMO_ENTRY_LIMIT * 3 - 1), lambda: "recomputed") == memo_module.MEMO_ENTRY_LIMIT * 3 - 1
    monkeypatch.setattr(memo_module, "kernel_code_identity", lambda: "another code")
    assert memo.remembered(("probe", memo_module.MEMO_ENTRY_LIMIT * 3 - 1), lambda: "recomputed") == "recomputed"


def test_the_registry_of_live_memories_clears_every_preparation_for_tests_that_replace_a_stage():
    prepared = domain("fold")
    _make, first, _other = FIXTURES["fold"]
    materialize(prepared, first)
    memo = memo_of(prepared)
    assert len(memo) > 0
    memo_module.clear_live_memos()
    assert len(memo) == 0 and (memo.hits, memo.misses) == (0, 0)
    clone = pickle.loads(pickle.dumps(prepared, protocol=5))
    materialize(clone, first)
    assert len(memo_of(clone)) > 0
    memo_module.clear_live_memos()
    assert len(memo_of(clone)) == 0, "a memory made by unpickling is registered as well"


def test_a_preparation_without_the_memo_is_materialized_as_before():
    import dataclasses

    prepared = domain("fold")
    _make, first, _other = FIXTURES["fold"]
    bare = dataclasses.replace(prepared, materialize_memo=None)
    assert memo_of(bare) is None
    with memo_disabled():
        reference = materialize(prepared, first)
    assert answer(materialize(bare, first)) == answer(reference)
