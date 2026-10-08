"""Корпус вызовов нативного ускорителя (`tools/native_corpus.py`): запись -> воспроизведение -> точное сравнение.

Нативная сборка ядра заменит две операции целиком (`wavefront.coverage._coverage_at`, `materialize.clip.clip_geometry`), и
доказательство замены — побитовое равенство исходов на корпусе настоящих вызовов. Этот тест держит САМ инструмент сверки на
чистом клоне, без корпуса и без Blender: маленький настоящий домен ядра проходит через рекордер, запись воспроизводится
эталоном (ядро питона) и равна записанному точно, а сравнение ловит каждое подложенное расхождение — иначе «равно» ничего
не значило бы.

1. РЕКОРДЕР пишет обе операции настоящего домена: вход, состояние до и после, исход, секунды.
2. ВОСПРОИЗВЕДЕНИЕ равно записи точно и не зависит от состояния процесса (память канонизации, счётчики знаков, неоплаченное).
3. СРАВНЕНИЕ ловит: другую дробь, `int` вместо `Fraction` (то, что `==` не видит), другой `float` на один ulp, статью бюджета
   на единицу, пропавшую или переставленную запись памяти, счётчик знаков, неоплаченное, `store`, текст исключения.
4. ИСКЛЮЧЕНИЕ операции (исчерпание бюджета) и вызов без бюджета записываются и воспроизводятся.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import pickle
import sys
from fractions import Fraction
from pathlib import Path

import pytest


def test_indexed_derived_cleanup_checks_every_path_before_removing_anything(tmp_path):
    nc = _load_tool("native_corpus")
    derived = tmp_path / "records" / "_derived" / "known.rec"
    derived.parent.mkdir(parents=True)
    derived.write_bytes(b"known")
    foreign = tmp_path / "foreign.rec"
    foreign.write_bytes(b"preserve")
    rows = [{"derived": {}, "path": "records/_derived/known.rec"}, {"derived": {}, "path": "foreign.rec"}]
    with pytest.raises(ValueError, match="escapes"):
        nc.remove_indexed_derived(tmp_path, rows)
    assert derived.read_bytes() == b"known" and foreign.read_bytes() == b"preserve"
    nc.remove_indexed_derived(tmp_path, rows[:1])
    assert not derived.exists() and foreign.read_bytes() == b"preserve"

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import developable_factories as df  # noqa: E402
from developable_route import materialize_developable  # noqa: E402

from cftuv_envelope import exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1  # noqa: E402
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402


def _load_tool(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


nc = _load_tool("native_corpus")

ROUTE = ("r0a", "r0b")


def _cold_process() -> None:
    exact.reset_factorization_memory()
    exact.reset_sign_counts()
    exact.reset_unbudgeted_work()
    exact.set_canonical_audit(False)


@pytest.fixture(autouse=True)
def _process_state_is_given_back():
    saved = nc.capture_state(None, None)
    _cold_process()
    yield
    nc.restore_state(saved)


def _recorder(tmp_path, monkeypatch):
    recorder = nc.Recorder(tmp_path, {"python": sys.version.split()[0], "kernel_identity": "test", "git_head": "test"})
    monkeypatch.setattr(nc.coverage, "_coverage_at", recorder.wrap(nc.OP_COVERAGE, nc.ORACLE[nc.OP_COVERAGE]))
    monkeypatch.setattr(nc.clip, "clip_geometry", recorder.wrap(nc.OP_CLIP, nc.ORACLE[nc.OP_CLIP]))
    monkeypatch.setattr(nc.clip_memo.MEMO, "enabled", False)
    return recorder


@pytest.fixture
def domain(tmp_path, monkeypatch):
    """Настоящий малый домен ядра (складка), прошедший через рекордер: `(рекордер, результат домена)`."""

    recorder = _recorder(tmp_path, monkeypatch)
    recorder.context.update(mesh="fold_strip", mesh_digest="synthetic", alpha="3.5")
    result, _prepared = materialize_developable(
        df.fold_strip(),
        ROUTE,
        alpha="3.5",
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
    )
    assert result.is_materialized, result.detail
    recorder.write_index()
    return recorder, result


def _records(recorder):
    return [nc.read_record(recorder.root / row["path"]) for row in recorder.rows]


def _replay(record):
    before = record.before()
    return before, nc.execute(nc.prepare_call(record.op, record.call_blob, before))


def _first(recorder, op):
    return next(record for record in _records(recorder) if record.op == op)


# --------------------------------------------------------------------------
# 1. Рекордер
# --------------------------------------------------------------------------


def test_the_recorder_writes_both_operations_of_a_real_domain(domain):
    recorder, _result = domain
    ops = [row["op"] for row in recorder.rows]
    assert nc.OP_COVERAGE in ops and nc.OP_CLIP in ops
    index = nc.load_index(recorder.root)
    assert index["records_count"] == len(ops) == len(index["records"])
    assert index["total_bytes"] == sum(row["bytes"] for row in index["records"]) > 0
    for row in recorder.rows:
        assert (recorder.root / row["path"]).stat().st_size == row["bytes"]
        assert nc.read_meta(recorder.root / row["path"])["id"] == row["id"]
        assert row["mesh"] == "fold_strip" and row["seconds"] >= 0.0 and row["outcome"] in {"EXACT", "CLIPPED"}
    clip_row = next(row for row in recorder.rows if row["op"] == nc.OP_CLIP)
    assert clip_row["faces"] > 0 and clip_row["budget"] is True and "exception" in clip_row


def test_the_recorded_call_pickles_the_plane_and_the_partition_with_the_budget_kept_by_identity(domain):
    recorder, _result = domain
    clip_record = _first(recorder, nc.OP_CLIP)
    before = clip_record.before()
    call = nc.decode_call(nc.OP_CLIP, clip_record.call_blob, nc.build_budget(before.budget), None)
    (plane,) = call.args
    assert plane._budget is call.budget
    assert call.budget.spent_by_article() == before.budget["articles"]
    coverage_record = _first(recorder, nc.OP_COVERAGE)
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, coverage_record.call_blob, None, {}).args
    assert type(alpha) is Fraction and partition.faces


def test_the_record_file_round_trips_and_refuses_a_foreign_file(domain, tmp_path):
    recorder, _result = domain
    path = recorder.root / recorder.rows[0]["path"]
    record = nc.read_record(path)
    assert record.meta["id"] == recorder.rows[0]["id"]
    assert set(record.payload) == {"before", "call", "seconds", "expected"}
    assert set(record.payload["expected"]) == {"result", "exception", "after", "observed", "answer_digest"}
    foreign = tmp_path / "foreign.rec"
    foreign.write_bytes(b"not a record at all")
    with pytest.raises(nc.CorpusError):
        nc.read_meta(foreign)


# --------------------------------------------------------------------------
# 2. Воспроизведение
# --------------------------------------------------------------------------


def test_a_replay_reproduces_every_recorded_outcome_exactly(domain):
    recorder, _result = domain
    for record in _records(recorder):
        before, replayed = _replay(record)
        expected = record.expected()
        assert nc.compare_outcomes(record.op, before, expected, replayed) == [], record.meta["id"]
        assert nc.outcome_digest(record.op, before, replayed) == nc.outcome_digest(record.op, before, expected)
        assert replayed.after == expected.after


def test_the_recorded_answer_digest_is_the_digest_of_the_recorded_result(domain):
    recorder, _result = domain
    for record in _records(recorder):
        expected = record.expected()
        assert record.payload["expected"]["answer_digest"] == nc.answer_digest(record.op, expected.result)


def test_a_replay_does_not_depend_on_the_state_the_process_is_in(domain):
    recorder, _result = domain
    record = _first(recorder, nc.OP_CLIP)
    before, clean = _replay(record)
    exact._FACTORIZATION_MEMO[10**40 + 7] = ((10**40 + 7, 1),)
    exact._KNOWN_PRIMES.insert(0, 3)
    exact._KNOWN_PRIME_SET.add(3)
    exact.SIGN_COUNTS["total"] += 12345
    exact.UNBUDGETED_WORK.gcd_operations += 77
    dirty_before, dirty = _replay(record)
    assert nc.compare_outcomes(record.op, before, clean, dirty) == []
    assert nc.compare_outcomes(record.op, before, record.expected(), dirty) == []
    assert dirty_before.sign_counts == before.sign_counts


def test_restore_keeps_the_insertion_order_of_every_memory_table():
    state = nc.capture_state(None, None)
    state.factorization = [(30, ((2, 1), (3, 1), (5, 1))), (7, ((7, 1),)), (11, ((11, 1),))]
    state.squarefree = [(12, (3, 2)), (5, (5, 1))]
    state.prime_support = [(30, (2, 3, 5)), (7, (7,))]
    state.known_primes = [2, 3, 5, 7, 11]
    state.store = [(("prime-universe", (Fraction(1, 3),)), ((2, 3), ()))]
    budget = nc.build_budget(
        {"mode": "BOUNDED", "cap": 100, "stage": "COVERAGE", "domain_id": "d", "superlevel": "s", "articles": (1, 2, 3, 4, 5, 6)}
    )
    state.budget = nc.budget_state(budget)
    rebuilt, store = nc.restore_state(state)
    assert nc.capture_state(rebuilt, store) == state
    assert list(exact._FACTORIZATION_MEMO) == [30, 7, 11]
    assert rebuilt.cap == 100 and rebuilt.stage == "COVERAGE" and rebuilt.spent == 21


# --------------------------------------------------------------------------
# 3. Сравнение ловит подложенное расхождение
# --------------------------------------------------------------------------


def _pair(domain, op):
    recorder, _result = domain
    record = _first(recorder, op)
    before, replayed = _replay(record)
    return record.op, before, record.expected(), replayed


def _fields(differences):
    return [item.field for item in differences]


def test_a_changed_fraction_inside_the_result_is_detected(domain):
    op, before, expected, actual = _pair(domain, nc.OP_COVERAGE)
    face = actual.result.faces[0]
    radicand, coefficient = face.doubled_area.terms[0]
    bent = SqrtSumV1(((radicand, coefficient + Fraction(1, 10**9)), *face.doubled_area.terms[1:]))
    faces = (dataclasses.replace(face, doubled_area=bent), *actual.result.faces[1:])
    actual = dataclasses.replace(actual, result=dataclasses.replace(actual.result, faces=faces))
    found = nc.compare_outcomes(op, before, expected, actual)
    assert _fields(found) == ["result.faces"] and "[0]" in found[0].detail


def test_an_int_in_place_of_a_fraction_is_detected_although_python_calls_them_equal(domain):
    op, before, expected, actual = _pair(domain, nc.OP_COVERAGE)
    as_fraction = SqrtSumV1(((1, Fraction(3)),))
    as_int = SqrtSumV1(((1, 3),))
    assert as_fraction == as_int
    left = dataclasses.replace(expected, result=dataclasses.replace(expected.result, doubled_area=as_fraction))
    right = dataclasses.replace(actual, result=dataclasses.replace(actual.result, doubled_area=as_int))
    assert _fields(nc.compare_outcomes(op, before, left, right)) == ["result.doubled_area"]


def test_a_float_one_ulp_away_and_a_note_change_are_detected(domain):
    op, before, expected, actual = _pair(domain, nc.OP_CLIP)
    normal = (0.5, 0.25, math.sqrt(0.6875))
    nudged = (normal[0], normal[1], math.nextafter(normal[2], math.inf))
    assert normal != nudged and normal[2] - nudged[2] != 0.0
    left = dataclasses.replace(expected, result=dataclasses.replace(expected.result, lifted={"clip:0": ((0.1, 0.2, 0.3), ("t0", normal))}))
    right = dataclasses.replace(actual, result=dataclasses.replace(actual.result, lifted={"clip:0": ((0.1, 0.2, 0.3), ("t0", nudged))}, note=actual.result.note + "!"))
    found = nc.compare_outcomes(op, before, left, right)
    assert set(_fields(found)) == {"result.lifted", "result.note"}
    assert nc.compare_outcomes(op, before, left, dataclasses.replace(right, result=dataclasses.replace(right.result, lifted=left.result.lifted, note=left.result.note))) == []


def test_a_budget_article_off_by_one_is_detected_and_named(domain):
    op, before, expected, actual = _pair(domain, nc.OP_CLIP)
    articles = list(actual.after.budget["articles"])
    articles[4] += 1
    after = dataclasses.replace(actual.after, budget={**actual.after.budget, "articles": tuple(articles)})
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=after))
    assert _fields(found) == ["budget.delta"] and "radical_materializations" in found[0].detail
    stage = dataclasses.replace(actual.after, budget={**actual.after.budget, "stage": "OTHER"})
    assert _fields(nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=stage))) == ["budget.stage"]


def test_a_missing_or_reordered_memory_entry_is_detected(domain):
    op, before, expected, actual = _pair(domain, nc.OP_CLIP)
    assert expected.after.factorization
    extra = [(10**30 + 1, ((10**30 + 1, 1),)), (10**30 + 3, ((10**30 + 3, 1),))]
    left = dataclasses.replace(expected, after=dataclasses.replace(expected.after, factorization=expected.after.factorization + extra))
    reordered = expected.after.factorization + extra[::-1]
    missing = expected.after.factorization + extra[:1]
    same = dataclasses.replace(actual, after=dataclasses.replace(actual.after, factorization=expected.after.factorization + extra))
    assert nc.compare_outcomes(op, before, left, same) == []
    swapped = dataclasses.replace(actual, after=dataclasses.replace(actual.after, factorization=reordered))
    found = nc.compare_outcomes(op, before, left, swapped)
    assert _fields(found) == ["memory.factorization"] and "order" in found[0].detail
    absent = dataclasses.replace(actual, after=dataclasses.replace(actual.after, factorization=missing))
    found = nc.compare_outcomes(op, before, left, absent)
    assert _fields(found) == ["memory.factorization"] and "length" in found[0].detail
    primes = dataclasses.replace(actual, after=dataclasses.replace(actual.after, known_primes=[*actual.after.known_primes, 1000003]))
    assert _fields(nc.compare_outcomes(op, before, expected, primes)) == ["memory.known_primes"]


def test_sign_counts_unbudgeted_store_and_exception_differences_are_detected(domain):
    op, before, expected, actual = _pair(domain, nc.OP_COVERAGE)
    signs = {**actual.after.sign_counts, "closed_by_enclosure": actual.after.sign_counts["closed_by_enclosure"] + 1}
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=dataclasses.replace(actual.after, sign_counts=signs)))
    assert _fields(found) == ["sign_counts.closed_by_enclosure"]
    unbudgeted = (actual.after.unbudgeted[0] + 1, *actual.after.unbudgeted[1:])
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=dataclasses.replace(actual.after, unbudgeted=unbudgeted)))
    assert _fields(found) == ["unbudgeted.delta"]
    assert expected.after.store, "the coverage call remembers its prime universe in the store"
    store = [(key, (value[0], value[1][:-1])) for key, value in actual.after.store]
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=dataclasses.replace(actual.after, store=store)))
    assert _fields(found) == ["store"]
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, exception=("ValueError", "boom")))
    assert _fields(found) == ["exception"]


def test_the_comparison_ignores_what_is_not_the_answer(domain):
    op, before, expected, actual = _pair(domain, nc.OP_CLIP)
    relabelled = dataclasses.replace(actual, result=dataclasses.replace(actual.result, memo="HIT"), seconds=99.0)
    assert nc.compare_outcomes(op, before, expected, relabelled) == []
    op, before, expected, actual = _pair(domain, nc.OP_COVERAGE)
    other_budget = nc.build_budget(actual.after.budget)
    carried = dataclasses.replace(actual, result=dataclasses.replace(actual.result, work_budget=other_budget))
    assert nc.compare_outcomes(op, before, expected, carried) == []


# --------------------------------------------------------------------------
# 4. Исключение операции и вызов без бюджета
# --------------------------------------------------------------------------


def test_an_exhausted_budget_is_recorded_as_an_exception_and_replays_to_the_same_text(domain, monkeypatch):
    recorder, _result = domain
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, _first(recorder, nc.OP_COVERAGE).call_blob, None, {}).args
    _cold_process()
    starved = exact.exact_work_budget(stage="COVERAGE", domain_id="starved", cap=0)
    recorded = nc.coverage._coverage_at
    with pytest.raises(exact.ExactCanonicalizationWorkBudgetExhausted):
        recorded(partition, alpha, starved, {})
    row = recorder.rows[-1]
    assert row["outcome"] == "raised:ExactCanonicalizationWorkBudgetExhausted" and row["exception"][0] == "ExactCanonicalizationWorkBudgetExhausted"
    record = nc.read_record(recorder.root / row["path"])
    before, replayed = _replay(record)
    assert replayed.exception == tuple(row["exception"]) and replayed.result is None
    assert replayed.exception[1].startswith(exact.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED) and "domain=starved" in replayed.exception[1]
    assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []
    assert record.expected().after.budget["articles"] != before.budget["articles"]


def test_a_call_without_a_budget_records_the_unbudgeted_work_and_replays_it(domain):
    recorder, _result = domain
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, _first(recorder, nc.OP_COVERAGE).call_blob, None, {}).args
    _cold_process()
    nc.coverage._coverage_at(partition, alpha, None, None)
    row = recorder.rows[-1]
    assert row["budget"] is False
    record = nc.read_record(recorder.root / row["path"])
    assert record.before().budget is None and record.before().store is None
    before, replayed = _replay(record)
    assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []
    assert any(replayed.after.unbudgeted) and replayed.after.store is None
    assert pickle.loads(record.payload["expected"]["result"]).work_budget is None


# --------------------------------------------------------------------------
# 5. Замер и выгрузка: чистые части инструментов
# --------------------------------------------------------------------------


def test_the_bench_times_a_record_and_names_a_planted_mismatch(domain, monkeypatch):
    bench = _load_tool("native_bench")
    recorder, _result = domain
    row = next(item for item in recorder.rows if item["op"] == nc.OP_CLIP)
    measured = bench.measure_record(recorder.root, row, 2)
    assert measured["differences"] == [] and len(measured["seconds"]) == 2 and measured["median"] > 0.0
    table = bench.aggregate([measured])
    assert table[nc.OP_CLIP]["ALL"]["n"] == 1 and table[nc.OP_CLIP]["fold_strip"]["max"] == max(measured["seconds"])

    original = nc.ORACLE[nc.OP_CLIP]

    def one_more_gcd(plane, budget, **kwargs):
        result = original(plane, budget, **kwargs)
        budget.gcd_operations += 1
        return result

    monkeypatch.setitem(nc.ORACLE, nc.OP_CLIP, one_more_gcd)
    broken = bench.measure_record(recorder.root, row, 2)
    assert [item["repeat"] for item in broken["differences"]] == [1, 2]
    assert broken["differences"][0]["fields"][0].startswith("budget.delta") and "gcd_operations" in broken["differences"][0]["fields"][0]
    assert bench.mismatch_fields([{"differences": broken["differences"]}]) == {"budget.delta": 1}


def test_the_bench_statistics_are_nearest_rank():
    bench = _load_tool("native_bench")
    values = [5.0, 1.0, 4.0, 2.0, 3.0]
    assert bench.percentile(values, 0.5) == 3.0 and bench.percentile(values, 0.95) == 5.0
    assert bench.summarize(values) == {"n": 5, "p50": 3.0, "p95": 5.0, "max": 5.0, "total": 15.0}


def test_the_exporter_builds_distinct_series_and_finds_the_events_inside_the_series_only():
    export = _load_tool("native_corpus_export")
    assert export.parse_plan("building=0.2239;2=0.2239,0.5") == {"building": [0.2239], "2": [0.2239, 0.5]}
    series = export.series_alphas(0.2239, [0.01, 0.001], 5)
    assert len(series) == len(set(series)) == 20 and series == sorted(series) and 0.2239 not in series
    assert all(abs(alpha / 0.2239 - 1) <= 0.0501 for alpha in series)
    assert min(series) == round(0.2239 * 0.95, 6) and max(series) == round(0.2239 * 1.05, 6)

    def row(alpha, faces, vertices, op="coverage_at", region=1):
        return {"mesh": "m", "domain_id": "d", "op": op, "domain_call": region, "alpha": str(alpha), "faces": faces, "vertices": vertices}

    rows = [row(0.2, 8, 30), row(0.21, 8, 30), row(0.22, 8, 31), row(0.9, 9, 40), row(0.22, 5, 5, region=2)]
    inside = export.find_events(rows, "m", {0.2, 0.21, 0.22})
    assert [(item["from"], item["to"], item["vertices"]) for item in inside] == [("0.21", "0.22", [30, 31])]
    assert len(export.find_events(rows, "m", {0.2, 0.21, 0.22, 0.9})) == 2
    assert export.find_events(rows, "other", {0.2}) == []


def test_derived_records_starve_the_budget_of_a_real_call_and_replay_exactly(domain):
    derive = _load_tool("native_corpus_derive")
    recorder, _result = domain
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, _first(recorder, nc.OP_COVERAGE).call_blob, None, {}).args
    _cold_process()
    nc.coverage._coverage_at(partition, alpha, exact.exact_work_budget(stage="COVERAGE", domain_id="cold"), {})
    recorder.write_index()
    assert derive.spent_by(nc.read_record(recorder.root / recorder.rows[-1]["path"])) > 0
    derived = derive.derive_records(recorder.root, per_group=2, shares=(0.0, 0.5), min_spent=1, preset=1)
    assert derived and all(row["derived"]["share"] in (0.0, 0.5) for row in derived)
    raised = [row for row in derived if row["outcome"].startswith("raised:")]
    assert raised and all(row["exception"][0] == "ExactCanonicalizationWorkBudgetExhausted" for row in raised)
    index = nc.load_index(recorder.root)
    assert index["derived_count"] == len(derived) and index["records_count"] == len(index["records"]) - len(derived)
    for row in derived:
        record = nc.read_record(recorder.root / row["path"])
        original = next(item for item in index["records"] if item["id"] == row["derived"]["from"])
        assert record.before().budget["cap"] == row["derived"]["cap"] < nc.read_record(recorder.root / original["path"]).before().budget["cap"]
        before, replayed = _replay(record)
        assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []
    zero = next(row for row in raised if row["derived"]["share"] == 0.0)
    detail = nc.read_record(recorder.root / zero["path"]).expected().exception[1]
    assert detail.startswith(exact.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED) and f"cap={zero['derived']['cap']}" in detail
    assert len(derive.derive_records(recorder.root, per_group=2, shares=(0.0, 0.5), min_spent=1, preset=1)) == len(derived)
    assert nc.load_index(recorder.root)["derived_count"] == len(derived)


def test_a_corpus_of_another_kernel_is_never_substituted_for_the_corpus_of_this_one(tmp_path):
    """`matching_corpus`: новейший каталог, чей индекс записан под ЭТО ядро; корпус старого ядра не берётся, нет подходящего — `None` с названной причиной."""

    import json
    import os

    identity = nc.clip_memo.kernel_code_identity()

    def write(name: str, kernel: str, age: int) -> Path:
        directory = tmp_path / name
        directory.mkdir()
        (directory / "index.json").write_text(json.dumps({"kernel_identity": kernel, "records": []}), encoding="utf-8")
        os.utime(directory / "index.json", (1_000_000 + age, 1_000_000 + age))
        return directory

    write("old-kernel", "0000000000000000", 30)
    assert nc.matching_corpus(str(tmp_path)) is None
    reason = nc.describe_missing_corpus(str(tmp_path))
    assert identity in reason and "old-kernel=0000000000000000" in reason
    older = write("this-kernel-older", identity, 10)
    newer = write("this-kernel-newer", identity, 20)
    write("another-kernel-newest", "ffffffffffffffff", 40)
    assert nc.matching_corpus(str(tmp_path)) == newer and older != newer
    (tmp_path / "broken").mkdir()
    (tmp_path / "broken" / "index.json").write_text("{not json", encoding="utf-8")
    assert nc.matching_corpus(str(tmp_path)) == newer
    assert nc.matching_corpus(str(tmp_path / "absent")) is None
