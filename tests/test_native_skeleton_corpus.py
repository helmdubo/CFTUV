"""Корпус вызовов скелета (`build_skeleton`) нативного ускорителя: запись -> воспроизведение -> точное сравнение, швы и детерминизм.

Нативный порт скелета заменит `wavefront.skeleton.build_skeleton` целиком, и доказательство замены — побитовое равенство исходов на корпусе настоящих
вызовов: результат (`SkeletonV1`), исключение, шесть статей бюджета вместе со строкой `superlevel`, четыре таблицы памяти канонизации С ПОРЯДКОМ, счётчики знаков,
неоплаченное. Этот тест держит САМ инструмент сверки на чистом клоне, без корпуса и без Blender: малая настоящая подготовка ядра (фикстура
`building_002_point_contact_v1`: 12 вершин, два веера, четыре вогнутые вершины) проходит через рекордер на том месте, где её зовёт конвейер.

1. РЕКОРДЕР пишет вызов `build_skeleton` подготовки: полигон, режим, действующую границу уровней, бюджет `PREPARE` с идентичностью домена, состояние до и после.
2. ВОСПРОИЗВЕДЕНИЕ равно записи точно, не зависит от состояния процесса и от зерна хэша, и повторяется.
3. СРАВНЕНИЕ ловит подложенные расхождения в каждом поле исхода (дробь, `int` вместо `Fraction`, уровни, счётчики, обязательства, `superlevel`, статья, знаки, порядок памяти).
4. ВЫЗОВЫ: подменённая граница уровней, без бюджета, плотная гидратация, полный перебор, тёплая память записываются и воспроизводятся.
5. ПРОИЗВОДНЫЕ записи (потолок ниже цены вызова, отказ на выбранной трате) называют операцию исчерпания и воспроизводятся точно.
6. ШВЫ: вызовы слоя времён событий и закона кандидата, их журнал памяти и вида проверяются повторным вызовом эталона; подложенное расхождение ловится.
7. ГЕНЕРАТОР детерминирован по зерну, а его записи воспроизводятся; инструмент покрытия строк различает достижимое по именам и мёртвое семейство.
"""

from __future__ import annotations

import dataclasses
import json
import os
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "tools", ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402
import native_skeleton_coverage as coverage  # noqa: E402
import native_skeleton_derive as derive  # noqa: E402
import native_skeleton_generated as generated  # noqa: E402
import native_skeleton_seams as seams  # noqa: E402
import native_skeleton_synthetic as synthetic  # noqa: E402
import native_skeleton_verify as verify  # noqa: E402
import wavefront_cases  # noqa: E402

import cftuv_envelope as kernel  # noqa: E402
import cftuv_envelope.wavefront as wavefront  # noqa: E402
from cftuv_envelope import backend  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton  # noqa: E402
from cftuv_envelope import exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402

FIXTURE = ROOT / "kernel" / "fixtures" / "building_002_point_contact_v1"


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


def _recorder(tmp_path) -> "nc.Recorder":
    return nc.Recorder(tmp_path, {"python": sys.version.split()[0], "kernel_identity": nc.clip_memo.kernel_code_identity(), "git_head": "test"}, operations=nc.SKELETON_OPERATIONS)


@pytest.fixture
def preparation(tmp_path, monkeypatch):
    """Настоящая подготовка малого домена: `build_skeleton` зовёт конвейер, и вызов идёт через рекордер. `(рекордер, подготовка)`."""

    recorder = _recorder(tmp_path)
    monkeypatch.setattr(skeleton, "build_skeleton", recorder.wrap(nc.OP_SKELETON, nc.ORACLE[nc.OP_SKELETON]))
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((FIXTURE / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((FIXTURE / "decal_request.json").read_bytes())
    recorder.context.update(mesh="point_contact", mesh_digest="fixture")
    with backend.use_backend("PYTHON", "PYTHON"):
        prepared = wavefront.prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT"
    recorder.write_index()
    return recorder, prepared


def _record(recorder, index: int = 0):
    return nc.read_record(recorder.root / recorder.rows[index]["path"])


def _replay(record, *, audit: bool | None = None):
    before = record.before()
    state = before if audit is None else dataclasses.replace(before, canonical_audit=audit)
    return before, nc.execute(nc.prepare_call(record.op, record.call_blob, state))


def _fields(differences) -> list:
    return [item.field for item in differences]


# --------------------------------------------------------------------------
# 1. Рекордер
# --------------------------------------------------------------------------


def test_the_field_operations_are_unchanged_and_the_skeleton_is_a_separate_operation():
    assert nc.OPERATIONS == (nc.OP_COVERAGE, nc.OP_CLIP) and nc.SKELETON_OPERATIONS == (nc.OP_SKELETON,)
    assert nc.ORACLE[nc.OP_SKELETON] is skeleton.build_skeleton


def test_the_recorder_writes_the_skeleton_call_of_a_real_preparation(preparation):
    recorder, prepared = preparation
    assert len(recorder.rows) == 1
    row = recorder.rows[0]
    polygon = prepared.regions[0].bridge.polygon
    assert row["op"] == nc.OP_SKELETON and row["outcome"] == "EXACT" and row["budget"] is True and row["exception"] is None
    assert row["domain_id"] == prepared.work_budget.domain_id != ""
    assert (row["polygon_vertices"], row["polygon_loops"], row["fan_supports"], row["reflex"]) == (12, 1, 2, 4)
    assert row["level_budget"] == skeleton.level_budget(polygon) and row["split_search"] == "MOTORCYCLE" and row["dense_hydration"] is False
    assert row["nodes"] == len(prepared.regions[0].skeleton.nodes) > 0 and row["levels"] == prepared.regions[0].skeleton.levels
    index = nc.load_index(recorder.root)
    assert index["records_count"] == 1 and index["total_bytes"] == row["bytes"] == (recorder.root / row["path"]).stat().st_size


def test_the_record_holds_the_polygon_the_budget_by_identity_and_the_state_around_the_call(preparation):
    recorder, prepared = preparation
    record = _record(recorder)
    before, expected = record.before(), record.expected()
    assert set(record.payload) == {"before", "call", "seconds", "expected"}
    call = nc.decode_call(nc.OP_SKELETON, record.call_blob, nc.build_budget(before.budget), None)
    assert call.budget.spent_by_article() == before.budget["articles"] and call.budget.stage == "PREPARE" and call.budget.superlevel == ""
    assert call.args[0] == prepared.regions[0].bridge.polygon and call.kwargs["level_budget"] == skeleton.level_budget(call.args[0])
    assert expected.result == prepared.regions[0].skeleton and expected.exception is None
    assert expected.after.budget["superlevel"] == str(expected.result.levels) == prepared.work_budget.superlevel
    assert expected.after.budget["articles"] == prepared.work_budget.spent_by_article()
    assert before.known_primes == [] and before.identity_mode == "CACHED" and before.canonical_audit is False


def test_the_cold_start_of_the_product_leaves_the_memory_empty_and_the_call_fills_it(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    before, expected = record.before(), record.expected()
    assert before.factorization == [] and before.squarefree == [] and before.prime_support == []
    assert expected.after.factorization and expected.after.known_primes and expected.after.sign_counts["total"] > before.sign_counts["total"]
    assert before.unbudgeted == (0,) * 6 and expected.after.unbudgeted == (0,) * 6


# --------------------------------------------------------------------------
# 2. Воспроизведение
# --------------------------------------------------------------------------


def test_a_replay_reproduces_the_recorded_outcome_exactly_and_twice(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    expected = record.expected()
    digests = set()
    for _ in range(2):
        before, replayed = _replay(record)
        assert nc.compare_outcomes(record.op, before, expected, replayed) == []
        assert replayed.after == expected.after and replayed.exception is None
        digests.add(nc.outcome_digest(record.op, before, replayed))
    assert digests == {nc.outcome_digest(record.op, record.before(), expected)} and len(digests) == 1
    assert record.payload["expected"]["answer_digest"] == nc.answer_digest(record.op, expected.result)


def test_a_replay_does_not_depend_on_the_state_the_process_is_in(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    before, clean = _replay(record)
    exact._FACTORIZATION_MEMO[10**40 + 7] = ((10**40 + 7, 1),)
    exact._KNOWN_PRIMES.insert(0, 3)
    exact._KNOWN_PRIME_SET.add(3)
    exact.SIGN_COUNTS["total"] += 12345
    exact.UNBUDGETED_WORK.gcd_operations += 77
    exact.set_canonical_audit(True)
    _dirty_before, dirty = _replay(record)
    assert nc.compare_outcomes(record.op, before, clean, dirty) == []
    assert nc.compare_outcomes(record.op, before, record.expected(), dirty) == []


def test_the_canonical_audit_does_not_change_the_outcome(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    before, off = _replay(record, audit=False)
    _before, on = _replay(record, audit=True)
    assert nc.compare_outcomes(record.op, before, off, on) == []


def test_the_replay_tool_proves_the_outcome_is_the_same_under_other_hash_seeds(preparation, tmp_path):
    recorder, _prepared = preparation
    outputs = []
    for seed in ("1", "424242"):
        out = tmp_path / f"digests_{seed}.json"
        command = [sys.executable, str(ROOT / "tools" / "native_skeleton_verify.py"), "replay", "--corpus", str(recorder.root), "--repeat", "2", "--out", str(out)]
        done = subprocess.run(command, env={**os.environ, "PYTHONSAFEPATH": "1", "PYTHONHASHSEED": seed}, capture_output=True, text=True, cwd=str(ROOT), check=False)
        assert done.returncode == 0 and "NATIVE_SKELETON_REPLAY_OK" in done.stdout, done.stdout + done.stderr
        outputs.append(out)
    report = verify.compare_files(outputs)
    assert report["differing"] == [] and report["only_in_some"] == [] and report["records_common"] == 1
    assert [item.split("seed=")[1] for item in report["runs"]] == ["1", "424242"]


def test_the_replay_comparison_names_a_planted_difference_between_two_runs(preparation, tmp_path):
    recorder, _prepared = preparation
    row = recorder.rows[0]
    first = verify.replay_one(recorder.root, row, 2, "keep")
    assert first["equal"] and first["stable"] and len(first["digests"]) == 2 and first["digests"][0] == first["expected_digest"]
    runs = []
    for number, digest in enumerate((first["digests"][0], first["digests"][0][:-1] + "0" if first["digests"][0][-1] != "0" else first["digests"][0][:-1] + "1")):
        item = dict(first, digests=[digest])
        path = tmp_path / f"run{number}.json"
        path.write_text(json.dumps({"summary": {"python": "x", "hash_seed": str(number)}, "records": [item]}), encoding="utf-8")
        runs.append(path)
    assert [item["id"] for item in verify.compare_files(runs)["differing"]] == [row["id"]]


# --------------------------------------------------------------------------
# 3. Сравнение ловит подложенное расхождение
# --------------------------------------------------------------------------


def _pair(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    before, replayed = _replay(record)
    return record.op, before, record.expected(), replayed


def _with_result(base, **changes):
    return dataclasses.replace(base, result=dataclasses.replace(base.result, **changes))


def test_a_changed_fraction_inside_a_node_time_is_detected_and_located(preparation):
    op, before, expected, actual = _pair(preparation)
    node = actual.result.nodes[0]
    bent = dataclasses.replace(node, time=dataclasses.replace(node.time, dividend=node.time.dividend + Fraction(1, 10**9)))
    found = nc.compare_outcomes(op, before, expected, _with_result(actual, nodes=(bent, *actual.result.nodes[1:])))
    assert _fields(found) == ["result.nodes"] and "[0]" in found[0].detail


def test_an_int_in_place_of_a_fraction_inside_a_node_point_is_detected_although_python_calls_them_equal(preparation):
    op, before, expected, actual = _pair(preparation)
    node = actual.result.nodes[0]
    as_fraction, as_int = SqrtSumV1(((1, Fraction(3)),)), SqrtSumV1(((1, 3),))
    assert as_fraction == as_int
    left = _with_result(expected, nodes=(dataclasses.replace(node, point=dataclasses.replace(node.point, x=as_fraction)), *expected.result.nodes[1:]))
    right = _with_result(actual, nodes=(dataclasses.replace(node, point=dataclasses.replace(node.point, x=as_int)), *actual.result.nodes[1:]))
    assert _fields(nc.compare_outcomes(op, before, left, right)) == ["result.nodes"]


def test_levels_counters_obligations_and_outcome_differences_are_detected(preparation):
    op, before, expected, actual = _pair(preparation)
    assert _fields(nc.compare_outcomes(op, before, expected, _with_result(actual, levels=actual.result.levels + 1))) == ["result.levels"]
    name, value = actual.result.counters[0]
    counters = ((name, value + 1), *actual.result.counters[1:])
    assert _fields(nc.compare_outcomes(op, before, expected, _with_result(actual, counters=counters))) == ["result.counters"]
    assert actual.result.proof_obligations
    assert _fields(nc.compare_outcomes(op, before, expected, _with_result(actual, proof_obligations=actual.result.proof_obligations[1:]))) == ["result.proof_obligations"]
    other = skeleton.SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
    assert _fields(nc.compare_outcomes(op, before, expected, _with_result(actual, outcome=other))) == ["result.outcome"]


def test_the_superlevel_string_and_a_budget_article_off_by_one_are_detected_and_named(preparation):
    op, before, expected, actual = _pair(preparation)
    assert expected.after.budget["superlevel"] != ""
    bent = dataclasses.replace(actual.after, budget={**actual.after.budget, "superlevel": str(int(actual.after.budget["superlevel"]) + 1)})
    assert _fields(nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=bent))) == ["budget.superlevel"]
    articles = list(actual.after.budget["articles"])
    articles[5] += 1
    after = dataclasses.replace(actual.after, budget={**actual.after.budget, "articles": tuple(articles)})
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=after))
    assert _fields(found) == ["budget.delta"] and "exact_position_hydrations" in found[0].detail
    stage = dataclasses.replace(actual.after, budget={**actual.after.budget, "stage": "COVERAGE"})
    assert _fields(nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=stage))) == ["budget.stage"]


def test_sign_counts_memory_order_and_exception_differences_are_detected(preparation):
    op, before, expected, actual = _pair(preparation)
    signs = {**actual.after.sign_counts, "closed_by_enclosure": actual.after.sign_counts["closed_by_enclosure"] + 1}
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=dataclasses.replace(actual.after, sign_counts=signs)))
    assert _fields(found) == ["sign_counts.closed_by_enclosure"]
    assert len(expected.after.factorization) >= 2
    swapped = dataclasses.replace(actual.after, factorization=[actual.after.factorization[1], actual.after.factorization[0], *actual.after.factorization[2:]])
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=swapped))
    assert _fields(found) == ["memory.factorization"] and "order" in found[0].detail
    primes = dataclasses.replace(actual.after, known_primes=[*actual.after.known_primes, 1000003])
    assert _fields(nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=primes))) == ["memory.known_primes"]
    unbudgeted = (actual.after.unbudgeted[0] + 1, *actual.after.unbudgeted[1:])
    assert _fields(nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, after=dataclasses.replace(actual.after, unbudgeted=unbudgeted)))) == ["unbudgeted.delta"]
    found = nc.compare_outcomes(op, before, expected, dataclasses.replace(actual, exception=("AssertionError", "boom")))
    assert _fields(found) == ["exception"]


# --------------------------------------------------------------------------
# 4. Условия вызова
# --------------------------------------------------------------------------


def _record_polygon(tmp_path, polygon, **options):
    recorder = _recorder(tmp_path)
    wrapped = recorder.wrap(nc.OP_SKELETON, nc.ORACLE[nc.OP_SKELETON])
    recorder.context.update(mesh="direct")
    try:
        wrapped(polygon, **options)
    except Exception:  # noqa: BLE001 - исключение записывается, и тест читает запись
        pass
    recorder.write_index()
    return recorder, _record(recorder, -1)


def test_a_patched_level_budget_is_recorded_as_an_input_and_replays_with_the_clean_module(tmp_path, monkeypatch):
    polygon = wavefront_cases.ell(12)
    monkeypatch.setattr(skeleton, "level_budget", lambda _polygon: 1)
    recorder, record = _record_polygon(tmp_path, polygon, work_budget=exact.exact_work_budget(stage="PREPARE", domain_id="d"))
    monkeypatch.undo()
    assert recorder.rows[-1]["level_budget"] == 1 and recorder.rows[-1]["outcome"] == "LEVEL_BUDGET_EXHAUSTED"
    assert skeleton.level_budget(polygon) > 1
    before, replayed = _replay(record)
    assert replayed.result.outcome is skeleton.SkeletonOutcome.LEVEL_BUDGET_EXHAUSTED and replayed.result.levels == 1
    assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []
    assert skeleton.level_budget(polygon) > 1, "the pin is taken off after the call"


def test_a_call_without_a_budget_records_the_unbudgeted_work_and_replays_it(tmp_path):
    recorder, record = _record_polygon(tmp_path, wavefront_cases.ell(12))
    assert recorder.rows[-1]["budget"] is False and recorder.rows[-1]["domain_id"] is None
    before, replayed = _replay(record)
    assert before.budget is None and replayed.after.budget is None and any(replayed.after.unbudgeted)
    assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []


def test_dense_hydration_exhaustive_search_and_a_warm_memory_are_recorded_and_replayed(tmp_path):
    polygon = wavefront_cases.cross(wide=6, tall=4)
    budget = exact.exact_work_budget(stage="PREPARE", domain_id="warm")
    nc.ORACLE[nc.OP_SKELETON](wavefront_cases.ell(12), work_budget=budget)  # leaves the canonicalization memory warm
    assert exact._FACTORIZATION_MEMO
    for number, options in enumerate(({"dense_hydration": True}, {"split_search": skeleton.SplitSearch.EXHAUSTIVE}, {})):
        recorder, record = _record_polygon(tmp_path / str(number), polygon, work_budget=exact.exact_work_budget(stage="PREPARE", domain_id=f"d{number}"), **options)
        row = recorder.rows[-1]
        assert row["dense_hydration"] is bool(options.get("dense_hydration")) and row["split_search"] == options.get("split_search", skeleton.SplitSearch.MOTORCYCLE).value
        before, replayed = _replay(record)
        assert before.factorization, "the recorded state before the call is the warm memory"
        assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []


def test_a_call_that_raises_inside_the_oracle_records_the_exception_and_the_partial_state(tmp_path):
    polygon = wavefront_cases.star(9, 2)
    free = exact.exact_work_budget(stage="PREPARE", domain_id="free")
    nc.ORACLE[nc.OP_SKELETON](polygon, work_budget=free)
    spent = free.spent
    assert spent > 40
    _cold_process()
    budget = exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=spent // 2)
    recorder, record = _record_polygon(tmp_path, polygon, work_budget=budget)
    row = recorder.rows[-1]
    assert row["outcome"] == "raised:ExactCanonicalizationWorkBudgetExhausted" and row["exception"][0] == "ExactCanonicalizationWorkBudgetExhausted"
    assert row["exception"][1].startswith(exact.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED) and "domain=starved" in row["exception"][1]
    before, replayed = _replay(record)
    assert replayed.exception == tuple(row["exception"]) and replayed.result is None
    assert replayed.after.budget["articles"] != before.budget["articles"] and sum(replayed.after.budget["articles"]) > spent // 2
    assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []


# --------------------------------------------------------------------------
# 5. Производные записи
# --------------------------------------------------------------------------


@pytest.mark.parametrize("index_schema", ["cftuv.native-corpus.v1", sc.SYNTHETIC_INDEX_SCHEMA])
def test_derived_records_starve_the_budget_and_name_the_operation_that_ran_out(preparation, index_schema):
    recorder, _prepared = preparation
    old_index = nc.load_index(recorder.root)
    old_index["schema"] = index_schema
    for row in old_index["records"]:
        row["schema"] = "cftuv.native-corpus.v1"
    (recorder.root / "index.json").write_text(json.dumps(old_index), encoding="utf-8")
    derived = derive.derive_records(recorder.root, per_mesh=1, shares=(0.05, 0.6), min_spent=1, max_seconds=60.0, occurrences=2, preset=1)
    assert derived and all(row["outcome"] == "raised:ExactCanonicalizationWorkBudgetExhausted" for row in derived)
    operations = derive.exhaustion_operations(derived)
    assert len(operations) >= 3 and set(operations) <= {item.value for item in exact.ExactWorkOperationV1}
    boundary = [row for row in derived if row["derived"].get("boundary")]
    exact_trigger = [row for row in derived if row["derived"].get("target") and not row["derived"]["boundary"]]
    assert boundary and exact_trigger
    index = nc.load_index(recorder.root)
    assert index["derived_count"] == len(derived) and index["records_count"] == 1
    assert index["schema"] == (sc.SYNTHETIC_INDEX_SCHEMA if index_schema == sc.SYNTHETIC_INDEX_SCHEMA else nc.RECORD_SCHEMA)
    base = nc.read_record(recorder.root / recorder.rows[0]["path"])
    for row in derived:
        assert row["schema"] == nc.read_meta(recorder.root / row["path"])["schema"] == nc.RECORD_SCHEMA
        record = nc.read_record(recorder.root / row["path"])
        assert record.before().budget["cap"] == row["derived"]["cap"] < sum(base.expected().after.budget["articles"])
        before, replayed = _replay(record)
        assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []
        assert replayed.exception == tuple(row["exception"])
        if row["derived"].get("target") and not row["derived"]["boundary"]:
            assert f"operation={row['derived']['target']}" in row["exception"][1]
    assert len(derive.derive_records(recorder.root, per_mesh=1, shares=(0.05, 0.6), min_spent=1, max_seconds=60.0, occurrences=2, preset=1)) == len(derived)


def test_the_spend_trace_is_the_order_of_the_budget_spends_of_the_recorded_call(preparation):
    recorder, _prepared = preparation
    record = _record(recorder)
    trace = derive.spend_trace(record)
    assert trace and [spent for _operation, spent in trace] == sorted(spent for _operation, spent in trace)
    assert trace[-1][1] <= sum(record.expected().after.budget["articles"]) and {name for name, _spent in trace} <= {item.value for item in exact.ExactWorkOperationV1}
    caps = derive.targeted_caps(trace, 2)
    assert {tag["target"] for _cap, tag in caps} == {name for name, _spent in trace}
    assert all(not tag["boundary"] or any(cap == spent for _name, spent in trace) for cap, tag in caps)


def test_deriving_from_an_unlimited_reference_builds_a_bounded_replay_and_preserves_unlisted_files(tmp_path):
    budget = exact.ExactWorkBudgetV1(mode=exact.ExactWorkBudgetModeV1.UNLIMITED_REFERENCE, cap=None, stage="PREPARE", domain_id="reference")
    recorder, base = _record_polygon(tmp_path, wavefront_cases.ell(12), work_budget=budget)
    sentinel = recorder.root / "records" / "_derived" / "unlisted.txt"
    sentinel.parent.mkdir(parents=True, exist_ok=True)
    sentinel.write_text("preserve", encoding="utf-8")
    rows = derive.derive_records(recorder.root, per_mesh=1, shares=(0.5,), min_spent=1, max_seconds=60, occurrences=1, preset=1)
    assert rows and sentinel.read_text(encoding="utf-8") == "preserve"
    assert base.before().budget["mode"] == exact.ExactWorkBudgetModeV1.UNLIMITED_REFERENCE.value
    for row in rows:
        record = sc.read(recorder.root, row)
        assert record.before().budget["mode"] == exact.ExactWorkBudgetModeV1.BOUNDED.value
        before, replayed = _replay(record)
        assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == []


# --------------------------------------------------------------------------
# 6. Швы
# --------------------------------------------------------------------------


@pytest.fixture
def seam_run(preparation):
    recorder, _prepared = preparation
    index = seams.record_corpus(recorder.root, full=frozenset(seams.SEAM_NAMES), full_ids=(recorder.rows[0]["id"],))
    return recorder, index


def test_the_seams_of_a_real_preparation_are_recorded_for_every_layer_and_replay_exactly(seam_run):
    recorder, index = seam_run
    assert set(index["recorded"]) >= {"COMPARE_TIMES", "CONCURRENCY_TIME", "EVENT_POINT_UNIVERSE", "TIME_NORMALIZED", "EVALUATE_EDGE", "EVALUATE_SPLIT", "TIMES_ARE_EQUAL"}
    assert index["recorded"] == {name: count for name, count in index["seen"].items()}, "full mode records every call"
    assert index["queue_ops"] > 0 and index["bytes"] > 0
    report = seams.verify_corpus(recorder.root)
    queue_checked = report["checked"].pop(seams.QUEUE_SEAM)
    assert report["problem_count"] == 0 and report["checked"] == index["recorded"] and queue_checked == index["queue_ops"]


def test_the_sampled_seams_are_a_head_and_the_calls_that_pay_or_change_memory(preparation):
    recorder, _prepared = preparation
    index = seams.record_corpus(recorder.root)
    full = seams.record_corpus(recorder.root, full=frozenset(seams.SEAM_NAMES), full_ids=(recorder.rows[0]["id"],))
    assert index["seen"] == full["seen"], "wrapping must not change what the oracle calls"
    for name, seen in index["seen"].items():
        assert index["recorded"].get(name, 0) <= seen and full["recorded"][name] == seen
    assert seams.verify_corpus(recorder.root)["problem_count"] == 0


def test_the_seam_reader_gives_calls_cost_and_the_memory_of_every_event(seam_run):
    recorder, _index = seam_run
    found = list(seams.iter_files(recorder.root))
    assert len(found) == 1
    row, seam_file = found[0]
    assert row["record"] == seam_file.record == recorder.rows[0]["id"] and seam_file.python == sys.version.split()[0]
    payers = [call for call in seam_file.calls if any(call.articles)]
    assert payers and all(len(call.sign) == len(seams.SIGN_KEYS) and len(call.articles) == 6 for call in seam_file.calls)
    last = seam_file.memory(len(seam_file.journal) - 1)
    record = _record(recorder)
    assert [key for key, _ in last["known_primes"]] == record.expected().after.known_primes
    assert last["factorization"] == record.expected().after.factorization and last["squarefree"] == record.expected().after.squarefree
    base = seam_file.memory(0)
    assert base["factorization"] == [] and seam_file.state_before(seam_file.calls[0]).budget["stage"] == "PREPARE"
    splits = list(seams.iter_calls(recorder.root, "EVALUATE_SPLIT"))
    assert splits and all(call.extra["view_log"] and "memo" in call.extra for _file, call in splits)


def test_the_journal_of_a_record_starts_from_its_recorded_state_not_from_what_the_previous_record_left():
    """Журнал создаётся ДО того, как воспроизведение ставит процесс в записанное состояние, и в процессе тогда лежит память прошлого прогона: отпечаток начала берётся с базы журнала.

    Иначе запись, чья память по ходу прогона становится такой же, какой кончил прошлая, не видела бы собственного заполнения: события не было, и база (пустая) выдавалась за состояние
    перед первым вызовом листа (так были записаны первые швы синтетики: повторный вызов эталона тратил единицу, которой в записи не было)."""

    exact._KNOWN_PRIMES.extend((2, 3, 5))
    exact._KNOWN_PRIME_SET.update((2, 3, 5))
    exact._FACTORIZATION_MEMO[30] = ((2, 1), (3, 1), (5, 1))
    exact._SQUAREFREE_MEMO[12] = (3, 2)
    assert seams.fingerprint() == seams.fingerprint_of(seams.live_tables())
    journal = seams.Journal({"known_primes": [], "factorization": [], "squarefree": [], "prime_support": []})
    assert journal.observe() == 1, "the live memory differs from the recorded (empty) base: an event is made"
    assert seams.apply_diff([], journal.events[1]["factorization"]) == [(30, ((2, 1), (3, 1), (5, 1)))]
    assert journal.observe() == 1, "and nothing changes while the memory stays"
    exact.reset_factorization_memory()
    assert journal.observe() == 2 and seams.apply_diff([(2, None), (3, None), (5, None)], journal.events[2]["known_primes"]) == []


def test_the_seam_memory_journal_is_a_chain_of_exact_differences():
    old = [(2, None), (3, None), (5, None)]
    assert seams.diff_items(old, old + [(7, None)]) == ("tail", 0, [(7, None)])
    assert seams.apply_diff(old, ("tail", 0, [(7, None)])) == old + [(7, None)]
    table = [(1, "a"), (2, "b"), (3, "c"), (4, "d")]
    touched = [(1, "a"), (3, "c"), (4, "d"), (2, "b")]
    diff = seams.diff_items(table, touched)
    assert diff == ("tail", 0, [(2, "b")]) and seams.apply_diff(table, diff) == touched
    evicted = [(3, "c"), (4, "d"), (5, "e")]
    diff = seams.diff_items(table, evicted)
    assert diff[0] == "tail" and seams.apply_diff(table, diff) == evicted
    different = [(9, "z"), (1, "q")]
    assert seams.diff_items(table, different)[0] == "full" or seams.apply_diff(table, seams.diff_items(table, different)) == different


def test_a_planted_difference_in_a_seam_call_is_named_by_the_verification(seam_run):
    recorder, _index = seam_run
    _row, seam_file = next(iter(seams.iter_files(recorder.root)))
    compared = next(call for call in seam_file.calls if call.seam == "COMPARE_TIMES")
    assert seams.verify_call(seam_file, compared) == []
    wrong = dataclasses.replace(compared, result=-compared.result if compared.result else 1)
    assert any(item.startswith("result") for item in seams.verify_call(seam_file, wrong))
    wrong = dataclasses.replace(compared, sign=(compared.sign[0] + 1, *compared.sign[1:]))
    assert any(item.startswith("sign_counts") for item in seams.verify_call(seam_file, wrong))
    paying = next(call for call in seam_file.calls if any(call.articles) and call.seam not in seams.CANDIDATE_SEAMS)
    assert seams.verify_call(seam_file, paying) == []
    wrong = dataclasses.replace(paying, articles=(paying.articles[0] + 1, *paying.articles[1:]))
    assert any(item.startswith("budget") for item in seams.verify_call(seam_file, wrong))
    candidate = next(call for call in seam_file.calls if call.seam == "EVALUATE_SPLIT")
    assert seams.verify_call(seam_file, candidate) == []
    log = list(candidate.extra["view_log"])
    log[0] = (log[0][0], log[0][1], ("raised", "RuntimeError", "planted"), log[0][3])
    broken = dataclasses.replace(candidate, extra={**candidate.extra, "view_log": log})
    assert any("candidate call" in item or item.startswith("exception") for item in seams.verify_call(seam_file, broken))


# --------------------------------------------------------------------------
# 7. Генератор и покрытие строк
# --------------------------------------------------------------------------


@pytest.mark.parametrize("family", sorted(generated.FAMILIES))
def test_every_generator_family_is_deterministic_in_its_seed(family):
    built = [generated.make(family, seed) for seed in range(12)]
    assert built == [generated.make(family, seed) for seed in range(12)]
    assert any(item is not None for item in built), f"{family} produces no valid polygon in twelve seeds"


def test_generated_records_replay_exactly_and_carry_the_variant_that_made_them(tmp_path):
    recorder = _recorder(tmp_path)
    chosen = (("histogram", 0), ("polyomino_weighted", 3), ("cross", 5), ("skew", 6), ("star", 7))
    outcomes = generated.generate(recorder, chosen)
    assert sum(outcomes.values()) == len(recorder.rows) >= len(chosen)
    variants = {row["variant"] for row in recorder.rows}
    assert {"fresh", "none", "dense", "warm", "level", "exhaustive", "saturated"} <= variants, variants
    recorder.write_index()
    for row in recorder.rows:
        record = nc.read_record(recorder.root / row["path"])
        before, replayed = _replay(record)
        assert nc.compare_outcomes(record.op, before, record.expected(), replayed) == [], row["id"]
        assert row["family"] in generated.FAMILIES and row["mesh"] == f"generated_{row['family']}"
    saturated = nc.read_record(recorder.root / next(row for row in recorder.rows if row["variant"] == "saturated")["path"])
    assert len(saturated.before().factorization) == exact._FACTORIZATION_MEMO_ENTRIES - 1 and len(saturated.before().known_primes) == exact._KNOWN_PRIME_REGISTRY_ENTRIES - 1
    assert len(saturated.expected().after.factorization) == exact._FACTORIZATION_MEMO_ENTRIES, "the first misses evict from a table at capacity"
    pinned = [row for row in recorder.rows if row["variant"] == "level"]
    assert all(row["level_budget"] <= skeleton.level_budget(nc.decode_call(nc.OP_SKELETON, nc.read_record(recorder.root / row["path"]).call_blob, None, None).args[0]) for row in pinned)


def test_the_variants_of_a_seed_are_a_function_of_the_seed():
    assert generated.variants_for(1) == ("fresh",) and generated.variants_for(0)[0] == "fresh"
    assert set(generated.variants_for(0)) == {"fresh", "none", "dense", "warm", "level", "exhaustive", "saturated"}
    assert generated.variants_for(12) == ("fresh", "none", "dense")


def test_the_reachability_closure_keeps_the_live_path_and_drops_the_legacy_family():
    live = coverage.reachable_functions()
    assert ("wavefront/skeleton.py", "_Builder.run") in live and ("wavefront/superlevel.py", "apply_superlevel_transaction") in live
    assert ("wavefront/symbolic_runtime_commit.py", "plan_symbolic_runtime_commit") in live
    assert ("wavefront/skeleton.py", "_Builder._apply_multi_split") not in live and ("wavefront/skeleton.py", "_Builder._group_splits") not in live


def test_the_line_coverage_tracer_sees_the_lines_of_a_call_and_names_the_uncovered_ones():
    tracer = coverage.Tracer()
    tracer.start()
    try:
        nc.ORACLE[nc.OP_SKELETON](wavefront_cases.axis_square(8), work_budget=exact.exact_work_budget(stage="PREPARE", domain_id="cov"))
    finally:
        tracer.stop()
    assert tracer.hits["wavefront/skeleton.py"] and tracer.hits["wavefront/event_time.py"]
    report = coverage.uncovered(dict(tracer.hits))
    by_name = {name: (count, missing) for name, _line, count, missing, live in report["wavefront/skeleton.py"] if live}
    count, missing = by_name["build_skeleton"]
    assert count > 0 and missing == []
    count, missing = by_name["_Builder._seed_fan_edges"]
    assert count > 0 and len(missing) >= count // 2, "a polygon without fans never reaches the body of the fan seeding"
    summary = coverage.summary(report)
    assert 0 < summary["covered_percent"] < 100 and summary["never_called_functions"] > 0


def test_the_skeleton_corpus_of_another_kernel_is_never_substituted_for_the_corpus_of_this_one(tmp_path):
    identity = nc.clip_memo.kernel_code_identity()

    def write(name: str, kernel_identity: str, age: int, kind: str = sc.FIELD_DIR, schema=nc.RECORD_SCHEMA) -> Path:
        directory = tmp_path / name / kind
        directory.mkdir(parents=True)
        (directory / "index.json").write_text(json.dumps({"schema": schema, "kernel_identity": kernel_identity, "records": []}), encoding="utf-8")
        os.utime(directory / "index.json", (1_000_000 + age, 1_000_000 + age))
        return directory

    write("old", "0000000000000000", 30)
    assert sc.matching("field", tmp_path) is None
    reason = sc.describe_missing("field", tmp_path)
    assert identity in reason and "old=0000000000000000" in reason
    older = write("this-older", identity, 10, schema="cftuv.native-corpus.v1")
    assert sc.matching("field", tmp_path) == older
    newer = write("this-newer", identity, 20)
    write("other-newest", "ffffffffffffffff", 40)
    write("this-synthetic", identity, 25, sc.SYNTHETIC_DIR, sc.SYNTHETIC_INDEX_SCHEMA)
    for kind in (sc.FIELD_DIR, sc.SYNTHETIC_DIR):
        write("unknown-newest", identity, 50, kind, "cftuv.native-corpus.v999")
        write("missing-schema", identity, 60, kind, None)
    assert "schema='cftuv.native-corpus.v999'" in sc.describe_missing("field", tmp_path)
    assert sc.matching("field", tmp_path) == newer and sc.matching("synthetic", tmp_path).parent.name == "this-synthetic"
    assert sc.matching("field", tmp_path / "absent") is None


def test_the_synthetic_writer_makes_the_memory_cold_drops_repeats_and_flags_what_the_test_saw(tmp_path):
    polygon, other = wavefront_cases.ell(12), wavefront_cases.cross(wide=6, tall=4)
    items = []
    for shape in (polygon, polygon, other):
        _cold_process()
        nc.ORACLE[nc.OP_SKELETON](wavefront_cases.staircase(), work_budget=exact.exact_work_budget(stage="PREPARE", domain_id="warm-up"))
        budget = exact.exact_work_budget(stage="PREPARE", domain_id="d")
        call = nc.unpack_call(nc.OP_SKELETON, (shape,), {"work_budget": budget})
        warm = nc.capture_state(budget, None)
        assert warm.factorization or warm.known_primes, "the test process leaves a warm memory behind"
        result = nc.ORACLE[nc.OP_SKELETON](shape, work_budget=budget)
        items.append({"test": "kernel/tests/test_some_group.py::test_case", "before": synthetic.cold_state(warm), "blob": nc.encode_call(call), "live": synthetic._live_view(result, None)})
    items.append({**items[2], "test": "kernel/tests/test_other_group.py::test_case", "live": ("result", "planted")})
    description = {"python": sys.version.split()[0], "kernel_identity": nc.clip_memo.kernel_code_identity(), "git_head": "test"}
    output = tmp_path / "actual-writer" / sc.SYNTHETIC_DIR
    document = synthetic.write_items(output, items, description, {})
    assert document["schema"] == synthetic.INDEX_SCHEMA == sc.SYNTHETIC_INDEX_SCHEMA
    assert sc.matching("synthetic", tmp_path) == output
    assert document["records_count"] == 2 and document["test_duplicates_after_normalization"] == 2, "the repeated polygon and its twin under another test are one record"
    rows = document["records"]
    assert [row["mesh"] for row in rows] == ["some_group", "some_group"] and [row["live_equal"] for row in rows] == [True, True]
    for row in rows:
        record = nc.read_record(output / row["path"])
        assert row["schema"] == record.meta["schema"] == nc.RECORD_SCHEMA
        before = record.before()
        assert before.factorization == before.squarefree == before.prime_support == before.known_primes == [] and before.unbudgeted == (0,) * 6
        assert set(before.sign_counts.values()) == {0} and before.canonical_audit is False
        assert nc.compare_outcomes(record.op, before, record.expected(), _replay(record)[1]) == []
    flagged = synthetic.write_items(tmp_path / "flagged", [items[3]], description, {})
    assert flagged["records"][0]["live_equal"] is False and flagged["test_live_differs"] == 1
    again = synthetic.rewrite(output, tmp_path / "again")
    assert [row["outcome"] for row in again["records"]] == [row["outcome"] for row in rows] and again["records_count"] == 2


def test_the_ci_subset_of_the_kernel_tests_is_a_part_of_the_full_list_and_every_file_exists():
    """`--ci` (the time-bounded corpus the native CI builds, `.github/workflows/native.yml`) names real files of the full list, each once."""

    assert len(set(synthetic.CI_FILES)) == len(synthetic.CI_FILES) and set(synthetic.CI_FILES) < set(synthetic.TEST_FILES)
    assert all((synthetic.ROOT / "kernel" / "tests" / name).is_file() for name in synthetic.CI_FILES)
    assert len(synthetic.CI_FILES) >= 15, "a subset that reaches the branches the generator's frozen cases do not"
