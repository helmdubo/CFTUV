"""Нативный `build_skeleton` как ВСТАВКА (`cftuv_native.build_skeleton`) равен эталону на Python: тот же вызов, те же побочные эффекты, те же исключения.

Вставка — граница, какой её увидит сеанс Python: сигнатура `skeleton.build_skeleton(polygon, *, split_search, work_budget, dense_hydration)`, результат — настоящий `SkeletonV1`,
а всё остальное — настоящие объекты процесса: шесть статей `ExactWorkBudgetV1` и строка `superlevel`, `SIGN_COUNTS`, `UNBUDGETED_WORK`, четыре таблицы памяти канонизации (порядок
вставки входит), настоящие исключения с текстом эталона (в том числе деталь исчерпания). Оба пути стартуют с одного восстановленного состояния ДО (`native_corpus.prepare_call`);
сравнение — `nc.compare_outcomes` (результат каноническим кодом: `int` и `Fraction` различны; исключение `(класс, текст)`; цена; память с порядком; знаки; неоплаченное).

Источники: полевые записи (163 холодные подготовки пяти мешей и производные записи с урезанным потолком), синтетические (тесты ядра и генераторы, с производными), живой эталон на
выборке, цепочки вызовов на ОДНОМ сеансе с тёплой памятью, подменённая граница уровней, вызов без бюджета, плотная гидратация. Один `mirror` живёт между вызовами (привязанные классы,
снимки таблиц памяти), как в сеансе.

Отказ ПОРТА (`cftuv_native.NATIVE_REFUSALS`) — второй вид исхода, не расхождение: он обязан оставить ВСЁ состояние как до вызова, после чего ядро на тех же бюджете и таблицах даёт записанный
исход. Допустимые отказы названы здесь: внутренний отказ самого эталона (`TypeError` символьной ссылки без концов и несравнимых ключей, 30 синтетических записей). Поиск `EXHAUSTIVE` (эталон тестов) порт несёт.

Модуль пропускается с названной причиной, пока расширение не собрано, нативный `skeleton` не сверен с этим деревом ядра (`native_gate`) или корпуса нет. Переменные окружения:
`CFTUV_SKELETON_SYNTHETIC_STRIDE` (каждая N-я синтетическая запись; по умолчанию все), `CFTUV_SKELETON_LIVE_LIMIT` (записей живого эталона; по умолчанию 40).
"""

from __future__ import annotations

import dataclasses
import inspect
import os
import subprocess
import sys
import threading
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка вставки build_skeleton с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "skeleton")

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402
import native_skeleton_whole as whole  # noqa: E402
import wavefront_cases  # noqa: E402

import cftuv_envelope as kernel  # noqa: E402
import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront as wavefront  # noqa: E402
import cftuv_envelope.wavefront.conveyor as conveyor  # noqa: E402
import cftuv_envelope.wavefront.motorcycle as motorcycle  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton  # noqa: E402

FIELD = sc.matching("field")
SYNTHETIC = sc.matching("synthetic")
SYNTHETIC_STRIDE = int(os.environ.get("CFTUV_SKELETON_SYNTHETIC_STRIDE", "1"))
LIVE_LIMIT = int(os.environ.get("CFTUV_SKELETON_LIVE_LIMIT", "40"))
needs_field = pytest.mark.skipif(FIELD is None, reason=sc.describe_missing("field"))
needs_synthetic = pytest.mark.skipif(SYNTHETIC is None, reason=sc.describe_missing("synthetic"))

CHECKED: Counter = Counter()


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон и вставка пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    audit = exact.canonical_audit_enabled()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)
    exact.set_canonical_audit(audit)


@pytest.fixture(scope="module")
def runner():
    """Один сеанс на весь модуль: привязанные классы и снимки таблиц переживают вызовы."""

    return whole.WholeRunner()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(
            f"native build_skeleton DROP-IN compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items()))
        )


def _rows(root, *, derived):
    return sc.rows_of(root, derived=derived)


def run_rows(runner, root, rows, source: str, *, live: bool = False) -> list:
    runs = []
    for row in rows:
        run = runner.compare(sc.read(root, row), live=live)
        runs.append((row["id"], run))
        CHECKED[source] += 1
        CHECKED[f"{source}:refused"] += run.refused is not None
    return runs


def assert_all_equal(runs: list) -> None:
    failed = [(name, run) for name, run in runs if not run.equal]
    assert not failed, whole.explain(failed)


def labels(runs: list) -> Counter:
    return Counter(run.label for _name, run in runs)


# --------------------------------------------------------------------------
# Граница
# --------------------------------------------------------------------------


def test_the_dropin_has_the_signature_of_the_oracle():
    oracle, native = inspect.signature(skeleton.build_skeleton), inspect.signature(cftuv_native.build_skeleton)
    assert list(oracle.parameters) == list(native.parameters) == ["polygon", "split_search", "work_budget", "dense_hydration"]
    assert [item.kind for item in oracle.parameters.values()] == [item.kind for item in native.parameters.values()]
    assert oracle.parameters["work_budget"].default is None is native.parameters["work_budget"].default
    assert oracle.parameters["dense_hydration"].default is False is native.parameters["dense_hydration"].default
    # the oracle's default search is spelt by its absence (the kernel's enum is not importable when the shim is)
    assert oracle.parameters["split_search"].default is skeleton.SplitSearch.MOTORCYCLE and native.parameters["split_search"].default is None


def test_the_operation_is_pinned_and_holds_the_native_lock_for_the_whole_call():
    assert cftuv_native.native_status()["skeleton"] == "available"
    assert cftuv_native.NATIVE_REFUSALS == (cftuv_native.NativePortStale, cftuv_native.NativeUnsupportedPython, cftuv_native.NativePortUnsupported, cftuv_native.NativeDivisionDiverged)
    # a public method of the mirror is wrapped by `serialize_public_methods`: the lock is taken by the call, not by the caller
    assert hasattr(cftuv_native.CostMirror.build_skeleton, "__wrapped__")


def test_the_oracle_statuses_of_the_operation_are_one_table_for_the_shim_and_the_extension():
    from cftuv_native import skeleton_op

    assert tuple(sorted(skeleton_op.SKELETON_STATUSES)) == tuple(sorted(cftuv_native.skeleton_oracle_statuses()))
    assert not {5, 6, 7, 12} & skeleton_op.SKELETON_STATUSES


def test_the_fixed_texts_of_the_shim_are_the_texts_the_oracle_raises():
    from cftuv_native import skeleton_op

    import cftuv_envelope.wavefront.event_time as event_time

    assert skeleton_op.ZERO_DIVISOR_TIME_TEXT == "знаменатель времени доказанно нулевой"
    with pytest.raises(event_time.ZeroDivisorTimeError, match=skeleton_op.ZERO_DIVISOR_TIME_TEXT):
        event_time.EventTimeV1.normalized(1, exact.SqrtSumV1(()))
    line = event_time.SupportLineV1.through((0, 0), (1, 0))
    other = event_time.SupportLineV1.through((0, 1), (1, 1))
    with pytest.raises(event_time.ParallelSupportLinesError, match=skeleton_op.PARALLEL_LINES_TEXT):
        event_time.event_point(line, other, event_time.ZERO_TIME)


# --------------------------------------------------------------------------
# Корпуса через вставку
# --------------------------------------------------------------------------


@needs_field
def test_every_field_record_through_the_dropin_equals_the_recorded_oracle(runner):
    runs = run_rows(runner, FIELD, _rows(FIELD, derived=False), "field")
    assert len(runs) >= 150
    assert_all_equal(runs)
    assert not any(run.refused for _name, run in runs), "the port answers every field domain: no refusal"
    assert labels(runs)["EXACT"] >= 150


@needs_field
def test_every_derived_starved_budget_field_record_through_the_dropin_equals_the_recorded_oracle(runner):
    runs = run_rows(runner, FIELD, _rows(FIELD, derived=True), "field-derived")
    assert len(runs) >= 900
    assert_all_equal(runs)
    assert not any(run.refused for _name, run in runs)
    assert labels(runs)["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 900


@needs_synthetic
def test_every_synthetic_record_through_the_dropin_equals_the_recorded_oracle(runner):
    rows = _rows(SYNTHETIC, derived=None)[::SYNTHETIC_STRIDE]
    runs = run_rows(runner, SYNTHETIC, rows, "synthetic")
    assert len(runs) >= 5000 // SYNTHETIC_STRIDE
    assert_all_equal(runs)
    refused = [(name, run) for name, run in runs if run.refused]
    for name, run in refused:
        record = sc.read(SYNTHETIC, next(row for row in rows if row["id"] == name))
        assert run.refused[0] == "NativePortUnsupported", (name, run.refused)
        assert sc.is_internal_error(record.expected().exception), f"{name}: the port refused a call the oracle answers: {run.refused}"
    counts = labels(runs)
    assert counts["EXACT"] > 1000 * 1 // SYNTHETIC_STRIDE
    for name in ("raised:ExactCanonicalizationWorkBudgetExhausted", "LEVEL_BUDGET_EXHAUSTED", "SUPERLEVEL_COMPONENT_UNRESOLVABLE", "WAVEFRONT_LEFT_UNRESOLVED"):
        assert counts[name] >= 1, (name, counts)


@needs_synthetic
def test_the_dense_hydration_and_the_replaced_level_budget_records_are_in_the_synthetic_set():
    rows = _rows(SYNTHETIC, derived=False)
    assert any(row["dense_hydration"] for row in rows) and any(row["outcome"] == "LEVEL_BUDGET_EXHAUSTED" for row in rows)
    assert any(row["budget"] is False for row in rows), "a call without a budget (UNBUDGETED_WORK) is in the corpus"


@pytest.mark.skipif(FIELD is None and SYNTHETIC is None, reason=sc.describe_missing("field"))
def test_a_sample_through_the_dropin_equals_the_LIVE_oracle(runner):
    """Не запись, а живой вызов ядра в этом интерпретаторе: запись могла быть сделана на другом (3.11 против 3.13), ответ и цена от него не зависят."""

    chosen = []
    for root in (FIELD, SYNTHETIC):
        if root is None:
            continue
        rows = [row for row in _rows(root, derived=None) if row["seconds"] <= 0.4 and not sc.is_internal_error(row["exception"])]
        chosen += [(root, row) for row in rows[:: max(1, len(rows) // (LIVE_LIMIT // 2))]][: LIVE_LIMIT // 2]
    assert chosen
    runs = []
    for root, row in chosen:
        run = runner.compare(sc.read(root, row), live=True)
        runs.append((row["id"], run))
        CHECKED["live"] += 1
    assert_all_equal(runs)


# --------------------------------------------------------------------------
# Вызовы одного сеанса
# --------------------------------------------------------------------------


def _chain(root, count: int) -> list:
    rows = [row for row in _rows(root, derived=False) if row["outcome"] == "EXACT" and 0.01 <= row["seconds"] <= 1.5]
    return rows[:: max(1, len(rows) // count)][:count]


@needs_field
def test_repeated_calls_on_one_session_carry_the_warm_memory_exactly_as_the_oracle_does(runner):
    """Три вызова подряд без восстановления состояния: память канонизации (LRU, касания) и статьи бюджета копятся, и копятся так же, как у эталона."""

    rows = _chain(FIELD, 6)
    assert len(rows) >= 4
    records = [sc.read(FIELD, row) for row in rows]
    first = records[0].before()
    steps = [records[index % len(records)] for index in range(9)]
    outcomes = {}
    for side in ("oracle", "native"):
        budget, store = nc.restore_state(first)
        shared = budget
        states, results = [], []
        for record in steps:
            call = nc.decode_call(nc.OP_SKELETON, record.call_blob, shared, store)
            states.append(nc.capture_state(call.budget, None))
            results.append(nc.execute(call, function=None if side == "oracle" else runner.mirror.build_skeleton))
        outcomes[side] = (states, results)
    for index, (want, got) in enumerate(zip(outcomes["oracle"][1], outcomes["native"][1])):
        before = outcomes["oracle"][0][index]
        assert not nc.compare_outcomes(nc.OP_SKELETON, before, want, got), f"step {index}: {nc.compare_outcomes(nc.OP_SKELETON, before, want, got)}"
        CHECKED["chain-steps"] += 1
    assert outcomes["oracle"][0][1].known_primes and outcomes["oracle"][0][1].factorization, "the second call started on a warm memory"


@needs_field
def test_the_level_budget_the_oracle_reads_live_is_read_live_here_too(runner):
    record = sc.read(FIELD, next(row for row in _rows(FIELD, derived=False) if row["outcome"] == "EXACT" and row["seconds"] < 0.3))
    before = record.before()
    for limit in (1, 2):
        # `nc.invoke` pins `skeleton.level_budget` to the `level_budget` of the call it is given (the recorded one), for the oracle and for the drop-in alike
        outcomes = []
        for function in (None, runner.mirror.build_skeleton):
            call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)  # the state is restored for EACH of the two runs
            call.kwargs["level_budget"] = limit
            outcomes.append(nc.execute(call, function=function))
        expected, actual = outcomes
        assert expected.result.outcome.value == "LEVEL_BUDGET_EXHAUSTED"
        assert not nc.compare_outcomes(nc.OP_SKELETON, before, expected, actual)


@needs_field
def test_a_call_without_a_budget_pays_the_unbudgeted_telemetry_and_writes_no_superlevel(runner):
    row = next(row for row in _rows(FIELD, derived=False) if row["outcome"] == "EXACT" and row["seconds"] < 0.3)
    record = sc.read(FIELD, row)
    before = record.before()
    call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
    kwargs = dict(call.kwargs)
    kwargs.pop("level_budget")
    polygon = call.args[0]
    exact.reset_unbudgeted_work()
    exact.reset_sign_counts()
    expected_result = skeleton.build_skeleton(polygon, **kwargs)
    oracle_state = nc.capture_state(None, None)
    nc.restore_state(before)
    exact.reset_unbudgeted_work()
    exact.reset_sign_counts()
    actual_result = runner.mirror.build_skeleton(polygon, **kwargs)
    native_state = nc.capture_state(None, None)
    assert nc.canonical(expected_result) == nc.canonical(actual_result)
    assert native_state.unbudgeted == oracle_state.unbudgeted != (0, 0, 0, 0, 0, 0)
    assert native_state.sign_counts == oracle_state.sign_counts
    assert native_state.known_primes == oracle_state.known_primes and native_state.factorization == oracle_state.factorization


@needs_field
def test_the_superlevel_string_of_a_named_budget_is_what_the_oracle_writes_even_when_it_was_not_empty(runner):
    record = sc.read(FIELD, next(row for row in _rows(FIELD, derived=False) if row["outcome"] == "EXACT" and row["seconds"] < 0.3))
    before = dataclasses.replace(record.before(), budget={**record.before().budget, "superlevel": "99"})
    expected = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before))
    actual = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before), function=runner.mirror.build_skeleton)
    assert not nc.compare_outcomes(nc.OP_SKELETON, before, expected, actual)
    assert actual.after.budget["superlevel"] == str(expected.result.levels) != "99"


# --------------------------------------------------------------------------
# Отказы порта оставляют состояние как до вызова
# --------------------------------------------------------------------------


def _small_record():
    return sc.read(FIELD, next(row for row in _rows(FIELD, derived=False) if row["outcome"] == "EXACT" and row["seconds"] < 0.3 and row["fan_supports"]))


def _refusal_case(runner, record, arrange, expected_text: str):
    """Вызов, на котором порт обязан отказать по имени: состояние (память, статьи, `superlevel`, знаки) как до вызова, и эталон после отказа даёт свой исход."""

    before = record.before()
    call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
    with arrange():
        actual = nc.execute(call, function=runner.mirror.build_skeleton)
        assert actual.exception is not None and actual.exception[0] == "NativePortUnsupported" and expected_text in actual.exception[1], actual.exception
        assert not whole._untouched(before, actual.after)
        fallback = nc.execute(call)
    assert not nc.compare_outcomes(nc.OP_SKELETON, before, record.expected(), fallback)


@needs_field
def test_a_split_search_the_port_does_not_know_is_a_named_refusal_that_leaves_the_state_alone(runner):
    record = _small_record()
    call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, record.before())
    before = record.before()
    options = {**call.kwargs, "split_search": "BREADTH_FIRST"}
    options.pop("level_budget")
    with pytest.raises(cftuv_native.NativePortUnsupported, match="exhaustive split search"):
        runner.mirror.build_skeleton(call.args[0], work_budget=call.budget, **options)
    assert not whole._untouched(before, nc.capture_state(call.budget, None))


@needs_synthetic
def test_the_exhaustive_split_search_of_the_oracle_is_carried_and_equals_the_oracle(runner):
    """`SplitSearch.EXHAUSTIVE` (эталон тестов: ни графа, ни индекса, каждый кандидат каждой reflex-вершины против каждого ребра) в синтетическом корпусе — сотни записей."""

    rows = [row for row in _rows(SYNTHETIC, derived=None) if row["split_search"] == "EXHAUSTIVE"]
    assert len(rows) >= 300
    runs = run_rows(runner, SYNTHETIC, rows, "exhaustive")
    assert_all_equal(runs)
    assert not any(run.refused for _name, run in runs)
    assert Counter(run.label for _n, run in runs)["EXACT"] >= 200


@needs_field
def test_a_replaced_march_budget_is_a_named_refusal(runner, monkeypatch):
    import contextlib

    @contextlib.contextmanager
    def patched():
        monkeypatch.setattr(motorcycle, "march_budget", lambda grid: 7)
        try:
            yield
        finally:
            monkeypatch.undo()

    _refusal_case(runner, _small_record(), patched, "march_budget")


@needs_field
def test_a_budget_that_is_not_an_exact_work_budget_is_a_named_refusal(runner):
    record = _small_record()
    call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, record.before())
    options = dict(call.kwargs)
    options.pop("level_budget")

    class Imitation:
        cap = None

        def spent_by_article(self):
            return (0, 0, 0, 0, 0, 0)

    with pytest.raises(cftuv_native.NativePortUnsupported, match="ExactWorkBudgetV1"):
        runner.mirror.build_skeleton(call.args[0], work_budget=Imitation(), **options)


@needs_field
def test_a_polygon_beyond_the_machine_range_is_a_named_refusal_and_the_oracle_answers_it(runner):
    from cftuv_envelope.wavefront.polygon import PolygonV1

    scale = 1 << 62
    polygon = PolygonV1.build([(0, 0), (4 * scale, 0), (4 * scale, 4 * scale), (0, 4 * scale)])
    before = nc.capture_state(None, None)
    with pytest.raises(cftuv_native.NativePortUnsupported, match="machine range"):
        runner.mirror.build_skeleton(polygon)
    after = nc.capture_state(None, None)
    assert after.known_primes == before.known_primes and after.factorization == before.factorization and after.unbudgeted == before.unbudgeted
    assert skeleton.build_skeleton(polygon).outcome is skeleton.SkeletonOutcome.EXACT


@needs_field
@pytest.mark.parametrize("kind", ("invalid_input", "diverged", "internal", "unsupported", "panic"))
def test_a_refusal_the_port_takes_after_it_computed_leaves_every_state_as_before(runner, kind):
    """Вызов СЧИТАЛСЯ целиком (есть статьи, знаки, журнал памяти), и только потом порт отказал: ничего из этого не должно дойти до процесса."""

    record = _small_record()
    before = record.before()
    call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
    mirror = cftuv_native.new_mirror()
    options = dict(call.kwargs)
    options.pop("level_budget")
    mirror.force_refusal(kind)
    expected = RuntimeError if kind == "panic" else cftuv_native.NATIVE_REFUSALS
    with pytest.raises(expected):
        mirror.build_skeleton(call.args[0], work_budget=call.budget, **options)
    assert not whole._untouched(before, nc.capture_state(call.budget, None))
    # the mirror forgot what the aborted call did to it: the next call (the oracle's own state) is answered whole
    again = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before), function=mirror.build_skeleton)
    assert not nc.compare_outcomes(nc.OP_SKELETON, before, record.expected(), again)


def test_an_interpreter_started_with_O_refuses_by_name():
    """`python -O` убирает два `assert compare_times(...) == 0`, которые у эталона стоят знак: порт их платит, поэтому отказывает названным отказом."""

    code = (
        "import sys; sys.path[:0] = [%r, %r]\n"
        "import cftuv_native\n"
        "from cftuv_envelope.wavefront.polygon import PolygonV1\n"
        "try:\n"
        "    cftuv_native.build_skeleton(PolygonV1.build([(0, 0), (4, 0), (4, 4), (0, 4)]))\n"
        "except cftuv_native.NativePortUnsupported as error:\n"
        "    print('REFUSED', error)\n"
    ) % (str(ROOT / "kernel" / "src"), os.environ.get("CFTUV_NATIVE_SITE", ""))
    environment = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    done = subprocess.run([sys.executable, "-O", "-c", code], capture_output=True, text=True, env=environment, timeout=120)
    assert done.returncode == 0, done.stderr
    assert "REFUSED" in done.stdout and "-O" in done.stdout, done.stdout + done.stderr


# --------------------------------------------------------------------------
# Объекты результата
# --------------------------------------------------------------------------


@needs_field
def test_the_result_is_made_of_ordinary_python_objects_that_pickle_and_compare_as_the_oracles_do(runner):
    import pickle

    record = sc.read(FIELD, next(row for row in _rows(FIELD, derived=False) if row["outcome"] == "EXACT" and row["seconds"] < 0.5))
    before = record.before()
    actual = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before), function=runner.mirror.build_skeleton)
    expected = record.expected()
    assert type(actual.result) is type(expected.result) and actual.result == expected.result
    assert pickle.loads(pickle.dumps(actual.result)) == expected.result
    assert hash(actual.result.nodes[0]) == hash(expected.result.nodes[0])
    node = actual.result.nodes[-1]
    assert type(node.time.dividend).__name__ == "Fraction" and type(node.point.x.terms[0][1]).__name__ in ("Fraction", "int")
    assert not hasattr(node, "__dict__")


@needs_field
def test_the_raw_layouts_of_the_result_classes_and_the_attribute_fallback_build_the_same_objects(runner):
    record = _small_record()
    before = record.before()
    mirror = cftuv_native.new_mirror()
    layouts = mirror.skeleton_raw_layouts()
    assert all(layouts.values()) and set(layouts) == {"SkeletonV1", "SkeletonNodeV1", "EventTimeV1", "EventPointV1", "ProofObligationV1"}
    fast = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before), function=mirror.build_skeleton)
    mirror.disable_raw()
    slow = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before), function=mirror.build_skeleton)
    assert not nc.compare_outcomes(nc.OP_SKELETON, before, fast, slow)
    assert not nc.compare_outcomes(nc.OP_SKELETON, before, record.expected(), slow)


# --------------------------------------------------------------------------
# Потоки
# --------------------------------------------------------------------------


def _polygons(count: int = 8) -> list:
    return [(name, polygon) for name, polygon in wavefront_cases.named_corpus()][:count]


@pytest.mark.parametrize("threads", [2, 4])
def test_calls_from_several_threads_are_bit_equal_to_sequential_calls(threads):
    """Сессия и таблицы памяти — состояние ПРОЦЕССА: вызовы идут под `NATIVE_LOCK`, и ответы потоков равны последовательным (цена тоже: счётчики знаков равны сумме последовательных)."""

    polygons = _polygons()
    assert len(polygons) >= 6
    before = dict(exact.SIGN_COUNTS)
    reference = {name: nc.canonical(cftuv_native.build_skeleton(polygon)) for name, polygon in polygons}
    sequential = {key: exact.SIGN_COUNTS[key] - before[key] for key in before}
    outcomes: dict = {}
    errors: list = []
    guard = threading.Lock()
    start = threading.Barrier(threads)

    def worker(number: int) -> None:
        try:
            start.wait(60)
            for round_number in range(2):
                for index in range(len(polygons)):
                    name, polygon = polygons[(index + number * 3 + round_number) % len(polygons)]
                    found = nc.canonical(cftuv_native.build_skeleton(polygon))
                    with guard:
                        outcomes.setdefault(name, set()).add(found)
        except BaseException as exc:  # noqa: BLE001 - any error of a thread fails the test, by name
            with guard:
                errors.append(f"thread {number}: {type(exc).__name__}: {exc}")

    before = dict(exact.SIGN_COUNTS)
    pool = [threading.Thread(target=worker, args=(number,)) for number in range(threads)]
    for thread in pool:
        thread.start()
    for thread in pool:
        thread.join()
    concurrent = {key: exact.SIGN_COUNTS[key] - before[key] for key in before}
    assert not errors, errors
    assert {name: found for name, found in outcomes.items()} == {name: {value} for name, value in reference.items()}
    assert concurrent == {key: value * threads * 2 for key, value in sequential.items()}, (sequential, concurrent)


# --------------------------------------------------------------------------
# Подготовка целиком: потребитель скелета видит то же самое
# --------------------------------------------------------------------------

FIXTURES = ("building_002_point_contact_v1", "building_002_weighted_normals_v1", "building_002_full_selection_v1", "wall_noise_top_rung_clip_v1")


def _prepared(fixture: str, skeleton_function):
    """`wavefront.prepare_conveyor` на холодном процессе (как в продукте: память и неоплаченное сброшены), где вызов `build_skeleton` конвейера — `skeleton_function`."""

    directory = ROOT / "kernel" / "fixtures" / fixture
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((directory / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((directory / "decal_request.json").read_bytes())
    exact.reset_factorization_memory()
    exact.reset_unbudgeted_work()
    exact.reset_sign_counts()
    original = conveyor.build_skeleton
    conveyor.build_skeleton = skeleton_function
    try:
        prepared = wavefront.prepare_conveyor(snapshot, request)
    finally:
        conveyor.build_skeleton = original
    return prepared, nc.capture_state(prepared.work_budget, None)


@pytest.mark.parametrize("fixture", FIXTURES)
def test_a_preparation_through_the_dropin_equals_the_oracles_down_to_the_face_partition(fixture, runner):
    """Читатели скелета (`build_faces_traced`: узлы, их точки как ТЕ ЖЕ `SqrtSumV1`, что потом лежат в гранях; шаг ширины читает РАЗБИЕНИЕ) видят то же, что у эталона."""

    import pickle

    oracle, oracle_state = _prepared(fixture, skeleton.build_skeleton)
    native, native_state = _prepared(fixture, runner.mirror.build_skeleton)
    assert oracle.outcome.value == native.outcome.value
    for left, right in zip(oracle.regions, native.regions):
        assert left.skeleton == right.skeleton and nc.canonical(left.skeleton) == nc.canonical(right.skeleton)
        if left.partition is not None:
            blank = lambda found: dataclasses.replace(found, work_budget=None)  # noqa: E731 - the budget is not an answer
            assert nc.canonical(blank(left.partition)) == nc.canonical(blank(right.partition))
            assert all(face.points == other.points for face, other in zip(left.partition.faces, right.partition.faces))
    assert native_state.budget["articles"] == oracle_state.budget["articles"] and native_state.budget["superlevel"] == oracle_state.budget["superlevel"]
    assert native_state.known_primes == oracle_state.known_primes and native_state.factorization == oracle_state.factorization
    assert native_state.squarefree == oracle_state.squarefree and native_state.prime_support == oracle_state.prime_support
    assert native_state.sign_counts == oracle_state.sign_counts
    # the preparation crosses processes (pool workers, the session cache): ordinary objects only, no native handle inside
    assert pickle.loads(pickle.dumps(native)).regions[0].skeleton == native.regions[0].skeleton

