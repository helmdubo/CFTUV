"""Скан остатка доменов идёт ЗА пулом (SCAN_BEHIND_THE_POOL_V1, хост без Blender, настоящие воркеры).

Утверждения:

1. ВИДИМОЕ ТО ЖЕ: холодное нажатие с потоковым обходом и с выключателем (`STREAMED_SCAN = False`) даёт равные ответы, равные ключи всех кэшей В ПОРЯДКЕ ЗАПИСИ, равные записи сборки для шага ширины, равные
   счётчики прогона (кроме названного `PRODUCTION_SCAN_STREAMED`), ту же последовательность приёма ответов и те же ЗАДАЧИ пула (номер, патч, домен, байты кадра); пул начинает с того же первого круга (`head_streamed`, `head_misses`);
2. КАНДИДАТЫ ПУЛА: пул, которому голова и остаток даны порознь (`LateTasksV1`), собирает тот же первый круг и тот же упорядоченный остаток, что даёт полный список (`_Run`);
3. ГРАНИЦЫ: нажатие, у которого в сессии есть хоть что-то (метрика, подготовка, результат, привязка ключа, запись хранилища, пометка фона, запись сборки), малая партия, пул без `late`, lean-прогон идут прежним путём
   и счётчика не имеют;
4. ОТКАЗЫ НАЗВАНЫ: пул, который не стартовал, не теряет остаток (он отсканирован после отказа, ответ прежний); исключение скана остатка поднимается из прогона после того, как пул кончил голову (тот же тип, что поднял бы `_scan`);
   запись домена не холодная с входом выгрузки - названный инвариант.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv import envelope_production_scan as scan_module  # noqa: E402
from cftuv import envelope_production_stream as stream  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPoolUnavailable,
    LateTasksV1,
    _Run,
    encode_frame,
    shutdown_domain_pool,
)
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_scan_memo import ScanRecordV1  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

REAL_ADOPT = production._adopt_cold  # noqa: SLF001 - настоящие функции: подмена теста не накладывается на подмену прошлого нажатия в том же тесте
REAL_SETTLE = production.settle_dispatch
ROWS = 24  # >= 8 доменов на воркера при двух воркерах (STREAMED_HEAD_MIN_TASKS_PER_WORKER)
WORKERS = 2
ALPHA = 0.25
#: Счётчики, которые зависят от того, КАКОЙ воркер взял задачу и сколько байт ушло: не ответы.
SCHEDULING = ("PRODUCTION_POOL_", "PRODUCTION_INTERVAL_", "PRODUCTION_CLIP_MEMO_")


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


@pytest.fixture(autouse=True)
def _fresh_pool_affinity(monkeypatch):
    """Каждое нажатие теста начинает с одинакового пула: привязка к воркерам прошлого нажатия не должна менять путь."""

    shutdown_domain_pool()
    yield


def _press(monkeypatch, *, streamed, rows=ROWS, controller=None, bundle=None, workers=WORKERS):
    monkeypatch.setattr(stream, "STREAMED_SCAN", streamed)
    controller = EnvelopeDebugSessionController() if controller is None else controller
    log = {"adopted": [], "tasks": None, "stats": None}

    def adopting(run, entry, reply, placement):
        log["adopted"].append(entry.domain_id)
        return REAL_ADOPT(run, entry, reply, placement)

    def settling(run, ready, cold, domain_pool, tasks, pooled, failure, shipping_failures):
        log["tasks"] = [(task.task_id, task.patch_id, task.domain_id, encode_frame(task)) for task in tasks]
        log["stats"] = None if pooled is None else (pooled.stats.head_streamed, pooled.stats.head_misses)
        return REAL_SETTLE(run, ready, cold, domain_pool, tasks, pooled, failure, shipping_failures)

    monkeypatch.setattr(production, "_adopt_cold", adopting)
    monkeypatch.setattr(production, "settle_dispatch", settling)
    run = run_production(
        controller,
        quad_row_bundle(rows) if bundle is None else bundle,
        frozenset(range(rows)),
        ALPHA,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
    )
    return run, controller, log


def _state(run, controller):
    """Всё видимое после нажатия: ответы, ключи кэшей В ПОРЯДКЕ ЗАПИСИ, записи сборки для шага ширины, счётчики прогона (кроме байтов, времени и размещения)."""

    counters = {
        (item.name, item.patch_domain_id): item.value
        for item in run.profile.counters
        if item.name != stream.PRODUCTION_SCAN_STREAMED
        and "BYTES" not in item.name
        and not item.name.endswith(("_US", "_SECONDS", "_MS"))
        and not item.name.startswith(SCHEDULING)
    }
    records = {
        tuple(map(repr, key)): {patch: (record.selected, record.prep_key, record.binding, record.request) for patch, record in sorted(found.items())}
        for key, found in controller.scan_memo._runs.items()  # noqa: SLF001
    }
    return {
        "answer": [(item.patch_id, item.domain_id, item.outcome, item.detail, item.content_digest if item.is_materialized else "") for item in run.results],
        "preparation_keys": list(map(repr, controller._conveyor_preparation_cache)),  # noqa: SLF001
        "result_keys": list(map(repr, controller._production_result_cache)),  # noqa: SLF001
        "bindings": sorted(map(repr, controller._content_bindings)),  # noqa: SLF001
        "metrics": sorted(map(repr, controller._patch_metric_cache)),  # noqa: SLF001
        "store": list(controller.content_store._entries),  # noqa: SLF001
        "records": records,
        "counters": counters,
    }


def _streamed(run):
    return run.counter(stream.PRODUCTION_SCAN_STREAMED)


# --------------------------------------------------------------------------
# 1. Видимое то же
# --------------------------------------------------------------------------


def test_a_cold_press_scans_the_rest_behind_the_pool_and_everything_observable_is_the_same(monkeypatch):
    streamed_run, streamed_controller, streamed_log = _press(monkeypatch, streamed=True)
    plain_run, plain_controller, plain_log = _press(monkeypatch, streamed=False)

    candidates = stream.STREAMED_SCAN_CANDIDATES_PER_WORKER * WORKERS
    assert _streamed(streamed_run) == ROWS - candidates  # остаток отсканирован за пулом
    assert _streamed(plain_run) is None  # прежний путь счётчика не пишет
    assert all(item.is_materialized for item in streamed_run.results) and len(streamed_run.results) == ROWS
    assert _state(streamed_run, streamed_controller) == _state(plain_run, plain_controller)
    assert streamed_log["adopted"] == plain_log["adopted"]  # порядок приёма ответов: по номеру задачи
    assert streamed_log["tasks"] == plain_log["tasks"]  # те же задачи: номер, патч, домен, байты кадра
    assert streamed_log["stats"] == plain_log["stats"] and streamed_log["stats"][0] == 1  # первый круг по отбору кандидатов в обоих


def test_the_second_press_of_the_same_session_is_warm_and_takes_the_old_path(monkeypatch):
    first, controller, _log = _press(monkeypatch, streamed=True)
    second, _controller, _log2 = _press(monkeypatch, streamed=True, controller=controller)

    assert _streamed(first) is not None and _streamed(second) is None
    assert not second.cold and all(item.is_materialized for item in second.results)


# --------------------------------------------------------------------------
# 2. Кандидаты пула
# --------------------------------------------------------------------------


def _tasks(rows=ROWS):
    """Настоящие холодные задачи прогона (без пула): те же, что построил бы `_dispatch`."""

    controller = EnvelopeDebugSessionController()
    captured = {}
    real = REAL_SETTLE

    def spy(run, ready, cold, domain_pool, tasks, pooled, failure, shipping_failures):
        captured["tasks"] = list(tasks)
        return real(run, ready, cold, domain_pool, tasks, pooled, failure, shipping_failures)

    class _Fake:
        requested = WORKERS

        def run(self, tasks, cancel=None):
            raise DomainPoolUnavailable("no workers in this test")

    import pytest as _pytest

    with _pytest.MonkeyPatch.context() as patch:
        patch.setattr(stream, "STREAMED_SCAN", False)
        patch.setattr(production, "settle_dispatch", spy)
        patch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": _Fake())
        run_production(
            controller,
            quad_row_bundle(rows),
            frozenset(range(rows)),
            ALPHA,
            source_object_key="object",
            source_data_key="mesh",
            density=None,
            workers=WORKERS,
        )
    return captured["tasks"]


def _participants(count=WORKERS):
    return [SimpleNamespace(index=index) for index in range(count)]


def test_the_pool_builds_the_same_first_round_and_the_same_ordered_rest_from_a_head_and_late_tasks():
    tasks = _tasks()
    assert len(tasks) == ROWS
    candidates = stream.STREAMED_SCAN_CANDIDATES_PER_WORKER * WORKERS
    ranked = sorted(tasks, key=lambda task: (-len(task.export.host_chains), task.task_id))
    head = sorted(ranked[:candidates], key=lambda task: task.task_id)
    rest = [task for task in tasks if task not in head]

    whole = _Run({}, _participants(), list(tasks), None)
    split = _Run({}, _participants(), list(head), None, LateTasksV1(len(tasks), lambda: list(rest)))
    whole._order_rest()  # noqa: SLF001
    split._order_rest()  # noqa: SLF001

    assert whole.streamed and split.streamed
    assert [(task.task_id, frame) for task, frame in whole.head] == [(task.task_id, frame) for task, frame in split.head]
    assert {worker: (item[0].task_id) for worker, item in whole.started.items()} == {worker: item[0].task_id for worker, item in split.started.items()}
    assert [(task.task_id, frame) for task, frame in whole.pending._items] == [(task.task_id, frame) for task, frame in split.pending._items]  # noqa: SLF001
    assert whole.counted._values == split.counted._values  # noqa: SLF001


def test_a_pool_that_does_not_stream_takes_the_late_tasks_before_it_starts():
    tasks = _tasks()
    few, late = tasks[:5], tasks[5:6]
    run = _Run({}, _participants(), list(few), None, LateTasksV1(6, lambda: list(late)))

    assert not run.streamed and run.late is None  # малая партия: остаток нужен до старта, как у полного списка
    seen = {item[0].task_id for item in run.pending._items} | {item[0].task_id for item in run.started.values()}  # noqa: SLF001
    assert seen == {task.task_id for task in tasks[:6]}


# --------------------------------------------------------------------------
# 3. Границы
# --------------------------------------------------------------------------


def test_a_small_batch_and_a_pool_without_late_take_the_old_path(monkeypatch):
    small, _controller, _log = _press(monkeypatch, streamed=True, rows=6)
    assert _streamed(small) is None and all(item.is_materialized for item in small.results)

    class _Plain:
        requested = WORKERS

        def run(self, tasks, cancel=None):
            raise DomainPoolUnavailable("a pool without late")

    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": _Plain())
    monkeypatch.setattr(stream, "STREAMED_SCAN", True)
    controller = EnvelopeDebugSessionController()
    run = run_production(
        controller, quad_row_bundle(ROWS), frozenset(range(ROWS)), ALPHA, source_object_key="object", source_data_key="mesh", density=None, workers=WORKERS
    )
    assert _streamed(run) is None


@pytest.mark.parametrize(
    "name, marker",
    (
        ("_patch_metric_cache", object()),
        ("_conveyor_preparation_cache", object()),
        ("_production_result_cache", object()),
        ("_content_bindings", "key"),
        ("_byte_check_refusals", ("another revision", None, None)),  # чужая ревизия: пометка есть, на домены этого прогона она не действует
    ),
)
def test_anything_in_the_session_that_can_change_a_decision_of_the_scan_keeps_the_old_path(monkeypatch, name, marker):
    controller = EnvelopeDebugSessionController()
    getattr(controller, name)["marker"] = marker

    run, _controller, _log = _press(monkeypatch, streamed=True, controller=controller)

    assert _streamed(run) is None and len(run.results) == ROWS


def test_a_store_entry_a_scan_record_or_a_lean_run_keep_the_old_path(monkeypatch):
    from cftuv.envelope_content_store import ContentStoreV1

    controller = EnvelopeDebugSessionController()
    controller.content_store._entries["marker"] = SimpleNamespace(prepared=None)  # noqa: SLF001 - запись другого содержимого: на домены этого прогона она не действует
    assert len(controller.content_store) == 1
    run, _c, _log = _press(monkeypatch, streamed=True, controller=controller)
    assert _streamed(run) is None
    assert isinstance(controller.content_store, ContentStoreV1)

    seen = EnvelopeDebugSessionController()
    real = seen.scan_memo.records_of
    marked = []

    def with_record(key):
        records = real(key)
        if records is not None and not marked:
            records[10_000] = ScanRecordV1(frozenset({-1}), None, None, ("key",), None)  # запись патча, которого в прогоне нет
            marked.append(key)
        return records

    monkeypatch.setattr(seen.scan_memo, "records_of", with_record)
    run, _c, _log = _press(monkeypatch, streamed=True, controller=seen)
    assert _streamed(run) is None


# --------------------------------------------------------------------------
# 4. Отказы названы
# --------------------------------------------------------------------------


class _DeadPool:
    """Пул, который принимает `late` и не стартует: `_run_tasks` называет отказ, остаток домены доскан после него."""

    requested = WORKERS
    streams_late = True

    def run(self, tasks, cancel=None, late=None):
        raise DomainPoolUnavailable("the workers did not start")


def test_a_pool_that_cannot_start_loses_no_domain_and_the_answer_is_the_old_one(monkeypatch):
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": _DeadPool())
    streamed_run, streamed_controller, streamed_log = _press(monkeypatch, streamed=True, rows=ROWS)
    plain_run, plain_controller, plain_log = _press(monkeypatch, streamed=False, rows=ROWS)

    assert _streamed(streamed_run) == 0  # пул не стартовал: за пулом ничего не отсканировано
    assert [item.placement for item in streamed_run.results] == [item.placement for item in plain_run.results]
    assert all(item.is_materialized for item in streamed_run.results)
    assert _state(streamed_run, streamed_controller) == _state(plain_run, plain_controller)
    assert streamed_log["tasks"] == plain_log["tasks"] and streamed_log["adopted"] == plain_log["adopted"]


def test_an_exception_of_the_rest_scan_comes_out_of_the_run_after_the_pool_finished_the_head(monkeypatch):
    real = scan_module._Scan.entry  # noqa: SLF001
    calls = []

    def entry(self, patch_id):
        calls.append(patch_id)
        if len(calls) == ROWS:  # последний отсканированный - из остатка
            raise ValueError("the scan of the last domain failed")
        return real(self, patch_id)

    monkeypatch.setattr(scan_module._Scan, "entry", entry)  # noqa: SLF001
    with pytest.raises(ValueError, match="the scan of the last domain failed"):
        _press(monkeypatch, streamed=True)
    assert len(calls) == ROWS
    monkeypatch.undo()
    run, _controller, _log = _press(monkeypatch, streamed=True)  # пул цел: следующее нажатие считается
    assert all(item.is_materialized for item in run.results)


def test_an_entry_that_is_not_a_plain_cold_domain_on_a_cold_session_is_a_named_invariant(monkeypatch):
    real = scan_module._Scan.entry  # noqa: SLF001

    def entry(self, patch_id):
        found = real(self, patch_id)
        return found if patch_id != 3 else scan_module._DomainEntryV1(found.patch_id, found.domain_id, found.selected, failure=RuntimeError("x"))  # noqa: SLF001

    monkeypatch.setattr(scan_module._Scan, "entry", entry)  # noqa: SLF001
    with pytest.raises(RuntimeError, match=stream.SCAN_BEHIND_THE_POOL_INVARIANT):
        _press(monkeypatch, streamed=True)


# --------------------------------------------------------------------------
# Очередь пула: готовая часть сразу, остаток позже
# --------------------------------------------------------------------------


def _queue_items(costs):
    """Пары `(задача, кадр)` холодных задач заданной цены: цена холодной задачи с выгрузкой - длина кадра, поэтому кадр нужной длины."""

    items = []
    for number, cost in enumerate(costs):
        task = SimpleNamespace(task_id=number, cold=object(), export=object(), production=None, coverage=None)
        items.append((task, b"x" * int(cost / pool_module.EXPORT_FRAME_COST_SCALE)))
    return items


def test_a_worker_that_drains_the_prefilled_queue_waits_for_the_rest_instead_of_leaving():
    import threading
    import time

    ready, rest = _queue_items([900, 800]), _queue_items([700, 600])
    for number, (task, _frame) in enumerate(rest):
        task.task_id = 10 + number
    queue = pool_module._PendingTasks()  # noqa: SLF001
    taken = []

    def worker():
        while True:
            item = queue.take(SimpleNamespace(held=()))
            if item is None:
                return
            taken.append(item[0].task_id)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    time.sleep(0.05)
    assert taken == []  # очередь не наполнена: воркер ждёт, а не уходит
    queue.prefill(ready, 1.0)
    deadline = time.time() + 5
    while len(taken) < 2 and time.time() < deadline:
        time.sleep(0.005)
    time.sleep(0.05)
    assert taken == [0, 1] and thread.is_alive()  # готовое взято, остатка ещё нет: воркер на `take`
    queue.extend(rest, 1.0)
    thread.join(5)
    assert taken == [0, 1, 10, 11] and not thread.is_alive()  # остаток встал по цене, очередь полна - воркер ушёл


def test_the_rest_merges_into_the_untaken_part_by_cost_and_a_released_queue_is_final():
    queue = pool_module._PendingTasks()  # noqa: SLF001
    first, second = _queue_items([900, 500]), _queue_items([700, 300])
    for number, (task, _frame) in enumerate(second):
        task.task_id = 10 + number
    queue.prefill(first, 1.0)
    assert queue.take(SimpleNamespace(held=()))[0].task_id == 0  # взят 900
    queue.extend(second, 1.0)
    assert [queue.take(SimpleNamespace(held=()))[0].task_id for _ in range(3)] == [10, 1, 11]  # 700, 500, 300: нерасхваченное и пришедшее по общей цене
    assert queue.take(SimpleNamespace(held=())) is None

    released = pool_module._PendingTasks()  # noqa: SLF001
    released.prefill(_queue_items([100]), 1.0)
    released.release()  # остатка не будет: очередь полна как есть
    assert released.take(SimpleNamespace(held=()))[0].task_id == 0 and released.take(SimpleNamespace(held=())) is None
