"""Динамика пула: когда воркеру уходит следующая задача и с чего стартует прогон (POOL-DYN).

Утверждение одно: пул меняет КОГДА воркер получает работу, а не ЧТО он отвечает и в каком порядке это видно снаружи. Поэтому здесь не ускорение (оно измерено на поле, см. `DECISIONS.md`), а
что держит ответ прежним при любом порядке прихода ответов:

* EARLY_DISPATCH_BEFORE_UNPICKLE_V1 - следующая ХОЛОДНАЯ задача уходит воркеру сразу после БАЙТОВ ответа на предыдущую, до их разбора. Выбор задачи тот же, что у `take` (очередь из одних холодных
  задач отдаёт голову любому воркеру), задач у воркера по-прежнему одна в работе, а принятие ответов родителем идёт по номеру задачи (`results.get(task_id)`), а не по приходу.
* HEAD_STREAMED_FIRST_ROUND_V1 - первый круг стартует по отбору кандидатов (число цепочек, без кодирования всех кадров), остаток упорядочивается, пока воркеры считают; порядок ВСЕХ задач тот же, что у полного
  порядка, когда точный первый круг лежит среди кандидатов, и отказ упорядочения не вешает воркеров.
* Названные потери: умерший воркер с задачей, ушедшей вперёд, и отмена посреди прогона не теряют задачу молча.

Воркеры здесь - настоящие подпроцессы с подставным циклом (ответ по номеру задачи после паузы, заданной в `alpha_text`): ядро им не нужно, разбор кадров и труб - настоящие.
"""

from __future__ import annotations

import io
import pickle
import random
import sys
import threading
import time
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv.envelope_export_input import HostExportInputV1  # noqa: E402
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPool,
    DomainPoolRunV1,
    DomainTaskV1,
    encode_frame,
    order_by_cost,
    read_frame_sized,
    shutdown_domain_pool,
    solve_task,
    split_first_round,
)

ROW = 5

#: Подставной цикл воркера: кадры настоящие, домена нет. Пауза задачи - её `alpha_text` (секунды), `die` - воркер умирает, не ответив.
STUB_WORKER = """
pool = importlib.import_module(spec["package"] + ".envelope_domain_pool")
import time as _time
channel = os.fdopen(os.dup(1), "wb")
os.dup2(2, 1)
sys.stdout = sys.stderr
source = sys.stdin.buffer
pool.write_frame(channel, ("ready", os.getpid(), {}))
while True:
    task = pool.read_frame(source)
    if task is None:
        break
    if task.alpha_text == "die":
        os._exit(3)
    _time.sleep(float(task.alpha_text))
    pool.write_frame(channel, pool.DomainTaskResultV1(
        task.task_id, snapshot=("echo", task.patch_id, task.domain_id), export_timings=((os.getpid(), _time.perf_counter()),)))
"""

LAST_LINE = 'importlib.import_module(spec["package"] + ".envelope_domain_pool").worker_main()'


@pytest.fixture(autouse=True)
def _no_pool_outlives_the_test():
    yield
    shutdown_domain_pool()


@pytest.fixture
def _stub_workers(monkeypatch):
    assert LAST_LINE in pool_module._BOOTSTRAP
    monkeypatch.setattr(pool_module, "_BOOTSTRAP", pool_module._BOOTSTRAP.replace(LAST_LINE, STUB_WORKER))


def _task(task_id, delay="0.0", *, cold="stub", weight=0):
    return DomainTaskV1(task_id, task_id, f"d{task_id}", "x" * weight, None, delay, frozenset(), cold=cold)


# --------------------------------------------------------------------------
# 1. Кадр: байты читаются до разбора
# --------------------------------------------------------------------------


class _Marker:
    """Объект, чей разбор оставляет след: порядок «байты прочитаны - разбор начат» виден по списку."""

    log: list = []

    def __reduce__(self):
        return (_Marker._unpickled, ())

    @staticmethod
    def _unpickled():
        _Marker.log.append("unpickled")
        return "value"


def test_the_bytes_of_a_frame_are_read_before_it_is_unpickled_and_the_hook_time_is_not_unpickle_time():
    stream = io.BytesIO(encode_frame(_Marker()))
    _Marker.log.clear()

    value, length, cpu, wall = read_frame_sized(stream, lambda: (_Marker.log.append("between"), time.sleep(0.05)))

    assert _Marker.log == ["between", "unpickled"]
    assert value == "value" and length == len(encode_frame(_Marker())) - pool_module.FRAME_HEADER.size
    assert wall < 0.05  # разбор измерен после вызова, а не вместе с ним
    assert read_frame_sized(stream) is None


def test_a_frame_cut_in_the_middle_is_an_error_and_never_reaches_the_hook():
    frame = encode_frame(("ready", 7))
    calls = []
    with pytest.raises(EOFError):
        read_frame_sized(io.BytesIO(frame[:-2]), lambda: calls.append(1))
    assert calls == []


# --------------------------------------------------------------------------
# 2. Очередь: вперёд ответа выбирается только то, что выбрал бы `take`
# --------------------------------------------------------------------------


def _items(*kinds):
    out = []
    for index, kind in enumerate(kinds):
        task = DomainTaskV1(index, index, f"d{index}", None, None, "0", frozenset(), cold="c" if kind == "cold" else None, coverage=None if kind == "cold" else object())
        out.append((task, b"x"))
    return out


def test_take_ahead_gives_the_head_of_a_queue_of_cold_tasks_exactly_as_take_would():
    ahead, plain = pool_module._PendingTasks(_items("cold", "cold", "cold"), 1.0), pool_module._PendingTasks(_items("cold", "cold", "cold"), 1.0)
    worker = type("W", (), {"held": {}})()

    assert [ahead.take_ahead()[0].task_id for _ in range(3)] == [plain.take(worker)[0].task_id for _ in range(3)] == [0, 1, 2]
    assert ahead.take_ahead() is None and plain.take(worker) is None


def test_take_ahead_declines_while_a_task_that_depends_on_the_workers_memory_is_queued():
    pending = pool_module._PendingTasks(_items("cold", "warm", "cold"), 1.0)
    worker = type("W", (), {"held": {}})()

    assert pending.take_ahead() is None  # теплая задача где-то в очереди: выбор зависит от памяти воркера
    assert pending.take(worker)[0].task_id == 0
    assert pending.take_ahead() is None
    assert pending.take(worker)[0].task_id == 1  # тёплую забрали...
    assert pending.take_ahead()[0].task_id == 2  # ...и очередь из одних холодных снова отдаёт голову вперёд


# --------------------------------------------------------------------------
# 3. Ответ не зависит от порядка прихода ответов и от момента отправки
# --------------------------------------------------------------------------


def _record_sends(monkeypatch):
    sends = []
    original = pool_module._Worker.send

    def recording(self, frame):
        sends.append(threading.current_thread().name)
        return original(self, frame)

    monkeypatch.setattr(pool_module._Worker, "send", recording)
    return sends


def _delays(seed, count):
    generator = random.Random(seed)
    return [round(generator.uniform(0.001, 0.03), 4) for _ in range(count)]


def _run_stub(pool, delays):
    tasks = [_task(index, str(delay), weight=index % 7) for index, delay in enumerate(delays)]
    run = pool.run(tasks)
    sequence = [(task.task_id, run.results[task.task_id].snapshot) for task in tasks]
    arrival = tuple(sorted(run.results, key=lambda task_id: run.results[task_id].export_timings[0][1]))
    return run, sequence, arrival


def test_a_cold_batch_gives_the_same_results_whatever_the_arrival_order_and_whether_tasks_go_ahead(_stub_workers, monkeypatch):
    sends = _record_sends(monkeypatch)
    pool = DomainPool(3)
    pool.ensure_started()
    count = 48
    outcomes = {}
    early_sends = {}
    for early in (True, False):
        monkeypatch.setattr(pool_module, "EARLY_DISPATCH", early)
        for seed in (1, 2, 3):
            sends.clear()
            run, sequence, arrival = _run_stub(pool, _delays(seed, count))
            outcomes[(early, seed)] = (sequence, arrival, run.stats)
            early_sends[(early, seed)] = sum("_pump_frames" in name for name in sends)
            assert len(sends) == count  # каждая задача ушла ровно один раз
    pool.close()

    baseline = outcomes[(False, 1)][0]
    assert baseline == [(index, ("echo", index, f"d{index}")) for index in range(count)]
    # то, что видно снаружи: ответы по номеру задачи и пересылки - те же при любых паузах, порядке прихода и моменте отправки
    for key, (sequence, _arrival, stats) in outcomes.items():
        assert sequence == baseline, key
        assert stats.bytes_sent == outcomes[(False, key[1])][2].bytes_sent, key  # те же кадры (паузы разных зёрен пишутся разной длиной, поэтому сравнение внутри зерна)
        assert (stats.blob_hits, stats.blobs_shipped, stats.blob_misses, stats.tasks_to_owner, stats.tasks_stolen) == (0, 0, 0, 0, 0), key
    # тест не пустой: порядки прихода различны, а вперёд уходит большинство задач ровно тогда, когда это включено
    assert len({outcomes[(early, seed)][1] for early in (True, False) for seed in (1, 2, 3)}) >= 3
    assert all(early_sends[(False, seed)] == 0 for seed in (1, 2, 3))
    assert all(early_sends[(True, seed)] >= count // 2 for seed in (1, 2, 3)), early_sends


def test_a_batch_with_a_warm_task_is_dispatched_by_the_workers_own_thread_only(_stub_workers, monkeypatch):
    sends = _record_sends(monkeypatch)
    pool = DomainPool(2)
    tasks = [_task(index, "0.005") for index in range(12)]
    tasks.append(DomainTaskV1(12, 12, "d12", None, None, "0.005", frozenset(), coverage=object()))
    try:
        run = pool.run(tasks)
    finally:
        pool.close()

    assert sorted(run.results) == list(range(13))
    assert not any("_pump_frames" in name for name in sends)  # теплая задача в очереди: выбор может зависеть от памяти воркера, вперёд ответа не выбирается


# --------------------------------------------------------------------------
# 3a. Первый круг по отбору кандидатов
# --------------------------------------------------------------------------


def _export_task(task_id, chains, weight=0, delay="0.0"):
    export = HostExportInputV1("rev", None, tuple(range(chains)), 0.1, "request", None)
    return DomainTaskV1(task_id, task_id, f"d{task_id}", "x" * weight, None, delay, frozenset(), export=export, cold="stub")


def _aligned_batch(count):
    """Число цепочек растёт вместе с размером кадра, как на поле: отбор кандидатов видит тяжёлых."""

    generator = random.Random(7)
    sizes = generator.sample(range(1, 400), count)
    return [_export_task(index, sizes[index], weight=sizes[index] * 40) for index in range(count)]


def test_the_first_round_is_the_exact_heaviest_of_the_candidates_and_the_rest_is_everything_else():
    tasks = _aligned_batch(60)
    workers = 4

    head, rest = split_first_round(tasks, workers)

    full = order_by_cost(tasks)
    assert [task.task_id for task, _ in head] == [task.task_id for task, _ in full[:workers]]  # точный первый круг лежал среди кандидатов: порядок тот же
    assert all(frame == encode_frame(task) for task, frame in head)
    assert {task.task_id for task, _ in head}.isdisjoint(task.task_id for task in rest)
    assert len(head) + len(rest) == len(tasks) and [task.task_id for task in rest] == sorted(task.task_id for task in rest)  # остаток в порядке задач, без кадров
    # весь порядок прежний: первый круг, затем остаток по полному порядку
    assert [task.task_id for task, _ in head] + [task.task_id for task, _ in order_by_cost(rest)] == [task.task_id for task, _ in full]


def test_a_candidate_miss_is_a_launch_order_matter_and_loses_no_task():
    # у тяжёлых задач мало цепочек: отбор кандидатов их не видит
    tasks = [_export_task(index, chains=100 - index, weight=index * 500) for index in range(60)]
    workers = 4

    head, rest = split_first_round(tasks, workers)

    assert len(head) == workers and len(head) + len(rest) == len(tasks)
    assert sorted([task.task_id for task, _ in head] + [task.task_id for task in rest]) == list(range(60))
    full = order_by_cost(tasks)
    assert [task.task_id for task, _ in head] != [task.task_id for task, _ in full[:workers]]  # тест не пустой: кандидаты промахнулись


def test_a_streamed_batch_answers_like_a_full_order_and_orders_the_rest_after_the_first_round_left(_stub_workers, monkeypatch):
    calls = []
    original = pool_module.order_by_cost

    def recording(tasks):
        tasks = list(tasks)
        calls.append(len(tasks))
        return original(tasks)

    monkeypatch.setattr(pool_module, "order_by_cost", recording)
    sends = _record_sends(monkeypatch)
    pool = DomainPool(3)
    pool.ensure_started()
    count = 60
    tasks = [_export_task(index, chains=1 + (index * 7) % 50, weight=(1 + (index * 7) % 50) * 30, delay="0.004") for index in range(count)]
    try:
        streamed = pool.run(tasks)
        calls_streamed = list(calls)
        calls.clear()
        monkeypatch.setattr(pool_module, "STREAMED_HEAD", False)
        full = pool.run(tasks)
    finally:
        pool.close()

    # первый вызов - кандидаты (2 * 3), второй - остаток; без отбора - один вызов на всё
    assert calls_streamed == [2 * 3, count - 3] and calls == [count]
    assert streamed.stats.head_streamed == 1 and full.stats.head_streamed == 0
    expected = {index: ("echo", index, f"d{index}") for index in range(count)}
    assert {task_id: result.snapshot for task_id, result in streamed.results.items()} == expected == {task_id: result.snapshot for task_id, result in full.results.items()}
    assert streamed.stats.bytes_sent == full.stats.bytes_sent  # те же кадры ушли ровно по разу
    assert streamed.stats.head_misses == 0  # цепочки и размер кадра согласованы: первый круг точный
    assert len(sends) == 2 * count


def test_a_failed_ordering_of_the_rest_releases_the_workers_and_reaches_the_caller(_stub_workers, monkeypatch):
    original = pool_module.order_by_cost
    calls = []

    def failing(tasks):
        calls.append(1)
        if len(calls) == 2:
            raise RuntimeError("cannot encode the rest")
        return original(tasks)

    monkeypatch.setattr(pool_module, "order_by_cost", failing)
    pool = DomainPool(3)
    pool.ensure_started()
    tasks = [_export_task(index, chains=1 + index % 9, weight=100, delay="0.01") for index in range(40)]
    try:
        started = time.perf_counter()
        with pytest.raises(RuntimeError, match="cannot encode the rest"):
            pool.run(tasks)
        assert time.perf_counter() - started < 5.0  # воркеры не повисли на очереди, которой не будет
        monkeypatch.setattr(pool_module, "order_by_cost", original)
        again = pool.run([_task(index, "0.001") for index in range(6)])
        assert sorted(again.results) == list(range(6)) and pool.worker_count == 3  # трубы в порядке: ответы первого круга прочитаны
    finally:
        pool.close()


# --------------------------------------------------------------------------
# 4. Потери названы
# --------------------------------------------------------------------------


def test_a_worker_that_dies_names_the_running_task_and_the_one_sent_ahead_and_the_others_are_answered(_stub_workers):
    pool = DomainPool(3)
    count = 30
    delays = [0.01] * count
    tasks = [_task(index, str(delay)) for index, delay in enumerate(delays)]
    tasks[17] = _task(17, "die")
    try:
        run = pool.run(tasks)
        assert pool.worker_count == 2
    finally:
        pool.close()

    assert sorted(run.results) == list(range(count))  # ни одна задача не пропала молча
    dead = {task_id for task_id, result in run.results.items() if result.error}
    assert 17 in dead and all("worker died" in run.results[task_id].error for task_id in dead)
    assert len(dead) <= 2  # умерший воркер держал не больше одной задачи в работе и не больше одной, ушедшей вперёд
    assert all(run.results[task_id].snapshot == ("echo", task_id, f"d{task_id}") for task_id in set(run.results) - dead)


def test_cancel_lets_the_tasks_in_flight_finish_and_takes_no_new_ones(_stub_workers):
    pool = DomainPool(2)
    cancel = threading.Event()
    count = 60
    tasks = [_task(index, "0.02") for index in range(count)]
    pool.ensure_started()
    timer = threading.Timer(0.3, cancel.set)
    try:
        timer.start()
        started = time.perf_counter()
        run = pool.run(tasks, cancel=cancel)
        seconds = time.perf_counter() - started
    finally:
        timer.cancel()
        pool.close()

    assert 0 < len(run.results) < count
    assert seconds < 0.3 + 0.5  # идущие дочитываются (две по 20 мс), новые не берутся
    assert all(not result.error and result.snapshot == ("echo", task_id, f"d{task_id}") for task_id, result in run.results.items())


# --------------------------------------------------------------------------
# 5. Принятие ответов идёт по номеру задачи, а не по приходу
# --------------------------------------------------------------------------


class _NoIterationDict(dict):
    """Ответы пула, перебрать которые нельзя: потребитель обязан спрашивать по номеру задачи (`results.get(task_id)`)."""

    def _refuse(self, *args, **kwargs):
        raise AssertionError("the consumer iterated the pool replies instead of asking for them by task id")

    __iter__ = items = values = keys = popitem = _refuse


class _ShufflingPool:
    """Пул без подпроцессов: ответы те же, что у `solve_task`, но кладёт их в словарь в ПЕРЕМЕШАННОМ порядке (порядок прихода, как у настоящего пула), и перебрать его нельзя."""

    requested = 2

    def __init__(self, seed):
        self.seed = seed

    def run(self, tasks, cancel=None):
        pairs = [(task.task_id, pickle.loads(pickle.dumps(solve_task(task)))) for task, _frame in order_by_cost(tasks)]
        random.Random(self.seed).shuffle(pairs)
        results = _NoIterationDict()
        for task_id, reply in pairs:
            dict.__setitem__(results, task_id, reply)
        return DomainPoolRunV1(results, self.requested)


def test_the_parent_adopts_replies_in_task_order_whatever_order_the_pool_returned_them(monkeypatch):
    from cftuv import envelope_production_export as export_module
    from cftuv import envelope_queue_pool
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController
    from cftuv.envelope_production_export import run_production
    from envelope_fixture_bundles import quad_row_bundle

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    bundle = quad_row_bundle(ROW)

    state = {"adopted": [], "remembered": []}
    original_adopt = export_module._adopt_cold
    original_remember = EnvelopeDebugSessionController.remember_production_result

    def adopt(run, entry, reply, placement):
        state["adopted"].append(entry.domain_id)
        return original_adopt(run, entry, reply, placement)

    def remember(self, key, result):
        state["remembered"].append(key)
        return original_remember(self, key, result)

    monkeypatch.setattr(export_module, "_adopt_cold", adopt)
    monkeypatch.setattr(EnvelopeDebugSessionController, "remember_production_result", remember)

    def press(seed):
        state["adopted"], state["remembered"] = [], []
        monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": _ShufflingPool(seed))
        run = run_production(EnvelopeDebugSessionController(), bundle, frozenset(range(ROW)), 0.25, source_object_key="object", source_data_key="mesh", density=None, workers=2)
        return list(state["adopted"]), list(state["remembered"]), tuple(run.results)

    first, second, third = press(1), press(2), press(3)

    assert len(first[0]) == ROW and first[1]
    assert first == second == third  # последовательность принятия, порядок записи в кэш сессии (а с ним и вытеснения) и сами результаты
