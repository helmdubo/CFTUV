"""Память подготовок воркера: подготовка пересылается один раз, ответ тот же, а промах и перезапуск — просто пересылка.

Подготовка домена alpha-независима, а воркеру на каждом шаге ширины уходил её пикл (8.65 МБ на `building`) и воркер его
разворачивал. Теперь воркер держит развёрнутые подготовки под ключом пикла, родитель шлёт ключ и пересылает пикл только там, где воркер
подготовки не держит. Здесь держится: арифметика вытеснения родителя и воркера одна; ключ — содержимое (и код); подготовка из памяти
даёт побитово тот же ответ и ту же цену, что свежеразвёрнутая, на нескольких alpha подряд (бюджет подготовки вычислением не меняется); промах
называется и лечится пересылкой; настоящие воркеры дают ответ последовательного пути, шлют ключ, а не пикл, и переживают смерть соседа.
"""

from __future__ import annotations

import pickle
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_content_key, envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv import envelope_queue_pool  # noqa: E402
from cftuv import envelope_worker_store as store  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import (  # noqa: E402
    PULL_BIG_DIVISOR,
    DomainPool,
    DomainTaskResultV1,
    DomainTaskV1,
    PoolStatsV1,
    _exchange,
    _PendingTasks,
    _PoolCounter,
    encode_frame,
    order_by_cost,
    shutdown_domain_pool,
    solve_task,
)
from cftuv.envelope_production_export import (  # noqa: E402
    PRODUCTION_POOL_BLOB_HITS,
    PRODUCTION_POOL_BLOB_MISSES,
    PRODUCTION_POOL_BLOBS_SHIPPED,
    PRODUCTION_POOL_BYTES_RECEIVED,
    PRODUCTION_POOL_BYTES_SENT,
    ProductionInputV1,
    run_production,
)
from cftuv.envelope_worker_store import (  # noqa: E402
    PreparationLruV1,
    PreparationMissing,
    PreparationStoreV1,
    blob_key,
    capture_budget,
    prepared_of,
)
from content_fixtures import cold_domain, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()
    store.STORE.clear()


@pytest.fixture
def _pool_always(monkeypatch):
    """Малая партия остаётся в родителе (порог); тесту нужен именно пул."""

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


# --------------------------------------------------------------------------
# 1. Арифметика вытеснения: одна у воркера и у зеркала родителя
# --------------------------------------------------------------------------


def test_the_lru_evicts_the_least_recently_used_by_bytes_and_refuses_what_cannot_fit():
    lru = PreparationLruV1(limit=100)

    assert lru.add("a", 40) == () and lru.add("b", 40) == ()
    assert lru.touch("a") and not lru.touch("missing")
    assert lru.add("c", 40) == ("b",)  # давняя - «b»: «a» освежена обращением
    assert lru.keys == ("a", "c") and lru.total_bytes == 80
    assert lru.add("a", 10) == () and lru.total_bytes == 50  # замена записи пересчитывает размер
    assert lru.add("huge", 101) == () and "huge" not in lru  # крупнее предела не хранится вовсе
    assert lru.discard("c") and not lru.discard("c") and lru.keys == ("a",)


@pytest.mark.parametrize("seed", range(8))
def test_the_parents_mirror_and_the_workers_store_make_the_same_decisions(seed):
    """Те же операции в том же порядке дают те же ключи в памяти: ключ без пикла уходит ровно тогда, когда воркер подготовку держит."""

    rng = random.Random(seed)
    mirror = PreparationLruV1(limit=1000)
    worker = PreparationStoreV1(limit=1000)
    names = [f"k{index}" for index in range(30)]
    sizes = {name: rng.randrange(50, 400) for name in names}
    for _ in range(400):
        key = rng.choice(names)
        if key in mirror:
            mirror.touch(key)
            assert worker.get(key) is key  # ключ без пикла: воркер подготовку держит
        else:
            mirror.add(key, sizes[key])
            worker.put(key, key, sizes[key])
        assert mirror.keys == worker.keys and mirror.total_bytes == worker.total_bytes <= 1000
        assert set(worker._objects) == set(worker.keys)  # вытесненный объект не остаётся в памяти воркера


def test_a_missing_key_is_a_named_request_for_the_blob_and_a_cleared_store_misses_everything():
    memory = PreparationStoreV1(limit=100)
    memory.put("a", object(), 10)

    memory.clear()

    with pytest.raises(PreparationMissing):
        memory.get("a")


# --------------------------------------------------------------------------
# 2. Ключ — содержимое и код
# --------------------------------------------------------------------------


def test_the_key_is_the_content_of_the_blob_and_the_code_of_the_process(monkeypatch):
    first = blob_key(b"preparation one")

    assert first and first == blob_key(b"preparation one")
    assert first != blob_key(b"preparation two")  # другая подготовка (ревизия, плотность, суженная карта, правка меша): другой ключ
    real = envelope_content_key.code_identity()
    monkeypatch.setattr(envelope_content_key, "code_identity", lambda: ("another kernel", real[1]))
    assert blob_key(b"preparation one") != first  # другой код процесса: ключ не совпадает ни с чьим


def test_without_a_key_the_preparation_is_never_remembered(monkeypatch):
    def refuse():
        raise envelope_content_key.ContentKeyUnsupported("no kernel")

    monkeypatch.setattr(envelope_content_key, "code_identity", refuse)

    assert blob_key(b"anything") == ""


# --------------------------------------------------------------------------
# 3. Подготовка из памяти равна свежеразвёрнутой
# --------------------------------------------------------------------------


def test_the_store_hands_the_preparation_out_as_it_is_and_a_coverage_leaves_its_budget_alone():
    """Покрытие считает на копии бюджета подготовки, поэтому бюджету в памяти воркера восстановление не нужно: он остаётся состоянием `PREPARE`."""

    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    root = KERNEL_SRC.parent / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request.json").read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    memory = PreparationStoreV1(limit=10**9)
    memory.put("a", prepared, 10)
    state = capture_budget(prepared)
    assert state is not None and prepared.work_budget.stage == "PREPARE"

    prices = []
    for alpha in ("0.25", "0.3", "0.25"):
        held = memory.get("a")
        assert held is prepared
        coverage = conveyor_coverage(held, alpha)
        assert coverage.outcome.value == "EXACT"
        prices.append(coverage.work_budget.spent_by_article() if alpha == "0.25" else None)
        assert capture_budget(held) == state
    assert prices[0] == prices[2]
    assert capture_budget(SimpleNamespace(work_budget=None)) is None


def test_a_blob_always_unpacks_afresh_and_only_a_bare_key_reads_the_memory():
    prepared = SimpleNamespace(marker="x")
    blob = pickle.dumps(prepared)
    inputs = SimpleNamespace(blob=blob, key="k1")
    store.STORE.clear()

    first = prepared_of(inputs)
    assert first is not prepared and first.marker == "x"
    assert prepared_of(SimpleNamespace(blob=None, key="k1")) is first  # из памяти воркера
    assert prepared_of(inputs) is not first  # пикл есть - разворот заново, память читают только задачи без пикла
    assert prepared_of(SimpleNamespace(blob=None, key="k1")).marker == "x"
    with pytest.raises(PreparationMissing):
        prepared_of(SimpleNamespace(blob=None, key="unknown"))
    prepared_of(SimpleNamespace(blob=blob, key=""))
    prepared_of(SimpleNamespace(blob=blob, key="k2"), retain=False)
    assert "k2" not in store.STORE and len(store.STORE) == 1  # без ключа и с retain=False память не трогается


def _production_task(prepared_blob, key, task_id, domain_id, patch, alpha):
    return DomainTaskV1(
        task_id,
        patch,
        domain_id,
        None,
        None,
        str(float(alpha)),
        EDGES,
        production=ProductionInputV1(prepared_blob, key=key),
    )


@pytest.mark.parametrize("lifted", (0.0, 1.0))
def test_a_remembered_preparation_answers_exactly_like_a_freshly_unpacked_one_over_several_widths(lifted):
    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    patch = ROW - 1  # при поднятом угле это домен-развёртка: нормали вершин, бюджет и память точных предикатов настоящие
    first, prepared, _revision, _request = cold_domain(source, patch, EDGES)
    blob = pickle.dumps(prepared, protocol=5)
    key = blob_key(blob)
    store.STORE.clear()
    alphas = (0.25, 0.4, 0.3, 0.25, 0.7, 0.4)

    for number, alpha in enumerate(alphas):
        fresh = solve_task(_production_task(blob, "", 100 + number, first.domain_id, patch, alpha)).production
        remembered = solve_task(
            _production_task(blob if number == 0 else None, key, number, first.domain_id, patch, alpha)
        ).production

        assert remembered == fresh, alpha  # поля сравнения: батч, оба дайджеста, счётчики (в том числе EXACT_WORK_*), нормали
        assert remembered.counters == fresh.counters and remembered.content_digest == fresh.content_digest
        assert remembered.is_materialized
    assert key in store.STORE and len(store.STORE) == 1
    assert bool(first.vertex_normals) == (lifted > 0)  # развёртка действительно прошла через память


def test_a_task_without_the_blob_that_the_worker_does_not_hold_is_a_request_not_a_failure():
    store.STORE.clear()

    reply = solve_task(_production_task(None, "nobody-sent-this", 7, "d", 1, 0.25))

    assert isinstance(reply, DomainTaskResultV1) and reply.needs_blob and not reply.ok and not reply.error and reply.task_id == 7


# --------------------------------------------------------------------------
# 4. Обмен: ключ или пикл, промах, очередь
# --------------------------------------------------------------------------


class _FakeWorker:
    """Воркер без процесса: держит подготовки так же, как настоящий, и отвечает заготовкой."""

    def __init__(self, *, forget=()):
        self.index = 0
        self.held = PreparationLruV1()
        self.memory = PreparationLruV1()
        self.forget = set(forget)
        self.frames: list[bytes] = []
        self._reply = None

    def send(self, frame):
        self.frames.append(frame)
        task = pickle.loads(frame[8:])
        inputs = task.production
        if inputs.blob is not None:
            self.memory.add(inputs.key, len(inputs.blob))
            self._reply = DomainTaskResultV1(task.task_id, queue_domain=object())
        elif inputs.key in self.memory and inputs.key not in self.forget:
            self.memory.touch(inputs.key)
            self._reply = DomainTaskResultV1(task.task_id, queue_domain=object())
        else:
            self._reply = DomainTaskResultV1(task.task_id, needs_blob=True)

    def receive(self, timeout=None):
        return self._reply


def _keyed_task(task_id, size, key):
    return DomainTaskV1(
        task_id, task_id, f"d{task_id}", None, None, "0.25", frozenset(),
        production=ProductionInputV1(b"x" * size, key=key), affinity=f"d{task_id}",
    )


def test_a_blob_goes_once_and_every_later_send_to_the_same_worker_is_a_bare_key():
    worker, counted = _FakeWorker(), _PoolCounter()
    task = _keyed_task(1, 5000, "k")
    frame = encode_frame(task)

    for _ in range(3):
        assert _exchange(worker, task, frame, counted).ok

    stats = counted.stats(0, 0.0, 0.0)
    assert [len(item) for item in worker.frames] == [len(frame), *[len(worker.frames[1])] * 2]
    assert worker.frames[1] == worker.frames[2] and len(worker.frames[1]) < 300  # ключ без пикла: сотни байтов, а не килобайты
    assert (stats.blobs_shipped, stats.blob_bytes_shipped, stats.blob_hits, stats.blob_misses) == (1, 5000, 2, 0)
    assert stats.bytes_sent == sum(len(item) for item in worker.frames)


def test_a_worker_that_lost_the_blob_asks_for_it_and_the_exchange_heals():
    worker, counted = _FakeWorker(), _PoolCounter()
    task = _keyed_task(1, 3000, "k")
    frame = encode_frame(task)
    assert _exchange(worker, task, frame, counted).ok
    worker.forget.add("k")  # воркер вытеснил подготовку, а зеркало родителя об этом не знает

    reply = _exchange(worker, task, frame, counted)

    assert reply.ok and not reply.needs_blob
    stats = counted.stats(0, 0.0, 0.0)
    assert (stats.blob_misses, stats.blobs_shipped, stats.blob_hits) == (1, 2, 0)
    worker.forget.clear()
    assert _exchange(worker, task, frame, counted).ok and counted.stats(0, 0.0, 0.0).blob_hits == 1


def test_a_failed_blob_task_is_forgotten_by_the_mirror_so_the_next_send_ships_again():
    worker, counted = _FakeWorker(), _PoolCounter()
    task = _keyed_task(1, 2000, "k")
    worker._reply = None
    original_send = worker.send

    def failing(frame):
        original_send(frame)
        worker._reply = DomainTaskResultV1(1, error="Traceback: boom")

    worker.send = failing
    assert _exchange(worker, task, encode_frame(task), counted).error
    assert "k" not in worker.held


def test_a_task_without_a_key_is_sent_whole_and_touches_no_memory():
    worker, counted = _FakeWorker(), _PoolCounter()
    task = DomainTaskV1(1, 1, "d", None, None, "0.25", frozenset(), production=ProductionInputV1(b"x" * 100, carried=True))

    worker.send = lambda frame: worker.frames.append(frame)
    worker.receive = lambda timeout=None: DomainTaskResultV1(1, queue_domain=object())
    assert _exchange(worker, task, encode_frame(task), counted).ok
    stats = counted.stats(0, 0.0, 0.0)
    assert (stats.blobs_shipped, stats.blob_hits, stats.blob_misses) == (0, 0, 0) and len(worker.held) == 0


def test_a_worker_prefers_the_task_whose_preparation_it_holds_unless_the_first_one_is_big():
    tasks = [_keyed_task(index, size, f"k{index}") for index, size in enumerate((900_000, 3000, 2000, 1500))]
    ordered = order_by_cost(tasks)
    total = sum(pool_module._frame_cost(*item) for item in ordered)
    worker = SimpleNamespace(held=PreparationLruV1())
    worker.held.add("k2", 2000)

    small_head = _PendingTasks(ordered[1:], total / PULL_BIG_DIVISOR)  # первая в очереди (3000) не крупная
    assert small_head.take(worker)[0].task_id == 2  # воркер берёт свою
    assert small_head.take(worker)[0].task_id == 1 and small_head.take(worker)[0].task_id == 3  # потом по очереди
    assert small_head.take(worker) is None

    big_head = _PendingTasks(ordered, total / PULL_BIG_DIVISOR)  # первая (900 000 байт) определяет длину прогона
    assert big_head.take(worker)[0].task_id == 0
    assert big_head.take(worker)[0].task_id == 2


def test_tasks_of_every_other_kind_keep_their_order_in_the_queue():
    plain = [DomainTaskV1(index, index, f"d{index}", "x" * (100 - index), None, "0.25", frozenset()) for index in range(5)]
    queue = _PendingTasks(order_by_cost(plain), 1.0)
    worker = SimpleNamespace(held=PreparationLruV1())

    assert [queue.take(worker)[0].task_id for _ in range(5)] == [0, 1, 2, 3, 4]


def test_a_preparation_blob_that_cannot_be_read_is_a_named_task_failure_not_a_dead_thread():
    worker, counted = _FakeWorker(), _PoolCounter()
    broken = DomainTaskResultV1(3, production=object(), prepared_blob=b"not a pickle", prepared_key="k", stored=(("k", 12),))

    reply = pool_module._received(worker, broken, counted)

    assert reply.task_id == 3 and not reply.ok and "Traceback" in reply.error
    assert "k" in worker.held  # воркер её положил: зеркало повторяет воркера, чем бы ни кончился разбор у родителя


# --------------------------------------------------------------------------
# 5. Настоящие воркеры
# --------------------------------------------------------------------------


def _production_run(bundle, controller=None, *, alpha, workers=2):
    controller = controller or EnvelopeDebugSessionController()
    run = run_production(
        controller,
        bundle,
        EDGES,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
    )
    return run, controller


def test_real_workers_are_primed_by_the_cold_press_and_then_answer_by_key_as_the_sequential_path(_pool_always):
    bundle = quad_row_bundle(ROW)
    # Последовательный прогон закрывает общий пул (workers < 2), поэтому эталоны считаются ДО того, как воркеры что-то запомнят.
    sequential = {alpha: _production_run(bundle, alpha=alpha, workers=0)[0] for alpha in (0.4, 0.5, 0.6)}
    first, controller = _production_run(bundle, alpha=0.25)  # холодное нажатие: подготовки строят воркеры и оставляют у себя

    runs = [(_production_run(bundle, controller, alpha=alpha)[0], alpha) for alpha in (0.4, 0.5, 0.6)]

    assert first.counter(PRODUCTION_POOL_BLOBS_SHIPPED) == 0  # у холодной задачи пикла подготовки нет: она его создаёт
    for run, alpha in runs:
        assert [item.placement for item in run.results] == ["worker"] * ROW
        assert all({"batch", "vertex_normals", "content_digest"}.isdisjoint(item.__dict__) for item in run.results)  # не развёрнуты
        assert run.counter(PRODUCTION_POOL_BLOB_MISSES) == 0
        assert run.counter(PRODUCTION_POOL_BLOB_HITS) + run.counter(PRODUCTION_POOL_BLOBS_SHIPPED) == ROW
        assert run.counter(PRODUCTION_POOL_BLOB_HITS) >= 1  # первый круг идёт к тем, кто подготовку держит - с самого первого шага
        assert run.counter(PRODUCTION_POOL_BYTES_RECEIVED) > 0
        assert tuple(run.results) == tuple(sequential[alpha].results)  # ответ последовательного пути, поля сравнения целиком
    cold_bytes = first.counter(PRODUCTION_POOL_BYTES_RECEIVED)
    assert cold_bytes > runs[0][0].counter(PRODUCTION_POOL_BYTES_RECEIVED)  # холодный ответ несёт подготовки, шаг - лишь вид и сжатый батч


def test_a_cold_reply_carries_the_preparation_bytes_once_and_the_worker_keeps_the_same_preparation():
    source = with_revision(quad_row_bundle(ROW, lifted_corner=1.0), "row")
    production_result, prepared, _revision, _request = cold_domain(source, ROW - 1, EDGES)
    reply = DomainTaskResultV1(0, prepared=prepared, production=production_result)
    task = SimpleNamespace(cold=object())
    store.STORE.clear()

    packed = pool_module._packed_cold_reply(task, reply)

    assert packed.prepared is None and packed.prepared_blob and packed.prepared_key == blob_key(packed.prepared_blob)
    assert packed.stored == ((packed.prepared_key, len(packed.prepared_blob)),) and packed.production is production_result
    assert store.STORE.get(packed.prepared_key) is prepared  # воркер держит ту же подготовку, пикл которой ушёл
    assert pool_module._packed_cold_reply(SimpleNamespace(cold=None), reply) is reply  # не холодная задача: ответ прежний
    assert pool_module._packed_cold_reply(task, DomainTaskResultV1(1, error="boom")) .prepared is None

    worker, counted = _FakeWorker(), _PoolCounter()
    received = pool_module._received(worker, packed, counted)

    assert packed.prepared_key in worker.held  # зеркало повторило то, что воркер положил в память
    assert received.prepared is not prepared and received.prepared.outcome == prepared.outcome
    # развёрнута из пикла, который воркер снял с ЭТОЙ подготовки: бюджет и состав те же (байты двух снятий пикла равны не обязаны - порядок множеств)
    assert capture_budget(received.prepared) == capture_budget(prepared) and received.prepared.counters == prepared.counters
    assert len(received.prepared.regions) == len(prepared.regions) and received.prepared.law_names == prepared.law_names
    assert counted.stats(0, 0.0, 0.0).unpickle_wall_seconds > 0.0
    blobs = EnvelopeDebugSessionController().preparation_blobs
    blobs.adopt(received.prepared, packed.prepared_blob, packed.prepared_key)
    assert blobs.blob_of(received.prepared) is packed.prepared_blob and blobs.key_of(received.prepared) == packed.prepared_key


def test_a_worker_killed_between_presses_costs_one_task_and_its_replacement_just_misses(_pool_always):
    bundle = quad_row_bundle(ROW)
    references = {alpha: _production_run(bundle, alpha=alpha, workers=0)[0] for alpha in (0.5, 0.6)}  # до воркеров: workers=0 закрывает пул
    _first, controller = _production_run(bundle, alpha=0.25)
    _production_run(bundle, controller, alpha=0.4)  # воркеры держат подготовки
    pool = pool_module.get_domain_pool(2)
    victim = pool._workers[0]
    victim.process.kill()
    victim.process.wait()

    healed, _ = _production_run(bundle, controller, alpha=0.5)

    assert tuple(healed.results) == tuple(references[0.5].results)  # потеря названа и досчитана, ответ тот же
    assert victim.dead and victim not in pool_module.get_domain_pool(2)._workers
    again, _ = _production_run(bundle, controller, alpha=0.6)  # пул довёл воркеров до заказанного: новый воркер просто ничего не держит
    assert pool_module.get_domain_pool(2).worker_count == 2
    assert tuple(again.results) == tuple(references[0.6].results)
    assert again.counter(PRODUCTION_POOL_BLOB_MISSES) == 0


def test_the_pool_run_reports_its_transfer_statistics():
    pool = DomainPool(2)
    try:
        run = pool.run([DomainTaskV1(0, 0, "d0", "x", None, "0.25", frozenset())])
    finally:
        pool.close()

    assert isinstance(run.stats, PoolStatsV1) and run.stats.bytes_sent > 0
    assert run.stats.blobs_shipped == 0 and run.stats.blob_hits == 0 and run.stats.bytes_received > 0
