"""Фоновое превью alpha: пауза, слияние, один полёт, устаревшее, отмена и РАВЕНСТВО ответа.

Утверждения среза:

1. калбэк ползунка только ЗАКАЗЫВАЕТ (микросекунды) и взводит таймер; считает таймер;
2. пауза после последнего изменения; замещённые значения не считаются, а считаются в `coalesced`;
3. в полёте не больше одного счёта; новое значение отменяет полёт; результат устаревшего
   значения выброшен и назван (`stale`), результат последнего применён;
4. смена цели за время счёта (другая тёплая сессия, исчезнувший объект) — `invalid` с причиной;
5. поток счёта не читает и не пишет данные Blender (`bpy` в нём не трогается);
6. ответ после перетаскивания ПОБИТОВО тот же, что у синхронного покрытия на тех же подготовках,
   в родителе и в настоящих воркерах пула;
7. кнопки и сброс сессии останавливают полёт ДО своей работы (`quiesce`), пул не принимает два
   прогона разом, отмена не даёт воркерам брать новые задачи.

Без Blender: планировщик получает фальшивые часы, таймеры и задачи; интеграция с ядром идёт на
фикстуре `quad_row_bundle`, а `bpy` подменён там, где склейка его читает.
"""

from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_alpha_preview_gp as gp  # noqa: E402
from cftuv import envelope_debug_renderer  # noqa: E402  (до подмены bpy: модуль читает его при импорте)
from cftuv import envelope_queue_pool  # noqa: E402
from cftuv.envelope_alpha_preview import (  # noqa: E402
    AlphaPreviewScheduler,
    PreviewCancelled,
    PreviewUnavailable,
    ThreadedPreviewJob,
)
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import (  # noqa: E402
    EnvelopeDebugSessionController,
    evaluate_envelope_debug_staged,
    remember_queue_session,
)
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPool,
    DomainTaskV1,
    get_domain_pool,
    shutdown_domain_pool,
)
from cftuv.envelope_queue_export import (  # noqa: E402
    POOL_COVERAGE_DISPATCHED,
    POOL_TASK_FALLBACK,
    CoverageCancelled,
    queue_scene_payload,
    recompute_queue_coverage,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# Фальшивки планировщика
# --------------------------------------------------------------------------


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeTimers:
    """`bpy.app.timers`: список зарегистрированных функций; `fire` — один заход главного цикла."""

    def __init__(self) -> None:
        self.registered: list = []
        self.first_intervals: list[float] = []

    def register(self, function, first_interval):
        self.registered.append(function)
        self.first_intervals.append(first_interval)

    def is_registered(self, function) -> bool:
        return function in self.registered

    def fire(self):
        """Вызывает таймер; вернувшая `None` функция снимается, как это делает Blender."""

        for function in list(self.registered):
            if function() is None:
                self.registered.remove(function)


class FakeJob:
    def __init__(self, value=None, *, error=None, finished=False):
        self.value = value
        self.error = error
        self.finished = finished
        self.cancelled = False
        self.joined: list = []
        self.seconds = 0.25

    def cancel(self):
        self.cancelled = True

    def join(self, timeout=None):
        self.joined.append(timeout)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value


class Harness:
    """Планировщик на фальшивках с записью вызовов `begin`/`apply`."""

    def __init__(self, *, valid=None, begin_error=None, apply_error=None, **extra):
        self.clock = FakeClock()
        self.timers = FakeTimers()
        self.jobs: list[FakeJob] = []
        self.begun: list = []
        self.applied: list = []
        self.statuses: list[str] = []
        self._begin_error = begin_error
        self._apply_error = apply_error

        def begin(request):
            self.begun.append(request)
            if self._begin_error is not None:
                raise self._begin_error
            job = FakeJob(value=("scene", request.alpha))
            self.jobs.append(job)
            return job

        def apply(request, job, value):
            if self._apply_error is not None:
                raise self._apply_error
            self.applied.append((request.alpha, value))

        self.scheduler = AlphaPreviewScheduler(
            begin=begin,
            apply=apply,
            valid=valid,
            timers=self.timers,
            clock=self.clock,
            sleep=lambda seconds: self.clock.advance(seconds),
            on_change=self.statuses.append,
            **extra,
        )

    def at(self, seconds: float):
        self.clock.now = seconds
        return self

    def step(self):
        return self.scheduler.step()


# --------------------------------------------------------------------------
# 1-2. Заказ, пауза, слияние
# --------------------------------------------------------------------------


def test_a_change_only_records_the_order_and_arms_the_timer_once():
    h = Harness()

    h.at(0.0).scheduler.request(0.30)
    h.at(0.01).scheduler.request(0.31)

    assert h.begun == []  # заказ ничего не считает
    assert len(h.timers.registered) == 1  # таймер один на все изменения
    assert h.timers.first_intervals == [h.scheduler._debounce]
    counters = h.scheduler.counters
    assert (counters.requested, counters.coalesced, counters.started) == (2, 1, 0)
    assert h.scheduler.busy and not h.scheduler.in_flight


def test_the_timer_callback_is_one_stable_object_for_is_registered():
    """`bpy.app.timers.is_registered` сравнивает объект функции: связка метода обязана быть одной."""

    h = Harness()
    h.scheduler.request(0.3)
    assert h.scheduler._callback is h.scheduler._callback
    assert h.timers.is_registered(h.scheduler._callback)


def test_the_debounce_waits_for_a_quiet_period_after_the_last_change():
    h = Harness()
    h.at(0.00).scheduler.request(0.30)
    h.at(0.10).step()
    assert h.begun == []
    h.at(0.10).scheduler.request(0.31)  # новое изменение сдвигает срок
    h.at(0.20).step()
    assert h.begun == []
    h.at(0.24).step()
    assert h.begun == []
    h.at(0.26).step()
    assert [item.alpha for item in h.begun] == [0.31]


def test_coalescing_computes_only_the_latest_value_and_counts_the_dropped_ones():
    h = Harness()
    for index, alpha in enumerate((0.30, 0.31, 0.32, 0.33, 0.34)):
        h.at(index * 0.02).scheduler.request(alpha)
    h.at(1.0).step()

    assert [item.alpha for item in h.begun] == [0.34]
    counters = h.scheduler.counters
    assert (counters.requested, counters.coalesced, counters.started) == (5, 4, 1)


def test_the_result_of_the_latest_value_is_applied_once_with_its_cost():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].finished = True
    h.at(0.4).step()

    assert h.applied == [(0.30, ("scene", 0.30))]
    counters = h.scheduler.counters
    assert (counters.started, counters.applied, counters.stale) == (1, 1, 0)
    assert h.scheduler.last_applied.alpha == 0.30
    assert h.scheduler.last_applied.latency_seconds == pytest.approx(0.4)
    assert h.scheduler.status_text.startswith("ready alpha=0.3")
    assert h.step() is None  # тишина: таймер снимается


def test_the_timer_goes_quiet_when_idle_and_is_armed_again_by_the_next_change():
    h = Harness()
    h.at(0.0).scheduler.request(0.3)
    h.at(0.2).timers.fire()
    h.jobs[0].finished = True
    h.at(0.3).timers.fire()
    assert h.timers.registered == []  # вернулся None -> Blender снял функцию

    h.at(1.0).scheduler.request(0.4)
    assert len(h.timers.registered) == 1


# --------------------------------------------------------------------------
# 3. Один полёт, отмена, устаревшее
# --------------------------------------------------------------------------


def test_a_newer_value_cancels_the_flight_and_the_stale_result_is_discarded_and_named():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()  # старт A
    first = h.jobs[0]
    h.at(0.3).scheduler.request(0.40)  # B отменяет A
    assert first.cancelled

    # A закончился ПОЛНЫМ результатом, но значение устарело: не применяется.
    first.finished = True
    h.at(0.35).step()
    assert h.applied == []
    assert h.scheduler.counters.stale == 1
    assert h.scheduler.in_flight is False

    h.at(0.46).step()  # срок B вышел: старт B
    assert [item.alpha for item in h.begun] == [0.30, 0.40]
    h.jobs[1].finished = True
    h.at(0.5).step()
    assert h.applied == [(0.40, ("scene", 0.40))]
    counters = h.scheduler.counters
    assert (counters.applied, counters.stale, counters.coalesced) == (1, 1, 0)
    assert h.scheduler.status_text.endswith("| superseded 1")


def test_there_is_never_more_than_one_flight_even_after_the_next_deadline():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.at(0.3).scheduler.request(0.40)
    h.at(5.0).step()  # срок B давно вышел, но A ещё в полёте
    assert len(h.begun) == 1
    assert h.scheduler.in_flight


def test_a_cancelled_flight_that_raises_cancelled_is_counted_as_stale_not_as_a_failure():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.at(0.3).scheduler.request(0.40)
    h.jobs[0].error = PreviewCancelled("stopped between domains")
    h.jobs[0].finished = True
    h.at(0.35).step()

    counters = h.scheduler.counters
    assert (counters.stale, counters.failed) == (1, 0)
    assert h.applied == []


def test_a_drag_of_many_values_ends_with_exactly_one_applied_result_of_the_last_value():
    h = Harness()
    values = [0.30 + 0.005 * index for index in range(40)]
    for index, alpha in enumerate(values):
        h.at(index * 0.016).scheduler.request(alpha)
        h.step()
    h.at(2.0).step()  # пауза после последнего значения: старт
    h.jobs[-1].finished = True
    h.at(2.1).step()

    assert [item[0] for item in h.applied] == [values[-1]]
    counters = h.scheduler.counters
    assert counters.requested == 40
    assert counters.applied == 1
    # Ни один заказ не потерян: каждый либо применён, либо замещён, либо назван выброшенным.
    assert counters.requested == counters.applied + counters.coalesced + counters.stale


# --------------------------------------------------------------------------
# 4. Цель, ошибки, названные исходы
# --------------------------------------------------------------------------


def test_a_target_that_changed_under_the_flight_discards_the_result_with_a_reason():
    h = Harness(valid=lambda request, job: "warm session was replaced while computing")
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].finished = True
    h.at(0.3).step()

    assert h.applied == []
    assert h.scheduler.counters.invalid == 1
    assert "warm session was replaced" in h.scheduler.status_text


def test_a_start_that_cannot_happen_is_named_and_counted():
    h = Harness(begin_error=PreviewUnavailable("no warm queue session: press Build"))
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()

    assert h.scheduler.counters.unavailable == 1
    assert h.scheduler.status_text == "no warm queue session: press Build"
    assert not h.scheduler.busy


def test_a_failed_compute_is_counted_named_and_printed(capsys):
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].error = ValueError("boom")
    h.jobs[0].finished = True
    h.at(0.3).step()

    assert h.scheduler.counters.failed == 1
    assert "ValueError" in h.scheduler.status_text
    assert "AlphaPreview" in capsys.readouterr().err


def test_a_failed_apply_is_counted_and_named_and_the_timer_survives(capsys):
    h = Harness(apply_error=RuntimeError("write failed"))
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].finished = True
    h.at(0.3).step()

    assert h.scheduler.counters.failed == 1 and h.scheduler.counters.applied == 0
    assert "apply" in h.scheduler.status_text and "RuntimeError" in h.scheduler.status_text
    assert "AlphaPreview" in capsys.readouterr().err
    # Следующий заказ работает.
    h._apply_error = None
    h.at(1.0).scheduler.request(0.31)
    h.at(1.2).step()
    h.jobs[-1].finished = True
    h.at(1.3).step()
    assert h.scheduler.counters.applied == 1


def test_an_apply_that_finds_no_target_is_an_invalid_result_not_a_failure():
    h = Harness(apply_error=PreviewUnavailable("Envelope debug object is gone"))
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].finished = True
    h.at(0.3).step()

    assert h.scheduler.counters.invalid == 1 and h.scheduler.counters.failed == 0
    assert "object is gone" in h.scheduler.status_text


def test_a_note_names_the_missing_warm_session_without_arming_a_timer():
    h = Harness()
    h.scheduler.note_unavailable("no warm queue session")
    assert h.scheduler.status_text == "no warm queue session"
    assert h.scheduler.counters.unavailable == 1
    assert h.timers.registered == []


def test_the_status_line_names_computing_ready_and_superseded():
    h = Harness()
    h.at(0.0).scheduler.request(0.31)
    assert h.scheduler.status_text == "computing alpha=0.31..."
    h.at(0.01).scheduler.request(0.32)
    assert h.scheduler.status_text == "computing alpha=0.32... | superseded 1"
    h.at(0.3).step()
    h.jobs[0].finished = True
    h.at(0.4).step()
    assert h.scheduler.status_text.startswith("ready alpha=0.32")
    assert h.scheduler.status_text.endswith("| superseded 1")
    # Следующее перетаскивание начинает счёт замещённых заново.
    h.at(1.0).scheduler.request(0.33)
    assert h.scheduler.status_text == "computing alpha=0.33..."
    assert h.statuses[0] == "computing alpha=0.31..."  # панель получила смену строки


def test_a_failing_redraw_hook_never_breaks_the_scheduler(capsys):
    h = Harness()

    def broken(_status):
        raise RuntimeError("no windows")

    h.scheduler._on_change = broken
    h.at(0.0).scheduler.request(0.3)
    assert h.scheduler.busy
    assert "status redraw failed" in capsys.readouterr().err


# --------------------------------------------------------------------------
# 7. Отмена перед тяжёлым синхронным путём
# --------------------------------------------------------------------------


def test_quiesce_drops_the_order_stops_the_flight_and_waits_for_it():
    h = Harness(quiesce_timeout=7.0)
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    job = h.jobs[0]
    h.at(0.3).scheduler.request(0.40)

    job.finished = True  # join вернулся бы, когда поток закончил
    assert h.scheduler.quiesce("Build Decal Mesh") is True

    assert job.cancelled and job.joined == [7.0]
    assert not h.scheduler.busy
    assert h.scheduler.counters.cancelled == 1
    assert h.scheduler.status_text.startswith("cancelled: Build Decal Mesh")
    assert h.applied == []
    assert h.step() is None
    assert h.scheduler.quiesce("again") is False  # пустой заказ ничего не пишет


def test_supersede_does_not_wait_and_the_late_result_is_dropped_without_a_second_count():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    job = h.jobs[0]

    assert h.scheduler.supersede("warm session dropped") is True
    assert job.cancelled and job.joined == []
    job.finished = True  # полный результат приходит позже
    h.at(0.3).step()

    assert h.applied == []
    counters = h.scheduler.counters
    assert (counters.cancelled, counters.stale) == (1, 0)


def test_settle_skips_the_debounce_and_drains_to_the_applied_result():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.01).scheduler.request(0.35)

    real_begin = h.scheduler._begin

    def begin_then_finish(request):
        job = real_begin(request)
        job.finished = True
        return job

    h.scheduler._begin = begin_then_finish
    h.scheduler.settle(timeout=5.0)

    assert [item[0] for item in h.applied] == [0.35]
    assert not h.scheduler.busy


def test_settle_gives_up_by_name_when_the_flight_never_returns():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    with pytest.raises(TimeoutError):
        h.scheduler.settle(timeout=0.5)


# --------------------------------------------------------------------------
# Поток счёта
# --------------------------------------------------------------------------


def test_the_job_computes_off_the_main_thread_and_returns_the_value():
    seen = {}

    def compute(cancel):
        seen["thread"] = threading.current_thread()
        return 42

    job = ThreadedPreviewJob(compute)
    job.join(5.0)
    assert job.finished and job.result() == 42
    assert seen["thread"] is not threading.main_thread()
    assert job.seconds >= 0.0


def test_the_cancel_event_reaches_the_compute_and_its_cancellation_is_the_result():
    started = threading.Event()

    def compute(cancel):
        started.set()
        while not cancel.is_set():
            time.sleep(0.001)
        raise PreviewCancelled("stopped")

    job = ThreadedPreviewJob(compute)
    assert started.wait(5.0)
    assert not job.finished
    with pytest.raises(RuntimeError):
        job.result()
    job.cancel()
    job.join(5.0)
    with pytest.raises(PreviewCancelled):
        job.result()


def test_a_compute_error_travels_to_the_result_not_to_the_thread_hook():
    job = ThreadedPreviewJob(lambda cancel: (_ for _ in ()).throw(KeyError("domain")))
    job.join(5.0)
    with pytest.raises(KeyError):
        job.result()


# --------------------------------------------------------------------------
# 5-6. Склейка с ядром: поток без bpy, ответ побитово тот же
# --------------------------------------------------------------------------


def _session(workers=0):
    controller = EnvelopeDebugSessionController()
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    evaluation = evaluate_envelope_debug_staged(
        quad_row_bundle(ROW),
        frozenset(range(ROW)),
        0.25,
        profile=profile,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density="1",
        workers=workers,
    )
    remember_queue_session(
        controller,
        "row",
        evaluation.topology_scene,
        evaluation.exact_debug_scenes,
        evaluation,
        density="1",
    )
    return controller


def _timing_free(scene):
    payload = queue_scene_payload(scene)
    for domain in payload["domains"]:
        for key in ("prepare_seconds", "coverage_seconds", "contour_seconds", "timings"):
            domain.pop(key)
    return payload


class _Tripwire(types.ModuleType):
    """`bpy`, чьи атрибуты читают только записывая, кто читал."""

    def __init__(self) -> None:
        super().__init__("bpy")
        self.reads: list[tuple[str, str]] = []

    def __getattr__(self, name):
        self.reads.append((name, threading.current_thread().name))
        raise AttributeError(name)


class _FakeBpy(types.ModuleType):
    """Ровно то, что читает склейка: объекты, сцена с настройками, таймеры."""

    def __init__(self, *, objects, settings) -> None:
        super().__init__("bpy")
        self.data = types.SimpleNamespace(objects=objects)
        self.context = types.SimpleNamespace(
            scene=types.SimpleNamespace(hotspotuv_settings=settings),
            window_manager=types.SimpleNamespace(windows=[]),
        )
        timers = FakeTimers()
        self.app = types.SimpleNamespace(timers=timers)
        self.timers = timers


def _settings(**overrides):
    values = dict(
        envelope_debug_engine="QUEUE",
        envelope_debug_source_object="row",
        envelope_debug_fan_density="1",
        envelope_debug_workers=0,
        envelope_debug_alpha=0.4,
        envelope_debug_queue_timing="",
    )
    values.update(overrides)
    return types.SimpleNamespace(**values)


def _context(controller):
    return types.SimpleNamespace(
        window_manager=types.SimpleNamespace(_cftuv_envelope_debug_session=controller)
    )


def test_the_compute_thread_never_touches_bpy(monkeypatch):
    controller = _session()
    entries = controller.queue_session.entries
    tripwire = _Tripwire()
    monkeypatch.setitem(sys.modules, "bpy", tripwire)

    def compute(cancel):
        return recompute_queue_coverage(entries, "0.4", cancel=cancel)

    job = ThreadedPreviewJob(compute)
    job.join(120.0)
    scene = job.result()

    assert tripwire.reads == []
    assert _timing_free(scene) == _timing_free(recompute_queue_coverage(entries, "0.4"))


def test_the_glue_begin_closure_reads_no_bpy_in_the_thread(monkeypatch):
    """`_begin` (главный поток) собирает замыкание; всё, что исполняет поток, bpy не читает."""

    controller = _session()
    tripwire = _Tripwire()
    monkeypatch.setitem(sys.modules, "bpy", tripwire)
    target = gp.GpPreviewTargetV1("row", "1", 0)
    request = types.SimpleNamespace(alpha=0.4, payload=target)

    job = gp._begin(controller, request)
    job.join(120.0)

    assert [name for name, thread in tripwire.reads if thread != "MainThread"] == []
    assert _timing_free(job.result()) == _timing_free(
        recompute_queue_coverage(controller.queue_session.entries, str(float(0.4)))
    )


def test_a_drag_in_the_glue_ends_with_the_synchronous_coverage_at_the_last_value(monkeypatch):
    """Пять быстрых изменений: считается и применяется ОДНО, последнее, и оно равно синхронному."""

    controller = _session()
    fake = _FakeBpy(objects={gp_object_name(): object()}, settings=_settings())
    monkeypatch.setitem(sys.modules, "bpy", fake)
    written = []

    def redraw(source_name, exact_scenes, scene, **kwargs):
        written.append((source_name, scene, kwargs))
        return object()

    monkeypatch.setattr(envelope_debug_renderer, "redraw_envelope_queue_layers", redraw)
    monkeypatch.setattr(envelope_debug_renderer, "visibility_from_settings", lambda settings: {})
    settings = fake.context.scene.hotspotuv_settings
    context = _context(controller)
    for alpha in (0.30, 0.31, 0.32, 0.33, 0.34):
        settings.envelope_debug_alpha = alpha
        gp.schedule_alpha_preview(settings, context)
    scheduler = controller.alpha_preview
    # Заказ ничего не считает: счёт не стартовал, ничего не применено, таймер взведён.
    assert scheduler.counters.started == 0 and written == []
    assert fake.timers.is_registered(scheduler._callback)

    gp.settle_alpha_preview(controller)

    assert len(written) == 1
    scene = written[0][1]
    expected = recompute_queue_coverage(
        controller.queue_session.entries, str(float(settings.envelope_debug_alpha))
    )
    assert _timing_free(scene) == _timing_free(expected)
    counters = scheduler.counters
    assert (counters.requested, counters.applied) == (5, 1)
    assert counters.requested == counters.applied + counters.coalesced + counters.stale
    assert "alpha redraw" in settings.envelope_debug_queue_timing
    assert scheduler.status_text.startswith("ready alpha=0.34")


def gp_object_name():
    return envelope_debug_renderer.envelope_debug_object_name("row")


def test_the_glue_discards_a_result_when_the_session_or_the_object_changed(monkeypatch):
    controller = _session()
    objects = {gp_object_name(): object()}
    fake = _FakeBpy(objects=objects, settings=_settings())
    monkeypatch.setitem(sys.modules, "bpy", fake)
    written = []
    monkeypatch.setattr(
        envelope_debug_renderer,
        "redraw_envelope_queue_layers",
        lambda *args, **kwargs: written.append(1) or object(),
    )
    monkeypatch.setattr(envelope_debug_renderer, "visibility_from_settings", lambda settings: {})
    settings = fake.context.scene.hotspotuv_settings
    context = _context(controller)

    gp.schedule_alpha_preview(settings, context)
    objects.clear()  # объект исчез за время счёта
    gp.settle_alpha_preview(controller)
    assert written == []
    assert "Envelope debug object is gone" in controller.alpha_preview.status_text

    objects[gp_object_name()] = object()
    gp.schedule_alpha_preview(settings, context)
    controller._queue_session = None  # сессию сбросила кнопка за время счёта
    gp.settle_alpha_preview(controller)
    assert written == []
    assert controller.alpha_preview.counters.invalid + controller.alpha_preview.counters.unavailable >= 2


def test_the_callback_keeps_the_old_silence_and_the_old_density_string(monkeypatch):
    controller = _session()
    fake = _FakeBpy(objects={}, settings=_settings())
    monkeypatch.setitem(sys.modules, "bpy", fake)
    context = _context(controller)

    # Другой движок, нет источника, нет сессии окна: ничего не заказано.
    gp.schedule_alpha_preview(_settings(envelope_debug_engine="LEGACY"), context)
    gp.schedule_alpha_preview(_settings(envelope_debug_source_object=""), context)
    gp.schedule_alpha_preview(_settings(), _context(None))
    assert controller.alpha_preview is None

    # Плотность сменена: прежняя строка в таймингах, счёта нет.
    settings = _settings(envelope_debug_fan_density="4")
    gp.schedule_alpha_preview(settings, context)
    assert settings.envelope_debug_queue_timing == "Fan Density changed; press Build"
    assert controller.alpha_preview is None  # ни заказа, ни планировщика

    # Сессии нет: ТЕПЕРЬ это названо в статусе превью, а тайминги не тронуты.
    controller.invalidate_queue_session()
    settings = _settings()
    gp.schedule_alpha_preview(settings, context)
    assert settings.envelope_debug_queue_timing == ""
    assert controller.alpha_preview.status_text == gp.NO_WARM_SESSION


def test_a_cleared_debug_object_is_named_at_order_time_and_nothing_is_computed(monkeypatch):
    """После Clear считать некуда: заказ называет причину, пул не гоняется ради выброшенного результата."""

    controller = _session()
    fake = _FakeBpy(objects={}, settings=_settings())
    monkeypatch.setitem(sys.modules, "bpy", fake)

    gp.schedule_alpha_preview(fake.context.scene.hotspotuv_settings, _context(controller))

    scheduler = controller.alpha_preview
    assert scheduler.status_text == gp.OBJECT_GONE
    assert scheduler.counters.started == 0 and scheduler.counters.unavailable == 1
    assert fake.timers.registered == []


# --------------------------------------------------------------------------
# 7. Кнопки, сброс сессии, пул
# --------------------------------------------------------------------------


class _Recorder:
    def __init__(self):
        self.calls = []

    def quiesce(self, reason):
        self.calls.append(("quiesce", reason))

    def supersede(self, reason):
        self.calls.append(("supersede", reason))


def test_the_session_controller_quiesces_the_preview_before_the_heavy_paths():
    controller = EnvelopeDebugSessionController()
    recorder = _Recorder()
    controller.alpha_preview = recorder

    controller.invalidate_queue_session()
    assert recorder.calls == [("supersede", "warm session dropped")]  # без ожидания: калбэк свойства

    controller.clear()
    assert recorder.calls[-1] == ("quiesce", "session cleared")

    bundle = quad_row_bundle(ROW)
    evaluate_envelope_debug_staged(
        bundle,
        frozenset(range(ROW)),
        0.25,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density=None,
    )
    assert ("quiesce", "Envelope debug build") in recorder.calls

    from cftuv.envelope_production_export import run_production

    recorder.calls.clear()
    run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        0.25,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=0,
    )
    assert recorder.calls[0] == ("quiesce", "Build Decal Mesh")


def test_a_pool_serializes_runs_from_two_threads():
    pool = DomainPool(0)
    pool._run_lock.acquire()
    finished = threading.Event()

    def other():
        pool.run([])
        finished.set()

    thread = threading.Thread(target=other, daemon=True)
    thread.start()
    assert not finished.wait(0.2)  # ждёт замок
    pool._run_lock.release()
    assert finished.wait(5.0)
    thread.join(5.0)


def _slider_pool(controller):
    pool = get_domain_pool(2)
    pool.ensure_started()
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    return pool, profile, controller.slider_coverage_pool(2, profile)


@pytest.fixture()
def _pool_always(monkeypatch):
    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


def test_a_cancelled_run_takes_no_new_task_and_the_slider_names_the_cancellation(_pool_always):
    controller = _session()
    entries = controller.queue_session.entries
    pool, profile, coverage_pool = _slider_pool(controller)
    assert coverage_pool is not None

    cancel = threading.Event()
    cancel.set()  # отмена до старта: ни одна задача не берётся
    tasks = [DomainTaskV1(0, 0, "d0", None, None, "0.4", frozenset())]
    assert pool.run(tasks, cancel=cancel).results == {}

    with pytest.raises(CoverageCancelled):
        recompute_queue_coverage(entries, "0.4", coverage_pool=coverage_pool, cancel=cancel)
    with pytest.raises(CoverageCancelled):
        # Без пула тот же заказ останавливается между доменами родителя.
        recompute_queue_coverage(entries, "0.4", cancel=cancel)
    # Остановленный заказ не оставил записей откатов в счётчиках пула.
    assert not [item for item in profile.snapshot().counters if item.name == POOL_TASK_FALLBACK]


def test_the_preview_in_real_workers_equals_the_synchronous_coverage(_pool_always):
    """Пул живой, покрытие уходит воркерам: поток счёта отдаёт ту же сцену, что и синхронный путь."""

    controller = _session()
    entries = controller.queue_session.entries
    pool, profile, coverage_pool = _slider_pool(controller)
    assert coverage_pool is not None

    for alpha in ("0.4", "0.3", "0.45"):
        job = ThreadedPreviewJob(
            lambda cancel, alpha=alpha: recompute_queue_coverage(
                entries, alpha, coverage_pool=coverage_pool, cancel=cancel
            )
        )
        job.join(120.0)
        assert _timing_free(job.result()) == _timing_free(
            recompute_queue_coverage(entries, alpha)
        )
    counters = {item.name: item.value for item in profile.snapshot().counters if item.patch_domain_id is None}
    assert counters[POOL_COVERAGE_DISPATCHED] == ROW
    assert counters[POOL_TASK_FALLBACK] == 0
