"""Живая ширина декали (хост без Blender): планировщик продуктовой цели, поток счёта, равенство ответа.

Утверждения среза DECAL-WIDTH-LIVE (точная часть), каждое стоит на проверяемом факте:

1. ПЛАНИРОВЩИК: тот же класс, что у превью alpha, называет величину `width`; два планировщика контроллера не
   летят разом (`hold`), заказ при этом не теряется; кнопки и сброс останавливают оба (`quiesce_preview`);
2. ПРОДУКТОВЫЙ ПРОГОН ОТМЕНЯЕМ: `cancel` останавливает его до работы родителя и сразу после возврата пула
   (`ProductionCancelled`), без молчаливого доделывания за пул; кэши сессии целы, следующий прогон равен
   эталону; без `cancel` поведение прежнее;
3. ПОТОК СЧЁТА НЕ ТРОГАЕТ BLENDER: замыкание `_begin` читает `bpy` только на главном потоке;
4. ПЯТЬ ИЗМЕНЕНИЙ: каждое сразу даёт превью (названное `PREVIEW_BINARY64_V1`), счёт один и на последней ширине,
   результат побитово равен прямому прогону холодной сессии на той же ширине, превью после применения снято;
5. ЦЕЛЬ: нет записи кнопки — ничего; другая плотность или допуск — названная причина и превью снято; правка меша
   после кнопки, исчезнувший объект, новая кнопка во время счёта — отброшено с причиной;
6. ИСТОРИЯ: после Undo/Redo превью снято, а расхождение ползунка и меша (`cftuv_decal_width` меша) заказывает пересчёт.
"""

from __future__ import annotations

import sys
import threading
import types
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as production_mesh  # noqa: E402
from cftuv import envelope_width_live as live  # noqa: E402
from cftuv.envelope_alpha_preview import AlphaPreviewScheduler  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_production_export import ProductionCancelled, run_production  # noqa: E402
from cftuv.envelope_width_preview import PREVIEW_BINARY64_V1  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402
from test_envelope_alpha_preview import FakeClock, FakeJob, FakeTimers  # noqa: E402

ROW = 5
SELECTED = frozenset(range(ROW))
OFFSET = 0.02


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# Фальшивки Blender
# --------------------------------------------------------------------------


class _Decal:
    """Объект декали: режим, имя и меш, чьи свойства (`keys`, индекс) хранят ширину записи."""

    mode = "OBJECT"
    name = "row.CFTUV_Decal"

    def __init__(self, width) -> None:
        self.data = {production_mesh.DECAL_WIDTH_PROPERTY: width}


class _FakeBpy(types.ModuleType):
    def __init__(self, *, settings, mesh_settings, objects) -> None:
        super().__init__("bpy")
        self.data = types.SimpleNamespace(objects=objects)
        self.context = types.SimpleNamespace(
            scene=types.SimpleNamespace(
                hotspotuv_settings=settings, hotspotuv_decal_mesh=mesh_settings
            ),
            window_manager=types.SimpleNamespace(windows=[]),
        )
        self.timers = FakeTimers()
        self.app = types.SimpleNamespace(timers=self.timers)


def _settings(**overrides):
    values = dict(
        envelope_debug_alpha=0.25,
        envelope_debug_fan_density="1",
        envelope_debug_max_stretch=20,
        envelope_debug_workers=0,
    )
    values.update(overrides)
    return types.SimpleNamespace(**values)


def _mesh_settings():
    return types.SimpleNamespace(
        offset=OFFSET, material_name="CFTUV_Decal", status="", timing=""
    )


def _direct(bundle, controller, alpha, **kwargs):
    return run_production(
        controller,
        bundle,
        SELECTED,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density="1",
        domain_pool=None,
        **kwargs,
    )


def _digest(results):
    return production_mesh.build_mesh_arrays(results, OFFSET).digest


class _World:
    """Контроллер после «Build Decal Mesh», подмена Blender и запись того, что писатель получил."""

    def __init__(self, monkeypatch, *, alpha=0.25):
        self.bundle = quad_row_bundle(ROW)
        self.controller = EnvelopeDebugSessionController()
        self.run = _direct(self.bundle, self.controller, alpha)
        self.settings = _settings(envelope_debug_alpha=alpha)
        self.mesh_settings = _mesh_settings()
        self.decal = _Decal(alpha)
        self.objects = {"row": types.SimpleNamespace(name="row", mode="OBJECT"), decal_key(): self.decal}
        self.fake = _FakeBpy(
            settings=self.settings, mesh_settings=self.mesh_settings, objects=self.objects
        )
        self.written: list = []
        self.digest_now = self.bundle.source_revision.digest
        monkeypatch.setitem(sys.modules, "bpy", self.fake)
        monkeypatch.setattr(production_mesh, "find_decal_object", lambda source: self.objects.get(decal_key()))
        monkeypatch.setattr(production_mesh, "rewrite_decal_mesh", self._rewrite)
        monkeypatch.setattr(live, "_current_digest", lambda source: self.digest_now)
        self.record = live.remember_build(
            self.controller,
            "row",
            self.bundle,
            self.run,
            source_object_key="object",
            source_data_key="mesh",
            selected=SELECTED,
            density="1",
            stretch_percent=20,
            width=alpha,
        )
        self.context = types.SimpleNamespace(
            window_manager=types.SimpleNamespace(_cftuv_envelope_debug_session=self.controller),
            scene=self.fake.context.scene,
        )

    def _rewrite(self, source, results, *, offset, material_name, width):
        self.written.append((tuple(results), offset, material_name, width))
        self.decal.data[production_mesh.DECAL_WIDTH_PROPERTY] = width
        arrays = production_mesh.build_mesh_arrays(results, offset)
        return types.SimpleNamespace(
            domains=arrays.domains,
            skipped=arrays.skipped,
            warnings=arrays.warnings,
            weld_counters=(),
            offset_counters=(),
            faces=len(arrays.faces),
            vertices=len(arrays.positions),
        )

    def drag(self, *values):
        for value in values:
            self.settings.envelope_debug_alpha = value
            live.schedule_width_live(self.settings, self.context)

    @property
    def scheduler(self):
        return self.controller.width_live


def decal_key():
    return "row.CFTUV_Decal"


# --------------------------------------------------------------------------
# 1. Планировщик
# --------------------------------------------------------------------------


def _harness(**extra):
    clock, timers = FakeClock(), FakeTimers()
    jobs, applied = [], []

    def begin(request):
        job = FakeJob(value=request.alpha)
        jobs.append(job)
        return job

    scheduler = AlphaPreviewScheduler(
        begin=begin,
        apply=lambda request, job, value: applied.append(value),
        timers=timers,
        clock=clock,
        sleep=clock.advance,
        **extra,
    )
    return scheduler, clock, jobs, applied


def test_the_scheduler_names_the_value_by_the_label_of_its_target():
    scheduler, clock, jobs, applied = _harness(label="width")

    scheduler.request(0.4)
    assert scheduler.status_text == "computing width=0.4..."
    clock.advance(1.0)
    scheduler.step()
    jobs[0].finished = True
    scheduler.step()

    assert applied == [0.4] and scheduler.status_text.startswith("ready width=0.4 ")


def test_a_held_scheduler_waits_for_the_other_flight_and_loses_no_order():
    busy = {"other": True}
    scheduler, clock, jobs, applied = _harness(hold=lambda: busy["other"])

    scheduler.request(0.3)
    clock.advance(1.0)
    delay = scheduler.step()

    assert jobs == [] and scheduler.counters.started == 0 and scheduler.busy
    assert delay == scheduler._poll  # опрос, а не вращение вхолостую
    busy["other"] = False
    scheduler.step()
    assert len(jobs) == 1 and scheduler.counters.started == 1
    jobs[0].finished = True
    scheduler.step()
    assert applied == [0.3]


class _JoinableJob(FakeJob):
    """Поток, который заканчивает, как только его дождались."""

    def join(self, timeout=None):
        super().join(timeout)
        self.finished = True


def test_the_two_schedulers_of_a_controller_hold_each_other_and_the_buttons_stop_both(monkeypatch):
    world = _World(monkeypatch)
    from cftuv import envelope_alpha_preview_gp as gp

    world.drag(0.3)
    width = world.scheduler
    debug = gp.scheduler_of(world.controller)
    flying = FakeJob()
    debug._job, debug._job_request = flying, object()
    assert debug.in_flight and width._hold() is True
    width_job = _JoinableJob()
    width._job, width._job_request = width_job, object()
    debug._job = None
    assert debug._hold() is True and width._hold() is False

    world.controller.quiesce_preview("Build Decal Mesh")
    assert width_job.cancelled and width_job.joined  # живая ширина остановлена и дождана
    assert not width.busy


# --------------------------------------------------------------------------
# 2. Продуктовый прогон отменяем
# --------------------------------------------------------------------------


def test_a_cancelled_production_run_raises_and_leaves_the_session_exact():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    stop = threading.Event()
    stop.set()

    with pytest.raises(ProductionCancelled) as error:
        _direct(bundle, controller, 0.3, cancel=stop, quiesce=False)

    assert "PRODUCTION_CANCELLED" in str(error.value)
    after = _direct(bundle, controller, 0.3)
    reference = _direct(quad_row_bundle(ROW), EnvelopeDebugSessionController(), 0.3)
    assert _digest(after.results) == _digest(reference.results)


class _CancellingPool:
    """Пул, которому заказ остановки пришёл во время работы: он возвращает неполный ответ."""

    requested = 2

    def __init__(self, stop):
        self.stop = stop
        self.cancels = []

    def run(self, tasks, cancel=None):
        self.cancels.append(cancel)
        results = {}
        for task, _frame in order_by_cost(tasks):
            results[task.task_id] = solve_task(task)
            self.stop.set()  # после первой задачи остальные пул не берёт
            break
        return DomainPoolRunV1(results, self.requested)


def test_a_cancel_during_the_pool_run_is_not_finished_by_the_parent(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    stop = threading.Event()
    pool = _CancellingPool(stop)

    with pytest.raises(ProductionCancelled):
        run_production(
            controller,
            bundle,
            SELECTED,
            0.3,
            source_object_key="object",
            source_data_key="mesh",
            density="1",
            domain_pool=pool,
            cancel=stop,
            quiesce=False,
        )

    assert pool.cancels == [stop]  # заказ дошёл до пула
    # Домены, которых пул не взял, родитель не досчитал: результатов под alpha 0.3 в кэше сессии не больше, чем сделал пул.
    assert controller.production_result_count < ROW


def test_a_production_run_without_cancel_is_exactly_what_it_was():
    bundle = quad_row_bundle(ROW)
    first = _direct(bundle, EnvelopeDebugSessionController(), 0.3)
    second = _direct(bundle, EnvelopeDebugSessionController(), 0.3, cancel=None)

    assert _digest(first.results) == _digest(second.results)
    assert first.selected_by_patch == tuple((patch, (patch,)) for patch in range(ROW))


# --------------------------------------------------------------------------
# 3. Поток счёта не трогает Blender
# --------------------------------------------------------------------------


class _Tripwire(types.ModuleType):
    def __init__(self) -> None:
        super().__init__("bpy")
        self.reads: list[tuple[str, str]] = []

    def __getattr__(self, name):
        self.reads.append((name, threading.current_thread().name))
        raise AttributeError(name)


def test_the_compute_closure_reads_no_bpy_in_the_thread(monkeypatch):
    world = _World(monkeypatch)
    objects = world.objects
    tripwire = _Tripwire()
    tripwire.data = types.SimpleNamespace(objects=objects)  # главный поток читает объекты в `_begin`
    monkeypatch.setitem(sys.modules, "bpy", tripwire)
    target = live.target_of(world.settings, world.mesh_settings, world.record)
    request = types.SimpleNamespace(alpha=0.4, payload=target)

    job = live._begin(world.controller, request)
    job.join(120.0)
    run = job.result()

    assert [name for name, thread in tripwire.reads if thread != "MainThread"] == []
    reference = _direct(quad_row_bundle(ROW), EnvelopeDebugSessionController(), 0.4)
    assert _digest(run.results) == _digest(reference.results)


# --------------------------------------------------------------------------
# 4. Пять изменений
# --------------------------------------------------------------------------


def test_five_changes_give_five_previews_and_one_exact_result_equal_to_a_direct_run(monkeypatch):
    world = _World(monkeypatch)
    drag = (0.30, 0.31, 0.32, 0.33, 0.34)

    serials = []
    for value in drag:
        world.drag(value)
        state = world.controller.width_preview
        # Превью есть СРАЗУ после каждого изменения, до единого шага таймера, и названо превью.
        assert state is not None and state.preview.method == PREVIEW_BINARY64_V1
        assert state.preview.width == pytest.approx(value) and state.preview.lines > 0
        serials.append(state.serial)
    assert serials == sorted(set(serials)) and len(serials) == 5
    scheduler = world.scheduler
    assert scheduler.counters.started == 0 and world.written == []  # заказ ничего не считает
    assert world.fake.timers.is_registered(scheduler._callback)
    assert "PREVIEW_BINARY64_V1" in live.status_lines(world.controller)[-1]

    live.settle_width_live(world.controller)

    counters = scheduler.counters
    assert (counters.requested, counters.coalesced, counters.applied) == (5, 4, 1)
    assert len(world.written) == 1
    results, offset, material, width = world.written[0]
    assert width == drag[-1] and offset == OFFSET and material == "CFTUV_Decal"
    reference = _direct(quad_row_bundle(ROW), EnvelopeDebugSessionController(), drag[-1])
    assert _digest(results) == _digest(reference.results)  # побитово равно прямому прогону холодной сессии
    assert world.controller.width_preview is None  # точный результат применён: превью снято
    assert scheduler.status_text.startswith("ready width=0.34")
    assert world.mesh_settings.status.startswith("MATERIALIZED")
    assert "live width 0.34" in world.mesh_settings.timing
    assert [line for line in live.status_lines(world.controller) if "PREVIEW" in line] == []


def test_a_width_change_without_a_build_does_nothing_and_orders_nothing(monkeypatch):
    world = _World(monkeypatch)
    world.controller.width_build = None
    world.controller.width_preview = None

    world.drag(0.4)

    assert world.controller.width_preview is None and world.scheduler is None


# --------------------------------------------------------------------------
# 5. Цель
# --------------------------------------------------------------------------


def test_a_changed_density_or_stretch_is_named_and_computes_nothing(monkeypatch):
    world = _World(monkeypatch)

    world.settings.envelope_debug_fan_density = "4"
    world.drag(0.4)
    scheduler = world.scheduler
    assert scheduler.status_text == live.POLICY_CHANGED.format(name="Fan Density")
    assert scheduler.counters.started == 0 and scheduler.counters.unavailable == 1
    assert world.controller.width_preview is None

    world.settings.envelope_debug_fan_density = "1"
    world.settings.envelope_debug_max_stretch = 30
    world.drag(0.41)
    assert scheduler.status_text == live.POLICY_CHANGED.format(name="Max stretch")
    assert world.written == []


def test_a_mesh_edit_after_the_build_discards_the_order_by_name(monkeypatch):
    world = _World(monkeypatch)
    world.digest_now = "edited"

    world.drag(0.4)
    live.settle_width_live(world.controller)

    assert world.written == []
    assert world.scheduler.status_text == live.SOURCE_CHANGED
    assert world.scheduler.counters.unavailable == 1


def test_an_exact_result_for_a_vanished_decal_or_a_replaced_build_is_discarded_with_the_reason(monkeypatch):
    world = _World(monkeypatch)
    world.drag(0.4)
    world.objects.pop(decal_key())
    live.settle_width_live(world.controller)
    assert world.written == [] and world.scheduler.counters.unavailable == 1
    assert world.scheduler.status_text == live.DECAL_GONE

    world.objects[decal_key()] = world.decal
    world.drag(0.45)
    scheduler = world.scheduler
    scheduler._deadline = scheduler._clock()
    scheduler.step()  # счёт стартовал
    assert scheduler.in_flight
    live.remember_build(  # кнопка нажата ещё раз за время счёта
        world.controller,
        "row",
        world.bundle,
        world.run,
        source_object_key="object",
        source_data_key="mesh",
        selected=SELECTED,
        density="1",
        stretch_percent=20,
        width=0.45,
    )
    live.settle_width_live(world.controller)
    assert world.written == []
    assert world.scheduler.status_text.endswith(live.BUILD_REPLACED)


def test_a_failed_writer_is_a_named_discard_not_a_crash(monkeypatch):
    world = _World(monkeypatch)

    def refuse(source, results, **kwargs):
        raise production_mesh.ProductionWriteError(
            production_mesh.OUTCOME_DECAL_IN_EDIT_MODE, "the decal is in Edit mode"
        )

    monkeypatch.setattr(production_mesh, "rewrite_decal_mesh", refuse)
    world.drag(0.4)
    live.settle_width_live(world.controller)

    assert world.scheduler.counters.invalid == 1 and world.scheduler.counters.failed == 0
    assert "DECAL_OBJECT_IN_EDIT_MODE" in world.scheduler.status_text
    assert world.controller.width_preview is not None  # точного результата нет: превью остаётся и названо превью


def test_the_preview_state_belongs_to_the_session_and_a_new_revision_drops_it(monkeypatch):
    world = _World(monkeypatch)
    world.drag(0.4)
    assert world.controller.width_preview is not None and world.controller.width_build is not None

    world.controller.clear()

    assert world.controller.width_preview is None and world.controller.width_build is None


# --------------------------------------------------------------------------
# 6. История
# --------------------------------------------------------------------------


def test_after_undo_or_redo_the_preview_is_gone_and_a_stale_mesh_is_recomputed(monkeypatch):
    world = _World(monkeypatch)
    world.drag(0.4)
    live.settle_width_live(world.controller)
    assert world.decal.data[production_mesh.DECAL_WIDTH_PROPERTY] == 0.4
    world.drag(0.5)
    assert world.controller.width_preview is not None
    scheduler = world.scheduler
    scheduler.supersede("test")  # заказ ушёл вместе со сценой при Undo

    # Redo вернул ползунок на 0.5, а в меше ширина 0.4.
    live.reconcile_after_history(world.context)

    assert world.controller.width_preview is None  # превью при истории снимается всегда
    assert scheduler.busy  # расхождение заказало точный пересчёт
    live.settle_width_live(world.controller)
    assert world.decal.data[production_mesh.DECAL_WIDTH_PROPERTY] == 0.5

    before = scheduler.counters.requested
    live.reconcile_after_history(world.context)  # ширина меша и ползунок равны: заказа нет
    assert scheduler.counters.requested == before and not scheduler.busy


def test_loading_a_file_forgets_the_build_and_the_preview_of_every_controller():
    """Контроллер окна переживает загрузку, а имя источника в новом файле может совпасть: превью не должно остаться."""

    from cftuv.envelope_debug_session import _WindowManagerSessionAttribute

    descriptor = _WindowManagerSessionAttribute()
    manager = types.SimpleNamespace(as_pointer=lambda: 7)
    controller = EnvelopeDebugSessionController()
    descriptor.__set__(manager, controller)
    controller.width_build = object()
    controller.width_preview = object()

    descriptor.forget_width_state()

    assert controller.width_build is None and controller.width_preview is None
