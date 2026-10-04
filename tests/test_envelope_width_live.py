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
6. ИСТОРИЯ: после Undo/Redo превью снято, а расхождение ползунка и меша (`cftuv_decal_width` меша) заказывает пересчёт;
7. ЦЕЛЬ — СОБСТВЕННАЯ ДЕКАЛЬ АКТИВНОГО ОБЪЕКТА: нет декали, декаль другого источника, запись кнопки про другой объект,
   сброшенная сессия, чужая ревизия — каждое названо «Build Decal Mesh first for <объект>»; ползунок на чужом объекте
   ничего не заказывает и не рисует; смена активного снимает превью, но не принятый заказ; поле подтягивается к
   ширине меша без пересчёта; удалённая декаль снова делает инструмент недоступным.
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


class _Props:
    """Свойства объекта Blender, как их читает хост: `keys`, индекс, `get`."""

    type = "MESH"
    mode = "OBJECT"
    parent = None

    def __init__(self, name, **props) -> None:
        self.name = name
        self._props = dict(props)

    def keys(self):
        return list(self._props)

    def __getitem__(self, key):
        return self._props[key]

    def get(self, key, default=None):
        return self._props.get(key, default)


class _Decal(_Props):
    """Объект декали: режим, имя, метки сборки и меш, чьи свойства (`keys`, индекс) хранят ширину записи."""

    def __init__(self, width, *, source="row", revision="") -> None:
        super().__init__(
            "row.CFTUV_Decal",
            **{
                production_mesh.DECAL_SOURCE_PROPERTY: source,
                production_mesh.DECAL_REVISION_PROPERTY: revision,
            },
        )
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


class _Settings(types.SimpleNamespace):
    """Настройки сцены; `hook` — калбэк `update` ползунка (как в Blender: срабатывает на каждую запись ширины)."""

    hook = None

    def __setattr__(self, name, value) -> None:
        super().__setattr__(name, value)
        if name == "envelope_debug_alpha" and self.hook is not None:
            self.hook(self)


def _settings(**overrides):
    values = dict(
        envelope_debug_alpha=0.25,
        envelope_debug_fan_density="1",
        envelope_debug_max_stretch=20,
        envelope_debug_workers=0,
    )
    values.update(overrides)
    return _Settings(**values)


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
        self.digest_now = self.bundle.source_revision.digest
        self.decal = _Decal(alpha, revision=f"host-source:{self.digest_now}:row")
        self.objects = {"row": _Props("row"), decal_key(): self.decal}
        self.decal.parent = self.objects["row"]
        self.fake = _FakeBpy(
            settings=self.settings, mesh_settings=self.mesh_settings, objects=self.objects
        )
        self.written: list = []
        monkeypatch.setitem(sys.modules, "bpy", self.fake)
        monkeypatch.setattr(
            production_mesh, "find_decal_object", lambda source: self.objects.get(f"{source.name}.CFTUV_Decal")
        )
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
            active_object=self.objects["row"],
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
    controller.width_target = "row"

    descriptor.forget_width_state()

    assert controller.width_build is None and controller.width_preview is None
    assert controller.width_target is None


# --------------------------------------------------------------------------
# 7. Цель — собственная декаль активного объекта
# --------------------------------------------------------------------------


def _probe(world, **changes):
    values = dict(
        object_name="row.CFTUV_Decal",
        source_name="row",
        revision=f"host-source:{world.digest_now}:row",
    )
    values.update(changes)
    return live.DecalProbeV1(**values)


def test_the_availability_predicate_names_every_reason_and_passes_only_a_fresh_own_decal(monkeypatch):
    import dataclasses

    world = _World(monkeypatch)
    controller = world.controller
    need = "Build Decal Mesh first for row"

    assert live.availability_problem(controller, None, None) == live.NO_ACTIVE
    assert live.availability_problem(controller, "row", None) == need  # у объекта нет декали
    assert live.availability_problem(controller, "row", _probe(world, source_name="other")) == (
        f"{need}: its decal was built for other"  # декаль чужого источника
    )
    assert live.availability_problem(controller, "row", _probe(world, source_name="")).endswith("another object")
    fresh = EnvelopeDebugSessionController()
    assert live.availability_problem(fresh, "row", _probe(world)) == f"{need}: this window holds no build session"
    assert live.availability_problem(controller, "other", _probe(world, source_name="other")) == (
        "Build Decal Mesh first for other: the build session of this window belongs to row"  # запись кнопки про другой
    )
    assert live.availability_problem(controller, "row", _probe(world, revision="host-source:other:row")) == (
        f"{need}: its decal is from another revision of the source"
    )
    stale = dataclasses.replace(world.record, invalidation_count=world.record.invalidation_count - 1)
    controller.width_build = stale  # сессия сброшена после записи
    assert live.availability_problem(controller, "row", _probe(world)) == f"{need}: the source changed since the last build"
    blank = dataclasses.replace(world.record, preview_inputs=types.SimpleNamespace(runs=()))
    controller.width_build = blank
    assert live.availability_problem(controller, "row", _probe(world)) == live.NO_PREVIEW

    controller.width_build = world.record
    assert live.availability_problem(controller, "row", _probe(world)) == ""
    assert live.availability_problem(controller, "row", _probe(world, revision="")) == ""  # пустая декаль ревизии не помнит


def test_the_availability_follows_the_active_object_and_its_own_decal(monkeypatch):
    world = _World(monkeypatch)
    context = world.context

    assert live.width_problem(context) == ""
    context.active_object = _Props("other")  # другой меш без своей декали
    assert live.width_problem(context) == "Build Decal Mesh first for other"
    context.active_object = None
    assert live.width_problem(context) == live.NO_ACTIVE
    context.active_object = _Props("light")
    context.active_object.type = "LIGHT"
    assert live.width_problem(context) == live.NO_ACTIVE
    context.active_object = world.decal  # активна сама декаль: цель — её источник
    assert live.width_problem(context) == ""

    context.active_object = world.objects["row"]
    del world.objects[decal_key()]  # декаль удалена: инструмент снова недоступен
    assert live.width_problem(context) == "Build Decal Mesh first for row"
    world.objects[decal_key()] = world.decal
    assert live.width_problem(context) == ""

    # Сборка для другого объекта заменила запись: у первого декаль есть, а сессия уже не про него.
    other = _Props("other")
    other_decal = _Decal(0.25, source="other", revision=f"host-source:{world.digest_now}:other")
    other_decal.name = "other.CFTUV_Decal"
    world.objects.update({"other": other, other_decal.name: other_decal})
    context.active_object = other
    assert live.width_problem(context) == (
        "Build Decal Mesh first for other: the build session of this window belongs to row"
    )


def test_a_slider_change_on_an_object_without_its_own_decal_orders_nothing_and_leaves_no_lines(monkeypatch):
    world = _World(monkeypatch)
    world.context.active_object = _Props("other")

    world.drag(0.4, 0.5)

    assert world.scheduler is None and world.controller.width_preview is None and world.written == []
    live.preview_now(world.controller, 0.4, OFFSET)  # чужие линии
    assert world.controller.width_preview is not None
    world.drag(0.6)
    assert world.controller.width_preview is None and world.scheduler is None
    world.context.active_object = world.objects["row"]  # вернулись к своему: путь ползунка работает
    world.drag(0.7)
    assert world.scheduler.counters.requested == 1 and world.controller.width_preview is not None


def test_a_change_of_the_active_object_drops_the_preview_but_not_an_order_already_accepted(monkeypatch):
    world = _World(monkeypatch)
    context = world.context
    world.drag(0.4)
    assert world.controller.width_target == "row" and world.controller.width_preview is not None
    assert live.follow_active_object(context) is False  # активный тот же: ничего не сброшено
    assert world.controller.width_preview is not None

    context.active_object = _Props("other")
    assert live.follow_active_object(context) is True
    assert world.controller.width_target == "other" and world.controller.width_preview is None
    assert world.scheduler.busy  # принятый заказ про декаль `row`: он не пропал и завершится ею
    live.settle_width_live(world.controller)
    assert [item[3] for item in world.written] == [0.4]
    assert live.follow_active_object(context) is False  # то же состояние повторно не сбрасывает

    context.active_object = None
    assert live.follow_active_object(context) is True and world.controller.width_target is None
    context.active_object = world.objects["row"]
    assert live.follow_active_object(context) is True and world.controller.width_target == "row"


def test_the_deleted_decal_takes_the_preview_lines_with_it(monkeypatch):
    world = _World(monkeypatch)
    world.drag(0.4)
    assert world.controller.width_preview is not None

    del world.objects[decal_key()]

    assert live.follow_active_object(world.context) is False  # активный объект тот же
    assert world.controller.width_preview is None  # но линий без декали не остаётся
    assert live.width_problem(world.context) == "Build Decal Mesh first for row"


def test_the_width_field_follows_the_meshs_own_width_without_ordering_a_recompute(monkeypatch):
    world = _World(monkeypatch)
    world.settings.envelope_debug_alpha = 0.9  # ползунок ушёл, пока был другой объект (хука ещё нет)
    world.settings.hook = lambda settings: live.schedule_width_live(settings, world.context)
    assert live.sync_width_field(world.context) is True
    assert world.settings.envelope_debug_alpha == 0.25  # ширина собственного меша
    assert world.scheduler is None and world.controller.width_preview is None  # калбэк пересчёта не заказал
    assert live.sync_width_field(world.context) is False  # уже равны

    world.settings.envelope_debug_alpha = 0.4  # запись пользователя заказывает, как и прежде
    assert world.scheduler.counters.requested == 1
    world.decal.data[production_mesh.DECAL_WIDTH_PROPERTY] = 0.3
    assert live.sync_width_field(world.context) is False and world.settings.envelope_debug_alpha == 0.4  # заказ в пути

    live.settle_width_live(world.controller)
    world.context.active_object = _Props("other")
    world.settings.hook = None
    world.settings.envelope_debug_alpha = 0.8
    assert live.sync_width_field(world.context) is False and world.settings.envelope_debug_alpha == 0.8


def test_the_target_is_remembered_by_the_build_and_forgotten_with_the_session(monkeypatch):
    world = _World(monkeypatch)
    assert world.controller.width_target == "row"

    assert live.retarget(world.controller, "row") is False
    assert live.retarget(world.controller, "other") is True and world.controller.width_target == "other"
    world.controller.clear()
    assert world.controller.width_target is None
