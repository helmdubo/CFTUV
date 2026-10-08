"""Blender 4.5 background smoke: живая ширина декали (DECAL-WIDTH-LIVE).

В фоновом Blender таймеры `bpy.app.timers` не срабатывают, а обработчики отрисовки не вызываются, поэтому
смок ШАГАЕТ таймер сам (как смок превью alpha) и проверяет геометрию превью чистой функцией; рисование (`gpu`)
проверяют глаза владельца. Утверждения стоят на числах и содержимом меша (`mesh_content_digest`):

1. ПЯТЬ ИЗМЕНЕНИЙ ШИРИНЫ через путь `update` ползунка после «Build Decal Mesh»: каждое сразу даёт превью
   `PREVIEW_BINARY64_V1` (линии есть, цена — доли миллисекунды), меш до паузы НЕ тронут, таймер взведён; шаги
   таймера считают ОДНО значение — последнее; результат применён НА МЕСТЕ (тот же объект, тот же датаблок
   меша, число датаблоков не изменилось) и ПОБИТОВО равен прямому нажатию кнопки на последней ширине — в
   тёплой сессии и в холодной; превью после применения снято, ширина записана в свойство объекта;
2. ТО ЖЕ НА ДВУХ ВОРКЕРАХ: живой пул берёт точный пересчёт из потока планировщика, ответ тот же;
3. МОДАЛЬНЫЙ ИНСТРУМЕНТ (автомат и исполнение, те же функции, что у оператора): перетаскивание меняет ТОЛЬКО
   превью (меш и счётчики планировщика те же, свойство ширины то же); подтверждение пишет ширину в ползунок один
   раз, заказывает точный пересчёт, итоговый меш равен прямой кнопке; отмена оставляет ширину и меш побитово
   прежними и снимает превью;
4. ОТМЕНА (Undo): применение таймером не создаёт и не освобождает датаблоков; после шага отмены, поставленного
   ДО точного результата (как у модального оператора), `ed.undo`/`ed.redo` оставляют сцену целой, а расхождение
   ползунка и меша после Redo сверяет `reconcile_after_history` и заказывает пересчёт;
5. РЕГИСТРАЦИЯ: оператор `hotspotuv.adjust_decal_width` объявлен с UNDO, оверлей и обработчики (история, загрузка,
   depsgraph) стоят;
6. ЦЕЛЬ — СОБСТВЕННАЯ ДЕКАЛЬ АКТИВНОГО ОБЪЕКТА (ошибка владельца: «аджастмент есть, а сетки для аджастмента нет»):
   сборка на меше A, переход на меш B без декали — инструмент недоступен с причиной «Build Decal Mesh first for B»
   (`poll` оператора, `begin_adjust` клавиши, путь ползунка ничего не заказывает и не рисует); сборка на B открывает
   его, возврат к A закрывает (запись кнопки одна на окно); смена активного снимает превью, но принятый заказ
   доезжает до декали прежнего объекта; поле подтягивается к ширине меша без пересчёта; активная декаль ведёт к своему
   источнику; удаление декали закрывает инструмент и снимает линии;
7. КОЛЬЦО КУПОЛА ПОД СУЖЕННОЙ ДОСЯГАЕМОСТЬЮ: карта купола при умолчании досягаемости отказывает швом, кнопка строит её под
   `alpha * (1 + b)` (0.3 м при ширине 0.25); ширина 0.3 через путь ползунка пересобирает карту под 0.36 м (своя
   карта, а не прежняя 0.3 м), меш применён на месте и ПОБИТОВО равен прямому нажатию в холодной сессии;
8. ПРЕВЬЮ МЕША (`PREVIEW_MESH_FROM_INTERVAL_V1`, ПРИБЛИЗИТЕЛЬНОЕ, `envelope_width_mesh_preview`): вход инструмента заказывает затравку модели (два точных
   прогона рядом, в меш не пишутся), настоящий меш двигается каждый кадр перетаскивания (позиции и UV; датаблоки, указатели и свойство ширины
   меша те же, заказа точного счёта нет), на ширине базы он побитово база, отмена возвращает его побитово, подтверждение даёт точный меш,
   равный холодной кнопке (отклонение превью от него названо числом), а меш чужого состава снимает модель с названной причиной;
9. ВЛАДЕНИЕ МЕШЕМ (аудит ad6074f, F2): превью пишется только в меш, который записал точный путь. Свои записи (кнопка, кадры, точный результат)
   обработчик depsgraph не принимает за чужие; подмена датаблока с теми же счётчиками и свойствами, правка раскладки при тех же счётчиках
   (в Object и через Edit с правкой UV), обновление геометрии декали мимо нас и шаг Undo/Redo снимают модель с названной причиной, ничего
   не записав в меш, а точный путь после этого работает как прежде (меш равен холодной кнопке) и заводит владение и модель заново.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_DECAL_WIDTH_LIVE_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
import sys
import time

import bpy


REPO_ROOT = Path(__file__).resolve().parents[2]
for path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import _enter_edge_selection  # noqa: E402
from test_envelope_production_mesh import (  # noqa: E402
    DECAL,
    DOME,
    SOURCE,
    _assert_scene_links_only_live_objects,
    _build_dome_ring,
    _decal_objects,
    _decal_settings,
    _enable_background_undo,
    _fresh_scene,
    _redo_and_check,
    _rim_edges,
    _settings,
    _assert_reaches,
    _walk_every_datablock,
)


DRAG = (0.30, 0.31, 0.32, 0.33, 0.34)
#: Заказ вместе с превью обязан вернуться быстро: превью на шве из одного ребра стоит доли миллисекунды.
ORDER_BUDGET_SECONDS = 0.05


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _scheduler():
    scheduler = _controller().width_live
    assert scheduler is not None
    return scheduler


def _digest():
    from cftuv.envelope_production_mesh import mesh_content_digest

    return mesh_content_digest(bpy.data.objects[DECAL].data)


def _datablock_counts():
    return {
        name: len(getattr(bpy.data, name))
        for name in ("objects", "meshes", "materials", "texts", "grease_pencils", "collections")
        if hasattr(bpy.data, name)
    }


def _fresh(*, workers=0):
    source = _fresh_scene(workers=workers, alpha=0.25)
    controller = _controller()
    if controller is not None:
        old = controller.width_live
        if old is not None and bpy.app.timers.is_registered(old._callback):
            bpy.app.timers.unregister(old._callback)
        controller.width_live = None  # счётчики каждого сценария свои
        old = controller.alpha_preview
        if old is not None and bpy.app.timers.is_registered(old._callback):
            bpy.app.timers.unregister(old._callback)
        controller.alpha_preview = None
    return source


def _press(source):
    seam = [edge.index for edge in source.data.edges if edge.use_seam]
    assert seam
    if bpy.context.mode != "EDIT_MESH":
        bpy.context.view_layer.objects.active = source
        _enter_edge_selection(source, seam)
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    return bpy.data.objects[DECAL]


def _pump(*, timeout=120.0):
    """Таймер без главного цикла: вызывает зарегистрированную функцию и спит, сколько она просит."""

    scheduler = _scheduler()
    callback = scheduler._callback
    end = time.perf_counter() + timeout
    steps = []
    while True:
        started = time.perf_counter()
        delay = callback()
        steps.append(time.perf_counter() - started)
        if delay is None:
            if bpy.app.timers.is_registered(callback):
                bpy.app.timers.unregister(callback)
            return steps
        assert time.perf_counter() < end, "width live did not settle"
        time.sleep(delay)


def _drag(values, *, pause=0.01):
    """Значения через путь `update`; возвращает секунды главного потока на каждое изменение."""

    settings = _settings()
    spent = []
    for value in values:
        started = time.perf_counter()
        settings.envelope_debug_alpha = value
        spent.append(time.perf_counter() - started)
        time.sleep(pause)
    return spent


def _direct_digest_at(width, *, cold):
    """Прямое нажатие кнопки на `width` (тёплая либо холодная сессия) и дайджест меша, который она оставила."""

    settings = _settings()
    controller = _controller()
    if cold:
        controller.clear()
        controller.width_live = None
    settings.envelope_debug_alpha = width
    controller.supersede_preview("direct press")
    source = bpy.data.objects[SOURCE]
    seam = [edge.index for edge in source.data.edges if edge.use_seam]
    bpy.context.view_layer.objects.active = source
    _enter_edge_selection(source, seam)
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    return _digest()


def _run_five_changes(*, workers):
    from cftuv.envelope_width_preview import PREVIEW_BINARY64_V1

    source = _fresh(workers=workers)
    decal = _press(source)
    controller = _controller()
    settings = _settings()
    record = controller.width_build
    assert record is not None and record.preview_inputs.runs, "the button must remember the build"
    assert controller.width_preview is None
    before = _digest()
    mesh_pointer, object_pointer = decal.data.as_pointer(), decal.as_pointer()
    blocks = _datablock_counts()

    spent = _drag(DRAG)
    scheduler = _scheduler()
    # 1. Заказ: быстро (превью внутри), превью есть, меш не тронут, таймер взведён, счёт не стартовал.
    assert max(spent) < ORDER_BUDGET_SECONDS, spent
    state = controller.width_preview
    assert state is not None and state.preview.method == PREVIEW_BINARY64_V1
    assert abs(state.preview.width - float(settings.envelope_debug_alpha)) < 1e-12
    assert state.preview.lines >= 2 and state.preview.points >= 4  # шов двух патчей: линия с каждой стороны
    assert state.source_name == SOURCE and state.serial == 5
    assert _digest() == before
    counters = scheduler.counters
    assert (counters.requested, counters.started, counters.applied) == (5, 0, 0), counters
    assert bpy.app.timers.is_registered(scheduler._callback)
    assert scheduler.status_text == "computing width=0.34... | superseded 4", scheduler.status_text
    assert any("PREVIEW_BINARY64_V1 preview, not final" in line for line in _status_lines())

    # 2. Шаги таймера: ОДНО значение, последнее; применено на месте.
    steps = _pump()
    counters = scheduler.counters
    assert (counters.started, counters.applied, counters.coalesced, counters.stale) == (1, 1, 4, 0), counters
    assert counters.failed == 0 and counters.invalid == 0, counters
    assert scheduler.status_text.startswith("ready width=0.34"), scheduler.status_text
    decal = bpy.data.objects[DECAL]
    assert (decal.data.as_pointer(), decal.as_pointer()) == (mesh_pointer, object_pointer)
    assert _datablock_counts() == blocks, (blocks, _datablock_counts())
    assert decal.data["cftuv_decal_width"] == float(settings.envelope_debug_alpha)
    assert controller.width_preview is None, "the exact result for the latest width clears the overlay"
    live_digest = _digest()
    assert live_digest != before
    assert len(_decal_objects()) == 1

    # 3. Побитово тот же меш, что у кнопки: тёплая сессия и холодная.
    last = float(settings.envelope_debug_alpha)
    assert _direct_digest_at(last, cold=False) == live_digest
    assert _direct_digest_at(last, cold=True) == live_digest
    latency = scheduler.last_applied
    print(
        f"FIVE_CHANGES workers={workers}: change max {max(spent) * 1000:.3f} ms (preview "
        f"{state.preview.seconds * 1000:.3f} ms, {state.preview.lines} lines), steps {len(steps)} "
        f"max {max(steps) * 1000:.1f} ms, exact latency {latency.latency_seconds * 1000:.0f} ms "
        f"(compute {latency.compute_seconds * 1000:.0f}, apply {latency.apply_seconds * 1000:.0f})"
    )
    return source


def _run_the_width_tool_rebuilds_a_tightened_dome_ring_for_the_new_width():
    from cftuv.envelope_production_mesh import mesh_content_digest

    source = _build_dome_ring()
    controller = _controller()
    for name in ("width_live", "alpha_preview"):
        old = getattr(controller, name)
        if old is not None and bpy.app.timers.is_registered(old._callback):
            bpy.app.timers.unregister(old._callback)
        setattr(controller, name, None)
    settings = _settings()

    def press():
        bpy.context.view_layer.objects.active = source
        _enter_edge_selection(source, _rim_edges(source))
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        return _decal_settings().status, mesh_content_digest(bpy.data.objects[DOME + ".CFTUV_Decal"].data)

    settings.envelope_debug_alpha = 0.25
    status, narrow = press()
    assert status == "MATERIALIZED 1 / refused 0", status
    _assert_reaches(controller, [0.3])
    _drag((0.3,))
    _pump()
    scheduler = _scheduler()
    counters = scheduler.counters
    assert (counters.started, counters.applied, counters.stale, counters.failed, counters.invalid) == (1, 1, 0, 0, 0), counters
    assert scheduler.status_text.startswith("ready width=0.3"), scheduler.status_text
    _assert_reaches(controller, [0.3, 0.36])
    live = mesh_content_digest(bpy.data.objects[DOME + ".CFTUV_Decal"].data)
    assert live != narrow
    # Побитово то же, что у кнопки на холодной сессии при ширине 0.3.
    controller.clear()
    controller.width_live = None
    settings.envelope_debug_alpha = 0.3
    controller.supersede_preview("direct press")
    status, direct = press()
    assert status == "MATERIALIZED 1 / refused 0", status
    assert direct == live
    print("DOME RING width tool: tightened reach rebuilt for 0.3, same mesh as a cold button press")


def _status_lines():
    from cftuv.envelope_width_live import status_lines

    return status_lines(_controller())


def _modal(width_events, *, confirm):
    """Модальный инструмент без интерфейса: автомат и исполнение те же, что у оператора."""

    from cftuv.envelope_width_adjust import KIND_CANCEL, KIND_CONFIRM, KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import ViewScaleV1, apply_step, begin_adjust, finish_adjust

    view = ViewScaleV1(pivot=(100.0, 100.0), metres_per_pixel=0.002)
    runtime = begin_adjust(bpy.context, (200.0, 100.0), view)
    assert not isinstance(runtime, str), runtime
    controller = _controller()
    assert controller.width_preview is not None  # линии видны ещё до первого движения
    widths = []
    for x in width_events:
        step = runtime.session.handle(WidthEventV1(KIND_MOVE, x, 100.0))
        apply_step(bpy.context, runtime, step)
        widths.append(step.width)
        assert controller.width_preview.preview.width == step.width  # превью следует за рукой
    closing = runtime.session.handle(WidthEventV1(KIND_CONFIRM if confirm else KIND_CANCEL))
    apply_step(bpy.context, runtime, closing)
    return runtime, widths, finish_adjust(bpy.context, runtime, confirmed=confirm)


def _run_the_modal_tool_drags_the_preview_and_the_buttons_answer_follows_only_a_confirm():
    source = _fresh()
    _press(source)
    controller = _controller()
    settings = _settings()
    scheduler_before = controller.width_live
    before = _digest()
    start = float(settings.envelope_debug_alpha)

    # Отмена: ширина и меш прежние побитово, превью снято, ни одного заказа точного счёта.
    runtime, widths, result = _modal((220.0, 260.0, 300.0, 240.0), confirm=False)
    assert result == "CANCELLED" and len(set(widths)) == 4
    assert float(settings.envelope_debug_alpha) == start
    assert _digest() == before
    assert controller.width_preview is None
    assert scheduler_before is None or scheduler_before.counters.requested == 0
    assert runtime.session.width == start
    assert controller.width_live is None or not controller.width_live.busy

    # Подтверждение: во время перетаскивания меш и счётчики те же; потом ОДИН заказ и точный меш.
    runtime, widths, result = _modal((220.0, 260.0, 300.0, 340.0), confirm=True)
    final = runtime.session.width
    assert result == "FINISHED" and final == widths[-1] and final > start
    assert float(settings.envelope_debug_alpha) == float(final) or abs(float(settings.envelope_debug_alpha) - final) < 1e-6
    scheduler = _scheduler()
    assert scheduler.counters.requested == 1, scheduler.counters
    assert _digest() == before  # точный счёт ещё не применён: превью — не меш
    assert controller.width_preview is not None
    _pump()
    assert scheduler.counters.applied == 1 and scheduler.counters.failed == 0
    assert controller.width_preview is None
    live = _digest()
    assert live != before
    assert _direct_digest_at(float(settings.envelope_debug_alpha), cold=False) == live
    print(f"MODAL widths {[round(item, 4) for item in widths]} -> exact apply ok")

    # Загрузка файла: запись кнопки и превью старой сцены забыты, счёт остановлен.
    from cftuv.envelope_width_modal import _after_load

    settings.envelope_debug_alpha = 0.5
    assert controller.width_preview is not None and controller.width_live.busy
    _after_load()
    assert controller.width_preview is None and controller.width_build is None
    assert not controller.width_live.busy


def _width_property(decal):
    return bpy.data.objects[decal].data["cftuv_decal_width"]


def _pump_all(*, timeout=120.0):
    """Таймеры обоих планировщиков (точный счёт и затравка) шагают, пока оба не опустеют."""

    controller = _controller()
    end = time.perf_counter() + timeout
    while True:
        busy = [item for item in (controller.width_live, controller.width_prime) if item is not None and item.busy]
        if not busy:
            return
        assert time.perf_counter() < end, "the schedulers did not settle"
        for scheduler in busy:
            scheduler._callback()
        time.sleep(0.01)


def _mesh_arrays(name):
    import numpy as np

    mesh = bpy.data.objects[name].data
    co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", co)
    uv = np.empty(len(mesh.uv_layers["UVMap"].data) * 2, dtype=np.float32)
    mesh.uv_layers["UVMap"].data.foreach_get("uv", uv)
    return co, uv


def _run_the_preview_mesh_follows_the_hand_inside_the_model_and_the_exact_result_replaces_it():
    """ПРЕВЬЮ МЕША (`PREVIEW_MESH_FROM_INTERVAL_V1`): настоящий меш двигается между точными пересчётами, и точный меш потом тот же, что у кнопки.

    1. кнопка кладёт образец и владение мешем, модели нет; вход инструмента заказывает ЗАТРАВКУ (точный прогон рядом, в меш не пишет), после неё
       модель есть (квадрат), а меш и датаблоки те же;
    2. перетаскивание теми же функциями, что у оператора: меш двигается КАЖДЫЙ кадр (позиции и UV), датаблоки, указатели и свойство
       ширины меша те же, строка статуса называет превью; кадр в пределах интервала близок к точному (отклонение названо числом);
    3. отмена возвращает меш побитово; подтверждение даёт точный меш, равный холодной кнопке, и новую модель;
    4. отказ назван: после правки состава меша (чужой меш) модель снята именем и кадров нет.
    """

    import numpy as np

    from cftuv.envelope_width_adjust import KIND_CANCEL, KIND_CONFIRM, KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import ViewScaleV1, apply_step, begin_adjust, finish_adjust

    source = _fresh()
    _press(source)
    controller = _controller()
    settings = _settings()
    start = float(settings.envelope_debug_alpha)
    assert controller.width_displayed is not None and controller.width_model is None
    assert controller.width_mesh_owner is not None and controller.width_mesh_owner.sample is controller.width_displayed
    assert controller.width_displayed.alpha == start and controller.width_displayed.arrays_digest

    mesh_pointer, object_pointer = bpy.data.objects[DECAL].data.as_pointer(), bpy.data.objects[DECAL].as_pointer()
    blocks = _datablock_counts()
    before = _digest()
    still = _mesh_arrays(DECAL)

    # 1. Вход инструмента: затравка (две подряд), модель, меш не тронут.
    view = ViewScaleV1(pivot=(0.0, 0.0), metres_per_pixel=1.0)
    runtime = begin_adjust(bpy.context, (0.0, 0.0), view)
    assert not isinstance(runtime, str), runtime
    assert controller.width_prime is not None and controller.width_prime.busy, "the tool entry orders the preview-model prime"
    _pump_all()
    model = controller.width_model
    assert model is not None, controller.width_model_refusal
    assert model.modelled_domains >= 1 and model.quadratic_domains == model.modelled_domains
    assert controller.width_prime.counters.applied == 2 and controller.width_prime.counters.failed == 0
    assert controller.width_live is None or controller.width_live.counters.requested == 0, "the prime never goes through the width order"
    assert _digest() == before and _datablock_counts() == blocks
    assert _width_property(DECAL) == start
    assert any(line.startswith("Preview model (approximate") for line in _status_lines())
    print(
        f"PREVIEW MODEL {model.modelled_domains}/{model.domain_count} domains, {model.quadratic_domains} quadratic, "
        f"{model.own_bytes} B (+{model.base.nbytes} B base)"
    )

    # 2. Перетаскивание: каждый кадр двигает меш; всё остальное на месте.
    frames = []
    for offset in (0.002, 0.004, 0.006, 0.008, 0.004, 0.0):
        step = runtime.session.handle(WidthEventV1(KIND_MOVE, offset, 0.0))
        apply_step(bpy.context, runtime, step)
        state = controller.width_mesh_preview
        assert state is not None and state.outcome == "PREVIEW_MESH_FROM_INTERVAL_V1", state
        frames.append((step.width, _mesh_arrays(DECAL)))
    assert abs(frames[0][0] - (start + 0.002)) < 1e-9 and abs(frames[-1][0] - start) < 1e-9
    moving = [width for width, (co, _uv) in frames[:-1] if not np.array_equal(co, still[0])]
    assert len(moving) == 5, "every frame of the drag moves the real mesh"
    assert np.array_equal(frames[-1][1][0], still[0]) and np.array_equal(frames[-1][1][1], still[1]), "the base width is the base mesh, bitwise"
    assert _datablock_counts() == blocks
    decal = bpy.data.objects[DECAL]
    assert (decal.data.as_pointer(), decal.as_pointer()) == (mesh_pointer, object_pointer)
    assert _width_property(DECAL) == start and float(settings.envelope_debug_alpha) == start
    assert (controller.width_live is None or controller.width_live.counters.requested == 0), "no exact order during the drag"
    step = runtime.session.handle(WidthEventV1(KIND_MOVE, 0.006, 0.0))
    apply_step(bpy.context, runtime, step)
    assert any("PREVIEW_MESH_FROM_INTERVAL_V1 preview (approximate, not certified)" in line for line in _status_lines())
    preview_co, preview_uv = _mesh_arrays(DECAL)
    assert controller.width_mesh_preview.seconds < 0.03, controller.width_mesh_preview.seconds

    # 3. Отмена: побитово. Затем настоящее подтверждение: меш точный и равен холодной кнопке; превью против точного — числом.
    closing = runtime.session.handle(WidthEventV1(KIND_CANCEL))
    apply_step(bpy.context, runtime, closing)
    assert finish_adjust(bpy.context, runtime, confirmed=False) == "CANCELLED"
    assert _digest() == before and controller.width_mesh_preview is None
    runtime = begin_adjust(bpy.context, (0.0, 0.0), view)
    for offset in (0.002, 0.004, 0.006):
        step = runtime.session.handle(WidthEventV1(KIND_MOVE, offset, 0.0))
        apply_step(bpy.context, runtime, step)
    again = _mesh_arrays(DECAL)
    assert np.array_equal(again[0], preview_co) and np.array_equal(again[1], preview_uv), "the same width gives the same frame"
    closing = runtime.session.handle(WidthEventV1(KIND_CONFIRM))
    apply_step(bpy.context, runtime, closing)
    assert finish_adjust(bpy.context, runtime, confirmed=True) == "FINISHED"
    assert controller.width_live.busy and controller.width_mesh_preview is not None  # точного ещё нет: меш показывает превью
    _pump_all()
    exact_co, exact_uv = _mesh_arrays(DECAL)
    final = float(settings.envelope_debug_alpha)
    assert abs(final - (start + 0.006)) < 1e-6  # ползунок — float32
    deviation = float(np.max(np.abs(exact_co - preview_co))), float(np.max(np.abs(exact_uv - preview_uv)))
    print(f"PREVIEW vs EXACT at width {final:.4f}: max position {deviation[0]:.3e} m, max uv {deviation[1]:.3e}")
    assert deviation[0] < 1e-4 and deviation[1] < 1e-4, deviation
    assert controller.width_mesh_preview is None and controller.width_displayed.alpha == final
    assert _datablock_counts() == blocks
    assert controller.width_live.counters.failed == 0 and controller.width_live.counters.applied == 1
    earlier = dataclasses.asdict(controller.width_preview_log)  # прямое нажатие ниже сбрасывает сессию вместе с журналом
    live = _digest()
    assert live != before
    assert _direct_digest_at(final, cold=False) == live and _direct_digest_at(final, cold=True) == live

    # 4. Меш чужого состава: модель снята с названной причиной, кадров нет.
    _pump_all()
    runtime = begin_adjust(bpy.context, (0.0, 0.0), view)
    _pump_all()
    assert controller.width_model is not None
    decal = bpy.data.objects[DECAL]
    decal["cftuv_source_revision"] = str(decal["cftuv_source_revision"]) + "-edited"
    step = runtime.session.handle(WidthEventV1(KIND_MOVE, 0.002, 0.0))
    apply_step(bpy.context, runtime, step)
    assert controller.width_model is None and controller.width_mesh_preview is None
    assert any("PREVIEW_MODEL_DROPPED:PREVIEW_MESH_NOT_THE_BASE" in line for line in _status_lines()), _status_lines()
    log = controller.width_preview_log
    assert log.dropped == {"PREVIEW_MODEL_DROPPED:PREVIEW_MESH_NOT_THE_BASE": 1}, log.dropped
    assert earlier["frames"] >= 12 and earlier["models"] >= 3 and earlier["checks"] >= 1, earlier
    assert earlier["max_position_error"] < 1e-4 and earlier["domains_refuted"] == 0, earlier
    closing = runtime.session.handle(WidthEventV1(KIND_CANCEL))
    apply_step(bpy.context, runtime, closing)
    finish_adjust(bpy.context, runtime, confirmed=False)
    print(
        f"PREVIEW MESH: {earlier['frames']} frames, max {earlier['frame_seconds_max'] * 1000:.2f} ms; models {earlier['models']}; "
        f"self-check max {earlier['max_position_error']:.3e} m over {earlier['domains_checked']} domains; dropped {dict(log.dropped)}"
    )


def _layout_of(mesh):
    """Отпечаток раскладки меша теми же функциями, что и кадр (`_read_layout`): индексы вершин петель и начала граней."""

    import numpy as np

    from cftuv.envelope_width_mesh_preview import _read_layout

    return _read_layout(mesh, np.empty(len(mesh.loops), dtype=np.int32), np.empty(len(mesh.polygons), dtype=np.int32))


def _manual_layout_change(mesh):
    """Ручная правка раскладки в режиме Object: те же вершины, петли, грани и свойства, но порядок вершин одной грани обратный."""

    import bmesh

    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
        bm.faces.ensure_lookup_table()
        bmesh.ops.reverse_faces(bm, faces=[bm.faces[0]])
        bm.to_mesh(mesh)
    finally:
        bm.free()


def _tool_ready(*, undo=False):
    """Кнопка, вход инструмента и затравка: у сессии есть модель и владение мешем; depsgraph доложил о нашей записи и ничего не снял."""

    from cftuv.envelope_width_session import ViewScaleV1, begin_adjust

    source = _fresh()
    if undo:
        _enable_background_undo(source)
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh(True) == {"FINISHED"}
    else:
        _press(source)
    bpy.ops.object.mode_set(mode="OBJECT")
    if undo:
        bpy.ops.ed.undo_push(message="Decal built")
    controller = _controller()
    runtime = begin_adjust(bpy.context, (0.0, 0.0), ViewScaleV1(pivot=(0.0, 0.0), metres_per_pixel=1.0))
    assert not isinstance(runtime, str), runtime
    _pump_all()
    assert controller.width_model is not None and controller.width_mesh_owner is not None, controller.width_model_refusal
    bpy.context.view_layer.update()
    assert controller.width_model is not None, "the depsgraph report of our own write must not drop the model"
    return controller, runtime


def _drag_frame(runtime, offset):
    from cftuv.envelope_width_adjust import KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import apply_step

    apply_step(bpy.context, runtime, runtime.session.handle(WidthEventV1(KIND_MOVE, offset, 0.0)))


def _dropped_names(controller):
    return set(controller.width_preview_log.dropped)


def _the_exact_path_recovers_and_owns_the_mesh_again(controller, runtime, start):
    """После снятия модели точный путь работает как прежде: меш равен холодной кнопке, владение и модель заведены заново, кадры снова идут."""

    settings = _settings()
    _pump_all()
    generation = controller.width_layout_generation
    width = start + 0.01
    for item in bpy.context.view_layer.objects:
        item.select_set(item == bpy.data.objects[SOURCE])
    bpy.context.view_layer.objects.active = bpy.data.objects[SOURCE]
    settings.envelope_debug_alpha = width
    _pump_all()
    assert controller.width_live.counters.failed == 0
    live = _digest()
    assert controller.width_layout_generation == generation + 1 and controller.width_mesh_owner is not None
    assert controller.width_mesh_owner.sample is controller.width_displayed
    assert controller.width_model is not None, controller.width_model_refusal  # затравка в конце применения заказала модель заново
    bpy.context.view_layer.update()
    assert controller.width_model is not None
    _drag_frame(runtime, 0.007)  # другая ширина, чем на прежних кадрах: автомат не двигает кадр на той же
    assert controller.width_mesh_preview is not None and controller.width_model is not None
    assert _direct_digest_at(float(settings.envelope_debug_alpha), cold=True) == live, "the exact result equals the cold button"


def _run_the_preview_mesh_belongs_only_to_the_mesh_the_exact_path_wrote():
    import bmesh
    import numpy as np

    from cftuv.envelope_width_adjust import KIND_CANCEL, WidthEventV1
    from cftuv.envelope_width_session import apply_step, finish_adjust

    drop = "PREVIEW_MODEL_DROPPED:"

    # a. Свои записи (кнопка, кадры, возврат базы) обработчик depsgraph не принимает за чужие.
    controller, runtime = _tool_ready()
    start = float(_settings().envelope_debug_alpha)
    for offset in (0.002, 0.004, 0.006):
        _drag_frame(runtime, offset)
        bpy.context.view_layer.update()
    assert controller.width_model is not None and controller.width_preview_log.dropped == {} and controller.width_preview_log.frames == 3
    closing = runtime.session.handle(WidthEventV1(KIND_CANCEL))
    apply_step(bpy.context, runtime, closing)
    assert finish_adjust(bpy.context, runtime, confirmed=False) == "CANCELLED"
    bpy.context.view_layer.update()
    assert controller.width_model is not None and controller.width_preview_log.dropped == {}
    assert controller.width_layout_generation >= 1 and controller.width_preview_log.layout_generation == controller.width_layout_generation

    # b. Другой датаблок с теми же счётчиками, раскладкой и пользовательскими свойствами (копия): тождество его не признаёт, записи нет.
    controller, runtime = _tool_ready()
    decal = bpy.data.objects[DECAL]
    original = decal.data
    twin = original.copy()
    assert twin["cftuv_decal_width"] == original["cftuv_decal_width"] and _layout_of(twin) == _layout_of(original)
    assert twin.as_pointer() != original.as_pointer() and twin.session_uid != original.session_uid
    decal.data = twin
    still = _mesh_arrays(DECAL)
    _drag_frame(runtime, 0.004)
    assert controller.width_model is None and controller.width_mesh_preview is None
    assert controller.width_preview_log.dropped == {drop + "PREVIEW_MESH_DATABLOCK_REPLACED": 1}, controller.width_preview_log.dropped
    moved = _mesh_arrays(DECAL)
    assert np.array_equal(moved[0], still[0]) and np.array_equal(moved[1], still[1]), "nothing was written into a mesh we do not own"
    _the_exact_path_recovers_and_owns_the_mesh_again(controller, runtime, start)

    # c. Раскладку изменили на месте при тех же счётчиках (порядок вершин грани), сигнала depsgraph нет: сеть (перепроверка отпечатка) видит.
    controller, runtime = _tool_ready()
    decal = bpy.data.objects[DECAL]
    owner = controller.width_mesh_owner
    counts = (len(decal.data.vertices), len(decal.data.loops), len(decal.data.polygons), len(decal.data.edges))
    _manual_layout_change(decal.data)
    assert (len(decal.data.vertices), len(decal.data.loops), len(decal.data.polygons), len(decal.data.edges)) == counts
    assert _layout_of(decal.data) != owner.layout, "the same counts, another loop-to-vertex order"
    still = _mesh_arrays(DECAL)
    owner.verified_at -= 10.0
    _drag_frame(runtime, 0.004)
    assert controller.width_model is None
    assert controller.width_preview_log.dropped == {drop + "PREVIEW_MESH_LAYOUT_CHANGED": 1}, controller.width_preview_log.dropped
    assert controller.width_preview_log.layout_checks == 1
    moved = _mesh_arrays(DECAL)
    assert np.array_equal(moved[0], still[0]) and np.array_equal(moved[1], still[1])
    _the_exact_path_recovers_and_owns_the_mesh_again(controller, runtime, start)

    # d. То же, но depsgraph докладывает об обновлении геометрии: настоящий обработчик снимает модель сразу, до любого кадра.
    controller, runtime = _tool_ready()
    _manual_layout_change(bpy.data.objects[DECAL].data)
    bpy.context.view_layer.update()
    assert controller.width_model is None and controller.width_mesh_owner is None
    assert controller.width_preview_log.dropped == {drop + "PREVIEW_DECAL_CHANGED_EXTERNALLY": 1}, controller.width_preview_log.dropped
    still = _mesh_arrays(DECAL)
    _drag_frame(runtime, 0.004)
    assert controller.width_mesh_preview is None and np.array_equal(_mesh_arrays(DECAL)[0], still[0])
    _the_exact_path_recovers_and_owns_the_mesh_again(controller, runtime, start)

    # e. Edit декали с ручной правкой раскладки и UV и выход из Edit: модель снята (сигналом либо режимом), меш не перезаписан превью.
    controller, runtime = _tool_ready()
    decal = bpy.data.objects[DECAL]
    for item in bpy.context.view_layer.objects:
        item.select_set(item == decal)
    bpy.context.view_layer.objects.active = decal
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.view_layer.update()
    _drag_frame(runtime, 0.002)
    assert controller.width_model is None and controller.width_mesh_preview is None
    first = _dropped_names(controller)
    assert first and first <= {drop + "PREVIEW_DECAL_CHANGED_EXTERNALLY", drop + "PREVIEW_DECAL_IN_EDIT_MODE"}, first
    bm = bmesh.from_edit_mesh(decal.data)
    bm.faces.ensure_lookup_table()
    bmesh.ops.reverse_faces(bm, faces=[bm.faces[0]])
    uv_layer = bm.loops.layers.uv.verify()
    bm.faces[0].loops[0][uv_layer].uv.x += 0.05
    bmesh.update_edit_mesh(decal.data)
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.update()
    manual = _mesh_arrays(DECAL)
    _drag_frame(runtime, 0.004)
    assert controller.width_model is None and np.array_equal(_mesh_arrays(DECAL)[0], manual[0]) and np.array_equal(_mesh_arrays(DECAL)[1], manual[1])
    print(f"EDIT ROUND TRIP: the model was dropped by {sorted(first)}")
    _the_exact_path_recovers_and_owns_the_mesh_again(controller, runtime, start)

    # f. Undo/Redo: шаг истории снимает модель (строго), сцена целая, точный путь заводит всё заново.
    from cftuv.envelope_width_modal import _reconcile_once

    controller, runtime = _tool_ready(undo=True)
    bpy.ops.ed.undo_push(message="Adjust Decal Width")
    _drag_frame(runtime, 0.004)
    assert controller.width_mesh_preview is not None
    assert bpy.ops.ed.undo() == {"FINISHED"}
    _assert_scene_links_only_live_objects()
    bpy.context.view_layer.update()
    _walk_every_datablock()
    handler_ran = bpy.app.timers.is_registered(_reconcile_once)
    if handler_ran:
        bpy.app.timers.unregister(_reconcile_once)
    _reconcile_once()  # то, что делает таймер обработчика undo_post
    controller = _controller()
    assert controller.width_model is None and controller.width_mesh_owner is None and controller.width_mesh_preview is None
    names = _dropped_names(controller)
    assert names and names <= {drop + "PREVIEW_HISTORY_STEP", drop + "PREVIEW_DECAL_CHANGED_EXTERNALLY"}, names
    _drag_frame(runtime, 0.006)
    assert controller.width_mesh_preview is None, "no frame without a model"
    _pump_all()
    _redo_and_check()
    _reconcile_once()
    _pump_all()
    assert _controller().width_model is None
    print(f"UNDO/REDO: handler timer registered by undo_post: {handler_ran}; model dropped by {sorted(names)}")
    _the_exact_path_recovers_and_owns_the_mesh_again(_controller(), runtime, start)
    print("OWNERSHIP: own writes pass; replaced datablock, other loop order, foreign geometry update, Edit round trip and Undo/Redo drop the preview by name")


def _run_timer_writes_keep_the_scene_and_the_undo_history_consistent():
    from cftuv.envelope_width_live import reconcile_after_history

    source = _fresh()
    _enable_background_undo(source)
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh(True) == {"FINISHED"}
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.ed.undo_push(message="Decal built")  # состояние Build: ширина 0.25, меш прежний
    controller = _controller()
    settings = _settings()
    blocks = _datablock_counts()
    initial = _digest()

    # Как модальный оператор: подтверждение, шаг отмены ДО точного результата, потом таймер применяет меш.
    settings.envelope_debug_alpha = 0.31
    bpy.ops.ed.undo_push(message="Adjust Decal Width")
    _pump()
    assert _scheduler().counters.applied == 1
    assert _datablock_counts() == blocks, "the timer must not create or free datablocks"
    applied = _digest()
    assert applied != initial

    # Ctrl+Z владельца: возврат к состоянию кнопки, сцена целая.
    assert bpy.ops.ed.undo() == {"FINISHED"}
    _assert_scene_links_only_live_objects()
    bpy.context.view_layer.update()
    _walk_every_datablock()
    settings = _settings()
    print(
        f"AFTER UNDO: slider {float(settings.envelope_debug_alpha)}, mesh width {_width_property(DECAL)}, "
        f"mesh is the button's {_digest() == initial}"
    )
    assert _digest() == initial and float(settings.envelope_debug_alpha) == 0.25
    reconcile_after_history()
    assert _controller().width_preview is None
    assert not _scheduler().busy  # ползунок и меш равны: пересчитывать нечего

    # Ctrl+Shift+Z: ползунок вернулся на 0.31, а меш мог остаться прежним (шаг записан до точного результата).
    _redo_and_check()
    settings = _settings()
    stale = _digest() != applied
    print(
        f"AFTER REDO: slider {float(settings.envelope_debug_alpha)}, mesh width {_width_property(DECAL)}, "
        f"mesh is stale {stale}"
    )
    assert abs(float(settings.envelope_debug_alpha) - 0.31) < 1e-6
    reconcile_after_history()
    assert _controller().width_preview is None
    assert _scheduler().busy == stale  # расхождение заказывает точный пересчёт, равенство — нет
    _pump()
    _walk_every_datablock()
    assert _digest() == applied
    assert abs(_width_property(DECAL) - float(settings.envelope_debug_alpha)) < 1e-6
    assert _scheduler().counters.failed == 0
    # Дальше ползунок снова работает: ещё один заказ применяется.
    settings.envelope_debug_alpha = 0.36
    _pump()
    assert abs(_width_property(DECAL) - float(settings.envelope_debug_alpha)) < 1e-6
    _walk_every_datablock()
    print("UNDO timer-applied width writes: datablocks unchanged, undo/redo clean, history reconciled")


def _twin(source, name):
    """Второй меш: копия источника (те же швы) под другим именем, без декали."""

    twin = source.copy()
    twin.data = source.data.copy()
    twin.name = name
    bpy.context.scene.collection.objects.link(twin)
    return twin


def _activate(obj):
    """Активным становится `obj` в режиме Object (как щелчок в 3D View)."""

    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for item in bpy.context.view_layer.objects:
        item.select_set(item == obj)
    bpy.context.view_layer.objects.active = obj


def _build_on(obj):
    seam = [edge.index for edge in obj.data.edges if edge.use_seam]
    assert seam
    _activate(obj)
    _enter_edge_selection(obj, seam)
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    bpy.ops.object.mode_set(mode="OBJECT")
    return bpy.data.objects[f"{obj.name}.CFTUV_Decal"]


def _drop_sync_timer():
    from cftuv.envelope_width_modal import _sync_once

    if bpy.app.timers.is_registered(_sync_once):
        bpy.app.timers.unregister(_sync_once)


def _view_context():
    """Контекст с областью 3D View: у фонового Blender её нет, а `poll` оператора спрашивает о ней первой."""

    from types import SimpleNamespace

    return SimpleNamespace(
        area=SimpleNamespace(type="VIEW_3D"),
        active_object=bpy.context.view_layer.objects.active,
        window_manager=bpy.context.window_manager,
        scene=bpy.context.scene,
    )


def _run_the_tool_belongs_to_the_active_objects_own_decal():
    from cftuv.envelope_width_live import follow_active_object, preview_now, sync_width_field, width_problem
    from cftuv.envelope_width_modal import HOTSPOTUV_OT_AdjustDecalWidth, _after_depsgraph, _sync_once
    from cftuv.envelope_width_session import ViewScaleV1, begin_adjust, poll_problem

    source = _fresh()
    _press(source)
    bpy.ops.object.mode_set(mode="OBJECT")
    controller, settings = _controller(), _settings()
    name_a, name_b = source.name, source.name + "B"
    view = ViewScaleV1(pivot=(100.0, 100.0), metres_per_pixel=0.002)
    poll = HOTSPOTUV_OT_AdjustDecalWidth.poll

    # A построен: инструмент доступен, ширина живая.
    assert width_problem(bpy.context) == "" and poll(_view_context())
    settings.envelope_debug_alpha = 0.33
    _pump()
    assert abs(_width_property(DECAL) - 0.33) < 1e-6

    # Новая ширина на A ещё в пути, а активным становится B без декали: превью снято, заказ A не пропал.
    settings.envelope_debug_alpha = 0.36
    assert controller.width_preview is not None and _scheduler().busy
    twin = _twin(source, name_b)
    _activate(twin)
    _after_depsgraph()  # то, что делает обработчик depsgraph при смене активного
    assert controller.width_target == name_b and controller.width_preview is None
    assert bpy.app.timers.is_registered(_sync_once)
    _drop_sync_timer()
    assert _scheduler().busy, "the order accepted for A is not dropped silently"

    # B без своей декали: причина названа, кнопка (poll) закрыта, клавиша (invoke) называет ту же причину.
    reason = f"Build Decal Mesh first for {name_b}"
    assert poll_problem(bpy.context) == reason == width_problem(bpy.context)
    assert not poll(_view_context())
    refusal = begin_adjust(bpy.context, (200.0, 100.0), view)
    assert refusal == reason, refusal
    requested = _scheduler().counters.requested
    settings.envelope_debug_alpha = 0.5  # поле чужого объекта: ничего не заказывается, чужих линий нет
    assert _scheduler().counters.requested == requested and controller.width_preview is None
    _pump()  # заказ A (0.36) доехал до ЕГО декали, а 0.5 на B никуда не ушло
    assert abs(_width_property(DECAL) - 0.36) < 1e-6, _width_property(DECAL)
    digest_a = _digest()
    assert name_b + ".CFTUV_Decal" not in bpy.data.objects
    assert _scheduler().counters.applied == 2 and _scheduler().counters.requested == requested
    print(f"TARGET: B without a decal -> {reason!r}; the order for A applied, none for B")

    # Сборка на B открывает инструмент для B и закрывает для A (запись кнопки одна на окно).
    decal_b = _build_on(twin)
    assert width_problem(bpy.context) == "" and poll(_view_context())
    assert abs(_width_property(decal_b.name) - 0.5) < 1e-6
    _activate(source)
    _after_depsgraph()
    _drop_sync_timer()
    problem_a = width_problem(bpy.context)
    assert problem_a == (
        f"Build Decal Mesh first for {name_a}: the build session of this window belongs to {name_b}"
    ), problem_a
    assert not poll(_view_context()) and begin_adjust(bpy.context, (200.0, 100.0), view) == problem_a
    assert abs(_width_property(DECAL) - 0.36) < 1e-6

    # Вернулись к B: поле подтягивается к ширине ЕГО меша (ползунок ушёл, пока был A), пересчёт не заказан.
    settings.envelope_debug_alpha = 0.9
    _activate(twin)
    _after_depsgraph()
    assert controller.width_target == name_b
    before = (controller.width_live.counters.requested, _width_property(decal_b.name))
    assert sync_width_field(bpy.context) is True
    assert abs(float(settings.envelope_debug_alpha) - 0.5) < 1e-6
    assert (controller.width_live.counters.requested, _width_property(decal_b.name)) == before
    _drop_sync_timer()

    # Активна сама декаль B: цель — её источник.
    _activate(decal_b)
    assert width_problem(bpy.context) == ""
    _activate(twin)

    # Декаль B удалена: инструмент снова недоступен, линии под ней сняты, ползунок ничего не заказывает.
    preview_now(controller, 0.4, 0.02)
    assert controller.width_preview is not None
    mesh_b = decal_b.data
    bpy.data.objects.remove(decal_b)
    bpy.data.meshes.remove(mesh_b)
    assert follow_active_object(bpy.context) is False  # активный тот же, но цели под линиями нет
    assert controller.width_preview is None
    assert width_problem(bpy.context) == reason and not poll(_view_context())
    requested = controller.width_live.counters.requested
    settings.envelope_debug_alpha = 0.7
    assert controller.width_live.counters.requested == requested and controller.width_preview is None
    print(f"TARGET: decal of B deleted -> {width_problem(bpy.context)!r}")

    # Перестроили B: снова доступен.
    _build_on(twin)
    assert width_problem(bpy.context) == "" and poll(_view_context())
    assert _digest() == digest_a, "the decal of A is not touched by anything done for B"
    print("TARGET: A -> B without decal -> build B -> A closed -> decal deleted -> rebuilt: availability follows the active object")


def _run_blender_events_become_the_events_of_the_automaton():
    """Перевод событий оператора: те же правила, что исполняет `modal`; события здесь — подставные."""

    from types import SimpleNamespace

    from cftuv import envelope_width_adjust as adjust
    from cftuv.envelope_width_modal import translate_event

    def event(kind, value="PRESS", *, x=300.0, y=250.0, ctrl=False, shift=False):
        return SimpleNamespace(type=kind, value=value, mouse_x=x, mouse_y=y, ctrl=ctrl, shift=shift)

    origin, last = (100.0, 50.0), (10.0, 20.0)
    moved = translate_event(event("MOUSEMOVE", "NOTHING", ctrl=True), origin, last)
    assert (moved.kind, moved.x, moved.y, moved.ctrl, moved.shift) == (adjust.KIND_MOVE, 200.0, 200.0, True, False)
    # Модификаторы пересчитывают ширину в ТОЙ ЖЕ точке: привязка и точность видны сразу.
    modifier = translate_event(event("LEFT_CTRL", ctrl=True), origin, last)
    assert (modifier.kind, modifier.x, modifier.y, modifier.ctrl) == (adjust.KIND_MOVE, 10.0, 20.0, True)
    for kind in ("LEFTMOUSE", "RET", "NUMPAD_ENTER"):
        assert translate_event(event(kind), origin, last).kind == adjust.KIND_CONFIRM, kind
    for kind in ("ESC", "RIGHTMOUSE"):
        assert translate_event(event(kind), origin, last).kind == adjust.KIND_CANCEL, kind
    assert translate_event(event("LEFTMOUSE", "RELEASE"), origin, last) is None
    digits = [translate_event(event(name), origin, last) for name in ("ZERO", "NINE", "NUMPAD_7")]
    assert [(item.kind, item.char) for item in digits] == [(adjust.KIND_DIGIT, "0"), (adjust.KIND_DIGIT, "9"), (adjust.KIND_DIGIT, "7")]
    assert translate_event(event("PERIOD"), origin, last).kind == adjust.KIND_POINT
    assert translate_event(event("BACK_SPACE"), origin, last).kind == adjust.KIND_BACKSPACE
    for kind in ("WHEELUPMOUSE", "WHEELDOWNMOUSE", "MIDDLEMOUSE"):
        assert translate_event(event(kind, "NOTHING"), origin, last) == "PASS", kind  # навигация вида
    assert translate_event(event("A"), origin, last) is None  # чужая клавиша проглочена


def _run_the_tool_is_registered_with_undo_and_the_overlay_and_history_handlers_stand():
    from cftuv.envelope_width_modal import (
        HOTSPOTUV_OT_AdjustDecalWidth,
        _after_depsgraph,
        _after_history,
        _after_load,
    )
    from cftuv.envelope_width_overlay import line_segments, overlay_registered

    assert hasattr(bpy.ops.hotspotuv, "adjust_decal_width")
    assert set(HOTSPOTUV_OT_AdjustDecalWidth.bl_options) == {"UNDO", "BLOCKING"}
    assert overlay_registered()
    for name, handler in (
        ("undo_post", _after_history),
        ("redo_post", _after_history),
        ("load_post", _after_load),
        ("depsgraph_update_post", _after_depsgraph),
    ):
        assert handler in getattr(bpy.app.handlers, name), name
    assert line_segments([((0, 0, 0), (1, 0, 0), (1, 1, 0))]) == [(0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 1, 0)]
    assert not bpy.ops.hotspotuv.adjust_decal_width.poll()  # фоновый Blender: нет области 3D View


def _main():
    import cftuv
    from cftuv import envelope_queue_pool

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован установленной копией
        pass
    assert hasattr(bpy.ops.hotspotuv, "build_envelope_decal_mesh")
    _run_the_tool_is_registered_with_undo_and_the_overlay_and_history_handlers_stand()
    _run_blender_events_become_the_events_of_the_automaton()
    _run_five_changes(workers=0)
    # Живой пул: точный пересчёт уходит воркерам из потока планировщика (порог малой партии на время снят).
    original = envelope_queue_pool.COVERAGE_POOL_MIN_BYTES
    envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        _run_five_changes(workers=2)
    finally:
        envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = original
    _run_the_modal_tool_drags_the_preview_and_the_buttons_answer_follows_only_a_confirm()
    _run_the_preview_mesh_follows_the_hand_inside_the_model_and_the_exact_result_replaces_it()
    _run_the_preview_mesh_belongs_only_to_the_mesh_the_exact_path_wrote()
    _run_timer_writes_keep_the_scene_and_the_undo_history_consistent()
    _run_the_tool_belongs_to_the_active_objects_own_decal()
    _run_the_width_tool_rebuilds_a_tightened_dome_ring_for_the_new_width()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_DECAL_WIDTH_LIVE_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
