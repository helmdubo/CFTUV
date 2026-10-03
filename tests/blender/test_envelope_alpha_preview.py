"""Blender 4.5 background smoke: фоновое превью alpha (ASYNC-ALPHA).

В фоновом Blender таймеры `bpy.app.timers` не срабатывают (проверено: функция зарегистрирована,
но главного цикла нет), поэтому смок ШАГАЕТ таймер сам: вызывает ту самую функцию, которую
Blender получил в `register` (`scheduler._callback`), и спит ровно столько, сколько она
вернула. Это тот же код, что исполнит интерфейс; нет только вызова ядром Blender.

Утверждения стоят на числах и содержимом, а не на виде картинки:

1. ПЯТЬ БЫСТРЫХ ИЗМЕНЕНИЙ через путь `update` ползунка: каждое — доли миллисекунды главного
   потока, ничего не посчитано и не применено до паузы (прежний результат на месте), таймер
   взведён;
2. шаги таймера считают ОДНО значение — последнее: `coalesced == 4`, `applied == 1`;
3. ОТВЕТ ТОТ ЖЕ, ЧТО У КНОПКИ: после нажатия кнопки отладки на том же alpha слои очереди,
   штрихи GP и запись sidecar побитово равны результату ползунка (в родителе и в воркерах);
4. ОТМЕНА В ПОЛЁТЕ: значение, пришедшее во время счёта, отменяет его; результат устаревшего
   значения не пишется в сцену ни на одном шаге, а считается в `stale`;
5. КНОПКИ ОСТАНАВЛИВАЮТ ПРЕВЬЮ до своей работы: после нажатия «Build Decal Mesh» заказ снят
   (`cancelled`), а опоздавший результат не пишется;
6. ОТМЕНА (Undo): применение результата таймером не создаёт и не освобождает датаблоков, а
   `ed.undo`/`ed.redo` после применённых результатов оставляют сцену целой, и ползунок после
   возврата снова работает.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_ALPHA_PREVIEW_BLENDER_SMOKE_OK
"""

from __future__ import annotations

from fractions import Fraction
import hashlib
import json
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
from test_envelope_debug_bridge import (  # noqa: E402
    _build_two_patch_seam,
    _reset_scene,
    _sidecar_payload,
)
from test_envelope_production_mesh import (  # noqa: E402
    _assert_scene_links_only_live_objects,
    _enable_background_undo,
    _redo_and_check,
    _undo_from_the_state_the_button_left,
    _walk_every_datablock,
)


SOURCE = "EnvelopeTwoPatch"
GP_OBJECT = "CFTUV_DEBUG_Envelope_" + SOURCE
SECONDS_KEYS = ("prepare_seconds", "coverage_seconds", "contour_seconds", "timings")
DRAG = (0.30, 0.31, 0.32, 0.33, 0.34)
#: Заказ обязан вернуться быстро: счёт одного значения здесь стоит десятки миллисекунд и больше.
ORDER_BUDGET_SECONDS = 0.05


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _scheduler():
    scheduler = _controller().alpha_preview
    assert scheduler is not None
    return scheduler


def _strip_seconds(value):
    if isinstance(value, dict):
        return {k: _strip_seconds(v) for k, v in value.items() if k not in SECONDS_KEYS}
    if isinstance(value, list):
        return [_strip_seconds(item) for item in value]
    return value


def _gp_digest(gp_obj):
    """Штрихи слоёв очереди (позиция, радиус, материал, замкнутость): GPENCIL и GREASEPENCIL v3."""

    rows = []
    for layer in gp_obj.data.layers:
        name = getattr(layer, "name", getattr(layer, "info", ""))
        if not name.startswith("ENV_9"):
            continue
        for frame in layer.frames:
            drawing = getattr(frame, "drawing", None)
            strokes = drawing.strokes if drawing is not None else frame.strokes
            for stroke in strokes:
                rows.append(
                    (
                        name,
                        int(getattr(stroke, "material_index", 0)),
                        bool(getattr(stroke, "cyclic", getattr(stroke, "use_cyclic", False))),
                        [
                            (
                                tuple(getattr(point, "position", None) or point.co),
                                round(float(point.radius), 9),
                            )
                            for point in stroke.points
                        ],
                    )
                )
    text = json.dumps(rows, sort_keys=True, separators=(",", ":"))
    return len(rows), hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def _answer():
    """Всё, что ползунок и кнопка обязаны выдать одинаково (без секунд)."""

    payload = _sidecar_payload(bpy.data.objects[SOURCE])
    queue = _strip_seconds(payload["queue"])
    strokes = [item for item in payload["strokes"] if item.get("stage") == "QUEUE"]
    gp_obj = bpy.data.objects[GP_OBJECT]
    return {
        "queue": queue,
        "queue_strokes": strokes,
        "gp": _gp_digest(gp_obj),
        "requested_alpha": gp_obj["requested_alpha"],
    }


def _fresh_scene(*, workers=0):
    _reset_scene()
    controller = _controller()
    if controller is not None:
        controller.clear()
        old = controller.alpha_preview
        if old is not None and bpy.app.timers.is_registered(old._callback):
            bpy.app.timers.unregister(old._callback)
        controller.alpha_preview = None  # счётчики каждого сценария свои
    source = _build_two_patch_seam()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = workers
    assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
    assert _controller().queue_session is not None
    return source


def _drag(values, *, pause=0.02):
    """Значения через путь `update`; возвращает секунды главного потока на каждое изменение."""

    settings = _settings()
    spent = []
    for value in values:
        started = time.perf_counter()
        settings.envelope_debug_alpha = value
        spent.append(time.perf_counter() - started)
        time.sleep(pause)
    return spent


def _pump(*, stop_when=None, timeout=60.0):
    """Таймер без главного цикла: вызывает зарегистрированную функцию и спит, сколько она просит.

    Возвращает секунды каждого шага (главный поток занят ровно столько). `stop_when()` —
    досрочная остановка (например, «счёт в полёте»).
    """

    scheduler = _scheduler()
    callback = scheduler._callback
    steps = []
    end = time.perf_counter() + timeout
    while True:
        if stop_when is not None and stop_when():
            return steps
        started = time.perf_counter()
        delay = callback()
        steps.append(time.perf_counter() - started)
        if delay is None:
            # Blender снимает функцию, вернувшую `None`; здесь это делаем мы.
            if bpy.app.timers.is_registered(callback):
                bpy.app.timers.unregister(callback)
            return steps
        assert time.perf_counter() < end, "alpha preview did not settle"
        time.sleep(delay)


def _datablock_counts():
    return {
        name: len(getattr(bpy.data, name))
        for name in ("objects", "meshes", "materials", "texts", "grease_pencils", "collections")
        if hasattr(bpy.data, name)
    }


def _run_a_fast_drag_is_ordered_coalesced_and_applied_once_with_the_buttons_answer(workers):
    source = _fresh_scene(workers=workers)
    settings = _settings()
    before_alpha = _answer()["requested_alpha"]

    spent = _drag(DRAG)
    scheduler = _scheduler()
    counters = scheduler.counters
    # 1. Заказ: быстро, ничего не посчитано и не применено, прежний результат на месте.
    assert max(spent) < ORDER_BUDGET_SECONDS, spent
    assert (counters.requested, counters.started, counters.applied) == (5, 0, 0), counters
    assert bpy.app.timers.is_registered(scheduler._callback)
    assert _answer()["requested_alpha"] == before_alpha
    assert scheduler.status_text == "computing alpha=0.34... | superseded 4", scheduler.status_text

    # 2. Шаги таймера: одно значение, последнее.
    steps = _pump()
    counters = scheduler.counters
    assert (counters.started, counters.applied, counters.coalesced, counters.stale) == (1, 1, 4, 0), counters
    assert scheduler.status_text.startswith("ready alpha=0.34"), scheduler.status_text
    assert "alpha redraw" in settings.envelope_debug_queue_timing
    if workers:
        assert f"on {workers} workers" in settings.envelope_debug_queue_timing, settings.envelope_debug_queue_timing
    slider = _answer()
    assert slider["requested_alpha"] != before_alpha
    print(
        f"DRAG workers={workers}: change max {max(spent) * 1000:.3f} ms, "
        f"steps {len(steps)} max {max(steps) * 1000:.1f} ms, "
        f"latency {scheduler.last_applied.latency_seconds * 1000:.0f} ms "
        f"(compute {scheduler.last_applied.compute_seconds * 1000:.0f}, "
        f"apply {scheduler.last_applied.apply_seconds * 1000:.0f})"
    )

    # 3. Кнопка на том же alpha даёт ту же запись.
    assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
    button = _answer()
    for key in ("queue", "queue_strokes", "gp"):
        assert slider[key] == button[key], key
    assert slider["gp"][0] > 0
    # Свойство GP `requested_alpha` слайдер пишет записью домена очереди (дробь), кнопка — записью
    # запроса (десятичная строка): ЧИСЛО одно и то же, запись разная и была разной до превью. Остальные
    # поля sidecar, кроме слоёв очереди (`decal_request_ids`, дайджесты), ползунок не переписывает никогда.
    assert Fraction(slider["requested_alpha"]) == Fraction(button["requested_alpha"])
    return source


def _slow_coverage(delay):
    """Покрытие, которому нужно `delay` секунд и которое слышит отмену (как слышит пул)."""

    import cftuv.envelope_queue_export as export

    real = export.recompute_queue_coverage

    def slow(entries, alpha_text, **kwargs):
        cancel = kwargs.get("cancel")
        end = time.perf_counter() + delay
        while time.perf_counter() < end:
            if cancel is not None and cancel.is_set():
                raise export.CoverageCancelled("test: cancelled while computing")
            time.sleep(0.005)
        return real(entries, alpha_text, **kwargs)

    export.recompute_queue_coverage = slow
    return real


def _run_a_value_during_the_flight_cancels_it_and_the_stale_result_never_lands():
    import cftuv.envelope_queue_export as export

    _fresh_scene()
    settings = _settings()
    scheduler = None
    real = _slow_coverage(0.4)
    try:
        settings.envelope_debug_alpha = 0.31
        scheduler = _scheduler()
        seen = {_answer()["requested_alpha"]}
        deadline = time.perf_counter() + 30.0
        while not scheduler.in_flight:
            assert time.perf_counter() < deadline
            scheduler._callback()
            time.sleep(0.01)
        # Счёт A в полёте: приходит B.
        settings.envelope_debug_alpha = 0.37
        steps_seen = 0
        while True:
            delay = scheduler._callback()
            seen.add(_answer()["requested_alpha"])
            steps_seen += 1
            if delay is None:
                break
            time.sleep(delay)
    finally:
        export.recompute_queue_coverage = real
    counters = scheduler.counters
    assert (counters.stale, counters.applied, counters.failed) == (1, 1, 0), counters
    assert len(seen) == 2, seen  # прежнее значение и B; A не появлялся ни на одном шаге
    final = _answer()["requested_alpha"]
    assert final == _answer()["queue"]["domains"][0]["alpha"] and final in seen
    assert float(Fraction(final)) == float(_settings().envelope_debug_alpha)
    assert scheduler.status_text.startswith("ready alpha=0.37"), scheduler.status_text
    assert scheduler.status_text.endswith("| superseded 1"), scheduler.status_text
    if bpy.app.timers.is_registered(scheduler._callback):
        bpy.app.timers.unregister(scheduler._callback)
    print("SUPERSEDE flight cancelled:", counters)


def _run_the_buttons_stop_the_preview_before_their_work():
    source = _fresh_scene()
    settings = _settings()
    settings.envelope_debug_alpha = 0.31
    scheduler = _scheduler()
    assert scheduler.busy and scheduler.counters.started == 0

    # «Build Decal Mesh» из EDIT-режима: заказ снят до работы кнопки.
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
    assert not scheduler.busy
    assert scheduler.counters.cancelled == 1 and scheduler.counters.applied == 0
    assert scheduler.status_text.startswith("cancelled: Build Decal Mesh"), scheduler.status_text
    assert scheduler.step() is None  # таймеру нечего делать: опоздавшего результата нет
    # Кнопка отладки: то же.
    settings.envelope_debug_alpha = 0.36
    assert scheduler.busy
    assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
    assert scheduler.counters.cancelled == 2 and not scheduler.busy
    assert scheduler.status_text.startswith("cancelled: Envelope debug build")
    assert float(Fraction(_answer()["queue"]["domains"][0]["alpha"])) == float(
        settings.envelope_debug_alpha
    )
    if bpy.app.timers.is_registered(scheduler._callback):
        bpy.app.timers.unregister(scheduler._callback)
    print("BUTTONS stopped the preview:", scheduler.counters)
    return source


def _run_timer_applied_results_keep_the_scene_and_undo_consistent():
    source = _fresh_scene()
    seam = _enable_background_undo(source)
    assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug(True) == {"FINISHED"}
    settings = _settings()

    blocks = _datablock_counts()
    for value in (0.31, 0.38):  # два применённых результата, как два отпускания ползунка
        settings.envelope_debug_alpha = value
        _pump()
    assert _scheduler().counters.applied == 2
    # Применение переписывает слои и sidecar в существующих датаблоках, не создавая и не освобождая их.
    assert _datablock_counts() == blocks, (blocks, _datablock_counts())

    # Как Ctrl+Z владельца после движения ползунка: выход из EDIT без шага, отмена, возврат.
    _undo_from_the_state_the_button_left()
    _assert_scene_links_only_live_objects()
    _walk_every_datablock()
    _redo_and_check()
    assert GP_OBJECT in bpy.data.objects

    # После возврата сцена целая, ползунок снова заказывает и применяет. Настройки берутся заново:
    # отмена по memfile могла пересоздать сцену, и прежний указатель RNA недействителен.
    bpy.context.view_layer.objects.active = bpy.data.objects[SOURCE]
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    settings = _settings()
    settings.envelope_debug_alpha = 0.33
    _pump()
    counters = _scheduler().counters
    assert counters.failed == 0, counters
    assert counters.applied + counters.invalid >= 3, counters
    _walk_every_datablock()
    print("UNDO timer-applied writes: datablocks unchanged, undo/redo clean;", counters)


def _main():
    import cftuv
    from cftuv import envelope_queue_pool

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован установленной копией
        pass
    assert hasattr(bpy.ops.hotspotuv, "build_exact_reference_envelope_debug")

    _run_a_fast_drag_is_ordered_coalesced_and_applied_once_with_the_buttons_answer(0)
    # Живой пул: покрытие уходит воркерам и из потока счёта (порог малой партии на время проверки снят).
    original = envelope_queue_pool.COVERAGE_POOL_MIN_BYTES
    envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        _run_a_fast_drag_is_ordered_coalesced_and_applied_once_with_the_buttons_answer(2)
    finally:
        envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = original
    _run_a_value_during_the_flight_cancels_it_and_the_stale_result_never_lands()
    _run_the_buttons_stop_the_preview_before_their_work()
    _run_timer_applied_results_keep_the_scene_and_undo_consistent()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_ALPHA_PREVIEW_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
