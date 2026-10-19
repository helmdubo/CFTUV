"""LIVE_DRAG_STREAM_V1: живой точный меш во время перетаскивания ширины («Live exact mesh while dragging», по умолчанию выключено).

Что держат тесты:
1. ПЛАНИРОВЩИК (фальшивки). Потоковый заказ стартует без паузы; новое значение НЕ отменяет полёт; промежуточные значения, замещённые до старта, не считаются
   (`coalesced`); результат полёта применяется как ближайший (`nearest`), и следующим стартует последнее значение; обычный заказ (подтверждение) отменяет полёт и делает
   его результат устаревшим, как прежде; полёт обычного заказа потоковое значение не отменяет. Простой спрашивается один раз после потокового полёта; вытесняемый заказ
   снимается новым потоковым значением до старта и в полёте (`preempted`, не `stale`). Ни один заказ не теряется: сумма исходов равна числу заказов.
2. ХОСТ (подмена Blender). Шаги перетаскивания - потоковые заказы инкрементального такта; такт ближайшей ширины ложится и меш сдвигается к курсору; последнее побеждает;
   срок сверки отложен и назван, сверка - в простое полным lean-тактом `LIVE_DRAG_RECONCILE_IDLE`, рука её вытесняет; подтверждение - COMMIT побитово равный прямому
   прогону и подтверждению без настройки; такт после конца перетаскивания отброшен названно; журнал называет всё в строке панели и консоли.
3. ВЫКЛЮЧЕНО - как прежде: без настройки шаги заказываются прежним заказом (пауза, отмена полёта) либо не заказываются вовсе, журнала нет.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as production_mesh  # noqa: E402
from cftuv import envelope_width_incremental as incremental  # noqa: E402
from cftuv import envelope_width_live as live  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv import envelope_width_stream as stream  # noqa: E402
from cftuv.envelope_alpha_preview import PreviewCancelled  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_production_export import REFRESH_INCREMENTAL, run_production  # noqa: E402
from test_envelope_alpha_preview import Harness  # noqa: E402
from test_envelope_refresh_lean import EDGES, OFFSET, _drag_world  # noqa: E402
from test_envelope_width_incremental import _after_the_button  # noqa: E402


def _outcomes(counters) -> int:
    return counters.applied + counters.coalesced + counters.stale + counters.preempted + counters.invalid + counters.unavailable + counters.failed + counters.cancelled


# ==========================================================================
# 1. Планировщик
# ==========================================================================


def test_a_stream_value_starts_without_the_pause_and_arms_the_timer_at_once():
    h = Harness()
    h.at(0.0).scheduler.request(0.30, stream=True)
    assert h.timers.first_intervals and h.timers.first_intervals[0] < h.scheduler._debounce
    h.at(0.006).step()
    assert [item.alpha for item in h.begun] == [0.30] and h.scheduler.in_flight


def test_latest_wins_the_flight_is_not_cancelled_the_superseded_values_are_dropped_and_the_nearest_result_is_applied():
    h = Harness()
    h.at(0.0).scheduler.request(0.30, stream=True)
    h.at(0.01).step()
    first = h.jobs[0]
    for index, alpha in enumerate((0.31, 0.32, 0.33)):
        h.at(0.02 + 0.016 * index).scheduler.request(alpha, stream=True)
        h.step()
    assert not first.cancelled and len(h.begun) == 1  # один полёт, и рука его не отменяет

    first.finished = True
    h.at(0.1).step()  # полёт применён как ближайший, следующим стартует ПОСЛЕДНЕЕ значение
    assert h.applied == [(0.30, ("scene", 0.30))] and [item.alpha for item in h.begun] == [0.30, 0.33]
    h.jobs[1].finished = True
    h.at(0.15).step()

    assert [item[0] for item in h.applied] == [0.30, 0.33]
    counters = h.scheduler.counters
    assert (counters.requested, counters.coalesced, counters.applied, counters.nearest, counters.stale) == (4, 2, 2, 1, 0)
    assert _outcomes(counters) == counters.requested


def test_a_plain_order_after_stream_values_cancels_the_flight_and_makes_its_result_stale():
    """Подтверждение - обычный заказ: оно отменяет потоковый полёт, и его результат не ложится (как прежде)."""

    h = Harness()
    h.at(0.0).scheduler.request(0.30, stream=True)
    h.at(0.01).step()
    h.at(0.02).scheduler.request(0.40)
    assert h.jobs[0].cancelled
    h.jobs[0].finished = True
    h.at(0.03).step()
    assert h.applied == [] and h.scheduler.counters.stale == 1
    h.at(0.3).step()
    h.jobs[1].finished = True
    h.at(0.31).step()
    assert h.applied == [(0.40, ("scene", 0.40))] and h.scheduler.counters.nearest == 0


def test_a_stream_value_never_cancels_a_plain_flight_and_the_plain_result_still_lands():
    h = Harness()
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.at(0.21).scheduler.request(0.31, stream=True)
    assert not h.jobs[0].cancelled
    h.jobs[0].finished = True
    h.at(0.22).step()
    assert h.applied[0] == (0.30, ("scene", 0.30)) and h.scheduler.counters.nearest == 1


def test_the_idle_is_asked_once_after_a_stream_flight_and_its_order_is_preempted_by_the_hand():
    asked: list = []

    def idle():
        asked.append(True)
        return (0.33, "reconcile")

    h = Harness(idle=idle, idle_delay=0.1)
    h.at(0.0).scheduler.request(0.33, stream=True)
    h.at(0.01).step()
    h.jobs[0].finished = True
    h.at(0.05).step()
    assert asked == [True]
    pending = h.scheduler._pending
    assert pending.preemptible and pending.payload == "reconcile" and h.scheduler._deadline == pytest.approx(0.15)

    h.at(0.06).scheduler.request(0.34, stream=True)  # рука снова движется: ожидающая сверка снята
    assert h.scheduler.counters.preempted == 1 and not h.scheduler._pending.preemptible
    h.at(0.07).step()
    h.jobs[1].finished = True
    h.at(0.08).step()  # такт применён, простой спрошен снова: сверка ждёт
    assert asked == [True, True] and h.scheduler._pending.preemptible
    h.at(0.2).step()  # сверка стартовала
    assert h.begun[-1].preemptible
    h.at(0.21).scheduler.request(0.35, stream=True)  # рука вытесняет сверку в полёте
    assert h.jobs[2].cancelled
    h.jobs[2].error = PreviewCancelled("stopped between domains")
    h.jobs[2].finished = True
    h.at(0.22).step()
    counters = h.scheduler.counters
    assert counters.preempted == 2 and counters.stale == 0
    h.jobs[3].finished = True
    h.at(0.25).step()
    assert _outcomes(h.scheduler.counters) + (1 if h.scheduler.busy else 0) == h.scheduler.counters.requested


def test_without_stream_orders_the_idle_is_never_asked():
    asked: list = []
    h = Harness(idle=lambda: asked.append(True))
    h.at(0.0).scheduler.request(0.30)
    h.at(0.2).step()
    h.jobs[0].finished = True
    h.at(0.3).step()
    assert asked == [] and h.step() is None


# ==========================================================================
# 2. Хост
# ==========================================================================


def _live_world(monkeypatch, *, enabled=True):
    """Мир после кнопки с перетаскиванием и включённой настройкой; такт ложится в сессию как у `lean_exact_now` (образец, модель, запись сверки), сдвиг к курсору записан."""

    monkeypatch.setattr(incremental, "INCREMENTAL_MIN_DOMAINS", 0)  # ряд из пяти доменов: правило малого выделения выключено, иначе каждый такт полный
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = _after_the_button(world)
    world.mesh_settings.drag_exact_ticks = False
    world.mesh_settings.drag_live_mesh = enabled
    live.begin_live_drag(controller, world.mesh_settings)
    world.calls, world.reframes = calls, []

    def lay(session, run):
        calls.append(run)
        preview.note_exact_display(session, run)
        return ""

    monkeypatch.setattr(live, "lean_exact_now", lay)
    monkeypatch.setattr(live, "width_event_preview", lambda session, width, offset: world.reframes.append(width))
    return world, controller


def _finish(scheduler):
    scheduler._job.join()
    scheduler.step()


def test_the_steps_stream_incremental_ticks_latest_wins_and_the_nearest_tick_is_moved_to_the_pointer(monkeypatch):
    world, controller = _live_world(monkeypatch)
    assert live.request_drag_tick(world.context, controller, 0.2505)
    scheduler = controller.width_live
    assert scheduler._pending.stream and scheduler._pending.payload.stream and scheduler._pending.payload.grade == REFRESH_INCREMENTAL
    scheduler.step()  # без паузы
    flight = scheduler._job
    assert flight is not None
    for width in (0.251, 0.2515, 0.252):
        assert live.request_drag_tick(world.context, controller, width)
    assert not flight._cancel.is_set()

    _finish(scheduler)  # 0.2505 лёг как ближайший, меш сдвинут к 0.252, стартовал 0.252
    assert [item.sample.alpha for item in world.calls] == [0.2505] and world.reframes == [0.252]
    assert scheduler._job_request.alpha == 0.252
    _finish(scheduler)

    assert [item.sample.alpha for item in world.calls] == [0.2505, 0.252] and world.reframes == [0.252]
    assert all(item.plan.incremental for item in world.calls)
    counters = scheduler.counters
    assert (counters.requested, counters.coalesced, counters.applied, counters.nearest) == (5, 2, 2, 1)  # пятый - сверка, заказанная простоем
    assert scheduler._pending.preemptible and scheduler._pending.payload.reconcile
    log = controller.width_live_drag
    assert (log.steps, log.exact_at_cursor, log.exact_nearest, log.reframes) == (4, 1, 1, 1)
    steps = [event for event in log.events if event[0] == stream.EVENT_STEP]
    exact = [event for event in log.events if event[0] == stream.EVENT_EXACT]
    assert [event[2] for event in steps] == [0.2505, 0.251, 0.2515, 0.252] and [event[2] for event in exact] == [0.2505, 0.252]


def test_the_verification_is_deferred_while_the_hand_moves_and_reconciled_by_name_when_the_pool_is_idle(monkeypatch):
    monkeypatch.setattr(incremental, "INCREMENTAL_VERIFY_AFTER_TICKS", 1)
    world, controller = _live_world(monkeypatch)
    for width in (0.2505, 0.251):
        assert live.request_drag_tick(world.context, controller, width)
        controller.width_live.step()
        _finish(controller.width_live)
    scheduler = controller.width_live
    assert world.calls[1].plan.incremental and world.calls[1].plan.deferred == incremental.REASON_PERIOD_TICKS  # срок назван, такт остался инкрементальным
    pending = scheduler._pending
    assert pending is not None and pending.preemptible and pending.payload.reconcile and pending.alpha == 0.251

    live.settle_width_live(controller)

    reconcile = world.calls[-1]
    assert not reconcile.plan.incremental and reconcile.plan.reason == stream.RECONCILE_IDLE and reconcile.sample.alpha == 0.251
    assert controller.width_incremental == incremental.IncrementalStateV1(0, 0.251)
    log = controller.width_live_drag
    # Простой после первого такта заказал сверку на 0.2505, второй шаг руки её вытеснил до старта; легла сверка на 0.251.
    assert (log.reconcile_requested, log.reconcile_applied, scheduler.counters.preempted) == (2, 1, 1) and log.deferred == {incremental.REASON_PERIOD_TICKS: 1}
    assert log.full_ticks == {stream.RECONCILE_IDLE: 1} and log.structure_changes == 0 and log.layout_changes == 0
    assert not scheduler.busy  # сверено на ширине курсора: простой больше ничего не заказывает


def test_the_hand_preempts_the_idle_reconcile(monkeypatch):
    world, controller = _live_world(monkeypatch)
    assert live.request_drag_tick(world.context, controller, 0.2505)
    scheduler = controller.width_live
    scheduler._idle_delay = 0.0
    scheduler.step()
    _finish(scheduler)  # такт лёг, простой заказал сверку
    assert scheduler._pending.preemptible
    scheduler.step()  # сверка в полёте
    reconcile = scheduler._job
    assert scheduler._job_request.preemptible
    assert live.request_drag_tick(world.context, controller, 0.251)
    assert reconcile._cancel.is_set()
    _finish(scheduler)
    counters = scheduler.counters
    assert counters.preempted + counters.nearest == 1 and counters.stale == 0  # вытеснена (либо успела и легла как ближайшая) - названо


def test_the_confirm_is_the_commit_of_the_direct_run_and_of_the_confirm_without_the_switch(monkeypatch):
    digests = []
    for enabled in (True, False):
        world, controller = _live_world(monkeypatch, enabled=enabled)
        if enabled:
            for width in (0.2505, 0.251, 0.2515):
                assert live.request_drag_tick(world.context, controller, width)
                controller.width_live.step()
            assert controller.width_live.in_flight  # такт в пути в момент подтверждения
        preview.finish_drag(controller, restore=False)
        live.end_live_drag(controller)
        world.drag(0.3)
        live.settle_width_live(controller)
        assert len(world.written) == 1 and world.written[0][3] == 0.3
        assert not any(item.lean for item in world.written[0][0])
        digests.append(production_mesh.build_mesh_arrays(world.written[0][0], OFFSET).digest)
    reference = run_production(
        EnvelopeDebugSessionController(), world.bundle, EDGES, 0.3, source_object_key="object", source_data_key="mesh", density="1", domain_pool=None
    )
    assert digests[0] == digests[1] == production_mesh.build_mesh_arrays(reference.results, OFFSET).digest


def test_a_stream_tick_that_lands_after_the_cancel_is_dropped_by_name(monkeypatch):
    world, controller = _live_world(monkeypatch)
    assert live.request_drag_tick(world.context, controller, 0.2505)
    scheduler = controller.width_live
    scheduler.step()
    controller.width_drag_active = False  # отмена пришла раньше, чем досчиталось
    _finish(scheduler)
    assert world.calls == [] and scheduler.counters.invalid == 1 and live.LEAN_DRAG_ENDED in scheduler.status_text
    assert not scheduler.busy  # простой после конца перетаскивания ничего не заказывает


def test_the_log_names_everything_in_the_panel_and_the_console(monkeypatch, capsys):
    world, controller = _live_world(monkeypatch)
    for width in (0.2505, 0.251):
        assert live.request_drag_tick(world.context, controller, width)
    live.settle_width_live(controller)
    line = live.end_live_drag(controller)
    assert line.startswith("Live drag LIVE_DRAG_STREAM_V1: 2 steps") and "superseded" in line and "reconciled" in line and "deferred" in line
    assert line in capsys.readouterr().out
    assert any(item.startswith("Live drag LIVE_DRAG_STREAM_V1") for item in live.status_lines(controller))
    data = stream.report(controller.width_live_drag, controller.width_live.counters)
    assert data["steps"] == 2 and data["scheduler"]["requested"] >= 2


def test_a_step_without_a_displayed_sample_is_named(monkeypatch):
    world, controller = _live_world(monkeypatch)
    controller.width_displayed = None
    assert live.request_drag_tick(world.context, controller, 0.2505) is False
    assert controller.width_live_drag.refused == {stream.STEP_NO_SAMPLE: 1}


# ==========================================================================
# 3. Выключено - как прежде
# ==========================================================================


def test_switched_off_the_steps_are_the_old_orders_or_none_and_there_is_no_log(monkeypatch):
    world, controller = _live_world(monkeypatch, enabled=False)
    assert controller.width_live_drag is None
    assert live.request_drag_tick(world.context, controller, 0.2505) is False  # обе настройки выключены: ничего
    world.mesh_settings.drag_exact_ticks = True
    assert live.request_drag_tick(world.context, controller, 0.2505)
    pending = controller.width_live._pending
    assert not pending.stream and not pending.payload.stream and controller.width_live_drag is None
    assert live.end_live_drag(controller) == ""


def test_the_stream_policy_of_the_plan_defers_or_forces_by_name_and_none_is_the_old_decision(monkeypatch):
    world, controller = _live_world(monkeypatch)
    displayed, model = controller.width_displayed, controller.width_model
    due = incremental.IncrementalStateV1(incremental.INCREMENTAL_VERIFY_AFTER_TICKS, 0.25)
    old = preview.plan_tick(REFRESH_INCREMENTAL, displayed, model, due, 0.2505)
    assert not old.incremental and old.reason == incremental.REASON_PERIOD_TICKS
    deferred = preview.plan_tick(REFRESH_INCREMENTAL, displayed, model, due, 0.2505, incremental.StreamPolicyV1())
    assert deferred.incremental and deferred.deferred == incremental.REASON_PERIOD_TICKS
    forced = preview.plan_tick(REFRESH_INCREMENTAL, displayed, model, incremental.IncrementalStateV1(0, 0.25), 0.2505, incremental.StreamPolicyV1(force=stream.RECONCILE_IDLE))
    assert not forced.incremental and forced.reason == stream.RECONCILE_IDLE
    no_base = preview.plan_tick(REFRESH_INCREMENTAL, displayed, model, None, 0.2505, incremental.StreamPolicyV1())
    assert not no_base.incremental and no_base.reason == incremental.REASON_NO_BASE  # без записи сверки отложить нечего: такт полный
