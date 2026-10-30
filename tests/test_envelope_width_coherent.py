"""LIVE_DRAG_COHERENT_V1: пока рука движется, за ней идёт каждый домен; точный меш ложится одним кадром на остановке.

Что держат тесты (без Blender; в Blender то же меряет `tools/blender_gui_drag.py --coherent on`):
1. КАДР. Связный кадр двигает и домены за окном модели (экстраполяция названа и посчитана), домены без наклонов идут конечной разностью двух точных образцов (названа),
   а без неё стоят под именем; на ширине базы кадр - сама база. Узлы Blender (`coherent_rig_inputs`) считают то же во float32.
2. ПОТОК. Пока рука движется, точный такт не ложится; на остановке ложится только такт ширины покоя; сверки простоя нет; на подтверждении такт другой ширины не ложится.
3. ВЫКЛЮЧЕНО - как прежде: без настройки кадр модели прежний, поток прежний.
"""

from __future__ import annotations

import sys
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_width_gpu_drag as gpu  # noqa: E402
from cftuv import envelope_width_live as live  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv import envelope_width_preview_model as pm  # noqa: E402
from cftuv import envelope_width_stream as stream  # noqa: E402
from test_envelope_width_preview_model import _fold_sample, _interval, _owned, _writes, row  # noqa: E402,F401  (`row` - фикстура настоящих доменов)


def _ulps(a, b):
    a, b = np.asarray(a, dtype=np.float32), np.asarray(b, dtype=np.float32)
    return float((np.abs(a - b) / np.spacing(np.maximum(np.abs(a), np.abs(b)))).max())


# ==========================================================================
# 1. Кадр
# ==========================================================================


def test_the_coherent_frame_moves_the_domain_the_model_holds_and_names_the_extrapolation():
    base, _ = _fold_sample(0.25, wall_interval=_interval(0.25, low=0.24, high=0.27))  # стену обычный кадр держит за полосой 0.24-0.27
    model = pm.build_model(base)
    coherent = pm.coherent_slopes(model)
    assert coherent.standing == () and coherent.estimated == 0 and np.all(coherent.kind == 0)
    extrapolated = 0
    for width in (0.2, 0.26, 0.4):
        exact, _ = _fold_sample(width)
        frame = pm.coherent_frame(model, coherent, width)
        assert frame.outcome == pm.LIVE_DRAG_COHERENT_V1 and frame.coherent and frame.held_domains == 0 and frame.live_domains == len(model.rows)
        assert _close(frame, exact)  # складка линейна: экстраполяция точна
        regular = pm.evaluate(model, width)
        extrapolated += frame.extrapolated
        assert frame.extrapolated == regular.held_domains  # что обычный кадр держит, связный ведёт и называет
        if regular.held_domains:
            assert frame.extrapolated_reasons and sum(count for _name, count in frame.extrapolated_reasons) == frame.extrapolated
    assert extrapolated > 0  # хотя бы одну ширину обычный кадр держит
    at_base = pm.coherent_frame(model, coherent, base.alpha)
    assert np.array_equal(at_base.positions, pm.base_frame(base).positions) and np.array_equal(at_base.uvs, pm.base_frame(base).uvs)


def _close(frame, exact):
    return np.allclose(frame.positions, np.asarray(exact.positions).reshape(-1), atol=1e-6) and np.allclose(frame.uvs, np.asarray(exact.uvs).reshape(-1), atol=1e-6)


def _wall_without_slopes(alpha, **kwargs):
    """Складка, у стены которой нет наклонов сертификата (как у домена без чисел): модель её не двигает."""

    sample, _ = _fold_sample(alpha, **kwargs)
    domains = tuple(replace(item, slopes=None, uv_slopes=None) if item.patch_id == 1 else item for item in sample.domains)
    return replace(sample, domains=domains)


def test_a_domain_without_slopes_follows_the_finite_difference_of_two_exact_samples_and_without_one_stands_named():
    first = _wall_without_slopes(0.25)
    model = pm.build_model(first)
    alone = pm.coherent_slopes(model)
    assert alone.estimated == 0 and len(alone.standing) == 1 and alone.standing[0][0] == 1 and alone.standing[0][1]  # одного образца мало: стоит под именем
    frame = pm.coherent_frame(model, alone, 0.3)
    assert frame.held_domains == 1 and frame.held == alone.standing

    _found, kept = pm.finite_difference(None, first)
    second = _wall_without_slopes(0.3)
    found, _kept = pm.finite_difference(kept, second)
    assert [key[0] for key in found] == [1]  # разность - только у домена без наклонов
    model = pm.build_model(second)
    estimated = pm.coherent_slopes(model, found)
    assert estimated.estimated == 1 and estimated.standing == ()
    exact, _ = _fold_sample(0.35)
    frame = pm.coherent_frame(model, estimated, 0.35)
    assert frame.estimated == 1 and frame.held_domains == 0
    assert _close(frame, exact)  # складка линейна: разность точна


def test_the_finite_difference_needs_the_same_structure_and_another_width():
    first = _wall_without_slopes(0.25)
    _found, kept = pm.finite_difference(None, first)
    same_width, _ = pm.finite_difference(kept, first)
    assert same_width == {}
    other = _wall_without_slopes(0.3, wall_token_suffix="-other")
    found, _ = pm.finite_difference(kept, other)
    assert found == {}  # стена сменила структуру: её разности нет


def test_the_nodes_compute_the_coherent_frame_up_to_float32_rounding(row):
    base, _, _ = row(0.25)
    model = pm.build_model(base)
    coherent = pm.coherent_slopes(model)
    arrays = gpu.rig_arrays(**pm.coherent_rig_inputs(model, coherent))
    p0, uv0 = pm.base_frame(base).positions, pm.base_frame(base).uvs
    for width in (0.25 * 0.5, 0.25 * 0.99, 0.25 * 1.02, 0.25 * 1.7):
        frame = pm.coherent_frame(model, coherent, width)
        positions, uvs = gpu.emulate(p0, uv0, arrays, width, base.alpha)
        assert max(_ulps(positions, frame.positions), _ulps(uvs, frame.uvs)) <= 4.0
        summary = pm.coherent_summary(model, coherent, width)
        assert (summary.live_domains, summary.held, summary.extrapolated, summary.estimated) == (frame.live_domains, frame.held, frame.extrapolated, frame.estimated)

# ==========================================================================
# 2. Путь кадра и поток
# ==========================================================================


def test_the_frame_path_is_coherent_only_while_the_coherent_drag_is_on(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    controller.width_drag_active = True
    regular = preview.preview_mesh_now(controller, 0.4)
    assert regular is not None and preview.log_of(controller).coherent_frames == 0
    controller.width_coherent_drag = True
    state = preview.preview_mesh_now(controller, 0.41)
    log = preview.log_of(controller)
    assert state is not None and log.coherent_frames == 1 and state.held == 0
    assert state.extrapolated == pm.evaluate(world.model, 0.41).held_domains
    assert _writes(world) > 0
    controller.width_coherent_drag = False
    preview.preview_mesh_now(controller, 0.42)
    assert preview.log_of(controller).coherent_frames == 1


def _request(sequence, alpha):
    return SimpleNamespace(sequence=sequence, alpha=alpha, stream=True)


def test_while_the_hand_moves_no_exact_tick_lands_and_at_rest_only_the_rest_width_lands():
    log = stream.LiveDragLogV1(cursor=0.3, coherent=True, last_step_at=10.0)
    assert stream.settle_hold(log, _request(1, 0.29), 10.05, drag_active=True)  # рука движется
    assert stream.settle_hold(log, _request(1, 0.29), 10.5, drag_active=True)  # рука стоит на другой ширине
    assert stream.settle_hold(log, _request(1, 0.29), 10.6, drag_active=True)
    assert log.outcomes == {stream.COHERENT_WAITS_FOR_REST: 1}  # названо раз на такт
    assert not stream.settle_hold(log, _request(2, 0.3), 10.6, drag_active=True)  # такт ширины покоя ложится
    assert not stream.settle_hold(log, _request(3, 0.29), 10.6, drag_active=False)  # перетаскивание кончилось - прежний путь


def test_the_coherent_drag_orders_no_idle_verification_and_names_it_once_per_pause():
    log = stream.LiveDragLogV1(cursor=0.3, coherent=True, target=object())
    displayed = SimpleNamespace(domains=(SimpleNamespace(patch_id=1),))
    assert stream.idle_answer(log, displayed, drag_active=True) is None
    assert stream.idle_answer(log, displayed, drag_active=True) is None
    assert log.outcomes == {stream.COHERENT_NO_IDLE_VERIFY: 1} and log.idle_ticks == 0
    plain = stream.LiveDragLogV1(cursor=0.3, target=object())
    assert stream.idle_answer(plain, displayed, drag_active=True) == 0.3  # выключено - как прежде


def test_without_the_setting_the_stream_is_unchanged():
    log = stream.LiveDragLogV1(cursor=0.3, last_step_at=10.0)
    assert not stream.settle_hold(log, _request(1, 0.29), 10.05, drag_active=True)  # ни settle, ни coherent: такт ложится сразу
    log.settle = True
    assert stream.settle_hold(log, _request(1, 0.29), 10.05, drag_active=True)
    assert not stream.settle_hold(log, _request(1, 0.29), 10.5, drag_active=True)  # settle без coherent: на остановке ложится любой такт


def test_the_coherent_switch_begins_with_the_tool_and_ends_with_it():
    controller = SimpleNamespace()
    assert live.begin_coherent_drag(controller, SimpleNamespace(drag_coherent=True, drag_live_mesh=True)) and controller.width_coherent_drag
    assert live.begin_coherent_drag(controller, SimpleNamespace(drag_live_mesh=True)) and controller.width_coherent_drag  # по умолчанию включена
    assert not live.begin_coherent_drag(controller, SimpleNamespace(drag_coherent=False, drag_live_mesh=True)) and not controller.width_coherent_drag
    assert not live.begin_coherent_drag(controller, SimpleNamespace(drag_coherent=True, drag_live_mesh=False)) and not controller.width_coherent_drag  # без живого меша - как прежде
    assert not live.begin_coherent_drag(controller, SimpleNamespace(drag_coherent=True)) and not controller.width_coherent_drag
    source = (Path(__file__).resolve().parents[1] / "cftuv" / "envelope_width_session.py").read_text(encoding="utf-8")
    assert "finally:\n        controller.width_coherent_drag = False" in source  # конец инструмента снимает режим на любом пути


def test_the_setting_exists_and_is_on_by_default():
    source = (Path(__file__).resolve().parents[1] / "cftuv" / "envelope_production_operator.py").read_text(encoding="utf-8")
    assert 'drag_coherent: BoolProperty(' in source.replace("drag_coherent : ", "drag_coherent: ")
    block = source[source.index("drag_coherent"):][:400]
    assert "default=True" in block and "default=False" not in block
