"""GPU_AFFINE_DRAG_V1: кадр модели превью ширины показывают узлы Blender (Geometry Nodes), а не запись меша из Python.

Что держат тесты (без Blender; в Blender то же доказывает `tools/blender_gpu_drag.py`):
1. ЧИСЛА. Атрибуты узлов (`rig_arrays`) и их формула во float32 (`emulate` - те же операции, что у группы узлов) дают кадр модели (`evaluate`) с точностью до округления float32 на
   настоящих доменах и на синтетической складке, на ширинах внутри и вне окон, с карантином; исходы кадра без массивов (`frame_summary`) - те же, что у `evaluate`.
2. ПУТЬ КАДРА. При перетаскивании с настройкой кадр не пишет меш: узлы ставятся один раз на модель (меш держит базу; держит кадр - база ложится точной записью), дальше - одно
   число в сокет; отказ узлов назван, и остаток перетаскивания пишет Python. Своя запись (сокет, снятие) не читается как внешняя правка.
3. СНЯТИЕ. Каждый путь назван и снимает: отмена, подтверждение, снятие модели, сбой постановки, остаток прошлого на старте; обработчики записи, загрузки и истории стоят в списке.
4. ВЫКЛЮЧЕНО - как прежде.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path
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
from test_envelope_width_preview_model import _fold_sample, _owned, _writes, row  # noqa: E402,F401  (`row` - фикстура настоящих доменов)

HOST = Path(__file__).resolve().parents[1] / "cftuv"


def _ulps(a, b):
    a, b = np.asarray(a, dtype=np.float32), np.asarray(b, dtype=np.float32)
    # UNFOLD_ALONG_U_V1: u - решение в плавающей точке, нулевая u выходит шумом ~1e-18, чьи относительные биты float32 не держит; ниже 2**-20 ulp меряется у 2**-20.
    return float((np.abs(a - b) / np.spacing(np.maximum(np.maximum(np.abs(a), np.abs(b)), np.float32(2.0**-20)))).max())


def _check_equal(model, widths):
    base = model.base
    arrays = gpu.rig_arrays(**pm.gpu_rig_inputs(model))
    p0, uv0 = pm.base_frame(base).positions, pm.base_frame(base).uvs
    worst = 0.0
    for width in widths:
        frame = pm.evaluate(model, width)
        positions, uvs = gpu.emulate(p0, uv0, arrays, width, base.alpha)
        worst = max(worst, _ulps(positions, frame.positions), _ulps(uvs, frame.uvs))
        summary = pm.frame_summary(model, width)
        assert (summary.live_domains, summary.held_domains, summary.held, summary.refusal, summary.approximate_domains) == (
            frame.live_domains, frame.held_domains, frame.held, frame.refusal, frame.approximate_domains)
        assert summary.positions is None and summary.uvs is None
    return worst


# ==========================================================================
# 1. Числа
# ==========================================================================


def test_the_node_formula_in_float32_is_the_model_frame_up_to_float32_rounding_on_real_domains(row):
    base, _, _ = row(0.25)
    model = pm.build_model(base)
    worst = _check_equal(model, [0.25 * (1.0 + rel) for rel in (-0.4, -0.1, -0.01, 0.0, 0.003, 0.02, 0.1, 0.6)])
    assert worst <= 4.0, worst  # Python округляет binary64 один раз, узлы - каждую операцию float32
    frame = pm.evaluate(model, 0.25)
    arrays = gpu.rig_arrays(**pm.gpu_rig_inputs(model))
    positions, uvs = gpu.emulate(frame.positions, frame.uvs, arrays, 0.25, 0.25)
    assert np.array_equal(positions, frame.positions) and np.array_equal(uvs, frame.uvs)  # на ширине базы - база побитово


def test_the_windows_of_the_nodes_hold_what_the_model_holds_and_the_quarantine_is_an_empty_window():
    base, _ = _fold_sample(0.25)
    model = pm.build_model(base)
    assert _check_equal(model, [0.2, 0.249, 0.25, 0.2501, 0.26, 0.4, 3.0]) <= 4.0
    quarantined = np.zeros(len(model.rows), dtype=bool)
    quarantined[0] = True
    vertex_quarantined = np.zeros(model.base.vertex_count, dtype=bool)
    vertex_quarantined[model.base.domains[0].vertices] = True
    from dataclasses import replace

    held = replace(model, quarantined=quarantined, vertex_quarantined=vertex_quarantined, shadow_low=model.dlow, shadow_high=model.dhigh)
    assert _check_equal(held, [0.24, 0.26]) <= 4.0
    arrays = gpu.rig_arrays(**pm.gpu_rig_inputs(held))
    assert np.all(arrays[gpu.VERTEX_LOW][vertex_quarantined] == np.inf) and np.all(arrays[gpu.VERTEX_HIGH][vertex_quarantined] == -np.inf)
    assert arrays[gpu.LOOP_LOW].shape == (model.base.loop_count,) and arrays[gpu.SLOPE].shape == (model.base.vertex_count * 3,)


# ==========================================================================
# 2. Путь кадра
# ==========================================================================


class _Rig:
    """Подмена узлов Blender: что поставлено, какие числа ушли в сокет, что снято."""

    def __init__(self, monkeypatch, *, fail_install=False):
        self.installs, self.widths, self.removed, self.strips = [], [], [], 0
        self.rig = None
        self.fail_install = fail_install
        monkeypatch.setattr(gpu, "current", lambda rig, decal, model: rig is not None and rig is self.rig and rig.model is model)
        monkeypatch.setattr(gpu, "install", self.install)
        monkeypatch.setattr(gpu, "set_width", lambda rig, decal, width: self.widths.append(width))
        monkeypatch.setattr(gpu, "remove", lambda decal: self.removed.append(decal) or True)
        monkeypatch.setattr(gpu, "strip_everywhere", self.strip)
        monkeypatch.setattr(preview, "_rig_object", lambda rig: "decal")

    def install(self, decal, arrays, base_width, model):
        if self.fail_install:
            raise gpu.GpuDragError(f"{gpu.REFUSED}:TEST")
        self.installs.append((decal, base_width, model))
        self.rig = gpu.GpuRigV1(model, "decal", 1, 2, "Socket_1", "Socket_2", base_width)
        return self.rig

    def strip(self):
        self.strips += 1
        return 1


def _gpu_world(monkeypatch, **kwargs):
    world = _owned(monkeypatch)
    world.rig = _Rig(monkeypatch, **kwargs)
    world.controller.width_drag_active = True
    world.controller.width_gpu_drag = True
    return world


def test_with_the_switch_a_frame_sets_one_number_and_never_writes_the_mesh(monkeypatch):
    world = _gpu_world(monkeypatch)
    controller, rig = world.controller, world.rig
    writes = _writes(world)
    for width in (0.26, 0.27, 0.255):
        state = preview.preview_mesh_now(controller, width)
        frame = pm.evaluate(world.model, width)
        assert state is not None and (state.live, state.held) == (frame.live_domains, frame.held_domains)
    assert _writes(world) == writes and len(rig.installs) == 1 and rig.widths == [0.26, 0.27, 0.255]  # поставлено один раз, дальше - число
    log = preview.log_of(controller)
    assert (log.gpu_frames, log.gpu_installs) == (3, 1) and log.frames == 3
    owner = controller.width_mesh_owner
    assert owner.pending_own_update and owner.holds_base  # число в сокете - наша запись (depsgraph о ней доложит), меш держит базу


def test_a_mesh_holding_a_python_frame_gets_its_base_back_before_the_nodes_take_it(monkeypatch):
    world = _gpu_world(monkeypatch)
    controller = world.controller
    controller.width_gpu_drag = False
    preview.preview_mesh_now(controller, 0.27)  # кадр Python: меш держит кадр, не базу
    assert not controller.width_mesh_owner.holds_base
    controller.width_gpu_drag = True
    writes = _writes(world)
    assert preview.preview_mesh_now(controller, 0.28) is not None
    assert _writes(world) == writes + 2  # одна точная запись базы (позиции и UV), потом - узлы
    assert controller.width_mesh_owner.holds_base and len(world.rig.installs) == 1


def test_a_new_model_is_installed_again_and_a_refused_install_falls_back_to_python_by_name(monkeypatch):
    world = _gpu_world(monkeypatch)
    controller = world.controller
    preview.preview_mesh_now(controller, 0.26)
    controller.width_model = pm.build_model(world.base)  # новый образец такта - новая модель: узлы ставятся заново
    preview.preview_mesh_now(controller, 0.26)
    assert len(world.rig.installs) == 2

    failing = _gpu_world(monkeypatch, fail_install=True)
    writes = _writes(failing)
    assert preview.preview_mesh_now(failing.controller, 0.26) is not None
    log = preview.log_of(failing.controller)
    assert _writes(failing) == writes + 2 and log.refusals == {f"{gpu.REFUSED}:TEST": 1}
    assert log.gpu_removed == {gpu.REMOVED_ON_FAILURE: 1} and failing.rig.removed and not failing.controller.width_gpu_drag


@pytest.mark.parametrize("restore, name", [(True, gpu.REMOVED_AT_CANCEL), (False, gpu.REMOVED_AT_CONFIRM)])
def test_the_end_of_the_drag_removes_the_nodes_first_by_name(monkeypatch, restore, name):
    world = _gpu_world(monkeypatch)
    controller = world.controller
    preview.preview_mesh_now(controller, 0.26)
    preview.finish_drag(controller, restore=restore)
    assert world.rig.removed == ["decal"] and controller.width_gpu_rig is None and not controller.width_gpu_drag
    assert preview.log_of(controller).gpu_removed == {name: 1}
    assert preview.preview_mesh_now(controller, 0.27) is not None and len(world.rig.installs) == 1  # после конца - кадр Python, как без настройки


def test_a_dropped_model_takes_its_nodes_with_it(monkeypatch):
    world = _gpu_world(monkeypatch)
    preview.preview_mesh_now(world.controller, 0.26)
    preview.drop_model(world.controller, preview.HISTORY_STEP)
    assert world.rig.removed == ["decal"] and preview.log_of(world.controller).gpu_removed == {gpu.REMOVED_ON_MODEL_DROP: 1}


def test_an_exact_sample_laid_under_the_nodes_reinstalls_them_at_zero_shift_or_removes_them_without_a_model(monkeypatch):
    """Прежние наклоны поверх новой базы сдвинули бы её второй раз: после точной записи узлы ставятся для новой модели на ширине образца (сдвиг ноль)."""

    world = _gpu_world(monkeypatch)
    controller = world.controller
    preview.preview_mesh_now(controller, 0.27)
    controller.width_model = pm.build_model(world.base)  # точный такт: новая модель
    preview._gpu_after_exact(controller, world.decal)
    assert len(world.rig.installs) == 2 and world.rig.installs[-1][2] is controller.width_model and world.rig.widths[-1] == world.base.alpha
    controller.width_model = None
    controller.width_model_refusal = pm.PreviewRefusalV1("TEST")
    preview._gpu_after_exact(controller, world.decal)
    assert controller.width_gpu_rig is None and preview.log_of(controller).gpu_removed == {gpu.REMOVED_ON_MODEL_DROP: 1}
    preview._gpu_after_exact(controller, world.decal)  # узлов нет - ничего
    assert preview.log_of(controller).gpu_removed == {gpu.REMOVED_ON_MODEL_DROP: 1}
    source = (HOST / "envelope_width_mesh_preview.py").read_text(encoding="utf-8")
    lean = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef) and node.name == "lean_exact_now")
    assert "_gpu_after_exact" in {getattr(node.func, "id", "") for node in ast.walk(lean) if isinstance(node, ast.Call)}


def test_a_live_tick_under_the_nodes_lays_its_own_numbers(monkeypatch):
    """LIVE_DRAG_ONE_WRITE_V1 пишет вместо чисел такта кадр к курсору; узлы берут меш за базу, поэтому под ними такт ложится своими числами, а к курсору его двигает число в сокете."""

    controller = SimpleNamespace(width_drag_active=True, width_gpu_drag=True)
    assert live.gpu_drag_active(controller)
    source = ast.get_source_segment((HOST / "envelope_width_live.py").read_text(encoding="utf-8"), next(
        node for node in ast.walk(ast.parse((HOST / "envelope_width_live.py").read_text(encoding="utf-8"))) if isinstance(node, ast.FunctionDef) and node.name == "_apply_lean"))
    assert "and not gpu_drag_active(controller)" in source


# ==========================================================================
# 3. Снятие
# ==========================================================================


def test_every_removal_path_is_named_and_the_file_handlers_strip_before_save_load_and_history():
    assert len(set(gpu.REMOVAL_PATHS)) == len(gpu.REMOVAL_PATHS) and all(name.startswith("GPU_DRAG_REMOVED_") for name in gpu.REMOVAL_PATHS)
    tree = ast.parse((HOST / "envelope_width_modal.py").read_text(encoding="utf-8"))
    handlers = next(node for node in ast.walk(tree) if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "_HANDLERS")
    pairs = {(item.elts[0].value, item.elts[1].id) for item in handlers.value.elts}
    assert {("save_pre", "_before_save"), ("load_pre", "_before_load"), ("undo_pre", "_before_history"), ("redo_pre", "_before_history"), ("load_post", "_after_load")} <= pairs
    functions = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    for function, name in (("_before_save", "REMOVED_ON_SAVE"), ("_before_load", "REMOVED_ON_LOAD"), ("_before_history", "REMOVED_ON_HISTORY"), ("_after_load", "REMOVED_ON_LOAD"),
                           ("unregister_width_tools", "REMOVED_ON_UNREGISTER")):
        names = {node.id for node in ast.walk(functions[function]) if isinstance(node, ast.Name)}
        assert "_strip_gpu" in names and name in names, function
    # Узлы ставит только кадр перетаскивания; снимают конец перетаскивания и снятие модели (каждое - своим именем).
    preview_tree = ast.parse((HOST / "envelope_width_mesh_preview.py").read_text(encoding="utf-8"))
    callers = {node.name for node in ast.walk(preview_tree) if isinstance(node, ast.FunctionDef) and any(
        isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "install" for call in ast.walk(node))}
    assert callers == {"_gpu_frame"}
    retire = {node.name for node in ast.walk(preview_tree) if isinstance(node, ast.FunctionDef) and any(
        isinstance(call, ast.Call) and getattr(call.func, "id", "") == "retire_gpu" for call in ast.walk(node))}
    assert {"finish_drag", "drop_model"} <= retire


def test_the_drag_start_strips_a_leftover_and_reads_the_switch(monkeypatch):
    stripped: list = []
    monkeypatch.setattr(live, "strip_gpu_everywhere", lambda controller, reason: stripped.append(reason) or 0)
    controller = SimpleNamespace(width_gpu_drag=None)
    assert live.begin_gpu_drag(controller, SimpleNamespace(drag_gpu_affine=True)) is True and controller.width_gpu_drag is True
    assert live.begin_gpu_drag(controller, SimpleNamespace()) is False and controller.width_gpu_drag is False  # нет настройки - выключено
    assert stripped == [gpu.REMOVED_STALE, gpu.REMOVED_STALE]


# ==========================================================================
# 4. Выключено
# ==========================================================================


def test_switched_off_the_frame_is_the_python_write_as_before(monkeypatch):
    world = _gpu_world(monkeypatch)
    world.controller.width_gpu_drag = False
    writes = _writes(world)
    assert preview.preview_mesh_now(world.controller, 0.26) is not None
    assert _writes(world) == writes + 2 and world.rig.installs == [] and world.rig.widths == []
    world.controller.width_gpu_drag = True
    world.controller.width_drag_active = False  # не перетаскивание (калбэк поля): как прежде
    assert preview.preview_mesh_now(world.controller, 0.27) is not None and world.rig.installs == []
