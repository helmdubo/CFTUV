"""Циклический сборщик стоит на паузе на время кнопки и возвращается в прежнее состояние (GC_PAUSED_DURING_BUTTON_V1).

Пауза не меняет ответа (GC возвращает память циклов, значений не трогает): здесь доказывается другое - что состояние сборщика после блока
ровно то, что было до него, при любом выходе, и что кнопка оператора действительно идёт под этим блоком.
"""

from __future__ import annotations

import ast
import gc
import inspect
import io
from pathlib import Path

import pytest

from cftuv import envelope_domain_pool as pool_module
from cftuv.envelope_domain_pool import DomainTaskResultV1, DomainTaskV1, read_frame, serve_task
from cftuv.envelope_gc_pause import GC_PAUSE_OUTCOME, gc_paused, gc_paused_call

OPERATOR = Path(__file__).resolve().parents[1] / "cftuv" / "envelope_production_operator.py"
POOL = Path(__file__).resolve().parents[1] / "cftuv" / "envelope_domain_pool.py"


@pytest.fixture(autouse=True)
def _the_collector_is_given_back():
    """Тест меняет состояние процесса: что бы он ни сделал, следующий тест застаёт сборщик таким, каким его застал этот."""

    held = gc.isenabled()
    yield
    gc.enable() if held else gc.disable()


def test_the_collector_is_paused_inside_and_enabled_again_after_a_normal_return():
    gc.enable()
    with gc_paused() as was_enabled:
        assert was_enabled is True
        assert not gc.isenabled()
    assert gc.isenabled()


def test_the_collector_is_enabled_again_after_an_exception_and_the_exception_is_not_swallowed():
    gc.enable()
    with pytest.raises(RuntimeError, match="boom"):
        with gc_paused():
            assert not gc.isenabled()
            raise RuntimeError("boom")
    assert gc.isenabled()


def test_a_collector_that_was_already_off_stays_off_inside_and_after():
    gc.disable()
    with gc_paused() as was_enabled:
        assert was_enabled is False
        assert not gc.isenabled()
    assert not gc.isenabled()
    with pytest.raises(ValueError):
        with gc_paused():
            raise ValueError("x")
    assert not gc.isenabled()


def test_a_nested_pause_does_not_enable_the_collector_before_the_outer_block_ends():
    gc.enable()
    with gc_paused():
        with gc_paused() as inner_was_enabled:
            assert inner_was_enabled is False
        assert not gc.isenabled()
    assert gc.isenabled()


def test_the_decorator_passes_both_arguments_and_the_value_and_keeps_the_name():
    seen = []

    @gc_paused_call
    def execute(self, context):
        seen.append((self, context, gc.isenabled()))
        return {"FINISHED"}

    gc.enable()
    assert execute("op", "ctx") == {"FINISHED"}
    assert seen == [("op", "ctx", False)]
    assert execute.__name__ == "execute" and gc.isenabled()


def test_the_decorated_execute_still_has_the_two_arguments_blender_checks_at_registration():
    """Blender сверяет число аргументов `execute` по коду функции при `register_class`: `*args` дало бы 0 (найдено полевым прогоном в Blender 4.5)."""

    @gc_paused_call
    def execute(self, context):
        return {"FINISHED"}

    assert execute.__code__.co_argcount == 2
    assert inspect.signature(execute).parameters.keys() == {"self", "context"}


def test_the_decorator_restores_the_collector_when_the_call_raises():
    @gc_paused_call
    def execute(self, context):
        raise KeyError("lost")

    gc.enable()
    with pytest.raises(KeyError):
        execute(None, None)
    assert gc.isenabled()


def test_the_build_button_runs_under_the_pause_and_nothing_else_in_the_operator_does():
    """Кнопка - `execute` оператора «Build Decal Mesh»; пауза держится на ней декоратором, а не вызовом где-то в середине тела.

    Оператор нельзя импортировать без Blender, поэтому проверка читает дерево разбора: именно `execute` класса оператора несёт декоратор
    `gc_paused_call`, и больше ни одна функция модуля его не использует (живая ширина идёт вне кнопки и не под паузой).
    """

    tree = ast.parse(OPERATOR.read_text(encoding="utf-8"))
    operator = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "HOTSPOTUV_OT_BuildEnvelopeDecalMesh")
    execute = next(node for node in operator.body if isinstance(node, ast.FunctionDef) and node.name == "execute")
    assert [getattr(item, "id", None) for item in execute.decorator_list] == ["gc_paused_call"]
    decorated = [
        node.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and any(getattr(item, "id", None) == "gc_paused_call" for item in node.decorator_list)
    ]
    assert decorated == ["execute"]


# --------------------------------------------------------------------------
# Воркер пула: пауза держится на одной задаче
# --------------------------------------------------------------------------


def _task(task_id=7):
    return DomainTaskV1(task_id, 1, "patch-domain:t:1", None, None, "0.25", frozenset())


def test_a_worker_task_runs_its_solution_packing_and_frame_with_the_collector_paused_and_gives_it_back(monkeypatch):
    seen = []

    def solved(task):
        seen.append(("solve", gc.isenabled()))
        return DomainTaskResultV1(task.task_id)

    def packed(task, result):
        seen.append(("pack", gc.isenabled()))
        return result

    monkeypatch.setattr(pool_module, "solve_task", solved)
    monkeypatch.setattr(pool_module, "_packed_cold_reply", packed)
    gc.enable()
    frame = serve_task(_task())
    assert seen == [("solve", False), ("pack", False)]
    assert gc.isenabled()
    assert read_frame(io.BytesIO(frame)) == DomainTaskResultV1(7)  # тот же кадр, что и без паузы


def test_a_worker_task_gives_the_collector_back_when_the_packing_raises_and_leaves_a_disabled_one_disabled(monkeypatch):
    monkeypatch.setattr(pool_module, "solve_task", lambda task: DomainTaskResultV1(task.task_id))

    def broken(task, result):
        raise RuntimeError("cannot pack")

    monkeypatch.setattr(pool_module, "_packed_cold_reply", broken)
    gc.enable()
    with pytest.raises(RuntimeError, match="cannot pack"):
        serve_task(_task())
    assert gc.isenabled()
    gc.disable()
    with pytest.raises(RuntimeError, match="cannot pack"):
        serve_task(_task())
    assert not gc.isenabled()


def test_the_worker_loop_answers_through_serve_task_and_nowhere_else_pauses_the_collector():
    tree = ast.parse(POOL.read_text(encoding="utf-8"))
    loop = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "worker_main")
    called = {item.func.id for item in ast.walk(loop) if isinstance(item, ast.Call) and isinstance(item.func, ast.Name)}
    assert "serve_task" in called and not {"solve_task", "_reply_frame", "_packed_cold_reply"} & called
    users = [node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and any(isinstance(item, ast.Name) and item.id == "gc_paused" for item in ast.walk(node))]
    assert users == ["serve_task"]


def test_the_outcome_is_named():
    assert GC_PAUSE_OUTCOME == "GC_PAUSED_DURING_BUTTON_V1"
