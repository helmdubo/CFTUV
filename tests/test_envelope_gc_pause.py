"""Циклический сборщик стоит на паузе на время кнопки и возвращается в прежнее состояние (GC_PAUSED_DURING_BUTTON_V1).

Пауза не меняет ответа (GC возвращает память циклов, значений не трогает): здесь доказывается другое - что состояние сборщика после блока
ровно то, что было до него, при любом выходе, и что кнопка оператора действительно идёт под этим блоком. Тот же блок стоит на работе тика ширины
(GC_PAUSED_DURING_WIDTH_TICK_V1): блоки разных потоков считаются, а какие функции несут паузу, закреплено разбором дерева (поведение на живой ширине - `test_envelope_width_live.py`).
"""

from __future__ import annotations

import ast
import gc
import inspect
import io
import threading
from pathlib import Path

import pytest

from cftuv import envelope_domain_pool as pool_module
from cftuv.envelope_domain_pool import DomainTaskResultV1, DomainTaskV1, read_frame, serve_task
from cftuv.envelope_gc_pause import GC_PAUSE_OUTCOME, GC_PAUSE_WIDTH_OUTCOME, gc_paused, gc_paused_call, gc_paused_function

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


# --------------------------------------------------------------------------
# Тик ширины (GC_PAUSED_DURING_WIDTH_TICK_V1): блоки разных потоков считаются, пауза стоит на работе, а не на простое
# --------------------------------------------------------------------------

HOST = Path(__file__).resolve().parents[1] / "cftuv"


class _Block(threading.Thread):
    """Поток, который входит в `gc_paused`, ждёт приказа и выходит: события говорят, где он, а `was_enabled` - что он застал при входе."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.entered, self.leave, self.left = threading.Event(), threading.Event(), threading.Event()
        self.was_enabled = None

    def run(self) -> None:
        with gc_paused() as was_enabled:
            self.was_enabled = was_enabled
            self.entered.set()
            assert self.leave.wait(10)
        self.left.set()

    def enter(self) -> None:
        self.start()
        assert self.entered.wait(10)

    def exit(self) -> None:
        self.leave.set()
        assert self.left.wait(10)
        self.join(10)


@pytest.mark.parametrize("leaves_first", ["the one that entered first", "the one that entered second"])
def test_blocks_of_two_threads_that_overlap_without_nesting_keep_the_collector_off_until_the_last_one_leaves(leaves_first):
    """Счёт тика (поток) ещё летит, а кнопка уже вошла и ждёт его; либо применение входит, пока счёт выходит: порядок выходов любой, сборщик включает последний."""

    gc.enable()
    first, second = _Block(), _Block()
    first.enter()
    second.enter()
    assert (first.was_enabled, second.was_enabled) == (True, False)  # второй застал его уже выключенным: он не хозяин состояния
    assert not gc.isenabled()
    early, late = (first, second) if leaves_first == "the one that entered first" else (second, first)
    early.exit()
    assert not gc.isenabled()  # под ещё открытым блоком сборщик не включён - ни тем, кто вошёл первым, ни тем, кто вошёл вторым
    late.exit()
    assert gc.isenabled()


def test_a_collector_that_was_off_before_two_overlapping_blocks_stays_off_after_both():
    gc.disable()
    first, second = _Block(), _Block()
    first.enter()
    second.enter()
    assert (first.was_enabled, second.was_enabled) == (False, False)
    first.exit()
    second.exit()
    assert not gc.isenabled()


def test_a_block_that_raises_in_a_thread_gives_the_collector_back_while_the_main_thread_block_holds_it():
    gc.enable()
    failure = []

    def worker():
        try:
            with gc_paused():
                raise RuntimeError("tick failed")
        except RuntimeError as exc:
            failure.append(exc)

    with gc_paused():
        thread = threading.Thread(target=worker)
        thread.start()
        thread.join(10)
        assert len(failure) == 1 and not gc.isenabled()  # блок потока вышел с исключением, блок главного потока всё ещё открыт
    assert gc.isenabled()


def test_the_pause_in_a_thread_is_the_pause_of_the_process_and_ends_with_the_thread_block():
    """Сборщик общий на процесс: пока поток внутри блока, главный поток видит его выключенным; вышел поток - включён, без участия главного."""

    gc.enable()
    block = _Block()
    block.enter()
    assert not gc.isenabled()  # это читает ГЛАВНЫЙ поток
    block.exit()
    assert gc.isenabled()


def test_the_function_decorator_passes_every_argument_and_the_value_and_keeps_the_name():
    seen = []

    @gc_paused_function
    def apply(controller, request, job, live, *, extra=None):
        seen.append((controller, request, job, live, extra, gc.isenabled()))
        return "applied"

    gc.enable()
    assert apply("c", "r", "j", "l", extra=7) == "applied"
    assert seen == [("c", "r", "j", "l", 7, False)]
    assert apply.__name__ == "apply" and apply.__wrapped__ is not apply and gc.isenabled()


def test_the_function_decorator_restores_the_collector_when_the_call_raises_and_leaves_a_disabled_one_disabled():
    @gc_paused_function
    def compute(cancel):
        raise KeyError("lost")

    gc.enable()
    with pytest.raises(KeyError):
        compute(None)
    assert gc.isenabled()
    gc.disable()
    with pytest.raises(KeyError):
        compute(None)
    assert not gc.isenabled()


def _imports_of_the_pause(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return sorted(
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module == "envelope_gc_pause"
        for alias in node.names
    )


def _decorated_with(path, decorator):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return sorted(
        node.name
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and any(getattr(item, "id", None) == decorator for item in node.decorator_list)
    )


def test_the_width_tick_pause_sits_on_the_compute_the_apply_and_the_confirm_and_on_nothing_else():
    """Работа тика под паузой, простой между тиками - нет: планировщик, его опрос, заказ и проверка валидности не помечены."""

    assert _decorated_with(HOST / "envelope_width_live.py", "gc_paused_function") == ["_apply", "compute"]
    assert _decorated_with(HOST / "envelope_width_session.py", "gc_paused_function") == ["finish_adjust"]
    tree = ast.parse((HOST / "envelope_width_live.py").read_text(encoding="utf-8"))
    begin = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_begin")
    assert [node.name for node in begin.body if isinstance(node, ast.FunctionDef)] == ["compute"]  # поток исполняет ровно эту функцию (ThreadedPreviewJob)


def test_the_model_build_at_the_tool_entry_is_paused_and_the_per_frame_entry_is_not():
    """Пауза стоит на постройке модели (`_build_displayed_model`), а `ensure_model` (зовётся каждый кадр, обычно с готовой моделью) не помечен: между кадрами сборщик включён."""

    assert _decorated_with(HOST / "envelope_width_mesh_preview.py", "gc_paused_function") == ["_build_displayed_model"]
    tree = ast.parse((HOST / "envelope_width_mesh_preview.py").read_text(encoding="utf-8"))
    ensure = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "ensure_model")
    called = {item.func.id for item in ast.walk(ensure) if isinstance(item, ast.Call) and isinstance(item.func, ast.Name)}
    assert "_build_displayed_model" in called and not ensure.decorator_list


def test_only_the_button_the_pool_worker_and_the_width_tick_pause_the_collector():
    users = {
        path.name: _imports_of_the_pause(path)
        for path in sorted(HOST.glob("*.py"))
        if path.name != "envelope_gc_pause.py" and _imports_of_the_pause(path)
    }
    assert users == {
        "envelope_domain_pool.py": ["gc_paused"],
        "envelope_production_operator.py": ["gc_paused_call"],
        "envelope_width_live.py": ["gc_paused_function"],
        # Вход инструмента (WIDTH_REFRESH_INCREMENTAL_V1): первая постройка модели образца на экране (`ensure_model`) на `cover.008` стоила 260-380 мс, потому что в неё попадал проход сборщика, установившаяся - 22 мс;
        # это работа на главном потоке, а не простой, поэтому пауза стоит на самой постройке (`_build_displayed_model`), а не на `ensure_model`, который каждый кадр возвращается с готовой моделью.
        "envelope_width_mesh_preview.py": ["gc_paused_function"],
        "envelope_width_session.py": ["gc_paused_function"],
    }  # планировщик (`envelope_alpha_preview`), его таймер и опрос сборщик не трогают: простой между событиями идёт при включённом


def test_the_width_tick_outcome_is_named():
    assert GC_PAUSE_WIDTH_OUTCOME == "GC_PAUSED_DURING_WIDTH_TICK_V1"


def test_the_outcome_is_named():
    assert GC_PAUSE_OUTCOME == "GC_PAUSED_DURING_BUTTON_V1"


# --------------------------------------------------------------------------
# GC_FROZEN_AFTER_BUTTON_V1: кнопка замораживает накопленное перед включением сборщика
# --------------------------------------------------------------------------


@pytest.fixture
def _unfrozen():
    """Тест замораживает объекты процесса: постоянное поколение возвращено сборщику до и после него."""

    gc.unfreeze()
    yield
    gc.unfreeze()


def test_a_button_block_freezes_what_the_press_accumulated_before_the_collector_comes_back(_unfrozen):
    from cftuv.envelope_gc_pause import GC_FROZEN_OUTCOME

    assert GC_FROZEN_OUTCOME == "GC_FROZEN_AFTER_BUTTON_V1"
    gc.enable()
    keep = []
    with gc_paused(freeze=True):
        keep.extend([[index] for index in range(2000)])
        assert gc.get_freeze_count() == 0
    assert gc.isenabled()
    assert gc.get_freeze_count() >= 2000


def test_a_plain_block_and_a_collector_that_was_off_freeze_nothing(_unfrozen):
    gc.enable()
    with gc_paused():
        pass
    assert gc.get_freeze_count() == 0
    gc.disable()
    with gc_paused(freeze=True):
        pass
    assert gc.get_freeze_count() == 0 and not gc.isenabled()


def test_the_outer_block_decides_when_a_nested_block_asks_for_the_freeze(_unfrozen):
    gc.enable()
    with gc_paused():
        with gc_paused(freeze=True):
            pass
        assert gc.get_freeze_count() == 0 and not gc.isenabled()
    assert gc.get_freeze_count() > 0 and gc.isenabled()


def test_a_frozen_set_above_the_valve_is_given_back_to_the_collector_instead_of_growing(_unfrozen, monkeypatch):
    from cftuv import envelope_gc_pause

    gc.enable()
    with gc_paused(freeze=True):
        pass
    assert gc.get_freeze_count() > 0
    monkeypatch.setattr(envelope_gc_pause, "FREEZE_VALVE_OBJECTS", 1)
    with gc_paused(freeze=True):
        pass
    assert gc.get_freeze_count() == 0 and gc.isenabled()


def test_the_button_decorator_requests_the_freeze_and_the_tick_decorator_does_not(_unfrozen):
    gc.enable()

    @gc_paused_function
    def tick():
        return [[0]]

    tick()
    assert gc.get_freeze_count() == 0

    @gc_paused_call
    def execute(self, context):
        return {"FINISHED"}

    execute(None, None)
    assert gc.get_freeze_count() > 0


def test_a_stored_refusal_carries_no_traceback_so_it_closes_no_cycle_with_the_run():
    """PRODUCTION_REFUSAL_WITHOUT_TRACEBACK_V1: исключение-отказ по значению не держит кадры вызвавшего (цикл `refused` <-> трасса <-> кадр)."""

    from cftuv.envelope_production_scan import released

    class Failure(Exception):
        pass

    def raise_it():
        raise Failure("named refusal")

    def caller(release):
        working_set = [object() for _ in range(5)]
        refused = {"set": working_set}
        try:
            raise_it()
        except Failure as exc:
            refused["domain"] = released(exc) if release else exc
        return refused

    gc.disable()
    try:
        gc.collect()
        refused = caller(release=True)
        assert refused["domain"].__traceback__ is None and str(refused["domain"]) == "named refusal"
        del refused
        assert gc.collect() == 0  # ни одного объекта в цикле: рабочий набор умер счётчиком ссылок
        # без снятия трассы тот же код оставляет цикл (кадр `caller` держит `refused`, а `refused` - исключение с трассой на этот кадр)
        refused = caller(release=False)
        del refused
        assert gc.collect() > 0
    finally:
        gc.enable()


def test_the_released_chain_covers_the_cause_and_the_context():
    from cftuv.envelope_production_scan import released

    def deep():
        try:
            raise KeyError("first")
        except KeyError as first:
            raise ValueError("second") from first

    try:
        deep()
    except ValueError as exc:
        kept = released(exc)
    assert kept.__traceback__ is None and kept.__cause__.__traceback__ is None and str(kept) == "second"
