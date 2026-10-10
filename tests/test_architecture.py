"""Исполняемые архитектурные инварианты проекта.

Этот файл — замена прозаическому разделу правил в `AGENTS.md`.

Причина существования: в этом репозитории уже прошёл естественный эксперимент.
Инвариант «ядро не импортирует bpy» был выражен кодом (`kernel/tests/test_isolation.py`
плюс `kernel/tools/check_forbidden_imports.py`) и не нарушался ни разу. Инварианты,
записанные прозой в `AGENTS.md`, разъехались с реальностью: `band_operator.py` был
объявлен удалённым в двух документах и при этом лежал в дереве, а раздел про
тестирование утверждал «No formal tests» при 761 живом тесте.

Правило простое: **инвариант, который не исполняется, — это слух.**

Файл намеренно использует только stdlib (`ast`, `pathlib`), не импортирует ни
`bpy`, ни `mathutils`, ни сам пакет `cftuv`. Он должен запускаться в любом
окружении, включая чистый клон без единой зависимости.

Как читать провал теста: сообщение об ошибке само говорит, что делать. Если
правило больше не нужно — удалите его здесь, а не обходите.
"""

from __future__ import annotations

import ast
import re
import sys
from functools import cache
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]

HOST_PACKAGE = REPO_ROOT / "cftuv"
KERNEL_SOURCE = REPO_ROOT / "kernel" / "src"
TOOLS = REPO_ROOT / "tools"
TESTS = REPO_ROOT / "tests"


# --------------------------------------------------------------------------
# Общие помощники
# --------------------------------------------------------------------------


@cache
def _python_files(root: Path) -> tuple[Path, ...]:
    return tuple(sorted(path for path in root.rglob("*.py")))


@cache
def _source_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


@cache
def _parse(path: Path) -> ast.Module:
    return ast.parse(_source_text(path), filename=str(path))


def _imported_roots(path: Path) -> set[str]:
    """Имена, на которые ссылается модуль своими импортами.

    Учитываются все три формы, встречающиеся в проекте:
    `from .model import ...` (относительная), `import model` (плоская, из
    fallback-веток) и `from cftuv.model import ...` (пакетная из тестов).
    Для пакетной формы возвращается и корень `cftuv`, и имя подмодуля —
    иначе ссылки из тестов невидимы и живой модуль выглядит сиротой.
    """

    def _expand(dotted: str) -> set[str]:
        segments = dotted.split(".")
        names = {segments[0]}
        if segments[0] in {"cftuv", "cftuv_envelope"} and len(segments) > 1:
            names.add(segments[1])
        return names

    roots: set[str] = set()
    for node in ast.walk(_parse(path)):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots |= _expand(alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots |= _expand(node.module)
    return roots


def _relative(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


@cache
def _line_count(path: Path) -> int:
    return len(_source_text(path).splitlines())


def _max_function_lines(path: Path) -> tuple[int, str]:
    longest = 0
    name = ""
    for node in ast.walk(_parse(path)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.end_lineno is None:
                continue
            length = node.end_lineno - node.lineno
            if length > longest:
                longest, name = length, node.name
    return longest, name


# --------------------------------------------------------------------------
# 1. Границы слоёв — импортные стены
#
# Каждое правило здесь было пунктом раздела «Invariants» в AGENTS.md.
# Теперь оно проверяется, а не обещается.
# --------------------------------------------------------------------------


def test_model_layer_is_free_of_blender_runtime():
    """AGENTS.md #1: `model.py` не импортирует `bpy`/`bmesh` (только `mathutils`).

    Топологический IR обязан оставаться переносимым: его читают тесты, экспорт
    в ядро и standalone-инструменты, где Blender недоступен.
    """

    forbidden = {"bpy", "bmesh"} & _imported_roots(HOST_PACKAGE / "model.py")
    assert not forbidden, (
        f"cftuv/model.py импортирует {sorted(forbidden)}. "
        "IR обязан быть переносимым — вынесите работу с Blender в вызывающий слой."
    )


def test_debug_layer_never_reads_bmesh_directly():
    """AGENTS.md #4: `debug.py` не читает BMesh — только PatchGraph.

    Иначе визуализация начинает расходиться с тем, что реально видел солвер,
    и перестаёт быть средством проверки.
    """

    assert "bmesh" not in _imported_roots(HOST_PACKAGE / "debug.py"), (
        "cftuv/debug.py импортирует bmesh. Визуализация должна строиться "
        "из PatchGraph/AnalysisBundle, иначе она показывает не то, что решал солвер."
    )


def test_analysis_layer_never_imports_solve_layer():
    """AGENTS.md #3: анализ не знает о солвере. Поток строго `analysis -> solve`.

    Обратное ребро делает форму патча зависимой от runtime-роли, а это ровно
    та цикличность, которую запрещает контракт слоёв ролей.
    """

    offenders = {}
    for path in _python_files(HOST_PACKAGE):
        if not path.name.startswith("analysis"):
            continue
        leaked = {
            root
            for root in _imported_roots(path)
            if root.startswith(("solve", "frontier"))
        }
        if leaked:
            offenders[_relative(path)] = sorted(leaked)
    assert not offenders, (
        f"Слой анализа импортирует слой решения: {offenders}. "
        "Поток обязан быть односторонним: analysis -> solve."
    )


def test_frontier_runtime_is_free_of_blender():
    """Ядро размещения (`frontier_*.py`) не должно зависеть от Blender.

    Это то, что делает фронтир тестируемым без Blender и переносимым дальше.
    Запись UV — обязанность `solve_transfer.py`, ему Blender разрешён.
    """

    offenders = {}
    for path in _python_files(HOST_PACKAGE):
        if not path.name.startswith("frontier_"):
            continue
        leaked = {"bpy", "bmesh"} & _imported_roots(path)
        if leaked:
            offenders[_relative(path)] = sorted(leaked)
    assert not offenders, (
        f"Runtime фронтира зависит от Blender: {offenders}. "
        "Запись в UV принадлежит solve_transfer.py, а не логике размещения."
    )


def test_alpha_preview_core_is_free_of_blender_runtime():
    """Планировщик фонового превью alpha не импортирует `bpy`: таймеры и цель приходят параметрами.

    Тогда его закон (пауза, слияние, один полёт, устаревшее) проверяется без Blender, а поток счёта,
    которому он принадлежит, не может незаметно дотянуться до данных Blender через импорт.
    """

    leaked = {"bpy", "bmesh", "mathutils"} & _imported_roots(
        HOST_PACKAGE / "envelope_alpha_preview.py"
    )
    assert not leaked, (
        f"envelope_alpha_preview.py импортирует {sorted(leaked)}. Данные Blender читает и пишет "
        "только главный поток (envelope_alpha_preview_gp: begin/valid/apply)."
    )


#: Имена, которые смеет использовать функция, исполняемая ПОТОКОМ счёта превью: питоновские объекты,
#: захваченные на главном потоке, ядро и отмена. Ни `bpy`, ни настроек, ни контроллера.
_PREVIEW_COMPUTE_NAMES = frozenset(
    {
        "entries",
        "alpha_text",
        "coverage_pool",
        "cancel",
        "recompute_queue_coverage",
        "CoverageCancelled",
        "PreviewCancelled",
        "exc",
        "str",
    }
)


def test_the_alpha_preview_worker_thread_function_touches_no_blender_data():
    """AGENTS.md (хост): данные Blender из потока не читаются и не пишутся.

    Поток счёта исполняет `compute` из `_begin`; все её имена обязаны быть из названного набора.
    Новое имя (`bpy`, `settings`, `controller`, `context`) — повод остановиться и вынести это в
    `_apply`/`_validity`, которые идут на главном потоке.
    """

    path = HOST_PACKAGE / "envelope_alpha_preview_gp.py"
    begin = next(
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef) and node.name == "_begin"
    )
    compute = next(
        node
        for node in ast.walk(begin)
        if isinstance(node, ast.FunctionDef) and node.name == "compute"
    )
    used = {node.id for node in ast.walk(compute) if isinstance(node, ast.Name)}
    used -= {arg.arg for arg in compute.args.args}
    foreign = used - _PREVIEW_COMPUTE_NAMES
    assert not foreign, (
        f"функция потока счёта превью использует {sorted(foreign)}: потоку нельзя ничего, "
        "что читает или пишет данные Blender."
    )


def test_the_alpha_slider_callback_only_orders_the_preview():
    """Калбэк `update` ползунка alpha не считает и не рисует на главном потоке.

    Прежний путь (`update_queue_alpha`: покрытие, пул и перерисовка слоёв прямо в калбэке) стоил 0.3 с
    на `2` и до 1.2 с на `building` на КАЖДОЕ движение мыши. Возврат к нему ловится здесь.
    """

    path = HOST_PACKAGE / "operators.py"
    callback = next(
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef)
        and node.name == "_update_envelope_debug_alpha"
    )
    names = {node.id for node in ast.walk(callback) if isinstance(node, ast.Name)} | {
        node.module or ""
        for node in ast.walk(callback)
        if isinstance(node, ast.ImportFrom)
    }
    assert "schedule_alpha_preview" in {
        alias.name
        for node in ast.walk(callback)
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    heavy = {
        "update_queue_alpha",
        "recompute_queue_coverage",
        "redraw_envelope_queue_layers",
    }
    assert not (heavy & names), sorted(heavy & names)
    assert not any(
        "update_queue_alpha" in _source_text(item)
        for item in _python_files(HOST_PACKAGE)
        if item.name != "operators.py"
    ), "update_queue_alpha удалён (синхронный счёт в калбэке); его возврат — регресс"


def _module_level_import_roots(path: Path) -> set[str]:
    """Корни импортов ТОЛЬКО верхнего уровня модуля (ленивые импорты внутри функций не в счёт)."""

    roots: set[str] = set()
    for node in _parse(path).body:
        if isinstance(node, ast.Import):
            roots |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def test_the_width_preview_and_adjust_cores_and_the_live_glue_load_without_blender():
    """Превью ширины и автомат инструмента не знают Blender вовсе; склейка живой ширины тянет `bpy` лишь лениво.

    Тогда геометрия превью и закон инструмента проверяются без Blender, а модуль, который их держит, не может
    незаметно дотянуться до данных Blender на импорте.
    """

    for name in ("envelope_width_preview.py", "envelope_width_adjust.py"):
        leaked = {"bpy", "bmesh", "mathutils", "gpu"} & _imported_roots(HOST_PACKAGE / name)
        assert not leaked, f"{name} импортирует {sorted(leaked)}: превью и автомат чистые"
    for name in ("envelope_width_live.py", "envelope_width_session.py", "envelope_width_mesh_preview.py"):
        leaked = {"bpy", "bmesh", "mathutils", "gpu"} & _module_level_import_roots(HOST_PACKAGE / name)
        assert not leaked, f"{name} импортирует {sorted(leaked)} на верхнем уровне: только лениво, внутри функций"
    # Модель превью меша — чистая математика над массивами: ей Blender не нужен нигде, даже лениво.
    leaked = {"bpy", "bmesh", "mathutils", "gpu"} & _imported_roots(HOST_PACKAGE / "envelope_width_preview_model.py")
    assert not leaked, f"envelope_width_preview_model.py импортирует {sorted(leaked)}: модель чистая (её строит поток счёта)"


#: Имена, которые смеет использовать функция, исполняемая ПОТОКОМ точного пересчёта живой ширины: объекты,
#: захваченные на главном потоке (пакет анализа, выделение, ключи, пул, допуски растяжения и UV), прогон продуктового пути и отмена.
_WIDTH_COMPUTE_NAMES = frozenset(
    {
        "run_production",
        "ProductionCancelled",
        "PreviewCancelled",
        "controller",
        "bundle",
        "selected",
        "alpha",
        "object_key",
        "data_key",
        "density",
        "budget",
        "slide",
        "kernel_backend",
        "skeleton_backend",
        "embedding_backend",
        "pool",
        "cancel",
        "exc",
        "str",
        # Хвост потока (`finish_live_run`): массивы меша, образец, модель, сверка и журнал доверия строятся ТАМ, а не на главном потоке;
        # все они значения (неизменяемые образцы, модели, журналы и числа), захваченные `_begin` на главном потоке.
        "finish_live_run",
        "trust",
        "run",
        "offset",
        "key",
        "displayed",
        "aux",
        "previous",
        "prime",
    }
)


def test_the_width_live_worker_thread_function_touches_no_blender_data():
    """Поток точного пересчёта ширины исполняет `compute` из `_begin`: имена только из названного набора.

    Контроллер и пакет анализа — питоновские объекты сессии, не данные Blender; `bpy`, объекты, настройки и
    контекст читает только главный поток (`_begin`, `_validity`, `_apply`).
    """

    path = HOST_PACKAGE / "envelope_width_live.py"
    begin = next(
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef) and node.name == "_begin"
    )
    compute = next(
        node
        for node in ast.walk(begin)
        if isinstance(node, ast.FunctionDef) and node.name == "compute"
    )
    used = {node.id for node in ast.walk(compute) if isinstance(node, ast.Name)}
    used -= {arg.arg for arg in compute.args.args}
    foreign = used - _WIDTH_COMPUTE_NAMES
    assert not foreign, (
        f"функция потока точной ширины использует {sorted(foreign)}: потоку нельзя ничего, "
        "что читает или пишет данные Blender."
    )
    calls = {
        node.func.id
        for node in ast.walk(compute)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "run_production" in calls


def test_the_width_overlay_only_draws_and_never_computes_the_geometry():
    """Обработчик отрисовки рисует готовые линии: формулы геометрии живут в чистой функции, которую тестируют."""

    path = HOST_PACKAGE / "envelope_width_overlay.py"
    names = {node.id for node in ast.walk(_parse(path)) if isinstance(node, ast.Name)} | {
        alias.name
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert not ({"compute_width_preview", "build_preview_inputs", "chain_centroid"} & names)
    assert "envelope_width_preview" not in _imported_roots(path)


def _writer_references(path: Path) -> set[str]:
    """Какие писатели меша декали модуль вызывает либо импортирует (по AST, а не по тексту)."""

    found = set()
    for node in ast.walk(_parse(path)):
        if isinstance(node, ast.Call):
            callee = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, "attr", "")
            found.add(callee)
        elif isinstance(node, ast.ImportFrom):
            found |= {alias.name for alias in node.names}
    return found & {"rewrite_decal_mesh", "write_decal_object"}


def test_the_width_tool_never_writes_the_mesh_from_the_preview_path():
    """Линии превью не пишутся в меш: писатели точного результата зовут только точные пути (кнопка и точный результат живой ширины).

    Превью МЕША (`PREVIEW_MESH_FROM_INTERVAL_V1`) пишет в меш другой писатель, `write_preview_geometry`, и о нём — следующий тест.
    """

    callers = {}
    for path in _python_files(HOST_PACKAGE):
        if path.name == "envelope_production_mesh.py":
            continue
        for writer in _writer_references(path):
            callers.setdefault(writer, set()).add(path.name)
    assert callers == {
        "rewrite_decal_mesh": {"envelope_width_live.py"},
        "write_decal_object": {"envelope_production_operator.py"},
    }, callers


def test_only_the_preview_mesh_glue_writes_the_preview_geometry_and_only_from_its_two_frame_functions():
    """Позиции и UV превью пишет один писатель (`write_preview_geometry`), и звать его вправе лишь кадр и возврат базы.

    Превью меша двигает ТОТ ЖЕ меш на месте (`foreach_set`), не создавая и не освобождая датаблоки: поэтому его можно звать из
    таймера и модального оператора, не трогая шаг отмены. Любой другой вызов писателя — повод остановиться: превью, записанное
    мимо проверки состава меша (`_mesh_problem`), легло бы на чужую геометрию.
    """

    callers = {}
    for path in _python_files(HOST_PACKAGE):
        if path.name == "envelope_production_mesh.py":
            continue
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.FunctionDef):
                for call in ast.walk(node):
                    if isinstance(call, ast.Call) and _called_names_of(call) == "write_preview_geometry":
                        callers.setdefault(path.name, set()).add(node.name)
    assert callers == {"envelope_width_mesh_preview.py": {"preview_mesh_now", "restore_base_mesh"}}, callers
    path = HOST_PACKAGE / "envelope_width_mesh_preview.py"
    for function in ("preview_mesh_now", "restore_base_mesh"):
        node = next(item for item in ast.walk(_parse(path)) if isinstance(item, ast.FunctionDef) and item.name == function)
        assert "_mesh_problem" in _called_names(node), f"{function} пишет превью, не спросив, принадлежит ли меш сертификату"


def _functions_of(name: str) -> dict:
    return {
        node.name: node
        for node in ast.walk(_parse(HOST_PACKAGE / name))
        if isinstance(node, ast.FunctionDef)
    }


def test_every_exact_write_of_the_decal_mesh_records_the_mesh_ownership_in_the_same_function():
    """Точная запись меша (кнопка, точный результат живой ширины) фиксирует владение мешем СРАЗУ после записи (`capture_ownership`).

    Превью меша пишется только в меш, которым владеет сессия: тождество объекта и датаблока, поколение раскладки и её отпечаток, снятые
    сразу после точной записи (аудит ad6074f, F2). Функция, которая ставит образец на экран (`note_button_display`, `note_exact_display`), без
    фиксации владения оставила бы превью без доказательства, что меш тот, а размеры и свойства такого доказательства не дают.
    """

    found: dict = {}
    for path in _python_files(HOST_PACKAGE):
        if path.name == "envelope_width_mesh_preview.py":
            continue
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.FunctionDef):
                names = _called_names(node)
                for noted in ("note_button_display", "note_exact_display"):
                    if noted in names:
                        found.setdefault(noted, {})[(path.name, node.name)] = "capture_ownership" in names
    assert set(found) == {"note_button_display", "note_exact_display"}, found
    assert {key[0] for key in found["note_button_display"]} == {"envelope_production_operator.py"}, found
    assert {key[0] for key in found["note_exact_display"]} == {"envelope_width_live.py"}, found
    assert all(captured for item in found.values() for captured in item.values()), found


def test_the_decal_ownership_is_checked_before_a_preview_write_and_every_external_change_drops_the_model():
    """Кадр сверяет владение (поколение, указатели и `session_uid`), а внешнее изменение (история, depsgraph) снимает модель названно.

    Строго, а не «сверим потом»: после шага истории или чужого обновления геометрии декали указатели, счётчики и свойства меша
    не доказывают ничего (аудит ad6074f, F2).
    """

    preview = _functions_of("envelope_width_mesh_preview.py")
    problem = preview["_mesh_problem"]
    attributes = {node.attr for node in ast.walk(problem) if isinstance(node, ast.Attribute)}
    assert {"width_mesh_owner", "sample", "as_pointer", "session_uid", "recheck_after"} <= attributes, sorted(attributes)
    assert "_read_layout" in _called_names(problem)
    for function in ("preview_mesh_now", "restore_base_mesh", "note_history", "note_decal_updates"):
        assert "drop_model" in _called_names(preview[function]), f"{function} не снимает модель названно"
    assert "_mesh_problem" in _called_names(preview["preview_mesh_now"]) and "_mesh_problem" in _called_names(preview["restore_base_mesh"])
    assert "note_decal_updates" in _called_names(_functions_of("envelope_width_live.py")["note_depsgraph"])
    assert "note_depsgraph" in _called_names(_functions_of("envelope_width_modal.py")["_after_depsgraph"])
    assert "note_history" in _called_names(_functions_of("envelope_width_live.py")["reconcile_after_history"])


def test_a_refuted_domain_reaches_the_trust_ledger_and_the_next_model_is_built_under_it():
    """Опровержение точным прогоном не только записывается: оно двигает журнал доверия, а следующая модель строится под ним (аудит F5)."""

    preview = _functions_of("envelope_width_mesh_preview.py")
    assert {"deviation", "advance_ledger", "_model_or_refusal"} <= _called_names(preview["finish_live_run"])
    assert "build_model" in _called_names(preview["_model_or_refusal"])
    assigned = {
        target.attr
        for node in ast.walk(preview["note_exact_display"])
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Attribute)
    }
    assert "width_trust" in assigned, "точный результат обязан положить журнал доверия в сессию"
    compute = next(
        node
        for node in ast.walk(_functions_of("envelope_width_live.py")["_begin"])
        if isinstance(node, ast.FunctionDef) and node.name == "compute"
    )
    keywords = {
        keyword.arg
        for call in ast.walk(compute)
        if isinstance(call, ast.Call) and _called_names_of(call) == "finish_live_run"
        for keyword in call.keywords
    }
    assert "trust" in keywords, "поток точного счёта обязан получить журнал доверия значением"


def test_the_preview_model_is_never_called_a_certificate():
    """Приблизительная модель превью (аудит ad6074f, F4) нигде не названа сертификатом: ни идентификатором, ни строкой исхода или статуса.

    Сертифицирован только ИНТЕРВАЛ событий и структуры у ядра (R1); многочлен внутри него, его область доверия и самопроверка — эвристика
    без доказанной границы ошибки. Имя, которое обещает больше, чем есть, — то, что аудит нашёл и что здесь закрыто исполняемо.
    """

    assert not (HOST_PACKAGE / "envelope_width_certificate.py").exists(), "модель превью называется `envelope_width_preview_model`"
    for name in (
        "envelope_width_preview_model.py",
        "envelope_width_mesh_preview.py",
        "envelope_width_live.py",
        "envelope_width_session.py",
        "envelope_width_modal.py",
    ):
        tree = _parse(HOST_PACKAGE / name)
        docstrings = {
            id(node.body[0].value)
            for node in ast.walk(tree)
            if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef))
            and node.body
            and isinstance(node.body[0], ast.Expr)
            and isinstance(node.body[0].value, ast.Constant)
        }
        words = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                words.append(node.id)
            elif isinstance(node, ast.Attribute):
                words.append(node.attr)
            elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                words.append(node.name)
            elif isinstance(node, ast.arg):
                words.append(node.arg)
            elif isinstance(node, ast.alias):
                words.append(node.name)
            elif isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings:
                words.append(node.value)
        called = [word for word in words if "certificate" in word.lower()]
        assert not called, f"{name} называет приблизительную модель сертификатом: {called}"


def _called_names_of(call: ast.Call) -> str:
    return call.func.id if isinstance(call.func, ast.Name) else getattr(call.func, "attr", "")


#: Входы инструмента ширины и вопрос, который каждый из них обязан задавать: у АКТИВНОГО объекта есть своя декаль,
#: а запись кнопки этого окна — про него (`envelope_width_live.availability_problem`). Запись кнопки одна на окно, и
#: вход без этого вопроса правит декаль объекта, который уже не активен (ошибка владельца: «аджастмент есть, а сетки
#: для аджастмента нет»).
_WIDTH_ENTRY_POINTS = (
    ("envelope_width_modal.py", "poll", "poll_problem"),
    ("envelope_width_modal.py", "invoke", "poll_problem"),
    ("envelope_width_session.py", "poll_problem", "width_problem"),
    ("envelope_width_session.py", "begin_adjust", "poll_problem"),
    ("envelope_width_live.py", "schedule_width_live", "width_problem"),
    ("envelope_width_live.py", "reconcile_after_history", "width_problem"),
    ("envelope_width_live.py", "draw_decal_width_rows", "width_problem"),
    ("envelope_width_live.py", "sync_width_field", "width_problem"),
    ("envelope_width_live.py", "follow_active_object", "width_problem"),
    ("envelope_width_live.py", "ensure_prime", "width_problem"),
    ("envelope_width_live.py", "width_problem", "availability_problem"),
)


def _called_names(function: ast.FunctionDef) -> set[str]:
    return {
        node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, "attr", "")
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
    }


@pytest.mark.parametrize("module, function, question", _WIDTH_ENTRY_POINTS)
def test_every_width_entry_point_asks_whether_the_active_object_has_its_own_decal(module, function, question):
    path = HOST_PACKAGE / module
    found = [
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef) and node.name == function
    ]
    assert found, f"{module}: функции {function} нет"
    assert all(question in _called_names(node) for node in found), (
        f"{module}:{function} не зовёт {question}: вход инструмента ширины обходит вопрос "
        "«есть ли у активного объекта своя свежая декаль»."
    )


def test_the_width_tool_names_the_reason_on_the_disabled_button_and_follows_the_active_object():
    """Отключённая кнопка называет причину (`poll_message_set`), а смена активного объекта идёт через depsgraph."""

    path = HOST_PACKAGE / "envelope_width_modal.py"
    poll = next(
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef) and node.name == "poll"
    )
    assert "poll_message_set" in _called_names(poll)
    handlers = next(
        node.value
        for node in _parse(path).body
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "_HANDLERS" for t in node.targets)
    )
    registered = {
        item.elts[0].value for item in handlers.elts if isinstance(item, ast.Tuple)
    }
    assert "depsgraph_update_post" in registered, sorted(registered)


# --------------------------------------------------------------------------
# 2. Мёртвый код
# --------------------------------------------------------------------------


# Модули, на которые не ссылается ни один импорт, но которые нужны как точки
# входа. Всё остальное без входящих ссылок — мёртвый код.
ENTRY_POINT_MODULES = frozenset({"__init__"})


def test_no_orphan_modules_in_host_package():
    """Модуль без единой входящей ссылки — мёртвый код, удаляйте его.

    Этот тест пойман на реальном случае: `cftuv/band_operator.py` (447 строк)
    жил в дереве, хотя `docs/cftuv_cleanup_decisions.md` объявлял его удалённым,
    а `docs/cftuv_cleanup_inventory.md` — что его возвращение считается
    регрессией. Проза не смогла это удержать, проверка может.
    """

    modules = {
        path.stem for path in _python_files(HOST_PACKAGE)
    } - ENTRY_POINT_MODULES

    referenced: set[str] = set()
    for path in _python_files(HOST_PACKAGE) + _python_files(TESTS) + _python_files(TOOLS):
        for root in _imported_roots(path):
            if root in modules and root != path.stem:
                referenced.add(root)

    orphans = sorted(modules - referenced)
    assert not orphans, (
        f"Модули без входящих ссылок: {orphans}. "
        "Мёртвый код удаляется, а не сохраняется «на всякий случай» — "
        "история git помнит его и без этого."
    )


DELETED_LEGACY_PATHS = (
    "Hotspot_UV_v2_5_19.py",
    "Hotspot_UV_v2_5_26.py",
    ".tmp_review",
    "cftuv/band_operator.py",
    # Фаза 4 роадмапа: legacy decal-конвейеры PATCH_VORONOI и RAIL_PLANAR
    # вместе со всем хостом, который существовал только ради них.
    "cftuv/decals.py",
    "cftuv/decal_voronoi.py",
    "cftuv/decal_rails.py",
    "cftuv/decal_rail_geometry.py",
    "cftuv/decal_charts.py",
    "cftuv/decal_chart_admission.py",
    "cftuv/decal_chart_measurement.py",
    "cftuv/decal_chart_parametrization.py",
    "cftuv/decal_atlas.py",
    "cftuv/decal_corner_model.py",
    "cftuv/decal_distance_witness.py",
    "cftuv/decal_diagram.py",
    "cftuv/decal_geometry.py",
    "cftuv/decal_session.py",
    "cftuv/decal_modal.py",
    "cftuv/decal_gpu_preview.py",
    "cftuv/decal_transform.py",
)


def test_deleted_legacy_stays_deleted():
    """Удалённое легаси не возвращается: монолит, BAND-оператор, decal-конвейеры.

    `AGENTS.md` требовал этого прозой; правило нарушалось. Теперь оно исполняемое.
    Decal-конвейеров было три, и каждая возможность стоила втрое; остался один —
    envelope-ядро. Возвращение старого движка — не откат, а третий конвейер снова.
    """

    resurrected = [name for name in DELETED_LEGACY_PATHS if (REPO_ROOT / name).exists()]
    assert not resurrected, (
        f"Удалённое легаси вернулось в дерево: {resurrected}. "
        "Если код снова нужен — достаньте его из истории git осознанным коммитом "
        "и удалите запись из DELETED_LEGACY_PATHS."
    )


# --------------------------------------------------------------------------
# 3. Бюджеты размера — храповик
#
# Значения ниже заморожены по состоянию на момент введения бюджетов.
# Их можно только УМЕНЬШАТЬ. Увеличение значения в таблице — это заявление
# «я осознанно наращиваю долг», и оно должно обсуждаться, а не проходить молча.
#
# Файла нет в таблице => действует строгий лимит для нового кода.
# --------------------------------------------------------------------------


NEW_MODULE_LINE_LIMIT = 2000
NEW_FUNCTION_LINE_LIMIT = 120


# Модули, превышающие NEW_MODULE_LINE_LIMIT на момент заморозки.
MODULE_LINE_ALLOWANCE = {
    # DENS-PROJECTIVE-CHART: владелец разрешил узкий остаточный потолок после
    # extraction exact-atlas helpers в sibling `adaptive_density_atlas.py`.
    # Старый single-chart закон оставлен в исходном модуле ради byte-stability.
    "kernel/src/cftuv_envelope/reference/adaptive_density_fan.py": 2200,
    # 3107 -> 3053 (измерение угла ушло в ядро) -> 3070. +17 куплены осознанно:
    # семь счётчиков стадии INTERACTION, самой дорогой в поле (4595 мс из
    # 11.8 с на центральном патче) и единственной, про которую до них нечего
    # было сказать. Оптимизировать вслепую дороже семнадцати строк. Файл всё
    # ещё на 37 строк меньше, чем был до этой ветки, и по-прежнему числится
    # открытым блокером HOST_REQUEST_EXPORT_COMPLEXITY: его настоящее лечение —
    # генерация маппера из JSON-схемы, а не бритьё строк.
    # 3086 -> 3101. +15 за счётчики привязки к решётке (R0). Заведены ДО самой
    # решётки намеренно: срез, который её вводит, иначе нечем принимать, а
    # счётчик, появившийся вместе с изменением, не может показать, что было до
    # него. Ниже — прежняя запись про ALGEBRAIC_CANONICALIZATIONS.
    # 3070 -> 3086. +16 за ALGEBRAIC_CANONICALIZATIONS — единственную величину,
    # которая объяснила полевое время после того, как его не объяснили ни
    # пересечения, ни число вкладов, ни длина чисел, ни сканы локализации точки.
    # Профиль bf6: 95 с из 158 в `_canonical_expr`.
    # 3114 -> 3115. +1: `grid_policy=` в вызове построения метрики. Политику
    # решётки хост обязан называть сам — ядро её за него не выбирает, ровно как
    # с планарностью, — и одна строка это ровно тот минимум, которым это
    # называется.
    # 3115 -> 3098. −17: пролог постадийного прогона (сцена топологии, перечень
    # доменов, их выделенные рёбра и `DecalRequestId`) вынесен в
    # `envelope_topology_export.stage_domain_inputs`. Второй движок (QUEUE)
    # обязан адресовать домены ТЕМИ ЖЕ номерами, иначе его колонка несравнима
    # с эталонной; копия пролога сделала бы сравнимость свойством копии.
    # Блокер HOST_REQUEST_EXPORT_COMPLEXITY остаётся открытым — файл стал
    # меньше, но не перестал быть слишком большим.
    # 3098 -> 3055. −43: разрешение выделения владельца (дополнение до полных
    # PhysicalChain, области доменов, проверка пары швов) вынесено в
    # `envelope_topology_export.resolve_selection_scope`. Выделение — факт
    # топологии хоста, а не запроса: тот же ответ нужен обоим движкам и
    # совместимому пути, и три копии дополнения разошлись бы. Блокер по-прежнему
    # открыт.
    # 3055 -> 3093. +38 за разведение схлопнутого имени отказа метрики. Хост
    # сводил ВСЕ `PlanarMetricAdmissionError` к
    # `RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED`, поэтому поле читало
    # «нужна near-planar политика» ровно тогда, когда она уже была включена, а
    # отказал бюджет невязки. Куплены: пять имён исходов ядра в
    # `EnvelopeDebugHostOutcome`, перенос исхода ПО ИМЕНИ (`_host_outcome_for`)
    # вместо одной константы и `METRIC_STAGE_OUTCOMES` — множество отказов
    # ступени METRIC, названное один раз, чтобы разведение имени не переносило
    # домен на ступень COMPILE молча. Дешевле не выходит: имя, указывающее не на
    # ту причину, стоило полевой сессии диагностики. Блокер
    # HOST_REQUEST_EXPORT_COMPLEXITY по-прежнему открыт.
    # 3093 -> 2921. −172 при ДОБАВЛЕННОМ законе разреза физической цепочки по
    # точному излому (~172 строки: точный тест коллинеарности в 3D, обобщение
    # `_normalize_physical_seam_partitions` с PATCH-швов на все виды цепей,
    # `_angular_sites` — вершины разреза наравне с объявленными углами).
    # Оплачено удалением мёртвого `_exact_frame` (344 строки, вызовов ноль:
    # рациональная аффинная метрика вытеснила точный кадр ещё в прошлой ветке,
    # а orphan-тест ловит модули, не функции). Число опущено до фактического:
    # храповик затягивается там, где освободилось место, иначе освобождённое
    # место молча превращается в разрешение расти обратно.
    # 2921 -> 2919. −2 при добавленных исходах метрики (σ NEAR_PLANAR V2)
    # политике укладки и политике репера хоста: трёхстрочные члены enum склеены
    # в однострочные, новый параметр метрики встал в существующий вызов. Число
    # опущено до фактического.
    # 2919 -> 2916. −3 при ДОБАВЛЕННЫХ девяти исходах ступени развёртки (S1 DEVELOPABLE) в
    # `EnvelopeDebugHostOutcome` и в `METRIC_STAGE_OUTCOMES` и одной строке политики лестницы
    # кривизны в вызове метрики: семь трёхстрочных членов enum склеены в однострочные, а
    # девятнадцать имён множества ступени METRIC разложены по два в строке. Число опущено до
    # фактического.
    # 2916 -> 2911. −5 при ДОБАВЛЕННОМ имени отказа прямизны объявленной цепи
    # (`DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT`: член enum и имя в множестве METRIC), одном
    # параметре метрики и одной строке выборки объявленных прямыми цепей из снапшота: сигнатура
    # `_rational_affine_metric` и её вызов уложены в меньшее число строк. Число опущено до
    # фактического.
    # 2911 -> 2915. +4 за допуск растяжения развёртки ЗАПРОСА (STRETCH-BUDGET-POLICY): параметр `budget` метрики и
    # `developable_stretch_budget` в вызове ядра, в `build_envelope_decal_request` и в вызове запроса постадийного прогона.
    # Строки оформлены обычно (по одному аргументу), а не склеены ради потолка: аудит нашёл склейки нечитаемыми. Блокер
    # HOST_REQUEST_EXPORT_COMPLEXITY по-прежнему открыт.
    # 2915 -> 2918. +3 за память замечаний снапшота (PERF MATERIALIZE-SPEED): параметр `snapshot_issues_of` у
    # `build_envelope_decal_request` и его вызов с допуском растяжения запроса (замечания от допуска зависят, и память
    # сессии ключует по нему). Число поднято осознанно, до фактического.
    # 2918 -> 2945. +27 за полосовую карту (ПОЛОСА-C1): четыре исхода хоста (`EnvelopeDebugHostOutcome`) и три из них в
    # `METRIC_STAGE_OUTCOMES` (+7), параметр `chart_band` метрики и его вызов (+5), пропуск углов вне карты-полосы
    # (`ANGULAR_CORNERS_BEYOND_CHART_REACH`: перехват `BeyondChartReach` вокруг двух опор, проверка грани вне носителя и
    # параметр `triangle_ids` построителя углов, +12), политика досягаемости в запросе и отказ по alpha (`chart_reach_cap`, +3
    # вызова помощников). Сама логика полосы (выбор цепей, триггеры, отказ по alpha, координаты карты-полосы) вынесена в
    # `envelope_chart_band.py`, чтобы файл не рос ею. Блокер HOST_REQUEST_EXPORT_COMPLEXITY по-прежнему открыт. Число поднято
    # осознанно, до фактического.
    # 2945 -> 2880. −65 при добавленных трёх исходах разреза кольца (+3 члена `EnvelopeDebugHostOutcome`, +1 строка
    # `METRIC_STAGE_OUTCOMES`), именах карты у вершин разреза (`chart_edge_ends`, `chart_face_points`, пропуск углов на пути
    # разреза: +5) и новом правиле объявленного угла без стыка цепей (+5). Перечень мест поворота `_angular_sites` (66 строк)
    # вынесен в `envelope_angular_sites.angular_sites`: правило про объявленные углы на гладких замкнутых петлях - свой модуль,
    # а файл на потолке растёт только вызовами. Число опущено до фактического: храповик затягивается там, где освободилось
    # место. Блокер HOST_REQUEST_EXPORT_COMPLEXITY по-прежнему открыт.
    # 2880 -> 2882. +2 за факт хоста «грани соседа шва» (`AnalysisSnapshotV1.seam_neighbour_faces`, план станций цепей): импорт
    # помощника и его вызов в сборке снапшота; сама логика (какие грани соседа касаются внутренних вершин цепей) живёт в
    # `envelope_seam_neighbours.py`, а срез поверхности несёт `envelope_topology_export._PatchSurfaceIdView.neighbour_faces`.
    # Число поднято осознанно, до фактического.
    # 2882 -> 2884. +2 за закон масштаба решётки повторной попытки после отказа лотереи привязки (SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1): строка
    # передачи закона ядру (только если он заказан) и строка его чтения из экспорта топологии. Повтор, множество отказов, ключи и запись
    # исхода живут в `envelope_snap_retry.py`. Число поднято осознанно, до фактического.
    "cftuv/envelope_request_export.py": 2884,
    # 2913 -> 3055. +142 за движок QUEUE в панели: EnumProperty движка,
    # строка тайминга, чекбокс слоёв очереди, update-callback ползунка alpha
    # (лёгкий путь без единой компиляции) и запоминание тёплой сессии. Панель
    # Envelope Debug при этом вынесена из `draw` отдельной функцией, и предел
    # самой длинной функции файла упал с 228 до 208 строк.
    # 3055 -> 3044. −11: файл стоял РОВНО на своём потолке, и чекбокс видимости
    # нового слоя отказа было некуда положить. Оплачено переносом двух функций
    # туда, где живут их данные: сборка строки сводки — в
    # `envelope_debug_profile.stage_summary_text`, запоминание тёплой сессии
    # очереди — в `envelope_debug_session.remember_queue_session`. Оператор
    # остался вызывающим, а не владельцем этих правил.
    # 3044 -> 2018. −1026: Фаза 4 удалила legacy decal-путь целиком — оператор
    # `HOTSPOTUV_OT_GenerateDecals` («Decal Seams»), пятнадцать decal-свойств
    # сцены и их блок панели. Число опущено до фактического: строгий лимит
    # нового кода файл превышает на 18 строк, и запись уйдёт вместе с ними.
    # 2018 -> 2016. Ручка числа воркеров пула доменов оплачена удалением
    # мёртвого `_envelope_debug_outcome_value` и неиспользуемого импорта;
    # импорт политики запроса сведён в одну строку — 2014.
    # 2014 -> 2031. +17 за ручку «Max stretch» (допуск растяжения развёртки запроса, STRETCH-BUDGET-POLICY): целочисленное
    # свойство с границами панели (+11), импорт политики запроса списком (+6), допуск в вызове построения (+3), помощник
    # `_live_session` и фабрика `_request_policy_update` на обе ручки запроса (две копии поиска сессии по девять строк
    # заменены одним помощником — −3 нетто). Ради потолка ничего не склеено. Число поднято осознанно, до фактического.
    # 2031 -> 2015. −16: калбэк ползунка alpha стал заказом фонового превью (`envelope_alpha_preview_gp`), а прежний
    # синхронный счёт и перерисовка ушли вместе с `update_queue_alpha`. Число опущено до фактического.
    "cftuv/operators.py": 2015,
    # 2000 -> 2004. +4: проверка нормали плоскости сменила предмет. Прежде
    # валидатор требовал побитового равенства `A × B`, то есть закреплял
    # КОНКРЕТНЫЙ вывод нормали, а не свойство плоскости, и любой другой (лучше
    # обусловленный) вывод объявлялся дефектом. Теперь проверяется
    # коллинеарность объявленной нормали с `A × B`; это на четыре строки
    # длиннее — нужны обе величины и защита от вырожденного базиса. Модуль
    # стоял РОВНО на общем потолке, поэтому запись появилась, а не выросла.
    # 2004 -> 1772. −232: ветка near-planar в проверку метрики не помещалась, а
    # потолок не поднимается. Проверки рациональной аффинной метрики уехали в
    # `validation_metric.py`, общий словарь отказов (код, запись, сборка) — в
    # `validation_issues.py`; оба соседа укладываются в строгий лимит нового
    # кода без записи в этой таблице. Число опущено до фактического: храповик
    # затягивается там, где освободилось место, иначе освобождённое место
    # молча превращается в разрешение расти обратно.
    # 1772 -> 1731. −41: сверка реконструкции карты с источником и пересчёт
    # сертификата искажения ширины (NEAR_PLANAR V2) вынесены из
    # `validate_analysis_snapshot` в `validation_metric.validate_metric_against_source`.
    # Число опущено до фактического: храповик затягивается там, где
    # освободилось место.
    # 1731 -> 1729. −2 при ДОБАВЛЕННОЙ проверке записей обработки угла плана: сама проверка живёт в
    # `validation_corner_treatment.py` (два вызова здесь), а место под них оплачено сведением
    # шестистрочной ссылки восстановлений канонического угла в одну строку. Число опущено до фактического.
    # 1729 -> 1726. -3 при слиянии ZERO-LENGTH-EDGE (+1 строка ссылки `validation_source_edges`) и JOIN-FLOW:
    # место оплачено сведением пятистрочной ссылки `validation_metric` в одну строку. Число опущено до фактического.
    # 1726 -> 1750. +24 за допуск растяжения развёртки ЗАПРОСА (STRETCH-BUDGET-POLICY): параметр `developable_stretch_budget`
    # проверки снапшота и его проводка в сверку метрики, проверка границ допуска в `validate_decal_request` и связь снапшота с
    # запросом в двух местах (`validate_snapshot_request_references`, `validate_cross_contract_references`). Строки оформлены
    # обычно, а не склеены под потолок. Число поднято осознанно, до фактического.
    # 1750 -> 1753. +3 за полосовую карту (ПОЛОСА-C1): проверка политики полосы против запроса в двух местах
    # (`validate_snapshot_request_references`, `validate_cross_contract_references`), проверка законности `chart_reach_cap` в
    # `validate_decal_request`, сверка покрытия вершин метрикой (полоса покрывает носитель, а не весь патч) и ссылка на помощников
    # `validation_band` в импортах. Сами проверки живут в `validation_band.py` и `validation_metric.metric_covers_patch`.
    # Число поднято осознанно, до фактического.
    # 1753 -> 1750. −3 при ДОБАВЛЕННЫХ проверках плана станций цепей и граней соседа шва: сами проверки живут в
    # `validation_chain_station.py` (два вызова и одна строка импорта здесь), а место оплачено сведением семистрочного импорта
    # `validation_issues` в одну строку. Число опущено до фактического: храповик затягивается там, где освободилось место.
    "kernel/src/cftuv_envelope/validation.py": 1750,
    # 1987 -> 2010. +23 за выбор бэкенда ядра (KERNEL-BACKEND): бэкенд едет в прогон, задачу пула и ключи кэшей результата
    # (`_result_key`, `_slot`, `_binding`, ключ записи сборки, ключ содержимого), запись бэкенда лежит в результате домена, а
    # строка журнала и сама настройка живут в `envelope_kernel_backend.py` (здесь — декоратор `produce_domain`, пять мест проводки
    # и два поля прогона). Файл стоял на 13 строках от общего потолка; вынести кусок, не трогая десяток имён, которые
    # импортируют тесты и инструменты, нечем. Число поднято осознанно, до фактического.
    # 2010 -> 1910: статус, консоль, квитанция и JSON-свидетельство вынесены в `envelope_production_report.py` (место под подготовку под блоком бэкенда).
    "cftuv/envelope_production_export.py": 1910,
}


# Файлы, в которых самая длинная функция превышает NEW_FUNCTION_LINE_LIMIT
# на момент заморозки.
FUNCTION_LINE_ALLOWANCE = {
    "cftuv/analysis_derived.py": 651,
    "kernel/src/cftuv_envelope/debug_scene.py": 595,
    "tools/validate_envelope_ec0.py": 594,
    "kernel/src/cftuv_envelope/reference/compile.py": 538,
    "cftuv/envelope_request_export.py": 509,
    # 480 -> 487. +7 за развязку эталона закона сохранения от `RawCoverage`:
    # параметр `conservation` со значением по умолчанию, его строка разрешения
    # и четыре строки докстроки о том, что от эталона требуется. Логика
    # инварианта не тронута — `_loop_set_signature` была общей на обе подписи
    # ещё до среза, к `RawCoverage` был привязан только аргумент. Долг признан:
    # `apply_policy_b` подлежит разбиению на этапы конвейера (сбор вкладов,
    # крой, доказательство), и семь строк этого не отменяют.
    "kernel/src/cftuv_envelope/interactions/policy_b.py": 487,
    # 426 -> 395: блок сверки метрики с источником ушёл в `validation_metric`.
    # 395 -> 401. +6 в `validate_analysis_snapshot`: параметр допуска запроса в сигнатуре (+4) и в сверке метрики с
    # источником (+2), без склейки строк.
    "kernel/src/cftuv_envelope/validation.py": 401,
    # +5: снятие дельты счётчика локализации точки и два поля в union. Плата за
    # то, чтобы следующий полевой прогон отвечал на вопрос, а не ставил его
    # заново; `exact_union` всё равно подлежит разбиению на этапы конвейера.
    "kernel/src/cftuv_envelope/reference/arrangement.py": 413,
    "kernel/src/cftuv_envelope/interactions/mutual_arrival.py": 385,
    "cftuv/frontier_rescue.py": 353,
    "kernel/src/cftuv_envelope/interactions/arrival.py": 344,
    "cftuv/analysis_validation.py": 336,
    # −28: словарь счётчиков arrangement вынесен в `_union_counters`.
    "kernel/src/cftuv_envelope/reference/raw_coverage.py": 291,
    "cftuv/solve_transfer.py": 317,
    "cftuv/solve_reporting.py": 293,
    "cftuv/solve_report_anomalies.py": 250,
    # 228 -> 208: панель Envelope Debug вынесена из `draw` в
    # `_draw_envelope_debug_box`, поэтому самой длинной стала `execute`.
    "cftuv/operators.py": 208,
    "cftuv/structural_tokens.py": 201,
    "cftuv/frontier_eval.py": 201,
    "cftuv/solve_frontier.py": 197,
    "kernel/src/cftuv_envelope/reference/boundary.py": 193,
    "kernel/src/cftuv_envelope/planar_metric.py": 190,
    # +1: печать диагностик ядра, а не только отказов.
    # 180 -> 174: сборка sidecar и запись свойств GP-объекта вынесены из
    # `render_staged_envelope_debug` отдельными функциями, и свойства объекта
    # теперь берутся из того же payload, что и sidecar, а не из второго
    # источника тех же полей.
    # 174 -> 154: отрисовка точных сцен доменов и сборка их идентичностей ушли
    # в `_accumulate_exact_scenes`. Функция стояла РОВНО на потолке, и слой
    # отказа домена было некуда вызвать.
    # 154 -> 145: строка `writer.commit()` массовой записи GP не помещалась в
    # `render_staged_envelope_debug`, стоявшую РОВНО на потолке. Счётчики GP и
    # запись sidecar/профиля ушли в `_record_gp_render` и `_write_debug_texts`;
    # самой длинной стала `_render_exact_scene`, потолок затянут до неё.
    "cftuv/envelope_debug_renderer.py": 145,
    "kernel/src/cftuv_envelope/interactions/validation.py": 167,
    "cftuv/analysis_boundary_loops.py": 163,
    "tools/benchmark_envelope_metric_models.py": 161,
    "cftuv/frontier_score.py": 161,
    "tools/export_envelope_metric_patch.py": 156,
    "tools/export_building_002_point_contact_fixture.py": 154,
    "cftuv/analysis_surface.py": 146,
    "kernel/src/cftuv_envelope/reference/angular.py": 145,
    "cftuv/frontier_finalize.py": 142,
    # 136 -> 116: пролог scope (топология и адресация доменов) вынесен в
    # `_scope_inputs`, чтобы стадия QUEUE встала рядом с RAW, а не вместо
    # чьей-нибудь строки.
    "tools/run_envelope_mr1_building_gate.py": 116,
    # 135 -> 133: запись треугольника заливки в `create_visualization`
    # (`_new_gp_stroke`, материал, стиль, цикл по точкам) стала одной строкой
    # `batch.add`.
    "cftuv/debug.py": 133,
    "kernel/src/cftuv_envelope/reference/strip.py": 134,
    "cftuv/solve_report_metrics.py": 133,
    "kernel/src/cftuv_envelope/reference/validation.py": 129,
    "cftuv/solve_planning.py": 128,
    "kernel/src/cftuv_envelope/interactions/resolved_coverage.py": 122,
    "cftuv/band_spine.py": 122,
    "cftuv/frontier_place.py": 121,
}


def _budgeted_files() -> tuple[Path, ...]:
    return (
        _python_files(HOST_PACKAGE)
        + _python_files(KERNEL_SOURCE)
        + _python_files(TOOLS)
    )


@pytest.mark.parametrize(
    "path", _budgeted_files(), ids=lambda path: _relative(path)
)
def test_module_stays_within_line_budget(path: Path):
    """Ни один модуль не растёт сверх своего замороженного размера.

    `decal_voronoi.py` дорос до 16 889 строк не по чьему-то решению, а потому
    что росту ничто не сопротивлялось.
    """

    name = _relative(path)
    budget = MODULE_LINE_ALLOWANCE.get(name, NEW_MODULE_LINE_LIMIT)
    actual = _line_count(path)
    assert actual <= budget, (
        f"{name}: {actual} строк при бюджете {budget}. "
        "Разделите модуль. Поднятие числа в MODULE_LINE_ALLOWANCE — "
        "осознанное наращивание долга, а не способ починить тест."
    )


@pytest.mark.parametrize(
    "path", _budgeted_files(), ids=lambda path: _relative(path)
)
def test_functions_stay_within_line_budget(path: Path):
    """Ни одна функция не растёт сверх замороженного предела для своего файла.

    Функцию на 1815 строк с 322 ветвлениями (`_m1_surface_arrangement`)
    невозможно сверить с контрактом — а весь проект держится на контрактах.
    """

    name = _relative(path)
    budget = FUNCTION_LINE_ALLOWANCE.get(name, NEW_FUNCTION_LINE_LIMIT)
    actual, function_name = _max_function_lines(path)
    assert actual <= budget, (
        f"{name}: функция `{function_name}` занимает {actual} строк "
        f"при бюджете {budget}. Разбейте её на этапы конвейера."
    )


# --------------------------------------------------------------------------
# 4. Бюджет обязательного чтения
# --------------------------------------------------------------------------


MANDATORY_READING_LINE_LIMIT = 150


def test_mandatory_reading_stays_small():
    """`AGENTS.md` — единственный обязательный к чтению документ, и он короткий.

    До введения бюджета обязательное чтение занимало 2705 строк в шести файлах.
    Агент сжигал на него половину контекста и всё равно не знал, что
    `band_operator.py` не должен существовать, — потому что правило было прозой.

    Всё, что длиннее этого предела, должно быть тестом, именованным исходом
    или типом, а не текстом.
    """

    actual = _line_count(REPO_ROOT / "AGENTS.md")
    assert actual <= MANDATORY_READING_LINE_LIMIT, (
        f"AGENTS.md: {actual} строк при бюджете {MANDATORY_READING_LINE_LIMIT}. "
        "Правило вида «нельзя X» переносится в этот файл как проверка; "
        "объяснение того, как работает код, удаляется — код скажет лучше."
    )


# --------------------------------------------------------------------------
# 5. Гигиена репозитория
# --------------------------------------------------------------------------


LARGE_FILE_BYTES = 512 * 1024

# Крупные файлы, попавшие в git до введения правила. История их уже содержит,
# переписывание истории — отдельное осознанное действие. Новые крупные файлы
# должны идти через LFS (см. .gitattributes).
KNOWN_LARGE_FILE_COUNT = 6


#: Каталог сборки нативного ускорителя (`cargo`): гигабайт объектных файлов, в `.gitignore`, частью репозитория не
#: является — ровно как рабочие каталоги субагентов `.claude`. Исключён по ПУТИ, а не по имени `target`: каталог с таким
#: именем в другом месте дерева остаётся под правилом.
NATIVE_BUILD_OUTPUT = ("native", "target")


def _is_native_build_output(path: Path) -> bool:
    return path.relative_to(REPO_ROOT).parts[: len(NATIVE_BUILD_OUTPUT)] == NATIVE_BUILD_OUTPUT


def _large_files_in_worktree() -> tuple[str, ...]:
    """Крупные файлы в рабочем дереве, кроме заведомо исключённых каталогов."""

    # `.claude` — рабочие каталоги субагентов: git-worktree с полной копией
    # репозитория на время запуска. Без этого исключения тест считал крупные
    # файлы по разу на каждого работающего агента и падал не от роста дерева,
    # а от того, что кто-то в этот момент работал. Каталог в `.gitignore`,
    # то есть частью репозитория не является — считать его нечего.
    skipped_roots = {".git", "__pycache__", ".claude"}
    large: list[str] = []
    for path in REPO_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if skipped_roots & set(path.relative_to(REPO_ROOT).parts):
            continue
        if _is_native_build_output(path):
            continue
        if path.stat().st_size > LARGE_FILE_BYTES:
            large.append(_relative(path))
    return tuple(sorted(large))


def test_large_binary_count_does_not_grow():
    """Число крупных файлов в дереве не растёт.

    69 PNG-скриншотов и 2 .blend-файла (~76 МБ) удалены из рабочего дерева:
    владелец подтвердил, что они устарели и недостаточного качества. В истории
    git они остаются — правило не откатывает прошлое, оно останавливает рост.
    Осталось 6 крупных файлов: JSON-корпуса и результаты замеров, они текстовые
    и служат спецификацией поведения.
    """

    large = _large_files_in_worktree()
    assert len(large) <= KNOWN_LARGE_FILE_COUNT, (
        f"Крупных файлов стало {len(large)} при разрешённых "
        f"{KNOWN_LARGE_FILE_COUNT}. Новые: см. список выше. "
        "Бинарные свидетельства складывайте под LFS или во внешнее хранилище "
        "со ссылкой из handoff-записи.\n"
        + "\n".join(f"  {name}" for name in large[:80])
    )


# Столько документов в дереве после уборки `docs/agent_execution/envelope_v1/`
# и восьми документов legacy decal-конвейеров (Фаза 4).
# Число не круглое намеренно: круглое приглашает «ну ещё один до сотни».
KNOWN_MARKDOWN_COUNT = 62


def _markdown_in_worktree() -> tuple[str, ...]:
    # Тот же список исключений, что у крупных файлов, и по той же причине:
    # `.claude` — рабочие каталоги субагентов, полные копии репозитория на
    # время запуска. Плюс `.pytest_cache`, где лежит собственный README pytest:
    # он не наш документ и в бюджет попадать не должен.
    skipped_roots = {".git", "__pycache__", ".claude", ".pytest_cache"}
    found: list[str] = []
    for path in REPO_ROOT.rglob("*.md"):
        if not path.is_file():
            continue
        if skipped_roots & set(path.relative_to(REPO_ROOT).parts):
            continue
        if _is_native_build_output(path):
            continue
        found.append(_relative(path))
    return tuple(sorted(found))


def test_markdown_document_count_does_not_grow():
    """Число документов в дереве не растёт. Это храповик, а не запрет.

    История, ради которой правило существует. Обязательное чтение однажды
    разрослось до 2705 строк в шести файлах; его срезали до `AGENTS.md` в 150
    строк и закрыли бюджетом (`test_mandatory_reading_stays_short`). Но сорняк
    вырос сбоку: к 2026-07-27 в дереве лежало 148 документов на ~34 000 строк, и
    среди них — `docs/agent_execution/envelope_v1/` из 47 файлов: мастер-план,
    протокол агента, 33 карточки, `HANDOFF_TEMPLATE.md` и
    `SESSION_BOOTSTRAP_TEMPLATE.md`. Всю эту систему отменила Фаза 3 роадмапа
    словами «Один чеклист вместо карт и handoff-документов» — но отменённое
    лежало рядом с отменяющим и ничем не было помечено, а шаблоны handoff прямо
    приглашали проблему вернуться.

    Бюджет на обязательное чтение этого не поймал: он меряет ОДИН файл, а
    разрастание пошло по другим. Здесь меряется дерево целиком.

    Почему храповик, а не потолок «сколько не жалко». Новый документ завести
    можно — но только удалив другой или изменив это число, то есть в диффе,
    который владелец увидит. Молча документы больше не заводятся. Правило не
    откатывает прошлое: 100 оставшихся документов оно не трогает, оно
    останавливает рост.
    """

    documents = _markdown_in_worktree()
    assert len(documents) <= KNOWN_MARKDOWN_COUNT, (
        f"Документов стало {len(documents)} при разрешённых "
        f"{KNOWN_MARKDOWN_COUNT}.\n"
        "Прежде чем поднимать число: `AGENTS.md` требует вместо документа "
        "писать тест, а вместо запрета — падающую проверку. Прозой пишется "
        "только строка в `DECISIONS.md`.\n"
        + "\n".join(f"  {name}" for name in documents)
    )


# Конструкции, которых нет в Windows PowerShell 5.1. Ключ в том, что каждая из
# них — ошибка РАЗБОРА, а не выполнения: скрипт падает целиком, не выполнив ни
# строки, поэтому никакая проверка внутри скрипта от них не спасает.
POWERSHELL_7_ONLY = (
    ("??", "оператор ?? (null-coalescing) появился в PowerShell 7"),
    ("?.", "оператор ?. (null-conditional) появился в PowerShell 7"),
    ("-Parallel", "ForEach-Object -Parallel появился в PowerShell 7"),
    ("-AsHashtable", "ConvertFrom-Json -AsHashtable появился в PowerShell 6"),
    ("Join-String", "командлет Join-String появился в PowerShell 6"),
    ("$PSStyle", "$PSStyle появился в PowerShell 7.2"),
)


def test_windows_scripts_stay_within_powershell_5_1():
    """Скрипты установки обязаны разбираться Windows PowerShell 5.1.

    Оплачено полем. `install_to_blender.ps1` содержал `??`, и у владельца
    установка не запускалась ВООБЩЕ: в 5.1 это ошибка разбора, а не выполнения,
    поэтому скрипт падал целиком, не выполнив ни строки и не напечатав ни одной
    своей диагностики. Владелец увидел `Unexpected token '??'` и всё.

    Почему правилом, а не аккуратностью. PowerShell 7 ставится отдельно, а в
    Windows штатно стоит 5.1 — значит писать надо под 5.1, и проверять это
    обязан не человек, а тест: в контейнере PowerShell'а нет, разобрать скрипт
    здесь нечем, и единственная защита от повторения — запрет на конструкции,
    которых в 5.1 не существует.

    Список намеренно узкий: только то, что ломает РАЗБОР. Расширять его до
    полноты не нужно — он должен ловить класс, который уже случился.
    """

    offenders: list[str] = []
    for path in sorted((REPO_ROOT / "tools").glob("*.ps1")):
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), start=1):
            code = line.split("#", 1)[0]
            for token, reason in POWERSHELL_7_ONLY:
                if token in code:
                    offenders.append(
                        f"{_relative(path)}:{number}: {reason}\n    {line.strip()}"
                    )
    assert not offenders, (
        "В скриптах установки есть синтаксис, которого нет в Windows "
        "PowerShell 5.1. Это ошибка РАЗБОРА: скрипт не выполнит ни строки.\n"
        + "\n".join(offenders)
    )


def test_binary_evidence_is_declared_binary():
    """`.gitattributes` помечает типы бинарных свидетельств как binary.

    Это не переносит их в LFS — миграция переписывает историю и остаётся
    осознанным действием владельца. Пометка лишь не даёт git считать PNG и
    .blend текстом и нормализовать в них переводы строк.
    """

    attributes = REPO_ROOT / ".gitattributes"
    assert attributes.exists(), (
        ".gitattributes отсутствует — git будет считать новые PNG/blend текстом."
    )
    declared = attributes.read_text(encoding="utf-8")
    for suffix in ("*.png", "*.jpg", "*.blend"):
        assert f"{suffix}" in declared, (
            f"{suffix} не объявлен в .gitattributes."
        )


# --------------------------------------------------------------------------
# Точная работа мимо бюджета: хостовая половина запрета
# --------------------------------------------------------------------------

# Правило — общее с ядром и живёт ОДНИМ модулем
# (`kernel/tests/exact_work_budget_ban.py`). Здесь оно только применяется к
# `cftuv/`: адаптер хоста тоже зовёт точную арифметику ядра, и ровно там
# нашлись два последних места из дыры BUDGET-COVERAGE-STRUCTURAL — усечение
# контуров для картинки (`coverage_at` без счёта, 281 вызов `sign` на трёх
# полевых доменах) и проверка простоты слитого контура (`contour_crossings`,
# 36 вызовов). Без этой проверки хост может открыть дыру заново, а ядро об
# этом не узнает: его сюита `cftuv/` не читает.
#
# Разбираются ТОЛЬКО модули хоста, действительно трогающие ядро. Причина
# измерена, а не гигиеническая: правило ловит вызов по ИМЕНИ, а в словаре
# хоста `normalized` — это `mathutils.Vector.normalized`, и на всём пакете имя
# совпадает шестьдесят раз при трёх настоящих кандидатах. Список исключений из
# шестидесяти строк шума не проверял бы ничего.
_HOST_EXACT_WORK_EXEMPTIONS: dict[tuple[str, str], tuple[int, str]] = {
    # `mathutils.Vector.normalized()` в отрисовке отладки — совпадение имени с
    # `EventTimeV1.normalized`, к точной арифметике отношения не имеет.
    ("cftuv/envelope_debug_renderer.py", "normalized"): (
        3,
        "RECEIVER_IS_NOT_A_SQRT_SUM",
    ),
}


@cache
def _exact_work_budget_ban():
    """Общее правило запрета, загруженное из `kernel/tests` по пути.

    Именно по пути, а не импортом пакета: хостовая сюита не имеет права
    зависеть от того, лежит ли `kernel/tests` на `sys.path`, а копия правила
    рядом разошлась бы с оригиналом.
    """

    import importlib.util

    path = REPO_ROOT / "kernel" / "tests" / "exact_work_budget_ban.py"
    spec = importlib.util.spec_from_file_location(
        "_exact_work_budget_ban", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_host_adapter_never_calls_exact_arithmetic_without_a_budget():
    """Хост, зовущий ядро, обязан называть бюджет — иначе счёт без исхода.

    Нарушение здесь СТРОИТСЯ ровно так же, как в ядре: проверка правила на
    заведомо нарушающем фрагменте живёт в
    `kernel/tests/test_exact_work_budget_coverage.py`, и это тот же самый код.
    """

    ban = _exact_work_budget_ban()
    grouped: dict[tuple[str, str], list[int]] = {}
    for path in _python_files(HOST_PACKAGE):
        source = _source_text(path)
        if "cftuv_envelope" not in source:
            continue
        module = _relative(path)
        for _, name, lineno in ban.unbudgeted_sites_in_source(source, module):
            grouped.setdefault((module, name), []).append(lineno)

    offenders = sorted(
        f"{module}:{sorted(lines)} -> {name}"
        for (module, name), lines in grouped.items()
        if len(lines) > _HOST_EXACT_WORK_EXEMPTIONS.get((module, name), (0,))[0]
    )
    assert not offenders, (
        "адаптер хоста зовёт точную арифметику ядра без названного бюджета:\n"
        + "\n".join(offenders)
        + "\n\nНазовите бюджет в вызове (у экспорта он свой, stage=EXPORT) "
        "либо заведите место в _HOST_EXACT_WORK_EXEMPTIONS с причиной из "
        "kernel/tests/exact_work_budget_ban.EXEMPTION_REASONS."
    )


def test_host_exact_work_exemptions_name_a_registered_reason():
    """Исключение хоста ссылается на общий словарь причин, а не на свои слова."""

    reasons = _exact_work_budget_ban().EXEMPTION_REASONS
    for key, (_, reason) in _HOST_EXACT_WORK_EXEMPTIONS.items():
        assert reason in reasons, (key, reason)


# --------------------------------------------------------------------------
# 10. Хранилище по содержимому процессно-локально
# --------------------------------------------------------------------------

_STORE_NAMES = frozenset({"content_store", "_content_store", "ContentStoreV1"})
_SERIALISING_CALLS = frozenset(
    {
        ("pickle", "dump"),
        ("pickle", "dumps"),
        ("json", "dump"),
        ("json", "dumps"),
        ("marshal", "dump"),
        ("marshal", "dumps"),
        ("shelve", "open"),
    }
)
#: Модули, которых у хранилища быть не должно: у него нет ни диска, ни базы.
_STORE_FORBIDDEN_IMPORTS = frozenset(
    {"os", "pathlib", "shelve", "marshal", "sqlite3", "dbm", "tempfile", "shutil", "json"}
)


def _mentions_store(node: ast.AST) -> bool:
    for child in ast.walk(node):
        if isinstance(child, ast.Name) and child.id in _STORE_NAMES:
            return True
        if isinstance(child, ast.Attribute) and child.attr in _STORE_NAMES:
            return True
    return False


def test_the_content_store_is_never_serialised_to_disk():
    """`ContentStoreV1` держит живые объекты подготовок и результатов ЭТОГО процесса и на диск не пишется.

    Запись, пережившая смену кода, была бы устаревшим результатом (ключ несёт отпечаток кода, но замок —
    отсутствие диска). Рантайм-замок: `ContentStoreV1.__reduce_ex__` отказывает в сериализации
    (тест в `tests/test_envelope_content_store.py`); здесь — статический: хост не отдаёт
    хранилище в pickle/json/marshal/shelve, а модуль хранилища не импортирует ни файловых, ни БД-модулей и
    не зовёт `open`/`pickle.dump`/`pickle.load` (его `pickle` работает только с `BytesIO`).
    """

    offenders = []
    for path in _python_files(HOST_PACKAGE):
        source = _source_text(path)
        if "content_store" not in source and "ContentStoreV1" not in source:
            continue
        for node in ast.walk(_parse(path)):
            if not (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and (node.func.value.id, node.func.attr) in _SERIALISING_CALLS
            ):
                continue
            arguments = [*node.args, *(item.value for item in node.keywords)]
            if any(_mentions_store(argument) for argument in arguments):
                offenders.append(f"{_relative(path)}:{node.lineno} {node.func.value.id}.{node.func.attr}")
    assert not offenders, "хранилище по содержимому отдано сериализатору:\n" + "\n".join(offenders)

    store_module = HOST_PACKAGE / "envelope_content_store.py"
    imported = _imported_roots(store_module)
    assert not imported & _STORE_FORBIDDEN_IMPORTS, sorted(imported & _STORE_FORBIDDEN_IMPORTS)
    for node in ast.walk(_parse(store_module)):
        if isinstance(node, ast.Call):
            function = node.func
            name = function.id if isinstance(function, ast.Name) else getattr(function, "attr", "")
            assert name not in {"open", "write_bytes", "write_text"}, (name, node.lineno)
            if isinstance(function, ast.Attribute) and isinstance(function.value, ast.Name):
                assert (function.value.id, function.attr) not in {("pickle", "dump"), ("pickle", "load")}, node.lineno


# --------------------------------------------------------------------------
# Порядок множеств батча в хосте
# --------------------------------------------------------------------------
#
# `GeometryBatchV1.vertices / station_facts / semantic_regions / boundary_chains / interface_chains / diagnostics` — `frozenset`,
# и порядок их обхода ходит с `PYTHONHASHSEED`. Хост, который идёт по ним «как лежат», выдаёт квитанцию, зависящую от зерна
# процесса: на `walls.003` предупреждение `ADAPTER_SEAM_T_JUNCTIONS` появлялось у трёх зёрен из шести при побитово равном
# меше (первая цепь вершины зависела от порядка цепей). Обход идёт через `sorted(...)`: порядок по ключу, а не по хешу.
# Ловится прямой обход (`for`, включение) поля батча; множество, сначала сложенное в переменную, правило не видит —
# его ловит подпроцессный тест на разных зёрнах (`test_envelope_production_weld.py`).

_BATCH_FROZENSET_FIELDS = frozenset(
    {
        "vertices",
        "station_facts",
        "semantic_regions",
        "boundary_chains",
        "interface_chains",
        "diagnostics",
        "contract_versions",
    }
)


def _batch_set_field(node: ast.AST) -> str | None:
    """Имя поля-множества батча, если выражение его называет: `<...>batch.<поле>` либо `getattr(<...>, "<поле>", ...)`."""

    if isinstance(node, ast.Attribute) and node.attr in _BATCH_FROZENSET_FIELDS:
        owner = node.value
        owner_name = owner.id if isinstance(owner, ast.Name) else getattr(owner, "attr", "")
        return node.attr if owner_name == "batch" else None
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "getattr"
        and len(node.args) >= 2
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value in _BATCH_FROZENSET_FIELDS
    ):
        return node.args[1].value
    return None


def _unsorted_batch_set_walks(tree: ast.AST) -> list[tuple[int, str]]:
    """`(строка, поле)` обходов поля-множества батча мимо `sorted(...)`: в `for` и в источниках включений."""

    found: list[tuple[int, str]] = []

    def visit(node: ast.AST) -> None:
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "sorted":
            return
        field = _batch_set_field(node)
        if field is not None:
            found.append((node.lineno, field))
            return
        for child in ast.iter_child_nodes(node):
            visit(child)

    for node in ast.walk(tree):
        if isinstance(node, (ast.For, ast.AsyncFor, ast.comprehension)):
            visit(node.iter)
    return sorted(found)


def test_the_batch_set_walk_rule_flags_an_unsorted_walk_and_passes_a_sorted_one():
    unsorted = (
        "for chain in batch.boundary_chains:\n"
        "    pass\n"
        "names = [k for k in getattr(batch, 'interface_chains', ()) or ()]\n"
        "pairs = {v for v in result.batch.vertices}\n"
    )
    ordered = (
        "for chain in sorted(batch.boundary_chains, key=len):\n"
        "    pass\n"
        "count = len(batch.vertices)\n"
    )

    assert _unsorted_batch_set_walks(ast.parse(unsorted)) == [
        (1, "boundary_chains"),
        (3, "interface_chains"),
        (4, "vertices"),
    ]
    assert _unsorted_batch_set_walks(ast.parse(ordered)) == []


def test_the_host_never_walks_a_batch_frozenset_in_hash_order():
    offenders = [
        f"{_relative(path)}:{line} .{field}"
        for path in _python_files(HOST_PACKAGE)
        for line, field in _unsorted_batch_set_walks(_parse(path))
    ]
    assert not offenders, (
        "хост идёт по `frozenset` батча в хеш-порядке (ответ зависит от PYTHONHASHSEED):\n"
        + "\n".join(offenders)
        + "\n\nОбходите через sorted(..., key=<имя или ключ вершин>)."
    )


# --------------------------------------------------------------------------
# Нативный ускоритель: одна точка входа
# --------------------------------------------------------------------------
#
# Расширение `cftuv_native._core` (Rust, `native/`) импортирует ТОЛЬКО шим `cftuv_native/__init__.py`: он переводит
# объекты ядра в буферы целой операции и воспроизводит её побочные эффекты (бюджет, память канонизации). Второй
# импортёр расширения обошёл бы этот перевод — и вместе с ним сверку с Python-эталоном.

NATIVE_EXTENSION = "cftuv_native._core"
NATIVE_SHIM = "native/cftuv-python/python/cftuv_native/__init__.py"


def _native_extension_imports(tree: ast.AST, inside_package: bool) -> list[int]:
    """Строки, где модуль добирается до расширения: импортом в любой форме либо строкой с его именем."""

    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(alias.name == NATIVE_EXTENSION or alias.name.startswith(NATIVE_EXTENSION + ".") for alias in node.names):
                found.append(node.lineno)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            names = {alias.name for alias in node.names}
            if node.level == 0 and (
                module == NATIVE_EXTENSION
                or module.startswith(NATIVE_EXTENSION + ".")
                or (module == "cftuv_native" and "_core" in names)
            ):
                found.append(node.lineno)
            elif node.level > 0 and inside_package and (module.split(".")[0] == "_core" or (not module and "_core" in names)):
                found.append(node.lineno)
        elif isinstance(node, ast.Constant) and isinstance(node.value, str) and NATIVE_EXTENSION in node.value:
            found.append(node.lineno)
    return sorted(found)


def _repository_python_files() -> tuple[Path, ...]:
    skipped_roots = {".git", "__pycache__", ".claude"}
    return tuple(
        path
        for path in sorted(REPO_ROOT.rglob("*.py"))
        if not (skipped_roots & set(path.relative_to(REPO_ROOT).parts)) and not _is_native_build_output(path)
    )


def test_the_native_extension_rule_flags_every_import_form():
    planted = (
        "import cftuv_native._core\n"
        "from cftuv_native import _core\n"
        "from cftuv_native._core import Session\n"
        "import importlib\n"
        "module = importlib.import_module('cftuv_native._core')\n"
        "import cftuv_native\n"
    )
    assert _native_extension_imports(ast.parse(planted), inside_package=False) == [1, 2, 3, 5]
    relative = "from . import _core\nfrom ._core import Session\nfrom . import codec\n"
    assert _native_extension_imports(ast.parse(relative), inside_package=True) == [1, 2]


def test_only_the_shim_imports_the_native_extension():
    offenders = []
    for path in _repository_python_files():
        name = _relative(path)
        # Шим — единственный импортёр; этот файл называет расширение строкой, потому что держит само правило.
        if name in (NATIVE_SHIM, _relative(Path(__file__).resolve())):
            continue
        inside = name.startswith("native/cftuv-python/python/cftuv_native/")
        lines = _native_extension_imports(_parse(path), inside_package=inside)
        if lines:
            offenders.append(f"{name}:{lines}")
    assert not offenders, (
        f"расширение {NATIVE_EXTENSION} импортируется мимо шима {NATIVE_SHIM}:\n"
        + "\n".join(offenders)
        + "\n\nЗовите `cftuv_native` (шим): он переводит вход и воспроизводит бюджет и память канонизации."
    )


# --------------------------------------------------------------------------
# Нативный ускоритель: пин эталона
# --------------------------------------------------------------------------
#
# Нативная операция побитово равна ОДНОЙ версии ядра на Python. `native/cftuv-python/python/cftuv_native/pin.py` держит sha256 тех файлов эталона, которые
# порт зеркалит; шим отказывается названным `NativePortStale`, если дерево ушло от пина (`tests/test_native_pin.py` проверяет сам механизм). Здесь — то, что
# проверяется без расширения и в чистом клоне: у каждого файла списков ровно один дайджест. Исчезновение или переименование зеркалимого файла ядро
# НЕ краснит: Python-сессия двигает ядро свободно, а шим называет порт устаревшим (`NativePortStale`, файл назван); догон порта — отдельная работа.

NATIVE_PIN = "native/cftuv-python/python/cftuv_native/pin.py"


def _load_native_pin():
    import importlib.util

    spec = importlib.util.spec_from_file_location("_cftuv_native_pin_under_test", REPO_ROOT / NATIVE_PIN)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_native_pins_hold_one_digest_per_mirrored_file():
    pin = _load_native_pin()
    listed = {name for files in pin.OPERATION_FILES.values() for name in files}
    assert set(pin.OPERATION_FILES) == {"coverage", "clip", "skeleton", "snap_embedding"}
    assert set(pin.PINS) == listed, "у каждого файла списка ровно один пин и ни одного лишнего"
    assert all(len(digest) == 64 and set(digest) <= set("0123456789abcdef") for digest in pin.PINS.values())
    assert all(len(set(files)) == len(files) for files in pin.OPERATION_FILES.values())


# --------------------------------------------------------------------------
# 11. Цена вычисления домена - функция его входа: бюджет подготовки не меняется покрытием
# --------------------------------------------------------------------------
# PRICE-WITHOUT-HISTORY (DECISIONS 2026-10-06). Покрытие каждого вычисления считает на КОПИИ состояния подготовки
# (`ExactWorkBudgetV1.forked`), а `at_stage` меняет сам бюджет: до правила покрытие звало `at_stage("COVERAGE")` на бюджете подготовки,
# холодная цена покрытия ездила в пикле, и тёплые шаги копились на ней (1724, 1726, 1728 ...). `at_stage` зовёт ТОЛЬКО начало подготовки.

_AT_STAGE_ALLOWED = {("kernel/src/cftuv_envelope/wavefront/conveyor.py", "_domain_work_budget")}


def _at_stage_calls(tree: ast.AST) -> list[tuple[str, int]]:
    """`(имя функции, строка)` вызовов `<...>.at_stage(...)`; вне функции имя пусто."""

    found: list[tuple[str, int]] = []

    def visit(node: ast.AST, function: str) -> None:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            function = node.name
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "at_stage":
            found.append((function, node.lineno))
        for child in ast.iter_child_nodes(node):
            visit(child, function)

    visit(tree, "")
    return sorted(found)


def test_the_at_stage_rule_flags_a_coverage_that_switches_the_stage_of_the_preparation_budget():
    spoiled = "def conveyor_coverage(prepared):\n    budget = prepared.work_budget\n    budget.at_stage('COVERAGE')\n"
    honest = "def conveyor_coverage(prepared):\n    budget = prepared.work_budget.forked('COVERAGE')\n"

    assert _at_stage_calls(ast.parse(spoiled)) == [("conveyor_coverage", 3)]
    assert _at_stage_calls(ast.parse(honest)) == []


def test_only_the_preparation_switches_the_stage_of_the_domain_budget():
    offenders = [
        f"{_relative(path)}:{line} {function or '<module>'}"
        for path in _python_files(KERNEL_SOURCE)
        for function, line in _at_stage_calls(_parse(path))
        if (_relative(path), function) not in _AT_STAGE_ALLOWED
    ]
    assert not offenders, (
        "стадию бюджета меняет не подготовка (цена вычисления станет свойством истории):\n"
        + "\n".join(offenders)
        + "\n\nПокрытию и материализации - `work_budget.forked(<стадия>)`, а не `at_stage`."
    )


# --------------------------------------------------------------------------
# 12. Нативное ядро видно остальному коду только через `backend.py`
# --------------------------------------------------------------------------
# KERNEL-BACKEND (DECISIONS 2026-10-06). Нативный бэкенд — переключатель, а не вторая реализация, и тихого отката на Python у него
# нет: любое его исключение из перечня названо и записано в журнал домена. Это держится на ОДНОМ месте, где нативное ядро
# загружается, и где его отсутствие превращается в именованный исход. Второй импортёр (ядро, хост, инструмент, тест) мог бы
# вызвать нативную операцию мимо журнала, и домен, посчитанный не тем бэкендом, остался бы без имени.

NATIVE_PACKAGE = "cftuv_native"
NATIVE_IMPORTER = "kernel/src/cftuv_envelope/backend.py"
_DYNAMIC_IMPORTS = frozenset({"import_module", "__import__"})


def _native_imports(tree: ast.AST) -> list[tuple[int, str]]:
    """`(строка, форма)` импортов `cftuv_native`: оператором либо строковым аргументом `import_module`/`__import__`."""

    found: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found += [
                (node.lineno, f"import {alias.name}")
                for alias in node.names
                if alias.name.split(".")[0] == NATIVE_PACKAGE
            ]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            if node.module.split(".")[0] == NATIVE_PACKAGE:
                found.append((node.lineno, f"from {node.module} import ..."))
        elif isinstance(node, ast.Call) and node.args:
            function = node.func
            name = function.id if isinstance(function, ast.Name) else getattr(function, "attr", "")
            first = node.args[0]
            if (
                name in _DYNAMIC_IMPORTS
                and isinstance(first, ast.Constant)
                and isinstance(first.value, str)
                and first.value.split(".")[0] == NATIVE_PACKAGE
            ):
                found.append((node.lineno, f"{name}({first.value!r})"))
    return sorted(found)


def test_the_native_import_rule_flags_every_form_of_import_and_passes_a_clean_module():
    spoiled = (
        "import cftuv_native\n"
        "import cftuv_native.codec as codec\n"
        "from cftuv_native import coverage_at\n"
        "import importlib\n"
        "importlib.import_module('cftuv_native')\n"
        "__import__('cftuv_native.cost')\n"
    )
    honest = (
        "import sys\n"
        "from cftuv_envelope import backend\n"
        "sys.modules['cftuv_native'] = None\n"
        "importlib.import_module('cftuv_envelope.backend')\n"
        "native = 'cftuv_native'\n"
    )

    assert [line for line, _form in _native_imports(ast.parse(spoiled))] == [1, 2, 3, 5, 6]
    assert _native_imports(ast.parse(honest)) == []


#: Собственные инструменты и тесты нативного порта (сессия `native/`): корпус, бенчмарки и дифференциальные тесты зовут `cftuv_native` напрямую,
#: потому что проверяют САМ порт. Продукт (`cftuv/`, `kernel/src/`) и остальные инструменты идут через `backend.py`.
NATIVE_PORT_FAMILY = ("tools/native_", "tests/test_native_")


#: Собственные инструменты и тесты нативного порта (сессия `native/`): корпус, бенчмарки и дифференциальные тесты зовут `cftuv_native` напрямую,
#: потому что проверяют САМ порт. Продукт (`cftuv/`, `kernel/src/`) и остальные инструменты идут через `backend.py`.
NATIVE_PORT_FAMILY = ("tools/native_", "tests/test_native_")


#: Собственные инструменты и тесты нативного порта (сессия `native/`): корпус, бенчмарки и дифференциальные тесты зовут `cftuv_native` напрямую,
#: потому что проверяют САМ порт. Продукт (`cftuv/`, `kernel/src/`) и остальные инструменты идут через `backend.py`.
NATIVE_PORT_FAMILY = ("tools/native_", "tests/test_native_")


def test_only_the_backend_module_imports_the_native_package():
    scanned = (
        _python_files(HOST_PACKAGE)
        + _python_files(KERNEL_SOURCE)
        + _python_files(TOOLS)
        + _python_files(TESTS)
        + _python_files(REPO_ROOT / "kernel" / "tests")
    )
    offenders = [
        f"{_relative(path)}:{line} {form}"
        for path in scanned
        if _relative(path) != NATIVE_IMPORTER and not _relative(path).startswith(NATIVE_PORT_FAMILY)
        for line, form in _native_imports(_parse(path))
    ]
    assert not offenders, (
        "нативное ядро импортируется мимо `cftuv_envelope/backend.py`:\n"
        + "\n".join(offenders)
        + "\n\nИспользуйте `backend.native_status()` и `backend.use_backend(...)`: откат на Python обязан быть назван."
    )
    assert _native_imports(_parse(REPO_ROOT / NATIVE_IMPORTER)), "backend.py перестал быть импортёром cftuv_native"
    # продукт не входит в исключение: оно только для собственных инструментов и тестов порта
    product = _python_files(HOST_PACKAGE) + _python_files(KERNEL_SOURCE)
    assert not [path for path in product if _relative(path).startswith(NATIVE_PORT_FAMILY)]


def _dispatch_hooks(tree: ast.AST) -> bool:
    """Файл обращается к модулю `backend` (импорт `backend` либо имя `backend.<...>`): в нём стоит диспетчер бэкенда."""

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and any(alias.name == "backend" for alias in node.names):
            return True
        if isinstance(node, ast.Name) and node.id == "backend":
            return True
    return False


def test_the_dispatch_hook_detector_flags_a_wired_module_and_passes_a_plain_one():
    wired = "from .. import backend\n\ndef f():\n    return backend.coverage_compute(1)\n"
    plain = "from .sqrt_sum import SqrtSumV1\n\ndef f():\n    return _coverage_at(1)\n"
    assert _dispatch_hooks(ast.parse(wired))
    assert not _dispatch_hooks(ast.parse(plain))


def _call_sites(tree: ast.AST, name: str) -> list[tuple[str, ...]]:
    """Вызовы `name(...)` (по имени либо как атрибут) и цепочка охватывающих функций каждого (внешняя первой)."""

    found: list[tuple[str, ...]] = []

    def visit(node: ast.AST, stack: tuple[str, ...]) -> None:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            stack = (*stack, node.name)
        if isinstance(node, ast.Call):
            func = node.func
            if (isinstance(func, ast.Name) and func.id == name) or (isinstance(func, ast.Attribute) and func.attr == name):
                found.append(stack)
        for child in ast.iter_child_nodes(node):
            visit(child, stack)

    visit(tree, ())
    return found


#: Где хост строит подготовку (`prepare_conveyor`): скелет считается в ней, и блок бэкенда обязан стоять вокруг. Внешняя функция: что вокруг.
#: `prepare_for_production_recorded` и `run_queue_domain` ставят блок сами (`prepared_under_backend`); провайдер подготовки отладочной сессии
#: (`evaluate_staged` -> `preparation_provider`) вызывается ИЗНУТРИ блока `run_queue_domain`, поэтому своего блока не ставит (второй заслонил бы запись первого).
_PREPARATION_SITES = {
    "envelope_production_export.py": {"prepare_for_production_recorded"},
    "envelope_queue_export.py": {"run_queue_domain"},
    "envelope_debug_session.py": {"evaluate_staged"},
}


def test_every_host_preparation_stands_under_the_backend_block():
    """Новый путь, строящий подготовку мимо блока бэкенда, считал бы скелет эталоном при заказе нативного (и записи бы не было): тихий разнобой стадий."""

    sites: dict[str, set[str]] = {}
    for path in _python_files(HOST_PACKAGE):
        for stack in _call_sites(_parse(path), "prepare_conveyor"):
            sites.setdefault(path.name, set()).add(stack[0] if stack else "<module>")
    assert sites == _PREPARATION_SITES, (
        f"подготовка строится в {sites}, а блок бэкенда стоит в {_PREPARATION_SITES}: новому пути нужен `prepared_under_backend` "
        "(`envelope_kernel_backend`) либо он идёт изнутри блока `run_queue_domain`, и тогда его место называется здесь"
    )
    for name in ("envelope_production_export.py", "envelope_queue_export.py"):
        scoped = {stack[0] for stack in _call_sites(_parse(HOST_PACKAGE / name), "prepared_under_backend")}
        assert scoped == _PREPARATION_SITES[name], name
    # холодная задача воркера и родитель строят подготовку одной функцией, блок которой несёт запись скелета;
    # задача воркера открывает журнал в `solve_cold_production_task`, а подготовку зовёт `_solve_cold_production_task`, заимствуя его
    export = _parse(HOST_PACKAGE / "envelope_production_export.py")
    for function in ("_solve_cold_production_task", "_produce_cold_in_parent"):
        assert any(stack[0] == function for stack in _call_sites(export, "prepare_for_production_recorded")), function
    assert any(stack[0] == "solve_cold_production_task" for stack in _call_sites(export, "_solve_cold_production_task")), "solve_cold_production_task"


def test_the_skeleton_is_called_in_the_kernel_only_through_the_dispatcher():
    """`build_skeleton` зовёт только диспетчер бэкенда (эталон берётся при вызове, `backend._python_skeleton`); `_prepare_region` зовёт `backend.skeleton_compute`."""

    callers: dict[str, int] = {}
    for path in _python_files(KERNEL_SOURCE):
        count = len(_call_sites(_parse(path), "build_skeleton"))
        if count:
            callers[_relative(path)] = count
    # единственный вызов по имени — `module.build_skeleton(...)` нативного шима внутри `backend.skeleton_compute`; эталон диспетчер зовёт через `oracle(...)`
    assert callers == {"kernel/src/cftuv_envelope/backend.py": 1}, f"build_skeleton зовут мимо диспетчера: {sorted(callers)}"
    conveyor = KERNEL_SOURCE / "cftuv_envelope" / "wavefront" / "conveyor.py"
    assert [stack[0] for stack in _call_sites(_parse(conveyor), "skeleton_compute")] == ["_prepare_region"]
    assert not any(isinstance(node, ast.ImportFrom) and any(alias.name == "build_skeleton" for alias in node.names) for node in ast.walk(_parse(conveyor)))
    backend_module = KERNEL_SOURCE / "cftuv_envelope" / "backend.py"
    assert "from .wavefront.skeleton import build_skeleton" in _source_text(backend_module)


#: Стадии, переведённые на Rust насовсем (решение владельца 2026-10-07: пересадка ядра по стадиям). Законы такой стадии меняются
#: ТОЛЬКО в Rust; её Python-файлы — замороженный эталон-архив. Правка закреплённого файла такой стадии — это работа Rust-сессии:
#: порт и новое закрепление в одном изменении, иначе эталон тихо разъехался бы с продуктом (продукт по умолчанию считает на Rust).
RUST_ONLY_OPERATIONS = ("coverage", "clip", "skeleton", "snap_embedding")


def test_python_sources_of_rust_only_stages_stay_at_their_pin():
    """Python-файлы стадий из `RUST_ONLY_OPERATIONS` побитово равны закреплению (`cftuv_native/pin.py`); исчезнувший файл — тоже дрейф."""

    import hashlib

    pin = _load_native_pin()
    root = KERNEL_SOURCE / "cftuv_envelope"
    drift = []
    for operation in RUST_ONLY_OPERATIONS:
        for name in pin.OPERATION_FILES[operation]:
            path = root / name
            found = hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest() if path.exists() else None
            if found != pin.PINS[name]:
                drift.append(f"{operation}: {name}" + ("" if path.exists() else " (missing)"))
    assert not drift, (
        "стадия переведена на Rust, а её Python-эталон изменён мимо закрепления:\n"
        + "\n".join(drift)
        + "\n\nЗакон этой стадии меняется в Rust: правка Python-файла идёт вместе с портом и новым закреплением "
        "(`python -m cftuv_native.pin`) от Rust-сессии, в одном изменении."
    )


def test_a_dispatch_hook_in_a_pinned_oracle_file_comes_with_its_new_pin():
    """Диспетчер в закреплённом файле эталона делает порт `stale`, пока закрепление (`python -m cftuv_native.pin`) не перевыпущено.

    Диспетчер покрытия стоит в НЕзакреплённых файлах (`conveyor.py`, `step.py`); диспетчер резки — в `cut_domain` закреплённого `materialize/clip.py`
    (`run_clip(backend.clip_compute, ...)`, без подмены имён при запуске). Правка закреплённого файла допустима только вместе с новым закреплением
    в том же слиянии: тогда дайджест файла равен записи `PINS`, и этот тест зелёный; иначе он называет файл (резка названа `stale(materialize/clip.py)`).
    """

    import hashlib

    pin = _load_native_pin()
    pinned = {name for files in pin.OPERATION_FILES.values() for name in files}
    root = KERNEL_SOURCE / "cftuv_envelope"
    stale = []
    for name in sorted(pinned):
        path = root / name
        if path.exists() and _dispatch_hooks(_parse(path)):
            digest = hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
            if digest != pin.PINS[name]:
                stale.append(name)
    assert not stale, (
        "диспетчер бэкенда вписан в закреплённый файл эталона без нового закрепления: "
        + ", ".join(stale)
        + "\n\nПеревыпустите закрепления (`python -m cftuv_native.pin`) вместе с правкой либо переставьте диспетчер в незакреплённый файл."
    )


# --------------------------------------------------------------------------
# 13. Установщики подменяют каталог, а не стирают установленное
# --------------------------------------------------------------------------
# Прежний `Deploy` стирал старую копию аддона целиком и копировал новую. Когда воркеры пула открытого Blender держали .pyc ядра,
# стирание обрывалось на середине: штамп установки исчезал, аддон оставался наполовину стёртым. Теперь новая копия собирается и
# сверяется в стороне, занятые файлы называются ДО подмены, а старый каталог уходит одним переименованием
# (`tools/install_common.ps1`). Правило держит форму: оба установщика зовут `Get-LockedFiles` и `Install-Directories` и не стирают
# целевой каталог напрямую.

_INSTALLERS = ("install_to_blender.ps1", "install_native_to_blender.ps1")


def test_the_installers_swap_directories_and_never_erase_the_installed_copy_first():
    problems: list[str] = []
    for name in _INSTALLERS:
        text = (TOOLS / name).read_text(encoding="utf-8")
        for required in ("install_common.ps1", "Get-LockedFiles", "Install-Directories"):
            if required not in text:
                problems.append(f"tools/{name}: нет {required}")
        for forbidden in ("Remove-Item -Recurse -Force $target", "Remove-Item -Recurse -Force $packageTarget"):
            if forbidden in text:
                problems.append(f"tools/{name}: стирает установленное напрямую ({forbidden})")
    common = (TOOLS / "install_common.ps1").read_text(encoding="utf-8")
    for required in ("Directory]::Move", "function Undo-Install", "function Commit-Install"):
        if required not in common:
            problems.append(f"tools/install_common.ps1: нет {required}")
    assert not problems, "установщик теряет прежнюю установку при сбое:\n" + "\n".join(problems)


# --------------------------------------------------------------------------
# 14. Каждое стороннее имя, которое импортируют код и тесты, объявлено для CI
# --------------------------------------------------------------------------
# Прежний CI ставил `pytest sympy` и месяцами падал на сборе двумя `ModuleNotFoundError: numpy`: `cftuv/envelope_width_certificate.py`
# импортирует numpy (в Blender он есть), а список зависимостей workflow об этом не знал. Единственное место версий —
# `tests/requirements.txt`; здесь держится, что (1) любое стороннее имя из `cftuv/`, `tests/`, `tools/`, `kernel/` в нём названо или
# отнесено к тому, что даёт Blender, (2) чистое ядро по-прежнему просит только sympy, (3) workflow ставят именно этот файл,
# (4) матрица Python покрывает и заявленный пол (`kernel/pyproject.toml`), и интерпретатор Blender (3.11, на нём собирается нативное колесо).

#: Даёт сам Blender (4.5: CPython 3.11.11, numpy 1.26.4); в CI заменяется заглушками `tests/conftest.py`.
_BLENDER_PROVIDED = frozenset({"bpy", "bmesh", "mathutils", "gpu", "gpu_extras", "bpy_extras", "addon_utils"})
#: Нативное расширение собирается отдельно (`native/`), не из PyPI; host-тесты пропускают его по названному статусу `unavailable`.
_NATIVE_EXTENSION = frozenset({"cftuv_native"})
#: Зависимость, которая приходит вместе с объявленной (`sympy` тянет `mpmath`).
_BROUGHT_BY = {"mpmath": "sympy"}
_CI_REQUIREMENTS = TESTS / "requirements.txt"
_CI_WORKFLOWS = REPO_ROOT / ".github" / "workflows"


def _declared_requirements() -> set[str]:
    names: set[str] = set()
    for line in _CI_REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            names.add(re.split(r"[=<>!~\[;\s]", line, maxsplit=1)[0].lower().replace("-", "_"))
    return names


def _third_party_imports(folders) -> dict[str, set[str]]:
    """`{стороннее имя: {файлы, что его импортируют}}`: не стандартная библиотека и не модуль, лежащий в самих деревьях проекта."""

    files = [path for folder in folders for path in _python_files(folder)]
    own = {path.stem for path in files} | {path.parent.name for path in files} | {"cftuv", "cftuv_envelope", "research"}
    found: dict[str, set[str]] = {}
    for path in files:
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.Import):
                roots = [alias.name.split(".")[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                roots = [node.module.split(".")[0]]
            else:
                continue
            for root in roots:
                if root not in sys.stdlib_module_names and root not in own:
                    found.setdefault(root, set()).add(_relative(path))
    return found


def test_every_third_party_import_is_declared_for_ci():
    declared = _declared_requirements()
    known = declared | _BLENDER_PROVIDED | _NATIVE_EXTENSION | {name for name, parent in _BROUGHT_BY.items() if parent in declared}
    found = _third_party_imports([HOST_PACKAGE, TESTS, TOOLS, KERNEL_SOURCE, REPO_ROOT / "kernel" / "tests", REPO_ROOT / "kernel" / "tools"])
    missing = {name: sorted(files)[:3] for name, files in found.items() if name not in known}

    assert not missing, (
        "CI_DEPENDENCY_UNDECLARED: стороннее имя импортируется, а в tests/requirements.txt его нет "
        f"(workflow его не поставит, и сбор тестов упадёт ModuleNotFoundError): {missing}"
    )
    assert {"numpy", "sympy", "pytest"} <= declared, "tests/requirements.txt потерял зависимость, которую импортирует хост"


def test_the_pure_kernel_asks_for_nothing_but_sympy():
    kernel = set(_third_party_imports([KERNEL_SOURCE]))
    extra = kernel - {"sympy", "mpmath"} - _NATIVE_EXTENSION

    assert not extra, (
        f"чистое ядро (kernel/src) потребовало стороннее имя сверх sympy: {sorted(extra)}; "
        "оно ставится колесом `cftuv-envelope-core` с одной зависимостью, и CI-ветка 3.10 его не получит"
    )


def test_the_host_workflows_install_the_declared_requirements():
    for name in ("host-suite.yml", "envelope-kernel.yml"):
        text = (_CI_WORKFLOWS / name).read_text(encoding="utf-8")
        assert "-r tests/requirements.txt" in text, f"{name}: host-тесты запускаются без tests/requirements.txt: зависимости разойдутся с набором"
        assert "sympy==" not in text and "numpy==" not in text, f"{name}: версия зависимости прибита в workflow, а не в tests/requirements.txt"


def test_the_python_matrix_covers_the_declared_floor_and_the_blender_interpreter():
    floor = re.search(r'requires-python\s*=\s*">=(\d+\.\d+)"', (REPO_ROOT / "kernel" / "pyproject.toml").read_text(encoding="utf-8"))
    assert floor is not None, "kernel/pyproject.toml без requires-python"
    for name in ("host-suite.yml", "envelope-kernel.yml"):
        text = (_CI_WORKFLOWS / name).read_text(encoding="utf-8")
        lines = [line for line in text.splitlines() if "python-version:" in line]
        versions = {version for line in lines for version in re.findall(r'"(3\.\d+)"', line)}
        assert floor.group(1) in versions, f"{name}: нет ветки на заявленном полу Python {floor.group(1)}: {sorted(versions)}"
        assert "3.11" in versions, f"{name}: нет ветки на CPython 3.11 (Blender 4.5, нативное колесо `abi3-py311`): {sorted(versions)}"


def _portable_ast_dump(node) -> str:
    """`ast.dump` без пустых полей: одинаков на 3.10–3.13 (3.12 добавил `type_params=[]`, 3.13 перестал печатать пустые поля)."""

    if isinstance(node, ast.AST):
        parts = []
        for name in node._fields:
            value = getattr(node, name, None)
            if value is None or (isinstance(value, list) and not value):
                continue
            parts.append(f"{name}={_portable_ast_dump(value)}")
        return f"{type(node).__name__}({', '.join(parts)})"
    if isinstance(node, list):
        return "[" + ", ".join(_portable_ast_dump(item) for item in node) + "]"
    return repr(node)


# B1_HOST_DISPATCH_V1: выбор исполнителя не становится законом памяти сертификатов.
def test_embedding_hook_preserves_the_value_memo_and_frozen_python_leaf():
    import copy
    import hashlib

    tree = _parse(KERNEL_SOURCE / "cftuv_envelope" / "_embedding.py")
    node = copy.deepcopy(next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "build_source_snap_embedding_certificate"))
    calls = [n for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name) and n.func.value.id == "backend"]
    assert [n.func.attr for n in calls].count("embedding_compute") == 3
    assert [n.func.attr for n in calls].count("note_embedding_cache_hit") == 1

    class OriginalMemo(ast.NodeTransformer):
        def visit_ImportFrom(self, n):
            return None if n.level == 1 and n.module is None and [a.name for a in n.names] == ["backend"] else n

        def visit_Expr(self, n):
            if isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Attribute) and n.value.func.attr == "note_embedding_cache_hit":
                return None
            return self.generic_visit(n)

        def visit_Call(self, n):
            if isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name) and n.func.value.id == "backend" and n.func.attr == "embedding_compute":
                n.func = ast.Name(id="_compute_source_snap_embedding_certificate", ctx=ast.Load())
            return self.generic_visit(n)

    original = OriginalMemo().visit(node)
    # дайджест обёртки до хука (a9744e54), снятый на 3.10, 3.11 и 3.13 одинаково
    assert hashlib.sha256(_portable_ast_dump(original).encode()).hexdigest() == "29107da47955c0e78171a6f214b2525dc1067c33d142ec0bb16e160707b24a9a", "B1 changed memo value/code key, normalization, lock, LRU, or returned identity"
    dispatcher = _parse(KERNEL_SOURCE / "cftuv_envelope" / "backend.py")
    required = next(n for n in dispatcher.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "_REQUIRED" for t in n.targets))
    assert "snap_embedding_certificate" not in ast.dump(required), "B1 must not disable the older three-operation wheel"
    assert "snap_embedding" in RUST_ONLY_OPERATIONS, "EMBEDDING_NATIVE_DEFAULT_V1: the certificate stage is Rust-only after the strict field A/B and parity"


# --------------------------------------------------------------------------
# Главный переключатель бэкенда ядра: одно именованное умолчание, стадии без своих
# --------------------------------------------------------------------------

#: Имена параметров, полей и свойств сцены, которыми хост заказывает бэкенд ядра.
_BACKEND_NAMES = frozenset({"backend", "kernel_backend", "backend_id", "skeleton_backend", "embedding_backend"})
#: Постадийные порядки: у стадии собственного умолчания нет, `None` значит «как главный переключатель» (`stage_orders`).
_BACKEND_STAGE_NAMES = frozenset({"skeleton_backend", "embedding_backend"})
_BACKEND_MASTER_DEFAULTS = frozenset({"DEFAULT_KERNEL_BACKEND", "None"})


def _backend_defaults(tree: ast.Module, file: str) -> list:
    """`[(файл, имя, текст умолчания)]` по параметрам функций, полям записей и свойствам сцены (`kernel_backend: EnumProperty(..., default=...)`)."""

    found: list = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            arguments = node.args
            positional = [*arguments.posonlyargs, *arguments.args]
            pairs = list(zip(positional[len(positional) - len(arguments.defaults) :], arguments.defaults))
            pairs += [(arg, default) for arg, default in zip(arguments.kwonlyargs, arguments.kw_defaults) if default is not None]
            found += [(file, arg.arg, ast.unparse(default)) for arg, default in pairs if arg.arg in _BACKEND_NAMES]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id in _BACKEND_NAMES:
            value = node.value
            if value is None and isinstance(node.annotation, ast.Call):
                value = next((item.value for item in node.annotation.keywords if item.arg == "default"), None)
            if value is not None:
                found.append((file, node.target.id, ast.unparse(value)))
    return found


def _backend_violations(tree: ast.Module, file: str) -> list:
    """Умолчание не из единственного места либо имя бэкенда литералом в вызове: `[(файл, имя, что нашли)]`."""

    bad = [
        item
        for item in _backend_defaults(tree, file)
        if item[2] not in (frozenset({"None"}) if item[1] in _BACKEND_STAGE_NAMES else _BACKEND_MASTER_DEFAULTS)
    ]
    bad += [
        (file, node.arg, f"literal {node.value.value!r} at line {node.lineno}")
        for node in ast.walk(tree)
        if isinstance(node, ast.keyword)
        and node.arg in _BACKEND_NAMES
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    ]
    return bad


#: Места проводки, которые правило обязано видеть: прогон, задача пула, запись живой ширины, свойство сцены, ключи кэшей и постадийные порядки API.
_BACKEND_WIRING_SITES = (
    ("envelope_kernel_backend.py", "backend"),
    ("envelope_production_export.py", "kernel_backend"),
    ("envelope_production_export.py", "backend"),
    ("envelope_domain_pool.py", "backend"),
    ("envelope_width_live.py", "kernel_backend"),
    ("envelope_production_operator.py", "kernel_backend"),
    ("envelope_content_key.py", "backend"),
    ("envelope_content_key.py", "backend_id"),
    ("envelope_kernel_backend.py", "skeleton_backend"),
    ("envelope_production_export.py", "skeleton_backend"),
    ("envelope_domain_pool.py", "skeleton_backend"),
    ("envelope_queue_export.py", "skeleton_backend"),
    ("envelope_width_live.py", "skeleton_backend"),
    ("envelope_width_live.py", "embedding_backend"),
    ("envelope_content_key.py", "skeleton_backend"),
    ("envelope_kernel_backend.py", "embedding_backend"),
    ("envelope_production_export.py", "embedding_backend"),
    ("envelope_domain_pool.py", "embedding_backend"),
    ("envelope_content_key.py", "embedding_backend"),
)


def test_every_backend_default_of_the_host_is_the_one_named_constant():
    """Умолчание бэкенда названо ОДНИМ местом — главным переключателем `DEFAULT_KERNEL_BACKEND`; у стадии умолчания нет.

    Литерал `"PYTHON"`/`"NATIVE"` в умолчании параметра, поля либо свойства сцены — дефект: прогон, задача пула, запись живой ширины и настройка сцены разошлись бы молча.
    `backend_id` без значения — `None` (идентичность берётся у умолчания). Порядок стадии (`skeleton_backend`, `embedding_backend`) без слова — `None`, то есть «как главный
    переключатель» (`stage_orders`): прогон с `kernel_backend="PYTHON"` без слов о стадиях считает эталон на ВСЕХ стадиях. Литерал имени бэкенда в вызове на продуктовом пути
    (`skeleton_backend="PYTHON"`) — тоже дефект: постадийный порядок живёт только в API и инструментах (`tools/`), а не в хосте.
    """

    found: list = []
    violations: list = []
    for path in _python_files(HOST_PACKAGE):
        tree = _parse(path)
        found += _backend_defaults(tree, path.name)
        violations += _backend_violations(tree, path.name)
    assert not violations, violations
    # единственная константа умолчания: ни у одной стадии своей нет
    module = _parse(HOST_PACKAGE / "envelope_kernel_backend.py")
    constants = {
        target.id
        for node in module.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name) and target.id.startswith("DEFAULT_") and target.id.endswith("_BACKEND")
    }
    assert constants == {"DEFAULT_KERNEL_BACKEND"}, constants
    # правило не пустое: оно видит каждое место проводки
    seen = {(file, name) for file, name, _default in found}
    missing = [site for site in _BACKEND_WIRING_SITES if site not in seen]
    assert not missing, missing


def test_the_backend_default_rule_flags_a_spoiled_source_and_passes_an_honest_one():
    spoiled = (
        "def run(kernel_backend='PYTHON', skeleton_backend='NATIVE', embedding_backend=DEFAULT_KERNEL_BACKEND):\n"
        "    return other(skeleton_backend='PYTHON')\n"
        "class Record:\n"
        "    backend: str = 'NATIVE'\n"
        "    skeleton_backend: str = DEFAULT_SKELETON_BACKEND\n"
    )
    honest = (
        "def run(kernel_backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None, backend_id=None):\n"
        "    return other(kernel_backend=kernel_backend, skeleton_backend=skeleton_backend)\n"
        "class Record:\n"
        "    backend: str = DEFAULT_KERNEL_BACKEND\n"
        "    skeleton_backend: str | None = None\n"
    )
    assert len(_backend_violations(ast.parse(spoiled), "spoiled.py")) == 6
    assert _backend_violations(ast.parse(honest), "honest.py") == []


def test_the_scene_has_exactly_one_backend_setting_the_master_switch():
    """Настройка владельца одна: «Kernel backend» заказывает все стадии; отдельного свойства стадии в сцене и в панели нет."""

    group = next(
        node
        for node in _parse(HOST_PACKAGE / "envelope_production_operator.py").body
        if isinstance(node, ast.ClassDef) and node.name == "HOTSPOTUV_DecalMeshSettings"
    )
    properties = [node.target.id for node in group.body if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)]
    assert [name for name in properties if "backend" in name] == ["kernel_backend"], properties
