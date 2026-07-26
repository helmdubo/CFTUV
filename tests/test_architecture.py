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


def _python_files(root: Path) -> tuple[Path, ...]:
    return tuple(sorted(path for path in root.rglob("*.py")))


def _parse(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


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


def _line_count(path: Path) -> int:
    return len(path.read_text(encoding="utf-8").splitlines())


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
)


def test_deleted_legacy_stays_deleted():
    """Легаси-монолит и отдельный BAND-оператор удалены и не возвращаются.

    `AGENTS.md` требовал этого прозой; правило нарушалось. Теперь оно исполняемое.
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
    "cftuv/decal_voronoi.py": 16889,
    "cftuv/decal_rail_geometry.py": 5583,
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
    "cftuv/envelope_request_export.py": 3115,
    "cftuv/operators.py": 2913,
    "cftuv/decals.py": 2823,
    "cftuv/decal_rails.py": 2486,
}


# Файлы, в которых самая длинная функция превышает NEW_FUNCTION_LINE_LIMIT
# на момент заморозки.
FUNCTION_LINE_ALLOWANCE = {
    "cftuv/decal_voronoi.py": 1815,
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
    "cftuv/decal_rails.py": 445,
    "kernel/src/cftuv_envelope/validation.py": 426,
    # +5: снятие дельты счётчика локализации точки и два поля в union. Плата за
    # то, чтобы следующий полевой прогон отвечал на вопрос, а не ставил его
    # заново; `exact_union` всё равно подлежит разбиению на этапы конвейера.
    "kernel/src/cftuv_envelope/reference/arrangement.py": 413,
    "cftuv/decal_rail_geometry.py": 392,
    "kernel/src/cftuv_envelope/interactions/mutual_arrival.py": 385,
    "cftuv/frontier_rescue.py": 353,
    "kernel/src/cftuv_envelope/interactions/arrival.py": 344,
    "cftuv/decals.py": 337,
    "cftuv/analysis_validation.py": 336,
    # −28: словарь счётчиков arrangement вынесен в `_union_counters`.
    "kernel/src/cftuv_envelope/reference/raw_coverage.py": 291,
    "cftuv/solve_transfer.py": 317,
    "cftuv/solve_reporting.py": 293,
    "cftuv/solve_report_anomalies.py": 250,
    "cftuv/operators.py": 228,
    "cftuv/structural_tokens.py": 201,
    "cftuv/frontier_eval.py": 201,
    "cftuv/solve_frontier.py": 197,
    "kernel/src/cftuv_envelope/reference/boundary.py": 193,
    "kernel/src/cftuv_envelope/planar_metric.py": 190,
    # +1: печать диагностик ядра, а не только отказов.
    "cftuv/envelope_debug_renderer.py": 180,
    "cftuv/decal_chart_admission.py": 170,
    "kernel/src/cftuv_envelope/interactions/validation.py": 167,
    "cftuv/analysis_boundary_loops.py": 163,
    "tools/benchmark_envelope_metric_models.py": 161,
    "cftuv/frontier_score.py": 161,
    "tools/export_envelope_metric_patch.py": 156,
    "tools/export_building_002_point_contact_fixture.py": 154,
    "cftuv/analysis_surface.py": 146,
    "kernel/src/cftuv_envelope/reference/angular.py": 145,
    "cftuv/frontier_finalize.py": 142,
    "tools/run_envelope_mr1_building_gate.py": 136,
    "cftuv/debug.py": 135,
    "kernel/src/cftuv_envelope/reference/strip.py": 134,
    "cftuv/solve_report_metrics.py": 133,
    "kernel/src/cftuv_envelope/reference/validation.py": 129,
    "cftuv/solve_planning.py": 128,
    "cftuv/decal_charts.py": 125,
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
# 5. Заморозка легаси decal-движков
#
# В проекте одновременно живы три decal-конвейера: PATCH_VORONOI, RAIL_PLANAR
# и envelope. Каждая новая возможность стоит втрое. Пока envelope не достиг
# паритета, два старых движка заморожены: в них допустимы только исправления
# падений в поле, но не рост.
#
# Значения совпадают с MODULE_LINE_ALLOWANCE намеренно — это отдельное
# утверждение с отдельным смыслом, и оно должно падать со своим сообщением.
# --------------------------------------------------------------------------


FROZEN_LEGACY_ENGINES = {
    "cftuv/decal_voronoi.py": 16889,
    "cftuv/decal_rails.py": 2486,
    "cftuv/decal_rail_geometry.py": 5583,
    "cftuv/decal_charts.py": 1436,
    "cftuv/decals.py": 2823,
}


@pytest.mark.parametrize("name, frozen_size", sorted(FROZEN_LEGACY_ENGINES.items()))
def test_legacy_decal_engines_do_not_grow(name: str, frozen_size: int):
    """Легаси decal-движки заморожены: только исправления, никаких новых возможностей."""

    path = REPO_ROOT / name
    if not path.exists():
        pytest.skip(f"{name} удалён — паритет envelope достигнут")
    actual = _line_count(path)
    assert actual <= frozen_size, (
        f"{name} вырос с {frozen_size} до {actual} строк. "
        "Легаси-движок заморожен: новые возможности идут в envelope-ядро. "
        "Исправление падения в поле, которое требует роста, — повод уменьшить "
        "число здесь после удаления мёртвого кода рядом, а не поднять его."
    )


# --------------------------------------------------------------------------
# 6. Гигиена репозитория
# --------------------------------------------------------------------------


LARGE_FILE_BYTES = 512 * 1024

# Крупные файлы, попавшие в git до введения правила. История их уже содержит,
# переписывание истории — отдельное осознанное действие. Новые крупные файлы
# должны идти через LFS (см. .gitattributes).
KNOWN_LARGE_FILE_COUNT = 6


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
