"""Общие ворота нативных сверок с эталоном: пропуск с названной причиной локально, ОТКАЗ в строгом режиме CI; плюс сам плагин строгого режима.

Нативная операция побитово равна ОДНОЙ версии ядра на Python (`cftuv_native.pin`). Пока основная ветка двигает ядро, сверка порта с ним
не имеет смысла и не должна краснить главную: модуль пропускается, причина названа (какие файлы эталона ушли от пина). Сам механизм пина
проверяется в `test_native_pin.py` и ворот не знает.

СТРОГИЙ РЕЖИМ (`CFTUV_NATIVE_STRICT=1`, ставит `.github/workflows/native.yml`). Пропуск локально значит «сверку не сделали, причина названа»; в CI он значил бы «приёмку Rust
выдали за пройденную, не запустив Rust». Поэтому в строгом режиме КАЖДЫЙ пропуск (модуль, тест, параметр, `pytest.skip` внутри теста) превращается в провал, с названной причиной:
расширение не установлено, пин устарел (эталон ушёл от порта), личность сборки не равна дереву, нет синтетического корпуса. Исключений два, оба названные:

* `skip_for_interpreter(...)`: проверка, которая по своей сути принадлежит одной версии интерпретатора (сортировка 3.11), пропуск остаётся пропуском и виден в отчёте;
* уровень `native_field` (`field_tier`): тесты, которым нужен ПОЛЕВОЙ корпус владельца (записи вызовов из сцены Blender, `E:/cftuv_native_corpus`; в CI его нет и быть не может). CI снимает их
  ПО ИМЕНИ (`-m "not native_field"`) и отчёт называет их: сколько и какие. Снять по имени что-либо ещё (`-k`, другой `-m`) в строгом режиме — провал: скрыть тест нельзя.

Всё это — хуки pytest этого модуля (`tests/conftest.py` регистрирует модуль плагином; `-p native_gate` делает то же): ни один нативный тестовый модуль, нынешний или будущий, не обязан помнить правило.
"""

from __future__ import annotations

import importlib.metadata
import importlib.util
import os
from pathlib import Path

import pytest

STRICT_ENVIRONMENT = "CFTUV_NATIVE_STRICT"
FIELD_MARKER = "native_field"
#: Начало причины пропуска, который строгий режим оставляет пропуском (`skip_for_interpreter`).
INTERPRETER_SKIP_PREFIX = "[interpreter] "
SUMMARY_ENVIRONMENT = "GITHUB_STEP_SUMMARY"

_STATE: dict = {"deselected_field": [], "deselected_other": [], "converted": []}


def strict() -> bool:
    """`CFTUV_NATIVE_STRICT`: `1` — строгий режим, пусто или `0` — нет; любое другое значение — ошибка запуска (опечатка не должна тихо выключать строгость)."""

    value = os.environ.get(STRICT_ENVIRONMENT, "").strip()
    if value in ("", "0"):
        return False
    if value == "1":
        return True
    raise pytest.UsageError(f"{STRICT_ENVIRONMENT} is `1` or `0` (or empty), not {value!r}")


def skip_unless_available(extension, operation: str) -> None:
    """Пропуск модуля (`allow_module_level`) с причиной, если нативная операция не `available`; иначе возврат (строгий режим превращает этот пропуск в провал)."""

    status = extension.native_status()[operation]
    if status != "available":
        pytest.skip(
            f"нативная операция `{operation}` сейчас `{status}`: сверять её с этим деревом ядра нечем "
            "(перенос дельты и новый пин — отдельный шаг; `python native/cftuv-python/python/cftuv_native/pin.py kernel/src/cftuv_envelope` печатает дайджесты дерева)",
            allow_module_level=True,
        )


def skip_for_interpreter(reason: str) -> None:
    """Пропуск проверки, которая принадлежит другой версии интерпретатора: единственный пропуск, который строгий режим не превращает в провал."""

    pytest.skip(INTERPRETER_SKIP_PREFIX + reason)


def field_tier(present: bool, reason: str):
    """Декоратор теста, которому нужен ПОЛЕВОЙ корпус владельца: метка `native_field` (CI снимает уровень по имени) и пропуск, пока корпуса нет (локально, вне CI)."""

    def decorate(test):
        return getattr(pytest.mark, FIELD_MARKER)(pytest.mark.skipif(not present, reason=reason)(test))

    return decorate


# --------------------------------------------------------------------------
# хуки pytest (плагин)
# --------------------------------------------------------------------------


def pytest_configure(config) -> None:
    config.addinivalue_line("markers", f"{FIELD_MARKER}: needs the owner's FIELD corpus (calls recorded from the Blender scene); the native CI deselects this tier by name and reports it")
    strict()  # a bad value fails the run before anything is collected


def _reason(report) -> str:
    longrepr = report.longrepr
    text = str(longrepr[2]) if isinstance(longrepr, tuple) and len(longrepr) == 3 else str(longrepr)
    return text.removeprefix("Skipped: ")


def _strict_failure(what: str, reason: str) -> str:
    return (
        f"{STRICT_ENVIRONMENT}=1: {what} was SKIPPED, and in strict mode a skip is a failure (a check that did not run is not a check that passed).\n"
        f"reason: {reason}\n"
        f"If it needs the owner's field corpus, mark it `@pytest.mark.{FIELD_MARKER}` (`native_gate.field_tier`) so CI deselects it by name; otherwise fix what is missing "
        "(install the wheel, move the pin, build the synthetic corpus)."
    )


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_make_collect_report(collector):
    outcome = yield
    report = outcome.get_result()
    if strict() and report.skipped:
        reason = _reason(report)
        report.outcome = "failed"
        report.longrepr = _strict_failure(f"the collection of {collector.nodeid}", reason)
        _STATE["converted"].append((collector.nodeid, reason))


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    if strict() and report.skipped and not hasattr(report, "wasxfail"):
        reason = _reason(report)
        if not reason.startswith(INTERPRETER_SKIP_PREFIX):
            report.outcome = "failed"
            report.longrepr = _strict_failure(item.nodeid, reason)
            _STATE["converted"].append((item.nodeid, reason))


def pytest_deselected(items) -> None:
    for item in items:
        _STATE["deselected_field" if item.get_closest_marker(FIELD_MARKER) is not None else "deselected_other"].append(item.nodeid)


def pytest_report_header(config) -> list:
    """What the run was asked to test: the strict switch, the slot mode and where `cftuv_native` would come from (found, NOT imported: the shim is imported by the tests, this module is not a native test)."""

    lines = [f"cftuv native gate: strict={'on' if strict() else 'off'} ({STRICT_ENVIRONMENT}), slots={os.environ.get('CFTUV_NATIVE_SLOTS') or 'unset'}, field tier marker `{FIELD_MARKER}`"]
    spec = importlib.util.find_spec("cftuv_native")
    if spec is None:
        lines.append("cftuv_native: NOT INSTALLED")
    else:
        try:
            version = importlib.metadata.version("cftuv-native")
        except importlib.metadata.PackageNotFoundError:
            version = "no dist-info (not an installed wheel)"
        lines.append(f"cftuv_native {version} from {Path(spec.origin).parent}")
    return lines


def pytest_terminal_summary(terminalreporter, exitstatus, config) -> None:
    field, other = _STATE["deselected_field"], _STATE["deselected_other"]
    if not (field or other or _STATE["converted"]):
        return
    write = terminalreporter.write_line
    terminalreporter.section("cftuv native gate")
    write(f"field tier (`{FIELD_MARKER}`) DESELECTED: {len(field)} tests (they need the owner's field corpus, which CI does not have; run them in the field cycle)")
    for nodeid in sorted(field):
        write(f"  deselected: {nodeid}")
    if other:
        write(f"NOT the field tier but deselected: {len(other)} tests" + (" (a failure in strict mode)" if strict() else " (a name filter of a developer's own run)"))
        for nodeid in sorted(other) if strict() else ():
            write(f"  hidden: {nodeid}")
    if _STATE["converted"]:
        write(f"skips turned into failures by strict mode: {len(_STATE['converted'])}")
    target = os.environ.get(SUMMARY_ENVIRONMENT)
    if target and field:
        with open(target, "a", encoding="utf-8") as handle:
            handle.write(f"\n### Native field tier: {len(field)} tests DESELECTED by name (`-m \"not {FIELD_MARKER}\"`)\n\n")
            handle.write("They need the owner's field corpus (calls recorded from the Blender scene) and run only in the field cycle.\n\n")
            handle.write("\n".join(f"- `{nodeid}`" for nodeid in sorted(field)) + "\n")


def pytest_sessionfinish(session, exitstatus) -> None:
    if strict() and _STATE["deselected_other"] and session.exitstatus == pytest.ExitCode.OK:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED
