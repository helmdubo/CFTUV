"""Нужна ли нативная приёмка этому коммиту: какие пути он трогает (`.github/workflows/native.yml`, задача `changes`).

Workflow `native.yml` работает на КАЖДОМ коммите и PR (фильтров путей на уровне workflow нет): обязательная проверка `native-gate` обязана отчитаться всегда, а проверка, чей workflow
не запустился, у защищённой ветки висит «ожидается» вечно. Тяжёлые задачи (cargo, колесо, корпус, матрица) идут только когда коммит трогает нативные пути; эта программа решает, трогает ли.

    python tools/native_ci_changes.py        # среда: EVENT, BASE_SHA, HEAD_SHA, BEFORE, SHA; пишет `native=true|false` в $GITHUB_OUTPUT, печатает совпавшие файлы

Диапазон (обычный git, чужих действий нет; репозиторий обязан быть получен с полной историей, `fetch-depth: 0`):

* `pull_request`: `BASE_SHA...HEAD_SHA` (от общего предка: ушедшая вперёд база не считается изменением PR);
* `push`: `BEFORE..SHA`; если `BEFORE` нулевой (новая ветка) либо такого коммита в репозитории нет (форс-пуш), то `merge-base origin/main SHA..SHA`, а если merge-base и есть `SHA`
  (пуш уже проверенного коммита main) — собственный diff коммита против первого родителя (у корневого коммита — все его файлы);
* `workflow_dispatch`: всегда `true`.

ЛЮБАЯ ошибка здесь — `native=true` (отказ в безопасную сторону: лишняя проверка дешевле пропущенной). Набор путей — тот, что прежде стоял фильтром workflow (`NATIVE_PATHS`).
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ZERO_SHA = "0" * 40

#: Пути, правка которых требует нативной приёмки (синтаксис фильтров GitHub: `**` — любые символы вместе с `/`, `*` и `?` — без `/`).
NATIVE_PATHS = (
    "native/**",
    "tools/native_*.py",
    "tests/test_native_*.py",
    "tests/native_gate.py",
    "tests/conftest.py",
    "tests/test_architecture.py",
    # every oracle file pinned in `cftuv_native/pin.py` lives here, and the tests import the kernel's own test helpers
    "kernel/src/cftuv_envelope/**",
    "kernel/tests/**",
    "kernel/pyproject.toml",
    "pytest.ini",
    ".github/workflows/native.yml",
)


def _regex(pattern: str) -> "re.Pattern[str]":
    out, index = [], 0
    while index < len(pattern):
        if pattern.startswith("**", index):
            out.append(".*")
            index += 2
            continue
        char = pattern[index]
        out.append("[^/]*" if char == "*" else "[^/]" if char == "?" else re.escape(char))
        index += 1
    return re.compile("".join(out))


_COMPILED = tuple(_regex(pattern) for pattern in NATIVE_PATHS)


def is_native_path(path: str) -> bool:
    """Путь (с `/`, от корня репозитория) подпадает под `NATIVE_PATHS`."""

    return any(compiled.fullmatch(path) for compiled in _COMPILED)


class ChangeError(RuntimeError):
    """Диапазон коммитов определить не удалось: вызывающий решает `native=true`."""


def _git(cwd: Path, *arguments: str) -> str:
    done = subprocess.run(["git", "-c", "core.quotepath=off", *arguments], cwd=str(cwd), capture_output=True, text=True, encoding="utf-8", check=False)
    if done.returncode != 0:
        raise ChangeError(f"git {' '.join(arguments)} -> exit {done.returncode}: {done.stderr.strip()[:300]}")
    return done.stdout


def _commit_exists(cwd: Path, sha: str) -> bool:
    done = subprocess.run(["git", "cat-file", "-e", f"{sha}^{{commit}}"], cwd=str(cwd), capture_output=True, check=False)
    return done.returncode == 0


def _names(cwd: Path, *arguments: str) -> list:
    return [name for name in _git(cwd, "diff", "--name-only", "-z", *arguments).split("\0") if name]


def _own_diff(cwd: Path, sha: str) -> tuple:
    """Файлы самого коммита `sha` против его первого родителя (корневой коммит — все его файлы)."""

    if _commit_exists(cwd, f"{sha}^1"):
        return _names(cwd, f"{sha}^1", sha), f"the commit's own diff {sha[:10]}^1..{sha[:10]} (an already checked commit)"
    listing = _git(cwd, "ls-tree", "-r", "--name-only", "-z", sha)
    return [name for name in listing.split("\0") if name], f"the files of the root commit {sha[:10]}"


def changed_files(environment: dict, cwd: Path) -> tuple:
    """`(файлы | None, как определено)`; `None` — диапазон не нужен (ручной запуск)."""

    event = environment.get("EVENT", "")
    if event == "workflow_dispatch":
        return None, "workflow_dispatch: a manual run always takes the native acceptance"
    if event == "pull_request":
        base, head = environment.get("BASE_SHA", ""), environment.get("HEAD_SHA", "")
        if not base or not head:
            raise ChangeError("pull_request without BASE_SHA or HEAD_SHA")
        return _names(cwd, f"{base}...{head}"), f"pull request {base[:10]}...{head[:10]}"
    if event == "push":
        sha = environment.get("SHA", "") or "HEAD"
        before = environment.get("BEFORE", "")
        if before and before != ZERO_SHA and _commit_exists(cwd, before):
            return _names(cwd, f"{before}..{sha}"), f"push {before[:10]}..{sha[:10]}"
        reason = "a new branch" if not before or before == ZERO_SHA else f"the previous tip {before[:10]} is not in the repository"
        base = _git(cwd, "merge-base", "origin/main", sha).strip()
        head = _git(cwd, "rev-parse", f"{sha}^{{commit}}").strip()
        if base == head:
            files, how = _own_diff(cwd, head)
            return files, f"{reason}; the tip is on origin/main: {how}"
        return _names(cwd, f"{base}..{head}"), f"{reason}: merge-base with origin/main {base[:10]}..{head[:10]}"
    raise ChangeError(f"unknown event {event!r}")


@dataclass
class Decision:
    native: bool
    how: str
    files: list = field(default_factory=list)
    matched: list = field(default_factory=list)


def decide(environment: dict, cwd: Path) -> Decision:
    """`native=true`, если коммит трогает нативные пути, ручной запуск либо диапазон определить не удалось."""

    try:
        files, how = changed_files(environment, cwd)
    except Exception as error:  # noqa: BLE001 - ЛЮБАЯ ошибка = лишняя проверка, не пропущенная
        return Decision(True, f"ERROR, so the native acceptance runs (fail safe): {type(error).__name__}: {error}")
    if files is None:
        return Decision(True, how)
    matched = [name for name in files if is_native_path(name)]
    return Decision(bool(matched), how, files, matched)


def report(decision: Decision) -> list:
    lines = [f"range: {decision.how}", f"changed files: {len(decision.files)}"]
    lines += [f"  {'NATIVE' if name in decision.matched else 'host  '}  {name}" for name in decision.files[:300]]
    if len(decision.files) > 300:
        lines.append(f"  ... and {len(decision.files) - 300} more")
    lines.append(f"native-relevant files: {len(decision.matched)}")
    lines.append(f"native={'true' if decision.native else 'false'}")
    return lines


def main(environment: dict | None = None, cwd: Path | None = None) -> int:
    environment = dict(os.environ if environment is None else environment)
    decision = decide(environment, Path.cwd() if cwd is None else cwd)
    lines = report(decision)
    print("\n".join(lines), flush=True)
    target = environment.get("GITHUB_OUTPUT")
    if target:
        with open(target, "a", encoding="utf-8") as handle:
            handle.write(f"native={'true' if decision.native else 'false'}\n")
    summary = environment.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a", encoding="utf-8") as handle:
            handle.write("### Native acceptance: " + ("REQUIRED" if decision.native else "not required (no native-relevant path changed)") + "\n\n```\n" + "\n".join(lines) + "\n```\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
