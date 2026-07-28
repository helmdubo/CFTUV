"""Read-only build identity для developer checkout аддона."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path


_UNAVAILABLE = "unavailable"
_EMBEDDED_BRANCH = "claude/blender-decal-corner-preview-yq4lir"
# Обновляется metadata-only commit после каждого implementation commit.
# ВНИМАНИЕ: процесс обновления этих констант мёртв, значения ископаемые.
# Установленная копия обязана опознаваться штампом установщика (ниже), а не
# ими; они остаются последним фолбэком для копий без штампа и без .git.
_EMBEDDED_CODE_COMMIT = "b19fe0dbbaca29e1b9841454d5b030ef29fd62f5"

#: Пишется tools/install_to_blender.ps1 в корень установленной копии.
_INSTALL_STAMP_FILE = "_install_stamp.txt"


@dataclass(frozen=True)
class AddonBuildInfo:
    """Ветка и commit, из которых Blender загрузил текущий Python package."""

    branch: str = _UNAVAILABLE
    commit: str = _UNAVAILABLE
    source: str = "embedded"

    @property
    def short_commit(self) -> str:
        if self.commit == _UNAVAILABLE:
            return self.commit
        return self.commit[:8]


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return ""


def _find_git_dir(start: Path) -> Path | None:
    directory = start if start.is_dir() else start.parent
    for candidate in (directory, *directory.parents):
        marker = candidate / ".git"
        if marker.is_dir():
            return marker.resolve()
        if not marker.is_file():
            continue
        marker_text = _read_text(marker)
        prefix = "gitdir:"
        if not marker_text.lower().startswith(prefix):
            continue
        git_dir = Path(marker_text[len(prefix) :].strip())
        if not git_dir.is_absolute():
            git_dir = marker.parent / git_dir
        return git_dir.resolve()
    return None


def _git_storage_dirs(git_dir: Path) -> tuple[Path, ...]:
    storage_dirs = [git_dir]
    common_dir_text = _read_text(git_dir / "commondir")
    if common_dir_text:
        common_dir = Path(common_dir_text)
        if not common_dir.is_absolute():
            common_dir = git_dir / common_dir
        common_dir = common_dir.resolve()
        if common_dir != git_dir:
            storage_dirs.append(common_dir)
    return tuple(storage_dirs)


def _resolve_ref(storage_dirs: tuple[Path, ...], ref_name: str) -> str:
    for storage_dir in storage_dirs:
        value = _read_text(storage_dir / ref_name)
        if value:
            return value.split()[0]
    for storage_dir in storage_dirs:
        packed_refs = _read_text(storage_dir / "packed-refs")
        for line in packed_refs.splitlines():
            if not line or line.startswith(("#", "^")):
                continue
            parts = line.split()
            if len(parts) == 2 and parts[1] == ref_name:
                return parts[0]
    return _UNAVAILABLE


def _stamp_build_info(start: Path) -> AddonBuildInfo | None:
    """Identity установленной копии из штампа установщика.

    Штамп — единственный источник правды для копии в `scripts\\addons`: у неё
    нет `.git`, а зашитые константы ископаемые. Формат строки задаёт
    установщик: `commit <hash> (<branch>), установлено ...`.
    """

    directory = start if start.is_dir() else start.parent
    text = _read_text(directory / _INSTALL_STAMP_FILE)
    found = re.search(r"commit\s+([0-9a-f]{7,40})\s+\(([^)]+)\)", text)
    if not found:
        return None
    return AddonBuildInfo(
        branch=found.group(2),
        commit=found.group(1),
        source="install_stamp",
    )


def _fallback_build_info(start: Path) -> AddonBuildInfo:
    stamped = _stamp_build_info(start)
    if stamped is not None:
        return stamped
    return AddonBuildInfo(branch=_EMBEDDED_BRANCH, commit=_EMBEDDED_CODE_COMMIT)


def resolve_addon_build_info(start: Path | str) -> AddonBuildInfo:
    """Определяет identity checkout, включая linked Git worktree."""

    start_path = Path(start).resolve()
    git_dir = _find_git_dir(start_path)
    if git_dir is None:
        return _fallback_build_info(start_path)
    head = _read_text(git_dir / "HEAD")
    if not head:
        return _fallback_build_info(start_path)
    if not head.startswith("ref:"):
        return AddonBuildInfo(
            branch="detached",
            commit=head.split()[0],
            source="checkout",
        )

    ref_name = head[4:].strip()
    branch_prefix = "refs/heads/"
    branch = (
        ref_name[len(branch_prefix) :]
        if ref_name.startswith(branch_prefix)
        else ref_name
    )
    commit = _resolve_ref(_git_storage_dirs(git_dir), ref_name)
    if commit == _UNAVAILABLE:
        return _fallback_build_info(start_path)
    return AddonBuildInfo(
        branch=branch or _UNAVAILABLE,
        commit=commit,
        source="checkout",
    )
