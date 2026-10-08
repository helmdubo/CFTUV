"""Пути тестового набора отдельно от исполняемого пакета и проверка колеса CI."""
from __future__ import annotations

from importlib.metadata import distribution
from pathlib import Path, PurePosixPath
import sys
import sysconfig

import cftuv_envelope


KERNEL_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = Path(cftuv_envelope.__file__).resolve().parent


def kernel_reference(reference: str) -> Path:
    """Логическая ссылка `kernel/tests/...` остаётся той же после извлечения."""
    relative = PurePosixPath(reference)
    assert relative.parts[:1] == ("kernel",) and ".." not in relative.parts, reference
    return KERNEL_ROOT.joinpath(*relative.parts[1:])


class InstalledKernelGuard:
    """Проверяет текущий граф модулей; повторный путь не требует файлового resolve."""

    def __init__(self, expected_root: Path):
        self.expected_root = expected_root.resolve()
        self._checked_paths: set[str] = set()
        self.checks = 0
        self.path_resolutions = 0

    def check(self, modules=None):
        modules = sys.modules if modules is None else modules
        self.checks += 1
        for name, module in tuple(modules.items()):
            if name != "cftuv_envelope" and not name.startswith("cftuv_envelope."):
                continue
            assert module is not None, f"KERNEL_WHEEL_ORIGIN_INVALID: {name} is None"
            paths = (getattr(module, "__file__", None), getattr(getattr(module, "__spec__", None), "origin", None))
            assert all(paths), f"KERNEL_WHEEL_ORIGIN_INVALID: {name} has no file/spec origin"
            for raw in paths:
                if raw in self._checked_paths:
                    continue
                path = Path(raw).resolve()
                self.path_resolutions += 1
                assert path.is_relative_to(self.expected_root) and path.is_file(), (
                    f"KERNEL_WHEEL_ORIGIN_INVALID: {name}: {raw}; expected {self.expected_root}"
                )
                self._checked_paths.add(raw)


def installed_kernel_guard() -> InstalledKernelGuard:
    """Метаданные именно установленного wheel, внутри текущего virtualenv."""
    root = Path(distribution("cftuv-envelope-core").locate_file("cftuv_envelope")).resolve()
    purelib = Path(sysconfig.get_path("purelib")).resolve()
    assert sys.prefix != sys.base_prefix, "KERNEL_WHEEL_ORIGIN_INVALID: CI must use a virtualenv"
    assert root.is_relative_to(purelib), f"KERNEL_WHEEL_ORIGIN_INVALID: distribution outside {purelib}: {root}"
    guard = InstalledKernelGuard(root)
    guard.check()
    return guard
