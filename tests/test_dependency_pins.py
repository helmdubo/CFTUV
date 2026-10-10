"""Версии, от которых зависит ответ ядра, объявлены в одном значении во всех местах объявления.

Ядро объявляет их в `kernel/pyproject.toml` (колесо `cftuv-envelope-core`), CI хоста ставит `tests/requirements.txt`. Расхождение двух файлов
означало бы, что CI проверяет ядро на версии, которой колесо не просит.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PINNED = ("sympy", "mpmath")


def _kernel_pins() -> dict[str, str]:
    text = (ROOT / "kernel" / "pyproject.toml").read_text(encoding="utf-8")
    return dict(re.findall(r'"([A-Za-z0-9_.-]+)==([^"\s]+)"', text))


def _ci_pins() -> dict[str, str]:
    pins = {}
    for line in (ROOT / "tests" / "requirements.txt").read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"([A-Za-z0-9_.-]+)==(\S+)", line.split("#", 1)[0].strip())
        if match is not None:
            pins[match.group(1)] = match.group(2)
    return pins


def test_the_ci_requirements_pin_what_the_kernel_pins():
    kernel, ci = _kernel_pins(), _ci_pins()

    for name in PINNED:
        assert name in kernel, f"DEPENDENCY_PIN_MISSING: kernel/pyproject.toml не прибивает {name}"
        assert ci.get(name) == kernel[name], (
            f"DEPENDENCY_PIN_MISMATCH: {name}: tests/requirements.txt {ci.get(name)!r}, kernel/pyproject.toml {kernel[name]!r}"
        )
