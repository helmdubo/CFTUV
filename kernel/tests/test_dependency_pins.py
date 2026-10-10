"""Версия `mpmath` объявлена одна, и стоит именно она.

Интервальный фильтр знака (`mpmath.iv`) входит в ответ ядра, и детерминизм доказан для `mpmath` 1.3.0 (`sympy` 1.14.0 допускает любой `mpmath`
в диапазоне `>=1.1.0,<1.4`, поэтому без своей строки версия плавала бы). Пул доменов сверяет версию воркера с родителем
(`cftuv/envelope_domain_pool.py`), но не с объявлением; здесь объявление сверяется с установкой.
"""

from pathlib import Path
import re

import mpmath
import sympy

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def pinned_versions(text: str) -> dict[str, str]:
    """`{имя: версия}` строк вида `"имя==версия"` в тексте манифеста."""

    return dict(re.findall(r'"([A-Za-z0-9_.-]+)==([^"\s]+)"', text))


def test_the_pin_reader_finds_exact_pins_and_ignores_ranges():
    text = 'dependencies = ["sympy==1.14.0", "mpmath==1.3.0", "numpy>=1.2"]'

    assert pinned_versions(text) == {"sympy": "1.14.0", "mpmath": "1.3.0"}
    assert pinned_versions('dependencies = ["mpmath>=1.1"]') == {}


def test_the_kernel_declares_one_exact_mpmath_pin():
    pins = pinned_versions(PYPROJECT.read_text(encoding="utf-8"))

    assert re.fullmatch(r"\d+\.\d+\.\d+", pins.get("mpmath", "")), f"MPMATH_PIN_MISSING: kernel/pyproject.toml не прибивает mpmath: {pins}"


def test_the_installed_mpmath_is_the_declared_one():
    pins = pinned_versions(PYPROJECT.read_text(encoding="utf-8"))

    assert mpmath.__version__ == pins["mpmath"], (
        f"MPMATH_PIN_MISMATCH: установлен mpmath {mpmath.__version__}, kernel/pyproject.toml прибивает {pins['mpmath']}; "
        "детерминизм ответа доказан только для прибитой версии"
    )


def test_the_installed_sympy_is_the_declared_one():
    pins = pinned_versions(PYPROJECT.read_text(encoding="utf-8"))

    assert sympy.__version__ == pins["sympy"], f"SYMPY_PIN_MISMATCH: установлен sympy {sympy.__version__}, объявлен {pins['sympy']}"
