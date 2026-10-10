"""Версия `mpmath` объявлена одна, и стоит именно она.

Интервальный фильтр знака (`mpmath.iv`) входит в ответ ядра, и детерминизм доказан для `mpmath` 1.3.0 (`sympy` 1.14.0 допускает любой `mpmath`
в диапазоне `>=1.1.0,<1.4`, поэтому без своей строки версия плавала бы). Пул доменов сверяет версию воркера с родителем
(`cftuv/envelope_domain_pool.py`), но не с объявлением; здесь объявление сверяется с установкой.

Где лежит объявление. В дереве (`kernel/`, извлечённое ядро) это `pyproject.toml` рядом с `tests/`. В герметичном прогоне CI
(`envelope-kernel.yml`, kernel-hermetic) тесты копируются отдельно от `pyproject.toml`, и объявлением служат метаданные установленного колеса
(`Requires-Dist`): они и есть то, что `pip` разрешал при установке.
"""

from importlib.metadata import PackageNotFoundError, requires
from pathlib import Path
import re

import mpmath
import sympy

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"
DISTRIBUTION = "cftuv-envelope-core"


def pinned_versions(text: str) -> dict[str, str]:
    """`{имя: версия}` строк вида `"имя==версия"` в тексте манифеста."""

    return dict(re.findall(r'"([A-Za-z0-9_.-]+)==([^"\s]+)"', text))


def declared_pins() -> dict[str, str]:
    """Прибитые версии из `pyproject.toml` ядра, а без него (колесо поставлено отдельно от дерева) — из метаданных колеса."""

    if PYPROJECT.is_file():
        return pinned_versions(PYPROJECT.read_text(encoding="utf-8"))
    try:
        requirements = requires(DISTRIBUTION) or []
    except PackageNotFoundError as error:
        raise AssertionError(
            f"DEPENDENCY_DECLARATION_MISSING: нет ни {PYPROJECT}, ни установленного колеса {DISTRIBUTION}; объявление версий читать неоткуда"
        ) from error
    return pinned_versions(" ".join(f'"{requirement}"' for requirement in requirements))


def test_the_pin_reader_finds_exact_pins_and_ignores_ranges():
    text = 'dependencies = ["sympy==1.14.0", "mpmath==1.3.0", "numpy>=1.2"]'

    assert pinned_versions(text) == {"sympy": "1.14.0", "mpmath": "1.3.0"}
    assert pinned_versions('dependencies = ["mpmath>=1.1"]') == {}


def test_the_kernel_declares_one_exact_mpmath_pin():
    pins = declared_pins()

    assert re.fullmatch(r"\d+\.\d+\.\d+", pins.get("mpmath", "")), f"MPMATH_PIN_MISSING: kernel/pyproject.toml не прибивает mpmath: {pins}"


def test_the_installed_mpmath_is_the_declared_one():
    pins = declared_pins()

    assert mpmath.__version__ == pins["mpmath"], (
        f"MPMATH_PIN_MISMATCH: установлен mpmath {mpmath.__version__}, kernel/pyproject.toml прибивает {pins['mpmath']}; "
        "детерминизм ответа доказан только для прибитой версии"
    )


def test_the_installed_sympy_is_the_declared_one():
    pins = declared_pins()

    assert sympy.__version__ == pins["sympy"], f"SYMPY_PIN_MISMATCH: установлен sympy {sympy.__version__}, объявлен {pins['sympy']}"
