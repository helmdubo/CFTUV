from __future__ import annotations

import os
from pathlib import Path

import pytest

from cftuv_envelope.exact_sqrt_sum import set_canonical_audit
from ec0_adapter import load_projection

# Набор тестов ядра всегда гоняется с полным аудитом каноники сумм корней:
# каждая величина, входящая в машину времён, проверяется на бесквадратность
# независимо от памяти канонизации (`require_canonical`). Без аудита
# `times_are_equal` и `compare_times == 0` расходятся на неканоническом входе
# молча. `CFTUV_CANONICAL_AUDIT=0` выключает его только для замера цены самого
# аудита; продуктовый путь аудит не включает никогда.
set_canonical_audit(os.environ.get("CFTUV_CANONICAL_AUDIT", "1") != "0")

# `CFTUV_SYMBOLIC_BACKEND=SHADOW|NATIVE_EXACT` гоняет ВЕСЬ набор ядра под выбранным символьным
# бэкендом (SHADOW с политикой RAISE: любое расхождение значений роняет тест, на котором случилось).
# Без переменной действует умолчание `SYMPY` (`reference/symbolic_backend.py`).
_SYMBOLIC_BACKEND = os.environ.get("CFTUV_SYMBOLIC_BACKEND", "")
if _SYMBOLIC_BACKEND:
    from cftuv_envelope.reference import symbolic_backend as _symbolic_backend

    _symbolic_backend.set_backend_mode(_symbolic_backend.SymbolicBackendV1(_SYMBOLIC_BACKEND))


def pytest_sessionfinish(session, exitstatus):
    """Под выбранным бэкендом сессия печатает свод счётчиков: сколько сверено и чем решено."""

    if not _SYMBOLIC_BACKEND:
        return
    from cftuv_envelope.reference import planar_types

    counts = dict(sorted(_symbolic_backend.BACKEND_COUNTS.items()))
    print(
        f"\nSYMBOLIC_BACKEND_SUMMARY {_SYMBOLIC_BACKEND} "
        f"text_differences={len(planar_types.TEXT_DIFFERENCES)} {counts}"
    )
    for legacy, native in planar_types.SINGLE_TERM_TEXT_DIFFERENCES[:4]:
        print(f"SINGLE_TERM_TEXT_DIFFERENCE legacy={legacy[:300]} native={native[:300]}")


@pytest.fixture(autouse=True)
def _fresh_developable_chart_memory():
    """Память построителя карты развёртки (по входам) не переживает тест: он подменяет внутренности построителя."""

    from cftuv_envelope._band_chart import clear_band_chart_memory
    from cftuv_envelope._developable import clear_developable_chart_memory

    clear_developable_chart_memory()
    clear_band_chart_memory()
    yield


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "session_a_v5"
CASE_PATHS = tuple(sorted((FIXTURE_ROOT / "cases").glob("*.json")))


@pytest.fixture(scope="session")
def fixture_root() -> Path:
    return FIXTURE_ROOT


@pytest.fixture(scope="session")
def projections():
    return tuple(load_projection(path) for path in CASE_PATHS)

