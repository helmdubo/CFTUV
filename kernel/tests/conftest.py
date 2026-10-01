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


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "session_a_v5"
CASE_PATHS = tuple(sorted((FIXTURE_ROOT / "cases").glob("*.json")))


@pytest.fixture(scope="session")
def fixture_root() -> Path:
    return FIXTURE_ROOT


@pytest.fixture(scope="session")
def projections():
    return tuple(load_projection(path) for path in CASE_PATHS)

