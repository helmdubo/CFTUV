from __future__ import annotations

from pathlib import Path

import pytest

from ec0_adapter import load_projection


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "session_a_v5"
CASE_PATHS = tuple(sorted((FIXTURE_ROOT / "cases").glob("*.json")))


@pytest.fixture(scope="session")
def fixture_root() -> Path:
    return FIXTURE_ROOT


@pytest.fixture(scope="session")
def projections():
    return tuple(load_projection(path) for path in CASE_PATHS)

