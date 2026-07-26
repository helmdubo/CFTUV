"""Реэкспорт корпуса волнового фронта. Сам корпус живёт в `kernel/tests/`.

Причина переноса названа там же: job `kernel-extraction-readiness`
(`.github/workflows/envelope-kernel.yml:56`) копирует один каталог `kernel/` в
пустой репозиторий и запускает тесты уже в нём. Пока корпус лежал в `tools/`,
кернельный тест не мог его импортировать после извлечения.

Здесь не копия, а именно реэкспорт: две копии корпуса разошлись бы, и тогда
стенд и тест мерили бы разные фигуры, называя их одинаково.
"""

from __future__ import annotations

from pathlib import Path
import sys


_CASES = Path(__file__).resolve().parents[1] / "kernel" / "tests"
if str(_CASES) not in sys.path:
    sys.path.insert(0, str(_CASES))

from wavefront_cases import (  # noqa: E402
    FIELD_DENOMINATOR,
    FIELD_FIXTURE,
    FIELD_NUMERATORS,
    axis_rectangle,
    axis_square,
    comb,
    diamond,
    ell,
    field_fixture_digest,
    field_fixture_exists,
    field_polygon,
    holes_grid,
    loop_points,
    named_corpus,
    right_triangle,
    star,
)

__all__ = [
    "FIELD_DENOMINATOR",
    "FIELD_FIXTURE",
    "FIELD_NUMERATORS",
    "axis_rectangle",
    "axis_square",
    "comb",
    "diamond",
    "ell",
    "field_fixture_digest",
    "field_fixture_exists",
    "field_polygon",
    "holes_grid",
    "loop_points",
    "named_corpus",
    "right_triangle",
    "star",
]
