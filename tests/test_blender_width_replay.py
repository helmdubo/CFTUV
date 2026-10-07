"""`tools/blender_width_replay.py`: чистая часть воспроизведения перетаскивания (путь ширины и сводки цен кадров), без Blender.

Сам прогон идёт в Blender headless на мешах сцены (`E:/testScene.blend`) и печатает цены кадров, сертификат, отклонение превью от точного, отмену и
подтверждение; здесь держится то, что решает, ЧТО измерено: путь ширины (вверх, назад, ниже базы) и порядок сводок. Сам инструмент и смок
`tests/blender/test_envelope_decal_width_live.py` зовут те же функции, что и модальный оператор.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[1] / "tools" / "blender_width_replay.py"


@pytest.fixture(scope="module")
def replay():
    spec = importlib.util.spec_from_file_location("blender_width_replay_under_test", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # `bpy` и `bmesh` под тестом — заглушки conftest; прогон стартует только из `__main__`
    return module


def test_the_drag_path_goes_up_by_the_share_then_back_below_the_base(replay):
    path = replay._path(0.25, 0.30, 240)

    assert len(path) == 240
    assert path[0] > 0.25 and max(path) == pytest.approx(0.25 * 1.30)
    assert path.index(max(path)) == 119, "the peak ends the first half"
    assert path[-1] == pytest.approx(0.25 * (1.0 - 0.30 / 3.0)) and min(path) == path[-1]
    steps = [b - a for a, b in zip(path, path[1:])]
    assert all(step > 0 for step in steps[:118]) and all(step < 0 for step in steps[119:])


def test_the_timing_summary_is_median_95th_percentile_and_maximum(replay):
    values = [float(item) for item in range(1, 101)]

    found = replay._stats(values)

    assert found == {"n": 100, "p50": 50.5, "p95": 95.0, "max": 100.0}
    assert replay._stats([]) == {}
    assert replay._percentile([3.0, 1.0, 2.0], 0.5) == 2.0 and replay._percentile([], 0.5) is None
