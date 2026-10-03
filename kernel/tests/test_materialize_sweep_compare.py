"""Стандартное сравнение свипа материализатора: отсутствующее число равно нулю.

Новый нулевой счётчик в `topology_counters` (числа потоков JOIN-FLOW) делал прежнюю запись и новую «разными»
и ломал штатный `compare` на доменах, где число не менялось. Нулевой ключ и отсутствующий — одно и то же;
настоящее расхождение (ненулевое число) по-прежнему ловится.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

SWEEP = Path(__file__).resolve().parents[2] / "artifacts" / "materialize_sweep" / "sweep.py"


def _sweep():
    sys.path.insert(0, str(SWEEP.parent))
    spec = importlib.util.spec_from_file_location("materialize_sweep_under_test", SWEEP)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _record(topology_counters):
    row = {
        "patch_id": 1,
        "density": 1,
        "prepare_outcome": "EXACT",
        "coverage_outcome": "EXACT",
        "materialization": "MATERIALIZED",
        "detail": "",
        "content_digest": "c",
        "offset_normals_digest": "o",
        "semantic_digest": "s",
        "counters": {"MATERIALIZE_FACES_IN": 3},
        "diagnostics": [],
        "chart": None,
        "planarity": None,
        "topology_counters": topology_counters,
    }
    return {"runs": {"1": {"domains": {"1": row}}}}


def _compare(tmp_path, left, right):
    sweep = _sweep()
    paths = []
    for name, record in (("left.json", left), ("right.json", right)):
        path = tmp_path / name
        path.write_text(json.dumps(record), encoding="utf-8")
        paths.append(path)
    return sweep.compare(paths)


def test_a_new_zero_counter_does_not_make_the_runs_different(tmp_path):
    base = _record({"MATERIALIZE_FACES_EMITTED": 4})
    new = _record({"MATERIALIZE_FACES_EMITTED": 4, "STATION_FLOW_CYCLES_OPENED": 0, "MATERIALIZE_CLIP_FLOW_FREE_CUT_EDGES": 0})
    assert _compare(tmp_path, base, new) == 0
    assert _compare(tmp_path, new, base) == 0


def test_a_changed_or_new_nonzero_counter_still_makes_the_runs_different(tmp_path):
    base = _record({"MATERIALIZE_FACES_EMITTED": 4})
    assert _compare(tmp_path, base, _record({"MATERIALIZE_FACES_EMITTED": 5})) == 1
    assert _compare(tmp_path, base, _record({"MATERIALIZE_FACES_EMITTED": 4, "STATION_FLOW_CYCLES_OPENED": 1})) == 1
