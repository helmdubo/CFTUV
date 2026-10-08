"""Репозиторные свипы закона митры: прежние записи и отрицательные контроли."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SWEEP_DIR = ROOT / "artifacts" / "materialize_sweep"
EXPECTED_CHANGE = ROOT / "kernel" / "fixtures" / "expected_change"

# --------------------------------------------------------------------------
# Настоящие записи ворот
# --------------------------------------------------------------------------


def _load_module(name: str, path: Path):
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def sweep():
    return _load_module("materialize_sweep_under_fold_test", SWEEP_DIR / "sweep.py")


def _record(name: str) -> dict:
    return json.loads((EXPECTED_CHANGE / name).read_text(encoding="utf-8"))


def test_real_records_the_sweep_moves_domain_89_alone_by_the_fold_miter_spec(sweep):
    ec = sweep.expected_change
    spec = ec.load_spec("fold_miter", "sweep", sweep.VOCABULARY)
    report = ec.evaluate(
        [_record("sweep_fold_before.json"), _record("sweep_fold_after.json")],
        ["base", "new"],
        spec,
        sweep.pair_views(False),
        sweep.VOCABULARY,
        False,
    )
    assert report.problems == []
    assert report.changed_domains == ["89"]
    assert ec.verdict_line(report) == "EXPECTED-CHANGE (spec fold_miter): 1 domains"
    # Без спецификации та же пара — расхождение именно на домене 89, и ни на каком другом из пяти.
    bare = ec.evaluate(
        [_record("sweep_fold_before.json"), _record("sweep_fold_after.json")],
        ["base", "new"], None, sweep.pair_views(False), sweep.VOCABULARY, False,
    )
    assert bare.exit_code == 1 and {line.split(" patch")[1].split(":")[0] for line in bare.problems} == {"89"}


def test_real_records_the_gate_moves_domain_89_alone_by_the_fold_miter_gate_spec(sweep):
    gate, ec = sweep.gate, sweep.expected_change
    spec = ec.load_spec("fold_miter_gate", "gate", gate.VOCABULARY)
    result = gate.compare_records(_record("gate_fold_before.json"), _record("gate_fold_after.json"), spec)
    assert result["report"].problems == [] and result["report"].changed_domains == ["89"]
    assert ec.verdict_line(result["report"]) == "EXPECTED-CHANGE (spec fold_miter_gate): 1 domains"
    # Контроль: спецификация без домена 89 (чужой список) валит ровно на нём.
    raw = json.loads((SWEEP_DIR / "specs" / "fold_miter_gate.json").read_text(encoding="utf-8"))
    raw["domains"] = {"explicit": [6]}
    wrong = gate.compare_records(
        _record("gate_fold_before.json"), _record("gate_fold_after.json"), ec.parse_spec(raw, "gate", gate.VOCABULARY), False
    )
    assert wrong["report"].exit_code == 1
