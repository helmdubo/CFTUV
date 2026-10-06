"""`tools/blender_native_ab.py`: чистая часть A/B (строка домена, сравнение, сводка, таблица, отчёт без нативного ядра).

Сам прогон идёт в Blender (`tools/blender_native_ab.py`, без нативного ядра — статус `UNAVAILABLE` и код возврата 0); здесь держится то, что
решает вердикт: что считается ответом, что ценой, что только таблицей, и что без нативного ядра отчёт называет статус, а не падает.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

TOOL = Path(__file__).resolve().parents[1] / "tools" / "blender_native_ab.py"


@pytest.fixture(scope="module")
def ab():
    spec = importlib.util.spec_from_file_location("blender_native_ab_under_test", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # `bpy` импортируется только внутри функций Blender: модуль грузится без него
    return module


def _result(patch, *, outcome="MATERIALIZED", digest="d", semantic="s", price=10, seconds=1.0, placement="parent", record=None):
    batch = None if semantic is None else SimpleNamespace(semantic_digest=SimpleNamespace(value=semantic))
    return SimpleNamespace(
        patch_id=patch,
        outcome=outcome,
        content_digest=digest,
        batch=batch,
        counters=(("EXACT_WORK_SPENT", price), ("EXACT_WORK_GCD", 3), ("OTHER", 99)),
        seconds=seconds,
        placement=placement,
        step_path="FAST_HIT",
        backend_record=record,
    )


def _record(ran="native", *outcomes):
    calls = {"native": (3, 0), "python": (0, 3), "mixed": (2, 1)}[ran]
    return SimpleNamespace(ran=ran, native_calls=calls[0], python_calls=calls[1], outcomes=tuple(outcomes))


def test_the_default_cases_are_the_22_field_cases(ab):
    cases = ab.FIELD_CASES.split(",")
    assert len(cases) == 22
    assert all(len(item.split(":")) == 4 for item in cases)
    assert "building:0.2239:2:42" in cases and "wall_noise_top:0.25:2:20" in cases


def test_a_domain_row_separates_the_answer_the_price_and_the_run_marks(ab):
    row = ab.domain_row(_result(7, record=_record("mixed", "NATIVE_PORT_UNSUPPORTED")))

    assert (row["patch_id"], row["outcome"], row["content_digest"], row["semantic_digest"]) == (7, "MATERIALIZED", "d", "s")
    assert row["prices"] == {"EXACT_WORK_GCD": 3, "EXACT_WORK_SPENT": 10}  # только статьи цены, без прочих счётчиков
    assert (row["ran"], row["native_calls"], row["python_calls"], row["fallbacks"]) == ("mixed", 2, 1, ["NATIVE_PORT_UNSUPPORTED"])
    refusal = ab.domain_row(_result(3, outcome="REFUSED", semantic=None))
    assert refusal["semantic_digest"] == "" and refusal["ran"] == "python" and refusal["fallbacks"] == []


def test_equal_runs_have_no_differences_and_each_kind_of_difference_is_named(ab):
    rows = [ab.domain_row(_result(patch)) for patch in range(3)]
    assert ab.compare_domains(rows, [dict(item) for item in rows]) == {"answer": [], "price": [], "missing": []}

    changed = [dict(item) for item in rows]
    changed[0]["outcome"] = "REFUSED"
    changed[1]["content_digest"] = "other"
    changed[2]["prices"] = {**rows[2]["prices"], "EXACT_WORK_SPENT": 11}
    found = ab.compare_domains(rows, changed)
    assert [line.split(":")[0] for line in found["answer"]] == ["patch 0", "patch 1"]
    assert "outcome" in found["answer"][0] and "content_digest" in found["answer"][1]
    assert found["price"] == ["patch 2: EXACT_WORK_SPENT"] and found["missing"] == []

    lost = ab.compare_domains(rows, changed[:2])
    assert lost["missing"] == ["patch 2: only in python"]


def test_a_width_summary_counts_the_runner_skips_cached_domains_and_names_the_fallbacks(ab):
    python = [ab.domain_row(_result(patch, seconds=2.0)) for patch in range(4)]
    native = [
        ab.domain_row(_result(0, seconds=0.5, record=_record("native"))),
        ab.domain_row(_result(1, seconds=0.5, record=_record("native"))),
        ab.domain_row(_result(2, seconds=2.0, record=_record("python", "NATIVE_PORT_STALE"))),
        ab.domain_row(_result(3, seconds=9.0, placement="cache", record=_record("native"))),
    ]
    summary = ab.summarize_width(0.25, python, native)

    assert summary["domains"] == 4 and not ab.has_differences(summary)
    assert summary["ran"] == {"native": 2, "python": 1, "mixed": 0}
    assert summary["fallbacks"] == {"NATIVE_PORT_STALE": [2]}
    assert (summary["python_seconds"], summary["native_seconds"], summary["speedup"]) == (8.0, 3.0, 2.67)


def test_the_table_has_one_row_per_width_and_names_a_failed_case(ab):
    python = [ab.domain_row(_result(patch, seconds=2.0)) for patch in range(2)]
    native = [ab.domain_row(_result(patch, seconds=1.0, record=_record("native"))) for patch in range(2)]
    case = {"case": "building:0.25:2:20", "widths": [ab.summarize_width(0.25, python, native), ab.summarize_width(0.26, python, native)]}
    broken = {"case": "sagging_wall:0.25:2:20", "failure": "Traceback...\nKeyError: 'nope'"}
    table = ab.format_table({"cases": [case, broken]})
    lines = table.splitlines()

    assert lines[0].split()[:3] == ["case", "width", "doms"]
    assert set(lines[1]) <= {"-", " "}
    assert len(lines) == 2 + 2 + 1
    assert "0.25" in lines[2] and "2/0/0" in lines[2] and "2.00" in lines[2]
    assert "FAILED: KeyError: 'nope'" in lines[-1]
    totals = ab.totals_of([case])
    assert (totals["cases"], totals["domains"], totals["native"], totals["answer"], totals["price"]) == (1, 4, 4, 0, 0)
    assert "NATIVE_AB_OK cases=1 domains=4 answer_differences=0 price_differences=0" in ab.final_line({"status": "OK", "totals": totals})


def test_without_the_native_kernel_the_report_names_the_status_and_is_not_a_failure(ab):
    status = {"coverage": "unavailable", "clip": "unavailable", "version": "", "detail": "ModuleNotFoundError: No module named 'cftuv_native'"}
    report = ab.unavailable_report(status, "C:/tree", ["building:0.25:2:20"])

    assert report["status"] == "UNAVAILABLE" and report["cases"] == [] and report["planned"] == ["building:0.25:2:20"]
    line = ab.final_line(report)
    assert line.startswith("NATIVE_AB_UNAVAILABLE coverage=unavailable clip=unavailable detail=")
    assert "cftuv_native" in line
