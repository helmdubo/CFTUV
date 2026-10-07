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


def _result(patch, *, outcome="MATERIALIZED", digest="d", semantic="s", price=10, seconds=1.0, placement="parent", record=None, step_path="FAST_HIT"):
    batch = None if semantic is None else SimpleNamespace(semantic_digest=SimpleNamespace(value=semantic))
    return SimpleNamespace(
        patch_id=patch,
        outcome=outcome,
        content_digest=digest,
        batch=batch,
        counters=(("EXACT_WORK_SPENT", price), ("EXACT_WORK_GCD", 3), ("OTHER", 99)),
        seconds=seconds,
        placement=placement,
        step_path=step_path,
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


def test_the_native_directory_is_prepended_so_an_installed_wheel_never_shadows_it(ab):
    root = Path("/repo")
    tree = [str(root), str(root / "kernel" / "src")]
    current = ["/site-packages-with-the-installed-wheel", str(root), "/other", "/native"]

    found = ab.path_with_tree(current, root, "/native")

    assert found[0] == "/native" and found[1:3] == tree
    assert found.index("/native") < found.index("/site-packages-with-the-installed-wheel")
    assert [found.count(item) for item in ("/native", *tree)] == [1, 1, 1]
    assert found[3:] == ["/site-packages-with-the-installed-wheel", "/other"]  # прежний порядок остального сохранён
    # Без названного каталога порядок прежний: хост и ядро дерева впереди, остальное как было.
    assert ab.path_with_tree(current, root)[:2] == tree and ab.path_with_tree(current, root)[2:] == [current[0], "/other", "/native"]


# --------------------------------------------------------------------------
# Строгий режим: приёмка не проходит «на нуле» (расширения нет, порт устарел, сборка не та, откат Python, Rust не вызван)
# --------------------------------------------------------------------------

NATIVE_OK = {"coverage": "available", "clip": "available", "version": "0.1.0", "detail": "", "build_id": "abc123"}


def _not_reached():
    """Запись домена, который не позвал ни одной операции (`NATIVE_NOT_REACHED`)."""

    return SimpleNamespace(ran="python", native_calls=0, python_calls=0, outcomes=("NATIVE_NOT_REACHED",))


def _width(ab, rows, *, python_wall=2.0, native_wall=1.0, pack=0.5, step=0, width=0.25):
    shared = lambda item: {key: value for key, value in item.get("result", {}).items() if key in ("outcome", "semantic")}  # noqa: E731 - the python run answers alike
    python = [ab.domain_row(_result(item["patch"], seconds=2.0, **shared(item))) for item in rows]
    native = [ab.domain_row(_result(item["patch"], seconds=item.get("seconds", 1.0), **item.get("result", {}))) for item in rows]
    summary = ab.summarize_width(
        width, python, native, {"wall_seconds": python_wall, "pack_seconds": pack}, {"wall_seconds": native_wall, "pack_seconds": pack}
    )
    summary["step"] = step
    return summary


def _native_rows(count=3, **result):
    return [{"patch": patch, "result": {"record": _record("native"), **result}} for patch in range(count)]


def _report(ab, cases, status=NATIVE_OK, **extra):
    return {"status": "OK", "native_status": dict(status), "root": "tree", "cases": cases, "totals": ab.totals_of(cases), **extra}


def _case(ab, rows, name="building:0.25:2:20"):
    return {"case": name, "widths": [_width(ab, rows)]}


def test_strict_is_the_default_and_the_diagnostic_mode_is_explicit(ab):
    assert ab.parse_arguments([]).strict is True and ab.parse_arguments(["--strict"]).strict is True
    assert ab.parse_arguments(["--diagnostic"]).strict is False
    assert ab.parse_arguments([]).expect_build_id is None
    assert ab.parse_arguments(["--expect-build-id", "ABC"]).expect_build_id == "ABC"
    with pytest.raises(SystemExit):
        ab.parse_arguments(["--strict", "--diagnostic"])  # две приёмки одним запуском: не гадаем, какая главнее


def test_each_way_the_native_kernel_is_not_what_was_asked_for_has_its_own_name(ab):
    assert ab.port_violations(NATIVE_OK, "abc123") == [] and ab.port_violations(NATIVE_OK, None) == []
    assert ab.port_violations(NATIVE_OK, "ABC123") == [], "ids are hex: the case of the letters is not a difference"

    missing = {"coverage": "unavailable", "clip": "unavailable", "version": "", "detail": "ModuleNotFoundError: cftuv_native", "build_id": ""}
    found = ab.port_violations(missing, None)
    assert [line.split(":")[0] for line in found] == ["STRICT_PORT_UNAVAILABLE"] * 2 and "ModuleNotFoundError" in found[0]

    stale = ab.port_violations({**NATIVE_OK, "clip": "stale(materialize/clip.py)"}, None)
    assert len(stale) == 1 and stale[0].startswith("STRICT_PORT_STALE: clip=stale(materialize/clip.py)")
    assert ab.port_violations({**NATIVE_OK, "coverage": "unsupported_python"}, None)[0].startswith("STRICT_PORT_NOT_AVAILABLE: coverage=")

    wrong = ab.port_violations(NATIVE_OK, "def456")
    assert wrong == ["STRICT_BUILD_ID_MISMATCH: expected def456, loaded abc123"]
    assert "loaded <none>" in ab.port_violations({**NATIVE_OK, "build_id": ""}, "def456")[0]


def test_a_clean_native_run_passes_and_any_fallback_but_not_reached_refuses(ab):
    clean = _report(ab, [_case(ab, _native_rows())])
    assert ab.strict_violations(clean, "abc123") == []

    stale_domain = _native_rows()
    stale_domain[1] = {"patch": 1, "result": {"record": _record("python", "NATIVE_PORT_STALE")}}
    refused = ab.strict_violations(_report(ab, [_case(ab, stale_domain)]), None)
    assert [line.split(":")[0] for line in refused] == ["STRICT_UNEXPECTED_FALLBACK"]
    assert "NATIVE_PORT_STALE case building:0.25:2:20 width 0.25 patches [1]" in refused[0]

    unsupported = _native_rows()
    unsupported[0] = {"patch": 0, "result": {"record": _record("mixed", "NATIVE_PORT_UNSUPPORTED")}}  # named refusals of the port are fallbacks too
    assert ab.strict_violations(_report(ab, [_case(ab, unsupported)]), None)[0].startswith("STRICT_UNEXPECTED_FALLBACK: NATIVE_PORT_UNSUPPORTED")


def test_not_reached_is_allowed_only_where_the_computation_was_honestly_eliminated(ab):
    rows = [
        {"patch": 0, "result": {"record": _record("native")}},
        {"patch": 1, "result": {"record": _not_reached(), "step_path": "FAST_HIT"}},  # coverage from the width-step template
        {"patch": 2, "result": {"record": _not_reached(), "placement": "cache", "step_path": ""}},  # result of an earlier run
        {"patch": 3, "result": {"record": _not_reached(), "outcome": "REFUSED", "semantic": None, "step_path": ""}},  # refused before the kernel
    ]
    assert ab.strict_violations(_report(ab, [_case(ab, rows)]), None) == []
    width = _width(ab, rows)
    assert width["unexplained_not_reached"] == [] and width["fallbacks"] == {"NATIVE_NOT_REACHED": [1, 3]}

    rows.append({"patch": 4, "result": {"record": _not_reached(), "step_path": "FALLBACK:NO_CERTIFICATE"}})  # a full computation that never reached Rust
    found = ab.strict_violations(_report(ab, [_case(ab, rows)]), None)
    assert [line.split(":")[0] for line in found] == ["STRICT_NOT_REACHED_UNEXPLAINED"] and "patches [4]" in found[0]


def test_a_case_that_never_called_native_checked_nothing_about_the_port(ab):
    python_only = [{"patch": patch, "result": {"record": _record("python")}} for patch in range(3)]  # no fallback is named: Rust was simply never asked
    found = ab.strict_violations(_report(ab, [_case(ab, python_only), _case(ab, _native_rows(), "walls.001:0.25:2:20")]), None)

    assert [line.split(":")[0] for line in found] == ["STRICT_CASE_NEVER_NATIVE"] and "building:0.25:2:20" in found[0]


def test_the_verdict_names_a_status_and_a_code_and_a_difference_outranks_a_refusal(ab):
    status = {"coverage": "unavailable", "clip": "unavailable", "version": "", "detail": "ModuleNotFoundError: cftuv_native", "build_id": ""}
    diagnostic = ab.finalize(ab.unavailable_report(status, "tree", ["building:0.25:2:20"]), strict=False)
    assert (diagnostic["status"], ab.exit_code(diagnostic), diagnostic["strict"]["violations"]) == ("UNAVAILABLE", 0, [])

    strict = ab.finalize(ab.unavailable_report(status, "tree", ["building:0.25:2:20"]), strict=True)
    assert (strict["status"], ab.exit_code(strict)) == ("REFUSED", 2)
    line = ab.final_line(strict)
    assert line.startswith("NATIVE_AB_REFUSED coverage=unavailable") and "violations=2 first=STRICT_PORT_UNAVAILABLE" in line

    ok = ab.finalize(_report(ab, [_case(ab, _native_rows())]), strict=True, expected_build_id="abc123")
    assert (ok["status"], ab.exit_code(ok), ok["strict"]["build_id_checked"]) == ("OK", 0, True)
    assert "NATIVE_AB_OK" in ab.final_line(ok) and "strict=on violations=0" in ab.final_line(ok) and "native_calls=9" in ab.final_line(ok)

    stale = ab.finalize(_report(ab, [_case(ab, _native_rows())], status={**NATIVE_OK, "clip": "stale(a.py)"}), strict=True)
    assert (stale["status"], ab.exit_code(stale)) == ("REFUSED", 2)

    different = _native_rows()
    different[0] = {"patch": 0, "result": {"record": _record("native"), "digest": "other"}}
    both = ab.finalize(_report(ab, [_case(ab, different)], status={**NATIVE_OK, "clip": "stale(a.py)"}), strict=True)
    assert (both["status"], ab.exit_code(both)) == ("FAILED", 1) and both["strict"]["violations"], "the answer is the stronger verdict, the refusal stays named"

    nothing = ab.finalize(_report(ab, []), strict=True)
    assert nothing["strict"]["violations"] == ["STRICT_NO_CASES: no case was run"] and nothing["status"] == "REFUSED"

    skipped = ab.unavailable_report({**NATIVE_OK, "clip": "stale(a.py)"}, "tree", ["building:0.25:2:20"])
    skipped["skipped"] = "STRICT_PREFLIGHT"
    refused = ab.finalize(skipped, strict=True)
    assert [line.split(":")[0] for line in refused["strict"]["violations"]] == ["STRICT_PORT_STALE"]  # a run that never started is not also "no cases"


def test_the_expected_build_id_is_the_tree_when_strict_and_unchecked_when_diagnostic(ab):
    seen = []

    def tree(root):
        seen.append(root)
        return "tree-id"

    assert ab.resolve_expected_build_id(None, True, Path("/repo"), tree) == ("tree-id", None) and seen == [Path("/repo")]
    assert ab.resolve_expected_build_id(None, False, Path("/repo"), tree) == (None, None) and len(seen) == 1
    assert ab.resolve_expected_build_id("none", True, Path("/repo"), tree) == (None, None)
    assert ab.resolve_expected_build_id(" ABC123 ", True, Path("/repo"), tree) == ("abc123", None) and len(seen) == 1

    def broken(root):
        raise FileNotFoundError("no native workspace")

    expected, error = ab.resolve_expected_build_id("tree", True, Path("/repo"), broken)
    assert expected is None and error.startswith("STRICT_BUILD_ID_EXPECTATION_UNKNOWN") and "FileNotFoundError" in error
    refused = ab.finalize(ab.unavailable_report(NATIVE_OK, "tree", []), strict=True, expectation_error=error)
    assert refused["status"] == "REFUSED", "an expectation that cannot be computed is a named refusal, not a silent skip of the check"


def test_interaction_latency_is_a_wall_clock_metric_apart_from_the_sum_of_domain_seconds(ab):
    width = _width(ab, _native_rows(4), python_wall=3.0, native_wall=1.0, pack=0.5)

    assert (width["python_seconds"], width["native_seconds"], width["speedup"]) == (8.0, 4.0, 2.0)  # sums over domains
    assert (width["python_interaction"], width["native_interaction"], width["wall_speedup"]) == (3.5, 1.5, 2.33)  # what the person waits for
    assert width["native_run"] == {"wall_seconds": 1.0, "pack_seconds": 0.5}
    bare = ab.summarize_width(0.25, [ab.domain_row(_result(0))], [ab.domain_row(_result(0, record=_record("native")))])
    assert (bare["python_interaction"], bare["native_interaction"], bare["wall_speedup"]) == (None, None, None)

    assert ab.percentile([], 0.5) is None and ab.latency_summary([]) == {"n": 0, "p50": None, "p95": None, "p99": None, "max": None}
    hundred = ab.latency_summary(range(1, 101))
    assert (hundred["n"], hundred["p50"], hundred["p95"], hundred["p99"], hundred["max"]) == (100, 50, 95, 99, 100)
    assert ab.latency_summary([5, 1, 3])["p99"] == 5, "below 100 samples p99 is the maximum: `n` travels with the number"


def test_latency_separates_every_step_from_the_warm_ones_and_prints_percentiles(ab):
    cases = [
        {"case": "a", "widths": [_width(ab, _native_rows(2), native_wall=9.0, step=0), _width(ab, _native_rows(2), native_wall=1.0, step=1), _width(ab, _native_rows(2), native_wall=2.0, step=2)]},
    ]
    latency = ab.latency_of(cases)

    assert latency["native"]["all"]["n"] == 3 and latency["native"]["all"]["max"] == 9.5
    assert latency["native"]["warm"]["n"] == 2 and latency["native"]["warm"]["max"] == 2.5 and latency["native"]["warm"]["p50"] == 1.5
    lines = ab.format_latency(latency)
    assert lines[0].startswith("LATENCY python all n=3 p50=") and any(line.startswith("LATENCY native warm n=2 p50=1.500 p95=2.500") for line in lines)
    report = _report(ab, cases, latency=latency)
    assert "warm_p95_python=2.500 warm_p95_native=2.500" in ab.final_line(ab.finalize(report, strict=True, expected_build_id="abc123"))


def test_the_live_mesh_share_is_counted_by_domains_and_by_packed_face_area(ab):
    arrays = SimpleNamespace(
        positions=[(0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0)],
        faces=[(0, 1, 2, 3), (0, 1, 2), (0, 1, 3)],
        face_domain=[7, 9, 9],
    )
    assert ab.polygon_area(arrays.positions, arrays.faces[0]) == 4.0
    assert ab.area_by_domain(arrays) == {7: 4.0, 9: 4.0}

    rows = [
        ab.domain_row(_result(7, record=_record("native"))),
        ab.domain_row(_result(9, record=_record("python"))),
        ab.domain_row(_result(11, placement="cache", record=_record("native"))),
        ab.domain_row(_result(12, outcome="REFUSED", semantic=None)),
        ab.domain_row(_result(13, record=_record("native"))),
    ]
    for row in rows:
        row["area"] = ab.area_by_domain(arrays).get(row["patch_id"])
    share = ab.share_of(rows)

    assert share["domains"] == {"native": 2, "python": 1, "not_reached": 0, "cache": 1, "refused": 1}
    assert share["area"]["native"] == 4.0 and share["area"]["python"] == 4.0 and share["mesh_area"] == 8.0
    assert share["mesh_domains"] == 4 and share["area_unknown_domains"] == 2  # the cached and the unplaced native one have no packed faces: named, not zero
    totals = ab.totals_of([{"case": "a", "widths": [_width(ab, _native_rows(2))]}])
    assert ab.native_share(totals) == {"domains": 1.0, "area": None, "area_unknown_domains": 2}


def test_the_table_carries_the_wall_clock_columns_and_a_failed_case_keeps_its_row_shape(ab):
    case = {"case": "building:0.25:2:20", "widths": [_width(ab, _native_rows(2), python_wall=3.0, native_wall=1.0)]}
    broken = {"case": "sagging_wall:0.25:2:20", "failure": "Traceback...\nKeyError: 'nope'"}
    lines = ab.format_table({"cases": [case, broken]}).splitlines()

    assert "py wall" in lines[0] and "nat wall" in lines[0] and "wall x" in lines[0]
    assert "3.50" in lines[2] and "1.50" in lines[2] and "2.33" in lines[2]
    assert lines[-1].split()[0] == "sagging_wall:0.25:2:20" and lines[-1].endswith("FAILED: KeyError: 'nope'")
    assert lines[-1].count(" - ") >= 9


def test_the_names_the_strict_gate_relies_on_are_the_names_of_the_kernel_and_the_host(ab):
    from cftuv.envelope_production_export import MATERIALIZED
    from cftuv_envelope.backend import BackendOutcomeV1
    from cftuv_envelope.materialize.step import PATH_FAST

    assert ab.STEP_FAST == PATH_FAST and ab.MATERIALIZED == MATERIALIZED
    assert ab.ALLOWED_FALLBACKS == {BackendOutcomeV1.NATIVE_NOT_REACHED.value}
    assert ab.NOT_MEASURED["apply_seconds"].startswith("NOT_MEASURED"), "the apply stage is named as not measured, never silently absent"
