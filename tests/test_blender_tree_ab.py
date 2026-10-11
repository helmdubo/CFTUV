"""`tools/blender_tree_ab.py`: the comparison of two trees' reports, the cases, the trees, the driver (with a stand-in for Blender). Nothing here needs Blender.

The tool runs a base tree and a branch tree on one scene in fresh headless Blenders and says whether the product's answer is the same. What decides "the same" is the comparison, so it is proved
here field by field: every field of the answer, every counter, the domain set, the mesh hash, the content keys each move the verdict; what is not an answer (timings, the label-only fields, the
sum of a scheduling group) does not. The driver is run against a stand-in `blender` that replays prepared reports, which proves the plumbing: the PID log, the alternating order, the scene digest
before and after, the exit codes.
"""

from __future__ import annotations

import copy
import json
import os
import re
import stat
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import blender_tree_ab as ab  # noqa: E402

# --------------------------------------------------------------------------
# synthetic reports
# --------------------------------------------------------------------------


def make_row(patch: int, **changes) -> dict:
    row = {
        "patch": patch, "domain_id": f"d{patch}", "outcome": "MATERIALIZED", "detail": "", "content_digest": f"c{patch}", "semantic_digest": f"s{patch}",
        "counters_all": {"EXACT_WORK_PRIME": 10 + patch, "EXACT_WORK_SQUAREFREE": 3, "PRODUCTION_INTERVAL_FAST_HITS": 1, "PRODUCTION_INTERVAL_FALLBACK_X": 0, "SOME_SECONDS": 0.5},
        "diagnostics": [], "normal": [0, 0, 1], "offset_normals_digest": f"n{patch}", "vertex_normals_hash": "v", "decal_topology_law": "PLANAR", "chart_orientation": "UP",
        "step_path": "MATERIALIZED", "placement": "pool:0", "structure_digest": f"t{patch}", "clip_memo": "MISS", "ran": "native",
    }
    row.update(changes)
    return row


def make_press(seconds: float = 1.0, **changes) -> dict:
    press = {
        "run_seconds": seconds, "rows": [make_row(1), make_row(2), make_row(3)], "mesh_hash": "ab" * 32, "mesh_counts": {"vertices": 30, "faces": 12, "seams": 4},
        "outcomes": {"MATERIALIZED": 3}, "materialized": 3, "domains": 3, "cold": True, "content_keys": [[1, "k1"], [2, "k2"], [3, "k3"]],
        "profile_counters": {"PRODUCTION_DOMAINS": 3, "PRODUCTION_INTERVAL_FAST_HITS": 2, "PRODUCTION_POOL_BLOB_HITS": 1, "PRODUCTION_POOL_BYTES": 1000, "STAGE_US": 55},
        "profile_domain_counters": "pdc", "profile_receipts": "rcp", "profile_timing_stages": ["A", "B"],
    }
    press.update(changes)
    return press


def make_report(workers: int = 8, labels=("cold", "step1", "step2"), **changes) -> dict:
    report = {
        "schema": ab.SCHEMA, "case": "building", "workers": workers, "failure": None, "selection_sha256_16": "sel", "native_build_id": "caea0d69",
        "presses": {label: make_press(1.0 + index) for index, label in enumerate(labels)}, "worker_origins_ok": True if workers >= 2 else None,
    }
    report.update(changes)
    return report


def pair(mutate=None):
    base, branch = make_report(), make_report()
    if mutate is not None:
        mutate(branch)
    return ab.compare_runs(base, branch)


def row_of(report, press="cold", patch=2) -> dict:
    return next(row for row in report["presses"][press]["rows"] if row["patch"] == patch)


# --------------------------------------------------------------------------
# the comparison: what is an answer
# --------------------------------------------------------------------------


def test_identical_reports_are_identical_and_every_press_and_domain_is_counted():
    comparison = pair()
    assert comparison.identical and comparison.differences == []
    assert comparison.domains_compared == 9 and list(comparison.presses) == ["cold", "step1", "step2"]
    assert comparison.presses["cold"]["materialized"] == 3 and comparison.presses["cold"]["mesh"] == "ab" * 6


@pytest.mark.parametrize("field", ab.ANSWER_FIELDS)
def test_every_field_of_the_answer_moves_the_verdict(field):
    comparison = pair(lambda branch: row_of(branch, "step1").update({field: "CHANGED"}))
    assert not comparison.identical and any(f"[step1] patch 2 {field}" in line for line in comparison.differences), comparison.differences


def test_every_counter_of_a_domain_is_compared_the_exact_work_ones_included():
    comparison = pair(lambda branch: row_of(branch)["counters_all"].update({"EXACT_WORK_PRIME": 999}))
    assert comparison.differences == ["[cold] patch 2: counter EXACT_WORK_PRIME: base 12 != branch 999"]
    comparison = pair(lambda branch: row_of(branch)["counters_all"].update({"SOME_NEW_COUNTER": 1}))
    assert comparison.differences == [f"[cold] patch 2: counter SOME_NEW_COUNTER: base {ab.ABSENT} != branch 1"], "a counter only one side has is a difference, not a silent extra"
    comparison = pair(lambda branch: row_of(branch)["counters_all"].pop("EXACT_WORK_SQUAREFREE"))
    assert comparison.differences == [f"[cold] patch 2: counter EXACT_WORK_SQUAREFREE: base 3 != branch {ab.ABSENT}"]


def test_timing_counters_are_not_answers():
    comparison = pair(lambda branch: row_of(branch)["counters_all"].update({"SOME_SECONDS": 9.0, "OTHER_US": 3}))
    assert comparison.identical, comparison.differences


def test_scheduling_counters_are_compared_by_the_sum_of_their_group():
    def moved_between_domains(branch):  # the same hits, other workers: the sum over the press is the same
        row_of(branch, patch=1)["counters_all"]["PRODUCTION_INTERVAL_FAST_HITS"] = 0
        row_of(branch, patch=3)["counters_all"]["PRODUCTION_INTERVAL_FAST_HITS"] = 2

    assert pair(moved_between_domains).identical
    comparison = pair(lambda branch: row_of(branch)["counters_all"].update({"PRODUCTION_INTERVAL_FAST_HITS": 5}))
    assert not comparison.identical and any("scheduling group 'interval path'" in line for line in comparison.differences)
    comparison = pair(lambda branch: row_of(branch)["counters_all"].update({"PRODUCTION_INTERVAL_FALLBACK_X": 4}))
    assert any("scheduling group 'interval path'" in line for line in comparison.differences), "the fallback counters are in the same group"


def test_the_counters_of_the_run_follow_the_same_rules():
    def edit(counters):
        return lambda branch: branch["presses"]["cold"]["profile_counters"].update(counters)

    assert pair(edit({"PRODUCTION_POOL_BYTES": 1, "STAGE_US": 1})).identical, "byte counters and timings of the run are not answers"
    assert pair(edit({"PRODUCTION_POOL_BLOB_HITS": 0, "PRODUCTION_POOL_BLOBS_SHIPPED": 1})).identical, "a group is compared by its sum: one hit became one shipment, the sum of the group is the same"
    comparison = pair(edit({"PRODUCTION_POOL_BLOB_HITS": 0, "PRODUCTION_POOL_BLOBS_SHIPPED": 0}))
    assert comparison.differences == ["[cold] run profile: scheduling group 'pool blobs' by sum: base 1 != branch 0"]
    comparison = pair(edit({"PRODUCTION_DOMAINS": 4}))
    assert comparison.differences == ["[cold] run profile: counter PRODUCTION_DOMAINS: base 3 != branch 4"]


def test_the_worker_local_clip_memo_is_a_scheduling_group_found_by_comparing_a_tree_with_itself():
    """The first real pool run of the tool compared origin/main with itself and saw hits 2 / misses 3 against hits 1 / misses 4: which worker had seen a domain. The sum is the answer."""

    base = make_report()
    base["presses"]["step1"]["profile_counters"].update({"PRODUCTION_CLIP_MEMO_HITS": 2, "PRODUCTION_CLIP_MEMO_MISSES": 3})
    for hits, misses, same in ((1, 4, True), (5, 0, True), (2, 4, False), (0, 0, False)):
        branch = make_report()
        branch["presses"]["step1"]["profile_counters"].update({"PRODUCTION_CLIP_MEMO_HITS": hits, "PRODUCTION_CLIP_MEMO_MISSES": misses})
        assert ab.compare_runs(base, branch).identical is same, (hits, misses)
    assert ab.scheduling_group("PRODUCTION_CLIP_MEMO_HITS") == "clip memo" and ab.scheduling_group("EXACT_WORK_PRIME") is None


def test_a_counter_a_change_adds_on_purpose_is_named_not_ignored():
    base, branch = make_report(), make_report()
    branch["presses"]["cold"]["profile_counters"]["PRODUCTION_SNAPSHOT_BYTE_CHECK_DOMAINS"] = 7
    assert not ab.compare_runs(base, branch).identical
    allowed = ab.compare_runs(base, branch, ("PRODUCTION_SNAPSHOT_BYTE_CHECK_",))
    assert allowed.identical and any("PRODUCTION_SNAPSHOT_BYTE_CHECK_DOMAINS" in note and "named on purpose" in note for note in allowed.notes)
    branch["presses"]["cold"]["profile_counters"]["PRODUCTION_OTHER"] = 1
    assert not ab.compare_runs(base, branch, ("PRODUCTION_SNAPSHOT_BYTE_CHECK_",)).identical, "only the named prefix is allowed"


def test_the_set_of_domains_the_mesh_the_keys_and_the_run_digests_are_compared():
    comparison = pair(lambda branch: branch["presses"]["cold"]["rows"].pop())
    assert any("DOMAIN SET" in line and "only base [3]" in line for line in comparison.differences)
    comparison = pair(lambda branch: branch["presses"]["step2"].update(mesh_hash="cd" * 32))
    assert any("[step2] MESH HASH differs" in line for line in comparison.differences)
    comparison = pair(lambda branch: branch["presses"]["cold"].update(content_keys=[[1, "k1"], [2, "other"], [3, "k3"]]))
    assert any("[cold] CONTENT KEYS differ" in line for line in comparison.differences)
    for name, value in (("profile_domain_counters", "x"), ("profile_receipts", "x"), ("profile_timing_stages", ["A"])):
        comparison = pair(lambda branch: branch["presses"]["step1"].update({name: value}))
        assert comparison.differences == [f"[step1] {name} differ"], name


def test_content_keys_are_compared_as_a_set_whatever_the_order_the_parent_computed_them_in():
    comparison = pair(lambda branch: branch["presses"]["cold"].update(content_keys=[[3, "k3"], [1, "k1"], [2, "k2"]]))
    assert comparison.identical


def test_label_only_fields_are_counted_and_never_a_failure():
    comparison = pair(lambda branch: row_of(branch).update(step_path="FAST_HIT", placement="parent", clip_memo="HIT", ran="python"))
    assert comparison.identical
    assert {note.split(" domains differ")[0].split("field ")[1] for note in comparison.notes} == {"step_path: 1", "placement: 1", "clip_memo: 1", "ran: 1"}


def test_a_failed_run_or_a_worker_that_imported_another_tree_is_a_failure_of_the_run():
    comparison = ab.compare_runs(make_report(), make_report(failure="Traceback ... boom"))
    assert not comparison.identical and comparison.differences[0].startswith("branch: the run FAILED: Traceback")
    comparison = ab.compare_runs(make_report(worker_origins_ok=False, worker_origin_problems=["pid 1: sympy_file differs"]), make_report())
    assert comparison.differences == ["base: WORKER ORIGIN: pid 1: sympy_file differs"]


def test_reports_that_are_not_of_the_same_case_presses_or_inputs_do_not_compare():
    assert "not comparable" in ab.compare_runs(make_report(workers=8), make_report(workers=0)).differences[0]
    assert "different presses" in ab.compare_runs(make_report(), make_report(labels=("cold", "step1"))).differences[0]
    comparison = ab.compare_runs(make_report(), make_report(selection_sha256_16="other", native_build_id="deadbeef"))
    assert len(comparison.differences) == 2 and all("did not start from the same inputs" in line for line in comparison.differences)


def test_the_inputs_are_not_modified_by_the_comparison():
    base, branch = make_report(), make_report()
    before = copy.deepcopy((base, branch))
    ab.compare_runs(base, branch)
    assert (base, branch) == before


# --------------------------------------------------------------------------
# cases, presses, scenes, tables
# --------------------------------------------------------------------------


def test_the_built_in_cases_are_the_recipes_the_scratch_harnesses_carried():
    assert set(ab.BUILT_IN_CASES) == {"building", "rounded", "two", "sagging0987", "sagging02239", "half_sphere", "cover008"}
    building = ab.parse_case("building")
    assert (building.mesh, building.alpha, building.density, building.stretch, building.offset, building.scene) == ("building", 0.2239, "2", 42, -1.0, "testscene")
    cover = ab.parse_case("cover008")
    assert (cover.mesh, cover.stretch, cover.offset, cover.scene) == ("cover.008", 20, 0.004999999888241291, "cover008")


def test_a_custom_case_names_its_mesh_alpha_density_stretch_and_optionally_offset_and_scene():
    assert ab.parse_case("mine=wall.001:0.5:1:30") == ab.Case("mine", "wall.001", 0.5, "1", 30, -1.0, "testscene")
    assert ab.parse_case("mine=wall.001:0.5:1:30:0.01:cover008") == ab.Case("mine", "wall.001", 0.5, "1", 30, 0.01, "cover008")
    for bad in ("nosuchcase", "x=a:b", "=a:0.1:2:3", "x=a:0.1:2"):
        with pytest.raises(ValueError):
            ab.parse_case(bad)


def test_the_presses_are_a_cold_one_then_steps_that_widen_by_the_step_width():
    assert ab.parse_presses("cold,step,step", 0.2239) == [("cold", "cold", 0.2239), ("step1", "step", 0.2261), ("step2", "step", 0.2283)]
    assert ab.parse_presses("cold,warm", 1.0, 0.5) == [("cold", "cold", 1.0), ("step1", "step", 1.5)]
    for bad in ("", "step", "cold,cold", "cold,jump"):
        with pytest.raises(ValueError):
            ab.parse_presses(bad, 1.0)


def test_scenes_and_pinned_digests_are_named_by_key():
    scenes = ab.parse_scenes(["main.blend", "cover008=other.blend"])
    assert scenes == {"testscene": Path("main.blend"), "cover008": Path("other.blend")}
    assert ab.parse_expected_shas(["ABC", "cover008=DEF"]) == {"testscene": "abc", "cover008": "def"}


def test_the_table_is_one_line_per_case_and_worker_count():
    comparison = pair()
    rows = [ab.summary_row("building", 8, comparison, {"cold": {"base": 10.0, "branch": 9.0, "change": -10.0}}), ab.summary_row("building", 0, pair(lambda b: row_of(b).update(detail="x")))]
    text = ab.format_table(rows)
    assert "IDENTICAL" in text and "DIFFERENT(1)" in text and "cold 10.00 -> 9.00 (-10.0%)" in text and text.count("building") == 2


def test_the_timing_helpers():
    assert [ab.alternating_order(n) for n in (1, 2, 3)] == [("base", "branch"), ("branch", "base"), ("base", "branch")]
    summary = ab.timing_summary([3.0, 1.0, 2.0])
    assert summary["median"] == 2.0 and summary["min"] == 1.0 and summary["max"] == 3.0 and summary["n"] == 3 and ab.timing_summary([]) == {"n": 0}
    assert ab.change_percent(10.0, 9.0) == pytest.approx(-10.0) and ab.change_percent(0.0, 1.0) is None


def test_a_foreign_blender_is_seen_by_its_cpu_or_by_appearing_and_ours_never_counts():
    before = {10: 5.0, 20: 7.0}
    assert ab.foreign_activity(before, {10: 5.2, 20: 7.1}, {10}) == (0.0, 0), "a little CPU is the noise of an idle process"
    assert ab.foreign_activity(before, {10: 9.0, 20: 7.0}, set()) == (4.0, 0)
    assert ab.foreign_activity(before, {10: 9.0, 20: 7.0}, {10}) == (0.0, 0), "this tool's own Blender is not foreign"
    assert ab.foreign_activity(before, {10: 5.0, 20: 7.0, 30: 1.0}, {10}) == (0.0, 1)
    assert ab.foreign_activity(before, {10: 5.0, 20: 7.0, 30: 1.0}, {10, 30}) == (0.0, 0)


def test_the_idle_guard_waits_while_a_foreign_blender_is_busy_and_stops_nothing(monkeypatch):
    samples = iter([{99: 0.0}, {99: 5.0}, {99: 5.0}, {99: 5.0}])
    monkeypatch.setattr(ab, "blender_cpu", lambda: next(samples))
    monkeypatch.setattr(ab.time, "sleep", lambda seconds: None)
    lines = []
    assert ab.wait_for_idle(set(), seconds_limit=60, window=0.01, log=lines.append) == "idle"
    assert len(lines) == 1 and "foreign blender is busy" in lines[0]
    monkeypatch.setattr(ab, "blender_cpu", lambda: {99: 0.0})
    assert ab.wait_for_idle(set(), seconds_limit=60, window=0.01) == "idle"
    counter = iter(range(10**6))
    monkeypatch.setattr(ab, "blender_cpu", lambda: {99: float(next(counter) * 10)})
    assert ab.wait_for_idle(set(), seconds_limit=0.0, window=0.01) == "TIMEOUT waiting for an idle machine"


def test_the_source_of_the_tool_carries_no_user_path():
    text = (TOOLS / "blender_tree_ab.py").read_text(encoding="utf-8")
    assert not re.search(r"(?<![A-Za-z])[A-Za-z]:[\\/]", text), "a drive-letter path in the code"
    assert not re.search(r"(?i)\b(users|helmd)\b[\\/]|[\\/]users[\\/]", text), "a user directory in the code"


# --------------------------------------------------------------------------
# trees
# --------------------------------------------------------------------------


def _git(repository: Path, *arguments: str) -> str:
    environment = {**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.invalid", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.invalid", "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_SYSTEM": os.devnull}
    return subprocess.run(["git", "-c", "commit.gpgsign=false", *arguments], cwd=str(repository), env=environment, capture_output=True, text=True, check=True).stdout.strip()


def _make_tree(repository: Path, marker: str) -> None:
    for relative, text in {"cftuv/__init__.py": f"MARKER = {marker!r}\n", "kernel/src/cftuv_envelope/__init__.py": "", "README.md": "x\n"}.items():
        target = repository / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(text.encode())


@pytest.fixture
def repository(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    _git(root, "init", "-q", "-b", "main")
    _make_tree(root, "one")
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "-m", "one")
    return root


def test_a_git_ref_is_frozen_into_the_work_directory_and_reused(repository, tmp_path):
    work = tmp_path / "work"
    work.mkdir()
    first = ab.resolve_tree("base", "HEAD", work, repository)
    assert first.root.parent == work / "trees" and first.commit == _git(repository, "rev-parse", "HEAD") and first.root.name == first.commit and first.dirty is False
    assert (first.root / "cftuv" / "__init__.py").read_text(encoding="utf-8") == "MARKER = 'one'\n"
    _make_tree(repository, "two")
    _git(repository, "commit", "-aq", "-m", "two")
    frozen = ab.resolve_tree("base", first.commit, work, repository)
    assert frozen.root == first.root, "a ref names a commit: the same commit is the same directory (reused, not extracted twice)"
    moved = ab.resolve_tree("branch", "HEAD", work, repository)
    assert moved.root != first.root and (moved.root / "cftuv" / "__init__.py").read_text(encoding="utf-8") == "MARKER = 'two'\n", "the checkout moved on; the frozen tree of the old commit did not"
    assert (first.root / "cftuv" / "__init__.py").read_text(encoding="utf-8") == "MARKER = 'one'\n"


def test_a_directory_is_used_as_it_is_and_a_dirty_work_tree_is_recorded(repository, tmp_path):
    work = tmp_path / "work"
    work.mkdir()
    tree = ab.resolve_tree("branch", str(repository), work, repository)
    assert tree.root == repository.resolve() and tree.dirty is False and tree.commit
    (repository / "cftuv" / "__init__.py").write_bytes(b"MARKER = 'edited'\n")
    assert ab.resolve_tree("branch", str(repository), work, repository).dirty is True, "an uncommitted edit of the add-on or the kernel is named in the report"


def test_a_tree_that_is_not_cftuv_or_a_ref_that_is_not_one_is_refused_with_its_name(repository, tmp_path):
    work = tmp_path / "work"
    work.mkdir()
    plain = tmp_path / "plain"
    plain.mkdir()
    with pytest.raises(ValueError, match="not a CFTUV tree"):
        ab.resolve_tree("base", str(plain), work, repository)
    with pytest.raises(ValueError, match="neither a directory nor a git ref"):
        ab.resolve_tree("base", "no-such-ref", work, repository)


# --------------------------------------------------------------------------
# the driver, with a stand-in for Blender
# --------------------------------------------------------------------------

STAND_IN = r'''
import json, shutil, sys
from pathlib import Path

arguments = sys.argv[sys.argv.index("--") + 1:]
options = dict(zip(arguments[1::2], arguments[2::2])) if arguments[0] == "--worker" else {}
here = Path(__file__).resolve().parent
label, case, workers = options["--label"], options["--case"], options["--workers"]
mode = (here / "mode.txt").read_text(encoding="utf-8").strip() if (here / "mode.txt").exists() else ""
if mode == "crash" and label == "branch":
    sys.exit(3)
if mode == "touch-scene":
    scene = Path(sys.argv[2])
    scene.write_bytes(scene.read_bytes() + b"changed")
source = here / "reports" / f"{label}_{case}_w{workers}.json"
shutil.copy(source, options["--out"])
print("TREEAB_OK")
'''


@pytest.fixture
def stand_in(tmp_path):
    folder = tmp_path / "standin"
    (folder / "reports").mkdir(parents=True)
    (folder / "stand_in.py").write_text(STAND_IN, encoding="utf-8")
    if os.name == "nt":
        launcher = folder / "blender.cmd"
        launcher.write_text(f'@echo off\r\n"{sys.executable}" "{folder / "stand_in.py"}" %*\r\n', encoding="utf-8")
    else:
        launcher = folder / "blender"
        launcher.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{folder / "stand_in.py"}" "$@"\n', encoding="utf-8")
        launcher.chmod(launcher.stat().st_mode | stat.S_IEXEC)
    return folder, launcher


def _publish(folder: Path, label: str, report: dict, case: str = "building", workers: int = 8) -> None:
    (folder / "reports" / f"{label}_{case}_w{workers}.json").write_text(json.dumps(report), encoding="utf-8")


def _arguments(tmp_path: Path, launcher: Path, repository: Path, *extra: str) -> list[str]:
    scene = tmp_path / "scene.blend"
    scene.write_bytes(b"BLENDER-scene")
    return [
        "--base", str(repository), "--branch", str(repository), "--scene", str(scene), "--case", "building", "--workers", "8", "--blender", str(launcher),
        "--repository", str(repository), "--work-dir", str(tmp_path / "work"), "--out", str(tmp_path / "report.json"), *extra,
    ]


def test_identical_trees_exit_zero_record_the_pids_and_the_scene_digest(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    for label in ("base", "branch"):
        _publish(folder, label, make_report())
    assert ab.driver_main(_arguments(tmp_path, launcher, repository)) == ab.EXIT_IDENTICAL
    text = capsys.readouterr().out
    assert "TREE_AB_IDENTICAL cases=1 differing=0" in text and "IDENTICAL" in text and "unchanged" in text and "a self-comparison" in text
    document = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    scene = document["scene_sha256"]["testscene"]
    assert scene["before"] == scene["after"] and scene["unchanged"] and document["results"][0]["verdict"] == "IDENTICAL"
    pids = (tmp_path / "work" / "pids.log").read_text(encoding="utf-8").splitlines()
    assert len(pids) == 2 and all(re.search(r"pid=\d+$", line) for line in pids) and {line.split()[1] for line in pids} == {"base", "branch"}


def test_a_different_answer_exits_one_and_names_the_field(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    _publish(folder, "base", make_report())
    changed = make_report()
    row_of(changed, "step2", 3)["content_digest"] = "other"
    _publish(folder, "branch", changed)
    assert ab.driver_main(_arguments(tmp_path, launcher, repository)) == ab.EXIT_DIFFERENT
    text = capsys.readouterr().out
    assert "TREE_AB_DIFFERENT cases=1 differing=1" in text and "[step2] patch 3 content_digest" in text


def test_a_crashed_blender_is_a_failed_run_not_a_silent_pass(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    (folder / "mode.txt").write_text("crash", encoding="utf-8")
    for label in ("base", "branch"):
        _publish(folder, label, make_report())
    assert ab.driver_main(_arguments(tmp_path, launcher, repository)) == ab.EXIT_DIFFERENT
    assert "no report (exit 3)" in capsys.readouterr().out


def test_a_scene_that_changed_while_the_runs_were_made_is_unrunnable(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    (folder / "mode.txt").write_text("touch-scene", encoding="utf-8")
    for label in ("base", "branch"):
        _publish(folder, label, make_report())
    assert ab.driver_main(_arguments(tmp_path, launcher, repository)) == ab.EXIT_UNRUNNABLE
    assert "a scene changed during the runs" in capsys.readouterr().out


def test_a_pinned_scene_digest_that_does_not_match_stops_before_any_run(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    assert ab.driver_main(_arguments(tmp_path, launcher, repository, "--expect-scene-sha", "00" * 32)) == ab.EXIT_UNRUNNABLE
    assert "not the pinned content" in capsys.readouterr().out
    assert not (tmp_path / "work" / "pids.log").exists(), "no Blender was started"


def test_a_request_that_cannot_run_is_named_and_starts_nothing(stand_in, repository, tmp_path, capsys):
    folder, launcher = stand_in
    assert ab.driver_main(_arguments(tmp_path, tmp_path / "no-such-blender.exe", repository)) == ab.EXIT_UNRUNNABLE
    assert "no blender.exe" in capsys.readouterr().out
    arguments = _arguments(tmp_path, launcher, repository)
    arguments[arguments.index("--workers") + 1] = "1"
    assert ab.driver_main(arguments) == ab.EXIT_UNRUNNABLE and "--workers" in capsys.readouterr().out
    arguments = _arguments(tmp_path, launcher, repository, "--case", "cover008")
    assert ab.driver_main(arguments) == ab.EXIT_UNRUNNABLE and "needs the scene 'cover008'" in capsys.readouterr().out


def test_timing_repeats_alternates_the_order_and_reports_medians(stand_in, repository, tmp_path, capsys, monkeypatch):
    folder, launcher = stand_in
    monkeypatch.setattr(ab, "wait_for_idle", lambda ours, **kwargs: "idle")
    monkeypatch.setattr(ab, "blender_cpu", lambda: {})
    _publish(folder, "base", make_report())
    faster = make_report()
    for index, label in enumerate(faster["presses"]):
        faster["presses"][label]["run_seconds"] = (1.0 + index) * 0.5
    _publish(folder, "branch", faster)
    assert ab.driver_main(_arguments(tmp_path, launcher, repository, "--timing", "--reps", "3")) == ab.EXIT_IDENTICAL
    lines = [line for line in capsys.readouterr().out.splitlines() if " rep" in line and "pid=" in line]
    assert [line.split()[3] for line in lines] == ["base", "branch", "branch", "base", "base", "branch"], "the order flips every repetition"
    timing = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))["results"][0]["timing"]
    assert timing["cold"]["base"] == 1.0 and timing["cold"]["branch"] == 0.5 and timing["cold"]["change"] == pytest.approx(-50.0)
    assert timing["step2"]["base_all"]["n"] == 3
