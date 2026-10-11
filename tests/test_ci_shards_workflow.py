"""The kernel workflow runs the kernel suite as parallel SHARDS (`.github/workflows/envelope-kernel.yml`): the shards are exactly the suite, on every interpreter, in both builds.

Before the sharding each of `kernel-hermetic` (Python 3.10, 3.11) and `kernel-extraction-readiness` (3.10) ran every file of `kernel/tests` in one process, 20-40 minutes each. Now each runs three legs;
a leg runs the files of its shard (`kernel/tools/shard_tests.py`). What would make that a loss of coverage, and what holds it here:

* a shard count in the workflow that disagrees with the legs (a leg that does not exist runs nothing): the shard is ONE value `index/total` per leg and the list must be all of `1..N`, one `N`;
* a shard function that loses or duplicates a file: `problems` is empty for the committed weights and the real `kernel/tests`, and every leg proves it (`check`) before it runs anything;
* a leg that runs the whole directory (or nothing) instead of its files: the pytest line takes `"${FILES[@]}"` produced by `list`, which exits nonzero rather than print an empty list;
* an interpreter or a build dropped from the matrix: the interpreters and the two builds are the ones that ran before.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github" / "workflows" / "envelope-kernel.yml"
KERNEL_TESTS = ROOT / "kernel" / "tests"
WEIGHTS = ROOT / "kernel" / "tools" / "shard_weights.json"

_spec = importlib.util.spec_from_file_location("kernel_shard_tests_host", ROOT / "kernel" / "tools" / "shard_tests.py")
shard_tests = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(shard_tests)


@pytest.fixture(scope="module")
def workflow():
    yaml = pytest.importorskip("yaml")
    return yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))


def _run_text(job) -> str:
    return "\n".join(str(step.get("run", "")) for step in job["steps"])


def _shards(job) -> list:
    return job["strategy"]["matrix"]["shard"]


def test_every_kernel_job_has_all_of_the_shards_one_to_n_and_the_same_n(workflow):
    jobs = workflow["jobs"]
    lists = {name: _shards(jobs[name]) for name in ("kernel-hermetic", "kernel-extraction-readiness")}
    totals = {int(shards[0].split("/")[1]) for shards in lists.values()}
    assert len(totals) == 1, f"the two sharded jobs disagree on the number of shards: {lists}"
    total = totals.pop()
    for name, shards in lists.items():
        assert shards == [f"{index}/{total}" for index in range(1, total + 1)], f"{name}: every shard of 1..{total} is a leg, once"


def test_the_shards_of_the_kernel_workflow_are_exactly_the_kernel_suite(workflow):
    total = int(_shards(workflow["jobs"]["kernel-hermetic"])[0].split("/")[1])
    files = shard_tests.discover(KERNEL_TESTS)
    assert len(files) > 100 and "test_ci_shards.py" in files, "the suite the shards split is the real one"
    default, weights = shard_tests.load_weights(WEIGHTS)
    groups = shard_tests.assign(files, default, weights, total)
    assert shard_tests.problems(files, groups) == []
    assert sorted(name for group in groups for name in group) == files, "every file of kernel/tests is in exactly one shard"


@pytest.mark.parametrize("name", ["kernel-hermetic", "kernel-extraction-readiness"])
def test_a_leg_proves_its_shard_and_runs_only_the_files_of_that_shard(workflow, name):
    job = workflow["jobs"][name]
    test_step = next(step for step in job["steps"] if "pytest" in str(step.get("run", "")))
    assert test_step["env"] == {"SHARD": "${{ matrix.shard }}"}
    run = test_step["run"]
    weights = "--root tests --weights tools/shard_weights.json"
    assert f'tools/shard_tests.py check {weights} --shards "${{SHARD#*/}}"' in run
    assert f'tools/shard_tests.py list {weights} --shard "$SHARD" > "$RUNNER_TEMP/shard-files.txt"' in run, "a failing `list` fails the step: an empty argument list never reaches pytest"
    pytest_line = next(line for line in run.splitlines() if " -m pytest " in line)
    assert pytest_line.rstrip().endswith('"${FILES[@]}"'), "pytest gets the files of the shard and no directory (a directory would run the whole suite in every leg)"
    assert run.index("shard_tests.py check") < run.index("shard_tests.py list") < run.index('mapfile -t FILES') < run.index(pytest_line), "the shards are proved before anything runs"
    assert "CFTUV_TEST_REQUIRE_WHEEL=1" in pytest_line and " -I " in pytest_line, "the guard of the installed wheel and the isolated interpreter are kept"


def test_the_interpreters_and_the_two_builds_are_the_ones_that_ran_before_the_sharding(workflow):
    jobs = workflow["jobs"]
    assert jobs["kernel-hermetic"]["strategy"]["matrix"]["python-version"] == ["3.10", "3.11"]
    extraction = [step for step in jobs["kernel-extraction-readiness"]["steps"] if "python-version" in step.get("with", {})]
    assert [step["with"]["python-version"] for step in extraction] == ["3.10"]
    text = _run_text(jobs["kernel-extraction-readiness"])
    for required in ("git init", "check_forbidden_imports.py src/cftuv_envelope", "sync_session_a_fixtures.py --check", 'test -z "$(git status --porcelain)"'):
        assert required in text, f"the extraction readiness checks are kept in every shard: {required}"
    assert "kernel/dist/*.whl" in _run_text(jobs["kernel-hermetic"]) and "CFTUV_TEST_REQUIRE_WHEEL=1" in _run_text(jobs["kernel-hermetic"])
    assert "blender-free-host-bridge-v0b" in jobs


def test_the_host_suite_runs_in_xdist_processes_per_file_and_its_requirements_declare_xdist():
    """The host suite takes 3 pytest-xdist processes (`--dist loadfile`: a file stays whole in one process, as in a single-process run); the flag without the package would fail the job before the first test."""

    text = (ROOT / ".github" / "workflows" / "host-suite.yml").read_text(encoding="utf-8")
    assert "-n 3 --dist loadfile" in text and "--ignore=tests/test_architecture.py" in text
    requirements = (ROOT / "tests" / "requirements.txt").read_text(encoding="utf-8")
    assert any(line.startswith("pytest-xdist==") for line in requirements.splitlines()), "the exact version of xdist is declared with the other test dependencies"
    architecture = next(line for line in text.splitlines() if "tests/test_architecture.py -q" in line)
    assert "-n " not in architecture, "the structural rules run alone, first, in one process, with a plain message"


def test_the_kernel_weights_table_is_well_formed_and_names_real_files():
    document = json.loads(WEIGHTS.read_text(encoding="utf-8"))
    assert document["schema"] == shard_tests.WEIGHTS_SCHEMA and document["default_seconds"] > 0
    files = set(shard_tests.discover(KERNEL_TESTS))
    assert set(document["seconds"]) <= files, f"weights of files that are gone: {sorted(set(document['seconds']) - files)}"
    assert sum(document["seconds"].values()) > 300, "the table was measured on a real run, not invented"
