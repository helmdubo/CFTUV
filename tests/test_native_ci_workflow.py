"""Приёмка Rust отчитывается ВСЕГДА (`.github/workflows/native.yml`): задача `changes`, вердикт `native-gate`, структура workflow.

Обязательная проверка у защищённой ветки не должна зависеть от того, запустился ли workflow: прежние фильтры путей на уровне workflow на хост-коммите оставляли `native-gate` «ожидаемым» навсегда.
Теперь workflow идёт на каждом пуше и PR, а нужны ли тяжёлые задачи, решает `tools/native_ci_changes.py` по тому же набору путей. Здесь это держится без GitHub:

1. набор путей и его синтаксис (`**`, `*`, `?` как у фильтров GitHub);
2. решение `changes` на настоящих временных git-репозиториях: пуш `before..sha`, новая ветка и пуш с недоступным `before` (merge-base с `origin/main`), пуш уже проверенного коммита main (его собственный diff),
   PR `base...head` (ушедшая вперёд база не считается), ручной запуск, ЛЮБАЯ ошибка = `true`;
3. таблица истинности `native-gate` (`tools/native_ci_gate.py`): все сочетания результатов против независимо записанного правила;
4. структура самого workflow (разбор YAML): нет фильтров путей, у каждой тяжёлой задачи `needs: changes` и условие на `native == 'true'`, `native-gate` с `if: always()` и всеми нужными `needs`, `changes` берёт только `actions/checkout`;
5. кэш корпусов и шарды дифференциала: корпус восстанавливается только по точному ключу из файлов (без `restore-keys`) и пишется заново только при промахе; шарды матрицы — ровно файлы `tests/test_native_*.py`, каждый
   на каждой ножке Python x слоты (то же покрытие, что до шардирования).
"""

from __future__ import annotations

import ast
import importlib.util
import itertools
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import native_ci_changes as changes  # noqa: E402
import native_ci_corpus as corpus  # noqa: E402
import native_ci_gate as gate  # noqa: E402

WORKFLOW = ROOT / ".github" / "workflows" / "native.yml"
SHARD_TOOL = ROOT / "kernel" / "tools" / "shard_tests.py"
SHARD_WEIGHTS = ROOT / "tests" / "data" / "native_ci_shard_weights.json"
ZERO = changes.ZERO_SHA

_shard_spec = importlib.util.spec_from_file_location("native_ci_shard_tests", SHARD_TOOL)
shard_tests = importlib.util.module_from_spec(_shard_spec)
_shard_spec.loader.exec_module(shard_tests)

# --------------------------------------------------------------------------
# 1. набор путей
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "path, native",
    [
        ("native/cftuv-python/src/pyobj.rs", True),
        ("native/Cargo.lock", True),
        ("tools/native_ci_changes.py", True),
        ("tools/native_clip_synthetic.py", True),
        ("tools/blender_native_ab.py", False),
        ("tools/field_cycle.bat", False),
        ("tools/sub/native_x.py", False),
        ("tests/test_native_pin.py", True),
        ("tests/test_native_ci_workflow.py", True),
        ("tests/native_gate.py", True),
        ("tests/conftest.py", True),
        ("tests/test_architecture.py", True),
        ("tests/data/native_embedding_kernel_suite.recs.xz", True),
        ("tests/data/native_effective_alpha_kernel_suite.recs.xz", True),
        ("tests/data/native_materialize_w1_kernel_suite.recs.xz", True),
        ("tests/data/native_radical_v2_ops.recs.xz", True),
        ("tests/data/native_snapshot_check_seed.xz", True),
        ("tests/data/native_frame_seed.xz", True),
        ("tests/data/native_density_batch_inputs.json.xz", True),
        ("tests/data/native_density_batch_inputs.manifest.json", True),
        ("tools/density_raw_wire.py", True),
        ("tools/density_raw_compile_audit.py", True),
        ("tools/density_raw_compile_diff.py", True),
        ("tools/density_shell_compile_diff.py", True),
        ("tests/data/native_ci_shard_weights.json", True),
        ("kernel/tools/shard_tests.py", True),
        ("kernel/tools/shard_weights.json", False),
        ("tests/test_envelope_host_adapter.py", False),
        ("tests/blender/test_native_x.py", False),
        ("kernel/src/cftuv_envelope/wavefront/coverage.py", True),
        ("kernel/src/cftuv_envelope/backend.py", True),
        ("kernel/tests/conftest.py", True),
        ("kernel/pyproject.toml", True),
        ("kernel/README.md", False),
        ("kernel/fixtures/a.json", False),
        ("pytest.ini", True),
        (".github/workflows/native.yml", True),
        (".github/workflows/host-suite.yml", False),
        ("cftuv/operators.py", False),
        ("DECISIONS.md", False),
        ("README.md", False),
    ],
)
def test_the_native_path_set_is_the_old_workflow_filter(path, native):
    assert changes.is_native_path(path) is native


def test_the_path_set_is_exactly_the_filter_the_workflow_used_to_carry():
    assert changes.NATIVE_PATHS == (
        "native/**", "tools/native_*.py", "tests/test_native_*.py", "tests/native_gate.py", "tests/conftest.py", "tests/test_architecture.py",
        "tests/data/native_embedding_kernel_suite.recs.xz", "tests/data/native_effective_alpha_kernel_suite.recs.xz", "tests/data/native_materialize_w1_kernel_suite.recs.xz",
        "tests/data/native_radical_v2_ops.recs.xz", "tests/data/native_snapshot_check_seed.xz", "tests/data/native_frame_seed.xz",
        "tests/data/native_density_batch_inputs.json.xz",
        "tests/data/native_density_batch_inputs.manifest.json", "tools/density_raw_wire.py",
        "tools/density_raw_compile_audit.py", "tools/density_raw_compile_diff.py", "tools/density_shell_compile_diff.py",
        "tests/data/native_ci_shard_weights.json", "kernel/tools/shard_tests.py",
        "kernel/src/cftuv_envelope/**", "kernel/tests/**", "kernel/pyproject.toml", "pytest.ini", ".github/workflows/native.yml",
    )
    assert all(
        changes.is_native_path(path)
        for path in ("tools/native_ci_changes.py", "tools/native_ci_gate.py", "tools/native_ci_corpus.py", "tests/test_native_ci_workflow.py", "tests/test_native_ci_corpus.py", "kernel/tools/shard_tests.py")
    ), "the files of this very mechanism are native-relevant"


# --------------------------------------------------------------------------
# 2. решение `changes` на настоящем git
# --------------------------------------------------------------------------


class Repo:
    """Временный репозиторий; `main` — то, что в CI называется `origin/main` (ref создаётся вручную)."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.env = {**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.invalid", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.invalid", "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_SYSTEM": os.devnull}
        path.mkdir(parents=True)
        self.git("init", "-q", "-b", "main")
        self.counter = 0

    def git(self, *arguments: str) -> str:
        done = subprocess.run(["git", "-c", "commit.gpgsign=false", *arguments], cwd=str(self.path), env=self.env, capture_output=True, text=True, check=True)
        return done.stdout.strip()

    def commit(self, *files: str, message: str = "c") -> str:
        for name in files:
            target = self.path / name
            target.parent.mkdir(parents=True, exist_ok=True)
            self.counter += 1
            target.write_text(f"{name} {self.counter}\n", encoding="utf-8")
        self.git("add", "-A")
        self.git("commit", "-q", "-m", message)
        return self.git("rev-parse", "HEAD")

    def origin_main_at(self, sha: str) -> None:
        self.git("update-ref", "refs/remotes/origin/main", sha)

    def checkout(self, *arguments: str) -> None:
        self.git("checkout", "-q", *arguments)


@pytest.fixture
def repo(tmp_path):
    """root(host) - c1(host) - c2(native) = origin/main; `HOST`, `NATIVE` — имена файлов."""

    created = Repo(tmp_path / "repo")
    created.root = created.commit("README.md", message="root")
    created.host = created.commit("cftuv/a.py", message="host")
    created.native = created.commit("native/x.rs", message="native")
    created.origin_main_at(created.native)
    return created


def push(repo, sha, before=ZERO):
    return changes.decide({"EVENT": "push", "SHA": sha, "BEFORE": before}, repo.path)


def test_a_push_of_host_files_only_does_not_require_the_native_acceptance(repo):
    tip = repo.commit("cftuv/b.py", "DECISIONS.md")
    decision = push(repo, tip, before=repo.native)
    assert decision.native is False and decision.matched == [] and sorted(decision.files) == ["DECISIONS.md", "cftuv/b.py"]


def test_an_embedding_corpus_only_push_requires_the_native_gate(repo):
    path = "tests/data/native_embedding_kernel_suite.recs.xz"
    tip = repo.commit(path)
    decision = push(repo, tip, before=repo.native)
    assert decision.native is True and decision.files == decision.matched == [path]


def test_an_effective_alpha_corpus_only_push_requires_the_native_gate(repo):
    path = "tests/data/native_effective_alpha_kernel_suite.recs.xz"
    tip = repo.commit(path)
    decision = push(repo, tip, before=repo.native)
    assert decision.native is True and decision.files == decision.matched == [path]


@pytest.mark.parametrize("path", ("tests/data/native_density_batch_inputs.json.xz", "tests/data/native_density_batch_inputs.manifest.json", "tools/density_raw_wire.py", "tools/density_raw_compile_audit.py", "tools/density_raw_compile_diff.py", "tools/density_shell_compile_diff.py"))
def test_a_density_raw_seed_input_only_push_requires_the_native_gate(repo, path):
    tip = repo.commit(path)
    decision = push(repo, tip, before=repo.native)
    assert decision.native is True and decision.files == decision.matched == [path]


def test_a_push_that_touches_one_native_path_requires_it_and_names_the_path(repo):
    first = repo.commit("cftuv/b.py")
    tip = repo.commit("tools/native_catchup.py")
    decision = push(repo, tip, before=first)
    assert decision.native is True and decision.matched == ["tools/native_catchup.py"]
    wide = push(repo, tip, before=repo.native)
    assert wide.native is True and sorted(wide.files) == ["cftuv/b.py", "tools/native_catchup.py"], "a push of several commits is judged as one range"


def test_a_new_branch_is_judged_against_the_merge_base_with_origin_main_not_against_nothing(repo):
    repo.checkout("-b", "feature")
    host = repo.commit("cftuv/b.py")
    assert push(repo, host).native is False, "one host commit on a new branch"
    tip = repo.commit("kernel/src/cftuv_envelope/numeric.py")
    decision = push(repo, tip)
    assert decision.native is True and "kernel/src/cftuv_envelope/numeric.py" in decision.matched
    assert sorted(decision.files) == ["cftuv/b.py", "kernel/src/cftuv_envelope/numeric.py"], "every commit of the branch counts, none of origin/main's own history does"


def test_a_previous_tip_that_is_not_in_the_repository_falls_back_like_a_new_branch(repo):
    repo.checkout("-b", "feature")
    host = repo.commit("cftuv/b.py")
    gone = "deadbeef" * 5
    decision = push(repo, host, before=gone)
    assert decision.native is False and "not in the repository" in decision.how
    native = repo.commit("pytest.ini")
    assert push(repo, native, before=gone).native is True


def test_a_push_of_a_commit_already_on_origin_main_is_judged_by_its_own_diff(repo):
    # the tip of main: its own diff against the first parent is the native commit
    decision = push(repo, repo.native)
    assert decision.native is True and decision.files == ["native/x.rs"] and "own diff" in decision.how
    # an older, host-only commit of main
    assert push(repo, repo.host).native is False
    # the root commit has no parent: all its files are judged
    assert push(repo, repo.root).native is False


def test_the_root_commit_with_a_native_file_requires_the_native_acceptance(tmp_path):
    created = Repo(tmp_path / "r")
    root = created.commit("native/x.rs", "README.md")
    created.origin_main_at(root)
    assert push(created, root).native is True


def test_a_pull_request_is_judged_from_the_merge_base_so_a_base_that_moved_on_does_not_count(repo):
    repo.checkout("-b", "feature", repo.host)  # branched BEFORE the native commit of main
    head = repo.commit("cftuv/b.py")
    decision = changes.decide({"EVENT": "pull_request", "BASE_SHA": repo.native, "HEAD_SHA": head}, repo.path)
    assert decision.native is False and decision.files == ["cftuv/b.py"], "the native commit is on the base, not on the pull request"
    native_head = repo.commit("tests/test_native_pin.py")
    assert changes.decide({"EVENT": "pull_request", "BASE_SHA": repo.native, "HEAD_SHA": native_head}, repo.path).native is True


def test_a_manual_run_always_takes_the_native_acceptance(tmp_path):
    decision = changes.decide({"EVENT": "workflow_dispatch"}, tmp_path)  # not even a repository
    assert decision.native is True and "manual" in decision.how


@pytest.mark.parametrize(
    "environment",
    [
        {"EVENT": "pull_request", "BASE_SHA": "0" * 40, "HEAD_SHA": "1" * 40},
        {"EVENT": "pull_request"},
        {"EVENT": "schedule"},
        {"EVENT": ""},
        {"EVENT": "push", "SHA": "2" * 40, "BEFORE": "3" * 40},
    ],
    ids=["unknown commits", "no commits", "unknown event", "no event", "unknown tip"],
)
def test_any_error_is_native_true_never_a_skipped_acceptance(repo, environment):
    decision = changes.decide(environment, repo.path)
    assert decision.native is True and decision.how.startswith("ERROR"), decision


def test_a_new_branch_without_origin_main_is_native_true(tmp_path):
    created = Repo(tmp_path / "r")
    tip = created.commit("README.md")
    decision = push(created, tip)  # origin/main was never fetched
    assert decision.native is True and decision.how.startswith("ERROR")


def test_the_program_writes_the_output_and_the_report_and_never_fails(repo, tmp_path, capsys):
    out, summary = tmp_path / "out.txt", tmp_path / "summary.md"
    tip = repo.commit("cftuv/b.py")
    environment = {"EVENT": "push", "SHA": tip, "BEFORE": repo.native, "GITHUB_OUTPUT": str(out), "GITHUB_STEP_SUMMARY": str(summary)}
    assert changes.main(environment, repo.path) == 0
    assert out.read_text(encoding="utf-8") == "native=false\n"
    assert "not required" in summary.read_text(encoding="utf-8") and "host    cftuv/b.py" in capsys.readouterr().out
    native = repo.commit("native/y.rs")
    environment.update(SHA=native, BEFORE=tip)
    assert changes.main(environment, repo.path) == 0
    assert out.read_text(encoding="utf-8").splitlines() == ["native=false", "native=true"]
    assert "NATIVE  native/y.rs" in capsys.readouterr().out


# --------------------------------------------------------------------------
# 3. таблица истинности native-gate
# --------------------------------------------------------------------------

RESULTS = ("success", "failure", "cancelled", "skipped")
HEAVY = {name: "success" for name in gate.HEAVY_JOBS}


def _rule(changes_result, native, results) -> bool:
    """Правило, записанное независимо от `gate.decide`: `changes` успешна И (`native == 'false'` ИЛИ (`native == 'true'` И все тяжёлые `success`))."""

    return changes_result == "success" and (native == "false" or (native == "true" and all(value == "success" for value in results.values())))


def test_the_gate_truth_table_in_words():
    ok = dict(HEAVY)
    assert gate.decide("success", "false", {name: "skipped" for name in HEAVY})[0] is True, "a host commit: the heavy jobs are skipped and the gate is green"
    assert gate.decide("success", "false", ok)[0] is True
    assert gate.decide("success", "true", ok)[0] is True, "a native commit: everything succeeded"
    for name in HEAVY:
        for result in ("failure", "cancelled", "skipped", ""):
            passed, why = gate.decide("success", "true", {**ok, name: result})
            assert passed is False and name in why, f"{name}={result!r} on a native commit must fail the gate and say which job"
    for result in ("failure", "cancelled", "skipped", ""):
        assert gate.decide(result, "false", {name: "skipped" for name in HEAVY})[0] is False, "`changes` did not succeed: nothing is known"
        assert gate.decide(result, "true", ok)[0] is False
    for native in ("", "True", "yes", "null"):
        assert gate.decide("success", native, ok)[0] is False, f"an unknown output {native!r} is not `false`"


def test_the_gate_agrees_with_the_independent_rule_on_every_combination():
    checked = 0
    for changes_result, native in itertools.product(RESULTS + ("",), ("true", "false", "", "maybe")):
        for combination in itertools.product(RESULTS + ("",), repeat=len(HEAVY)):
            results = dict(zip(HEAVY, combination))
            assert gate.decide(changes_result, native, results)[0] is _rule(changes_result, native, results), (changes_result, native, results)
            checked += 1
    assert checked == 5 * 4 * 5 ** len(HEAVY)


def test_the_gate_judges_every_job_the_native_acceptance_consists_of():
    assert gate.HEAVY_JOBS == ("rust", "wheel", "corpus-clip", "corpus-skeleton", "differential"), "a job dropped from the verdict would no longer be required"
    assert [gate.result_variable(name) for name in gate.HEAVY_JOBS] == ["RUST_RESULT", "WHEEL_RESULT", "CORPUS_CLIP_RESULT", "CORPUS_SKELETON_RESULT", "DIFFERENTIAL_RESULT"]


def test_the_gate_program_reads_the_needs_from_the_environment_and_exits_nonzero_on_failure(capsys):
    results = {gate.result_variable(name): "success" for name in gate.HEAVY_JOBS}
    environment = {"CHANGES_RESULT": "success", "NATIVE": "true", **results}
    assert gate.main(environment) == 0 and "NATIVE_GATE_PASSED" in capsys.readouterr().out
    assert gate.main({**environment, "DIFFERENTIAL_RESULT": "skipped"}) == 1
    out = capsys.readouterr().out
    assert "NATIVE_GATE_FAILED" in out and "differential=skipped" in out
    for name in gate.HEAVY_JOBS:
        assert gate.main({**environment, gate.result_variable(name): "failure"}) == 1, f"{name} red on a native commit must fail the gate"
        assert f"{name}=failure" in capsys.readouterr().out
        assert gate.main({key: value for key, value in environment.items() if key != gate.result_variable(name)}) == 1, f"{name} missing from the environment (a broken wiring) is a failure"
        capsys.readouterr()
    assert gate.main({}) == 1, "an empty environment (a broken wiring) is a failure, not a pass"
    assert gate.main({**environment, "NATIVE": "false", **{gate.result_variable(name): "skipped" for name in gate.HEAVY_JOBS}}) == 0


# --------------------------------------------------------------------------
# 4. структура workflow
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def workflow():
    yaml = pytest.importorskip("yaml")
    return yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))


def _needs(job) -> list:
    needs = job.get("needs", [])
    return [needs] if isinstance(needs, str) else list(needs)


def test_the_workflow_has_no_path_filters_so_the_required_check_always_reports(workflow):
    triggers = workflow.get("on", workflow.get(True))  # PyYAML reads the key `on` as a boolean
    assert set(triggers) == {"push", "pull_request", "workflow_dispatch"}
    for name in ("push", "pull_request"):
        assert not triggers[name] or not ({"paths", "paths-ignore"} & set(triggers[name])), f"{name}: a path filter on the workflow leaves a required check pending forever"
        assert not triggers[name] or "branches" not in triggers[name], f"{name}: every branch"


def test_every_heavy_job_waits_for_changes_and_runs_only_on_a_native_relevant_commit(workflow):
    jobs = workflow["jobs"]
    for name in (*gate.HEAVY_JOBS, "interpreters-advisory"):
        assert "changes" in _needs(jobs[name]), f"{name}: needs `changes`"
        assert jobs[name].get("if") == "needs.changes.outputs.native == 'true'", f"{name}: runs only when `changes` said native=true"
    assert "wheel" in _needs(jobs["corpus-clip"]), "the clip corpus is encoded by the shim, which imports the compiled extension"
    assert set(_needs(jobs["differential"])) >= {"wheel", "corpus-clip", "corpus-skeleton"}, "the earlier chains are kept: the differential legs read the wheel and both corpora"
    assert "wheel" not in _needs(jobs["corpus-skeleton"]), "the skeleton corpus is recorded and answered by the oracle alone: it starts with `changes`, beside the wheel"
    text = "\n".join(str(step) for step in jobs["corpus-skeleton"]["steps"])
    assert "cftuv_native" not in text and "native-wheel" not in text, "the skeleton corpus job installs no extension"


def test_the_changes_job_is_plain_git_on_the_full_history_and_falls_back_to_true(workflow):
    job = workflow["jobs"]["changes"]
    uses = [step["uses"] for step in job["steps"] if "uses" in step]
    assert uses == ["actions/checkout@v4"], f"`changes` uses nothing but checkout (no third-party action): {uses}"
    assert job["steps"][0]["with"] == {"fetch-depth": 0}
    decide = next(step for step in job["steps"] if step.get("id") == "decide")
    assert decide.get("continue-on-error") is True and decide["run"].strip() == "python3 tools/native_ci_changes.py"
    assert {"EVENT", "BASE_SHA", "HEAD_SHA", "BEFORE", "SHA"} <= set(decide["env"])
    assert job["outputs"]["native"] == "${{ steps.decide.outputs.native || 'true' }}", "no output (the program did not run at all) means true"


def test_the_gate_always_runs_waits_for_everything_it_judges_and_hands_the_results_to_the_script(workflow):
    job = workflow["jobs"]["native-gate"]
    assert job["if"] == "always()"
    assert set(_needs(job)) == {"changes", *gate.HEAVY_JOBS}, "the advisory job is not part of the verdict"
    step = next(step for step in job["steps"] if "run" in step)
    assert step["run"].strip() == "python3 tools/native_ci_gate.py"
    assert step["env"] == {
        "CHANGES_RESULT": "${{ needs.changes.result }}",
        "NATIVE": "${{ needs.changes.outputs.native }}",
        **{gate.result_variable(name): f"${{{{ needs.{name}.result }}}}" for name in gate.HEAVY_JOBS},
    }


def test_the_workflow_installs_the_yaml_reader_this_test_needs(workflow):
    assert "pyyaml==" in workflow["env"]["TEST_REQUIREMENTS"], "the strict differential job parses the workflow: PyYAML is among its exact dependencies"


def test_the_skeleton_corpus_is_built_by_the_corpus_job_under_a_time_bound_and_handed_to_every_differential_leg(workflow):
    """The skeleton has no field corpus in CI: its synthetic corpus (a time-bounded subset of the kernel tests, the generator's frozen cases, the derived records) is built beside the clip one,
    uploaded per interpreter, downloaded by each differential leg and found there through `CFTUV_NATIVE_CORPUS`."""

    jobs = workflow["jobs"]
    build = next(step for step in jobs["corpus-skeleton"]["steps"] if step.get("name", "").startswith("Build the synthetic skeleton corpus"))
    assert build["timeout-minutes"] <= 30, "the step is time-bounded"
    assert build["run"].strip() == 'python tools/native_ci_corpus.py build --kind skeleton --out "$RUNNER_TEMP/skeleton-corpus"', "the recipe lives in tools/native_ci_corpus.py (its source is part of the cache key)"
    commands = [" ".join(command) for command in corpus.build_commands("skeleton", Path("BASE"))]
    assert "native_skeleton_synthetic.py build" in commands[0] and " --ci" in commands[0] and "native_skeleton_generated.py generate" in commands[1] and "native_skeleton_derive.py" in commands[2]
    assert all(str(Path("BASE", "ci", "synthetic_skeleton")) in command for command in commands), "the layout `native_skeleton_corpus.matching('synthetic')` finds: <base>/<name>/synthetic_skeleton"
    uploads = [step["with"] for step in jobs["corpus-skeleton"]["steps"] if str(step.get("uses", "")).startswith("actions/upload-artifact")]
    assert {"name": "synthetic-skeleton-py${{ matrix.python }}", "path": "${{ runner.temp }}/skeleton-corpus"} == {key: uploads[-1][key] for key in ("name", "path")}
    differential = jobs["differential"]["steps"]
    downloads = [step["with"] for step in differential if str(step.get("uses", "")).startswith("actions/download-artifact")]
    assert {"name": "synthetic-skeleton-py${{ matrix.python }}", "path": "${{ runner.temp }}/skeleton-corpus"} in downloads
    strict = next(step for step in differential if step.get("name", "").startswith("Differential tests against the live oracle"))
    assert strict["env"]["CFTUV_NATIVE_CORPUS"] == "${{ runner.temp }}/skeleton-corpus", "the strict run reads the synthetic corpus and nothing of the owner's drive"
    assert strict["env"]["CFTUV_NATIVE_STRICT"] == "1" and "-m \"not native_field\"" in strict["run"]
    available = next(step for step in differential if step.get("name", "").startswith("All native operations are available"))
    assert '"skeleton": "available"' in available["run"] and "skeleton_raw_layouts" in available["run"]
    assert '"snap_embedding": "available"' in available["run"]
    assert '"effective_alpha": "available"' in available["run"]
    assert '"materialize_w1": "available"' in available["run"]
    assert '"snapshot_check": "available"' in available["run"] and "snapshot_check_raw_layouts" in available["run"], "the reader of the snapshot validators reports its slot layouts in every leg"
    assert '"radical_v2": "available"' in available["run"], "the capability of the RadicalSumV2 mirror is reported available in every leg"
    assert '"frame": "available"' in available["run"] and "frame_raw_layouts" in available["run"], "the reader of the frame admission reports its slot layouts in every leg"
    assert '"density": "available"' in available["run"]


def test_embedding_inline_and_tracked_kernel_checks_are_mandatory_in_the_strict_matrix(workflow):
    differential = workflow["jobs"]["differential"]
    assert set(differential["strategy"]["matrix"]["python"]) == {"3.11", "3.13"}
    assert set(differential["strategy"]["matrix"]["slots"]) == {"auto", "raw", "attr"}
    strict = next(step for step in differential["steps"] if step.get("name", "").startswith("Differential tests against the live oracle"))
    assert strict["env"]["CFTUV_EMBEDDING_SYNTHETIC"] == "600"
    assert '"${FILES[@]}"' in strict["run"] and '-m "not native_field"' in strict["run"], "the files of the shard, and the field tier deselected by NAME (nothing else)"
    assert " -k " not in strict["run"], "strict mode refuses a name filter anyway; the workflow does not ask for one"
    # Уровень поля снимается в CI; inline и tracked seed обязаны оставаться в каждом строгом прогоне.
    module = ast.parse((ROOT / "tests" / "test_native_snap_embedding.py").read_text(encoding="utf8"))
    required = {"test_synthetic_cases_equal_the_oracle", "test_the_kernel_suite_records_equal_the_oracle"}
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    assert all(not node.decorator_list for node in found.values())
    seed = ROOT / "tests" / "data" / "native_embedding_kernel_suite.recs.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())


def test_effective_alpha_synthetic_and_tracked_kernel_seed_checks_are_mandatory_in_the_strict_matrix(workflow):
    differential = workflow["jobs"]["differential"]
    strict = next(step for step in differential["steps"] if step.get("name", "").startswith("Differential tests against the live oracle"))
    assert strict["env"]["CFTUV_EFFECTIVE_ALPHA_SYNTHETIC"] == "0.3", "the scale of the generated pairs is pinned by the workflow, like the embedding's"
    # the generated pairs and the tracked seed run in every strict leg: they are not the field tier and carry no marker that CI deselects
    module = ast.parse((ROOT / "tests" / "test_native_effective_alpha.py").read_text(encoding="utf8"))
    required = {"test_synthetic_pairs_equal_the_oracle_and_every_kind_of_pair_is_there", "test_the_kernel_suite_seed_equals_the_oracle", "test_the_outcomes_do_not_depend_on_the_hash_seed_of_the_process"}
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    assert all(not node.decorator_list for node in found.values())
    seed = ROOT / "tests" / "data" / "native_effective_alpha_kernel_suite.recs.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())
    default = next(node.value.args[0].args[1].value for node in ast.walk(module) if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "SYNTHETIC_SCALE")
    assert default == strict["env"]["CFTUV_EFFECTIVE_ALPHA_SYNTHETIC"], "the default of the test is the value the workflow pins"


def test_materialize_w1_seed_and_seams_are_mandatory_in_the_strict_matrix_and_its_corpora_are_the_field_tier():
    # the tracked seed, the generated contours and the object/refusal checks run in every strict leg: no marker that CI deselects, no data outside the repository
    module = ast.parse((ROOT / "tests" / "test_native_materialize_w1.py").read_text(encoding="utf8"))
    required = {
        "test_the_seed_shows_every_outcome_the_port_has_to_get_right",
        "test_generated_contours_and_fans_are_tessellated_as_the_oracle_does",
        "test_the_result_reuses_the_objects_the_oracle_returns",
        "test_a_refusal_of_the_port_after_the_computation_leaves_every_state_as_it_was",
        "test_the_slot_modes_give_the_same_answer",
    }
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    gated = {name for name, node in found.items() if any("native_field" in ast.unparse(item) or "needs_field" in ast.unparse(item) for item in node.decorator_list)}
    assert not gated, f"{sorted(gated)} would be deselected in CI"
    seed = ROOT / "tests" / "data" / "native_materialize_w1_kernel_suite.recs.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())
    field = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == "test_the_native_operation_equals_the_oracle_on_the_field_corpora")
    assert any("needs_field" in ast.unparse(item) for item in field.decorator_list), "the owner's corpora are the field tier"


def test_radical_v2_seed_synthetic_and_vector_checks_are_mandatory_in_the_strict_matrix(workflow):
    differential = workflow["jobs"]["differential"]
    strict = next(step for step in differential["steps"] if step.get("name", "").startswith("Differential tests against the live oracle"))
    assert strict["env"]["CFTUV_RADICAL_V2_SYNTHETIC"] == "1500", "the scale of the synthetic programs is pinned by the workflow, like the other captures"
    # the tracked seed, the synthetic programs, the near-zero families, the PRICE vectors and the vector file run in every strict leg: only the owner's field records are a tier
    module = ast.parse((ROOT / "tests" / "test_native_radical_v2.py").read_text(encoding="utf8"))
    required = {
        "test_the_tracked_seed_of_recorded_operations_equals_the_oracle", "test_synthetic_programs_equal_the_oracle_and_every_refusal_is_there",
        "test_near_zero_families_equal_the_oracle_and_the_cap_of_4096_is_a_named_price", "test_the_price_vectors_of_the_kernel_test_hold_in_the_extension",
        "test_the_vector_file_of_cargo_test_is_what_the_oracle_writes_now", "test_the_outcomes_do_not_depend_on_the_hash_seed_of_the_process",
    }
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    assert all(not node.decorator_list for node in found.values())
    seed = ROOT / "tests" / "data" / "native_radical_v2_ops.recs.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())
    default = next(node.value.args[0].args[1].value for node in ast.walk(module) if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "SYNTHETIC")
    assert default == strict["env"]["CFTUV_RADICAL_V2_SYNTHETIC"], "the default of the test is the value the workflow pins"


def test_snapshot_check_seed_and_mutants_are_mandatory_in_the_strict_matrix_and_its_corpora_are_the_field_tier():
    # the tracked seed, the soundness mutants, the strictness of the reader, the slot modes and the dispatcher run in every strict leg: no marker that CI deselects, no data outside the repository
    module = ast.parse((ROOT / "tests" / "test_native_snapshot_check.py").read_text(encoding="utf8"))
    required = {
        "test_the_seed_shows_every_outcome_the_port_has_to_get_right",
        "test_the_native_answer_is_sound_and_its_counters_equal_the_validators_on_every_row_of_the_seed",
        "test_the_270_degree_corners_of_the_seed_are_decided_natively_with_the_counters_of_the_validators",
        "test_no_mutant_of_the_seed_is_attested_that_the_validators_do_not_pass",
        "test_a_value_that_is_not_exactly_the_class_of_its_field_is_declined_by_name",
        "test_the_port_declines_when_the_process_is_not_in_the_state_it_mirrors",
        "test_the_slot_modes_give_the_same_answer",
        "test_a_stale_pin_is_a_named_fallback_of_the_dispatcher_not_a_verdict",
    }
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    gated = {name for name, node in found.items() if any("native_field" in ast.unparse(item) or "needs_field" in ast.unparse(item) for item in node.decorator_list)}
    assert not gated, f"{sorted(gated)} would be deselected in CI"
    seed = ROOT / "tests" / "data" / "native_snapshot_check_seed.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())
    field = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == "test_the_native_stage_equals_the_validators_on_the_field_corpora")
    assert any("needs_field" in ast.unparse(item) for item in field.decorator_list), "the owner's corpora are the field tier"


def test_frame_seed_and_mutants_are_mandatory_in_the_strict_matrix_and_its_corpora_are_the_field_tier():
    # the tracked seed, the soundness mutants, the strictness of the reader, the slot modes and the dispatcher run in every strict leg; the owner's corpora are the field tier
    module = ast.parse((ROOT / "tests" / "test_native_frame.py").read_text(encoding="utf8"))
    required = {
        "test_the_seed_shows_every_outcome_the_port_has_to_get_right",
        "test_the_native_answer_equals_the_builder_or_is_declined_by_name_on_every_row_of_the_seed",
        "test_no_mutant_of_the_seed_is_answered_otherwise_than_the_builder",
        "test_a_value_that_is_not_exactly_the_class_of_its_field_is_declined_by_name",
        "test_the_port_declines_when_the_process_is_not_in_the_state_it_mirrors",
        "test_the_slot_modes_give_the_same_answer",
        "test_a_stale_pin_is_a_named_fallback_of_the_dispatcher_not_an_answer",
    }
    found = {node.name: node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in required}
    assert set(found) == required
    gated = {name for name, node in found.items() if any("native_field" in ast.unparse(item) or "needs_field" in ast.unparse(item) for item in node.decorator_list)}
    assert not gated, f"{sorted(gated)} would be deselected in CI"
    seed = ROOT / "tests" / "data" / "native_frame_seed.xz"
    assert seed.is_file() and changes.is_native_path(seed.relative_to(ROOT).as_posix())
    field = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == "test_the_native_frame_equals_the_builder_on_the_field_corpora")
    assert any("needs_field" in ast.unparse(item) for item in field.decorator_list), "the owner's corpora are the field tier"


# --------------------------------------------------------------------------
# 5. кэш корпусов и шарды дифференциала
# --------------------------------------------------------------------------

_HIT = "steps.cache.outputs.cache-hit != 'true'"


def _step(job, prefix):
    return next(step for step in job["steps"] if step.get("name", "").startswith(prefix) or str(step.get("uses", "")).startswith(prefix))


@pytest.mark.parametrize("job_name, kind, path", [("corpus-clip", "clip", "synthetic-clip"), ("corpus-skeleton", "skeleton", "skeleton-corpus")])
def test_a_corpus_is_restored_by_its_exact_key_and_rebuilt_only_on_a_miss(workflow, job_name, kind, path):
    job = workflow["jobs"][job_name]
    steps = job["steps"]
    key = next(step for step in steps if step.get("id") == "key")
    assert key["run"].strip() == f"python tools/native_ci_corpus.py key --kind {kind}", "the key is computed from the files by the tool whose tests prove it"
    restore = next(step for step in steps if step.get("id") == "cache")
    assert restore["uses"] == "actions/cache/restore@v4", "restore and save are separate steps: a build that failed saves nothing"
    assert restore["with"] == {"path": f"${{{{ runner.temp }}}}/{path}", "key": "${{ steps.key.outputs.key }}"}, "the exact key and nothing else: no `restore-keys` (a prefix match restores the corpus of another kernel)"
    save = next(step for step in steps if str(step.get("uses", "")).startswith("actions/cache/save"))
    assert save["with"] == restore["with"] and save["if"] == _HIT
    index = {id(step): number for number, step in enumerate(steps)}
    build = next(step for step in steps if str(step.get("run", "")).startswith(f"python tools/native_ci_corpus.py build --kind {kind}"))
    assert build["if"] == _HIT and f'--out "$RUNNER_TEMP/{path}"' in build["run"]
    assert index[id(key)] < index[id(restore)] < index[id(build)] < index[id(save)], "key, restore, build, save: in that order"
    upload = next(step for step in steps if str(step.get("uses", "")).startswith("actions/upload-artifact"))
    assert upload["with"]["path"] == restore["with"]["path"], "what the differential legs download is what the cache holds"
    for step in steps:
        if str(step.get("run", "")).startswith(("python -m pip install", "python tools/native_ci_corpus.py build")) or str(step.get("uses", "")).startswith("actions/download-artifact"):
            assert step["if"] == _HIT, f"{step.get('name')}: a cache hit installs and builds nothing"


def test_only_the_cargo_build_caches_fall_back_to_a_prefix_match(workflow):
    """A corpus is a function of its inputs and is restored by the exact key only (a prefix match returns the corpus of another kernel). A cargo build cache is different in kind: the dependencies
    are kept and cargo's own fingerprints (toolchain, flags, sources) decide what to rebuild, so a nearby entry cannot make a build wrong, only slower."""

    for name, job in workflow["jobs"].items():
        for step in job["steps"]:
            if "restore-keys" in step.get("with", {}):
                assert name == "rust" and step["with"]["key"].startswith("cargo-"), f"{name}: a prefix match is the same mistake as a key that is too narrow"
                assert "native/target" in step["with"]["path"] and "Cargo.lock" in step["with"]["key"], "the build cache is keyed by the lock file"
    assert any("restore-keys" in step.get("with", {}) for step in workflow["jobs"]["rust"]["steps"]), "the test build keeps its dependencies (175 s -> 25 s measured)"
    assert not any(str(step.get("uses", "")).startswith("actions/cache") for step in workflow["jobs"]["wheel"]["steps"]), "the release build was slower with a restored target (167-187 s against 133 s)"


def test_the_matrix_shards_are_all_of_one_to_n_and_are_exactly_the_native_test_files(workflow):
    matrix = workflow["jobs"]["differential"]["strategy"]["matrix"]
    shards = matrix["shard"]
    total = int(shards[0].split("/")[1])
    assert shards == [f"{index}/{total}" for index in range(1, total + 1)], "every shard of 1..N is a leg, with the SAME N"
    files = shard_tests.discover(ROOT / "tests", "test_native_*.py")
    assert len(files) >= 20 and "test_native_ci_workflow.py" in files and "test_native_skeleton_closure.py" in files, "the suite the shards split is the real one"
    default, weights = shard_tests.load_weights(SHARD_WEIGHTS)
    groups = shard_tests.assign(files, default, weights, total)
    assert shard_tests.problems(files, groups) == [], "the shards are disjoint, none is empty, together they are tests/test_native_*.py"
    assert sorted(name for group in groups for name in group) == files
    legs = [(python, slots, shard) for python in matrix["python"] for slots in matrix["slots"] for shard in shards]
    covered = {(python, slots): sorted(name for python_, slots_, shard in legs if (python_, slots_) == (python, slots) for name in groups[int(shard.split("/")[0]) - 1]) for python in matrix["python"] for slots in matrix["slots"]}
    assert all(names == files for names in covered.values()), "every test file runs on every (interpreter, slot mode) leg, as before the matrix was sharded"


def test_every_leg_proves_its_shard_and_runs_only_the_files_of_that_shard(workflow):
    job = workflow["jobs"]["differential"]
    assert job["env"]["SHARD"] == "${{ matrix.shard }}"
    step = _step(job, "The shard of the differential tests")
    run = step["run"]
    default = "--root tests --glob \"test_native_*.py\" --weights tests/data/native_ci_shard_weights.json"
    assert f'kernel/tools/shard_tests.py check {default} --shards "${{SHARD#*/}}"' in run
    assert f'kernel/tools/shard_tests.py list {default} --shard "$SHARD" > "$RUNNER_TEMP/shard-files.txt"' in run, "the list goes to a file: a failing `list` fails the step, an empty one never reaches pytest"
    strict = _step(job, "Differential tests against the live oracle")
    assert strict["run"].index("""mapfile -t FILES < <(tr -d '\\r' < "$RUNNER_TEMP/shard-files.txt")""") < strict["run"].index('"${FILES[@]}"'), "the list is read without a CR (a Windows text stream ends its lines with CR LF)"
    steps = job["steps"]
    assert steps.index(step) < steps.index(strict), "the shard is proved before the tests run"
    architecture = _step(job, "Architecture rules of the native port")
    assert architecture["if"] == "matrix.slots == 'auto' && startsWith(matrix.shard, '1/')", "once per interpreter, as before: on the `auto` leg of the first shard"


def test_the_weights_of_the_native_shards_are_a_well_formed_table_of_real_files():
    document = json.loads(SHARD_WEIGHTS.read_text(encoding="utf-8"))
    assert document["schema"] == shard_tests.WEIGHTS_SCHEMA and document["default_seconds"] > 0
    files = set(shard_tests.discover(ROOT / "tests", "test_native_*.py"))
    assert set(document["seconds"]) <= files, f"weights of files that are gone: {sorted(set(document['seconds']) - files)}"
    assert sum(document["seconds"].values()) > 300, "the table was measured on a real run, not invented"


def test_the_pinned_dependencies_of_the_workflow_name_every_package_the_corpus_key_pins():
    text = WORKFLOW.read_text(encoding="utf-8")
    line = re.search(r"^  TEST_REQUIREMENTS: (.+)$", text, re.MULTILINE).group(1)
    assert {item.split("==")[0] for item in line.split()} >= {"pytest", "sympy", "mpmath", "pyyaml"}, "mpmath's version is part of the answer: it is pinned like sympy's, and so it is part of the key"
    assert "mpmath==1.3.0" in line and "mpmath==1.3.0" in (ROOT / "tests" / "requirements.txt").read_text(encoding="utf-8"), "the workflow pins what the host requirements and the kernel pin"


def test_density_pinned_status_and_tracked_live_oracle_seed_are_mandatory_in_every_strict_leg(workflow):
    differential = workflow['jobs']['differential']
    strict = _step(differential, 'Differential tests against the live oracle')
    assert strict['env']['CFTUV_NATIVE_STRICT'] == '1'
    assert '-m "not native_field"' in strict['run'] and ' -k ' not in strict['run']
    assert set(differential['strategy']['matrix']['python']) == {'3.11', '3.13'}
    assert set(differential['strategy']['matrix']['slots']) == {'auto', 'raw', 'attr'}
    available = _step(differential, 'All native operations are available')
    script = available['run'].split("<<'PYCODE'", 1)[1].rsplit('PYCODE', 1)[0]
    guards = [node for node in ast.parse(script).body if isinstance(node, ast.If)
              and isinstance(node.test, ast.Compare) and isinstance(node.test.left, ast.Name)
              and node.test.left.id == 'status']
    assert len(guards) == 1 and isinstance(guards[0].test.ops[0], ast.NotEq)
    assert ast.literal_eval(guards[0].test.comparators[0])['density'] == 'available'
    assert any(isinstance(node, ast.Raise) for node in ast.walk(guards[0])), 'density stale/unavailable must fail before pytest'
    module = ast.parse((ROOT/'tests/test_native_density_batch.py').read_text(encoding='utf8'))
    required = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                    and node.name == 'test_native_density_batch_seed_matches_live_oracle')
    assert [ast.unparse(item.func) for item in required.decorator_list] == ['pytest.mark.parametrize'], 'the tracked seed is not field/skip tier'
    call = next(node for node in ast.walk(required) if isinstance(node, ast.Call) and ast.unparse(node.func) == 'corpus.compare_case')
    assert ast.unparse(call.args[2]) == 'cftuv_native.density_compute', 'CI uses the pinned public operation, never candidate bypass'
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'skip_unless_available'
               and ast.literal_eval(node.args[1]) == 'density' for node in ast.walk(module))
    seed = ROOT/'tests/data/native_density_batch_inputs.json.xz'
    manifest = seed.with_name('native_density_batch_inputs.manifest.json')
    assert seed.is_file() and manifest.is_file()
    assert changes.is_native_path(seed.relative_to(ROOT).as_posix())
    assert changes.is_native_path(manifest.relative_to(ROOT).as_posix())
    assert changes.is_native_path('tools/density_raw_wire.py')
    spec = importlib.util.spec_from_file_location('ci_density_seed_reader', ROOT/'tools/native_density_batch_corpus.py')
    reader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reader)
    cases = reader.read_seed()  # verifies the portable seed and its manifest hashes/cardinality
    assert len(cases) >= 20 and set(reader.LAWS) == {'NARROW_ROTATION_BAND', 'VORONOI_OF_NEIGHBOUR_MIDPOINTS'}
    assert {item['selection']['q'] for case in cases for item in case['specs']} == set(range(2, 7))
    assert 'density' not in corpus.KINDS and 'corpus-density' not in workflow['jobs'], 'tracked live-oracle seed needs no generated corpus job/cache'


def test_all_density_test_files_have_explicit_estimates_and_reach_every_matrix_leg(workflow):
    density = {path.name for path in (ROOT/'tests').glob('test_native_density_*.py')}
    document = json.loads(SHARD_WEIGHTS.read_text(encoding='utf8'))
    assert {'test_native_density_batch.py', 'test_native_density_work_replay.py'} <= density
    assert density <= set(document['seconds'])
    assert density <= set(document['estimate_provenance'])
    for name in density:
        assert document['seconds'][name] > 0
        status = document['estimate_provenance'][name]['status']
        assert status in {'PENDING_CI_MEASUREMENT', 'MEASURED_CI'}
        assert document['estimate_provenance'][name]['basis']
        if status == 'MEASURED_CI':
            assert document['estimate_provenance'][name]['evidence'], 'a measured weight needs its CI run/log source'
    files = shard_tests.discover(ROOT/'tests', 'test_native_*.py')
    default, weights = shard_tests.load_weights(SHARD_WEIGHTS)
    matrix = workflow['jobs']['differential']['strategy']['matrix']
    groups = shard_tests.assign(files, default, weights, len(matrix['shard']))
    assert shard_tests.problems(files, groups) == []
    for _python in matrix['python']:
        for _slots in matrix['slots']:
            assert all(sum(name in group for group in groups) == 1 for name in density)
