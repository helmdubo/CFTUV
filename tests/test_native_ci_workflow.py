"""Приёмка Rust отчитывается ВСЕГДА (`.github/workflows/native.yml`): задача `changes`, вердикт `native-gate`, структура workflow.

Обязательная проверка у защищённой ветки не должна зависеть от того, запустился ли workflow: прежние фильтры путей на уровне workflow на хост-коммите оставляли `native-gate` «ожидаемым» навсегда.
Теперь workflow идёт на каждом пуше и PR, а нужны ли тяжёлые задачи, решает `tools/native_ci_changes.py` по тому же набору путей. Здесь это держится без GitHub:

1. набор путей и его синтаксис (`**`, `*`, `?` как у фильтров GitHub);
2. решение `changes` на настоящих временных git-репозиториях: пуш `before..sha`, новая ветка и пуш с недоступным `before` (merge-base с `origin/main`), пуш уже проверенного коммита main (его собственный diff),
   PR `base...head` (ушедшая вперёд база не считается), ручной запуск, ЛЮБАЯ ошибка = `true`;
3. таблица истинности `native-gate` (`tools/native_ci_gate.py`): все сочетания результатов против независимо записанного правила;
4. структура самого workflow (разбор YAML): нет фильтров путей, у каждой тяжёлой задачи `needs: changes` и условие на `native == 'true'`, `native-gate` с `if: always()` и всеми нужными `needs`, `changes` берёт только `actions/checkout`.
"""

from __future__ import annotations

import itertools
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import native_ci_changes as changes  # noqa: E402
import native_ci_gate as gate  # noqa: E402

WORKFLOW = ROOT / ".github" / "workflows" / "native.yml"
ZERO = changes.ZERO_SHA

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
        "kernel/src/cftuv_envelope/**", "kernel/tests/**", "kernel/pyproject.toml", "pytest.ini", ".github/workflows/native.yml",
    )
    assert all(changes.is_native_path(path) for path in ("tools/native_ci_changes.py", "tools/native_ci_gate.py", "tests/test_native_ci_workflow.py")), "the files of this very mechanism are native-relevant"


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
HEAVY = {"rust": "success", "wheel": "success", "corpus": "success", "differential": "success"}


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
    assert checked == 5 * 4 * 5**4


def test_the_gate_program_reads_the_needs_from_the_environment_and_exits_nonzero_on_failure(capsys):
    environment = {"CHANGES_RESULT": "success", "NATIVE": "true", "RUST_RESULT": "success", "WHEEL_RESULT": "success", "CORPUS_RESULT": "success", "DIFFERENTIAL_RESULT": "success"}
    assert gate.main(environment) == 0 and "NATIVE_GATE_PASSED" in capsys.readouterr().out
    assert gate.main({**environment, "DIFFERENTIAL_RESULT": "skipped"}) == 1
    out = capsys.readouterr().out
    assert "NATIVE_GATE_FAILED" in out and "differential=skipped" in out
    assert gate.main({}) == 1, "an empty environment (a broken wiring) is a failure, not a pass"
    assert gate.main({**environment, "NATIVE": "false", "RUST_RESULT": "skipped", "WHEEL_RESULT": "skipped", "CORPUS_RESULT": "skipped", "DIFFERENTIAL_RESULT": "skipped"}) == 0


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
    for name in ("rust", "wheel", "corpus", "differential", "interpreters-advisory"):
        assert "changes" in _needs(jobs[name]), f"{name}: needs `changes`"
        assert jobs[name].get("if") == "needs.changes.outputs.native == 'true'", f"{name}: runs only when `changes` said native=true"
    assert "wheel" in _needs(jobs["corpus"]) and set(_needs(jobs["differential"])) >= {"wheel", "corpus"}, "the earlier chains are kept"


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
        **{f"{name.upper()}_RESULT": f"${{{{ needs.{name}.result }}}}" for name in gate.HEAVY_JOBS},
    }


def test_the_workflow_installs_the_yaml_reader_this_test_needs(workflow):
    assert "pyyaml==" in workflow["env"]["TEST_REQUIREMENTS"], "the strict differential job parses the workflow: PyYAML is among its exact dependencies"
