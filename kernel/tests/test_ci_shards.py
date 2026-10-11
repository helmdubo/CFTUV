"""CI shards of the kernel suite: `tools/shard_tests.py` splits a test directory into N file sets that are, together, exactly the suite.

The CI runs the kernel suite as N parallel jobs per interpreter (`.github/workflows/envelope-kernel.yml`). A file in no shard is a test that
no job runs, a file in two shards is a test that runs twice, an empty shard makes pytest run the WHOLE directory: all three are named failures
of `problems`. The properties are proved here on synthetic lists and on the real `kernel/tests` for every shard count a matrix could use.
"""

from __future__ import annotations

import importlib.util
import itertools
import json
import random
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
WEIGHTS = TOOLS / "shard_weights.json"


def _load():
    spec = importlib.util.spec_from_file_location("kernel_shard_tests", TOOLS / "shard_tests.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


shards = _load()


def test_every_shard_count_partitions_a_synthetic_suite():
    files = [f"test_{name}.py" for name in "abcdefghijklmnopqrstuvwxyz"]
    weights = {name: float(index * index % 17 + 1) for index, name in enumerate(files)}
    for total in range(1, len(files) + 1):
        groups = shards.assign(files, 2.0, weights, total)
        assert len(groups) == total and shards.problems(files, groups) == []
    assert shards.problems(files, shards.assign(files, 2.0, weights, len(files) + 1)) != [], "more shards than files leaves an empty shard, which is a named failure"


def test_the_assignment_is_a_pure_function_of_the_names_and_the_weights():
    files = [f"test_{number:03d}.py" for number in range(40)]
    weights = {name: float((index * 7) % 13) for index, name in enumerate(files)}
    first = shards.assign(files, 2.0, weights, 5)
    shuffled = list(files)
    random.Random(7).shuffle(shuffled)
    assert shards.assign(shuffled, 2.0, weights, 5) == first, "the order the files were discovered in must not move a test between shards"
    assert shards.assign(files, 2.0, weights, 5) == first


def test_the_heaviest_files_are_spread_and_the_load_is_balanced():
    files = [f"test_{name}.py" for name in "abcdef"]
    weights = {"test_a.py": 100.0, "test_b.py": 100.0, "test_c.py": 50.0, "test_d.py": 50.0, "test_e.py": 1.0, "test_f.py": 1.0}
    groups = shards.assign(files, 2.0, weights, 2)
    loads = sorted(sum(weights[name] for name in group) for group in groups)
    assert loads == [151.0, 151.0]
    assert next(group for group in groups if "test_a.py" in group) != next(group for group in groups if "test_b.py" in group), "the two heaviest files do not share a shard"


def test_a_file_without_a_weight_is_still_in_exactly_one_shard():
    files = ["test_known.py", "test_new_file.py"]
    groups = shards.assign(files, 5.0, {"test_known.py": 100.0}, 2)
    assert shards.problems(files, groups) == [] and sorted(itertools.chain(*groups)) == sorted(files)


def test_no_property_of_the_union_depends_on_the_weights_table(tmp_path):
    root = tmp_path / "tests"
    root.mkdir()
    names = [f"test_{number}.py" for number in range(11)]
    for name in names:
        (root / name).write_text("", encoding="utf-8")
    (root / "helper.py").write_text("", encoding="utf-8")
    (root / "__pycache__").mkdir()
    (root / "__pycache__" / "test_stale.py").write_text("", encoding="utf-8")
    found = shards.discover(root)
    assert found == sorted(names), "helpers and __pycache__ are not part of the suite"
    for seed in range(5):
        rng = random.Random(seed)
        weights = {name: rng.random() * 100 for name in names + ["test_deleted_long_ago.py"]}
        for total in (1, 2, 3, 4, 7, 11):
            assert shards.problems(found, shards.assign(found, 2.0, weights, total)) == []


def test_the_problems_are_named():
    files = ["test_a.py", "test_b.py"]
    assert any("empty" in text for text in shards.problems(files, [["test_a.py", "test_b.py"], []]))
    assert any("in no shard" in text for text in shards.problems(files, [["test_a.py"]]))
    assert any("is in shard 1 and in shard 2" in text for text in shards.problems(files, [["test_a.py", "test_b.py"], ["test_a.py"]]))
    assert any("not files of the suite" in text for text in shards.problems(files, [["test_a.py", "test_b.py", "test_gone.py"]]))
    assert any("no files" in text for text in shards.problems([], [[]]))


@pytest.mark.parametrize("text", ["0/4", "5/4", "1", "a/b", "1/0", "-1/3", "1/4/5"])
def test_a_malformed_shard_is_refused(text):
    with pytest.raises(SystemExit):
        shards.parse_shard(text)


def test_the_shard_is_one_value_so_the_index_and_the_count_cannot_disagree():
    assert shards.parse_shard("2/4") == (2, 4) and shards.parse_shard(" 4/4 ") == (4, 4)


def test_the_real_kernel_suite_is_partitioned_for_every_plausible_shard_count():
    root = Path(__file__).resolve().parent
    files = shards.discover(root)
    assert len(files) > 100 and "test_ci_shards.py" in files, "the suite the shards split is the real one"
    default, weights = shards.load_weights(WEIGHTS)
    for total in range(1, 13):
        groups = shards.assign(files, default, weights, total)
        assert shards.problems(files, groups) == [], total
        assert sorted(itertools.chain(*groups)) == files


def test_the_weights_table_is_well_formed_and_names_real_files():
    document = json.loads(WEIGHTS.read_text(encoding="utf-8"))
    assert document["schema"] == shards.WEIGHTS_SCHEMA and document["default_seconds"] > 0
    files = set(shards.discover(Path(__file__).resolve().parent))
    stale = sorted(set(document["seconds"]) - files)
    assert not stale, f"weights of files that no longer exist (regenerate the table, `tools/shard_tests.py weights`): {stale}"
    assert all(value >= 0 for value in document["seconds"].values())


def test_the_command_line_prints_the_shard_and_refuses_an_empty_or_vacuous_one(tmp_path, capsys):
    root = tmp_path / "tests"
    root.mkdir()
    for name in ("test_a.py", "test_b.py", "test_c.py"):
        (root / name).write_text("", encoding="utf-8")
    printed: list[str] = []
    for index in (1, 2):
        assert shards.main(["list", "--root", str(root), "--shard", f"{index}/2"]) == 0
        printed += capsys.readouterr().out.split()
    assert sorted(Path(item).name for item in printed) == ["test_a.py", "test_b.py", "test_c.py"]
    assert shards.main(["list", "--root", str(root), "--shard", "4/4"]) == shards.EXIT_EMPTY_SHARD, "an empty shard must not print an empty argument list"
    assert shards.main(["check", "--root", str(root), "--shards", "3"]) == 0
    assert shards.main(["check", "--root", str(root), "--shards", "4"]) == 1
    assert shards.main(["check", "--root", str(root), "--shards", "2", "--glob", "nothing_*.py"]) == 1, "a glob that finds nothing is not a passing suite"


def test_the_weights_are_read_from_a_junit_report_and_from_duration_lines():
    junit = (
        '<testsuites><testsuite><testcase classname="tests.test_a" name="one" time="1.5"/>'
        '<testcase classname="tests.test_a" name="two" time="2.0"/><testcase classname="test_b" name="x" time="0.25"/>'
        '<testcase classname="tests.test_c.TestK" name="y" time="3"/></testsuite></testsuites>'
    )
    assert shards.weights_from_junit(junit) == {"test_a.py": 3.5, "test_b.py": 0.25, "test_c.py": 3.0}
    log = (
        "2026-10-11T09:45:00.1058485Z 123.92s call     tests/test_native_x.py::test_the_first\n"
        "2026-10-11T09:45:00.1060359Z 7.50s setup    tests/test_native_x.py::test_the_first\n"
        "  0.80s call     kernel/tests/test_y.py::TestK::test_z[param]\n"
    )
    assert shards.weights_from_durations_log(log) == pytest.approx({"test_native_x.py": 131.42, "test_y.py": 0.8})
    document = json.loads(shards.weights_document({"test_b.py": 2.04, "test_a.py": 1.0}, 2.0))
    assert document["seconds"] == {"test_a.py": 1.0, "test_b.py": 2.0} and document["schema"] == shards.WEIGHTS_SCHEMA
