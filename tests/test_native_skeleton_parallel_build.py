"""The synthetic skeleton corpus built by several pytest processes at once (`tools/native_skeleton_synthetic.py build --processes N`, used by the `corpus-skeleton` CI job).

One process used one of the four cores of a runner and took 11 minutes of the critical path of the native CI. The tests now run in N processes; each collects its distinct calls into a file and the parent
writes the corpus once, in the order a single process would have met the calls. What must hold, and is proved here:

* the files are split into groups that are exactly the files (none lost, none run twice);
* merging the parts keeps the FIRST item of each dedupe key in the order of `files`, the rule a single process applies while the tests run, and counts the duplicates as it does;
* a process of a parallel build writes its part and no corpus;
* end to end, on real kernel test files: the records recorded from the tests are the same records (same ids, same call, same state, same outcome) as the single-process build's.
"""

from __future__ import annotations

import hashlib
import json
import pickle
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402
import native_skeleton_synthetic as synthetic  # noqa: E402

FILES = ("test_a.py", "test_b.py", "test_c.py", "test_d.py")


def item(file: str, name: str, key: str) -> dict:
    return {"test": f"tests/{file}::{name}", "before": None, "blob": key.encode(), "live": None, "key": (key.encode(), b"shape")}


def part(items, calls, duplicates=0, skipped=None, exit_code=0) -> dict:
    return {"items": items, "calls": calls, "duplicates": duplicates, "skipped": skipped or {}, "exit": exit_code}


def test_the_ci_files_are_split_into_groups_that_are_exactly_the_files():
    for processes in range(1, 8):
        groups = synthetic.split_files(synthetic.CI_FILES, processes)
        assert len(groups) == min(processes, len(synthetic.CI_FILES))
        assert sorted(name for group in groups for name in group) == sorted(synthetic.CI_FILES), "every file in exactly one group"
        assert all(groups), "no empty group: an empty process would run the whole directory"
    assert synthetic.split_files(("test_corner_fold.py",), 3) == [("test_corner_fold.py",)], "more processes than files: one group per file"


def test_the_groups_of_the_ci_files_are_balanced_by_the_measured_seconds():
    weights = json.loads((ROOT / "kernel" / "tools" / "shard_weights.json").read_text(encoding="utf-8"))["seconds"]
    loads = sorted(sum(weights.get(name, 2.0) for name in group) for group in synthetic.split_files(synthetic.CI_FILES, 3))
    assert loads[-1] / loads[0] < 1.2, loads


def test_the_parts_are_merged_in_the_order_of_the_files_and_the_first_item_of_a_key_stays():
    # serial order: a (x, y), b (y again, z), c (x again), d (w). Process 1 ran c and a, process 2 ran d and b.
    first = part([item("test_a.py", "t1", "x"), item("test_a.py", "t2", "y"), item("test_c.py", "t1", "w2")], calls=6, duplicates=1)
    second = part([item("test_b.py", "t1", "y"), item("test_b.py", "t2", "z"), item("test_d.py", "t1", "w")], calls=4, duplicates=0)
    first["items"].insert(2, item("test_c.py", "t0", "x"))  # `x` again, in a later file than test_a.py
    items, calls, duplicates, skipped, exit_code = synthetic.merge_parts([first, second], FILES)
    assert [(Path(entry["test"].split("::")[0]).name, entry["key"][0]) for entry in items] == [
        ("test_a.py", b"x"), ("test_a.py", b"y"), ("test_b.py", b"z"), ("test_c.py", b"w2"), ("test_d.py", b"w"),
    ], "files in the given order; the repeats of `x` (test_c.py) and `y` (test_b.py) are gone"
    assert calls == 10 and exit_code == 0
    assert duplicates == 1 + 0 + (7 - 5), "the duplicates found inside each process, and the ones found across them"


def test_the_merge_is_what_one_process_would_have_collected_whatever_the_split():
    streams = {
        "test_a.py": ["k1", "k2", "k1", "k3"], "test_b.py": ["k2", "k4"], "test_c.py": ["k5", "k1"], "test_d.py": ["k4", "k6", "k5"],
    }
    seen, serial = set(), []
    for file in FILES:
        for number, key in enumerate(streams[file]):
            if key not in seen:
                seen.add(key)
                serial.append((file, key))
    calls = sum(len(stream) for stream in streams.values())
    for grouping in ([FILES], [FILES[:2], FILES[2:]], [FILES[3:], FILES[:1], FILES[1:3]], [(name,) for name in reversed(FILES)]):
        parts = []
        for group in grouping:
            local_seen, local, duplicates, calls_here = set(), [], 0, 0
            for file in group:
                for number, key in enumerate(streams[file]):
                    calls_here += 1
                    if key in local_seen:
                        duplicates += 1
                        continue
                    local_seen.add(key)
                    local.append(item(file, f"t{number}", key))
            parts.append(part(local, calls_here, duplicates))
        items, merged_calls, duplicates, _skipped, _exit = synthetic.merge_parts(parts, FILES)
        assert [(Path(entry["test"].split("::")[0]).name, entry["key"][0].decode()) for entry in items] == serial, grouping
        assert merged_calls == calls and duplicates == calls - len(serial), grouping


def test_the_exit_status_and_the_skips_of_all_the_parts_are_kept():
    items, calls, duplicates, skipped, exit_code = synthetic.merge_parts(
        [part([], 2, skipped={"TypeError": 1}, exit_code=0), part([], 3, skipped={"TypeError": 2, "live: X": 1}, exit_code=1)], ("test_a.py",)
    )
    assert (items, calls, duplicates, exit_code) == ([], 5, 0, 1) and skipped == Counter({"TypeError": 3, "live: X": 1})


def test_a_process_of_a_parallel_build_writes_its_part_and_no_corpus(tmp_path, monkeypatch, capsys):
    collector = synthetic._Collector(tmp_path / "corpus")
    collector.pending.append(item("test_a.py", "t", "x"))
    collector.calls, collector.duplicates = 3, 1
    target = tmp_path / "part.pkl"
    monkeypatch.setenv(synthetic.ITEMS_ENVIRONMENT, str(target))
    synthetic.write_corpus(collector, 0)
    assert not (tmp_path / "corpus").exists(), "the parent writes the corpus"
    written = pickle.loads(target.read_bytes())
    assert written["calls"] == 3 and written["duplicates"] == 1 and written["exit"] == 0 and written["items"][0]["key"] == (b"x", b"shape")
    assert "synthetic skeleton part" in capsys.readouterr().out


# --------------------------------------------------------------------------
# end to end: the same records as the single-process build
# --------------------------------------------------------------------------

SMALL = ("test_exact_work_budget.py", "test_density_exact_limit_lift.py", "test_wavefront_weighted_wall_differential.py")


def _records_from_tests(root: Path) -> dict:
    found = {}
    for row in sc.load_index(root)["records"]:
        if row.get("derived") is not None or row.get("label") != "test":
            continue
        record = sc.read(root, row)
        stable = {key: value for key, value in row.items() if key not in ("seconds", "bytes", "git_head", "python")}
        found[row["id"]] = (
            json.dumps(stable, sort_keys=True, default=str), hashlib.sha256(record.call_blob).hexdigest(), hashlib.sha256(repr(record.before()).encode()).hexdigest(),
            nc.outcome_digest(record.op, record.before(), record.expected()),
        )
    return found


def test_the_derived_records_of_two_processes_are_the_derived_records_of_one(tmp_path):
    """The derivation (`native_skeleton_derive.py --processes N`) traces and re-executes records under a cut ceiling; the parent numbers the derived records beforehand, in the order a single
    process walks, so the set, the numbers and the contents are the same."""

    import shutil

    import native_skeleton_derive as derive

    base = tmp_path / "base"
    assert synthetic.build(base, SMALL) == 0
    serial, parallel = tmp_path / "serial", tmp_path / "parallel"
    shutil.copytree(base, serial)
    shutil.copytree(base, parallel)
    first = derive.derive_records(serial, shares=(0.2, 0.6), occurrences=2, processes=1)
    second = derive.derive_records(parallel, shares=(0.2, 0.6), occurrences=2, processes=2)
    assert len(first) >= 20, "the base corpus gives enough derived records to be a comparison"

    def stable(root: Path) -> dict:
        found = {}
        for row in sc.load_index(root)["records"]:
            if row.get("derived") is None:
                continue
            record = sc.read(root, row)
            volatile = {key: value for key, value in row.items() if key not in ("seconds", "bytes", "git_head", "python")}
            found[row["id"]] = (json.dumps(volatile, sort_keys=True, default=str), hashlib.sha256(record.call_blob).hexdigest(), nc.outcome_digest(record.op, record.before(), record.expected()))
        return found

    assert [row["id"] for row in first] == [row["id"] for row in second] == [f"d{number:05d}-build_skeleton" for number in range(1, len(first) + 1)], "the numbers are the order of the walk"
    assert stable(serial) == stable(parallel)
    assert sc.load_index(serial)["derived_count"] == sc.load_index(parallel)["derived_count"] == len(first)


def test_two_processes_record_the_same_records_from_the_tests_as_one(tmp_path):
    serial, parallel = tmp_path / "serial", tmp_path / "parallel"
    assert synthetic.build(serial, SMALL) == 0
    assert synthetic.build(parallel, SMALL, processes=2) == 0
    first, second = _records_from_tests(serial), _records_from_tests(parallel)
    assert len(first) >= 30, "the files reach enough calls to be a comparison"
    assert first == second, "same ids, same call, same state before, same outcome: the parallel build is the same corpus"
    one, two = sc.load_index(serial), sc.load_index(parallel)
    for key in ("test_calls_seen", "test_duplicates", "records_count", "kernel_identity", "tests", "pytest_exit", "test_live_differs", "skipped", "test_duplicates_after_normalization"):
        assert one[key] == two[key], key
    assert two["parallel_processes"] == 2 and "parallel_processes" not in one
