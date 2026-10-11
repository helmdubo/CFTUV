"""The synthetic clip corpus built by several pytest processes at once (`tools/native_clip_synthetic.py build --processes N`, used by the `corpus-clip` CI job).

The clip corpus keeps every call a test made (no repeats are dropped), so the merge is a concatenation in the order a single process would have met the calls. What is proved here:

* the merge orders the recorded calls and the seam files by the position of the test file, stable within a file, and sums the skips and takes the worst exit status;
* a process of a parallel build writes its part and no corpus;
* end to end (needs the extension: the seam calls are encoded by the shim): on real kernel test files the corpus of 2 processes has the same records in the same order (same ids, same test, same label, same
  outcome label, same inputs) and the same seam files as the corpus of 1. The expected outcomes may differ in the counters of the oracle when its process was warm (the single process replays the oracle after
  the tests ran in it, the parent of a parallel build replays it cold: both are valid answers of the oracle from the recorded state, and the differential tests compare the native side with a LIVE oracle).
"""

from __future__ import annotations

import json
import lzma
import pickle
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_clip_synthetic as clip  # noqa: E402
import native_corpus as nc  # noqa: E402

FILES = ("test_a.py", "test_b.py", "test_c.py")


def entry(file: str, name: str, label: str = "clip_geometry") -> dict:
    return {"test": f"tests/{file}::{name}", "label": label, "before": None, "blob": f"{file}{name}".encode()}


def seam(file: str, name: str) -> dict:
    return {"test": f"tests/{file}::{name}", "calls": [], "seen": {}}


def part(pending, seam_files, skipped=None, exit_code=0) -> dict:
    return {"pending": pending, "seam_files": seam_files, "skipped": skipped or {}, "exit": exit_code}


def test_the_parts_are_merged_in_the_order_of_the_files_and_nothing_is_dropped():
    first = part([entry("test_c.py", "t1"), entry("test_a.py", "t1"), entry("test_a.py", "t1")], [seam("test_c.py", "t1"), seam("test_a.py", "t1")], {"ValueError": 1})
    second = part([entry("test_b.py", "t1"), entry("test_b.py", "t2", "ClipStageV1.run")], [seam("test_b.py", "t2")], {"ValueError": 2, "live: X": 1}, exit_code=1)
    pending, seam_files, skipped, exit_code = clip.merge_parts([first, second], FILES)
    assert [(item["test"].split("::")[0][6:], item["test"].split("::")[1]) for item in pending] == [
        ("test_a.py", "t1"), ("test_a.py", "t1"), ("test_b.py", "t1"), ("test_b.py", "t2"), ("test_c.py", "t1"),
    ], "files in the given order; a call a test made twice is two records"
    assert [item["test"] for item in seam_files] == ["tests/test_a.py::t1", "tests/test_b.py::t2", "tests/test_c.py::t1"]
    assert skipped == Counter({"ValueError": 3, "live: X": 1}) and exit_code == 1


def test_a_process_of_a_parallel_build_writes_its_part_and_no_corpus(tmp_path, monkeypatch, capsys):
    collector = clip._Collector(tmp_path / "corpus")
    collector.pending.append(entry("test_a.py", "t"))
    collector.seam_files.append(seam("test_a.py", "t"))
    target = tmp_path / "part.pkl"
    monkeypatch.setenv(clip.ITEMS_ENVIRONMENT, str(target))
    clip.write_corpus(collector, 0)
    assert not (tmp_path / "corpus").exists(), "the parent writes the corpus"
    written = pickle.loads(target.read_bytes())
    assert written["pending"][0]["blob"] == b"test_a.pyt" and written["seam_files"][0]["test"] == "tests/test_a.py::t" and written["exit"] == 0
    assert "synthetic clip part" in capsys.readouterr().out


def test_the_clip_files_are_split_into_groups_that_are_exactly_the_files():
    for processes in range(1, 6):
        groups = clip.split_files(clip.TEST_FILES, processes)
        assert sorted(name for group in groups for name in group) == sorted(clip.TEST_FILES) and all(groups)


SMALL = ("test_clip_law.py", "test_clip_faces_law.py")


def _extension():
    try:
        import cftuv_native  # noqa: F401  (the family of native tests may import it; it is the shim that encodes the seam calls)
    except ModuleNotFoundError as error:
        if error.name != "cftuv_native":
            raise
        pytest.skip("the extension is not installed: the seam calls of the recorder are encoded by the shim")


def _shape(root: Path) -> tuple:
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))
    records = []
    for row in index["records"]:
        record = nc.read_record(root / row["path"])
        records.append((row["id"], row["mesh"], row.get("label"), row["outcome"], row["op"], row["budget"], record.op))
    seams = []
    for entry_ in index["seam_files"]:
        calls = pickle.loads(lzma.decompress((root / entry_["path"]).read_bytes()))
        seams.append((entry_["test"], entry_["path"], len(calls)))
    return records, seams, index


def test_two_processes_record_the_same_calls_in_the_same_order_as_one(tmp_path):
    _extension()
    serial, parallel = tmp_path / "serial", tmp_path / "parallel"
    assert clip.build(serial, SMALL) == 0
    assert clip.build(parallel, SMALL, processes=2) == 0
    first, second = _shape(serial), _shape(parallel)
    assert len(first[0]) >= 100, "the files reach enough calls to be a comparison"
    assert first[0] == second[0], "the same records: ids, tests, labels, outcome labels, in the same order"
    assert [item[0] for item in first[1]] == [item[0] for item in second[1]] and [item[1] for item in first[1]] == [item[1] for item in second[1]], "the same seam files in the same order"
    for key in ("records_count", "labels", "generated_outcomes", "kernel_identity", "tests", "pytest_exit"):
        assert first[2][key] == second[2][key], key
