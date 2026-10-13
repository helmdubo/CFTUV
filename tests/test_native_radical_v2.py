"""The native capability `radical_v2` (`native/cftuv-radical`, behind `cftuv_native.radical_v2_eval`) equals the Python oracle `reference/radical_v2.py` (`RadicalSumV2`): the same values as wires, the same
texts, signs, enclosures, the same THREE WORK COUNTERS and the same named refusals (code, detail and the work done before them).

The unit is one operation PROGRAM (`tools/native_radical_v2_corpus.py`): the oracle runs it, the extension runs it, and every instruction's outcome tuple must be equal. Sources:

* the tracked seed `tests/data/native_radical_v2_ops.recs.xz`: a deterministic stratified sample of the operations the V1 recorder saw (the effective-alpha field corpus and the whole kernel suite), read BY VALUE
  (`tools/radical_v2_shadow.py`), with the price caps and the separation bound asked of every operand that has three classes or more;
* synthetic programs: deep sums, hidden squares, big integers, caps, every refusal and every error of the readers;
* the constructive near-zero families (sqrt2 - 1)^n and (2 - sqrt3)^m, including (3000, 2000) and (4000, 2700) under the cap of 4096;
* the owner's FIELD records (`E:/cftuv_native_corpus`, written by `tools/radical_v2_shadow.py field|kernel`): tier `native_field`, not in CI;
* the vector file of `cargo test` (`native/cftuv-radical/tests/vectors.txt`), which must be what the oracle writes now.

The capability is not a product stage: it is pinned (`reference/radical_v2.py`), reported by `native_status()["radical_v2"]`, and not among the Rust-only stages.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка возможности radical_v2 с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "radical_v2")

import native_radical_v2_corpus as corpus  # noqa: E402
import native_radical_v2_vectors as vectors  # noqa: E402

from cftuv_envelope.reference import radical_v2 as v2  # noqa: E402

SYNTHETIC = int(os.environ.get("CFTUV_RADICAL_V2_SYNTHETIC", "1500"))
FIELD_RECORDS = Path(os.environ.get("CFTUV_RADICAL_V2_RECORDS", "E:/cftuv_native_corpus/radical_v2"))
CHECKED: Counter = Counter()

needs_field = field_tier(FIELD_RECORDS.is_dir() and any(FIELD_RECORDS.glob("ops-*.pkl.xz")), f"нет полевых записей операций ({FIELD_RECORDS}): `python tools/radical_v2_shadow.py field --out <dir>` и `kernel --out <dir>`")


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native radical_v2 compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def settle(tally: corpus.Tally, label: str) -> dict:
    report = tally.report()
    assert report["instructions"] and not report["differ"], f"{label}: {report['differ']} of {report['instructions']} outcomes differ, first: {json.dumps(report['mismatches'][:2], default=str)[:3000]}"
    for key in ("programs", "instructions", "equal", "refusals_equal", "other_errors_equal"):
        CHECKED[f"{label}: {key}"] += report[key]
    return report


# --------------------------------------------------------------------------
# the sources
# --------------------------------------------------------------------------


def test_the_tracked_seed_of_recorded_operations_equals_the_oracle():
    rows = corpus.read_seed()
    tally = corpus.Tally()
    corpus.compare_rows(rows, tally)
    report = settle(tally, "seed")
    kinds = Counter(row[0] for row in rows)
    assert set(kinds) >= {"add", "sub", "mul", "neg", "scaled", "inverse", "signum", "enclosure", "as_rational", "sqrt_of_rational", "rational_ratio", "canonical_text", "terms_wire"}, kinds
    assert len(rows) > 3000 and report["instructions"] > 12000, report
    # the shapes the field never makes are in the seed: several classes, refusals by cap, sums of more than three classes
    assert report["outcomes"].get("r", 0) > 50 and report["outcomes"].get("e", 0) > 100 and report["outcomes"].get("s", 0) > 1000, report["outcomes"]
    assert any(len(corpus._terms_of(row[1])) >= 3 for row in rows if row[1] is not None), "sums of three classes and more are in the seed"


def test_synthetic_programs_equal_the_oracle_and_every_refusal_is_there():
    tally = corpus.Tally()
    for program in corpus.synthetic_programs(SYNTHETIC, big=True):
        corpus.compare_program(program, tally)
    report = settle(tally, "synthetic")
    assert report["outcomes"].get("r", 0) > 100 and report["outcomes"].get("x", 0) > 10 and report["outcomes"].get("z", 0) > 10, report["outcomes"]
    # every named refusal of the algebra was produced and agreed on (the codes are those of the oracle)
    seen = Counter()
    for program in corpus.synthetic_programs(max(300, SYNTHETIC // 5), seed=77, big=False):
        for outcome in corpus.run_native(program)[0]:
            if outcome[0] == "r":
                seen[outcome[1]] += 1
    assert set(seen) == {v2.EXACT_NATIVE_OUTSIDE_FIELD, v2.EXACT_NATIVE_SIGN_UNDECIDED, v2.EXACT_NATIVE_INVERSE_CLASS_LIMIT}, seen


def test_near_zero_families_equal_the_oracle_and_the_cap_of_4096_is_a_named_price():
    tally = corpus.Tally()
    for program in corpus.near_zero_programs():
        corpus.compare_program(program, tally)
    settle(tally, "near zero")
    # (sqrt2 - 1)^n + (2 - sqrt3)^m > 0 near 2^-5100: the eighth rung (8192 bits) decides, a cap of 4096 refuses with the work of the seven rungs it paid
    ((a, b), (c, d)) = corpus.pell_powers(4000, 2700)
    program = [("terms", ((1, a + c, 1), (2, b, 1), (3, d, 1))), ("signum", 0, None), ("signum", 0, 4096), ("signum", 0, 8192)]
    native = corpus.run_native(program)[0]
    assert native == corpus.run_oracle(program)
    assert native[1] == ("s", 1, (0, 0, 8)) and native[3] == ("s", 1, (0, 0, 8))
    assert native[2][:3] == ("r", v2.EXACT_NATIVE_SIGN_UNDECIDED, (0, 0, 7)) and native[2][3].endswith(" bits, the cap is 4096")


def test_the_price_vectors_of_the_kernel_test_hold_in_the_extension():
    expected = vectors.price_vectors()
    for name, program in vectors.price_programs().items():
        program = [*program, ("text_of", len(program) - 1)]
        outcomes = corpus.run_native(program)[0]
        text, work = expected[name]
        assert outcomes[-1] == ("t", text) and outcomes[-2][0] == "v" and outcomes[-2][2] == work, (name, outcomes[-2:])
    CHECKED["price vectors"] += len(expected)


def test_the_vector_file_of_cargo_test_is_what_the_oracle_writes_now():
    path = vectors.DEFAULT_OUT
    written = "\n".join(vectors.lines(200)) + "\n"
    assert path.read_bytes().decode("utf-8").replace("\r\n", "\n") == written, "run `python tools/native_radical_v2_vectors.py write --random 200` (the oracle moved, or the generator did)"
    CHECKED["vector file bytes"] += len(written)


@needs_field
def test_the_owners_field_records_equal_the_oracle():
    merged = corpus.load_records([FIELD_RECORDS])
    rows = sorted((key for key in merged if key[0] != "measure"), key=corpus._row_key)
    tally = corpus.Tally()
    corpus.compare_rows(rows, tally)
    settle(tally, "field")


# --------------------------------------------------------------------------
# the boundary
# --------------------------------------------------------------------------


def test_integers_of_any_size_and_sign_cross_the_boundary_exactly():
    big = 3**5000 + 7
    program = [("rational", -big, big - 1), ("rational", big, -(big + 2)), ("mul", 0, 1), ("wire_of", 2), ("scaled", 2, -(10**400), 3), ("enclosure", 4, 300), ("sqrt", big * big, 1), ("text_of", 6)]
    assert corpus.run_native(program)[0] == corpus.run_oracle(program)


def test_a_defect_of_the_program_is_a_value_error_naming_the_instruction():
    for program, needle in (
        ([("frobnicate", 1)], "instruction 0"),
        ([("zero",), ("add", 0)], "instruction 1"),
        ([("rational", 1.5, 2)], "an int expected"),
        ([("zero",), ("signum", 0, "x")], "instruction 1"),
        ([("terms", ((1, 2),))], "terms"),
        (["add"], "not a tuple"),
    ):
        with pytest.raises(ValueError, match=needle):
            cftuv_native.radical_v2_eval(program)
    # a register that holds no value is the defect `?`, not an outcome of the oracle
    assert cftuv_native.radical_v2_eval([("sqrt", -1, 1), ("neg", 0)])[0][1][0] == "?"


def test_the_timings_are_per_instruction_and_the_outcomes_are_those_of_a_plain_run():
    program, tags = corpus.program_of_rows(corpus.read_seed()[:300])
    plain = cftuv_native.radical_v2_eval(program)
    timed = cftuv_native.radical_v2_eval(program, passes=2, inner=3)
    assert plain[2] is None and plain[1] > 0
    assert timed[0] == plain[0] and len(timed[2]) == len(program) == len(tags) and all(value >= 0 for value in timed[2])


def test_the_outcomes_do_not_depend_on_the_hash_seed_of_the_process():
    script = (
        "import hashlib, json\n"
        "import native_radical_v2_corpus as c\n"
        "rows = []\n"
        "for program in c.synthetic_programs(120, seed=5, big=True):\n"
        "    rows.append(c.run_native(program)[0])\n"
        "print(hashlib.sha256(json.dumps(rows, default=str).encode()).hexdigest())\n"
    )
    digests = set()
    for seed in ("0", "1", "4242"):
        environment = {**os.environ, "PYTHONHASHSEED": seed, "PYTHONSAFEPATH": "1", "PYTHONPATH": os.pathsep.join(filter(None, [str(ROOT / "kernel" / "src"), str(ROOT / "tools"), os.environ.get("PYTHONPATH", "")]))}
        done = subprocess.run([sys.executable, "-c", script], env=environment, capture_output=True, text=True, check=False)
        assert done.returncode == 0, done.stderr[-2000:]
        digests.add(done.stdout.strip())
    assert len(digests) == 1, digests
    CHECKED["hash seeds"] += 3


# --------------------------------------------------------------------------
# the pin
# --------------------------------------------------------------------------


def test_the_capability_is_pinned_reported_and_not_a_rust_only_stage():
    pin = cftuv_native.pin
    assert pin.OPERATION_FILES["radical_v2"] == pin.RADICAL_V2_FILES == ("reference/radical_v2.py",)
    assert cftuv_native.native_status()["radical_v2"] == "available"
    digest = hashlib.sha256((ROOT / "kernel" / "src" / "cftuv_envelope" / "reference" / "radical_v2.py").read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    assert pin.PINS["reference/radical_v2.py"] == digest
    import ast

    architecture = ast.parse((ROOT / "tests" / "test_architecture.py").read_text(encoding="utf-8"))
    rust_only = next(ast.literal_eval(node.value) for node in architecture.body if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "RUST_ONLY_OPERATIONS")
    assert "radical_v2" not in rust_only, "radical_v2 is a capability, not a stage the product runs: its oracle may still move"


def test_a_moved_oracle_stales_the_capability_and_dependent_density_and_refuses_by_name():
    pin = cftuv_native.pin
    original = pin.read_source

    def moved(root, file):
        found = original(root, file)
        return found if file != "reference/radical_v2.py" or found is None else found + b"\n# the oracle moved\n"

    pin.read_source = moved
    pin.refresh()
    try:
        status = cftuv_native.native_status()
        assert status["radical_v2"] == "stale(reference/radical_v2.py)"
        assert status["density"] == "stale(reference/radical_v2.py)"
        assert all(status[name] == "available" for name in status if name not in ("radical_v2", "density")), status
        with pytest.raises(cftuv_native.NativePortStale, match="radical_v2.py"):
            cftuv_native.radical_v2_eval([("zero",)])
    finally:
        pin.read_source = original
        pin.refresh()
    assert cftuv_native.native_status()["radical_v2"] == "available"
    assert cftuv_native.radical_v2_eval([("zero",)])[0] == [("v", (), (0, 0, 0))]
