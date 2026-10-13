"""The native `snapshot_check` operation (the snapshot validators of the worker EXPORT stage) is CLEAN-OR-DECLINE against the Python validators, which stay the oracle.

The port either PROVES that `validate_analysis_snapshot` (under the request budget) and `validate_reference_geometry_payload` (every domain) would report nothing, and leaves on the process
counters of the symbolic backend the very increments the validators would have left; or it DECLINES by name (`NativePortUnsupported`) and the validators run exactly as before. It never refuses a
snapshot itself. A clean answer for a snapshot the validators do not pass would be a silent acceptance (AGENTS.md rule 4), so soundness is what is tested, with the oracle on the other side:

* the tracked seed (`tests/data/native_snapshot_check_seed.xz`: the smallest real domains of the field scenes, canonical codec bytes: 270 degree reflex corners, other reflex corners, convex
  domains, and the domains the port declines by name: near-planar and developable charts);
* MUTANTS of those snapshots (`tools/native_snapshot_check_mutations.py`): every field of every record perturbed, one injection per `ValidationCode` and per reference outcome, junction relations
  of all five kinds (the host writes none), and mutants at the decision boundary of the reflex angle (the ends of the certified intervals moved by 10^-3 ... 10^-40, the supports of a 270 degree
  corner nudged, turned, swapped). A VIOLATION is "native attested, Python refuses / raises / leaves other counters"; the number of violations must be zero;
* the owner's FIELD corpus (the cold press of the frozen scenes, replayed through the worker's `task_inputs` under both stage orders: bytes, request, refusal and counters equal), tier
  `native_field`: not in CI.
"""

from __future__ import annotations

import os
import sys
from collections import Counter
from decimal import Decimal, localcontext
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка операции snapshot_check с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "snapshot_check")

import native_snapshot_check_mutations as mut  # noqa: E402

from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.reference import symbolic_backend  # noqa: E402
from cftuv_native import pin, snapshot_check_op  # noqa: E402

SEED = ROOT / "tests" / "data" / "native_snapshot_check_seed.xz"
FIELD_BASE = Path(os.environ.get("CFTUV_NATIVE_CORPUS") or "E:/cftuv_native_corpus") / "snapshot_check"
FIELD_FILES = sorted(FIELD_BASE.glob("*.tasks.xz")) if FIELD_BASE.is_dir() else []
needs_field = field_tier(bool(FIELD_FILES), f"нет полевого корпуса snapshot_check ({FIELD_BASE}): `CFTUV_SNAPSHOT_RECORD_DIR=<каталог> blender -b <сцена> --python tools/blender_tree_ab.py -- --workers 12 ...`")
MUTANTS_PER_SNAPSHOT = int(os.environ.get("CFTUV_SNAPSHOT_CHECK_MUTANTS", "30"))
CHECKED: Counter = Counter()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native snapshot_check compared with the Python validators (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def _seed() -> list:
    assert SEED.is_file(), f"the tracked seed is missing: {SEED} (`python tools/native_snapshot_check_mutations.py <corpus> --write-seed tests/data/native_snapshot_check_seed.xz`)"
    return mut.read_seed(SEED)


SEED_ROWS = _seed() if SEED.is_file() else []
CATEGORIES = {label: mut.seed_category(cftuv_native, snapshot)[0] for label, snapshot, _budget in SEED_ROWS}


def _of_kind(kind: str) -> list:
    return [row for row in SEED_ROWS if CATEGORIES[row[0]].startswith(kind)]


@pytest.fixture(autouse=True)
def _default_process_state():
    """The state the verdicts are statements about: the native-exact symbolic backend, a 28-digit decimal context, no backend block."""

    assert symbolic_backend.backend_mode() is symbolic_backend.SymbolicBackendV1.NATIVE_EXACT
    yield


def _label(row) -> str:
    return row[0].split(" [")[0].replace("host-v0:patch-domain:", "")


# --------------------------------------------------------------------------
# the seed
# --------------------------------------------------------------------------


def test_the_seed_shows_every_outcome_the_port_has_to_get_right():
    """The seed is a SELECTION; the guard is on what it selects (a refresh that loses a category would still pass every row)."""

    kinds = Counter(category.split(" ")[0] for category in CATEGORIES.values())
    assert kinds["right-angle"] >= 4 and kinds["reflex"] >= 2 and kinds["convex"] >= 2 and kinds["declined"] >= 3, kinds
    declined = " ".join(category for category in CATEGORIES.values() if category.startswith("declined"))
    assert "NearPlanarProjectionCer" in declined and "DevelopableUnfoldCertif" in declined, "both kinds of chart the port declines by name"
    corners = sum(1 for _label_, snapshot, _budget in _of_kind("right-angle") for node in mut.nodes_of(snapshot) if type(node).__name__ == "CertifiedReflexAngleMeasureV1" and mut.is_right_angle(node))
    assert corners >= 6, corners
    budgets = {budget for _label_, _snapshot, budget in SEED_ROWS}
    assert None in budgets and len(budgets) >= 2, "the request budget comes both ways: absent (cover) and present"


@pytest.mark.parametrize("row", SEED_ROWS, ids=_label)
def test_the_native_answer_is_sound_and_its_counters_equal_the_validators_on_every_row_of_the_seed(row):
    label, snapshot, budget = row
    found, verdict, ((export, export_why), (request, request_why)) = mut.violations_of(cftuv_native, snapshot, budget)
    assert not found, found
    assert mut.export_clean(verdict) and mut.request_clean(verdict), "the seed is made of snapshots the validators pass"
    if CATEGORIES[label].startswith("declined"):
        assert export is None and export_why.startswith("snapshot_check declined: UNSUPPORTED: "), export_why
        assert request is None, "a chart the port does not carry is declined under both scopes"
        CHECKED["seed declined by name"] += 1
        return
    assert export is not None and request is not None, (export_why, request_why)
    backend_counts = verdict["counters"]["backend"]
    assert (export.certified, export.straddled) == (backend_counts.get("angle_certificate_sign.certified", 0), backend_counts.get("angle_certificate_sign.exact_fallback.straddles_zero", 0))
    assert request.corners == 0 and export.domains >= 1, "the request scope checks no reference payload, the export scope checks every domain"
    CHECKED["seed attested"] += 1
    CHECKED["seed corners certified"] += export.corners


def test_the_270_degree_corners_of_the_seed_are_decided_natively_with_the_counters_of_the_validators():
    right = _of_kind("right-angle")
    assert right
    total = Counter()
    for _label_, snapshot, _budget in right + _of_kind("reflex"):
        answer = cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH)
        symbolic_backend.reset_backend_counts()
        for domain in sorted((item.patch_domain_id for item in snapshot.patch_domains), key=lambda item: item.value):
            mut.reference_validation.validate_reference_geometry_payload(snapshot, domain)
        counts = symbolic_backend.BACKEND_COUNTS
        assert (answer.certified, answer.straddled) == (counts.get("angle_certificate_sign.certified", 0), counts.get("angle_certificate_sign.exact_fallback.straddles_zero", 0))
        assert answer.corners == answer.certified + answer.straddled, "every reflex corner is accounted for by one of the two counters"
        total["corners"] += answer.corners
        total["certified"] += answer.certified
        total["straddled"] += answer.straddled
    # an exact 270 degree corner has the cosine exactly zero: its interval straddles zero and the exact path decides it (that is the counter `exact_fallback.straddles_zero`); the other reflex corners are certified
    assert total["corners"] >= 10 and total["straddled"] >= 6 and total["certified"] >= 2, total
    CHECKED["270 degree snapshots"] += len(right)


# --------------------------------------------------------------------------
# the mutants: soundness
# --------------------------------------------------------------------------


@pytest.mark.parametrize("seed", [1, 2])
def test_no_mutant_of_the_seed_is_attested_that_the_validators_do_not_pass(seed):
    """Every field, every issue class, junctions of every kind, the decision boundary of the reflex angle: the number of violations is zero."""

    boundary_rows = [(snapshot, budget) for _label_, snapshot, budget in _of_kind("right-angle")[:3] + _of_kind("reflex")[:2]]
    plain_rows = [(snapshot, budget) for _label_, snapshot, budget in _of_kind("convex")[:3] + _of_kind("declined")[:3]]
    summary = mut.run(cftuv_native, boundary_rows, None, MUTANTS_PER_SNAPSHOT, seed, junctions=True, boundary=True)
    other = mut.run(cftuv_native, plain_rows, None, MUTANTS_PER_SNAPSHOT, seed, junctions=True, boundary=False)
    for part in (summary, other):
        assert not part["violations"], part["violations"][:3]
        # a port that declines everything would be sound and worthless: most of what the validators pass is attested, the rest is declined by a named reason
        assert part["native_attests"] > 0
        for reason in part["decline_reasons"]:
            assert reason.startswith("snapshot_check declined: "), reason
    assert summary["mutants"] + other["mutants"] >= 2500
    assert summary["native_attests"] >= 0.6 * summary["python_clean"], (summary["native_attests"], summary["python_clean"])
    codes = summary["codes"] + other["codes"]
    for code in ("SCHEMA_VERSION", "CAPABILITY", "DUPLICATE_ID", "MISSING_REFERENCE", "CROSS_CONTRACT_MISMATCH", "ANGULAR_CERTIFICATE", "SURFACE_TOPOLOGY", "SURFACE_METRIC", "ROUTE_TOPOLOGY", "TERMINAL_RELATION", "SOURCE_EDGE_ZERO_LENGTH"):
        assert codes[code], f"no mutant produced the issue class {code}: the generator no longer reaches it"
    assert (summary["diagnostics"] + other["diagnostics"])["REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH"]
    assert summary["by_kind"]["boundary"]["python_refuses"] and summary["by_kind"]["boundary"]["python_clean"], "boundary mutants fall on both sides of the decision"
    assert summary["by_kind"]["right-angle"]["python_refuses"], "a moved support of a 270 degree corner is refused by the validators and never attested"
    assert summary["by_kind"]["junctions targeted"]["python_refuses"], "the junction relations are checked (ROUTE_TOPOLOGY)"
    CHECKED["mutants"] += summary["mutants"] + other["mutants"]
    CHECKED["mutants refused by Python"] += summary["python_refuses"] + other["python_refuses"]
    CHECKED["mutants attested"] += summary["native_attests"] + other["native_attests"]


def test_a_junction_enriched_snapshot_is_attested_exactly_when_the_validators_pass_it():
    for _label_, snapshot, budget in _of_kind("convex")[:1] + _of_kind("right-angle")[:1]:
        enriched = mut.with_junctions(snapshot)
        kinds = {item.relation_kind.name for item in enriched.junction_relations}
        assert {"T", "Y", "CROSS_PATCH", "MIXED_ALPHA"} <= kinds, kinds  # X needs four chain uses
        found, verdict, ((export, why), _request) = mut.violations_of(cftuv_native, enriched, budget)
        assert not found and mut.export_clean(verdict) and export is not None, (found, verdict["export_issues"], why)
        CHECKED["junction snapshots"] += 1


# --------------------------------------------------------------------------
# the reader is strict: what it does not know exactly is a named decline, never a verdict
# --------------------------------------------------------------------------


class _StrSubclass(str):
    pass


def _declines(snapshot, fragment: str) -> None:
    with pytest.raises(cftuv_native.NativePortUnsupported, match=fragment):
        cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH)


def _a_right_angle_snapshot():
    return _of_kind("right-angle")[0][1]


def test_a_value_that_is_not_exactly_the_class_of_its_field_is_declined_by_name():
    snapshot = _a_right_angle_snapshot()
    use = next(item for item in mut.nodes_of(snapshot) if type(item).__name__ == "ChainUseV1")
    _declines(mut.rebuild(snapshot, use, "chain_use_id", _StrSubclass(str(use.chain_use_id))), "SHAPE")
    measure = next(item for item in mut.nodes_of(snapshot) if type(item).__name__ == "CertifiedReflexAngleMeasureV1")
    _declines(mut.rebuild(snapshot, measure.phi_over_pi, "lower", float(measure.phi_over_pi.lower)), "SHAPE")
    _declines(mut.rebuild(snapshot, snapshot, "analysis_capabilities", set(snapshot.analysis_capabilities)), "SHAPE")
    _declines(mut.rebuild(snapshot, snapshot, "chain_uses", list(snapshot.chain_uses)), "SHAPE")
    rational = next(item for item in mut.nodes_of(snapshot) if type(item).__name__ == "ExactRationalV1")
    _declines(mut.rebuild(snapshot, rational, "numerator", True), "SHAPE")
    CHECKED["shape declines"] += 5


def test_a_snapshot_the_validators_refuse_is_declined_under_both_scopes_and_the_declines_are_named_issues():
    snapshot = _a_right_angle_snapshot()
    broken = mut.rebuild(snapshot, snapshot, "schema_version", "cftuv.envelope.analysis_snapshot.v0")
    for scope in (mut.SCOPE_ANALYSIS, mut.SCOPE_BOTH):
        with pytest.raises(cftuv_native.NativePortUnsupported, match="ISSUE"):
            cftuv_native.snapshot_check(broken, scope)
    # and the refusal itself is the validators': the port added nothing
    assert [issue.code.value for issue in mut.validation.validate_analysis_snapshot(broken)] == ["SCHEMA_VERSION"]


def test_the_port_declines_when_the_process_is_not_in_the_state_it_mirrors():
    snapshot = _of_kind("reflex")[0][1]  # the bounds of a reflex corner have 28 digits: `phi - 1` needs them all
    assert cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH).corners >= 1
    previous = symbolic_backend.set_backend_mode(symbolic_backend.SymbolicBackendV1.SYMPY)
    try:
        _declines(snapshot, "MODE")
    finally:
        symbolic_backend.set_backend_mode(previous)
    # `phi.lower - Decimal(1)` is computed in the AMBIENT context: a context too narrow for the digits is the validators' rounding, which the port does not mirror, so it declines
    with localcontext() as context:
        context.prec = 20
        _declines(snapshot, "MODE")
    with localcontext() as context:
        context.prec = 60
        assert cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH).corners >= 1
    assert cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH).corners >= 1
    CHECKED["process state"] += 1


def test_a_decimal_context_the_port_cannot_mirror_never_changes_the_answer_it_gives():
    """The same boundary mutants under a narrow context: whatever the port attests, the validators pass under that very context."""

    snapshot = _a_right_angle_snapshot()
    rng = mut.random.Random(5)
    for precision in (6, 12, 28, 40):
        with localcontext() as context:
            context.prec = precision
            checked = 0
            for label, mutant in list(mut.boundary_mutants(snapshot, rng, 2))[::3]:
                found, verdict, ((export, _why), _request) = mut.violations_of(cftuv_native, mutant, None)
                assert not found, (precision, label, found)
                checked += 1
            assert checked >= 20
    CHECKED["narrow contexts"] += 4


# --------------------------------------------------------------------------
# the slot modes, the pin, the dispatcher
# --------------------------------------------------------------------------


_CHILD = """
import json, sys
sys.path[:0] = [sys.argv[1], sys.argv[2]]
import cftuv_native
import native_snapshot_check_mutations as mut
layouts = cftuv_native.snapshot_check_raw_layouts()
answers = []
for label, snapshot, _budget in mut.read_seed(sys.argv[3]):
    try:
        found = cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH)
        answers.append([label, "clean", found.domains, found.corners, found.certified, found.straddled])
    except cftuv_native.NativePortUnsupported as error:
        answers.append([label, "declined", str(error)])
print(json.dumps({"mode": cftuv_native.slot_mode(), "raw": all(layouts.values()), "classes": len(layouts), "answers": answers}))
"""


def _in_a_process_with_slots(mode: str) -> dict:
    import json
    import subprocess

    environment = {**os.environ, "CFTUV_NATIVE_SLOTS": mode, "PYTHONSAFEPATH": "1"}
    done = subprocess.run([sys.executable, "-c", _CHILD, str(ROOT / "kernel" / "src"), str(ROOT / "tools"), str(SEED)], capture_output=True, text=True, env=environment, check=False)
    assert done.returncode == 0, done.stderr[-2000:]
    return json.loads(done.stdout.strip().splitlines()[-1])


@pytest.mark.parametrize("slots", ["raw", "attr"])
def test_the_slot_modes_give_the_same_answer(slots):
    """The raw slot access and the attribute protocol read the same graph and decide the same (`CFTUV_NATIVE_SLOTS`, read once per process: a child process per mode); the reader says which of its classes it reads raw."""

    layouts = cftuv_native.snapshot_check_raw_layouts()
    assert all(layouts.values()) is (cftuv_native.slot_mode() != "attr"), (cftuv_native.slot_mode(), layouts)
    here = []
    for label, snapshot, _budget in SEED_ROWS:
        try:
            found = cftuv_native.snapshot_check(snapshot, mut.SCOPE_BOTH)
            here.append([label, "clean", found.domains, found.corners, found.certified, found.straddled])
        except cftuv_native.NativePortUnsupported as error:
            here.append([label, "declined", str(error)])
    child = _in_a_process_with_slots(slots)
    assert child["mode"] == slots and child["raw"] is (slots != "attr") and child["classes"] == len(layouts)
    assert child["answers"] == here
    CHECKED[f"slot mode {slots}"] += len(SEED_ROWS)


def test_the_stage_is_one_of_the_native_operations_and_the_pin_holds_its_files():
    assert "snapshot_check" in pin.OPERATION_FILES
    assert {"validation_snapshot.py", "validation_source_edges.py", "reference/angle_certificate_sign.py"} <= set(pin.OPERATION_FILES["snapshot_check"])
    assert backend.native_status().snapshot_check == "available"
    assert len(snapshot_check_op.SHAPE_FUNCTIONS) >= 30, "the functions of the shared oracle files the Rust code mirrors are digested at bind"
    live = snapshot_check_op.shape_digests(snapshot_check_op._kernel_modules())
    assert all(live[(file, name)] == wanted for file, name, wanted in snapshot_check_op.SHAPE_FUNCTIONS), "the digests are the ones of THIS tree"


def test_a_moved_oracle_function_stales_the_operation_before_it_reads_anything(monkeypatch):
    row = SEED_ROWS[0]
    file, name, wanted = snapshot_check_op.SHAPE_FUNCTIONS[0]
    moved = (*snapshot_check_op.SHAPE_FUNCTIONS[1:], (file, name, "0" * 64))
    monkeypatch.setattr(snapshot_check_op, "SHAPE_FUNCTIONS", moved)
    snapshot_check_op._KERNEL[:] = []
    try:
        with pytest.raises(cftuv_native.NativePortStale, match=name.split(".")[-1]):
            cftuv_native.snapshot_check(row[1], mut.SCOPE_BOTH)
    finally:
        monkeypatch.undo()
        snapshot_check_op._KERNEL[:] = []
    assert cftuv_native.snapshot_check(_a_right_angle_snapshot(), mut.SCOPE_BOTH).corners >= 1


def test_the_ordered_stage_computes_natively_and_leaves_the_counters_of_the_validators_and_a_declined_call_is_left_to_them_by_name():
    snapshot = _a_right_angle_snapshot()
    other = _of_kind("declined")[0][1]
    # the validators alone
    symbolic_backend.reset_backend_counts()
    for domain in sorted((item.patch_domain_id for item in snapshot.patch_domains), key=lambda item: item.value):
        mut.reference_validation.validate_reference_geometry_payload(snapshot, domain)
    expected = dict(symbolic_backend.BACKEND_COUNTS)
    # the stage not ordered: no ledger, the dispatcher does nothing
    assert backend.snapshot_check_compute(snapshot, backend.SNAPSHOT_CHECK_EXPORT) is False
    # ordered: native computes, records it, and the process counters move exactly as the validators moved them
    symbolic_backend.reset_backend_counts()
    with backend.use_backend("PYTHON", snapshot_check_backend="NATIVE") as ledger:
        assert backend.snapshot_check_compute(snapshot, backend.SNAPSHOT_CHECK_EXPORT) is True
        assert backend.snapshot_check_compute(other, backend.SNAPSHOT_CHECK_EXPORT) is False
    record = ledger.record()
    assert dict(symbolic_backend.BACKEND_COUNTS) == expected
    assert record.snapshot_check_native_calls == 1 and record.snapshot_check_python_calls == 1
    assert [item[0] for item in record.snapshot_check_fallbacks] == ["NATIVE_PORT_UNSUPPORTED"], record.snapshot_check_fallbacks
    assert record.snapshot_check_fallbacks[0][1] == "snapshot_check" and "declined: UNSUPPORTED" in record.snapshot_check_fallbacks[0][3], record.snapshot_check_fallbacks
    CHECKED["dispatcher"] += 3


def test_a_stale_pin_is_a_named_fallback_of_the_dispatcher_not_a_verdict(monkeypatch):
    snapshot = _a_right_angle_snapshot()
    original = pin.read_source

    def reading(root, file):
        found = original(root, file)
        return found if file != "validation_snapshot.py" or found is None else found + b"\n# the oracle moved\n"

    monkeypatch.setattr(pin, "read_source", reading)
    pin.refresh()
    try:
        assert cftuv_native.native_status()["snapshot_check"] == "stale(validation_snapshot.py)"
        with backend.use_backend("PYTHON", snapshot_check_backend="NATIVE") as ledger:
            assert backend.snapshot_check_compute(snapshot, backend.SNAPSHOT_CHECK_EXPORT) is False
        assert ledger.record().snapshot_check_native_calls == 0 and ledger.record().snapshot_check_python_calls == 1
        assert ledger.record().snapshot_check_fallbacks[0][0].startswith("NATIVE_"), ledger.record().snapshot_check_fallbacks
    finally:
        monkeypatch.undo()
        pin.refresh()
    assert cftuv_native.native_status()["snapshot_check"] == "available"


# --------------------------------------------------------------------------
# the field
# --------------------------------------------------------------------------


@needs_field
@pytest.mark.parametrize("path", FIELD_FILES, ids=lambda path: path.name)
def test_the_native_stage_equals_the_validators_on_the_field_corpora(path):
    """Every task the cold press of a frozen scene sent to a worker, through the worker's `task_inputs` under both orders of the stage: canonical snapshot bytes, request, refusal, counters."""

    import native_snapshot_check_corpus as corpus

    directory = path.parent
    summary = corpus.replay(directory, name=path.name.split(".")[0], quiet=True)
    assert not summary["differences"], summary["differences"][:2]
    assert summary["tasks"] >= 1 and summary["equal"] == summary["tasks"]
    CHECKED[f"field {path.name}"] += summary["tasks"]
    CHECKED["field declined"] += sum(summary["declined"].values())
