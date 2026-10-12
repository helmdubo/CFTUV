"""The native `materialize_w1` operation (W1 of the MATERIALIZE chain: `mesh.exact_mesh`, `mesh.canonical_triangles`) equals the Python oracle: the same answer AND the same cost.

The oracle is `kernel/src/cftuv_envelope/materialize/mesh.py` (pinned whole, `cftuv_native.pin`): the vertices of the merged faces of a domain, their exact `(s, r)`, their faces under the law of
the domain, and the second (canonical) triangulation that pays again. Equality is everything the oracle leaves: the result with the type of every node (an `int` coefficient is not a `Fraction`,
the tally is a `Counter` in first-touch order with its zero entries), the exception with its text and the runs of a station conflict, the six articles of the budget, `SIGN_COUNTS`, the four tables
of the canonicalization memory WITH their order, and the lines appended to `notes` in place. A refusal of the PORT is by name and leaves all of it as it was.

Sources: the tracked seed `tests/data/native_materialize_w1_kernel_suite.recs.xz` (the smallest calls of the kernel tests, of mutations of real domains and of budgets cut below their cost that between
them show every outcome and every number of the tally), the part seams of the tessellation of one contour (generated polygons and UV maps, `tools/native_materialize_w1_seams.py`), and the OWNER'S FIELD
corpora (the cold press of `building`, `cover.008`, ..., their mutations and budgets: `E:/cftuv_native_corpus/materialize_w1`, tier `native_field`: not in CI).
"""

from __future__ import annotations

import os
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка операции materialize_w1 с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "materialize_w1")

import native_corpus as nc  # noqa: E402
import native_materialize_w1_corpus as w1  # noqa: E402
import native_materialize_w1_seams as seams  # noqa: E402

from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.materialize import mesh  # noqa: E402

SEED = ROOT / "tests" / "data" / "native_materialize_w1_kernel_suite.recs.xz"
FIELD_BASE = Path(os.environ.get(nc.CORPUS_ENVIRONMENT) or nc.DEFAULT_CORPUS_BASE) / "materialize_w1"
FIELD_CORPORA = sorted(path.parent for path in FIELD_BASE.glob("*/index.json")) if FIELD_BASE.is_dir() else []
needs_field = field_tier(bool(FIELD_CORPORA), f"нет полевых корпусов W1 ({FIELD_BASE}): `blender -b <сцена> --python tools/blender_tree_ab.py -- --worker ...` с CFTUV_W1_RECORD_DIR")
SEAM_CASES = int(os.environ.get("CFTUV_W1_SEAM_CASES", "300"))
CHECKED: Counter = Counter()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native materialize_w1 compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def _seed() -> list:
    assert SEED.is_file(), f"the tracked seed is missing: {SEED} (`python tools/native_materialize_w1_corpus.py seed ...`)"
    return w1.read_seed(SEED)


SEED_RECORDS = _seed() if SEED.is_file() else []


def _id(record: nc.Record) -> str:
    return record.meta["id"] + ":" + str(record.meta.get("label", ""))


# --------------------------------------------------------------------------
# the seed
# --------------------------------------------------------------------------


@pytest.mark.parametrize("record", SEED_RECORDS, ids=_id)
def test_the_native_operation_equals_the_oracle_on_every_record_of_the_seed(record):
    replayed = w1.replay_record(record)
    assert not replayed.oracle_diffs, f"the seed no longer describes the oracle (regenerate it with the pin): {[str(item) for item in replayed.oracle_diffs[:3]]}"
    assert replayed.refusal is None, f"the port refused a call it carries: {replayed.refusal}"
    assert not replayed.native_diffs, [str(item) for item in replayed.native_diffs[:4]]
    CHECKED[f"seed {record.op}"] += 1
    CHECKED["seed oracle exceptions"] += replayed.expected_exception is not None


def test_the_seed_shows_every_outcome_the_port_has_to_get_right():
    """The seed is a SELECTION; the guard is on what it selects (a refresh that loses a branch would still pass every record)."""

    names: set = set()
    raised: set = set()
    operations: set = set()
    charged: set = set()
    for record in SEED_RECORDS:
        expected = record.expected()
        operations.add(record.op)
        if expected.exception is not None:
            raised.add(expected.exception[0])
            if "operation=" in expected.exception[1]:
                charged.add(expected.exception[1].split("operation=", 1)[1].split()[0])
        if expected.result is not None and record.op == nc.OP_MESH:
            names.update(name for name, _value in expected.result.tally.items())
    assert operations == {nc.OP_MESH, nc.OP_CANONICAL}
    assert {"MaterializationRefusal", "ExactCanonicalizationWorkBudgetExhausted", "KeyError"} <= raised, raised
    assert len(charged) >= 2, f"budgets cut below the cost of a call: the exhaustion at several charge sites, {sorted(charged)}"
    needed = {
        mesh.POLYGON_FACES_EMITTED,
        mesh.QUADS_UV_BILINEAR,
        mesh.MERGED_RUNS_SPLIT_AT_RUNGS,
        mesh.RUNG_STATIONS_FROM_CHAIN_VERTEX,
        mesh.FAN_FACES_CUT_BY_NEIGHBOUR,
        mesh.POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
        "MATERIALIZE_CONVEX_PARTITION_PIECES",
    }
    assert needed <= names, sorted(needed - names)


# --------------------------------------------------------------------------
# the part seams: one contour at a time
# --------------------------------------------------------------------------


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_generated_contours_and_fans_are_tessellated_as_the_oracle_does(seed):
    """Random star polygons and fan cells with rational and square-root coordinates and a UV map that is affine, kinked or folded: polygons, tally, exceptions, cost."""

    tally: Counter = Counter()
    for case in seams.cases(seed, SEAM_CASES):
        differences, refusal = seams.compare(case)
        assert refusal is None, f"{case['id']}: the port refused a contour it carries: {refusal}"
        assert not differences, (case["id"], differences[:3])
        tally["cases"] += 1
        outcome = seams.run(case, False)
        tally["oracle exceptions"] += outcome[1] is not None
        tally.update(name for name, _value in outcome[2][1])
    for name in ("MATERIALIZE_CONVEX_PARTITION_PIECES_EXACT", "MATERIALIZE_POLYGONS_UV_BILINEAR", "MATERIALIZE_FAN_FACES_CUT_BY_NEIGHBOUR", "MATERIALIZE_POLYGON_FACES_TRIANGULATED_NOT_SIMPLE"):
        assert tally[name], f"the generator no longer reaches {name} at seed {seed}"
    for key, value in tally.items():
        CHECKED[f"seams {key}"] += value


# --------------------------------------------------------------------------
# what a record cannot show
# --------------------------------------------------------------------------


def _a_mesh_record() -> nc.Record:
    for record in SEED_RECORDS:
        expected = record.expected()
        if record.op == nc.OP_MESH and expected.exception is None and len(expected.result.cycles) > 2:
            return record
    raise AssertionError("the seed has no answered mesh record")


def test_the_result_reuses_the_objects_the_oracle_returns():
    """A point of a cycle is the very tuple of the face, `points[key]` is the FIRST point of that key, and a key is one string wherever it appears."""

    record = _a_mesh_record()
    before = record.before()
    call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
    frame_faces = call.args[0]
    answer = cftuv_native.new_mirror().exact_mesh(*call.args[:7], call.budget, *call.args[7:], None)
    faces = [frame_face.face.points for frame_face in frame_faces]
    for face_points, cycle in zip(faces, answer.cycles):
        for _key, point in cycle:
            assert any(point is candidate for candidate in face_points), "the point of a cycle is an object of its face"
    first_seen: dict = {}
    for cycle in answer.cycles:
        for key, point in cycle:
            first_seen.setdefault(key, point)
    assert set(first_seen) == set(answer.points) and all(answer.points[key] is first_seen[key] for key in answer.points)
    keys = {id(key): key for cycle in answer.cycles for key, _point in cycle}
    by_name: dict = {}
    for key in keys.values():
        by_name.setdefault(key, set()).add(id(key))
    assert all(len(ids) == 1 for ids in by_name.values()), "a key is one string object"
    assert [list(face) for face in answer.polygons] and all(key in answer.points for face in answer.polygons for polygon in face for key in polygon)
    CHECKED["objects"] += 1


def test_a_refusal_of_the_port_after_the_computation_leaves_every_state_as_it_was():
    record = _a_mesh_record()
    before = record.before()
    for kind in ("unsupported", "invalid_input", "diverged", "internal"):
        call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
        notes_before = list(call.args[4])
        mirror = cftuv_native.new_mirror()
        mirror.force_refusal(kind)
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            mirror.exact_mesh(*call.args[:7], call.budget, *call.args[7:], None)
        after = nc.capture_state(call.budget, None)
        assert w1.state_unchanged(before, after) == [], kind
        assert list(call.args[4]) == notes_before, "a refusal of the port appends nothing to notes"
        # and the oracle answers on the very same state as a pure run of the oracle does
        redone = nc.execute(call)
        assert redone.exception is None or redone.exception[0] != "NativePortUnsupported"
    CHECKED["port refusals leave the state"] += 4


def test_verify_plain_affine_is_a_named_refusal_before_any_state(monkeypatch):
    record = _a_mesh_record()
    before = record.before()
    call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
    monkeypatch.setattr(mesh, "VERIFY_PLAIN_AFFINE", True)
    with pytest.raises(cftuv_native.NativePortUnsupported, match="VERIFY_PLAIN_AFFINE"):
        cftuv_native.new_mirror().exact_mesh(*call.args[:7], call.budget, *call.args[7:], None)
    assert w1.state_unchanged(before, nc.capture_state(call.budget, None)) == []


@pytest.mark.parametrize("slots", ["attr", "raw"])
def test_the_slot_modes_give_the_same_answer(slots):
    """The raw slot access and the attribute protocol read the same inputs and build the same results (`CFTUV_NATIVE_SLOTS`)."""

    mirror = cftuv_native.new_mirror(slots=slots)
    for record in SEED_RECORDS[:12]:
        replayed = w1.replay_record(record, mirror.exact_mesh if record.op == nc.OP_MESH else mirror.canonical_triangles)
        assert not replayed.native_diffs and replayed.refusal is None, (slots, record.meta["id"])
    CHECKED[f"slot mode {slots}"] += 12


# --------------------------------------------------------------------------
# the dispatcher
# --------------------------------------------------------------------------


def test_the_ordered_stage_computes_natively_and_records_it_and_a_declined_call_is_computed_by_the_oracle_by_name(monkeypatch):
    record = _a_mesh_record()
    before = record.before()
    call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
    arguments = (*call.args[:7], call.budget, *call.args[7:], None)
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        ran = backend.exact_mesh_compute(mesh.exact_mesh, *arguments)
    assert ledger.record().materialize_native_calls == 1 and ledger.record().materialize_python_calls == 0 and ledger.record().materialize_ran == "native"
    # the stage not ordered: no ledger, the oracle itself
    call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
    oracle = backend.exact_mesh_compute(mesh.exact_mesh, *call.args[:7], call.budget, *call.args[7:], None)
    assert nc.canonical(nc.answer_view(nc.OP_MESH, oracle)) == nc.canonical(nc.answer_view(nc.OP_MESH, ran))

    def declining(*_arguments):
        raise cftuv_native.NativePortUnsupported("declined for the test")

    monkeypatch.setattr(cftuv_native, "exact_mesh", declining)
    call = nc.prepare_call(nc.OP_MESH, record.call_blob, before)
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        fallen = backend.exact_mesh_compute(mesh.exact_mesh, *call.args[:7], call.budget, *call.args[7:], None)
    named = ledger.record()
    assert named.materialize_native_calls == 0 and named.materialize_python_calls == 1
    assert [item[0] for item in named.materialize_fallbacks] == ["NATIVE_PORT_UNSUPPORTED"] and named.materialize_ran == "python"
    assert nc.canonical(nc.answer_view(nc.OP_MESH, fallen)) == nc.canonical(nc.answer_view(nc.OP_MESH, oracle))
    CHECKED["dispatcher"] += 3


def test_the_stage_is_one_of_the_native_operations_and_the_pin_holds_its_files():
    from cftuv_native import pin

    assert "materialize_w1" in pin.OPERATION_FILES and "materialize/mesh.py" in pin.OPERATION_FILES["materialize_w1"]
    assert "materialize/convex_partition.py" in pin.OPERATION_FILES["materialize_w1"]
    assert backend.native_status().materialize_w1 == "available"


# --------------------------------------------------------------------------
# the field
# --------------------------------------------------------------------------


@needs_field
@pytest.mark.parametrize("directory", FIELD_CORPORA, ids=lambda path: path.name)
def test_the_native_operation_equals_the_oracle_on_the_field_corpora(directory):
    """Every record of a corpus of the owner's field: the cold press of the frozen scenes, their mutations and their budgets (replay by the oracle, then by the port)."""

    summary = w1.run_replay(directory, quiet=True)
    assert not summary["oracle_drift"], f"{directory.name}: the corpus no longer describes the oracle: {summary['oracle_drift'][:2]}"
    assert not summary["refusal_state_moved"], summary["refusal_state_moved"][:2]
    assert not summary["refusals"], f"{directory.name}: the port refused calls it carries: {summary['refusals']}"
    assert not summary["mismatches"], f"{directory.name}: {len(summary['mismatches'])} of {summary['records']} records differ, first: {summary['mismatches'][0]}"
    CHECKED[f"field {directory.name}"] += summary["records"]
