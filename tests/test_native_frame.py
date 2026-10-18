"""The native `frame` operation (the first rung of the planar-grid admission of the worker EXPORT stage) ANSWERS-OR-DECLINES against the kernel builder, which stays the oracle.

The port either builds the very `RationalAffinePlanarMetricV2` that `build_rational_affine_planar_metric` builds for a patch exactly planar after the source snap (equal, equal hash, and the process
counters of the symbolic backend moved as the builder moves them), or it DECLINES by name (`NativePortUnsupported`) and the builder runs exactly as before. It never refuses a patch itself. An answer
the builder would not give would be a silent change of the product (AGENTS.md rule 4), so that is what is tested, with the builder on the other side:

* the tracked seed (`tests/data/native_frame_seed.xz`: recorded calls of the seam `_rational_affine_metric`, the smallest of every outcome of the field scenes - answered with 0, 1 and 2 restored right
  corners and with one and two scale trials, declined PLANE / GRID / LAW, a decline where the builder raises - and one large answered call per scene);
* MUTANTS of those calls (`tools/native_frame_mutations.py`): coordinates moved by 10^-15 ... 10^-3 and by one ulp, a vertex lifted off the plane, the patch scaled, moved, turned, mirrored, collapsed,
  faces reversed, dropped, doubled, given away, edges and normals replaced, ids renamed, inexact value types, the laws; a VIOLATION is "the port answered and the builder did not give that answer";
* the owner's FIELD corpus (the calls of the cold press of the frozen scenes, recorded through the worker's `task_inputs`), tier `native_field`: not in CI.
"""

from __future__ import annotations

import os
import sys
from collections import Counter
from decimal import localcontext
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка операции frame с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "frame")

import native_frame_corpus as corpus  # noqa: E402
import native_frame_mutations as mut  # noqa: E402

from cftuv_envelope import backend  # noqa: E402
from cftuv_native import frame_op, pin  # noqa: E402

SEED = ROOT / "tests" / "data" / "native_frame_seed.xz"
FIELD_BASE = Path(os.environ.get("CFTUV_NATIVE_CORPUS") or "E:/cftuv_native_corpus") / "snapshot_check"
FIELD_FILES = sorted(FIELD_BASE.glob("*.tasks.xz")) if FIELD_BASE.is_dir() else []
needs_field = field_tier(bool(FIELD_FILES), f"нет полевого корпуса задач ({FIELD_BASE}): `CFTUV_SNAPSHOT_RECORD_DIR=<каталог> blender -b <сцена> --python tools/blender_tree_ab.py -- --workers 12 ...`")
CHECKED: Counter = Counter()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native frame compared with the kernel builder (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def _seed() -> list:
    assert SEED.is_file(), f"the tracked seed is missing: {SEED} (`python tools/native_frame_corpus.py replay <corpus> --write-seed tests/data/native_frame_seed.xz`)"
    return corpus.read_calls(SEED, corpus.SEED_SCHEMA)


SEED_ROWS = _seed() if SEED.is_file() else []
CATEGORIES = {label: corpus.category_of(cftuv_native, arguments) for label, arguments in SEED_ROWS}


def _of_kind(kind: str) -> list:
    return [row for row in SEED_ROWS if CATEGORIES[row[0]].startswith(kind)]


def _label(row) -> str:
    return row[0].split(" [")[0].replace("host-v0:patch-domain:", "")


def _answered():
    return _of_kind("accepted")[0][1]


# --------------------------------------------------------------------------
# the seed
# --------------------------------------------------------------------------


def test_the_seed_shows_every_outcome_the_port_has_to_get_right():
    """The seed is a SELECTION; the guard is on what the live port says of it (a refresh that loses a category would still pass every row)."""

    categories = Counter(CATEGORIES.values())
    for corners in (0, 1, 2):
        assert any(name.startswith(f"accepted corners={corners}") for name in categories), categories
    assert any(name.endswith("trials=2") for name in categories), "a call whose scale search tried two scales"
    for reason in ("PLANE", "GRID", "LAW"):
        assert any(name.startswith(f"declined {reason}") for name in categories), (reason, categories)
    assert any(name.endswith(" raised") for name in categories), "a call the builder raises on is declined by name"
    assert "violation" not in categories
    assert sum(1 for _label_, arguments in _of_kind("accepted") if corpus.size_of(arguments) >= 30) >= 3, "large answered calls: the embedding certificate over many faces"


@pytest.mark.parametrize("row", SEED_ROWS, ids=_label)
def test_the_native_answer_equals_the_builder_or_is_declined_by_name_on_every_row_of_the_seed(row):
    label, arguments = row
    outcome, detail = corpus.compare(cftuv_native, arguments)
    assert outcome != corpus.VIOLATION, detail
    if "[accepted" in label:
        assert outcome == corpus.EQUAL, f"the seed recorded this call answered by the port; now: {detail}"
        CHECKED["seed answered"] += 1
    else:
        assert outcome == corpus.DECLINED and detail.startswith("frame declined: "), detail
        CHECKED["seed declined by name"] += 1


# --------------------------------------------------------------------------
# the mutants: soundness
# --------------------------------------------------------------------------


def test_no_mutant_of_the_seed_is_answered_otherwise_than_the_builder():
    """Every mutation of every row of the seed: the number of violations is zero, and the port still answers most of what the builder answers on the first rung."""

    summary = mut.run(cftuv_native, SEED_ROWS, seed=1, quiet=True)
    assert not summary["violations"], summary["violations"][:3]
    assert summary["mutants"] >= 1200, summary["mutants"]
    # a port that declines everything would be sound and worthless
    assert summary["equal"] >= 0.4 * summary["mutants"], (summary["equal"], summary["mutants"])
    assert set(summary["declined"]) <= {"LAW", "SHAPE", "PLANE", "GRID", "FRAME", "RANGE", "MODE"}, summary["declined"]
    assert {"LAW", "SHAPE", "PLANE", "GRID"} <= set(summary["declined"]), summary["declined"]
    assert {"raised PlanarMetricAdmissionError", "raised ValueError"} <= set(summary["oracle"]), "the mutants reach the refusals and the exceptions of the builder"
    for kind in ("NUDGE", "ULP", "LIFT", "SCALE", "ROTATE", "REVERSE", "RENAME", "COARSE"):
        assert summary["accepted_labels"][kind], f"no {kind} mutant was answered: the generator no longer reaches the answered path"
    CHECKED["mutants"] += summary["mutants"]
    CHECKED["mutants answered"] += summary["equal"]
    CHECKED["mutants declined"] += sum(summary["declined"].values())


# --------------------------------------------------------------------------
# the reader is strict: what it does not know exactly is a named decline, never an answer
# --------------------------------------------------------------------------


class _StrSubclass(str):
    pass


class _FloatSubclass(float):
    pass


def _declines(arguments: dict, fragment: str) -> None:
    with pytest.raises(cftuv_native.NativePortUnsupported, match=fragment):
        cftuv_native.frame_metric(**corpus.native_arguments(arguments))


def test_a_value_that_is_not_exactly_the_class_of_its_field_is_declined_by_name():
    arguments = _answered()
    vertices = sorted(arguments["source_vertices"], key=lambda item: item.vertex_id.value)
    first = vertices[0]

    def with_first(replacement):
        return {**arguments, "source_vertices": frozenset([replacement, *vertices[1:]])}

    _declines(with_first(mut.make(type(first), vertex_id=type(first.vertex_id)(_StrSubclass(first.vertex_id.value)), position=first.position)), "SHAPE")
    _declines(with_first(mut._replace(first, position=mut._replace(first.position, x=_FloatSubclass(first.position.x)))), "SHAPE")
    _declines(with_first(mut._replace(first, position=mut._replace(first.position, x=int(first.position.x)))), "SHAPE")
    _declines({**arguments, "source_faces": tuple(arguments["source_faces"])}, "SHAPE")
    _declines({**arguments, "source_vertices": list(vertices)}, "SHAPE")
    CHECKED["shape declines"] += 5


def test_the_port_declines_when_the_process_is_not_in_the_state_it_mirrors():
    arguments = _answered()
    assert corpus.compare(cftuv_native, arguments)[0] == corpus.EQUAL
    # the count of restored right corners relies on the rational path of the angle certificates, written in 28 digits in the AMBIENT context: a narrower one is a named decline
    with localcontext() as context:
        context.prec = 20
        _declines(arguments, "MODE")
    with localcontext() as context:
        context.prec = 60
        assert corpus.compare(cftuv_native, arguments)[0] == corpus.EQUAL
    # the retry law and a law that does not snap are the builder's
    from cftuv_envelope.contracts.metric import GridScaleLawV1

    for law in GridScaleLawV1:
        _declines({**arguments, "grid_scale_law": law}, "LAW")
    _declines({**arguments, "grid_policy": type(arguments["grid_policy"]).UNSNAPPED_EXACT_V1}, "LAW")
    CHECKED["process state"] += 1


# --------------------------------------------------------------------------
# the slot modes, the pin, the dispatcher
# --------------------------------------------------------------------------


_CHILD = """
import json, sys
sys.path[:0] = [sys.argv[1], sys.argv[2]]
import cftuv_native
import native_frame_corpus as corpus
layouts = cftuv_native.frame_raw_layouts()
answers = []
for label, arguments in corpus.read_calls(sys.argv[3], corpus.SEED_SCHEMA):
    try:
        answers.append([label, "metric", corpus.digest(cftuv_native.frame_metric(**corpus.native_arguments(arguments)))])
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
    """The raw slot access and the attribute protocol read the same call and answer the same (`CFTUV_NATIVE_SLOTS`, read once per process: a child process per mode)."""

    layouts = cftuv_native.frame_raw_layouts()
    assert all(layouts.values()) is (cftuv_native.slot_mode() != "attr"), (cftuv_native.slot_mode(), layouts)
    here = []
    for label, arguments in SEED_ROWS:
        try:
            here.append([label, "metric", corpus.digest(cftuv_native.frame_metric(**corpus.native_arguments(arguments)))])
        except cftuv_native.NativePortUnsupported as error:
            here.append([label, "declined", str(error)])
    child = _in_a_process_with_slots(slots)
    assert child["mode"] == slots and child["raw"] is (slots != "attr") and child["classes"] == len(layouts)
    assert child["answers"] == here
    CHECKED[f"slot mode {slots}"] += len(SEED_ROWS)


def test_the_stage_is_one_of_the_native_operations_and_the_pin_holds_its_files():
    assert "frame" in pin.OPERATION_FILES
    assert {"planar_metric.py", "source_grid.py", "robust/grid.py", "robust/snapping.py", "_embedding.py"} <= set(pin.OPERATION_FILES["frame"])
    assert backend.native_status().frame == "available"
    live = frame_op.shape_digests(frame_op._kernel_modules())
    assert all(live[(file, name)] == wanted for file, name, wanted in frame_op.SHAPE_FUNCTIONS), "the digests are the ones of THIS tree"


def test_a_moved_oracle_function_stales_the_operation_before_it_reads_anything(monkeypatch):
    file, name, _wanted = frame_op.SHAPE_FUNCTIONS[0]
    monkeypatch.setattr(frame_op, "SHAPE_FUNCTIONS", (*frame_op.SHAPE_FUNCTIONS[1:], (file, name, "0" * 64)))
    frame_op._KERNEL[:] = []
    try:
        with pytest.raises(cftuv_native.NativePortStale, match=name.split(".")[-1]):
            cftuv_native.frame_metric(**corpus.native_arguments(_answered()))
    finally:
        monkeypatch.undo()
        frame_op._KERNEL[:] = []
    assert corpus.compare(cftuv_native, _answered())[0] == corpus.EQUAL


def test_the_ordered_stage_answers_natively_and_a_declined_call_is_left_to_the_builder_by_name():
    answered, declined = _answered(), _of_kind("declined PLANE")[0][1]
    expected = corpus.oracle(answered)[0]
    # the stage not ordered: no ledger, the dispatcher does nothing
    assert backend.frame_compute(**corpus.native_arguments(answered)) is None
    with backend.use_backend("PYTHON", embedding_backend="NATIVE", frame_backend="NATIVE") as ledger:
        got = backend.frame_compute(**corpus.native_arguments(answered))
        assert backend.frame_compute(**corpus.native_arguments(declined)) is None
    assert got == expected and hash(got) == hash(expected)
    record = ledger.record()
    assert record.frame_native_calls == 1 and record.frame_python_calls == 1 and record.frame_ran == "mixed"
    assert [item[0] for item in record.frame_fallbacks] == ["NATIVE_PORT_UNSUPPORTED"] and "declined: PLANE" in record.frame_fallbacks[0][3], record.frame_fallbacks
    # the port computes the embedding certificate of the snap itself: an embedding stage ordered on PYTHON is a named decline, not a silent breach of the order
    with backend.use_backend("PYTHON", embedding_backend="PYTHON", frame_backend="NATIVE") as ledger:
        assert backend.frame_compute(**corpus.native_arguments(answered)) is None
    assert ledger.record().frame_fallbacks[0][0] == "NATIVE_PORT_UNSUPPORTED" and "embedding stage is ordered on PYTHON" in ledger.record().frame_fallbacks[0][3]
    CHECKED["dispatcher"] += 3


def test_the_reader_can_be_bound_before_the_first_domain_and_a_moved_oracle_does_not_make_the_warm_up_raise(monkeypatch):
    """`warm_frame` is what a pool worker does in its start (`warm_worker_identities`): the one-time cost of the process (digests of the mirrored functions, the class table) is not paid by the first domain."""

    frame_op._KERNEL[:] = []
    assert backend.warm_frame() is True and frame_op._KERNEL, "bound"
    before = frame_op._KERNEL[0]
    assert backend.warm_frame() is True and frame_op._KERNEL[0] is before, "bound once"
    file, name, _wanted = frame_op.SHAPE_FUNCTIONS[0]
    monkeypatch.setattr(frame_op, "SHAPE_FUNCTIONS", (*frame_op.SHAPE_FUNCTIONS[1:], (file, name, "0" * 64)))
    frame_op._KERNEL[:] = []
    try:
        assert backend.warm_frame() is False
        with backend.use_backend("PYTHON", embedding_backend="NATIVE", frame_backend="NATIVE") as ledger:
            assert backend.frame_compute(**corpus.native_arguments(_answered())) is None
        assert ledger.record().frame_fallbacks[0][0] == "NATIVE_PORT_STALE", ledger.record().frame_fallbacks
    finally:
        monkeypatch.undo()
        frame_op._KERNEL[:] = []
    assert backend.warm_frame() is True
    CHECKED["warm-up"] += 1


def test_a_stale_pin_is_a_named_fallback_of_the_dispatcher_not_an_answer(monkeypatch):
    original = pin.read_source

    def reading(root, file):
        found = original(root, file)
        return found if file != "planar_metric.py" or found is None else found + b"\n# the oracle moved\n"

    monkeypatch.setattr(pin, "read_source", reading)
    pin.refresh()
    try:
        status = cftuv_native.native_status()
        assert status["frame"] == "stale(planar_metric.py)"
        assert all(status[name] == "available" for name in status if name not in ("frame", "angular", "chain_station")), status
        with backend.use_backend("PYTHON", embedding_backend="NATIVE", frame_backend="NATIVE") as ledger:
            assert backend.frame_compute(**corpus.native_arguments(_answered())) is None
        assert ledger.record().frame_native_calls == 0 and ledger.record().frame_python_calls == 1
        assert ledger.record().frame_fallbacks[0][0].startswith("NATIVE_"), ledger.record().frame_fallbacks
    finally:
        monkeypatch.undo()
        pin.refresh()
    assert cftuv_native.native_status()["frame"] == "available"


# --------------------------------------------------------------------------
# the field
# --------------------------------------------------------------------------


@needs_field
@pytest.mark.parametrize("path", FIELD_FILES, ids=lambda path: path.name)
def test_the_native_frame_equals_the_builder_on_the_field_corpora(path):
    """Every call of the builder from the seam, through the worker's `task_inputs` on every task the cold press of a frozen scene sent to a worker: equal, or declined by name."""

    calls = corpus.record_calls(path.parent, name=path.name.split(".")[0])
    summary = corpus.replay(cftuv_native, calls, quiet=True)
    assert not summary["violations"], summary["violations"][:2]
    assert summary["calls"] >= 1 and summary["equal"] + sum(summary["declined"].values()) == summary["calls"]
    CHECKED[f"field {path.name}"] += summary["calls"]
    CHECKED["field declined"] += sum(summary["declined"].values())
