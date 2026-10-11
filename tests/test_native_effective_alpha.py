"""The native `effective_alpha` operation (the pair step of the effective-alpha resolver) equals the Python oracle: the same contacts, the same terms, the same counters.

The oracle is `boundary._contact_candidates(context, source, boundary, frame)` -- a source segment against a barrier segment: the binary64 prefilter `SourceContactFrame.excludes` and the exact leaf
`boundary_native.contact_candidates_native`, applied to the process counters `symbolic_backend.BACKEND_COUNTS` (the two prefilter counters, and `contact_candidates.native`) and
`planar_types.SYMBOLIC_FALLBACK_COUNTS`. The port is one operation behind the same call (`backend.effective_alpha_compute`, stage ordered NATIVE): equality is the answer (every contact with the TERMS of
alpha and station as the oracle's `RadicalSumV1` holds them -- representative of every square class, order of the terms -- and the two texts of the point; or the exception), the delta of BOTH counter
dicts, and nothing else of the process. A pair the port does not carry it DECLINES by name (`NativePortUnsupported`), and the oracle then computes it with its own counts and its own sympy route: a
declined pair is equal by construction, and the tests count them and name what may be declined (a point that is not an `Integer`/`Rational` text, an operation of the algebra that refuses, a contact with an
irrational parameter, a frame the oracle refuses): a pair that does not fall under those reasons MUST be answered by the extension.

Sources: the adversarial pairs of `tools/native_effective_alpha_synthetic.py` (generated here and recorded with the oracle's answers in this process; `CFTUV_EFFECTIVE_ALPHA_SYNTHETIC` is the scale),
the tracked seed `tests/data/native_effective_alpha_kernel_suite.recs.xz` (the pairs the kernel tests make, a deterministic cover of them) and the OWNER'S FIELD corpus (the cold press of the frozen scenes
`building`, `2`, `rounded_wall_noise_top`, `cover.008`, and the whole kernel suite, `E:/cftuv_native_corpus/effective_alpha`; tier `native_field`: not in CI).
"""

from __future__ import annotations

import dataclasses
import json
import os
import pickle
import re
import subprocess
import sys
import types
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка операции effective_alpha с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "effective_alpha")

import native_effective_alpha_corpus as nec  # noqa: E402
import native_effective_alpha_synthetic as synthetic  # noqa: E402

from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.reference import boundary as boundary_module  # noqa: E402
from cftuv_envelope.reference import planar_types  # noqa: E402
from cftuv_envelope.reference import symbolic_backend  # noqa: E402
from cftuv_envelope.reference.boundary_native import SourceContactFrame  # noqa: E402
from cftuv_native import effective_alpha_op  # noqa: E402

SYNTHETIC_SCALE = float(os.environ.get("CFTUV_EFFECTIVE_ALPHA_SYNTHETIC", "0.3"))
KERNEL_SUITE_SEED = ROOT / "tests" / "data" / "native_effective_alpha_kernel_suite.recs.xz"
FIELD_DIRECTORY = nec.corpus_directory() / "field"
FIELD_FILES = sorted(FIELD_DIRECTORY.glob("*.recs.xz")) if FIELD_DIRECTORY.is_dir() else []
needs_field = field_tier(bool(FIELD_FILES), f"нет полевого корпуса пар ({FIELD_DIRECTORY}): `blender -b <копия сцены> --python tools/native_effective_alpha_export.py -- --case building`")
needs_kernel_suite = field_tier((nec.corpus_directory() / "kernel_suite.recs.xz").is_file(), "нет записи всего набора ядра: `python tools/native_effective_alpha_corpus.py kernel-suite`")

#: What a barrier or source point must be written as for the extension to read it (`native_exact._INTEGER_TEXT`, `_RATIONAL_TEXT`; a zero denominator is the oracle's `ZeroDivisionError`).
RATIONAL_TEXT = re.compile(r"\A(?:Integer\(-?[0-9]+\)|Rational\(-?[0-9]+, -?0*[1-9][0-9]*\))\Z")
CHECKED: Counter = Counter()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(
            f"native effective_alpha compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items()))
        )


@pytest.fixture(autouse=True)
def _own_counters():
    saved = dict(symbolic_backend.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    yield
    symbolic_backend.BACKEND_COUNTS.clear()
    symbolic_backend.BACKEND_COUNTS.update(saved[0])
    planar_types.SYMBOLIC_FALLBACK_COUNTS.update(saved[1])


# --------------------------------------------------------------------------
# the comparison
# --------------------------------------------------------------------------


def native_must_answer(document: dict, pair: dict) -> bool:
    """A pair that falls under none of the named reasons to decline: the extension MUST answer it (the oracle took its exact native path, with every point a rational text)."""

    source = document["sources"][pair["source"]]
    return (
        pair["frame"]
        and pair["mode"] != "SYMPY"
        and pair["answer"][0] == "ok"
        and not any(name in pair["counts"] for name in nec.REFUSAL_COUNTERS)
        and not pair["symbolic"]
        and all(RATIONAL_TEXT.match(text) for text in (*pair["start"], *pair["end"], *source["start"]))
    )


def compare_document(document: dict, label: str, *, must_answer: bool = True) -> Counter:
    """Every pair of the document on the three sides (recorded, oracle now, extension): equal answers and equal counter deltas; returns the tallies."""

    objects = nec.Objects(document)
    tally: Counter = Counter()
    mismatches: list = []
    unexpected: list = []
    for pair in document["pairs"]:
        recorded, python, native = nec.recorded_outcome(pair), nec.run_pair(objects, pair, "python"), nec.run_pair(objects, pair, "native")
        tally["pairs"] += 1
        if not (nec.equal_outcomes(recorded, python) and nec.equal_outcomes(python, native)):
            mismatches.append((pair["id"], pair.get("test"), pair["mode"], pair["frame"], recorded, python, native))
            continue
        ledger = native.get("ledger", {"native": 0, "python": 0})
        tally["answered"] += ledger["native"]
        tally["declined"] += ledger["python"]
        tally["oracle pairs (no frame, or none)"] += not (ledger["native"] or ledger["python"])
        if must_answer and native_must_answer(document, pair) and ledger["python"]:
            unexpected.append((pair["id"], pair["mode"], pair["start"], pair["end"], native["ledger"]))
        tally["refused by the oracle"] += any(name in pair["counts"] for name in nec.REFUSAL_COUNTERS)
        tally["contacts"] += len(pair["answer"][1]) if pair["answer"][0] == "ok" else 0
    assert not mismatches, f"{label}: {len(mismatches)} of {tally['pairs']} pairs differ, first: {json.dumps(mismatches[0], default=str)[:3000]}"
    assert not unexpected, f"{label}: {len(unexpected)} pairs the port had no reason to decline were declined, first: {unexpected[0]}"
    for key, value in tally.items():
        CHECKED[f"{label}: {key}"] += value
    return tally


def simple_call(*, boundary_ends=((1, -1), (1, 3)), source=((0, 0), (4, 0)), gram=((1, 0), (0, 1))):
    """`(context, source, boundary, frame)` of one small pair on the identity metric."""

    context = synthetic.metric_of([list(row) for row in gram])
    segment = synthetic.source_of(context, *source)
    barrier = synthetic.boundary_of(synthetic.R(*boundary_ends[0]), synthetic.R(*boundary_ends[1]), "b")
    return context, segment, barrier, SourceContactFrame(context, segment)


def oracle_and_native(call, mode="NATIVE_EXACT", with_frame=True):
    """The pair on both sides with fresh frames (or none); `[oracle outcome, native outcome]`, the native one with the record of its ledger."""

    context, source, boundary, _frame = call
    outcomes = []
    for side in ("python", "native"):
        frame = SourceContactFrame(context, source) if with_frame else None
        before, symbolic_before = dict(symbolic_backend.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
        ledger = None
        with symbolic_backend.symbolic_backend(symbolic_backend.SymbolicBackendV1(mode), symbolic_backend.DisagreementPolicyV1.RECORD):
            try:
                if side == "native":
                    with backend.use_backend(*nec.NATIVE_ORDER) as ledger:
                        found = nec.ORACLE(context, source, boundary, frame)
                else:
                    found = nec.ORACLE(context, source, boundary, frame)
                answer = ["ok", nec.encode_contacts(found)]
            except Exception as error:  # noqa: BLE001
                answer = ["raised", type(error).__name__, str(error)]
        outcomes.append({"answer": answer, "counts": nec.delta(before, dict(symbolic_backend.BACKEND_COUNTS)), "symbolic": nec.delta(symbolic_before, dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)), "record": None if ledger is None else ledger.record()})
    return outcomes


# --------------------------------------------------------------------------
# the boundary
# --------------------------------------------------------------------------


def test_the_dropin_has_the_shape_the_dispatcher_calls():
    import inspect

    assert list(inspect.signature(cftuv_native.effective_alpha_contacts).parameters) == ["source", "boundary", "frame"]
    assert list(inspect.signature(backend.effective_alpha_compute).parameters) == ["oracle", "context", "source", "boundary", "frame", "counts"]
    assert backend.native_status().effective_alpha_available and backend.native_status().as_record()["effective_alpha"] == "available"


def test_the_operation_is_pinned_to_the_files_it_mirrors_and_the_tool_names_the_same_files():
    pin = cftuv_native.pin
    assert pin.OPERATION_FILES["effective_alpha"] == pin.EFFECTIVE_ALPHA_FILES == nec.ORACLE_FILES == ("_cpython311.py", "float_filter.py", "reference/boundary_native.py", "reference/native_exact.py")
    assert cftuv_native.native_status()["effective_alpha"] == "available"
    assert cftuv_native.NativePortUnsupported in cftuv_native.NATIVE_REFUSALS
    assert set(pin.EFFECTIVE_ALPHA_FILES) <= set(pin.PINS)


def test_a_stale_pin_refuses_by_name_before_it_computes_anything():
    pin = cftuv_native.pin
    context, source, boundary, frame = simple_call()
    pin._VERDICTS["effective_alpha"] = ("stale", ("reference/native_exact.py",))
    try:
        with pytest.raises(cftuv_native.NativePortStale, match="native_exact.py"):
            cftuv_native.effective_alpha_contacts(source, boundary, frame)
        assert cftuv_native.native_status()["effective_alpha"] == "stale(reference/native_exact.py)"
        assert frame.native is None, "nothing was converted"
        with backend.use_backend(*nec.NATIVE_ORDER) as ledger:  # the dispatcher names it and the oracle answers
            counts = boundary_module.ContactCountsV1()
            backend.effective_alpha_compute(boundary_module._native_contacts, context, source, boundary, frame, counts)
        assert ledger.record().effective_alpha_outcomes == ("NATIVE_PORT_STALE",) and (counts.rejected, counts.passed) != (0, 0)
    finally:
        pin.refresh()
    assert cftuv_native.native_status()["effective_alpha"] == "available"


def test_a_function_of_an_unpinned_file_that_moved_is_stale_by_name(monkeypatch):
    for file, name, _digest in effective_alpha_op.SHAPE_FUNCTIONS:
        moved = tuple((f, n, "0" * 64 if (f, n) == (file, name) else d) for f, n, d in effective_alpha_op.SHAPE_FUNCTIONS)
        monkeypatch.setattr(effective_alpha_op, "SHAPE_FUNCTIONS", moved)
        effective_alpha_op._KERNEL[:] = []
        context, source, boundary, frame = simple_call()
        with pytest.raises(cftuv_native.NativePortStale, match=re.escape(name)) as error:
            cftuv_native.effective_alpha_contacts(source, boundary, frame)
        assert file in str(error.value)
        monkeypatch.undo()
    effective_alpha_op._KERNEL[:] = []
    context, source, boundary, frame = simple_call()
    assert cftuv_native.effective_alpha_contacts(source, boundary, frame)[2], "the live kernel has the shape the port mirrors"


def test_the_shape_digests_in_the_shim_are_those_of_the_live_kernel():
    from cftuv_envelope.reference import metric

    live = effective_alpha_op.shape_digests({"reference/boundary.py": boundary_module, "reference/metric.py": metric})
    assert {(file, name): digest for file, name, digest in effective_alpha_op.SHAPE_FUNCTIONS} == live


def test_a_class_of_the_oracle_that_changed_shape_is_stale_by_name():
    from cftuv_envelope.reference import boundary_native, metric, native_exact

    broken_scalar = dataclasses.make_dataclass("ExactScalar", ["expression", "extra"], frozen=True, slots=True)
    planar = types.SimpleNamespace(ExactScalar=broken_scalar, ExactPlanarPoint=planar_types.ExactPlanarPoint)
    with pytest.raises(cftuv_native.NativePortStale, match="planar_types.py"):
        effective_alpha_op._bind(native_exact, planar, boundary_native, boundary_module, metric)
    with_post_init = dataclasses.make_dataclass("ExactScalar", ["expression"], frozen=True, slots=True, namespace={"__post_init__": lambda self: None})
    planar = types.SimpleNamespace(ExactScalar=with_post_init, ExactPlanarPoint=planar_types.ExactPlanarPoint)
    with pytest.raises(cftuv_native.NativePortStale, match="__post_init__"):
        effective_alpha_op._bind(native_exact, planar, boundary_native, boundary_module, metric)


# --------------------------------------------------------------------------
# the dispatcher
# --------------------------------------------------------------------------


def test_without_a_ledger_and_in_a_block_that_does_not_order_the_stage_the_oracle_is_called_with_the_same_objects():
    context, source, boundary, frame = simple_call()
    counts = boundary_module.ContactCountsV1()
    seen = []

    def oracle(*arguments):
        seen.append(arguments)
        return ("answer",)

    assert backend.effective_alpha_compute(oracle, context, source, boundary, frame, counts) == ("answer",)
    with backend.use_backend("NATIVE", "NATIVE", "NATIVE") as ledger:  # every stage but this one
        assert backend.effective_alpha_compute(oracle, context, source, boundary, frame, counts) == ("answer",)
        assert backend.active_effective_alpha_backend() is backend.KernelBackendV1.PYTHON
    assert all(item[0] is context and item[1] is source and item[2] is boundary and item[3] is frame and item[4] is counts for item in seen) and len(seen) == 2
    record = ledger.record()
    assert (record.effective_alpha_native_calls, record.effective_alpha_python_calls) == (0, 0) and record.effective_alpha_ran == ""


def test_a_pair_without_a_frame_has_no_prefilter_and_is_the_oracles_even_when_the_stage_is_ordered():
    context, source, boundary, _frame = simple_call()
    counts = boundary_module.ContactCountsV1()
    seen = []
    with backend.use_backend(*nec.NATIVE_ORDER) as ledger:
        backend.effective_alpha_compute(lambda *arguments: seen.append(arguments) or (), context, source, boundary, None, counts)
    assert len(seen) == 1 and ledger.record().effective_alpha_ran == ""


def test_an_ordered_stage_answers_the_pair_in_the_extension_and_applies_its_counts_once():
    call = simple_call()
    python, native = oracle_and_native(call)
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"] and python["symbolic"] == native["symbolic"]
    assert python["answer"][0] == "ok" and python["answer"][1], "the pair has contacts"
    assert python["counts"] == {"contact_candidates.prefilter_passed": 1, "contact_candidates.native": 1}
    record = native["record"]
    assert (record.effective_alpha_native_calls, record.effective_alpha_python_calls, record.effective_alpha_ran) == (1, 0, "native")
    assert record.effective_alpha_requested == "NATIVE" and not record.effective_alpha_fallbacks and not record.fallbacks
    # and a pair the prefilter proves empty
    far = simple_call(boundary_ends=((10, 10), (12, 14)))
    python, native = oracle_and_native(far)
    assert python["answer"] == ["ok", []] == native["answer"]
    assert python["counts"] == native["counts"] == {"contact_candidates.prefilter_rejected": 1, "contact_candidates.native": 1}
    assert native["record"].effective_alpha_native_calls == 1
    CHECKED["dispatch: answered"] += 2


@pytest.mark.parametrize(
    "why, ends",
    [
        ("an irrational end of the barrier", (("Sqrt(Rational(2, 1))", "Integer(0)"), ("Integer(3)", "Integer(2)"))),
        ("an end outside the native field", (("pi", "Integer(0)"), ("Integer(3)", "Integer(2)"))),
    ],
)
def test_a_pair_the_port_does_not_carry_is_declined_by_name_and_the_oracle_counts_it_alone(why, ends):
    context, source, _boundary, _frame = simple_call()
    boundary = synthetic.boundary_of(*ends, "text")
    python, native = oracle_and_native((context, source, boundary, None), with_frame=False)
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"] and python["symbolic"] == native["symbolic"]
    record = native["record"]
    assert (record.effective_alpha_native_calls, record.effective_alpha_python_calls) == (0, 0), "without a frame the dispatcher does not even ask"
    with_frame = oracle_and_native((context, source, boundary, None))
    assert with_frame[0]["answer"] == with_frame[1]["answer"] and with_frame[0]["counts"] == with_frame[1]["counts"] and with_frame[0]["symbolic"] == with_frame[1]["symbolic"]
    record = with_frame[1]["record"]
    assert (record.effective_alpha_native_calls, record.effective_alpha_python_calls) == (0, 1), why
    assert record.effective_alpha_outcomes == ("NATIVE_PORT_UNSUPPORTED",) and record.effective_alpha_ran == "python"
    CHECKED["dispatch: declined by name"] += 1


def test_a_pair_the_algebra_cannot_invert_is_declined_and_the_oracle_counts_its_refusal():
    """`da` in four square classes: the oracle's native path refuses (`OutsideNativeField`), counts it and goes to sympy; the extension never produces that refusal, it declines the pair."""

    context = synthetic.metric_of([[1, 0], [0, 1]])
    base = synthetic.source_of(context, (0, 0), (4, 0))
    normal = planar_types.ExactPlanarVector(
        planar_types.ExactScalar("Add(Integer(1), Pow(Integer(2), Rational(1, 2)), Pow(Integer(3), Rational(1, 2)))"), planar_types.ExactScalar("Sqrt(Rational(5, 1))"),
    )
    source = types.SimpleNamespace(start=base.start, end=base.end, tangent=base.tangent, owner_normal=normal, support_id="four classes")
    barrier = synthetic.boundary_of(synthetic.R(1, -1), synthetic.R(2, 3), "b")
    python, native = oracle_and_native((context, source, barrier, None))
    assert python["counts"].get("contact_candidates.outside_field") == 1, python["counts"]
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"] and python["symbolic"] == native["symbolic"]
    record = native["record"]
    assert (record.effective_alpha_native_calls, record.effective_alpha_python_calls) == (0, 1) and record.effective_alpha_outcomes == ("NATIVE_PORT_UNSUPPORTED",)
    CHECKED["dispatch: declined, the oracle refused"] += 1


def test_the_zero_vector_of_a_source_raises_the_oracles_value_error_with_its_text_on_both_sides():
    context = synthetic.metric_of([[1, 0], [0, 1]])
    start = synthetic.point(synthetic.rat(1), synthetic.rat(2))
    zero = types.SimpleNamespace(
        start=start, end=start, tangent=planar_types.ExactPlanarVector(planar_types.ExactScalar(synthetic.rat(1)), planar_types.ExactScalar(synthetic.rat(0))),
        owner_normal=planar_types.ExactPlanarVector(planar_types.ExactScalar(synthetic.rat(0)), planar_types.ExactScalar(synthetic.rat(1))), support_id="zero",
    )
    barrier = synthetic.boundary_of(synthetic.R(0, 0), synthetic.R(3, 3))
    python, native = oracle_and_native((context, zero, barrier, None))
    assert python["answer"][:2] == ["raised", "ValueError"] and "zero vector has no metric length" in python["answer"][2]
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"]


def test_a_refusal_of_the_port_after_the_dispatch_leaves_the_counts_to_the_oracle_alone(monkeypatch):
    context, source, boundary, _frame = simple_call()

    def refuses(*arguments):
        raise cftuv_native.NativePortUnsupported("a late named refusal")

    monkeypatch.setattr(cftuv_native, "effective_alpha_contacts", refuses)
    python, native = oracle_and_native((context, source, boundary, None))
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"] == {"contact_candidates.prefilter_passed": 1, "contact_candidates.native": 1}
    assert native["record"].effective_alpha_outcomes == ("NATIVE_PORT_UNSUPPORTED",)


def test_a_wheel_without_the_operation_falls_back_by_name(monkeypatch):
    context, source, boundary, _frame = simple_call()
    monkeypatch.delattr(cftuv_native, "effective_alpha_contacts")
    python, native = oracle_and_native((context, source, boundary, None))
    assert python["answer"] == native["answer"] and python["counts"] == native["counts"]
    assert native["record"].effective_alpha_outcomes == ("NATIVE_UNAVAILABLE",)


def test_the_converted_frame_lives_with_the_frame_and_is_made_once():
    context, source, boundary, frame = simple_call()
    assert frame.native is None
    first = cftuv_native.effective_alpha_contacts(source, boundary, frame)
    handle = frame.native
    assert handle is not None
    again = cftuv_native.effective_alpha_contacts(source, boundary, frame)
    assert frame.native is handle and nec.encode_contacts(first[2]) == nec.encode_contacts(again[2])
    other_source = synthetic.source_of(context, (0, 0), (0, 5))
    with pytest.raises(cftuv_native.NativePortUnsupported, match="another source"):
        cftuv_native.effective_alpha_contacts(other_source, boundary, frame)
    assert frame.native is handle


def test_the_objects_of_a_contact_are_the_ones_the_oracle_builds():
    context, source, boundary, frame = simple_call()
    rejected, passed, contacts = cftuv_native.effective_alpha_contacts(source, boundary, frame)
    oracle_contacts = nec.ORACLE(context, source, boundary, SourceContactFrame(context, source))
    assert (rejected, passed) == (0, 1) and len(contacts) == len(oracle_contacts) > 0
    for (alpha, station, point), (want_alpha, want_station, want_point) in zip(contacts, oracle_contacts):
        assert type(alpha) is type(want_alpha) and type(station) is type(want_station) and type(point) is type(want_point)
        assert type(point.x) is planar_types.ExactScalar and type(point.x.expression) is str
        for got, want in ((alpha, want_alpha), (station, want_station)):
            assert got.terms == want.terms and [(type(r), type(c)) for r, c in got.terms] == [(type(r), type(c)) for r, c in want.terms]
        assert point == want_point and hash(point) == hash(want_point)
        assert type(point.x.expression) is type(want_point.x.expression)


def test_the_slot_mode_of_the_process_decides_how_the_contacts_are_built_and_the_counters_say_which_path_ran():
    """`CFTUV_NATIVE_SLOTS`: `attr` builds every instance through the attribute protocol, `raw` (and `auto` where the probe confirms the layout) writes the slots; the answer is the same bytes."""

    mode = cftuv_native.slot_mode()
    context, source, boundary, frame = simple_call()
    cftuv_native.reset_slot_counters()
    contacts = cftuv_native.effective_alpha_contacts(source, boundary, frame)[2]
    counters = cftuv_native.slot_counters()
    assert contacts and counters["raw_builds"] + counters["attr_builds"] >= len(contacts) * 5, "per contact at least: alpha, station, two scalars and the point"
    if mode == "attr":
        assert counters["raw_builds"] == 0 and counters["attr_builds"] > 0
    elif mode == "raw":
        assert counters["attr_builds"] == 0 and counters["raw_builds"] > 0
    else:
        assert counters["raw_builds"] > 0, "the product's mode takes the raw path where the probe confirms the layout, and it does on CPython 3.11 and 3.13"
    assert nec.encode_contacts(contacts) == nec.encode_contacts(nec.ORACLE(context, source, boundary, SourceContactFrame(context, source)))
    CHECKED[f"slot mode {mode}: builds raw/attr"] += 1


def test_native_contacts_travel_with_the_preparation_as_the_oracles_do():
    """The memo (`ContactCandidatesMemoV1`) pickles ints and strings: the extension's contacts give the same wire, and a restored memo decodes to the same terms."""

    document = synthetic_document()
    objects = nec.Objects(document)
    memos = {"python": boundary_module.ContactCandidatesMemoV1(), "native": boundary_module.ContactCandidatesMemoV1()}
    taken = 0
    for pair in document["pairs"]:
        if pair["mode"] != "NATIVE_EXACT" or not pair["frame"] or pair["answer"][0] != "ok" or not pair["answer"][1]:
            continue
        for side, memo in memos.items():
            context, source, boundary, frame = objects.call(pair, side)
            with symbolic_backend.symbolic_backend(symbolic_backend.SymbolicBackendV1.NATIVE_EXACT):
                if side == "native":
                    with backend.use_backend(*nec.NATIVE_ORDER):
                        boundary_module._contacts_of(context, source, boundary, memo, frame)
                else:
                    boundary_module._contacts_of(context, source, boundary, memo, frame)
        taken += 1
        if taken == 120:
            break
    assert taken >= 60
    wires = {side: boundary_module._entries_wire(memo.entries) for side, memo in memos.items()}
    assert wires["python"] == wires["native"]
    restored = pickle.loads(pickle.dumps(memos["native"]))
    assert boundary_module._entries_wire(restored.entries) == wires["native"]
    for key, wanted in memos["python"].entries.items():
        assert nec.encode_contacts(restored.entries[key].decode()) == nec.encode_contacts(wanted)
    CHECKED["memo wire: contact entries"] += len(wires["native"])


# --------------------------------------------------------------------------
# the corpora
# --------------------------------------------------------------------------

_SYNTHETIC: list = []


def synthetic_document() -> dict:
    """The generated adversarial pairs, recorded once per process with the oracle's answers."""

    if not _SYNTHETIC:
        _SYNTHETIC.append(nec.synthetic_document(SYNTHETIC_SCALE))
    return _SYNTHETIC[0]


def test_synthetic_pairs_equal_the_oracle_and_every_kind_of_pair_is_there():
    document = synthetic_document()
    tally = compare_document(document, "synthetic")
    shape = nec.shape_of(document)
    assert tally["answered"] > 0 and tally["declined"] > 0 and tally["refused by the oracle"] > 0
    # the generator must reach every corner of the port, or a green run says little
    assert shape["raised"] >= 1 and shape["via_sympy"] > 0 and shape["rejected"] > 0 and shape["passed"] > 0 and shape["irrational_contacts"] > 0
    assert set(shape["modes"]) == {"NATIVE_EXACT", "SHADOW", "SYMPY"} and shape["with_frame"] < shape["pairs"]
    several = [pair for pair in document["pairs"] if pair["answer"][0] == "ok" and len(pair["answer"][1]) > 1]
    assert len(several) >= 20, "pairs with several contacts exercise the sort and the value-distinct filter"
    ties = [pair for pair in several if len({json.dumps(contact[0]) for contact in pair["answer"][1]}) < len(pair["answer"][1])]
    assert ties, "pairs with equal alpha (ties) must be there: the order of tied contacts is the order of insertion"


def test_what_the_extension_declines_is_named_and_nothing_else():
    """For every declined pair of the generated set: the reason the extension names is one of the reasons of the module note."""

    document = synthetic_document()
    objects = nec.Objects(document)
    reasons: Counter = Counter()
    for pair in document["pairs"]:
        if not pair["frame"] or pair["mode"] == "SYMPY":
            continue
        context, source, boundary, frame = objects.call(pair, "probe")
        try:
            cftuv_native.effective_alpha_contacts(source, boundary, frame)
        except cftuv_native.NativePortUnsupported as why:
            reasons[re.sub(r".*: ", "", str(why))] += 1
        except ValueError:
            reasons["the oracle's ValueError"] += 1
    allowed = (
        "a barrier point is not a rational text",
        "the source start is not a rational text",
        "a value outside the native field (too many square classes to invert)",
        "a sign the enclosure did not decide",
        "a contact with an irrational parameter (its point is written through sympy)",
        "the oracle's ValueError",
    )
    refused_frames = [name for name in reasons if name.startswith("the source frame refused") or "source frame refused" in name]
    assert set(reasons) - set(refused_frames) <= set(allowed), dict(reasons)
    assert reasons, "the generator provokes declines"
    for reason, count in sorted(reasons.items()):
        CHECKED[f"declined: {reason}"] += count


def test_the_kernel_suite_seed_equals_the_oracle():
    assert KERNEL_SUITE_SEED.is_file(), "the tracked seed of the kernel-suite pairs"
    document = nec.read_document(KERNEL_SUITE_SEED)
    assert len(document["pairs"]) >= 500
    categories = Counter(nec._category(pair) for pair in document["pairs"])
    assert categories["several"] >= 10 and categories["rejected"] >= 10 and categories["single"] >= 10 and categories["via_sympy"] >= 10, dict(categories)
    tally = compare_document(document, "kernel suite seed")
    assert tally["answered"] > 0


@needs_kernel_suite
def test_the_whole_kernel_suite_equals_the_oracle():
    document = nec.read_document(nec.corpus_directory() / "kernel_suite.recs.xz")
    assert len(document["pairs"]) > 1000
    compare_document(document, "kernel suite")


@needs_field
def test_the_field_corpus_equals_the_oracle():
    assert len(FIELD_FILES) >= 1
    for path in FIELD_FILES:
        document = nec.read_document(path)
        tally = compare_document(document, f"field {path.name.removesuffix('.recs.xz')}")
        shape = nec.shape_of(document)
        print(f"EFFECTIVE_ALPHA_FIELD_REPLAY mesh={path.name.removesuffix('.recs.xz')} pairs={shape['pairs']} answered={tally['answered']} declined={tally['declined']} rejected={shape['rejected']} passed={shape['passed']} mismatches=0")
        assert tally["declined"] == 0, "no pair of the field is outside the port"


@needs_field
def test_the_synthetic_corpus_on_disk_equals_the_oracle():
    path = nec.corpus_directory() / "synthetic.recs.xz"
    if not path.exists():
        pytest.skip("нет записанного синтетического корпуса (`python tools/native_effective_alpha_corpus.py synthetic`)")
    compare_document(nec.read_document(path), "synthetic corpus on disk")


def test_the_outcomes_do_not_depend_on_the_hash_seed_of_the_process():
    """`PYTHONHASHSEED` plays no part: the recorded answers, the oracle's now and the extension's are the same bytes in three processes with three seeds."""

    environment = {**os.environ, "PYTHONSAFEPATH": "1", "PYTHONPATH": os.pathsep.join([str(ROOT / "kernel" / "src"), str(ROOT / "tools")])}
    digests = {}
    for seed in ("0", "1", "4242"):
        done = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "native_effective_alpha_corpus.py"), "digest", "--cases", "0.1", "--only", "ties,irrational,roots"],
            env={**environment, "PYTHONHASHSEED": seed}, capture_output=True, text=True, check=False,
        )
        assert done.returncode == 0, done.stderr[-2000:]
        digests[seed] = json.loads(done.stdout.strip().splitlines()[-1])
    assert len({json.dumps(item, sort_keys=True) for item in digests.values()}) == 1, digests
    only = next(iter(digests.values()))
    assert only["recorded"] == only["python"] == only["native"] and only["pairs"] > 60
    CHECKED["hash seeds"] += len(digests)


def test_corpus_provenance_stays_with_its_source(tmp_path, monkeypatch):
    monkeypatch.setattr(nec.sys, "version", "3.11.11 field")
    nec.write_index(tmp_path, {"field": {"mesh": {"pairs": 1}}, "blender": "4.5.12", "scene": "scene.blend", "scene_sha256": "ab"})
    nec.write_index(tmp_path, {"field": {"other": {"pairs": 3}}, "blender": "4.5.12", "scene": "second.blend", "scene_sha256": "cd"})
    monkeypatch.setattr(nec.sys, "version", "3.13.1 synthetic")
    nec.write_index(tmp_path, {"synthetic": {"pairs": 2}})
    index = json.loads((tmp_path / "index.json").read_text(encoding="utf-8"))
    assert "python" not in index and "scene" not in index and "blender" not in index
    digest = nec.oracle_digest()
    assert index["provenance"]["field"] == {
        "mesh": {"python": "3.11.11", "oracle_digest": digest, "blender": "4.5.12", "scene": "scene.blend", "scene_sha256": "ab"},
        "other": {"python": "3.11.11", "oracle_digest": digest, "blender": "4.5.12", "scene": "second.blend", "scene_sha256": "cd"},
    }
    assert index["provenance"]["synthetic"] == {"python": "3.13.1", "oracle_digest": digest}
    assert set(index["field"]) == {"mesh", "other"} and index["synthetic"] == {"pairs": 2}


def test_the_seed_of_a_recorded_file_is_a_pure_cover_of_it():
    document = synthetic_document()
    wanted = Counter(nec._category(pair) for pair in document["pairs"])
    limit = len(document["pairs"]) // 2
    first, second = nec.seed_of(document, limit), nec.seed_of(document, limit)
    assert first == second and len(first["pairs"]) <= limit < len(document["pairs"])
    kept = Counter(nec._category(pair) for pair in first["pairs"])
    assert set(kept) == set(wanted), "every category of the file is in the seed"
    assert all(kept[name] >= min(count, limit // len(wanted) // 2) for name, count in wanted.items()), "each category has its share of the budget (or all it has)"
    assert [pair["id"] for pair in first["pairs"]] == [pair["id"] for pair in document["pairs"] if pair["id"] in {item["id"] for item in first["pairs"]}], "the order of the file"
    # every pair still points at a source and a metric that are in the file
    assert all(pair["source"] < len(first["sources"]) for pair in first["pairs"]) and all(source["metric"] < len(first["metrics"]) for source in first["sources"])
    merged = nec.merge_documents([document, document])
    assert len(merged["pairs"]) == len(document["pairs"]) and sum(pair["count"] for pair in merged["pairs"]) == 2 * sum(pair["count"] for pair in document["pairs"])
