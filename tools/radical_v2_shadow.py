"""SHADOW of `RadicalSumV1` by `RadicalSumV2` (slice S2 of the RadicalSumV2 plan): record every V1 operation with its operands and its result, then
check V2 against each one by value, and count every observable that would change when the consumers switch (R2).

    python tools/radical_v2_shadow.py field --out DIR [--corpus DIR]      the effective-alpha field corpus (`field/*.recs.xz`, python side) under the recorder, and
                                                                          a second replay whose float measures are taken from the V2 form (the prefilter counters)
    python tools/radical_v2_shadow.py kernel --out DIR [-- pytest args]   the kernel suite under the recorder (`-p radical_v2_shadow`; every xdist worker writes its file)
    python tools/radical_v2_shadow.py analyse FILE... [--json OUT]        (a) values, (c) signs, (d) cost per op, (e) observables that change
    python tools/radical_v2_shadow.py probe FILE... [--routes N]          (b) the V2 representation is a function of the value: random operation orders over recorded values

As a pytest plugin (`-p radical_v2_shadow`, environment `CFTUV_RADICAL_SHADOW=<dir>`) it records any pytest run. Records are `ops-<worker>.pkl.xz`: a pickled list of
`(kind, operand, argument, result, calls)` in ints and strings only (terms as flat `(r, num, den, ...)` tuples, the V1 representatives kept).
"""

from __future__ import annotations

import argparse
import contextlib
import json
import lzma
import os
import pickle
import random
import sys
import time
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from cftuv_envelope import float_filter  # noqa: E402
from cftuv_envelope.reference import native_exact  # noqa: E402
from cftuv_envelope.reference.native_exact import NativeExactError, RadicalSumV1  # noqa: E402
from cftuv_envelope.reference.radical_v2 import (  # noqa: E402
    RadicalRefusal,
    RadicalSumV2,
    _integer_form,
    from_v1,
    separation_bits,
    terms_wire,
)

RECORD_ENVIRONMENT = "CFTUV_RADICAL_SHADOW"
#: Distinct operations kept per kind and process (calls beyond it are counted, not kept).
KEEP_PER_KIND = 150_000


def flat(terms) -> tuple:
    out: list = []
    for radicand, coefficient in terms:
        out += (radicand, coefficient.numerator, coefficient.denominator)
    return tuple(out)


def unflat(wire) -> RadicalSumV1:
    return RadicalSumV1(tuple((wire[at], Fraction(wire[at + 1], wire[at + 2])) for at in range(0, len(wire), 3)))


def _fraction(value) -> tuple:
    return (value.numerator, value.denominator)


# --------------------------------------------------------------------------
# the recorder
# --------------------------------------------------------------------------


class OpRecorder:
    """Distinct V1 operations by `(kind, operand terms, argument)`, the result of the first call, and the number of calls."""

    def __init__(self) -> None:
        self.ops: dict = {}
        self.calls: dict = {}
        self.dropped: dict = {}
        self.kept: dict = {}
        self.active = True

    def note(self, kind: str, operand, argument, result) -> None:
        key = (kind, operand, argument)
        self.calls[kind] = self.calls.get(kind, 0) + 1
        entry = self.ops.get(key)
        if entry is not None:
            entry[1] += 1
            return
        if self.kept.get(kind, 0) >= KEEP_PER_KIND:
            self.dropped[kind] = self.dropped.get(kind, 0) + 1
            return
        self.kept[kind] = self.kept.get(kind, 0) + 1
        self.ops[key] = [result, 1]

    def rows(self) -> list:
        rows = []
        for (kind, operand, argument), (result, calls) in self.ops.items():
            rows.append((kind, _operand_wire(operand), _argument_wire(argument), result, calls))
        return rows

    def write(self, path: Path) -> int:
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {"rows": self.rows(), "calls": self.calls, "dropped": self.dropped}
        data = lzma.compress(pickle.dumps(document, protocol=4), preset=6)
        path.write_bytes(data)
        return len(data)


def _operand_wire(operand):
    if isinstance(operand, tuple) and operand and isinstance(operand[0], tuple):
        return ("T", flat(operand))
    if operand == ():
        return ("T", ())
    if isinstance(operand, Fraction):
        return ("Q", _fraction(operand))
    return operand


def _argument_wire(argument):
    if isinstance(argument, tuple) and argument and isinstance(argument[0], tuple):
        return ("T", flat(argument))
    if argument == ():
        return ("T", ())
    if isinstance(argument, Fraction):
        return ("Q", _fraction(argument))
    return argument


def _result_of(value):
    if type(value) is RadicalSumV1:
        return ("T", flat(value.terms))
    if isinstance(value, Fraction):
        return ("Q", _fraction(value))
    if value is None:
        return ("N",)
    if isinstance(value, tuple) and len(value) == 2 and all(isinstance(item, Fraction) for item in value):
        return ("E", _fraction(value[0]), _fraction(value[1]))
    return ("V", value)


def _raised(error: BaseException):
    return ("R", getattr(error, "code", "") or type(error).__name__)


@contextlib.contextmanager
def installed(recorder: OpRecorder):
    """Wraps the V1 methods and the module functions that read V1 values; every call is computed by the original and noted once."""

    from cftuv_envelope.reference import boundary, metric, planar_types

    cls = RadicalSumV1
    originals = {name: cls.__dict__[name] for name in ("__add__", "__radd__", "__sub__", "__neg__", "scaled", "__mul__", "__rmul__", "inverse", "signum", "enclosure", "as_rational")}
    original_sqrt = cls.__dict__["sqrt_of_rational"]
    module_originals = {
        (native_exact, "canonical_text"): native_exact.canonical_text,
        (planar_types, "_canonical_text"): planar_types._canonical_text,
        (native_exact, "terms_wire"): native_exact.terms_wire,
        (boundary, "terms_wire"): boundary.terms_wire,
        (native_exact, "rational_ratio"): native_exact.rational_ratio,
        (metric, "_product"): metric._product,
        (float_filter, "_measure"): float_filter._measure,
    }

    def note(kind, operand, argument, compute, shape=_result_of):
        if not recorder.active:
            return compute()
        try:
            result = compute()
        except (NativeExactError, ZeroDivisionError, native_exact.ExactScalarTextCanonUnsupported) as error:
            recorder.note(kind, operand, argument, _raised(error))
            raise
        recorder.note(kind, operand, argument, shape(result))
        return result

    def coerce(other):
        try:
            return native_exact._coerce(other)
        except Exception:  # noqa: BLE001 - the original raises the same on its own
            return None

    def binary(kind, original):
        def wrapped(self, other):
            if isinstance(other, (int, Fraction)) and kind == "mul":
                return original(self, other)  # `scaled`, noted there
            value = coerce(other)
            if value is None:
                return original(self, other)
            return note(kind, self.terms, value.terms, lambda: original(self, other))

        return wrapped

    def unary(kind, original):
        def wrapped(self):
            return note(kind, self.terms, None, lambda: original(self))

        return wrapped

    def scaled(self, factor):
        return note("scaled", self.terms, Fraction(factor), lambda: originals["scaled"](self, factor))

    def enclosure(self, bits):
        return note("enclosure", self.terms, int(bits), lambda: originals["enclosure"](self, bits))

    def sqrt_of_rational(value):
        return note("sqrt_of_rational", (), Fraction(value), lambda: original_sqrt.__func__(value))

    def canonical_text(value):
        if type(value) is not RadicalSumV1:
            return module_originals[(native_exact, "canonical_text")](value)
        return note("canonical_text", value.terms, None, lambda: module_originals[(native_exact, "canonical_text")](value))

    def wire(value):
        return note("terms_wire", value.terms, None, lambda: module_originals[(native_exact, "terms_wire")](value))

    def rational_ratio(numerator, denominator):
        top, bottom = coerce(numerator), coerce(denominator)
        original = module_originals[(native_exact, "rational_ratio")]
        if top is None or bottom is None:
            return original(numerator, denominator)
        return note("rational_ratio", top.terms, bottom.terms, lambda: original(numerator, denominator))

    def product(left, right):
        return note("mul", left.terms, right.terms, lambda: module_originals[(metric, "_product")](left, right))

    def measure(value):
        original = module_originals[(float_filter, "_measure")]
        if type(value) is not RadicalSumV1:
            return original(value)
        return note("measure", value.terms, None, lambda: original(value), lambda entry: ("F", entry[1], entry[2]))

    replacements = {
        "__add__": binary("add", originals["__add__"]), "__radd__": binary("add", originals["__radd__"]),
        "__sub__": binary("sub", originals["__sub__"]), "__neg__": unary("neg", originals["__neg__"]),
        "scaled": scaled, "__mul__": binary("mul", originals["__mul__"]), "__rmul__": binary("mul", originals["__rmul__"]),
        "inverse": unary("inverse", originals["inverse"]), "signum": unary("signum", originals["signum"]),
        "enclosure": enclosure, "as_rational": unary("as_rational", originals["as_rational"]),
    }
    for name, function in replacements.items():
        setattr(cls, name, function)
    cls.sqrt_of_rational = staticmethod(sqrt_of_rational)
    native_exact.canonical_text = canonical_text
    planar_types._canonical_text = canonical_text
    native_exact.terms_wire = wire
    boundary.terms_wire = wire
    native_exact.rational_ratio = rational_ratio
    from cftuv_envelope.wavefront import conveyor

    conveyor_ratio = conveyor.rational_ratio
    conveyor.rational_ratio = rational_ratio
    metric._product = product
    float_filter._measure = measure
    try:
        yield recorder
    finally:
        for name, function in originals.items():
            setattr(cls, name, function)
        cls.sqrt_of_rational = original_sqrt
        for (module, name), function in module_originals.items():
            setattr(module, name, function)
        conveyor.rational_ratio = conveyor_ratio


# --------------------------------------------------------------------------
# pytest plugin
# --------------------------------------------------------------------------

_ACTIVE: list = []


def pytest_configure(config):
    target = os.environ.get(RECORD_ENVIRONMENT)
    if target and not _ACTIVE:
        recorder = OpRecorder()
        manager = installed(recorder)
        manager.__enter__()
        _ACTIVE.append((recorder, manager, Path(target)))


def pytest_runtest_setup(item):
    if _ACTIVE:
        # a test that patches the algebra is not a statement about it
        _ACTIVE[0][0].active = "monkeypatch" not in getattr(item, "fixturenames", ())


def pytest_unconfigure(config):
    if _ACTIVE:
        recorder, manager, target = _ACTIVE.pop()
        manager.__exit__(None, None, None)
        if recorder.ops:
            worker = os.environ.get("PYTEST_XDIST_WORKER", "main")
            size = recorder.write(target / f"ops-kernel-{worker}.pkl.xz")
            print(f"\nRADICAL_V2_SHADOW_RECORDED worker={worker} distinct={len(recorder.ops)} calls={sum(recorder.calls.values())} bytes={size}")


# --------------------------------------------------------------------------
# the field: the effective-alpha pair corpus
# --------------------------------------------------------------------------


@contextlib.contextmanager
def v2_float_measures():
    """`float_filter._measure` of a V1 value computed on its V2 terms (`pairs()`): the prefilter as it would read V2 values."""

    original = float_filter._measure

    def measure(value):
        if type(value) is not RadicalSumV1:
            return original(value)
        proxy = SimpleNamespace(terms=from_v1(value)[0].pairs())
        entry = original(proxy)
        float_filter._TABLE.pop(id(proxy), None)
        entry = (value, entry[1], entry[2])
        float_filter._TABLE[id(value)] = entry
        return entry

    float_filter._measure = measure
    float_filter.clear_table()
    try:
        yield
    finally:
        float_filter._measure = original
        float_filter.clear_table()


def shadow_pairs(document: dict) -> tuple[OpRecorder, dict]:
    """Every pair of an effective-alpha corpus document on the python side under the recorder, then again with the V2 float measures.

    Returns the recorder and the report: pairs whose replay differs from the record (must be 0), and the pairs whose counters or answer
    move when the prefilter reads the V2 form (what R2 would change).
    """

    import native_effective_alpha_corpus as alpha

    recorder = OpRecorder()
    objects = alpha.Objects(document)
    started = time.perf_counter()
    float_filter.clear_table()
    mismatched = 0
    baseline = []
    with installed(recorder):
        for pair in document["pairs"]:
            outcome = alpha.run_pair(objects, pair, "python")
            baseline.append(outcome)
            mismatched += not alpha.equal_outcomes(outcome, alpha.recorded_outcome(pair))
    recorded_seconds = time.perf_counter() - started
    objects = alpha.Objects(document)
    counts_changed = answers_changed = 0
    moved: dict = {}
    with v2_float_measures():
        for pair, before in zip(document["pairs"], baseline):
            after = alpha.run_pair(objects, pair, "python")
            if after["counts"] != before["counts"]:
                counts_changed += 1
                for key in set(after["counts"]) | set(before["counts"]):
                    delta = after["counts"].get(key, 0) - before["counts"].get(key, 0)
                    if delta:
                        moved[key] = moved.get(key, 0) + delta
            answers_changed += after["answer"] != before["answer"]
    report = {
        "pairs": len(document["pairs"]), "calls": sum(pair["count"] for pair in document["pairs"]), "replay_vs_recorded_mismatch": mismatched,
        "distinct_ops": len(recorder.ops), "op_calls": recorder.calls, "dropped": recorder.dropped, "seconds": round(recorded_seconds, 1),
        "v2_floats": {"pairs_with_other_counts": counts_changed, "pairs_with_other_answer": answers_changed, "counter_moves": moved},
    }
    return recorder, report


def command_field(out: Path, corpus: Path) -> int:
    import native_effective_alpha_corpus as alpha

    report = {}
    for path in sorted((corpus / "field").glob("*.recs.xz")):
        mesh = path.name.split(".recs")[0]
        recorder, report[mesh] = shadow_pairs(alpha.read_document(path))
        report[mesh]["bytes"] = recorder.write(out / f"ops-field-{mesh}.pkl.xz")
        print(f"RADICAL_V2_SHADOW_FIELD {mesh} {json.dumps(report[mesh])}", flush=True)
    (out / "field_report.json").write_text(json.dumps(report, indent=1, sort_keys=True), encoding="utf-8")
    return 0


def loaded_of(recorder: OpRecorder) -> dict:
    """The recorder's operations in the form `load` returns (no file in between)."""

    ops = {(kind, operand, argument): [result, calls] for kind, operand, argument, result, calls in recorder.rows()}
    return {"ops": ops, "calls": dict(recorder.calls), "dropped": dict(recorder.dropped)}


def command_kernel(out: Path, extra) -> int:
    import subprocess

    environment = dict(os.environ)
    environment[RECORD_ENVIRONMENT] = str(out)
    environment["PYTHONPATH"] = os.pathsep.join([str(ROOT / "kernel" / "src"), str(ROOT / "tools"), str(ROOT / "kernel" / "tests")])
    command = [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "-p", "radical_v2_shadow", "--rootdir", str(ROOT / "kernel"), str(ROOT / "kernel" / "tests"), *extra]
    return subprocess.call(command, env=environment, cwd=str(ROOT))


# --------------------------------------------------------------------------
# the analysis
# --------------------------------------------------------------------------


def load(paths) -> dict:
    """`(kind, operand, argument) -> [result, calls]` over all files (the first file's result wins; results of one key are equal by construction)."""

    merged: dict = {}
    calls: dict = {}
    dropped: dict = {}
    files = []
    for path in map(Path, paths):
        files += sorted(path.glob("ops-*.pkl.xz")) if path.is_dir() else [path]
    for path in files:
        document = pickle.loads(lzma.decompress(Path(path).read_bytes()))
        for kind, operand, argument, result, count in document["rows"]:
            key = (kind, operand, argument)
            entry = merged.get(key)
            if entry is None:
                merged[key] = [result, count]
            else:
                entry[1] += count
        for name, value in document["calls"].items():
            calls[name] = calls.get(name, 0) + value
        for name, value in document["dropped"].items():
            dropped[name] = dropped.get(name, 0) + value
    return {"ops": merged, "calls": calls, "dropped": dropped}


def _v1_of(wire) -> RadicalSumV1:
    return unflat(wire[1]) if wire[0] == "T" else RadicalSumV1.rational(Fraction(*wire[1]))


def _v2(value: RadicalSumV1) -> RadicalSumV2:
    return from_v1(value)[0]


def _v2_of_result(result):
    return _v2(unflat(result[1])) if result[0] == "T" else None


def _v1_ladder_rounds(terms) -> int | None:
    """Rungs V1's `signum` evaluates for >= 3 terms (`None`: its ladder refuses)."""

    _, items = native_exact._integer_form(terms)
    for rounds, bits in enumerate(native_exact._SIGN_BITS, 1):
        low, high = native_exact._integer_enclosure(items, bits)
        if low > 0 or high < 0:
            return rounds
    return None


def _v2_answer(kind, operand: RadicalSumV1, argument):
    """`(V2 answer, work)` of one recorded operation; refusals as `("R", code)`."""

    x = _v2(operand) if operand is not None else None
    try:
        if kind == "add":
            return x.add(_v2(argument))
        if kind == "sub":
            return x.sub(_v2(argument))
        if kind == "mul":
            return x.mul(_v2(argument))
        if kind == "neg":
            return x.neg(), None
        if kind == "scaled":
            return x.scaled(argument), None
        if kind == "inverse":
            return x.inverse()
        if kind == "signum":
            return x.signum()
        if kind == "enclosure":
            return x.enclosure(argument)
        if kind == "as_rational":
            return x.as_rational(), None
        if kind == "sqrt_of_rational":
            return RadicalSumV2.sqrt_of_rational(argument)
        if kind == "rational_ratio":
            bottom = _v2(argument)
            if bottom.is_zero:
                return None, None
            quotient, work = x.div(bottom)
            return quotient.as_rational(), work
        if kind == "canonical_text":
            return x.text(), None
        if kind == "terms_wire":
            return terms_wire(x), None
        if kind == "measure":
            proxy = SimpleNamespace(terms=x.pairs())
            entry = float_filter._measure(proxy)
            float_filter._TABLE.pop(id(proxy), None)
            return ("F", entry[1], entry[2]), None
    except (RadicalRefusal, ZeroDivisionError) as error:
        return ("R", getattr(error, "code", "") or type(error).__name__), getattr(error, "work", None)
    raise KeyError(kind)


def _decode(kind, operand_wire, argument_wire):
    operand = None if kind == "sqrt_of_rational" else _v1_of(operand_wire)
    if kind in ("add", "sub", "mul", "rational_ratio"):
        argument = _v1_of(argument_wire)
    elif kind == "scaled" or kind == "sqrt_of_rational":
        argument = Fraction(*argument_wire[1])
    else:
        argument = argument_wire
    return operand, argument


def analyse(loaded: dict) -> dict:
    """(a) every op's V2 value against V1's, (c) signs and ladders, (e) the observables that change; counts by kind."""

    by_kind: dict = {}
    report = {"calls": loaded["calls"], "dropped": loaded["dropped"], "kinds": by_kind, "examples": []}
    for (kind, operand_wire, argument_wire), (result, calls) in loaded["ops"].items():
        row = by_kind.setdefault(kind, {"distinct": 0, "calls": 0, "value_equal": 0, "value_differs": 0, "refusal_equal": 0, "refusal_differs": 0})
        row["distinct"] += 1
        row["calls"] += calls
        operand, argument = _decode(kind, operand_wire, argument_wire)
        answer, work = _v2_answer(kind, operand, argument)
        same = _same_answer(kind, result, answer, operand, row, calls)
        if same:
            row["value_equal"] += 1
        else:
            row["value_differs"] += 1
            if len(report["examples"]) < 20:
                report["examples"].append({"kind": kind, "operand": operand_wire, "argument": str(argument_wire), "v1": str(result), "v2": str(answer)})
        _observe(kind, operand, argument, result, answer, work, row, calls)
    return report


def _same_answer(kind, result, answer, operand, row, calls) -> bool:
    if result[0] == "R":
        if isinstance(answer, tuple) and answer and answer[0] == "R":
            row["refusal_equal"] += answer[1] == result[1]
            row.setdefault("refusal_codes", {})
            key = f"{result[1]}->{answer[1]}"
            row["refusal_codes"][key] = row["refusal_codes"].get(key, 0) + 1
            return True  # a refusal on both sides; the codes are counted separately
        row["refusal_differs"] += 1
        row.setdefault("v1_refused_v2_answered", {})
        row["v1_refused_v2_answered"][result[1]] = row["v1_refused_v2_answered"].get(result[1], 0) + 1
        return kind == "canonical_text" or kind == "signum"  # V2 answers where V1 refuses: an observable change, not a value difference
    if isinstance(answer, tuple) and answer and answer[0] == "R":
        row["refusal_differs"] += 1
        return False
    if result[0] == "T":
        return answer == _v2_of_result(result)
    if result[0] == "Q":
        return answer == Fraction(*result[1])
    if result[0] == "N":
        return answer is None
    if result[0] == "E":
        low, high = Fraction(*result[1]), Fraction(*result[2])
        (v2_low, v2_high) = answer
        return v2_low <= high and low <= v2_high  # both enclose the same value: they must overlap
    if result[0] == "F":
        return True  # float measures are compared in `_observe`
    if result[0] == "V":
        if kind == "canonical_text" or kind == "terms_wire":
            return True  # compared in `_observe`
        return answer == result[1]
    return False


def _observe(kind, operand, argument, result, answer, work, row, calls) -> None:
    def bump(name, amount=1):
        row[name] = row.get(name, 0) + amount

    if work is not None:
        bump("work_class_merge_checks", work.class_merge_checks * calls)
        bump("work_conjugation_products", work.conjugation_products * calls)
        bump("work_enclosure_rounds", work.enclosure_rounds * calls)
    if operand is not None and kind != "sqrt_of_rational":
        if _v2(operand).pairs() != operand.terms:
            bump("operand_representation_differs")
    if kind == "canonical_text":
        if result[0] == "V":
            bump("text_equal" if result[1] == answer else "text_differs")
        else:
            bump("text_new_multi_term", calls)
    elif kind == "terms_wire":
        bump("wire_equal" if tuple(result[1]) == answer else "wire_differs")
    elif kind == "enclosure" and result[0] == "E":
        low, high = Fraction(*result[1]), Fraction(*result[2])
        if (low, high) != answer:
            bump(f"bounds_differ_at_{argument}")
            bump(f"bounds_differ_at_{argument}_calls", calls)
        else:
            bump(f"bounds_equal_at_{argument}")
    elif kind == "measure" and result[0] == "F":
        if (result[1], result[2]) != (answer[1], answer[2]):
            bump("measure_differs")
            bump("measure_differs_calls", calls)
            if (result[1] is None) != (answer[1] is None):
                bump("measure_takes_differs")
        else:
            bump("measure_equal")
    elif kind == "signum" and operand is not None:
        count = len(operand.terms)
        bump(f"terms_{min(count, 4)}{'+' if count >= 4 else ''}")
        if count >= 3:
            v1_rounds = _v1_ladder_rounds(operand.terms)
            v2_rounds = work.enclosure_rounds if work is not None else 0
            _, items = _integer_form(_v2(operand).terms)
            required = separation_bits(items)
            bump("ladder_v1_rounds", v1_rounds or 0)
            bump("ladder_v2_rounds", v2_rounds)
            bump("v1_needed_more_than_64_bits", v1_rounds is not None and v1_rounds > 1)
            bump("v2_decided_at_the_bound", v2_rounds >= 2 and (64 << (v2_rounds - 1)) >= required)
            bump("v2_bound_below_64_bits", required < 64)
            bump("v2_cheaper_rounds", v2_rounds < (v1_rounds or 99))
            row["max_separation_bits"] = max(row.get("max_separation_bits", 0), required)
    elif kind == "inverse" and operand is not None:
        bump(f"inverse_terms_{len(operand.terms)}")


# --------------------------------------------------------------------------
# (d) cost per op, V1 vs V2 (Python)
# --------------------------------------------------------------------------


def cost(loaded: dict, repeats: int = 3) -> dict:
    """Mean microseconds per distinct operation of each kind, V1 and V2 on the same values (operands decoded and converted beforehand)."""

    groups: dict = {}
    for (kind, operand_wire, argument_wire), _ in loaded["ops"].items():
        if kind in ("measure", "terms_wire", "rational_ratio"):
            continue
        operand, argument = _decode(kind, operand_wire, argument_wire)
        groups.setdefault(kind, []).append((operand, argument))
        if kind == "signum" and len(operand.terms) >= 3:
            groups.setdefault("signum_3plus", []).append((operand, argument))
    table = {}
    for kind, rows in sorted(groups.items()):
        v1_rows = rows
        v2_rows = [(_v2(operand) if operand is not None else None, _v2(argument) if isinstance(argument, RadicalSumV1) else argument) for operand, argument in rows]
        v1 = _time(lambda: [_v1_call(kind.split("_3plus")[0], operand, argument) for operand, argument in v1_rows], repeats)
        v2 = _time(lambda: [_v2_call(kind.split("_3plus")[0], operand, argument) for operand, argument in v2_rows], repeats)
        table[kind] = {"ops": len(rows), "v1_us": round(v1 * 1e6 / len(rows), 3), "v2_us": round(v2 * 1e6 / len(rows), 3), "v2_over_v1": round(v2 / v1, 3) if v1 else None}
    return table


def _time(function, repeats: int) -> float:
    best = None
    for _ in range(repeats):
        started = time.perf_counter()
        function()
        elapsed = time.perf_counter() - started
        best = elapsed if best is None else min(best, elapsed)
    return best


def _v1_call(kind, x, y):
    try:
        if kind == "add":
            return x + y
        if kind == "sub":
            return x - y
        if kind == "mul":
            return x * y
        if kind == "neg":
            return -x
        if kind == "scaled":
            return x.scaled(y)
        if kind == "inverse":
            return x.inverse()
        if kind == "signum":
            return x.signum()
        if kind == "enclosure":
            return x.enclosure(y)
        if kind == "as_rational":
            return x.as_rational()
        if kind == "sqrt_of_rational":
            return RadicalSumV1.sqrt_of_rational(y)
        if kind == "canonical_text":
            return native_exact.canonical_text(x)
    except (NativeExactError, ZeroDivisionError, native_exact.ExactScalarTextCanonUnsupported):
        return None
    return None


def _v2_call(kind, x, y):
    try:
        if kind == "add":
            return x.add(y)
        if kind == "sub":
            return x.sub(y)
        if kind == "mul":
            return x.mul(y)
        if kind == "neg":
            return x.neg()
        if kind == "scaled":
            return x.scaled(y)
        if kind == "inverse":
            return x.inverse()
        if kind == "signum":
            return x.signum()
        if kind == "enclosure":
            return x.enclosure(y)
        if kind == "as_rational":
            return x.as_rational()
        if kind == "sqrt_of_rational":
            return RadicalSumV2.sqrt_of_rational(y)
        if kind == "canonical_text":
            return x.text()
    except (RadicalRefusal, ZeroDivisionError):
        return None
    return None


# --------------------------------------------------------------------------
# (b) the representation is a function of the value
# --------------------------------------------------------------------------


def _hidden(terms, rng) -> list:
    """`(r, c)` pairs of the same value with squares hidden in the radicands and classes split in parts."""

    pairs = []
    for radicand, coefficient in terms:
        parts = rng.randint(1, 3)
        remaining = coefficient
        for index in range(parts):
            share = remaining if index == parts - 1 else remaining * Fraction(rng.randint(1, 5), rng.randint(2, 7))
            remaining -= share
            square = rng.choice((1, 1, 2, 3, 5, 6, 7))
            pairs.append((radicand * square * square, share / square))
    rng.shuffle(pairs)
    return pairs


def _routes(value: RadicalSumV1, rng, count: int):
    """`count` V1 values and the V2 values built by different operation orders from the same value."""

    helper = RadicalSumV1(tuple(sorted(((r, Fraction(rng.randint(-9, 9) or 1, rng.randint(1, 9))) for r, _ in value.terms[:2]))))
    for route in range(count):
        pairs = _hidden(value.terms, rng)
        kind = route % 4
        if kind == 0:  # sum of the pairs, in a shuffled order
            v1 = unsorted_sum(pairs)
            v2 = RadicalSumV2.from_terms(pairs)[0]
        elif kind == 1:  # (x + h) - h
            v1 = (unsorted_sum(pairs) + helper) - helper
            v2 = RadicalSumV2.from_terms(pairs)[0].add(_v2(helper))[0].sub(_v2(helper))[0]
        elif kind == 2:  # (x * f) * (1/f)
            factor = Fraction(rng.randint(1, 12), rng.randint(1, 12))
            v1 = unsorted_sum(pairs).scaled(factor).scaled(1 / factor)
            v2 = RadicalSumV2.from_terms(pairs)[0].scaled(factor).scaled(1 / factor)
        else:  # (x * h) / h, h with at most two classes
            v1 = (unsorted_sum(pairs) * helper) / helper if helper.terms else unsorted_sum(pairs)
            v2 = RadicalSumV2.from_terms(pairs)[0]
            if helper.terms:
                v2 = v2.mul(_v2(helper))[0].div(_v2(helper))[0]
        yield v1, v2


def unsorted_sum(pairs) -> RadicalSumV1:
    """V1's sum of the parts as V1 reads them from a text: `sqrt_of_rational(r).scaled(c)` (the representative is `r`, squares hidden in it)."""

    total = RadicalSumV1.zero()
    for radicand, coefficient in pairs:
        total = total + RadicalSumV1.sqrt_of_rational(radicand).scaled(coefficient)
    return total


def _identity_v1(value: RadicalSumV1) -> tuple:
    proxy = SimpleNamespace(terms=value.terms)
    entry = float_filter._measure(proxy)
    float_filter._TABLE.pop(id(proxy), None)
    return (value.terms, value.enclosure(128), entry[1], entry[2])


def _identity_v2(value: RadicalSumV2) -> tuple:
    proxy = SimpleNamespace(terms=value.pairs())
    entry = float_filter._measure(proxy)
    float_filter._TABLE.pop(id(proxy), None)
    return (value.terms, value.enclosure(128)[0], entry[1], entry[2], value.text())


def probe(loaded: dict, routes: int, limit: int, seed: int = 20261012) -> dict:
    """For distinct recorded irrational values: `routes` operation orders each; any second form of one value is a representation difference."""

    rng = random.Random(seed)
    values = []
    seen = set()
    for (kind, operand_wire, _), (result, _) in loaded["ops"].items():
        for wire in (operand_wire, result):
            if wire and wire[0] == "T" and len(wire[1]) >= 3 and any(wire[1][at] != 1 for at in range(0, len(wire[1]), 3)):
                value = _v2(unflat(wire[1]))
                if value not in seen:
                    seen.add(value)
                    values.append(value)
    rng.shuffle(values)
    values = values[:limit]
    tally = {"values": len(values), "routes": 0, "v1_value_differs": 0, "v2_value_differs": 0, "v2_form_differs": 0, "v1_form_differs": 0, "v1_values_with_several_forms": 0, "v2_values_with_several_forms": 0}
    for value in values:
        target = RadicalSumV1(value.pairs())
        forms_v1, forms_v2 = set(), set()
        for v1, v2 in _routes(target, rng, routes):
            tally["routes"] += 1
            if v2 != value:
                tally["v2_value_differs"] += 1
            if v1 != target:
                tally["v1_value_differs"] += 1
            forms_v2.add(_identity_v2(v2))
            forms_v1.add(_identity_v1(v1))
        tally["v2_values_with_several_forms"] += len(forms_v2) > 1
        tally["v1_values_with_several_forms"] += len(forms_v1) > 1
        tally["v2_form_differs"] += len(forms_v2) - 1
        tally["v1_form_differs"] += len(forms_v1) - 1
    return tally


# --------------------------------------------------------------------------
# command line
# --------------------------------------------------------------------------


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("field", "kernel", "analyse", "probe"))
    parser.add_argument("files", nargs="*")
    parser.add_argument("--out", default="")
    parser.add_argument("--corpus", default="E:/cftuv_native_corpus/effective_alpha")
    parser.add_argument("--json", default="")
    parser.add_argument("--routes", type=int, default=8)
    parser.add_argument("--limit", type=int, default=3000)
    parser.add_argument("--no-cost", action="store_true")
    arguments, extra = parser.parse_known_args(argv)
    if arguments.command == "field":
        return command_field(Path(arguments.out), Path(arguments.corpus))
    if arguments.command == "kernel":
        return command_kernel(Path(arguments.out), [*arguments.files, *extra])
    loaded = load(arguments.files)
    if arguments.command == "probe":
        result = probe(loaded, arguments.routes, arguments.limit)
    else:
        result = analyse(loaded)
        if not arguments.no_cost:
            result["cost"] = cost(loaded)
    text = json.dumps(result, indent=1, sort_keys=True, default=str)
    if arguments.json:
        Path(arguments.json).write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
