"""Корпус raw-turn границы Huber Density до bind: аргументы, лучи, исходы, тёплая память.

python tools/native_density_turn_corpus.py synthetic --out DIR
python tools/native_density_turn_corpus.py kernel-suite --out DIR -- kernel/tests/test_plan_compile_fast_paths.py -q
python tools/native_density_turn_corpus.py replay --file DIR/synthetic.turns.xz

Сохраняет все девять полей _TurnAtom до/после, порядок таблиц памяти и отказ
после частично выполненной ступени. Диагностика hit/H0 хранится отдельно от
наблюдаемого ответа. JSON/xz, frozen bind tools переиспользуются без изменения.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import lzma
import os
import sys
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import sympy as sp  # noqa: E402

from cftuv_envelope.reference import angular, density_turn as turn, planar_types, symbolic_backend as symbolic  # noqa: E402
from native_density_corpus import Observation, applied_settings, settings  # noqa: E402
from native_density_wire import CorpusWireError, ExpressionVector, Wire, json_key, require_keys  # noqa: E402

SCHEMA = "cftuv.native-density-turn-corpus.v1"
ORACLE = turn._huber_density_interpolated_normals
TURN_MEMO_FIELDS = ("dot_expressions", "unit_vectors", "turn_atoms", "fans")
ATOM_FIELDS = tuple(turn._TurnAtom.__slots__)
INPUT_KEYS = ("metric", "incoming", "outgoing", "count", "orientation", "canonical", "rotation", "settings")
OUTCOME_KEYS = ("answer", "budgets", "counts", "symbolic", "disagreements", "memo_after")


class TurnWire(Wire):
    """Отдельное расширение frozen bind-проволоки; ни одно поле _TurnAtom не выводится заново."""

    def value(self, value):
        if isinstance(value, turn._TurnAtom):
            return ["turn_atom", {name: self.value(getattr(value, name)) for name in ATOM_FIELDS}]
        return super().value(value)

    def unvalue(self, value):
        if type(value) is list and value and value[0] == "turn_atom":
            if len(value) != 2:
                raise CorpusWireError("turn_atom requires a field map")
            require_keys(value[1], ATOM_FIELDS, "turn_atom")
            fields = {name: self.unvalue(value[1][name]) for name in ATOM_FIELDS}
            atom = turn._TurnAtom(*[fields[name] for name in ATOM_FIELDS[:5]])
            for name in ATOM_FIELDS[5:]:
                setattr(atom, name, fields[name])
            return atom
        return super().unvalue(value)

    def metric(self, metric):
        return {"base": super().metric(metric),
                "turn": {name: self.value(getattr(metric._density_exact_memo, name)) for name in TURN_MEMO_FIELDS}}

    def unmetric(self, body):
        require_keys(body, ("base", "turn"), "turn.metric")
        require_keys(body["turn"], TURN_MEMO_FIELDS, "turn.metric.turn")
        metric = super().unmetric(body["base"])
        for name in TURN_MEMO_FIELDS:
            value = self.unvalue(body["turn"][name])
            if type(value) is not dict:
                raise CorpusWireError(f"turn.metric.{name} must be a map")
            setattr(metric._density_exact_memo, name, value)
        return metric


def input_body(wire, metric, incoming, outgoing, count, orientation_sign, canonical=None, rotation=None):
    return {"metric": wire.metric(metric), "incoming": wire.value(incoming), "outgoing": wire.value(outgoing),
            "count": wire.value(count), "orientation": wire.value(orientation_sign),
            "canonical": wire.value(canonical), "rotation": wire.value(rotation), "settings": settings()}


def decode_call(wire, body):
    require_keys(body, INPUT_KEYS, "turn.input")
    args = (wire.unmetric(body["metric"]), wire.unvalue(body["incoming"]), wire.unvalue(body["outgoing"]),
            wire.unvalue(body["count"]), wire.unvalue(body["orientation"]))
    kwargs = {"canonical_excess_over_pi": wire.unvalue(body["canonical"]), "rational_rotation": wire.unvalue(body["rotation"])}
    return args, kwargs


def cache_diagnostics(wire, body):
    """Только клон записанной памяти: диагностика не прогревает метрик исходного вызова."""

    metric, incoming, outgoing, count, orientation = decode_call(wire, body)[0]
    memo = metric._density_exact_memo
    result = {"atom_entries_before": len(memo.turn_atoms), "fan_entries_before": len(memo.fans),
              "atom_key_cached": False, "fan_key_cached": False, "zero_hidden_count": count == 0}
    try:
        if count not in range(0, 6):
            result["lookup"] = "count_rejected_before_lookup"
            return result
        key = (*metric.density_expressions(incoming), *metric.density_expressions(outgoing), orientation)
        result["atom_key_cached"] = key in memo.turn_atoms
        fan_key = (key, count, wire.unvalue(body["canonical"]), wire.unvalue(body["rotation"]))
        result["fan_key_cached"] = count != 0 and fan_key in memo.fans
        result["lookup"] = "keyed"
    except Exception as error:  # noqa: BLE001 - отдельная диагностика не подменяет вычислительный исход
        result["lookup"] = "not_available:" + type(error).__name__
    return result


def outcome(wire, metric, result, error, observed):
    body = observed.outcome(None, error, [])
    del body["refusals"]
    if error is None:
        body["answer"] = ["ok", wire.value(result)]
    body["memo_after"] = wire.metric(metric)
    return body


class Recorder:
    def __init__(self, source):
        self.source, self.test = source, ""
        self.wire, self.rows = TurnWire(), []

    def wrapper(self, real=None):
        compute = real or ORACLE

        def recorded(metric, incoming, outgoing, count, orientation_sign, canonical_excess_over_pi=None, rational_rotation=None):
            body = input_body(self.wire, metric, incoming, outgoing, count, orientation_sign,
                              canonical_excess_over_pi, rational_rotation)
            # Wire использует общую DAG-таблицу; декодирование диагностики не меняет её и исходную память.
            diagnostics = cache_diagnostics(self.wire, body)
            with Observation() as observed:
                try:
                    result = compute(metric, incoming, outgoing, count, orientation_sign,
                                     canonical_excess_over_pi, rational_rotation)
                except BaseException as error:  # noqa: BLE001 - исходное исключение и traceback сохраняются
                    self.note(body, outcome(self.wire, metric, None, error, observed), diagnostics)
                    raise
                self.note(body, outcome(self.wire, metric, result, None, observed), diagnostics)
            return result

        return recorded

    def note(self, body, result, diagnostics):
        identifier = hashlib.sha256(json_key([body, result]).encode("utf-8")).hexdigest()
        self.rows.append({"id": identifier, "sequence": len(self.rows), "origin": self.source,
                          "test": self.test, "input": body, "outcome": result, "diagnostics": diagnostics})

    @contextlib.contextmanager
    def installed(self):
        previous, alias = turn._huber_density_interpolated_normals, angular._huber_density_interpolated_normals
        wrapped = self.wrapper(previous)
        turn._huber_density_interpolated_normals = angular._huber_density_interpolated_normals = wrapped
        try:
            yield self
        finally:
            turn._huber_density_interpolated_normals, angular._huber_density_interpolated_normals = previous, alias

    def document(self):
        return {"schema": SCHEMA, "expressions": self.wire.nodes, "records": self.rows}


def run_record(document, row, compute=None):
    # Новый результат может добавить узлы; исходная DAG документа при replay не меняется.
    wire = TurnWire([list(node) for node in document["expressions"]])
    args, kwargs = decode_call(wire, row["input"])
    diagnostics = cache_diagnostics(wire, row["input"])
    saved_counts, saved_symbolic = dict(symbolic.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    saved_disagreements = list(symbolic.DISAGREEMENTS)
    result = error = None
    try:
        with applied_settings(row["input"]["settings"]), Observation() as observed:
            try:
                result = (compute or ORACLE)(*args, **kwargs)
            except Exception as caught:  # noqa: BLE001 - отказ равноправен успешному набору лучей
                error = caught
            return outcome(wire, args[0], result, error, observed), diagnostics
    finally:
        symbolic.BACKEND_COUNTS.clear()
        symbolic.BACKEND_COUNTS.update(saved_counts)
        planar_types.SYMBOLIC_FALLBACK_COUNTS.clear()
        planar_types.SYMBOLIC_FALLBACK_COUNTS.update(saved_symbolic)
        symbolic.DISAGREEMENTS[:] = saved_disagreements


def validate_document(document):
    require_keys(document, ("schema", "expressions", "records"), "turn.document")
    if document["schema"] != SCHEMA:
        raise CorpusWireError("unexpected turn corpus schema")
    if type(document["expressions"]) is not list or type(document["records"]) is not list:
        raise CorpusWireError("turn expressions and records must be arrays")
    wire = TurnWire(document["expressions"])
    if len(wire._index) != len(wire.nodes):
        raise CorpusWireError("turn DAG has duplicate nodes")
    for index in range(len(wire.nodes)):
        wire.unexpression(index)
    for sequence, row in enumerate(document["records"]):
        require_keys(row, ("id", "sequence", "origin", "test", "input", "outcome", "diagnostics"), "turn.record")
        if type(row["sequence"]) is not int or row["sequence"] != sequence:
            raise CorpusWireError("turn sequence is not contiguous")
        decode_call(wire, row["input"])
        require_keys(row["outcome"], OUTCOME_KEYS, "turn.outcome")
        wire.unmetric(row["outcome"]["memo_after"])
        if row["id"] != hashlib.sha256(json_key([row["input"], row["outcome"]]).encode("utf-8")).hexdigest():
            raise CorpusWireError("turn input/outcome digest mismatch")


def write_document(path, document):
    validate_document(document)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = lzma.compress(json_key(document).encode("ascii"), preset=6)
    path.write_bytes(data)
    return len(data)


def read_document(path):
    document = json.loads(lzma.decompress(path.read_bytes()).decode("ascii"))
    validate_document(document)
    return document


def shape_of(document):
    rows = document["records"]
    return {"records": len(rows), "raised": sum(row["outcome"]["answer"][0] == "raised" for row in rows),
            "zero_hidden_count": sum(row["diagnostics"]["zero_hidden_count"] for row in rows),
            "atom_key_cached": sum(row["diagnostics"]["atom_key_cached"] for row in rows),
            "fan_key_cached": sum(row["diagnostics"]["fan_key_cached"] for row in rows),
            "expression_nodes": len(document["expressions"])}


def replay(document, compute=None):
    failures, diagnostic_failures = [], []
    for row in document["records"]:
        actual, diagnostics = run_record(document, row, compute)
        if row["outcome"] != actual:
            failures.append({"sequence": row["sequence"], "test": row["test"],
                             "fields": [key for key in OUTCOME_KEYS if row["outcome"][key] != actual[key]]})
        if row["diagnostics"] != diagnostics:
            diagnostic_failures.append(row["sequence"])
    return {"records": len(document["records"]), "equal": len(document["records"]) - len(failures),
            "mismatch": len(failures), "first_mismatches": failures[:5],
            "diagnostic_mismatch": len(diagnostic_failures), "first_diagnostic_mismatches": diagnostic_failures[:5]}


def _metric(sign=1, negative=False):
    diagonal = sp.Integer(-1 if negative else 1)
    matrix = ((diagonal, sp.Integer(0)), (sp.Integer(0), diagonal))
    from cftuv_envelope.reference.metric import ExactPlanarMetric

    return ExactPlanarMetric(matrix, matrix, sign)


def _vector(x, y):
    return ExpressionVector(sp.sympify(x), sp.sympify(y))


def synthetic_document():
    recorder = Recorder("synthetic_turn")
    call = recorder.wrapper()

    def invoke(name, metric, incoming, outgoing, count, orientation=1, canonical=None, rotation=None):
        recorder.test = name
        try:
            return call(metric, incoming, outgoing, count, orientation, canonical, rotation)
        except (ValueError, TypeError, IndexError):
            # Исключение уже записано с частичной памятью; ни один ошибочный case не выдаётся за успех.
            return None

    for count in range(6):
        metric = _metric()
        for repeat in range(2):
            invoke(f"principal_positive_h{count}:r{repeat}", metric, _vector(1, 0), _vector(1, 2), count)
    for name, outgoing, orientation, owner in (
        ("principal_negative", (-1, 2), 1, 1), ("principal_zero", (0, 1), 1, 1),
        ("clockwise", (1, -2), -1, 1), ("reflected_owner", (1, -2), 1, -1),
    ):
        metric = _metric(owner)
        invoke(name, metric, _vector(1, 0), _vector(*outgoing), 3, orientation)
        invoke(name + ":repeat", metric, _vector(1, 0), _vector(*outgoing), 3, orientation)
    metric = _metric()
    invoke("canonical_half_turn", metric, _vector(1, 0), _vector(sp.Rational(1, 1000), 1), 2, canonical=Fraction(1, 2))
    invoke("canonical_half_turn:repeat", metric, _vector(1, 0), _vector(sp.Rational(1, 1000), 1), 2, canonical=Fraction(1, 2))
    rotation = ((12, 5), (1, 1), (5, 12))
    metric = _metric()
    invoke("rational_table_h3", metric, _vector(1, 0), _vector(0, 1), 3, rotation=rotation)
    invoke("rational_table_h3:repeat", metric, _vector(1, 0), _vector(0, 1), 3, rotation=rotation)
    for count in (-1, 6, "bad", None):
        invoke(f"count_bad:{count}", _metric(), _vector(1, 0), _vector(0, 1), count)
    invoke("collinear", _metric(), _vector(1, 0), _vector(2, 0), 0)
    invoke("zero_vector", _metric(), _vector(0, 0), _vector(0, 1), 1)
    invoke("nonpositive_norm", _metric(negative=True), _vector(1, 0), _vector(0, 1), 1)
    invoke("irrational_norm", _metric(), _vector(sp.sqrt(2 + sp.sqrt(2)), 0), _vector(0, 1), 1)
    metric = _metric()
    outgoing = _vector((sp.sqrt(2) + sp.sqrt(3)) / 2, (sp.sqrt(3) - sp.sqrt(2)) / 2)
    invoke("irrational_cos_h0", metric, _vector(1, 0), outgoing, 0)
    invoke("irrational_cos_h1", metric, _vector(1, 0), outgoing, 1)
    invoke("irrational_cos_h1:repeat", metric, _vector(1, 0), outgoing, 1)
    metric = _metric()
    invoke("short_rotation", metric, _vector(1, 0), _vector(0, 1), 2, rotation=((1, 1),))
    invoke("short_rotation:repeat", metric, _vector(1, 0), _vector(0, 1), 2, rotation=((1, 1),))
    return recorder.document()


def record_kernel_suite(out, arguments):
    import pytest

    recorder = Recorder("kernel_suite_turn")

    class Plugin:
        def pytest_runtest_setup(self, item):
            recorder.test = item.nodeid

    with recorder.installed():
        code = pytest.main(arguments or [str(ROOT / "kernel" / "tests" / "test_plan_compile_fast_paths.py"), "-q"], plugins=[Plugin()])
    write_document(out / "kernel_suite.turns.xz", recorder.document())
    print("NATIVE_DENSITY_TURN_KERNEL " + json_key(shape_of(recorder.document())), flush=True)
    return int(code) if code else (0 if recorder.rows else 1)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    separator = argv.index("--") if "--" in argv else len(argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("synthetic", "kernel-suite", "replay", "summary"))
    parser.add_argument("--out", type=Path, default=Path(os.environ.get("CFTUV_NATIVE_CORPUS", "D:/CFTUV-r3-density-corpus")) / "turn")
    parser.add_argument("--file", type=Path, action="append", default=[])
    args = parser.parse_args(argv[:separator])
    if args.command == "synthetic":
        document = synthetic_document()
        write_document(args.out / "synthetic.turns.xz", document)
        print("NATIVE_DENSITY_TURN_SYNTHETIC " + json_key(shape_of(document)))
        return 0
    if args.command == "kernel-suite":
        return record_kernel_suite(args.out, argv[separator + 1:])
    if not args.file:
        parser.error("replay/summary require --file")
    failed = False
    for path in args.file:
        document = read_document(path)
        result = shape_of(document) if args.command == "summary" else replay(document)
        print("NATIVE_DENSITY_TURN " + json_key({"file": str(path), **result}), flush=True)
        failed |= bool(result.get("mismatch", 0) or result.get("diagnostic_mismatch", 0))
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
