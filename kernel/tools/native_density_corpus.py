"""Корпус публичной границы bind_density_fan: вход, исход, отказы и точная работа.

python tools/native_density_corpus.py kernel-suite --out DIR -- kernel/tests/test_density_exact_limit_lift.py
python tools/native_density_corpus.py synthetic --out DIR
python tools/native_density_corpus.py replay --file DIR/synthetic.recs.xz
python tools/native_density_corpus.py summary --dir DIR

JSON/xz, без pickle. Начальная память metric входит в запись: тёплый hit не
выдаётся за холодный вызов. Считаются ВСЕ DensityExactWorkBudget в порядке создания,
включая preparation и расход перед отказом. Work RadicalSumV2 здесь не наблюдается.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import lzma
import os
import sys
from pathlib import Path

from density_tool_paths import KERNEL_ROOT, PACKAGE_ROOT, TOOLS_ROOT

ROOT = KERNEL_ROOT.parent

from mpmath import iv  # noqa: E402

from cftuv_envelope import codec  # noqa: E402
from cftuv_envelope.reference import adaptive_density_atlas as atlas  # noqa: E402
from cftuv_envelope.reference import adaptive_density_fan as fan  # noqa: E402
from cftuv_envelope.reference import density_fan_binding as binding  # noqa: E402
from cftuv_envelope.reference import planar_types, symbolic_backend as symbolic  # noqa: E402
from cftuv_envelope.reference.contracts import ReferenceDiagnosticSeverity, ReferenceEvaluationDiagnosticV1  # noqa: E402
from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector, SHELL_EVENT_OUTCOMES  # noqa: E402
from native_density_wire import CorpusWireError, Wire, json_key, require_keys  # noqa: E402

SCHEMA = "cftuv.native-density-corpus.v1"
RECORD_ENVIRONMENT = "CFTUV_DENSITY_RECORD"
ORACLE = binding.bind_density_fan
INPUT_KEYS = ("metric", "ideal", "orientation", "q", "reasons", "lifted", "law", "spec_id", "refusals", "settings")
SETTINGS_KEYS = ("search_cap", "preparation_cap", "box_cap", "height_cap", "mode", "policy", "iv_precision")
OUTCOME_KEYS = ("answer", "refusals", "budgets", "counts", "symbolic", "disagreements")
OPTIONAL_OUTCOME_KEYS = ("shell_events",)
EXCLUDED_PATCHED_TESTS = {
    ("test_density_exact_limit_lift.py", "test_box_refinement_exhaustion_name_survives_an_undecided_limit_hint"):
        "patched _termination_boxes / _subturn_boundary and rotation table are not boundary inputs",
    ("test_density_exact_limit_lift.py", "test_box_refinement_exhaustion_has_its_own_honest_name"):
        "patched _termination_boxes feasibility oracle and rotation table are not boundary inputs",
    ("test_plan_compile_dedup.py", "test_bind_density_fan_does_not_remember_a_refusal"):
        "patched certification functions return a fabricated authority, not the oracle",
    ("test_fan_narrow_band.py", "test_a_band_that_cannot_hold_a_ray_is_a_named_refusal_and_the_voronoi_window_decides"):
        "patched narrow-band certification raises a fabricated refusal independent of exact input",
    ("test_adaptive_density_fan_authority.py", "test_public_compile_names_unrepresentable_density_window"):
        "patched certify_density_bindings_and_adaptive_fallback always raises a fabricated chart refusal",
}


def corpus_directory() -> Path:
    return Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus")) / "density"


def delta(before: dict, after: dict) -> dict:
    return {key: after.get(key, 0) - before.get(key, 0) for key in sorted(set(before) | set(after)) if before.get(key, 0) != after.get(key, 0)}


def settings() -> dict:
    return {
        "search_cap": fan._DENSITY_EXACT_WORK_CAP,
        "preparation_cap": atlas.DENSITY_FAN_PREPARATION_WORK_CAP,
        "box_cap": fan._BOX_REFINEMENT_CAP, "height_cap": fan._RESOURCE_HEIGHT_CAP,
        "mode": symbolic.backend_mode().value, "policy": symbolic.disagreement_policy().value,
        "iv_precision": iv.prec,
    }


@contextlib.contextmanager
def applied_settings(body):
    require_keys(body, SETTINGS_KEYS, "settings")
    saved = settings()
    names = ((fan, "_DENSITY_EXACT_WORK_CAP", "search_cap"), (atlas, "DENSITY_FAN_PREPARATION_WORK_CAP", "preparation_cap"),
             (fan, "_BOX_REFINEMENT_CAP", "box_cap"), (fan, "_RESOURCE_HEIGHT_CAP", "height_cap"))
    for _, _, key in names:
        if type(body[key]) is not int or body[key] < 0:
            raise CorpusWireError(f"settings.{key} must be a nonnegative integer")
    if type(body["iv_precision"]) is not int or body["iv_precision"] <= 0:
        raise CorpusWireError("settings.iv_precision must be a positive integer")
    try:
        for module, name, key in names:
            setattr(module, name, body[key])
        iv.prec = body["iv_precision"]
        with symbolic.symbolic_backend(symbolic.SymbolicBackendV1(body["mode"]), symbolic.DisagreementPolicyV1(body["policy"])):
            yield
    finally:
        for module, name, key in names:
            setattr(module, name, saved[key])
        iv.prec = saved["iv_precision"]


class Observation:
    """Наблюдает расход внутри одного вызова; существующую instrumentation не отменяет."""

    def __enter__(self):
        self.budgets = []
        self.before = dict(symbolic.BACKEND_COUNTS)
        self.symbolic_before = dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
        self.disagreements_before = len(symbolic.DISAGREEMENTS)
        self.initialize = atlas.DensityExactWorkBudget.__init__

        def remember(budget, exhausted, cap):
            self.initialize(budget, exhausted, cap)
            self.budgets.append(budget)

        atlas.DensityExactWorkBudget.__init__ = remember
        return self

    def __exit__(self, *_):
        atlas.DensityExactWorkBudget.__init__ = self.initialize

    def outcome(self, result, error, refusals, shell_event_collector=None) -> dict:
        payload = {} if error is None else {
            name: codec.to_canonical_data(getattr(error, name))
            for name in ("outcome", "code", "predicate", "detail") if hasattr(error, name)
        }
        answer = ["ok", codec.to_canonical_data(result)] if error is None else ["raised", type(error).__name__, str(error), payload]
        outcome = {
            "answer": answer, "refusals": [list(item) for item in refusals],
            "budgets": [{"cap": budget.cap, "shell_probes": budget.shell_probes, "order_steps": budget.order_steps,
                         "exhausted_type": budget.exhausted.__name__} for budget in self.budgets],
            "counts": delta(self.before, symbolic.BACKEND_COUNTS),
            "symbolic": delta(self.symbolic_before, planar_types.SYMBOLIC_FALLBACK_COUNTS),
            "disagreements": [list(item) for item in symbolic.DISAGREEMENTS[self.disagreements_before:]],
        }
        events = () if shell_event_collector is None else shell_event_collector.diagnostics()
        if events:
            outcome["shell_events"] = codec.to_canonical_data(events)
        return outcome


class _CallShellEvents:
    """События ЭТОГО bind call; caller compilation продолжает собственный dedup."""

    def __init__(self, caller):
        self.caller, self.local = caller, DensityShellEventCollector()

    def replay(self, spec_id, bundle):
        self.local.replay(spec_id, bundle)
        if self.caller is not None:
            self.caller.replay(spec_id, bundle)

    def diagnostics(self):
        return self.local.diagnostics()


def input_body(wire, metric, ideal, orientation, q, reasons, *, lifted, law, spec_id, refusals):
    if type(ideal) is not tuple:
        raise CorpusWireError("bind_density_fan corpus accepts the plain tuple used by compile; attributed tuple subclasses are not value-equivalent")
    return {
        "metric": wire.metric(metric), "ideal": [wire.value(normal) for normal in ideal],
        "orientation": orientation.value, "q": q, "reasons": wire.value(reasons),
        "lifted": lifted, "law": law, "spec_id": spec_id,
        "refusals": [list(item) for item in refusals], "settings": settings(),
    }


def decode_call(wire: Wire, body: dict):
    from cftuv_envelope.contracts.analysis import TurnOrientation

    require_keys(body, INPUT_KEYS, "input")
    if type(body["q"]) is not int or type(body["lifted"]) is not bool:
        raise CorpusWireError("input.q must be an integer and lifted a boolean")
    if any(type(body[name]) is not str for name in ("law", "spec_id", "orientation")):
        raise CorpusWireError("law, spec_id, orientation must be strings")
    if type(body["ideal"]) is not list or type(body["refusals"]) is not list:
        raise CorpusWireError("ideal and refusals must be arrays")
    if any(type(item) is not list or len(item) != 2 or any(type(part) is not str for part in item) for item in body["refusals"]):
        raise CorpusWireError("refusals must be string pairs")
    reasons = wire.unvalue(body["reasons"])
    if type(reasons) is not tuple:
        raise CorpusWireError("binding reasons must be a tuple")
    args = (wire.unmetric(body["metric"]), tuple(wire.unvalue(item) for item in body["ideal"]),
            TurnOrientation(body["orientation"]), body["q"], reasons)
    kwargs = {name: body[name] for name in ("lifted", "law", "spec_id")}
    kwargs["refusals"] = [tuple(item) for item in body["refusals"]]
    return args, kwargs


def run_record(document: dict, row: dict, compute=None) -> dict:
    """Исход replay: исключение — такой же сравниваемый исход, как результат."""

    wire = Wire(document["expressions"])
    args, kwargs = decode_call(wire, row["input"])
    collector = DensityShellEventCollector()
    kwargs["shell_event_collector"] = collector
    saved_counts, saved_symbolic = dict(symbolic.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    saved_disagreements = list(symbolic.DISAGREEMENTS)
    result = error = None
    try:
        with applied_settings(row["input"]["settings"]), Observation() as observed:
            try:
                result = (compute or ORACLE)(*args, **kwargs)
            except Exception as caught:  # noqa: BLE001 - исключение является ответом; не успешным fallback
                error = caught
            return observed.outcome(result, error, kwargs["refusals"], collector)
    finally:
        symbolic.BACKEND_COUNTS.clear()
        symbolic.BACKEND_COUNTS.update(saved_counts)
        planar_types.SYMBOLIC_FALLBACK_COUNTS.clear()
        planar_types.SYMBOLIC_FALLBACK_COUNTS.update(saved_symbolic)
        symbolic.DISAGREEMENTS[:] = saved_disagreements


class Recorder:
    """Каждый invocation сохраняется по порядку; cache hit не дедуплицируется с холодным."""

    def __init__(self, source: str) -> None:
        self.source, self.test = source, ""
        self.wire = Wire()
        self.rows = []
        self.enabled = True
        self.exclusion_reason = "explicitly excluded patched oracle"
        self.excluded_tests: dict = {}

    def wrapper(self, real=None):
        compute = real or ORACLE

        def recorded(metric, ideal, orientation, q, binding_reasons, *, lifted, law, spec_id, refusals,
                     shell_event_collector=None):
            if not self.enabled:
                excluded = self.excluded_tests.setdefault(self.test, {"calls": 0, "reason": self.exclusion_reason})
                excluded["calls"] += 1
                return compute(metric, ideal, orientation, q, binding_reasons,
                               lifted=lifted, law=law, spec_id=spec_id, refusals=refusals,
                               shell_event_collector=shell_event_collector)
            body = input_body(self.wire, metric, ideal, orientation, q, binding_reasons,
                              lifted=lifted, law=law, spec_id=spec_id, refusals=refusals)
            result = None
            collector = _CallShellEvents(shell_event_collector)
            with Observation() as observed:
                try:
                    result = compute(metric, ideal, orientation, q, binding_reasons,
                                     lifted=lifted, law=law, spec_id=spec_id, refusals=refusals,
                                     shell_event_collector=collector)
                except BaseException as error:  # noqa: BLE001 - запись перед исходным raise, traceback остаётся прежним
                    self.note(body, observed.outcome(None, error, refusals, collector))
                    raise
                self.note(body, observed.outcome(result, None, refusals, collector))
            return result

        return recorded

    def note(self, body, outcome):
        digest = hashlib.sha256(json_key([body, outcome]).encode("utf-8")).hexdigest()
        self.rows.append({"id": digest, "origin": self.source, "test": self.test,
                          "sequence": len(self.rows), "input": body, "outcome": outcome})

    @contextlib.contextmanager
    def installed(self):
        from cftuv_envelope.reference import compile as compile_module

        previous, compile_previous = binding.bind_density_fan, compile_module.bind_density_fan
        wrapped = self.wrapper(previous)
        binding.bind_density_fan = compile_module.bind_density_fan = wrapped
        try:
            yield self
        finally:
            binding.bind_density_fan, compile_module.bind_density_fan = previous, compile_previous

    def document(self) -> dict:
        return {"schema": SCHEMA, "expressions": self.wire.nodes, "records": self.rows,
                "excluded_patched_calls": self.excluded_tests}


def validate_document(document):
    require_keys(document, ("schema", "expressions", "records", "excluded_patched_calls"), "document")
    if document["schema"] != SCHEMA:
        raise CorpusWireError(f"expected {SCHEMA}, found {document['schema']!r}")
    if type(document["expressions"]) is not list or type(document["records"]) is not list:
        raise CorpusWireError("expressions and records must be arrays")
    excluded = document["excluded_patched_calls"]
    if type(excluded) is not dict:
        raise CorpusWireError("excluded_patched_calls must be a map")
    for key, body in excluded.items():
        require_keys(body, ("calls", "reason"), "excluded_patched_calls entry")
        if type(key) is not str or type(body["calls"]) is not int or body["calls"] <= 0 or type(body["reason"]) is not str:
            raise CorpusWireError("excluded_patched_calls must name tests, reasons and positive call counts")
    wire = Wire(document["expressions"])
    for index in range(len(wire.nodes)):
        wire.unexpression(index)
    for sequence, row in enumerate(document["records"]):
        require_keys(row, ("id", "origin", "test", "sequence", "input", "outcome"), "record")
        if row["sequence"] != sequence:
            raise CorpusWireError("record sequence is not contiguous")
        decode_call(wire, row["input"])
        validate_outcome(row["outcome"])
        digest = hashlib.sha256(json_key([row["input"], row["outcome"]]).encode("utf-8")).hexdigest()
        if row["id"] != digest:
            raise CorpusWireError("record input/outcome digest does not match")


def validate_outcome(outcome):
    if type(outcome) is not dict:
        raise CorpusWireError("outcome must be a map")
    optional = tuple(key for key in OPTIONAL_OUTCOME_KEYS if key in outcome)
    require_keys(outcome, (*OUTCOME_KEYS, *optional), "outcome")
    if "shell_events" not in outcome:
        return
    events = outcome["shell_events"]
    if type(events) is not list or not events:
        raise CorpusWireError("shell_events must be a nonempty diagnostic array")
    for item in events:
        diagnostic = codec._decode_as(item, ReferenceEvaluationDiagnosticV1)
        if (diagnostic.outcome not in SHELL_EVENT_OUTCOMES or diagnostic.severity is not ReferenceDiagnosticSeverity.INFO
                or type(diagnostic.envelope_spec_id) is not str):
            raise CorpusWireError("shell_events must contain named INFO density-shell diagnostics")


def write_document(path: Path, document: dict) -> int:
    validate_document(document)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = lzma.compress(json_key(document).encode("ascii"), preset=6)
    path.write_bytes(data)
    return len(data)


def read_document(path: Path) -> dict:
    document = json.loads(lzma.decompress(path.read_bytes()).decode("ascii"))
    validate_document(document)
    return document


def shape_of(document: dict) -> dict:
    rows = document["records"]
    return {
        "records": len(rows), "raised": sum(row["outcome"]["answer"][0] == "raised" for row in rows),
        "refusals": sum(len(row["outcome"]["refusals"]) - len(row["input"]["refusals"]) for row in rows),
        "without_search_work": sum(not row["outcome"]["budgets"] for row in rows),
        "shell_probes": sum(b["shell_probes"] for row in rows for b in row["outcome"]["budgets"]),
        "order_steps": sum(b["order_steps"] for row in rows for b in row["outcome"]["budgets"]),
        "expression_nodes": len(document["expressions"]),
        "excluded_patched_calls": sum(item["calls"] for item in document["excluded_patched_calls"].values()),
        "excluded_patched_tests": document["excluded_patched_calls"],
    }


def replay(document: dict, limit: int | None = None, compute=None) -> dict:
    failures = []
    rows = document["records"][:limit]
    for row in rows:
        actual = run_record(document, row, compute)
        if row["outcome"] != actual:
            keys = (*OUTCOME_KEYS, *(key for key in OPTIONAL_OUTCOME_KEYS if key in row["outcome"] or key in actual))
            fields = [key for key in keys if row["outcome"].get(key, []) != actual.get(key, [])]
            failures.append({"sequence": row["sequence"], "test": row["test"], "fields": fields,
                             "expected": row["outcome"], "actual": actual})
    return {"records": len(rows), "equal": len(rows) - len(failures), "mismatch": len(failures), "first_mismatches": failures[:3]}


def record_kernel_suite(out: Path, arguments) -> int:
    import pytest

    recorder = Recorder("kernel_suite")

    class Plugin:
        def pytest_runtest_setup(self, item):
            recorder.test = item.nodeid
            # Только перечисленные semantic-law подмены. Unrelated/cap-only monkeypatch записывается.
            key = (Path(item.path).name, getattr(item, "originalname", None) or item.name.split("[")[0])
            recorder.exclusion_reason = EXCLUDED_PATCHED_TESTS.get(key, "")
            recorder.enabled = not recorder.exclusion_reason

    with recorder.installed():
        code = pytest.main(arguments or [str(KERNEL_ROOT / "tests" / "test_density_exact_limit_lift.py"), "-q"], plugins=[Plugin()])
    document = recorder.document()
    write_document(out / "kernel_suite.recs.xz", document)
    print("NATIVE_DENSITY_KERNEL_SUITE " + json_key(shape_of(document)), flush=True)
    return int(code) if code else (0 if document["records"] else 1)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    separator = argv.index("--") if "--" in argv else len(argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("kernel-suite", "synthetic", "replay", "summary"))
    parser.add_argument("--out", type=Path, default=corpus_directory())
    parser.add_argument("--dir", type=Path)
    parser.add_argument("--file", type=Path, action="append", default=[])
    parser.add_argument("--limit", type=int)
    args = parser.parse_args(argv[:separator])
    if args.command == "kernel-suite":
        return record_kernel_suite(args.out, argv[separator + 1:])
    if args.command == "synthetic":
        from native_density_synthetic import synthetic_document

        document = synthetic_document()
        write_document(args.out / "synthetic.recs.xz", document)
        print("NATIVE_DENSITY_SYNTHETIC " + json_key(shape_of(document)))
        return 0
    if args.command == "summary":
        for path in sorted((args.dir or args.out).rglob("*.recs.xz")):
            print(json_key({"file": str(path), **shape_of(read_document(path))}))
        return 0
    if not args.file:
        parser.error("replay requires --file")
    failed = False
    for path in args.file:
        result = replay(read_document(path), args.limit)
        print("NATIVE_DENSITY_REPLAY " + json_key({"file": str(path), **result}), flush=True)
        failed |= bool(result["mismatch"])
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
