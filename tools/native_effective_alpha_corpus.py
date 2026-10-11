"""Corpus of the pair step of the effective-alpha resolver for the native `effective_alpha` port: record, store, read, replay, compare.

The unit is one PAIR: `boundary._contact_candidates(context, source, boundary, frame)` -- a source segment against a barrier segment in the metric of the context, with or without the
per-source `SourceContactFrame` -- under one symbolic backend mode. Its answer is the ordered contacts `(alpha, station, point)` (`RadicalSumV1` TERMS with their representatives and
order, the two texts of the point) or the exception it raised, plus what it did to two process counters: `symbolic_backend.BACKEND_COUNTS` (the `contact_candidates.*` of the prefilter and
of the native/refusal outcomes) and `planar_types.SYMBOLIC_FALLBACK_COUNTS` (the sympy route). The native port is ONE operation, `effective_alpha`, behind the same call; equality is the
contacts, the exception and BOTH counter deltas.

A record keeps ONLY what the pair reads, in texts and ints (no object of the kernel): the metric (gram, inverse gram, orientation), the source segment (the texts of its points and the wires of
its tangent and owner normal), the barrier segment (its id and the texts of its points). Files: `<name>.recs.xz` = xz(JSON object `{"schema", "metrics", "sources", "pairs"}`), the pairs
refer to metrics and sources by index. The corpus directory (`E:/cftuv_native_corpus/effective_alpha`, `CFTUV_NATIVE_CORPUS` moves the base) holds `field/<mesh>.recs.xz`,
`kernel_suite.recs.xz`, `synthetic.recs.xz` and `index.json`; `tests/data/native_effective_alpha_kernel_suite.recs.xz` is the small tracked seed of the kernel suite.

    python tools/native_effective_alpha_corpus.py kernel-suite [--out DIR] [-- <pytest arguments>]   records the pairs the kernel tests make (pytest in this process)
    python tools/native_effective_alpha_corpus.py synthetic [--out DIR] [--cases N]                  writes the generated adversarial pairs with the oracle's answers
    python tools/native_effective_alpha_corpus.py seed --from FILE --out FILE [--limit N]            the tracked seed: a deterministic cover of a recorded file
    python tools/native_effective_alpha_corpus.py summary [--dir DIR]                                 what is in the corpus
    python tools/native_effective_alpha_corpus.py replay --file FILE [--max-pairs N]                  the three-sided comparison without pytest (Blender's Python 3.11 has none)
    python tools/native_effective_alpha_corpus.py digest [--cases S] [--only a,b]                     one JSON line: the digests of the generated pairs on the three sides (the sweep of PYTHONHASHSEED)

As a pytest plugin (`-p native_effective_alpha_corpus`, environment `CFTUV_ALPHA_RECORD=<file>`) it records the pairs of any run; the field recorder is `tools/native_effective_alpha_export.py`.
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
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SOURCE = ROOT / "kernel" / "src"
for _path in (KERNEL_SOURCE, ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import sympy as sp  # noqa: E402

from cftuv_envelope import backend as kernel_backend  # noqa: E402
from cftuv_envelope.reference import boundary as boundary_module  # noqa: E402
from cftuv_envelope.reference import planar_types as planar_types  # noqa: E402
from cftuv_envelope.reference import symbolic_backend as symbolic  # noqa: E402
from cftuv_envelope.reference.boundary_native import SourceContactFrame  # noqa: E402
from cftuv_envelope.reference.metric import ExactPlanarMetric  # noqa: E402
from cftuv_envelope.reference.native_exact import RadicalSumV1  # noqa: E402

#: The pair function of the oracle, taken at import: replacing the module attribute (the recorder does) does not change it.
ORACLE = boundary_module._contact_candidates
SCHEMA = "cftuv.native-effective-alpha-corpus.v1"
DEFAULT_BASE = "E:/cftuv_native_corpus"
CORPUS_ENVIRONMENT = "CFTUV_NATIVE_CORPUS"
RECORD_ENVIRONMENT = "CFTUV_ALPHA_RECORD"
#: The files of the oracle the native operation is pinned to (`cftuv_native.pin.EFFECTIVE_ALPHA_FILES`; a test holds the equality).
ORACLE_FILES = ("_cpython311.py", "float_filter.py", "reference/boundary_native.py", "reference/native_exact.py")
#: The counters of the pair that mean "the oracle's native path refused" (the pair went to sympy).
REFUSAL_COUNTERS = ("contact_candidates.outside_field", "contact_candidates.sign_undecided")
NATIVE_ORDER = ("PYTHON", "PYTHON", "PYTHON", "NATIVE")


class CorpusError(RuntimeError):
    """The corpus is malformed or missing: a named failure, not a quiet "almost"."""


# --------------------------------------------------------------------------
# canonical encodings
# --------------------------------------------------------------------------


def encode_value(value) -> list:
    """A contact's alpha or station: the TERMS of a `RadicalSumV1` with their representatives (int is not Fraction), or the srepr of a sympy value (the sympy route)."""

    if type(value) is RadicalSumV1:
        return ["R", [[int(radicand), int(coefficient.numerator), int(coefficient.denominator)] for radicand, coefficient in value.terms]]
    return ["S", sp.srepr(value)]


def encode_contacts(contacts) -> list:
    return [[encode_value(alpha), encode_value(station), point.x.expression, point.y.expression] for alpha, station, point in contacts]


def delta(before: dict, after: dict) -> dict:
    keys = set(before) | set(after)
    return {key: after.get(key, 0) - before.get(key, 0) for key in sorted(keys) if after.get(key, 0) != before.get(key, 0)}


def vector_wire(vector) -> list:
    """The texts of an `ExactPlanarVector` (`"T"`) or the srepr of a runtime vector with sympy components (`"E"`, the density sources)."""

    if isinstance(getattr(vector, "x", None), planar_types.ExactScalar):
        return ["T", vector.x.expression, vector.y.expression]
    x, y = vector.expressions()
    return ["E", sp.srepr(x), sp.srepr(y)]


def _entry(entry) -> list:
    return [int(entry.p), int(entry.q)] if getattr(entry, "is_Rational", False) else ["S", sp.srepr(entry)]


def _unentry(value):
    return sp.sympify(value[1]) if value and value[0] == "S" else sp.Rational(value[0], value[1])


def metric_body(metric) -> dict:
    return {
        "gram": [[_entry(item) for item in row] for row in metric.gram],
        "inverse": [[_entry(item) for item in row] for row in metric.inverse_gram],
        "sign": int(metric.owner_orientation_sign),
    }


def source_body(metric_index: int, source) -> dict:
    return {
        "metric": metric_index, "support_id": str(source.support_id),
        "start": [source.start.x.expression, source.start.y.expression], "end": [source.end.x.expression, source.end.y.expression],
        "tangent": vector_wire(source.tangent), "normal": vector_wire(source.owner_normal),
    }


def pair_id(body: dict) -> str:
    text = json.dumps([body["source"], body["segment_id"], body["start"], body["end"], body["mode"], body["policy"], body["frame"], body.get("fault")], separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:20]


# --------------------------------------------------------------------------
# the replay: records -> objects -> outcomes
# --------------------------------------------------------------------------


def _point(pair):
    return planar_types.ExactPlanarPoint(planar_types.ExactScalar(pair[0]), planar_types.ExactScalar(pair[1]))


def _vector(wire):
    if wire[0] == "T":
        return planar_types.ExactPlanarVector(planar_types.ExactScalar(wire[1]), planar_types.ExactScalar(wire[2]))
    from cftuv_envelope.reference.angular import _density_runtime_vector

    return _density_runtime_vector(sp.sympify(wire[1]), sp.sympify(wire[2]))


class Objects:
    """The decoded metrics and sources of a corpus file, and a per-source `SourceContactFrame` for each side of a comparison (`python` / `native`): the frames of one source are
    shared by its pairs in record order, as the resolver shares them, and one frame never serves both sides (a frame keeps the converted copy the native operation made)."""

    def __init__(self, document: dict) -> None:
        if document.get("schema") != SCHEMA:
            raise CorpusError(f"the corpus file is {document.get('schema')!r}, not {SCHEMA!r}")
        self.document = document
        self.contexts = [
            SimpleNamespace(metric=ExactPlanarMetric(
                tuple(tuple(_unentry(item) for item in row) for row in body["gram"]),
                tuple(tuple(_unentry(item) for item in row) for row in body["inverse"]), int(body["sign"])))
            for body in document["metrics"]
        ]
        self.sources = [
            SimpleNamespace(
                start=_point(body["start"]), end=_point(body["end"]), tangent=_vector(body["tangent"]), owner_normal=_vector(body["normal"]),
                support_id=body["support_id"])
            for body in document["sources"]
        ]
        self.frames: dict = {}

    def call(self, pair: dict, side: str) -> tuple:
        """`(context, source, boundary, frame)` of a pair for one side of a comparison."""

        source = self.sources[pair["source"]]
        context = self.contexts[self.document["sources"][pair["source"]]["metric"]]
        boundary = SimpleNamespace(segment=SimpleNamespace(start=_point(pair["start"]), end=_point(pair["end"]), segment_id=pair["segment_id"]))
        frame = None
        if pair["frame"]:
            key = (pair["source"], side)
            frame = self.frames.get(key)
            if frame is None:
                frame = self.frames[key] = SourceContactFrame(context, source)
        return context, source, boundary, frame


def run_pair(objects: Objects, pair: dict, side: str, *, instrument=None) -> dict:
    """What the pair does on `side` (`python`: no ledger, the oracle; `native`: the stage `effective_alpha` ordered NATIVE): the contacts or the exception, and both counter deltas.

    The result also names, for `native`, how many pairs the extension computed and which it declined (`ledger`). `instrument(pair)`, when given, is a context manager around the call (fault injection).
    """

    context, source, boundary, frame = objects.call(pair, side)
    mode = symbolic.SymbolicBackendV1(pair["mode"])
    policy = symbolic.DisagreementPolicyV1(pair.get("policy", "RAISE"))
    ledger = None
    before, symbolic_before = dict(symbolic.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    answer = None
    with symbolic.symbolic_backend(mode, policy), (instrument(pair) if instrument is not None else contextlib.nullcontext()):
        try:
            if side == "native":
                with kernel_backend.use_backend(*NATIVE_ORDER) as ledger:
                    contacts = ORACLE(context, source, boundary, frame)
            else:
                contacts = ORACLE(context, source, boundary, frame)
            answer = ["ok", encode_contacts(contacts)]
        except Exception as error:  # noqa: BLE001 - the exception IS the answer
            answer = ["raised", type(error).__name__, str(error)]
    outcome = {"answer": answer, "counts": delta(before, dict(symbolic.BACKEND_COUNTS)), "symbolic": delta(symbolic_before, dict(planar_types.SYMBOLIC_FALLBACK_COUNTS))}
    if ledger is not None:
        record = ledger.record()
        outcome["ledger"] = {"native": record.effective_alpha_native_calls, "python": record.effective_alpha_python_calls, "fallbacks": [list(item) for item in record.effective_alpha_fallbacks]}
    return outcome


def recorded_outcome(pair: dict) -> dict:
    return {"answer": pair["answer"], "counts": pair["counts"], "symbolic": pair["symbolic"]}


def equal_outcomes(left: dict, right: dict) -> bool:
    """The three things that make a pair equal: the answer, the delta of `BACKEND_COUNTS` and the delta of `SYMBOLIC_FALLBACK_COUNTS`."""

    return all(left[name] == right[name] for name in ("answer", "counts", "symbolic"))


# --------------------------------------------------------------------------
# the recorder
# --------------------------------------------------------------------------


class Recorder:
    """Wraps `boundary._contact_candidates`: every call is computed as the oracle computes it and recorded once per distinct input (with a call count)."""

    def __init__(self, source: str, limit: int | None = None) -> None:
        self.source, self.limit = source, limit
        self.metrics: list = []
        self.sources: list = []
        self.pairs: dict = {}
        self._metric_index: dict = {}
        self._source_index: dict = {}
        self._keep: list = []  # the objects whose id() keys the tables stay alive while recording
        self.calls = 0
        self.skipped = 0
        self.enabled = True
        self.test = ""

    def _metric(self, metric) -> int:
        found = self._metric_index.get(id(metric))
        if found is None:
            self._keep.append(metric)
            body = metric_body(metric)
            key = json.dumps(body, sort_keys=True)
            found = self._metric_index.get(key)
            if found is None:
                found = self._metric_index[key] = len(self.metrics)
                self.metrics.append(body)
            self._metric_index[id(metric)] = found
        return found

    def _source(self, metric_index: int, source) -> int:
        body = source_body(metric_index, source)
        key = json.dumps(body, sort_keys=True)
        found = self._source_index.get(key)
        if found is None:
            found = self._source_index[key] = len(self.sources)
            self.sources.append(body)
        return found

    def wrapper(self):
        recorder, real = self, ORACLE

        def recorded(context, source, boundary, frame=None):
            if not recorder.enabled:
                recorder.skipped += 1
                return real(context, source, boundary, frame)
            recorder.calls += 1
            before, symbolic_before = dict(symbolic.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
            error = result = None
            try:
                result = real(context, source, boundary, frame)
            except BaseException as caught:  # noqa: BLE001
                error = caught
            segment = boundary.segment
            body = {
                "source": recorder._source(recorder._metric(context.metric), source), "segment_id": str(segment.segment_id),
                "start": [segment.start.x.expression, segment.start.y.expression], "end": [segment.end.x.expression, segment.end.y.expression],
                "mode": symbolic.backend_mode().value, "policy": symbolic.disagreement_policy().value, "frame": frame is not None,
            }
            key = pair_id(body)
            found = recorder.pairs.get(key)
            if found is not None:
                found["count"] += 1
            elif recorder.limit is None or len(recorder.pairs) < recorder.limit:
                body.update(
                    id=key, origin=recorder.source, count=1,
                    answer=["raised", type(error).__name__, str(error)] if error is not None else ["ok", encode_contacts(result)],
                    counts=delta(before, dict(symbolic.BACKEND_COUNTS)), symbolic=delta(symbolic_before, dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)),
                )
                if recorder.test:
                    body["test"] = recorder.test
                recorder.pairs[key] = body
            if error is not None:
                raise error
            return result

        return recorded

    @contextlib.contextmanager
    def installed(self):
        previous = boundary_module._contact_candidates
        boundary_module._contact_candidates = self.wrapper()
        try:
            yield self
        finally:
            boundary_module._contact_candidates = previous

    def document(self) -> dict:
        return {"schema": SCHEMA, "metrics": self.metrics, "sources": self.sources, "pairs": list(self.pairs.values())}


# --------------------------------------------------------------------------
# files
# --------------------------------------------------------------------------


def corpus_directory(base: str | Path | None = None) -> Path:
    return Path(base or os.environ.get(CORPUS_ENVIRONMENT) or DEFAULT_BASE) / "effective_alpha"


def write_document(path: Path, document: dict) -> int:
    """xz(JSON(document)); the size in bytes."""

    path.parent.mkdir(parents=True, exist_ok=True)
    data = lzma.compress(json.dumps(document, separators=(",", ":"), ensure_ascii=True).encode("ascii"), preset=6)
    path.write_bytes(data)
    return len(data)


def read_document(path: Path) -> dict:
    try:
        document = json.loads(lzma.decompress(path.read_bytes()).decode("ascii"))
    except (OSError, lzma.LZMAError, ValueError, EOFError) as error:
        raise CorpusError(f"cannot read the effective-alpha corpus file {path}: {error}") from error
    if document.get("schema") != SCHEMA:
        raise CorpusError(f"{path} is {document.get('schema')!r}, not {SCHEMA!r}")
    return document


def corpus_files(directory: Path | None = None, kind: str | None = None) -> list:
    """The `.recs.xz` files of the corpus (or of `kind`: `field`, `kernel_suite`, `synthetic`)."""

    directory = directory or corpus_directory()
    paths = sorted(directory.rglob("*.recs.xz"))
    if kind is not None:
        paths = [path for path in paths if path.parent.name == kind or path.name == f"{kind}.recs.xz"]
    return paths


def merge_documents(documents) -> dict:
    """One document from several: metrics and sources re-indexed, pairs deduplicated by id (counts add)."""

    metrics: list = []
    sources: list = []
    metric_index: dict = {}
    source_index: dict = {}
    pairs: dict = {}

    def metric_of(body: dict) -> int:
        key = json.dumps(body, sort_keys=True)
        found = metric_index.get(key)
        if found is None:
            found = metric_index[key] = len(metrics)
            metrics.append(body)
        return found

    def source_of(body: dict, metric: int) -> int:
        moved = {**body, "metric": metric}
        key = json.dumps(moved, sort_keys=True)
        found = source_index.get(key)
        if found is None:
            found = source_index[key] = len(sources)
            sources.append(moved)
        return found

    for document in documents:
        local: dict = {}  # a source of the document -> its index in the result, made when a pair first needs it (a source no pair refers to is not kept)
        for pair in document["pairs"]:
            index = pair["source"]
            if index not in local:
                body = document["sources"][index]
                local[index] = source_of(body, metric_of(document["metrics"][body["metric"]]))
            moved = {**pair, "source": local[index]}
            moved["id"] = pair_id(moved)
            found = pairs.get(moved["id"])
            if found is None:
                pairs[moved["id"]] = moved
            else:
                found["count"] += moved["count"]
    return {"schema": SCHEMA, "metrics": metrics, "sources": sources, "pairs": list(pairs.values())}


def shape_of(document: dict) -> dict:
    """Counts for the index and the report."""

    pairs = document["pairs"]
    summary = {
        "pairs": len(pairs), "calls": sum(item["count"] for item in pairs), "metrics": len(document["metrics"]), "sources": len(document["sources"]),
        "with_frame": sum(item["frame"] for item in pairs), "raised": 0, "refused": 0, "via_sympy": 0, "empty": 0, "contacts": 0, "rejected": 0, "passed": 0,
        "irrational_contacts": 0, "modes": {},
    }
    for item in pairs:
        answer = item["answer"]
        summary["modes"][item["mode"]] = summary["modes"].get(item["mode"], 0) + 1
        summary["raised"] += answer[0] == "raised"
        summary["refused"] += any(name in item["counts"] for name in REFUSAL_COUNTERS)
        summary["via_sympy"] += bool(item["symbolic"])
        summary["rejected"] += item["counts"].get("contact_candidates.prefilter_rejected", 0)
        summary["passed"] += item["counts"].get("contact_candidates.prefilter_passed", 0)
        if answer[0] == "ok":
            summary["empty"] += not answer[1]
            summary["contacts"] += len(answer[1])
            summary["irrational_contacts"] += sum(1 for contact in answer[1] if contact[0][0] == "S" or any(term[0] != 1 for term in contact[0][1]))
    return summary


def oracle_digest(root: Path | None = None) -> str:
    """sha256 over the digests of the oracle files the port is pinned to (line endings normalised)."""

    base = (root or KERNEL_SOURCE) / "cftuv_envelope"
    lines = "".join(f"{name} {hashlib.sha256((base / name).read_bytes().replace(chr(13).encode() + chr(10).encode(), chr(10).encode())).hexdigest()}\n" for name in ORACLE_FILES)
    return hashlib.sha256(lines.encode("ascii")).hexdigest()


def write_index(directory: Path, extra: dict) -> None:
    """`index.json`: what oracle it was recorded against and what each source holds; `extra` is merged into the one already there, and each source keeps ITS OWN provenance."""

    path = directory / "index.json"
    index = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    origin = {"python": sys.version.split()[0], "oracle_digest": oracle_digest()}
    for name in ("blender", "scene", "scene_sha256"):
        if name in extra:  # the provenance of a source, not a property of the whole corpus
            origin[name] = extra[name]
    extra = {key: value for key, value in extra.items() if key not in ("blender", "scene", "scene_sha256")}
    provenance = index.setdefault("provenance", {})
    for source in ("kernel_suite", "synthetic"):
        if source in extra:
            provenance[source] = dict(origin)
    if "field" in extra:  # every mesh of the field carries ITS OWN scene and Blender: the meshes of one corpus come from different runs and files
        provenance["field"] = {**provenance.get("field", {}), **{mesh: dict(origin) for mesh in extra["field"]}}
        extra = {**extra, "field": {**index.get("field", {}), **extra["field"]}}
    index.pop("python", None)
    index.update({"schema": SCHEMA, "oracle_files": list(ORACLE_FILES), "oracle_digest": oracle_digest(), **extra})
    path.write_text(json.dumps(index, indent=1, sort_keys=True), encoding="utf-8")


# --------------------------------------------------------------------------
# the seed: a deterministic cover of a recorded file
# --------------------------------------------------------------------------


def _category(pair: dict) -> str:
    answer = pair["answer"]
    if answer[0] == "raised":
        return "raised"
    if any(name in pair["counts"] for name in REFUSAL_COUNTERS):
        return "refused"
    if pair["symbolic"]:
        return "via_sympy"
    if pair["mode"] != "NATIVE_EXACT":
        return f"mode:{pair['mode']}"
    if not pair["frame"]:
        return "no-frame"
    if not answer[1]:
        return "rejected" if "contact_candidates.prefilter_rejected" in pair["counts"] else "empty"
    if len(answer[1]) > 1:
        return "several"
    return "irrational" if any(term[0] != 1 for term in answer[1][0][0][1]) or answer[1][0][0][0] == "S" else "single"


def _strided(indexes: list, count: int) -> list:
    """`count` of `indexes`, evenly spread over them (all of them when there are no more than `count`)."""

    if count >= len(indexes):
        return list(indexes)
    return [indexes[(position * len(indexes)) // count] for position in range(count)]


def seed_of(document: dict, limit: int) -> dict:
    """At most `limit` pairs that COVER the document: every category (raised, refused, via sympy, each other mode, no frame, rejected, empty, single, several contacts, irrational) gets an equal share
    of the budget, spread evenly over its pairs in the order of the file; a category with fewer pairs than its share gives the rest to the others, in the order of their names.

    The metrics and sources the kept pairs refer to are kept, re-indexed; the order of the pairs is the order of the file. A pure function of the document: the same input gives the same seed.
    """

    pairs = document["pairs"]
    categories: dict = {}
    for index, pair in enumerate(pairs):
        categories.setdefault(_category(pair), []).append(index)
    names = sorted(categories)
    chosen: dict = {name: [] for name in names}
    budget = min(limit, len(pairs))
    while budget and any(len(chosen[name]) < len(categories[name]) for name in names):
        open_names = [name for name in names if len(chosen[name]) < len(categories[name])]
        share = max(1, budget // len(open_names))
        for name in open_names:
            if not budget:
                break
            take = min(share, budget, len(categories[name]) - len(chosen[name]))
            chosen[name] = _strided(categories[name], len(chosen[name]) + take)
            budget -= take
    kept = sorted(index for indexes in chosen.values() for index in indexes)
    selected = {"schema": SCHEMA, "metrics": document["metrics"], "sources": document["sources"], "pairs": [pairs[index] for index in kept]}
    return merge_documents([selected])


# --------------------------------------------------------------------------
# command line
# --------------------------------------------------------------------------


def record_kernel_suite(out: Path, extra_arguments=()) -> int:
    """Runs the kernel tests under the recorder (in this process) and writes `kernel_suite.recs.xz`."""

    import pytest

    recorder = Recorder("kernel-suite")
    plugin = _Plugin(recorder)
    with recorder.installed():
        code = pytest.main(["-q", "--no-header", "-p", "no:cacheprovider", "--rootdir", str(ROOT / "kernel"), str(ROOT / "kernel" / "tests"), *extra_arguments], plugins=[plugin])
    document = recorder.document()
    size = write_document(out / "kernel_suite.recs.xz", document)
    print(f"EFFECTIVE_ALPHA_KERNEL_SUITE pytest_exit={int(code)} calls={recorder.calls} skipped={recorder.skipped} pairs={len(document['pairs'])} bytes={size}")
    write_index(out, {"kernel_suite": {**shape_of(document), "bytes": size, "calls_recorded": recorder.calls}})
    return int(code)


class _Plugin:
    """pytest hooks of a recording run: a test that patches what is recorded is not a statement about it (its calls are not recorded)."""

    def __init__(self, recorder: Recorder) -> None:
        self.recorder = recorder

    def pytest_runtest_setup(self, item):
        self.recorder.test = item.nodeid
        self.recorder.enabled = "monkeypatch" not in getattr(item, "fixturenames", ())


_ACTIVE: list = []


def pytest_configure(config):
    path = os.environ.get(RECORD_ENVIRONMENT)
    if path:
        recorder = Recorder("pytest")
        _ACTIVE.append((recorder, path, recorder.installed()))
        _ACTIVE[0][2].__enter__()
        config.pluginmanager.register(_Plugin(recorder), "native_effective_alpha_recorder")


def pytest_sessionfinish(session, exitstatus):
    if _ACTIVE:
        recorder, path, installed = _ACTIVE.pop()
        installed.__exit__(None, None, None)
        document = recorder.document()
        size = write_document(Path(path), document)
        print(f"\nEFFECTIVE_ALPHA_RECORDED pairs={len(document['pairs'])} calls={recorder.calls} bytes={size}")


def synthetic_document(scale: int, seed: int | None = None, only: tuple = ()) -> dict:
    """The generated adversarial pairs (`native_effective_alpha_synthetic`), recorded with the oracle's answers: a document like any other corpus file."""

    import native_effective_alpha_synthetic as synthetic

    recorder = Recorder("synthetic")
    with recorder.installed():
        synthetic.generate(scale, synthetic.DEFAULT_SEED if seed is None else seed, only)
    return recorder.document()


def command_synthetic(out: Path, cases: int) -> int:
    document = synthetic_document(cases)
    size = write_document(out / "synthetic.recs.xz", document)
    shape = shape_of(document)
    print(f"EFFECTIVE_ALPHA_SYNTHETIC pairs={len(document['pairs'])} bytes={size} {json.dumps(shape)}")
    write_index(out, {"synthetic": {**shape, "bytes": size, "cases": cases}})
    return 0


def command_seed(source: Path, target: Path, limit: int) -> int:
    document = seed_of(read_document(source), limit)
    size = write_document(target, document)
    print(f"EFFECTIVE_ALPHA_SEED pairs={len(document['pairs'])} bytes={size} {json.dumps(shape_of(document))}")
    return 0


def command_summary(directory: Path) -> int:
    for path in corpus_files(directory):
        document = read_document(path)
        print(f"{path.relative_to(directory)}: {path.stat().st_size} bytes {json.dumps(shape_of(document))}")
    return 0


def outcomes_digest(document: dict, side: str) -> str:
    """sha256 of the outcomes of every pair of `document` on `side` (`recorded`, `python`, `native`), in the order of the file: a number that two processes can compare."""

    objects = Objects(document)
    rows = []
    for pair in document["pairs"]:
        outcome = recorded_outcome(pair) if side == "recorded" else run_pair(objects, pair, side)
        rows.append([pair["id"], outcome["answer"], outcome["counts"], outcome["symbolic"]])
    return hashlib.sha256(json.dumps(rows, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def replay(document: dict, limit: int | None = None) -> dict:
    """Every pair of the document on three sides (recorded, oracle now, extension), without pytest: the tallies and the first mismatches (for an interpreter that has no pytest: Blender's)."""

    objects = Objects(document)
    tally = {"pairs": 0, "equal": 0, "mismatch": 0, "answered": 0, "declined": 0, "no frame": 0}
    first: list = []
    for pair in document["pairs"][:limit]:
        recorded, python, native = recorded_outcome(pair), run_pair(objects, pair, "python"), run_pair(objects, pair, "native")
        tally["pairs"] += 1
        if equal_outcomes(recorded, python) and equal_outcomes(python, native):
            tally["equal"] += 1
        else:
            tally["mismatch"] += 1
            if len(first) < 3:
                first.append({"id": pair["id"], "mode": pair["mode"], "recorded": recorded, "python": python, "native": native})
        ledger = native.get("ledger", {"native": 0, "python": 0})
        tally["answered"] += ledger["native"]
        tally["declined"] += ledger["python"]
        tally["no frame"] += not (ledger["native"] or ledger["python"])
    return {**tally, "first_mismatches": first}


def command_replay(paths, limit: int | None) -> int:
    failed = False
    for path in paths:
        result = replay(read_document(Path(path)), limit)
        print(f"EFFECTIVE_ALPHA_REPLAY {Path(path).name} python={sys.version.split()[0]} " + json.dumps({key: value for key, value in result.items() if key != "first_mismatches"}))
        if result["mismatch"]:
            failed = True
            print(json.dumps(result["first_mismatches"], default=str)[:4000])
    return 1 if failed else 0


def command_digest(scale: float, only: tuple) -> int:
    """For the sweep of `PYTHONHASHSEED`: the generated pairs with the oracle's recorded answers and the digests of the three sides; one JSON line."""

    document = synthetic_document(scale, only=only)
    print(json.dumps({"pairs": len(document["pairs"]), **{side: outcomes_digest(document, side) for side in ("recorded", "python", "native")}}))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("kernel-suite", "synthetic", "seed", "summary", "digest", "replay"))
    parser.add_argument("--out", default="")
    parser.add_argument("--file", action="append", default=[], help="a corpus file for `replay`")
    parser.add_argument("--max-pairs", type=int, default=None)
    parser.add_argument("--dir", default="")
    parser.add_argument("--from", dest="source", default="")
    parser.add_argument("--cases", type=float, default=1.0, help="the scale of the generated scenarios (1 is the full set)")
    parser.add_argument("--only", default="", help="comma list of scenario names")
    parser.add_argument("--limit", type=int, default=1500)
    parser.add_argument("pytest_arguments", nargs="*")
    arguments = parser.parse_args(argv)
    if arguments.command == "seed":
        return command_seed(Path(arguments.source), Path(arguments.out), arguments.limit)
    if arguments.command == "digest":
        return command_digest(arguments.cases, tuple(item for item in arguments.only.split(",") if item))
    if arguments.command == "replay":
        return command_replay(arguments.file, arguments.max_pairs)
    out = Path(arguments.out) if arguments.out else corpus_directory()
    if arguments.command == "summary":
        return command_summary(Path(arguments.dir) if arguments.dir else out)
    out.mkdir(parents=True, exist_ok=True)
    return record_kernel_suite(out, arguments.pytest_arguments) if arguments.command == "kernel-suite" else command_synthetic(out, arguments.cases)


if __name__ == "__main__":
    raise SystemExit(main())
