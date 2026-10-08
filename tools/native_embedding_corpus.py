"""Corpus of calls of `_embedding._compute_source_snap_embedding_certificate` for the native `snap_embedding` port: record, store, read, replay, compare.

The unit is the pure leaf the memo wrapper `build_source_snap_embedding_certificate` calls at three places: positions before and after the snap, the faces of the
patch, the intended and the unclassifiable corners, the snapping law -> `SourceSnapEmbeddingCertificateV1` or a `ValueError`. It has no state and no cost accounting, so a
record is only (input, answer-or-exception); equality is the twelve fields and the exception type and text.

A record keeps ONLY what the leaf reads (`face_id.value`, the vertex and edge ids of the cycles, the numbers as `int` or `(numerator, denominator)`), so a record is some KB:

    {"id", "source", "count", "law", "before": [(vertex, (x, y, z)), ...], "after": None | [...], "faces": [(face, (vertices), (edges)), ...],
     "intended": [(a, b, c), ...], "unclassifiable": [...], "answer": ("ok", (law, ids, ints...)) | ("raised", class, text)}

`after` is `None` when the call passed the very same dict (`UNSNAPPED_EXACT_V1`); the replay then passes `before` twice. Files: `<name>.recs.xz` = xz(JSON(list of records)); the corpus
directory (`E:/cftuv_native_corpus/embedding`, `CFTUV_NATIVE_CORPUS` moves the base) holds `field/<mesh>.recs.xz`, `kernel_suite.recs.xz`, `synthetic.recs.xz` and `index.json`.

    python tools/native_embedding_corpus.py kernel-suite [--out DIR]     records the calls the kernel tests make (pytest in this process)
    python tools/native_embedding_corpus.py synthetic [--out DIR] [--cases N]   writes the Blender-free generated cases with the oracle's answers
    python tools/native_embedding_corpus.py summary [--dir DIR]          what is in the corpus
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
KERNEL_SOURCE = ROOT / "kernel" / "src"
for _path in (KERNEL_SOURCE, ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import cftuv_envelope._embedding as embedding  # noqa: E402
from cftuv_envelope.contracts.metric import GridSnappingLawV1, SourceSnapEmbeddingCertificateV1  # noqa: E402
from cftuv_envelope.contracts.surface import SourceFaceV1  # noqa: E402
from cftuv_envelope.ids import PatchId, PhysicalEdgeId, SourceFaceId, SourceVertexId  # noqa: E402
from cftuv_envelope.numeric import LocalVector3V1  # noqa: E402

#: The oracle function of this operation, taken at import: replacing the module attribute (the recorder does) does not change it.
ORACLE = embedding._compute_source_snap_embedding_certificate
SCHEMA = "cftuv.native-embedding-corpus.v1"
DEFAULT_BASE = "E:/cftuv_native_corpus"
CORPUS_ENVIRONMENT = "CFTUV_NATIVE_CORPUS"
CERTIFICATE_FIELDS = (
    "snapping_law", "source_vertex_ids", "source_vertex_count", "source_edge_count", "intended_right_corner_count", "newly_coincident_vertex_pair_count",
    "collapsed_nonzero_source_edge_count", "new_nonadjacent_edge_intersection_count", "unclassifiable_source_corner_count",
    "unchanged_unclassifiable_source_corner_count", "degenerated_intended_right_corner_count", "exact_pair_test_count",
)
#: Counters whose non-zero values a corpus should show (the four violation counters and the corner counters).
VIOLATION_FIELDS = (
    "newly_coincident_vertex_pair_count", "collapsed_nonzero_source_edge_count", "new_nonadjacent_edge_intersection_count",
    "unchanged_unclassifiable_source_corner_count", "degenerated_intended_right_corner_count",
)


class CorpusError(RuntimeError):
    """The corpus is malformed or missing: a named failure, not a quiet "almost"."""


class FaceLike:
    """A face the leaf can read but `SourceFaceV1` would refuse (fewer than three vertices, cycles of unequal length): the leaf reads three attributes."""

    __slots__ = ("face_id", "vertex_cycle", "edge_cycle")

    def __init__(self, face_id, vertex_cycle, edge_cycle):
        self.face_id, self.vertex_cycle, self.edge_cycle = face_id, vertex_cycle, edge_cycle

    def __repr__(self):
        return f"FaceLike({self.face_id.value!r})"


# --------------------------------------------------------------------------
# the record <-> the call
# --------------------------------------------------------------------------


def encode_number(value):
    """`int` stays an `int`, a `Fraction` is `(numerator, denominator)`: the type of a coordinate is part of the input."""

    if type(value) is int:
        return value
    if type(value) is Fraction:
        return (value.numerator, value.denominator)
    raise CorpusError(f"a coordinate is an int or a Fraction, not {type(value).__name__}")


def decode_number(value):
    return Fraction(value[0], value[1]) if isinstance(value, (tuple, list)) else value


def encode_positions(positions) -> list:
    return [(vertex.value, tuple(encode_number(item) for item in point)) for vertex, point in positions.items()]


def decode_positions(rows) -> dict:
    return {SourceVertexId(name): tuple(decode_number(item) for item in point) for name, point in rows}


def encode_face(face) -> tuple:
    return (face.face_id.value, tuple(item.value for item in face.vertex_cycle), tuple(item.value for item in face.edge_cycle))


def decode_face(row):
    key, vertices, edges = row
    face_id, cycle, edge_cycle = SourceFaceId(key), tuple(SourceVertexId(name) for name in vertices), tuple(PhysicalEdgeId(name) for name in edges)
    if len(cycle) < 3 or len(cycle) != len(edge_cycle):
        return FaceLike(face_id, cycle, edge_cycle)
    return SourceFaceV1(face_id, PatchId("patch"), cycle, edge_cycle, LocalVector3V1(0.0, 0.0, 1.0), ())


def encode_call(before, after, faces, intended, unclassifiable, law) -> dict:
    """The input of one call as a record body (without `id`, `source`, `count`, `answer`)."""

    return {
        "law": law.value,
        "before": encode_positions(before),
        "after": None if after is before else encode_positions(after),
        "faces": [encode_face(face) for face in faces],
        "intended": [tuple(item.value for item in corner) for corner in intended],
        "unclassifiable": [tuple(item.value for item in corner) for corner in unclassifiable],
    }


def decode_call(record: dict) -> tuple:
    """`(before, after, faces, intended_corners, unclassifiable_corners, snapping_law)`: the arguments of the leaf, fresh objects (`after is before` when recorded so)."""

    before = decode_positions(record["before"])
    after = before if record["after"] is None else decode_positions(record["after"])
    corner = lambda row: tuple(SourceVertexId(name) for name in row)  # noqa: E731
    return (
        before, after, tuple(decode_face(row) for row in record["faces"]), tuple(corner(row) for row in record["intended"]),
        tuple(corner(row) for row in record["unclassifiable"]), GridSnappingLawV1(record["law"]),
    )


def input_id(body: dict) -> str:
    text = json.dumps((body["law"], body["before"], body["after"], body["faces"], body["intended"], body["unclassifiable"]), separators=(",", ":"))
    return hashlib.sha256(text.encode("ascii")).hexdigest()[:20]


def answer_of(record: dict) -> tuple:
    """The recorded answer with its lists turned back into tuples (the JSON file keeps no tuples): comparable with `outcome_of`."""

    def freeze(value):
        return tuple(freeze(item) for item in value) if isinstance(value, (list, tuple)) else value

    return freeze(record["answer"])


def certificate_fields(certificate) -> tuple:
    """The twelve fields as plain values: the law's value, the ids' values, the ints (a `bool` or any other type would show)."""

    return tuple(
        value.value if name == "snapping_law" else tuple(item.value for item in value) if name == "source_vertex_ids" else value
        for name, value in ((name, getattr(certificate, name)) for name in CERTIFICATE_FIELDS)
    )


def outcome_of(function, arguments) -> tuple:
    """`("ok", fields)` or `("raised", class name, text)`: what the function does with the arguments."""

    try:
        certificate = function(*arguments)
    except Exception as error:  # noqa: BLE001 - the exception IS the recorded answer
        return ("raised", type(error).__name__, str(error))
    if not isinstance(certificate, SourceSnapEmbeddingCertificateV1):
        return ("wrong-type", type(certificate).__name__, "")
    return ("ok", certificate_fields(certificate))


def is_violating(answer: tuple) -> bool:
    return answer[0] == "ok" and any(answer[1][CERTIFICATE_FIELDS.index(name)] for name in VIOLATION_FIELDS[:3])


def record_of(arguments, source: str, answer: tuple | None = None) -> dict:
    body = encode_call(*arguments)
    body.update(id=input_id(body), source=source, count=1, answer=outcome_of(ORACLE, arguments) if answer is None else answer)
    return body


# --------------------------------------------------------------------------
# the recorder
# --------------------------------------------------------------------------


class Recorder:
    """Wraps the oracle leaf of `_embedding`: every call is computed (the memo is off), answered as the oracle answers, and recorded once per distinct input."""

    def __init__(self, source: str, limit: int | None = None):
        self.source, self.limit = source, limit
        self.records: dict = {}
        self.calls = 0

    def wrapper(self):
        recorder, real = self, ORACLE

        def recorded(before, after, faces, intended, unclassifiable, law):
            recorder.calls += 1
            try:
                certificate = real(before, after, faces, intended, unclassifiable, law)
            except Exception as error:  # noqa: BLE001
                recorder._note((before, after, faces, intended, unclassifiable, law), ("raised", type(error).__name__, str(error)))
                raise
            recorder._note((before, after, faces, intended, unclassifiable, law), ("ok", certificate_fields(certificate)))
            return certificate

        return recorded

    def _note(self, arguments, answer) -> None:
        body = encode_call(*arguments)
        key = input_id(body)
        found = self.records.get(key)
        if found is not None:
            found["count"] += 1
        elif self.limit is None or len(self.records) < self.limit:
            body.update(id=key, source=self.source, count=1, answer=answer)
            self.records[key] = body

    @contextlib.contextmanager
    def installed(self):
        """The memo is off and `_embedding._compute_...` is the recording wrapper for the block."""

        previous = embedding._compute_source_snap_embedding_certificate
        embedding._compute_source_snap_embedding_certificate = self.wrapper()
        try:
            with embedding.embedding_memo_limit(0):
                yield self
        finally:
            embedding._compute_source_snap_embedding_certificate = previous


# --------------------------------------------------------------------------
# files
# --------------------------------------------------------------------------


def corpus_directory(base: str | Path | None = None) -> Path:
    return Path(base or os.environ.get(CORPUS_ENVIRONMENT) or DEFAULT_BASE) / "embedding"


def write_records(path: Path, records) -> int:
    """xz(JSON(list of records)); the size in bytes."""

    path.parent.mkdir(parents=True, exist_ok=True)
    data = lzma.compress(json.dumps(list(records), separators=(",", ":")).encode("ascii"), preset=6)
    path.write_bytes(data)
    return len(data)


def read_records(path: Path) -> list:
    try:
        return json.loads(lzma.decompress(path.read_bytes()).decode("ascii"))
    except (OSError, lzma.LZMAError, ValueError, EOFError) as error:
        raise CorpusError(f"cannot read the embedding corpus file {path}: {error}") from error


def read_corpus(directory: Path | None = None, kind: str | None = None) -> list:
    """Every record of the corpus (or of `kind`: `field`, `kernel_suite`, `synthetic`)."""

    directory = directory or corpus_directory()
    paths = sorted(directory.rglob("*.recs.xz"))
    if kind is not None:
        paths = [path for path in paths if (path.parent.name == kind or path.name == f"{kind}.recs.xz")]
    return [record for path in paths for record in read_records(path)]


def shape_of(records) -> dict:
    """Counts for the index and the report: records, calls, exceptions, non-zero counters, size of the biggest record's patch."""

    summary = {"records": len(records), "calls": sum(item["count"] for item in records), "raised": 0, "after_is_before": 0, "violating": 0, "max_vertices": 0, "max_edges": 0}
    summary.update({name: 0 for name in VIOLATION_FIELDS})
    for item in records:
        answer = item["answer"]
        summary["raised"] += answer[0] == "raised"
        summary["after_is_before"] += item["after"] is None
        summary["violating"] += is_violating(answer)
        summary["max_vertices"] = max(summary["max_vertices"], len(item["before"]))
        if answer[0] == "ok":
            summary["max_edges"] = max(summary["max_edges"], answer[1][CERTIFICATE_FIELDS.index("source_edge_count")])
            for name in VIOLATION_FIELDS:
                summary[name] += bool(answer[1][CERTIFICATE_FIELDS.index(name)])
    return summary


def oracle_digest() -> str:
    """sha256 (line endings normalised) of `_embedding.py`: the file the port is pinned to."""

    return hashlib.sha256((KERNEL_SOURCE / "cftuv_envelope" / "_embedding.py").read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def write_index(directory: Path, extra: dict) -> None:
    """`index.json` of the corpus: what oracle file it was recorded against and what each source holds; `extra` is merged into the one already there."""

    path = directory / "index.json"
    index = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    index.update({"schema": SCHEMA, "oracle_file": "_embedding.py", "oracle_digest": oracle_digest(), "python": sys.version.split()[0], **extra})
    path.write_text(json.dumps(index, indent=1, sort_keys=True), encoding="utf-8")


def merge_into(path: Path, records) -> int:
    """Adds `records` to the file at `path` (dedupe by input id, counts add); returns the number of records now in the file."""

    merged: dict = {item["id"]: item for item in (read_records(path) if path.exists() else [])}
    for item in records:
        found = merged.get(item["id"])
        if found is None:
            merged[item["id"]] = item
        else:
            found["count"] += item["count"]
    write_records(path, merged.values())
    return len(merged)


# --------------------------------------------------------------------------
# command line
# --------------------------------------------------------------------------


def record_kernel_suite(out: Path, extra_arguments=()) -> int:
    """Runs the kernel tests that reach the leaf under the recorder (in this process) and writes `kernel_suite.recs.xz`."""

    import pytest

    tests = ROOT / "kernel" / "tests"
    if str(tests) not in sys.path:
        sys.path.insert(0, str(tests))
    selection = [str(tests / name) for name in (
        "test_embedding_certificates.py", "test_embedding_memo.py", "test_grid_wiring.py", "test_planar_metric_v2.py",
        "test_near_planar_policy.py", "test_near_planar_reduced_frame.py", "test_near_planar_snapshot_validation.py",
        "test_near_planar_surface_law.py", "test_near_planar_width_distortion.py",
    )]
    recorder = Recorder("kernel-suite")
    with recorder.installed():
        code = pytest.main(["-q", "-x", "--no-header", "-p", "no:cacheprovider", "--rootdir", str(ROOT / "kernel"), *selection, *extra_arguments])
    records = list(recorder.records.values())
    size = write_records(out / "kernel_suite.recs.xz", records)
    print(f"EMBEDDING_KERNEL_SUITE pytest_exit={int(code)} calls={recorder.calls} records={len(records)} bytes={size}")
    write_index(out, {"kernel_suite": {**shape_of(records), "bytes": size, "calls_recorded": recorder.calls}})
    return int(code)


def command_synthetic(out: Path, cases: int) -> int:
    import native_embedding_synthetic as synthetic

    records = [record_of(arguments, f"synthetic:{label}") for label, arguments in synthetic.cases(cases)]
    merged: dict = {}
    for item in records:
        found = merged.get(item["id"])
        if found is None:
            merged[item["id"]] = item
        else:
            found["count"] += 1
    size = write_records(out / "synthetic.recs.xz", merged.values())
    print(f"EMBEDDING_SYNTHETIC records={len(merged)} bytes={size} {json.dumps(shape_of(list(merged.values())))}")
    write_index(out, {"synthetic": {**shape_of(list(merged.values())), "bytes": size, "cases": cases}})
    return 0


def command_summary(directory: Path) -> int:
    for path in sorted(directory.rglob("*.recs.xz")):
        records = read_records(path)
        print(f"{path.relative_to(directory)}: {path.stat().st_size} bytes {json.dumps(shape_of(records))}")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("kernel-suite", "synthetic", "summary"))
    parser.add_argument("--out", default="")
    parser.add_argument("--dir", default="")
    parser.add_argument("--cases", type=int, default=2500)
    parser.add_argument("pytest_arguments", nargs="*")
    arguments = parser.parse_args(argv)
    out = Path(arguments.out) if arguments.out else corpus_directory()
    if arguments.command == "summary":
        return command_summary(Path(arguments.dir) if arguments.dir else out)
    out.mkdir(parents=True, exist_ok=True)
    return record_kernel_suite(out, arguments.pytest_arguments) if arguments.command == "kernel-suite" else command_synthetic(out, arguments.cases)


if __name__ == "__main__":
    raise SystemExit(main())
