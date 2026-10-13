"""Offline raw facts на реальной pre-evaluation границе; Blender не запускается."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from dataclasses import fields, is_dataclass
from fractions import Fraction
import hashlib
import importlib.util
import json
import lzma
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
sys.set_int_max_str_digits(0)

import cftuv_envelope as kernel
from cftuv_envelope.codec import _decode_as, canonical_json_bytes
from cftuv_envelope.reference import compile as compiler
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.density_raw_v1 import extract_raw_density_spec_input
from cftuv_envelope.validation import ValidationIssue


def packed(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")


def raw_wire(value):
    if isinstance(value, Fraction):
        return [value.numerator, value.denominator]
    if is_dataclass(value):
        return {item.name: raw_wire(getattr(value, item.name)) for item in fields(value)}
    if type(value) is tuple:
        return [raw_wire(child) for child in value]
    if value is None or type(value) in (str, int, bool):
        return value
    raise TypeError("RAW_AUDIT_UNSUPPORTED_CARRIER:" + type(value).__name__)


def sources(corpus_root, manifest):
    # Принимаются только уже выбранные однозначные старые field runs.
    versions = {}
    for pair in manifest["pairs"]:
        version = pair["version"]
        if version not in versions:
            name = "audit_compile_field.py" if version == 2 else f"audit_compile_field_v{version}.py"
            spec = importlib.util.spec_from_file_location(f"raw_audit_v{version}", corpus_root / name)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            allowed, _, _ = module.accepted_runs()
            groups, blobs, failures = module.load("base", allowed["base"])
            if failures:
                raise ValueError("RAW_AUDIT_SOURCE_RECORDER_FAILURE")
            versions[version] = (groups, blobs)
        groups, blobs = versions[version]
        run = f"{pair['case']}/w{pair['workers']}"
        selected = [(key, rows) for key, rows in sorted(groups.items()) if key[0] == run]
        if len(selected) != pair["counts"]["input_keys"] or sum(len(rows) for _, rows in selected) != pair["counts"]["base_calls"]:
            raise ValueError("RAW_AUDIT_SOURCE_COUNTS_CHANGED")
        for (_, digest), rows in selected:
            for row in rows:
                yield {"run": run, "version": version, "input": digest, "path": blobs[digest],
                       "historical_call": row}


def decode_input(path, digest):
    payload = lzma.decompress(path.read_bytes())
    if hashlib.sha256(payload).hexdigest() != digest:
        raise ValueError("RAW_AUDIT_SOURCE_SHA256_MISMATCH")
    values = json.loads(payload)
    if type(values) is not list or len(values) != 4:
        raise ValueError("RAW_AUDIT_SOURCE_TUPLE_INVALID")
    return (_decode_as(values[0], kernel.AnalysisSnapshotV1),
            _decode_as(values[1], kernel.DecalRequestV1),
            _decode_as(values[2], kernel.PatchDomainId | None),
            _decode_as(values[3], tuple[ValidationIssue, ...] | None))


@contextmanager
def capture_raw(rows, failures):
    descriptor = GeometryContext.__dict__["build"]
    evaluate = compiler._evaluation_density_spec
    attach = compiler._attach_direction_bindings
    contexts = []
    active = False

    def build(cls, compilation, frame, **kwargs):
        result = descriptor.__func__(cls, compilation, frame, **kwargs)
        if active and kwargs.get("require_evaluation_binding") is False:
            contexts.append(result)
        return result

    def attached(*args, **kwargs):
        nonlocal active
        if active:
            raise RuntimeError("RAW_AUDIT_NESTED_ATTACHMENT")
        contexts.clear()
        active = True
        try:
            return attach(*args, **kwargs)
        finally:
            active = False

    def evaluated(context, spec, selection, q):
        try:
            matches = [item for item in contexts if item.compilation.evaluation_geometry_binding is None
                       and item.compilation.plan_key == context.compilation.plan_key]
            if len(matches) != 1:
                raise ValueError("RAW_AUDIT_SOURCE_CONTEXT_NOT_UNIQUE")
            raw = extract_raw_density_spec_input(matches[0], context, spec, selection)
            if raw.selection.q != q:
                raise ValueError("RAW_AUDIT_SELECTION_Q_MISMATCH")
            rows.append(raw_wire(raw))
        except Exception as error:
            failures.append({"spec": spec.envelope_spec_id.value, "type": type(error).__name__, "detail": str(error)})
        # Наблюдение не заменяет ответ; исходный контроллер вызывается ровно один раз.
        return evaluate(context, spec, selection, q)

    GeometryContext.build = classmethod(build)
    compiler._attach_direction_bindings = attached
    compiler._evaluation_density_spec = evaluated
    try:
        yield
    finally:
        GeometryContext.build = descriptor
        compiler._attach_direction_bindings = attach
        compiler._evaluation_density_spec = evaluate


def source_hashes():
    paths = sorted((ROOT / "kernel/src").rglob("*.py"))
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(args.out)
    args.out.mkdir(parents=True)
    manifest = json.loads((args.corpus_root / "a1-compile-combined.json").read_text())
    before, counts, started = source_hashes(), Counter(), time.perf_counter()
    instrumentation_failures, compile_errors = [], []
    with (args.out / "records.jsonl").open("wb") as stream:
        for index, source in enumerate(sources(args.corpus_root, manifest)):
            if args.limit is not None and index >= args.limit:
                break
            snapshot, request, domain, issues = decode_input(source["path"], source["input"])
            raw, errors = [], []
            with capture_raw(raw, errors):
                try:
                    result = kernel.compile_reference_envelopes(snapshot, request, domain, snapshot_issues=issues)
                    outcome = {"type": "returned", "outcome": result.outcome.value,
                               "sha256": hashlib.sha256(canonical_json_bytes(result)).hexdigest()}
                except Exception as error:
                    outcome = {"type": "raised", "exception": type(error).__name__, "detail": str(error)}
                    compile_errors.append({"run": source["run"], "input": source["input"], **outcome})
            counts["compile_calls"] += 1
            counts["raw_specs"] += len(raw)
            for item in raw:
                counts[f"q{item['selection']['q']}"] += 1
                counts[f"H{item['selection']['source_hidden_count']}"] += 1
                for geometry in ("source_geometry", "evaluation_geometry"):
                    for incident in ("incoming", "outgoing"):
                        if item[geometry][incident]["owner_face_cycles"]:
                            counts["incident_with_face_cycles"] += 1
                if item["canonical"]["restoration_declaration"]:
                    counts["canonical_restoration"] += 1
            instrumentation_failures.extend({"run": source["run"], "input": source["input"], **error} for error in errors)
            stream.write(packed({"run": source["run"], "input": source["input"], "version": source["version"],
                                 "raw": raw, "instrumentation_errors": errors, "outcome": outcome}) + b"\n")
            stream.flush()
            if index % 100 == 0:
                print(json.dumps({"counts": dict(counts), "instrumentation_failures": len(instrumentation_failures)}), flush=True)
    after = source_hashes()
    changed = sorted(key for key in set(before) | set(after) if before.get(key) != after.get(key))
    receipt = {"scope": "Fresh-memo offline replay of selected saved full compilation inputs; raw facts captured before actual evaluation density spec. No historical budget equality claim.",
               "pid": os.getpid(), "seconds": time.perf_counter() - started, "counts": dict(counts),
               "instrumentation_failures": instrumentation_failures, "compile_exceptions": compile_errors,
               "source_hashes": before, "source_changed": changed, "partial": args.limit is not None}
    (args.out / "receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({"counts": dict(counts), "instrumentation_failures": len(instrumentation_failures), "source_changed": changed}), flush=True)
    return bool(instrumentation_failures)


if __name__ == "__main__":
    raise SystemExit(main())
