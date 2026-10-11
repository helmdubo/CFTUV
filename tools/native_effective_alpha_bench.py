"""Timing of the pair step of the effective-alpha resolver, Python oracle against the native operation, call by call.

    python tools/native_effective_alpha_bench.py [FILE.recs.xz ...] [--reps 3] [--json OUT]

A file is a recorded corpus (default: the field files of `E:/cftuv_native_corpus/effective_alpha/field`). Every pair is asked of `boundary._contact_candidates` as the resolver asks it: the frames of a
source are shared by its pairs in record order, the first pair of a source pays for its frame (on the native side: the conversion of the frame as well), the stage is ordered NATIVE for the native
side and the oracle is called with no ledger for the other. What is timed is the whole call, "incl. conversion": the dispatcher, the shim, the crossing, the contacts built as the oracle's own
objects. Each pair is timed `--reps` times, alternating the sides, and the SMALLEST time of the repetitions stands for the pair (a pair has no noise of its own: the noise is the machine); the report is
then over pairs: p50, p95, p99 and max of the call, the mean and the total, for all pairs and for the two kinds the prefilter makes (rejected without a leaf, and passed to the leaf), and the ratios.
The garbage collector is off while timing (the button pauses it too).
"""

from __future__ import annotations

import argparse
import gc
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import native_effective_alpha_corpus as nec  # noqa: E402

from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.reference import symbolic_backend as symbolic  # noqa: E402


def percentile(values: list, fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def timings(document: dict, reps: int) -> dict:
    """`{side: [seconds per pair, ...]}` in the order of the file, the minimum over `reps`."""

    objects = nec.Objects(document)
    pairs = document["pairs"]
    best = {"python": [float("inf")] * len(pairs), "native": [float("inf")] * len(pairs)}
    clock = time.perf_counter_ns
    for repetition in range(reps):
        for side in (("python", "native") if repetition % 2 == 0 else ("native", "python")):
            objects.frames.clear()  # a new session of the resolver: the first pair of a source pays for its frame
            native = side == "native"
            for index, pair in enumerate(pairs):
                context, source, boundary, frame = objects.call(pair, side)
                mode = symbolic.SymbolicBackendV1(pair["mode"])
                with symbolic.symbolic_backend(mode, symbolic.DisagreementPolicyV1.RECORD):
                    if native:
                        with backend.use_backend(*nec.NATIVE_ORDER):
                            started = clock()
                            nec.ORACLE(context, source, boundary, frame)
                            elapsed = clock() - started
                    else:
                        started = clock()
                        nec.ORACLE(context, source, boundary, frame)
                        elapsed = clock() - started
                if elapsed < best[side][index]:
                    best[side][index] = elapsed
    return {side: [item / 1e9 for item in values] for side, values in best.items()}


def summary(values: list) -> dict:
    return {
        "n": len(values), "p50_us": round(percentile(values, 0.50) * 1e6, 2), "p95_us": round(percentile(values, 0.95) * 1e6, 2), "p99_us": round(percentile(values, 0.99) * 1e6, 2),
        "max_us": round(max(values) * 1e6, 2), "mean_us": round(sum(values) / len(values) * 1e6, 2), "total_s": round(sum(values), 4),
    }


def report(document: dict, reps: int) -> dict:
    measured = timings(document, reps)
    kinds = {"all": range(len(document["pairs"]))}
    rejected = [index for index, pair in enumerate(document["pairs"]) if "contact_candidates.prefilter_rejected" in pair["counts"]]
    passed = [index for index, pair in enumerate(document["pairs"]) if "contact_candidates.prefilter_passed" in pair["counts"]]
    kinds.update({"rejected by the prefilter": rejected, "passed to the leaf": passed})
    rows = {}
    for kind, indexes in kinds.items():
        indexes = list(indexes)
        if not indexes:
            continue
        python, native = (summary([measured[side][index] for index in indexes]) for side in ("python", "native"))
        rows[kind] = {
            "python": python, "native": native,
            "ratio": {name: round(python[name] / native[name], 1) for name in ("p50_us", "p95_us", "p99_us", "mean_us", "total_s")},
        }
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="*")
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--json", default="")
    arguments = parser.parse_args(argv)
    paths = [Path(item) for item in arguments.files] or nec.corpus_files(nec.corpus_directory(), "field")
    gc.collect()
    gc.disable()
    result = {}
    try:
        for path in paths:
            document = nec.read_document(path)
            result[path.name.removesuffix(".recs.xz")] = report(document, arguments.reps)
    finally:
        gc.enable()
    for name, rows in result.items():
        print(f"== {name}")
        for kind, row in rows.items():
            print(f"  {kind}: python {row['python']}")
            print(f"  {' ' * len(kind)}  native {row['native']}")
            print(f"  {' ' * len(kind)}  python/native {row['ratio']}")
    if arguments.json:
        Path(arguments.json).write_text(json.dumps(result, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
