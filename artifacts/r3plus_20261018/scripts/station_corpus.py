"""S2 corpus: snapshots of the EXPORT corpus (tasks.xz, task_inputs under PYTHON) and the CHAIN_STATION_PLAN_V1 oracle answer per (snapshot, domain).

python station_corpus.py build <tasks dir> <out.pkl.xz> [--name PART]   # pickle stream (case, domain_id value, snapshot codec bytes, plans pickle)
python station_corpus.py time <out.pkl.xz> [--repeat 3]                 # oracle cost split: StationFacts vs the rest, per call
"""
import argparse
import contextlib
import dataclasses
import io
import lzma
import pickle
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def _setup(native_site):
    if native_site:
        sys.path.insert(0, native_site)
    sys.path.insert(0, str(ROOT / "tools"))
    import native_snapshot_check_corpus as corpus

    corpus.install_host_stubs()
    return corpus


def build(args):
    corpus = _setup(args.native_site)
    import cftuv_envelope as kernel
    from cftuv.envelope_export_input import TaskInputsV1, task_inputs
    from cftuv.envelope_kernel_backend import entered_backend
    from cftuv_envelope import _chain_station as law

    count = 0
    with lzma.open(args.output, "wb", preset=3) as handle:
        for path in corpus.corpus_files(args.directory, args.name):
            for case, task in corpus.read_tasks(path):
                with entered_backend("PYTHON"), contextlib.redirect_stdout(io.StringIO()):
                    inputs = task_inputs(dataclasses.replace(task, cold=None))
                if not isinstance(inputs, TaskInputsV1):
                    continue
                snapshot = inputs.snapshot
                (domain,) = [item.patch_domain_id for item in snapshot.patch_domains if item.patch_domain_id.value == task.domain_id]
                plans = law.chain_station_plans(snapshot, domain)
                blob = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
                pickle.dump((case, domain, blob, pickle.dumps(plans, protocol=5)), handle, protocol=5)
                count += 1
    print("wrote", count, args.output)


def read(path):
    with lzma.open(path, "rb") as handle:
        while True:
            try:
                yield pickle.load(handle)
            except EOFError:
                return


def load(path):
    import cftuv_envelope as kernel

    rows = []
    for case, domain, blob, plans in read(path):
        rows.append((case, domain, kernel.AnalysisSnapshotCodecV1.loads(blob), pickle.loads(plans)))
    return rows


def timing(args):
    _setup(args.native_site)
    from cftuv_envelope import _chain_station as law

    rows = load(args.corpus)
    bad = sum(law.chain_station_plans(s, d) != p for _c, d, s, p in rows)
    print("rows", len(rows), "oracle mismatches", bad)
    for _ in range(args.repeat):
        fresh = [(pickle.loads(pickle.dumps(s, protocol=5)), d) for _c, d, s, _p in rows]
        t0 = time.perf_counter()
        facts = [law.StationFacts(s) for s, _d in fresh]
        t1 = time.perf_counter()
        for (s, d), f in zip(fresh, facts):
            law.chain_station_plans(s, d, f)
        t2 = time.perf_counter()
        print(f"facts {1e3 * (t1 - t0) / len(rows):.3f} ms/call  rest {1e3 * (t2 - t1) / len(rows):.3f} ms/call  total {(t2 - t0):.3f} s")


def seed(args):
    """The tracked seed: per case the smallest snapshots, and per (case, station kind, reason) the smallest that shows it; `(case, domain value, codec bytes)` under lzma."""

    _setup(args.native_site)
    chosen: dict = {}
    rows = sorted(read(args.corpus), key=lambda row: len(row[2]))
    smallest: dict = {}
    shown: dict = {}
    for case, domain, blob, plans in rows:
        key = (case, domain.value)
        if smallest.get(case, 0) < args.per_case:
            smallest[case] = smallest.get(case, 0) + 1
            chosen[key] = blob
        for plan in pickle.loads(plans):
            for station in plan.stations:
                label = (case, station.kind.name, station.reason.name)
                if shown.get(label, 0) < args.per_reason:
                    shown[label] = shown.get(label, 0) + 1
                    chosen[key] = blob
    with lzma.open(args.output, "wb", preset=9) as handle:
        for (case, value), blob in sorted(chosen.items()):
            pickle.dump((case, value, blob), handle, protocol=5)
    print("seed rows", len(chosen), "bytes", Path(args.output).stat().st_size)


def approve_in_memory():
    """TEST-ONLY: the pin candidate installed in this process (no file changes, the build id stays); a wheel with the approved table
    (CHAIN_STATION_PINS_APPROVED_V1) has no candidate and needs nothing."""

    from cftuv_native import chain_station_op

    if hasattr(chain_station_op, "CANDIDATE"):
        chain_station_op.APPROVED_PINS = dict(chain_station_op.CANDIDATE["files"])
    chain_station_op.refresh()


def _ordered(plans):
    """The plans' bytes one plan at a time (exact types and values; object sharing between plans, which pickle memoizes, is not part of the answer)."""

    return [pickle.dumps(plan, protocol=5) for plan in sorted(plans, key=lambda plan: plan.physical_chain_id.value)]


def parity(args):
    _setup(args.native_site)
    import cftuv_native
    from cftuv_envelope import _chain_station as law

    approve_in_memory()
    rows = load(args.corpus)
    equal = declined = 0
    differences = []
    for case, domain, snapshot, plans in rows:
        oracle = law.chain_station_plans(snapshot, domain)
        assert oracle == plans
        try:
            native = cftuv_native.chain_station_plans(snapshot, domain)
        except cftuv_native.NativePortUnsupported as error:
            declined += 1
            differences.append((case, domain.value, f"declined: {error}"))
            continue
        if native == oracle and _ordered(native) == _ordered(oracle):
            equal += 1
        else:
            differences.append((case, domain.value, "differs"))
    print("rows", len(rows), "equal", equal, "declined", declined, "differences", len(differences) - declined)
    for row in differences[:10]:
        print(" ", row)
    fresh = [(pickle.loads(pickle.dumps(s, protocol=5)), d) for _c, d, s, _p in rows]
    for _ in range(args.repeat):
        t0 = time.perf_counter()
        for s, d in fresh:
            law.chain_station_plans(s, d)
        t1 = time.perf_counter()
        for s, d in fresh:
            cftuv_native.chain_station_plans(s, d)
        t2 = time.perf_counter()
        print(f"python {1e3 * (t1 - t0) / len(rows):.4f} ms/call  native {1e3 * (t2 - t1) / len(rows):.4f} ms/call  speedup {(t1 - t0) / (t2 - t1):.1f}x")
    return 0 if len(differences) == declined else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-site", default="")
    sub = parser.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("directory")
    b.add_argument("output")
    b.add_argument("--name", default="")
    t = sub.add_parser("time")
    t.add_argument("corpus")
    t.add_argument("--repeat", type=int, default=3)
    p = sub.add_parser("parity")
    p.add_argument("corpus")
    p.add_argument("--repeat", type=int, default=3)
    s = sub.add_parser("seed")
    s.add_argument("corpus")
    s.add_argument("output")
    s.add_argument("--per-case", type=int, default=2)
    s.add_argument("--per-reason", type=int, default=1)
    args = parser.parse_args()
    return {"build": build, "time": timing, "parity": parity, "seed": seed}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
