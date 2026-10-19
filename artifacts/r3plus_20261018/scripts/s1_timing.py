"""R3+ S1 per-call timing: density_pin.runtime_stale before (a package directory with the fc817d2d files) and after (this tree), warm, alternating repeats, medians.

python s1_timing.py --before DIR --after DIR [--reps 7 --calls 500]   (DIR: a cftuv_native package directory; the kernel is this tree's kernel/src)
"""
import argparse
import importlib
import importlib.machinery
import statistics
import sys
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def load(name, directory):
    package = types.ModuleType(name)
    package.__path__ = [str(directory)]
    package.__spec__ = importlib.machinery.ModuleSpec(name, loader=None, is_package=True)
    sys.modules[name] = package
    return importlib.import_module(name + ".density_pin")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", required=True)
    parser.add_argument("--after", required=True)
    parser.add_argument("--reps", type=int, default=7)
    parser.add_argument("--calls", type=int, default=500)
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / "kernel/src"))
    sides = {"before": load("_s1_before", args.before), "after": load("_s1_after", args.after)}
    for side in sides.values():
        assert side.runtime_stale() == (), side.runtime_stale()
    samples = {name: [] for name in sides}
    for repeat in range(args.reps):
        for name in (("before", "after") if repeat % 2 == 0 else ("after", "before")):
            check = sides[name].runtime_stale
            start = time.perf_counter()
            for _ in range(args.calls):
                check()
            samples[name].append((time.perf_counter() - start) / args.calls * 1e3)
    print(f"python {sys.version.split()[0]}: " + ", ".join(f"{name} median {statistics.median(values):.4f} ms/call (min {min(values):.4f}, max {max(values):.4f})" for name, values in samples.items()))


if __name__ == "__main__":
    main()
