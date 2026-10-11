"""Investigation driver (branch native/investigate-raw-crash only, never merged): runs the shard of the native differential tests many times and records the EXACT 32-bit exit code
of every child, so that a death by STATUS_SINGLE_STEP (0x80000004) is told apart from a plain failure.

    python hunt.py --py PY --files-from shard-files.txt --startups 60 --first-file 6 --full 1 --out DIR

Phases (each child is a direct child of THIS process, so Windows hands over the whole exit code, nothing is truncated by a shell):
  startup     `pytest <shard files> --collect-only`   (import of the extension, the pin check, collection: the first ~5 s of the failing run)
  first-file  `pytest <first file of the shard>`     (the first tests)
  full        the whole shard exactly as native.yml runs it
Every child runs with `-X faulthandler`: an exception of a Python process with the code 0x80000004 would print `Windows fatal exception: code 0x80000004` and the traceback.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def run(label: str, command: list[str], out: Path, index: int, timeout: int) -> int:
    log = out / f"{label}-{index:03d}.log"
    started = time.time()
    with open(log, "wb") as handle:
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT)
        try:
            code = child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            child.kill()
            code = -1
    took = time.time() - started
    code32 = code & 0xFFFFFFFF
    flag = "" if code == 0 else "   <<<<<<<<<< NONZERO"
    if code32 == 0x80000004:
        flag += " STATUS_SINGLE_STEP"
    print(f"HUNT {label} #{index:03d} exit {code} = 0x{code32:08X} in {took:.1f}s{flag}", flush=True)
    if code == 0:
        log.unlink()
    else:
        print(log.read_text(encoding="utf-8", errors="replace")[-6000:], flush=True)
    return code


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--py", required=True)
    parser.add_argument("--files-from", required=True)
    parser.add_argument("--startups", type=int, default=0)
    parser.add_argument("--first-file", type=int, default=0)
    parser.add_argument("--full", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    files = [line.strip() for line in Path(args.files_from).read_text(encoding="utf-8").splitlines() if line.strip()]
    base = [args.py, "-X", "faulthandler", "-m", "pytest"]
    common = ["-m", "not native_field", "-p", "no:cacheprovider", "-q", "-rsfE"]
    failures: list[tuple[str, int, int]] = []
    totals = {"startup": 0, "first-file": 0, "full": 0}
    for phase, count, command in (
        ("startup", args.startups, base + files + common + ["--collect-only"]),
        ("first-file", args.first_file, base + files[:1] + common),
        ("full", args.full, base + files + common + ["--durations=10"]),
    ):
        for index in range(count):
            totals[phase] += 1
            code = run(phase, command, out, index, timeout=3000)
            if code != 0:
                failures.append((phase, index, code & 0xFFFFFFFF))
    summary = f"HUNT SUMMARY runs {totals} failures {[(p, i, hex(c)) for p, i, c in failures]}"
    print(summary, flush=True)
    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        with open(step_summary, "a", encoding="utf-8") as handle:
            handle.write(f"### {os.environ.get('HUNT_LABEL', 'hunt')}\n\n`{summary}`\n\n")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
