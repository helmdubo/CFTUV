"""Независимый повтор π-профиля Rust против actual mpmath 1.3.0."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from mpmath.libmp import libelefun as oracle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    requests, expected = [], []
    cells = dict(zip(oracle.pi_fixed.__code__.co_freevars, oracle.pi_fixed.__closure__))
    primitive = cells["f"].cell_contents
    # Начальный cold-порядок и три известные границы истории cache.
    for warm in (0, 849, 850, 1536):
        primitive.memo_prec, primitive.memo_val = -1, None
        oracle.pi_fixed(warm)
        for bits in range(801):
            requests.append(f"pi_fixed\t{bits}")
            expected.append(f"FIXED\t{oracle.pi_fixed(bits)}")
        for bits in range(2, 513):
            for rounding in "fcdun":
                requests.append(f"pi\t{bits}\t{rounding}")
                expected.append("MPF\t" + ",".join(map(str, oracle.mpf_pi(bits, rounding))))
    program = "\n".join(requests) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    args.out.with_suffix(".pid").write_text(str(process.pid), encoding="utf-8")
    output, error = process.communicate(program)
    rows = output.splitlines()
    mismatches = [{"index": index, "input": request, "expected": wanted, "actual": actual}
                  for index, (request, wanted, actual) in enumerate(zip(requests, expected, rows))
                  if wanted != actual]
    failed = bool(process.returncode or len(rows) != len(expected) or mismatches)
    receipt = {"scope": "bounded raw<=800 and mpf2..512, four reset cache histories",
               "cases": len(expected), "rows": len(rows), "exit_code": process.returncode,
               "mismatches": mismatches, "stderr": error,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(), "pid": process.pid}
    args.out.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key != "mismatches"}
                     | {"mismatches": len(mismatches)}), flush=True)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
