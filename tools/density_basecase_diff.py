"""Повтор actual полевых basecase-входов на Rust; выходы уже записаны оракулом."""

import argparse
import hashlib
import json
import lzma
from pathlib import Path
import subprocess
import sys

sys.set_int_max_str_digits(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(lzma.decompress(args.file.read_bytes()))
    requests, expected, unsupported = [], [], []
    for name, values, keywords, result in data["records"]:
        if name != "cos_sin_basecase":
            continue
        x, precision = values
        if keywords or not 8 <= precision <= 400 or result[0] != "ok":
            unsupported.append([values, keywords, result])
            continue
        requests.append(f"basecase\t{precision}\t{x}")
        expected.append("FIXED_PAIR\t" + "\t".join(map(str, result[1])))
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
    receipt = {"scope": data["scope"], "cases": len(expected), "rows": len(rows),
               "exit_code": process.returncode, "mismatches": mismatches,
               "unsupported": unsupported, "stderr": error,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "source_sha256": hashlib.sha256(args.file.read_bytes()).hexdigest(), "pid": process.pid}
    args.out.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items()
                      if key not in ("mismatches", "unsupported")}
                    | {"mismatches": len(mismatches), "unsupported": len(unsupported)}), flush=True)
    return int(bool(process.returncode or len(rows) != len(expected) or mismatches or unsupported))


if __name__ == "__main__":
    raise SystemExit(main())
