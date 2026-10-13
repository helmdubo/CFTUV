"""Сравнение сырых MPF/MPI концов с фактическим mpmath 1.3.0, без смены backend."""

from __future__ import annotations

import argparse
from collections import Counter
from functools import cmp_to_key
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys

import mpmath
from mpmath.libmp import libmpf as f, libmpi as iv

sys.set_int_max_str_digits(0)


def wire(value):
    if len(value) == 2:
        return "|".join(wire(endpoint) for endpoint in value)
    return ",".join(str(int(part)) for part in value)


def expected(call, interval=False):
    try:
        return ("MPI" if interval else "MPF") + "\t" + wire(call())
    except ZeroDivisionError:
        return "ERROR\tZeroDivision"
    except f.ComplexResult:
        return "ERROR\tComplexResult"


def cases(samples):
    rng = random.Random(0x160A213)
    values = [f.fzero, f.fone, f.fnone, f.fhalf, f.finf, f.fninf, f.fnan]
    for mantissa in (-33, -31, -17, -15, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 15, 17, 31, 33):
        for exponent in (-163, -2, 0, 159):
            values.append(f.from_man_exp(mantissa, exponent))
    for _ in range(samples):
        bits = rng.choice((1, 2, 53, 159, 160, 161, 257, 513))
        mantissa = rng.getrandbits(bits) * rng.choice((-1, 1))
        values.append(f.from_man_exp(mantissa, rng.randrange(-1024, 1025)))
    values += [f.from_man_exp(sign, exponent)
               for exponent in (-1000000, 1000000) for sign in (-1, 1)]
    precisions = (2, 3, 7, 53, 160, 257)
    unary = {"pos": f.mpf_pos, "neg": f.mpf_neg, "sqrt": f.mpf_sqrt,
             "square": lambda a, p, r: f.mpf_mul(a, a, p, r)}
    binary = {"add": f.mpf_add, "sub": f.mpf_sub, "mul": f.mpf_mul, "div": f.mpf_div}
    for p in precisions:
        for rounding in "fcdun":
            for index, a in enumerate(values):
                b = values[(index * 17 + 3) % len(values)]
                for op, fn in unary.items():
                    yield f"{op}\t{p}\t{rounding}\t{wire(a)}\t-", expected(lambda: fn(a, p, rounding))
                for op, fn in binary.items():
                    yield f"{op}\t{p}\t{rounding}\t{wire(a)}\t{wire(b)}", expected(lambda: fn(a, b, p, rounding))
                power = (-17, -4, -3, -2, -1, 0, 1, 2, 3, 4, 17)[index % 11]
                yield f"pow\t{p}\t{rounding}\t{wire(a)}\t{power}", expected(lambda: f.mpf_pow_int(a, power, p, rounding))
            # Все комбинации особых значений, включая inf*0 и деление на ноль.
            for a in values[:7]:
                for b in values[:7]:
                    for op, fn in binary.items():
                        yield f"{op}\t{p}\t{rounding}\t{wire(a)}\t{wire(b)}", expected(lambda: fn(a, b, p, rounding))
            for n, d in [(1657, 5120), (3647, 5120), (-1657, 5120), (0, 1),
                         (1, 0), (0, 0), (3, -7), (2**521-1, 2**257+1)]:
                yield f"rational\t{p}\t{rounding}\t{n}\t{d}", expected(lambda: f.from_rational(n, d, p, rounding))
            for n in (-35, -33, -31, -17, -15, -9, -7, -5, -3, 0, 3, 5, 7, 9, 15, 17, 31, 33, 35, 2**521-1):
                yield f"normalize\t{p}\t{rounding}\t{n},-83\t-", expected(lambda: f.from_man_exp(n, -83, p, rounding))

    # NaN не является концом упорядоченного интервала; scalar NaN покрыт выше.
    intervals = [(f.fzero, f.fzero), (f.fnone, f.fone), (f.fninf, f.finf),
                 (f.fninf, f.fzero), (f.fzero, f.finf), (f.finf, f.finf),
                 (f.fninf, f.fninf), (f.fone, f.fone)]
    finite = [a for a in values if a not in (f.fnan, f.finf, f.fninf)]
    for index, a in enumerate(finite):
        b = finite[(index * 11 + 1) % len(finite)]
        intervals.append(tuple(sorted((a, b), key=cmp_to_key(f.mpf_cmp))))
    i_unary = {"i_pos": iv.mpi_pos, "i_neg": iv.mpi_neg, "i_abs": iv.mpi_abs,
               "i_square": iv.mpi_square, "i_sqrt": iv.mpi_sqrt}
    i_binary = {"i_add": iv.mpi_add, "i_sub": iv.mpi_sub, "i_mul": iv.mpi_mul, "i_div": iv.mpi_div}
    for p in precisions:
        for index, a in enumerate(intervals):
            b = intervals[(index * 13 + 7) % len(intervals)]
            for op, fn in i_unary.items():
                yield f"{op}\t{p}\t-\t{wire(a)}\t-", expected(lambda: fn(a, p), True)
            for op, fn in i_binary.items():
                yield f"{op}\t{p}\t-\t{wire(a)}\t{wire(b)}", expected(lambda: fn(a, b, p), True)
            for power in (-7, -4, -3, -2, -1, 0, 1, 2, 3, 4, 7):
                yield f"i_pow\t{p}\t-\t{wire(a)}\t{power}", expected(lambda: iv.mpi_pow_int(a, power, p), True)
        for a in intervals[:8]:
            for b in intervals[:8]:
                for op, fn in i_binary.items():
                    yield f"{op}\t{p}\t-\t{wire(a)}\t{wire(b)}", expected(lambda: fn(a, b, p), True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    if mpmath.__version__ != "1.3.0":
        raise SystemExit("Oracle requires mpmath 1.3.0")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = list(cases(args.samples))
    program = "\n".join(row for row, _ in rows)+"\n"
    input_path = args.out.with_suffix(".input.tsv")
    expected_path = args.out.with_suffix(".expected.tsv")
    input_path.write_text(program, encoding="utf-8")
    expected_path.write_text("\n".join(answer for _, answer in rows)+"\n", encoding="utf-8")
    receipt = {"status": "PREPARED", "scope": "mpmath 1.3.0 scalar and interval arithmetic endpoints; no trig or product parity",
               "cases": len(rows), "operations": dict(Counter(row.split("\t")[0] for row, _ in rows)),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(),
               "oracle_source_sha256": {module.__name__: hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() for module in (f, iv)}}
    failures = []
    if not args.prepare_only:
        if args.exe is None:
            parser.error("--exe required without --prepare-only")
        with input_path.open(encoding="utf-8") as source, args.out.with_suffix(".actual.tsv").open("w", encoding="utf-8") as target, args.out.with_suffix(".stderr").open("w", encoding="utf-8") as errors:
            process = subprocess.Popen([str(args.exe)], stdin=source, stdout=target, stderr=errors, text=True)
            args.out.with_suffix(".pid").write_text(str(process.pid), encoding="utf-8")
            code = process.wait()
        actual = args.out.with_suffix(".actual.tsv").read_text(encoding="utf-8").splitlines()
        if code or len(actual) != len(rows):
            failures.append({"exit": code, "actual_rows": len(actual), "expected_rows": len(rows)})
        for index, ((request, answer), result) in enumerate(zip(rows, actual)):
            if answer != result:
                failures.append({"index": index, "input": request, "expected": answer, "actual": result})
        receipt.update(status="PASS" if not failures else "FAIL", mismatches=len(failures),
                       failures=failures, binary_sha256=hashlib.sha256(args.exe.read_bytes()).hexdigest(), pid=process.pid)
    args.out.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key != "failures"}), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
