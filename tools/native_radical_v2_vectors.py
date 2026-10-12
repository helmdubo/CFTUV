"""The test vectors of the Rust crate `cftuv-radical`: programs and the outcomes the ORACLE (`reference/radical_v2.py`) gives them, as plain text for `cargo test` (no Python at test time).

    python tools/native_radical_v2_vectors.py write [--out native/cftuv-radical/tests/vectors.txt] [--random 300]
    python tools/native_radical_v2_vectors.py check

`cargo test -p cftuv-radical` replays them (`tests/vectors.rs`): every instruction's outcome rendered by the crate must be the line the oracle's outcome renders to -- values as wires, texts, signs,
enclosures, the three work counters, and the named refusals with their work and detail. `check` (and `tests/test_native_radical_v2.py`) compares the file with what the oracle writes NOW: a
vector file the oracle has moved away from is a failure, not a stale fixture.

Format (one block per case; the grammar of the lines is `program.rs`):

    P name
    I <instruction>        one per instruction, in order
    O <outcome>            one per instruction, in order
    E

The cases: the PRICE vectors of `kernel/tests/test_radical_sum_v2.py` (the counting rules on fixed inputs), the near-zero families (sqrt2 - 1)^n and (2 - sqrt3)^m including (3000, 2000) and
(4000, 2700) under the cap of 4096 and the bound, the cancellations across representations, one value in several forms (a canonical form is a function of the value), the inverse class limit and the
other refusals, the readers of wires and texts with every message, and random programs (deep sums, big integers, caps) from `native_radical_v2_corpus.synthetic_programs`.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import native_radical_v2_corpus as corpus  # noqa: E402

DEFAULT_OUT = ROOT / "native" / "cftuv-radical" / "tests" / "vectors.txt"
KERNEL_TEST = ROOT / "kernel" / "tests" / "test_radical_sum_v2.py"


def price_vectors() -> dict:
    """`PRICE_VECTORS` of the kernel test, read from the test file itself (the kernel tests directory is put on the path for its helper import)."""

    sys.path.insert(0, str(ROOT / "kernel" / "tests"))
    sys.path.insert(0, str(ROOT / "kernel" / "src"))
    spec = importlib.util.spec_from_file_location("kernel_test_radical_sum_v2", KERNEL_TEST)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PRICE_VECTORS


def _terms(*pairs) -> tuple:
    return ("terms", tuple((radicand, Fraction(c).numerator, Fraction(c).denominator) for radicand, c in pairs))


def price_programs() -> dict:
    one_plus_root2 = _terms((1, 1), (2, 1))
    return {
        "from_terms": [("terms", ((2, 1, 1), (8, 1, 1), (3, 1, 1), (12, 1, 1)))],
        "mul": [one_plus_root2, _terms((1, 1), (3, 1)), ("mul", 0, 1)],
        "inverse3": [_terms((1, 1), (2, 1), (3, 1)), ("inverse", 0)],
        "inverse3_norm2": [_terms((1, Fraction(1, 2)), (5, Fraction(-3, 7)), (7, 2)), ("inverse", 0)],
        "div": [_terms((1, Fraction(1, 2)), (5, Fraction(-3, 7)), (7, 2)), one_plus_root2, ("div", 0, 1)],
        "add_equal_m": [_terms((2, 1), (3, 1)), _terms((8, Fraction(1, 2)), (27, Fraction(-1, 3))), ("add", 0, 1)],
        "sqrt": [("sqrt", 8, 9)],
        "sqrt_square": [("sqrt", 4, 9)],
    }


def cases(random_count: int) -> list:
    out: list = []
    expected = price_vectors()
    for name, program in sorted(price_programs().items()):
        program = [*program, ("text_of", len(program) - 1)]
        outcomes = corpus.run_oracle(program)
        text, work = expected[name]
        assert outcomes[-2][0] == "v" and outcomes[-1] == ("t", text) and outcomes[-2][2] == work, f"PRICE_VECTORS[{name}] is not what the oracle gives: {outcomes[-2:]}"
        out.append((f"price/{name}", program))
    for index, program in enumerate(corpus.near_zero_programs()):
        out.append((f"near_zero/{index}", program))
    # cancellations across representatives, and one value in several forms
    cancellations = [
        [(2, 1), (8, 1), (18, -1)], [(12, 1), (3, -2)], [(24, Fraction(1, 6)), (6, Fraction(-1, 3))], [(50, Fraction(3, 5)), (2, -3)], [(1, 5), (4, Fraction(-5, 2))], [(72, 1), (8, -3)],
    ]
    for index, pairs in enumerate(cancellations):
        out.append((f"cancel/{index}", [_terms(*pairs), ("text_of", 0), ("signum", 0, None), ("as_rational", 0)]))
    forms = [[(18, 1)], [(2, 3)], [(8, Fraction(3, 2))], [(2, 1), (8, 1)], [(50, Fraction(3, 5))], [(2, 5), (8, -1)], [(32, Fraction(3, 4))], [(72, Fraction(1, 2))]]
    program = [_terms(*pairs) for pairs in forms]
    program += [("eq", 0, index) for index in range(1, len(forms))] + [("wire_of", index) for index in range(len(forms))] + [("text_of", index) for index in range(len(forms))]
    out.append(("unique/one_value_eight_forms", program))
    program = [_terms((1, 1), (2, 1), (3, 1), (5, 1)), ("inverse", 0), ("div", 0, 0), ("zero",), ("inverse", 3), ("div", 0, 3), ("sqrt", -1, 2), ("sqrt", 5, 0), ("terms", ((-3, 1, 1),)), ("terms", ((2, 1, 0),))]
    out.append(("refusals/inverse_and_field", program))
    out.append(("refusals/inverse_after_work", [_terms((1, 1), (2, 1)), _terms((2, 1), (3, 1), (5, 1), (7, 1)), ("terms", ((2, 1, 1), (-5, 1, 1))), ("div", 0, 1), ("mul", 0, 1), ("inverse", 1)]))
    # readers
    wires = [(), (1,), (2, 1, 1, 8, 1, 1), (2, 1, 1, 8, 1), (8, 2, 1), (4, 1, 1), (1, 2, 4), (1, 3, 2), (1, 0, 1), (6, 1, 4), (12, 1, 4), (2, -1, 1, 12, 1, 4), (3, 1, 1, 2, 1, 1), (1, 1, 0), (0, 1, 1), (-2, 1, 1), (8, 1, 3)]
    out.append(("readers/wires", [("wire", wire) for wire in wires]))
    texts = [
        "Integer(0)", "Integer(-5)", "Integer(007)", "Rational(3, 6)", "Rational(-4, 6)", "Rational(1, 0)", "Rational(5, -2)", "Sqrt(Rational(2, 1))", "-Sqrt(Rational(18, 4))", "Sqrt(Rational(1, 0))", "Sqrt(Rational(0, 5))",
        "Sqrt(Rational(4, 1))", "Sum(Rational(-3, 4), -Sqrt(Rational(2, 1)), Sqrt(Rational(3, 4)))", "Sum(Sqrt(Rational(2, 1)), Sqrt(Rational(8, 1)))", "Sum(Integer(1), Integer(2))", "Sum(Integer(1), -Integer(2))",
        "Sum()", "Sum(Integer(1),)", "Sum(Integer(1)", "Integer(1), Integer(2)", "", " ", "Integer(1) ", "Integer(+1)", "Sqrt(Rational(2,1))", "it's", 'say "x"', "it's \"x\"", "Integer(x)", "Rational(1 2)", "-Integer(1)",
        "Sum(Sqrt(Rational(2, 1)), Sqrt(Rational(2, 1)))", "Sum(Sqrt(Rational(2, 1)), -Sqrt(Rational(2, 1)))", "Sum(Integer(1), Rational(1, 2), Sqrt(Rational(3, 2)))", "café",
    ]
    out.append(("readers/texts", [("text", text) for text in texts]))
    # a few scaled / enclosure / signum edge cases on small values
    base = [_terms((1, Fraction(-3, 4)), (2, -1), (3, Fraction(1, 2)))]
    edge = base + [("scaled", 0, 0, 1), ("scaled", 0, 3, 0), ("scaled", 0, -2, 4), ("enclosure", 0, 0), ("enclosure", 0, 5), ("enclosure", 0, -1), ("signum", 0, None), ("signum", 0, 0), ("signum", 0, -7), ("sep_bits", 0)]
    zero_at = len(edge)
    edge += [("zero",), ("sep_bits", zero_at), ("enclosure", zero_at, 3), ("signum", zero_at, 0), ("neg", 0), ("neg", zero_at), ("as_rational", zero_at), ("as_rational", 0), ("eq", 0, zero_at)]
    out.append(("edges/small", edge))
    # random programs: small integers (deep sums, caps, refusals) and a few with big integers; the ones whose text is long are left to the Python differential tests
    for label, big, wanted, limit, seed in (("small", False, random_count, 4000, 20261012), ("big", True, max(1, random_count // 25), 12000, 20261013)):
        kept = 0
        for program in corpus.synthetic_programs(wanted * 8, seed=seed, big=big):
            size = sum(len(corpus.outcome_text(outcome)) + 1 for outcome in corpus.run_oracle(program)) + sum(len(corpus.instruction_text(instruction)) + 1 for instruction in program)
            if size > limit:
                continue
            out.append((f"random_{label}/{kept}", program))
            kept += 1
            if kept == wanted:
                break
        assert kept == wanted, f"only {kept} of {wanted} random {label} programs fit in {limit} characters"
    return out


def lines(random_count: int = 300) -> list:
    text: list = ["# generated by tools/native_radical_v2_vectors.py from reference/radical_v2.py; do not edit"]
    for name, program in cases(random_count):
        outcomes = corpus.run_oracle(program)
        assert not any(outcome[0] == "?" for outcome in outcomes), f"{name}: a program with a read of an empty register"
        text.append(f"P {name}")
        text.extend(f"I {corpus.instruction_text(instruction)}" for instruction in program)
        text.extend(f"O {corpus.outcome_text(outcome)}" for outcome in outcomes)
        text.append("E")
    return text


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("write", "check"))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--random", type=int, default=300)
    arguments = parser.parse_args(argv)
    written = "\n".join(lines(arguments.random)) + "\n"
    path = Path(arguments.out)
    if arguments.command == "write":
        path.write_bytes(written.encode("utf-8"))
        print(f"RADICAL_V2_VECTORS {path} bytes={len(written.encode('utf-8'))} cases={written.count(chr(10) + 'P ') + 1}")
        return 0
    current = path.read_bytes().decode("utf-8").replace("\r\n", "\n")
    if current != written:
        print(f"RADICAL_V2_VECTORS_STALE {path} differs from what the oracle writes now")
        return 1
    print("RADICAL_V2_VECTORS_CURRENT")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
