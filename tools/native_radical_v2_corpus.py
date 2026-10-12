"""Operation programs over `RadicalSumV2`: the Python oracle runner, the native runner, the records of the V1 shadow as programs, the comparison and the bench of the Rust mirror.

    python tools/native_radical_v2_corpus.py seed  --records DIR [DIR ...] [--out tests/data/native_radical_v2_ops.recs.xz]
    python tools/native_radical_v2_corpus.py compare [--records DIR ...] [--seed] [--synthetic N]
    python tools/native_radical_v2_corpus.py bench  [--records DIR ...] [--seed] [--limit N] [--json OUT]
    python tools/native_radical_v2_corpus.py program [--records DIR ...] [--seed] [--limit N] --out FILE      the rows as a text program for `cargo run --release --example replay`

A PROGRAM is a list of instruction tuples (`native/cftuv-radical/src/program.rs` is the other reader of the same tuples and of the text form below):

    ("zero",)  ("rational", n, d)  ("sqrt", n, d)  ("terms", ((r, n, d), ...))  ("wire", (int, ...))  ("text", str)
    ("add"|"sub"|"mul"|"div"|"eq", i, j)  ("inverse"|"neg"|"text_of"|"wire_of"|"as_rational"|"sep_bits", i)
    ("scaled", i, n, d)  ("signum", i, cap or None)  ("enclosure", i, bits)

An instruction that makes a value defines the register of its own index; the others read earlier registers. Every instruction has ONE OUTCOME tuple, the same on both sides:

    ("v", wire, work)  ("s", sign, work)  ("e", (ln, ld), (hn, hd), work)  ("t", text)  ("w", wire)  ("q", None | (n, d))  ("b", bool)  ("i", int)
    ("r", code, work, detail)  ("z", message)  ("x", message)  ("!", message)       (refusal with the work done before it; ZeroDivisionError; ValueError; separation bound violated)

The ORACLE side is `reference/radical_v2.py` and nothing else. The records are the operations of the V1 recorder (`tools/radical_v2_shadow.py`: the field corpus and the kernel suite) read BY VALUE:
each operand is loaded with `from_terms` (priced, so the price of the reading is compared too) and the operation is asked of the V2 values. The tracked seed
`tests/data/native_radical_v2_ops.recs.xz` is a deterministic stratified sample of them (JSON in xz, no pickle).
"""

from __future__ import annotations

import argparse
import gc
import json
import lzma
import pickle
import random
import statistics
import sys
import time
from fractions import Fraction
from functools import partial
from itertools import repeat
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from cftuv_envelope.reference import radical_v2 as v2  # noqa: E402

# the near-zero families have integers of thousands of digits, and the text form writes them in decimal
sys.set_int_max_str_digits(0)

SCHEMA = "RADICAL_V2_OPS_V1"
SEED_PATH = ROOT / "tests" / "data" / "native_radical_v2_ops.recs.xz"
NO_WORK = (0, 0, 0)

# --------------------------------------------------------------------------
# the oracle runner
# --------------------------------------------------------------------------


def _apply(instruction: tuple, values: dict):
    """The raw operation of one instruction on the oracle: `(tag, payload..., work)`; the ONLY place the methods of `RadicalSumV2` are called for a program."""

    op = instruction[0]
    if op == "zero":
        return ("v", v2.RadicalSumV2.zero(), NO_WORK)
    if op == "rational":
        return ("v", v2.RadicalSumV2.rational(Fraction(instruction[1], instruction[2])), NO_WORK)
    if op == "sqrt":
        value, work = v2.RadicalSumV2.sqrt_of_rational(Fraction(instruction[1], instruction[2]))
        return ("v", value, work.wire())
    if op == "terms":
        value, work = v2.RadicalSumV2.from_terms([(radicand, Fraction(numerator, denominator)) for radicand, numerator, denominator in instruction[1]])
        return ("v", value, work.wire())
    if op == "wire":
        return ("v", v2.value_of_wire(tuple(instruction[1])), NO_WORK)
    if op == "text":
        value, work = v2.value_of_text(instruction[1])
        return ("v", value, work.wire())
    x = values[instruction[1]]
    if op in ("add", "sub", "mul", "div"):
        value, work = getattr(x, op)(values[instruction[2]])
        return ("v", value, work.wire())
    if op == "inverse":
        value, work = x.inverse()
        return ("v", value, work.wire())
    if op == "neg":
        return ("v", x.neg(), NO_WORK)
    if op == "scaled":
        return ("v", x.scaled(Fraction(instruction[2], instruction[3])), NO_WORK)
    if op == "signum":
        sign, work = x.signum(instruction[2])
        return ("s", sign, work.wire())
    if op == "enclosure":
        (low, high), work = x.enclosure(instruction[2])
        return ("e", (low.numerator, low.denominator), (high.numerator, high.denominator), work.wire())
    if op == "text_of":
        return ("t", v2.canonical_text(x))
    if op == "wire_of":
        return ("w", v2.terms_wire(x))
    if op == "as_rational":
        rational = x.as_rational()
        return ("q", None if rational is None else (rational.numerator, rational.denominator))
    if op == "eq":
        return ("b", x == values[instruction[2]])
    if op == "sep_bits":
        return ("i", v2.separation_bits(v2._integer_form(x.terms)[1]))
    raise KeyError(op)


def _outcome(raw: tuple) -> tuple:
    return ("v", v2.terms_wire(raw[1]), raw[2]) if raw[0] == "v" else raw


def oracle_step(instruction: tuple, values: dict, index: int) -> tuple:
    """The outcome of one instruction on the oracle; a value is also stored in `values[index]`."""

    try:
        raw = _apply(instruction, values)
    except v2.RadicalRefusal as error:  # before ValueError: a refusal is one
        return ("r", error.code, error.work.wire(), error.detail)
    except v2.RadicalSeparationBoundViolated as error:
        return ("!", str(error))
    except ZeroDivisionError as error:
        return ("z", str(error))
    except ValueError as error:
        return ("x", str(error))
    except KeyError as error:
        return ("?", f"register {error} holds no value")
    if raw[0] == "v":
        values[index] = raw[1]
    return _outcome(raw)


def run_oracle(program) -> list:
    values: dict = {}
    return [oracle_step(instruction, values, index) for index, instruction in enumerate(program)]


def run_native(program, *, passes: int = 0, inner: int = 1):
    """`(outcomes, run_ns, timings)` of the extension (`cftuv_native.radical_v2_eval`)."""

    import cftuv_native

    return cftuv_native.radical_v2_eval(program, passes=passes, inner=inner)


# --------------------------------------------------------------------------
# the text form (one line per instruction / outcome; the vector files of `cargo test`)
# --------------------------------------------------------------------------


def _wire_text(wire) -> str:
    return ",".join(str(item) for item in wire) or "-"


def _work_text(work) -> str:
    return "%d,%d,%d" % tuple(work)


def instruction_text(instruction: tuple) -> str:
    op = instruction[0]
    if op == "zero":
        return "zero"
    if op in ("rational", "sqrt"):
        return f"{op} {instruction[1]} {instruction[2]}"
    if op == "terms":
        return "terms " + _wire_text([item for triple in instruction[1] for item in triple])
    if op == "wire":
        return "wire " + _wire_text(instruction[1])
    if op == "text":
        if any(ord(char) < 32 for char in instruction[1]):
            raise ValueError("a text with a control character has no line form")
        return "text " + instruction[1]
    if op == "scaled":
        return f"scaled {instruction[1]} {instruction[2]} {instruction[3]}"
    if op == "signum":
        return f"signum {instruction[1]} {'-' if instruction[2] is None else instruction[2]}"
    return " ".join(str(item) for item in instruction)


def outcome_text(outcome: tuple) -> str:
    tag = outcome[0]
    if tag == "v":
        return f"v {_wire_text(outcome[1])} {_work_text(outcome[2])}"
    if tag == "s":
        return f"s {outcome[1]} {_work_text(outcome[2])}"
    if tag == "e":
        return f"e {outcome[1][0]}/{outcome[1][1]} {outcome[2][0]}/{outcome[2][1]} {_work_text(outcome[3])}"
    if tag == "w":
        return f"w {_wire_text(outcome[1])}"
    if tag == "q":
        return "q -" if outcome[1] is None else f"q {outcome[1][0]}/{outcome[1][1]}"
    if tag == "b":
        return f"b {int(outcome[1])}"
    if tag == "r":
        return f"r {outcome[1]} {_work_text(outcome[2])} {outcome[3]}"
    return f"{tag} {outcome[1]}"


# --------------------------------------------------------------------------
# the records of the V1 shadow as programs
# --------------------------------------------------------------------------


def _terms_of(wire) -> tuple:
    """`("T", flat)` or `("Q", (n, d))` of a record as `(r, n, d)` triples (V1 representatives, any order); a rational is `(1, n, d)`."""

    if wire[0] == "Q":
        numerator, denominator = wire[1]
        return ((1, numerator, denominator),) if numerator else ()
    flat = wire[1]
    return tuple((flat[at], flat[at + 1], flat[at + 2]) for at in range(0, len(flat), 3))


class Builder:
    """A program under construction: operands are loaded once per distinct value (`from_terms`), operations point at the registers."""

    def __init__(self) -> None:
        self.program: list = []
        self.tags: list = []
        self._loaded: dict = {}

    def add(self, tag: str, instruction: tuple) -> int:
        self.program.append(instruction)
        self.tags.append(tag)
        return len(self.program) - 1

    def load(self, wire) -> int:
        terms = _terms_of(wire)
        register = self._loaded.get(terms)
        if register is None:
            register = self._loaded[terms] = self.add("from_terms", ("terms", terms))
            if len(terms) >= 1:
                self.add("text_of", ("text_of", register))
                self.add("wire_of", ("wire_of", register))
            if len(terms) >= 3:
                # the price caps: below the first rung, and at it (a refusal at the next rung when the first does not decide)
                self.add("signum_cap", ("signum", register, 63))
                self.add("signum_cap", ("signum", register, 64))
                self.add("sep_bits", ("sep_bits", register))
        return register

    def row(self, kind: str, operand, argument) -> None:
        if kind == "measure":
            return  # a binary64 measure of a V1 value: V1's, no V2 operation
        if kind == "sqrt_of_rational":
            numerator, denominator = argument[1]
            self.add("sqrt_of_rational", ("sqrt", numerator, denominator))
            return
        a = self.load(operand)
        if kind in ("add", "sub", "mul"):
            self.add(kind, (kind, a, self.load(argument)))
        elif kind == "rational_ratio":
            if not _terms_of(argument):
                return  # a zero divisor is answered `None` before any algebra
            quotient = self.add("div", ("div", a, self.load(argument)))
            self.add("as_rational", ("as_rational", quotient))
        elif kind == "neg":
            self.add("neg", ("neg", a))
        elif kind == "scaled":
            self.add("scaled", ("scaled", a, argument[1][0], argument[1][1]))
        elif kind == "inverse":
            self.add("inverse", ("inverse", a))
        elif kind == "signum":
            self.add("signum", ("signum", a, None))
        elif kind == "enclosure":
            self.add("enclosure", ("enclosure", a, argument))
        elif kind == "as_rational":
            self.add("as_rational", ("as_rational", a))
        elif kind == "canonical_text":
            self.add("text_of", ("text_of", a))
        elif kind == "terms_wire":
            self.add("wire_of", ("wire_of", a))
        else:
            raise KeyError(kind)


def program_of_rows(rows) -> tuple:
    """`(program, tags)` of record rows `(kind, operand, argument)`."""

    builder = Builder()
    for kind, operand, argument in rows:
        builder.row(kind, operand, argument)
    return builder.program, builder.tags


def readers_of(outcomes, limit: int) -> tuple:
    """The program that READS BACK what the oracle wrote: `("wire", w)` and `("text", t)` for the distinct values and texts of `outcomes`, in order of appearance."""

    program: list = []
    seen: set = set()
    for outcome in outcomes:
        if outcome[0] == "v":
            key = ("wire", outcome[1])
        elif outcome[0] == "t":
            key = ("text", outcome[1])
        else:
            continue
        if key not in seen and len(program) < limit:
            seen.add(key)
            program.append(key)
    return tuple(program)


def chunks_of(rows, size: int):
    for at in range(0, len(rows), size):
        yield rows[at : at + size]


# --------------------------------------------------------------------------
# reading and writing the rows
# --------------------------------------------------------------------------


def _row_key(row) -> str:
    return json.dumps(row, separators=(",", ":"), default=str)


def load_records(paths) -> dict:
    """`(kind, operand, argument) -> calls` over every `ops-*.pkl.xz` of the directories (the files `tools/radical_v2_shadow.py` writes; a local, trusted pickle)."""

    merged: dict = {}
    for path in map(Path, paths):
        files = sorted(path.glob("ops-*.pkl.xz")) if path.is_dir() else [path]
        for file in files:
            document = pickle.loads(lzma.decompress(file.read_bytes()))
            for kind, operand, argument, _result, count in document["rows"]:
                key = (kind, operand, argument)
                merged[key] = merged.get(key, 0) + count
    return merged


def _jsonable(value):
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    return value


def _tupled(value):
    if isinstance(value, list):
        return tuple(_tupled(item) for item in value)
    return value


def write_seed(path: Path, rows) -> int:
    document = {"schema": SCHEMA, "rows": [_jsonable(row) for row in rows]}
    data = lzma.compress(json.dumps(document, separators=(",", ":")).encode("ascii"), preset=9)
    path.write_bytes(data)
    return len(data)


def read_seed(path: Path = SEED_PATH) -> list:
    document = json.loads(lzma.decompress(path.read_bytes()).decode("ascii"))
    if document.get("schema") != SCHEMA:
        raise ValueError(f"{path}: schema {document.get('schema')!r}, expected {SCHEMA}")
    return [_tupled(row) for row in document["rows"]]


def _row_weight(row) -> tuple:
    """A sort key that puts the interesting rows first: more classes, bigger numbers."""

    kind, operand, argument = row
    terms = _terms_of(operand) if operand is not None else ()
    other = _terms_of(argument) if isinstance(argument, tuple) and argument and argument[0] in ("T", "Q") else ()
    size = max([item.bit_length() for term in terms + other for item in term] or [0])
    return (-(len(terms) + len(other)), -size)


def sample_rows(merged: dict, per_kind: int, seed: int) -> list:
    """A deterministic stratified sample: per kind, the rows with the most classes and the biggest numbers, and an even random sample of the rest."""

    by_kind: dict = {}
    for key in merged:
        if key[0] != "measure":  # a binary64 measure of a V1 value has no V2 operation
            by_kind.setdefault(key[0], []).append(key)
    rng = random.Random(seed)
    chosen: list = []
    for kind in sorted(by_kind):
        keys = sorted(by_kind[kind], key=_row_key)
        heaviest = sorted(keys, key=lambda row: (_row_weight(row), _row_key(row)))[: per_kind // 3]
        taken = set(heaviest)
        rest = [key for key in keys if key not in taken]
        rng.shuffle(rest)
        chosen += heaviest + rest[: per_kind - len(heaviest)]
    return sorted(set(chosen), key=_row_key)


# --------------------------------------------------------------------------
# synthetic programs
# --------------------------------------------------------------------------

SQUAREFREE = (2, 3, 5, 6, 7, 10, 11, 13, 14, 15, 17, 19, 21, 22, 23, 26, 29, 30, 31, 33, 35, 37, 38, 39, 41, 42, 43, 46, 47, 51, 53, 55)
CAPS = (None, None, None, 0, 63, 64, 65, 100, 129, 300, 4096)
BITS = (0, 1, 7, 64, 100, 129)


def _coefficient(rng: random.Random, big: bool) -> tuple:
    if big and rng.random() < 0.3:
        bits = rng.choice((70, 130, 260))
        return (rng.randint(-(1 << bits), 1 << bits) or 1, rng.randint(1, 1 << bits) * rng.choice((1, -1, 1)))
    return (rng.choice((-1, 1)) * rng.randint(0, 12), rng.choice((1, 1, 2, 3, 5, 7, 12, -4, -1)))


def _random_load(rng: random.Random, big: bool) -> tuple:
    kind = rng.random()
    if kind < 0.1:
        return ("rational", *_coefficient(rng, big))
    if kind < 0.18:
        n, d = rng.randint(-3, 40), rng.choice((1, 2, 3, 4, 9, 16, 25, -5, 7))
        return ("sqrt", n, d)
    classes = rng.randint(1, 4)
    pairs = []
    for radicand in rng.sample(SQUAREFREE, classes):
        for _ in range(rng.randint(1, 2)):
            square = rng.choice((1, 1, 1, 2, 3, 4, 5, 6))
            n, d = _coefficient(rng, big)
            if big and rng.random() < 0.25:
                radicand = radicand * rng.randint(1, 1 << 100) ** 2 if rng.random() < 0.5 else rng.randint(2, 1 << 200)
            pairs.append((radicand * square * square, n, d))
    if rng.random() < 0.5:
        n, d = _coefficient(rng, big)
        pairs.append((rng.choice((0, 1, 4, 9, 16, 100, 1)), n, d))
    if rng.random() < 0.04:
        pairs.append((-rng.randint(1, 30), 1, 1))
    rng.shuffle(pairs)
    return ("terms", tuple(pairs))


def random_program(rng: random.Random, big: bool = True) -> list:
    """Loads, then operations over random earlier values (results feed later operations: deep sums, refusals, cancellations)."""

    program: list = [_random_load(rng, big) for _ in range(rng.randint(2, 4))]
    values = [index for index, instruction in enumerate(program) if instruction[0] != "zero"]
    for _ in range(rng.randint(6, 18)):
        op = rng.choice(("add", "add", "sub", "sub", "mul", "mul", "div", "inverse", "neg", "scaled", "signum", "signum", "enclosure", "text_of", "wire_of", "as_rational", "eq", "sep_bits", "load"))
        pick = lambda: rng.choice(values)  # noqa: E731
        if op == "load":
            program.append(_random_load(rng, big))
            values.append(len(program) - 1)
            continue
        if op in ("add", "sub", "mul", "div", "eq"):
            program.append((op, pick(), pick()))
        elif op == "scaled":
            program.append((op, pick(), *_coefficient(rng, big)))
        elif op == "signum":
            program.append((op, pick(), rng.choice(CAPS)))
        elif op == "enclosure":
            program.append((op, pick(), rng.choice(BITS)))
        else:
            program.append((op, pick()))
        if op in ("add", "sub", "mul", "div", "inverse", "neg", "scaled") and len(values) < 14:
            values.append(len(program) - 1)  # a failed instruction holds no value: a later read of it is `?`, avoided below
    return program


def _read_positions(instruction: tuple) -> tuple:
    op = instruction[0]
    if op in ("add", "sub", "mul", "div", "eq"):
        return (1, 2)
    if op in ("inverse", "neg", "scaled", "signum", "enclosure", "text_of", "wire_of", "as_rational", "sep_bits"):
        return (1,)
    return ()


def synthetic_programs(count: int, seed: int = 20261012, big: bool = True):
    """`count` random programs whose reads are all of registers that hold a value (checked against the oracle)."""

    rng = random.Random(seed)
    for _ in range(count):
        program = random_program(rng, big)
        # the oracle decides which registers hold a value; a read of an empty one is cut, with the registers renumbered
        values: dict = {}
        kept: list = []
        mapping: dict = {}
        for index, instruction in enumerate(program):
            positions = _read_positions(instruction)
            if any(instruction[position] not in mapping for position in positions):
                continue
            rewritten = list(instruction)
            for position in positions:
                rewritten[position] = mapping[instruction[position]]
            rewritten = tuple(rewritten)
            outcome = oracle_step(rewritten, values, len(kept))
            if outcome[0] == "v":
                mapping[index] = len(kept)
            kept.append(rewritten)
        yield kept


def pell_powers(n: int, m: int) -> tuple:
    """`(sqrt2 - 1)^n = a + b sqrt2` and `(2 - sqrt3)^m = c + d sqrt3`, exact integers (both values in (0, 1))."""

    a, b = 1, 0
    for _ in range(n):
        a, b = -a + 2 * b, a - b
    c, d = 1, 0
    for _ in range(m):
        c, d = 2 * c - 3 * d, 2 * d - c
    return (a, b), (c, d)


def near_zero_programs(shapes=((3, 2), (6, 4), (9, 6), (25, 17), (40, 27), (97, 65), (300, 201), (3000, 2000), (4000, 2700))) -> list:
    """The constructive near-zero sums of the kernel test: tiny units whose sign is known without evaluating them, with no cap, with a cap of 4096, and with the bound."""

    programs = []
    for n, m in shapes:
        (a, b), (c, d) = pell_powers(n, m)
        program = [
            ("terms", ((1, a + c, 1), (2, b, 1), (3, d, 1))),
            ("terms", ((1, a - c, 1), (2, b, 1), (3, -d, 1))),
            ("terms", ((1, a * c, 1), (2, b * c, 1), (3, a * d, 1), (6, b * d, 1))),
        ]
        for register in range(3):
            program += [("signum", register, None), ("signum", register, 4096), ("signum", register, 8192), ("sep_bits", register), ("enclosure", register, 64)]
        programs.append(program)
    return programs


# --------------------------------------------------------------------------
# comparison
# --------------------------------------------------------------------------


class Tally:
    def __init__(self) -> None:
        self.programs = 0
        self.instructions = 0
        self.equal = 0
        self.refused = 0
        self.other_errors = 0
        self.dead_reads = 0
        self.by_tag: dict = {}
        self.mismatches: list = []

    def note(self, program, oracle, native) -> None:
        self.programs += 1
        for index, (expected, got) in enumerate(zip(oracle, native)):
            self.instructions += 1
            tag = expected[0]
            self.by_tag[tag] = self.by_tag.get(tag, 0) + 1
            if expected == got:
                self.equal += 1
                self.refused += tag == "r"
                self.other_errors += tag in ("z", "x", "!")
                self.dead_reads += tag == "?"  # a read of a register whose instruction failed (the `as_rational` of a refused `div` of a record): equal on both sides, no outcome of the oracle
            elif len(self.mismatches) < 10:
                self.mismatches.append({"index": index, "instruction": repr(program[index])[:600], "oracle": repr(expected)[:600], "native": repr(got)[:600]})
        if len(oracle) != len(native):
            self.mismatches.append({"length": (len(oracle), len(native))})

    def report(self) -> dict:
        return {
            "programs": self.programs, "instructions": self.instructions, "equal": self.equal, "differ": self.instructions - self.equal,
            "refusals_equal": self.refused, "other_errors_equal": self.other_errors, "dead_reads": self.dead_reads, "outcomes": dict(sorted(self.by_tag.items())), "mismatches": self.mismatches,
        }


def compare_program(program, tally: Tally, *, readers: int = 4000) -> None:
    """The program on both sides, then the readers of what the oracle wrote on both sides."""

    oracle = run_oracle(program)
    native = run_native(program)[0]
    tally.note(program, oracle, native)
    back = readers_of(oracle, readers)
    if back:
        oracle_back = run_oracle(back)
        tally.note(back, oracle_back, run_native(back)[0])


def compare_rows(rows, tally: Tally, *, chunk: int = 2500) -> None:
    for part in chunks_of(rows, chunk):
        program, _tags = program_of_rows(part)
        compare_program(program, tally)


# --------------------------------------------------------------------------
# the bench
# --------------------------------------------------------------------------


def _calls(program, values: dict, outcomes: list) -> list:
    """For every instruction a zero-argument callable of the oracle's RAW operation (the outcome tuple is not built), or `None` when there is nothing to time."""

    calls: list = []
    for index, instruction in enumerate(program):
        op = instruction[0]
        call = None
        if outcomes[index][0] != "?":
            if op in ("add", "sub", "mul", "div"):
                call = partial(getattr(values[instruction[1]], op), values[instruction[2]])
            elif op in ("inverse", "neg", "signum", "enclosure", "as_rational"):
                call = partial(getattr(values[instruction[1]], op), *instruction[2:]) if op in ("signum", "enclosure") else getattr(values[instruction[1]], op)
            elif op == "scaled":
                call = partial(values[instruction[1]].scaled, Fraction(instruction[2], instruction[3]))
            elif op == "terms":
                pairs = [(radicand, Fraction(numerator, denominator)) for radicand, numerator, denominator in instruction[1]]
                call = partial(v2.RadicalSumV2.from_terms, pairs)
            elif op == "sqrt":
                call = partial(v2.RadicalSumV2.sqrt_of_rational, Fraction(instruction[1], instruction[2]))
            elif op == "text_of":
                call = partial(v2.canonical_text, values[instruction[1]])
            elif op == "wire_of":
                call = partial(v2.terms_wire, values[instruction[1]])
            elif op == "wire":
                call = partial(v2.value_of_wire, tuple(instruction[1]))
            elif op == "text":
                call = partial(v2.value_of_text, instruction[1])
            elif op == "eq":
                call = partial(values[instruction[1]].__eq__, values[instruction[2]])
            elif op == "sep_bits":
                items = v2._integer_form(values[instruction[1]].terms)[1]
                call = partial(v2.separation_bits, items)
        calls.append(call)
    return calls


def _time(call, inner: int, passes: int) -> float:
    best = None
    ticks = repeat(None, inner)
    for _ in range(passes):
        started = time.perf_counter_ns()
        for _ in ticks:
            try:
                call()
            except (v2.RadicalRefusal, ZeroDivisionError, ValueError):
                pass
        elapsed = time.perf_counter_ns() - started
        best = elapsed if best is None else min(best, elapsed)
        ticks = repeat(None, inner)
    return best / inner


def python_timings(program, passes: int, inner: int) -> list:
    """Nanoseconds per execution of each instruction on the oracle, the loop's own cost (a call of a `partial` that does nothing) taken off, never below zero."""

    values: dict = {}
    outcomes = [oracle_step(instruction, values, index) for index, instruction in enumerate(program)]
    baseline = _time(partial(len, ()), inner, passes)
    timings = []
    for call in _calls(program, values, outcomes):
        timings.append(None if call is None else max(0.0, _time(call, inner, passes) - baseline))
    return timings


def _stats(values) -> dict:
    values = sorted(values)
    if not values:
        return {"n": 0}
    return {"n": len(values), "mean": statistics.fmean(values), "p50": values[len(values) // 2], "p95": values[min(len(values) - 1, int(len(values) * 0.95))]}


def bench_rows(rows, *, chunk: int = 2500, passes: int = 3, inner: int = 8) -> dict:
    """Per kind: the Rust and the Python time per execution over the distinct records; and the whole replay: Python, Rust run, Rust call (conversions included)."""

    per_kind_py: dict = {}
    per_kind_rs: dict = {}
    totals = {"python_run_s": 0.0, "rust_run_s": 0.0, "rust_call_s": 0.0, "instructions": 0}
    gc.disable()
    try:
        for part in chunks_of(rows, chunk):
            program, tags = program_of_rows(part)
            started = time.perf_counter()
            run_oracle(program)
            totals["python_run_s"] += time.perf_counter() - started
            started = time.perf_counter()
            _outcomes, run_ns, rust_timings = run_native(program, passes=passes, inner=inner)
            totals["rust_call_s"] += time.perf_counter() - started
            totals["rust_run_s"] += run_ns / 1e9
            totals["instructions"] += len(program)
            python_times = python_timings(program, passes, inner)
            for tag, python, rust in zip(tags, python_times, rust_timings):
                if python is None:
                    continue
                per_kind_py.setdefault(tag, []).append(python)
                per_kind_rs.setdefault(tag, []).append(rust)
    finally:
        gc.enable()
    table = {}
    for tag in sorted(per_kind_py):
        py, rs = _stats(per_kind_py[tag]), _stats(per_kind_rs[tag])
        table[tag] = {"python_ns": py, "rust_ns": rs, "speedup_mean": py["mean"] / rs["mean"] if rs["mean"] else None, "speedup_p50": py["p50"] / rs["p50"] if rs["p50"] else None}
    return {"per_kind": table, "totals": totals, "passes": passes, "inner": inner}


def bench_totals(rows, *, chunk: int = 2500) -> dict:
    """The whole replay of `rows` only (no per-instruction timing): the oracle, the extension's RUN, and the extension's CALL with the conversions of the boundary."""

    totals = {"python_run_s": 0.0, "rust_run_s": 0.0, "rust_call_s": 0.0, "instructions": 0, "rows": len(rows)}
    gc.disable()
    try:
        for part in chunks_of(rows, chunk):
            program, _tags = program_of_rows(part)
            started = time.perf_counter()
            run_oracle(program)
            totals["python_run_s"] += time.perf_counter() - started
            started = time.perf_counter()
            _outcomes, run_ns, _none = run_native(program)
            totals["rust_call_s"] += time.perf_counter() - started
            totals["rust_run_s"] += run_ns / 1e9
            totals["instructions"] += len(program)
    finally:
        gc.enable()
    return {"per_kind": {}, "totals": totals}


def format_bench(report: dict) -> str:
    lines = [f"{'op':<18}{'n':>8}  {'py mean':>10}{'py p50':>10}{'py p95':>10}  {'rs mean':>10}{'rs p50':>10}{'rs p95':>10}  {'x mean':>8}{'x p50':>8}   (ns per execution)"]
    for tag, row in report["per_kind"].items():
        py, rs = row["python_ns"], row["rust_ns"]
        lines.append(
            f"{tag:<18}{py['n']:>8}  {py['mean']:>10.0f}{py['p50']:>10.0f}{py['p95']:>10.0f}  {rs['mean']:>10.0f}{rs['p50']:>10.0f}{rs['p95']:>10.0f}  "
            f"{row['speedup_mean'] or 0:>7.1f}x{row['speedup_p50'] or 0:>7.1f}x"
        )
    totals = report["totals"]
    lines.append(
        f"whole replay, {totals['instructions']} instructions: python {totals['python_run_s']:.2f} s | rust run {totals['rust_run_s']:.3f} s "
        f"({totals['python_run_s'] / max(totals['rust_run_s'], 1e-9):.1f}x) | rust call incl. conversions {totals['rust_call_s']:.2f} s ({totals['python_run_s'] / max(totals['rust_call_s'], 1e-9):.1f}x)"
    )
    return "\n".join(lines)


# --------------------------------------------------------------------------
# command line
# --------------------------------------------------------------------------


def _rows_from(arguments) -> list:
    rows: list = []
    if arguments.seed:
        rows += read_seed()
    if arguments.records:
        merged = load_records(arguments.records)
        rows += sorted(merged, key=_row_key)
    seen: set = set()
    unique = []
    for row in rows:
        if row not in seen:
            seen.add(row)
            unique.append(row)
    if arguments.limit and len(unique) > arguments.limit:
        random.Random(arguments.sample_seed).shuffle(unique)
        unique = unique[: arguments.limit]
    return unique


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("seed", "compare", "bench", "program"))
    parser.add_argument("--records", nargs="*", default=[], help="directories (or files) of `ops-*.pkl.xz` written by tools/radical_v2_shadow.py")
    parser.add_argument("--seed", action="store_true", help="the tracked seed")
    parser.add_argument("--out", default=str(SEED_PATH))
    parser.add_argument("--per-kind", type=int, default=420, help="seed: rows per kind")
    parser.add_argument("--limit", type=int, default=0, help="at most this many rows (a seeded random sample)")
    parser.add_argument("--sample-seed", type=int, default=7)
    parser.add_argument("--synthetic", type=int, default=0, help="compare: synthetic programs")
    parser.add_argument("--json", default="")
    parser.add_argument("--passes", type=int, default=3, help="bench: passes per instruction (the best is kept)")
    parser.add_argument("--inner", type=int, default=8, help="bench: executions in a row per pass")
    parser.add_argument("--totals-only", action="store_true", help="bench: only the whole replay (no per-instruction timing)")
    arguments = parser.parse_args(argv)
    if arguments.command == "seed":
        merged = load_records(arguments.records)
        rows = sample_rows(merged, arguments.per_kind, arguments.sample_seed)
        size = write_seed(Path(arguments.out), rows)
        kinds: dict = {}
        for row in rows:
            kinds[row[0]] = kinds.get(row[0], 0) + 1
        print(f"RADICAL_V2_SEED rows={len(rows)} of {len(merged)} bytes={size} kinds={json.dumps(kinds, sort_keys=True)}")
        return 0
    rows = _rows_from(arguments)
    if arguments.command == "program":
        # the rows as ONE program in the text form: `cargo run --release --example replay -- FILE` times it without Python (native/cftuv-radical/examples/replay.rs)
        program, _tags = program_of_rows(rows)
        text = chr(10).join(instruction_text(instruction) for instruction in program) + chr(10)
        Path(arguments.out).write_bytes(text.encode("utf-8"))
        print(f"RADICAL_V2_PROGRAM instructions={len(program)} bytes={len(text)} -> {arguments.out}")
        return 0
    if arguments.command == "compare":
        tally = Tally()
        started = time.perf_counter()
        compare_rows(rows, tally)
        for program in near_zero_programs():
            compare_program(program, tally)
        for program in synthetic_programs(arguments.synthetic):
            compare_program(program, tally)
        report = tally.report()
        report["rows"] = len(rows)
        report["seconds"] = round(time.perf_counter() - started, 1)
        print(json.dumps(report, indent=1, sort_keys=True))
        return 0 if not report["differ"] else 1
    report = bench_totals(rows) if arguments.totals_only else bench_rows(rows, passes=arguments.passes, inner=arguments.inner)
    print(format_bench(report))
    if arguments.json:
        Path(arguments.json).write_text(json.dumps(report, indent=1, sort_keys=True), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
