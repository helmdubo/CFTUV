"""Numeric-only construction candidate audit; roots and fresh generated recipes.

Old source literal deltas are named observations, never compatibility acceptance.
The shared process never edits SymPy caches; state witnesses run in isolated
children. No product controller, sign upgrade, constructor hook or native call.
"""

from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import lzma
import os
from pathlib import Path
import subprocess
import sys
import time

from density_tool_paths import KERNEL_ROOT, PACKAGE_ROOT, TOOLS_ROOT, kernel_child

ROOT = KERNEL_ROOT.parent

from cftuv_envelope.reference.density_scalar_v2 import (  # noqa: E402
    DENSITY_SOURCE_CONSTRUCTION_V1,
    DensityScalarProfileUnsupported,
    ScalarV2,
)


class ImportedNumericProfileUnsupported(ValueError):
    code = "DENSITY_SCALAR_V2_IMPORTED_AST_UNSUPPORTED"


class _RustRunner:
    """Harness-only live parity at every operation; not a product dispatch."""

    def __init__(self, executable, out):
        self.executable = executable
        self.stderr = (out/"rust.stderr.log").open("w", encoding="utf-8")
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.stderr, text=True, bufsize=1,
                                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        log = os.environ.get("CFTUV_DENSITY_PID_LOG")
        if log:
            with Path(log).open("a", encoding="utf-8") as stream:
                _write(stream, {"pid": self.process.pid, "task": "density-scalar-v2-actual-rust-parity"})
        self.commands, self.compared, self.expected_errors, self.registers = 0, 0, 0, 0
        self.reset()

    def request(self, command):
        self.process.stdin.write(command+"\n")
        self.process.stdin.flush()
        self.commands += 1
        response = self.process.stdout.readline().rstrip("\n\r")
        if not response:
            raise RuntimeError("Rust scalar process ended without response")
        return response

    def reset(self):
        if self.request("clear") != "CLEARED":
            raise RuntimeError("Rust scalar clear response mismatch")
        self.registers = 0

    def operation(self, command, calculate):
        try:
            scalar = calculate()
        except (DensityScalarProfileUnsupported, ZeroDivisionError) as error:
            response = self.request(command)
            if isinstance(error, DensityScalarProfileUnsupported):
                expected = "PROFILE\t"+error.code.value+"\t"
                if not response.startswith(expected):
                    raise AssertionError(f"Rust profile differs: {response}; Python: {error}") from error
            elif not response.startswith("INTERNAL\t") or "ZERO" not in response.upper():
                raise AssertionError(f"Rust exact zero differs: {response}") from error
            self.expected_errors += 1
            raise
        response = self.request(command)
        fields = response.split("\t", 2)
        if len(fields) != 3 or fields[0] != "SCALAR" or int(fields[1]) != self.registers:
            raise AssertionError(f"Rust scalar register response mismatch: {response}")
        literal = scalar.literal_ast()
        expected = {"canon": literal.canon, "v2_wire": list(scalar.value.wire()),
                    "nodes": [list(row) for row in literal.nodes], "root": literal.root}
        actual = json.loads(fields[2])
        if actual != expected:
            raise AssertionError(_dump({"command": command, "expected": expected, "actual": actual}))
        result = _DualScalar(self, scalar, self.registers)
        self.registers += 1
        self.compared += 1
        return result

    def rational(self, value):
        rational = Fraction(value)
        return self.operation(f"q\t{rational.numerator}\t{rational.denominator}", lambda: ScalarV2.rational(rational))

    def input_value(self, value):
        rational = value.value.as_rational()
        if rational is not None:
            return self.rational(rational)
        return self.operation("wire\t"+",".join(map(str, value.value.wire())), lambda: value)

    def close(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=45)
        self.stderr.close()
        if code:
            raise RuntimeError(f"Rust scalar executable exited {code}")
        return {"executable": str(self.executable), "sha256": _sha(self.executable), "exit_code": code,
                "stderr_bytes": (Path(self.stderr.name)).stat().st_size,
                "commands": self.commands, "successful_operation_values_and_literals_compared": self.compared,
                "expected_numeric_or_profile_errors_compared": self.expected_errors,
                "rust_integer_power_profile": "Signed i64 exponent with |e| <= 2**20; an explicit internal price boundary, not a geometry refusal.",
                "scope": "Actual Rust fresh-operation results, including every emitted decision literal; input values do not substitute generated results."}


class _DualScalar:
    def __init__(self, runner, scalar, index):
        self.runner, self.scalar, self.index = runner, scalar, index

    @property
    def value(self):
        return self.scalar.value

    def literal_ast(self):
        return self.scalar.literal_ast()

    def coerce(self, other):
        if isinstance(other, _DualScalar):
            if other.runner is not self.runner:
                raise ValueError("scalar parity transaction mismatch")
            return other
        return self.runner.input_value(other) if isinstance(other, ScalarV2) else self.runner.rational(other)

    def binary(self, operation, other):
        other = self.coerce(other)
        return self.runner.operation(f"{operation}\t{self.index}\t{other.index}",
                                     lambda: getattr(self.scalar, operation)(other.scalar))

    def add(self, other):
        return self.binary("add", other)

    def sub(self, other):
        return self.binary("sub", other)

    def mul(self, other):
        return self.binary("mul", other)

    def div(self, other):
        return self.binary("div", other)

    def unary(self, operation):
        return self.runner.operation(f"{operation}\t{self.index}", lambda: getattr(self.scalar, operation)())

    def sqrt(self):
        return self.unary("sqrt")

    def inverse(self):
        return self.unary("inverse")

    def neg(self):
        return self.unary("neg")

    def scaled(self, factor):
        rational = Fraction(factor)
        return self.runner.operation(f"scaled\t{self.index}\t{rational.numerator}\t{rational.denominator}",
                                     lambda: self.scalar.scaled(rational))

    def integer_power(self, exponent):
        return self.runner.operation(f"pow\t{self.index}\t{exponent}", lambda: self.scalar.integer_power(exponent))

    def __eq__(self, other):
        return self.scalar == (other.scalar if isinstance(other, _DualScalar) else other)

    __add__ = add
    __sub__ = sub
    __mul__ = mul
    __truediv__ = div
    __neg__ = neg
    __pow__ = integer_power


def _constant(reference, value):
    return reference.runner.rational(value) if isinstance(reference, _DualScalar) else ScalarV2.rational(value)


class _NumericReader:
    """Tool-only source-fact reader, from raw integer rows, never evaluated SymPy."""

    def __init__(self, nodes, constructor=ScalarV2.rational):
        self.nodes, self.memo = nodes, {}
        self.constructor = constructor

    def read(self, index):
        if index in self.memo:
            return self.memo[index]
        tag, *args = self.nodes[index]
        if tag == "Q":
            result = self.constructor(Fraction(*args))
        elif tag in ("Add", "Mul"):
            result = self.constructor(0 if tag == "Add" else 1)
            for child in args:
                result = result.add(self.read(child)) if tag == "Add" else result.mul(self.read(child))
        elif tag == "Pow":
            exponent = self.read(args[1]).value.as_rational()
            if exponent is None or exponent.denominator not in (1, 2):
                raise ImportedNumericProfileUnsupported("power outside numeric integer/half-integer profile")
            base = self.read(args[0])
            result = (base if exponent.denominator == 1 else base.sqrt()).integer_power(exponent.numerator)
        else:
            raise ImportedNumericProfileUnsupported(f"opaque source node: {tag}")
        self.memo[index] = result
        return result


def _dump(value):
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True)


def _write(stream, value):
    stream.write(_dump(value) + "\n")


def _tree(nodes, index):
    tag, *args = nodes[index]
    return (tag, *args) if tag == "Q" else (tag, *(_tree(nodes, child) for child in args))


def _shape_tree(coefficient, radicand):
    if radicand == 1:
        return ("Q", coefficient.numerator, coefficient.denominator)
    root = ("Pow", ("Q", radicand, 1), ("Q", 1, 2))
    return root if coefficient == 1 else ("Mul", ("Q", coefficient.numerator, coefficient.denominator), root)


def _record(scalar):
    literal = scalar.literal_ast()
    if literal.canon != DENSITY_SOURCE_CONSTRUCTION_V1:
        raise ValueError("source construction canon mismatch")
    if _NumericReader(literal.nodes).read(literal.root).value != scalar.value:
        raise AssertionError("literal AST exact value differs from its scalar")
    return {"canon": literal.canon, "v2_wire": scalar.value.wire(),
            "nodes": literal.nodes, "root": literal.root}


def _roots(path, out, runner=None):
    counts = Counter()
    with path.open(encoding="utf-8") as source, (out/"roots.jsonl").open("w", encoding="ascii") as records:
        with (out/"root-literal-deltas.jsonl").open("w", encoding="ascii") as deltas:
            for index, line in enumerate(source):
                row = json.loads(line)
                argument = Fraction(row["rational"])
                if runner:
                    runner.reset()
                root = (runner.rational(argument).sqrt() if runner else ScalarV2.sqrt_rational(argument))
                coefficient = Fraction(row["source"]["coefficient"])
                radicand = int(row["source"]["integer_radicand"])
                if coefficient*coefficient*radicand != argument:
                    raise AssertionError("recorded source root shape has wrong exact square")
                old = ScalarV2.sqrt_rational(radicand).scaled(coefficient)
                if root != old or root**2 != ScalarV2.rational(argument):
                    raise AssertionError("root construction differs from exact source value")
                candidate = _record(root)
                old_tree = _shape_tree(coefficient, radicand)
                new_tree = _tree(candidate["nodes"], candidate["root"])
                changed = old_tree != new_tree
                document = {"record": index, "argument": [argument.numerator, argument.denominator],
                            "calls_inferred": row["count"], "categories": row["categories"],
                            "candidate": candidate, "old_source_srepr": row["source_srepr"],
                            "old_literal": old_tree, "old_ast_changed": changed,
                            "exact_value_and_square_verified": True}
                if argument:
                    inverse = root.inverse()
                    if root*inverse != ScalarV2.rational(1) or inverse**2 != ScalarV2.rational(1/argument):
                        raise AssertionError("generated root reciprocal recipe failed")
                    document["generated_inverse"] = _record(inverse)
                _write(records, document)
                counts["root_arguments"] += 1
                counts["root_calls_inferred"] += row["count"]
                counts["old_ast_changed_arguments" if changed else "old_ast_same_arguments"] += 1
                counts["old_ast_changed_calls" if changed else "old_ast_same_calls"] += row["count"]
                if changed:
                    _write(deltas, {**document, "code": "DENSITY_SOURCE_LITERAL_AST_CHANGED",
                                    "claim": "Exact value preserved; old AST identity changed, not automatically accepted."})
    return {"file": str(path), "sha256": _sha(path), "counts": dict(counts)}


def _entries(value):
    if value[0] != "map":
        raise ValueError("expected corpus map")
    return value[1]


def _vector_ids(vector, before, after):
    if vector[0] == "vector_expr":
        return vector[1:]
    if vector[0] != "vector_text":
        raise ValueError("unsupported vector wire")
    lookup = {}
    for metric in (before, after):
        for name, value in _entries(metric["base"]["memo"]["expressions"]):
            if isinstance(value, list) and value[0] == "expr":
                lookup[name] = value[1]
    return [lookup[name] for name in vector[1:]]


def _dot(gram, left, right):
    return left[0]*(gram[0][0]*right[0]+gram[0][1]*right[1])+left[1]*(gram[1][0]*right[0]+gram[1][1]*right[1])


def _unit(vector, squared):
    length = squared.sqrt()
    return tuple(component/length for component in vector)


def _recipe(gram, incoming, outgoing, owner_sign, count):
    left_squared, right_squared = _dot(gram, incoming, incoming), _dot(gram, outgoing, outgoing)
    facts = {"incoming_squared": left_squared, "outgoing_squared": right_squared,
             "raw_dot": _dot(gram, incoming, outgoing),
             "raw_cross": (incoming[0]*outgoing[1]-incoming[1]*outgoing[0]).scaled(owner_sign)}
    left_unit, right_unit = _unit(incoming, left_squared), _unit(outgoing, right_squared)
    facts.update(incoming_x=left_unit[0], incoming_y=left_unit[1], outgoing_x=right_unit[0], outgoing_y=right_unit[1])
    one = _constant(incoming[0], 1)
    if _dot(gram, left_unit, left_unit) != one or _dot(gram, right_unit, right_unit) != one:
        raise AssertionError("generated unit vector does not have exact Gram norm one")
    if count == 0:
        return facts
    cosine_squared = facts["raw_dot"]**2/(left_squared*right_squared)
    if cosine_squared.value.as_rational() is None:
        raise ImportedNumericProfileUnsupported("generated principal cosine squared is not rational")
    facts.update(cosine_squared=cosine_squared, cosine_root=cosine_squared.sqrt(),
                 sine_root=(one-cosine_squared).sqrt())
    if facts["cosine_root"]**2+facts["sine_root"]**2 != one:
        raise AssertionError("principal root squared identity failed")
    covector = (gram[0][0]*left_unit[0]+gram[0][1]*left_unit[1],
                gram[1][0]*left_unit[0]+gram[1][1]*left_unit[1])
    normal = (covector[1].scaled(-owner_sign), covector[0].scaled(owner_sign))
    if not _dot(gram, left_unit, normal).value.is_zero:
        raise AssertionError("generated Gram normal is not exactly orthogonal")
    normal_unit = _unit(normal, _dot(gram, normal, normal))
    if _dot(gram, normal_unit, normal_unit) != one:
        raise AssertionError("generated unit normal does not have exact Gram norm one")
    facts.update(basis_0=left_unit[0], basis_1=left_unit[1], basis_2=normal_unit[0], basis_3=normal_unit[1])
    unit_cross = left_unit[0]*right_unit[1]-left_unit[1]*right_unit[0]
    determinant = gram[0][0]*gram[1][1]-gram[0][1]*gram[1][0]
    if unit_cross**2*determinant+cosine_squared != one:
        raise AssertionError("Gram dot/cross determinant identity failed")
    facts["unit_cross"] = unit_cross
    return facts


def _rotation_facts(rotation, count, facts, reader):
    if rotation is None or count == 0:
        return
    def number(value):
        if type(value) is int:
            return _constant(facts["basis_0"], value)
        if value[0] == "fraction":
            return _constant(facts["basis_0"], Fraction(*value[1:]))
        if value[0] == "expr":
            return reader.read(value[1])
        raise ImportedNumericProfileUnsupported("rotation coefficient outside numeric profile")
    for ordinal, pair in enumerate(rotation[1][:count], 1):
        a, b = map(number, pair[1])
        facts[f"table_a_{ordinal}"] = a
        facts[f"table_b_{ordinal}"] = b
        length = (a**2+b**2).sqrt()
        facts[f"table_length_{ordinal}"] = length
        for axis in range(2):
            facts[f"table_ray_{ordinal}_{axis}"] = (a*facts[f"basis_{axis}"]+b*facts[f"basis_{axis+2}"])/length


def _old_facts(row, incoming_ids, outgoing_ids):
    key = ["tuple", [["expr", index] for index in incoming_ids+outgoing_ids] + [row["input"]["orientation"]]]
    atoms = _entries(row["outcome"]["memo_after"]["turn"]["turn_atoms"])
    body = next((value[1] for atom_key, value in atoms if atom_key == key), None)
    if body is None:
        return {}
    result = {name: body[name] for name in ("incoming_squared", "outgoing_squared", "raw_dot", "cosine_squared")
              if body[name] is not None}
    for name in ("incoming", "outgoing"):
        if body[name] is not None and body[name][0] == "vector_expr":
            result.update({name+"_"+axis: ["expr", index] for axis, index in zip(("x", "y"), body[name][1:])})
    if body["basis"] is not None:
        result.update({f"basis_{index}": value for index, value in enumerate(body["basis"][1])})
    return result


def _compare_fact(key, name, generated, source, reader, delta_stream):
    if source[0] != "expr":
        raise ValueError("expected generated source fact expression")
    old = reader.read(source[1])
    if old != generated:
        raise AssertionError(f"fresh generated scalar differs from recorded fact {name}")
    old_tree = _tree(reader.nodes, source[1])
    literal = generated.literal_ast()
    new_tree = _tree(literal.nodes, literal.root)
    if old_tree != new_tree:
        _write(delta_stream, {**key, "operation": name, "code": "DENSITY_SOURCE_LITERAL_AST_CHANGED",
                              "old_node": source[1], "old_literal": old_tree, "candidate": _record(generated),
                              "claim": "Exact recorded value preserved; changed literal is not old-AST acceptance."})
        return True
    return False


def _forest_record(row, nodes, streams, counts, filename, runner=None):
    key = {"file": filename, "sequence": row["sequence"], "test": row["test"]}
    counts["records"] += 1
    try:
        if runner:
            runner.reset()
        reader = _NumericReader(nodes, runner.rational if runner else ScalarV2.rational)
        body = row["input"]
        before, after = body["metric"], row["outcome"]["memo_after"]
        count = body["count"]
        if isinstance(count, list) and count[0] == "expr":
            rational_count = reader.read(count[1]).value.as_rational()
            count = (rational_count.numerator if rational_count is not None
                     and rational_count.denominator == 1 else None)
        if type(count) is not int or count not in range(6):
            raise ImportedNumericProfileUnsupported("request count outside declared 0..5 recipe")
        incoming_ids, outgoing_ids = (_vector_ids(body[name], before, after) for name in ("incoming", "outgoing"))
        incoming, outgoing = (tuple(reader.read(index) for index in ids) for ids in (incoming_ids, outgoing_ids))
        gram = tuple(tuple(reader.read(index) for index in line) for line in before["base"]["gram"])
        facts = _recipe(gram, incoming, outgoing, before["base"]["sign"], count)
        _rotation_facts(body["rotation"], count, facts, reader)
        record = {**key, "canon": DENSITY_SOURCE_CONSTRUCTION_V1,
                  "recipe": "fresh Gram dot/cross -> norm root -> units -> principal squared-root facts/basis/table numeric rays",
                  "inputs": {"gram": [[_record(value) for value in line] for line in gram],
                             "incoming": [_record(value) for value in incoming],
                             "outgoing": [_record(value) for value in outgoing],
                             "owner_sign": before["base"]["sign"], "count": count},
                  "facts": {name: _record(value) for name, value in facts.items()}}
        _write(streams["forest"], record)
        counts["closed_numeric_recipe_records"] += 1
        for name, source in _old_facts(row, incoming_ids, outgoing_ids).items():
            if name not in facts:
                continue
            changed = _compare_fact(key, name, facts[name], source, reader, streams["forest-literal-deltas"])
            counts["recorded_generated_facts_value_verified"] += 1
            counts["recorded_generated_fact_old_ast_deltas"] += int(changed)
    except (DensityScalarProfileUnsupported, ImportedNumericProfileUnsupported) as error:
        counts["profile_gap_records"] += 1
        _write(streams["profile-gaps"], {**key, "code": str(getattr(error.code, "value", error.code)),
                                       "detail": str(error), "recorded_source_outcome": row["outcome"]["answer"]})
    except ZeroDivisionError as error:
        counts["exact_zero_division_records"] += 1
        _write(streams["numeric-failures"], {**key, "code": "DENSITY_SCALAR_V2_EXACT_ZERO_DIVISION",
                                           "type": type(error).__name__, "detail": str(error),
                                           "recorded_source_outcome": row["outcome"]["answer"],
                                           "claim": "Arithmetic failure propagated by candidate; not unsupported or legacy geometry refusal."})
    except Exception as error:
        counts["failure_records"] += 1
        _write(streams["failures"], {**key, "code": "DENSITY_SCALAR_V2_RECIPE_FAILURE",
                                   "type": type(error).__name__, "detail": str(error)})


def _forest(path, streams, runner=None):
    document = json.loads(lzma.decompress(path.read_bytes()))
    if document["schema"] != "cftuv.native-density-turn-corpus.v1":
        raise ValueError("expected native-density-turn-corpus.v1")
    counts = Counter()
    for index, row in enumerate(document["records"]):
        _forest_record(row, document["expressions"], streams, counts, path.name, runner)
        if (index+1) % 250 == 0:
            print(_dump({"forest_progress": path.name, "done": index+1,
                         "records": len(document["records"]), "counts": dict(counts),
                         "rust_operations_compared": runner.compared if runner else None}), flush=True)
    print(_dump({"forest_complete": path.name, "counts": dict(counts)}), flush=True)
    return {"file": str(path), "sha256": _sha(path), "counts": dict(counts)}


def _state_probe(mode):
    # Эта функция запускается только отдельным --state-probe child, не общим audit/pytest.
    import sympy as sp
    from sympy.core.cache import clear_cache
    from sympy.ntheory.factor_ import factor_cache
    p, q = 2**89-1, 2**61-1
    n = p*p*q
    clear_cache()
    factor_cache.clear()
    if mode == "warm":
        factor_cache[n] = q
    source = sp.sqrt(sp.Integer(n))
    candidate = ScalarV2.sqrt_rational(n)
    if candidate != ScalarV2.sqrt_rational(q).scaled(p):
        raise AssertionError("cache witness scalar differs by value")
    return {"pid": os.getpid(), "mode": mode, "source_srepr": sp.srepr(source), "candidate": _record(candidate)}


def _cache_witness():
    results = []
    for mode in ("cold", "warm"):
        # CLI сохраняет argv, но не может выбрать другое ядро через PYTHONPATH.
        code = '''
import runpy
from pathlib import Path
saved_argv = list(sys.argv)
script, mode = sys.argv[2:4]
sys.path.insert(0, str(Path(script).parent))
try:
    sys.argv = [script, "--state-probe", mode]
    try:
        runpy.run_path(script, run_name="__main__")
    except SystemExit as error:
        if error.code not in (None, 0):
            raise
finally:
    sys.argv = saved_argv
'''
        command, environment = kernel_child(code, str(Path(__file__).resolve()), mode)
        process = subprocess.Popen(command, env=environment,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                   creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        log = os.environ.get("CFTUV_DENSITY_PID_LOG")
        if log:
            with Path(log).open("a", encoding="utf-8") as stream:
                _write(stream, {"pid": process.pid, "task": "scalar-v2-isolated-cache-witness", "mode": mode})
        stdout, stderr = process.communicate(timeout=45)
        if process.returncode or stderr:
            raise RuntimeError(f"isolated cache witness failed: {stderr}")
        results.append(json.loads(stdout))
    if results[0]["candidate"] != results[1]["candidate"] or results[0]["source_srepr"] == results[1]["source_srepr"]:
        raise AssertionError("isolated source-state witness did not establish independent candidate")
    return {"scope": "Synthetic proven-divisor state injection in isolated children; no production reachability claim.",
            "code": "DENSITY_SOURCE_LITERAL_AST_CHANGED_BY_AMBIENT_STATE",
            "candidate_value_and_literal_identical": True, "states": results}


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roots", type=Path)
    parser.add_argument("--file", type=Path, action="append", default=[])
    parser.add_argument("--decision", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--state-probe", choices=("cold", "warm"))
    parser.add_argument("--exe", type=Path)
    args = parser.parse_args(argv)
    sys.set_int_max_str_digits(0)
    if args.state_probe:
        print(_dump(_state_probe(args.state_probe)))
        return 0
    if args.roots is None or args.out is None or args.decision is None:
        parser.error("--roots, --decision and --out are required for the audit")
    decision = json.loads(args.decision.read_text(encoding="utf-8"))
    if decision["proposed_canon_id"] != DENSITY_SOURCE_CONSTRUCTION_V1:
        raise ValueError("architecture decision canon mismatch")
    started = time.monotonic()
    source_paths = (Path(__file__).resolve(), PACKAGE_ROOT/"reference/density_scalar_v2.py",
                    PACKAGE_ROOT/"reference/radical_v2.py")
    source_files = [{"path": str(path), "sha256": _sha(path)} for path in source_paths]
    args.out.mkdir(parents=True, exist_ok=True)
    runner = _RustRunner(args.exe, args.out) if args.exe else None
    roots = _roots(args.roots, args.out, runner)
    streams = {name: (args.out/(name+".jsonl")).open("w", encoding="ascii")
               for name in ("forest", "forest-literal-deltas", "profile-gaps", "numeric-failures", "failures")}
    try:
        forests = [_forest(path, streams, runner) for path in args.file]
    finally:
        for stream in streams.values():
            stream.close()
    rust = runner.close() if runner else None
    witness = _cache_witness()
    (args.out/"cache-state-witness.json").write_text(json.dumps(witness, indent=2), encoding="utf-8")
    totals = Counter()
    for forest in forests:
        totals.update(forest["counts"])
    receipt = {"schema": "cftuv.density-scalar-v2-diff.v1", "canon": DENSITY_SOURCE_CONSTRUCTION_V1,
               "status": "FAILED" if totals["failure_records"] else "COMPLETED", "pid": os.getpid(),
               "scope": "First numeric scalar-island layer only; actual Rust scalar parity is reported separately when exercised. No opaque ring/full constructor, legacy sign/controller/admission parity, full consumer or R3 completion claim.",
               "construction_contract": "Canonical actual RadicalSumV2 arithmetic. Literal terms increasing mprime; coefficient before root; identity omitted; zero Q0/1; child-first first-occurrence local interning.",
               "old_ast_delta_policy": "Changed old literals are named and retained; exact value equality does not bless legacy AST/outcome identity.",
               "numeric_failure_policy": "Exact zero division is separately named, not unsupported. Forest zero-vector and other unclosed inputs remain outside successful recipe scope. Internal unexpected failures make the receipt FAILED.",
               "root_source": roots, "forest_sources": forests, "forest_totals": dict(totals),
               "cache_state_witness": witness,
               "actual_rust_operation_parity": rust,
               "decision": {"path": str(args.decision), "sha256": _sha(args.decision)},
               "source_files": source_files,
               "elapsed_seconds_diagnostic_only": time.monotonic()-started}
    receipt["source_changed_during_run"] = [entry["path"] for entry in source_files
                                            if _sha(Path(entry["path"])) != entry["sha256"]]
    if receipt["source_changed_during_run"]:
        receipt["status"] = "FAILED"
    receipt["artifacts"] = [{"path": str(path), "sha256": _sha(path), "bytes": path.stat().st_size}
                            for path in sorted(args.out.glob("*.jsonl"))]
    (args.out/"receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(_dump({"receipt": str(args.out/"receipt.json"), "status": receipt["status"],
                 "roots": roots["counts"], "forest": dict(totals)}), flush=True)
    return int(receipt["status"] != "COMPLETED")


if __name__ == "__main__":
    raise SystemExit(main())
