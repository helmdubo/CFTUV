"""Fresh owned-consumer CANDIDATE audit; no product or sign-controller hook.

Source numeric facts are inputs. Every generated arithmetic/opaque operation is
performed afresh, and optionally replayed in the actual Rust arena. Old literals
and old/staged decisions are observations, never old-source admission acceptance.
"""

from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
from itertools import permutations
import lzma
from math import isqrt
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
sys.path.insert(0, str(ROOT / "tools"))

from cftuv_envelope.reference.density_construct_v1 import (  # noqa: E402
    DensityConstructArenaV1, DensityConstructProfileUnsupported, OwnedVec2V1,
    ExpandLimitsV1,
    owned_cross, owned_dot, owned_oriented_basis, owned_rational_norm_unit,
    owned_principal_linear_combination,
)
from cftuv_envelope.reference.density_scalar_v2 import (  # noqa: E402
    DENSITY_SOURCE_CONSTRUCTION_V1, DensityScalarProfileUnsupported, ScalarV2,
)
from density_scalar_v2_diff import (  # noqa: E402
    ImportedNumericProfileUnsupported, _NumericReader, _entries, _old_facts, _tree, _vector_ids,
)


def _dump(value):
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"))


def _write(stream, value):
    stream.write(_dump(value)+"\n")


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _record(arena, expr):
    literal = arena.export_raw(expr)
    scalar = arena.scalar(expr)
    return {"canon": literal.canon, "nodes": [list(row) for row in literal.nodes], "root": literal.root,
            "numeric_v2_wire": None if scalar is None else list(scalar.value.wire())}


class _RustRunner:
    def __init__(self, executable, out):
        self.executable = Path(executable)
        self.stderr = (out/"rust.stderr.log").open("w", encoding="utf-8")
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.stderr, text=True, bufsize=1,
                                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        _log_pid(self.process.pid, "construct-v1-actual-rust")
        self.commands = self.compared = self.errors = self.registers = self.sign_compared = 0

    def request(self, command):
        self.process.stdin.write(command+"\n")
        self.process.stdin.flush()
        self.commands += 1
        response = self.process.stdout.readline().rstrip("\r\n")
        if not response:
            raise RuntimeError("Rust construction process ended without response")
        return response

    def clear(self):
        if self.request("clear") != "CLEARED":
            raise AssertionError("Rust clear response mismatch")
        self.registers = 0

    def operation(self, arena, command, calculate):
        try:
            expr = calculate()
        except (DensityConstructProfileUnsupported, DensityScalarProfileUnsupported, ZeroDivisionError) as error:
            response = self.request(command)
            if isinstance(error, ZeroDivisionError):
                valid = response.startswith("INTERNAL\t") and "ZERO" in response.upper()
            else:
                valid = response.split("\t")[:2] == ["PROFILE", error.code.value]
            if not valid:
                raise AssertionError(f"Rust error differs: {response}; Python: {error}") from error
            self.errors += 1
            raise
        response = self.request(command)
        fields = response.split("\t", 2)
        if len(fields) != 3 or fields[0] != "EXPR" or int(fields[1]) != self.registers:
            raise AssertionError(f"Rust expression/register differs: {response}")
        expected, actual = _record(arena, expr), json.loads(fields[2])
        if expected != actual:
            raise AssertionError(_dump({"command": command, "expected": expected, "actual": actual}))
        index = self.registers
        self.registers += 1
        self.compared += 1
        return expr, index

    def close(self):
        self.process.stdin.close()
        exit_code = self.process.wait(timeout=45)
        self.stderr.close()
        if exit_code:
            raise RuntimeError(f"Rust construction process exited {exit_code}")
        return {"executable": str(self.executable), "sha256": _sha(self.executable), "exit_code": exit_code,
                "stderr_bytes": Path(self.stderr.name).stat().st_size, "commands": self.commands,
                "successful_fresh_operation_values_and_literal_views_compared": self.compared,
                "expected_internal_profile_or_numeric_errors_compared": self.errors,
                "legacy_lazy_sign_decisions_compared": self.sign_compared,
                "native_import_parity": "OUTSIDE_THIS_GATE: only fresh operations and owned consumers are replayed"}


class _PairedArena:
    """Harness facade: native evaluates the same fresh caller operation.

    The Python arena's internal normalization is not replayed as postvalue
    numeric imports. Generated results are compared to actual native operations.
    Only admitted source facts use the explicit numeric input transport.
    """

    def __init__(self, runner=None):
        self.arena, self.runner, self.registers = DensityConstructArenaV1(), runner, {}
        self.sign_memo = {}
        if runner:
            runner.clear()

    def __getattr__(self, name):
        method = getattr(self.arena, name)
        op = {"rational": "q", "numeric": "numeric", "pi": "pi", "raw_sin": "sin",
              "raw_cos": "cos", "raw_atan2": "atan2", "integer_power": "pow", "finite_expand": "expand"}.get(name, name)
        if name not in {"rational", "numeric", "pi", "raw_sin", "raw_cos", "raw_atan2",
                        "add", "mul", "sub", "neg", "scaled", "integer_power", "raw_pow", "sqrt", "div",
                        "reciprocal", "partial_div", "finite_expand"}:
            return method

        def call(*args):
            if not self.runner:
                return method(*args)
            fields = [op]
            if name == "rational":
                rational = Fraction(args[0])
                fields += [str(rational.numerator), str(rational.denominator)]
            elif name == "numeric":
                fields.append(",".join(map(str, args[0].value.wire())))
            elif name in ("scaled", "raw_pow"):
                rational = Fraction(args[1])
                fields += [str(self.registers[args[0].node]), str(rational.numerator), str(rational.denominator)]
            elif name == "integer_power":
                fields += [str(self.registers[args[0].node]), str(args[1])]
            elif name == "finite_expand":
                limits = args[1]
                fields += [str(self.registers[args[0].node]), str(limits.max_terms),
                           str(limits.max_polynomial_exponent), str(limits.max_rational_power_bits)]
            else:
                fields += [str(self.registers[arg.node]) for arg in args]
            expr, index = self.runner.operation(self.arena, "\t".join(fields), lambda: method(*args))
            self.registers[expr.node] = index
            return expr
        return call


def _log_pid(pid, task):
    log = os.environ.get("CFTUV_DENSITY_PID_LOG")
    if log:
        with Path(log).open("a", encoding="utf-8") as stream:
            _write(stream, {"pid": pid, "task": task})


def _legacy_staged_probe(expression, allow_unexpanded_reader=True):
    """Existing iv160 and reader only; no SymPy expansion after export.

    NEEDS_OWNED_EXPANDED_VIEW is not legacy UNDECIDABLE. No V2 sign or A2
    value enclosure participates. Old recorded decisions are separate evidence.
    """
    import sympy as sp
    from mpmath import iv
    from cftuv_envelope._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
    from cftuv_envelope.reference.planar_types import ExactQuadraticFieldUnsupported, exact_quadratic_value

    if isinstance(expression, sp.Rational):
        return {"stage": "RATIONAL", "sign": (expression.p > 0)-(expression.p < 0)}
    if expression == 0:
        return {"stage": "STRUCTURAL_ZERO", "sign": 0}
    saved = iv.prec
    iv.prec = 160
    try:
        try:
            interval = density_interval_enclosure(expression)
        except (DensityIntervalEnclosureUnsupported, ArithmeticError, TypeError, ValueError):
            interval = None
        if interval is not None and interval.a > 0:
            return {"stage": "ORDERED_IV160", "sign": 1}
        if interval is not None and interval.b < 0:
            return {"stage": "ORDERED_IV160", "sign": -1}
    finally:
        iv.prec = saved
    if not allow_unexpanded_reader:
        return {"stage": "NEEDS_OWNED_EXPANDED_VIEW", "sign": None}
    try:
        return {"stage": "UNEXPANDED_OLD_QUADRATIC_READER", "sign": exact_quadratic_value(expression).sign()}
    except ExactQuadraticFieldUnsupported:
        return {"stage": "NEEDS_OWNED_EXPANDED_VIEW", "sign": None}


def _old_readonly(nodes, index, memo):
    import sympy as sp

    if index in memo:
        return memo[index]
    tag, *args = nodes[index]
    if tag == "Q":
        result = sp.Rational(*args)
    elif tag == "pi":
        result = sp.pi
    else:
        constructors = {"Add": sp.Add, "Mul": sp.Mul, "Pow": sp.Pow, "sin": sp.sin,
                        "cos": sp.cos, "atan2": sp.atan2, "atan": sp.atan, "tan": sp.tan}
        result = constructors[tag](*(_old_readonly(nodes, child, memo) for child in args), evaluate=False)
    memo[index] = result
    return result


def _numeric_recipe(arena, gram, incoming, outgoing, owner_sign, count):
    squared_left = owned_dot(arena, gram, incoming, incoming)
    squared_right = owned_dot(arena, gram, outgoing, outgoing)
    dot, cross = owned_dot(arena, gram, incoming, outgoing), owned_cross(arena, incoming, outgoing, owner_sign)
    left = owned_rational_norm_unit(arena, gram, incoming)
    right = owned_rational_norm_unit(arena, gram, outgoing)
    facts = {"incoming_squared": squared_left, "outgoing_squared": squared_right, "raw_dot": dot,
             "raw_cross": cross, "incoming_x": left.x, "incoming_y": left.y, "outgoing_x": right.x, "outgoing_y": right.y}
    one = arena.rational(1)
    if arena.whole_rational(owned_dot(arena, gram, left, left)) != 1 or arena.whole_rational(owned_dot(arena, gram, right, right)) != 1:
        raise AssertionError("generated unit Gram norm differs from one")
    if count == 0:
        return facts, None
    cosine_squared = arena.div(arena.integer_power(dot, 2), arena.mul(squared_left, squared_right))
    rational = arena.whole_rational(cosine_squared)
    if rational is None:
        raise ImportedNumericProfileUnsupported("generated principal cosine squared is non-rational")
    if not 0 <= rational < 1:
        raise ImportedNumericProfileUnsupported("principal cosine squared outside strict 0<=r<1 recipe")
    cosine, sine = arena.sqrt(cosine_squared), arena.sqrt(arena.sub(one, cosine_squared))
    basis, normal = owned_oriented_basis(arena, gram, incoming, owner_sign)
    if arena.whole_rational(owned_dot(arena, gram, normal, normal)) != 1:
        raise AssertionError("generated normal unit Gram norm differs from one")
    if arena.whole_rational(owned_dot(arena, gram, basis, normal)) != 0:
        raise AssertionError("generated basis is not orthogonal")
    facts.update(cosine_squared=cosine_squared, cosine_root=cosine, sine_root=sine,
                 basis_0=basis.x, basis_1=basis.y, basis_2=normal.x, basis_3=normal.y)
    return facts, (basis, normal)


def _number(arena, value, reader):
    if type(value) is int:
        return arena.rational(value)
    if value[0] == "fraction":
        return arena.rational(Fraction(*value[1:]))
    if value[0] == "expr":
        return arena.numeric(reader.read(value[1]))
    raise ImportedNumericProfileUnsupported("source coefficient outside numeric input profile")


def _rays(arena, body, facts, basis, reader, count):
    if not count:
        return [], "ENDPOINTS_ONLY"
    incoming, normal = basis
    orientation = body["orientation"]
    if type(orientation) is not int or orientation not in (-1, 1):
        raise ImportedNumericProfileUnsupported("orientation outside +/-1 source fact profile")
    signed_normal = OwnedVec2V1(arena.scaled(normal.x, orientation), arena.scaled(normal.y, orientation))
    rotation = body["rotation"]
    if rotation is not None:
        rays = []
        for ordinal, pair in enumerate(rotation[1][:count], 1):
            a, b = (_number(arena, value, reader) for value in pair[1])
            length = arena.sqrt(arena.add(arena.integer_power(a, 2), arena.integer_power(b, 2)))
            rays.append(OwnedVec2V1(arena.div(arena.add(arena.mul(a, incoming.x), arena.mul(b, signed_normal.x)), length),
                                    arena.div(arena.add(arena.mul(a, incoming.y), arena.mul(b, signed_normal.y)), length)))
        if len(rays) != count:
            raise ImportedNumericProfileUnsupported("rotation coefficient table shorter than requested count")
        return rays, "NUMERIC_TABLE_LINEAR_COMBINATION"
    canonical = body["canonical"]
    if canonical is None:
        probe = _legacy_staged_probe(arena.to_sympy_readonly(facts["raw_dot"]))
        if probe["sign"] is None:
            raise ImportedNumericProfileUnsupported("old staged reader needs owned expanded dot view")
        signed_cosine = arena.scaled(facts["cosine_root"], probe["sign"])
        angle = arena.raw_atan2(facts["sine_root"], signed_cosine)
    else:
        fraction = Fraction(*canonical[1:])
        angle = arena.scaled(arena.pi(), fraction)
    return [owned_principal_linear_combination(arena, arena.scaled(angle, Fraction(ordinal, count+1)),
                                               incoming, signed_normal) for ordinal in range(1, count+1)], "OPAQUE_PRINCIPAL_LINEAR_COMBINATION"


def _old_decisions(row):
    result = {}
    for key, sign in _entries(row["outcome"]["memo_after"]["base"]["memo"]["signs"]):
        if isinstance(key, list) and key[0] == "expr":
            result[key[1]] = sign
    return result


def _compare_fact(key, name, generated, source_id, arena, reader, streams, counts, row, old_memo):
    old_tree, new_tree = _tree(reader.nodes, source_id), arena.literal_tree(generated)
    scalar = arena.scalar(generated)
    if scalar is not None:
        if reader.read(source_id) != scalar:
            raise AssertionError(f"fresh generated exact value differs from old fact {name}")
        counts["recorded_numeric_facts_value_verified"] += 1
    if old_tree != new_tree:
        counts["old_literal_ast_deltas"] += 1
        _write(streams["old-literal-deltas"], {**key, "name": name, "code": "DENSITY_SOURCE_LITERAL_AST_CHANGED",
                                               "old_node": source_id, "old_literal": old_tree,
                                               "candidate": _record(arena, generated), "value_check": scalar is not None})
    candidate_probe = _legacy_staged_probe(arena.to_sympy_readonly(generated))
    old_probe = _legacy_staged_probe(_old_readonly(reader.nodes, source_id, old_memo))
    recorded_sign = _old_decisions(row).get(source_id)
    counts["staged_decision_pairs_observed"] += 1
    if old_probe != candidate_probe or (recorded_sign is not None and recorded_sign != candidate_probe["sign"]):
        counts["old_admission_observation_deltas"] += 1
        _write(streams["old-admission-deltas"], {**key, "name": name, "old_node": source_id,
                "recorded_old_controller_sign": recorded_sign, "old_readonly_staged_probe": old_probe,
                "candidate_readonly_staged_probe": candidate_probe,
                "code": "DENSITY_SOURCE_DECISION_OBSERVATION_CHANGED",
                "claim": "No controller/refusal parity acceptance; finite-expanded opaque reader remains outside this first slice."})


def _row(row, nodes, filename, streams, counts, runner, limits=None):
    key = {"file": filename, "sequence": row["sequence"], "test": row["test"]}
    counts["records"] += 1
    try:
        arena, reader = _PairedArena(runner), _NumericReader(nodes)
        body = row["input"]
        before, after = body["metric"], row["outcome"]["memo_after"]
        counts[f"owner_{before['base']['sign']}_turn_{body['orientation']}_source_records"] += 1
        count = body["count"]
        if isinstance(count, list) and count[0] == "expr":
            rational = reader.read(count[1]).value.as_rational()
            count = rational.numerator if rational is not None and rational.denominator == 1 else None
        if type(count) is not int or count not in range(6):
            raise ImportedNumericProfileUnsupported("source count outside 0..5 recipe")
        incoming_ids, outgoing_ids = (_vector_ids(body[name], before, after) for name in ("incoming", "outgoing"))
        incoming, outgoing = (OwnedVec2V1(*(arena.numeric(reader.read(index)) for index in ids))
                              for ids in (incoming_ids, outgoing_ids))
        grid = before["base"]["gram"]
        gram = tuple(arena.numeric(reader.read(index)) for index in (grid[0][0], grid[0][1], grid[1][1]))
        facts, basis = _numeric_recipe(arena, gram, incoming, outgoing, before["base"]["sign"], count)
        rays, recipe = _rays(arena, body, facts, basis, reader, count)
        expanded_rays = []
        if limits is not None:
            for ray in rays:
                expanded_rays.append(OwnedVec2V1(arena.finite_expand(ray.x, limits), arena.finite_expand(ray.y, limits)))
                counts["fresh_owned_ray_axes_expanded"] += 2
        counts["closed_owned_consumer_records"] += 1
        counts[recipe] += 1
        counts["fresh_owned_rays"] += len(rays)
        _write(streams["forest"], {**key, "recipe": recipe,
                "facts": {name: _record(arena, expr) for name, expr in facts.items()},
                "rays": [[_record(arena, ray.x), _record(arena, ray.y)] for ray in rays],
                "expanded_rays": [[_record(arena, ray.x), _record(arena, ray.y)] for ray in expanded_rays],
                "recorded_source_outcome": row["outcome"]["answer"][0]})
        old_memo = {}
        for name, source in _old_facts(row, incoming_ids, outgoing_ids).items():
            if name in facts:
                _compare_fact(key, name, facts[name], source[1], arena, reader, streams, counts, row, old_memo)
        answer = row["outcome"]["answer"]
        if answer[0] == "ok" and answer[1][0] == "tuple" and len(answer[1][1]) == count+2:
            for ordinal, (ray, source) in enumerate(zip(rays, answer[1][1][1:-1]), 1):
                if source[0] != "vector_expr":
                    continue
                for axis, expr, source_id in zip(("x", "y"), (ray.x, ray.y), source[1:]):
                    _compare_fact(key, f"ray_{ordinal}_{axis}", expr, source_id, arena, reader, streams, counts, row, old_memo)
    except (DensityConstructProfileUnsupported, DensityScalarProfileUnsupported, ImportedNumericProfileUnsupported) as error:
        counts["internal_profile_gap_records"] += 1
        _write(streams["profile-gaps"], {**key, "code": getattr(error.code, "value", error.code), "detail": str(error),
                                       "recorded_source_outcome": row["outcome"]["answer"]})
    except ZeroDivisionError as error:
        counts["exact_zero_division_records"] += 1
        _write(streams["numeric-failures"], {**key, "type": type(error).__name__, "detail": str(error),
                                            "code": "DENSITY_CONSTRUCT_V1_EXACT_ZERO_DIVISION",
                                            "recorded_source_outcome": row["outcome"]["answer"]})
    except Exception as error:
        counts["unexpected_failure_records"] += 1
        _write(streams["failures"], {**key, "type": type(error).__name__, "detail": str(error)})


def _forest(path, streams, runner, limit, expand_limits=None):
    document = json.loads(lzma.decompress(path.read_bytes()))
    if document["schema"] != "cftuv.native-density-turn-corpus.v1":
        raise ValueError("source schema mismatch")
    counts = Counter()
    for index, row in enumerate(document["records"][:limit]):
        _row(row, document["expressions"], path.name, streams, counts, runner, expand_limits)
        if (index+1) % 200 == 0:
            print(_dump({"progress": path.name, "done": index+1, "counts": dict(counts),
                         "rust_operations": None if runner is None else runner.compared}), flush=True)
    return {"file": str(path), "sha256": _sha(path), "counts": dict(counts)}


def _fixtures(runner, stream):
    arena = _PairedArena(runner)
    angle = arena.raw_atan2(arena.rational(1), arena.rational(2))
    sine, cosine = arena.raw_sin(angle), arena.raw_cos(angle)
    total = arena.add(sine, cosine)
    roots = [arena.sqrt(arena.rational(p)) for p in (2, 3, 5, 7)]
    island = arena.add(roots[0], roots[1])
    term = arena.mul(island, sine)
    raw_coefficient_add = arena.mul(arena.rational(2), total, arena.rational(3))
    expressions = [arena.mul(arena.rational(2), total), arena.add(term, term),
                   arena.mul(arena.rational(2), raw_coefficient_add),
                   arena.mul(arena.rational(Fraction(1, 6)), raw_coefficient_add),
                   arena.add(arena.mul(roots[0], sine), arena.mul(roots[1], sine)),
                   arena.mul(sine, sine, arena.integer_power(sine, 2)),
                   arena.mul(arena.sqrt(sine), arena.sqrt(sine)),
                   arena.integer_power(arena.integer_power(sine, 2), 3),
                   arena.raw_pow(arena.rational(9), Fraction(-3, 2)), arena.raw_sin(arena.rational(0))]
    three, four = arena.add(*roots[:3]), arena.add(*roots)
    expressions.append(arena.integer_power(three, -1))
    half = arena.sqrt(sine)
    odd = tuple(arena.integer_power(half, e) for e in (3, 5, 7))
    for factors in permutations(odd):
        result = arena.mul(*factors)
        assert result == arena.integer_power(half, 15)
        expressions.append(result)
    cube = odd[0]
    assert arena.mul(cube, cube) == arena.integer_power(sine, 3)
    for item in (term, arena.mul(roots[0], sine)):
        assert arena.whole_rational(arena.sub(item, item)) == 0
        assert arena.whole_rational(arena.add(arena.scaled(item, 2), arena.neg(item), arena.neg(item))) == 0
    errors = []
    for calculate in (lambda: arena.integer_power(four, -2), lambda: arena.raw_pow(four, -1),
                      lambda: arena.integer_power(sine, -1), lambda: arena.sqrt(arena.rational(-1)),
                      lambda: arena.raw_pow(sine, Fraction(-1, 2)), lambda: arena.integer_power(arena.rational(0), -1),
                      lambda: arena.integer_power(arena.rational(1), 2**20+1),
                      lambda: arena.mul(arena.integer_power(sine, 2**20), sine)):
        try:
            calculate()
            raise AssertionError("expected fixture boundary did not occur")
        except (DensityConstructProfileUnsupported, DensityScalarProfileUnsupported, ZeroDivisionError) as error:
            errors.append({"code": "EXACT_ZERO_DIVISION" if isinstance(error, ZeroDivisionError) else error.code.value})
    record = {"expressions": [_record(arena, expr) for expr in expressions], "expected_internal_errors": errors}
    _write(stream, record)
    return {"successful_results": len(expressions), "expected_errors": errors}


def _old_sign_outcome(expression):
    from cftuv_envelope.reference.density_sign import _density_exact_sign
    from cftuv_envelope.reference.common import ReferenceGeometryError
    try:
        return ["sign", _density_exact_sign(expression)]
    except ReferenceGeometryError as error:
        return ["error", error.outcome.value]


def _special_norms(runner, stream):
    """Actual old producer versus fresh owned finite recipes, including q5 law."""
    import sympy as sp
    from cftuv_envelope.reference.density_trig_v1 import owned_rational_pi_trig, owned_exact_q_trig
    from cftuv_envelope.reference import density_projective as old
    from cftuv_envelope.reference.metric import ExactPlanarMetric
    inventory = Path("D:/CFTUV-r3-density-corpus/canon-review/special-norm-inventory.json")
    rows = json.loads(inventory.read_text(encoding="utf-8"))["rows"]
    limits, counts = ExpandLimitsV1(4096, 4, 65536), Counter()
    for row in rows:
        angle = Fraction(row["angle"])
        theta = sp.Rational(angle.numerator, angle.denominator) * sp.pi
        oc, os = sp.cos(theta), sp.sin(theta)
        expected = _old_sign_outcome(oc**2 + os**2 - 1)
        arena = _PairedArena(runner)
        c, s = owned_rational_pi_trig(arena, angle)
        source = arena.sub(arena.add(arena.integer_power(c, 2), arena.integer_power(s, 2)), arena.rational(1))
        expanded = arena.finite_expand(source, limits)
        actual = _old_sign_outcome(arena.to_sympy_readonly(expanded))
        assert actual == expected, (row["angle"], expected, actual)
        if expected == ["sign", 0]:
            assert arena.whole_rational(expanded) == 0
            _paired_lazy_sign(arena, expanded, limits)
        elif runner:
            response = runner.request(f"lazy_sign\t{arena.registers[expanded.node]}\t4096\t4\t65536")
            assert response == "LEGACY\tUNDECIDABLE", response
            runner.sign_compared += 1
        assert sp.simplify(arena.to_sympy_readonly(c)-oc) == 0
        assert sp.simplify(arena.to_sympy_readonly(s)-os) == 0
        counts["catalog_rows"] += 1
        counts["old_exact_zero" if expected == ["sign", 0] else "old_undecidable"] += 1
        _write(stream, {"kind": "special_norm", "angle": row["angle"], "old": expected,
                        "owned": actual, "expanded": _record(arena, expanded)})
    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    metric = ExactPlanarMetric(identity, identity, 1)
    left = old._vector(sp.Integer(1), sp.Integer(0), metric)
    for turn in (-1, 1):
        rotated = old._rotate_q(metric, left, turn, 5)
        assert old._subturn_boundary(metric, left, rotated, 5)
        assert old._subturn(metric, left, rotated, 5)
        arena = _PairedArena(runner)
        c, s = owned_exact_q_trig(arena, 5)
        s = arena.scaled(s, turn)
        norm = arena.add(arena.mul(c, c), arena.mul(s, s))
        residual = arena.sub(arena.scaled(arena.mul(c, c), 8),
                             arena.mul(arena.add(arena.rational(3), arena.sqrt(arena.rational(5))), norm))
        expanded = arena.finite_expand(residual, limits)
        assert arena.whole_rational(norm) == 1 and arena.whole_rational(expanded) == 0
        assert _paired_lazy_sign(arena, expanded, limits)["sign"] == 0
        _write(stream, {"kind": "actual_q5_rotate_subturn_boundary", "turn_sign": turn,
                        "old_subturn": True, "old_boundary": True, "owned_boundary_residual": _record(arena, expanded)})
        counts["actual_q5_boundary_rows"] += 1
    return {"inventory": str(inventory), "inventory_sha256": _sha(inventory), "counts": dict(counts),
            "scope": "Finite producer norm and q5 boundary residual only; no generic trig identity or full controller parity."}


def _roots(path, stream, runner):
    counts = Counter()
    for line in path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        rational = Fraction(row["rational"])
        arena = _PairedArena(runner)
        root = arena.sqrt(arena.rational(rational))
        if arena.whole_rational(arena.integer_power(root, 2)) != rational:
            raise AssertionError("fresh owned root square differs from rational input")
        sine = arena.raw_sin(arena.rational(1))
        mixed = arena.mul(root, sine)
        counts["rational_arguments"] += 1
        counts["inferred_calls"] += row["count"]
        _write(stream, {"rational": str(rational), "root": _record(arena, root), "mixed": _record(arena, mixed)})
    return {"file": str(path), "sha256": _sha(path), "counts": dict(counts)}


def _finite_fixtures(runner, stream):
    arena = _PairedArena(runner)
    a = arena.raw_sin(arena.rational(1))
    b = arena.raw_cos(arena.rational(2))
    c = arena.raw_atan2(arena.rational(1), arena.rational(2))
    total = arena.add(a, b)
    reducible = arena.sub(arena.mul(a, total), arena.integer_power(a, 2))
    numeric_sum = arena.add(arena.sqrt(arena.rational(2)), arena.sqrt(arena.rational(3)), arena.rational(Fraction(2, 5)))
    reciprocal = arena.reciprocal(a)
    partial_scalar_term = arena.mul(numeric_sum, reciprocal)
    good_limits = ExpandLimitsV1(256, 16, 4096)
    sources = [arena.integer_power(total, 2), arena.mul(reducible, c), arena.integer_power(reducible, 2),
               arena.mul(numeric_sum, a), arena.mul(numeric_sum, total), reciprocal,
               arena.partial_div(a, b), arena.raw_sin(arena.rational(0))]
    successes, expected_errors = 0, 0
    for source in sources:
        expanded = arena.finite_expand(source, good_limits)
        _write(stream, {"kind": "finite_expansion", "source": _record(arena, source), "expanded": _record(arena, expanded)})
        successes += 1
    for cap in (0, 1, 2):
        limits = ExpandLimitsV1(cap, 16, 4096)
        for source in sources[:5]:
            try:
                expanded = arena.finite_expand(source, limits)
            except DensityConstructProfileUnsupported as error:
                _write(stream, {"kind": "tight_limit", "max_terms": cap, "code": error.code.value})
                expected_errors += 1
            else:
                _write(stream, {"kind": "tight_limit", "max_terms": cap, "expanded": _record(arena, expanded)})
                successes += 1
    opaque_child = arena.raw_sin(arena.mul(total, arena.add(a, c)))
    for calculate in (lambda: arena.finite_expand(opaque_child, good_limits),
                      lambda: arena.mul(arena.rational(0), reciprocal),
                      lambda: arena.sub(reciprocal, reciprocal), lambda: arena.integer_power(reciprocal, 0),
                      lambda: arena.sub(partial_scalar_term, partial_scalar_term),
                      lambda: arena.mul(arena.rational(0), arena.raw_cos(reciprocal)),
                      lambda: arena.partial_div(arena.rational(0), arena.raw_cos(arena.rational(0))),
                      lambda: arena.finite_expand(arena.integer_power(total, 2), ExpandLimitsV1(16, 1, 4096))):
        try:
            calculate()
            raise AssertionError("expected finite/partial boundary missing")
        except DensityConstructProfileUnsupported as error:
            _write(stream, {"kind": "expected_boundary", "code": error.code.value})
            expected_errors += 1
    lazy_child = arena.mul(arena.add(arena.pi(), arena.raw_cos(arena.rational(0))),
                           arena.add(arena.rational(Fraction(1, 4)), arena.scaled(arena.raw_sin(arena.rational(1)), Fraction(1, 4))))
    lazy_positive = arena.add(arena.rational(2), arena.raw_sin(lazy_child))
    lazy = [_paired_lazy_sign(arena, lazy_positive, ExpandLimitsV1(0, 0, 0)),
            _paired_lazy_sign(arena, lazy_positive, ExpandLimitsV1(0, 0, 0))]
    floor = isqrt(2 << 400)
    near_zero = arena.sub(arena.sqrt(arena.rational(2)), arena.rational(Fraction(floor, 1 << 200)))
    lazy.append(_paired_lazy_sign(arena, near_zero, good_limits))
    assert lazy[0] == {"sign": 1, "expanded": False, "path": "INTERVAL"}
    assert lazy[1] == {"sign": 1, "expanded": False, "path": "MEMO"}
    assert lazy[2] == {"sign": 1, "expanded": True, "path": "QUADRATIC"}
    _write(stream, {"kind": "lazy_legacy_sign", "observations": lazy,
                    "claim": "Only legacy decision view; no V2/A2 sign or enclosure substitutes admission."})
    return {"successes": successes, "expected_profile_boundaries": expected_errors,
            "lazy_legacy_sign_fixtures": lazy,
            "scope": "Fresh operations, Numeric sum boundary, reducible Add/product/power, transient limits0/1/2, opaque-child guard and partial-domain discard."}


def _paired_lazy_sign(arena, expr, limits):
    """Harness-only exact lazy controller order; no SymPy expand after view."""
    if expr.node in arena.sign_memo:
        expected = {"sign": arena.sign_memo[expr.node], "path": "MEMO", "expanded": False}
    else:
        probe = _legacy_staged_probe(arena.to_sympy_readonly(expr), allow_unexpanded_reader=False)
        if probe["sign"] is not None:
            path = "INTERVAL" if probe["stage"] == "ORDERED_IV160" else "RATIONAL"
            expected = {"sign": probe["sign"], "path": path, "expanded": False}
        else:
            from cftuv_envelope.reference.planar_types import exact_quadratic_value
            # Отдельный owned worker вызывается только после NeedsExpanded.
            expanded = arena.arena.finite_expand(expr, limits)
            sign = exact_quadratic_value(arena.to_sympy_readonly(expanded)).sign()
            expected = {"sign": sign, "path": "QUADRATIC", "expanded": True}
        if expected["path"] != "RATIONAL":
            arena.sign_memo[expr.node] = expected["sign"]
    if arena.runner:
        response = arena.runner.request(f"lazy_sign\t{arena.registers[expr.node]}\t{limits.max_terms}\t{limits.max_polynomial_exponent}\t{limits.max_rational_power_bits}")
        fields = response.split("\t")
        if len(fields) != 4 or fields[0] != "SIGN":
            raise AssertionError(f"native lazy sign response mismatch: {response}")
        actual = {"sign": int(fields[1]), "path": fields[2], "expanded": bool(int(fields[3]))}
        if actual != expected:
            raise AssertionError(_dump({"lazy_expected": expected, "actual": actual}))
        arena.runner.sign_compared += 1
    return expected


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=Path, action="append", default=[])
    parser.add_argument("--roots", type=Path)
    parser.add_argument("--exe", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--expand-limits", type=int, nargs=3)
    parser.add_argument("--special-norms", action="store_true")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    _log_pid(os.getpid(), "construct-v1-controller")
    started = time.time()
    tracked = [Path(__file__), ROOT/"kernel/src/cftuv_envelope/reference/density_construct_v1.py",
               ROOT/"kernel/src/cftuv_envelope/reference/density_scalar_v2.py", ROOT/"tools/density_scalar_v2_diff.py"]
    if args.special_norms:
        tracked.append(ROOT/"kernel/src/cftuv_envelope/reference/density_trig_v1.py")
    before = {str(path): _sha(path) for path in tracked}
    streams = {name: (args.out/(name+".jsonl")).open("w", encoding="utf-8") for name in
               ("forest", "old-literal-deltas", "old-admission-deltas", "profile-gaps", "numeric-failures", "failures", "fixtures", "roots")}
    runner = _RustRunner(args.exe, args.out) if args.exe else None
    expand_limits = None if args.expand_limits is None else ExpandLimitsV1(*args.expand_limits)
    try:
        fixtures = _fixtures(runner, streams["fixtures"])
        finite_fixtures = None if expand_limits is None else _finite_fixtures(runner, streams["fixtures"])
        special_norms = _special_norms(runner, streams["fixtures"]) if args.special_norms else None
        roots = None if args.roots is None else _roots(args.roots, streams["roots"], runner)
        forests = [_forest(path, streams, runner, args.limit, expand_limits) for path in args.file]
        native = None if runner is None else runner.close()
    finally:
        for stream in streams.values():
            stream.close()
        if runner is not None and runner.process.poll() is None:
            runner.process.kill()
            runner.process.wait(timeout=15)
            runner.stderr.close()
    totals = Counter()
    for forest in forests:
        totals.update(forest["counts"])
    changed = [path for path, digest in before.items() if _sha(path) != digest]
    receipt = {"schema": "cftuv.density-owned-construct-v1-audit.v1", "status": "COMPLETED" if not totals["unexpected_failure_records"] and not changed else "FAILED",
               "canon": DENSITY_SOURCE_CONSTRUCTION_V1, "controller_pid": os.getpid(), "elapsed_seconds": time.time()-started,
               "fixtures": fixtures, "roots": roots, "forest_sources": forests, "forest_totals": dict(totals),
               "finite_fixtures": finite_fixtures, "explicit_expand_limits": args.expand_limits,
               "special_norms": special_norms,
               "actual_rust_operation_parity": native, "source_hashes": before, "source_changed_during_run": changed,
               "scope": "First closed owned Gram dot/cross/rational-norm units and table/principal linear combination; candidate only, no product dispatch.",
               "admission_scope": "Old recorded controller signs and old/new read-only staged iv160/unexpanded reader observations, without sp.expand after export. NEEDS_OWNED_EXPANDED_VIEW is a candidate boundary, not old UNDECIDABLE. No full consumer/refusal parity claim.",
               "excluded": ["native structural importer", "midpoint/window inversions", "A2 composition enclosures", "full fan authority/controller", "backend density_compute"]}
    (args.out/"receipt.json").write_text(json.dumps(receipt, indent=2)+"\n", encoding="utf-8")
    print(_dump({"receipt": str(args.out/"receipt.json"), "status": receipt["status"], "counts": dict(totals), "native": native}), flush=True)
    return 0 if receipt["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
