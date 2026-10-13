"""Numeric-only construction candidate: canonical values and literal decision DAG."""

import ast
from dataclasses import FrozenInstanceError
from fractions import Fraction
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import sympy as sp

from cftuv_envelope.reference.density_scalar_v2 import (
    DENSITY_SOURCE_CONSTRUCTION_V1,
    DensityScalarProfileCode,
    DensityScalarProfileUnsupported,
    ScalarV2,
)
from cftuv_envelope.reference.density_shell_v2 import DensityShellV2Candidate, density_shell_v2

ROOT = Path(__file__).resolve().parents[2]
P, Q, R = 2**89 - 1, 2**61 - 1, 2**31 - 1


def _sympy_raw(literal):
    values = []
    for tag, *args in literal.nodes:
        if tag == "Q":
            values.append(sp.Rational(*args))
        else:
            constructor = {"Add": sp.Add, "Mul": sp.Mul, "Pow": sp.Pow}[tag]
            assert all(type(index) is int and 0 <= index < len(values) for index in args)
            values.append(constructor(*(values[index] for index in args), evaluate=False))
    return values[literal.root]


def _check_literal_value(scalar):
    literal = scalar.literal_ast()
    assert literal.canon == DENSITY_SOURCE_CONSTRUCTION_V1
    read = density_shell_v2(_sympy_raw(literal))
    assert isinstance(read, DensityShellV2Candidate)
    assert read.value == scalar.value
    return literal


def _isolated(code, mode=""):
    command = [sys.executable, "-c", code, str(ROOT / "kernel/src"), mode]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    log = os.environ.get("CFTUV_DENSITY_PID_LOG")
    if log:
        with Path(log).open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"pid": process.pid, "task": "scalar-v2-isolated-test", "mode": mode}) + "\n")
    stdout, stderr = process.communicate(timeout=45)
    assert process.returncode == 0, stderr
    assert not stderr
    return json.loads(stdout)


def test_literal_zero_identity_coefficient_and_negative_conventions():
    assert ScalarV2.rational(0).literal_ast().nodes == (("Q", 0, 1),)
    assert ScalarV2.rational(Fraction(-3, 5)).literal_ast().nodes == (("Q", -3, 5),)
    positive = ScalarV2.sqrt_rational(2).literal_ast()
    assert positive.nodes == (("Q", 2, 1), ("Q", 1, 2), ("Pow", 0, 1))
    negative = ScalarV2.sqrt_rational(2).neg().literal_ast()
    assert negative.nodes == (("Q", -1, 1), ("Q", 2, 1), ("Q", 1, 2), ("Pow", 1, 2), ("Mul", 0, 3))
    assert negative.root == 4


def test_literal_term_order_and_first_encounter_sharing_are_pinned():
    scalar = ScalarV2.sqrt_rational(3) + ScalarV2.rational(Fraction(1, 2)) + ScalarV2.sqrt_rational(2)
    literal = _check_literal_value(scalar)
    assert scalar.value.terms == ((1, 1, 2), (2, 1, 1), (3, 1, 1))
    assert literal.nodes == (
        ("Q", 1, 2), ("Q", 2, 1), ("Pow", 1, 0),
        ("Q", 3, 1), ("Pow", 3, 0), ("Add", 0, 2, 4),
    )
    assert literal.root == 5
    assert literal == (ScalarV2.sqrt_rational(2) + ScalarV2.sqrt_rational(3) + Fraction(1, 2)).literal_ast()


def test_equal_large_squarefree_products_and_hidden_roots_encode_identically():
    forms = (
        ScalarV2.sqrt_rational(Q*R) * ScalarV2.sqrt_rational(P*Q),
        ScalarV2.sqrt_rational(P*R).scaled(Q),
        ScalarV2.sqrt_rational(P*Q*Q*R),
    )
    assert len({form.value for form in forms}) == 1
    assert len({_check_literal_value(form) for form in forms}) == 1
    hidden = ScalarV2.sqrt_rational(P*P*Q)
    explicit = ScalarV2.sqrt_rational(Q).scaled(P)
    assert hidden == explicit
    assert _check_literal_value(hidden) == _check_literal_value(explicit)


def test_class_merging_cancellation_distributivity_and_scaled_decompositions():
    sqrt = ScalarV2.sqrt_rational
    assert (sqrt(18) - sqrt(2).scaled(3)).literal_ast() == ScalarV2.rational(0).literal_ast()
    assert ((sqrt(2) + sqrt(3))*sqrt(2)).literal_ast() == (ScalarV2.rational(2) + sqrt(6)).literal_ast()
    assert (sqrt(Fraction(2, 9)).scaled(3)).literal_ast() == sqrt(2).literal_ast()
    assert sqrt(50).scaled(Fraction(-2, 5)).literal_ast() == sqrt(2).scaled(-2).literal_ast()


@pytest.mark.parametrize("exponent", (-7, -2, -1, 0, 1, 2, 9))
def test_integer_power_and_inverse_literal_values(exponent):
    scalar = ScalarV2.rational(1) + ScalarV2.sqrt_rational(2)
    factor = scalar.inverse() if exponent < 0 else scalar
    expected = ScalarV2.rational(1)
    for _ in range(abs(exponent)):
        expected = expected * factor
    assert (scalar**exponent).literal_ast() == expected.literal_ast()
    _check_literal_value(scalar**exponent)


def test_three_classes_inverse_and_four_classes_price_limit_are_explicit():
    scalar = ScalarV2.rational(1) + ScalarV2.sqrt_rational(2) + ScalarV2.sqrt_rational(3)
    inverse = scalar**-3
    assert inverse * scalar**3 == ScalarV2.rational(1)
    _check_literal_value(inverse)
    four = scalar + ScalarV2.sqrt_rational(5)
    with pytest.raises(DensityScalarProfileUnsupported) as caught:
        four**-2
    assert caught.value.code is DensityScalarProfileCode.INVERSE_CLASS_LIMIT
    assert "4 classes" in caught.value.detail


def test_negative_power_inverts_base_before_any_multiplication(monkeypatch):
    calls = []
    original = ScalarV2.inverse
    def recorded(self):
        calls.append(self.value)
        return original(self)
    monkeypatch.setattr(ScalarV2, "inverse", recorded)
    base = ScalarV2.rational(1) + ScalarV2.sqrt_rational(2)
    _check_literal_value(base**-4)
    assert calls == [base.value]


@pytest.mark.parametrize("value", (ScalarV2.rational(-1), ScalarV2.rational(2) + ScalarV2.sqrt_rational(2)))
def test_nonquadratic_and_negative_roots_are_profile_boundaries(value):
    with pytest.raises(DensityScalarProfileUnsupported) as caught:
        value.sqrt()
    assert caught.value.code is DensityScalarProfileCode.ROOT_REQUIRES_NONNEGATIVE_RATIONAL


def test_sqrt_reads_whole_value_and_exact_zero_errors_propagate():
    square = ScalarV2.sqrt_rational(2)**2
    assert square.sqrt() == ScalarV2.sqrt_rational(2)
    assert ScalarV2.rational(0).sqrt() == ScalarV2.rational(0)
    with pytest.raises(ZeroDivisionError):
        (ScalarV2.sqrt_rational(18) - ScalarV2.sqrt_rational(2).scaled(3))**-1


@pytest.mark.parametrize("value", (sp.Rational(1, 2), sp.Integer(2), sp.sqrt(2), 0.5, True, "1/2"))
def test_core_rejects_symbolic_or_lossy_constructor_inputs(value):
    with pytest.raises(TypeError):
        ScalarV2.rational(value)


def test_scalar_and_literal_dag_are_immutable():
    value = ScalarV2.sqrt_rational(2)
    literal = value.literal_ast()
    with pytest.raises(FrozenInstanceError):
        value.value = ScalarV2.rational(2).value
    with pytest.raises(FrozenInstanceError):
        literal.root = 0
    with pytest.raises(TypeError):
        literal.nodes[0] = ("Q", 0, 1)


def test_generated_gram_dot_cross_units_and_principal_root_recipe():
    scalar = ScalarV2.rational
    gram = ((scalar(2), scalar(1)), (scalar(1), scalar(3)))
    left, right = (scalar(1), scalar(2)), (scalar(-2), scalar(1))
    def dot(a, b):
        return a[0]*(gram[0][0]*b[0]+gram[0][1]*b[1])+a[1]*(gram[1][0]*b[0]+gram[1][1]*b[1])
    left_squared, right_squared = dot(left, left), dot(right, right)
    assert left_squared == scalar(18) and right_squared == scalar(7)
    raw_dot = dot(left, right)
    assert raw_dot == scalar(-1)
    left_unit = tuple(component/left_squared.sqrt() for component in left)
    right_unit = tuple(component/right_squared.sqrt() for component in right)
    cosine_squared = raw_dot**2/(left_squared*right_squared)
    assert cosine_squared == scalar(Fraction(1, 126))
    assert dot(left_unit, left_unit) == dot(right_unit, right_unit) == scalar(1)
    assert dot(left_unit, right_unit)**2 == cosine_squared
    cross = left_unit[0]*right_unit[1]-left_unit[1]*right_unit[0]
    assert cross**2 * scalar(5) + cosine_squared == scalar(1)
    cosine_root, sine_root = cosine_squared.sqrt(), (scalar(1)-cosine_squared).sqrt()
    assert cosine_root**2+sine_root**2 == scalar(1)
    normal = (scalar(-7), scalar(4))
    assert dot(left, normal) == scalar(0)
    normal_unit = tuple(component/dot(normal, normal).sqrt() for component in normal)
    assert dot(normal_unit, normal_unit) == scalar(1)
    for value in (*left_unit, *right_unit, *normal_unit, cosine_root, sine_root, cross):
        _check_literal_value(value)


def test_factor_cache_state_witness_is_isolated_and_scalar_ast_is_identical():
    script = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import sympy as sp
from sympy.core.cache import clear_cache
from sympy.ntheory.factor_ import factor_cache
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2
p, q = 2**89-1, 2**61-1
n = p*p*q
clear_cache(); factor_cache.clear()
if sys.argv[2] == "warm":
    factor_cache[n] = q
source = sp.sqrt(sp.Integer(n))
candidate = ScalarV2.sqrt_rational(n)
equivalent = ScalarV2.sqrt_rational(q).scaled(p)
assert candidate.literal_ast() == equivalent.literal_ast()
literal = candidate.literal_ast()
print(json.dumps({"source_srepr": sp.srepr(source), "value": candidate.value.wire(),
                  "canon": literal.canon, "nodes": literal.nodes, "root": literal.root}))
'''
    cold, warm = _isolated(script, "cold"), _isolated(script, "warm")
    assert cold["source_srepr"] != warm["source_srepr"]
    assert {key: value for key, value in cold.items() if key != "source_srepr"} == {
        key: value for key, value in warm.items() if key != "source_srepr"}


def test_numeric_leaf_import_and_operations_need_no_sympy_or_global_cache():
    # Изолированный leaf loader обходится без тяжёлого существующего reference/__init__.
    script = r'''
import builtins, importlib, json, pathlib, sys, types
root = pathlib.Path(sys.argv[1])/"cftuv_envelope"/"reference"
package = types.ModuleType("isolated_scalar_leaf")
package.__path__ = [str(root)]
sys.modules[package.__name__] = package
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name == "sympy" or name.startswith("sympy."):
        raise AssertionError("numeric leaf imported sympy")
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
module = importlib.import_module("isolated_scalar_leaf.density_scalar_v2")
value = module.ScalarV2.sqrt_rational(18)*module.ScalarV2.sqrt_rational(2)
assert value == module.ScalarV2.rational(6)
assert not any(name == "sympy" or name.startswith("sympy.") for name in sys.modules)
print(json.dumps({"nodes": value.literal_ast().nodes}))
'''
    assert _isolated(script)["nodes"] == [["Q", 6, 1]]
    tree = ast.parse((ROOT/"kernel/src/cftuv_envelope/reference/density_scalar_v2.py").read_text(encoding="utf-8"))
    imported = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert "sympy" not in imported and "native_exact" not in imported
