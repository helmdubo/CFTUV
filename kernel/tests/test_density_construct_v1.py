"""Closed owned numeric/formal candidate, with no product/controller switch."""

import ast
from dataclasses import FrozenInstanceError
from fractions import Fraction
import json
from itertools import permutations
import os
from pathlib import Path
import subprocess
import sys

import pytest
import sympy as sp

from cftuv_envelope.reference.density_construct_v1 import (
    DensityConstructArenaV1, DensityConstructProfileCode, DensityConstructProfileUnsupported,
    ExpandLimitsV1,
    OwnedExprV1, OwnedVec2V1, owned_cross, owned_dot, owned_oriented_basis,
    owned_principal_linear_combination, owned_rational_norm_unit,
)
from cftuv_envelope.reference.density_scalar_v2 import (
    DENSITY_SOURCE_CONSTRUCTION_V1, DensityScalarProfileCode,
    DensityScalarProfileUnsupported, ScalarLiteralAstV1, ScalarV2,
)

ROOT = Path(__file__).resolve().parents[2]


def _tree(arena, expr):
    return arena.literal_tree(expr)


def _fixture():
    arena = DensityConstructArenaV1()
    angle = arena.raw_atan2(arena.rational(1), arena.rational(2))
    return arena, arena.raw_sin(angle), arena.raw_cos(angle)


def test_handles_are_immutable_arena_and_canon_bound_before_identities():
    arena, sine, _ = _fixture()
    with pytest.raises(FrozenInstanceError):
        sine.node = 0
    foreign = DensityConstructArenaV1().rational(0)
    invalid = OwnedExprV1(arena, 999)
    mismatched = OwnedExprV1(arena, sine.node, "future-canon")
    for bad in (foreign, invalid, mismatched, None):
        for calculate in (lambda: arena.mul(arena.rational(0), bad),
                          lambda: arena.add(bad), lambda: arena.integer_power(bad, 0)):
            with pytest.raises(DensityConstructProfileUnsupported) as error:
                calculate()
            assert error.value.code == DensityConstructProfileCode.ARENA_MISMATCH


def test_local_interner_literal_order_is_allocation_independent():
    a, sine_a, cosine_a = _fixture()
    b = DensityConstructArenaV1()
    b.rational(789)
    b.pi()
    angle = b.raw_atan2(b.rational(1), b.rational(2))
    cosine_b, sine_b = b.raw_cos(angle), b.raw_sin(angle)
    assert a.add(sine_a, cosine_a) == a.add(cosine_a, sine_a)
    assert a.export_raw(a.add(sine_a, cosine_a)) == b.export_raw(b.add(cosine_b, sine_b))
    assert _tree(a, a.add(sine_a, cosine_a))[1:] == (_tree(a, cosine_a), _tree(a, sine_a))


def test_numeric_island_literal_add_and_mul_stay_atomic_in_formal_operations():
    arena, sine, _ = _fixture()
    island = ScalarV2.sqrt_rational(2).add(ScalarV2.sqrt_rational(3))
    numeric = arena.numeric(island)
    product = arena.mul(numeric, sine)
    assert _tree(arena, product) == ("Mul", _tree(arena, numeric), _tree(arena, sine))
    assert arena.scalar(arena.mul(numeric, arena.numeric(island))) == island.mul(island)
    added = arena.add(numeric, sine)
    assert len(_tree(arena, added)) == 3
    doubled = arena.add(product, product)
    assert _tree(arena, doubled) == _tree(arena, arena.mul(arena.numeric(island.scaled(2)), sine))


def test_actual_scalar_coefficients_collect_only_identical_opaque_monomials():
    arena, sine, _ = _fixture()
    a = arena.mul(arena.numeric(ScalarV2.sqrt_rational(2)), sine)
    b = arena.mul(arena.numeric(ScalarV2.sqrt_rational(3)), sine)
    coefficient = ScalarV2.sqrt_rational(2).add(ScalarV2.sqrt_rational(3))
    assert arena.add(a, b) == arena.mul(arena.numeric(coefficient), sine)
    assert arena.sub(a, a) == arena.rational(0)
    assert arena.add(arena.scaled(a, 2), arena.neg(a), arena.neg(a)) == arena.rational(0)
    assert arena.add(arena.scaled(sine, 2), arena.scaled(sine, -2)) == arena.rational(0)
    assert arena.add(arena.scaled(sine, 2), sine) == arena.scaled(sine, 3)
    assert arena.scalar(arena.add(arena.numeric(ScalarV2.sqrt_rational(2)),
                                  arena.numeric(ScalarV2.sqrt_rational(3)))) is not None
    nonleading = arena.mul(arena.numeric(coefficient), sine)
    assert arena.sub(nonleading, nonleading) == arena.rational(0)
    reciprocal = arena.reciprocal(sine)
    partial = arena.mul(arena.numeric(coefficient), reciprocal)
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.sub(partial, partial)
    assert error.value.code == DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD


def test_three_rational_times_formal_add_branches_are_pinned():
    arena, sine, cosine = _fixture()
    total = arena.add(sine, cosine)
    distributed = arena.mul(arena.rational(2), total)
    assert _tree(arena, distributed)[0] == "Add"
    raw_twice = arena.import_raw(ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1,
        (("Q", 2, 1), ("Q", 1, 1), ("sin", 1), ("cos", 1), ("Add", 2, 3), ("Mul", 0, 4)), 5))
    raw_add = arena._handle(arena._check(raw_twice).children[1])
    kept = arena.mul(arena.rational(3), raw_twice)
    assert _tree(arena, kept) == ("Mul", ("Q", 6, 1), _tree(arena, raw_add))
    assert arena.mul(arena.rational(Fraction(1, 2)), raw_twice) == raw_add
    scalar_sum = arena.numeric(ScalarV2.sqrt_rational(2).add(3))
    assert arena.scalar(arena.mul(arena.rational(2), scalar_sum)) == ScalarV2.sqrt_rational(2).add(3).scaled(2)


def test_opaque_positive_integer_exponents_and_identical_positive_half_pairs():
    arena, sine, _ = _fixture()
    assert arena.mul(sine, sine) == arena.integer_power(sine, 2)
    assert arena.mul(sine, arena.integer_power(sine, 2)) == arena.integer_power(sine, 3)
    half = arena.sqrt(sine)
    assert arena.mul(half, half) == sine
    other = arena.sqrt(arena.pi())
    assert _tree(arena, arena.mul(half, other))[0] == "Mul"
    assert arena.integer_power(sine, 0) == arena.rational(1)
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.integer_power(sine, -1)
    assert error.value.code == DensityConstructProfileCode.NEGATIVE_OPAQUE_POWER


def test_positive_half_even_power_pairs_odd_leftover_and_partial_domain():
    arena, sine, _ = _fixture()
    half = arena.raw_pow(sine, Fraction(3, 2))
    assert arena.mul(half, half) == arena.integer_power(sine, 3)
    assert arena.integer_power(half, 4) == arena.integer_power(sine, 6)
    assert _tree(arena, arena.integer_power(half, 3)) == ("Pow", _tree(arena, half), ("Q", 3, 1))
    assert arena.mul(half, half, half) == arena.mul(arena.integer_power(sine, 3), half)
    reciprocal = arena.reciprocal(sine)
    root = arena.sqrt(reciprocal)
    assert arena.integer_power(root, 2) == reciprocal
    assert arena.mul(root, root) == reciprocal
    assert arena.has_partial_domain(arena.mul(root, root))
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.mul(arena.mul(root, root), arena.rational(0))
    assert error.value.code == DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.integer_power(arena.raw_pow(sine, Fraction(2**20-1, 2)), 4)
    assert error.value.code == DensityConstructProfileCode.POWER_PROFILE


def test_accumulated_half_powers_fold_after_complete_commutative_collection():
    arena, sine, _ = _fixture()
    half = arena.sqrt(sine)
    cube = arena.integer_power(half, 3)
    assert arena.mul(cube, cube) == arena.integer_power(sine, 3)
    factors = tuple(arena.integer_power(half, exponent) for exponent in (3, 5, 7))
    expected = arena.integer_power(half, 15)
    for ordered in permutations(factors):
        assert arena.mul(*ordered) == expected
    for ordered in permutations((*factors, cube)):
        assert arena.mul(*ordered) == arena.integer_power(sine, 9)


def test_raw_trig_atoms_do_not_evaluate_or_supply_trig_identities():
    arena = DensityConstructArenaV1()
    zero = arena.rational(0)
    cosine, sine = arena.raw_cos(zero), arena.raw_sin(zero)
    assert _tree(arena, sine) == ("sin", ("Q", 0, 1))
    assert _tree(arena, cosine) == ("cos", ("Q", 0, 1))
    total = arena.add(arena.integer_power(sine, 2), arena.integer_power(cosine, 2))
    assert arena.scalar(total) is None
    assert _tree(arena, arena.raw_atan2(zero, zero)) == ("atan2", ("Q", 0, 1), ("Q", 0, 1))


def test_nested_integer_power_retains_literal_and_generated_power_cap():
    arena, sine, _ = _fixture()
    squared = arena.integer_power(sine, 2)
    nested = arena.integer_power(squared, 3)
    assert _tree(arena, nested) == ("Pow", _tree(arena, squared), ("Q", 3, 1))
    huge = arena.integer_power(sine, 2**20)
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.mul(huge, sine)
    assert error.value.code == DensityConstructProfileCode.POWER_PROFILE


def test_roots_raw_powers_and_inverse_price_cannot_be_bypassed():
    arena = DensityConstructArenaV1()
    roots = [ScalarV2.sqrt_rational(p) for p in (2, 3, 5, 7)]
    three = arena.numeric(roots[0].add(roots[1]).add(roots[2]))
    four = arena.numeric(roots[0].add(roots[1]).add(roots[2]).add(roots[3]))
    assert arena.scalar(arena.integer_power(three, -1)).mul(arena.scalar(three)) == ScalarV2.rational(1)
    for calculate in (lambda: arena.integer_power(four, -2), lambda: arena.raw_pow(four, -1),
                      lambda: arena.div(arena.rational(1), four)):
        with pytest.raises(DensityScalarProfileUnsupported) as error:
            calculate()
        assert error.value.code == DensityScalarProfileCode.INVERSE_CLASS_LIMIT
    assert arena.scalar(arena.raw_pow(arena.rational(9), Fraction(-3, 2))) == ScalarV2.rational(Fraction(1, 27))
    assert arena.scalar(arena.sqrt(three)) is None
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.sqrt(arena.rational(-1))
    assert error.value.code == DensityConstructProfileCode.NEGATIVE_RATIONAL_ROOT
    with pytest.raises(ZeroDivisionError):
        arena.integer_power(arena.rational(0), -1)


@pytest.mark.parametrize("exponent", [2**20+1, -(2**20+1), 2**63])
def test_shared_power_profile_is_internal_and_checked_before_identity(exponent):
    arena = DensityConstructArenaV1()
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.integer_power(arena.rational(1), exponent)
    assert error.value.code == DensityConstructProfileCode.POWER_PROFILE


def test_huge_hidden_root_equal_values_have_identical_mixed_literals():
    p, q, r = 2**89-1, 2**61-1, 2**31-1
    arena, sine, _ = _fixture()
    forms = [ScalarV2.sqrt_rational(q*r).mul(ScalarV2.sqrt_rational(p*q)),
             ScalarV2.sqrt_rational(p*r).scaled(q), ScalarV2.sqrt_rational(p*q*q*r)]
    assert len({arena.export_raw(arena.mul(arena.numeric(value), sine)) for value in forms}) == 1


def test_raw_import_preserves_opaque_order_and_reduces_numeric_subtrees():
    arena = DensityConstructArenaV1()
    literal = ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1,
        (("Q", 2, 1), ("Q", 1, 2), ("Pow", 0, 1), ("Q", 8, 1), ("Pow", 3, 1),
         ("Mul", 2, 4), ("sin", 5), ("cos", 5), ("Add", 6, 7)), 8)
    value = arena.import_raw(literal)
    assert _tree(arena, value) == ("Add", ("sin", ("Q", 4, 1)), ("cos", ("Q", 4, 1)))
    other = DensityConstructArenaV1()
    exported = arena.export_raw(value)
    assert other.export_raw(other.import_raw(exported)) == exported
    with pytest.raises(DensityConstructProfileUnsupported):
        arena.import_raw(ScalarLiteralAstV1("unknown", literal.nodes, literal.root))


@pytest.mark.parametrize("rows,root", [((("Q", 2, 2),), 0), ((("Q", 1, 0),), 0),
    ((("Float", 1),), 0), ((("Symbol", "x"),), 0), ((("Add", 0, 0),), 0),
    ((("Q", 1, 1), ("tan", 0)), 1), ((("pi", 0),), 0), ((("Q", 1, 1),), -1)])
def test_import_rejects_nonprofile_syntax(rows, root):
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        DensityConstructArenaV1().import_raw(ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1, rows, root))
    assert error.value.code == DensityConstructProfileCode.WIRE_PROFILE


def test_imported_opaque_reciprocal_is_readonly_not_cancellation_rule():
    arena, sine, _ = _fixture()
    view = arena.export_raw(sine)
    rows = view.nodes + (("Q", -1, 1), ("Pow", view.root, len(view.nodes)))
    reciprocal = arena.import_raw(ScalarLiteralAstV1(view.canon, rows, len(rows)-1))
    assert arena.scalar(arena.mul(sine, reciprocal)) is None
    assert _tree(arena, arena.mul(sine, reciprocal))[0] == "Mul"


def test_closed_gram_unit_and_principal_ray_stays_owned():
    arena = DensityConstructArenaV1()
    gram = tuple(arena.rational(q) for q in (1, 0, 3))
    left, right = OwnedVec2V1(arena.rational(1), arena.rational(1)), OwnedVec2V1(arena.rational(-3), arena.rational(1))
    assert arena.whole_rational(owned_dot(arena, gram, left, right)) == 0
    assert arena.whole_rational(owned_cross(arena, left, right, 1)) == 4
    incoming, normal = owned_oriented_basis(arena, gram, left, 1)
    for vector in (incoming, normal, owned_rational_norm_unit(arena, gram, right)):
        assert arena.whole_rational(owned_dot(arena, gram, vector, vector)) == 1
    assert arena.whole_rational(owned_dot(arena, gram, incoming, normal)) == 0
    angle = arena.scaled(arena.raw_atan2(arena.rational(1), arena.rational(0)), Fraction(1, 4))
    ray = owned_principal_linear_combination(arena, angle, incoming, normal)
    assert ray.x.arena is arena and ray.y.arena is arena
    assert arena.scalar(ray.x) is None
    assert arena.scalar(owned_dot(arena, gram, ray, ray)) is None


def test_unit_profile_and_exact_zero_stay_distinct():
    arena = DensityConstructArenaV1()
    gram = (arena.numeric(ScalarV2.sqrt_rational(2)), arena.rational(0), arena.rational(1))
    vector = OwnedVec2V1(arena.rational(1), arena.rational(0))
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        owned_rational_norm_unit(arena, gram, vector)
    assert error.value.code == DensityConstructProfileCode.RATIONAL_NORM_REQUIRED
    with pytest.raises(ZeroDivisionError):
        owned_rational_norm_unit(arena, gram, OwnedVec2V1(arena.rational(0), arena.rational(0)))


@pytest.mark.parametrize("owner,turn", [(1, 1), (1, -1), (-1, 1), (-1, -1)])
def test_owner_orientation_and_turn_orientation_are_separate_facts(owner, turn):
    arena = DensityConstructArenaV1()
    gram = tuple(arena.rational(q) for q in (1, 0, 1))
    incoming, normal = owned_oriented_basis(arena, gram, OwnedVec2V1(arena.rational(1), arena.rational(0)), owner)
    assert arena.whole_rational(normal.y) == owner
    signed = OwnedVec2V1(arena.scaled(normal.x, turn), arena.scaled(normal.y, turn))
    angle = arena.scaled(arena.pi(), Fraction(1, 4))
    ray = owned_principal_linear_combination(arena, angle, incoming, signed)
    assert _tree(arena, ray.y) == _tree(arena, arena.scaled(arena.raw_sin(angle), owner*turn))


def test_sympy_view_raw_only_and_new_arithmetic_ignores_export(monkeypatch):
    arena, sine, cosine = _fixture()
    expr = arena.add(arena.scaled(sine, 2), cosine)
    for name in ("Add", "Mul", "Pow", "sin", "cos", "atan2"):
        original = getattr(sp, name)
        def checked(*args, _constructor=original, **kwargs):
            assert kwargs == {"evaluate": False}
            return _constructor(*args, **kwargs)
        monkeypatch.setattr(sp, name, checked)
    view = arena.to_sympy_readonly(expr)
    assert view is not None
    def forbidden(*args, **kwargs):
        raise AssertionError("evaluated SymPy operation after export")
    for name in ("Add", "Mul", "Pow", "sin", "cos", "atan2", "expand", "cancel", "simplify"):
        monkeypatch.setattr(sp, name, forbidden)
    assert arena.scalar(arena.mul(arena.rational(2), arena.rational(3))) == ScalarV2.rational(6)
    assert arena.scalar(arena.add(expr, arena.rational(1))) is None


def test_core_has_no_sympy_math_outside_readonly_adapter():
    module = ROOT / "kernel/src/cftuv_envelope/reference/density_construct_v1.py"
    parsed = ast.parse(module.read_text(encoding="utf-8"))
    adapter = next(node for node in ast.walk(parsed) if isinstance(node, ast.FunctionDef) and node.name == "to_sympy_readonly")
    for node in ast.walk(parsed):
        if isinstance(node, ast.Import):
            assert all(alias.name != "sympy" or adapter.lineno <= node.lineno <= adapter.end_lineno for alias in node.names)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "sp":
            assert adapter.lineno <= node.lineno <= adapter.end_lineno
    assert not any(isinstance(node, ast.Attribute) and node.attr in ("simplify", "factor", "from_sympy") for node in ast.walk(parsed))


def test_source_cache_witness_in_isolated_child_keeps_owned_mixed_literal():
    code = r'''
import json,sys
sys.path.insert(0,sys.argv[1])
import sympy as sp
from sympy.core.cache import clear_cache
from sympy.ntheory.factor_ import factor_cache
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2
p,q=2**89-1,2**61-1;n=p*p*q
def candidate():
 a=DensityConstructArenaV1();s=a.raw_sin(a.rational(1));v=a.numeric(ScalarV2.sqrt_rational(n));raw=a.export_raw(a.mul(v,s));return [raw.nodes,raw.root]
clear_cache();factor_cache.clear();cold=sp.srepr(sp.sqrt(sp.Integer(n)));left=candidate()
clear_cache();factor_cache[n]=p;warm=sp.srepr(sp.sqrt(sp.Integer(n)));right=candidate()
print(json.dumps({'source_changed':cold!=warm,'candidate_same':left==right}))
'''
    process = subprocess.Popen([sys.executable, "-c", code, str(ROOT/"kernel/src")],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    log = os.environ.get("CFTUV_DENSITY_PID_LOG")
    if log:
        with Path(log).open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"pid": process.pid, "task": "construct-v1-isolated-cache-test"})+"\n")
    stdout, stderr = process.communicate(timeout=45)
    assert process.returncode == 0, stderr
    assert json.loads(stdout) == {"source_changed": True, "candidate_same": True}


def test_partial_reciprocal_numeric_inverse_and_opaque_atomic_no_cancellation():
    arena, sine, _ = _fixture()
    reciprocal = arena.reciprocal(sine)
    assert _tree(arena, reciprocal) == ("Pow", _tree(arena, sine), ("Q", -1, 1))
    assert _tree(arena, arena.partial_div(sine, sine))[0] == "Mul"
    assert arena.whole_rational(arena.reciprocal(arena.rational(2))) == Fraction(1, 2)
    with pytest.raises(ZeroDivisionError):
        arena.reciprocal(arena.rational(0))
    four = arena.add(*(arena.sqrt(arena.rational(p)) for p in (2, 3, 5, 7)))
    with pytest.raises(DensityScalarProfileUnsupported) as error:
        arena.reciprocal(four)
    assert error.value.code == DensityScalarProfileCode.INVERSE_CLASS_LIMIT


def test_owned_form_is_immutable_semantic_view_not_literal_arithmetic():
    arena, sine, _ = _fixture()
    numeric = arena.numeric(ScalarV2.sqrt_rational(2).add(3))
    form = arena.owned_form(numeric)
    assert form.kind == "Numeric" and form.children == () and form.payload == arena.scalar(numeric)
    assert _tree(arena, numeric)[0] == "Add"
    product = arena.mul(numeric, sine)
    mixed = arena.owned_form(product)
    assert mixed.kind == "Mul" and mixed.payload is None
    assert all(child.arena is arena for child in mixed.children)
    with pytest.raises(FrozenInstanceError):
        mixed.kind = "Numeric"
    reciprocal = arena.reciprocal(sine)
    assert arena.has_partial_domain(reciprocal)
    assert arena.has_partial_domain(arena.raw_cos(reciprocal))
    assert not arena.has_partial_domain(numeric)


@pytest.mark.parametrize("nesting", ["direct", "sin", "cos", "atan2", "sum", "product"])
def test_partial_domain_is_not_discarded_by_zero_cancel_or_power(nesting):
    arena, sine, cosine = _fixture()
    reciprocal = arena.reciprocal(sine)
    expr = {"direct": lambda: reciprocal, "sin": lambda: arena.raw_sin(reciprocal),
            "cos": lambda: arena.raw_cos(reciprocal), "atan2": lambda: arena.raw_atan2(reciprocal, sine),
            "sum": lambda: arena.add(reciprocal, cosine), "product": lambda: arena.mul(reciprocal, cosine)}[nesting]()
    for operation in (lambda: arena.mul(arena.rational(0), expr), lambda: arena.sub(expr, expr),
                      lambda: arena.integer_power(expr, 0), lambda: arena.scaled(expr, 0)):
        with pytest.raises(DensityConstructProfileUnsupported) as error:
            operation()
        assert error.value.code == DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD


def test_zero_over_known_nonzero_opaque_is_a_named_gap_until_nonzero_aware_gateway():
    arena = DensityConstructArenaV1()
    known_nonzero_math = arena.raw_cos(arena.rational(0))
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.partial_div(arena.rational(0), known_nonzero_math)
    assert error.value.code == DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD
    assert arena.whole_rational(arena.partial_div(arena.rational(0), arena.rational(3))) == 0


def test_imported_partial_domain_also_propagates_through_opaque_leaves():
    arena = DensityConstructArenaV1()
    raw = ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1,
        (("Q", 0, 1), ("sin", 0), ("Q", -1, 1), ("Pow", 1, 2), ("cos", 3)), 4)
    expr = arena.import_raw(raw)
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.sub(expr, expr)
    assert error.value.code == DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD


def test_finite_expansion_distributes_polynomial_with_exact_rational_grouping():
    arena, sine, cosine = _fixture()
    limits = ExpandLimitsV1(64, 8, 256)
    total = arena.add(sine, cosine)
    power = arena.integer_power(total, 2)
    expanded = arena.finite_expand(power, limits)
    expected = arena.add(arena.integer_power(sine, 2), arena.integer_power(cosine, 2),
                         arena.scaled(arena.mul(sine, cosine), 2))
    assert arena.export_raw(expanded) == arena.export_raw(expected)
    assert arena.export_expanded_raw(power, limits) == arena.export_raw(expected)
    assert arena.finite_expand(expanded, limits) == expanded


def test_numeric_sum_is_exposed_only_as_canonical_values_at_expansion_boundary():
    arena, sine, _ = _fixture()
    value = ScalarV2.sqrt_rational(2).add(ScalarV2.sqrt_rational(3)).add(Fraction(2, 5))
    numeric = arena.numeric(value)
    limits = ExpandLimitsV1(16, 8, 256)
    assert arena.finite_expand(numeric, limits) == numeric
    product = arena.mul(numeric, sine)
    assert _tree(arena, product)[0] == "Mul"
    expanded = arena.finite_expand(product, limits)
    expected = arena.add(arena.mul(arena.numeric(ScalarV2.sqrt_rational(2)), sine),
                         arena.mul(arena.numeric(ScalarV2.sqrt_rational(3)), sine), arena.scaled(sine, Fraction(2, 5)))
    assert arena.export_raw(expanded) == arena.export_raw(expected)
    assert expanded == product  # После выбора слагаемых Scalar-коэффициенты снова собираются.


def test_finite_expansion_does_not_rewrite_opaque_child_or_reciprocal():
    arena, sine, cosine = _fixture()
    limits = ExpandLimitsV1(64, 8, 256)
    reciprocal = arena.reciprocal(sine)
    assert arena.finite_expand(reciprocal, limits) == reciprocal
    assert arena.finite_expand(arena.sqrt(sine), limits) == arena.sqrt(sine)
    child = arena.mul(arena.add(sine, cosine), arena.add(sine, arena.pi()))
    for leaf in (arena.raw_sin(child), arena.raw_cos(child), arena.raw_atan2(child, sine),
                 arena.sqrt(child), arena.reciprocal(child)):
        with pytest.raises(DensityConstructProfileUnsupported) as error:
            arena.finite_expand(leaf, limits)
        assert error.value.code == DensityConstructProfileCode.OPAQUE_CHILD_CHANGES


@pytest.mark.parametrize("limits,expression", [(ExpandLimitsV1(3, 8, 256), "square"),
    (ExpandLimitsV1(16, 1, 256), "square"), (ExpandLimitsV1(0, 0, 0), "scalar"),
    (ExpandLimitsV1(4, 8, 1), "raw_rational_product")])
def test_finite_expansion_limits_are_explicit_internal_profiles(limits, expression):
    arena, sine, cosine = _fixture()
    if expression == "square":
        expr = arena.integer_power(arena.add(sine, cosine), 2)
    elif expression == "scalar":
        expr = arena.rational(1)
    else:
        expr = arena.mul(arena.rational(8), sine)
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.finite_expand(expr, limits)
    assert error.value.code == DensityConstructProfileCode.EXPANSION_LIMIT


def test_finite_profile_matches_transient_choices_before_late_cancellation():
    arena, a, b = _fixture()
    c = arena.pi()
    reducible = arena.sub(arena.mul(a, arena.add(a, b)), arena.integer_power(a, 2))
    product = arena.mul(reducible, c)
    # Внутренняя Add даёт три выбора до позднего сокращения a²-a².
    with pytest.raises(DensityConstructProfileUnsupported) as error:
        arena.finite_expand(product, ExpandLimitsV1(2, 8, 256))
    assert error.value.code == DensityConstructProfileCode.EXPANSION_LIMIT
    expanded = arena.finite_expand(product, ExpandLimitsV1(3, 8, 256))
    assert arena.export_raw(expanded) == arena.export_raw(arena.mul(a, b, c))


def test_lazy_legacy_decision_does_not_expand_source_interval_success(monkeypatch):
    import importlib.util
    spec = importlib.util.spec_from_file_location("construct_candidate_audit", ROOT/"tools/density_construct_v1_diff.py")
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    arena = audit._PairedArena()
    child = arena.mul(arena.add(arena.pi(), arena.raw_cos(arena.rational(0))),
                      arena.add(arena.rational(Fraction(1, 4)), arena.scaled(arena.raw_sin(arena.rational(1)), Fraction(1, 4))))
    positive = arena.add(arena.rational(2), arena.raw_sin(child))
    original = arena.arena.finite_expand
    calls = []
    def counted(expr, limits):
        calls.append(expr)
        return original(expr, limits)
    monkeypatch.setattr(arena.arena, "finite_expand", counted)
    def forbidden(*args, **kwargs):
        raise AssertionError("SymPy expand after read-only export")
    monkeypatch.setattr(sp, "expand", forbidden)
    assert audit._paired_lazy_sign(arena, positive, ExpandLimitsV1(0, 0, 0)) == {"sign": 1, "path": "INTERVAL", "expanded": False}
    assert not calls
    assert audit._paired_lazy_sign(arena, positive, ExpandLimitsV1(0, 0, 0))["path"] == "MEMO"
    from math import isqrt
    near_zero = arena.sub(arena.sqrt(arena.rational(2)), arena.rational(Fraction(isqrt(2 << 400), 1 << 200)))
    assert audit._paired_lazy_sign(arena, near_zero, ExpandLimitsV1(256, 16, 4096)) == {"sign": 1, "path": "QUADRATIC", "expanded": True}
    assert calls == [near_zero]
