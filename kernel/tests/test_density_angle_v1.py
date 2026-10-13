"""Контрпримеры ветви и строгие интервальные гарантии отдельного A2 value-path."""

import ast
from dataclasses import FrozenInstanceError
from fractions import Fraction
from pathlib import Path
import pickle

import pytest

from cftuv_envelope.reference.density_angle_v1 import (
    AngleAtomV1, AngleV1Error, AngleV1ErrorCode, COMPONENT_BITS,
    FractionIntervalV1, profile_bound_is_valid, root_bits,
)


def test_principal_floor_precedes_polynomial_equality():
    # T_4(1/2)=-1/2, но это корень другой ветви при отрицательном dot.
    atom = AngleAtomV1(4, Fraction(1, 4), -1)
    assert 8 * Fraction(1, 2)**4 - 8 * Fraction(1, 2)**2 + 1 == -Fraction(1, 2)
    assert atom.compare(Fraction(1, 2)) == -1
    enclosure = atom.enclose(16)
    assert enclosure.lower > Fraction(1, 2)
    assert enclosure.stats.bisections == 17


def test_rational_principal_root_stops_on_exact_midpoint():
    atom = AngleAtomV1(3, Fraction(81, 256), -1)
    root = atom.enclose(32)
    assert (root.lower, root.upper) == (Fraction(3, 4), Fraction(3, 4))
    assert root.stats.bisections == 3
    assert (root.stats.polynomial_degree, root.stats.sturm_polynomials,
            root.stats.variation_evaluations) == (6, 0, 0)
    ray = atom.ray(2)
    assert ray.cosine == FractionIntervalV1.point(Fraction(1, 8))
    assert ray.sine.lower**2 <= Fraction(63, 64) <= ray.sine.upper**2


@pytest.mark.parametrize("n,bits", [(2, 325), (3, 327), (4, 329), (5, 331), (6, 333)])
def test_fixed_profile_is_integer_proved(n, bits):
    assert root_bits(n) == bits
    assert profile_bound_is_valid(n)


@pytest.mark.parametrize("n", range(2, 7))
@pytest.mark.parametrize("ratio,sign", [
    (Fraction(0), 0), (Fraction(1, 2**512), -1),
    (Fraction(1, 2**512), 1), (Fraction(2**512 - 1, 2**512), -1),
    (Fraction(2**512 - 1, 2**512), 1),
])
def test_fixed_components_are_strictly_narrow_even_when_sine_is_near_zero(n, ratio, sign):
    atom = AngleAtomV1(n, ratio, sign)
    for ordinal in range(1, n):
        ray = atom.ray(ordinal)
        assert -1 <= ray.cosine.lower <= ray.cosine.upper <= 1
        assert 0 <= ray.sine.lower <= ray.sine.upper <= 1
        assert ray.cosine.width < Fraction(1, 2**COMPONENT_BITS)
        assert ray.sine.width < Fraction(1, 2**COMPONENT_BITS)
        assert ray.root.stats.bisections <= root_bits(n) + 1


def test_interval_dependency_square_and_integer_sqrt_round_outward():
    interval = FractionIntervalV1(-3, 2)
    assert interval.square() == FractionIntervalV1(0, 9)
    assert interval.mul(interval) == FractionIntervalV1(-6, 9)
    assert FractionIntervalV1.point(Fraction(9, 16)).positive_sqrt(2) == (
        FractionIntervalV1.point(Fraction(3, 4)))
    root = FractionIntervalV1(Fraction(1, 3), Fraction(2, 3)).positive_sqrt(5)
    assert root.lower**2 <= Fraction(1, 3)
    assert root.upper**2 >= Fraction(2, 3)
    assert root.lower == Fraction(18, 32) and root.upper == Fraction(27, 32)


@pytest.mark.parametrize("n,ratio,sign,code", [
    (0, Fraction(0), 0, AngleV1ErrorCode.UNSUPPORTED_SUBDIVISION),
    (7, Fraction(0), 0, AngleV1ErrorCode.UNSUPPORTED_SUBDIVISION),
    (True, Fraction(0), 0, AngleV1ErrorCode.UNSUPPORTED_SUBDIVISION),
    (2, Fraction(-1, 4), -1, AngleV1ErrorCode.RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL),
    (2, Fraction(1), 1, AngleV1ErrorCode.RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL),
    (2, Fraction(0), 1, AngleV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (2, Fraction(1, 4), 0, AngleV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (2, Fraction(1, 4), True, AngleV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (2, 0.25, 1, AngleV1ErrorCode.NON_RATIONAL_INPUT),
])
def test_atom_input_errors_are_internal_and_exact(n, ratio, sign, code):
    with pytest.raises(AngleV1Error) as failure:
        AngleAtomV1(n, ratio, sign)
    assert failure.value.code is code


def test_ray_error_order_and_bit_count_overflow_match_bounded_rust_api():
    atom = AngleAtomV1(4, Fraction(1, 4), -1)
    for ordinal, bits, code in [
        (0, 2**32 - 1, AngleV1ErrorCode.INVALID_ORDINAL),
        (4, 32, AngleV1ErrorCode.INVALID_ORDINAL),
        (1, 2**31, AngleV1ErrorCode.BIT_COUNT_OVERFLOW),
        (1, -1, AngleV1ErrorCode.INVALID_BIT_COUNT),
    ]:
        with pytest.raises(AngleV1Error) as failure:
            atom.ray_enclosure(ordinal, bits)
        assert failure.value.code is code
    with pytest.raises(AngleV1Error) as failure:
        atom.enclose(2**32 - 1)
    assert failure.value.code is AngleV1ErrorCode.BIT_COUNT_OVERFLOW
    with pytest.raises(AngleV1Error) as failure:
        AngleAtomV1(1, 0, 0).ray(0)
    assert failure.value.code is AngleV1ErrorCode.NO_HIDDEN_ORDINAL


def test_immutable_values_survive_pickle_without_mutable_memo():
    atom = AngleAtomV1(3, Fraction(81, 256), -1)
    result = atom.ray(2)
    assert pickle.loads(pickle.dumps((atom, result))) == (atom, result)
    with pytest.raises(FrozenInstanceError):
        atom.dot_sign = 1
    with pytest.raises(FrozenInstanceError):
        result.root.stats.bisections = 0


def test_value_module_has_no_symbolic_float_or_transcendental_dependency():
    import cftuv_envelope.reference.density_angle_v1 as module

    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imports = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert imports == {"__future__", "dataclasses", "enum", "fractions", "math"}
    math_import = next(node for node in tree.body
                       if isinstance(node, ast.ImportFrom) and node.module == "math")
    assert [alias.name for alias in math_import.names] == ["isqrt"]
    assert not any(isinstance(node, ast.Constant) and isinstance(node.value, float)
                   for node in ast.walk(tree))
