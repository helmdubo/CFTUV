"""A2 bounds не становятся admission; композиция может дать честный internal gap."""

from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import subprocess

import pytest

from cftuv_envelope.reference.density_angle_v1 import AngleAtomV1, FractionIntervalV1
from cftuv_envelope.reference.density_angle_value_v1 import (
    AngleValueError, AngleValueErrorCode, AngleValueRegistryV1,
    canonical_pi_recipe, interval_divide, interval_integer_power,
)
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2
from cftuv_envelope.reference.density_sign import _density_exact_sign
from cftuv_envelope.reference.density_trig_v1 import owned_exact_q_trig, principal_pi_multiple


def _pair(arena, atom, ordinal, canonical=False):
    if canonical:
        multiple = principal_pi_multiple(atom.ratio, atom.dot_sign) * Fraction(ordinal, atom.subdivision)
        angle = arena.scaled(arena.pi(), multiple)
    else:
        cosine = arena.scaled(arena.sqrt(arena.rational(atom.ratio)), atom.dot_sign)
        sine = arena.sqrt(arena.rational(1 - atom.ratio))
        angle = arena.scaled(arena.raw_atan2(sine, cosine), Fraction(ordinal, atom.subdivision))
    return arena.raw_cos(angle), arena.raw_sin(angle)


def test_exact_a2_value_does_not_imply_integer_literal_decimal_quantum():
    from cftuv_envelope.reference.adaptive_density_atlas import _exact_decimal_interval
    from cftuv_envelope.reference.density_binding_values import _density_decimal_envelope
    arena = DensityConstructArenaV1()
    atom = AngleAtomV1(2, Fraction(1, 64), 1)
    cosine, sine = _pair(arena, atom, 1)
    registry = AngleValueRegistryV1(arena)
    registry.register_ray(cosine, sine, atom, 1)
    scaled = arena.scaled(cosine, Fraction(4, 3))
    assert arena.owned_form(cosine).kind == "cos"
    assert arena.owned_form(scaled).kind == "Mul" and arena.scalar(scaled) is None
    assert registry.enclosure(cosine) == FractionIntervalV1.point(Fraction(3, 4))
    assert registry.enclosure(scaled) == FractionIntervalV1.point(1)
    literal = arena.to_sympy_readonly(scaled)
    assert literal not in (-1, 1)
    shell = _density_decimal_envelope(literal)
    assert shell.lower.as_tuple().exponent == shell.upper.as_tuple().exponent == -27
    assert _exact_decimal_interval(1).lower.as_tuple().exponent == 0


def test_raw_atan_semantic_bounds_leave_literal_and_old_sign_unchanged():
    arena = DensityConstructArenaV1()
    atom = AngleAtomV1(4, Fraction(0), 0)
    cosine, sine = _pair(arena, atom, 1)
    old_literal = (arena.export_raw(cosine), arena.export_raw(sine))
    old_signs = tuple(_density_exact_sign(arena.to_sympy_readonly(v)) for v in (cosine, sine))
    registry = AngleValueRegistryV1(arena)
    with pytest.raises(AngleValueError) as failure:
        registry.enclosure(cosine)
    assert failure.value.code is AngleValueErrorCode.UNREGISTERED_OPAQUE
    registry.register_ray(cosine, sine, atom, 1)
    expected = atom.ray(1)
    assert registry.enclosure(cosine) == expected.cosine
    assert registry.enclosure(sine) == expected.sine
    assert (arena.export_raw(cosine), arena.export_raw(sine)) == old_literal
    assert tuple(_density_exact_sign(arena.to_sympy_readonly(v)) for v in (cosine, sine)) == old_signs


def test_foreign_arena_precedes_memo_and_conflicting_metadata_never_overwrites():
    arena = DensityConstructArenaV1()
    atom = AngleAtomV1(4, Fraction(0), 0)
    cosine, sine = _pair(arena, atom, 1)
    registry = AngleValueRegistryV1(arena)
    registry.register_ray(cosine, sine, atom, 1)
    saved = registry.enclosure(cosine)
    registry.register_ray(cosine, sine, atom, 1)
    assert registry.enclosure(cosine) == saved
    with pytest.raises(AngleValueError) as failure:
        registry.register_ray(cosine, sine, AngleAtomV1(2, Fraction(1, 2), 1), 1)
    assert failure.value.code is AngleValueErrorCode.CONFLICTING_REGISTRATION
    assert registry.enclosure(cosine) == saved
    other = DensityConstructArenaV1()
    for number in range(cosine.node + 1):
        foreign = other.rational(number + 100)
    assert foreign.node in registry._memo
    with pytest.raises(AngleValueError) as failure:
        registry.enclosure(foreign)
    assert failure.value.code is AngleValueErrorCode.FOREIGN_ARENA


def test_numeric_whole_v2_island_cannot_be_overridden_by_registry():
    arena = DensityConstructArenaV1()
    value = arena.numeric(ScalarV2.sqrt_rational(2).add(ScalarV2.sqrt_rational(3)))
    form = arena.owned_form(value)
    assert form.kind == "Numeric" and arena.literal_tree(value)[0] == "Add"
    registry = AngleValueRegistryV1(arena)
    registry.register_ray(value, arena.rational(0), AngleAtomV1(4, 0, 0), 1)
    endpoints, _ = form.payload.value.enclosure(160)
    assert registry.enclosure(value) == FractionIntervalV1(*endpoints)


def test_canonical_pi_recipe_has_fixed_tuple_order_and_non_numeric_coverage():
    assert canonical_pi_recipe(Fraction(1, 8)) == (AngleAtomV1(2, Fraction(1, 2), 1), 1)
    assert canonical_pi_recipe(Fraction(5, 36)) == (AngleAtomV1(6, Fraction(3, 4), -1), 1)
    arena = DensityConstructArenaV1()
    registry = AngleValueRegistryV1(arena)
    atom, ordinal = canonical_pi_recipe(Fraction(5, 36))
    cosine, sine = _pair(arena, atom, ordinal, canonical=True)
    registry.register_canonical_ray(cosine, sine, Fraction(5, 36))
    assert registry.enclosure(cosine) == atom.ray(ordinal).cosine
    assert registry.enclosure(sine) == atom.ray(ordinal).sine
    with pytest.raises(AngleValueError) as failure:
        registry.register_canonical_ray(cosine, sine, Fraction(1, 7))
    assert failure.value.code is AngleValueErrorCode.NO_CANONICAL_RECIPE
    # Одна и та же pi/8 literal не зависит от первого эквивалентного atom descriptor.
    cosine, sine = _pair(arena, AngleAtomV1(4, 0, 0), 1, canonical=True)
    registry.register_ray(cosine, sine, AngleAtomV1(4, 0, 0), 1)
    fixed = AngleAtomV1(2, Fraction(1, 2), 1).ray(1)
    assert registry.enclosure(cosine) == fixed.cosine
    registry.register_ray(cosine, sine, AngleAtomV1(2, Fraction(1, 2), 1), 1)
    assert registry.enclosure(cosine) == fixed.cosine


def test_composition_and_nested_q_five_have_closed_outward_bounds():
    arena = DensityConstructArenaV1()
    registry = AngleValueRegistryV1(arena)
    cosine, sine = owned_exact_q_trig(arena, 5)
    c, s = registry.enclosure(cosine), registry.enclosure(sine)
    assert c.lower > 0 and s.lower > 0
    residual = arena.sub(arena.add(arena.integer_power(cosine, 2), arena.integer_power(sine, 2)), arena.rational(1))
    bounds = registry.enclosure(residual)
    assert bounds.lower <= 0 <= bounds.upper
    quotient = arena.partial_div(sine, cosine)
    # Numeric coefficient regrouping сохраняет whole-V2 boundary при композиции.
    bounds = registry.enclosure(quotient)
    tan_squared = ScalarV2.sqrt_rational(5).scaled(-2).add(5)
    assert ScalarV2.rational(bounds.lower**2).sub(tan_squared).value.signum()[0] <= 0
    assert ScalarV2.rational(bounds.upper**2).sub(tan_squared).value.signum()[0] >= 0
    assert interval_integer_power(FractionIntervalV1(-2, 1), 2) == FractionIntervalV1(0, 4)


def test_component_160_does_not_guarantee_reciprocal_even_when_legacy_sign_is_positive():
    arena = DensityConstructArenaV1()
    atom = AngleAtomV1(2, 1 - Fraction(1, 10**200), 1)
    cosine, sine = _pair(arena, atom, 1)
    assert _density_exact_sign(arena.to_sympy_readonly(sine)) == 1
    registry = AngleValueRegistryV1(arena)
    registry.register_ray(cosine, sine, atom, 1)
    bounds = registry.enclosure(sine)
    assert bounds.lower == 0 < bounds.upper < Fraction(1, 2**160)
    reciprocal = arena.reciprocal(sine)
    with pytest.raises(AngleValueError) as failure:
        registry.enclosure(reciprocal)
    assert failure.value.code is AngleValueErrorCode.DENOMINATOR_CONTAINS_ZERO
    assert _density_exact_sign(arena.to_sympy_readonly(sine)) == 1


def _live_cases():
    for mode, ratios in [("plain", (Fraction(0), Fraction(1, 4), Fraction(81, 256))),
                         ("canonical", (Fraction(0), Fraction(1, 4), Fraction(1, 2), Fraction(3, 4)))]:
        for n in range(2, 7):
            for ratio in ratios:
                for sign in ((0,) if not ratio else (-1, 1)):
                    for j in range(1, n):
                        yield mode, "cos", n, ratio, sign, j
    for mode, n, ratio, sign in [("plain", 4, Fraction(0), 0),
                                  ("canonical", 6, Fraction(3, 4), -1), ("q", 5, Fraction(0), 0)]:
        for operation in ("sum", "mul", "unit", "ratio", "reciprocal", "sqrt"):
            yield mode, operation, n, ratio, sign, 1
    yield "plain", "reciprocal", 2, 1 - Fraction(1, 10**200), 1, 1


def _live_expected(case):
    mode, operation, n, ratio, sign, j = case
    arena = DensityConstructArenaV1()
    registry = AngleValueRegistryV1(arena)
    atom = AngleAtomV1(n, ratio, sign)
    cosine, sine = owned_exact_q_trig(arena, n) if mode == "q" else _pair(arena, atom, j, mode == "canonical")
    if mode != "q":
        registry.register_ray(cosine, sine, atom, j)
    operations = {
        "cos": lambda: cosine, "sin": lambda: sine,
        "sum": lambda: arena.add(cosine, sine), "mul": lambda: arena.mul(cosine, sine),
        "unit": lambda: arena.sub(arena.add(arena.integer_power(cosine, 2),
                                           arena.integer_power(sine, 2)), arena.rational(1)),
        "ratio": lambda: arena.partial_div(sine, cosine),
        "reciprocal": lambda: arena.reciprocal(sine), "sqrt": lambda: arena.sqrt(sine),
    }
    expression = operations[operation]()
    try:
        value = registry.enclosure(expression)
    except AngleValueError as error:
        return f"GAP\t{error.code.value}"
    endpoints = [registry.enclosure(cosine), registry.enclosure(sine), value]
    return "\t".join(["VALUE", *(f"{bound.numerator}/{bound.denominator}"
                                  for interval in endpoints for bound in (interval.lower, interval.upper))])


@pytest.mark.skipif(not os.environ.get("CFTUV_DENSITY_ANGLE_VALUE_EXE"), reason="explicit native paired gate")
def test_live_native_owned_value_composition_has_identical_exact_bounds_and_gaps():
    executable = Path(os.environ["CFTUV_DENSITY_ANGLE_VALUE_EXE"])
    output = Path(os.environ["CFTUV_DENSITY_ANGLE_VALUE_OUT"])
    output.mkdir(parents=True, exist_ok=True)
    cases = list(_live_cases())
    program = "\n".join("\t".join((mode, operation, str(n),
                         f"{ratio.numerator}/{ratio.denominator}", str(sign), str(j)))
                        for mode, operation, n, ratio, sign, j in cases) + "\n"
    process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    output.joinpath("pids.json").write_text(json.dumps({"python": os.getpid(), "rust": process.pid}), encoding="utf-8")
    native, errors = process.communicate(program)
    output.joinpath("inputs.tsv").write_text(program, encoding="utf-8")
    output.joinpath("native.stdout").write_text(native, encoding="utf-8")
    output.joinpath("native.stderr").write_text(errors, encoding="utf-8")
    assert process.returncode == 0 and not errors
    rows = native.splitlines()
    assert len(rows) == len(cases)
    observations = [{"input": line, "native": row, "python": _live_expected(case)}
                    for line, row, case in zip(program.splitlines(), rows, cases, strict=True)]
    failures = [row for row in observations if row["native"] != row["python"]]
    output.joinpath("observations.json").write_text(json.dumps(observations, indent=2), encoding="utf-8")
    receipt = {"cases": len(cases), "mismatches": len(failures), "failures": failures,
               "binary_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(),
               "explicit_denominator_gaps": sum(row.startswith("GAP") for row in rows)}
    output.joinpath("receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    assert not failures
