"""Две реальные prepared principal формы; не generic AST/trig simplifier."""

from fractions import Fraction
from pathlib import Path
import sys

import pytest
import sympy as sp

from kernel_test_paths import KERNEL_ROOT

sys.path.insert(0, str(KERNEL_ROOT / "tools"))
from density_owned_laws_v1_diff import _PreparedImporter
from cftuv_envelope.reference.density_angle_v1 import AngleAtomV1, FractionIntervalV1
from cftuv_envelope.reference.density_angle_value_v1 import AngleValueError, AngleValueErrorCode, AngleValueRegistryV1
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import DensityOracleLimitsV1
from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1


def _importer():
    limits = DensityOracleLimitsV1(65536, 131072, 96, 1 << 20, ExpandLimitsV1(4096, 16, 65536))
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits)
    return _PreparedImporter(laws)


# Значения из OLD kernel/field failures, включая обе стороны phi=pi/2.
_RECORDED = (
    (3, 1, 0, sp.Rational(824243, 25836830)),
    (3, 2, -1, sp.Rational(824243, 25836830)),
    (4, 1, 0, sp.Rational(323259515397890, 107859600856367)),
    (4, 3, -1, sp.Rational(323259515397890, 107859600856367)),
    (5, 3, -1, sp.Rational(44, 117)), (5, 4, -1, sp.Rational(44, 117)),
    (6, 4, -1, sp.Rational(44, 117)), (6, 5, -1, sp.Rational(44, 117)),
    (3, 2, -1, sp.Rational(106, 1109475369438447078367128560387512551)
     * sp.sqrt(1181078942632300604368038323254095655200724511333709510885673274914534409)),
)


@pytest.mark.parametrize("n,j,shift,tangent", _RECORDED)
def test_recorded_principal_literals_get_only_value_metadata(n, j, shift, tangent):
    importer = _importer()
    argument = (sp.Rational(j, n) + sp.Rational(shift, 2))*sp.pi - sp.Rational(j, n)*sp.atan(tangent)
    source = (sp.cos(argument, evaluate=False), sp.sin(argument, evaluate=False))
    cosine, sine = tuple(importer.expression(value).value for value in source)
    arena = importer.laws.arena
    before = tuple(arena.export_raw(value) for value in (cosine, sine))
    count, gaps = importer.register_angles(source, n)
    assert count == 1 and gaps == []
    assert tuple(arena.export_raw(value) for value in (cosine, sine)) == before
    squared = tangent*tangent
    assert squared.is_Rational
    atom = AngleAtomV1(n, Fraction(1 / (1 + squared)), -1)
    ray = atom.ray(j)
    expected = (ray.cosine, ray.sine) if not shift else (ray.sine, FractionIntervalV1.point(0).sub(ray.cosine))
    assert tuple(importer.laws.values.enclosure(value) for value in (cosine, sine)) == expected
    assert importer.register_angles(source, n) == (1, [])


@pytest.mark.parametrize("argument,n", [
    (sp.pi/3 + sp.atan(sp.Rational(44, 117))/3, 3),
    (sp.pi/4 - sp.atan(sp.Rational(44, 117))/3, 3),
    (sp.pi/3 - sp.atan(sp.Rational(44, 117))/4, 3),
    (sp.pi/2 - sp.atan(sp.Rational(44, 117)), 3),
    (sp.pi/3 - sp.atan(sp.Rational(-44, 117), evaluate=False)/3, 3),
    (sp.pi/3 - sp.atan(sp.Rational(44, 117))/3, 7),
    (sp.pi/3 - sp.atan(sp.Rational(44, 117))/3, True),
])
def test_unobserved_shift_invalid_ordinal_or_wrong_principal_branch_stays_unregistered(argument, n):
    importer = _importer()
    expression = sp.cos(argument, evaluate=False)
    owned = importer.expression(expression).value
    count, gaps = importer.register_angles([expression], n)
    assert count == 0 and gaps == ["OUTSIDE_PRINCIPAL_RECIPE"]
    with pytest.raises(AngleValueError) as failure:
        importer.laws.values.enclosure(owned)
    assert failure.value.code is AngleValueErrorCode.UNREGISTERED_OPAQUE


@pytest.mark.parametrize("ratio,sign,n,j", [(Fraction(81, 100), -1, 3, 2), (Fraction(81, 100), -1, 4, 1)])
def test_closed_quarter_registry_preserves_bounds_sign_literal_and_atomic_conflicts(ratio, sign, n, j):
    importer = _importer()
    arena = importer.laws.arena
    atom = AngleAtomV1(n, ratio, sign)
    cosine = arena.scaled(arena.sqrt(arena.rational(ratio)), sign)
    sine = arena.sqrt(arena.rational(1-ratio))
    principal = arena.raw_atan2(sine, cosine)
    shifted = arena.sub(arena.scaled(principal, Fraction(j, n)), arena.scaled(arena.pi(), Fraction(1, 2)))
    pair = arena.raw_cos(shifted), arena.raw_sin(shifted)
    registry = AngleValueRegistryV1(arena)
    before = tuple(arena.export_raw(value) for value in pair)
    registry.register_ray_with_quarter_turns(*pair, atom, j, -1)
    ray = atom.ray(j)
    expected = (ray.sine, FractionIntervalV1(-ray.cosine.upper, -ray.cosine.lower))
    assert tuple(registry.enclosure(value) for value in pair) == expected
    assert (expected[1].lower > 0) if j == 2 else (expected[1].upper < 0)
    assert tuple(arena.export_raw(value) for value in pair) == before
    with pytest.raises(AngleValueError) as failure:
        registry.register_ray(*pair, atom, j)
    assert failure.value.code is AngleValueErrorCode.CONFLICTING_REGISTRATION
    assert tuple(registry.enclosure(value) for value in pair) == expected
    for invalid in (1, -2, True):
        with pytest.raises(AngleValueError) as failure:
            registry.register_ray_with_quarter_turns(*pair, atom, j, invalid)
        assert failure.value.code is AngleValueErrorCode.INVALID_REGISTRATION
    foreign = DensityConstructArenaV1()
    with pytest.raises(AngleValueError) as failure:
        registry.register_ray_with_quarter_turns(foreign.rational(0), foreign.rational(1), atom, j, 1)
    assert failure.value.code is AngleValueErrorCode.FOREIGN_ARENA
