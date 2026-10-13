"""Owned projective atlas keeps exact containment at the explicit sign seam."""

from fractions import Fraction

import pytest

from cftuv_envelope.reference.adaptive_density_atlas import build_atlas_window_record
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import DensityOracleLimitsV1
from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1


@pytest.mark.parametrize("direction", (-1, 1))
def test_owned_atlas_containment_uses_certified_sign_without_scalar_order(direction):
    limits = DensityOracleLimitsV1(65536, 131072, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits)
    metric = laws.metric((1, 0, 1), (1, 0, 1), 1)
    vectors = [laws.vector(1, Fraction(1, 2)), laws.vector(Fraction(1, 2), 1),
               laws.vector(Fraction(-1, 2), 1)]
    left, ideal, right = vectors if direction == 1 else reversed(vectors)
    record = build_atlas_window_record(
        metric, ideal, left, right, sign=laws.sign, slope=laws.slope,
        vector=laws.vector, decimal_envelope=laws.envelope, invalid=ValueError,
        strict_between=laws.strict_between,
        literal_unit=laws.literal_unit,
    )
    assert len(record.pieces) == 2
    assert record.termination_piece_index == (1 if direction == 1 else 0)
    with pytest.raises(TypeError):
        Fraction(0) < laws.slope(ideal, False, metric)
    assert not laws.budgets
    piece = record.pieces[0 if direction == 1 else 1]
    assert piece.upper_slope_envelope.upper.as_tuple().exponent == 0


def test_owned_literal_unit_does_not_promote_A2_exact_value_to_numeric_sentinel():
    limits = DensityOracleLimitsV1(65536, 131072, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits)
    assert laws.literal_unit(laws.scalar(-1)) == -1
    assert laws.literal_unit(laws.scalar(1)) == 1
    cosine, _ = laws.principal_pair(Fraction(1, 64), 1, 2, 1)
    expression = Fraction(4, 3)*cosine
    assert laws.values.enclosure(expression.value).lower == 1
    assert laws.literal_unit(expression) is None
    assert laws.envelope(expression).upper.as_tuple().exponent == -27


@pytest.mark.parametrize("value,expected,calls", ((-1, False, 1), (0, False, 1),
                                                  (1, True, 2), (2, False, 2)))
def test_owned_strict_containment_preserves_left_to_right_short_circuit(value, expected, calls):
    limits = DensityOracleLimitsV1(65536, 131072, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits)
    original = laws.sign
    visited = []

    def sign(expression, metric=None):
        visited.append(laws.fraction(expression))
        return original(expression, metric)

    laws.sign = sign
    assert laws.strict_between(Fraction(0), laws.scalar(value), Fraction(2)) is expected
    assert visited == [Fraction(value), Fraction(2-value)][:calls]
