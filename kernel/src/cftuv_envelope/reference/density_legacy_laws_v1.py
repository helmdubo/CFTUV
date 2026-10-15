"""Explicit frozen leaf adapter for the shared Density controller.

This adapter calls existing leaf laws verbatim. It never constructs a candidate
owned scalar, changes caches, or changes default caps/Decimal context.
"""

from fractions import Fraction

from . import adaptive_density_band as band
from . import density_projective as projective


class DensityLegacyLawsV1:
    preparation_cap = None
    search_cap = None
    box_cap = None

    dual_dot = staticmethod(projective._dual_dot)
    oriented_cross = staticmethod(projective._oriented_cross)
    sign = staticmethod(projective._sign)
    vector = staticmethod(projective._vector)
    inside_ordinal = staticmethod(projective._inside_ordinal)
    slope = staticmethod(projective._slope)
    chart_for_window = staticmethod(projective._chart_for_window)
    subturn = staticmethod(projective._subturn)
    subturn_boundary = staticmethod(projective._subturn_boundary)
    quarter_turn = staticmethod(projective._quarter_turn)
    exact_q_trig = staticmethod(projective._exact_q_trig)
    rotate_q = staticmethod(projective._rotate_q)
    subturn_boundary_vectors = staticmethod(projective._subturn_boundary_vectors)
    window_neighbours = staticmethod(band.window_neighbours)
    window_triple = staticmethod(band.window_triple)
    fraction = staticmethod(Fraction)

    @staticmethod
    def strict_between(lower, value, upper, metric=None):
        return lower < value < upper

    @staticmethod
    def literal_unit(value):
        return int(value) if value in (-1, 1) else None

    @staticmethod
    def preparation_budget(factory):
        return factory()

    @staticmethod
    def search_budget(factory):
        return factory()

    @staticmethod
    def envelope(expression, metric=None):
        from .density_binding_values import _density_decimal_envelope

        return _density_decimal_envelope(expression, metric)

    @staticmethod
    def width(lower, upper):
        return upper - lower
