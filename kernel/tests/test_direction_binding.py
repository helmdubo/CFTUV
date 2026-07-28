from __future__ import annotations

from dataclasses import replace

import pytest
import sympy as sp

from cftuv_envelope import TurnOrientation
from cftuv_envelope.reference.angular import _interpolated_normals
from cftuv_envelope.reference.direction_binding import (
    BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    BINDING_MONOTONE,
    BINDING_SUBTURN_LE_DELTA_MAX,
    DirectionBindingCertificateUnproven,
    bound_unit_normal,
    certify_direction_bindings,
    has_rational_support_direction,
    verify_direction_bindings,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector

from direction_binding_oracle import assert_k2_identity_binding


def _metric() -> ExactPlanarMetric:
    return ExactPlanarMetric(
        ((sp.Rational(2), sp.Rational(0)), (sp.Rational(0), sp.Rational(1))),
        (
            (sp.Rational(1, 2), sp.Rational(0)),
            (sp.Rational(0), sp.Rational(1)),
        ),
        1,
    )


def _ideal(
    incoming=(1, 0),
    outgoing=(0, 1),
    orientation=TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
):
    metric = _metric()
    normals = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(*incoming),
        ExactPlanarVector.from_values(*outgoing),
        1,
        orientation,
    )
    return metric, normals


def test_stern_brocot_binding_records_a_primitive_rational_support():
    metric, normals = _ideal()
    assert has_rational_support_direction(metric, normals[0])
    assert not has_rational_support_direction(metric, normals[1])
    assert has_rational_support_direction(metric, normals[2])

    (certificate,) = certify_direction_bindings(
        metric, normals, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    assert certificate.bound_primitive_integer_vector == (1, 1)
    assert certificate.certified_window_width_lower_bound > 0
    verify_direction_bindings(
        metric,
        normals,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        (certificate,),
    )
    bound = bound_unit_normal(metric, certificate)
    assert metric.dot_g(bound, bound) == 1
    assert has_rational_support_direction(metric, bound)


@pytest.mark.parametrize(
    ("vector", "predicate"),
    [
        ((-1, -1), BINDING_MONOTONE),
        ((1, 3), BINDING_INSIDE_OWN_ORDINAL_WINDOW),
        ((2, 3), BINDING_SUBTURN_LE_DELTA_MAX),
    ],
)
def test_verifier_names_each_forged_predicate(vector, predicate):
    metric, normals = _ideal()
    (certificate,) = certify_direction_bindings(
        metric, normals, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    forged = replace(
        certificate,
        bound_primitive_integer_vector=vector,
    )
    with pytest.raises(
        DirectionBindingCertificateUnproven,
        match=rf"привязка: сертификат не доказан \({predicate}\)",
    ):
        verify_direction_bindings(
            metric,
            normals,
            TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
            (forged,),
        )


def test_binding_is_equivariant_under_positive_scale_and_global_sign():
    metric, normals = _ideal()
    base = certify_direction_bindings(
        metric, normals, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    scaled = tuple(
        ExactPlanarVector.from_values(
            normal.x.as_expr() * 7,
            normal.y.as_expr() * 7,
        )
        for normal in normals
    )
    assert certify_direction_bindings(
        metric, scaled, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    ) == base

    negated = tuple(
        ExactPlanarVector.from_values(
            -normal.x.as_expr(),
            -normal.y.as_expr(),
        )
        for normal in normals
    )
    (opposite,) = certify_direction_bindings(
        metric, negated, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    assert opposite.bound_primitive_integer_vector == (-1, -1)


def test_vertical_chart_and_reversed_endpoint_order_are_exact():
    metric, vertical = _ideal(incoming=(0, 1), outgoing=(-1, 0))
    (certificate,) = certify_direction_bindings(
        metric, vertical, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    )
    verify_direction_bindings(
        metric,
        vertical,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        (certificate,),
    )
    assert certificate.bound_primitive_integer_vector[0] < 0
    assert certificate.bound_primitive_integer_vector[1] > 0

    metric, forward = _ideal()
    reversed_normals = tuple(reversed(forward))
    reversed_certificate = certify_direction_bindings(
        metric,
        reversed_normals,
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
    )
    verify_direction_bindings(
        metric,
        reversed_normals,
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
        reversed_certificate,
    )
    assert (
        reversed_certificate[0].bound_primitive_integer_vector
        == (1, 1)
    )


def test_k2_recorded_vectors_pass_the_independent_decimal_cubic_oracle():
    metric = ExactPlanarMetric(
        ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1))),
        ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1))),
        1,
    )
    # Явный certified cubic fixture: production SymPy solve здесь не зовётся.
    normals = tuple(
        ExactPlanarVector.from_values(x, y)
        for x, y in (
            (1, 0),
            (sp.Rational(3, 5), sp.Rational(-4, 5)),
            (sp.Rational(-7, 25), sp.Rational(-24, 25)),
            (sp.Rational(-117, 125), sp.Rational(-44, 125)),
        )
    )
    certificates = certify_direction_bindings(
        metric,
        normals,
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
    )
    vectors = tuple(
        certificate.bound_primitive_integer_vector
        for certificate in certificates
    )
    assert_k2_identity_binding(vectors)
