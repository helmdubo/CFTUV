from decimal import Decimal
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
import cftuv_envelope.reference.compile as compile_module
import cftuv_envelope.reference.direction_binding as binding
from cftuv_envelope import TurnOrientation
from cftuv_envelope.reference.angular import _interpolated_normals
from cftuv_envelope.reference.direction_binding import (
    BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    BINDING_SUBTURN_LE_DELTA_MAX,
    DirectionBindingCertificateUnproven,
    _ExactWindowUnsupported,
    _FastK1Geometry,
    _ProjectiveRay,
    _common_rays_with_lengths,
    _decimal_envelope_ray,
    _denominator_sign,
    _fast_subturn,
    _raw_cross_with_candidate,
)
from cftuv_envelope.reference.direction_window_exact import (
    _MQField,
    _TowerField,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector


_FULL = (
    Path(__file__).parents[1]
    / "fixtures"
    / "building_002_full_selection_v1"
)


def _k1_input():
    metric = ExactPlanarMetric(
        ((sp.Rational(2), sp.Rational(0)), (sp.Rational(0), sp.Rational(1))),
        (
            (sp.Rational(1, 2), sp.Rational(0)),
            (sp.Rational(0), sp.Rational(1)),
        ),
        1,
    )
    normals = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(1, 0),
        ExactPlanarVector.from_values(0, 1),
        1,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
    )
    return metric, normals


def test_multiquadratic_basis_reduces_rational_square_dependencies():
    field = _MQField()
    root_two = field.radical(2)
    root_eight = field.radical(8)

    assert root_eight == root_two.scaled(2)
    assert (root_eight - root_two.scaled(2)).sign() == 0
    assert field.generators == [Fraction(2)]


def test_multiquadratic_sign_closes_without_numeric_tolerance():
    field = _MQField()
    root_two = field.radical(2)
    value = field.rational(3) - root_two.scaled(2)

    assert value.sign() > 0
    lower, upper = value.enclosure(96)
    assert 0 < lower <= upper


def test_tower_structural_boundary_is_an_exact_zero():
    base = _MQField()
    extension = _TowerField(base, base.rational(4))
    boundary = extension.rational(2) - extension.root()

    assert boundary.sign() == 0
    assert boundary.enclosure(96)[0] <= 0 <= boundary.enclosure(96)[1]


def test_k1_window_path_never_calls_symbolic_solvers(monkeypatch):
    metric, normals = _k1_input()

    def forbidden(*args, **kwargs):
        raise AssertionError("symbolic solver entered the exact K1 window path")

    for name in ("factor", "cancel", "ask", "nsimplify"):
        monkeypatch.setattr(sp, name, forbidden)
    monkeypatch.setattr(sp.Expr, "equals", forbidden)

    certificates = binding._certify_k1_recipe_direction_bindings(
        metric,
        normals[0],
        normals[-1],
        normals,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
    )
    binding._verify_k1_recipe_direction_bindings(
        metric,
        normals[0],
        normals[-1],
        normals,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        certificates,
    )
    assert certificates[0].bound_primitive_integer_vector == (1, 1)


def test_public_tuple_api_honors_a_forged_middle_vector(monkeypatch):
    metric, normals = _k1_input()
    forged = (normals[0], normals[0], normals[-1])

    def forbidden(*args, **kwargs):
        raise AssertionError("public tuple API entered the private K1 recipe path")

    monkeypatch.setattr(binding, "_certify_fast_k1", forbidden)
    certificates = binding.certify_direction_bindings(
        metric,
        forged,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
    )

    assert certificates[0].bound_primitive_integer_vector == (2, 1)


def test_full_selection_proof_depths_are_frozen(monkeypatch):
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (_FULL / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (_FULL / "decal_request.json").read_bytes()
    )
    original_certify = compile_module._certify_k1_recipe_direction_bindings
    original_window = binding._fast_certificate_in_window
    active = [-1]
    attempts: list[int] = []

    def counted_certify(*args, **kwargs):
        attempts.append(0)
        active[0] = len(attempts) - 1
        try:
            return original_certify(*args, **kwargs)
        finally:
            active[0] = -1

    def counted_window(*args, **kwargs):
        if active[0] >= 0:
            attempts[active[0]] += 1
        return original_window(*args, **kwargs)

    monkeypatch.setattr(
        compile_module,
        "_certify_k1_recipe_direction_bindings",
        counted_certify,
    )
    monkeypatch.setattr(binding, "_fast_certificate_in_window", counted_window)
    result = compile_module.compile_reference_envelopes(snapshot, request)

    assert result.outcome is kernel.ReferenceOutcome.EXACT
    assert attempts == [1, 2, 1, 2]


def test_depth_beyond_two_returns_to_the_symbolic_solver(monkeypatch):
    metric, normals = _k1_input()
    expected = binding._certify_direction_bindings_symbolic(
        metric,
        normals,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
    )
    attempts = [0]
    original_window = binding._fast_certificate_in_window

    def counted_window(*args, **kwargs):
        attempts[0] += 1
        return original_window(*args, **kwargs)

    monkeypatch.setattr(binding, "_fast_certificate_in_window", counted_window)
    monkeypatch.setattr(
        binding,
        "_fast_failed_predicate",
        lambda *args, **kwargs: BINDING_SUBTURN_LE_DELTA_MAX,
    )
    actual = binding._certify_k1_recipe_direction_bindings(
        metric,
        normals[0],
        normals[-1],
        normals,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
    )

    assert attempts == [2]
    assert actual == expected


def test_cross_root_midpoint_is_a_private_fallback_boundary():
    field = _MQField()
    root_two = field.radical(2)
    left_squared = field.rational(2) + root_two
    right_squared = field.rational(3) + root_two
    left = _ProjectiveRay(
        field,
        field.rational(1),
        field.rational(0),
        left_squared,
    )
    right = _ProjectiveRay(
        field,
        field.rational(0),
        field.rational(1),
        right_squared,
    )

    with pytest.raises(_ExactWindowUnsupported, match="cross-root"):
        _common_rays_with_lengths(left, right)


def test_exact_slope_grid_boundary_and_zero_denominator():
    field = _MQField()
    ray = _ProjectiveRay(
        field,
        field.rational(10**27),
        field.rational(1),
        field.rational(1),
    )
    envelope = _decimal_envelope_ray(ray, True)
    assert envelope.lower == envelope.upper == Decimal("1E-27")
    assert envelope.absolute_error_bound == 0

    vertical = _ProjectiveRay(
        field,
        field.rational(0),
        field.rational(1),
        field.rational(1),
    )
    with pytest.raises(
        DirectionBindingCertificateUnproven,
        match=BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    ):
        _denominator_sign(vertical, True)


def test_collinear_cross_and_exact_sixty_degree_are_not_approximated():
    field = _MQField()
    ray = _ProjectiveRay(
        field,
        field.rational(2),
        field.rational(3),
        field.rational(1),
    )
    assert _raw_cross_with_candidate(ray, (2, 3)).sign() == 0

    geometry = _FastK1Geometry(
        ((Fraction(1), Fraction(0)), (Fraction(0), Fraction(3))),
        1,
        None,
        None,
        None,
        None,
        None,
        (1, 0),
        (1, 1),
    )
    assert _fast_subturn(geometry, (1, 0), (1, 1))
