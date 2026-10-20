"""The native `parametric.key_slopes_of` (MATERIALIZE_CERTIFICATE_NATIVE_V1, `materialize_hot.rs` `key_slopes`) against the oracle: the same dict, or a named decline where the
oracle refuses (`ParametricRefusal`) and then names the refusal itself.

The values: every call of the certificate on the kernel's fixtures (field domains, developable strips with cut nodes) captured at the dispatcher, plus synthetic regions for the
identity (an `int` and a `Fraction` coefficient are one coefficient, numbers beyond 64 bits, a slope met twice) and for each refusal.
"""

from __future__ import annotations

import sys
from decimal import Decimal
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "key_slopes"), reason="расширение cftuv_native не собрано (или собрано без key_slopes)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _same(prepared, coverage, templates, points):
    """`"answered"` when both give the same dict (keys, order, values, `None` where the point stands still), `"declined"` when the oracle refuses and the port declines."""

    from cftuv_envelope.materialize import parametric

    try:
        expected = parametric.key_slopes_of(prepared, coverage, templates, points)
    except parametric.ParametricRefusal:
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.key_slopes(prepared, coverage, templates, points)
        return "declined"
    found = cftuv_native.key_slopes(prepared, coverage, templates, points)
    assert type(found) is dict and list(found) == list(expected)
    assert list(found.values()) == list(expected.values())
    assert all((left is None) == (right is None) for left, right in zip(found.values(), expected.values()))
    return "answered"


def _captured():
    """`(prepared, coverage, templates, points)` of every certificate the kernel's fixtures compute, taken at the dispatcher."""

    import materialize_factories as factories
    from developable_route import developable_domain
    import developable_factories as df
    from materialize_factories import prepare_and_cover

    from cftuv_envelope import backend
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.step import step_domain
    from cftuv_envelope.wavefront import prepare_conveyor

    host = (NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    certified = (NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    cases = []
    for name in ("building_002_full_selection_v1", "building_002_point_contact_v1", "mesh2_patch0_cut_fans_v1", "sagging_wall_convex_partition_v1"):
        snapshot, request = factories.load_fixture(name)
        prepared = prepare_conveyor(snapshot, request)
        cases.append((prepared, host, str(prepared.requested_alpha.value)))
    for make, alpha in ((df.fold_strip, "0.3"), (df.slant_fold, "0.7"), (df.quarter_cylinder, "0.7")):
        snapshot, request = developable_domain(make(), ("r0a", "r0b"), alpha="1")
        cases.append((prepare_and_cover(snapshot, request)[0], certified, alpha))
    calls = []
    saved = backend.key_slopes_compute

    def capture(oracle, prepared, coverage, templates, points):
        calls.append((prepared, coverage, dict(templates), dict(points)))
        return oracle(prepared, coverage, templates, points)

    backend.key_slopes_compute = capture
    try:
        for prepared, laws, alpha in cases:
            for text in (alpha, str(Decimal(alpha) * Decimal("1.001"))):
                step_domain(
                    prepared, text, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=laws[0], decal_topology_law=laws[1],
                    digests=False, parametric=True,
                )
    finally:
        backend.key_slopes_compute = saved
    return calls


@needs_native
def test_every_certificate_of_the_kernel_fixtures_gets_the_same_slopes(approved):
    calls = _captured()
    assert len(calls) >= 10
    outcomes = [_same(*call) for call in calls]
    assert outcomes == ["answered"] * len(calls)
    assert any(any(value is not None for value in _answer(call).values()) for call in calls), "no fixture moves a point: the test would prove nothing"


def _answer(call):
    from cftuv_envelope.materialize import parametric

    return parametric.key_slopes_of(*call)


def _region(prepared, faces, contours):
    partition = prepared.regions[0].partition
    return SimpleNamespace(regions=(SimpleNamespace(region_id=prepared.regions[0].region_id, contours=contours),)), {id(partition): SimpleNamespace(faces=faces)}


def _prepared():
    import materialize_factories as factories

    from cftuv_envelope.wavefront import prepare_conveyor

    snapshot, request = factories.load_fixture("building_002_point_contact_v1")
    return prepare_conveyor(snapshot, request)


@needs_native
def test_the_identity_of_a_point_is_its_value(approved):
    """An `int` and a `Fraction` coefficient of one value are one point; numbers beyond 64 bits are compared whole; a point met twice with one slope is not ambiguous."""

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.wavefront.coverage_template import KIND_BEHIND, CutPointV1, FaceTemplateV1, KIND_CUT

    prepared = _prepared()
    big = Fraction(2**130 + 1, 3**50)
    as_int = (SqrtSumV1(((1, 3),)), SqrtSumV1(((1, big), (2, Fraction(1, 2)))))
    twin = Fraction(int(str(big.numerator)), int(str(big.denominator)))  # the same value in other objects: identities compare values, not objects
    assert twin.numerator is not big.numerator
    as_fraction = (SqrtSumV1(((1, Fraction(3)),)), SqrtSumV1(((1, twin), (2, Fraction(1, 2)))))
    near = (SqrtSumV1(((1, 3),)), SqrtSumV1(((1, big + Fraction(1, 3**50)), (2, Fraction(1, 2)))))
    slope = SqrtSumV1(((1, Fraction(-2**70, 7)),))
    other = SqrtSumV1(((3, Fraction(1, 5)),))
    faces = (
        FaceTemplateV1("a", KIND_CUT, (0, CutPointV1(as_fraction[0], as_fraction[1], slope, other)), None, 1),
        FaceTemplateV1("b", KIND_BEHIND, (), SqrtSumV1.zero()),
        FaceTemplateV1("c", KIND_CUT, (CutPointV1(as_fraction[0], as_fraction[1], slope, other), CutPointV1(near[0], near[1], other, slope)), None, 2),
    )
    contours = (SimpleNamespace(points=(None, as_fraction)), SimpleNamespace(points=()), SimpleNamespace(points=(as_fraction, near)))
    coverage, templates = _region(prepared, faces, contours)
    points = {"k1": as_fraction, "k2": near, "k3": (SqrtSumV1.zero(), SqrtSumV1.zero()), "k4": as_int}
    assert _same(prepared, coverage, templates, points) == "answered"
    found = cftuv_native.key_slopes(prepared, coverage, templates, points)
    assert found["k1"] == (slope, other) and found["k4"] == (slope, other) and found["k2"] == (other, slope) and found["k3"] is None
    # nothing moves: the empty dict, whatever the points
    still = (FaceTemplateV1("a", KIND_CUT, (0, 1, 2), None, 0),)
    coverage, templates = _region(prepared, still, (SimpleNamespace(points=(as_int, as_int, as_int)),))
    assert _same(prepared, coverage, templates, points) == "answered" and cftuv_native.key_slopes(prepared, coverage, templates, points) == {}


@needs_native
def test_every_refusal_of_the_oracle_is_declined(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.wavefront.coverage_template import CutPointV1, FaceTemplateV1, KIND_CUT

    prepared = _prepared()
    point = (SqrtSumV1.rational(3), SqrtSumV1.rational(4))
    one, two = SqrtSumV1.rational(1), SqrtSumV1.rational(2)
    two_slopes = (FaceTemplateV1("a", KIND_CUT, (CutPointV1(*point, one, one),), None, 1), FaceTemplateV1("b", KIND_CUT, (CutPointV1(*point, two, one),), None, 1))
    pairs = (SimpleNamespace(points=(point,)), SimpleNamespace(points=(point,)))
    region = prepared.regions[0]
    cases = [
        _region(prepared, two_slopes, pairs),
        (SimpleNamespace(regions=(SimpleNamespace(region_id=region.region_id, contours=None),)), {}),
        (SimpleNamespace(regions=(SimpleNamespace(region_id=region.region_id, contours=()),)), {}),
        _region(prepared, two_slopes[:1], (SimpleNamespace(points=(point, point)),)),
        _region(prepared, two_slopes, pairs[:1]),
    ]
    assert [_same(prepared, coverage, templates, {"k": point}) for coverage, templates in cases] == ["declined"] * len(cases)


@needs_native
def test_what_the_port_does_not_carry_is_declined_and_the_oracle_answers(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.wavefront.coverage_template import CutPointV1, FaceTemplateV1, KIND_CUT

    prepared = _prepared()
    point = (SqrtSumV1.rational(3), SqrtSumV1.rational(4))
    one = SqrtSumV1.rational(1)
    faces = (FaceTemplateV1("a", KIND_CUT, (CutPointV1(*point, one, one),), None, 1),)
    coverage, templates = _region(prepared, faces, (SimpleNamespace(points=(point,)),))
    for points in ({"k": [point[0], point[1]]}, {"k": (point[0],)}, {"k": (Fraction(3), point[1])}):
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.key_slopes(prepared, coverage, templates, points)


@needs_native
def test_the_dispatcher_runs_the_port_under_the_w1_switch(approved, monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.materialize import parametric

    call = _captured()[0]
    calls = []
    real = cftuv_native.key_slopes
    monkeypatch.setattr(cftuv_native, "key_slopes", lambda *args: calls.append(args) or real(*args))
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        assert backend.key_slopes_compute(parametric.key_slopes_of, *call) == parametric.key_slopes_of(*call)
    assert len(calls) == 1 and ledger.record().materialize_native_calls >= 1
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        backend.key_slopes_compute(parametric.key_slopes_of, *call)
    assert len(calls) == 1


def test_the_certificate_routes_its_key_slopes_through_the_dispatcher():
    """MATERIALIZE_CERTIFICATE_NATIVE_V1: the certificate takes the slopes of the points before the cut from `backend.key_slopes_compute`."""

    text = (ROOT / "kernel/src/cftuv_envelope/materialize/parametric.py").read_text(encoding="utf-8")
    assert "key_slope = backend.key_slopes_compute(key_slopes_of, prepared, coverage, request.templates, inputs.points_before)" in text
