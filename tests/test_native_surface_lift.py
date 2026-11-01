"""SURFACE_LIFT_NATIVE_V1: the lift of points on the source triangles (`assemble.lift_vertices` on a `BoundSurfaceLiftV1`) and the exact facts of a moving vertex the drag
certificate reads (`parametric._surface_vertex_facts_of`) in Rust (`cftuv_clip::surface`), against the oracle.

The lift: the same positions, triangle names and offset normals in the same order, the same counters in `plane._tally`, the same writes into `plane._normal_by_position`, the
same articles of the plane's budget and the same `SIGN_COUNTS`; a point in no triangle (the continuation `_nearest`) is declined with nothing applied, and the oracle answers.
The facts: the same list. The whole: a developable domain materialized and certified under the W1 stage NATIVE and PYTHON gives the same answer and the same certificate.
"""

from __future__ import annotations

import sys
from decimal import Decimal
from pathlib import Path

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
    cftuv_native is None or not hasattr(cftuv_native, "surface_vertex_facts"), reason="расширение cftuv_native не собрано (или собрано без SURFACE_LIFT_NATIVE_V1)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op
    from cftuv_native import pin

    op.refresh()
    pin.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _cases():
    """`(prepare, laws, alpha)` of developable domains (their lift is a `BoundSurfaceLiftV1`) and of the fixtures the host clips by faces."""

    import developable_factories as df
    import materialize_factories as factories
    from developable_route import developable_domain
    from materialize_factories import prepare_and_cover

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.wavefront import prepare_conveyor

    host = (NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    certified = (NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    cases = []
    for make, alpha in ((df.fold_strip, "0.3"), (df.slant_fold, "0.7"), (df.quarter_cylinder, "0.7")):
        for laws in (certified, host):
            def prepare(make=make):
                snapshot, request = developable_domain(make(), ("r0a", "r0b"), alpha="1")
                return prepare_and_cover(snapshot, request)[0]

            cases.append((prepare, laws, alpha))
    for name in ("mesh2_patch0_cut_fans_v1", "sagging_wall_convex_partition_v1"):
        def prepare(name=name):
            snapshot, request = factories.load_fixture(name)
            return prepare_conveyor(snapshot, request)

        cases.append((prepare, host, None))
    return cases


def _step(prepare, laws, alpha, materialize_backend):
    """Two widths of one fresh preparation under the W1 stage `materialize_backend`: `[(result, sign counts gained)]`."""

    from cftuv_envelope import backend, exact_sqrt_sum
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.clip_memo import memo_disabled
    from cftuv_envelope.materialize.step import step_domain

    prepared = prepare()
    text = str(prepared.requested_alpha.value) if alpha is None else alpha
    found = []
    # the clip memory is keyed by content across preparations: a run after the other backend's would replay its clips (and not pay their signs)
    with backend.use_backend("PYTHON", materialize_backend=materialize_backend), memo_disabled():
        for width in (text, str(Decimal(text) * Decimal("1.001"))):
            before = dict(exact_sqrt_sum.SIGN_COUNTS)
            stepped = step_domain(
                prepared, width, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=laws[0], decal_topology_law=laws[1],
                digests=False, parametric=True,
            )
            after = dict(exact_sqrt_sum.SIGN_COUNTS)
            found.append((stepped.result, {key: after.get(key, 0) - before.get(key, 0) for key in after}))
    return found


def _certificate_fields(certificate):
    if certificate is None:
        return None
    return (
        certificate.markers, certificate.detail, certificate.vertex_count, certificate.loop_count, tuple(certificate.position_slopes), tuple(certificate.loop_slopes),
        certificate.approximate, repr(certificate.approximate_rate), tuple(certificate.vertex_limit_widths), tuple(certificate.vertex_floor_widths), certificate.counters,
    )


@needs_native
def test_a_developable_domain_answers_and_certifies_the_same_under_both_backends(approved, monkeypatch):
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1
    from cftuv_envelope.materialize.step import answer_differences

    lifted, facts = [], []
    real_lift, real_facts = cftuv_native.lift_vertices, cftuv_native.surface_vertex_facts
    monkeypatch.setattr(cftuv_native, "lift_vertices", lambda points, plane: lifted.append(type(plane)) or real_lift(points, plane))
    monkeypatch.setattr(cftuv_native, "surface_vertex_facts", lambda plane, items, floats: facts.append((len(items), floats)) or real_facts(plane, items, floats))
    compared = 0
    for prepare, laws, alpha in _cases():
        oracle = _step(prepare, laws, alpha, "PYTHON")
        port = _step(prepare, laws, alpha, "NATIVE")
        for (expected, expected_signs), (found, found_signs) in zip(oracle, port):
            assert answer_differences(expected, found) == (), (laws, alpha)
            assert expected.counters == found.counters
            assert _certificate_fields(expected.parametric) == _certificate_fields(found.parametric)
            assert expected_signs == found_signs
            compared += 1
    assert compared >= 10
    assert BoundSurfaceLiftV1 in lifted, "no bound surface was lifted natively: the test would prove nothing"
    assert any(count for count, _floats in facts) and any(floats for _count, floats in facts), facts


def _captured_lifts():
    """`(lift, budget, points)` of every bound-surface `lift_vertices` call of the developable cases (the oracle's run), to be replayed on fresh planes."""

    from cftuv_envelope import backend
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    calls = []
    saved = backend.lift_vertices_compute

    def capture(oracle, points, plane):
        if type(plane) is BoundSurfaceLiftV1:
            calls.append((plane._lift, dict(points)))
        return saved(oracle, points, plane)

    backend.lift_vertices_compute = capture
    try:
        for prepare, laws, alpha in _cases()[:6]:
            _step(prepare, laws, alpha, "PYTHON")
    finally:
        backend.lift_vertices_compute = saved
    return calls


def _fresh(lift, cap=None):
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetModeV1, ExactWorkBudgetV1, unlimited_reference_budget
    from cftuv_envelope.materialize.lift_surface import BoundSurfaceLiftV1

    if cap is None:
        return BoundSurfaceLiftV1(lift, unlimited_reference_budget(stage="MATERIALIZE", domain_id="test"))
    return BoundSurfaceLiftV1(lift, ExactWorkBudgetV1(mode=ExactWorkBudgetModeV1.BOUNDED, cap=cap, stage="MATERIALIZE", domain_id="test"))


def _state(plane):
    return dict(plane._tally), list(plane._normal_by_position.items()), plane._max_outside, plane._budget.spent_by_article()


@needs_native
def test_every_lift_of_the_developable_cases_is_the_oracles(approved):
    from cftuv_envelope import exact_sqrt_sum
    from cftuv_envelope.materialize.assemble import lift_vertices

    calls = _captured_lifts()
    assert len(calls) >= 6
    for lift, points in calls:
        oracle_plane, port_plane = _fresh(lift), _fresh(lift)
        before = dict(exact_sqrt_sum.SIGN_COUNTS)
        expected = lift_vertices(points, oracle_plane)
        middle = dict(exact_sqrt_sum.SIGN_COUNTS)
        found = cftuv_native.lift_vertices(points, port_plane)
        after = dict(exact_sqrt_sum.SIGN_COUNTS)
        assert list(found[0].items()) == list(expected[0].items())
        assert list(found[1].items()) == list(expected[1].items())
        assert _state(port_plane) == _state(oracle_plane)
        assert {key: middle.get(key, 0) - before.get(key, 0) for key in middle} == {key: after.get(key, 0) - middle.get(key, 0) for key in after}


@needs_native
def test_a_point_in_no_triangle_is_declined_with_nothing_applied(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize.assemble import lift_vertices

    lift, points = _captured_lifts()[0]
    far = dict(points)
    far["far"] = (SqrtSumV1.rational(10**9), SqrtSumV1.rational(-(10**9)))
    plane = _fresh(lift)
    untouched = _state(plane)
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.lift_vertices(far, plane)
    assert _state(plane) == untouched
    with pytest.raises(Exception) as raised:
        lift_vertices(far, _fresh(lift))
    assert "SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION" in str(raised.value)


@needs_native
def test_an_exhausted_budget_is_declined_and_the_oracle_exhausts_the_same_way(approved):
    from cftuv_envelope.materialize.assemble import lift_vertices

    lift, points = max(_captured_lifts(), key=lambda call: len(call[1]))
    plane = _fresh(lift, cap=0)
    untouched = _state(plane)
    try:
        lift_vertices(points, _fresh(lift, cap=0))
    except Exception:  # noqa: BLE001 - the oracle's own outcome; the port must not answer for it
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.lift_vertices(points, plane)
        assert _state(plane) == untouched


@needs_native
def test_the_vertex_facts_are_the_oracles(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize import parametric

    lift, points = _captured_lifts()[0]
    plane = _fresh(lift)
    names = [triangle.name for triangle in lift.triangles]
    slopes = [(SqrtSumV1.rational(1), SqrtSumV1.zero()), (SqrtSumV1.zero(), SqrtSumV1(((2, 1),))), (SqrtSumV1.zero(), SqrtSumV1.zero())]
    items = [(names[index % len(names)], point, slopes[index % len(slopes)]) for index, point in enumerate(points.values())]
    for floats in (False, True):
        assert cftuv_native.surface_vertex_facts(plane, items, floats) == parametric._surface_vertex_facts_of(plane, items, floats)
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.surface_vertex_facts(plane, [("no such triangle", *items[0][1:])], True)


def test_the_certificate_and_the_lift_route_through_the_dispatcher():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/parametric.py").read_text(encoding="utf-8")
    assert "backend.surface_vertex_compute(_surface_vertex_facts_of, plane, items, bool(developable))" in text
    dispatch = (ROOT / "kernel/src/cftuv_envelope/backend.py").read_text(encoding="utf-8")
    assert "surface = type(plane) is BoundSurfaceLiftV1" in dispatch
