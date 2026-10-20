"""The native `coverage_template.instantiate` (MATERIALIZE_INSTANTIATE_NATIVE_V1, `materialize_hot.rs` `instantiate`) against the oracle: the same coverage (faces, contours,
areas, total, the price on the budget) or the same `None`, or a named decline the oracle then answers for.

The values: every template the kernel's fixtures instantiate on a width step (field domains, developable strips), captured at the dispatcher, plus synthetic templates for
every kind of face, the checks before the arithmetic and a price the cap refuses.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
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
    cftuv_native is None or not hasattr(cftuv_native, "instantiate"), reason="расширение cftuv_native не собрано (или собрано без instantiate)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _budget(cap=None):
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetModeV1, ExactWorkBudgetV1

    if cap is None:
        return ExactWorkBudgetV1(mode=ExactWorkBudgetModeV1.UNLIMITED_REFERENCE, cap=None, stage="TEST")
    return ExactWorkBudgetV1(mode=ExactWorkBudgetModeV1.BOUNDED, cap=cap, stage="TEST")


def _view(coverage):
    """Everything of a coverage, coefficient types included (`int` and `Fraction` stay apart)."""

    def number(value):
        return (type(value).__name__, value)

    def sum_view(value):
        return (type(value).__name__, tuple((radicand, number(coefficient)) for radicand, coefficient in value.terms))

    if coverage is None:
        return None
    faces = tuple((face.owner, tuple(tuple(sum_view(item) for item in point) for point in face.points), sum_view(face.doubled_area)) for face in coverage.faces)
    return (coverage.outcome, number(coverage.alpha), faces, sum_view(coverage.doubled_area), coverage.polygon_doubled_area, coverage.detail)


def _same(template, partition, alpha, cap=None, priced=True):
    """`"answered"` / `"none"` when both give the same coverage (and leave the same budget), `"declined"` when the port declines."""

    from cftuv_envelope.wavefront.coverage_template import instantiate

    oracle_budget, native_budget = (_budget(cap), _budget(cap)) if priced else (None, None)
    expected = instantiate(template, partition, alpha, oracle_budget)
    try:
        found = cftuv_native.instantiate(template, partition, alpha, native_budget)
    except cftuv_native.NATIVE_REFUSALS:
        return "declined"
    assert _view(found) == _view(expected)
    if priced:
        assert native_budget.spent_by_article() == oracle_budget.spent_by_article()
        assert found is None or found.work_budget is native_budget
    if found is not None:
        # the original point objects and the area of a face behind the front are handed back, as the oracle hands them
        for face, mine, theirs in zip(partition.faces, found.faces, expected.faces):
            if theirs.points is face.points:
                assert mine.points is face.points and mine.doubled_area is theirs.doubled_area
    return "none" if found is None else "answered"


def _captured():
    """`(template, partition, alpha)` of every width step from a template on the kernel's fixtures, taken at the dispatcher."""

    import developable_factories as df
    import materialize_factories as factories
    from developable_route import developable_domain
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
    saved = backend.instantiate_compute

    def capture(oracle, template, partition, alpha, work_budget):
        calls.append((template, partition, alpha))
        return oracle(template, partition, alpha, work_budget)

    backend.instantiate_compute = capture
    try:
        for prepared, laws, alpha in cases:
            for text in (alpha, str(Decimal(alpha) * Decimal("1.001")), str(Decimal(alpha) * Decimal("0.999"))):
                step_domain(
                    prepared, text, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=laws[0], decal_topology_law=laws[1],
                    digests=False,
                )
    finally:
        backend.instantiate_compute = saved
    return calls


@needs_native
def test_every_template_of_the_kernel_fixtures_gives_the_same_coverage(approved):
    calls = _captured()
    assert len(calls) >= 10
    outcomes = [_same(*call) for call in calls]
    assert outcomes == ["answered"] * len(calls)
    assert [_same(*call, priced=False) for call in calls] == outcomes
    kinds = {pattern.kind for template, _partition, _alpha in calls for pattern in template.faces}
    assert {"BEHIND", "CUT"} <= kinds, kinds


@needs_native
def test_a_face_the_front_has_not_reached_is_empty(approved):
    """`KIND_AHEAD` (the fixtures step within windows where every face is reached): no contour, a zero area, the total of the others."""

    from cftuv_envelope.wavefront.coverage_template import KIND_AHEAD, FaceTemplateV1

    for template, partition, alpha in _captured()[:4]:
        for position in range(len(template.faces)):
            faces = list(template.faces)
            faces[position] = FaceTemplateV1(faces[position].owner, KIND_AHEAD)
            assert _same(replace(template, faces=tuple(faces)), partition, alpha) == "answered"


@needs_native
def test_a_cut_contour_of_three_kept_points_and_of_two_has_the_oracles_area(approved):
    """The shoelace from three points on, a zero below: a cut face whose contour is the first three (or two) original points."""

    from cftuv_envelope.wavefront.coverage_template import KIND_CUT, FaceTemplateV1

    for template, partition, alpha in _captured()[:4]:
        for position, face in enumerate(partition.faces):
            for items in ((0, 1, 2), (0, 1)):
                faces = list(template.faces)
                faces[position] = FaceTemplateV1(face.owner, KIND_CUT, items, None, 0)
                assert _same(replace(template, faces=tuple(faces)), partition, alpha) == "answered"


@needs_native
def test_the_checks_and_the_price_answer_none_as_the_oracle_does(approved):
    """A partition the template is not about, a negative width, and a recorded price the cap does not hold: `None`, the budget untouched."""

    template, partition, alpha = next(call for call in _captured() if call[0].price is not None and sum(call[0].price) > 0)
    owners = replace(partition, faces=(replace(partition.faces[0], owner=("someone", "else")), *partition.faces[1:]))
    assert _same(template, owners, alpha) == "none"
    assert _same(template, partition, -alpha) == "none"
    assert _same(replace(template, polygon_doubled_area=template.polygon_doubled_area + 1), partition, alpha) == "none"
    assert _same(replace(template, faces=template.faces[1:]), partition, alpha) == "none"
    assert _same(template, partition, alpha, cap=sum(template.price) - 1) == "none"
    priced = replace(template, price=(1, 2, 3, 4, 5, 6))
    assert _same(priced, partition, alpha) == "answered" and _same(priced, partition, alpha, cap=20) == "none" and _same(priced, partition, alpha, cap=21) == "answered"
    assert _same(template, partition, alpha, cap=sum(template.price)) == "answered"


@needs_native
def test_a_width_on_another_lattice_and_an_int_width_are_the_same_coverage(approved):
    template, partition, alpha = _captured()[0]
    for other in (alpha * Fraction(10007, 10000), Fraction(1, 3), Fraction(2**80 + 1, 2**81), 1, 0):
        assert _same(template, partition, other) == "answered"


@needs_native
def test_one_template_on_two_partitions_reads_each_partition(approved):
    """The template is kept converted by the identity of the template AND the partition: the same template on another partition (same owners and area, other
    points) takes that partition's points."""

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    template, partition, alpha = next(call for call in _captured() if any(pattern.kind == "CUT" for pattern in call[0].faces))
    one = SqrtSumV1.rational(1)
    moved = replace(partition, faces=tuple(replace(face, points=tuple((x + one, y) for x, y in face.points)) for face in partition.faces))
    assert _same(template, partition, alpha) == "answered"
    assert _same(template, moved, alpha) == "answered"
    assert _same(template, partition, alpha) == "answered"


@needs_native
def test_what_the_port_does_not_carry_is_declined(approved):
    from cftuv_envelope.wavefront.coverage_template import FaceTemplateV1

    template, partition, alpha = next(call for call in _captured() if any(pattern.kind == "CUT" for pattern in call[0].faces))
    position = next(index for index, pattern in enumerate(template.faces) if pattern.kind == "CUT")
    pattern = template.faces[position]
    for broken in (
        FaceTemplateV1(pattern.owner, "SIDEWAYS", pattern.items, None, pattern.cuts),
        FaceTemplateV1(pattern.owner, pattern.kind, (len(partition.faces[position].points) + 5, *pattern.items[1:]), None, pattern.cuts),
        FaceTemplateV1(pattern.owner, pattern.kind, (-1, *pattern.items[1:]), None, pattern.cuts),
    ):
        faces = (*template.faces[:position], broken, *template.faces[position + 1:])
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.instantiate(replace(template, faces=faces), partition, alpha, None)


@needs_native
def test_the_dispatcher_runs_the_port_under_the_w1_switch(approved, monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.wavefront.coverage_template import instantiate

    template, partition, alpha = _captured()[0]
    calls = []
    real = cftuv_native.instantiate
    monkeypatch.setattr(cftuv_native, "instantiate", lambda *args: calls.append(args) or real(*args))
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        assert _view(backend.instantiate_compute(instantiate, template, partition, alpha, None)) == _view(instantiate(template, partition, alpha, None))
    assert len(calls) == 1 and ledger.record().materialize_native_calls >= 1
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        backend.instantiate_compute(instantiate, template, partition, alpha, None)
    assert len(calls) == 1


def test_the_width_step_routes_its_template_coverage_through_the_dispatcher():
    """MATERIALIZE_INSTANTIATE_NATIVE_V1: the coverage source of a width step takes the coverage from a template through `backend.instantiate_compute`."""

    text = (ROOT / "kernel/src/cftuv_envelope/materialize/step.py").read_text(encoding="utf-8")
    assert "backend.instantiate_compute(instantiate, template, partition, alpha, work_budget)" in text
