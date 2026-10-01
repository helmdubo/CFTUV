"""NEAR_PLANAR V2, ступень 2: укладка меша на треугольники источника.

`SurfaceLiftV1` находит точку покрытия в проекции треугольника источника ТОЧНО
(знаки ориентации `SqrtSumV1` под бюджетом транзакции) и поднимает её
барицентрически по привязанным 3D-вершинам с ОДНИМ округлением. Закон
`SOURCE_TRIANGLES_V1` запрашивается явно; по умолчанию домен кладётся на
сертифицированную плоскость, и ответ материализатора не меняется.

Ожидаемые значения здесь считаются независимым путём: аффинное отображение
треугольника, заданное его тремя вершинами, `f(x, y) = P0 + x·∂x + y·∂y`, а не
повторным вызовом барицентрического кода.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.metric import (
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.exact_sqrt_sum import (
    ExactCanonicalizationWorkBudgetExhausted,
    SqrtSumV1,
    exact_work_budget,
)
from cftuv_envelope.materialize import lift_surface
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.domain import materialize_domain as _materialize
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.lift import sqrt_sum_binary64
from cftuv_envelope.materialize.lift_surface import (
    CANDIDATES,
    CHART_SNAPPED,
    DEGENERATE,
    LOCATIONS,
    ON_EDGE,
    PREDICATES,
    TRIANGLES,
    SurfaceLiftV1,
    _refuse_flipped,
)
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.outcomes import NamedOutcome

import cftuv_envelope as kernel
from materialize_factories import (
    SKEW_BOTTOM,
    SKEW_FACE,
    affine_domain,
    near_planar_domain,
    prepare_and_cover,
    straight_chain_domain,
)

ON_SURFACE = NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
ON_PLANE = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
UV = PolicyId("UV_DIRECT_STRIP_V1")


def materialize_domain(prepared, coverage, **kwargs):
    """Материализатор с законом UV продукта: запрос подготовки, закон заменён."""

    request = replace(prepared.compilation.decal_request, uv_policy_id=UV)
    return _materialize(prepared, coverage, request=request, **kwargs)


def budget(cap=None):
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="lift", cap=cap)


def exact(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


# Квадрат 4x4 решётки, разрезанный диагональю (0,0)-(4,4). Позиции — дроби
# двоичных чисел с плавающей точкой, в том числе недвоично-«круглых» (0.1, 0.3),
# поэтому «побитово» значит именно побитово, а не «до эпсилон».
V00, V10, V11, V01 = (0, 0), (4, 0), (4, 4), (0, 4)
P00 = tuple(Fraction(item) for item in (0.1, 0.2, 0.30000000000000004))
P10 = tuple(Fraction(item) for item in (4.1, 0.2, 0.7))
P11 = tuple(Fraction(item) for item in (4.1, 4.2, 1.9))
P01 = tuple(Fraction(item) for item in (0.1, 4.2, 1.1))


def two_triangle_lift():
    return SurfaceLiftV1.from_triangles(
        [
            ("t0", (V00, V10, V11), (P00, P10, P11)),
            ("t1", (V00, V11, V01), (P00, P11, P01)),
        ],
        scale=4,
    )


def affine_value(origin, ex, ey, x, y):
    """`f(x, y) = P0 + x·ex + y·ey` — аффинное отображение треугольника."""

    return tuple(origin[i] + x * ex[i] + y * ey[i] for i in range(3))


def t0(x, y):
    return affine_value(
        P00,
        tuple((P10[i] - P00[i]) / 4 for i in range(3)),
        tuple((P11[i] - P10[i]) / 4 for i in range(3)),
        Fraction(x),
        Fraction(y),
    )


def t1(x, y):
    return affine_value(
        P00,
        tuple((P11[i] - P01[i]) / 4 for i in range(3)),
        tuple((P01[i] - P00[i]) / 4 for i in range(3)),
        Fraction(x),
        Fraction(y),
    )


def as_point(values) -> LocalPoint3V1:
    return LocalPoint3V1(*(float(axis) for axis in values))


# --------------------------------------------------------------------------
# Подъём: узлы, внутренность, ребро.
# --------------------------------------------------------------------------


def test_source_nodes_lift_to_the_snapped_positions_bitwise():
    bound = two_triangle_lift().bind(budget())
    for node, position in ((V00, P00), (V10, P10), (V11, P11), (V01, P01)):
        assert bound.lift(exact(*node)) == as_point(position)


def test_an_interior_point_is_the_exact_barycentric_image():
    bound = two_triangle_lift().bind(budget())
    assert bound.lift(exact(3, 1)) == as_point(t0(3, 1))
    assert bound.lift(exact(1, 3)) == as_point(t1(1, 3))
    # И точная величина, не только округление: рациональная точка даёт
    # рациональный подъём без радикалов.
    lifted = bound.lift_exact(exact(Fraction(5, 2), Fraction(3, 2)))
    assert tuple(item.as_rational() for item in lifted) == t0(
        Fraction(5, 2), Fraction(3, 2)
    )


def test_an_edge_point_lifts_identically_from_both_triangles():
    """Шов не расходится: подъём на общем ребре зависит только от его концов."""

    lift = two_triangle_lift()
    bound = lift.bind(budget())
    midpoint = exact(2, 2)
    first, second = lift.triangles
    from_first = bound.lift_in(first, bound.values_in(first, midpoint))
    from_second = bound.lift_in(second, bound.values_in(second, midpoint))
    for left, right in zip(from_first, from_second):
        assert (left - right).is_zero
    assert tuple(sqrt_sum_binary64(axis) for axis in from_first) == tuple(
        sqrt_sum_binary64(axis) for axis in from_second
    )
    # Выбор канонический (первый по имени), и значение равно ожидаемому.
    triangle, _values = bound.locate(midpoint)
    assert triangle.name == "t0"
    assert bound.lift(midpoint) == as_point(t0(2, 2)) == as_point(t1(2, 2))
    # `locate` и `lift` — две локализации одной точки на ребре.
    assert dict(bound.counters())[ON_EDGE] == 2


def test_a_radical_edge_point_lifts_identically_from_both_triangles():
    """Точка на общем ребре с радикальными координатами: те же 53 бита с двух сторон."""

    lift = two_triangle_lift()
    bound = lift.bind(budget())
    root = SqrtSumV1.radical(1, 2, budget())
    point = (root, root)
    first, second = lift.triangles
    from_first = bound.lift_in(first, bound.values_in(first, point))
    from_second = bound.lift_in(second, bound.values_in(second, point))
    assert all((a - b).is_zero for a, b in zip(from_first, from_second))
    assert not all(item.is_rational() for item in from_first)
    assert bound.lift(point) == LocalPoint3V1(
        *(sqrt_sum_binary64(axis) for axis in from_first)
    )


def test_a_point_outside_the_projection_is_a_named_refusal_with_numbers():
    bound = two_triangle_lift().bind(budget())
    with pytest.raises(MaterializationRefusal) as refusal:
        bound.lift(exact(5, 1))
    assert (
        refusal.value.outcome
        is MaterializationOutcome.SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION
    )
    assert "lies outside the projection of all 2 source triangles" in refusal.value.detail
    assert dict(refusal.value.counters)[LOCATIONS] == 1
    assert dict(refusal.value.counters)[TRIANGLES] == 2
    # Точка на волосок вне границы — тоже вне: допуска у замкнутого треугольника
    # нет, и «почти внутри» отказ, а не подъём.
    just_outside = (
        SqrtSumV1.rational(Fraction(4) + Fraction(1, 10**12)),
        SqrtSumV1.rational(2),
    )
    with pytest.raises(MaterializationRefusal):
        two_triangle_lift().bind(budget()).lift(just_outside)


# --------------------------------------------------------------------------
# Счётчики и бюджет.
# --------------------------------------------------------------------------


def test_counters_name_locations_candidates_and_predicates():
    bound = two_triangle_lift().bind(budget())
    bound.lift(exact(3, 1))
    counters = dict(bound.counters())
    assert (counters[LOCATIONS], counters[CANDIDATES], counters[PREDICATES]) == (
        1,
        1,
        3,
    )
    bound.lift(exact(1, 3))
    counters = dict(bound.counters())
    # Вторая точка: первый кандидат отвергнут на третьем ребре, второй принят.
    assert (counters[LOCATIONS], counters[CANDIDATES], counters[PREDICATES]) == (
        2,
        3,
        9,
    )
    assert counters[TRIANGLES] == 2
    assert counters[DEGENERATE] == 0
    assert counters[CHART_SNAPPED] == 0


def test_every_sign_is_paid_from_the_transaction_budget(monkeypatch):
    """Ни один знак не уходит без бюджета: каждый вызов получает ЭТОТ объект."""

    seen = []
    original = SqrtSumV1.sign

    def spy(self, *, filter_bits=64, budget=None):
        seen.append(budget)
        return original(self, filter_bits=filter_bits, budget=budget)

    monkeypatch.setattr(SqrtSumV1, "sign", spy)
    paid = budget()
    bound = two_triangle_lift().bind(paid)
    bound.lift(exact(1, 3))
    assert len(seen) == dict(bound.counters())[PREDICATES] > 0
    assert all(item is paid for item in seen)


def test_an_exhausted_budget_is_the_named_refusal_of_the_materializer(monkeypatch):
    """Бюджет кончился на знаке — домен отказывает `EXACT_WORK_BUDGET_EXHAUSTED`."""

    class Exhausting:
        is_zero = False

        def sign(self, *, filter_bits=64, budget=None):
            raise ExactCanonicalizationWorkBudgetExhausted(
                "EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED: stage=MATERIALIZE"
            )

    monkeypatch.setattr(lift_surface, "_edge_value", lambda *args: Exhausting())
    prepared, coverage, request = near_planar_domain()
    result = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    assert result.outcome is MaterializationOutcome.EXACT_WORK_BUDGET_EXHAUSTED
    assert not result.is_materialized


def test_a_degenerate_projection_is_skipped_and_counted_not_silently_lost():
    lift = SurfaceLiftV1.from_triangles(
        [
            ("flat", (V00, V10, (8, 0)), (P00, P10, P11)),
            ("good", (V00, V10, V11), (P00, P10, P11)),
        ],
        scale=4,
    )
    assert [item.name for item in lift.triangles] == ["good"]
    assert lift.degenerate_projections == 1
    assert dict(lift.bind(budget()).counters())[DEGENERATE] == 1


def test_a_snap_that_flips_a_projection_is_a_named_refusal():
    owned = [
        type(
            "T",
            (),
            {
                "triangle_id": type("I", (), {"value": "t-flipped"})(),
                "vertex_ids": ("a", "b", "c"),
            },
        )()
    ]
    exact_chart = {
        "a": (Fraction(0), Fraction(0)),
        "b": (Fraction(1), Fraction(0)),
        "c": (Fraction(0), Fraction(1, 2)),
    }
    snapped = dict(exact_chart, c=(Fraction(0), Fraction(-1)))
    with pytest.raises(MaterializationRefusal) as refusal:
        _refuse_flipped(owned, exact_chart, snapped)
    assert (
        refusal.value.outcome
        is MaterializationOutcome.SURFACE_LIFT_CHART_SNAP_FLIPPED_TRIANGLE
    )
    assert "t-flipped" in refusal.value.detail
    # Сжатие в ноль — не переворот: проекция без площади никого не накрывает.
    collapsed = dict(exact_chart, c=(Fraction(0), Fraction(0)))
    _refuse_flipped(owned, exact_chart, collapsed)


# --------------------------------------------------------------------------
# Материализатор: закон по запросу, по умолчанию плоскость.
# --------------------------------------------------------------------------


def _vertices(batch):
    return {item.vert_key.value: item.position for item in batch.vertices}


def test_the_default_law_is_the_certified_plane_and_nothing_changes():
    prepared, coverage, request = near_planar_domain()
    default = materialize_domain(prepared, coverage)
    explicit = materialize_domain(prepared, coverage, near_planar_lift_law=ON_PLANE)
    assert default.is_materialized and explicit.is_materialized
    assert default.content_digest == explicit.content_digest
    assert default.counters == explicit.counters
    assert not any("SURFACE_LIFT" in name for name, _ in default.counters)
    assert any(
        line.startswith(NamedOutcome.NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE.value)
        for line in default.diagnostics
    )


def _on_some_source_triangle(point, corners) -> float:
    """Расстояние точки до плоскости треугольника, если она в его границах."""

    a, b, c = corners
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    normal = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(item * item for item in normal))
    offset = tuple(point[i] - a[i] for i in range(3))
    distance = abs(sum(normal[i] * offset[i] for i in range(3))) / length
    # барицентрические координаты в плоскости треугольника
    dot = lambda u, v: sum(u[i] * v[i] for i in range(3))  # noqa: E731
    d00, d01, d11 = dot(ab, ab), dot(ab, ac), dot(ac, ac)
    d20, d21 = dot(offset, ab), dot(offset, ac)
    denominator = d00 * d11 - d01 * d01
    v = (d11 * d20 - d01 * d21) / denominator
    w = (d00 * d21 - d01 * d20) / denominator
    inside = min(v, w, 1 - v - w) >= -1e-9
    return distance if inside else math.inf


def test_a_near_planar_domain_lies_on_the_source_surface():
    prepared, coverage, request = near_planar_domain()
    on_plane = materialize_domain(prepared, coverage, near_planar_lift_law=ON_PLANE)
    on_surface = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    assert on_plane.is_materialized
    assert on_surface.is_materialized, on_surface.detail
    assert on_plane.content_digest != on_surface.content_digest

    sigma = prepared.context.frame.planarity_certificate.width_distortion
    positions = {
        item.source_vertex_id: tuple(
            Fraction(axis.numerator, axis.denominator)
            for axis in (item.position.x, item.position.y, item.position.z)
        )
        for item in sigma.snapped_source_positions
    }
    owner = prepared.compilation.owner_patch_id
    faces = {
        face.face_id
        for face in prepared.context.snapshot.surface_ir.source_faces
        if face.patch_id == owner
    }
    triangles = [
        tuple(tuple(float(axis) for axis in positions[v]) for v in item.vertex_ids)
        for item in prepared.context.snapshot.surface_ir.surface_triangles
        if item.source_face_id in faces
    ]
    plane = _vertices(on_plane.batch)
    surface = _vertices(on_surface.batch)
    assert plane.keys() == surface.keys()
    worst_distance = 0.0
    moved = 0.0
    for key, point in surface.items():
        coordinates = (point.x, point.y, point.z)
        worst_distance = max(
            worst_distance,
            min(_on_some_source_triangle(coordinates, item) for item in triangles),
        )
        flat = plane[key]
        moved = max(
            moved,
            math.dist(coordinates, (flat.x, flat.y, flat.z)),
        )
    # Расстояние до поверхности источника — округление binary64, не допуск.
    assert worst_distance < 1e-12, worst_distance
    # Меш сместился на поверхность: у приподнятой вершины невязка 0.002, и не больше.
    assert 0.0 < moved <= 0.002 + 1e-9, moved

    counters = dict(on_surface.counters)
    assert counters[LOCATIONS] == len(surface)
    assert counters[TRIANGLES] == len(triangles)
    lines = [
        line
        for line in on_surface.diagnostics
        if line.startswith(NamedOutcome.NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES.value)
    ]
    assert len(lines) == 1
    for fragment in ("min_cos_squared=", "width_budget=0.02", "recorded, not judging"):
        assert fragment in lines[0], lines[0]


def test_an_exactly_planar_domain_ignores_the_law():
    prepared, coverage, request = straight_chain_domain()
    default = materialize_domain(prepared, coverage)
    asked = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    assert default.is_materialized and asked.is_materialized
    assert default.content_digest == asked.content_digest
    assert default.counters == asked.counters


def test_a_near_planar_domain_without_the_distortion_record_cannot_lie_on_the_surface():
    snapshot, request = affine_domain(
        faces=(SKEW_FACE,),
        routes=({"name": "source", "points": SKEW_BOTTOM},),
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        lift={3: 0.002},
        with_triangles=False,
    )
    prepared, coverage = prepare_and_cover(snapshot, request)
    certificate = prepared.context.frame.planarity_certificate
    assert type(certificate) is NearPlanarProjectionCertificateV1
    assert certificate.width_distortion is None
    result = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    assert result.outcome is MaterializationOutcome.SURFACE_LIFT_UNAVAILABLE
    # А на плоскости тот же домен материализуется, как раньше.
    assert materialize_domain(prepared, coverage).is_materialized


def test_the_lift_construction_is_deterministic():
    prepared, coverage, request = near_planar_domain()
    first = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    second = materialize_domain(prepared, coverage, near_planar_lift_law=ON_SURFACE)
    assert first.content_digest == second.content_digest
    assert first.counters == second.counters
    assert replace is not None and kernel is not None
