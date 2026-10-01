"""NEAR_PLANAR V2, ступень 4: приведённый целочисленный базис плоскости.

Проекция на точную плоскость делит на `n·n`, и репер от разностей спроецированных
вершин наследует знаменатели: матрица Грама получает 80-битные знаменатели, а
радиканды `SqrtSumV1` — простые делители до 73 бит (`building.004` patch 4:
Ро-Поллард на 110-битном радиканде не возвращается за кап 2^23). Закон
`REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1` берёт базис иначе — приведённый
(Лагранж—Гаусс) базис целочисленной решётки плоскости, делённый на масштаб решётки
источника, — а вершины и их позиции оставляет ровно теми же.

Ожидаемые значения здесь считаются независимым путём: свойства базиса (`w·n = 0`,
`w1 × w2 = ±n`, условие приведения) и точная реконструкция позиций, а не повторный
вызов проверяемого кода.
"""

from __future__ import annotations

import math
import random
from dataclasses import replace
from fractions import Fraction
from math import gcd

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope._plane_basis import (
    chart_of_positions,
    reduced_frame,
    reduced_plane_basis,
)
from cftuv_envelope.codec import RationalAffinePlanarMetricCodecV2, canonical_json_bytes
from cftuv_envelope.contracts.metric import (
    AffineFrameSelectionLawV1,
    GridSnappingLawV1,
    NearPlanarFramePolicyV1,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import (
    PlanarMetricAdmissionError,
    build_embedding_certified_rational_affine_planar_metric,
    build_rational_affine_planar_metric,
)
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
    validate_rational_affine_planar_metric,
)
from cftuv_envelope.wavefront import prepare_conveyor

import materialize_factories as factories
from test_near_planar_surface_law import (
    DOMAIN,
    PATCH,
    REVISION,
    _arc,
    _strip,
)

CANONICAL = NearPlanarFramePolicyV1.CANONICAL_ONLY_V1
REDUCED = NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1
NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
SOURCE_SNAP = GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
UNSNAPPED = GridSnappingLawV1.UNSNAPPED_EXACT_V1
ON_SURFACE = NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1


def _dot(left, right):
    return sum(a * b for a, b in zip(left, right, strict=True))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _primitive(vector):
    divisor = gcd(gcd(abs(vector[0]), abs(vector[1])), abs(vector[2]))
    return tuple(item // divisor for item in vector)


# Нормали: оси, диагонали, мелкие и крупные, в том числе настоящие полевые —
# `building.004` patch 4 и `walls.012` (REPORT.txt: n·n = 8.8e18).
NORMALS = (
    (0, 0, 1),
    (1, 0, 0),
    (0, 1, 0),
    (0, 1, 1),
    (3, 4, 5),
    (-6, 10, 15),
    (7, 0, -13),
    (441642243, -17041, -2449242),
    (2974878944, 1358043, 3362),
)


@pytest.mark.parametrize("raw", NORMALS)
def test_the_reduced_basis_spans_the_whole_plane_lattice(raw):
    normal = _primitive(raw)
    first, second = reduced_plane_basis(normal)
    assert _dot(first, normal) == 0 and _dot(second, normal) == 0
    # Образующие порождают ВСЮ решётку `{w : w·n = 0}`, а не подрешётку.
    assert tuple(abs(item) for item in _cross(first, second)) == tuple(
        abs(item) for item in normal
    )
    assert _cross(first, second) in (normal, tuple(-item for item in normal))
    # Приведение Лагранжа—Гаусса: `|2 w1·w2| <= |w1|² <= |w2|²`.
    assert 2 * abs(_dot(first, second)) <= _dot(first, first) <= _dot(second, second)
    for vector in (first, second):
        assert next(item for item in vector if item) > 0
    assert reduced_plane_basis(normal) == (first, second)

    # Каждый целочисленный вектор плоскости — ЦЕЛАЯ комбинация образующих.
    generator = random.Random(20261003)
    for _ in range(40):
        free = tuple(generator.randint(-50, 50) for _ in range(3))
        vector = _cross(normal, free)
        if not any(vector):
            continue
        pair = next(
            (i, j)
            for i in range(3)
            for j in range(i + 1, 3)
            if first[i] * second[j] - first[j] * second[i]
        )
        i, j = pair
        determinant = first[i] * second[j] - first[j] * second[i]
        a = Fraction(vector[i] * second[j] - vector[j] * second[i], determinant)
        b = Fraction(first[i] * vector[j] - first[j] * vector[i], determinant)
        assert a.denominator == 1 and b.denominator == 1
        assert tuple(a * x + b * y for x, y in zip(first, second)) == vector


# --------------------------------------------------------------------------
# Метрика: вершины на месте, репер другой, закон записан.
# --------------------------------------------------------------------------


def _build(rings, policy, *, grid=SOURCE_SNAP, plan=NEAR, wrapper=False):
    vertices, faces, triangles = _strip(rings)
    build = (
        build_embedding_certified_rational_affine_planar_metric
        if wrapper
        else build_rational_affine_planar_metric
    )
    record = build(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=plan,
        grid_policy=grid,
        surface_triangles=triangles,
        near_planar_lift_law=ON_SURFACE,
        near_planar_frame_policy=policy,
    )
    return record, vertices, faces, triangles


# Наклонная дуга: плоскость патча не оси-выровнена, поэтому проекция раздувает
# знаменатели (нормаль Ньюэлла — большие целые), а исходный репер — разности
# спроецированных вершин — наследует их.
ARC = _arc(24, 6, radius=4.37, width=1.31)


def _positions(metric):
    origin = tuple(
        Fraction(item.numerator, item.denominator)
        for item in (metric.exact_origin.x, metric.exact_origin.y, metric.exact_origin.z)
    )
    first = tuple(
        Fraction(item.numerator, item.denominator)
        for item in (metric.exact_basis_a.x, metric.exact_basis_a.y, metric.exact_basis_a.z)
    )
    second = tuple(
        Fraction(item.numerator, item.denominator)
        for item in (metric.exact_basis_b.x, metric.exact_basis_b.y, metric.exact_basis_b.z)
    )
    result = {}
    for item in metric.exact_source_vertex_coordinates:
        u = Fraction(item.domain_coordinate.x.numerator, item.domain_coordinate.x.denominator)
        v = Fraction(item.domain_coordinate.y.numerator, item.domain_coordinate.y.denominator)
        result[item.source_vertex_id] = tuple(
            origin[axis] + u * first[axis] + v * second[axis] for axis in range(3)
        )
    return result


def _bits(metric):
    gram = metric.exact_gram_matrix
    return sum(
        abs(item.numerator).bit_length() + item.denominator.bit_length()
        for item in (gram.m00, gram.m01, gram.m11)
    )


def test_the_reduced_frame_keeps_every_projected_position_exactly():
    canonical, *_ = _build(ARC, CANONICAL)
    reduced, *_ = _build(ARC, REDUCED)
    assert isinstance(canonical.planarity_certificate, NearPlanarProjectionCertificateV1)
    assert reduced.frame_selection_law is (
        AffineFrameSelectionLawV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1
    )
    assert canonical.frame_selection_law is (
        AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
    )
    # Вершины НЕ ДВИГАЛИСЬ: обе метрики восстанавливают побитово те же позиции,
    # а различаются только репер, Грам и координаты карты.
    assert _positions(canonical) == _positions(reduced)
    assert reduced.exact_origin == canonical.exact_origin
    assert reduced.exact_basis_a != canonical.exact_basis_a
    assert reduced.exact_gram_matrix != canonical.exact_gram_matrix
    # Репер лежит в той же плоскости: оба вектора перпендикулярны нормали.
    normal = tuple(
        Fraction(item.numerator, item.denominator)
        for item in (
            reduced.planarity_certificate.exact_plane_normal.x,
            reduced.planarity_certificate.exact_plane_normal.y,
            reduced.planarity_certificate.exact_plane_normal.z,
        )
    )
    for basis in (reduced.exact_basis_a, reduced.exact_basis_b):
        vector = tuple(
            Fraction(item.numerator, item.denominator)
            for item in (basis.x, basis.y, basis.z)
        )
        assert _dot(vector, normal) == 0
    # Грам проще: суммарные биты числителей и знаменателей ниже.
    assert _bits(reduced) < _bits(canonical)


def test_the_basis_is_in_source_grid_steps_and_deterministic():
    reduced, *_ = _build(ARC, REDUCED)
    scale = reduced.grid_certificate.source_scale
    first, second = reduced_frame(
        normal=tuple(
            int(Fraction(item.numerator, item.denominator))
            for item in (
                reduced.planarity_certificate.exact_plane_normal.x,
                reduced.planarity_certificate.exact_plane_normal.y,
                reduced.planarity_certificate.exact_plane_normal.z,
            )
        ),
        source_scale=scale,
    )
    declared = tuple(
        tuple(Fraction(item.numerator, item.denominator) for item in (v.x, v.y, v.z))
        for v in (reduced.exact_basis_a, reduced.exact_basis_b)
    )
    assert declared == (first, second)
    for vector in declared:
        assert all((axis * scale).denominator == 1 for axis in vector)
    again, *_ = _build(ARC, REDUCED)
    assert canonical_json_bytes(again) == canonical_json_bytes(reduced)


def test_an_exactly_planar_domain_is_byte_identical_under_both_policies():
    flat = tuple(((float(x), 0.0, 0.0), (float(x), 1.0, 0.0)) for x in range(3))
    first, *_ = _build(flat, CANONICAL, plan=EXACT)
    second, *_ = _build(flat, REDUCED, plan=EXACT)
    assert canonical_json_bytes(first) == canonical_json_bytes(second)
    assert first.frame_selection_law is (
        AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
    )


def test_the_default_policy_changes_nothing():
    """Без политики метрика та же, что строилась прежде: приведённый базис — по запросу."""

    vertices, faces, triangles = _strip(ARC)
    default = build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=NEAR,
        grid_policy=SOURCE_SNAP,
        surface_triangles=triangles,
        near_planar_lift_law=ON_SURFACE,
    )
    explicit, *_ = _build(ARC, CANONICAL)
    assert canonical_json_bytes(default) == canonical_json_bytes(explicit)


def test_the_reduced_basis_needs_a_snapped_source_by_name():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _build(ARC, REDUCED, grid=UNSNAPPED)
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP
    )


# --------------------------------------------------------------------------
# Валидатор и кодек.
# --------------------------------------------------------------------------


def test_the_validator_recomputes_the_reduced_basis_from_the_source():
    wrapper, vertices, faces, triangles = _build(ARC, REDUCED, wrapper=True)
    assert validate_rational_affine_planar_metric(wrapper.metric) == ()
    issues = validate_embedding_certified_rational_affine_planar_metric(
        wrapper,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
    )
    assert issues == ()

    # Подмена базиса — замечание: базис пересчитывается из нормали, а не из записи.
    metric = wrapper.metric

    def doubled_vector(vector):
        def double(item):
            value = Fraction(item.numerator, item.denominator) * 2
            return type(item)(value.numerator, value.denominator)

        return type(vector)(double(vector.x), double(vector.y), double(vector.z))

    doubled = replace(metric, exact_basis_a=doubled_vector(metric.exact_basis_a))
    assert any(
        "differs from the reduced integer plane basis" in item.message
        for item in validate_rational_affine_planar_metric(doubled)
    )


def test_the_law_is_refused_on_the_wire_where_it_does_not_apply():
    flat = tuple(((float(x), 0.0, 0.0), (float(x), 1.0, 0.0)) for x in range(3))
    exact, *_ = _build(flat, CANONICAL, plan=EXACT)
    forged = replace(
        exact,
        frame_selection_law=AffineFrameSelectionLawV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
    )
    assert any(
        "applies only to a near-planar projection" in item.message
        for item in validate_rational_affine_planar_metric(forged)
    )


def test_the_reduced_metric_survives_the_canonical_codec():
    reduced, *_ = _build(ARC, REDUCED)
    encoded = RationalAffinePlanarMetricCodecV2.dumps(reduced)
    assert RationalAffinePlanarMetricCodecV2.loads(encoded) == reduced


# --------------------------------------------------------------------------
# Цена: подготовка домена дешевле, ответ тот же.
# --------------------------------------------------------------------------

#: Наклонная плоскость `z = 0.0071·x + 0.0113·y` и подъём одной вершины на 7e-4:
#: ровно полевой случай — раздутые знаменатели проекции в репере и Грамме.
COORDINATES = dict(enumerate(factories.SKEW_FACE))
TILT = {
    index: 0.0071 * x + 0.0113 * y + (0.0007 if index == 3 else 0.0)
    for index, (x, y) in COORDINATES.items()
}


def _prepared(policy):
    canon.reset_factorization_memory()
    snapshot, request = factories.affine_domain(
        faces=(factories.SKEW_FACE,),
        routes=({"name": "source", "points": factories.SKEW_BOTTOM},),
        alpha="1",
        planarity_policy=NEAR,
        lift=TILT,
        near_planar_frame_policy=policy,
    )
    prepared = prepare_conveyor(snapshot, request)
    bits = max(
        int(key).bit_length()
        for key in (*canon._SQUAREFREE_MEMO, *canon._FACTORIZATION_MEMO)
    )
    return snapshot, prepared, bits


def test_preparation_is_cheaper_on_the_reduced_frame_and_the_answer_is_the_same():
    """Измерено: 137 211 единиц работы и радиканды до 179 бит против 5 028 и 118.

    Снижение — в десятки раз, а не в проценты, поэтому тест требует порядка, а не
    точного числа (число — свойство арифметики на этой машине и версии кода). Ответ
    подготовки тот же: исход, число граней, число узлов решётки.
    """

    _, canonical, canonical_bits = _prepared(CANONICAL)
    _, reduced, reduced_bits = _prepared(REDUCED)
    assert canonical.outcome.value == reduced.outcome.value == "EXACT"
    assert reduced.work_budget.spent * 10 < canonical.work_budget.spent
    assert reduced_bits + 20 < canonical_bits
    assert dict(canonical.counters)["CONVEYOR_FACES"] == dict(reduced.counters)[
        "CONVEYOR_FACES"
    ]
    assert math.isfinite(reduced.work_budget.spent)
    assert kernel is not None and chart_of_positions is not None
