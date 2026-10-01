"""NEAR_PLANAR V2, ступень 1: сертификат искажения ширины записывается точно.

Проекция треугольника на плоскость карты имеет сингулярные числа `1` и
`cos θ`, а `cos² θ = (n_T·n)² / ((n_T·n_T)(n·n))` — рациональное число, поэтому
сертификат не вводит ни корня, ни допуска вычисления. Допуск один: относительная
ширина `NEAR_PLANAR_WIDTH_BUDGET = 1/50`, условие приёма `cos² ≥ 1/(1+b)²`.

Здесь сертификат ЗАПИСЫВАЕТСЯ и пересчитывается валидатором, но строитель по
нему ещё не судит: домены принимаются и отказывают ровно как раньше.
Ожидаемые числа считаются в тесте независимым путём (тождество Лагранжа
`cos² = 1 − |n_T × n|² / (|n_T|²|n|²)` и формулы по руками выведенной нормали),
а не повторным вызовом проверяемого кода.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope._width_distortion import (
    triangle_cos_squared,
    width_distortion_refusal_text,
    width_distortion_threshold,
    width_distortion_violation,
    width_distortion_violations,
)
from cftuv_envelope.codec import (
    RationalAffinePlanarMetricCodecV2,
    canonical_json_bytes,
)
from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import (
    NEAR_PLANAR_WIDTH_BUDGET,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    GridSnappingLawV1,
    NearPlanarProjectionCertificateV1,
    NearPlanarWidthDistortionCertificateV1,
    NearPlanarWidthDistortionLawV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.contracts.surface import SourceFaceV1, SurfaceTriangleV1
from cftuv_envelope.ids import (
    PatchDomainId,
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
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

NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
UNSNAPPED = GridSnappingLawV1.UNSNAPPED_EXACT_V1
REVISION = SourceRevision("width-revision")
DOMAIN = PatchDomainId("width-domain")
PATCH = PatchId("width-patch")


# --------------------------------------------------------------------------
# Вход: полигональные грани, веерная триангуляция, независимая формула.
# --------------------------------------------------------------------------


def _vertices(points):
    ids = tuple(SourceVertexId(f"w{index:02d}") for index in range(len(points)))
    records = tuple(
        SourceVertexV1(vertex_id, LocalPoint3V1(*(float(axis) for axis in point)))
        for vertex_id, point in zip(ids, points, strict=True)
    )
    return ids, records


def _face(name, cycle):
    return SourceFaceV1(
        face_id=SourceFaceId(name),
        patch_id=PATCH,
        vertex_cycle=tuple(cycle),
        edge_cycle=tuple(
            PhysicalEdgeId(
                "e:"
                + ":".join(
                    sorted(
                        (cycle[index].value, cycle[(index + 1) % len(cycle)].value)
                    )
                )
            )
            for index in range(len(cycle))
        ),
        polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )


def _fan(face):
    """Веер из первой вершины: те же треугольники, что получил бы хост."""

    first = face.vertex_cycle[0]
    return tuple(
        SurfaceTriangleV1(
            triangle_id=SurfaceTriangleId(f"{face.face_id.value}:t{index}"),
            source_face_id=face.face_id,
            vertex_ids=(first, face.vertex_cycle[index], face.vertex_cycle[index + 1]),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for index in range(1, len(face.vertex_cycle) - 1)
    )


def _build(points, cycles, *, triangles="fan", grid=UNSNAPPED, policy=NEAR):
    ids, records = _vertices(points)
    faces = tuple(
        _face(f"face{index}", [ids[item] for item in cycle])
        for index, cycle in enumerate(cycles)
    )
    if triangles == "fan":
        triangles = tuple(item for face in faces for item in _fan(face))
    metric = build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=records,
        source_faces=faces,
        planarity_policy=policy,
        grid_policy=grid,
        surface_triangles=triangles,
    )
    return metric, records, faces, triangles


def _lagrange_cos_squared(corners, normal):
    """`cos² = 1 − sin²`: независимый от кода ядра путь к тому же числу."""

    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = (
        tuple(Fraction(axis) for axis in item) for item in corners
    )
    first = (bx - ax, by - ay, bz - az)
    second = (cx - ax, cy - ay, cz - az)
    tri = (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )
    n = tuple(Fraction(axis) for axis in normal)
    cross = (
        tri[1] * n[2] - tri[2] * n[1],
        tri[2] * n[0] - tri[0] * n[2],
        tri[0] * n[1] - tri[1] * n[0],
    )
    tri_sq = sum(axis * axis for axis in tri)
    n_sq = sum(axis * axis for axis in n)
    return 1 - sum(axis * axis for axis in cross) / (tri_sq * n_sq)


def _value(rational: ExactRationalV1) -> Fraction:
    return Fraction(rational.numerator, rational.denominator)


def _sigma(metric):
    return metric.planarity_certificate.width_distortion


# --------------------------------------------------------------------------
# Формула: кривой квад с прогибом h.
# --------------------------------------------------------------------------

SAG = Fraction(1, 128)
WARPED = (
    (0, 0, 0),
    (1, 0, 0),
    (1, 1, 0),
    (0, 1, SAG),
)


def test_a_warped_quad_records_cos_squared_by_the_closed_formula():
    """Квад с поднятым углом: нормаль Ньюэлла `(h, -h, 2)`, `cos²` — дробями.

    Вершины `P0 P1 P2 P3`, поднят `P3` на `h`. Веер даёт два треугольника:
    `(P0 P1 P2)` с нормалью `(0, 0, 1)` и `(P0 P2 P3)` с нормалью `(h, -h, 1)`.
    Нормаль патча — вектор площади, `(h, -h, 2)`:

        cos²(T1) = 4 / (2h² + 4) = 2 / (h² + 2)
        cos²(T2) = (2h² + 2)² / ((2h² + 1)(2h² + 4))

    Число сертификата обязано равняться меньшему из них БИТ В БИТ, а худшим
    назван тот треугольник, который его даёт.
    """

    metric, _records, _faces, _triangles = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)
    assert type(metric.planarity_certificate) is NearPlanarProjectionCertificateV1
    h = SAG
    first = 2 / (h * h + 2)
    second = (2 * h * h + 2) ** 2 / ((2 * h * h + 1) * (2 * h * h + 4))
    assert first != second
    assert _value(sigma.min_cos_squared) == min(first, second)
    expected_worst = "face0:t1" if first < second else "face0:t2"
    assert sigma.worst_triangle_id.value == expected_worst
    assert sigma.worst_face_id.value == "face0"
    assert sigma.triangles_measured == 2
    assert sigma.degenerate_triangle_count == 0
    assert sigma.first_degenerate_triangle_id is None
    assert sigma.law is NearPlanarWidthDistortionLawV1.INTRINSIC_WIDTH_RELATIVE_V1
    assert _value(sigma.width_budget) == NEAR_PLANAR_WIDTH_BUDGET == Fraction(1, 50)


def test_the_helper_formula_agrees_with_the_independent_lagrange_identity():
    """Две формулы одного числа: проверяемая и независимая сходятся на дробях."""

    normal = (Fraction(1, 128), Fraction(-1, 128), Fraction(2))
    corners = ((0, 0, 0), (1, 1, 0), (0, 1, Fraction(1, 128)))
    exact = tuple(tuple(Fraction(axis) for axis in item) for item in corners)
    assert triangle_cos_squared(exact, normal) == _lagrange_cos_squared(
        corners, normal
    )


def test_the_certificate_measures_the_snapped_positions_not_the_projected_ones():
    """Меряется отображение проекции, поэтому вход — позиции ДО неё.

    По проекции квад плоский (`cos² = 1` у всех треугольников): измерение по
    ним было бы пустым. Записанные позиции — те, что до проекции, и вершина
    `P3` в них поднята.
    """

    metric, _records, _faces, _triangles = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)
    recorded = {
        item.source_vertex_id.value: tuple(_value(axis) for axis in (item.position.x, item.position.y, item.position.z))
        for item in sigma.snapped_source_positions
    }
    assert recorded["w03"] == (0, 1, SAG)
    assert recorded["w00"] == (0, 0, 0)
    assert _value(sigma.min_cos_squared) < 1


# --------------------------------------------------------------------------
# Допуск: граница, имя, числа отказа.
# --------------------------------------------------------------------------


def test_a_gentle_slope_is_within_the_width_budget():
    """Положительная фикстура допуска: пологий скат проходит с запасом."""

    metric, *_ = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)
    assert _value(sigma.min_cos_squared) > width_distortion_threshold(
        NEAR_PLANAR_WIDTH_BUDGET
    )
    assert width_distortion_violations(sigma) == ()
    assert width_distortion_violation(sigma) is None


# Гребень: два прямоугольника, второй круто поднят. Габарит сантиметровый,
# поэтому невязка плоскости (порядка миллиметров) укладывается в 1.25 см, и
# метрику строитель ПРИНИМАЕТ, а искажение ширины на крутом скате велико.
RIDGE = (
    (0, 0, 0),
    (Fraction(1, 100), 0, 0),
    (Fraction(1, 100), Fraction(1, 100), 0),
    (0, Fraction(1, 100), 0),
    (Fraction(1, 100), Fraction(2, 100), Fraction(1, 125)),
    (0, Fraction(2, 100), Fraction(1, 125)),
)
RIDGE_FACES = [(0, 1, 2, 3), (3, 2, 4, 5)]


def test_a_steep_slope_is_beyond_the_width_budget_by_name():
    """Отрицательная фикстура допуска: крутой скат называется по имени и числам.

    Невязка гребня сантиметровая, то есть домен ПРИНЯТ строителем (он по
    сертификату пока не судит), но записанный `cos²` худшего треугольника ниже
    порога `2500/2601`, и судья отвечает именем с числами.
    """

    metric, _records, _faces, _triangles = _build(RIDGE, RIDGE_FACES)
    sigma = _sigma(metric)
    threshold = width_distortion_threshold(NEAR_PLANAR_WIDTH_BUDGET)
    assert threshold == Fraction(2500, 2601)
    assert _value(sigma.min_cos_squared) < threshold
    assert width_distortion_violations(sigma) == (
        NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED,
    )
    text = width_distortion_refusal_text(sigma)
    for fragment in (
        "min_cos_squared=",
        "threshold=",
        "width_budget=2.000000e-02",
        "law=INTRINSIC_WIDTH_RELATIVE_V1",
        "worst_triangle=face",
    ):
        assert fragment in text, text


def test_the_budget_boundary_admits_and_refuses_by_name():
    """Ровно на пороге — принимается; на единицу ниже по `1/10⁹` — отказ."""

    metric, *_ = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)
    threshold = width_distortion_threshold(NEAR_PLANAR_WIDTH_BUDGET)

    def at(value):
        item = Fraction(value)
        return replace(
            sigma, min_cos_squared=ExactRationalV1(item.numerator, item.denominator)
        )

    assert width_distortion_violations(at(threshold)) == ()
    assert width_distortion_violations(at(threshold + Fraction(1, 10**9))) == ()
    assert width_distortion_violations(at(threshold - Fraction(1, 10**9))) == (
        NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED,
    )


def test_a_degenerate_snapped_triangle_is_a_named_closed_outcome():
    """Нулевая нормаль треугольника — не пропуск, а отказ с именем.

    Пятиугольник с тремя коллинеарными подряд вершинами: веер из `P0` даёт
    треугольник `(P0 P1 P2)` на одной прямой. Измерить наклон нельзя, и
    сертификат НЕ молчит: счёт, первое имя и именованный исход судьи.
    """

    points = (
        (0, 0, 0),
        (1, 0, 0),
        (2, 0, 0),
        (2, 1, 0),
        (0, 1, Fraction(1, 128)),
    )
    metric, *_ = _build(points, [(0, 1, 2, 3, 4)])
    sigma = _sigma(metric)
    assert sigma.degenerate_triangle_count == 1
    assert sigma.first_degenerate_triangle_id.value == "face0:t1"
    assert sigma.triangles_measured == 2
    assert width_distortion_violation(sigma) is (
        NamedOutcome.NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE
    )


def test_no_triangles_to_measure_is_a_named_input_defect_not_a_pass():
    """Нет треугольников у владельца — `UNAVAILABLE`, а не «искажения нет»."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _build(WARPED, [(0, 1, 2, 3)], triangles=())
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE
    )


def test_a_triangle_naming_a_vertex_outside_the_patch_is_a_named_input_defect():
    ids, _records = _vertices(WARPED)
    stranger = SurfaceTriangleV1(
        triangle_id=SurfaceTriangleId("stray"),
        source_face_id=SourceFaceId("face0"),
        vertex_ids=(ids[0], ids[1], SourceVertexId("outside")),
        physical_edge_ids=(None, None, None),
        triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
    )
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _build(WARPED, [(0, 1, 2, 3)], triangles=(stranger,))
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE
    )
    assert "outside" in str(failure.value)


# --------------------------------------------------------------------------
# Планарные домены: байты не двигаются.
# --------------------------------------------------------------------------

PLANAR = ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0))


def test_an_exact_plane_carries_no_certificate_and_its_bytes_do_not_move():
    """Точная плоскость искажения не имеет: записи нет, байты прежние."""

    with_triangles, records, faces, _ = _build(PLANAR, [(0, 1, 2, 3)], policy=EXACT)
    without = build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=records,
        source_faces=faces,
        planarity_policy=EXACT,
        grid_policy=UNSNAPPED,
    )
    assert type(with_triangles.planarity_certificate) is ExactSourcePlaneCertificateV1
    assert canonical_json_bytes(with_triangles) == canonical_json_bytes(without)


def test_a_caller_without_triangles_gets_no_record_rather_than_a_good_one():
    """Не мерили — не пишем: `None` читается как «не измерялось»."""

    _metric, records, faces, _triangles = _build(WARPED, [(0, 1, 2, 3)])
    unmeasured = build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=records,
        source_faces=faces,
        planarity_policy=NEAR,
        grid_policy=UNSNAPPED,
    )
    assert unmeasured.planarity_certificate.width_distortion is None


def test_recording_the_certificate_does_not_change_what_the_builder_admits():
    """Сертификат пишется, но не судит: домен, отвергаемый невязкой, отвергается."""

    steep = ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, Fraction(1, 2)))
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _build(steep, [(0, 1, 2, 3)])
    assert (
        failure.value.outcome is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    )


# --------------------------------------------------------------------------
# Валидатор пересчитывает.
# --------------------------------------------------------------------------


def _validated(metric, records, faces, triangles):
    return validate_embedding_certified_rational_affine_planar_metric(
        build_embedding_certified_rational_affine_planar_metric(
            source_revision=REVISION,
            patch_domain_id=DOMAIN,
            owner_patch_id=PATCH,
            source_vertices=records,
            source_faces=faces,
            planarity_policy=NEAR,
            grid_policy=UNSNAPPED,
            surface_triangles=triangles,
        )
        if metric is None
        else metric,
        source_vertices=records,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
    )


def test_the_validator_recomputes_the_certificate_from_the_source():
    """Честная запись проходит; подмена каждого числа — замечание валидатора."""

    metric, records, faces, triangles = _build(RIDGE, RIDGE_FACES)
    assert validate_rational_affine_planar_metric(metric) == ()
    assert _validated(None, records, faces, triangles) == ()
    wrapper = build_embedding_certified_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=records,
        source_faces=faces,
        planarity_policy=NEAR,
        grid_policy=UNSNAPPED,
        surface_triangles=triangles,
    )
    sigma = wrapper.metric.planarity_certificate.width_distortion
    forgeries = (
        replace(sigma, min_cos_squared=ExactRationalV1(1, 1)),
        replace(sigma, triangles_measured=sigma.triangles_measured + 1),
        replace(sigma, worst_triangle_id=SurfaceTriangleId("face1:t2")
                if sigma.worst_triangle_id.value != "face1:t2"
                else SurfaceTriangleId("face0:t1")),
    )
    for forged in forgeries:
        broken = replace(
            wrapper,
            metric=replace(
                wrapper.metric,
                planarity_certificate=replace(
                    wrapper.metric.planarity_certificate, width_distortion=forged
                ),
            ),
        )
        issues = _validated(broken, records, faces, triangles)
        assert any(
            "width-distortion certificate differs from exact recomputation"
            in item.message
            for item in issues
        ), issues


def test_the_validator_refuses_a_forged_budget_or_foreign_domain():
    metric, *_ = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)

    def broken(**changes):
        return replace(
            metric,
            planarity_certificate=replace(
                metric.planarity_certificate,
                width_distortion=replace(sigma, **changes),
            ),
        )

    assert validate_rational_affine_planar_metric(
        broken(width_budget=ExactRationalV1(1, 10))
    )
    assert validate_rational_affine_planar_metric(
        broken(patch_domain_id=PatchDomainId("another-domain"))
    )
    assert validate_rational_affine_planar_metric(
        broken(snapped_source_positions=frozenset())
    )


def test_the_certificate_survives_the_canonical_codec():
    metric, *_ = _build(RIDGE, RIDGE_FACES)
    encoded = RationalAffinePlanarMetricCodecV2.dumps(metric)
    assert RationalAffinePlanarMetricCodecV2.loads(encoded) == metric


def test_the_contract_rejects_an_incoherent_record():
    metric, *_ = _build(WARPED, [(0, 1, 2, 3)])
    sigma = _sigma(metric)
    with pytest.raises(ValueError):
        replace(sigma, min_cos_squared=ExactRationalV1(3, 2))
    with pytest.raises(ValueError):
        replace(sigma, worst_triangle_id=None)
    with pytest.raises(ValueError):
        replace(sigma, degenerate_triangle_count=1)
    with pytest.raises(ValueError):
        replace(sigma, triangles_measured=0, worst_triangle_id=None, worst_face_id=None)
    assert isinstance(sigma, NearPlanarWidthDistortionCertificateV1)
