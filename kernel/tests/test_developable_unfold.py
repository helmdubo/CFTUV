"""DEVELOPABLE (S1), срез C1: развёртка по шарнирам, судимая точным растяжением.

Карта домена — привязанная к решётке шарнирная развёртка треугольников источника.
Власть — точный сертификат растяжения (`DevelopableStretchCertificateV1`): все квадраты
сингулярных чисел отображения треугольник источника -> треугольник карты в
`[1/(1+b)², (1+b)²]`, `b = 1/50`, тремя знаками рациональных чисел. Фикстуры этого
файла идут НАПРЯМУЮ через `build_developable_chart` (без лестницы метрики — она в C2):

* складка 90° — растяжение тождественно единице, побитово (золотой дайджест);
* фаска 3-5 сегментов, четверть цилиндра в 16 сегментов (`V` — длина дуги);
* конус: вершина на границе — обычный сектор, вершина внутри — отказ с именем вершины;
* полусфера и седло — отказ по растяжению; спираль — перекрытие карты;
* кольцо — `PERIODIC_CUT_REQUIRED`; топологические отказы названы;
* решётка карты: слишком грубая — имя, а не допуск; вторая ступень — когда первая груба.
"""

from __future__ import annotations

import hashlib
import math
from fractions import Fraction

import pytest

from cftuv_envelope._developable import UNFOLD_CHART_SCALE_FACTORS, build_developable_chart
from cftuv_envelope._fan_closure import angle_defect_lower_bound, worst_defect_vertex
from cftuv_envelope._stretch import (
    band_bounds,
    band_squared_upper,
    chart_gram,
    in_stretch_band,
    source_gram,
    stretch_violations,
)
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.metric import (
    DEVELOPABLE_STRETCH_BUDGET,
    DevelopableFanClosureLawV1,
    DevelopableUnfoldCertificateV1,
    ExactRationalV1,
    GridSnappingLawV1,
    VertexDevelopabilityClassV1,
)
from cftuv_envelope.contracts.surface import SurfaceTriangleV1
from cftuv_envelope.exact_sqrt_sum import (
    SqrtSumV1,
    exact_work_budget,
    isolated_factorization_memory,
    reset_factorization_memory,
)
from cftuv_envelope.ids import SourceFaceId, SourceVertexId, SurfaceTriangleId
from cftuv_envelope.numeric import LocalVector3V1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError

import developable_factories as factories
from developable_factories import REVISION, DOMAIN, developable_chart

#: Золотой дайджест сертификата складки 90°: карта на целых узлах, растяжение 1.
FOLD_STRIP_CERTIFICATE_SHA256 = (
    "1f9061784ba42197c71f92fed3da219ff91afad47cd10ca1ea9a9382b7d91977"
)


def _digest(record) -> str:
    return hashlib.sha256(canonical_json_bytes(record)).hexdigest()


def _band(certificate) -> Fraction:
    band = certificate.stretch.worst_band_squared_upper
    return Fraction(band.numerator, band.denominator)


def _refusal(parts, **overrides) -> PlanarMetricAdmissionError:
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(parts, **overrides)
    return failure.value


def _single_triangle(*corners):
    ids = [SourceVertexId(f"v:{index}") for index in range(3)]
    triangle = SurfaceTriangleV1(
        SurfaceTriangleId("t0"),
        SourceFaceId("f0"),
        tuple(ids),
        (None, None, None),
        LocalVector3V1(0.0, 0.0, 1.0),
    )
    positions = {
        vertex: tuple(Fraction(item) for item in corner)
        for vertex, corner in zip(ids, corners)
    }
    return ids, triangle, positions


def _chart_of_single(*corners, source_scale=1):
    ids, triangle, positions = _single_triangle(*corners)
    return build_developable_chart(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        snapped=positions,
        owner_triangles=(triangle,),
        required_ids=tuple(ids),
        source_scale=source_scale,
    )


# --------------------------------------------------------------------------
# Складка, фаска, цилиндр
# --------------------------------------------------------------------------


def test_a_fold_of_ninety_degrees_unfolds_with_stretch_exactly_one():
    chart = developable_chart(factories.fold_strip())
    certificate = chart.certificate
    assert type(certificate) is DevelopableUnfoldCertificateV1
    assert certificate.stretch.worst_band_squared_upper == ExactRationalV1(1, 1)
    assert certificate.stretch.triangles_measured == 6
    assert certificate.stretch.triangles_outside_budget == 0
    assert certificate.stretch.chart_flipped_triangle_count == 0
    assert certificate.chart_boundary_overlap_count == 0
    # Узлы лежат на целых: полоса из трёх единичных квадов — три шага по `scale`.
    scale = chart.chart_scale
    rungs = sorted({node[1] for node in chart.nodes.values()})
    assert rungs == [0, scale, 2 * scale, 3 * scale]
    assert {node[0] for node in chart.nodes.values()} == {0, scale}
    assert _digest(certificate) == FOLD_STRIP_CERTIFICATE_SHA256


def test_the_fold_chart_gram_equals_the_source_gram_bitwise():
    """σ ≡ 1 — не «около единицы»: Грам карты равен Граму источника как дроби."""

    parts = factories.fold_strip()
    chart = developable_chart(parts)
    vertices, _faces, triangles = parts
    position = {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    scale = chart.chart_scale
    for triangle in triangles:
        source = source_gram(tuple(position[v] for v in triangle.vertex_ids))
        points = tuple(
            (Fraction(chart.nodes[v][0], scale), Fraction(chart.nodes[v][1], scale))
            for v in triangle.vertex_ids
        )
        assert chart_gram(points) == source, triangle.triangle_id


@pytest.mark.parametrize("segments", (3, 4, 5))
def test_a_gentle_bevel_is_within_the_stretch_budget(segments):
    chart = developable_chart(factories.bevel_strip(segments))
    certificate = chart.certificate
    _low, high = band_bounds(DEVELOPABLE_STRETCH_BUDGET)
    assert _band(certificate) <= high
    assert not stretch_violations(certificate.stretch)
    # Развёртка сохраняет длину ленты: segments + 1 квад по единице вдоль неё.
    scale = chart.chart_scale
    ys = [node[1] for node in chart.nodes.values()]
    assert abs((max(ys) - min(ys)) / scale - (segments + 1)) < 1e-3
    assert certificate.previous_refusals == ()


def test_a_quarter_cylinder_of_sixteen_segments_unfolds_to_its_arc_length():
    chart = developable_chart(factories.quarter_cylinder(16))
    scale = chart.chart_scale
    ys = [node[1] for node in chart.nodes.values()]
    xs = [node[0] for node in chart.nodes.values()]
    polyline = sum(
        math.dist(
            (math.sin(math.pi / 2 * k / 16), 1 - math.cos(math.pi / 2 * k / 16)),
            (math.sin(math.pi / 2 * (k + 1) / 16), 1 - math.cos(math.pi / 2 * (k + 1) / 16)),
        )
        for k in range(16)
    )
    assert abs((max(ys) - min(ys)) / scale - polyline) < 1e-4
    assert abs(polyline - math.pi / 2) < 1e-3  # 16 хорд против дуги
    assert (max(xs) - min(xs)) / scale == pytest.approx(1.0, abs=1e-9)
    assert _band(chart.certificate) <= band_bounds(DEVELOPABLE_STRETCH_BUDGET)[1]


# --------------------------------------------------------------------------
# Конус, купол, седло, спираль
# --------------------------------------------------------------------------


def test_a_cone_sector_with_the_apex_on_the_boundary_is_an_ordinary_sector():
    chart = developable_chart(factories.cone(8, boundary_apex=True))
    certificate = chart.certificate
    assert not stretch_violations(certificate.stretch)
    # Вершина на границе — веер разомкнут: замкнутых вееров, значит ярлыков, нет.
    assert certificate.vertex_classes == frozenset()
    assert _band(certificate) <= band_bounds(DEVELOPABLE_STRETCH_BUDGET)[1]


def test_a_cone_with_an_interior_apex_is_beyond_the_stretch_budget_by_name():
    error = _refusal(factories.cone(8, boundary_apex=False))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:apex" in str(error)
    assert "outside_budget=1" in str(error)


def test_a_half_sphere_is_beyond_the_stretch_budget():
    error = _refusal(factories.dome())
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:" in str(error)


def test_a_saddle_is_refused_and_names_the_vertex_with_the_excess_angle():
    error = _refusal(factories.dome(saddle=True))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:c" in str(error)


def test_a_spiral_ribbon_covers_itself_by_name():
    error = _refusal(factories.spiral_strip())
    assert error.outcome is NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
    assert "boundary edge pairs of the chart meet or overlap" in str(error)


def test_a_ribbon_that_does_not_reach_a_full_turn_does_not_overlap():
    chart = developable_chart(factories.spiral_strip(steps=30, step_degrees=10.0))
    assert chart.certificate.chart_boundary_overlap_count == 0


# --------------------------------------------------------------------------
# Топология носителя: диск, кольцо, несвязность, немногообразие
# --------------------------------------------------------------------------


def test_a_ring_support_requires_a_periodic_cut_by_name():
    error = _refusal(factories.closed_cylinder())
    assert error.outcome is NamedOutcome.PERIODIC_CUT_REQUIRED
    assert "chi=0" in str(error) and "boundary_loops=2" in str(error)


def test_disconnected_triangles_are_not_a_disk():
    points = {
        "a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (0.0, 1.0, 0.0),
        "d": (5.0, 0.0, 0.0), "e": (6.0, 0.0, 0.0), "f": (5.0, 1.0, 0.0),
    }
    error = _refusal(factories.surface(points, [["a", "b", "c"], ["d", "e", "f"]]))
    assert error.outcome is NamedOutcome.DEVELOPABLE_SUPPORT_NOT_A_DISK
    assert "not connected" in str(error)


def test_a_side_with_three_carriers_is_adjacency_unavailable():
    points = {
        "a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0),
        "c": (0.0, 1.0, 0.0), "d": (0.0, -1.0, 0.0), "e": (0.0, 0.0, 1.0),
    }
    error = _refusal(
        factories.surface(points, [["a", "b", "c"], ["b", "a", "d"], ["a", "b", "e"]])
    )
    assert error.outcome is NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE
    assert "carried by 3 owner triangles" in str(error)


def test_inconsistent_orientation_is_adjacency_unavailable():
    points = {
        "a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (0.0, 1.0, 0.0), "d": (1.0, 1.0, 0.0),
    }
    error = _refusal(factories.surface(points, [["a", "b", "c"], ["a", "b", "d"]]))
    assert error.outcome is NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE
    assert "same direction" in str(error)


def test_a_pinched_vertex_is_adjacency_unavailable():
    points = {
        "p": (0.0, 0.0, 0.0), "a": (1.0, 0.0, 0.0), "b": (0.0, 1.0, 0.0),
        "c": (-1.0, 0.0, 0.0), "d": (0.0, -1.0, 0.0),
    }
    error = _refusal(factories.surface(points, [["p", "a", "b"], ["p", "c", "d"]]))
    assert error.outcome is NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE
    assert "vertex v:p" in str(error)


def test_a_degenerate_owner_triangle_is_named():
    points = {"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (2.0, 0.0, 0.0)}
    error = _refusal(factories.surface(points, [["a", "b", "c"]]))
    assert error.outcome is NamedOutcome.DEVELOPABLE_SOURCE_TRIANGLE_DEGENERATE


def test_an_unsnapped_source_has_no_chart_scale():
    error = _refusal(
        factories.fold_strip(), grid_policy=GridSnappingLawV1.UNSNAPPED_EXACT_V1
    )
    assert error.outcome is NamedOutcome.DEVELOPABLE_REQUIRES_SOURCE_SNAP


# --------------------------------------------------------------------------
# Решётка карты: ступени и имя вместо допуска
# --------------------------------------------------------------------------


def test_a_triangle_thinner_than_every_chart_cell_is_lattice_too_coarse_by_name():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _chart_of_single((0, 0, 0), (100, 0, 0), (Fraction(50) + Fraction(1, 7), Fraction(1, 100), 0))
    assert failure.value.outcome is NamedOutcome.DEVELOPABLE_CHART_LATTICE_TOO_COARSE
    assert "the unsnapped proposal is within budget" in str(failure.value)


def test_the_second_chart_scale_takes_over_when_the_first_cell_is_too_coarse():
    height = Fraction(3)
    apex = (Fraction(50) + Fraction(1, 7), height, 0)
    first = _chart_of_single((0, 0, 0), (100, 0, 0), apex)
    assert first.certificate.chart_scale_trials == 2
    assert first.chart_scale == UNFOLD_CHART_SCALE_FACTORS[1] * 1
    easy = _chart_of_single((0, 0, 0), (100, 0, 0), (Fraction(50) + Fraction(1, 7), Fraction(30), 0))
    assert easy.certificate.chart_scale_trials == 1


# --------------------------------------------------------------------------
# Предикат растяжения: точный, без корней
# --------------------------------------------------------------------------


def _gram(scale_squared, base=(Fraction(1), Fraction(0), Fraction(1))):
    return tuple(item * scale_squared for item in base)


def test_the_stretch_predicate_is_exact_at_both_ends_of_the_band():
    low, high = band_bounds(DEVELOPABLE_STRETCH_BUDGET)
    unit = (Fraction(1), Fraction(0), Fraction(1))
    assert in_stretch_band(unit, unit, DEVELOPABLE_STRETCH_BUDGET)
    assert in_stretch_band(unit, _gram(high), DEVELOPABLE_STRETCH_BUDGET)
    assert in_stretch_band(unit, _gram(low), DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, _gram(high + Fraction(1, 10**9)), DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, _gram(low - Fraction(1, 10**9)), DEVELOPABLE_STRETCH_BUDGET)


def test_the_stretch_predicate_sees_a_shear_that_a_trace_would_hide():
    """Квадраты сингулярных чисел сдвига: `λ_max λ_min = 1`, `tr = 2 + s²`."""

    unit = (Fraction(1), Fraction(0), Fraction(1))
    shear = lambda s: (Fraction(1), s, 1 + s * s)  # noqa: E731
    assert in_stretch_band(unit, shear(Fraction(1, 100)), DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, shear(Fraction(1, 5)), DEVELOPABLE_STRETCH_BUDGET)


def test_a_degenerate_chart_triangle_is_outside_the_band_and_has_no_finite_bound():
    unit = (Fraction(1), Fraction(0), Fraction(1))
    flat = (Fraction(1), Fraction(1), Fraction(1))
    assert not in_stretch_band(unit, flat, DEVELOPABLE_STRETCH_BUDGET)
    assert band_squared_upper(unit, flat) is None


def test_the_certified_band_bound_contains_the_true_band():
    unit = (Fraction(1), Fraction(0), Fraction(1))
    for chart in (
        (Fraction(1), Fraction(1, 10), Fraction(1)),
        (Fraction(3, 2), Fraction(0), Fraction(2, 3)),
        (Fraction(4), Fraction(1), Fraction(1, 3)),
    ):
        c00, c01, c11 = chart
        mean = (c00 + c11) / 2
        spread = math.sqrt(float(((c00 - c11) / 2) ** 2 + c01 * c01))
        lowest, highest = float(mean) - spread, float(mean) + spread
        truth = max(highest, 1.0 / lowest)
        bound = band_squared_upper(unit, chart)
        assert float(bound) >= truth * (1 - 1e-12)
        assert float(bound) <= truth * (1 + 1e-9)


# --------------------------------------------------------------------------
# Ярлыки вершин: точное замыкание веера
# --------------------------------------------------------------------------


def test_interior_fold_vertices_are_exactly_developable_by_the_sqrt_sum_closure():
    classes = developable_chart(factories.fold_grid()).certificate.vertex_classes
    assert len(classes) == 6
    for item in classes:
        assert item.developability_class is VertexDevelopabilityClassV1.EXACT_DEVELOPABLE
        assert item.closure_law is DevelopableFanClosureLawV1.EXACT_FAN_CLOSURE_SQRT_SUM_V1


def test_a_planar_closed_fan_is_exact_without_any_root():
    points = {"c": (0.0, 0.0, 0.0)}
    count = 6
    for k in range(count):
        points[f"p{k}"] = (math.cos(2 * math.pi * k / count), math.sin(2 * math.pi * k / count), 0.0)
    cycles = [["c", f"p{k}", f"p{(k + 1) % count}"] for k in range(count)]
    (item,) = developable_chart(factories.surface(points, cycles)).certificate.vertex_classes
    assert item.closure_law is DevelopableFanClosureLawV1.EXACT_PLANAR_CLOSED_FAN_V1
    assert item.developability_class is VertexDevelopabilityClassV1.EXACT_DEVELOPABLE


def test_a_fold_fan_longer_than_the_declared_cap_is_undecided_by_structure():
    (item,) = developable_chart(factories.fold_fan(5)).certificate.vertex_classes
    assert item.fan_triangle_count == 10
    assert item.developability_class is VertexDevelopabilityClassV1.UNDECIDED_WORK_BUDGET
    assert item.closure_law is DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1


def test_a_fold_fan_within_the_cap_is_decided_exactly():
    (item,) = developable_chart(factories.fold_fan(2)).certificate.vertex_classes
    assert item.fan_triangle_count == 4
    assert item.developability_class is VertexDevelopabilityClassV1.EXACT_DEVELOPABLE


def _perturbed_fold_grid(drop):
    vertices, faces, _triangles = factories.fold_grid()
    points = {
        item.vertex_id.value[2:]: (item.position.x, item.position.y, item.position.z)
        for item in vertices
    }
    x, y, z = points["g1_2"]
    points["g1_2"] = (x - drop, y, z)
    return factories.surface(
        points, [[v.value[2:] for v in face.vertex_cycle] for face in faces]
    )


def test_a_proven_non_closing_vertex_in_budget_is_accepted_and_labelled_near_developable():
    """Решение владельца: доказанное `≠ 2π` при растяжении в бюджете — принимается с ярлыком."""

    certificate = developable_chart(_perturbed_fold_grid(0.005)).certificate
    near = [
        item
        for item in certificate.vertex_classes
        if item.developability_class is VertexDevelopabilityClassV1.NEAR_DEVELOPABLE
    ]
    assert near
    assert all(
        item.closure_law is DevelopableFanClosureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1
        for item in near
    )
    assert not stretch_violations(certificate.stretch)
    assert worst_defect_vertex(near).value.startswith("v:g")
    assert max(angle_defect_lower_bound(item) for item in near) > 0


def test_the_same_vertex_beyond_the_budget_is_refused():
    error = _refusal(_perturbed_fold_grid(0.05))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED


# --------------------------------------------------------------------------
# Воспроизводимость и память канонизации
# --------------------------------------------------------------------------


def test_the_chart_is_reproducible_bit_for_bit():
    first = developable_chart(factories.bevel_strip(4))
    second = developable_chart(factories.bevel_strip(4))
    assert first.nodes == second.nodes
    assert canonical_json_bytes(first.certificate) == canonical_json_bytes(second.certificate)


def test_the_closure_label_does_not_depend_on_the_warmth_of_the_factorization_memory():
    cold = developable_chart(factories.fold_grid()).certificate
    SqrtSumV1.radical(1, Fraction(9973 * 10007), exact_work_budget(stage="TEST"))
    warm = developable_chart(factories.fold_grid()).certificate
    assert cold == warm


def test_the_isolated_memory_is_cold_inside_and_restored_outside():
    from cftuv_envelope import exact_sqrt_sum as canon

    reset_factorization_memory()
    SqrtSumV1.radical(1, 2 * 3 * 5 * 7 * 11 * 13 * 17, exact_work_budget(stage="TEST"))
    before = dict(canon._SQUAREFREE_MEMO)
    assert before
    with isolated_factorization_memory():
        assert not canon._SQUAREFREE_MEMO
        SqrtSumV1.radical(1, 19 * 23 * 29, exact_work_budget(stage="TEST"))
        assert 19 * 23 * 29 in canon._SQUAREFREE_MEMO
    assert canon._SQUAREFREE_MEMO == before


# --------------------------------------------------------------------------
# Запись: законы, бюджет, кодек
# --------------------------------------------------------------------------


def test_the_certificate_names_its_laws_budget_and_unit_normal():
    certificate = developable_chart(factories.fold_strip()).certificate
    assert certificate.exact is False
    assert certificate.tree_law.value == "CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1"
    assert certificate.proposal_law.value == "BINARY64_HINGE_V1"
    assert certificate.lift_law.value == "UNFOLDED_SOURCE_TRIANGLES_V1"
    assert certificate.stretch.law.value == "EXACT_GRAM_SINGULAR_VALUE_BAND_V1"
    assert certificate.stretch.stretch_budget == ExactRationalV1(1, 50)
    assert certificate.root_triangle_id.value == "face000:t01"
    assert (
        certificate.exact_plane_normal.x.numerator,
        certificate.exact_plane_normal.y.numerator,
        certificate.exact_plane_normal.z.numerator,
    ) == (0, 0, 1)


def test_the_certificate_survives_the_codec():
    from cftuv_envelope.codec import to_canonical_data

    certificate = developable_chart(factories.bevel_strip(3)).certificate
    data = to_canonical_data(certificate)
    assert data["$type"] == "DevelopableUnfoldCertificateV1"
    assert data["stretch"]["$type"] == "DevelopableStretchCertificateV1"
