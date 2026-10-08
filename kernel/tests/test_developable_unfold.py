"""DEVELOPABLE (S1), срез C1: развёртка по шарнирам, судимая точным растяжением.

Карта домена — привязанная к решётке шарнирная развёртка треугольников источника.
Власть — точный сертификат растяжения (`DevelopableStretchCertificateV1`): все квадраты
сингулярных чисел отображения треугольник источника -> треугольник карты в
`[1/(1+b)², (1+b)²]`, `b = 1/5` (решение владельца: растяжения до 20 %), тремя знаками рациональных чисел. Фикстуры этого
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
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
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

#: Золотой дайджест сертификата складки 90°: карта на целых узлах, растяжение 1, без объявленных цепей.
FOLD_STRIP_CERTIFICATE_SHA256 = (
    "946877f2c015965a1b14041576eba1d5a2a19eed5d67cd5a5801734607726a20"
)

#: Тот же сертификат при прежнем бюджете `1/50` (до решения владельца 2026-10-03 «до 20 %»). Единственное, чем
#: он отличается от золотого, — записанный `stretch_budget`: карта, узлы и все числа растяжения прежние.
FOLD_STRIP_CERTIFICATE_SHA256_AT_ONE_FIFTIETH = (
    "9602eddb03cd1e0219495def2880a2b4c0de81a859572905049fb31bc43f8a33"
)
# Оба дайджеста перезаписаны 2026-10-03 (STRETCH-BUDGET-POLICY + BEST-PROPOSAL): сертификат получил четыре поля
# выбора предложения. Прежние значения (`f492bc8c...` при 1/5, `ef2baa0b...` при 1/50) восстанавливаются
# вычёркиванием ровно этих полей — это закрыто тестом `test_the_new_certificate_fields_are_the_only_bytes_that_changed`.


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


def test_the_budget_is_the_only_byte_that_the_owners_twenty_percent_changed_in_the_fold_certificate():
    """Бюджет 1/50 -> 1/5 меняет в принятом сертификате ровно записанный `stretch_budget`: карта и числа прежние."""

    from dataclasses import replace

    now = developable_chart(factories.fold_strip())
    before = developable_chart(factories.fold_strip(), budget=Fraction(1, 50))
    assert _digest(before.certificate) == FOLD_STRIP_CERTIFICATE_SHA256_AT_ONE_FIFTIETH
    assert before.nodes == now.nodes
    assert before.chart_scale == now.chart_scale
    assert now.certificate.stretch.stretch_budget == ExactRationalV1(1, 5)
    assert before.certificate.stretch.stretch_budget == ExactRationalV1(1, 50)
    restored = replace(
        now.certificate,
        stretch=replace(now.certificate.stretch, stretch_budget=ExactRationalV1(1, 50)),
    )
    assert restored == before.certificate


def test_the_developable_budget_is_one_fifth_and_the_near_planar_width_budget_stays_one_fiftieth():
    from cftuv_envelope.contracts.metric import NEAR_PLANAR_WIDTH_BUDGET

    assert DEFAULT_DEVELOPABLE_STRETCH_BUDGET == Fraction(1, 5)
    assert NEAR_PLANAR_WIDTH_BUDGET == Fraction(1, 50)


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
    _low, high = band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
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
    assert _band(chart.certificate) <= band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)[1]


# --------------------------------------------------------------------------
# Конус, купол, седло, спираль
# --------------------------------------------------------------------------


def test_a_cone_sector_with_the_apex_on_the_boundary_is_an_ordinary_sector():
    chart = developable_chart(factories.cone(8, boundary_apex=True))
    certificate = chart.certificate
    assert not stretch_violations(certificate.stretch)
    # Вершина на границе — веер разомкнут: замкнутых вееров, значит ярлыков, нет.
    assert certificate.vertex_classes == frozenset()
    assert _band(certificate) <= band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)[1]


def test_a_cone_with_an_interior_apex_is_beyond_the_stretch_budget_by_name():
    """Красный контроль над 20 %: конус высотой 1.0 даёт 28.2 % (при высоте 0.5 — 7.3 %, он принят)."""

    error = _refusal(factories.cone(8, rise=1.0, boundary_apex=False))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:apex" in str(error)
    assert "outside_budget=1" in str(error)


def test_a_gentle_cone_with_an_interior_apex_is_within_the_twenty_percent_budget():
    """Владелец принял растяжения до 20 %: конус высотой 0.5 (7.3 %) принимается вторым предложением, ARAP."""

    certificate = developable_chart(factories.cone(8, boundary_apex=False)).certificate
    assert certificate.proposal_law.value == "ARAP_LOCAL_GLOBAL_80_BINARY64_V1"
    assert not stretch_violations(certificate.stretch)
    assert 1.0 < float(_band(certificate)) <= float(band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)[1])


def test_a_half_sphere_is_beyond_the_stretch_budget():
    error = _refusal(factories.dome())
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:" in str(error)


def test_a_saddle_is_refused_and_names_the_vertex_with_the_excess_angle():
    """Седло высотой 1.0 даёт 26.5 % — за 20 %; седло высотой 0.5 (9.9 %) принято."""

    error = _refusal(factories.dome(saddle=True, saddle_height=1.0))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:c" in str(error)


def test_a_gentle_saddle_is_within_the_twenty_percent_budget():
    certificate = developable_chart(factories.dome(saddle=True)).certificate
    assert not stretch_violations(certificate.stretch)
    assert float(_band(certificate)) <= float(band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)[1])


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
    height = Fraction(1, 3)
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
    low, high = band_bounds(DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    unit = (Fraction(1), Fraction(0), Fraction(1))
    assert in_stretch_band(unit, unit, DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert in_stretch_band(unit, _gram(high), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert in_stretch_band(unit, _gram(low), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, _gram(high + Fraction(1, 10**9)), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, _gram(low - Fraction(1, 10**9)), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)


def test_the_stretch_predicate_sees_a_shear_that_a_trace_would_hide():
    """Квадраты сингулярных чисел сдвига: `λ_max λ_min = 1`, `tr = 2 + s²`."""

    unit = (Fraction(1), Fraction(0), Fraction(1))
    shear = lambda s: (Fraction(1), s, 1 + s * s)  # noqa: E731
    assert in_stretch_band(unit, shear(Fraction(1, 100)), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert in_stretch_band(unit, shear(Fraction(1, 5)), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
    assert not in_stretch_band(unit, shear(Fraction(1, 2)), DEFAULT_DEVELOPABLE_STRETCH_BUDGET)


def test_a_degenerate_chart_triangle_is_outside_the_band_and_has_no_finite_bound():
    unit = (Fraction(1), Fraction(0), Fraction(1))
    flat = (Fraction(1), Fraction(1), Fraction(1))
    assert not in_stretch_band(unit, flat, DEFAULT_DEVELOPABLE_STRETCH_BUDGET)
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
    """Сдвиг 0.8 не вмещается в бюджет 20 % ни у шарнира (63 %), ни у ARAP (24 %); при 0.3 ARAP вмещает (10 %)."""

    error = _refusal(_perturbed_fold_grid(0.8))
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
    assert certificate.stretch.stretch_budget == ExactRationalV1(1, 5)
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


# --------------------------------------------------------------------------
# Независимые граничные controls детерминированного кандидата угла
# --------------------------------------------------------------------------


def _machin_two_pi_bounds():
    """Рациональный oracle: 2π = 32 atan(1/5) - 8 atan(1/239), знак остатка ряда."""
    def atan_bounds(x):
        terms = 64
        partial = sum(((-1)**k * x**(2*k + 1) / (2*k + 1) for k in range(terms)), Fraction(0))
        following = partial + (-1)**terms * x**(2*terms + 1) / (2*terms + 1)
        return min(partial, following), max(partial, following)

    a, b = atan_bounds(Fraction(1, 5)), atan_bounds(Fraction(1, 239))
    return 32*a[0] - 8*b[1], 32*a[1] - 8*b[0]


def _rational_fan(rays, *, name="c", scale=1):
    """Замкнутый ориентированный диск: точные рациональные лучи, без snap/float."""
    center = SourceVertexId(f"v:{name}")
    rim = [SourceVertexId(f"v:{name}:p{k}") for k in range(len(rays))]
    positions = {center: (Fraction(0),)*3}
    positions.update({vertex: tuple(Fraction(x)*scale for x in ray)
                      for vertex, ray in zip(rim, rays, strict=True)})
    triangles = [SurfaceTriangleV1(
        SurfaceTriangleId(f"{name}:t{k}"), SourceFaceId(f"{name}:f{k}"),
        (center, rim[k], rim[(k + 1) % len(rim)]), (None,)*3, LocalVector3V1(0, 0, 1),
    ) for k in range(len(rim))]
    return center, tuple(item.triangle_id for item in triangles), {item.triangle_id: item for item in triangles}, positions, name


def _symmetric_cone_rays(sign, height):
    # Смежные скалярные произведения = sign*h²; четыре нормы² = 1+h².
    return ((1, 0, height), (0, 1, sign*height), (-1, 0, height), (0, -1, sign*height))


def _folded_rational_rays(subdivisions):
    # Четыре квадранта исходной плоскости; нижняя полуплоскость сложена на 90°.
    quarter = {1: ((1, 0),), 2: ((1, 0), (1, 1)), 3: ((1, 0), (2, 1), (1, 2))}[subdivisions]
    rays = []
    for turn in range(4):
        for first, second in quarter:
            x, y = first, second
            for _ in range(turn):
                x, y = -y, x
            rays.append((x, y, 0) if y >= 0 else (x, 0, -y))
    return tuple(rays)


def _boundary_seed_controls(*, captured=False):
    from cftuv_envelope import surface_cone_angle as cone

    controls = [("local_libm", math.acos), ("deterministic", cone._acos_seed)]
    if captured:
        from test_developable_arap import _legacy_cone_seed

        controls.extend((platform, _legacy_cone_seed(platform)) for platform in ("windows", "linux"))
    return controls


def _classify_seed_control(monkeypatch, fan, seed, *, cap=None):
    from cftuv_envelope import _fan_closure as closure, surface_cone_angle as cone

    calls, budgets = [], []
    def spy(argument):
        result = seed(argument)
        calls.append((argument.hex(), result.hex()))
        return result
    def budget_factory(**kwargs):
        budget = exact_work_budget(**kwargs, cap=cap)
        budgets.append(budget)
        return budget
    with monkeypatch.context() as scoped:
        scoped.setattr(cone, "_acos_seed", spy)
        if cap is not None:
            scoped.setattr(closure, "exact_work_budget", budget_factory)
        result = closure.classify_vertex(*fan)
    return result, calls, [(item.cap, item.spent) for item in budgets]


def _boundary_control_record(mode, item, calls, budgets=()):
    from cftuv_envelope.codec import to_canonical_data

    return dict(mode=mode, classification=to_canonical_data(item), acos_calls=calls,
                budgets=budgets, defect_lower_bound=str(angle_defect_lower_bound(item)))


@pytest.mark.parametrize("sign", (1, -1), ids=("below", "above"))
@pytest.mark.parametrize("power", (10, 30))
def test_seed_boundary_two_pi_has_independent_rational_bounds(monkeypatch, record_property, sign, power):
    """Истинный знак не заменяет классификацию: при пересечении 2π разрешён именованный UNKNOWN."""
    height = Fraction(1, 2**power)
    cosine = height*height / (1 + height*height)
    pi_low, pi_high = _machin_two_pi_bounds()
    # Для 0<c<1: c <= asin(c) <= c/(1-c²), интеграл монотонной производной.
    delta_low, delta_high = 4*cosine, 4*cosine/(1 - cosine*cosine)
    oracle = (pi_low - delta_high, pi_high - delta_low) if sign > 0 else (pi_low + delta_low, pi_high + delta_high)
    assert oracle[1] < pi_low if sign > 0 else oracle[0] > pi_high
    fan = _rational_fan(_symmetric_cone_rays(sign, height))
    records, decisions = [], []
    for mode, seed in _boundary_seed_controls():
        item, calls, budgets = _classify_seed_control(monkeypatch, fan, seed)
        low, high = Fraction(item.angle_sum_enclosure.lower), Fraction(item.angle_sum_enclosure.upper)
        assert low <= oracle[0] <= oracle[1] <= high
        assert item.developability_class is not VertexDevelopabilityClassV1.EXACT_DEVELOPABLE
        contains = low <= pi_low and pi_high <= high
        assert contains is (power == 30)
        if power == 10:
            assert item.developability_class is VertexDevelopabilityClassV1.NEAR_DEVELOPABLE
            assert item.closure_law is DevelopableFanClosureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1
        else:
            assert item.closure_law in (DevelopableFanClosureLawV1.EXACT_FAN_CLOSURE_SQRT_SUM_V1,
                                        DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1)
        decisions.append((item.developability_class, item.closure_law, angle_defect_lower_bound(item) > 0))
        records.append(_boundary_control_record(mode, item, calls, budgets))
    assert decisions[0] == decisions[1]
    record_property("seed_boundary", dict(height=str(height), cosine=str(sign*cosine),
                    oracle=[str(x) for x in oracle], linux_legacy="UNKNOWN", records=records))


def test_seed_flat_equality_bypasses_acos_and_exact_work(monkeypatch, record_property):
    fan = _rational_fan(_symmetric_cone_rays(1, Fraction(0)))
    def forbidden(_argument):
        raise AssertionError("flat exact fan must not request a seed")
    item, calls, budgets = _classify_seed_control(monkeypatch, fan, forbidden, cap=0)
    oracle = _machin_two_pi_bounds()
    assert Fraction(item.angle_sum_enclosure.lower) <= oracle[0] <= oracle[1] <= Fraction(item.angle_sum_enclosure.upper)
    assert item.developability_class is VertexDevelopabilityClassV1.EXACT_DEVELOPABLE
    assert item.closure_law is DevelopableFanClosureLawV1.EXACT_PLANAR_CLOSED_FAN_V1
    assert calls == budgets == []
    assert angle_defect_lower_bound(item) == 0 and worst_defect_vertex((item,)) is None
    record_property("seed_boundary", _boundary_control_record("seed_independent", item, calls, budgets))


@pytest.mark.parametrize("subdivisions,cap", ((1, None), (2, None), (3, None), (2, 0)))
def test_seed_folded_fan_keeps_structural_and_exact_work_limits(monkeypatch, record_property, subdivisions, cap):
    """Изометрия двух полуплоскостей доказывает 2π независимо от численного ядра."""
    fan = _rational_fan(_folded_rational_rays(subdivisions))
    oracle = _machin_two_pi_bounds()
    records, decisions = [], []
    for mode, seed in _boundary_seed_controls(captured=subdivisions <= 2):
        item, calls, budgets = _classify_seed_control(monkeypatch, fan, seed, cap=cap)
        assert Fraction(item.angle_sum_enclosure.lower) <= oracle[0] <= oracle[1] <= Fraction(item.angle_sum_enclosure.upper)
        assert item.fan_triangle_count == 4*subdivisions
        limited = subdivisions > 2 or cap == 0
        assert item.developability_class is (VertexDevelopabilityClassV1.UNDECIDED_WORK_BUDGET if limited else VertexDevelopabilityClassV1.EXACT_DEVELOPABLE)
        assert item.closure_law is (DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1 if limited else DevelopableFanClosureLawV1.EXACT_FAN_CLOSURE_SQRT_SUM_V1)
        assert angle_defect_lower_bound(item) == 0 and worst_defect_vertex((item,)) is None
        if cap == 0:
            assert len(budgets) == 1 and budgets[0][0] == 0 and budgets[0][1] > 0
        decisions.append((item.developability_class, item.closure_law))
        records.append(_boundary_control_record(mode, item, calls, budgets))
    assert all(item == decisions[0] for item in decisions)
    record_property("seed_boundary", dict(count=4*subdivisions, cap=cap,
                    linux_legacy="CAPTURED" if subdivisions <= 2 else "UNKNOWN", records=records))


def test_seed_equal_defects_have_a_stable_id_tie_break(monkeypatch, record_property):
    from itertools import permutations

    rays = _symmetric_cone_rays(1, Fraction(1, 2**10))
    records = []
    for mode, seed in _boundary_seed_controls():
        # Изометрия и равномерный масштаб сохраняют истинный ненулевой дефект точно.
        first, calls, _ = _classify_seed_control(monkeypatch, _rational_fan(rays, name="a"), seed)
        second, other_calls, _ = _classify_seed_control(monkeypatch, _rational_fan(rays, name="z", scale=8), seed)
        flat, _, _ = _classify_seed_control(monkeypatch, _rational_fan(_symmetric_cone_rays(1, 0), name="0"), seed)
        assert first.angle_sum_enclosure == second.angle_sum_enclosure
        assert angle_defect_lower_bound(first) == angle_defect_lower_bound(second) > 0
        for order in permutations((first, second, flat)):
            assert worst_defect_vertex(order) == SourceVertexId("v:a")
            ranking = [item.vertex_id.value for item in sorted(order, key=lambda item: (-angle_defect_lower_bound(item), item.vertex_id.value))]
            assert ranking == ["v:a", "v:z", "v:0"]
        records.append(dict(mode=mode, first=_boundary_control_record(mode, first, calls),
                            second=_boundary_control_record(mode, second, other_calls), winner="v:a"))
    record_property("seed_boundary", dict(linux_legacy="UNKNOWN", records=records))
