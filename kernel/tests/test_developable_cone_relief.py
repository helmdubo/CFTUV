"""DEVELOPABLE-CONE-RELIEF: запас угла у граничной вершины, чей разомкнутый веер не помещается в оборот.

Патч 89 `building` — ступенька 1.6 см на стене: 12 треугольников, внутренних вершин нет, диск с одной
петлёй. Шарнирная развёртка изометрична (растяжение 1 + 1e-15), но граничная вершина `34` несёт веер
360.167° (> 2π), и последний треугольник веера накрывает первый: граничные рёбра карты пересекаются.
ARAP из изометрии не выходит (энергия нуль, накрытие её не штрафует), поэтому раньше отказ оставался
`DEVELOPABLE_CHART_SELF_OVERLAP`. Третье предложение меняет ЦЕЛЬ: угол при такой вершине сжат до зазора
(`_cone_relief`, закон `CONE_RELIEF_NLERP_V1`), а карту судит тот же точный суд.

Здесь проверяется: план называет ровно вершину с избытком; шарнир без плана отказывает по-прежнему;
с планом карта принята в бюджете растяжения с простой границей и называет закон и отказ шарнира;
предложение побитово воспроизводимо; сертификат проходит проводную проверку и пересчёт, а подмена
закона либо следа ловится; красные контроли — спираль ленты (вершины с избытком нет) и избыток,
который стоит больше бюджета, остаются именованными отказами; ARAP, когда он нужен, прежний.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import replace
from fractions import Fraction

import pytest

import cftuv_envelope._arap as arap
import cftuv_envelope._developable as developable
from cftuv_envelope._cone_relief import (
    CONE_RELIEF_GAP_HALF_TURNS,
    CONE_RELIEF_MAX_STEPS,
    CONE_RELIEF_STEPS,
    cone_relief_plan,
    relief_note,
    relieved_squares,
)
from cftuv_envelope._stretch import band_bounds, stretch_violations
from cftuv_envelope._unfold import owner_topology
from cftuv_envelope.contracts.metric import (
    CurvatureLadderPolicyV1,
    DevelopableProposalLawV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.validation_developable import (
    check_developable_certificate,
    validate_developable_recomputation,
)
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
from developable_factories import PATCH, REVISION, developable_chart
from developable_route import build_metric

ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1
HINGE = DevelopableProposalLawV1.BINARY64_HINGE_V1
ARAP = DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1
RELIEF = DevelopableProposalLawV1.ARAP_CONE_RELIEF_80_BINARY64_V1
OVERLAP = NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP

#: Ступенька `building` патч 89 (слепок поля 2026-10-03): позиции источника после привязки (сетка
#: 1/4096 м) и 12 треугольников владельца в порядке обхода. Вершина `34` — граничная, веер 360.167°.
STEP_POINTS = {
    "7": (-22.301513671875, 28.18017578125, 58.417724609375),
    "9": (-22.301513671875, 28.18017578125, 49.7177734375),
    "33": (-22.301513671875, 20.980224609375, 58.417724609375),
    "34": (-22.285400390625, 20.980224609375, 52.9853515625),
    "40": (-22.301513671875, 18.414794921875, 49.7177734375),
    "41": (-22.301513671875, 18.414794921875, 52.9853515625),
    "44": (-22.301513671875, 20.980224609375, 49.7177734375),
    "46": (-22.301513671875, 18.414794921875, 51.802001953125),
    "48": (-22.301513671875, 28.18017578125, 52.9853515625),
    "121": (-22.285400390625, 20.980224609375, 55.6123046875),
    "242": (-22.285400390625, 20.380126953125, 52.9853515625),
    "244": (-22.285400390625, 20.380126953125, 55.6123046875),
    "296": (-22.285400390625, 17.914794921875, 49.360107421875),
    "300": (-22.285400390625, 17.914794921875, 51.802001953125),
}
STEP_TRIANGLES = [
    ["242", "34", "121"],
    ["242", "121", "244"],
    ["300", "296", "40"],
    ["300", "40", "46"],
    ["46", "40", "44"],
    ["44", "34", "41"],
    ["46", "44", "41"],
    ["44", "9", "48"],
    ["44", "48", "34"],
    ["33", "121", "34"],
    ["34", "48", "7"],
    ["34", "7", "33"],
]
#: Золотой дайджест целых узлов карты: побитовая воспроизводимость закона (атан2 решает только план).
STEP_NODES_DIGEST = "b0297d2282a1266a5c390fd7cc48f73e9a5cc561fa8437082b99eb11ef64c18b"


def _step_patch():
    return factories.surface(STEP_POINTS, STEP_TRIANGLES)


def _exact(parts):
    vertices, _faces, triangles = parts
    snapped = {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    return owner_topology(triangles, snapped), snapped


def _saddle_fan(excess_degrees, sectors=6, radius=1.0):
    """Веер из `sectors` треугольников вокруг граничной вершины `apex`, сумма углов `360 + excess`.

    Кольцо точек на окружности радиуса `radius` с чередующейся высотой `±h`; азимутальный шаг и `h`
    подобраны делением пополам так, чтобы сумма трёхмерных углов при `apex` дала нужную величину.
    """

    def build(height):
        azimuth = 2.0 * math.pi * 0.84 / sectors
        points = {"apex": (0.0, 0.0, 0.0)}
        for k in range(sectors + 1):
            z = height * (1 if k % 2 == 0 else -1)
            points[f"b{k}"] = (
                radius * math.cos(azimuth * k),
                radius * math.sin(azimuth * k),
                z,
            )
        return points

    def total(points):
        names = [f"b{k}" for k in range(sectors + 1)]
        angle = 0.0
        for first, second in zip(names, names[1:]):
            a, b = points[first], points[second]
            dot = sum(x * y for x, y in zip(a, b))
            cross = math.sqrt(
                (a[1] * b[2] - a[2] * b[1]) ** 2
                + (a[2] * b[0] - a[0] * b[2]) ** 2
                + (a[0] * b[1] - a[1] * b[0]) ** 2
            )
            angle += math.atan2(cross, dot)
        return math.degrees(angle)

    low, high = 0.0, radius
    for _ in range(60):
        middle = (low + high) / 2.0
        if total(build(middle)) < 360.0 + excess_degrees:
            low = middle
        else:
            high = middle
    points = build(high)
    cycles = [["apex", f"b{k}", f"b{k + 1}"] for k in range(sectors)]
    return factories.surface(points, cycles), total(points)


def _refusal(parts, **overrides) -> PlanarMetricAdmissionError:
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(parts, **overrides)
    return failure.value


# --------------------------------------------------------------------------
# План: кому нужен запас
# --------------------------------------------------------------------------


def test_the_plan_names_exactly_the_boundary_vertex_with_the_excess():
    topology, snapped = _exact(_step_patch())

    plan = cone_relief_plan(topology, snapped)

    assert [item.vertex_id.value for item in plan] == ["v:34"]
    (item,) = plan
    assert math.degrees(item.angle_sum) == pytest.approx(360.167351, abs=1e-5)
    assert 0 < item.steps <= CONE_RELIEF_MAX_STEPS
    # Первый порядок: сжатие `rho * sum(sin)` покрывает избыток над `2π − g` (с запасом квантования вверх).
    gap = float(CONE_RELIEF_GAP_HALF_TURNS) * math.pi
    assert item.angle_sum > 2.0 * math.pi - gap
    assert "v:34" in relief_note(plan) and "CONE_RELIEF_NLERP_V1" in relief_note(plan)


def test_a_fan_that_fits_the_turn_with_the_gap_needs_no_relief():
    parts, total = _saddle_fan(-5.0)
    assert total < 360.0 - 2.0
    topology, snapped = _exact(parts)
    assert cone_relief_plan(topology, snapped) == ()


def test_interior_vertices_and_flat_patches_are_never_in_the_plan():
    for parts in (factories.fold_strip(), factories.cone(8, boundary_apex=False), factories.dome()):
        topology, snapped = _exact(parts)
        assert cone_relief_plan(topology, snapped) == ()


def test_the_relieved_target_shrinks_the_far_side_and_leaves_other_triangles_alone():
    topology, snapped = _exact(_step_patch())
    plan = cone_relief_plan(topology, snapped)
    targets = relieved_squares(topology, snapped, plan)

    relieved_ids = {
        triangle.triangle_id
        for triangle in topology.triangles
        if any(plan[0].vertex_id == vertex for vertex in triangle.vertex_ids)
    }
    assert set(targets) == relieved_ids
    assert len(relieved_ids) == 6
    for triangle in topology.triangles:
        if triangle.triangle_id not in targets:
            continue
        ids = triangle.vertex_ids
        apex = ids.index(plan[0].vertex_id)
        left, right = (apex + 1) % 3, (apex + 2) % 3
        pair = tuple(sorted((left, right)))
        key = {(0, 1): 0, (0, 2): 1, (1, 2): 2}[pair]
        original = sum(
            (snapped[ids[left]][axis] - snapped[ids[right]][axis]) ** 2 for axis in range(3)
        )
        assert targets[triangle.triangle_id][key] < original


# --------------------------------------------------------------------------
# Предложение принято, и это записано
# --------------------------------------------------------------------------


def test_the_hinge_alone_covers_itself_on_the_step_patch(monkeypatch):
    """Красный контроль: без плана запаса ступенька отказывает самонакрытием, как отказывала раньше."""

    monkeypatch.setattr(developable, "cone_relief_plan", lambda *_args: ())
    error = _refusal(_step_patch())
    assert error.outcome is OVERLAP
    assert "the hinge unfolding covers itself before any snapping" in str(error)
    assert "ARAP" not in str(error)


def test_the_step_patch_is_accepted_with_the_relief_within_the_stretch_budget():
    chart = developable_chart(_step_patch())
    certificate = chart.certificate

    assert certificate.proposal_law is RELIEF
    assert certificate.previous_refusals == (OVERLAP.value,)
    assert not stretch_violations(certificate.stretch)
    assert certificate.chart_boundary_overlap_count == 0
    assert certificate.stretch.chart_flipped_triangle_count == 0
    band = certificate.stretch.worst_band_squared_upper
    # Избыток 0.167° и запас 2° стоят доли процента; изометрия была 1 + 1e-15.
    assert 1.0 < band.numerator / band.denominator < 1.05
    assert band.numerator / band.denominator <= float(band_bounds(Fraction(1, 5))[1])


def test_the_relief_is_reproducible_bit_for_bit():
    first = developable_chart(_step_patch())
    again = developable_chart(_step_patch())
    assert first.nodes == again.nodes and first.certificate == again.certificate
    digest = hashlib.sha256(
        "\n".join(
            f"{vertex.value}:{node[0]}:{node[1]}"
            for vertex, node in sorted(first.nodes.items(), key=lambda item: item[0].value)
        ).encode()
    ).hexdigest()
    assert digest == STEP_NODES_DIGEST


def test_a_saddle_fan_with_an_excess_is_accepted_and_the_same_fan_without_relief_is_not(monkeypatch):
    parts, total = _saddle_fan(1.0)
    assert total == pytest.approx(361.0, abs=1e-6)
    chart = developable_chart(parts)
    assert chart.certificate.proposal_law is RELIEF
    assert chart.certificate.chart_boundary_overlap_count == 0

    monkeypatch.setattr(developable, "cone_relief_plan", lambda *_args: ())
    assert _refusal(parts).outcome is OVERLAP


def test_a_domain_the_hinge_accepts_never_reaches_the_relief(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("the cone relief ran for a hinge-accepted domain")

    monkeypatch.setattr(developable, "cone_relief_plan", forbidden)
    for parts in (factories.fold_strip(), factories.bevel_strip(4), factories.quarter_cylinder(8)):
        assert developable_chart(parts).certificate.proposal_law is HINGE


def test_the_second_proposal_still_belongs_to_arap_when_the_hinge_is_distorted(monkeypatch):
    """ARAP лечит искажение: домен, где шарнир за бюджетом, получает прежний ARAP, а не запас угла."""

    def forbidden(*_args, **_kwargs):
        raise AssertionError("the cone relief ran for a distorted hinge")

    monkeypatch.setattr(developable, "cone_relief_plan", forbidden)
    vertices, faces, _triangles = factories.fold_grid()
    points = {
        item.vertex_id.value[2:]: (item.position.x, item.position.y, item.position.z)
        for item in vertices
    }
    x, y, z = points["g1_2"]
    points["g1_2"] = (x - 0.3, y, z)
    parts = factories.surface(points, [[v.value[2:] for v in face.vertex_cycle] for face in faces])
    assert developable_chart(parts).certificate.proposal_law is ARAP


# --------------------------------------------------------------------------
# Красные контроли: запас не лечит то, что лечить нельзя
# --------------------------------------------------------------------------


def test_a_spiral_ribbon_has_no_cone_vertex_and_keeps_its_refusal(monkeypatch):
    """Лента, закрученная на 400°, накрывает себя без единой вершины с избытком: план пуст, отказ прежний."""

    def forbidden(*_args, **_kwargs):
        raise AssertionError("ARAP was tried for an overlap without distortion")

    monkeypatch.setattr(developable, "arap_proposal", forbidden)
    topology, snapped = _exact(factories.spiral_strip())
    assert cone_relief_plan(topology, snapped) == ()
    error = _refusal(factories.spiral_strip())
    assert error.outcome is OVERLAP
    assert "the hinge unfolding covers itself before any snapping" in str(error)


def test_a_moderate_excess_inside_the_budget_is_accepted_and_a_large_one_is_not():
    """Владелец принял растяжения до 20 %: веер 420° (сжатие угла на 14 %) принят, веер 510° (на 29 %) — нет."""

    parts, total = _saddle_fan(60.0)
    assert total == pytest.approx(420.0, abs=1e-5)
    certificate = developable_chart(parts).certificate
    assert certificate.proposal_law is RELIEF and not stretch_violations(certificate.stretch)


def test_an_excess_that_costs_more_than_the_budget_is_refused_by_name_with_the_relief_numbers():
    parts, total = _saddle_fan(150.0)
    assert total == pytest.approx(510.0, abs=1e-5)

    error = _refusal(parts)

    assert error.outcome in {
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED,
        OVERLAP,
        NamedOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED,
    }
    text = str(error)
    assert "ARAP_CONE_RELIEF_80_BINARY64_V1 after the hinge proposal was refused" in text
    assert "cone relief CONE_RELIEF_NLERP_V1" in text
    assert "v:apex" in text


def test_a_stricter_budget_refuses_the_step_patch_by_name_and_not_silently():
    error = _refusal(_step_patch(), budget=Fraction(1, 10_000))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "ARAP_CONE_RELIEF_80_BINARY64_V1" in str(error)


# --------------------------------------------------------------------------
# Сертификат: по проводу и пересчётом
# --------------------------------------------------------------------------


def _record(parts):
    return build_metric(parts, ladder=ON)


def _issues(record, parts):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=factories.DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
    )


def test_the_ladder_builds_the_step_patch_and_the_validator_passes_and_recomputes():
    parts = _step_patch()
    record = _record(parts)
    certificate = record.metric.planarity_certificate

    assert certificate.proposal_law is RELIEF
    assert certificate.previous_refusals[-1] == OVERLAP.value
    assert len(certificate.previous_refusals) == 2
    assert _issues(record, parts) == ()
    vertices, faces, triangles = parts
    assert (
        validate_developable_recomputation(
            record.metric,
            source_vertices=vertices,
            source_faces=faces,
            surface_triangles=triangles,
            owner_patch_id=PATCH,
        )
        == ()
    )


def _tampered(record, **changes):
    certificate = replace(record.metric.planarity_certificate, **changes)
    return replace(record, metric=replace(record.metric, planarity_certificate=certificate))


def test_a_swapped_law_is_caught_by_the_recomputation():
    parts = _step_patch()
    record = _record(parts)
    for law in (HINGE, ARAP):
        issues = _issues(_tampered(record, proposal_law=law), parts)
        assert any("differs from exact recomputation" in item.message for item in issues), law


def test_the_relief_trace_must_end_with_the_self_overlap():
    parts = _step_patch()
    record = _record(parts)
    certificate = record.metric.planarity_certificate
    wrong = (*certificate.previous_refusals[:-1], NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value)
    issues: list = []
    check_developable_certificate(issues, ("path",), _tampered(record, previous_refusals=wrong).metric)
    assert any("self-overlap" in item.message for item in issues)


def test_the_law_name_carries_its_iteration_count_like_arap():
    assert RELIEF.value == f"ARAP_CONE_RELIEF_{arap.ARAP_PROPOSAL_ITERATIONS}_BINARY64_V1"
    assert CONE_RELIEF_STEPS > CONE_RELIEF_MAX_STEPS > 0
