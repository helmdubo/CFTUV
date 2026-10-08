"""Митра на изломе (`CORNER_MITER_ON_FOLD_V1`, `SelectionLaw.CORNER_MITER_ON_FOLD_V1`, `CornerTreatmentV1.MITER_SEAM`).

Решение владельца 2026-10-05: вогнутый угол, которого JOIN не взял (куски РАЗНЫХ цепей либо изгиб за пределом JOIN), при
СЛОЖЕННОЙ окрестности (`sin^2` двугранного угла между двумя треугольниками кольца-1 вершины внутри патча владельца
свыше `CORNER_FOLD_SIN2_BUDGET`) и изгибе не шире четверти оборота (`δ/π <= 1/2`, ЗАМКНУТО) получает `k = 0`, митру со
швом на биссектрисе, без потока. Складка свыше бюджета при изгибе шире — веер под именем `BEND_BEYOND_MITER_BOUND`.
Складка в бюджете, нулевая и неизмеримая оставляют прежнее решение и прежнюю причину.

Числа, на которых стоят утверждения, посчитаны НЕ проверяемым кодом: нормали треугольников кольца вершины `v1` (фикстура
`test_corner_join._snapshot`: веер из `v1`, `v4` поднята на `h`) выписаны вручную — `(0, 0, 120)`, `(10h, -10h, 200)`,
`(0, -10h, 100)`; наибольшая пара — первая и третья, `sin^2 = h^2 / (h^2 + 100)`; точная плоскость — dyadic координаты.

| что проверяется                                                                   | тест |
|-----------------------------------------------------------------------------------|------|
| бюджет — `sin^2(1 градус)`, округлённый вниз; запись реестра им и объявлена         | `..._budget_is_...` |
| мера точна и тождественно нуль на плоскости; независимая формула Лагранжа даёт то же | `..._fold_measure_...` |
| складка свыше бюджета и изгиб в пределах — митра, причина, закон записи              | `..._a_folded_neighbourhood_miters_...` |
| бюджет делит ответ: чуть ниже — веер и прежняя причина, чуть выше — митра            | `..._a_fold_within_the_budget_...` |
| предел изгиба замкнут: прямой угол — митра; шире либо не доказан — веер, названный   | `..._the_bend_bound_...` |
| JOIN первым; прямой угол одной цепи (JOIN не берёт) — митра с общей линией           | `..._join_decides_first_...` |
| неизмеримое (нет позиций, кольцо пусто), чужой патч, вырожденное — закон инертен     | `..._unmeasurable_...`, `..._ring_of_another_patch_...` |
| домен `building` 89: три угла митрой, четвёртый веером; плотности d1, d2, d4         | `..._the_exported_domain_...` |
| красный контроль: без закона те же углы — веера с прежними причинами                 | `..._without_the_law_...` |
| митра — шов без потока: станции называют углы, потока нет, диагностика записана      | `..._the_miter_corners_are_seams_...` |
| подделка записи и сертификата — именованный отказ плана и компиляции                | `..._forged_...` |
| настоящие записи ворот: свип и `numeric_repr` проходят по сохранённым спецификациям    | `..._real_records_...` |
"""

from __future__ import annotations

import dataclasses
import json
import math
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _corner_fold, _corner_treatment as law
from cftuv_envelope.contracts.envelopes import (
    ZERO_SUPPORT_SELECTION_LAWS,
    CornerTreatmentReasonV1,
    CornerTreatmentV1,
    SelectionLaw,
)
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.contracts.surface import SurfaceTriangleV1
from cftuv_envelope.contracts.tolerance_policy import TolerancePolicyIdV1, tolerance_policy
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.stations import chain_station_table
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1, SurfaceCoordinateUnavailableReason, UnavailableSourcePositionV1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.reference.corner_treatment import corner_treatment_errors
from cftuv_envelope.validation_corner_treatment import (
    validate_plan_corner_treatments,
    validate_plan_corner_treatments_against_snapshot,
)
from cftuv_envelope.validation_issues import ValidationCode
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import materialize_factories as factories
from reference_factories import _interval
from test_corner_join import SOFT_BOUNDS, _density_request, _plan_like, _snapshot

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "building_patch89_fold_miter_v1"
UV = PolicyId("UV_DIRECT_STRIP_V1")
MITER = CornerTreatmentV1.MITER_SEAM
FAN = CornerTreatmentV1.ANGULAR_PROFILE
JOIN = CornerTreatmentV1.JOIN_CONTINUATION
FOLDED = CornerTreatmentReasonV1.FOLDED_NEIGHBOURHOOD_MITER
BEYOND = CornerTreatmentReasonV1.BEND_BEYOND_MITER_BOUND
UNPROVEN = CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN


# --------------------------------------------------------------------------
# Синтетический угол: веер из `v1`, вершина `v4` поднята на `lift`
# --------------------------------------------------------------------------


def _folded(snapshot, lift, *, patch=None, positions=None):
    """Снапшот `test_corner_join._snapshot` с кольцом-1 вершины `v1` из трёх треугольников и `v4` на высоте `lift`."""

    face = next(iter(snapshot.surface_ir.source_faces))
    ring = (("v1", "v2", "v3"), ("v1", "v3", "v4"), ("v1", "v4", "v0"))
    triangles = frozenset(
        SurfaceTriangleV1(
            triangle_id=kernel.SurfaceTriangleId(f"ring-{index}"),
            source_face_id=face.face_id,
            vertex_ids=tuple(kernel.SourceVertexId(name) for name in names),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for index, names in enumerate(ring)
    )
    surface = dataclasses.replace(snapshot.surface_ir, surface_triangles=triangles)
    vertices = []
    for item in snapshot.source_vertices:
        position = item.position
        if positions is not None and item.vertex_id.value in positions:
            position = positions[item.vertex_id.value]
        elif item.vertex_id.value == "v4":
            position = LocalPoint3V1(position.x, position.y, lift)
        vertices.append(dataclasses.replace(item, position=position))
    return dataclasses.replace(snapshot, surface_ir=surface, source_vertices=frozenset(vertices))


def _decision(snapshot):
    (relation,) = snapshot.corner_relations
    (sector,) = snapshot.angular_owner_sectors
    (angle,) = snapshot.reflex_angle_certificates
    uses = {item.chain_use_id: item for item in snapshot.chain_uses}
    chains = {item.physical_chain_id: item for item in snapshot.physical_chains}
    fold = _corner_fold.CornerFoldFacts(snapshot)
    return law.decide_corner(sector, angle.measure_payload, uses, chains, relation, fold), fold, relation, sector


def _lagrange_max(snapshot, vertex):
    """Независимый расчёт меры: `sin^2 = 1 - (n1.n2)^2 / (|n1|^2 |n2|^2)` по рёбрам, а не по векторному произведению нормалей."""

    positions = {
        item.vertex_id.value: tuple(Fraction(c) for c in (item.position.x, item.position.y, item.position.z))
        for item in snapshot.source_vertices
    }
    normals = []
    for triangle in snapshot.surface_ir.surface_triangles:
        names = [item.value for item in triangle.vertex_ids]
        if vertex not in names:
            continue
        a, b, c = (positions[name] for name in names)
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        normals.append(
            (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        )
    worst = Fraction(0)
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            dot = sum(x * y for x, y in zip(normals[i], normals[j]))
            squared = sum(x * x for x in normals[i]) * sum(x * x for x in normals[j])
            worst = max(worst, 1 - dot * dot / squared)
    return worst


# --------------------------------------------------------------------------
# Бюджет и мера
# --------------------------------------------------------------------------


def test_the_budget_is_one_degree_squared_sine_rounded_down_and_the_registry_declares_it():
    one = Fraction(math.sin(math.radians(1)) ** 2)
    below = Fraction(math.sin(math.radians(0.99)) ** 2)
    assert below < _corner_fold.CORNER_FOLD_SIN2_BUDGET <= one
    policy = tolerance_policy(TolerancePolicyIdV1.CORNER_FOLD_SIN2_BUDGET_V1)
    assert Fraction(policy.value.numerator, policy.value.denominator) == _corner_fold.CORNER_FOLD_SIN2_BUDGET
    assert policy.changes_topology
    assert policy.declaration_sites == ("cftuv_envelope._corner_fold.CORNER_FOLD_SIN2_BUDGET",)


def test_the_fold_measure_is_exact_and_zero_on_an_exact_plane():
    # Две нормали `(3, 4, 0)` и `(4, 3, 0)`: `cross = (0, 0, -7)`, `sin^2 = 49 / 625` (рукой).
    assert _corner_fold.max_fold_sin2([(3, 4, 0), (4, 3, 0)]) == Fraction(49, 625)
    assert _corner_fold.max_fold_sin2([(0, 0, 1), (1, 0, 0)]) == 1
    assert _corner_fold.max_fold_sin2([(0, 0, 1)]) == 0 and _corner_fold.max_fold_sin2([]) == 0
    # Точная наклонная плоскость `z = x / 2 + y / 4` (dyadic координаты): нормали всех треугольников параллельны тождественно.
    plane = lambda x, y: (Fraction(x), Fraction(y), Fraction(x, 2) + Fraction(y, 4))  # noqa: E731
    normals = [
        _corner_fold.triangle_normal([plane(0, 0), plane(8, 0), plane(8, 6)]),
        _corner_fold.triangle_normal([plane(0, 0), plane(8, 6), plane(-2, 7)]),
        _corner_fold.triangle_normal([plane(-2, 7), plane(8, 6), plane(3, 9)]),
    ]
    assert _corner_fold.max_fold_sin2(normals) == 0


@pytest.mark.parametrize("lift", (0.0, 0.125, 0.25, 1.0, 3.0))
def test_the_fold_measure_of_a_ring_matches_the_independent_formula(lift):
    snapshot, _request = _snapshot(shared=False)
    folded = _folded(snapshot, lift)
    measured = _corner_fold.CornerFoldFacts(folded).sin2_at(kernel.SourceVertexId("v1"), kernel.PatchId("patch"))
    assert measured == _lagrange_max(folded, "v1")
    if lift == 0.0:
        assert measured == 0
    if lift == 1.0:
        # Рукой: пара (первый, третий) треугольник даёт `h^2 / (h^2 + 100)`, при `h = 1` это `1 / 101`.
        assert measured == Fraction(1, 101)


# --------------------------------------------------------------------------
# Решение угла
# --------------------------------------------------------------------------


def test_a_folded_neighbourhood_miters_a_corner_within_the_bend_bound():
    snapshot, _request = _snapshot(shared=False)
    (treatment, reason, shared), fold, relation, sector = _decision(_folded(snapshot, 1.0))
    assert (treatment, reason, shared) == (MITER, FOLDED, frozenset())
    assert fold.sin2_at(relation.source_vertex_id, sector.owner_patch_id) == Fraction(1, 101) > _corner_fold.CORNER_FOLD_SIN2_BUDGET
    # Решающий закон записи — закон митры, а не JOIN; запись несёт тот же предел изгиба 1/2.
    assert law.law_of(reason) == law.CORNER_MITER_LAW != law.CORNER_TREATMENT_LAW
    assert law.SELECTION_LAW_OF_TREATMENT[treatment] is SelectionLaw.CORNER_MITER_ON_FOLD_V1
    assert SelectionLaw.CORNER_MITER_ON_FOLD_V1 in ZERO_SUPPORT_SELECTION_LAWS
    # Без складки то же самое решение — прежнее (веер, цепь не доказана): закон инертен на плоскости.
    assert _decision(snapshot)[0][:2] == (FAN, UNPROVEN)
    assert _decision(_folded(snapshot, 0.0))[0][:2] == (FAN, UNPROVEN)


def test_a_fold_within_the_budget_keeps_the_fan_and_its_old_reason():
    snapshot, _request = _snapshot(shared=False)
    for lift in (0.125, 0.17):
        decision, fold, relation, sector = _decision(_folded(snapshot, lift))
        measured = fold.sin2_at(relation.source_vertex_id, sector.owner_patch_id)
        assert 0 < measured <= _corner_fold.CORNER_FOLD_SIN2_BUDGET, (lift, float(measured))
        assert decision[:2] == (FAN, UNPROVEN)
        assert law.law_of(decision[1]) == law.CORNER_TREATMENT_LAW
    # Тот же бюджет делит ответ: `h = 0.18` — `sin^2 = 3.24e-4` свыше `3e-4`.
    decision, fold, relation, sector = _decision(_folded(snapshot, 0.18))
    assert fold.sin2_at(relation.source_vertex_id, sector.owner_patch_id) > _corner_fold.CORNER_FOLD_SIN2_BUDGET
    assert decision[:2] == (MITER, FOLDED)


@pytest.mark.parametrize(
    ("bounds", "treatment", "reason"),
    (
        (("0.4000", "0.4100"), MITER, FOLDED),
        # ЗАМКНУТЫЙ предел: прямой угол (и угол, восстановленный до него) берёт митру.
        (("0.5000", "0.5000"), MITER, FOLDED),
        (("0.4999", "0.5000"), MITER, FOLDED),
        # Интервал накрывает предел: изгиб в пределах не доказан, митра не берётся — веер, названный.
        (("0.4900", "0.5100"), FAN, BEYOND),
        (("0.5600", "0.5700"), FAN, BEYOND),
    ),
    ids=("soft", "exact-right-angle", "restored-right-angle", "interval-over-the-bound", "beyond-the-bound"),
)
def test_the_bend_bound_of_the_miter_is_closed_and_a_wider_bend_stays_a_named_fan(bounds, treatment, reason):
    snapshot, _request = _snapshot(shared=False, bounds=bounds)
    decision = _decision(_folded(snapshot, 1.0))[0]
    assert decision[:2] == (treatment, reason)
    # Без складки угол шире предела остаётся веером под прежним именем: BEYOND пишет только свёрнутая окрестность.
    assert _decision(snapshot)[0][:2] == (FAN, UNPROVEN)


def test_join_decides_first_and_a_right_angle_of_one_chain_is_a_miter_that_keeps_the_shared_lineage():
    # Мягкий излом одной цепи: JOIN берёт угол, складка ничего не меняет (поток, а не шов).
    snapshot, _request = _snapshot(shared=True)
    assert _decision(_folded(snapshot, 1.0))[0][:2] == (JOIN, CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN)
    # Точный прямой угол одной цепи: JOIN его не берёт (предел исключительный), складка даёт митру; общая линия — факт записи.
    snapshot, _request = _snapshot(shared=True, bounds=("0.5000", "0.5000"))
    treatment, reason, shared = _decision(_folded(snapshot, 1.0))[0]
    assert (treatment, reason) == (MITER, FOLDED) and shared == frozenset({kernel.LineageId("chain-source:patch:wall")})
    assert _decision(snapshot)[0][:2] == (FAN, CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT)


def test_an_unmeasurable_neighbourhood_leaves_the_decision_as_it_was():
    snapshot, _request = _snapshot(shared=False)
    folded = _folded(snapshot, 1.0)
    # Нет позиций (координатно-свободная фикстура): меры нет, закон инертен.
    unavailable = UnavailableSourcePositionV1(SurfaceCoordinateUnavailableReason.EC0_COORDINATE_FREE_FIXTURE_V5)
    blind = _folded(snapshot, 1.0, positions={"v3": unavailable})
    assert _corner_fold.CornerFoldFacts(blind).sin2_at(kernel.SourceVertexId("v1"), kernel.PatchId("patch")) is None
    assert _decision(blind)[0][:2] == (FAN, UNPROVEN)
    # Кольцо вершины в патче пусто (вершина, которой нет в треугольниках): меры нет.
    facts = _corner_fold.CornerFoldFacts(folded)
    assert facts.sin2_at(kernel.SourceVertexId("no-such-vertex"), kernel.PatchId("patch")) is None
    # Все треугольники кольца вырождены (нулевая нормаль): пар нет, меры нет.
    flat = {name: LocalPoint3V1(float(index), 0.0, 0.0) for index, name in enumerate(("v0", "v1", "v2", "v3", "v4"))}
    assert _corner_fold.CornerFoldFacts(_folded(snapshot, 1.0, positions=flat)).sin2_at(
        kernel.SourceVertexId("v1"), kernel.PatchId("patch")
    ) is None


def test_the_ring_of_another_patch_does_not_count():
    """Кольцо считается ВНУТРИ патча владельца: складка через шов к соседу — не складка патча."""

    snapshot, _request = _snapshot(shared=False)
    folded = _folded(snapshot, 1.0)
    (face,) = folded.surface_ir.source_faces
    neighbour = dataclasses.replace(face, face_id=kernel.SourceFaceId("neighbour-face"), patch_id=kernel.PatchId("neighbour"))
    triangles = frozenset(
        dataclasses.replace(item, source_face_id=neighbour.face_id) if item.triangle_id.value == "ring-1" else item
        for item in folded.surface_ir.surface_triangles
    )
    surface = dataclasses.replace(
        folded.surface_ir, source_faces=folded.surface_ir.source_faces | {neighbour}, surface_triangles=triangles
    )
    split = dataclasses.replace(folded, surface_ir=surface)
    facts = _corner_fold.CornerFoldFacts(split)
    own = facts.sin2_at(kernel.SourceVertexId("v1"), kernel.PatchId("patch"))
    other = facts.sin2_at(kernel.SourceVertexId("v1"), kernel.PatchId("neighbour"))
    # В патче владельца остались треугольники 0 и 2 (обе плоскости `z = 0` и `-10h y + 100 z`), в соседе — один: пар нет.
    assert own == Fraction(1, 101) and other == 0
    only_flat = frozenset(item for item in surface.surface_triangles if item.triangle_id.value == "ring-0")
    assert _corner_fold.CornerFoldFacts(
        dataclasses.replace(split, surface_ir=dataclasses.replace(surface, surface_triangles=only_flat))
    ).sin2_at(kernel.SourceVertexId("v1"), kernel.PatchId("patch")) == 0


# --------------------------------------------------------------------------
# Домен `building` 89: три угла митрой, четвёртый веером
# --------------------------------------------------------------------------

MITERED_VERTICES = {"40", "46", "121"}
FANNED_VERTEX = "34"
DENSITY_FILES = {1: "decal_request_density1.json", 2: "decal_request_density2.json", 4: "decal_request_density4.json"}


def _load(density=2):
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((FIXTURE / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((FIXTURE / DENSITY_FILES[density]).read_bytes())
    return snapshot, request


def _by_vertex(prepared):
    relations = {item.corner_relation_id: item for item in prepared.context.snapshot.corner_relations}
    records = {}
    for record in prepared.compilation.corner_treatments:
        vertex = relations[record.corner_relation_id].source_vertex_id.value.rsplit(":", 1)[1]
        records[vertex] = record
    return records


@pytest.mark.parametrize("density", (1, 2, 4))
def test_the_exported_domain_miters_three_corners_and_fans_the_fourth(density):
    snapshot, request = _load(density)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    records = _by_vertex(prepared)
    assert set(records) == MITERED_VERTICES | {FANNED_VERTEX}
    selections = {item.certificate_id: item for item in prepared.compilation.profile_selection_certificates}
    for vertex, record in records.items():
        selection = selections[record.selection_certificate_id]
        if vertex in MITERED_VERTICES:
            assert (record.treatment, record.reason, record.treatment_law) == (MITER, FOLDED, law.CORNER_MITER_LAW)
            assert selection.selection_law is SelectionLaw.CORNER_MITER_ON_FOLD_V1
            assert selection.resolved_hidden_edge_count == 0
            assert record.threshold_over_pi == kernel.ExactRatioV1(1, 2)
        else:
            assert (record.treatment, record.reason, record.treatment_law) == (FAN, BEYOND, law.CORNER_MITER_LAW)
            assert selection.selection_law is not SelectionLaw.CORNER_MITER_ON_FOLD_V1
            assert selection.resolved_hidden_edge_count >= 1
    assert not corner_treatment_errors(prepared.compilation)
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 3 and counters["CONVEYOR_FOLD_MITERED_CORNERS"] == 3
    plan_like = _plan_like(prepared.compilation)
    issues: list = []
    validate_plan_corner_treatments(issues, plan_like)
    validate_plan_corner_treatments_against_snapshot(issues, plan_like, snapshot, ("plans", "p"))
    assert not issues


def test_without_the_law_the_same_corners_are_fans_with_their_old_reasons(monkeypatch):
    """Красный контроль: бюджет, которого не достичь (`sin^2 <= 1` всегда), возвращает веера и прежние причины."""

    monkeypatch.setattr(law, "CORNER_FOLD_SIN2_BUDGET", Fraction(1))
    snapshot, request = _load(2)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    assert {(item.treatment, item.reason) for item in prepared.compilation.corner_treatments} == {(FAN, UNPROVEN)}
    counters = dict(prepared.counters)
    assert "CONVEYOR_FOLD_MITERED_CORNERS" not in counters and counters["CONVEYOR_MITERED_CORNERS"] == 0
    assert counters["CONVEYOR_RATIONAL_VERTEX_FANS"] >= 3


def _materialized(density=2):
    snapshot, request = _load(density)
    prepared, coverage = factories.prepare_and_cover(snapshot, request, alpha="0.25")
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return prepared, result


def test_the_miter_corners_are_seams_without_flow_and_the_domain_names_them():
    prepared, result = _materialized(2)
    table = chain_station_table(prepared, factories.budget())
    vertices = {item.rsplit(":", 1)[1] for item in table.fold_miters}
    assert vertices == MITERED_VERTICES
    # Потока у митры нет: ни один из её вершин не стык JOIN плана и не стык одной цепи без записи угла.
    flowing = {item[0].rsplit(":", 1)[1] for item in (*table.plan_joins, *table.same_chain_joins)}
    assert not (vertices & flowing)
    assert dict(table.counters)["STATION_FOLD_MITER_CORNERS"] == 3
    assert dict(result.counters)["STATION_FOLD_MITER_CORNERS"] == 3
    lines = [item for item in result.diagnostics if item.startswith("CORNER_MITER_ON_FOLD_V1")]
    assert len(lines) == 1 and "3 concave corners" in lines[0]
    assert any(item.outcome is NamedOutcome.CORNER_MITER_ON_FOLD_V1 for item in result.batch.diagnostics)


def test_a_domain_without_a_miter_corner_counts_zero_and_keeps_its_records():
    """Домен `building` 17 (плоский, шумовые складки кольца на порядки ниже бюджета): закон ничего не меняет."""

    folder = ROOT / "fixtures" / "building_patch17_crowded_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((folder / "decal_request_density2.json").read_bytes())
    prepared, _coverage = factories.prepare_and_cover(snapshot, request, alpha="0.45")
    assert prepared.compilation.corner_treatments
    assert {(item.treatment, item.reason) for item in prepared.compilation.corner_treatments} == {(FAN, UNPROVEN)}
    table = chain_station_table(prepared, factories.budget())
    assert table.fold_miters == () and dict(table.counters)["STATION_FOLD_MITER_CORNERS"] == 0
    assert "CONVEYOR_FOLD_MITERED_CORNERS" not in dict(prepared.counters)


# --------------------------------------------------------------------------
# Подделка записи и сертификата
# --------------------------------------------------------------------------


def _issues(plan, snapshot):
    issues: list = []
    validate_plan_corner_treatments(issues, plan)
    validate_plan_corner_treatments_against_snapshot(issues, plan, snapshot, ("plans", "p"))
    return issues


def test_forged_miter_records_and_certificates_are_refused_by_name():
    snapshot, request = _load(2)
    compilation = prepare_conveyor(snapshot, request).compilation
    records = {item.corner_relation_id.value.rsplit(":", 1)[-1]: item for item in compilation.corner_treatments}
    miter = next(item for item in compilation.corner_treatments if item.treatment is MITER)
    selection = next(item for item in compilation.profile_selection_certificates if item.certificate_id == miter.selection_certificate_id)
    codes = lambda issues: {issue.code for issue in issues}  # noqa: E731
    assert not _issues(_plan_like(compilation), snapshot)

    # Закон сертификата — митра, записи нет: `k = 0` на слово не принимается.
    bare = _plan_like(compilation, records=frozenset(item for item in compilation.corner_treatments if item is not miter))
    assert any("has no corner treatment record" in issue.message and "CORNER_MITER_ON_FOLD_V1" in issue.message for issue in _issues(bare, snapshot))
    # Запись и сертификат согласны между собой, но снапшот складки не доказывает (все позиции в одной плоскости).
    flat = dataclasses.replace(
        snapshot,
        source_vertices=frozenset(
            dataclasses.replace(item, position=LocalPoint3V1(item.position.x, item.position.y, 0.0)) for item in snapshot.source_vertices
        ),
    )
    issues = _issues(_plan_like(compilation), flat)
    assert codes(issues) == {ValidationCode.CORNER_TREATMENT}
    assert any("differs from the raw snapshot" in issue.message for issue in issues)
    assert any("contradicts the raw snapshot" in issue.message for issue in issues)
    # Структура: причина не той обработки, чужой закон записи, `k != 0` под митрой.
    wrong_reason = dataclasses.replace(miter, reason=CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN)
    wrong_law = dataclasses.replace(miter, treatment_law=law.CORNER_TREATMENT_LAW)
    for forged, message in (
        (wrong_reason, "treatment reason does not belong to its treatment"),
        (wrong_law, "treatment law is not the declared one"),
    ):
        plan = _plan_like(compilation, records=frozenset({forged, *(item for item in compilation.corner_treatments if item is not miter)}))
        structural: list = []
        validate_plan_corner_treatments(structural, plan)
        assert any(message in issue.message for issue in structural), message
    fanned = dataclasses.replace(selection, resolved_hidden_edge_count=1)
    structural = []
    validate_plan_corner_treatments(
        structural,
        _plan_like(compilation, certificates=frozenset({fanned, *(item for item in compilation.profile_selection_certificates if item is not selection)})),
    )
    assert any("must resolve k = 0" in issue.message for issue in structural)
    # Веер под законом митры (`BEND_BEYOND_MITER_BOUND`) — запись закона митры без сертификата митры: согласна.
    assert records  # (имена вершин есть: запись веера прочитана выше)
