"""Ворота приёмки near-planar метрики: валидатор обязан принимать то, что
пишет собственный строитель, и отвергать всё остальное.

Дыра, которую закрывает этот файл, полевая. Наклонный плоский домен владельца
(`building.002`) строится: отклонение от собственной плоскости 4.3e-07, бюджет
проходит, метрика собрана. И тот же домен отвергается на валидации снапшота
двумя претензиями сразу:

* «metric requires an exact same-domain source-plane certificate» —
  `validate_rational_affine_planar_metric` знала ровно один закон допуска
  (`EXACT_SOURCE_PLANE_V1`) и не знала near-planar ветку вовсе, хотя сертификат
  `NearPlanarProjectionCertificateV1` пишет ТОТ ЖЕ строитель;
* «exact affine reconstruction disagrees with the source position the declared
  grid law produces» — снапшот-ветка сверяла `origin + u·A + v·B` с УЗЛОМ
  решётки, а метрика для спроецированных вершин построена от узла,
  СПРОЕЦИРОВАННОГО на сертифицированную плоскость.

Класс отказа общий: строитель и его проверяющий разошлись в законе. Поэтому
первый тест здесь — не частный случай, а само правило: всё, что строитель
выпускает в окне допуска, валидатор обязан принять без единого issue.

Опора проекции у валидатора та же, что у строителя, и новых полей контракту для
этого не нужно: опора плоскости — позиция первой вершины кадра, она никогда не
отклоняется от себя, поэтому не проецируется и совпадает с `exact_origin`
метрики; формула `p − ((p−o)·n/(n·n))·n` инвариантна к масштабу нормали,
поэтому пара (`exact_origin`, `exact_plane_normal`) задаёт плоскость полностью.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope import (
    AnalysisCapability,
    AnalysisSnapshotV1,
    ANALYSIS_SNAPSHOT_SCHEMA_V1,
    BoundaryConstraintId,
    BoundaryConstraintKind,
    BoundaryConstraintV1,
    BoundaryLoopId,
    BoundaryLoopKind,
    BoundaryLoopV1,
    ChainUseConstraintTargetV1,
    ChainUseId,
    ChainUseOrientation,
    ChainUseRole,
    ChainUseV1,
    ExactPoint2V1,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    ExactSourceVertexCoordinateV2,
    GridSnappingLawV1,
    LaunchLocusV1,
    LawId,
    LineageId,
    LocalPoint3V1,
    LocalVector3V1,
    OwnerInteriorDirection,
    OwnerSectorId,
    PatchContextTag,
    PatchDescriptorV1,
    PatchDomainId,
    PatchDomainV1,
    PatchId,
    PatchSectorV1,
    PatchShapeClass,
    PatchSurfaceIRV1,
    PhysicalChainId,
    PhysicalChainKind,
    PhysicalChainV1,
    PhysicalEdgeId,
    PlanarityAdmissionLawV1,
    PlanarityCertificateId,
    SourceEdgeV1,
    SourceFaceId,
    SourceFaceV1,
    SourceRevision,
    SourceVertexId,
    SourceVertexV1,
    SurfacePayloadMode,
    SurfaceRegime,
    SurfaceTriangleId,
    SurfaceTriangleV1,
    TerminalEndpointRole,
    TerminalRelationId,
    TerminalRelationKind,
    TerminalRelationV1,
    build_rational_affine_planar_metric,
    validate_analysis_snapshot,
    validate_rational_affine_planar_metric,
)

# Оба имени не входят в `__all__` пакета намеренно — публичный API ими не
# расширяется, поэтому берутся оттуда, где живут, как это уже делает
# `test_near_planar_policy.py`.
from cftuv_envelope.contracts.metric import (
    NearPlanarProjectionCertificateV1,
    NearPlanarResidualBudgetLawV1,
)


NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
SOURCE_SNAP = GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
UNSNAPPED = GridSnappingLawV1.UNSNAPPED_EXACT_V1

REVISION = SourceRevision("near-planar-revision")
PATCH = PatchId("patch")
DOMAIN = PatchDomainId("domain")

SLOPE = -0.4525 / 0.8918
"""Наклон полевого ската `building.002`: нормаль (0, −0.4525, −0.8918)."""

WIDTH = 3.3125
"""Габарит полевого патча по оси выдавливания."""

ROWS = (0.0, 0.9137, 1.8271)
"""Ряды полевого ската. Три — минимум, на котором домен ПЕРЕЖИВАЕТ снап.

Ловушка, найденная в поле: при мелкой ячейке отклонение меньше полуячейки
садится В плоскость точно, сертификат near-planar не пишется вовсе, и тест
проверял бы EXACT-ветку под именем near-planar. Наклонная плоскость этого не
допускает — она распределяет ошибку снапа по осям. Замер на этой форме: после
снапа невязка 1.26e-05 при ячейке 6.10e-05 (пережила снап, в бюджете); без
снапа 3.60e-18 при бюджете представления 3.31e-07.
"""


VERTEX_IDS = tuple(SourceVertexId(f"v{index:02d}") for index in range(6))
EDGE_IDS = tuple(PhysicalEdgeId(f"e{index}") for index in range(7))
FACE_IDS = (SourceFaceId("f0"), SourceFaceId("f1"))
TRIANGLE_IDS = tuple(SurfaceTriangleId(f"t{index}") for index in range(4))

# v00 v01 v02 — ближний ряд по y, v03 v04 v05 — дальний, выдавленный по x.
_V00, _V01, _V02, _V03, _V04, _V05 = VERTEX_IDS
_EA, _EB, _EC, _ED, _EE, _EF, _EG = EDGE_IDS

_EDGE_PAIRS = {
    _EA: (_V00, _V01),
    _EB: (_V01, _V04),
    _EC: (_V03, _V04),
    _ED: (_V00, _V03),
    _EE: (_V01, _V02),
    _EF: (_V02, _V05),
    _EG: (_V04, _V05),
}
_FACE_CYCLES = {
    FACE_IDS[0]: ((_V00, _V01, _V04, _V03), (_EA, _EB, _EC, _ED)),
    FACE_IDS[1]: ((_V01, _V02, _V05, _V04), (_EE, _EF, _EG, _EB)),
}
_TRIANGLES = {
    TRIANGLE_IDS[0]: (FACE_IDS[0], (_V00, _V01, _V04), (_EA, _EB, None)),
    TRIANGLE_IDS[1]: (FACE_IDS[0], (_V00, _V04, _V03), (None, _EC, _ED)),
    TRIANGLE_IDS[2]: (FACE_IDS[1], (_V01, _V02, _V05), (_EE, _EF, None)),
    TRIANGLE_IDS[3]: (FACE_IDS[1], (_V01, _V05, _V04), (None, _EG, _EB)),
}

_NORMAL = LocalVector3V1(0.0, 1.0, 2.0)


def _slope_positions() -> dict[SourceVertexId, LocalPoint3V1]:
    positions = {}
    for column, x in enumerate((0.0, WIDTH)):
        for row, y in enumerate(ROWS):
            index = column * len(ROWS) + row
            positions[VERTEX_IDS[index]] = LocalPoint3V1(x, y, SLOPE * y)
    return positions


def _source_faces() -> frozenset[SourceFaceV1]:
    triangles_of = {
        face_id: tuple(
            sorted(
                (
                    triangle_id
                    for triangle_id, item in _TRIANGLES.items()
                    if item[0] == face_id
                ),
                key=lambda item: item.value,
            )
        )
        for face_id in FACE_IDS
    }
    return frozenset(
        SourceFaceV1(
            face_id=face_id,
            patch_id=PATCH,
            vertex_cycle=cycle,
            edge_cycle=edges,
            polygon_normal=_NORMAL,
            triangle_ids=triangles_of[face_id],
        )
        for face_id, (cycle, edges) in _FACE_CYCLES.items()
    )


def _surface_ir(positions) -> PatchSurfaceIRV1:
    return PatchSurfaceIRV1(
        schema_version="cftuv.envelope.patch_surface_ir.v1",
        source_revision=REVISION,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(positions),
        source_edges=frozenset(
            SourceEdgeV1(edge_id, *pair) for edge_id, pair in _EDGE_PAIRS.items()
        ),
        source_faces=_source_faces(),
        surface_triangles=frozenset(
            SurfaceTriangleV1(
                triangle_id=triangle_id,
                source_face_id=face_id,
                vertex_ids=vertex_ids,
                physical_edge_ids=edge_ids,
                triangle_normal=_NORMAL,
            )
            for triangle_id, (face_id, vertex_ids, edge_ids) in _TRIANGLES.items()
        ),
    )


def _skeleton(positions, surface_ir, descriptor) -> AnalysisSnapshotV1:
    """Минимальный полный снапшот вокруг одного PLANAR-домена."""

    loop_id = BoundaryLoopId("outer-loop")
    chain_id = PhysicalChainId("chain")
    use_id = ChainUseId("chain-use")
    constraint_id = BoundaryConstraintId("constraint")
    sector_id = OwnerSectorId("owner-sector")
    constraint = BoundaryConstraintV1(
        boundary_constraint_id=constraint_id,
        patch_domain_id=DOMAIN,
        constraint_kind=BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
        target=ChainUseConstraintTargetV1(use_id),
        originating_chain_use_id=use_id,
        blocks_originating_seed=False,
        blocks_other_fronts=True,
    )
    chain_use = ChainUseV1(
        chain_use_id=use_id,
        physical_chain_id=chain_id,
        owner_patch_id=PATCH,
        patch_domain_id=DOMAIN,
        boundary_loop_id=loop_id,
        orientation=ChainUseOrientation.SEMANTIC_START_TO_END,
        roles=frozenset({ChainUseRole.DECAL_SOURCE}),
        launch_locus=LaunchLocusV1(
            boundary_constraint_id=constraint_id,
            direction=OwnerInteriorDirection.OWNER_INTERIOR,
            source_support_non_blocking=True,
        ),
    )
    return AnalysisSnapshotV1(
        schema_version=ANALYSIS_SNAPSHOT_SCHEMA_V1,
        source_revision=REVISION,
        analysis_capabilities=frozenset(
            {
                AnalysisCapability.PATCH_DOMAIN_V1,
                AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
                AnalysisCapability.PATCH_SURFACE_IR_V1,
                AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
                AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
                AnalysisCapability.RELATION_LINEAGE_V1,
                AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
                AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
                AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
                AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
                AnalysisCapability.TERMINAL_RELATION_V1,
            }
        ),
        patches=frozenset(
            {
                PatchDescriptorV1(
                    patch_id=PATCH,
                    surface_regime=SurfaceRegime.PLANAR,
                    shape_class=PatchShapeClass.MIX,
                    context_tags=frozenset({PatchContextTag.WALL}),
                )
            }
        ),
        patch_domains=frozenset(
            {
                PatchDomainV1(
                    patch_domain_id=DOMAIN,
                    owner_patch_id=PATCH,
                    surface_regime=SurfaceRegime.PLANAR,
                    boundary_constraint_ids=frozenset({constraint_id}),
                    sectors=frozenset(
                        {
                            PatchSectorV1(
                                owner_sector_id=sector_id,
                                chain_use_id=use_id,
                                analysis_proven=True,
                                multiplicity_reason=LawId("single-owner-interior-front"),
                            )
                        }
                    ),
                )
            }
        ),
        surface_metric_descriptors=frozenset({descriptor}),
        surface_ir=surface_ir,
        source_vertices=frozenset(
            SourceVertexV1(vertex_id, point)
            for vertex_id, point in positions.items()
        ),
        physical_chains=frozenset(
            {
                PhysicalChainV1(
                    physical_chain_id=chain_id,
                    kind=PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
                    is_closed=False,
                    ordered_source_vertex_ids=(_V00, _V01),
                    ordered_physical_edge_ids=(_EA,),
                    source_lineage=frozenset({LineageId("source-chain")}),
                    data_record_lineage=frozenset(),
                )
            }
        ),
        chain_uses=frozenset({chain_use}),
        boundary_loops=frozenset(
            {
                BoundaryLoopV1(
                    boundary_loop_id=loop_id,
                    patch_domain_id=DOMAIN,
                    kind=BoundaryLoopKind.OUTER,
                    ordered_chain_use_ids=(use_id,),
                )
            }
        ),
        boundary_constraints=frozenset({constraint}),
        angular_owner_sectors=frozenset(),
        reflex_angle_certificates=frozenset(),
        corner_relations=frozenset(),
        junction_relations=frozenset(),
        terminal_relations=frozenset(
            {
                TerminalRelationV1(
                    terminal_relation_id=TerminalRelationId("terminal-start"),
                    source_vertex_id=_V00,
                    chain_use_id=use_id,
                    owner_patch_id=PATCH,
                    patch_domain_id=DOMAIN,
                    relation_kind=TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                    endpoint_role=TerminalEndpointRole.START,
                    owner_sector_id=sector_id,
                    source_lineage=frozenset({LineageId("source-chain")}),
                )
            }
        ),
    )


def near_planar_metric(grid_law: GridSnappingLawV1):
    """Метрика полевого ската — ровно то, что выпускает собственный строитель."""

    positions = _slope_positions()
    surface_ir = _surface_ir(positions)
    return build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=[
            SourceVertexV1(vertex_id, point)
            for vertex_id, point in positions.items()
        ],
        source_faces=surface_ir.source_faces,
        planarity_policy=NEAR,
        grid_policy=grid_law,
    )


def near_planar_snapshot(grid_law: GridSnappingLawV1, descriptor=None):
    positions = _slope_positions()
    surface_ir = _surface_ir(positions)
    metric = descriptor if descriptor is not None else near_planar_metric(grid_law)
    return _skeleton(positions, surface_ir, metric), metric


def _fraction(value: ExactRationalV1) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _rational(value: Fraction) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _with_certificate(metric, **changes):
    return replace(
        metric,
        planarity_certificate=replace(metric.planarity_certificate, **changes),
    )


# --------------------------------------------------------------------------
# T7 / R1. Валидатор принимает всё, что строит собственный строитель.
# --------------------------------------------------------------------------


@pytest.mark.parametrize("grid_law", [UNSNAPPED, SOURCE_SNAP])
def test_the_validator_admits_what_its_own_builder_writes(grid_law):
    """R1. Допущенный near-planar домен валиден — и как метрика, и в снапшоте.

    Это репродукция полевого отказа: до починки первая же проверка находит
    «metric requires an exact same-domain source-plane certificate», а вторая —
    расхождение реконструкции с узлом решётки на спроецированных вершинах.

    Оба закона решётки проверяются одной формой намеренно. Снапнутый —
    полевой default, и именно он даёт непустой перечень спроецированных вершин
    ПОСЛЕ привязки; неснапнутый — прежний путь, где та же ветка обязана вести
    себя так же.
    """

    snapshot, metric = near_planar_snapshot(grid_law)
    certificate = metric.planarity_certificate

    # Синтетика обязана целить в окно «пережил снап, но в бюджете»: иначе
    # проверялась бы EXACT-ветка под именем near-planar. Тип — по проводу, тем
    # же `type() is`, каким его читает валидатор.
    assert type(certificate) is NearPlanarProjectionCertificateV1
    assert certificate.admission_law is NEAR
    assert certificate.exact is False
    assert certificate.projected_source_vertex_ids

    assert validate_rational_affine_planar_metric(metric) == ()
    assert validate_analysis_snapshot(snapshot) == ()


# --------------------------------------------------------------------------
# T8. Красные контроли.
# --------------------------------------------------------------------------


def test_a_forged_coordinate_is_refused():
    """R2. Одна подделанная координата карты — реконструкция мимо плоскости."""

    snapshot, metric = near_planar_snapshot(SOURCE_SNAP)
    victim = min(
        metric.exact_source_vertex_coordinates,
        key=lambda item: item.source_vertex_id.value,
    )
    forged = ExactSourceVertexCoordinateV2(
        source_vertex_id=victim.source_vertex_id,
        domain_coordinate=ExactPoint2V1(
            _rational(_fraction(victim.domain_coordinate.x) + 1),
            victim.domain_coordinate.y,
        ),
    )
    broken = replace(
        metric,
        exact_source_vertex_coordinates=frozenset(
            {forged}
            | {
                item
                for item in metric.exact_source_vertex_coordinates
                if item.source_vertex_id != victim.source_vertex_id
            }
        ),
    )
    snapshot, _ = near_planar_snapshot(SOURCE_SNAP, descriptor=broken)
    issues = validate_analysis_snapshot(snapshot)
    assert any(
        "exact affine reconstruction disagrees" in item.message for item in issues
    ), issues


def test_a_certificate_naming_another_domain_is_refused():
    """R3. Сертификат чужого домена — не сертификат этой метрики."""

    metric = near_planar_metric(SOURCE_SNAP)
    broken = _with_certificate(metric, patch_domain_id=PatchDomainId("other-domain"))
    assert validate_rational_affine_planar_metric(broken) != ()


def test_an_exact_certificate_over_projected_coordinates_is_refused():
    """R5. EXACT-сертификат при непланарных координатах — INVALID.

    Нынешнее поведение, замороженное явно: координаты карты построены от
    СПРОЕЦИРОВАННЫХ позиций, а `EXACT_SOURCE_PLANE_V1` объявляет, что источник
    лежит в плоскости точно и никуда не двигался. Ожидание реконструкции для
    такого сертификата — узел объявленного закона решётки, и оно обязано
    разойтись с картой. Если бы эта проверка ослабла вместе с введением
    near-planar ветки, объявить «точно» стало бы бесплатно.
    """

    metric = near_planar_metric(SOURCE_SNAP)
    certificate = metric.planarity_certificate
    forged = ExactSourcePlaneCertificateV1(
        certificate_id=PlanarityCertificateId("forged-exact"),
        patch_domain_id=certificate.patch_domain_id,
        admission_law=EXACT,
        exact=True,
        exact_plane_normal=certificate.exact_plane_normal,
        source_vertex_ids=certificate.source_vertex_ids,
        reconstruction_law=certificate.reconstruction_law,
    )
    snapshot, _ = near_planar_snapshot(
        SOURCE_SNAP, descriptor=replace(metric, planarity_certificate=forged)
    )
    issues = validate_analysis_snapshot(snapshot)
    assert any(
        "exact affine reconstruction disagrees" in item.message for item in issues
    ), issues


def test_a_recorded_residual_above_the_recorded_budget_is_refused():
    """R4. `max_residual_squared > budget²` — сертификат противоречит себе.

    Величина, объявленная допущенной, обязана быть допущенной по объявленному
    же бюджету. Без этой проверки бюджет был бы полем, которое никто не читает:
    строитель сравнил у себя, записал число, и дальше сравнения нет.
    """

    metric = near_planar_metric(SOURCE_SNAP)
    certificate = metric.planarity_certificate
    budget = _fraction(certificate.residual_budget)
    broken = _with_certificate(
        metric,
        max_residual_squared=_rational(budget * budget * 2),
    )
    assert validate_rational_affine_planar_metric(broken) != ()


@pytest.mark.parametrize(
    "grid_law,forged_law",
    [
        (SOURCE_SNAP, NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1),
        (UNSNAPPED, NearPlanarResidualBudgetLawV1.GRID_STEP_CELL_V1),
    ],
    ids=["snapped-claims-representation", "unsnapped-claims-cell"],
)
def test_a_budget_law_that_does_not_reproduce_the_number_is_refused(
    grid_law, forged_law
):
    """R6. Объявленный закон бюджета обязан воспроизводить записанное число.

    Обе стороны подмены проверяются отдельно, потому что они разные: снапнутый
    домен, объявляющий представление, и неснапнутый, объявляющий ячейку. Ровно
    этим полевой дефект и был — строитель считал по ячейке, а писал закон
    представления, и расхождение (6.10e-05 против 3.31e-07) никто не видел.
    """

    metric = near_planar_metric(grid_law)
    broken = _with_certificate(metric, residual_budget_law=forged_law)
    issues = validate_rational_affine_planar_metric(broken)
    assert issues != ()
    assert any("residual" in "/".join(item.path) for item in issues), issues


def test_a_certificate_of_an_unknown_type_is_refused_by_name():
    """Третий тип на проводе — названный отказ, а не молчаливый пропуск.

    Разбор идёт точным `type() is`, поэтому «ни то ни другое» обязано иметь
    свой исход. И обязано быть отказом, а не `AttributeError` изнутри
    проверяющего: у неизвестной записи может не быть ни одного поля, которое
    он собирался прочесть.
    """

    metric = near_planar_metric(SOURCE_SNAP)

    class _ForeignCertificate:
        patch_domain_id = DOMAIN

    broken = replace(metric, planarity_certificate=_ForeignCertificate())
    issues = validate_rational_affine_planar_metric(broken)
    assert any("unknown planarity-admission" in item.message for item in issues), issues


def test_a_projected_vertex_dropped_from_the_certificate_is_refused():
    """Забыть назвать спроецированную вершину — тот же отказ, не новая проверка.

    Следствие правильного T2, а не отдельное правило: вершина, не названная в
    `projected_source_vertex_ids`, судится по узлу решётки, а её координаты
    построены от проекции этого узла. Расхождение обязано всплыть само.
    """

    metric = near_planar_metric(SOURCE_SNAP)
    certificate = metric.planarity_certificate
    projected = sorted(
        certificate.projected_source_vertex_ids, key=lambda item: item.value
    )
    assert len(projected) >= 2, projected
    broken = _with_certificate(
        metric,
        projected_source_vertex_ids=frozenset(projected[1:]),
    )
    snapshot, _ = near_planar_snapshot(SOURCE_SNAP, descriptor=broken)
    issues = validate_analysis_snapshot(snapshot)
    assert any(
        "exact affine reconstruction disagrees" in item.message for item in issues
    ), issues
