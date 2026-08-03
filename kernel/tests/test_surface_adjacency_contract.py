"""Контракт смежности S0: власть, отказы и совместимый путь.

Отрицательные контроли пронумерованы так же, как в отчёте карточки, чтобы
запись решения и исполняемая проверка ссылались на одно и то же имя.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.canonical import snapshot_digest
from cftuv_envelope.contracts.surface_adjacency import (
    SURFACE_ADJACENCY_IR_SCHEMA_V1,
    SURFACE_COMPLETE_IR_SCHEMA_V1,
    VERTEX_FAN_ORDER_LAW_V1,
    SideBoundaryReasonV1,
    SideBoundaryV1,
    SurfaceAdjacencyContractError,
    SurfaceAdjacencyIRV1,
    SurfaceAdjacencyScopeV1,
    SurfaceCompleteIRV1,
    SurfaceOrientationLawV1,
    TriangleSideKindV1,
    TriangleSideRefV1,
    TriangleSideV1,
    VertexFanUnavailableReasonV1,
    VertexFanUnavailableV1,
    VertexFanV1,
    surface_adjacency_digest,
    validate_surface_adjacency,
    validate_surface_complete,
)
from cftuv_envelope.ids import PatchId
from cftuv_envelope.schema import json_schema_for
from cftuv_envelope.surface_adjacency_compat import (
    SNAPSHOTS_WITHOUT_ADJACENCY_TABLE,
    SurfaceAdjacencyCompatOutcome,
    SurfaceAdjacencyCompatRefused,
    compat_counters,
    read_surface_adjacency,
    reset_compat_counters,
)

from surface_adjacency_factories import (
    PATCH,
    REVISION,
    bowtie_pinch,
    edge,
    flipped_quad,
    non_manifold_fin,
    three_coincident_components,
    triangle,
    triangulated_surface,
    two_triangle_quad,
    vertex,
)


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures"

FIELD_SNAPSHOTS = (
    "building_002_point_contact_v1",
    "building_002_full_selection_v1",
    "building_002_weighted_normals_v1",
    "building_patch10_density4_v1",
    "sem_clb_02_lost_domains_v1/cases/building_001_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_002_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_003_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_001_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_006_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_011_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_105_lost_resolved_v1",
)


# Единственная замороженная фикстура со старой (положительно
# отмасштабированной) нормалью плоскости: байты полевого свидетельства не
# переписываются, читатель проходит закатное окно совместимости P0-4, и
# счётчик хитов сверяется с этим списком. Новой неканоничной фикстуре
# окно не светит.
LEGACY_POSITIVE_SCALED_SNAPSHOTS = frozenset({"building_patch10_density4_v1"})


def _legacy_hits(name: str) -> int:
    return 1 if name.rsplit("/", 1)[-1] in LEGACY_POSITIVE_SCALED_SNAPSHOTS else 0


def _load(name: str):
    folder = FIXTURE_ROOT / name
    loaded = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        (folder / "analysis_snapshot.json").read_bytes()
    )
    hits = loaded.compatibility_receipt.positive_scaled_normal_compat_hits
    assert hits == _legacy_hits(name), (
        f"{name}: positive-scaled compatibility hits {hits} diverge from "
        f"the frozen allowlist expectation {_legacy_hits(name)}"
    )
    return loaded.record


def _table(surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH, **kwargs):
    return read_surface_adjacency(surface, scope=scope, **kwargs)


# --------------------------------------------------------------------------
# Положительная сторона: закон читается и проверяется
# --------------------------------------------------------------------------


def test_two_triangle_quad_glues_once_and_keeps_the_orientation_law():
    surface = two_triangle_quad()
    table = _table(surface)
    validate_surface_adjacency(table, surface)

    glued = [
        side
        for side in table.triangle_sides
        if isinstance(side.opposite, TriangleSideRefV1)
    ]
    assert len(glued) == 2
    left, right = glued
    assert {left.from_vertex_id, left.to_vertex_id} == {
        right.from_vertex_id,
        right.to_vertex_id,
    }
    # Комбинаторная ориентация: общая пара концов проходится встречно.
    assert left.from_vertex_id == right.to_vertex_id
    assert (
        table.orientation_law
        is SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
    )


def test_face_diagonal_never_carries_a_physical_edge_id():
    """Диагональ тесселяции — свойство разбиения, а не источника."""

    surface = _quad_with_diagonal()
    table = _table(surface)
    validate_surface_adjacency(table, surface)
    diagonals = [
        side
        for side in table.triangle_sides
        if side.kind is TriangleSideKindV1.FACE_DIAGONAL
    ]
    assert diagonals
    assert all(side.physical_edge_id is None for side in diagonals)


def _quad_with_diagonal():
    """Один четырёхугольник, разрезанный диагональю: два треугольника."""

    from cftuv_envelope.contracts.surface import (
        PatchSurfaceIRV1,
        SourceEdgeV1,
        SourceFaceV1,
        SurfacePayloadMode,
        SurfaceTriangleV1,
    )
    from cftuv_envelope.numeric import LocalVector3V1

    normal = LocalVector3V1(0.0, 0.0, 1.0)
    corners = (0, 1, 2, 3)
    ring = {frozenset((corners[i], corners[(i + 1) % 4])): i for i in range(4)}
    edges = frozenset(
        SourceEdgeV1(
            edge_id=edge(index),
            vertex_a_id=vertex(min(pair)),
            vertex_b_id=vertex(max(pair)),
        )
        for pair, index in ring.items()
    )
    triangles = (
        SurfaceTriangleV1(
            triangle_id=triangle(0),
            source_face_id=_face(0),
            vertex_ids=(vertex(0), vertex(1), vertex(2)),
            physical_edge_ids=(edge(0), edge(1), None),
            triangle_normal=normal,
        ),
        SurfaceTriangleV1(
            triangle_id=triangle(1),
            source_face_id=_face(0),
            vertex_ids=(vertex(0), vertex(2), vertex(3)),
            physical_edge_ids=(None, edge(2), edge(3)),
            triangle_normal=normal,
        ),
    )
    source_face = SourceFaceV1(
        face_id=_face(0),
        patch_id=PATCH,
        vertex_cycle=tuple(vertex(value) for value in corners),
        edge_cycle=tuple(edge(ring[frozenset((corners[i], corners[(i + 1) % 4]))]) for i in range(4)),
        polygon_normal=normal,
        triangle_ids=(triangle(0), triangle(1)),
    )
    return PatchSurfaceIRV1(
        schema_version="cftuv.envelope.patch_surface_ir.v1",
        source_revision=REVISION,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(vertex(value) for value in corners),
        source_edges=edges,
        source_faces=frozenset({source_face}),
        surface_triangles=frozenset(triangles),
    )


def _face(index):
    from cftuv_envelope.ids import SourceFaceId

    return SourceFaceId(f"synthetic:face:{index:04d}")


def test_complete_ir_binds_the_stored_digest_to_the_table():
    surface = two_triangle_quad()
    table = _table(surface)
    complete = SurfaceCompleteIRV1(
        schema_version=SURFACE_COMPLETE_IR_SCHEMA_V1,
        surface_ir=surface,
        adjacency=table,
        adjacency_digest=surface_adjacency_digest(table),
    )
    validate_surface_complete(complete)

    forged = SurfaceCompleteIRV1(
        schema_version=SURFACE_COMPLETE_IR_SCHEMA_V1,
        surface_ir=surface,
        adjacency=table,
        adjacency_digest=surface_adjacency_digest(_table(_quad_with_diagonal())),
    )
    with pytest.raises(SurfaceAdjacencyContractError) as caught:
        validate_surface_complete(forged)
    assert caught.value.invariant == "ADJACENCY_DIGEST"


# --------------------------------------------------------------------------
# Отрицательные контроли
# --------------------------------------------------------------------------


def test_n2_synthetic_physical_edge_id_on_a_diagonal_is_refused():
    """N-2: заражение нормативного тождества синтетикой."""

    with pytest.raises(ValueError, match="FACE_DIAGONAL"):
        TriangleSideV1(
            side=TriangleSideRefV1(triangle_id=triangle(0), side_ordinal=0),
            from_vertex_id=vertex(0),
            to_vertex_id=vertex(1),
            kind=TriangleSideKindV1.FACE_DIAGONAL,
            physical_edge_id=edge(0),
            opposite=SideBoundaryV1(reason=SideBoundaryReasonV1.MESH_BOUNDARY),
        )


def test_n3_source_edge_without_a_named_physical_edge_is_refused():
    """N-3: обратная сторона того же тождества."""

    with pytest.raises(ValueError, match="SOURCE_EDGE"):
        TriangleSideV1(
            side=TriangleSideRefV1(triangle_id=triangle(0), side_ordinal=0),
            from_vertex_id=vertex(0),
            to_vertex_id=vertex(1),
            kind=TriangleSideKindV1.SOURCE_EDGE,
            physical_edge_id=None,
            opposite=SideBoundaryV1(reason=SideBoundaryReasonV1.MESH_BOUNDARY),
        )


def test_n4_patch_scoped_payload_declared_as_a_full_mesh_is_refused():
    """N-4, главный: срез, объявленный полным мешем."""

    surface = two_triangle_quad()
    scoped = _table(
        surface,
        scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
    )
    outside = frozenset(
        TriangleSideV1(
            side=side.side,
            from_vertex_id=side.from_vertex_id,
            to_vertex_id=side.to_vertex_id,
            kind=side.kind,
            physical_edge_id=side.physical_edge_id,
            opposite=(
                SideBoundaryV1(
                    reason=SideBoundaryReasonV1.OUTSIDE_REQUESTED_SCOPE
                )
                if isinstance(side.opposite, SideBoundaryV1)
                else side.opposite
            ),
        )
        for side in scoped.triangle_sides
    )
    with pytest.raises(ValueError, match="OUTSIDE_REQUESTED_SCOPE"):
        SurfaceAdjacencyIRV1(
            schema_version=SURFACE_ADJACENCY_IR_SCHEMA_V1,
            source_revision=REVISION,
            scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
            scope_patch_ids=frozenset(),
            orientation_law=(
                SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
            ),
            triangle_sides=outside,
            vertex_fans=frozenset(),
        )


def test_n4b_full_mesh_scope_may_not_enumerate_patches():
    with pytest.raises(ValueError, match="FULL_SOURCE_MESH"):
        SurfaceAdjacencyIRV1(
            schema_version=SURFACE_ADJACENCY_IR_SCHEMA_V1,
            source_revision=REVISION,
            scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
            scope_patch_ids=frozenset({PatchId("synthetic:patch")}),
            orientation_law=(
                SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
            ),
            triangle_sides=frozenset(),
            vertex_fans=frozenset(),
        )


def test_n5_inconsistent_orientation_is_a_named_compat_refusal():
    """N-5: binary64-нормаль здесь не участвует — расходятся ЦИКЛЫ."""

    reset_compat_counters()
    with pytest.raises(SurfaceAdjacencyCompatRefused) as caught:
        _table(flipped_quad())
    assert caught.value.outcome is (
        SurfaceAdjacencyCompatOutcome.COMPAT_ORIENTATION_NOT_CONSISTENT_WITH_SOURCE_FACE_CYCLE
    )


def test_n6_broken_opposite_involution_is_refused():
    """N-6: односторонняя склейка не бывает склейкой."""

    surface = two_triangle_quad()
    table = _table(surface)
    broken = frozenset(
        TriangleSideV1(
            side=side.side,
            from_vertex_id=side.from_vertex_id,
            to_vertex_id=side.to_vertex_id,
            kind=side.kind,
            physical_edge_id=side.physical_edge_id,
            opposite=(
                SideBoundaryV1(reason=SideBoundaryReasonV1.MESH_BOUNDARY)
                if isinstance(side.opposite, TriangleSideRefV1)
                and side.side.triangle_id == triangle(0)
                else side.opposite
            ),
        )
        for side in table.triangle_sides
    )
    half_glued = SurfaceAdjacencyIRV1(
        schema_version=SURFACE_ADJACENCY_IR_SCHEMA_V1,
        source_revision=REVISION,
        scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
        scope_patch_ids=frozenset(),
        orientation_law=(
            SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
        ),
        triangle_sides=broken,
        vertex_fans=frozenset(),
    )
    with pytest.raises(SurfaceAdjacencyContractError) as caught:
        validate_surface_adjacency(half_glued, surface)
    assert caught.value.invariant == "OPPOSITE_INVOLUTION"


def test_n10_three_coincident_components_are_not_glued_by_endpoints():
    """N-10: совпадение координат склейкой не является."""

    surface = three_coincident_components()
    table = _table(surface)
    validate_surface_adjacency(table, surface)

    assert not [
        side
        for side in table.triangle_sides
        if isinstance(side.opposite, TriangleSideRefV1)
    ]
    assert len(table.triangle_sides) == 9
    assert all(
        side.opposite.reason is SideBoundaryReasonV1.MESH_BOUNDARY
        for side in table.triangle_sides
    )
    assert len(table.vertex_fans) == 9
    assert all(isinstance(fan, VertexFanV1) for fan in table.vertex_fans)


def test_a_pinched_vertex_is_named_non_manifold_not_a_mesh_boundary():
    """Веер, распавшийся при одних лишь краевых сторонах, — защемление."""

    surface = bowtie_pinch()
    table = _table(surface)
    validate_surface_adjacency(table, surface)
    unavailable = {
        fan.vertex_id: fan.reason
        for fan in table.vertex_fans
        if isinstance(fan, VertexFanUnavailableV1)
    }
    assert unavailable == {
        vertex(0): VertexFanUnavailableReasonV1.NON_MANIFOLD_FAN
    }
    assert all(
        isinstance(side.opposite, SideBoundaryV1)
        and side.opposite.reason is SideBoundaryReasonV1.MESH_BOUNDARY
        for side in table.triangle_sides
    )


def test_vertex_fan_order_is_the_declared_canonical_one():
    surface = two_triangle_quad()
    table = _table(surface)
    fans = {fan.vertex_id: fan for fan in table.vertex_fans}
    shared = fans[vertex(0)]
    assert isinstance(shared, VertexFanV1)
    assert len(shared.ordered_triangle_ids) == 2

    reversed_fan = VertexFanV1(
        vertex_id=shared.vertex_id,
        ordered_triangle_ids=tuple(reversed(shared.ordered_triangle_ids)),
        is_closed=shared.is_closed,
    )
    forged = SurfaceAdjacencyIRV1(
        schema_version=SURFACE_ADJACENCY_IR_SCHEMA_V1,
        source_revision=REVISION,
        scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
        scope_patch_ids=frozenset(),
        orientation_law=(
            SurfaceOrientationLawV1.CONSISTENT_WITH_SOURCE_FACE_CYCLE_V1
        ),
        triangle_sides=table.triangle_sides,
        vertex_fans=frozenset(
            reversed_fan if fan.vertex_id == vertex(0) else fan
            for fan in table.vertex_fans
        ),
    )
    with pytest.raises(SurfaceAdjacencyContractError) as caught:
        validate_surface_adjacency(forged, surface)
    assert caught.value.invariant == "VERTEX_FAN_ORDER"
    assert VERTEX_FAN_ORDER_LAW_V1 in caught.value.details


# --------------------------------------------------------------------------
# Совместимый путь: три конфигурации и счётчики
# --------------------------------------------------------------------------


def test_compat_reader_fails_closed_on_the_three_ambiguous_configurations():
    reset_compat_counters()
    outcomes = []
    for surface, scope in (
        (
            two_triangle_quad(),
            SurfaceAdjacencyScopeV1.REQUEST_SCOPED_PATCH_SET,
        ),
        (non_manifold_fin(), SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH),
        (
            triangulated_surface([(0, 1, 2), (0, 2, 3)], repeat_vertex_in_cycle=True),
            SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
        ),
    ):
        with pytest.raises(SurfaceAdjacencyCompatRefused) as caught:
            read_surface_adjacency(
                surface,
                scope=scope,
                scope_patch_ids=(
                    frozenset({PATCH})
                    if scope is SurfaceAdjacencyScopeV1.REQUEST_SCOPED_PATCH_SET
                    else frozenset()
                ),
            )
        outcomes.append(caught.value.outcome)

    assert outcomes == [
        SurfaceAdjacencyCompatOutcome.COMPAT_PATCH_SCOPED_BOUNDARY_UNDECIDABLE,
        SurfaceAdjacencyCompatOutcome.COMPAT_NON_MANIFOLD_TRIANGLE_SIDE,
        SurfaceAdjacencyCompatOutcome.COMPAT_REPEATED_VERTEX_IN_SOURCE_FACE_CYCLE,
    ]
    counters = compat_counters()
    assert counters["reads"] == 3
    assert counters["accepted"] == 0
    assert counters["refused"] == 3
    assert set(counters["refusals_by_outcome"]) == {
        outcome.value for outcome in outcomes
    }


@pytest.mark.parametrize("name", FIELD_SNAPSHOTS)
def test_compat_reader_reads_every_field_snapshot_as_a_full_mesh(name: str):
    """Одиннадцать полевых снапшотов выводятся, и вывод проходит валидатор."""

    snapshot = _load(name)
    table = read_surface_adjacency(
        snapshot.surface_ir, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    validate_surface_adjacency(table, snapshot.surface_ir)
    assert table.source_revision == snapshot.surface_ir.source_revision
    assert not [
        fan for fan in table.vertex_fans if isinstance(fan, VertexFanUnavailableV1)
    ]


@pytest.mark.parametrize("name", FIELD_SNAPSHOTS)
def test_compat_reader_refuses_every_field_snapshot_as_a_patch_scoped_slice(name):
    """Тот же payload под честным скоупом — отказ, а не другая таблица."""

    snapshot = _load(name)
    with pytest.raises(SurfaceAdjacencyCompatRefused) as caught:
        read_surface_adjacency(
            snapshot.surface_ir,
            scope=SurfaceAdjacencyScopeV1.REQUEST_SCOPED_PATCH_SET,
            scope_patch_ids=frozenset(
                item.patch_id for item in snapshot.patches
            ),
        )
    assert caught.value.outcome is (
        SurfaceAdjacencyCompatOutcome.COMPAT_PATCH_SCOPED_BOUNDARY_UNDECIDABLE
    )


def test_sunset_ratchet_counts_the_snapshots_that_still_need_the_compat_path():
    """Храповик: число снапшотов без таблицы только вниз. При нуле — изъятие."""

    assert SNAPSHOTS_WITHOUT_ADJACENCY_TABLE == len(FIELD_SNAPSHOTS)
    without_table = sum(
        1
        for name in FIELD_SNAPSHOTS
        if not (FIXTURE_ROOT / name / "surface_adjacency.json").exists()
    )
    assert without_table <= SNAPSHOTS_WITHOUT_ADJACENCY_TABLE, (
        "фикстур без таблицы стало больше: храповик крутится только вниз"
    )


# --------------------------------------------------------------------------
# Байты снапшотов неподвижны, схемы сгенерированы из типов
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", FIELD_SNAPSHOTS)
def test_field_snapshot_bytes_and_digest_do_not_move(name: str):
    """Обёртка не трогает снапшот. Свойство проверяется, а не обещается.

    После P0-4 у свойства две ветви. Канонические фикстуры проходят кодек
    байт-в-байт, как и раньше. Единственная легаси-фикстура (см.
    LEGACY_POSITIVE_SCALED_SNAPSHOTS) канонизируется ПРИ ЧТЕНИИ по
    именованному окну совместимости — её байты на диске неприкосновенны,
    а свойство кодека становится идемпотентностью: повторное чтение
    канонической формы не тратит окно и воспроизводит те же байты.
    Чтение смежности не мутирует снапшот в обеих ветвях.
    """

    folder = FIXTURE_ROOT / name
    raw = (folder / "analysis_snapshot.json").read_bytes()
    snapshot = _load(name)
    canonical = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    if _legacy_hits(name) == 0:
        assert canonical == raw
    else:
        reread = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
            canonical
        )
        hits = reread.compatibility_receipt.positive_scaled_normal_compat_hits
        assert hits == 0, "каноническая форма не имеет права тратить окно"
        assert kernel.AnalysisSnapshotCodecV1.dumps(reread.record) == canonical
    read_surface_adjacency(
        snapshot.surface_ir, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    assert kernel.AnalysisSnapshotCodecV1.dumps(snapshot) == canonical
    assert snapshot_digest(snapshot).sha256_hex == snapshot_digest(
        _load(name)
    ).sha256_hex


def test_checked_in_surface_schemas_are_generated_from_the_contract_types():
    root = Path(__file__).resolve().parents[1] / "schema"
    from cftuv_envelope.contracts.surface_metric_v2 import (
        SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
        SurfaceMetricDescriptorV2,
    )

    expected = {
        "surface_adjacency_ir_v1.schema.json": (
            SurfaceAdjacencyIRV1,
            SURFACE_ADJACENCY_IR_SCHEMA_V1,
        ),
        "surface_complete_ir_v1.schema.json": (
            SurfaceCompleteIRV1,
            SURFACE_COMPLETE_IR_SCHEMA_V1,
        ),
        "surface_metric_descriptor_v2.schema.json": (
            SurfaceMetricDescriptorV2,
            SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
        ),
    }
    for filename, (record_type, schema_id) in expected.items():
        checked_in = json.loads((root / filename).read_text(encoding="utf-8"))
        assert checked_in == json_schema_for(record_type, schema_id)


def test_new_surface_records_stay_frozen_and_slotted():
    """Публичные записи заморожены и со слотами — тот же закон, что у фасада."""

    import dataclasses

    from cftuv_envelope.contracts import surface_adjacency, surface_metric_v2

    for module in (surface_adjacency, surface_metric_v2):
        records = [
            value
            for value in vars(module).values()
            if isinstance(value, type) and dataclasses.is_dataclass(value)
        ]
        assert records
        for record in records:
            assert record.__dataclass_params__.frozen, record
            assert hasattr(record, "__slots__"), record


def test_adjacency_table_is_not_part_of_the_snapshot_union():
    """V2 и таблица в union не проведены: транспорт — отдельная карточка."""

    from cftuv_envelope.contracts import analysis
    from cftuv_envelope.contracts.surface_metric_v2 import SurfaceMetricDescriptorV2

    import typing

    members = typing.get_args(analysis.SurfaceMetricDescriptorV1)
    assert SurfaceMetricDescriptorV2 not in members
    assert SurfaceAdjacencyIRV1 not in members


def test_vertex_fan_unavailable_reasons_are_named_not_boolean():
    reasons = {item.value for item in VertexFanUnavailableReasonV1}
    assert reasons == {
        "NON_MANIFOLD_FAN",
        "OUTSIDE_REQUESTED_SCOPE",
        "HOST_DECLARED_UNKNOWN",
    }
