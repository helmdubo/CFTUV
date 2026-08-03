"""Полевая приёмка смежности: хост против совместимого вывода, 11 снапшотов.

Приёмка сравнивает ДВА НЕЗАВИСИМЫХ ВЫВОДА одной величины. Хост считает по
хостовому `PatchSurfaceIR` и знает полный меш; ядро выводит из байтов
снапшота и полного меша не видит. Совпадение склейки и вееров — свидетельство;
расхождение было бы дефектом, а не поводом выбрать любимую версию.

Здесь же стоит N-4 с настоящими зубами: срез, объявленный полным мешем,
отвергается ХОСТОМ, потому что только у хоста есть число, которым это
доказывается — сколько граней полного меша держат ребро.
"""

from __future__ import annotations

import pytest

from cftuv.analysis_surface import (
    SurfaceAdjacencyBuildRejected,
    build_surface_adjacency_ir,
)
from cftuv_envelope.contracts.surface_adjacency import (
    SideBoundaryReasonV1,
    SideBoundaryV1,
    SurfaceAdjacencyScopeV1,
    TriangleSideRefV1,
    VertexFanUnavailableV1,
    surface_adjacency_digest,
    validate_surface_adjacency,
)
from cftuv_envelope.surface_adjacency_compat import read_surface_adjacency

from surface_adjacency_field_corpus import (
    FIELD_SNAPSHOT_NAMES,
    field_snapshot_paths,
    host_patch_surface,
    kernel_identity,
    load_edge_face_counts,
    load_snapshot,
)


# Полные меши корпуса: payload покрывает мешь целиком, и объявление
# `FULL_SOURCE_MESH` для них правдиво. Остальные восемь — срезы; число ниже
# ДОКАЗЫВАЕТСЯ прогоном, а не берётся на веру.
FULL_MESH_SNAPSHOTS = frozenset(
    {
        "building_patch10_density4_v1",
        "sem_clb_02_lost_domains_v1/cases/building_001_single_edge_patch_000_named_outcome_v1",
        "sem_clb_02_lost_domains_v1/cases/building_003_single_edge_patch_000_named_outcome_v1",
    }
)


def _host_table(folder, snapshot, *, scope):
    surface = host_patch_surface(snapshot)
    triangles, vertices, edges = kernel_identity(snapshot.surface_ir)
    return build_surface_adjacency_ir(
        surface,
        source_revision=snapshot.surface_ir.source_revision,
        triangle_ids=triangles,
        vertex_ids=vertices,
        edge_ids=edges,
        scope=scope,
        scope_patch_ids=(
            frozenset(item.patch_id for item in snapshot.patches)
            if scope == "REQUEST_SCOPED_PATCH_SET"
            else frozenset()
        ),
        edge_face_counts=load_edge_face_counts(folder),
    )


def _glue(table):
    return {
        (side.side.triangle_id, side.side.side_ordinal): (
            side.opposite.triangle_id,
            side.opposite.side_ordinal,
        )
        for side in table.triangle_sides
        if isinstance(side.opposite, TriangleSideRefV1)
    }


@pytest.fixture(scope="module", params=field_snapshot_paths(), ids=FIELD_SNAPSHOT_NAMES)
def field_case(request):
    folder = request.param
    return folder, load_snapshot(folder)


def test_the_corpus_is_the_eleven_field_snapshots():
    assert len(FIELD_SNAPSHOT_NAMES) == 11
    assert all(
        (folder / "analysis_snapshot.json").is_file()
        for folder in field_snapshot_paths()
    )


def test_host_table_passes_its_own_cross_validator(field_case):
    folder, snapshot = field_case
    table = _host_table(folder, snapshot, scope="REQUEST_SCOPED_PATCH_SET")
    validate_surface_adjacency(table, snapshot.surface_ir)
    assert table.scope is SurfaceAdjacencyScopeV1.REQUEST_SCOPED_PATCH_SET
    assert table.scope_patch_ids


def test_host_and_compat_derivations_agree_on_gluing_and_fans(field_case):
    """Главная сверка карточки: два вывода, одна смежность."""

    folder, snapshot = field_case
    host = _host_table(folder, snapshot, scope="REQUEST_SCOPED_PATCH_SET")
    reader = read_surface_adjacency(
        snapshot.surface_ir, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    assert _glue(host) == _glue(reader)
    assert host.vertex_fans == reader.vertex_fans
    assert not [
        fan for fan in host.vertex_fans if isinstance(fan, VertexFanUnavailableV1)
    ]


def test_only_the_host_can_name_the_boundary_of_a_request(field_case):
    """Разница двух выводов ровно там, где она обязана быть: в ПРИЧИНЕ края."""

    folder, snapshot = field_case
    if load_edge_face_counts(folder) is None:
        pytest.skip("для этой фикстуры полный меш не приложен")
    host = _host_table(folder, snapshot, scope="REQUEST_SCOPED_PATCH_SET")
    reader = read_surface_adjacency(
        snapshot.surface_ir, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    reader_boundary = {
        side.side
        for side in reader.triangle_sides
        if isinstance(side.opposite, SideBoundaryV1)
    }
    host_boundary = {
        side.side: side.opposite.reason
        for side in host.triangle_sides
        if isinstance(side.opposite, SideBoundaryV1)
    }
    assert reader_boundary == set(host_boundary)
    assert set(host_boundary.values()) <= {
        SideBoundaryReasonV1.MESH_BOUNDARY,
        SideBoundaryReasonV1.OUTSIDE_REQUESTED_SCOPE,
    }


def test_n4_the_host_refuses_a_slice_declared_as_a_full_mesh(field_case):
    """N-4: восемь срезов корпуса отвергаются, три полных меша проходят."""

    folder, snapshot = field_case
    name = "/".join(folder.parts[folder.parts.index("fixtures") + 1 :])
    if name in FULL_MESH_SNAPSHOTS:
        host = _host_table(folder, snapshot, scope="FULL_SOURCE_MESH")
        reader = read_surface_adjacency(
            snapshot.surface_ir, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
        )
        assert surface_adjacency_digest(host) == surface_adjacency_digest(reader)
        return
    with pytest.raises(SurfaceAdjacencyBuildRejected) as caught:
        _host_table(folder, snapshot, scope="FULL_SOURCE_MESH")
    assert caught.value.reason == "SURFACE_ADJACENCY_SCOPE_DECLARATION_INVALID"


def test_full_mesh_snapshots_are_exactly_three():
    assert len(FULL_MESH_SNAPSHOTS) == 3
    assert FULL_MESH_SNAPSHOTS <= set(FIELD_SNAPSHOT_NAMES)


def test_diagonals_of_the_corpus_never_carry_a_physical_edge_id(field_case):
    folder, snapshot = field_case
    host = _host_table(folder, snapshot, scope="REQUEST_SCOPED_PATCH_SET")
    diagonals = [
        side
        for side in host.triangle_sides
        if side.physical_edge_id is None
    ]
    assert diagonals, "в полевом корпусе диагонали тесселяции есть"
    named_edges = {
        side.physical_edge_id
        for side in host.triangle_sides
        if side.physical_edge_id is not None
    }
    assert None not in named_edges
