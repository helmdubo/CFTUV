"""Прежний (до индекса) способ собрать вид пакета анализа, ключ полосы и вход выгрузки домена: СВЕРКА, а не продукт.

Копии кода 2b89b89d, слово в слово: срез поверхности фильтром по ВСЕЙ поверхности на каждый домен
(`AnalysisBundleIdView.__post_init__`), ключ полосы проходом по всем цепочкам (`band_key_of`), цепочки домена проходом по всем
цепочкам (`build_host_export_input`). Продуктовый код после `surface_index` обязан давать ТО ЖЕ содержимое в ТОМ ЖЕ порядке (порядок
питает дайджесты), и эта сверка держит равенство на каждом домене синтетической поверхности, фикстур выпущенных снапшотов и
записанного полевого слепка (`tests/test_surface_index.py`, `tests/prologue_index_field_probe.py`).
"""

from __future__ import annotations

from types import SimpleNamespace

from cftuv.envelope_export_input import (
    HostExportInputV1,
    _LightBundleV1,
    _LightGraphV1,
    _light_patch,
)
from cftuv.envelope_seam_neighbours import narrowed_surface
from cftuv.envelope_topology_export import (
    NeighbourFaceV1,
    _PatchGraphIdView,
    _PatchSurfaceIdView,
)
from cftuv.envelope_request_policy import topology_chart_reach_cap


def legacy_view(analysis_bundle, included_patch_ids):
    """Вид пакета по патчам, как его собирал фильтр по всей поверхности: `SimpleNamespace` с теми же четырьмя свойствами."""

    included = frozenset(int(item) for item in included_patch_ids)
    available = frozenset(int(value) for value in analysis_bundle.patch_graph.nodes)
    unknown = included - available
    if not included or unknown:
        from cftuv.envelope_request_export import EnvelopeDebugHostOutcome, EnvelopeHostAdapterError

        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "request-scoped PatchDomain set is empty or unknown: " f"{sorted(included)}",
        )
    graph = analysis_bundle.patch_graph
    surface = analysis_bundle.patch_surface
    nodes = {int(patch_id): node for patch_id, node in graph.nodes.items() if int(patch_id) in included}
    edges = {
        key: edge
        for key, edge in graph.edges.items()
        if (int(edge.patch_a_id) in included and int(edge.patch_b_id) in included)
    }
    faces = tuple(item for item in surface.faces if int(item.patch_id) in included)
    face_ids = frozenset(int(item.face_id) for item in faces)
    edge_ids = frozenset(int(edge_id) for face in faces for edge_id in face.edge_cycle)
    vertex_ids = frozenset(int(vertex_id) for face in faces for vertex_id in face.vertex_cycle)
    position_of = {int(item.vertex_id): item.position for item in surface.vertices}
    neighbours = tuple(getattr(surface, "neighbour_faces", ())) + tuple(
        NeighbourFaceV1(
            int(item.face_id),
            int(item.patch_id),
            tuple(int(vertex_id) for vertex_id in item.vertex_cycle),
            tuple(tuple(float(axis) for axis in position_of[int(vertex_id)]) for vertex_id in item.vertex_cycle),
        )
        for item in surface.faces
        if int(item.patch_id) not in included and not vertex_ids.isdisjoint(int(vertex_id) for vertex_id in item.vertex_cycle)
    )
    revision = analysis_bundle.source_revision
    return SimpleNamespace(
        source_revision=revision,
        capabilities=analysis_bundle.capabilities,
        patch_graph=_PatchGraphIdView(revision, nodes, edges),
        patch_surface=_PatchSurfaceIdView(
            revision,
            tuple(item for item in surface.vertices if int(item.vertex_id) in vertex_ids),
            tuple(item for item in surface.edges if int(item.edge_id) in edge_ids),
            faces,
            tuple(item for item in surface.triangles if int(item.source_face_id) in face_ids),
            neighbours,
        ),
    )


def legacy_band_key_of(topology_export, patch_id):
    policy = topology_export.chart_band
    if policy is None:
        return None
    own = {
        int(edge)
        for record in topology_export.host_chains
        if record.patch_id == int(patch_id)
        for edge in record.canonical_edge_ids
    }
    key = (policy.reach_cap, frozenset(policy.selected_physical_edge_ids) & own)
    return key if policy.tightened_reach_cap is None else key + (("tightened", policy.tightened_reach_cap),)


def legacy_host_export_input(topology_export, patch_id, *, alpha, request_id, density) -> HostExportInputV1:
    patch_id = int(patch_id)
    view = legacy_view(topology_export.analysis_bundle, frozenset({patch_id}))
    graph = view.patch_graph
    chains = tuple(record for record in topology_export.host_chains if record.patch_id == patch_id)
    bundle = _LightBundleV1(
        view.source_revision,
        _LightGraphV1(
            view.source_revision,
            {int(key): _light_patch(node) for key, node in graph.nodes.items()},
            {},
        ),
        narrowed_surface(view.patch_surface, chains),
        view.capabilities,
    )
    return HostExportInputV1(
        topology_export.source_revision_value,
        bundle,
        chains,
        alpha,
        request_id,
        density,
        topology_export.developable_stretch_budget,
        topology_chart_reach_cap(topology_export),
        topology_export.silhouette_uv_slide,
    )


def view_parts(view):
    """Содержимое вида для сверки: узлы и рёбра графа (ключи и тождество записей по порядку), пять кортежей поверхности."""

    graph, surface = view.patch_graph, view.patch_surface
    return {
        "nodes": [(key, id(node)) for key, node in graph.nodes.items()],
        "edges": [(key, id(edge)) for key, edge in graph.edges.items()],
        "vertices": [id(item) for item in surface.vertices],
        "surface_edges": [id(item) for item in surface.edges],
        "faces": [id(item) for item in surface.faces],
        "triangles": [id(item) for item in surface.triangles],
        "neighbours": list(surface.neighbour_faces),
    }
