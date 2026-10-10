"""Индекс поверхности (`cftuv/surface_index.py`) отдаёт ТО ЖЕ, что фильтр по всей поверхности, в ТОМ ЖЕ порядке.

Вид пакета анализа по патчам, ключ полосы и цепочки домена считались проходом по всей поверхности (по всем цепочкам) на каждый домен:
на `cover.008` (1051 домен) это 4.8 с родителя до первой задачи пула и 2.3 с на каждом шаге ширины. Индекс строится один раз на объект
поверхности (графа, экспорта), а срез домена идёт по нему. Ответ не меняется: порядок записей питает дайджесты снапшота и ключи содержимого.

Сверка дифференциальная: `tests/prologue_index_legacy.py` хранит слово в слово прежний код (`2b89b89d`), а здесь он сравнивается с
продуктовым на КАЖДОМ домене синтетической поверхности (перемешанные порядки записей, чередующиеся патчи, рёбра графа между и внутри
патча, изолированный патч, повторная запись вершины), фикстур выпущенных снапшотов и меша `quad_row`; для двух последних сверяются и вход
выгрузки домена, и ключ содержимого, и канонические байты снапшота. Записанный полевой слепок `building` (122 домена) сверяет
`artifacts/host_prologue_index/field_probe.py` в отдельном процессе (`test_field_snapshot_building_views_keys_and_inputs_equal_the_filter`).
"""

from __future__ import annotations

import gc
import itertools
import json
import random
import subprocess
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import surface_index  # noqa: E402
from cftuv.envelope_content_key import domain_content_key  # noqa: E402
from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_metric_export import band_key_of  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    build_envelope_analysis_snapshot,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    EnvelopeTopologyExportV1,
    build_analysis_bundle_id_view,
    build_envelope_topology_export,
)
from cftuv.model import PatchGraph, PatchNode, SeamEdge  # noqa: E402
from cftuv.surface_ir import (  # noqa: E402
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)
from cftuv_envelope.canonical import canonical_json_bytes  # noqa: E402
from content_fixtures import with_revision  # noqa: E402
from envelope_fixture_bundles import (  # noqa: E402
    bundle_from_exported_snapshot,
    host_exported_snapshot_paths,
    quad_row_bundle,
)
from prologue_index_legacy import (  # noqa: E402
    legacy_band_key_of,
    legacy_host_export_input,
    legacy_view,
    view_parts,
)
from surface_adjacency_field_corpus import load_snapshot  # noqa: E402


# --------------------------------------------------------------------------
# Синтетическая поверхность: сетка четырёхгранников, патчи чередуются в порядке граней
# --------------------------------------------------------------------------


def grid_bundle(width=7, height=6, patch_count=9, seed=3, *, duplicate_vertex=False):
    """Сетка `width x height` четырёхгранников; номера вершин, рёбер и граней непрерывны НЕ везде, записи перемешаны.

    Патч клетки зависит от `(x // 2, y // 2, x * y % 2)`, поэтому грани одного патча чередуются с гранями других в порядке
    поверхности (по строкам), а общие вершины лежат на стыках многих патчей. Патч `patch_count + 5` узел графа без граней.
    """

    rng = random.Random(seed)
    revision = SourceRevision("grid", "sha256:grid")

    def vertex(x, y):
        return 10 + y * (width + 1) + x

    edge_numbers: dict = {}

    def edge(first, second):
        key = (min(first, second), max(first, second))
        return edge_numbers.setdefault(key, 3 * len(edge_numbers) + 1)

    vertices = {}
    for y in range(height + 1):
        for x in range(width + 1):
            vertices[vertex(x, y)] = (float(x), float(y), 0.05 * ((x * y) % 3))
    faces, triangles, edge_faces = [], [], {}
    cell_patch = {}
    for y in range(height):
        for x in range(width):
            patch = ((x // 2) + (y // 2) * 4 + (x * y) % 2) % patch_count
            cell_patch[(x, y)] = patch
            cycle = (vertex(x, y), vertex(x + 1, y), vertex(x + 1, y + 1), vertex(x, y + 1))
            edge_cycle = tuple(edge(cycle[i], cycle[(i + 1) % 4]) for i in range(4))
            face_id = 100 + 2 * (y * width + x)
            first_triangle = 7 * len(triangles) + 3
            triangle_ids = (first_triangle, first_triangle + 7)
            faces.append(SourceFace(face_id, patch, cycle, edge_cycle, (0.0, 0.0, 1.0), triangle_ids))
            for number, ring in zip(triangle_ids, ((cycle[0], cycle[1], cycle[2]), (cycle[0], cycle[2], cycle[3]))):
                triangles.append(SurfaceTriangle(number, face_id, ring, (edge_cycle[0], None, None), (0.0, 0.0, 1.0)))
            for number in edge_cycle:
                edge_faces.setdefault(number, []).append(face_id)
    source_edges = []
    for (first, second), number in edge_numbers.items():
        source_edges.append(SourceEdge(number, (first, second), tuple(edge_faces[number])))
    source_vertices = [SourceVertex(number, position) for number, position in vertices.items()]
    if duplicate_vertex:
        source_vertices.append(SourceVertex(vertex(2, 2), (9.0, 9.0, 9.0)))
    for records in (faces, triangles, source_edges, source_vertices):
        rng.shuffle(records)
    surface = PatchSurfaceIR(revision, tuple(source_vertices), tuple(source_edges), tuple(faces), tuple(triangles))

    graph = PatchGraph(source_revision=revision)
    patch_order = list(range(patch_count)) + [patch_count + 5]
    rng.shuffle(patch_order)
    for patch in patch_order:
        graph.add_node(
            PatchNode(patch_id=patch, face_indices=[item.face_id for item in faces if item.patch_id == patch])
        )
    pairs = set()
    for (x, y), patch in cell_patch.items():
        for dx, dy in ((1, 0), (0, 1)):
            other = cell_patch.get((x + dx, y + dy))
            if other is not None and other != patch:
                pairs.add((min(patch, other), max(patch, other)))
    pairs = sorted(pairs)
    rng.shuffle(pairs)
    for first, second in pairs:
        graph.add_edge(SeamEdge(first, second))
    # ребро графа внутри одного патча (оба конца в любом срезе, где есть патч) и ребро к изолированному патчу
    graph.edges[(2, 2)] = SeamEdge(2, 2)
    graph.add_edge(SeamEdge(1, patch_count + 5))
    return AnalysisBundle(revision, graph, surface)


def assert_same_view(bundle, included):
    new = build_analysis_bundle_id_view(bundle, frozenset(included))
    old = legacy_view(bundle, frozenset(included))
    assert view_parts(new) == view_parts(old), sorted(included)
    assert new.patch_surface == old.patch_surface
    assert new.source_revision is old.source_revision
    assert new.capabilities is old.capabilities


@pytest.fixture(scope="module")
def grid():
    return grid_bundle()


def test_every_single_patch_view_equals_the_filter(grid):
    for patch in grid.patch_graph.nodes:
        assert_same_view(grid, {patch})


def test_every_pair_of_patches_and_random_subsets_equal_the_filter(grid):
    patches = sorted(grid.patch_graph.nodes)
    for pair in itertools.combinations(patches, 2):
        assert_same_view(grid, set(pair))
    rng = random.Random(11)
    for _ in range(60):
        assert_same_view(grid, set(rng.sample(patches, rng.randint(3, len(patches)))))
    assert_same_view(grid, set(patches))


def test_the_ring_of_neighbours_is_the_ring_of_the_filter_and_not_empty(grid):
    """Сверка не тавтология: у среза одного патча кольцо соседей есть (иначе проверка граней вне среза ничего не проверяла)."""

    ring = build_analysis_bundle_id_view(grid, frozenset({0})).patch_surface.neighbour_faces
    assert ring
    assert all(item.patch_id != 0 for item in ring)


def test_a_duplicated_vertex_record_stays_in_surface_order():
    """Две записи с одним номером вершины остались обе, на своих местах, как у фильтра; положение берёт последняя запись."""

    bundle = grid_bundle(duplicate_vertex=True)
    for patch in bundle.patch_graph.nodes:
        assert_same_view(bundle, {patch})


def test_an_empty_and_an_unknown_domain_set_are_refused_as_before(grid):
    for included in (set(), {0, 999}, {999}):
        with pytest.raises(EnvelopeHostAdapterError) as new:
            build_analysis_bundle_id_view(grid, frozenset(included))
        with pytest.raises(EnvelopeHostAdapterError) as old:
            legacy_view(grid, frozenset(included))
        assert (new.value.outcome, str(new.value)) == (old.value.outcome, str(old.value))


def test_a_view_of_a_view_carries_the_ring_through_and_equals_the_filter(grid):
    """Срез воркера (класс со `slots`, слабой ссылки нет) несёт кольцо сам: оно переходит как есть, поверх него - вычисленное."""

    first = build_analysis_bundle_id_view(grid, frozenset({0, 1}))
    inner = SimpleNamespace(
        source_revision=first.source_revision,
        capabilities=first.capabilities,
        patch_graph=first.patch_graph,
        patch_surface=first.patch_surface,
    )
    assert first.patch_surface.neighbour_faces
    for included in ({0}, {1}, {0, 1}):
        new = build_analysis_bundle_id_view(inner, frozenset(included))
        old = legacy_view(inner, frozenset(included))
        assert view_parts(new) == view_parts(old), included
        assert new.patch_surface.neighbour_faces[: len(first.patch_surface.neighbour_faces)] == first.patch_surface.neighbour_faces


def test_the_patch_faces_helper_equals_the_filter(grid):
    surface = grid.patch_surface
    for patch in list(grid.patch_graph.nodes) + [-1, 77]:
        expected = tuple(face for face in surface.faces if face.patch_id == int(patch))
        found = surface_index.patch_faces_of(surface, patch)
        assert [id(item) for item in found] == [id(item) for item in expected]


# --------------------------------------------------------------------------
# Жизнь индекса
# --------------------------------------------------------------------------


def test_the_index_is_built_once_per_surface_and_graph(grid):
    surface, graph = grid.patch_surface, grid.patch_graph
    assert surface_index.surface_index_of(surface) is surface_index.surface_index_of(surface)
    assert surface_index.graph_index_of(graph) is surface_index.graph_index_of(graph)
    build_analysis_bundle_id_view(grid, frozenset({0}))
    assert surface_index.surface_index_of(surface) is surface_index.surface_index_of(surface)


def test_the_index_goes_away_with_its_surface_and_graph():
    bundle = grid_bundle(seed=5)
    keys = {id(bundle.patch_surface), id(bundle.patch_graph)}
    build_analysis_bundle_id_view(bundle, frozenset({0}))
    assert keys <= set(surface_index._INDEXES)
    del bundle
    gc.collect()
    assert not keys & set(surface_index._INDEXES)


def test_a_replaced_field_or_a_grown_graph_rebuilds_the_index_and_the_view_follows():
    bundle = grid_bundle(seed=7)
    before = view_parts(build_analysis_bundle_id_view(bundle, frozenset({0})))
    assert before == view_parts(legacy_view(bundle, frozenset({0})))
    surface = bundle.patch_surface
    kept = tuple(item for item in surface.faces if item.patch_id != 0)
    object.__setattr__(surface, "faces", kept)  # поле замороженной записи подменено: тождество кортежа другое
    assert view_parts(build_analysis_bundle_id_view(bundle, frozenset({0}))) == view_parts(legacy_view(bundle, frozenset({0})))
    assert not view_parts(build_analysis_bundle_id_view(bundle, frozenset({0})))["faces"]
    bundle.patch_graph.add_node(PatchNode(patch_id=40, face_indices=[]))
    assert_same_view(bundle, {40})
    bundle.patch_graph.add_edge(SeamEdge(1, 40))
    assert_same_view(bundle, {1, 40})


def test_a_replaced_graph_node_is_read_live():
    bundle = grid_bundle(seed=9)
    assert_same_view(bundle, {3})
    replacement = PatchNode(patch_id=3, face_indices=[1, 2, 3])
    bundle.patch_graph.nodes[3] = replacement
    assert build_analysis_bundle_id_view(bundle, frozenset({3})).patch_graph.nodes[3] is replacement


# --------------------------------------------------------------------------
# Ключ полосы и цепочки домена
# --------------------------------------------------------------------------


def _records():
    """Цепочки хоста для ключа полосы: он читает у записи ровно `patch_id` и `canonical_edge_ids`."""

    rng = random.Random(5)
    records = []
    for patch in (3, 0, 7, 1, 3, 0, 9):
        records.append(SimpleNamespace(patch_id=patch, canonical_edge_ids=tuple(rng.sample(range(40), 4))))
    return tuple(records)


@pytest.mark.parametrize("reach", [None, Fraction(1, 2), Fraction(7, 5)])
@pytest.mark.parametrize("selected", [(), (1, 5, 9), tuple(range(40)), (100, 200)])
def test_the_band_key_equals_the_scan_of_all_chains(reach, selected):
    topology = EnvelopeTopologyExportV1("rev", None, _records(), {patch: f"d{patch}" for patch in (0, 1, 3, 7, 9, 11)})
    plain = topology.with_chart_band(reach, selected)
    narrow = plain.with_band_tightened(Fraction(3, 10), "REFUSED")
    assert band_key_of(topology, 3) is None and legacy_band_key_of(topology, 3) is None
    for export in (plain, narrow):
        for patch in (0, 1, 3, 7, 9, 11, 40):
            assert band_key_of(export, patch) == legacy_band_key_of(export, patch)
            assert type(band_key_of(export, patch)[1]) is frozenset


def test_the_chain_index_belongs_to_its_export_and_is_not_part_of_its_value():
    topology = EnvelopeTopologyExportV1("rev", None, _records(), {0: "d0"})
    other = EnvelopeTopologyExportV1("rev", None, _records(), {0: "d0"})
    assert topology == other
    assert topology.patch_chains(3) == tuple(item for item in topology.host_chains if item.patch_id == 3)
    assert topology.patch_chains(5) == ()
    assert "_derived" not in repr(topology)
    assert type(topology.patch_edge_ids(3)) is frozenset
    copy = topology.with_chart_band(None, {1})
    assert copy._derived is not topology._derived
    assert copy.patch_edge_ids(3) == topology.patch_edge_ids(3)


def test_the_loops_of_the_seam_partition_are_renumbered_by_the_definition():
    """Номера цепочек петли идут по `(source_chain_index, source_segment_index)`, петли - по `(patch_id, loop_index)`.

    Группировка записей по петле одним проходом (вместо прохода по всем записям на петлю) оракулом: определение, не копия кода.
    """

    from cftuv.envelope_request_export import _HostChainRecord, _HostChainSlice, _normalize_physical_seam_partitions
    from cftuv.model_enums import ChainNeighborKind

    rng = random.Random(8)
    raw = []
    for patch in (5, 2, 9):
        for loop in (1, 0, 3):
            for source_chain in rng.sample(range(7), 7):
                first = 10 * patch + source_chain
                chain = _HostChainSlice(ChainNeighborKind.MESH_BORDER, -1, False, (first, first + 100), (first + 200,))
                raw.append(_HostChainRecord(patch, loop, source_chain, source_chain, 0, chain, chain.vert_indices, chain.edge_indices, False, False))
    rng.shuffle(raw)
    positions = {vertex: (float(vertex), 0.0, 0.0) for record in raw for vertex in record.chain.vert_indices}
    exact = {vertex: tuple(Fraction(axis) for axis in point) for vertex, point in positions.items()}

    found = _normalize_physical_seam_partitions(tuple(raw), exact)

    expected = [
        (patch, loop, rank, source_chain, 0)
        for patch in (2, 5, 9)
        for loop in (0, 1, 3)
        for rank, source_chain in enumerate(range(7))
    ]
    assert [(r.patch_id, r.loop_index, r.chain_index, r.source_chain_index, r.source_segment_index) for r in found] == expected
    assert len(found) == len(raw)


# --------------------------------------------------------------------------
# Вход выгрузки домена, ключ содержимого и байты снапшота: настоящие цепочки хоста
# --------------------------------------------------------------------------


def _field_bundles():
    yield "quad_row", with_revision(quad_row_bundle(5), "row")
    for path in host_exported_snapshot_paths():
        bundle, _patch = bundle_from_exported_snapshot(load_snapshot(path.parent))
        yield path.parent.name, bundle


def _compare_domains(name, bundle):
    topology = build_envelope_topology_export(bundle)
    every_edge = frozenset(
        int(edge) for record in topology.host_chains for edge in record.canonical_edge_ids
    )
    patch_ids = sorted(int(patch) for patch in bundle.patch_graph.nodes)
    banded = topology.with_chart_band(None, every_edge)
    checked = 0
    for patch_id in patch_ids:
        selected = frozenset(
            int(edge) for record in topology.host_chains if record.patch_id == patch_id for edge in record.canonical_edge_ids
        )
        fresh = build_host_export_input(topology, patch_id, alpha=0.25, request_id="request", density="1")
        old = legacy_host_export_input(topology, patch_id, alpha=0.25, request_id="request", density="1")
        assert fresh == old, (name, patch_id)
        # Воркер собирает вид ещё раз - на лёгком пакете этого домена (класс со `slots`: слабой ссылки нет, индекс на вызов).
        assert view_parts(build_analysis_bundle_id_view(fresh.bundle, frozenset({patch_id}))) == view_parts(
            legacy_view(fresh.bundle, frozenset({patch_id}))
        ), (name, patch_id)
        assert domain_content_key(fresh, selected, band_key_of(banded, patch_id)) == domain_content_key(
            old, selected, legacy_band_key_of(banded, patch_id)
        ), (name, patch_id)
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle, included_patch_ids=frozenset({patch_id}), topology_export=topology
            )
        except EnvelopeHostAdapterError as refused:
            with pytest.raises(EnvelopeHostAdapterError) as again:
                build_envelope_analysis_snapshot(
                    bundle,
                    included_patch_ids=frozenset({patch_id}),
                    topology_export=topology,
                    analysis_view=legacy_view(bundle, frozenset({patch_id})),
                )
            assert (again.value.outcome, str(again.value)) == (refused.outcome, str(refused)), (name, patch_id)
            continue
        legacy = build_envelope_analysis_snapshot(
            bundle,
            included_patch_ids=frozenset({patch_id}),
            topology_export=topology,
            analysis_view=legacy_view(bundle, frozenset({patch_id})),
        )
        assert canonical_json_bytes(snapshot) == canonical_json_bytes(legacy), (name, patch_id)
        checked += 1
    return len(patch_ids), checked


def test_export_input_content_key_and_snapshot_bytes_equal_the_filter_on_fixture_meshes():
    totals = {name: _compare_domains(name, bundle) for name, bundle in _field_bundles()}
    assert totals["quad_row"][0] == 5
    assert sum(checked for _domains, checked in totals.values()) >= 5, totals


# --------------------------------------------------------------------------
# Записанный полевой слепок `building` (отдельный процесс: харнесс дополняет заглушку Vector на весь процесс)
# --------------------------------------------------------------------------

FIELD_PROBE = ROOT / "artifacts" / "host_prologue_index" / "field_probe.py"
BUILDING_DOMAINS = 122


def test_field_snapshot_building_views_keys_and_inputs_equal_the_filter():
    finished = subprocess.run(
        [sys.executable, str(FIELD_PROBE)],
        capture_output=True,
        text=True,
        timeout=900,
        cwd=str(ROOT),
    )
    assert finished.returncode == 0, finished.stderr[-3000:]
    report = json.loads(finished.stdout.strip().splitlines()[-1])
    assert report["domains"] == BUILDING_DOMAINS
    assert report["mismatches"] == [], report["mismatches"][:5]
    assert report["views"] == BUILDING_DOMAINS and report["inputs"] == BUILDING_DOMAINS
    assert report["ring_faces"] > 0
