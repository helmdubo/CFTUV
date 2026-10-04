"""Ключ СОДЕРЖИМОТА домена полон: любой вход, от которого зависит ответ домена, меняет ключ.

Недостающий вход — устаревший результат, поэтому полнота не заявляется, а доказывается тремя способами:

1. ПЕРЕЧЕНЬ ПОЛЕЙ. Ключ строится из входа воркера (`HostExportInputV1`): каждое его поле либо в ключе, либо
   названо исключённым (`EXCLUDED_FIELDS`), и тест держит равенство перечней: новое поле входа без решения
   «входит или нет» — красный тест.
2. ВОЗМУЩЕНИЕ КАЖДОГО КЛАССА ВХОДА. Позиции, индексы поверхности, цепочки (в том числе общие со смежным
   патчем), углы, выделение, плотность, допуск растяжения, политики хоста, версия контрактов — каждый меняет
   ключ.
3. ДИФФЕРЕНЦИАЛЬНЫЙ ОРАКУЛ. Для набора правок меша: если СНАПШОТ домена (то, что реально видит ядро) изменился,
   изменился и ключ. Ключ, который мельче снапшота, — это и есть устаревший результат.

Обратная сторона тоже держится: ревизия источника, `alpha` и id запроса ключ не меняют, а сдвиг номеров
патчей (шов на другом конце меша) — тоже, потому что номера кодируются рангом.
"""

from __future__ import annotations

import dataclasses
import math
import sys
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_request_export as export_module  # noqa: E402
from cftuv.envelope_content_key import (  # noqa: E402
    EXCLUDED_FIELDS,
    ContentKeyUnsupported,
    domain_content_key,
    result_slot,
)
from cftuv.envelope_export_input import HostExportInputV1, build_host_export_input  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    build_envelope_analysis_snapshot,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
    stage_domain_inputs,
)
from cftuv.model_enums import ChainNeighborKind, LoopKind, PatchType  # noqa: E402
from cftuv.surface_ir import (  # noqa: E402
    HostCurvatureLadderPolicy,
    HostNearPlanarFramePolicy,
    HostNearPlanarLiftPolicy,
)
from content_fixtures import moved_vertex, renumbered, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))


def _domains(bundle, *, density="1", alpha=0.25, budget=None, edges=EDGES):
    """`{номер патча: (вход воркера, выделенные рёбра домена)}` для всех доменов меша."""

    topology = build_envelope_topology_export(bundle).with_developable_stretch_budget(budget)
    _scene, _revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, edges, topology_export=topology
    )
    return {
        patch_id: (
            build_host_export_input(
                topology, patch_id, alpha=alpha, request_id=request_id, density=density
            ),
            frozenset(by_domain[topology.patch_domain_id_by_patch[patch_id]]),
        )
        for patch_id in patch_ids
    }


def _keys(bundle, **kwargs):
    return {
        patch_id: domain_content_key(export, selected)
        for patch_id, (export, selected) in _domains(bundle, **kwargs).items()
    }


@pytest.fixture(scope="module")
def row():
    return with_revision(quad_row_bundle(ROW), "row")


# --------------------------------------------------------------------------
# Что ключ НЕ видит: ревизия, alpha, id запроса, номера патчей
# --------------------------------------------------------------------------


def test_the_key_ignores_the_revision_alpha_and_request_id(row):
    other = with_revision(row, "another-object", "f" * 64)

    assert other.source_revision != row.source_revision
    assert _keys(row, alpha=0.25) == _keys(other, alpha=0.9)
    first, second = _domains(row), _domains(other, alpha=0.9)
    assert first[2][0].source_revision_value != second[2][0].source_revision_value
    assert first[2][0].alpha != second[2][0].alpha


def test_the_key_of_every_domain_is_distinct_inside_one_mesh(row):
    assert len(set(_keys(row).values())) == ROW


def test_a_shift_of_patch_numbers_does_not_change_the_keys(row):
    """Шов на другом конце меша сдвигает номера следующих патчей при том же содержимом."""

    shifted = renumbered(row, {patch: patch + 3 for patch in range(ROW)})
    keys, moved = _keys(row), _keys(shifted)

    assert moved == {patch + 3: key for patch, key in keys.items()}


def test_the_order_of_neighbour_numbers_is_part_of_the_key(row):
    """Ранг соседей кодируется порядком их номеров: поменяли порядок — другой домен."""

    # Патч 2 граничит с 1 и 3; после перестановки 1 -> 8, 3 -> 6 сосед «левый» получает бОльший номер.
    swapped = renumbered(row, {1: 8, 3: 6})

    assert _keys(row)[2] != _keys(swapped)[2]


# --------------------------------------------------------------------------
# 1. Перечень полей входа воркера
# --------------------------------------------------------------------------


def test_every_field_of_the_worker_input_is_keyed_or_named_excluded():
    keyed = {"bundle", "host_chains", "density", "developable_stretch_budget"}
    names = {item.name for item in dataclasses.fields(HostExportInputV1)}

    assert names == keyed | set(EXCLUDED_FIELDS)
    assert set(EXCLUDED_FIELDS) == {"source_revision_value", "alpha", "request_id"}


# --------------------------------------------------------------------------
# 2. Возмущение каждого класса входа
# --------------------------------------------------------------------------


def _surface(export, **changes):
    view = dataclasses.replace(export.bundle.patch_surface, **changes)
    return dataclasses.replace(export, bundle=dataclasses.replace(export.bundle, patch_surface=view))


def _first(export, name, **changes):
    items = getattr(export.bundle.patch_surface, name)
    return _surface(export, **{name: (dataclasses.replace(items[0], **changes), *items[1:])})


def _chain(export, index=0, **changes):
    record = export.host_chains[index]
    chain = dataclasses.replace(record.chain, **changes)
    record = dataclasses.replace(record, chain=chain)
    return dataclasses.replace(export, host_chains=(*export.host_chains[:index], record, *export.host_chains[index + 1:]))


def _record(export, index=0, **changes):
    record = dataclasses.replace(export.host_chains[index], **changes)
    return dataclasses.replace(export, host_chains=(*export.host_chains[:index], record, *export.host_chains[index + 1:]))


def _patch(export, **changes):
    graph = export.bundle.patch_graph
    (number, node), = graph.nodes.items()
    light = dataclasses.replace(node, **changes)
    graph = dataclasses.replace(graph, nodes={number: light})
    return dataclasses.replace(export, bundle=dataclasses.replace(export.bundle, patch_graph=graph))


def _loop(export, **changes):
    (number, node), = export.bundle.patch_graph.nodes.items()
    loops = (dataclasses.replace(node.boundary_loops[0], **changes), *node.boundary_loops[1:])
    return _patch(export, boundary_loops=loops)


def _corner(export, **changes):
    (number, node), = export.bundle.patch_graph.nodes.items()
    loop = node.boundary_loops[0]
    corners = (dataclasses.replace(loop.corners[0], **changes), *loop.corners[1:])
    return _loop(export, corners=corners)


def _nudged(value):
    return math.nextafter(value, math.inf)


PERTURBATIONS = {
    # позиции и поверхность источника
    "vertex position, one ulp": lambda e: _first(e, "vertices", position=tuple(_nudged(v) if i == 0 else v for i, v in enumerate(e.bundle.patch_surface.vertices[0].position))),
    "vertex index": lambda e: _first(e, "vertices", vertex_id=e.bundle.patch_surface.vertices[0].vertex_id + 100),
    "edge adjacency": lambda e: _first(e, "edges", source_face_ids=(*e.bundle.patch_surface.edges[0].source_face_ids, 99)),
    "edge endpoints": lambda e: _first(e, "edges", vertex_ids=tuple(reversed(e.bundle.patch_surface.edges[0].vertex_ids))),
    "face normal": lambda e: _first(e, "faces", polygon_normal=(0.0, _nudged(0.0), 1.0)),
    "face vertex cycle": lambda e: _first(e, "faces", vertex_cycle=tuple(reversed(e.bundle.patch_surface.faces[0].vertex_cycle))),
    "face edge cycle": lambda e: _first(e, "faces", edge_cycle=tuple(reversed(e.bundle.patch_surface.faces[0].edge_cycle))),
    "face triangle ids": lambda e: _first(e, "faces", triangle_ids=(7, 8)),
    "triangle normal": lambda e: _first(e, "triangles", triangle_normal=(0.0, 0.0, -1.0)),
    "triangle vertices": lambda e: _first(e, "triangles", vertex_ids=tuple(reversed(e.bundle.patch_surface.triangles[0].vertex_ids))),
    "triangle physical edges": lambda e: _first(e, "triangles", physical_edge_ids=(None, None, None)),
    "triangle order": lambda e: _surface(e, triangles=tuple(reversed(e.bundle.patch_surface.triangles))),
    "face order": lambda e: _surface(e, faces=(e.bundle.patch_surface.faces[0], e.bundle.patch_surface.faces[0])),
    "vertex removed": lambda e: _surface(e, vertices=e.bundle.patch_surface.vertices[1:]),
    # цепочки хоста, в том числе общий шов со смежным патчем
    "chain neighbour kind": lambda e: _chain(e, neighbor_kind=ChainNeighborKind.SEAM_SELF),
    "chain neighbour mesh border": lambda e: _chain(e, 1, neighbor_patch_id=-1, neighbor_kind=ChainNeighborKind.MESH_BORDER),
    "chain closedness": lambda e: _chain(e, is_closed=True),
    "chain vertices": lambda e: _chain(e, vert_indices=(*e.host_chains[0].chain.vert_indices, 99)),
    "chain edges": lambda e: _chain(e, edge_indices=(*e.host_chains[0].chain.edge_indices, 99)),
    "record canonical vertices": lambda e: _record(e, canonical_vertex_ids=tuple(reversed(e.host_chains[0].canonical_vertex_ids))),
    "record canonical edges": lambda e: _record(e, canonical_edge_ids=(*e.host_chains[0].canonical_edge_ids, 99)),
    "record direction": lambda e: _record(e, reversed_from_canonical=not e.host_chains[0].reversed_from_canonical),
    "record source closedness": lambda e: _record(e, source_is_closed=True),
    "record chain index": lambda e: _record(e, chain_index=e.host_chains[0].chain_index + 7),
    "record loop index": lambda e: _record(e, loop_index=e.host_chains[0].loop_index + 7),
    "record source chain": lambda e: _record(e, source_chain_index=e.host_chains[0].source_chain_index + 7),
    "record source segment": lambda e: _record(e, source_segment_index=e.host_chains[0].source_segment_index + 7),
    "chain dropped": lambda e: dataclasses.replace(e, host_chains=e.host_chains[1:]),
    "chain order": lambda e: dataclasses.replace(e, host_chains=tuple(reversed(e.host_chains))),
    # патч и его углы
    "patch type": lambda e: _patch(e, patch_type=PatchType.WALL if next(iter(e.bundle.patch_graph.nodes.values())).patch_type is not PatchType.WALL else PatchType.SLOPE),
    "patch shape class": lambda e: _patch(e, shape_class="BAND"),
    "loop kind": lambda e: _loop(e, kind=LoopKind.HOLE),
    "loop chain count": lambda e: _loop(e, chains=(None,) * 9),
    # плотность и допуск растяжения запроса
    "density": lambda e: dataclasses.replace(e, density="3"),
    "density to the old law": lambda e: dataclasses.replace(e, density=None),
    "stretch budget": lambda e: dataclasses.replace(e, developable_stretch_budget=Fraction(1, 4)),
    "capabilities": lambda e: dataclasses.replace(e, bundle=dataclasses.replace(e.bundle, capabilities=dataclasses.replace(e.bundle.capabilities, geometry_batch_schema=2))),
}


@pytest.mark.parametrize("name", sorted(PERTURBATIONS))
def test_every_class_of_input_changes_the_key(row, name):
    export, selected = _domains(row)[2]
    baseline = domain_content_key(export, selected)

    perturbed = PERTURBATIONS[name](export)

    assert perturbed != export
    assert domain_content_key(perturbed, selected) != baseline


def test_a_corner_of_the_loop_changes_the_key():
    """Углы петли в квадрате отсутствуют: проверяем на домене с углами из полевого снапшота хоста."""

    from envelope_fixture_bundles import bundle_from_exported_snapshot, host_exported_snapshot_paths
    from surface_adjacency_field_corpus import load_snapshot

    export = corner = None
    for path in host_exported_snapshot_paths():
        bundle, _patch_id = bundle_from_exported_snapshot(load_snapshot(path.parent))
        patch_id = next(iter(bundle.patch_graph.nodes))
        topology = build_envelope_topology_export(bundle)
        export = build_host_export_input(topology, patch_id, alpha=0.25, request_id="r", density="1")
        node = next(iter(export.bundle.patch_graph.nodes.values()))
        corner = next((c for loop in node.boundary_loops for c in loop.corners), None)
        if corner is not None:
            break
    assert corner is not None, "no field snapshot of the corpus declares a corner"
    baseline = domain_content_key(export, frozenset())

    for change in ({"prev_chain_index": corner.prev_chain_index + 1}, {"next_chain_index": corner.next_chain_index + 1}, {"vert_index": corner.vert_index + 1}):
        assert domain_content_key(_corner(export, **change), frozenset()) != baseline


def test_the_selection_of_the_domain_changes_the_key_and_a_foreign_one_does_not(row):
    export, selected = _domains(row)[2]
    keys = _keys(row)
    wider = _keys(row, edges=EDGES | {ROW + 2})  # верхнее ребро патча 2: выделение ТОЛЬКО его домена

    assert domain_content_key(export, selected | {ROW + 99}) != keys[2]
    assert wider[2] != keys[2]
    assert {patch: key for patch, key in wider.items() if patch != 2} == {
        patch: key for patch, key in keys.items() if patch != 2
    }


@pytest.mark.parametrize(
    "name,value",
    (
        ("HOST_PLANARITY_POLICY", None),
        ("HOST_GRID_POLICY", None),
        ("HOST_NEAR_PLANAR_FRAME_POLICY", HostNearPlanarFramePolicy.CANONICAL_ONLY_V1),
        ("HOST_NEAR_PLANAR_LIFT_POLICY", HostNearPlanarLiftPolicy.CERTIFIED_PLANE_V1),
        ("HOST_CURVATURE_LADDER_POLICY", HostCurvatureLadderPolicy.NEAR_PLANAR_ONLY_V1),
    ),
)
def test_every_host_policy_that_the_export_reads_changes_the_key(row, monkeypatch, name, value):
    export, selected = _domains(row)[2]
    baseline = domain_content_key(export, selected)
    current = getattr(export_module, name)
    if value is None:
        members = [item for item in type(current) if item is not current]
        value = members[0]

    monkeypatch.setattr(export_module, name, value)

    assert domain_content_key(export, selected) != baseline


def test_the_code_identity_changes_the_key(row, monkeypatch):
    from cftuv import envelope_content_key

    export, selected = _domains(row)[2]
    baseline = domain_content_key(export, selected)
    kernel, host = envelope_content_key.code_identity()

    monkeypatch.setattr(envelope_content_key, "code_identity", lambda: (kernel + "+x", host))
    assert domain_content_key(export, selected) != baseline
    monkeypatch.setattr(envelope_content_key, "code_identity", lambda: (kernel, host + "+x"))
    assert domain_content_key(export, selected) != baseline


def test_the_code_identity_is_the_fingerprint_the_installer_computes():
    """Отпечаток кода — не `__version__` (она не менялась за десяток слияний ядра), а sha256 по содержимому .py."""

    import subprocess

    from cftuv.envelope_content_key import code_identity

    repository = Path(__file__).resolve().parents[1]
    printed = subprocess.run(
        [sys.executable, str(repository / "tools" / "blender_check_install.py"), "--fingerprint"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    expected = dict(line.strip().split(": ") for line in printed.strip().splitlines())

    assert code_identity() == (expected["cftuv_envelope"], expected["cftuv"])
    assert all(len(item) == 16 for item in code_identity())


def test_the_fingerprint_follows_the_content_of_the_sources_and_not_their_line_endings(tmp_path):
    from cftuv.envelope_content_key import package_fingerprint

    package = tmp_path / "package"
    (package / "inner").mkdir(parents=True)
    (package / "a.py").write_bytes(b"x = 1\r\ny = 2\r\n")
    (package / "inner" / "b.py").write_bytes(b"z = 3\n")
    (package / "notes.txt").write_text("not a source")
    first = package_fingerprint(str(package))

    (package / "a.py").write_bytes(b"x = 1\ny = 2\n")  # те же строки в LF
    (package / "notes.txt").write_text("changed, still not a source")
    assert package_fingerprint(str(package)) == first
    (package / "inner" / "b.py").write_bytes(b"z = 4\n")  # правка одного файла
    changed = package_fingerprint(str(package))
    assert changed != first
    (package / "inner" / "b.py").rename(package / "inner" / "c.py")  # то же содержимое под другим именем
    assert package_fingerprint(str(package)) != changed
    assert package_fingerprint(str(tmp_path / "missing")) == "<нет каталога>"


def test_a_constant_of_the_request_policy_changes_the_key(row, monkeypatch):
    """Подмена константы политики запроса между нажатиями — другой ключ, а не устаревший результат."""

    from cftuv import envelope_request_policy as policy

    export, selected = _domains(row)[2]
    baseline = domain_content_key(export, selected)

    monkeypatch.setattr(policy, "DEFAULT_ENVELOPE_STRETCH_BUDGET", Fraction(1, 4))
    assert domain_content_key(export, selected) != baseline
    monkeypatch.undo()
    assert domain_content_key(export, selected) == baseline

    real = policy.envelope_angular_policy

    def other_fan(kernel, density, budget=None):
        angular = real(kernel, density, budget)
        return dataclasses.replace(angular, selection_policy_id="ANOTHER_TABLE")

    monkeypatch.setattr(policy, "envelope_angular_policy", other_fan)
    assert domain_content_key(export, selected) != baseline


def test_a_value_the_encoder_does_not_know_refuses_the_key(row):
    export, selected = _domains(row)[2]
    poisoned = dataclasses.replace(export, host_chains=(*export.host_chains, object()))

    with pytest.raises(ContentKeyUnsupported):
        domain_content_key(poisoned, selected)


def test_the_result_slot_is_alpha_and_the_materialization_laws():
    assert result_slot("0.25", "UV", "PLANAR", "LIFT") != result_slot("0.3", "UV", "PLANAR", "LIFT")
    assert len({result_slot("0.25", "UV", "PLANAR", "LIFT"), result_slot("0.25", "UV2", "PLANAR", "LIFT"), result_slot("0.25", "UV", "TRI", "LIFT"), result_slot("0.25", "UV", "PLANAR", "LIFT2")}) == 4


# --------------------------------------------------------------------------
# 3. Дифференциальный оракул: изменился снапшот — изменился и ключ
# --------------------------------------------------------------------------


def _snapshot_bytes(bundle, patch_id):
    from cftuv_envelope.canonical import canonical_json_bytes

    topology = build_envelope_topology_export(bundle)
    try:
        snapshot = build_envelope_analysis_snapshot(
            bundle,
            included_patch_ids=frozenset({patch_id}),
            topology_export=topology,
        )
    except EnvelopeHostAdapterError as refusal:
        return b"refused:" + str(refusal).encode("utf-8")
    return canonical_json_bytes(snapshot)


def _same_revision_variants():
    """Правки меша при ОДНОЙ ревизии: идентичности снапшотов совпадают, и снапшоты сравнимы побайтово."""

    base = quad_row_bundle(ROW)

    def surface(**changes):
        from cftuv.surface_ir import AnalysisBundle

        edited = dataclasses.replace(base.patch_surface, **changes)
        return AnalysisBundle(base.source_revision, base.patch_graph, edited)

    vertices = base.patch_surface.vertices
    triangles = base.patch_surface.triangles
    return {
        "lifted corner": quad_row_bundle(ROW, lifted_corner=0.3),
        "moved bottom vertex": surface(
            vertices=tuple(
                dataclasses.replace(item, position=(item.position[0] + 0.5, item.position[1], item.position[2]))
                if item.vertex_id == 2
                else item
                for item in vertices
            )
        ),
        "tilted triangle normal": surface(
            triangles=tuple(
                dataclasses.replace(item, triangle_normal=(0.0, 0.6, 0.8)) if index == 4 else item
                for index, item in enumerate(triangles)
            )
        ),
    }


def test_when_the_snapshot_of_a_domain_changes_so_does_its_key():
    base = quad_row_bundle(ROW)
    base_keys = _keys(base)
    base_snapshots = {patch: _snapshot_bytes(base, patch) for patch in range(ROW)}
    touched_somewhere = False

    for name, variant in _same_revision_variants().items():
        keys = _keys(variant)
        for patch in range(ROW):
            snapshot_changed = _snapshot_bytes(variant, patch) != base_snapshots[patch]
            touched_somewhere = touched_somewhere or snapshot_changed
            if snapshot_changed:
                assert keys[patch] != base_keys[patch], f"{name}: patch {patch} changed but its key did not"
    assert touched_somewhere


def test_the_oracle_is_not_vacuous_a_vertex_belongs_to_the_domains_around_it(row):
    """Позиция вершины — вход ровно тех доменов, чьи грани её содержат; остальные ключи не двигаются."""

    keys = _keys(row)
    moved = _keys(moved_vertex(row, 2, (4.0, 0.0, 0.5)))

    # Вершина 2 — нижний угол между квадратами 1 и 2.
    assert {patch for patch in keys if keys[patch] != moved[patch]} == {1, 2}


def _seam_partition_pair():
    """Один меш в двух видах: сосед делит общий шов на две цепочки либо держит его одной (одна ревизия на оба)."""

    import copy

    from test_envelope_host_adapter import _mismatched_seam_partition_bundle

    from cftuv.surface_ir import AnalysisBundle

    split = _mismatched_seam_partition_bundle()
    graph = copy.deepcopy(split.patch_graph)
    graph.source_revision = split.source_revision
    coarse = AnalysisBundle(split.source_revision, graph, split.patch_surface)
    left = coarse.patch_graph.nodes[0].boundary_loops[0]
    first, second = left.chains[1], left.chains[2]
    left.chains[1:3] = [
        dataclasses.replace(
            first,
            vert_indices=[1, 2, 3],
            vert_cos=[first.vert_cos[0], first.vert_cos[1], second.vert_cos[1]],
            edge_indices=[1, 2],
            side_face_normals=[*first.side_face_normals, *second.side_face_normals],
        )
    ]
    return split, coarse


def test_a_neighbour_that_splits_the_shared_seam_changes_the_key_of_the_other_domain():
    """Домен не видит соседа, но видит общее измельчение шва: цепочка, разрезанная по вершине соседа, — вход домена.

    Это класс «общие цепочки смежных доменов» для сварки: правка шва у соседа меняет записи цепочек домена, а
    с ними и ключ; снапшот домена при этом действительно другой (оракул из предыдущего раздела).
    """

    split, coarse = _seam_partition_pair()
    own_edge = frozenset({5})  # внешнее ребро правого патча: выделен только его домен

    keys = {name: _keys(bundle, edges=own_edge) for name, bundle in (("split", split), ("coarse", coarse))}

    assert set(keys["split"]) == set(keys["coarse"]) == {1}
    assert keys["split"][1] != keys["coarse"][1]
    assert _snapshot_bytes(split, 1) != _snapshot_bytes(coarse, 1)
