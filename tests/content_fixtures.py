"""Мешы-правки для тестов хранилища по содержимому: та же сцена при другой ревизии, номерах патчей, позиции.

`quad_row_bundle` держит ОДНУ ревизию на любое содержимое, а настоящая ревизия — хэш всего меша. Здесь ревизия
считается от содержимого так же: правка вершины или шва меняет её, переименование объекта — нет.
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import pickle

from cftuv.envelope_domain_pool import DomainPoolRunV1, DomainTaskV1, order_by_cost, solve_task
from cftuv.envelope_export_input import build_host_export_input
from cftuv.envelope_production_export import ColdProductionInputV1, solve_cold_production_task
from cftuv.envelope_topology_export import build_envelope_topology_export, stage_domain_inputs
from cftuv.model import PatchGraph
from cftuv.surface_ir import AnalysisBundle, SourceRevision


def content_digest(bundle: AnalysisBundle) -> str:
    """Хэш содержимого, как у настоящей ревизии: позиции, топология поверхности и швы патчей."""

    surface = bundle.patch_surface
    seams = [
        (
            patch_id,
            [
                (loop.kind.value, [(tuple(c.vert_indices), tuple(c.edge_indices), c.neighbor_patch_id) for c in loop.chains])
                for loop in node.boundary_loops
            ],
        )
        for patch_id, node in sorted(bundle.patch_graph.nodes.items())
    ]
    text = repr((surface.vertices, surface.edges, surface.faces, surface.triangles, seams))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def with_revision(bundle: AnalysisBundle, name: str = "row", digest: str | None = None) -> AnalysisBundle:
    """Тот же меш под ревизией `(name, digest)`; без `digest` он считается от содержимого."""

    revision = SourceRevision(name, digest or content_digest(bundle))
    graph = copy.copy(bundle.patch_graph)
    graph.source_revision = revision
    surface = dataclasses.replace(bundle.patch_surface, source_revision=revision)
    return AnalysisBundle(revision, graph, surface)


def moved_vertex(bundle: AnalysisBundle, vertex_id: int, position, name: str = "row") -> AnalysisBundle:
    """Правка позиции одной вершины: ревизия меняется, номера патчей и индексы те же."""

    surface = bundle.patch_surface
    vertices = tuple(
        dataclasses.replace(item, position=tuple(float(value) for value in position))
        if item.vertex_id == vertex_id
        else item
        for item in surface.vertices
    )
    edited = dataclasses.replace(surface, vertices=vertices)
    return with_revision(
        AnalysisBundle(bundle.source_revision, bundle.patch_graph, edited), name
    )


def renumbered(bundle: AnalysisBundle, mapping: dict[int, int], name: str = "row") -> AnalysisBundle:
    """Те же патчи под другими номерами (шов на другом конце меша сдвигает номера следующих патчей).

    Индексы вершин, рёбер, граней и треугольников источника не меняются: это ровно та правка, после которой
    содержимое домена то же, а номер его патча и номера соседей другие.
    """

    def moved(number: int) -> int:
        return mapping.get(number, number) if number >= 0 else number

    graph = PatchGraph(source_revision=bundle.source_revision)
    for node in bundle.patch_graph.nodes.values():
        clone = copy.deepcopy(node)
        clone.patch_id = moved(node.patch_id)
        for loop in clone.boundary_loops:
            for chain in loop.chains:
                chain.neighbor_patch_id = moved(chain.neighbor_patch_id)
        graph.add_node(clone)
    surface = dataclasses.replace(
        bundle.patch_surface,
        faces=tuple(
            dataclasses.replace(face, patch_id=moved(face.patch_id)) for face in bundle.patch_surface.faces
        ),
    )
    return with_revision(AnalysisBundle(bundle.source_revision, graph, surface), name)


def cold_domain(bundle, patch_id, edges, *, alpha=0.25, density="1"):
    """Холодный домен так, как его считает воркер: `(результат с записью токенов, подготовка, ревизия, id запроса)`."""

    topology = build_envelope_topology_export(bundle)
    _scene, revision, _ids, request_id, by_domain = stage_domain_inputs(
        bundle, edges, topology_export=topology
    )
    domain_id = topology.patch_domain_id_by_patch[patch_id]
    export = build_host_export_input(
        topology, patch_id, alpha=alpha, request_id=request_id, density=density
    )
    task = DomainTaskV1(
        0,
        patch_id,
        domain_id,
        None,
        None,
        str(float(alpha)),
        frozenset(by_domain[domain_id]),
        export=export,
        cold=ColdProductionInputV1(),
    )
    reply = solve_cold_production_task(task)
    return reply.production, reply.prepared, revision, request_id


class InProcessPool:
    """Пул без подпроцессов: та же `solve_task`, ответ через pickle, как по трубе."""

    requested = 2

    def __init__(self) -> None:
        self.kinds: list[str] = []

    def run(self, tasks):
        results = {}
        for task, _frame in order_by_cost(tasks):
            self.kinds.append(
                "production"
                if task.production is not None
                else "cold"
                if task.cold is not None
                else "other"
            )
            results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        return DomainPoolRunV1(results, self.requested)


__all__ = (
    "InProcessPool",
    "cold_domain",
    "content_digest",
    "moved_vertex",
    "renumbered",
    "with_revision",
)
