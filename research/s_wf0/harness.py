"""Research-only comparative harness for S-WF0.

Ни один символ этого модуля не импортируется production-кодом аддона.
"""

from __future__ import annotations

import json
import importlib.metadata
import math
import platform
import runpy
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    from .fixtures import (
        Fixture,
        boundary_edges,
        connected_components,
        mesh_edges,
        source_records,
    )
except ImportError:  # Direct script execution from research/s_wf0.
    from fixtures import (
        Fixture,
        boundary_edges,
        connected_components,
        mesh_edges,
        source_records,
    )


SCHEMA = "cftuv.decal_s_wf0_results.v1"
METHODS = ("CURRENT_RAIL_CHART", "HEAT", "FMM", "MMP_EXACT")
TIMING_REPEATS = 3


@dataclass
class FieldResult:
    method: str
    status: str
    distances: np.ndarray
    owners: tuple[str, ...]
    source_s: np.ndarray
    ambiguous_owner: np.ndarray
    ambiguous_source: np.ndarray
    compile_ms: float
    extract_ms: float
    diagnostics: dict = field(default_factory=dict)
    native_events: tuple[dict, ...] = ()
    native_loci: tuple[str, ...] = ()


def _median_timed(factory, repeats=TIMING_REPEATS):
    values = []
    result = None
    for _repeat in range(repeats):
        started = time.perf_counter_ns()
        result = factory()
        values.append((time.perf_counter_ns() - started) / 1_000_000.0)
    return result, float(statistics.median(values))


def _component_payloads(fixture):
    payloads = []
    records, _length = source_records(fixture)
    for component in connected_components(fixture):
        component_set = set(component)
        local_by_global = {value: index for index, value in enumerate(component)}
        local_faces = [
            tuple(local_by_global[int(value)] for value in face)
            for face in fixture.faces
            if all(int(value) in component_set for value in face)
        ]
        component_records = tuple(
            record for record in records if record["vertex_id"] in component_set
        )
        payloads.append(
            {
                "global_ids": component,
                "local_by_global": local_by_global,
                "vertices": np.asarray(fixture.vertices[list(component)], dtype=float),
                "faces": np.asarray(local_faces, dtype=np.int32),
                "records": component_records,
            }
        )
    return tuple(payloads)


def _external_solver_factory(method, vertices, faces):
    if method in {"HEAT", "FMM"}:
        import potpourri3d_bindings as bindings

        if method == "HEAT":
            return bindings.MeshHeatMethodDistance(vertices, faces, 1.0, True)
        return bindings.MeshFastMarchingDistance(vertices, faces)
    if method == "MMP_EXACT":
        from pygeodesic import geodesic

        return geodesic.PyGeodesicAlgorithmExact(vertices, faces)
    raise ValueError(method)


def _solve_one(method, solver, source_index):
    if method == "HEAT":
        values = np.asarray(solver.compute_distance(int(source_index)), dtype=float)
        values = values - values[int(source_index)]
        return np.maximum(values, 0.0)
    if method == "FMM":
        return np.asarray(
            solver.compute_distance(
                [[(int(source_index), [])]],
                [],
                False,
            ),
            dtype=float,
        )
    values, _best_source = solver.geodesicDistances(
        np.asarray([int(source_index)], dtype=np.int32),
        None,
    )
    return np.asarray(values, dtype=float)


def run_external_field(fixture, method):
    payloads = _component_payloads(fixture)

    def build():
        return tuple(
            _external_solver_factory(method, payload["vertices"], payload["faces"])
            for payload in payloads
            if payload["records"]
        )

    solvers, compile_ms = _median_timed(build)

    def extract():
        distances = np.full(len(fixture.vertices), np.inf, dtype=float)
        source_s = np.full(len(fixture.vertices), np.nan, dtype=float)
        owners = [""] * len(fixture.vertices)
        ambiguous_owner = np.zeros(len(fixture.vertices), dtype=bool)
        ambiguous_source = np.zeros(len(fixture.vertices), dtype=bool)
        solver_index = 0
        source_count = 0
        for payload in payloads:
            if not payload["records"]:
                continue
            solver = solvers[solver_index]
            solver_index += 1
            cache = {}
            rows = []
            for record in payload["records"]:
                local_source = payload["local_by_global"][record["vertex_id"]]
                if local_source not in cache:
                    cache[local_source] = _solve_one(method, solver, local_source)
                rows.append(cache[local_source])
            matrix = np.vstack(rows)
            source_count += len(rows)
            finite_matrix = np.where(np.isfinite(matrix), matrix, np.inf)
            order = np.argsort(finite_matrix, axis=0, kind="stable")
            best_rows = order[0]
            best = finite_matrix[best_rows, np.arange(finite_matrix.shape[1])]
            second = (
                finite_matrix[order[1], np.arange(finite_matrix.shape[1])]
                if len(rows) > 1
                else np.full(finite_matrix.shape[1], np.inf)
            )
            owner_names = tuple(sorted({record["owner"] for record in payload["records"]}))
            owner_matrix = np.vstack(
                [
                    np.min(
                        finite_matrix[
                            [
                                index
                                for index, record in enumerate(payload["records"])
                                if record["owner"] == owner
                            ]
                        ],
                        axis=0,
                    )
                    for owner in owner_names
                ]
            )
            owner_order = np.argsort(owner_matrix, axis=0, kind="stable")
            owner_best = owner_order[0]
            owner_second = (
                owner_matrix[owner_order[1], np.arange(owner_matrix.shape[1])]
                if len(owner_names) > 1
                else np.full(owner_matrix.shape[1], np.inf)
            )
            tolerance = max(1e-10, fixture.alpha_reference * 1e-7)
            global_ids = np.asarray(payload["global_ids"], dtype=int)
            distances[global_ids] = best
            source_s[global_ids] = np.asarray(
                [payload["records"][index]["source_s"] for index in best_rows],
                dtype=float,
            )
            for local_index, global_id in enumerate(global_ids):
                owners[int(global_id)] = owner_names[int(owner_best[local_index])]
            ambiguous_owner[global_ids] = owner_second - owner_matrix[
                owner_best, np.arange(owner_matrix.shape[1])
            ] <= tolerance
            ambiguous_source[global_ids] = second - best <= tolerance
        return (
            distances,
            tuple(owners),
            source_s,
            ambiguous_owner,
            ambiguous_source,
            source_count,
        )

    extracted, extract_ms = _median_timed(extract)
    distances, owners, station, ambiguous_owner, ambiguous_source, source_count = extracted
    return FieldResult(
        method=method,
        status="ok",
        distances=distances,
        owners=owners,
        source_s=station,
        ambiguous_owner=ambiguous_owner,
        ambiguous_source=ambiguous_source,
        compile_ms=compile_ms,
        extract_ms=extract_ms,
        diagnostics={
            "solver_components": sum(bool(payload["records"]) for payload in payloads),
            "source_record_count": source_count,
            "source_semantics": "DISCRETE_SOURCE_VERTICES",
        },
    )


def _bootstrap_current_runtime():
    root = Path(__file__).resolve().parents[2]
    tests = root / "tests"
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    if str(tests) not in sys.path:
        sys.path.insert(0, str(tests))
    if "mathutils" not in sys.modules:
        runpy.run_path(str(tests / "conftest.py"))


def _normal(points):
    value = np.cross(points[1] - points[0], points[2] - points[0])
    length = float(np.linalg.norm(value))
    if length <= 1e-15:
        return (0.0, 0.0, 0.0)
    return tuple(float(component) for component in value / length)


def _source_edge_metadata(fixture):
    records, _length = source_records(fixture)
    records_by_owner_vertex = {
        (record["owner"], record["vertex_id"]): record for record in records
    }
    result = {}
    for branch in fixture.branches:
        ids = branch.vertex_ids
        pairs = list(zip(ids, ids[1:]))
        if branch.closed:
            pairs.append((ids[-1], ids[0]))
        for first, second in pairs:
            result[tuple(sorted((first, second)))] = {
                "owner": branch.owner,
                "s0": records_by_owner_vertex[(branch.owner, first)]["source_s"],
                "s1": records_by_owner_vertex[(branch.owner, second)]["source_s"],
                "oriented": (first, second),
            }
    return result


def _build_current_input(fixture):
    _bootstrap_current_runtime()
    from mathutils import Vector
    from cftuv.decal_charts import ChartSiteSeed
    from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode
    from cftuv.surface_ir import (
        AnalysisBundle,
        PatchSurfaceIR,
        SourceEdge,
        SourceFace,
        SourceRevision,
        SourceVertex,
        SurfaceTriangle,
    )

    edge_pairs = mesh_edges(fixture)
    edge_id_by_pair = {pair: 10_000 + index for index, pair in enumerate(edge_pairs)}
    edge_faces = {pair: [] for pair in edge_pairs}
    face_ids = tuple(20_000 + index for index in range(len(fixture.faces)))
    normals = []
    source_faces = []
    surface_triangles = []
    for triangle_id, face in enumerate(fixture.faces):
        ids = tuple(int(value) for value in face)
        points = fixture.vertices[list(ids)]
        normal = _normal(points)
        normals.append(normal)
        cycle_pairs = (
            tuple(sorted((ids[0], ids[1]))),
            tuple(sorted((ids[1], ids[2]))),
            tuple(sorted((ids[2], ids[0]))),
        )
        for pair in cycle_pairs:
            edge_faces[pair].append(face_ids[triangle_id])
        opposite_pairs = (cycle_pairs[1], cycle_pairs[2], cycle_pairs[0])
        surface_triangles.append(
            SurfaceTriangle(
                triangle_id=triangle_id,
                source_face_id=face_ids[triangle_id],
                vertex_ids=ids,
                physical_edge_ids=tuple(edge_id_by_pair[pair] for pair in opposite_pairs),
                triangle_normal=normal,
            )
        )
        source_faces.append(
            SourceFace(
                face_id=face_ids[triangle_id],
                patch_id=0,
                vertex_cycle=ids,
                edge_cycle=tuple(edge_id_by_pair[pair] for pair in cycle_pairs),
                polygon_normal=normal,
                triangle_ids=(triangle_id,),
            )
        )
    revision = SourceRevision("s-wf0", fixture.key)
    surface = PatchSurfaceIR(
        source_revision=revision,
        vertices=tuple(
            SourceVertex(index, tuple(float(value) for value in position))
            for index, position in enumerate(fixture.vertices)
        ),
        edges=tuple(
            SourceEdge(
                edge_id_by_pair[pair],
                pair,
                tuple(sorted(edge_faces[pair])),
            )
            for pair in edge_pairs
        ),
        faces=tuple(source_faces),
        triangles=tuple(surface_triangles),
    )
    chains = []
    seeds = []
    selected = []
    for chain_index, branch in enumerate(fixture.branches):
        ids = branch.vertex_ids
        pairs = list(zip(ids, ids[1:]))
        if branch.closed:
            pairs.append((ids[-1], ids[0]))
        chain_edge_ids = []
        side_faces = []
        side_normals = []
        for segment_index, oriented_pair in enumerate(pairs):
            pair = tuple(sorted(oriented_pair))
            edge_id = edge_id_by_pair[pair]
            owner_face = min(edge_faces[pair])
            owner_triangle = owner_face - 20_000
            chain_edge_ids.append(edge_id)
            side_faces.append(owner_face)
            side_normals.append(Vector(normals[owner_triangle]))
            seeds.append(
                ChartSiteSeed(
                    edge_index=edge_id,
                    source_vertex_ids=pair,
                    source_face_id=owner_face,
                    chain_ref=(0, 0, chain_index),
                )
            )
            selected.append(edge_id)
        chains.append(
            BoundaryChain(
                vert_indices=list(ids),
                vert_cos=[Vector(fixture.vertices[index]) for index in ids],
                edge_indices=chain_edge_ids,
                side_face_indices=side_faces,
                side_face_normals=side_normals,
                is_closed=branch.closed,
            )
        )
    node = PatchNode(
        patch_id=0,
        face_indices=list(face_ids),
        centroid=Vector(np.mean(fixture.vertices, axis=0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.boundary_loops = [BoundaryLoop(chains=chains)]
    graph = PatchGraph()
    graph.add_node(node)
    graph.source_revision = revision
    bundle = AnalysisBundle(revision, graph, surface)
    return bundle, node, tuple(seeds), tuple(sorted(set(selected))), edge_id_by_pair


def _project_to_segment(point, first, second):
    direction = np.asarray(second, dtype=float) - np.asarray(first, dtype=float)
    denominator = float(np.dot(direction, direction))
    if denominator <= 1e-20:
        return float(np.linalg.norm(np.asarray(point) - np.asarray(first))), 0.0
    factor = float(
        np.dot(np.asarray(point) - np.asarray(first), direction) / denominator
    )
    factor = max(0.0, min(1.0, factor))
    closest = np.asarray(first) + factor * direction
    return float(np.linalg.norm(np.asarray(point) - closest)), factor


def _extract_current_chart(fixture, charts):
    from cftuv.decal_chart_parametrization import parameterize_intrinsic_strip_width

    distance = np.full(len(fixture.vertices), np.inf, dtype=float)
    station = np.full(len(fixture.vertices), np.nan, dtype=float)
    owners = [""] * len(fixture.vertices)
    ambiguous_owner = np.zeros(len(fixture.vertices), dtype=bool)
    ambiguous_source = np.zeros(len(fixture.vertices), dtype=bool)
    metadata = _source_edge_metadata(fixture)
    fallbacks = []
    overlap_count = 0
    for chart in charts:
        overlap_count += int(chart.metrics.triangle_overlap_count)
        parameterized = None
        if len(fixture.branches) == 1 and not fixture.branches[0].closed:
            try:
                parameterized = parameterize_intrinsic_strip_width(chart)
            except Exception as exc:
                fallbacks.append(
                    {
                        "exception_type": type(exc).__name__,
                        "code": getattr(exc, "code", None),
                        "details": getattr(exc, "details", None),
                    }
                )
        if parameterized is not None:
            owner = fixture.branches[0].owner
            for triangle in parameterized.triangles:
                for vertex_id, point in zip(triangle.source_vertex_ids, triangle.chart_points):
                    candidate = abs(float(point[1]))
                    if candidate < distance[vertex_id]:
                        distance[vertex_id] = candidate
                        station[vertex_id] = float(point[0])
                        owners[vertex_id] = owner
            continue
        point_by_vertex = {}
        triangles_by_id = {triangle.triangle_id: triangle for triangle in chart.triangles}
        for triangle in chart.triangles:
            for vertex_id, point in zip(triangle.source_vertex_ids, triangle.chart_points):
                point_by_vertex.setdefault(vertex_id, point)
        sites = []
        for seed in chart.site_seeds:
            owner_candidates = [
                triangle
                for triangle in chart.triangles
                if seed.source_vertex_ids in triangle.source_edge_ids
                and triangle.source_face_id == seed.source_face_id
            ]
            if not owner_candidates:
                continue
            owner_triangle = min(owner_candidates, key=lambda value: value.triangle_id)
            mapping = dict(zip(owner_triangle.source_vertex_ids, owner_triangle.chart_points))
            info = metadata[seed.source_vertex_ids]
            oriented_first, oriented_second = info["oriented"]
            sites.append(
                (
                    mapping[oriented_first],
                    mapping[oriented_second],
                    info,
                )
            )
        tolerance = max(1e-10, fixture.alpha_reference * 1e-7)
        for vertex_id, point in point_by_vertex.items():
            candidates = []
            for first, second, info in sites:
                candidate_distance, factor = _project_to_segment(point, first, second)
                candidates.append(
                    (
                        candidate_distance,
                        info["owner"],
                        info["s0"] + factor * (info["s1"] - info["s0"]),
                    )
                )
            if not candidates:
                continue
            candidates.sort(key=lambda value: (value[0], value[1], value[2]))
            best = candidates[0]
            if best[0] < distance[vertex_id]:
                distance[vertex_id] = best[0]
                owners[vertex_id] = best[1]
                station[vertex_id] = best[2]
                if len(candidates) > 1:
                    ambiguous_source[vertex_id] = candidates[1][0] - best[0] <= tolerance
                    other_owner = next(
                        (value for value in candidates[1:] if value[1] != best[1]),
                        None,
                    )
                    ambiguous_owner[vertex_id] = (
                        other_owner is not None and other_owner[0] - best[0] <= tolerance
                    )
    return (
        distance,
        tuple(owners),
        station,
        ambiguous_owner,
        ambiguous_source,
        {"chart_overlap_count": overlap_count, "fallbacks": fallbacks},
    )


def run_current_field(fixture):
    _bootstrap_current_runtime()
    from cftuv.decal_charts import build_intrinsic_strip_charts, unroll_intrinsic_strip_chart
    from cftuv.decal_rails import compile_decal_rail_plan

    def build():
        bundle, node, seeds, selected, edge_id_by_pair = _build_current_input(fixture)
        rail_plan = compile_decal_rail_plan(
            bundle,
            selected,
            alpha_budget=fixture.alpha_reference,
        )
        charts = build_intrinsic_strip_charts(
            node,
            seeds,
            fixture.alpha_reference * 1.05,
            patch_surface=bundle.patch_surface,
        )
        charts = tuple(unroll_intrinsic_strip_chart(chart) for chart in charts)
        return bundle, rail_plan, charts, edge_id_by_pair

    try:
        compiled, compile_ms = _median_timed(build)
        bundle, rail_plan, charts, edge_id_by_pair = compiled
        extracted, extract_ms = _median_timed(
            lambda: _extract_current_chart(fixture, charts)
        )
        distance, owners, station, ambiguous_owner, ambiguous_source, diagnostics = extracted
        route_by_id = {route.route_id: route for route in rail_plan.routes}
        native_events = []
        for event in rail_plan.events:
            route = route_by_id.get(event.route_id)
            alpha = None
            if route is not None and 0 <= event.station_index < len(route.stations):
                alpha = float(route.stations[event.station_index].distance)
            native_events.append(
                {
                    "kind": event.kind.value,
                    "route_id": int(event.route_id),
                    "station_index": int(event.station_index),
                    "alpha": alpha,
                }
            )
        edge_pair_by_id = {value: key for key, value in edge_id_by_pair.items()}
        native_loci = []
        for locus in rail_plan.freeze_loci:
            if locus.source_vertex_id is not None:
                native_loci.append(f"v:{int(locus.source_vertex_id)}")
            elif locus.source_edge_id in edge_pair_by_id:
                first, second = edge_pair_by_id[locus.source_edge_id]
                native_loci.append(f"e:{first}-{second}")
        diagnostics.update(
            {
                "chart_count": len(charts),
                "rail_route_count": len(rail_plan.routes),
                "rail_event_count": len(rail_plan.events),
                "rail_freeze_count": len(rail_plan.freeze_loci),
                "rail_event_counts": dict(rail_plan.event_counts),
            }
        )
        return FieldResult(
            "CURRENT_RAIL_CHART",
            "ok",
            distance,
            owners,
            station,
            ambiguous_owner,
            ambiguous_source,
            compile_ms,
            extract_ms,
            diagnostics,
            tuple(native_events),
            tuple(sorted(set(native_loci))),
        )
    except Exception as exc:
        count = len(fixture.vertices)
        return FieldResult(
            "CURRENT_RAIL_CHART",
            "failed",
            np.full(count, np.inf),
            tuple("" for _ in range(count)),
            np.full(count, np.nan),
            np.zeros(count, dtype=bool),
            np.zeros(count, dtype=bool),
            0.0,
            0.0,
            {"failure": f"{type(exc).__name__}: {exc}"},
        )


def _neighbors(fixture):
    result = {index: set() for index in range(len(fixture.vertices))}
    for first, second in mesh_edges(fixture):
        result[first].add(second)
        result[second].add(first)
    return result


def extract_field_topology(fixture, result):
    tolerance = max(1e-10, fixture.alpha_reference * 1e-7)
    edges = mesh_edges(fixture)
    loci = set()
    pair_alpha = {}
    for first, second in edges:
        if not (np.isfinite(result.distances[first]) and np.isfinite(result.distances[second])):
            continue
        first_owner, second_owner = result.owners[first], result.owners[second]
        if first_owner and second_owner and first_owner != second_owner:
            key = tuple(sorted((first_owner, second_owner)))
            alpha = 0.5 * (result.distances[first] + result.distances[second])
            pair_alpha[key] = min(float(alpha), pair_alpha.get(key, math.inf))
            loci.add(f"e:{first}-{second}")
    neighbors = _neighbors(fixture)
    maxima = set()
    for vertex_id, value in enumerate(result.distances):
        if not np.isfinite(value) or value <= tolerance:
            continue
        neighbor_values = [result.distances[index] for index in neighbors[vertex_id]]
        finite = [candidate for candidate in neighbor_values if np.isfinite(candidate)]
        if finite and value + tolerance >= max(finite):
            maxima.add(vertex_id)
    max_edges = {
        (first, second)
        for first, second in edges
        if first in maxima
        and second in maxima
        and abs(result.distances[first] - result.distances[second]) <= tolerance
    }
    covered = {value for edge in max_edges for value in edge}
    for first, second in max_edges:
        loci.add(f"e:{first}-{second}")
    for vertex_id in sorted(maxima - covered):
        loci.add(f"v:{vertex_id}")
    events = [
        {"kind": f"MERGE:{first}|{second}", "alpha": alpha}
        for (first, second), alpha in pair_alpha.items()
    ]
    plateau_remaining = set(maxima)
    while plateau_remaining:
        seed = min(plateau_remaining)
        plateau_remaining.remove(seed)
        component = {seed}
        stack = [seed]
        while stack:
            current = stack.pop()
            for neighbor in neighbors[current]:
                if neighbor not in plateau_remaining:
                    continue
                if abs(result.distances[current] - result.distances[neighbor]) > tolerance:
                    continue
                plateau_remaining.remove(neighbor)
                component.add(neighbor)
                stack.append(neighbor)
        events.append(
            {
                "kind": "FREEZE",
                "alpha": float(np.mean(result.distances[list(component)])),
            }
        )
    boundary_vertices = {value for edge in boundary_edges(fixture) for value in edge}
    source_vertices = {record["vertex_id"] for record in source_records(fixture)[0]}
    candidates = [
        vertex_id
        for vertex_id in boundary_vertices
        if vertex_id not in source_vertices and np.isfinite(result.distances[vertex_id])
    ]
    if candidates:
        alpha = min(float(result.distances[index]) for index in candidates)
        if alpha > tolerance:
            events.append({"kind": "BOUNDARY", "alpha": alpha})
    events.sort(key=lambda value: (value["alpha"], value["kind"]))
    return {"events": tuple(events), "loci": tuple(sorted(loci))}


def _percentile(values, percentile):
    return float(np.percentile(values, percentile)) if len(values) else None


def _levenshtein(first, second):
    previous = list(range(len(second) + 1))
    for first_index, first_value in enumerate(first, 1):
        current = [first_index]
        for second_index, second_value in enumerate(second, 1):
            current.append(
                min(
                    current[-1] + 1,
                    previous[second_index] + 1,
                    previous[second_index - 1] + (first_value != second_value),
                )
            )
        previous = current
    return previous[-1]


def _locus_f1(actual, expected):
    actual, expected = set(actual), set(expected)
    if not actual and not expected:
        return 1.0
    if not actual or not expected:
        return 0.0
    intersection = len(actual.intersection(expected))
    precision = intersection / len(actual)
    recall = intersection / len(expected)
    return 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0


def compare_to_exact(fixture, result, exact, topology, exact_topology):
    query = (
        np.isfinite(exact.distances)
        & (exact.distances <= fixture.alpha_reference * 1.05)
    )
    valid_width = query & np.isfinite(result.distances)
    width_error = (
        np.abs(result.distances[valid_width] - exact.distances[valid_width])
        / fixture.alpha_reference
    )
    owner_valid = query & ~exact.ambiguous_owner
    owner_mismatch = np.asarray(
        [result.owners[index] != exact.owners[index] for index in range(len(query))],
        dtype=bool,
    )
    owner_error = (
        float(np.mean(owner_mismatch[owner_valid])) if np.any(owner_valid) else None
    )
    station_valid = (
        query
        & ~exact.ambiguous_source
        & np.isfinite(exact.source_s)
        & np.isfinite(result.source_s)
    )
    source_length = source_records(fixture)[1]
    station_error = (
        np.abs(result.source_s[station_valid] - exact.source_s[station_valid])
        / source_length
    )
    source_components = {
        component_index
        for component_index, component in enumerate(connected_components(fixture))
        if any(record["vertex_id"] in component for record in source_records(fixture)[0])
    }
    leak_count = 0
    for component_index, component in enumerate(connected_components(fixture)):
        if component_index in source_components:
            continue
        leak_count += int(np.sum(np.isfinite(result.distances[list(component)])))
    # Labeled witness ties are cut-locus evidence, not duplicate materialization.
    # Of the compared research backends only current chart IR materializes a
    # second geometric cover; scalar fields therefore report zero here.
    double_cover = int(result.diagnostics.get("chart_overlap_count", 0))
    actual_kinds = tuple(event["kind"] for event in topology["events"])
    exact_kinds = tuple(event["kind"] for event in exact_topology["events"])
    alpha_error = None
    if actual_kinds == exact_kinds and actual_kinds:
        alpha_error = max(
            abs(actual["alpha"] - expected["alpha"]) / fixture.alpha_reference
            for actual, expected in zip(topology["events"], exact_topology["events"])
        )
    return {
        "query_count": int(np.sum(query)),
        "coverage_rate": float(np.mean(valid_width[query])) if np.any(query) else None,
        "width_error_mean": float(np.mean(width_error)) if len(width_error) else None,
        "width_error_p95": _percentile(width_error, 95),
        "width_error_max": float(np.max(width_error)) if len(width_error) else None,
        "owner_error_rate": owner_error,
        "source_s_error_mean": float(np.mean(station_error)) if len(station_error) else None,
        "source_s_error_p95": _percentile(station_error, 95),
        "source_s_error_max": float(np.max(station_error)) if len(station_error) else None,
        "event_sequence_edit_distance": _levenshtein(actual_kinds, exact_kinds),
        "event_alpha_error_max": alpha_error,
        "cut_freeze_locus_f1": _locus_f1(topology["loci"], exact_topology["loci"]),
        "double_cover_count": double_cover,
        "boundary_leak_count": leak_count,
    }


def run_straight_skeleton(fixture):
    if not fixture.planar_polygon:
        return {"status": "unsupported", "reason": "NOT_A_SIMPLE_PLANAR_POLYGON_FRONT"}
    from py_straight_skeleton import compute_skeleton

    skeleton, compile_ms = _median_timed(
        lambda: compute_skeleton(exterior=list(fixture.planar_polygon), holes=[])
    )

    def extract():
        return tuple(
            {
                "node_id": int(node._skn_id),
                "x": float(node.position.x),
                "y": float(node.position.y),
                "alpha": float(node.time),
            }
            for node in skeleton.nodes
            if float(node.time) > 1e-10
        )

    nodes, extract_ms = _median_timed(extract)
    return {
        "status": "ok",
        "compile_ms": compile_ms,
        "extract_ms": extract_ms,
        "events": nodes,
    }


def _serialize_field(result, topology, metrics):
    def number(value):
        return float(value) if np.isfinite(value) else None

    return {
        "status": result.status,
        "compile_ms": result.compile_ms,
        "extract_ms": result.extract_ms,
        "metrics": metrics,
        "diagnostics": result.diagnostics,
        "native_events": list(result.native_events),
        "native_loci": list(result.native_loci),
        "field_events": list(topology["events"]),
        "field_loci": list(topology["loci"]),
        "samples": [
            {
                "vertex_id": index,
                "distance": number(result.distances[index]),
                "owner": result.owners[index] or None,
                "source_s": number(result.source_s[index]),
                "ambiguous_owner": bool(result.ambiguous_owner[index]),
                "ambiguous_source": bool(result.ambiguous_source[index]),
            }
            for index in range(len(result.distances))
        ],
    }


def run_fixture(fixture):
    exact = run_external_field(fixture, "MMP_EXACT")
    exact_topology = extract_field_topology(fixture, exact)
    results = {
        "MMP_EXACT": (exact, exact_topology),
        "CURRENT_RAIL_CHART": (
            current := run_current_field(fixture),
            extract_field_topology(fixture, current),
        ),
        "HEAT": (
            heat := run_external_field(fixture, "HEAT"),
            extract_field_topology(fixture, heat),
        ),
        "FMM": (
            fmm := run_external_field(fixture, "FMM"),
            extract_field_topology(fixture, fmm),
        ),
    }
    serialized = {}
    for method in METHODS:
        result, topology = results[method]
        metrics = compare_to_exact(
            fixture,
            result,
            exact,
            topology,
            exact_topology,
        )
        serialized[method] = _serialize_field(result, topology, metrics)
    return {
        "fixture": fixture.name,
        "variant": fixture.variant or None,
        "vertex_count": len(fixture.vertices),
        "triangle_count": len(fixture.faces),
        "alpha_reference": fixture.alpha_reference,
        "source_network_length": source_records(fixture)[1],
        "source_branch_count": len(fixture.branches),
        "component_count": len(connected_components(fixture)),
        "methods": serialized,
        "straight_skeleton_2d": run_straight_skeleton(fixture),
    }


def _field_arrays(run, method):
    samples = run["methods"][method]["samples"]
    distance = np.asarray(
        [math.inf if sample["distance"] is None else sample["distance"] for sample in samples]
    )
    station = np.asarray(
        [math.nan if sample["source_s"] is None else sample["source_s"] for sample in samples]
    )
    owners = tuple(sample["owner"] or "" for sample in samples)
    ambiguous = np.asarray([sample["ambiguous_source"] for sample in samples], dtype=bool)
    return distance, station, owners, ambiguous


def compare_retriangulation(runs):
    variants = {
        run["variant"]: run
        for run in runs
        if run["fixture"] == "retriangulated_surface"
    }
    if set(variants) != {"a", "b"}:
        return {"status": "missing_variants"}
    first, second = variants["a"], variants["b"]
    result = {"status": "ok", "methods": {}}
    for method in METHODS:
        da, sa, oa, aa = _field_arrays(first, method)
        db, sb, ob, ab = _field_arrays(second, method)
        finite = np.isfinite(da) & np.isfinite(db)
        alpha = first["alpha_reference"]
        delta = np.abs(da[finite] - db[finite]) / alpha
        station_valid = finite & np.isfinite(sa) & np.isfinite(sb) & ~aa & ~ab
        source_length = max(float(first["source_network_length"]), 1e-12)
        events_a = first["methods"][method]["field_events"]
        events_b = second["methods"][method]["field_events"]
        loci_a = first["methods"][method]["field_loci"]
        loci_b = second["methods"][method]["field_loci"]
        event_kinds_a = tuple(event["kind"] for event in events_a)
        event_kinds_b = tuple(event["kind"] for event in events_b)
        event_alpha_delta = None
        if event_kinds_a == event_kinds_b and events_a:
            event_alpha_delta = max(
                abs(a["alpha"] - b["alpha"]) / alpha
                for a, b in zip(events_a, events_b)
            )
        result["methods"][method] = {
            "common_finite_count": int(np.sum(finite)),
            "width_delta_p95": _percentile(delta, 95),
            "width_delta_max": float(np.max(delta)) if len(delta) else None,
            "owner_disagreement_rate": (
                float(np.mean([oa[index] != ob[index] for index in np.flatnonzero(finite)]))
                if np.any(finite)
                else None
            ),
            "source_s_delta_max": (
                float(
                    np.max(np.abs(sa[station_valid] - sb[station_valid]))
                    / source_length
                )
                if np.any(station_valid)
                else None
            ),
            "event_sequence_equal": event_kinds_a == event_kinds_b,
            "event_alpha_delta_max": event_alpha_delta,
            "locus_f1": _locus_f1(loci_a, loci_b),
        }
    return result


def run_all(fixtures):
    runs = []
    for fixture in fixtures:
        print(f"[S-WF0] {fixture.key}", flush=True)
        runs.append(run_fixture(fixture))
    return {
        "schema": SCHEMA,
        "timing_repeats": TIMING_REPEATS,
        "distance_reference": "MMP_EXACT_DISCRETE_SOURCE_VERTICES",
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "packages": {
                package: importlib.metadata.version(package)
                for package in (
                    "numpy",
                    "potpourri3d",
                    "pygeodesic",
                    "py_straight_skeleton",
                )
            },
        },
        "fixtures": runs,
        "retriangulation": compare_retriangulation(runs),
    }


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


__all__ = (
    "METHODS",
    "SCHEMA",
    "compare_retriangulation",
    "run_all",
    "run_external_field",
    "run_fixture",
    "write_json",
)
