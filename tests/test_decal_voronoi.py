from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import pytest
from math import cos, pi, sin, sqrt
from mathutils import Vector

pytest.importorskip("pyvoronoi")

import cftuv.decal_voronoi as decal_voronoi  # noqa: E402
from cftuv.decal_voronoi import (  # noqa: E402
    _merge_polygon_fragments,
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
)
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
)


@pytest.mark.parametrize(
    "error_type",
    (
        decal_voronoi.pyvoronoi.FocusOnDirectixException,
        decal_voronoi.pyvoronoi.UnsolvableParabolaEquation,
    ),
)
def test_invalid_boost_parabola_falls_back_to_shared_chord(error_type):
    class BrokenDiagram:
        def DiscretizeCurvedEdge(self, *_args):
            raise error_type("invalid curved edge")

    edges = [
        SimpleNamespace(start=0, end=1, is_linear=False),
    ]
    vertices = [
        SimpleNamespace(X=1.25, Y=-2.0),
        SimpleNamespace(X=3.75, Y=4.0),
    ]

    assert decal_voronoi._edge_points(
        BrokenDiagram(), edges, vertices, 0, 0.5
    ) == [(1.25, -2.0), (3.75, 4.0)]


def test_unrelated_curve_backend_error_is_not_hidden():
    class BrokenDiagram:
        def DiscretizeCurvedEdge(self, *_args):
            raise RuntimeError("unexpected backend failure")

    edges = [SimpleNamespace(start=0, end=1, is_linear=False)]
    vertices = [
        SimpleNamespace(X=0.0, Y=0.0),
        SimpleNamespace(X=1.0, Y=1.0),
    ]

    with pytest.raises(RuntimeError, match="unexpected backend failure"):
        decal_voronoi._edge_points(
            BrokenDiagram(), edges, vertices, 0, 0.5
        )


# Band-тесты A11/A12 включают экспериментальный путь явно: дефолт
# dynamic_corner_bands=False во всех entry points (стабильная A10-семантика).
_BAND_SETTINGS = decal_voronoi.CornerRuntimeSettings(dynamic_corner_bands=True)


def _planar_two_site_graph():
    node = PatchNode(
        patch_id=0,
        face_indices=[0, 1],
        centroid=Vector((2.0, 1.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((4.0, 0.0, 0.0)),
        Vector((4.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
    ]
    node.mesh_tris = [(0, 1, 2), (0, 2, 3)]
    left = BoundaryChain(
        vert_indices=[0, 3],
        vert_cos=[node.mesh_verts[0], node.mesh_verts[3]],
        edge_indices=[10],
    )
    right = BoundaryChain(
        vert_indices=[1, 2],
        vert_cos=[node.mesh_verts[1], node.mesh_verts[2]],
        edge_indices=[12],
    )
    node.boundary_loops = [BoundaryLoop(chains=[left, right])]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _scaled_planar_two_site_graph(scale, translation=None):
    graph = deepcopy(_planar_two_site_graph())
    node = graph.nodes[0]
    translation = translation or Vector((0.0, 0.0, 0.0))
    node.centroid = node.centroid * scale + translation
    node.mesh_verts = [
        point * scale + translation for point in node.mesh_verts
    ]
    for loop in node.boundary_loops:
        for chain in loop.chains:
            chain.vert_cos = [
                point * scale + translation for point in chain.vert_cos
            ]
    return graph


def _compiled_surface_topology(plan):
    surface = plan.surfaces[0]
    return (
        len(surface.sites),
        len(surface.corners),
        tuple(
            sorted(
                (
                    atom.site_index,
                    atom.cell_kind,
                    atom.source_category,
                    len(atom.fragments),
                )
                for atom in surface.atoms
            )
        ),
    )


def test_b2_triangle_grid_is_immutable_and_never_omits_aabb_candidates():
    triangles = []
    for grid_x in range(8):
        for grid_y in range(8):
            point_a = (float(grid_x), float(grid_y))
            point_b = (float(grid_x + 1), float(grid_y))
            point_c = (float(grid_x + 1), float(grid_y + 1))
            point_d = (float(grid_x), float(grid_y + 1))
            triangles.extend(
                (
                    (point_a, point_b, point_c),
                    (point_a, point_c, point_d),
                )
            )
    triangles = tuple(triangles)
    grid = decal_voronoi._build_triangle_aabb_grid(triangles)
    bounds = (2.2, 3.2, 2.8, 3.8)
    candidates = grid.query_aabb(*bounds)
    exact = tuple(
        triangle_id
        for triangle_id, triangle in enumerate(triangles)
        if not (
            max(point[0] for point in triangle) < bounds[0]
            or min(point[0] for point in triangle) > bounds[2]
            or max(point[1] for point in triangle) < bounds[1]
            or min(point[1] for point in triangle) > bounds[3]
        )
    )

    assert set(exact).issubset(candidates)
    assert len(candidates) < len(triangles) // 4
    with pytest.raises(TypeError):
        grid.cell_keys[0] = (100, 100)


def test_b2_compiled_relation_indices_match_reference_scans():
    plan = compile_patch_voronoi_plan(
        _planar_two_site_graph(), [10, 12], offset=0.01
    )
    surface = plan.surfaces[0]

    for site_index in range(len(surface.sites)):
        assert surface.atoms_by_site.get(site_index, ()) == tuple(
            atom_index
            for atom_index, atom in enumerate(surface.atoms)
            if atom.site_index == site_index
        )
        assert surface.corners_by_site.get(site_index, ()) == tuple(
            corner_index
            for corner_index, corner in enumerate(surface.corners)
            if site_index in corner.incident_sites
        )
        assert tuple(
            port.vert_index for port in surface.ports_by_site[site_index]
        ) == (
            surface.sites[site_index].vert_a,
            surface.sites[site_index].vert_b,
        )
    for corner_index, corner in enumerate(surface.corners):
        assert surface.owner_atoms_by_corner[corner_index] == tuple(
            atom_index
            for atom_index, atom in enumerate(surface.atoms)
            if (
                atom.cell_kind == "SEGMENT"
                and atom.site_index in corner.incident_sites
            )
            or (
                atom.cell_kind == "POINT"
                and atom.corner_index == corner_index
            )
        )
    for vert_index, site_indices in surface.sites_by_vertex.items():
        assert site_indices == tuple(
            site_index
            for site_index, site in enumerate(surface.sites)
            if vert_index in (site.vert_a, site.vert_b)
        )
        assert tuple(
            port.site_index for port in surface.ports_by_vertex[vert_index]
        ) == site_indices


def test_b4_full_component_records_requested_budget_without_false_limit():
    plan = compile_patch_voronoi_plan(
        _planar_two_site_graph(),
        [10, 12],
        offset=0.01,
        alpha_budget=0.25,
    )

    assert plan.requested_alpha_budget == pytest.approx(0.25)
    assert plan.alpha_budget == float("inf")
    assert plan.budget_source == "FULL_CONNECTED_COMPONENT"
    assert plan.active_triangle_ids(1000.0) == tuple(
        tuple(range(len(surface.domain.boundary_triangles)))
        for surface in plan.surfaces
    )


def test_b4_finite_strip_budget_rejects_before_evaluation():
    plan = compile_patch_voronoi_plan(
        _planar_two_site_graph(), [10, 12], offset=0.01
    )
    finite = replace(
        plan,
        alpha_budget=0.25,
        budget_source="STRIP_BUDGET",
        requested_alpha_budget=0.25,
    )

    assert finite.active_triangle_ids(0.25) == plan.support_triangle_ids
    with pytest.raises(
        decal_voronoi.DomainBudgetExceeded,
        match="DOMAIN_BUDGET_EXCEEDED",
    ):
        evaluate_patch_voronoi_plan(
            finite, width=0.5001, preview=True
        )


@pytest.mark.parametrize("width", (0.5, 1.0545852, 2.0, 4.0))
def test_b2_indices_are_byte_identical_to_reference_full_scans(width):
    graph, edge_indices = _wide_t_junction_front_graph()
    indexed_diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    reference_diagnostics = decal_voronoi.PatchVoronoiDiagnostics(
        reference_full_scan=True
    )
    indexed_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.02, diagnostics=indexed_diagnostics
    )
    reference_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.02, diagnostics=reference_diagnostics
    )

    indexed_faces = evaluate_patch_voronoi_plan(
        indexed_plan,
        width=width,
        preview=True,
        diagnostics=indexed_diagnostics,
    )
    reference_faces = evaluate_patch_voronoi_plan(
        reference_plan,
        width=width,
        preview=True,
        diagnostics=reference_diagnostics,
    )
    assert decal_voronoi.serialize_network_faces(
        indexed_faces
    ) == decal_voronoi.serialize_network_faces(reference_faces)


def test_adaptive_quantization_preserves_scaled_micro_sites():
    plan = compile_patch_voronoi_plan(
        _scaled_planar_two_site_graph(1.0e-4),
        [10, 12],
        offset=1.0e-6,
    )
    surface = plan.surfaces[0]

    assert all(
        site.segment_length > surface.diagram_transform.quantum
        for site in surface.sites
    )


def test_quantized_duplicate_site_is_localized_as_compile_failure():
    graph = _planar_two_site_graph()
    chain = graph.nodes[0].boundary_loops[0].chains[0]
    chain.vert_cos[1] = Vector((0.0, 0.00005, 0.0))

    attempt = decal_voronoi.compile_patch_voronoi_attempt(
        graph, [10], offset=0.01, allow_partial=True
    )

    assert attempt.plan is None
    assert attempt.rejected_edge_indices == (10,)
    assert len(attempt.failures) == 1
    failure = attempt.failures[0]
    assert failure.reason == "QUANTIZED_DEGENERATE_SITE"
    assert failure.edge_indices == (10,)
    assert "raw_length=" in failure.details
    assert "quantized_length=0" in failure.details
    assert "quantum=" in failure.details


def test_diagram_transform_is_translation_invariant():
    base = decal_voronoi._build_diagram_transform(
        ((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0))
    )
    shift = (1.0e12, -2.0e12)
    translated = decal_voronoi._build_diagram_transform(
        tuple(
            (point[0] + shift[0], point[1] + shift[1])
            for point in ((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0))
        )
    )

    assert translated.scale == base.scale
    assert translated.quantum == base.quantum
    assert translated.center == pytest.approx(
        (base.center[0] + shift[0], base.center[1] + shift[1])
    )
    assert translated.to_diagram((shift[0] + 4.0, shift[1] + 2.0)) \
        == pytest.approx(base.to_diagram((4.0, 2.0)))


def test_diagram_topology_is_equivalent_across_scale_and_translation():
    plans = [
        compile_patch_voronoi_plan(
            _scaled_planar_two_site_graph(scale),
            [10, 12],
            offset=0.01 * scale,
        )
        for scale in (1.0e-4, 1.0, 1.0e4)
    ]
    signatures = [_compiled_surface_topology(plan) for plan in plans]
    assert signatures[0] == signatures[1] == signatures[2]
    assert plans[0].surfaces[0].diagram_transform.scale \
        > plans[1].surfaces[0].diagram_transform.scale \
        > plans[2].surfaces[0].diagram_transform.scale

    translated = compile_patch_voronoi_plan(
        _scaled_planar_two_site_graph(
            1.0, Vector((10000.0, -20000.0, 3000.0))
        ),
        [10, 12],
        offset=0.01,
    )
    assert _compiled_surface_topology(translated) == signatures[1]


def test_long_chart_stays_inside_diagram_integer_safety_range():
    plan = compile_patch_voronoi_plan(
        _scaled_planar_two_site_graph(25000.0),
        [10, 12],
        offset=250.0,
    )
    transform = plan.surfaces[0].diagram_transform

    assert transform.max_abs_input * transform.scale <= (
        decal_voronoi._DIAGRAM_INT_LIMIT * transform.int_safety_margin
    )


def test_unsupported_dynamic_range_is_localized_before_construct():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    attempt = decal_voronoi.compile_patch_voronoi_attempt(
        _scaled_planar_two_site_graph(250000.0),
        [10, 12],
        offset=2500.0,
        allow_partial=True,
        diagnostics=diagnostics,
    )

    assert attempt.plan is None
    assert attempt.rejected_edge_indices == (10, 12)
    assert [failure.reason for failure in attempt.failures] == [
        "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED"
    ]
    assert diagnostics.construct_calls == 0


def test_corner_offset_lines_reject_zero_length_site():
    surface = SimpleNamespace(
        sites=(
            SimpleNamespace(
                segment_length=0.0,
                vert_a=1,
                point_a=(0.0, 0.0),
                point_b=(0.0, 0.0),
                inward_normal=(0.0, 1.0),
            ),
        )
    )
    corner = SimpleNamespace(
        vert_index=1,
        point=(0.0, 0.0),
        ordered_sites=(0,),
    )

    assert decal_voronoi._corner_offset_lines(surface, corner, 0.5) == []


def test_cell_triangulation_accepts_only_zero_area_collinear_remainder():
    # Реальная segment-cell walls.010: ненулевые ears покрывают весь contour,
    # после чего остаётся четыре коллинеарные stations. Это не compile failure.
    polygon = [
        (-2.508528, 4.30705),
        (-2.508529, 4.3071),
        (-76.00318, 4.3071),
        (-76.00318, 1.7071),
        (-2.508529, 1.7071),
        (-2.5088, 1.7428),
        (-2.5088, 2.9713),
        (-2.508528, 3.00705),
        (-2.5088, 3.0428),
        (-2.5088, 4.2713),
    ]

    triangles = decal_voronoi._triangulate_cell_polygon(polygon)

    assert triangles
    assert sum(
        decal_voronoi._polygon_area2(triangle) for triangle in triangles
    ) == pytest.approx(
        decal_voronoi._polygon_area2(polygon),
        abs=1e-7,
    )


def test_network_face_serializer_is_stable_across_face_order():
    first = decal_voronoi._NetworkFace(
        surface_id=2,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("b", 2), ("a", 1), ("c", 3)],
        positions=[
            Vector((1.000000001, 0.0, 0.0)),
            Vector((0.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[1.0, -1.0, 0.0],
        v_lengths=[2.0, 1.0, 3.0],
        component_kind="SEGMENT",
        component_side="",
    )
    second = decal_voronoi._NetworkFace(
        surface_id=1,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("d", 4), ("e", 5), ("f", 6)],
        positions=[
            Vector((2.0, 0.0, 0.0)),
            Vector((3.0, 0.0, 0.0)),
            Vector((2.0, 1.0, 0.0)),
        ],
        u_fracs=[-1.0, 1.0, 0.0],
        v_lengths=[0.0, 0.0, 1.0],
        component_kind="KITE",
        component_side="OUTER",
    )

    forward = decal_voronoi.serialize_network_faces([first, second])
    reversed_faces = decal_voronoi.serialize_network_faces([second, first])

    assert forward == reversed_faces
    assert forward["topology_signature"] == {
        "vertex_count": 6,
        "edge_count": 6,
        "face_count": 2,
        "face_loops": ((3, 4, 5), (0, 2, 1)),
    }
    assert forward["faces"][1]["positions"][2] == (1.0, 0.0, 0.0)


def _face_area_xy(face):
    area = 0.0
    points = face.positions
    for index, point in enumerate(points):
        other = points[(index + 1) % len(points)]
        area += point.x * other.y - other.x * point.y
    return abs(area) * 0.5


def _signed_face_area(face):
    area_vector = Vector((0.0, 0.0, 0.0))
    origin = face.positions[0]
    for index in range(1, len(face.positions) - 1):
        area_vector += (face.positions[index] - origin).cross(
            face.positions[index + 1] - origin
        )
    return area_vector.dot(face.surface_normal) * 0.5


def _orthogonal_corner_graph():
    graph = PatchGraph()
    patch_specs = (
        (
            0,
            Vector((0.0, 0.0, 1.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 1.0, 0.0)),
                Vector((0.0, 1.0, 0.0)),
            ],
        ),
        (
            1,
            Vector((0.0, -1.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 1.0)),
                Vector((0.0, 0.0, 1.0)),
            ],
        ),
    )
    for patch_id, normal, basis_u, basis_v, verts in patch_specs:
        node = PatchNode(
            patch_id=patch_id,
            face_indices=[patch_id],
            centroid=sum(verts, Vector((0.0, 0.0, 0.0))) / 4.0,
            normal=normal,
            basis_u=basis_u,
            basis_v=basis_v,
        )
        node.mesh_verts = verts
        node.mesh_tris = [(0, 1, 2), (0, 2, 3)]
        node.boundary_loops = [
            BoundaryLoop(
                chains=[
                    BoundaryChain(
                        vert_indices=[0, 1],
                        vert_cos=[verts[0], verts[1]],
                        edge_indices=[30],
                    )
                ]
            )
        ]
        graph.add_node(node)
    return graph


def _planar_unselected_fold_graph():
    """Две PLANAR-полосы встречаются на общем невыбранном edge 99."""

    graph = PatchGraph()
    patch_specs = (
        (
            20,
            Vector((0.0, 0.0, 1.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 0.5, 0.0)),
                Vector((1.0, 0.5, 0.0)),
                Vector((0.0, 0.5, 0.0)),
            ],
            [0, 1, 2, 6, 3],
            [99, 101, 12, 10, 102],
            [3, 6, 2],
            [4, 3, 2],
            [10, 12],
        ),
        (
            21,
            Vector((0.0, -1.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((0.0, 0.0, 0.5)),
                Vector((1.0, 0.0, 0.5)),
                Vector((2.0, 0.0, 0.5)),
                Vector((2.0, 0.0, 0.0)),
            ],
            [0, 4, 7, 5, 1],
            [103, 11, 13, 104, 99],
            [4, 7, 5],
            [1, 2, 3],
            [11, 13],
        ),
    )
    for (
        patch_id,
        normal,
        basis_u,
        basis_v,
        points,
        loop_vertices,
        loop_edges,
        chain_vertices,
        chain_point_indices,
        chain_edges,
    ) in patch_specs:
        node = PatchNode(
            patch_id=patch_id,
            face_indices=[patch_id],
            centroid=sum(points, Vector()) / len(points),
            normal=normal,
            basis_u=basis_u,
            basis_v=basis_v,
            mesh_verts=points,
            mesh_tris=[(0, 1, 2), (0, 2, 3), (0, 3, 4)],
        )
        node.boundary_loops = [
            BoundaryLoop(
                vert_indices=loop_vertices,
                vert_cos=points,
                edge_indices=loop_edges,
                chains=[
                    BoundaryChain(
                        vert_indices=chain_vertices,
                        vert_cos=[points[index] for index in chain_point_indices],
                        edge_indices=chain_edges,
                    )
                ],
            )
        ]
        graph.add_node(node)
    return graph, (10, 11, 12, 13)


def _single_patch_fold_graph():
    """Один topology patch с двумя реальными owner-плоскостями."""

    verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0, 0.0, 0.0)),
        Vector((2.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
        Vector((2.0, 0.0, 1.0)),
        Vector((0.0, 0.0, 1.0)),
    ]
    node = PatchNode(
        patch_id=7,
        face_indices=[0, 1, 2, 3],
        centroid=sum(verts, Vector((0.0, 0.0, 0.0))) / len(verts),
        normal=Vector((0.0, -1.0, 1.0)).normalized(),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = verts
    node.mesh_vert_indices = list(range(len(verts)))
    node.mesh_tris = [
        (0, 1, 2),
        (0, 2, 3),
        (0, 4, 1),
        (0, 5, 4),
    ]
    node.mesh_tri_face_indices = [100, 100, 101, 101]
    node.mesh_tri_face_normals = [
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, -1.0, 0.0)),
        Vector((0.0, -1.0, 0.0)),
    ]
    node.mesh_tri_edge_indices = [
        (-1, -1, -1),
        (70, -1, -1),
        (-1, -1, -1),
        (71, -1, -1),
    ]
    front = BoundaryChain(
        vert_indices=[3, 2],
        vert_cos=[verts[3], verts[2]],
        edge_indices=[70],
        side_face_indices=[100],
        side_face_normals=[Vector((0.0, 0.0, 1.0))],
    )
    folded = BoundaryChain(
        vert_indices=[5, 4],
        vert_cos=[verts[5], verts[4]],
        edge_indices=[71],
        side_face_indices=[101],
        side_face_normals=[Vector((0.0, -1.0, 0.0))],
    )
    node.boundary_loops = [BoundaryLoop(chains=[front, folded])]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _folded_turn_graph():
    """Два selected edges поворачивают через три owner patches."""

    graph = PatchGraph()
    patch_specs = (
        (
            0,
            Vector((0.0, 0.0, 1.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 2.0, 0.0)),
                Vector((0.0, 2.0, 0.0)),
            ],
            (
                ([0, 1], [0, 1], [30]),
                ([0, 3], [0, 3], [31]),
            ),
        ),
        (
            1,
            Vector((0.0, -1.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 2.0)),
                Vector((0.0, 0.0, 2.0)),
            ],
            (([0, 1], [0, 1], [30]),),
        ),
        (
            2,
            Vector((-1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((0.0, 2.0, 0.0)),
                Vector((0.0, 2.0, 2.0)),
                Vector((0.0, 0.0, 2.0)),
            ],
            (([0, 3], [0, 1], [31]),),
        ),
    )
    for patch_id, normal, basis_u, basis_v, verts, chains in patch_specs:
        node = PatchNode(
            patch_id=patch_id,
            face_indices=[patch_id],
            centroid=sum(verts, Vector((0.0, 0.0, 0.0))) / 4.0,
            normal=normal,
            basis_u=basis_u,
            basis_v=basis_v,
        )
        node.mesh_verts = verts
        node.mesh_tris = [(0, 1, 2), (0, 2, 3)]
        node.boundary_loops = [
            BoundaryLoop(
                chains=[
                    BoundaryChain(
                        vert_indices=vert_indices,
                        vert_cos=[verts[index] for index in point_indices],
                        edge_indices=edge_indices,
                    )
                    for vert_indices, point_indices, edge_indices in chains
                ]
            )
        ]
        graph.add_node(node)
    return graph


def _acute_corner_graph():
    """Planar convex apex с intrinsic углом 30 градусов."""

    angle = pi / 12.0
    points = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0 * cos(angle), -2.0 * sin(angle), 0.0)),
        Vector((2.0 * cos(angle), 2.0 * sin(angle), 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / 3.0,
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [(0, 1, 2)]
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[0, 1],
                    vert_cos=[points[0], points[1]],
                    edge_indices=[40],
                ),
                BoundaryChain(
                    vert_indices=[2, 0],
                    vert_cos=[points[2], points[0]],
                    edge_indices=[41],
                ),
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _acute_notch_graph():
    """Reflex patch corner 330°, extrusion wedge 30°."""

    shoulder = 2.0 * sin(pi / 12.0) / cos(pi / 12.0)
    points = [
        Vector((-3.0, -2.0, 0.0)),
        Vector((3.0, -2.0, 0.0)),
        Vector((3.0, 2.0, 0.0)),
        Vector((shoulder, 2.0, 0.0)),
        Vector((0.0, 0.0, 0.0)),
        Vector((-shoulder, 2.0, 0.0)),
        Vector((-3.0, 2.0, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((0.0, 0.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 4),
        (1, 2, 3),
        (1, 3, 4),
        (0, 4, 5),
        (0, 5, 6),
    ]
    all_edges = list(range(50, 57))
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(7)),
            vert_cos=[point.copy() for point in points],
            edge_indices=all_edges,
            chains=[
                BoundaryChain(
                    vert_indices=[3, 4],
                    vert_cos=[points[3], points[4]],
                    edge_indices=[53],
                ),
                BoundaryChain(
                    vert_indices=[4, 5],
                    vert_cos=[points[4], points[5]],
                    edge_indices=[54],
                ),
            ],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _door_opening_graph():
    """Один planar patch с concave дверным проёмом без owner diagonals."""

    points = [
        Vector((-4.0, 0.0, 0.0)),
        Vector((-1.0, 0.0, 0.0)),
        Vector((-1.0, 3.0, 0.0)),
        Vector((1.0, 3.0, 0.0)),
        Vector((1.0, 0.0, 0.0)),
        Vector((4.0, 0.0, 0.0)),
        Vector((4.0, 4.0, 0.0)),
        Vector((-4.0, 4.0, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((0.0, 2.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 7),
        (1, 2, 7),
        (2, 3, 7),
        (3, 6, 7),
        (3, 4, 6),
        (4, 5, 6),
    ]
    edge_indices = list(range(100, 108))
    chain = BoundaryChain(
        vert_indices=list(range(8)),
        vert_cos=[point.copy() for point in points],
        edge_indices=edge_indices,
        is_closed=True,
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(8)),
            vert_cos=[point.copy() for point in points],
            edge_indices=edge_indices,
            chains=[chain],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_indices


def _wide_t_junction_front_graph():
    """Точный planar front patch walls.006 вокруг широкого T-junction."""

    points = [
        Vector((-37.679363, 15.175671, 0.0)),
        Vector((-37.679363, 19.733892, 0.0)),
        Vector((-25.643166, 19.733900, 0.0)),
        Vector((-25.643166, 15.175671, 0.0)),
        Vector((-30.796854, 15.175671, 0.0)),
        Vector((-30.796854, 18.608587, 0.0)),
        Vector((-32.621742, 18.608587, 0.0)),
        Vector((-32.621742, 15.175671, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 6),
        (0, 6, 7),
        (1, 2, 5),
        (1, 5, 6),
        (2, 3, 4),
        (2, 4, 5),
    ]
    edge_indices = list(range(200, 208))
    chain = BoundaryChain(
        vert_indices=list(range(8)),
        vert_cos=[point.copy() for point in points],
        edge_indices=edge_indices,
        is_closed=True,
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(8)),
            vert_cos=[point.copy() for point in points],
            edge_indices=edge_indices,
            chains=[chain],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_indices


def _smooth_arc_boundary_graph(segment_count=12):
    """Convex planar boundary arc с поворотом 5 градусов на вершину."""

    radius = 10.0
    arc = [
        Vector(
            (
                radius * sin((-30.0 + 60.0 * index / segment_count) * pi / 180.0),
                radius * cos((-30.0 + 60.0 * index / segment_count) * pi / 180.0),
                0.0,
            )
        )
        for index in range(segment_count + 1)
    ]
    points = [Vector((-5.0, 0.0, 0.0)), Vector((5.0, 0.0, 0.0))]
    points.extend(reversed(arc))
    edge_indices = [700 + index for index in range(len(points))]
    arc_vertex_indices = list(range(2, len(points)))
    arc_edge_indices = edge_indices[2:-1]
    node = PatchNode(
        patch_id=70,
        face_indices=[70],
        centroid=sum(points, Vector()) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        mesh_verts=points,
        mesh_vert_indices=list(range(len(points))),
        mesh_tris=[
            (0, index, index + 1)
            for index in range(1, len(points) - 1)
        ],
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(len(points))),
            vert_cos=[point.copy() for point in points],
            edge_indices=edge_indices,
            chains=[
                BoundaryChain(
                    vert_indices=arc_vertex_indices,
                    vert_cos=[points[index] for index in arc_vertex_indices],
                    edge_indices=arc_edge_indices,
                )
            ],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, arc_edge_indices


def test_width_drag_does_not_reconstruct_voronoi_diagram():
    graph, edge_indices = _door_opening_graph()
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        graph,
        edge_indices,
        offset=0.01,
        diagnostics=diagnostics,
    )
    compile_construct_calls = diagnostics.construct_calls

    snapshots = []
    for width in (2.0, 3.0, 3.7076, 4.5):
        faces = evaluate_patch_voronoi_plan(
            plan,
            width=width,
            preview=True,
            diagnostics=diagnostics,
        )
        snapshots.append(decal_voronoi.serialize_network_faces(faces))

    assert compile_construct_calls > 0
    assert diagnostics.construct_calls == compile_construct_calls
    assert all(snapshot["faces"] for snapshot in snapshots)


def test_repeated_preview_and_confirm_serialization_is_deterministic():
    graph, edge_indices = _door_opening_graph()
    first_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.01
    )
    second_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.01
    )

    first = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            first_plan, width=3.7076, preview=True
        )
    )
    repeated = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            second_plan, width=3.7076, preview=True
        )
    )
    confirmed = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            first_plan, width=3.7076, preview=False
        )
    )

    assert first == repeated == confirmed


def test_patch_voronoi_front_rebuilds_and_never_leaves_patch():
    plan = compile_patch_voronoi_plan(
        _planar_two_site_graph(), [10, 12], offset=0.01
    )
    assert plan is not None
    assert len(plan.surfaces) == 1
    assert len(plan.surfaces[0].sites) == 2

    narrow = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    wide = evaluate_patch_voronoi_plan(plan, width=4.0, preview=True)
    assert narrow and wide
    assert sum(_face_area_xy(face) for face in narrow) == pytest.approx(
        1.0, abs=1e-5
    )
    assert sum(_face_area_xy(face) for face in wide) == pytest.approx(
        8.0, abs=1e-5
    )
    for face in narrow + wide:
        for point in face.positions:
            assert -1e-7 <= point.x <= 4.0 + 1e-7
            assert -1e-7 <= point.y <= 2.0 + 1e-7
            assert point.z == pytest.approx(0.01)

    # Две source triangles остаются вычислительной деталью: каждая
    # Voronoi-cell материализуется одним цельным quad, а не двумя tris.
    assert len(wide) == 2
    assert all(len(face.positions) == 4 for face in wide)


def test_door_opening_builds_realtime_endpoint_miters_with_shared_keys():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    assert plan is not None
    surface = plan.surfaces[0]
    point_atoms = [atom for atom in surface.atoms if atom.cell_kind == "POINT"]
    assert len(point_atoms) == 2
    assert {
        surface.corners[atom.corner_index].vert_index for atom in point_atoms
    } == {2, 3}

    narrow = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    wide = evaluate_patch_voronoi_plan(plan, width=2.0, preview=True)
    assert narrow and wide
    assert [tuple(face.positions) for face in wide] != [
        tuple(face.positions) for face in narrow
    ]
    for faces in (narrow, wide):
        for corner_vertex in (2, 3):
            corner_key = ("pv-sv", corner_vertex)
            owners = [face for face in faces if corner_key in face.vert_keys]
            # Endpoint-cell и обе соседние segment-cells используют одну
            # identity до финального remove-doubles.
            assert len(owners) >= 3
        for face in faces:
            for point in face.positions:
                assert -4.0 - 1e-7 <= point.x <= 4.0 + 1e-7
                assert -1e-7 <= point.y <= 4.0 + 1e-7

    def signature(faces):
        return [
            (
                tuple(face.vert_keys),
                tuple(tuple(round(value, 9) for value in point) for point in face.positions),
            )
            for face in faces
        ]

    assert signature(wide) == signature(
        evaluate_patch_voronoi_plan(plan, width=2.0, preview=False)
    )


def test_corner_spec_classifies_intrinsic_convex_concave_and_acute():
    door_graph, door_edges = _door_opening_graph()
    door_plan = compile_patch_voronoi_plan(
        door_graph, door_edges, offset=0.01
    )
    door_corners = {
        corner.vert_index: corner for corner in door_plan.surfaces[0].corners
    }
    for vert_index in (2, 3):
        corner = door_corners[vert_index]
        assert not corner.is_convex
        assert corner.interior_angle == pytest.approx(1.5 * pi)
        assert corner.extrusion_angle == pytest.approx(0.5 * pi)
        assert (
            decal_voronoi.classify_corner_runtime(corner, _BAND_SETTINGS)
            == decal_voronoi._CornerPolicy.KITE
        )
        assert len(corner.ordered_sites) == 2
        assert corner.turn_sign in (-1.0, 1.0)

    folded_plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    folded_corner = next(
        corner
        for corner in folded_plan.surfaces[0].corners
        if corner.vert_index == 0
    )
    assert folded_corner.is_convex
    assert folded_corner.interior_angle == pytest.approx(0.5 * pi)
    assert folded_corner.extrusion_angle == pytest.approx(0.5 * pi)
    assert (
        decal_voronoi.classify_corner_runtime(folded_corner, _BAND_SETTINGS)
        == decal_voronoi._CornerPolicy.KITE
    )

    acute_plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01
    )
    acute_corner = next(
        corner
        for corner in acute_plan.surfaces[0].corners
        if corner.vert_index == 0
    )
    assert acute_corner.is_convex
    assert acute_corner.interior_angle == pytest.approx(pi / 6.0)
    assert acute_corner.extrusion_angle == pytest.approx(pi / 6.0)
    assert (
        decal_voronoi.classify_corner_runtime(acute_corner, _BAND_SETTINGS)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )

    notch_plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    notch_corner = next(
        corner
        for corner in notch_plan.surfaces[0].corners
        if corner.vert_index == 4
    )
    assert not notch_corner.is_convex
    assert notch_corner.interior_angle == pytest.approx(11.0 * pi / 6.0)
    assert notch_corner.extrusion_angle == pytest.approx(pi / 6.0)
    assert (
        decal_voronoi.classify_corner_runtime(notch_corner)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )


@pytest.mark.parametrize(
    ("angle_degrees", "expected"),
    (
        (170, decal_voronoi._CornerPolicy.MITER),
        (130, decal_voronoi._CornerPolicy.MITER),
        (100, decal_voronoi._CornerPolicy.KITE),
        (75, decal_voronoi._CornerPolicy.FAN),
        (45, decal_voronoi._CornerPolicy.ACUTE_SPLIT),
        (20, decal_voronoi._CornerPolicy.HAIRPIN),
    ),
)
def test_a11_oracle_angle_table(angle_degrees, expected):
    theta = angle_degrees * pi / 180.0
    corner = decal_voronoi.CornerSpec(
        vert_index=0,
        point=(0.0, 0.0),
        incident_sites=(0, 1),
        ordered_sites=(0, 1),
        turn_sign=1.0,
        interior_angle=theta,
        extrusion_angle=theta,
        is_convex=True,
        miter_ratio=1.0,
    )

    assert decal_voronoi.classify_corner_runtime(corner, _BAND_SETTINGS) == expected


def _smooth_corner_surface(turn_degrees=5.0):
    turn = turn_degrees * pi / 180.0
    sites = (
        decal_voronoi._PatchVoronoiSite(
            patch_id=0,
            edge_index=600,
            vert_a=10,
            vert_b=11,
            source_a=Vector((-2.0, 0.0, 0.0)),
            source_b=Vector((0.0, 0.0, 0.0)),
            point_a=(-2.0, 0.0),
            point_b=(0.0, 0.0),
            arc_start=0.0,
            segment_length=2.0,
            uv_sign=1.0,
            inward_normal=(0.0, 1.0),
            two_sided=True,
        ),
        decal_voronoi._PatchVoronoiSite(
            patch_id=0,
            edge_index=601,
            vert_a=11,
            vert_b=12,
            source_a=Vector((0.0, 0.0, 0.0)),
            source_b=Vector((2.0 * cos(turn), 2.0 * sin(turn), 0.0)),
            point_a=(0.0, 0.0),
            point_b=(2.0 * cos(turn), 2.0 * sin(turn)),
            arc_start=2.0,
            segment_length=2.0,
            uv_sign=1.0,
            inward_normal=(-sin(turn), cos(turn)),
            two_sided=True,
        ),
    )
    corner = decal_voronoi.CornerSpec(
        vert_index=11,
        point=(0.0, 0.0),
        incident_sites=(0, 1),
        ordered_sites=(0, 1),
        turn_sign=1.0,
        interior_angle=pi - turn,
        extrusion_angle=pi - turn,
        is_convex=True,
        miter_ratio=1.0,
    )
    return SimpleNamespace(sites=sites), corner


def test_c8_3_smooth_is_stable_angle_only_policy():
    _surface, smooth_corner = _smooth_corner_surface(5.0)
    _surface, miter_corner = _smooth_corner_surface(11.0)
    stable = decal_voronoi.CornerRuntimeSettings(
        dynamic_corner_bands=False
    )

    assert (
        decal_voronoi.classify_corner_runtime(smooth_corner, stable)
        == decal_voronoi._CornerPolicy.SMOOTH
    )
    assert (
        decal_voronoi.classify_corner_runtime(miter_corner, stable)
        == decal_voronoi._CornerPolicy.MITER
    )
    # Экспериментальная band-грамматика остаётся differential-неизменной.
    assert (
        decal_voronoi.classify_corner_runtime(
            smooth_corner, _BAND_SETTINGS
        )
        == decal_voronoi._CornerPolicy.MITER
    )
    exact_surface = SimpleNamespace(
        domain=SimpleNamespace(admission_tier="EXACT")
    )
    approximate_surface = SimpleNamespace(
        domain=SimpleNamespace(admission_tier="APPROXIMATE")
    )
    assert (
        decal_voronoi._classify_surface_corner_runtime(
            exact_surface, smooth_corner, stable
        )
        == decal_voronoi._CornerPolicy.SMOOTH
    )
    # W-CP заморозил APPROXIMATE atlas до GL: C8.3 не меняет его семантику.
    assert (
        decal_voronoi._classify_surface_corner_runtime(
            approximate_surface, smooth_corner, stable
        )
        == decal_voronoi._CornerPolicy.MITER
    )


def test_c8_3_smooth_uses_static_bisector_and_continuous_uv():
    surface, corner = _smooth_corner_surface(5.0)
    settings = decal_voronoi.CornerRuntimeSettings(
        dynamic_corner_bands=False
    )
    supporting_lines = []

    for alpha in (0.25, 0.5, 1.0):
        components = decal_voronoi._smooth_crop_components(
            surface,
            corner,
            alpha,
            settings,
        )
        assert len(components) == 2
        assert {component.kind for component in components} == {"SEGMENT"}
        assert {
            component.owner_site_indices for component in components
        } == {(0,), (1,)}
        shared = set(components[0].points).intersection(
            components[1].points
        )
        assert len(shared) == 2
        shared = sorted(shared)
        direction = decal_voronoi._norm2(
            decal_voronoi._sub2(shared[1], shared[0])
        )
        supporting_lines.append(
            tuple(round(abs(value), 9) for value in direction)
        )
        for point in shared:
            uv_values = []
            for component in components:
                point_index = component.points.index(point)
                uv_values.append(
                    (
                        component.uv_anchors[point_index][0],
                        component.v_origin
                        + component.uv_anchors[point_index][1],
                    )
                )
            assert uv_values[0] == pytest.approx(uv_values[1])

    assert len(set(supporting_lines)) == 1


def test_c8_3_dense_convex_arc_has_only_static_segment_bisectors():
    graph, edge_indices = _smooth_arc_boundary_graph(12)
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        graph,
        edge_indices,
        offset=0.01,
        diagnostics=diagnostics,
    )
    settings = decal_voronoi.CornerRuntimeSettings(
        dynamic_corner_bands=False
    )
    previous_lines = None

    for width in (0.2, 0.4, 0.8):
        faces = evaluate_patch_voronoi_plan(
            plan,
            width=width,
            preview=True,
            diagnostics=diagnostics,
            corner_settings=settings,
        )
        assert diagnostics.runtime_policy_counts == {
            "CAP": 2,
            "SMOOTH": 11,
        }
        assert all(
            face.component_kind in {"CAP", "SEGMENT"} for face in faces
        )
        lines = set()
        surface = plan.surfaces[0]
        for corner in surface.corners:
            if decal_voronoi.classify_corner_runtime(corner, settings) is not (
                decal_voronoi._CornerPolicy.SMOOTH
            ):
                continue
            components = decal_voronoi._corner_crop_components(
                surface,
                corner,
                decal_voronoi._CornerPolicy.SMOOTH,
                width * 0.5,
                settings,
            )
            assert len(components) == 2
            assert all(component.kind == "SEGMENT" for component in components)
            rays = []
            for site_index in corner.ordered_sites:
                site = surface.sites[site_index]
                other = (
                    site.point_b
                    if site.vert_a == corner.vert_index
                    else site.point_a
                )
                rays.append(
                    decal_voronoi._norm2(
                        decal_voronoi._sub2(other, corner.point)
                    )
                )
            bisector = decal_voronoi._norm2(
                (
                    rays[0][0] + rays[1][0],
                    rays[0][1] + rays[1][1],
                )
            )
            assert bisector is not None
            for component in components:
                on_bisector = [
                    point
                    for point in component.points
                    if abs(
                        decal_voronoi._cross2(
                            bisector,
                            decal_voronoi._sub2(point, corner.point),
                        )
                    )
                    <= 1e-8
                ]
                assert len(on_bisector) >= 2
            a, b = -bisector[1], bisector[0]
            c = -(a * corner.point[0] + b * corner.point[1])
            if a < -1e-9 or (abs(a) <= 1e-9 and b < 0.0):
                a, b, c = -a, -b, -c
            lines.add((round(a, 6), round(b, 6), round(c, 6)))
        assert len(lines) == 11
        if previous_lines is not None:
            assert lines == previous_lines
        previous_lines = lines


def test_a11_thresholds_normalize_in_order_and_soft_band_wins_equality():
    normalized = decal_voronoi._normalized_corner_runtime_settings(
        decal_voronoi.CornerRuntimeSettings(
            miter_angle=100.0 * pi / 180.0,
            kite_angle=130.0 * pi / 180.0,
            split_angle=120.0 * pi / 180.0,
            hairpin_angle=150.0 * pi / 180.0,
        )
    )
    assert (
        normalized.miter_angle,
        normalized.kite_angle,
        normalized.split_angle,
        normalized.hairpin_angle,
    ) == pytest.approx((100.0 * pi / 180.0,) * 4)
    assert decal_voronoi._classify_extrusion_angle(
        normalized.miter_angle, normalized
    ) == decal_voronoi._CornerPolicy.MITER
    assert decal_voronoi.CornerRuntimeSettings(
        acute_split_angle=0.75
    ).split_angle == pytest.approx(0.75)


def test_corner_policy_threshold_changes_without_recompiling_plan():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01,
        diagnostics=diagnostics,
    )
    construct_calls = diagnostics.construct_calls
    corner = next(
        item for item in plan.surfaces[0].corners if item.vert_index == 0
    )
    split_settings = decal_voronoi.CornerRuntimeSettings(
        acute_split_angle=pi / 3.0,
    )
    miter_settings = decal_voronoi.CornerRuntimeSettings(
        miter_angle=pi / 12.0,
        kite_angle=pi / 12.0,
        split_angle=pi / 12.0,
        hairpin_angle=pi / 12.0,
    )

    assert (
        decal_voronoi.classify_corner_runtime(corner, split_settings)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )
    assert (
        decal_voronoi.classify_corner_runtime(corner, miter_settings)
        == decal_voronoi._CornerPolicy.MITER
    )

    split_faces = evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=split_settings,
        diagnostics=diagnostics,
    )
    split_policy_counts = dict(diagnostics.runtime_policy_counts)
    miter_faces = evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=miter_settings,
        diagnostics=diagnostics,
    )
    miter_policy_counts = dict(diagnostics.runtime_policy_counts)
    assert sum(
        face.component_kind == "ACUTE_SPLIT" for face in split_faces
    ) == 2
    assert not any(face.component_kind == "ACUTE_SPLIT" for face in miter_faces)
    assert split_policy_counts["ACUTE_SPLIT"] == 1
    assert miter_policy_counts.get("ACUTE_SPLIT", 0) == 0
    assert miter_policy_counts["MITER"] > split_policy_counts.get("MITER", 0)
    assert diagnostics.construct_calls == construct_calls


def _apex_limit_evaluations(
    plan, acute_split_angle, **corner_thresholds
):
    results = {}
    for apex_limit in (1.0, 8.0, 100.0):
        settings = decal_voronoi.CornerRuntimeSettings(
            acute_split_angle=acute_split_angle,
            apex_limit=apex_limit,
            dynamic_corner_bands=True,
            **corner_thresholds,
        )
        diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
        preview = evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=True,
            corner_settings=settings,
            diagnostics=diagnostics,
        )
        confirmed = evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=False,
            corner_settings=settings,
        )
        assert decal_voronoi.serialize_network_faces(
            preview
        ) == decal_voronoi.serialize_network_faces(confirmed)
        assert all(_signed_face_area(face) > 1e-10 for face in preview)
        results[apex_limit] = (preview, diagnostics)
    return results


def test_apex_limit_changes_convex_miter_contour():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    results = _apex_limit_evaluations(
        plan,
        acute_split_angle=0.1,
        miter_angle=pi / 2.0,
        kite_angle=pi / 2.0,
    )

    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert sum(
        face.component_kind == "MITER" for face in results[1.0][0]
    ) == 1
    assert results[1.0][1].clamped_miter_count == 1
    assert results[8.0][1].clamped_miter_count == 0


def test_apex_limit_changes_kite_contour():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    results = _apex_limit_evaluations(plan, acute_split_angle=1.0)

    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert results[1.0][1].clamped_kite_count == 8
    assert results[8.0][1].clamped_kite_count == 0


def test_apex_limit_clamps_acute_outer_without_gap():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        item for item in surface.corners if item.vert_index == 4
    )
    low_settings = decal_voronoi.CornerRuntimeSettings(
        apex_limit=1.0, dynamic_corner_bands=True
    )
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    reach_alpha = max(
        decal_voronoi._dist2(corner.point, point)
        for point in corner.split_chord
    )
    components = decal_voronoi._acute_crop_components(
        surface,
        corner,
        alpha=reach_alpha + 0.5,
        settings=low_settings,
        diagnostics=diagnostics,
    )

    inner = next(component for component in components if component.side == "INNER")
    outer = next(component for component in components if component.side == "OUTER")
    shared_cap = set(inner.points).intersection(outer.points)
    assert shared_cap == set(corner.split_chord)
    assert abs(decal_voronoi._polygon_area2(inner.points)) > 1e-10
    assert abs(decal_voronoi._polygon_area2(outer.points)) > 1e-10
    assert decal_voronoi._polygon_is_simple(list(inner.points))
    assert decal_voronoi._polygon_is_simple(list(outer.points))
    assert diagnostics.clamped_acute_count == 1

    results = _apex_limit_evaluations(plan, acute_split_angle=1.0)
    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert all(
        sum(face.component_kind == "ACUTE_SPLIT" for face in faces) == 1
        for faces, _diagnostics in results.values()
    )


def test_bevel_policy_is_reserved_and_never_classified():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    corner = next(
        item for item in plan.surfaces[0].corners if item.vert_index == 0
    )
    legacy_bevel_candidate = decal_voronoi.replace(
        corner,
        is_convex=False,
        interior_angle=pi * 0.75,
        extrusion_angle=pi * 0.75,
        miter_ratio=1000.0,
    )

    assert decal_voronoi.classify_corner_runtime(
        legacy_bevel_candidate,
        decal_voronoi.CornerRuntimeSettings(apex_limit=1.0),
    ) == decal_voronoi._CornerPolicy.MITER
    assert decal_voronoi.CornerRuntimeSettings(
        miter_limit=3.0
    ).apex_limit == 3.0


def test_acute_apex_limit_saturates_outside_cap_chord():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    apex = decal_voronoi._clamped_acute_apex(
        corner_point=(0.0, 0.0),
        cap_a=(2.0, -1.0),
        cap_b=(2.0, 1.0),
        intersection=(4.0, 0.0),
        alpha=0.5,
        settings=decal_voronoi.CornerRuntimeSettings(apex_limit=1.0),
        diagnostics=diagnostics,
    )

    assert apex[0] > 2.0
    assert diagnostics.clamped_acute_count == 1
    assert diagnostics.apex_limit_saturated_count == 1


def test_convex_corner_builds_explicit_realtime_kite():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 2
    )
    kite = decal_voronoi._kite_crop_polygon(surface, corner, alpha=0.5)
    assert len(kite) == 4
    assert corner.point in kite
    assert abs(decal_voronoi._polygon_area2(kite)) == pytest.approx(0.25)

    narrow = decal_voronoi._kite_crop_polygon(surface, corner, alpha=0.25)
    assert len(narrow) == 4
    assert abs(decal_voronoi._polygon_area2(narrow)) == pytest.approx(
        0.25 * abs(decal_voronoi._polygon_area2(kite))
    )


def test_corner_absorbs_incident_point_cell_boundaries_before_collision():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    faces = evaluate_patch_voronoi_plan(
        plan, width=0.1, preview=True, corner_settings=_BAND_SETTINGS
    )

    kite_faces = [face for face in faces if face.component_kind == "KITE"]
    segment_faces = [
        face for face in faces if face.component_kind == "SEGMENT"
    ]
    assert len(kite_faces) == 8
    assert all(len(face.positions) == 4 for face in kite_faces)
    assert len(segment_faces) == len(plan.surfaces[0].sites)


def _short_segment_endpoint_surface(reverse_site=False):
    if reverse_site:
        vert_a, vert_b = 1, 0
        point_a, point_b = (1.0, 0.0), (0.0, 0.0)
    else:
        vert_a, vert_b = 0, 1
        point_a, point_b = (0.0, 0.0), (1.0, 0.0)
    site = decal_voronoi._PatchVoronoiSite(
        patch_id=0,
        edge_index=500,
        vert_a=vert_a,
        vert_b=vert_b,
        source_a=Vector((*point_a, 0.0)),
        source_b=Vector((*point_b, 0.0)),
        point_a=point_a,
        point_b=point_b,
        arc_start=0.0,
        segment_length=1.0,
        uv_sign=1.0,
        inward_normal=(0.0, 1.0),
    )
    corners = tuple(
        decal_voronoi.CornerSpec(
            vert_index=vert_index,
            point=(float(vert_index), 0.0),
            incident_sites=(0,),
            ordered_sites=(0,),
            turn_sign=1.0,
            interior_angle=pi,
            extrusion_angle=pi,
            is_convex=vert_index == 0,
            miter_ratio=1.0,
        )
        for vert_index in (0, 1)
    )
    segment_fragment = ((0.0, -1.0), (1.0, -1.0), (1.0, 1.0), (0.0, 1.0))
    point_fragments = (
        ((-1.0, -1.0), (0.0, -1.0), (0.0, 1.0), (-1.0, 1.0)),
        ((1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (1.0, 1.0)),
    )
    atoms = (
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(segment_fragment,),
            cell_kind="SEGMENT",
        ),
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(point_fragments[0],),
            cell_kind="POINT",
            corner_index=0,
        ),
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(point_fragments[1],),
            cell_kind="POINT",
            corner_index=1,
        ),
    )
    return SimpleNamespace(sites=(site,), corners=corners, atoms=atoms)


def _short_segment_owned_crops(monkeypatch, reverse_site=False):
    monkeypatch.setattr(
        decal_voronoi,
        "classify_corner_runtime",
        lambda _corner, _settings=None: decal_voronoi._CornerPolicy.CAP,
    )
    pending = []
    decal_voronoi._evaluate_surface_crops(
        _short_segment_endpoint_surface(reverse_site),
        alpha=1.0,
        pending=pending,
        corner_settings=_BAND_SETTINGS,
    )
    return pending


def _serialized_short_segment_owned_crops(settings):
    pending = []
    decal_voronoi._evaluate_surface_crops(
        _short_segment_endpoint_surface(),
        alpha=1.0,
        pending=pending,
        corner_settings=settings,
    )
    return tuple(
        (
            face.crop.kind,
            face.crop.side,
            tuple(face.points),
            tuple(face.crop.uv_anchors),
            face.crop.v_origin,
        )
        for face in pending
    )


def test_stable_flat_cap_matches_tangent_aligned_dynamic_cap():
    stable = _serialized_short_segment_owned_crops(
        decal_voronoi.CornerRuntimeSettings(dynamic_corner_bands=False)
    )
    dynamic = _serialized_short_segment_owned_crops(_BAND_SETTINGS)

    assert stable == dynamic
    assert {item[1] for item in stable} == {"START", "END"}


def test_stable_flat_cap_never_reaches_behind_endpoint():
    surface = _short_segment_endpoint_surface()
    settings = decal_voronoi.CornerRuntimeSettings(
        dynamic_corner_bands=False
    )

    for corner in surface.corners:
        components = decal_voronoi._corner_crop_components(
            surface,
            corner,
            decal_voronoi._CornerPolicy.CAP,
            1.0,
            settings,
        )
        assert len(components) == 1
        assert len(components[0].points) == 4
        site = surface.sites[corner.incident_sites[0]]
        other = (
            site.point_b
            if site.vert_a == corner.vert_index
            else site.point_a
        )
        tangent = decal_voronoi._norm2(
            decal_voronoi._sub2(other, corner.point)
        )
        reaches = [
            decal_voronoi._dot2(
                decal_voronoi._sub2(point, corner.point), tangent
            )
            for point in components[0].points
        ]
        assert min(reaches) >= -1e-12
        assert max(reaches) <= site.segment_length * 0.5 + 1e-12


def test_short_segment_corner_ownership_is_disjoint_and_order_independent(
    monkeypatch,
):
    pending = _short_segment_owned_crops(monkeypatch, reverse_site=False)
    assert {face.crop.kind for face in pending} == {"CAP"}
    assert {face.crop.side for face in pending} == {"START", "END"}
    assert len(pending) == 2

    by_kind = {face.crop.side: face for face in pending}
    overlap = decal_voronoi._clip_to_convex(
        by_kind["START"].points,
        by_kind["END"].points,
    )
    assert not overlap or abs(decal_voronoi._polygon_area2(overlap)) <= 1e-10
    assert sum(
        abs(decal_voronoi._polygon_area2(face.points)) for face in pending
    ) == pytest.approx(2.0)

    shared_edge = set(by_kind["START"].points).intersection(
        by_kind["END"].points
    )
    assert shared_edge == {(0.5, -1.0), (0.5, 1.0)}
    face_keys = {
        tuple(sorted(face.points))
        for face in pending
    }
    assert len(face_keys) == len(pending)

    reversed_pending = _short_segment_owned_crops(
        monkeypatch, reverse_site=True
    )

    def signature(faces):
        return {tuple(sorted(face.points)) for face in faces}

    assert signature(reversed_pending) == signature(pending)


def test_endpoint_ownership_preserves_affine_uv_after_triangle_clip():
    surface = _short_segment_endpoint_surface()
    corner = surface.corners[0]
    crop = decal_voronoi._CropComponent(
        kind="ACUTE_SPLIT",
        side="OUTER",
        points=((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)),
        uv_anchors=((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)),
        v_origin=2.0,
    )

    owned = decal_voronoi._corner_endpoint_ownership_crop(
        surface, corner, crop
    )

    assert owned is not None
    assert len(owned.points) == 4
    assert len(owned.uv_anchors) == 4
    for point in owned.points:
        assert decal_voronoi._crop_component_uv(
            owned, point
        ) == pytest.approx((point[0], point[1] + 2.0))


def test_planar_chart_is_invariant_to_patch_normal_sign():
    graph, edge_indices = _door_opening_graph()
    flipped_graph = deepcopy(graph)
    flipped_node = flipped_graph.nodes[0]
    flipped_node.normal = flipped_node.normal * -1.0
    flipped_node.basis_v = flipped_node.basis_v * -1.0

    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    flipped_plan = compile_patch_voronoi_plan(
        flipped_graph, edge_indices, offset=0.01
    )
    surface = plan.surfaces[0]
    flipped_surface = flipped_plan.surfaces[0]
    assert [
        (site.point_a, site.point_b) for site in surface.sites
    ] == [
        (site.point_a, site.point_b) for site in flipped_surface.sites
    ]
    assert surface.atoms == flipped_surface.atoms

    for width in (0.1, 0.5, 2.0):
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        flipped_faces = evaluate_patch_voronoi_plan(
            flipped_plan, width, preview=True
        )
        assert [
            (
                face.component_kind,
                face.component_side,
                tuple(face.vert_keys),
            )
            for face in faces
        ] == [
            (
                face.component_kind,
                face.component_side,
                tuple(reversed(face.vert_keys)),
            )
            for face in flipped_faces
        ]


def test_realtime_corner_partition_has_no_planar_gaps():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    surface = plan.surfaces[0]
    domain_points = [
        point
        for triangle in surface.domain.boundary_triangles
        for point in triangle
    ]
    min_x = min(point[0] for point in domain_points)
    max_x = max(point[0] for point in domain_points)
    min_y = min(point[1] for point in domain_points)
    max_y = max(point[1] for point in domain_points)

    def inside_polygon(point, polygon):
        inside = False
        for index, point_a in enumerate(polygon):
            point_b = polygon[(index + 1) % len(polygon)]
            if (point_a[1] > point[1]) == (point_b[1] > point[1]):
                continue
            crossing_x = point_a[0] + (
                (point[1] - point_a[1])
                * (point_b[0] - point_a[0])
                / (point_b[1] - point_a[1])
            )
            if crossing_x > point[0]:
                inside = not inside
        return inside

    for width in (0.2, 1.0, 3.0):
        alpha = width * 0.5
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        polygons = [
            [
                (
                    (position - surface.origin).dot(surface.basis_u),
                    (position - surface.origin).dot(surface.basis_v),
                )
                for position in face.positions
            ]
            for face in faces
            if face.surface_id == surface.patch_id
        ]
        for grid_y in range(31):
            for grid_x in range(31):
                point = (
                    min_x + (grid_x + 0.37) / 31.0 * (max_x - min_x),
                    min_y + (grid_y + 0.61) / 31.0 * (max_y - min_y),
                )
                if not decal_voronoi._point_in_domain(
                    point, surface.domain.boundary_triangles
                ):
                    continue
                distance = min(
                    decal_voronoi._segment_point_distance2(
                        site.point_a, site.point_b, point
                    )[0]
                    for site in surface.sites
                )
                if abs(distance - alpha) <= 1e-5:
                    continue
                actual = any(
                    inside_polygon(point, polygon) for polygon in polygons
                )
                if distance < alpha:
                    assert actual


def test_acute_corner_splits_inner_outer_with_shared_mesh_edge_and_uv_seam():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 4
    )
    components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=0.5, settings=_BAND_SETTINGS
    )
    assert [component.side for component in components] == ["INNER"]
    assert len(components[0].points) >= 3
    full_kite = decal_voronoi._kite_crop_polygon(
        surface, corner, alpha=0.5
    )
    assert abs(decal_voronoi._polygon_area2(components[0].points)) == pytest.approx(
        abs(decal_voronoi._polygon_area2(full_kite))
    )

    reach_alpha = max(
        decal_voronoi._dist2(corner.point, point)
        for point in corner.split_chord
    )
    split_components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=reach_alpha + 0.5, settings=_BAND_SETTINGS
    )
    assert [component.side for component in split_components] == [
        "INNER",
        "OUTER",
    ]
    assert set(split_components[0].points).intersection(
        split_components[1].points
    ) == set(corner.split_chord)

    faces = evaluate_patch_voronoi_plan(
        plan, width=1.0, preview=True, corner_settings=_BAND_SETTINGS
    )
    acute_faces = {
        face.component_side: face
        for face in faces
        if face.component_kind == "ACUTE_SPLIT"
    }
    assert set(acute_faces) == {"INNER"}
    confirmed = evaluate_patch_voronoi_plan(
        plan, width=1.0, preview=False, corner_settings=_BAND_SETTINGS
    )
    assert [
        (face.component_kind, face.component_side, tuple(face.vert_keys))
        for face in faces
    ] == [
        (face.component_kind, face.component_side, tuple(face.vert_keys))
        for face in confirmed
    ]


def test_acute_convex_corner_splits_before_wide_miter_competition():
    plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 0
    )

    components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=2.0, settings=_BAND_SETTINGS
    )
    assert {component.side for component in components} == {"INNER"}

    reach_alpha = max(
        decal_voronoi._dist2(corner.point, point)
        for point in corner.split_chord
    )
    components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=reach_alpha + 0.5, settings=_BAND_SETTINGS
    )
    assert {component.side for component in components} == {"INNER", "OUTER"}
    assert all(
        component.kind == decal_voronoi._CornerPolicy.ACUTE_SPLIT.value
        for component in components
    )
    shared_points = set(components[0].points) & set(components[1].points)
    assert shared_points == set(corner.split_chord)

    faces = evaluate_patch_voronoi_plan(
        plan, width=4.0, preview=False, corner_settings=_BAND_SETTINGS
    )
    acute_faces = [
        face for face in faces if face.component_kind == "ACUTE_SPLIT"
    ]
    assert {face.component_side for face in acute_faces} == {"INNER"}


def test_a11_fan_and_hairpin_have_semantic_uv_components():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        item for item in surface.corners if item.vert_index == 4
    )
    fan = decal_voronoi._fan_crop_components(
        surface, corner, alpha=0.5
    )
    assert len(fan) == 2
    assert {component.side for component in fan} == {"A", "B"}
    assert all(component.kind == "FAN" for component in fan)
    assert all(len(component.points) == 3 for component in fan)
    assert all(
        len(component.uv_anchors) == len(component.points)
        for component in fan
    )

    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    hairpin = decal_voronoi._hairpin_crop_components(
        surface,
        corner,
        alpha=0.5,
        settings=decal_voronoi.CornerRuntimeSettings(
            hairpin_angle=pi / 3.0
        ),
        diagnostics=diagnostics,
    )
    assert len(hairpin) == 1
    assert hairpin[0].kind == "HAIRPIN"
    assert hairpin[0].side == "BLUNT"
    assert len(hairpin[0].uv_anchors) == len(hairpin[0].points)
    assert diagnostics.apex_limit_saturated_count >= 1


def test_a12_split_chord_is_compiled_static_geometry_with_stable_uv():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(item for item in surface.corners if item.vert_index == 4)
    assert len(corner.static_wedge) == 3
    assert len(corner.split_chord) == 2

    coordinates = [
        decal_voronoi._corner_wedge_coordinates(
            surface.sites, corner, point
        )
        for point in corner.split_chord
    ]
    assert coordinates[0][1] == pytest.approx(0.0, abs=1e-8)
    assert coordinates[1][0] == pytest.approx(0.0, abs=1e-8)

    reach_alpha = max(
        decal_voronoi._dist2(corner.point, point)
        for point in corner.split_chord
    )
    inner_frames = []
    for alpha in (reach_alpha + 0.5, reach_alpha + 1.0):
        components = decal_voronoi._acute_crop_components(
            surface, corner, alpha=alpha
        )
        inner = next(
            component for component in components if component.side == "INNER"
        )
        inner_frames.append(
            (
                inner.points,
                tuple(
                    (uv[0] * alpha, uv[1]) for uv in inner.uv_anchors
                ),
            )
        )
    assert inner_frames[0][0] == inner_frames[1][0]
    for first_uv, second_uv in zip(
        inner_frames[0][1], inner_frames[1][1]
    ):
        assert first_uv == pytest.approx(second_uv, abs=1e-12)


def test_a12_hairpin_materializes_blunt_front_after_static_chord():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(item for item in surface.corners if item.vert_index == 4)
    reach_alpha = max(
        decal_voronoi._dist2(corner.point, point)
        for point in corner.split_chord
    )
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    components = decal_voronoi._hairpin_crop_components(
        surface,
        corner,
        alpha=reach_alpha + 0.5,
        diagnostics=diagnostics,
    )
    assert {component.side for component in components} == {"INNER", "BLUNT"}
    inner = next(component for component in components if component.side == "INNER")
    blunt = next(component for component in components if component.side == "BLUNT")
    assert set(inner.points).intersection(blunt.points) == set(
        corner.split_chord
    )
    assert diagnostics.apex_limit_saturated_count == 1


def test_a12_miter_and_kite_ignore_static_interior_fields_byte_for_byte():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    stripped_surface = decal_voronoi.replace(
        surface,
        corners=tuple(
            decal_voronoi.replace(
                corner,
                split_chord=(),
                static_wedge=(),
            )
            for corner in surface.corners
        ),
    )
    stripped_plan = decal_voronoi.replace(
        plan,
        surfaces=(stripped_surface,),
    )

    settings_by_policy = (
        decal_voronoi.CornerRuntimeSettings(miter_angle=0.0),
        decal_voronoi.CornerRuntimeSettings(
            miter_angle=pi,
            kite_angle=0.0,
        ),
    )
    for settings in settings_by_policy:
        baseline = evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=True,
            corner_settings=settings,
        )
        stripped = evaluate_patch_voronoi_plan(
            stripped_plan,
            width=1.0,
            preview=True,
            corner_settings=settings,
        )
        assert decal_voronoi.serialize_network_faces(
            baseline
        ) == decal_voronoi.serialize_network_faces(stripped)


def test_a11_cap_is_tangent_aligned_and_has_deterministic_uv():
    tangent = Vector((1.0, 1.0)).normalized()
    site = decal_voronoi._PatchVoronoiSite(
        patch_id=0,
        edge_index=700,
        vert_a=0,
        vert_b=1,
        source_a=Vector((0.0, 0.0, 0.0)),
        source_b=Vector((1.0, 1.0, 0.0)),
        point_a=(0.0, 0.0),
        point_b=(1.0, 1.0),
        arc_start=4.0,
        segment_length=sqrt(2.0),
        uv_sign=1.0,
        inward_normal=(-tangent.y, tangent.x),
    )
    corner = decal_voronoi.CornerSpec(
        vert_index=0,
        point=(0.0, 0.0),
        incident_sites=(0,),
        ordered_sites=(0,),
        turn_sign=0.0,
        interior_angle=pi,
        extrusion_angle=pi,
        is_convex=False,
        miter_ratio=1.0,
    )
    component = decal_voronoi._cap_crop_components(
        SimpleNamespace(sites=(site,)), corner, alpha=0.25
    )[0]
    terminal = [
        point
        for point in component.points
        if abs(point[0] * tangent.x + point[1] * tangent.y) <= 1e-8
    ]
    assert len(terminal) == 2
    cap_edge = Vector(terminal[1]) - Vector(terminal[0])
    assert cap_edge.dot(tangent) == pytest.approx(0.0, abs=1e-8)
    assert len(component.uv_anchors) == len(component.points) == 4
    assert component.v_origin == pytest.approx(4.0)


def test_a11_valence_n_junction_uses_ordered_non_reflex_sectors():
    directions = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))
    sites = tuple(
        decal_voronoi._PatchVoronoiSite(
            patch_id=0,
            edge_index=800 + index,
            vert_a=0,
            vert_b=index + 1,
            source_a=Vector((0.0, 0.0, 0.0)),
            source_b=Vector((*direction, 0.0)),
            point_a=(0.0, 0.0),
            point_b=direction,
            arc_start=0.0,
            segment_length=1.0,
            uv_sign=1.0,
            inward_normal=(-direction[1], direction[0]),
        )
        for index, direction in enumerate(directions)
    )
    corner = decal_voronoi.CornerSpec(
        vert_index=0,
        point=(0.0, 0.0),
        incident_sites=(2, 0, 1),
        ordered_sites=(2, 0, 1),
        turn_sign=0.0,
        interior_angle=0.0,
        extrusion_angle=0.0,
        is_convex=False,
        miter_ratio=float("inf"),
    )
    surface = SimpleNamespace(sites=sites)
    sectors = decal_voronoi._junction_sector_specs(surface, corner)
    assert len(sectors) == 2
    assert all(
        sector.extrusion_angle == pytest.approx(pi / 2.0)
        for _index, sector in sectors
    )
    policies = [
        decal_voronoi._classify_extrusion_angle(
            sector.extrusion_angle,
            decal_voronoi.CornerRuntimeSettings(),
        )
        for _index, sector in sectors
    ]
    assert policies == [
        decal_voronoi._CornerPolicy.KITE,
        decal_voronoi._CornerPolicy.KITE,
    ]


def test_a11_cross_surface_valence_n_junction_builds_sector_fan():
    alpha = 0.5
    directions = tuple(
        (
            cos(index * 2.0 * pi / 3.0),
            sin(index * 2.0 * pi / 3.0),
        )
        for index in range(3)
    )
    surfaces = []
    faces = []
    for index, direction in enumerate(directions):
        site = decal_voronoi._PatchVoronoiSite(
            patch_id=index,
            edge_index=900 + index,
            vert_a=0,
            vert_b=index + 1,
            source_a=Vector((0.0, 0.0, 0.0)),
            source_b=Vector((direction[0], direction[1], 0.0)),
            point_a=(0.0, 0.0),
            point_b=direction,
            arc_start=0.0,
            segment_length=1.0,
            uv_sign=1.0,
            inward_normal=direction,
        )
        surfaces.append(
            SimpleNamespace(
                patch_id=index,
                origin=Vector((0.0, 0.0, 0.0)),
                basis_u=Vector((1.0, 0.0, 0.0)),
                basis_v=Vector((0.0, 1.0, 0.0)),
                sites=(site,),
            )
        )
        outer = Vector((direction[0] * alpha, direction[1] * alpha, 0.0))
        faces.append(
            decal_voronoi._NetworkFace(
                surface_id=index,
                surface_normal=Vector((0.0, 0.0, 1.0)),
                vert_keys=[
                    ("pv-sv", 0),
                    ("outer", index),
                    ("tail", index),
                ],
                positions=[
                    Vector((0.0, 0.0, 0.0)),
                    outer,
                    outer + Vector((0.0, 0.0, 0.1)),
                ],
                u_fracs=[0.0, 1.0, 1.0],
                v_lengths=[0.0, 0.5, 1.0],
            )
        )
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    connectors = decal_voronoi._junction_connector_faces(
        SimpleNamespace(surfaces=tuple(surfaces)),
        faces,
        alpha,
        decal_voronoi.CornerRuntimeSettings(),
        diagnostics,
    )
    assert len(connectors) == 3
    assert {face.component_kind for face in connectors} == {"MITER"}
    assert all(
        face.component_side.startswith("JUNCTION_SECTOR_")
        for face in connectors
    )
    assert diagnostics.runtime_policy_counts == {"MITER": 3}


def test_a11_network_v_phase_and_same_chart_signed_u():
    raw_by_patch = {
        0: [
            {
                "edge_index": 1,
                "vert_a": 0,
                "vert_b": 1,
                "segment_length": 2.0,
            },
            {
                "edge_index": 2,
                "vert_a": 2,
                "vert_b": 1,
                "segment_length": 3.0,
            },
        ]
    }
    decal_voronoi._assign_network_v_phases(raw_by_patch)
    first, second = raw_by_patch[0]
    assert first["arc_start"] == pytest.approx(0.0)
    assert first["arc_sign"] == 1.0
    assert second["arc_start"] == pytest.approx(5.0)
    assert second["arc_sign"] == -1.0
    assert second["arc_start"] + second["arc_sign"] * 3.0 == pytest.approx(
        first["arc_start"] + first["arc_sign"] * 2.0
    )

    site = decal_voronoi._PatchVoronoiSite(
        patch_id=0,
        edge_index=3,
        vert_a=0,
        vert_b=1,
        source_a=Vector((0.0, 0.0, 0.0)),
        source_b=Vector((2.0, 0.0, 0.0)),
        point_a=(0.0, 0.0),
        point_b=(2.0, 0.0),
        arc_start=0.0,
        segment_length=2.0,
        uv_sign=1.0,
        inward_normal=(0.0, 1.0),
        two_sided=True,
    )
    assert decal_voronoi._site_lateral_u(
        site, (1.0, 0.5), 0.5
    ) == pytest.approx(1.0)
    assert decal_voronoi._site_lateral_u(
        site, (1.0, -0.5), 0.5
    ) == pytest.approx(-1.0)


def test_cross_surface_spine_station_is_mirrored_without_new_face():
    from types import SimpleNamespace

    station_key = ("pv-se", 40, 0.25)
    station = Vector((0.25, 0.0, 0.0))
    owner = decal_voronoi._NetworkFace(
        surface_id=0,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("pv-sv", 0), station_key, ("pv", 0, 1, 1)],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            station,
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[0.0, 0.25, 1.0],
        v_lengths=[0.0, 0.25, 1.0],
    )
    neighbour = decal_voronoi._NetworkFace(
        surface_id=1,
        surface_normal=Vector((0.0, 1.0, 0.0)),
        vert_keys=[("pv-sv", 0), ("pv-sv", 1), ("pv", 1, 1, 1)],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
        ],
        u_fracs=[0.0, 1.0, 0.0],
        v_lengths=[0.0, 4.0, 0.0],
    )
    plan = SimpleNamespace(
        surfaces=(
            SimpleNamespace(
                sites=(
                    SimpleNamespace(edge_index=40, vert_a=0, vert_b=1),
                )
            ),
        )
    )

    decal_voronoi._synchronize_cross_surface_spine_stations(
        plan, [owner, neighbour]
    )

    assert neighbour.vert_keys == [
        ("pv-sv", 0),
        station_key,
        ("pv-sv", 1),
        ("pv", 1, 1, 1),
    ]
    assert tuple(neighbour.positions[1]) == pytest.approx(tuple(station))
    assert neighbour.u_fracs[1] == pytest.approx(0.25)
    assert neighbour.v_lengths[1] == pytest.approx(1.0)


def test_c8_2_cross_surface_spine_merges_already_split_runs():
    first_station = ("pv-se", 40, 0.25)
    second_station = ("pv-se", 40, 0.5)
    first = decal_voronoi._NetworkFace(
        surface_id=0,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[
            ("pv-sv", 0),
            first_station,
            ("pv-sv", 1),
            ("pv", 0, 1, 1),
        ],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            Vector((0.25, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[0.0, 0.25, 1.0, 0.0],
        v_lengths=[0.0, 1.0, 4.0, 0.0],
    )
    second = decal_voronoi._NetworkFace(
        surface_id=1,
        surface_normal=Vector((0.0, 1.0, 0.0)),
        vert_keys=[
            ("pv-sv", 0),
            second_station,
            ("pv-sv", 1),
            ("pv", 1, 1, 1),
        ],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            Vector((0.5, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
        ],
        u_fracs=[0.0, 0.5, 1.0, 0.0],
        v_lengths=[0.0, 2.0, 4.0, 0.0],
    )
    plan = SimpleNamespace(
        surfaces=(
            SimpleNamespace(
                sites=(
                    SimpleNamespace(edge_index=40, vert_a=0, vert_b=1),
                )
            ),
        )
    )

    decal_voronoi._synchronize_cross_surface_spine_stations(
        plan, [first, second]
    )

    expected = [
        ("pv-sv", 0),
        first_station,
        second_station,
        ("pv-sv", 1),
    ]
    assert first.vert_keys[:4] == expected
    assert second.vert_keys[:4] == expected
    assert tuple(first.positions[2]) == pytest.approx((0.5, 0.0, 0.0))
    assert tuple(second.positions[1]) == pytest.approx((0.25, 0.0, 0.0))


def test_c8_6_spine_sync_does_not_split_interior_to_source_edge_frontier():
    station_key = ("pv-se", 32, 0.9440057)
    near_station_key = ("pv-se", 32, 0.9439329)
    face = decal_voronoi._NetworkFace(
        surface_id=109,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[
            station_key,
            ("pv-feature", "EDGE", "triangulation", 1),
            ("pv", 109, 0, 883, 19771),
            near_station_key,
        ],
        positions=[
            Vector((0.9995, 0.0, 0.0)),
            Vector((0.5, 1.0, 0.0)),
            Vector((0.0, 0.0, 0.0)),
            Vector((0.9996, 0.0, 0.0)),
        ],
        u_fracs=[0.9995, 0.5, 0.0, 0.9996],
        v_lengths=[1.0, 1.0, 0.0, 1.0],
        component_kind="SEGMENT",
    )
    plan = SimpleNamespace(
        surfaces=(
            SimpleNamespace(
                sites=(
                    SimpleNamespace(
                        edge_index=32,
                        vert_a=10,
                        vert_b=20,
                    ),
                )
            ),
        )
    )
    original_keys = tuple(face.vert_keys)
    original_positions = tuple(tuple(value) for value in face.positions)

    decal_voronoi._synchronize_cross_surface_spine_stations(plan, [face])

    assert tuple(face.vert_keys) == original_keys
    assert tuple(tuple(value) for value in face.positions) == original_positions
    assert len(face.vert_keys) == len(set(face.vert_keys))


def test_c8_2_unselected_fold_is_connected_without_spine_sync(monkeypatch):
    graph, selected_edges = _planar_unselected_fold_graph()
    plan = compile_patch_voronoi_plan(
        graph, selected_edges, offset=0.01
    )
    assert plan is not None
    assert {surface.domain.kind for surface in plan.surfaces} == {"PLANAR"}
    monkeypatch.setattr(
        decal_voronoi,
        "_synchronize_cross_surface_spine_stations",
        lambda _plan, _faces: None,
    )

    for width in (1.0, 1.2, 1.5):
        faces = evaluate_patch_voronoi_plan(
            plan, width=width, preview=True
        )
        assert faces
        station_key = ("pv-se", 99, 0.5)
        station_surfaces = {
            face.surface_id
            for face in faces
            if station_key in face.vert_keys
        }
        assert station_surfaces == {20, 21}

        edges_by_face = []
        for face in faces:
            edges_by_face.append(
                {
                    tuple(
                        sorted(
                            (
                                face.vert_keys[index],
                                face.vert_keys[(index + 1) % len(face.vert_keys)],
                            ),
                            key=repr,
                        )
                    )
                    for index in range(len(face.vert_keys))
                }
            )
        unseen = set(range(len(faces)))
        components = 0
        while unseen:
            components += 1
            stack = [unseen.pop()]
            while stack:
                face_index = stack.pop()
                neighbours = {
                    other_index
                    for other_index in unseen
                    if edges_by_face[face_index] & edges_by_face[other_index]
                }
                unseen.difference_update(neighbours)
                stack.extend(neighbours)
        assert components == 1


def test_surface_domain_separates_planar_solver_from_intrinsic_lift():
    planar = decal_voronoi.DecalSurfaceDomain(
        patch_id=0,
        kind="PLANAR",
        origin=Vector((1.0, 2.0, 3.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(),
    )
    assert tuple(planar.lift((2.0, 4.0), 0.5)) == pytest.approx(
        (3.0, 6.0, 3.5)
    )
    assert planar.project(Vector((3.0, 6.0, 3.5))) == pytest.approx(
        (2.0, 4.0)
    )

    normals = (
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, 1.0, 1.0)).normalized(),
    )
    intrinsic = decal_voronoi.DecalSurfaceDomain(
        patch_id=1,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),),
        intrinsic_triangles=(
            decal_voronoi._IntrinsicDomainTriangle(
                chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
                positions=(
                    Vector((0.0, 0.0, 0.0)),
                    Vector((1.0, 0.0, 0.0)),
                    Vector((0.0, 1.0, 1.0)),
                ),
                normals=normals,
            ),
        ),
        normal_mode="SMOOTH_INTERPOLATED",
    )
    point = (0.25, 0.25)
    expected_position = Vector((0.25, 0.25, 0.25))
    expected_normal = (
        normals[0] * 0.5 + normals[1] * 0.25 + normals[2] * 0.25
    ).normalized()
    assert tuple(intrinsic.normal_at(point)) == pytest.approx(
        tuple(expected_normal)
    )
    assert tuple(intrinsic.lift(point, 0.1)) == pytest.approx(
        tuple(expected_position + expected_normal * 0.1)
    )
    with pytest.raises(ValueError):
        intrinsic.project(Vector((0.0, 0.0, 0.0)))


def test_c8_2_planar_source_edge_keys_are_cross_surface_exact():
    first = decal_voronoi.DecalSurfaceDomain(
        patch_id=10,
        kind="PLANAR",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(),
        planar_source_edges=((77, 100, 101, (0.0, 0.0), (1.0, 0.0)),),
        planar_source_vertices=((100, (0.0, 0.0)), (101, (1.0, 0.0))),
        planar_source_edge_positions=(
            (77, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ),
    )
    second = decal_voronoi.DecalSurfaceDomain(
        patch_id=11,
        kind="PLANAR",
        origin=Vector((1.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, -1.0, 0.0)),
        basis_u=Vector((-1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 0.0, 1.0)),
        boundary_triangles=(),
        planar_source_edges=((77, 100, 101, (1.0, 0.0), (0.0, 0.0)),),
        planar_source_vertices=((100, (1.0, 0.0)), (101, (0.0, 0.0))),
        planar_source_edge_positions=(
            (77, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ),
    )
    location_a = first.locate((0.375, 0.0))
    location_b = second.locate((0.625, 0.0))
    surface_a = SimpleNamespace(domain=first, patch_id=10)
    surface_b = SimpleNamespace(domain=second, patch_id=11)

    assert (location_a.source_feature, location_a.source_feature_id) == (
        "EDGE",
        77,
    )
    assert (location_b.source_feature, location_b.source_feature_id) == (
        "EDGE",
        77,
    )
    assert decal_voronoi._domain_location_key(
        surface_a, location_a
    ) == decal_voronoi._domain_location_key(surface_b, location_b)
    assert decal_voronoi._domain_location_key(
        surface_a, location_a
    ) == ("pv-se", 77, 0.375)
    assert decal_voronoi._domain_location_key(
        surface_a, first.locate((0.0, 0.0))
    ) == ("pv-sv", 100)


def _c8_6_planar_domain():
    return decal_voronoi.DecalSurfaceDomain(
        patch_id=12,
        kind="PLANAR",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(),
        planar_source_edges=(
            (32, 10, 20, (0.0, 0.0), (1.0, 0.0)),
        ),
        planar_source_vertices=(
            (10, (0.0, 0.0)),
            (20, (1.0, 0.0)),
        ),
        planar_source_edge_positions=(
            (32, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ),
    )


def _c8_6_reversed_site():
    return decal_voronoi._PatchVoronoiSite(
        patch_id=12,
        edge_index=32,
        vert_a=20,
        vert_b=10,
        source_a=Vector((1.0, 0.0, 0.0)),
        source_b=Vector((0.0, 0.0, 0.0)),
        point_a=(1.0, 0.0),
        point_b=(0.0, 0.0),
        arc_start=0.0,
        segment_length=1.0,
        uv_sign=1.0,
        inward_normal=(0.0, 1.0),
    )


def _c8_6_lift_plan(site):
    return SimpleNamespace(
        lifted_vertices={
            10: Vector((0.0, 0.0, 0.1)),
            20: Vector((1.0, 0.0, 0.1)),
        },
        surfaces=(SimpleNamespace(sites=(site,)),),
    )


def test_c8_6_reversed_direct_site_uses_canonical_source_edge_parameter():
    domain = _c8_6_planar_domain()
    surface = SimpleNamespace(domain=domain, patch_id=12)
    site = _c8_6_reversed_site()
    plan = _c8_6_lift_plan(site)

    direct_keys = []
    domain_keys = []
    for parameter in (0.0559943, 0.9440057):
        point = (parameter, 0.0)
        _position, direct_key, location = decal_voronoi._position_and_key(
            plan,
            surface,
            site,
            point,
            lift_scale=0.0,
            effective_offset=0.0,
        )
        direct_keys.append(direct_key)
        domain_keys.append(
            decal_voronoi._domain_location_key(surface, location)
        )

    assert direct_keys == domain_keys
    assert direct_keys == [
        ("pv-se", 32, 0.0559943),
        ("pv-se", 32, 0.9440057),
    ]
    assert len(set(direct_keys)) == 2


def test_c8_6_exact_intrinsic_direct_site_keeps_canonical_pv_se_key():
    triangle = decal_voronoi._IntrinsicDomainTriangle(
        chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
        positions=(
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ),
        normals=(Vector((0.0, 0.0, 1.0)),) * 3,
        source_triangle_id=120,
        source_face_id=12,
        source_edge_ids=(33, 34, 32),
        source_vertex_ids=(10, 20, 30),
    )
    domain = decal_voronoi.DecalSurfaceDomain(
        patch_id=12,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(),
        intrinsic_triangles=(triangle,),
    )
    surface = SimpleNamespace(domain=domain, patch_id=12)
    site = _c8_6_reversed_site()
    plan = _c8_6_lift_plan(site)

    keys = []
    for parameter in (0.0559943, 0.9440057):
        _position, key, location = decal_voronoi._position_and_key(
            plan,
            surface,
            site,
            (parameter, 0.0),
            lift_scale=0.0,
            effective_offset=0.0,
        )
        assert location.source_feature == "EDGE"
        assert location.source_feature_id == 32
        keys.append(key)

    assert keys == [
        ("pv-se", 32, 0.0559943),
        ("pv-se", 32, 0.9440057),
    ]


@pytest.mark.parametrize("lift_scale", (0.0, 0.5, 1.0))
def test_c8_6_cross_surface_sync_keeps_complementary_stations_unique(
    lift_scale,
):
    from cftuv import decals as decals_module

    domain = _c8_6_planar_domain()
    surface = SimpleNamespace(domain=domain, patch_id=12)
    site = _c8_6_reversed_site()
    plan = _c8_6_lift_plan(site)
    low_parameter = 0.0559943
    high_parameter = 0.9440057
    low_position, low_key, _location = decal_voronoi._position_and_key(
        plan,
        surface,
        site,
        (low_parameter, 0.0),
        lift_scale=lift_scale,
        effective_offset=0.0,
    )
    high_location = domain.locate((high_parameter, 0.0))
    high_key = decal_voronoi._domain_location_key(surface, high_location)
    high_position = Vector(
        (high_parameter, 0.0, 0.1 * float(lift_scale))
    )
    assert low_key != high_key

    def face(surface_id, station_key, station_position, interior_position):
        return decal_voronoi._NetworkFace(
            surface_id=surface_id,
            surface_normal=Vector((0.0, 0.0, 1.0)),
            vert_keys=[
                ("pv-sv", 10),
                station_key,
                ("pv-sv", 20),
                ("pv", surface_id, 1, 1),
            ],
            positions=[
                Vector((0.0, 0.0, 0.1 * float(lift_scale))),
                station_position,
                Vector((1.0, 0.0, 0.1 * float(lift_scale))),
                interior_position,
            ],
            u_fracs=[0.0, station_position.x, 1.0, 0.0],
            v_lengths=[0.0, station_position.x, 1.0, 1.0],
        )

    faces = [
        face(0, low_key, low_position, Vector((0.0, 1.0, 0.0))),
        face(1, high_key, high_position, Vector((0.0, -1.0, 0.0))),
    ]
    decal_voronoi._synchronize_cross_surface_spine_stations(plan, faces)

    for network_face in faces:
        assert network_face.vert_keys.count(low_key) == 1
        assert network_face.vert_keys.count(high_key) == 1
        assert len(network_face.vert_keys) == len(set(network_face.vert_keys))
    decals_module._validate_network_faces_for_materialization(
        faces, "PATCH_VORONOI", (32,)
    )


def test_c8_2_planar_compile_keeps_unselected_boundary_provenance():
    points = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0, 0.0, 0.0)),
        Vector((2.0, 1.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
    ]
    node = PatchNode(
        patch_id=12,
        face_indices=[12],
        centroid=sum(points, Vector()) / 4.0,
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        mesh_verts=points,
        mesh_tris=[(0, 1, 2), (0, 2, 3)],
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=[0, 1, 2, 3],
            vert_cos=points,
            edge_indices=[80, 81, 82, 83],
            chains=[
                BoundaryChain(
                    vert_indices=[0, 1],
                    vert_cos=points[:2],
                    edge_indices=[80],
                )
            ],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)

    plan = compile_patch_voronoi_plan(graph, [80], offset=0.01)
    domain = plan.surfaces[0].domain
    assert {record[0] for record in domain.planar_source_edges} == {
        80,
        81,
        82,
        83,
    }
    edge_record = next(
        record for record in domain.planar_source_edges if record[0] == 83
    )
    unselected_point = (
        (edge_record[3][0] + edge_record[4][0]) * 0.5,
        (edge_record[3][1] + edge_record[4][1]) * 0.5,
    )
    unselected = domain.locate(unselected_point)
    assert (unselected.source_feature, unselected.source_feature_id) == (
        "EDGE",
        83,
    )
    assert decal_voronoi._domain_location_key(
        plan.surfaces[0], unselected
    ) == ("pv-se", 83, 0.5)


def test_b2_intrinsic_location_matches_reference_full_scan():
    chart_triangles = (
        ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
        ((10.0, 0.0), (11.0, 0.0), (10.0, 1.0)),
    )
    intrinsic_triangles = tuple(
        decal_voronoi._IntrinsicDomainTriangle(
            chart_points=chart_points,
            positions=tuple(
                Vector((point[0], point[1], float(triangle_id)))
                for point in chart_points
            ),
            normals=(Vector((0.0, 0.0, 1.0)),) * 3,
        )
        for triangle_id, chart_points in enumerate(chart_triangles)
    )

    def domain(reference_full_scan):
        return decal_voronoi.DecalSurfaceDomain(
            patch_id=0,
            kind="INTRINSIC",
            origin=Vector((0.0, 0.0, 0.0)),
            reference_normal=Vector((0.0, 0.0, 1.0)),
            basis_u=Vector((1.0, 0.0, 0.0)),
            basis_v=Vector((0.0, 1.0, 0.0)),
            boundary_triangles=chart_triangles,
            intrinsic_triangles=intrinsic_triangles,
            reference_full_scan=reference_full_scan,
        )

    point = (10.25, 0.25)
    indexed = domain(False)
    reference = domain(True)
    assert indexed.triangle_grid.query_point(point) == (1,)
    assert tuple(indexed.lift(point, 0.2)) == pytest.approx(
        tuple(reference.lift(point, 0.2))
    )


def test_b3_domain_location_classifies_source_features_and_transitions():
    source_positions = (
        Vector((0.0, 0.0, 0.0)),
        Vector((1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
    )
    triangle = decal_voronoi._IntrinsicDomainTriangle(
        chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
        positions=source_positions,
        normals=(Vector((0.0, 0.0, 1.0)),) * 3,
        face_normal=Vector((0.0, 0.0, 1.0)),
        source_triangle_id=500,
        source_edge_ids=(20, 21, 22),
        source_vertex_ids=(10, 11, 12),
        edge_transition_keys=(None, None, "cut-22"),
        vertex_transition_keys=("cut-10", None, None),
    )
    domain = decal_voronoi.DecalSurfaceDomain(
        patch_id=1,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(triangle.chart_points,),
        intrinsic_triangles=(triangle,),
        chart_id=7,
    )

    interior = domain.locate((0.25, 0.25))
    edge = domain.locate((0.25, 0.0))
    vertex = domain.locate((0.0, 0.0))
    assert (
        interior.chart_id,
        interior.triangle_id,
        interior.source_feature,
        interior.source_feature_id,
    ) == (7, 0, "TRIANGLE", 500)
    assert (edge.source_feature, edge.source_feature_id) == ("EDGE", 22)
    assert edge.transition_key == "cut-22"
    assert (vertex.source_feature, vertex.source_feature_id) == (
        "VERTEX",
        10,
    )
    assert vertex.transition_key == "cut-10"


def test_b3_hard_fold_lift_and_chart_cut_identity_are_shared():
    normal_a = Vector((0.0, 0.0, 1.0))
    normal_b = Vector((0.0, 1.0, 0.0))
    source_a = Vector((0.0, 0.0, 0.0))
    source_b = Vector((1.0, 0.0, 0.0))
    triangle_a = decal_voronoi._IntrinsicDomainTriangle(
        chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
        positions=(source_a, source_b, Vector((0.0, 1.0, 0.0))),
        normals=(normal_a,) * 3,
        face_normal=normal_a,
        source_triangle_id=30,
        source_edge_ids=(None, None, 77),
        source_vertex_ids=(100, 101, 102),
        edge_transition_keys=(None, None, "fold-cut"),
        vertex_transition_keys=("vertex-cut", None, None),
    )
    triangle_b = decal_voronoi._IntrinsicDomainTriangle(
        chart_points=((10.0, 0.0), (11.0, 0.0), (10.0, -1.0)),
        positions=(source_a, source_b, Vector((0.0, 0.0, 1.0))),
        normals=(normal_b,) * 3,
        face_normal=normal_b,
        source_triangle_id=31,
        source_edge_ids=(None, None, 77),
        source_vertex_ids=(100, 101, 103),
        edge_transition_keys=(None, None, "fold-cut"),
        vertex_transition_keys=("vertex-cut", None, None),
    )
    domain = decal_voronoi.DecalSurfaceDomain(
        patch_id=1,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=normal_a,
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(
            triangle_a.chart_points,
            triangle_b.chart_points,
        ),
        intrinsic_triangles=(triangle_a, triangle_b),
        chart_id=9,
    )
    first = domain.locate((0.25, 0.0))
    copied = domain.locate((10.25, 0.0))
    surface = SimpleNamespace(domain=domain, patch_id=1)

    assert first.source_feature == copied.source_feature == "EDGE"
    assert first.source_feature_id == copied.source_feature_id == 77
    assert decal_voronoi._domain_location_key(
        surface, first
    ) == decal_voronoi._domain_location_key(surface, copied)
    expected = Vector((0.25, 0.2, 0.2))
    assert tuple(domain.lift(first.uv, 0.2, first)) == pytest.approx(
        tuple(expected)
    )
    assert tuple(domain.lift(copied.uv, 0.2, copied)) == pytest.approx(
        tuple(expected)
    )


def test_b3_source_vertex_uses_all_incident_owner_planes():
    normals = (
        Vector((1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
        Vector((0.0, 0.0, 1.0)),
    )
    triangles = tuple(
        decal_voronoi._IntrinsicDomainTriangle(
            chart_points=(
                (float(index * 10), 0.0),
                (float(index * 10 + 1), 0.0),
                (float(index * 10), 1.0),
            ),
            positions=(
                Vector((0.0, 0.0, 0.0)),
                Vector((1.0, 0.0, 0.0)),
                Vector((0.0, 1.0, 0.0)),
            ),
            normals=(normal,) * 3,
            face_normal=normal,
            source_triangle_id=index,
            source_vertex_ids=(900, 901 + index * 2, 902 + index * 2),
            vertex_transition_keys=("vertex-900", None, None),
        )
        for index, normal in enumerate(normals)
    )
    domain = decal_voronoi.DecalSurfaceDomain(
        patch_id=1,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=tuple(
            triangle.chart_points for triangle in triangles
        ),
        intrinsic_triangles=triangles,
    )
    location = domain.locate((0.0, 0.0))

    assert location.source_feature == "VERTEX"
    assert location.source_feature_id == 900
    assert tuple(domain.lift(location.uv, 0.2, location)) == pytest.approx(
        (0.2, 0.2, 0.2)
    )


def test_wide_t_junction_cells_remain_simple_and_non_overlapping():
    graph, edge_indices = _wide_t_junction_front_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.02)
    assert plan is not None

    for width in (1.0545852, 2.0, 4.0):
        faces = evaluate_patch_voronoi_plan(plan, width=width, preview=True)
        polygons = [
            [(point.x, point.y) for point in face.positions] for face in faces
        ]
        assert polygons
        assert all(decal_voronoi._polygon_is_simple(polygon) for polygon in polygons)
        for index, polygon in enumerate(polygons):
            for other in polygons[index + 1 :]:
                for triangle in decal_voronoi._triangulate_cell_polygon(
                    polygon
                ):
                    for other_triangle in decal_voronoi._triangulate_cell_polygon(
                        other
                    ):
                        intersection = decal_voronoi._clip_to_convex(
                            triangle, other_triangle
                        )
                        if intersection:
                            assert abs(
                                decal_voronoi._polygon_area2(intersection)
                            ) <= 1e-5


def test_wide_t_junction_endpoint_front_never_loses_covered_point():
    """Параболическая хорда не должна открывать мигрирующую дырку при drag."""

    graph, edge_indices = _wide_t_junction_front_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.02)
    surface = plan.surfaces[0]
    witness = (1.07068, 1.96819)

    def inside_polygon(point, polygon):
        inside = False
        for index, point_a in enumerate(polygon):
            point_b = polygon[(index + 1) % len(polygon)]
            if (point_a[1] > point[1]) == (point_b[1] > point[1]):
                continue
            crossing_x = point_a[0] + (
                (point[1] - point_a[1])
                * (point_b[0] - point_a[0])
                / (point_b[1] - point_a[1])
            )
            if crossing_x > point[0]:
                inside = not inside
        return inside

    distance = min(
        decal_voronoi._segment_point_distance2(
            site.point_a, site.point_b, witness
        )[0]
        for site in surface.sites
    )
    for width in (1.10, 1.15, 1.20):
        assert distance < width * 0.5
        faces = evaluate_patch_voronoi_plan(
            plan,
            width=width,
            preview=True,
            corner_settings=decal_voronoi.CornerRuntimeSettings(
                dynamic_corner_bands=False
            ),
        )
        polygons = [
            [
                (
                    (position - surface.origin).dot(surface.basis_u),
                    (position - surface.origin).dot(surface.basis_v),
                )
                for position in face.positions
            ]
            for face in faces
        ]
        assert any(inside_polygon(witness, polygon) for polygon in polygons), (
            f"width {width}: покрытая endpoint-зона исчезла между "
            "Voronoi chord и segment front"
        )


@pytest.mark.parametrize(
    ("polygon", "expected"),
    [
        (
            [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 1.0), (0.0, 2.0)],
            True,
        ),
        (
            [(0.0, 0.0), (3.0, 2.0), (0.0, 2.0), (3.0, 0.0)],
            False,
        ),
    ],
)
def test_polygon_sweep_preserves_exact_intersection_result(polygon, expected):
    assert decal_voronoi._polygon_is_simple(polygon) is expected


def test_polygon_sweep_accepts_dense_convex_cell():
    polygon = [
        (cos(index * 2.0 * pi / 512), sin(index * 2.0 * pi / 512))
        for index in range(512)
    ]
    assert decal_voronoi._polygon_is_simple(polygon)


def test_patch_voronoi_fragment_union_removes_internal_triangulation():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
            [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        ]
    )
    assert len(components) == 1
    assert len(components[0]) == 4
    assert abs(sum(
        point[0] * components[0][(index + 1) % 4][1]
        - components[0][(index + 1) % 4][0] * point[1]
        for index, point in enumerate(components[0])
    )) * 0.5 == pytest.approx(4.0)


def test_g1_fragment_union_preserves_non_coplanar_source_face_imprint():
    fragments = [
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
        [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
    ]

    assert len(
        _merge_polygon_fragments(fragments, merge_groups=(0, 0))
    ) == 1
    separated = _merge_polygon_fragments(
        fragments, merge_groups=(0, 1)
    )

    assert len(separated) == 2
    assert sum(
        abs(decal_voronoi._polygon_area2(component))
        for component in separated
    ) == pytest.approx(4.0)


def test_g1_triangle_groups_merge_only_same_face_or_coplanar_adjacency():
    def triangle(source_face_id, face_normal, shared_edge):
        return decal_voronoi._IntrinsicDomainTriangle(
            chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
            positions=(Vector(), Vector(), Vector()),
            normals=(face_normal,) * 3,
            face_normal=face_normal,
            source_face_id=source_face_id,
            source_edge_ids=(shared_edge, None, None),
        )

    same_face = (
        triangle(10, Vector((0.0, 0.0, 1.0)), ("diag", 1)),
        triangle(10, Vector((0.0, 1.0, 0.0)), ("diag", 1)),
    )
    coplanar_faces = (
        triangle(10, Vector((0.0, 0.0, 1.0)), 51),
        triangle(11, Vector((0.0, 0.0, 1.0)), 51),
    )
    folded_faces = (
        triangle(10, Vector((0.0, 0.0, 1.0)), 52),
        triangle(11, Vector((0.0, 1.0, 0.0)), 52),
    )

    assert decal_voronoi._intrinsic_triangle_merge_groups(same_face) == (
        0,
        0,
    )
    assert decal_voronoi._intrinsic_triangle_merge_groups(
        coplanar_faces
    ) == (0, 0)
    assert decal_voronoi._intrinsic_triangle_merge_groups(
        folded_faces
    ) == (0, 1)


def test_fragment_union_normalizes_t_junction_before_edge_cancellation():
    """Разное Boost-разбиение общей грани не создаёт runtime topology."""

    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0)],
            [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0)],
            [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)],
        ],
        tolerance=1e-7,
        normalize_t_junctions=True,
    )

    assert len(components) == 1
    assert len(components[0]) == 4
    assert abs(decal_voronoi._polygon_area2(components[0])) == pytest.approx(
        4.0
    )


def test_fragment_union_preserves_simple_concave_ngon():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
            [(0.0, 0.0), (2.0, 1.0), (1.0, 1.0)],
            [(0.0, 0.0), (1.0, 1.0), (1.0, 2.0)],
            [(0.0, 0.0), (1.0, 2.0), (0.0, 2.0)],
        ]
    )
    assert len(components) == 1
    assert len(components[0]) == 6
    assert decal_voronoi._polygon_is_simple(components[0])
    assert not decal_voronoi._polygon_is_convex(components[0])


def test_fragment_union_falls_back_when_traced_loop_loses_area():
    """Production T-junction stations не должны выбрасывать полосу union."""

    fragments = [
        [
            (-3.8092498, 1.506048),
            (-3.83964577, 1.50594658),
            (-3.83964577, 1.47427826),
            (-3.83960345, 1.47427826),
        ],
        [
            (-3.7811233, 1.50614185),
            (-3.8092498, 1.506048),
            (-3.83960345, 1.47427826),
            (-3.83956453, 1.47427826),
        ],
        [
            (-3.34820625, 1.98860064),
            (-3.34820625, 2.65209438),
            (-3.83964577, 2.59418529),
            (-3.83964577, 1.50594658),
            (-3.8092498, 1.506048),
        ],
        [
            (-3.34820625, 1.98860064),
            (-3.8092498, 1.506048),
            (-3.7811233, 1.50614185),
            (-3.34820625, 1.74217883),
        ],
    ]

    components = _merge_polygon_fragments(fragments, tolerance=0.00025)
    source_area = sum(
        abs(decal_voronoi._polygon_area2(fragment))
        for fragment in fragments
    )
    merged_area = sum(
        abs(decal_voronoi._polygon_area2(component))
        for component in components
    )

    assert merged_area == pytest.approx(source_area, rel=1e-5)
    assert all(
        decal_voronoi._polygon_is_simple(component)
        for component in components
    )


def test_pending_fragment_union_does_not_expose_pre_collision_triangulation():
    """Runtime topology tolerance сохраняет близкие production stations."""

    fragments = [
        [
            (-3.8092498, 1.506048),
            (-3.83964577, 1.50594658),
            (-3.83964577, 1.47427826),
            (-3.83960345, 1.47427826),
        ],
        [
            (-3.7811233, 1.50614185),
            (-3.8092498, 1.506048),
            (-3.83960345, 1.47427826),
            (-3.83956453, 1.47427826),
        ],
        [
            (-3.34820625, 1.98860064),
            (-3.34820625, 2.65209438),
            (-3.83964577, 2.59418529),
            (-3.83964577, 1.50594658),
            (-3.8092498, 1.506048),
        ],
        [
            (-3.34820625, 1.98860064),
            (-3.8092498, 1.506048),
            (-3.7811233, 1.50614185),
            (-3.34820625, 1.74217883),
        ],
    ]
    pending = []

    decal_voronoi._append_pending_fragments(
        pending,
        surface=object(),
        site=object(),
        crop=object(),
        fragments=fragments,
    )

    assert len(pending) == 1
    assert decal_voronoi._polygon_is_simple(pending[0].points)
    assert abs(decal_voronoi._polygon_area2(pending[0].points)) == pytest.approx(
        sum(abs(decal_voronoi._polygon_area2(fragment)) for fragment in fragments),
        rel=1e-8,
    )


def test_fragment_union_keeps_point_contact_as_separate_components():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
            [(0.0, 0.0), (-1.0, 0.0), (0.0, -1.0)],
        ]
    )
    assert len(components) == 2


def test_arrangement_splits_point_on_neighbor_edge_before_materialization():
    polygons, inserted = decal_voronoi._insert_surface_edge_stations(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
            [(0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)],
        ],
        tolerance=1e-7,
    )
    assert inserted == 1
    assert polygons[0] == [
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (0.0, 1.0),
    ]
    assert (1.0, 1.0) in polygons[1]


def test_patch_domain_uses_boundary_loop_instead_of_owner_triangulation(
    monkeypatch,
):
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((1.0, 1.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0, 0.0, 0.0)),
        Vector((2.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
    ]
    # Намеренно неполная owner topology: старый backend потерял бы половину
    # patch. Boundary-domain должен полностью её игнорировать.
    node.mesh_tris = [(0, 1, 2)]
    node.boundary_loops = [
        BoundaryLoop(vert_cos=[point.copy() for point in node.mesh_verts])
    ]
    monkeypatch.setattr(
        decal_voronoi,
        "_tessellate_polygon",
        lambda _loops: [(0, 1, 2), (0, 2, 3)],
    )

    triangles = decal_voronoi._patch_domain_triangles(
        node,
        node.centroid,
        node.basis_u,
        node.basis_v,
    )
    assert len(triangles) == 2
    assert sum(
        abs(decal_voronoi._polygon_area2(tri))
        for tri in triangles
    ) == pytest.approx(4.0)


def test_patch_voronoi_rejects_non_planar_owner_patch():
    graph = _planar_two_site_graph()
    graph.nodes[0].mesh_verts[2] = Vector((4.0, 2.0, 0.2))
    assert compile_patch_voronoi_plan(graph, [10, 12], offset=0.01) is None
    attempt = decal_voronoi.compile_patch_voronoi_attempt(
        graph, [10, 12], offset=0.01, allow_partial=True
    )
    assert attempt.plan is None
    assert attempt.rejected_edge_indices == (10, 12)
    assert [(failure.patch_id, failure.reason) for failure in attempt.failures] == [
        (0, "MISSING_SITE_FACE_PROVENANCE")
    ]


def test_warped_owner_face_uses_serialized_face_provenance():
    graph = _planar_two_site_graph()
    node = graph.nodes[0]
    node.mesh_verts[2] = Vector((4.0, 2.0, 0.01))
    node.mesh_vert_indices = list(range(len(node.mesh_verts)))
    node.mesh_tri_face_indices = [17, 17]
    node.mesh_tri_face_normals = [
        Vector((0.0, -0.0025, 1.0)).normalized(),
        Vector((0.0, -0.0025, 1.0)).normalized(),
    ]
    for chain in node.boundary_loops[0].chains:
        chain.side_face_indices = [17]
        chain.side_face_normals = [node.mesh_tri_face_normals[0].copy()]

    plan = compile_patch_voronoi_plan(graph, [10, 12], offset=0.01)

    assert plan is not None
    assert len(plan.surfaces) == 1
    assert sorted(site.edge_index for site in plan.surfaces[0].sites) == [10, 12]
    assert evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)


def test_non_planar_topology_patch_compiles_real_planar_owner_surfaces():
    plan = compile_patch_voronoi_plan(
        _single_patch_fold_graph(), [70, 71], offset=0.01
    )
    assert plan is not None
    assert plan.backend_kind == "INTRINSIC_DEVELOPABLE"
    assert len(plan.surfaces) == 1
    assert len(plan.surfaces[0].sites) == 2
    assert plan.surfaces[0].domain.kind == "INTRINSIC"
    faces = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    assert faces
    assert all(face.component_kind != "JUNCTION" for face in faces)


def test_planar_crop_topology_is_stable_for_reversed_normal_and_float_noise():
    base, selected = _door_opening_graph()
    perturbed = deepcopy(base)
    node = perturbed.nodes[0]
    node.normal *= -1.0
    offsets = {
        index: Vector(
            (
                ((index % 3) - 1) * 3e-6,
                (((index + 1) % 3) - 1) * 2e-6,
                0.0,
            )
        )
        for index in range(len(node.mesh_verts))
    }
    node.mesh_verts = [
        point + offsets[index]
        for index, point in enumerate(node.mesh_verts)
    ]
    node.centroid = (
        sum(node.mesh_verts, Vector((0.0, 0.0, 0.0)))
        / len(node.mesh_verts)
    )
    for loop in node.boundary_loops:
        loop.vert_cos = [
            point + offsets[vert_index]
            for vert_index, point in zip(loop.vert_indices, loop.vert_cos)
        ]
        for chain in loop.chains:
            chain.vert_cos = [
                point + offsets[vert_index]
                for vert_index, point in zip(
                    chain.vert_indices, chain.vert_cos
                )
            ]

    base_plan = compile_patch_voronoi_plan(base, selected, offset=0.01)
    perturbed_plan = compile_patch_voronoi_plan(
        perturbed, selected, offset=0.01
    )
    assert base_plan is not None
    assert perturbed_plan is not None
    for width in (0.418, 1.0, 2.5):
        base_signature = tuple(
            len(face.positions)
            for face in evaluate_patch_voronoi_plan(base_plan, width)
        )
        perturbed_signature = tuple(
            len(face.positions)
            for face in evaluate_patch_voronoi_plan(perturbed_plan, width)
        )
        assert perturbed_signature == base_signature


def test_preview_and_confirm_keep_identical_miter_geometry():
    graph = _planar_two_site_graph()
    graph.nodes[0].boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[4, 5],
                    vert_cos=[
                        Vector((1.0, 0.0, 0.0)),
                        Vector((3.0, 0.0, 0.0)),
                    ],
                    edge_indices=[20],
                )
            ]
        )
    ]
    plan = compile_patch_voronoi_plan(graph, [20], offset=0.01)
    preview = evaluate_patch_voronoi_plan(plan, width=1.0, preview=True)
    confirmed = evaluate_patch_voronoi_plan(plan, width=1.0, preview=False)

    def signature(faces):
        return [
            (
                tuple(face.vert_keys),
                tuple(
                    tuple(round(value, 9) for value in point)
                    for point in face.positions
                ),
            )
            for face in faces
        ]

    assert signature(preview) == signature(confirmed)


def test_narrow_corner_offset_cannot_invert_owner_wings():
    plan = compile_patch_voronoi_plan(
        _orthogonal_corner_graph(), [30], offset=0.2
    )
    faces = evaluate_patch_voronoi_plan(plan, width=0.01, preview=True)
    assert faces
    assert min(_signed_face_area(face) for face in faces) >= -1e-12


def test_folded_turn_builds_one_realtime_junction_sector():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    faces = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    connectors = [
        face
        for face in faces
        if face.surface_id == -1 and face.vert_keys[0] == ("pv-sv", 0)
    ]
    assert len(connectors) == 1

    edge_uses = {}
    for face in faces:
        for index, key_a in enumerate(face.vert_keys):
            key_b = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge_key = tuple(sorted((key_a, key_b), key=repr))
            edge_uses[edge_key] = edge_uses.get(edge_key, 0) + 1
    open_rail_edges = [
        edge_key
        for edge_key, use_count in edge_uses.items()
        if use_count == 1
        and ("pv-sv", 0) in edge_key
        and not all(key[:1] == ("pv-sv",) for key in edge_key)
    ]
    assert not open_rail_edges

    confirmed = evaluate_patch_voronoi_plan(plan, width=0.5, preview=False)
    assert [
        (
            tuple(face.vert_keys),
            tuple(tuple(round(value, 9) for value in point) for point in face.positions),
        )
        for face in faces
    ] == [
        (
            tuple(face.vert_keys),
            tuple(tuple(round(value, 9) for value in point) for point in face.positions),
        )
        for face in confirmed
    ]
