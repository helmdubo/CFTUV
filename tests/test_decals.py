from __future__ import annotations

import inspect
from math import pi
from types import SimpleNamespace

import pytest
from mathutils import Vector

from cftuv import decals as decals_module

from cftuv.decals import (
    _OrientedCornerRun,
    _OrientedRibbonRun,
    _boundary_wing_direction,
    _collect_manual_chain_decals,
    _collect_manual_edge_decals,
    _collect_trim_ribbon_runs,
    _collect_wall_pair_chains,
    _corner_offset_join,
    _corner_wing_directions,
    _dedupe_polyline,
    _junction_miter_position,
    _offset_plane_junction_center,
    _polyline_tangents,
    _prepare_seam_junctions,
    _ribbon_vertex_frames,
    _stitch_corner_runs,
    _stitch_ribbon_runs,
    _trim_run_for_junctions,
    _trim_quad_layout,
    _trim_quad_requires_flip,
    chain_refs_for_edge_indices,
)
from cftuv.model import (
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
    PatchType,
    DecalSettings,
)


def _make_wall_node(patch_id, normal, basis_v, chains):
    node = PatchNode(patch_id=patch_id, face_indices=[patch_id * 100])
    node.patch_type = PatchType.WALL
    node.normal = Vector(normal)
    node.basis_v = Vector(basis_v)
    node.boundary_loops = [BoundaryLoop(chains=chains)]
    return node


def _backend_test_run(edge_indices, vert_indices):
    segment_count = len(edge_indices)
    return _OrientedCornerRun(
        vert_indices=list(vert_indices),
        points=[Vector((float(index), 0.0, 0.0)) for index in vert_indices],
        segment_normals_a=[Vector((0.0, 0.0, 1.0))] * segment_count,
        segment_normals_b=[Vector((0.0, 1.0, 0.0))] * segment_count,
        segment_convexities=[0.0] * segment_count,
        segment_edge_indices=list(edge_indices),
    )


def test_rm9_terminal_routing_reports_plan_choice_and_materializing_backend():
    run = _backend_test_run((10,), (1, 2))
    partition = SimpleNamespace(
        backend="PATCH_VORONOI",
        compiled_plan=SimpleNamespace(backend_kind="INTRINSIC_DEVELOPABLE"),
        edge_indices=(10,),
        corner_runs=(run,),
        boundary_runs=(),
    )
    rail_plan = SimpleNamespace(
        edges=(
            SimpleNamespace(edge_id=20, is_pchain=True),
            SimpleNamespace(edge_id=21, is_pchain=True),
            SimpleNamespace(edge_id=30, is_pchain=False),
        ),
        routes=(
            SimpleNamespace(
                route_id=7,
                segments=(
                    SimpleNamespace(edge_id=20),
                    SimpleNamespace(edge_id=21),
                ),
            ),
            SimpleNamespace(
                route_id=8,
                segments=(SimpleNamespace(edge_id=30),),
            ),
        ),
        terminal_uses=(
            SimpleNamespace(
                spine_vertex_id=1,
                spine_edge_id=10,
                source_face_ids=(100,),
                kind=decals_module.RailTerminalKind.ROUTE,
                route_edge_id=20,
                route_id=7,
            ),
            SimpleNamespace(
                spine_vertex_id=2,
                spine_edge_id=10,
                source_face_ids=(101,),
                kind=decals_module.RailTerminalKind.ROUTE,
                route_edge_id=30,
                route_id=8,
            ),
            SimpleNamespace(
                spine_vertex_id=2,
                spine_edge_id=10,
                source_face_ids=(102,),
                kind=decals_module.RailTerminalKind.IN_PLANE_EMPTY,
                route_edge_id=None,
                route_id=None,
            ),
        ),
    )

    records = decals_module._manual_terminal_routing(
        SimpleNamespace(face_to_patch={100: 4, 101: 4, 102: 4}),
        rail_plan,
        (partition,),
    )

    assert [(record.choice, record.edge_ids) for record in records] == [
        ("PCHAIN", (20, 21)),
        ("FOLD", (30,)),
        ("PERP", ()),
    ]
    assert all(
        record.backend == "PATCH_VORONOI"
        and record.backend_kind == "INTRINSIC_DEVELOPABLE"
        for record in records
    )
    assert records[0].report_line == (
        "component=0 patch=4 terminal=v1/e10 faces=100 choice=PCHAIN 20,21 "
        "backend=PATCH_VORONOI/INTRINSIC_DEVELOPABLE plan=ROUTE"
    )


def test_manual_seam_plan_rejects_failed_topology_component_atomically(
    monkeypatch,
):
    joined = _backend_test_run((1, 2), (0, 1, 2))
    isolated = _backend_test_run((10,), (10, 11))

    def collect(_graph, edge_indices):
        edges = set(edge_indices)
        runs = []
        if edges.intersection({1, 2}):
            runs.append(joined)
        if 10 in edges:
            runs.append(isolated)
        return decals_module._ManualEdgeDecalCollection(
            corner_runs=tuple(runs),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        )

    accepted_plan = SimpleNamespace(backend_kind="PLANAR")
    monkeypatch.setattr(decals_module, "_collect_manual_edge_decals", collect)
    monkeypatch.setattr(
        decals_module,
        "compile_decal_rail_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(plan=None, failures=()),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(
            plan=object(),
            rejected_edge_indices=(1,),
            failures=(
                SimpleNamespace(
                    patch_id=99,
                    reason="NO_OWNER_SURFACES",
                    edge_indices=(1,),
                ),
            ),
        ),
    )
    strict_calls = []
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_plan",
        lambda _graph, edges, _offset, **_kwargs: (
            strict_calls.append(tuple(edges)) or accepted_plan
        ),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(), DecalSettings(), (1, 2, 10)
    )

    assert strict_calls == [(10,)]
    assert [partition.backend for partition in plan.backend_partitions] == [
        "PATCH_VORONOI",
    ]
    assert plan.backend_partitions[0].edge_indices == (10,)
    assert plan.backend_partitions[0].topology_component_count == 1
    assert plan.accepted_legacy_edge_indices == ()
    assert plan.rejected_edge_indices == (1, 2)
    assert {
        rejection.edge_index: rejection.reason
        for rejection in plan.rejected_edges
    } == {1: "NO_OWNER_SURFACES", 2: "NO_OWNER_SURFACES"}
    assert plan.accounting_is_exact is True
    assert plan.backend_summary == (
        "PLANAR:1c/1e | Unsupported[NO_OWNER_SURFACES:x1] | "
        "Failed:2e[NO_OWNER_SURFACES:x2]"
    )


@pytest.mark.parametrize("backend", ("LEGACY_NETWORK", "UNKNOWN"))
def test_manual_seam_backend_partition_rejects_disabled_backend(backend):
    with pytest.raises(ValueError, match="backend is disabled"):
        decals_module._ManualSeamBackendPartition(
            backend=backend,
            edge_indices=(1,),
            topology_component_count=1,
            corner_runs=(),
            boundary_runs=(),
            compiled_plan=object(),
        )


@pytest.mark.parametrize(
    "legacy_fields",
    (
        {"network_plan": object()},
        {"direct_legacy_edge_indices": (1,)},
    ),
)
def test_manual_seam_plan_rejects_legacy_fields(legacy_fields):
    with pytest.raises(ValueError, match="Legacy SEAMS plans are disabled"):
        decals_module.ManualSeamDecalPlan(
            corner_runs=(),
            boundary_runs=(),
            selected_edge_indices=(1,),
            **legacy_fields,
        )


def test_unknown_backend_evaluation_fails_visibly():
    partition = SimpleNamespace(backend="UNKNOWN", edge_indices=(3,))

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._evaluate_manual_backend_partition(
            partition,
            DecalSettings(),
            width=1.0,
            preview=False,
        )

    assert exc_info.value.edge_indices == (3,)
    assert exc_info.value.reason == "UNSUPPORTED_BACKEND:UNKNOWN"


@pytest.mark.parametrize(
    ("edge_indices", "expected_ids"),
    (
        ((7, 3, 7), "[3,7]"),
        (tuple(range(15)), "[0,1,2,3,4,5,6,7,8,9,10,11,...]"),
    ),
)
def test_strict_seam_runtime_error_lists_at_most_twelve_edge_ids(
    edge_indices,
    expected_ids,
):
    error = decals_module.StrictSeamRuntimeError(
        edge_indices,
        "UNSUPPORTED_TEST_SCOPE",
    )

    assert expected_ids in str(error)
    assert str(error).endswith(": UNSUPPORTED_TEST_SCOPE")
    assert error.edge_indices == tuple(sorted(set(edge_indices)))


def test_strict_seams_do_not_reference_legacy_compilers_or_evaluators():
    legacy_symbols = {
        "build_seam_network_faces",
        "compile_seam_network_plan",
        "evaluate_seam_network_plan",
    }
    strict_runtime_source = "\n".join(
        inspect.getsource(function)
        for function in (
            decals_module.compile_manual_seam_decal_plan,
            decals_module._evaluate_manual_backend_partition,
            decals_module._fill_manual_chain_decals,
            decals_module._fill_decal_bmesh,
        )
    )

    assert legacy_symbols.isdisjoint(vars(decals_module))
    assert all(
        symbol not in strict_runtime_source for symbol in legacy_symbols
    )


@pytest.mark.parametrize("seam_network", (False, True))
def test_r0_manual_compile_ignores_legacy_toggle_and_attempts_modern_backend(
    monkeypatch,
    seam_network,
):
    run = _backend_test_run((7,), (0, 1))
    monkeypatch.setattr(
        decals_module,
        "_collect_manual_edge_decals",
        lambda _graph, _edges: decals_module._ManualEdgeDecalCollection(
            corner_runs=(run,),
            boundary_runs=(),
            accepted_edge_indices=(7,),
            rejected_edges=(),
        ),
    )
    rail_failure = SimpleNamespace(reason="TEST_RAIL_DIAGNOSTIC")
    rail_calls = []

    def compile_rails(graph, edges, **kwargs):
        rail_calls.append((graph, tuple(edges), kwargs))
        return SimpleNamespace(plan=None, failures=(rail_failure,))

    monkeypatch.setattr(
        decals_module,
        "compile_decal_rail_attempt",
        compile_rails,
    )
    graph = object()
    patch_plan = SimpleNamespace(backend_kind="PLANAR")
    patch_calls = []
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda graph, edges, *_args, **kwargs: (
            patch_calls.append((graph, tuple(edges), kwargs))
            or SimpleNamespace(
                plan=patch_plan,
                rejected_edge_indices=(),
                failures=(),
            )
        ),
    )
    settings = DecalSettings(seam_network=seam_network)

    plan = decals_module.compile_manual_seam_decal_plan(
        graph,
        settings,
        (7,),
        alpha_budget=3.0,
        rail_mark_edge_indices=(99,),
    )

    assert rail_calls == [
        (
            graph,
            (7,),
            {
                "alpha_budget": 3.0,
                "rail_mark_edge_indices": (99,),
            },
        )
    ]
    assert [call[1] for call in patch_calls] == [(7,)]
    assert plan.rail_plan is None
    assert plan.rail_compile_failures == (rail_failure,)
    assert plan.accepted_patch_voronoi_edge_indices == (7,)
    assert plan.accepted_legacy_edge_indices == ()
    assert plan.rejected_edge_indices == ()
    assert plan.accounting_is_exact is True


def test_r1_mixed_success_scope_stays_on_joint_patch_competition(monkeypatch):
    joined = _backend_test_run((1, 2), (0, 1, 2))
    isolated = _backend_test_run((10,), (10, 11))

    def collect(_graph, edge_indices):
        edges = set(edge_indices)
        runs = []
        if edges.intersection({1, 2}):
            runs.append(joined)
        if 10 in edges:
            runs.append(isolated)
        return decals_module._ManualEdgeDecalCollection(
            corner_runs=tuple(runs),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        )

    rail_plan = object()
    rail_geometry_plan = object()
    rail_failure = SimpleNamespace(reason="NON_PLANAR_RAIL_COMPONENT")
    geometry_calls = []

    def compile_geometry(plan, *, edge_indices, **kwargs):
        geometry_calls.append((plan, tuple(edge_indices), kwargs))
        if tuple(edge_indices) == (10,):
            return SimpleNamespace(plan=rail_geometry_plan, failures=())
        return SimpleNamespace(plan=None, failures=(rail_failure,))

    patch_plan = SimpleNamespace(backend_kind="PLANAR")
    patch_calls = []
    monkeypatch.setattr(decals_module, "_collect_manual_edge_decals", collect)
    monkeypatch.setattr(
        decals_module,
        "compile_decal_rail_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(plan=rail_plan, failures=()),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_planar_rail_geometry_attempt",
        compile_geometry,
    )
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda _graph, edges, *_args, **_kwargs: (
            patch_calls.append(tuple(edges))
            or SimpleNamespace(
                plan=patch_plan,
                rejected_edge_indices=(),
                failures=(),
            )
        ),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(),
        DecalSettings(),
        (1, 2, 10),
        alpha_budget=3.0,
    )

    assert geometry_calls == [
        (
            rail_plan,
            (1, 2),
            {
                "apex_limit": 8.0,
                "split_angle": pytest.approx(pi / 3.0),
                "dynamic_corner_bands": False,
                "join_mode": "MITER",
            },
        ),
        (
            rail_plan,
            (10,),
            {
                "apex_limit": 8.0,
                "split_angle": pytest.approx(pi / 3.0),
                "dynamic_corner_bands": False,
                "join_mode": "MITER",
            },
        ),
    ]
    assert patch_calls == [(1, 2, 10)]
    assert [partition.backend for partition in plan.backend_partitions] == [
        "PATCH_VORONOI",
    ]
    assert plan.backend_partitions[0].edge_indices == (1, 2, 10)
    assert plan.accepted_rail_planar_edge_indices == ()
    assert plan.accepted_patch_voronoi_edge_indices == (1, 2, 10)
    assert plan.rail_geometry_failures == (rail_failure,)
    assert plan.accounting_is_exact is True
    assert plan.backend_summary == (
        "PLANAR:2c/3e | RailUnsupported[NON_PLANAR_RAIL_COMPONENT:x1]"
    )


def test_r1_face_disjoint_scope_can_use_multiple_rail_partitions(monkeypatch):
    joined = _backend_test_run((1, 2), (0, 1, 2))
    isolated = _backend_test_run((10,), (10, 11))

    def collect(_graph, edge_indices):
        edges = set(edge_indices)
        runs = []
        if edges.intersection({1, 2}):
            runs.append(joined)
        if 10 in edges:
            runs.append(isolated)
        return decals_module._ManualEdgeDecalCollection(
            corner_runs=tuple(runs),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        )

    def geometry_plan(owner_face_id):
        return SimpleNamespace(
            channels=(
                SimpleNamespace(
                    cells=(SimpleNamespace(owner_face_id=owner_face_id),)
                ),
            ),
            corner_partitions=(),
            path_reach_scales=(),
        )

    plans = {(1, 2): geometry_plan(100), (10,): geometry_plan(200)}
    rail_plan = SimpleNamespace(alpha_budget=3.0)
    monkeypatch.setattr(decals_module, "_collect_manual_edge_decals", collect)
    monkeypatch.setattr(
        decals_module,
        "compile_decal_rail_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(plan=rail_plan, failures=()),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_planar_rail_geometry_attempt",
        lambda _plan, *, edge_indices, **_kwargs: SimpleNamespace(
            plan=plans[tuple(edge_indices)], failures=()
        ),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda *_args, **_kwargs: pytest.fail("disjoint rail scope used Patch"),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(),
        DecalSettings(),
        (1, 2, 10),
        alpha_budget=3.0,
    )

    assert [partition.backend for partition in plan.backend_partitions] == [
        "RAIL_PLANAR",
        "RAIL_PLANAR",
    ]
    assert plan.accepted_rail_planar_edge_indices == (1, 2, 10)
    assert plan.accepted_patch_voronoi_edge_indices == ()
    assert plan.rail_geometry_failures == ()
    assert plan.accounting_is_exact is True


def test_r1_same_face_components_stay_on_joint_patch_competition(monkeypatch):
    joined = _backend_test_run((1, 2), (0, 1, 2))
    isolated = _backend_test_run((10,), (10, 11))

    def collect(_graph, edge_indices):
        edges = set(edge_indices)
        runs = []
        if edges.intersection({1, 2}):
            runs.append(joined)
        if 10 in edges:
            runs.append(isolated)
        return decals_module._ManualEdgeDecalCollection(
            corner_runs=tuple(runs),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        )

    shared_plan = SimpleNamespace(
        channels=(
            SimpleNamespace(cells=(SimpleNamespace(owner_face_id=100),)),
        ),
        corner_partitions=(),
    )
    patch_plan = SimpleNamespace(backend_kind="PLANAR")
    patch_calls = []
    monkeypatch.setattr(decals_module, "_collect_manual_edge_decals", collect)
    monkeypatch.setattr(
        decals_module,
        "compile_decal_rail_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(plan=object(), failures=()),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_planar_rail_geometry_attempt",
        lambda *_args, **_kwargs: SimpleNamespace(
            plan=shared_plan, failures=()
        ),
    )
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda _graph, edges, *_args, **_kwargs: (
            patch_calls.append(tuple(edges))
            or SimpleNamespace(
                plan=patch_plan,
                rejected_edge_indices=(),
                failures=(),
            )
        ),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(),
        DecalSettings(),
        (1, 2, 10),
        alpha_budget=3.0,
    )

    assert patch_calls == [(1, 2, 10)]
    assert [partition.backend for partition in plan.backend_partitions] == [
        "PATCH_VORONOI"
    ]
    assert plan.accepted_rail_planar_edge_indices == ()
    assert plan.accepted_patch_voronoi_edge_indices == (1, 2, 10)
    assert [failure.reason for failure in plan.rail_geometry_failures] == [
        "RAIL_GEOMETRY_COMPETITION_PENDING"
    ]
    assert plan.accounting_is_exact is True


def test_r1_scope_conflict_includes_wide_corner_owner_faces():
    def geometry_plan(channel_face_id, corner_face_id):
        return SimpleNamespace(
            channels=(
                SimpleNamespace(
                    cells=(SimpleNamespace(owner_face_id=channel_face_id),)
                ),
            ),
            corner_partitions=(),
            corner_cells=(SimpleNamespace(owner_face_id=corner_face_id),),
        )

    assert decals_module._rail_geometry_scope_has_face_conflicts(
        (
            ((1,), geometry_plan(100, 300)),
            ((2,), geometry_plan(200, 300)),
        )
    ) is True


def test_r1_miter_scale_recompiles_trace_to_preserve_effective_headroom(
    monkeypatch,
):
    run = _backend_test_run((7,), (0, 1))
    monkeypatch.setattr(
        decals_module,
        "_collect_manual_edge_decals",
        lambda _graph, edges: decals_module._ManualEdgeDecalCollection(
            corner_runs=(run,),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        ),
    )
    trace_scale = 5.758770483143633
    base_budget = 2.0
    expanded_budget = (
        base_budget * trace_scale
        + decals_module.DECAL_WELD_DISTANCE * 8.0
    )
    base_rail = SimpleNamespace(alpha_budget=base_budget)
    expanded_rail = SimpleNamespace(alpha_budget=expanded_budget)

    def geometry_plan(alpha_budget):
        return SimpleNamespace(
            channels=(
                SimpleNamespace(cells=(SimpleNamespace(owner_face_id=100),)),
            ),
            corner_partitions=(),
            path_reach_scales=((('corner', 7), trace_scale),),
            alpha_budget=alpha_budget,
        )

    base_geometry = geometry_plan(base_budget / trace_scale)
    expanded_geometry = geometry_plan(base_budget + 1.0e-6)
    rail_calls = []

    def compile_rail(_graph, _edges, **kwargs):
        rail_calls.append(kwargs["alpha_budget"])
        return SimpleNamespace(
            plan=(
                base_rail
                if len(rail_calls) == 1
                else expanded_rail
            ),
            failures=(),
        )

    geometry_calls = []

    def compile_geometry(plan, **_kwargs):
        geometry_calls.append(plan)
        return SimpleNamespace(
            plan=(
                base_geometry
                if plan is base_rail
                else expanded_geometry
            ),
            failures=(),
        )

    monkeypatch.setattr(decals_module, "compile_decal_rail_attempt", compile_rail)
    monkeypatch.setattr(
        decals_module,
        "compile_planar_rail_geometry_attempt",
        compile_geometry,
    )
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda *_args, **_kwargs: pytest.fail(
            "expanded rail support unexpectedly used Patch"
        ),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(),
        DecalSettings(corner_acute_split_angle=pi / 180.0),
        (7,),
        alpha_budget=base_budget,
    )

    assert rail_calls == pytest.approx([base_budget, expanded_budget])
    assert geometry_calls == [base_rail, expanded_rail]
    assert plan.rail_plan is expanded_rail
    assert plan.backend_partitions[0].backend == "RAIL_PLANAR"
    assert plan.backend_partitions[0].compiled_plan is expanded_geometry
    assert expanded_geometry.alpha_budget >= base_budget
    assert plan.rail_geometry_failures == ()
    assert plan.accounting_is_exact is True


def test_r1_extended_trace_failure_keeps_whole_scope_on_patch(monkeypatch):
    run = _backend_test_run((7,), (0, 1))
    monkeypatch.setattr(
        decals_module,
        "_collect_manual_edge_decals",
        lambda _graph, edges: decals_module._ManualEdgeDecalCollection(
            corner_runs=(run,),
            boundary_runs=(),
            accepted_edge_indices=tuple(sorted(edges)),
            rejected_edges=(),
        ),
    )
    base_rail = SimpleNamespace(alpha_budget=2.0)
    expanded_budget = 4.0 + decals_module.DECAL_WELD_DISTANCE * 8.0
    expanded_rail = SimpleNamespace(alpha_budget=expanded_budget)
    base_geometry = SimpleNamespace(
        channels=(
            SimpleNamespace(cells=(SimpleNamespace(owner_face_id=100),)),
        ),
        corner_partitions=(),
        path_reach_scales=((('corner', 7), 2.0),),
        alpha_budget=1.0,
    )
    expanded_failure = SimpleNamespace(
        reason="RAIL_GEOMETRY_MERGE_UNSUPPORTED"
    )
    rail_calls = []

    def compile_rail(_graph, _edges, **kwargs):
        rail_calls.append(kwargs["alpha_budget"])
        return SimpleNamespace(
            plan=base_rail if len(rail_calls) == 1 else expanded_rail,
            failures=(),
        )

    monkeypatch.setattr(decals_module, "compile_decal_rail_attempt", compile_rail)
    monkeypatch.setattr(
        decals_module,
        "compile_planar_rail_geometry_attempt",
        lambda plan, **_kwargs: (
            SimpleNamespace(plan=base_geometry, failures=())
            if plan is base_rail
            else SimpleNamespace(plan=None, failures=(expanded_failure,))
        ),
    )
    patch_plan = SimpleNamespace(backend_kind="PLANAR")
    patch_calls = []
    monkeypatch.setattr(
        decals_module,
        "compile_patch_voronoi_attempt",
        lambda _graph, edges, *_args, **_kwargs: (
            patch_calls.append(tuple(edges))
            or SimpleNamespace(
                plan=patch_plan,
                rejected_edge_indices=(),
                failures=(),
            )
        ),
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(),
        DecalSettings(),
        (7,),
        alpha_budget=2.0,
    )

    assert rail_calls == pytest.approx([2.0, expanded_budget])
    assert patch_calls == [(7,)]
    assert plan.accepted_rail_planar_edge_indices == ()
    assert plan.accepted_patch_voronoi_edge_indices == (7,)
    assert plan.rail_plan is base_rail
    assert plan.rail_geometry_failures == (expanded_failure,)
    assert plan.accounting_is_exact is True


def test_r1_planar_partition_uses_strict_rail_evaluator(monkeypatch):
    compiled_plan = object()
    partition = decals_module._ManualSeamBackendPartition(
        backend="RAIL_PLANAR",
        edge_indices=(7,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=compiled_plan,
    )
    calls = []
    monkeypatch.setattr(
        decals_module,
        "evaluate_planar_rail_geometry_plan",
        lambda plan, width, **kwargs: (
            calls.append((plan, width, kwargs)) or ("rail-face",)
        ),
    )

    evaluation = decals_module._evaluate_manual_backend_partition(
        partition,
        DecalSettings(offset=0.125),
        2.0,
        True,
    )

    assert evaluation.faces == ("rail-face",)
    assert evaluation.evaluation_ms >= 0.0
    assert calls == [
        (
            compiled_plan,
            2.0,
            {"offset": 0.125, "preview": True},
        )
    ]


def test_r1_planar_partition_never_falls_back_at_runtime(monkeypatch):
    partition = decals_module._ManualSeamBackendPartition(
        backend="RAIL_PLANAR",
        edge_indices=(7,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    monkeypatch.setattr(
        decals_module,
        "evaluate_planar_rail_geometry_plan",
        lambda *_args, **_kwargs: (),
    )

    with pytest.raises(decals_module.RailPlanarRuntimeError) as exc_info:
        decals_module._evaluate_manual_backend_partition(
            partition,
            DecalSettings(),
            1.0,
            False,
        )

    assert exc_info.value.edge_indices == (7,)
    assert exc_info.value.reason == "evaluation produced no faces"


def test_live_corner_controls_require_clean_patch_voronoi_accounting():
    patch_partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10, 11),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    clean = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(patch_partition,),
        selected_edge_indices=(10, 11),
    )
    failed = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(patch_partition,),
        rejected_edges=(
            decals_module.ManualSeamEdgeRejection(
                edge_index=12,
                reason="NO_OWNER_SURFACES",
            ),
        ),
        selected_edge_indices=(10, 11, 12),
    )

    assert clean.supports_live_corner_controls is True
    assert failed.accounting_is_exact is True
    assert failed.supports_live_corner_controls is False


def test_failed_backend_summary_counts_all_reasons_deterministically():
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        selected_edge_indices=(1, 2, 3, 4),
        rejected_edges=(
            decals_module.ManualSeamEdgeRejection(4, "Z_REASON"),
            decals_module.ManualSeamEdgeRejection(1, "A_REASON"),
            decals_module.ManualSeamEdgeRejection(3, "Z_REASON"),
            decals_module.ManualSeamEdgeRejection(2, ""),
        ),
    )

    assert plan.accounting_is_exact is True
    assert plan.backend_summary == (
        "Failed:4e[A_REASON:x1,UNKNOWN:x1,Z_REASON:x2]"
    )


def test_patch_voronoi_partition_runtime_failure_is_not_rebuilt_as_legacy(
    monkeypatch,
):
    patch_run = _backend_test_run((10,), (10, 11))
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(patch_run,),
        boundary_runs=(),
        compiled_plan=object(),
    )
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("broken crop")
        ),
    )
    with pytest.raises(
        decals_module.PatchVoronoiRuntimeError,
        match=r"1 edge\(s\).*broken crop",
    ):
        decals_module._evaluate_manual_backend_partition(
            partition,
            DecalSettings(),
            width=3.7,
            preview=True,
        )


def test_patch_voronoi_partition_forwards_runtime_corner_settings(
    monkeypatch,
):
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    settings = DecalSettings(
        corner_acute_split_angle=0.37,
        corner_apex_limit=4.25,
        corner_join_mode="MITER",
    )
    captured = []

    def evaluate(_plan, width, **kwargs):
        captured.append((width, kwargs))
        kwargs["diagnostics"].runtime_policy_counts.update(
            {"MITER": 12, "KITE": 3, "ACUTE_SPLIT": 2}
        )
        return ("face",)

    monkeypatch.setattr(
        decals_module, "evaluate_patch_voronoi_plan", evaluate
    )

    rail_plan = object()
    terminal_routing = (object(),)
    evaluation = decals_module._evaluate_manual_backend_partition(
        partition,
        settings,
        width=2.5,
        preview=True,
        rail_plan=rail_plan,
        terminal_routing=terminal_routing,
    )
    assert evaluation.faces == ("face",)
    assert evaluation.evaluation_ms >= 0.0
    assert evaluation.policy_counts == (
        ("ACUTE_SPLIT", 2),
        ("KITE", 3),
        ("MITER", 12),
    )
    corner_settings = captured[0][1]["corner_settings"]
    assert captured[0][0] == 2.5
    assert captured[0][1]["preview"] is True
    assert captured[0][1]["rail_plan"] is rail_plan
    assert captured[0][1]["terminal_routing"] is terminal_routing
    assert corner_settings.acute_split_angle == pytest.approx(0.37)
    assert corner_settings.apex_limit == pytest.approx(4.25)
    assert corner_settings.join_mode == "MITER"


def test_bevel_runtime_gate_precedes_compile_and_bmesh(monkeypatch):
    settings = DecalSettings(corner_join_mode="BEVEL")
    reason = "^DECAL_CORNER_JOIN_ARCHIVED_UNTIL_CORNER_MODEL$"

    with pytest.raises(
        decals_module.DecalCornerJoinArchivedError,
        match=reason,
    ):
        decals_module.compile_manual_seam_decal_plan(None, settings, ())

    with pytest.raises(
        decals_module.DecalCornerJoinArchivedError,
        match=reason,
    ):
        decals_module.evaluate_manual_seam_faces(
            None, settings, object(), preview=True
        )

    monkeypatch.setattr(
        decals_module.bmesh,
        "new",
        lambda: (_ for _ in ()).throw(AssertionError("BMesh was opened")),
        raising=False,
    )
    result = decals_module.generate_decal_result(
        None,
        None,
        settings,
        "SEAMS",
    )
    assert result.status == decals_module.PreviewStatus.ERROR
    assert (
        result.reason
        == decals_module.DECAL_CORNER_JOIN_ARCHIVED_UNTIL_CORNER_MODEL
    )
    with pytest.raises(
        decals_module.DecalCornerJoinArchivedError,
        match=reason,
    ):
        decals_module.generate_decal_objects(
            None,
            None,
            settings,
            "SEAMS",
        )


@pytest.mark.parametrize(
    ("mode", "reason"),
    (
        ("TOP", "DECAL_TOP_ARCHIVED_UNTIL_ENGINE_PLAN"),
        ("BOTTOM", "DECAL_BOTTOM_ARCHIVED_UNTIL_ENGINE_PLAN"),
        ("CORNERS", "DECAL_CORNERS_ARCHIVED_UNTIL_ENGINE_PLAN"),
    ),
)
def test_archived_modes_fail_before_transform_and_bmesh(
    monkeypatch, mode, reason
):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda *_args: (_ for _ in ()).throw(
            AssertionError("Source transform was inspected")
        ),
    )
    monkeypatch.setattr(
        decals_module.bmesh,
        "new",
        lambda: (_ for _ in ()).throw(AssertionError("BMesh was opened")),
        raising=False,
    )

    result = decals_module.generate_decal_result(
        None,
        None,
        DecalSettings(),
        mode,
    )
    assert result.status == decals_module.PreviewStatus.ERROR
    assert result.reason == reason

    with pytest.raises(decals_module.DecalModeArchivedError, match=reason):
        decals_module.generate_decal_objects(
            None,
            None,
            DecalSettings(),
            mode,
        )
    with pytest.raises(decals_module.DecalModeArchivedError, match=reason):
        decals_module._fill_decal_bmesh(
            None,
            None,
            DecalSettings(),
            mode,
        )


def test_patch_voronoi_transaction_fails_before_any_bmesh_write(monkeypatch):
    patch_run = _backend_test_run((10,), (10, 11))
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(patch_run,),
        boundary_runs=(),
        compiled_plan=object(),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(patch_run,),
        boundary_runs=(),
        backend_partitions=(partition,),
        selected_edge_indices=(10,),
    )
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: (),
    )
    materialized = []
    monkeypatch.setattr(
        decals_module,
        "_materialize_network_faces",
        lambda *_args, **_kwargs: materialized.append(True),
    )

    with pytest.raises(
        decals_module.PatchVoronoiRuntimeError,
        match="evaluation produced no faces",
    ):
        decals_module._fill_manual_chain_decals(
            object(),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10,),
            decal_plan=plan,
        )

    assert materialized == []


def test_strict_seams_without_compiled_plan_fail_before_bmesh_write(
    monkeypatch,
):
    run = _backend_test_run((1,), (0, 1))
    monkeypatch.setattr(
        decals_module,
        "_collect_manual_edge_decals",
        lambda *_args, **_kwargs: decals_module._ManualEdgeDecalCollection(
            corner_runs=(run,),
            boundary_runs=(),
            accepted_edge_indices=(1,),
            rejected_edges=(),
        ),
    )
    bm = SimpleNamespace(faces=[])

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            bm,
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(1,),
            decal_plan=None,
        )

    assert exc_info.value.edge_indices == (1,)
    assert exc_info.value.reason == "SEAMS_REQUIRE_COMPILED_PLAN"
    assert bm.faces == []


def test_automatic_seams_without_selected_edge_plan_fail_before_bmesh_write():
    bm = SimpleNamespace(faces=[])

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_decal_bmesh(
            bm,
            object(),
            DecalSettings(),
            "SEAMS",
        )

    assert exc_info.value.edge_indices == ()
    assert exc_info.value.reason == "SEAMS_REQUIRE_SELECTED_EDGE_PLAN"
    assert bm.faces == []


def test_failed_strict_component_aborts_supported_scope_before_evaluation(
    monkeypatch,
):
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(partition,),
        selected_edge_indices=(10, 11),
        rejected_edges=(
            decals_module.ManualSeamEdgeRejection(
                edge_index=11,
                reason="NO_OWNER_SURFACES",
            ),
        ),
        compile_failures=(
            SimpleNamespace(reason="NO_OWNER_SURFACES"),
        ),
    )
    evaluated = []
    materialized = []
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: evaluated.append(True) or ("face",),
    )
    monkeypatch.setattr(
        decals_module,
        "_materialize_network_faces",
        lambda *_args, **_kwargs: materialized.append(True),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10, 11),
            decal_plan=plan,
        )

    assert exc_info.value.edge_indices == (11,)
    assert "Failed:1e" in exc_info.value.reason
    assert evaluated == []
    assert materialized == []


def test_stale_strict_plan_scope_fails_before_evaluation(monkeypatch):
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(partition,),
        selected_edge_indices=(10,),
    )
    evaluated = []
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: evaluated.append(True) or ("face",),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10, 11),
            decal_plan=plan,
        )

    assert exc_info.value.edge_indices == (11,)
    assert exc_info.value.reason == (
        "SEAMS_PLAN_SCOPE_MISMATCH:runtime=(10, 11),compiled=(10,)"
    )
    assert evaluated == []


def test_broken_strict_plan_accounting_fails_before_evaluation(monkeypatch):
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(partition,),
        selected_edge_indices=(10, 11),
    )
    evaluated = []
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: evaluated.append(True) or ("face",),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10, 11),
            decal_plan=plan,
        )

    assert exc_info.value.edge_indices == (10, 11)
    assert exc_info.value.reason == "SEAMS_PLAN_ACCOUNTING_MISMATCH"
    assert evaluated == []


def test_fake_plan_cannot_bypass_runtime_accounting_with_claimed_property(
    monkeypatch,
):
    fake_plan = SimpleNamespace(
        corner_runs=(),
        boundary_runs=(),
        accounting_is_exact=True,
    )
    evaluated = []
    monkeypatch.setattr(
        decals_module,
        "_evaluate_manual_backend_partition",
        lambda *_args, **_kwargs: evaluated.append(True),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10,),
            decal_plan=fake_plan,
        )

    assert exc_info.value.edge_indices == (10,)
    assert exc_info.value.reason == "SEAMS_PLAN_TYPE_INVALID"
    assert evaluated == []


def test_fake_backend_partition_fails_runtime_structural_accounting(
    monkeypatch,
):
    fake_partition = SimpleNamespace(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(fake_partition,),
        selected_edge_indices=(10,),
    )
    evaluated = []
    monkeypatch.setattr(
        decals_module,
        "_evaluate_manual_backend_partition",
        lambda *_args, **_kwargs: evaluated.append(True),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=(10,),
            decal_plan=plan,
        )

    assert exc_info.value.edge_indices == (10,)
    assert exc_info.value.reason == "SEAMS_PLAN_ACCOUNTING_MISMATCH"
    assert evaluated == []


@pytest.mark.parametrize(
    "malformation",
    (
        "DUPLICATE_SELECTED",
        "DUPLICATE_ACCEPTED",
        "DUPLICATE_REJECTED",
        "ACCEPTED_REJECTED_OVERLAP",
    ),
)
def test_raw_strict_plan_accounting_rejects_duplicates_and_overlap(
    monkeypatch,
    malformation,
):
    partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(),
        boundary_runs=(),
        compiled_plan=object(),
    )
    partitions = (partition,)
    selected = (10,)
    rejected = ()
    runtime_scope = (10,)
    if malformation == "DUPLICATE_SELECTED":
        selected = (10, 10)
    elif malformation == "DUPLICATE_ACCEPTED":
        partitions = (partition, partition)
    elif malformation == "DUPLICATE_REJECTED":
        selected = (10, 11)
        runtime_scope = (10, 11)
        rejection = decals_module.ManualSeamEdgeRejection(
            edge_index=11,
            reason="UNSUPPORTED_TEST_SCOPE",
        )
        rejected = (rejection, rejection)
    else:
        rejected = (
            decals_module.ManualSeamEdgeRejection(
                edge_index=10,
                reason="UNSUPPORTED_TEST_SCOPE",
            ),
        )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=partitions,
        selected_edge_indices=selected,
        rejected_edges=rejected,
    )
    evaluated = []
    monkeypatch.setattr(
        decals_module,
        "_evaluate_manual_backend_partition",
        lambda *_args, **_kwargs: evaluated.append(True),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._fill_manual_chain_decals(
            SimpleNamespace(faces=[]),
            object(),
            DecalSettings(),
            (),
            mode="SEAMS",
            selected_edge_indices=runtime_scope,
            decal_plan=plan,
        )

    assert exc_info.value.edge_indices == tuple(sorted(set(selected)))
    assert exc_info.value.reason == "SEAMS_PLAN_ACCOUNTING_MISMATCH"
    assert evaluated == []


def test_generate_decal_objects_reuses_existing_object_only_for_preview(
    monkeypatch,
):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    fake_bm = _FakeMaterializationBMesh()
    fake_bm.faces.append(object())
    monkeypatch.setattr(
        decals_module.bmesh,
        "new",
        lambda: fake_bm,
        raising=False,
    )
    monkeypatch.setattr(
        decals_module,
        "_fill_decal_bmesh",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        decals_module,
        "_prepare_decal_bmesh",
        lambda bm: bool(bm.faces),
    )
    finalize_calls = []

    class FakeObject(dict):
        def __init__(self, name):
            super().__init__()
            self.name = name
            self.hide_render = False

    def finalize(_bm, name, *_args, **kwargs):
        finalize_calls.append(
            (name, kwargs["reuse_existing"], kwargs["preview_state"])
        )
        return FakeObject(name)

    monkeypatch.setattr(
        decals_module,
        "_finalize_decal_object",
        finalize,
    )
    source = SimpleNamespace(name="Source")
    preview_state = decals_module.DecalPreviewState()

    preview_names = decals_module.generate_decal_objects(
        PatchGraph(),
        source,
        DecalSettings(),
        "SEAMS",
        scene=object(),
        preview=True,
        preview_state=preview_state,
    )
    final_names = decals_module.generate_decal_objects(
        PatchGraph(),
        source,
        DecalSettings(),
        "SEAMS",
        scene=object(),
        preview=False,
    )

    assert preview_names == [".CFTUV_Preview_Seams_Source"]
    assert final_names == ["Decal_Seams_Source"]
    assert finalize_calls == [
        (".CFTUV_Preview_Seams_Source", True, preview_state),
        ("Decal_Seams_Source", False, None),
    ]


def test_structured_generation_result_exposes_runtime_summary(monkeypatch):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda *_args, **_kwargs: decals_module._DecalTransactionResult(
            obj=SimpleNamespace(name="Decal_Seams_Source"),
            topology_changed=False,
            policy_counts=(("KITE", 2), ("SEGMENT", 4)),
            evaluation_ms=12.5,
        ),
    )
    plan = SimpleNamespace(backend_summary="Patch Voronoi:1c/3e")

    result = decals_module.generate_decal_result(
        PatchGraph(),
        SimpleNamespace(name="Source"),
        DecalSettings(),
        "SEAMS",
        scene=object(),
        decal_plan=plan,
    )

    assert result == decals_module.DecalGenerationResult(
        status=decals_module.PreviewStatus.UPDATED,
        object_name="Decal_Seams_Source",
        topology_changed=False,
        backend_summary="Patch Voronoi:1c/3e",
        policy_counts=(("KITE", 2), ("SEGMENT", 4)),
        evaluation_ms=12.5,
    )


def test_remove_preview_deletes_only_marked_object_and_orphan_mesh(monkeypatch):
    class FakeObject(dict):
        def __init__(self, name, mesh):
            super().__init__(cftuv_decal_preview=True)
            self.name = name
            self.mode = "OBJECT"
            self.type = "MESH"
            self.data = mesh

    mesh = SimpleNamespace(users=0)
    name = ".CFTUV_Preview_Seams_Source"
    preview = FakeObject(name, mesh)
    objects = {name: preview}
    removed_objects = []
    removed_meshes = []

    class ObjectStore:
        def get(self, key):
            return objects.get(key)

        def remove(self, obj, do_unlink=False):
            assert do_unlink is True
            removed_objects.append(obj)
            objects.pop(obj.name)

    class MeshStore:
        def remove(self, value):
            removed_meshes.append(value)

    monkeypatch.setattr(
        decals_module.bpy,
        "data",
        SimpleNamespace(objects=ObjectStore(), meshes=MeshStore()),
        raising=False,
    )
    state = decals_module.DecalPreviewState(
        topology_signature=((4,),),
        canonical_mesh_indices=(0, 1, 2, 3),
        object_name=name,
        object_pointer=10,
        mesh_pointer=20,
    )

    removed = decals_module.remove_decal_preview_object(
        "SEAMS", SimpleNamespace(name="Source"), state
    )

    assert removed is True
    assert removed_objects == [preview]
    assert removed_meshes == [mesh]
    assert state.object_name == ""
    assert state.object_pointer == 0
    assert state.mesh_pointer == 0
    assert state.topology_signature == ()


def test_production_transaction_does_not_rebind_preview_state(monkeypatch):
    """Apply обязан оставить preview identity доступной terminal cleanup."""

    class FakeBMesh:
        def free(self):
            pass

    preview_name = ".CFTUV_Preview_Seams_Source"
    preview_state = decals_module.DecalPreviewState(
        object_name=preview_name,
        object_pointer=10,
        mesh_pointer=20,
    )
    finalized_states = []
    monkeypatch.setattr(
        decals_module.bmesh, "new", FakeBMesh, raising=False
    )
    monkeypatch.setattr(
        decals_module,
        "_fill_decal_bmesh",
        lambda *_args, **_kwargs: (),
    )
    monkeypatch.setattr(
        decals_module,
        "_prepare_decal_bmesh",
        lambda _bm: True,
    )
    monkeypatch.setattr(
        decals_module,
        "_finalize_decal_object",
        lambda *_args, **kwargs: (
            finalized_states.append(kwargs["preview_state"])
            or SimpleNamespace(name="Decal_Seams_Source")
        ),
    )

    result = decals_module._generate_decal_transaction(
        PatchGraph(),
        SimpleNamespace(name="Source"),
        DecalSettings(),
        "SEAMS",
        object(),
        None,
        None,
        False,
        None,
        preview_state,
    )

    assert result.obj.name == "Decal_Seams_Source"
    assert finalized_states == [None]
    assert preview_state.object_name == preview_name
    assert preview_state.object_pointer == 10
    assert preview_state.mesh_pointer == 20


def test_final_swap_preserves_object_identity_properties_and_materials(
    monkeypatch,
):
    class Matrix:
        def copy(self):
            return self

    class Mesh:
        def __init__(self, materials=()):
            self.materials = list(materials)
            self.users = 0
            self.updated = False

        def update(self):
            self.updated = True

    class DecalObject(dict):
        pass

    old_mesh = Mesh(("Trim", "Damage"))
    old_obj = DecalObject(artist="kept", export_tag="production")
    old_obj.name = "Decal_Seams_Source"
    old_obj.mode = "OBJECT"
    old_obj.data = old_mesh
    old_obj.matrix_world = Matrix()
    new_mesh = Mesh()
    removed_meshes = []

    monkeypatch.setattr(
        decals_module.bpy,
        "data",
        SimpleNamespace(
            objects=SimpleNamespace(get=lambda _name: old_obj),
            meshes=SimpleNamespace(
                new=lambda _name: new_mesh,
                remove=lambda mesh: removed_meshes.append(mesh),
            ),
        ),
        raising=False,
    )

    class BMesh:
        faces = [object()]
        freed = False

        def to_mesh(self, mesh):
            assert mesh is new_mesh

        def free(self):
            self.freed = True

    bm = BMesh()
    source = SimpleNamespace(matrix_world=Matrix())

    result = decals_module._finalize_decal_object(
        bm,
        old_obj.name,
        source,
        object(),
        prepared=True,
    )

    assert result is old_obj
    assert old_obj.data is new_mesh
    assert dict(old_obj) == {
        "artist": "kept",
        "export_tag": "production",
    }
    assert new_mesh.materials == ["Trim", "Damage"]
    assert removed_meshes == [old_mesh]
    assert bm.freed is True


def test_final_mesh_build_failure_does_not_touch_production_object(monkeypatch):
    old_mesh = SimpleNamespace(materials=[], users=0)
    old_obj = SimpleNamespace(
        name="Decal_Seams_Source",
        mode="OBJECT",
        data=old_mesh,
    )
    temporary_mesh = SimpleNamespace(users=0)
    removed_meshes = []
    monkeypatch.setattr(
        decals_module.bpy,
        "data",
        SimpleNamespace(
            objects=SimpleNamespace(get=lambda _name: old_obj),
            meshes=SimpleNamespace(
                new=lambda _name: temporary_mesh,
                remove=lambda mesh: removed_meshes.append(mesh),
            ),
        ),
        raising=False,
    )

    class FailingBMesh:
        faces = [object()]
        freed = False

        def to_mesh(self, _mesh):
            raise RuntimeError("mesh conversion failed")

        def free(self):
            self.freed = True

    bm = FailingBMesh()

    with pytest.raises(RuntimeError, match="mesh conversion failed"):
        decals_module._finalize_decal_object(
            bm,
            old_obj.name,
            SimpleNamespace(matrix_world=object()),
            object(),
            prepared=True,
        )

    assert old_obj.data is old_mesh
    assert removed_meshes == [temporary_mesh]
    assert bm.freed is True


@pytest.mark.parametrize(
    ("preview", "has_existing", "expected_status"),
    (
        (True, True, decals_module.PreviewStatus.ERROR),
        (True, False, decals_module.PreviewStatus.ERROR),
        (False, True, decals_module.PreviewStatus.ERROR),
    ),
)
def test_structured_generation_classifies_transaction_error(
    monkeypatch, preview, has_existing, expected_status
):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("invalid crop")
        ),
    )
    existing = SimpleNamespace(
        name=".CFTUV_Preview_Seams_Source",
        mode="OBJECT",
        type="MESH",
        data=object(),
        cftuv_decal_preview=True,
    )
    monkeypatch.setattr(
        decals_module,
        "_existing_decal_object",
        lambda _name: existing if has_existing else None,
    )
    removed = []
    monkeypatch.setattr(
        decals_module,
        "remove_decal_preview_object",
        lambda mode, source, preview_state=None: removed.append(
            (mode, source.name, preview_state)
        ),
    )

    result = decals_module.generate_decal_result(
        PatchGraph(),
        SimpleNamespace(name="Source"),
        DecalSettings(),
        "SEAMS",
        scene=object(),
        preview=preview,
    )

    assert result.status == expected_status
    assert result.reason == "invalid crop"
    assert result.object_name is None
    assert removed == ([("SEAMS", "Source", None)] if preview else [])


@pytest.mark.parametrize(
    ("preview", "has_existing", "expected_status"),
    (
        (True, True, decals_module.PreviewStatus.EMPTY),
        (True, False, decals_module.PreviewStatus.EMPTY),
        (False, True, decals_module.PreviewStatus.EMPTY),
    ),
)
def test_structured_generation_classifies_empty_transaction(
    monkeypatch, preview, has_existing, expected_status
):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda *_args, **_kwargs: decals_module._DecalTransactionResult(
            obj=None,
            topology_changed=False,
            policy_counts=(),
        ),
    )
    existing = SimpleNamespace(
        name=".CFTUV_Preview_Seams_Source",
        mode="OBJECT",
        type="MESH",
        data=object(),
        cftuv_decal_preview=True,
    )
    monkeypatch.setattr(
        decals_module,
        "_existing_decal_object",
        lambda _name: existing if has_existing else None,
    )
    removed = []
    monkeypatch.setattr(
        decals_module,
        "remove_decal_preview_object",
        lambda mode, source, preview_state=None: removed.append(
            (mode, source.name, preview_state)
        ),
    )

    result = decals_module.generate_decal_result(
        PatchGraph(),
        SimpleNamespace(name="Source"),
        DecalSettings(),
        "SEAMS",
        scene=object(),
        preview=preview,
    )

    assert result.status == expected_status
    assert result.reason == "generation produced no faces"
    assert result.object_name is None
    assert removed == ([("SEAMS", "Source", None)] if preview else [])


def test_seams_source_transform_error_removes_preview(monkeypatch):
    source = SimpleNamespace(name="Source")
    preview_state = decals_module.DecalPreviewState(
        object_name=".CFTUV_Preview_Seams_Source"
    )
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            decals_module.DecalSourceTransformError(
                "NON_UNIFORM_SOURCE_SCALE",
                "source scale is unsupported",
            )
        ),
    )
    removed = []
    monkeypatch.setattr(
        decals_module,
        "remove_decal_preview_object",
        lambda mode, obj, state: removed.append((mode, obj, state)),
    )

    result = decals_module.generate_decal_result(
        PatchGraph(),
        source,
        DecalSettings(),
        "SEAMS",
        scene=object(),
        preview=True,
        preview_state=preview_state,
    )

    assert result.status == decals_module.PreviewStatus.ERROR
    assert result.object_name is None
    assert result.reason == (
        "NON_UNIFORM_SOURCE_SCALE: source scale is unsupported"
    )
    assert removed == [("SEAMS", source, preview_state)]


def test_compatibility_seams_empty_preview_removes_object_and_raises(
    monkeypatch,
):
    source = SimpleNamespace(name="Source")
    preview_state = decals_module.DecalPreviewState(
        object_name=".CFTUV_Preview_Seams_Source"
    )
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda *_args, **_kwargs: decals_module._DecalTransactionResult(
            obj=None,
            topology_changed=False,
            policy_counts=(),
        ),
    )
    removed = []
    monkeypatch.setattr(
        decals_module,
        "remove_decal_preview_object",
        lambda mode, obj, state: removed.append((mode, obj, state)),
    )

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module.generate_decal_objects(
            PatchGraph(),
            source,
            DecalSettings(),
            "SEAMS",
            scene=object(),
            selected_edge_indices=(4, 9),
            preview=True,
            preview_state=preview_state,
        )

    assert exc_info.value.edge_indices == (4, 9)
    assert exc_info.value.reason == "SEAMS_GENERATION_PRODUCED_NO_FACES"
    assert removed == [("SEAMS", source, preview_state)]


def _materialization_face(keys, kind="SEGMENT", side=""):
    count = len(keys)
    return SimpleNamespace(
        vert_keys=list(keys),
        positions=[Vector((float(index), float(index % 2), 0.0)) for index in range(count)],
        u_fracs=[float(index) / max(1, count - 1) for index in range(count)],
        v_lengths=[float(index) for index in range(count)],
        component_kind=kind,
        component_side=side,
    )


def test_materialization_rejects_late_invalid_face_before_any_bmesh_write():
    valid = _materialization_face((("v", 0), ("v", 1), ("v", 2)))
    invalid = _materialization_face(
        (("v", 3), ("v", 3), ("v", 4)),
        kind="ACUTE_SPLIT",
        side="OUTER",
    )

    with pytest.raises(decals_module.DecalMaterializationError) as caught:
        # ``object()`` доказывает, что preflight нашёл второй face до любого
        # доступа к BMesh после валидного первого face.
        decals_module._materialize_network_faces(
            object(),
            (valid, invalid),
            DecalSettings(),
            (0.0, 0.0, 1.0, 1.0),
            backend="PATCH_VORONOI",
            edge_indices=(10, 11),
        )

    error = caught.value
    assert error.backend == "PATCH_VORONOI"
    assert error.edge_indices == (10, 11)
    assert error.face_index == 1
    assert error.vertex_count == 3
    assert error.repeated_keys == (("v", 3),)
    assert error.cycle_keys == (("v", 3), ("v", 3), ("v", 4))
    assert error.repeated_occurrences == (
        (("v", 3), (0, 1), ((0, 1),)),
    )
    assert error.component_kind == "ACUTE_SPLIT"
    assert error.component_side == "OUTER"


def test_materialization_reports_nonadjacent_repeat_as_bowtie():
    invalid = _materialization_face(
        (("v", 0), ("v", 1), ("v", 0), ("v", 2))
    )

    with pytest.raises(decals_module.DecalMaterializationError) as caught:
        decals_module._validate_network_faces_for_materialization(
            (invalid,), "PATCH_VORONOI", (32,)
        )

    error = caught.value
    assert error.repeated_keys == (("v", 0),)
    assert error.repeated_occurrences == (
        (("v", 0), (0, 2), ()),
    )
    assert "repeated_occurrences" in str(error)


def test_materialization_cycle_dump_does_not_mask_malformed_position():
    invalid = _materialization_face((('v', 0), ('v', 0), ('v', 1)))
    invalid.positions[0] = object()

    with pytest.raises(decals_module.DecalMaterializationError) as caught:
        decals_module._validate_network_faces_for_materialization(
            (invalid,), "PATCH_VORONOI", (32,)
        )

    error = caught.value
    assert error.reason == "repeated vertex keys"
    assert isinstance(error.cycle_positions[0], str)


@pytest.mark.parametrize("field_name", ("vert_keys", "positions"))
def test_materialization_cycle_dump_handles_noniterable_loop_array(field_name):
    invalid = _materialization_face((("v", 0), ("v", 1), ("v", 2)))
    setattr(invalid, field_name, None)

    with pytest.raises(decals_module.DecalMaterializationError) as caught:
        decals_module._validate_network_faces_for_materialization(
            (invalid,), "PATCH_VORONOI", (32,)
        )

    assert caught.value.reason.startswith("invalid loop arrays")


class _FakeMaterializationBMesh:
    class _UVValue:
        uv = None

    class _Loop:
        def __init__(self):
            self.value = _FakeMaterializationBMesh._UVValue()

        def __getitem__(self, _layer):
            return self.value

    class _Vert:
        def __init__(self, position):
            self.position = position

    class _Verts(list):
        def new(self, position):
            vert = _FakeMaterializationBMesh._Vert(position)
            self.append(vert)
            return vert

    class _Faces(list):
        def new(self, verts):
            face = SimpleNamespace(
                verts=tuple(verts),
                loops=[
                    _FakeMaterializationBMesh._Loop() for _vert in verts
                ],
            )
            self.append(face)
            return face

    def __init__(self):
        self.verts = self._Verts()
        self.faces = self._Faces()
        self.loops = SimpleNamespace(
            layers=SimpleNamespace(
                uv=SimpleNamespace(verify=lambda: object())
            )
        )
        self.freed = False

    def free(self):
        self.freed = True


def test_materialization_returns_structured_complete_result():
    bm = _FakeMaterializationBMesh()
    result = decals_module._materialize_network_faces(
        bm,
        (_materialization_face((("v", 0), ("v", 1), ("v", 2))),),
        DecalSettings(),
        (0.0, 0.0, 1.0, 1.0),
        backend="PATCH_VORONOI",
        edge_indices=(8,),
    )

    assert result == decals_module.DecalMaterializationResult(
        backend="PATCH_VORONOI",
        edge_indices=(8,),
        source_face_count=1,
        created_face_count=1,
        created_vertex_count=3,
        policy_counts=(("SEGMENT", 1),),
    )
    assert len(bm.faces) == 1


def test_legacy_materialization_fails_before_bmesh_write():
    bm = _FakeMaterializationBMesh()

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._materialize_network_faces(
            bm,
            (_materialization_face((("v", 0), ("v", 1), ("v", 2))),),
            DecalSettings(),
            (0.0, 0.0, 1.0, 1.0),
            backend="LEGACY_NETWORK",
            edge_indices=(8,),
        )

    assert exc_info.value.edge_indices == (8,)
    assert exc_info.value.reason == "LEGACY_NETWORK_MATERIALIZATION_DISABLED"
    assert len(bm.faces) == 0


def test_unknown_materialization_backend_fails_before_bmesh_write():
    bm = _FakeMaterializationBMesh()

    with pytest.raises(decals_module.StrictSeamRuntimeError) as exc_info:
        decals_module._materialize_network_faces(
            bm,
            (_materialization_face((("v", 0), ("v", 1), ("v", 2))),),
            DecalSettings(),
            (0.0, 0.0, 1.0, 1.0),
            backend="UNKNOWN_TEST_BACKEND",
            edge_indices=(13,),
        )

    assert exc_info.value.edge_indices == (13,)
    assert exc_info.value.reason == (
        "UNSUPPORTED_MATERIALIZATION_BACKEND:UNKNOWN_TEST_BACKEND"
    )
    assert len(bm.faces) == 0


@pytest.mark.parametrize("preview", (False, True))
def test_invalid_face_frees_transaction_without_publishing(
    monkeypatch, preview
):
    monkeypatch.setattr(
        decals_module,
        "local_decal_settings_for_source",
        lambda settings, _source: settings,
    )
    bm = _FakeMaterializationBMesh()
    monkeypatch.setattr(
        decals_module.bmesh, "new", lambda: bm, raising=False
    )
    valid = _materialization_face((("v", 0), ("v", 1), ("v", 2)))
    invalid = _materialization_face((("v", 3), ("v", 3), ("v", 4)))

    def fill(target_bm, *_args, **_kwargs):
        decals_module._materialize_network_faces(
            target_bm,
            (valid, invalid),
            DecalSettings(),
            (0.0, 0.0, 1.0, 1.0),
            backend="PATCH_VORONOI",
            edge_indices=(10,),
        )

    monkeypatch.setattr(decals_module, "_fill_decal_bmesh", fill)
    published = []
    monkeypatch.setattr(
        decals_module,
        "_finalize_decal_object",
        lambda *_args, **_kwargs: published.append(True),
    )
    preview_state = decals_module.DecalPreviewState(
        topology_signature=((3,),),
        canonical_mesh_indices=(0, 1, 2),
        object_name=".CFTUV_Preview_Seams_Source",
        object_pointer=101,
        mesh_pointer=202,
    )
    monkeypatch.setattr(
        decals_module,
        "_existing_decal_object",
        lambda _name: None,
    )
    original_remove = decals_module.remove_decal_preview_object
    removed = []

    def remove(mode, source, state):
        removed.append((mode, source, state))
        return original_remove(mode, source, state)

    monkeypatch.setattr(decals_module, "remove_decal_preview_object", remove)
    source = SimpleNamespace(name="Source")

    with pytest.raises(decals_module.DecalMaterializationError):
        decals_module.generate_decal_objects(
            PatchGraph(),
            source,
            DecalSettings(),
            "SEAMS",
            scene=object(),
            preview=preview,
            preview_state=preview_state if preview else None,
        )

    assert bm.freed
    assert published == []
    if preview:
        assert removed == [("SEAMS", source, preview_state)]
        assert preview_state.topology_signature == ()
        assert preview_state.canonical_mesh_indices == ()
        assert preview_state.object_name == ""
        assert preview_state.object_pointer == 0
        assert preview_state.mesh_pointer == 0
    else:
        assert removed == []
        assert preview_state.topology_signature == ((3,),)
        assert preview_state.canonical_mesh_indices == (0, 1, 2)
        assert preview_state.object_name == ".CFTUV_Preview_Seams_Source"
        assert preview_state.object_pointer == 101
        assert preview_state.mesh_pointer == 202


def _make_chain(
    vert_indices,
    vert_cos,
    neighbor_patch_id,
    edge_indices=None,
    dihedral_convexity=0.0,
    side_face_normals=None,
):
    chain = BoundaryChain(
        vert_indices=list(vert_indices),
        vert_cos=[Vector(co) for co in vert_cos],
        edge_indices=(
            list(edge_indices)
            if edge_indices is not None
            else list(range(len(vert_indices) - 1))
        ),
        neighbor_patch_id=neighbor_patch_id,
        side_face_normals=[Vector(normal) for normal in (side_face_normals or ())],
    )
    chain.dihedral_convexity = dihedral_convexity
    return chain


def _make_graph(*nodes):
    graph = PatchGraph()
    for node in nodes:
        graph.add_node(node)
    return graph


def _make_ribbon_run(start, end, point_a, point_b, normal=(0, 1, 0)):
    return _OrientedRibbonRun(
        vert_indices=[start, end],
        points=[Vector(point_a), Vector(point_b)],
        segment_normals=[Vector(normal)],
        segment_ups=[Vector((0, 0, 1))],
        segment_chain_refs=[(start, 0, 0)],
    )


def _make_corner_run(
    start,
    end,
    point_a,
    point_b,
    edge_index,
    normal_a=(0, 1, 0),
    normal_b=(0, 0, 1),
):
    return _OrientedCornerRun(
        vert_indices=[start, end],
        points=[Vector(point_a), Vector(point_b)],
        segment_normals_a=[Vector(normal_a)],
        segment_normals_b=[Vector(normal_b)],
        segment_convexities=[1.0],
        segment_edge_indices=[edge_index],
    )


def _ribbon_edges(runs):
    return [
        (run.vert_indices[index], run.vert_indices[index + 1])
        for run in runs
        for index in range(len(run.vert_indices) - 1)
    ]


class TestCollectTrimSegments:
    def test_uses_local_owner_face_normal_instead_of_patch_average(self):
        # Один wrapped WALL patch может содержать стены с разными нормалями.
        # Средняя +Y ошибочно объявила бы нижнее +X ребро верхним; локальная
        # owner-face normal -Y правильно оставляет его в BOTTOM.
        chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (1, 0, 0)],
            -1,
            side_face_normals=[(0, -1, 0)],
        )
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == []
        assert _ribbon_edges(bottom_runs) == [(0, 1)]
        assert bottom_runs[0].segment_normals[0].y == -1.0

    def test_top_bottom_classification(self):
        # Стена с нормалью +Y, вертикаль +Z. Верхняя кромка идёт по +X
        # (outward = d x n = +Z), нижняя — по -X (outward = -Z).
        top_chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        bottom_chain = _make_chain([2, 3], [(1, 0, 0), (0, 0, 0)], -1)
        side_chain = _make_chain([1, 2], [(1, 0, 1), (1, 0, 0)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [top_chain, bottom_chain, side_chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert _ribbon_edges(top_runs) == [(0, 1)]
        assert _ribbon_edges(bottom_runs) == [(2, 3)]
        normal = top_runs[0].segment_normals[0]
        up = top_runs[0].segment_ups[0]
        assert abs(normal.y - 1.0) < 1e-6
        assert abs(up.z - 1.0) < 1e-6

    def test_wall_wall_chains_excluded(self):
        # Кромка к WALL соседу — под corner/seam декали, в тримы не идёт.
        # Кромка к FLOOR соседу — идёт.
        to_wall = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], 1)
        to_floor = _make_chain([2, 3], [(1, 0, 0), (0, 0, 0)], 2)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [to_wall, to_floor])

        other_wall = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [])
        floor = PatchNode(patch_id=2, face_indices=[200])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(wall, other_wall, floor)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == []
        assert _ribbon_edges(bottom_runs) == [(2, 3)]

    def test_non_wall_patches_ignored(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        floor = PatchNode(patch_id=0, face_indices=[0])
        floor.patch_type = PatchType.FLOOR
        floor.boundary_loops = [BoundaryLoop(chains=[chain])]
        graph = _make_graph(floor)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == [] and bottom_runs == []

    def test_short_edges_skipped(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (0.01, 0, 1)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == [] and bottom_runs == []

    def test_selected_edge_enables_its_complete_chain(self):
        top_chain = _make_chain(
            [0, 1, 2],
            [(0, 0, 1), (1, 0, 1), (2, 0, 1)],
            -1,
            edge_indices=[10, 11],
        )
        other_top_chain = _make_chain(
            [3, 4],
            [(3, 0, 1), (4, 0, 1)],
            -1,
            edge_indices=[12],
        )
        wall = _make_wall_node(
            0,
            (0, 1, 0),
            (0, 0, 1),
            [top_chain, other_top_chain],
        )
        graph = _make_graph(wall)

        chain_refs = chain_refs_for_edge_indices(graph, [11])
        top_runs, bottom_runs = _collect_trim_ribbon_runs(
            graph, chain_refs=chain_refs
        )

        assert chain_refs == {(0, 0, 0)}
        assert _ribbon_edges(top_runs) == [(0, 1), (1, 2)]
        assert bottom_runs == []


class TestOrientedRibbonRuns:
    def test_mixed_chain_directions_form_one_closed_run(self):
        runs = [
            _make_ribbon_run(0, 1, (0, 0, 0), (1, 0, 0), (0, 1, 0)),
            _make_ribbon_run(2, 1, (1, 1, 0), (1, 0, 0), (1, 0, 0)),
            _make_ribbon_run(2, 3, (1, 1, 0), (0, 1, 0), (0, -1, 0)),
            _make_ribbon_run(0, 3, (0, 0, 0), (0, 1, 0), (-1, 0, 0)),
        ]

        stitched = _stitch_ribbon_runs(runs)

        assert len(stitched) == 1
        ring = stitched[0]
        assert ring.is_closed
        assert ring.vert_indices == [0, 1, 2, 3, 0]
        assert len(ring.segment_normals) == 4
        frames = _ribbon_vertex_frames(ring)
        assert (frames[0][0] - frames[-1][0]).length < 1e-9

    def test_point_contact_branch_is_not_joined_arbitrarily(self):
        runs = [
            _make_ribbon_run(0, 1, (0, 0, 0), (1, 0, 0)),
            _make_ribbon_run(1, 2, (1, 0, 0), (2, 0, 0)),
            _make_ribbon_run(1, 3, (1, 0, 0), (1, 1, 0)),
        ]

        stitched = _stitch_ribbon_runs(runs)

        assert len(stitched) == 3
        assert all(len(run.vert_indices) == 2 for run in stitched)

    def test_reversed_segment_requests_winding_flip(self):
        down = Vector((0, 0, -1))
        desired_normal = Vector((0, 1, 0))

        assert not _trim_quad_requires_flip(
            Vector((0, 0, 0)), Vector((1, 0, 0)), down, down, desired_normal
        )
        assert _trim_quad_requires_flip(
            Vector((1, 0, 0)), Vector((0, 0, 0)), down, down, desired_normal
        )

    def test_winding_flip_does_not_swap_base_tip_uv(self):
        verts, uvs = _trim_quad_layout(
            "base_a", "base_b", "tip_a", "tip_b", 2.0, 3.0, 0.8, 1.0, True
        )

        uv_by_vert = dict(zip(verts, uvs))
        assert uv_by_vert["base_a"][1] == 0.8
        assert uv_by_vert["base_b"][1] == 0.8
        assert uv_by_vert["tip_a"][1] == 1.0
        assert uv_by_vert["tip_b"][1] == 1.0


class TestCollectWallPairChains:
    def test_paired_self_seam_on_wrapped_wall_becomes_corner(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            dihedral_convexity=0.5,
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            dihedral_convexity=0.5,
            side_face_normals=[(1, 0, 0)],
        )
        wrapped_wall = _make_wall_node(
            0, (0.707, 0.707, 0), (0, 0, 1), [side_a, side_b]
        )
        graph = _make_graph(wrapped_wall)

        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert len(corner_chains) == 1
        assert seam_chains == []
        points, normal_a, normal_b, _closed, convexity = corner_chains[0]
        assert points == side_a.vert_cos
        assert normal_a.dot(Vector((0, 1, 0))) > 0.999
        assert normal_b.dot(Vector((1, 0, 0))) > 0.999
        assert convexity == 0.5

    def test_nearly_coplanar_self_seam_remains_corner(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0.05, 0.99875, 0)],
        )
        wrapped_wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [side_a, side_b])

        corner_chains, seam_chains = _collect_wall_pair_chains(
            _make_graph(wrapped_wall)
        )

        assert len(corner_chains) == 1
        assert seam_chains == []

    def test_corner_vs_seam_split_and_dedupe(self):
        # patch 0 | patch 1 — перпендикулярны (угол), patch 0 | patch 2 —
        # копланарны (шов). Обратные цепочки от соседей должны быть
        # отброшены дедупликацией (owner id < neighbor id).
        corner_chain = _make_chain([0, 1], [(0, 0, 0), (0, 0, 1)], 1)
        seam_chain = _make_chain([2, 3], [(1, 0, 0), (1, 0, 1)], 2)
        wall_0 = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [corner_chain, seam_chain])

        back_chain = _make_chain([1, 0], [(0, 0, 1), (0, 0, 0)], 0)
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [back_chain])

        back_seam = _make_chain([3, 2], [(1, 0, 1), (1, 0, 0)], 0)
        wall_2 = _make_wall_node(2, (0, 1, 0), (0, 0, 1), [back_seam])

        graph = _make_graph(wall_0, wall_1, wall_2)
        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert len(corner_chains) == 1
        assert len(seam_chains) == 1
        corner_points, normal_a, normal_b, _closed, _convexity = corner_chains[0]
        assert len(corner_points) == 2
        assert abs(normal_a.y - 1.0) < 1e-6
        assert abs(normal_b.x - 1.0) < 1e-6

    def test_wall_floor_pairs_ignored(self):
        chain = _make_chain([0, 1], [(0, 0, 0), (0, 0, 1)], 1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        floor = PatchNode(patch_id=1, face_indices=[100])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(wall, floor)

        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert corner_chains == [] and seam_chains == []

    def test_selected_shared_edge_captures_both_chain_sides(self):
        owner_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[42],
        )
        neighbor_chain = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, 0)],
            0,
            edge_indices=[42],
        )
        wall_0 = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [owner_chain])
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [neighbor_chain])
        graph = _make_graph(wall_0, wall_1)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, seam_chains = _collect_wall_pair_chains(
            graph, chain_refs=chain_refs
        )

        assert chain_refs == {(0, 0, 0), (1, 0, 0)}
        assert len(corner_chains) == 1
        assert seam_chains == []

    def test_unselected_wall_pair_is_filtered_out(self):
        first_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[20],
        )
        selected_chain = _make_chain(
            [2, 3],
            [(1, 0, 0), (1, 0, 1)],
            2,
            edge_indices=[30],
        )
        wall_0 = _make_wall_node(
            0,
            (0, 1, 0),
            (0, 0, 1),
            [first_chain, selected_chain],
        )
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [])
        wall_2 = _make_wall_node(2, (0, 1, 0), (0, 0, 1), [])
        graph = _make_graph(wall_0, wall_1, wall_2)

        chain_refs = chain_refs_for_edge_indices(graph, [30])
        corner_chains, seam_chains = _collect_wall_pair_chains(
            graph, chain_refs=chain_refs
        )

        assert corner_chains == []
        assert len(seam_chains) == 1


class TestManualChainDecals:
    def test_selected_edges_are_atomic_across_asymmetric_chain_splits(self):
        wall_chain = _make_chain(
            [0, 1, 2, 3],
            [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)],
            1,
            edge_indices=[5, 14, 20],
            side_face_normals=[(0, 1, 0), (0, 1, 0), (0, 1, 0)],
        )
        top_edge_a = _make_chain(
            [1, 0],
            [(1, 0, 0), (0, 0, 0)],
            0,
            edge_indices=[5],
            side_face_normals=[(0, 0, 1)],
        )
        top_edge_b = _make_chain(
            [2, 1],
            [(2, 0, 0), (1, 0, 0)],
            0,
            edge_indices=[14],
            side_face_normals=[(0, 0, 1)],
        )
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [wall_chain])
        top = _make_wall_node(
            1, (0, 0, 1), (0, 1, 0), [top_edge_a, top_edge_b]
        )
        top.patch_type = PatchType.FLOOR

        paired_runs, boundary_edges = _collect_manual_edge_decals(
            _make_graph(wall, top), [5, 14]
        )

        assert len(paired_runs) == 1
        assert boundary_edges == []
        run = paired_runs[0]
        assert [tuple(point) for point in run.points] == [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
        ]
        assert run.segment_edge_indices == [5, 14]
        assert all(
            normal.dot(Vector((0, 1, 0))) > 0.999
            for normal in run.segment_normals_a
        )
        assert all(
            normal.dot(Vector((0, 0, 1))) > 0.999
            for normal in run.segment_normals_b
        )

    def test_selected_self_seam_pairs_two_uses_of_one_edge(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[35],
            side_face_normals=[(1, 0, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[35],
            side_face_normals=[(0, -1, 0)],
        )
        wall = _make_wall_node(0, (1, -1, 0), (0, 0, 1), [side_a, side_b])

        paired_runs, boundary_edges = _collect_manual_edge_decals(
            _make_graph(wall), [35]
        )

        assert len(paired_runs) == 1
        assert abs(paired_runs[0].segment_convexities[0]) > 0.99
        assert boundary_edges == []

    def test_selected_edge_without_chain_use_is_rejected(self):
        collection = _collect_manual_edge_decals(PatchGraph(), [404])

        assert collection.accepted_edge_indices == ()
        assert collection.corner_runs == ()
        assert collection.boundary_runs == ()
        assert [
            (item.edge_index, item.reason, item.use_count)
            for item in collection.rejected_edges
        ] == [(404, "NO_BOUNDARY_CHAIN_USE", 0)]

    def test_non_manifold_selected_edge_does_not_take_first_two_uses(self):
        nodes = []
        for patch_id, normal in enumerate(
            ((0, 0, 1), (0, 1, 0), (1, 0, 0))
        ):
            chain = _make_chain(
                [0, 1],
                [(0, 0, 0), (1, 0, 0)],
                -1,
                edge_indices=[77],
                side_face_normals=[normal],
            )
            nodes.append(
                _make_wall_node(patch_id, normal, (0, 0, 1), [chain])
            )

        collection = _collect_manual_edge_decals(_make_graph(*nodes), [77])

        assert collection.accepted_edge_indices == ()
        assert collection.corner_runs == ()
        assert collection.boundary_runs == ()
        assert len(collection.rejected_edges) == 1
        assert collection.rejected_edges[0].reason == "NON_MANIFOLD_EDGE_USE"
        assert collection.rejected_edges[0].use_count == 3

    def test_one_sided_boundary_and_missing_edge_account_exactly(self):
        chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (1, 0, 0)],
            -1,
            edge_indices=[70],
            side_face_normals=[(0, 0, 1)],
        )
        graph = _make_graph(
            _make_wall_node(0, (0, 0, 1), (0, 1, 0), [chain])
        )

        collection = _collect_manual_edge_decals(graph, [70, 99])
        plan = decals_module.compile_manual_seam_decal_plan(
            graph, DecalSettings(), (70, 99)
        )

        assert collection.accepted_edge_indices == (70,)
        assert len(collection.boundary_runs) == 1
        assert collection.rejected_edges[0].edge_index == 99
        assert plan.selected_edge_indices == (70, 99)
        assert plan.accepted_patch_voronoi_edge_indices == ()
        assert plan.accepted_legacy_edge_indices == ()
        assert plan.rejected_edge_indices == (99, 70)
        assert {
            rejection.edge_index: rejection.reason
            for rejection in plan.rejected_edges
        } == {
            70: "MISSING_SITE_FACE_PROVENANCE",
            99: "NO_BOUNDARY_CHAIN_USE",
        }
        assert plan.accounting_is_exact
        assert plan.backend_summary == (
            "Unsupported[MISSING_SITE_FACE_PROVENANCE:x1] | "
            "Failed:2e["
            "MISSING_SITE_FACE_PROVENANCE:x1,"
            "NO_BOUNDARY_CHAIN_USE:x1]"
        )

    def test_single_use_internal_seam_fails_component_visibly(self):
        points = [
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((1.0, 1.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ]
        chain = _make_chain(
            [0, 1],
            points[:2],
            -2,
            edge_indices=[170],
            side_face_normals=[(0.0, 0.0, 1.0)],
        )
        node = _make_wall_node(17, (0, 0, 1), (0, 1, 0), [chain])
        node.centroid = sum(points, Vector()) / 4.0
        node.basis_u = Vector((1.0, 0.0, 0.0))
        node.mesh_verts = points
        node.mesh_tris = [(0, 1, 2), (0, 2, 3)]

        plan = decals_module.compile_manual_seam_decal_plan(
            _make_graph(node), DecalSettings(), (170,)
        )

        assert plan.accepted_patch_voronoi_edge_indices == ()
        assert plan.accepted_legacy_edge_indices == ()
        assert plan.rejected_edge_indices == (170,)
        assert plan.rejected_edges[0].reason == "SINGLE_USE_INTERNAL_SEAM"
        assert [failure.reason for failure in plan.compile_failures] == [
            "SINGLE_USE_INTERNAL_SEAM"
        ]
        assert plan.accounting_is_exact
        assert plan.backend_summary == (
            "Unsupported[SINGLE_USE_INTERNAL_SEAM:x1] | "
            "Failed:1e[SINGLE_USE_INTERNAL_SEAM:x1]"
        )

    def test_corner_runs_preserve_surface_sides_across_chain_splits(self):
        first = _make_corner_run(0, 1, (0, 0, 0), (1, 0, 0), 5)
        second = _make_corner_run(
            1,
            2,
            (1, 0, 0),
            (2, 0.5, 0),
            14,
            normal_a=(0, 0, 1),
            normal_b=(0, 1, 0),
        )

        stitched = _stitch_corner_runs([first, second])

        assert len(stitched) == 1
        run = stitched[0]
        assert run.segment_edge_indices == [5, 14]
        assert all(normal.y > 0.99 for normal in run.segment_normals_a)
        assert all(normal.z > 0.99 for normal in run.segment_normals_b)

    def test_corner_runs_stop_at_valence_three_junction(self):
        runs = [
            _make_corner_run(0, 1, (0, 0, 0), (1, 0, 0), 5),
            _make_corner_run(1, 2, (1, 0, 0), (2, 0, 0), 14),
            _make_corner_run(1, 3, (1, 0, 0), (1, 1, 0), 35),
        ]

        stitched = _stitch_corner_runs(runs)

        assert len(stitched) == 3
        assert all(len(run.segment_edge_indices) == 1 for run in stitched)

    def test_t_junction_prepares_equal_branch_cuts_for_seam_patch(self):
        runs = _stitch_corner_runs(
            [
                _make_corner_run(
                    1,
                    2,
                    (0, 0, 0),
                    (2, 0, 0),
                    5,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
                _make_corner_run(
                    1,
                    3,
                    (0, 0, 0),
                    (-2, 0, 0),
                    14,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
                _make_corner_run(
                    1,
                    4,
                    (0, 0, 0),
                    (0, 2, 0),
                    35,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
            ]
        )
        settings = DecalSettings(width_seam=0.2, offset=0.02)

        specs, cuts = _prepare_seam_junctions(runs, settings, 0.1)

        assert set(specs) == {1}
        assert len(cuts) == 3
        assert all(abs(cut - 0.1) < 1e-6 for cut in cuts.values())
        assert (specs[1].core_pos - Vector((0, 0, 0.02))).length < 1e-6

        trimmed = _trim_run_for_junctions(runs[0], start_cut=cuts[(0, True)])
        assert (trimmed.points[0] - Vector((0.1, 0, 0))).length < 1e-6
        assert trimmed.vert_indices == runs[0].vert_indices

    def test_trihedral_offset_planes_share_one_junction_core(self):
        center = _offset_plane_junction_center(
            Vector((0, 0, 0)),
            [
                Vector((1, 0, 0)),
                Vector((0, 1, 0)),
                Vector((0, 0, 1)),
            ],
            0.02,
        )

        assert (center - Vector((0.02, 0.02, 0.02))).length < 1e-6

    def test_junction_miter_intersects_outer_branch_contours(self):
        miter = _junction_miter_position(
            Vector((0.02, 0.02, 0.02)),
            Vector((0.12, -0.08, 0.02)),
            Vector((1, 0, 0)),
            Vector((-0.08, 0.12, 0.02)),
            Vector((0, 1, 0)),
            0.1,
        )

        assert (miter - Vector((-0.08, -0.08, 0.02))).length < 1e-6

    def test_junction_miter_averages_parallel_outer_contours(self):
        miter = _junction_miter_position(
            Vector((0, 0, 0)),
            Vector((-0.1, -0.1, 0)),
            Vector((-1, 0, 0)),
            Vector((0.1, -0.1, 0)),
            Vector((1, 0, 0)),
            0.1,
        )

        assert (miter - Vector((0, -0.1, 0))).length < 1e-6

    def test_selected_self_seam_uses_both_owner_sides(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(1, 0, 0)],
        )
        wrapped_wall = _make_wall_node(
            0, (0.707, 0.707, 0), (0, 0, 1), [side_a, side_b]
        )
        graph = _make_graph(wrapped_wall)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert chain_refs == {(0, 0, 0), (0, 0, 1)}
        assert len(corner_chains) == 1
        assert boundary_chains == []

    def test_patch_pair_ignores_semantic_patch_types(self):
        owner_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[42],
            dihedral_convexity=-1.0,
        )
        neighbor_chain = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, 0)],
            0,
            edge_indices=[42],
            dihedral_convexity=-1.0,
        )
        floor = _make_wall_node(0, (0, 0, 1), (0, 1, 0), [owner_chain])
        slope = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [neighbor_chain])
        floor.patch_type = PatchType.FLOOR
        slope.patch_type = PatchType.SLOPE
        graph = _make_graph(floor, slope)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert len(corner_chains) == 1
        assert corner_chains[0][4] == -1.0
        assert boundary_chains == []

    def test_mesh_boundary_becomes_flat_corner_width_decal(self):
        border_chain = _make_chain(
            [0, 1, 2],
            [(0, 0, 0), (1, 0, 0), (2, 0, 0)],
            -1,
            edge_indices=[7, 8],
        )
        floor = _make_wall_node(0, (0, 0, 1), (0, 1, 0), [border_chain])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(floor)

        chain_refs = chain_refs_for_edge_indices(graph, [8])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert corner_chains == []
        assert len(boundary_chains) == 1
        assert len(boundary_chains[0][0]) == 3

    def test_boundary_wing_points_inside_owner_patch(self):
        wing_dir = _boundary_wing_direction(
            Vector((1, 0, 0)),
            Vector((0, 0, 1)),
        )

        assert wing_dir is not None
        assert wing_dir.y > 0.0
        assert abs(wing_dir.x) < 1e-9


class TestCornerWingDirections:
    def test_planar_wings_follow_non_right_dihedral(self):
        # Две плоскости под 60/120 градусов вокруг вертикального seam.
        # Направления крыльев должны сохранить этот угол, а не стать 90°.
        wings = _corner_wing_directions(
            Vector((0, 0, 1)),
            Vector((0, -1, 0)),
            Vector((0.8660254038, -0.5, 0)),
            dihedral_convexity=1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert abs(abs(wing_a.dot(wing_b)) - 0.5) < 1e-6
        assert abs(wing_a.dot(wing_b)) > 0.1  # явно не 90°

    def test_convex_wings_follow_patch_interiors(self):
        wings = _corner_wing_directions(
            Vector((0, 0, 1)),
            Vector((0, -1, 0)),
            Vector((1, 0, 0)),
            dihedral_convexity=1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert wing_a.x < 0.0
        assert wing_b.y > 0.0

    def test_concave_wings_are_not_inverted(self):
        wings = _corner_wing_directions(
            Vector((0, 0, -1)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 0)),
            dihedral_convexity=-1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert wing_a.x < 0.0
        assert wing_b.y > 0.0


class TestCornerOffsetJoin:
    def test_right_angle_miter_preserves_width_to_both_segments(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 0)),
            1.0,
        )

        assert not join.is_bevel
        assert (join.incoming - join.outgoing).length < 1e-9
        assert abs(join.incoming.x + 1.0) < 1e-6
        assert abs(join.incoming.y - 1.0) < 1e-6

    def test_straight_offset_remains_one_shared_point(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            0.5,
        )

        assert not join.is_bevel
        assert (join.incoming - Vector((0, 0.5, 0))).length < 1e-9

    def test_acute_turn_uses_bevel_instead_of_long_spike(self):
        next_tangent = Vector((-0.984807753, 0.173648178, 0))
        next_wing = Vector((-0.173648178, -0.984807753, 0))
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            next_tangent,
            next_wing,
            1.0,
        )

        assert join.is_bevel
        assert join.incoming.length <= 1.000001
        assert join.outgoing.length <= 1.000001

    def test_skew_surface_offsets_use_bevel(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 1)).normalized(),
            1.0,
        )

        assert join.is_bevel


class TestPolylineTangents:
    def test_straight_line(self):
        points = [Vector((0, 0, 0)), Vector((1, 0, 0)), Vector((2, 0, 0))]
        tangents = _polyline_tangents(points)
        for tangent in tangents:
            assert abs(tangent.x - 1.0) < 1e-6

    def test_corner_bisector(self):
        points = [Vector((0, 0, 0)), Vector((1, 0, 0)), Vector((1, 1, 0))]
        tangents = _polyline_tangents(points)
        mid = tangents[1]
        assert abs(mid.x - mid.y) < 1e-6

    def test_closed_wraparound_endpoints_match(self):
        # Квадрат, замкнутый дублированием первой точки: касательные на
        # обоих концах — одинаковая биссектриса последнего и первого
        # сегментов (иначе крылья ленты дают щель на стыке кольца).
        square = [
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((1, 1, 0)),
            Vector((0, 1, 0)),
            Vector((0, 0, 0)),
        ]
        tangents = _polyline_tangents(square, closed=True)
        first, last = tangents[0], tangents[-1]
        assert (first - last).length < 1e-9
        # биссектриса сегментов (0,-1,0) и (1,0,0)
        assert abs(first.x + first.y) < 1e-6 and first.x > 0


class TestDedupePolyline:
    def test_collapses_near_duplicates(self):
        points = [
            Vector((0, 0, 0)),
            Vector((0.005, 0, 0)),
            Vector((1, 0, 0)),
        ]
        result = _dedupe_polyline(points)
        assert len(result) == 2
        assert result[1].x == 1.0

    def test_empty(self):
        assert _dedupe_polyline([]) == []


class TestTrimEdgeDedup:
    def test_duplicate_edge_registered_once(self):
        # Non-manifold: одно и то же ребро в двух WALL patches — в сборку
        # путей должно попасть один раз (иначе путь дублируется обратно).
        chain_a = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        wall_a = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain_a])
        chain_b = _make_chain([1, 0], [(1, 0, 1), (0, 0, 1)], -1)
        wall_b = _make_wall_node(1, (0, -1, 0), (0, 0, 1), [chain_b])
        graph = _make_graph(wall_a, wall_b)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        # ребро (0,1) — верхняя кромка обеих стен, но ключ регистрируется
        # только первым patch; дубликат отброшен (first-wins)
        assert _ribbon_edges(top_runs) == [(0, 1)]
        assert bottom_runs == []
