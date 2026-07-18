from __future__ import annotations

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


def test_manual_seam_plan_routes_only_failed_topology_component_to_legacy(
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

    accepted_plan = object()
    legacy_plan = object()
    monkeypatch.setattr(decals_module, "_collect_manual_edge_decals", collect)
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
    legacy_calls = []
    monkeypatch.setattr(
        decals_module,
        "compile_seam_network_plan",
        lambda runs, _offset: legacy_calls.append(tuple(runs)) or legacy_plan,
    )

    plan = decals_module.compile_manual_seam_decal_plan(
        object(), DecalSettings(), (1, 2, 10)
    )

    assert strict_calls == [(10,)]
    assert len(legacy_calls) == 1
    assert [partition.backend for partition in plan.backend_partitions] == [
        "PATCH_VORONOI",
        "LEGACY_NETWORK",
    ]
    assert plan.backend_partitions[0].edge_indices == (10,)
    assert plan.backend_partitions[0].topology_component_count == 1
    assert plan.backend_partitions[1].edge_indices == (1, 2)
    assert plan.backend_summary == (
        "PLANAR:1c/1e | LEGACY:1c/2e | "
        "Fallback[NO_OWNER_SURFACES:x1]"
    )


def test_manual_seam_hybrid_materializes_both_backend_partitions(monkeypatch):
    patch_run = _backend_test_run((10,), (10, 11))
    legacy_run = _backend_test_run((1,), (0, 1))
    patch_partition = decals_module._ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(10,),
        topology_component_count=1,
        corner_runs=(patch_run,),
        boundary_runs=(),
        compiled_plan=object(),
    )
    legacy_partition = decals_module._ManualSeamBackendPartition(
        backend="LEGACY_NETWORK",
        edge_indices=(1,),
        topology_component_count=1,
        corner_runs=(legacy_run,),
        boundary_runs=(),
        compiled_plan=object(),
    )
    plan = decals_module.ManualSeamDecalPlan(
        corner_runs=(patch_run, legacy_run),
        boundary_runs=(),
        backend_partitions=(patch_partition, legacy_partition),
    )
    monkeypatch.setattr(
        decals_module,
        "evaluate_patch_voronoi_plan",
        lambda *_args, **_kwargs: ("patch-face",),
    )
    monkeypatch.setattr(
        decals_module,
        "evaluate_seam_network_plan",
        lambda *_args, **_kwargs: ("legacy-face",),
    )
    materialized = []
    monkeypatch.setattr(
        decals_module,
        "_materialize_network_faces",
        lambda _bm, faces, _settings, _uv_rect, **context: materialized.append(
            (tuple(faces), context)
        ),
    )

    decals_module._fill_manual_chain_decals(
        object(),
        object(),
        DecalSettings(),
        (),
        mode="SEAMS",
        selected_edge_indices=(1, 10),
        decal_plan=plan,
    )

    assert [entry[0] for entry in materialized] == [
        ("patch-face",),
        ("legacy-face",),
    ]
    assert materialized[0][1]["backend"] == "PATCH_VORONOI"
    assert materialized[0][1]["edge_indices"] == (10,)
    assert materialized[0][1]["evaluator_policy_counts"] == ()
    assert materialized[0][1]["evaluation_ms"] >= 0.0
    assert materialized[1][1]["backend"] == "LEGACY_NETWORK"
    assert materialized[1][1]["edge_indices"] == (1,)
    assert materialized[1][1]["evaluator_policy_counts"] is None
    assert materialized[1][1]["evaluation_ms"] >= 0.0


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
    hybrid = decals_module.ManualSeamDecalPlan(
        corner_runs=(),
        boundary_runs=(),
        backend_partitions=(
            patch_partition,
            decals_module._ManualSeamBackendPartition(
                backend="LEGACY_NETWORK",
                edge_indices=(12,),
                topology_component_count=1,
                corner_runs=(),
                boundary_runs=(),
                compiled_plan=object(),
            ),
        ),
        selected_edge_indices=(10, 11, 12),
    )

    assert clean.supports_live_corner_controls is True
    assert hybrid.supports_live_corner_controls is False


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
    legacy_calls = []
    monkeypatch.setattr(
        decals_module,
        "build_seam_network_faces",
        lambda *_args, **_kwargs: legacy_calls.append(True),
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

    assert legacy_calls == []


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

    evaluation = decals_module._evaluate_manual_backend_partition(
        partition,
        settings,
        width=2.5,
        preview=True,
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
    assert corner_settings.acute_split_angle == pytest.approx(0.37)
    assert corner_settings.apex_limit == pytest.approx(4.25)


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
        (True, True, decals_module.PreviewStatus.RETAINED_LAST_VALID),
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
    assert result.object_name == (
        ".CFTUV_Preview_Seams_Source"
        if expected_status == decals_module.PreviewStatus.RETAINED_LAST_VALID
        else None
    )


@pytest.mark.parametrize(
    ("preview", "has_existing", "expected_status"),
    (
        (True, True, decals_module.PreviewStatus.RETAINED_LAST_VALID),
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
        backend="LEGACY_NETWORK",
        edge_indices=(8,),
    )

    assert result == decals_module.DecalMaterializationResult(
        backend="LEGACY_NETWORK",
        edge_indices=(8,),
        source_face_count=1,
        created_face_count=1,
        created_vertex_count=3,
        policy_counts=(("SEGMENT", 1),),
    )
    assert len(bm.faces) == 1


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
        object_pointer=101,
        mesh_pointer=202,
    )

    with pytest.raises(decals_module.DecalMaterializationError):
        decals_module.generate_decal_objects(
            PatchGraph(),
            SimpleNamespace(name="Source"),
            DecalSettings(),
            "SEAMS",
            scene=object(),
            preview=preview,
            preview_state=preview_state if preview else None,
        )

    assert bm.freed
    assert published == []
    if preview:
        assert preview_state.object_pointer == 101
        assert preview_state.mesh_pointer == 202
        assert preview_state.topology_signature == ((3,),)


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
        assert plan.accepted_legacy_edge_indices == (70,)
        assert plan.rejected_edge_indices == (99,)
        assert plan.accounting_is_exact
        assert plan.backend_summary.endswith("Rejected:1e")

    def test_single_use_internal_seam_routes_component_to_legacy(self):
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
        assert plan.accepted_legacy_edge_indices == (170,)
        assert plan.rejected_edge_indices == ()
        assert [failure.reason for failure in plan.compile_failures] == [
            "SINGLE_USE_INTERNAL_SEAM"
        ]
        assert plan.accounting_is_exact

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
