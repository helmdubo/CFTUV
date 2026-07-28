"""Blender 4.3 background smoke for the exact-planar Envelope debug bridge.

Смок гоняет РАБОЧЕЕ ДЕРЕВО, а не установленную копию аддона. Оплачено
измерением: с `addon_utils.enable` по умолчанию Blender брал `cftuv` и
`cftuv_envelope` из своего каталога аддонов, там лежала копия месячной
давности, и смок падал `module 'cftuv_envelope' has no attribute
'GridSnappingLawV1'` — то есть измерял давность установки, а не состояние ветки.
Точка входа аддона при этом проверяется по-прежнему: `addon_utils.enable`
остаётся, ему лишь подставлен путь.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
for _path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(_path) in sys.path:
        sys.path.remove(str(_path))
    sys.path.insert(0, str(_path))
for _module_name in tuple(sys.modules):
    if _module_name == "cftuv" or _module_name.startswith("cftuv"):
        del sys.modules[_module_name]

import addon_utils
import bpy
import bmesh


REQUIRED_LAYERS = {
    "ENV_00_PATCH_DOMAIN",
    "ENV_01_HOLES",
    "ENV_02_BARRIERS",
    "ENV_10_PHYSICAL_CHAINS",
    "ENV_11_CHAIN_USES",
    "ENV_12_SELECTED_SOURCES",
    "ENV_13_PATCH_PAIRS",
    "ENV_14_SEAM_SELF_PAIRS",
    "ENV_20_SOURCE_SUPPORTS",
    "ENV_21_MOVING_FRONTS",
    "ENV_22_HIDDEN_SUPPORTS",
    "ENV_30_ENVELOPE_INSTANCES",
    "ENV_40_RAW_COVERAGE",
    "ENV_53_POINT_CONTACTS",
    "ENV_54_BOUNDARY_OCCURRENCES",
    "ENV_50_INTERACTION_COMPONENTS",
    "ENV_51_FRONT_READINGS",
    "ENV_52_EQUALITY_LOCI",
    "ENV_60_RESOLVED_COVERAGE",
    "ENV_61_RETAINED_REGIONS",
    "ENV_62_REMOVED_REGIONS",
    "ENV_70_EVENT_ANCHORS",
    "ENV_80_DIAGNOSTICS",
    # Слой отказа домена создаётся ОБОИМИ движками: METRIC_REJECTED бывает и на
    # LEGACY, поэтому он в обязательных, а не в наборе очереди.
    "ENV_93_QUEUE_REFUSED",
    "ENV_LABELS",
}


def _layer_name(layer):
    return getattr(layer, "name", getattr(layer, "info", ""))


def _reset_scene():
    if bpy.context.active_object is not None:
        active = bpy.context.active_object
        if active.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for text in list(bpy.data.texts):
        if (
            text.name.startswith("CFTUV_EnvelopeDebug_")
            or text.name.startswith("CFTUV_EnvelopeProfile_")
        ):
            bpy.data.texts.remove(text)


def _enter_edge_selection(obj, edge_indices):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select = edge.index in set(edge_indices)
    bmesh.update_edit_mesh(obj.data)


def _build_axis_plane():
    mesh = bpy.data.meshes.new("EnvelopeAxisPlaneMesh")
    mesh.from_pydata(
        (
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 0.0),
            (0.0, 2.0, 0.0),
        ),
        (),
        ((0, 1, 2, 3),),
    )
    mesh.update()
    obj = bpy.data.objects.new("EnvelopeAxisPlane", mesh)
    bpy.context.scene.collection.objects.link(obj)
    _enter_edge_selection(obj, range(len(mesh.edges)))
    return obj


def _build_two_patch_seam(
    *,
    nonplanar_second_patch=False,
    second_patch_offset=0.001,
):
    """Два патча через шов из одного ребра; второй при желании выведен из плоскости.

    Величина отклонения — параметр, а не литерал: после
    `SOURCE_ONLY_GRID_SNAP_V1` привязка источника кладёт в плоскость ТОЧНО всё,
    что ближе половины ячейки, поэтому «кривой патч» и «патч, отвергаемый
    бюджетом планарности» — это два разных числа, а не одно.
    """

    mesh = bpy.data.meshes.new("EnvelopeTwoPatchMesh")
    mesh.from_pydata(
        (
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (1.0, 1.0, 0.0),
            (
                2.0,
                1.0,
                float(second_patch_offset) if nonplanar_second_patch else 0.0,
            ),
        ),
        (),
        (
            (0, 1, 4, 3),
            (1, 2, 5, 4),
        ),
    )
    mesh.update()
    shared_index = None
    for edge in mesh.edges:
        if set(edge.vertices) == {1, 4}:
            edge.use_seam = True
            shared_index = edge.index
            break
    assert shared_index is not None
    obj = bpy.data.objects.new("EnvelopeTwoPatch", mesh)
    bpy.context.scene.collection.objects.link(obj)
    _enter_edge_selection(obj, (shared_index,))
    return obj


def _build_two_patch_multi_edge_seam():
    """Два патча, шов между ними разбит вершиной на ДВА коллинеарных ребра.

    Именно эта конфигурация валила билд в поле: владелец попадает мышью в одно
    ребро шва, а PhysicalChain состоит из двух. Возвращает объект и номера
    рёбер шва, чтобы тест выделял их по одному, а не угадывал индексы.
    """

    mesh = bpy.data.meshes.new("EnvelopeMultiEdgeSeamMesh")
    mesh.from_pydata(
        tuple(
            (float(column), float(row), 0.0)
            for row in range(3)
            for column in range(3)
        ),
        (),
        (
            (0, 1, 4, 3),
            (3, 4, 7, 6),
            (1, 2, 5, 4),
            (4, 5, 8, 7),
        ),
    )
    mesh.update()
    seam_pairs = ({1, 4}, {4, 7})
    seam_indices = []
    for edge in mesh.edges:
        if set(edge.vertices) in seam_pairs:
            edge.use_seam = True
            seam_indices.append(edge.index)
    assert len(seam_indices) == 2, seam_indices
    obj = bpy.data.objects.new("EnvelopeMultiEdgeSeam", mesh)
    bpy.context.scene.collection.objects.link(obj)
    _enter_edge_selection(obj, tuple(seam_indices))
    return obj, tuple(sorted(seam_indices))


def _build_axis_plane_with_unselected_tilted_patch():
    mesh = bpy.data.meshes.new("EnvelopePatchScopedFrameMesh")
    mesh.from_pydata(
        (
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 0.0),
            (0.0, 2.0, 0.0),
            (4.0, 0.0, 0.0),
            (6.0, 0.0, 0.0),
            (6.0, 1.4142135623730951, 1.4142135623730951),
            (4.0, 1.4142135623730951, 1.4142135623730951),
        ),
        (),
        (
            (0, 1, 2, 3),
            (4, 5, 6, 7),
        ),
    )
    mesh.update()
    obj = bpy.data.objects.new("EnvelopePatchScopedFrame", mesh)
    bpy.context.scene.collection.objects.link(obj)
    selected_edges = tuple(
        edge.index
        for edge in mesh.edges
        if set(edge.vertices).issubset({0, 1, 2, 3})
    )
    assert len(selected_edges) == 4
    _enter_edge_selection(obj, selected_edges)
    return obj


def _assert_debug_result(
    source_obj,
    expected_domain_count,
    *,
    expect_resolved,
):
    import cftuv_envelope as kernel

    object_name = "CFTUV_DEBUG_Envelope_" + source_obj.name
    text_name = "CFTUV_EnvelopeDebug_" + source_obj.name + ".json"
    gp_obj = bpy.data.objects.get(object_name)
    assert gp_obj is not None
    assert gp_obj.type in {"GPENCIL", "GREASEPENCIL"}
    assert {_layer_name(layer) for layer in gp_obj.data.layers} == REQUIRED_LAYERS
    assert gp_obj["kernel_version"] == kernel.__version__
    assert len(json.loads(gp_obj["patch_domain_ids"])) == expected_domain_count
    assert json.loads(gp_obj["raw_coverage_digests"])
    resolved_digests = json.loads(gp_obj["resolved_coverage_digests"])
    assert bool(resolved_digests) is expect_resolved
    sidecar = bpy.data.texts.get(text_name)
    assert sidecar is not None
    payload = json.loads(sidecar.as_string())
    assert payload["object_name"] == object_name
    assert len(payload["patch_domain_ids"]) == expected_domain_count
    assert payload["strokes"]
    profile = bpy.data.texts.get(
        "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
    )
    assert profile is not None
    return gp_obj


def _sidecar_payload(source_obj):
    text = bpy.data.texts[
        "CFTUV_EnvelopeDebug_" + source_obj.name + ".json"
    ]
    return json.loads(text.as_string())


def _kernel_debug_scene(snapshot, request):
    import cftuv_envelope as kernel

    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    domain_ids = tuple(
        sorted(
            {
                use_by_id[use_id].patch_domain_id
                for use_id in request.selected_chain_use_ids
            },
            key=lambda item: item.value,
        )
    )
    compilations = []
    raws = []
    interactions = []
    diagnostics = []
    for domain_id in domain_ids:
        compiled = kernel.compile_reference_envelopes(
            snapshot,
            request,
            domain_id,
        )
        if compiled.compilation is None:
            diagnostics.extend(
                (domain_id, item) for item in compiled.diagnostics
            )
            continue
        compilations.append(compiled.compilation)
        evaluated = kernel.evaluate_reference_raw_coverage(
            compiled.compilation,
            request.requested_alpha.value,
        )
        if evaluated.raw_coverage is None:
            diagnostics.extend(
                (domain_id, item) for item in evaluated.diagnostics
            )
            continue
        raw = evaluated.raw_coverage
        raws.append(raw)
        interaction = kernel.resolve_coverage_interactions(
            compiled.compilation,
            raw.boundary_resolved_envelopes,
            raw,
        )
        interactions.append(interaction)
        diagnostics.extend(
            (domain_id, item) for item in interaction.diagnostics
        )

    external = tuple(
        kernel.DebugDiagnosticV1(
            kernel.DebugDiagnosticId(
                f"blender-smoke:{ordinal}:{item.outcome.value}"
            ),
            domain_id,
            kernel.EnvelopeDebugStage.DIAGNOSTIC,
            kernel.DebugPrimitiveKind.DIAGNOSTIC,
            item.outcome.value,
            (
                kernel.DebugDiagnosticSeverity.ERROR
                if item.severity.value == "ERROR"
                else kernel.DebugDiagnosticSeverity.UNSUPPORTED
            ),
            item.message,
            None,
            frozenset(),
            "ENV_80_DIAGNOSTICS",
            item.outcome.value,
        )
        for ordinal, (domain_id, item) in enumerate(diagnostics)
    )
    return kernel.build_envelope_debug_scene(
        snapshot,
        request,
        tuple(compilations),
        tuple(raws),
        tuple(interactions),
        external,
    )


def _render_kernel_scene(source_obj, scene):
    from cftuv.envelope_debug_renderer import render_envelope_debug_scene

    render_envelope_debug_scene(scene, source_obj)
    return _sidecar_payload(source_obj)


def _new_display_source(name):
    mesh = bpy.data.meshes.new(name + "Mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _run_angular_profile_smoke():
    kernel_tests = (
        Path(__file__).resolve().parents[2] / "kernel" / "tests"
    )
    if str(kernel_tests) not in sys.path:
        sys.path.insert(0, str(kernel_tests))
    from reference_factories import angular_snapshot

    _reset_scene()
    source_obj = _new_display_source("EnvelopeAngularProfiles")
    observed_hidden_counts = []
    for hidden_count in (0, 1, 2):
        snapshot, request = angular_snapshot(hidden_count)
        payload = _render_kernel_scene(
            source_obj,
            _kernel_debug_scene(snapshot, request),
        )
        observed_hidden_counts.append(
            sum(
                item["layer"] == "ENV_22_HIDDEN_SUPPORTS"
                for item in payload["strokes"]
            )
        )
    assert observed_hidden_counts[0] == 0
    assert observed_hidden_counts[1] > 0
    assert observed_hidden_counts[2] > observed_hidden_counts[1]


def _run_mutual_arrival_smoke():
    kernel_tests = (
        Path(__file__).resolve().parents[2] / "kernel" / "tests"
    )
    if str(kernel_tests) not in sys.path:
        sys.path.insert(0, str(kernel_tests))
    from reference_factories import straight_snapshot

    square = (
        (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
        ),
    )
    _reset_scene()
    source_obj = _new_display_source("EnvelopeMutualArrival")
    equality_counts = []
    resolved_counts = []
    for alpha in ("4", "5", "6"):
        snapshot, request = straight_snapshot(
            faces=square,
            source_routes=(
                {
                    "name": "bottom",
                    "points": ((0.0, 0.0), (10.0, 0.0)),
                },
                {
                    "name": "top",
                    "points": ((10.0, 10.0), (0.0, 10.0)),
                },
            ),
            alpha=alpha,
            revision_name=f"blender-mutual-{alpha}",
        )
        payload = _render_kernel_scene(
            source_obj,
            _kernel_debug_scene(snapshot, request),
        )
        equality_counts.append(
            sum(
                item["layer"] == "ENV_52_EQUALITY_LOCI"
                for item in payload["strokes"]
            )
        )
        resolved_counts.append(
            len(payload["resolved_coverage_digests"])
        )
    assert equality_counts[0] == 0
    assert equality_counts[1] > 0
    assert equality_counts[2] > 0
    assert resolved_counts == [1, 1, 1]


def _run_capacity_and_unsupported_smoke():
    kernel_tests = (
        Path(__file__).resolve().parents[2] / "kernel" / "tests"
    )
    if str(kernel_tests) not in sys.path:
        sys.path.insert(0, str(kernel_tests))
    import cftuv_envelope as kernel
    from reference_factories import junction_snapshot, straight_snapshot

    hole_ring = (
        (
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 3.0),
            (4.0, 5.0),
            (4.0, 10.0),
            (0.0, 10.0),
        ),
        (
            (6.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (6.0, 10.0),
            (6.0, 5.0),
            (6.0, 3.0),
        ),
        ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
        ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
    )
    concave = (
        (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (6.0, 10.0),
            (6.0, 3.0),
            (4.0, 3.0),
            (4.0, 10.0),
            (0.0, 10.0),
        ),
    )
    _reset_scene()
    source_obj = _new_display_source("EnvelopeCapacity")
    capacity_inputs = (
        straight_snapshot(
            faces=hole_ring,
            source_routes=(
                {
                    "name": "bottom",
                    "points": (
                        (0.0, 0.0),
                        (4.0, 0.0),
                        (6.0, 0.0),
                        (10.0, 0.0),
                    ),
                },
                {
                    "name": "top",
                    "points": (
                        (10.0, 10.0),
                        (6.0, 10.0),
                        (4.0, 10.0),
                        (0.0, 10.0),
                    ),
                },
            ),
            alpha="4",
            revision_name="blender-hole-capacity",
        ),
        straight_snapshot(
            faces=concave,
            source_routes=(
                {
                    "name": "source",
                    "points": ((0.0, 0.0), (10.0, 0.0)),
                },
            ),
            alpha="4",
            revision_name="blender-concave-capacity",
        ),
    )
    for snapshot, request in capacity_inputs:
        payload = _render_kernel_scene(
            source_obj,
            _kernel_debug_scene(snapshot, request),
        )
        outcomes = {
            item["outcome"] for item in payload["diagnostics"]
        }
        assert "BARRIER_SPLIT_REQUIRED" in outcomes

    snapshot, request = junction_snapshot(kernel.JunctionRelationKind.T)
    payload = _render_kernel_scene(
        source_obj,
        _kernel_debug_scene(snapshot, request),
    )
    unsupported_outcomes = {
        item["outcome"] for item in payload["diagnostics"]
    }
    assert unsupported_outcomes, unsupported_outcomes
    assert any(
        "JUNCTION" in outcome or "UNSUPPORTED" in outcome
        for outcome in unsupported_outcomes
    ), unsupported_outcomes


def _run_axis_plane_smoke():
    _reset_scene()
    source_obj = _build_axis_plane()
    local_positions = tuple(tuple(vertex.co) for vertex in source_obj.data.vertices)
    objects_before = set(bpy.data.objects.keys())
    assert bpy.ops.hotspotuv.build_envelope_debug() == {"FINISHED"}
    # Ребейзлайн, измеренный на рабочем дереве: четырёхсторонняя встреча в
    # центре квадрата больше не остаётся недоказанной — ядро разрешает её ОДНИМ
    # событием и говорит это именем. Прежние ожидания
    # (`expect_resolved=False` и `MULTIWAY_INTERACTION_POLICY_UNPROVEN`)
    # описывали состояние до этой работы и держались только тем, что смок
    # грузил месячную установку вместо дерева.
    gp_obj = _assert_debug_result(
        source_obj,
        1,
        expect_resolved=True,
    )
    gp_object_name = gp_obj.name
    assert (
        bpy.context.scene.hotspotuv_settings.envelope_debug_outcome
        == "RESOLVED"
    )
    assert {
        item["outcome"] for item in _sidecar_payload(source_obj)["diagnostics"]
    } == {"MULTIWAY_MEET_RESOLVED_AS_ONE_EVENT"}
    assert tuple(tuple(vertex.co) for vertex in source_obj.data.vertices) == local_positions
    assert set(bpy.data.objects.keys()) - objects_before == {gp_obj.name}

    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_show_raw = False
    raw_layer = next(
        layer
        for layer in gp_obj.data.layers
        if _layer_name(layer) == "ENV_40_RAW_COVERAGE"
    )
    assert raw_layer.hide

    bm = bmesh.from_edit_mesh(source_obj.data)
    for edge in bm.edges:
        edge.select = False
    bmesh.update_edit_mesh(source_obj.data)
    assert bpy.ops.hotspotuv.build_envelope_debug() == {"CANCELLED"}
    assert bpy.data.objects.get(gp_object_name) is None
    assert bpy.data.texts.get(
        "CFTUV_EnvelopeDebug_" + source_obj.name + ".json"
    ) is None
    assert bpy.data.texts.get(
        "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
    ) is None


def _run_topology_only_smoke():
    _reset_scene()
    source_obj = _build_two_patch_seam()
    assert (
        bpy.ops.hotspotuv.build_envelope_topology_debug()
        == {"FINISHED"}
    )
    gp_obj = bpy.data.objects[
        "CFTUV_DEBUG_Envelope_" + source_obj.name
    ]
    assert gp_obj["kernel_version"] == "NOT_LOADED_TOPOLOGY_ONLY"
    assert json.loads(gp_obj["raw_coverage_digests"]) == []
    assert json.loads(gp_obj["resolved_coverage_digests"]) == []
    assert len(json.loads(gp_obj["patch_domain_ids"])) == 2
    sidecar = _sidecar_payload(source_obj)
    assert sidecar["topology_pairs"]
    assert any(
        item["kind"] == "DIRECTED_CHAIN_USE"
        for item in sidecar["strokes"]
    )
    profile = json.loads(
        bpy.data.texts[
            "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
        ].as_string()
    )
    stages = {item["stage"] for item in profile["timings"]}
    assert "TOPOLOGY_SCENE" in stages
    assert "FRAME_ADMISSION" not in stages
    assert profile["stage_summary"] == {
        "metric": 0,
        "raw": 0,
        "resolved": 0,
        "topology": 2,
        "total": 2,
    }


def _run_staged_metric_rejection_smoke():
    _reset_scene()
    # 1e-2, а не прежние 1e-3: `SOURCE_ONLY_GRID_SNAP_V1` привязывает вершины
    # источника ДО проверки планарности, и всё ближе половины ячейки ложится в
    # плоскость ТОЧНО — на тысячной отвергать стало нечего. Тот же порог держит
    # `test_a_deviation_below_half_a_cell_is_absorbed_by_the_source_snap`.
    source_obj = _build_two_patch_seam(
        nonplanar_second_patch=True,
        second_patch_offset=0.01,
    )
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    gp_obj = bpy.data.objects[
        "CFTUV_DEBUG_Envelope_" + source_obj.name
    ]
    assert len(json.loads(gp_obj["patch_domain_ids"])) == 2
    receipts = json.loads(gp_obj["stage_receipts"])
    assert len(receipts) == 2
    assert {item["stage"] for item in receipts} >= {
        "METRIC_REJECTED",
    }
    assert all(
        layer in {_layer_name(item) for item in gp_obj.data.layers}
        for layer in {
            "ENV_00_PATCH_DOMAIN",
            "ENV_10_PHYSICAL_CHAINS",
            "ENV_11_CHAIN_USES",
        }
    )
    settings = bpy.context.scene.hotspotuv_settings
    assert "Topology built" in settings.envelope_debug_status
    assert "Exact Envelope unavailable" in settings.envelope_debug_status
    assert "Metric: 1/2" in settings.envelope_debug_stage_summary
    assert settings.envelope_debug_domain_status.endswith(
        "METRIC_REJECTED"
    )


def _run_two_patch_smoke():
    _reset_scene()
    source_obj = _build_two_patch_seam()
    assert bpy.ops.hotspotuv.build_envelope_debug() == {"FINISHED"}
    _assert_debug_result(
        source_obj,
        2,
        expect_resolved=True,
    )
    assert bpy.ops.hotspotuv.clear_envelope_debug() == {"FINISHED"}
    assert bpy.data.objects.get(
        "CFTUV_DEBUG_Envelope_" + source_obj.name
    ) is None
    assert bpy.data.texts.get(
        "CFTUV_EnvelopeDebug_" + source_obj.name + ".json"
    ) is None
    assert bpy.data.texts.get(
        "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
    ) is None


def _profile_counter_values(profile, name):
    return [
        item["value"]
        for item in profile["counters"]
        if item["name"] == name
    ]


def _run_session_cache_smoke():
    _reset_scene()
    source_obj = _build_axis_plane()
    settings = bpy.context.scene.hotspotuv_settings
    profiles = []
    for alpha in (0.2, 0.3, 0.4):
        settings.envelope_debug_alpha = alpha
        assert (
            bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
            == {"FINISHED"}
        )
        profiles.append(
            json.loads(
                bpy.data.texts[
                    "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
                ].as_string()
            )
        )

    assert hasattr(
        bpy.context.window_manager,
        "_cftuv_envelope_debug_session",
    )
    final = profiles[-1]
    assert _profile_counter_values(
        final,
        "ANALYSIS_BUNDLE_BUILD_COUNT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "TOPOLOGY_EXPORT_BUILD_COUNT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "PATCH_METRIC_BUILD_COUNT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "DOMAIN_GEOMETRY_BUILD_COUNT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "ANALYSIS_BUNDLE_CACHE_HIT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "TOPOLOGY_EXPORT_CACHE_HIT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "PATCH_METRIC_CACHE_HIT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "DOMAIN_GEOMETRY_CACHE_HIT",
    ) == [1]
    assert _profile_counter_values(
        final,
        "COMPILED_ENVELOPE_CACHE_BYPASS_ALPHA_DEPENDENT",
    ) == [1]
    later_stages = {item["stage"] for item in final["timings"]}
    assert "HOST_CHAIN_COLLECTION" not in later_stages
    assert "SEAM_PARTITION_NORMALIZATION" not in later_stages
    assert "FRAME_ADMISSION" not in later_stages

    controller = (
        bpy.context.window_manager._cftuv_envelope_debug_session
    )
    counts_after_alpha = controller.build_counts
    settings.envelope_debug_show_raw = not settings.envelope_debug_show_raw
    assert controller.build_counts == counts_after_alpha

    bm = bmesh.from_edit_mesh(source_obj.data)
    bm.verts.ensure_lookup_table()
    bm.verts[0].co.x += 0.125
    bmesh.update_edit_mesh(source_obj.data)
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    vertex_profile = json.loads(
        bpy.data.texts[
            "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
        ].as_string()
    )
    assert _profile_counter_values(
        vertex_profile,
        "SESSION_CACHE_INVALIDATED",
    ) == [1]
    assert (
        controller.build_counts["ANALYSIS_BUNDLE"]
        == counts_after_alpha["ANALYSIS_BUNDLE"] + 1
    )
    assert (
        controller.build_counts["TOPOLOGY_EXPORT"]
        == counts_after_alpha["TOPOLOGY_EXPORT"] + 1
    )
    assert (
        controller.build_counts["PATCH_METRIC"]
        == counts_after_alpha["PATCH_METRIC"] + 1
    )
    assert (
        controller.build_counts["DOMAIN_GEOMETRY"]
        == counts_after_alpha["DOMAIN_GEOMETRY"] + 1
    )

    bm = bmesh.from_edit_mesh(source_obj.data)
    bm.edges.ensure_lookup_table()
    bm.edges[0].seam = not bm.edges[0].seam
    bmesh.update_edit_mesh(source_obj.data)
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    assert (
        controller.build_counts["ANALYSIS_BUNDLE"]
        == counts_after_alpha["ANALYSIS_BUNDLE"] + 2
    )
    assert (
        controller.build_counts["TOPOLOGY_EXPORT"]
        == counts_after_alpha["TOPOLOGY_EXPORT"] + 2
    )
    assert (
        controller.build_counts["PATCH_METRIC"]
        == counts_after_alpha["PATCH_METRIC"] + 2
    )
    assert (
        controller.build_counts["DOMAIN_GEOMETRY"]
        == counts_after_alpha["DOMAIN_GEOMETRY"] + 2
    )


def _run_patch_scoped_frame_smoke():
    _reset_scene()
    source_obj = _build_axis_plane_with_unselected_tilted_patch()
    assert bpy.ops.hotspotuv.build_envelope_debug() == {"FINISHED"}
    # Тот же ребейзлайн, что и на осевой плоскости: выбранный патч здесь — та же
    # четырёхсторонняя встреча, и она теперь разрешается. Утверждение этого
    # смока — про НЕВЛИЯНИЕ невыбранного наклонного патча на кадр, и оно ниже.
    _assert_debug_result(
        source_obj,
        1,
        expect_resolved=True,
    )
    assert (
        bpy.context.scene.hotspotuv_settings.envelope_debug_outcome
        != "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE"
    )


def _create_viewport_evidence(filepath):
    _reset_scene()
    source_obj = _build_two_patch_seam()
    assert bpy.ops.hotspotuv.build_envelope_debug() == {"FINISHED"}
    _assert_debug_result(
        source_obj,
        2,
        expect_resolved=True,
    )
    bpy.ops.object.mode_set(mode="OBJECT")
    source_obj.display_type = "WIRE"
    source_obj.show_wire = True
    source_obj.select_set(True)
    bpy.context.view_layer.objects.active = source_obj
    bpy.ops.wm.save_as_mainfile(filepath=filepath)
    print("ENVELOPE_DEBUG_VIEWPORT_EVIDENCE_READY", filepath)


def _main():
    addon_utils.enable("cftuv", default_set=False, persistent=False)
    assert hasattr(bpy.ops.hotspotuv, "build_envelope_debug")
    assert hasattr(bpy.ops.hotspotuv, "build_envelope_topology_debug")
    assert hasattr(
        bpy.ops.hotspotuv,
        "build_exact_reference_envelope_debug",
    )
    args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    if len(args) == 2 and args[0] == "--evidence":
        _create_viewport_evidence(args[1])
        return
    _run_topology_only_smoke()
    _run_staged_metric_rejection_smoke()
    _run_session_cache_smoke()
    _run_axis_plane_smoke()
    _run_two_patch_smoke()
    _run_patch_scoped_frame_smoke()
    _run_angular_profile_smoke()
    _run_mutual_arrival_smoke()
    _run_capacity_and_unsupported_smoke()
    print("ENVELOPE_DEBUG_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
