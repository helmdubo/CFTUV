from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import bpy
import pytest


# Минимальный Blender UI stub нужен только для импорта operators.py вне Blender.
_props = types.ModuleType("bpy.props")
for _name in (
    "BoolProperty",
    "EnumProperty",
    "FloatProperty",
    "IntProperty",
    "PointerProperty",
    "StringProperty",
):
    setattr(_props, _name, lambda **_kwargs: None)
sys.modules["bpy.props"] = _props
bpy.props = _props
bpy.types = SimpleNamespace(
    AddonPreferences=object,
    Operator=object,
    Panel=object,
    PropertyGroup=object,
    Scene=type("Scene", (), {}),
)
bpy.app = SimpleNamespace(timers=SimpleNamespace(register=lambda *_args, **_kwargs: None))

_bpy_extras = types.ModuleType("bpy_extras")
_view3d_utils = types.ModuleType("bpy_extras.view3d_utils")
_view3d_utils.location_3d_to_region_2d = lambda *_args, **_kwargs: None
_bpy_extras.view3d_utils = _view3d_utils
sys.modules["bpy_extras"] = _bpy_extras
sys.modules["bpy_extras.view3d_utils"] = _view3d_utils

from mathutils import Vector
from cftuv.decal_modal import (
    DECAL_SIZE_MIN,
    decal_drag_anchor,
    decal_drag_value,
)
from cftuv.decals import (
    DecalGenerationResult,
    DecalPreviewState,
    PreviewStatus,
)
from cftuv.model import DecalSettings
from cftuv.operators import HOTSPOTUV_OT_GenerateDecals


class _IdentityMatrix:
    def __matmul__(self, value):
        return value


def _bbox_object():
    return SimpleNamespace(
        bound_box=(
            (-1.0, -1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, -1.0),
            (1.0, 1.0, 1.0),
        ),
        matrix_world=_IdentityMatrix(),
    )


@pytest.mark.parametrize(
    ("mode", "property_name", "settings_field", "base_value"),
    (
        ("TOP", "decal_height_trim", "height_trim", 0.25),
        ("BOTTOM", "decal_height_trim", "height_trim", 0.25),
        ("CORNERS", "decal_width_corner", "width_corner", 0.20),
        ("SEAMS", "decal_width_seam", "width_seam", 0.15),
    ),
)
def test_decal_modal_drag_is_horizontal_and_has_same_sign(
    mode,
    property_name,
    settings_field,
    base_value,
):
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = mode
    operator._modal_base_settings = DecalSettings()
    operator._modal_base_value = base_value
    operator._modal_current_value = base_value
    operator._modal_start_mouse = 100
    operator._modal_property = property_name
    operator._modal_settings_field = settings_field
    operator._modal_label = "Seam Width" if mode == "SEAMS" else "Size"
    operator._modal_state = object()
    operator._modal_created = ["Decal"]
    operator._modal_area = None

    generated_settings = []

    def _generate(_context, _state, settings, preview=False):
        generated_settings.append(settings)
        return ["Decal"]

    operator._generate = _generate
    scene_settings = SimpleNamespace(
        decal_height_trim=0.25,
        decal_width_corner=0.20,
        decal_width_seam=0.15,
    )
    context = SimpleNamespace(
        scene=SimpleNamespace(hotspotuv_settings=scene_settings),
    )

    result = operator.modal(
        context,
        SimpleNamespace(
            type="MOUSEMOVE",
            mouse_x=120,
            mouse_y=-10000,
            shift=False,
        ),
    )
    assert result == {"RUNNING_MODAL"}
    assert getattr(generated_settings[-1], settings_field) > base_value

    operator.modal(
        context,
        SimpleNamespace(
            type="MOUSEMOVE",
            mouse_x=80,
            mouse_y=10000,
            shift=False,
        ),
    )
    assert getattr(generated_settings[-1], settings_field) < base_value


def test_decal_drag_clamps_to_positive_minimum_and_shift_is_precise():
    assert decal_drag_value(0.25, -10000) == DECAL_SIZE_MIN

    normal_delta = decal_drag_value(0.25, 10) - 0.25
    precise_delta = decal_drag_value(0.25, 10, precise=True) - 0.25
    assert precise_delta == pytest.approx(normal_delta * 0.1)


def test_decal_modal_ignores_vertical_only_mousemove():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "TOP"
    operator._modal_base_settings = DecalSettings()
    operator._modal_base_value = 0.25
    operator._modal_current_value = 0.25
    operator._modal_start_mouse = 500
    operator._modal_property = "decal_height_trim"
    operator._modal_settings_field = "height_trim"
    operator._modal_label = "Trim Height"
    operator._modal_state = object()
    operator._modal_created = ["Decal"]
    operator._modal_area = None

    generated_settings = []
    operator._generate = (
        lambda _context, _state, settings, preview=False: generated_settings.append(
            settings
        )
    )
    context = SimpleNamespace(
        scene=SimpleNamespace(
            hotspotuv_settings=SimpleNamespace(decal_height_trim=0.25)
        )
    )

    result = operator.modal(
        context,
        SimpleNamespace(
            type="MOUSEMOVE",
            mouse_x=500,
            mouse_y=-10000,
            shift=False,
        ),
    )

    assert result == {"RUNNING_MODAL"}
    assert generated_settings == []
    assert operator._modal_current_value == 0.25


def test_decal_modal_keeps_last_valid_preview_after_runtime_error():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_base_settings = DecalSettings(width_seam=0.15)
    operator._modal_base_value = 0.15
    operator._modal_current_value = 0.15
    operator._modal_current_settings = None
    operator._modal_start_mouse = 100
    operator._modal_property = "decal_width_seam"
    operator._modal_settings_field = "width_seam"
    operator._modal_label = "Seam Width"
    operator._modal_state = object()
    operator._modal_created = ["LastValidDecal"]
    operator._modal_area = None
    operator._generate = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        RuntimeError("invalid crop")
    )
    scene_settings = SimpleNamespace(decal_width_seam=0.15)
    context = SimpleNamespace(
        scene=SimpleNamespace(hotspotuv_settings=scene_settings),
    )

    result = operator.modal(
        context,
        SimpleNamespace(
            type="MOUSEMOVE",
            mouse_x=120,
            mouse_y=0,
            shift=False,
        ),
    )

    assert result == {"RUNNING_MODAL"}
    assert operator._modal_created == ["LastValidDecal"]
    assert operator._modal_current_value == 0.15
    assert operator._modal_current_settings is None
    assert operator._modal_last_preview_error == "invalid crop"
    assert scene_settings.decal_width_seam == 0.15


def test_modal_invalid_status_series_preserves_last_valid_and_confirm_uses_it():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    base_settings = DecalSettings(width_seam=0.15)
    operator._modal_base_settings = base_settings
    operator._modal_base_value = 0.15
    operator._modal_current_value = 0.15
    operator._modal_current_settings = base_settings
    operator._modal_last_valid_settings = base_settings
    operator._modal_start_mouse = 100
    operator._modal_property = "decal_width_seam"
    operator._modal_settings_field = "width_seam"
    operator._modal_label = "Seam Width"
    operator._modal_state = object()
    operator._modal_created = ["BaseDecal"]
    header = SimpleNamespace(text=None)
    operator._modal_area = SimpleNamespace(
        header_text_set=lambda text: setattr(header, "text", text)
    )
    operator.report = lambda *_args: None
    operator._report_created = lambda *_args, **_kwargs: None
    preview_results = iter(
        (
            DecalGenerationResult(
                PreviewStatus.UPDATED,
                "ValidDecal",
                topology_changed=True,
            ),
            DecalGenerationResult(
                PreviewStatus.RETAINED_LAST_VALID,
                "ValidDecal",
                reason="invalid crop",
            ),
            DecalGenerationResult(
                PreviewStatus.EMPTY,
                None,
                reason="no faces",
            ),
            DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason="backend exploded",
            ),
        )
    )
    confirm_calls = []

    def generate(_context, _state, settings, preview=False):
        if preview:
            return next(preview_results)
        confirm_calls.append(settings)
        return DecalGenerationResult(PreviewStatus.UPDATED, "FinalDecal")

    operator._generate = generate
    scene_settings = SimpleNamespace(decal_width_seam=0.15)
    context = SimpleNamespace(
        scene=SimpleNamespace(hotspotuv_settings=scene_settings)
    )

    for mouse_x in (120, 130, 140, 150):
        assert operator.modal(
            context,
            SimpleNamespace(
                type="MOUSEMOVE",
                mouse_x=mouse_x,
                mouse_y=0,
                shift=False,
            ),
        ) == {"RUNNING_MODAL"}

    valid_value = decal_drag_value(0.15, 20)
    assert operator._modal_current_value == pytest.approx(valid_value)
    assert operator._modal_current_settings.width_seam == pytest.approx(
        valid_value
    )
    assert operator._modal_last_valid_settings.width_seam == pytest.approx(
        valid_value
    )
    assert scene_settings.decal_width_seam == pytest.approx(valid_value)
    assert operator._modal_created == ["ValidDecal"]
    assert operator._modal_last_preview_error == "backend exploded"
    assert "ERROR: backend exploded" in header.text

    assert operator.modal(
        context,
        SimpleNamespace(type="LEFTMOUSE", value="PRESS"),
    ) == {"FINISHED"}
    assert len(confirm_calls) == 1
    assert confirm_calls[0].width_seam == pytest.approx(valid_value)
    assert operator._modal_created == ["FinalDecal"]


def test_modal_confirm_without_valid_settings_is_explicit_error():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_label = "Seam Width"
    operator._modal_current_value = 0.15
    operator._modal_current_settings = None
    operator._modal_created = []
    operator._modal_state = object()
    operator._modal_area = None
    reports = []
    operator.report = lambda level, message: reports.append((level, message))

    result = operator.modal(
        SimpleNamespace(),
        SimpleNamespace(type="LEFTMOUSE", value="PRESS"),
    )

    assert result == {"CANCELLED"}
    assert reports == [
        (
            {"ERROR"},
            "Final decal rebuild failed: no valid preview settings",
        )
    ]


def test_decal_drag_anchor_projects_bbox_center_and_clamps_to_viewport(monkeypatch):
    from cftuv import decal_modal

    region = SimpleNamespace(type="WINDOW", x=100, y=50, width=800, height=600)
    context = SimpleNamespace(
        area=SimpleNamespace(regions=[region]),
        space_data=SimpleNamespace(region_3d=object()),
    )
    monkeypatch.setattr(
        decal_modal,
        "location_3d_to_region_2d",
        lambda _region, _region_3d, _world_center: Vector((1.0, 1.0)),
    )

    assert decal_drag_anchor(context, _bbox_object(), (7, 9)) == (124, 74)


def test_decal_drag_anchor_falls_back_to_viewport_center_when_offscreen(monkeypatch):
    from cftuv import decal_modal

    region = SimpleNamespace(type="WINDOW", x=100, y=50, width=800, height=600)
    context = SimpleNamespace(
        area=SimpleNamespace(regions=[region]),
        space_data=SimpleNamespace(region_3d=object()),
    )
    monkeypatch.setattr(
        decal_modal,
        "location_3d_to_region_2d",
        lambda _region, _region_3d, _world_center: Vector((-10.0, 300.0)),
    )

    assert decal_drag_anchor(context, _bbox_object(), (7, 9)) == (500, 350)


@pytest.mark.parametrize("mode", ("TOP", "BOTTOM", "CORNERS", "SEAMS"))
def test_decal_invoke_starts_horizontal_drag_for_every_mode(monkeypatch, mode):
    from cftuv import operators as operators_module
    from cftuv import decal_modal

    settings = DecalSettings()
    source_obj = _bbox_object()
    state = (source_obj, object(), settings, None, False, 0, frozenset())
    monkeypatch.setattr(
        operators_module,
        "_prepare_decal_generation",
        lambda _context: state,
    )
    monkeypatch.setattr(
        operators_module,
        "_is_edge_select_mode",
        lambda _context, _obj: False,
    )
    monkeypatch.setattr(
        decal_modal,
        "location_3d_to_region_2d",
        lambda _region, _region_3d, _world_center: Vector((400.0, 300.0)),
    )

    header = SimpleNamespace(text=None)
    cursor_warps = []
    region = SimpleNamespace(type="WINDOW", x=100, y=50, width=800, height=600)

    def _set_header(text):
        header.text = text

    context = SimpleNamespace(
        active_object=source_obj,
        window=SimpleNamespace(
            cursor_warp=lambda x, y: cursor_warps.append((x, y)),
        ),
        area=SimpleNamespace(header_text_set=_set_header, regions=[region]),
        space_data=SimpleNamespace(region_3d=object()),
        window_manager=SimpleNamespace(modal_handler_add=lambda _operator: None),
    )
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = mode
    operator._generate = (
        lambda _context, _state, settings=None, preview=False: ["Decal"]
    )

    result = operator.invoke(
        context,
        SimpleNamespace(mouse_x=321, mouse_y=654),
    )

    assert result == {"RUNNING_MODAL"}
    assert cursor_warps == [(500, 654)]
    assert operator._modal_start_mouse == 500
    assert "Move Left/Right" in header.text
    if mode == "SEAMS":
        assert operator._modal_property == "decal_width_seam"
        assert operator._modal_settings_field == "width_seam"
        assert header.text.startswith("Seam Width: 0.1500")


def test_manual_edge_seams_enters_interactive_width_modal(monkeypatch):
    from cftuv import operators as operators_module

    settings = DecalSettings()
    source_obj = _bbox_object()
    selected_edges = (5, 14, 35)
    state = (
        source_obj,
        object(),
        settings,
        ((0, 0, 0),),
        True,
        len(selected_edges),
        selected_edges,
    )
    monkeypatch.setattr(
        operators_module,
        "_prepare_decal_generation",
        lambda _context: state,
    )
    monkeypatch.setattr(
        operators_module,
        "_is_edge_select_mode",
        lambda _context, _obj: True,
    )
    modal_plan = object()
    compile_calls = []
    monkeypatch.setattr(
        operators_module,
        "compile_manual_seam_decal_plan",
        lambda graph, plan_settings, edges: (
            compile_calls.append((graph, plan_settings, edges)) or modal_plan
        ),
    )

    cursor_warps = []
    context = SimpleNamespace(
        active_object=source_obj,
        window=SimpleNamespace(
            cursor_warp=lambda x, y: cursor_warps.append((x, y)),
        ),
        area=SimpleNamespace(header_text_set=lambda _text: None, regions=[]),
        space_data=SimpleNamespace(region_3d=object()),
        window_manager=SimpleNamespace(modal_handler_add=lambda _operator: None),
    )
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    generated_states = []
    operator._generate = lambda _context, generation_state, settings=None, preview=False: (
        generated_states.append((generation_state, settings)) or ["Decal"]
    )

    result = operator.invoke(
        context,
        SimpleNamespace(mouse_x=321, mouse_y=654),
    )

    assert result == {"RUNNING_MODAL"}
    assert operator._modal_state is state
    assert operator._modal_property == "decal_width_seam"
    assert operator._modal_base_value == 0.15
    assert generated_states[0][0][-1] == selected_edges
    assert operator._modal_decal_plan is modal_plan
    assert isinstance(operator._modal_preview_state, DecalPreviewState)
    assert compile_calls == [(state[1], settings, selected_edges)]


def test_decal_generate_forwards_modal_plan(monkeypatch):
    from cftuv import operators as operators_module

    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_decal_plan = object()
    operator._modal_preview_state = DecalPreviewState()
    settings = DecalSettings()
    state = (
        object(),
        object(),
        settings,
        ((0, 0, 0),),
        True,
        2,
        (5, 14),
    )
    calls = []
    monkeypatch.setattr(
        operators_module,
        "generate_decal_result",
        lambda *args, **kwargs: calls.append((args, kwargs))
        or operators_module.DecalGenerationResult(
            operators_module.PreviewStatus.UPDATED,
            "Decal",
        ),
    )

    generation = operator._generate(
        SimpleNamespace(scene=object()), state, preview=True
    )

    assert generation.status == operators_module.PreviewStatus.UPDATED
    assert generation.object_name == "Decal"
    assert calls[0][1]["decal_plan"] is operator._modal_decal_plan
    assert calls[0][1]["preview_state"] is operator._modal_preview_state
    assert calls[0][1]["preview"] is True


def test_seam_execute_compiles_and_forwards_same_plan_as_modal(monkeypatch):
    from cftuv import operators as operators_module

    settings = DecalSettings()
    selected_edges = (5, 14)
    state = (
        object(),
        object(),
        settings,
        ((0, 0, 0),),
        True,
        2,
        selected_edges,
    )
    modal_plan = object()
    monkeypatch.setattr(
        operators_module,
        "_prepare_decal_generation",
        lambda _context: state,
    )
    compile_calls = []
    monkeypatch.setattr(
        operators_module,
        "compile_manual_seam_decal_plan",
        lambda graph, plan_settings, edges: (
            compile_calls.append((graph, plan_settings, edges)) or modal_plan
        ),
    )

    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    generated_plans = []
    operator._generate = lambda _context, _state, settings=None, preview=False: (
        generated_plans.append(operator._modal_decal_plan) or ["Decal"]
    )
    operator._report_created = lambda *_args, **_kwargs: None

    result = operator.execute(SimpleNamespace())

    assert result == {"FINISHED"}
    assert compile_calls == [(state[1], settings, selected_edges)]
    assert generated_plans == [modal_plan]


def test_seam_modal_cancel_restores_scene_width_and_base_geometry():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_base_settings = DecalSettings(width_seam=0.15)
    operator._modal_base_value = 0.15
    operator._modal_current_value = 0.40
    operator._modal_property = "decal_width_seam"
    operator._modal_settings_field = "width_seam"
    operator._modal_label = "Seam Width"
    operator._modal_state = object()
    operator._modal_created = ["Decal"]
    operator._modal_area = None

    generated_settings = []
    operator._generate = lambda _context, _state, settings, preview=False: (
        generated_settings.append(settings) or ["Decal"]
    )
    scene_settings = SimpleNamespace(decal_width_seam=0.40)
    context = SimpleNamespace(
        scene=SimpleNamespace(hotspotuv_settings=scene_settings),
    )

    result = operator.modal(
        context,
        SimpleNamespace(type="ESC", value="PRESS"),
    )

    assert result == {"CANCELLED"}
    assert scene_settings.decal_width_seam == 0.15
    assert generated_settings == [operator._modal_base_settings]


def test_seam_modal_confirm_reports_seam_width():
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_label = "Seam Width"
    operator._modal_current_value = 0.30
    operator._modal_created = ["Decal"]
    operator._modal_state = object()
    operator._modal_area = None
    operator._modal_last_valid_settings = DecalSettings(width_seam=0.30)
    operator._generate = (
        lambda _context, _state, _settings, preview=False: ["Decal"]
    )

    reports = []
    operator._report_created = lambda created, state, suffix="": reports.append(
        (created, state, suffix)
    )

    result = operator.modal(
        SimpleNamespace(),
        SimpleNamespace(type="LEFTMOUSE", value="PRESS"),
    )

    assert result == {"FINISHED"}
    assert reports == [(["Decal"], operator._modal_state, "Seam Width:0.3000")]


def test_decal_created_report_exposes_hybrid_backend_routing():
    reports = []
    operator = SimpleNamespace(
        _modal_decal_plan=SimpleNamespace(
            backend_summary="Patch Voronoi:22c/259e | Legacy:1c/199e"
        ),
        report=lambda level, message: reports.append((level, message)),
    )
    state = (
        object(),
        object(),
        DecalSettings(),
        ((0, 0, 0),),
        True,
        458,
        tuple(range(458)),
    )

    HOTSPOTUV_OT_GenerateDecals._report_created(
        operator, ["Decal_Seams_walls.010"], state
    )

    assert reports == [
        (
            {"INFO"},
            "Created: Decal_Seams_walls.010 | chains:1 edges:458 | "
            "Patch Voronoi:22c/259e | Legacy:1c/199e",
        )
    ]


def test_seam_modal_confirm_rebuilds_last_drag_at_full_precision():
    # Во время drag декаль строится в preview-режиме; на подтверждении
    # LMB/Enter последний размер перегенерируется с preview=False.
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = "SEAMS"
    operator._modal_label = "Seam Width"
    operator._modal_current_value = 0.30
    operator._modal_created = ["PreviewDecal"]
    operator._modal_state = object()
    operator._modal_area = None
    final_settings = DecalSettings(width_seam=0.30)
    operator._modal_current_settings = final_settings

    calls = []
    operator._generate = lambda _context, _state, settings, preview=False: (
        calls.append((settings, preview)) or ["FinalDecal"]
    )
    reports = []
    operator._report_created = lambda created, state, suffix="": reports.append(
        created
    )

    result = operator.modal(
        SimpleNamespace(),
        SimpleNamespace(type="LEFTMOUSE", value="PRESS"),
    )

    assert result == {"FINISHED"}
    # Ровно одна финальная перегенерация в полной точности.
    assert calls == [(final_settings, False)]
    # Рапортуется полностью пересобранная геометрия, а не preview.
    assert reports == [["FinalDecal"]]
