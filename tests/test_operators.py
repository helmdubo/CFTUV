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

from cftuv.model import DecalSettings
from cftuv.operators import (
    HOTSPOTUV_OT_GenerateDecals,
    _DECAL_SIZE_MIN,
    _decal_drag_value,
)


@pytest.mark.parametrize(
    ("mode", "property_name", "settings_field", "base_value"),
    (
        ("TOP", "decal_height_trim", "height_trim", 0.25),
        ("BOTTOM", "decal_height_trim", "height_trim", 0.25),
        ("CORNERS", "decal_width_corner", "width_corner", 0.20),
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
    operator._modal_state = object()
    operator._modal_created = ["Decal"]
    operator._modal_area = None

    generated_settings = []

    def _generate(_context, _state, settings):
        generated_settings.append(settings)
        return ["Decal"]

    operator._generate = _generate
    scene_settings = SimpleNamespace(
        decal_height_trim=0.25,
        decal_width_corner=0.20,
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
    assert _decal_drag_value(0.25, -10000) == _DECAL_SIZE_MIN

    normal_delta = _decal_drag_value(0.25, 10) - 0.25
    precise_delta = _decal_drag_value(0.25, 10, precise=True) - 0.25
    assert precise_delta == pytest.approx(normal_delta * 0.1)


@pytest.mark.parametrize("mode", ("TOP", "BOTTOM", "CORNERS"))
def test_decal_invoke_starts_horizontal_drag_for_every_mode(monkeypatch, mode):
    from cftuv import operators as operators_module

    settings = DecalSettings()
    state = (object(), object(), settings, None, False, 0, frozenset())
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

    header = SimpleNamespace(text=None)

    def _set_header(text):
        header.text = text

    context = SimpleNamespace(
        active_object=object(),
        window=object(),
        area=SimpleNamespace(header_text_set=_set_header),
        window_manager=SimpleNamespace(modal_handler_add=lambda _operator: None),
    )
    operator = HOTSPOTUV_OT_GenerateDecals()
    operator.mode = mode
    operator._generate = lambda _context, _state, settings=None: ["Decal"]

    result = operator.invoke(
        context,
        SimpleNamespace(mouse_x=321, mouse_y=654),
    )

    assert result == {"RUNNING_MODAL"}
    assert operator._modal_start_mouse == 321
    assert "Move Left/Right" in header.text
