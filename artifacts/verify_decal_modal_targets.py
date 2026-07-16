"""Blender smoke для A8 operator-owned drag targets.

Run:
    blender --background --factory-startup --python \
        artifacts/verify_decal_modal_targets.py
"""

from __future__ import annotations

from math import pi
from pathlib import Path
from types import MethodType, SimpleNamespace
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cftuv.decals import (  # noqa: E402
    DecalGenerationResult,
    PreviewStatus,
)
from cftuv.model import DecalSettings  # noqa: E402
from cftuv.operators import HOTSPOTUV_OT_GenerateDecals  # noqa: E402


def _bind(operator, *names):
    for name in names:
        setattr(
            operator,
            name,
            MethodType(getattr(HOTSPOTUV_OT_GenerateDecals, name), operator),
        )


def main():
    settings = DecalSettings(
        width_seam=0.15,
        corner_acute_split_angle=pi / 3.0,
        corner_apex_limit=8.0,
    )
    operator = SimpleNamespace(mode="SEAMS")
    _bind(
        operator,
        "_active_modal_drag_target",
        "_configure_modal_drag_targets",
        "_rebase_modal_drag",
        "_modal_value_text",
        "_set_modal_header",
        "_restore_modal_scene_targets",
        "modal",
    )
    operator._configure_modal_drag_targets(settings)
    operator._rebase_modal_drag("W", 100, False)
    operator._modal_state = object()
    operator._modal_created = ["Preview"]
    header = SimpleNamespace(text=None)
    operator._modal_area = SimpleNamespace(
        header_text_set=lambda text: setattr(header, "text", text)
    )
    generated = []
    operator._generate = lambda _context, _state, current, preview=False: (
        generated.append(current)
        or DecalGenerationResult(PreviewStatus.UPDATED, "Preview")
    )
    scene_settings = SimpleNamespace(
        decal_width_seam=settings.width_seam,
        decal_corner_acute_split_angle=settings.corner_acute_split_angle,
        decal_corner_miter_limit=settings.corner_apex_limit,
    )
    context = SimpleNamespace(
        scene=SimpleNamespace(hotspotuv_settings=scene_settings)
    )

    operator.modal(
        context,
        SimpleNamespace(type="MOUSEMOVE", mouse_x=120, shift=False),
    )
    width = operator._modal_current_value
    operator.modal(
        context,
        SimpleNamespace(
            type="LEFT_SHIFT",
            value="PRESS",
            mouse_x=120,
            shift=True,
        ),
    )
    assert operator._modal_current_value == width
    assert len(generated) == 1

    operator.modal(
        context,
        SimpleNamespace(type="A", value="PRESS", mouse_x=120, shift=True),
    )
    assert operator._modal_current_value == pi / 3.0
    assert header.text.startswith("Acute Split Angle: 60.0°")
    operator.modal(
        context,
        SimpleNamespace(type="MOUSEMOVE", mouse_x=122, shift=True),
    )
    angle = operator._modal_current_settings.corner_acute_split_angle
    assert angle > pi / 3.0

    operator.modal(
        context,
        SimpleNamespace(type="M", value="PRESS", mouse_x=122, shift=False),
    )
    assert operator._modal_current_value == 8.0
    assert operator._modal_current_settings.width_seam == width
    assert operator._modal_current_settings.corner_acute_split_angle == angle

    operator._restore_modal_scene_targets(scene_settings)
    assert scene_settings.decal_width_seam == settings.width_seam
    assert (
        scene_settings.decal_corner_acute_split_angle
        == settings.corner_acute_split_angle
    )
    assert scene_settings.decal_corner_miter_limit == settings.corner_apex_limit
    print(
        "[CFTUV][A8] PASS",
        f"width={width:.6f}",
        f"angle={angle * 180.0 / pi:.3f}deg",
        "apex=8.0000",
    )


main()
