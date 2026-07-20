"""B1 shared decal geometry contracts."""

import ast
import importlib.util
from pathlib import Path

import pytest
from mathutils import Vector

from cftuv import decal_voronoi
from cftuv.decal_geometry import (
    DecalGeometryFace,
    lift_offset_position,
    polygon_area2,
    segment_point_distance2,
)


def test_legacy_decal_network_module_is_removed():
    assert importlib.util.find_spec("cftuv.decal_network") is None


def test_patch_voronoi_uses_shared_geometry_contracts():
    assert decal_voronoi._NetworkFace is DecalGeometryFace
    assert decal_voronoi._lift_position is lift_offset_position
    assert decal_voronoi._polygon_area2 is polygon_area2
    assert (
        decal_voronoi._segment_point_distance2
        is segment_point_distance2
    )


def test_shared_geometry_has_no_backend_imports():
    source_path = Path(decal_voronoi.__file__).with_name("decal_geometry.py")
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }

    assert "decal_network" not in imported_modules
    assert "decal_voronoi" not in imported_modules

    voronoi_tree = ast.parse(
        Path(decal_voronoi.__file__).read_text(encoding="utf-8")
    )
    voronoi_imports = {
        node.module
        for node in ast.walk(voronoi_tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    assert "decal_network" not in voronoi_imports


def test_shared_face_and_primitives_preserve_legacy_values():
    face = DecalGeometryFace(
        surface_id=3,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("v", 0), ("v", 1), ("v", 2)],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            Vector((2.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[-1.0, 1.0, -1.0],
        v_lengths=[0.0, 0.0, 1.0],
    )

    assert face.component_kind == "SURFACE"
    assert polygon_area2(((0.0, 0.0), (2.0, 0.0), (0.0, 1.0))) == 1.0
    assert segment_point_distance2(
        (0.0, 0.0), (2.0, 0.0), (1.0, 1.0)
    ) == (1.0, 0.5)
    lifted = lift_offset_position(
        Vector((1.0, 2.0, 3.0)),
        (Vector((0.0, 0.0, 1.0)),),
        0.25,
    )
    assert tuple(lifted) == pytest.approx((1.0, 2.0, 3.25))
