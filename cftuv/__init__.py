bl_info = {
    "name": "Hotspot UV + Mesh Decals (Unified Adaptive)",
    "author": "Tech Artist & AI",
    "version": (2, 5, 25),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Hotspot UV",
    "description": "Constraint-First Trim UV: Three-layer (Form/Semantic/Topology) system for trim sheet workflows.",
    "category": "UV",
}

from .Hotspot_UV_v2_5_26 import register, unregister

__all__ = ["bl_info", "register", "unregister"]
