bl_info = {
    "name": "CFTUV — Constraint-First Trim UV",
    "author": "Tech Artist & AI",
    "version": (2, 6, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Hotspot UV",
    "description": "Chain-first strongest-frontier UV system for trim sheet workflows.",
    "category": "UV",
}

from .operators import register, unregister

__all__ = ["bl_info", "register", "unregister"]
